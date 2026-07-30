//! SDO server — services requests against an object dictionary.
//!
//! [`SdoServer`] is a sans-I/O state machine: feed it a decoded request frame
//! with [`SdoServer::handle`] and it reads or writes the supplied
//! [`ObjectDictionary`], returning the response frame to transmit (or `None`
//! for a client abort, which needs no reply). It handles expedited transfers
//! in a single exchange and drives segmented transfers — needed for values of
//! five to eight bytes, such as `UNSIGNED64` — across multiple frames, holding
//! the per-transfer state between calls.

use heapless::Vec;

use super::{
    decode_data_segment, decode_download_initiate_segmented, encode_abort, encode_data_segment,
    encode_download_response, encode_download_segment_response, encode_upload_expedited_response,
    encode_upload_initiate_segmented_response, request_cob_id, response_cob_id, SdoAbortCode,
    SdoPayload, CCS_DOWNLOAD_INITIATE, CCS_DOWNLOAD_SEGMENT, CCS_UPLOAD_INITIATE,
    CCS_UPLOAD_SEGMENT, CS_ABORT, CS_MASK, EXPEDITED, SEGMENT_DATA_MAX, SIZE_INDICATED, TOGGLE,
};
use crate::datatypes::{DataType, Value};
use crate::object_dictionary::{Address, ObjectDictionary};
use crate::types::NodeId;
use crate::Error;

/// An in-progress segmented transfer held between frames.
///
/// All CANopen numeric values are at most eight bytes, so the buffers are
/// fixed at eight; variable-length (string/`DOMAIN`) transfers will need a
/// larger buffer when those types are modelled.
#[derive(Debug)]
enum Transfer {
    /// The client is reading a value larger than four bytes from us.
    Upload {
        addr: Address,
        data: [u8; 8],
        len: usize,
        pos: usize,
        toggle: bool,
    },
    /// The client is writing a value larger than four bytes to us.
    Download {
        addr: Address,
        data_type: DataType,
        buf: Vec<u8, 8>,
        declared: usize,
        toggle: bool,
    },
}

/// An SDO server bound to a node id, servicing requests against an object
/// dictionary.
#[derive(Debug)]
pub struct SdoServer {
    node: NodeId,
    transfer: Option<Transfer>,
}

impl SdoServer {
    /// Create a server for `node`.
    pub const fn new(node: NodeId) -> Self {
        Self {
            node,
            transfer: None,
        }
    }

    /// The COB-ID this server receives requests on (`0x600 + node`).
    pub fn request_cob_id(&self) -> u16 {
        request_cob_id(self.node)
    }

    /// The COB-ID this server sends responses on (`0x580 + node`).
    pub fn response_cob_id(&self) -> u16 {
        response_cob_id(self.node)
    }

    /// Whether a segmented transfer is currently in progress.
    pub fn is_busy(&self) -> bool {
        self.transfer.is_some()
    }

    /// Handle an SDO request against `od`, returning the response to transmit.
    ///
    /// Returns `None` only for a client abort, which is unconfirmed. Any
    /// protocol or access error produces an SDO abort response frame.
    pub fn handle<const N: usize>(
        &mut self,
        od: &mut ObjectDictionary<N>,
        req: &SdoPayload,
    ) -> Option<SdoPayload> {
        match req[0] & CS_MASK {
            CCS_DOWNLOAD_INITIATE => self.on_download_initiate(od, req),
            CCS_UPLOAD_INITIATE => self.on_upload_initiate(od, req),
            CCS_DOWNLOAD_SEGMENT => self.on_download_segment(od, req),
            CCS_UPLOAD_SEGMENT => self.on_upload_segment(req),
            CS_ABORT => {
                self.transfer = None;
                None
            }
            _ => abort(req_address(req), SdoAbortCode::CommandInvalid),
        }
    }

    fn on_download_initiate<const N: usize>(
        &mut self,
        od: &mut ObjectDictionary<N>,
        req: &SdoPayload,
    ) -> Option<SdoPayload> {
        let addr = req_address(req);
        let (data_type, writable) = match od.entry(addr) {
            Some(e) => (e.value.data_type(), e.access.is_writable()),
            None => return abort(addr, SdoAbortCode::ObjectDoesNotExist),
        };
        if !writable {
            return abort(addr, SdoAbortCode::WriteOfReadOnly);
        }

        if req[0] & EXPEDITED != 0 {
            // Expedited write: 1..=4 data bytes inline.
            let len = if req[0] & SIZE_INDICATED != 0 {
                4 - ((req[0] >> 2) & 0x03) as usize
            } else {
                data_type.size()
            };
            if len != data_type.size() {
                return abort(addr, SdoAbortCode::DataTypeMismatchLengthHigh);
            }
            let value = match Value::decode_le(data_type, &req[4..4 + len]) {
                Ok(v) => v,
                Err(_) => return abort(addr, SdoAbortCode::General),
            };
            write_value(od, addr, value)
        } else {
            // Segmented write: declare the size now, receive segments later.
            let (_, size) = match decode_download_initiate_segmented(req) {
                Ok(x) => x,
                Err(_) => return abort(addr, SdoAbortCode::CommandInvalid),
            };
            if size as usize != data_type.size() {
                return abort(addr, SdoAbortCode::DataTypeMismatchLengthHigh);
            }
            self.transfer = Some(Transfer::Download {
                addr,
                data_type,
                buf: Vec::new(),
                declared: size as usize,
                toggle: false,
            });
            Some(encode_download_response(addr))
        }
    }

    fn on_upload_initiate<const N: usize>(
        &mut self,
        od: &mut ObjectDictionary<N>,
        req: &SdoPayload,
    ) -> Option<SdoPayload> {
        let addr = req_address(req);
        let value = match od.read(addr) {
            Ok(v) => v,
            Err(Error::ObjectNotFound) => return abort(addr, SdoAbortCode::ObjectDoesNotExist),
            Err(Error::WriteOnly) => return abort(addr, SdoAbortCode::ReadOfWriteOnly),
            Err(_) => return abort(addr, SdoAbortCode::General),
        };
        let size = value.size();
        if size <= 4 {
            Some(encode_upload_expedited_response(addr, &value).expect("size <= 4"))
        } else {
            let mut data = [0u8; 8];
            value.encode_le(&mut data[..size]).expect("size <= 8");
            self.transfer = Some(Transfer::Upload {
                addr,
                data,
                len: size,
                pos: 0,
                toggle: false,
            });
            Some(encode_upload_initiate_segmented_response(addr, size as u32))
        }
    }

    fn on_upload_segment(&mut self, req: &SdoPayload) -> Option<SdoPayload> {
        let want_toggle = req[0] & TOGGLE != 0;
        match self.transfer.take() {
            Some(Transfer::Upload {
                addr,
                data,
                len,
                pos,
                toggle,
            }) => {
                if want_toggle != toggle {
                    return abort(addr, SdoAbortCode::ToggleBitNotAlternated);
                }
                let remaining = len - pos;
                let n = remaining.min(SEGMENT_DATA_MAX);
                let last = remaining <= SEGMENT_DATA_MAX;
                let seg = encode_data_segment(&data[pos..pos + n], toggle, last).expect("1..=7");
                if !last {
                    self.transfer = Some(Transfer::Upload {
                        addr,
                        data,
                        len,
                        pos: pos + n,
                        toggle: !toggle,
                    });
                }
                Some(seg)
            }
            _ => abort(Address::new(0, 0), SdoAbortCode::CommandInvalid),
        }
    }

    fn on_download_segment<const N: usize>(
        &mut self,
        od: &mut ObjectDictionary<N>,
        req: &SdoPayload,
    ) -> Option<SdoPayload> {
        let seg = match decode_data_segment(req) {
            Ok(s) => s,
            Err(_) => return abort(Address::new(0, 0), SdoAbortCode::CommandInvalid),
        };
        match self.transfer.take() {
            Some(Transfer::Download {
                addr,
                data_type,
                mut buf,
                declared,
                toggle,
            }) => {
                if seg.toggle != toggle {
                    return abort(addr, SdoAbortCode::ToggleBitNotAlternated);
                }
                if buf.extend_from_slice(seg.data).is_err() {
                    return abort(addr, SdoAbortCode::DataTypeMismatchLengthHigh);
                }
                let ack = encode_download_segment_response(seg.toggle);
                if seg.last {
                    if buf.len() != declared {
                        return abort(addr, SdoAbortCode::DataTypeMismatchLengthLow);
                    }
                    let value = match Value::decode_le(data_type, &buf) {
                        Ok(v) => v,
                        Err(_) => return abort(addr, SdoAbortCode::General),
                    };
                    // The final segment is acknowledged with a segment
                    // response, not a download-initiate response.
                    match od.write(addr, value) {
                        Ok(()) => Some(ack),
                        Err(Error::ReadOnly) => abort(addr, SdoAbortCode::WriteOfReadOnly),
                        Err(_) => abort(addr, SdoAbortCode::General),
                    }
                } else {
                    self.transfer = Some(Transfer::Download {
                        addr,
                        data_type,
                        buf,
                        declared,
                        toggle: !toggle,
                    });
                    Some(ack)
                }
            }
            _ => abort(Address::new(0, 0), SdoAbortCode::CommandInvalid),
        }
    }
}

/// Write `value` to `od`, mapping OD errors to SDO abort responses.
fn write_value<const N: usize>(
    od: &mut ObjectDictionary<N>,
    addr: Address,
    value: Value,
) -> Option<SdoPayload> {
    match od.write(addr, value) {
        Ok(()) => Some(encode_download_response(addr)),
        Err(Error::ReadOnly) => abort(addr, SdoAbortCode::WriteOfReadOnly),
        Err(Error::TypeMismatch) => abort(addr, SdoAbortCode::DataTypeMismatchLengthHigh),
        Err(_) => abort(addr, SdoAbortCode::General),
    }
}

fn req_address(req: &SdoPayload) -> Address {
    Address::new(u16::from_le_bytes([req[1], req[2]]), req[3])
}

fn abort(addr: Address, code: SdoAbortCode) -> Option<SdoPayload> {
    Some(encode_abort(addr, code))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::object_dictionary::Entry;

    fn server() -> SdoServer {
        SdoServer::new(NodeId::new(1).unwrap())
    }

    fn od() -> ObjectDictionary<8> {
        let mut od = ObjectDictionary::new();
        od.insert(
            Address::new(0x1000, 0),
            Entry::constant(Value::Unsigned32(0x0000_0192)),
        )
        .unwrap();
        od.insert(Address::new(0x1017, 0), Entry::rw(Value::Unsigned16(1000)))
            .unwrap();
        od.insert(Address::new(0x2000, 0), Entry::rw(Value::Unsigned64(0)))
            .unwrap();
        od
    }

    #[test]
    fn cob_ids_track_node() {
        let s = SdoServer::new(NodeId::new(5).unwrap());
        assert_eq!(s.request_cob_id(), 0x605);
        assert_eq!(s.response_cob_id(), 0x585);
    }

    #[test]
    fn expedited_read_returns_value() {
        let mut od = od();
        let req = super::super::encode_upload_request(Address::new(0x1000, 0));
        let resp = server().handle(&mut od, &req).unwrap();
        let (_, value) =
            super::super::decode_upload_expedited_response(&resp, DataType::Unsigned32).unwrap();
        assert_eq!(value, Value::Unsigned32(0x192));
    }

    #[test]
    fn expedited_write_updates_od() {
        let mut od = od();
        let req = super::super::encode_download_expedited(
            Address::new(0x1017, 0),
            &Value::Unsigned16(1234),
        )
        .unwrap();
        let resp = server().handle(&mut od, &req).unwrap();
        assert!(super::super::decode_download_response(&resp).is_ok());
        assert_eq!(
            od.read(Address::new(0x1017, 0)).unwrap(),
            Value::Unsigned16(1234)
        );
    }

    #[test]
    fn read_missing_object_aborts() {
        let mut od = od();
        let req = super::super::encode_upload_request(Address::new(0x9999, 0));
        let resp = server().handle(&mut od, &req).unwrap();
        let (_, code) = super::super::decode_abort(&resp).unwrap();
        assert_eq!(code, 0x0602_0000);
    }

    #[test]
    fn write_read_only_aborts() {
        let mut od = od();
        let req =
            super::super::encode_download_expedited(Address::new(0x1000, 0), &Value::Unsigned32(1))
                .unwrap();
        let resp = server().handle(&mut od, &req).unwrap();
        let (_, code) = super::super::decode_abort(&resp).unwrap();
        assert_eq!(code, 0x0601_0002); // write of a read-only object
    }

    #[test]
    fn client_abort_clears_state_without_reply() {
        let mut od = od();
        let mut s = server();
        // Start a segmented download so the server is busy.
        let init = super::super::encode_download_initiate_segmented(Address::new(0x2000, 0), 8);
        s.handle(&mut od, &init).unwrap();
        assert!(s.is_busy());
        let abort_frame =
            super::super::encode_abort(Address::new(0x2000, 0), SdoAbortCode::General);
        assert!(s.handle(&mut od, &abort_frame).is_none());
        assert!(!s.is_busy());
    }
}
