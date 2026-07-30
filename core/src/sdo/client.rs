//! SDO client — drives read (upload) and write (download) transactions.
//!
//! [`SdoClient`] is a sans-I/O state machine. Start a transaction with
//! [`SdoClient::read`] or [`SdoClient::write`] — each returns the first request
//! frame to transmit — then feed every server response to
//! [`SdoClient::on_response`], which returns the next [`SdoEvent`]: another
//! frame to send, the completed result, or an abort. The client picks
//! expedited or segmented transfer automatically from the value size, so
//! callers need not care which is used.
//!
//! ```no_run
//! # use canopen_rs::sdo::{SdoClient, SdoEvent};
//! # use canopen_rs::{NodeId, Address, DataType};
//! # fn bus_send(_cob: u16, _f: &[u8; 8]) {}
//! # fn bus_recv() -> [u8; 8] { [0; 8] }
//! let mut client = SdoClient::new(NodeId::new(0x10).unwrap());
//! let mut frame = client.read(Address::new(0x1000, 0), DataType::Unsigned32);
//! loop {
//!     bus_send(client.request_cob_id(), &frame);
//!     match client.on_response(&bus_recv()) {
//!         SdoEvent::Send(next) => frame = next,
//!         SdoEvent::Complete(value) => break, // value: Option<Value>
//!         SdoEvent::Aborted(code) => break,
//!     }
//! }
//! ```

use heapless::Vec;

use super::{
    decode_abort, decode_data_segment, decode_download_response, decode_download_segment_response,
    decode_upload_expedited_response, decode_upload_initiate_segmented_response, encode_data_segment,
    encode_download_expedited, encode_download_initiate_segmented, encode_upload_request,
    encode_upload_segment_request, request_cob_id, response_cob_id, SdoPayload, SEGMENT_DATA_MAX,
};
use crate::datatypes::{DataType, Value};
use crate::object_dictionary::Address;
use crate::types::NodeId;

/// The abort code for "toggle bit not alternated" (CiA 301 §7.2.4.3.17), used
/// when the client itself detects an out-of-sequence segment.
const ABORT_TOGGLE: u32 = 0x0503_0000;
/// The abort code for a general internal error.
const ABORT_GENERAL: u32 = 0x0800_0000;

/// The outcome of feeding a response to [`SdoClient::on_response`].
#[derive(Debug, Clone, PartialEq)]
pub enum SdoEvent {
    /// Transmit this frame, then feed the reply back to [`SdoClient::on_response`].
    Send(SdoPayload),
    /// The transaction finished. A read yields `Some(value)`; a write `None`.
    Complete(Option<Value>),
    /// The transaction was aborted with this raw SDO abort code.
    Aborted(u32),
}

#[derive(Debug)]
enum State {
    Idle,
    /// Awaiting the response to an expedited download (write).
    DownloadExpedited,
    /// Awaiting the segmented-download initiate response, then sending segments.
    DownloadInit { data: [u8; 8], len: usize },
    /// Awaiting the acknowledgement of the last-sent segment.
    DownloadSeg { data: [u8; 8], len: usize, pos: usize, last_toggle: bool },
    /// Awaiting the upload initiate response.
    UploadInit { data_type: DataType },
    /// Awaiting the next upload data segment; `toggle` is the polled toggle.
    UploadSeg { data_type: DataType, buf: Vec<u8, 8>, toggle: bool },
}

/// An SDO client bound to the target node's id.
#[derive(Debug)]
pub struct SdoClient {
    node: NodeId,
    state: State,
}

impl SdoClient {
    /// Create a client targeting `node`.
    pub const fn new(node: NodeId) -> Self {
        Self { node, state: State::Idle }
    }

    /// The COB-ID the client transmits requests on (`0x600 + node`).
    pub fn request_cob_id(&self) -> u16 {
        request_cob_id(self.node)
    }

    /// The COB-ID the client receives responses on (`0x580 + node`).
    pub fn response_cob_id(&self) -> u16 {
        response_cob_id(self.node)
    }

    /// Whether a transaction is in progress.
    pub fn is_busy(&self) -> bool {
        !matches!(self.state, State::Idle)
    }

    /// Begin reading `addr`, interpreting the result as `data_type`. Returns the
    /// first request frame to transmit.
    pub fn read(&mut self, addr: Address, data_type: DataType) -> SdoPayload {
        self.state = State::UploadInit { data_type };
        encode_upload_request(addr)
    }

    /// Begin writing `value` to `addr`. Returns the first request frame to
    /// transmit; expedited or segmented is chosen from the value's size.
    pub fn write(&mut self, addr: Address, value: Value) -> SdoPayload {
        let size = value.size();
        if size <= 4 {
            self.state = State::DownloadExpedited;
            encode_download_expedited(addr, &value).expect("size <= 4")
        } else {
            let mut data = [0u8; 8];
            value.encode_le(&mut data[..size]).expect("size <= 8");
            self.state = State::DownloadInit { data, len: size };
            encode_download_initiate_segmented(addr, size as u32)
        }
    }

    /// Feed a server response frame and advance the transaction.
    pub fn on_response(&mut self, resp: &SdoPayload) -> SdoEvent {
        // Any response may be an abort; handle it uniformly first.
        if let Ok((_, code)) = decode_abort(resp) {
            self.state = State::Idle;
            return SdoEvent::Aborted(code);
        }
        match core::mem::replace(&mut self.state, State::Idle) {
            State::Idle => SdoEvent::Aborted(ABORT_GENERAL),
            State::DownloadExpedited => self.on_download_expedited(resp),
            State::DownloadInit { data, len } => self.on_download_init(resp, data, len),
            State::DownloadSeg { data, len, pos, last_toggle } => {
                self.on_download_seg(resp, data, len, pos, last_toggle)
            }
            State::UploadInit { data_type } => self.on_upload_init(resp, data_type),
            State::UploadSeg { data_type, buf, toggle } => {
                self.on_upload_seg(resp, data_type, buf, toggle)
            }
        }
    }

    fn on_download_expedited(&mut self, resp: &SdoPayload) -> SdoEvent {
        match decode_download_response(resp) {
            Ok(_) => SdoEvent::Complete(None),
            Err(_) => SdoEvent::Aborted(ABORT_GENERAL),
        }
    }

    fn on_download_init(&mut self, resp: &SdoPayload, data: [u8; 8], len: usize) -> SdoEvent {
        // The initiate response for a segmented download is the same frame as
        // an expedited download response.
        if decode_download_response(resp).is_err() {
            return SdoEvent::Aborted(ABORT_GENERAL);
        }
        self.send_download_segment(data, len, 0, false)
    }

    fn on_download_seg(
        &mut self,
        resp: &SdoPayload,
        data: [u8; 8],
        len: usize,
        pos: usize,
        last_toggle: bool,
    ) -> SdoEvent {
        match decode_download_segment_response(resp) {
            Ok(t) if t == last_toggle => {
                if pos >= len {
                    SdoEvent::Complete(None)
                } else {
                    self.send_download_segment(data, len, pos, !last_toggle)
                }
            }
            Ok(_) => SdoEvent::Aborted(ABORT_TOGGLE),
            Err(_) => SdoEvent::Aborted(ABORT_GENERAL),
        }
    }

    /// Emit the next download data segment starting at `pos` with `toggle`, and
    /// park in [`State::DownloadSeg`] awaiting its acknowledgement.
    fn send_download_segment(&mut self, data: [u8; 8], len: usize, pos: usize, toggle: bool) -> SdoEvent {
        let remaining = len - pos;
        let n = remaining.min(SEGMENT_DATA_MAX);
        let last = remaining <= SEGMENT_DATA_MAX;
        let frame = encode_data_segment(&data[pos..pos + n], toggle, last).expect("1..=7");
        self.state = State::DownloadSeg { data, len, pos: pos + n, last_toggle: toggle };
        SdoEvent::Send(frame)
    }

    fn on_upload_init(&mut self, resp: &SdoPayload, data_type: DataType) -> SdoEvent {
        // A small value arrives expedited; a large one starts a segmented read.
        if let Ok((_, value)) = decode_upload_expedited_response(resp, data_type) {
            return SdoEvent::Complete(Some(value));
        }
        if decode_upload_initiate_segmented_response(resp).is_ok() {
            self.state = State::UploadSeg { data_type, buf: Vec::new(), toggle: false };
            return SdoEvent::Send(encode_upload_segment_request(false));
        }
        SdoEvent::Aborted(ABORT_GENERAL)
    }

    fn on_upload_seg(
        &mut self,
        resp: &SdoPayload,
        data_type: DataType,
        mut buf: Vec<u8, 8>,
        toggle: bool,
    ) -> SdoEvent {
        let seg = match decode_data_segment(resp) {
            Ok(s) => s,
            Err(_) => return SdoEvent::Aborted(ABORT_GENERAL),
        };
        if seg.toggle != toggle {
            return SdoEvent::Aborted(ABORT_TOGGLE);
        }
        if buf.extend_from_slice(seg.data).is_err() {
            return SdoEvent::Aborted(ABORT_GENERAL);
        }
        if seg.last {
            match Value::decode_le(data_type, &buf) {
                Ok(value) => SdoEvent::Complete(Some(value)),
                Err(_) => SdoEvent::Aborted(ABORT_GENERAL),
            }
        } else {
            let next = !toggle;
            self.state = State::UploadSeg { data_type, buf, toggle: next };
            SdoEvent::Send(encode_upload_segment_request(next))
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn client() -> SdoClient {
        SdoClient::new(NodeId::new(0x10).unwrap())
    }

    #[test]
    fn cob_ids_track_node() {
        let c = client();
        assert_eq!(c.request_cob_id(), 0x610);
        assert_eq!(c.response_cob_id(), 0x590);
    }

    #[test]
    fn read_emits_upload_request_then_completes_on_expedited_response() {
        let mut c = client();
        let req = c.read(Address::new(0x1000, 0), DataType::Unsigned32);
        assert_eq!(req[0], 0x40); // upload initiate
        assert!(c.is_busy());
        // Server replies with an expedited upload response carrying 0x192.
        let resp =
            super::super::encode_upload_expedited_response(Address::new(0x1000, 0), &Value::Unsigned32(0x192))
                .unwrap();
        assert_eq!(c.on_response(&resp), SdoEvent::Complete(Some(Value::Unsigned32(0x192))));
        assert!(!c.is_busy());
    }

    #[test]
    fn write_small_value_is_expedited() {
        let mut c = client();
        let req = c.write(Address::new(0x1017, 0), Value::Unsigned16(1234));
        assert_eq!(req[0] & 0xE0, 0x20); // download initiate
        assert_ne!(req[0] & 0x02, 0); // expedited
        let resp = super::super::encode_download_response(Address::new(0x1017, 0));
        assert_eq!(c.on_response(&resp), SdoEvent::Complete(None));
    }

    #[test]
    fn abort_response_surfaces_code() {
        let mut c = client();
        c.read(Address::new(0x9999, 0), DataType::Unsigned32);
        let abort = super::super::encode_abort(
            Address::new(0x9999, 0),
            super::super::SdoAbortCode::ObjectDoesNotExist,
        );
        assert_eq!(c.on_response(&abort), SdoEvent::Aborted(0x0602_0000));
        assert!(!c.is_busy());
    }
}
