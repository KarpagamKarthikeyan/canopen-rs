//! SDO **block** transfer (CiA 301 §7.2.4.3) — high-throughput bulk transfer.
//!
//! Where segmented transfer acknowledges every 7-byte segment, block transfer
//! streams a whole *sub-block* of up to 127 segments before a single
//! acknowledgement, then verifies the transfer with a CRC. This module
//! implements both directions — client→server **download** and server→client
//! **upload** — as frame codecs plus the CRC-16 ([`crc16`]) and the
//! [`BlockWriter`] / [`BlockReceiver`] helpers that split and reassemble the
//! data on the happy path (in-order, no retransmission).
//!
//! Block download and upload are mirror images: several frames (the sub-block
//! segment, sub-block response, end, and end-response) are byte-identical in
//! both directions and share one codec here; only the initiate handshake and
//! the upload *start* frame differ.
//!
//! ```
//! use canopen_rs::sdo::block::crc16;
//!
//! // CRC-16/XMODEM — the algorithm CANopen block transfer uses.
//! assert_eq!(crc16(b"123456789"), 0x31C3);
//! ```

use heapless::Vec;

use super::SdoPayload;
use crate::object_dictionary::Address;
use crate::{Error, Result};

// Block transfer uses just two command-specifier values in byte 0's top three
// bits, swapped between client and server by direction:
//   CS_6 (110b): download client / upload server
//   CS_5 (101b): download server / upload client
const CS_6: u8 = 0xC0;
const CS_5: u8 = 0xA0;
const CS_MASK: u8 = 0xE0;

const CRC_SUPPORTED: u8 = 0x04; // 'cc'/'sc' bit in an initiate frame
const SIZE_INDICATED: u8 = 0x02; // 's' bit in an initiate frame

const SUBCMD_MASK: u8 = 0x03;
const SUBCMD_INITIATE: u8 = 0x00;
const SUBCMD_END: u8 = 0x01;
const SUBCMD_RESPONSE: u8 = 0x02;
const SUBCMD_START: u8 = 0x03;

const LAST_SEGMENT: u8 = 0x80; // 'c' bit in a sub-block segment
const SEQNO_MASK: u8 = 0x7F;

/// Maximum data bytes in one sub-block segment.
pub const SEGMENT_DATA_MAX: usize = 7;
/// Maximum segments in one sub-block.
pub const MAX_BLKSIZE: u8 = 127;

/// CRC-16/XMODEM (CCITT, polynomial `0x1021`, initial value `0`), computed over
/// the complete transferred data — the checksum CANopen block transfer uses.
pub fn crc16(data: &[u8]) -> u16 {
    let mut crc: u16 = 0;
    for &byte in data {
        crc ^= (byte as u16) << 8;
        for _ in 0..8 {
            crc = if crc & 0x8000 != 0 {
                (crc << 1) ^ 0x1021
            } else {
                crc << 1
            };
        }
    }
    crc
}

/// The size-carrying initiate frame: a download-initiate *request* or an
/// upload-initiate *response*.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BlockInitiate {
    /// The object being transferred.
    pub address: Address,
    /// The declared total size in bytes, if indicated.
    pub size: Option<u32>,
    /// Whether the sender supports the end-of-transfer CRC.
    pub crc_support: bool,
}

/// The blksize-carrying initiate frame: a download-initiate *response* or an
/// upload-initiate *request*.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BlockNegotiation {
    /// The object being transferred.
    pub address: Address,
    /// Segments allowed per sub-block before an acknowledgement (`1..=127`).
    pub blksize: u8,
    /// Whether the sender supports the end-of-transfer CRC.
    pub crc_support: bool,
}

/// A decoded sub-block segment.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SubSegment<'a> {
    /// The sequence number within the sub-block (`1..=blksize`).
    pub seqno: u8,
    /// Whether this is the last segment of the whole transfer.
    pub last: bool,
    /// The segment's seven raw payload bytes (the final segment's unused tail
    /// is trimmed later using the end frame's byte count).
    pub data: &'a [u8],
}

fn address_of(p: &SdoPayload) -> Address {
    Address::new(u16::from_le_bytes([p[1], p[2]]), p[3])
}

fn put_address(p: &mut SdoPayload, addr: Address) {
    p[1..3].copy_from_slice(&addr.index.to_le_bytes());
    p[3] = addr.subindex;
}

// --- Initiate handshake ----------------------------------------------------

/// Encode a client **block-download initiate** request (`CS_6`).
pub fn encode_download_initiate(addr: Address, size: Option<u32>, crc_support: bool) -> SdoPayload {
    encode_initiate_with_size(CS_6, addr, size, crc_support)
}

/// Decode a block-download initiate request.
pub fn decode_download_initiate(p: &SdoPayload) -> Result<BlockInitiate> {
    decode_initiate_with_size(CS_6, p)
}

/// Encode the server's **block-download initiate response** (`CS_5`).
pub fn encode_download_initiate_response(
    addr: Address,
    blksize: u8,
    crc_support: bool,
) -> SdoPayload {
    encode_initiate_with_blksize(CS_5, addr, blksize, crc_support)
}

/// Decode a block-download initiate response.
pub fn decode_download_initiate_response(p: &SdoPayload) -> Result<BlockNegotiation> {
    decode_initiate_with_blksize(CS_5, p)
}

/// Encode a client **block-upload initiate** request (`CS_5`).
pub fn encode_upload_initiate(addr: Address, blksize: u8, crc_support: bool) -> SdoPayload {
    encode_initiate_with_blksize(CS_5, addr, blksize, crc_support)
}

/// Decode a block-upload initiate request.
pub fn decode_upload_initiate(p: &SdoPayload) -> Result<BlockNegotiation> {
    decode_initiate_with_blksize(CS_5, p)
}

/// Encode the server's **block-upload initiate response** (`CS_6`).
pub fn encode_upload_initiate_response(
    addr: Address,
    size: Option<u32>,
    crc_support: bool,
) -> SdoPayload {
    encode_initiate_with_size(CS_6, addr, size, crc_support)
}

/// Decode a block-upload initiate response.
pub fn decode_upload_initiate_response(p: &SdoPayload) -> Result<BlockInitiate> {
    decode_initiate_with_size(CS_6, p)
}

/// Encode the client's **start block-upload** request, telling the server to
/// begin streaming segments (`CS_5`, sub-command start).
pub fn encode_upload_start() -> SdoPayload {
    let mut p = [0u8; 8];
    p[0] = CS_5 | SUBCMD_START;
    p
}

/// Decode a start block-upload request.
pub fn decode_upload_start(p: &SdoPayload) -> Result<()> {
    if p[0] & CS_MASK != CS_5 || p[0] & SUBCMD_MASK != SUBCMD_START {
        return Err(Error::UnexpectedCommand);
    }
    Ok(())
}

fn encode_initiate_with_size(cs: u8, addr: Address, size: Option<u32>, crc: bool) -> SdoPayload {
    let mut p = [0u8; 8];
    p[0] = cs | SUBCMD_INITIATE;
    if crc {
        p[0] |= CRC_SUPPORTED;
    }
    if let Some(size) = size {
        p[0] |= SIZE_INDICATED;
        p[4..8].copy_from_slice(&size.to_le_bytes());
    }
    put_address(&mut p, addr);
    p
}

fn decode_initiate_with_size(cs: u8, p: &SdoPayload) -> Result<BlockInitiate> {
    // On a size-carrying initiate, bit 1 is the 's' flag, so the initiate/end
    // sub-command is bit 0 alone (0 = initiate); don't match the full 2 bits.
    if p[0] & CS_MASK != cs || p[0] & SUBCMD_END != 0 {
        return Err(Error::UnexpectedCommand);
    }
    let size = (p[0] & SIZE_INDICATED != 0).then(|| u32::from_le_bytes([p[4], p[5], p[6], p[7]]));
    Ok(BlockInitiate {
        address: address_of(p),
        size,
        crc_support: p[0] & CRC_SUPPORTED != 0,
    })
}

fn encode_initiate_with_blksize(cs: u8, addr: Address, blksize: u8, crc: bool) -> SdoPayload {
    let mut p = [0u8; 8];
    p[0] = cs | SUBCMD_INITIATE;
    if crc {
        p[0] |= CRC_SUPPORTED;
    }
    put_address(&mut p, addr);
    p[4] = blksize;
    p
}

fn decode_initiate_with_blksize(cs: u8, p: &SdoPayload) -> Result<BlockNegotiation> {
    if p[0] & CS_MASK != cs || p[0] & SUBCMD_MASK != SUBCMD_INITIATE {
        return Err(Error::UnexpectedCommand);
    }
    Ok(BlockNegotiation {
        address: address_of(p),
        blksize: p[4],
        crc_support: p[0] & CRC_SUPPORTED != 0,
    })
}

// --- Sub-block segments and acknowledgements (shared both directions) -------

/// Encode a **sub-block segment** carrying 1..=7 bytes with sequence number
/// `seqno` (`1..=127`); `last` marks the final segment of the transfer.
pub fn encode_sub_segment(seqno: u8, data: &[u8], last: bool) -> Result<SdoPayload> {
    if seqno == 0 || seqno > MAX_BLKSIZE || data.is_empty() || data.len() > SEGMENT_DATA_MAX {
        return Err(Error::BadLength);
    }
    let mut p = [0u8; 8];
    p[0] = seqno;
    if last {
        p[0] |= LAST_SEGMENT;
    }
    p[1..1 + data.len()].copy_from_slice(data);
    Ok(p)
}

/// Decode a sub-block segment frame.
pub fn decode_sub_segment(p: &SdoPayload) -> SubSegment<'_> {
    SubSegment {
        seqno: p[0] & SEQNO_MASK,
        last: p[0] & LAST_SEGMENT != 0,
        data: &p[1..8],
    }
}

/// Encode a **sub-block response**: `ackseq` is the highest sequence number
/// correctly received, `blksize` the size of the next sub-block. (Sent by the
/// download server or the upload client — identical bytes.)
pub fn encode_sub_response(ackseq: u8, blksize: u8) -> SdoPayload {
    let mut p = [0u8; 8];
    p[0] = CS_5 | SUBCMD_RESPONSE;
    p[1] = ackseq;
    p[2] = blksize;
    p
}

/// Decode a sub-block response into `(ackseq, blksize)`.
pub fn decode_sub_response(p: &SdoPayload) -> Result<(u8, u8)> {
    if p[0] & CS_MASK != CS_5 || p[0] & SUBCMD_MASK != SUBCMD_RESPONSE {
        return Err(Error::UnexpectedCommand);
    }
    Ok((p[1], p[2]))
}

/// Encode the **end** frame. `unused` is how many of the last segment's seven
/// bytes carried no data; `crc` is the CRC over the whole transfer (`0` if
/// unused). (Sent by the download client or the upload server.)
pub fn encode_end(unused: u8, crc: u16) -> SdoPayload {
    let mut p = [0u8; 8];
    p[0] = CS_6 | ((unused & 0x07) << 2) | SUBCMD_END;
    p[1..3].copy_from_slice(&crc.to_le_bytes());
    p
}

/// Decode an end frame into `(unused, crc)`.
pub fn decode_end(p: &SdoPayload) -> Result<(u8, u16)> {
    if p[0] & CS_MASK != CS_6 || p[0] & SUBCMD_MASK != SUBCMD_END {
        return Err(Error::UnexpectedCommand);
    }
    Ok(((p[0] >> 2) & 0x07, u16::from_le_bytes([p[1], p[2]])))
}

/// Encode the **end response** confirming completion (download server or upload
/// client).
pub fn encode_end_response() -> SdoPayload {
    let mut p = [0u8; 8];
    p[0] = CS_5 | SUBCMD_END;
    p
}

/// Decode an end response.
pub fn decode_end_response(p: &SdoPayload) -> Result<()> {
    if p[0] & CS_MASK != CS_5 || p[0] & SUBCMD_MASK != SUBCMD_END {
        return Err(Error::UnexpectedCommand);
    }
    Ok(())
}

// --- Stateful helpers (happy path, both directions) ------------------------

/// Splits a byte buffer into sub-block segments — used by the download client
/// and the upload server.
///
/// Pull [`BlockWriter::next_segment`] until it yields `None`; if
/// [`BlockWriter::is_done`] then send the [`BlockWriter::end_frame`], otherwise
/// transmit the sub-block, await the acknowledgement, call
/// [`BlockWriter::start_sub_block`] with the next blksize, and repeat.
#[derive(Debug)]
pub struct BlockWriter<'a> {
    data: &'a [u8],
    pos: usize,
    seqno: u8,
    blksize: u8,
}

impl<'a> BlockWriter<'a> {
    /// Start splitting `data` using the negotiated first `blksize`.
    pub fn new(data: &'a [u8], blksize: u8) -> Self {
        Self {
            data,
            pos: 0,
            seqno: 1,
            blksize,
        }
    }

    /// Whether every byte has been emitted (then send [`Self::end_frame`]).
    pub const fn is_done(&self) -> bool {
        self.pos >= self.data.len()
    }

    /// The next segment of the current sub-block, or `None` when the sub-block
    /// is full or the data is exhausted (disambiguate with [`Self::is_done`]).
    pub fn next_segment(&mut self) -> Option<SdoPayload> {
        if self.is_done() || self.seqno > self.blksize {
            return None;
        }
        let remaining = self.data.len() - self.pos;
        let take = remaining.min(SEGMENT_DATA_MAX);
        let last = remaining <= SEGMENT_DATA_MAX;
        let segment = encode_sub_segment(self.seqno, &self.data[self.pos..self.pos + take], last)
            .expect("seqno and length are in range");
        self.pos += take;
        self.seqno += 1;
        Some(segment)
    }

    /// Begin the next sub-block after an acknowledgement, using `blksize`.
    pub fn start_sub_block(&mut self, blksize: u8) {
        self.seqno = 1;
        self.blksize = blksize;
    }

    /// The end frame: the CRC over all data (when `crc_support`) and the count
    /// of unused bytes in the last segment.
    pub fn end_frame(&self, crc_support: bool) -> SdoPayload {
        let last_len = match self.data.len() % SEGMENT_DATA_MAX {
            0 if !self.data.is_empty() => SEGMENT_DATA_MAX,
            r => r,
        };
        let unused = (SEGMENT_DATA_MAX - last_len) as u8;
        let crc = if crc_support { crc16(self.data) } else { 0 };
        encode_end(unused, crc)
    }
}

/// Reassembles sub-block segments into a bounded buffer of capacity `N` — used
/// by the download server and the upload client.
#[derive(Debug)]
pub struct BlockReceiver<const N: usize> {
    buf: Vec<u8, N>,
    done: bool,
}

impl<const N: usize> Default for BlockReceiver<N> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const N: usize> BlockReceiver<N> {
    /// A new, empty receiver.
    pub const fn new() -> Self {
        Self {
            buf: Vec::new(),
            done: false,
        }
    }

    /// Whether the last segment has been received (then call [`Self::finish`]).
    pub const fn is_done(&self) -> bool {
        self.done
    }

    /// Append a decoded sub-block segment's seven bytes.
    ///
    /// Returns [`Error::Overflow`] beyond capacity `N`, or
    /// [`Error::UnexpectedCommand`] if the transfer is already complete.
    pub fn push(&mut self, segment: &SubSegment) -> Result<()> {
        if self.done {
            return Err(Error::UnexpectedCommand);
        }
        self.buf
            .extend_from_slice(segment.data)
            .map_err(|_| Error::Overflow)?;
        if segment.last {
            self.done = true;
        }
        Ok(())
    }

    /// Finalise: trim the `unused` tail bytes of the last segment and, when
    /// `verify_crc`, check the transfer CRC. Returns the reassembled data.
    ///
    /// Returns [`Error::BadLength`] if `unused` exceeds the buffer, or
    /// [`Error::CrcMismatch`] on a CRC failure.
    pub fn finish(&mut self, unused: u8, crc: u16, verify_crc: bool) -> Result<&[u8]> {
        let unused = unused as usize;
        if unused > self.buf.len() {
            return Err(Error::BadLength);
        }
        self.buf.truncate(self.buf.len() - unused);
        if verify_crc && crc16(&self.buf) != crc {
            return Err(Error::CrcMismatch);
        }
        Ok(&self.buf)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn crc16_known_vector() {
        assert_eq!(crc16(b"123456789"), 0x31C3); // canonical CRC-16/XMODEM check
        assert_eq!(crc16(&[]), 0x0000);
    }

    // Known-good frames: initiate a 100-byte download to 0x2000 sub 0 with CRC
    // and size (command 0xC6 = CS_6 | cc | s), and its response with blksize 10
    // and server CRC (command 0xA4 = CS_5 | sc).
    #[test]
    fn download_initiate_frames() {
        let req = encode_download_initiate(Address::new(0x2000, 0), Some(100), true);
        assert_eq!(req, [0xC6, 0x00, 0x20, 0x00, 100, 0, 0, 0]);
        assert_eq!(
            decode_download_initiate(&req).unwrap(),
            BlockInitiate {
                address: Address::new(0x2000, 0),
                size: Some(100),
                crc_support: true
            }
        );
        let resp = encode_download_initiate_response(Address::new(0x2000, 0), 10, true);
        assert_eq!(resp, [0xA4, 0x00, 0x20, 0x00, 10, 0, 0, 0]);
        assert_eq!(
            decode_download_initiate_response(&resp).unwrap(),
            BlockNegotiation {
                address: Address::new(0x2000, 0),
                blksize: 10,
                crc_support: true
            }
        );
    }

    // Upload mirrors download: request carries blksize (CS_5), response carries
    // size (CS_6), plus the start frame (0xA3).
    #[test]
    fn upload_initiate_and_start_frames() {
        let req = encode_upload_initiate(Address::new(0x2000, 0), 5, false);
        assert_eq!(req, [0xA0, 0x00, 0x20, 0x00, 5, 0, 0, 0]);
        assert_eq!(decode_upload_initiate(&req).unwrap().blksize, 5);

        let resp = encode_upload_initiate_response(Address::new(0x2000, 0), Some(42), true);
        assert_eq!(resp, [0xC6, 0x00, 0x20, 0x00, 42, 0, 0, 0]);
        assert_eq!(
            decode_upload_initiate_response(&resp).unwrap().size,
            Some(42)
        );

        assert_eq!(encode_upload_start(), [0xA3, 0, 0, 0, 0, 0, 0, 0]);
        assert!(decode_upload_start(&encode_upload_start()).is_ok());
    }

    #[test]
    fn shared_segment_and_ack_frames() {
        let f = encode_sub_segment(1, &[1, 2, 3, 4, 5, 6, 7], false).unwrap();
        assert_eq!(f, [0x01, 1, 2, 3, 4, 5, 6, 7]);
        let last = encode_sub_segment(3, &[0xAA, 0xBB], true).unwrap();
        assert_eq!(last, [0x83, 0xAA, 0xBB, 0, 0, 0, 0, 0]);
        assert!(decode_sub_segment(&last).last);

        assert_eq!(encode_sub_response(10, 20), [0xA2, 10, 20, 0, 0, 0, 0, 0]);
        assert_eq!(
            decode_sub_response(&encode_sub_response(10, 20)).unwrap(),
            (10, 20)
        );

        assert_eq!(encode_end(5, 0xBEEF), [0xD5, 0xEF, 0xBE, 0, 0, 0, 0, 0]);
        assert_eq!(decode_end(&encode_end(5, 0xBEEF)).unwrap(), (5, 0xBEEF));
        assert_eq!(encode_end_response(), [0xA1, 0, 0, 0, 0, 0, 0, 0]);
        assert!(decode_end_response(&encode_end_response()).is_ok());
    }

    /// Split `data` with a [`BlockWriter`] and reassemble with a
    /// [`BlockReceiver`], verifying the CRC. This is the shared engine of both
    /// download and upload; the direction only changes which side runs which.
    fn roundtrip(data: &[u8], blksize: u8) {
        let mut writer = BlockWriter::new(data, blksize);
        let mut receiver = BlockReceiver::<128>::new();
        loop {
            while let Some(seg) = writer.next_segment() {
                receiver.push(&decode_sub_segment(&seg)).unwrap();
            }
            if writer.is_done() {
                break;
            }
            writer.start_sub_block(blksize);
        }
        assert!(receiver.is_done());
        let (unused, crc) = decode_end(&writer.end_frame(true)).unwrap();
        assert_eq!(receiver.finish(unused, crc, true).unwrap(), data);
    }

    #[test]
    fn block_transfer_roundtrips() {
        let data: [u8; 30] = core::array::from_fn(|i| i as u8);
        roundtrip(&data, 2); // spans multiple sub-blocks
        roundtrip(&data, 127); // one sub-block
        roundtrip(&[0xAB; 7], 4); // exactly one full segment
    }

    #[test]
    fn crc_mismatch_is_detected() {
        let data = [1u8, 2, 3, 4, 5];
        let mut writer = BlockWriter::new(&data, 1);
        let mut receiver = BlockReceiver::<16>::new();
        while let Some(seg) = writer.next_segment() {
            receiver.push(&decode_sub_segment(&seg)).unwrap();
        }
        let (unused, _) = decode_end(&writer.end_frame(true)).unwrap();
        assert_eq!(
            receiver.finish(unused, 0x0000, true),
            Err(Error::CrcMismatch)
        );
    }
}
