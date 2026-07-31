//! SDO **block** download (CiA 301 §7.2.4.3.5) — high-throughput bulk transfer.
//!
//! Where segmented transfer acknowledges every 7-byte segment, block transfer
//! streams a whole *sub-block* of up to 127 segments before a single
//! acknowledgement, then verifies the transfer with a CRC. This module
//! implements the client→server **download** direction: the initiate/end frame
//! codecs, the sub-block segment and acknowledgement codecs, the CRC-16
//! ([`crc16`]), and the [`BlockDownloadWriter`] / [`BlockDownloadReceiver`]
//! helpers that split and reassemble the data on the happy path (in-order, no
//! retransmission).
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

const CCS_BLOCK_DOWNLOAD: u8 = 0xC0; // client command specifier 6
const SCS_BLOCK_DOWNLOAD: u8 = 0xA0; // server command specifier 5
const CS_MASK: u8 = 0xE0;

const CLIENT_CRC: u8 = 0x04; // 'cc' bit in the initiate request
const SERVER_CRC: u8 = 0x04; // 'sc' bit in the initiate response
const SIZE_INDICATED: u8 = 0x02; // 's' bit in the initiate request

const SUBCMD_MASK: u8 = 0x03;
const SUBCMD_INITIATE: u8 = 0x00;
const SUBCMD_END: u8 = 0x01;
const SUBCMD_RESPONSE: u8 = 0x02;

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

/// A decoded block-download initiate response.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct InitiateResponse {
    /// The object being written.
    pub address: Address,
    /// The number of segments the client may send before an acknowledgement.
    pub blksize: u8,
    /// Whether the server supports the end-of-transfer CRC.
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

// --- Frame codecs ----------------------------------------------------------

/// Encode a client **block-download initiate** request for `addr`, optionally
/// declaring the total `size` and whether the client supports the CRC.
pub fn encode_download_initiate(addr: Address, size: Option<u32>, crc_support: bool) -> SdoPayload {
    let mut p = [0u8; 8];
    p[0] = CCS_BLOCK_DOWNLOAD | SUBCMD_INITIATE;
    if crc_support {
        p[0] |= CLIENT_CRC;
    }
    if let Some(size) = size {
        p[0] |= SIZE_INDICATED;
        p[4..8].copy_from_slice(&size.to_le_bytes());
    }
    p[1..3].copy_from_slice(&addr.index.to_le_bytes());
    p[3] = addr.subindex;
    p
}

/// Decode a server block-download initiate response.
pub fn decode_download_initiate_response(p: &SdoPayload) -> Result<InitiateResponse> {
    if p[0] & CS_MASK != SCS_BLOCK_DOWNLOAD || p[0] & SUBCMD_MASK != SUBCMD_INITIATE {
        return Err(Error::UnexpectedCommand);
    }
    Ok(InitiateResponse {
        address: Address::new(u16::from_le_bytes([p[1], p[2]]), p[3]),
        blksize: p[4],
        crc_support: p[0] & SERVER_CRC != 0,
    })
}

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

/// Encode the server's **sub-block response**: `ackseq` is the highest
/// sequence number correctly received, and `blksize` the size of the next
/// sub-block.
pub fn encode_sub_response(ackseq: u8, blksize: u8) -> SdoPayload {
    let mut p = [0u8; 8];
    p[0] = SCS_BLOCK_DOWNLOAD | SUBCMD_RESPONSE;
    p[1] = ackseq;
    p[2] = blksize;
    p
}

/// Decode a sub-block response into `(ackseq, blksize)`.
pub fn decode_sub_response(p: &SdoPayload) -> Result<(u8, u8)> {
    if p[0] & CS_MASK != SCS_BLOCK_DOWNLOAD || p[0] & SUBCMD_MASK != SUBCMD_RESPONSE {
        return Err(Error::UnexpectedCommand);
    }
    Ok((p[1], p[2]))
}

/// Encode the client's **end block-download** request. `unused` is how many of
/// the last segment's seven bytes carried no data; `crc` is the CRC over the
/// whole transfer (`0` if unused).
pub fn encode_download_end(unused: u8, crc: u16) -> SdoPayload {
    let mut p = [0u8; 8];
    p[0] = CCS_BLOCK_DOWNLOAD | ((unused & 0x07) << 2) | SUBCMD_END;
    p[1..3].copy_from_slice(&crc.to_le_bytes());
    p
}

/// Decode an end block-download request into `(unused, crc)`.
pub fn decode_download_end(p: &SdoPayload) -> Result<(u8, u16)> {
    if p[0] & CS_MASK != CCS_BLOCK_DOWNLOAD || p[0] & SUBCMD_MASK != SUBCMD_END {
        return Err(Error::UnexpectedCommand);
    }
    Ok(((p[0] >> 2) & 0x07, u16::from_le_bytes([p[1], p[2]])))
}

/// Encode the server's **end block-download response**.
pub fn encode_download_end_response() -> SdoPayload {
    let mut p = [0u8; 8];
    p[0] = SCS_BLOCK_DOWNLOAD | SUBCMD_END;
    p
}

/// Decode an end block-download response (confirms completion).
pub fn decode_download_end_response(p: &SdoPayload) -> Result<()> {
    if p[0] & CS_MASK != SCS_BLOCK_DOWNLOAD || p[0] & SUBCMD_MASK != SUBCMD_END {
        return Err(Error::UnexpectedCommand);
    }
    Ok(())
}

// --- Stateful helpers (happy path) -----------------------------------------

/// Splits a byte buffer into block-download sub-block segments.
///
/// Drive it after the initiate handshake: pull [`BlockDownloadWriter::next_segment`]
/// until it yields `None`; if [`BlockDownloadWriter::is_done`] then send the
/// [`BlockDownloadWriter::end_frame`], otherwise transmit the sub-block, await
/// the acknowledgement, call [`BlockDownloadWriter::start_sub_block`] with the
/// next blksize, and repeat.
#[derive(Debug)]
pub struct BlockDownloadWriter<'a> {
    data: &'a [u8],
    pos: usize,
    seqno: u8,
    blksize: u8,
}

impl<'a> BlockDownloadWriter<'a> {
    /// Start splitting `data` using the server's first `blksize`.
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
        encode_download_end(unused, crc)
    }
}

/// Reassembles block-download sub-block segments into a bounded buffer of
/// capacity `N`.
#[derive(Debug)]
pub struct BlockDownloadReceiver<const N: usize> {
    buf: Vec<u8, N>,
    done: bool,
}

impl<const N: usize> Default for BlockDownloadReceiver<N> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const N: usize> BlockDownloadReceiver<N> {
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
        // The canonical CRC-16/XMODEM check value.
        assert_eq!(crc16(b"123456789"), 0x31C3);
        assert_eq!(crc16(&[]), 0x0000);
    }

    // Known-good frame: initiate a 100-byte download to 0x2000 sub 0 with CRC
    // and size indicated. Command 0xC6 = block download, cc + s.
    #[test]
    fn initiate_matches_known_frame() {
        let f = encode_download_initiate(Address::new(0x2000, 0), Some(100), true);
        assert_eq!(f, [0xC6, 0x00, 0x20, 0x00, 100, 0, 0, 0]);
    }

    #[test]
    fn initiate_response_roundtrips() {
        // Command 0xA4 = block download response, server CRC, subcmd initiate.
        let f = [0xA4, 0x00, 0x20, 0x00, 10, 0, 0, 0];
        let r = decode_download_initiate_response(&f).unwrap();
        assert_eq!(
            r,
            InitiateResponse {
                address: Address::new(0x2000, 0),
                blksize: 10,
                crc_support: true
            }
        );
    }

    #[test]
    fn sub_segment_frames() {
        // First segment, 7 bytes, not last: command byte = seqno 1.
        let f = encode_sub_segment(1, &[1, 2, 3, 4, 5, 6, 7], false).unwrap();
        assert_eq!(f, [0x01, 1, 2, 3, 4, 5, 6, 7]);
        // Last segment, seqno 3, 2 bytes: command byte = 0x80 | 3.
        let f = encode_sub_segment(3, &[0xAA, 0xBB], true).unwrap();
        assert_eq!(f, [0x83, 0xAA, 0xBB, 0, 0, 0, 0, 0]);
        let seg = decode_sub_segment(&f);
        assert!(seg.last);
        assert_eq!(seg.seqno, 3);
    }

    #[test]
    fn sub_response_frame() {
        assert_eq!(encode_sub_response(10, 20), [0xA2, 10, 20, 0, 0, 0, 0, 0]);
        assert_eq!(
            decode_sub_response(&[0xA2, 10, 20, 0, 0, 0, 0, 0]).unwrap(),
            (10, 20)
        );
    }

    #[test]
    fn end_frames() {
        // 2 real bytes in the last segment -> 5 unused. Command 0xD5.
        let f = encode_download_end(5, 0xBEEF);
        assert_eq!(f, [0xD5, 0xEF, 0xBE, 0, 0, 0, 0, 0]);
        assert_eq!(decode_download_end(&f).unwrap(), (5, 0xBEEF));
        assert_eq!(encode_download_end_response(), [0xA1, 0, 0, 0, 0, 0, 0, 0]);
        assert!(decode_download_end_response(&[0xA1, 0, 0, 0, 0, 0, 0, 0]).is_ok());
    }

    // End-to-end: split 30 bytes across sub-blocks of 2 segments, reassemble,
    // and verify the CRC.
    #[test]
    fn full_block_download_roundtrip() {
        let data: [u8; 30] = core::array::from_fn(|i| i as u8);
        let blksize = 2;
        let mut writer = BlockDownloadWriter::new(&data, blksize);
        let mut receiver = BlockDownloadReceiver::<64>::new();

        loop {
            while let Some(seg) = writer.next_segment() {
                receiver.push(&decode_sub_segment(&seg)).unwrap();
            }
            if writer.is_done() {
                break;
            }
            writer.start_sub_block(blksize); // acknowledge, same blksize
        }

        assert!(receiver.is_done());
        let end = writer.end_frame(true);
        let (unused, crc) = decode_download_end(&end).unwrap();
        let received = receiver.finish(unused, crc, true).unwrap();
        assert_eq!(received, &data);
    }

    #[test]
    fn crc_mismatch_is_detected() {
        let data = [1u8, 2, 3, 4, 5];
        let mut writer = BlockDownloadWriter::new(&data, 1);
        let mut receiver = BlockDownloadReceiver::<16>::new();
        while let Some(seg) = writer.next_segment() {
            receiver.push(&decode_sub_segment(&seg)).unwrap();
        }
        let (unused, _) = decode_download_end(&writer.end_frame(true)).unwrap();
        assert_eq!(
            receiver.finish(unused, 0x0000, true),
            Err(Error::CrcMismatch)
        );
    }
}
