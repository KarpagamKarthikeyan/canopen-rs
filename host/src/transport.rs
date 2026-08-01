//! Linux SocketCAN transport for `canopen-rs`.
//!
//! Bridges the CANopen core codecs to a Linux SocketCAN interface. The
//! [`socketcan`] crate's [`CanFrame`](socketcan::CanFrame) implements the
//! [`embedded_can`] traits, so the core's frame helpers
//! ([`canopen_rs::transport`]) turn COB-IDs and payloads straight into bus
//! traffic.
//!
//! [`SocketCan::open`](crate::transport::SocketCan::open) binds a named
//! interface (e.g. `"can0"`, or `"vcan0"` for a virtual bus). Beyond raw
//! [`SocketCan::send`](crate::transport::SocketCan::send) /
//! [`SocketCan::recv`](crate::transport::SocketCan::recv), the
//! [`SocketCan::sdo_read`](crate::transport::SocketCan::sdo_read) and
//! [`SocketCan::sdo_write`](crate::transport::SocketCan::sdo_write) helpers run
//! a whole SDO transaction — expedited or segmented — against a remote node in
//! one call.
//!
//! [`socketcan`]: https://docs.rs/socketcan

use std::fmt;
use std::io;
use std::time::Duration;

use embedded_can::Frame as _;
use socketcan::{CanFrame, CanSocket, Socket};

use canopen_rs::datatypes::{DataType, Value};
use canopen_rs::nmt::{encode_command, NMT_COMMAND_COB_ID};
use canopen_rs::object_dictionary::Address;
use canopen_rs::sdo::{SdoClient, SdoEvent};
use canopen_rs::transport::{cob_id, frame_from};
use canopen_rs::types::NodeId;
use canopen_rs::NmtCommand;

/// A CANopen frame received from the bus: its COB-ID and up to eight bytes.
#[derive(Debug, Clone)]
pub struct Received {
    /// The frame's COB-ID (11-bit standard identifier).
    pub cob_id: u16,
    data: [u8; 8],
    len: usize,
}

impl Received {
    /// Build a received frame from a COB-ID and its data bytes (clamped to 8).
    pub(crate) fn new(cob_id: u16, bytes: &[u8]) -> Self {
        let len = bytes.len().min(8);
        let mut data = [0u8; 8];
        data[..len].copy_from_slice(&bytes[..len]);
        Self { cob_id, data, len }
    }

    /// The received bytes, trimmed to the frame's data length.
    pub fn data(&self) -> &[u8] {
        &self.data[..self.len]
    }

    /// The full eight-byte payload, zero-padded — convenient for the
    /// fixed-size SDO codecs, which expect a `&[u8; 8]`.
    pub fn payload(&self) -> &[u8; 8] {
        &self.data
    }
}

/// An error from an SDO transaction over the bus.
#[derive(Debug)]
pub enum SdoError {
    /// An I/O error on the socket (including a read timeout).
    Io(io::Error),
    /// The server (or client) aborted with this raw SDO abort code.
    Aborted(u32),
    /// The transfer completed without yielding an expected value.
    NoValue,
}

impl fmt::Display for SdoError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            SdoError::Io(e) => write!(f, "SDO transport I/O error: {e}"),
            SdoError::Aborted(code) => write!(f, "SDO transfer aborted (code {code:#010x})"),
            SdoError::NoValue => f.write_str("SDO transfer completed without a value"),
        }
    }
}

impl std::error::Error for SdoError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            SdoError::Io(e) => Some(e),
            _ => None,
        }
    }
}

impl From<io::Error> for SdoError {
    fn from(e: io::Error) -> Self {
        SdoError::Io(e)
    }
}

/// Wrap a socket-open failure with the interface name and, when the interface
/// does not exist (`ENODEV`), a hint that it may not be up.
pub(crate) fn open_error(interface: &str, e: io::Error) -> io::Error {
    // ENODEV (19) is what SocketCAN returns for a missing/downed interface.
    let hint = if e.raw_os_error() == Some(19) {
        format!(" — interface not found; is it up? (e.g. `sudo ip link set up {interface}`)")
    } else {
        String::new()
    };
    io::Error::new(
        e.kind(),
        format!("opening CAN interface '{interface}': {e}{hint}"),
    )
}

/// A CANopen transport over a Linux SocketCAN interface.
#[derive(Debug)]
pub struct SocketCan {
    socket: CanSocket,
}

impl SocketCan {
    /// Open the named CAN interface (e.g. `"can0"` or `"vcan0"`).
    ///
    /// Fails with a message naming the interface — and hinting that it may not
    /// be up — if it does not exist.
    pub fn open(interface: &str) -> io::Result<Self> {
        CanSocket::open(interface)
            .map(|socket| Self { socket })
            .map_err(|e| open_error(interface, e))
    }

    /// Set a read timeout, so [`SocketCan::recv`] (and the SDO helpers) fail
    /// with a timeout error rather than blocking forever.
    pub fn set_read_timeout(&self, timeout: Duration) -> io::Result<()> {
        self.socket.set_read_timeout(timeout)
    }

    /// Put the socket into (non-)blocking mode.
    pub fn set_nonblocking(&self, nonblocking: bool) -> io::Result<()> {
        self.socket.set_nonblocking(nonblocking)
    }

    /// Transmit `data` on COB-ID `cob_id` as a standard data frame.
    pub fn send(&self, cob_id: u16, data: &[u8]) -> io::Result<()> {
        let frame: CanFrame = frame_from(cob_id, data).ok_or_else(|| {
            io::Error::new(
                io::ErrorKind::InvalidInput,
                "invalid COB-ID or over-long data",
            )
        })?;
        self.socket.write_frame(&frame)
    }

    /// Receive the next CANopen data frame, skipping remote and error frames
    /// and any frame with a 29-bit extended identifier.
    pub fn recv(&self) -> io::Result<Received> {
        loop {
            let frame = self.socket.read_frame()?;
            if !frame.is_data_frame() {
                continue;
            }
            let Some(cob) = cob_id(&frame) else { continue };
            return Ok(Received::new(cob, frame.data()));
        }
    }

    /// Send an NMT node-control command to `target` on COB-ID `0x000`.
    ///
    /// Use [`NodeId::BROADCAST`] to address every node at once — e.g.
    /// `send_nmt(NmtCommand::StartRemoteNode, NodeId::BROADCAST)`.
    pub fn send_nmt(&self, command: NmtCommand, target: NodeId) -> io::Result<()> {
        self.send(NMT_COMMAND_COB_ID, &encode_command(command, target))
    }

    /// Receive frames until one arrives on `cob_id`, discarding the rest.
    fn recv_on(&self, cob_id: u16) -> io::Result<Received> {
        loop {
            let frame = self.recv()?;
            if frame.cob_id == cob_id {
                return Ok(frame);
            }
        }
    }

    /// Read object `addr` from `node`, interpreting the result as `data_type`.
    ///
    /// Runs the full SDO upload transaction (expedited or segmented) and
    /// returns the value, or an [`SdoError`] on abort or I/O failure. Set a
    /// read timeout first so an unresponsive node cannot block forever.
    pub fn sdo_read(
        &self,
        node: NodeId,
        addr: Address,
        data_type: DataType,
    ) -> Result<Value, SdoError> {
        let mut client = SdoClient::new(node);
        let mut request = client.read(addr, data_type);
        loop {
            self.send(client.request_cob_id(), &request)?;
            let reply = self.recv_on(client.response_cob_id())?;
            match client.on_response(reply.payload()) {
                SdoEvent::Send(next) => request = next,
                SdoEvent::Complete(value) => return value.ok_or(SdoError::NoValue),
                SdoEvent::Aborted(code) => return Err(SdoError::Aborted(code)),
            }
        }
    }

    /// Write `value` to object `addr` on `node`.
    ///
    /// Runs the full SDO download transaction (expedited or segmented),
    /// choosing the transfer type from the value's size.
    pub fn sdo_write(&self, node: NodeId, addr: Address, value: Value) -> Result<(), SdoError> {
        let mut client = SdoClient::new(node);
        let mut request = client.write(addr, value);
        loop {
            self.send(client.request_cob_id(), &request)?;
            let reply = self.recv_on(client.response_cob_id())?;
            match client.on_response(reply.payload()) {
                SdoEvent::Send(next) => request = next,
                SdoEvent::Complete(_) => return Ok(()),
                SdoEvent::Aborted(code) => return Err(SdoError::Aborted(code)),
            }
        }
    }
}
