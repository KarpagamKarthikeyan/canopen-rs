//! Async (tokio) SocketCAN transport.
//!
//! [`AsyncSocketCan`] mirrors the blocking [`SocketCan`](crate::transport::SocketCan)
//! but every bus operation is an `async fn`, built on socketcan's tokio
//! integration. The protocol logic is the same sans-I/O [`SdoClient`] driving
//! the transactions — only the I/O is awaited — so it drops straight into a
//! tokio application.
//!
//! Enable the `tokio` feature; a tokio runtime is provided by your application.
//! There is no built-in read timeout — wrap a call in
//! [`tokio::time::timeout`](https://docs.rs/tokio/latest/tokio/time/fn.timeout.html)
//! for a deadline.
//!
//! ```ignore
//! use canopen_host::async_transport::AsyncSocketCan;
//! use canopen_rs::{Address, DataType, NodeId, Value};
//!
//! #[tokio::main]
//! async fn main() -> Result<(), Box<dyn std::error::Error>> {
//!     let bus = AsyncSocketCan::open("can0")?;
//!     let node = NodeId::new(0x10)?;
//!
//!     let device_type = bus.sdo_read(node, Address::new(0x1000, 0), DataType::Unsigned32).await?;
//!     println!("device type = {device_type:?}");
//!
//!     bus.sdo_write(node, Address::new(0x1017, 0), Value::Unsigned16(1000)).await?;
//!     Ok(())
//! }
//! ```

use std::io;

use embedded_can::Frame as _;
use socketcan::tokio::CanSocket;
use socketcan::CanFrame;

use canopen_rs::datatypes::{DataType, Value};
use canopen_rs::nmt::{encode_command, NMT_COMMAND_COB_ID};
use canopen_rs::object_dictionary::Address;
use canopen_rs::sdo::{SdoClient, SdoEvent};
use canopen_rs::transport::{cob_id, frame_from};
use canopen_rs::types::NodeId;
use canopen_rs::NmtCommand;

use crate::transport::{Received, SdoError};

/// An async CANopen transport over a Linux SocketCAN interface.
#[derive(Debug)]
pub struct AsyncSocketCan {
    socket: CanSocket,
}

impl AsyncSocketCan {
    /// Open the named CAN interface (e.g. `"can0"` or `"vcan0"`).
    pub fn open(interface: &str) -> io::Result<Self> {
        Ok(Self {
            socket: CanSocket::open(interface)?,
        })
    }

    /// Transmit `data` on COB-ID `cob_id` as a standard data frame.
    pub async fn send(&self, cob_id: u16, data: &[u8]) -> io::Result<()> {
        let frame: CanFrame = frame_from(cob_id, data).ok_or_else(|| {
            io::Error::new(
                io::ErrorKind::InvalidInput,
                "invalid COB-ID or over-long data",
            )
        })?;
        self.socket.write_frame(frame).await
    }

    /// Receive the next CANopen data frame, skipping remote/error frames and
    /// frames with a 29-bit extended identifier.
    pub async fn recv(&self) -> io::Result<Received> {
        loop {
            let frame = self.socket.read_frame().await?;
            if !frame.is_data_frame() {
                continue;
            }
            let Some(cob) = cob_id(&frame) else { continue };
            return Ok(Received::new(cob, frame.data()));
        }
    }

    /// Receive frames until one arrives on `cob_id`, discarding the rest.
    async fn recv_on(&self, cob_id: u16) -> io::Result<Received> {
        loop {
            let frame = self.recv().await?;
            if frame.cob_id == cob_id {
                return Ok(frame);
            }
        }
    }

    /// Send an NMT node-control command to `target` (use [`NodeId::BROADCAST`]
    /// for all nodes).
    pub async fn send_nmt(&self, command: NmtCommand, target: NodeId) -> io::Result<()> {
        self.send(NMT_COMMAND_COB_ID, &encode_command(command, target))
            .await
    }

    /// Read object `addr` from `node`, interpreting the result as `data_type`.
    /// Runs the full SDO upload (expedited or segmented).
    pub async fn sdo_read(
        &self,
        node: NodeId,
        addr: Address,
        data_type: DataType,
    ) -> Result<Value, SdoError> {
        let mut client = SdoClient::new(node);
        let mut request = client.read(addr, data_type);
        loop {
            self.send(client.request_cob_id(), &request).await?;
            let reply = self.recv_on(client.response_cob_id()).await?;
            match client.on_response(reply.payload()) {
                SdoEvent::Send(next) => request = next,
                SdoEvent::Complete(value) => return value.ok_or(SdoError::NoValue),
                SdoEvent::Aborted(code) => return Err(SdoError::Aborted(code)),
            }
        }
    }

    /// Write `value` to object `addr` on `node`. Runs the full SDO download
    /// (expedited or segmented), chosen from the value's size.
    pub async fn sdo_write(
        &self,
        node: NodeId,
        addr: Address,
        value: Value,
    ) -> Result<(), SdoError> {
        let mut client = SdoClient::new(node);
        let mut request = client.write(addr, value);
        loop {
            self.send(client.request_cob_id(), &request).await?;
            let reply = self.recv_on(client.response_cob_id()).await?;
            match client.on_response(reply.payload()) {
                SdoEvent::Send(next) => request = next,
                SdoEvent::Complete(_) => return Ok(()),
                SdoEvent::Aborted(code) => return Err(SdoError::Aborted(code)),
            }
        }
    }
}
