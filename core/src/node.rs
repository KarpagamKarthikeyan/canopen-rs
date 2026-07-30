//! A CANopen device node — object dictionary + SDO server + NMT state in one
//! frame-driven type.
//!
//! [`Node`] bundles the pieces a device needs and, like the rest of the stack,
//! is **sans-I/O**: hand each incoming CAN frame to [`Node::on_frame`] and
//! transmit the [`TxFrame`] it returns (if any). It serves SDO requests against
//! its object dictionary, tracks NMT state from node-control commands, and
//! produces boot-up and heartbeat frames — the same logic on a host or an MCU.
//!
//! ```no_run
//! # use canopen_rs::{Address, Entry, NodeId, ObjectDictionary, Value};
//! # use canopen_rs::node::Node;
//! # fn bus_recv() -> (u16, [u8; 8]) { (0, [0; 8]) }
//! # fn bus_send(_cob: u16, _data: &[u8]) {}
//! let mut od = ObjectDictionary::<16>::new();
//! od.insert(Address::new(0x1000, 0), Entry::constant(Value::Unsigned32(0x0004_0192))).unwrap();
//! let mut node = Node::new(NodeId::new(0x10).unwrap(), od);
//!
//! let boot = node.boot();            // enter pre-operational, announce boot-up
//! bus_send(boot.cob_id, boot.data());
//!
//! loop {
//!     let (cob_id, data) = bus_recv();
//!     if let Some(tx) = node.on_frame(cob_id, &data) {
//!         bus_send(tx.cob_id, tx.data());
//!     }
//! }
//! ```

use crate::nmt::{self, NmtState, NmtStateMachine};
use crate::object_dictionary::ObjectDictionary;
use crate::sdo::{self, SdoServer};
use crate::types::NodeId;

/// A frame to transmit: an 11-bit COB-ID and up to eight data bytes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TxFrame {
    /// The COB-ID to transmit on.
    pub cob_id: u16,
    data: [u8; 8],
    len: u8,
}

impl TxFrame {
    fn new(cob_id: u16, bytes: &[u8]) -> Self {
        let len = bytes.len().min(8);
        let mut data = [0u8; 8];
        data[..len].copy_from_slice(&bytes[..len]);
        Self {
            cob_id,
            data,
            len: len as u8,
        }
    }

    /// The frame's data bytes (its DLC-trimmed payload).
    pub fn data(&self) -> &[u8] {
        &self.data[..self.len as usize]
    }
}

/// A CANopen device node: an object dictionary served over SDO, with NMT state
/// and heartbeat/boot-up production.
#[derive(Debug)]
pub struct Node<const N: usize> {
    node_id: NodeId,
    od: ObjectDictionary<N>,
    sdo: SdoServer,
    nmt: NmtStateMachine,
}

impl<const N: usize> Node<N> {
    /// Create a node with `node_id` serving `od`. It starts in
    /// [`NmtState::Initialising`]; call [`Node::boot`] to go operational-ready.
    pub fn new(node_id: NodeId, od: ObjectDictionary<N>) -> Self {
        Self {
            node_id,
            od,
            sdo: SdoServer::new(node_id),
            nmt: NmtStateMachine::new(),
        }
    }

    /// This node's id.
    pub fn node_id(&self) -> NodeId {
        self.node_id
    }

    /// The current NMT state.
    pub fn state(&self) -> NmtState {
        self.nmt.state()
    }

    /// Borrow the object dictionary (e.g. to publish process data).
    pub fn od(&self) -> &ObjectDictionary<N> {
        &self.od
    }

    /// Mutably borrow the object dictionary.
    pub fn od_mut(&mut self) -> &mut ObjectDictionary<N> {
        &mut self.od
    }

    /// Finish initialisation: enter pre-operational and return the boot-up
    /// frame to transmit (`0x700 + node`, data `0x00`).
    pub fn boot(&mut self) -> TxFrame {
        self.nmt.boot();
        TxFrame::new(nmt::heartbeat_cob_id(self.node_id), &nmt::BOOTUP_FRAME)
    }

    /// The heartbeat frame for the current state. Transmit it on your heartbeat
    /// timer (the producer heartbeat time lives in object `0x1017`).
    pub fn heartbeat(&self) -> TxFrame {
        TxFrame::new(
            nmt::heartbeat_cob_id(self.node_id),
            &nmt::encode_heartbeat(self.nmt.state()),
        )
    }

    /// Process an incoming CAN frame, returning a response to transmit, if any.
    ///
    /// Handles NMT node-control (`0x000`) by advancing the state machine, and
    /// SDO requests (`0x600 + node`) by serving the object dictionary. SDO is
    /// answered only in pre-operational and operational states, per CiA 301.
    /// Frames for other COB-IDs are ignored.
    pub fn on_frame(&mut self, cob_id: u16, data: &[u8]) -> Option<TxFrame> {
        if cob_id == nmt::NMT_COMMAND_COB_ID {
            self.on_nmt(data);
            None
        } else if cob_id == self.sdo.request_cob_id() {
            self.on_sdo(data)
        } else {
            None
        }
    }

    fn on_nmt(&mut self, data: &[u8]) {
        // An NMT node-control frame is [command specifier, target node].
        if data.len() < 2 {
            return;
        }
        if let Ok((command, target)) = nmt::decode_command(&[data[0], data[1]]) {
            if target == NodeId::BROADCAST || target == self.node_id {
                self.nmt.apply(command);
            }
        }
    }

    fn on_sdo(&mut self, data: &[u8]) -> Option<TxFrame> {
        // SDO is inactive outside pre-operational / operational (CiA 301 §7.3).
        if !matches!(
            self.nmt.state(),
            NmtState::PreOperational | NmtState::Operational
        ) {
            return None;
        }
        let mut payload: sdo::SdoPayload = [0u8; 8];
        if data.len() > payload.len() {
            return None;
        }
        payload[..data.len()].copy_from_slice(data);
        let response = self.sdo.handle(&mut self.od, &payload)?;
        Some(TxFrame::new(self.sdo.response_cob_id(), &response))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::object_dictionary::{Address, Entry};
    use crate::sdo::{encode_download_expedited, encode_upload_request};
    use crate::{DataType, NmtCommand, Value};

    fn od() -> ObjectDictionary<8> {
        let mut od = ObjectDictionary::new();
        od.insert(
            Address::new(0x1000, 0),
            Entry::constant(Value::Unsigned32(0x192)),
        )
        .unwrap();
        od.insert(Address::new(0x1017, 0), Entry::rw(Value::Unsigned16(1000)))
            .unwrap();
        od
    }

    fn node() -> Node<8> {
        Node::new(NodeId::new(0x10).unwrap(), od())
    }

    #[test]
    fn boots_from_init_to_preop_and_announces() {
        let mut n = node();
        assert_eq!(n.state(), NmtState::Initialising);
        let boot = n.boot();
        assert_eq!(n.state(), NmtState::PreOperational);
        assert_eq!(boot.cob_id, 0x710); // 0x700 + node
        assert_eq!(boot.data(), &[0x00]);
    }

    #[test]
    fn heartbeat_reflects_state() {
        let mut n = node();
        n.boot();
        assert_eq!(n.heartbeat().data(), &[0x7F]); // pre-operational
        n.on_frame(
            nmt::NMT_COMMAND_COB_ID,
            &[NmtCommand::StartRemoteNode as u8, 0x10],
        );
        assert_eq!(n.state(), NmtState::Operational);
        assert_eq!(n.heartbeat().data(), &[0x05]); // operational
    }

    #[test]
    fn serves_sdo_read_when_preoperational() {
        let mut n = node();
        n.boot();
        let req = encode_upload_request(Address::new(0x1000, 0));
        let resp = n.on_frame(0x610, &req).expect("SDO response");
        assert_eq!(resp.cob_id, 0x590); // 0x580 + node
        let (_, value) = crate::sdo::decode_upload_expedited_response(
            resp.data().try_into().unwrap(),
            DataType::Unsigned32,
        )
        .unwrap();
        assert_eq!(value, Value::Unsigned32(0x192));
    }

    #[test]
    fn ignores_sdo_before_boot() {
        let mut n = node(); // still Initialising
        let req = encode_upload_request(Address::new(0x1000, 0));
        assert!(n.on_frame(0x610, &req).is_none());
    }

    #[test]
    fn ignores_sdo_when_stopped() {
        let mut n = node();
        n.boot();
        n.on_frame(
            nmt::NMT_COMMAND_COB_ID,
            &[NmtCommand::StopRemoteNode as u8, 0x10],
        );
        assert_eq!(n.state(), NmtState::Stopped);
        let req = encode_upload_request(Address::new(0x1000, 0));
        assert!(n.on_frame(0x610, &req).is_none());
    }

    #[test]
    fn nmt_command_for_other_node_is_ignored() {
        let mut n = node();
        n.boot();
        // Start addressed to node 0x20, not us.
        n.on_frame(
            nmt::NMT_COMMAND_COB_ID,
            &[NmtCommand::StartRemoteNode as u8, 0x20],
        );
        assert_eq!(n.state(), NmtState::PreOperational); // unchanged
    }

    #[test]
    fn broadcast_nmt_applies() {
        let mut n = node();
        n.boot();
        n.on_frame(
            nmt::NMT_COMMAND_COB_ID,
            &[NmtCommand::StartRemoteNode as u8, 0x00],
        );
        assert_eq!(n.state(), NmtState::Operational);
    }

    #[test]
    fn serves_sdo_write_and_updates_od() {
        let mut n = node();
        n.boot();
        let req =
            encode_download_expedited(Address::new(0x1017, 0), &Value::Unsigned16(1234)).unwrap();
        assert!(n.on_frame(0x610, &req).is_some());
        assert_eq!(
            n.od().read(Address::new(0x1017, 0)).unwrap(),
            Value::Unsigned16(1234)
        );
    }

    #[test]
    fn ignores_unrelated_cob_id() {
        let mut n = node();
        n.boot();
        assert!(n.on_frame(0x123, &[0; 8]).is_none());
    }
}
