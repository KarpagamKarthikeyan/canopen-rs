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

use heapless::Vec;

use crate::lss::{self, LssAddress, LssSlave};
use crate::nmt::{self, NmtState, NmtStateMachine};
use crate::object_dictionary::ObjectDictionary;
use crate::pdo::{self, PdoMapping, TransmissionType};
use crate::sdo::{self, SdoServer};
use crate::types::NodeId;
use crate::{Error, Result};

/// The maximum number of transmit (or receive) PDOs a [`Node`] holds — the four
/// of the predefined connection set.
pub const MAX_PDOS: usize = 4;

/// The maximum objects mapped into one PDO: a full eight-byte frame of
/// one-byte objects.
pub const MAX_PDO_MAPPING: usize = 8;

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

/// A configured receive PDO: the COB-ID it listens on and its object mapping.
#[derive(Debug)]
struct RpdoSlot {
    cob_id: u16,
    mapping: PdoMapping<MAX_PDO_MAPPING>,
}

/// A configured transmit PDO: its COB-ID, object mapping, and trigger type.
#[derive(Debug)]
struct TpdoSlot {
    cob_id: u16,
    mapping: PdoMapping<MAX_PDO_MAPPING>,
    transmission: TransmissionType,
}

/// A CANopen device node: an object dictionary served over SDO, with NMT state,
/// heartbeat/boot-up production, and PDO exchange.
#[derive(Debug)]
pub struct Node<const N: usize> {
    node_id: NodeId,
    od: ObjectDictionary<N>,
    sdo: SdoServer,
    nmt: NmtStateMachine,
    rpdos: Vec<RpdoSlot, MAX_PDOS>,
    tpdos: Vec<TpdoSlot, MAX_PDOS>,
    lss: Option<LssSlave>,
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
            rpdos: Vec::new(),
            tpdos: Vec::new(),
            lss: None,
        }
    }

    /// Enable LSS with this node's 128-bit identity ([`LssAddress`], object
    /// `0x1018`). The node then answers LSS master requests on `0x7E5`, letting
    /// a master (re)assign its node-id over the bus.
    ///
    /// A node awaiting an LSS-assigned id should be left in
    /// [`NmtState::Initialising`] (do not call [`Node::boot`]) so it serves only
    /// LSS; after the id is assigned, call [`Node::apply_lss_node_id`] then boot.
    pub fn enable_lss(&mut self, address: LssAddress) {
        self.lss = Some(LssSlave::new(address, self.node_id.raw()));
    }

    /// Change the node-id, rebuilding the SDO server for the new COB-IDs. Call
    /// on the reset that follows an LSS reconfiguration.
    pub fn set_node_id(&mut self, node_id: NodeId) {
        self.node_id = node_id;
        self.sdo = SdoServer::new(node_id);
    }

    /// Adopt a node-id assigned over LSS: if the LSS slave holds a valid pending
    /// id, apply it (rebuilding the SDO server) and return it. Call this on the
    /// node's reset after an LSS configuration.
    pub fn apply_lss_node_id(&mut self) -> Option<NodeId> {
        let pending = self.lss.as_ref()?.pending_node_id();
        let node_id = NodeId::new(pending).ok()?;
        self.set_node_id(node_id);
        if let Some(lss) = &mut self.lss {
            lss.adopt_pending();
        }
        Some(node_id)
    }

    /// The LSS slave, if LSS is enabled (e.g. to read its pending node-id).
    pub fn lss(&self) -> Option<&LssSlave> {
        self.lss.as_ref()
    }

    /// Configure a receive PDO: when a frame arrives on `cob_id` (while
    /// operational), its bytes are unpacked into the mapped objects.
    ///
    /// Returns [`Error::MappingFull`] once [`MAX_PDOS`] receive PDOs are set.
    pub fn add_rpdo(&mut self, cob_id: u16, mapping: PdoMapping<MAX_PDO_MAPPING>) -> Result<()> {
        self.rpdos
            .push(RpdoSlot { cob_id, mapping })
            .map_err(|_| Error::MappingFull)
    }

    /// Configure a transmit PDO: [`Node::sync_tpdos`] packs and emits it on SYNC
    /// (for synchronous types) and [`Node::tpdo`] emits it on demand.
    ///
    /// Returns [`Error::MappingFull`] once [`MAX_PDOS`] transmit PDOs are set.
    pub fn add_tpdo(
        &mut self,
        cob_id: u16,
        mapping: PdoMapping<MAX_PDO_MAPPING>,
        transmission: TransmissionType,
    ) -> Result<()> {
        self.tpdos
            .push(TpdoSlot {
                cob_id,
                mapping,
                transmission,
            })
            .map_err(|_| Error::MappingFull)
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
    /// Handles NMT node-control (`0x000`), SDO requests (`0x600 + node`), LSS
    /// master requests (`0x7E5`, when enabled), and received PDOs. SDO is served
    /// only in pre-operational and operational states, and PDOs only in
    /// operational, per CiA 301; LSS is served regardless of NMT state. Frames
    /// for other COB-IDs are ignored.
    pub fn on_frame(&mut self, cob_id: u16, data: &[u8]) -> Option<TxFrame> {
        if cob_id == nmt::NMT_COMMAND_COB_ID {
            self.on_nmt(data);
            None
        } else if cob_id == lss::LSS_MASTER_COB_ID {
            self.on_lss(data)
        } else if cob_id == self.sdo.request_cob_id() {
            self.on_sdo(data)
        } else {
            self.on_rpdo(cob_id, data);
            None
        }
    }

    fn on_lss(&mut self, data: &[u8]) -> Option<TxFrame> {
        let lss = self.lss.as_mut()?;
        if data.len() > 8 {
            return None;
        }
        let mut frame: lss::LssFrame = [0u8; 8];
        frame[..data.len()].copy_from_slice(data);
        lss.handle(&frame)
            .map(|resp| TxFrame::new(lss::LSS_SLAVE_COB_ID, &resp))
    }

    /// The synchronous transmit PDOs to send in response to a SYNC.
    ///
    /// Packs every configured TPDO with a synchronous transmission type from the
    /// current object dictionary. Empty unless the node is operational — PDOs
    /// are exchanged only in that state (CiA 301 §7.3.5).
    pub fn sync_tpdos(&self) -> Vec<TxFrame, MAX_PDOS> {
        let mut frames = Vec::new();
        if self.nmt.state() != NmtState::Operational {
            return frames;
        }
        for slot in &self.tpdos {
            if is_synchronous(slot.transmission) {
                if let Some(frame) = self.build_tpdo(slot) {
                    // Capacity matches self.tpdos, so this never overflows.
                    let _ = frames.push(frame);
                }
            }
        }
        frames
    }

    /// Emit transmit PDO `index` on demand (an event-driven transmission), or
    /// `None` if there is no such PDO or the node is not operational.
    pub fn tpdo(&self, index: usize) -> Option<TxFrame> {
        if self.nmt.state() != NmtState::Operational {
            return None;
        }
        self.build_tpdo(self.tpdos.get(index)?)
    }

    fn build_tpdo(&self, slot: &TpdoSlot) -> Option<TxFrame> {
        if slot.mapping.is_empty() {
            return None;
        }
        let mut buf = [0u8; 8];
        let len = pdo::pack(&slot.mapping, &self.od, &mut buf).ok()?;
        Some(TxFrame::new(slot.cob_id, &buf[..len]))
    }

    fn on_rpdo(&mut self, cob_id: u16, data: &[u8]) {
        // PDOs are exchanged only in the operational state.
        if self.nmt.state() != NmtState::Operational {
            return;
        }
        if let Some(i) = self.rpdos.iter().position(|r| r.cob_id == cob_id) {
            // Disjoint field borrows: `rpdos` (shared) and `od` (mutable).
            let _ = pdo::unpack(&self.rpdos[i].mapping, &mut self.od, data);
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

/// Whether a transmission type is SYNC-triggered (as opposed to event-driven).
fn is_synchronous(transmission: TransmissionType) -> bool {
    matches!(
        transmission,
        TransmissionType::SynchronousAcyclic | TransmissionType::SynchronousCyclic(_)
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::object_dictionary::{Address, Entry};
    use crate::pdo::MappingEntry;
    use crate::sdo::{encode_download_expedited, encode_upload_request};
    use crate::{DataType, NmtCommand, Value};

    fn start(n: &mut Node<8>) {
        n.on_frame(
            nmt::NMT_COMMAND_COB_ID,
            &[NmtCommand::StartRemoteNode as u8, 0x10],
        );
    }

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

    // --- PDO ---------------------------------------------------------------
    fn pdo_od() -> ObjectDictionary<8> {
        let mut od = ObjectDictionary::new();
        // TPDO source objects (readable) and an RPDO target (writable).
        od.insert(
            Address::new(0x6000, 1),
            Entry::rw(Value::Unsigned16(0xBEEF)),
        )
        .unwrap();
        od.insert(Address::new(0x6000, 2), Entry::rw(Value::Unsigned8(0x42)))
            .unwrap();
        od.insert(Address::new(0x6200, 1), Entry::rw(Value::Unsigned16(0)))
            .unwrap();
        od
    }

    fn mapping(entries: &[(u16, u8, u8)]) -> PdoMapping<MAX_PDO_MAPPING> {
        let mut m = PdoMapping::new();
        for &(index, sub, bits) in entries {
            m.push(MappingEntry::new(index, sub, bits)).unwrap();
        }
        m
    }

    #[test]
    fn tpdo_transmits_only_when_operational() {
        let mut n = Node::new(NodeId::new(0x10).unwrap(), pdo_od());
        n.add_tpdo(
            0x18A,
            mapping(&[(0x6000, 1, 16), (0x6000, 2, 8)]),
            TransmissionType::SynchronousAcyclic,
        )
        .unwrap();
        n.boot();

        // Pre-operational: no PDO traffic.
        assert!(n.sync_tpdos().is_empty());

        start(&mut n);
        let frames = n.sync_tpdos();
        assert_eq!(frames.len(), 1);
        assert_eq!(frames[0].cob_id, 0x18A);
        // U16 0xBEEF little-endian then U8 0x42.
        assert_eq!(frames[0].data(), &[0xEF, 0xBE, 0x42]);
    }

    #[test]
    fn event_tpdo_by_index() {
        let mut n = Node::new(NodeId::new(0x10).unwrap(), pdo_od());
        // Event-driven type is not emitted by sync_tpdos, only by tpdo().
        n.add_tpdo(
            0x18A,
            mapping(&[(0x6000, 2, 8)]),
            TransmissionType::EventDrivenProfile,
        )
        .unwrap();
        n.boot();
        start(&mut n);
        assert!(n.sync_tpdos().is_empty());
        assert_eq!(n.tpdo(0).unwrap().data(), &[0x42]);
        assert!(n.tpdo(1).is_none());
    }

    #[test]
    fn rpdo_applies_only_when_operational() {
        let mut n = Node::new(NodeId::new(0x10).unwrap(), pdo_od());
        n.add_rpdo(0x20A, mapping(&[(0x6200, 1, 16)])).unwrap();
        n.boot();

        // Pre-operational: the RPDO is ignored.
        assert!(n.on_frame(0x20A, &[0x34, 0x12]).is_none());
        assert_eq!(
            n.od().read(Address::new(0x6200, 1)).unwrap(),
            Value::Unsigned16(0)
        );

        // Operational: the frame is unpacked into the object dictionary.
        start(&mut n);
        n.on_frame(0x20A, &[0x34, 0x12]);
        assert_eq!(
            n.od().read(Address::new(0x6200, 1)).unwrap(),
            Value::Unsigned16(0x1234)
        );
    }

    #[test]
    fn pdo_capacity_is_enforced() {
        let mut n = Node::new(NodeId::new(0x10).unwrap(), pdo_od());
        for _ in 0..MAX_PDOS {
            n.add_tpdo(
                0x18A,
                mapping(&[(0x6000, 2, 8)]),
                TransmissionType::SynchronousAcyclic,
            )
            .unwrap();
        }
        assert_eq!(
            n.add_tpdo(
                0x18A,
                mapping(&[(0x6000, 2, 8)]),
                TransmissionType::SynchronousAcyclic
            ),
            Err(Error::MappingFull)
        );
    }

    // --- LSS ---------------------------------------------------------------
    use crate::lss::{self, encode_configure_node_id, encode_switch_global, LssAddress, LssState};

    fn lss_address() -> LssAddress {
        LssAddress {
            vendor_id: 0x1F,
            product_code: 0x2A,
            revision_number: 1,
            serial_number: 0x99,
        }
    }

    #[test]
    fn routes_lss_frames_when_enabled() {
        let mut n = node();
        n.enable_lss(lss_address());
        // Switch into configuration via LSS (COB-ID 0x7E5), no response.
        assert!(n
            .on_frame(lss::LSS_MASTER_COB_ID, &encode_switch_global(true))
            .is_none());
        assert_eq!(n.lss().unwrap().state(), LssState::Configuration);
    }

    #[test]
    fn lss_frames_ignored_when_disabled() {
        let mut n = node(); // LSS not enabled
        assert!(n
            .on_frame(lss::LSS_MASTER_COB_ID, &encode_switch_global(true))
            .is_none());
        assert!(n.lss().is_none());
    }

    #[test]
    fn lss_assigns_node_id_and_moves_sdo_cob_id() {
        // A node that comes up unconfigured: leave it in Initialising and serve
        // only LSS until a master assigns an id.
        let mut n = Node::new(NodeId::new(1).unwrap(), od());
        n.enable_lss(lss_address());
        assert_eq!(n.node_id(), NodeId::new(1).unwrap());

        // Master: switch to configuration, then assign node-id 0x20.
        n.on_frame(lss::LSS_MASTER_COB_ID, &encode_switch_global(true));
        let resp = n
            .on_frame(lss::LSS_MASTER_COB_ID, &encode_configure_node_id(0x20))
            .expect("configure response");
        assert_eq!(resp.cob_id, lss::LSS_SLAVE_COB_ID);
        assert_eq!(&resp.data()[..2], &[0x11, 0x00]); // configure success

        // On the node's reset, adopt the assigned id — SDO COB-ID moves.
        assert_eq!(n.apply_lss_node_id(), Some(NodeId::new(0x20).unwrap()));
        assert_eq!(n.node_id(), NodeId::new(0x20).unwrap());

        n.boot();
        let req = encode_upload_request(Address::new(0x1000, 0));
        assert!(n.on_frame(0x601, &req).is_none()); // old COB-ID no longer served
        assert!(n.on_frame(0x620, &req).is_some()); // new COB-ID (0x600 + 0x20)
    }
}
