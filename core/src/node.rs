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

use crate::datatypes::Value;
use crate::emcy::{self, EmergencyMessage, ErrorRegister};
use crate::lss::{self, LssAddress, LssSlave};
use crate::nmt::{self, NmtState, NmtStateMachine};
use crate::object_dictionary::{Address, ObjectDictionary};
use crate::pdo::{self, PdoMapping, TransmissionType};
use crate::sdo::{self, SdoServer};
use crate::sync::{self, SyncCounter};
use crate::types::NodeId;
use crate::{Error, Result};

/// The maximum number of transmit (or receive) PDOs a [`Node`] holds — the four
/// of the predefined connection set.
pub const MAX_PDOS: usize = 4;

/// The maximum objects mapped into one PDO: a full eight-byte frame of
/// one-byte objects.
pub const MAX_PDO_MAPPING: usize = 8;

/// How many past emergencies a [`Node`] keeps in its pre-defined error field
/// (object `0x1003`), most-recent-first. Older entries are dropped once full.
pub const MAX_ERROR_HISTORY: usize = 8;

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
    guard_toggle: bool,
    error_register: u8,
    error_history: Vec<u32, MAX_ERROR_HISTORY>,
    sync_producer: Option<SyncCounter>,
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
            guard_toggle: false,
            error_register: 0,
            error_history: Vec::new(),
            sync_producer: None,
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

    /// Enable LSS on a node that comes up **unconfigured** — it has no node-id
    /// yet, so it answers *only* LSS (including [`FastscanMaster`] discovery,
    /// when the master doesn't know its address) and serves no SDO, NMT, or PDO
    /// traffic until a master assigns an id. Construct the node with any
    /// placeholder id, call this, and leave it in [`NmtState::Initialising`]
    /// (do not [`boot`](Node::boot)); once the id is assigned over LSS, call
    /// [`Node::apply_lss_node_id`] on the node's reset, then boot.
    ///
    /// [`FastscanMaster`]: crate::lss::FastscanMaster
    pub fn enable_lss_unconfigured(&mut self, address: LssAddress) {
        self.lss = Some(LssSlave::new(address, lss::UNCONFIGURED_NODE_ID));
    }

    /// Whether LSS is enabled and the node is still unconfigured (no node-id),
    /// so it should serve only LSS.
    fn lss_unconfigured(&self) -> bool {
        matches!(&self.lss, Some(l) if l.node_id() == lss::UNCONFIGURED_NODE_ID)
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

    /// (Re)build the PDO configuration from the PDO parameter objects in the
    /// object dictionary — communication parameters at `0x1400`+ (RPDO) /
    /// `0x1800`+ (TPDO) and mapping parameters at `0x1600`+ / `0x1A00`+ (CiA 301
    /// §7.5.2). Call this after a master configures PDOs by writing those
    /// objects over SDO, so the node picks up the new layout.
    ///
    /// Any programmatic PDO configuration is replaced. PDOs whose communication
    /// COB-ID has the "invalid" bit set are skipped, as are entries that are
    /// absent or malformed.
    pub fn configure_pdos_from_od(&mut self) {
        self.rpdos.clear();
        self.tpdos.clear();
        for n in 0..MAX_PDOS as u16 {
            if let Some(slot) = build_rpdo(&self.od, n) {
                let _ = self.rpdos.push(slot);
            }
            if let Some(slot) = build_tpdo(&self.od, n) {
                let _ = self.tpdos.push(slot);
            }
        }
    }

    /// Take the address of the object a master most recently wrote over SDO,
    /// clearing it. Poll after [`Node::on_frame`] to react to configuration
    /// writes — for example, re-read the PDO layout when a PDO parameter object
    /// (`0x1400`–`0x1BFF`) changes:
    ///
    /// ```
    /// # use canopen_rs::{node::Node, NodeId, ObjectDictionary};
    /// # let mut node = Node::new(NodeId::new(1).unwrap(), ObjectDictionary::<8>::new());
    /// # let (cob_id, data) = (0u16, [0u8; 8]);
    /// node.on_frame(cob_id, &data);
    /// if let Some(addr) = node.take_written_object() {
    ///     if (0x1400..=0x1BFF).contains(&addr.index) {
    ///         node.configure_pdos_from_od();
    ///     }
    /// }
    /// ```
    pub fn take_written_object(&mut self) -> Option<crate::object_dictionary::Address> {
        self.sdo.take_write()
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
        self.guard_toggle = false;
        TxFrame::new(nmt::heartbeat_cob_id(self.node_id), &nmt::BOOTUP_FRAME)
    }

    /// The node-guarding response frame (`0x700 + node`), reporting the current
    /// state with an alternating toggle bit. Call this when the master polls the
    /// node with an RTR frame on that COB-ID (the legacy error-control
    /// alternative to [`Node::heartbeat`]).
    pub fn node_guard_response(&mut self) -> TxFrame {
        let byte = nmt::encode_node_guard(self.nmt.state(), self.guard_toggle);
        self.guard_toggle = !self.guard_toggle;
        TxFrame::new(nmt::heartbeat_cob_id(self.node_id), &[byte])
    }

    /// The heartbeat frame for the current state. Transmit it on your heartbeat
    /// timer (the producer heartbeat time lives in object `0x1017`).
    pub fn heartbeat(&self) -> TxFrame {
        TxFrame::new(
            nmt::heartbeat_cob_id(self.node_id),
            &nmt::encode_heartbeat(self.nmt.state()),
        )
    }

    /// The current error register (object `0x1001`): the bitfield of active
    /// error classes.
    pub fn error_register(&self) -> ErrorRegister {
        ErrorRegister(self.error_register)
    }

    /// The pre-defined error field (object `0x1003`): recorded emergency codes,
    /// most-recent-first (up to [`MAX_ERROR_HISTORY`]). The low 16 bits of each
    /// entry are the emergency error code.
    pub fn error_history(&self) -> &[u32] {
        &self.error_history
    }

    /// Raise an emergency: set `register` (and the generic bit) in the error
    /// register, record `code` at the front of the pre-defined error field,
    /// mirror both into the object dictionary (`0x1001` / `0x1003`) when those
    /// objects exist, and return the EMCY frame to transmit on `0x080 + node`.
    ///
    /// `info` carries five manufacturer-specific bytes in the frame. The
    /// returned frame must still be transmitted by the caller (sans-I/O).
    pub fn raise_emergency(
        &mut self,
        code: u16,
        register: ErrorRegister,
        info: [u8; 5],
    ) -> TxFrame {
        // Any active error sets the generic bit (CiA 301 §7.5.2.2).
        self.error_register |= register.0 | ErrorRegister::GENERIC;

        // Record most-recent-first, dropping the oldest once full.
        if self.error_history.is_full() {
            self.error_history.pop();
        }
        let len = self.error_history.len();
        let _ = self.error_history.push(0); // room guaranteed by the pop above
        for i in (0..len).rev() {
            self.error_history[i + 1] = self.error_history[i];
        }
        self.error_history[0] = u32::from(code);

        self.mirror_error_objects();
        let msg = EmergencyMessage::new(code, ErrorRegister(self.error_register), info);
        TxFrame::new(emcy::emcy_cob_id(self.node_id), &msg.encode())
    }

    /// Clear all errors: reset the error register and pre-defined error field,
    /// mirror the cleared state into the object dictionary, and return an
    /// *error-reset* EMCY frame (code `0x0000`) to transmit.
    pub fn clear_errors(&mut self) -> TxFrame {
        self.error_register = 0;
        self.error_history.clear();
        self.mirror_error_objects();
        TxFrame::new(
            emcy::emcy_cob_id(self.node_id),
            &EmergencyMessage::error_reset().encode(),
        )
    }

    /// Mirror the error register and pre-defined error field into the object
    /// dictionary so a master can read them over SDO. Best effort: absent (or
    /// wrongly-typed) objects are simply skipped, so a node that does not model
    /// `0x1001` / `0x1003` still works.
    fn mirror_error_objects(&mut self) {
        let _ = self.od.set(
            Address::new(0x1001, 0),
            Value::Unsigned8(self.error_register),
        );
        let count = self.error_history.len() as u8;
        let _ = self
            .od
            .set(Address::new(0x1003, 0), Value::Unsigned8(count));
        for (i, &entry) in self.error_history.iter().enumerate() {
            let sub = (i + 1) as u8;
            let _ = self
                .od
                .set(Address::new(0x1003, sub), Value::Unsigned32(entry));
        }
    }

    /// Configure this node as the network **SYNC producer**, with the given
    /// synchronous-counter-overflow value (object `0x1019`): `0` produces
    /// counter-less (empty) SYNC frames, and `2..=240` a counting SYNC.
    ///
    /// Returns [`Error::InvalidSyncCounter`] for the reserved value `1` or any
    /// value above `240`.
    pub fn enable_sync_producer(&mut self, counter_overflow: u8) -> Result<()> {
        self.sync_producer = Some(SyncCounter::new(counter_overflow)?);
        Ok(())
    }

    /// Produce the next SYNC frame to broadcast on [`SYNC_COB_ID`](crate::sync::SYNC_COB_ID),
    /// advancing the counter when one is configured. Returns `None` unless the
    /// node is a SYNC producer (see [`enable_sync_producer`](Node::enable_sync_producer)).
    ///
    /// Call this on your SYNC-period timer. SYNC is a broadcast network service,
    /// so it is emitted independently of this node's own NMT state; gate the
    /// call on the network state yourself if you only want SYNC while
    /// operational.
    pub fn produce_sync(&mut self) -> Option<TxFrame> {
        let counter = self.sync_producer.as_mut()?;
        Some(match counter.advance() {
            Some(count) => TxFrame::new(sync::SYNC_COB_ID, &sync::encode_counter(count)),
            None => TxFrame::new(sync::SYNC_COB_ID, &[]),
        })
    }

    /// Process an incoming CAN frame, returning a response to transmit, if any.
    ///
    /// Handles NMT node-control (`0x000`), SDO requests (`0x600 + node`), LSS
    /// master requests (`0x7E5`, when enabled), and received PDOs. SDO is served
    /// only in pre-operational and operational states, and PDOs only in
    /// operational, per CiA 301; LSS is served regardless of NMT state. Frames
    /// for other COB-IDs are ignored.
    ///
    /// While the node is LSS-unconfigured (see [`Node::enable_lss_unconfigured`])
    /// it serves *only* LSS — everything else is ignored until a node-id is
    /// assigned — since its data COB-IDs are not yet meaningful.
    pub fn on_frame(&mut self, cob_id: u16, data: &[u8]) -> Option<TxFrame> {
        if cob_id == lss::LSS_MASTER_COB_ID {
            return self.on_lss(data);
        }
        if self.lss_unconfigured() {
            return None; // no node-id yet: LSS only
        }
        if cob_id == nmt::NMT_COMMAND_COB_ID {
            self.on_nmt(data);
            None
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

// --- Reading PDO parameter objects out of the object dictionary -------------

fn od_u32<const N: usize>(od: &ObjectDictionary<N>, index: u16, sub: u8) -> Option<u32> {
    match od
        .read(crate::object_dictionary::Address::new(index, sub))
        .ok()?
    {
        crate::datatypes::Value::Unsigned32(v) => Some(v),
        _ => None,
    }
}

fn od_u8<const N: usize>(od: &ObjectDictionary<N>, index: u16, sub: u8) -> Option<u8> {
    match od
        .read(crate::object_dictionary::Address::new(index, sub))
        .ok()?
    {
        crate::datatypes::Value::Unsigned8(v) => Some(v),
        _ => None,
    }
}

/// Read a mapping-parameter record (`sub 0` = count, `sub 1..=count` = the
/// `0xIIII_SSLL` mapping entries) into a [`PdoMapping`].
fn read_pdo_mapping<const N: usize>(
    od: &ObjectDictionary<N>,
    map_index: u16,
) -> Option<PdoMapping<MAX_PDO_MAPPING>> {
    let count = od_u8(od, map_index, 0)?;
    let mut mapping = PdoMapping::new();
    for sub in 1..=count {
        mapping
            .push(pdo::MappingEntry::from_u32(od_u32(od, map_index, sub)?))
            .ok()?;
    }
    Some(mapping)
}

fn build_rpdo<const N: usize>(od: &ObjectDictionary<N>, n: u16) -> Option<RpdoSlot> {
    let cob_id = od_u32(od, 0x1400 + n, 1)?;
    if !pdo::pdo_is_valid(cob_id) {
        return None;
    }
    Some(RpdoSlot {
        cob_id: pdo::pdo_can_id(cob_id),
        mapping: read_pdo_mapping(od, 0x1600 + n)?,
    })
}

fn build_tpdo<const N: usize>(od: &ObjectDictionary<N>, n: u16) -> Option<TpdoSlot> {
    let cob_id = od_u32(od, 0x1800 + n, 1)?;
    if !pdo::pdo_is_valid(cob_id) {
        return None;
    }
    // Transmission type (comm sub 2); default to event-driven if absent/invalid.
    let transmission = od_u8(od, 0x1800 + n, 2)
        .and_then(|b| TransmissionType::from_byte(b).ok())
        .unwrap_or(TransmissionType::EventDrivenProfile);
    Some(TpdoSlot {
        cob_id: pdo::pdo_can_id(cob_id),
        mapping: read_pdo_mapping(od, 0x1A00 + n)?,
        transmission,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::object_dictionary::{Address, Entry};
    use crate::pdo::MappingEntry;
    use crate::sdo::{encode_download_expedited, encode_upload_request};
    use crate::{DataType, NmtCommand, Value};

    fn start<const N: usize>(n: &mut Node<N>) {
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
    fn node_guard_response_toggles() {
        let mut n = node();
        n.boot();
        start(&mut n); // operational (0x05)
        let first = n.node_guard_response();
        assert_eq!(first.cob_id, 0x710); // 0x700 + node
        assert_eq!(first.data(), &[0x05]); // toggle clear
        assert_eq!(n.node_guard_response().data(), &[0x85]); // toggle set
        assert_eq!(n.node_guard_response().data(), &[0x05]); // toggles back
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

    #[test]
    fn unconfigured_node_serves_only_lss() {
        let mut n = Node::new(NodeId::new(1).unwrap(), od());
        n.enable_lss_unconfigured(lss_address());
        n.boot(); // even booted, an unconfigured node serves no data traffic

        // The placeholder id's SDO COB-ID is not served while unconfigured.
        let req = encode_upload_request(Address::new(0x1000, 0));
        assert!(n.on_frame(0x601, &req).is_none());

        // ...but LSS still is.
        n.on_frame(lss::LSS_MASTER_COB_ID, &encode_switch_global(true));
        assert_eq!(n.lss().unwrap().state(), LssState::Configuration);
    }

    #[test]
    fn fastscan_discovers_then_configures_node_via_on_frame() {
        use crate::lss::FastscanMaster;
        let addr = lss_address();
        let mut n = Node::new(NodeId::new(1).unwrap(), od());
        n.enable_lss_unconfigured(addr);

        // The master discovers the unknown address purely through on_frame.
        let mut master = FastscanMaster::new();
        let mut steps = 0;
        while let Some(req) = master.next_request() {
            let answered = n.on_frame(lss::LSS_MASTER_COB_ID, &req).is_some();
            master.on_response(answered);
            steps += 1;
            assert!(steps < 200, "fastscan did not converge");
        }
        assert!(master.found());
        assert_eq!(master.address(), addr);
        assert_eq!(n.lss().unwrap().state(), LssState::Configuration);

        // Assign an id; the node then serves SDO on the new COB-ID.
        n.on_frame(lss::LSS_MASTER_COB_ID, &encode_configure_node_id(0x33));
        assert_eq!(n.apply_lss_node_id(), Some(NodeId::new(0x33).unwrap()));
        n.boot();
        let req = encode_upload_request(Address::new(0x1000, 0));
        assert!(n.on_frame(0x600 + 0x33, &req).is_some());
    }

    // --- PDO configuration from the object dictionary ----------------------
    #[test]
    fn configures_pdos_from_od() {
        let mut od = ObjectDictionary::<16>::new();
        // RPDO1: comm 0x1400 (COB-ID 0x210, valid), mapping 0x1600 -> 0x6200/1.
        od.insert(Address::new(0x1400, 1), Entry::rw(Value::Unsigned32(0x210)))
            .unwrap();
        od.insert(Address::new(0x1600, 0), Entry::rw(Value::Unsigned8(1)))
            .unwrap();
        od.insert(
            Address::new(0x1600, 1),
            Entry::rw(Value::Unsigned32(MappingEntry::new(0x6200, 1, 16).to_u32())),
        )
        .unwrap();
        // TPDO1: comm 0x1800 (COB-ID 0x190, valid, sync-cyclic), mapping -> 0x6000/1.
        od.insert(Address::new(0x1800, 1), Entry::rw(Value::Unsigned32(0x190)))
            .unwrap();
        od.insert(Address::new(0x1800, 2), Entry::rw(Value::Unsigned8(1)))
            .unwrap(); // transmission type 1
        od.insert(Address::new(0x1A00, 0), Entry::rw(Value::Unsigned8(1)))
            .unwrap();
        od.insert(
            Address::new(0x1A00, 1),
            Entry::rw(Value::Unsigned32(MappingEntry::new(0x6000, 1, 16).to_u32())),
        )
        .unwrap();
        // The mapped process-data objects.
        od.insert(Address::new(0x6200, 1), Entry::rw(Value::Unsigned16(0)))
            .unwrap();
        od.insert(
            Address::new(0x6000, 1),
            Entry::rw(Value::Unsigned16(0xBEEF)),
        )
        .unwrap();

        let mut n = Node::new(NodeId::new(0x10).unwrap(), od);
        n.configure_pdos_from_od();
        n.boot();
        start(&mut n);

        // The RPDO now applies to the OD…
        n.on_frame(0x210, &[0x34, 0x12]);
        assert_eq!(
            n.od().read(Address::new(0x6200, 1)).unwrap(),
            Value::Unsigned16(0x1234)
        );
        // …and the (synchronous) TPDO is emitted on SYNC.
        let tpdos = n.sync_tpdos();
        assert_eq!(tpdos.len(), 1);
        assert_eq!(tpdos[0].cob_id, 0x190);
        assert_eq!(tpdos[0].data(), &[0xEF, 0xBE]);
    }

    #[test]
    fn skips_pdo_with_invalid_cob_id() {
        let mut od = ObjectDictionary::<8>::new();
        // TPDO1 with the COB-ID validity bit (31) set — disabled.
        od.insert(
            Address::new(0x1800, 1),
            Entry::rw(Value::Unsigned32(0x8000_0190)),
        )
        .unwrap();
        od.insert(Address::new(0x1800, 2), Entry::rw(Value::Unsigned8(1)))
            .unwrap();
        od.insert(Address::new(0x1A00, 0), Entry::rw(Value::Unsigned8(1)))
            .unwrap();
        od.insert(
            Address::new(0x1A00, 1),
            Entry::rw(Value::Unsigned32(MappingEntry::new(0x6000, 1, 16).to_u32())),
        )
        .unwrap();
        od.insert(
            Address::new(0x6000, 1),
            Entry::rw(Value::Unsigned16(0xBEEF)),
        )
        .unwrap();

        let mut n = Node::new(NodeId::new(0x10).unwrap(), od);
        n.configure_pdos_from_od();
        n.boot();
        start(&mut n);
        assert!(n.sync_tpdos().is_empty()); // the invalid TPDO was not configured
    }

    #[test]
    fn reacts_to_a_pdo_parameter_write() {
        // TPDO1 params present but initially disabled (COB-ID invalid bit set).
        let mut od = ObjectDictionary::<16>::new();
        od.insert(
            Address::new(0x1800, 1),
            Entry::rw(Value::Unsigned32(0x8000_0190)),
        )
        .unwrap();
        od.insert(Address::new(0x1800, 2), Entry::rw(Value::Unsigned8(1)))
            .unwrap();
        od.insert(Address::new(0x1A00, 0), Entry::rw(Value::Unsigned8(1)))
            .unwrap();
        od.insert(
            Address::new(0x1A00, 1),
            Entry::rw(Value::Unsigned32(MappingEntry::new(0x6000, 1, 16).to_u32())),
        )
        .unwrap();
        od.insert(
            Address::new(0x6000, 1),
            Entry::rw(Value::Unsigned16(0xBEEF)),
        )
        .unwrap();

        let mut n = Node::new(NodeId::new(0x10).unwrap(), od);
        n.configure_pdos_from_od();
        n.boot();
        start(&mut n);
        assert!(n.sync_tpdos().is_empty());

        // A master enables the TPDO by writing a valid COB-ID to 0x1800/1.
        let req =
            encode_download_expedited(Address::new(0x1800, 1), &Value::Unsigned32(0x190)).unwrap();
        n.on_frame(0x610, &req);

        // The node notices the PDO-parameter write and reconfigures.
        let written = n.take_written_object().expect("a write was recorded");
        assert_eq!(written, Address::new(0x1800, 1));
        assert_eq!(n.take_written_object(), None); // reported once
        if (0x1400..=0x1BFF).contains(&written.index) {
            n.configure_pdos_from_od();
        }

        // The TPDO is now active.
        let tpdos = n.sync_tpdos();
        assert_eq!(tpdos.len(), 1);
        assert_eq!(tpdos[0].cob_id, 0x190);
    }

    // --- Emergencies (EMCY + error register / pre-defined error field) ------
    fn od_with_error_objects() -> ObjectDictionary<8> {
        let mut od = ObjectDictionary::new();
        od.insert(Address::new(0x1001, 0), Entry::ro(Value::Unsigned8(0)))
            .unwrap();
        od.insert(Address::new(0x1003, 0), Entry::ro(Value::Unsigned8(0)))
            .unwrap();
        od.insert(Address::new(0x1003, 1), Entry::ro(Value::Unsigned32(0)))
            .unwrap();
        od.insert(Address::new(0x1003, 2), Entry::ro(Value::Unsigned32(0)))
            .unwrap();
        od
    }

    #[test]
    fn emergency_sets_register_and_emits_frame() {
        let mut n = node();
        let tx = n.raise_emergency(
            emcy::error_code::VOLTAGE,
            ErrorRegister(ErrorRegister::VOLTAGE),
            [0xAA, 0xBB, 0xCC, 0xDD, 0xEE],
        );
        // The generic bit is set alongside the class bit.
        assert_eq!(
            n.error_register(),
            ErrorRegister(ErrorRegister::GENERIC | ErrorRegister::VOLTAGE)
        );
        assert_eq!(tx.cob_id, 0x090); // 0x080 + node 0x10
        let msg = EmergencyMessage::decode(tx.data()).unwrap();
        assert_eq!(msg.error_code, emcy::error_code::VOLTAGE);
        assert_eq!(
            msg.error_register.0,
            ErrorRegister::GENERIC | ErrorRegister::VOLTAGE
        );
        assert_eq!(msg.vendor_specific, [0xAA, 0xBB, 0xCC, 0xDD, 0xEE]);
        assert_eq!(n.error_history(), &[u32::from(emcy::error_code::VOLTAGE)]);
    }

    #[test]
    fn error_history_is_most_recent_first_and_bounded() {
        let mut n = node();
        let total = MAX_ERROR_HISTORY as u16 + 2;
        for i in 0..total {
            n.raise_emergency(0x1000 + i, ErrorRegister::NONE, [0; 5]);
        }
        assert_eq!(n.error_history().len(), MAX_ERROR_HISTORY);
        assert_eq!(n.error_history()[0], u32::from(0x1000 + total - 1)); // newest first
        let oldest_kept = 0x1000 + total - MAX_ERROR_HISTORY as u16;
        assert_eq!(*n.error_history().last().unwrap(), u32::from(oldest_kept));
    }

    #[test]
    fn emergency_mirrors_into_object_dictionary() {
        let mut n = Node::new(NodeId::new(0x10).unwrap(), od_with_error_objects());
        n.raise_emergency(0x5530, ErrorRegister(ErrorRegister::DEVICE_PROFILE), [0; 5]);

        assert_eq!(
            n.od().read(Address::new(0x1001, 0)).unwrap(),
            Value::Unsigned8(ErrorRegister::GENERIC | ErrorRegister::DEVICE_PROFILE)
        );
        assert_eq!(
            n.od().read(Address::new(0x1003, 0)).unwrap(),
            Value::Unsigned8(1)
        );
        assert_eq!(
            n.od().read(Address::new(0x1003, 1)).unwrap(),
            Value::Unsigned32(0x5530)
        );
    }

    #[test]
    fn clear_errors_resets_register_history_and_od() {
        let mut n = Node::new(NodeId::new(0x10).unwrap(), od_with_error_objects());
        n.raise_emergency(
            0x3210,
            ErrorRegister(ErrorRegister::VOLTAGE),
            [1, 2, 3, 4, 5],
        );

        let tx = n.clear_errors();
        assert_eq!(n.error_register(), ErrorRegister::NONE);
        assert!(n.error_history().is_empty());
        let msg = EmergencyMessage::decode(tx.data()).unwrap();
        assert!(msg.is_error_reset());
        assert_eq!(tx.cob_id, 0x090);
        assert_eq!(
            n.od().read(Address::new(0x1001, 0)).unwrap(),
            Value::Unsigned8(0)
        );
        assert_eq!(
            n.od().read(Address::new(0x1003, 0)).unwrap(),
            Value::Unsigned8(0)
        );
    }

    #[test]
    fn emergency_without_error_objects_still_works() {
        // node()'s OD has no 0x1001/0x1003 — mirroring is best-effort and must
        // neither panic nor suppress the frame.
        let mut n = node();
        let tx = n.raise_emergency(0x8130, ErrorRegister(ErrorRegister::COMMUNICATION), [0; 5]);
        assert_eq!(tx.cob_id, 0x090);
        assert_eq!(
            n.error_register(),
            ErrorRegister(ErrorRegister::GENERIC | ErrorRegister::COMMUNICATION)
        );
    }

    // --- SYNC producer -----------------------------------------------------
    #[test]
    fn sync_production_is_off_by_default() {
        let mut n = node();
        assert!(n.produce_sync().is_none());
    }

    #[test]
    fn counterless_sync_producer_emits_empty_frames() {
        let mut n = node();
        n.enable_sync_producer(0).unwrap();
        let tx = n.produce_sync().expect("a SYNC frame");
        assert_eq!(tx.cob_id, sync::SYNC_COB_ID); // 0x080
        assert_eq!(tx.data(), &[] as &[u8]); // counter-less
    }

    #[test]
    fn counting_sync_producer_cycles_and_wraps() {
        let mut n = node();
        n.enable_sync_producer(3).unwrap();
        let counts: [&[u8]; 4] = [&[1], &[2], &[3], &[1]];
        for expected in counts {
            let tx = n.produce_sync().unwrap();
            assert_eq!(tx.cob_id, sync::SYNC_COB_ID);
            assert_eq!(tx.data(), expected);
        }
    }

    #[test]
    fn enable_sync_producer_rejects_reserved_overflow() {
        let mut n = node();
        assert_eq!(n.enable_sync_producer(1), Err(Error::InvalidSyncCounter));
        assert!(n.produce_sync().is_none()); // still not a producer
    }
}
