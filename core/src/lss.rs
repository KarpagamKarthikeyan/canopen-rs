//! Layer Setting Services (LSS, CiA 305) — configure a node's node-id and bit
//! timing over the bus.
//!
//! A node that ships without a preset node-id comes up *unconfigured* and
//! answers only LSS. An LSS master switches it into the *configuration* state —
//! globally, or selectively by matching its 128-bit [`LssAddress`] (the object
//! `0x1018` identity) — then assigns a node-id and asks it to store the setting.
//! When the master does not know the address, [`FastscanMaster`] discovers it
//! by bisecting the identity bit by bit (CiA 305 §3.7).
//!
//! LSS uses two fixed COB-IDs: master→slave on [`LSS_MASTER_COB_ID`] (`0x7E5`)
//! and slave→master on [`LSS_SLAVE_COB_ID`] (`0x7E4`). Every frame is eight
//! bytes: a command specifier in byte 0, then service data.
//!
//! [`LssSlave`] is the node-side state machine, sans-I/O like the rest of the
//! stack; the `encode_*` / `decode_*` functions build and read the master's
//! frames.
//!
//! ```
//! use canopen_rs::lss::{encode_configure_node_id, encode_switch_global, LssAddress, LssSlave, LssState};
//!
//! let address = LssAddress { vendor_id: 0x1F, product_code: 0x2A, revision_number: 1, serial_number: 0x1234 };
//! let mut slave = LssSlave::new(address, 0xFF); // 0xFF = unconfigured
//!
//! // Master switches all nodes into configuration, then assigns node-id 0x20.
//! slave.handle(&encode_switch_global(true));
//! assert_eq!(slave.state(), LssState::Configuration);
//!
//! let response = slave.handle(&encode_configure_node_id(0x20)).unwrap();
//! assert_eq!(response, [0x11, 0x00, 0, 0, 0, 0, 0, 0]); // success
//! slave.adopt_pending(); // applied on the node's next reset
//! assert_eq!(slave.node_id(), 0x20);
//! ```

/// The LSS master→slave request channel (`0x7E5`).
pub const LSS_MASTER_COB_ID: u16 = 0x7E5;
/// The LSS slave→master response channel (`0x7E4`).
pub const LSS_SLAVE_COB_ID: u16 = 0x7E4;

/// The eight-byte LSS message data field.
pub type LssFrame = [u8; 8];

/// The node-id value denoting an unconfigured node.
pub const UNCONFIGURED_NODE_ID: u8 = 0xFF;

// --- Command specifiers (byte 0) -------------------------------------------
const CS_SWITCH_GLOBAL: u8 = 0x04;
const CS_CONFIGURE_NODE_ID: u8 = 0x11;
const CS_STORE_CONFIGURATION: u8 = 0x17;
const CS_SWITCH_SELECTIVE_VENDOR: u8 = 0x40;
const CS_SWITCH_SELECTIVE_PRODUCT: u8 = 0x41;
const CS_SWITCH_SELECTIVE_REVISION: u8 = 0x42;
const CS_SWITCH_SELECTIVE_SERIAL: u8 = 0x43;
const CS_SWITCH_SELECTIVE_RESPONSE: u8 = 0x44;
const CS_INQUIRE_VENDOR: u8 = 0x5A;
const CS_INQUIRE_PRODUCT: u8 = 0x5B;
const CS_INQUIRE_REVISION: u8 = 0x5C;
const CS_INQUIRE_SERIAL: u8 = 0x5D;
const CS_INQUIRE_NODE_ID: u8 = 0x5E;
const CS_IDENTIFY_SLAVE: u8 = 0x4F;
const CS_FASTSCAN: u8 = 0x51;

// Switch-state-global modes (byte 1).
const MODE_WAITING: u8 = 0;
const MODE_CONFIGURATION: u8 = 1;

/// The `BitCheck` value (byte 5) that resets/starts a Fastscan cycle: no bits
/// are checked, so every unconfigured node answers. Bit checks otherwise run
/// `31..=0`.
pub const FASTSCAN_INIT: u8 = 32;

/// A node's 128-bit LSS address — the identity object (`0x1018`) that
/// uniquely selects it on the bus.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LssAddress {
    /// Vendor id (`0x1018` sub 1).
    pub vendor_id: u32,
    /// Product code (`0x1018` sub 2).
    pub product_code: u32,
    /// Revision number (`0x1018` sub 3).
    pub revision_number: u32,
    /// Serial number (`0x1018` sub 4).
    pub serial_number: u32,
}

/// The LSS state of a node.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LssState {
    /// Waiting — ignores configuration services until switched.
    Waiting,
    /// Configuration — accepts node-id/bit-timing/store services.
    Configuration,
}

fn frame(cs: u8) -> LssFrame {
    let mut f = [0u8; 8];
    f[0] = cs;
    f
}

fn frame_u32(cs: u8, value: u32) -> LssFrame {
    let mut f = frame(cs);
    f[1..5].copy_from_slice(&value.to_le_bytes());
    f
}

fn u32_at(f: &LssFrame) -> u32 {
    u32::from_le_bytes([f[1], f[2], f[3], f[4]])
}

// --- Master-side encoders --------------------------------------------------

/// Switch every node globally to the configuration (`true`) or waiting
/// (`false`) state.
pub fn encode_switch_global(configuration: bool) -> LssFrame {
    let mut f = frame(CS_SWITCH_GLOBAL);
    f[1] = if configuration {
        MODE_CONFIGURATION
    } else {
        MODE_WAITING
    };
    f
}

/// The four-message sequence that selectively switches the node matching
/// `address` into the configuration state.
pub fn encode_switch_selective(address: &LssAddress) -> [LssFrame; 4] {
    [
        frame_u32(CS_SWITCH_SELECTIVE_VENDOR, address.vendor_id),
        frame_u32(CS_SWITCH_SELECTIVE_PRODUCT, address.product_code),
        frame_u32(CS_SWITCH_SELECTIVE_REVISION, address.revision_number),
        frame_u32(CS_SWITCH_SELECTIVE_SERIAL, address.serial_number),
    ]
}

/// Assign `node_id` to the node currently in the configuration state.
pub fn encode_configure_node_id(node_id: u8) -> LssFrame {
    let mut f = frame(CS_CONFIGURE_NODE_ID);
    f[1] = node_id;
    f
}

/// Ask the configured node to store its configuration to non-volatile memory.
pub fn encode_store() -> LssFrame {
    frame(CS_STORE_CONFIGURATION)
}

/// Inquire the configured node's active node-id.
pub fn encode_inquire_node_id() -> LssFrame {
    frame(CS_INQUIRE_NODE_ID)
}

// --- Master-side response decoders -----------------------------------------

/// Whether `response` is a selective-switch confirmation (the node entered the
/// configuration state).
pub fn is_switch_selective_response(response: &LssFrame) -> bool {
    response[0] == CS_SWITCH_SELECTIVE_RESPONSE
}

/// Decode a configure-node-id response: `Ok(())` on success, `Err(error_code)`
/// otherwise (`1` = node-id out of range). `None` if not such a response.
pub fn decode_configure_node_id_response(response: &LssFrame) -> Option<Result<(), u8>> {
    if response[0] != CS_CONFIGURE_NODE_ID {
        return None;
    }
    Some(if response[1] == 0 {
        Ok(())
    } else {
        Err(response[1])
    })
}

/// Decode an inquire-node-id response into the reported node-id.
pub fn decode_inquire_node_id_response(response: &LssFrame) -> Option<u8> {
    (response[0] == CS_INQUIRE_NODE_ID).then_some(response[1])
}

// --- Fastscan (identify an unknown node by bisecting its address) -----------

/// Encode an LSS Fastscan request (CiA 305 §3.7).
///
/// `id_number` is the candidate value being tested, `bit_check` the bit being
/// probed ([`FASTSCAN_INIT`] to start; then `31..=0`), `lss_sub` which of the
/// four identity values is under scan (`0` = vendor, `1` = product,
/// `2` = revision, `3` = serial), and `lss_next` the sub to move to once this
/// one is confirmed. Most callers drive this through [`FastscanMaster`].
pub fn encode_fastscan(id_number: u32, bit_check: u8, lss_sub: u8, lss_next: u8) -> LssFrame {
    let mut f = frame_u32(CS_FASTSCAN, id_number);
    f[5] = bit_check;
    f[6] = lss_sub;
    f[7] = lss_next;
    f
}

/// Whether `response` is the "identify slave" answer a node sends during
/// Fastscan (and identify-remote-slave) — i.e. an unconfigured node is present
/// and matches the request so far.
pub fn is_identify_slave_response(response: &LssFrame) -> bool {
    response[0] == CS_IDENTIFY_SLAVE
}

/// A sans-I/O LSS master that discovers an unconfigured node's 128-bit
/// [`LssAddress`] by Fastscan, when the master does not know it up front.
///
/// It walks the four identity values, bisecting each from the most significant
/// bit: for every bit it emits a [`next_request`](FastscanMaster::next_request)
/// frame, the caller transmits it and reports — via
/// [`on_response`](FastscanMaster::on_response) — whether an
/// [`is_identify_slave_response`] arrived on [`LSS_SLAVE_COB_ID`] within the
/// LSS timeout. After [`is_complete`](FastscanMaster::is_complete), the matched
/// node is in the configuration state and its [`address`](FastscanMaster::address)
/// is known — ready for [`encode_configure_node_id`].
///
/// ```
/// use canopen_rs::lss::{FastscanMaster, LssAddress, LssSlave, UNCONFIGURED_NODE_ID};
///
/// let address = LssAddress { vendor_id: 0x1F, product_code: 0x2A, revision_number: 1, serial_number: 0x1234 };
/// let mut slave = LssSlave::new(address, UNCONFIGURED_NODE_ID);
/// let mut master = FastscanMaster::new();
///
/// while let Some(req) = master.next_request() {
///     let answered = slave.handle(&req).is_some(); // stand-in for the bus round-trip
///     master.on_response(answered);
/// }
/// assert!(master.found());
/// assert_eq!(master.address(), address);
/// ```
#[derive(Debug)]
pub struct FastscanMaster {
    id: [u32; 4],
    sub: u8, // 0..=3 while scanning; 4 once complete
    bit: i8, // -2 = initial probe, 31..=0 = bit under test, -1 = confirm
    found: bool,
}

impl Default for FastscanMaster {
    fn default() -> Self {
        Self::new()
    }
}

impl FastscanMaster {
    /// Start a fresh Fastscan.
    pub const fn new() -> Self {
        Self {
            id: [0; 4],
            sub: 0,
            bit: -2,
            found: false,
        }
    }

    /// The next Fastscan frame to transmit on [`LSS_MASTER_COB_ID`], or `None`
    /// once discovery has finished.
    pub fn next_request(&self) -> Option<LssFrame> {
        if self.sub > 3 {
            return None;
        }
        let sub = self.sub;
        let idx = sub as usize;
        Some(match self.bit {
            -2 => encode_fastscan(0, FASTSCAN_INIT, 0, 0),
            -1 => encode_fastscan(self.id[idx], 0, sub, (sub + 1) & 3),
            b => encode_fastscan(self.id[idx], b as u8, sub, sub),
        })
    }

    /// Report whether the last [`next_request`](FastscanMaster::next_request)
    /// drew an identify-slave response, advancing the scan.
    pub fn on_response(&mut self, responded: bool) {
        match self.bit {
            -2 => {
                if responded {
                    self.found = true;
                    self.bit = 31;
                } else {
                    self.sub = 4; // no unconfigured node answered
                }
            }
            -1 => {
                // Sub confirmed; move on to the next identity value.
                self.sub += 1;
                self.bit = 31;
            }
            b => {
                // No response means this bit of the identity is a 1.
                if !responded {
                    self.id[self.sub as usize] |= 1u32 << b;
                }
                self.bit -= 1; // 31..=0, then -1 to confirm the sub
            }
        }
    }

    /// Whether an unconfigured node answered the initial probe.
    pub const fn found(&self) -> bool {
        self.found
    }

    /// Whether the scan has run to completion.
    pub const fn is_complete(&self) -> bool {
        self.sub > 3
    }

    /// The discovered address — meaningful once [`is_complete`](FastscanMaster::is_complete)
    /// and [`found`](FastscanMaster::found) are both true.
    pub const fn address(&self) -> LssAddress {
        LssAddress {
            vendor_id: self.id[0],
            product_code: self.id[1],
            revision_number: self.id[2],
            serial_number: self.id[3],
        }
    }
}

/// The node-side LSS state machine (CiA 305).
///
/// Feed each frame received on [`LSS_MASTER_COB_ID`] to [`LssSlave::handle`] and
/// transmit any [`LssFrame`] it returns on [`LSS_SLAVE_COB_ID`]. A configured
/// node-id set here becomes active on the node's next reset — call
/// [`LssSlave::adopt_pending`] then.
#[derive(Debug)]
pub struct LssSlave {
    address: LssAddress,
    node_id: u8,
    pending_node_id: u8,
    state: LssState,
    selective: u8,
    fs_sub: u8,
}

impl LssSlave {
    /// Create a slave with the given identity and current node-id
    /// ([`UNCONFIGURED_NODE_ID`] if it has none yet).
    pub const fn new(address: LssAddress, node_id: u8) -> Self {
        Self {
            address,
            node_id,
            pending_node_id: node_id,
            state: LssState::Waiting,
            selective: 0,
            fs_sub: 0,
        }
    }

    /// The identity value for a Fastscan sub (0 = vendor .. 3 = serial).
    fn identity(&self, sub: u8) -> u32 {
        match sub {
            0 => self.address.vendor_id,
            1 => self.address.product_code,
            2 => self.address.revision_number,
            _ => self.address.serial_number,
        }
    }

    /// Handle an LSS Fastscan request, bisecting this node's identity so a
    /// master can discover its [`LssAddress`] without knowing it in advance.
    /// Only an unconfigured node still in the waiting state participates; a full
    /// match on the serial number (the last sub) switches it into the
    /// configuration state, exactly as a selective switch would.
    fn handle_fastscan(&mut self, req: &LssFrame) -> Option<LssFrame> {
        if self.node_id != UNCONFIGURED_NODE_ID || self.state != LssState::Waiting {
            return None;
        }
        let id_number = u32_at(req);
        let bit_check = req[5];
        let lss_sub = req[6];
        let lss_next = req[7];

        if bit_check >= FASTSCAN_INIT {
            // Initial probe: restart the scan and announce our presence.
            self.fs_sub = 0;
            return Some(frame(CS_IDENTIFY_SLAVE));
        }
        if lss_sub != self.fs_sub {
            return None;
        }
        // Check only the bits the master has determined so far ([`bit_check`..=31]).
        let mask = 0xFFFF_FFFFu32 << bit_check;
        if (id_number ^ self.identity(lss_sub)) & mask != 0 {
            return None;
        }
        // A full match with a changing `lss_next` confirms this sub is done.
        if bit_check == 0 && lss_next != lss_sub {
            if lss_sub >= 3 {
                self.state = LssState::Configuration;
            } else {
                self.fs_sub = lss_next;
            }
        }
        Some(frame(CS_IDENTIFY_SLAVE))
    }

    /// The active node-id (`0xFF` while unconfigured).
    pub const fn node_id(&self) -> u8 {
        self.node_id
    }

    /// The node-id set by the master but not yet active (applied on reset).
    pub const fn pending_node_id(&self) -> u8 {
        self.pending_node_id
    }

    /// The current LSS state.
    pub const fn state(&self) -> LssState {
        self.state
    }

    /// This node's LSS address.
    pub const fn address(&self) -> LssAddress {
        self.address
    }

    /// Adopt the pending node-id as the active one — call on the reset that
    /// follows an LSS configuration.
    pub fn adopt_pending(&mut self) {
        self.node_id = self.pending_node_id;
    }

    /// Process an LSS master frame, returning the response to transmit, if any.
    pub fn handle(&mut self, req: &LssFrame) -> Option<LssFrame> {
        let configuring = self.state == LssState::Configuration;
        match req[0] {
            CS_SWITCH_GLOBAL => {
                self.selective = 0;
                self.state = if req[1] == MODE_CONFIGURATION {
                    LssState::Configuration
                } else {
                    LssState::Waiting
                };
                None
            }
            CS_SWITCH_SELECTIVE_VENDOR => {
                self.selective = u8::from(u32_at(req) == self.address.vendor_id);
                None
            }
            CS_SWITCH_SELECTIVE_PRODUCT => {
                self.selective = if self.selective == 1 && u32_at(req) == self.address.product_code
                {
                    2
                } else {
                    0
                };
                None
            }
            CS_SWITCH_SELECTIVE_REVISION => {
                self.selective =
                    if self.selective == 2 && u32_at(req) == self.address.revision_number {
                        3
                    } else {
                        0
                    };
                None
            }
            CS_SWITCH_SELECTIVE_SERIAL => {
                if self.selective == 3 && u32_at(req) == self.address.serial_number {
                    self.selective = 0;
                    self.state = LssState::Configuration;
                    Some(frame(CS_SWITCH_SELECTIVE_RESPONSE))
                } else {
                    self.selective = 0;
                    None
                }
            }
            CS_CONFIGURE_NODE_ID if configuring => {
                let id = req[1];
                let error = if (1..=127).contains(&id) || id == UNCONFIGURED_NODE_ID {
                    self.pending_node_id = id;
                    0
                } else {
                    1 // node-id out of range
                };
                let mut f = frame(CS_CONFIGURE_NODE_ID);
                f[1] = error;
                Some(f)
            }
            CS_STORE_CONFIGURATION if configuring => Some(frame(CS_STORE_CONFIGURATION)),
            CS_INQUIRE_NODE_ID if configuring => {
                let mut f = frame(CS_INQUIRE_NODE_ID);
                f[1] = self.node_id;
                Some(f)
            }
            CS_INQUIRE_VENDOR if configuring => {
                Some(frame_u32(CS_INQUIRE_VENDOR, self.address.vendor_id))
            }
            CS_INQUIRE_PRODUCT if configuring => {
                Some(frame_u32(CS_INQUIRE_PRODUCT, self.address.product_code))
            }
            CS_INQUIRE_REVISION if configuring => {
                Some(frame_u32(CS_INQUIRE_REVISION, self.address.revision_number))
            }
            CS_INQUIRE_SERIAL if configuring => {
                Some(frame_u32(CS_INQUIRE_SERIAL, self.address.serial_number))
            }
            CS_FASTSCAN => self.handle_fastscan(req),
            _ => None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn address() -> LssAddress {
        LssAddress {
            vendor_id: 0x0000_001F,
            product_code: 0x0000_002A,
            revision_number: 0x0000_0001,
            serial_number: 0x0000_1234,
        }
    }

    fn slave() -> LssSlave {
        LssSlave::new(address(), UNCONFIGURED_NODE_ID)
    }

    // --- Master encoders: known-good frames --------------------------------
    #[test]
    fn switch_global_frames() {
        assert_eq!(encode_switch_global(true), [0x04, 0x01, 0, 0, 0, 0, 0, 0]);
        assert_eq!(encode_switch_global(false), [0x04, 0x00, 0, 0, 0, 0, 0, 0]);
    }

    #[test]
    fn configure_node_id_frame() {
        assert_eq!(
            encode_configure_node_id(0x7F),
            [0x11, 0x7F, 0, 0, 0, 0, 0, 0]
        );
    }

    #[test]
    fn switch_selective_sequence() {
        let f = encode_switch_selective(&address());
        assert_eq!(f[0], [0x40, 0x1F, 0x00, 0x00, 0x00, 0, 0, 0]); // vendor id LE
        assert_eq!(f[3], [0x43, 0x34, 0x12, 0x00, 0x00, 0, 0, 0]); // serial LE
    }

    // --- Slave state machine ----------------------------------------------
    #[test]
    fn switch_global_changes_state() {
        let mut s = slave();
        assert_eq!(s.state(), LssState::Waiting);
        assert!(s.handle(&encode_switch_global(true)).is_none());
        assert_eq!(s.state(), LssState::Configuration);
        s.handle(&encode_switch_global(false));
        assert_eq!(s.state(), LssState::Waiting);
    }

    #[test]
    fn selective_switch_matches_address() {
        let mut s = slave();
        let seq = encode_switch_selective(&address());
        assert!(s.handle(&seq[0]).is_none());
        assert!(s.handle(&seq[1]).is_none());
        assert!(s.handle(&seq[2]).is_none());
        let resp = s.handle(&seq[3]).expect("selective response");
        assert!(is_switch_selective_response(&resp));
        assert_eq!(s.state(), LssState::Configuration);
    }

    #[test]
    fn selective_switch_rejects_wrong_address() {
        let mut s = slave();
        let mut seq = encode_switch_selective(&address());
        seq[3][1] = 0xFF; // corrupt the serial number
        for f in &seq {
            assert!(s.handle(f).is_none());
        }
        assert_eq!(s.state(), LssState::Waiting);
    }

    #[test]
    fn configure_node_id_only_in_configuration() {
        let mut s = slave();
        // Ignored while waiting.
        assert!(s.handle(&encode_configure_node_id(0x20)).is_none());

        s.handle(&encode_switch_global(true));
        let resp = s.handle(&encode_configure_node_id(0x20)).unwrap();
        assert_eq!(decode_configure_node_id_response(&resp), Some(Ok(())));
        assert_eq!(s.pending_node_id(), 0x20);
        assert_eq!(s.node_id(), UNCONFIGURED_NODE_ID); // not active until reset
        s.adopt_pending();
        assert_eq!(s.node_id(), 0x20);
    }

    #[test]
    fn configure_node_id_rejects_out_of_range() {
        let mut s = slave();
        s.handle(&encode_switch_global(true));
        let resp = s.handle(&encode_configure_node_id(200)).unwrap();
        assert_eq!(decode_configure_node_id_response(&resp), Some(Err(1)));
    }

    #[test]
    fn inquiries_report_identity_in_configuration() {
        let mut s = slave();
        s.handle(&encode_switch_global(true));

        let node = s.handle(&encode_inquire_node_id()).unwrap();
        assert_eq!(
            decode_inquire_node_id_response(&node),
            Some(UNCONFIGURED_NODE_ID)
        );

        let vendor = s.handle(&frame(CS_INQUIRE_VENDOR)).unwrap();
        assert_eq!(vendor, [0x5A, 0x1F, 0x00, 0x00, 0x00, 0, 0, 0]);
    }

    #[test]
    fn store_acknowledges() {
        let mut s = slave();
        s.handle(&encode_switch_global(true));
        assert_eq!(
            s.handle(&encode_store()).unwrap(),
            [0x17, 0, 0, 0, 0, 0, 0, 0]
        );
    }

    // --- Fastscan ----------------------------------------------------------
    #[test]
    fn fastscan_frame_layout() {
        // CS=0x51, id LE, bit_check, lss_sub, lss_next.
        assert_eq!(
            encode_fastscan(0x1234_5678, 31, 0, 0),
            [0x51, 0x78, 0x56, 0x34, 0x12, 31, 0, 0]
        );
        assert_eq!(
            encode_fastscan(0, FASTSCAN_INIT, 0, 0),
            [0x51, 0, 0, 0, 0, 32, 0, 0]
        );
    }

    #[test]
    fn fastscan_probe_only_answered_while_unconfigured() {
        let mut s = slave();
        let probe = encode_fastscan(0, FASTSCAN_INIT, 0, 0);
        assert!(is_identify_slave_response(&s.handle(&probe).unwrap()));

        // A configured node ignores Fastscan entirely.
        let mut configured = LssSlave::new(address(), 0x20);
        assert!(configured.handle(&probe).is_none());
    }

    /// Drive the sans-I/O master against the slave: it should recover the exact
    /// address and leave the slave in the configuration state.
    fn run_fastscan(addr: LssAddress) -> (FastscanMaster, LssState) {
        let mut slave = LssSlave::new(addr, UNCONFIGURED_NODE_ID);
        let mut master = FastscanMaster::new();
        let mut steps = 0;
        while let Some(req) = master.next_request() {
            let answered = slave.handle(&req).is_some();
            master.on_response(answered);
            steps += 1;
            assert!(steps < 200, "fastscan did not converge");
        }
        (master, slave.state())
    }

    #[test]
    fn fastscan_discovers_the_address() {
        let (master, slave_state) = run_fastscan(address());
        assert!(master.found());
        assert!(master.is_complete());
        assert_eq!(master.address(), address());
        assert_eq!(slave_state, LssState::Configuration);
    }

    #[test]
    fn fastscan_recovers_extreme_identities() {
        for addr in [
            LssAddress {
                vendor_id: 0xFFFF_FFFF,
                product_code: 0,
                revision_number: 0x8000_0000,
                serial_number: 0x0000_0001,
            },
            LssAddress {
                vendor_id: 0xDEAD_BEEF,
                product_code: 0xCAFE_F00D,
                revision_number: 0x0BAD_C0DE,
                serial_number: 0xFEED_FACE,
            },
        ] {
            let (master, slave_state) = run_fastscan(addr);
            assert_eq!(master.address(), addr);
            assert_eq!(slave_state, LssState::Configuration);
        }
    }

    #[test]
    fn fastscan_reports_no_node() {
        // With no slave answering, the probe fails and the scan ends unfound.
        let mut master = FastscanMaster::new();
        let _req = master.next_request().unwrap();
        master.on_response(false);
        assert!(!master.found());
        assert!(master.is_complete());
        assert!(master.next_request().is_none());
    }

    #[test]
    fn fastscan_ignores_non_matching_slave_then_configures_match() {
        // Two unconfigured nodes; the master converges on one exact address,
        // and only that node ends up configured.
        let a = address();
        let b = LssAddress {
            serial_number: 0x9999,
            ..address()
        };
        let mut sa = LssSlave::new(a, UNCONFIGURED_NODE_ID);
        let mut sb = LssSlave::new(b, UNCONFIGURED_NODE_ID);
        let mut master = FastscanMaster::new();
        while let Some(req) = master.next_request() {
            // Either node answering is enough for the master to see a response.
            let ra = sa.handle(&req).is_some();
            let rb = sb.handle(&req).is_some();
            master.on_response(ra || rb);
        }
        assert_eq!(master.address(), a);
        assert_eq!(sa.state(), LssState::Configuration);
        assert_eq!(sb.state(), LssState::Waiting); // diverged, never selected
    }

    /// A tiny linear-congruential generator (glibc constants) so the sweep is
    /// deterministic and dependency-free — no external `rand` crate.
    fn lcg(state: &mut u32) -> u32 {
        *state = state.wrapping_mul(1_103_515_245).wrapping_add(12_345);
        *state
    }

    #[test]
    fn fastscan_recovers_pathological_bit_patterns() {
        // All-zeros, all-ones, and the two alternating-bit patterns in every
        // field — the cases most likely to expose a bit 31 vs bit 0 masking or
        // shift error in the bisection.
        for &v in &[0x0000_0000u32, 0xFFFF_FFFF, 0xAAAA_AAAA, 0x5555_5555] {
            let addr = LssAddress {
                vendor_id: v,
                product_code: v,
                revision_number: v,
                serial_number: v,
            };
            let (master, slave_state) = run_fastscan(addr);
            assert!(master.found());
            assert_eq!(master.address(), addr, "failed to recover {addr:?}");
            assert_eq!(slave_state, LssState::Configuration);
        }

        // A different pattern in each field, so the four subs cannot alias.
        let mixed = LssAddress {
            vendor_id: 0x0000_0000,
            product_code: 0xFFFF_FFFF,
            revision_number: 0xAAAA_AAAA,
            serial_number: 0x5555_5555,
        };
        let (master, slave_state) = run_fastscan(mixed);
        assert_eq!(master.address(), mixed);
        assert_eq!(slave_state, LssState::Configuration);
    }

    #[test]
    fn fastscan_recovers_pseudo_random_addresses() {
        // A deterministic pseudo-random sweep: every recovered address must
        // equal the slave's true one, and the slave must end configured.
        let mut seed = 0x1234_5678u32;
        for _ in 0..512 {
            let addr = LssAddress {
                vendor_id: lcg(&mut seed),
                product_code: lcg(&mut seed),
                revision_number: lcg(&mut seed),
                serial_number: lcg(&mut seed),
            };
            let (master, slave_state) = run_fastscan(addr);
            assert!(master.found());
            assert_eq!(master.address(), addr, "failed to recover {addr:?}");
            assert_eq!(slave_state, LssState::Configuration);
        }
    }

    #[test]
    fn fastscan_converges_on_one_of_many_nodes() {
        // Several unconfigured slaves with distinct identities share the bus.
        // The master sees a response if *any* node answers, yet it must settle
        // on exactly one real address; every other node stays in Waiting.
        let addrs = [
            LssAddress {
                vendor_id: 0x1F,
                product_code: 0x2A,
                revision_number: 1,
                serial_number: 0x1000,
            },
            // Shares vendor/product/revision with the first — diverges only in
            // the serial (the last sub), the hardest case for the bisection.
            LssAddress {
                vendor_id: 0x1F,
                product_code: 0x2A,
                revision_number: 1,
                serial_number: 0x2000,
            },
            LssAddress {
                vendor_id: 0x1F,
                product_code: 0x2A,
                revision_number: 2,
                serial_number: 0x3000,
            },
            LssAddress {
                vendor_id: 0x20,
                product_code: 0x01,
                revision_number: 9,
                serial_number: 0x4000,
            },
            LssAddress {
                vendor_id: 0xDEAD_BEEF,
                product_code: 0x0000_FEED,
                revision_number: 0,
                serial_number: 0xFFFF_FFFF,
            },
        ];
        let mut slaves = [
            LssSlave::new(addrs[0], UNCONFIGURED_NODE_ID),
            LssSlave::new(addrs[1], UNCONFIGURED_NODE_ID),
            LssSlave::new(addrs[2], UNCONFIGURED_NODE_ID),
            LssSlave::new(addrs[3], UNCONFIGURED_NODE_ID),
            LssSlave::new(addrs[4], UNCONFIGURED_NODE_ID),
        ];
        let mut master = FastscanMaster::new();
        while let Some(req) = master.next_request() {
            let mut answered = false;
            for s in &mut slaves {
                answered |= s.handle(&req).is_some();
            }
            master.on_response(answered);
        }

        assert!(master.found());
        let found = master.address();
        assert!(
            addrs.contains(&found),
            "converged on a phantom address {found:?}"
        );
        // Exactly one node — the recovered one — ended up configured.
        let mut configured = 0;
        for (a, s) in addrs.iter().zip(&slaves) {
            if *a == found {
                assert_eq!(s.state(), LssState::Configuration);
                configured += 1;
            } else {
                assert_eq!(s.state(), LssState::Waiting);
            }
        }
        assert_eq!(configured, 1);
    }

    #[test]
    fn fastscan_init_probe_resets_an_in_progress_scan() {
        // The reset probe (bit_check >= FASTSCAN_INIT) must restart the slave's
        // scan, whatever sub it had reached.
        let mut s = slave();
        // Confirm sub 0, advancing the slave to sub 1.
        let confirm_sub0 = encode_fastscan(address().vendor_id, 0, 0, 1);
        assert!(is_identify_slave_response(
            &s.handle(&confirm_sub0).unwrap()
        ));
        // A sub-0 bit frame is now ignored (the slave is scanning sub 1).
        assert!(s.handle(&encode_fastscan(0, 31, 0, 0)).is_none());

        // The reset probe answers and restarts at sub 0…
        let probe = encode_fastscan(0, FASTSCAN_INIT, 0, 0);
        assert!(is_identify_slave_response(&s.handle(&probe).unwrap()));
        // …so the sub-0 bit frame is answered again.
        assert!(s.handle(&encode_fastscan(0, 31, 0, 0)).is_some());
    }

    #[test]
    fn fastscan_never_converges_on_a_phantom_address() {
        // Randomised multi-node stress: for many bus populations, the master —
        // which only sees "did anyone answer?" — must always land on one of the
        // *real* addresses (never a bitwise blend of several) and configure
        // exactly that node.
        let mut seed = 0x0BAD_F00Du32;
        let rand_addr = |seed: &mut u32| LssAddress {
            vendor_id: lcg(seed),
            product_code: lcg(seed),
            revision_number: lcg(seed),
            serial_number: lcg(seed),
        };
        for _ in 0..256 {
            let addrs = [
                rand_addr(&mut seed),
                rand_addr(&mut seed),
                rand_addr(&mut seed),
                rand_addr(&mut seed),
            ];
            let mut slaves = [
                LssSlave::new(addrs[0], UNCONFIGURED_NODE_ID),
                LssSlave::new(addrs[1], UNCONFIGURED_NODE_ID),
                LssSlave::new(addrs[2], UNCONFIGURED_NODE_ID),
                LssSlave::new(addrs[3], UNCONFIGURED_NODE_ID),
            ];
            let mut master = FastscanMaster::new();
            while let Some(req) = master.next_request() {
                let mut answered = false;
                for s in &mut slaves {
                    answered |= s.handle(&req).is_some();
                }
                master.on_response(answered);
            }
            let found = master.address();
            assert!(
                addrs.contains(&found),
                "phantom address {found:?} for population {addrs:?}"
            );
            let configured = slaves
                .iter()
                .filter(|s| s.state() == LssState::Configuration)
                .count();
            assert_eq!(configured, 1, "population {addrs:?}");
        }
    }

    #[test]
    fn fastscan_bit_check_at_boundaries() {
        // bit_check = 31 checks only the MSB; bit_check = 0 checks all 32 bits.
        // Both must stay within `0xFFFF_FFFF << bit_check` (no shift overflow).
        let addr = LssAddress {
            vendor_id: 0x8000_0001, // MSB and LSB set, nothing between
            ..address()
        };
        let mut s = LssSlave::new(addr, UNCONFIGURED_NODE_ID);
        // MSB set: a candidate with bit 31 clear must NOT match at bit_check 31.
        assert!(s.handle(&encode_fastscan(0x0000_0000, 31, 0, 0)).is_none());
        // A candidate with bit 31 set matches when only bit 31 is checked.
        assert!(s.handle(&encode_fastscan(0x8000_0000, 31, 0, 0)).is_some());
        // At bit_check 0 the whole word must match exactly.
        assert!(s.handle(&encode_fastscan(0x8000_0000, 0, 0, 0)).is_none());
        assert!(s.handle(&encode_fastscan(0x8000_0001, 0, 0, 0)).is_some());
    }
}
