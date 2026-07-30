//! Layer Setting Services (LSS, CiA 305) — configure a node's node-id and bit
//! timing over the bus.
//!
//! A node that ships without a preset node-id comes up *unconfigured* and
//! answers only LSS. An LSS master switches it into the *configuration* state —
//! globally, or selectively by matching its 128-bit [`LssAddress`] (the object
//! `0x1018` identity) — then assigns a node-id and asks it to store the setting.
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

// Switch-state-global modes (byte 1).
const MODE_WAITING: u8 = 0;
const MODE_CONFIGURATION: u8 = 1;

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
        }
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
}
