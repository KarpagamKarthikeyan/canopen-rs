//! Network Management (NMT) — the node state machine (CiA 301 §7.3).
//!
//! NMT governs a node's lifecycle — Initialisation → Pre-operational →
//! Operational / Stopped — driven by master command frames (COB-ID `0x000`)
//! and reported to the network via the heartbeat producer (`0x700 + node`).

// TODO: `NmtState`, `NmtCommand`, the guarded state-machine transitions,
// and the heartbeat producer/consumer.
