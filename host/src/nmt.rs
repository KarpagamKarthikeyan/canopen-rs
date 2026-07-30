//! NMT master tooling: heartbeat monitoring and node health.
//!
//! [`HeartbeatMonitor`](crate::nmt::HeartbeatMonitor) tracks the heartbeats a
//! master hears from the nodes on a bus and flags those that have gone silent.
//! It is transport-agnostic — feed it received frames and a timestamp — so it
//! builds on every platform; sending NMT node-control commands lives on the
//! Linux `SocketCan` transport (`send_nmt`).

use std::collections::BTreeMap;
use std::time::{Duration, Instant};

use canopen_rs::nmt::{decode_heartbeat, HEARTBEAT_COB_BASE};
use canopen_rs::{NmtState, NodeId};

/// The last-known health of a monitored node.
#[derive(Debug, Clone, Copy)]
pub struct NodeHealth {
    /// The NMT state last reported by the node's heartbeat.
    pub state: NmtState,
    /// When that heartbeat was received.
    pub last_seen: Instant,
}

/// Tracks per-node heartbeats and flags nodes that stop producing them.
///
/// A CANopen node emits a heartbeat on `0x700 + node`; a master watches those
/// frames to know each node is alive and in which state. Construct with the
/// consumer timeout, feed every received frame to [`HeartbeatMonitor::on_frame`],
/// and query [`HeartbeatMonitor::timed_out`] / [`HeartbeatMonitor::is_alive`].
#[derive(Debug)]
pub struct HeartbeatMonitor {
    nodes: BTreeMap<u8, NodeHealth>,
    timeout: Duration,
}

impl HeartbeatMonitor {
    /// Create a monitor that considers a node dead after `timeout` without a
    /// heartbeat.
    pub fn new(timeout: Duration) -> Self {
        Self {
            nodes: BTreeMap::new(),
            timeout,
        }
    }

    /// Feed a received frame, timestamped `now`.
    ///
    /// If it is a heartbeat or boot-up frame, records it and returns the
    /// `(node, state)` it reported; otherwise returns `None`.
    pub fn on_frame(
        &mut self,
        cob_id: u16,
        data: &[u8],
        now: Instant,
    ) -> Option<(NodeId, NmtState)> {
        // Heartbeat producers use 0x700 + node, with node in 1..=127.
        let raw = cob_id.checked_sub(HEARTBEAT_COB_BASE)?;
        let node = NodeId::new(u8::try_from(raw).ok()?).ok()?;
        let state = decode_heartbeat(&[*data.first()?]).ok()?;
        self.nodes.insert(
            node.raw(),
            NodeHealth {
                state,
                last_seen: now,
            },
        );
        Some((node, state))
    }

    /// The last state reported by `node`, if it has ever been heard from.
    pub fn state(&self, node: NodeId) -> Option<NmtState> {
        self.nodes.get(&node.raw()).map(|h| h.state)
    }

    /// The full health record for `node`.
    pub fn health(&self, node: NodeId) -> Option<NodeHealth> {
        self.nodes.get(&node.raw()).copied()
    }

    /// Whether `node` produced a heartbeat within the timeout as of `now`.
    pub fn is_alive(&self, node: NodeId, now: Instant) -> bool {
        self.nodes
            .get(&node.raw())
            .is_some_and(|h| now.saturating_duration_since(h.last_seen) <= self.timeout)
    }

    /// The nodes that have gone silent — no heartbeat within the timeout.
    pub fn timed_out(&self, now: Instant) -> impl Iterator<Item = NodeId> + '_ {
        self.nodes.iter().filter_map(move |(&id, h)| {
            (now.saturating_duration_since(h.last_seen) > self.timeout)
                .then(|| NodeId::new(id).ok())
                .flatten()
        })
    }

    /// Every monitored node paired with its health record.
    pub fn nodes(&self) -> impl Iterator<Item = (NodeId, NodeHealth)> + '_ {
        self.nodes
            .iter()
            .filter_map(|(&id, &h)| NodeId::new(id).ok().map(|n| (n, h)))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn node(id: u8) -> NodeId {
        NodeId::new(id).unwrap()
    }

    #[test]
    fn records_heartbeat_state() {
        let mut m = HeartbeatMonitor::new(Duration::from_secs(1));
        let t0 = Instant::now();
        // Heartbeat from node 5 reporting operational (0x05) on 0x705.
        assert_eq!(
            m.on_frame(0x705, &[0x05], t0),
            Some((node(5), NmtState::Operational))
        );
        assert_eq!(m.state(node(5)), Some(NmtState::Operational));
    }

    #[test]
    fn decodes_bootup_frame() {
        let mut m = HeartbeatMonitor::new(Duration::from_secs(1));
        // Boot-up is 0x700+node with data 0x00 -> Initialising.
        assert_eq!(
            m.on_frame(0x70A, &[0x00], Instant::now()),
            Some((node(10), NmtState::Initialising))
        );
    }

    #[test]
    fn ignores_non_heartbeat_frames() {
        let mut m = HeartbeatMonitor::new(Duration::from_secs(1));
        let now = Instant::now();
        assert_eq!(m.on_frame(0x585, &[0x43, 0, 0, 0], now), None); // an SDO response
        assert_eq!(m.on_frame(0x000, &[0x01, 0x05], now), None); // an NMT command
        assert_eq!(m.on_frame(0x700, &[0x00], now), None); // node 0 is not valid
    }

    #[test]
    fn liveness_and_timeout() {
        let mut m = HeartbeatMonitor::new(Duration::from_secs(1));
        let t0 = Instant::now();
        m.on_frame(0x705, &[0x05], t0);

        assert!(m.is_alive(node(5), t0));
        assert!(m.is_alive(node(5), t0 + Duration::from_millis(900)));
        assert!(!m.is_alive(node(5), t0 + Duration::from_millis(1500)));

        let timed_out: Vec<_> = m.timed_out(t0 + Duration::from_secs(2)).collect();
        assert_eq!(timed_out, vec![node(5)]);
        assert!(m.timed_out(t0).next().is_none());
    }

    #[test]
    fn unknown_node_is_not_alive() {
        let m = HeartbeatMonitor::new(Duration::from_secs(1));
        assert!(!m.is_alive(node(9), Instant::now()));
        assert_eq!(m.state(node(9)), None);
    }
}
