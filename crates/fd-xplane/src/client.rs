//! X-Plane native UDP client: background receiver + typed writes.
//!
//! Threading model (Task 4 §24): a single background thread owns the
//! receive socket and updates a mutex-guarded value board. The adapter
//! thread never blocks on the network for reads (latest values are served
//! from the board); writes fire datagrams without waiting on a reply.
//!
//! Hardening (adversarial audit, Live Lab V1):
//! * the receiver parks in a 1 s `recv_timeout`, so `Drop`/join terminates
//!   deterministically even when X-Plane goes completely silent;
//! * the receive buffer is 65 547 B (max UDP payload) — an oversized or
//!   hostile datagram can no longer kill the thread via `WSAEMSGSIZE`;
//! * unexpected recv errors are counted and skipped, never fatal;
//! * datagrams from any source other than the configured simulator are
//!   rejected (countered) — telemetry is only fed by the endpoint we
//!   subscribed to;
//! * record ids outside the subscribed set are dropped (countered) so a
//!   flood of foreign ids cannot grow the map without bound;
//! * per-value freshness: a value older than [`STALE_AFTER`] is reported
//!   as absent even while the packet stream as a whole is alive — a frozen
//!   channel must not masquerade as live telemetry.

use std::collections::{HashMap, HashSet};
use std::net::UdpSocket;
use std::sync::Arc;
use std::sync::atomic::{AtomicU64, Ordering};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use parking_lot::Mutex;

use crate::protocol;

/// How long without packets before the adapter reports disconnect.
pub const STALE_AFTER: Duration = Duration::from_secs(3);

/// Receive buffer: above the practical max UDP payload (65 507 B =
/// 65 535 − 20 IP − 8 UDP) so a full-size datagram never triggers a
/// truncated-read error on Windows.
pub const RECV_BUFFER_BYTES: usize = 65_527;

/// Pure staleness predicate — injectable clock for deterministic tests.
pub fn is_stale(last: Option<Instant>, now: Instant, after: Duration) -> bool {
    match last {
        None => true,
        Some(t) => now.duration_since(t) > after,
    }
}

/// Per-value freshness board: latest value per wire id with receive time.
///
/// Extracted from the client so staleness semantics are deterministically
/// testable without sockets or real time.
#[derive(Default)]
pub struct ValueBoard {
    entries: Mutex<HashMap<i32, (Instant, f32)>>,
}

impl ValueBoard {
    pub fn new() -> Self {
        Self::default()
    }

    /// Record a fresh value observed at `now`.
    pub fn insert(&self, id: i32, value: f32, now: Instant) {
        self.entries.lock().insert(id, (now, value));
    }

    /// Latest value for `id`, or `None` when missing or older than `after`
    /// relative to `now`. Stale values stay in the board (a resumed stream
    /// overwrites them) but are never served as current.
    pub fn latest(&self, id: i32, now: Instant, after: Duration) -> Option<f32> {
        let entries = self.entries.lock();
        let (t, v) = entries.get(&id)?;
        if now.duration_since(*t) > after {
            None
        } else {
            Some(*v)
        }
    }

    /// Channel quality for `id` at `now` (spec §21): Fresh when a recent
    /// value exists, Stale when present but older than `after`, Missing
    /// when never received.
    pub fn quality(
        &self,
        id: i32,
        now: Instant,
        after: Duration,
    ) -> fd_core::telemetry::DataQuality {
        use fd_core::telemetry::DataQuality;
        let entries = self.entries.lock();
        match entries.get(&id) {
            None => DataQuality::Missing,
            Some((t, _)) => {
                if now.duration_since(*t) > after {
                    DataQuality::Stale
                } else {
                    DataQuality::Fresh
                }
            }
        }
    }

    pub fn len(&self) -> usize {
        self.entries.lock().len()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

struct Shared {
    board: ValueBoard,
    last_packet: Mutex<Option<Instant>>,
    packets_rx: AtomicU64,
    rejected_foreign: AtomicU64,
    unknown_ids_dropped: AtomicU64,
    recv_errors: AtomicU64,
}

#[derive(Debug)]
pub enum ClientError {
    Network(String),
    NotSubscribed,
}

impl std::fmt::Display for ClientError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Network(e) => write!(f, "xplane udp network error: {e}"),
            Self::NotSubscribed => write!(f, "not subscribed to xplane datarefs"),
        }
    }
}

pub struct XPlaneUdpClient {
    socket: UdpSocket,
    remote: std::net::SocketAddr,
    shared: Arc<Shared>,
    rx_thread: Option<JoinHandle<()>>,
    subscribe_payload: Vec<u8>,
    stop_flag: Arc<std::sync::atomic::AtomicBool>,
    /// Local bind port actually used.
    pub local_port: u16,
}

impl XPlaneUdpClient {
    /// Bind a local socket and prepare (but do not start) subscription.
    pub fn new(local_port: u16, xp_host: &str, xp_port: u16) -> Result<Self, ClientError> {
        let socket = UdpSocket::bind(("0.0.0.0", local_port))
            .map_err(|e| ClientError::Network(format!("local bind {local_port}: {e}")))?;
        let bound_port = socket.local_addr().map(|a| a.port()).unwrap_or(local_port);
        let remote = format!("{xp_host}:{xp_port}");
        let addr: std::net::SocketAddr = remote
            .parse()
            .map_err(|e| ClientError::Network(format!("bad xp address {remote}: {e}")))?;
        Ok(Self {
            socket,
            remote: addr,
            shared: Arc::new(Shared {
                board: ValueBoard::new(),
                last_packet: Mutex::new(None),
                packets_rx: AtomicU64::new(0),
                rejected_foreign: AtomicU64::new(0),
                unknown_ids_dropped: AtomicU64::new(0),
                recv_errors: AtomicU64::new(0),
            }),
            rx_thread: None,
            subscribe_payload: Vec::new(),
            stop_flag: Arc::new(std::sync::atomic::AtomicBool::new(false)),
            local_port: bound_port,
        })
    }

    /// Send the RREF subscriptions (one packet per dataref) and spawn the
    /// receive thread.
    pub fn start(&mut self, freq_hz: i32, refs: &[(i32, &'static str)]) -> Result<(), ClientError> {
        let packets: Vec<Vec<u8>> = refs
            .iter()
            .map(|(id, path)| protocol::rref_subscribe(freq_hz, *id, path))
            .collect();
        for pkt in &packets {
            self.socket
                .send_to(pkt, self.remote)
                .map_err(|e| ClientError::Network(format!("subscribe send: {e}")))?;
        }
        // Keep the full set for keepalive/resubscribe.
        self.subscribe_payload = packets.concat();
        let subscribed: HashSet<i32> = refs.iter().map(|(id, _)| *id).collect();

        let socket = self
            .socket
            .try_clone()
            .map_err(|e| ClientError::Network(format!("socket clone: {e}")))?;
        // Bounded receive: without this the thread would park forever inside
        // recv_from when the simulator goes silent, hanging Drop/join.
        socket
            .set_read_timeout(Some(Duration::from_secs(1)))
            .map_err(|e| ClientError::Network(format!("set_read_timeout: {e}")))?;
        let shared = self.shared.clone();
        let remote = self.remote;
        let payload = self.subscribe_payload.clone();
        let stop = self.stop_flag.clone();
        let handle = std::thread::Builder::new()
            .name("fd-xplane-rx".into())
            .spawn(move || {
                let mut last_keepalive = Instant::now();
                let mut buf = [0u8; RECV_BUFFER_BYTES];
                loop {
                    if stop.load(Ordering::Relaxed) {
                        break;
                    }
                    match socket.recv_from(&mut buf) {
                        Ok((n, from)) => {
                            // Source authenticity: only the configured
                            // simulator HOST may feed telemetry. The source
                            // PORT is deliberately not checked: X-Plane 12
                            // streams RREF replies from a secondary socket
                            // (observed live: command port 49000, data
                            // source port 49001). Host identity is the
                            // security boundary the protocol can express.
                            if from.ip() != remote.ip() {
                                shared.rejected_foreign.fetch_add(1, Ordering::Relaxed);
                                continue;
                            }
                            let now = Instant::now();
                            for (id, v) in protocol::parse_rref_records(&buf[..n]) {
                                if subscribed.contains(&id) {
                                    shared.board.insert(id, v as f64 as f32, now);
                                } else {
                                    shared.unknown_ids_dropped.fetch_add(1, Ordering::Relaxed);
                                }
                            }
                            *shared.last_packet.lock() = Some(now);
                            shared.packets_rx.fetch_add(1, Ordering::Relaxed);
                        }
                        Err(e)
                            if e.kind() == std::io::ErrorKind::WouldBlock
                                || e.kind() == std::io::ErrorKind::TimedOut =>
                        {
                            // Timeout: keepalive the subscription every 10 s
                            // so X-Plane does not drop it during silence.
                            if last_keepalive.elapsed() >= Duration::from_secs(10)
                                && !payload.is_empty()
                            {
                                let _ = socket.send_to(&payload, remote);
                                last_keepalive = Instant::now();
                            }
                            continue;
                        }
                        Err(_) => {
                            // Non-fatal: a single malformed/oversized datagram
                            // (or transient OS error) must not kill telemetry
                            // for the rest of the session. Count and keep
                            // going; the stop flag is the only exit.
                            shared.recv_errors.fetch_add(1, Ordering::Relaxed);
                            std::thread::sleep(Duration::from_millis(100));
                            continue;
                        }
                    }
                }
            })
            .map_err(|e| ClientError::Network(format!("rx thread spawn: {e}")))?;
        self.rx_thread = Some(handle);
        Ok(())
    }

    /// Channel quality as seen by the client (spec §21).
    pub fn quality(&self, id: i32) -> fd_core::telemetry::DataQuality {
        self.shared.board.quality(id, Instant::now(), STALE_AFTER)
    }

    /// Latest FRESH value received for a wire id. Values older than
    /// [`STALE_AFTER`] read as `None` even while the packet stream lives.
    pub fn latest(&self, id: i32) -> Option<f32> {
        self.shared.board.latest(id, Instant::now(), STALE_AFTER)
    }

    /// Whether fresh packets have arrived within [`STALE_AFTER`].
    pub fn connected(&self) -> bool {
        !is_stale(*self.shared.last_packet.lock(), Instant::now(), STALE_AFTER)
    }

    /// Age of the newest packet (upper bound of read latency).
    pub fn newest_packet_age(&self) -> Duration {
        match *self.shared.last_packet.lock() {
            None => Duration::MAX,
            Some(t) => t.elapsed(),
        }
    }

    pub fn packets_received(&self) -> u64 {
        self.shared.packets_rx.load(Ordering::Relaxed)
    }

    /// Datagrams dropped because their source was not the simulator.
    pub fn rejected_foreign_packets(&self) -> u64 {
        self.shared.rejected_foreign.load(Ordering::Relaxed)
    }

    /// Record ids dropped because they are outside the subscribed set.
    pub fn unknown_ids_dropped(&self) -> u64 {
        self.shared.unknown_ids_dropped.load(Ordering::Relaxed)
    }

    /// Receive errors survived (non-fatal by design).
    pub fn recv_errors(&self) -> u64 {
        self.shared.recv_errors.load(Ordering::Relaxed)
    }

    /// Send an allowlisted DREF0 write.
    pub fn write_dref(&self, path: &str, value: f32) -> Result<(), ClientError> {
        self.socket
            .send_to(&protocol::dref_set(path, value), self.remote)
            .map(|_| ())
            .map_err(|e| ClientError::Network(format!("dref write {path}: {e}")))
    }

    /// Dispatch an allowlisted CMND0 command.
    pub fn send_command(&self, path: &str) -> Result<(), ClientError> {
        self.socket
            .send_to(&protocol::cmnd(path), self.remote)
            .map(|_| ())
            .map_err(|e| ClientError::Network(format!("command {path}: {e}")))
    }

    /// Re-send the original subscription (reconnect path).
    pub fn resubscribe(&self) -> Result<(), ClientError> {
        if self.subscribe_payload.is_empty() {
            return Err(ClientError::NotSubscribed);
        }
        self.socket
            .send_to(&self.subscribe_payload, self.remote)
            .map(|_| ())
            .map_err(|e| ClientError::Network(format!("resubscribe: {e}")))
    }
}

impl Drop for XPlaneUdpClient {
    fn drop(&mut self) {
        self.stop_flag
            .store(true, std::sync::atomic::Ordering::Relaxed);
        if let Some(h) = self.rx_thread.take() {
            let _ = h.join();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn at_offset(secs: u64) -> Instant {
        Instant::now() - Duration::from_secs(secs)
    }

    #[test]
    fn value_board_serves_fresh_and_drops_stale() {
        let board = ValueBoard::new();
        let t0 = at_offset(10);
        board.insert(1, 42.0, t0);
        assert_eq!(
            board.latest(1, t0 + Duration::from_secs(1), STALE_AFTER),
            Some(42.0)
        );
        // Past the freshness window: absent, even though still stored.
        assert_eq!(
            board.latest(1, t0 + STALE_AFTER + Duration::from_secs(1), STALE_AFTER),
            None
        );
        // A resumed stream revives the channel.
        let t1 = t0 + STALE_AFTER + Duration::from_secs(2);
        board.insert(1, 7.0, t1);
        assert_eq!(
            board.latest(1, t1 + Duration::from_millis(1), STALE_AFTER),
            Some(7.0)
        );
    }

    #[test]
    fn value_board_missing_id_is_none_and_empty_board_is_empty() {
        let board = ValueBoard::new();
        assert!(board.is_empty());
        let now = Instant::now();
        assert_eq!(board.latest(9, now, STALE_AFTER), None);
        board.insert(9, 1.0, now);
        assert_eq!(board.len(), 1);
    }

    #[test]
    fn staleness_predicate_covers_never_received() {
        let now = Instant::now();
        assert!(is_stale(None, now, STALE_AFTER));
        assert!(!is_stale(Some(now), now, STALE_AFTER));
        assert!(is_stale(
            Some(now - STALE_AFTER - Duration::from_secs(1)),
            now,
            STALE_AFTER
        ));
    }
}
