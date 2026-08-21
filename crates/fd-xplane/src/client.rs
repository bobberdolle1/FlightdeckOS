//! X-Plane native UDP client: background receiver + typed writes.
//!
//! Threading model (Task 4 §24): a single background thread owns the
//! receive socket and updates a mutex-guarded value map. The adapter thread
//! never blocks on the network for reads (latest values are served from the
//! map); writes fire datagrams without waiting on a reply. No busy loop:
//! the receiver parks in a bounded `recv_timeout`.

use std::collections::HashMap;
use std::net::UdpSocket;
use std::sync::Arc;
use std::sync::atomic::{AtomicU64, Ordering};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use parking_lot::Mutex;

use crate::protocol;

/// How long without packets before the adapter reports disconnect.
pub const STALE_AFTER: Duration = Duration::from_secs(3);

/// Pure staleness predicate — injectable clock for deterministic tests.
pub fn is_stale(last: Option<Instant>, now: Instant, after: Duration) -> bool {
    match last {
        None => true,
        Some(t) => now.duration_since(t) > after,
    }
}

struct Shared {
    values: Mutex<HashMap<i32, f32>>,
    last_packet: Mutex<Option<Instant>>,
    packets_rx: AtomicU64,
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
                values: Mutex::new(HashMap::new()),
                last_packet: Mutex::new(None),
                packets_rx: AtomicU64::new(0),
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

        let socket = self
            .socket
            .try_clone()
            .map_err(|e| ClientError::Network(format!("socket clone: {e}")))?;
        let shared = self.shared.clone();
        let remote = self.remote;
        let payload = self.subscribe_payload.clone();
        let stop = self.stop_flag.clone();
        let handle = std::thread::Builder::new()
            .name("fd-xplane-rx".into())
            .spawn(move || {
                let mut last_keepalive = Instant::now();
                let mut buf = [0u8; 2048];
                loop {
                    if stop.load(Ordering::Relaxed) {
                        break;
                    }
                    match socket.recv_from(&mut buf) {
                        Ok((n, _from)) => {
                            let now = Instant::now();
                            for (id, v) in protocol::parse_rref_records(&buf[..n]) {
                                shared.values.lock().insert(id, v as f64 as f32);
                            }
                            *shared.last_packet.lock() = Some(now);
                            shared.packets_rx.fetch_add(1, Ordering::Relaxed);
                        }
                        Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => {
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
                        Err(_) => break,
                    }
                }
            })
            .map_err(|e| ClientError::Network(format!("rx thread spawn: {e}")))?;
        self.rx_thread = Some(handle);
        Ok(())
    }

    /// Latest value received for a wire id.
    pub fn latest(&self, id: i32) -> Option<f32> {
        self.shared.values.lock().get(&id).copied()
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

    #[test]
    fn staleness_semantics() {
        let now = Instant::now();
        assert!(is_stale(None, now, STALE_AFTER), "never seen => stale");
        let fresh = now - Duration::from_millis(100);
        assert!(!is_stale(Some(fresh), now, STALE_AFTER));
        let old = now - Duration::from_secs(5);
        assert!(is_stale(Some(old), now, STALE_AFTER));
    }

    #[test]
    fn binds_ephemeral_and_reports_port() {
        let c = XPlaneUdpClient::new(0, "127.0.0.1", 49000).unwrap();
        assert_ne!(c.local_port, 0);
        assert!(!c.connected(), "no packets yet => not connected");
    }
}
