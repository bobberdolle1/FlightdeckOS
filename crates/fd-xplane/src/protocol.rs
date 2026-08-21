//! X-Plane native UDP protocol encoding/parsing.
//!
//! Wire formats verified empirically against X-Plane 12.4.3 (and matching
//! the community reference implementation charlylima/XPlaneUDP):
//!
//! * Subscribe: ONE packet PER dataref —
//!   `"RREF\0"` + i32 freq + i32 client_id + 400-byte NUL-padded path
//!   (total 413 bytes).
//! * Response datagrams: header `"RREF,"` (5 bytes) followed by repeating
//!   8-byte records `{ i32 client_id, f32 value }` (little-endian).
//! * Write: `"DREF\0"` + f32 value + 500-byte NUL-padded path (509 bytes).
//!
//! Pure functions — fully unit-testable offline.

/// Encode an `RREF` subscription packet for ONE dataref (413 bytes).
pub fn rref_subscribe(freq_hz: i32, client_id: i32, path: &str) -> Vec<u8> {
    let mut buf = Vec::with_capacity(413);
    buf.extend_from_slice(b"RREF\0");
    buf.extend_from_slice(&freq_hz.to_le_bytes());
    buf.extend_from_slice(&client_id.to_le_bytes());
    let mut field = [0u8; 400];
    let bytes = path.as_bytes();
    let n = bytes.len().min(399); // keep at least one NUL terminator
    field[..n].copy_from_slice(&bytes[..n]);
    buf.extend_from_slice(&field);
    debug_assert_eq!(buf.len(), 413);
    buf
}

/// Encode a `DREF` dataref write (509 bytes).
pub fn dref_set(path: &str, value: f32) -> Vec<u8> {
    let mut buf = Vec::with_capacity(509);
    buf.extend_from_slice(b"DREF\0");
    buf.extend_from_slice(&value.to_le_bytes());
    let mut field = [0u8; 500];
    let bytes = path.as_bytes();
    let n = bytes.len().min(499);
    field[..n].copy_from_slice(&bytes[..n]);
    buf.extend_from_slice(&field);
    debug_assert_eq!(buf.len(), 509);
    buf
}

/// Encode a `CMND` command dispatch (NUL-terminated command).
pub fn cmnd(command: &str) -> Vec<u8> {
    let mut buf = Vec::with_capacity(5 + command.len() + 1);
    buf.extend_from_slice(b"CMND\0");
    buf.extend_from_slice(command.as_bytes());
    buf.push(0);
    buf
}

/// Parse one response datagram into `(client_id, value)` pairs.
/// Trailing partial records are ignored (truncation must not poison us).
pub fn parse_rref_records(buf: &[u8]) -> Vec<(i32, f32)> {
    if buf.len() < 5 || &buf[..5] != b"RREF," {
        return Vec::new();
    }
    let mut out = Vec::new();
    let mut off = 5usize;
    while off + 8 <= buf.len() {
        let id = i32::from_le_bytes(buf[off..off + 4].try_into().unwrap());
        let value = f32::from_le_bytes(buf[off + 4..off + 8].try_into().unwrap());
        out.push((id, value));
        off += 8;
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn subscribe_packet_layout() {
        let pkt = rref_subscribe(4, 7, "sim/flightmodel/position/psi");
        assert_eq!(pkt.len(), 413);
        assert_eq!(&pkt[..5], b"RREF\0");
        assert_eq!(&pkt[5..9], &4i32.to_le_bytes());
        assert_eq!(&pkt[9..13], &7i32.to_le_bytes());
        let path = b"sim/flightmodel/position/psi";
        assert_eq!(&pkt[13..13 + path.len()], &path[..]);
        // Rest of the 400-byte field must be NUL padding.
        assert!(pkt[13 + path.len()..].iter().all(|&b| b == 0));
    }

    #[test]
    fn long_path_keeps_nul_terminator() {
        let long = "x".repeat(450);
        let pkt = rref_subscribe(1, 1, &long);
        assert_eq!(pkt.len(), 413);
        assert_eq!(pkt[13 + 399], 0, "last byte must stay a NUL");
    }

    #[test]
    fn dref_packet_layout() {
        let pkt = dref_set("sim/cockpit/autopilot/heading_mag", 123.5);
        assert_eq!(pkt.len(), 509);
        assert_eq!(&pkt[..5], b"DREF\0");
        assert_eq!(&pkt[5..9], &123.5f32.to_le_bytes());
        let path = b"sim/cockpit/autopilot/heading_mag";
        assert_eq!(&pkt[9..9 + path.len()], &path[..]);
        assert!(pkt[9 + path.len()..].iter().all(|&b| b == 0));
    }

    #[test]
    fn cmnd_packet_layout() {
        let pkt = cmnd("sim/autopilot/heading_hold");
        assert_eq!(&pkt[..5], b"CMND\0");
        assert_eq!(&pkt[5..31], b"sim/autopilot/heading_hold");
        assert_eq!(pkt[pkt.len() - 1], 0);
    }

    #[test]
    fn parses_rref_response_records() {
        let mut buf = b"RREF,".to_vec();
        for (id, v) in [(401i32, 12.5f32), (-2, -300.25)] {
            buf.extend_from_slice(&id.to_le_bytes());
            buf.extend_from_slice(&v.to_le_bytes());
        }
        // Truncated record tail must be ignored.
        buf.extend_from_slice(&[0xAA, 0xBB]);
        let vals = parse_rref_records(&buf);
        assert_eq!(vals, vec![(401, 12.5), (-2, -300.25)]);
    }

    #[test]
    fn rejects_foreign_datagrams() {
        assert!(parse_rref_records(b"DREF\x00").is_empty());
        assert!(parse_rref_records(&[]).is_empty());
        assert!(parse_rref_records(b"RREF").is_empty()); // header only
    }
}
