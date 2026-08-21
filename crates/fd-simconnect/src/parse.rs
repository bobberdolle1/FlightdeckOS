//! Pure decoding of SimConnect RECV records.
//!
//! The trusted decoding boundary is [`parse_record`], which operates on a
//! plain byte slice and validates EVERY read against the slice length:
//!
//! 1. minimum base header size;
//! 2. sanity of the record's own `dwSize` claim (must not exceed the buffer);
//! 3. `dwDefineCount` clamped to the telemetry definition width;
//! 4. payload size computed with checked arithmetic;
//! 5. `required_end <= buf.len()` proven before any payload read.
//!
//! A malformed/truncated record decodes to [`RecvRecord::Malformed`] — it is
//! never dereferenced beyond the buffer, never panics, and never produces
//! fabricated state. `read_unaligned` is used for the packed SDK layout; it
//! addresses alignment ONLY — all validity comes from the bounds checks.

use crate::defs::*;
use crate::ffi;

/// Size of the packed base header (`SIMCONNECT_RECV`).
const BASE_HEADER_LEN: usize = std::mem::size_of::<ffi::SIMCONNECT_RECV>();
/// Offset of the FLOAT64 payload after the SIMOBJECT_DATA header fields.
const PAYLOAD_OFFSET: usize = ffi::SIMOBJECT_DATA_PAYLOAD_OFFSET;
/// Packed size of `SIMCONNECT_RECV_SIMOBJECT_DATA` (44 = PAYLOAD_OFFSET).
const SIMOBJECT_DATA_HEADER_LEN: usize = std::mem::size_of::<ffi::SIMCONNECT_RECV_SIMOBJECT_DATA>();
/// Packed size of `SIMCONNECT_RECV_EVENT` (24).
const EVENT_LEN: usize = std::mem::size_of::<ffi::SIMCONNECT_RECV_EVENT>();
/// Packed size of `SIMCONNECT_RECV_EXCEPTION` (24).
const EXCEPTION_LEN: usize = std::mem::size_of::<ffi::SIMCONNECT_RECV_EXCEPTION>();

fn read_u32(buf: &[u8], offset: usize) -> u32 {
    // SAFETY: callers guarantee `offset + 4 <= buf.len()`; the pointer is
    // derived from the live slice, and the read is unaligned-safe by design
    // (packed SDK layout).
    unsafe { std::ptr::read_unaligned(buf.as_ptr().add(offset) as *const u32) }
}

fn read_f64(buf: &[u8], offset: usize) -> f64 {
    // SAFETY: same contract as `read_u32`, for an 8-byte read.
    unsafe { std::ptr::read_unaligned(buf.as_ptr().add(offset) as *const f64) }
}

/// A decoded record from the dispatch callback.
#[derive(Debug, Clone, PartialEq)]
pub enum RecvRecord {
    /// Periodic telemetry payload (length already validated).
    SimObjectData { values: Vec<f64> },
    /// Pause/unpause system event.
    SystemEventPause(bool),
    /// Exception reported by the sim (non-fatal for polling).
    Exception { code: ffi::DWORD },
    /// Connection acknowledged.
    Open,
    /// Simulator is quitting.
    Quit,
    /// Recognized record kind that carries no Task 1 semantics.
    Ignored,
    /// Record failed bounds validation. Never dereferenced beyond the
    /// buffer; surfaced for diagnostics only.
    Malformed { detail: &'static str },
}

/// Decode one dispatch record from its raw bytes.
///
/// Pure function of `(bytes)` — no FFI, fully unit-testable. Every access
/// is preceded by a bounds check against `buf.len()`; integer arithmetic on
/// attacker-controlled counts uses checked operations before any dereference.
pub fn parse_record(buf: &[u8]) -> RecvRecord {
    // 1. Base header must be present.
    if buf.len() < BASE_HEADER_LEN {
        return RecvRecord::Malformed {
            detail: "record shorter than base header",
        };
    }
    let dw_size = read_u32(buf, 0) as usize;
    let id = read_u32(buf, 8);

    // 2. The record's own size claim must not exceed the supplied buffer.
    if dw_size > buf.len() {
        return RecvRecord::Malformed {
            detail: "dwSize exceeds callback buffer",
        };
    }

    if id == ffi::SIMCONNECT_RECV_ID_SIMOBJECT_DATA {
        // Header fields (including the dwData placeholder) must be present.
        if buf.len() < SIMOBJECT_DATA_HEADER_LEN || dw_size < SIMOBJECT_DATA_HEADER_LEN {
            return RecvRecord::Malformed {
                detail: "SIMOBJECT_DATA shorter than fixed header",
            };
        }
        let define_count = read_u32(
            buf,
            std::mem::offset_of!(ffi::SIMCONNECT_RECV_SIMOBJECT_DATA, dwDefineCount),
        ) as usize;
        // 3./4. Clamp to the definition width, then compute the required end
        // offset without overflow (count is already bounded by the clamp).
        let n = define_count.min(DATUM_COUNT);
        let required_end = match n
            .checked_mul(8)
            .and_then(|payload| PAYLOAD_OFFSET.checked_add(payload))
        {
            Some(end) => end,
            None => {
                return RecvRecord::Malformed {
                    detail: "payload size overflow",
                };
            }
        };
        // 5. Both sizes must cover the payload we are about to read.
        if buf.len() < required_end || dw_size < required_end {
            return RecvRecord::Malformed {
                detail: "SIMOBJECT_DATA payload truncated",
            };
        }
        let mut values = Vec::with_capacity(n);
        for i in 0..n {
            values.push(read_f64(buf, PAYLOAD_OFFSET + i * 8));
        }
        RecvRecord::SimObjectData { values }
    } else if id == ffi::SIMCONNECT_RECV_ID_EVENT {
        if buf.len() < EVENT_LEN || dw_size < EVENT_LEN {
            return RecvRecord::Malformed {
                detail: "EVENT shorter than fixed layout",
            };
        }
        let event = read_u32(buf, 16); // uEventID offset in packed layout
        if event == EVT_PAUSE {
            RecvRecord::SystemEventPause(true)
        } else if event == EVT_UNPAUSE {
            RecvRecord::SystemEventPause(false)
        } else {
            // One-shot action acks and other client events: ignored — the
            // action pipeline verifies via observed state, never via acks.
            RecvRecord::Ignored
        }
    } else if id == ffi::SIMCONNECT_RECV_ID_EXCEPTION {
        if buf.len() < EXCEPTION_LEN || dw_size < EXCEPTION_LEN {
            return RecvRecord::Malformed {
                detail: "EXCEPTION shorter than fixed layout",
            };
        }
        RecvRecord::Exception {
            code: read_u32(buf, 12), // dwException offset in packed layout
        }
    } else if id == ffi::SIMCONNECT_RECV_ID_OPEN {
        // Version fields are not consumed yet (deferred hardening); the base
        // header check above is sufficient for safe recognition.
        RecvRecord::Open
    } else if id == ffi::SIMCONNECT_RECV_ID_QUIT {
        RecvRecord::Quit
    } else {
        RecvRecord::Ignored
    }
}

/// Decode one dispatch record from the raw callback pointer.
///
/// # Safety
/// `p` must point to `cb_data` initialized bytes delivered by
/// `SimConnect_CallDispatch` for this handle. All bounds validation happens
/// inside [`parse_record`]; nothing beyond `cb_data` is ever read.
pub unsafe fn parse_recv(p: *const ffi::SIMCONNECT_RECV, cb_data: ffi::DWORD) -> RecvRecord {
    // SAFETY: caller guarantees `p` points at `cb_data` initialized bytes;
    // the slice only bounds subsequent reads, it never dereferences.
    let bytes = unsafe { std::slice::from_raw_parts(p as *const u8, cb_data as usize) };
    parse_record(bytes)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Build a packed SIMCONNECT_RECORD-style buffer: 12-byte base + typed
    /// tail + optional f64 payload, mirroring the SDK packed layout.
    fn simobject_data_buf(define_count: u32, values: &[f64], omit_tail_bytes: usize) -> Vec<u8> {
        let mut b = Vec::new();
        b.extend_from_slice(&0u32.to_le_bytes()); // dwSize (overwritten below)
        b.extend_from_slice(&0u32.to_le_bytes()); // dwVersion
        b.extend_from_slice(&ffi::SIMCONNECT_RECV_ID_SIMOBJECT_DATA.to_le_bytes());
        b.extend_from_slice(&REQUEST_TELEMETRY.to_le_bytes()); // dwRequestID
        b.extend_from_slice(&0u32.to_le_bytes()); // dwObjectID
        b.extend_from_slice(&DEFINE_TELEMETRY.to_le_bytes()); // dwDefineID
        b.extend_from_slice(&0u32.to_le_bytes()); // dwFlags
        b.extend_from_slice(&1u32.to_le_bytes()); // dwentrynumber
        b.extend_from_slice(&1u32.to_le_bytes()); // dwoutof
        b.extend_from_slice(&define_count.to_le_bytes()); // dwDefineCount
        b.extend_from_slice(&0u32.to_le_bytes()); // dwData placeholder
        debug_assert_eq!(b.len(), PAYLOAD_OFFSET);
        for v in values {
            b.extend_from_slice(&v.to_le_bytes());
        }
        let total = b.len();
        let kept = total - omit_tail_bytes;
        b.truncate(kept);
        // dwSize = full logical size BEFORE truncation.
        b[0..4].copy_from_slice(&(total as u32).to_le_bytes());
        b
    }

    fn event_buf(event_id: u32) -> Vec<u8> {
        let mut b = Vec::new();
        b.extend_from_slice(&24u32.to_le_bytes()); // dwSize
        b.extend_from_slice(&0u32.to_le_bytes()); // dwVersion
        b.extend_from_slice(&ffi::SIMCONNECT_RECV_ID_EVENT.to_le_bytes());
        b.extend_from_slice(&0u32.to_le_bytes()); // uGroupID
        b.extend_from_slice(&event_id.to_le_bytes()); // uEventID
        b.extend_from_slice(&0u32.to_le_bytes()); // dwData
        assert_eq!(b.len(), EVENT_LEN);
        b
    }

    #[test]
    fn fake_layout_matches_sdk_layout() {
        assert_eq!(BASE_HEADER_LEN, 12);
        assert_eq!(PAYLOAD_OFFSET, 44);
        assert_eq!(SIMOBJECT_DATA_HEADER_LEN, 44);
        assert_eq!(EVENT_LEN, 24);
        assert_eq!(EXCEPTION_LEN, 24);
    }

    #[test]
    fn valid_unaligned_payload_parses() {
        // 3 f64s at offset 44: unaligned (44 % 8 == 4).
        let buf = simobject_data_buf(3, &[1.5, 2.5, 3.5], 0);
        match parse_record(&buf) {
            RecvRecord::SimObjectData { values } => assert_eq!(values, vec![1.5, 2.5, 3.5]),
            other => panic!("unexpected: {other:?}"),
        }
    }

    #[test]
    fn truncated_base_record_is_malformed() {
        let buf = vec![0u8; BASE_HEADER_LEN - 1];
        match parse_record(&buf) {
            RecvRecord::Malformed { .. } => {}
            other => panic!("unexpected: {other:?}"),
        }
    }

    #[test]
    fn truncated_simobject_header_is_malformed() {
        // Enough for base header, not for the 44-byte SIMOBJECT_DATA header.
        let mut buf = simobject_data_buf(1, &[1.0], 0);
        buf.truncate(30);
        // Fix dwSize to be consistent with the truncation so the rejection is
        // attributable to the header bound, not the size claim.
        let len = buf.len() as u32;
        buf[0..4].copy_from_slice(&len.to_le_bytes());
        match parse_record(&buf) {
            RecvRecord::Malformed { detail } => {
                assert!(detail.contains("shorter than fixed header"), "{detail}");
            }
            other => panic!("unexpected: {other:?}"),
        }
    }

    #[test]
    fn define_count_larger_than_payload_is_malformed() {
        // Claims 27 datums but supplies only 1.
        let buf = simobject_data_buf(27, &[1.0], 0);
        match parse_record(&buf) {
            RecvRecord::Malformed { detail } => {
                assert!(detail.contains("truncated"), "{detail}");
            }
            other => panic!("unexpected: {other:?}"),
        }
    }

    #[test]
    fn absurd_define_count_does_not_overflow_and_is_malformed() {
        let buf = simobject_data_buf(u32::MAX, &[], 0);
        match parse_record(&buf) {
            RecvRecord::Malformed { detail } => {
                assert!(detail.contains("truncated"), "{detail}");
            }
            other => panic!("unexpected: {other:?}"),
        }
    }

    #[test]
    fn dw_size_exceeding_buffer_is_malformed() {
        let mut buf = simobject_data_buf(1, &[1.0], 0);
        // Claim the record is larger than the buffer actually delivered.
        buf[0..4].copy_from_slice(&u32::MAX.to_le_bytes());
        match parse_record(&buf) {
            RecvRecord::Malformed { detail } => {
                assert!(detail.contains("exceeds callback buffer"), "{detail}");
            }
            other => panic!("unexpected: {other:?}"),
        }
    }

    #[test]
    fn cb_data_smaller_than_required_is_malformed() {
        // Full record built, but the callback delivers fewer bytes.
        let full = simobject_data_buf(3, &[1.0, 2.0, 3.0], 0);
        let short = &full[..full.len() - 8];
        match parse_record(short) {
            RecvRecord::Malformed { detail } => {
                // dwSize still claims the full length -> caught by the size
                // claim check.
                assert!(detail.contains("exceeds callback buffer"), "{detail}");
            }
            other => panic!("unexpected: {other:?}"),
        }
    }

    #[test]
    fn event_record_decodes_pause_by_client_event_id() {
        match parse_record(&event_buf(EVT_PAUSE)) {
            RecvRecord::SystemEventPause(true) => {}
            other => panic!("unexpected: {other:?}"),
        }
        match parse_record(&event_buf(EVT_UNPAUSE)) {
            RecvRecord::SystemEventPause(false) => {}
            other => panic!("unexpected: {other:?}"),
        }
    }

    #[test]
    fn unknown_event_id_is_ignored_not_pause() {
        // A one-shot action ack must NEVER be interpreted as a pause toggle.
        match parse_record(&event_buf(EVT_ACTION)) {
            RecvRecord::Ignored => {}
            other => panic!("unexpected: {other:?}"),
        }
    }

    #[test]
    fn unknown_recv_kind_is_ignored() {
        let mut b = Vec::new();
        b.extend_from_slice(&12u32.to_le_bytes());
        b.extend_from_slice(&0u32.to_le_bytes());
        b.extend_from_slice(&9999u32.to_le_bytes()); // unknown id
        assert!(matches!(parse_record(&b), RecvRecord::Ignored));
    }
}
