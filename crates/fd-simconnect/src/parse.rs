//! Pure decoding of SimConnect RECV records (no FFI calls — unit-testable).
//!
//! `SIMCONNECT_RECV*` structs are `#pragma pack(1)`: all field access goes
//! through unaligned reads; the `FLOAT64` payload may also be unaligned.

use crate::defs::*;
use crate::ffi;

/// A decoded record from the dispatch callback.
#[derive(Debug, Clone, PartialEq)]
pub enum RecvRecord {
    /// Periodic telemetry payload (length already bounded).
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
}

/// Decode one `SIMCONNECT_RECV*` pointer.
///
/// # Safety
/// `p` must point to a valid record delivered by `SimConnect_CallDispatch`.
pub unsafe fn parse_recv(p: *const ffi::SIMCONNECT_RECV) -> RecvRecord {
    // SAFETY: caller guarantees `p` points at a valid delivered record;
    // all reads below are unaligned-safe per the packed SDK layout.
    unsafe {
        let id = (*p).dwID;
        if id == ffi::SIMCONNECT_RECV_ID_SIMOBJECT_DATA {
            let q = p as *const ffi::SIMCONNECT_RECV_SIMOBJECT_DATA;
            let count = (*q).dwDefineCount as usize;
            // `dwData` is a single DWORD placeholder (SIMCONNECT_DATAV expands to
            // `DWORD dwData;`); the FLOAT64 payload begins immediately after it.
            let data_base = (q as *const u8).add(ffi::SIMOBJECT_DATA_PAYLOAD_OFFSET);
            let n = count.min(DATUM_COUNT);
            let mut values = Vec::with_capacity(n);
            for i in 0..n {
                values.push(std::ptr::read_unaligned(data_base.add(i * 8) as *const f64));
            }
            RecvRecord::SimObjectData { values }
        } else if id == ffi::SIMCONNECT_RECV_ID_EVENT {
            let q = p as *const ffi::SIMCONNECT_RECV_EVENT;
            let event = (*q).uEventID;
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
            let q = p as *const ffi::SIMCONNECT_RECV_EXCEPTION;
            RecvRecord::Exception {
                code: (*q).dwException,
            }
        } else if id == ffi::SIMCONNECT_RECV_ID_OPEN {
            RecvRecord::Open
        } else if id == ffi::SIMCONNECT_RECV_ID_QUIT {
            RecvRecord::Quit
        } else {
            RecvRecord::Ignored
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Construct a packed SIMCONNECT_RECV_SIMOBJECT_DATA record in memory
    /// mirroring the SDK layout (packed, DWORD dwData placeholder).
    #[repr(C, packed)]
    struct FakeRecv {
        dw_size: u32,
        dw_version: u32,
        dw_id: u32,
        dw_request_id: u32,
        dw_object_id: u32,
        dw_define_id: u32,
        dw_flags: u32,
        dw_entry_number: u32,
        dw_out_of: u32,
        dw_define_count: u32,
        dw_data: u32,
        // f64 payload follows
    }

    #[test]
    fn fake_layout_matches_sdk_layout() {
        assert_eq!(std::mem::size_of::<FakeRecv>(), 44);
    }

    #[test]
    fn parses_simobject_data_with_unaligned_f64_payload() {
        let values = [1.5f64, 2.5, 3.5];
        let rec = FakeRecv {
            dw_size: 0,
            dw_version: 0,
            dw_id: ffi::SIMCONNECT_RECV_ID_SIMOBJECT_DATA,
            dw_request_id: REQUEST_TELEMETRY,
            dw_object_id: 0,
            dw_define_id: DEFINE_TELEMETRY,
            dw_flags: 0,
            dw_entry_number: 1,
            dw_out_of: 1,
            dw_define_count: values.len() as u32,
            dw_data: 0,
        };

        // Allocate a packed buffer: struct + payload.
        let total = std::mem::size_of::<FakeRecv>() + values.len() * 8;
        let mut buf = vec![0u8; total];
        unsafe {
            let head = &rec as *const FakeRecv as *const u8;
            let payload = values.as_ptr() as *const u8;
            std::ptr::copy_nonoverlapping(head, buf.as_mut_ptr(), std::mem::size_of::<FakeRecv>());
            std::ptr::copy_nonoverlapping(
                payload,
                buf.as_mut_ptr().add(std::mem::size_of::<FakeRecv>()),
                values.len() * 8,
            );
        }

        let record = unsafe { parse_recv(buf.as_ptr() as *const ffi::SIMCONNECT_RECV) };
        match record {
            RecvRecord::SimObjectData { values: got } => {
                assert_eq!(got, values.to_vec());
            }
            other => panic!("unexpected record: {other:?}"),
        }
    }
}
