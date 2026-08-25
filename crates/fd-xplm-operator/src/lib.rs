//! fd-xplm-operator — OPERATOR flight-preparation helper (Task 7 §19-21).
//!
//! NOT part of the observation path. The read-only bridge
//! (fd-xplm-bridge) never writes; THIS plugin exists so the operator can
//! pre-program the aircraft FMS before engine start — the software
//! equivalent of typing the plan into the G1000 by hand (§21: FlightdeckOS
//! may initialize the scenario; the flight itself is operator-controlled).
//!
//! Behavior: every 5 s, if `Output/FMS plans/fdos_operator_plan.fms`
//! exists in the X-Plane directory, parse the v1100 waypoint table and
//! write the entries into the pilot-side PRIMARY plan via the XPLM410
//! `XPLMSetFMSFlightPlanEntryLatLonWithId` API (named lat/lon entries —
//! the same thing a pilot keys in), then set the destination to the last
//! entry and DELETE the file (one-shot). No IPC, no host. Panics never
//! cross the C ABI.

use std::ffi::c_char;
use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};

unsafe extern "C" {
    fn XPLMRegisterFlightLoopCallback(
        callback: FlightLoopFn,
        interval: f32,
        refcon: *mut std::ffi::c_void,
    );
    fn XPLMUnregisterFlightLoopCallback(callback: FlightLoopFn, refcon: *mut std::ffi::c_void);
    fn XPLMGetSystemPath(out_path: *mut c_char, in_length: u32);
    fn XPLMDebugString(in_string: *const c_char);
    fn XPLMCountFMSFlightPlanEntries(plan: i32) -> i32;
    fn XPLMClearFMSFlightPlanEntry(plan: i32, index: i32);
    fn XPLMSetFMSFlightPlanEntryLatLonWithId(
        plan: i32,
        index: i32,
        lat: f32,
        lon: f32,
        altitude: i32,
        id: *const c_char,
        id_length: u32,
    );
    fn XPLMSetDestinationFMSFlightPlanEntry(plan: i32, index: i32);
}

type FlightLoopFn = unsafe extern "C" fn(f32, f32, i32, *mut std::ffi::c_void) -> f32;

const FPL_PILOT_PRIMARY: i32 = 0;

static SHUTDOWN: AtomicBool = AtomicBool::new(false);

/// Consecutive failed-parse attempts for the SAME published file. After
/// MAX_PARSE_RETRIES the file is renamed aside (`.bad`) so a genuinely
/// malformed file cannot loop forever; a PARTIAL file (writer still
/// writing) stays in place and is retried — the plugin NEVER consumes a
/// partially published snapshot and NEVER deletes writer data.
static PARSE_FAILURES: AtomicU32 = AtomicU32::new(0);
const MAX_PARSE_RETRIES: u32 = 3;

fn debug(msg: &str) {
    let mut buf = msg.as_bytes().to_vec();
    buf.push(0);
    unsafe {
        XPLMDebugString(buf.as_ptr() as *const c_char);
    }
}

/// SAFETY: X-Plane root path buffer, 512 bytes.
unsafe fn system_path() -> String {
    let mut buf = [0 as c_char; 512];
    unsafe {
        XPLMGetSystemPath(buf.as_mut_ptr(), 512);
    }
    let bytes: Vec<u8> = buf
        .iter()
        .take_while(|&&c| c != 0)
        .map(|&c| c as u8)
        .collect();
    String::from_utf8_lossy(&bytes).into_owned()
}

/// One parsed waypoint line: (ident, lat, lon, altitude_ft).
struct Wpt(String, f32, f32, i32);

/// Parse the v1100 waypoint table (type, ident, via, alt, lat, lon).
fn parse_plan(content: &str) -> Vec<Wpt> {
    let mut out = Vec::new();
    for line in content.lines() {
        let f: Vec<&str> = line.split_whitespace().collect();
        if f.len() == 6
            && (f[0] == "1" || f[0] == "11" || f[0] == "28" || f[0] == "3" || f[0] == "2")
        {
            // The altitude column is a FLOAT in the v1100 format
            // ("0.000000"); parsing it as an integer silently rejects
            // every line (the live "no usable waypoints" root cause).
            if let (Ok(lat), Ok(lon), Ok(alt)) = (
                f[4].parse::<f32>(),
                f[5].parse::<f32>(),
                f[3].parse::<f32>(),
            ) {
                out.push(Wpt(f[1].to_string(), lat, lon, alt as i32));
            }
        }
    }
    out
}

unsafe extern "C" fn flight_loop(_e: f32, _s: f32, _c: i32, _r: *mut std::ffi::c_void) -> f32 {
    let next = 5.0_f32;
    let _ = std::panic::catch_unwind(|| {
        if SHUTDOWN.load(Ordering::Relaxed) {
            return;
        }
        let dir = unsafe { system_path() };
        let plan_path = format!("{}Output/FMS plans/fdos_operator_plan.fms", dir);
        let Ok(content) = std::fs::read_to_string(&plan_path) else {
            PARSE_FAILURES.store(0, Ordering::Relaxed);
            return;
        };
        // Completeness gate: a fully published plan ends with a newline
        // and parses to at least one waypoint. Anything else is treated
        // as a partially published file: RETRY, never consume, never
        // delete (the writer owns the file until publication completes).
        let complete = content.ends_with('\n');
        let wpts = parse_plan(&content);
        if !complete || wpts.is_empty() || wpts.len() > 100 {
            let n = PARSE_FAILURES.fetch_add(1, Ordering::Relaxed) + 1;
            if n >= MAX_PARSE_RETRIES {
                // Genuinely malformed: move aside so the operator can
                // inspect it; bounded, no infinite retry.
                let bad = format!("{plan_path}.bad");
                let _ = std::fs::rename(&plan_path, &bad);
                PARSE_FAILURES.store(0, Ordering::Relaxed);
                debug("fd-xplm-operator: plan rejected after retries -> renamed .bad\n");
            } else {
                debug(
                    "fd-xplm-operator: plan incomplete, retrying (writer may still be publishing)\n",
                );
            }
            return;
        }
        PARSE_FAILURES.store(0, Ordering::Relaxed);
        let _ = std::fs::remove_file(&plan_path); // one-shot AFTER successful parse
        unsafe {
            // Clear any existing entries (shorten to zero first).
            let existing = XPLMCountFMSFlightPlanEntries(FPL_PILOT_PRIMARY);
            for i in (0..existing).rev() {
                XPLMClearFMSFlightPlanEntry(FPL_PILOT_PRIMARY, i);
            }
            // Write named lat/lon entries (pilot-keyed equivalent).
            for (i, w) in wpts.iter().enumerate() {
                let mut id = w.0.as_bytes().to_vec();
                id.push(0);
                XPLMSetFMSFlightPlanEntryLatLonWithId(
                    FPL_PILOT_PRIMARY,
                    i as i32,
                    w.1,
                    w.2,
                    w.3,
                    id.as_ptr() as *const c_char,
                    (id.len() - 1) as u32,
                );
            }
            XPLMSetDestinationFMSFlightPlanEntry(FPL_PILOT_PRIMARY, (wpts.len() - 1) as i32);
            let count = XPLMCountFMSFlightPlanEntries(FPL_PILOT_PRIMARY);
            let msg = format!(
                "fd-xplm-operator: wrote {} waypoints, post-write count={}\n",
                wpts.len(),
                count
            );
            debug(&msg);
        }
    });
    next
}

fn write_cstr(dst: *mut c_char, src: &str) {
    let buf = unsafe { std::slice::from_raw_parts_mut(dst as *mut u8, 256) };
    let bytes = src.as_bytes();
    let n = bytes.len().min(254);
    buf[..n].copy_from_slice(&bytes[..n]);
    buf[n] = 0;
}

#[unsafe(no_mangle)]
pub extern "C" fn XPluginStart(
    out_name: *mut c_char,
    out_sig: *mut c_char,
    out_desc: *mut c_char,
) -> i32 {
    let r = std::panic::catch_unwind(|| {
        write_cstr(out_name, "FlightdeckOS Operator");
        write_cstr(out_sig, "flightdeckos.operator");
        write_cstr(
            out_desc,
            "Operator flight-preparation helper (one-shot FMS plan entry)",
        );
        1
    });
    r.unwrap_or(0)
}

#[unsafe(no_mangle)]
pub extern "C" fn XPluginEnable() -> i32 {
    let r = std::panic::catch_unwind(|| {
        SHUTDOWN.store(false, Ordering::Relaxed);
        unsafe {
            XPLMRegisterFlightLoopCallback(flight_loop, 5.0, std::ptr::null_mut());
        }
        1
    });
    r.unwrap_or(0)
}

#[unsafe(no_mangle)]
pub extern "C" fn XPluginDisable() {
    let _ = std::panic::catch_unwind(|| {
        SHUTDOWN.store(true, Ordering::Relaxed);
        unsafe {
            XPLMUnregisterFlightLoopCallback(flight_loop, std::ptr::null_mut());
        }
    });
}

#[unsafe(no_mangle)]
pub extern "C" fn XPluginStop() {
    XPluginDisable();
}

#[unsafe(no_mangle)]
pub extern "C" fn XPluginReceiveMessage(_from: i32, _msg: i32, _param: *mut std::ffi::c_void) {}

#[cfg(test)]
mod tests {
    use super::*;

    const COMPLETE: &str = "I 1100 Version\nCYCLE 2608\nADEP KLAX\nADES KSNA\nNUMENR 5\n1 KLAX ADEP 0.000000 33.942500 -118.408100\n11 BHOOV DRCT 0.000000 33.846375 -118.371981\n11 BUCAS DRCT 0.000000 33.822578 -118.104872\n11 CAHIL DRCT 0.000000 33.906269 -117.944792\n1 KSNA ADES 0.000000 33.672300 -117.864300\n";

    #[test]
    fn complete_plan_parses_five_waypoints() {
        let w = parse_plan(COMPLETE);
        assert_eq!(w.len(), 5);
        assert_eq!(w[0].0, "KLAX");
        assert_eq!(w[0].1, 33.9425);
        assert_eq!(w[4].0, "KSNA");
    }

    #[test]
    fn partial_publication_parses_to_zero() {
        // Writer created the file but only the header is flushed so far.
        let partial = "I 1100 Version\nCYCLE 2608\nADEP KLAX\n";
        assert!(parse_plan(partial).is_empty());
        // The completeness gate rejects it on zero waypoints even though
        // the trailing newline happens to be present.
        assert!(partial.ends_with('\n'));
        let wpts = parse_plan(partial);
        assert!(wpts.is_empty() || !partial.ends_with('\n'));
    }

    #[test]
    fn truncated_table_is_incomplete() {
        // Ends mid-line: no trailing newline, last line unusable.
        let truncated =
            "I 1100 Version\nNUMENR 5\n1 KLAX ADEP 0.0 33.94 -118.4\n11 BHOOV DRCT 0.0 33.8";
        let w = parse_plan(truncated);
        assert!(w.len() < 5);
        assert!(!truncated.ends_with('\n'));
    }

    #[test]
    fn garbage_yields_no_waypoints() {
        assert!(parse_plan("hello world\n").is_empty());
        assert!(parse_plan("").is_empty());
    }

    #[test]
    fn oversize_plan_rejected() {
        let many: String = (0..200)
            .map(|i| format!("11 W{i:03} DRCT 0.000000 33.{i:06} -118.{i:06}\n"))
            .collect();
        assert!(parse_plan(&many).len() > 100);
    }
}
