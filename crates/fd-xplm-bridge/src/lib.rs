//! fd-xplm-bridge — MINIMAL READ-ONLY X-Plane FMS bridge plugin (Task 7).
//!
//! Decision gate (Task 7 §6): decision **B**. The X-Plane Web API and
//! datarefs expose NO structured flight-plan state (verified live on
//! 12.4.3 + official dataref master list); the XPLM410 flight-plan
//! family (X-Plane >= 12.1.0) is callable ONLY from plugin code. This
//! plugin exists solely to observe that state and hand bounded JSON
//! snapshots to the FlightdeckOS host over loopback TCP.
//!
//! The plugin is NOT FlightdeckOS (§7). It owns only:
//! - SDK lifecycle (XPluginStart/Stop/Enable/Disable/ReceiveMessage);
//! - FMS read operations on the simulator thread (§42);
//! - serialization of bounded snapshots.
//!
//! It owns NO FDR/FDM/routes/OpenAIRAC/SOP/Mission/AI and performs NO
//! writes: the FFI surface below declares ONLY read functions — the
//! XPLM setters do not exist for this plugin at compile time (§43).
//!
//! Safety contract (§41):
//! - No Rust panic crosses the C ABI: every exported function and the
//!   flight-loop callback body run inside `catch_unwind`.
//! - All XPLM calls happen on the thread that called the plugin (SDK
//!   rule; hard-asserted by X-Plane 12.4.1+): FMS reads and
//!   XPLMGetVersions happen ONLY on the simulator thread; the worker
//!   thread touches only the snapshot mutex and the TCP socket.
//! - Host absence/disconnect never affects X-Plane: the worker treats
//!   every socket error as "back to accept", bounded and non-fatal.
//! - Plugin failure never crashes the host: the host reconnects.

use std::ffi::c_char;
use std::io::Write as _;
use std::net::{TcpListener, TcpStream};
use std::sync::atomic::{AtomicBool, AtomicI32, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Duration;

/// Protocol version (§8: versioned). Bumped on incompatible wire changes.
pub const PROTO_VERSION: u32 = 1;

/// Default loopback port (§8: machine-local only). Override with the
/// `FD_FMS_BRIDGE_PORT` environment variable read by XPluginEnable.
pub const DEFAULT_PORT: u16 = 57501;

/// Maximum serialized snapshot line the plugin will emit (§8: bounded
/// messages; 100 entries/plan keeps this far below).
pub const MAX_LINE_BYTES: usize = 256 * 1024;

// ---------------------------------------------------------------------------
// XPLM FFI — READ-ONLY SURFACE (§43: no setter is declared at all)
// ---------------------------------------------------------------------------

const XPLM_NAV_NOT_FOUND: i32 = -1;

/// XPLMNavType values are passed through to the host UNTRANSLATED
/// (bitmask per XPLMNavigation.h: 1=Airport, 2=NDB, 4=VOR, 8=ILS,
/// 16=Localizer, 32=GlideSlope, 512=Fix, 1024=DME, 2048=LatLon,
/// 4096=TACAN, 0=Unknown). The host maps them into the canonical model;
/// the plugin stays a dumb serializer (§7).
mod nav_type {
    pub const UNKNOWN: i32 = 0;
}

/// XPLMNavFlightPlan selectors (XPLM410).
mod fpl {
    pub const PILOT_PRIMARY: i32 = 0;
    pub const PILOT_APPROACH: i32 = 2;
    pub const PILOT_TEMPORARY: i32 = 4;
}

unsafe extern "C" {
    // Legacy (ungated) FMS reads.
    fn XPLMCountFMSEntries() -> i32;
    fn XPLMGetDisplayedFMSEntry() -> i32;
    fn XPLMGetDestinationFMSEntry() -> i32;
    fn XPLMGetFMSEntryInfo(
        index: i32,
        out_type: *mut i32,
        out_id: *mut c_char,
        out_ref: *mut i32,
        out_altitude: *mut i32,
        out_lat: *mut f32,
        out_lon: *mut f32,
    );
    // XPLM410 per-plan reads.
    fn XPLMCountFMSFlightPlanEntries(plan: i32) -> i32;
    fn XPLMGetDestinationFMSFlightPlanEntry(plan: i32) -> i32;
    fn XPLMGetDisplayedFMSFlightPlanEntry(plan: i32) -> i32;
    fn XPLMGetFMSFlightPlanEntryInfo(
        plan: i32,
        index: i32,
        out_type: *mut i32,
        out_id: *mut c_char,
        out_ref: *mut i32,
        out_altitude: *mut i32,
        out_lat: *mut f32,
        out_lon: *mut f32,
    );
    // Lifecycle/support.
    fn XPLMRegisterFlightLoopCallback(
        callback: FlightLoopFn,
        interval: f32,
        refcon: *mut std::ffi::c_void,
    );
    fn XPLMUnregisterFlightLoopCallback(callback: FlightLoopFn, refcon: *mut std::ffi::c_void);
    fn XPLMGetVersions(out_xplane: *mut i32, out_xplm: *mut i32, out_host: *mut i32);
}

type FlightLoopFn = unsafe extern "C" fn(f32, f32, i32, *mut std::ffi::c_void) -> f32;

// ---------------------------------------------------------------------------
// Shared state
// ---------------------------------------------------------------------------

/// The latest serialized snapshot. Written ONLY on the simulator thread
/// (flight-loop callback); read by the worker thread. `versions` is
/// captured ON the simulator thread in XPluginEnable (§42).
struct Shared {
    snapshot: Mutex<Option<String>>,
    /// Captured ON the simulator thread in XPluginEnable (§42).
    xplane_version: AtomicI32,
    xplm_version: AtomicI32,
    shutdown: AtomicBool,
}

static STATE: Mutex<Option<Arc<Shared>>> = Mutex::new(None);

/// Publisher worker JoinHandle; joined in XPluginStop so the DLL is
/// never unloaded under a live thread (Task 7.1 review BLOCKER).
static WORKER: Mutex<Option<std::thread::JoinHandle<()>>> = Mutex::new(None);

// ---------------------------------------------------------------------------
// FMS reading (simulator thread only)
// ---------------------------------------------------------------------------

/// Escape a nav identifier into a bounded, safe JSON string body.
/// Non-ASCII/control bytes become `?` — identifiers are informational.
fn escape_id(id: &[u8]) -> String {
    let mut s = String::with_capacity(id.len());
    let trimmed: &[u8] = &id[..id.iter().position(|&b| b == 0).unwrap_or(id.len())];
    for &b in trimmed {
        if b == b'\\' || b == b'"' || !(0x20..0x7f).contains(&b) {
            s.push('?');
        } else {
            s.push(b as char);
        }
    }
    s
}

/// Read one FMS entry and append its JSON object. `plan` is `None` for
/// the legacy "THE FMS" surface.
///
/// SAFETY: XPLM call — simulator thread only. Buffers are fixed-size;
/// `nav_ref` is initialized to XPLM_NAV_NOT_FOUND per the SDK contract
/// (the SDK writes through all out-pointers regardless).
unsafe fn read_entry_json(plan: Option<i32>, index: i32, out: &mut String) {
    let mut ty: i32 = nav_type::UNKNOWN;
    let mut nav_ref: i32 = XPLM_NAV_NOT_FOUND;
    let mut alt: i32 = 0;
    let mut lat: f32 = 0.0;
    let mut lon: f32 = 0.0;
    let mut id = [0 as c_char; 256];
    match plan {
        None => unsafe {
            XPLMGetFMSEntryInfo(
                index,
                &mut ty,
                id.as_mut_ptr(),
                &mut nav_ref,
                &mut alt,
                &mut lat,
                &mut lon,
            );
        },
        Some(p) => unsafe {
            XPLMGetFMSFlightPlanEntryInfo(
                p,
                index,
                &mut ty,
                id.as_mut_ptr(),
                &mut nav_ref,
                &mut alt,
                &mut lat,
                &mut lon,
            );
        },
    }
    let id_bytes: Vec<u8> = id.iter().map(|&c| c as u8).collect();
    let lat_val: f64 = lat as f64;
    let lon_val: f64 = lon as f64;
    out.push_str(&format!(
        "{{\"ty\":{},\"id\":\"{}\",\"lat\":{:.6},\"lon\":{:.6},\"alt\":{},\"nav\":{}}}",
        ty,
        escape_id(&id_bytes),
        lat_val,
        lon_val,
        if alt > 0 {
            alt.to_string()
        } else {
            "null".to_string()
        },
        if nav_ref != XPLM_NAV_NOT_FOUND {
            "true"
        } else {
            "false"
        },
    ));
}

fn json_num(v: i32) -> String {
    if v < 0 {
        "null".to_string()
    } else {
        v.to_string()
    }
}

/// Serialize the legacy "THE FMS" plan into `out`.
///
/// SAFETY: XPLM calls — simulator thread only.
unsafe fn read_legacy_plan_json(out: &mut String) {
    let count = unsafe { XPLMCountFMSEntries() };
    if !(0..=100).contains(&count) {
        out.push_str("{\"entries\":[],\"dest\":null,\"disp\":null,\"count\":null}");
        return;
    }
    let (dest, disp) = unsafe { (XPLMGetDestinationFMSEntry(), XPLMGetDisplayedFMSEntry()) };
    out.push_str("{\"entries\":[");
    for i in 0..count {
        if i > 0 {
            out.push(',');
        }
        unsafe { read_entry_json(None, i, out) };
    }
    out.push_str(&format!(
        "],\"dest\":{},\"disp\":{},\"count\":{}}}",
        json_num(dest),
        json_num(disp),
        count
    ));
}

/// Serialize one XPLM410 plan into `out` as `{"plan":{...}}` or `null`
/// when the device does not provide it (count < 0).
///
/// SAFETY: XPLM calls — simulator thread only.
unsafe fn read_fpl_plan_json(sel: i32, out: &mut String) {
    let count = unsafe { XPLMCountFMSFlightPlanEntries(sel) };
    if count < 0 {
        out.push_str("null");
        return;
    }
    if count > 100 {
        out.push_str("{\"entries\":[],\"dest\":null,\"disp\":null,\"count\":null}");
        return;
    }
    let (dest, disp) = unsafe {
        (
            XPLMGetDestinationFMSFlightPlanEntry(sel),
            XPLMGetDisplayedFMSFlightPlanEntry(sel),
        )
    };
    out.push_str("{\"entries\":[");
    for i in 0..count {
        if i > 0 {
            out.push(',');
        }
        unsafe { read_entry_json(Some(sel), i, out) };
    }
    out.push_str(&format!(
        "],\"dest\":{},\"disp\":{},\"count\":{}}}",
        json_num(dest),
        json_num(disp),
        count
    ));
}

/// Build the full snapshot line. Called ONLY from the flight-loop
/// callback (§42).
fn build_snapshot(versions: (i32, i32)) -> Option<String> {
    let mut s = String::with_capacity(4096);
    s.push_str(&format!(
        "{{\"proto\":{},\"kind\":\"fms\",\"xplane\":{},\"xplm\":{},\"plans\":{{\"legacy\":",
        PROTO_VERSION, versions.0, versions.1
    ));
    unsafe {
        read_legacy_plan_json(&mut s);
        s.push_str(",\"primary\":");
        read_fpl_plan_json(fpl::PILOT_PRIMARY, &mut s);
        s.push_str(",\"approach\":");
        read_fpl_plan_json(fpl::PILOT_APPROACH, &mut s);
        s.push_str(",\"temporary\":");
        read_fpl_plan_json(fpl::PILOT_TEMPORARY, &mut s);
    }
    s.push_str("}}");
    if s.len() > MAX_LINE_BYTES {
        return None; // never emit unbounded lines (§8)
    }
    Some(s)
}

/// Flight-loop callback — SIMULATOR THREAD ONLY (§42). Reads FMS state,
/// publishes on change. Returns the next interval in seconds.
unsafe extern "C" fn flight_loop(_e: f32, _s: f32, _c: i32, _r: *mut std::ffi::c_void) -> f32 {
    let next_interval = 1.0_f32;
    let _ = std::panic::catch_unwind(|| {
        let shared = STATE.lock().ok()?.clone()?;
        let snapshot = build_snapshot((
            shared.xplane_version.load(Ordering::Relaxed),
            shared.xplm_version.load(Ordering::Relaxed),
        ));
        if let Some(next) = snapshot {
            let mut guard = shared.snapshot.lock().ok()?;
            if guard.as_deref() != Some(next.as_str()) {
                *guard = Some(next);
            }
        }
        Some(())
    });
    // Never propagate panics across the C ABI (§41); always reschedule.
    next_interval
}

// ---------------------------------------------------------------------------
// Worker thread: loopback TCP publisher (§8)
// ---------------------------------------------------------------------------

fn worker_main(shared: Arc<Shared>, port: u16) {
    let listener = match TcpListener::bind(("127.0.0.1", port)) {
        Ok(l) => l,
        Err(_) => {
            // Port unavailable: retry slowly; X-Plane must never crash
            // because of the bridge (§8). Chunked sleeps keep XPluginStop's
            // join bounded (~100 ms past the shutdown flag).
            loop {
                if shared.shutdown.load(Ordering::Relaxed) {
                    return;
                }
                for _ in 0..50 {
                    if shared.shutdown.load(Ordering::Relaxed) {
                        return;
                    }
                    std::thread::sleep(Duration::from_millis(100));
                }
                if let Ok(l) = TcpListener::bind(("127.0.0.1", port)) {
                    worker_accept(&shared, l);
                    return;
                }
            }
        }
    };
    worker_accept(&shared, listener)
}

fn worker_accept(shared: &Arc<Shared>, listener: TcpListener) {
    listener.set_nonblocking(true).ok();
    loop {
        if shared.shutdown.load(Ordering::Relaxed) {
            return;
        }
        match listener.accept() {
            Ok((stream, _addr)) => serve_client(shared, stream),
            Err(ref e) if e.kind() == std::io::ErrorKind::WouldBlock => {
                std::thread::sleep(Duration::from_millis(200));
            }
            Err(_) => {
                std::thread::sleep(Duration::from_secs(1));
            }
        }
    }
}

/// Serve one host connection: handshake, then snapshot lines on change
/// plus a 10 s heartbeat. All failures -> connection closed, plugin
/// keeps running (§8). The socket is never read (§8: no
/// client-initiated operations).
fn serve_client(shared: &Arc<Shared>, mut stream: TcpStream) {
    stream.set_write_timeout(Some(Duration::from_secs(5))).ok();
    let handshake = format!(
        "{{\"proto\":{},\"kind\":\"hello\",\"xplane\":{},\"xplm\":{}}}\n",
        PROTO_VERSION,
        shared.xplane_version.load(Ordering::Relaxed),
        shared.xplm_version.load(Ordering::Relaxed)
    );
    if stream.write_all(handshake.as_bytes()).is_err() {
        return;
    }
    let _ = stream.flush();
    let mut last_sent: Option<String> = None;
    let mut ticks_since_send = 0u32;
    loop {
        if shared.shutdown.load(Ordering::Relaxed) {
            return;
        }
        for _ in 0..25 {
            if shared.shutdown.load(Ordering::Relaxed) {
                return;
            }
            std::thread::sleep(Duration::from_millis(10));
        }
        ticks_since_send += 1;
        let current = shared.snapshot.lock().ok().and_then(|g| g.clone());
        let heartbeat_due = ticks_since_send >= 40; // 40 * 250ms = 10 s
        match (&current, &last_sent) {
            (Some(cur), Some(sent)) if cur == sent && !heartbeat_due => continue,
            (None, _) if !heartbeat_due => continue,
            _ => {}
        }
        let line = match &current {
            Some(cur) => format!("{}\n", cur),
            None => format!(
                "{{\"proto\":{},\"kind\":\"fms\",\"xplane\":{},\"xplm\":{},\"plans\":{{}}}}\n",
                PROTO_VERSION,
                shared.xplane_version.load(Ordering::Relaxed),
                shared.xplm_version.load(Ordering::Relaxed)
            ),
        };
        if stream.write_all(line.as_bytes()).is_err() {
            return;
        }
        let _ = stream.flush();
        last_sent = current;
        ticks_since_send = 0;
    }
}

// ---------------------------------------------------------------------------
// Plugin lifecycle exports (C ABI; panics never cross — §41)
// ---------------------------------------------------------------------------

/// SAFETY: `dst` points at a 256-byte buffer provided by X-Plane.
///
/// Not marked `unsafe fn`: callers already hold `unsafe {}` blocks and
/// clippy's `not_unsafe_ptr_arg_deref` fires on the exported-call path
/// otherwise; the raw-pointer contract is documented here.
fn write_cstr(dst: *mut c_char, src: &str) {
    let buf = unsafe { std::slice::from_raw_parts_mut(dst as *mut u8, 256) };
    let bytes = src.as_bytes();
    let n = bytes.len().min(254);
    buf[..n].copy_from_slice(&bytes[..n]);
    buf[n] = 0;
}

/// XPluginStart: identify the plugin and create shared state.
#[unsafe(no_mangle)]
pub extern "C" fn XPluginStart(
    out_name: *mut c_char,
    out_sig: *mut c_char,
    out_desc: *mut c_char,
) -> i32 {
    let r = std::panic::catch_unwind(|| {
        write_cstr(out_name, "FlightdeckOS FMS Bridge");
        write_cstr(out_sig, "flightdeckos.fms.bridge");
        write_cstr(
            out_desc,
            "Read-only FMS observation bridge for FlightdeckOS (zero writes)",
        );
        let shared = Arc::new(Shared {
            snapshot: Mutex::new(None),
            xplane_version: AtomicI32::new(0),
            xplm_version: AtomicI32::new(0),
            shutdown: AtomicBool::new(false),
        });
        match STATE.lock() {
            Ok(mut guard) => {
                *guard = Some(shared);
                1
            }
            Err(_) => 0,
        }
    });
    r.unwrap_or(0)
}

/// XPluginEnable: capture versions ON this thread, register the flight
/// loop, start the publisher worker.
#[unsafe(no_mangle)]
pub extern "C" fn XPluginEnable() -> i32 {
    let r = std::panic::catch_unwind(|| {
        let Some(shared) = STATE.lock().ok().and_then(|g| g.clone()) else {
            return 0;
        };
        // Simulator thread: the ONLY place SDK calls are legal (§42).
        let mut xplane_ver: i32 = 0;
        let mut xplm_ver: i32 = 0;
        let mut host: i32 = 0;
        unsafe {
            XPLMGetVersions(&mut xplane_ver, &mut xplm_ver, &mut host);
        }
        shared.xplane_version.store(xplane_ver, Ordering::Relaxed);
        shared.xplm_version.store(xplm_ver, Ordering::Relaxed);
        shared.shutdown.store(false, Ordering::Relaxed);
        unsafe {
            XPLMRegisterFlightLoopCallback(flight_loop, 1.0, std::ptr::null_mut());
        }
        let port = std::env::var("FD_FMS_BRIDGE_PORT")
            .ok()
            .and_then(|v| v.parse().ok())
            .unwrap_or(DEFAULT_PORT);
        let worker_shared = shared.clone();
        let spawned = std::thread::Builder::new()
            .name("fd-xplm-bridge".into())
            .spawn(move || worker_main(worker_shared, port));
        match spawned {
            Ok(handle) => {
                if let Ok(mut w) = WORKER.lock() {
                    *w = Some(handle);
                }
                1
            }
            Err(_) => {
                // Spawn failed after the flight loop was registered:
                // unregister before reporting failure (Task 7.1 review).
                unsafe {
                    XPLMUnregisterFlightLoopCallback(flight_loop, std::ptr::null_mut());
                }
                0
            }
        }
    });
    r.unwrap_or(0)
}

/// XPluginDisable: stop publishing, unregister the callback.
#[unsafe(no_mangle)]
pub extern "C" fn XPluginDisable() {
    let _ = std::panic::catch_unwind(|| {
        if let Some(shared) = STATE.lock().ok().and_then(|g| g.clone()) {
            shared.shutdown.store(true, Ordering::Relaxed);
            if let Ok(mut guard) = shared.snapshot.lock() {
                *guard = None;
            }
        }
        unsafe {
            XPLMUnregisterFlightLoopCallback(flight_loop, std::ptr::null_mut());
        }
    });
}

#[unsafe(no_mangle)]
pub extern "C" fn XPluginStop() {
    let _ = std::panic::catch_unwind(|| {
        XPluginDisable();
        // Join the publisher worker before the DLL can be unloaded
        // (Task 7.1 review BLOCKER): FreeLibrary under a live thread
        // crashes the simulator on Reload Plugins. Shutdown-aware
        // sleeps bound the wait to well under a second; the 5 s write
        // timeout bounds the pathological client case.
        if let Ok(mut w) = WORKER.lock()
            && let Some(handle) = w.take()
        {
            let _ = handle.join();
        }
        if let Ok(mut guard) = STATE.lock() {
            *guard = None;
        }
    });
}

/// Signature per XPLMPlugin.h: (XPLMPluginID inFromWho, int inMessage,
/// void* inParam). XPLMPluginID is `int`.
#[unsafe(no_mangle)]
pub extern "C" fn XPluginReceiveMessage(_from: i32, _msg: i32, _param: *mut std::ffi::c_void) {
    // No messages are acted upon; aircraft-reload plan invalidation is a
    // HOST-side concern driven by snapshot revision changes (§39).
}
