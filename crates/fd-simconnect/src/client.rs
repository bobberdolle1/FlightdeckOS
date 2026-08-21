//! Thin unsafe FFI wrapper: connection lifecycle, data definitions, the
//! dispatch loop, and system-event subscriptions.
//!
//! This module owns the resolved [`SimConnectApi`]; everything above sees
//! decoded [`crate::parse::RecvRecord`]s.

use std::ffi::CString;

use fd_core::adapter::AdapterError;

use crate::defs::*;
use crate::ffi::{self, SimConnectApi};
use crate::parse::{RecvRecord, parse_recv};

/// Dispatch context shared with the C callback.
struct PollCtx {
    records: Vec<RecvRecord>,
}

unsafe extern "system" fn dispatch_cb(
    pdata: *mut ffi::SIMCONNECT_RECV,
    cb_data: ffi::DWORD,
    pcontext: *mut std::ffi::c_void,
) {
    if pdata.is_null() || pcontext.is_null() || cb_data == 0 {
        return;
    }
    let ctx = unsafe { &mut *(pcontext as *mut PollCtx) };
    // SAFETY: `pdata` points to `cb_data` initialized bytes delivered by
    // SimConnect_CallDispatch for this handle; parse_recv never reads beyond
    // cb_data (bounds are validated inside parse_record).
    let record = unsafe { parse_recv(pdata, cb_data) };
    ctx.records.push(record);
}

/// Owned SimConnect session.
pub struct SimClient {
    api: SimConnectApi,
    handle: ffi::HANDLE,
}

impl SimClient {
    /// Open a SimConnect connection to the local simulator.
    ///
    /// Fails with a typed error when the DLL is missing, the sim is not
    /// running, or the connection is refused.
    pub fn open(app_name: &str) -> Result<Self, AdapterError> {
        let api = SimConnectApi::load().map_err(AdapterError::ConnectionFailed)?;
        // SAFETY: SimConnect_Open writes the handle on success; all pointer
        // arguments are either valid or explicitly NULL per the SDK contract.
        unsafe {
            let mut handle: ffi::HANDLE = std::ptr::null_mut();
            let name = CString::new(app_name)
                .map_err(|_| AdapterError::ConnectionFailed("invalid app name".into()))?;
            let hr = (api.open)(
                &mut handle,
                name.as_ptr(),
                std::ptr::null_mut(),
                0,
                std::ptr::null_mut(),
                ffi::SIMCONNECT_OPEN_CONFIGINDEX_LOCAL,
            );
            if hr < 0 || handle.is_null() {
                Err(AdapterError::ConnectionFailed(format!(
                    "SimConnect_Open failed (hr=0x{hr:08X}). Is MSFS running?"
                )))
            } else {
                Ok(Self { api, handle })
            }
        }
    }

    /// Set up the telemetry data definition, the periodic request, and the
    /// pause/unpause system event subscriptions.
    pub fn setup(&mut self) -> Result<(), AdapterError> {
        // SAFETY: valid connected handle; all strings are NUL-terminated.
        unsafe {
            for (i, (name, units)) in DATUMS.iter().enumerate() {
                let name_c = CString::new(*name).expect("datum names have no interior NUL");
                let units_c = CString::new(*units).expect("unit names have no interior NUL");
                let hr = (self.api.add_to_data_definition)(
                    self.handle,
                    DEFINE_TELEMETRY,
                    name_c.as_ptr(),
                    units_c.as_ptr(),
                    ffi::SIMCONNECT_DATATYPE_FLOAT64,
                    0.0,
                    i as ffi::DWORD,
                );
                if hr < 0 {
                    return Err(AdapterError::ConnectionFailed(format!(
                        "AddToDataDefinition({name}) failed (hr=0x{hr:08X})"
                    )));
                }
            }

            // Per-frame delivery (no CHANGED flag): the runtime diff already
            // suppresses no-op deltas, and per-frame cadence matches the
            // phase engine's tick-based hysteresis semantics.
            let hr = (self.api.request_data_on_sim_object)(
                self.handle,
                REQUEST_TELEMETRY,
                DEFINE_TELEMETRY,
                ffi::SIMCONNECT_OBJECT_ID_USER,
                ffi::SIMCONNECT_PERIOD_SIM_FRAME,
                0,
                0,
                0,
                0,
            );
            if hr < 0 {
                return Err(AdapterError::ConnectionFailed(format!(
                    "RequestDataOnSimObject failed (hr=0x{hr:08X})"
                )));
            }

            for name in SYSTEM_EVENT_PAUSE_NAMES {
                let name_c = CString::new(*name).expect("event names have no interior NUL");
                let hr =
                    (self.api.subscribe_to_system_event)(self.handle, EVT_PAUSE, name_c.as_ptr());
                if hr < 0 {
                    return Err(AdapterError::ConnectionFailed(format!(
                        "SubscribeToSystemEvent({name}) failed (hr=0x{hr:08X})"
                    )));
                }
            }
            for name in SYSTEM_EVENT_UNPAUSE_NAMES {
                let name_c = CString::new(*name).expect("event names have no interior NUL");
                let hr =
                    (self.api.subscribe_to_system_event)(self.handle, EVT_UNPAUSE, name_c.as_ptr());
                if hr < 0 {
                    return Err(AdapterError::ConnectionFailed(format!(
                        "SubscribeToSystemEvent({name}) failed (hr=0x{hr:08X})"
                    )));
                }
            }
        }
        Ok(())
    }

    /// Drain all pending messages into decoded records.
    pub fn poll(&mut self) -> Vec<RecvRecord> {
        let mut ctx = PollCtx {
            records: Vec::new(),
        };
        // SAFETY: ctx outlives the dispatch loop; the callback only appends.
        unsafe {
            let ctx_ptr = &mut ctx as *mut PollCtx as *mut std::ffi::c_void;
            while (self.api.call_dispatch)(self.handle, Some(dispatch_cb), ctx_ptr) != 0 {}
        }
        ctx.records
    }

    /// Access for the write primitives.
    pub(crate) fn parts(&self) -> (&SimConnectApi, ffi::HANDLE) {
        (&self.api, self.handle)
    }
}

impl Drop for SimClient {
    fn drop(&mut self) {
        // SAFETY: closing an owned, valid handle.
        unsafe {
            (self.api.close)(self.handle);
        }
    }
}
