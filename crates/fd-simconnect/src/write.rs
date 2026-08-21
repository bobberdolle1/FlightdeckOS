//! Raw write primitives — `pub(crate)`: the closed-action write path.
//!
//! These are the ONLY functions in the workspace that perform simulator
//! writes. They are deliberately private to the crate; the public surface
//! accepts only [`fd_core::actions::CockpitAction`] values through
//! [`crate::SimConnectAdapter`].

use std::ffi::CString;

use fd_core::adapter::AdapterError;

use crate::ffi;

use crate::client::SimClient;
use crate::defs::{DEFINE_WRITE, EVT_ACTION};

/// Write one settable variable via a fresh data definition.
///
/// # Safety
/// Caller guarantees the client is connected.
pub(crate) unsafe fn write_simvar(
    client: &SimClient,
    name: &str,
    unit: &str,
    value: f64,
) -> Result<(), AdapterError> {
    let (api, handle) = client.parts();
    let name_c = CString::new(name).map_err(|_| AdapterError::WriteFailed("NUL in name".into()))?;
    let unit_c = CString::new(unit).map_err(|_| AdapterError::WriteFailed("NUL in unit".into()))?;

    // SAFETY: connected handle; CStrings are valid NUL-terminated; the value
    // pointer is valid for one FLOAT64 element as declared by the definition.
    unsafe {
        let mut hr = (api.clear_data_definition)(handle, DEFINE_WRITE);
        if hr < 0 {
            return Err(AdapterError::WriteFailed(format!(
                "ClearDataDefinition failed (hr=0x{hr:08X})"
            )));
        }

        hr = (api.add_to_data_definition)(
            handle,
            DEFINE_WRITE,
            name_c.as_ptr(),
            unit_c.as_ptr(),
            ffi::SIMCONNECT_DATATYPE_FLOAT64,
            0.0,
            0,
        );
        if hr < 0 {
            return Err(AdapterError::WriteFailed(format!(
                "AddToDataDefinition({name}) failed (hr=0x{hr:08X})"
            )));
        }

        hr = (api.set_data_on_sim_object)(
            handle,
            DEFINE_WRITE,
            ffi::SIMCONNECT_OBJECT_ID_USER,
            0,
            1,
            std::mem::size_of::<f64>() as u32,
            &value as *const f64 as *const std::ffi::c_void,
        );
        if hr < 0 {
            return Err(AdapterError::WriteFailed(format!(
                "SetDataOnSimObject({name}={value}) failed (hr=0x{hr:08X})"
            )));
        }
    }
    Ok(())
}

/// Fire a named key event with a numeric parameter.
///
/// # Safety
/// Caller guarantees the client is connected.
pub(crate) unsafe fn fire_event(
    client: &SimClient,
    name: &str,
    param: f64,
) -> Result<(), AdapterError> {
    let (api, handle) = client.parts();
    let name_c = CString::new(name).map_err(|_| AdapterError::WriteFailed("NUL in name".into()))?;

    // SAFETY: connected handle; CString valid for the duration of the calls.
    unsafe {
        let hr = (api.map_client_event_to_sim_event)(handle, EVT_ACTION, name_c.as_ptr());
        if hr < 0 {
            return Err(AdapterError::WriteFailed(format!(
                "MapClientEventToSimEvent({name}) failed (hr=0x{hr:08X})"
            )));
        }

        let hr = (api.transmit_client_event)(
            handle,
            ffi::SIMCONNECT_OBJECT_ID_USER,
            EVT_ACTION,
            param as u32,
            ffi::SIMCONNECT_GROUP_PRIORITY_HIGHEST,
            ffi::SIMCONNECT_EVENT_FLAG_GROUPID_IS_PRIORITY,
        );
        if hr < 0 {
            return Err(AdapterError::WriteFailed(format!(
                "TransmitClientEvent({name}) failed (hr=0x{hr:08X})"
            )));
        }
    }
    Ok(())
}
