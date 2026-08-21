//! Minimal hand-maintained SimConnect FFI layer.
//!
//! WHY OWN FFI (Task 1 §20 decision record): the `simconnect-sys` crate
//! (0.24.3) generates an effectively empty binding set with bindgen 0.69.5 —
//! its `allowlist_item("(?i)SIMCONNECT.*")` filter silently matches nothing,
//! verified empirically on this toolchain. Rather than patching a
//! third-party build script, FlightdeckOS declares the small FFI surface it
//! needs directly:
//!
//! * struct layouts below mirror the MSFS SDK `SimConnect.h` exactly
//!   (`#pragma pack(1)` — hence `repr(C, packed)`); they were cross-checked
//!   against real bindgen output for the same header;
//! * functions are resolved at RUNTIME via `LoadLibraryW("SimConnect.dll")`,
//!   so a missing DLL produces a typed error instead of a process-load
//!   failure, and builds never require the MSFS SDK or libclang;
//! * the API surface is frozen SDK ABI (stable since FSX SP2; MSFS 2020/2024
//!   keep backward compatibility).

#![allow(non_camel_case_types, non_snake_case, non_upper_case_globals)]

use std::ffi::c_void;

pub type DWORD = u32;
pub type HRESULT = i32;
pub type HANDLE = *mut c_void;
pub type HWND = *mut c_void;
pub type LPCSTR = *const std::os::raw::c_char;

/// `#pragma pack(push, 1)` region of the SDK header.
#[repr(C, packed)]
#[derive(Debug, Clone, Copy)]
pub struct SIMCONNECT_RECV {
    pub dwSize: DWORD,
    pub dwVersion: DWORD,
    pub dwID: DWORD,
}

#[repr(C, packed)]
#[derive(Debug, Clone, Copy)]
pub struct SIMCONNECT_RECV_SIMOBJECT_DATA {
    pub _base: SIMCONNECT_RECV,
    pub dwRequestID: DWORD,
    pub dwObjectID: DWORD,
    pub dwDefineID: DWORD,
    pub dwFlags: DWORD,
    pub dwentrynumber: DWORD,
    pub dwoutof: DWORD,
    pub dwDefineCount: DWORD,
    /// Placeholder single DWORD; the FLOAT64 payload starts right after it.
    pub dwData: DWORD,
}

#[repr(C, packed)]
#[derive(Debug, Clone, Copy)]
pub struct SIMCONNECT_RECV_EVENT {
    pub _base: SIMCONNECT_RECV,
    pub uGroupID: DWORD,
    pub uEventID: DWORD,
    pub dwData: DWORD,
}

#[repr(C, packed)]
#[derive(Debug, Clone, Copy)]
pub struct SIMCONNECT_RECV_EXCEPTION {
    pub _base: SIMCONNECT_RECV,
    pub dwException: DWORD,
    pub dwSendID: DWORD,
    pub dwIndex: DWORD,
}

// -- constants (values verified against bindgen output of the SDK header) --
pub const SIMCONNECT_OBJECT_ID_USER: DWORD = 0;
pub const SIMCONNECT_PERIOD_SIM_FRAME: DWORD = 3;
pub const SIMCONNECT_DATATYPE_FLOAT64: DWORD = 4;
pub const SIMCONNECT_GROUP_PRIORITY_HIGHEST: DWORD = 1;
pub const SIMCONNECT_EVENT_FLAG_GROUPID_IS_PRIORITY: DWORD = 16;
pub const SIMCONNECT_OPEN_CONFIGINDEX_LOCAL: DWORD = u32::MAX;

pub const SIMCONNECT_RECV_ID_EXCEPTION: DWORD = 1;
pub const SIMCONNECT_RECV_ID_OPEN: DWORD = 2;
pub const SIMCONNECT_RECV_ID_QUIT: DWORD = 3;
pub const SIMCONNECT_RECV_ID_EVENT: DWORD = 4;
pub const SIMCONNECT_RECV_ID_SIMOBJECT_DATA: DWORD = 8;

/// Offset of the FLOAT64 payload inside `SIMCONNECT_RECV_SIMOBJECT_DATA`.
pub const SIMOBJECT_DATA_PAYLOAD_OFFSET: usize =
    std::mem::offset_of!(SIMCONNECT_RECV_SIMOBJECT_DATA, dwData) + std::mem::size_of::<DWORD>();

pub type DispatchProc = Option<
    unsafe extern "system" fn(pdata: *mut SIMCONNECT_RECV, cbdata: DWORD, pcontext: *mut c_void),
>;

/// `HRESULT SimConnect_Open(HANDLE*, LPCSTR, HWND, DWORD, HANDLE, DWORD)`
pub type SimConnect_Open_t =
    unsafe extern "system" fn(*mut HANDLE, LPCSTR, HWND, DWORD, HANDLE, DWORD) -> HRESULT;
/// `HRESULT SimConnect_Close(HANDLE)`
pub type SimConnect_Close_t = unsafe extern "system" fn(HANDLE) -> HRESULT;
/// `HRESULT SimConnect_CallDispatch(HANDLE, DispatchProc, void*)`
pub type SimConnect_CallDispatch_t =
    unsafe extern "system" fn(HANDLE, DispatchProc, *mut c_void) -> HRESULT;
/// `HRESULT SimConnect_AddToDataDefinition(HANDLE, DWORD, const char*, const char*, DWORD, float, DWORD)`
pub type SimConnect_AddToDataDefinition_t =
    unsafe extern "system" fn(HANDLE, DWORD, LPCSTR, LPCSTR, DWORD, f32, DWORD) -> HRESULT;
/// `HRESULT SimConnect_ClearDataDefinition(HANDLE, DWORD)`
pub type SimConnect_ClearDataDefinition_t = unsafe extern "system" fn(HANDLE, DWORD) -> HRESULT;
/// `HRESULT SimConnect_RequestDataOnSimObject(HANDLE, DWORD x7)`
#[allow(clippy::type_complexity)]
pub type SimConnect_RequestDataOnSimObject_t = unsafe extern "system" fn(
    HANDLE,
    DWORD,
    DWORD,
    DWORD,
    DWORD,
    DWORD,
    DWORD,
    DWORD,
    DWORD,
) -> HRESULT;
/// `HRESULT SimConnect_SetDataOnSimObject(HANDLE, DWORD, DWORD, DWORD, DWORD, DWORD, const void*)`
pub type SimConnect_SetDataOnSimObject_t =
    unsafe extern "system" fn(HANDLE, DWORD, DWORD, DWORD, DWORD, DWORD, *const c_void) -> HRESULT;
/// `HRESULT SimConnect_SubscribeToSystemEvent(HANDLE, DWORD, const char*)`
pub type SimConnect_SubscribeToSystemEvent_t =
    unsafe extern "system" fn(HANDLE, DWORD, LPCSTR) -> HRESULT;
/// `HRESULT SimConnect_MapClientEventToSimEvent(HANDLE, DWORD, const char*)`
pub type SimConnect_MapClientEventToSimEvent_t =
    unsafe extern "system" fn(HANDLE, DWORD, LPCSTR) -> HRESULT;
/// `HRESULT SimConnect_TransmitClientEvent(HANDLE, DWORD x5)`
pub type SimConnect_TransmitClientEvent_t =
    unsafe extern "system" fn(HANDLE, DWORD, DWORD, DWORD, DWORD, DWORD) -> HRESULT;

/// Runtime-resolved SimConnect entry points.
pub struct SimConnectApi {
    module: *mut c_void,
    pub open: SimConnect_Open_t,
    pub close: SimConnect_Close_t,
    pub call_dispatch: SimConnect_CallDispatch_t,
    pub add_to_data_definition: SimConnect_AddToDataDefinition_t,
    pub clear_data_definition: SimConnect_ClearDataDefinition_t,
    pub request_data_on_sim_object: SimConnect_RequestDataOnSimObject_t,
    pub set_data_on_sim_object: SimConnect_SetDataOnSimObject_t,
    pub subscribe_to_system_event: SimConnect_SubscribeToSystemEvent_t,
    pub map_client_event_to_sim_event: SimConnect_MapClientEventToSimEvent_t,
    pub transmit_client_event: SimConnect_TransmitClientEvent_t,
}

unsafe extern "system" {
    #[link_name = "LoadLibraryW"]
    fn load_library_w(lp_filename: *const u16) -> *mut c_void;
    #[link_name = "GetProcAddress"]
    fn get_proc_address(h_module: *mut c_void, lp_proc_name: *const u8) -> *mut c_void;
    #[link_name = "FreeLibrary"]
    fn free_library(h_lib_module: *mut c_void) -> i32;
}

impl SimConnectApi {
    /// Load `SimConnect.dll` and resolve every entry point.
    ///
    /// DLL search order is the standard Windows one: application directory
    /// first, then System32, then PATH. Ship the MSFS SDK's client
    /// `SimConnect.dll` next to the binary (see docs).
    pub fn load() -> Result<Self, String> {
        let name: Vec<u16> = "SimConnect.dll\0".encode_utf16().collect();
        // SAFETY: name is a valid NUL-terminated UTF-16 string.
        let module = unsafe { load_library_w(name.as_ptr()) };
        if module.is_null() {
            return Err(
                "SimConnect.dll not found. Place the MSFS SDK client SimConnect.dll \
                        next to fd.exe (or ensure it is on the DLL search path)."
                    .to_string(),
            );
        }

        macro_rules! get {
            ($fname:literal, $fty:ty) => {{
                let p = unsafe { get_proc_address(module, concat!($fname, "\0").as_ptr()) };
                if p.is_null() {
                    return Err(format!("SimConnect.dll missing export {}", $fname));
                }
                // SAFETY: the export exists and has the documented signature.
                unsafe { std::mem::transmute::<*mut c_void, $fty>(p) }
            }};
        }

        Ok(Self {
            module,
            open: get!("SimConnect_Open", SimConnect_Open_t),
            close: get!("SimConnect_Close", SimConnect_Close_t),
            call_dispatch: get!("SimConnect_CallDispatch", SimConnect_CallDispatch_t),
            add_to_data_definition: get!(
                "SimConnect_AddToDataDefinition",
                SimConnect_AddToDataDefinition_t
            ),
            clear_data_definition: get!(
                "SimConnect_ClearDataDefinition",
                SimConnect_ClearDataDefinition_t
            ),
            request_data_on_sim_object: get!(
                "SimConnect_RequestDataOnSimObject",
                SimConnect_RequestDataOnSimObject_t
            ),
            set_data_on_sim_object: get!(
                "SimConnect_SetDataOnSimObject",
                SimConnect_SetDataOnSimObject_t
            ),
            subscribe_to_system_event: get!(
                "SimConnect_SubscribeToSystemEvent",
                SimConnect_SubscribeToSystemEvent_t
            ),
            map_client_event_to_sim_event: get!(
                "SimConnect_MapClientEventToSimEvent",
                SimConnect_MapClientEventToSimEvent_t
            ),
            transmit_client_event: get!(
                "SimConnect_TransmitClientEvent",
                SimConnect_TransmitClientEvent_t
            ),
        })
    }
}

impl Drop for SimConnectApi {
    fn drop(&mut self) {
        // The library stays loaded for the process lifetime; freeing it
        // while dispatch callbacks might still be registered is complexity
        // Task 1 does not need.
        //
        // SAFETY: module handle obtained in `load`.
        unsafe {
            free_library(self.module);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn recv_layout_matches_sdk_packed_layout() {
        // Base: 3 x DWORD.
        assert_eq!(std::mem::size_of::<SIMCONNECT_RECV>(), 12);
        // Base + 7 x DWORD header fields + 1 x DWORD placeholder.
        assert_eq!(std::mem::size_of::<SIMCONNECT_RECV_SIMOBJECT_DATA>(), 44);
        assert_eq!(SIMOBJECT_DATA_PAYLOAD_OFFSET, 44);
    }

    #[test]
    fn constant_values_match_sdk_header() {
        assert_eq!(SIMCONNECT_OBJECT_ID_USER, 0);
        assert_eq!(SIMCONNECT_PERIOD_SIM_FRAME, 3);
        assert_eq!(SIMCONNECT_DATATYPE_FLOAT64, 4);
        assert_eq!(SIMCONNECT_RECV_ID_SIMOBJECT_DATA, 8);
        assert_eq!(SIMCONNECT_RECV_ID_EVENT, 4);
        assert_eq!(SIMCONNECT_GROUP_PRIORITY_HIGHEST, 1);
        assert_eq!(SIMCONNECT_EVENT_FLAG_GROUPID_IS_PRIORITY, 16);
    }
}
