//! A32NX binding table: logical FlightdeckOS actions → raw simulator writes,
//! with provenance for every non-obvious binding.
//!
//! Task 1 §21 requires: logical id, raw binding, read/write capability,
//! unit/type, source/provenance, tested addon version — per binding.
//!
//! Provenance sources:
//! * MSFS SDK documentation (SimVars, settable flags).
//! * FlyByWire A32NX repository, `fbw-a32nx/docs/a320-simvars.md` (master).
//! * FlyByWire A32NX model behaviors,
//!   `ModelBehaviorDefs/A32NX/Interior/Overhead/A32NX_Lights.xml`.
//!
//! NOT live-verified (no MSFS on the development machine, Task 1 §25):
//! every entry here is a documented binding that must be proven in the live
//! spike before it graduates to `Supported`-in-production.

use fd_core::actions::NavLogoMode;
use fd_core::actions::{CockpitAction, SwitchPosition};

/// Raw write primitives supported by the adapter.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum WritePrimitive {
    /// Write a named settable variable via `SetDataOnSimObject`.
    SimVarWrite {
        name: &'static str,
        unit: &'static str,
        value: f64,
    },
    /// Fire a named key event via `TransmitClientEvent` (param = value).
    Event { name: &'static str, param: f64 },
}

/// A documented binding entry.
#[derive(Debug, Clone, Copy)]
pub struct BindingEntry {
    pub logical: &'static str,
    pub primitive: WritePrimitive,
    /// Read capability: the canonical state field this binding is verified
    /// against (post-condition), when applicable.
    pub verifies: &'static str,
    pub provenance: &'static str,
    pub tested_addon_version: &'static str,
}

pub const TESTED_ADDON_VERSION: &str = "A32NX master @ 2026-08-21 (NOT live-verified)";

/// Stable numeric extension-value ids written into the canonical
/// `aircraft_values` map by this adapter. The aircraft layer (fd-aircraft)
/// owns their meaning; these constants are the single trusted mapping point
/// between raw A32NX L:Vars and opaque canonical ids.
pub const EXT_ID_APU_N: u16 = 1;
pub const EXT_ID_APU_BLEED_VALVE_OPEN: u16 = 2;
pub const EXT_ID_FLAPS_HANDLE_INDEX: u16 = 3;
pub const EXT_ID_NAV_LOGO: u16 = 4;
pub const EXT_ID_PACK1_PB_ON: u16 = 5;

const MSFS_SDK_SIMVARS: &str =
    "MSFS SDK SimVars documentation (LIGHT BEACON, settable BOOL; BEACON_LIGHTS_SET key event)";
const FBW_A32NX_SIMVARS_DOC: &str =
    "https://github.com/flybywiresim/aircraft/blob/master/fbw-a32nx/docs/a320-simvars.md";
#[allow(dead_code)]
const FBW_A32NX_LIGHTS_BEHAVIOR: &str = "https://github.com/flybywiresim/aircraft/blob/master/fbw-a32nx/src/base/flybywire-aircraft-a320-neo/ModelBehaviorDefs/A32NX/Interior/Overhead/A32NX_Lights.xml";

/// Resolve a closed cockpit action to its raw write binding.
///
/// `None` = not in the binding table (fail-closed: the adapter reports
/// `Unsupported` and never guesses).
pub fn lookup_write(action: CockpitAction) -> Option<BindingEntry> {
    match action {
        // Beacon: A32NX has NO custom behavior for the beacon switch (verified
        // in the FBW repo — the only A32NX light behavior file is the NAV/LOGO
        // switch). The beacon is the standard Asobo light switch: settable
        // simvar `LIGHT BEACON`. Primary path: data-definition write.
        // Alternative documented path: key event `BEACON_LIGHTS_SET` (param
        // 0/1). Live spike must confirm which one the A32NX honors.
        CockpitAction::SetBeacon(SwitchPosition::On) => Some(BindingEntry {
            logical: "set_beacon_on",
            primitive: WritePrimitive::SimVarWrite {
                name: "LIGHT BEACON",
                unit: "bool",
                value: 1.0,
            },
            verifies: "beacon_light",
            provenance: MSFS_SDK_SIMVARS,
            tested_addon_version: TESTED_ADDON_VERSION,
        }),
        CockpitAction::SetBeacon(SwitchPosition::Off) => Some(BindingEntry {
            logical: "set_beacon_off",
            primitive: WritePrimitive::SimVarWrite {
                name: "LIGHT BEACON",
                unit: "bool",
                value: 0.0,
            },
            verifies: "beacon_light",
            provenance: MSFS_SDK_SIMVARS,
            tested_addon_version: TESTED_ADDON_VERSION,
        }),
        // NAV/LOGO: documented FBW enum L:Var `A32NX_LIGHTS_NAV_LOGO`
        // (0=Off, 1=Sys1, 2=Sys2). The model behavior's own SET_CODE writes
        // this exact L:Var, so it is the switch's input state — an external
        // L:Var write via SetDataOnSimObject is the documented write path.
        CockpitAction::SetNavLogo(mode) => Some(BindingEntry {
            logical: match mode {
                NavLogoMode::Off => "set_nav_logo_off",
                NavLogoMode::Sys1 => "set_nav_logo_sys1",
                NavLogoMode::Sys2 => "set_nav_logo_sys2",
            },
            primitive: WritePrimitive::SimVarWrite {
                name: "L:A32NX_LIGHTS_NAV_LOGO",
                unit: "number",
                value: mode.raw(),
            },
            verifies: "a32nx.nav_logo",
            provenance: FBW_A32NX_SIMVARS_DOC,
            tested_addon_version: TESTED_ADDON_VERSION,
        }),
    }
}

/// All catalog actions, for CLI/docs enumeration.
pub const ALL_ACTIONS: &[CockpitAction] = &[
    CockpitAction::SetBeacon(SwitchPosition::On),
    CockpitAction::SetBeacon(SwitchPosition::Off),
    CockpitAction::SetNavLogo(NavLogoMode::Off),
    CockpitAction::SetNavLogo(NavLogoMode::Sys1),
    CockpitAction::SetNavLogo(NavLogoMode::Sys2),
];

/// The five documented A32NX read bindings used in Task 1 (telemetry side).
/// Names come verbatim from `a320-simvars.md`; the mapping to canonical
/// state happens in `defs.rs` / `mapping.rs`.
#[derive(Debug, Clone, Copy)]
pub struct ReadBinding {
    pub canonical: &'static str,
    pub raw: &'static str,
    pub unit: &'static str,
    pub doc: &'static str,
}

pub const A32NX_READ_BINDINGS: &[ReadBinding] = &[
    ReadBinding {
        canonical: "a32nx.apu_n_percent",
        raw: "L:A32NX_APU_N",
        unit: "percent",
        doc: "APU RPM in percent of max (a320-simvars.md, 'A32NX_APU_N')",
    },
    ReadBinding {
        canonical: "a32nx.apu_bleed_valve_open",
        raw: "L:A32NX_APU_BLEED_AIR_VALVE_OPEN",
        unit: "bool",
        doc: "APU bleed air valve open (a320-simvars.md, 'A32NX_APU_BLEED_AIR_VALVE_OPEN')",
    },
    ReadBinding {
        canonical: "a32nx.flaps_handle_index",
        raw: "L:A32NX_FLAPS_HANDLE_INDEX",
        unit: "number",
        doc: "Physical flaps handle position 0..4 (a320-simvars.md, 'A32NX_FLAPS_HANDLE_INDEX')",
    },
    ReadBinding {
        canonical: "a32nx.nav_logo",
        raw: "L:A32NX_LIGHTS_NAV_LOGO",
        unit: "number",
        doc: "NAV/LOGO switch enum 0..2 (a320-simvars.md, 'L:A32NX_LIGHTS_NAV_LOGO')",
    },
    ReadBinding {
        canonical: "a32nx.pack_1_pb_on",
        raw: "L:A32NX_OVHD_COND_PACK_1_PB_IS_ON",
        unit: "bool",
        doc: "PACK 1 pushbutton on (a320-simvars.md, 'A32NX_OVHD_COND_PACK_{index}_PB_IS_ON')",
    },
];

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_catalog_action_resolves_to_a_binding() {
        for action in [
            CockpitAction::SetBeacon(SwitchPosition::On),
            CockpitAction::SetBeacon(SwitchPosition::Off),
            CockpitAction::SetNavLogo(NavLogoMode::Off),
            CockpitAction::SetNavLogo(NavLogoMode::Sys1),
            CockpitAction::SetNavLogo(NavLogoMode::Sys2),
        ] {
            assert!(lookup_write(action).is_some(), "no binding for {action:?}");
        }
    }

    #[test]
    fn nav_logo_values_match_documented_enum() {
        let e = lookup_write(CockpitAction::SetNavLogo(NavLogoMode::Sys2)).unwrap();
        assert_eq!(
            e.primitive,
            WritePrimitive::SimVarWrite {
                name: "L:A32NX_LIGHTS_NAV_LOGO",
                unit: "number",
                value: 2.0
            }
        );
    }

    #[test]
    fn all_five_read_bindings_are_documented() {
        assert_eq!(A32NX_READ_BINDINGS.len(), 5);
        for b in A32NX_READ_BINDINGS {
            assert!(
                b.raw.starts_with("L:A32NX_"),
                "raw name must be an A32NX L:Var: {}",
                b.raw
            );
            assert!(!b.doc.is_empty());
        }
    }

    #[test]
    fn beacon_uses_standard_simvar_not_a_custom_lvar() {
        let e = lookup_write(CockpitAction::SetBeacon(SwitchPosition::On)).unwrap();
        assert_eq!(
            e.primitive,
            WritePrimitive::SimVarWrite {
                name: "LIGHT BEACON",
                unit: "bool",
                value: 1.0
            }
        );
    }
}
