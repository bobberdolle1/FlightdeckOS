//! A32NX action catalog: pre- and post-conditions for the closed actions
//! this package uses.
//!
//! Moved here from fd-runtime in Task 2: post-condition knowledge is
//! aircraft-specific, and fd-core/fd-runtime must stay aircraft-neutral.
//! Verification reads only canonical state (beacon light + the opaque
//! extension ids owned by this crate's state-field registry).

use fd_core::actions::{
    ActionCatalog, ActionKind, CatalogEntry, CockpitAction, NavLogoMode, PreconditionDef,
    SwitchPosition,
};
use fd_core::telemetry::TelemetrySnapshot;

use crate::state_field::StateField;

fn pre_beacon_state_known(s: &TelemetrySnapshot) -> Result<(), &'static str> {
    if s.beacon_light.is_some() {
        Ok(())
    } else {
        Err("beacon light state is unknown; refusing blind write")
    }
}

fn pre_nav_logo_state_known(s: &TelemetrySnapshot) -> Result<(), &'static str> {
    if s.aircraft_values
        .contains_key(&StateField::NavLogoSwitch.ext_id().unwrap())
    {
        Ok(())
    } else {
        Err("NAV/LOGO switch state is unknown; refusing blind write")
    }
}

/// Post-condition verifier shared by all SetBeacon variants.
fn verify_beacon(action: CockpitAction, s: &TelemetrySnapshot) -> Option<bool> {
    let pos = match action {
        CockpitAction::SetBeacon(pos) => pos,
        _ => return None,
    };
    s.beacon_light
        .map(|on| on == matches!(pos, SwitchPosition::On))
}

/// Post-condition verifier for SetNavLogo variants.
fn verify_nav_logo(action: CockpitAction, s: &TelemetrySnapshot) -> Option<bool> {
    let mode = match action {
        CockpitAction::SetNavLogo(mode) => mode,
        _ => return None,
    };
    s.aircraft_values
        .get(&StateField::NavLogoSwitch.ext_id().unwrap())
        .and_then(|v| NavLogoMode::from_raw(*v))
        .map(|observed| observed == mode)
}

/// The A32NX Task 2 action catalog.
pub fn a32nx_default_catalog() -> ActionCatalog {
    ActionCatalog {
        entries: vec![
            CatalogEntry {
                kind: ActionKind::SetBeacon,
                preconditions: vec![PreconditionDef {
                    id: "beacon_state_known",
                    check: pre_beacon_state_known,
                }],
                verify: verify_beacon,
            },
            CatalogEntry {
                kind: ActionKind::SetNavLogo,
                preconditions: vec![PreconditionDef {
                    id: "nav_logo_state_known",
                    check: pre_nav_logo_state_known,
                }],
                verify: verify_nav_logo,
            },
        ],
    }
}
