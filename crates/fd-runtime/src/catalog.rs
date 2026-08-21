//! Default Task 1 action catalog for the A32NX reference aircraft.
//!
//! Aircraft knowledge (preconditions, postconditions) lives here, not in
//! `fd-core`: Phase 2 moves this into the aircraft package. The raw
//! simulator binding names remain in `fd-simconnect` — this module only
//! reasons over canonical state.

use fd_core::actions::{
    ActionCatalog, ActionKind, CatalogEntry, CockpitAction, PreconditionDef, SwitchPosition,
};
use fd_core::telemetry::TelemetrySnapshot;

/// Canonical post-condition of an action: does the snapshot confirm it?
///
/// `Some(true)` = confirmed; `Some(false)` = contradicted; `None` = state
/// currently unknown (cannot confirm yet — fail closed: verification must
/// eventually time out, never succeed on unknown).
pub fn postcondition(action: CockpitAction, snapshot: &TelemetrySnapshot) -> Option<bool> {
    match action {
        CockpitAction::SetBeacon(pos) => snapshot
            .beacon_light
            .map(|on| on == matches!(pos, SwitchPosition::On)),
        CockpitAction::SetNavLogo(mode) => snapshot.a32nx.nav_logo.map(|m| m == mode),
    }
}

/// Precondition: current beacon state must be known before we attempt a
/// write (fail-closed: never write blind).
fn pre_beacon_state_known(s: &TelemetrySnapshot) -> Result<(), &'static str> {
    if s.beacon_light.is_some() {
        Ok(())
    } else {
        Err("beacon light state is unknown; refusing blind write")
    }
}

fn pre_nav_logo_state_known(s: &TelemetrySnapshot) -> Result<(), &'static str> {
    if s.a32nx.nav_logo.is_some() {
        Ok(())
    } else {
        Err("NAV/LOGO switch state is unknown; refusing blind write")
    }
}

/// Build the default Task 1 catalog.
pub fn a32nx_default_catalog() -> ActionCatalog {
    ActionCatalog {
        entries: vec![
            CatalogEntry {
                kind: ActionKind::SetBeacon,
                preconditions: vec![PreconditionDef {
                    id: "beacon_state_known",
                    check: pre_beacon_state_known,
                }],
            },
            CatalogEntry {
                kind: ActionKind::SetNavLogo,
                preconditions: vec![PreconditionDef {
                    id: "nav_logo_state_known",
                    check: pre_nav_logo_state_known,
                }],
            },
        ],
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn snap() -> TelemetrySnapshot {
        let mut s = TelemetrySnapshot::empty(fd_core::telemetry::SimTimestamp::new(0));
        s.beacon_light = Some(false);
        s.a32nx.nav_logo = Some(fd_core::telemetry::NavLogoMode::Off);
        s
    }

    #[test]
    fn postcondition_confirms_only_observed_state() {
        let s = snap();
        assert_eq!(
            postcondition(CockpitAction::SetBeacon(SwitchPosition::On), &s),
            Some(false)
        );
        assert_eq!(
            postcondition(CockpitAction::SetBeacon(SwitchPosition::Off), &s),
            Some(true)
        );
        let mut s2 = snap();
        s2.beacon_light = None;
        assert_eq!(
            postcondition(CockpitAction::SetBeacon(SwitchPosition::On), &s2),
            None
        );
    }

    #[test]
    fn preconditions_reject_unknown_state() {
        let unknown = TelemetrySnapshot::empty(fd_core::telemetry::SimTimestamp::new(0));
        for entry in a32nx_default_catalog().entries {
            for p in entry.preconditions {
                assert!(
                    (p.check)(&unknown).is_err(),
                    "precondition {} passed on empty state",
                    p.id
                );
            }
        }
        let known = snap();
        for entry in a32nx_default_catalog().entries {
            for p in entry.preconditions {
                assert!(
                    (p.check)(&known).is_ok(),
                    "precondition {} failed on known state",
                    p.id
                );
            }
        }
    }
}
