//! Aircraft / SOP Package Identity and Compatibility Binding.
//!
//! Enforces strict aircraft-to-SOP package isolation so an aircraft (e.g. TU154)
//! never silently receives an incompatible aircraft's SOP (e.g. A32NX).

use fd_sop::package::ValidatedPackage;
use serde::{Deserialize, Serialize};

/// Binding status of an SOP package to the active aircraft.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "status", rename_all = "SCREAMING_SNAKE_CASE")]
pub enum SopBindingStatus {
    Active {
        package_id: String,
        aircraft_family: String,
        current_flow: String,
        completed_steps_count: usize,
        pending_steps_count: usize,
    },
    UnavailableForAircraft {
        aircraft: String,
        reason: String,
    },
    NotInstalled,
}

pub struct SopAircraftBinding;

impl SopAircraftBinding {
    /// Evaluate whether the active aircraft is compatible with a loaded SOP package.
    pub fn evaluate(aircraft_type: &str, package: Option<&ValidatedPackage>) -> SopBindingStatus {
        let clean_type = aircraft_type.trim().to_uppercase();

        let Some(pkg) = package else {
            return SopBindingStatus::NotInstalled;
        };

        // Strict compatibility matching against package manifest
        let is_compatible = match clean_type.as_str() {
            "A320" | "A32NX" | "A321" | "A319" => {
                pkg.manifest.package_id == "a32nx"
                    || pkg.manifest.aircraft_family.to_lowercase().contains("a320")
            }
            _ => false,
        };

        if is_compatible {
            let flow_id = pkg
                .flows
                .first()
                .map(|f| f.id.clone())
                .unwrap_or_else(|| "none".to_string());
            let steps_count = pkg.flows.first().map(|f| f.steps.len()).unwrap_or(0);

            SopBindingStatus::Active {
                package_id: pkg.manifest.package_id.clone(),
                aircraft_family: pkg.manifest.aircraft_family.clone(),
                current_flow: flow_id,
                completed_steps_count: 0,
                pending_steps_count: steps_count,
            }
        } else {
            SopBindingStatus::UnavailableForAircraft {
                aircraft: clean_type.clone(),
                reason: format!(
                    "No SOP package installed for aircraft '{}'; navigation and flight context remain fully operational",
                    clean_type
                ),
            }
        }
    }
}
