//! Known read-binding names a package may declare as provenance metadata.
//!
//! This list mirrors the trusted A32NX read bindings actually implemented by
//! the fd-simconnect adapter. Package `bindings.toml` entries must resolve
//! against it — unknown names fail package validation. The mapping from
//! these fields to raw L:Vars lives in fd-simconnect (trusted code); package
//! data can neither add nor alter it.

use crate::error::PackageError;

pub const KNOWN_READ_BINDINGS: &[&str] = &[
    "apu_n_percent",
    "apu_bleed_valve_open",
    "flaps_handle_index",
    "nav_logo_switch",
    "pack_1_pb_on",
];

/// Validate a package's declared binding-name list (fail-closed, duplicates
/// rejected).
pub fn validate_binding_names(names: &[String]) -> Result<(), PackageError> {
    let mut seen = Vec::new();
    for n in names {
        if !KNOWN_READ_BINDINGS.contains(&n.as_str()) {
            return Err(PackageError::UnknownBindingName(n.clone()));
        }
        if seen.contains(n) {
            return Err(PackageError::InvalidValue(format!(
                "duplicate binding declaration `{n}`"
            )));
        }
        seen.push(n.clone());
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn known_names_pass_and_unknown_fail() {
        assert!(validate_binding_names(&["apu_n_percent".into()]).is_ok());
        assert!(validate_binding_names(&["warp_drive".into()]).is_err());
    }

    #[test]
    fn duplicates_rejected() {
        assert!(validate_binding_names(&["apu_n_percent".into(), "apu_n_percent".into()]).is_err());
    }
}
