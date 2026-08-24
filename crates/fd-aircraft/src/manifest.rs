//! Package identity and version compatibility.

use std::path::Path;

use serde::Deserialize;

use crate::error::PackageError;

/// Package schema version understood by THIS build. Packages declaring a
/// different major schema are rejected (fail-closed).
pub const SCHEMA_VERSION: u32 = 1;
/// Runtime API version this package was validated against.
pub const RUNTIME_API_VERSION: u32 = 1;

/// Package identity block (`manifest.toml`).
#[derive(Debug, Clone, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PackageManifest {
    pub package_id: String,
    pub display_name: String,
    /// Aircraft family (e.g. "Airbus A320 family") — broader than one addon.
    pub aircraft_family: String,
    /// Target simulator (e.g. "MSFS").
    pub simulator: String,
    /// Concrete addon (e.g. "FlyByWire A32NX").
    pub addon: String,
    /// Package's own semver-ish version.
    pub package_version: String,
    pub schema_version: u32,
    pub runtime_api_version: u32,
    /// Source revision the definitions were derived from (provenance).
    #[serde(default)]
    pub addon_source_rev: String,
    /// True ONLY after a live simulator validation of every write binding.
    /// The bundled A32NX package ships `false` until the MSFS spike runs.
    #[serde(default)]
    pub live_verified: bool,
    /// Free-form provenance / scope notes.
    #[serde(default)]
    pub notes: String,
    /// ICAO type designator this package targets (e.g. "IL76"). Used by
    /// package matching; empty means unknown and matches nothing.
    #[serde(default)]
    pub icao: String,
    /// Addon author/publisher. Used by package matching (Exact tier);
    /// empty means unknown and never matches.
    #[serde(default)]
    pub author: String,
}

fn non_empty(v: &str, field: &'static str) -> Result<(), PackageError> {
    if v.trim().is_empty() {
        Err(PackageError::EmptyField { field })
    } else {
        Ok(())
    }
}

impl PackageManifest {
    pub(crate) fn validate(&self) -> Result<(), PackageError> {
        for (v, f) in [
            (&self.package_id, "package_id"),
            (&self.display_name, "display_name"),
            (&self.aircraft_family, "aircraft_family"),
            (&self.simulator, "simulator"),
            (&self.addon, "addon"),
            (&self.package_version, "package_version"),
        ] {
            non_empty(v, f)?;
        }
        if self.schema_version != SCHEMA_VERSION {
            return Err(PackageError::SchemaVersion {
                found: self.schema_version,
                supported: SCHEMA_VERSION,
            });
        }
        if self.runtime_api_version != RUNTIME_API_VERSION {
            return Err(PackageError::RuntimeApiVersion {
                found: self.runtime_api_version,
                supported: RUNTIME_API_VERSION,
            });
        }
        Ok(())
    }
}

/// Load and validate `manifest.toml` from a package directory.
pub fn load_manifest(dir: &Path) -> Result<PackageManifest, PackageError> {
    let path = dir.join("manifest.toml");
    let text = std::fs::read_to_string(&path)
        .map_err(|e| PackageError::Io(format!("{}: {e}", path.display())))?;
    let manifest: PackageManifest = toml::from_str(&text).map_err(|e| PackageError::Toml {
        file: "manifest.toml",
        source_text: e.to_string(),
    })?;
    manifest.validate()?;
    Ok(manifest)
}

#[cfg(test)]
mod tests {
    use super::*;

    const VALID: &str = r#"
package_id = "a32nx"
display_name = "FlyByWire A32NX"
aircraft_family = "Airbus A320 family"
simulator = "MSFS"
addon = "FlyByWire A32NX"
package_version = "0.1.0"
schema_version = 1
runtime_api_version = 1
"#;

    #[test]
    fn valid_manifest_parses_and_validates() {
        let m: PackageManifest = toml::from_str(VALID).unwrap();
        m.validate().unwrap();
        assert_eq!(m.package_id, "a32nx");
    }

    #[test]
    fn unsupported_schema_version_rejected() {
        let bad = VALID.replace("schema_version = 1", "schema_version = 99");
        let m: PackageManifest = toml::from_str(&bad).unwrap();
        assert!(matches!(
            m.validate(),
            Err(PackageError::SchemaVersion { found: 99, .. })
        ));
    }

    #[test]
    fn empty_required_field_rejected() {
        let bad = VALID.replace("package_id = \"a32nx\"", "package_id = \" \"");
        let m: PackageManifest = toml::from_str(&bad).unwrap();
        assert!(matches!(
            m.validate(),
            Err(PackageError::EmptyField {
                field: "package_id"
            })
        ));
    }

    #[test]
    fn unknown_fields_rejected() {
        let bad = format!("{VALID}\nnot_a_field = 1\n");
        assert!(toml::from_str::<PackageManifest>(&bad).is_err());
    }
}
