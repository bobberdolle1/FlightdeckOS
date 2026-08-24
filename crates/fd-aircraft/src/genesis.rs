//! Profile Genesis foundation: scaffold a **DRAFT** profile for an unknown
//! addon (the IL-76 problem, step 3).
//!
//! When no known package matches, Genesis turns whatever identity and
//! telemetry-field knowledge exists into a starting point for human or
//! automated verification. Two hard rules shape everything here:
//!
//! * **Never invent values.** Fields we cannot know stay explicitly empty;
//!   only identity-sourced values (ICAO, author) and build-time constants
//!   (`schema_version`, `runtime_api_version`) are filled in.
//! * **A draft is never trusted.** A generated profile always starts at
//!   [`VerificationState::Draft`], which cannot execute actions; and because
//!   every required manifest field is left empty on purpose, the emitted
//!   TOML is *rejected by package validation* until a verifier fills it in.
//!   Promotion through the verification ladder is mandatory before any
//!   action execution.

use fd_core::identity::{AircraftIdentity, IdentitySource};

use crate::manifest::{RUNTIME_API_VERSION, SCHEMA_VERSION};
use crate::verification::VerificationState;

/// A freshly scaffolded, unverified profile for an unknown addon.
#[derive(Debug, Clone, PartialEq)]
pub struct DraftProfile {
    /// The identity that triggered genesis (provenance preserved).
    pub identity: AircraftIdentity,
    /// Always [`VerificationState::Draft`] at creation; promoted only
    /// through the ladder in [`crate::verification`].
    pub verification: VerificationState,
    /// Capability/telemetry field names this addon exposes (as observed or
    /// reported). Sorted + deduplicated so identical inputs give identical
    /// profiles. These never enter the package manifest — packages may only
    /// reference the closed state-field registry.
    pub available_state_fields: Vec<String>,
    /// Deterministic provenance note. Deliberately NOT a wall-clock
    /// timestamp: identical inputs must produce byte-identical output.
    pub generated_at_note: String,
}

impl DraftProfile {
    /// Scaffold a draft from an identity plus the observed capability/field
    /// list. The result is fully determined by its inputs.
    pub fn from_identity(identity: AircraftIdentity, available_fields: Vec<String>) -> Self {
        let mut fields = available_fields;
        fields.sort();
        fields.dedup();
        let generated_at_note = format!(
            "Profile Genesis draft; identity source: {}; nothing verified",
            source_name(identity.source),
        );
        Self {
            identity,
            verification: VerificationState::Draft,
            available_state_fields: fields,
            generated_at_note,
        }
    }

    /// Explicit trust gate. A `DraftProfile` can only ever be loaded as a
    /// *trusted* package after promotion to [`VerificationState::Trusted`] —
    /// and structurally it cannot even load before then, see
    /// [`Self::to_manifest_toml`].
    pub fn can_be_loaded_as_trusted_package(&self) -> bool {
        self.verification == VerificationState::Trusted
    }

    /// Human-readable promotion requirement (kept next to the gate).
    pub fn promotion_requirement(&self) -> &'static str {
        "Fill required manifest fields, verify bindings live, and promote \
         verification Draft -> Observed -> Correlated -> Documented -> Tested \
         -> Trusted; drafts are rejected by package validation and can never \
         execute actions."
    }

    /// Deterministic draft `manifest.toml` skeleton.
    ///
    /// Stable key order mirrors `PackageManifest`'s declaration order. Values
    /// are either copied from the identity (`icao`, `author`), known
    /// constants (`schema_version`, `runtime_api_version`), or honest
    /// emptiness — never guesses. Because the six required string fields stay
    /// empty, feeding this output back into package loading fails validation:
    /// that rejection IS the draft gate at this layer.
    pub fn to_manifest_toml(&self) -> String {
        let empty_or = |opt: Option<&str>| opt.unwrap_or_default().to_owned();
        format!(
            concat!(
                "# FlightdeckOS DRAFT package manifest (Profile Genesis).\n",
                "# VERIFICATION STATUS: {status} - NOT trusted. This file is a\n",
                "# scaffold for human/automated review, not a loadable package:\n",
                "# required fields are intentionally left empty and will fail\n",
                "# package validation until verified.\n",
                "# {note}\n",
                "# Unknown fields stay empty; nothing is inferred.\n",
                "package_id = \"\"\n",
                "display_name = \"\"\n",
                "aircraft_family = \"\"\n",
                "simulator = \"\"\n",
                "addon = \"\"\n",
                "package_version = \"\"\n",
                "schema_version = {schema}\n",
                "runtime_api_version = {api}\n",
                "addon_source_rev = \"\"\n",
                "live_verified = false\n",
                "notes = \"{note}\"\n",
                "icao = \"{icao}\"\n",
                "author = \"{author}\"\n",
            ),
            status = format!("{:?}", self.verification).to_ascii_lowercase(),
            note = self.generated_at_note,
            schema = SCHEMA_VERSION,
            api = RUNTIME_API_VERSION,
            icao = empty_or(self.identity.icao.as_deref()),
            author = empty_or(self.identity.author.as_deref()),
        )
    }
}

/// snake_case provenance name matching the serde representation.
fn source_name(source: IdentitySource) -> &'static str {
    match source {
        IdentitySource::Unknown => "unknown",
        IdentitySource::UserProvided => "user_provided",
        IdentitySource::Adapter => "adapter",
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::manifest::PackageManifest;

    /// Guard: skeleton must parse as exactly the known manifest schema (no
    /// stray keys), while still failing value validation.
    fn parses_but_fails_validation(text: &str) {
        let parsed: PackageManifest =
            toml::from_str(text).expect("draft skeleton must use exact manifest keys");
        assert!(parsed.validate().is_err(), "draft must fail validation");
    }

    fn il76_identity() -> AircraftIdentity {
        AircraftIdentity {
            icao: Some("IL76".into()),
            author: None,
            ..AircraftIdentity::user_provided(None)
        }
    }

    #[test]
    fn same_input_byte_identical_output() {
        let fields = vec!["beacon".to_owned(), "nav_logo".to_owned()];
        let a = DraftProfile::from_identity(il76_identity(), fields.clone());
        let b = DraftProfile::from_identity(il76_identity(), fields);
        assert_eq!(a, b);
        assert_eq!(a.to_manifest_toml(), b.to_manifest_toml());
        // Field order in the input must not leak into the output.
        let shuffled =
            DraftProfile::from_identity(il76_identity(), vec!["nav_logo".into(), "beacon".into()]);
        assert_eq!(shuffled.to_manifest_toml(), a.to_manifest_toml());
    }

    #[test]
    fn creation_is_always_draft_and_cannot_execute() {
        let draft = DraftProfile::from_identity(il76_identity(), Vec::new());
        assert_eq!(draft.verification, VerificationState::Draft);
        assert!(!draft.can_be_loaded_as_trusted_package());
        assert!(!draft.verification.can_execute_actions());
    }

    #[test]
    fn draft_toml_rejected_by_real_package_validation() {
        parses_but_fails_validation(
            &DraftProfile::from_identity(il76_identity(), Vec::new()).to_manifest_toml(),
        );
    }

    #[test]
    fn no_fields_invented_only_identity_and_constants() {
        let toml_text = DraftProfile::from_identity(il76_identity(), Vec::new()).to_manifest_toml();
        for required in [
            "package_id",
            "display_name",
            "aircraft_family",
            "simulator",
            "addon",
            "package_version",
        ] {
            assert!(
                toml_text.contains(&format!("{required} = \"\"")),
                "{required} must stay empty, never invented"
            );
        }
        assert!(toml_text.contains("icao = \"IL76\""));
        assert!(
            toml_text.contains("author = \"\""),
            "unknown author stays empty"
        );
        assert!(toml_text.contains(&format!("schema_version = {SCHEMA_VERSION}")));
        assert!(toml_text.contains("live_verified = false"));
    }

    #[test]
    fn unknown_identity_produces_all_empty_skeleton() {
        let draft = DraftProfile::from_identity(AircraftIdentity::unknown(), Vec::new());
        let toml_text = draft.to_manifest_toml();
        assert!(toml_text.contains("icao = \"\""));
        assert!(toml_text.contains("identity source: unknown"));
        parses_but_fails_validation(&toml_text);
    }

    #[test]
    fn available_state_fields_sorted_deduplicated() {
        let draft = DraftProfile::from_identity(
            il76_identity(),
            vec![
                "zulu_field".to_owned(),
                "alpha_field".to_owned(),
                "alpha_field".to_owned(),
            ],
        );
        assert_eq!(draft.available_state_fields, ["alpha_field", "zulu_field"]);
    }
}
