//! Package matching: step 1 of the unknown-addon story (the IL-76 problem).
//!
//! When a simulator reports an [`AircraftIdentity`] we have never seen, we
//! first try to recognize it against the known package catalog before falling
//! back to generic mode. Matching is intentionally narrow:
//!
//! * An identity whose source is [`IdentitySource::Unknown`] matches NOTHING —
//!   unknown never silently becomes known.
//! * Only the ICAO type designator (`icao`) and the addon author (`author`)
//!   produce a usable match. A family-level or single-field coincidence is
//!   never enough to rank a package as a candidate.
//! * Results are deterministically ordered: best tier first, ties broken by
//!   manifest id. Identical inputs always yield an identical ranking.

use crate::manifest::PackageManifest;
use fd_core::identity::{AircraftIdentity, IdentitySource};

/// How strongly a known package corresponds to an observed identity.
///
/// Tier order is meaningful: `Exact > TypeMatch > NoMatch`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum MatchConfidence {
    /// ICAO **and** author both matched exactly. Strongest signal.
    Exact,
    /// ICAO matched, but the author did not (or is unknown on one side).
    /// Same type, different publisher — still just a candidate.
    TypeMatch,
    /// At most a family-level correspondence. Never returned as a ranked
    /// match; kept explicit because "looks like the family" is exactly the
    /// kind of guess FlightdeckOS refuses to promote silently.
    NoMatch,
}

/// One ranked candidate package.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PackageMatch {
    /// `package_id` of the matched manifest.
    pub manifest_id: String,
    /// Strength of the correspondence.
    pub confidence: MatchConfidence,
    /// Which identity fields produced the match (e.g. `["icao", "author"]`).
    pub matched_fields: Vec<&'static str>,
}

/// Matches an aircraft identity against known package manifests.
#[derive(Debug, Clone, Copy, Default)]
pub struct PackageMatcher;

/// Normalized equality: trimmed, case-insensitive ASCII, and NEVER true when
/// either side is empty (an empty field is unknown, not a wildcard).
fn eq_field(a: &str, b: &str) -> bool {
    let (a, b) = (a.trim(), b.trim());
    !a.is_empty() && !b.is_empty() && a.eq_ignore_ascii_case(b)
}

impl PackageMatcher {
    /// Rank known packages against `identity`.
    ///
    /// Returns candidates ordered best-first (tier, then manifest id).
    /// Manifests sharing no signal with the identity are omitted entirely;
    /// an empty result means "run generic mode", never "guess".
    ///
    /// Guarantees:
    /// * `identity.source == IdentitySource::Unknown` → empty list, always.
    /// * Without an ICAO on the identity side, nothing can rank above
    ///   family-level, so the list stays empty too.
    /// * Same inputs → same output, byte-for-byte ordering included.
    pub fn matches(
        identity: &AircraftIdentity,
        manifests: &[PackageManifest],
    ) -> Vec<PackageMatch> {
        // Unknown provenance: we do not know what flew here. Matching would
        // turn silence into a claim, so there is deliberately nothing to do.
        if identity.source == IdentitySource::Unknown {
            return Vec::new();
        }
        // Tiers require an ICAO; without one every manifest lands at
        // NoMatch, which is excluded below. Skip the scan honestly.
        let Some(icao) = identity
            .icao
            .as_deref()
            .map(str::trim)
            .filter(|s| !s.is_empty())
        else {
            return Vec::new();
        };

        let mut matches = Vec::new();
        for manifest in manifests {
            let icao_match = eq_field(icao, &manifest.icao);
            if !icao_match {
                continue;
            }
            let author_match = identity
                .author
                .as_deref()
                .is_some_and(|a| eq_field(a, &manifest.author));
            let (confidence, matched_fields) = if author_match {
                (MatchConfidence::Exact, vec!["icao", "author"])
            } else {
                (MatchConfidence::TypeMatch, vec!["icao"])
            };
            matches.push(PackageMatch {
                manifest_id: manifest.package_id.clone(),
                confidence,
                matched_fields,
            });
        }

        // Deterministic ranking: tier first (Exact < TypeMatch by derive),
        // then manifest id. Stable sort keeps equal keys in input order.
        matches.sort_by(|a, b| {
            a.confidence
                .cmp(&b.confidence)
                .then_with(|| a.manifest_id.cmp(&b.manifest_id))
        });
        matches
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::manifest::{RUNTIME_API_VERSION, SCHEMA_VERSION};

    fn identity(icao: Option<&str>, author: Option<&str>) -> AircraftIdentity {
        AircraftIdentity {
            icao: icao.map(str::to_owned),
            author: author.map(str::to_owned),
            ..AircraftIdentity::user_provided(None)
        }
    }

    fn manifest(id: &str, icao: &str, author: &str) -> PackageManifest {
        PackageManifest {
            package_id: id.to_owned(),
            display_name: id.to_owned(),
            aircraft_family: "Ilyushin Il-76 family".to_owned(),
            simulator: "X-Plane".to_owned(),
            addon: id.to_owned(),
            package_version: "0.1.0".to_owned(),
            schema_version: SCHEMA_VERSION,
            runtime_api_version: RUNTIME_API_VERSION,
            addon_source_rev: String::new(),
            live_verified: false,
            notes: String::new(),
            icao: icao.to_owned(),
            author: author.to_owned(),
        }
    }

    #[test]
    fn exact_and_type_tiers_ranked_deterministically() {
        let packages = [
            manifest("beta-il76", "IL76", "OtherTeam"),
            manifest("alpha-il76", "il76", "Felis"),
            manifest("unrelated", "A320", "FlyByWire"),
        ];
        let identity = identity(Some("IL76"), Some("felis"));
        let matches = PackageMatcher::matches(&identity, &packages);
        assert_eq!(matches.len(), 2);
        assert_eq!(matches[0].manifest_id, "alpha-il76");
        assert_eq!(matches[0].confidence, MatchConfidence::Exact);
        assert_eq!(matches[0].matched_fields, vec!["icao", "author"]);
        assert_eq!(matches[1].manifest_id, "beta-il76");
        assert_eq!(matches[1].confidence, MatchConfidence::TypeMatch);
        assert_eq!(matches[1].matched_fields, vec!["icao"]);

        // Deterministic across runs and input orderings.
        let mut shuffled = [
            packages[2].clone(),
            packages[1].clone(),
            packages[0].clone(),
        ];
        shuffled.sort_by(|a, b| a.package_id.cmp(&b.package_id));
        assert_eq!(PackageMatcher::matches(&identity, &shuffled), matches);
    }

    #[test]
    fn unknown_identity_never_matches_even_with_fields_set() {
        let mut identity = AircraftIdentity::unknown();
        identity.icao = Some("IL76".into());
        identity.author = Some("Felis".into());
        let packages = [manifest("a32nx", "IL76", "Felis")];
        assert!(PackageMatcher::matches(&identity, &packages).is_empty());
    }

    #[test]
    fn identity_without_icao_matches_nothing() {
        let packages = [manifest("felis-il76", "IL76", "Felis")];
        let claimed_only_author = AircraftIdentity {
            author: Some("Felis".into()),
            ..AircraftIdentity::user_provided(None)
        };
        assert!(PackageMatcher::matches(&claimed_only_author, &packages).is_empty());

        let adapter_read = AircraftIdentity {
            source: IdentitySource::Adapter,
            ..AircraftIdentity::default()
        };
        assert!(PackageMatcher::matches(&adapter_read, &packages).is_empty());
    }

    #[test]
    fn blank_manifest_identity_fields_are_not_wildcards() {
        let packages = [
            manifest("blank-icao", "", "Felis"),
            manifest("blank-author", "IL76", ""),
        ];
        let identity = identity(Some("IL76"), Some("Felis"));
        let matches = PackageMatcher::matches(&identity, &packages);
        // Only the blank-author entry qualifies, and only via ICAO.
        assert_eq!(matches.len(), 1);
        assert_eq!(matches[0].manifest_id, "blank-author");
        assert_eq!(matches[0].confidence, MatchConfidence::TypeMatch);
    }

    #[test]
    fn matching_normalizes_case_and_whitespace() {
        let packages = [manifest("felis-il76", " il76 ", "FELIS")];
        let identity = identity(Some("IL76"), Some(" felis "));
        let matches = PackageMatcher::matches(&identity, &packages);
        assert_eq!(matches.len(), 1);
        assert_eq!(matches[0].confidence, MatchConfidence::Exact);
    }
}
