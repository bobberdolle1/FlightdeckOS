//! Capability model: per-capability status instead of binary
//! supported/unsupported aircraft support.
//!
//! Design rules (Task 3 §7–9, Task 4 §1–2):
//! * `Unsupported` = known not to exist / not applicable.
//! * `Unavailable` = exists but currently unusable (e.g. adapter offline).
//! * `Unknown`     = NOT YET DISCOVERED — absence of evidence is never
//!   treated as unsupported.
//! * `Supported`   = implemented and wired, but not live-proven.
//! * `Degraded`    = was available, now reduced (e.g. live connection lost).
//! * `Verified`    = proven through a LIVE adapter/environment — NEVER by
//!   headless/virtual proof alone.
//!
//! Every entry carries its EVIDENCE SOURCE so reports can never confuse
//! virtual-simulator proof with live-simulator proof.
//!
//! A human-facing "support tier" may be DERIVED from a report for UX, but it
//! must never drive behavior: behavior reads individual capabilities.

use serde::{Deserialize, Serialize};

/// Status of one named capability.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CapabilityStatus {
    /// Proven against a real simulator/live environment.
    Verified,
    /// Implemented and wired; not live-proven.
    Supported,
    /// Was available; now reduced (e.g. SIMULATOR_DISCONNECTED).
    Degraded,
    /// Implemented but currently unusable (offline, missing addon).
    Unavailable,
    /// Known not to exist for this aircraft/adapter.
    Unsupported,
    /// Not yet discovered — no evidence either way.
    Unknown,
}

impl CapabilityStatus {
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::Verified => "verified",
            Self::Supported => "supported",
            Self::Degraded => "degraded",
            Self::Unavailable => "unavailable",
            Self::Unsupported => "unsupported",
            Self::Unknown => "unknown",
        }
    }
}

/// Where a capability status was established. Task 4 §2: a future report
/// must not confuse virtual proof with live proof.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum EvidenceSource {
    /// Established by code inspection / static wiring.
    Static,
    /// Proven against the deterministic headless VirtualSimulator.
    VirtualSim,
    /// Proven against a genuinely running X-Plane 12.
    LiveXplane,
    /// Proven against a genuinely running MSFS.
    LiveMsfs,
    /// No source could establish this capability.
    Unavailable,
}

impl EvidenceSource {
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::Static => "static",
            Self::VirtualSim => "virtual_sim",
            Self::LiveXplane => "live_xplane",
            Self::LiveMsfs => "live_msfs",
            Self::Unavailable => "unavailable",
        }
    }
}

/// One capability entry: path, status, and where the evidence came from.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct CapabilityEntry {
    pub path: String,
    pub status: CapabilityStatus,
    pub evidence: EvidenceSource,
}

/// A capability report: dotted capability paths mapped to statuses plus
/// evidence provenance.
///
/// Paths use stable dotted names, e.g.:
/// `telemetry.position`, `systems.electrical`, `action.lights`,
/// `procedure.before_start`, `autonomy.flight`, `fdr.recording`.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct CapabilityReport {
    entries: Vec<CapabilityEntry>,
}

impl CapabilityReport {
    pub fn new() -> Self {
        Self::default()
    }

    /// Set status with STATIC evidence (wiring-level knowledge).
    pub fn set(&mut self, path: impl Into<String>, status: CapabilityStatus) -> &mut Self {
        self.set_with_evidence(path, status, EvidenceSource::Static)
    }

    /// Set status with explicit evidence provenance.
    ///
    /// Fail-closed provenance rule: `Verified` claims MUST rest on live or
    /// simulator-backed evidence. `EvidenceSource::Static` means "wiring-level
    /// knowledge only" and can never prove a live-verified capability, so the
    /// combination is rejected as a programmer error instead of silently
    /// fabricating an escalation path.
    pub fn set_with_evidence(
        &mut self,
        path: impl Into<String>,
        status: CapabilityStatus,
        evidence: EvidenceSource,
    ) -> &mut Self {
        let path = path.into();
        assert!(
            status != CapabilityStatus::Verified
                || matches!(
                    evidence,
                    EvidenceSource::LiveXplane | EvidenceSource::LiveMsfs
                ),
            "Verified capability requires LIVE adapter evidence (LiveXplane/LiveMsfs), got {evidence:?}: {path}"
        );
        match self.entries.iter_mut().find(|e| e.path == path) {
            Some(entry) => {
                entry.status = status;
                entry.evidence = evidence;
            }
            None => self.entries.push(CapabilityEntry {
                path,
                status,
                evidence,
            }),
        }
        self
    }

    /// Current status; missing entries are `Unknown` (not yet discovered).
    pub fn status(&self, path: &str) -> CapabilityStatus {
        self.entries
            .iter()
            .find(|e| e.path == path)
            .map(|e| e.status)
            .unwrap_or(CapabilityStatus::Unknown)
    }

    /// Evidence for a path; missing entries are `Unavailable`.
    pub fn evidence(&self, path: &str) -> EvidenceSource {
        self.entries
            .iter()
            .find(|e| e.path == path)
            .map(|e| e.evidence)
            .unwrap_or(EvidenceSource::Unavailable)
    }

    /// Sorted snapshot for deterministic serialization/reporting.
    pub fn entries_sorted(&self) -> Vec<CapabilityEntry> {
        let mut v = self.entries.clone();
        v.sort_by(|a, b| a.path.cmp(&b.path));
        v
    }

    pub fn len(&self) -> usize {
        self.entries.len()
    }

    pub fn is_empty(&self) -> bool {
        self.entries.is_empty()
    }
}

/// UX-facing summary tier derived from a report.
///
/// NEVER used to gate behavior — individual capabilities decide that.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SupportTier {
    /// No package, nothing discovered yet.
    Unknown,
    /// Generic telemetry only (FDR/FDM/phase work).
    Generic,
    /// Some aircraft-specific capabilities discovered.
    Discovered,
    /// A validated package is loaded.
    Profiled,
}

/// Derive the summary tier from a report. Pure presentation helper.
pub fn support_tier(report: &CapabilityReport, has_package: bool) -> SupportTier {
    if has_package && report.status("procedure.any") != CapabilityStatus::Unknown {
        return SupportTier::Profiled;
    }
    let discovered = report.status("telemetry.position") != CapabilityStatus::Unknown
        || report.status("aircraft.values") != CapabilityStatus::Unknown;
    if discovered {
        SupportTier::Generic
    } else {
        SupportTier::Unknown
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn missing_capability_is_unknown_not_unsupported() {
        let r = CapabilityReport::new();
        assert_eq!(r.status("systems.pneumatic"), CapabilityStatus::Unknown);
        assert_eq!(r.evidence("systems.pneumatic"), EvidenceSource::Unavailable);
    }

    #[test]
    fn set_overwrites_and_entries_sort_deterministically() {
        let mut r = CapabilityReport::new();
        r.set("b.x", CapabilityStatus::Supported);
        r.set_with_evidence(
            "a.y",
            CapabilityStatus::Verified,
            EvidenceSource::LiveXplane,
        );
        r.set("b.x", CapabilityStatus::Degraded);
        let e = r.entries_sorted();
        assert_eq!(e[0].path, "a.y");
        assert_eq!(e[1].status, CapabilityStatus::Degraded);
    }

    #[test]
    fn tier_is_presentation_only() {
        let mut r = CapabilityReport::new();
        r.set_with_evidence(
            "telemetry.position",
            CapabilityStatus::Verified,
            EvidenceSource::LiveXplane,
        );
        assert_eq!(support_tier(&r, false), SupportTier::Generic);
        r.set("procedure.any", CapabilityStatus::Supported);
        assert_eq!(support_tier(&r, true), SupportTier::Profiled);
    }

    #[test]
    fn evidence_preserved_and_updated() {
        let mut r = CapabilityReport::new();
        r.set_with_evidence(
            "telemetry.position",
            CapabilityStatus::Supported,
            EvidenceSource::VirtualSim,
        );
        assert_eq!(r.evidence("telemetry.position"), EvidenceSource::VirtualSim);
        // Live promotion replaces both status and evidence together.
        r.set_with_evidence(
            "telemetry.position",
            CapabilityStatus::Verified,
            EvidenceSource::LiveXplane,
        );
        assert_eq!(r.status("telemetry.position"), CapabilityStatus::Verified);

        assert_eq!(r.evidence("telemetry.position"), EvidenceSource::LiveXplane);
    }

    #[test]
    #[should_panic(expected = "Verified capability requires LIVE adapter evidence")]
    fn verified_with_virtual_evidence_panics() {
        let mut r = CapabilityReport::new();
        r.set_with_evidence(
            "telemetry.position",
            CapabilityStatus::Verified,
            EvidenceSource::VirtualSim,
        );
    }
}
