//! Capability model: per-capability status instead of binary
//! supported/unsupported aircraft support.
//!
//! Design rules (Task 3 §7–9):
//! * `Unsupported` = known not to exist / not applicable.
//! * `Unavailable` = exists but currently unusable (e.g. adapter offline).
//! * `Unknown`     = NOT YET DISCOVERED — absence of evidence is never
//!   treated as unsupported.
//! * `Supported`   = implemented and wired, but not live-proven.
//! * `Verified`    = proven against a real simulator.
//! * `Partial`     = some sub-capabilities present.
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
    /// Some sub-capabilities present.
    Partial,
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
            Self::Partial => "partial",
            Self::Unavailable => "unavailable",
            Self::Unsupported => "unsupported",
            Self::Unknown => "unknown",
        }
    }
}

/// A capability report: dotted capability paths mapped to statuses.
///
/// Paths use stable dotted names, e.g.:
/// `telemetry.position`, `systems.electrical`, `action.lights`,
/// `procedure.before_start`, `autonomy.flight`, `fdr.recording`.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct CapabilityReport {
    entries: Vec<(String, CapabilityStatus)>,
}

impl CapabilityReport {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn set(&mut self, path: impl Into<String>, status: CapabilityStatus) -> &mut Self {
        let path = path.into();
        match self.entries.iter_mut().find(|(p, _)| *p == path) {
            Some(entry) => entry.1 = status,
            None => self.entries.push((path, status)),
        }
        self
    }

    /// Current status; missing entries are `Unknown` (not yet discovered).
    pub fn status(&self, path: &str) -> CapabilityStatus {
        self.entries
            .iter()
            .find(|(p, _)| p == path)
            .map(|(_, s)| *s)
            .unwrap_or(CapabilityStatus::Unknown)
    }

    /// Sorted snapshot for deterministic serialization/reporting.
    pub fn entries_sorted(&self) -> Vec<(String, CapabilityStatus)> {
        let mut v = self.entries.clone();
        v.sort_by(|a, b| a.0.cmp(&b.0));
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
    }

    #[test]
    fn set_overwrites_and_entries_sort_deterministically() {
        let mut r = CapabilityReport::new();
        r.set("b.x", CapabilityStatus::Supported);
        r.set("a.y", CapabilityStatus::Verified);
        r.set("b.x", CapabilityStatus::Partial);
        let e = r.entries_sorted();
        assert_eq!(e[0].0, "a.y");
        assert_eq!(e[1].1, CapabilityStatus::Partial);
    }

    #[test]
    fn tier_is_presentation_only() {
        let mut r = CapabilityReport::new();
        r.set("telemetry.position", CapabilityStatus::Verified);
        assert_eq!(support_tier(&r, false), SupportTier::Generic);
        r.set("procedure.any", CapabilityStatus::Supported);
        assert_eq!(support_tier(&r, true), SupportTier::Profiled);
    }
}
