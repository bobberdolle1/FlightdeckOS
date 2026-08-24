//! Structured flight debrief for FlightdeckOS (Task 6 §54).
//!
//! Aggregates the deterministic outputs of one observed flight — identity,
//! session, route, phase timeline, FDM, approach, landing, Mission Shadow,
//! data quality — into one serializable document. No prose, no AI: every
//! field traces to a concrete analyzer output.
//!
//! Summary sections embed their source-crate report types where the dependency
//! direction allows (fd-fdm, fd-mission); sections whose producers are not
//! dependencies are carried as `serde_json::Value` so the debrief schema can
//! evolve without coupling crates that must stay decoupled.

use fd_core::identity::AircraftIdentity;
use serde::{Deserialize, Serialize};

/// Bumped on incompatible debrief schema changes.
pub const DEBRIEF_FORMAT_VERSION: u32 = 1;

/// One contiguous flight-phase span with entry evidence.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PhaseSpan {
    pub phase: String,
    pub entered_ms: u64,
    pub exited_ms: Option<u64>,
    /// Number of samples observed in this phase.
    pub samples: u64,
}

/// Data-quality summary across a recording.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct DataQualitySummary {
    pub samples_total: u64,
    /// Per-channel (wire id) count of samples NOT fresh, keyed by channel id.
    pub non_fresh_by_channel: std::collections::BTreeMap<u16, u64>,
    /// Channels that were never observed fresh at all.
    pub never_fresh_channels: Vec<u16>,
}

/// Route outcome of the observed flight.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct RouteSummary {
    /// Human-stable route source description (e.g. "openairac:world@2026-08-20T19:15:00Z").
    pub source: Option<String>,
    pub waypoint_count: Option<usize>,
    pub off_route_events: u64,
    /// True when the monitor observed route completion.
    pub completed: Option<bool>,
}

/// The full structured debrief document.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct FlightDebrief {
    pub format_version: u32,
    pub identity: AircraftIdentity,
    /// Aircraft package id when a package was active (None = generic mode).
    pub package: Option<String>,
    /// Session summary (lifecycle states traversed, sample count, wall bounds).
    pub session: serde_json::Value,
    pub route: RouteSummary,
    pub phase_timeline: Vec<PhaseSpan>,
    /// fd-fdm FdmAnalyzer summary (events, episodes).
    pub fdm_summary: serde_json::Value,
    /// fd-fdm ApproachReport.
    pub approach: serde_json::Value,
    /// fd-fdm landing analysis (TouchdownRecord + runway-relative metrics).
    pub landing: serde_json::Value,
    /// fd-mission ShadowSummary.
    pub shadow: serde_json::Value,
    pub data_quality: DataQualitySummary,
}

impl FlightDebrief {
    /// Start a debrief for an identified aircraft.
    pub fn new(identity: AircraftIdentity) -> Self {
        Self {
            format_version: DEBRIEF_FORMAT_VERSION,
            identity,
            package: None,
            session: serde_json::Value::Null,
            route: RouteSummary::default(),
            phase_timeline: Vec::new(),
            fdm_summary: serde_json::Value::Null,
            approach: serde_json::Value::Null,
            landing: serde_json::Value::Null,
            shadow: serde_json::Value::Null,
            data_quality: DataQualitySummary::default(),
        }
    }

    /// Serialize to pretty JSON.
    pub fn to_json_pretty(&self) -> Result<String, serde_json::Error> {
        serde_json::to_string_pretty(self)
    }

    /// Load a debrief document (fail-closed on version mismatch).
    pub fn from_json_str(s: &str) -> Result<Self, DebriefError> {
        let d: Self = serde_json::from_str(s).map_err(DebriefError::Serialization)?;
        if d.format_version != DEBRIEF_FORMAT_VERSION {
            return Err(DebriefError::VersionMismatch {
                got: d.format_version,
                expected: DEBRIEF_FORMAT_VERSION,
            });
        }
        Ok(d)
    }
}

#[derive(Debug, thiserror::Error)]
pub enum DebriefError {
    #[error("serialization: {0}")]
    Serialization(#[from] serde_json::Error),
    #[error("debrief version mismatch: got {got}, expected {expected}")]
    VersionMismatch { got: u32, expected: u32 },
}

/// Build a phase timeline from an ordered stream of (timestamp, phase) samples.
///
/// Deterministic: equal consecutive phases extend the current span; phase
/// change closes the previous span. Unknown/empty input yields an empty timeline.
pub fn phase_timeline_from_samples(samples: &[(u64, &str)]) -> Vec<PhaseSpan> {
    let mut spans: Vec<PhaseSpan> = Vec::new();
    for &(ts, phase) in samples {
        match spans.last_mut() {
            Some(span) if span.phase == phase => {
                span.samples += 1;
            }
            _ => {
                if let Some(prev) = spans.last_mut() {
                    prev.exited_ms = Some(ts);
                }
                spans.push(PhaseSpan {
                    phase: phase.to_string(),
                    entered_ms: ts,
                    exited_ms: None,
                    samples: 1,
                });
            }
        }
    }
    spans
}

/// Summarize per-sample channel quality maps into a DataQualitySummary.
pub fn data_quality_summary(
    sample_count: u64,
    per_sample_non_fresh: impl Iterator<
        Item = std::collections::BTreeMap<u16, fd_core::telemetry::DataQuality>,
    >,
) -> DataQualitySummary {
    use std::collections::BTreeMap;
    let mut non_fresh: BTreeMap<u16, u64> = BTreeMap::new();
    let mut ever_fresh: BTreeMap<u16, bool> = BTreeMap::new();
    for map in per_sample_non_fresh {
        for (&ch, _q) in &map {
            *non_fresh.entry(ch).or_insert(0) += 1;
            ever_fresh.entry(ch).or_insert(false);
        }
    }
    // Channels present in some map but never fresh are exactly those with
    // non_fresh count == sample_count (annotated every sample) — the exception
    // map only records NON-fresh channels, so presence in every sample means
    // never fresh.
    let never_fresh = non_fresh
        .iter()
        .filter(|&(_, &c)| c == sample_count)
        .map(|(&ch, _)| ch)
        .collect();
    DataQualitySummary {
        samples_total: sample_count,
        non_fresh_by_channel: non_fresh,
        never_fresh_channels: never_fresh,
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    use fd_core::identity::IdentitySource;
    use std::collections::BTreeMap;

    fn identity() -> AircraftIdentity {
        AircraftIdentity {
            icao: Some("C172".to_string()),
            tail_number: None,
            author: None,
            description: None,
            acf_name: None,
            source: IdentitySource::UserProvided,
        }
    }
    #[test]
    fn version_mismatch_fails_closed() {
        let identity = identity();
        let mut d = FlightDebrief::new(identity);
        d.format_version = 99;
        let s = d.to_json_pretty().unwrap();
        assert!(matches!(
            FlightDebrief::from_json_str(&s),
            Err(DebriefError::VersionMismatch { .. })
        ));
    }

    #[test]
    fn phase_timeline_groups_and_closes_spans() {
        let spans = phase_timeline_from_samples(&[
            (0, "Parked"),
            (10, "Parked"),
            (20, "Taxi"),
            (30, "Takeoff"),
        ]);
        assert_eq!(spans.len(), 3);
        assert_eq!(spans[0].samples, 2);
        assert_eq!(spans[0].exited_ms, Some(20));
        assert_eq!(spans[2].exited_ms, None);
    }

    #[test]
    fn empty_timeline() {
        assert!(phase_timeline_from_samples(&[]).is_empty());
    }

    #[test]
    fn data_quality_counts_never_fresh() {
        use fd_core::telemetry::DataQuality;
        let maps = vec![
            {
                let mut m = BTreeMap::new();
                m.insert(7u16, DataQuality::Stale);
                m
            },
            {
                let mut m = BTreeMap::new();
                m.insert(7u16, DataQuality::Stale);
                m
            },
            BTreeMap::new(), // ch 7 fresh this sample (absent from exception map)
        ];
        let s = data_quality_summary(3, maps.into_iter());
        assert_eq!(s.non_fresh_by_channel.get(&7), Some(&2));
        assert!(s.never_fresh_channels.is_empty());
    }
}
