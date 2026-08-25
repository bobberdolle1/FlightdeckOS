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

/// Bumped on incompatible debrief schema changes. V2 (Task 7): bounded
/// summary inputs, FMS/plan section, transport gap statistics.
pub const DEBRIEF_FORMAT_VERSION: u32 = 2;

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
    /// Largest sim-time gap between consecutive samples (Task 7 §24).
    pub max_gap_ms: u64,
    /// Gaps exceeding the summarizer threshold.
    pub gaps_over_threshold: u64,
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

/// FMS / flight-plan summary of the observed session (Task 7 §48).
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct FmsPlanSummary {
    /// True when the FMS bridge delivered at least one snapshot.
    pub observed: bool,
    /// Device kind string ("StockGps"/"StockFms"/...), when observed.
    pub device: Option<String>,
    pub primary_entries: Option<usize>,
    pub destination_id: Option<String>,
    pub approach_loaded: Option<bool>,
    /// Number of distinct plan revisions observed (change events + 1).
    pub revisions_observed: u64,
    /// Classified changes (insert/remove/active-leg/approach/...).
    pub changes: Vec<String>,
    /// Best correlated procedure, when deterministically supported.
    pub procedure: Option<fd_core::fplan::ProcedureContext>,
    /// Read-only navigation phase, when deterministically supported.
    pub navigation_phase: Option<String>,
    /// Correlation quality counters (Task 7 §14).
    pub nav_matches: u64,
    pub nav_ambiguous: u64,
    pub nav_not_found: u64,
    /// OpenAIRAC dataset provenance used for correlation.
    pub openairac_provenance: Option<String>,
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
    /// FMS / flight-plan observation summary (None = no FMS source).
    pub plan: Option<FmsPlanSummary>,
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
            plan: None,
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
        for (&ch, _) in map.iter() {
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
        max_gap_ms: 0,
        gaps_over_threshold: 0,
    }
}

// -- Debrief assembly (Task 6 §54) -------------------------------------------

/// Inputs to the structured debrief builder.
pub struct BuildDebriefArgs<'a> {
    pub identity: AircraftIdentity,
    pub session: &'a fd_fdm::session::SessionTracker,
    pub sample_count: u64,
    pub origin: Option<&'a str>,
    pub destination: Option<&'a str>,
    pub route_source_str: Option<String>,
    pub waypoint_count: usize,
    pub route_usable: bool,
    pub off_route_events: u64,
    pub route_complete: bool,
    /// Bounded session summary (Task 7 §45): phase spans, quality counts,
    /// gap stats. Replaces whole-flight sample retention.
    pub summary: &'a fd_fdm::summary::SessionSummary,
    /// Bounded recent-sample window for landing analysis (final approach
    /// + touchdown always fit).
    pub landing_window: &'a [fd_fdm::fdr::FdrSample],
    /// FMS / flight-plan observation summary, when an FMS source existed.
    pub plan: Option<FmsPlanSummary>,
    pub fdm_events: u64,
    pub approach: &'a fd_fdm::qoa::ApproachReport,
    /// Runway context when OpenAIRAC geometry resolved one; landing
    /// runway-relative metrics stay None without it (never fabricated).
    pub runway: Option<&'a fd_mission::runway::RunwayContext>,
    pub shadow_summary: Option<fd_mission::shadow::ShadowSummary>,
}

/// Bridge: fd-fdm's RunwayGeometry consumed from an fd-mission RunwayContext
/// without coupling the two crates (dependency direction preserved).
struct RunwayBridge<'a>(&'a fd_mission::runway::RunwayContext);

impl fd_fdm::qol::RunwayGeometry for RunwayBridge<'_> {
    fn centerline_offset_m(&self, lat: f64, lon: f64) -> Option<f64> {
        self.0.centerline_offset_m(lat, lon)
    }
    fn distance_to_threshold_m(&self, lat: f64, lon: f64) -> Option<f64> {
        self.0.distance_to_threshold_m(lat, lon)
    }
    fn remaining_runway_m(&self, lat: f64, lon: f64) -> Option<f64> {
        self.0.remaining_runway_m(lat, lon)
    }
}

/// Assemble the structured flight debrief from analyzer outputs
/// (Task 6 §54). Deterministic: same inputs -> same document.
pub fn build_debrief(a: BuildDebriefArgs<'_>) -> Result<FlightDebrief, serde_json::Error> {
    let recording = fd_fdm::fdr::FlightRecording {
        meta: None,
        samples: a.landing_window.to_vec(),
        events: vec![],
    };
    let landing = match a.runway {
        Some(rw) => fd_fdm::qol::analyze_with_runway(&recording, &RunwayBridge(rw)),
        None => fd_fdm::qol::analyze(a.landing_window),
    };
    let mut debrief = FlightDebrief::new(a.identity);
    debrief.session = serde_json::json!({
        "state": format!("{:?}", a.session.state()),
        "samples": a.sample_count,
        "ever_airborne": a.session.ever_airborne(),
        "adapter_source": "xplane-udp",
        "origin": a.origin,
        "destination": a.destination,
    });
    debrief.route = RouteSummary {
        source: a.route_source_str,
        waypoint_count: a.route_usable.then_some(a.waypoint_count),
        off_route_events: a.off_route_events,
        completed: a.route_usable.then_some(a.route_complete),
    };
    debrief.phase_timeline = a
        .summary
        .phase_spans
        .iter()
        .map(|span| PhaseSpan {
            phase: span.phase.clone(),
            entered_ms: span.first_ms,
            exited_ms: Some(span.last_ms),
            samples: span.samples,
        })
        .collect();
    debrief.fdm_summary = serde_json::json!({
        "events": a.fdm_events,
    });
    debrief.approach = serde_json::to_value(a.approach)?;
    debrief.landing = serde_json::to_value(&landing)?;
    debrief.shadow = serde_json::json!(match a.shadow_summary {
        Some(summary) => serde_json::to_value(summary)?,
        None => serde_json::Value::Null,
    });
    debrief.plan = a.plan.clone();
    debrief.data_quality = DataQualitySummary {
        samples_total: a.summary.sample_count,
        non_fresh_by_channel: a
            .summary
            .channel_quality
            .iter()
            .map(|(ch, counts)| (*ch, counts.annotated.values().sum()))
            .collect(),
        never_fresh_channels: Vec::new(),
        max_gap_ms: a.summary.gaps.max_gap_ms,
        gaps_over_threshold: a.summary.gaps.gaps_over_threshold,
    };
    Ok(debrief)
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
