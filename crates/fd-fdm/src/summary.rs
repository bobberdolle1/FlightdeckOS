//! Bounded session summary (Task 7 §45-46).
//!
//! The live observer streams samples to the FDR on disk and must NOT
//! retain the whole flight in RAM. [`SessionSummarizer`] consumes the
//! sample/event stream and keeps only bounded state:
//!
//! - phase spans (merged; capped with an explicit truncation counter);
//! - per-channel data-quality occurrence counts (bounded by channels ×
//!   quality variants);
//! - a fixed-size ring of the most recent samples, sized to always cover
//!   the FINAL landing for landing analysis (§29);
//! - inter-sample gap statistics (§24: gaps are reported, never hidden);
//! - FDM event count.
//!
//! Deterministic: the same input stream produces the same summary.

use std::collections::{BTreeMap, VecDeque};

use serde::{Deserialize, Serialize};

use crate::fdr::FdrSample;

/// Ring capacity for the landing-analysis window: at the observer's 4 Hz
/// this covers ~17 minutes — the final approach + landing always fit,
/// because the landing is the last thing that happens before session end.
pub const LANDING_WINDOW_SAMPLES: usize = 4096;

/// Phase-span cap. A pathological phase-oscillating session merges
/// overflow into an explicit truncation counter instead of growing.
pub const MAX_PHASE_SPANS: usize = 10_000;

/// Gaps longer than this (sim ms) are counted and reported (§24).
pub const GAP_REPORT_THRESHOLD_MS: u64 = 5000;

/// One contiguous phase span in the summary.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SummaryPhaseSpan {
    pub phase: String,
    pub first_seq: u64,
    pub last_seq: u64,
    pub first_ms: u64,
    pub last_ms: u64,
    pub samples: u64,
}

/// Per-channel quality occurrence counts. Absent annotation = fresh; the
/// fresh count is derivable as `sample_count - annotated_occurrences`.
#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct ChannelQualityCounts {
    /// quality debug name -> occurrences
    pub annotated: BTreeMap<String, u64>,
}

/// Transport health statistics (§24).
#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct GapStats {
    /// Largest sim-time gap between consecutive samples.
    pub max_gap_ms: u64,
    /// Gaps exceeding [`GAP_REPORT_THRESHOLD_MS`].
    pub gaps_over_threshold: u64,
}

/// The bounded summary of one session.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SessionSummary {
    pub sample_count: u64,
    pub first_seq: Option<u64>,
    pub last_seq: Option<u64>,
    pub first_ms: Option<u64>,
    pub last_ms: Option<u64>,
    pub phase_spans: Vec<SummaryPhaseSpan>,
    /// Samples that would open a new phase span beyond [`MAX_PHASE_SPANS`]
    /// collapsed into this counter.
    pub phase_spans_truncated: u64,
    /// channel id -> quality counts.
    pub channel_quality: BTreeMap<u16, ChannelQualityCounts>,
    pub gaps: GapStats,
    pub fdm_events: u64,
}

/// Full summary including the bounded sample ring (kept out of
/// [`SessionSummary`] so the serializable summary stays small).
#[derive(Debug)]
pub struct SessionSummarizer {
    summary: SessionSummary,
    landing_window: std::collections::VecDeque<FdrSample>,
    last_ms: Option<u64>,
}

impl Default for SessionSummarizer {
    fn default() -> Self {
        Self::new()
    }
}

impl SessionSummarizer {
    /// Samples consumed so far.
    pub fn sample_count(&self) -> u64 {
        self.summary.sample_count
    }

    /// FDM events recorded so far.
    pub fn fdm_events(&self) -> u64 {
        self.summary.fdm_events
    }

    pub fn new() -> Self {
        Self {
            summary: SessionSummary {
                sample_count: 0,
                first_seq: None,
                last_seq: None,
                first_ms: None,
                last_ms: None,
                phase_spans: Vec::new(),
                phase_spans_truncated: 0,
                channel_quality: BTreeMap::new(),
                gaps: GapStats::default(),
                fdm_events: 0,
            },
            landing_window: VecDeque::with_capacity(LANDING_WINDOW_SAMPLES),
            last_ms: None,
        }
    }

    /// Consume one sample (insertion order).
    pub fn push_sample(&mut self, s: &FdrSample) {
        let st = &mut self.summary;
        if st.first_seq.is_none() {
            st.first_seq = Some(s.seq);
            st.first_ms = Some(s.timestamp.ms);
        }
        st.last_seq = Some(s.seq);
        st.last_ms = Some(s.timestamp.ms);

        // Gap statistics (sim time; equal/lower stamps are not gaps).
        if let Some(prev) = self.last_ms
            && s.timestamp.ms > prev
        {
            let gap = s.timestamp.ms - prev;
            if gap > st.gaps.max_gap_ms {
                st.gaps.max_gap_ms = gap;
            }
            if gap > GAP_REPORT_THRESHOLD_MS {
                st.gaps.gaps_over_threshold += 1;
            }
        }
        self.last_ms = Some(s.timestamp.ms);

        // Phase spans: extend or close+open. At the cap, further samples
        // that would open a new span are collapsed into an explicit
        // counter — never merged under a wrong phase label (§46: bounded
        // AND honest).
        let at_cap = st.phase_spans.len() >= MAX_PHASE_SPANS;
        match st.phase_spans.last_mut() {
            Some(span) if span.phase == s.flight_phase => {
                span.last_seq = s.seq;
                span.last_ms = s.timestamp.ms;
                span.samples += 1;
            }
            _ if at_cap => {
                st.phase_spans_truncated += 1;
            }
            _ => {
                st.phase_spans.push(SummaryPhaseSpan {
                    phase: s.flight_phase.clone(),
                    first_seq: s.seq,
                    last_seq: s.seq,
                    first_ms: s.timestamp.ms,
                    last_ms: s.timestamp.ms,
                    samples: 1,
                });
            }
        }

        // Channel quality counts.
        for (ch, q) in &s.channel_quality {
            let entry = st.channel_quality.entry(*ch).or_default();
            *entry.annotated.entry(format!("{q:?}")).or_insert(0) += 1;
        }

        // Bounded landing window.
        if self.landing_window.len() == LANDING_WINDOW_SAMPLES {
            self.landing_window.pop_front();
        }
        self.landing_window.push_back(s.clone());
        st.sample_count += 1;
    }

    /// Consume one FDM event (count only; events are recorded elsewhere).
    pub fn record_fdm_event(&mut self) {
        self.summary.fdm_events += 1;
    }

    /// Finish: detach the serializable summary; the landing window moves
    /// out for landing analysis.
    pub fn finish(self) -> (SessionSummary, Vec<FdrSample>) {
        (self.summary, self.landing_window.into_iter().collect())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::fdr::FdrEvent;
    use fd_core::telemetry::SimTimestamp;

    fn sample(seq: u64, ms: u64, phase: &str) -> FdrSample {
        FdrSample {
            seq,
            timestamp: SimTimestamp { ms },
            altitude_msl: None,
            radio_altitude: None,
            indicated_airspeed: None,
            groundspeed: None,
            vertical_speed: None,
            heading_true: None,
            pitch: None,
            bank: None,
            on_ground: Some(true),
            gear_down: None,
            flaps_handle_index: None,
            any_engine_running: None,
            autopilot_master: None,
            flight_phase: phase.into(),
            sim_state: "running".into(),
            position: None,
            track_true_deg: None,
            sim_rate: None,
            slew: None,
            channel_quality: BTreeMap::new(),
        }
    }

    #[test]
    fn phase_spans_merge_and_cap() {
        let mut sum = SessionSummarizer::new();
        for i in 0..100u64 {
            sum.push_sample(&sample(
                i,
                i * 250,
                if i < 40 { "PREFLIGHT" } else { "CRUISE" },
            ));
        }
        let (s, _) = sum.finish();
        assert_eq!(s.sample_count, 100);
        assert_eq!(s.phase_spans.len(), 2);
        assert_eq!(s.phase_spans[0].samples, 40);
        assert_eq!(s.phase_spans[1].samples, 60);
        assert_eq!(s.phase_spans_truncated, 0);
    }

    #[test]
    fn phase_span_cap_never_grows_unbounded() {
        let mut sum = SessionSummarizer::new();
        // Alternate phases far beyond the cap. Same-phase samples keep
        // extending the last span (bounded, accurate); rejected span
        // OPENS are counted.
        for i in 0..(MAX_PHASE_SPANS as u64 + 500) {
            let phase = if i % 2 == 0 { "A" } else { "B" };
            sum.push_sample(&sample(i, i * 250, phase));
        }
        let (s, _) = sum.finish();
        assert_eq!(s.phase_spans.len(), MAX_PHASE_SPANS);
        // Of the 500 overflow samples, every other one matches the last
        // span's phase (extension, not a rejected open).
        assert_eq!(s.phase_spans_truncated, 250);
        assert_eq!(s.sample_count, MAX_PHASE_SPANS as u64 + 500);
    }

    #[test]
    fn landing_window_is_bounded_and_keeps_tail() {
        let mut sum = SessionSummarizer::new();
        let n = (LANDING_WINDOW_SAMPLES * 2) as u64;
        for i in 0..n {
            sum.push_sample(&sample(i, i * 250, "CRUISE"));
        }
        let (s, window) = sum.finish();
        assert_eq!(window.len(), LANDING_WINDOW_SAMPLES);
        assert_eq!(window[0].seq, n - LANDING_WINDOW_SAMPLES as u64);
        assert_eq!(window.last().unwrap().seq, n - 1);
        assert_eq!(s.sample_count, n);
    }

    #[test]
    fn gap_stats_report_largest_and_over_threshold() {
        let mut sum = SessionSummarizer::new();
        sum.push_sample(&sample(0, 0, "CRUISE"));
        sum.push_sample(&sample(1, 1000, "CRUISE"));
        sum.push_sample(&sample(2, 9000, "CRUISE")); // 8 s gap
        sum.push_sample(&sample(3, 9500, "CRUISE"));
        let (s, _) = sum.finish();
        assert_eq!(s.gaps.max_gap_ms, 8000);
        assert_eq!(s.gaps.gaps_over_threshold, 1);
    }

    #[test]
    fn quality_counts_are_bounded_per_channel() {
        let mut sum = SessionSummarizer::new();
        let mut s1 = sample(0, 0, "CRUISE");
        s1.channel_quality
            .insert(3, fd_core::telemetry::DataQuality::WarmingUp);
        let mut s2 = sample(1, 250, "CRUISE");
        s2.channel_quality
            .insert(3, fd_core::telemetry::DataQuality::Stale);
        sum.push_sample(&s1);
        sum.push_sample(&s2);
        let (s, _) = sum.finish();
        let counts = &s.channel_quality[&3];
        assert_eq!(counts.annotated["WarmingUp"], 1);
        assert_eq!(counts.annotated["Stale"], 1);
    }

    #[test]
    fn fdm_events_counted() {
        let mut sum = SessionSummarizer::new();
        sum.record_fdm_event();
        sum.record_fdm_event();
        let (s, _) = sum.finish();
        assert_eq!(s.fdm_events, 2);
    }

    #[test]
    fn empty_session_is_honest_zero() {
        let (s, w) = SessionSummarizer::new().finish();
        assert_eq!(s.sample_count, 0);
        assert!(s.phase_spans.is_empty());
        assert!(w.is_empty());
        assert_eq!(s.first_seq, None);
    }

    #[allow(dead_code)]
    fn event_type_anchor(_e: &FdrEvent) {}
}
