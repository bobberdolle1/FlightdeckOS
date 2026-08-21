//! Quality of Approach: MEASUREMENTS over the approach segment.
//!
//! Deliberately not a single score (Task 3 §17). Metrics with unknown input
//! are reported as `None` — never zero, never fabricated.
//!
//! "Stabilized" here means a DEVELOPMENT definition: sink rate within the
//! development limit and gear down (when gear state is known). It is not an
//! airline stabilized-approach gate definition.

use crate::fdr::FdrSample;
use serde::{Deserialize, Serialize};

/// Development stabilization criteria.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct StabilizationCriteria {
    /// |VS| at or below this counts as a stable sample (fpm).
    pub max_stable_sink_fpm: f64,
    /// AGL gate heights where stabilization is assessed (ft).
    pub gates_ft: [f64; 2], // [1000, 500]
}

impl Default for StabilizationCriteria {
    fn default() -> Self {
        // DEVELOPMENT DEFAULTS — not airline policy.
        Self {
            max_stable_sink_fpm: 1000.0,
            gates_ft: [1000.0, 500.0],
        }
    }
}

/// One stabilization-gate result. `None` = gate never crossed / unknown data.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct GateResult {
    pub stabilized: Option<bool>,
}

/// Approach measurements.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct ApproachReport {
    /// Gate results ordered [1000 ft, 500 ft]; `None` entries = not assessable.
    pub gates: Vec<Option<GateResult>>,
    pub max_ias_deviation_kt: Option<f64>,
    pub rms_ias_deviation_kt: Option<f64>,
    pub max_sink_rate_fpm: Option<f64>,
    pub max_bank_deg: Option<f64>,
    pub gear_configured_at_500ft: Option<bool>,
    pub approach_duration_s: Option<f64>,
    pub go_around_detected: Option<bool>,
}

/// Analysis window for one approach: samples between entering the approach
/// phase and touchdown/end.
#[derive(Debug, Default)]
pub struct ApproachAnalyzer {
    criteria: StabilizationCriteria,
    /// Reference speed used for IAS deviation when known (kt); None = unknown.
    reference_ias_kt: Option<f64>,
    samples: Vec<FdrSample>,
    started_at_ms: Option<u64>,
    finished: bool,
}

impl ApproachAnalyzer {
    pub fn new(criteria: StabilizationCriteria, reference_ias_kt: Option<f64>) -> Self {
        Self {
            criteria,
            reference_ias_kt,
            samples: Vec::new(),
            started_at_ms: None,
            finished: false,
        }
    }

    /// Begin collecting at the first approach sample.
    pub fn begin(&mut self, first_sample_ts_ms: u64) {
        if self.started_at_ms.is_none() {
            self.started_at_ms = Some(first_sample_ts_ms);
        }
    }

    pub fn push(&mut self, s: FdrSample) {
        self.begin(s.timestamp.ms);
        self.samples.push(s);
    }

    /// Finish collection and compute all supported metrics.
    pub fn finish(mut self) -> ApproachReport {
        self.finished = true;
        let mut report = ApproachReport::default();

        // Gates: find first crossing of each gate height (descending through).
        for &gate in &self.criteria.gates_ft {
            report.gates.push(self.gate_result(gate));
        }

        // IAS deviation vs reference (only when a reference is known).
        if let Some(reference) = self.reference_ias_kt {
            let mut max_dev: f64 = 0.0;
            let mut sum_sq = 0.0;
            let mut n = 0usize;
            for s in &self.samples {
                if let Some(ias) = s.indicated_airspeed.filter(|v| v.is_finite()) {
                    let dev = (ias - reference).abs();
                    max_dev = max_dev.max(dev);
                    sum_sq += dev * dev;
                    n += 1;
                }
            }
            if n > 0 {
                report.max_ias_deviation_kt = Some(max_dev);
                report.rms_ias_deviation_kt = Some((sum_sq / n as f64).sqrt());
            }
        }

        // Max sink rate (|VS| max while airborne).
        let mut max_sink: Option<f64> = None;
        let mut max_bank: Option<f64> = None;
        for s in &self.samples {
            if !matches!(s.on_ground, Some(true)) {
                if let Some(vs) = s.vertical_speed.filter(|v| v.is_finite()) {
                    let sink = -vs;
                    max_sink = Some(max_sink.unwrap_or(0.0).max(sink));
                }
                if let Some(bank) = s.bank.filter(|v| v.is_finite()) {
                    max_bank = Some(max_bank.unwrap_or(0.0).max(bank.abs()));
                }
            }
        }
        report.max_sink_rate_fpm = max_sink;
        report.max_bank_deg = max_bank;

        // Gear configured near the 500 ft gate (first sample at/below 500).
        report.gear_configured_at_500ft = self
            .samples
            .iter()
            .find(|s| s.radio_altitude.map(|a| a <= 500.0).unwrap_or(false))
            .and_then(|s| s.gear_down);

        // Duration.
        if let (Some(t0), Some(t1)) = (
            self.started_at_ms,
            self.samples.last().map(|s| s.timestamp.ms),
        ) {
            report.approach_duration_s = Some((t1.saturating_sub(t0)) as f64 / 1000.0);
        }

        // Go-around detection: airborne again after having been low+slow
        // near the ground (development heuristic: on_ground false AFTER
        // radio altitude < 100 ft within the window).
        let mut went_low = false;
        let mut go_around = false;
        for s in &self.samples {
            if let Some(agl) = s.radio_altitude {
                if agl < 100.0 && !matches!(s.on_ground, Some(true)) {
                    went_low = true;
                }
                if went_low && agl > 200.0 && !matches!(s.on_ground, Some(true)) {
                    go_around = true;
                    break;
                }
            }
        }
        report.go_around_detected = Some(go_around);
        report
    }

    fn gate_result(&self, gate_ft: f64) -> Option<GateResult> {
        // First descending crossing of the gate.
        let mut prev_agl: Option<f64> = None;
        for s in &self.samples {
            let agl = s.radio_altitude?;
            if let Some(prev) = prev_agl
                && prev > gate_ft
                && agl <= gate_ft
            {
                // Assess stability at this sample.
                let vs_ok = s
                    .vertical_speed
                    .filter(|v| v.is_finite())
                    .map(|vs| -vs <= self.criteria.max_stable_sink_fpm);
                let gear_ok = s.gear_down; // None stays unknown
                return Some(GateResult {
                    stabilized: match (vs_ok, gear_ok) {
                        (Some(v), Some(g)) => Some(v && g),
                        (Some(v), None) => Some(v),
                        (None, _) => Some(false),
                    },
                });
            }
            prev_agl = Some(agl);
        }
        None // gate never crossed inside the window -> not assessable
    }
}
