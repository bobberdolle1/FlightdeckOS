//! Flight Data Recorder: deterministic, ordered stream of aircraft state
//! samples with attached events.
//!
//! Records ONLY what is present in the canonical snapshot; unknown fields
//! stay unknown (serialized as `null`). Ordering is strictly insertion
//! order — the recorder adds nothing and reorders nothing.

use fd_core::telemetry::{SimTimestamp, TelemetrySnapshot};
use serde::{Deserialize, Serialize};

/// One recorded FDR sample.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct FdrSample {
    /// Sample index within the recording (monotonic, 0-based).
    pub seq: u64,
    pub timestamp: SimTimestamp,
    // Generic core fields; `None` = unknown for this airframe/adapter.
    pub altitude_msl: Option<f64>,
    pub radio_altitude: Option<f64>,
    pub indicated_airspeed: Option<f64>,
    pub groundspeed: Option<f64>,
    pub vertical_speed: Option<f64>,
    pub heading_true: Option<f64>,
    pub pitch: Option<f64>,
    pub bank: Option<f64>,
    pub on_ground: Option<bool>,
    pub gear_down: Option<bool>,
    pub flaps_handle_index: Option<u8>,
    /// Any engine running (where engine state is available).
    pub any_engine_running: Option<bool>,
    pub autopilot_master: Option<bool>,
    /// Flight phase label from the runtime phase engine.
    pub flight_phase: String,
    /// Simulator timing state ("running"/"paused"/"unknown").
    pub sim_state: String,
}

/// A non-sample event attached to the recording (phase change, action).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct FdrEvent {
    pub seq: u64,
    pub timestamp: SimTimestamp,
    pub kind: String,
    pub detail: String,
}

/// The recording itself. Deterministic: same input sequence ⇒ same output.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct FlightRecording {
    pub samples: Vec<FdrSample>,
    pub events: Vec<FdrEvent>,
}

impl FlightRecording {
    pub fn push_sample(&mut self, sample: FdrSample) {
        self.samples.push(sample);
    }

    pub fn push_event(&mut self, event: FdrEvent) {
        self.events.push(event);
    }

    pub fn len(&self) -> usize {
        self.samples.len()
    }

    pub fn is_empty(&self) -> bool {
        self.samples.is_empty()
    }
}

/// Recorder that converts canonical snapshots into FDR samples.
///
/// Unknown fields remain unknown: every optional snapshot field is copied
/// as-is (`None` stays `None`).
#[derive(Debug, Default)]
pub struct Recorder {
    next_seq: u64,
}

impl Recorder {
    pub fn new() -> Self {
        Self::default()
    }

    /// Convert one canonical snapshot into an ordered FDR sample.
    pub fn record(&mut self, snap: &TelemetrySnapshot, flight_phase: &str) -> FdrSample {
        let seq = self.next_seq;
        self.next_seq += 1;
        FdrSample {
            seq,
            timestamp: snap.timestamp,
            altitude_msl: snap.altitude_msl.map(|v| v.value()),
            radio_altitude: snap.altitude_agl.map(|v| v.value()),
            indicated_airspeed: snap.indicated_airspeed.map(|v| v.value()),
            groundspeed: snap.groundspeed.map(|v| v.value()),
            vertical_speed: snap.vertical_speed.map(|v| v.value()),
            heading_true: snap.heading_true.map(|v| v.value()),
            pitch: snap.pitch.map(|v| v.value()),
            bank: snap.bank.map(|v| v.value()),
            on_ground: snap.on_ground,
            gear_down: snap.gear_handle_down,
            flaps_handle_index: snap.flaps_handle_index,
            any_engine_running: snap
                .engine_combustion
                .as_ref()
                .map(|e| e.iter().any(|e| e == &Some(true))),
            autopilot_master: snap.autopilot_master,
            flight_phase: flight_phase.to_string(),
            sim_state: match snap.sim_timing.state {
                fd_core::telemetry::SimState::Running => "running",
                fd_core::telemetry::SimState::Paused => "paused",
                fd_core::telemetry::SimState::Unknown => "unknown",
            }
            .to_string(),
        }
    }

    /// Current sample counter (next seq to be assigned).
    pub const fn samples_recorded(&self) -> u64 {
        self.next_seq
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use fd_core::telemetry::TelemetrySnapshot;
    use fd_core::units::{AltitudeFt, SpeedKt};

    #[test]
    fn unknown_fields_stay_unknown() {
        let mut rec = Recorder::new();
        let s = TelemetrySnapshot::empty(fd_core::telemetry::SimTimestamp::new(7));
        let sample = rec.record(&s, "PREFLIGHT");
        assert_eq!(sample.seq, 0);
        assert!(sample.altitude_msl.is_none());
        assert!(sample.radio_altitude.is_none());
        assert!(sample.any_engine_running.is_none());
        assert_eq!(sample.flight_phase, "PREFLIGHT");
        assert_eq!(sample.sim_state, "unknown");
    }

    #[test]
    fn samples_preserve_order_and_present_values() {
        let mut rec = Recorder::new();
        let mut a = TelemetrySnapshot::empty(fd_core::telemetry::SimTimestamp::new(1));
        a.altitude_msl = Some(AltitudeFt::new(100.0));
        let mut b = TelemetrySnapshot::empty(fd_core::telemetry::SimTimestamp::new(2));
        b.altitude_msl = Some(AltitudeFt::new(200.0));
        b.indicated_airspeed = Some(SpeedKt::new(140.0));
        let sa = rec.record(&a, "TAKEOFF");
        let sb = rec.record(&b, "TAKEOFF");
        let rec_flight = FlightRecording {
            samples: vec![sa, sb],
            events: vec![],
        };
        assert_eq!(rec_flight.samples[0].altitude_msl, Some(100.0));
        assert_eq!(rec_flight.samples[1].altitude_msl, Some(200.0));
        assert!(rec_flight.samples[0].indicated_airspeed.is_none());
        assert_eq!(rec_flight.samples[1].seq, 1);
    }

    #[test]
    fn serialization_is_stable() {
        let mut rec = Recorder::new();
        let s = TelemetrySnapshot::empty(fd_core::telemetry::SimTimestamp::new(3));
        let sample = rec.record(&s, "PARKED");
        let text = serde_json::to_string(&sample).unwrap();
        let back: FdrSample = serde_json::from_str(&text).unwrap();
        assert_eq!(back, sample);
    }
}
