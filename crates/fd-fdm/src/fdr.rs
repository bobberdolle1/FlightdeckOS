//! Flight Data Recorder: deterministic, ordered stream of aircraft state
//! samples with attached events.
//!
//! Records ONLY what is present in the canonical snapshot; unknown fields
//! stay unknown (serialized as `null`). Ordering is strictly insertion
//! order — the recorder adds nothing and reorders nothing.

use fd_core::identity::AircraftIdentity;
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

/// Session-level metadata attached to a recording (spec §19).
///
/// Pure data supplied by the caller — the recorder never reads a wall
/// clock, so identical inputs still produce identical recordings.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct FdrSessionMeta {
    /// Caller-chosen unique session identifier.
    pub session_id: String,
    /// Simulator product name (e.g. `"X-Plane 12"`).
    pub simulator: String,
    /// Simulator version string, when known.
    #[serde(default)]
    pub sim_version: Option<String>,
    /// Aircraft identity claim with provenance.
    pub aircraft: AircraftIdentity,
    /// FlightdeckOS version that produced this recording.
    pub fdos_version: String,
    /// Adapter/transport that produced the samples (e.g.
    /// "xplane-udp", "virtual", "replay"). Diagnostics only.
    #[serde(default)]
    pub adapter_source: Option<String>,
    /// Wall-clock start (unix ms) for cross-referencing logs. NOT part of
    /// the deterministic timeline.
    #[serde(default)]
    pub started_wall_unix_ms: Option<u64>,
    /// Wall-clock end (unix ms), stamped by `finish`.
    #[serde(default)]
    pub ended_wall_unix_ms: Option<u64>,
    /// Departure airport ICAO, when known.
    #[serde(default)]
    pub origin: Option<String>,
    /// Destination airport ICAO, when known.
    #[serde(default)]
    pub destination: Option<String>,
    /// Session start in sim milliseconds (caller-supplied).
    pub started_ms: u64,
    /// Session end in sim milliseconds; `None` while the session is open.
    #[serde(default)]
    pub ended_ms: Option<u64>,
}

/// The recording itself. Deterministic: same input sequence ⇒ same output.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct FlightRecording {
    /// Session metadata (spec §19). Absent (`None`) in legacy recordings;
    /// old JSON without this key still deserializes.
    #[serde(default)]
    pub meta: Option<FdrSessionMeta>,
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
    meta: Option<FdrSessionMeta>,
}

impl Recorder {
    pub fn new() -> Self {
        Self::default()
    }

    /// Attach session metadata to be stamped on finished recordings
    /// (builder style).
    pub fn with_meta(mut self, meta: FdrSessionMeta) -> Self {
        self.meta = Some(meta);
        self
    }

    /// Session metadata attached to this recorder, if any.
    pub fn meta(&self) -> Option<&FdrSessionMeta> {
        self.meta.as_ref()
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

    /// Seal a finished recording.
    ///
    /// Attaches this recorder's session metadata when the recording has
    /// none, stamping `ended_ms` from the last recorded sample's sim
    /// timestamp. A sample-free recording keeps `ended_ms: None` — an
    /// honest unknown. Deterministic: derived from recorded data only,
    /// never the wall clock.
    pub fn finish(&self, mut recording: FlightRecording) -> FlightRecording {
        if recording.meta.is_none() {
            let mut sealed = self.meta.clone();
            if let Some(meta) = sealed.as_mut()
                && let Some(last) = recording.samples.last()
            {
                meta.ended_ms = Some(last.timestamp.ms);
            }
            recording.meta = sealed;
        }
        recording
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use fd_core::telemetry::TelemetrySnapshot;
    use fd_core::units::{AltitudeFt, SpeedKt};

    fn test_meta(started_ms: u64) -> FdrSessionMeta {
        FdrSessionMeta {
            session_id: "session-0001".to_string(),
            simulator: "X-Plane 12".to_string(),
            sim_version: Some("12.1.4".to_string()),
            aircraft: AircraftIdentity::user_provided(Some("A320".to_string())),
            fdos_version: "0.1.0".to_string(),
            adapter_source: None,
            started_wall_unix_ms: None,
            ended_wall_unix_ms: None,
            origin: Some("EDDF".to_string()),
            destination: Some("EDDM".to_string()),
            started_ms,
            ended_ms: None,
        }
    }

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
            meta: None,
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

    #[test]
    fn with_meta_is_visible_via_accessor() {
        let meta = test_meta(100);
        let rec = Recorder::new().with_meta(meta.clone());
        assert_eq!(rec.meta(), Some(&meta));

        let plain = Recorder::new();
        assert_eq!(plain.meta(), None);
    }

    #[test]
    fn finish_attaches_meta_with_ended_ms_from_last_sample() {
        let mut rec = Recorder::new().with_meta(test_meta(500));
        let s1 = TelemetrySnapshot::empty(SimTimestamp::new(1000));
        let s2 = TelemetrySnapshot::empty(SimTimestamp::new(1500));
        let mut recording = FlightRecording::default();
        recording.push_sample(rec.record(&s1, "CRUISE"));
        recording.push_sample(rec.record(&s2, "DESCENT"));

        let finished = rec.finish(recording);
        let meta = finished
            .meta
            .as_ref()
            .expect("meta must be attached by finish");
        assert_eq!(meta.started_ms, 500);
        assert_eq!(meta.ended_ms, Some(1500));
        assert_eq!(finished.len(), 2);
    }

    #[test]
    fn finish_without_samples_keeps_ended_ms_unknown() {
        let rec = Recorder::new().with_meta(test_meta(10));
        let finished = rec.finish(FlightRecording::default());
        let meta = finished.meta.expect("meta present");
        assert_eq!(meta.ended_ms, None);
    }

    #[test]
    fn finish_without_meta_leaves_recording_meta_free() {
        let mut rec = Recorder::new();
        let s = TelemetrySnapshot::empty(SimTimestamp::new(42));
        let mut recording = FlightRecording::default();
        recording.push_sample(rec.record(&s, "TAXI"));
        let finished = rec.finish(recording);
        assert!(finished.meta.is_none());
    }

    #[test]
    fn finish_never_overrides_existing_meta() {
        let rec = Recorder::new().with_meta(test_meta(1));
        let existing = FdrSessionMeta {
            ended_ms: Some(9999),
            ..test_meta(77)
        };
        let recording = FlightRecording {
            meta: Some(existing.clone()),
            ..FlightRecording::default()
        };
        let finished = rec.finish(recording);
        assert_eq!(finished.meta, Some(existing));
    }

    #[test]
    fn session_meta_serde_roundtrip() {
        let meta = test_meta(321);
        let text = serde_json::to_string(&meta).unwrap();
        let back: FdrSessionMeta = serde_json::from_str(&text).unwrap();
        assert_eq!(back, meta);

        // Optional fields may be omitted in hand-written JSON.
        let sparse: FdrSessionMeta = serde_json::from_str(
            r#"{
                "session_id": "s",
                "simulator": "XP12",
                "aircraft": {"source": "unknown"},
                "fdos_version": "0.1.0",
                "started_ms": 5
            }"#,
        )
        .unwrap();
        assert_eq!(sparse.session_id, "s");
        assert_eq!(sparse.aircraft, AircraftIdentity::default());
        assert_eq!(sparse.destination, None);
        assert_eq!(sparse.ended_ms, None);
    }

    #[test]
    fn old_recording_json_without_meta_still_loads() {
        // Legacy fixture shape: no `meta` key at all.
        let legacy = r#"{"samples":[],"events":[]}"#;
        let recording: FlightRecording = serde_json::from_str(legacy).unwrap();
        assert!(recording.meta.is_none());
        assert!(recording.is_empty());

        // Roundtrip with meta keeps equality.
        let mut rec = Recorder::new().with_meta(test_meta(8));
        let s = TelemetrySnapshot::empty(SimTimestamp::new(80));
        let mut flight = FlightRecording::default();
        flight.push_sample(rec.record(&s, "CLIMB"));
        let finished = rec.finish(flight);
        let text = serde_json::to_string(&finished).unwrap();
        let back: FlightRecording = serde_json::from_str(&text).unwrap();
        assert_eq!(back, finished);
    }
}
