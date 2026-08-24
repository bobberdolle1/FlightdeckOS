//! Flight Data Recorder: deterministic, ordered stream of aircraft state
//! samples with attached events.
//!
//! Records ONLY what is present in the canonical snapshot; unknown fields
//! stay unknown (serialized as `null`). Ordering is strictly insertion
//! order — the recorder adds nothing and reorders nothing.

use std::collections::BTreeMap;

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

    // -- V2 fields (Task 6). All defaulted so V1 lines still parse. --------
    /// Geodetic position; `None` when the transport does not report it.
    #[serde(default)]
    pub position: Option<fd_core::telemetry::Position>,
    /// Track (course over ground) TRUE, degrees; `None` = not reported.
    #[serde(default)]
    pub track_true_deg: Option<f64>,
    /// Simulation rate (1.0 = real time); `None` = not reported.
    #[serde(default)]
    pub sim_rate: Option<f64>,
    /// Slew/teleport mode active; `None` = not reported.
    #[serde(default)]
    pub slew: Option<bool>,
    /// Per-channel quality annotations (exception list: absent = fresh).
    /// Carried from the snapshot so downstream analytics can refuse
    /// non-fresh evidence (Task 6 §7-8).
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub channel_quality: std::collections::BTreeMap<u16, fd_core::telemetry::DataQuality>,
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
            position: snap.position,
            // Track is not (yet) a canonical snapshot channel; recorded as
            // unknown rather than derived from heading (ground track vs
            // heading differ under wind).
            track_true_deg: None,
            sim_rate: snap.sim_timing.sim_rate,
            slew: snap.sim_timing.slew_active,
            channel_quality: snap.channel_quality.clone(),
        }
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

/// FDR container format version (Task 6 §13: versioned, streamable,
/// recoverable, replayable). Bumped on incompatible sample/meta schemas.
pub const FDR_FORMAT_VERSION: u32 = 2;

/// First-line container tag of a V2 streamed recording.
const FDR_FORMAT_TAG: &str = "fdos-fdr";

/// Streaming FDR writer (Task 6 §13).
///
/// Writes the versioned JSONL container incrementally — one sample per
/// line — so a crash or Ctrl+C preserves every flushed sample (torn FINAL
/// line is recovered by the loader). Flushes every [`FLUSH_EVERY_SAMPLES`]
/// samples and on [`finish`](Self::finish). The recorder never reads a
/// wall clock; determinism is preserved (wall stamps arrive via meta).
pub struct StreamedRecorder {
    writer: std::io::BufWriter<std::fs::File>,
    samples_since_flush: u64,
    samples_written: u64,
    events_written: u64,
    finished: bool,
}

/// DEVELOPMENT DEFAULT flush cadence: at ~4 Hz this bounds loss to ~8 s of
/// telemetry between flushes.
pub const FLUSH_EVERY_SAMPLES: u64 = 32;

impl StreamedRecorder {
    /// Create the recording file and write the header + meta lines.
    pub fn create(path: &std::path::Path, meta: &FdrSessionMeta) -> Result<Self, FdrIoError> {
        if let Some(parent) = path.parent()
            && !parent.as_os_str().is_empty()
        {
            std::fs::create_dir_all(parent).map_err(FdrIoError::Write)?;
        }
        let file = std::fs::File::create(path).map_err(FdrIoError::Write)?;
        let mut rec = Self {
            writer: std::io::BufWriter::new(file),
            samples_since_flush: 0,
            samples_written: 0,
            events_written: 0,
            finished: false,
        };
        rec.write_line(&serde_json::json!({
            "fdr_format": FDR_FORMAT_TAG,
            "version": FDR_FORMAT_VERSION,
        }))?;
        rec.write_line(&serde_json::json!({ "meta": meta }))?;
        Ok(rec)
    }

    fn write_line(&mut self, value: &serde_json::Value) -> Result<(), FdrIoError> {
        use std::io::Write;
        serde_json::to_writer(&mut self.writer, value).map_err(FdrIoError::Serialize)?;
        self.writer.write_all(b"\n").map_err(FdrIoError::Write)?;
        Ok(())
    }

    /// Stream one sample.
    pub fn record_sample(&mut self, sample: &FdrSample) -> Result<(), FdrIoError> {
        self.write_line(&serde_json::json!({ "sample": sample }))?;
        self.samples_written += 1;
        self.samples_since_flush += 1;
        if self.samples_since_flush >= FLUSH_EVERY_SAMPLES {
            use std::io::Write;
            self.writer.flush().map_err(FdrIoError::Write)?;
            self.samples_since_flush = 0;
        }
        Ok(())
    }

    /// Stream one event.
    pub fn record_event(&mut self, event: &FdrEvent) -> Result<(), FdrIoError> {
        self.write_line(&serde_json::json!({ "event": event }))?;
        self.events_written += 1;
        Ok(())
    }

    /// Flush and close the recording. Idempotent.
    pub fn finish(&mut self) -> Result<(), FdrIoError> {
        use std::io::Write;
        if !self.finished {
            self.write_line(&serde_json::json!({ "session_end": true }))?;
            self.finished = true;
        }
        self.writer.flush().map_err(FdrIoError::Write)
    }

    pub fn samples_written(&self) -> u64 {
        self.samples_written
    }

    pub fn events_written(&self) -> u64 {
        self.events_written
    }
}

impl Drop for StreamedRecorder {
    fn drop(&mut self) {
        // Best-effort flush on drop; explicit finish() is the real close.
        let _ = self.finish();
    }
}

/// FDR container I/O errors (Task 6 §13: fail-closed, never silent).
#[derive(Debug, thiserror::Error)]
pub enum FdrIoError {
    #[error("fdr write: {0}")]
    Write(std::io::Error),
    #[error("fdr serialize: {0}")]
    Serialize(serde_json::Error),
    #[error("fdr parse: {0}")]
    Parse(serde_json::Error),
    #[error("fdr format version mismatch: got {got}, expected {expected}")]
    VersionMismatch { got: u32, expected: u32 },
    #[error("fdr container: {0}")]
    Container(String),
}

impl FlightRecording {
    /// Load a recording from a V2 streamed JSONL file or a legacy V1
    /// pretty-JSON document.
    ///
    /// Recovery semantics (Task 6 §13): a torn FINAL line (crash mid-write)
    /// is dropped with a warning; interior corruption is a hard error —
    /// previous good data is preserved either way.
    pub fn load(path: &std::path::Path) -> Result<Self, FdrIoError> {
        let raw = std::fs::read_to_string(path).map_err(FdrIoError::Write)?;
        let trimmed = raw.trim_start();
        if trimmed.starts_with('{')
            && trimmed.contains("\"samples\"")
            && !trimmed.contains("\"sample\"")
        {
            // Legacy V1: single pretty-JSON document.
            return serde_json::from_str(&raw).map_err(FdrIoError::Parse);
        }
        Self::load_v2(&raw)
    }

    fn load_v2(raw: &str) -> Result<Self, FdrIoError> {
        let mut recording = Self::default();
        let mut saw_header = false;
        let lines: Vec<&str> = raw.lines().collect();
        for (idx, line) in lines.iter().enumerate() {
            let line = line.trim();
            if line.is_empty() {
                continue;
            }
            let is_last = idx + 1 == lines.len();
            let parsed: serde_json::Result<serde_json::Value> = serde_json::from_str(line);
            if let Err(e) = parsed {
                if is_last {
                    // Torn final line: preserve previous good data.
                    eprintln!("fdr load: dropping torn final line: {e}");
                    break;
                }
                return Err(FdrIoError::Parse(e));
            }
            let v = parsed.unwrap();
            if let Some(header) = v.get("fdr_format") {
                if header.as_str() != Some(FDR_FORMAT_TAG) {
                    return Err(FdrIoError::Container(format!(
                        "unknown fdr_format {header:?}"
                    )));
                }
                let version = v.get("version").and_then(|v| v.as_u64()).unwrap_or(0) as u32;
                if version != FDR_FORMAT_VERSION {
                    return Err(FdrIoError::VersionMismatch {
                        got: version,
                        expected: FDR_FORMAT_VERSION,
                    });
                }
                saw_header = true;
                continue;
            }
            if !saw_header {
                return Err(FdrIoError::Container(
                    "first line is not an fdr header".into(),
                ));
            }
            if let Some(meta) = v.get("meta") {
                recording.meta =
                    Some(serde_json::from_value(meta.clone()).map_err(FdrIoError::Parse)?);
            } else if let Some(sample) = v.get("sample") {
                recording
                    .samples
                    .push(serde_json::from_value(sample.clone()).map_err(FdrIoError::Parse)?);
            } else if let Some(event) = v.get("event") {
                recording
                    .events
                    .push(serde_json::from_value(event.clone()).map_err(FdrIoError::Parse)?);
            } else if v.get("session_end").is_some() {
                // Container terminator; trailing lines (if any) are ignored.
            } else {
                return Err(FdrIoError::Container(format!("unknown line kind: {line}")));
            }
        }
        if !saw_header {
            return Err(FdrIoError::Container("missing fdr header".into()));
        }
        Ok(recording)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use fd_core::telemetry::TelemetrySnapshot;
    use fd_core::units::{AltitudeFt, SpeedKt};

    fn test_meta(started_ms: u64) -> FdrSessionMeta {
        FdrSessionMeta {
            session_id: format!("t-{started_ms}"),
            simulator: "test".into(),
            sim_version: None,
            aircraft: AircraftIdentity::unknown(),
            fdos_version: "0.0.0".into(),
            adapter_source: None,
            started_wall_unix_ms: None,
            ended_wall_unix_ms: None,
            origin: None,
            destination: None,
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
    fn v2_sample(seq: u64, ms: u64) -> FdrSample {
        let mut rec = Recorder::new();
        let mut s = TelemetrySnapshot::empty(SimTimestamp::new(ms));
        s.altitude_msl = Some(AltitudeFt::new(5000.0));
        s.position = Some(fd_core::telemetry::Position {
            lat: fd_core::units::LatDeg::new(55.9),
            lon: fd_core::units::LonDeg::new(37.4),
        });
        let mut sample = rec.record(&s, "Cruise");
        sample.seq = seq;
        sample
    }

    #[test]
    fn streamed_round_trip_preserves_samples_and_meta() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("sub").join("rec.jsonl");
        let meta = test_meta(100);
        {
            let mut w = StreamedRecorder::create(&path, &meta).unwrap();
            for seq in 0..40u64 {
                w.record_sample(&v2_sample(seq, 1000 + seq * 250)).unwrap();
            }
            w.record_event(&FdrEvent {
                seq: 1,
                timestamp: SimTimestamp::new(2000),
                kind: "phase".into(),
                detail: "Climb->Cruise".into(),
            })
            .unwrap();
            w.finish().unwrap();
        }
        let loaded = FlightRecording::load(&path).unwrap();
        assert_eq!(loaded.samples.len(), 40);
        assert_eq!(loaded.events.len(), 1);
        assert_eq!(loaded.meta.as_ref().unwrap().session_id, "t-100");
        assert_eq!(loaded.samples[7].seq, 7);
        assert_eq!(loaded.samples[7].altitude_msl, Some(5000.0));
        let pos = loaded.samples[7].position.as_ref().unwrap();
        assert!((pos.lat.value() - 55.9).abs() < 1e-9);
    }

    #[test]
    fn torn_final_sample_recovers_previous_good_data() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("rec.jsonl");
        {
            let mut w = StreamedRecorder::create(&path, &test_meta(1)).unwrap();
            w.record_sample(&v2_sample(0, 100)).unwrap();
            w.record_sample(&v2_sample(1, 350)).unwrap();
            w.finish().unwrap();
        }
        // Simulate a crash mid-write: append a truncated line.
        use std::io::Write as _;
        let mut f = std::fs::OpenOptions::new()
            .append(true)
            .open(&path)
            .unwrap();
        write!(f, "{{\"sample\":{{\"seq\":2,").unwrap();
        drop(f);
        let loaded = FlightRecording::load(&path).unwrap();
        assert_eq!(loaded.samples.len(), 2, "torn tail dropped, good data kept");
        assert_eq!(loaded.samples[1].seq, 1);
    }

    #[test]
    fn interior_corruption_is_hard_error() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("rec.jsonl");
        {
            let mut w = StreamedRecorder::create(&path, &test_meta(1)).unwrap();
            w.record_sample(&v2_sample(0, 100)).unwrap();
            w.finish().unwrap();
        }
        let mut raw = std::fs::read_to_string(&path).unwrap();
        // json! serialization sorts keys alphabetically, so target a value.
        raw = raw.replace("\"seq\":0,", "SEQ BROKEN");
        std::fs::write(&path, &raw).unwrap();
        assert!(
            FlightRecording::load(&path).is_err(),
            "interior corruption fails closed"
        );
    }

    #[test]
    fn version_mismatch_fails_closed() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("rec.jsonl");
        std::fs::write(
            &path,
            "{\"fdr_format\":\"fdos-fdr\",\"version\":99}\n{\"meta\":null}\n",
        )
        .unwrap();
        assert!(matches!(
            FlightRecording::load(&path),
            Err(FdrIoError::VersionMismatch { .. })
        ));
    }

    #[test]
    fn legacy_v1_pretty_json_still_loads() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("legacy.json");
        let legacy = r#"{"samples":[{"seq":0,"timestamp":{"ms":5},"altitude_msl":null,"radio_altitude":null,"indicated_airspeed":null,"groundspeed":null,"vertical_speed":null,"heading_true":null,"pitch":null,"bank":null,"on_ground":null,"gear_down":null,"flaps_handle_index":null,"any_engine_running":null,"autopilot_master":null,"flight_phase":"Parked","sim_state":"running"}],"events":[]}"#;
        std::fs::write(&path, legacy).unwrap();
        let loaded = FlightRecording::load(&path).unwrap();
        assert_eq!(loaded.samples.len(), 1);
        assert_eq!(loaded.samples[0].flight_phase, "Parked");
        // V2 defaults applied to V1 samples.
        assert!(loaded.samples[0].position.is_none());
        assert!(loaded.samples[0].channel_quality.is_empty());
    }

    #[test]
    fn recorder_carries_quality_and_timing_v2_fields() {
        let mut rec = Recorder::new();
        let mut s = TelemetrySnapshot::empty(SimTimestamp::new(9));
        s.sim_timing.sim_rate = Some(2.0);
        s.sim_timing.slew_active = Some(true);
        s.channel_quality
            .insert(7, fd_core::telemetry::DataQuality::Stale);
        let sample = rec.record(&s, "Taxi");
        assert_eq!(sample.sim_rate, Some(2.0));
        assert_eq!(sample.slew, Some(true));
        assert_eq!(
            sample.channel_quality.get(&7),
            Some(&fd_core::telemetry::DataQuality::Stale)
        );
        assert!(sample.track_true_deg.is_none());
    }
}
