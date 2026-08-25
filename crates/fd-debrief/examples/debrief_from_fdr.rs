//! Build a structured debrief from a recorded FDR V2 file (dev tool).
//!
//! Usage: cargo run -p fd-debrief --example debrief_from_fdr -- <fdr.jsonl> [out.json]
//! Offline replay through the production loader + analytics (Task 6 §36).

use fd_debrief::{BuildDebriefArgs, build_debrief};
use fd_fdm::fdm::FdmAnalyzer;
use fd_fdm::fdr::FlightRecording;
use fd_fdm::qoa::ApproachAnalyzer;
use fd_fdm::session::{SessionEvidence, SessionTracker};
use fd_fdm::summary::SessionSummarizer;

fn main() -> anyhow::Result<()> {
    let args: Vec<String> = std::env::args().collect();
    let Some(path) = args.get(1) else {
        anyhow::bail!("usage: debrief_from_fdr <fdr.jsonl> [out.json]");
    };
    let recording = FlightRecording::load(std::path::Path::new(path))?;
    println!(
        "loaded: samples={} events={} origin={:?} dest={:?}",
        recording.samples.len(),
        recording.events.len(),
        recording.meta.as_ref().and_then(|m| m.origin.clone()),
        recording.meta.as_ref().and_then(|m| m.destination.clone())
    );
    let mut fdm = FdmAnalyzer::new_development_default();
    let mut qoa = ApproachAnalyzer::new(Default::default(), None);
    let mut session = SessionTracker::new();
    for s in &recording.samples {
        fdm.process(s);
        qoa.push(s.clone());
        session.advance(SessionEvidence {
            connected: true,
            identity_known: recording
                .meta
                .as_ref()
                .map(|m| m.aircraft.icao.is_some())
                .unwrap_or(false),
            sample_recorded: true,
            altitude_agl_ft: s.radio_altitude,
            on_ground: s.on_ground,
            groundspeed_kt: s.groundspeed,
            descending: s.vertical_speed.map(|v| v < -100.0),
        });
    }
    let meta = recording.meta.clone();
    let debrief = build_debrief(BuildDebriefArgs {
        identity: meta
            .as_ref()
            .map(|m| m.aircraft.clone())
            .unwrap_or_default(),
        session: &session,
        sample_count: recording.samples.len() as u64,
        origin: meta.as_ref().and_then(|m| m.origin.as_deref()),
        destination: meta.as_ref().and_then(|m| m.destination.as_deref()),
        route_source_str: None,
        waypoint_count: 0,
        route_usable: false,
        off_route_events: 0,
        route_complete: false,
        summary: &{
            let mut sum = SessionSummarizer::new();
            for s in &recording.samples {
                sum.push_sample(s);
            }
            sum.finish().0
        },
        landing_window: &recording.samples[recording.samples.len().saturating_sub(4096)..],
        plan: None,
        fdm_events: recording.events.len() as u64,
        approach: &qoa.finish(),
        runway: None,
        shadow_summary: None,
    })?;
    let json = debrief.to_json_pretty()?;
    match args.get(2) {
        Some(out) => std::fs::write(out, json)?,
        None => println!("{json}"),
    }
    Ok(())
}
