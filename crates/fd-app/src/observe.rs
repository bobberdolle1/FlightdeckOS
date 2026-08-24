//! `fd observe` — LIVE FLIGHT OBSERVATORY V1 (Task 6 §56).
//!
//! Connect → identify → record → analyze → finish, in ONE process, with
//! ZERO simulator writes: the adapter is always constructed with
//! `allow_writes: false` and the mission shadow drives a null control
//! sink. Nothing in this module can move the aircraft.

use anyhow::Context as _;
use fd_core::adapter::{FlightControlTargets, SimulatorAdapter};
use fd_core::identity::AircraftIdentity;
use fd_core::phase::FlightPhaseEngine;
use fd_core::telemetry::TelemetrySnapshot;
use fd_fdm::fdm::FdmAnalyzer;
use fd_fdm::fdr::{FdrEvent, StreamedRecorder};
use fd_fdm::qoa::ApproachAnalyzer;
use fd_fdm::qol;
use fd_fdm::session::{SessionEvidence, SessionTracker};
use fd_mission::controller::{MissionContext, MissionController, MissionParameters};
use fd_mission::intents::intent_from_tick;
use fd_mission::monitor::{
    OffRouteConfig, OffRouteDetector, RouteMonitor, RouteSource, RouteState,
};
use fd_mission::runway::RunwayContext;
use fd_mission::shadow::{MissionShadow, ObservedApTargets};
use fd_openairac::NavDataStore;
use fd_xplane::{XPlaneAdapter, XPlaneConfig};

/// Options for one observation session.
pub struct ObserveOpts {
    pub port: u16,
    pub monitor_secs: u64,
    pub wait_first_secs: u64,
    pub aircraft_icao: Option<String>,
    pub fdr_out: std::path::PathBuf,
    pub debrief_out: Option<std::path::PathBuf>,
    /// Operator-declared origin ICAO (evidence: operator).
    pub origin_icao: Option<String>,
    /// Operator-declared destination ICAO (evidence: operator).
    pub destination_icao: Option<String>,
    /// OpenAIRAC world store path (read-only) for airport/runway context.
    pub world_store: Option<std::path::PathBuf>,
    /// Declaring a cruise altitude ARMS the Mission Shadow (zero-write):
    /// without a mission definition the shadow honestly reports nothing.
    pub cruise_altitude_ft: Option<f64>,
}

/// Null control sink for the Mission Shadow: every setter is a no-op.
/// The shadow NEVER writes (Task 6 §48, §52) — this type is the
/// compile-level guarantee for the observe path.
struct NullControls;

impl FlightControlTargets for NullControls {
    fn flight_guidance_supported(&self) -> bool {
        true
    }
    fn set_target_altitude(&mut self, _altitude_ft: f64) {}
    fn set_target_heading(&mut self, _heading_deg: f64) {}
    fn set_target_vertical_speed(&mut self, _vs_fpm: f64) {}
    fn set_target_speed(&mut self, _speed_kt: f64) {}
}

/// Bridge: fd-fdm's RunwayGeometry consumed from an fd-mission RunwayContext
/// without coupling the two crates (dependency direction preserved).
struct RunwayBridge<'a>(&'a RunwayContext);

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

/// OpenAIRAC-derived navigation context for the session.
struct NavContext {
    store: NavDataStore,
    at: String,
    origin_icao: String,
    destination_icao: String,
    runway: Option<RunwayContext>,
}

impl NavContext {
    fn resolve(opts: &ObserveOpts) -> anyhow::Result<Option<Self>> {
        let (Some(store_path), Some(origin), Some(dest)) =
            (&opts.world_store, &opts.origin_icao, &opts.destination_icao)
        else {
            return Ok(None);
        };
        let store = NavDataStore::open_read_only(store_path)
            .context("open OpenAIRAC world store (read-only)")?;
        // Reference instant pin: deterministic dataset revision for the
        // current world store build (Task 6 §14). A future multi-cycle
        // store passes the cycle time explicitly.
        let at = fd_openairac::REFERENCE_QUERY_INSTANT.to_string();
        let o = store
            .airport_by_icao(origin, &at)?
            .with_context(|| format!("origin {origin} not in OpenAIRAC store"))?;
        let d = store
            .airport_by_icao(dest, &at)?
            .with_context(|| format!("destination {dest} not in OpenAIRAC store"))?;
        let runways = store.runways(dest, &at)?;
        // Runway selection: DEVELOPMENT DEFAULT — the first runway with
        // complete threshold geometry. NOT a wind/ATC-informed selection;
        // evidence string says exactly that.
        let runway = runways
            .iter()
            .find(|r| r.le_lat.is_some() && r.he_lat.is_some() && r.true_heading_deg.is_some())
            .map(|r| RunwayContext {
                runway: fd_mission::runway::Runway {
                    airport_icao: r.airport_ident.clone(),
                    le_ident: r.le_ident.clone(),
                    he_ident: r.he_ident.clone(),
                    length_ft: r.length_ft.unwrap_or(0.0),
                    ends: [
                        fd_mission::runway::RunwayEnd {
                            ident: r.le_ident.clone(),
                            lat_deg: r.le_lat.unwrap(),
                            lon_deg: r.le_lon.unwrap(),
                            elevation_ft: r.le_elevation_ft.unwrap_or(0.0),
                            true_heading_deg: r.true_heading_deg.unwrap(),
                        },
                        fd_mission::runway::RunwayEnd {
                            ident: r.he_ident.clone(),
                            lat_deg: r.he_lat.unwrap(),
                            lon_deg: r.he_lon.unwrap(),
                            elevation_ft: r.he_elevation_ft.unwrap_or(0.0),
                            true_heading_deg: (r.true_heading_deg.unwrap() + 180.0)
                                .rem_euclid(360.0),
                        },
                    ],
                },
                landing_end: 0,
                evidence: "dev_default:first_complete_geometry (not wind/ATC informed)".into(),
            });
        println!(
            "OPENAIRAC_CONTEXT: origin={}({:.4},{:.4},elev={:?}ft) dest={} runways={} selected={:?}",
            o.ident,
            o.lat_deg,
            o.lon_deg,
            o.elevation_ft,
            d.ident,
            runways.len(),
            runway.as_ref().map(|r| r.runway.le_ident.clone())
        );
        Ok(Some(Self {
            store,
            at,
            origin_icao: origin.clone(),
            destination_icao: dest.clone(),
            runway,
        }))
    }

    /// Route state from the declared airports (great-circle 2-point route
    /// through airport coordinates; RU enroute airways are not usable in
    /// the current dataset — honest degradation, Task 6 §14).
    fn route_state(&self) -> RouteState {
        let o = self
            .store
            .airport_by_icao(&self.origin_icao, &self.at)
            .ok()
            .flatten();
        let d = self
            .store
            .airport_by_icao(&self.destination_icao, &self.at)
            .ok()
            .flatten();
        match (o, d) {
            (Some(o), Some(d)) => RouteState {
                source: RouteSource::OpenAirac {
                    provenance: format!("world@{}", self.at),
                },
                waypoints: vec![
                    fd_mission::route::Waypoint {
                        id: o.ident.clone(),
                        lat_deg: o.lat_deg,
                        lon_deg: o.lon_deg,
                    },
                    fd_mission::route::Waypoint {
                        id: d.ident.clone(),
                        lat_deg: d.lat_deg,
                        lon_deg: d.lon_deg,
                    },
                ],
            },
            _ => RouteState {
                source: RouteSource::Unknown,
                waypoints: vec![],
            },
        }
    }
}

/// Run one zero-write observation session.
pub fn run_observe(opts: ObserveOpts) -> anyhow::Result<()> {
    println!("FLIGHTDECKOS LIVE FLIGHT OBSERVATORY (zero-write observation)");
    let cfg = XPlaneConfig {
        host: "127.0.0.1".into(),
        port: opts.port,
        subscribe_hz: 4,
        // OBSERVATORY INVARIANT: the observer never arms writes (§48).
        allow_writes: false,
        ..XPlaneConfig::default()
    };
    let identity = match &opts.aircraft_icao {
        Some(icao) => AircraftIdentity {
            icao: Some(icao.clone()),
            tail_number: None,
            author: None,
            description: None,
            acf_name: None,
            source: fd_core::identity::IdentitySource::UserProvided,
        },
        None => AircraftIdentity::unknown(),
    };
    let mut adapter = XPlaneAdapter::with_identity(cfg, identity)?;

    // Bounded first-packet wait (XP boot window).
    let wait_started = std::time::Instant::now();
    loop {
        match adapter.poll() {
            Ok(v) if !v.is_empty() => break,
            _ => {
                if wait_started.elapsed().as_secs() >= opts.wait_first_secs {
                    return Err(anyhow::anyhow!(
                        "no telemetry from X-Plane within {}s (is the sim running with a loaded flight?)",
                        opts.wait_first_secs
                    ));
                }
                std::thread::sleep(std::time::Duration::from_millis(500));
            }
        }
    }
    let sim_version = adapter.simulator_version();
    println!(
        "CONNECTED: identity={:?} (source={:?}) sim_version={:?}",
        adapter.identity().icao,
        adapter.identity().source,
        sim_version
    );

    // Navigation context (optional, operator-declared + OpenAIRAC).
    let nav = NavContext::resolve(&opts)?;
    let route_state = nav.as_ref().map(|n| n.route_state()).unwrap_or(RouteState {
        source: RouteSource::Unknown,
        waypoints: vec![],
    });
    let mut route_monitor = RouteMonitor::new(&route_state);
    let mut off_route = OffRouteDetector::new(OffRouteConfig::default());
    let route_usable = route_state.is_usable();

    // Mission Shadow (zero-write): armed ONLY by an explicit mission
    // definition (§30: without a mission the shadow reports nothing).
    let mut shadow = opts.cruise_altitude_ft.map(|cruise| {
        (
            MissionController::new(MissionParameters {
                cruise_altitude_ft: cruise,
                ..MissionParameters::default()
            }),
            MissionShadow::new(),
            NullControls,
        )
    });

    // Analytics.
    let mut fdr_seq = fd_fdm::fdr::Recorder::new();
    let mut writer =
        StreamedRecorder::create(&opts.fdr_out, &live_meta(&adapter, sim_version, &opts))?;
    let mut session = SessionTracker::new();
    let mut phase_engine = FlightPhaseEngine::new();
    let mut fdm = FdmAnalyzer::new_development_default();
    let mut qoa = ApproachAnalyzer::new(Default::default(), None);
    let mut fdr_events = 0u64;
    let mut samples: Vec<fd_fdm::fdr::FdrSample> = Vec::new();
    let mut route_observations = 0u64;
    let mut off_route_events = 0u64;
    let mut route_complete = false;

    let started = std::time::Instant::now();
    loop {
        std::thread::sleep(std::time::Duration::from_millis(250));
        let snap = match adapter.poll() {
            Ok(v) => v.into_iter().next(),
            Err(_) => continue,
        };
        let Some(s) = snap else { continue };

        // Session lifecycle evidence.
        session.advance(SessionEvidence {
            connected: true,
            identity_known: adapter.identity().icao.is_some(),
            sample_recorded: true,
            altitude_agl_ft: s.altitude_agl.map(|v| v.value()),
            on_ground: s.on_ground,
            groundspeed_kt: s.groundspeed.map(|v| v.value()),
            descending: s.vertical_speed.map(|v| v.value() < -100.0),
        });

        // Phase + FDR + FDM + QoA.
        let assessment = phase_engine.evaluate(&fd_core::phase::PhaseTelemetry::from(&s));
        let sample = fdr_seq.record(&s, assessment.phase.as_str());
        for ev in fdm.process(&sample) {
            fdr_events += 1;
            writer
                .record_event(&FdrEvent {
                    seq: fdr_events,
                    timestamp: sample.timestamp,
                    kind: "fdm".into(),
                    detail: format!("{:?} measured={:.0}", ev.kind, ev.measured),
                })
                .map_err(|e| anyhow::anyhow!("{e}"))?;
        }
        qoa.push(sample.clone());

        // Route monitoring (deterministic, zero-write).
        if route_usable && let Some(pos) = &s.position {
            route_observations += 1;
            let obs = route_monitor.update(pos.lat.value(), pos.lon.value());
            route_complete = obs.route_complete;
            if off_route.update(sample.seq, &obs).is_some() {
                off_route_events += 1;
                println!(
                    "OFF_ROUTE_EVENT: seq={} peak_xtk={:.1}nm (dev config)",
                    sample.seq,
                    obs.cross_track_error_nm.unwrap_or(f64::NAN)
                );
            }
        }

        // Mission Shadow: intended vs observed, zero writes.
        if let (Some((controller, shadow_rec, null)), Some(pos), true) =
            (shadow.as_mut(), &s.position, route_usable)
        {
            let obs = route_monitor.update(pos.lat.value(), pos.lon.value());
            // Bearing to the ACTIVE waypoint from route geometry
            // (fd_core geo), not a coordinate-space hack.
            let bearing = route_state
                .waypoints
                .get(obs.active_leg.map(|i| i + 1).unwrap_or(1))
                .map(|wp| {
                    fd_core::geo::initial_bearing_deg(
                        pos.lat.value(),
                        pos.lon.value(),
                        wp.lat_deg,
                        wp.lon_deg,
                    )
                })
                .unwrap_or(0.0);
            let ctx = MissionContext {
                snapshot: &s,
                distance_to_destination_nm: obs.destination_distance_nm.unwrap_or(0.0),
                bearing_to_waypoint_deg: bearing,
            };
            let phase = controller_phase(controller);
            let params = controller_params(controller).clone();
            let follower = &mut dummy_follower(&route_state);
            let cmds = controller.step(&ctx, null, follower);
            // Intent derived from the SAME tick output (single decision
            // source); carried by the shadow entries with reasons.
            let _intent = intent_from_tick(&phase, &cmds, &ctx, &params);
            shadow_rec.observe(
                sample.seq,
                phase,
                &ctx,
                &params,
                ObservedApTargets {
                    heading_deg: None,        // live AP dial reads not subscribed
                    altitude_ft: None,        // in this milestone; channels stay
                    vertical_speed_fpm: None, // honestly Unknown
                    speed_kt: None,
                },
            );
        }

        writer
            .record_sample(&sample)
            .map_err(|e| anyhow::anyhow!("{e}"))?;
        samples.push(sample);

        if opts.monitor_secs > 0 && started.elapsed().as_secs() >= opts.monitor_secs {
            break;
        }
    }
    writer.finish().map_err(|e| anyhow::anyhow!("{e}"))?;
    println!(
        "FDR_SESSION: samples={} events={} -> {} (v2 jsonl)",
        writer.samples_written(),
        writer.events_written(),
        opts.fdr_out.display()
    );
    println!("SESSION_LIFECYCLE: {:?}", session.state());
    if route_usable {
        println!(
            "ROUTE_MONITOR: observations={} off_route_events={} (dev config)",
            route_observations, off_route_events
        );
    } else {
        println!("ROUTE_MONITOR: no usable route — monitoring honestly absent");
    }

    // Debrief.
    if let Some(debrief_path) = &opts.debrief_out {
        let approach = qoa.finish();
        let recording = fd_fdm::fdr::FlightRecording {
            meta: None,
            samples: samples.clone(),
            events: vec![],
        };
        let landing = match nav.as_ref().and_then(|n| n.runway.as_ref()) {
            Some(rw) => qol::analyze_with_runway(&recording, &RunwayBridge(rw)),
            None => qol::analyze(&samples),
        };
        let mut debrief = fd_debrief::FlightDebrief::new(adapter.identity().clone());
        debrief.session = serde_json::json!({
            "state": format!("{:?}", session.state()),
            "samples": writer.samples_written(),
            "ever_airborne": session.ever_airborne(),
            "adapter_source": "xplane-udp",
            "origin": opts.origin_icao,
            "destination": opts.destination_icao,
        });
        debrief.route = fd_debrief::RouteSummary {
            source: route_usable.then(|| format!("{:?}", route_state.source)), // format! is lazy: keep then
            waypoint_count: route_usable.then_some(route_state.waypoints.len()),
            off_route_events,
            completed: route_usable.then_some(route_complete),
        };
        debrief.phase_timeline = fd_debrief::phase_timeline_from_samples(
            &samples
                .iter()
                .map(|s| (s.timestamp.ms, s.flight_phase.as_str()))
                .collect::<Vec<_>>(),
        );
        debrief.fdm_summary = serde_json::json!({
            "events": fdr_events,
        });
        debrief.approach = serde_json::to_value(&approach)?;
        debrief.landing = serde_json::to_value(&landing)?;
        debrief.shadow = serde_json::json!(match &shadow {
            Some((_, rec, _)) => serde_json::to_value(rec.summary())?,
            None => serde_json::Value::Null,
        });
        debrief.data_quality = fd_debrief::data_quality_summary(
            samples.len() as u64,
            samples.iter().map(|s| s.channel_quality.clone()),
        );
        let json = debrief.to_json_pretty()?;
        std::fs::write(debrief_path, json).context("write debrief")?;
        println!("DEBRIEF: {}", debrief_path.display());
    }
    Ok(())
}

// Small accessors so the shadow block reads cleanly without cloning the
// controller each tick.
fn controller_phase(c: &MissionController) -> fd_mission::MissionPhase {
    c.phase()
}

fn controller_params(c: &MissionController) -> &MissionParameters {
    c.params()
}

/// A throwaway follower for the controller's signature; the observe path
/// never uses its guidance (the shadow is read-only).
fn dummy_follower(state: &RouteState) -> fd_mission::route::RouteFollower {
    fd_mission::route::RouteFollower::new(state.waypoints.clone(), 2.5)
}

fn live_meta(
    adapter: &XPlaneAdapter,
    sim_version: Option<String>,
    opts: &ObserveOpts,
) -> fd_fdm::fdr::FdrSessionMeta {
    let started_wall = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .ok()
        .map(|d| d.as_millis() as u64);
    fd_fdm::fdr::FdrSessionMeta {
        session_id: format!("observe-{}", started_wall.unwrap_or(0)),
        simulator: "X-Plane 12".into(),
        sim_version,
        aircraft: adapter.identity().clone(),
        fdos_version: env!("CARGO_PKG_VERSION").into(),
        adapter_source: Some("xplane-udp".into()),
        started_wall_unix_ms: started_wall,
        ended_wall_unix_ms: None,
        origin: opts.origin_icao.clone(),
        destination: opts.destination_icao.clone(),
        started_ms: 0,
        ended_ms: None,
    }
}

// -- unused-silence helpers --------------------------------------------------
// The TelemetrySnapshot import is used in the MissionContext construction
// above; keep the import list honest.
#[allow(dead_code)]
fn _type_anchor(_s: &TelemetrySnapshot) {}
