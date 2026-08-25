//! `fd observe` — LIVE FLIGHT OBSERVATORY V2 (Task 7).
//!
//! Connect → identify → observe FMS → record → analyze → finish, in ONE
//! process, with ZERO simulator writes: the adapter is always constructed
//! with `allow_writes: false` and the mission shadow drives a null
//! control sink. Nothing in this module can move the aircraft.
//!
//! Task 7 additions:
//! - FMS observation via the read-only XPLM bridge (§5-13), correlated
//!   against OpenAIRAC (§14-17) and fed into the RouteMonitor (§26);
//! - flight-plan change events recorded into the FDR stream (§37) so
//!   replay can rebuild the same understanding offline (§35-36);
//! - bounded memory: samples stream to disk; only the
//!   [`SessionSummarizer`] state is retained (§45);
//! - a bounded [`CrewView`] written at session end (§33-34);
//! - periodic concise status lines (§47) and automatic debrief (§48).

use anyhow::Context as _;
use fd_core::adapter::{FlightControlTargets, SimulatorAdapter};
use fd_core::fplan::{
    FlightPlanChange, FmsEntryKind, FmsSnapshot, NavMatch, ProcedurePhase, classify_primary_change,
};
use fd_core::identity::AircraftIdentity;
use fd_core::phase::FlightPhaseEngine;
use fd_fdm::fdm::FdmAnalyzer;
use fd_fdm::fdr::{FdrEvent, FdrEventPayload, StreamedRecorder};
use fd_fdm::qoa::ApproachAnalyzer;
use fd_fdm::session::{SessionEvidence, SessionTracker};
use fd_fdm::summary::SessionSummarizer;
use fd_mission::controller::{MissionContext, MissionController, MissionParameters};
use fd_mission::intents::intent_from_tick;
use fd_mission::monitor::{
    OffRouteConfig, OffRouteDetector, RouteMonitor, RouteSource, RouteState,
};
use fd_mission::runway::RunwayContext;
use fd_mission::shadow::{MissionShadow, ObservedApTargets};
use fd_openairac::NavDataStore;
use fd_openairac::match_nav::{FacilityFilter, best_procedure, correlate_procedures, match_entry};
use fd_xplane::bridge::{BridgeError, FmsBridgeClient};
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
    /// FMS bridge port (0 disables FMS observation).
    pub fms_bridge_port: u16,
    /// Bounded crew-view JSON output (Task 7 §33-34).
    pub crew_view_out: Option<std::path::PathBuf>,
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

/// OpenAIRAC-derived navigation context for the session.
struct NavContext {
    store: NavDataStore,
    at: String,
    /// Operator-declared endpoints (either may be absent; the FMS
    /// observation takes precedence when it resolves an airport).
    origin_icao: Option<String>,
    destination_icao: Option<String>,
}

impl NavContext {
    fn resolve(opts: &ObserveOpts) -> anyhow::Result<Option<Self>> {
        let Some(store_path) = &opts.world_store else {
            return Ok(None);
        };
        let store = NavDataStore::open_read_only(store_path)
            .context("open OpenAIRAC world store (read-only)")?;
        // Reference instant pin: deterministic dataset revision for the
        // current world store build (Task 6 §14). A future multi-cycle
        // store passes the cycle time explicitly.
        let at = fd_openairac::REFERENCE_QUERY_INSTANT.to_string();
        for (role, icao) in [
            ("origin", &opts.origin_icao),
            ("destination", &opts.destination_icao),
        ] {
            if let Some(icao) = icao {
                let ap = store
                    .airport_by_icao(icao, &at)?
                    .with_context(|| format!("{role} {icao} not in OpenAIRAC store"))?;
                println!(
                    "OPENAIRAC_CONTEXT: {}={}({:.4},{:.4},elev={:?}ft)",
                    role, ap.ident, ap.lat_deg, ap.lon_deg, ap.elevation_ft
                );
            }
        }
        Ok(Some(Self {
            store,
            at,
            origin_icao: opts.origin_icao.clone(),
            destination_icao: opts.destination_icao.clone(),
        }))
    }

    /// Runway context for an airport (dev-default first complete
    /// geometry — NOT wind/ATC informed; evidence says exactly that).
    fn runway_for(&self, icao: &str) -> anyhow::Result<Option<RunwayContext>> {
        let runways = self.store.runways(icao, &self.at)?;
        Ok(runways
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
            }))
    }

    /// Route state from the declared airports (great-circle 2-point route
    /// through airport coordinates; RU enroute airways are not usable in
    /// the current dataset — honest degradation, Task 6 §14).
    fn route_state_openairac(&self) -> RouteState {
        let (Some(origin), Some(dest)) = (&self.origin_icao, &self.destination_icao) else {
            return RouteState {
                source: RouteSource::Unknown,
                waypoints: vec![],
            };
        };
        let o = self.store.airport_by_icao(origin, &self.at).ok().flatten();
        let d = self.store.airport_by_icao(dest, &self.at).ok().flatten();
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

/// Correlation outcome for one FMS snapshot (computed on change only).
struct PlanCorrelation {
    route: RouteState,
    matches: Vec<NavMatch>,
    procedure: Option<fd_core::fplan::ProcedureContext>,
    procedure_phase: ProcedurePhase,
    destination_airport: Option<String>,
}

/// Map an FMS entry kind to the correlation facility filter.
fn facility_filter(kind: FmsEntryKind) -> FacilityFilter {
    match kind {
        FmsEntryKind::Airport => FacilityFilter::Airport,
        FmsEntryKind::Fix | FmsEntryKind::LatLon => FacilityFilter::Waypoint,
        FmsEntryKind::Vor | FmsEntryKind::Ndb => FacilityFilter::Navaid,
        FmsEntryKind::Unknown => FacilityFilter::Any,
    }
}

/// Correlate an FMS snapshot against OpenAIRAC and build the route.
///
/// Route geometry uses ONLY positioned entries (§50: an unknown position
/// never becomes a guessed route point).
fn correlate_snapshot(nav: &NavContext, snapshot: &FmsSnapshot) -> PlanCorrelation {
    let Some(primary) = snapshot.primary() else {
        return PlanCorrelation {
            route: RouteState {
                source: RouteSource::Unknown,
                waypoints: vec![],
            },
            matches: vec![],
            procedure: None,
            procedure_phase: ProcedurePhase::Unknown,
            destination_airport: None,
        };
    };
    let mut matches = Vec::new();
    let mut waypoints = Vec::new();
    let mut destination_airport: Option<String> = None;
    for e in &primary.entries {
        let m = match_entry(
            &nav.store,
            e.id.as_deref(),
            e.lat_deg,
            e.lon_deg,
            facility_filter(e.kind),
            &nav.at,
        );
        if let (Some(lat), Some(lon)) = (e.lat_deg, e.lon_deg) {
            waypoints.push(fd_mission::route::Waypoint {
                id: e.id.clone().unwrap_or_else(|| "----".into()),
                lat_deg: lat,
                lon_deg: lon,
            });
        }
        if e.kind == FmsEntryKind::Airport
            && let NavMatch::Matched { ident, .. } = &m
        {
            destination_airport = Some(ident.clone());
        }
        matches.push(m);
    }
    let route = if waypoints.len() >= 2 {
        RouteState {
            source: RouteSource::XPlaneFms {
                provenance: format!("bridge rev {:016x}", snapshot.revision_hash),
            },
            waypoints,
        }
    } else {
        nav.route_state_openairac()
    };
    // Procedure correlation at the RESOLVED destination (§15: never from
    // proximity alone).
    let dest_for_proc = destination_airport
        .clone()
        .or_else(|| nav.destination_icao.clone());
    let fix_ids: Vec<String> = primary
        .entries
        .iter()
        .filter_map(|e| e.id.clone())
        .collect();
    let (procedure, procedure_phase) = match &dest_for_proc {
        Some(ap) => {
            let contexts =
                correlate_procedures(&nav.store, ap, &fix_ids, &nav.at).unwrap_or_default();
            match best_procedure(&contexts) {
                Some(ctx) => {
                    // Navigation phase (§16): deterministic only.
                    let approach_loaded = snapshot.approach_loaded() == Some(true);
                    let phase = match ctx.kind {
                        fd_core::fplan::ProcedureKind::Approach if approach_loaded => {
                            ProcedurePhase::Approach
                        }
                        fd_core::fplan::ProcedureKind::Approach => ProcedurePhase::Unknown,
                        fd_core::fplan::ProcedureKind::Sid => ProcedurePhase::Sid,
                        fd_core::fplan::ProcedureKind::Star => ProcedurePhase::Star,
                    };
                    (Some(ctx), phase)
                }
                None => (None, ProcedurePhase::Enroute),
            }
        }
        None => (None, ProcedurePhase::Unknown),
    };
    PlanCorrelation {
        route,
        matches,
        procedure,
        procedure_phase,
        destination_airport: dest_for_proc,
    }
}

/// FMS bridge lifecycle + change detection (Task 7 §11-12).
struct FmsWatcher {
    port: u16,
    client: Option<FmsBridgeClient>,
    last_attempt: Option<std::time::Instant>,
    latest: Option<FmsSnapshot>,
    /// (revision_hash, destination_entry) of the last ingested primary.
    last_primary: Option<(u64, Option<usize>)>,
    /// Distinct plan revisions observed (change events + first observe).
    revisions: u64,
    /// Bounded classified-change log (§12; capped, overflow counted).
    changes: Vec<String>,
    changes_truncated: u64,
    /// Correlation quality counters (§14).
    matched: u64,
    ambiguous: u64,
    not_found: u64,
    /// Typed classification of the LAST ingestion (§12/§37).
    last_changes: Vec<FlightPlanChange>,
    /// Last observed approach-plan presence (§12: approach loaded/cleared
    /// is a meaningful revision).
    last_approach_loaded: Option<bool>,
}

impl FmsWatcher {
    fn new(port: u16) -> Self {
        Self {
            port,
            client: None,
            last_attempt: None,
            latest: None,
            last_primary: None,
            revisions: 0,
            changes: Vec::new(),
            changes_truncated: 0,
            matched: 0,
            ambiguous: 0,
            not_found: 0,
            last_changes: Vec::new(),
            last_approach_loaded: None,
        }
    }

    fn enabled(&self) -> bool {
        self.port != 0
    }

    /// Poll the bridge: reconnect with backoff, read one snapshot when
    /// available. Returns the NEWEST snapshot when it changed.
    fn poll(&mut self) -> Option<FmsSnapshot> {
        if !self.enabled() {
            return None;
        }
        if self.client.is_none() {
            // Reconnect at most every 5 s (§8: reconnectable, patient).
            let ready = self
                .last_attempt
                .map(|t| t.elapsed() >= std::time::Duration::from_secs(5))
                .unwrap_or(true);
            if ready {
                self.last_attempt = Some(std::time::Instant::now());
                match FmsBridgeClient::connect(self.port) {
                    Ok(c) => {
                        println!(
                            "FMS_BRIDGE: connected (xplane={:?} xplm={:?})",
                            c.hello.xplane, c.hello.xplm
                        );
                        self.client = Some(c);
                    }
                    Err(_) => return None,
                }
            }
            return None;
        }
        match self.client.as_mut().unwrap().poll() {
            Ok(Some(snapshot)) => self.ingest(snapshot),
            Ok(None) => None,
            Err(BridgeError::Io(_) | BridgeError::Connect(_)) => {
                // Dead connection: drop and retry later. Honest
                // degradation — no plan is fabricated from absence (§13).
                self.client = None;
                None
            }
            Err(e) => {
                // Protocol errors are real defects: report once and drop
                // the connection.
                println!("FMS_BRIDGE: protocol error: {e}");
                self.client = None;
                None
            }
        }
    }

    /// Ingest a snapshot: change classification (§12) + counters.
    fn ingest(&mut self, snapshot: FmsSnapshot) -> Option<FmsSnapshot> {
        if let Some(prev) = &self.latest
            && prev.revision_hash == snapshot.revision_hash
        {
            return None; // no meaningful change (§12: no per-tick spam)
        }
        let primary_key = snapshot
            .primary()
            .map(|p| (p.revision_hash(), p.destination_entry));
        if let (Some((prev_hash, prev_dest)), Some(primary)) =
            (self.last_primary, snapshot.primary())
        {
            // Reconstruct the previous primary for classification.
            let prev_plan = self.latest.as_ref().and_then(|s| s.primary());
            if let Some(prev_plan) = prev_plan {
                if prev_hash == prev_plan.revision_hash() {
                    let changes = classify_primary_change(Some((prev_plan, prev_dest)), primary);
                    for c in changes {
                        self.push_change(format!("{c:?}"));
                    }
                } else {
                    self.push_change("PlanReplaced".into());
                }
            }
        } else if snapshot
            .primary()
            .map(|p| !p.entries.is_empty())
            .unwrap_or(false)
        {
            self.push_change("FirstObserved".into());
        }
        // Typed classification of THIS transition (§12).
        self.last_changes = match (
            self.latest.as_ref().and_then(|s| s.primary()),
            snapshot.primary(),
        ) {
            (Some(prev_plan), Some(next_plan)) => {
                let prev_dest = self.last_primary.and_then(|(_, d)| d);
                if prev_plan.revision_hash() == primary_key.map(|(h, _)| h).unwrap_or(0) {
                    classify_primary_change(Some((prev_plan, prev_dest)), next_plan)
                } else {
                    vec![FlightPlanChange::PlanReplaced]
                }
            }
            (None, Some(_)) => vec![FlightPlanChange::PlanReplaced],
            _ => vec![],
        };
        self.last_primary = primary_key;
        self.revisions += 1;
        self.latest = Some(snapshot.clone());
        Some(snapshot)
    }

    fn push_change(&mut self, change: String) {
        if self.changes.len() >= 1000 {
            self.changes_truncated += 1;
        } else {
            self.changes.push(change);
        }
    }

    /// Count correlation outcomes (§14 counters).
    fn record_matches(&mut self, matches: &[NavMatch]) {
        for m in matches {
            match m {
                NavMatch::Matched { .. } => self.matched += 1,
                NavMatch::Ambiguous { .. } => self.ambiguous += 1,
                NavMatch::NotFound { .. } => self.not_found += 1,
            }
        }
    }

    /// Aircraft swap / sim restart (§39-40): all plan state invalidated.
    fn reset(&mut self, reason: &str) {
        println!("FMS_BRIDGE: state reset ({reason})");
        self.latest = None;
        self.last_primary = None;
        self.revisions = 0;
        self.changes.clear();
        self.changes_truncated = 0;
        self.last_changes.clear();
        self.last_approach_loaded = None;
        // The connection itself survives an aircraft reload (session-
        // scoped ids are dataref-level, the bridge is independent); a
        // simulator restart kills it — the reconnect path handles both.
    }
}

/// Run one zero-write observation session.
pub fn run_observe(opts: ObserveOpts) -> anyhow::Result<()> {
    println!("FLIGHTDECKOS LIVE FLIGHT OBSERVATORY V2 (zero-write, FMS-aware)");
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

    // Initial route: FMS if the bridge is up, else OpenAIRAC 2-point.
    let mut fms = FmsWatcher::new(opts.fms_bridge_port);
    let mut route_state = nav
        .as_ref()
        .map(|n| n.route_state_openairac())
        .unwrap_or(RouteState {
            source: RouteSource::Unknown,
            waypoints: vec![],
        });
    let mut runway_ctx: Option<RunwayContext> = None;
    let mut plan_correlation: Option<PlanCorrelation> = None;

    // Try an immediate FMS snapshot so the session starts plan-aware.
    if let Some(snap) = fms.poll()
        && let Some(nav) = &nav
    {
        {
            let corr = correlate_snapshot(nav, &snap);
            fms.record_matches(&corr.matches);
            if corr.route.is_usable() {
                route_state = corr.route.clone();
            }
            if let Some(ap) = &corr.destination_airport {
                runway_ctx = nav.runway_for(ap).unwrap_or(None);
            }
            plan_correlation = Some(corr);
        }
    }
    let mut route_monitor = RouteMonitor::new(&route_state);
    let mut off_route = OffRouteDetector::new(OffRouteConfig::default());
    let mut route_usable = route_state.is_usable();

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
            // Hoisted: the controller signature requires a follower, but
            // the observe path never consumes its guidance.
            fd_mission::RouteFollower::new(route_state.waypoints.clone(), 2.5),
        )
    });

    // Analytics. BOUNDED state only (§45): samples stream to the FDR.
    let mut fdr_seq = fd_fdm::fdr::Recorder::new();
    let mut writer =
        StreamedRecorder::create(&opts.fdr_out, &live_meta(&adapter, sim_version, &opts))?;
    let mut session = SessionTracker::new();
    let mut phase_engine = FlightPhaseEngine::new();
    let mut fdm = FdmAnalyzer::new_development_default();
    let mut qoa = ApproachAnalyzer::new(Default::default(), None);
    let mut summarizer = SessionSummarizer::new();
    let mut fdr_events = 0u64;
    let mut route_observations = 0u64;
    let mut off_route_events = 0u64;
    let mut route_complete = false;
    let mut last_identity_icao = adapter.identity().icao.clone();

    let started = std::time::Instant::now();
    let mut last_status = started;
    let mut plan_observed_event_done = false;
    let mut last_fdr_sample: Option<fd_fdm::fdr::FdrSample> = None;
    let mut last_route_obs: Option<fd_mission::monitor::RouteObservation> = None;
    loop {
        std::thread::sleep(std::time::Duration::from_millis(250));
        let snap = match adapter.poll() {
            Ok(v) => v.into_iter().next(),
            Err(_) => continue,
        };
        let Some(s) = snap else { continue };

        // Aircraft swap invalidation (§39).
        let identity_icao = adapter.identity().icao.clone();
        if identity_icao != last_identity_icao {
            println!(
                "AIRCRAFT_SWAP: {:?} -> {:?}; plan/route/package state invalidated",
                last_identity_icao, identity_icao
            );
            fms.reset("aircraft swap");
            plan_correlation = None;
            route_state = nav
                .as_ref()
                .map(|n| n.route_state_openairac())
                .unwrap_or(RouteState {
                    source: RouteSource::Unknown,
                    waypoints: vec![],
                });
            route_monitor = RouteMonitor::new(&route_state);
            off_route = OffRouteDetector::new(OffRouteConfig::default());
            runway_ctx = None;
            if let Some((_, _, _, follower)) = shadow.as_mut() {
                *follower = fd_mission::RouteFollower::new(route_state.waypoints.clone(), 2.5);
            }
            last_identity_icao = identity_icao;
        }

        // FMS observation (§11-12): poll + classify on change.
        if let Some(new_snap) = fms.poll()
            && let Some(nav) = &nav
        {
            {
                let corr = correlate_snapshot(nav, &new_snap);
                fms.record_matches(&corr.matches);
                // Route rebuild only when the route actually changed.
                if corr.route != route_state {
                    route_state = corr.route.clone();
                    // Recompute: a route that becomes usable only after
                    // startup (late FMS bridge delivery) must enable
                    // monitoring/shadow (Task 7.1 review HIGH).
                    route_usable = route_state.is_usable();
                    off_route = OffRouteDetector::new(OffRouteConfig::default());
                    if let Some((_, _, _, follower)) = shadow.as_mut() {
                        *follower =
                            fd_mission::RouteFollower::new(route_state.waypoints.clone(), 2.5);
                    }
                }
                // Destination runway context (§27) — event on change.
                if corr.destination_airport
                    != plan_correlation
                        .as_ref()
                        .and_then(|c| c.destination_airport.clone())
                {
                    let new_rw = corr
                        .destination_airport
                        .as_ref()
                        .and_then(|ap| nav.runway_for(ap).unwrap_or(None));
                    fdr_events += 1;
                    writer
                        .record_event(&plan_event(
                            fdr_events,
                            last_fdr_sample
                                .as_ref()
                                .map(|f| f.timestamp)
                                .unwrap_or(fd_core::telemetry::SimTimestamp { ms: 0 }),
                            &FdrEventPayload::RunwayContextChanged {
                                airport: corr.destination_airport.clone().unwrap_or_default(),
                                runway_end: new_rw
                                    .as_ref()
                                    .map(|r| format!("{}/{}", r.runway.le_ident, r.runway.he_ident))
                                    .unwrap_or_default(),
                                evidence: new_rw
                                    .as_ref()
                                    .map(|r| r.evidence.clone())
                                    .unwrap_or_else(|| "no runway context".into()),
                            },
                        ))
                        .map_err(|e| anyhow::anyhow!("{e}"))?;
                    runway_ctx = new_rw;
                }
                // Procedure context (§15-16) — event on change.
                if corr.procedure != plan_correlation.as_ref().and_then(|c| c.procedure.clone()) {
                    fdr_events += 1;
                    writer
                        .record_event(&plan_event(
                            fdr_events,
                            last_fdr_sample
                                .as_ref()
                                .map(|f| f.timestamp)
                                .unwrap_or(fd_core::telemetry::SimTimestamp { ms: 0 }),
                            &FdrEventPayload::ProcedureContextChanged {
                                context: corr.procedure.clone(),
                            },
                        ))
                        .map_err(|e| anyhow::anyhow!("{e}"))?;
                }
                // Plan-change event (§37) with the typed classification.
                fdr_events += 1;
                writer
                    .record_event(&FdrEvent {
                        seq: fdr_events,
                        timestamp: last_fdr_sample
                            .as_ref()
                            .map(|f| f.timestamp)
                            .unwrap_or(fd_core::telemetry::SimTimestamp { ms: 0 }),
                        kind: "flight_plan".into(),
                        detail: format!("{:?}", fms.last_changes),
                        payload: Some(FdrEventPayload::FlightPlanChanged {
                            changes: {
                                let mut changes = fms.last_changes.clone();
                                // Approach presence flips are part of the same
                                // revision classification (§12).
                                for c in &fms.changes {
                                    if c == "ApproachLoaded"
                                        && !changes
                                            .iter()
                                            .any(|c| matches!(c, FlightPlanChange::ApproachLoaded))
                                    {
                                        changes.push(FlightPlanChange::ApproachLoaded);
                                    }
                                    if c == "ApproachCleared"
                                        && !changes
                                            .iter()
                                            .any(|c| matches!(c, FlightPlanChange::ApproachCleared))
                                    {
                                        changes.push(FlightPlanChange::ApproachCleared);
                                    }
                                }
                                changes
                            },
                            revision_hash: new_snap.revision_hash,
                            primary_entries: new_snap
                                .primary()
                                .map(|p| p.entries.len())
                                .unwrap_or(0),
                            destination_entry: new_snap.primary().and_then(|p| p.destination_entry),
                        }),
                    })
                    .map_err(|e| anyhow::anyhow!("{e}"))?;
                plan_correlation = Some(corr);
            }
        }

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
        if !plan_observed_event_done && fms.latest.is_some() {
            plan_observed_event_done = true;
            if let (Some(corr), Some(snap)) = (&plan_correlation, fms.latest.as_ref()) {
                fdr_events += 1;
                writer
                    .record_event(&plan_event(
                        fdr_events,
                        sample.timestamp,
                        &FdrEventPayload::FlightPlanObserved {
                            device: format!("{:?}", snap.device),
                            revision_hash: snap.revision_hash,
                            primary_entries: snap.primary().map(|p| p.entries.len()).unwrap_or(0),
                            approach_entries: snap.approach_loaded().map(|b| b as usize),
                            destination_entry: snap.primary().and_then(|p| p.destination_entry),
                            destination_id: corr.destination_airport.clone(),
                        },
                    ))
                    .map_err(|e| anyhow::anyhow!("{e}"))?;
            }
        }
        for ev in fdm.process(&sample) {
            summarizer.record_fdm_event();
            fdr_events += 1;
            writer
                .record_event(&FdrEvent {
                    seq: fdr_events,
                    timestamp: sample.timestamp,
                    kind: "fdm".into(),
                    detail: format!("{:?} measured={:.0}", ev.kind, ev.measured),
                    payload: None,
                })
                .map_err(|e| anyhow::anyhow!("{e}"))?;
        }
        qoa.push(sample.clone());

        // Route monitoring (deterministic, zero-write).
        if route_usable && let Some(pos) = &s.position {
            route_observations += 1;
            let obs = route_monitor.update(pos.lat.value(), pos.lon.value());
            last_route_obs = Some(obs.clone());
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
        if let (Some((controller, shadow_rec, null, follower)), Some(pos), true) =
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
        summarizer.push_sample(&sample);
        last_fdr_sample = Some(sample.clone());

        // Concise status line every 5 s (§47).
        if last_status.elapsed().as_secs() >= 5 {
            last_status = std::time::Instant::now();
            let plan_txt = match (&fms.latest, &plan_correlation) {
                (Some(snap), _) => {
                    let entries = snap.primary().map(|p| p.entries.len()).unwrap_or(0);
                    let dest = plan_correlation
                        .as_ref()
                        .and_then(|c| c.destination_airport.clone())
                        .unwrap_or_else(|| "?".into());
                    let leg = last_route_obs
                        .as_ref()
                        .and_then(|o| o.active_leg)
                        .map(|i| format!("{}", i + 1))
                        .unwrap_or_else(|| "?".into());
                    let xtk = last_route_obs
                        .as_ref()
                        .and_then(|o| o.cross_track_error_nm)
                        .map(|x| format!("{x:+.1}nm"))
                        .unwrap_or_else(|| "?".into());
                    format!("{entries}wp dest={dest} leg={leg} xtk={xtk}")
                }
                (None, _) => "no fms".into(),
            };
            let intents: usize = shadow
                .as_ref()
                .map(|(_, rec, _, _)| rec.summary().intents_emitted.values().sum())
                .unwrap_or(0);
            println!(
                "STATUS: phase={} plan=[{}] fdr={} fdm={} shadow=armed({intents})",
                assessment.phase.as_str(),
                plan_txt,
                summarizer.sample_count(),
                summarizer.fdm_events(),
            );
        }

        if opts.monitor_secs > 0 && started.elapsed().as_secs() >= opts.monitor_secs {
            break;
        }
    }
    writer.finish().map_err(|e| anyhow::anyhow!("{e}"))?;
    println!(
        "FDR_SESSION: samples={} events={} -> {} (v2 jsonl)",
        summarizer.sample_count(),
        fdr_events,
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
    if fms.enabled() {
        println!(
            "FMS_SESSION: revisions={} changes={:?}+{} truncated matches={} ambiguous={} not_found={}",
            fms.revisions,
            fms.changes,
            fms.changes_truncated,
            fms.matched,
            fms.ambiguous,
            fms.not_found
        );
    }

    // Bounded summary + landing window (§45).
    let (summary, landing_window) = summarizer.finish();

    // Debrief (§48) with the plan summary.
    if let Some(debrief_path) = &opts.debrief_out {
        let plan_summary = fms.enabled().then(|| fd_debrief::FmsPlanSummary {
            observed: fms.latest.is_some(),
            device: fms.latest.as_ref().map(|s| format!("{:?}", s.device)),
            primary_entries: fms
                .latest
                .as_ref()
                .and_then(|s| s.primary())
                .map(|p| p.entries.len()),
            destination_id: plan_correlation
                .as_ref()
                .and_then(|c| c.destination_airport.clone()),
            approach_loaded: fms.latest.as_ref().and_then(|s| s.approach_loaded()),
            revisions_observed: fms.revisions,
            changes: fms.changes.clone(),
            procedure: plan_correlation.as_ref().and_then(|c| c.procedure.clone()),
            navigation_phase: plan_correlation
                .as_ref()
                .map(|c| format!("{:?}", c.procedure_phase)),
            nav_matches: fms.matched,
            nav_ambiguous: fms.ambiguous,
            nav_not_found: fms.not_found,
            openairac_provenance: nav.as_ref().map(|n| format!("world@{}", n.at)),
        });
        let debrief = fd_debrief::build_debrief(fd_debrief::BuildDebriefArgs {
            identity: adapter.identity().clone(),
            session: &session,
            sample_count: summary.sample_count,
            origin: opts.origin_icao.as_deref(),
            destination: opts.destination_icao.as_deref(),
            route_source_str: route_usable.then(|| format!("{:?}", route_state.source)),
            waypoint_count: route_state.waypoints.len(),
            route_usable,
            off_route_events,
            route_complete,
            summary: &summary,
            landing_window: &landing_window,
            plan: plan_summary,
            fdm_events: fdr_events,
            approach: &qoa.finish(),
            runway: runway_ctx.as_ref(),
            shadow_summary: shadow.as_ref().map(|(_, rec, _, _)| rec.summary()),
        })?;
        let json = debrief.to_json_pretty()?;
        std::fs::write(debrief_path, json).context("write debrief")?;
        println!("DEBRIEF: {}", debrief_path.display());
    }

    // Crew view (§33-34): bounded, read-only, serializable.
    if let Some(crew_path) = &opts.crew_view_out {
        let mut view = fd_crew::view::CrewView::new();
        view.aircraft = fd_crew::view::aircraft_summary(adapter.identity(), None);
        view.phase = last_fdr_sample.as_ref().map(|f| f.flight_phase.clone());
        if let Some(corr) = &plan_correlation {
            view.navigation = fd_crew::view::NavigationSummary {
                origin: opts.origin_icao.clone(),
                destination: corr.destination_airport.clone(),
                procedure_phase: Some(format!("{:?}", corr.procedure_phase)),
                procedure: corr.procedure.clone(),
            };
        }
        if let Some(snap) = &fms.latest {
            let primary = snap.primary();
            view.flight_plan = fd_crew::view::FlightPlanSummaryLine {
                observed: true,
                device: Some(format!("{:?}", snap.device)),
                entry_count: primary.map(|p| p.entries.len()),
                active_waypoint: primary
                    .and_then(|p| p.destination())
                    .and_then(|e| e.id.clone()),
                destination_waypoint: primary
                    .and_then(|p| p.entries.last())
                    .and_then(|e| e.id.clone()),
                approach_loaded: snap.approach_loaded(),
                revision: fms.revisions,
            };
        }
        view.route = fd_crew::view::RouteStatusLine {
            source: route_usable.then(|| format!("{:?}", route_state.source)),
            waypoint_count: route_usable.then_some(route_state.waypoints.len()),
            active_leg: last_route_obs.as_ref().and_then(|o| o.active_leg),
            next_waypoint: last_route_obs
                .as_ref()
                .and_then(|o| o.active_leg)
                .and_then(|i| route_state.waypoints.get(i + 1))
                .map(|w| w.id.clone()),
            distance_to_destination_nm: last_route_obs
                .as_ref()
                .and_then(|o| o.destination_distance_nm),
            cross_track_nm: last_route_obs.as_ref().and_then(|o| o.cross_track_error_nm),
        };
        if let Some(f) = &last_fdr_sample {
            view.systems = fd_crew::view::SystemsSummary {
                on_ground: f.on_ground,
                gear_down: f.gear_down,
                any_engine_running: f.any_engine_running,
                autopilot_master: f.autopilot_master,
                altitude_msl_ft: f.altitude_msl,
                indicated_airspeed_kt: f.indicated_airspeed,
                vertical_speed_fpm: f.vertical_speed,
            };
        }
        view.capabilities.push((
            "fms.plan".into(),
            if fms.latest.is_some() {
                "Available"
            } else {
                "Unavailable"
            }
            .into(),
        ));
        view.data_quality = fd_crew::view::DataQualityLine {
            samples: summary.sample_count,
            non_fresh_channels: summary
                .channel_quality
                .iter()
                .map(|(ch, c)| (*ch, c.annotated.values().sum()))
                .collect(),
            max_gap_ms: summary.gaps.max_gap_ms,
            gaps_over_threshold: summary.gaps.gaps_over_threshold,
        };
        std::fs::write(crew_path, view.to_json_pretty()?).context("write crew view")?;
        println!("CREW_VIEW: {}", crew_path.display());
    }
    Ok(())
}

/// Plan observation/runway/procedure event stamped with the CURRENT
/// sample's sim time (causal ordering with the sample stream, §32/§36).
fn plan_event(
    seq: u64,
    ts: fd_core::telemetry::SimTimestamp,
    payload: &FdrEventPayload,
) -> FdrEvent {
    FdrEvent {
        seq,
        timestamp: ts,
        kind: "flight_plan".into(),
        detail: format!("{payload:?}"),
        payload: Some(payload.clone()),
    }
}

// Small accessors so the shadow block reads cleanly without cloning the
// controller each tick.
fn controller_phase(c: &MissionController) -> fd_mission::MissionPhase {
    c.phase()
}

fn controller_params(c: &MissionController) -> &MissionParameters {
    c.params()
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
