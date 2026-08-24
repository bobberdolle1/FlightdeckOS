//! FlightdeckOS application host.
//!
//! Commands:
//! * `fd replay` — deterministic offline replay of a fixture (no simulator).
//! * `fd live`  — live MSFS via SimConnect (Windows only).
//! * `fd bindings` — print the A32NX binding table with provenance.
//!
//! `anyhow` is used here at the application boundary only.

use std::path::PathBuf;
use std::time::Duration;

use clap::{Parser, Subcommand};
use fd_aircraft::catalog::a32nx_default_catalog;
use fd_core::actions::Actor;
use fd_core::events::EventSource;
use fd_runtime::{DeadlineTicks, ReplayAdapter, ReplayStep, Runtime, SessionId, TraceWriter};
use fd_sop::package::load_package;

#[derive(Parser)]
#[command(name = "fd", version, about = "FlightdeckOS runtime host")]
struct Cli {
    #[command(subcommand)]
    command: Command,
}

#[derive(Subcommand)]
enum Command {
    /// Deterministic offline replay of a fixture file.
    Replay {
        /// Fixture path (JSONL of snapshots/actions).
        #[arg(long, short)]
        fixture: PathBuf,
        /// Output trace path (default: ./traces/replay.jsonl).
        #[arg(long, short)]
        out: Option<PathBuf>,
        /// Injected session id for determinism (default 0).
        #[arg(long, default_value_t = 0)]
        session_id: u64,
        /// Optional aircraft package directory; starts the named flow.
        #[arg(long)]
        package: Option<PathBuf>,
        /// Flow id to start (requires --package).
        #[arg(long)]
        flow: Option<String>,
    },
    /// Validate an aircraft package directory (fail-closed).
    Package {
        #[arg(long)]
        dir: PathBuf,
    },
    /// Run a deterministic headless scenario (virtual simulator).
    Scenario {
        /// Scenario TOML path.
        #[arg(long)]
        run: PathBuf,
    },
    /// Print the capability report for an optional package.
    Capabilities {
        /// Optional package directory; omit for generic mode.
        #[arg(long)]
        package: Option<PathBuf>,
    },
    /// Live X-Plane 12 session via the native UDP transport.
    Xplane {
        /// X-Plane UDP command port (default 49000).
        #[arg(long, default_value_t = 49000)]
        port: u16,
        /// Seconds to monitor telemetry (0 = until Ctrl+C).
        #[arg(long, default_value_t = 30)]
        monitor_secs: u64,
        /// Closed-loop heading smoke: target TRUE heading via stock AP.
        #[arg(long)]
        set_heading_true: Option<f64>,
        /// Closed-loop vertical-speed smoke: target VS in fpm.
        #[arg(long)]
        set_vs_fpm: Option<f64>,
        /// Seconds to wait for the FIRST telemetry packet (XP boot window).
        #[arg(long, default_value_t = 240)]
        wait_first_secs: u64,
        /// Operator-claimed aircraft ICAO (stock UDP cannot read identity
        /// strings; recorded with UserProvided provenance, never trusted).
        #[arg(long)]
        aircraft_icao: Option<String>,
        /// ARM live writes for this process (spec §14: default DISABLED).
        #[arg(long)]
        allow_write: bool,
        /// Dispatch the first safe action: beacon on|off (requires
        /// --allow-write). Verified only by fresh observed state.
        #[arg(long)]
        beacon: Option<String>,
        /// Record a live FDR session (normalized telemetry + session
        /// metadata) to this JSON file (spec §20).
        #[arg(long)]
        fdr_out: Option<PathBuf>,
    },
    /// Live MSFS session via SimConnect (Windows only).
    Live {
        /// Output trace path (default: ./traces/live.jsonl).
        #[arg(long, short)]
        out: Option<PathBuf>,
        /// Max ticks (0 = unlimited until Ctrl+C).
        #[arg(long, default_value_t = 0)]
        max_ticks: u64,
        /// Poll interval between ticks, ms (default 50).
        #[arg(long, default_value_t = 50)]
        interval_ms: u64,
    },
    /// Print the A32NX action binding table with provenance.
    Bindings,
    /// Connect to OpenAIRAC Gateway and interact with AI Crew Runtime.
    Crew {
        /// OpenAIRAC Gateway URL (default: http://127.0.0.1:8989/api/openairac/v1).
        #[arg(long, default_value = "http://127.0.0.1:8989/api/openairac/v1")]
        url: String,
        /// Ask a natural language question to the AI Crew.
        #[arg(long, short)]
        ask: Option<String>,
        /// Display structured crew flight status panel.
        #[arg(long)]
        status: bool,
    },
    /// Query OpenAIRAC Gateway directly.
    Openairac {
        /// OpenAIRAC Gateway URL.
        #[arg(long, default_value = "http://127.0.0.1:8989/api/openairac/v1")]
        url: String,
        /// Fetch recent flight events.
        #[arg(long)]
        events: bool,
        /// Resolve airport multi-identity (e.g. URAS, URFF).
        #[arg(long)]
        identity: Option<String>,
    },
}

fn main() -> anyhow::Result<()> {
    let cli = Cli::parse();
    match cli.command {
        Command::Xplane {
            port,
            monitor_secs,
            set_heading_true,
            set_vs_fpm,
            wait_first_secs,
            aircraft_icao,
            allow_write,
            beacon,
            fdr_out,
        } => run_xplane_live(XplaneSmokeOpts {
            port,
            monitor_secs,
            set_heading_true,
            set_vs_fpm,
            wait_first_secs,
            aircraft_icao,
            allow_write,
            beacon,
            fdr_out,
        }),
        Command::Replay {
            fixture,
            out,
            session_id,
            package,
            flow,
        } => {
            if package.is_some() != flow.is_some() {
                return Err(anyhow::anyhow!(
                    "--package and --flow must be used together"
                ));
            }
            run_replay(&fixture, out, session_id, package, flow)
        }
        Command::Live {
            out,
            max_ticks,
            interval_ms,
        } => run_live(out, max_ticks, interval_ms),
        Command::Package { dir } => run_package_validate(&dir),
        Command::Scenario { run } => {
            let report = fd_scenario::run_scenario(&run).map_err(|e| anyhow::anyhow!("{e}"))?;
            println!("HEADLESS VIRTUAL TEST");
            println!("NOT LIVE SIMULATOR VALIDATION");
            println!("NOT REAL AIRCRAFT PERFORMANCE VALIDATION");
            println!(
                "scenario: {}  route: {} -> {}",
                report.scenario_id, report.origin_id, report.destination_id
            );
            println!(
                "simulated: {:.1}s  wall: {:.2}s  ticks: {}  fdr samples: {}",
                report.simulated_seconds, report.wall_seconds, report.sim_ticks, report.fdr_samples
            );
            println!("final phase: {}", report.final_phase);
            println!(
                "autonomy: requested={} verified={} failed={} timed_out={}",
                report.autonomy.actions_requested,
                report.autonomy.actions_verified,
                report.autonomy.actions_failed,
                report.autonomy.actions_timed_out
            );
            println!(
                "procedures: completed={} failed={}",
                report.autonomy.procedure_steps_completed, report.autonomy.procedure_steps_failed
            );
            if !report.fdm_events.is_empty() {
                println!("fdm events:");
                for e in &report.fdm_events {
                    println!("  {}: {}", e.kind, e.count);
                }
            } else {
                println!("fdm events: none");
            }
            fn fmt_gate(v: &Option<bool>) -> String {
                match v {
                    Some(true) => "true".into(),
                    Some(false) => "false (unstable)".into(),
                    None => "not_assessed".into(),
                }
            }
            println!(
                "approach: stabilized@1000={} @500={} max sink={:?} fpm",
                fmt_gate(&report.approach.stabilized_at_1000ft),
                fmt_gate(&report.approach.stabilized_at_500ft),
                report.approach.max_sink_rate_fpm
            );
            println!(
                "landing: touchdown={} vs={:?}",
                report.landing.touchdown_occurred, report.landing.touchdown_vertical_speed_fpm
            );
            match &report.result {
                fd_scenario::ScenarioResult::Passed => println!("RESULT: PASSED"),
                fd_scenario::ScenarioResult::Failed { reason } => {
                    println!("RESULT: FAILED — {reason}")
                }
            }
            Ok(())
        }
        Command::Capabilities { package } => {
            let has_pkg = package.is_some();
            if let Some(dir) = &package {
                let pkg = load_package(dir).map_err(|e| anyhow::anyhow!("{e}"))?;
                println!(
                    "package: {} ({})",
                    pkg.manifest.package_id, pkg.manifest.display_name
                );
            }
            // Compose a capability report for the current configuration.
            let mut r = fd_core::capability::CapabilityReport::new();
            use fd_core::capability::{CapabilityStatus as S, EvidenceSource as E};
            for cap in [
                "telemetry.position",
                "telemetry.airspeed",
                "telemetry.attitude",
                "telemetry.gear",
                "fdr.recording",
                "fdm.analysis",
                "qoa.approach",
                "autonomy.route_guidance",
            ] {
                r.set_with_evidence(cap, S::Supported, E::VirtualSim);
            }
            for cap in ["systems.electrical", "systems.pneumatic", "action.lights"] {
                r.set(cap, if has_pkg { S::Supported } else { S::Unknown });
            }
            r.set(
                "action.apu",
                if has_pkg {
                    S::Supported
                } else {
                    S::Unsupported
                },
            );
            r.set(
                "procedure.before_start",
                if has_pkg {
                    S::Supported
                } else {
                    S::Unavailable
                },
            );
            r.set(
                "procedure.any",
                if has_pkg {
                    S::Supported
                } else {
                    S::Unavailable
                },
            );
            r.set_with_evidence("autonomy.flight", S::Supported, E::VirtualSim);
            r.set("autonomy.ground", S::Unavailable);
            for e in r.entries_sorted() {
                println!(
                    "{:<32} {:<12} {}",
                    e.path,
                    e.status.as_str(),
                    e.evidence.as_str()
                );
            }
            Ok(())
        }
        Command::Bindings => {
            print_bindings();
            Ok(())
        }
        Command::Crew { url, ask, status } => run_crew(&url, ask.as_deref(), status),
        Command::Openairac {
            url,
            events,
            identity,
        } => run_openairac(&url, events, identity.as_deref()),
    }
}

fn run_package_validate(dir: &std::path::Path) -> anyhow::Result<()> {
    let pkg = load_package(dir).map_err(|e| anyhow::anyhow!("package INVALID: {e}"))?;
    println!(
        "package: {} ({})",
        pkg.manifest.package_id, pkg.manifest.display_name
    );
    println!(
        "family/addon/sim: {} / {} / {}",
        pkg.manifest.aircraft_family, pkg.manifest.addon, pkg.manifest.simulator
    );
    println!(
        "schema_version: {} · runtime_api_version: {}",
        pkg.manifest.schema_version, pkg.manifest.runtime_api_version
    );
    println!("live_verified: {}", pkg.manifest.live_verified);
    println!("roles: {:?}", pkg.roles);
    println!("flows: {}", pkg.flows.len());
    for f in &pkg.flows {
        println!("  - {} \"{}\" ({} steps)", f.id, f.title, f.steps.len());
        for st in &f.steps {
            let kind = match &st.kind {
                fd_sop::package::StepKind::Observe { .. } => "observe",
                fd_sop::package::StepKind::Action { .. } => "action",
            };
            println!(
                "      {:<18} actor={:<14} kind={}",
                st.id,
                format!("{:?}", st.actor).to_lowercase(),
                kind
            );
        }
    }
    println!("PACKAGE VALID");
    Ok(())
}

fn default_out(name: &str) -> PathBuf {
    let dir = std::path::Path::new("traces");
    std::fs::create_dir_all(dir).ok();
    dir.join(name)
}

fn run_replay(
    fixture: &PathBuf,
    out: Option<PathBuf>,
    session_id: u64,
    package: Option<PathBuf>,
    flow: Option<String>,
) -> anyhow::Result<()> {
    let steps = fd_runtime::replay::load_fixture(fixture)
        .map_err(|e| anyhow::anyhow!("fixture load failed: {e}"))?;
    let out = out.unwrap_or_else(|| default_out("replay.jsonl"));

    // Snapshots are pre-loaded into the adapter (one delivered per tick);
    // action steps are injected between ticks, exactly like a live stream.
    let adapter = ReplayAdapter::new(steps.clone());
    let trace = TraceWriter::create(&out)?;
    let mut runtime = Runtime::new(
        Box::new(adapter),
        trace,
        SessionId(session_id),
        a32nx_default_catalog(),
        DeadlineTicks::default(),
    );
    runtime
        .start()
        .map_err(|e| anyhow::anyhow!("replay start failed: {e}"))?;

    if let (Some(pkg_dir), Some(flow_id)) = (&package, &flow) {
        let pkg = load_package(pkg_dir).map_err(|e| anyhow::anyhow!("package load failed: {e}"))?;
        let def = pkg
            .flows
            .iter()
            .find(|f| &f.id == flow_id)
            .ok_or_else(|| anyhow::anyhow!("flow `{flow_id}` not found in package"))?
            .clone();
        runtime.start_flow(def)?;
    }

    let mut ticks = 0u64;
    for step in steps {
        match step {
            ReplayStep::Snapshot(_) => {
                // The adapter delivers the next scripted snapshot on poll.
                runtime.tick(EventSource::Replay)?;
                ticks += 1;
            }
            ReplayStep::Action { ts, action } => {
                runtime
                    .submit_action(action, Actor::User, ts)
                    .map_err(|e| anyhow::anyhow!("action injection failed: {e}"))?;
            }
        }
    }
    let final_phase = runtime.current_phase();
    let flow_status = runtime.flow_status();
    runtime.finish()?;
    if let Some(st) = flow_status {
        println!("Flow {}: {:?}", flow.as_deref().unwrap_or("?"), st);
    }
    println!(
        "REPLAY OK: {} ticks, phase {}, trace written to {}",
        ticks,
        final_phase.as_str(),
        out.display()
    );
    Ok(())
}

#[cfg(windows)]
fn run_live(out: Option<PathBuf>, max_ticks: u64, interval_ms: u64) -> anyhow::Result<()> {
    use fd_simconnect::SimConnectAdapter;

    let out = out.unwrap_or_else(|| default_out("live.jsonl"));
    let adapter = SimConnectAdapter::new();
    let trace = TraceWriter::create(&out)?;
    let mut runtime = Runtime::new(
        Box::new(adapter),
        trace,
        SessionId(1),
        a32nx_default_catalog(),
        DeadlineTicks::default(),
    );

    match runtime.start() {
        Ok(()) => {}
        Err(e) => {
            println!("LIVE VALIDATION PENDING: {e}");
            std::process::exit(2);
        }
    }

    let mut ticks = 0u64;
    loop {
        match runtime.tick(EventSource::Simulator) {
            Ok(_) => {}
            Err(fd_runtime::RuntimeError::Adapter(
                fd_core::adapter::AdapterError::NotConnected,
            )) => {
                println!("simulator disconnected after {ticks} ticks");
                break;
            }
            Err(e) => {
                runtime.finish().ok();
                return Err(anyhow::anyhow!("live tick failed: {e}"));
            }
        }
        ticks += 1;
        if max_ticks > 0 && ticks >= max_ticks {
            println!("reached max ticks ({ticks}); stopping");
            break;
        }
        std::thread::sleep(Duration::from_millis(interval_ms));
    }
    runtime.finish()?;
    println!("LIVE OK: {ticks} ticks, trace written to {}", out.display());
    Ok(())
}

#[cfg(not(windows))]
fn run_live(_out: Option<PathBuf>, _max_ticks: u64, _interval_ms: u64) -> anyhow::Result<()> {
    println!("LIVE VALIDATION PENDING: SimConnect adapter requires Windows");
    std::process::exit(2);
}

fn print_bindings() {
    #[cfg(windows)]
    {
        use fd_simconnect::bindings;
        println!("Action binding table (A32NX / MSFS):");
        println!(
            "{:<24} {:<34} {:<24} provenance",
            "logical", "raw", "verifies"
        );
        for action in bindings::ALL_ACTIONS {
            if let Some(e) = bindings::lookup_write(*action) {
                let raw = match e.primitive {
                    bindings::WritePrimitive::SimVarWrite { name, unit, value } => {
                        format!("simvar {name}[{unit}]={value}")
                    }
                    bindings::WritePrimitive::Event { name, param } => {
                        format!("event {name} param={param}")
                    }
                };
                println!(
                    "{:<24} {:<34} {:<24} {}",
                    e.logical, raw, e.verifies, e.provenance
                );
            }
        }
        println!();
        println!("A32NX read bindings (telemetry):");
        for b in bindings::A32NX_READ_BINDINGS {
            println!(
                "  {:<30} {:<38} {}",
                b.canonical,
                format!("{}[{}]", b.raw, b.unit),
                b.doc
            );
        }
        println!();
        println!("tested addon version: {}", bindings::TESTED_ADDON_VERSION);
    }
    #[cfg(not(windows))]
    {
        println!(
            "binding table is available on Windows builds only (fd-simconnect is Windows-gated)"
        );
    }
}

fn run_crew(url: &str, ask: Option<&str>, show_status: bool) -> anyhow::Result<()> {
    use fd_crew::AiCrewRuntime;
    use fd_openairac::OpenAiracClient;

    println!("================================================================================");
    println!("FlightdeckOS v0.2 — AI Crew Runtime & OpenAIRAC Integration");
    println!("================================================================================");
    println!("Connecting to OpenAIRAC Gateway at {}...", url);

    let client = OpenAiracClient::new(url);
    let mut crew = AiCrewRuntime::default();

    match client.get_snapshot() {
        Ok(snap) => {
            println!(
                "[+] Connected to OpenAIRAC Gateway successfully (Schema: {})",
                snap.schema_version
            );
            crew.update_from_snapshot(&snap);

            if let Some(ctx) = crew.context() {
                if show_status || ask.is_none() {
                    println!(
                        "\n┌── CREW FLIGHT STATUS PANEL ───────────────────────────────────────────────────┐"
                    );
                    println!(
                        "│ Flight: {:<12} Phase: {:<14} Aircraft: {:<25}│",
                        ctx.flight_id, ctx.flight_phase, ctx.aircraft_type
                    );
                    println!(
                        "│ Alt: {:<6.0} ft MSL  GS: {:<5.0} kt   Track: {:<5.0}°   XTK: {:<4.2} NM {:<12}│",
                        ctx.altitude_ft,
                        ctx.groundspeed_kts,
                        ctx.track_deg,
                        ctx.xtk_nm,
                        ctx.xtk_side
                    );
                    println!("│ Active Leg: {:<64}│", ctx.active_leg);
                    println!(
                        "│ Next Fix: {:<14} Distance: {:<5.1} NM   Next Constraint: {:<16}│",
                        ctx.next_fix, ctx.distance_to_next_nm, ctx.next_constraint
                    );
                    println!(
                        "│ TOD Distance: {:<10} Profile Status: {:<37}│",
                        ctx.tod_distance_nm
                            .map(|d| format!("{:.1} NM", d))
                            .unwrap_or_else(|| "N/A".to_string()),
                        ctx.descent_profile_status
                    );
                    println!("│ Weather: {:<67}│", ctx.destination_weather);
                    println!(
                        "│ Freshness: Telem={:<7} Wx={:<7} Online={:<7} Nav={:<14}│",
                        ctx.freshness.telemetry_status,
                        ctx.freshness.weather_status,
                        ctx.freshness.online_atc_status,
                        ctx.freshness.navdata_status
                    );
                    println!(
                        "└───────────────────────────────────────────────────────────────────────────────┘"
                    );
                }

                if let Some(q) = ask {
                    println!("\n[User]: {}", q);
                    let resp = crew.ask(q)?;
                    println!("[AI Crew]: {}", resp.message);
                    if let Some(qual) = resp.freshness_qualification {
                        println!("  ⚠️ Freshness Notice: {}", qual);
                    }
                    println!(
                        "  🔍 Tool Evidence: {} factual source traces",
                        resp.tool_evidence.len()
                    );
                }
            }
        }
        Err(e) => {
            println!("[-] OpenAIRAC Gateway connection notice: {e}");
            println!(
                "[*] FlightdeckOS failure isolation active: SOP and aircraft runtime remain operational."
            );
        }
    }

    Ok(())
}

fn run_openairac(url: &str, events: bool, identity: Option<&str>) -> anyhow::Result<()> {
    use fd_openairac::OpenAiracClient;
    let client = OpenAiracClient::new(url);

    if events {
        let evts = client.get_events(None)?;
        println!("Received {} OpenAIRAC flight events:", evts.len());
        for e in evts {
            println!("  [{:>4}] {} — {}", e.id, e.event_type, e.description);
        }
    } else if let Some(ident) = identity {
        let id = client.resolve_identity(ident)?;
        println!("Airport Multi-Identity Resolution for {}:", ident);
        println!("  Authoritative Ident: {}", id.authoritative_ident);
        println!("  IATA Code:           {:?}", id.iata_code);
        println!("  Airport Name:        {}", id.airport_name);
        println!("  Primary Provider:    {}", id.primary_provider);
        println!("  Procedures Status:   {}", id.terminal_procedures_status);
    } else {
        let snap = client.get_compact_snapshot()?;
        println!("OpenAIRAC Compact AI Snapshot:");
        println!("  Flight:   {}", snap.flight);
        println!("  Phase:    {}", snap.phase);
        println!("  Leg:      {}", snap.active_leg);
        println!("  Next:     {}", snap.next_fix);
        println!("  TOD:      {}", snap.tod);
        println!("  Arrival:  {}", snap.arrival);
        println!("  Fresh:    {:?}", snap.freshness);
    }

    Ok(())
}

/// Live X-Plane 12 smoke: telemetry monitor + optional closed-loop control.
///
/// LIVE means LIVE: values shown come only from UDP packets received from a
/// running X-Plane. When packets stop, the loop prints
/// SIMULATOR_DISCONNECTED and keeps FlightdeckOS alive, retrying the
/// subscription (Task 4 §7/§13/§16). No virtual fallback exists on this
/// path.
const BEACON_ON_CMD: &str = fd_xplane::BEACON_ON_COMMAND;
const BEACON_OFF_CMD: &str = fd_xplane::BEACON_OFF_COMMAND;

/// Options for the live X-Plane smoke (grouped to keep the signature sane).
struct XplaneSmokeOpts {
    port: u16,
    monitor_secs: u64,
    set_heading_true: Option<f64>,
    set_vs_fpm: Option<f64>,
    wait_first_secs: u64,
    aircraft_icao: Option<String>,
    allow_write: bool,
    beacon: Option<String>,
    fdr_out: Option<PathBuf>,
}

fn run_xplane_live(opts: XplaneSmokeOpts) -> anyhow::Result<()> {
    let XplaneSmokeOpts {
        port,
        monitor_secs,
        set_heading_true,
        set_vs_fpm,
        wait_first_secs,
        aircraft_icao,
        allow_write,
        beacon,
        fdr_out,
    } = opts;
    use fd_core::adapter::{FlightControlTargets, SimulatorAdapter};
    use fd_core::capability::{CapabilityReport, CapabilityStatus, EvidenceSource};
    use fd_core::identity::AircraftIdentity;
    use fd_xplane::{XPlaneAdapter, XPlaneConfig};

    // Spec §14: writes stay disabled unless explicitly armed THIS process.
    if beacon.is_some() && !allow_write {
        return Err(anyhow::anyhow!(
            "--beacon requires --allow-write (live writes are disabled by default)"
        ));
    }

    println!("XPLANE LIVE SMOKE (native UDP transport)");
    let cfg = XPlaneConfig {
        host: "127.0.0.1".into(),
        port,
        subscribe_hz: 4,
        allow_writes: allow_write,
        ..XPlaneConfig::default()
    };
    let identity = match aircraft_icao {
        Some(icao) => AircraftIdentity::user_provided(Some(icao)),
        None => AircraftIdentity::unknown(),
    };
    let mut adapter = XPlaneAdapter::with_identity(cfg, identity)
        .map_err(|e| anyhow::anyhow!("adapter init failed: {e}"))?;

    print!("waiting for first telemetry packet from 127.0.0.1:{port} .. ");
    if !adapter.wait_first_packet(std::time::Duration::from_secs(wait_first_secs)) {
        return Err(anyhow::anyhow!(
            "no telemetry from X-Plane within {wait_first_secs} s — is X-Plane 12 running with a flight loaded?"
        ));
    }
    adapter
        .connect()
        .map_err(|e| anyhow::anyhow!("connect failed: {e}"))?;
    let sim_version = adapter.simulator_version();
    println!(
        "SIMULATOR: X-Plane {}",
        sim_version.as_deref().unwrap_or("(web api unavailable)")
    );
    println!(
        "WRITE_GUARD: {}",
        if adapter.write_guard().is_armed() {
            "ARMED (live writes enabled)"
        } else {
            "DISABLED (live writes inhibited)"
        }
    );
    println!("CONNECTED");
    println!(
        "TRANSPORT_CONNECTED: udp rx {} pkts so far",
        adapter.packets_received()
    );
    let id = adapter.identity();
    println!(
        "AIRCRAFT_IDENTITY: icao={} source={} trusted={}",
        id.icao.as_deref().unwrap_or("(unknown)"),
        match id.source {
            fd_core::identity::IdentitySource::Unknown => "unknown",
            fd_core::identity::IdentitySource::UserProvided => "user_provided",
            fd_core::identity::IdentitySource::Adapter => "adapter",
        },
        id.is_trusted()
    );
    let mut caps = CapabilityReport::new();
    caps.set_with_evidence(
        "telemetry.position",
        CapabilityStatus::Supported,
        EvidenceSource::LiveXplane,
    );
    caps.set_with_evidence(
        "telemetry.attitude",
        CapabilityStatus::Supported,
        EvidenceSource::LiveXplane,
    );
    caps.set_with_evidence(
        "fdr.recording",
        CapabilityStatus::Supported,
        EvidenceSource::LiveXplane,
    );
    caps.set_with_evidence(
        "action.discrete",
        CapabilityStatus::Unsupported,
        EvidenceSource::LiveXplane,
    );
    for e in caps.entries_sorted() {
        println!(
            "CAPABILITY: {:<24} {:<12} {}",
            e.path,
            e.status.as_str(),
            e.evidence.as_str()
        );
    }

    // ---- First safe action (spec §15): beacon through the full runtime
    // action pipeline — guard -> catalog -> precondition -> dispatch ->
    // fresh observed post-condition -> Verified. HTTP/command success is
    // NOT Verified; only fresh telemetry is evidence.
    if let Some(beacon_arg) = beacon.as_deref() {
        let target = match beacon_arg {
            "on" => fd_core::actions::SwitchPosition::On,
            "off" => fd_core::actions::SwitchPosition::Off,
            other => return Err(anyhow::anyhow!("--beacon expects on|off, got {other}")),
        };
        let initial = adapter.beacon_state();
        println!(
            "ACTION_PRECONDITION: beacon current={:?} target={:?} command_on={BEACON_ON_CMD} command_off={BEACON_OFF_CMD}",
            initial, target
        );
        let trace_path = std::path::PathBuf::from("traces/live_beacon_action.jsonl");
        let _ = std::fs::create_dir_all("traces");
        let trace = fd_runtime::TraceWriter::create(&trace_path)
            .map_err(|e| anyhow::anyhow!("action trace create failed: {e}"))?;
        let mut rt = fd_runtime::Runtime::new(
            Box::new(adapter),
            trace,
            fd_runtime::SessionId(1),
            a32nx_default_catalog(),
            fd_runtime::DeadlineTicks(20),
        );
        rt.start()
            .map_err(|e| anyhow::anyhow!("runtime start failed: {e}"))?;
        rt.submit_action(
            CockpitAction::SetBeacon(target),
            fd_core::actions::Actor::User,
            fd_core::telemetry::SimTimestamp::new(0),
        )
        .map_err(|e| anyhow::anyhow!("submit failed: {e}"))?;

        let mut verified = false;
        let mut failed = String::new();
        use fd_core::actions::{ActionStatus, CockpitAction};
        for tick in 0..30 {
            match rt.tick(EventSource::Simulator) {
                Ok(_) => {}
                Err(e) => {
                    failed = format!("runtime tick failed: {e}");
                    break;
                }
            }
            let completed = rt.take_completed_actions();
            for (id, st) in &completed {
                println!("[action t={tick}] id={id:?} status={st:?}");
                match st {
                    ActionStatus::Verified => verified = true,
                    other => failed = format!("action terminal: {other:?}"),
                }
            }
            if verified || !failed.is_empty() {
                break;
            }
            std::thread::sleep(std::time::Duration::from_millis(300));
        }
        if verified {
            println!(
                "ACTION_RESULT: VERIFIED (beacon {:?} confirmed by fresh telemetry)",
                target
            );
        } else {
            println!("ACTION_RESULT: NOT VERIFIED — {failed}");
        }
        rt.finish().ok();
        println!("ACTION_TRACE: {}", trace_path.display());
        return Ok(());
    }

    // Live FDR session (spec §20): normalized telemetry + real phase
    // engine + FDM analysis, session metadata on exit.
    let mut fdr = fdr_out
        .map(|path| {
            let started_wall = std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .ok()
                .map(|d| d.as_millis() as u64);
            let meta = fd_fdm::fdr::FdrSessionMeta {
                session_id: format!("live-{}", started_wall.unwrap_or(0)),
                simulator: "X-Plane 12".into(),
                sim_version: sim_version.clone(),
                aircraft: adapter.identity().clone(),
                fdos_version: env!("CARGO_PKG_VERSION").into(),
                adapter_source: Some("xplane-udp".into()),
                started_wall_unix_ms: started_wall,
                ended_wall_unix_ms: None,
                origin: None,
                destination: None,
                started_ms: 0,
                ended_ms: None,
            };
            // V2 streamed JSONL (Task 6 §13): crash-safe, torn-tail
            // recoverable; flushes every 32 samples.
            fd_fdm::fdr::StreamedRecorder::create(&path, &meta)
                .map(|w| (w, path))
                .map_err(|e| anyhow::anyhow!("{e}"))
        })
        .transpose()?;
    let mut phase_engine = fd_core::phase::FlightPhaseEngine::new();
    let mut fdm_live = fd_fdm::fdm::FdmAnalyzer::new_development_default();
    let mut fdr_seq = fd_fdm::fdr::Recorder::new();
    let mut session = fd_fdm::session::SessionTracker::new();
    let mut fdr_events = 0u64;

    let mut last_heading_cmd: Option<(f64, std::time::Instant)> = None;
    let mut last_vs_cmd: Option<(f64, std::time::Instant)> = None;
    let mut was_connected = true;
    let started = std::time::Instant::now();
    let mut last_pkts = 0u64;
    let mut last_hz_t = started;

    loop {
        std::thread::sleep(std::time::Duration::from_millis(500));

        // Typed disconnect handling: stay alive, expose state, resubscribe.
        if !adapter.is_connected() {
            if was_connected {
                println!("SIMULATOR_DISCONNECTED (telemetry stale > 3 s)");
                was_connected = false;
            }
            let _ = adapter.connect(); // bounded retry: resubscribe + wait 3 s
            continue;
        }
        if !was_connected {
            println!("SIMULATOR_RECONNECTED — telemetry resumed");
            was_connected = true;
        }

        // Closed-loop control smokes fire once telemetry is trusted.
        if let Some(target) = set_heading_true.filter(|_| last_heading_cmd.is_none()) {
            let me = &mut adapter as &mut dyn FlightControlTargets;
            me.set_target_heading(target);
            last_heading_cmd = Some((target, std::time::Instant::now()));
            println!("CONTROL_REQUEST: target heading TRUE {target:.1}");
        }
        if let Some(vs) = set_vs_fpm.filter(|_| last_vs_cmd.is_none()) {
            let me = &mut adapter as &mut dyn FlightControlTargets;
            me.set_target_vertical_speed(vs);
            last_vs_cmd = Some((vs, std::time::Instant::now()));
            println!("CONTROL_REQUEST: target VS {vs:.0} fpm");
        }

        let snap = match adapter.poll() {
            Ok(v) => v.into_iter().next(),
            Err(_) => continue,
        };
        if let Some(s) = snap {
            let pos = s.position.as_ref();
            let hz = {
                let dt = last_hz_t.elapsed().as_secs_f64().max(1e-6);
                let hz = (adapter.packets_received() - last_pkts) as f64 / dt;
                last_pkts = adapter.packets_received();
                last_hz_t = std::time::Instant::now();
                hz
            };
            if let Some((w, _)) = fdr.as_mut() {
                let assessment = phase_engine.evaluate(&fd_core::phase::PhaseTelemetry::from(&s));
                let sample = fdr_seq.record(&s, assessment.phase.as_str());
                for ev in fdm_live.process(&sample) {
                    fdr_events += 1;
                    w.record_event(&fd_fdm::fdr::FdrEvent {
                        seq: fdr_events,
                        timestamp: sample.timestamp,
                        kind: "fdm".into(),
                        detail: format!("{:?} measured={:.0}", ev.kind, ev.measured),
                    })
                    .map_err(|e| anyhow::anyhow!("{e}"))?;
                }
                w.record_sample(&sample)
                    .map_err(|e| anyhow::anyhow!("{e}"))?;
                session.advance(fd_fdm::session::SessionEvidence {
                    connected: true,
                    identity_known: adapter.identity().icao.is_some(),
                    sample_recorded: true,
                    altitude_agl_ft: s.altitude_agl.map(|v| v.value()),
                    on_ground: s.on_ground,
                    groundspeed_kt: s.groundspeed.map(|v| v.value()),
                    descending: s.vertical_speed.map(|v| v.value() < -100.0),
                });
            }
            println!(
                "[t={:>5.1}s {:>4.0}Hz] lat={:<10} lon={:<10} msl={:>7.0}ft agl={:>7.0}ft ias={:>6.1} gs={:>6.1} vs={:>7.1} hdg={:>6.1} gnd={} ap={} pkt_age={:?}",
                started.elapsed().as_secs_f64(),
                hz,
                pos.map(|p| format!("{:.5}", p.lat.value()))
                    .unwrap_or("-".into()),
                pos.map(|p| format!("{:.5}", p.lon.value()))
                    .unwrap_or("-".into()),
                s.altitude_msl.map(|v| v.value()).unwrap_or(0.0),
                s.altitude_agl.map(|v| v.value()).unwrap_or(0.0),
                s.indicated_airspeed.map(|v| v.value()).unwrap_or(0.0),
                s.groundspeed.map(|v| v.value()).unwrap_or(0.0),
                s.vertical_speed.map(|v| v.value()).unwrap_or(0.0),
                s.heading_true.map(|v| v.value()).unwrap_or(0.0),
                s.on_ground.map(|b| b.to_string()).unwrap_or("?".into()),
                s.autopilot_master.unwrap_or(false),
                adapter.newest_packet_age(),
            );

            // Closed-loop reporting: commanded vs observed convergence.
            if let Some((target, t0)) = last_heading_cmd {
                let obs = s.heading_true.map(|v| v.value()).unwrap_or(0.0);
                let err = (target - obs + 180.0).rem_euclid(360.0) - 180.0;
                let settled = err.abs() < 2.0 && t0.elapsed() >= std::time::Duration::from_secs(3);
                let timed_out = t0.elapsed() >= std::time::Duration::from_secs(45);
                if settled || timed_out {
                    println!(
                        "CLOSED_LOOP_HEADING_{}: target={target:.1} observed={obs:.1} elapsed={:?} steady_err={err:.2}deg",
                        if settled { "OK" } else { "TIMEOUT" },
                        t0.elapsed()
                    );
                    last_heading_cmd = None;
                }
            }
        }

        if monitor_secs > 0 && started.elapsed().as_secs() >= monitor_secs {
            break;
        }
    }
    println!(
        "SMOKE_END: total_packets={} disconnects={}",
        adapter.packets_received(),
        adapter.disconnect_count()
    );
    if let Some((mut w, path)) = fdr {
        w.finish().map_err(|e| anyhow::anyhow!("{e}"))?;
        println!(
            "FDR_SESSION: samples={} events={} -> {} (v2 jsonl)",
            w.samples_written(),
            w.events_written(),
            path.display()
        );
        println!("SESSION_LIFECYCLE: {:?}", session.state());
    }
    Ok(())
}
