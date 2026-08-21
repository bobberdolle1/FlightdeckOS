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
