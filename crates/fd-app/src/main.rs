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
use fd_core::actions::Actor;
use fd_core::events::EventSource;
use fd_runtime::{
    DeadlineTicks, ReplayAdapter, ReplayStep, Runtime, SessionId, TraceWriter,
    a32nx_default_catalog,
};

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
}

fn main() -> anyhow::Result<()> {
    let cli = Cli::parse();
    match cli.command {
        Command::Replay {
            fixture,
            out,
            session_id,
        } => run_replay(&fixture, out, session_id),
        Command::Live {
            out,
            max_ticks,
            interval_ms,
        } => run_live(out, max_ticks, interval_ms),
        Command::Bindings => {
            print_bindings();
            Ok(())
        }
    }
}

fn default_out(name: &str) -> PathBuf {
    let dir = std::path::Path::new("traces");
    std::fs::create_dir_all(dir).ok();
    dir.join(name)
}

fn run_replay(fixture: &PathBuf, out: Option<PathBuf>, session_id: u64) -> anyhow::Result<()> {
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
    runtime.finish()?;
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
            Err(fd_core::adapter::AdapterError::NotConnected) => {
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
