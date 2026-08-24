# FlightdeckOS Architecture

FlightdeckOS normalizes different simulators and aircraft into one
deterministic runtime. This document describes the current architecture and
its dependency direction. Status labels follow [STATUS.md](STATUS.md).

## Dependency direction

```text
fd-app (CLI host)
  └─ fd-runtime ── fd-core (canonical types; depends on nothing internal)
       │              ↑
       ├─ fd-xplane ──┤ (adapters implement fd-core traits)
       ├─ fd-simconnect┤
       ├─ fd-virtual ──┤
       ├─ fd-aircraft (packages, catalogues)      fd-sop (procedures)
       ├─ fd-fdm (FDR/FDM/QoA/QoL)   fd-mission (mission, route, shadow)
       ├─ fd-scenario (headless engine)   fd-openairac (nav data)
       └─ fd-debrief (structured reports)
```

`fd-core` is the vocabulary: every other crate speaks its types. Adapters
point inward (they implement `fd-core::adapter::SimulatorAdapter`);
analytics consume normalized snapshots and never talk to simulators.

## Core runtime

- **Normalized aircraft state** (`fd-core::telemetry`): one
  `TelemetrySnapshot` — position, altitudes, speeds, attitude, gear, flaps,
  engines, AP, lights, simulator timing — with per-channel
  `DataQuality` (`Fresh`/`WarmingUp`/`Stale`/`Missing`/`Invalid`).
  Unknown stays unknown: optional fields, never fabricated zeros.
- **Flight phase engine** (`fd-core::phase`): deterministic state machine
  over the snapshot; arrival/approach gating consumes route/runway
  distances when provided and falls back conservatively when not.
- **Runtime tick** (`fd-runtime`): poll → ingest → phase → action pipeline
  → SOP flow pass → trace append, with strictly monotonic event sequence
  and poison-on-trace-failure. Byte-deterministic across runs.

## Simulator adapters

All simulators implement one trait surface:

| Adapter | Transport | Status |
| --- | --- | --- |
| `fd-xplane` | UDP RREF telemetry + Local Web API (loopback REST) | live verified |
| `fd-simconnect` | SimConnect | experimental, never live-validated |
| `fd-virtual` | in-process deterministic model | headless verified |
| `ReplayAdapter` | recorded fixture steps | offline verified |

X-Plane specifics: per-channel warm-up (a channel becomes `Fresh` only
after 3 consecutive finite samples), transport health (UDP and Web API
independently: Available/Degraded/Unavailable), bounded web operations
with cooldown, aircraft hot-swap invalidation. Details:
[XPLANE.md](XPLANE.md).

## Capability evidence and identity

`AircraftIdentity` carries provenance (`Unknown`/`UserProvided`/`Adapter`);
user claims are recorded but never trusted. `Capability` entries state what
an action/transport can do **with evidence**; the executor refuses actions
without capability support. Aircraft packages (see
[AIRCRAFT_PROFILES.md](AIRCRAFT_PROFILES.md)) provide catalogues; an
unknown aircraft runs in Generic mode with generic capabilities only.

## Typed actions and safe control

There is no arbitrary "write variable" API in upper layers. The entire
discrete action surface is the closed `CockpitAction` enum. Execution:

```text
CockpitAction request
  → catalogue lookup (unknown kind → reject)
  → capability check (adapter must advertise support, with evidence)
  → preconditions (fail-closed; unknown state → reject)
  → LiveWriteGuard (live writes disabled unless explicitly armed)
  → adapter dispatch
  → verification: observed post-condition from a snapshot STRICTLY NEWER
    than dispatch, with all verification channels Fresh
  → Verified / Failed / Rejected / TimedOut (traced)
```

The full invariant list lives in [SAFETY_MODEL.md](SAFETY_MODEL.md).

## SOP and mission

`fd-sop` provides procedure primitives (flows, triggers, step lifecycle)
driven by the runtime. `fd-mission` provides the mission controller — a
pure decision function (`intended_tick`) whose outputs both drive the
headless mission and feed the shadow; phase transitions, guidance targets.

## Mission Shadow

`fd-mission::shadow` records what autonomy *would* command versus what the
aircraft actually did, per channel, four-way classified
(Match/Divergence/Unknown/NotComparable), plus `HighLevelIntent` records
with deterministic reason tokens. The shadow has **no** adapter or action
surface — zero-write by construction.

## Flight data

- **Trace** (`fd-runtime::trace`): what FlightdeckOS *did* — actions,
  validations, phase changes, procedure steps. JSONL, torn-tail tolerant.
- **FDR** (`fd-fdm::fdr`): what the *aircraft* did — versioned streaming
  JSONL container (header/meta/samples/events), crash-safe flushes,
  torn-tail recovery, V1 compatibility. Session lifecycle
  (`FlightSessionState`) is evidence-driven, not UI-scripted.
- **Analytics**: FDM exceedance episodes; Quality of Approach
  stabilization gates with evidence and an Indeterminate class that can
  never become Stable; Quality of Landing with debounced touchdown
  evidence and runway-relative metrics only when geometry is resolved.
- **Debrief** (`fd-debrief`): one structured, versioned document
  aggregating identity, session, route, phase timeline, FDM, approach,
  landing, shadow and data-quality summaries.

Details: [FLIGHT_DATA.md](FLIGHT_DATA.md).

## Navigation context (OpenAIRAC)

`fd-openairac` reads the OpenAIRAC world store (SQLite) **strictly
read-only** with temporal queries (validity windows) and a deterministic
query-instant pin for reproducible reference data. It provides airport
identity/elevation, runway geometry (threshold coordinates, headings,
lengths) and waypoints. Airway routing is deliberately not built on the
current dataset (RU enroute legs are incomplete). A Gateway HTTP client
exists for the external OpenAIRAC Map/AI-Crew service.

## Headless simulation

`fd-virtual` implements the same adapter traits with a deterministic
virtual clock, semantic systems and bounded-rate kinematics, plus
declarative fault injection (telemetry freeze, stale values, sensor
masking, action ignoring, disconnect windows, ground bounce). `fd-scenario`
drives complete reference missions (UUEE→ULLI) from TOML specs and produces
machine-checkable reports. See [HEADLESS_FLIGHT_LAB.md](HEADLESS_FLIGHT_LAB.md).

## Future AI boundary

The designed boundary for any LLM/AI component:

```text
LLM
  → structured intent (HighLevelIntent-style data)
  → validated deterministic tools
  → CockpitAction (closed catalogue)
  → capability + precondition gates
  → simulator adapter
```

Never: LLM → raw dataref/SimVar writes. Today no LLM is wired into the
runtime; the shadow layer already speaks in intents so future AI decisions
can be compared against observed behavior without any control authority.
