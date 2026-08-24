# Flight Data: FDR, FDM, QoA, Debrief

## Trace vs FDR — the separation

- **Trace** (`fd-runtime`): what *FlightdeckOS did* — action requests,
  validations, dispatches, verifications, phase changes, procedure steps.
  Technical audit log.
- **FDR** (`fd-fdm`): what the *aircraft did* — the deterministic canonical
  state stream, plus attached flight events. Analysis input.

They are different files, different formats, different crates, and the
separation is deliberate.

## FDR V2 container

Versioned, streamable, recoverable, replayable JSONL:

```text
{"fdr_format":"fdos-fdr","version":2}
{"meta":{...session metadata...}}
{"sample":{...one normalized sample...}}
{"event":{...attached flight event...}}
{"session_end":true}
```

- Streamed: flush every 32 samples — a crash or Ctrl+C preserves flushed
  data.
- Recovery: a torn final line is dropped with a warning; interior
  corruption is a hard error; unknown versions fail closed.
- V1 compatibility: legacy single-document recordings still load.
- Samples carry position, sim rate, slew flag and per-channel quality
  annotations (V1 samples load with these defaulted).

## Flight session lifecycle

`AwaitingSimulator → Connected → AircraftDetected → Recording → Airborne →
Landing → Parked → SessionClosed` — transitions are pure functions of
telemetry evidence with named development constants (airborne floor,
parked speed, sustain counts). Unknown data never advances the lifecycle
past what the evidence supports.

## DataQuality

Every channel is `Fresh`, `WarmingUp`, `Stale`, `Missing` or `Invalid`.

- `WarmingUp`: received this session but fewer than 3 consecutive finite
  samples — never authoritative, never action-verification evidence.
- `Stale`: present but outside the freshness window.
- Quality annotations survive the FDR round trip unchanged.

## FDM (Flight Data Monitoring)

Streaming exceedance detectors with episode lifecycle (`Started`/`Ended`):
excessive sink rate at low altitude, excessive bank at low altitude, hard
touchdown. Fail-closed: unknown inputs close episodes rather than guess.

**Development defaults.** FDM thresholds are NOT airline operational
limits, NOT certification criteria, NOT real-world FOQA policy. They exist
to make the analytics pipeline testable.

## Quality of Approach (QoA)

Stabilization gates (development heights, default 1000/500 ft) produce
**evidence**, not a bool: per-gate IAS/VS/bank/gear/flaps status with
sample references, classified `Stable`/`Unstable`/`Indeterminate`.
Indeterminate (any required criterion unknown) can never become Stable.
Go-around detection requires descent arrest plus sustained positive climb
— not a single noisy sample.

## Quality of Landing (QoL)

Touchdown = airborne evidence (≥3 samples) + approach/descent context +
on-ground transition, with debounce of sub-2-sample ground flicker. The
touchdown record captures signed impact VS, speeds, attitude, heading and
position when available. Runway-relative metrics (centerline offset,
distance beyond threshold, remaining runway) are produced **only** when a
runway context with real geometry is supplied — otherwise `None`.

## Autonomy observability

Raw counters only (mission transitions, intents emitted, shadow
matches/divergences/unknown/not-comparable, capability-missing events) —
no synthetic 0–100 "autonomy score".

## Debrief

`fd-debrief` aggregates one flight into a versioned structured document:
identity, session summary, route outcome, phase timeline, FDM summary,
approach report, landing report, shadow summary, data-quality summary.
Deterministic: same inputs → same document. Offline tool:
`cargo run -p fd-debrief --example debrief_from_fdr -- <fdr.jsonl> [out.json]`.
