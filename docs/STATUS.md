# FlightdeckOS — Current Status

Publication date: 2026-08-25 · HEAD: `242fe27` · 295 tests passing ·
Primary live target: X-Plane 12.4.3

This document is the evidence-backed capability snapshot. Labels:

- **LIVE VERIFIED** — exercised against a real simulator session with
  recorded evidence.
- **SHORT-LIVE VERIFIED** — real session, bounded duration (not a complete
  flight).
- **OFFLINE VERIFIED** — deterministic tests against the real code path,
  no simulator.
- **HEADLESS VERIFIED** — full pipeline through VirtualSimulator.
- **EXPERIMENTAL / PARTIAL / PLANNED** — see wording.

## LIVE VERIFIED

- X-Plane 12.4.3 native UDP telemetry: normalized snapshots at ~3–4 Hz,
  per-channel freshness, disconnect/reconnect handling.
- X-Plane Local Web API (drogon, v1/v2/v3): resource discovery
  (session-scoped ids), command activation with the v2 singular
  `command/{id}/activate` route + `duration` body (a real API mismatch was
  found and fixed by live evidence).
- Aircraft identity with provenance (`UserProvided` claims are recorded,
  never trusted).
- Safe Control V1 — first closed cockpit action end-to-end:
  `SetBeacon` → `LiveWriteGuard` → capability/precondition gate → Web API
  command → fresh post-dispatch telemetry → observed post-condition →
  `Verified` → restoration through the same path. Guard negative test
  (no `--allow-write` → rejection, simulator untouched) verified live.
- Transport health: bounded web operations, cooldown after failure.

## SHORT-LIVE VERIFIED

- Live flight observatory (`fd observe`): one zero-write session against a
  real X-Plane flight — identity, OpenAIRAC airport context (EDDF from the
  local world store), 256 live telemetry samples recorded to FDR V2,
  flight-session lifecycle, route monitor active, warm-up freshness
  annotations visible in the recording.
- Live FDR recording (short session), replayed offline through the
  production loader + analytics into a structured debrief.

## OFFLINE VERIFIED

- FDR V2 container: versioned JSONL streaming, torn-tail recovery, legacy
  V1 compatibility, corruption fail-closed.
- Replay determinism: byte-identical traces across runs; recorded flights
  replay through production analytics with identical semantic results.
- FDM: exceedance episode state machines (sink rate, bank at low altitude,
  hard touchdown), Started/Ended aggregation, typed thresholds.
- Quality of Approach: evidence-based stabilization gates
  (Stable/Unstable/Indeterminate — unknown never counts as stable),
  go-around detection with sustained-climb requirement.
- Quality of Landing: touchdown evidence with debounce, signed impact VS,
  runway-relative metrics only with resolved geometry.
- Route monitor: active leg, signed cross-track error, remaining distance,
  off-route detection (development thresholds).
- Runway awareness: centerline offset, threshold distance, remaining
  runway, heading difference (planar local frame, sign-tested).
- OpenAIRAC world-store reader: strictly read-only SQLite access, temporal
  queries, airport/runway/waypoint records.
- Mission Shadow: four-way channel classification
  (Match/Divergence/Unknown/NotComparable), HighLevelIntent derivation from
  the same tick output the controller acts on, zero-write by construction.
- Generic Aircraft mode: no package → telemetry, phase, FDR, FDM, monitoring
  still work; nothing aircraft-specific is invented.
- Aircraft packages: A32NX reference package, fail-closed validation,
  capability catalogues with provenance.
- SOP primitives: flows, triggers, step lifecycle (offline).
- Debrief: structured, versioned, serializable.

## HEADLESS VERIFIED

- Headless Flight Lab: VirtualSimulator (semantic + bounded-rate
  kinematic model), deterministic virtual clock, scenario engine with TOML
  specs, fault injection (freeze, stale values, sensor masking, action
  ignoring, disconnect, ground bounce), UUEE→ULLI reference mission,
  negative scenarios with specific expected triggers.

## EXPERIMENTAL

- MSFS SimConnect adapter (`fd-simconnect`): implemented against the
  SimConnect API, developed and tested offline only. No live MSFS
  validation has ever been performed on the development environment.
- `fd-crew` OpenAIRAC Gateway client: schema-negotiated client for the
  external OpenAIRAC 3.2 Gateway; exercised only against test doubles.
- Profile Genesis: concept + package foundation only; observation tooling
  not yet implemented.

## PLANNED (not implemented)

- Full end-to-end live flight observation.
- Autonomous flight of any kind.
- AI Crew / LLM integration in the runtime (structured-intent boundary is
  designed; no model is wired in).
- Voice interaction, Passenger Mode, career/dispatch/ATC systems.
- FMS state extraction, SID/STAR/APPR procedure automation.

## Known limitations

- **Full live-flight observation is pending.** The observatory
  infrastructure and a short live session are verified; the local X-Plane
  instance became unstable (recurring embedded Web API wedges, one crash,
  boot stalls) during the validation campaign, so a complete flight under
  observation has not been recorded yet.
- **X-Plane Web API stability**: the embedded HTTP server intermittently
  stopped responding after idle periods on the development machine
  (X-Plane 12.4.3). FlightdeckOS treats this as an environmental fact: all
  web operations are bounded and a wedged server surfaces as
  `Unavailable`, never as a hang. This is an observation about one
  environment, not a claim about X-Plane in general.
- **OpenAIRAC RU enroute data** in the current world bundle has dangling
  airway legs; full Russian airway routing is not relied upon. Airport and
  runway context are used instead.
- **FDM thresholds are development defaults** — not airline operational
  limits, not certification criteria, not real FOQA policy.
- **Runway selection** in the observatory is a development default (first
  runway with complete geometry), not wind/ATC-informed.
- **License**: no LICENSE file exists yet. The project is All rights
  reserved until one is defined. This must be resolved before soliciting
  external contributions.
