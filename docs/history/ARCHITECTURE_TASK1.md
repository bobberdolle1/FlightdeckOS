# FlightdeckOS — Task 1 Architecture (SimConnect Runtime Foundation)

Status: implemented (offline-complete). Live MSFS validation: **PENDING** (no
simulator on the development machine — see §8).

## 1. Scope of this slice

```text
MSFS → SimConnect → typed canonical state → state delta/event
     → validated cockpit action → SimConnect write
     → observed post-condition → append-only trace
```

Explicitly out of scope (and absent from the codebase): LLM/agents, SOP
engine, flows/checklists, crew model, FMS/autopilot control, X-Plane, UI.

## 2. Crates & dependency direction

```text
fd-app ──► fd-runtime ──► fd-core
   │                          ▲
   └──► fd-simconnect ────────┘
```

* **fd-core** — canonical types and pure logic: typed units, telemetry
  snapshot, state deltas, monotonic events, the flight phase engine, the
  closed action catalog types, and the `SimulatorAdapter` *contract*.
  `#![forbid(unsafe_code)]`. Knows nothing about SimConnect, L:Vars, MSFS
  events, or addon internals.
* **fd-simconnect** — the only crate that knows SimConnect: own FFI layer,
  datum table, A32NX binding table, raw writes (`pub(crate)` only).
  Windows-gated; non-Windows builds get a stub adapter.
* **fd-runtime** — deterministic loop: session sequencing, snapshot
  ingestion, phase tracking (pause-aware), action pipeline, JSONL trace.
  Consumes `dyn SimulatorAdapter`; never references fd-simconnect.
* **fd-app** — wiring + CLI (`replay` / `live` / `bindings`). `anyhow` only
  here.

Deviation from the Task 1 sketch: the adapter trait lives in `fd-core`, not
`fd-runtime`. The implementation crate must not depend on the runtime crate
(cycle); the task's hard rule — *fd-core must not know simulator specifics* —
is preserved: the trait references only canonical types.

## 3. State model

* `TelemetrySnapshot` — one canonical sample. Generic core fields
  (position, altitudes, speeds, VS, attitude, on-ground, gear, flaps,
  engines, AP/ATHR, beacon light, `sim_timing`) + a small `A32NxState`
  extension (5 values). Every field is `Option` — **unknown is first-class;
  nothing is ever fabricated**.
* `SimTiming { state: Running|Paused|Unknown, sim_rate, slew_active }` —
  pause/slew/time-scale are first-class runtime inputs, not afterthoughts.
* `SimTimestamp { ms }` — simulator clock (`ABSOLUTE TIME`) or fixture-
  injected. Ordering uses this plus the session's monotonic `EventSeq`;
  wall clock never participates in decisions.
* `StateDelta` — named changed-field sets (`diff()` is a pure function).
* `FlightPhaseEngine` — extracted from OpenAIRAC (`openairac-charts/src/
  efb.rs`, MIT, commit `c7a2bfd`), faithful port; only the timestamp type
  changed. Extraction (instead of a git dependency) because `openairac-charts`
  pulls the whole navdata stack (store/rusqlite/reqwest) for ~200 lines of
  pure logic, and the engine will diverge here. If open-airac ever extracts
  `efb` into its own crate, swap back and delete this module.

## 4. Simulator boundary (fd-simconnect)

* **Own FFI, dynamic loading.** `simconnect-sys` 0.24.3 was evaluated and
  rejected: with bindgen 0.69.5 its `allowlist_item("(?i)SIMCONNECT.*")`
  silently generates an empty binding set (verified empirically). Instead,
  `ffi.rs` hand-declares the needed packed structs (`repr(C, packed)`,
  layouts cross-checked against real bindgen output) and resolves the ten
  required functions at runtime via `LoadLibraryW("SimConnect.dll")`.
  Consequences: no MSFS SDK / libclang needed to build; a missing DLL is a
  typed `ConnectionFailed` error, not a process-load failure.
* **Telemetry**: one data definition of 27 `FLOAT64` datums (standard
  simvars + 5 A32NX L:Vars + `ABSOLUTE TIME`), requested per sim frame;
  `mapping.rs` maps the payload to canonical state (fail-closed: NaN /
  out-of-range → unknown).
* **Write path**: `write.rs` (`pub(crate)`) — `SetDataOnSimObject` for
  settable vars, `TransmitClientEvent` for key events. There is no public
  raw-write API anywhere in the workspace.

## 5. A32NX bindings (provenance in `bindings.rs`, printable via `fd bindings`)

Read (5, all documented in FBW `fbw-a32nx/docs/a320-simvars.md`):
`A32NX_APU_N`, `A32NX_APU_BLEED_AIR_VALVE_OPEN`, `A32NX_FLAPS_HANDLE_INDEX`,
`L:A32NX_LIGHTS_NAV_LOGO`, `A32NX_OVHD_COND_PACK_1_PB_IS_ON`.

Write (2 actions): beacon via settable simvar `LIGHT BEACON` (A32NX has no
custom beacon behavior — verified against the FBW repo; the only custom
light behavior is the NAV/LOGO switch); NAV/LOGO via `L:A32NX_LIGHTS_NAV_LOGO`
(the model behavior's own SET_CODE writes this L:Var).

**All bindings are NOT live-verified yet** — that is exactly what the live
spike must prove. Deferred with rationale: the FBW Input-Event (`B:`)
write path needs the Asobo name-hash algorithm, which cannot be verified
offline; the L:Var/simvar paths above are fully documented.

## 6. Action pipeline

`Requested → Validated (catalog ∧ capability ∧ preconditions) → Dispatched
(write) → Verified (post-condition OBSERVED in state)` with terminal
`Rejected` / `Failed` / `TimedOut`. One `advance` pushes an action through
as many stages as current state allows. Verification deadlines are counted
in ticks. Fail-closed rules: unknown binding → reject; unknown current
state → reject (no blind writes); post-condition not observed → timeout.

## 7. Trace & replay

* JSONL, version-tagged lines (`{"v":1, ...}`), append-only, flushed per
  event. Chosen over SQLite: append-only is trivial, output is byte-
  deterministic for tests, human-inspectable; the format version allows a
  later migration. Categories: session_start/end, state_delta,
  phase_change, sim_state_changed, action_requested/validated/dispatched/
  verified/rejected/failed.
* Replay: fixture = versioned JSONL of snapshots + action injections.
  `ReplayAdapter` feeds one snapshot per tick; the runtime is unchanged.
  Determinism proof: two runs produce byte-identical traces (tested).

## 8. Known limitations / deferred

1. **LIVE VALIDATION PENDING** — no MSFS on this machine. The live spike
   must confirm: connection, telemetry rate (observed, not assumed), A32NX
   L:Var reads, the `LIGHT BEACON` write + observed post-condition, full
   trace, disconnect/reconnect. `fd live` prints
   `LIVE VALIDATION PENDING: …` and exits non-zero until then.
2. `SimConnect.dll` must be placed next to `fd.exe` (MSFS SDK client DLL;
   MSFS 2024 SDK build recommended for MSFS 2024). Not auto-copied by the
   build (no SDK dependency by design).
3. Engine combustion reads engines 1–2 only (A320 twin; slots 3–4 null).
4. `FLAPS HANDLE INDEX` range check 0..4 is A32NX-specific and lives in the
   adapter mapping; a future aircraft package owns such validation.
5. FBW Input-Event (`B:`) writes deferred (hash algorithm unverified).
6. The A32NX state extension lives in fd-core as a placeholder; Phase 2
   moves aircraft-specific state into aircraft packages.
