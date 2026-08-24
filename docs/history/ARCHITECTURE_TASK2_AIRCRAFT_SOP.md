# FlightdeckOS — Task 2: Aircraft Package + SOP Primitives

Status: implemented (offline-complete). Live MSFS validation: **PENDING**.

## 1. Crate boundary

```text
fd-app ──► fd-runtime ──► fd-sop ──► fd-aircraft ──► fd-core
   │                          │
   └──► fd-simconnect ────────┴──► fd-core
```

* **fd-core** — aircraft-neutral. The only aircraft-flavored item is the
  closed `CockpitAction` enum (generic cockpit primitives: beacon, tri-state
  NAV/LOGO switch) plus its canonical-name registry; the A32NX typed state
  struct from Task 1 was removed. Aircraft values now travel in an opaque
  `aircraft_values: BTreeMap<u16, f64>` extension map on the snapshot;
  fd-core attaches no meaning to the ids.
* **fd-aircraft** — package format + registries: manifest identity/version,
  roles, closed `StateField` set (with value types and ext-id mapping),
  typed `Condition` model with tri-state evaluator (`True/False/Unknown`),
  raw flow TOML shapes, and the A32NX action catalog (pre-/post-conditions).
* **fd-sop** — procedure semantics: package loading with full semantic
  validation (duplicate ids, unknown role/state-field/action, missing/self
  dependency, cycles) and the deterministic `FlowEngine`. Knows nothing
  about SimConnect or how actions are physically performed.
* **fd-simconnect** — unchanged boundary: maps the five A32NX L:Vars into
  the opaque extension ids defined by fd-aircraft's registry constants.

## 2. Trusted code vs package data

Package data may reference ONLY:
* state fields from the closed `StateField` registry (unknown name → load
  failure);
* actions from the closed `CockpitAction` name registry
  (`ACTION_NAMES`/`try_from_name`; unknown name → load failure);
* roles from the closed `Role` set;
* binding names from the closed provenance list.

There is deliberately no way to express a raw variable write in package
data. All writes remain hard-coded Rust (`fd-simconnect::write`, reachable
only through the runtime action pipeline).

Format: **TOML** (human-editable, comment support, established `toml`
parser, serde-typed structures with `deny_unknown_fields` on the manifest).

## 3. Condition model

`Condition ∈ { IsTrue, IsFalse, Equals, AtLeast, AtMost, Known }` over a
`StateField`; evaluation returns `TriBool { True, False, Unknown }`.
`Unknown` never satisfies a step — missing simulator data is never guessed.
Type mismatches (e.g. `is_true` on a numeric field) are rejected at
package-validation time.

## 4. Flow lifecycle

```
FlowStarted → per step: Pending → Ready (deps verified)
    → observe: WaitingForVerification (emitted once) → Verified on True
    → action: StepActionRequested → runtime pipeline → Verified on observed
      post-condition / Failed on Rejected|Failed|TimedOut
→ FlowCompleted | FlowFailed (non-terminal steps Aborted)
```

Deterministic: steps evaluated in definition order; ready-set ordering is
definition order; outcomes applied at the next tick's flow pass.

## 5. Action integration

SOP requests route through the SAME two-phase submit as user actions:
stage → `ActionRequested` trace append → commit → validate (catalog +
capability + preconditions) → dispatch → verify against observed state.
No second action state machine exists. Failure mapping: action
Rejected/Failed/TimedOut ⇒ step Failed ⇒ flow Failed (no retries yet).

## 6. Replay

Fixtures (JSONL v2) feed raw snapshots through the production runtime +
flow engine. Determinism: identical fixture ⇒ byte-identical trace.
The Task 1 fixture was migrated to v2 (snapshot gained the opaque
`aircraft_values` map; trace schema bumped to v2 for SOP events).

## 7. Deliberately deferred

Beacon live mechanism (simvar vs event), SIMCONNECT_RECV_OPEN version
diagnostics, NAV/LOGO fractional decode hardening, L:Var zero-ambiguity,
pause subscription aliases, automatic FlightPhase triggers, interruption/
resume/abnormal branches, retries, crew model beyond Role tags, LLM
(anything AI-related).
