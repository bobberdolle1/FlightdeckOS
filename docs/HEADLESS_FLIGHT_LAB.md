# FlightdeckOS Headless Flight Lab

The headless flight lab lets FlightdeckOS run, record and analyze complete
flights **without any flight simulator installed or running**. The same
production runtime (actions, SOP, mission controller, trace) executes
against a `VirtualSimulatorAdapter` instead of MSFS/X-Plane.

## What a virtual PASS proves

* Mission/SOP/action ORCHESTRATION is internally correct.
* Phase progression, dependencies and post-condition gating work.
* FDR/FDM/QoA pipelines produce real computed metrics from generated data.
* Deterministic replay: identical scenario ⇒ identical semantic output.

## What a virtual PASS does NOT prove

* X-Plane or MSFS bindings work (no live simulator involved).
* Real A32NX behavior matches the model.
* Real flight dynamics / handling are correct.
* Any airline procedure fidelity beyond what package provenance states.

Every CLI result prints this explicitly:
`HEADLESS VIRTUAL TEST — NOT LIVE SIMULATOR VALIDATION — NOT REAL AIRCRAFT
PERFORMANCE VALIDATION`.

## Two simulation layers

### Semantic aircraft simulation (`fd-virtual::systems`)
Discrete system behavior for procedure orchestration: APU
Off→Starting→Available with a time-based spool, bleed gated on availability,
beacon/engines discrete states. Invalid transitions are rejected. Delays are
development values (`APU_SPOOL_MS = 90 s simulated`).

### Kinematic flight simulation (`fd-virtual::kinematics`)
Bounded-rate state evolution: altitude integrates from vertical speed
(bounded by climb/descent limits), speed changes at bounded
accel/deceleration, heading turns at bounded turn rate, position dead-
reckons along track. Targets are COMMANDS — the model moves gradually and
never teleports. Touchdown happens by descending through field elevation.
**Development limits only; not aircraft performance data.**

## Deterministic clock

The virtual world advances by a fixed timestep (`dt_ms`, default 100 ms).
No wall-clock decisions anywhere in scenario semantics; wall time appears
only as a diagnostic (speedup factor). Hours of flight simulate in seconds.

## Scenario structure

TOML spec: identity, optional package+flow, origin/destination airports,
initial conditions, mission parameters (cruise altitude), simulation
(dt_ms, max_sim_seconds cap). Runner output: structured JSON-serializable
report (phases, actions, procedure steps, FDM events, approach/landing
measurements, autonomy counters) plus machine-checkable assertions.

## FDM / QoA

FDM consumes FDR samples through the SAME pipeline later fed from real
simulators. Development thresholds (sink rate, low-altitude bank, hard
touchdown) are named, configurable, and explicitly NOT airline policy.
Development defaults: excessive sink >= 1200 fpm at AGL <= 2500 ft,
excessive bank >= 30 deg at AGL <= 1500 ft, hard touchdown >= 600 fpm
(600 is the shipped default; unit tests override thresholds locally).
Unknown measurements produce unknown metrics — never fabricated events or
zeros.

## Substituting a real simulator

`VirtualSimulator` implements the same `SimulatorAdapter` +
`FlightControlTargets` boundaries as any real adapter. Replacing it with an
XPlaneAdapter changes zero lines in Mission/SOP/FDM code.

## Next live step: X-Plane 12

Because X-Plane 12 is available to the developer, it becomes the first
live-validation target. Smallest first slice:

1. fd-xplane adapter skeleton implementing `SimulatorAdapter` read path
   (position, altitude, speeds, on-ground) via the X-Plane SDK UDP/plugin
   bridge.
2. Verify telemetry against the running simulator (no writes yet).
3. Then add one closed write action (e.g. beacon) and verify post-condition
   observation — the same shape as the pending MSFS spike.

MSFS/SimConnect remains supported code; its live validation stays deferred
until MSFS is available.
