# Data Source-of-Truth Matrix

Architectural rule established after the 2026-08-25 live-spawn forensic
(C172 spawned at the KLAX airport reference point, inside terminal
structures). Each concept has exactly one primary source; secondary
sources are for validation only; the "must NOT use" column is binding.

| Concept | Primary source | Secondary validation | Must NOT use |
|---|---|---|---|
| Airport ident / reference position / elevation | OpenAIRAC world store (temporal query) | Navigraph AIRAC (local, read-only), X-Plane apt.dat `1` header | — |
| **Ground spawn / ramp / parking position** | **X-Plane native apt.dat code-1400 startup locations** (`scripts/live_setup.py`) | operator visual confirmation | **OpenAIRAC/Navigraph airport reference point — an ARP is navigation context, never a parking position** |
| Runway geometry (thresholds, heading, length) | OpenAIRAC world store | Navigraph AIRAC compare; X-Plane apt.dat `100`/`1300` records | — |
| Fixes / navaids (navigation matching) | OpenAIRAC world store | Navigraph AIRAC compare (local only) | — |
| Enroute airway connectivity | OpenAIRAC world store | Navigraph AIRAC compare | — |
| SIDs / STARs / approaches (procedure context) | OpenAIRAC world store (`procedure_legs`) | Navigraph CIFP compare (local only) | — |
| **X-Plane FMS state (live)** | **fd-xplm-bridge (XPLM410 read APIs, simulator thread)** | GPS indicator datarefs (`sim/cockpit/gps/destination_*`) | dataref "flight plan" reconstruction — none exists |
| FMS plan loading (pre-flight preparation) | fd-xplm-operator (operator action, one-shot file) | — | must never be reachable from FlightdeckOS upper layers or the bridge |
| Aircraft live state | X-Plane UDP telemetry (normalized snapshots) | Web API dataref reads | — |
| Flight initialization (scenario setup) | X-Plane Web API `POST /api/v3/flight` | `scripts/live_setup.py` machine-readable readback | UI automation |
| Flight phases / FDM / QoA / landing | FlightdeckOS deterministic analytics over recorded FDR | replay equality tests | — |

## Binding rules

1. **Navigation position ≠ simulator spawn position.** An airport
   reference point answers "where is the airport for context, proximity
   and navigation" — never "where do I place the aircraft".
2. **Navigraph data is a local diagnostic reference only.** It is never
   copied into the repository, never committed, never redistributed; only
   counts/hashes/identifiers/differential results may be quoted.
3. **The FMS bridge is read-only.** Plan entry is an operator
   preparation action (fd-xplm-operator), separated from the observation
   path by construction; FlightdeckOS upper layers have no FMS write
   surface (Task 7 §43).
4. **A broken upstream dataset stays broken.** Known OpenAIRAC RU-enroute
   gap (8 UU-region airway legs vs ~1293 in Navigraph 2608) is reported
   honestly; FlightdeckOS degrades to Unknown/2-point routes instead of
   repairing or masking upstream data.
