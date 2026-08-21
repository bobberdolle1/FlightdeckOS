# FlightdeckOS Platform Atoms

Architectural map of FlightdeckOS: stable product/runtime atoms, their
ownership boundaries and dependency direction. This document is the
reference for deciding where new functionality belongs.

## Dependency direction (invariant)

```text
AI (future) ──► Mission ──► SOP ──► Aircraft ──► Core ◄── Sim adapters
                     │                                  ▲
                     └────────► FDM/FDR ────────────────┘
Scenario runner wires everything; nothing depends on Scenario.
```

Hard invariants:
* AI never owns aircraft state or simulator writes.
* FDM never controls the aircraft (analysis only).
* UI never touches raw simulator APIs.
* Package data can never perform arbitrary simulator writes.
* MissionController contains no simulator-specific APIs.
* Virtual simulator contains no SOP truth.
* SOP engine contains no aircraft physics.

## Crates today

| Crate | Atom(s) | Maturity |
|---|---|---|
| fd-core | Core types, Units, Events, Capabilities, Geo, Adapter boundary | implemented |
| fd-runtime | Trace, Action pipeline, Phase engine, Replay driver | implemented |
| fd-simconnect | Simulator adapter (MSFS), Binding discovery (A32NX) | implemented; LIVE VALIDATION PENDING |
| fd-aircraft | Aircraft package, State fields, Conditions, Action catalog (A32NX) | implemented |
| fd-sop | SOP flow engine (observe/action steps, dependencies) | implemented (minimal) |
| fd-fdm | Flight Data Recorder, FDM events, QoA/QoL metrics | implemented (development thresholds) |
| fd-virtual | Virtual simulator, Virtual aircraft model (semantic+kinematic), deterministic clock | implemented (test model) |
| fd-mission | Mission Controller, Route follower | implemented (minimal) |
| fd-scenario | Scenario Engine, assertions, headless runner | implemented |
| fd-app | CLI surface | implemented |

## Planned atoms (documented, not implemented)

### Foundation
| Atom | Purpose | Maturity |
|---|---|---|
| Time/Clock abstraction | injectable clock beyond scenario mode | planned |
| Configuration | global config layer | planned |

### Simulator
| Atom | Purpose | Maturity |
|---|---|---|
| Simulator Discovery | detect running sims | planned |
| Aircraft Detection | identify loaded addon | planned (mock only) |
| X-Plane adapter (fd-xplane) | first live-validated adapter: SDK/plugin, DataRef discovery/cache, Commands, flight-loop | **NEXT live target** (see HEADLESS_FLIGHT_LAB.md) |
| Binding Discovery | runtime binding inventory | planned |

### Aircraft
| Atom | Purpose | Maturity |
|---|---|---|
| Generic Aircraft State | core snapshot | implemented |
| Aircraft Extension State | opaque ext-value map + registry semantics | implemented (registry), enrichment planned |
| Aircraft Package | versioned TOML package | implemented |
| Aircraft Systems | semantic system models in packages | partial (virtual model only) |
| Action Catalog | closed actions + verify fns | implemented |
| Performance Model | per-addon performance data | planned |

### Procedures / Crew
| Atom | Maturity |
|---|---|
| SOP Engine (flows, observe/action, deps) | implemented (minimal) |
| Checklist Engine (challenge/response) | planned |
| Trigger Engine (phase/state triggers) | planned |
| Crew Roles (tags) | implemented (minimal) |
| PF/PM Ownership | planned |
| Callouts | planned |

### Autonomy
| Atom | Maturity |
|---|---|
| Mission Controller | implemented (minimal) |
| Control Authority ladder | planned |
| Handoff protocol | planned |
| AP/FMS Driver | partially represented by FlightControlTargets |
| Ground Controller | planned |
| Flight Controller (L3) | planned (deterministic first) |

### Data
| Atom | Maturity |
|---|---|
| Flight Data Recorder | implemented |
| FDM (development rules) | implemented |
| FOQA-style detection | = FDM development rules |
| Quality of Approach | implemented (measurements) |
| Quality of Landing | implemented |
| Quality of Autonomy | implemented (raw counters) |
| Debrief | planned |

### Operations
Navigation (route follower minimal; deeper nav via open-airac reuse later),
Flight Planning, Weather, Dispatch, ATC, Failures, Maintenance, Career — all
**planned**; none started.

### AI
LLM Gateway, Crew Brain, RAG/Knowledge, Voice/STT/TTS — **planned**; the
runtime is deliberately AI-free until primitives are proven.

### Testing
| Atom | Maturity |
|---|---|
| Scenario Engine | implemented |
| Virtual Simulator | implemented |
| Virtual Aircraft Models | implemented (A32NX subset kinematic+semantic) |
| Fault Injection | planned (APU failure scenario pattern exists as a test target) |
| Scenario Assertions | implemented |
| Regression Corpus | starting (fixtures/) |

### Presentation
API server, openairac-map integration, Cockpit UI, Dev/Diagnostics UI —
**planned**.

## Real-simulator roadmap decision

X-Plane 12 is the simulator available to the developer, therefore:
**X-Plane 12 becomes the first live-validation target.** The existing
SimConnect (MSFS) adapter remains supported code; MSFS live validation is
deferred until MSFS is available. No SimConnect code will be deleted or
rewritten for this.
