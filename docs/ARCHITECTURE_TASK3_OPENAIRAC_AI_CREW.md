# FlightdeckOS v0.2 — OpenAIRAC Integration & AI Crew Runtime

## Architecture Overview

FlightdeckOS v0.2 connects the core aircraft simulation and SOP engine with the released **OpenAIRAC 3.2 AI Crew Gateway** (`http://127.0.0.1:8989/api/openairac/v1`), establishing a deterministic AI co-pilot / crew assistance runtime.

```
┌─────────────────────────────────────────────────────────────┐
│                       AI Flight Crew                        │
│         (fd-crew: AiCrewRuntime + Deterministic Provider)   │
└──────────────────────────────▲──────────────────────────────┘
                               │
                      CrewToolRegistry
               - get_flight_state
               - get_active_leg
               - get_tod_and_descent
               - get_arrival_brief
               - get_weather
               - get_data_freshness
                               │
┌──────────────────────────────┴──────────────────────────────┐
│                    CrewFlightContext                        │
│            (Canonical FlightdeckOS Flight State)            │
└──────────────────────────────▲──────────────────────────────┘
                               │
                    OpenAiracClient (REST / JSON)
                               │
┌──────────────────────────────┴──────────────────────────────┐
│                  OpenAIRAC 3.2 Gateway                      │
│            (127.0.0.1:8989 / localhost-only)                │
└─────────────────────────────────────────────────────────────┘
```

---

## 1. Crate Hierarchy & Boundaries

- **`fd-openairac`**: Dedicated OpenAIRAC 3.2 client crate. Handles HTTP transport over localhost, schema version negotiation (`flightdeck_snapshot_v2`, `compact_ai_snapshot_v1`), monotonic event polling, structured briefings, and mapping into canonical `CrewFlightContext`.
- **`fd-crew`**: AI Crew runtime and tool-calling engine. Implements pluggable `AiModelProvider`, `DeterministicAiProvider`, `CrewToolRegistry`, tool fact evidence tracking (`ToolEvidence`), and safe action proposal separation.
- **`fd-sop`**: Deterministic Standard Operating Procedures engine (checklists, flows, step verifications).
- **`fd-aircraft`**: Aircraft package specifications, closed state field registries, and condition evaluator.
- **`fd-simconnect`**: Low-level simulator communication layer.
- **`fd-runtime`**: Main simulation and flight execution loop.
- **`fd-app`**: Application binary with CLI commands (`live`, `replay`, `crew`, `openairac`, `bindings`, `package`).

---

## 2. Source of Truth Ownership (No Competition)

| Domain | Authoritative Owner | Description |
| :--- | :--- | :--- |
| **Navigation & Route** | **OpenAIRAC** | Geodesic waypoints, active leg, XTK, DTG, route progress |
| **Flight Phase** | **OpenAIRAC** | 12-state `FlightPhaseEngine` with multi-tick anti-flapping hysteresis |
| **Descent Profile & TOD** | **OpenAIRAC** | Standard 3.0° descent profile, TOD distance, required vertical speed |
| **Weather Context** | **OpenAIRAC Gateway** | Origin/destination METAR/TAF, runway wind components |
| **Online ATC Context** | **OpenAIRAC Gateway** | Active VATSIM/IVAO controllers along corridor & at aerodromes |
| **Aircraft System State** | **FlightdeckOS** (`fd-simconnect` / `fd-aircraft`) | Discrete cockpit switch states, engine combustion, gear/flaps |
| **SOP & Checklist State** | **FlightdeckOS** (`fd-sop`) | Active flows, completed/pending steps, crew role ownership |
| **Crew Dialogue & AI** | **FlightdeckOS** (`fd-crew`) | Tool-grounded natural language question answering |

---

## 3. Freshness Propagation & Independence

FlightdeckOS faithfully preserves independent freshness across all data subsystems:

- **Telemetry**: `CURRENT` ($\le 5\text{s}$), `STALE` ($> 5\text{s}$), `DISCONNECTED`
- **Weather**: `CURRENT`, `STALE`, `UNAVAILABLE`
- **Online ATC**: `CURRENT`, `STALE`, `UNAVAILABLE`
- **Navdata**: `CURRENT`, `FUTURE`, `STALE`, `SOURCE_REQUIRED`, `UNKNOWN`

If OpenAIRAC flags weather as `STALE`, FlightdeckOS exposes `weather = STALE` immediately without silently assuming freshness.

---

## 4. Strict SOURCE_REQUIRED Semantics

For airports lacking public terminal procedure datasets (e.g. `URAS` Sukhumi / Babushara):
- `STAR` returns `None`
- `Approach` returns `None`
- `is_source_required` is `true`
- AI Crew responses strictly report `SOURCE_REQUIRED` and never hallucinate plausible procedures.

---

## 5. Security & Read-Only Safety

- **Localhost Only**: The OpenAIRAC Gateway is accessed strictly on `127.0.0.1:8989`.
- **Read-Only / Advisory in v0.2**: The AI Crew Runtime observes, briefs, and answers questions. No arbitrary simulator commands, datarefs, or shell commands can be executed by model outputs.
