# FlightdeckOS Documentation

Canonical documentation. Historical design notes live in
[history/](history/) and must not be treated as current statements.

## Architecture & safety

- [ARCHITECTURE.md](ARCHITECTURE.md) — components, boundaries, dependency direction
- [SAFETY_MODEL.md](SAFETY_MODEL.md) — the project-defining invariants

## Status & planning

- [STATUS.md](STATUS.md) — evidence-backed capability snapshot
- [ROADMAP.md](ROADMAP.md) — ordered progression, no dates

## Simulator integration

- [XPLANE.md](XPLANE.md) — X-Plane 12 transports, safe control, limitations
- MSFS: SimConnect foundation only — see ARCHITECTURE.md and STATUS.md

## Aircraft & data

- [AIRCRAFT_PROFILES.md](AIRCRAFT_PROFILES.md) — packages, identity, Generic mode, Profile Genesis
- [FLIGHT_DATA.md](FLIGHT_DATA.md) — FDR V2, session lifecycle, FDM, QoA, landing, debrief
- [HEADLESS_FLIGHT_LAB.md](HEADLESS_FLIGHT_LAB.md) — VirtualSimulator, scenarios, proof boundaries

## Development

- [DEVELOPMENT.md](DEVELOPMENT.md) — build, gates, common tasks, workflow
- [PLATFORM_ATOMS.md](PLATFORM_ATOMS.md) — atom ownership map and dependency invariants
- [../CONTRIBUTING.md](../CONTRIBUTING.md) — contribution rules
