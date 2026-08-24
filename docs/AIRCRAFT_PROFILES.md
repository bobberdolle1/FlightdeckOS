# Aircraft Profiles and Generic Mode

## Aircraft identity

`AircraftIdentity` = ICAO type + optional tail/author/description/ACF name
+ **provenance**: `Unknown`, `UserProvided` (operator claim — recorded,
never trusted) or `Adapter` (read from a trusted transport). Stock X-Plane
UDP cannot read identity strings, so live X-Plane sessions use
operator-claimed identity with `UserProvided` provenance.

## Aircraft packages

A package is a validated directory describing one aircraft for
FlightdeckOS: capability catalogue, action bindings with provenance, SOP
flows, mission parameters. The reference package is `aircraft/a32nx`.

```bash
cargo run -p fd-app -- package --dir aircraft/a32nx   # fail-closed validation
cargo run -p fd-app -- capabilities --package aircraft/a32nx
cargo run -p fd-app -- bindings                        # binding table + provenance
```

Validation is fail-closed: an invalid package is rejected, not repaired.

## Capability model

Capabilities state what may be attempted **with evidence**
(`CapabilityStatus` + evidence source). The executor refuses any action the
active capability set does not support. Capability without evidence is
worth nothing in FlightdeckOS.

## Generic Aircraft mode

No package ≠ useless. An unknown aircraft (e.g. an unsupported IL-76
addon) still receives:

- generic normalized telemetry and flight phase;
- FDR recording and session lifecycle;
- FDM analytics (development thresholds);
- route monitoring and OpenAIRAC context;
- QoA/landing analytics where the data exists.

Without trusted aircraft-specific knowledge FlightdeckOS does **not**
invent: systems state, SOP, cockpit actions, performance limits, or
autonomy. The unknown-aircraft scenario (`scenarios/unknown_aircraft_generic.toml`)
pins this behavior.

## Profile Genesis (future/partial)

The planned onboarding path for unsupported aircraft:

```text
unknown aircraft
  → generic capability discovery
  → resource observation (snapshot → user manipulates a control → delta)
  → candidate bindings
  → DraftProfile
  → documentation + testing
  → trusted aircraft package
```

Core rule: **observed ≠ trusted; correlated ≠ verified; writable ≠ safe.**
Observation tooling is not yet implemented (see [ROADMAP.md](ROADMAP.md)).

## Trust summary

| Level | Meaning |
| --- | --- |
| Adapter-read identity | trusted provenance (trusted transport required) |
| Operator-claimed identity | recorded, never trusted |
| Package capability | trusted after fail-closed validation |
| Generic capability | telemetry/analysis only, no aircraft-specific claims |
| Observed resource behavior | candidate evidence only, never auto-promoted |
