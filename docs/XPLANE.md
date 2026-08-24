# FlightdeckOS on X-Plane 12

X-Plane 12 is the primary live integration target. Everything below reflects
evidence from real X-Plane **12.4.3** sessions on the development machine.

## Transports

Two independent transports, reported independently by `TransportHealth`
(`Available`/`Degraded`/`Unavailable` each):

- **Native UDP** — RREF subscriptions for normalized telemetry (~3–4 Hz
  observed). Per-channel freshness windows; values outside the window read
  as absent, never as stale-trusted. Source-authenticity check: only the
  configured simulator host may feed telemetry.
- **Local Web API** (loopback REST, v1/v2/v3) — resource discovery and
  discrete command control. No XPLM plugin required for the proven slice.

## Resource discovery

Datarefs and commands are resolved by name at runtime through the Web API
(`GET /api/v3/datarefs?filter[name]=...`). Ids are **session-scoped**: every
simulator restart invalidates them, so FlightdeckOS resolves resources
fresh per session and evicts cached ids on session expiry.

Command activation uses the **v2 singular route** with a required body:

```text
POST /api/v3/command/{id}/activate        body: {"duration": 0.1}
```

(The plural `/commands/{id}/activate` route with an empty body — present in
older assumptions — returns an HTML 404 on 12.4.3. This mismatch was found
by live evidence and fixed; a regression test pins the correct route.)

## Safe control path

```text
CockpitAction (closed catalogue)
  → LiveWriteGuard (DISABLED by default; --allow-write arms explicitly)
  → capability evidence check
  → preconditions (fail-closed on unknown state)
  → Web API command dispatch
  → verification: fresh post-dispatch snapshot only
    (strictly newer than dispatch AND all verification channels Fresh)
  → Verified / Failed (traced with timestamps)
```

Live-proven end to end with `SetBeacon`, including restoration of the
original cockpit state through the same path, and the guard negative test
(rejection without `--allow-write`, simulator untouched).

## Freshness and warm-up

A newly connected channel is `WarmingUp` until 3 consecutive finite samples
arrive (~1 s at the subscribe rate). WarmingUp data never verifies an
action and never satisfies a positive safety condition. The transient
first-sample disagreement observed right after adapter connect is exactly
what warm-up absorbs; post-dispatch verification remained correct before
and after the mechanism was added.

## Simulator lifecycle

- **Disconnect**: typed state, bounded resubscribe/reconnect loop, no
  unbounded hangs.
- **Restart**: web session ids invalidated, freshness reset, warm-up
  restarts; no pending action survives.
- **Aircraft reload/hot-swap**: `invalidate_aircraft` /
  `set_identity_claim` clear identity-derived state, cached web resources,
  and warm-up counters — no stale-fresh carryover across an aircraft
  change (spec-tested).

## Known observations (environment-specific)

- The **embedded Web API server intermittently stopped responding after
  idle periods** on the development machine (12.4.3): even a bare HTTP
  client got connection timeouts while the simulator process lived.
  FlightdeckOS treats this as an environmental fact, not a code bug: every
  web operation is bounded (2 s connect + 5 s total), failures set a
  cooldown (5 s, no request spam), and a wedged server surfaces as
  `Unavailable` — never as a silent hang.
- One simulator crash occurred during the campaign after loading a flight
  at an airport with no installed scenery; flights at airports with
  present scenery loaded and ran normally.
- One boot stall at the OpenGL-init stage required manual dismissal.
  These observations motivated the "full live-flight validation pending"
  status rather than any architectural change.

## CLI

```bash
# read-only telemetry monitoring (no writes possible)
cargo run -p fd-app -- xplane --monitor-secs 30 --aircraft-icao C172

# zero-write flight observatory: FDR + phase + route + debrief
cargo run -p fd-app -- observe --monitor-secs 60 \
  --aircraft-icao C172 \
  --fdr-out traces/observe.jsonl \
  --debrief-out traces/debrief.json \
  --origin-icao EDDF --destination-icao EDDM \
  --world-store /path/to/openairac/world.openairac.sqlite

# live-write smoke (requires explicit arming; verification is mandatory)
cargo run -p fd-app -- xplane --beacon off --allow-write --aircraft-icao C172
```

The observatory verb has no write capability at all; the smoke action
requires an explicit flag and is the only live-write surface in the
project.
