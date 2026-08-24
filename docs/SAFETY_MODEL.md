# Safety Model

These invariants are project-defining. Changes that weaken them are rejected
regardless of other merit.

## 1. Closed action catalogue

The entire discrete action surface is the `CockpitAction` enum. There is no
"execute arbitrary command" / "write arbitrary dataref" API exposed to
upper layers. Adding an action means adding a typed variant + catalogue
entry + verification + tests.

## 2. Live writes disabled by default

`LiveWriteGuard` starts DISABLED. Arming requires an explicit operator flag
(`--allow-write`) at process start. The observation tooling (`fd observe`)
has no write capability at all; Mission Shadow has no write surface at all.

## 3. Capability evidence required

An action dispatches only if the active capability set advertises support
with evidence. Capability without evidence is not capability.

## 4. Fail-closed unknown state

Preconditions and analytics treat unknown as "not proven". Unknown state
rejects actions, closes FDM episodes, and yields `None`/`Indeterminate`
metrics — never fabricated zeros or guessed positives.

## 5. Fresh post-condition verification

Action success requires an **observed** post-condition from a snapshot
strictly newer than the dispatch boundary. Verification channels must be
`Fresh` — warm-up/transient state can never verify an action. Cached or
pre-dispatch state is inadmissible evidence.

## 6. Telemetry freshness semantics

`WarmingUp`/`Stale`/`Missing`/`Invalid` data is never silently trusted as
authoritative. A newly connected channel must prove stability (consecutive
consistent samples) before becoming evidence.

## 7. Simulator adapter boundary

All simulator access goes through the `SimulatorAdapter` trait. Analytics,
shadow, route monitoring and debrief never touch an adapter. Dependency
direction is enforced by crate structure.

## 8. Aircraft hot-swap invalidation

Aircraft change/reload clears identity-derived state, cached resources and
warm-up counters. No stale aircraft-specific state survives a swap.

## 9. Shadow is zero-write

Mission Shadow takes no adapter and has no action surface — it cannot write
by construction. Intents are data records for comparison and debrief.

## 10. No analytics → control authority

FDM, QoA, landing analysis, route monitoring and the debrief are read-only
consumers. None of them can dispatch, command, or arm the write guard.

## 11. No AI → raw simulator writes

The designed AI boundary (when AI arrives):

```text
LLM → structured intent → validated deterministic tools → CockpitAction
      → capability/precondition gates → simulator adapter
```

Never `LLM → raw dataref/SimVar`. Today no LLM is wired into the runtime.

## 12. Evidence honesty

Live claims require live evidence; headless evidence must never be
presented as simulator evidence; unknown stays unknown in docs as in code.
