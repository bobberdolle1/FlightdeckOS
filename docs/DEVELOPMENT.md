# FlightdeckOS Development Guide

## Toolchain

Rust stable, edition 2024. Windows is the primary development platform;
the workspace is plain Cargo with no OS-specific build scripts (the
SimConnect crate is Windows-only by nature).

## Build / test

```bash
cargo build --workspace
cargo fmt
cargo fmt --check
cargo clippy --workspace --all-targets -- -D warnings
cargo test --workspace
git diff --check
```

All five gates are expected green before every commit. Clippy warnings are
errors by policy.

## Workspace layout

See the crate table in the root [README](../README.md#workspace).
Dependency direction: `fd-core` ← everything; adapters implement
`fd-core::adapter` traits; analytics consume snapshots only.

## Common tasks

### Add a test

Unit tests live beside the code (`#[cfg(test)] mod tests`); integration
tests in `crates/<crate>/tests/`. Test behavior and invariants — unknown
semantics, transitions, sign conventions — not plumbing. Deterministic:
no wall clock, no RNG.

### Add a scenario

Create `scenarios/<name>.toml` following the existing specs
(`deny_unknown_fields` is on). Run:

```bash
cargo run -p fd-app -- scenario --run scenarios/<name>.toml
```

Nominal scenarios must reach mission `Completed`; negative scenarios
declare a specific `expected_failure`.

### Add a CockpitAction

1. Typed variant in `fd-core::actions` + catalogue entry with
   preconditions, verifier and verification channels.
2. Transport support behind capability evidence (adapter-side).
3. Fail-closed precondition tests + fresh-verification tests.
4. Live claims require live evidence (see CONTRIBUTING.md).

### Add an FDM detector

Extend `fd-fdm::fdm` with an episode state machine (Started/Ended),
typed thresholds with named development defaults, unknown-input fail-closed
behavior, and tests for the unknown case.

### Add an aircraft package

Copy the structure of `aircraft/a32nx`; `cargo run -p fd-app -- package
--dir <dir>` must pass. Bindings carry provenance; never claim a binding
without evidence.

## Documentation expectations

- Canonical docs live in `docs/`; task/historical material in
  `docs/history/`.
- Status labels: LIVE VERIFIED / SHORT-LIVE VERIFIED / OFFLINE VERIFIED /
  HEADLESS VERIFIED / EXPERIMENTAL / PLANNED — do not upgrade a label
  without evidence.
- Every documented CLI command must exist (`--help` is the source of
  truth).

## Git workflow

- Small, atomic commits with conventional messages
  (`feat(crate): ...`, `fix(crate): ...`, `docs: ...`).
- Never force-push `master`; never rewrite public history.
- Do not commit runtime recordings (`traces/` is ignored); small
  deterministic fixtures under `fixtures/` are fine.
