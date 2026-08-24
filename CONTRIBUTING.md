# Contributing to FlightdeckOS

Short rules, no bureaucracy:

1. **Maintain the safety boundaries.** No arbitrary simulator writes; new
   control paths go through the closed `CockpitAction` catalogue with
   capability evidence and fresh post-condition verification. See
   [docs/SAFETY_MODEL.md](docs/SAFETY_MODEL.md).
2. **New behavior needs tests.** Test invariants (unknown semantics,
   transitions, sign conventions), not plumbing.
3. **Unknown remains unknown.** Fail closed; never fabricate values,
   metrics or capabilities.
4. **Evidence honesty.** Live claims require live evidence; headless
   evidence must never be presented as simulator evidence.
5. **Do not commit generated recordings.** `traces/` is ignored; small
   deterministic fixtures under `fixtures/` are fine.
6. **Do not copy proprietary aviation datasets or manuals** into the
   repository.

## Gates

Every commit: `cargo fmt --check`, `cargo clippy --workspace --all-targets
-- -D warnings`, `cargo test --workspace`, `git diff --check` — all green.
See [docs/DEVELOPMENT.md](docs/DEVELOPMENT.md) for the details.

## License

Contributions are licensed under [Apache-2.0](LICENSE), like the rest of
the project. By submitting a contribution you agree to license it under
the same terms.
