# FlightdeckOS Roadmap

No dates. Order reflects dependencies and risk. Status labels follow
[STATUS.md](STATUS.md).

## DONE

- **Deterministic core** — canonical state, phase engine, typed units,
  closed action catalogue, executor with fail-closed validation.
- **Headless Flight Lab** — VirtualSimulator, scenario engine, fault
  injection, UUEE→ULLI reference mission, replay determinism.
- **Generic Aircraft mode** — unknown aircraft still get telemetry, phase,
  FDR, FDM, monitoring; nothing invented.
- **Aircraft packages + SOP primitives** — A32NX reference package,
  capability catalogues with provenance, procedure flows (offline).
- **Safe Control V1** — first closed cockpit action verified live on
  X-Plane 12.4.3 with fresh post-dispatch verification and restoration;
  write guard, capability evidence, warm-up freshness gate.
- **Flight Observatory infrastructure** — FDR V2 streaming, session
  lifecycle, route/runway monitoring, OpenAIRAC context, Mission Shadow
  (zero-write), structured debrief, short live session verified.

## NEXT

1. **Full real live-flight observation** — a complete manually flown flight
   recorded end-to-end by `fd observe` (FDR + debrief + shadow), on a
   stable simulator instance.
2. **Live FDR + debrief across a complete flight** — crash-safe recording
   over hours, torn-tail recovery exercised for real.
3. **Real Mission Shadow flight** — shadow intents vs observed behavior on
   a real flight (zero-write).
4. **FMS state extraction** — read the aircraft's active route/waypoints
   for observation and comparison.
5. **SID/STAR/APPR context** — procedure awareness from OpenAIRAC data
   (observation first, not automation).
6. **Profile Genesis observation tooling** — resource snapshot → user
   manipulates a control → delta → candidate bindings (observed ≠ trusted).
7. **CI** — GitHub Actions for fmt/check/clippy/test.

## THEN

- Supported X-Plane aircraft package beyond the A32NX reference
  (C172-class first: the live test aircraft).
- Expanded SOP coverage per package.
- AP/FMS typed actions (heading/altitude/speed targets through the same
  safe-control path).
- Supervised autonomy: autonomy proposes, operator confirms.
- Ground control (taxi), takeoff and landing control — each behind the
  same capability/verification gates.
- Autonomous reference flight (headless-proven mission flown live).

## LATER

- AI Crew: LLM emitting structured intents into validated tools; voice
  interaction; crew roles and procedures.
- Passenger Mode (full-crew autonomous operation, observer-only user).
- Career/dispatch/ATC ecosystem layers.

## Unknown-aircraft onboarding track (parallel)

Profile Genesis pipeline: generic capability discovery → resource
observation → candidate bindings → DraftProfile → documentation/testing →
trusted aircraft package. Core rule at every step: **observed ≠ trusted,
correlated ≠ verified, writable ≠ safe.**
