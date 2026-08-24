//! Mission Shadow Mode (spec §30–31): observe real telemetry, compute what
//! mission autonomy WOULD have commanded via
//! [`intended_commands`](crate::controller::intended_commands), and compare
//! that intent against the autopilot state actually observed — **zero
//! writes**.
//!
//! # Zero-write enforcement is by construction
//!
//! * [`MissionShadow`] takes no adapter parameter at all: there is no path
//!   from a [`ShadowEntry`] back to the aircraft.
//! * Derived intents ([`HighLevelIntent`], [`crate::intents`]) are plain
//!   data records attached to entries: the shadow has no action/dispatch
//!   surface to hand them to, so recording an intent can never fly the
//!   aircraft. Same compile-level argument as above — a dispatch would
//!   require importing an adapter/action type, which is absent here.
//! * No non-test code in this module imports or references
//!   [`FlightControlTargets`](fd_core::adapter::FlightControlTargets) or any
//!   other write surface. The type consumes snapshots/selections and
//!   produces records; nothing else. (`cargo` cannot grep this claim — but a
//!   compile error does: adding a write here would require importing the
//!   adapter trait, which is exactly what is absent.)
//!
//! # Observed AP targets provenance
//!
//! Observed selections are bridged into [`ObservedApTargets`] from the
//! canonical [`TelemetrySnapshot`]. NOTE: the current frozen `fd-core`
//! snapshot type exposes only `autopilot_master` / `autothrottle_arm` —
//! dedicated AP-selection fields (`autopilot_heading_sel`,
//! `autopilot_altitude_sel`, `autopilot_vs_sel`) do not exist there yet.
//! Until they land upstream, adapters bridge their raw AP selections into
//! [`ObservedApTargets`]; the shadow itself stays simulator-independent.
//!
//! # Matching rule
//!
//! A channel of an entry is *evaluated* only when BOTH the intended value
//! and the observed value are present (`Some`). An unknown observed channel
//! is absence of evidence — never a divergence — and the mission being
//! silent on a channel is likewise not comparable evidence; both are
//! excluded from the match-ratio denominator. Evaluated channels match iff
//! the difference is within tolerance: heading ±3 deg (angular-aware across
//! north), altitude ±200 ft, vertical speed ±100 fpm, speed ±10 kt.
//!
//! # Channel coverage notes
//!
//! * Route-leg channel: intentionally deferred this task — it needs the
//!   route monitor observation type (Lane R owns `monitor.rs`); Integration
//!   wires it into [`ObservedApTargets`] or a sibling observed struct later.
//! * Vertical-trend channel: already covered by the existing
//!   VERTICAL_SPEED comparison (commanded VS target vs observed VS).
//! * Mission-phase channel: not implementable today — the observed side
//!   carries no mission phase (AP selections have no phase concept), so
//!   there is nothing comparable on the observed half; revisit when an
//!   observed-phase source exists.

use crate::controller::{
    MissionCommands, MissionContext, MissionParameters, MissionPhase, intended_commands,
};
use std::collections::BTreeMap;

use serde::{Deserialize, Serialize};

use crate::intents::{HighLevelIntent, intent_from_tick};

/// Channel slots in [`ShadowEntry::matches`] and [`ShadowReport::channels`].
pub mod chan {
    /// Altitude channel ([`MissionCommands::set_target_altitude_ft`]).
    pub const ALTITUDE: usize = 0;
    /// Speed channel ([`MissionCommands::set_target_speed_kt`]).
    pub const SPEED: usize = 1;
    /// Heading channel ([`MissionCommands::set_target_heading_deg`]).
    pub const HEADING: usize = 2;
    /// Vertical-speed channel
    /// ([`MissionCommands::set_target_vertical_speed_fpm`]).
    pub const VERTICAL_SPEED: usize = 3;
    /// Number of compared channels.
    pub const COUNT: usize = 4;
}

/// Per-channel tolerances (same units as the compared values).
pub const HEADING_TOL_DEG: f64 = 3.0;
/// Altitude tolerance in feet.
pub const ALTITUDE_TOL_FT: f64 = 200.0;
/// Vertical-speed tolerance in feet per minute.
pub const VS_TOL_FPM: f64 = 100.0;
/// Speed tolerance in knots.
pub const SPEED_TOL_KT: f64 = 10.0;

const TOLERANCES: [f64; chan::COUNT] = [ALTITUDE_TOL_FT, SPEED_TOL_KT, HEADING_TOL_DEG, VS_TOL_FPM];
const CHANNEL_NAMES: [&str; chan::COUNT] = ["altitude", "speed", "heading", "vertical speed"];
const UNITS: [&str; chan::COUNT] = ["ft", "kt", "deg", "fpm"];

/// Autopilot selections actually observed for one sample. Every field is
/// `Option`: `None` means unknown/not reported, which is absence of
/// evidence — never a divergence.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct ObservedApTargets {
    pub heading_deg: Option<f64>,
    pub altitude_ft: Option<f64>,
    pub vertical_speed_fpm: Option<f64>,
    pub speed_kt: Option<f64>,
}

impl ObservedApTargets {
    /// Pair each channel's (intended, observed) values, in [`chan`] order.
    fn pairs(&self, intended: &MissionCommands) -> [(Option<f64>, Option<f64>); chan::COUNT] {
        [
            (intended.set_target_altitude_ft, self.altitude_ft),
            (intended.set_target_speed_kt, self.speed_kt),
            (intended.set_target_heading_deg, self.heading_deg),
            (
                intended.set_target_vertical_speed_fpm,
                self.vertical_speed_fpm,
            ),
        ]
    }
}

/// Angular-aware absolute difference between two headings in degrees,
/// normalized to `[0, 180]`. NaN inputs yield NaN (never a match).
fn heading_delta_deg(a: f64, b: f64) -> f64 {
    let d = (a - b).rem_euclid(360.0);
    if d > 180.0 { 360.0 - d } else { d }
}

fn within_tolerance(channel: usize, want: f64, got: f64) -> bool {
    if channel == chan::HEADING {
        heading_delta_deg(want, got) <= TOLERANCES[channel]
    } else {
        (want - got).abs() <= TOLERANCES[channel]
    }
}

/// One recorded shadow sample: what autonomy intended vs what the real
/// autopilot showed, at one sample sequence point.
///
/// `matches[i]` is `true` iff channel `i` was evaluated AND matched within
/// tolerance. It is `false` both for divergence AND for unevaluated
/// channels (unknown observed value, or the mission commanded nothing on
/// that channel); [`ShadowEntry::divergences`] disambiguates — only real
/// divergences get reasons.
#[derive(Debug, Clone, PartialEq)]
pub struct ShadowEntry {
    /// Caller-assigned sample ordering (e.g. snapshot counter or timestamp).
    pub sample_seq: u64,
    /// Mission phase the decision was replayed for.
    pub phase: MissionPhase,
    /// What autonomy WOULD have commanded this tick.
    pub intended: MissionCommands,
    /// What the autopilot actually had selected.
    pub observed: ObservedApTargets,
    /// Per-channel match verdicts, indexed by [`chan`].
    pub matches: [bool; chan::COUNT],
    /// Human-readable reasons, one per diverged channel (in [`chan`] order).
    pub divergences: Vec<String>,
    /// High-level intent derived from this tick's intended commands
    /// (single decision source — see [`crate::intents`]). `None` when the
    /// mission issues no flight-guidance intent this tick.
    pub intent: Option<HighLevelIntent>,
    /// The active intent's reason token, when an intent was derived
    /// (spec §33: every classified entry carries the active reason).
    pub reason: Option<String>,
}

impl ShadowEntry {
    /// Classify channel `i` for this entry (spec §25): Match / Divergence
    /// / Unknown (observed absent) / NotComparable (mission silent).
    /// Unknown and NotComparable never count as matches and are excluded
    /// from the match-ratio denominator.
    pub fn classify_channel(&self, i: usize) -> ChannelClass {
        let pairs = self.observed.pairs(&self.intended);
        let (intended, observed) = pairs[i];
        match (intended, observed) {
            (None, _) => ChannelClass::NotComparable,
            (Some(_), None) => ChannelClass::Unknown,
            (Some(w), Some(g)) => {
                if within_tolerance(i, w, g) {
                    ChannelClass::Match
                } else {
                    ChannelClass::Divergence
                }
            }
        }
    }
}

/// Per-channel comparison classification (spec §25).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ChannelClass {
    /// Both sides present, within tolerance.
    Match,
    /// Both sides present, outside tolerance.
    Divergence,
    /// Observed side unknown: absence of evidence — excluded from the
    /// accuracy denominator, never a match.
    Unknown,
    /// The mission intends nothing on this channel: nothing to compare.
    NotComparable,
}

/// Running per-channel comparison statistics.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct ChannelStats {
    /// Channels where both intended and observed were present.
    pub evaluated: usize,
    /// Evaluated channels within tolerance.
    pub matched: usize,
    /// Evaluated channels outside tolerance.
    pub diverged: usize,
}

impl ChannelStats {
    /// Fraction of evaluated channels that matched. `None` when nothing was
    /// ever evaluated on this channel (no evidence at all).
    pub fn match_ratio(&self) -> Option<f64> {
        if self.evaluated == 0 {
            None
        } else {
            Some(self.matched as f64 / self.evaluated as f64)
        }
    }
}

/// Aggregate view over all recorded samples.
#[derive(Debug, Clone, PartialEq)]
pub struct ShadowReport {
    /// All entries, in observation order.
    pub entries: Vec<ShadowEntry>,
    /// Per-channel statistics, indexed by [`chan`].
    pub channels: [ChannelStats; chan::COUNT],
    /// Total divergence reasons across every entry.
    pub divergences_count: usize,
    /// Full-flight aggregate (states, intents, classification totals,
    /// ambiguous transitions) over [`ShadowReport::entries`].
    pub summary: ShadowSummary,
}

/// Shadow Mode recorder. Consumes per-sample (phase, context, observed AP
/// targets) triples, replays the pure controller decision, and records the
/// comparison. Holds NO write path by construction — see module docs.
#[derive(Debug, Clone, Default)]
pub struct MissionShadow {
    entries: Vec<ShadowEntry>,
    channels: [ChannelStats; chan::COUNT],
    divergences_count: usize,
}

impl MissionShadow {
    pub fn new() -> Self {
        Self::default()
    }

    /// Record one shadow sample. `sample_seq` is caller-assigned ordering
    /// (the shadow never invents or renumbers it). Read-only w.r.t. any
    /// flight state: this computes intent and compares — nothing more.
    pub fn observe(
        &mut self,
        sample_seq: u64,
        phase: MissionPhase,
        ctx: &MissionContext<'_>,
        params: &MissionParameters,
        observed: ObservedApTargets,
    ) {
        let intended = intended_commands(&phase, ctx, params);
        // Same tick output the controller acts on → the intent can never
        // disagree with the recorded commands (single decision source).
        let intent = intent_from_tick(&phase, &intended, ctx, params);
        let reason = intent.as_ref().map(|i| i.reason().0.clone());

        let mut matches = [false; chan::COUNT];
        let mut divergences = Vec::new();
        for (i, (want, got)) in intended_pairs(&intended, &observed).into_iter().enumerate() {
            let (Some(want), Some(got)) = (want, got) else {
                continue; // not comparable evidence: excluded, never divergent
            };
            self.channels[i].evaluated += 1;
            if within_tolerance(i, want, got) {
                matches[i] = true;
                self.channels[i].matched += 1;
            } else {
                self.channels[i].diverged += 1;
                self.divergences_count += 1;
                divergences.push(format!(
                    "{}: intended {:.1} {} vs observed {:.1} {} exceeds +/-{} {} tolerance",
                    CHANNEL_NAMES[i], want, UNITS[i], got, UNITS[i], TOLERANCES[i], UNITS[i],
                ));
            }
        }

        self.entries.push(ShadowEntry {
            sample_seq,
            phase,
            intended,
            observed,
            matches,
            divergences,
            intent,
            reason,
        });
    }

    /// Recorded entries in observation order.
    pub fn entries(&self) -> &[ShadowEntry] {
        &self.entries
    }

    /// Number of recorded samples.
    pub fn len(&self) -> usize {
        self.entries.len()
    }

    /// Whether no samples were recorded (a valid, meaningful state).
    pub fn is_empty(&self) -> bool {
        self.entries.is_empty()
    }

    /// Aggregate report over every recorded sample. The per-channel
    /// statistics are the running totals accumulated by [`Self::observe`];
    /// `match_ratio()` is `None` for channels with no evaluated evidence.
    pub fn report(&self) -> ShadowReport {
        ShadowReport {
            entries: self.entries.clone(),
            channels: self.channels,
            divergences_count: self.divergences_count,
            summary: self.summary(),
        }
    }

    /// Full-flight summary over every recorded sample (contract C11
    /// spirit): phase traversal counts, intents by variant, four-way
    /// classification totals, and ambiguous phase boundaries.
    pub fn summary(&self) -> ShadowSummary {
        ShadowSummary::from_entries(&self.entries)
    }
}

/// Full-flight shadow aggregate (contract C11 spirit): what phases were
/// traversed, which intents autonomy emitted, four-way classification
/// totals across every entry × channel, and how many phase boundaries left
/// no comparable evidence.
#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct ShadowSummary {
    /// Recorded ticks per mission phase (keyed by [`MissionPhase::name`]).
    pub states_traversed: BTreeMap<String, usize>,
    /// Emitted intents counted by variant ([`HighLevelIntent::variant_name`]).
    pub intents_emitted: BTreeMap<String, usize>,
    /// Evaluated channels within tolerance.
    pub matches: usize,
    /// Evaluated channels outside tolerance.
    pub divergences: usize,
    /// Channels whose observed value was unknown (absence of evidence).
    pub unknown: usize,
    /// Channels on which the mission was silent (nothing to compare).
    pub not_comparable: usize,
    /// Phase boundaries with zero comparable evidence — see
    /// [`ambiguous_boundary`].
    pub ambiguous_transitions: usize,
}

impl ShadowSummary {
    /// Aggregate a full-flight summary over recorded entries.
    pub fn from_entries(entries: &[ShadowEntry]) -> Self {
        let mut summary = Self::default();
        let mut prev_phase: Option<MissionPhase> = None;
        for entry in entries {
            *summary
                .states_traversed
                .entry(entry.phase.name().to_string())
                .or_default() += 1;
            if let Some(intent) = &entry.intent {
                *summary
                    .intents_emitted
                    .entry(intent.variant_name().to_string())
                    .or_default() += 1;
            }
            for i in 0..chan::COUNT {
                match entry.classify_channel(i) {
                    ChannelClass::Match => summary.matches += 1,
                    ChannelClass::Divergence => summary.divergences += 1,
                    ChannelClass::Unknown => summary.unknown += 1,
                    ChannelClass::NotComparable => summary.not_comparable += 1,
                }
            }
            if prev_phase.is_some_and(|p| p != entry.phase) && ambiguous_boundary(entry) {
                summary.ambiguous_transitions += 1;
            }
            prev_phase = Some(entry.phase);
        }
        summary
    }
}

/// A phase boundary (an entry whose `phase` differs from the previous
/// recorded entry's) is *ambiguous* iff NO channel carried comparable
/// evidence (`Match` or `Divergence`) on that first entry of the new phase:
/// every channel was Unknown/NotComparable, so the transition happened
/// entirely inside an evidence gap and left no verifiable trace.
fn ambiguous_boundary(first_entry_of_new_phase: &ShadowEntry) -> bool {
    (0..chan::COUNT).all(|i| {
        !matches!(
            first_entry_of_new_phase.classify_channel(i),
            ChannelClass::Match | ChannelClass::Divergence
        )
    })
}

/// Autonomy observability counters (spec §34): plain counts of what the
/// mission/shadow pipeline did — deliberately NOT a score. The shadow-
/// derived fields fill from [`MissionShadow::summary`];
/// `capability_missing_events` and `route_unknown_ticks` are app-side
/// counters the integration layer increments as those events occur.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct AutonomyObservability {
    /// Mission phase transitions observed in the shadow timeline.
    pub mission_transitions: u64,
    /// Ticks for which a high-level intent was derived.
    pub intents_emitted: u64,
    /// Shadow channel comparisons that matched within tolerance.
    pub shadow_matches: u64,
    /// Shadow channel comparisons that diverged.
    pub shadow_divergences: u64,
    /// Shadow channels with unknown observed values.
    pub shadow_unknown: u64,
    /// Shadow channels where the mission was silent.
    pub shadow_not_comparable: u64,
    /// Times a requested capability was unavailable (app-filled).
    pub capability_missing_events: u64,
    /// Ticks with unknown route state (app-filled).
    pub route_unknown_ticks: u64,
}

impl AutonomyObservability {
    /// Fill the shadow-derived counters from a recorder; the two app-side
    /// counters start at zero.
    pub fn from_shadow(shadow: &MissionShadow) -> Self {
        let summary = shadow.summary();
        let transitions = shadow
            .entries()
            .windows(2)
            .filter(|w| w[0].phase != w[1].phase)
            .count() as u64;
        Self {
            mission_transitions: transitions,
            intents_emitted: summary.intents_emitted.values().sum::<usize>() as u64,
            shadow_matches: summary.matches as u64,
            shadow_divergences: summary.divergences as u64,
            shadow_unknown: summary.unknown as u64,
            shadow_not_comparable: summary.not_comparable as u64,
            ..Self::default()
        }
    }

    /// Record one capability-miss event.
    pub const fn record_capability_missing(&mut self) {
        self.capability_missing_events += 1;
    }

    /// Record one route-unknown tick.
    pub const fn record_route_unknown_tick(&mut self) {
        self.route_unknown_ticks += 1;
    }
}

/// Channel pairing shared between recording and introspection, so field
/// order stays defined in exactly one place per direction.
fn intended_pairs(
    intended: &MissionCommands,
    observed: &ObservedApTargets,
) -> [(Option<f64>, Option<f64>); chan::COUNT] {
    observed.pairs(intended)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::controller::{MissionController, intended_next_phase};
    use crate::route::{RouteFollower, Waypoint};
    use fd_core::adapter::FlightControlTargets;
    use fd_core::telemetry::{SimState, SimTimestamp, TelemetrySnapshot};
    use fd_core::units::{AltitudeAglFt, AltitudeFt, SpeedKt};

    /// Null write sink used ONLY to drive a real MissionController through
    /// its public step() so the shadow follows an actual controller-driven
    /// flight. The shadow itself never sees this type.
    struct NullTargets;
    impl FlightControlTargets for NullTargets {
        fn flight_guidance_supported(&self) -> bool {
            true
        }
        fn set_target_altitude(&mut self, _v: f64) {}
        fn set_target_speed(&mut self, _v: f64) {}
        fn set_target_heading(&mut self, _v: f64) {}
        fn set_target_vertical_speed(&mut self, _v: f64) {}
    }

    fn snap(alt_ft: f64, agl_ft: f64, ias: f64, on_ground: bool) -> TelemetrySnapshot {
        let mut s = TelemetrySnapshot::empty(SimTimestamp::new(0));
        s.altitude_msl = Some(AltitudeFt::new(alt_ft));
        s.altitude_agl = Some(AltitudeAglFt::new(agl_ft));
        s.indicated_airspeed = Some(SpeedKt::new(ias));
        s.on_ground = Some(on_ground);
        s.sim_timing.state = SimState::Running;
        s
    }

    fn route() -> RouteFollower {
        let wpts = vec![
            Waypoint {
                id: "UUEE".into(),
                lat_deg: 55.972642,
                lon_deg: 37.414589,
            },
            Waypoint {
                id: "ULLI".into(),
                lat_deg: 59.800278,
                lon_deg: 30.2625,
            },
        ];
        RouteFollower::new(wpts, 5.0)
    }

    /// Deterministic mid-route context: far from top of descent, constant
    /// bearing — a stable climb segment.
    fn ctx<'a>(snap: &'a TelemetrySnapshot) -> MissionContext<'a> {
        MissionContext {
            snapshot: snap,
            distance_to_destination_nm: 400.0,
            bearing_to_waypoint_deg: 45.0,
        }
    }

    fn observed_from(cmds: &MissionCommands) -> ObservedApTargets {
        ObservedApTargets {
            heading_deg: cmds.set_target_heading_deg,
            altitude_ft: cmds.set_target_altitude_ft,
            vertical_speed_fpm: cmds.set_target_vertical_speed_fpm,
            speed_kt: cmds.set_target_speed_kt,
        }
    }

    /// (a) A shadow following a controller-driven climb matches 1.0 on every
    /// commanded channel; silent channels stay out of the denominator.
    #[test]
    fn shadow_of_controller_driven_climb_matches_commanded_channels() {
        let params = MissionParameters::default();
        let mut controller = MissionController::new(params.clone());
        let mut shadow = MissionShadow::new();

        let mut route = route();
        // Deterministic climb segment: airborne entry -> takeoff -> climb.
        let agls = [1_600.0, 1_700.0, 1_800.0, 1_900.0];
        for (seq, agl) in agls.into_iter().enumerate() {
            let s = snap(622.0 + agl, agl, 300.0, false);
            let c = ctx(&s);
            let phase_before = controller.phase();
            let cmds = controller.step(&c, &mut NullTargets, &mut route);

            // The AP dutifully follows: observed == commanded exactly.
            shadow.observe(seq as u64, phase_before, &c, &params, observed_from(&cmds));

            assert_eq!(
                cmds,
                crate::controller::intended_commands(&phase_before, &c, &params),
                "step output must equal the pure intent it replays"
            );
        }
        assert_eq!(controller.phase(), MissionPhase::Climb);
        assert_eq!(shadow.len(), 4);

        let report = shadow.report();
        assert_eq!(report.divergences_count, 0);
        assert_eq!(report.channels[chan::ALTITUDE].match_ratio(), Some(1.0));
        assert_eq!(report.channels[chan::SPEED].match_ratio(), Some(1.0));
        assert_eq!(report.channels[chan::HEADING].match_ratio(), Some(1.0));
        // Climb commands carry no vertical speed: never evaluated, no fake 0.
        assert_eq!(report.channels[chan::VERTICAL_SPEED].match_ratio(), None);
        assert_eq!(report.channels[chan::VERTICAL_SPEED].evaluated, 0);
    }

    /// (b) A user dialing different values produces divergence entries with
    /// reasons, per-channel ratios below 1.0, and matching channels still count.
    #[test]
    fn divergent_observed_ap_yields_divergence_entries_with_reasons() {
        let params = MissionParameters::default();
        let s = snap(20_000.0, 19_400.0, 290.0, false);
        let c = ctx(&s);
        let mut shadow = MissionShadow::new();

        // User dialed 18,000 ft / 250 kt / 046 deg; VS unreported.
        shadow.observe(
            7,
            MissionPhase::Climb,
            &c,
            &params,
            ObservedApTargets {
                heading_deg: Some(46.0),
                altitude_ft: Some(18_000.0),
                vertical_speed_fpm: None,
                speed_kt: Some(250.0),
            },
        );

        let entry = &shadow.entries()[0];
        assert_eq!(entry.sample_seq, 7);
        assert_eq!(entry.phase, MissionPhase::Climb);
        assert_eq!(entry.intended.set_target_altitude_ft, Some(34_000.0));
        assert!(!entry.matches[chan::ALTITUDE]);
        assert!(!entry.matches[chan::SPEED]);
        assert!(entry.matches[chan::HEADING], "1 deg off must match");
        assert!(!entry.matches[chan::VERTICAL_SPEED], "unknown != mismatch");
        assert_eq!(entry.divergences.len(), 2);
        assert!(entry.divergences.iter().any(|d| d.contains("altitude")));
        assert!(entry.divergences.iter().any(|d| d.contains("speed")));
        assert!(!entry.divergences.iter().any(|d| d.contains("vertical")));

        let report = shadow.report();
        assert_eq!(report.divergences_count, 2);
        assert_eq!(report.channels[chan::ALTITUDE].match_ratio(), Some(0.0));
        assert_eq!(report.channels[chan::ALTITUDE].diverged, 1);
        assert_eq!(report.channels[chan::SPEED].match_ratio(), Some(0.0));
        assert_eq!(report.channels[chan::HEADING].match_ratio(), Some(1.0));
        assert_eq!(report.channels[chan::VERTICAL_SPEED].match_ratio(), None);
    }

    /// (c) Fully unknown observed AP: no divergence, no match either —
    /// every ratio denominator stays empty.
    #[test]
    fn unknown_observed_ap_yields_no_divergence_and_no_match() {
        let params = MissionParameters::default();
        let s = snap(20_000.0, 19_400.0, 290.0, false);
        let c = ctx(&s);
        let mut shadow = MissionShadow::new();

        shadow.observe(
            1,
            MissionPhase::Climb,
            &c,
            &params,
            ObservedApTargets::default(),
        );

        let entry = &shadow.entries()[0];
        assert_eq!(entry.matches, [false; chan::COUNT]);
        assert!(entry.divergences.is_empty());
        let report = shadow.report();
        assert_eq!(report.divergences_count, 0);
        for stats in &report.channels {
            assert_eq!(stats.evaluated, 0);
            assert_eq!(stats.matched, 0);
            assert_eq!(stats.diverged, 0);
            assert_eq!(stats.match_ratio(), None);
        }
    }

    /// (d) A shadow with zero entries is valid and reports empty evidence.
    #[test]
    fn zero_entry_shadow_is_valid() {
        let shadow = MissionShadow::new();
        assert!(shadow.is_empty());
        assert_eq!(shadow.len(), 0);
        assert!(shadow.entries().is_empty());

        let report = shadow.report();
        assert!(report.entries.is_empty());
        assert_eq!(report.divergences_count, 0);
        assert!(report.channels.iter().all(|s| s.match_ratio().is_none()));
    }

    /// Heading comparison is angular-aware across north (359 deg vs 1 deg
    /// matches; 180 deg apart does not).
    #[test]
    fn heading_tolerance_wraps_across_north() {
        assert_eq!(heading_delta_deg(359.0, 1.0), 2.0);
        assert!(within_tolerance(chan::HEADING, 359.0, 1.0));
        assert!(!within_tolerance(chan::HEADING, 0.0, 180.0));
    }

    /// Phase progression derived read-only agrees with the mutating
    /// controller, proving the shadow replays the identical decision.
    #[test]
    fn intended_next_phase_tracks_controller_progression() {
        let params = MissionParameters::default();
        let mut controller = MissionController::new(params.clone());
        let mut route = route();

        let s0 = snap(622.0 + 1_600.0, 1_600.0, 300.0, false);
        let c0 = ctx(&s0);
        let p0 = controller.phase();
        controller.step(&c0, &mut NullTargets, &mut route);
        assert_eq!(
            intended_next_phase(&p0, &c0, &params),
            Some(MissionPhase::Takeoff)
        );
        assert_eq!(controller.phase(), MissionPhase::Takeoff);

        let s1 = snap(622.0 + 1_700.0, 1_700.0, 300.0, false);
        let c1 = ctx(&s1);
        let p1 = controller.phase();
        controller.step(&c1, &mut NullTargets, &mut route);
        assert_eq!(
            intended_next_phase(&p1, &c1, &params),
            Some(MissionPhase::Climb)
        );
        assert_eq!(controller.phase(), MissionPhase::Climb);

        // Mid-climb with no transition trigger: pure replay says "stay".
        let s2 = snap(622.0 + 1_800.0, 1_800.0, 300.0, false);
        let c2 = ctx(&s2);
        assert_eq!(
            intended_next_phase(&MissionPhase::Climb, &c2, &params),
            None
        );
        assert_eq!(controller.phase(), MissionPhase::Climb);
    }
}

#[cfg(test)]
mod classification_tests {
    use super::*;

    #[test]
    fn classification_is_four_way() {
        // Build via observe to reuse the real pairing logic.
        let mut sh = MissionShadow::default();
        let ctx = test_context(10_000.0, 250.0, 45.0, 500.0, 0.0);
        // Cruise hold intends the cruise altitude (default 34000):
        // observed 34000 (match), 18000 (divergence), None (unknown).
        sh.observe(
            1,
            MissionPhase::Cruise,
            &ctx,
            &MissionParameters::default(),
            ObservedApTargets {
                altitude_ft: Some(34_000.0),
                speed_kt: None,
                heading_deg: None,
                vertical_speed_fpm: None,
            },
        );
        sh.observe(
            2,
            MissionPhase::Cruise,
            &ctx,
            &MissionParameters::default(),
            ObservedApTargets {
                altitude_ft: Some(18_000.0),
                speed_kt: None,
                heading_deg: None,
                vertical_speed_fpm: None,
            },
        );
        sh.observe(
            3,
            MissionPhase::Cruise,
            &ctx,
            &MissionParameters::default(),
            ObservedApTargets {
                altitude_ft: None,
                speed_kt: None,
                heading_deg: None,
                vertical_speed_fpm: None,
            },
        );
        let e = sh.entries();
        assert_eq!(e[0].classify_channel(chan::ALTITUDE), ChannelClass::Match);
        assert_eq!(
            e[1].classify_channel(chan::ALTITUDE),
            ChannelClass::Divergence
        );
        assert_eq!(e[2].classify_channel(chan::ALTITUDE), ChannelClass::Unknown);
        // Mission-silent channel (no VS command in cruise hold) is
        // NotComparable regardless of observation.
        assert_eq!(
            e[0].classify_channel(chan::VERTICAL_SPEED),
            ChannelClass::NotComparable
        );
        // Unknown never counts as match and stays out of the denominator.
        let report = sh.report();
        let alt = &report.channels[chan::ALTITUDE];
        assert_eq!(alt.evaluated, 2, "unknown excluded from denominator");
        assert_eq!(alt.matched, 1);
        assert_eq!(alt.diverged, 1);
    }

    fn test_context(alt: f64, ias: f64, hdg: f64, dist: f64, _vs: f64) -> MissionContext<'static> {
        let snap = Box::leak(Box::new(fd_core::telemetry::TelemetrySnapshot {
            timestamp: fd_core::telemetry::SimTimestamp::new(0),
            position: None,
            altitude_msl: Some(fd_core::units::AltitudeFt::new(alt)),
            altitude_agl: None,
            groundspeed: Some(fd_core::units::SpeedKt::new(ias)),
            indicated_airspeed: Some(fd_core::units::SpeedKt::new(ias)),
            vertical_speed: Some(fd_core::units::VerticalSpeedFpm::new(0.0)),
            heading_true: Some(fd_core::units::AngleDeg::new(hdg)),
            pitch: None,
            bank: None,
            on_ground: Some(false),
            gear_handle_down: None,
            flaps_handle_index: None,
            engine_combustion: None,
            autopilot_master: None,
            autothrottle_arm: None,
            beacon_light: None,
            aircraft_values: Default::default(),
            channel_quality: Default::default(),
            sim_timing: fd_core::telemetry::SimTiming::default(),
        }));
        MissionContext {
            snapshot: snap,
            distance_to_destination_nm: dist,
            bearing_to_waypoint_deg: 0.0,
        }
    }
}

#[cfg(test)]
mod shadow_v2_tests {
    use super::*;
    use crate::intents::Reason;
    use fd_core::telemetry::{SimState, SimTimestamp, TelemetrySnapshot};
    use fd_core::units::{AltitudeFt, SpeedKt};

    fn snap(alt_msl: f64, ias: f64) -> TelemetrySnapshot {
        let mut s = TelemetrySnapshot::empty(SimTimestamp::new(0));
        s.altitude_msl = Some(AltitudeFt::new(alt_msl));
        s.indicated_airspeed = Some(SpeedKt::new(ias));
        s.on_ground = Some(false);
        s.sim_timing.state = SimState::Running;
        s
    }

    fn ctx(snap: &TelemetrySnapshot, dist_nm: f64) -> MissionContext<'_> {
        MissionContext {
            snapshot: snap,
            distance_to_destination_nm: dist_nm,
            bearing_to_waypoint_deg: 45.0,
        }
    }

    /// Observed AP mirror of the intended commands (a dutiful autopilot).
    fn observed_from(cmds: &MissionCommands) -> ObservedApTargets {
        ObservedApTargets {
            heading_deg: cmds.set_target_heading_deg,
            altitude_ft: cmds.set_target_altitude_ft,
            vertical_speed_fpm: cmds.set_target_vertical_speed_fpm,
            speed_kt: cmds.set_target_speed_kt,
        }
    }

    /// Full timeline: cruise hold → top-of-descent trigger (silent
    /// observed AP) → descent (first tick silent = ambiguous boundary)
    /// → descent with a dutiful AP except one diverged speed dial.
    #[test]
    fn summary_aggregates_states_intents_classifications_and_ambiguity() {
        let params = MissionParameters::default();
        let mut shadow = MissionShadow::new();

        // seq 0: cruise hold, dutiful AP → 3 matches + 1 not-comparable.
        let s0 = snap(34_000.0, 450.0);
        let c0 = ctx(&s0, 400.0);
        shadow.observe(
            0,
            MissionPhase::Cruise,
            &c0,
            &params,
            observed_from(&intended_commands(&MissionPhase::Cruise, &c0, &params)),
        );

        // seq 1: still Cruise but past the descent threshold; observed AP
        // fully unreported → every channel Unknown. Same phase: no boundary.
        let c1 = ctx(&s0, 100.0);
        shadow.observe(
            1,
            MissionPhase::Cruise,
            &c1,
            &params,
            ObservedApTargets::default(),
        );

        // seq 2: Descent begins, observed still unreported → the boundary
        // Cruise→Descent carries zero comparable evidence → ambiguous.
        let s2 = snap(20_000.0, 300.0);
        let c2 = ctx(&s2, 60.0);
        shadow.observe(
            2,
            MissionPhase::Descent,
            &c2,
            &params,
            ObservedApTargets::default(),
        );

        // seq 3: descent with a dutiful AP except speed dialed to 250 kt.
        let intended3 = intended_commands(&MissionPhase::Descent, &c2, &params);
        let mut observed3 = observed_from(&intended3);
        observed3.speed_kt = Some(250.0);
        shadow.observe(3, MissionPhase::Descent, &c2, &params, observed3);

        let summary = shadow.summary();
        assert_eq!(
            summary.states_traversed,
            BTreeMap::from([("cruise".to_string(), 2), ("descent".to_string(), 2)])
        );
        assert_eq!(
            summary.intents_emitted,
            BTreeMap::from([
                ("maintain_altitude".to_string(), 1),
                ("prepare_descent".to_string(), 1),
                ("descend_to".to_string(), 2),
            ])
        );
        // seq0: 3 matched + 1 NC. seq1/seq2: all-unknown (descent commands
        // VS too, so 4 unknown each). seq3: 3 matched + 1 divergence.
        assert_eq!(summary.matches, 6);
        assert_eq!(summary.divergences, 1);
        assert_eq!(summary.unknown, 8);
        assert_eq!(summary.not_comparable, 1);
        assert_eq!(summary.ambiguous_transitions, 1);
    }

    /// A phase boundary WITH comparable evidence is never ambiguous, even
    /// if some channels are unknown at the boundary.
    #[test]
    fn evidenced_boundary_is_not_ambiguous() {
        let params = MissionParameters::default();
        let mut shadow = MissionShadow::new();
        let s = snap(34_000.0, 450.0);
        let cruise_ctx = ctx(&s, 400.0);
        shadow.observe(
            0,
            MissionPhase::Cruise,
            &cruise_ctx,
            &params,
            observed_from(&intended_commands(
                &MissionPhase::Cruise,
                &cruise_ctx,
                &params,
            )),
        );
        // Descent entry with altitude reported correctly, rest unknown:
        // one comparable match ⇒ boundary not ambiguous.
        let descent_ctx = ctx(&s, 60.0);
        let intended = intended_commands(&MissionPhase::Descent, &descent_ctx, &params);
        shadow.observe(
            1,
            MissionPhase::Descent,
            &descent_ctx,
            &params,
            ObservedApTargets {
                altitude_ft: intended.set_target_altitude_ft,
                ..ObservedApTargets::default()
            },
        );
        let summary = shadow.summary();
        // Cruise dutiful entry: alt/speed/heading match (VS silent);
        // Descent entry: altitude match, remaining three unknown.
        assert_eq!(summary.matches, 4); // 3 cruise + 1 descent altitude
        assert_eq!(summary.unknown, 3);
        assert_eq!(summary.not_comparable, 1);
        assert_eq!(summary.ambiguous_transitions, 0);
    }

    /// Spec §33: every classified entry carries the active intent's reason
    /// token when an intent exists — and terminal phases record None
    /// without fabricating a reason.
    #[test]
    fn entries_carry_active_intent_reason() {
        let params = MissionParameters::default();
        let mut shadow = MissionShadow::new();
        let s = snap(34_000.0, 450.0);
        let c = ctx(&s, 100.0); // past TOD → PrepareDescent intent
        shadow.observe(
            9,
            MissionPhase::Cruise,
            &c,
            &params,
            ObservedApTargets::default(),
        );

        let entry = &shadow.entries()[0];
        let intent = entry.intent.as_ref().expect("TOD tick must have intent");
        assert_eq!(intent.variant_name(), "prepare_descent");
        assert_eq!(
            entry.reason.as_deref(),
            Some("because distance_to_destination_nm<=descent_distance_nm")
        );
        assert_eq!(entry.reason.as_deref(), Some(intent.reason().as_str()));

        // Ground/terminal ticks: no intent, no fabricated reason.
        let ground_snap = fd_core::telemetry::TelemetrySnapshot::empty(SimTimestamp::new(1));
        let ground_ctx = ctx(&ground_snap, 0.0);
        shadow.observe(
            10,
            MissionPhase::Landing,
            &ground_ctx,
            &params,
            ObservedApTargets::default(),
        );
        let landing_entry = &shadow.entries()[1];
        assert_eq!(landing_entry.intent, None);
        assert_eq!(landing_entry.reason, None);
    }

    /// Spec §34 counters fill from the shadow; app-side counters increment
    /// explicitly and stay plain counts (no score anywhere).
    #[test]
    fn autonomy_observability_counts_shadow_and_app_events() {
        let params = MissionParameters::default();
        let mut shadow = MissionShadow::new();
        let s = snap(34_000.0, 450.0);
        let cruise_ctx = ctx(&s, 400.0);
        shadow.observe(
            0,
            MissionPhase::Cruise,
            &cruise_ctx,
            &params,
            observed_from(&intended_commands(
                &MissionPhase::Cruise,
                &cruise_ctx,
                &params,
            )),
        );
        let descent_ctx = ctx(&s, 60.0);
        shadow.observe(
            1,
            MissionPhase::Descent,
            &descent_ctx,
            &params,
            ObservedApTargets::default(),
        );

        let mut obs = AutonomyObservability::from_shadow(&shadow);
        assert_eq!(obs.mission_transitions, 1);
        assert_eq!(obs.intents_emitted, 2);
        assert_eq!(obs.shadow_matches, 3); // cruise dutiful tick
        assert_eq!(obs.shadow_divergences, 0);
        assert_eq!(obs.shadow_unknown, 4); // descent entry, nothing reported
        assert_eq!(obs.shadow_not_comparable, 1); // cruise VS silence
        assert_eq!(obs.capability_missing_events, 0);
        assert_eq!(obs.route_unknown_ticks, 0);

        obs.record_capability_missing();
        obs.record_route_unknown_tick();
        obs.record_route_unknown_tick();
        assert_eq!(obs.capability_missing_events, 1);
        assert_eq!(obs.route_unknown_ticks, 2);
    }

    /// Zero-write invariant, runtime level: `observe` is a pure function of
    /// its inputs — caller-owned inputs come back untouched, repeated
    /// observation of identical inputs yields identical reports, and the
    /// derived intent/reason are plain data on the entry. Combined with the
    /// compile-level argument in the module docs (this module imports no
    /// adapter or action type), no [`MissionShadow`] method can reach a
    /// dispatch path: there is none to reach.
    #[test]
    fn observe_writes_nothing_outside_the_shadow() {
        let params = MissionParameters::default();
        let s = snap(34_000.0, 450.0);
        let dist_before = 400.0;
        let bearing_before = 45.0;
        let alt_before = s.altitude_msl.map(|v| v.value());
        let mut c = ctx(&s, dist_before);
        let observed = ObservedApTargets {
            altitude_ft: Some(33_000.0),
            ..ObservedApTargets::default()
        };

        let mut shadow = MissionShadow::new();
        shadow.observe(1, MissionPhase::Cruise, &c, &params, observed);

        // Caller-owned inputs unchanged (no hidden mutation of context).
        assert_eq!(c.distance_to_destination_nm, dist_before);
        assert_eq!(c.bearing_to_waypoint_deg, bearing_before);
        assert_eq!(c.snapshot.altitude_msl.map(|v| v.value()), alt_before);

        // Deterministic replay: identical inputs → identical report.
        let mut replay = MissionShadow::new();
        replay.observe(1, MissionPhase::Cruise, &c, &params, observed);
        assert_eq!(shadow.report(), replay.report());

        // Recorded intent is inert data, not an action.
        let intent = shadow.entries()[0].intent.clone().unwrap();
        assert_eq!(
            intent,
            crate::intents::HighLevelIntent::MaintainAltitude {
                target_ft: 34_000.0,
                reason: Reason("because altitude_at_target".into()),
            }
        );
    }
}
