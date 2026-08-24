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

use crate::controller::{
    MissionCommands, MissionContext, MissionParameters, MissionPhase, intended_commands,
};

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
        }
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
