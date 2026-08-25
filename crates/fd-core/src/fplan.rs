//! Canonical, simulator-independent flight plan and FMS observation model
//! (Task 7 §9-13).
//!
//! This module is the shared contract between transports (X-Plane FMS
//! bridge, OpenAIRAC correlation, scenarios, replay) and consumers (route
//! monitoring, Mission Shadow, debrief, future crew tooling).
//!
//! Design rules:
//! - X-Plane structures are NOT mirrored 1:1; only observation-relevant
//!   state is modeled (§9).
//! - Unknown stays unknown: every optional field is `None` when the
//!   transport did not report it. An unknown FMS never becomes a guessed
//!   route (§50).
//! - Every plan knows where it came from ([`FlightPlanSource`], §10) and
//!   never pretends two sources agree.
//! - Revisions are deterministic: [`FmsSnapshot::revision_hash`] is a pure
//!   function of the observed plan content, so change detection emits
//!   nothing when nothing changed (§12).

use serde::{Deserialize, Serialize};

use crate::telemetry::Position;

/// Where a flight plan came from (Task 7 §10).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum FlightPlanSource {
    /// Operator typed it into FlightdeckOS.
    UserProvided,
    /// Correlated against the OpenAIRAC world store.
    OpenAirac,
    /// Read from the simulator's own FMS/GPS.
    XPlaneFms,
    /// Read from an aircraft addon's avionics.
    AircraftAddon,
    /// Deterministic scenario definition.
    Scenario,
    /// Origin not established.
    Unknown,
}

/// Which logical plan of a nav device an [`FmsPlan`] describes (§11, §13).
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub enum PlanKind {
    /// Enroute / active plan.
    Primary,
    /// Approach plan, when the device provides one.
    Approach,
    /// Temporary plan (FMS devices), when provided.
    Temporary,
}

impl PlanKind {
    /// Stable wire/tag name.
    pub fn as_str(self) -> &'static str {
        match self {
            PlanKind::Primary => "primary",
            PlanKind::Approach => "approach",
            PlanKind::Temporary => "temporary",
        }
    }
}

/// Kind of one FMS entry, from the transport's nav-type report.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum FmsEntryKind {
    Airport,
    Vor,
    Ndb,
    Fix,
    /// User lat/lon waypoint with no nav-database identity.
    LatLon,
    /// Transport reported an entry but not a recognized type.
    Unknown,
}

/// One waypoint entry of an observed FMS plan (§11).
///
/// `altitude_constraint_ft` is feet MSL as reported by X-Plane FMS APIs.
/// X-Plane exposes NO speed constraints (SDK 4.3.0 headers); the field is
/// deliberately absent instead of always-`None`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct FmsEntry {
    pub kind: FmsEntryKind,
    /// Waypoint identifier as shown by the FMS (`None` = not reported).
    #[serde(default)]
    pub id: Option<String>,
    #[serde(default)]
    pub lat_deg: Option<f64>,
    #[serde(default)]
    pub lon_deg: Option<f64>,
    /// Altitude constraint, feet MSL.
    #[serde(default)]
    pub altitude_constraint_ft: Option<i32>,
    /// True when the transport resolved a nav-database reference for this
    /// entry. X-Plane resolves nav refs asynchronously (~1 s); `false`
    /// means "not (yet) resolved", NOT "not a nav waypoint".
    #[serde(default)]
    pub nav_ref_resolved: bool,
}

/// One observed plan of one nav device (§11).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct FmsPlan {
    /// Ordered entries. Empty = empty plan (which is distinct from
    /// "device has no such plan" — that plan is simply absent from the
    /// snapshot's map, §13).
    pub entries: Vec<FmsEntry>,
    /// Index of the entry the device is flying TO (destination), when
    /// reported.
    #[serde(default)]
    pub destination_entry: Option<usize>,
    /// Index of the entry the pilot is viewing, when reported.
    #[serde(default)]
    pub displayed_entry: Option<usize>,
}

impl FmsPlan {
    /// Deterministic content hash (FNV-1a 64) over entries + destination
    /// index. Pure function of observed state (§12).
    pub fn revision_hash(&self) -> u64 {
        let mut h: u64 = 0xcbf2_9ce4_8422_2325;
        fn feed(h: &mut u64, bytes: &[u8]) {
            for b in bytes {
                *h ^= u64::from(*b);
                *h = h.wrapping_mul(0x0000_0100_0000_01b3);
            }
        }
        for e in &self.entries {
            feed(&mut h, &[e.kind as u8]);
            feed(&mut h, e.id.as_deref().unwrap_or("").as_bytes());
            feed(
                &mut h,
                &e.lat_deg.map(f64::to_bits).unwrap_or(0).to_le_bytes(),
            );
            feed(
                &mut h,
                &e.lon_deg.map(f64::to_bits).unwrap_or(0).to_le_bytes(),
            );
            feed(
                &mut h,
                &e.altitude_constraint_ft.unwrap_or(-1).to_le_bytes(),
            );
            feed(&mut h, &[u8::from(e.nav_ref_resolved)]);
        }
        feed(
            &mut h,
            &self
                .destination_entry
                .unwrap_or(u64::MAX as usize)
                .to_le_bytes(),
        );
        h
    }

    /// The destination entry, when the index is in bounds.
    pub fn destination(&self) -> Option<&FmsEntry> {
        self.destination_entry.and_then(|i| self.entries.get(i))
    }
}

/// Which nav device produced an [`FmsSnapshot`] (§13).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum FmsDeviceKind {
    /// Stock X-Plane FMS.
    StockFms,
    /// Stock X-Plane GPS (e.g. GNS/G1000).
    StockGps,
    /// Third-party avionics exposing the generic plan surface.
    AddonFms,
    /// Transport did not identify the device.
    Unknown,
}

/// Normalized observational snapshot of a simulator FMS/GPS (§11).
///
/// Answers: how many entries, which is active/next, where is the
/// destination, is an approach loaded, has the plan changed. Unknown
/// fields stay unknown; a plan type the device does not provide is absent
/// from [`plans`] (§13: absence is honest, not an empty plan).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct FmsSnapshot {
    pub device: FmsDeviceKind,
    /// Observed plans by kind. Only plans the device actually provides.
    pub plans: std::collections::BTreeMap<PlanKind, FmsPlan>,
    /// Deterministic content hash over all plans (change detection, §12).
    pub revision_hash: u64,
    /// Human-readable evidence string (transport + provenance, §10).
    pub evidence: String,
}

impl FmsSnapshot {
    /// Build with a computed revision hash.
    pub fn new(
        device: FmsDeviceKind,
        plans: std::collections::BTreeMap<PlanKind, FmsPlan>,
        evidence: impl Into<String>,
    ) -> Self {
        let mut s = Self {
            device,
            plans,
            revision_hash: 0,
            evidence: evidence.into(),
        };
        s.revision_hash = s.content_hash();
        s
    }

    /// Hash over all plans in deterministic (PlanKind-ordered) sequence.
    fn content_hash(&self) -> u64 {
        let mut h: u64 = 0xcbf2_9ce4_8422_2325;
        for (kind, plan) in &self.plans {
            for b in kind.as_str().bytes() {
                h ^= u64::from(b);
                h = h.wrapping_mul(0x0000_0100_0000_01b3);
            }
            h ^= plan.revision_hash();
            h = h.wrapping_mul(0x0000_0100_0000_01b3);
        }
        h
    }

    /// The primary plan, when the device exposes one.
    pub fn primary(&self) -> Option<&FmsPlan> {
        self.plans.get(&PlanKind::Primary)
    }

    /// Is an approach plan present with at least one entry? (§11: there
    /// is no explicit "loaded" flag in X-Plane; count > 0 is the only
    /// observable.)
    pub fn approach_loaded(&self) -> Option<bool> {
        self.plans
            .get(&PlanKind::Approach)
            .map(|p| !p.entries.is_empty())
    }
}

/// Classification of one OpenAIRAC correlation attempt (§14).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum NavMatch {
    /// Identifier AND position agree with exactly one record.
    Matched {
        ident: String,
        lat_deg: f64,
        lon_deg: f64,
        /// "waypoint" | "navaid" | "airport"
        facility: String,
    },
    /// Identifier exists in the store but positions disagree, or several
    /// same-ident records exist and the position cannot disambiguate.
    /// NEVER silently resolved (§50).
    Ambiguous { ident: String, reason: String },
    /// No record with this identifier in the store.
    NotFound { ident: String },
}

impl NavMatch {
    pub fn ident(&self) -> &str {
        match self {
            NavMatch::Matched { ident, .. }
            | NavMatch::Ambiguous { ident, .. }
            | NavMatch::NotFound { ident } => ident,
        }
    }
}

/// Read-only navigation context state (§16). Navigation context, NOT
/// Mission phase — the two are kept separate by construction.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ProcedurePhase {
    Enroute,
    Sid,
    Star,
    Approach,
    MissedApproach,
    Unknown,
}

/// Correlated procedure family (§15).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ProcedureKind {
    Sid,
    Star,
    Approach,
}

impl ProcedureKind {
    pub fn as_str(self) -> &'static str {
        match self {
            ProcedureKind::Sid => "SID",
            ProcedureKind::Star => "STAR",
            ProcedureKind::Approach => "APPROACH",
        }
    }
}

/// Procedure correlation evidence for the observed plan (§15, §17).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ProcedureContext {
    pub kind: ProcedureKind,
    pub procedure_ident: String,
    pub airport_ident: String,
    /// Number of consecutive observed fixes that matched the procedure
    /// sequence.
    pub matched_fixes: usize,
    pub evidence: String,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum FlightPlanChange {
    WaypointInserted {
        index: usize,
    },
    WaypointRemoved {
        index: usize,
    },
    ActiveLegChanged {
        from: Option<usize>,
        to: Option<usize>,
    },
    ApproachLoaded,
    ApproachCleared,
    DestinationChanged {
        from: Option<String>,
        to: Option<String>,
    },
    PlanCleared,
    PlanReplaced,
}

/// Classify the change between two revision states of the primary plan.
///
/// Deterministic; conservative (§12, §17: false Unknown beats false
/// confident). `prev == None` means "no previous observation".
pub fn classify_primary_change(
    prev: Option<(&FmsPlan, Option<usize>)>,
    next: &FmsPlan,
) -> Vec<FlightPlanChange> {
    use FlightPlanChange::*;
    let mut out = Vec::new();
    let Some((prev_plan, prev_dest)) = prev else {
        if next.entries.is_empty() {
            return out;
        }
        out.push(PlanReplaced);
        return out;
    };
    if next.entries.is_empty() && !prev_plan.entries.is_empty() {
        out.push(PlanCleared);
        return out;
    }
    // Active leg / destination.
    if prev_dest != next.destination_entry {
        let from = prev_dest
            .and_then(|i| prev_plan.entries.get(i))
            .and_then(|e| e.id.clone());
        let to = next
            .destination_entry
            .and_then(|i| next.entries.get(i))
            .and_then(|e| e.id.clone());
        out.push(ActiveLegChanged {
            from: prev_dest,
            to: next.destination_entry,
        });
        if from != to {
            out.push(DestinationChanged { from, to });
        }
    }
    // Entry-count deltas: conservative single insert/remove at the first
    // differing index; larger rewrites are PlanReplaced.
    let common = prev_plan.entries.len().min(next.entries.len());
    let first_diff = (0..common).find(|&i| prev_plan.entries[i] != next.entries[i]);
    match (first_diff, prev_plan.entries.len(), next.entries.len()) {
        (None, p, n) if p == n => {}
        (None, p, n) if n > p => out.push(WaypointInserted { index: p }),
        (None, p, n) if n < p => out.push(WaypointRemoved { index: n }),
        (Some(i), p, n) if n == p + 1 && prev_plan.entries[i..] == next.entries[i + 1..] => {
            out.push(WaypointInserted { index: i })
        }
        (Some(i), p, n) if n + 1 == p && prev_plan.entries[i + 1..] == next.entries[i..] => {
            out.push(WaypointRemoved { index: i })
        }
        _ => out.push(PlanReplaced),
    }
    out
}

/// Canonical simulator-independent flight plan (§9).
///
/// Built by the application layer from an [`FmsSnapshot`] (source
/// `XPlaneFms`), OpenAIRAC correlation, operator input, or scenarios.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct FlightPlan {
    pub source: FlightPlanSource,
    #[serde(default)]
    pub origin: Option<String>,
    #[serde(default)]
    pub destination: Option<String>,
    pub legs: Vec<FlightPlanLeg>,
    /// Index of the active leg (leg i goes from legs[i] to legs[i+1]).
    #[serde(default)]
    pub active_leg: Option<usize>,
    /// Monotonic revision counter, bumped on meaningful change (§12).
    pub revision: u64,
    /// Evidence string: where this plan came from and how it was built.
    pub evidence: String,
}

/// One leg of a [`FlightPlan`] (§9).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct FlightPlanLeg {
    pub kind: FmsEntryKind,
    pub identifier: String,
    #[serde(default)]
    pub position: Option<Position>,
    /// Altitude constraint, feet MSL.
    #[serde(default)]
    pub altitude_constraint_ft: Option<i32>,
    #[serde(default)]
    pub airway: Option<String>,
    /// Procedure identifier when this leg belongs to a correlated
    /// procedure (§15).
    #[serde(default)]
    pub procedure_context: Option<String>,
}

#[cfg(test)]
mod tests {
    use super::*;

    fn entry(id: &str, _i: usize) -> FmsEntry {
        // Position derived from the id so insert/remove keep other
        // entries byte-identical (the property under test).
        let seed = id
            .bytes()
            .fold(0u64, |a, b| a.wrapping_mul(31).wrapping_add(u64::from(b)));
        FmsEntry {
            kind: FmsEntryKind::Fix,
            id: Some(id.into()),
            lat_deg: Some(33.0 + (seed % 1000) as f64 / 100.0),
            lon_deg: Some(-118.0 - (seed % 97) as f64 / 100.0),
            altitude_constraint_ft: None,
            nav_ref_resolved: true,
        }
    }

    fn plan(ids: &[&str], dest: Option<usize>) -> FmsPlan {
        FmsPlan {
            entries: ids.iter().enumerate().map(|(i, id)| entry(id, i)).collect(),
            destination_entry: dest,
            displayed_entry: None,
        }
    }

    #[test]
    fn revision_hash_is_content_deterministic_and_sensitive() {
        let a = plan(&["A", "B"], Some(1));
        let b = plan(&["A", "B"], Some(1));
        assert_eq!(a.revision_hash(), b.revision_hash());
        let c = plan(&["A", "C"], Some(1));
        assert_ne!(a.revision_hash(), c.revision_hash());
        let d = plan(&["A", "B"], Some(0));
        assert_ne!(
            a.revision_hash(),
            d.revision_hash(),
            "active leg change must change hash"
        );
    }

    #[test]
    fn snapshot_hash_covers_plan_kinds() {
        let mut plans = std::collections::BTreeMap::new();
        plans.insert(PlanKind::Primary, plan(&["A"], Some(0)));
        let s1 = FmsSnapshot::new(FmsDeviceKind::StockGps, plans.clone(), "t");
        plans.insert(PlanKind::Approach, plan(&["I"], Some(0)));
        let s2 = FmsSnapshot::new(FmsDeviceKind::StockGps, plans, "t");
        assert_ne!(s1.revision_hash, s2.revision_hash);
        assert_eq!(s1.approach_loaded(), None);
        assert_eq!(s2.approach_loaded(), Some(true));
    }

    #[test]
    fn classify_first_observation_and_clear() {
        let p = plan(&["A", "B"], Some(1));
        assert_eq!(
            classify_primary_change(None, &p),
            vec![FlightPlanChange::PlanReplaced]
        );
        let empty = plan(&[], None);
        assert_eq!(classify_primary_change(None, &empty), Vec::new());
        assert_eq!(
            classify_primary_change(Some((&p, Some(1))), &empty),
            vec![FlightPlanChange::PlanCleared]
        );
    }

    #[test]
    fn classify_insert_remove_and_active_leg() {
        let ab = plan(&["A", "B"], Some(1));
        let axb = plan(&["A", "X", "B"], Some(1));
        assert_eq!(
            classify_primary_change(Some((&ab, Some(1))), &axb),
            vec![FlightPlanChange::WaypointInserted { index: 1 }]
        );
        assert_eq!(
            classify_primary_change(Some((&axb, Some(1))), &ab),
            vec![FlightPlanChange::WaypointRemoved { index: 1 }]
        );
        // Same entries, destination moved 1 -> 2.
        let ab2 = plan(&["A", "B"], Some(2));
        let changes = classify_primary_change(Some((&ab, Some(1))), &ab2);
        assert!(changes.contains(&FlightPlanChange::ActiveLegChanged {
            from: Some(1),
            to: Some(2)
        }));
        assert!(changes.contains(&FlightPlanChange::DestinationChanged {
            from: Some("B".into()),
            to: None,
        }));
    }

    #[test]
    fn classify_full_rewrite() {
        let ab = plan(&["A", "B"], Some(1));
        let xy = plan(&["X", "Y"], Some(1));
        assert_eq!(
            classify_primary_change(Some((&ab, Some(1))), &xy),
            vec![FlightPlanChange::PlanReplaced]
        );
    }
}
