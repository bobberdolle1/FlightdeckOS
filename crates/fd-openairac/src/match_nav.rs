//! Conservative OpenAIRAC correlation of observed FMS state (Task 7
//! §14-17).
//!
//! Principles:
//! - Identifier alone matches ONLY when it is unambiguous across every
//!   facility family; position (when reported) must agree.
//! - Facility type from the FMS entry (airport/fix/VOR/NDB) narrows
//!   candidates BEFORE geometry; a type conflict is Ambiguous, never a
//!   silent remap.
//! - `Ambiguous` and `NotFound` are honest outcomes (§50: an ambiguous
//!   nav identifier never silently matches an arbitrary record).
//! - Procedure correlation requires a CONTIGUOUS run of at
//!   [`MIN_PROCEDURE_RUN`] consecutive observed fixes inside the
//!   procedure's fix sequence — false Unknown beats false confident
//!   (§17).

use crate::store::NavDataStore;
use fd_core::fplan::{NavMatch, ProcedureContext, ProcedureKind};

/// A position fix match must be within this distance of the record.
pub const POSITION_TOLERANCE_NM: f64 = 5.0;

/// Two same-ident records closer than this are the SAME physical point
/// (e.g. a waypoint co-located with its VOR).
pub const CO_LOCATION_NM: f64 = 0.5;

/// Minimum contiguous fix run for a procedure correlation.
pub const MIN_PROCEDURE_RUN: usize = 3;

/// Map a raw OpenAIRAC procedure_kind code to the canonical kind.
/// Unknown codes return None (never guessed).
pub fn procedure_kind(code: &str) -> Option<ProcedureKind> {
    match code {
        "D" => Some(ProcedureKind::Sid),
        "E" => Some(ProcedureKind::Star),
        "F" => Some(ProcedureKind::Approach),
        _ => None,
    }
}

/// Which facility families the entry's nav-type admits.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FacilityFilter {
    Airport,
    Waypoint,
    Navaid,
    /// Type not reported (or unrecognized): all families admissible.
    Any,
}

/// Correlate one observed FMS entry against the world store.
pub fn match_entry(
    store: &NavDataStore,
    ident: Option<&str>,
    lat_deg: Option<f64>,
    lon_deg: Option<f64>,
    filter: FacilityFilter,
    at: &str,
) -> NavMatch {
    let Some(ident) = ident else {
        return NavMatch::NotFound {
            ident: String::new(),
        };
    };
    if ident.is_empty() {
        return NavMatch::NotFound {
            ident: ident.to_string(),
        };
    }
    let mut candidates: Vec<(String, &'static str, f64, f64)> = Vec::new();
    let want = |fam: &'static str| -> bool {
        match filter {
            FacilityFilter::Any => true,
            FacilityFilter::Airport => fam == "airport",
            FacilityFilter::Waypoint => fam == "waypoint",
            FacilityFilter::Navaid => fam == "navaid",
        }
    };
    if want("waypoint")
        && let Ok(wps) = store.waypoints_by_ident(ident, at)
    {
        for w in wps {
            candidates.push((w.ident, "waypoint", w.lat_deg, w.lon_deg));
        }
    }
    if want("navaid")
        && let Ok(navs) = store.navaids_by_ident(ident, at)
    {
        for n in navs {
            candidates.push((n.ident, "navaid", n.lat_deg, n.lon_deg));
        }
    }
    if want("airport")
        && let Ok(Some(a)) = store.airport_by_icao(ident, at)
    {
        candidates.push((a.ident, "airport", a.lat_deg, a.lon_deg));
    }
    if candidates.is_empty() {
        return NavMatch::NotFound {
            ident: ident.to_string(),
        };
    }
    let Some((lat, lon)) = lat_deg.zip(lon_deg) else {
        // No observed position: match ONLY when exactly one record of the
        // requested family exists (§14: never match solely by short
        // identifier where ambiguous).
        return if candidates.len() == 1 {
            let (id, fam, la, lo) = candidates[0].clone();
            NavMatch::Matched {
                ident: id,
                lat_deg: la,
                lon_deg: lo,
                facility: fam.to_string(),
            }
        } else {
            NavMatch::Ambiguous {
                ident: ident.to_string(),
                reason: format!("{} records, no observed position", candidates.len()),
            }
        };
    };
    // Position-aware: keep candidates within tolerance.
    let near: Vec<_> = candidates
        .iter()
        .filter(|(_, _, la, lo)| {
            fd_core::geo::distance_nm(lat, lon, *la, *lo) <= POSITION_TOLERANCE_NM
        })
        .cloned()
        .collect();
    if near.is_empty() {
        let nearest = candidates
            .iter()
            .map(|(_, f, la, lo)| (*f, fd_core::geo::distance_nm(lat, lon, *la, *lo)))
            .min_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal));
        return NavMatch::Ambiguous {
            ident: ident.to_string(),
            reason: match nearest {
                Some((fam, d)) => {
                    format!("nearest {fam} {d:.1}nm away (tolerance {POSITION_TOLERANCE_NM}nm)")
                }
                None => "no candidates".into(),
            },
        };
    }
    // Collapse co-located duplicates (waypoint+VOR at the same point are
    // ONE physical fix when the filter admits both).
    let mut distinct: Vec<(String, &'static str, f64, f64)> = Vec::new();
    for c in near {
        let co_located = distinct
            .iter()
            .any(|d| fd_core::geo::distance_nm(c.2, c.3, d.2, d.3) <= CO_LOCATION_NM);
        if !co_located {
            distinct.push(c);
        }
    }
    match distinct.len() {
        1 => {
            let (id, fam, la, lo) = distinct[0].clone();
            NavMatch::Matched {
                ident: id,
                lat_deg: la,
                lon_deg: lo,
                facility: fam.to_string(),
            }
        }
        _ => NavMatch::Ambiguous {
            ident: ident.to_string(),
            reason: format!(
                "{} distinct same-ident locations near the observed position",
                distinct.len()
            ),
        },
    }
}

/// Longest contiguous run of `observed` (in order) appearing in
/// `procedure_fixes`. Returns the run length and the procedure index
/// where the run sits.
fn longest_contiguous_run(observed: &[String], procedure_fixes: &[String]) -> usize {
    if observed.len() < MIN_PROCEDURE_RUN || procedure_fixes.len() < MIN_PROCEDURE_RUN {
        return 0;
    }
    let mut best = 0usize;
    for start in 0..procedure_fixes.len() {
        let mut i = start;
        let mut run = 0usize;
        for o in observed {
            if procedure_fixes.get(i).map(|f| f == o).unwrap_or(false) {
                i += 1;
                run += 1;
            } else if run >= MIN_PROCEDURE_RUN {
                break;
            } else {
                // Allow the observed sequence to skip non-matching
                // procedure fixes only BEFORE a run has started.
                if run == 0 {
                    i += 1;
                    if i >= procedure_fixes.len() {
                        break;
                    }
                }
            }
        }
        if run > best {
            best = run;
        }
    }
    best
}

/// Correlate observed fix identifiers against an airport's procedures.
///
/// Returns contexts with runs >= [`MIN_PROCEDURE_RUN`], best first.
/// Conservative by construction (§17).
pub fn correlate_procedures(
    store: &NavDataStore,
    airport_ident: &str,
    observed_fix_ids: &[String],
    at: &str,
) -> Result<Vec<ProcedureContext>, crate::error::OpenAiracError> {
    let mut out = Vec::new();
    if observed_fix_ids.len() < MIN_PROCEDURE_RUN {
        return Ok(out);
    }
    for proc in store.procedures(airport_ident, at)? {
        let Some(kind) = procedure_kind(&proc.kind_code) else {
            continue;
        };
        let run = longest_contiguous_run(observed_fix_ids, &proc.fixes);
        if run >= MIN_PROCEDURE_RUN {
            out.push(ProcedureContext {
                kind,
                procedure_ident: proc.procedure_ident.clone(),
                airport_ident: airport_ident.to_string(),
                matched_fixes: run,
                evidence: format!(
                    "contiguous run of {run} observed fixes inside {} fix sequence ({} fixes)",
                    proc.procedure_ident,
                    proc.fixes.len()
                ),
            });
        }
    }
    out.sort_by_key(|c| std::cmp::Reverse(c.matched_fixes));
    Ok(out)
}

/// The single best procedure context, when one dominates.
///
/// Ambiguity policy (§17): if the best two contexts have EQUAL run
/// lengths, the result is None (ambiguous) — never an arbitrary pick.
pub fn best_procedure(contexts: &[ProcedureContext]) -> Option<ProcedureContext> {
    match contexts {
        [best] => Some(best.clone()),
        [best, second, ..] if best.matched_fixes > second.matched_fixes => Some(best.clone()),
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fixes(list: &[&str]) -> Vec<String> {
        list.iter().map(|s| s.to_string()).collect()
    }

    #[test]
    fn procedure_kind_mapping() {
        assert_eq!(procedure_kind("D"), Some(ProcedureKind::Sid));
        assert_eq!(procedure_kind("E"), Some(ProcedureKind::Star));
        assert_eq!(procedure_kind("F"), Some(ProcedureKind::Approach));
        assert_eq!(procedure_kind("X"), None);
    }

    #[test]
    fn run_requires_minimum_length() {
        let proc = fixes(&["A", "B", "C", "D", "E"]);
        assert_eq!(longest_contiguous_run(&fixes(&["B", "C"]), &proc), 0);
    }

    #[test]
    fn reversed_order_never_reaches_run_threshold() {
        let proc = fixes(&["A", "B", "C", "D", "E"]);
        // Reversed observed sequence may pick up stray single fixes but
        // must never form a run >= MIN_PROCEDURE_RUN (§17).
        assert!(longest_contiguous_run(&fixes(&["D", "C", "B"]), &proc) < MIN_PROCEDURE_RUN);
        assert!(
            longest_contiguous_run(&fixes(&["E", "D", "C", "B", "A"]), &proc) < MIN_PROCEDURE_RUN
        );
    }

    #[test]
    fn run_allows_procedure_prefix_skip_before_run() {
        // Observed enters the procedure mid-sequence (common: joining a
        // STAR at a transition fix).
        let proc = fixes(&["A", "B", "C", "D", "E"]);
        assert_eq!(
            longest_contiguous_run(&fixes(&["Q", "C", "D", "E"]), &proc),
            3
        );
    }

    #[test]
    fn best_prefers_dominant_run_and_rejects_ties() {
        let a = ProcedureContext {
            kind: ProcedureKind::Star,
            procedure_ident: "A".into(),
            airport_ident: "X".into(),
            matched_fixes: 4,
            evidence: String::new(),
        };
        let b = ProcedureContext {
            matched_fixes: 3,
            procedure_ident: "B".into(),
            ..a.clone()
        };
        assert_eq!(
            best_procedure(&[a.clone(), b]).unwrap().procedure_ident,
            "A"
        );
        let c = ProcedureContext {
            matched_fixes: 4,
            procedure_ident: "C".into(),
            ..a.clone()
        };
        assert!(
            best_procedure(&[a, c]).is_none(),
            "equal runs are ambiguous"
        );
    }
}
