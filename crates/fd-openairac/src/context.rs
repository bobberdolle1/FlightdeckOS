//! Canonical FlightdeckOS Crew Flight Context & Source Ownership Model.
//!
//! Maintains authoritative navigation, phase, descent, weather, and provenance
//! context received from OpenAIRAC, preventing competing calculations.

use crate::types::{OpenAiracCompactSnapshot, OpenAiracSnapshotV2};
use serde::{Deserialize, Serialize};
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SourceOwnershipTable {
    pub navigation_and_route: String,
    pub flight_phase: String,
    pub weather_context: String,
    pub online_atc_context: String,
    pub aircraft_system_state: String,
    pub sop_and_checklist_state: String,
    pub crew_dialogue_and_ai: String,
}

impl Default for SourceOwnershipTable {
    fn default() -> Self {
        Self {
            navigation_and_route: "OpenAIRAC (Authoritative)".to_string(),
            flight_phase: "OpenAIRAC (FlightPhaseEngine)".to_string(),
            weather_context: "OpenAIRAC Gateway".to_string(),
            online_atc_context: "OpenAIRAC Gateway".to_string(),
            aircraft_system_state: "FlightdeckOS (fd-simconnect / fd-aircraft)".to_string(),
            sop_and_checklist_state: "FlightdeckOS (fd-sop)".to_string(),
            crew_dialogue_and_ai: "FlightdeckOS (fd-crew / AI Runtime)".to_string(),
        }
    }
}

/// Independent multi-source freshness indicators.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SubsystemFreshness {
    pub telemetry_status: String,
    pub telemetry_age_ms: u64,
    pub weather_status: String,
    pub weather_age_sec: Option<u64>,
    pub online_atc_status: String,
    pub navdata_status: String,
    pub is_telemetry_stale: bool,
    pub is_weather_stale: bool,
}

/// FlightdeckOS Canonical Crew Flight Context.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CrewFlightContext {
    pub session_id: String,
    pub flight_id: String,
    pub origin_icao: String,
    pub destination_icao: String,
    pub aircraft_type: String,
    pub flight_phase: String,
    pub phase_evidence: String,
    pub altitude_ft: f64,
    pub groundspeed_kts: f64,
    pub vertical_speed_fpm: f64,
    pub track_deg: f64,
    pub active_leg: String,
    pub next_fix: String,
    pub next_constraint: String,
    pub distance_to_next_nm: f64,
    pub remaining_route_nm: f64,
    pub xtk_nm: f64,
    pub xtk_side: String,
    pub is_off_route: bool,
    pub tod_distance_nm: Option<f64>,
    pub descent_profile_status: String,
    pub required_vs_fpm: Option<f64>,
    pub profile_deviation_ft: Option<f64>,
    pub departure_brief_text: String,
    pub arrival_brief_text: String,
    pub star_procedure: Option<String>,
    pub approach_procedure: Option<String>,
    pub approach_type: Option<String>,
    pub is_source_required_at_destination: bool,
    pub destination_weather: String,
    pub online_atc: Vec<String>,
    pub advisories: Vec<String>,
    pub navdata_provenance: String,
    pub freshness: SubsystemFreshness,
    pub ownership: SourceOwnershipTable,
}

impl CrewFlightContext {
    /// Create canonical context from full OpenAIRAC Snapshot v2.
    pub fn from_snapshot_v2(snap: &OpenAiracSnapshotV2) -> Self {
        let pos = snap.position.as_ref();
        let leg = snap.active_leg.as_ref();
        let geom = &snap.navigation_geometry;
        let dprof = &snap.descent_profile;
        let fresh = &snap.freshness_report;

        let active_leg = leg
            .map(|l| {
                format!(
                    "{} ({} | TRK {:.0}°)",
                    l.leg_name, l.leg_type, l.desired_track_deg
                )
            })
            .unwrap_or_else(|| "NONE".to_string());

        let next_fix = leg
            .and_then(|l| l.next_fix.clone())
            .unwrap_or_else(|| snap.destination.ident.clone());

        let next_constraint = snap
            .next_constraint
            .as_ref()
            .map(|c| format!("{}: {}", c.fix_ident, c.constraint_type))
            .unwrap_or_else(|| "NONE".to_string());

        let departure_brief_text = format!(
            "Departure from {} (Elev {:.0} ft), Runway {}, SID {}.",
            snap.origin.ident,
            snap.origin.elevation_ft.unwrap_or(0.0),
            snap.origin.selected_runway.as_deref().unwrap_or("DEFAULT"),
            snap.origin.procedure_name.as_deref().unwrap_or("NONE")
        );

        let arrival_brief_text = if snap.destination.is_source_required {
            format!(
                "Arrival at {}: SOURCE_REQUIRED. No terminal procedures available in open data.",
                snap.destination.ident
            )
        } else {
            format!(
                "Arrival at {} (Elev {:.0} ft), Runway {}, STAR {}, Approach {}.",
                snap.destination.ident,
                snap.destination.elevation_ft.unwrap_or(0.0),
                snap.destination
                    .selected_runway
                    .as_deref()
                    .unwrap_or("DEFAULT"),
                snap.destination
                    .star_procedure
                    .as_deref()
                    .unwrap_or("DIRECT"),
                snap.destination
                    .approach_procedure
                    .as_deref()
                    .unwrap_or("VISUAL")
            )
        };

        let destination_weather = snap
            .weather_summary
            .destination_metar
            .clone()
            .unwrap_or_else(|| "UNAVAILABLE".to_string());

        let online_atc = snap
            .online_atc
            .iter()
            .map(|a| format!("{} [{}] ({})", a.callsign, a.facility_type, a.frequency_mhz))
            .collect();

        let advisories = snap
            .advisories
            .iter()
            .map(|a| format!("[{}] {}: {}", a.level, a.code, a.message))
            .collect();

        let navdata_provenance = format!(
            "{} | AIRAC {}",
            snap.data_provenance.active_provider_datasets.join(", "),
            snap.data_provenance
                .airac_cycle
                .as_deref()
                .unwrap_or("CURRENT")
        );

        let freshness = SubsystemFreshness {
            telemetry_status: fresh.telemetry.status.clone(),
            telemetry_age_ms: fresh.telemetry.age_ms,
            weather_status: fresh.weather.status.clone(),
            weather_age_sec: fresh.weather.age_sec,
            online_atc_status: fresh.online_atc.status.clone(),
            navdata_status: fresh.navdata.status.clone(),
            is_telemetry_stale: snap.stale_flags.telemetry_stale,
            is_weather_stale: snap.weather_summary.weather_stale,
        };

        Self {
            session_id: snap.session_id.clone(),
            flight_id: format!("{}-{}", snap.origin.ident, snap.destination.ident),
            origin_icao: snap.origin.ident.clone(),
            destination_icao: snap.destination.ident.clone(),
            aircraft_type: snap.aircraft.icao_type.clone(),
            flight_phase: snap.flight_phase.clone(),
            phase_evidence: snap.phase_evidence.clone(),
            altitude_ft: pos.map(|p| p.altitude_msl_ft).unwrap_or(0.0),
            groundspeed_kts: pos.map(|p| p.groundspeed_kts).unwrap_or(0.0),
            vertical_speed_fpm: pos.map(|p| p.vertical_speed_fpm).unwrap_or(0.0),
            track_deg: pos.map(|p| p.track_true_deg).unwrap_or(0.0),
            active_leg,
            next_fix,
            next_constraint,
            distance_to_next_nm: geom.distance_to_next_fix_nm,
            remaining_route_nm: geom.remaining_route_distance_nm,
            xtk_nm: geom.xtk_nm,
            xtk_side: geom.xtk_side.clone(),
            is_off_route: geom.is_off_route,
            tod_distance_nm: dprof.tod_distance_nm,
            descent_profile_status: dprof.profile_status.clone(),
            required_vs_fpm: dprof.required_descent_rate_fpm,
            profile_deviation_ft: dprof.profile_deviation_ft,
            departure_brief_text,
            arrival_brief_text,
            star_procedure: snap.destination.star_procedure.clone(),
            approach_procedure: snap.destination.approach_procedure.clone(),
            approach_type: snap.destination.approach_type.clone(),
            is_source_required_at_destination: snap.destination.is_source_required,
            destination_weather,
            online_atc,
            advisories,
            navdata_provenance,
            freshness,
            ownership: SourceOwnershipTable::default(),
        }
    }

    /// Create canonical context from Compact AI Snapshot.
    pub fn from_compact(compact: &OpenAiracCompactSnapshot) -> Self {
        let parts: Vec<&str> = compact.flight.split("->").map(|s| s.trim()).collect();
        let origin = parts.first().copied().unwrap_or("ORIG");
        let dest = parts.get(1).copied().unwrap_or("DEST");

        let is_source_req = compact.arrival.contains("SOURCE REQUIRED");

        let freshness = SubsystemFreshness {
            telemetry_status: compact.freshness.telemetry.clone(),
            telemetry_age_ms: compact.freshness.telemetry_age_ms,
            weather_status: compact.freshness.weather.clone(),
            weather_age_sec: compact.freshness.weather_age_sec,
            online_atc_status: compact.freshness.online.clone(),
            navdata_status: compact.freshness.navdata.clone(),
            is_telemetry_stale: compact.freshness.telemetry == "STALE",
            is_weather_stale: compact.freshness.weather == "STALE",
        };

        Self {
            session_id: format!("compact_{}_{}", origin, dest),
            flight_id: compact.flight.clone(),
            origin_icao: origin.to_string(),
            destination_icao: dest.to_string(),
            aircraft_type: compact.aircraft.clone(),
            flight_phase: compact.phase.clone(),
            phase_evidence: compact.position.clone(),
            altitude_ft: 0.0,
            groundspeed_kts: 0.0,
            vertical_speed_fpm: 0.0,
            track_deg: 0.0,
            active_leg: compact.active_leg.clone(),
            next_fix: compact.next_fix.clone(),
            next_constraint: compact.next_constraint.clone(),
            distance_to_next_nm: 0.0,
            remaining_route_nm: 0.0,
            xtk_nm: 0.0,
            xtk_side: "ON_TRACK".to_string(),
            is_off_route: compact.xtk.contains("OFF ROUTE"),
            tod_distance_nm: None,
            descent_profile_status: compact.descent_profile.clone(),
            required_vs_fpm: None,
            profile_deviation_ft: None,
            departure_brief_text: format!("Departure from {origin}."),
            arrival_brief_text: compact.arrival.clone(),
            star_procedure: None,
            approach_procedure: None,
            approach_type: None,
            is_source_required_at_destination: is_source_req,
            destination_weather: compact.destination_weather.clone(),
            online_atc: compact.online_atc.clone(),
            advisories: compact.advisories.clone(),
            navdata_provenance: compact.provenance.clone(),
            freshness,
            ownership: SourceOwnershipTable::default(),
        }
    }
}
