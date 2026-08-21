//! Typed JSON models corresponding to OpenAIRAC 3.2 schema contracts.

use serde::{Deserialize, Serialize};

pub const EXPECTED_SNAPSHOT_V2_SCHEMA: &str = "flightdeck_snapshot_v2";
pub const EXPECTED_COMPACT_SCHEMA: &str = "compact_ai_snapshot_v1";

/// Aircraft position in OpenAIRAC snapshot.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct OpenAiracPosition {
    pub latitude_deg: f64,
    pub longitude_deg: f64,
    pub altitude_msl_ft: f64,
    pub altitude_agl_ft: Option<f64>,
    pub groundspeed_kts: f64,
    pub vertical_speed_fpm: f64,
    pub track_true_deg: f64,
    pub on_ground: bool,
}

/// Aircraft operational profile.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct OpenAiracAircraftProfile {
    pub icao_type: String,
    pub model_name: Option<String>,
    pub cruise_altitude_ft: u32,
    pub cruise_speed_kts: Option<u32>,
}

/// Structured airport brief in snapshot.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct OpenAiracAirportBrief {
    pub ident: String,
    pub iata_code: Option<String>,
    pub name: String,
    pub municipality: Option<String>,
    pub elevation_ft: Option<f64>,
    pub selected_runway: Option<String>,
    pub procedure_name: Option<String>,
    pub sid_procedure: Option<String>,
    pub star_procedure: Option<String>,
    pub approach_procedure: Option<String>,
    pub approach_type: Option<String>,
    pub transition_name: Option<String>,
    #[serde(default)]
    pub initial_or_final_restrictions: Vec<String>,
    pub provider_name: Option<String>,
    #[serde(default)]
    pub is_source_required: bool,
    pub source_required_note: Option<String>,
}

/// Active navigation leg.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct OpenAiracActiveLeg {
    pub leg_index: usize,
    pub leg_name: String,
    pub prev_fix: Option<String>,
    pub next_fix: Option<String>,
    pub leg_type: String,
    pub route_or_procedure: Option<String>,
    pub desired_track_deg: f64,
    pub distance_nm: f64,
    pub altitude_constraint: Option<String>,
    pub speed_constraint_kts: Option<u32>,
    pub provider_name: Option<String>,
}

/// Upcoming constraint.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct OpenAiracConstraint {
    pub fix_ident: String,
    pub constraint_type: String,
    pub altitude_ft: Option<u32>,
    pub speed_kts: Option<u32>,
    pub distance_to_constraint_nm: f64,
    pub is_active: bool,
}

/// Navigation geometry & progress.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct OpenAiracNavGeometry {
    pub xtk_nm: f64,
    pub xtk_side: String,
    pub is_off_route: bool,
    pub distance_to_next_fix_nm: f64,
    pub remaining_route_distance_nm: f64,
    pub direct_destination_distance_nm: f64,
    pub ete_next_fix_sec: Option<u32>,
    pub eta_next_fix_utc: Option<String>,
    pub ete_destination_sec: Option<u32>,
    pub eta_destination_utc: Option<String>,
}

/// Descent profile & TOD.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct OpenAiracDescentProfile {
    pub tod_distance_nm: Option<f64>,
    pub tod_eta_utc: Option<String>,
    pub required_descent_rate_fpm: Option<f64>,
    pub profile_deviation_ft: Option<f64>,
    pub profile_status: String,
}

/// Runway wind components.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct OpenAiracRunwayWind {
    pub runway_ident: String,
    pub headwind_kts: f64,
    pub crosswind_kts: f64,
    pub is_tailwind: bool,
    pub is_recommended: bool,
}

/// Real-time weather summary.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct OpenAiracWeatherSummary {
    pub origin_metar: Option<String>,
    pub origin_category: Option<String>,
    pub destination_metar: Option<String>,
    pub destination_taf: Option<String>,
    pub destination_category: Option<String>,
    pub destination_runway_wind: Option<OpenAiracRunwayWind>,
    pub weather_age_sec: Option<u64>,
    #[serde(default)]
    pub weather_stale: bool,
}

/// Relevant online ATC controller.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct OpenAiracOnlineAtc {
    pub network: String,
    pub callsign: String,
    pub frequency_mhz: String,
    pub facility_type: String,
    pub role_context: String,
    pub distance_nm: Option<f64>,
}

/// Rule-based crew advisory.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct OpenAiracAdvisory {
    pub level: String,
    pub code: String,
    pub message: String,
    pub evidence: String,
    pub timestamp: String,
}

/// Data provenance.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct OpenAiracDataProvenance {
    #[serde(default)]
    pub active_provider_datasets: Vec<String>,
    pub airac_cycle: Option<String>,
    #[serde(default)]
    pub source_required_items: Vec<String>,
    pub confidence: String,
}

/// Granular multi-source freshness report.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct OpenAiracFreshnessReport {
    pub telemetry: OpenAiracTelemetryFreshness,
    pub weather: OpenAiracWeatherFreshness,
    pub online_atc: OpenAiracOnlineAtcFreshness,
    pub navdata: OpenAiracNavdataFreshness,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct OpenAiracTelemetryFreshness {
    pub source_timestamp: Option<String>,
    pub received_at: Option<String>,
    pub age_ms: u64,
    pub status: String,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct OpenAiracWeatherFreshness {
    pub source: String,
    pub observation_time: Option<String>,
    pub fetched_at: Option<String>,
    pub age_sec: Option<u64>,
    pub status: String,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct OpenAiracOnlineAtcFreshness {
    pub network: String,
    pub fetched_at: Option<String>,
    pub age_sec: Option<u64>,
    pub status: String,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct OpenAiracNavdataFreshness {
    pub primary_provider: String,
    pub airac_cycle: String,
    pub effective_from: Option<String>,
    pub effective_to: Option<String>,
    pub status: String,
}

/// Stale flags.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct OpenAiracStaleFlags {
    pub telemetry_stale: bool,
    pub telemetry_age_ms: u64,
    pub weather_stale: bool,
    pub navdata_stale: bool,
}

/// Full Flightdeck Snapshot v2.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct OpenAiracSnapshotV2 {
    pub schema_version: String,
    pub session_id: String,
    pub timestamp: String,
    pub simulator: String,
    pub connection_state: String,
    pub flight_phase: String,
    pub phase_evidence: String,
    pub aircraft: OpenAiracAircraftProfile,
    pub origin: OpenAiracAirportBrief,
    pub destination: OpenAiracAirportBrief,
    pub alternate: Option<OpenAiracAirportBrief>,
    pub position: Option<OpenAiracPosition>,
    pub active_leg: Option<OpenAiracActiveLeg>,
    pub next_constraint: Option<OpenAiracConstraint>,
    pub navigation_geometry: OpenAiracNavGeometry,
    pub descent_profile: OpenAiracDescentProfile,
    pub weather_summary: OpenAiracWeatherSummary,
    #[serde(default)]
    pub online_atc: Vec<OpenAiracOnlineAtc>,
    #[serde(default)]
    pub advisories: Vec<OpenAiracAdvisory>,
    pub data_provenance: OpenAiracDataProvenance,
    pub stale_flags: OpenAiracStaleFlags,
    pub freshness_report: OpenAiracFreshnessReport,
    #[serde(default)]
    pub navigation_warnings: Vec<String>,
}

/// Structured freshness in Compact AI Snapshot.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CompactFreshness {
    pub telemetry: String,
    pub weather: String,
    pub online: String,
    pub navdata: String,
    pub telemetry_age_ms: u64,
    pub weather_age_sec: Option<u64>,
}

/// Compact AI Snapshot v1.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct OpenAiracCompactSnapshot {
    pub schema_version: String,
    pub flight: String,
    pub phase: String,
    pub aircraft: String,
    pub position: String,
    pub active_leg: String,
    pub next_fix: String,
    pub next_constraint: String,
    pub xtk: String,
    pub route_remaining: String,
    pub tod: String,
    pub descent_profile: String,
    pub arrival: String,
    pub destination_weather: String,
    #[serde(default)]
    pub online_atc: Vec<String>,
    #[serde(default)]
    pub advisories: Vec<String>,
    pub provenance: String,
    pub freshness: CompactFreshness,
    #[serde(default)]
    pub warnings: Vec<String>,
}

/// Monotonic flight lifecycle event from OpenAIRAC.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct OpenAiracEvent {
    pub id: u64,
    pub timestamp: String,
    pub event_type: String,
    pub description: String,
    #[serde(default)]
    pub metadata: serde_json::Value,
}

/// Departure briefing.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct OpenAiracDepartureBrief {
    pub origin_icao: String,
    pub origin_name: String,
    pub elevation_ft: f64,
    pub departure_runway: Option<String>,
    pub sid_procedure: Option<String>,
    pub sid_transition: Option<String>,
    #[serde(default)]
    pub initial_altitude_constraints: Vec<String>,
    pub weather_metar: Option<String>,
    pub runway_wind: Option<OpenAiracRunwayWind>,
    pub provider_name: Option<String>,
    #[serde(default)]
    pub warnings: Vec<String>,
    pub briefing_text: String,
}

/// Arrival briefing.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct OpenAiracArrivalBrief {
    pub destination_icao: String,
    pub destination_name: String,
    pub elevation_ft: f64,
    pub arrival_runway: Option<String>,
    pub star_procedure: Option<String>,
    pub star_transition: Option<String>,
    pub approach_procedure: Option<String>,
    pub approach_type: Option<String>,
    #[serde(default)]
    pub final_approach_restrictions: Vec<String>,
    pub weather_metar: Option<String>,
    pub weather_taf: Option<String>,
    pub runway_wind: Option<OpenAiracRunwayWind>,
    #[serde(default)]
    pub is_source_required: bool,
    pub source_required_note: Option<String>,
    pub provider_name: Option<String>,
    #[serde(default)]
    pub warnings: Vec<String>,
    pub briefing_text: String,
}

/// Multi-identity airport resolution.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct OpenAiracResolvedIdentity {
    pub query_ident: String,
    pub authoritative_ident: String,
    pub iata_code: Option<String>,
    pub airport_name: String,
    pub country_code: String,
    pub primary_provider: String,
    #[serde(default)]
    pub alternate_identities: Vec<OpenAiracProviderIdentity>,
    pub terminal_procedures_status: String,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct OpenAiracProviderIdentity {
    pub provider: String,
    pub ident: String,
    pub name: String,
    pub note: Option<String>,
}
