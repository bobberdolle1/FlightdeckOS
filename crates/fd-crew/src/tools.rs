//! Deterministic Crew Tool Registry and Fact Evidence Tracking.

use fd_openairac::context::CrewFlightContext;
use serde::{Deserialize, Serialize};
use std::time::{SystemTime, UNIX_EPOCH};

/// Evidence trace for a flight fact used by the AI crew.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ToolEvidence {
    pub tool_name: String,
    pub timestamp_utc: String,
    pub source_subsystem: String,
    pub freshness_status: String,
    pub factual_payload: serde_json::Value,
}

/// JSON Schema tool definition for AI models.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CrewToolDefinition {
    pub name: String,
    pub description: String,
    pub parameters: serde_json::Value,
}

/// Non-executing proposed action for future action pipeline.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ProposedAction {
    pub action_type: String,
    pub target: String,
    pub value: Option<String>,
    pub reason: String,
    pub is_executable_in_v02: bool,
}

/// Registry of all deterministic flight facts and tools available to the AI crew.
pub struct CrewToolRegistry;

impl CrewToolRegistry {
    /// Return tool definitions schema for model prompting.
    pub fn definitions() -> Vec<CrewToolDefinition> {
        vec![
            CrewToolDefinition {
                name: "get_flight_state".to_string(),
                description: "Get current flight phase, altitude, groundspeed, track, position, and connection status.".to_string(),
                parameters: serde_json::json!({ "type": "object", "properties": {} }),
            },
            CrewToolDefinition {
                name: "get_active_leg".to_string(),
                description: "Get active flight route leg, next waypoint, cross-track error (XTK), distance to next fix, and ETE.".to_string(),
                parameters: serde_json::json!({ "type": "object", "properties": {} }),
            },
            CrewToolDefinition {
                name: "get_next_constraint".to_string(),
                description: "Get the next upcoming altitude or speed constraint along the active flight plan.".to_string(),
                parameters: serde_json::json!({ "type": "object", "properties": {} }),
            },
            CrewToolDefinition {
                name: "get_tod_and_descent".to_string(),
                description: "Get Top of Descent (TOD) distance, descent profile status, and required vertical speed.".to_string(),
                parameters: serde_json::json!({ "type": "object", "properties": {} }),
            },
            CrewToolDefinition {
                name: "get_arrival_brief".to_string(),
                description: "Get destination arrival briefing, runway, STAR, approach, and SOURCE_REQUIRED status.".to_string(),
                parameters: serde_json::json!({ "type": "object", "properties": {} }),
            },
            CrewToolDefinition {
                name: "get_departure_brief".to_string(),
                description: "Get origin departure briefing, runway, SID, and initial altitude restrictions.".to_string(),
                parameters: serde_json::json!({ "type": "object", "properties": {} }),
            },
            CrewToolDefinition {
                name: "get_weather".to_string(),
                description: "Get destination METAR, runway wind components, and weather freshness status.".to_string(),
                parameters: serde_json::json!({ "type": "object", "properties": {} }),
            },
            CrewToolDefinition {
                name: "get_online_atc".to_string(),
                description: "Get online VATSIM/IVAO controllers active along the route corridor or at destination.".to_string(),
                parameters: serde_json::json!({ "type": "object", "properties": {} }),
            },
            CrewToolDefinition {
                name: "get_advisories".to_string(),
                description: "Get active rule-based crew advisories (INFO, CAUTION, WARNING).".to_string(),
                parameters: serde_json::json!({ "type": "object", "properties": {} }),
            },
            CrewToolDefinition {
                name: "get_data_freshness".to_string(),
                description: "Get multi-source freshness statuses for telemetry, weather, online ATC, and navdata.".to_string(),
                parameters: serde_json::json!({ "type": "object", "properties": {} }),
            },
            CrewToolDefinition {
                name: "get_sop_state".to_string(),
                description: "Get active SOP checklist flow, step status, and aircraft compatibility.".to_string(),
                parameters: serde_json::json!({ "type": "object", "properties": {} }),
            },
        ]
    }

    /// Execute a tool against the current deterministic flight context.
    pub fn execute_tool(
        tool_name: &str,
        context: &CrewFlightContext,
    ) -> (serde_json::Value, ToolEvidence) {
        let now_str = match SystemTime::now().duration_since(UNIX_EPOCH) {
            Ok(d) => format!("ts:{}s", d.as_secs()),
            Err(_) => "ts:0s".to_string(),
        };

        match tool_name {
            "get_flight_state" => {
                let payload = serde_json::json!({
                    "flight": context.flight_id,
                    "phase": context.flight_phase,
                    "phase_evidence": context.phase_evidence,
                    "aircraft": context.aircraft_type,
                    "altitude_ft": context.altitude_ft,
                    "groundspeed_kts": context.groundspeed_kts,
                    "track_deg": context.track_deg,
                    "telemetry_freshness": context.freshness.telemetry_status,
                });
                let evidence = ToolEvidence {
                    tool_name: tool_name.to_string(),
                    timestamp_utc: now_str,
                    source_subsystem: "OpenAIRAC Navigation & Telemetry Engine".to_string(),
                    freshness_status: context.freshness.telemetry_status.clone(),
                    factual_payload: payload.clone(),
                };
                (payload, evidence)
            }
            "get_active_leg" => {
                let payload = serde_json::json!({
                    "active_leg": context.active_leg,
                    "next_fix": context.next_fix,
                    "distance_to_next_nm": context.distance_to_next_nm,
                    "xtk_nm": context.xtk_nm,
                    "xtk_side": context.xtk_side,
                    "is_off_route": context.is_off_route,
                    "remaining_route_nm": context.remaining_route_nm,
                });
                let evidence = ToolEvidence {
                    tool_name: tool_name.to_string(),
                    timestamp_utc: now_str,
                    source_subsystem: "OpenAIRAC Geodesic Active Leg Tracker".to_string(),
                    freshness_status: context.freshness.telemetry_status.clone(),
                    factual_payload: payload.clone(),
                };
                (payload, evidence)
            }
            "get_next_constraint" => {
                let payload = serde_json::json!({
                    "next_constraint": context.next_constraint,
                });
                let evidence = ToolEvidence {
                    tool_name: tool_name.to_string(),
                    timestamp_utc: now_str,
                    source_subsystem: "OpenAIRAC Procedure Constraints".to_string(),
                    freshness_status: context.freshness.navdata_status.clone(),
                    factual_payload: payload.clone(),
                };
                (payload, evidence)
            }
            "get_tod_and_descent" => {
                let payload = serde_json::json!({
                    "tod_distance_nm": context.tod_distance_nm,
                    "descent_profile_status": context.descent_profile_status,
                    "required_vs_fpm": context.required_vs_fpm,
                    "profile_deviation_ft": context.profile_deviation_ft,
                });
                let evidence = ToolEvidence {
                    tool_name: tool_name.to_string(),
                    timestamp_utc: now_str,
                    source_subsystem: "OpenAIRAC TOD & VNAV Profile Engine".to_string(),
                    freshness_status: context.freshness.telemetry_status.clone(),
                    factual_payload: payload.clone(),
                };
                (payload, evidence)
            }
            "get_arrival_brief" => {
                let payload = serde_json::json!({
                    "destination": context.destination_icao,
                    "star": context.star_procedure,
                    "approach": context.approach_procedure,
                    "approach_type": context.approach_type,
                    "arrival_brief": context.arrival_brief_text,
                    "is_source_required": context.is_source_required_at_destination,
                    "weather": context.destination_weather,
                    "navdata_provenance": context.navdata_provenance,
                });
                let evidence = ToolEvidence {
                    tool_name: tool_name.to_string(),
                    timestamp_utc: now_str,
                    source_subsystem: "OpenAIRAC Structured Arrival Briefing".to_string(),
                    freshness_status: context.freshness.navdata_status.clone(),
                    factual_payload: payload.clone(),
                };
                (payload, evidence)
            }
            "get_departure_brief" => {
                let payload = serde_json::json!({
                    "origin": context.origin_icao,
                    "departure_brief": context.departure_brief_text,
                    "navdata_provenance": context.navdata_provenance,
                });
                let evidence = ToolEvidence {
                    tool_name: tool_name.to_string(),
                    timestamp_utc: now_str,
                    source_subsystem: "OpenAIRAC Structured Departure Briefing".to_string(),
                    freshness_status: context.freshness.navdata_status.clone(),
                    factual_payload: payload.clone(),
                };
                (payload, evidence)
            }
            "get_weather" => {
                let payload = serde_json::json!({
                    "destination_icao": context.destination_icao,
                    "destination_metar": context.destination_weather,
                    "weather_freshness": context.freshness.weather_status,
                    "is_weather_stale": context.freshness.is_weather_stale,
                    "weather_age_sec": context.freshness.weather_age_sec,
                });
                let evidence = ToolEvidence {
                    tool_name: tool_name.to_string(),
                    timestamp_utc: now_str,
                    source_subsystem: "OpenAIRAC Weather Gateway".to_string(),
                    freshness_status: context.freshness.weather_status.clone(),
                    factual_payload: payload.clone(),
                };
                (payload, evidence)
            }
            "get_online_atc" => {
                let payload = serde_json::json!({
                    "online_atc": context.online_atc,
                    "online_freshness": context.freshness.online_atc_status,
                });
                let evidence = ToolEvidence {
                    tool_name: tool_name.to_string(),
                    timestamp_utc: now_str,
                    source_subsystem: "OpenAIRAC Online Network Gateway".to_string(),
                    freshness_status: context.freshness.online_atc_status.clone(),
                    factual_payload: payload.clone(),
                };
                (payload, evidence)
            }
            "get_advisories" => {
                let payload = serde_json::json!({
                    "advisories": context.advisories,
                });
                let evidence = ToolEvidence {
                    tool_name: tool_name.to_string(),
                    timestamp_utc: now_str,
                    source_subsystem: "OpenAIRAC Crew Advisory Engine".to_string(),
                    freshness_status: "CURRENT".to_string(),
                    factual_payload: payload.clone(),
                };
                (payload, evidence)
            }
            "get_sop_state" => {
                let sop_status =
                    crate::sop_binding::SopAircraftBinding::evaluate(&context.aircraft_type, None);
                let payload = serde_json::to_value(&sop_status)
                    .unwrap_or(serde_json::json!({ "status": "NOT_INSTALLED" }));
                let evidence = ToolEvidence {
                    tool_name: tool_name.to_string(),
                    timestamp_utc: now_str,
                    source_subsystem: "FlightdeckOS SOP Engine (fd-sop)".to_string(),
                    freshness_status: "CURRENT".to_string(),
                    factual_payload: payload.clone(),
                };
                (payload, evidence)
            }
            _ => {
                let payload = serde_json::json!({
                    "telemetry": context.freshness.telemetry_status,
                    "telemetry_age_ms": context.freshness.telemetry_age_ms,
                    "weather": context.freshness.weather_status,
                    "weather_age_sec": context.freshness.weather_age_sec,
                    "online_atc": context.freshness.online_atc_status,
                    "navdata": context.freshness.navdata_status,
                });
                let evidence = ToolEvidence {
                    tool_name: "get_data_freshness".to_string(),
                    timestamp_utc: now_str,
                    source_subsystem: "OpenAIRAC Subsystem Freshness Engine".to_string(),
                    freshness_status: "CURRENT".to_string(),
                    factual_payload: payload.clone(),
                };
                (payload, evidence)
            }
        }
    }
}
