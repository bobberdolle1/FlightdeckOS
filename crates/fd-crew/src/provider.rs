//! Model Provider abstraction and deterministic AI provider.

use fd_openairac::context::CrewFlightContext;
use serde::{Deserialize, Serialize};

use crate::error::AiCrewError;
use crate::tools::{CrewToolRegistry, ProposedAction, ToolEvidence};

/// Prompt payload sent to AI model provider.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AiCrewPrompt {
    pub system_prompt: String,
    pub user_query: String,
    pub conversation_history: Vec<(String, String)>,
    pub flight_context_compact: Option<String>,
}

/// Structured AI response with fact evidence and proposed actions.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AiCrewResponse {
    pub message: String,
    pub tool_evidence: Vec<ToolEvidence>,
    pub proposed_actions: Vec<ProposedAction>,
    pub advisories: Vec<String>,
    pub freshness_qualification: Option<String>,
}

/// Pluggable AI Model Provider interface.
pub trait AiModelProvider: Send + Sync {
    fn name(&self) -> &'static str;
    fn generate_response(
        &self,
        prompt: &AiCrewPrompt,
        context: &CrewFlightContext,
    ) -> Result<AiCrewResponse, AiCrewError>;
}

/// Deterministic, offline-safe AI Crew Provider powered by OpenAIRAC tools.
pub struct DeterministicAiProvider;

impl AiModelProvider for DeterministicAiProvider {
    fn name(&self) -> &'static str {
        "Deterministic / Tool-Grounded Provider"
    }

    fn generate_response(
        &self,
        prompt: &AiCrewPrompt,
        context: &CrewFlightContext,
    ) -> Result<AiCrewResponse, AiCrewError> {
        let q = prompt.user_query.to_lowercase();
        let mut evidence = Vec::new();
        let proposed_actions = Vec::new();
        let mut freshness_qualification = None;

        let message = if q.contains("where are we")
            || q.contains("position")
            || q.contains("location")
        {
            let (_, ev) = CrewToolRegistry::execute_tool("get_flight_state", context);
            let (_, ev_leg) = CrewToolRegistry::execute_tool("get_active_leg", context);
            evidence.push(ev);
            evidence.push(ev_leg);

            if context.freshness.is_telemetry_stale {
                freshness_qualification = Some(format!(
                    "Telemetry is STALE (age {} ms)",
                    context.freshness.telemetry_age_ms
                ));
                format!(
                    "Last known position was in {} phase at {:.0} ft MSL (GS {:.0} kt, TRK {:.0}°). Note: Telemetry is currently STALE (last packet {} ms ago).",
                    context.flight_phase,
                    context.altitude_ft,
                    context.groundspeed_kts,
                    context.track_deg,
                    context.freshness.telemetry_age_ms
                )
            } else {
                format!(
                    "We are currently in {} phase at FL{:.0} ({:.0} ft MSL), groundspeed {:.0} kt, tracking {:.0}°. XTK is {:.2} NM {} ({}).",
                    context.flight_phase,
                    context.altitude_ft / 100.0,
                    context.altitude_ft,
                    context.groundspeed_kts,
                    context.track_deg,
                    context.xtk_nm,
                    context.xtk_side,
                    if context.is_off_route {
                        "OFF ROUTE"
                    } else {
                        "ON ROUTE"
                    }
                )
            }
        } else if q.contains("flying now")
            || q.contains("what are we flying")
            || q.contains("active leg")
        {
            let (_, ev) = CrewToolRegistry::execute_tool("get_active_leg", context);
            evidence.push(ev);
            format!(
                "We are flying leg {}. Next waypoint is {} in {:.1} NM.",
                context.active_leg, context.next_fix, context.distance_to_next_nm
            )
        } else if q.contains("next") || q.contains("waypoint") || q.contains("fix") {
            let (_, ev) = CrewToolRegistry::execute_tool("get_active_leg", context);
            let (_, ev_c) = CrewToolRegistry::execute_tool("get_next_constraint", context);
            evidence.push(ev);
            evidence.push(ev_c);
            format!(
                "Next waypoint is {} in {:.1} NM. Next constraint: {}.",
                context.next_fix, context.distance_to_next_nm, context.next_constraint
            )
        } else if q.contains("tod") || q.contains("descend") || q.contains("descent") {
            let (_, ev) = CrewToolRegistry::execute_tool("get_tod_and_descent", context);
            evidence.push(ev);

            if let Some(tod_nm) = context.tod_distance_nm {
                format!(
                    "Top of Descent is in {:.1} NM (advisory/estimated 3.0° profile). Status: {}.",
                    tod_nm, context.descent_profile_status
                )
            } else if context.flight_phase == "DESCENT" || context.flight_phase == "APPROACH" {
                format!(
                    "Descent is in progress ({}). Required vertical speed: {} fpm, profile deviation: {} ft.",
                    context.descent_profile_status,
                    context
                        .required_vs_fpm
                        .map(|v| format!("{:.0}", v))
                        .unwrap_or_else(|| "--".to_string()),
                    context
                        .profile_deviation_ft
                        .map(|v| format!("{:.0}", v))
                        .unwrap_or_else(|| "--".to_string())
                )
            } else {
                format!(
                    "Descent profile status: {}.",
                    context.descent_profile_status
                )
            }
        } else if q.contains("star")
            || q.contains("approach")
            || q.contains("arrival")
            || q.contains("brief")
        {
            let (_, ev) = CrewToolRegistry::execute_tool("get_arrival_brief", context);
            evidence.push(ev);

            if context.is_source_required_at_destination {
                format!(
                    "Arrival at {}: SOURCE_REQUIRED. No official source-backed terminal procedures (STAR / Approach) are available in the dataset.",
                    context.destination_icao
                )
            } else {
                let star_str = context.star_procedure.as_deref().unwrap_or("DIRECT");
                let app_str = context.approach_procedure.as_deref().unwrap_or("VISUAL");
                format!(
                    "Arrival briefing for {}: Arrival at {} (Elev {:.0} ft), Runway {}, STAR {}, Approach {}. Weather: {}.",
                    context.destination_icao,
                    context.destination_icao,
                    context.altitude_ft,
                    context
                        .arrival_brief_text
                        .split("Runway ")
                        .nth(1)
                        .and_then(|s| s.split(',').next())
                        .unwrap_or("DEFAULT"),
                    star_str,
                    app_str,
                    context.destination_weather
                )
            }
        } else if q.contains("weather") || q.contains("metar") || q.contains("wind") {
            let (_, ev) = CrewToolRegistry::execute_tool("get_weather", context);
            evidence.push(ev);

            if context.freshness.is_weather_stale {
                freshness_qualification = Some("Weather data is STALE".to_string());
                format!(
                    "Destination weather for {} (STALE): {}.",
                    context.destination_icao, context.destination_weather
                )
            } else {
                format!(
                    "Destination weather for {}: {}.",
                    context.destination_icao, context.destination_weather
                )
            }
        } else if q.contains("current")
            || q.contains("fresh")
            || q.contains("stale")
            || q.contains("status")
        {
            let (_, ev) = CrewToolRegistry::execute_tool("get_data_freshness", context);
            evidence.push(ev);
            format!(
                "Data freshness: Telemetry is {} ({} ms age), Weather is {}, Online ATC is {}, Navdata is {}.",
                context.freshness.telemetry_status,
                context.freshness.telemetry_age_ms,
                context.freshness.weather_status,
                context.freshness.online_atc_status,
                context.freshness.navdata_status
            )
        } else if q.contains("checklist")
            || q.contains("sop")
            || q.contains("what do we need to do")
        {
            let (_, ev) = CrewToolRegistry::execute_tool("get_sop_state", context);
            evidence.push(ev);
            let sop_status =
                crate::sop_binding::SopAircraftBinding::evaluate(&context.aircraft_type, None);
            match sop_status {
                crate::sop_binding::SopBindingStatus::Active {
                    current_flow,
                    pending_steps_count,
                    ..
                } => {
                    format!(
                        "Active SOP flow for {}: '{}' ({} pending items).",
                        context.aircraft_type, current_flow, pending_steps_count
                    )
                }
                crate::sop_binding::SopBindingStatus::UnavailableForAircraft {
                    aircraft,
                    reason,
                } => {
                    format!("SOP UNAVAILABLE FOR {}: {}.", aircraft, reason)
                }
                crate::sop_binding::SopBindingStatus::NotInstalled => {
                    format!("No SOP package loaded for {}.", context.aircraft_type)
                }
            }
        } else {
            let (_, ev) = CrewToolRegistry::execute_tool("get_flight_state", context);
            evidence.push(ev);
            format!(
                "Flight {} to {} in {} phase. All flight parameters monitored.",
                context.origin_icao, context.destination_icao, context.flight_phase
            )
        };

        Ok(AiCrewResponse {
            message,
            tool_evidence: evidence,
            proposed_actions,
            advisories: context.advisories.clone(),
            freshness_qualification,
        })
    }
}
