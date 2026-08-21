//! Integration tests for OpenAIRAC client, schemas, and freshness mapping.

use fd_openairac::context::CrewFlightContext;
use fd_openairac::types::{
    CompactFreshness, EXPECTED_COMPACT_SCHEMA, EXPECTED_SNAPSHOT_V2_SCHEMA, OpenAiracActiveLeg,
    OpenAiracAircraftProfile, OpenAiracAirportBrief, OpenAiracCompactSnapshot, OpenAiracConstraint,
    OpenAiracDataProvenance, OpenAiracDescentProfile, OpenAiracFreshnessReport,
    OpenAiracNavGeometry, OpenAiracNavdataFreshness, OpenAiracOnlineAtcFreshness,
    OpenAiracPosition, OpenAiracRunwayWind, OpenAiracSnapshotV2, OpenAiracStaleFlags,
    OpenAiracTelemetryFreshness, OpenAiracWeatherFreshness, OpenAiracWeatherSummary,
};

fn create_sample_snapshot(
    origin: &str,
    dest: &str,
    is_dest_source_req: bool,
) -> OpenAiracSnapshotV2 {
    OpenAiracSnapshotV2 {
        schema_version: EXPECTED_SNAPSHOT_V2_SCHEMA.to_string(),
        session_id: format!("exec_{}_{}", origin, dest),
        timestamp: "2026-08-21T16:00:00Z".to_string(),
        simulator: "X-Plane 12 Protocol".to_string(),
        connection_state: "CONNECTED".to_string(),
        flight_phase: "CRUISE".to_string(),
        phase_evidence: "Enroute cruise at FL360".to_string(),
        aircraft: OpenAiracAircraftProfile {
            icao_type: "TU154".to_string(),
            model_name: Some("Tupolev Tu-154M".to_string()),
            cruise_altitude_ft: 36000,
            cruise_speed_kts: Some(460),
        },
        origin: OpenAiracAirportBrief {
            ident: origin.to_string(),
            iata_code: Some("SVO".to_string()),
            name: "Sheremetyevo".to_string(),
            municipality: Some("Moscow".to_string()),
            elevation_ft: Some(622.0),
            selected_runway: Some("24C".to_string()),
            procedure_name: Some("EMGAS 3H".to_string()),
            sid_procedure: Some("EMGAS 3H".to_string()),
            star_procedure: None,
            approach_procedure: None,
            approach_type: None,
            transition_name: None,
            initial_or_final_restrictions: vec!["EMGAS: FL120".to_string()],
            provider_name: Some("CAICA".to_string()),
            is_source_required: false,
            source_required_note: None,
        },
        destination: OpenAiracAirportBrief {
            ident: dest.to_string(),
            iata_code: if is_dest_source_req {
                Some("SUI".to_string())
            } else {
                Some("SIP".to_string())
            },
            name: if is_dest_source_req {
                "Sukhumi Babushara".to_string()
            } else {
                "Simferopol".to_string()
            },
            municipality: if is_dest_source_req {
                Some("Sukhumi".to_string())
            } else {
                Some("Simferopol".to_string())
            },
            elevation_ft: if is_dest_source_req {
                Some(53.0)
            } else {
                Some(639.0)
            },
            selected_runway: Some("19R".to_string()),
            procedure_name: if is_dest_source_req {
                None
            } else {
                Some("BURUD 2Y".to_string())
            },
            sid_procedure: None,
            star_procedure: if is_dest_source_req {
                None
            } else {
                Some("BURUD 2Y".to_string())
            },
            approach_procedure: if is_dest_source_req {
                None
            } else {
                Some("ILS 19R".to_string())
            },
            approach_type: if is_dest_source_req {
                None
            } else {
                Some("ILS".to_string())
            },
            transition_name: None,
            initial_or_final_restrictions: Vec::new(),
            provider_name: Some("CAICA".to_string()),
            is_source_required: is_dest_source_req,
            source_required_note: if is_dest_source_req {
                Some("Terminal procedures unavailable in open source dataset; official AIP source required".to_string())
            } else {
                None
            },
        },
        alternate: None,
        position: Some(OpenAiracPosition {
            latitude_deg: 52.41,
            longitude_deg: 37.89,
            altitude_msl_ft: 36000.0,
            altitude_agl_ft: Some(35400.0),
            groundspeed_kts: 460.0,
            vertical_speed_fpm: 0.0,
            track_true_deg: 195.0,
            on_ground: false,
        }),
        active_leg: Some(OpenAiracActiveLeg {
            leg_index: 1,
            leg_name: "EMGAS -> BURUD".to_string(),
            prev_fix: Some("EMGAS".to_string()),
            next_fix: Some("BURUD".to_string()),
            leg_type: "ATS_ROUTE".to_string(),
            route_or_procedure: Some("W109".to_string()),
            desired_track_deg: 195.0,
            distance_nm: 560.0,
            altitude_constraint: Some("FL360".to_string()),
            speed_constraint_kts: None,
            provider_name: Some("CAICA".to_string()),
        }),
        next_constraint: Some(OpenAiracConstraint {
            fix_ident: "BURUD".to_string(),
            constraint_type: "FL360".to_string(),
            altitude_ft: Some(36000),
            speed_kts: None,
            distance_to_constraint_nm: 84.2,
            is_active: true,
        }),
        navigation_geometry: OpenAiracNavGeometry {
            xtk_nm: 0.20,
            xtk_side: "RIGHT".to_string(),
            is_off_route: false,
            distance_to_next_fix_nm: 84.2,
            remaining_route_distance_nm: 385.4,
            direct_destination_distance_nm: 400.0,
            ete_next_fix_sec: Some(659),
            eta_next_fix_utc: Some("2026-08-21T16:11:00Z".to_string()),
            ete_destination_sec: Some(3016),
            eta_destination_utc: Some("2026-08-21T16:50:00Z".to_string()),
        },
        descent_profile: OpenAiracDescentProfile {
            tod_distance_nm: Some(42.5),
            tod_eta_utc: None,
            required_descent_rate_fpm: Some(-1850.0),
            profile_deviation_ft: Some(0.0),
            profile_status: "CRUISE_LEVEL".to_string(),
        },
        weather_summary: OpenAiracWeatherSummary {
            origin_metar: Some("UUEE 24008KT 9999 FEW040 18/10 Q1018".to_string()),
            origin_category: Some("VFR".to_string()),
            destination_metar: Some("URFF 19012KT 9999 SCT030 22/14 Q1013".to_string()),
            destination_taf: Some("URFF 211200Z 2112/2212 19014KT 9999 BKN030".to_string()),
            destination_category: Some("VFR".to_string()),
            destination_runway_wind: Some(OpenAiracRunwayWind {
                runway_ident: "19R".to_string(),
                headwind_kts: 12.0,
                crosswind_kts: 0.0,
                is_tailwind: false,
                is_recommended: true,
            }),
            weather_age_sec: Some(120),
            weather_stale: false,
        },
        online_atc: Vec::new(),
        advisories: Vec::new(),
        data_provenance: OpenAiracDataProvenance {
            active_provider_datasets: vec!["CAICA".to_string(), "WORLD_OPEN".to_string()],
            airac_cycle: Some("2608".to_string()),
            source_required_items: if is_dest_source_req {
                vec!["URAS: TERMINAL PROCEDURES".to_string()]
            } else {
                Vec::new()
            },
            confidence: "AUTHORITATIVE_FEDERATED".to_string(),
        },
        stale_flags: OpenAiracStaleFlags {
            telemetry_stale: false,
            telemetry_age_ms: 150,
            weather_stale: false,
            navdata_stale: false,
        },
        freshness_report: OpenAiracFreshnessReport {
            telemetry: OpenAiracTelemetryFreshness {
                source_timestamp: Some("2026-08-21T16:00:00Z".to_string()),
                received_at: Some("2026-08-21T16:00:00Z".to_string()),
                age_ms: 150,
                status: "CURRENT".to_string(),
            },
            weather: OpenAiracWeatherFreshness {
                source: "NOAA/AviationWeather.gov".to_string(),
                observation_time: None,
                fetched_at: Some("2026-08-21T15:58:00Z".to_string()),
                age_sec: Some(120),
                status: "CURRENT".to_string(),
            },
            online_atc: OpenAiracOnlineAtcFreshness {
                network: "NONE".to_string(),
                fetched_at: None,
                age_sec: None,
                status: "UNAVAILABLE".to_string(),
            },
            navdata: OpenAiracNavdataFreshness {
                primary_provider: "CAICA".to_string(),
                airac_cycle: "2608".to_string(),
                effective_from: None,
                effective_to: None,
                status: if is_dest_source_req {
                    "SOURCE_REQUIRED".to_string()
                } else {
                    "CURRENT".to_string()
                },
            },
        },
        navigation_warnings: Vec::new(),
    }
}

#[test]
fn test_snapshot_v2_parsing_and_crew_context_mapping() {
    let snap = create_sample_snapshot("UUEE", "URFF", false);
    let ctx = CrewFlightContext::from_snapshot_v2(&snap);

    assert_eq!(ctx.flight_id, "UUEE-URFF");
    assert_eq!(ctx.origin_icao, "UUEE");
    assert_eq!(ctx.destination_icao, "URFF");
    assert_eq!(ctx.aircraft_type, "TU154");
    assert_eq!(ctx.flight_phase, "CRUISE");
    assert_eq!(ctx.altitude_ft, 36000.0);
    assert_eq!(ctx.groundspeed_kts, 460.0);
    assert_eq!(ctx.next_fix, "BURUD");
    assert_eq!(ctx.star_procedure, Some("BURUD 2Y".to_string()));
    assert_eq!(ctx.approach_procedure, Some("ILS 19R".to_string()));
    assert_eq!(ctx.approach_type, Some("ILS".to_string()));
    assert!(ctx.arrival_brief_text.contains("STAR BURUD 2Y"));
    assert!(ctx.arrival_brief_text.contains("Approach ILS 19R"));
    assert_eq!(ctx.xtk_side, "RIGHT");
    assert_eq!(ctx.tod_distance_nm, Some(42.5));
    assert_eq!(ctx.freshness.telemetry_status, "CURRENT");
    assert_eq!(ctx.freshness.weather_status, "CURRENT");
    assert_eq!(ctx.freshness.navdata_status, "CURRENT");
    assert_eq!(
        ctx.ownership.navigation_and_route,
        "OpenAIRAC (Authoritative)"
    );
    assert_eq!(ctx.ownership.flight_phase, "OpenAIRAC (FlightPhaseEngine)");
}

#[test]
fn test_compact_snapshot_parsing_and_context_mapping() {
    let compact = OpenAiracCompactSnapshot {
        schema_version: EXPECTED_COMPACT_SCHEMA.to_string(),
        flight: "UUEE -> URFF".to_string(),
        phase: "CRUISE".to_string(),
        aircraft: "TU154".to_string(),
        position: "LAT 52.4100° LON 37.8900° | 36000 ft MSL | GS 460 kt | TRK 195°".to_string(),
        active_leg: "EMGAS -> BURUD (ATS_ROUTE | Desired TRK: 195°)".to_string(),
        next_fix: "BURUD (84.2 NM, ETE: 10m 59s)".to_string(),
        next_constraint: "BURUD: FL360".to_string(),
        xtk: "0.20 NM RIGHT (ON ROUTE)".to_string(),
        route_remaining: "385.4 NM (ETE: 50m 16s)".to_string(),
        tod: "42.5 NM".to_string(),
        descent_profile: "CRUISE_LEVEL (Req VS: -1850 fpm | Dev: 0 ft)".to_string(),
        arrival: "BURUD 2Y / ILS 19R / RWY 19R (URFF)".to_string(),
        destination_weather: "URFF 19012KT 9999 SCT030 22/14 Q1013".to_string(),
        online_atc: vec!["URFF_APP [APP] (125.700 MHz)".to_string()],
        advisories: Vec::new(),
        provenance: "CAICA, WORLD_OPEN | AIRAC 2608".to_string(),
        freshness: CompactFreshness {
            telemetry: "CURRENT".to_string(),
            weather: "CURRENT".to_string(),
            online: "CURRENT".to_string(),
            navdata: "CURRENT".to_string(),
            telemetry_age_ms: 150,
            weather_age_sec: Some(120),
        },
        warnings: Vec::new(),
    };

    let ctx = CrewFlightContext::from_compact(&compact);
    assert_eq!(ctx.origin_icao, "UUEE");
    assert_eq!(ctx.destination_icao, "URFF");
    assert_eq!(ctx.aircraft_type, "TU154");
    assert_eq!(ctx.flight_phase, "CRUISE");
    assert_eq!(ctx.freshness.telemetry_status, "CURRENT");
    assert_eq!(ctx.freshness.weather_status, "CURRENT");
    assert!(!ctx.is_source_required_at_destination);
}

#[test]
fn test_uras_source_required_canary_mapping() {
    let snap_uras = create_sample_snapshot("URSS", "URAS", true);
    let ctx_uras = CrewFlightContext::from_snapshot_v2(&snap_uras);

    assert_eq!(ctx_uras.destination_icao, "URAS");
    assert!(ctx_uras.is_source_required_at_destination);
    assert!(ctx_uras.arrival_brief_text.contains("SOURCE_REQUIRED"));
    assert_eq!(ctx_uras.freshness.navdata_status, "SOURCE_REQUIRED");
}
