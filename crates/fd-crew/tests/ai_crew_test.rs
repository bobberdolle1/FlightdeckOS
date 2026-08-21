//! Unit and E2E integration tests for FlightdeckOS AI Crew Runtime.

use std::sync::Arc;

use fd_crew::{AiCrewRuntime, DeterministicAiProvider};
use fd_openairac::types::{
    EXPECTED_SNAPSHOT_V2_SCHEMA, OpenAiracActiveLeg, OpenAiracAircraftProfile,
    OpenAiracAirportBrief, OpenAiracConstraint, OpenAiracDataProvenance, OpenAiracDescentProfile,
    OpenAiracFreshnessReport, OpenAiracNavGeometry, OpenAiracNavdataFreshness,
    OpenAiracOnlineAtcFreshness, OpenAiracPosition, OpenAiracRunwayWind, OpenAiracSnapshotV2,
    OpenAiracStaleFlags, OpenAiracTelemetryFreshness, OpenAiracWeatherFreshness,
    OpenAiracWeatherSummary,
};

fn create_uuee_urff_snapshot() -> OpenAiracSnapshotV2 {
    OpenAiracSnapshotV2 {
        schema_version: EXPECTED_SNAPSHOT_V2_SCHEMA.to_string(),
        session_id: "exec_UUEE_URFF_1724248800".to_string(),
        timestamp: "2026-08-21T16:00:00Z".to_string(),
        simulator: "X-Plane 12 Protocol".to_string(),
        connection_state: "CONNECTED".to_string(),
        flight_phase: "CRUISE".to_string(),
        phase_evidence: "Enroute cruise (Alt FL360, GS 460 kt)".to_string(),
        aircraft: OpenAiracAircraftProfile {
            icao_type: "TU154".to_string(),
            model_name: Some("Tupolev Tu-154M".to_string()),
            cruise_altitude_ft: 36000,
            cruise_speed_kts: Some(460),
        },
        origin: OpenAiracAirportBrief {
            ident: "UUEE".to_string(),
            iata_code: Some("SVO".to_string()),
            name: "Sheremetyevo".to_string(),
            municipality: Some("Moscow".to_string()),
            elevation_ft: Some(622.0),
            selected_runway: Some("24C".to_string()),
            procedure_name: Some("EMGAS 3H".to_string()),
            transition_name: None,
            initial_or_final_restrictions: Vec::new(),
            provider_name: Some("CAICA".to_string()),
            is_source_required: false,
            source_required_note: None,
        },
        destination: OpenAiracAirportBrief {
            ident: "URFF".to_string(),
            iata_code: Some("SIP".to_string()),
            name: "Simferopol".to_string(),
            municipality: Some("Simferopol".to_string()),
            elevation_ft: Some(639.0),
            selected_runway: Some("19R".to_string()),
            procedure_name: Some("BURUD 2Y".to_string()),
            transition_name: None,
            initial_or_final_restrictions: Vec::new(),
            provider_name: Some("CAICA".to_string()),
            is_source_required: false,
            source_required_note: None,
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
            leg_index: 3,
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
            source_required_items: Vec::new(),
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
                status: "CURRENT".to_string(),
            },
        },
        navigation_warnings: Vec::new(),
    }
}

#[test]
fn test_ai_crew_questions_uuee_urff_cruise() {
    let mut runtime = AiCrewRuntime::new(Arc::new(DeterministicAiProvider));
    let snap = create_uuee_urff_snapshot();
    runtime.update_from_snapshot(&snap);

    // 1. Where are we?
    let r1 = runtime.ask("Where are we?").unwrap();
    assert!(r1.message.contains("CRUISE"));
    assert!(r1.message.contains("FL360"));
    assert!(r1.message.contains("460 kt"));
    assert_eq!(r1.tool_evidence.len(), 2);

    // 2. What are we flying now?
    let r2 = runtime.ask("What are we flying now?").unwrap();
    assert!(r2.message.contains("EMGAS -> BURUD"));
    assert!(r2.message.contains("BURUD"));

    // 3. What is the next fix?
    let r3 = runtime.ask("What's the next fix?").unwrap();
    assert!(r3.message.contains("BURUD"));
    assert!(r3.message.contains("84.2 NM"));

    // 4. When is TOD?
    let r4 = runtime.ask("When is TOD?").unwrap();
    assert!(r4.message.contains("42.5 NM"));
    assert!(r4.message.contains("estimated"));

    // 5. Brief the arrival
    let r5 = runtime.ask("Brief me for the arrival.").unwrap();
    assert!(r5.message.contains("URFF"));
    assert!(r5.message.contains("URFF 19012KT"));

    // 6. Is our data current?
    let r6 = runtime.ask("Is our data current?").unwrap();
    assert!(r6.message.contains("Telemetry is CURRENT"));
    assert!(r6.message.contains("Weather is CURRENT"));
}

#[test]
fn test_ai_crew_negative_canary_uras_source_required() {
    let mut runtime = AiCrewRuntime::new(Arc::new(DeterministicAiProvider));
    let mut snap_uras = create_uuee_urff_snapshot();
    snap_uras.destination.ident = "URAS".to_string();
    snap_uras.destination.name = "Sukhumi Babushara".to_string();
    snap_uras.destination.is_source_required = true;
    snap_uras.destination.procedure_name = None;

    runtime.update_from_snapshot(&snap_uras);

    let resp_star = runtime.ask("What STAR are we flying?").unwrap();
    assert!(resp_star.message.contains("SOURCE_REQUIRED"));
    assert!(resp_star.message.contains("URAS"));
    assert!(!resp_star.message.contains("BURUD 2Y")); // 0 procedure hallucinations!

    let resp_app = runtime.ask("What approach are we flying?").unwrap();
    assert!(resp_app.message.contains("SOURCE_REQUIRED"));
}

#[test]
fn test_ai_crew_independent_freshness_qualification() {
    let mut runtime = AiCrewRuntime::new(Arc::new(DeterministicAiProvider));
    let mut snap_stale = create_uuee_urff_snapshot();

    // Telemetry stale
    snap_stale.stale_flags.telemetry_stale = true;
    snap_stale.stale_flags.telemetry_age_ms = 8500;
    snap_stale.freshness_report.telemetry.status = "STALE".to_string();
    snap_stale.freshness_report.telemetry.age_ms = 8500;

    runtime.update_from_snapshot(&snap_stale);

    let resp_pos = runtime.ask("Where are we?").unwrap();
    assert!(resp_pos.message.contains("STALE"));
    assert!(resp_pos.freshness_qualification.is_some());
}

#[test]
fn test_ai_crew_failure_isolation_offline() {
    let mut runtime = AiCrewRuntime::default();
    // Context is None -> asking must return clean typed error, not panic
    let err = runtime.ask("Where are we?").unwrap_err();
    assert!(matches!(err, fd_crew::AiCrewError::RuntimeUnavailable));
}
