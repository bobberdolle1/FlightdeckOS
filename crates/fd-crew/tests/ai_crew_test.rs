//! Unit and E2E integration tests for FlightdeckOS AI Crew Runtime.

use std::sync::Arc;

use fd_crew::{AiCrewRuntime, DeterministicAiProvider, SopAircraftBinding, SopBindingStatus};
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
            sid_procedure: Some("EMGAS 3H".to_string()),
            star_procedure: None,
            approach_procedure: None,
            approach_type: None,
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
            sid_procedure: None,
            star_procedure: Some("BURUD 2Y".to_string()),
            approach_procedure: Some("ILS 19R".to_string()),
            approach_type: Some("ILS".to_string()),
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
fn test_arrival_brief_semantic_roundtrip_uuee_urff() {
    let mut runtime = AiCrewRuntime::new(Arc::new(DeterministicAiProvider));
    let snap = create_uuee_urff_snapshot();
    runtime.update_from_snapshot(&snap);

    // 1. Verify semantic extraction in context
    let ctx = runtime.context().unwrap();
    assert_eq!(ctx.star_procedure, Some("BURUD 2Y".to_string()));
    assert_eq!(ctx.approach_procedure, Some("ILS 19R".to_string()));
    assert_eq!(ctx.approach_type, Some("ILS".to_string()));

    // 2. Ask for arrival brief
    let r_arr = runtime.ask("Brief me for the arrival.").unwrap();
    assert!(r_arr.message.contains("STAR BURUD 2Y"));
    assert!(r_arr.message.contains("Approach ILS 19R"));
    assert!(!r_arr.message.contains("Approach BURUD 2Y")); // No relabeling of STAR as Approach!
    assert!(r_arr.message.contains("URFF 19012KT"));
}

#[test]
fn test_ai_crew_negative_canary_uras_source_required() {
    let mut runtime = AiCrewRuntime::new(Arc::new(DeterministicAiProvider));
    let mut snap_uras = create_uuee_urff_snapshot();
    snap_uras.destination.ident = "URAS".to_string();
    snap_uras.destination.name = "Sukhumi Babushara".to_string();
    snap_uras.destination.is_source_required = true;
    snap_uras.destination.procedure_name = None;
    snap_uras.destination.star_procedure = None;
    snap_uras.destination.approach_procedure = None;
    snap_uras.destination.approach_type = None;

    runtime.update_from_snapshot(&snap_uras);

    let resp_star = runtime.ask("What STAR are we flying?").unwrap();
    assert!(resp_star.message.contains("SOURCE_REQUIRED"));
    assert!(resp_star.message.contains("URAS"));
    assert!(!resp_star.message.contains("BURUD 2Y")); // 0 procedure hallucinations!

    let resp_app = runtime.ask("What approach are we flying?").unwrap();
    assert!(resp_app.message.contains("SOURCE_REQUIRED"));
}

#[test]
fn test_aircraft_sop_package_isolation() {
    // 1. TU154 flight context -> a32nx SOP is strictly rejected as UNAVAILABLE
    let tu154_status = SopAircraftBinding::evaluate("TU154", None);
    assert!(matches!(tu154_status, SopBindingStatus::NotInstalled));

    let manifest = fd_aircraft::manifest::PackageManifest {
        package_id: "a32nx".to_string(),
        display_name: "FlyByWire A32NX".to_string(),
        aircraft_family: "Airbus A320 family".to_string(),
        simulator: "MSFS".to_string(),
        addon: "FlyByWire A32NX".to_string(),
        package_version: "0.1.0".to_string(),
        schema_version: 1,
        runtime_api_version: 1,
        addon_source_rev: "master".to_string(),
        live_verified: false,
        notes: "Test fixture".to_string(),
    };
    let a32nx_pkg = fd_sop::package::ValidatedPackage {
        manifest,
        roles: vec![
            fd_aircraft::roles::Role::Captain,
            fd_aircraft::roles::Role::FirstOfficer,
        ],
        flows: Vec::new(),
    };

    let mismatch_status = SopAircraftBinding::evaluate("TU154", Some(&a32nx_pkg));
    assert!(matches!(
        mismatch_status,
        SopBindingStatus::UnavailableForAircraft { .. }
    ));
    if let SopBindingStatus::UnavailableForAircraft { aircraft, reason } = mismatch_status {
        assert_eq!(aircraft, "TU154");
        assert!(reason.contains("No SOP package installed"));
    }

    // 2. A320 / A32NX flight context -> a32nx package is accepted
    let match_status = SopAircraftBinding::evaluate("A320", Some(&a32nx_pkg));
    assert!(matches!(match_status, SopBindingStatus::Active { .. }));
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
    let err = runtime.ask("Where are we?").unwrap_err();
    assert!(matches!(err, fd_crew::AiCrewError::RuntimeUnavailable));
}
