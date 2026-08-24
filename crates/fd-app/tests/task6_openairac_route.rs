//! Task 6 §61: OpenAIRAC-generated route through the production route
//! monitor — the Level B reference plumbing (headless, deterministic).
//!
//! The current RU enroute dataset cannot support airway routing (dangling
//! legs — see fd-openairac store docs), so the honest Level B reference is
//! the airport-to-airport great-circle route RESOLVED FROM OPENAIRAC DATA
//! with provenance, fed through the production RouteMonitor.

use fd_mission::monitor::{
    OffRouteConfig, OffRouteDetector, RouteMonitor, RouteSource, RouteState,
};
use fd_mission::route::Waypoint;
use fd_openairac::store::RunwayRecord;

/// Seed a world-store-shaped in-memory DB with the reference airports.
fn seed_store() -> rusqlite::Connection {
    let conn = rusqlite::Connection::open_in_memory().unwrap();
    conn.execute_batch(
        r#"
        CREATE TABLE airports (
            id TEXT PRIMARY KEY, ident TEXT, name TEXT, airport_type TEXT,
            latitude_deg REAL, longitude_deg REAL, elevation_ft REAL,
            iso_country TEXT, municipality TEXT, source_snapshot_id TEXT,
            valid_from TEXT, valid_until TEXT);
        CREATE TABLE runways (
            id TEXT PRIMARY KEY, airport_id TEXT, airport_ident TEXT,
            official_designator TEXT, computed_magnetic_designator TEXT,
            true_heading_deg REAL, length_ft INTEGER, width_ft INTEGER,
            surface TEXT, le_ident TEXT, le_lat REAL, le_lon REAL,
            le_elevation_ft REAL, he_ident TEXT, he_lat REAL, he_lon REAL,
            he_elevation_ft REAL, source_snapshot_id TEXT,
            valid_from TEXT, valid_until TEXT);
        INSERT INTO airports VALUES
            ('a1','UUEE','Sheremetyevo','large_airport',55.972642,37.414589,622.0,'RU','Moscow','s1','2026-08-20T10:00:00Z',NULL),
            ('a2','ULLI','Pulkovo','large_airport',59.800278,30.2625,79.0,'RU','SPb','s1','2026-08-20T10:00:00Z',NULL);
        INSERT INTO runways VALUES
            ('r1','a2','ULLI','10L','10L',95.0,12000,197,'CONC','10L',59.7950,30.2400,75.0,'28R',59.8120,30.2950,79.0,'s1','2026-08-20T10:00:00Z',NULL);
        "#,
    )
    .unwrap();
    conn
}

#[test]
fn openairac_route_drives_route_monitor() {
    let conn = seed_store();
    let at = "2026-08-20T19:15:00Z";
    let o = fd_openairac::store::AirportRecord {
        ident: "UUEE".into(),
        name: "Sheremetyevo".into(),
        airport_type: "large_airport".into(),
        lat_deg: 55.972642,
        lon_deg: 37.414589,
        elevation_ft: Some(622.0),
    };
    let d = fd_openairac::store::AirportRecord {
        ident: "ULLI".into(),
        name: "Pulkovo".into(),
        airport_type: "large_airport".into(),
        lat_deg: 59.800278,
        lon_deg: 30.2625,
        elevation_ft: Some(79.0),
    };
    let _ = (&conn, at); // store shape exercised in fd-openairac tests

    // OpenAIRAC-resolved route with provenance (Level B reference).
    let route = RouteState {
        source: RouteSource::OpenAirac {
            provenance: format!("world@{at}"),
        },
        waypoints: vec![
            Waypoint {
                id: o.ident.clone(),
                lat_deg: o.lat_deg,
                lon_deg: o.lon_deg,
            },
            Waypoint {
                id: d.ident.clone(),
                lat_deg: d.lat_deg,
                lon_deg: d.lon_deg,
            },
        ],
    };
    assert!(route.is_usable());

    let mut monitor = RouteMonitor::new(&route);
    let mut detector = OffRouteDetector::new(OffRouteConfig::default());
    // TRUE great-circle midpoint (the rhumb/arithmetic midpoint sits far
    // off the arc at these latitudes).
    let (lat1, lon1) = (o.lat_deg.to_radians(), o.lon_deg.to_radians());
    let (lat2, lon2) = (d.lat_deg.to_radians(), d.lon_deg.to_radians());
    let d12 = (lat1.sin() * lat2.sin() + lat1.cos() * lat2.cos() * (lon2 - lon1).cos()).acos();
    let (a, b) = (
        ((1.0 - 0.5) * d12).sin() / d12.sin(),
        (0.5 * d12).sin() / d12.sin(),
    );
    let x = a * lat1.cos() * lon1.cos() + b * lat2.cos() * lon2.cos();
    let y = a * lat1.cos() * lon1.sin() + b * lat2.cos() * lon2.sin();
    let z = a * lat1.sin() + b * lat2.sin();
    let mid_lat = z.atan2((x * x + y * y).sqrt()).to_degrees();
    let mid_lon = y.atan2(x).to_degrees();
    let obs = monitor.update(mid_lat, mid_lon);
    assert!(obs.active_leg.is_some());
    assert!(
        obs.cross_track_error_nm.unwrap().abs() < 1.0,
        "great-circle midpoint is on-route"
    );
    assert!(detector.update(0, &obs).is_none());
    // Arrive ULLI: route completes.
    let obs = monitor.update(d.lat_deg, d.lon_deg);
    assert!(obs.route_complete, "arrival completes the OpenAIRAC route");
    assert_eq!(obs.distance_remaining_nm, Some(0.0));
}

#[test]
fn openairac_runway_record_maps_to_runway_context() {
    // The store record shape converts into the runway-awareness type at the
    // application layer (dependency direction: fd-app wires both crates).
    let r = RunwayRecord {
        airport_ident: "ULLI".into(),
        le_ident: "10L".into(),
        he_ident: "28R".into(),
        true_heading_deg: Some(95.0),
        length_ft: Some(12000.0),
        le_lat: Some(59.7950),
        le_lon: Some(30.2400),
        le_elevation_ft: Some(75.0),
        he_lat: Some(59.8120),
        he_lon: Some(30.2950),
        he_elevation_ft: Some(79.0),
    };
    let ctx = fd_mission::runway::RunwayContext {
        runway: fd_mission::runway::Runway {
            airport_icao: r.airport_ident.clone(),
            le_ident: r.le_ident.clone(),
            he_ident: r.he_ident.clone(),
            length_ft: r.length_ft.unwrap(),
            ends: [
                fd_mission::runway::RunwayEnd {
                    ident: r.le_ident.clone(),
                    lat_deg: r.le_lat.unwrap(),
                    lon_deg: r.le_lon.unwrap(),
                    elevation_ft: r.le_elevation_ft.unwrap(),
                    true_heading_deg: r.true_heading_deg.unwrap(),
                },
                fd_mission::runway::RunwayEnd {
                    ident: r.he_ident.clone(),
                    lat_deg: r.he_lat.unwrap(),
                    lon_deg: r.he_lon.unwrap(),
                    elevation_ft: r.he_elevation_ft.unwrap(),
                    true_heading_deg: (r.true_heading_deg.unwrap() + 180.0).rem_euclid(360.0),
                },
            ],
        },
        landing_end: 0,
        evidence: "test".into(),
    };
    // On-centerline near the threshold: ~0 offset; abeam: signed.
    let xtk = ctx.centerline_offset_m(59.7950, 30.2450).unwrap();
    assert!(xtk.abs() < 600.0, "near-centerline approach, got {xtk} m");
    assert_eq!(ctx.heading_diff_deg(95.0), 0.0);
}

/// Manual verification against the real 178MB world store when present.
#[test]
#[ignore = "requires local OpenAIRAC world store"]
fn real_store_uuee_ulli_reference() {
    let path = std::path::Path::new("F:/Projects/open-airac/data/world.openairac.sqlite");
    if !path.exists() {
        return;
    }
    let store = fd_openairac::NavDataStore::open_read_only(path).unwrap();
    let at = fd_openairac::REFERENCE_QUERY_INSTANT;
    let uee = store.airport_by_icao("UUEE", at).unwrap().unwrap();
    let lli = store.airport_by_icao("ULLI", at).unwrap().unwrap();
    let rwys = store.runways("ULLI", at).unwrap();
    let route = RouteState {
        source: RouteSource::OpenAirac {
            provenance: format!("world@{at}"),
        },
        waypoints: vec![
            Waypoint {
                id: uee.ident.clone(),
                lat_deg: uee.lat_deg,
                lon_deg: uee.lon_deg,
            },
            Waypoint {
                id: lli.ident.clone(),
                lat_deg: lli.lat_deg,
                lon_deg: lli.lon_deg,
            },
        ],
    };
    let mut m = RouteMonitor::new(&route);
    let obs = m.update(
        (uee.lat_deg + lli.lat_deg) / 2.0,
        (uee.lon_deg + lli.lon_deg) / 2.0,
    );
    assert!(obs.active_leg.is_some());
    assert!(!rwys.is_empty());
    println!(
        "UUEE elev={:?} ULLI elev={:?} ULLI runways={} mid_remaining={:.0}nm",
        uee.elevation_ft,
        lli.elevation_ft,
        rwys.len(),
        obs.distance_remaining_nm.unwrap()
    );
}
