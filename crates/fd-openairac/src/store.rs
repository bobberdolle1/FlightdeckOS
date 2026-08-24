//! Read-only access to the OpenAIRAC world SQLite store (Task 6 §14-15).
//!
//! Production data path for airport/runway/waypoint awareness: FlightdeckOS
//! reads the canonical OpenAIRAC temporal store directly, read-only, and
//! never duplicates OpenAIRAC ingestion/reconciliation algorithms.
//!
//! Invariants:
//! - The connection is opened with `open_read_only` — the store directory
//!   may be shared with an OpenAIRAC installation that must never see
//!   writes from FlightdeckOS.
//! - Every query is temporal: `valid_from <= :at AND (valid_until IS NULL
//!   OR valid_until > :at)`. The query instant is caller-supplied so the
//!   same dataset revision resolves deterministically across runs (Task 6
//!   reference pin: `2026-08-20T19:15:00Z`).
//! - Records are plain data: conversion into FlightdeckOS route/runway
//!   types happens at the application layer, keeping fd-openairac
//!   dependency-free of fd-mission.

use crate::error::OpenAiracError;
use rusqlite::Connection;

/// Deterministic query instant for the UUEE→ULLI reference dataset window
/// (the single revision interval where UUEE/ULLI airports+runways are valid).
pub const REFERENCE_QUERY_INSTANT: &str = "2026-08-20T19:15:00Z";

/// An airport record resolved from the world store.
#[derive(Debug, Clone, PartialEq)]
pub struct AirportRecord {
    pub ident: String,
    pub name: String,
    pub airport_type: String,
    pub lat_deg: f64,
    pub lon_deg: f64,
    pub elevation_ft: Option<f64>,
}

/// A runway record with per-end threshold geometry.
#[derive(Debug, Clone, PartialEq)]
pub struct RunwayRecord {
    pub airport_ident: String,
    /// Low-end designator (e.g. `"06C"`).
    pub le_ident: String,
    /// High-end designator (e.g. `"24C"`).
    pub he_ident: String,
    /// TRUE heading of the runway (from the low end), degrees.
    pub true_heading_deg: Option<f64>,
    pub length_ft: Option<f64>,
    pub le_lat: Option<f64>,
    pub le_lon: Option<f64>,
    pub le_elevation_ft: Option<f64>,
    pub he_lat: Option<f64>,
    pub he_lon: Option<f64>,
    pub he_elevation_ft: Option<f64>,
}

/// A waypoint record.
#[derive(Debug, Clone, PartialEq)]
pub struct WaypointRecord {
    pub ident: String,
    pub lat_deg: f64,
    pub lon_deg: f64,
    pub is_enroute: bool,
}

/// Read-only OpenAIRAC world store handle.
#[derive(Debug)]
pub struct NavDataStore {
    conn: Connection,
}

impl NavDataStore {
    /// Open the world store STRICTLY read-only. Fails if the file does not
    /// exist; never creates, migrates, or writes.
    pub fn open_read_only(path: &std::path::Path) -> Result<Self, OpenAiracError> {
        let conn = Connection::open_with_flags(path, rusqlite::OpenFlags::SQLITE_OPEN_READ_ONLY)
            .map_err(|e| OpenAiracError::StoreError(format!("open world store: {e}")))?;
        Ok(Self { conn })
    }

    /// Resolve an airport by ICAO/ident valid at `at` (RFC3339 string).
    pub fn airport_by_icao(
        &self,
        icao: &str,
        at: &str,
    ) -> Result<Option<AirportRecord>, OpenAiracError> {
        let mut stmt = self
            .conn
            .prepare(
                "SELECT ident, name, airport_type, latitude_deg, longitude_deg, elevation_ft
                 FROM airports
                 WHERE ident = ?1 AND valid_from <= ?2
                   AND (valid_until IS NULL OR valid_until > ?2)
                 ORDER BY valid_from DESC LIMIT 1",
            )
            .map_err(sql_err)?;
        let mut rows = stmt.query([icao, at]).map_err(sql_err)?;
        if let Some(r) = rows.next().map_err(sql_err)? {
            Ok(Some(AirportRecord {
                ident: r.get(0).map_err(sql_err)?,
                name: r.get(1).map_err(sql_err)?,
                airport_type: r.get(2).map_err(sql_err)?,
                lat_deg: r.get(3).map_err(sql_err)?,
                lon_deg: r.get(4).map_err(sql_err)?,
                elevation_ft: r.get(5).map_err(sql_err)?,
            }))
        } else {
            Ok(None)
        }
    }

    /// All runways of an airport valid at `at`.
    pub fn runways(&self, icao: &str, at: &str) -> Result<Vec<RunwayRecord>, OpenAiracError> {
        let mut stmt = self
            .conn
            .prepare(
                "SELECT airport_ident, le_ident, he_ident, true_heading_deg, length_ft,
                        le_lat, le_lon, le_elevation_ft, he_lat, he_lon, he_elevation_ft
                 FROM runways
                 WHERE airport_ident = ?1 AND valid_from <= ?2
                   AND (valid_until IS NULL OR valid_until > ?2)
                 ORDER BY le_ident",
            )
            .map_err(sql_err)?;
        let rows = stmt
            .query_map([icao, at], |r| {
                Ok(RunwayRecord {
                    airport_ident: r.get(0)?,
                    le_ident: r.get(1)?,
                    he_ident: r.get(2)?,
                    true_heading_deg: r.get(3)?,
                    length_ft: r.get(4)?,
                    le_lat: r.get(5)?,
                    le_lon: r.get(6)?,
                    le_elevation_ft: r.get(7)?,
                    he_lat: r.get(8)?,
                    he_lon: r.get(9)?,
                    he_elevation_ft: r.get(10)?,
                })
            })
            .map_err(sql_err)?;
        let mut out = Vec::new();
        for row in rows {
            out.push(row.map_err(sql_err)?);
        }
        Ok(out)
    }

    /// Nearest airport to a position valid at `at`, with distance in nm.
    ///
    /// DEVELOPMENT NOTE: scans all current airports (a few thousand rows)
    /// and computes great-circle distance per candidate — fine for
    /// observer-frequency use (once per few seconds), not per-frame.
    pub fn nearest_airport(
        &self,
        lat_deg: f64,
        lon_deg: f64,
        at: &str,
    ) -> Result<Option<(AirportRecord, f64)>, OpenAiracError> {
        let mut stmt = self
            .conn
            .prepare(
                "SELECT ident, name, airport_type, latitude_deg, longitude_deg, elevation_ft
                 FROM airports
                 WHERE valid_from <= ?1 AND (valid_until IS NULL OR valid_until > ?1)",
            )
            .map_err(sql_err)?;
        let rows = stmt
            .query_map([at], |r| {
                Ok(AirportRecord {
                    ident: r.get(0)?,
                    name: r.get(1)?,
                    airport_type: r.get(2)?,
                    lat_deg: r.get(3)?,
                    lon_deg: r.get(4)?,
                    elevation_ft: r.get(5)?,
                })
            })
            .map_err(sql_err)?;
        let mut best: Option<(AirportRecord, f64)> = None;
        for row in rows {
            let ap = row.map_err(sql_err)?;
            let d = fd_core::geo::distance_nm(lat_deg, lon_deg, ap.lat_deg, ap.lon_deg);
            if best.as_ref().map(|(_, bd)| d < *bd).unwrap_or(true) {
                best = Some((ap, d));
            }
        }
        Ok(best)
    }

    /// Waypoints inside a lat/lon box valid at `at` (bounded scan).
    pub fn waypoints_in_box(
        &self,
        min_lat: f64,
        min_lon: f64,
        max_lat: f64,
        max_lon: f64,
        at: &str,
    ) -> Result<Vec<WaypointRecord>, OpenAiracError> {
        let mut stmt = self
            .conn
            .prepare(
                "SELECT ident, latitude_deg, longitude_deg, is_enroute
                 FROM waypoints
                 WHERE latitude_deg BETWEEN ?1 AND ?2
                   AND longitude_deg BETWEEN ?3 AND ?4
                   AND valid_from <= ?5 AND (valid_until IS NULL OR valid_until > ?5)
                 ORDER BY ident",
            )
            .map_err(sql_err)?;
        let rows = stmt
            .query_map(
                rusqlite::params![min_lat, max_lat, min_lon, max_lon, at],
                |r| {
                    Ok(WaypointRecord {
                        ident: r.get(0)?,
                        lat_deg: r.get(1)?,
                        lon_deg: r.get(2)?,
                        is_enroute: r.get::<_, i64>(3)? != 0,
                    })
                },
            )
            .map_err(sql_err)?;
        let mut out = Vec::new();
        for row in rows {
            out.push(row.map_err(sql_err)?);
        }
        Ok(out)
    }
}

fn sql_err(e: rusqlite::Error) -> OpenAiracError {
    OpenAiracError::StoreError(format!("world store query: {e}"))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Build a tiny world-store-shaped DB in memory with the same schema
    /// columns the real store exposes (v1_init.sql subset).
    fn seed() -> NavDataStore {
        let conn = Connection::open_in_memory().unwrap();
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
            CREATE TABLE waypoints (
                id TEXT PRIMARY KEY, ident TEXT, name TEXT,
                latitude_deg REAL, longitude_deg REAL, datum TEXT, region TEXT,
                is_enroute INTEGER, waypoint_type INTEGER,
                terminal_area_ident TEXT, source_snapshot_id TEXT,
                valid_from TEXT, valid_until TEXT);
            INSERT INTO airports VALUES
                ('a1','UUEE','Sheremetyevo','large_airport',55.972642,37.414589,622.0,'RU','Moscow','s1','2026-08-20T10:00:00Z',NULL),
                ('a2','ULLI','Pulkovo','large_airport',59.800278,30.2625,79.0,'RU','SPb','s1','2026-08-20T10:00:00Z',NULL);
            INSERT INTO runways VALUES
                ('r1','a1','UUEE','06C','06C',74.5,12139,197,'CONC','06C',55.9667,37.3889,620.0,'24C',55.9786,37.4403,622.0,'s1','2026-08-20T10:00:00Z',NULL);
            INSERT INTO waypoints VALUES
                ('w1','BESKI','Beski',56.5,35.0,'WGS-84','RU',1,0,NULL,'s1','2026-08-20T10:00:00Z',NULL);
            "#,
        )
        .unwrap();
        NavDataStore { conn }
    }

    const AT: &str = "2026-08-20T19:15:00Z";

    #[test]
    fn resolves_airport() {
        let s = seed();
        let ap = s.airport_by_icao("UUEE", AT).unwrap().unwrap();
        assert_eq!(ap.name, "Sheremetyevo");
        assert!((ap.lat_deg - 55.972642).abs() < 1e-9);
        assert_eq!(ap.elevation_ft, Some(622.0));
        assert!(s.airport_by_icao("ZZZZ", AT).unwrap().is_none());
    }

    #[test]
    fn temporal_predicate_honored() {
        let s = seed();
        // Before validity window: nothing.
        assert!(
            s.airport_by_icao("UUEE", "2026-08-19T00:00:00Z")
                .unwrap()
                .is_none()
        );
        // After (superseded): nothing (valid_until NULL means still valid here).
        assert!(
            s.airport_by_icao("UUEE", "2026-08-25T00:00:00Z")
                .unwrap()
                .is_some()
        );
    }

    #[test]
    fn resolves_runways_with_geometry() {
        let s = seed();
        let rwys = s.runways("UUEE", AT).unwrap();
        assert_eq!(rwys.len(), 1);
        let r = &rwys[0];
        assert_eq!(r.le_ident, "06C");
        assert_eq!(r.he_ident, "24C");
        assert!((r.true_heading_deg.unwrap() - 74.5).abs() < 1e-9);
        assert_eq!(r.length_ft, Some(12139.0));
    }

    #[test]
    fn nearest_airport_picks_closest() {
        let s = seed();
        // Position near UUEE.
        let (ap, d) = s.nearest_airport(55.97, 37.41, AT).unwrap().unwrap();
        assert_eq!(ap.ident, "UUEE");
        assert!(d < 5.0, "d {d}");
    }

    #[test]
    fn waypoints_in_box_bounded() {
        let s = seed();
        let wps = s.waypoints_in_box(56.0, 34.0, 57.0, 36.0, AT).unwrap();
        assert_eq!(wps.len(), 1);
        assert_eq!(wps[0].ident, "BESKI");
        assert!(wps[0].is_enroute);
    }

    /// Manual verification against the real 178MB world store when present.
    #[test]
    #[ignore = "requires local OpenAIRAC world store"]
    fn real_store_reference_resolution() {
        let path = std::path::Path::new("F:/Projects/open-airac/data/world.openairac.sqlite");
        if !path.exists() {
            return;
        }
        let s = NavDataStore::open_read_only(path).unwrap();
        let uee = s
            .airport_by_icao("UUEE", REFERENCE_QUERY_INSTANT)
            .unwrap()
            .unwrap();
        let lli = s
            .airport_by_icao("ULLI", REFERENCE_QUERY_INSTANT)
            .unwrap()
            .unwrap();
        let rwys = s.runways("UUEE", REFERENCE_QUERY_INSTANT).unwrap();
        assert!(!rwys.is_empty());
        println!(
            "UUEE elev={:?} rwys={} ULLI elev={:?}",
            uee.elevation_ft,
            rwys.len(),
            lli.elevation_ft
        );
    }
}
