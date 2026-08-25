//! OpenAIRAC Integration Client & Canonical Crew Context for FlightdeckOS.
//!
//! Provides typed connection, schema negotiation, multi-source freshness tracking,
//! and canonical crew context mapping from the released OpenAIRAC 3.2 Gateway.

pub mod client;
pub mod context;
pub mod error;
pub mod match_nav;
pub mod store;
pub mod types;

pub use client::{DEFAULT_GATEWAY_URL, OpenAiracClient};
pub use context::{CrewFlightContext, SourceOwnershipTable, SubsystemFreshness};
pub use error::OpenAiracError;
pub use store::{
    AirportRecord, NavDataStore, REFERENCE_QUERY_INSTANT, RunwayRecord, WaypointRecord,
};
pub use types::{
    CompactFreshness, EXPECTED_COMPACT_SCHEMA, EXPECTED_SNAPSHOT_V2_SCHEMA, OpenAiracActiveLeg,
    OpenAiracAircraftProfile, OpenAiracAirportBrief, OpenAiracArrivalBrief,
    OpenAiracCompactSnapshot, OpenAiracConstraint, OpenAiracDataProvenance,
    OpenAiracDepartureBrief, OpenAiracDescentProfile, OpenAiracEvent, OpenAiracFreshnessReport,
    OpenAiracNavGeometry, OpenAiracNavdataFreshness, OpenAiracOnlineAtc,
    OpenAiracOnlineAtcFreshness, OpenAiracPosition, OpenAiracProviderIdentity,
    OpenAiracResolvedIdentity, OpenAiracRunwayWind, OpenAiracSnapshotV2, OpenAiracStaleFlags,
    OpenAiracTelemetryFreshness, OpenAiracWeatherFreshness, OpenAiracWeatherSummary,
};
