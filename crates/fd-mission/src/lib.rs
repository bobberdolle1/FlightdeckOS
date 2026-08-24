//! FlightdeckOS deterministic mission controller and route follower.
//!
//! The MissionController coordinates high-level phase progression for a
//! flight by issuing flight-guidance targets through the
//! [`FlightControlTargets`](fd_core::adapter::FlightControlTargets)
//! boundary. It contains NO simulator APIs, no physics, no SOP content —
//! phases delegate outward (SOP/control/ATC later).
//!
//! It must never become a god-object: it reads canonical state, decides the
//! current phase, and emits targets/events.

pub mod controller;
pub mod route;
pub mod shadow;

pub mod intents;
pub mod monitor;
pub mod runway;
pub use controller::{
    MissionCommands, MissionContext, MissionController, MissionParameters, MissionPhase,
    intended_commands, intended_next_phase,
};
pub use route::{RouteFollower, Waypoint};
pub use shadow::{ChannelStats, MissionShadow, ObservedApTargets, ShadowEntry, ShadowReport};
