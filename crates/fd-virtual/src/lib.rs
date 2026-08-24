//! FlightdeckOS headless virtual simulator.
//!
//! **TEST MODEL — NOT REAL AIRCRAFT PERFORMANCE DATA.**
//!
//! Two independent layers:
//!
//! * **Semantic systems model** ([`systems`]): enough A320-style system
//!   behavior (APU spool, bleed, beacon...) to exercise procedure
//!   orchestration offline. Transitions have delays/guards; invalid actions
//!   are rejected. This is NOT the full Airbus simulation.
//! * **Kinematic flight model** ([`kinematics`]): bounded-rate state
//!   evolution (climb/descent/turn/acceleration limits) plus a route
//!   position integrator. No aerodynamics, no control laws, no engine
//!   thermodynamics.
//!
//! Determinism: the model advances by a FIXED simulated timestep. It never
//! reads the wall clock. Simulated hours run as fast as the CPU allows.
//!
//! What a virtual PASS proves: Mission/SOP/action ORCHESTRATION correctness.
//! What it does NOT prove: X-Plane/MSFS bindings, real aircraft behavior,
//! real flight dynamics.

pub mod adapter;
pub mod faults;

pub use adapter::VirtualSimulator;
pub use faults::{FaultConfig, MASKABLE_FIELDS};
pub use fd_core::adapter::FlightControlTargets;
pub mod kinematics;
pub mod systems;

/// Fixed simulated timestep used by the virtual simulator (ms).
///
/// 100 ms keeps tick counts manageable for multi-hour flights while being
/// fine-grained enough for bounded-rate integration in tests.
pub const DEFAULT_DT_MS: u64 = 100;

/// Virtual world clock: pure simulated time advanced by fixed steps.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct VirtualClock {
    /// Elapsed simulated milliseconds since scenario start.
    sim_ms: u64,
    /// Number of advance() calls (ticks).
    ticks: u64,
    pub dt_ms: u64,
}

impl VirtualClock {
    pub fn new(dt_ms: u64) -> Self {
        Self {
            sim_ms: 0,
            ticks: 0,
            dt_ms,
        }
    }

    /// Advance one fixed step.
    pub fn advance(&mut self) {
        self.sim_ms = self.sim_ms.saturating_add(self.dt_ms);
        self.ticks += 1;
    }

    /// Current simulated time in ms since start.
    pub const fn sim_ms(&self) -> u64 {
        self.sim_ms
    }

    /// Number of ticks since start.
    pub const fn ticks(&self) -> u64 {
        self.ticks
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn clock_is_pure_simulated_time() {
        let mut c = VirtualClock::new(100);
        assert_eq!(c.sim_ms(), 0);
        c.advance();
        c.advance();
        assert_eq!(c.sim_ms(), 200);
        assert_eq!(c.ticks(), 2);
    }
}
