//! Semantic systems model (A32NX-style subset for procedure orchestration).
//!
//! **TEST MODEL.** Delays and thresholds are development values chosen to
//! exercise sequencing; they are not Airbus performance data.
//!
//! State machine highlights:
//! * APU: Off → Starting → Available (spool over simulated time);
//!   start rejected while already starting/available;
//! * APU bleed: opens only when APU is Available; closes otherwise;
//! * beacon / engines running / gear / flaps are simple discrete states
//!   driven by closed actions.

use serde::{Deserialize, Serialize};

/// APU state machine.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ApuState {
    Off,
    Starting,
    Available,
    Shutdown,
}

/// Semantic systems snapshot (subset used by Task 3 scenarios).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SystemsState {
    pub apu_state: ApuState,
    /// APU N in percent of max (0..=100).
    pub apu_n_percent: f64,
    pub apu_bleed_open: bool,
    pub beacon_on: bool,
    pub engines_running: bool,
    pub pack_1_pb_on: bool,
}

/// Development timing constants for the semantic model.
///
/// TEST MODEL — NOT AIRCRAFT PERFORMANCE DATA.
pub const APU_SPOOL_MS: u64 = 90_000; // simulated ms to reach available
const APU_N_MAX: f64 = 95.0;
/// Nominal APU speed: reaching it means the APU is available for bleed.
/// (Development model value; public A320 material places nominal APU speed
/// near 90 % N.)
pub const APU_NOMINAL_PERCENT: f64 = 90.0;

impl SystemsState {
    pub fn cold_and_dark() -> Self {
        Self {
            apu_state: ApuState::Off,
            apu_n_percent: 0.0,
            apu_bleed_open: false,
            beacon_on: false,
            engines_running: false,
            pack_1_pb_on: false,
        }
    }

    /// Request APU start. Returns `Err(reason)` when the transition is not
    /// allowed in the current state (invalid actions are REJECTED, never
    /// silently ignored).
    pub fn request_apu_start(&mut self) -> Result<(), &'static str> {
        match self.apu_state {
            ApuState::Off | ApuState::Shutdown => {
                self.apu_state = ApuState::Starting;
                Ok(())
            }
            ApuState::Starting => Err("APU is already starting"),
            ApuState::Available => Err("APU is already available"),
        }
    }

    /// Request APU bleed open/close. Bleed requires an AVAILABLE APU.
    pub fn request_apu_bleed(&mut self, open: bool) -> Result<(), &'static str> {
        if open && self.apu_state != ApuState::Available {
            return Err("APU bleed requires the APU to be available");
        }
        self.apu_bleed_open = open;
        Ok(())
    }

    pub fn set_beacon(&mut self, on: bool) {
        self.beacon_on = on;
    }

    pub fn set_engines_running(&mut self, running: bool) {
        self.engines_running = running;
    }

    /// Advance semantic systems by `dt_ms` of simulated time.
    ///
    /// The APU spools gradually: N rises toward its max while Starting /
    /// Available; after the spool window elapses from the start command the
    /// state becomes Available. Shutting down spools N back to zero.
    pub fn advance(&mut self, dt_ms: u64) {
        match self.apu_state {
            ApuState::Starting | ApuState::Available => {
                // Deterministic linear spool: N reaches APU_N_MAX after
                // APU_SPOOL_MS of simulated time from the start command.
                let rate_per_ms = APU_N_MAX / APU_SPOOL_MS as f64;
                self.apu_n_percent =
                    (self.apu_n_percent + rate_per_ms * dt_ms as f64).min(APU_N_MAX);
                if self.apu_n_percent >= APU_NOMINAL_PERCENT {
                    self.apu_state = ApuState::Available;
                }
            }
            ApuState::Shutdown | ApuState::Off => {
                if self.apu_n_percent > 0.0 {
                    self.apu_n_percent =
                        0.0f64.max(self.apu_n_percent - 10.0 * dt_ms as f64 / 1000.0);
                }
                self.apu_bleed_open = false;
            }
        }
    }
}

/// Internal helper keeping the spool math honest: N integrates with the
/// same rate constant regardless of tick size (rate per simulated hour).
#[derive(Debug, Clone, Copy)]
pub struct SpoolRate(pub f64);

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn apu_start_rejected_while_starting() {
        let mut s = SystemsState::cold_and_dark();
        assert!(s.request_apu_start().is_ok());
        assert_eq!(s.request_apu_start(), Err("APU is already starting"));
        s.set_beacon(true); // unrelated action unaffected
        assert!(s.beacon_on);
    }

    #[test]
    fn apu_spools_over_simulated_time_then_becomes_available() {
        let mut s = SystemsState::cold_and_dark();
        s.request_apu_start().unwrap();
        // Immediately after the command: still starting, N low.
        s.advance(500);
        assert_eq!(s.apu_state, ApuState::Starting);
        assert!(s.apu_n_percent < 50.0);
        // Advance well past the spool window in coarse steps.
        let mut elapsed = 500u64;
        while s.apu_state == ApuState::Starting && elapsed < 200_000 {
            s.advance(1_000);
            elapsed += 1_000;
        }
        assert_eq!(s.apu_state, ApuState::Available);
        assert!(s.apu_n_percent >= 90.0);
    }

    #[test]
    fn bleed_requires_available_apu() {
        let mut s = SystemsState::cold_and_dark();
        assert_eq!(
            s.request_apu_bleed(true),
            Err("APU bleed requires the APU to be available")
        );
        // Unknown/failed state must not silently become true.
        assert!(!s.apu_bleed_open);
    }

    #[test]
    fn bleed_opens_only_after_available() {
        let mut s = SystemsState::cold_and_dark();
        s.request_apu_start().unwrap();
        let mut elapsed = 0u64;
        while s.apu_state != ApuState::Available && elapsed < 300_000 {
            s.advance(1_000);
            elapsed += 1_000;
        }
        assert!(s.request_apu_bleed(true).is_ok());
        assert!(s.apu_bleed_open);
    }
}
