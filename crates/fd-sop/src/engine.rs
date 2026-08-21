//! Deterministic flow engine.
//!
//! Consumes canonical snapshots plus action outcomes and produces typed
//! requests + events. It NEVER performs simulator writes itself: action
//! steps produce [`CockpitAction`] requests that the host routes through
//! the existing runtime action pipeline; a step completes only when that
//! pipeline reports the action VERIFIED (observed post-condition).

use fd_aircraft::condition::TriBool;
use fd_core::actions::{ActionId, ActionStatus, CockpitAction};
use fd_core::telemetry::TelemetrySnapshot;

use crate::package::{FlowDefinition, StepKind};

/// SOP-level events (mapped onto the runtime trace by the host).
#[derive(Debug, Clone, PartialEq)]
pub enum SopEvent {
    FlowStarted {
        flow: String,
    },
    StepReady {
        flow: String,
        step: String,
    },
    /// Observe step evaluated but not yet satisfied (emitted once).
    StepWaitingForVerification {
        flow: String,
        step: String,
    },
    StepActionRequested {
        flow: String,
        step: String,
        action: CockpitAction,
    },
    StepVerified {
        flow: String,
        step: String,
    },
    StepFailed {
        flow: String,
        step: String,
        reason: String,
    },
    FlowCompleted {
        flow: String,
    },
    FlowFailed {
        flow: String,
    },
}

/// One requested action emitted by the engine.
#[derive(Debug, Clone, PartialEq)]
pub struct FlowActionRequest {
    pub step_id: String,
    pub action: CockpitAction,
}

/// Output of one engine pass.
#[derive(Debug, Default)]
pub struct FlowTickOutput {
    pub events: Vec<SopEvent>,
    pub requests: Vec<FlowActionRequest>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StepStatus {
    Pending,
    Ready,
    WaitingForVerification,
    Verified,
    Failed,
    Aborted,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FlowStatus {
    NotStarted,
    Running,
    Completed,
    Failed,
}

struct StepState {
    status: StepStatus,
    action_id: Option<ActionId>,
    waiting_emitted: bool,
}

/// Deterministic flow instance driver for one flow definition.
pub struct FlowEngine {
    flow: FlowDefinition,
    steps: Vec<StepState>,
    status: FlowStatus,
    started: bool,
}

impl FlowEngine {
    pub fn new(flow: FlowDefinition) -> Self {
        let steps = flow
            .steps
            .iter()
            .map(|_| StepState {
                status: StepStatus::Pending,
                action_id: None,
                waiting_emitted: false,
            })
            .collect();
        Self {
            flow,
            steps,
            status: FlowStatus::NotStarted,
            started: false,
        }
    }

    pub fn flow_id(&self) -> &str {
        &self.flow.id
    }

    pub fn status(&self) -> FlowStatus {
        self.status
    }

    /// Assign the runtime action id for a step's request (host calls this
    /// right after submitting through the action pipeline).
    pub fn assign_action_id(&mut self, step_id: &str, id: ActionId) {
        if let Some((idx, _)) = self
            .flow
            .steps
            .iter()
            .enumerate()
            .find(|(_, s)| s.id == step_id)
        {
            self.steps[idx].action_id = Some(id);
        }
    }

    /// Report terminal action outcomes (drained from the runtime pipeline).
    /// `Verified` completes the step; any failure fails step AND flow.
    pub fn report_outcomes(
        &mut self,
        outcomes: &[(ActionId, ActionStatus)],
        out: &mut FlowTickOutput,
    ) {
        for (id, status) in outcomes {
            for (idx, st) in self.steps.iter_mut().enumerate() {
                if st.action_id.as_ref() == Some(id)
                    && st.status == StepStatus::WaitingForVerification
                {
                    let step_id = self.flow.steps[idx].id.clone();
                    match status {
                        ActionStatus::Verified => {
                            st.status = StepStatus::Verified;
                            out.events.push(SopEvent::StepVerified {
                                flow: self.flow.id.clone(),
                                step: step_id,
                            });
                        }
                        ActionStatus::Failed(f) => {
                            st.status = StepStatus::Failed;
                            out.events.push(SopEvent::StepFailed {
                                flow: self.flow.id.clone(),
                                step: step_id,
                                reason: format!("action failed: {f:?}"),
                            });
                        }
                        ActionStatus::Rejected(r) => {
                            st.status = StepStatus::Failed;
                            out.events.push(SopEvent::StepFailed {
                                flow: self.flow.id.clone(),
                                step: step_id,
                                reason: format!("action rejected: {r:?}"),
                            });
                        }
                        ActionStatus::TimedOut => {
                            st.status = StepStatus::Failed;
                            out.events.push(SopEvent::StepFailed {
                                flow: self.flow.id.clone(),
                                step: step_id,
                                reason: "verification timed out".into(),
                            });
                        }
                        // Requested/Validated/Dispatched are NOT completion.
                        _ => {}
                    }
                }
            }
        }
        self.recheck_flow_completion(&mut out.events);
    }

    /// One deterministic pass over the flow against the current snapshot.
    pub fn process(&mut self, snap: &TelemetrySnapshot) -> FlowTickOutput {
        let mut out = FlowTickOutput::default();
        if self.status == FlowStatus::Failed || self.status == FlowStatus::Completed {
            return out;
        }
        if !self.started {
            self.started = true;
            self.status = FlowStatus::Running;
            out.events.push(SopEvent::FlowStarted {
                flow: self.flow.id.clone(),
            });
        }

        // Verified set for dependency resolution.
        let verified: Vec<bool> = self
            .steps
            .iter()
            .map(|s| s.status == StepStatus::Verified)
            .collect();
        let position_of = |id: &str| self.flow.steps.iter().position(|s| s.id == id);

        for idx in 0..self.flow.steps.len() {
            let def = &self.flow.steps[idx];
            let st = &mut self.steps[idx];

            if st.status == StepStatus::Pending {
                let deps_ok = def
                    .depends_on
                    .iter()
                    .all(|d| position_of(d).map(|i| verified[i]).unwrap_or(false));
                if deps_ok {
                    st.status = StepStatus::Ready;
                    out.events.push(SopEvent::StepReady {
                        flow: self.flow.id.clone(),
                        step: def.id.clone(),
                    });
                } else {
                    continue;
                }
            }

            if self.steps[idx].status != StepStatus::Ready {
                continue;
            }

            match &def.kind {
                StepKind::Observe { condition } => {
                    match condition.evaluate(snap) {
                        TriBool::True => {
                            self.steps[idx].status = StepStatus::Verified;
                            out.events.push(SopEvent::StepVerified {
                                flow: self.flow.id.clone(),
                                step: def.id.clone(),
                            });
                        }
                        TriBool::False | TriBool::Unknown => {
                            // Unknown NEVER satisfies; emit waiting exactly once.
                            if !self.steps[idx].waiting_emitted {
                                self.steps[idx].waiting_emitted = true;
                                out.events.push(SopEvent::StepWaitingForVerification {
                                    flow: self.flow.id.clone(),
                                    step: def.id.clone(),
                                });
                            }
                        }
                    }
                }
                StepKind::Action { action } => {
                    self.steps[idx].status = StepStatus::WaitingForVerification;
                    out.events.push(SopEvent::StepActionRequested {
                        flow: self.flow.id.clone(),
                        step: def.id.clone(),
                        action: *action,
                    });
                    out.requests.push(FlowActionRequest {
                        step_id: def.id.clone(),
                        action: *action,
                    });
                }
            }
        }

        self.recheck_flow_completion(&mut out.events);
        out
    }

    fn recheck_flow_completion(&mut self, events: &mut Vec<SopEvent>) {
        if self.status != FlowStatus::Running {
            return;
        }
        if self.steps.iter().any(|s| s.status == StepStatus::Failed) {
            self.status = FlowStatus::Failed;
            // Abort non-terminal steps (documented minimal failure behavior).
            for s in &mut self.steps {
                if !matches!(
                    s.status,
                    StepStatus::Failed | StepStatus::Verified | StepStatus::Aborted
                ) {
                    s.status = StepStatus::Aborted;
                }
            }
            events.push(SopEvent::FlowFailed {
                flow: self.flow.id.clone(),
            });
            return;
        }
        if self.steps.iter().all(|s| s.status == StepStatus::Verified) {
            self.status = FlowStatus::Completed;
            events.push(SopEvent::FlowCompleted {
                flow: self.flow.id.clone(),
            });
        }
    }
}
