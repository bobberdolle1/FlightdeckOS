//! Typed condition model with tri-state evaluation.
//!
//! Evaluation distinguishes `True / False / Unknown`: missing simulator data
//! yields `Unknown`, which never satisfies a procedure item. Nothing is ever
//! guessed.
//!
//! Type mismatches (boolean field with a numeric comparison, etc.) are
//! rejected at package-validation time by [`RawCondition::validate`]; the
//! typed [`Condition`] makes them unrepresentable.

use serde::{Deserialize, Serialize};

use crate::error::PackageError;
use crate::state_field::{StateField, ValueType};
use fd_core::telemetry::TelemetrySnapshot;

/// Tri-state condition result.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum TriBool {
    True,
    False,
    Unknown,
}

impl TriBool {
    /// A procedure item may only complete on `True`.
    pub const fn satisfies(self) -> bool {
        matches!(self, Self::True)
    }
}

/// Typed, validated condition.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Condition {
    /// Boolean field is observed true.
    IsTrue { field: StateField },
    /// Boolean field is observed false.
    IsFalse { field: StateField },
    /// Numeric field equals the value exactly.
    Equals { field: StateField, value: f64 },
    /// Numeric field >= value.
    AtLeast { field: StateField, value: f64 },
    /// Numeric field <= value.
    AtMost { field: StateField, value: f64 },
    /// Field is known at all (data present).
    Known { field: StateField },
}

impl Condition {
    pub const fn field(&self) -> StateField {
        match self {
            Self::IsTrue { field }
            | Self::IsFalse { field }
            | Self::Equals { field, .. }
            | Self::AtLeast { field, .. }
            | Self::AtMost { field, .. }
            | Self::Known { field } => *field,
        }
    }

    /// Evaluate against a canonical snapshot. Pure and deterministic.
    pub fn evaluate(&self, snap: &TelemetrySnapshot) -> TriBool {
        match self {
            Self::IsTrue { field } => match typed_value(*field, snap) {
                Some(LeafValue::Bool(b)) => tri(b),
                _ => TriBool::Unknown,
            },
            Self::IsFalse { field } => match typed_value(*field, snap) {
                Some(LeafValue::Bool(b)) => tri(!b),
                _ => TriBool::Unknown,
            },
            Self::Equals { field, value } => match numeric_of(*field, snap) {
                Some(v) => tri(v == *value),
                None => TriBool::Unknown,
            },
            Self::AtLeast { field, value } => match numeric_of(*field, snap) {
                Some(v) => tri(v >= *value),
                None => TriBool::Unknown,
            },
            Self::AtMost { field, value } => match numeric_of(*field, snap) {
                Some(v) => tri(v <= *value),
                None => TriBool::Unknown,
            },
            Self::Known { field } => match typed_value(*field, snap) {
                Some(_) => TriBool::True,
                None => TriBool::False,
            },
        }
    }

    /// Validate + build from raw package data (fail-closed type checks).
    pub fn from_raw(raw: &RawCondition) -> Result<Self, PackageError> {
        let field = StateField::from_name(&raw.field)?;
        let op = raw.op.as_str();
        let mismatch = |detail: String| PackageError::ConditionTypeMismatch {
            field: raw.field.clone(),
            op: op.to_string(),
            detail,
        };
        match op {
            "is_true" => {
                if field.value_type() != ValueType::Boolean {
                    return Err(mismatch("is_true requires a boolean field".into()));
                }
                Ok(Self::IsTrue { field })
            }
            "is_false" => {
                if field.value_type() != ValueType::Boolean {
                    return Err(mismatch("is_false requires a boolean field".into()));
                }
                Ok(Self::IsFalse { field })
            }
            "known" => Ok(Self::Known { field }),
            "equals" | "at_least" | "at_most" => {
                if field.value_type() != ValueType::Numeric {
                    return Err(mismatch(format!("`{op}` requires a numeric field")));
                }
                let value = raw
                    .value
                    .ok_or_else(|| mismatch(format!("`{op}` requires an explicit `value`")))?;
                if !value.is_finite() {
                    return Err(mismatch("value must be finite".into()));
                }
                Ok(match op {
                    "equals" => Self::Equals { field, value },
                    "at_least" => Self::AtLeast { field, value },
                    _ => Self::AtMost { field, value },
                })
            }
            other => Err(PackageError::ConditionTypeMismatch {
                field: raw.field.clone(),
                op: other.to_string(),
                detail: "unknown operator".into(),
            }),
        }
    }
}

const fn tri(b: bool) -> TriBool {
    if b { TriBool::True } else { TriBool::False }
}

enum LeafValue {
    Bool(bool),
    Num(f64),
}

fn typed_value(field: StateField, snap: &TelemetrySnapshot) -> Option<LeafValue> {
    match field {
        StateField::OnGround => snap.on_ground.map(LeafValue::Bool),
        StateField::BeaconLight => snap.beacon_light.map(LeafValue::Bool),
        StateField::ApuBleedValveOpen | StateField::Pack1PbOn => {
            let id = field.ext_id()?;
            as_bool(*snap.aircraft_values.get(&id)?).map(LeafValue::Bool)
        }
        StateField::ApuNPercent | StateField::FlapsHandleIndex | StateField::NavLogoSwitch => {
            let id = field.ext_id()?;
            let v = *snap.aircraft_values.get(&id)?;
            if v.is_finite() {
                Some(LeafValue::Num(v))
            } else {
                None
            }
        }
    }
}

fn numeric_of(field: StateField, snap: &TelemetrySnapshot) -> Option<f64> {
    match typed_value(field, snap)? {
        LeafValue::Num(v) => Some(v),
        LeafValue::Bool(_) => None,
    }
}

fn as_bool(v: f64) -> Option<bool> {
    (v.is_finite()).then_some(v != 0.0)
}

/// Raw (unvalidated) condition as written in package TOML.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct RawCondition {
    pub field: String,
    pub op: String,
    #[serde(default)]
    pub value: Option<f64>,
}

// Re-exported alias for readability in flow files.
pub use RawCondition as RawConditionToml;

/// Free-function evaluator (method form: `Condition::evaluate`).
#[cfg(test)]
pub fn evaluate(cond: &Condition, snap: &TelemetrySnapshot) -> TriBool {
    cond.evaluate(snap)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::BTreeMap;

    fn snap(values: &[(u16, f64)], beacon: Option<bool>) -> TelemetrySnapshot {
        let mut s = TelemetrySnapshot::empty(fd_core::telemetry::SimTimestamp::new(0));
        s.beacon_light = beacon;
        s.aircraft_values = values.iter().copied().collect::<BTreeMap<u16, f64>>();
        s
    }

    #[test]
    fn known_true_and_false_are_distinguished() {
        let c = Condition::IsTrue {
            field: StateField::BeaconLight,
        };
        assert_eq!(evaluate(&c, &snap(&[], Some(true))), TriBool::True);
        assert_eq!(evaluate(&c, &snap(&[], Some(false))), TriBool::False);
        assert_eq!(evaluate(&c, &snap(&[], None)), TriBool::Unknown);
    }

    #[test]
    fn unknown_numeric_never_satisfies_threshold() {
        let c = Condition::AtLeast {
            field: StateField::ApuNPercent,
            value: 90.0,
        };
        // Absent key -> Unknown, not False-as-true.
        assert_eq!(evaluate(&c, &snap(&[], None)), TriBool::Unknown);
        assert_eq!(evaluate(&c, &snap(&[(1, 95.0)], None)), TriBool::True);
        assert_eq!(evaluate(&c, &snap(&[(1, 50.0)], None)), TriBool::False);
    }

    #[test]
    fn known_condition_reports_data_presence() {
        let c = Condition::Known {
            field: StateField::BeaconLight,
        };
        assert_eq!(evaluate(&c, &snap(&[], Some(false))), TriBool::True);
        assert_eq!(evaluate(&c, &snap(&[], None)), TriBool::False);
    }

    #[test]
    fn raw_validation_rejects_type_mismatches() {
        let raw_is_true_on_number = RawCondition {
            field: "apu_n_percent".into(),
            op: "is_true".into(),
            value: None,
        };
        assert!(matches!(
            Condition::from_raw(&raw_is_true_on_number),
            Err(PackageError::ConditionTypeMismatch { .. })
        ));

        let raw_at_least_on_bool = RawCondition {
            field: "beacon_light".into(),
            op: "at_least".into(),
            value: Some(1.0),
        };
        assert!(matches!(
            Condition::from_raw(&raw_at_least_on_bool),
            Err(PackageError::ConditionTypeMismatch { .. })
        ));

        let raw_unknown_field = RawCondition {
            field: "warp_drive_engaged".into(),
            op: "is_true".into(),
            value: None,
        };
        assert!(matches!(
            Condition::from_raw(&raw_unknown_field),
            Err(PackageError::UnknownStateField(_))
        ));

        let raw_unknown_op = RawCondition {
            field: "beacon_light".into(),
            op: "exceeds".into(),
            value: Some(1.0),
        };
        assert!(matches!(
            Condition::from_raw(&raw_unknown_op),
            Err(PackageError::ConditionTypeMismatch { .. })
        ));
    }
}
