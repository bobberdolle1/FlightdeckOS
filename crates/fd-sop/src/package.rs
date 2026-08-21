//! Package loading + semantic resolution.
//!
//! `load_package(dir)` performs the full fail-closed pipeline:
//!
//! 1. manifest identity/version validation (fd-aircraft);
//! 2. role registry load;
//! 3. bindings provenance metadata validation (closed name set);
//! 4. every `procedures/*.toml` parsed and resolved:
//!    duplicate flow/step ids, unknown roles / state fields / actions,
//!    missing or self dependencies, dependency cycles — all rejected.
//!
//! Output is a fully resolved [`ValidatedPackage`] whose flows reference
//! only closed typed actions and validated conditions.

use std::path::Path;

use fd_aircraft::condition::{Condition, RawConditionToml};
use fd_aircraft::error::PackageError;
use fd_aircraft::manifest::PackageManifest;
use fd_aircraft::raw_flow::{RawFlowDef, RawStep, RawStepBody};
use fd_aircraft::roles::{Role, role_from_name};
use fd_core::actions::CockpitAction;

/// A resolved flow step: everything already validated and typed.
#[derive(Debug, Clone)]
pub struct StepDef {
    pub id: String,
    pub actor: Role,
    pub depends_on: Vec<String>,
    pub kind: StepKind,
}

/// Two supported step kinds (Task 2).
#[derive(Debug, Clone)]
pub enum StepKind {
    /// Completes when the condition evaluates to True against observed state.
    Observe { condition: Condition },
    /// Requests a closed cockpit action; completes when the runtime action
    /// pipeline observes the action's post-condition.
    Action { action: CockpitAction },
}

/// A resolved flow definition.
#[derive(Debug, Clone)]
pub struct FlowDefinition {
    pub id: String,
    pub title: String,
    pub scope_note: String,
    pub steps: Vec<StepDef>,
}

/// Fully validated package.
#[derive(Debug, Clone)]
pub struct ValidatedPackage {
    pub manifest: PackageManifest,
    pub roles: Vec<Role>,
    pub flows: Vec<FlowDefinition>,
}

/// Typed SOP/package errors. Wraps aircraft-layer errors unchanged so the
/// fail-closed origin stays visible.
#[derive(Debug, thiserror::Error)]
pub enum SopError {
    #[error(transparent)]
    Package(#[from] PackageError),
    #[error("package io error: {0}")]
    Io(String),
}

fn resolve_condition(raw: &RawConditionToml) -> Result<Condition, SopError> {
    Ok(Condition::from_raw(raw)?)
}

fn resolve_step(flow_id: &str, raw: &RawStep) -> Result<StepDef, SopError> {
    let actor = role_from_name(&raw.actor)?;
    let mut depends_on = Vec::new();
    for dep in &raw.depends_on {
        if dep == &raw.id {
            return Err(SopError::Package(PackageError::SelfDependency {
                flow: flow_id.to_string(),
                step: raw.id.clone(),
            }));
        }
        depends_on.push(dep.clone());
    }
    if raw.id.trim().is_empty() {
        return Err(SopError::Package(PackageError::EmptyField {
            field: "step id",
        }));
    }
    let kind = match &raw.body {
        RawStepBody::Observe { condition } => StepKind::Observe {
            condition: resolve_condition(condition)?,
        },
        RawStepBody::Action { action } => {
            let action = CockpitAction::try_from_name(action)
                .ok_or_else(|| SopError::Package(PackageError::UnknownAction(action.clone())))?;
            StepKind::Action { action }
        }
    };
    Ok(StepDef {
        id: raw.id.clone(),
        actor,
        depends_on,
        kind,
    })
}

fn resolve_flow(raw: &RawFlowDef) -> Result<FlowDefinition, SopError> {
    if raw.id.trim().is_empty() || raw.title.trim().is_empty() {
        return Err(SopError::Package(PackageError::EmptyField {
            field: "flow id/title",
        }));
    }
    if raw.steps.is_empty() {
        return Err(SopError::Package(PackageError::EmptyField {
            field: "flow steps",
        }));
    }
    let mut steps = Vec::new();
    let mut seen = std::collections::BTreeSet::new();
    for rs in &raw.steps {
        if !seen.insert(rs.id.clone()) {
            return Err(SopError::Package(PackageError::DuplicateStepId {
                flow: raw.id.clone(),
                step: rs.id.clone(),
            }));
        }
        steps.push(resolve_step(&raw.id, rs)?);
    }
    // Missing dependency check.
    for st in &steps {
        for dep in &st.depends_on {
            if !seen.contains(dep) {
                return Err(SopError::Package(PackageError::MissingDependency {
                    flow: raw.id.clone(),
                    step: st.id.clone(),
                    dep: dep.clone(),
                }));
            }
        }
    }
    // Cycle check: iterative DFS with visit states (0=unvisited,1=visiting,2=done).
    let index_of = |id: &str| steps.iter().position(|s| s.id == id).unwrap();
    let mut state = vec![0u8; steps.len()];
    let mut stack_path: Vec<usize> = Vec::new();
    for root in 0..steps.len() {
        if state[root] != 0 {
            continue;
        }
        let mut stack: Vec<(usize, usize)> = vec![(root, 0)];
        while let Some(&mut (node, ref mut child_i)) = stack.last_mut() {
            let deps_len = steps[node].depends_on.len();
            if *child_i == 0 {
                if state[node] == 1 {
                    let pos = stack_path.iter().position(|&n| n == node).unwrap_or(0);
                    let cycle_ids: Vec<String> = stack_path[pos..]
                        .iter()
                        .chain(std::iter::once(&node))
                        .map(|&n| steps[n].id.clone())
                        .collect();
                    return Err(SopError::Package(PackageError::DependencyCycle {
                        flow: raw.id.clone(),
                        cycle: cycle_ids.join(" -> "),
                    }));
                }
                state[node] = 1;
                stack_path.push(node);
            }
            if *child_i < deps_len {
                let dep = steps[node].depends_on[*child_i].clone();
                *child_i += 1;
                let next_idx = index_of(&dep);
                if state[next_idx] == 1 {
                    return Err(SopError::Package(PackageError::DependencyCycle {
                        flow: raw.id.clone(),
                        cycle: format!("{} -> {}", steps[node].id, dep),
                    }));
                }
                if state[next_idx] == 0 {
                    stack.push((next_idx, 0));
                }
                continue;
            }
            state[node] = 2;
            stack.pop();
            stack_path.pop();
        }
    }
    Ok(FlowDefinition {
        id: raw.id.clone(),
        title: raw.title.clone(),
        scope_note: raw.scope_note.clone(),
        steps,
    })
}

/// Load a package directory: manifest + roles + bindings metadata +
/// procedures/*.toml with full semantic validation.
pub fn load_package(dir: &Path) -> Result<ValidatedPackage, SopError> {
    let manifest = fd_aircraft::manifest::load_manifest(dir).map_err(SopError::from)?;

    let roles_path = dir.join("roles.toml");
    let roles_text = std::fs::read_to_string(&roles_path)
        .map_err(|e| SopError::Io(format!("{}: {e}", roles_path.display())))?;
    #[derive(serde::Deserialize)]
    struct RolesFile {
        roles: Vec<String>,
    }
    let roles_file: RolesFile =
        toml::from_str(&roles_text).map_err(|e| SopError::Io(e.to_string()))?;
    let mut roles = Vec::new();
    for r in &roles_file.roles {
        roles.push(role_from_name(r)?);
    }

    let bindings_path = dir.join("bindings.toml");
    let bindings_text = std::fs::read_to_string(&bindings_path)
        .map_err(|e| SopError::Io(format!("{}: {e}", bindings_path.display())))?;
    #[derive(serde::Deserialize)]
    struct BindingsFile {
        reads: Vec<String>,
    }
    let bf: BindingsFile =
        toml::from_str(&bindings_text).map_err(|e| SopError::Io(e.to_string()))?;
    fd_aircraft::bindings_meta::validate_binding_names(&bf.reads).map_err(SopError::from)?;

    let proc_dir = dir.join("procedures");
    let mut entries: Vec<std::path::PathBuf> = std::fs::read_dir(&proc_dir)
        .map_err(|e| SopError::Io(format!("{}: {e}", proc_dir.display())))?
        .filter_map(|e| e.ok())
        .map(|e| e.path())
        .filter(|p| p.extension().map(|x| x == "toml").unwrap_or(false))
        .collect();
    entries.sort(); // deterministic load order

    let mut flows = Vec::new();
    let mut seen_flow_ids = std::collections::BTreeSet::new();
    for path in entries {
        let text = std::fs::read_to_string(&path)
            .map_err(|e| SopError::Io(format!("{}: {e}", path.display())))?;
        let raw: RawFlowDef =
            toml::from_str(&text).map_err(|e| SopError::Io(format!("{path:?}: {e}")))?;
        if !seen_flow_ids.insert(raw.id.clone()) {
            return Err(SopError::Package(PackageError::DuplicateFlowId(
                raw.id.clone(),
            )));
        }
        flows.push(resolve_flow(&raw)?);
    }

    Ok(ValidatedPackage {
        manifest,
        roles,
        flows,
    })
}

// re-export StateField for downstream convenience (fd-app CLI reporting).
pub use fd_aircraft::state_field::StateField as _StateFieldMarker;

// Silence unused import warning for Role re-export used in doc context.
#[allow(unused_imports)]
use Role as _RoleForDocs;

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Write as _;

    fn write_pkg(root: &Path, files: &[(&str, &str)]) -> PathBuf {
        for (rel, content) in files {
            let p = root.join(rel);
            if let Some(parent) = p.parent() {
                std::fs::create_dir_all(parent).unwrap();
            }
            let mut f = std::fs::File::create(p).unwrap();
            f.write_all(content.as_bytes()).unwrap();
        }
        root.to_path_buf()
    }

    const MANIFEST: &str = r#"
package_id = "a32nx"
display_name = "FlyByWire A32NX"
aircraft_family = "Airbus A320 family"
simulator = "MSFS"
addon = "FlyByWire A32NX"
package_version = "0.1.0"
schema_version = 1
runtime_api_version = 1
"#;
    const ROLES: &str = "roles = [\"Captain\", \"FirstOfficer\"]\n";
    const BINDINGS: &str = "reads = [\n  \"apu_n_percent\",\n  \"apu_bleed_valve_open\",\n]\n";

    const FLOW_OK: &str = r#"
id = "before_start"
title = "Before Start (architecture slice)"
scope_note = "demonstration slice; NOT a certified airline procedure"

[[steps]]
id = "beacon_on"
actor = "FirstOfficer"
kind = "action"
action = "set_beacon_on"

[[steps]]
id = "apu_available"
actor = "FirstOfficer"
kind = "observe"
[steps.condition]
field = "apu_n_percent"
op = "at_least"
value = 90.0

[[steps]]
id = "apu_bleed_open"
actor = "FirstOfficer"
depends_on = ["apu_available"]
kind = "observe"
[steps.condition]
field = "apu_bleed_valve_open"
op = "is_true"
"#;

    use std::path::PathBuf;

    #[test]
    fn valid_package_loads_with_resolved_flows() {
        let dir = tempfile::tempdir().unwrap();
        let pkg = write_pkg(
            dir.path(),
            &[
                ("manifest.toml", MANIFEST),
                ("roles.toml", ROLES),
                ("bindings.toml", BINDINGS),
                ("procedures/before_start.toml", FLOW_OK),
            ],
        );
        let p = load_package(&pkg).unwrap();
        assert_eq!(p.manifest.package_id, "a32nx");
        assert_eq!(p.flows.len(), 1);
        assert_eq!(p.flows[0].steps.len(), 3);
        // Action step resolves into a typed closed action.
        assert!(matches!(
            p.flows[0].steps[0].kind,
            StepKind::Action {
                action: CockpitAction::SetBeacon(_)
            }
        ));
    }

    #[test]
    fn duplicate_flow_id_rejected_across_files() {
        let dir = tempfile::tempdir().unwrap();
        let pkg = write_pkg(
            dir.path(),
            &[
                ("manifest.toml", MANIFEST),
                ("roles.toml", ROLES),
                ("bindings.toml", BINDINGS),
                ("procedures/a.toml", FLOW_OK),
                ("procedures/b.toml", FLOW_OK), // same id
            ],
        );
        assert!(matches!(
            load_package(&pkg),
            Err(SopError::Package(PackageError::DuplicateFlowId(_)))
        ));
    }

    #[test]
    fn unknown_role_rejected() {
        let bad = FLOW_OK.replace("actor = \"FirstOfficer\"", "actor = \"Purser\"");
        let dir = tempfile::tempdir().unwrap();
        let pkg = write_pkg(
            dir.path(),
            &[
                ("manifest.toml", MANIFEST),
                ("roles.toml", ROLES),
                ("bindings.toml", BINDINGS),
                ("procedures/a.toml", &bad),
            ],
        );
        assert!(matches!(
            load_package(&pkg),
            Err(SopError::Package(PackageError::UnknownRole(_)))
        ));
    }

    #[test]
    fn unknown_action_rejected() {
        let bad = FLOW_OK.replace(
            "action = \"set_beacon_on\"",
            "action = \"engage_warp_drive\"",
        );
        let dir = tempfile::tempdir().unwrap();
        let pkg = write_pkg(
            dir.path(),
            &[
                ("manifest.toml", MANIFEST),
                ("roles.toml", ROLES),
                ("bindings.toml", BINDINGS),
                ("procedures/a.toml", &bad),
            ],
        );
        assert!(matches!(
            load_package(&pkg),
            Err(SopError::Package(PackageError::UnknownAction(_)))
        ));
    }

    #[test]
    fn self_dependency_rejected() {
        let bad = FLOW_OK.replace(
            "depends_on = [\"apu_available\"]",
            "depends_on = [\"apu_bleed_open\"]",
        );
        let dir = tempfile::tempdir().unwrap();
        let pkg = write_pkg(
            dir.path(),
            &[
                ("manifest.toml", MANIFEST),
                ("roles.toml", ROLES),
                ("bindings.toml", BINDINGS),
                ("procedures/a.toml", &bad),
            ],
        );
        assert!(matches!(
            load_package(&pkg),
            Err(SopError::Package(PackageError::SelfDependency { .. }))
        ));
    }

    #[test]
    fn missing_dependency_rejected() {
        let bad = FLOW_OK.replace(
            "depends_on = [\"apu_available\"]",
            "depends_on = [\"nonexistent_step\"]",
        );
        let dir = tempfile::tempdir().unwrap();
        let pkg = write_pkg(
            dir.path(),
            &[
                ("manifest.toml", MANIFEST),
                ("roles.toml", ROLES),
                ("bindings.toml", BINDINGS),
                ("procedures/a.toml", &bad),
            ],
        );
        assert!(matches!(
            load_package(&pkg),
            Err(SopError::Package(PackageError::MissingDependency { .. }))
        ));
    }

    #[test]
    fn dependency_cycle_rejected() {
        let cyclic = r#"
id = "before_start"
title = "cyclic demo"
scope_note = "negative test"

[[steps]]
id = "a"
actor = "FirstOfficer"
depends_on = ["b"]
kind = "observe"
[steps.condition]
field = "beacon_light"
op = "is_true"

[[steps]]
id = "b"
actor = "FirstOfficer"
depends_on = ["a"]
kind = "observe"
[steps.condition]
field = "on_ground"
op = "is_true"
"#;
        let dir = tempfile::tempdir().unwrap();
        let pkg = write_pkg(
            dir.path(),
            &[
                ("manifest.toml", MANIFEST),
                ("roles.toml", ROLES),
                ("bindings.toml", BINDINGS),
                ("procedures/a.toml", cyclic),
            ],
        );
        assert!(matches!(
            load_package(&pkg),
            Err(SopError::Package(PackageError::DependencyCycle { .. }))
        ));
    }

    #[test]
    fn duplicate_step_id_rejected() {
        let dup = FLOW_OK.replace("id = \"apu_available\"", "id = \"beacon_on\"");
        let dir = tempfile::tempdir().unwrap();
        let pkg = write_pkg(
            dir.path(),
            &[
                ("manifest.toml", MANIFEST),
                ("roles.toml", ROLES),
                ("bindings.toml", BINDINGS),
                ("procedures/a.toml", &dup),
            ],
        );
        assert!(matches!(
            load_package(&pkg),
            Err(SopError::Package(PackageError::DuplicateStepId { .. }))
        ));
    }

    #[test]
    fn unknown_state_field_rejected_in_condition() {
        let bad = FLOW_OK.replace("field = \"apu_n_percent\"", "field = \"cabin_pressure\"");
        let dir = tempfile::tempdir().unwrap();
        let pkg = write_pkg(
            dir.path(),
            &[
                ("manifest.toml", MANIFEST),
                ("roles.toml", ROLES),
                ("bindings.toml", BINDINGS),
                ("procedures/a.toml", &bad),
            ],
        );
        assert!(matches!(
            load_package(&pkg),
            Err(SopError::Package(PackageError::UnknownStateField(_)))
        ));
    }

    #[test]
    fn wrong_condition_type_rejected() {
        let bad = FLOW_OK
            .replace("op = \"at_least\"", "op = \"is_true\"")
            .replace("value = 90.0", "");
        let dir = tempfile::tempdir().unwrap();
        let pkg = write_pkg(
            dir.path(),
            &[
                ("manifest.toml", MANIFEST),
                ("roles.toml", ROLES),
                ("bindings.toml", BINDINGS),
                ("procedures/a.toml", &bad),
            ],
        );
        assert!(matches!(
            load_package(&pkg),
            Err(SopError::Package(
                PackageError::ConditionTypeMismatch { .. }
            ))
        ));
    }

    #[test]
    fn unsupported_schema_version_rejected_via_loader() {
        let bad = MANIFEST.replace("schema_version = 1", "schema_version = 42");
        let dir = tempfile::tempdir().unwrap();
        let pkg = write_pkg(
            dir.path(),
            &[
                ("manifest.toml", &bad),
                ("roles.toml", ROLES),
                ("bindings.toml", BINDINGS),
                ("procedures/a.toml", FLOW_OK),
            ],
        );
        assert!(matches!(
            load_package(&pkg),
            Err(SopError::Package(PackageError::SchemaVersion { .. }))
        ));
    }

    #[test]
    fn unknown_binding_name_rejected() {
        let bad = BINDINGS.replace("\"apu_n_percent\",", "\"warp_coil_percent\",");
        let dir = tempfile::tempdir().unwrap();
        let pkg = write_pkg(
            dir.path(),
            &[
                ("manifest.toml", MANIFEST),
                ("roles.toml", ROLES),
                ("bindings.toml", &bad),
                ("procedures/a.toml", FLOW_OK),
            ],
        );
        assert!(matches!(
            load_package(&pkg),
            Err(SopError::Package(PackageError::UnknownBindingName(_)))
        ));
    }
}
