//! Crew-role primitive (minimal for Task 2).
//!
//! Full crew model (personality, authority ladder, PF/PM) comes later; this
//! is only the ownership tag a procedure item carries.

use serde::{Deserialize, Serialize};

use crate::error::PackageError;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Role {
    Captain,
    FirstOfficer,
    User,
}

/// Strict name resolution for package data.
pub fn role_from_name(name: &str) -> Result<Role, PackageError> {
    // Package data is human-edited TOML: accept any casing of the canonical
    // snake_case names.
    let canon = name.to_ascii_lowercase().replace('_', "");
    match canon.as_str() {
        "captain" => Ok(Role::Captain),
        "firstofficer" => Ok(Role::FirstOfficer),
        "user" => Ok(Role::User),
        other => Err(PackageError::UnknownRole(other.to_string())),
    }
}
