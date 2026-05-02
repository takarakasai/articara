//! MuJoCo runtime version check.
//!
//! `mujoco-rs` asserts on every model construction that the linked
//! MuJoCo runtime version matches the version its FFI bindings were
//! generated against. The assertion **panics** deep inside the crate
//! the first time `MjModel::*` is touched, which lands on the user as
//! a backtrace from `~/.cargo/registry/src/.../mujoco-rs-x.y.z/src/util.rs`
//! with no actionable advice.
//!
//! This module provides a Result-returning pre-check we call ourselves
//! at startup so the mismatch surfaces as a clear log line and a UI
//! status rather than a panic. The check is cheap (one FFI call to
//! `mj_version`).
//!
//! Strategy:
//! 1. Call once at app startup ([`crate::main`]) and log a clear
//!    error if mismatched. The app keeps running so non-MuJoCo
//!    features (URDF / `.misa` view, IK, gait, etc.) remain usable.
//! 2. Cache the result via [`is_runtime_compatible`] so any code
//!    that's about to call MuJoCo can short-circuit with a friendly
//!    message instead of triggering the panic.

use std::sync::OnceLock;

/// Decoded MuJoCo version (e.g. "3.8.0").
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MujocoVersion {
    pub major: u32,
    pub minor: u32,
    pub patch: u32,
}

impl MujocoVersion {
    /// Decode the integer encoding MuJoCo uses internally
    /// (`MAJOR * 1_000_000 + MINOR * 1_000 + PATCH`).
    fn from_packed(v: u32) -> Self {
        Self {
            major: v / 1_000_000,
            minor: (v / 1_000) % 1_000,
            patch: v % 1_000,
        }
    }

    pub fn packed(self) -> u32 {
        self.major * 1_000_000 + self.minor * 1_000 + self.patch
    }
}

impl std::fmt::Display for MujocoVersion {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}.{}.{}", self.major, self.minor, self.patch)
    }
}

/// Outcome of [`check`].
#[derive(Debug, Clone)]
pub enum CheckResult {
    /// Linked runtime version matches what `mujoco-rs` was generated
    /// against — MuJoCo is safe to use.
    Compatible(MujocoVersion),
    /// Runtime version differs from what `mujoco-rs` expects. Calling
    /// any `MjModel::*` constructor will panic; callers must avoid
    /// MuJoCo and surface a friendly message instead.
    Mismatch {
        linked: MujocoVersion,
        expected: MujocoVersion,
    },
}

impl CheckResult {
    pub fn is_compatible(&self) -> bool {
        matches!(self, CheckResult::Compatible(_))
    }

    /// Human-readable diagnostic. Shown both in the startup log and
    /// in the dynamics panel when MuJoCo is unavailable.
    pub fn diagnostic(&self) -> String {
        match self {
            CheckResult::Compatible(v) => {
                format!("MuJoCo runtime {} matches expected version", v)
            }
            CheckResult::Mismatch { linked, expected } => {
                format!(
                    "MuJoCo version mismatch: linked runtime is {linked} but \
                     `mujoco-rs` was generated against {expected}. Calling \
                     any MuJoCo function would panic. Install MuJoCo {expected} \
                     and re-export MUJOCO_DYNAMIC_LINK_DIR (or run via \
                     `cargo xtask` for auto-detection). See MUJOCO_SETUP.md."
                )
            }
        }
    }
}

/// Check the linked runtime version against what `mujoco-rs` expects.
/// Cheap: one FFI call to `mj_version()` (a pure query, no allocations,
/// safe to call before any other MuJoCo code).
pub fn check() -> CheckResult {
    // SAFETY: mj_version is a pure read-only query into the loaded
    // libmujoco; no global init required, no thread-local state.
    let linked_packed = unsafe { mujoco::mujoco_c::mj_version() } as u32;
    let expected_packed = mujoco::mujoco_c::mjVERSION_HEADER;

    let linked = MujocoVersion::from_packed(linked_packed);
    let expected = MujocoVersion::from_packed(expected_packed);

    if linked_packed == expected_packed {
        CheckResult::Compatible(linked)
    } else {
        CheckResult::Mismatch { linked, expected }
    }
}

// ─── Cached startup check ─────────────────────────────────────────────────

static CACHED: OnceLock<CheckResult> = OnceLock::new();

/// Run the version check once and cache the result for the rest of the
/// process. Idempotent — subsequent calls return the cached value.
///
/// Call this from `main()` immediately after logger init so the result
/// shows up in stderr before any MuJoCo-touching code runs.
pub fn init() -> &'static CheckResult {
    CACHED.get_or_init(check)
}

/// Cached version-compatibility flag for hot paths (rendering, dynamics
/// panel) that need to gate MuJoCo features without re-querying. Returns
/// `true` if [`init`] has not been called yet (best-effort: avoid
/// hiding MuJoCo if we don't know).
pub fn is_runtime_compatible() -> bool {
    CACHED
        .get()
        .map_or(true, |r| r.is_compatible())
}

/// Get the cached check result. Returns `None` if [`init`] hasn't run.
pub fn cached() -> Option<&'static CheckResult> {
    CACHED.get()
}

// ─── Tests ─────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn version_packed_round_trip() {
        let v = MujocoVersion { major: 3, minor: 8, patch: 0 };
        assert_eq!(v.packed(), 3_008_000);
        let back = MujocoVersion::from_packed(3_008_000);
        assert_eq!(back, v);
    }

    #[test]
    fn version_display() {
        let v = MujocoVersion::from_packed(3_006_000);
        assert_eq!(v.to_string(), "3.6.0");
    }

    #[test]
    fn diagnostic_contains_both_versions() {
        let r = CheckResult::Mismatch {
            linked: MujocoVersion::from_packed(3_006_000),
            expected: MujocoVersion::from_packed(3_008_000),
        };
        let msg = r.diagnostic();
        assert!(msg.contains("3.6.0"), "diagnostic missing linked: {msg}");
        assert!(msg.contains("3.8.0"), "diagnostic missing expected: {msg}");
        assert!(msg.contains("MUJOCO_DYNAMIC_LINK_DIR") || msg.contains("xtask"));
    }
}
