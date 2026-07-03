//! Shared helpers for the integration-test crates under `tests/`.
//!
//! Each `tests/<x>.rs` file is its own integration-test crate, so the
//! convention is to declare `mod common;` from each test file and pull
//! the helpers as needed. cargo special-cases `tests/common/mod.rs` so
//! it isn't compiled as a standalone test crate.
//!
//! - [`fixtures`] — fixture paths shared by the format / editor suites
//!   (always available).
//! - [`sim`] — MuJoCo sim builders / joint seeding for the gait-control
//!   suites (`mujoco` feature only; re-exported at this level so the
//!   existing `common::<helper>` call sites keep working).

pub mod fixtures;

#[cfg(feature = "mujoco")]
pub mod sim;
#[cfg(feature = "mujoco")]
#[allow(unused_imports)]
pub use sim::*;
