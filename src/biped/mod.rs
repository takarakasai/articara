//! Whole-body control for a biped.
//!
//! # Why this is not [`crate::wbc_pipeline`]
//!
//! `WbcPipeline` and `quadruped_gait::wbc` are four-legged all the way down --
//! the leg count is spelled `[String; 4]` and `LegId` in a dozen places, so a
//! two-legged machine cannot be expressed in them at all. Rather than
//! generalise a working quadruped stack (and risk its measured behaviour) this
//! module drives misa-wbc's task catalogue and misarta's dynamics directly,
//! with the leg count left as `Vec` everywhere it appears.
//!
//! # Layout
//!
//! The simulator-free planners live in the `misa-biped` crate and the wrench
//! MPC in `misa-mpc` (lifted out 2026-10-29); they are re-exported here under
//! their old paths so `articara::biped::gait` etc. keep working.
//!
//! - [`profile`] -- per-machine constants (kyo46rs, Unitree G1). (misa-biped)
//! - [`rig`]     -- URDF -> plant bring-up, and the per-tick state sync.
//! - [`contact`] -- contact anchors, contact Jacobians, sole-wrench selection.
//! - [`tasks`]   -- one builder per priority level of the hierarchical QP.
//! - [`wbc_stack`] -- the `TaskStack`: the levels above, ordered, weighted and
//!   solved behind misa-biped's `WbcStack` trait.
//! - [`actuate`] -- degraded-solve policy, fallback PD, and the plant write.
//! - [`log`]     -- the trajectory CSV the replay tooling reads.
//! - [`gait`]    -- support schedule and swing-foot trajectory. (misa-biped)
//! - [`dcm`]     -- LIPM / divergent-component-of-motion balance reference. (misa-biped)
//! - [`wrench_mpc`] -- single-rigid-body wrench MPC. (misa-mpc)
//!
//! # Two things that must not be quietly removed
//!
//! The self-collision asserts in [`rig::BipedRig::build`] and the two-source
//! contact-force columns in [`log`] are not defensive boilerplate. Each one
//! caught a result that had already been reported as real: a 37.2 kN
//! self-brace that made single-leg stance "work", and a phantom 24 N
//! tangential reaction the QP was planning against. See
//! `doc/kyo46rs_biped_wbc.md`.

pub use misa_biped::{contact_plan, dcm, gait, height, preview, profile, reach, terrain};
pub use misa_mpc::wrench_mpc;

#[cfg(feature = "mujoco")]
pub mod rig;

pub mod contact;
pub mod tasks;
pub mod wbc_stack;

#[cfg(feature = "mujoco")]
pub mod actuate;
#[cfg(feature = "mujoco")]
pub mod log;

pub use profile::Profile;

#[cfg(feature = "mujoco")]
pub mod tick_audit;
