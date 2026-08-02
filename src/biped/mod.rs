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
//! - [`profile`] -- per-machine constants (kyo46rs, Unitree G1).
//! - [`rig`]     -- URDF -> plant bring-up, and the per-tick state sync.
//! - [`contact`] -- contact anchors, contact Jacobians, sole-wrench selection.
//! - [`tasks`]   -- one builder per priority level of the hierarchical QP.
//! - [`actuate`] -- degraded-solve policy, fallback PD, and the plant write.
//! - [`log`]     -- the trajectory CSV the replay tooling reads.
//! - [`gait`]    -- support schedule and swing-foot trajectory.
//! - [`dcm`]     -- LIPM / divergent-component-of-motion balance reference.
//!
//! # Two things that must not be quietly removed
//!
//! The self-collision asserts in [`rig::BipedRig::build`] and the two-source
//! contact-force columns in [`log`] are not defensive boilerplate. Each one
//! caught a result that had already been reported as real: a 37.2 kN
//! self-brace that made single-leg stance "work", and a phantom 24 N
//! tangential reaction the QP was planning against. See
//! `doc/kyo46rs_biped_wbc.md`.

pub mod profile;

#[cfg(feature = "mujoco")]
pub mod rig;

pub mod contact;
pub mod dcm;
pub mod gait;
pub mod tasks;

#[cfg(feature = "mujoco")]
pub mod actuate;
#[cfg(feature = "mujoco")]
pub mod log;

pub use profile::Profile;
