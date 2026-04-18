//! Rigid Body Dynamics (`rbd`) module.
//!
//! Consolidates robot body structure, kinematics, and dynamics into a
//! single namespace that can later be extracted into an independent crate.
//!
//! # Sub-modules
//!
//! - [`model`] — Body structure: data types for links, joints, geometry,
//!   inertial properties, FK, and tree navigation.
//! - [`kinematics`] — Kinematic chains, Jacobian computation, IK solver.
//! - [`dynamics`] — Gravity torques, payload capacity, forward dynamics (WIP).

pub mod model;
pub mod kinematics;
pub mod dynamics;
pub mod adapter;
