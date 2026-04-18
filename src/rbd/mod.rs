//! Rigid Body Dynamics (`rbd`) module.
//!
//! Consolidates robot body structure, kinematics, and dynamics into a
//! single namespace that can later be extracted into an independent crate.
//!
//! # Sub-modules
//!
//! - [`model`] — Body structure: data types for links, joints, geometry,
//!   inertial properties, FK, tree navigation, and misarta integration.
//! - [`dynamics`] — Gravity torques, payload capacity, forward dynamics.

pub mod model;
pub mod dynamics;
