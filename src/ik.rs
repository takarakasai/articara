//! Differential Inverse Kinematics solver using Damped Least Squares (DLS).
//!
//! This module re-exports all items from [`crate::rbd::kinematics`].
//! The canonical implementation lives in the `rbd` module; this file
//! exists for backward-compatibility so that `crate::ik::*` still works.

pub use crate::rbd::kinematics::*;
