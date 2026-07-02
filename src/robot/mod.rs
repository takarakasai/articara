//! Robot model facade.
//!
//! The data types live in [`crate::rbd::model`] and are re-exported here
//! so `crate::robot::RobotModel` (and friends) works everywhere. The
//! `impl RobotModel` surface is split by concern:
//!
//! - [`io`] — URDF / `.misa` import & export, mesh materialisation
//! - [`edit`] — structural editing (add / remove / rename)
//! - [`pick`] — viewport ray picking

mod edit;
mod io;
mod pick;

pub use crate::rbd::model::*;
pub use io::*;
pub use pick::*;
