//! Floating-base state estimators.
//!
//! Provides estimators that fuse IMU + joint encoder + contact-sensor
//! measurements into a consistent body / foot world-frame state. The
//! WBC and gait controllers consume the estimated state instead of
//! relying on simulator ground truth.
//!
//! Currently the only implementation is [`LinearKalmanEstimator`], a
//! port of `legged_control`'s `KalmanFilterEstimate` (18-state KF over
//! `[body_pos; body_vel; foot_pos_world]`). Future estimators (e.g.
//! invariant EKF, factor-graph) plug into the same module.
//!
//! See [`doc/mpc_wbc_gait_control.md`](../../../doc/mpc_wbc_gait_control.md)
//! Phase B for design context.

pub mod linear_kalman;
// Pipeline depends on `wbc_pipeline::build_floating_base_model`,
// which is gated behind the `mujoco` feature; mirror the gate here so
// non-mujoco builds (e.g. WASM, CI without MuJoCo) still compile.
#[cfg(feature = "mujoco")]
pub mod pipeline;

pub use linear_kalman::{LinearKalmanEstimator, LinearKalmanInputs, LinearKalmanOutput};
#[cfg(feature = "mujoco")]
pub use pipeline::LkfPipeline;
