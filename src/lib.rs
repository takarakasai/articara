// Re-export modules for integration tests.
pub mod attitude_estimator;
pub mod camera;
pub mod collision;
pub mod dynamics;
pub mod format;
pub mod gait;
pub mod history;
pub mod isaac;
pub mod leg_odometry;
pub mod mesh_paths;
pub mod mjcf;
#[cfg(feature = "mujoco")]
pub mod mujoco_sim;
#[cfg(feature = "mujoco")]
pub mod mujoco_version;
pub mod primitives;
pub mod rbd;
pub mod robot;
pub mod sdf;
#[cfg(feature = "scripting")]
pub mod scripting;
#[cfg(feature = "scripting")]
pub mod scripting_model;
pub mod usd;
pub mod usd_import;

// Note: app.rs and renderer.rs are not exported here because they
// depend on glow/egui context which is not available in tests.
