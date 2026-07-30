// Re-export modules for integration tests.
pub use legged_estimation::attitude_estimator;
pub mod camera;
pub mod chicken_head;
pub mod collision;
pub mod dynamics;
pub mod estimator;
pub mod format;
pub mod gait;
pub mod history;
pub mod isaac;
pub use legged_estimation::leg_odometry;
pub mod mesh_ops;
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
pub mod standing_gesture;
#[cfg(feature = "scripting")]
pub mod scripting;
#[cfg(feature = "scripting")]
pub mod scripting_model;
pub mod usd;
pub mod usd_import;
#[cfg(feature = "mujoco")]
pub mod wbc_pipeline;

/// Live gait viewer: subscribe to a `go2-gait-runner --viz` Zenoh stream and
/// drive the loaded model in real time. See [`viz_feed`].
#[cfg(feature = "viz")]
pub mod viz_feed;

// Note: app.rs and renderer.rs are not exported here because they
// depend on glow/egui context which is not available in tests.
