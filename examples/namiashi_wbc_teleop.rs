//! Interactive, real-time keyboard teleop of the WBC/MPC pipeline on the
//! same 5 cm / 10-step staircase every WBC/MPC measurement in
//! `tests/wbc_walk.rs` is built on -- the model-based counterpart to
//! `namiashi_rl_teleop.rs`, sharing the same key bindings so the same
//! muscle memory drives either controller.
//!
//! This is a proper binary, not a `#[test]`, because `mujoco-rs`'s
//! viewer creates a `winit` event loop, and `winit` requires that to
//! happen on the real process main thread -- `cargo test`'s harness
//! always runs each test body on a worker thread (by design, so tests
//! can run in parallel), which the viewer cannot tolerate regardless of
//! `--test-threads`. A `cargo run` binary's `fn main()` IS the process
//! main thread, so this works where a `#[test]`-based version could not.
//!
//! Calls `articara::wbc_harness::run_wbc_sim` directly -- the exact same,
//! already-validated WBC/MPC pipeline (misa-wbc QP, GaitController,
//! contact reflex/footplan machinery, all of it) that every other
//! measurement in `tests/wbc_walk.rs` uses, not a simplified stand-in.
//!
//! Run: `cargo run --release --no-default-features --features
//! "mujoco,mujoco-viewer" --example namiashi_wbc_teleop`
//!
//! Keys (identical to `sim2sim_namiashi_mujoco.py --interactive` and
//! `namiashi_rl_teleop.rs`):
//!   W/S or Up/Down          -- forward/back speed (vx)
//!   A/D or Left/Right       -- turn (wz)
//!   Q/E or PageUp/PageDown  -- strafe (vy)
//!   Space                   -- zero all three

#[cfg(all(feature = "mujoco", feature = "mujoco-viewer"))]
fn main() {
    use std::sync::{Arc, Mutex};

    use articara::mjcf::StaircaseCfg;
    use articara::wbc_harness::{run_wbc_sim, Actuation, WbcParams, NAMIASHI_CAPTURE_GAIN_S};
    use quadruped_gait::GaitType;

    let stairs = StaircaseCfg {
        rise_m: 0.05,
        run_m: 0.20,
        n_steps: 10,
        approach_m: 1.5,
        top_platform_m: 8.0,
        half_width_m: 6.0,
    };
    let live = Arc::new(Mutex::new([0.0_f64; 3]));

    // The known-good Trot preset (tests/wbc_walk.rs's NAMIASHI_TUNED[0]),
    // not the hip_bias_gate experiment -- that was shown non-robust
    // under trivial parameter perturbation
    // (namiashi_staircase_5cm_hip_gate_robustness) and has no place in a
    // hands-on demo.
    let params = WbcParams {
        actuation: Actuation::Torque { kp: 100.0, kd: 1.2 },
        host_rate_hz: Some(400.0),
        dt: 0.0005,
        gait_type: Some(GaitType::Trot),
        cycle_period_s: Some(0.320),
        duty_factor: Some(0.50),
        max_step_length_m: Some(0.145),
        swing_height_m: Some(0.040),
        k_capture_s: Some(NAMIASHI_CAPTURE_GAIN_S),
        cmd_vx: 0.800,
        total_time_s: 1800.0, // ends via the viewer window closing, not a timeout
        wbc_real_inertia: true,
        staircase: Some(stairs),
        live_cmd: Some(live),
        live_viewer: true,
        ..WbcParams::forward_walk()
    };
    run_wbc_sim(params);
}

#[cfg(not(all(feature = "mujoco", feature = "mujoco-viewer")))]
fn main() {
    eprintln!(
        "this example needs: cargo run --release --no-default-features \
         --features mujoco,mujoco-viewer --example namiashi_wbc_teleop"
    );
    std::process::exit(2);
}
