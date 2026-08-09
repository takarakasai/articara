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
//! Keys: see `articara::teleop`'s module docs -- W/S (or arrows) drive,
//! A/D turn, Q/E (or PgUp/PgDn) strafe, Shift for full speed instead of
//! half, 1/2/3 switch between Crawl / Walk / Trot, and R/F raise/lower
//! the swing foot in 5 mm steps. O/L change the ground's friction and
//! P/. change what the controller believes that friction is (they were
//! found to disagree: 0.5 assumed against 0.7 simulated). Holding a key
//! moves, releasing it stops.
//!
//! Each gait keeps its own tuned speed envelope (Crawl 0.17, Walk 0.33,
//! Trot 0.80 m/s), since each is bounded by its own
//! `max_step_length_m / (cycle_period_s * duty_factor)` -- commanding
//! Trot's speed in Crawl would just saturate. Switching gait is cleanest
//! from a standstill: the phase generator holds its cycle phase across a
//! swap, so a mid-stride switch snaps the legs to their new offsets.

#[cfg(all(feature = "mujoco", feature = "mujoco-viewer"))]
fn main() {
    use std::sync::{Arc, Mutex};

    use articara::mjcf::StaircaseCfg;
    use articara::teleop::LiveTeleop;
    use articara::wbc_harness::{namiashi_tuned_params, run_wbc_sim, Actuation, WbcParams};
    use quadruped_gait::GaitType;

    let stairs = StaircaseCfg {
        rise_m: 0.05,
        run_m: 0.20,
        n_steps: 10,
        approach_m: 1.5,
        top_platform_m: 8.0,
        half_width_m: 6.0,
    };
    // Start stopped, in Trot -- NAMIASHI_TUNED[0], the known-good preset.
    // Deliberately NOT the hip_bias_gate experiment: that was shown
    // non-robust under trivial parameter perturbation
    // (namiashi_staircase_5cm_hip_gate_robustness) and has no place in a
    // hands-on demo.
    let live = Arc::new(Mutex::new(LiveTeleop::new(GaitType::Trot)));

    let params = WbcParams {
        actuation: Actuation::Torque { kp: 100.0, kd: 1.2 },
        host_rate_hz: Some(400.0),
        dt: 0.0005,
        // Stand still until a key is pressed; live_teleop overrides this
        // from the first post-burn-in tick anyway.
        cmd_vx: 0.0,
        total_time_s: 1800.0, // ends via the viewer window closing, not a timeout
        wbc_real_inertia: true,
        staircase: Some(stairs),
        live_teleop: Some(live),
        live_viewer: true,
        ..namiashi_tuned_params(0)
    };
    eprintln!(
        "[teleop] W/S drive, A/D turn, Q/E strafe (arrows + PgUp/PgDn too), \
         Shift = full speed, 1/2/3 = Crawl/Walk/Trot, R/F = swing height, \
         O/L = ground mu, P/. = controller mu. Release to stop."
    );
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
