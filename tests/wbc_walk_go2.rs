//! End-to-end MuJoCo regression for the Hierarchical WBC pipeline on
//! the **real Go2 model** (`models/unitree_go2/go2.misa`), not the
//! lightweight `namiashi` fixture [`wbc_walk`](./wbc_walk.rs) uses.
//!
//! Same harness and invariants as `wbc_walk.rs` (static gravity
//! balance, forward displacement under a velocity command) — this
//! file exists to answer a question `wbc_walk.rs` alone can't:
//! **does misa-wbc's WBC solve path actually walk the genuine Go2
//! model (real ~15.6 kg mass, real joint limits/actuator gains) in
//! MuJoCo, not just a small synthetic quadruped?** `go2.misa` already
//! carries real Kp=500/Kv=5 actuator gains and per-joint torque
//! limits (hip/thigh 23.7 N·m, calf 45.43 N·m) — no manual retuning
//! duplicated here; `GaitController::build` auto-detects the SRBD
//! MPC's `mass_kg` from the model too (`articara::gait::
//! auto_detect_srbd_mpc_config`), so this is the *same* code path as
//! `wbc_walk.rs`, just pointed at a different, real model.
//!
//! ## Known limitations (see `wbc_walk.rs` for the shared ones)
//!
//! - Home pose (hip=0, thigh=0.9, calf=-1.8) and initial base height
//!   (0.30 m) match the existing `examples/go2_crawl.rs` /
//!   `examples/go2_sim.rs` conventions for this model.
//! - Go2 is ~6x heavier than `namiashi` — thresholds below are Go2-
//!   specific, not copy-pasted from `wbc_walk.rs`.

#![cfg(feature = "mujoco")]

use std::path::PathBuf;

use articara::gait::{auto_detect_kinematics_config, GaitController, DEFAULT_FOOT_LINKS};
use articara::mjcf::{GroundPlaneCfg, MjcfExportOptions};
use articara::mujoco_sim::MujocoSim;
use articara::robot::RobotModel;
use articara::wbc_pipeline::WbcPipeline;
use nalgebra::Vector3;
use quadruped_gait::wbc;
use quadruped_gait::{
    solve_leg_ik, ContactDrivenPhase, GaitConfig, GaitMode, KinematicsConfig,
    LegIkSolution, VelocityCmd,
};
use quadruped_gait::FullCentroidalMpcConfig;

fn go2_misa() -> PathBuf {
    // Sibling to articara/tests/ -- the model lives at the repo root,
    // not under tests/fixtures/ (it's a submodule shared with
    // go2-gait-runner, not a test-only asset).
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("models")
        .join("unitree_go2")
        .join("go2.misa")
}

/// Same idea as `wbc_walk.rs`'s namiashi seeding: start each leg at
/// its nominal IK solution rather than q=0 (hip=0, thigh=0, calf=0 is
/// Go2's near-fully-extended kinematic singularity -- see this
/// session's `go2_leg_singularity_demo` D6 CBF work).
fn seed_joint_positions_from_kinematics(robot: &mut RobotModel, kin: &KinematicsConfig) {
    for leg_kin in [&kin.fl, &kin.fr, &kin.rl, &kin.rr] {
        let target = leg_kin.nominal_foot_body;
        let sol = solve_leg_ik(leg_kin, target, false);
        let LegIkSolution::Reached { hip, thigh, calf } = sol else {
            panic!("{:?}: nominal_foot_body unreachable", leg_kin.leg);
        };
        for (joint_name, q_ik, sign) in [
            (&leg_kin.hip_joint, hip, 1.0),
            (&leg_kin.thigh_joint, thigh, -1.0),
            (&leg_kin.calf_joint, calf, -1.0),
        ] {
            let Some(&ji) = robot.joint_map.get(joint_name.as_str()) else {
                continue;
            };
            robot.joint_positions[ji] = q_ik * sign;
        }
    }
}

#[derive(Debug)]
struct WbcSample {
    t: f64,
    body_x: f64,
    body_z: f64,
    roll: f64,
    pitch: f64,
    total_fz_world: f64,
}

/// Go2's real standing height is ~0.30 m; a collapse/faceplant drops
/// well below this. Looser than namiashi's 0.18 m floor only because
/// Go2 starts taller, not because the bar is lower in relative terms
/// (0.15 m ≈ 50% of nominal height, same ratio `wbc_walk.rs` uses).
const TRUNK_Z_FALL_THRESHOLD_M: f64 = 0.15;

/// Go2 is heavier and its default trot cadence/step length differ
/// from namiashi's tuning, so this threshold is deliberately loose —
/// the point of this test is "does it walk at all", not a precise
/// speed-tracking check.
const MIN_DISPLACEMENT_M: f64 = 0.03;

struct WbcParams {
    total_time_s: f64,
    burn_in_s: f64,
    cmd_vx: f64,
    dt: f64,
    misa_wbc_mode: Option<(wbc::Formulation, wbc::SolveConfig)>,
    /// When set, `cmd_vx` is ignored and the commanded forward
    /// velocity instead steps through `0.0, step, 2*step, … max`
    /// (`staircase_step_mps` / `staircase_max_mps`), holding each
    /// level for this many seconds — a stress test to find where
    /// Trot+WBC+MPC stops tracking cleanly, not a pass/fail
    /// regression check.
    staircase_step_s: Option<f64>,
    staircase_step_mps: f64,
    staircase_max_mps: f64,
    /// Override the auto-detected `SrbdMpcConfig`'s `(horizon_steps,
    /// dt_per_step)` after `GaitController::build` — an experiment
    /// prompted by `ref/legged_control`'s OCS2 NMPC using a 1.0s
    /// horizon (`task.info`'s `mpc.timeHorizon`) against our SRBD
    /// MPC's default 0.3s (10 steps x 30ms): does a longer horizon
    /// narrow the steady-state velocity-tracking gap Sec.5s found
    /// (no integral action anywhere in either codebase, so the two
    /// codebases' *horizon length* is the concrete, borrowable
    /// difference, not "integral gain").
    mpc_horizon_override: Option<(usize, f64)>,
    /// Override `GaitConfig::trot()`'s `cycle_period_s` (default 0.4s)
    /// after construction — used to test whether Sec.5t's narrow
    /// good-tracking horizon band (0.60-0.65s) is a resonance with the
    /// Trot gait cycle period rather than an absolute-time effect: if
    /// the band shifts proportionally when the cycle period changes,
    /// that confirms a ratio (horizon/cycle_period_s), not a fixed
    /// horizon in seconds.
    gait_cycle_period_override: Option<f64>,
    /// Override the per-leg standing-height bias fraction applied on
    /// top of the auto-detected `nominal_foot_body.z` (as
    /// `nominal_foot_body.z += bias_frac * (upper_leg_m +
    /// lower_leg_m)`, same formula `run_wbc_sim` always applies with
    /// the hardcoded default `0.08`). This is what the SRBD MPC
    /// regulates body z to (`mpc_controller.rs::build_srbd_inputs`
    /// uses `-kin.legs()[0].nominal_foot_body.z` as its target height
    /// proxy), so sweeping this sweeps standing height. `None` keeps
    /// the existing `0.08` default.
    body_height_bias_frac: Option<f64>,
    /// Use `GaitMode::FullCentroidal` (24-state, joint_q folded into
    /// the MPC state so the per-leg moment arm updates within the
    /// horizon) instead of the default `GaitMode::Mpc` (12-state SRBD,
    /// fixed foot position — Sec.5t/5u/5v's subject). `None` keeps the
    /// existing `GaitMode::Mpc` default.
    full_centroidal: Option<FullCentroidalOpts>,
}

/// `legged_control_parity`: per-leg phase contact schedule + swing
/// vertical-velocity tracking, matching OCS2's `centroidalModelType=0`
/// setup (see `full_centroidal_controller.rs` module docs).
/// `use_mpc_predicted_footstep`: replace capture-point feedback with a
/// foothold correction derived from the MPC's own predicted recovery
/// trajectory (closed loop between the MPC's optimized trunk response
/// and the footstep target) — the closest existing analogue to
/// jointly optimizing contact force / trunk / swing-leg trajectory.
/// `dynamic_joint_q_reference`: the MPC's joint_q tracking reference
/// becomes a real per-horizon-step trajectory (sampled from the same
/// open-loop swing/stance foot curve `tick()` uses) instead of a flat
/// hold — the D3.3.5a reversal, requires `legged_control_parity`.
/// `mpc_override`: override `(horizon_steps, dt_per_step, sqp_iterations)`
/// after `GaitController::build` — legged_control/OCS2's `ocs2_legged_robot`
/// example runs a real-time-iteration style `sqp_iterations=1` at a much
/// higher re-solve rate than our `sqp_iterations=3 @ dt_per_step=0.030`
/// default; this tests whether that "fewer iterations, more frequent
/// solves" tradeoff point is better for our (non-realtime-constrained,
/// wall-clock-agnostic) sim too, not just cheaper on real hardware.
#[derive(Clone, Copy)]
struct FullCentroidalOpts {
    legged_control_parity: bool,
    use_mpc_predicted_footstep: bool,
    dynamic_joint_q_reference: bool,
    mpc_override: Option<(usize, f64, usize)>,
}

impl WbcParams {
    fn static_stand() -> Self {
        Self {
            total_time_s: 1.5, burn_in_s: 0.5, cmd_vx: 0.0, dt: 0.002,
            misa_wbc_mode: None, staircase_step_s: None,
            staircase_step_mps: 0.0, staircase_max_mps: 0.0,
            mpc_horizon_override: None, gait_cycle_period_override: None,
            body_height_bias_frac: None, full_centroidal: None,
        }
    }
    fn forward_walk() -> Self {
        Self {
            total_time_s: 3.0, burn_in_s: 0.5, cmd_vx: 0.15, dt: 0.002,
            misa_wbc_mode: None, staircase_step_s: None,
            staircase_step_mps: 0.0, staircase_max_mps: 0.0,
            mpc_horizon_override: None, gait_cycle_period_override: None,
            body_height_bias_frac: None, full_centroidal: None,
        }
    }
    fn static_stand_misa_wbc(formulation: wbc::Formulation, cfg: wbc::SolveConfig) -> Self {
        Self { misa_wbc_mode: Some((formulation, cfg)), ..Self::static_stand() }
    }
    fn forward_walk_misa_wbc(formulation: wbc::Formulation, cfg: wbc::SolveConfig) -> Self {
        Self { misa_wbc_mode: Some((formulation, cfg)), ..Self::forward_walk() }
    }

    /// General staircase: `0.0` to `max_mps` in `step_mps` increments,
    /// evenly dividing `total_time_s` across all levels, routed
    /// through misa-wbc's ForceSpace+ActiveSet.
    fn velocity_staircase_custom_misa_wbc(
        formulation: wbc::Formulation,
        cfg: wbc::SolveConfig,
        step_mps: f64,
        max_mps: f64,
        total_time_s: f64,
    ) -> Self {
        let n_levels = (max_mps / step_mps).round() as usize + 1;
        Self {
            total_time_s,
            burn_in_s: 0.0, // level 0 (vx=0) already acts as the settle window
            cmd_vx: 0.0,    // unused; staircase_step_s drives the command instead
            dt: 0.002,
            misa_wbc_mode: Some((formulation, cfg)),
            staircase_step_s: Some(total_time_s / n_levels as f64),
            staircase_step_mps: step_mps,
            staircase_max_mps: max_mps,
            mpc_horizon_override: None,
            gait_cycle_period_override: None,
            body_height_bias_frac: None,
            full_centroidal: None,
        }
    }

    /// 30s staircase, 0 to 5 m/s in 0.5 m/s steps (11 levels, ~2.73s
    /// each) — the original coarse sweep that found the footstep-
    /// planner's speed ceiling (`ref/wbc_comparison.md` Sec.5r).
    fn velocity_staircase_misa_wbc(formulation: wbc::Formulation, cfg: wbc::SolveConfig) -> Self {
        Self::velocity_staircase_custom_misa_wbc(formulation, cfg, 0.5, 5.0, 30.0)
    }

    /// 60s staircase, 0 to 1.0 m/s in 0.05 m/s steps (21 levels,
    /// ~2.86s each) — fine-grained resweep around the ~0.46 m/s
    /// ceiling Sec.5r found, without the >1.5 m/s region that just
    /// produces the (already-explained) footstep-clamp saturation and
    /// capture-point-driven reversal.
    fn velocity_staircase_fine_misa_wbc(formulation: wbc::Formulation, cfg: wbc::SolveConfig) -> Self {
        Self::velocity_staircase_custom_misa_wbc(formulation, cfg, 0.05, 1.0, 60.0)
    }

    /// Same as [`Self::velocity_staircase_fine_misa_wbc`], with the
    /// SRBD MPC's horizon overridden to `(horizon_steps, dt_per_step)`
    /// after `GaitController::build` -- see `mpc_horizon_override`'s
    /// doc comment for why this is the experiment worth running.
    fn velocity_staircase_fine_with_horizon_misa_wbc(
        formulation: wbc::Formulation,
        cfg: wbc::SolveConfig,
        horizon_steps: usize,
        dt_per_step: f64,
    ) -> Self {
        Self {
            mpc_horizon_override: Some((horizon_steps, dt_per_step)),
            ..Self::velocity_staircase_fine_misa_wbc(formulation, cfg)
        }
    }

    /// Same as [`Self::velocity_staircase_fine_with_horizon_misa_wbc`],
    /// also overriding `GaitConfig::trot()`'s `cycle_period_s` — see
    /// `gait_cycle_period_override`'s doc comment.
    fn velocity_staircase_fine_with_horizon_and_cycle_misa_wbc(
        formulation: wbc::Formulation,
        cfg: wbc::SolveConfig,
        horizon_steps: usize,
        dt_per_step: f64,
        cycle_period_s: f64,
    ) -> Self {
        Self {
            gait_cycle_period_override: Some(cycle_period_s),
            ..Self::velocity_staircase_fine_with_horizon_misa_wbc(formulation, cfg, horizon_steps, dt_per_step)
        }
    }

    /// Same fine 0-1.0 m/s staircase, with the standing-height bias
    /// fraction overridden — see `body_height_bias_frac`'s doc comment.
    /// Horizon and cycle period stay at their defaults so height is the
    /// sole independent variable.
    fn velocity_staircase_fine_with_height_misa_wbc(
        formulation: wbc::Formulation,
        cfg: wbc::SolveConfig,
        body_height_bias_frac: f64,
    ) -> Self {
        Self {
            body_height_bias_frac: Some(body_height_bias_frac),
            ..Self::velocity_staircase_fine_misa_wbc(formulation, cfg)
        }
    }

    /// Combines the Sec.5t horizon override and the Sec.5u height
    /// override in one run — tests whether their independently-found
    /// reversal-free-plateau effects stack, cancel, or are redundant
    /// (the same underlying mechanism seen twice).
    fn velocity_staircase_fine_with_horizon_and_height_misa_wbc(
        formulation: wbc::Formulation,
        cfg: wbc::SolveConfig,
        horizon_steps: usize,
        dt_per_step: f64,
        body_height_bias_frac: f64,
    ) -> Self {
        Self {
            body_height_bias_frac: Some(body_height_bias_frac),
            ..Self::velocity_staircase_fine_with_horizon_misa_wbc(formulation, cfg, horizon_steps, dt_per_step)
        }
    }

    /// Same fine 0-1.0 m/s staircase, on `GaitMode::FullCentroidal`
    /// instead of the default `GaitMode::Mpc` — see `full_centroidal`'s
    /// doc comment. `horizon_steps`/`dt_per_step`/`body_height_bias_frac`
    /// stay at their `GaitMode::Mpc` defaults (irrelevant here; the
    /// FullCentroidal controller has its own separately auto-detected
    /// `FullCentroidalMpcConfig`).
    fn velocity_staircase_fine_full_centroidal_misa_wbc(
        formulation: wbc::Formulation,
        cfg: wbc::SolveConfig,
        legged_control_parity: bool,
        use_mpc_predicted_footstep: bool,
    ) -> Self {
        Self::velocity_staircase_fine_full_centroidal_dynamic_q_misa_wbc(
            formulation, cfg, legged_control_parity, use_mpc_predicted_footstep, false,
        )
    }

    /// Same as [`Self::velocity_staircase_fine_full_centroidal_misa_wbc`],
    /// also toggling `dynamic_joint_q_reference` — see
    /// `FullCentroidalOpts`'s doc comment.
    fn velocity_staircase_fine_full_centroidal_dynamic_q_misa_wbc(
        formulation: wbc::Formulation,
        cfg: wbc::SolveConfig,
        legged_control_parity: bool,
        use_mpc_predicted_footstep: bool,
        dynamic_joint_q_reference: bool,
    ) -> Self {
        Self {
            full_centroidal: Some(FullCentroidalOpts {
                legged_control_parity, use_mpc_predicted_footstep, dynamic_joint_q_reference,
                mpc_override: None,
            }),
            ..Self::velocity_staircase_fine_misa_wbc(formulation, cfg)
        }
    }

    /// Same as [`Self::velocity_staircase_fine_full_centroidal_dynamic_q_misa_wbc`],
    /// also overriding the FullCentroidal MPC's `(horizon_steps,
    /// dt_per_step, sqp_iterations)` — see `mpc_override`'s doc comment.
    fn velocity_staircase_fine_full_centroidal_mpc_override_misa_wbc(
        formulation: wbc::Formulation,
        cfg: wbc::SolveConfig,
        legged_control_parity: bool,
        dynamic_joint_q_reference: bool,
        horizon_steps: usize,
        dt_per_step: f64,
        sqp_iterations: usize,
    ) -> Self {
        let mut params = Self::velocity_staircase_fine_full_centroidal_dynamic_q_misa_wbc(
            formulation, cfg, legged_control_parity, false, dynamic_joint_q_reference,
        );
        let opts = params.full_centroidal.as_mut().expect("full_centroidal always Some here");
        opts.mpc_override = Some((horizon_steps, dt_per_step, sqp_iterations));
        params
    }
}

fn run_wbc_sim(params: WbcParams) -> Option<Vec<WbcSample>> {
    let path = go2_misa();
    if !path.exists() {
        eprintln!("go2.misa missing at {} — skipping Go2 WBC test", path.display());
        return None;
    }
    let mut robot = RobotModel::from_misa(&path).expect("load go2.misa");

    let mut kin = auto_detect_kinematics_config(&robot, &DEFAULT_FOOT_LINKS)
        .expect("auto-detect kinematics");
    let height_bias_frac = params.body_height_bias_frac.unwrap_or(0.08);
    let raw_z = kin.fl.nominal_foot_body.z;
    let total_leg = kin.fl.upper_leg_m + kin.fl.lower_leg_m;
    for leg_kin in [&mut kin.fl, &mut kin.fr, &mut kin.rl, &mut kin.rr] {
        let total_leg = leg_kin.upper_leg_m + leg_kin.lower_leg_m;
        leg_kin.nominal_foot_body.z += height_bias_frac * total_leg;
    }
    if params.body_height_bias_frac.is_some() {
        eprintln!(
            "[body-height] bias_frac={height_bias_frac:.3} (leg_len={total_leg:.3}m, raw_z={raw_z:.3}m) \
             -> nominal_foot_body.z={:.3}m (standing height ~{:.3}m)",
            kin.fl.nominal_foot_body.z, -kin.fl.nominal_foot_body.z,
        );
    }
    seed_joint_positions_from_kinematics(&mut robot, &kin);

    let opts = MjcfExportOptions {
        base_pos: Some([0.0, 0.0, 0.30]),
        ground_plane: Some(GroundPlaneCfg { z: 0.0, half_size: 4.0, roll: 0.0, pitch: 0.0 }),
        add_actuators: true,
        ..Default::default()
    };
    let mut sim = MujocoSim::new(&robot, opts).expect("MujocoSim::new");
    sim.set_gravity_compensation(true);

    let mut cfg = GaitConfig::trot();
    if let Some(cycle_period_s) = params.gait_cycle_period_override {
        eprintln!(
            "[gait-cycle] overriding cycle_period_s {:.3}s -> {:.3}s",
            cfg.cycle_period_s, cycle_period_s
        );
        cfg.cycle_period_s = cycle_period_s;
    }
    let gait_mode = if params.full_centroidal.is_some() { GaitMode::FullCentroidal } else { GaitMode::Mpc };
    let mut gc = GaitController::build(&robot, kin.clone(), cfg, gait_mode)
        .expect("GaitController::build");
    if let Some((horizon_steps, dt_per_step)) = params.mpc_horizon_override {
        let mut mpc_cfg = gc.srbd_mpc_config().expect("Mpc mode has a config").clone();
        eprintln!(
            "[mpc-horizon] overriding {}x{:.3}s={:.2}s -> {}x{:.3}s={:.2}s",
            mpc_cfg.horizon_steps, mpc_cfg.dt_per_step,
            mpc_cfg.horizon_steps as f64 * mpc_cfg.dt_per_step,
            horizon_steps, dt_per_step, horizon_steps as f64 * dt_per_step,
        );
        mpc_cfg.horizon_steps = horizon_steps;
        mpc_cfg.dt_per_step = dt_per_step;
        gc.set_srbd_mpc_config(mpc_cfg);
    }
    if let Some(opts) = params.full_centroidal {
        eprintln!(
            "[full-centroidal] legged_control_parity={} use_mpc_predicted_footstep={} dynamic_joint_q_reference={}",
            opts.legged_control_parity, opts.use_mpc_predicted_footstep, opts.dynamic_joint_q_reference,
        );
        gc.set_legged_control_parity(opts.legged_control_parity);
        gc.set_use_mpc_predicted_footstep(opts.use_mpc_predicted_footstep);
        gc.set_dynamic_joint_q_reference(opts.dynamic_joint_q_reference);
        if let Some((horizon_steps, dt_per_step, sqp_iterations)) = opts.mpc_override {
            let mut mpc_cfg: FullCentroidalMpcConfig =
                gc.full_centroidal_mpc_config().expect("FullCentroidal mode has a config").clone();
            eprintln!(
                "[full-centroidal-mpc] overriding {}x{:.3}s sqp={} -> {}x{:.3}s sqp={}",
                mpc_cfg.horizon_steps, mpc_cfg.dt_per_step, mpc_cfg.sqp_iterations,
                horizon_steps, dt_per_step, sqp_iterations,
            );
            mpc_cfg.horizon_steps = horizon_steps;
            mpc_cfg.dt_per_step = dt_per_step;
            mpc_cfg.sqp_iterations = sqp_iterations;
            gc.set_full_centroidal_mpc_config(mpc_cfg);
        }
    }

    let foot_links: [String; 4] = [
        DEFAULT_FOOT_LINKS[0].1.to_string(),
        DEFAULT_FOOT_LINKS[1].1.to_string(),
        DEFAULT_FOOT_LINKS[2].1.to_string(),
        DEFAULT_FOOT_LINKS[3].1.to_string(),
    ];
    let mut wbc_pipeline = WbcPipeline::new(&robot, foot_links);
    if let Some((formulation, cfg)) = params.misa_wbc_mode.clone() {
        wbc_pipeline = wbc_pipeline.with_wbc_solver(formulation, cfg);
    }

    // Optional per-tick link-pose CSV export for external video
    // rendering (visualization side-channel only, no effect on the
    // pass/fail assertions below). Every body MuJoCo actually
    // simulates -- base + all 4 legs' hip/thigh/calf/foot -- queried
    // by name directly from MuJoCo (sidesteps needing to know
    // misarta's floating-base joint indexing at all).
    const LINK_NAMES: [&str; 17] = [
        "base",
        "FL_hip", "FL_thigh", "FL_calf", "FL_foot",
        "FR_hip", "FR_thigh", "FR_calf", "FR_foot",
        "RL_hip", "RL_thigh", "RL_calf", "RL_foot",
        "RR_hip", "RR_thigh", "RR_calf", "RR_foot",
    ];
    let csv_out = std::env::var("WBC_WALK_CSV_OUT").ok();
    let mut csv_buf = String::new();
    if csv_out.is_some() {
        csv_buf.push_str("tick,t");
        for name in LINK_NAMES {
            csv_buf.push_str(&format!(
                ",{name}_tx,{name}_ty,{name}_tz,\
                 {name}_r00,{name}_r01,{name}_r02,\
                 {name}_r10,{name}_r11,{name}_r12,\
                 {name}_r20,{name}_r21,{name}_r22"
            ));
        }
        csv_buf.push('\n');
    }

    let n_steps = (params.total_time_s / params.dt).round() as usize;
    let burn_in_steps = (params.burn_in_s / params.dt).round() as usize;
    let mut samples: Vec<WbcSample> = Vec::with_capacity(n_steps);

    let mut last_staircase_level: Option<usize> = None;
    for k in 0..n_steps {
        let t = k as f64 * params.dt;

        if k == 0 {
            gc.enable();
        }
        if let Some(step_s) = params.staircase_step_s {
            let n_levels =
                (params.staircase_max_mps / params.staircase_step_mps).round() as usize + 1;
            let level = ((t / step_s) as usize).min(n_levels - 1);
            if last_staircase_level != Some(level) {
                let vx = level as f64 * params.staircase_step_mps;
                gc.set_velocity_cmd(VelocityCmd { vx, vy: 0.0, wz: 0.0 });
                eprintln!("[staircase] t={t:6.2}s level={level:2} cmd_vx={vx:.2} m/s");
                last_staircase_level = Some(level);
            }
        } else if k == burn_in_steps {
            gc.set_velocity_cmd(VelocityCmd { vx: params.cmd_vx, vy: 0.0, wz: 0.0 });
        }

        let v_obs = sim.body_world_linear_velocity(&robot.root_link).unwrap_or([0.0, 0.0, 0.0]);
        let w_obs = sim.body_world_angular_velocity(&robot.root_link).unwrap_or([0.0, 0.0, 0.0]);
        gc.set_body_state_observed(
            Vector3::new(v_obs[0], v_obs[1], v_obs[2]),
            Vector3::new(w_obs[0], w_obs[1], w_obs[2]),
        );

        if gc.is_enabled() {
            let (out, targets, torque_ff) = gc.tick(params.dt);
            for (idx, q) in targets {
                sim.set_position_target(idx, q);
            }
            if k >= burn_in_steps {
                let f_grf_world =
                    gc.predicted_grfs().map(|sol| sol.grfs_first_step).unwrap_or([Vector3::zeros(); 4]);
                let cmd = gc.velocity_cmd();
                let v_cmd_body = Vector3::new(cmd.vx, cmd.vy, 0.0);
                let foot_links_str: [&str; 4] = [
                    wbc_pipeline.foot_links[0].as_str(),
                    wbc_pipeline.foot_links[1].as_str(),
                    wbc_pipeline.foot_links[2].as_str(),
                    wbc_pipeline.foot_links[3].as_str(),
                ];
                let force_z = sim.contact_force_per_foot(&foot_links_str);
                let nominal_phases =
                    [out.legs[0].phase, out.legs[1].phase, out.legs[2].phase, out.legs[3].phase];
                let corrected = ContactDrivenPhase::apply_correction(
                    &nominal_phases,
                    force_z,
                    5.0,
                    0.0,
                );
                let contact_flag = [
                    corrected[0].is_stance,
                    corrected[1].is_stance,
                    corrected[2].is_stance,
                    corrected[3].is_stance,
                ];
                let taus = wbc_pipeline.solve(
                    &robot,
                    &sim,
                    &out,
                    gc.kinematics(),
                    gc.joint_indices(),
                    gc.joint_signs(),
                    &v_cmd_body,
                    cmd.wz,
                    &Vector3::new(v_obs[0], v_obs[1], v_obs[2]),
                    &Vector3::new(w_obs[0], w_obs[1], w_obs[2]),
                    &f_grf_world,
                    contact_flag,
                    params.dt,
                );
                if k % 250 == 0 {
                    let body_pos = sim.body_world_position(&robot.root_link).unwrap_or([0.0, 0.0, 0.0]);
                    let tau_max = taus.iter().cloned().fold(0.0_f64, |a, b| a.max(b.abs()));
                    let mpc_fz_sum: f64 = f_grf_world.iter().map(|v| v.z).sum();
                    let stance_count = contact_flag.iter().filter(|b| **b).count();
                    eprintln!(
                        "[diag k={k:5} t={:.3}s] z={:.3} m  Σmpc_f_z={:.2} N  max|τ|={:.2} N·m  stance={}/4",
                        k as f64 * params.dt, body_pos[2], mpc_fz_sum, tau_max, stance_count,
                    );
                }
                let _ = torque_ff;
                for (ji, &tau) in taus.iter().enumerate() {
                    sim.set_torque_feedforward(ji, tau);
                }
                sim.clear_wbc_torques();
            } else {
                sim.clear_wbc_torques();
                for ji in 0..robot.joints.len() {
                    sim.set_torque_feedforward(ji, 0.0);
                }
            }
        }

        sim.step(&mut robot, params.dt, true);

        let tx = robot.base_transform.translation;
        let (roll, pitch, _yaw) = robot.base_transform.rotation.euler_angles();
        let total_fz_world: f64 = sim.contacts().iter().map(|c| c.force_world[2]).sum();
        samples.push(WbcSample { t, body_x: tx.x, body_z: tx.z, roll, pitch, total_fz_world });

        if csv_out.is_some() {
            csv_buf.push_str(&format!("{k},{t:.4}"));
            for name in LINK_NAMES {
                let p = sim.body_world_position(name).unwrap_or([0.0, 0.0, 0.0]);
                let r = sim
                    .body_world_orientation(name)
                    .map(|q| *q.to_rotation_matrix().matrix())
                    .unwrap_or_else(nalgebra::Matrix3::identity);
                csv_buf.push_str(&format!(",{:.5},{:.5},{:.5}", p[0], p[1], p[2]));
                for row in 0..3 {
                    for col in 0..3 {
                        csv_buf.push_str(&format!(",{:.6}", r[(row, col)]));
                    }
                }
            }
            csv_buf.push('\n');
        }
    }
    if let Some(path) = csv_out {
        std::fs::write(&path, csv_buf).expect("write WBC_WALK_CSV_OUT");
        eprintln!("wrote {path}");
    }
    Some(samples)
}

fn robot_mass(robot: &RobotModel) -> f64 {
    robot.links.iter().map(|l| l.inertial.mass).sum()
}

#[test]
fn go2_wbc_static_stand_balances_gravity() {
    let Some(samples) = run_wbc_sim(WbcParams::static_stand()) else { return };
    assert_static_stand_balances_gravity(&samples);
}

#[test]
fn go2_wbc_static_stand_balances_gravity_force_space_active_set() {
    let cfg = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
    let Some(samples) =
        run_wbc_sim(WbcParams::static_stand_misa_wbc(wbc::Formulation::ForceSpace, cfg))
    else {
        return;
    };
    assert_static_stand_balances_gravity(&samples);
}

fn assert_static_stand_balances_gravity(samples: &[WbcSample]) {
    let min_z = samples.iter().map(|s| s.body_z).fold(f64::INFINITY, f64::min);
    assert!(
        min_z > TRUNK_Z_FALL_THRESHOLD_M,
        "static stand: trunk fell, min_z = {min_z:.3} m (threshold {:.2})",
        TRUNK_Z_FALL_THRESHOLD_M,
    );

    let dt: f64 = 0.002;
    let total_time = 1.5;
    let total_n = (total_time / dt).round() as usize;
    let window_n = (0.5 / dt).round() as usize;
    let start = total_n.saturating_sub(window_n);
    let avg_fz: f64 =
        samples[start..].iter().map(|s| s.total_fz_world).sum::<f64>() / (samples.len() - start) as f64;

    let path = go2_misa();
    let robot = RobotModel::from_misa(&path).unwrap();
    let mg = robot_mass(&robot) * 9.81;

    let pct_err = ((avg_fz - mg) / mg).abs();
    eprintln!("[wbc:go2] static_stand: avg Σf_z = {avg_fz:.2} N, m·g = {mg:.2} N (err = {:.1}%)", pct_err * 100.0);
    assert!(
        pct_err < 0.60,
        "static stand: Σf_z = {avg_fz:.2} N deviates from m·g = {mg:.2} N by {:.1}%",
        pct_err * 100.0,
    );
}

#[test]
fn go2_wbc_forward_command_advances_body() {
    let Some(samples) = run_wbc_sim(WbcParams::forward_walk()) else { return };
    assert_forward_command_advances_body(&samples);
}

#[test]
fn go2_wbc_forward_command_advances_body_force_space_active_set() {
    let cfg = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
    let Some(samples) =
        run_wbc_sim(WbcParams::forward_walk_misa_wbc(wbc::Formulation::ForceSpace, cfg))
    else {
        return;
    };
    assert_forward_command_advances_body(&samples);
}

/// Stress test, not a regression check: command a `0 -> max_mps`
/// staircase (`step_mps` per level) over `total_time_s` and report
/// per-level tracking quality -- where does Trot+WBC+SRBD-MPC stop
/// keeping up? No hard pass/fail assertion (a fall or a tracking
/// plateau is an expected, informative outcome, not a bug).
fn report_velocity_staircase(samples: &[WbcSample], step_mps: f64, max_mps: f64, total_time_s: f64) {
    let n_levels = (max_mps / step_mps).round() as usize + 1;
    let step_s = total_time_s / n_levels as f64;

    eprintln!(
        "\n{:>6} {:>8} {:>9} {:>9} {:>9} {:>9}",
        "level", "cmd_vx", "meas_vx", "min_z", "peak_roll", "peak_pitch"
    );
    for level in 0..n_levels {
        let t0 = level as f64 * step_s;
        let t1 = (level as f64 + 1.0) * step_s;
        let window: Vec<&WbcSample> =
            samples.iter().filter(|s| s.t >= t0 && s.t < t1).collect();
        if window.is_empty() {
            continue;
        }
        let cmd_vx = level as f64 * step_mps;
        let x0 = window.first().unwrap().body_x;
        let x1 = window.last().unwrap().body_x;
        let meas_vx = (x1 - x0) / (t1 - t0).min(window.last().unwrap().t - t0);
        let min_z = window.iter().map(|s| s.body_z).fold(f64::INFINITY, f64::min);
        let peak_roll = window.iter().map(|s| s.roll.abs()).fold(0.0_f64, f64::max);
        let peak_pitch = window.iter().map(|s| s.pitch.abs()).fold(0.0_f64, f64::max);
        eprintln!(
            "{level:6} {cmd_vx:8.2} {meas_vx:9.3} {min_z:9.3} {peak_roll:9.2} {peak_pitch:9.2}",
        );
    }

    let min_z_overall = samples.iter().map(|s| s.body_z).fold(f64::INFINITY, f64::min);
    eprintln!(
        "\n[wbc:go2] velocity_staircase: min_z over full run = {min_z_overall:.3} m \
         (fall threshold {TRUNK_Z_FALL_THRESHOLD_M:.2} m)"
    );
}

/// Coarse sweep, 0 to 5 m/s in 0.5 m/s steps over 30s -- found the
/// footstep-planner speed ceiling (`ref/wbc_comparison.md` Sec.5r).
/// `#[ignore]`d like the other exploratory benchmarks in this
/// session; run manually with `WBC_WALK_CSV_OUT=<path> cargo test
/// --release --features mujoco --test wbc_walk_go2 -- --ignored
/// --nocapture go2_wbc_velocity_staircase`.
#[test]
#[ignore = "exploratory stress test — run with --ignored"]
fn go2_wbc_velocity_staircase() {
    let cfg = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
    let Some(samples) =
        run_wbc_sim(WbcParams::velocity_staircase_misa_wbc(wbc::Formulation::ForceSpace, cfg))
    else {
        return;
    };
    report_velocity_staircase(&samples, 0.5, 5.0, 30.0);
}

/// Fine resweep, 0 to 1.0 m/s in 0.05 m/s steps over 60s -- higher
/// resolution around the ~0.46 m/s ceiling Sec.5r found, without the
/// >1.5 m/s region whose saturation/reversal is already explained by
/// the footstep planner's `max_step_length_m` clamp
/// (`mpc_controller.rs::compute_mpc_footstep`).
#[test]
#[ignore = "exploratory stress test — run with --ignored"]
fn go2_wbc_velocity_staircase_fine() {
    let cfg = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
    let Some(samples) = run_wbc_sim(WbcParams::velocity_staircase_fine_misa_wbc(
        wbc::Formulation::ForceSpace,
        cfg,
    )) else {
        return;
    };
    report_velocity_staircase(&samples, 0.05, 1.0, 60.0);
}

/// Same fine sweep, MPC horizon doubled (10 steps x 60ms = 0.6s,
/// vs the 0.3s default) -- same QP size (n=120, no extra per-solve
/// cost), testing whether a longer horizon narrows the steady-state
/// tracking gap Sec.5s found, the way `ref/legged_control`'s OCS2
/// NMPC (1.0s horizon) might benefit from versus our 0.3s default.
#[test]
#[ignore = "exploratory stress test — run with --ignored"]
fn go2_wbc_velocity_staircase_fine_long_horizon() {
    let cfg = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
    let Some(samples) = run_wbc_sim(WbcParams::velocity_staircase_fine_with_horizon_misa_wbc(
        wbc::Formulation::ForceSpace,
        cfg,
        10,
        0.06,
    )) else {
        return;
    };
    report_velocity_staircase(&samples, 0.05, 1.0, 60.0);
}

/// Same fine sweep again, MPC horizon stretched to match
/// `ref/legged_control`'s OCS2 NMPC `mpc.timeHorizon 1.0` exactly
/// (10 steps x 100ms = 1.0s, still n=120 -- only the per-step
/// discretization gets coarser, not the QP size). Sec.5t found the
/// 0.6s trial fixed the high-speed reversal and pushed mid-range
/// tracking to ~100%, at the cost of a small low-speed regression;
/// this checks whether 1.0s keeps improving or starts to degrade from
/// discretization error.
#[test]
#[ignore = "exploratory stress test — run with --ignored"]
fn go2_wbc_velocity_staircase_fine_full_horizon() {
    let cfg = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
    let Some(samples) = run_wbc_sim(WbcParams::velocity_staircase_fine_with_horizon_misa_wbc(
        wbc::Formulation::ForceSpace,
        cfg,
        10,
        0.10,
    )) else {
        return;
    };
    report_velocity_staircase(&samples, 0.05, 1.0, 60.0);
}

/// Sec.5t found a non-monotonic dependence on MPC horizon: 0.3s
/// (default) saturates-then-reverses gently, 0.6s tracks near-100% up
/// to ~0.5 m/s with no reversal, but 1.0s (legged_control-matched)
/// diverges into sustained backward walking at high commanded speed.
/// Sweeps the gap between the known-good 0.6s and known-bad 1.0s to
/// locate where the collapse actually begins.
#[test]
#[ignore = "exploratory stress test — run with --ignored"]
fn go2_wbc_velocity_staircase_fine_horizon_sweep() {
    for dt_per_step in [0.07, 0.08, 0.09] {
        let cfg = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
        let Some(samples) = run_wbc_sim(WbcParams::velocity_staircase_fine_with_horizon_misa_wbc(
            wbc::Formulation::ForceSpace,
            cfg,
            10,
            dt_per_step,
        )) else {
            continue;
        };
        eprintln!("\n=== horizon = 10 x {dt_per_step:.2}s = {:.2}s ===", dt_per_step * 10.0);
        report_velocity_staircase(&samples, 0.05, 1.0, 60.0);
    }
}

/// Sec.5t's coarse sweep found 0.6s tracking near-ideal but 0.7s
/// already reversed -- a cliff, not a gentle rolloff, suggesting 0.6s
/// might be an isolated spike rather than a stable operating point.
/// Zooms into 0.55/0.58/0.62/0.65s to find the spike's actual width.
#[test]
#[ignore = "exploratory stress test — run with --ignored"]
fn go2_wbc_velocity_staircase_fine_horizon_zoom() {
    for dt_per_step in [0.055, 0.058, 0.062, 0.065] {
        let cfg = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
        let Some(samples) = run_wbc_sim(WbcParams::velocity_staircase_fine_with_horizon_misa_wbc(
            wbc::Formulation::ForceSpace,
            cfg,
            10,
            dt_per_step,
        )) else {
            continue;
        };
        eprintln!("\n=== horizon = 10 x {dt_per_step:.3}s = {:.2}s ===", dt_per_step * 10.0);
        report_velocity_staircase(&samples, 0.05, 1.0, 60.0);
    }
}

/// Sec.5t found good tracking only in a narrow horizon band
/// (0.60-0.65s) around the Trot default `cycle_period_s=0.4s` — i.e.
/// 1.5-1.625 gait cycles. Tests whether that's a resonance with the
/// gait cycle (band should shift proportionally when `cycle_period_s`
/// changes) or a fixed absolute-time effect (band should stay put).
/// For each `cycle_period_s`, tries both the ORIGINAL absolute horizon
/// (0.6s) and the PROPORTIONAL horizon (1.5x the new cycle period) —
/// resonance hypothesis predicts proportional wins, absolute-time
/// hypothesis predicts the original 0.6s keeps winning regardless.
#[test]
#[ignore = "exploratory stress test — run with --ignored"]
fn go2_wbc_velocity_staircase_fine_cycle_resonance() {
    let trials: [(f64, &str, f64); 4] = [
        (0.3, "0.3s cycle, absolute 0.6s horizon", 0.060),
        (0.3, "0.3s cycle, proportional 1.5x = 0.45s horizon", 0.045),
        (0.5, "0.5s cycle, absolute 0.6s horizon", 0.060),
        (0.5, "0.5s cycle, proportional 1.5x = 0.75s horizon", 0.075),
    ];
    for (cycle_period_s, label, dt_per_step) in trials {
        let cfg = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
        let Some(samples) = run_wbc_sim(WbcParams::velocity_staircase_fine_with_horizon_and_cycle_misa_wbc(
            wbc::Formulation::ForceSpace,
            cfg,
            10,
            dt_per_step,
            cycle_period_s,
        )) else {
            continue;
        };
        eprintln!("\n=== {label} ===");
        report_velocity_staircase(&samples, 0.05, 1.0, 60.0);
    }
}

/// Sweeps standing height (via `body_height_bias_frac`, applied on top
/// of the auto-detected `nominal_foot_body.z` — see that field's doc
/// comment) across the same fine 0-1.0 m/s staircase, holding MPC
/// horizon and gait cycle at their defaults so height is the sole
/// independent variable: does a taller/crouchier stance change
/// velocity-tracking quality or stability, and where does it fail
/// (leg near-full-extension singularity at the tall end, insufficient
/// swing clearance at the crouched end)?
#[test]
#[ignore = "exploratory stress test — run with --ignored"]
fn go2_wbc_velocity_staircase_fine_body_height_sweep() {
    for bias_frac in [-0.08, -0.04, 0.0, 0.08, 0.16, 0.24, 0.32] {
        let cfg = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
        let Some(samples) = run_wbc_sim(WbcParams::velocity_staircase_fine_with_height_misa_wbc(
            wbc::Formulation::ForceSpace,
            cfg,
            bias_frac,
        )) else {
            continue;
        };
        eprintln!("\n=== body_height_bias_frac = {bias_frac:.2} ===");
        report_velocity_staircase(&samples, 0.05, 1.0, 60.0);
    }
}

/// Sec.5t (0.6s horizon) and Sec.5u (h=0.20m, bias_frac=0.16) each
/// independently produced a reversal-free plateau from the default
/// (0.3s horizon, h=0.23m) baseline's peak-then-rolloff shape. Tests
/// whether combining both pushes further (stacking), makes no further
/// difference (same underlying mechanism, already saturated), or
/// interacts badly (cancels/destabilizes) — run alongside the two
/// solo configurations for direct three-way comparison in one table.
#[test]
#[ignore = "exploratory stress test — run with --ignored"]
fn go2_wbc_velocity_staircase_fine_horizon_and_height_combo() {
    let solver_cfg = || wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
    let trials: [(&str, WbcParams); 3] = [
        (
            "height only (h=0.20m)",
            WbcParams::velocity_staircase_fine_with_height_misa_wbc(
                wbc::Formulation::ForceSpace, solver_cfg(), 0.16,
            ),
        ),
        (
            "horizon only (0.6s)",
            WbcParams::velocity_staircase_fine_with_horizon_misa_wbc(
                wbc::Formulation::ForceSpace, solver_cfg(), 10, 0.06,
            ),
        ),
        (
            "combined: height 0.20m + horizon 0.6s",
            WbcParams::velocity_staircase_fine_with_horizon_and_height_misa_wbc(
                wbc::Formulation::ForceSpace, solver_cfg(), 10, 0.06, 0.16,
            ),
        ),
    ];
    for (label, params) in trials {
        let Some(samples) = run_wbc_sim(params) else { continue };
        eprintln!("\n=== {label} ===");
        report_velocity_staircase(&samples, 0.05, 1.0, 60.0);
    }
}

/// First look at `GaitMode::FullCentroidal` (joint_q folded into the
/// MPC state, so the per-leg moment arm updates within the horizon —
/// architecturally the closest existing thing to legged_control's
/// jointly-optimized contact-force/trunk/swing-leg formulation) against
/// the `GaitMode::Mpc` (SRBD) baseline Sec.5s-5v all used, on the same
/// fine 0-1.0 m/s staircase, at FullCentroidal's own auto-detected
/// defaults (no height/horizon tuning — that's a separate follow-up
/// once we know whether this architecture is worth tuning at all).
/// Three FullCentroidal configurations, each opt-in on top of the last:
/// legacy (D3.3.5a), `legged_control_parity` (OCS2-matched contact
/// schedule + swing normal-velocity tracking), and adding
/// `use_mpc_predicted_footstep` (closes the loop: footstep target
/// comes from the MPC's own predicted recovery trajectory instead of
/// capture-point feedback).
#[test]
#[ignore = "exploratory stress test — run with --ignored"]
fn go2_wbc_velocity_staircase_fine_full_centroidal() {
    let trials = [
        ("FullCentroidal legacy (parity off)", false, false),
        ("FullCentroidal + legged_control_parity", true, false),
        ("FullCentroidal + parity + mpc_predicted_footstep", true, true),
    ];
    for (label, parity, predicted_footstep) in trials {
        let cfg = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
        let params = WbcParams::velocity_staircase_fine_full_centroidal_misa_wbc(
            wbc::Formulation::ForceSpace,
            cfg,
            parity,
            predicted_footstep,
        );
        let Some(samples) = run_wbc_sim(params) else { continue };
        eprintln!("\n=== {label} ===");
        report_velocity_staircase(&samples, 0.05, 1.0, 60.0);
    }
}

/// Sec.5w found `legged_control_parity` alone matched the hand-found
/// SRBD 0.6s-horizon plateau with zero tuning. This is the next
/// increment the user asked for: does actually closing the joint-space
/// loop -- `dynamic_joint_q_reference`, the D3.3.5a reversal where the
/// MPC's joint_q cost tracks a real per-horizon-step swing/stance
/// trajectory instead of a flat hold, the most literal reading of
/// "jointly optimize contact force / trunk / swing-leg trajectory" --
/// improve on `legged_control_parity` alone, or (like Sec.5v's
/// height+horizon combo, and Sec.5w's own use_mpc_predicted_footstep)
/// interact badly instead.
#[test]
#[ignore = "exploratory stress test — run with --ignored"]
fn go2_wbc_velocity_staircase_fine_full_centroidal_dynamic_q() {
    let trials = [
        ("FullCentroidal + parity (Sec.5w baseline)", false),
        ("FullCentroidal + parity + dynamic_joint_q_reference", true),
    ];
    for (label, dynamic_q) in trials {
        let cfg = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
        let params = WbcParams::velocity_staircase_fine_full_centroidal_dynamic_q_misa_wbc(
            wbc::Formulation::ForceSpace,
            cfg,
            true,
            false,
            dynamic_q,
        );
        let Some(samples) = run_wbc_sim(params) else { continue };
        eprintln!("\n=== {label} ===");
        report_velocity_staircase(&samples, 0.05, 1.0, 60.0);
    }
}

/// Desk-research (`ref/ocs2` verified) found legged_control/OCS2's
/// `ocs2_legged_robot` example runs `sqp_iterations=1` at a much higher
/// re-solve rate (100 Hz, dt=0.015s) than what this test originally
/// assumed was our default (`sqp_iterations=3 @ dt_per_step=0.030s`,
/// read off `FullCentroidalMpcConfig::default_with_kin`'s own literal
/// defaults) -- an RTI-style few-iterations/high-frequency tradeoff
/// instead of our presumed few-solves/thorough-iteration one.
///
/// **Correction**: `default_with_kin`'s literal
/// `(horizon_steps=20, dt_per_step=0.030, sqp_iterations=3)` is NOT
/// what Sec.5w/5x actually ran with -- `auto_detect_full_centroidal_
/// mpc_config` (`gait.rs`) overwrites exactly those three fields from
/// `auto_detect_centroidal_mpc_config`'s (12-state) result, which in
/// turn is just `CentroidalMpcConfig::default()`:
/// `(horizon_steps=10, dt_per_step=0.030, sqp_iterations=1)` --
/// i.e. the REAL baseline every prior FullCentroidal test in this file
/// actually ran at is a 0.3s horizon with a single SQP iteration, not
/// the 0.6s/3-iteration config this test originally (wrongly) used as
/// its "baseline" label. That mislabeled first attempt is itself an
/// interesting data point (going 20x0.030 sqp=3 -> sqp=1 flipped a
/// sustained-reversal failure into a stable plateau -- more SQP
/// iterations at that horizon made tracking *worse*, not better), but
/// it wasn't a comparison against Sec.5w at all. This corrected version
/// compares against the REAL default (10x0.030 sqp=1).
#[test]
#[ignore = "exploratory stress test — run with --ignored"]
fn go2_wbc_velocity_staircase_fine_full_centroidal_sqp_tuning() {
    let trials = [
        ("true baseline: 10x0.030s sqp=1 (Sec.5w/5x actual default)", 10, 0.030, 1),
        ("more iterations, same horizon: 10x0.030s sqp=3", 10, 0.030, 3),
        ("RTI-style, same 0.3s horizon: 20x0.015s sqp=1 (legged_control dt)", 20, 0.015, 1),
    ];
    for (label, horizon_steps, dt_per_step, sqp_iterations) in trials {
        let cfg = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
        let params = WbcParams::velocity_staircase_fine_full_centroidal_mpc_override_misa_wbc(
            wbc::Formulation::ForceSpace,
            cfg,
            true,
            false,
            horizon_steps,
            dt_per_step,
            sqp_iterations,
        );
        let Some(samples) = run_wbc_sim(params) else { continue };
        eprintln!("\n=== {label} ===");
        report_velocity_staircase(&samples, 0.05, 1.0, 60.0);
    }
}

fn assert_forward_command_advances_body(samples: &[WbcSample]) {
    let dt: f64 = 0.002;
    let burn_in_steps = (0.5 / dt).round() as usize;
    let walk = &samples[burn_in_steps..];

    let min_z = samples.iter().map(|s| s.body_z).fold(f64::INFINITY, f64::min);
    assert!(min_z > TRUNK_Z_FALL_THRESHOLD_M, "forward walk: trunk fell, min_z = {min_z:.3} m");

    let x_start = walk.first().map(|s| s.body_x).unwrap_or(0.0);
    let x_end = walk.last().map(|s| s.body_x).unwrap_or(0.0);
    let dx = x_end - x_start;
    eprintln!(
        "[wbc:go2] forward_command: Δx = {dx:.3} m over {:.1} s (threshold ≥ {:.2})",
        walk.last().map(|s| s.t).unwrap_or(0.0) - walk.first().map(|s| s.t).unwrap_or(0.0),
        MIN_DISPLACEMENT_M,
    );
    assert!(
        dx >= MIN_DISPLACEMENT_M,
        "forward walk: Δx = {dx:.3} m < {} m — WBC produced near-zero net forward motion",
        MIN_DISPLACEMENT_M,
    );

    let peak_roll = walk.iter().map(|s| s.roll.abs()).fold(0.0_f64, f64::max);
    let peak_pitch = walk.iter().map(|s| s.pitch.abs()).fold(0.0_f64, f64::max);
    eprintln!("[wbc:go2] forward_command: peak |roll| = {peak_roll:.2} rad, peak |pitch| = {peak_pitch:.2} rad");
    assert!(peak_roll < 0.5, "forward walk: |roll| peak {peak_roll:.2} rad too large");
    assert!(peak_pitch < 0.5, "forward walk: |pitch| peak {peak_pitch:.2} rad too large");
}
