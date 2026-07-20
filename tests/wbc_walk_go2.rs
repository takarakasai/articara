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
    foot_jacobian_body, solve_leg_ik, ContactDrivenPhase, GaitConfig, GaitMode, GaitType,
    KinematicsConfig, LegIkSolution, VelocityCmd,
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

/// Diagnostic (no MuJoCo needed): computes Go2's actual FL-leg
/// Jacobian at its nominal stance pose, to properly convert
/// legged_control's Cartesian-space swing-task PD gains
/// (`swingLegTask.kp=350, kd=37`, units 1/s^2 and 1/s on a *metre*
/// position/velocity error) into the joint-space equivalent our own
/// `WbcPipeline::{swing_kp, swing_kd}` uses (same units, but on a
/// *radian* error) — see Sec.5ag/5ai's finding that a naive same-
/// number import (350/37) degrades tracking, and the hypothesis that
/// this is partly a missing unit conversion (metres vs radians),
/// not purely an A1-vs-Go2 mass/torque mismatch.
///
/// Small-angle: `Δp_cartesian ≈ J · Δq_joint`, so
/// `kp_cart · Δp ≈ (kp_cart · ‖J‖) · Δq` — i.e. the correctly
/// converted joint-space gain is legged_control's Cartesian gain
/// scaled by the leg Jacobian's magnitude (metres of foot travel per
/// radian of joint rotation) at the nominal pose.
#[test]
#[ignore = "one-off diagnostic — prints the Jacobian-derived swing PD gain conversion"]
fn go2_diag_swing_pd_gain_jacobian_conversion() {
    let path = go2_misa();
    if !path.exists() {
        eprintln!("go2.misa missing at {} — skipping", path.display());
        return;
    }
    let robot = RobotModel::from_misa(&path).expect("load go2.misa");
    let kin = auto_detect_kinematics_config(&robot, &DEFAULT_FOOT_LINKS)
        .expect("auto-detect kinematics");
    let leg_kin = &kin.fl;
    let target = leg_kin.nominal_foot_body;
    let sol = solve_leg_ik(leg_kin, target, false);
    let LegIkSolution::Reached { hip, thigh, calf } = sol else {
        panic!("FL: nominal_foot_body unreachable");
    };
    let j = foot_jacobian_body(leg_kin, hip, thigh, calf);
    eprintln!("[jacobian] FL foot_jacobian_body at nominal pose (hip={hip:.3}, thigh={thigh:.3}, calf={calf:.3}):");
    eprintln!("{j:.4}");
    let svd = j.svd(true, true);
    eprintln!("[jacobian] singular values: {:.4}", svd.singular_values);
    let sigma_max = svd.singular_values.max();
    let sigma_min = svd.singular_values.min();
    let frobenius = j.norm();
    eprintln!(
        "[jacobian] sigma_max={sigma_max:.4} m/rad, sigma_min={sigma_min:.4} m/rad, frobenius={frobenius:.4}"
    );
    for (label, scale) in [("sigma_max", sigma_max), ("sigma_min", sigma_min), ("frobenius", frobenius)] {
        eprintln!(
            "[jacobian] using {label}={scale:.4}: kp_joint_equiv = 350*{scale:.4} = {:.2}, kd_joint_equiv = 37*{scale:.4} = {:.2}",
            350.0 * scale, 37.0 * scale,
        );
    }
}

/// Diagnostic (no MuJoCo needed): sweeps `GaitConfig::bound()`'s
/// `duty_factor` down from its 0.5 default and samples
/// `ContactDrivenPhase::nominal_legs()` at fine time resolution across
/// one full cycle, counting how many legs are in stance at each
/// sample.
///
/// Bound's phase offsets (`FL=FR=0.0, RL=RR=0.5`) exactly tile two
/// legs in stance at `duty=0.5` with zero gap — the front pair lifts
/// off exactly as the rear pair lands. Reducing duty below 0.5 should
/// open a genuine flight phase (0 legs in stance) twice per cycle —
/// the aerial phase real bounding/galloping gaits have, which neither
/// the SRBD MPC's `continuous_dynamics` nor the WBC's
/// `friction_cone`/`no_contact_motion` tasks have ever actually been
/// exercised against (both degrade gracefully to `n_stance=0` on
/// paper, per the code, but untested in practice). This is the cheap
/// sanity check before investing in a dedicated Canter/Gallop
/// `GaitType` (which would need asymmetric per-leg duty factors,
/// unsupported today): confirms the flight-phase *schedule* itself
/// behaves as expected before testing whether the *dynamics* handle
/// it.
#[test]
fn go2_diag_bound_duty_factor_flight_phase_sweep() {
    const N_SAMPLES: usize = 2000;
    for duty in [0.5, 0.45, 0.40, 0.35, 0.30, 0.25] {
        let cfg = GaitConfig::bound().with_duty_factor(duty);
        let dt = cfg.cycle_period_s / N_SAMPLES as f64;
        let mut phase = ContactDrivenPhase::new(cfg);
        let vel = VelocityCmd { vx: 0.3, vy: 0.0, wz: 0.0 };
        // Prime past the zero-velocity "holding" state before sampling.
        phase.advance(dt, &vel);
        let mut min_stance = 4usize;
        let mut zero_stance_samples = 0usize;
        for _ in 0..N_SAMPLES {
            let legs = phase.nominal_legs();
            let n_stance = legs.iter().filter(|p| p.is_stance).count();
            min_stance = min_stance.min(n_stance);
            if n_stance == 0 {
                zero_stance_samples += 1;
            }
            phase.advance(dt, &vel);
        }
        let flight_frac = zero_stance_samples as f64 / N_SAMPLES as f64;
        eprintln!(
            "[bound-duty-sweep] duty={duty:.2}: min_stance={min_stance}, flight_phase_fraction={flight_frac:.3} ({:.1}% of cycle)",
            flight_frac * 100.0,
        );
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
    /// Override `GaitConfig::trot()`'s `max_step_length_m` (default
    /// 0.10m) after construction. Sec.5aj found the observed velocity-
    /// tracking plateau (~0.46-0.48 m/s) matches almost exactly the
    /// Raibert footstep planner's own theoretical kinematic ceiling
    /// `v_max = max_step_length_m / (cycle_period_s * duty_factor)
    /// = 0.10 / (0.4 * 0.5) = 0.5 m/s` — i.e. this specific Trot
    /// configuration's own limit, not an algorithmic bug. Raising
    /// `max_step_length_m` (Go2's leg reach is ~0.426m, so 0.10m is
    /// only ~23% of it — real quadruped trots often use 30-50%) should
    /// raise this theoretical ceiling proportionally if the prediction
    /// is right. `None` keeps the existing 0.10m default.
    max_step_length_override: Option<f64>,
    /// Override `GaitConfig::swing_height_m` (0.04 for Trot, 0.05 for
    /// Bound) after construction. Sec.5ap follow-up: Bound's baseline-
    /// isolation survey found a large sustained pitch oscillation
    /// (0.27-0.34 rad) in every configuration tested, alongside a
    /// consistent forward-command reversal — swing height is the most
    /// direct lever on how hard each stance-to-swing transition kicks
    /// the trunk. `None` keeps the gait family's own default.
    swing_height_override: Option<f64>,
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
    /// Override `WbcPipeline::{swing_kp, swing_kd}` (default 80.0/8.0)
    /// after construction. Desk-research gap ⑤ (broad legged_control
    /// survey, 2026-07-18): `legged_wbc`'s `task.info` uses
    /// `swingLegTask.kp=350, kd=37` — same ~10:1 ratio we already use,
    /// but ~4.4x stiffer in absolute terms. This is the WBC's own
    /// Cartesian-tracking-equivalent joint-space stiffness (downstream
    /// of the MPC entirely — orthogonal to the already-tested
    /// task-space `joint_v` MPC-cost remap, gap ②). `None` keeps the
    /// existing (80.0, 8.0) default.
    swing_pd_gain_override: Option<(f64, f64)>,
    /// Override `WbcPipeline::friction_mu` (default 0.5) after
    /// construction. Sec.5as follow-up (external Explore-agent audit
    /// of `misa-wbc`'s `ho_qp.rs`, 2026-07-21): Bound's front-pair/
    /// rear-pair stance shares the same body-frame `r_x` moment arm
    /// between its two simultaneously-stance feet, so — unlike Trot's
    /// diagonal pair, which gets pitch torque nearly for free from
    /// `Δf_z · Δr_x` — Bound's pitch authority must come almost
    /// entirely from `Σf_x`, which is friction-cone-limited
    /// (`|f_x| ≤ μ·f_z`). This is the leading hypothesis for the
    /// `Σf_x` chaos and `f_z` saturation Sec.5as measured (raising
    /// `max_normal_force` didn't help because the real bottleneck is
    /// friction-limited `f_x`, not the normal-force cap itself). The
    /// sim's actual MJCF ground friction defaults to 0.7
    /// (`MjcfExportOptions::default_friction`'s sliding component) —
    /// higher than the WBC's conservative 0.5, so raising this up to
    /// 0.7 is a "free" real-world-consistent increase; higher values
    /// test the hypothesis further but would need matching ground
    /// friction to be physically deployable. `None` keeps the
    /// existing 0.5 default.
    friction_mu_override: Option<f64>,
    /// Override `WbcPipeline::pitch_pd_gain` (default (0.0, 0.0)) after
    /// construction. Sec.5au/5av: real model-based bounding controllers
    /// (Raibert's hopping-machine 3-way decomposition; MIT Cheetah 2/3)
    /// treat attitude control as an explicit, closed-loop feedback
    /// channel — not something inferred purely from the MPC's GRF
    /// allocation, which for Bound's front/rear-only stance is
    /// friction-cone-limited (Sec.5as/5at). `WbcPipeline::solve`'s
    /// `a_base_des` angular component was, until this change, pure
    /// feedforward from the MPC's optimised GRF via Newton-Euler, with
    /// no direct feedback on measured pitch error at all. `(kp, kd)`
    /// adds `kp*(0 - pitch) - kd*pitch_rate` directly on top of that
    /// feedforward. `None` keeps the existing (0.0, 0.0) no-op default.
    pitch_pd_gain_override: Option<(f64, f64)>,
    /// Scale every joint's `effort` (N·m torque limit) by this factor,
    /// applied to `robot.joints[*].effort` right after loading —
    /// before *both* the MJCF export (so MuJoCo's own actuator
    /// `forcerange` clamp relaxes too, not just the WBC's belief) and
    /// `WbcPipeline::new` (whose `torque_max` reads straight from
    /// `robot.joints[ji].effort`). Sec.5aw: measured Bound demanding
    /// torque up to 44.71 N·m — ~1.9x Go2's real 23.7 N·m hip/thigh
    /// limit — in 12.5% of sampled ticks (Trot: 0%), while MuJoCo
    /// silently clips the excess. Tests whether genuinely relaxing
    /// the actuator envelope (not just the WBC's internal belief)
    /// resolves the reversal. `None` keeps the real Go2 catalogue
    /// values (scale 1.0).
    actuator_effort_scale_override: Option<f64>,
    /// Override `MjcfExportOptions::default_friction`'s sliding
    /// component (default 0.7) — the REAL ground-foot friction MuJoCo
    /// simulates, as opposed to `friction_mu_override` (the WBC/MPC's
    /// own *belief* about available friction). Sec.5at raised
    /// `friction_mu` up to 5.0 while the ground stayed fixed at 0.7 —
    /// i.e. the WBC was often assuming *more* grip than physically
    /// existed, which would show up as slip. `None` keeps the 0.7
    /// default; `Some(mu)` typically wants to match whatever
    /// `friction_mu_override` is set to, for a physically consistent
    /// (not just solver-internal) comparison.
    ground_friction_override: Option<f64>,
    /// Override the base gait family (`GaitConfig::for_type(ty)`)
    /// instead of the hardcoded `GaitConfig::trot()`. Applied before
    /// `duty_factor_override`/`gait_cycle_period_override`/
    /// `max_step_length_override` so those still compose on top.
    /// `None` keeps `GaitConfig::trot()`.
    gait_type_override: Option<GaitType>,
    /// Override `GaitConfig::duty_factor` (default depends on
    /// `gait_type_override`, e.g. `0.5` for Bound) after construction.
    /// 2026-07-19 flight-phase validation: Bound's phase offsets
    /// (`FL=FR=0.0, RL=RR=0.5`) exactly tile two legs in stance at
    /// `duty=0.5`; reducing duty opens a genuine flight phase (0 legs
    /// in stance) twice per cycle, per `go2_diag_bound_duty_factor_
    /// flight_phase_sweep`'s schedule-level confirmation
    /// (`flight_fraction = 1 - 2*duty`). `None` keeps the gait
    /// family's own default duty factor.
    duty_factor_override: Option<f64>,
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
/// `task_space_joint_vel_weight`: replace the flat per-joint `joint_v`
/// cost with a task-space (foot-velocity) weight mapped through each
/// leg's fixed-nominal-pose Jacobian — legged_control/OCS2's own
/// technique (`ocs2_legged_robot`'s `initializeInputCostWeight`,
/// confirmed against `ref/ocs2`), point ② from the desk-research gap
/// analysis (Sec.5v-era discussion).
/// `true_centroidal_coupling`: desk-research gap ① — an additive bias
/// term (from `misarta`'s CRBA-based centroidal momentum matrix) that
/// couples joint velocity/acceleration into the base's predicted
/// motion, matching OCS2's `centroidalModelType=FullCentroidalDynamics`
/// coupling without changing our own state representation (see
/// `FullCentroidalMpcConfig`'s doc comment, confirmed against
/// `ref/ocs2`).
#[derive(Clone, Copy)]
struct FullCentroidalOpts {
    legged_control_parity: bool,
    use_mpc_predicted_footstep: bool,
    dynamic_joint_q_reference: bool,
    mpc_override: Option<(usize, f64, usize)>,
    task_space_joint_vel_weight: Option<[f64; 3]>,
    true_centroidal_coupling: bool,
    /// Override `k_capture` (default `0.05`) after `GaitController::build`.
    /// Confounder check: `0.05` was tuned in the unrelated "η experiment"
    /// (2026-05-15, lateral-push disturbance recovery on the pre-
    /// FullCentroidal SRBD path) and has never been re-tuned against
    /// any of the ①②③ plants it's since been reused on unchanged. The
    /// code's own doc comment on `DEFAULT_CAPTURE_POINT_GAIN_S` notes
    /// legged_control itself uses `0.0` — its reference tracking closes
    /// the loop differently. `None` keeps the existing 0.05 default.
    capture_point_gain_override: Option<f64>,
    /// Override `q_diag[6]`/`q_diag[7]` (base position x/y tracking
    /// weight) after `GaitController::build`. Desk-research finding
    /// (broad legged_control survey, 2026-07-18): legged_control
    /// weights base **position** tracking at 1000/1000/1500 (x/y/z,
    /// `task.info`'s `Q` block) — roughly 50-67x its own v_com weight
    /// (15) — against the *same* velocity-ramp position reference our
    /// own controller already builds
    /// (`full_centroidal_controller.rs`'s `sk.base_pos_world = s_now...
    /// + v_world_cmd*t`). Our own `q_diag[6]` (x) is literally `0.0`
    /// and `q_diag[7]` (y) is `5.0` (`default_with_kin`) — `q_diag[8]`
    /// (z) is already `50.0`, a value never questioned before this
    /// survey. `None` keeps the existing (0.0, 5.0) default.
    base_pos_xy_weight_override: Option<(f64, f64)>,
    /// Override `FullCentroidalMpcConfig::max_normal_force` (default
    /// 200.0 N) after `GaitController::build`. Desk-research gap ⑥
    /// (broad legged_control survey, 2026-07-18): legged_control's
    /// `FrictionConeConstraint` has no upper bound on `f_z` anywhere —
    /// only `frictionCoefficient`/`regularization`/`gripperForce`
    /// (`task.info`, `ocs2_legged_robot`'s `FrictionConeConstraint.h`).
    /// `None` keeps the existing 200.0 N cap; `Some(f64::INFINITY)`
    /// removes it entirely, matching legged_control.
    max_normal_force_override: Option<f64>,
    /// Override `q_diag[9]`/`q_diag[10]` (base roll/pitch attitude
    /// tracking weight, default 25.0/25.0) after `GaitController::build`.
    /// Sec.5al found peak body roll roughly doubles (0.06→0.10 rad) as
    /// `max_step_length_m` rises 0.10→0.20, plausibly eating into the
    /// tracking budget available for forward velocity. `None` keeps
    /// the existing (25.0, 25.0) default.
    roll_pitch_weight_override: Option<(f64, f64)>,
}

impl WbcParams {
    fn static_stand() -> Self {
        Self {
            total_time_s: 1.5, burn_in_s: 0.5, cmd_vx: 0.0, dt: 0.002,
            misa_wbc_mode: None, staircase_step_s: None,
            staircase_step_mps: 0.0, staircase_max_mps: 0.0,
            mpc_horizon_override: None, gait_cycle_period_override: None, max_step_length_override: None,
            swing_height_override: None,
            body_height_bias_frac: None, full_centroidal: None,
            swing_pd_gain_override: None, friction_mu_override: None, pitch_pd_gain_override: None, actuator_effort_scale_override: None, ground_friction_override: None,
            gait_type_override: None, duty_factor_override: None,
        }
    }
    fn forward_walk() -> Self {
        Self {
            total_time_s: 3.0, burn_in_s: 0.5, cmd_vx: 0.15, dt: 0.002,
            misa_wbc_mode: None, staircase_step_s: None,
            staircase_step_mps: 0.0, staircase_max_mps: 0.0,
            mpc_horizon_override: None, gait_cycle_period_override: None, max_step_length_override: None,
            swing_height_override: None,
            body_height_bias_frac: None, full_centroidal: None,
            swing_pd_gain_override: None, friction_mu_override: None, pitch_pd_gain_override: None, actuator_effort_scale_override: None, ground_friction_override: None,
            gait_type_override: None, duty_factor_override: None,
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
            max_step_length_override: None,
            swing_height_override: None,
            body_height_bias_frac: None,
            full_centroidal: None,
            swing_pd_gain_override: None, friction_mu_override: None, pitch_pd_gain_override: None, actuator_effort_scale_override: None, ground_friction_override: None,
            gait_type_override: None,
            duty_factor_override: None,
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
                mpc_override: None, task_space_joint_vel_weight: None,
                true_centroidal_coupling: false, capture_point_gain_override: None,
                base_pos_xy_weight_override: None, max_normal_force_override: None,
                roll_pitch_weight_override: None,
            }),
            ..Self::velocity_staircase_fine_misa_wbc(formulation, cfg)
        }
    }

    /// Same as [`Self::velocity_staircase_fine_full_centroidal_dynamic_q_misa_wbc`],
    /// also toggling `true_centroidal_coupling` — see
    /// `FullCentroidalOpts`'s doc comment.
    fn velocity_staircase_fine_full_centroidal_true_coupling_misa_wbc(
        formulation: wbc::Formulation,
        cfg: wbc::SolveConfig,
        legged_control_parity: bool,
        true_centroidal_coupling: bool,
    ) -> Self {
        let mut params = Self::velocity_staircase_fine_full_centroidal_dynamic_q_misa_wbc(
            formulation, cfg, legged_control_parity, false, false,
        );
        let opts = params.full_centroidal.as_mut().expect("full_centroidal always Some here");
        opts.true_centroidal_coupling = true_centroidal_coupling;
        params
    }

    /// Same as [`Self::velocity_staircase_fine_full_centroidal_true_coupling_misa_wbc`],
    /// also overriding `k_capture` — see
    /// `FullCentroidalOpts::capture_point_gain_override`'s doc comment.
    /// The confounder-check experiment: does the reversal Sec.5aa found
    /// persist once the gain isn't a leftover from an unrelated plant?
    fn velocity_staircase_fine_full_centroidal_true_coupling_kcap_misa_wbc(
        formulation: wbc::Formulation,
        cfg: wbc::SolveConfig,
        legged_control_parity: bool,
        true_centroidal_coupling: bool,
        k_capture: f64,
    ) -> Self {
        let mut params = Self::velocity_staircase_fine_full_centroidal_true_coupling_misa_wbc(
            formulation, cfg, legged_control_parity, true_centroidal_coupling,
        );
        let opts = params.full_centroidal.as_mut().expect("full_centroidal always Some here");
        opts.capture_point_gain_override = Some(k_capture);
        params
    }

    /// Desk-research gap ④ (broad legged_control survey, 2026-07-18):
    /// override `q_diag[6]`/`q_diag[7]` (base position x/y tracking
    /// weight) on top of the now-healthy `k_capture=0` baseline — see
    /// `FullCentroidalOpts::base_pos_xy_weight_override`'s doc comment.
    fn velocity_staircase_fine_full_centroidal_base_pos_weight_misa_wbc(
        formulation: wbc::Formulation,
        cfg: wbc::SolveConfig,
        legged_control_parity: bool,
        k_capture: f64,
        q_pos_x: f64,
        q_pos_y: f64,
    ) -> Self {
        let mut params = Self::velocity_staircase_fine_full_centroidal_true_coupling_kcap_misa_wbc(
            formulation, cfg, legged_control_parity, false, k_capture,
        );
        let opts = params.full_centroidal.as_mut().expect("full_centroidal always Some here");
        opts.base_pos_xy_weight_override = Some((q_pos_x, q_pos_y));
        params
    }

    /// Desk-research gap ⑤ (broad legged_control survey, 2026-07-18):
    /// override `WbcPipeline::{swing_kp, swing_kd}` on top of the
    /// healthy `k_capture=0` baseline — see
    /// `WbcParams::swing_pd_gain_override`'s doc comment.
    fn velocity_staircase_fine_full_centroidal_swing_pd_misa_wbc(
        formulation: wbc::Formulation,
        cfg: wbc::SolveConfig,
        legged_control_parity: bool,
        k_capture: f64,
        swing_kp: f64,
        swing_kd: f64,
    ) -> Self {
        let mut params = Self::velocity_staircase_fine_full_centroidal_true_coupling_kcap_misa_wbc(
            formulation, cfg, legged_control_parity, false, k_capture,
        );
        params.swing_pd_gain_override = Some((swing_kp, swing_kd));
        params
    }

    /// Desk-research gap ⑥ (broad legged_control survey, 2026-07-18):
    /// override `FullCentroidalMpcConfig::max_normal_force` on top of
    /// the healthy `k_capture=0` baseline — see
    /// `FullCentroidalOpts::max_normal_force_override`'s doc comment.
    fn velocity_staircase_fine_full_centroidal_max_normal_force_misa_wbc(
        formulation: wbc::Formulation,
        cfg: wbc::SolveConfig,
        legged_control_parity: bool,
        k_capture: f64,
        max_normal_force: f64,
    ) -> Self {
        let mut params = Self::velocity_staircase_fine_full_centroidal_true_coupling_kcap_misa_wbc(
            formulation, cfg, legged_control_parity, false, k_capture,
        );
        let opts = params.full_centroidal.as_mut().expect("full_centroidal always Some here");
        opts.max_normal_force_override = Some(max_normal_force);
        params
    }

    /// Sec.5aj follow-up: override `GaitConfig::trot()`'s
    /// `max_step_length_m` on top of the healthy `k_capture=0`
    /// baseline — see `WbcParams::max_step_length_override`'s doc
    /// comment.
    fn velocity_staircase_fine_full_centroidal_max_step_length_misa_wbc(
        formulation: wbc::Formulation,
        cfg: wbc::SolveConfig,
        legged_control_parity: bool,
        k_capture: f64,
        max_step_length_m: f64,
    ) -> Self {
        let mut params = Self::velocity_staircase_fine_full_centroidal_true_coupling_kcap_misa_wbc(
            formulation, cfg, legged_control_parity, false, k_capture,
        );
        params.max_step_length_override = Some(max_step_length_m);
        params
    }

    /// Sec.5al follow-up: override `q_diag[9]`/`q_diag[10]` (roll/pitch
    /// attitude weight) on top of the `max_step_length_m=0.20` baseline
    /// — see `FullCentroidalOpts::roll_pitch_weight_override`'s doc
    /// comment.
    fn velocity_staircase_fine_full_centroidal_roll_pitch_weight_misa_wbc(
        formulation: wbc::Formulation,
        cfg: wbc::SolveConfig,
        legged_control_parity: bool,
        max_step_length_m: f64,
        q_roll: f64,
        q_pitch: f64,
    ) -> Self {
        let mut params = Self::velocity_staircase_fine_full_centroidal_max_step_length_misa_wbc(
            formulation, cfg, legged_control_parity, 0.0, max_step_length_m,
        );
        let opts = params.full_centroidal.as_mut().expect("full_centroidal always Some here");
        opts.roll_pitch_weight_override = Some((q_roll, q_pitch));
        params
    }

    /// Same as [`Self::velocity_staircase_fine_full_centroidal_dynamic_q_misa_wbc`],
    /// also setting the task-space→joint-space `joint_v` weight
    /// mapping — see `FullCentroidalOpts::task_space_joint_vel_weight`'s
    /// doc comment.
    fn velocity_staircase_fine_full_centroidal_taskspace_weight_misa_wbc(
        formulation: wbc::Formulation,
        cfg: wbc::SolveConfig,
        legged_control_parity: bool,
        r_taskspace: [f64; 3],
    ) -> Self {
        let mut params = Self::velocity_staircase_fine_full_centroidal_dynamic_q_misa_wbc(
            formulation, cfg, legged_control_parity, false, false,
        );
        let opts = params.full_centroidal.as_mut().expect("full_centroidal always Some here");
        opts.task_space_joint_vel_weight = Some(r_taskspace);
        params
    }

    /// Same as [`Self::velocity_staircase_fine_full_centroidal_taskspace_weight_misa_wbc`],
    /// also overriding `k_capture` — the Sec.5ab confounder-check
    /// extended to ②, since it shared the same un-retuned gain.
    fn velocity_staircase_fine_full_centroidal_taskspace_weight_kcap_misa_wbc(
        formulation: wbc::Formulation,
        cfg: wbc::SolveConfig,
        legged_control_parity: bool,
        r_taskspace: [f64; 3],
        k_capture: f64,
    ) -> Self {
        let mut params = Self::velocity_staircase_fine_full_centroidal_taskspace_weight_misa_wbc(
            formulation, cfg, legged_control_parity, r_taskspace,
        );
        let opts = params.full_centroidal.as_mut().expect("full_centroidal always Some here");
        opts.capture_point_gain_override = Some(k_capture);
        params
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

    /// 2026-07-19 flight-phase validation (Canter/Gallop scoping):
    /// `GaitMode::FullCentroidal` + `legged_control_parity` +
    /// `k_capture=0` (the established healthy baseline) on
    /// `GaitType::Bound` with `duty_factor` reduced below its 0.5
    /// default, opening a genuine flight phase (0 legs in stance)
    /// twice per cycle — see `WbcParams::duty_factor_override`'s doc
    /// comment and `go2_diag_bound_duty_factor_flight_phase_sweep`'s
    /// schedule-level confirmation. Single fixed-speed forward walk
    /// (not a staircase): the question here is "does the dynamics
    /// survive an aerial phase at all", not a tracking-ceiling sweep.
    fn bound_flight_phase_full_centroidal_misa_wbc(
        formulation: wbc::Formulation,
        cfg: wbc::SolveConfig,
        cmd_vx: f64,
        duty_factor: f64,
    ) -> Self {
        Self {
            full_centroidal: Some(FullCentroidalOpts {
                legged_control_parity: true,
                use_mpc_predicted_footstep: false,
                dynamic_joint_q_reference: false,
                mpc_override: None,
                task_space_joint_vel_weight: None,
                true_centroidal_coupling: false,
                capture_point_gain_override: Some(0.0),
                base_pos_xy_weight_override: None,
                max_normal_force_override: None,
                roll_pitch_weight_override: None,
            }),
            gait_type_override: Some(GaitType::Bound),
            duty_factor_override: Some(duty_factor),
            cmd_vx,
            total_time_s: 5.0,
            burn_in_s: 0.5,
            ..Self::forward_walk_misa_wbc(formulation, cfg)
        }
    }
}

fn run_wbc_sim(params: WbcParams) -> Option<Vec<WbcSample>> {
    let path = go2_misa();
    if !path.exists() {
        eprintln!("go2.misa missing at {} — skipping Go2 WBC test", path.display());
        return None;
    }
    let mut robot = RobotModel::from_misa(&path).expect("load go2.misa");
    if let Some(scale) = params.actuator_effort_scale_override {
        for joint in &mut robot.joints {
            let before = joint.effort;
            joint.effort *= scale;
            if before > 0.0 {
                eprintln!("[actuator] {} effort {:.2} -> {:.2} N·m", joint.name, before, joint.effort);
            }
        }
    }

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

    let mut opts = MjcfExportOptions {
        base_pos: Some([0.0, 0.0, 0.30]),
        ground_plane: Some(GroundPlaneCfg { z: 0.0, half_size: 4.0, roll: 0.0, pitch: 0.0 }),
        add_actuators: true,
        ..Default::default()
    };
    if let Some(mu_ground) = params.ground_friction_override {
        eprintln!(
            "[ground] default_friction[0] (sliding) {:.2} -> {:.2}",
            opts.default_friction[0], mu_ground,
        );
        opts.default_friction[0] = mu_ground;
    }
    let mut sim = MujocoSim::new(&robot, opts).expect("MujocoSim::new");
    sim.set_gravity_compensation(true);

    let mut cfg = match params.gait_type_override {
        Some(ty) => GaitConfig::for_type(ty),
        None => GaitConfig::trot(),
    };
    if let Some(duty_factor) = params.duty_factor_override {
        eprintln!(
            "[gait] overriding duty_factor {:.3} -> {:.3} (flight_phase_fraction {:.3} -> {:.3})",
            cfg.duty_factor, duty_factor,
            (1.0 - 2.0 * cfg.duty_factor).max(0.0),
            (1.0 - 2.0 * duty_factor).max(0.0),
        );
        cfg = cfg.with_duty_factor(duty_factor);
    }
    if let Some(cycle_period_s) = params.gait_cycle_period_override {
        eprintln!(
            "[gait-cycle] overriding cycle_period_s {:.3}s -> {:.3}s",
            cfg.cycle_period_s, cycle_period_s
        );
        cfg.cycle_period_s = cycle_period_s;
    }
    if let Some(max_step_length_m) = params.max_step_length_override {
        eprintln!(
            "[gait] overriding max_step_length_m {:.3}m -> {:.3}m (theoretical v_max {:.3} -> {:.3} m/s)",
            cfg.max_step_length_m, max_step_length_m,
            cfg.max_step_length_m / (cfg.cycle_period_s * cfg.duty_factor),
            max_step_length_m / (cfg.cycle_period_s * cfg.duty_factor),
        );
        cfg.max_step_length_m = max_step_length_m;
    }
    if let Some(swing_height_m) = params.swing_height_override {
        eprintln!(
            "[gait] overriding swing_height_m {:.3}m -> {:.3}m",
            cfg.swing_height_m, swing_height_m,
        );
        cfg.swing_height_m = swing_height_m;
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
        if let Some(r_taskspace) = opts.task_space_joint_vel_weight {
            eprintln!("[full-centroidal] task_space_joint_vel_weight={r_taskspace:?}");
            gc.set_task_space_joint_vel_weight(Some(r_taskspace));
        }
        if opts.true_centroidal_coupling {
            gc.set_true_centroidal_coupling(true);
            eprintln!(
                "[full-centroidal] true_centroidal_coupling enabled (data available: {})",
                gc.full_centroidal_mpc_config()
                    .map(|c| c.true_centroidal_coupling_data.is_some())
                    .unwrap_or(false),
            );
        }
        if let Some(k) = opts.capture_point_gain_override {
            eprintln!("[full-centroidal] k_capture override -> {k:.3}");
            gc.set_capture_point_gain(k);
        }
        if let Some((q_x, q_y)) = opts.base_pos_xy_weight_override {
            let mut mpc_cfg: FullCentroidalMpcConfig =
                gc.full_centroidal_mpc_config().expect("FullCentroidal mode has a config").clone();
            eprintln!(
                "[full-centroidal] q_diag[6..8] (base pos x/y) {:.1}/{:.1} -> {:.1}/{:.1}",
                mpc_cfg.q_diag[6], mpc_cfg.q_diag[7], q_x, q_y,
            );
            mpc_cfg.q_diag[6] = q_x;
            mpc_cfg.q_diag[7] = q_y;
            gc.set_full_centroidal_mpc_config(mpc_cfg);
        }
        if let Some(f_max) = opts.max_normal_force_override {
            let mut mpc_cfg: FullCentroidalMpcConfig =
                gc.full_centroidal_mpc_config().expect("FullCentroidal mode has a config").clone();
            eprintln!(
                "[full-centroidal] max_normal_force {:.1} -> {:.1}",
                mpc_cfg.max_normal_force, f_max,
            );
            mpc_cfg.max_normal_force = f_max;
            gc.set_full_centroidal_mpc_config(mpc_cfg);
        }
        if let Some((q_roll, q_pitch)) = opts.roll_pitch_weight_override {
            let mut mpc_cfg: FullCentroidalMpcConfig =
                gc.full_centroidal_mpc_config().expect("FullCentroidal mode has a config").clone();
            eprintln!(
                "[full-centroidal] q_diag[9..11] (roll/pitch) {:.1}/{:.1} -> {:.1}/{:.1}",
                mpc_cfg.q_diag[9], mpc_cfg.q_diag[10], q_roll, q_pitch,
            );
            mpc_cfg.q_diag[9] = q_roll;
            mpc_cfg.q_diag[10] = q_pitch;
            gc.set_full_centroidal_mpc_config(mpc_cfg);
        }
        if let Some(mu) = params.friction_mu_override {
            let mut mpc_cfg: FullCentroidalMpcConfig =
                gc.full_centroidal_mpc_config().expect("FullCentroidal mode has a config").clone();
            eprintln!("[full-centroidal] friction_mu {:.2} -> {:.2}", mpc_cfg.friction_mu, mu);
            mpc_cfg.friction_mu = mu;
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
    if let Some((kp, kd)) = params.swing_pd_gain_override {
        eprintln!(
            "[wbc] swing_kp/kd {:.1}/{:.1} -> {:.1}/{:.1}",
            wbc_pipeline.swing_kp, wbc_pipeline.swing_kd, kp, kd,
        );
        wbc_pipeline.swing_kp = kp;
        wbc_pipeline.swing_kd = kd;
    }
    if let Some(mu) = params.friction_mu_override {
        eprintln!("[wbc] friction_mu {:.2} -> {:.2}", wbc_pipeline.friction_mu, mu);
        wbc_pipeline.friction_mu = mu;
    }
    if let Some((kp, kd)) = params.pitch_pd_gain_override {
        eprintln!(
            "[wbc] pitch_pd_gain {:?} -> ({:.1}, {:.1})",
            wbc_pipeline.pitch_pd_gain, kp, kd,
        );
        wbc_pipeline.pitch_pd_gain = (kp, kd);
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
                    let mpc_fx_sum: f64 = f_grf_world.iter().map(|v| v.x).sum();
                    let stance_count = contact_flag.iter().filter(|b| **b).count();
                    eprintln!(
                        "[diag k={k:5} t={:.3}s] z={:.3} m  Σmpc_f_z={:.2} N  Σmpc_f_x={:.2} N  max|τ|={:.2} N·m  stance={}/4",
                        k as f64 * params.dt, body_pos[2], mpc_fz_sum, mpc_fx_sum, tau_max, stance_count,
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

/// Desk-research gap ② (`ref/ocs2` verified): legged_control/OCS2's
/// `ocs2_legged_robot` example maps its `joint_v` cost through each
/// leg's fixed-nominal-pose Jacobian (`R_jointspace = J_nom^T *
/// R_taskspace * J_nom`) instead of a flat per-joint diagonal. Tests
/// this against the Sec.5w/5y true default (`legged_control_parity`,
/// horizon 10x0.030s sqp=1) with `r_taskspace = [1,1,1]` — the same
/// overall scale as the existing flat `r_diag[12..24] = 1.0` default,
/// isolating the Jacobian *shape* effect from any weight-magnitude
/// change.
#[test]
#[ignore = "exploratory stress test — run with --ignored"]
fn go2_wbc_velocity_staircase_fine_full_centroidal_taskspace_weight() {
    let cfg = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
    let params = WbcParams::velocity_staircase_fine_full_centroidal_taskspace_weight_misa_wbc(
        wbc::Formulation::ForceSpace,
        cfg,
        true,
        [1.0, 1.0, 1.0],
    );
    let Some(samples) = run_wbc_sim(params) else { return };
    eprintln!("\n=== FullCentroidal + parity + task_space_joint_vel_weight([1,1,1]) ===");
    report_velocity_staircase(&samples, 0.05, 1.0, 60.0);
}

/// Desk-research gap ① (`ref/ocs2` verified): OCS2's
/// `centroidalModelType=FullCentroidalDynamics` couples joint velocity
/// into the base's predicted motion via the centroidal momentum
/// matrix. Implemented as an additive bias term (not a state-
/// representation change — see `FullCentroidalMpcConfig`'s doc
/// comment) gated by `true_centroidal_coupling`. Tests it against the
/// Sec.5w/5y true default (`legged_control_parity`, horizon 10x0.030s,
/// sqp=1) — the same baseline ②③ were compared against.
#[test]
#[ignore = "exploratory stress test — run with --ignored"]
fn go2_wbc_velocity_staircase_fine_full_centroidal_true_coupling() {
    let cfg = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
    let params = WbcParams::velocity_staircase_fine_full_centroidal_true_coupling_misa_wbc(
        wbc::Formulation::ForceSpace,
        cfg,
        true,
        true,
    );
    let Some(samples) = run_wbc_sim(params) else { return };
    eprintln!("\n=== FullCentroidal + parity + true_centroidal_coupling ===");
    report_velocity_staircase(&samples, 0.05, 1.0, 60.0);
}

/// Confounder check on Sec.5aa's reversal: `k_capture=0.05` (the
/// default `set_capture_point_gain` never overrides in any ①②③ test)
/// was tuned in an unrelated 2026-05-15 disturbance-recovery experiment
/// against the pre-FullCentroidal SRBD plant, and legged_control itself
/// uses `k_capture=0` (its own reference-tracking loop closes
/// differently — see `DEFAULT_CAPTURE_POINT_GAIN_S`'s doc comment).
/// Tests whether Sec.5aa's high-speed reversal survives once that
/// mismatched gain is removed, and whether removing it changes the
/// already-healthy baseline on its own.
#[test]
#[ignore = "exploratory stress test — run with --ignored"]
fn go2_wbc_velocity_staircase_fine_full_centroidal_true_coupling_kcap_zero() {
    let trials = [
        ("true_centroidal_coupling + k_capture=0.0 (confounder removed)", true, 0.0),
        ("baseline (no coupling) + k_capture=0.0 (isolate gain-alone effect)", false, 0.0),
    ];
    for (label, true_centroidal_coupling, k_capture) in trials {
        let cfg = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
        let params = WbcParams::velocity_staircase_fine_full_centroidal_true_coupling_kcap_misa_wbc(
            wbc::Formulation::ForceSpace,
            cfg,
            true,
            true_centroidal_coupling,
            k_capture,
        );
        let Some(samples) = run_wbc_sim(params) else { continue };
        eprintln!("\n=== {label} ===");
        report_velocity_staircase(&samples, 0.05, 1.0, 60.0);
    }
}

/// Extends the Sec.5ab confounder check to ② and ③: both experiments
/// also ran with the un-retuned `k_capture=0.05` leftover from the
/// unrelated 2026-05-15 disturbance-recovery tuning, and neither ever
/// touched the gain. `k_capture=0` fully fixed ①'s reversal; this
/// checks whether it does the same for ②'s task-space joint_v weight
/// reversal (Sec.5z, cmd=0.80: -0.024) and ③'s worst SQP-tuning
/// reversal (Sec.5y, 20x0.030s sqp=3, cmd=1.0: -0.356).
#[test]
#[ignore = "exploratory stress test — run with --ignored"]
fn go2_wbc_velocity_staircase_fine_kcap_zero_recheck_2_3() {
    let cfg_ = || wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };

    // ② recheck.
    {
        let params = WbcParams::velocity_staircase_fine_full_centroidal_taskspace_weight_kcap_misa_wbc(
            wbc::Formulation::ForceSpace,
            cfg_(),
            true,
            [1.0, 1.0, 1.0],
            0.0,
        );
        if let Some(samples) = run_wbc_sim(params) {
            eprintln!("\n=== ② recheck: task_space_joint_vel_weight([1,1,1]) + k_capture=0.0 ===");
            report_velocity_staircase(&samples, 0.05, 1.0, 60.0);
        }
    }

    // ③ recheck: the worst Sec.5y case (20x0.030s sqp=3, the "more
    // iterations at 0.6s horizon" reversal), now at k_capture=0.
    {
        let mut params = WbcParams::velocity_staircase_fine_full_centroidal_mpc_override_misa_wbc(
            wbc::Formulation::ForceSpace,
            cfg_(),
            true,
            false,
            20,
            0.030,
            3,
        );
        let opts = params.full_centroidal.as_mut().expect("full_centroidal always Some here");
        opts.capture_point_gain_override = Some(0.0);
        if let Some(samples) = run_wbc_sim(params) {
            eprintln!("\n=== ③ recheck: 20x0.030s sqp=3 (Sec.5y worst case) + k_capture=0.0 ===");
            report_velocity_staircase(&samples, 0.05, 1.0, 60.0);
        }
    }
}

/// Desk-research gap ④ (broad legged_control survey, 2026-07-18):
/// legged_control weights base **position** tracking at 1000/1000/1500
/// (x/y/z) against the same velocity-ramp reference our own controller
/// already builds — ~50-67x its own v_com weight (15). Our own
/// `q_diag[6]` (base pos x) is literally `0.0` and `q_diag[7]` (y) is
/// `5.0`; `q_diag[8]` (z) is already `50.0` and was never questioned by
/// this survey. Sweeps `q_diag[6]=q_diag[7]` on top of the healthy
/// `k_capture=0` baseline (matching `q_diag[8]`'s existing value as the
/// simplest hypothesis: comparable priority on all three position
/// axes) to see whether closing this gap helps, following the same
/// "test on the confound-free baseline" discipline as Sec.5ae.
#[test]
#[ignore = "exploratory stress test — run with --ignored"]
fn go2_wbc_velocity_staircase_fine_base_pos_weight() {
    let trials = [
        ("q_diag[6..8]=(0,5,50) (current default)", 0.0, 5.0),
        ("q_diag[6..8]=(25,25,50)", 25.0, 25.0),
        ("q_diag[6..8]=(50,50,50) (match z)", 50.0, 50.0),
    ];
    for (label, q_x, q_y) in trials {
        let cfg = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
        let params = WbcParams::velocity_staircase_fine_full_centroidal_base_pos_weight_misa_wbc(
            wbc::Formulation::ForceSpace,
            cfg,
            true,
            0.0,
            q_x,
            q_y,
        );
        let Some(samples) = run_wbc_sim(params) else { continue };
        eprintln!("\n=== {label} ===");
        report_velocity_staircase(&samples, 0.05, 1.0, 60.0);
    }
}

/// Desk-research gap ⑤ (broad legged_control survey, 2026-07-18):
/// `legged_wbc`'s `swingLegTask.kp=350, kd=37` vs our own
/// `WbcPipeline::{swing_kp: 80.0, swing_kd: 8.0}` — same ~10:1 ratio,
/// ~4.4x stiffer in absolute terms. Sweeps toward legged_control's
/// actual values on top of the healthy `k_capture=0` baseline.
#[test]
#[ignore = "exploratory stress test — run with --ignored"]
fn go2_wbc_velocity_staircase_fine_swing_pd_gain() {
    let trials = [
        ("swing_kp/kd=80/8 (current default)", 80.0, 8.0),
        ("swing_kp/kd=175/18.5 (halfway to legged_control)", 175.0, 18.5,
        ),
        ("swing_kp/kd=350/37 (legged_control's value)", 350.0, 37.0),
    ];
    for (label, kp, kd) in trials {
        let cfg = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
        let params = WbcParams::velocity_staircase_fine_full_centroidal_swing_pd_misa_wbc(
            wbc::Formulation::ForceSpace,
            cfg,
            true,
            0.0,
            kp,
            kd,
        );
        let Some(samples) = run_wbc_sim(params) else { continue };
        eprintln!("\n=== {label} ===");
        report_velocity_staircase(&samples, 0.05, 1.0, 60.0);
    }
}

/// Follow-up to Sec.5ag/5ai: the raw `swing_kp/kd=350/37` import
/// degraded tracking, but that number is a *Cartesian-space* gain
/// (legged_control's WBC swing task is a foot-position/velocity PD,
/// units 1/s^2 and 1/s on a *metre* error) while our own
/// `WbcPipeline::{swing_kp, swing_kd}` is a *joint-space* PD (same
/// units, but on a *radian* error) — comparing "350" to "80" directly
/// was never a like-for-like comparison. `go2_diag_swing_pd_gain_
/// jacobian_conversion` computed Go2's actual FL-leg Jacobian at its
/// nominal stance pose: singular values 0.317/0.280/0.133 m/rad,
/// Frobenius norm 0.443. Using those as the "metres of foot travel
/// per radian" conversion factor gives properly-dimensioned joint-
/// space equivalents of roughly 111-155 (kp) / 12-16 (kd) — much
/// closer to (moderately above) our own default than to the raw 350.
#[test]
#[ignore = "exploratory stress test — run with --ignored"]
fn go2_wbc_velocity_staircase_fine_swing_pd_gain_jacobian_converted() {
    let trials = [
        ("swing_kp/kd=80/8 (current default)", 80.0, 8.0),
        ("swing_kp/kd=111/12 (sigma_max-converted 350/37)", 111.0, 12.0),
        ("swing_kp/kd=155/16 (frobenius-converted 350/37)", 155.0, 16.0),
    ];
    for (label, kp, kd) in trials {
        let cfg = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
        let params = WbcParams::velocity_staircase_fine_full_centroidal_swing_pd_misa_wbc(
            wbc::Formulation::ForceSpace,
            cfg,
            true,
            0.0,
            kp,
            kd,
        );
        let Some(samples) = run_wbc_sim(params) else { continue };
        eprintln!("\n=== {label} ===");
        report_velocity_staircase(&samples, 0.05, 1.0, 60.0);
    }
}

/// Desk-research gap ⑥ (broad legged_control survey, 2026-07-18):
/// legged_control's `FrictionConeConstraint` has no upper bound on
/// `f_z` anywhere; ours caps `max_normal_force` at 200N. Tests removing
/// the cap on top of the healthy `k_capture=0` baseline.
#[test]
#[ignore = "exploratory stress test — run with --ignored"]
fn go2_wbc_velocity_staircase_fine_max_normal_force() {
    let trials = [
        ("max_normal_force=200N (current default)", 200.0),
        ("max_normal_force=inf (legged_control has no cap)", f64::INFINITY),
    ];
    for (label, f_max) in trials {
        let cfg = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
        let params = WbcParams::velocity_staircase_fine_full_centroidal_max_normal_force_misa_wbc(
            wbc::Formulation::ForceSpace,
            cfg,
            true,
            0.0,
            f_max,
        );
        let Some(samples) = run_wbc_sim(params) else { continue };
        eprintln!("\n=== {label} ===");
        report_velocity_staircase(&samples, 0.05, 1.0, 60.0);
    }
}

/// Sec.5aj's kinematic-ceiling hypothesis, tested directly: the
/// observed ~0.46-0.48 m/s plateau matches
/// `v_max = max_step_length_m / (cycle_period_s * duty_factor)
/// = 0.10 / 0.2 = 0.5 m/s` almost exactly. If this is really the
/// binding constraint (not some other tuning artifact), raising
/// `max_step_length_m` should raise the plateau proportionally:
/// `0.15m -> 0.75 m/s`, `0.20m -> 1.0 m/s` (Go2's total leg reach is
/// ~0.426m, so 0.20m is still under half that).
#[test]
#[ignore = "exploratory stress test — run with --ignored"]
fn go2_wbc_velocity_staircase_fine_max_step_length() {
    let trials = [
        ("max_step_length_m=0.10 (current default, v_max=0.5)", 0.10),
        ("max_step_length_m=0.15 (v_max=0.75)", 0.15),
        ("max_step_length_m=0.20 (v_max=1.0)", 0.20),
    ];
    for (label, step_m) in trials {
        let cfg = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
        let params = WbcParams::velocity_staircase_fine_full_centroidal_max_step_length_misa_wbc(
            wbc::Formulation::ForceSpace,
            cfg,
            true,
            0.0,
            step_m,
        );
        let Some(samples) = run_wbc_sim(params) else { continue };
        eprintln!("\n=== {label} ===");
        report_velocity_staircase(&samples, 0.05, 1.0, 60.0);
    }
}

/// Sec.5ak follow-up: ① (true_centroidal_coupling) was characterized
/// as "neutral" in Sec.5ae, but that test never actually reached
/// speeds much above ~0.48 m/s (the old max_step_length_m=0.10
/// ceiling) -- so the swing-leg momentum reactive coupling ① models
/// was only ever exercised at fairly gentle swing speeds. Re-tests ①
/// on top of the new max_step_length_m=0.20 baseline (which genuinely
/// reaches ~0.85 m/s), where swing-leg momentum should matter more.
#[test]
#[ignore = "exploratory stress test — run with --ignored"]
fn go2_wbc_velocity_staircase_fine_max_step_length_true_coupling() {
    let trials = [
        ("max_step_length=0.20, no coupling (Sec.5ak baseline)", false),
        ("max_step_length=0.20, + true_centroidal_coupling", true),
    ];
    for (label, true_centroidal_coupling) in trials {
        let cfg = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
        let mut params = WbcParams::velocity_staircase_fine_full_centroidal_true_coupling_kcap_misa_wbc(
            wbc::Formulation::ForceSpace,
            cfg,
            true,
            true_centroidal_coupling,
            0.0,
        );
        params.max_step_length_override = Some(0.20);
        let Some(samples) = run_wbc_sim(params) else { continue };
        eprintln!("\n=== {label} ===");
        report_velocity_staircase(&samples, 0.05, 1.0, 60.0);
    }
}

/// Sec.5al's theory-vs-measured gap analysis: peak body roll roughly
/// doubles (0.06→0.10 rad) as `max_step_length_m` rises 0.10→0.20,
/// plausibly eating into the tracking budget available for forward
/// velocity. Tests whether raising `q_diag[9]`/`q_diag[10]` (roll/pitch
/// attitude weight, default 25/25) on top of the `max_step_length_m
/// =0.20` baseline reduces that disturbance and narrows the gap to
/// the theoretical ceiling (1.0 m/s).
#[test]
#[ignore = "exploratory stress test — run with --ignored"]
fn go2_wbc_velocity_staircase_fine_roll_pitch_weight() {
    let trials = [
        ("q_diag[9,10]=25/25 (current default)", 25.0, 25.0),
        ("q_diag[9,10]=50/50", 50.0, 50.0),
        ("q_diag[9,10]=100/100", 100.0, 100.0),
    ];
    for (label, q_roll, q_pitch) in trials {
        let cfg = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
        let params = WbcParams::velocity_staircase_fine_full_centroidal_roll_pitch_weight_misa_wbc(
            wbc::Formulation::ForceSpace,
            cfg,
            true,
            0.20,
            q_roll,
            q_pitch,
        );
        let Some(samples) = run_wbc_sim(params) else { continue };
        eprintln!("\n=== {label} ===");
        report_velocity_staircase(&samples, 0.05, 1.0, 60.0);
    }
}

/// Finds the *actual* ceiling of the current best configuration
/// (`legged_control_parity=true, k_capture=0, max_step_length_m=0.20`)
/// by widening the staircase past the 0-1.0 m/s range every prior
/// `max_step_length` test used — at cmd_vx=1.0 tracking (0.852) hadn't
/// clearly plateaued yet (theoretical ceiling for this step length is
/// 1.0 m/s), so the true saturation/reversal point, if any, is still
/// unknown. Mirrors Sec.5r's original coarse 0-5 m/s sweep that found
/// the old (0.10m) ceiling, but re-run against this session's current
/// best config.
#[test]
#[ignore = "exploratory stress test — run with --ignored"]
fn go2_wbc_velocity_staircase_coarse_max_step_length_ceiling() {
    let cfg = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
    let mut params = WbcParams::velocity_staircase_fine_full_centroidal_max_step_length_misa_wbc(
        wbc::Formulation::ForceSpace,
        cfg,
        true,
        0.0,
        0.20,
    );
    // Widen from the fine sweep's 0-1.0 m/s (0.05 steps) to 0-2.0 m/s
    // (0.10 steps) -- same 21 levels, same ~2.86s/level, same 60s
    // total, so the per-level dynamics stay comparable to every other
    // fine-staircase result this session.
    params.staircase_step_mps = 0.10;
    params.staircase_max_mps = 2.0;
    let n_levels = (params.staircase_max_mps / params.staircase_step_mps).round() as usize + 1;
    params.staircase_step_s = Some(params.total_time_s / n_levels as f64);
    let Some(samples) = run_wbc_sim(params) else { return };
    eprintln!("\n=== max_step_length=0.20 ceiling search, 0-2.0 m/s coarse ===");
    report_velocity_staircase(&samples, 0.10, 2.0, 60.0);
}

/// 2026-07-19 flight-phase validation (Canter/Gallop scoping): does
/// the SRBD/FullCentroidal MPC dynamics and WBC actually survive a
/// genuine aerial phase (0 legs in stance), or does the `n_stance=0`
/// code path that's only ever been reasoned about on paper (per the
/// quadruped-gait Explore survey) blow up in practice? Runs
/// `GaitType::Bound` at its stock `duty_factor=0.5` (baseline, no
/// gap — front/rear pairs tile exactly, matching every prior Bound
/// assumption) against `duty_factor=0.35` (30% of each cycle airborne,
/// twice per cycle, per `go2_diag_bound_duty_factor_flight_phase_
/// sweep`'s schedule-level confirmation). Single fixed speed
/// (cmd_vx=0.3), not a staircase — the question is survival, not a
/// tracking ceiling.
#[test]
#[ignore = "exploratory stress test — run with --ignored"]
fn go2_wbc_bound_flight_phase_duty_sweep() {
    let trials = [("duty=0.50 (no flight, baseline)", 0.5), ("duty=0.35 (30% flight/cycle)", 0.35)];
    for (label, duty) in trials {
        let cfg = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
        let params =
            WbcParams::bound_flight_phase_full_centroidal_misa_wbc(wbc::Formulation::ForceSpace, cfg, 0.3, duty);
        let Some(samples) = run_wbc_sim(params) else { continue };
        report_walk_summary(label, &samples, 0.3);
    }
}

/// Single-speed forward-walk summary (min_z / peak roll+pitch / net
/// Δx / measured vx / finiteness), same metrics `go2_wbc_bound_
/// flight_phase_duty_sweep` used, factored out so the baseline-
/// isolation survey below can reuse it across many trials.
fn report_walk_summary(label: &str, samples: &[WbcSample], cmd_vx: f64) {
    let burn_in_steps = (0.5 / 0.002_f64).round() as usize;
    let walk = &samples[burn_in_steps.min(samples.len())..];
    let min_z = samples.iter().map(|s| s.body_z).fold(f64::INFINITY, f64::min);
    let peak_roll = walk.iter().map(|s| s.roll.abs()).fold(0.0_f64, f64::max);
    let peak_pitch = walk.iter().map(|s| s.pitch.abs()).fold(0.0_f64, f64::max);
    let x0 = walk.first().map(|s| s.body_x).unwrap_or(0.0);
    let x1 = walk.last().map(|s| s.body_x).unwrap_or(0.0);
    let t0 = walk.first().map(|s| s.t).unwrap_or(0.0);
    let t1 = walk.last().map(|s| s.t).unwrap_or(0.0);
    let meas_vx = (x1 - x0) / (t1 - t0).max(1e-6);
    let has_nan = samples
        .iter()
        .any(|s| !s.body_x.is_finite() || !s.body_z.is_finite() || !s.roll.is_finite() || !s.pitch.is_finite());
    eprintln!(
        "\n=== {label} (cmd_vx={cmd_vx:.2}) ===\n\
         min_z={min_z:.3}m, peak_roll={peak_roll:.3}rad, peak_pitch={peak_pitch:.3}rad, \
         dx={:.3}m over {:.2}s (meas_vx≈{meas_vx:.3}), finite={}",
        x1 - x0, t1 - t0, !has_nan,
    );
}

/// Isolates *why* Bound reverses (§5ao): is it Bound itself, or the
/// Trot-tuned overrides (`legged_control_parity`, `k_capture=0`,
/// …) fighting Bound's very different footfall pattern? Sweeps the
/// same low speed (cmd_vx=0.15, `WbcParams::forward_walk`'s own
/// default) across four configurations of increasing sophistication,
/// each layered on `GaitConfig::bound()`'s own untouched defaults
/// (duty=0.5, cycle=0.3s, max_step_length_m=0.12 — no Trot-derived
/// overrides at all):
/// 1. `GaitMode::Mpc` (legacy 12-state SRBD) — the oldest, simplest
///    controller, never tuned for anything but Trot either, but at
///    least not carrying any of this session's Trot-specific
///    FullCentroidal changes.
/// 2. `GaitMode::FullCentroidal`, `legged_control_parity=false`
///    (D3.3.5a legacy contact schedule) — FullCentroidal's own
///    pre-parity behavior.
/// 3. `GaitMode::FullCentroidal` + `legged_control_parity=true`,
///    `k_capture` left at its default 0.05 (i.e. *before* the Sec.5ab
///    confounder fix — parity on, but not yet the Trot-specific
///    k_capture retune).
/// 4. `GaitMode::FullCentroidal` + `legged_control_parity=true` +
///    `k_capture=0` — the exact "healthy Trot baseline" combination
///    Sec.5ao's flight-phase check used, now at Bound's own native
///    speed/step-length instead of a 0.3 m/s command with Trot-scale
///    max_step_length_m.
#[test]
#[ignore = "exploratory stress test — run with --ignored"]
fn go2_wbc_bound_baseline_survey() {
    let cmd_vx = 0.15;

    let cfg1 = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
    let p1 = WbcParams {
        cmd_vx,
        gait_type_override: Some(GaitType::Bound),
        ..WbcParams::forward_walk_misa_wbc(wbc::Formulation::ForceSpace, cfg1)
    };
    if let Some(samples) = run_wbc_sim(p1) {
        report_walk_summary("1. GaitMode::Mpc (legacy SRBD), Bound defaults", &samples, cmd_vx);
    }

    let cfg2 = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
    let p2 = WbcParams {
        cmd_vx,
        gait_type_override: Some(GaitType::Bound),
        full_centroidal: Some(FullCentroidalOpts {
            legged_control_parity: false,
            use_mpc_predicted_footstep: false,
            dynamic_joint_q_reference: false,
            mpc_override: None,
            task_space_joint_vel_weight: None,
            true_centroidal_coupling: false,
            capture_point_gain_override: None,
            base_pos_xy_weight_override: None,
            max_normal_force_override: None,
            roll_pitch_weight_override: None,
        }),
        ..WbcParams::forward_walk_misa_wbc(wbc::Formulation::ForceSpace, cfg2)
    };
    if let Some(samples) = run_wbc_sim(p2) {
        report_walk_summary("2. FullCentroidal, parity=false (legacy), Bound defaults", &samples, cmd_vx);
    }

    let cfg3 = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
    let p3 = WbcParams {
        cmd_vx,
        gait_type_override: Some(GaitType::Bound),
        full_centroidal: Some(FullCentroidalOpts {
            legged_control_parity: true,
            use_mpc_predicted_footstep: false,
            dynamic_joint_q_reference: false,
            mpc_override: None,
            task_space_joint_vel_weight: None,
            true_centroidal_coupling: false,
            capture_point_gain_override: None, // default 0.05, NOT yet the Trot k_capture=0 fix
            base_pos_xy_weight_override: None,
            max_normal_force_override: None,
            roll_pitch_weight_override: None,
        }),
        ..WbcParams::forward_walk_misa_wbc(wbc::Formulation::ForceSpace, cfg3)
    };
    if let Some(samples) = run_wbc_sim(p3) {
        report_walk_summary("3. FullCentroidal + parity, k_capture=0.05 (default), Bound defaults", &samples, cmd_vx);
    }

    let cfg4 = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
    let p4 = WbcParams {
        cmd_vx,
        gait_type_override: Some(GaitType::Bound),
        full_centroidal: Some(FullCentroidalOpts {
            legged_control_parity: true,
            use_mpc_predicted_footstep: false,
            dynamic_joint_q_reference: false,
            mpc_override: None,
            task_space_joint_vel_weight: None,
            true_centroidal_coupling: false,
            capture_point_gain_override: Some(0.0),
            base_pos_xy_weight_override: None,
            max_normal_force_override: None,
            roll_pitch_weight_override: None,
        }),
        ..WbcParams::forward_walk_misa_wbc(wbc::Formulation::ForceSpace, cfg4)
    };
    if let Some(samples) = run_wbc_sim(p4) {
        report_walk_summary("4. FullCentroidal + parity + k_capture=0, Bound defaults", &samples, cmd_vx);
    }
}

/// Single-trial, video-friendly rerun of `go2_wbc_bound_baseline_
/// survey`'s config #4 (`FullCentroidal` + `legged_control_parity` +
/// `k_capture=0`, `GaitConfig::bound()`'s own untouched defaults) —
/// same configuration and speed (cmd_vx=0.15), just alone in the
/// process and run long enough (4.5s) to produce a legible `WBC_WALK_
/// CSV_OUT` trace for `render_go2_walk.py`. See Sec.5ap: this is the
/// case with the large sustained pitch oscillation (peak ~0.29 rad)
/// and net backward drift the survey found.
#[test]
#[ignore = "exploratory stress test — run with --ignored; also the WBC_WALK_CSV_OUT video-capture source for Sec.5ap"]
fn go2_wbc_bound_forward_walk_video_source() {
    let cfg = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
    let params = WbcParams {
        cmd_vx: 0.15,
        total_time_s: 4.5,
        burn_in_s: 0.5,
        gait_type_override: Some(GaitType::Bound),
        full_centroidal: Some(FullCentroidalOpts {
            legged_control_parity: true,
            use_mpc_predicted_footstep: false,
            dynamic_joint_q_reference: false,
            mpc_override: None,
            task_space_joint_vel_weight: None,
            true_centroidal_coupling: false,
            capture_point_gain_override: Some(0.0),
            base_pos_xy_weight_override: None,
            max_normal_force_override: None,
            roll_pitch_weight_override: None,
        }),
        ..WbcParams::forward_walk_misa_wbc(wbc::Formulation::ForceSpace, cfg)
    };
    let Some(samples) = run_wbc_sim(params) else { return };
    report_walk_summary("bound video-capture source (config #4, cmd_vx=0.15)", &samples, 0.15);
}

/// Sec.5ap follow-up: does softening `GaitConfig::bound()`'s own
/// timing/sizing (its `cycle_period_s=0.3s` is faster than Trot's
/// 0.4s, its `swing_height_m=0.05` is higher than Trot's 0.04) reduce
/// the large sustained pitch oscillation (0.27-0.34 rad) the baseline
/// survey found in every configuration, and does that in turn shrink
/// the forward-command reversal? All four trials keep the "healthy
/// Trot baseline" (`legged_control_parity=true, k_capture=0`) and
/// cmd_vx=0.15 fixed — only `cycle_period_s`/`swing_height_m` vary.
#[test]
#[ignore = "exploratory stress test — run with --ignored"]
fn go2_wbc_bound_gentler_parameters_sweep() {
    let trials = [
        ("A. Bound defaults (cycle=0.30s, swing=0.05m)", None, None),
        ("B. slower cycle (cycle=0.40s, swing=0.05m)", Some(0.40), None),
        ("C. lower swing (cycle=0.30s, swing=0.02m)", None, Some(0.02)),
        ("D. both (cycle=0.40s, swing=0.02m)", Some(0.40), Some(0.02)),
    ];
    for (label, cycle_period_s, swing_height_m) in trials {
        let cfg = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
        let params = WbcParams {
            cmd_vx: 0.15,
            gait_type_override: Some(GaitType::Bound),
            gait_cycle_period_override: cycle_period_s,
            swing_height_override: swing_height_m,
            full_centroidal: Some(FullCentroidalOpts {
                legged_control_parity: true,
                use_mpc_predicted_footstep: false,
                dynamic_joint_q_reference: false,
                mpc_override: None,
                task_space_joint_vel_weight: None,
                true_centroidal_coupling: false,
                capture_point_gain_override: Some(0.0),
                base_pos_xy_weight_override: None,
                max_normal_force_override: None,
                roll_pitch_weight_override: None,
            }),
            ..WbcParams::forward_walk_misa_wbc(wbc::Formulation::ForceSpace, cfg)
        };
        if let Some(samples) = run_wbc_sim(params) {
            report_walk_summary(label, &samples, 0.15);
        }
    }
}

/// Single-trial, video-friendly rerun of `go2_wbc_bound_gentler_
/// parameters_sweep`'s trial C (`swing_height_m=0.02`, `cycle_period_s`
/// left at Bound's own 0.30s default) — the config that cut the peak
/// pitch oscillation ~4x (0.291 -> 0.067 rad) and eliminated the
/// reversal entirely (meas_vx -0.124 -> +0.007) relative to Bound's
/// stock defaults. Same duration/speed as `go2_wbc_bound_forward_
/// walk_video_source` so the two videos are a direct before/after
/// comparison.
#[test]
#[ignore = "exploratory stress test — run with --ignored; also the WBC_WALK_CSV_OUT video-capture source for Sec.5aq"]
fn go2_wbc_bound_low_swing_video_source() {
    let cfg = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
    let params = WbcParams {
        cmd_vx: 0.15,
        total_time_s: 4.5,
        burn_in_s: 0.5,
        gait_type_override: Some(GaitType::Bound),
        swing_height_override: Some(0.02),
        full_centroidal: Some(FullCentroidalOpts {
            legged_control_parity: true,
            use_mpc_predicted_footstep: false,
            dynamic_joint_q_reference: false,
            mpc_override: None,
            task_space_joint_vel_weight: None,
            true_centroidal_coupling: false,
            capture_point_gain_override: Some(0.0),
            base_pos_xy_weight_override: None,
            max_normal_force_override: None,
            roll_pitch_weight_override: None,
        }),
        ..WbcParams::forward_walk_misa_wbc(wbc::Formulation::ForceSpace, cfg)
    };
    let Some(samples) = run_wbc_sim(params) else { return };
    report_walk_summary("bound low-swing video-capture source (swing_height_m=0.02, cmd_vx=0.15)", &samples, 0.15);
}

/// Sec.5aq found `swing_height_m=0.02` (down from Bound's 0.05
/// default) eliminates the reversal but leaves the robot barely
/// progressing (meas_vx≈0.007 at cmd_vx=0.15) — reversal-free but not
/// yet actually walking. Sweeps `max_step_length_m` (Bound's own
/// default 0.12m, vs. the values this session found useful for Trot:
/// 0.08/0.16/0.20) on top of the now-healthy `swing_height_m=0.02`
/// baseline, same cmd_vx=0.15, to see whether the footstep planner's
/// own stride-length clamp — not the swing dynamics — is now the
/// binding constraint on forward progress.
#[test]
#[ignore = "exploratory stress test — run with --ignored"]
fn go2_wbc_bound_low_swing_max_step_length_sweep() {
    let trials = [
        ("max_step_length_m=0.08", 0.08),
        ("max_step_length_m=0.12 (Bound default)", 0.12),
        ("max_step_length_m=0.16", 0.16),
        ("max_step_length_m=0.20", 0.20),
    ];
    for (label, max_step_length_m) in trials {
        let cfg = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
        let params = WbcParams {
            cmd_vx: 0.15,
            gait_type_override: Some(GaitType::Bound),
            swing_height_override: Some(0.02),
            max_step_length_override: Some(max_step_length_m),
            full_centroidal: Some(FullCentroidalOpts {
                legged_control_parity: true,
                use_mpc_predicted_footstep: false,
                dynamic_joint_q_reference: false,
                mpc_override: None,
                task_space_joint_vel_weight: None,
                true_centroidal_coupling: false,
                capture_point_gain_override: Some(0.0),
                base_pos_xy_weight_override: None,
                max_normal_force_override: None,
                roll_pitch_weight_override: None,
            }),
            ..WbcParams::forward_walk_misa_wbc(wbc::Formulation::ForceSpace, cfg)
        };
        if let Some(samples) = run_wbc_sim(params) {
            report_walk_summary(label, &samples, 0.15);
        }
    }
}

/// Sec.5aq/5ar follow-up: with `swing_height_m=0.02` fixed (the lever
/// that killed the reversal), does the robot's forward progress scale
/// with the commanded speed at all, or is it stuck near zero
/// regardless? Sweeps `cmd_vx` itself instead of `max_step_length_m` —
/// orthogonal axis, same healthy-baseline configuration.
#[test]
#[ignore = "exploratory stress test — run with --ignored"]
fn go2_wbc_bound_low_swing_cmd_vx_sweep() {
    for cmd_vx in [0.05, 0.10, 0.15, 0.20, 0.30] {
        let cfg = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
        let params = WbcParams {
            cmd_vx,
            gait_type_override: Some(GaitType::Bound),
            swing_height_override: Some(0.02),
            full_centroidal: Some(FullCentroidalOpts {
                legged_control_parity: true,
                use_mpc_predicted_footstep: false,
                dynamic_joint_q_reference: false,
                mpc_override: None,
                task_space_joint_vel_weight: None,
                true_centroidal_coupling: false,
                capture_point_gain_override: Some(0.0),
                base_pos_xy_weight_override: None,
                max_normal_force_override: None,
                roll_pitch_weight_override: None,
            }),
            ..WbcParams::forward_walk_misa_wbc(wbc::Formulation::ForceSpace, cfg)
        };
        if let Some(samples) = run_wbc_sim(params) {
            report_walk_summary(&format!("cmd_vx={cmd_vx:.2}"), &samples, cmd_vx);
        }
    }
}

/// Sec.5ar's dense `Σmpc_f_x`/`Σmpc_f_z` diagnostic found Bound's MPC
/// GRF solution repeatedly saturating at exactly `max_normal_force *
/// 2` (400N, i.e. both stance legs pinned at the 200N/leg default cap
/// — the same cap Sec.5ag found *never binds* for Trot) alongside
/// wild swings in `Σmpc_f_x` (-173 to +200N, vs Trot's tame -3.91 to
/// +4.66N). Hypothesis: 200N/leg, sized for Trot's diagonal-pair
/// support, may be too tight for Bound's front/rear-only support
/// pattern (which must carry the full body weight *and* counter a
/// much larger pitch moment using only 2 collinear legs). Tests
/// whether raising the cap resolves the force chaos and, in turn, the
/// reversal — at Bound's stock `swing_height_m=0.05` (the actual
/// reversal case, not the low-swing "shuffle" config), cmd_vx=0.15.
#[test]
#[ignore = "exploratory stress test — run with --ignored"]
fn go2_wbc_bound_max_normal_force_sweep() {
    for max_normal_force in [200.0, 400.0, 800.0, f64::INFINITY] {
        let cfg = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
        let params = WbcParams {
            cmd_vx: 0.15,
            gait_type_override: Some(GaitType::Bound),
            full_centroidal: Some(FullCentroidalOpts {
                legged_control_parity: true,
                use_mpc_predicted_footstep: false,
                dynamic_joint_q_reference: false,
                mpc_override: None,
                task_space_joint_vel_weight: None,
                true_centroidal_coupling: false,
                capture_point_gain_override: Some(0.0),
                base_pos_xy_weight_override: None,
                max_normal_force_override: Some(max_normal_force),
                roll_pitch_weight_override: None,
            }),
            ..WbcParams::forward_walk_misa_wbc(wbc::Formulation::ForceSpace, cfg)
        };
        if let Some(samples) = run_wbc_sim(params) {
            report_walk_summary(&format!("max_normal_force={max_normal_force:.0}"), &samples, 0.15);
        }
    }
}

/// Sec.5as's external Explore-agent audit of `misa-wbc`'s `ho_qp.rs`
/// derived a concrete mechanism: Bound's front-pair/rear-pair stance
/// shares the same body-frame `r_x` moment arm between its two
/// simultaneously-stance feet, so pitch torque (unlike Trot's, which
/// comes nearly free from `Δf_z · Δr_x` between a genuinely front and
/// rear foot) must come almost entirely from `Σf_x`, friction-limited
/// by `|f_x| ≤ μ·f_z`. This raises `friction_mu` (WBC's task AND the
/// FullCentroidal MPC's own friction cone, kept in sync — see
/// `WbcParams::friction_mu_override`'s doc comment) at Bound's stock
/// `swing_height_m=0.05` (the actual reversal case), cmd_vx=0.15, to
/// test whether relaxing the friction limit resolves the reversal.
/// 0.5 is the default; 0.7 matches the sim's actual MJCF ground
/// friction (a "free" real-world-consistent increase); 1.0/1.5 test
/// the hypothesis further (would need matching ground friction to be
/// physically deployable).
#[test]
#[ignore = "exploratory stress test — run with --ignored"]
fn go2_wbc_bound_friction_mu_sweep() {
    for friction_mu in [0.5, 0.7, 1.0, 1.5, 2.0, 3.0, 5.0] {
        let cfg = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
        let params = WbcParams {
            cmd_vx: 0.15,
            gait_type_override: Some(GaitType::Bound),
            friction_mu_override: Some(friction_mu),
            full_centroidal: Some(FullCentroidalOpts {
                legged_control_parity: true,
                use_mpc_predicted_footstep: false,
                dynamic_joint_q_reference: false,
                mpc_override: None,
                task_space_joint_vel_weight: None,
                true_centroidal_coupling: false,
                capture_point_gain_override: Some(0.0),
                base_pos_xy_weight_override: None,
                max_normal_force_override: None,
                roll_pitch_weight_override: None,
            }),
            ..WbcParams::forward_walk_misa_wbc(wbc::Formulation::ForceSpace, cfg)
        };
        if let Some(samples) = run_wbc_sim(params) {
            report_walk_summary(&format!("friction_mu={friction_mu:.2}"), &samples, 0.15);
        }
    }
}

/// Sec.5at's `friction_mu` sweep confirmed the "pitch authority is
/// friction-limited" mechanism (real, monotonic improvement 0.5-1.5)
/// but never reached genuine forward tracking alone. Real-world
/// model-based bounding controllers (Raibert's hopping-machine
/// three-part decomposition; MIT Cheetah 2/3) treat attitude/pitch
/// control as an independent channel — typically direct hip-joint
/// torque exploiting the leg's own mass/inertia during stance —
/// rather than relying solely on ground-reaction-force allocation
/// bounded by the friction cone. `true_centroidal_coupling` (desk-
/// research gap ①, Sec.5aa-5ae: an additive bias term from misarta's
/// CRBA-based centroidal momentum matrix, coupling joint acceleration
/// into the base's predicted motion) is architecturally exactly this
/// leg-mass-reaction-torque channel — it was found "neutral" for Trot
/// (Sec.5ae), plausibly because Trot's diagonal pair barely needs it
/// (pitch torque already comes cheaply from `Δf_z·Δr_x`). Bound is
/// exactly the regime where it should matter. Tests ① alone, and ①
/// combined with Sec.5at's best `friction_mu=1.5`, against the plain
/// reversal-case baseline, at Bound's stock `swing_height_m=0.05`,
/// cmd_vx=0.15.
#[test]
#[ignore = "exploratory stress test — run with --ignored"]
fn go2_wbc_bound_true_coupling_sweep() {
    let trials = [
        ("A. baseline (coupling off, mu=0.5)", false, 0.5),
        ("B. true_centroidal_coupling on, mu=0.5", true, 0.5),
        ("C. true_centroidal_coupling on, mu=1.5 (combined)", true, 1.5),
    ];
    for (label, coupling, friction_mu) in trials {
        let cfg = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
        let params = WbcParams {
            cmd_vx: 0.15,
            gait_type_override: Some(GaitType::Bound),
            friction_mu_override: Some(friction_mu),
            full_centroidal: Some(FullCentroidalOpts {
                legged_control_parity: true,
                use_mpc_predicted_footstep: false,
                dynamic_joint_q_reference: false,
                mpc_override: None,
                task_space_joint_vel_weight: None,
                true_centroidal_coupling: coupling,
                capture_point_gain_override: Some(0.0),
                base_pos_xy_weight_override: None,
                max_normal_force_override: None,
                roll_pitch_weight_override: None,
            }),
            ..WbcParams::forward_walk_misa_wbc(wbc::Formulation::ForceSpace, cfg)
        };
        if let Some(samples) = run_wbc_sim(params) {
            report_walk_summary(label, &samples, 0.15);
        }
    }
}

/// Sec.5au found `true_centroidal_coupling` (a passive linearization-
/// accuracy correction) doesn't deliver the independent pitch-
/// authority channel real bounding controllers (Raibert; MIT Cheetah
/// 2/3) rely on. This tests the more literal reproduction: an
/// explicit closed-loop pitch PD (`WbcPipeline::pitch_pd_gain`,
/// Sec.5av) added directly on top of the MPC-GRF-derived feedforward
/// that `a_base_des`'s angular component was previously pure
/// feedforward-only for. Same Bound reversal case as every other
/// sweep in this investigation: stock `swing_height_m=0.05`,
/// cmd_vx=0.15, `legged_control_parity=true, k_capture=0`.
#[test]
#[ignore = "exploratory stress test — run with --ignored"]
fn go2_wbc_bound_pitch_pd_sweep() {
    let trials = [
        ("A. baseline (pitch PD off)", (0.0, 0.0)),
        ("B. pitch_pd_gain=(50,5)", (50.0, 5.0)),
        ("C. pitch_pd_gain=(100,10)", (100.0, 10.0)),
        ("D. pitch_pd_gain=(200,20)", (200.0, 20.0)),
        ("E. pitch_pd_gain=(400,40)", (400.0, 40.0)),
    ];
    for (label, (kp, kd)) in trials {
        let cfg = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
        let params = WbcParams {
            cmd_vx: 0.15,
            gait_type_override: Some(GaitType::Bound),
            pitch_pd_gain_override: Some((kp, kd)),
            full_centroidal: Some(FullCentroidalOpts {
                legged_control_parity: true,
                use_mpc_predicted_footstep: false,
                dynamic_joint_q_reference: false,
                mpc_override: None,
                task_space_joint_vel_weight: None,
                true_centroidal_coupling: false,
                capture_point_gain_override: Some(0.0),
                base_pos_xy_weight_override: None,
                max_normal_force_override: None,
                roll_pitch_weight_override: None,
            }),
            ..WbcParams::forward_walk_misa_wbc(wbc::Formulation::ForceSpace, cfg)
        };
        if let Some(samples) = run_wbc_sim(params) {
            report_walk_summary(label, &samples, 0.15);
        }
    }
}

/// User follow-up to Sec.5aw's torque-saturation finding: does
/// genuinely relaxing the actuator envelope (not just the WBC's
/// internal belief) resolve the reversal? Scales every joint's
/// `effort` (N·m) by `1.0/2.0/5.0` before *both* the MJCF export
/// (MuJoCo's own `forcerange` clamp) and `WbcPipeline::new`'s
/// `torque_max` — a genuinely stronger simulated motor, not a solver-
/// internal relaxation like `friction_mu`/`pitch_pd_gain`. Same Bound
/// reversal case as every other sweep: stock `swing_height_m=0.05`,
/// cmd_vx=0.15, `legged_control_parity=true, k_capture=0`.
///
/// Note: joint *velocity* limits (`JointData::velocity`) were also
/// considered, but a code check found they aren't actually enforced
/// anywhere in the MJCF export / MuJoCo sim path today (no `.velocity`
/// reference in `src/mjcf.rs`) — relaxing that field would be a no-op
/// at this stage, so it's not included in this sweep.
#[test]
#[ignore = "exploratory stress test — run with --ignored"]
fn go2_wbc_bound_actuator_effort_scale_sweep() {
    for scale in [1.0, 2.0, 5.0] {
        let cfg = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
        let params = WbcParams {
            cmd_vx: 0.15,
            gait_type_override: Some(GaitType::Bound),
            actuator_effort_scale_override: Some(scale),
            full_centroidal: Some(FullCentroidalOpts {
                legged_control_parity: true,
                use_mpc_predicted_footstep: false,
                dynamic_joint_q_reference: false,
                mpc_override: None,
                task_space_joint_vel_weight: None,
                true_centroidal_coupling: false,
                capture_point_gain_override: Some(0.0),
                base_pos_xy_weight_override: None,
                max_normal_force_override: None,
                roll_pitch_weight_override: None,
            }),
            ..WbcParams::forward_walk_misa_wbc(wbc::Formulation::ForceSpace, cfg)
        };
        if let Some(samples) = run_wbc_sim(params) {
            report_walk_summary(&format!("effort_scale={scale:.1}"), &samples, 0.15);
        }
    }
}

/// User follow-up to Sec.5at: that sweep raised the WBC/MPC's
/// *belief* about friction (`friction_mu`) up to 5.0 while the real
/// MuJoCo ground stayed fixed at its 0.7 default — a mismatch where
/// the solver could ask for more grip than physically existed. This
/// tests the *matched* case: raise the real ground friction
/// (`ground_friction_override`) alongside `friction_mu` to the same
/// value, so the WBC's belief and the physical world agree, at
/// Sec.5at's best point (1.5) and beyond.
#[test]
#[ignore = "exploratory stress test — run with --ignored"]
fn go2_wbc_bound_matched_friction_sweep() {
    let trials = [
        ("A. mu=1.5, ground=0.7 (Sec.5at mismatched)", 1.5, 0.7),
        ("B. mu=1.5, ground=1.5 (matched)", 1.5, 1.5),
        ("C. mu=3.0, ground=3.0 (matched)", 3.0, 3.0),
    ];
    for (label, mu, ground_mu) in trials {
        let cfg = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
        let params = WbcParams {
            cmd_vx: 0.15,
            gait_type_override: Some(GaitType::Bound),
            friction_mu_override: Some(mu),
            ground_friction_override: Some(ground_mu),
            full_centroidal: Some(FullCentroidalOpts {
                legged_control_parity: true,
                use_mpc_predicted_footstep: false,
                dynamic_joint_q_reference: false,
                mpc_override: None,
                task_space_joint_vel_weight: None,
                true_centroidal_coupling: false,
                capture_point_gain_override: Some(0.0),
                base_pos_xy_weight_override: None,
                max_normal_force_override: None,
                roll_pitch_weight_override: None,
            }),
            ..WbcParams::forward_walk_misa_wbc(wbc::Formulation::ForceSpace, cfg)
        };
        if let Some(samples) = run_wbc_sim(params) {
            report_walk_summary(label, &samples, 0.15);
        }
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
