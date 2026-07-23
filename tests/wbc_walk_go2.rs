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

use articara::gait::{auto_detect_kinematics_config, auto_detect_srbd_mpc_config, GaitController, DEFAULT_FOOT_LINKS};
use articara::mjcf::{GroundPlaneCfg, MjcfExportOptions};
use articara::mujoco_sim::MujocoSim;
use articara::robot::RobotModel;
use articara::wbc_pipeline::WbcPipeline;
use nalgebra::Vector3;
use quadruped_gait::wbc;
use quadruped_gait::{
    foot_jacobian_body, solve_leg_ik, ContactDrivenPhase, GaitConfig, GaitMode, GaitType,
    KinematicsConfig, LegIkSolution, PhaseErrorTracker, VelocityCmd,
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

/// Diagnostic (no MuJoCo needed): prompted by the user asking whether
/// the required momentum/impulse budget for Bound has ever actually
/// been checked against Go2's real physical parameters. Answering that
/// properly requires knowing what mass/inertia the WBC *thinks* it's
/// working with — and this surfaced a real discrepancy: `WbcPipeline::
/// new`'s `mass_kg`/`inertia_diag_body` defaults (9.0 kg,
/// (0.07, 0.26, 0.242) — a leftover "Cheetah-class" placeholder, per
/// the field doc comment) are used to derive `a_base_des` (the WBC's
/// dominant weight-200 task) via Newton-Euler from the MPC's GRF, but
/// are **never overridden** anywhere in this test file — unlike the
/// MPC's own `auto_detect_srbd_mpc_config`, which correctly detects
/// Go2's real mass/inertia from the model. Prints both so the gap can
/// be quantified before deciding whether it's worth fixing.
#[test]
fn go2_diag_wbc_mass_inertia_mismatch() {
    let path = go2_misa();
    if !path.exists() {
        eprintln!("go2.misa missing at {} — skipping", path.display());
        return;
    }
    let robot = RobotModel::from_misa(&path).expect("load go2.misa");
    let real = auto_detect_srbd_mpc_config(&robot);
    eprintln!(
        "[mass-inertia] WbcPipeline::new() default: mass_kg=9.00, inertia_diag_body=(0.070, 0.260, 0.242)"
    );
    eprintln!(
        "[mass-inertia] auto_detect_srbd_mpc_config (real Go2): mass_kg={:.3}, inertia_diag_body=({:.4}, {:.4}, {:.4})",
        real.mass_kg, real.inertia_diag_body.x, real.inertia_diag_body.y, real.inertia_diag_body.z,
    );
    eprintln!(
        "[mass-inertia] mass ratio (real/placeholder) = {:.3}x, pitch-inertia ratio = {:.3}x",
        real.mass_kg / 9.0, real.inertia_diag_body.y / 0.26,
    );
}

/// Plan Phase 0 (2026-07-21): a closed-form, single-rigid-body (SRBD)
/// periodic-trim model for Bound's front-pair/rear-pair stance,
/// derived by exploiting front/rear mirror symmetry to solve only a
/// half-cycle boundary-value problem (see `ref/wbc_comparison.md`
/// Sec.5bb for the full derivation). With piecewise-constant GRF per
/// phase:
///   - height closure forces `F_z ≡ m·g` (flat trunk height — Bound
///     at duty_factor=0.5 has no aerial phase, so no bounce is
///     kinematically possible or required),
///   - the "trim" horizontal force that zeroes net pitch torque is
///     `F_x* = -r_x·m·g/h0` (pitch-torque relation `τ = -h0·F_x -
///     r_x·F_z`, since both feet of a stance pair share the same
///     body-frame `r_x` -- Sec.5at's geometric root cause),
///   - `θ_peak = |α_p|·T_st²/8`, `α_p = (-h0·F_x - r_x·m·g)/I_yy`,
///   - the friction-feasibility threshold is `μ_needed = |F_x*|/(m·g)
///     = r_x/h0`.
/// This test evaluates these formulas with Go2's real, auto-detected
/// mass/inertia/geometry (no hand-typed numbers) and the real leg
/// Jacobian (`foot_jacobian_body`, reused from Sec.5ai), as the
/// go/no-go feasibility gate before writing any pipeline code.
#[test]
fn go2_diag_bound_trim_model_feasibility() {
    let path = go2_misa();
    if !path.exists() {
        eprintln!("go2.misa missing at {} — skipping", path.display());
        return;
    }
    let robot = RobotModel::from_misa(&path).expect("load go2.misa");
    let srbd = auto_detect_srbd_mpc_config(&robot);
    let kin = auto_detect_kinematics_config(&robot, &DEFAULT_FOOT_LINKS).expect("auto-detect kinematics");

    let m = srbd.mass_kg;
    let i_yy = srbd.inertia_diag_body.y;
    let g = 9.81;
    let r_x_front = kin.fl.nominal_foot_body.x;
    let r_x_rear = -kin.rl.nominal_foot_body.x; // rear foot's own body-frame x is negative; use magnitude
    let h0 = -kin.fl.nominal_foot_body.z;
    let cycle_period_s = 0.30_f64;
    let duty_factor = 0.5_f64;
    let t_st = cycle_period_s * duty_factor;

    eprintln!(
        "[bound-trim] real params: m={m:.3}kg, I_yy={i_yy:.4}kg*m^2, r_x_front={r_x_front:.4}m, \
         r_x_rear={r_x_rear:.4}m, h0={h0:.4}m, T_st={t_st:.3}s"
    );

    let f_z_total = m * g;
    let f_x_trim = -r_x_front * m * g / h0;
    let mu_needed = f_x_trim.abs() / f_z_total;
    eprintln!(
        "[bound-trim] F_z (total, both stance legs) = {f_z_total:.2} N, F_x* (trim) = {f_x_trim:.2} N, \
         mu_needed = |F_x*|/(m*g) = {mu_needed:.3}"
    );

    let theta_peak = |f_x: f64| -> f64 {
        let alpha_p = (-h0 * f_x - r_x_front * f_z_total) / i_yy;
        (alpha_p.abs() * t_st * t_st) / 8.0
    };
    for (label, f_x) in [
        ("F_x=0 (no thrust)", 0.0),
        ("F_x=F_x* (full trim)", f_x_trim),
        ("F_x=clip(F_x*, mu=0.5)", f_x_trim.clamp(-0.5 * f_z_total, 0.5 * f_z_total)),
        ("F_x=clip(F_x*, mu=0.7, real ground)", f_x_trim.clamp(-0.7 * f_z_total, 0.7 * f_z_total)),
        ("F_x=clip(F_x*, mu=1.5)", f_x_trim.clamp(-1.5 * f_z_total, 1.5 * f_z_total)),
    ] {
        eprintln!(
            "[bound-trim] {label}: F_x={f_x:.2}N -> theta_peak={:.4} rad ({:.2} deg)",
            theta_peak(f_x), theta_peak(f_x).to_degrees(),
        );
    }

    // Required joint torque at nominal front-leg stance pose, for the
    // trim force split evenly across the 2 front-pair legs.
    let leg_kin = &kin.fl;
    let target = leg_kin.nominal_foot_body;
    let sol = solve_leg_ik(leg_kin, target, false);
    let LegIkSolution::Reached { hip, thigh, calf } = sol else {
        panic!("FL: nominal_foot_body unreachable");
    };
    let j = foot_jacobian_body(leg_kin, hip, thigh, calf);
    for (label, f_x_total) in [
        ("clip(mu=0.5)", f_x_trim.clamp(-0.5 * f_z_total, 0.5 * f_z_total)),
        ("clip(mu=0.7, real ground)", f_x_trim.clamp(-0.7 * f_z_total, 0.7 * f_z_total)),
        ("F_x* (unclipped)", f_x_trim),
    ] {
        let f_per_leg = Vector3::new(f_x_total / 2.0, 0.0, f_z_total / 2.0);
        let tau = -(j.transpose() * f_per_leg);
        let tau_max = tau.iter().cloned().fold(0.0_f64, |a, b: f64| a.max(b.abs()));
        eprintln!(
            "[bound-trim] required per-leg torque at {label}: tau=({:.2},{:.2},{:.2}) N*m, max|tau|={tau_max:.2} N*m \
             (real limits: hip/thigh=23.7, calf=45.43)",
            tau.x, tau.y, tau.z,
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
    body_y: f64,
    body_z: f64,
    roll: f64,
    pitch: f64,
    /// World-frame yaw (rad). Sec.5bo (local doc): a ramp-up + PLL run
    /// that looked like it was "walking backward" in `body_x` turned
    /// out to be an uncorrected ~180° yaw rotation instead (still
    /// walking FORWARD in the body frame, just facing the other way)
    /// -- tracked from here on so that class of misdiagnosis doesn't
    /// recur; `body_x`/`body_y` alone can't distinguish "reversed"
    /// from "turned around".
    yaw: f64,
    total_fz_world: f64,
    /// True if any leg's contact-driven-corrected `is_stance`
    /// (`ContactDrivenPhase::apply_correction`, from real measured
    /// GRF) disagreed with the nominal open-loop schedule's
    /// `is_stance` this tick -- i.e. an early-touchdown or
    /// late-liftoff event relative to the fixed-frequency phase
    /// clock. Diagnostic for the adaptive-frequency CPG investigation
    /// (Sec.5bk, local doc): if this correlates with the sparse
    /// cmd_vx "bad points" found in Sec.5bi, the phase clock running
    /// out of sync with the real contact timing is a plausible
    /// mechanism, and entraining the clock's own frequency to this
    /// signal (rather than fixing `cycle_period_s`) is worth pursuing.
    contact_phase_mismatch: bool,
    /// Sum of any [`PhaseErrorTracker::observe`] events (signed
    /// seconds) that fired this tick, across all 4 legs -- 0.0 on
    /// almost every tick (events are edge-triggered, one-shot per
    /// mismatch window). Negative = real gait running faster than
    /// `cycle_period_s` assumes, positive = slower. See `contact_
    /// phase_mismatch`'s doc comment and Sec.5bl (local doc).
    phase_error_sum_s: f64,
    /// The gait's `cycle_period_s` at this tick -- constant unless
    /// `WbcParams::adaptive_cycle_period` is enabled (Sec.5bl), in
    /// which case this traces the PLL's convergence.
    cycle_period_s: f64,
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

/// Sec.5bl (local doc): phase-locked-loop tuning for `WbcParams::
/// adaptive_cycle_period`. `gain` converts a mean signed phase error
/// (seconds) into a `cycle_period_s` nudge (seconds) -- e.g.
/// `gain=1.0` fully corrects the estimated error each update, `<1.0`
/// under-corrects (slower, more stable convergence).
#[derive(Clone, Copy, Debug)]
struct AdaptivePeriodConfig {
    gain: f64,
    update_interval_s: f64,
    min_period_s: f64,
    max_period_s: f64,
}

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
    /// Same idea as `pitch_pd_gain_override`, for `WbcPipeline::
    /// yaw_pd_gain` (Sec.5bp, local doc) -- the explicit yaw-holding
    /// feedback added after a `cmd_vx_ramp_s` startup was found to
    /// drift the body's heading 90-180 degrees uncorrected (Sec.5bo).
    /// `None` keeps the existing `(0.0, 0.0)` no-op default.
    yaw_pd_gain_override: Option<(f64, f64)>,
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
    /// Instead of stepping `cmd_vx` instantaneously at `burn_in_s`
    /// (every prior test in this file's behaviour), linearly ramp the
    /// commanded forward velocity from `0` to `cmd_vx` over this many
    /// seconds. Motivated by a user observation: real animals never
    /// enter a steady bounding gait from a dead stop — gait transitions
    /// go through a preparatory/transient phase (a crouch-and-launch
    /// "wind-up", anticipatory postural adjustments in the motor-
    /// control literature), never an instantaneous jump onto the
    /// steady-state limit cycle. Every Bound test so far commands the
    /// full step target from a static stance in one tick — precisely
    /// the "cold start" transition animals avoid — right as the WBC's
    /// `a_base_des` has to reconcile a large state mismatch, plausibly
    /// the worst moment for the HoQP's numerical conditioning
    /// (Sec.5at/5aw). `None` keeps the existing instantaneous-step
    /// behaviour.
    cmd_vx_ramp_s: Option<f64>,
    /// STEP-synchronized cmd_vx ramp (Sec.5d8, the MIT Cheetah 2 startup
    /// recipe). `Some(increment)` raises the commanded speed as a
    /// STAIRCASE, jumping by `increment` m/s once per gait step
    /// (half-cycle = the front/rear pair alternation period), capped at
    /// `cmd_vx`, instead of the continuous-time `cmd_vx_ramp_s`. Park/
    /// Wensing/Kim (IJRR 2017) start bounding with "v_d = 0 and the
    /// desired speed increased FROM STEP TO STEP" -- discrete, synced to
    /// contact events, not continuous in time. Our time-based ramp
    /// desynced from the gait phase (Sec.5c0-5c2); this fixes that. Takes
    /// priority over `cmd_vx_ramp_s`. `None` (default) keeps prior
    /// behaviour.
    cmd_vx_step_increment: Option<f64>,
    /// If `Some(start_m)` AND `cmd_vx_ramp_s` is also `Some(ramp_s)`,
    /// linearly ramps `max_step_length_m` from `start_m` up to its
    /// final (post-`max_step_length_override`) target value over the
    /// same `ramp_s` duration, in lockstep with the `cmd_vx` ramp --
    /// so stride grows smoothly with commanded speed instead of
    /// snapping to its full value at t=0 while the body is still near
    /// a dead stop (the startup transient Sec.5c0 found rough at
    /// duty<0.5). No-op if `cmd_vx_ramp_s` is `None`. `None` (default)
    /// preserves prior behaviour exactly.
    max_step_length_ramp_start_m: Option<f64>,
    /// Same idea as `max_step_length_ramp_start_m`, for `cycle_
    /// period_s`: ramps from `start_s` to the final (post-`gait_cycle_
    /// period_override`, or the gait's own default) target over the
    /// same `cmd_vx_ramp_s` duration. Real quadrupeds slow their
    /// stride frequency at low speed and quicken it as they accelerate
    /// (trot-to-gallop cadence increases with speed) -- this lets the
    /// startup transient mimic that instead of running at the
    /// steady-state cadence from a dead stop. No-op if `cmd_vx_ramp_s`
    /// is `None`. `None` (default) preserves prior behaviour exactly.
    cycle_period_ramp_start_s: Option<f64>,
    /// Same idea as `max_step_length_ramp_start_m`/`cycle_period_
    /// ramp_start_s`, for `bound_trim_thrust_scale` (Sec.5c1, local
    /// doc): ramps from `start_scale` up to its final (post-`bound_
    /// trim_thrust_scale_override`) target over the same `cmd_vx_
    /// ramp_s` duration. Motivated by Sec.5c0's diagnosis of why
    /// ramping `cmd_vx`/stride/period alone made the startup transient
    /// WORSE (one config fell over outright): the trim's periodic
    /// pitch/`F_x` schedule is sized by `thrust_scale` alone, entirely
    /// independent of `cmd_vx` -- so it fires at full strength from
    /// t=0 regardless of how slowly the commanded velocity is still
    /// ramping, creating exactly the mismatch those experiments hit.
    /// Ramping trim intensity itself in lockstep with `cmd_vx` is the
    /// one lever that was still untried. No-op if `cmd_vx_ramp_s` is
    /// `None`. `None` (default) preserves prior behaviour exactly.
    thrust_scale_ramp_start: Option<f64>,
    /// Extra seconds (beyond `cmd_vx_ramp_s` itself) to keep the PLL's
    /// phase-error accumulation gated off after the ramp otherwise
    /// ends, letting the gait settle before the PLL starts reacting to
    /// (possibly still-transient) contact timing. Sec.5c1 (local doc)
    /// found the PLL resuming exactly at the ramp/cruise boundary
    /// coincided with a new bout of instability -- this tests whether
    /// a settle buffer avoids that. No-op if `cmd_vx_ramp_s` is `None`.
    /// `None` (default, `0.0` effectively) preserves prior behaviour.
    post_ramp_settle_s: Option<f64>,
    /// When `true`, the PLL accumulates phase-error samples DURING the
    /// `cmd_vx_ramp_s` window too, instead of resetting to zero every
    /// tick (Sec.5bo's original reasoning for the reset: ramp-induced
    /// contact-timing shifts aren't real clock error and poison the
    /// very next post-ramp update). Sec.5c3 (local doc) found the
    /// t=6s+ collapse in the 4.0s-ramp best pattern persists unchanged
    /// even with a much tighter PLL clamp -- this tests whether
    /// letting the PLL "warm up" during the ramp (arriving at cruise
    /// with an already-primed `cycle_period_s` instead of a cold
    /// blank-slate accumulator) avoids the fresh-start windup instead.
    /// No-op if `cmd_vx_ramp_s` is `None`. `false` (default) preserves
    /// prior behaviour exactly.
    pll_accumulate_during_ramp: bool,
    /// Override `WbcPipeline::grf_smoothing_alpha` (default 1.0 = raw,
    /// no smoothing) and `WbcPipeline::qp_prox_weight` (default 1e-4)
    /// after construction. The `misa-wbc` `ho_qp.rs` audit (Sec.5at)
    /// flagged both as "directly available, code-supported levers
    /// worth testing before touching solver internals": Bound's
    /// upstream MPC GRF swings wildly tick-to-tick (Sec.5as), so the
    /// `contact_force` task's raw (unsmoothed) target, combined with a
    /// warm-start anchor (`qp_prox_weight`) pulling toward the
    /// *previous* tick's very different solution, could be feeding the
    /// HoQP a moving target *and* a stale, mismatched working-set seed
    /// every single tick — plausibly compounding the numerical
    /// ill-conditioning rather than causing it outright. `(alpha,
    /// prox_weight)`; `None` keeps the existing (1.0, 1e-4) defaults.
    grf_smoothing_and_prox_override: Option<(f64, f64)>,
    /// When `true`, syncs `WbcPipeline::{mass_kg, inertia_diag_body}`
    /// to `articara::gait::auto_detect_srbd_mpc_config`'s real,
    /// model-derived values instead of leaving them at `WbcPipeline::
    /// new`'s hardcoded "Cheetah-class" placeholder (`mass_kg=9.0`,
    /// `inertia_diag_body=(0.07, 0.26, 0.242)`). `go2_diag_wbc_mass_
    /// inertia_mismatch` found this placeholder is never overridden
    /// anywhere in this file, despite this file's own module doc
    /// claiming "real ~15.6 kg mass" throughout — real Go2 is 15.606
    /// kg (1.73x the placeholder) with pitch inertia 0.098 (0.38x the
    /// placeholder — the placeholder is ~2.65x too LARGE). Since these
    /// fields feed `a_base_des`'s Newton-Euler derivation (the WBC's
    /// dominant weight-200 task) — `a_lin = Σf/m + g`, `a_ang =
    /// I⁻¹·(Σr×f − ω×Iω)` — using a mass 42% too small and a pitch
    /// inertia 165% too large would make every `a_base_des` reference
    /// this session has ever computed for FullCentroidal+WBC wrong,
    /// for both Trot and Bound. `false` (default) preserves this
    /// file's existing behaviour exactly.
    sync_real_mass_inertia: bool,
    /// Enable Bound's closed-form periodic trim reference (Sec.5bb/
    /// 5bc, local doc): both the FullCentroidal MPC's own per-horizon-
    /// step pitch/fore-aft-GRF reference
    /// (`GaitController::set_bound_trim_reference`) and the WBC's
    /// explicit pitch-PD (`WbcPipeline::{pitch_pd_gain, pitch_ref}`)
    /// track the same time-varying trim pitch, computed fresh every
    /// tick from the real, auto-detected mass/inertia/geometry (never
    /// hand-typed). `Some((pitch_kp, pitch_kd))` sets the WBC-level PD
    /// gain; `None` keeps every gait's original flat/zero reference.
    /// Implicitly also enables `sync_real_mass_inertia` (the trim
    /// formulas are only correct with real physical parameters — see
    /// that field's own doc comment for the mass/inertia mismatch this
    /// depends on being fixed).
    bound_trim_reference: Option<(f64, f64)>,
    /// Override `BoundTrimConfig::thrust_scale` (see that field's doc
    /// comment, `quadruped-gait/src/bound_reference.rs`) when
    /// `bound_trim_reference` is `Some`. `None` uses the default
    /// `1.0` (full friction-clipped trim, prior behaviour). Values
    /// `<1.0` deliberately under-cancel pitch torque to free real
    /// friction-cone headroom for velocity tracking (Sec.5bf, local
    /// doc) — a no-op if `bound_trim_reference` is `None`.
    bound_trim_thrust_scale_override: Option<f64>,
    /// Override `BoundTrimConfig::velocity_ripple_fraction` (Sec.5bj,
    /// local doc) when `bound_trim_reference` is `Some`. `Some(fraction)`
    /// takes priority over `bound_trim_thrust_scale_override`, sizing
    /// `F_x` from a target velocity ripple (`fraction * cmd_vx`)
    /// instead -- the MIT-Cheetah-style "impulse scaling" alternative.
    /// `None` (default) uses the `thrust_scale`-based path.
    bound_trim_velocity_ripple_fraction_override: Option<f64>,
    /// Sec.5bl (local doc): if `Some`, nudge `cycle_period_s` every
    /// `update_interval_s` of sim time toward whatever period the
    /// `PhaseErrorTracker` signal implies (a phase-locked loop) --
    /// `new_period = clamp(period + gain*mean_signed_error_s, min, max)`.
    /// `None` (default) keeps `cycle_period_s` fixed at whatever
    /// `gait_cycle_period_override`/the gait preset set it to.
    adaptive_cycle_period: Option<AdaptivePeriodConfig>,
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
    /// Enable `GaitConfig::mpc_optimized_footstep` before build (Sec.5c8,
    /// local doc): the FullCentroidal MPC adds a quadratic foot-XY
    /// landing cost (`q_foot_xy_world`) so it actively chooses the
    /// footstep JOINTLY with the GRF, instead of tracking the open-loop
    /// Raibert target. This is option (B) from the Sec.5c5 review -- the
    /// principled alternative to the reactive foot-placement bolt-on
    /// that failed in Sec.5c6/5c7 (which conflicted with the directly-
    /// commanded F_x). Pair with `FullCentroidalOpts::
    /// use_mpc_predicted_footstep=true` so the controller actually uses
    /// the MPC's chosen foothold. `None` keeps the gait default (off).
    mpc_optimized_footstep_override: Option<bool>,
    /// Override `GaitConfig::q_foot_xy_world` (the foot-XY landing cost
    /// weight, default 500.0) before build. Sec.5c9 found the default
    /// 500 destabilizes the flight-phase Bound's pitch-critical GRF
    /// balance; Sec.5d0 sweeps it down to find a weight where the MPC
    /// gently shapes the footstep without wrecking pitch. `None` keeps
    /// the gait default. Only meaningful with
    /// `mpc_optimized_footstep_override: Some(true)`.
    q_foot_xy_world_override: Option<f64>,
    /// Set `GaitConfig::foot_xy_cost_body_frame` before build (Sec.5d2):
    /// the body-frame (base-relative) foot-XY cost variant that drops
    /// the base_pos term so the MPC places the foot by swing-leg motion
    /// alone, decoupled from the pitch-critical GRF (the fix designed in
    /// Sec.5d1 for why the world-frame cost destabilized the flight-
    /// phase Bound). `None` keeps the gait default (false = world-frame).
    /// Only meaningful with `mpc_optimized_footstep_override: Some(true)`.
    foot_xy_cost_body_frame_override: Option<bool>,
    /// Set `GaitConfig::bound_symmetric_foothold` before build (Sec.5d3):
    /// symmetrize the MPC-predicted swing footholds across each L/R pair
    /// (front FL/FR, rear RL/RR) so the aerial phase can't roll from
    /// asymmetric planting -- the Sec.5d2 faceplant's root cause. `None`
    /// keeps the gait default (false). Only meaningful with
    /// `mpc_optimized_footstep_override: Some(true)` +
    /// `use_mpc_predicted_footstep: true`.
    bound_symmetric_foothold_override: Option<bool>,
    /// Set `GaitConfig::bound_trim_vertical_reference` before build
    /// (Sec.5d4): feed the ballistic vertical bounce (F_z surplus +
    /// CoM vertical velocity) into the MPC reference. NOTE: the MIT
    /// literature verification (Sec.5d4) found MIT commands a FLAT
    /// reference and lets the bounce emerge, so this is AWAY from the
    /// MIT design -- kept as a measurable A/B. `None` = gait default
    /// (false = flat, MIT-aligned).
    bound_trim_vertical_reference_override: Option<bool>,
    /// Set `GaitConfig::bound_fx_thrust_bias` (N, total forward GRF on
    /// stance feet) before build (Sec.5d7): a constant forward thrust
    /// feedforward to raise the trimless MIT line's ~1.3 m/s ceiling.
    /// `None` = gait default (0.0 = no-op).
    bound_fx_thrust_bias_override: Option<f64>,
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
    /// Bound-specific fore-aft (x-only) foot-placement feedback gain
    /// (`GaitController::set_bound_fore_aft_placement_gain`, Sec.5c6):
    /// the Raibert running-speed regulator `x_foot += k·(v_x−v_x_des)`
    /// applied to the fore-aft foothold only. This is option (A) from
    /// the Sec.5c5 holistic review -- the literature-standard velocity
    /// stabilizer that the current architecture lacked (the generic
    /// `k_capture` was gated off at 0 for Bound because its x+y form
    /// rolled the body over, Sec.5bt). `None`/`0.0` (default) leaves
    /// the cmd-based Raibert `half` untouched.
    bound_fore_aft_placement_gain_override: Option<f64>,
    /// Sec.5f (energetic Bound, 1b): override `q_diag[3]` (base ROLL-RATE
    /// tracking weight, default 0.5). Sec.5f2 established that the death
    /// mode of an energetic (real-air-time) Bound is ROLL, not pitch: the
    /// long underactuated flight (all four feet airborne, zero GRF) lets a
    /// tiny roll rate integrate unchecked into a rollover (roll=pi) at
    /// t~2-3s. During STANCE, though, the front-pair / rear-pair are a
    /// Left-Right foot pair, so a differential vertical GRF *can* produce a
    /// roll moment -- the authority exists, the MPC just barely penalizes
    /// roll rate (0.5). Raising this weight turns that existing L/R GRF
    /// authority into an active roll-rate *deadbeat* reflex: each short
    /// stance drives the accumulated roll rate back toward zero before the
    /// next flight. `.0` sets `q_diag[3]` (roll rate), `.1` sets
    /// `q_diag[4]` (pitch rate) -- Sec.5f3 found the tumble is actually
    /// PITCH-led (peak_pitch reaches pi/2 at the rollover, a forward
    /// somersault), so pitch-rate deadbeat is the co-lever. `None` keeps
    /// the 0.5/0.5 defaults.
    roll_rate_weight_override: Option<(f64, f64)>,
    /// Sec.5f6: Poincaré/deadbeat pitch foot-placement gains
    /// `(k_angle, k_rate)` -- shift the touchdown fore-aft by the pitch
    /// error so the next stance's GRF moment NULLS (not just damps) the
    /// accumulated pitch momentum that Sec.5f5 showed the rate-deadbeat
    /// state weights only delay. `None` leaves the foothold untouched.
    bound_pitch_placement_gain_override: Option<(f64, f64)>,
    /// Sec.5f8 DC-blocker time constant (s) for the pitch foot-placement
    /// (`GaitController::set_bound_pitch_placement_dc_tau`). Removes the
    /// residual persistent forward foothold bias that drags the body
    /// backward, leaving only the deviation-stabilizing AC part. `None`
    /// keeps the raw (un-blocked) shift.
    bound_pitch_placement_dc_tau_override: Option<f64>,
    /// Sec.5f9 (P2): path to a CSV of the trajopt forward-Bound reference
    /// orbit (`phase,z,pitch,vx,vz,w` rows, header line skipped), produced
    /// by `ref/scripts/bound_trajopt_p0_shooting.py`. When set, the
    /// harness loads it and installs it via
    /// `GaitController::set_bound_tabulated_reference`, so the MPC tracks
    /// a CONSISTENT feasible forward orbit. `None` keeps the flat/trim ref.
    bound_tabulated_reference_csv: Option<&'static str>,
    /// P3-a: prescribed `(front, rear)` footholds from the trajopt orbit
    /// (`GaitController::set_bound_prescribed_footholds`). When set, the
    /// footstep planner follows the orbit's own footholds instead of
    /// Raibert+deadbeat. `None` keeps the normal footstep.
    bound_prescribed_footholds_override: Option<(f64, f64)>,
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
            swing_pd_gain_override: None, friction_mu_override: None, pitch_pd_gain_override: None, yaw_pd_gain_override: None, actuator_effort_scale_override: None, ground_friction_override: None, cmd_vx_ramp_s: None, cmd_vx_step_increment: None, max_step_length_ramp_start_m: None, cycle_period_ramp_start_s: None, thrust_scale_ramp_start: None, post_ramp_settle_s: None, pll_accumulate_during_ramp: false, grf_smoothing_and_prox_override: None, sync_real_mass_inertia: false, bound_trim_reference: None, bound_trim_thrust_scale_override: None, bound_trim_velocity_ripple_fraction_override: None, adaptive_cycle_period: None,
            gait_type_override: None, duty_factor_override: None, mpc_optimized_footstep_override: None, q_foot_xy_world_override: None, foot_xy_cost_body_frame_override: None, bound_symmetric_foothold_override: None, bound_trim_vertical_reference_override: None, bound_fx_thrust_bias_override: None,
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
            swing_pd_gain_override: None, friction_mu_override: None, pitch_pd_gain_override: None, yaw_pd_gain_override: None, actuator_effort_scale_override: None, ground_friction_override: None, cmd_vx_ramp_s: None, cmd_vx_step_increment: None, max_step_length_ramp_start_m: None, cycle_period_ramp_start_s: None, thrust_scale_ramp_start: None, post_ramp_settle_s: None, pll_accumulate_during_ramp: false, grf_smoothing_and_prox_override: None, sync_real_mass_inertia: false, bound_trim_reference: None, bound_trim_thrust_scale_override: None, bound_trim_velocity_ripple_fraction_override: None, adaptive_cycle_period: None,
            gait_type_override: None, duty_factor_override: None, mpc_optimized_footstep_override: None, q_foot_xy_world_override: None, foot_xy_cost_body_frame_override: None, bound_symmetric_foothold_override: None, bound_trim_vertical_reference_override: None, bound_fx_thrust_bias_override: None,
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
            swing_pd_gain_override: None, friction_mu_override: None, pitch_pd_gain_override: None, yaw_pd_gain_override: None, actuator_effort_scale_override: None, ground_friction_override: None, cmd_vx_ramp_s: None, cmd_vx_step_increment: None, max_step_length_ramp_start_m: None, cycle_period_ramp_start_s: None, thrust_scale_ramp_start: None, post_ramp_settle_s: None, pll_accumulate_during_ramp: false, grf_smoothing_and_prox_override: None, sync_real_mass_inertia: false, bound_trim_reference: None, bound_trim_thrust_scale_override: None, bound_trim_velocity_ripple_fraction_override: None, adaptive_cycle_period: None,
            gait_type_override: None,
            duty_factor_override: None, mpc_optimized_footstep_override: None, q_foot_xy_world_override: None, foot_xy_cost_body_frame_override: None, bound_symmetric_foothold_override: None, bound_trim_vertical_reference_override: None, bound_fx_thrust_bias_override: None,
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
                roll_pitch_weight_override: None, bound_fore_aft_placement_gain_override: None, roll_rate_weight_override: None, bound_pitch_placement_gain_override: None, bound_pitch_placement_dc_tau_override: None, bound_tabulated_reference_csv: None, bound_prescribed_footholds_override: None,
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
                roll_pitch_weight_override: None, bound_fore_aft_placement_gain_override: None, roll_rate_weight_override: None, bound_pitch_placement_gain_override: None, bound_pitch_placement_dc_tau_override: None, bound_tabulated_reference_csv: None, bound_prescribed_footholds_override: None,
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
    if let Some(enable) = params.mpc_optimized_footstep_override {
        eprintln!(
            "[gait] overriding mpc_optimized_footstep {} -> {enable} (q_foot_xy_world={:.1})",
            cfg.mpc_optimized_footstep, cfg.q_foot_xy_world,
        );
        cfg.mpc_optimized_footstep = enable;
    }
    if let Some(q) = params.q_foot_xy_world_override {
        eprintln!("[gait] overriding q_foot_xy_world {:.1} -> {q:.1}", cfg.q_foot_xy_world);
        cfg.q_foot_xy_world = q;
    }
    if let Some(body_frame) = params.foot_xy_cost_body_frame_override {
        eprintln!("[gait] overriding foot_xy_cost_body_frame -> {body_frame}");
        cfg.foot_xy_cost_body_frame = body_frame;
    }
    if let Some(sym) = params.bound_symmetric_foothold_override {
        eprintln!("[gait] overriding bound_symmetric_foothold -> {sym}");
        cfg.bound_symmetric_foothold = sym;
    }
    if let Some(vr) = params.bound_trim_vertical_reference_override {
        eprintln!("[gait] overriding bound_trim_vertical_reference -> {vr}");
        cfg.bound_trim_vertical_reference = vr;
    }
    if let Some(bias) = params.bound_fx_thrust_bias_override {
        eprintln!("[gait] overriding bound_fx_thrust_bias -> {bias:.1} N");
        cfg.bound_fx_thrust_bias = bias;
    }
    if let Some(swing_height_m) = params.swing_height_override {
        eprintln!(
            "[gait] overriding swing_height_m {:.3}m -> {:.3}m",
            cfg.swing_height_m, swing_height_m,
        );
        cfg.swing_height_m = swing_height_m;
    }
    // Captured before `cfg` moves into `GaitController::build` -- these
    // are the post-override steady-state targets the startup ramp (if
    // any) interpolates toward, per `max_step_length_ramp_start_m`/
    // `cycle_period_ramp_start_s`'s doc comments.
    let target_max_step_length_m = cfg.max_step_length_m;
    let target_cycle_period_s = cfg.cycle_period_s;
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
        if let Some((k_angle, k_rate)) = opts.bound_pitch_placement_gain_override {
            eprintln!(
                "[full-centroidal] bound_pitch_placement_gain (k_angle/k_rate) -> {:.3}/{:.3}",
                k_angle, k_rate,
            );
            gc.set_bound_pitch_placement_gain(k_angle, k_rate);
        }
        if let Some(tau) = opts.bound_pitch_placement_dc_tau_override {
            eprintln!("[full-centroidal] bound_pitch_placement_dc_tau -> {:.3}", tau);
            gc.set_bound_pitch_placement_dc_tau(tau);
        }
        if let Some(path) = opts.bound_tabulated_reference_csv {
            let text = std::fs::read_to_string(path)
                .unwrap_or_else(|e| panic!("read tabulated ref {path}: {e}"));
            let mut table: Vec<[f64; 6]> = Vec::new();
            for line in text.lines().skip(1) {
                let cols: Vec<f64> =
                    line.split(',').filter_map(|s| s.trim().parse().ok()).collect();
                if cols.len() == 6 {
                    table.push([cols[0], cols[1], cols[2], cols[3], cols[4], cols[5]]);
                }
            }
            eprintln!("[full-centroidal] tabulated reference: {} rows from {path}", table.len());
            gc.set_bound_tabulated_reference(Some(table));
        }
        if let Some((front, rear)) = opts.bound_prescribed_footholds_override {
            eprintln!("[full-centroidal] prescribed footholds front={front:.3} rear={rear:.3}");
            gc.set_bound_prescribed_footholds(Some((front, rear)));
        }
        if let Some(k) = opts.bound_fore_aft_placement_gain_override {
            eprintln!("[full-centroidal] bound_fore_aft_placement_gain -> {k:.3}");
            gc.set_bound_fore_aft_placement_gain(k);
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
        if let Some((q_roll_rate, q_pitch_rate)) = opts.roll_rate_weight_override {
            let mut mpc_cfg: FullCentroidalMpcConfig =
                gc.full_centroidal_mpc_config().expect("FullCentroidal mode has a config").clone();
            eprintln!(
                "[full-centroidal] q_diag[3]/[4] (roll-rate/pitch-rate) {:.1}/{:.1} -> {:.1}/{:.1}",
                mpc_cfg.q_diag[3], mpc_cfg.q_diag[4], q_roll_rate, q_pitch_rate,
            );
            mpc_cfg.q_diag[3] = q_roll_rate;
            mpc_cfg.q_diag[4] = q_pitch_rate;
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
    if params.sync_real_mass_inertia || params.bound_trim_reference.is_some() {
        let real = auto_detect_srbd_mpc_config(&robot);
        eprintln!(
            "[wbc] mass_kg {:.2} -> {:.2}, inertia_diag_body ({:.3},{:.3},{:.3}) -> ({:.3},{:.3},{:.3})",
            wbc_pipeline.mass_kg, real.mass_kg,
            wbc_pipeline.inertia_diag_body.x, wbc_pipeline.inertia_diag_body.y, wbc_pipeline.inertia_diag_body.z,
            real.inertia_diag_body.x, real.inertia_diag_body.y, real.inertia_diag_body.z,
        );
        wbc_pipeline.mass_kg = real.mass_kg;
        wbc_pipeline.inertia_diag_body = real.inertia_diag_body;
    }
    let bound_trim_thrust_scale = params.bound_trim_thrust_scale_override.unwrap_or(1.0);
    let bound_trim_velocity_ripple_fraction = params.bound_trim_velocity_ripple_fraction_override;
    let bound_trim_cfg: Option<quadruped_gait::BoundTrimConfig> =
        params.bound_trim_reference.map(|(kp, kd)| {
            gc.set_bound_trim_reference(true);
            gc.set_bound_trim_thrust_scale(bound_trim_thrust_scale);
            gc.set_bound_trim_velocity_ripple_fraction(bound_trim_velocity_ripple_fraction);
            wbc_pipeline.pitch_pd_gain = (kp, kd);
            let fl_kin = &kin.fl;
            let rl_kin = &kin.rl;
            let r_x_front = fl_kin.nominal_foot_body.x;
            let r_x_rear = -rl_kin.nominal_foot_body.x;
            let h0 = -fl_kin.nominal_foot_body.z;
            let trim_cfg = quadruped_gait::BoundTrimConfig {
                mass_kg: wbc_pipeline.mass_kg,
                inertia_yy: wbc_pipeline.inertia_diag_body.y,
                r_x: 0.5 * (r_x_front + r_x_rear),
                h0,
                cycle_period_s: gc.config().cycle_period_s,
                duty_factor: gc.config().duty_factor,
                friction_mu: wbc_pipeline.friction_mu,
                // Sec.5bc: empirically-corrected sign (see the same
                // constructor's comment in
                // full_centroidal_controller.rs) -- must match.
                sign: -1.0,
                thrust_scale: bound_trim_thrust_scale,
                cmd_vx_mps: params.cmd_vx,
                velocity_ripple_fraction: bound_trim_velocity_ripple_fraction,
            };
            eprintln!(
                "[bound-trim] enabled: F_x*={:.2}N, mu_needed={:.3}, thrust_scale={:.2}, ripple_fraction={:?}, F_x_used={:.2}N, theta_peak(used)={:.4}rad (pitch_pd_gain=({kp:.1},{kd:.1}))",
                trim_cfg.f_x_trim(), trim_cfg.mu_needed(), bound_trim_thrust_scale, bound_trim_velocity_ripple_fraction, trim_cfg.f_x_used(), trim_cfg.theta_peak(trim_cfg.f_x_used()),
            );
            trim_cfg
        });
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
    if let Some((kp, kd)) = params.yaw_pd_gain_override {
        eprintln!(
            "[wbc] yaw_pd_gain {:?} -> ({:.1}, {:.1})",
            wbc_pipeline.yaw_pd_gain, kp, kd,
        );
        wbc_pipeline.yaw_pd_gain = (kp, kd);
    }
    if let Some((alpha, prox_weight)) = params.grf_smoothing_and_prox_override {
        eprintln!(
            "[wbc] grf_smoothing_alpha {:.2} -> {:.2}, qp_prox_weight {:.1e} -> {:.1e}",
            wbc_pipeline.grf_smoothing_alpha, alpha, wbc_pipeline.qp_prox_weight, prox_weight,
        );
        wbc_pipeline.grf_smoothing_alpha = alpha;
        wbc_pipeline.qp_prox_weight = prox_weight;
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
    let mut phase_error_tracker = PhaseErrorTracker::new();
    let mut pll_error_sum = 0.0;
    let mut pll_error_count = 0u32;
    let mut pll_last_update_t = 0.0;

    let mut last_staircase_level: Option<usize> = None;
    for k in 0..n_steps {
        let t = k as f64 * params.dt;
        let mut contact_phase_mismatch = false;
        let mut phase_error_sum_s = 0.0;

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
        } else if let Some(increment) = params.cmd_vx_step_increment {
            // Sec.5d8 MIT Cheetah 2 startup: staircase cmd_vx, one jump
            // per gait step (half-cycle = pair alternation period),
            // synced to contact events rather than continuous time.
            if k >= burn_in_steps {
                let elapsed = (k - burn_in_steps) as f64 * params.dt;
                let step_period = 0.5 * gc.config().cycle_period_s;
                let n_steps = (elapsed / step_period.max(1e-6)).floor() + 1.0;
                let vx = (n_steps * increment).min(params.cmd_vx);
                gc.set_velocity_cmd(VelocityCmd { vx, vy: 0.0, wz: 0.0 });
                if k == burn_in_steps {
                    eprintln!(
                        "[step-ramp] MIT-style step ramp: +{increment:.3} m/s per {step_period:.3}s step, target {:.3}",
                        params.cmd_vx,
                    );
                }
            }
        } else if let Some(ramp_s) = params.cmd_vx_ramp_s {
            if k >= burn_in_steps {
                let elapsed = (k - burn_in_steps) as f64 * params.dt;
                let frac = (elapsed / ramp_s.max(1e-6)).min(1.0);
                let vx = frac * params.cmd_vx;
                gc.set_velocity_cmd(VelocityCmd { vx, vy: 0.0, wz: 0.0 });
                if let Some(start_m) = params.max_step_length_ramp_start_m {
                    gc.set_max_step_length_m(start_m + frac * (target_max_step_length_m - start_m));
                }
                if let Some(start_s) = params.cycle_period_ramp_start_s {
                    gc.set_cycle_period_s(start_s + frac * (target_cycle_period_s - start_s));
                }
                if let Some(start_scale) = params.thrust_scale_ramp_start {
                    gc.set_bound_trim_thrust_scale(start_scale + frac * (bound_trim_thrust_scale - start_scale));
                }
                if k == burn_in_steps {
                    eprintln!("[cmd-ramp] ramping cmd_vx 0 -> {:.3} over {:.2}s", params.cmd_vx, ramp_s);
                    if let Some(start_m) = params.max_step_length_ramp_start_m {
                        eprintln!(
                            "[cmd-ramp] ramping max_step_length_m {:.3} -> {:.3}m in lockstep",
                            start_m, target_max_step_length_m,
                        );
                    }
                    if let Some(start_s) = params.cycle_period_ramp_start_s {
                        eprintln!(
                            "[cmd-ramp] ramping cycle_period_s {:.3} -> {:.3}s in lockstep",
                            start_s, target_cycle_period_s,
                        );
                    }
                    if let Some(start_scale) = params.thrust_scale_ramp_start {
                        eprintln!(
                            "[cmd-ramp] ramping bound_trim_thrust_scale {:.3} -> {:.3} in lockstep",
                            start_scale, bound_trim_thrust_scale,
                        );
                    }
                }
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
        // Sec.5f6: feed observed roll/pitch for the Poincaré/deadbeat
        // pitch foot-placement (no-op unless a pitch-placement gain set).
        let (roll_obs, pitch_obs, _yaw_obs) = robot.base_transform.rotation.euler_angles();
        gc.set_body_attitude_observed(roll_obs, pitch_obs);

        if gc.is_enabled() {
            let (out, targets, torque_ff) = gc.tick(params.dt);
            for (idx, q) in targets {
                sim.set_position_target(idx, q);
            }
            if k >= burn_in_steps {
                // Sec.5bp: yaw-holding target is gait-independent (unlike
                // pitch_ref, which only Bound's trim reference sets) --
                // `world_yaw()` is `cmd.wz` integrated open-loop, so this
                // is `0.0` (hold straight) whenever wz=0, and tracks an
                // actual turn command otherwise.
                wbc_pipeline.yaw_ref = gc.world_yaw();
                if let Some(trim_cfg) = &bound_trim_cfg {
                    let cycle_phase = out.legs[0].phase.cycle_position;
                    wbc_pipeline.pitch_ref = trim_cfg.sample(cycle_phase).pitch;
                }
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
                contact_phase_mismatch =
                    (0..4).any(|slot| nominal_phases[slot].is_stance != corrected[slot].is_stance);
                let phase_errors = phase_error_tracker.observe(
                    &nominal_phases,
                    force_z,
                    5.0,
                    0.0,
                    gc.config().cycle_period_s,
                    gc.config().duty_factor,
                );
                phase_error_sum_s = phase_errors.iter().filter_map(|e| *e).sum();
                // While cmd_vx is still ramping, the real contact timing is
                // shifting for a reason that has nothing to do with the
                // clock being wrong -- accumulating those samples poisons
                // the very next post-ramp update with stale, transient-
                // laden error (found empirically: it pushed cycle_period_s
                // the wrong way right at the ramp/cruise boundary and sent
                // the robot walking backward for the rest of the run, see
                // wbc_comparison.md Sec.5bo). Skip accumulation during the
                // ramp and restart the window clean the instant it ends.
                let ramp_in_progress = params.cmd_vx_ramp_s.is_some_and(|ramp_s| {
                    t < params.burn_in_s + ramp_s + params.post_ramp_settle_s.unwrap_or(0.0)
                });
                if ramp_in_progress && !params.pll_accumulate_during_ramp {
                    pll_error_sum = 0.0;
                    pll_error_count = 0;
                    pll_last_update_t = t;
                } else if let Some(pll) = &params.adaptive_cycle_period {
                    for e in phase_errors.iter().filter_map(|e| *e) {
                        pll_error_sum += e;
                        pll_error_count += 1;
                    }
                    if t - pll_last_update_t >= pll.update_interval_s && pll_error_count > 0 {
                        let mean_error = pll_error_sum / pll_error_count as f64;
                        let current = gc.config().cycle_period_s;
                        let new_period =
                            (current + pll.gain * mean_error).clamp(pll.min_period_s, pll.max_period_s);
                        eprintln!(
                            "[pll] t={t:.2}s mean_error={:.4}s ({} samples) cycle_period_s {current:.4} -> {new_period:.4}",
                            mean_error, pll_error_count,
                        );
                        gc.set_cycle_period_s(new_period);
                        pll_error_sum = 0.0;
                        pll_error_count = 0;
                        pll_last_update_t = t;
                    }
                }
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
                    let (_r, pitch_now, _y) = robot.base_transform.rotation.euler_angles();
                    eprintln!(
                        "[diag k={k:5} t={:.3}s] z={:.3} m  Σmpc_f_z={:.2} N  Σmpc_f_x={:.2} N  max|τ|={:.2} N·m  stance={}/4  \
                         pitch_ref={:.4} pitch_meas={:.4}",
                        k as f64 * params.dt, body_pos[2], mpc_fz_sum, mpc_fx_sum, tau_max, stance_count,
                        wbc_pipeline.pitch_ref, pitch_now,
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
        let (roll, pitch, yaw) = robot.base_transform.rotation.euler_angles();
        let total_fz_world: f64 = sim.contacts().iter().map(|c| c.force_world[2]).sum();
        samples.push(WbcSample {
            t, body_x: tx.x, body_y: tx.y, body_z: tx.z, roll, pitch, yaw, total_fz_world,
            contact_phase_mismatch, phase_error_sum_s,
            cycle_period_s: gc.config().cycle_period_s,
        });

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
    let y0 = walk.first().map(|s| s.body_y).unwrap_or(0.0);
    let y1 = walk.last().map(|s| s.body_y).unwrap_or(0.0);
    let yaw0 = walk.first().map(|s| s.yaw).unwrap_or(0.0);
    let yaw1 = walk.last().map(|s| s.yaw).unwrap_or(0.0);
    let t0 = walk.first().map(|s| s.t).unwrap_or(0.0);
    let t1 = walk.last().map(|s| s.t).unwrap_or(0.0);
    let meas_vx = (x1 - x0) / (t1 - t0).max(1e-6);
    // Sec.5bo: world-x displacement alone can't tell "walked backward"
    // apart from "turned around and kept walking forward" -- planar
    // speed (direction-agnostic) and yaw drift disambiguate the two.
    let planar_speed = ((x1 - x0).powi(2) + (y1 - y0).powi(2)).sqrt() / (t1 - t0).max(1e-6);
    let yaw_drift_deg = (yaw1 - yaw0).to_degrees();
    let has_nan = samples
        .iter()
        .any(|s| !s.body_x.is_finite() || !s.body_z.is_finite() || !s.roll.is_finite() || !s.pitch.is_finite());
    let mismatch_frac = if walk.is_empty() {
        0.0
    } else {
        walk.iter().filter(|s| s.contact_phase_mismatch).count() as f64 / walk.len() as f64
    };
    let phase_errors: Vec<f64> = walk.iter().map(|s| s.phase_error_sum_s).filter(|e| *e != 0.0).collect();
    let n_fast = phase_errors.iter().filter(|e| **e < 0.0).count();
    let n_slow = phase_errors.iter().filter(|e| **e > 0.0).count();
    let mean_signed_err_ms = if phase_errors.is_empty() {
        0.0
    } else {
        1000.0 * phase_errors.iter().sum::<f64>() / phase_errors.len() as f64
    };
    let final_period_s = walk.last().map(|s| s.cycle_period_s).unwrap_or(0.0);
    eprintln!(
        "\n=== {label} (cmd_vx={cmd_vx:.2}) ===\n\
         min_z={min_z:.3}m, peak_roll={peak_roll:.3}rad, peak_pitch={peak_pitch:.3}rad, \
         dx={:.3}m over {:.2}s (meas_vx≈{meas_vx:.3}), finite={}, contact_phase_mismatch={:.1}%, \
         phase_err: n_fast={n_fast} n_slow={n_slow} mean_signed={mean_signed_err_ms:.2}ms, \
         final_cycle_period_s={final_period_s:.4}, planar_speed={planar_speed:.3}m/s, yaw_drift={yaw_drift_deg:.1}deg",
        x1 - x0, t1 - t0, !has_nan, mismatch_frac * 100.0,
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
            roll_pitch_weight_override: None, bound_fore_aft_placement_gain_override: None, roll_rate_weight_override: None, bound_pitch_placement_gain_override: None, bound_pitch_placement_dc_tau_override: None, bound_tabulated_reference_csv: None, bound_prescribed_footholds_override: None,
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
            roll_pitch_weight_override: None, bound_fore_aft_placement_gain_override: None, roll_rate_weight_override: None, bound_pitch_placement_gain_override: None, bound_pitch_placement_dc_tau_override: None, bound_tabulated_reference_csv: None, bound_prescribed_footholds_override: None,
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
            roll_pitch_weight_override: None, bound_fore_aft_placement_gain_override: None, roll_rate_weight_override: None, bound_pitch_placement_gain_override: None, bound_pitch_placement_dc_tau_override: None, bound_tabulated_reference_csv: None, bound_prescribed_footholds_override: None,
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
            roll_pitch_weight_override: None, bound_fore_aft_placement_gain_override: None, roll_rate_weight_override: None, bound_pitch_placement_gain_override: None, bound_pitch_placement_dc_tau_override: None, bound_tabulated_reference_csv: None, bound_prescribed_footholds_override: None,
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
                roll_pitch_weight_override: None, bound_fore_aft_placement_gain_override: None, roll_rate_weight_override: None, bound_pitch_placement_gain_override: None, bound_pitch_placement_dc_tau_override: None, bound_tabulated_reference_csv: None, bound_prescribed_footholds_override: None,
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
            roll_pitch_weight_override: None, bound_fore_aft_placement_gain_override: None, roll_rate_weight_override: None, bound_pitch_placement_gain_override: None, bound_pitch_placement_dc_tau_override: None, bound_tabulated_reference_csv: None, bound_prescribed_footholds_override: None,
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
                roll_pitch_weight_override: None, bound_fore_aft_placement_gain_override: None, roll_rate_weight_override: None, bound_pitch_placement_gain_override: None, bound_pitch_placement_dc_tau_override: None, bound_tabulated_reference_csv: None, bound_prescribed_footholds_override: None,
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
                roll_pitch_weight_override: None, bound_fore_aft_placement_gain_override: None, roll_rate_weight_override: None, bound_pitch_placement_gain_override: None, bound_pitch_placement_dc_tau_override: None, bound_tabulated_reference_csv: None, bound_prescribed_footholds_override: None,
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
                roll_pitch_weight_override: None, bound_fore_aft_placement_gain_override: None, roll_rate_weight_override: None, bound_pitch_placement_gain_override: None, bound_pitch_placement_dc_tau_override: None, bound_tabulated_reference_csv: None, bound_prescribed_footholds_override: None,
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
                roll_pitch_weight_override: None, bound_fore_aft_placement_gain_override: None, roll_rate_weight_override: None, bound_pitch_placement_gain_override: None, bound_pitch_placement_dc_tau_override: None, bound_tabulated_reference_csv: None, bound_prescribed_footholds_override: None,
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
                roll_pitch_weight_override: None, bound_fore_aft_placement_gain_override: None, roll_rate_weight_override: None, bound_pitch_placement_gain_override: None, bound_pitch_placement_dc_tau_override: None, bound_tabulated_reference_csv: None, bound_prescribed_footholds_override: None,
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
                roll_pitch_weight_override: None, bound_fore_aft_placement_gain_override: None, roll_rate_weight_override: None, bound_pitch_placement_gain_override: None, bound_pitch_placement_dc_tau_override: None, bound_tabulated_reference_csv: None, bound_prescribed_footholds_override: None,
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
                roll_pitch_weight_override: None, bound_fore_aft_placement_gain_override: None, roll_rate_weight_override: None, bound_pitch_placement_gain_override: None, bound_pitch_placement_dc_tau_override: None, bound_tabulated_reference_csv: None, bound_prescribed_footholds_override: None,
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
                roll_pitch_weight_override: None, bound_fore_aft_placement_gain_override: None, roll_rate_weight_override: None, bound_pitch_placement_gain_override: None, bound_pitch_placement_dc_tau_override: None, bound_tabulated_reference_csv: None, bound_prescribed_footholds_override: None,
            }),
            ..WbcParams::forward_walk_misa_wbc(wbc::Formulation::ForceSpace, cfg)
        };
        if let Some(samples) = run_wbc_sim(params) {
            report_walk_summary(label, &samples, 0.15);
        }
    }
}

/// User observation: real animals never enter a steady bounding gait
/// from a dead stop — gait transitions go through a preparatory/
/// transient phase (crouch-and-launch "wind-up", anticipatory
/// postural adjustments), never an instantaneous jump onto the
/// steady-state limit cycle. Every Bound test so far commands the
/// full `cmd_vx` step from a static stance in a single tick — exactly
/// the "cold start" animals avoid — right when `a_base_des` has to
/// reconcile the largest state mismatch, plausibly the worst moment
/// for the HoQP's numerical conditioning (Sec.5at/5aw/5ax all point
/// to Bound's collinear-stance QP ill-conditioning as the actual root
/// cause). Tests whether ramping `cmd_vx` in linearly (instead of
/// stepping it) avoids triggering the worst of the transient and
/// yields a better steady-state outcome, at Bound's stock reversal-
/// case config (`swing_height_m=0.05`, `legged_control_parity=true,
/// k_capture=0`), target cmd_vx=0.15.
///
/// Uses a bespoke (not `report_walk_summary`) measurement window —
/// `[burn_in_s + ramp_s, total_time_s]`, i.e. *after* the ramp
/// completes — so different ramp durations are compared on their
/// actual steady-state behaviour, not diluted by the ramp's own
/// necessarily-slower average speed.
#[test]
#[ignore = "exploratory stress test — run with --ignored"]
fn go2_wbc_bound_cmd_vx_ramp_sweep() {
    let burn_in_s = 0.5;
    let total_time_s = 5.0;
    for ramp_s in [0.0, 0.5, 1.0, 2.0] {
        let cfg = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
        let params = WbcParams {
            cmd_vx: 0.15,
            total_time_s,
            burn_in_s,
            cmd_vx_ramp_s: if ramp_s > 0.0 { Some(ramp_s) } else { None },
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
                roll_pitch_weight_override: None, bound_fore_aft_placement_gain_override: None, roll_rate_weight_override: None, bound_pitch_placement_gain_override: None, bound_pitch_placement_dc_tau_override: None, bound_tabulated_reference_csv: None, bound_prescribed_footholds_override: None,
            }),
            ..WbcParams::forward_walk_misa_wbc(wbc::Formulation::ForceSpace, cfg)
        };
        let Some(samples) = run_wbc_sim(params) else { continue };
        let t0 = burn_in_s + ramp_s;
        let steady: Vec<&WbcSample> = samples.iter().filter(|s| s.t >= t0).collect();
        let min_z = samples.iter().map(|s| s.body_z).fold(f64::INFINITY, f64::min);
        let peak_pitch = steady.iter().map(|s| s.pitch.abs()).fold(0.0_f64, f64::max);
        let x0 = steady.first().map(|s| s.body_x).unwrap_or(0.0);
        let x1 = steady.last().map(|s| s.body_x).unwrap_or(0.0);
        let ts0 = steady.first().map(|s| s.t).unwrap_or(0.0);
        let ts1 = steady.last().map(|s| s.t).unwrap_or(0.0);
        let meas_vx = (x1 - x0) / (ts1 - ts0).max(1e-6);
        eprintln!(
            "\n=== ramp_s={ramp_s:.1} (steady window t=[{t0:.2}, {total_time_s:.2}]) ===\n\
             min_z={min_z:.3}m, peak_pitch(steady)={peak_pitch:.3}rad, \
             steady_meas_vx≈{meas_vx:.3} (dx={:.3}m over {:.2}s)",
            x1 - x0, ts1 - ts0,
        );
    }
}

/// Sec.5ay ruled out transient onset; the root cause remains Bound's
/// collinear-stance QP ill-conditioning (Sec.5at/5aw/5ax). Before
/// touching `misa-wbc`'s `ho_qp.rs` internals, this tests two already-
/// implemented, zero-new-code levers the external audit flagged as
/// worth trying first: `grf_smoothing_alpha` (EMA-smooths the
/// `contact_force` task's target instead of feeding it Bound's raw,
/// wildly-swinging MPC GRF) and `qp_prox_weight` (0.0 disables the
/// warm-start anchor entirely — a cold solve every tick, avoiding a
/// stale working-set seed from the *previous* tick's very different
/// solution). Same Bound reversal case as every other sweep in this
/// investigation: stock `swing_height_m=0.05`, cmd_vx=0.15.
#[test]
#[ignore = "exploratory stress test — run with --ignored"]
fn go2_wbc_bound_grf_smoothing_and_prox_sweep() {
    let trials = [
        ("A. baseline (alpha=1.0, prox=1e-4)", 1.0, 1e-4),
        ("B. smoothed GRF (alpha=0.3, prox=1e-4)", 0.3, 1e-4),
        ("C. cold solve (alpha=1.0, prox=0.0)", 1.0, 0.0),
        ("D. combined (alpha=0.3, prox=0.0)", 0.3, 0.0),
    ];
    for (label, alpha, prox_weight) in trials {
        let cfg = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
        let params = WbcParams {
            cmd_vx: 0.15,
            gait_type_override: Some(GaitType::Bound),
            grf_smoothing_and_prox_override: Some((alpha, prox_weight)),
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
                roll_pitch_weight_override: None, bound_fore_aft_placement_gain_override: None, roll_rate_weight_override: None, bound_pitch_placement_gain_override: None, bound_pitch_placement_dc_tau_override: None, bound_tabulated_reference_csv: None, bound_prescribed_footholds_override: None,
            }),
            ..WbcParams::forward_walk_misa_wbc(wbc::Formulation::ForceSpace, cfg)
        };
        if let Some(samples) = run_wbc_sim(params) {
            report_walk_summary(label, &samples, 0.15);
        }
    }
}

/// `go2_diag_wbc_mass_inertia_mismatch` found `WbcPipeline`'s
/// `mass_kg`/`inertia_diag_body` (feeding `a_base_des`'s dominant
/// weight-200 Newton-Euler reference) are never synced to Go2's real,
/// auto-detected values anywhere in this file — stuck at a "Cheetah-
/// class" placeholder that's 42% too light and has pitch inertia 165%
/// too large. Tests whether correcting this (a plausible latent bug
/// affecting every FullCentroidal+WBC test this session, not a Bound-
/// specific tuning knob) changes Bound's reversal case, and confirms
/// it doesn't regress Trot.
#[test]
#[ignore = "exploratory stress test — run with --ignored"]
fn go2_wbc_mass_inertia_fix_sweep() {
    // Bound reversal case, with vs without the fix.
    for (label, sync) in [("A. Bound, placeholder mass/inertia (baseline)", false), ("B. Bound, real mass/inertia", true)] {
        let cfg = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
        let params = WbcParams {
            cmd_vx: 0.15,
            gait_type_override: Some(GaitType::Bound),
            sync_real_mass_inertia: sync,
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
                roll_pitch_weight_override: None, bound_fore_aft_placement_gain_override: None, roll_rate_weight_override: None, bound_pitch_placement_gain_override: None, bound_pitch_placement_dc_tau_override: None, bound_tabulated_reference_csv: None, bound_prescribed_footholds_override: None,
            }),
            ..WbcParams::forward_walk_misa_wbc(wbc::Formulation::ForceSpace, cfg)
        };
        if let Some(samples) = run_wbc_sim(params) {
            report_walk_summary(label, &samples, 0.15);
        }
    }
    // Trot sanity check: same fix, Trot's own healthy-baseline config
    // (legged_control_parity + k_capture=0), to confirm no regression.
    for (label, sync) in [("C. Trot, placeholder mass/inertia (baseline)", false), ("D. Trot, real mass/inertia", true)] {
        let cfg = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
        let params = WbcParams {
            cmd_vx: 0.15,
            sync_real_mass_inertia: sync,
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
                roll_pitch_weight_override: None, bound_fore_aft_placement_gain_override: None, roll_rate_weight_override: None, bound_pitch_placement_gain_override: None, bound_pitch_placement_dc_tau_override: None, bound_tabulated_reference_csv: None, bound_prescribed_footholds_override: None,
            }),
            ..WbcParams::forward_walk_misa_wbc(wbc::Formulation::ForceSpace, cfg)
        };
        if let Some(samples) = run_wbc_sim(params) {
            report_walk_summary(label, &samples, 0.15);
        }
    }
}

/// Phase c (plan `splendid-chasing-lollipop.md`): validates the
/// closed-form Bound trim reference (Sec.5bb/5bc) end-to-end in
/// MuJoCo. Both the FullCentroidal MPC's own per-step reference
/// (`grfs[leg].x`, `base_euler_zyx.y`) and the WBC's explicit
/// pitch-PD now track the same time-varying periodic pitch/thrust
/// profile, instead of the flat zero-pitch/zero-Fx hold Bound (and
/// every gait) used before. Same Bound reversal case as every prior
/// sweep in this investigation: stock `swing_height_m=0.05`,
/// `legged_control_parity=true, k_capture=0`, cmd_vx=0.15. Success
/// criteria (Sec.5bb's predictions): `meas_vx` should cross from
/// negative toward +0.15, and `peak_pitch` should drop from the
/// current chaotic ~0.29 rad toward the theoretical ~0.025 rad
/// (mu=0.7-clipped trim).
#[test]
#[ignore = "exploratory stress test — run with --ignored"]
fn go2_wbc_bound_template_reference_forward_walk() {
    let trials = [
        ("A. baseline (no trim reference)", None),
        ("B. trim reference, pitch_pd_gain=(0,0) (MPC x_ref only)", Some((0.0, 0.0))),
        ("C. trim reference, pitch_pd_gain=(100,10)", Some((100.0, 10.0))),
        ("D. trim reference, pitch_pd_gain=(200,20)", Some((200.0, 20.0))),
    ];
    for (label, bound_trim) in trials {
        let cfg = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
        let params = WbcParams {
            cmd_vx: 0.15,
            gait_type_override: Some(GaitType::Bound),
            bound_trim_reference: bound_trim,
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
                roll_pitch_weight_override: None, bound_fore_aft_placement_gain_override: None, roll_rate_weight_override: None, bound_pitch_placement_gain_override: None, bound_pitch_placement_dc_tau_override: None, bound_tabulated_reference_csv: None, bound_prescribed_footholds_override: None,
            }),
            ..WbcParams::forward_walk_misa_wbc(wbc::Formulation::ForceSpace, cfg)
        };
        if let Some(samples) = run_wbc_sim(params) {
            report_walk_summary(label, &samples, 0.15);
        }
    }
}

/// Video-capture source for the current (sign-corrected, Sec.5bc)
/// Bound trim-reference state: config C from `go2_wbc_bound_template_
/// reference_forward_walk` (`pitch_pd_gain=(100,10)`), the point where
/// the spurious roll instability the sign bug caused is essentially
/// resolved (peak|roll| 0.173 -> 0.006) but the core reversal still
/// isn't. Same cmd_vx/duration as the other Bound videos this session
/// (`go2_bound_reversal.mp4`, `go2_bound_low_swing.mp4`) for direct
/// comparison.
#[test]
#[ignore = "exploratory stress test — run with --ignored; also the WBC_WALK_CSV_OUT video-capture source for Sec.5bc"]
fn go2_wbc_bound_template_reference_video_source() {
    let cfg = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
    let params = WbcParams {
        cmd_vx: 0.15,
        total_time_s: 4.5,
        burn_in_s: 0.5,
        gait_type_override: Some(GaitType::Bound),
        bound_trim_reference: Some((100.0, 10.0)),
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
            roll_pitch_weight_override: None, bound_fore_aft_placement_gain_override: None, roll_rate_weight_override: None, bound_pitch_placement_gain_override: None, bound_pitch_placement_dc_tau_override: None, bound_tabulated_reference_csv: None, bound_prescribed_footholds_override: None,
        }),
        ..WbcParams::forward_walk_misa_wbc(wbc::Formulation::ForceSpace, cfg)
    };
    let Some(samples) = run_wbc_sim(params) else { return };
    report_walk_summary("bound trim-reference video-capture source (sign-fixed, pitch_pd_gain=(100,10))", &samples, 0.15);
}

/// Phase B (plan `frolicking-munching-scone.md`): MuJoCo confirmation
/// of the closed-form screening done in `ref/scripts/
/// simulate_point_mass_bound_sweep.py`. That script found `mu_needed
/// (0.721) > friction_mu (0.7)`, so `F_x` is always friction-clipped
/// regardless of `cycle_period_s`, which makes the velocity ripple
/// `delta_v(T) = mu*g*duty*T` -- LINEAR in `cycle_period_s` and
/// independent of `cmd_vx` -- while `theta_peak(T)` shrinks
/// QUADRATICALLY with `T`. Both knobs (raising `cmd_vx` above
/// `delta_v/2`, or shortening `cycle_period_s` to shrink `delta_v`
/// itself) were predicted to eliminate the reversal; this test
/// exercises the actual closed-loop pipeline (MPC + WBC, not just the
/// point-mass integration) at the trial points the script flagged as
/// theoretically viable, at the same sign-fixed trim reference config
/// as `go2_wbc_bound_template_reference_forward_walk`.
///
/// Trials (all with `bound_trim_reference: Some((100.0, 10.0))`):
/// - A: baseline (T=0.30, cmd_vx=0.15) -- known-reversing case, sanity check.
/// - B: cmd_vx-only fix (T=0.30, cmd_vx=0.60) -- script predicts
///   `min_cmd_vx_for_no_reversal=0.515`, comfortably cleared.
/// - C: period-only fix, extreme (T=0.09, cmd_vx=0.15) -- script's
///   `min_cmd_vx=0.1545` is barely above the tested 0.15, so this
///   trial is right at the theoretical edge; also stress-tests
///   whether the WBC/MPC solve loop and swing-leg kinematics tolerate
///   such a short cycle at all (T_st=0.045s per pair).
/// - D: combined, moderate (T=0.18, cmd_vx=0.40) -- script predicts
///   comfortable margin (`min_cmd_vx=0.309`) without an extreme period.
/// - E: combined, moderate (T=0.16, cmd_vx=0.30) -- same idea, closer
///   to the original target speed.
#[test]
#[ignore = "exploratory stress test — run with --ignored"]
fn go2_wbc_bound_period_cmd_vx_screening() {
    let trials: [(&str, f64, Option<f64>); 5] = [
        ("A. baseline (T=0.30, vx=0.15)", 0.15, None),
        ("B. cmd_vx-only fix (T=0.30, vx=0.60)", 0.60, None),
        ("C. period-only fix, extreme (T=0.09, vx=0.15)", 0.15, Some(0.09)),
        ("D. combined, moderate (T=0.18, vx=0.40)", 0.40, Some(0.18)),
        ("E. combined, moderate (T=0.16, vx=0.30)", 0.30, Some(0.16)),
    ];
    for (label, cmd_vx, period_override) in trials {
        let cfg = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
        let params = WbcParams {
            cmd_vx,
            gait_type_override: Some(GaitType::Bound),
            gait_cycle_period_override: period_override,
            bound_trim_reference: Some((100.0, 10.0)),
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
                roll_pitch_weight_override: None, bound_fore_aft_placement_gain_override: None, roll_rate_weight_override: None, bound_pitch_placement_gain_override: None, bound_pitch_placement_dc_tau_override: None, bound_tabulated_reference_csv: None, bound_prescribed_footholds_override: None,
            }),
            ..WbcParams::forward_walk_misa_wbc(wbc::Formulation::ForceSpace, cfg)
        };
        if let Some(samples) = run_wbc_sim(params) {
            report_walk_summary(label, &samples, cmd_vx);
        }
    }
}

/// Follow-up to `go2_wbc_bound_period_cmd_vx_screening`'s trial B
/// (T=0.30, cmd_vx=0.60 made meas_vx *more* negative than the
/// cmd_vx=0.15 baseline, contradicting the point-mass model's
/// prediction that raising cmd_vx alone should suffice). That was a
/// single data point; this sweeps cmd_vx finely at the SAME fixed
/// `cycle_period_s=0.30` to see whether the degradation is monotonic
/// (any cmd_vx increase hurts) or non-monotonic (some intermediate
/// speed is fine, only high speed misbehaves) -- which would point to
/// different root causes (a fundamental cmd_vx-scaling problem in the
/// MPC/WBC's handling of Bound, vs. something that only kicks in past
/// a specific speed threshold, e.g. footstep/friction-cone
/// saturation as the MPC's own velocity-tracking Fx demand starts
/// competing with the trim reference's oscillating Fx demand for the
/// same friction budget).
#[test]
#[ignore = "exploratory stress test — run with --ignored"]
fn go2_wbc_bound_cmd_vx_alone_curve() {
    for cmd_vx in [0.15, 0.20, 0.25, 0.30, 0.40, 0.50, 0.60, 0.80] {
        let cfg = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
        let params = WbcParams {
            cmd_vx,
            gait_type_override: Some(GaitType::Bound),
            gait_cycle_period_override: None, // fixed at GaitConfig::bound()'s default 0.30
            bound_trim_reference: Some((100.0, 10.0)),
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
                roll_pitch_weight_override: None, bound_fore_aft_placement_gain_override: None, roll_rate_weight_override: None, bound_pitch_placement_gain_override: None, bound_pitch_placement_dc_tau_override: None, bound_tabulated_reference_csv: None, bound_prescribed_footholds_override: None,
            }),
            ..WbcParams::forward_walk_misa_wbc(wbc::Formulation::ForceSpace, cfg)
        };
        if let Some(samples) = run_wbc_sim(params) {
            report_walk_summary(&format!("cmd_vx-alone curve (T=0.30, vx={cmd_vx:.2})"), &samples, cmd_vx);
        }
    }
}

/// Why cmd_vx-alone doesn't help (explains `go2_wbc_bound_cmd_vx_
/// alone_curve`'s flat, cmd_vx-insensitive meas_vx): the trim
/// reference's `F_x` (`BoundTrimConfig::f_x_clipped()`) doesn't depend
/// on `cmd_vx` at all -- it's sized purely to fight pitch torque -- and
/// at the real Go2 numbers `mu_needed=0.721 > friction_mu=0.7`, so it's
/// ALREADY clipped at the hard friction-cone boundary
/// (`friction_cone_soft=false` for Bound, `config.rs` L233/L351) by
/// itself, with zero headroom left in the same cone for the MPC's
/// separate velocity-tracking `F_x` term or the WBC's explicit
/// pitch-PD correction (`wbc_pipeline.rs` L511) to add anything. Raising
/// `cmd_vx` just widens the tracking error against a control authority
/// that was never available. Shortening `cycle_period_s` works instead
/// because it shrinks `delta_v=|F_x|/m*T_st` by reducing `T_st` (time),
/// not by freeing any `F_x` budget -- the saturation itself is
/// unchanged.
///
/// This predicts the WBC's explicit pitch-PD term (`pitch_pd_gain`)
/// should be a wash or actively counterproductive at a shortened
/// period too: it also draws on the same already-saturated `F_x`
/// budget as the MPC's own trim/velocity terms, so cranking pitch_kp
/// higher shouldn't buy better tracking. Sweeps `pitch_pd_gain` at the
/// most hardware-plausible operating point found so far (T=0.18,
/// cmd_vx=0.40 -- trial D, `meas_vx=0.270`, peak swing-foot speed
/// ≈2.65 m/s, under the repo's own 3.0 m/s Go2-leg guideline,
/// `config.rs` L322 `default_max_swing_foot_speed`).
#[test]
#[ignore = "exploratory stress test — run with --ignored"]
fn go2_wbc_bound_pitch_pd_gain_at_shortened_period_sweep() {
    let trials = [
        ("(0,0) -- MPC trim reference only, no explicit PD", (0.0, 0.0)),
        ("(50,5)", (50.0, 5.0)),
        ("(100,10) -- prior default", (100.0, 10.0)),
        ("(150,15)", (150.0, 15.0)),
        ("(200,20)", (200.0, 20.0)),
    ];
    for (label, gain) in trials {
        let cfg = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
        let params = WbcParams {
            cmd_vx: 0.40,
            gait_type_override: Some(GaitType::Bound),
            gait_cycle_period_override: Some(0.18),
            bound_trim_reference: Some(gain),
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
                roll_pitch_weight_override: None, bound_fore_aft_placement_gain_override: None, roll_rate_weight_override: None, bound_pitch_placement_gain_override: None, bound_pitch_placement_dc_tau_override: None, bound_tabulated_reference_csv: None, bound_prescribed_footholds_override: None,
            }),
            ..WbcParams::forward_walk_misa_wbc(wbc::Formulation::ForceSpace, cfg)
        };
        if let Some(samples) = run_wbc_sim(params) {
            report_walk_summary(&format!("pitch_pd_gain={label} (T=0.18, vx=0.40)"), &samples, 0.40);
        }
    }
}

/// Sec.5bf's diagnosis: at T=0.18/cmd_vx=0.40 (trial D), the trim's
/// own `F_x_clipped` already saturates the hard friction cone
/// (`mu_needed=0.721 > friction_mu=0.7`) by itself, leaving zero
/// headroom for velocity tracking -- confirmed by
/// `go2_wbc_bound_pitch_pd_gain_at_shortened_period_sweep` finding no
/// systematic improvement from retuning `pitch_pd_gain` alone. This
/// test exercises the alternative from the handover memo's Sec.6(c)
/// and `ref/scripts/simulate_point_mass_bound_sweep.py`'s partial-trim
/// analysis: deliberately command LESS than the fully-clipped trim
/// force (`BoundTrimConfig::thrust_scale < 1.0`), trading a larger
/// theoretical `theta_peak` for real friction-cone headroom
/// (`(1-thrust_scale)*mu*F_z` per pair) the MPC's own velocity-
/// tracking cost can actually spend. `thrust_scale=1.0` reproduces
/// trial D exactly (regression check against the prior result,
/// `meas_vx=0.270`).
#[test]
#[ignore = "exploratory stress test — run with --ignored"]
fn go2_wbc_bound_thrust_scale_sweep() {
    for thrust_scale in [1.0, 0.9, 0.8, 0.7, 0.6, 0.5, 0.4, 0.3] {
        let cfg = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
        let params = WbcParams {
            cmd_vx: 0.40,
            gait_type_override: Some(GaitType::Bound),
            gait_cycle_period_override: Some(0.18),
            bound_trim_reference: Some((100.0, 10.0)),
            bound_trim_thrust_scale_override: Some(thrust_scale),
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
                roll_pitch_weight_override: None, bound_fore_aft_placement_gain_override: None, roll_rate_weight_override: None, bound_pitch_placement_gain_override: None, bound_pitch_placement_dc_tau_override: None, bound_tabulated_reference_csv: None, bound_prescribed_footholds_override: None,
            }),
            ..WbcParams::forward_walk_misa_wbc(wbc::Formulation::ForceSpace, cfg)
        };
        if let Some(samples) = run_wbc_sim(params) {
            report_walk_summary(&format!("thrust_scale={thrust_scale:.1} (T=0.18, vx=0.40, pitch_pd=(100,10))"), &samples, 0.40);
        }
    }
}

/// Sec.5bj: the "impulse scaling" alternative to `go2_wbc_bound_
/// thrust_scale_sweep` above. Instead of scaling down the pitch-
/// cancelling trim by an arbitrary constant (`thrust_scale`), size
/// `F_x` directly from a target velocity ripple fraction of cmd_vx
/// (`BoundTrimConfig::velocity_ripple_fraction`, MIT Cheetah 2's
/// "vertical impulse scaling" philosophy -- Park/Wensing/Kim 2017).
/// `ref/scripts/simulate_point_mass_bound_sweep.py`'s closed-form
/// calibration found `fraction≈0.62` should reproduce `thrust_scale
/// =0.4`'s `F_x_used` at this same T=0.18/cmd_vx=0.40 point
/// (`meas_vx=0.270`, 67.5% tracking, the same starting point
/// `go2_wbc_bound_thrust_scale_sweep` used) -- sweep around it to see
/// whether cmd_vx-driven sizing tracks more consistently than the
/// constant-multiplier approach did.
#[test]
#[ignore = "exploratory stress test — run with --ignored"]
fn go2_wbc_bound_velocity_ripple_fraction_sweep() {
    for fraction in [0.3, 0.4, 0.5, 0.6, 0.62, 0.7, 0.8, 1.0] {
        let cfg = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
        let params = WbcParams {
            cmd_vx: 0.40,
            gait_type_override: Some(GaitType::Bound),
            gait_cycle_period_override: Some(0.18),
            bound_trim_reference: Some((100.0, 10.0)),
            bound_trim_velocity_ripple_fraction_override: Some(fraction),
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
                roll_pitch_weight_override: None, bound_fore_aft_placement_gain_override: None, roll_rate_weight_override: None, bound_pitch_placement_gain_override: None, bound_pitch_placement_dc_tau_override: None, bound_tabulated_reference_csv: None, bound_prescribed_footholds_override: None,
            }),
            ..WbcParams::forward_walk_misa_wbc(wbc::Formulation::ForceSpace, cfg)
        };
        if let Some(samples) = run_wbc_sim(params) {
            report_walk_summary(&format!("ripple_fraction={fraction:.2} (T=0.18, vx=0.40, pitch_pd=(100,10))"), &samples, 0.40);
        }
    }
}

/// Can Bound go faster than cmd_vx=0.40? At T=0.18, the footstep
/// planner's own speed ceiling (`v_max = max_step_length_m/
/// (cycle_period_s*duty_factor) = 0.12/0.09 = 1.33 m/s`, same formula
/// `wbc_walk_go2.rs` L1113-1114 uses) leaves headroom well above the
/// 0.40 tested so far, and raising `cmd_vx` doesn't add any NEW
/// swing-foot-speed risk (Sec.5bf point 3 sized `max_step_length_m`,
/// not `cmd_vx`, against the 3.0 m/s guideline). Sweeps `cmd_vx`
/// upward at the best config found so far (T=0.18,
/// thrust_scale=0.4, pitch_pd_gain=(100,10)) to find how far actual
/// tracked speed (not just the command) can be pushed before
/// something breaks.
#[test]
#[ignore = "exploratory stress test — run with --ignored"]
fn go2_wbc_bound_faster_cmd_vx_at_best_config_sweep() {
    for cmd_vx in [0.40, 0.50, 0.60, 0.70, 0.80, 1.00, 1.20, 1.33, 1.50, 1.80, 2.20] {
        let cfg = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
        let params = WbcParams {
            cmd_vx,
            gait_type_override: Some(GaitType::Bound),
            gait_cycle_period_override: Some(0.18),
            bound_trim_reference: Some((100.0, 10.0)),
            bound_trim_thrust_scale_override: Some(0.4),
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
                roll_pitch_weight_override: None, bound_fore_aft_placement_gain_override: None, roll_rate_weight_override: None, bound_pitch_placement_gain_override: None, bound_pitch_placement_dc_tau_override: None, bound_tabulated_reference_csv: None, bound_prescribed_footholds_override: None,
            }),
            ..WbcParams::forward_walk_misa_wbc(wbc::Formulation::ForceSpace, cfg)
        };
        if let Some(samples) = run_wbc_sim(params) {
            report_walk_summary(&format!("cmd_vx={cmd_vx:.2} (T=0.18, thrust_scale=0.4, pitch_pd=(100,10))"), &samples, cmd_vx);
        }
    }
}

/// Phase C of the flight-phase plan (local doc Sec.5bq): now that
/// `bound_reference.rs`'s trim model is generalized to `duty_factor
/// <0.5` (closed-form aerial-phase pitch/`F_z`, Phase 0/B of the plan),
/// re-run `go2_wbc_bound_flight_phase_duty_sweep`'s bare `n_stance=0`
/// survival check -- but layered on the actually-established best
/// config (T=0.18, `bound_trim_reference`=(100,10), thrust_scale=0.4,
/// `yaw_pd_gain`=(10,1), PLL gain=0.10/interval=1.0 -- the same combo
/// `go2_wbc_bound_adaptive_cycle_period_ramp_up_video_source` used)
/// instead of bare `forward_walk` defaults, at a previously-good
/// cmd_vx=1.30. Sweeps `duty_factor` down from 0.50 (baseline, no
/// flight) to see whether opening a genuine aerial phase raises the
/// achievable speed ceiling (Sec.5bi's ~1.0-1.2 m/s cap under the old
/// duty=0.5-only model) or breaks tracking/stability first.
#[test]
#[ignore = "exploratory stress test — run with --ignored"]
fn go2_wbc_bound_flight_phase_at_best_config_sweep() {
    let cmd_vx = 1.30;
    for duty_factor in [0.50, 0.45, 0.40, 0.35, 0.30, 0.25] {
        let cfg = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
        let params = WbcParams {
            cmd_vx,
            gait_type_override: Some(GaitType::Bound),
            duty_factor_override: Some(duty_factor),
            gait_cycle_period_override: Some(0.18),
            bound_trim_reference: Some((100.0, 10.0)),
            bound_trim_thrust_scale_override: Some(0.4),
            yaw_pd_gain_override: Some((10.0, 1.0)),
            adaptive_cycle_period: Some(AdaptivePeriodConfig {
                gain: 0.10,
                update_interval_s: 1.0,
                min_period_s: 0.14,
                max_period_s: 0.26,
            }),
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
                roll_pitch_weight_override: None, bound_fore_aft_placement_gain_override: None, roll_rate_weight_override: None, bound_pitch_placement_gain_override: None, bound_pitch_placement_dc_tau_override: None, bound_tabulated_reference_csv: None, bound_prescribed_footholds_override: None,
            }),
            ..WbcParams::forward_walk_misa_wbc(wbc::Formulation::ForceSpace, cfg)
        };
        if let Some(samples) = run_wbc_sim(params) {
            report_walk_summary(
                &format!(
                    "duty_factor={duty_factor:.2} (T=0.18, thrust_scale=0.4, bound_trim=(100,10), yaw_pd=(10,1), PLL)"
                ),
                &samples, cmd_vx,
            );
        }
    }
}

/// Companion to `go2_wbc_bound_flight_phase_at_best_config_sweep`:
/// that fixed-cmd_vx sweep found tracking monotonically WORSE as
/// `duty_factor` shrinks (contact_phase_mismatch growing 5.8%->27.2%),
/// the opposite of the hoped-for result -- but a fixed-cmd_vx sweep
/// can't distinguish "the achievable ceiling didn't move" from "the
/// ceiling moved but cmd_vx=1.30 is now past it in the other
/// direction". This sweeps `cmd_vx` at both `duty_factor=0.50`
/// (baseline, no flight) and `duty_factor=0.35` (30% flight/cycle),
/// with every other knob held byte-for-byte identical (T=0.18,
/// bound_trim=(100,10), thrust_scale=0.4, yaw_pd_gain=(10,1),
/// PLL gain=0.10/interval=1.0) -- a true apples-to-apples ceiling
/// comparison, unlike `go2_wbc_bound_faster_cmd_vx_at_best_config_
/// sweep` (an older sweep predating the yaw_pd_gain fix, so its
/// duty=0.5 numbers aren't directly comparable here).
#[test]
#[ignore = "exploratory stress test — run with --ignored"]
fn go2_wbc_bound_flight_phase_cmd_vx_ceiling_sweep() {
    for duty_factor in [0.50, 0.35] {
        for cmd_vx in [0.40, 0.50, 0.60, 0.70, 0.80, 1.00, 1.20, 1.33, 1.50, 1.80, 2.20] {
            let cfg = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
            let params = WbcParams {
                cmd_vx,
                gait_type_override: Some(GaitType::Bound),
                duty_factor_override: Some(duty_factor),
                gait_cycle_period_override: Some(0.18),
                bound_trim_reference: Some((100.0, 10.0)),
                bound_trim_thrust_scale_override: Some(0.4),
                yaw_pd_gain_override: Some((10.0, 1.0)),
                adaptive_cycle_period: Some(AdaptivePeriodConfig {
                    gain: 0.10,
                    update_interval_s: 1.0,
                    min_period_s: 0.14,
                    max_period_s: 0.26,
                }),
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
                    roll_pitch_weight_override: None, bound_fore_aft_placement_gain_override: None, roll_rate_weight_override: None, bound_pitch_placement_gain_override: None, bound_pitch_placement_dc_tau_override: None, bound_tabulated_reference_csv: None, bound_prescribed_footholds_override: None,
                }),
                ..WbcParams::forward_walk_misa_wbc(wbc::Formulation::ForceSpace, cfg)
            };
            if let Some(samples) = run_wbc_sim(params) {
                report_walk_summary(
                    &format!("cmd_vx={cmd_vx:.2} (duty_factor={duty_factor:.2}, T=0.18, thrust_scale=0.4, bound_trim=(100,10), yaw_pd=(10,1), PLL)"),
                    &samples, cmd_vx,
                );
            }
        }
    }
}

/// Sec.5bj companion to `go2_wbc_bound_faster_cmd_vx_at_best_config_
/// sweep`: same `cmd_vx` grid, same T=0.18/pitch_pd_gain=(100,10), but
/// `velocity_ripple_fraction=0.4` (best point from `go2_wbc_bound_
/// velocity_ripple_fraction_sweep`) instead of `thrust_scale=0.4`.
/// Unlike `thrust_scale` (which freezes `F_x`'s magnitude regardless
/// of `cmd_vx`, relying entirely on the MPC's separate velocity-
/// tracking cost to do the actual work), `velocity_ripple_fraction`
/// scales `F_x` WITH `cmd_vx` by construction -- the direct test of
/// whether that difference makes tracking more consistent across the
/// same speed range (particularly through the Sec.5bi instability band
/// around the footstep planner's `v_max=1.33`), not just at the single
/// cmd_vx=0.40 point the previous sweep checked.
#[test]
#[ignore = "exploratory stress test — run with --ignored"]
fn go2_wbc_bound_faster_cmd_vx_at_ripple_fraction_config_sweep() {
    for cmd_vx in [0.40, 0.50, 0.60, 0.70, 0.80, 1.00, 1.20, 1.33, 1.50, 1.80, 2.20] {
        let cfg = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
        let params = WbcParams {
            cmd_vx,
            gait_type_override: Some(GaitType::Bound),
            gait_cycle_period_override: Some(0.18),
            bound_trim_reference: Some((100.0, 10.0)),
            bound_trim_velocity_ripple_fraction_override: Some(0.4),
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
                roll_pitch_weight_override: None, bound_fore_aft_placement_gain_override: None, roll_rate_weight_override: None, bound_pitch_placement_gain_override: None, bound_pitch_placement_dc_tau_override: None, bound_tabulated_reference_csv: None, bound_prescribed_footholds_override: None,
            }),
            ..WbcParams::forward_walk_misa_wbc(wbc::Formulation::ForceSpace, cfg)
        };
        if let Some(samples) = run_wbc_sim(params) {
            report_walk_summary(&format!("cmd_vx={cmd_vx:.2} (T=0.18, ripple_fraction=0.4, pitch_pd=(100,10))"), &samples, cmd_vx);
        }
    }
}

/// Follow-up to `go2_wbc_bound_faster_cmd_vx_at_best_config_sweep`'s
/// Sec.5bh finding: tracking is clean and roughly proportional for
/// `cmd_vx <= 1.20` (90% of the footstep planner's own `v_max=1.33`
/// ceiling), then turns non-monotonic right around/above `v_max`
/// (1.33 and 1.80 degrade sharply -- `peak_pitch` up to 0.26-0.29 rad,
/// `min_z` down to 0.215 -- while 1.50 and 2.20 recover). This test
/// samples the boundary region densely (fine steps either side of
/// `v_max=1.33`) to see whether the transition is a single hard wall
/// at `v_max` or a genuinely oscillating/aliased region, and to find
/// where the sim actually fails outright (not just degrades) -- rather
/// than the coarse 0.3-0.5 spacing the prior sweep used, which could
/// easily straddle a much narrower unstable band without resolving
/// its shape.
#[test]
#[ignore = "exploratory stress test — run with --ignored"]
fn go2_wbc_bound_cmd_vx_boundary_fine_sweep() {
    for cmd_vx in [
        1.00, 1.10, 1.20, 1.25, 1.30, 1.33, 1.36, 1.40, 1.45, 1.50, 1.55, 1.60, 1.65, 1.70, 1.75,
        1.80, 1.85, 1.90, 1.95, 2.00, 2.10, 2.20,
    ] {
        let cfg = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
        let params = WbcParams {
            cmd_vx,
            gait_type_override: Some(GaitType::Bound),
            gait_cycle_period_override: Some(0.18),
            bound_trim_reference: Some((100.0, 10.0)),
            bound_trim_thrust_scale_override: Some(0.4),
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
                roll_pitch_weight_override: None, bound_fore_aft_placement_gain_override: None, roll_rate_weight_override: None, bound_pitch_placement_gain_override: None, bound_pitch_placement_dc_tau_override: None, bound_tabulated_reference_csv: None, bound_prescribed_footholds_override: None,
            }),
            ..WbcParams::forward_walk_misa_wbc(wbc::Formulation::ForceSpace, cfg)
        };
        if let Some(samples) = run_wbc_sim(params) {
            report_walk_summary(&format!("cmd_vx={cmd_vx:.2} (boundary fine sweep)"), &samples, cmd_vx);
        }
    }
}

/// Sec.5bl: does letting `cycle_period_s` adapt (a phase-locked loop
/// driven by `PhaseErrorTracker`'s signed error, Sec.5bk) fix the
/// sparse cmd_vx instability points `go2_wbc_bound_cmd_vx_boundary_
/// fine_sweep` found at fixed T=0.18 (1.25, 1.33, 1.36, 1.40, 1.80,
/// 1.90 all degraded; everything else clean)? Same cmd_vx grid,
/// same trim config, `total_time_s` extended to 6.0s (5.5s active)
/// to give the PLL time to converge. `gain=0.10`/`update_interval_s
/// =1.0` -- Sec.5bm's grid sweep default (the best point found there);
/// an initial `gain=1.0`/`update_interval_s=1.0` attempt made EVERY
/// point worse (including previously-healthy ones), and a first fix
/// (`gain=0.15`/`update_interval_s=2.0`) helped but turned out to be
/// one of the WORSE points in Sec.5bm's later grid -- `update_
/// interval_s` (not `gain`) was the dominant lever: shorter intervals
/// consistently outperform longer ones regardless of gain, as long as
/// gain stays under ~0.3. Reports both the walk-quality metrics
/// (comparable to the fixed-period sweep) and the converged
/// `cycle_period_s` -- the latter is this investigation's
/// answer to "what is the optimal period for this cmd_vx".
#[test]
#[ignore = "exploratory stress test — run with --ignored"]
fn go2_wbc_bound_adaptive_cycle_period_sweep() {
    for cmd_vx in [1.00, 1.20, 1.25, 1.30, 1.33, 1.36, 1.40, 1.45, 1.80, 1.90, 2.20] {
        let cfg = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
        let params = WbcParams {
            cmd_vx,
            total_time_s: 6.0,
            gait_type_override: Some(GaitType::Bound),
            gait_cycle_period_override: Some(0.18),
            bound_trim_reference: Some((100.0, 10.0)),
            bound_trim_thrust_scale_override: Some(0.4),
            adaptive_cycle_period: Some(AdaptivePeriodConfig {
                gain: 0.10,
                update_interval_s: 1.0,
                min_period_s: 0.14,
                max_period_s: 0.26,
            }),
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
                roll_pitch_weight_override: None, bound_fore_aft_placement_gain_override: None, roll_rate_weight_override: None, bound_pitch_placement_gain_override: None, bound_pitch_placement_dc_tau_override: None, bound_tabulated_reference_csv: None, bound_prescribed_footholds_override: None,
            }),
            ..WbcParams::forward_walk_misa_wbc(wbc::Formulation::ForceSpace, cfg)
        };
        if let Some(samples) = run_wbc_sim(params) {
            report_walk_summary(&format!("cmd_vx={cmd_vx:.2} (adaptive cycle_period)"), &samples, cmd_vx);
        }
    }
}

/// Video-capture source for the current best-evidenced Bound
/// configuration (Sec.5bm, local doc): T=0.18 base + `thrust_scale
/// =0.4` + `pitch_pd_gain=(100,10)` + the PLL at its Sec.5bm grid-
/// sweep default (`gain=0.10`, `update_interval_s=1.0`) at
/// `cmd_vx=1.33` -- the footstep planner's own `v_max`, and the
/// single worst point found in Sec.5bi's fixed-period sweep (34.0%
/// tracking, `peak_pitch` 0.261 rad). With the PLL enabled this
/// recovers to ~84% tracking
/// (Sec.5bm). Run with `WBC_WALK_CSV_OUT=<path> cargo test --release
/// --features mujoco --test wbc_walk_go2 go2_wbc_bound_adaptive_
/// cycle_period_video_source -- --ignored --nocapture`.
#[test]
#[ignore = "exploratory stress test — run with --ignored; also the WBC_WALK_CSV_OUT video-capture source for Sec.5bm"]
fn go2_wbc_bound_adaptive_cycle_period_video_source() {
    let cfg = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
    let params = WbcParams {
        cmd_vx: 1.33,
        total_time_s: 8.0,
        burn_in_s: 0.5,
        gait_type_override: Some(GaitType::Bound),
        gait_cycle_period_override: Some(0.18),
        bound_trim_reference: Some((100.0, 10.0)),
        bound_trim_thrust_scale_override: Some(0.4),
        adaptive_cycle_period: Some(AdaptivePeriodConfig {
            gain: 0.10,
            update_interval_s: 1.0,
            min_period_s: 0.14,
            max_period_s: 0.26,
        }),
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
            roll_pitch_weight_override: None, bound_fore_aft_placement_gain_override: None, roll_rate_weight_override: None, bound_pitch_placement_gain_override: None, bound_pitch_placement_dc_tau_override: None, bound_tabulated_reference_csv: None, bound_prescribed_footholds_override: None,
        }),
        ..WbcParams::forward_walk_misa_wbc(wbc::Formulation::ForceSpace, cfg)
    };
    let Some(samples) = run_wbc_sim(params) else { return };
    report_walk_summary("bound adaptive cycle_period video-capture source (cmd_vx=1.33, gain=0.10)", &samples, 1.33);
}

/// Companion to `go2_wbc_bound_adaptive_cycle_period_video_source`
/// (which captures the hardest point, cmd_vx=1.33=v_max): same
/// current-default config, but at cmd_vx=1.30 -- one of the best-
/// performing points in Sec.5bm's grid (88.1% tracking, peak_pitch
/// 0.100 rad), for a side-by-side "good case" reference alongside the
/// "hard case" video.
#[test]
#[ignore = "exploratory stress test — run with --ignored; also the WBC_WALK_CSV_OUT video-capture source"]
fn go2_wbc_bound_adaptive_cycle_period_good_point_video_source() {
    let cfg = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
    let params = WbcParams {
        cmd_vx: 1.30,
        total_time_s: 8.0,
        burn_in_s: 0.5,
        gait_type_override: Some(GaitType::Bound),
        gait_cycle_period_override: Some(0.18),
        bound_trim_reference: Some((100.0, 10.0)),
        bound_trim_thrust_scale_override: Some(0.4),
        adaptive_cycle_period: Some(AdaptivePeriodConfig {
            gain: 0.10,
            update_interval_s: 1.0,
            min_period_s: 0.14,
            max_period_s: 0.26,
        }),
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
            roll_pitch_weight_override: None, bound_fore_aft_placement_gain_override: None, roll_rate_weight_override: None, bound_pitch_placement_gain_override: None, bound_pitch_placement_dc_tau_override: None, bound_tabulated_reference_csv: None, bound_prescribed_footholds_override: None,
        }),
        ..WbcParams::forward_walk_misa_wbc(wbc::Formulation::ForceSpace, cfg)
    };
    let Some(samples) = run_wbc_sim(params) else { return };
    report_walk_summary("bound adaptive cycle_period video-capture source (cmd_vx=1.30, GOOD point)", &samples, 1.30);
}

/// Sec.5bp: does the new explicit yaw-holding feedback
/// (`WbcPipeline::yaw_pd_gain`/`yaw_ref`) fix the yaw drift Sec.5bo
/// found in the HARDER 3s ramp case (yaw drifted ~176 degrees,
/// misdiagnosed as "walking backward" before `yaw`/`body_y` were
/// tracked)? Sweeps `yaw_pd_gain` at fixed cmd_vx_ramp_s=3.0,
/// cmd_vx=1.30, everything else matching the Sec.5bo repro.
#[test]
#[ignore = "exploratory stress test — run with --ignored"]
fn go2_wbc_bound_yaw_pd_gain_ramp_fix_sweep() {
    for (kp, kd) in [(0.0, 0.0), (5.0, 0.5), (10.0, 1.0), (20.0, 2.0), (50.0, 5.0)] {
        let cfg = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
        let params = WbcParams {
            cmd_vx: 1.30,
            cmd_vx_ramp_s: Some(3.0),
            total_time_s: 10.0,
            burn_in_s: 0.5,
            gait_type_override: Some(GaitType::Bound),
            gait_cycle_period_override: Some(0.18),
            bound_trim_reference: Some((100.0, 10.0)),
            bound_trim_thrust_scale_override: Some(0.4),
            yaw_pd_gain_override: Some((kp, kd)),
            adaptive_cycle_period: Some(AdaptivePeriodConfig {
                gain: 0.10,
                update_interval_s: 1.0,
                min_period_s: 0.14,
                max_period_s: 0.26,
            }),
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
                roll_pitch_weight_override: None, bound_fore_aft_placement_gain_override: None, roll_rate_weight_override: None, bound_pitch_placement_gain_override: None, bound_pitch_placement_dc_tau_override: None, bound_tabulated_reference_csv: None, bound_prescribed_footholds_override: None,
            }),
            ..WbcParams::forward_walk_misa_wbc(wbc::Formulation::ForceSpace, cfg)
        };
        if let Some(samples) = run_wbc_sim(params) {
            report_walk_summary(&format!("yaw_pd_gain=({kp:.0},{kd:.1}) (3s ramp, cmd_vx=1.30)"), &samples, 1.30);
        }
    }
}

/// Does a smooth `cmd_vx_ramp_s` startup (stride grows with cmd_vx via
/// the existing Raibert-heuristic footstep planner, no new machinery
/// needed) coexist with the PLL? The PLL was only ever validated at
/// STEADY cmd_vx (Sec.5bl-5bn) -- during a ramp, both the commanded
/// speed and the real contact timing are changing simultaneously.
/// FIXED (Sec.5bp, local doc): a 3s ramp to cmd_vx=1.30 (the "GOOD
/// point" video's target) drifted yaw ~176 degrees uncorrected
/// (Sec.5bo -- misdiagnosed as "walking backward" before `yaw`/
/// `body_y` were tracked); the new explicit yaw-holding feedback
/// (`yaw_pd_gain_override`, tuned in `go2_wbc_bound_yaw_pd_gain_
/// ramp_fix_sweep`) resolves it.
#[test]
#[ignore = "exploratory stress test — run with --ignored; also the WBC_WALK_CSV_OUT video-capture source"]
fn go2_wbc_bound_adaptive_cycle_period_ramp_up_video_source() {
    let cfg = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
    let params = WbcParams {
        cmd_vx: 1.30,
        cmd_vx_ramp_s: Some(3.0),
        total_time_s: 10.0,
        burn_in_s: 0.5,
        gait_type_override: Some(GaitType::Bound),
        gait_cycle_period_override: Some(0.18),
        bound_trim_reference: Some((100.0, 10.0)),
        bound_trim_thrust_scale_override: Some(0.4),
        yaw_pd_gain_override: Some((10.0, 1.0)),
        adaptive_cycle_period: Some(AdaptivePeriodConfig {
            gain: 0.10,
            update_interval_s: 1.0,
            min_period_s: 0.14,
            max_period_s: 0.26,
        }),
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
            roll_pitch_weight_override: None, bound_fore_aft_placement_gain_override: None, roll_rate_weight_override: None, bound_pitch_placement_gain_override: None, bound_pitch_placement_dc_tau_override: None, bound_tabulated_reference_csv: None, bound_prescribed_footholds_override: None,
        }),
        ..WbcParams::forward_walk_misa_wbc(wbc::Formulation::ForceSpace, cfg)
    };
    let Some(samples) = run_wbc_sim(params) else { return };
    report_walk_summary("bound ramp-up video-capture source (0 -> cmd_vx=1.30 over 3s, yaw_pd_gain=(10,1) -- FIXED)", &samples, 1.30);
}

/// Sec.5bl found the PLL (gain=0.15, update_interval_s=2.0) barely moved `peak_pitch` at
/// cmd_vx=1.33 (0.261 rad fixed-period -> 0.251 rad with PLL, <4%
/// change) despite a large `meas_vx` improvement (34.0% -> 54.6%) --
/// i.e. `peak_pitch` looks like an inherent feature of this speed
/// regime, largely independent of the period-tuning mechanism. This
/// sweeps `thrust_scale` (Sec.5bg's independent Fx-sizing knob, which
/// DOES directly affect `theta_peak` in the closed-form model) WITH
/// the PLL enabled, at the worst point (cmd_vx=1.33), to see whether
/// that orthogonal lever succeeds where period-tuning alone didn't.
#[test]
#[ignore = "exploratory stress test — run with --ignored"]
fn go2_wbc_bound_thrust_scale_with_pll_sweep() {
    for thrust_scale in [0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 1.0] {
        let cfg = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
        let params = WbcParams {
            cmd_vx: 1.33,
            total_time_s: 6.0,
            gait_type_override: Some(GaitType::Bound),
            gait_cycle_period_override: Some(0.18),
            bound_trim_reference: Some((100.0, 10.0)),
            bound_trim_thrust_scale_override: Some(thrust_scale),
            adaptive_cycle_period: Some(AdaptivePeriodConfig {
                gain: 0.10,
                update_interval_s: 1.0,
                min_period_s: 0.14,
                max_period_s: 0.26,
            }),
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
                roll_pitch_weight_override: None, bound_fore_aft_placement_gain_override: None, roll_rate_weight_override: None, bound_pitch_placement_gain_override: None, bound_pitch_placement_dc_tau_override: None, bound_tabulated_reference_csv: None, bound_prescribed_footholds_override: None,
            }),
            ..WbcParams::forward_walk_misa_wbc(wbc::Formulation::ForceSpace, cfg)
        };
        if let Some(samples) = run_wbc_sim(params) {
            report_walk_summary(&format!("thrust_scale={thrust_scale:.1} + PLL (cmd_vx=1.33)"), &samples, 1.33);
        }
    }
}

/// Does `thrust_scale=0.5` (best single point from `go2_wbc_bound_
/// thrust_scale_with_pll_sweep`, at cmd_vx=1.33: 87.8% tracking,
/// peak_pitch=0.097 rad vs 0.4's 54.6%/0.251) generalize to the OTHER
/// points Sec.5bi/5bl found degraded (1.25, 1.36, 1.80, 1.90), or was
/// it a one-off lucky fit to 1.33 specifically?
#[test]
#[ignore = "exploratory stress test — run with --ignored"]
fn go2_wbc_bound_thrust_scale_0_5_with_pll_generalization_sweep() {
    for cmd_vx in [1.25, 1.33, 1.36, 1.80, 1.90] {
        let cfg = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
        let params = WbcParams {
            cmd_vx,
            total_time_s: 6.0,
            gait_type_override: Some(GaitType::Bound),
            gait_cycle_period_override: Some(0.18),
            bound_trim_reference: Some((100.0, 10.0)),
            bound_trim_thrust_scale_override: Some(0.5),
            adaptive_cycle_period: Some(AdaptivePeriodConfig {
                gain: 0.10,
                update_interval_s: 1.0,
                min_period_s: 0.14,
                max_period_s: 0.26,
            }),
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
                roll_pitch_weight_override: None, bound_fore_aft_placement_gain_override: None, roll_rate_weight_override: None, bound_pitch_placement_gain_override: None, bound_pitch_placement_dc_tau_override: None, bound_tabulated_reference_csv: None, bound_prescribed_footholds_override: None,
            }),
            ..WbcParams::forward_walk_misa_wbc(wbc::Formulation::ForceSpace, cfg)
        };
        if let Some(samples) = run_wbc_sim(params) {
            report_walk_summary(&format!("cmd_vx={cmd_vx:.2} (thrust_scale=0.5 + PLL)"), &samples, cmd_vx);
        }
    }
}

/// Sec.5bl only tried 2 (gain, update_interval_s) points
/// (`(1.0, 1.0)` -- catastrophic, `(0.15, 2.0)` -- good). Grids the
/// neighbourhood around the working point to see if a better-
/// converging combination exists, at cmd_vx=1.33 with `thrust_scale`
/// held at the ORIGINAL 0.4 (not Step 1's 0.5) so this stays an
/// isolated test of the PLL's own gain/interval, not conflated with
/// the Fx-sizing finding.
#[test]
#[ignore = "exploratory stress test — run with --ignored"]
fn go2_wbc_bound_pll_gain_interval_grid_sweep() {
    for gain in [0.05, 0.10, 0.15, 0.20, 0.30] {
        for update_interval_s in [1.0, 2.0, 4.0] {
            let cfg = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
            let params = WbcParams {
                cmd_vx: 1.33,
                total_time_s: 8.0,
                gait_type_override: Some(GaitType::Bound),
                gait_cycle_period_override: Some(0.18),
                bound_trim_reference: Some((100.0, 10.0)),
                bound_trim_thrust_scale_override: Some(0.4),
                adaptive_cycle_period: Some(AdaptivePeriodConfig {
                    gain,
                    update_interval_s,
                    min_period_s: 0.14,
                    max_period_s: 0.26,
                }),
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
                    roll_pitch_weight_override: None, bound_fore_aft_placement_gain_override: None, roll_rate_weight_override: None, bound_pitch_placement_gain_override: None, bound_pitch_placement_dc_tau_override: None, bound_tabulated_reference_csv: None, bound_prescribed_footholds_override: None,
                }),
                ..WbcParams::forward_walk_misa_wbc(wbc::Formulation::ForceSpace, cfg)
            };
            if let Some(samples) = run_wbc_sim(params) {
                report_walk_summary(
                    &format!("gain={gain:.2} interval={update_interval_s:.1}s (cmd_vx=1.33)"),
                    &samples,
                    1.33,
                );
            }
        }
    }
}

/// Literature re-check (Cheng/Alqaham/Gan 2024 "Harnessing Natural
/// Oscillations", Poulakakis/Buehler's Scout II): both explicitly
/// reject servoing pitch to a target, letting it rotate passively as
/// part of the limit cycle instead. `wbc_pipeline.pitch_pd_gain`
/// actively tracks the trim reference's pitch target every tick --
/// exactly the thing that literature says fights the natural
/// dynamics. Sweeps it toward (0,0) at cmd_vx=1.33 (PLL gain=0.10/
/// interval=1.0, thrust_scale=0.4 -- Sec.5bm's baseline, so this test
/// isolates pitch_pd_gain's own effect) to see whether backing off
/// the correction lets peak_pitch settle into a SMALLER natural
/// oscillation rather than the current ~0.22 rad.
/// `pitch_pd_gain_override` takes effect after `bound_trim_reference`
/// sets `wbc_pipeline.pitch_pd_gain` from its own tuple (both write
/// the same field; the override applies later in `run_wbc_sim`), so
/// `bound_trim_reference`'s `(100,10)` here only enables the trim
/// machinery -- the actual gain used is whatever this override says.
#[test]
#[ignore = "exploratory stress test — run with --ignored"]
fn go2_wbc_bound_pitch_pd_gain_toward_zero_sweep() {
    for (kp, kd) in [(100.0, 10.0), (50.0, 5.0), (20.0, 2.0), (10.0, 1.0), (5.0, 0.5), (0.0, 0.0)] {
        let cfg = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
        let params = WbcParams {
            cmd_vx: 1.33,
            total_time_s: 8.0,
            gait_type_override: Some(GaitType::Bound),
            gait_cycle_period_override: Some(0.18),
            bound_trim_reference: Some((100.0, 10.0)),
            bound_trim_thrust_scale_override: Some(0.4),
            pitch_pd_gain_override: Some((kp, kd)),
            adaptive_cycle_period: Some(AdaptivePeriodConfig {
                gain: 0.10,
                update_interval_s: 1.0,
                min_period_s: 0.14,
                max_period_s: 0.26,
            }),
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
                roll_pitch_weight_override: None, bound_fore_aft_placement_gain_override: None, roll_rate_weight_override: None, bound_pitch_placement_gain_override: None, bound_pitch_placement_dc_tau_override: None, bound_tabulated_reference_csv: None, bound_prescribed_footholds_override: None,
            }),
            ..WbcParams::forward_walk_misa_wbc(wbc::Formulation::ForceSpace, cfg)
        };
        if let Some(samples) = run_wbc_sim(params) {
            report_walk_summary(&format!("pitch_pd_gain=({kp:.0},{kd:.1}) (cmd_vx=1.33)"), &samples, 1.33);
        }
    }
}

/// Does `pitch_pd_gain=(0,0)` (the literature-backed finding from
/// `go2_wbc_bound_pitch_pd_gain_toward_zero_sweep`: at cmd_vx=1.33 it
/// improved tracking 84.1%->88.7% AND halved peak_pitch 0.222->0.103)
/// generalize across the cmd_vx grid, or was 1.33 a lucky fit? Same
/// grid as `go2_wbc_bound_cmd_vx_boundary_fine_sweep`/`go2_wbc_bound_
/// thrust_scale_0_5_with_pll_generalization_sweep`, PLL(gain=0.10,
/// interval=1.0)+thrust_scale=0.4 held fixed, `pitch_pd_gain_
/// override=(0,0)` throughout -- i.e. Bound's WBC never explicitly
/// servos pitch at all, relying solely on the trim reference's
/// periodic Fx/GRF schedule and the PLL's timing correction.
#[test]
#[ignore = "exploratory stress test — run with --ignored"]
fn go2_wbc_bound_pitch_pd_gain_zero_generalization_sweep() {
    for cmd_vx in [1.00, 1.20, 1.25, 1.30, 1.33, 1.36, 1.40, 1.45, 1.80, 1.90, 2.20] {
        let cfg = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
        let params = WbcParams {
            cmd_vx,
            total_time_s: 8.0,
            gait_type_override: Some(GaitType::Bound),
            gait_cycle_period_override: Some(0.18),
            bound_trim_reference: Some((100.0, 10.0)),
            bound_trim_thrust_scale_override: Some(0.4),
            pitch_pd_gain_override: Some((0.0, 0.0)),
            adaptive_cycle_period: Some(AdaptivePeriodConfig {
                gain: 0.10,
                update_interval_s: 1.0,
                min_period_s: 0.14,
                max_period_s: 0.26,
            }),
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
                roll_pitch_weight_override: None, bound_fore_aft_placement_gain_override: None, roll_rate_weight_override: None, bound_pitch_placement_gain_override: None, bound_pitch_placement_dc_tau_override: None, bound_tabulated_reference_csv: None, bound_prescribed_footholds_override: None,
            }),
            ..WbcParams::forward_walk_misa_wbc(wbc::Formulation::ForceSpace, cfg)
        };
        if let Some(samples) = run_wbc_sim(params) {
            report_walk_summary(&format!("cmd_vx={cmd_vx:.2} (pitch_pd_gain=(0,0), PLL+thrust_scale=0.4)"), &samples, cmd_vx);
        }
    }
}

/// Pushes `cmd_vx` far beyond the footstep planner's `v_max=1.33`
/// ceiling to find the ultimate breaking point (outright fall /
/// non-finite state), not just the degraded-but-still-standing
/// behaviour `go2_wbc_bound_cmd_vx_boundary_fine_sweep` found just
/// above `v_max`.
#[test]
#[ignore = "exploratory stress test — run with --ignored"]
fn go2_wbc_bound_cmd_vx_extreme_ceiling_sweep() {
    for cmd_vx in [2.50, 3.00, 3.50, 4.00, 5.00, 6.00, 8.00] {
        let cfg = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
        let params = WbcParams {
            cmd_vx,
            gait_type_override: Some(GaitType::Bound),
            gait_cycle_period_override: Some(0.18),
            bound_trim_reference: Some((100.0, 10.0)),
            bound_trim_thrust_scale_override: Some(0.4),
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
                roll_pitch_weight_override: None, bound_fore_aft_placement_gain_override: None, roll_rate_weight_override: None, bound_pitch_placement_gain_override: None, bound_pitch_placement_dc_tau_override: None, bound_tabulated_reference_csv: None, bound_prescribed_footholds_override: None,
            }),
            ..WbcParams::forward_walk_misa_wbc(wbc::Formulation::ForceSpace, cfg)
        };
        if let Some(samples) = run_wbc_sim(params) {
            report_walk_summary(&format!("cmd_vx={cmd_vx:.2} (extreme ceiling sweep)"), &samples, cmd_vx);
        }
    }
}

/// Video-capture source for `go2_wbc_bound_thrust_scale_sweep`'s BEST
/// case (`thrust_scale=0.4`, `meas_vx=0.296`, 74.0% of cmd_vx=0.40 --
/// the best tracking found across every knob swept in Sec.5be/5bf/5bg).
/// Run with `WBC_WALK_CSV_OUT=<path> cargo test --release --features
/// mujoco --test wbc_walk_go2 go2_wbc_bound_thrust_scale_best_video_source
/// -- --ignored --nocapture`.
#[test]
#[ignore = "exploratory stress test — run with --ignored; also the WBC_WALK_CSV_OUT video-capture source for Sec.5bg (best case)"]
fn go2_wbc_bound_thrust_scale_best_video_source() {
    let cfg = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
    let params = WbcParams {
        cmd_vx: 0.40,
        total_time_s: 4.5,
        burn_in_s: 0.5,
        gait_type_override: Some(GaitType::Bound),
        gait_cycle_period_override: Some(0.18),
        bound_trim_reference: Some((100.0, 10.0)),
        bound_trim_thrust_scale_override: Some(0.4),
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
            roll_pitch_weight_override: None, bound_fore_aft_placement_gain_override: None, roll_rate_weight_override: None, bound_pitch_placement_gain_override: None, bound_pitch_placement_dc_tau_override: None, bound_tabulated_reference_csv: None, bound_prescribed_footholds_override: None,
        }),
        ..WbcParams::forward_walk_misa_wbc(wbc::Formulation::ForceSpace, cfg)
    };
    let Some(samples) = run_wbc_sim(params) else { return };
    report_walk_summary("bound thrust_scale=0.4 video-capture source (BEST, T=0.18, vx=0.40)", &samples, 0.40);
}

/// Video-capture source for `go2_wbc_bound_thrust_scale_sweep`'s WORST
/// case (`thrust_scale=0.5`, `meas_vx=0.201`, 50.3% of cmd_vx=0.40 --
/// the worst tracking within the same otherwise-identical sweep,
/// direct before/after comparison against the best case above). Run
/// with `WBC_WALK_CSV_OUT=<path> cargo test --release --features
/// mujoco --test wbc_walk_go2 go2_wbc_bound_thrust_scale_worst_video_source
/// -- --ignored --nocapture`.
#[test]
#[ignore = "exploratory stress test — run with --ignored; also the WBC_WALK_CSV_OUT video-capture source for Sec.5bg (worst case)"]
fn go2_wbc_bound_thrust_scale_worst_video_source() {
    let cfg = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
    let params = WbcParams {
        cmd_vx: 0.40,
        total_time_s: 4.5,
        burn_in_s: 0.5,
        gait_type_override: Some(GaitType::Bound),
        gait_cycle_period_override: Some(0.18),
        bound_trim_reference: Some((100.0, 10.0)),
        bound_trim_thrust_scale_override: Some(0.5),
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
            roll_pitch_weight_override: None, bound_fore_aft_placement_gain_override: None, roll_rate_weight_override: None, bound_pitch_placement_gain_override: None, bound_pitch_placement_dc_tau_override: None, bound_tabulated_reference_csv: None, bound_prescribed_footholds_override: None,
        }),
        ..WbcParams::forward_walk_misa_wbc(wbc::Formulation::ForceSpace, cfg)
    };
    let Some(samples) = run_wbc_sim(params) else { return };
    report_walk_summary("bound thrust_scale=0.5 video-capture source (WORST, T=0.18, vx=0.40)", &samples, 0.40);
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

/// Video-capture source for Sec.5bq's `duty_factor=0.35` flight-phase
/// result: the *slower* of the two ceilings found (`go2_wbc_bound_
/// flight_phase_cmd_vx_ceiling_sweep` -- ~0.82 m/s vs duty=0.50's
/// ~0.99 m/s), but the point the user explicitly asked to see despite
/// the negative tracking result -- confirming visually that all 4
/// legs really do leave the ground together (the original "any true
/// flight phase?" question that started this investigation), not just
/// that the schedule-level diagnostic (`go2_diag_bound_duty_factor_
/// flight_phase_sweep`) says so. `cmd_vx=1.20` sits inside
/// this duty's own tracked-speed plateau (0.821 m/s, Sec.5bq's table).
/// Run with `WBC_WALK_CSV_OUT=<path> cargo test --release --features
/// mujoco --test wbc_walk_go2 go2_wbc_bound_flight_phase_duty_035_video_source
/// -- --ignored --nocapture`.
#[test]
#[ignore = "exploratory stress test — run with --ignored; also the WBC_WALK_CSV_OUT video-capture source for Sec.5bq (duty=0.35 flight phase)"]
fn go2_wbc_bound_flight_phase_duty_035_video_source() {
    let cfg = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
    let params = WbcParams {
        cmd_vx: 1.20,
        total_time_s: 4.5,
        burn_in_s: 0.5,
        gait_type_override: Some(GaitType::Bound),
        duty_factor_override: Some(0.35),
        gait_cycle_period_override: Some(0.18),
        bound_trim_reference: Some((100.0, 10.0)),
        bound_trim_thrust_scale_override: Some(0.4),
        yaw_pd_gain_override: Some((10.0, 1.0)),
        adaptive_cycle_period: Some(AdaptivePeriodConfig {
            gain: 0.10,
            update_interval_s: 1.0,
            min_period_s: 0.14,
            max_period_s: 0.26,
        }),
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
            roll_pitch_weight_override: None, bound_fore_aft_placement_gain_override: None, roll_rate_weight_override: None, bound_pitch_placement_gain_override: None, bound_pitch_placement_dc_tau_override: None, bound_tabulated_reference_csv: None, bound_prescribed_footholds_override: None,
        }),
        ..WbcParams::forward_walk_misa_wbc(wbc::Formulation::ForceSpace, cfg)
    };
    let Some(samples) = run_wbc_sim(params) else { return };
    report_walk_summary(
        "bound duty_factor=0.35 flight-phase video-capture source (T=0.18, thrust_scale=0.4, bound_trim=(100,10), yaw_pd=(10,1), PLL, vx=1.20)",
        &samples, 1.20,
    );
}

/// Sec.5bq found `duty_factor<0.5` doesn't raise the tracked-speed
/// ceiling under `thrust_scale=0.4` (established for `duty=0.5`) --
/// but `f_z_total()`'s duty-aware growth (`m·g/(2·duty)`, Sec.5bq)
/// means the *absolute* friction budget (`mu·F_z_total`) is objectively
/// larger at `duty=0.35` than `duty=0.50`, even though `thrust_scale`'s
/// fraction of it is unchanged -- `thrust_scale=0.4` was chosen at
/// `duty=0.5` specifically to leave headroom below the pitch-canceling
/// trim's own friction-cone saturation (Sec.5bf/5bg), and might be
/// needlessly conservative once a genuine flight phase is already
/// absorbing some of the pitch disturbance (Phase 0's `theta_peak`
/// only grows mildly, 0.00904->0.01175 rad, duty 0.50->0.35). Sweeps
/// `thrust_scale` upward at `duty_factor=0.35`, fixed cmd_vx=1.33 (near
/// Sec.5bq's duty=0.35 ceiling), to see whether more of that headroom
/// can be spent before instability (rising `contact_phase_mismatch`,
/// `peak_pitch`, or an outright fall) sets in.
#[test]
#[ignore = "exploratory stress test — run with --ignored"]
fn go2_wbc_bound_flight_phase_duty035_thrust_scale_sweep() {
    let cmd_vx = 1.33;
    for thrust_scale in [0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0] {
        let cfg = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
        let params = WbcParams {
            cmd_vx,
            gait_type_override: Some(GaitType::Bound),
            duty_factor_override: Some(0.35),
            gait_cycle_period_override: Some(0.18),
            bound_trim_reference: Some((100.0, 10.0)),
            bound_trim_thrust_scale_override: Some(thrust_scale),
            yaw_pd_gain_override: Some((10.0, 1.0)),
            adaptive_cycle_period: Some(AdaptivePeriodConfig {
                gain: 0.10,
                update_interval_s: 1.0,
                min_period_s: 0.14,
                max_period_s: 0.26,
            }),
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
                roll_pitch_weight_override: None, bound_fore_aft_placement_gain_override: None, roll_rate_weight_override: None, bound_pitch_placement_gain_override: None, bound_pitch_placement_dc_tau_override: None, bound_tabulated_reference_csv: None, bound_prescribed_footholds_override: None,
            }),
            ..WbcParams::forward_walk_misa_wbc(wbc::Formulation::ForceSpace, cfg)
        };
        if let Some(samples) = run_wbc_sim(params) {
            report_walk_summary(
                &format!("thrust_scale={thrust_scale:.1} (duty_factor=0.35, T=0.18, cmd_vx=1.33)"),
                &samples, cmd_vx,
            );
        }
    }
}

/// Companion to `go2_wbc_bound_flight_phase_duty035_thrust_scale_
/// sweep`: `cycle_period_s=0.18` was tuned for `duty_factor=0.5`
/// (Sec.5bg-on) -- with a genuine flight phase now in the picture, the
/// stance/flight time split changes character at a fixed `duty`, so
/// the period that best balances stride frequency against per-stance
/// impulse may no longer be 0.18s. Sweeps `cycle_period_s` at
/// `duty_factor=0.35`, `thrust_scale=0.4` (baseline, isolating the
/// period's own effect first), fixed cmd_vx=1.33.
#[test]
#[ignore = "exploratory stress test — run with --ignored"]
fn go2_wbc_bound_flight_phase_duty035_cycle_period_sweep() {
    let cmd_vx = 1.33;
    for cycle_period_s in [0.14, 0.16, 0.18, 0.20, 0.22, 0.24, 0.26] {
        let cfg = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
        let params = WbcParams {
            cmd_vx,
            gait_type_override: Some(GaitType::Bound),
            duty_factor_override: Some(0.35),
            gait_cycle_period_override: Some(cycle_period_s),
            bound_trim_reference: Some((100.0, 10.0)),
            bound_trim_thrust_scale_override: Some(0.4),
            yaw_pd_gain_override: Some((10.0, 1.0)),
            adaptive_cycle_period: Some(AdaptivePeriodConfig {
                gain: 0.10,
                update_interval_s: 1.0,
                min_period_s: (cycle_period_s - 0.04).max(0.05),
                max_period_s: cycle_period_s + 0.08,
            }),
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
                roll_pitch_weight_override: None, bound_fore_aft_placement_gain_override: None, roll_rate_weight_override: None, bound_pitch_placement_gain_override: None, bound_pitch_placement_dc_tau_override: None, bound_tabulated_reference_csv: None, bound_prescribed_footholds_override: None,
            }),
            ..WbcParams::forward_walk_misa_wbc(wbc::Formulation::ForceSpace, cfg)
        };
        if let Some(samples) = run_wbc_sim(params) {
            report_walk_summary(
                &format!("cycle_period_s={cycle_period_s:.2} (duty_factor=0.35, thrust_scale=0.4, cmd_vx=1.33)"),
                &samples, cmd_vx,
            );
        }
    }
}

/// `go2_wbc_bound_flight_phase_duty035_thrust_scale_sweep` found
/// `thrust_scale=1.0` at `duty_factor=0.35` (with the non-monotonic
/// 0.6-0.8 instability pocket avoided) tracks a fixed cmd_vx=1.33
/// better than the `duty=0.5`-tuned `thrust_scale=0.4` baseline
/// (0.875 vs 0.805 m/s, +8.7%) -- `cycle_period_s=0.18` remained
/// the best period at this duty (`..._cycle_period_sweep`, no gain
/// there). This re-runs the full `cmd_vx` ceiling sweep (same grid as
/// `go2_wbc_bound_flight_phase_cmd_vx_ceiling_sweep`) at
/// `duty_factor=0.35`+`thrust_scale=1.0` to see whether that
/// modest fixed-point gain also raises the actual tracking ceiling
/// close to (or past) `duty=0.50`'s ~0.99 m/s (Sec.5bq).
#[test]
#[ignore = "exploratory stress test — run with --ignored"]
fn go2_wbc_bound_flight_phase_duty035_thrust_scale_1_ceiling_sweep() {
    for cmd_vx in [0.40, 0.50, 0.60, 0.70, 0.80, 1.00, 1.20, 1.33, 1.50, 1.80, 2.20] {
        let cfg = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
        let params = WbcParams {
            cmd_vx,
            gait_type_override: Some(GaitType::Bound),
            duty_factor_override: Some(0.35),
            gait_cycle_period_override: Some(0.18),
            bound_trim_reference: Some((100.0, 10.0)),
            bound_trim_thrust_scale_override: Some(1.0),
            yaw_pd_gain_override: Some((10.0, 1.0)),
            adaptive_cycle_period: Some(AdaptivePeriodConfig {
                gain: 0.10,
                update_interval_s: 1.0,
                min_period_s: 0.14,
                max_period_s: 0.26,
            }),
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
                roll_pitch_weight_override: None, bound_fore_aft_placement_gain_override: None, roll_rate_weight_override: None, bound_pitch_placement_gain_override: None, bound_pitch_placement_dc_tau_override: None, bound_tabulated_reference_csv: None, bound_prescribed_footholds_override: None,
            }),
            ..WbcParams::forward_walk_misa_wbc(wbc::Formulation::ForceSpace, cfg)
        };
        if let Some(samples) = run_wbc_sim(params) {
            report_walk_summary(
                &format!("cmd_vx={cmd_vx:.2} (duty_factor=0.35, thrust_scale=1.0, T=0.18)"),
                &samples, cmd_vx,
            );
        }
    }
}

/// Video-capture source for Sec.5br's positive result: `duty_factor=
/// 0.35` (genuine flight phase) with `thrust_scale=1.0` (raised from
/// `duty=0.50`'s conservative 0.4, per `go2_wbc_bound_flight_phase_
/// duty035_thrust_scale_sweep`'s finding that the friction-cone
/// headroom `thrust_scale=0.4` was protecting no longer needs
/// protecting at this duty) tracks `cmd_vx=1.50` at `meas_vx≈0.985` --
/// essentially matching `duty=0.50`'s own ~0.99 m/s ceiling (Sec.5bq)
/// while keeping the aerial phase. This is the "leap AND keep the
/// speed" configuration the user asked for. Run with `WBC_WALK_
/// CSV_OUT=<path> cargo test --release --features mujoco --test
/// wbc_walk_go2 go2_wbc_bound_flight_phase_duty035_thrust_scale_1_
/// video_source -- --ignored --nocapture`.
#[test]
#[ignore = "exploratory stress test — run with --ignored; also the WBC_WALK_CSV_OUT video-capture source for Sec.5br (duty=0.35, thrust_scale=1.0, ceiling-matching)"]
fn go2_wbc_bound_flight_phase_duty035_thrust_scale_1_video_source() {
    let cfg = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
    let params = WbcParams {
        cmd_vx: 1.50,
        total_time_s: 3.0,
        burn_in_s: 0.5,
        gait_type_override: Some(GaitType::Bound),
        duty_factor_override: Some(0.35),
        gait_cycle_period_override: Some(0.18),
        bound_trim_reference: Some((100.0, 10.0)),
        bound_trim_thrust_scale_override: Some(1.0),
        yaw_pd_gain_override: Some((10.0, 1.0)),
        adaptive_cycle_period: Some(AdaptivePeriodConfig {
            gain: 0.10,
            update_interval_s: 1.0,
            min_period_s: 0.14,
            max_period_s: 0.26,
        }),
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
            roll_pitch_weight_override: None, bound_fore_aft_placement_gain_override: None, roll_rate_weight_override: None, bound_pitch_placement_gain_override: None, bound_pitch_placement_dc_tau_override: None, bound_tabulated_reference_csv: None, bound_prescribed_footholds_override: None,
        }),
        ..WbcParams::forward_walk_misa_wbc(wbc::Formulation::ForceSpace, cfg)
    };
    let Some(samples) = run_wbc_sim(params) else { return };
    report_walk_summary(
        "bound duty_factor=0.35, thrust_scale=1.0 video-capture source (T=0.18, bound_trim=(100,10), yaw_pd=(10,1), PLL, vx=1.50 -- ceiling-matching)",
        &samples, 1.50,
    );
}

/// Per-`window_s`-second breakdown (vx / planar_speed / peak_pitch /
/// peak_roll / min_z / contact_phase_mismatch), for spotting WHEN
/// within one long run a configuration destabilizes -- `report_walk_
/// summary`'s single aggregate over the whole post-burn-in window
/// can't distinguish "stable throughout" from "fine at first, then
/// drifts apart partway through" (exactly what Sec.5br's `duty=0.35,
/// thrust_scale=1.0, cmd_vx=1.50` video-capture probe found: stable
/// over a 3.0s run, degraded when extended to 4.5s).
fn report_time_windowed_summary(label: &str, samples: &[WbcSample], window_s: f64) {
    let t_max = samples.last().map(|s| s.t).unwrap_or(0.0);
    eprintln!("\n=== {label} (time-windowed, {window_s:.1}s buckets) ===");
    let mut window_start = 0.0;
    while window_start < t_max {
        let window_end = window_start + window_s;
        let window: Vec<&WbcSample> =
            samples.iter().filter(|s| s.t >= window_start && s.t < window_end).collect();
        if window.is_empty() {
            window_start += window_s;
            continue;
        }
        let peak_pitch = window.iter().map(|s| s.pitch.abs()).fold(0.0_f64, f64::max);
        let peak_roll = window.iter().map(|s| s.roll.abs()).fold(0.0_f64, f64::max);
        let min_z = window.iter().map(|s| s.body_z).fold(f64::INFINITY, f64::min);
        let max_z = window.iter().map(|s| s.body_z).fold(f64::NEG_INFINITY, f64::max);
        let z_range = max_z - min_z; // vertical excursion -- bounce amplitude / air time proxy
        let x0 = window.first().unwrap().body_x;
        let x1 = window.last().unwrap().body_x;
        let y0 = window.first().unwrap().body_y;
        let y1 = window.last().unwrap().body_y;
        let t0 = window.first().unwrap().t;
        let t1 = window.last().unwrap().t;
        let vx = (x1 - x0) / (t1 - t0).max(1e-6);
        let planar = ((x1 - x0).powi(2) + (y1 - y0).powi(2)).sqrt() / (t1 - t0).max(1e-6);
        let mismatch_pct =
            100.0 * window.iter().filter(|s| s.contact_phase_mismatch).count() as f64 / window.len() as f64;
        let has_nan = window.iter().any(|s| !s.body_x.is_finite() || !s.pitch.is_finite());
        eprintln!(
            "[{window_start:>4.1}-{window_end:<4.1}s] vx={vx:>6.3} planar={planar:>6.3} \
             peak_pitch={peak_pitch:>6.3}rad peak_roll={peak_roll:>6.3}rad min_z={min_z:>5.3}m \
             z_range={z_range:>5.3}m mismatch={mismatch_pct:>5.1}% finite={}",
            !has_nan,
        );
        window_start += window_s;
    }
}

/// Sec.5br's `duty=0.35, thrust_scale=1.0, cmd_vx=1.50` config matched
/// `duty=0.50`'s ~0.99 m/s ceiling over a 3.0s run, but one probe
/// extended to 4.5s came back degraded (`meas_vx` 0.985 -> 0.787,
/// `peak_pitch` 0.112 -> 0.349 rad) -- unclear if that's a slow drift
/// toward instability or a one-off. Runs this exact config for 10s
/// (well past both probes) with `report_time_windowed_summary` to see
/// WHEN (if ever) it destabilizes, not just whether the whole-run
/// aggregate looks bad.
#[test]
#[ignore = "exploratory stress test — run with --ignored"]
fn go2_wbc_bound_flight_phase_duty035_thrust_scale_1_long_duration_stability() {
    let cfg = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
    let params = WbcParams {
        cmd_vx: 1.50,
        total_time_s: 10.0,
        burn_in_s: 0.5,
        gait_type_override: Some(GaitType::Bound),
        duty_factor_override: Some(0.35),
        gait_cycle_period_override: Some(0.18),
        bound_trim_reference: Some((100.0, 10.0)),
        bound_trim_thrust_scale_override: Some(1.0),
        yaw_pd_gain_override: Some((10.0, 1.0)),
        adaptive_cycle_period: Some(AdaptivePeriodConfig {
            gain: 0.10,
            update_interval_s: 1.0,
            min_period_s: 0.14,
            max_period_s: 0.26,
        }),
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
            roll_pitch_weight_override: None, bound_fore_aft_placement_gain_override: None, roll_rate_weight_override: None, bound_pitch_placement_gain_override: None, bound_pitch_placement_dc_tau_override: None, bound_tabulated_reference_csv: None, bound_prescribed_footholds_override: None,
        }),
        ..WbcParams::forward_walk_misa_wbc(wbc::Formulation::ForceSpace, cfg)
    };
    let Some(samples) = run_wbc_sim(params) else { return };
    report_time_windowed_summary(
        "duty=0.35, thrust_scale=1.0, cmd_vx=1.50, 10s long-duration stability probe",
        &samples, 1.0,
    );
}

/// Sec.5bs found `duty=0.35`+`thrust_scale=1.0`+`cmd_vx=1.50`
/// collapses after ~3s despite every prior sweep (all `total_time_s=
/// 3.0`) reading it as matching `duty=0.50`'s ceiling -- raising the
/// general question of whether this session's whole short-window
/// sweep methodology has been misreading transients as steady state.
/// Runs `duty=0.50`'s OWN established best config (Sec.5bp,
/// `thrust_scale=0.4`) for 10s at the same `cmd_vx=1.50` to check
/// whether the baseline itself holds up, or whether it too was only
/// ever validated over a lucky short window.
#[test]
#[ignore = "exploratory stress test — run with --ignored"]
fn go2_wbc_bound_duty050_baseline_long_duration_stability() {
    let cfg = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
    let params = WbcParams {
        cmd_vx: 1.50,
        total_time_s: 10.0,
        burn_in_s: 0.5,
        gait_type_override: Some(GaitType::Bound),
        gait_cycle_period_override: Some(0.18),
        bound_trim_reference: Some((100.0, 10.0)),
        bound_trim_thrust_scale_override: Some(0.4),
        yaw_pd_gain_override: Some((10.0, 1.0)),
        adaptive_cycle_period: Some(AdaptivePeriodConfig {
            gain: 0.10,
            update_interval_s: 1.0,
            min_period_s: 0.14,
            max_period_s: 0.26,
        }),
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
            roll_pitch_weight_override: None, bound_fore_aft_placement_gain_override: None, roll_rate_weight_override: None, bound_pitch_placement_gain_override: None, bound_pitch_placement_dc_tau_override: None, bound_tabulated_reference_csv: None, bound_prescribed_footholds_override: None,
        }),
        ..WbcParams::forward_walk_misa_wbc(wbc::Formulation::ForceSpace, cfg)
    };
    let Some(samples) = run_wbc_sim(params) else { return };
    report_time_windowed_summary(
        "duty=0.50 (baseline), thrust_scale=0.4, cmd_vx=1.50, 10s long-duration stability probe",
        &samples, 1.0,
    );
}

/// Sec.5bs's collapse hypothesis: every established Bound config this
/// session (Sec.5ao on) sets `capture_point_gain_override: Some(0.0)`
/// -- disabling foot-placement-based velocity feedback entirely, to
/// isolate Bound-specific tuning from Trot-derived confounders. That
/// leaves velocity regulation to the periodic `F_x` trim (designed
/// purely to cancel pitch torque, not to track velocity) plus the
/// timing-only PLL -- neither of which corrects foot placement itself.
/// Classic bounding/galloping literature (Raibert 1986's three-part
/// hopping control; Di Carlo et al. 2018's convex-MPC Cheetah 3
/// bounding; Park/Wensing/Kim 2017's impulse scaling) all rely on a
/// real-time foot-placement correction (`x_foot ~ v̄·T_st/2 +
/// k·(v-v_des)`) as the primary velocity-error corrector, precisely
/// because during flight there's no ground force to lean on -- landing
/// spot is the only lever before touchdown. Re-enables the WBC's own
/// default capture-point gain (`DEFAULT_CAPTURE_POINT_GAIN_S=0.05`,
/// quadruped-gait's `mpc_controller.rs`) for exactly the config that
/// collapsed at t=3-4s (`duty=0.35`, `thrust_scale=1.0`, cmd_vx=1.50)
/// to see whether restoring foot-placement feedback prevents it.
#[test]
#[ignore = "exploratory stress test — run with --ignored"]
fn go2_wbc_bound_flight_phase_duty035_capture_point_reenabled_stability() {
    let cfg = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
    let params = WbcParams {
        cmd_vx: 1.50,
        total_time_s: 10.0,
        burn_in_s: 0.5,
        gait_type_override: Some(GaitType::Bound),
        duty_factor_override: Some(0.35),
        gait_cycle_period_override: Some(0.18),
        bound_trim_reference: Some((100.0, 10.0)),
        bound_trim_thrust_scale_override: Some(1.0),
        yaw_pd_gain_override: Some((10.0, 1.0)),
        adaptive_cycle_period: Some(AdaptivePeriodConfig {
            gain: 0.10,
            update_interval_s: 1.0,
            min_period_s: 0.14,
            max_period_s: 0.26,
        }),
        full_centroidal: Some(FullCentroidalOpts {
            legged_control_parity: true,
            use_mpc_predicted_footstep: false,
            dynamic_joint_q_reference: false,
            mpc_override: None,
            task_space_joint_vel_weight: None,
            true_centroidal_coupling: false,
            capture_point_gain_override: Some(0.05),
            base_pos_xy_weight_override: None,
            max_normal_force_override: None,
            roll_pitch_weight_override: None, bound_fore_aft_placement_gain_override: None, roll_rate_weight_override: None, bound_pitch_placement_gain_override: None, bound_pitch_placement_dc_tau_override: None, bound_tabulated_reference_csv: None, bound_prescribed_footholds_override: None,
        }),
        ..WbcParams::forward_walk_misa_wbc(wbc::Formulation::ForceSpace, cfg)
    };
    let Some(samples) = run_wbc_sim(params) else { return };
    report_time_windowed_summary(
        "duty=0.35, thrust_scale=1.0, cmd_vx=1.50, capture_point_gain=0.05 (re-enabled), 10s stability probe",
        &samples, 1.0,
    );
}

/// Sec.5bu's diagnosis: the collapse (`duty=0.35`, `thrust_scale=1.0`,
/// cmd_vx=1.50) is a phase-timing divergence that runs away in
/// ~0.6s (t=3.04-3.68s, pitch_meas +0.087->-0.239rad), faster than
/// the PLL's `update_interval_s=1.0` (tuned for `duty=0.50`, Sec.5bl)
/// can correct -- it only re-averages `cycle_period_s` once per
/// second. Sweeps `update_interval_s` down (1.0/0.5/0.3/0.2) at the
/// exact collapsing config, 10s each, with `report_time_windowed_
/// summary` to see whether a faster PLL response prevents the runaway
/// instead of merely reacting to it after the fact.
#[test]
#[ignore = "exploratory stress test — run with --ignored"]
fn go2_wbc_bound_flight_phase_duty035_pll_interval_stability_sweep() {
    for update_interval_s in [1.0, 0.5, 0.3, 0.2] {
        let cfg = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
        let params = WbcParams {
            cmd_vx: 1.50,
            total_time_s: 10.0,
            burn_in_s: 0.5,
            gait_type_override: Some(GaitType::Bound),
            duty_factor_override: Some(0.35),
            gait_cycle_period_override: Some(0.18),
            bound_trim_reference: Some((100.0, 10.0)),
            bound_trim_thrust_scale_override: Some(1.0),
            yaw_pd_gain_override: Some((10.0, 1.0)),
            adaptive_cycle_period: Some(AdaptivePeriodConfig {
                gain: 0.10,
                update_interval_s,
                min_period_s: 0.14,
                max_period_s: 0.26,
            }),
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
                roll_pitch_weight_override: None, bound_fore_aft_placement_gain_override: None, roll_rate_weight_override: None, bound_pitch_placement_gain_override: None, bound_pitch_placement_dc_tau_override: None, bound_tabulated_reference_csv: None, bound_prescribed_footholds_override: None,
            }),
            ..WbcParams::forward_walk_misa_wbc(wbc::Formulation::ForceSpace, cfg)
        };
        if let Some(samples) = run_wbc_sim(params) {
            report_time_windowed_summary(
                &format!("duty=0.35, thrust_scale=1.0, cmd_vx=1.50, PLL update_interval_s={update_interval_s:.1}, 10s"),
                &samples, 1.0,
            );
        }
    }
}

/// Sec.5bv found `update_interval_s` shortening monotonically improves
/// stability down to `0.2` (9-window avg vx 1.092 m/s, never negative)
/// -- this pushes further (0.15/0.10/0.05) to find where the trend
/// stops helping (diminishing returns, or a new instability from an
/// overly twitchy PLL overreacting to noisy per-cycle phase-error
/// samples).
#[test]
#[ignore = "exploratory stress test — run with --ignored"]
fn go2_wbc_bound_flight_phase_duty035_pll_interval_fine_sweep() {
    for update_interval_s in [0.20, 0.15, 0.10, 0.05] {
        let cfg = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
        let params = WbcParams {
            cmd_vx: 1.50,
            total_time_s: 10.0,
            burn_in_s: 0.5,
            gait_type_override: Some(GaitType::Bound),
            duty_factor_override: Some(0.35),
            gait_cycle_period_override: Some(0.18),
            bound_trim_reference: Some((100.0, 10.0)),
            bound_trim_thrust_scale_override: Some(1.0),
            yaw_pd_gain_override: Some((10.0, 1.0)),
            adaptive_cycle_period: Some(AdaptivePeriodConfig {
                gain: 0.10,
                update_interval_s,
                min_period_s: 0.14,
                max_period_s: 0.26,
            }),
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
                roll_pitch_weight_override: None, bound_fore_aft_placement_gain_override: None, roll_rate_weight_override: None, bound_pitch_placement_gain_override: None, bound_pitch_placement_dc_tau_override: None, bound_tabulated_reference_csv: None, bound_prescribed_footholds_override: None,
            }),
            ..WbcParams::forward_walk_misa_wbc(wbc::Formulation::ForceSpace, cfg)
        };
        if let Some(samples) = run_wbc_sim(params) {
            report_time_windowed_summary(
                &format!("duty=0.35, thrust_scale=1.0, cmd_vx=1.50, PLL update_interval_s={update_interval_s:.2}, 10s"),
                &samples, 1.0,
            );
        }
    }
}

/// Video-capture source for Sec.5bz's winning "leap AND keep the
/// speed" configuration: `duty_factor=0.35` (genuine flight phase),
/// `thrust_scale=1.0` (Sec.5br), `PLL update_interval_s=0.2` (Sec.5bv/
/// 5bw -- the local-optimum PLL response speed, fast enough to
/// recover from the phase-timing divergence Sec.5bu diagnosed, but not
/// so fast it resonates with the gait's own ~0.18s cycle like `0.15s`
/// did), `max_step_length_m=0.18` (Sec.5bx -- raised from Bound's
/// stock 0.12m per the user's own observation that stride length
/// hadn't been tried; the footstep planner's own ceiling at this
/// duty/cmd_vx), a tightened PLL clamp `min_period_s=0.16`/
/// `max_period_s=0.20` (Sec.5by/5bz), and NOW a smooth startup
/// transient (Sec.5c0-5c3, local doc): `cmd_vx`+`thrust_scale` ramp
/// together over 4.0s (0->2.20, 0.2->1.0) so forward speed climbs
/// near-monotonically from rest instead of snapping to full strength
/// at t=0 (the user's original complaint), plus `pll_accumulate_
/// during_ramp` so the PLL arrives at cruise already adapted instead
/// of windup-diverging from a cold start right at the ramp/cruise
/// boundary (Sec.5c3 -- without this, the ramped version collapsed at
/// t=6-7s and never recovered). Even with the full fix there's a
/// brief, shallow dip around t=4.5-6.5s (min vx=-0.023, no fall) before
/// it recovers into a sustained ~1.5 m/s cruise from t=7.5s on --
/// captured honestly, not cropped out. 14.5s run (matches the
/// `..._pll_warm` stability probe) to show the full startup, dip, and
/// recovery. Run with `WBC_WALK_CSV_OUT=<path> cargo test --release
/// --features mujoco --test wbc_walk_go2 go2_wbc_bound_flight_phase_
/// duty035_best_pattern_video_source -- --ignored --nocapture`.
#[test]
#[ignore = "exploratory stress test — run with --ignored; also the WBC_WALK_CSV_OUT video-capture source for Sec.5c3 (duty=0.35, thrust_scale=1.0 ramped w/ cmd_vx over 4.0s, PLL warm-start during ramp, max_step_length_m=0.18 -- best pattern, smooth start + recovers to stable cruise)"]
fn go2_wbc_bound_flight_phase_duty035_best_pattern_video_source() {
    let cfg = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
    let params = WbcParams {
        cmd_vx: 2.20,
        cmd_vx_ramp_s: Some(4.0),
        thrust_scale_ramp_start: Some(0.2),
        pll_accumulate_during_ramp: true,
        total_time_s: 14.5,
        burn_in_s: 0.5,
        gait_type_override: Some(GaitType::Bound),
        duty_factor_override: Some(0.35),
        gait_cycle_period_override: Some(0.18),
        max_step_length_override: Some(0.18),
        bound_trim_reference: Some((100.0, 10.0)),
        bound_trim_thrust_scale_override: Some(1.0),
        yaw_pd_gain_override: Some((10.0, 1.0)),
        adaptive_cycle_period: Some(AdaptivePeriodConfig {
            gain: 0.10,
            update_interval_s: 0.2,
            min_period_s: 0.16,
            max_period_s: 0.20,
        }),
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
            roll_pitch_weight_override: None, bound_fore_aft_placement_gain_override: None, roll_rate_weight_override: None, bound_pitch_placement_gain_override: None, bound_pitch_placement_dc_tau_override: None, bound_tabulated_reference_csv: None, bound_prescribed_footholds_override: None,
        }),
        ..WbcParams::forward_walk_misa_wbc(wbc::Formulation::ForceSpace, cfg)
    };
    let Some(samples) = run_wbc_sim(params) else { return };
    report_walk_summary(
        "bound duty=0.35, thrust_scale=1.0, PLL interval=0.2, max_step_length_m=0.18, smooth 4.0s startup ramp video-capture source (T=0.18, bound_trim=(100,10), yaw_pd=(10,1), vx=2.20 -- best leap+speed pattern)",
        &samples, 2.20,
    );
    report_time_windowed_summary(
        "bound duty=0.35, thrust_scale=1.0, PLL interval=0.2, max_step_length_m=0.18, smooth startup, vx=2.20 (video source, time-windowed)",
        &samples, 1.0,
    );
}

/// Every knob swept this session (`cmd_vx`, `duty_factor`, `thrust_
/// scale`, `cycle_period_s`, PLL `update_interval_s`) left
/// `max_step_length_m` untouched at Bound's stock 0.12m. The
/// footstep-planner ceiling `v_max = max_step_length_m / (cycle_
/// period_s·duty_factor)` (this file's own recurring formula, e.g.
/// Sec.2327/3620) gives 0.12/(0.18·0.35)≈1.9 m/s at Sec.5bw's best
/// pattern -- comfortably above the ~1.3 m/s actually achieved, so
/// stride length wasn't the suspected bottleneck. Untested until now,
/// though: this sweeps `max_step_length_m` at Sec.5bw's best pattern
/// (`duty=0.35`, `thrust_scale=1.0`, PLL `update_interval_s=0.2`), at
/// an aggressive `cmd_vx=2.20` (previously untracked well at any
/// config), 10s each via `report_time_windowed_summary` to catch
/// collapse dynamics, not just a short favorable window.
#[test]
#[ignore = "exploratory stress test — run with --ignored"]
fn go2_wbc_bound_flight_phase_duty035_best_pattern_stride_length_sweep() {
    let cmd_vx = 2.20;
    for max_step_length_m in [0.12, 0.15, 0.18, 0.22, 0.26] {
        let cfg = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
        let params = WbcParams {
            cmd_vx,
            total_time_s: 10.0,
            burn_in_s: 0.5,
            gait_type_override: Some(GaitType::Bound),
            duty_factor_override: Some(0.35),
            gait_cycle_period_override: Some(0.18),
            max_step_length_override: Some(max_step_length_m),
            bound_trim_reference: Some((100.0, 10.0)),
            bound_trim_thrust_scale_override: Some(1.0),
            yaw_pd_gain_override: Some((10.0, 1.0)),
            adaptive_cycle_period: Some(AdaptivePeriodConfig {
                gain: 0.10,
                update_interval_s: 0.2,
                min_period_s: 0.14,
                max_period_s: 0.26,
            }),
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
                roll_pitch_weight_override: None, bound_fore_aft_placement_gain_override: None, roll_rate_weight_override: None, bound_pitch_placement_gain_override: None, bound_pitch_placement_dc_tau_override: None, bound_tabulated_reference_csv: None, bound_prescribed_footholds_override: None,
            }),
            ..WbcParams::forward_walk_misa_wbc(wbc::Formulation::ForceSpace, cfg)
        };
        if let Some(samples) = run_wbc_sim(params) {
            report_time_windowed_summary(
                &format!("max_step_length_m={max_step_length_m:.2} (duty=0.35, thrust_scale=1.0, PLL=0.2, cmd_vx=2.20)"),
                &samples, 1.0,
            );
        }
    }
}

/// Sec.5by's diagnosis: the t=7s+ collapse (new best pattern -- `duty=
/// 0.35`, `thrust_scale=1.0`, `max_step_length_m=0.18`, cmd_vx=2.20)
/// is PLL integral windup, not a sudden fast divergence -- `mean_
/// error` stays consistently one-signed for 8+ seconds, so `cycle_
/// period_s` walks monotonically from 0.198 up to 0.215+ (well inside
/// the `min_period_s=0.14`/`max_period_s=0.26` clamp, but far outside
/// this session's known-good ~0.18 sweet spot). Tightens the clamp to
/// `0.16`/`0.20` (vs the established `0.14`/`0.26`) at the same
/// config, 15s (longer than Sec.5by's 8.5s probe) to see whether
/// bounding the drift closer to the tuned period prevents the walk-
/// away-then-collapse pattern.
#[test]
#[ignore = "exploratory stress test — run with --ignored"]
fn go2_wbc_bound_flight_phase_duty035_best_pattern_tight_pll_clamp_stability() {
    let cfg = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
    let params = WbcParams {
        cmd_vx: 2.20,
        total_time_s: 15.0,
        burn_in_s: 0.5,
        gait_type_override: Some(GaitType::Bound),
        duty_factor_override: Some(0.35),
        gait_cycle_period_override: Some(0.18),
        max_step_length_override: Some(0.18),
        bound_trim_reference: Some((100.0, 10.0)),
        bound_trim_thrust_scale_override: Some(1.0),
        yaw_pd_gain_override: Some((10.0, 1.0)),
        adaptive_cycle_period: Some(AdaptivePeriodConfig {
            gain: 0.10,
            update_interval_s: 0.2,
            min_period_s: 0.16,
            max_period_s: 0.20,
        }),
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
            roll_pitch_weight_override: None, bound_fore_aft_placement_gain_override: None, roll_rate_weight_override: None, bound_pitch_placement_gain_override: None, bound_pitch_placement_dc_tau_override: None, bound_tabulated_reference_csv: None, bound_prescribed_footholds_override: None,
        }),
        ..WbcParams::forward_walk_misa_wbc(wbc::Formulation::ForceSpace, cfg)
    };
    let Some(samples) = run_wbc_sim(params) else { return };
    report_time_windowed_summary(
        "duty=0.35, thrust_scale=1.0, max_step_length_m=0.18, cmd_vx=2.20, PLL clamp tightened to 0.16-0.20, 15s",
        &samples, 1.0,
    );
}

/// User observation: the best pattern's startup (t=0-2s of Sec.5bz's
/// video -- vx=0.026/0.455, peak_pitch=0.207/0.216rad, both far off
/// the ~1.7 m/s / ~0.1rad steady state) looks rough because `cmd_vx`,
/// `max_step_length_m`, and `cycle_period_s` all snap to their full
/// target values at `burn_in_s` while the body is still near a dead
/// stop. Ramps all three in lockstep over the same `cmd_vx_ramp_s`
/// window (2.0s) via the new `max_step_length_ramp_start_m`/`cycle_
/// period_ramp_start_s` knobs: stride grows 0.06->0.18m, cadence
/// quickens 0.24->0.18s, alongside cmd_vx's existing 0->2.20 ramp --
/// mimicking how real quadrupeds quicken cadence as they accelerate
/// (trot-to-gallop transitions raise stride frequency with speed).
/// 0.5s windows (finer
/// than the 1.0s stability probes) to inspect the startup transient
/// closely -- forward position/velocity should now climb smoothly
/// instead of the sharp jump Sec.5bz's abrupt start showed.
#[test]
#[ignore = "exploratory stress test — run with --ignored"]
fn go2_wbc_bound_flight_phase_duty035_best_pattern_smooth_startup() {
    let cfg = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
    let params = WbcParams {
        cmd_vx: 2.20,
        cmd_vx_ramp_s: Some(2.0),
        max_step_length_ramp_start_m: Some(0.06),
        cycle_period_ramp_start_s: Some(0.24),
        total_time_s: 12.5,
        burn_in_s: 0.5,
        gait_type_override: Some(GaitType::Bound),
        duty_factor_override: Some(0.35),
        gait_cycle_period_override: Some(0.18),
        max_step_length_override: Some(0.18),
        bound_trim_reference: Some((100.0, 10.0)),
        bound_trim_thrust_scale_override: Some(1.0),
        yaw_pd_gain_override: Some((10.0, 1.0)),
        adaptive_cycle_period: Some(AdaptivePeriodConfig {
            gain: 0.10,
            update_interval_s: 0.2,
            min_period_s: 0.16,
            max_period_s: 0.20,
        }),
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
            roll_pitch_weight_override: None, bound_fore_aft_placement_gain_override: None, roll_rate_weight_override: None, bound_pitch_placement_gain_override: None, bound_pitch_placement_dc_tau_override: None, bound_tabulated_reference_csv: None, bound_prescribed_footholds_override: None,
        }),
        ..WbcParams::forward_walk_misa_wbc(wbc::Formulation::ForceSpace, cfg)
    };
    let Some(samples) = run_wbc_sim(params) else { return };
    report_time_windowed_summary(
        "duty=0.35, thrust_scale=1.0, max_step_length_m=0.18, cmd_vx=2.20, smooth startup ramp (cmd_vx/stride/period, 2.0s)",
        &samples, 0.5,
    );
}

/// Isolation check: `go2_wbc_bound_flight_phase_duty035_best_pattern_
/// smooth_startup` (ramping cmd_vx+stride+period together over 2.0s)
/// made the WHOLE 12.5s run worse, not just the startup -- never
/// reaching Sec.5bz's abrupt-start ~1.7 m/s steady state at all.
/// Isolates which knob is responsible: `cmd_vx_ramp_s` alone (no
/// stride/period ramp), same 2.0s duration, same 0.5s windowed report.
#[test]
#[ignore = "exploratory stress test — run with --ignored"]
fn go2_wbc_bound_flight_phase_duty035_best_pattern_cmd_vx_ramp_only() {
    let cfg = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
    let params = WbcParams {
        cmd_vx: 2.20,
        cmd_vx_ramp_s: Some(2.0),
        total_time_s: 12.5,
        burn_in_s: 0.5,
        gait_type_override: Some(GaitType::Bound),
        duty_factor_override: Some(0.35),
        gait_cycle_period_override: Some(0.18),
        max_step_length_override: Some(0.18),
        bound_trim_reference: Some((100.0, 10.0)),
        bound_trim_thrust_scale_override: Some(1.0),
        yaw_pd_gain_override: Some((10.0, 1.0)),
        adaptive_cycle_period: Some(AdaptivePeriodConfig {
            gain: 0.10,
            update_interval_s: 0.2,
            min_period_s: 0.16,
            max_period_s: 0.20,
        }),
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
            roll_pitch_weight_override: None, bound_fore_aft_placement_gain_override: None, roll_rate_weight_override: None, bound_pitch_placement_gain_override: None, bound_pitch_placement_dc_tau_override: None, bound_tabulated_reference_csv: None, bound_prescribed_footholds_override: None,
        }),
        ..WbcParams::forward_walk_misa_wbc(wbc::Formulation::ForceSpace, cfg)
    };
    let Some(samples) = run_wbc_sim(params) else { return };
    report_time_windowed_summary(
        "duty=0.35, thrust_scale=1.0, max_step_length_m=0.18, cmd_vx=2.20, cmd_vx-only ramp (2.0s, no stride/period ramp)",
        &samples, 0.5,
    );
}

/// User question, after confirming the best-pattern video source
/// commands `cmd_vx` as a step function (no ramp): what happens with
/// a much gentler `cmd_vx_ramp_s=10.0` (vs the 2.0s ramp that caused
/// an outright fall in `..._cmd_vx_ramp_only`)? Same stride/period
/// held fixed at target from t=0 (isolating the cmd_vx-ramp effect
/// alone, per that same test's finding that partial ramps mismatch
/// against the cmd_vx-independent `thrust_scale`-based trim). 20s
/// total (10s ramp + 10s post-ramp cruise) to see both the ramp itself
/// and whether it settles into Sec.5bz's stable ~1.7 m/s cruise
/// afterward.
#[test]
#[ignore = "exploratory stress test — run with --ignored"]
fn go2_wbc_bound_flight_phase_duty035_best_pattern_cmd_vx_ramp_10s() {
    let cfg = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
    let params = WbcParams {
        cmd_vx: 2.20,
        cmd_vx_ramp_s: Some(10.0),
        total_time_s: 20.5,
        burn_in_s: 0.5,
        gait_type_override: Some(GaitType::Bound),
        duty_factor_override: Some(0.35),
        gait_cycle_period_override: Some(0.18),
        max_step_length_override: Some(0.18),
        bound_trim_reference: Some((100.0, 10.0)),
        bound_trim_thrust_scale_override: Some(1.0),
        yaw_pd_gain_override: Some((10.0, 1.0)),
        adaptive_cycle_period: Some(AdaptivePeriodConfig {
            gain: 0.10,
            update_interval_s: 0.2,
            min_period_s: 0.16,
            max_period_s: 0.20,
        }),
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
            roll_pitch_weight_override: None, bound_fore_aft_placement_gain_override: None, roll_rate_weight_override: None, bound_pitch_placement_gain_override: None, bound_pitch_placement_dc_tau_override: None, bound_tabulated_reference_csv: None, bound_prescribed_footholds_override: None,
        }),
        ..WbcParams::forward_walk_misa_wbc(wbc::Formulation::ForceSpace, cfg)
    };
    let Some(samples) = run_wbc_sim(params) else { return };
    report_time_windowed_summary(
        "duty=0.35, thrust_scale=1.0, max_step_length_m=0.18, cmd_vx=2.20, cmd_vx-only ramp (10.0s, no stride/period ramp)",
        &samples, 1.0,
    );
}

/// Sec.5c0's diagnosis: ramping `cmd_vx`/stride/period while
/// `thrust_scale` stays pinned at its full target from t=0 fails
/// because the trim's periodic `F_x`/pitch schedule is `thrust_scale`-
/// sized alone, entirely independent of `cmd_vx` -- it "kicks" a
/// near-stationary body with the same force magnitude it would use at
/// cruising speed. This ramps `thrust_scale` itself (0.2->1.0) in
/// lockstep with `cmd_vx` (0->2.20) via the new `thrust_scale_ramp_
/// start`, holding stride/period fixed at target throughout (isolating
/// this one lever, since combining it with the stride/period ramps
/// already failed twice). A low `thrust_scale` DOES raise `theta_peak`
/// (the trim's own math: less cancellation -> more reference pitch to
/// track), but that's a PD-tracked reference, not a raw force kick --
/// the hypothesis is that gentling the force (not the reference) is
/// what actually matters for a body starting near rest.
#[test]
#[ignore = "exploratory stress test — run with --ignored"]
fn go2_wbc_bound_flight_phase_duty035_best_pattern_thrust_scale_ramp() {
    let cfg = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
    let params = WbcParams {
        cmd_vx: 2.20,
        cmd_vx_ramp_s: Some(2.0),
        thrust_scale_ramp_start: Some(0.2),
        total_time_s: 12.5,
        burn_in_s: 0.5,
        gait_type_override: Some(GaitType::Bound),
        duty_factor_override: Some(0.35),
        gait_cycle_period_override: Some(0.18),
        max_step_length_override: Some(0.18),
        bound_trim_reference: Some((100.0, 10.0)),
        bound_trim_thrust_scale_override: Some(1.0),
        yaw_pd_gain_override: Some((10.0, 1.0)),
        adaptive_cycle_period: Some(AdaptivePeriodConfig {
            gain: 0.10,
            update_interval_s: 0.2,
            min_period_s: 0.16,
            max_period_s: 0.20,
        }),
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
            roll_pitch_weight_override: None, bound_fore_aft_placement_gain_override: None, roll_rate_weight_override: None, bound_pitch_placement_gain_override: None, bound_pitch_placement_dc_tau_override: None, bound_tabulated_reference_csv: None, bound_prescribed_footholds_override: None,
        }),
        ..WbcParams::forward_walk_misa_wbc(wbc::Formulation::ForceSpace, cfg)
    };
    let Some(samples) = run_wbc_sim(params) else { return };
    report_time_windowed_summary(
        "duty=0.35, max_step_length_m=0.18, cmd_vx=2.20, cmd_vx+thrust_scale ramp together (2.0s, stride/period fixed)",
        &samples, 0.5,
    );
}

/// (a) of the 3-option follow-up to Sec.5c1: the new instability
/// started right at t=2.5s, exactly where the PLL resumes phase-error
/// accumulation (`ramp_in_progress` ends at `burn_in_s+cmd_vx_ramp_s`).
/// Adds a `post_ramp_settle_s=1.0` buffer (via the new field) so the
/// PLL stays gated off an extra second after the ramp itself ends,
/// giving the gait time to settle onto its own limit cycle before the
/// PLL starts reacting to (possibly still-transient) contact timing.
#[test]
#[ignore = "exploratory stress test — run with --ignored"]
fn go2_wbc_bound_flight_phase_duty035_best_pattern_pll_settle_buffer() {
    let cfg = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
    let params = WbcParams {
        cmd_vx: 2.20,
        cmd_vx_ramp_s: Some(2.0),
        thrust_scale_ramp_start: Some(0.2),
        post_ramp_settle_s: Some(1.0),
        total_time_s: 12.5,
        burn_in_s: 0.5,
        gait_type_override: Some(GaitType::Bound),
        duty_factor_override: Some(0.35),
        gait_cycle_period_override: Some(0.18),
        max_step_length_override: Some(0.18),
        bound_trim_reference: Some((100.0, 10.0)),
        bound_trim_thrust_scale_override: Some(1.0),
        yaw_pd_gain_override: Some((10.0, 1.0)),
        adaptive_cycle_period: Some(AdaptivePeriodConfig {
            gain: 0.10,
            update_interval_s: 0.2,
            min_period_s: 0.16,
            max_period_s: 0.20,
        }),
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
            roll_pitch_weight_override: None, bound_fore_aft_placement_gain_override: None, roll_rate_weight_override: None, bound_pitch_placement_gain_override: None, bound_pitch_placement_dc_tau_override: None, bound_tabulated_reference_csv: None, bound_prescribed_footholds_override: None,
        }),
        ..WbcParams::forward_walk_misa_wbc(wbc::Formulation::ForceSpace, cfg)
    };
    let Some(samples) = run_wbc_sim(params) else { return };
    report_time_windowed_summary(
        "duty=0.35, cmd_vx=2.20, cmd_vx+thrust_scale ramp (2.0s) + 1.0s post-ramp PLL settle buffer",
        &samples, 0.5,
    );
}

/// (b) of the 3-option follow-up to Sec.5c1: (a) (delaying PLL resume)
/// made things worse (Sec.5c2). Tries the other direction instead --
/// stretching the `cmd_vx`+`thrust_scale` ramp itself from 2.0s to
/// 4.0s, so the transition is gentler throughout rather than ending
/// abruptly at a fixed 2.0s mark.
#[test]
#[ignore = "exploratory stress test — run with --ignored"]
fn go2_wbc_bound_flight_phase_duty035_best_pattern_longer_ramp() {
    let cfg = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
    let params = WbcParams {
        cmd_vx: 2.20,
        cmd_vx_ramp_s: Some(4.0),
        thrust_scale_ramp_start: Some(0.2),
        total_time_s: 14.5,
        burn_in_s: 0.5,
        gait_type_override: Some(GaitType::Bound),
        duty_factor_override: Some(0.35),
        gait_cycle_period_override: Some(0.18),
        max_step_length_override: Some(0.18),
        bound_trim_reference: Some((100.0, 10.0)),
        bound_trim_thrust_scale_override: Some(1.0),
        yaw_pd_gain_override: Some((10.0, 1.0)),
        adaptive_cycle_period: Some(AdaptivePeriodConfig {
            gain: 0.10,
            update_interval_s: 0.2,
            min_period_s: 0.16,
            max_period_s: 0.20,
        }),
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
            roll_pitch_weight_override: None, bound_fore_aft_placement_gain_override: None, roll_rate_weight_override: None, bound_pitch_placement_gain_override: None, bound_pitch_placement_dc_tau_override: None, bound_tabulated_reference_csv: None, bound_prescribed_footholds_override: None,
        }),
        ..WbcParams::forward_walk_misa_wbc(wbc::Formulation::ForceSpace, cfg)
    };
    let Some(samples) = run_wbc_sim(params) else { return };
    report_time_windowed_summary(
        "duty=0.35, cmd_vx=2.20, cmd_vx+thrust_scale ramp stretched to 4.0s",
        &samples, 0.5,
    );
}

/// Fine-grained trace on `..._longer_ramp`'s t=6-8s collapse found the
/// SAME mechanism as Sec.5by: `cycle_period_s` drifts monotonically
/// (0.179->0.198) once the PLL resumes accumulating at the ramp's end
/// (t=4.5s here), just delayed relative to Sec.5by's t=0.5s-start
/// collapse because the ramp postpones when PLL activity begins. The
/// existing 0.16/0.20 clamp (Sec.5bz's fix for the no-ramp case)
/// isn't tight enough here -- narrows it further to 0.17/0.19 to see
/// if a tighter band prevents the drift from ever reaching the
/// runaway zone.
#[test]
#[ignore = "exploratory stress test — run with --ignored"]
fn go2_wbc_bound_flight_phase_duty035_best_pattern_longer_ramp_tighter_clamp() {
    let cfg = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
    let params = WbcParams {
        cmd_vx: 2.20,
        cmd_vx_ramp_s: Some(4.0),
        thrust_scale_ramp_start: Some(0.2),
        total_time_s: 14.5,
        burn_in_s: 0.5,
        gait_type_override: Some(GaitType::Bound),
        duty_factor_override: Some(0.35),
        gait_cycle_period_override: Some(0.18),
        max_step_length_override: Some(0.18),
        bound_trim_reference: Some((100.0, 10.0)),
        bound_trim_thrust_scale_override: Some(1.0),
        yaw_pd_gain_override: Some((10.0, 1.0)),
        adaptive_cycle_period: Some(AdaptivePeriodConfig {
            gain: 0.10,
            update_interval_s: 0.2,
            min_period_s: 0.17,
            max_period_s: 0.19,
        }),
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
            roll_pitch_weight_override: None, bound_fore_aft_placement_gain_override: None, roll_rate_weight_override: None, bound_pitch_placement_gain_override: None, bound_pitch_placement_dc_tau_override: None, bound_tabulated_reference_csv: None, bound_prescribed_footholds_override: None,
        }),
        ..WbcParams::forward_walk_misa_wbc(wbc::Formulation::ForceSpace, cfg)
    };
    let Some(samples) = run_wbc_sim(params) else { return };
    report_time_windowed_summary(
        "duty=0.35, cmd_vx=2.20, 4.0s ramp, PLL clamp tightened further to 0.17-0.19",
        &samples, 0.5,
    );
}

/// Sec.5c3's finding: narrowing the PLL clamp further (0.17-0.19)
/// left the 4.0s-ramp best pattern's t=6-8s collapse completely
/// unchanged -- ruling out clamp width as the mechanism here (unlike
/// Sec.5by/5bz's no-ramp case). Tries `pll_accumulate_during_ramp`
/// instead: let the PLL accumulate throughout the 4.0s ramp so
/// `cycle_period_s` arrives at cruise already adapted to the settling
/// gait, rather than starting a blank-slate accumulator right at the
/// ramp/cruise boundary.
#[test]
#[ignore = "exploratory stress test — run with --ignored"]
fn go2_wbc_bound_flight_phase_duty035_best_pattern_longer_ramp_pll_warm() {
    let cfg = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
    let params = WbcParams {
        cmd_vx: 2.20,
        cmd_vx_ramp_s: Some(4.0),
        thrust_scale_ramp_start: Some(0.2),
        pll_accumulate_during_ramp: true,
        total_time_s: 14.5,
        burn_in_s: 0.5,
        gait_type_override: Some(GaitType::Bound),
        duty_factor_override: Some(0.35),
        gait_cycle_period_override: Some(0.18),
        max_step_length_override: Some(0.18),
        bound_trim_reference: Some((100.0, 10.0)),
        bound_trim_thrust_scale_override: Some(1.0),
        yaw_pd_gain_override: Some((10.0, 1.0)),
        adaptive_cycle_period: Some(AdaptivePeriodConfig {
            gain: 0.10,
            update_interval_s: 0.2,
            min_period_s: 0.16,
            max_period_s: 0.20,
        }),
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
            roll_pitch_weight_override: None, bound_fore_aft_placement_gain_override: None, roll_rate_weight_override: None, bound_pitch_placement_gain_override: None, bound_pitch_placement_dc_tau_override: None, bound_tabulated_reference_csv: None, bound_prescribed_footholds_override: None,
        }),
        ..WbcParams::forward_walk_misa_wbc(wbc::Formulation::ForceSpace, cfg)
    };
    let Some(samples) = run_wbc_sim(params) else { return };
    report_time_windowed_summary(
        "duty=0.35, cmd_vx=2.20, 4.0s ramp, PLL accumulates during ramp (warm start)",
        &samples, 0.5,
    );
}

/// Sec.5c4's fix: `pll_accumulate_during_ramp` (Sec.5c3) left T
/// converged to ~0.167 during the ramp (too short for cruise),
/// forcing a slow 0.167->0.20 re-climb across the full 0.16/0.20
/// clamp that WAS the residual t=4.5-6.5s dip. Keeps warm-start but
/// tightens+centers the clamp on the tuned 0.18 (`min_period_s=0.175`/
/// `max_period_s=0.185`) so the PLL physically can't wander to 0.167
/// -- T stays pinned near 0.18 through the ramp/cruise transition,
/// ideally shrinking the re-climb distance (and the dip) to nothing.
#[test]
#[ignore = "exploratory stress test — run with --ignored"]
fn go2_wbc_bound_flight_phase_duty035_best_pattern_warm_centered_clamp() {
    let cfg = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
    let params = WbcParams {
        cmd_vx: 2.20,
        cmd_vx_ramp_s: Some(4.0),
        thrust_scale_ramp_start: Some(0.2),
        pll_accumulate_during_ramp: true,
        total_time_s: 14.5,
        burn_in_s: 0.5,
        gait_type_override: Some(GaitType::Bound),
        duty_factor_override: Some(0.35),
        gait_cycle_period_override: Some(0.18),
        max_step_length_override: Some(0.18),
        bound_trim_reference: Some((100.0, 10.0)),
        bound_trim_thrust_scale_override: Some(1.0),
        yaw_pd_gain_override: Some((10.0, 1.0)),
        adaptive_cycle_period: Some(AdaptivePeriodConfig {
            gain: 0.10,
            update_interval_s: 0.2,
            min_period_s: 0.175,
            max_period_s: 0.185,
        }),
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
            roll_pitch_weight_override: None, bound_fore_aft_placement_gain_override: None, roll_rate_weight_override: None, bound_pitch_placement_gain_override: None, bound_pitch_placement_dc_tau_override: None, bound_tabulated_reference_csv: None, bound_prescribed_footholds_override: None,
        }),
        ..WbcParams::forward_walk_misa_wbc(wbc::Formulation::ForceSpace, cfg)
    };
    let Some(samples) = run_wbc_sim(params) else { return };
    report_time_windowed_summary(
        "duty=0.35, cmd_vx=2.20, 4.0s ramp, PLL warm-start + centered clamp 0.175-0.185",
        &samples, 0.5,
    );
}

/// Option (A) from the Sec.5c5 holistic review: a Bound-specific
/// fore-aft foot-placement feedback (Raibert running-speed regulator,
/// `x_foot += k·(v_x−v_x_des)`, x-ONLY). Sec.5c4 established that the
/// residual cruise collapse is NOT a PLL-clamp problem (it persists
/// with T frozen at 0.18) -- it's the missing velocity-error foot
/// placement the literature says is the primary bounding stabilizer.
/// Sweeps the new gain at the CONSTANT-cmd_vx=2.20 best pattern (no
/// startup ramp -- isolating the cruise-stability effect from the
/// startup transient), 15s each, to see whether closing the fore-aft
/// foot-placement loop keeps the cruise from collapsing.
#[test]
#[ignore = "exploratory stress test — run with --ignored"]
fn go2_wbc_bound_flight_phase_duty035_foot_placement_sweep() {
    for gain in [0.0, 0.02, 0.05, 0.10, 0.15] {
        let cfg = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
        let params = WbcParams {
            cmd_vx: 2.20,
            total_time_s: 15.0,
            burn_in_s: 0.5,
            gait_type_override: Some(GaitType::Bound),
            duty_factor_override: Some(0.35),
            gait_cycle_period_override: Some(0.18),
            max_step_length_override: Some(0.18),
            bound_trim_reference: Some((100.0, 10.0)),
            bound_trim_thrust_scale_override: Some(1.0),
            yaw_pd_gain_override: Some((10.0, 1.0)),
            adaptive_cycle_period: Some(AdaptivePeriodConfig {
                gain: 0.10,
                update_interval_s: 0.2,
                min_period_s: 0.16,
                max_period_s: 0.20,
            }),
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
                bound_fore_aft_placement_gain_override: Some(gain),
                roll_rate_weight_override: None,
                bound_pitch_placement_gain_override: None,
                bound_pitch_placement_dc_tau_override: None,
                bound_tabulated_reference_csv: None,
                bound_prescribed_footholds_override: None,
            }),
            ..WbcParams::forward_walk_misa_wbc(wbc::Formulation::ForceSpace, cfg)
        };
        if let Some(samples) = run_wbc_sim(params) {
            report_time_windowed_summary(
                &format!("duty=0.35, cmd_vx=2.20 (constant), bound_fore_aft_placement_gain={gain:.2}, 15s"),
                &samples, 1.0,
            );
        }
    }
}

/// Option (B) from the Sec.5c5 review (Sec.5c8, local doc): instead of
/// a reactive foot-placement bolt-on (Sec.5c6/5c7, which conflicted
/// with the directly-commanded F_x), let the FullCentroidal MPC choose
/// the footstep JOINTLY with the GRF -- `mpc_optimized_footstep=true`
/// (MPC adds a foot-XY landing cost, actively deviating the swing-leg
/// plan to place the foot) + `use_mpc_predicted_footstep=true`
/// (controller uses the MPC's chosen foothold instead of open-loop
/// Raibert). The MPC's contact schedule already projects the flight
/// phase forward (`legged_control_parity`), so in principle it plans
/// footholds and forces consistently across the aerial phase. Runs the
/// constant-cmd=2.20 best pattern (Sec.5c6's stable-cruise baseline)
/// 15s, comparing the footstep source OFF (baseline) vs ON.
#[test]
#[ignore = "exploratory stress test — run with --ignored"]
fn go2_wbc_bound_flight_phase_duty035_mpc_footstep() {
    for (label, mpc_fs) in [("baseline (open-loop Raibert footstep)", false),
                            ("MPC-optimized footstep (B)", true)] {
        let cfg = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
        let params = WbcParams {
            cmd_vx: 2.20,
            total_time_s: 15.0,
            burn_in_s: 0.5,
            gait_type_override: Some(GaitType::Bound),
            duty_factor_override: Some(0.35),
            gait_cycle_period_override: Some(0.18),
            max_step_length_override: Some(0.18),
            mpc_optimized_footstep_override: Some(mpc_fs),
            bound_trim_reference: Some((100.0, 10.0)),
            bound_trim_thrust_scale_override: Some(1.0),
            yaw_pd_gain_override: Some((10.0, 1.0)),
            adaptive_cycle_period: Some(AdaptivePeriodConfig {
                gain: 0.10,
                update_interval_s: 0.2,
                min_period_s: 0.16,
                max_period_s: 0.20,
            }),
            full_centroidal: Some(FullCentroidalOpts {
                legged_control_parity: true,
                use_mpc_predicted_footstep: mpc_fs,
                dynamic_joint_q_reference: false,
                mpc_override: None,
                task_space_joint_vel_weight: None,
                true_centroidal_coupling: false,
                capture_point_gain_override: Some(0.0),
                base_pos_xy_weight_override: None,
                max_normal_force_override: None,
                roll_pitch_weight_override: None,
                bound_fore_aft_placement_gain_override: None,
                roll_rate_weight_override: None,
                bound_pitch_placement_gain_override: None,
                bound_pitch_placement_dc_tau_override: None,
                bound_tabulated_reference_csv: None,
                bound_prescribed_footholds_override: None,
            }),
            ..WbcParams::forward_walk_misa_wbc(wbc::Formulation::ForceSpace, cfg)
        };
        if let Some(samples) = run_wbc_sim(params) {
            report_time_windowed_summary(
                &format!("duty=0.35, cmd_vx=2.20 (constant), {label}, 15s"),
                &samples, 1.0,
            );
        }
    }
}

/// Sec.5c8 isolation: the full (B) (both flags on) faceplanted with
/// roll=π by t=1-2s. Which half breaks? `mpc_optimized_footstep`
/// (add the foot-XY COST to the MPC, keep the open-loop Raibert swing
/// target) vs `use_mpc_predicted_footstep` (take the swing IK target
/// from the MPC's predicted joint_q FK, but no foot-XY cost so the MPC
/// isn't even optimizing the footstep -- the documented "P2" case).
/// 3s each (the faceplant is fast), 0.5s windows, to localize the
/// destabilizing half before designing the flight-phase fix.
#[test]
#[ignore = "exploratory stress test — run with --ignored"]
fn go2_wbc_bound_flight_phase_duty035_mpc_footstep_isolation() {
    for (label, opt_fs, use_fs) in [
        ("cost-only (mpc_optimized_footstep, Raibert swing target)", true, false),
        ("predict-only (use_mpc_predicted_footstep, no foot-XY cost)", false, true),
    ] {
        let cfg = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
        let params = WbcParams {
            cmd_vx: 2.20,
            total_time_s: 3.0,
            burn_in_s: 0.5,
            gait_type_override: Some(GaitType::Bound),
            duty_factor_override: Some(0.35),
            gait_cycle_period_override: Some(0.18),
            max_step_length_override: Some(0.18),
            mpc_optimized_footstep_override: Some(opt_fs),
            bound_trim_reference: Some((100.0, 10.0)),
            bound_trim_thrust_scale_override: Some(1.0),
            yaw_pd_gain_override: Some((10.0, 1.0)),
            adaptive_cycle_period: Some(AdaptivePeriodConfig {
                gain: 0.10,
                update_interval_s: 0.2,
                min_period_s: 0.16,
                max_period_s: 0.20,
            }),
            full_centroidal: Some(FullCentroidalOpts {
                legged_control_parity: true,
                use_mpc_predicted_footstep: use_fs,
                dynamic_joint_q_reference: false,
                mpc_override: None,
                task_space_joint_vel_weight: None,
                true_centroidal_coupling: false,
                capture_point_gain_override: Some(0.0),
                base_pos_xy_weight_override: None,
                max_normal_force_override: None,
                roll_pitch_weight_override: None,
                bound_fore_aft_placement_gain_override: None,
                roll_rate_weight_override: None,
                bound_pitch_placement_gain_override: None,
                bound_pitch_placement_dc_tau_override: None,
                bound_tabulated_reference_csv: None,
                bound_prescribed_footholds_override: None,
            }),
            ..WbcParams::forward_walk_misa_wbc(wbc::Formulation::ForceSpace, cfg)
        };
        if let Some(samples) = run_wbc_sim(params) {
            report_time_windowed_summary(
                &format!("duty=0.35, cmd_vx=2.20, {label}, 3s"),
                &samples, 0.5,
            );
        }
    }
}

/// Sec.5d0: Sec.5c9 localized (B)'s instability to the foot-XY COST
/// (`q_foot_xy_world=500`) fighting the pitch-critical GRF solution.
/// Sweeps that weight DOWN (500/100/20/5/1) with both footstep flags
/// on, constant cmd=2.20, 8s each, to find a weight where the MPC can
/// gently shape the footstep (jointly with GRF -- the real option B)
/// without wrecking the delicate Bound pitch balance. gain=0-equivalent
/// baseline (Sec.5c6) held ~1.7 m/s stable for 15s; the target is a
/// low q_foot that stays stable while letting the MPC own the foothold.
#[test]
#[ignore = "exploratory stress test — run with --ignored"]
fn go2_wbc_bound_flight_phase_duty035_q_foot_sweep() {
    for q_foot in [500.0, 100.0, 20.0, 5.0, 1.0] {
        let cfg = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
        let params = WbcParams {
            cmd_vx: 2.20,
            total_time_s: 8.0,
            burn_in_s: 0.5,
            gait_type_override: Some(GaitType::Bound),
            duty_factor_override: Some(0.35),
            gait_cycle_period_override: Some(0.18),
            max_step_length_override: Some(0.18),
            mpc_optimized_footstep_override: Some(true),
            q_foot_xy_world_override: Some(q_foot),
            bound_trim_reference: Some((100.0, 10.0)),
            bound_trim_thrust_scale_override: Some(1.0),
            yaw_pd_gain_override: Some((10.0, 1.0)),
            adaptive_cycle_period: Some(AdaptivePeriodConfig {
                gain: 0.10,
                update_interval_s: 0.2,
                min_period_s: 0.16,
                max_period_s: 0.20,
            }),
            full_centroidal: Some(FullCentroidalOpts {
                legged_control_parity: true,
                use_mpc_predicted_footstep: true,
                dynamic_joint_q_reference: false,
                mpc_override: None,
                task_space_joint_vel_weight: None,
                true_centroidal_coupling: false,
                capture_point_gain_override: Some(0.0),
                base_pos_xy_weight_override: None,
                max_normal_force_override: None,
                roll_pitch_weight_override: None,
                bound_fore_aft_placement_gain_override: None,
                roll_rate_weight_override: None,
                bound_pitch_placement_gain_override: None,
                bound_pitch_placement_dc_tau_override: None,
                bound_tabulated_reference_csv: None,
                bound_prescribed_footholds_override: None,
            }),
            ..WbcParams::forward_walk_misa_wbc(wbc::Formulation::ForceSpace, cfg)
        };
        if let Some(samples) = run_wbc_sim(params) {
            report_time_windowed_summary(
                &format!("duty=0.35, cmd_vx=2.20, MPC footstep both-on, q_foot_xy_world={q_foot:.0}, 8s"),
                &samples, 1.0,
            );
        }
    }
}

/// Sec.5d2: the (B) formulation fix. Sec.5d1 pinned the instability to
/// the foot-XY cost's `base_pos` term coupling into the GRF/pitch
/// solution; the body-frame variant (`foot_xy_cost_body_frame`) drops
/// that term so the MPC places the foot by swing-leg motion alone.
/// Compares, on the constant-cmd=2.20 best pattern, 15s each:
///  1. baseline (no MPC footstep) -- Sec.5c6's stable ~1.7 m/s;
///  2. world-frame cost (the Sec.5c8 faceplant), q_foot=100;
///  3. body-frame cost (the fix), q_foot=100;
/// with `use_mpc_predicted_footstep=true` in the two cost cases so the
/// controller uses the MPC's chosen foothold. If the body-frame fix
/// works, case 3 should stay stable (unlike case 2) while letting the
/// MPC own the footstep.
#[test]
#[ignore = "exploratory stress test — run with --ignored"]
fn go2_wbc_bound_flight_phase_duty035_footstep_body_frame() {
    let trials = [
        ("1. baseline (open-loop Raibert, no MPC footstep)", false, false, false),
        ("2. world-frame foot-XY cost q=100 (Sec.5c8 faceplant)", true, true, false),
        ("3. body-frame foot-XY cost q=100 (Sec.5d1 fix)", true, true, true),
    ];
    for (label, opt_fs, use_fs, body_frame) in trials {
        let cfg = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
        let params = WbcParams {
            cmd_vx: 2.20,
            total_time_s: 15.0,
            burn_in_s: 0.5,
            gait_type_override: Some(GaitType::Bound),
            duty_factor_override: Some(0.35),
            gait_cycle_period_override: Some(0.18),
            max_step_length_override: Some(0.18),
            mpc_optimized_footstep_override: Some(opt_fs),
            q_foot_xy_world_override: if opt_fs { Some(100.0) } else { None },
            foot_xy_cost_body_frame_override: Some(body_frame),
            bound_trim_reference: Some((100.0, 10.0)),
            bound_trim_thrust_scale_override: Some(1.0),
            yaw_pd_gain_override: Some((10.0, 1.0)),
            adaptive_cycle_period: Some(AdaptivePeriodConfig {
                gain: 0.10,
                update_interval_s: 0.2,
                min_period_s: 0.16,
                max_period_s: 0.20,
            }),
            full_centroidal: Some(FullCentroidalOpts {
                legged_control_parity: true,
                use_mpc_predicted_footstep: use_fs,
                dynamic_joint_q_reference: false,
                mpc_override: None,
                task_space_joint_vel_weight: None,
                true_centroidal_coupling: false,
                capture_point_gain_override: Some(0.0),
                base_pos_xy_weight_override: None,
                max_normal_force_override: None,
                roll_pitch_weight_override: None,
                bound_fore_aft_placement_gain_override: None,
                roll_rate_weight_override: None,
                bound_pitch_placement_gain_override: None,
                bound_pitch_placement_dc_tau_override: None,
                bound_tabulated_reference_csv: None,
                bound_prescribed_footholds_override: None,
            }),
            ..WbcParams::forward_walk_misa_wbc(wbc::Formulation::ForceSpace, cfg)
        };
        if let Some(samples) = run_wbc_sim(params) {
            report_time_windowed_summary(
                &format!("duty=0.35, cmd_vx=2.20, {label}, 15s"),
                &samples, 1.0,
            );
        }
    }
}

/// Sec.5d3: the (B) research step. Sec.5d2 pinned the faceplant to the
/// MPC's ASYMMETRIC per-leg predicted footholds rolling the body over
/// in the aerial phase. `bound_symmetric_foothold` symmetrizes each
/// L/R pair (front FL/FR, rear RL/RR) before the footholds become swing
/// targets. Compares, on the constant-cmd=2.20 best pattern, 15s each:
///  1. baseline (no MPC footstep) -- Sec.5c6's stable ~1.7 m/s;
///  2. body-frame cost + symmetric foothold (the Sec.5d3 fix);
///  3. world-frame cost + symmetric foothold;
/// both cost cases q_foot=100 with use_mpc_predicted_footstep on. If
/// pair symmetry is the missing piece, the MPC-footstep cases should
/// now stay upright (no roll=pi) and ideally track toward baseline.
#[test]
#[ignore = "exploratory stress test — run with --ignored"]
fn go2_wbc_bound_flight_phase_duty035_symmetric_foothold() {
    let trials = [
        ("1. baseline (no MPC footstep)", false, false, false, false),
        ("2. body-frame cost + symmetric foothold", true, true, true, true),
        ("3. world-frame cost + symmetric foothold", true, true, false, true),
    ];
    for (label, opt_fs, use_fs, body_frame, sym) in trials {
        let cfg = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
        let params = WbcParams {
            cmd_vx: 2.20,
            total_time_s: 15.0,
            burn_in_s: 0.5,
            gait_type_override: Some(GaitType::Bound),
            duty_factor_override: Some(0.35),
            gait_cycle_period_override: Some(0.18),
            max_step_length_override: Some(0.18),
            mpc_optimized_footstep_override: Some(opt_fs),
            q_foot_xy_world_override: if opt_fs { Some(100.0) } else { None },
            foot_xy_cost_body_frame_override: Some(body_frame),
            bound_symmetric_foothold_override: Some(sym),
            bound_trim_reference: Some((100.0, 10.0)),
            bound_trim_thrust_scale_override: Some(1.0),
            yaw_pd_gain_override: Some((10.0, 1.0)),
            adaptive_cycle_period: Some(AdaptivePeriodConfig {
                gain: 0.10,
                update_interval_s: 0.2,
                min_period_s: 0.16,
                max_period_s: 0.20,
            }),
            full_centroidal: Some(FullCentroidalOpts {
                legged_control_parity: true,
                use_mpc_predicted_footstep: use_fs,
                dynamic_joint_q_reference: false,
                mpc_override: None,
                task_space_joint_vel_weight: None,
                true_centroidal_coupling: false,
                capture_point_gain_override: Some(0.0),
                base_pos_xy_weight_override: None,
                max_normal_force_override: None,
                roll_pitch_weight_override: None,
                bound_fore_aft_placement_gain_override: None,
                roll_rate_weight_override: None,
                bound_pitch_placement_gain_override: None,
                bound_pitch_placement_dc_tau_override: None,
                bound_tabulated_reference_csv: None,
                bound_prescribed_footholds_override: None,
            }),
            ..WbcParams::forward_walk_misa_wbc(wbc::Formulation::ForceSpace, cfg)
        };
        if let Some(samples) = run_wbc_sim(params) {
            report_time_windowed_summary(
                &format!("duty=0.35, cmd_vx=2.20, {label}, 15s"),
                &samples, 1.0,
            );
        }
    }
}

/// Sec.5d4: A/B on the ballistic vertical-bounce reference against the
/// flat (MIT-aligned) reference, on the constant-cmd=2.20 best pattern
/// (Sec.5c6, uses the Bound trim), 15s each. The MIT literature check
/// (Sec.5d4) found their convex-MPC bounders command a FLAT reference
/// (z-vel=0, pitch=0) and let the bounce EMERGE -- so
/// `vertical_reference=true` is a hypothesis AGAINST that evidence.
/// This measures it directly: if OFF (flat) >= ON, it confirms the MIT
/// finding empirically for our own controller.
#[test]
#[ignore = "exploratory stress test — run with --ignored"]
fn go2_wbc_bound_flight_phase_duty035_vertical_reference_ab() {
    for (label, vr) in [("OFF (flat reference, MIT-aligned)", false),
                        ("ON (ballistic bounce reference)", true)] {
        let cfg = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
        let params = WbcParams {
            cmd_vx: 2.20,
            total_time_s: 15.0,
            burn_in_s: 0.5,
            gait_type_override: Some(GaitType::Bound),
            duty_factor_override: Some(0.35),
            gait_cycle_period_override: Some(0.18),
            max_step_length_override: Some(0.18),
            bound_trim_reference: Some((100.0, 10.0)),
            bound_trim_thrust_scale_override: Some(1.0),
            bound_trim_vertical_reference_override: Some(vr),
            yaw_pd_gain_override: Some((10.0, 1.0)),
            adaptive_cycle_period: Some(AdaptivePeriodConfig {
                gain: 0.10,
                update_interval_s: 0.2,
                min_period_s: 0.16,
                max_period_s: 0.20,
            }),
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
                bound_fore_aft_placement_gain_override: None,
                roll_rate_weight_override: None,
                bound_pitch_placement_gain_override: None,
                bound_pitch_placement_dc_tau_override: None,
                bound_tabulated_reference_csv: None,
                bound_prescribed_footholds_override: None,
            }),
            ..WbcParams::forward_walk_misa_wbc(wbc::Formulation::ForceSpace, cfg)
        };
        if let Some(samples) = run_wbc_sim(params) {
            report_time_windowed_summary(
                &format!("duty=0.35, cmd_vx=2.20, vertical_reference {label}, 15s"),
                &samples, 1.0,
            );
        }
    }
}

/// Sec.5d5: the MIT-faithful Bound experiment. Per the Sec.5d4 paper
/// verification, MIT's convex-MPC bounders use (a) a FLAT reference
/// (pitch=0, z-vel=0), (b) a LOW orientation/pitch weight so the bound
/// pitch oscillation EMERGES rather than being tracked/fought, and (c)
/// Raibert footsteps outside the MPC. Our best pattern instead injects
/// a bespoke closed-form PITCH TRIM (non-flat pitch reference) -- the
/// biggest divergence from MIT. This drops the trim entirely
/// (`bound_trim_reference: None`, so no MPC pitch/F_x/F_z injection and
/// no WBC pitch PD, which defaults off) and sweeps the MPC's base
/// roll/pitch attitude weight DOWN from the 25.0 default toward MIT's
/// ~1.0, letting the emergent pitch bounce happen. Fixed timing (no
/// PLL), open-loop Raibert footstep, duty=0.35, cmd_vx=1.5. Keeps the
/// real mass/inertia sync (normally piggy-backed on the trim path) and
/// the yaw-hold PD (a straight-line stabilizer). Historically the
/// trim-less MPC Bound "reversed", but low pitch weight was never tried
/// and predates the yaw-PD / contact-schedule fixes.
#[test]
#[ignore = "exploratory stress test — run with --ignored"]
fn go2_wbc_bound_flight_phase_duty035_mit_emergent_pitch() {
    for pitch_w in [25.0, 10.0, 5.0, 1.0, 0.5] {
        let cfg = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
        let params = WbcParams {
            cmd_vx: 1.50,
            total_time_s: 10.0,
            burn_in_s: 0.5,
            gait_type_override: Some(GaitType::Bound),
            duty_factor_override: Some(0.35),
            gait_cycle_period_override: Some(0.18),
            max_step_length_override: Some(0.18),
            // MIT-faithful: NO bespoke trim (flat pitch reference), NO
            // PLL (fixed timing). Keep real mass/inertia + yaw hold.
            bound_trim_reference: None,
            sync_real_mass_inertia: true,
            yaw_pd_gain_override: Some((10.0, 1.0)),
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
                roll_pitch_weight_override: Some((pitch_w, pitch_w)),
                bound_fore_aft_placement_gain_override: None,
                roll_rate_weight_override: None,
                bound_pitch_placement_gain_override: None,
                bound_pitch_placement_dc_tau_override: None,
                bound_tabulated_reference_csv: None,
                bound_prescribed_footholds_override: None,
            }),
            ..WbcParams::forward_walk_misa_wbc(wbc::Formulation::ForceSpace, cfg)
        };
        if let Some(samples) = run_wbc_sim(params) {
            report_time_windowed_summary(
                &format!("duty=0.35, cmd_vx=1.50, MIT-faithful (no trim, roll/pitch weight={pitch_w:.1}), 10s"),
                &samples, 1.0,
            );
        }
    }
}

/// Video-capture source for Sec.5d5's MIT-faithful stable Bound: NO
/// bespoke trim, flat pitch reference, low MPC roll/pitch weight (5.0,
/// the sweet spot) so the bound pitch oscillation emerges rather than
/// being fought, open-loop Raibert footstep, fixed timing, duty=0.35,
/// cmd_vx=1.5. This is the second, MIT-aligned line -- much simpler
/// than the trim+PLL+thrust_scale stack -- stable ~1.2 m/s with a real
/// flight phase. Run with `WBC_WALK_CSV_OUT=<path> cargo test --release
/// --features mujoco --test wbc_walk_go2 go2_wbc_bound_flight_phase_
/// duty035_mit_video_source -- --ignored --nocapture`.
#[test]
#[ignore = "exploratory stress test — run with --ignored; also the WBC_WALK_CSV_OUT video-capture source for Sec.5d5 (MIT-faithful trimless Bound)"]
fn go2_wbc_bound_flight_phase_duty035_mit_video_source() {
    let cfg = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
    let params = WbcParams {
        cmd_vx: 1.50,
        total_time_s: 8.5,
        burn_in_s: 0.5,
        gait_type_override: Some(GaitType::Bound),
        duty_factor_override: Some(0.35),
        gait_cycle_period_override: Some(0.18),
        max_step_length_override: Some(0.18),
        bound_trim_reference: None,
        sync_real_mass_inertia: true,
        yaw_pd_gain_override: Some((10.0, 1.0)),
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
            roll_pitch_weight_override: Some((5.0, 5.0)),
            bound_fore_aft_placement_gain_override: None,
            roll_rate_weight_override: None,
            bound_pitch_placement_gain_override: None,
            bound_pitch_placement_dc_tau_override: None,
            bound_tabulated_reference_csv: None,
            bound_prescribed_footholds_override: None,
        }),
        ..WbcParams::forward_walk_misa_wbc(wbc::Formulation::ForceSpace, cfg)
    };
    let Some(samples) = run_wbc_sim(params) else { return };
    report_walk_summary(
        "bound MIT-faithful (no trim, roll/pitch weight=5, Raibert footstep) video source (duty=0.35, cmd_vx=1.5)",
        &samples, 1.50,
    );
    report_time_windowed_summary(
        "bound MIT-faithful (no trim, weight=5), cmd_vx=1.5 (video source, time-windowed)",
        &samples, 1.0,
    );
}

/// Sec.5d6 (b): fine pitch-weight sweep for the MIT-faithful Bound.
/// Sec.5d5 found weight=5 the coarse sweet spot (25/10 too stiff, 0.5
/// drifts); this refines 3..7 at cmd_vx=1.5, duty=0.35, no trim, to
/// pick the optimum before layering ramp/PLL (c) and the cmd_vx
/// ceiling (a).
#[test]
#[ignore = "exploratory stress test — run with --ignored"]
fn go2_wbc_bound_flight_phase_duty035_mit_weight_fine() {
    for w in [3.0, 4.0, 5.0, 6.0, 7.0] {
        let cfg = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
        let params = WbcParams {
            cmd_vx: 1.50,
            total_time_s: 12.0,
            burn_in_s: 0.5,
            gait_type_override: Some(GaitType::Bound),
            duty_factor_override: Some(0.35),
            gait_cycle_period_override: Some(0.18),
            max_step_length_override: Some(0.18),
            bound_trim_reference: None,
            sync_real_mass_inertia: true,
            yaw_pd_gain_override: Some((10.0, 1.0)),
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
                roll_pitch_weight_override: Some((w, w)),
                bound_fore_aft_placement_gain_override: None,
                roll_rate_weight_override: None,
                bound_pitch_placement_gain_override: None,
                bound_pitch_placement_dc_tau_override: None,
                bound_tabulated_reference_csv: None,
                bound_prescribed_footholds_override: None,
            }),
            ..WbcParams::forward_walk_misa_wbc(wbc::Formulation::ForceSpace, cfg)
        };
        if let Some(samples) = run_wbc_sim(params) {
            report_time_windowed_summary(
                &format!("duty=0.35, cmd_vx=1.50, MIT-faithful weight={w:.1}, 12s"),
                &samples, 1.0,
            );
        }
    }
}

/// Sec.5d6 (c): layer a smooth startup ramp + PLL onto the MIT-faithful
/// Bound (weight=4, Sec.5d6 b's pick). NOTE the trim-version ramp
/// failed (Sec.5c0-5c2) because ramping cmd_vx desynced the cmd-
/// independent F_x trim -- but the MIT version has NO trim, so the ramp
/// should compose cleanly. Compares plain vs (cmd_vx ramp 2s + PLL) at
/// cmd_vx=1.5, 12s.
#[test]
#[ignore = "exploratory stress test — run with --ignored"]
fn go2_wbc_bound_flight_phase_duty035_mit_ramp_pll() {
    let trials: [(&str, Option<f64>, bool); 2] = [
        ("plain (no ramp, no PLL)", None, false),
        ("ramp 2s + PLL", Some(2.0), true),
    ];
    for (label, ramp, pll) in trials {
        let cfg = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
        let params = WbcParams {
            cmd_vx: 1.50,
            cmd_vx_ramp_s: ramp,
            pll_accumulate_during_ramp: pll && ramp.is_some(),
            total_time_s: 12.0,
            burn_in_s: 0.5,
            gait_type_override: Some(GaitType::Bound),
            duty_factor_override: Some(0.35),
            gait_cycle_period_override: Some(0.18),
            max_step_length_override: Some(0.18),
            bound_trim_reference: None,
            sync_real_mass_inertia: true,
            yaw_pd_gain_override: Some((10.0, 1.0)),
            adaptive_cycle_period: if pll {
                Some(AdaptivePeriodConfig { gain: 0.10, update_interval_s: 0.2, min_period_s: 0.16, max_period_s: 0.20 })
            } else {
                None
            },
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
                roll_pitch_weight_override: Some((4.0, 4.0)),
                bound_fore_aft_placement_gain_override: None,
                roll_rate_weight_override: None,
                bound_pitch_placement_gain_override: None,
                bound_pitch_placement_dc_tau_override: None,
                bound_tabulated_reference_csv: None,
                bound_prescribed_footholds_override: None,
            }),
            ..WbcParams::forward_walk_misa_wbc(wbc::Formulation::ForceSpace, cfg)
        };
        if let Some(samples) = run_wbc_sim(params) {
            report_time_windowed_summary(
                &format!("duty=0.35, cmd_vx=1.50, MIT weight=4, {label}, 12s"),
                &samples, 0.5,
            );
        }
    }
}

/// Sec.5d6 (a): cmd_vx ceiling for the MIT-faithful Bound. Sec.5d6 (c)
/// found plain weight=4 (no ramp, no PLL) the best config -- the MIT
/// fixed-timing structure needs neither. Sweeps cmd_vx up (1.5/2.0/
/// 2.5/3.0) to find how fast this trimless line can actually track,
/// 12s each, duty=0.35.
#[test]
#[ignore = "exploratory stress test — run with --ignored"]
fn go2_wbc_bound_flight_phase_duty035_mit_cmd_vx_ceiling() {
    for cmd_vx in [1.50, 2.00, 2.50, 3.00] {
        let cfg = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
        let params = WbcParams {
            cmd_vx,
            total_time_s: 12.0,
            burn_in_s: 0.5,
            gait_type_override: Some(GaitType::Bound),
            duty_factor_override: Some(0.35),
            gait_cycle_period_override: Some(0.18),
            max_step_length_override: Some(0.18),
            bound_trim_reference: None,
            sync_real_mass_inertia: true,
            yaw_pd_gain_override: Some((10.0, 1.0)),
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
                roll_pitch_weight_override: Some((4.0, 4.0)),
                bound_fore_aft_placement_gain_override: None,
                roll_rate_weight_override: None,
                bound_pitch_placement_gain_override: None,
                bound_pitch_placement_dc_tau_override: None,
                bound_tabulated_reference_csv: None,
                bound_prescribed_footholds_override: None,
            }),
            ..WbcParams::forward_walk_misa_wbc(wbc::Formulation::ForceSpace, cfg)
        };
        if let Some(samples) = run_wbc_sim(params) {
            report_time_windowed_summary(
                &format!("duty=0.35, MIT weight=4, cmd_vx={cmd_vx:.2}, 12s"),
                &samples, 1.0,
            );
        }
    }
}

/// Sec.5d7 (hybrid): the MIT line caps at ~1.3 m/s because at cmd_vx=2.0
/// the low pitch weight (4) can't hold the larger pitch disturbance
/// (pitch ran to 0.49). Before reaching for an explicit F_x thrust
/// bias, test whether the ceiling is simply WEIGHT-limited: at the
/// failing cmd_vx=2.0, sweep the MPC roll/pitch weight UP (4/8/12/16/25)
/// -- higher weight = more pitch authority = maybe it tracks. If a
/// higher weight rescues cmd_vx=2.0, the fix is a speed-adaptive weight
/// (still MIT-structured, no trim), simpler than an F_x hybrid. Still
/// trimless, no PLL, duty=0.35, 12s.
#[test]
#[ignore = "exploratory stress test — run with --ignored"]
fn go2_wbc_bound_flight_phase_duty035_mit_highspeed_weight() {
    for w in [4.0, 8.0, 12.0, 16.0, 25.0] {
        let cfg = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
        let params = WbcParams {
            cmd_vx: 2.00,
            total_time_s: 12.0,
            burn_in_s: 0.5,
            gait_type_override: Some(GaitType::Bound),
            duty_factor_override: Some(0.35),
            gait_cycle_period_override: Some(0.18),
            max_step_length_override: Some(0.18),
            bound_trim_reference: None,
            sync_real_mass_inertia: true,
            yaw_pd_gain_override: Some((10.0, 1.0)),
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
                roll_pitch_weight_override: Some((w, w)),
                bound_fore_aft_placement_gain_override: None,
                roll_rate_weight_override: None,
                bound_pitch_placement_gain_override: None,
                bound_pitch_placement_dc_tau_override: None,
                bound_tabulated_reference_csv: None,
                bound_prescribed_footholds_override: None,
            }),
            ..WbcParams::forward_walk_misa_wbc(wbc::Formulation::ForceSpace, cfg)
        };
        if let Some(samples) = run_wbc_sim(params) {
            report_time_windowed_summary(
                &format!("duty=0.35, MIT cmd_vx=2.00, roll/pitch weight={w:.0}, 12s"),
                &samples, 1.0,
            );
        }
    }
}

/// Sec.5d7 (hybrid): MIT structure (trimless, weight=4) + a constant
/// forward F_x thrust bias to break the ~1.3 m/s ceiling. Sec.5d7's
/// weight sweep showed higher pitch weight can't rescue cmd_vx=2.0
/// (thrust-limited, not weight-limited), so this adds the missing
/// forward force directly. Sweeps the bias (0/20/40/60 N total) at
/// cmd_vx=2.0, duty=0.35, 12s. If a bias lets cmd_vx=2.0 track stably,
/// the hybrid raises the MIT line's ceiling with one extra scalar.
#[test]
#[ignore = "exploratory stress test — run with --ignored"]
fn go2_wbc_bound_flight_phase_duty035_mit_fx_bias() {
    for bias in [0.0, 20.0, 40.0, 60.0] {
        let cfg = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
        let params = WbcParams {
            cmd_vx: 2.00,
            total_time_s: 12.0,
            burn_in_s: 0.5,
            gait_type_override: Some(GaitType::Bound),
            duty_factor_override: Some(0.35),
            gait_cycle_period_override: Some(0.18),
            max_step_length_override: Some(0.18),
            bound_trim_reference: None,
            sync_real_mass_inertia: true,
            bound_fx_thrust_bias_override: Some(bias),
            yaw_pd_gain_override: Some((10.0, 1.0)),
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
                roll_pitch_weight_override: Some((4.0, 4.0)),
                bound_fore_aft_placement_gain_override: None,
                roll_rate_weight_override: None,
                bound_pitch_placement_gain_override: None,
                bound_pitch_placement_dc_tau_override: None,
                bound_tabulated_reference_csv: None,
                bound_prescribed_footholds_override: None,
            }),
            ..WbcParams::forward_walk_misa_wbc(wbc::Formulation::ForceSpace, cfg)
        };
        if let Some(samples) = run_wbc_sim(params) {
            report_time_windowed_summary(
                &format!("duty=0.35, MIT weight=4, cmd_vx=2.00, fx_thrust_bias={bias:.0}N, 12s"),
                &samples, 1.0,
            );
        }
    }
}

/// Sec.5d8 (推奨1): the MIT Cheetah 2 startup recipe on the MIT-faithful
/// Bound (trimless, weight=4). Park/Wensing/Kim (IJRR 2017) start
/// bounding with the desired speed increased "from step to step" --
/// discrete, contact-synced -- not the continuous-time ramp that
/// desynced from the gait phase for us (Sec.5c0-5c2). Compares three
/// startups to cmd_vx=1.5, duty=0.35, 10s, in 0.5s windows to inspect
/// the transient:
///   1. abrupt (cmd steps to 1.5 at t=0);
///   2. time ramp (2.0s continuous);
///   3. step ramp (+0.15 m/s per half-cycle, MIT-style).
#[test]
#[ignore = "exploratory stress test — run with --ignored"]
fn go2_wbc_bound_flight_phase_duty035_mit_startup() {
    let trials: [(&str, Option<f64>, Option<f64>); 3] = [
        ("1. abrupt", None, None),
        ("2. time ramp 2.0s", Some(2.0), None),
        ("3. step ramp +0.15/step (MIT)", None, Some(0.15)),
    ];
    for (label, time_ramp, step_inc) in trials {
        let cfg = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
        let params = WbcParams {
            cmd_vx: 1.50,
            cmd_vx_ramp_s: time_ramp,
            cmd_vx_step_increment: step_inc,
            total_time_s: 10.0,
            burn_in_s: 0.5,
            gait_type_override: Some(GaitType::Bound),
            duty_factor_override: Some(0.35),
            gait_cycle_period_override: Some(0.18),
            max_step_length_override: Some(0.18),
            bound_trim_reference: None,
            sync_real_mass_inertia: true,
            yaw_pd_gain_override: Some((10.0, 1.0)),
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
                roll_pitch_weight_override: Some((4.0, 4.0)),
                bound_fore_aft_placement_gain_override: None,
                roll_rate_weight_override: None,
                bound_pitch_placement_gain_override: None,
                bound_pitch_placement_dc_tau_override: None,
                bound_tabulated_reference_csv: None,
                bound_prescribed_footholds_override: None,
            }),
            ..WbcParams::forward_walk_misa_wbc(wbc::Formulation::ForceSpace, cfg)
        };
        if let Some(samples) = run_wbc_sim(params) {
            report_time_windowed_summary(
                &format!("duty=0.35, MIT weight=4, startup {label}, cmd_vx=1.5, 10s"),
                &samples, 0.5,
            );
        }
    }
}

/// Sec.5d9: the MIT Cheetah 2 step-ramp on the TRIM line -- its proper
/// home. The trim carries a feedforward F_x (like Cheetah 2's impulse-
/// scaling), so ramping the command step-by-step (so the feedforward
/// starts small and grows) should fit here where the time ramp failed
/// (Sec.5c0-5c2). Compares abrupt / time ramp 2s / step ramp +0.2/step
/// on the trim best pattern (duty=0.35, thrust_scale=1.0, PLL, clamp
/// 0.16-0.20, weight left at default), cmd_vx=1.5, 12s, 0.5s windows.
#[test]
#[ignore = "exploratory stress test — run with --ignored"]
fn go2_wbc_bound_flight_phase_duty035_trim_startup() {
    let trials: [(&str, Option<f64>, Option<f64>); 3] = [
        ("1. abrupt", None, None),
        ("2. time ramp 2.0s", Some(2.0), None),
        ("3. step ramp +0.2/step (MIT)", None, Some(0.2)),
    ];
    for (label, time_ramp, step_inc) in trials {
        let cfg = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
        let params = WbcParams {
            cmd_vx: 1.50,
            cmd_vx_ramp_s: time_ramp,
            cmd_vx_step_increment: step_inc,
            pll_accumulate_during_ramp: time_ramp.is_some(),
            total_time_s: 12.0,
            burn_in_s: 0.5,
            gait_type_override: Some(GaitType::Bound),
            duty_factor_override: Some(0.35),
            gait_cycle_period_override: Some(0.18),
            max_step_length_override: Some(0.18),
            bound_trim_reference: Some((100.0, 10.0)),
            bound_trim_thrust_scale_override: Some(1.0),
            yaw_pd_gain_override: Some((10.0, 1.0)),
            adaptive_cycle_period: Some(AdaptivePeriodConfig {
                gain: 0.10,
                update_interval_s: 0.2,
                min_period_s: 0.16,
                max_period_s: 0.20,
            }),
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
                bound_fore_aft_placement_gain_override: None,
                roll_rate_weight_override: None,
                bound_pitch_placement_gain_override: None,
                bound_pitch_placement_dc_tau_override: None,
                bound_tabulated_reference_csv: None,
                bound_prescribed_footholds_override: None,
            }),
            ..WbcParams::forward_walk_misa_wbc(wbc::Formulation::ForceSpace, cfg)
        };
        if let Some(samples) = run_wbc_sim(params) {
            report_time_windowed_summary(
                &format!("duty=0.35, TRIM, startup {label}, cmd_vx=1.5, 12s"),
                &samples, 0.5,
            );
        }
    }
}

/// Sec.5e0 (1b): toward an ENERGETIC bound with real air time. Air-time
/// apex ~= g*T_flight^2/8 with T_flight=(0.5-duty)*T, so a visible hop
/// needs a larger T_flight -> lower duty and/or longer cycle. And the
/// MPC's flat-height reference itself supplies the launch impulse (it
/// must push hard in a short stance to undo a long-flight fall), so no
/// explicit pump is needed at steady state -- just room for real flight.
/// Probes the MIT-faithful base (trimless, weight=4) at longer cycles /
/// lower duty, higher swing height, cmd_vx=1.0, 10s, reading z_range
/// (CoM vertical excursion = bounce amplitude) and stability.
#[test]
#[ignore = "exploratory stress test — run with --ignored"]
fn go2_wbc_bound_energetic_sweep() {
    let trials = [
        ("T=0.30 duty=0.30", 0.30, 0.30),
        ("T=0.30 duty=0.25", 0.30, 0.25),
        ("T=0.30 duty=0.20", 0.30, 0.20),
        ("T=0.36 duty=0.20", 0.36, 0.20),
    ];
    for (label, period, duty) in trials {
        let cfg = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
        let params = WbcParams {
            cmd_vx: 1.00,
            total_time_s: 10.0,
            burn_in_s: 0.5,
            gait_type_override: Some(GaitType::Bound),
            duty_factor_override: Some(duty),
            gait_cycle_period_override: Some(period),
            max_step_length_override: Some(0.18),
            swing_height_override: Some(0.10),
            bound_trim_reference: None,
            sync_real_mass_inertia: true,
            yaw_pd_gain_override: Some((10.0, 1.0)),
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
                roll_pitch_weight_override: Some((4.0, 4.0)),
                bound_fore_aft_placement_gain_override: None,
                roll_rate_weight_override: None,
                bound_pitch_placement_gain_override: None,
                bound_pitch_placement_dc_tau_override: None,
                bound_tabulated_reference_csv: None,
                bound_prescribed_footholds_override: None,
            }),
            ..WbcParams::forward_walk_misa_wbc(wbc::Formulation::ForceSpace, cfg)
        };
        if let Some(samples) = run_wbc_sim(params) {
            report_time_windowed_summary(
                &format!("ENERGETIC bound, MIT weight=4, {label}, cmd_vx=1.0, 10s"),
                &samples, 1.0,
            );
        }
    }
}

/// Sec.5e1 (1b): Sec.5e0 got REAL air time (z_range up to 0.24 m) at
/// T=0.30 duty=0.30 without falling, but pitch ran high (0.4-0.7 rad) --
/// the energetic gait's larger attitude disturbance overwhelms the
/// weight=4 tuned for the low-flight gait. Raises the MPC roll/pitch
/// weight (8/15/25/40) at that config to tame the pitch while keeping
/// the real bounce, cmd_vx=0.5 (modest -- establish a stable energetic
/// bounce first), 10s. Reads z_range for the retained air time.
#[test]
#[ignore = "exploratory stress test — run with --ignored"]
fn go2_wbc_bound_energetic_weight() {
    for w in [8.0, 15.0, 25.0, 40.0] {
        let cfg = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
        let params = WbcParams {
            cmd_vx: 0.50,
            total_time_s: 10.0,
            burn_in_s: 0.5,
            gait_type_override: Some(GaitType::Bound),
            duty_factor_override: Some(0.30),
            gait_cycle_period_override: Some(0.30),
            max_step_length_override: Some(0.18),
            swing_height_override: Some(0.10),
            bound_trim_reference: None,
            sync_real_mass_inertia: true,
            yaw_pd_gain_override: Some((10.0, 1.0)),
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
                roll_pitch_weight_override: Some((w, w)),
                bound_fore_aft_placement_gain_override: None,
                roll_rate_weight_override: None,
                bound_pitch_placement_gain_override: None,
                bound_pitch_placement_dc_tau_override: None,
                bound_tabulated_reference_csv: None,
                bound_prescribed_footholds_override: None,
            }),
            ..WbcParams::forward_walk_misa_wbc(wbc::Formulation::ForceSpace, cfg)
        };
        if let Some(samples) = run_wbc_sim(params) {
            report_time_windowed_summary(
                &format!("ENERGETIC bound, T=0.30 duty=0.30, weight={w:.0}, cmd_vx=0.5, 10s"),
                &samples, 1.0,
            );
        }
    }
}

/// Sec.5e2 (1b): the TRIM version's shot at a stable energetic bound.
/// Sec.5e1 showed the MIT (emergent-pitch) line gets real air time but
/// rolls over in the long flight -- attitude isn't actively controlled.
/// The trim provides a STRUCTURED (prescribed) pitch profile + F_x/F_z,
/// which may stabilize the energetic bounce where emergent pitch can't.
/// At T=0.30 duty=0.30 (f_z=m*g/0.6 = strong bounce force), trim on,
/// vertical reference ON (now the intended bounce is large, so the
/// bounce reference should HELP unlike Sec.5d4's tiny-bounce case),
/// thrust_scale=1.0, pitch_pd, yaw hold, PLL clamp centered on 0.30.
/// cmd_vx=0.5, 10s. Reads z_range for retained air time.
#[test]
#[ignore = "exploratory stress test — run with --ignored"]
fn go2_wbc_bound_energetic_trim() {
    for vref in [false, true] {
        let cfg = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
        let params = WbcParams {
            cmd_vx: 0.50,
            total_time_s: 10.0,
            burn_in_s: 0.5,
            gait_type_override: Some(GaitType::Bound),
            duty_factor_override: Some(0.30),
            gait_cycle_period_override: Some(0.30),
            max_step_length_override: Some(0.18),
            swing_height_override: Some(0.10),
            bound_trim_reference: Some((100.0, 10.0)),
            bound_trim_thrust_scale_override: Some(1.0),
            bound_trim_vertical_reference_override: Some(vref),
            yaw_pd_gain_override: Some((10.0, 1.0)),
            adaptive_cycle_period: Some(AdaptivePeriodConfig {
                gain: 0.10,
                update_interval_s: 0.2,
                min_period_s: 0.28,
                max_period_s: 0.32,
            }),
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
                bound_fore_aft_placement_gain_override: None,
                roll_rate_weight_override: None,
                bound_pitch_placement_gain_override: None,
                bound_pitch_placement_dc_tau_override: None,
                bound_tabulated_reference_csv: None,
                bound_prescribed_footholds_override: None,
            }),
            ..WbcParams::forward_walk_misa_wbc(wbc::Formulation::ForceSpace, cfg)
        };
        if let Some(samples) = run_wbc_sim(params) {
            report_time_windowed_summary(
                &format!("ENERGETIC bound TRIM, T=0.30 duty=0.30, vert_ref={vref}, cmd_vx=0.5, 10s"),
                &samples, 1.0,
            );
        }
    }
}

/// Sec.5f (1b): the roll-rate DEADBEAT reflex for the energetic Bound.
/// Sec.5f1/5f2 proved the death mode is ROLL: the long flight integrates
/// a tiny roll rate into a rollover (roll=pi) at t~2-3s. But during the
/// short stance, the front/rear pair are a Left-Right foot pair, so a
/// differential vertical GRF *can* generate a roll moment -- the MPC just
/// barely penalizes roll rate (q_diag[3]=0.5). This sweeps that weight up
/// (0.5 -> 5/20/50/100) on the MIT-faithful energetic base (T=0.30
/// duty=0.30 swing=0.10, the config that stayed upright ~8s at the stock
/// weight in Sec.5f0). If the deadbeat hypothesis holds, higher roll-rate
/// weight should push the rollover out in time (or eliminate it).
#[test]
#[ignore = "exploratory stress test — run with --ignored"]
fn go2_wbc_bound_energetic_roll_rate_deadbeat() {
    for (q_roll_rate, q_pitch_rate) in [(100.0_f64, 0.5_f64), (100.0, 20.0), (100.0, 100.0)] {
        let cfg = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
        let params = WbcParams {
            cmd_vx: 0.50,
            total_time_s: 10.0,
            burn_in_s: 0.5,
            gait_type_override: Some(GaitType::Bound),
            duty_factor_override: Some(0.30),
            gait_cycle_period_override: Some(0.30),
            max_step_length_override: Some(0.18),
            swing_height_override: Some(0.10),
            bound_trim_reference: None,
            sync_real_mass_inertia: true,
            yaw_pd_gain_override: Some((10.0, 1.0)),
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
                roll_pitch_weight_override: Some((4.0, 4.0)),
                bound_fore_aft_placement_gain_override: None,
                roll_rate_weight_override: Some((q_roll_rate, q_pitch_rate)),
            bound_pitch_placement_gain_override: None,
            bound_pitch_placement_dc_tau_override: None,
            bound_tabulated_reference_csv: None,
            bound_prescribed_footholds_override: None,
            }),
            ..WbcParams::forward_walk_misa_wbc(wbc::Formulation::ForceSpace, cfg)
        };
        if let Some(samples) = run_wbc_sim(params) {
            report_time_windowed_summary(
                &format!("ENERGETIC deadbeat q_roll_rate={q_roll_rate} q_pitch_rate={q_pitch_rate}, T=0.30 duty=0.30, 10s"),
                &samples, 1.0,
            );
        }
    }
}

/// Sec.5f4 (1b): find the MAXIMUM STABLE air time. The aggressive
/// duty=0.30 tumbles (Sec.5f1-5f3); single rate-deadbeat levers each buy
/// only ~1 cycle. Practical goal = the largest z_range (real air time)
/// that stays UPRIGHT the full 10s. Backs duty off 0.30 -> 0.34/0.38/0.42
/// (shorter flight = smaller attitude disturbance) with the rate-deadbeat
/// (roll=100, pitch=100) engaged. Reports z_range (air time) vs whether it
/// survives; the largest surviving z_range is the stable energetic Bound.
#[test]
#[ignore = "exploratory stress test — run with --ignored"]
fn go2_wbc_bound_energetic_stable_edge() {
    for duty in [0.34_f64, 0.38, 0.42] {
        let cfg = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
        let params = WbcParams {
            cmd_vx: 0.50,
            total_time_s: 10.0,
            burn_in_s: 0.5,
            gait_type_override: Some(GaitType::Bound),
            duty_factor_override: Some(duty),
            gait_cycle_period_override: Some(0.30),
            max_step_length_override: Some(0.18),
            swing_height_override: Some(0.10),
            bound_trim_reference: None,
            sync_real_mass_inertia: true,
            yaw_pd_gain_override: Some((10.0, 1.0)),
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
                roll_pitch_weight_override: Some((4.0, 4.0)),
                bound_fore_aft_placement_gain_override: None,
                roll_rate_weight_override: Some((100.0, 100.0)),
            bound_pitch_placement_gain_override: None,
            bound_pitch_placement_dc_tau_override: None,
            bound_tabulated_reference_csv: None,
            bound_prescribed_footholds_override: None,
            }),
            ..WbcParams::forward_walk_misa_wbc(wbc::Formulation::ForceSpace, cfg)
        };
        if let Some(samples) = run_wbc_sim(params) {
            report_time_windowed_summary(
                &format!("STABLE-EDGE bound, duty={duty}, T=0.30, rate-deadbeat(100,100), 10s"),
                &samples, 1.0,
            );
        }
    }
}

/// Sec.5f5 (1b): confirm + map the stable energetic-Bound window. Sec.5f4
/// found duty=0.34 + rate-deadbeat(100,100) stayed upright the full 10s
/// with real air time (z_range up to 0.20m) while 0.38/0.42 tumbled
/// (non-monotonic resonance). This maps the window 0.32..0.36 over a
/// longer 15s horizon to confirm duty=0.34 truly sustains (not a lucky
/// 10s window) and to find the largest sustained air time.
#[test]
#[ignore = "exploratory stress test — run with --ignored"]
fn go2_wbc_bound_energetic_stable_window() {
    for duty in [0.32_f64, 0.33, 0.34, 0.35, 0.36] {
        let cfg = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
        let params = WbcParams {
            cmd_vx: 0.50,
            total_time_s: 15.0,
            burn_in_s: 0.5,
            gait_type_override: Some(GaitType::Bound),
            duty_factor_override: Some(duty),
            gait_cycle_period_override: Some(0.30),
            max_step_length_override: Some(0.18),
            swing_height_override: Some(0.10),
            bound_trim_reference: None,
            sync_real_mass_inertia: true,
            yaw_pd_gain_override: Some((10.0, 1.0)),
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
                roll_pitch_weight_override: Some((4.0, 4.0)),
                bound_fore_aft_placement_gain_override: None,
                roll_rate_weight_override: Some((100.0, 100.0)),
            bound_pitch_placement_gain_override: None,
            bound_pitch_placement_dc_tau_override: None,
            bound_tabulated_reference_csv: None,
            bound_prescribed_footholds_override: None,
            }),
            ..WbcParams::forward_walk_misa_wbc(wbc::Formulation::ForceSpace, cfg)
        };
        if let Some(samples) = run_wbc_sim(params) {
            report_time_windowed_summary(
                &format!("STABLE-WINDOW bound, duty={duty}, T=0.30, rate-deadbeat(100,100), 15s"),
                &samples, 1.0,
            );
        }
    }
}

/// Sec.5f6 (1b): the Poincaré/deadbeat pitch foot-placement -- the real
/// limit-cycle stabilizer. Sec.5f5 proved rate-deadbeat state weights
/// only DELAY the pitch tumble 4x (duty=0.34 survives ~10s, tumbles by
/// 15s) because a quadratic state cost can only damp, not null, the
/// per-cycle pitch-momentum accumulation. This shifts the touchdown
/// fore-aft by the measured pitch RATE so the next stance's GRF moment
/// nulls the momentum at the Poincaré section. Sign is unknown (Sec.5f
/// pitch convention is antiphase to euler_angles), so both signs are
/// swept. Runs 15s at duty=0.34 with the rate-deadbeat(100,100) still on;
/// a gain that keeps the body upright the full 15s (no roll=pi, min_z>0)
/// is the first PERMANENTLY stable energetic Bound.
#[test]
#[ignore = "exploratory stress test — run with --ignored"]
fn go2_wbc_bound_energetic_pitch_deadbeat_placement() {
    for k_rate in [0.03_f64, 0.045, 0.06, 0.08] {
        let cfg = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
        let params = WbcParams {
            cmd_vx: 0.50,
            total_time_s: 25.0,
            burn_in_s: 0.5,
            gait_type_override: Some(GaitType::Bound),
            duty_factor_override: Some(0.34),
            gait_cycle_period_override: Some(0.30),
            max_step_length_override: Some(0.18),
            swing_height_override: Some(0.10),
            bound_trim_reference: None,
            sync_real_mass_inertia: true,
            yaw_pd_gain_override: Some((10.0, 1.0)),
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
                roll_pitch_weight_override: Some((4.0, 4.0)),
                bound_fore_aft_placement_gain_override: None,
                roll_rate_weight_override: Some((100.0, 100.0)),
                bound_pitch_placement_gain_override: Some((0.0, k_rate)),
            bound_pitch_placement_dc_tau_override: None,
            bound_tabulated_reference_csv: None,
            bound_prescribed_footholds_override: None,
            }),
            ..WbcParams::forward_walk_misa_wbc(wbc::Formulation::ForceSpace, cfg)
        };
        if let Some(samples) = run_wbc_sim(params) {
            report_time_windowed_summary(
                &format!("PITCH-DEADBEAT placement k_rate={k_rate}, duty=0.34, rate-deadbeat(100,100), 15s"),
                &samples, 1.0,
            );
        }
    }
}

/// Sec.5f6 video source: the PERMANENTLY STABLE energetic Bound.
/// k_rate=0.045 / duty=0.34 stayed upright the full 25s (83 cycles) with
/// real air time (z_range up to 0.22m, min_z always positive) via the
/// Poincaré/deadbeat pitch foot-placement. Single-trial, 10s, for a
/// `WBC_WALK_CSV_OUT` trace → `render_go2_walk.py`:
///   WBC_WALK_CSV_OUT=/tmp/bound_energetic_stable.csv \
///     cargo xtask test --release --features mujoco --test wbc_walk_go2 \
///     -- go2_wbc_bound_energetic_stable_video_source --ignored --nocapture
#[test]
#[ignore = "exploratory stress test — run with --ignored; also the WBC_WALK_CSV_OUT video-capture source for Sec.5f6"]
fn go2_wbc_bound_energetic_stable_video_source() {
    let cfg = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
    let params = WbcParams {
        cmd_vx: 0.50,
        total_time_s: 10.0,
        burn_in_s: 0.5,
        gait_type_override: Some(GaitType::Bound),
        duty_factor_override: Some(0.34),
        gait_cycle_period_override: Some(0.30),
        max_step_length_override: Some(0.18),
        swing_height_override: Some(0.10),
        bound_trim_reference: None,
        sync_real_mass_inertia: true,
        yaw_pd_gain_override: Some((10.0, 1.0)),
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
            roll_pitch_weight_override: Some((4.0, 4.0)),
            bound_fore_aft_placement_gain_override: None,
            roll_rate_weight_override: Some((100.0, 100.0)),
            bound_pitch_placement_gain_override: Some((0.0, 0.045)),
        bound_pitch_placement_dc_tau_override: None,
        bound_tabulated_reference_csv: None,
        bound_prescribed_footholds_override: None,
        }),
        ..WbcParams::forward_walk_misa_wbc(wbc::Formulation::ForceSpace, cfg)
    };
    if let Some(samples) = run_wbc_sim(params) {
        report_time_windowed_summary(
            "STABLE energetic Bound (video), k_rate=0.045, duty=0.34, 10s",
            &samples, 1.0,
        );
    }
}

/// Sec.5f7 (1b): give the stable energetic Bound FORWARD motion. Sec.5f6
/// stabilized attitude (k_rate=0.045 pitch-deadbeat) but the body drifts
/// BACKWARD (~0.5 m/s) despite cmd_vx=+0.5 -- the pitch-deadbeat shifts
/// feet forward to make the nose-down moment, and the reaction pushes the
/// body back (Raibert coupling). Compose the fore-aft speed regulator
/// (bound_fore_aft_placement_gain, Sec.5c6) on top: it OVERWRITES the
/// neutral half.x with `v_filt·½T_st + k·(v_filt−cmd)` (driving measured
/// speed toward +cmd), then the pitch-deadbeat ADDS its correction. Sweep
/// the speed gain at the stable pitch-deadbeat, cmd_vx=0.5, 15s: find the
/// gain that yields net FORWARD vx while staying upright.
#[test]
#[ignore = "exploratory stress test — run with --ignored"]
fn go2_wbc_bound_energetic_forward() {
    // touch_down-only deadbeat (Sec.5f7): (k_rate, speed_gain) pairs.
    // (_, 0.0) checks stability + residual drift of the decoupled form;
    // (_, >0) adds the speed regulator to drive net forward.
    for (k_rate, k_speed) in [(0.045_f64, 0.0_f64), (0.045, 0.05), (0.06, 0.05), (0.06, 0.10)] {
        let cfg = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
        let speed_override = if k_speed > 0.0 { Some(k_speed) } else { None };
        let params = WbcParams {
            cmd_vx: 0.50,
            total_time_s: 15.0,
            burn_in_s: 0.5,
            gait_type_override: Some(GaitType::Bound),
            duty_factor_override: Some(0.34),
            gait_cycle_period_override: Some(0.30),
            max_step_length_override: Some(0.18),
            swing_height_override: Some(0.10),
            bound_trim_reference: None,
            sync_real_mass_inertia: true,
            yaw_pd_gain_override: Some((10.0, 1.0)),
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
                roll_pitch_weight_override: Some((4.0, 4.0)),
                bound_fore_aft_placement_gain_override: speed_override,
                roll_rate_weight_override: Some((100.0, 100.0)),
                bound_pitch_placement_gain_override: Some((0.0, k_rate)),
            bound_pitch_placement_dc_tau_override: None,
            bound_tabulated_reference_csv: None,
            bound_prescribed_footholds_override: None,
            }),
            ..WbcParams::forward_walk_misa_wbc(wbc::Formulation::ForceSpace, cfg)
        };
        if let Some(samples) = run_wbc_sim(params) {
            report_time_windowed_summary(
                &format!("FORWARD(td-only) k_rate={k_rate} speed_gain={k_speed}, duty=0.34, 15s"),
                &samples, 1.0,
            );
        }
    }
}

/// Sec.5f7b (1b): decouple forward DRIVE from pitch CONTROL. Sec.5f7
/// showed the pitch-deadbeat foot-placement inherently drives the body
/// BACKWARD (feet must shift forward to make the nose-down moment, and
/// at the stabilizing gain the shift saturates the step envelope). The
/// speed regulator can't reverse it without flattening the bounce because
/// it fights over the SAME foothold. Physically correct decoupling:
/// forward propulsion from the horizontal GRF (bound_fx_thrust_bias, a
/// constant forward stance thrust, Sec.5d7), attitude from the placement.
/// Sweep the forward thrust at the stable pitch-deadbeat (k_rate=0.045,
/// touch_down-only), cmd_vx=0.5, 15s: find a thrust that yields net
/// FORWARD vx while the placement keeps it upright.
#[test]
#[ignore = "exploratory stress test — run with --ignored"]
fn go2_wbc_bound_energetic_forward_thrust() {
    for fx in [20.0_f64, 40.0, 60.0, 90.0] {
        let cfg = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
        let params = WbcParams {
            cmd_vx: 0.50,
            total_time_s: 15.0,
            burn_in_s: 0.5,
            gait_type_override: Some(GaitType::Bound),
            duty_factor_override: Some(0.34),
            gait_cycle_period_override: Some(0.30),
            max_step_length_override: Some(0.18),
            swing_height_override: Some(0.10),
            bound_trim_reference: None,
            sync_real_mass_inertia: true,
            yaw_pd_gain_override: Some((10.0, 1.0)),
            bound_fx_thrust_bias_override: Some(fx),
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
                roll_pitch_weight_override: Some((4.0, 4.0)),
                bound_fore_aft_placement_gain_override: None,
                roll_rate_weight_override: Some((100.0, 100.0)),
                bound_pitch_placement_gain_override: Some((0.0, 0.045)),
            bound_pitch_placement_dc_tau_override: None,
            bound_tabulated_reference_csv: None,
            bound_prescribed_footholds_override: None,
            }),
            ..WbcParams::forward_walk_misa_wbc(wbc::Formulation::ForceSpace, cfg)
        };
        if let Some(samples) = run_wbc_sim(params) {
            report_time_windowed_summary(
                &format!("FORWARD-THRUST energetic Bound, fx_bias={fx}N, k_rate=0.045, duty=0.34, 15s"),
                &samples, 1.0,
            );
        }
    }
}

/// Sec.5f8 (1b): HZD-flavored FORWARD energetic Bound. Sec.5f7 proved
/// the pitch-deadbeat keyed on ABSOLUTE pitch (implicit nominal = static
/// in-place orbit) saturates the foothold forward and drags the body
/// backward. Fix: run the TRIM line (a closed-form FORWARD-moving pitch/Fx
/// orbit) and make the deadbeat correct only the DEVIATION from that
/// orbit's nominal pitch/pitch_rate (now automatic when a trim orbit is
/// active). On the orbit the correction vanishes, so the foot sits at the
/// forward-neutral point and the trim's Fx drives forward, while the
/// deadbeat stabilizes deviations. k_rate=0 is the trim-only baseline
/// (does it move forward? is it stable without the deadbeat?).
#[test]
#[ignore = "exploratory stress test — run with --ignored"]
fn go2_wbc_bound_energetic_hzd_forward() {
    for k_rate in [0.0_f64, 0.03, 0.045, 0.06] {
        let cfg = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
        let params = WbcParams {
            cmd_vx: 0.50,
            total_time_s: 15.0,
            burn_in_s: 0.5,
            gait_type_override: Some(GaitType::Bound),
            duty_factor_override: Some(0.34),
            gait_cycle_period_override: Some(0.30),
            max_step_length_override: Some(0.18),
            swing_height_override: Some(0.10),
            bound_trim_reference: Some((100.0, 10.0)),
            bound_trim_thrust_scale_override: Some(1.0),
            sync_real_mass_inertia: true,
            yaw_pd_gain_override: Some((10.0, 1.0)),
            adaptive_cycle_period: Some(AdaptivePeriodConfig {
                gain: 0.10,
                update_interval_s: 0.2,
                min_period_s: 0.28,
                max_period_s: 0.32,
            }),
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
                roll_pitch_weight_override: Some((4.0, 4.0)),
                bound_fore_aft_placement_gain_override: None,
                roll_rate_weight_override: Some((100.0, 100.0)),
                bound_pitch_placement_gain_override: Some((0.0, k_rate)),
            bound_pitch_placement_dc_tau_override: None,
            bound_tabulated_reference_csv: None,
            bound_prescribed_footholds_override: None,
            }),
            ..WbcParams::forward_walk_misa_wbc(wbc::Formulation::ForceSpace, cfg)
        };
        if let Some(samples) = run_wbc_sim(params) {
            report_time_windowed_summary(
                &format!("HZD-FORWARD (trim orbit) k_rate={k_rate}, duty=0.34, cmd_vx=0.5, 15s"),
                &samples, 1.0,
            );
        }
    }
}

/// Sec.5f8b (1b): DC-blocker sweep to kill the residual backward drift.
/// Sec.5f8 (orbit-relative deadbeat on the trim line) stayed stable but
/// still drifted backward -- the trim closed-form nominal ≠ the real
/// orbit's pitch_rate at the sampled phase, leaving a persistent forward
/// foothold bias. The DC-blocker (slow EMA of the applied shift,
/// subtracted) removes exactly that bias while keeping the AC
/// deviation-stabilizing part. Sweep tau at the stable k_rate=0.045 trim
/// config: tau=0 is the Sec.5f8 baseline (backward); a good tau should
/// give net FORWARD (or at least zero-drift) vx while staying upright.
#[test]
#[ignore = "exploratory stress test — run with --ignored"]
fn go2_wbc_bound_energetic_hzd_dcblock() {
    for tau in [0.0_f64, 0.4, 0.8, 1.5] {
        let cfg = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
        let params = WbcParams {
            cmd_vx: 0.50,
            total_time_s: 15.0,
            burn_in_s: 0.5,
            gait_type_override: Some(GaitType::Bound),
            duty_factor_override: Some(0.34),
            gait_cycle_period_override: Some(0.30),
            max_step_length_override: Some(0.18),
            swing_height_override: Some(0.10),
            bound_trim_reference: Some((100.0, 10.0)),
            bound_trim_thrust_scale_override: Some(1.0),
            sync_real_mass_inertia: true,
            yaw_pd_gain_override: Some((10.0, 1.0)),
            adaptive_cycle_period: Some(AdaptivePeriodConfig {
                gain: 0.10,
                update_interval_s: 0.2,
                min_period_s: 0.28,
                max_period_s: 0.32,
            }),
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
                roll_pitch_weight_override: Some((4.0, 4.0)),
                bound_fore_aft_placement_gain_override: None,
                roll_rate_weight_override: Some((100.0, 100.0)),
                bound_pitch_placement_gain_override: Some((0.0, 0.045)),
                bound_pitch_placement_dc_tau_override: Some(tau),
            bound_tabulated_reference_csv: None,
            bound_prescribed_footholds_override: None,
            }),
            ..WbcParams::forward_walk_misa_wbc(wbc::Formulation::ForceSpace, cfg)
        };
        if let Some(samples) = run_wbc_sim(params) {
            report_time_windowed_summary(
                &format!("HZD-DCBLOCK tau={tau}, k_rate=0.045, duty=0.34, cmd_vx=0.5, 15s"),
                &samples, 1.0,
            );
        }
    }
}

/// Sec.5f9 (P2): track the trajopt forward-Bound reference orbit. P0
/// confirmed a feasible FORWARD periodic Bound orbit exists (vx=1.0,
/// pitch<0.16, friction ok); this feeds that orbit as the MPC reference
/// (via bound_tabulated_reference_csv) so the MPC has a CONSISTENT
/// forward target -- the missing piece §5f7/5f8 identified. cmd_vx matches
/// the orbit's design speed (1.0). Deadbeat (orbit-relative, now against
/// the tabulated orbit) + roll-rate weight for stability. Sweeps the
/// pitch-deadbeat gain incl. 0 (does the reference alone move it forward?).
#[test]
#[ignore = "exploratory stress test — run with --ignored"]
fn go2_wbc_bound_trajopt_forward() {
    // Make the MPC actively PURSUE forward: raise base_pos.x weight
    // (q_diag[6], default 0 -> the forward-advancing position reference is
    // otherwise unpenalized) so it tracks the forward orbit via GRF, and
    // DROP the placement deadbeat (its foothold-forward shift kinematically
    // drags the body backward, §5f7). Stabilize pitch/roll via the
    // state-weight rate deadbeat (no kinematic drag) against the feasible
    // tabulated reference. Sweep (base_pos.x weight, rate weight).
    for (q_px, q_rate) in [(10.0_f64, 150.0_f64), (30.0, 150.0), (30.0, 400.0), (80.0, 400.0)] {
        let cfg = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
        let params = WbcParams {
            cmd_vx: 1.00,
            total_time_s: 12.0,
            burn_in_s: 0.5,
            gait_type_override: Some(GaitType::Bound),
            duty_factor_override: Some(0.34),
            gait_cycle_period_override: Some(0.30),
            max_step_length_override: Some(0.18),
            swing_height_override: Some(0.10),
            bound_trim_reference: None,
            sync_real_mass_inertia: true,
            yaw_pd_gain_override: Some((10.0, 1.0)),
            full_centroidal: Some(FullCentroidalOpts {
                legged_control_parity: true,
                use_mpc_predicted_footstep: false,
                dynamic_joint_q_reference: false,
                mpc_override: None,
                task_space_joint_vel_weight: None,
                true_centroidal_coupling: false,
                capture_point_gain_override: Some(0.0),
                base_pos_xy_weight_override: Some((q_px, 5.0)),
                max_normal_force_override: None,
                roll_pitch_weight_override: Some((4.0, 25.0)),
                bound_fore_aft_placement_gain_override: None,
                roll_rate_weight_override: Some((100.0, q_rate)),
                bound_pitch_placement_gain_override: None,
                bound_pitch_placement_dc_tau_override: None,
                bound_tabulated_reference_csv: Some("ref/scripts/bound_p0_orbit.csv"),
            bound_prescribed_footholds_override: None,
            }),
            ..WbcParams::forward_walk_misa_wbc(wbc::Formulation::ForceSpace, cfg)
        };
        if let Some(samples) = run_wbc_sim(params) {
            report_time_windowed_summary(
                &format!("TRAJOPT-REF pursue-fwd, q_px={q_px} q_rate={q_rate}, no-placement, cmd_vx=1.0, 12s"),
                &samples, 1.0,
            );
        }
    }
}

/// Sec.5f9 (P1->P2): track the RUST-generated periodic orbit. The stage-1
/// solver (quadruped_gait::solve_bound_orbit) produces a clean low-pitch
/// forward orbit (pitch~0.02, vx=1.0, friction margin +0.31) -- a much
/// EASIER reference to track than the higher-pitch P0 orbit. A low-pitch
/// forward orbit needs little attitude authority in flight, so it may
/// sidestep the §5f10 stabilize-vs-forward dilemma (which was driven by the
/// pitch tumble + the backward-dragging pitch deadbeat). This test
/// generates the orbit in-process, writes it to CSV, and tracks it with
/// base_pos.x weight (pursue forward) + rate state-weight (stabilize),
/// NO placement deadbeat (no backward drag).
#[test]
#[ignore = "exploratory stress test — run with --ignored"]
fn go2_wbc_bound_p1_orbit_forward() {
    // stage 1: generate the orbit in Rust and dump it to a CSV the harness
    // can load (FullCentroidalOpts is Copy, so it can't carry the Vec).
    let params = quadruped_gait::PeriodicBoundParams::go2(1.0, 0.30, 0.34);
    let orbit = quadruped_gait::solve_bound_orbit(&params).expect("P1 orbit");
    let path = "ref/scripts/bound_p1_orbit.csv";
    let mut csv = String::from("phase,z,pitch,vx,vz,w\n");
    for r in &orbit.table {
        csv.push_str(&format!(
            "{:.5},{:.6},{:.6},{:.6},{:.6},{:.6}\n",
            r[0], r[1], r[2], r[3], r[4], r[5]
        ));
    }
    std::fs::write(path, csv).expect("write P1 orbit csv");
    eprintln!(
        "[P1] orbit periodicity={:.2e} friction={:.3} rows={}",
        orbit.periodicity_residual, orbit.friction_margin, orbit.table.len()
    );

    for (q_px, q_rate) in [(30.0_f64, 150.0_f64), (30.0, 400.0)] {
        let cfg = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
        let p = WbcParams {
            cmd_vx: 1.00,
            total_time_s: 12.0,
            burn_in_s: 0.5,
            gait_type_override: Some(GaitType::Bound),
            duty_factor_override: Some(0.34),
            gait_cycle_period_override: Some(0.30),
            max_step_length_override: Some(0.18),
            swing_height_override: Some(0.10),
            bound_trim_reference: None,
            sync_real_mass_inertia: true,
            yaw_pd_gain_override: Some((10.0, 1.0)),
            full_centroidal: Some(FullCentroidalOpts {
                legged_control_parity: true,
                use_mpc_predicted_footstep: false,
                dynamic_joint_q_reference: false,
                mpc_override: None,
                task_space_joint_vel_weight: None,
                true_centroidal_coupling: false,
                capture_point_gain_override: Some(0.0),
                base_pos_xy_weight_override: Some((q_px, 5.0)),
                max_normal_force_override: None,
                roll_pitch_weight_override: Some((4.0, 25.0)),
                bound_fore_aft_placement_gain_override: None,
                roll_rate_weight_override: Some((100.0, q_rate)),
                bound_pitch_placement_gain_override: None,
                bound_pitch_placement_dc_tau_override: None,
                bound_tabulated_reference_csv: Some("ref/scripts/bound_p1_orbit.csv"),
            bound_prescribed_footholds_override: None,
            }),
            ..WbcParams::forward_walk_misa_wbc(wbc::Formulation::ForceSpace, cfg)
        };
        if let Some(samples) = run_wbc_sim(p) {
            report_time_windowed_summary(
                &format!("P1-ORBIT track, q_px={q_px} q_rate={q_rate}, cmd_vx=1.0, duty=0.34, 12s"),
                &samples, 1.0,
            );
        }
    }
}

/// Sec.5f9 (P3-a): FOLLOW THE ORBIT'S FOOTHOLDS. The core hypothesis: the
/// trajopt orbit's own footholds are the placement that is forward-moving
/// AND pitch-balanced by construction, so following them directly should
/// give forward + stable where Raibert+deadbeat could not (§5f10: the two
/// fought). Generates the P1 orbit, installs its base-state reference AND
/// its prescribed (front,rear) footholds, no placement deadbeat. Rate
/// state-weight kept modest for perturbation damping. cmd_vx = orbit speed.
#[test]
#[ignore = "exploratory stress test — run with --ignored"]
fn go2_wbc_bound_p3a_orbit_footholds() {
    let params = quadruped_gait::PeriodicBoundParams::go2(1.0, 0.30, 0.34);
    let orbit = quadruped_gait::solve_bound_orbit(&params).expect("P3a orbit");
    let path = "ref/scripts/bound_p1_orbit.csv";
    let mut csv = String::from("phase,z,pitch,vx,vz,w\n");
    for r in &orbit.table {
        csv.push_str(&format!("{:.5},{:.6},{:.6},{:.6},{:.6},{:.6}\n",
            r[0], r[1], r[2], r[3], r[4], r[5]));
    }
    std::fs::write(path, csv).expect("write orbit csv");
    eprintln!("[P3a] footholds front={:.3} rear={:.3} friction={:.3}",
        orbit.front_foothold, orbit.rear_foothold, orbit.friction_margin);

    // P3-b: orbit foothold NEUTRAL + landing-reflex deadbeat correction.
    // Pure open-loop foothold following (k_place=0) collapses immediately;
    // the reflex (pitch-rate deadbeat around the forward orbit foothold)
    // supplies the missing feedback. Sweep the reflex gain.
    for k_place in [0.0_f64, 0.02, 0.045, 0.08] {
        let cfg = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
        let p = WbcParams {
            cmd_vx: 1.00,
            total_time_s: 12.0,
            burn_in_s: 0.5,
            gait_type_override: Some(GaitType::Bound),
            duty_factor_override: Some(0.34),
            gait_cycle_period_override: Some(0.30),
            max_step_length_override: Some(0.22),
            swing_height_override: Some(0.10),
            bound_trim_reference: None,
            sync_real_mass_inertia: true,
            yaw_pd_gain_override: Some((10.0, 1.0)),
            full_centroidal: Some(FullCentroidalOpts {
                legged_control_parity: true,
                use_mpc_predicted_footstep: false,
                dynamic_joint_q_reference: false,
                mpc_override: None,
                task_space_joint_vel_weight: None,
                true_centroidal_coupling: false,
                capture_point_gain_override: Some(0.0),
                base_pos_xy_weight_override: Some((20.0, 5.0)),
                max_normal_force_override: None,
                roll_pitch_weight_override: Some((4.0, 25.0)),
                bound_fore_aft_placement_gain_override: None,
                roll_rate_weight_override: Some((100.0, 100.0)),
                bound_pitch_placement_gain_override: Some((0.0, k_place)),
                bound_pitch_placement_dc_tau_override: None,
                bound_tabulated_reference_csv: Some("ref/scripts/bound_p1_orbit.csv"),
                bound_prescribed_footholds_override: Some((orbit.front_foothold, orbit.rear_foothold)),
            }),
            ..WbcParams::forward_walk_misa_wbc(wbc::Formulation::ForceSpace, cfg)
        };
        if let Some(samples) = run_wbc_sim(p) {
            report_time_windowed_summary(
                &format!("P3b orbit-foothold+reflex, k_place={k_place}, cmd_vx=1.0, duty=0.34, 12s"),
                &samples, 1.0,
            );
        }
    }
}

/// Sec.5f9 (P3 confirm): the forward + stable Bound. P3-a found that
/// following the trajopt orbit's own (reachability-clamped) footholds --
/// with NO placement deadbeat (k_place=0) -- gives forward + stable, where
/// every §5f bolt-on failed (the deadbeat itself was the backward-drag
/// culprit). This confirms it over a longer 25s horizon and dumps a CSV
/// for video. Guards against the §5f4-style premature "stable" claim.
#[test]
#[ignore = "exploratory stress test — run with --ignored; WBC_WALK_CSV_OUT video source for P3"]
fn go2_wbc_bound_p3_forward_stable_confirm() {
    let params = quadruped_gait::PeriodicBoundParams::go2(1.0, 0.30, 0.34);
    let orbit = quadruped_gait::solve_bound_orbit(&params).expect("orbit");
    let mut csv = String::from("phase,z,pitch,vx,vz,w\n");
    for r in &orbit.table {
        csv.push_str(&format!("{:.5},{:.6},{:.6},{:.6},{:.6},{:.6}\n",
            r[0], r[1], r[2], r[3], r[4], r[5]));
    }
    std::fs::write("ref/scripts/bound_p1_orbit.csv", csv).expect("csv");

    let cfg = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
    let p = WbcParams {
        cmd_vx: 1.00,
        total_time_s: 25.0,
        burn_in_s: 0.5,
        gait_type_override: Some(GaitType::Bound),
        duty_factor_override: Some(0.34),
        gait_cycle_period_override: Some(0.30),
        max_step_length_override: Some(0.22),
        swing_height_override: Some(0.10),
        bound_trim_reference: None,
        sync_real_mass_inertia: true,
        yaw_pd_gain_override: Some((10.0, 1.0)),
        full_centroidal: Some(FullCentroidalOpts {
            legged_control_parity: true,
            use_mpc_predicted_footstep: false,
            dynamic_joint_q_reference: false,
            mpc_override: None,
            task_space_joint_vel_weight: None,
            true_centroidal_coupling: false,
            capture_point_gain_override: Some(0.0),
            base_pos_xy_weight_override: Some((20.0, 5.0)),
            max_normal_force_override: None,
            roll_pitch_weight_override: Some((4.0, 25.0)),
            bound_fore_aft_placement_gain_override: None,
            roll_rate_weight_override: Some((100.0, 100.0)),
            bound_pitch_placement_gain_override: None,
            bound_pitch_placement_dc_tau_override: None,
            bound_tabulated_reference_csv: Some("ref/scripts/bound_p1_orbit.csv"),
            bound_prescribed_footholds_override: Some((orbit.front_foothold, orbit.rear_foothold)),
        }),
        ..WbcParams::forward_walk_misa_wbc(wbc::Formulation::ForceSpace, cfg)
    };
    if let Some(samples) = run_wbc_sim(p) {
        report_time_windowed_summary(
            "P3 FORWARD+STABLE confirm (orbit footholds, no deadbeat), cmd_vx=1.0, 25s",
            &samples, 1.0,
        );
    }
}
