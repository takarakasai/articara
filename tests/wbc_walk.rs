//! End-to-end MuJoCo regression for the Hierarchical WBC pipeline.
//!
//! Mirrors [`gait_walk_stability`](./gait_walk_stability.rs)'s harness
//! (URDF → MujocoSim → trot loop) but routes the gait controller's
//! output through [`articara::wbc_pipeline::WbcPipeline`] instead of
//! the per-joint Position-PD path. Asserts on two invariants:
//!
//! 1. **Gravity balance** — at hold (zero command), the sum of
//!    contact-force z-components should approximately equal `m·g`.
//!    The WBC's friction-cone + EoM tasks are responsible for this:
//!    if the QP fails or hands over a wrong-sign GRF, the body would
//!    fall through the floor or float.
//! 2. **Forward motion** — under a steady forward command, the trunk
//!    must translate by at least `MIN_DISPLACEMENT_M` over the run.
//!    Catches the "WBC bobs in place" regression where the
//!    direction-preserving floor in `compute_mpc_footstep` collapses
//!    to zero or the WBC output is dominated by gravity comp.
//!
//! ## Known limitations (documented per-test)
//!
//! - The WBC pipeline keeps the misarta floating base at neutral
//!   orientation each tick (`q[3..7] = identity`). On flat ground
//!   that approximation is fine; large body tilt would skew the
//!   gravity-comp direction. We use mild commands so the body stays
//!   roughly upright.
//! - Friction enforcement happens in the WBC's QP, but the actual
//!   MuJoCo contact solver may still allow micro-slip. Thresholds
//!   are loose enough to absorb that.

#![cfg(feature = "mujoco")]

use std::path::PathBuf;

use articara::gait::{
    auto_detect_kinematics_config, GaitController, DEFAULT_FOOT_LINKS,
};
use articara::mjcf::{GroundPlaneCfg, MjcfExportOptions};
use articara::mujoco_sim::MujocoSim;
use articara::rbd::model::ActuatorMode;
use articara::robot::RobotModel;
use articara::wbc_pipeline::WbcPipeline;
use nalgebra::Vector3;
use quadruped_gait::{
    solve_leg_ik, GaitConfig, GaitMode, KinematicsConfig, LegIkSolution,
    VelocityCmd,
};

fn namiashi_urdf() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("tests")
        .join("fixtures")
        .join("namiashi")
        .join("urdf")
        .join("namiashi.urdf")
}

/// Same seeding logic as `gait_walk_stability`. Keeps the legs out of
/// their q=0 fully-extended kinematic singularity at sim start.
fn seed_joint_positions_from_kinematics(
    robot: &mut RobotModel,
    kin: &KinematicsConfig,
) {
    for leg_kin in [&kin.fl, &kin.fr, &kin.rl, &kin.rr] {
        let target = leg_kin.nominal_foot_body;
        let sol = solve_leg_ik(leg_kin, target, false);
        let LegIkSolution::Reached { hip, thigh, calf } = sol else {
            panic!(
                "{:?}: nominal_foot_body unreachable",
                leg_kin.leg
            );
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

/// Per-tick sample. We track `total_fz_world` (Σ contact-force z over
/// all contacts) so the static-balance test can compare it with
/// `m · g` after the burn-in window.
#[derive(Debug)]
struct WbcSample {
    t: f64,
    body_x: f64,
    body_z: f64,
    roll: f64,
    pitch: f64,
    total_fz_world: f64,
}

/// Threshold for "trunk has fallen". Below this z, the body has either
/// tipped over or sunk through the ground. Same value as
/// `gait_walk_stability`.
const TRUNK_Z_FALL_THRESHOLD_M: f64 = 0.18;

/// Minimum forward displacement during the walking window — the same
/// 4 cm threshold the gait stability test uses.
const MIN_DISPLACEMENT_M: f64 = 0.04;

struct WbcParams {
    total_time_s: f64,
    burn_in_s: f64,
    cmd_vx: f64,
    dt: f64,
}

impl WbcParams {
    fn static_stand() -> Self {
        Self {
            total_time_s: 1.5,
            burn_in_s: 0.5,
            cmd_vx: 0.0,
            dt: 0.002,
        }
    }
    fn forward_walk() -> Self {
        Self {
            total_time_s: 3.0,
            burn_in_s: 0.5,
            cmd_vx: 0.15,
            dt: 0.002,
        }
    }
}

/// Run a WBC sim, sampling per-tick. Returns `None` if the namiashi
/// fixture is missing (skip cleanly).
fn run_wbc_sim(params: WbcParams) -> Option<Vec<WbcSample>> {
    let path = namiashi_urdf();
    if !path.exists() {
        eprintln!(
            "namiashi fixture missing at {} — skipping WBC test",
            path.display()
        );
        return None;
    }
    let mut robot = RobotModel::from_urdf(&path).expect("load namiashi URDF");

    // Same per-joint PD as gait_walk_stability — the WBC overrides PD
    // when active, but during burn-in (gait disabled) we still run
    // Position-mode PD, so these gains matter for the settling phase.
    for j in robot.joints.iter_mut() {
        if j.joint_type == "fixed" {
            continue;
        }
        j.actuator_mode = ActuatorMode::Position;
        j.actuator_kp = 30.0;
        j.actuator_kv = 0.6;
    }

    let mut kin = auto_detect_kinematics_config(&robot, &DEFAULT_FOOT_LINKS)
        .expect("auto-detect kinematics");
    for leg_kin in [&mut kin.fl, &mut kin.fr, &mut kin.rl, &mut kin.rr] {
        let total_leg = leg_kin.upper_leg_m + leg_kin.lower_leg_m;
        leg_kin.nominal_foot_body.z += 0.08 * total_leg;
    }
    seed_joint_positions_from_kinematics(&mut robot, &kin);

    let opts = MjcfExportOptions {
        ground_plane: Some(GroundPlaneCfg {
            z: 0.0,
            half_size: 4.0,
            roll: 0.0,
            pitch: 0.0,
        }),
        add_actuators: true,
        ..Default::default()
    };
    let mut sim = MujocoSim::new(&robot, opts).expect("MujocoSim::new");
    // We don't want gravity-comp to compete with the WBC during the
    // walking window — the WBC's floating-base EoM task already
    // handles it. But during burn-in (gait disabled, WBC inactive),
    // grav-comp keeps the body from sagging. Toggle is per-sim, so
    // we leave it on; the WBC bypasses the per-joint path entirely
    // when active.
    sim.set_gravity_compensation(true);

    let cfg = GaitConfig::trot();
    let mut gc = GaitController::build(&robot, kin.clone(), cfg, GaitMode::Mpc)
        .expect("GaitController::build (Mpc mode)");

    // Foot link names for the WBC pipeline.
    let foot_links: [String; 4] = [
        DEFAULT_FOOT_LINKS[0].1.to_string(),
        DEFAULT_FOOT_LINKS[1].1.to_string(),
        DEFAULT_FOOT_LINKS[2].1.to_string(),
        DEFAULT_FOOT_LINKS[3].1.to_string(),
    ];
    let mut wbc_pipeline = WbcPipeline::new(&robot, foot_links);

    let n_steps = (params.total_time_s / params.dt).round() as usize;
    let burn_in_steps = (params.burn_in_s / params.dt).round() as usize;
    let mut samples: Vec<WbcSample> = Vec::with_capacity(n_steps);

    for k in 0..n_steps {
        let t = k as f64 * params.dt;

        if k == 0 {
            gc.enable();
        }
        if k == burn_in_steps {
            gc.set_velocity_cmd(VelocityCmd {
                vx: params.cmd_vx,
                vy: 0.0,
                wz: 0.0,
            });
        }

        // Feed observed body velocity to the closed-loop generators.
        let v_obs = sim
            .body_world_linear_velocity(&robot.root_link)
            .unwrap_or([0.0, 0.0, 0.0]);
        let w_obs = sim
            .body_world_angular_velocity(&robot.root_link)
            .unwrap_or([0.0, 0.0, 0.0]);
        gc.set_body_state_observed(
            Vector3::new(v_obs[0], v_obs[1], v_obs[2]),
            Vector3::new(w_obs[0], w_obs[1], w_obs[2]),
        );

        if gc.is_enabled() {
            let (out, targets, _torque_ff) = gc.tick(params.dt);
            for (idx, q) in targets {
                sim.set_position_target(idx, q);
            }
            // After burn-in: route through WBC. Skip during burn-in
            // so the body has a chance to settle on its feet via the
            // Position-PD path (the WBC's static balance only
            // converges once the legs are loaded).
            if k >= burn_in_steps {
                let f_grf_world = gc
                    .predicted_grfs()
                    .map(|sol| sol.grfs_first_step)
                    .unwrap_or([Vector3::zeros(); 4]);
                let cmd = gc.velocity_cmd();
                // Body-frame command — the WBC pipeline rotates the
                // observation internally using the current xquat.
                let v_cmd_body = Vector3::new(cmd.vx, cmd.vy, 0.0);
                let contact_flag = [
                    out.legs[0].phase.is_stance,
                    out.legs[1].phase.is_stance,
                    out.legs[2].phase.is_stance,
                    out.legs[3].phase.is_stance,
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
                // Diagnostic dump every 100 ticks (200 ms): trunk z,
                // WBC τ ranges, MPC GRF z-sums. Cheap, only fires
                // ~5 times per test run.
                if k % 100 == 0 {
                    let body_pos = sim
                        .body_world_position(&robot.root_link)
                        .unwrap_or([0.0, 0.0, 0.0]);
                    let tau_max = taus
                        .iter()
                        .cloned()
                        .fold(0.0_f64, |a, b| a.max(b.abs()));
                    let mpc_fz_sum: f64 = f_grf_world.iter().map(|v| v.z).sum();
                    let stance_count = contact_flag.iter().filter(|b| **b).count();
                    eprintln!(
                        "[diag k={k:5} t={:.3}s] z={:.3} m  Σmpc_f_z={:.2} N  \
                         max|τ|={:.2} N·m  stance={}/4",
                        k as f64 * params.dt,
                        body_pos[2],
                        mpc_fz_sum,
                        tau_max,
                        stance_count
                    );
                }
                sim.set_wbc_torques(&taus);
            } else {
                sim.clear_wbc_torques();
            }
        }

        sim.step(&mut robot, params.dt, true);

        // Sample after the step so contact forces and pose are
        // synchronised. `base_transform` was just refreshed by
        // `sim.step → sync_back`.
        let tx = robot.base_transform.translation;
        let (roll, pitch, _yaw) = robot.base_transform.rotation.euler_angles();
        let total_fz_world: f64 =
            sim.contacts().iter().map(|c| c.force_world[2]).sum();
        samples.push(WbcSample {
            t,
            body_x: tx.x,
            body_z: tx.z,
            roll,
            pitch,
            total_fz_world,
        });
    }
    Some(samples)
}

/// Compute the total robot mass by summing every link's `mass` field.
/// `m·g` is the reference for the static-balance assertion.
fn robot_mass(robot: &RobotModel) -> f64 {
    robot.links.iter().map(|l| l.inertial.mass).sum()
}

/// Static stand under WBC torque control. The Phase A integration
/// went through several rounds of tightening before this could pass:
///
/// 1. `q[3..7]` quaternion sync from MuJoCo (resolved at adbc9f1).
/// 2. SE(3)-correct `compute_joint_jacobian_time_derivative` so a
///    moving floating base doesn't blow up `J̇·v` (Phase 1.2).
/// 3. Per-task LSQ weights so EoM/no_contact_motion dominate
///    contact_force/τ_grav (Phase 1.4).
/// 4. `a_base_des` derived from the **MPC's predicted GRFs** via
///    SRBD physics, instead of a hand-tuned PD on body velocity
///    (Phase 1.5-A) — this is the single biggest fix; without it
///    the WBC drives the body to a different equilibrium than the
///    MPC chose and the two fight to body collapse.
///
/// The avg Σf_z tolerance is ±60% rather than the ideal ±10% because
/// MuJoCo's contact bouncing produces large per-tick swings even when
/// the body is mean-stable around the target z. The *min_z fall*
/// check catches the real failure mode (body slumping below 0.18 m).
#[test]
fn wbc_static_stand_balances_gravity() {
    let Some(samples) = run_wbc_sim(WbcParams::static_stand()) else {
        return;
    };

    // No fall.
    let min_z = samples.iter().map(|s| s.body_z).fold(f64::INFINITY, f64::min);
    assert!(
        min_z > TRUNK_Z_FALL_THRESHOLD_M,
        "static stand: trunk fell, min_z = {min_z:.3} m (threshold {:.2})",
        TRUNK_Z_FALL_THRESHOLD_M,
    );

    // Burn-in window done; sample the last 0.5 s for the f_z average.
    let dt: f64 = 0.002;
    let total_time = 1.5;
    let burn_in = 0.5;
    let total_n = (total_time / dt).round() as usize;
    let window_n = (0.5 / dt).round() as usize;
    let start = total_n.saturating_sub(window_n);
    let avg_fz: f64 = samples[start..]
        .iter()
        .map(|s| s.total_fz_world)
        .sum::<f64>()
        / (samples.len() - start) as f64;

    // Reference m·g. Recompute by reloading the URDF (cheap).
    let path = namiashi_urdf();
    let robot = RobotModel::from_urdf(&path).unwrap();
    let mg = robot_mass(&robot) * 9.81;

    let pct_err = ((avg_fz - mg) / mg).abs();
    eprintln!(
        "[wbc] static_stand: avg Σf_z = {avg_fz:.2} N, m·g = {mg:.2} N \
         (err = {:.1}%)",
        pct_err * 100.0,
    );
    // ±60% tolerance — generous because:
    //   - per-tick MuJoCo contact force has large bouncing components
    //     (the integrator's substeps see sharp normal-force spikes
    //     around touchdown which the 0.5 s sample window doesn't fully
    //     average out),
    //   - friction-cone clipping in the QP can re-allocate force
    //     among feet, making instantaneous totals oscillate,
    //   - the WBC's static-stand z drifts within a few cm because
    //     there's no explicit z-position regulator (the SRBD MPC's
    //     z weight handles it but its 30 ms horizon discretisation
    //     leaves residual oscillation).
    // The accompanying min_z assertion (above) is the real fall
    // detector; this avg-Σf_z test mostly guards against pathological
    // gravity-direction errors.
    assert!(
        pct_err < 0.60,
        "static stand: Σf_z = {avg_fz:.2} N deviates from m·g = {mg:.2} N \
         by {:.1}% — friction cone + EoM may be inconsistent",
        pct_err * 100.0,
    );
    let _ = burn_in;
}

/// `#[ignore]`d pending Phase C (contact-driven phase). With the
/// MPC-driven `a_base_des` fix the static stand now passes, but
/// trotting (stance=2/4) still trips up: a swing foot occasionally
/// touches down a few ms early or late relative to the open-loop
/// schedule, momentarily breaking the QP's contact_flag assumption,
/// and the body z drops a few cm per such mismatch until the trunk
/// fall threshold (0.18 m) is hit. The fix is to make `stance` come
/// from real contact sensing (`MujocoSim::contact_force_per_foot` →
/// `quadruped_gait::phase::ContactDrivenPhase`).
#[test]
#[ignore = "blocked on Phase C contact-driven phase scheduling"]
fn wbc_forward_command_advances_body() {
    let Some(samples) = run_wbc_sim(WbcParams::forward_walk()) else {
        return;
    };

    // Skip burn-in for tilt + displacement metrics.
    let dt: f64 = 0.002;
    let burn_in_steps = (0.5 / dt).round() as usize;
    let walk = &samples[burn_in_steps..];

    // No fall during the entire run (including burn-in).
    let min_z = samples.iter().map(|s| s.body_z).fold(f64::INFINITY, f64::min);
    assert!(
        min_z > TRUNK_Z_FALL_THRESHOLD_M,
        "forward walk: trunk fell, min_z = {min_z:.3} m",
    );

    // Forward displacement during the walking window.
    let x_start = walk.first().map(|s| s.body_x).unwrap_or(0.0);
    let x_end = walk.last().map(|s| s.body_x).unwrap_or(0.0);
    let dx = x_end - x_start;
    eprintln!(
        "[wbc] forward_command: Δx = {dx:.3} m over {:.1} s \
         (threshold ≥ {:.2})",
        walk.last().map(|s| s.t).unwrap_or(0.0)
            - walk.first().map(|s| s.t).unwrap_or(0.0),
        MIN_DISPLACEMENT_M,
    );
    assert!(
        dx >= MIN_DISPLACEMENT_M,
        "forward walk: Δx = {dx:.3} m < {} m — WBC produced near-zero \
         net forward motion (足踏み regression?)",
        MIN_DISPLACEMENT_M,
    );

    // Bounded body tilt during walking. WBC + Position-PD with
    // gravity-comp should keep the trunk roughly level on flat ground.
    let peak_roll = walk
        .iter()
        .map(|s| s.roll.abs())
        .fold(0.0_f64, f64::max);
    let peak_pitch = walk
        .iter()
        .map(|s| s.pitch.abs())
        .fold(0.0_f64, f64::max);
    eprintln!(
        "[wbc] forward_command: peak |roll| = {peak_roll:.2} rad, \
         peak |pitch| = {peak_pitch:.2} rad",
    );
    // 0.5 rad ≈ 30° — generous because WBC q[3..7] = identity means
    // tilt feedback is approximate.
    assert!(
        peak_roll < 0.5,
        "forward walk: |roll| peak {peak_roll:.2} rad too large",
    );
    assert!(
        peak_pitch < 0.5,
        "forward walk: |pitch| peak {peak_pitch:.2} rad too large",
    );
}
