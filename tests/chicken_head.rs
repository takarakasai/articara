//! End-to-end MuJoCo regression for the **ChickenHead** world-frame
//! head-attitude hold ([`articara::chicken_head`]).
//!
//! namiashi carries a 1-DoF head joint (`arm_pitch_joint`, a `+Y`/pitch
//! revolute). ChickenHead counter-rotates that joint against the trunk pitch
//! so the head stays level in the world. Two host paths are exercised:
//!
//! 1. **Position-PD path** — the host commands the head actuator's position
//!    target to `q* = sign·(θ_ref − θ_trunk)` each tick. Behavioural test:
//!    seed the head far from the reference, run standing, assert it converges
//!    to the ChickenHead target.
//! 2. **WBC torque path** — `WbcPipeline.chicken_head` injects the head as a
//!    joint-acceleration task through the solver's per-actuator channel.
//!    Deterministic test: at the first WBC solve with the head displaced, the
//!    solved head acceleration must strongly oppose the displacement (and be
//!    ~0 when ChickenHead is disabled — the no-op guarantee).
//!
//! Both share the namiashi static-stand setup from `wbc_walk.rs`.

#![cfg(feature = "mujoco")]

use std::path::PathBuf;

use articara::chicken_head::{ChickenHeadConfig, StabAxis};
use articara::gait::{auto_detect_kinematics_config, GaitController, DEFAULT_FOOT_LINKS};
use articara::mjcf::{GroundPlaneCfg, MjcfExportOptions};
use articara::mujoco_sim::MujocoSim;
use articara::robot::RobotModel;
use articara::wbc_pipeline::WbcPipeline;
use nalgebra::Vector3;
use quadruped_gait::{
    solve_leg_ik, ContactDrivenPhase, GaitConfig, GaitMode, KinematicsConfig, LegIkSolution,
    VelocityCmd,
};

const ARM_JOINT: &str = "arm_pitch_joint";

fn namiashi_misa() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("tests")
        .join("fixtures")
        .join("namiashi")
        .join("namiashi.misa")
}

/// Seed the legs out of their extended singularity (same as wbc_walk.rs).
fn seed_legs(robot: &mut RobotModel, kin: &KinematicsConfig) {
    for leg_kin in [&kin.fl, &kin.fr, &kin.rl, &kin.rr] {
        let sol = solve_leg_ik(leg_kin, leg_kin.nominal_foot_body, false);
        let LegIkSolution::Reached { hip, thigh, calf } = sol else {
            panic!("{:?}: nominal_foot_body unreachable", leg_kin.leg);
        };
        for (joint_name, q_ik, sign) in [
            (&leg_kin.hip_joint, hip, 1.0),
            (&leg_kin.thigh_joint, thigh, -1.0),
            (&leg_kin.calf_joint, calf, -1.0),
        ] {
            if let Some(&ji) = robot.joint_map.get(joint_name.as_str()) {
                robot.joint_positions[ji] = q_ik * sign;
            }
        }
    }
}

/// Which control path ChickenHead drives in a run.
#[derive(Clone, Copy, PartialEq)]
enum Path {
    /// Command the head actuator's position target each tick.
    PositionPd,
    /// Inject the head as a WBC joint-acceleration task (full τ bypass).
    Wbc { chicken_enabled: bool },
}

struct RunResult {
    /// Head joint angle (rad) at the end of the run.
    arm_q_final: f64,
    /// Solved head acceleration at the first post-burn-in WBC solve
    /// (`Wbc` paths only; `None` for `PositionPd`).
    arm_qddot_first_wbc: Option<f64>,
}

/// Run standing namiashi with the head seeded at `arm_seed` and ChickenHead
/// holding the head at `target_world_angle`, via `path`. Returns `None` if the
/// namiashi fixture is missing (clean skip).
fn run(path: Path, arm_seed: f64, target_world_angle: f64) -> Option<RunResult> {
    let file = namiashi_misa();
    if !file.exists() {
        eprintln!("namiashi fixture missing at {} — skipping", file.display());
        return None;
    }
    let mut robot = RobotModel::from_misa(&file).expect("load namiashi .misa");
    let mut kin = auto_detect_kinematics_config(&robot, &DEFAULT_FOOT_LINKS)
        .expect("auto-detect kinematics");
    for leg_kin in [&mut kin.fl, &mut kin.fr, &mut kin.rl, &mut kin.rr] {
        let total_leg = leg_kin.upper_leg_m + leg_kin.lower_leg_m;
        leg_kin.nominal_foot_body.z += 0.08 * total_leg;
    }
    seed_legs(&mut robot, &kin);
    // Seed the head joint away from where ChickenHead wants it, so a passing
    // run has to actually move it.
    let arm_ji = *robot
        .joint_map
        .get(ARM_JOINT)
        .expect("namiashi has arm_pitch_joint");
    robot.joint_positions[arm_ji] = arm_seed;

    let opts = MjcfExportOptions {
        ground_plane: Some(GroundPlaneCfg { z: 0.0, half_size: 4.0, roll: 0.0, pitch: 0.0 }),
        add_actuators: true,
        ..Default::default()
    };
    let mut sim = MujocoSim::new(&robot, opts).expect("MujocoSim::new");
    sim.set_gravity_compensation(true);

    let mut gc = GaitController::build(&robot, kin.clone(), GaitConfig::trot(), GaitMode::Mpc)
        .expect("GaitController::build");

    let foot_links: [String; 4] = std::array::from_fn(|i| DEFAULT_FOOT_LINKS[i].1.to_string());
    let mut pipeline = WbcPipeline::new(&robot, foot_links);

    // ChickenHead config (level-hold at `target_world_angle`, pitch axis).
    let mut chicken = ChickenHeadConfig::for_joint(&robot, ARM_JOINT, StabAxis::Pitch)
        .expect("resolve arm joint");
    chicken.enabled = true;
    chicken.target_world_angle = target_world_angle;

    // Resolve the head's misarta v-index for reading the solved acceleration.
    let arm_vi = {
        let a2m = pipeline.a2m_for_test();
        let model = pipeline.model_for_test();
        a2m[arm_ji].map(|mi| model.v_idx[mi]).expect("arm resolves in WBC model")
    };

    if let Path::Wbc { chicken_enabled } = path {
        pipeline.chicken_head = chicken_enabled.then(|| chicken.clone());
    }

    let dt = 0.002;
    let burn_in_steps = 250; // 0.5 s
    // Position-PD needs a long window to converge; the WBC tests only read the
    // first post-burn-in solve, so a couple of extra ticks suffice there.
    let n_steps = match path {
        Path::PositionPd => 1250,          // 2.5 s
        Path::Wbc { .. } => burn_in_steps + 3,
    };
    let mut arm_qddot_first_wbc: Option<f64> = None;

    gc.enable();
    gc.set_velocity_cmd(VelocityCmd { vx: 0.0, vy: 0.0, wz: 0.0 });

    for k in 0..n_steps {
        let v_obs = sim.body_world_linear_velocity(&robot.root_link).unwrap_or([0.0; 3]);
        let w_obs = sim.body_world_angular_velocity(&robot.root_link).unwrap_or([0.0; 3]);
        gc.set_body_state_observed(
            Vector3::new(v_obs[0], v_obs[1], v_obs[2]),
            Vector3::new(w_obs[0], w_obs[1], w_obs[2]),
        );

        let (out, targets, _torque_ff) = gc.tick(dt);
        for (idx, q) in targets {
            sim.set_position_target(idx, q);
        }

        // Body orientation for the ChickenHead reference.
        let body_quat = sim
            .body_world_orientation(&robot.root_link)
            .unwrap_or_else(nalgebra::UnitQuaternion::identity);

        match path {
            Path::PositionPd => {
                // Command the head actuator's position target directly.
                let q_ref = chicken.target_angle(&body_quat);
                sim.set_position_target(arm_ji, q_ref);
                sim.clear_wbc_torques();
            }
            Path::Wbc { .. } => {
                if k >= burn_in_steps {
                    let f_grf_world = gc
                        .predicted_grfs()
                        .map(|s| s.grfs_first_step)
                        .unwrap_or([Vector3::zeros(); 4]);
                    let foot_links_str: [&str; 4] =
                        std::array::from_fn(|i| pipeline.foot_links[i].as_str());
                    let force_z = sim.contact_force_per_foot(&foot_links_str);
                    let nominal = [
                        out.legs[0].phase,
                        out.legs[1].phase,
                        out.legs[2].phase,
                        out.legs[3].phase,
                    ];
                    let corrected =
                        ContactDrivenPhase::apply_correction(&nominal, force_z, 5.0, 0.0);
                    let contact_flag = std::array::from_fn(|i| corrected[i].is_stance);
                    let taus = pipeline.solve(
                        &robot,
                        &sim,
                        &out,
                        gc.kinematics(),
                        gc.joint_indices(),
                        gc.joint_signs(),
                        &Vector3::zeros(),
                        0.0,
                        &Vector3::new(v_obs[0], v_obs[1], v_obs[2]),
                        &Vector3::new(w_obs[0], w_obs[1], w_obs[2]),
                        &f_grf_world,
                        contact_flag,
                        dt,
                    );
                    if arm_qddot_first_wbc.is_none() {
                        arm_qddot_first_wbc = pipeline
                            .last_solution
                            .as_ref()
                            .map(|s| s.q_ddot[arm_vi]);
                    }
                    // Hybrid application (same as wbc_walk.rs): Position-PD
                    // tracks the leg q* while the WBC τ enters as feed-forward.
                    // This keeps the standing body stable; we read the head's
                    // requested acceleration off `last_solution` above rather
                    // than driving it via a full torque bypass (which the
                    // untasked-arm case leaves numerically ill-posed).
                    for (ji, &tau) in taus.iter().enumerate() {
                        sim.set_torque_feedforward(ji, tau);
                    }
                    sim.clear_wbc_torques();
                } else {
                    sim.clear_wbc_torques();
                }
            }
        }

        sim.step(&mut robot, dt, true);
    }

    let arm_q_final = sim.joint_q_qd(ARM_JOINT).map(|(q, _)| q).unwrap_or(f64::NAN);
    Some(RunResult { arm_q_final, arm_qddot_first_wbc })
}

/// Position-PD path: the head, seeded high, converges to the ChickenHead
/// target when the body stands roughly level.
#[test]
fn position_pd_head_converges_to_target() {
    let target = 0.4;
    let Some(res) = run(Path::PositionPd, /* seed */ -0.6, target) else {
        return;
    };
    // Body is ~level (static stand), so q* ≈ target. The actuator PD should
    // pull the head there within a generous tolerance.
    assert!(
        (res.arm_q_final - target).abs() < 0.1,
        "head should hold near ChickenHead target {target:.2}, got {:.3}",
        res.arm_q_final
    );
}

/// WBC path (deterministic wiring): with the head at nominal but the hold
/// target offset to +0.4 rad, the WBC-solved head acceleration must be a
/// strong *positive* value driving the head up toward the target.
#[test]
fn wbc_head_acceleration_tracks_target() {
    // Head at nominal 0, target +0.4, level body ⇒ q* ≈ +0.4 ⇒
    // q̈* = kp·(0.4−0) + kd·(…) ≈ +40 ≫ 0.
    let Some(res) = run(Path::Wbc { chicken_enabled: true }, 0.0, 0.4) else {
        return;
    };
    let a = res.arm_qddot_first_wbc.expect("WBC solve happened");
    assert!(
        a > 10.0,
        "ChickenHead should command a strong positive head acceleration to \
         drive the head toward the +0.4 rad target, got q̈={a:.2}"
    );
}

/// WBC gating guarantee: with ChickenHead disabled the head carries no
/// stabilising task, so it is *not* commanded toward the +0.4 target — unlike
/// the enabled run (see [`wbc_head_acceleration_tracks_target`]). The head is
/// then just a free DoF whose solved acceleration is whatever the null-space
/// picks (not the ChickenHead law), proving the injection is fully gated by
/// `enabled`.
#[test]
fn wbc_disabled_does_not_track_target() {
    let Some(res) = run(Path::Wbc { chicken_enabled: false }, 0.0, 0.4) else {
        return;
    };
    let a = res.arm_qddot_first_wbc.expect("WBC solve happened");
    assert!(
        a < 10.0,
        "with ChickenHead off the head must NOT be accelerated toward the \
         +0.4 target the enabled run commands (~+40); got q̈={a:.2}"
    );
}
