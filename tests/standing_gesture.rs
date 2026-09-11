//! Integration regression for the **standing gesture** head channel
//! ([`articara::standing_gesture`]).
//!
//! Drives namiashi standing in place and plays a `Nod` head gesture the same
//! way the app driver does — composing the gesture offset onto a base head
//! angle and commanding it via `set_position_target` each tick. Asserts the
//! head joint actually oscillates (peak-to-peak ≈ 2·amplitude), i.e. the sim
//! tracks the commanded bob.
//!
//! The oscillator math itself is unit-tested in the library; this pins the
//! end-to-end Position-PD path (the head channel needs no WBC).

#![cfg(feature = "mujoco")]

use std::path::PathBuf;

use articara::gait::{auto_detect_kinematics_config, GaitController, DEFAULT_FOOT_LINKS};
use articara::mjcf::{GroundPlaneCfg, MjcfExportOptions};
use articara::mujoco_sim::MujocoSim;
use articara::robot::RobotModel;
use articara::standing_gesture::{GestureKind, StandingGestureConfig};
use nalgebra::Vector3;
use quadruped_gait::{
    solve_leg_ik, GaitConfig, GaitMode, KinematicsConfig, LegIkSolution, VelocityCmd,
};

const ARM_JOINT: &str = "arm_pitch_joint";

fn namiashi_misa() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("tests")
        .join("fixtures")
        .join("namiashi")
        .join("namiashi.misa")
}

fn seed_legs(robot: &mut RobotModel, kin: &KinematicsConfig) {
    for leg_kin in [&kin.fl, &kin.fr, &kin.rl, &kin.rr] {
        let LegIkSolution::Reached { hip, thigh, calf } =
            solve_leg_ik(leg_kin, leg_kin.nominal_foot_body, false)
        else {
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

/// Standing namiashi + a Nod head gesture. Returns the peak-to-peak swing of
/// the head joint over the run, or `None` if the fixture is missing.
#[test]
fn nod_gesture_oscillates_the_head() {
    let file = namiashi_misa();
    if !file.exists() {
        eprintln!("namiashi fixture missing at {} — skipping", file.display());
        return;
    }
    let mut robot = RobotModel::from_misa(&file).expect("load namiashi .misa");
    let mut kin = auto_detect_kinematics_config(&robot, &DEFAULT_FOOT_LINKS)
        .expect("auto-detect kinematics");
    for leg_kin in [&mut kin.fl, &mut kin.fr, &mut kin.rl, &mut kin.rr] {
        let total_leg = leg_kin.upper_leg_m + leg_kin.lower_leg_m;
        leg_kin.nominal_foot_body.z += 0.08 * total_leg;
    }
    seed_legs(&mut robot, &kin);

    let opts = MjcfExportOptions {
        ground_plane: Some(GroundPlaneCfg { z: 0.0, half_size: 4.0, roll: 0.0, pitch: 0.0 }),
        add_actuators: true,
        ..Default::default()
    };
    let mut sim = MujocoSim::new(&robot, opts).expect("MujocoSim::new");
    sim.set_gravity_compensation(true);

    let mut gc = GaitController::build(&robot, kin.clone(), GaitConfig::trot(), GaitMode::Mpc)
        .expect("GaitController::build");
    gc.enable();
    gc.set_velocity_cmd(VelocityCmd { vx: 0.0, vy: 0.0, wz: 0.0 });

    // Nod gesture: ~9° at 0.8 Hz.
    let mut gesture = StandingGestureConfig::for_robot(&robot, GestureKind::Nod);
    gesture.enabled = true;
    gesture.frequency_hz = 0.8;
    assert!(
        gesture.head_joint_idx.is_some(),
        "namiashi should expose the arm_pitch_joint head"
    );

    let dt = 0.002;
    let burn_in = 250; // settle first
    let n = 1500; // 3 s
    let mut t_gesture = 0.0_f64;
    let mut q_min = f64::INFINITY;
    let mut q_max = f64::NEG_INFINITY;

    for k in 0..n {
        let v = sim.body_world_linear_velocity(&robot.root_link).unwrap_or([0.0; 3]);
        let w = sim.body_world_angular_velocity(&robot.root_link).unwrap_or([0.0; 3]);
        gc.set_body_state_observed(
            Vector3::new(v[0], v[1], v[2]),
            Vector3::new(w[0], w[1], w[2]),
        );
        let (_out, targets, _ff) = gc.tick(dt);
        for (idx, q) in targets {
            sim.set_position_target(idx, q);
        }
        // Head channel exactly as the app driver composes it (base 0 — no
        // ChickenHead here — plus the gesture offset).
        if k >= burn_in {
            if let Some(q) = gesture.head_target(t_gesture, 0.0) {
                sim.set_position_target(gesture.head_joint_idx.unwrap(), q);
            }
            t_gesture += dt;
            if let Some((q, _)) = sim.joint_q_qd(ARM_JOINT) {
                q_min = q_min.min(q);
                q_max = q_max.max(q);
            }
        }
        sim.step(&mut robot, dt, true);
    }

    let peak_to_peak = q_max - q_min;
    // Commanded swing is 2·amplitude ≈ 0.30 rad; the actuator PD tracks it
    // with some lag, so require a clear majority of the commanded swing.
    assert!(
        peak_to_peak > 0.15,
        "head should visibly bob (peak-to-peak > 0.15 rad); got {peak_to_peak:.3} \
         (min {q_min:.3}, max {q_max:.3})"
    );
}
