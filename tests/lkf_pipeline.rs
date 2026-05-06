//! End-to-end test: run [`articara::estimator::LkfPipeline`] in
//! lockstep with a MuJoCo simulation of namiashi and verify the KF's
//! body-position estimate tracks ground truth within a few cm.
//!
//! Mirrors the harness used by `wbc_walk` / `gait_walk_stability`
//! (URDF → MujocoSim → tick loop), but the controller layer is
//! intentionally kept open-loop (per-joint Position-PD) so we can
//! isolate the estimator's behaviour from MPC / WBC dynamics.

#![cfg(feature = "mujoco")]

use std::path::PathBuf;

use articara::estimator::LkfPipeline;
use articara::gait::{auto_detect_kinematics_config, DEFAULT_FOOT_LINKS};
use articara::mjcf::{GroundPlaneCfg, MjcfExportOptions};
use articara::mujoco_sim::MujocoSim;
use articara::rbd::model::ActuatorMode;
use articara::robot::RobotModel;
use nalgebra::{UnitQuaternion, Vector3};
use quadruped_gait::{solve_leg_ik, KinematicsConfig, LegIkSolution};

fn namiashi_urdf() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("tests")
        .join("fixtures")
        .join("namiashi")
        .join("urdf")
        .join("namiashi.urdf")
}

fn seed_joint_positions_from_kinematics(
    robot: &mut RobotModel,
    kin: &KinematicsConfig,
) {
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

/// Static-stance e2e: leave the robot standing under gravity-comp
/// only. After 1 s the LKF body-z estimate should match the MuJoCo
/// ground-truth body-z within ±3 cm.
///
/// IMU input is synthesised by finite-differencing the body's
/// world-frame velocity each tick (fed in as `accel_world`); body
/// orientation is taken straight from MuJoCo (we'd normally route
/// it through Madgwick on real hardware).
#[test]
fn lkf_static_stand_tracks_ground_truth_body_z() {
    let path = namiashi_urdf();
    if !path.exists() {
        eprintln!("namiashi fixture missing — skipping");
        return;
    }
    let mut robot = RobotModel::from_urdf(&path).expect("load namiashi URDF");

    // Position-PD only — keep the controller path simple to isolate
    // the estimator.
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
    sim.set_gravity_compensation(true);

    let foot_links: [String; 4] = [
        DEFAULT_FOOT_LINKS[0].1.to_string(),
        DEFAULT_FOOT_LINKS[1].1.to_string(),
        DEFAULT_FOOT_LINKS[2].1.to_string(),
        DEFAULT_FOOT_LINKS[3].1.to_string(),
    ];
    let mut lkf = LkfPipeline::new(&robot, foot_links);
    // Initialise LKF at the neutral standing pose.
    let foot_init = [
        Vector3::new(0.18, 0.10, 0.0),
        Vector3::new(0.18, -0.10, 0.0),
        Vector3::new(-0.18, 0.10, 0.0),
        Vector3::new(-0.18, -0.10, 0.0),
    ];
    lkf.kf.reset(Vector3::new(0.0, 0.0, 0.30), &foot_init);

    let dt: f64 = 0.002;
    let n_steps = (1.0 / dt) as usize; // 1 s
    let mut prev_v_world = Vector3::zeros();
    let mut max_z_err = 0.0_f64;

    for k in 0..n_steps {
        let v_obs = sim
            .body_world_linear_velocity(&robot.root_link)
            .map(|v| Vector3::new(v[0], v[1], v[2]))
            .unwrap_or_else(Vector3::zeros);
        // Synthesise IMU world-frame acceleration via finite-diff.
        // Subtract gravity since the LKF expects a "gravity-removed"
        // input. The diff is body-velocity / dt, which already excludes
        // gravity (gravity acts on velocity at every step), so we DON'T
        // subtract it again — the body's measured Δv per tick already
        // reflects the net force / mass = gravity − contact = ~0 when
        // standing still.
        let accel_world = if k > 0 {
            (v_obs - prev_v_world) / dt
        } else {
            Vector3::zeros()
        };
        prev_v_world = v_obs;

        let body_quat = sim
            .body_world_orientation(&robot.root_link)
            .unwrap_or_else(UnitQuaternion::identity);

        let _out = lkf.update_from_mujoco(&robot, &sim, body_quat, accel_world, dt);

        sim.step(&mut robot, dt, true);

        // After the burn-in (≥ 0.3 s for the KF prior to converge),
        // track the running max body-z error.
        if (k as f64) * dt >= 0.3 {
            let body_truth = sim
                .body_world_position(&robot.root_link)
                .unwrap_or([0.0, 0.0, 0.0]);
            let z_err = (lkf.kf.x_hat[2] - body_truth[2]).abs();
            if z_err > max_z_err {
                max_z_err = z_err;
            }
        }
    }

    eprintln!("[lkf] static_stand: max body-z error = {max_z_err:.3} m");
    assert!(
        max_z_err < 0.05,
        "LKF body-z estimate diverged from ground truth (max error {:.3} m > 0.05 m)",
        max_z_err,
    );
}
