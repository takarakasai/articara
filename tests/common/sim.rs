//! Shared helpers for the MuJoCo gait-control regression tests.
//!
//! Each `tests/<x>.rs` file is its own integration-test crate, so the
//! convention here is to declare `mod common;` from each test file
//! and pull the helpers as needed. cargo special-cases `tests/common/
//! mod.rs` so it isn't compiled as a standalone test crate.
//!
//! What lives here:
//! - **Fixture loaders**: paths and parsers for the namiashi URDF
//!   used across `gait_walk_stability` / `wbc_walk` / `lkf_pipeline`
//!   / `integration_walk`.
//! - **Joint seeding**: solve per-leg IK at `nominal_foot_body` so
//!   the legs don't sit at their q=0 fully-extended kinematic
//!   singularity at sim start.
//! - **Sim builder**: stamp out a [`MujocoSim`] with the standard
//!   ground-plane + actuator-on-every-joint configuration.
//!
//! Keep the surface narrow: per-test specific tuning (cmd_vx, dt,
//! WBC params, etc.) stays in the test file. This module is for
//! plumbing the call sites all share.



// Each integration-test binary compiles this module in full but
// only calls the helpers it needs — per-binary dead_code noise
// is expected, not a defect.
#![allow(dead_code)]

#[allow(unused_imports)] // some tests don't use every helper
use std::path::PathBuf;

use articara::mjcf::{GroundPlaneCfg, MjcfExportOptions};
use articara::mujoco_sim::MujocoSim;
use articara::rbd::model::ActuatorMode;
use articara::robot::RobotModel;
use quadruped_gait::{solve_leg_ik, KinematicsConfig, LegIkSolution};

/// Path to the namiashi URDF fixture (1.5 kg-class quadruped — the
/// reference robot for every regression test in this directory).
///
/// **Note**: URDF only carries kinematics. For dynamics-fidelity tests
/// (PD-tracking, walk envelopes), prefer [`namiashi_misa`] — the URDF
/// lacks the `.misa` actuator config (kp=100/kv=1.2) and joint damping
/// (0.1) that the GUI uses, so URDF-loaded tests under-stiff the legs
/// and the body settles ~50 mm lower than reality.
pub fn namiashi_urdf() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("tests")
        .join("fixtures")
        .join("namiashi")
        .join("urdf")
        .join("namiashi.urdf")
}

/// Path to the namiashi master format (.misa) fixture. Use this in
/// preference to [`namiashi_urdf`] for any test that asserts on body
/// trajectory or PD-tracking behaviour — the `.misa` carries actuator
/// kp/kv and joint damping that the URDF doesn't, so loading via
/// `RobotModel::from_misa` reproduces the GUI exactly (validated by
/// `integration_walk::diag_walk_champ_forward_gui_replica_full_misa`).
pub fn namiashi_misa() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("tests")
        .join("fixtures")
        .join("namiashi")
        .join("namiashi.misa")
}

/// Solve per-leg IK at `nominal_foot_body` and write the joint angles
/// into `robot.joint_positions`. Without this seeding, MuJoCo starts
/// with every joint at 0 (= legs fully extended), which is the
/// kinematic singularity for stance and immediately collapses.
pub fn seed_joint_positions_from_kinematics(
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

/// Standard Position-PD setup for the namiashi joints. Same gains as
/// `gait_walk_stability` / `wbc_walk` use (`kp = 30, kv = 0.6`); these
/// are tuned so the body stays upright under gravity-comp during the
/// burn-in window before any controller is enabled.
pub fn setup_position_pd_actuators(robot: &mut RobotModel) {
    for j in robot.joints.iter_mut() {
        if j.joint_type == "fixed" {
            continue;
        }
        j.actuator_mode = ActuatorMode::Position;
        j.actuator_kp = 30.0;
        j.actuator_kv = 0.6;
    }
}

/// Default MJCF export options for the regression tests: flat ground
/// plane at z=0 with a 4 m half-extent, every movable joint backed
/// by a motor actuator. Matches the configuration in `wbc_walk` /
/// `lkf_pipeline` / `gait_walk_stability`.
pub fn default_mjcf_export_options() -> MjcfExportOptions {
    MjcfExportOptions {
        ground_plane: Some(GroundPlaneCfg {
            z: 0.0,
            half_size: 4.0,
            roll: 0.0,
            pitch: 0.0,
        }),
        add_actuators: true,
        ..Default::default()
    }
}

/// Bundle that the regression-test harnesses keep around: the
/// loaded `RobotModel`, the auto-detected `KinematicsConfig`, and a
/// freshly-built `MujocoSim`. Always paired together so threading
/// these three through every test signature would be repetitive.
pub struct StandFixture {
    pub robot: RobotModel,
    pub kin: KinematicsConfig,
    pub sim: MujocoSim,
}

/// Build the standard "namiashi standing on flat ground" fixture:
/// loads the URDF, seeds joint angles via IK, configures Position-PD,
/// constructs a `MujocoSim`, and turns on gravity compensation.
///
/// Returns `None` when the URDF fixture is missing — caller should
/// `eprintln!` + early-return so the test silently skips on bare
/// CI environments without the namiashi mesh fixture.
///
/// **For dynamics-fidelity tests, prefer [`build_namiashi_stand_fixture_misa`]**
/// — the URDF path uses fabricated `kp=30, kv=0.6` and `damping=0` so
/// the body settles ~50 mm lower than the GUI. Existing call sites
/// retain this behaviour for envelope compatibility.
pub fn build_namiashi_stand_fixture() -> Option<StandFixture> {
    use articara::gait::{auto_detect_kinematics_config, DEFAULT_FOOT_LINKS};

    let path = namiashi_urdf();
    if !path.exists() {
        eprintln!(
            "namiashi fixture missing at {} — skipping regression test",
            path.display()
        );
        return None;
    }
    let mut robot = RobotModel::from_urdf(&path).expect("load namiashi URDF");
    setup_position_pd_actuators(&mut robot);

    let mut kin = auto_detect_kinematics_config(&robot, &DEFAULT_FOOT_LINKS)
        .expect("auto-detect kinematics");
    // Standard 8 % nudge above the URDF's nominal foot pose — same
    // adjustment the existing tests use to avoid bottoming out the
    // legs at full extension.
    for leg_kin in [&mut kin.fl, &mut kin.fr, &mut kin.rl, &mut kin.rr] {
        let total_leg = leg_kin.upper_leg_m + leg_kin.lower_leg_m;
        leg_kin.nominal_foot_body.z += 0.08 * total_leg;
    }
    seed_joint_positions_from_kinematics(&mut robot, &kin);

    let mut sim =
        MujocoSim::new(&robot, default_mjcf_export_options()).expect("MujocoSim::new");
    sim.set_gravity_compensation(true);

    Some(StandFixture { robot, kin, sim })
}

/// Same shape as [`build_namiashi_stand_fixture`] but loads from
/// `namiashi.misa` so the robot carries the GUI's real actuator config
/// (`kp=100, kv=1.2` per leg motor, `damping=0.1` per leg joint).
///
/// Use this in any regression test that asserts on body trajectory
/// or PD tracking. The URDF-based variant settles the body ~50 mm
/// lower than reality (3.4× dx tracking gap) — see commit `bcbcdf7`
/// + `diag_walk_champ_forward_gui_replica_full_misa` for the
/// validation that the misa loader reproduces the GUI exactly.
pub fn build_namiashi_stand_fixture_misa() -> Option<StandFixture> {
    use articara::gait::{auto_detect_kinematics_config, DEFAULT_FOOT_LINKS};

    let path = namiashi_misa();
    if !path.exists() {
        eprintln!(
            "namiashi .misa fixture missing at {} — skipping regression test",
            path.display()
        );
        return None;
    }
    let mut robot = RobotModel::from_misa(&path).expect("load namiashi .misa");
    // No `setup_position_pd_actuators` call — the .misa already carries
    // the actuator config (kp=100, kv=1.2) per joint.

    let mut kin = auto_detect_kinematics_config(&robot, &DEFAULT_FOOT_LINKS)
        .expect("auto-detect kinematics");
    for leg_kin in [&mut kin.fl, &mut kin.fr, &mut kin.rl, &mut kin.rr] {
        let total_leg = leg_kin.upper_leg_m + leg_kin.lower_leg_m;
        leg_kin.nominal_foot_body.z += 0.08 * total_leg;
    }
    seed_joint_positions_from_kinematics(&mut robot, &kin);

    let mut sim =
        MujocoSim::new(&robot, default_mjcf_export_options()).expect("MujocoSim::new");
    sim.set_gravity_compensation(true);

    Some(StandFixture { robot, kin, sim })
}

/// Foot link names in canonical FL / FR / RL / RR slot order, ready
/// to be `String`-cloned into [`articara::wbc_pipeline::WbcPipeline`]
/// or [`articara::estimator::LkfPipeline`].
pub fn default_foot_links() -> [String; 4] {
    use articara::gait::DEFAULT_FOOT_LINKS;
    [
        DEFAULT_FOOT_LINKS[0].1.to_string(),
        DEFAULT_FOOT_LINKS[1].1.to_string(),
        DEFAULT_FOOT_LINKS[2].1.to_string(),
        DEFAULT_FOOT_LINKS[3].1.to_string(),
    ]
}
