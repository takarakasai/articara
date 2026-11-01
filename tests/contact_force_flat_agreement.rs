//! On flat ground the two per-foot contact-force measures agree.
//!
//! **That agreement is the trap.** [`MujocoSim::contact_force_per_foot`]
//! returns only the world-`z` component; the normal points up on a flat
//! floor, so a contact test built on it looks correct. On a staircase the
//! normal tilts at the riser and the nosing, oppositely-signed components
//! cancel, and a loaded foot reports a near-zero vertical force and is
//! missed. Anything deciding **whether a foot is on the ground** must use
//! [`MujocoSim::contact_force_magnitude_per_foot`].
//!
//! The tilted cases are covered by the unit tests on `sum_per_foot`
//! (`src/mujoco_sim.rs`): standing the robot on a real slope steep enough to
//! show the shortfall also tips it over before it settles, so the physical
//! test here only pins the flat-ground agreement.
//!
//! That is not hypothetical: on Go2 (2026-09-27, 20 cm × 10 steps) the
//! `z`-component test lost stance feet, the leg-odometry estimate that
//! averages over them drifted by −0.72 m/s, and the blind stair policy
//! collapsed after ~5 steps. The magnitude test brought the bias to
//! +0.01 m/s and the same policy climbed all ten.

#![cfg(feature = "mujoco")]

use std::path::PathBuf;

use articara::gait::{auto_detect_kinematics_config, DEFAULT_FOOT_LINKS};
use articara::mjcf::{GroundPlaneCfg, MjcfExportOptions};
use articara::mujoco_sim::MujocoSim;
use articara::robot::RobotModel;
use quadruped_gait::{solve_leg_ik, LegIkSolution};

fn namiashi_misa() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("tests")
        .join("fixtures")
        .join("namiashi")
        .join("namiashi.misa")
}

/// Stand the robot on a plane tilted by `roll_rad` and return, per foot,
/// `(vertical component, magnitude)` of the contact force once it settles.
fn settle_and_measure(roll_rad: f64) -> ([f64; 4], [f64; 4]) {
    let mut robot = RobotModel::from_misa(&namiashi_misa()).expect("load namiashi .misa");
    let kin = auto_detect_kinematics_config(&robot, &DEFAULT_FOOT_LINKS)
        .expect("auto-detect kinematics");
    for leg_kin in [&kin.fl, &kin.fr, &kin.rl, &kin.rr] {
        let LegIkSolution::Reached { hip, thigh, calf } =
            solve_leg_ik(leg_kin, leg_kin.nominal_foot_body, false)
        else {
            panic!("{:?}: nominal_foot_body unreachable", leg_kin.leg);
        };
        for (joint_name, q, sign) in [
            (&leg_kin.hip_joint, hip, 1.0),
            (&leg_kin.thigh_joint, thigh, -1.0),
            (&leg_kin.calf_joint, calf, -1.0),
        ] {
            if let Some(&ji) = robot.joint_map.get(joint_name.as_str()) {
                robot.joint_positions[ji] = sign * q;
            }
        }
    }
    robot.rebuild_misarta_model();

    let opts = MjcfExportOptions {
        ground_plane: Some(GroundPlaneCfg {
            z: 0.0,
            half_size: 4.0,
            roll: roll_rad,
            pitch: 0.0,
        }),
        add_actuators: true,
        ..Default::default()
    };
    let mut sim = MujocoSim::new(&robot, opts).expect("MujocoSim::new");
    sim.set_gravity_compensation(true);
    for _ in 0..1000 {
        sim.step(&mut robot, 0.002, true);
    }
    let feet: [&str; 4] = [
        DEFAULT_FOOT_LINKS[0].1,
        DEFAULT_FOOT_LINKS[1].1,
        DEFAULT_FOOT_LINKS[2].1,
        DEFAULT_FOOT_LINKS[3].1,
    ];
    (
        sim.contact_force_per_foot(&feet),
        sim.contact_force_magnitude_per_foot(&feet),
    )
}

/// On flat ground the two agree — which is exactly why a `z`-only contact
/// test looks correct until the terrain tilts.
#[test]
fn on_flat_ground_the_vertical_component_equals_the_magnitude() {
    if !namiashi_misa().exists() {
        eprintln!("namiashi fixture missing — skipping");
        return;
    }
    let (fz, mag) = settle_and_measure(0.0);
    let loaded = fz.iter().filter(|f| **f > 1.0).count();
    assert!(loaded >= 2, "expected the robot to be standing, got {fz:?}");
    for l in 0..4 {
        if mag[l] > 1.0 {
            let ratio = fz[l] / mag[l];
            assert!(
                ratio > 0.97,
                "foot {l}: flat ground should give fz ≈ |f|, got {:.3} vs {:.3}",
                fz[l],
                mag[l]
            );
        }
    }
}
