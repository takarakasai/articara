//! Headless MuJoCo render of the ChickenHead controller on **namiashi**.
//!
//! The physically-correct quadruped body-pose motion: the **feet stay planted
//! at fixed world contact points** while the trunk pitches, the legs bending
//! (per-leg IK) to reposition the body about the fixed feet — exactly how a
//! quadruped shifts its body pose. On top of that the head joint
//! (`arm_pitch_joint`) is driven by the **real**
//! [`articara::chicken_head::ChickenHeadConfig`] to hold the head level in the
//! world. Real MuJoCo physics (position-PD, feet planted by contact), rendered
//! offscreen (EGL).
//!
//!   * `--head hold`  (default) — ChickenHead ON: head held level in the world.
//!   * `--head fixed`           — ChickenHead OFF: head fixed at neutral.
//!
//! PNGs → `--outdir` (default `/tmp/ch_render/<head>`), for ffmpeg.
//!
//! Run:
//!   MUJOCO_DYNAMIC_LINK_DIR=~/.mujoco/mujoco-3.8.0/lib \
//!   cargo run --example chicken_head_render --features render -- --head hold

use std::path::PathBuf;

use articara::chicken_head::{ChickenHeadConfig, StabAxis};
use articara::gait::{auto_detect_kinematics_config, DEFAULT_FOOT_LINKS};
use articara::mjcf::{GroundPlaneCfg, MjcfExportOptions};
use articara::mujoco_sim::MujocoSim;
use articara::robot::RobotModel;
use nalgebra::Vector3;
use quadruped_gait::{solve_leg_ik, KinematicsConfig, LegIkSolution};

use mujoco::prelude::*;
use mujoco::renderer::MjRenderer;

const ARM_JOINT: &str = "arm_pitch_joint";
// MuJoCo's default offscreen framebuffer is 640×480.
const W: u32 = 640;
const H: u32 = 480;
const FPS: f64 = 30.0;

fn namiashi_misa() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures/namiashi/namiashi.misa")
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

/// Scripted trunk-pitch target (rad) — a lively rock plus a mid-clip lurch.
/// Kept moderate so the feet stay reachable and the CoM stays over support.
fn trunk_pitch_cmd(t: f64) -> f64 {
    let tau = std::f64::consts::TAU;
    let rock = 11.0_f64.to_radians() * (tau * 0.45 * t).sin();
    let harmonic = 3.0_f64.to_radians() * (tau * 1.1 * t + 0.6).sin();
    let lurch = 7.0_f64.to_radians() * (-((t - 5.0) / 0.45).powi(2)).exp();
    rock + harmonic + lurch
}

fn arg(name: &str) -> Option<String> {
    std::env::args().skip_while(|a| a != name).nth(1)
}

fn main() {
    let head_mode = arg("--head").unwrap_or_else(|| "hold".to_string());
    let chicken_on = head_mode == "hold";
    let outdir = arg("--outdir").unwrap_or_else(|| format!("/tmp/ch_render/{head_mode}"));
    let duration: f64 = arg("--dur").and_then(|s| s.parse().ok()).unwrap_or(8.0);
    std::fs::create_dir_all(&outdir).expect("create outdir");

    let file = namiashi_misa();
    if !file.exists() {
        eprintln!("namiashi fixture missing at {} — abort", file.display());
        std::process::exit(1);
    }
    let mut robot = RobotModel::from_misa(&file).expect("load namiashi .misa");
    let mut kin =
        auto_detect_kinematics_config(&robot, &DEFAULT_FOOT_LINKS).expect("kinematics");
    for leg_kin in [&mut kin.fl, &mut kin.fr, &mut kin.rl, &mut kin.rr] {
        let total = leg_kin.upper_leg_m + leg_kin.lower_leg_m;
        // Raise the foot in the body frame so the legs stay out of the fully
        // extended singularity. The `0.08·leg` term keeps a small margin; the
        // extra 16 cm crouches the standing pose so the legs are clearly bent
        // (not over-extended) — a natural, low quadruped stance.
        leg_kin.nominal_foot_body.z += 0.08 * total + 0.16;
    }
    seed_legs(&mut robot, &kin);

    // Per-leg kinematics, joint indices + IK→URDF sign, and the FIXED world
    // foot contact point (level-body home stance; foot z = 0 on the ground).
    let legs_kin = [&kin.fl, &kin.fr, &kin.rl, &kin.rr];
    let jidx = |n: &str| *robot.joint_map.get(n).expect("leg joint in model");
    let leg_joints: Vec<[(usize, f64); 3]> = legs_kin
        .iter()
        .map(|lk| {
            [
                (jidx(&lk.hip_joint), 1.0),
                (jidx(&lk.thigh_joint), -1.0),
                (jidx(&lk.calf_joint), -1.0),
            ]
        })
        .collect();
    let body_h = -kin.fl.nominal_foot_body.z; // standing body height (feet at z=0)
    let foot_world_fixed: Vec<Vector3<f64>> = legs_kin
        .iter()
        .map(|lk| Vector3::new(lk.nominal_foot_body.x, lk.nominal_foot_body.y, 0.0))
        .collect();

    let opts = MjcfExportOptions {
        ground_plane: Some(GroundPlaneCfg { z: 0.0, half_size: 4.0, roll: 0.0, pitch: 0.0 }),
        add_actuators: true,
        ..Default::default()
    };
    let mut sim = MujocoSim::new(&robot, opts).expect("MujocoSim::new");
    sim.set_gravity_compensation(true);

    let mut chicken = ChickenHeadConfig::for_joint(&robot, ARM_JOINT, StabAxis::Pitch)
        .expect("arm_pitch_joint");
    chicken.enabled = chicken_on;

    // ── Offscreen renderer against the live model ───────────────────────
    let model = sim.mj_model();
    let mut renderer = MjRenderer::builder()
        .width(W)
        .height(H)
        .num_visual_user_geom(0)
        .num_visual_internal_geom(0)
        .rgb(true)
        .depth(false)
        .build(model.clone())
        .expect("build offscreen renderer (EGL)");

    let trunk_id = model.body("trunk").expect("trunk body").id;
    let mut cam = MjvCamera::new_tracking(trunk_id);
    cam.azimuth = 90.0; // side view (x-z plane) so pitch is visible
    cam.elevation = -6.0;
    cam.distance = 1.15;
    cam.lookat = [0.03, 0.0, 0.24];
    renderer.set_camera(cam);

    let dt = 0.002;
    let steps_per_frame = (1.0 / (FPS * dt)).round() as usize; // ≈ 17
    let burn_in = 400; // 0.8 s to settle at the nominal stance first
    let n_frames = (duration * FPS) as usize;

    let mut frame = 0usize;
    let mut k = 0usize;
    let mut pitch_sq = 0.0;
    let mut pitch_n = 0usize;
    while frame < n_frames {
        // Body-pose command: pitch the trunk about the ground support centre
        // while keeping every foot at its FIXED world contact point. Per-leg
        // IK gives the joint angles that place the body at that pitched pose.
        let theta = if k >= burn_in {
            trunk_pitch_cmd((k - burn_in) as f64 * dt)
        } else {
            0.0
        };
        let r = nalgebra::Rotation3::from_euler_angles(0.0, theta, 0.0);
        let p_body = r * Vector3::new(0.0, 0.0, body_h);
        for (leg, lk) in legs_kin.iter().enumerate() {
            // Foot target in the body frame for the desired pitched body pose.
            let foot_body = r.inverse() * (foot_world_fixed[leg] - p_body);
            if let LegIkSolution::Reached { hip, thigh, calf } =
                solve_leg_ik(lk, foot_body, false)
            {
                let qs = [hip, thigh, calf];
                for kk in 0..3 {
                    let (idx, sign) = leg_joints[leg][kk];
                    sim.set_position_target(idx, sign * qs[kk]);
                }
            }
        }
        // Head: real ChickenHead command off the *measured* trunk attitude.
        let body_quat = sim
            .body_world_orientation(&robot.root_link)
            .unwrap_or_else(nalgebra::UnitQuaternion::identity);
        let head_q = if chicken.enabled { chicken.target_angle(&body_quat) } else { 0.0 };
        sim.set_position_target(chicken.joint_idx, head_q);

        sim.step(&mut robot, dt, true);

        if k >= burn_in && (k - burn_in).is_multiple_of(steps_per_frame) {
            renderer.sync_data(sim.mj_data_mut()).expect("sync_data");
            renderer.render().expect("render");
            renderer
                .save_rgb(format!("{outdir}/frame_{frame:04}.png"))
                .expect("save_rgb");
            let p = body_quat.euler_angles().1;
            pitch_sq += p * p;
            pitch_n += 1;
            frame += 1;
        }
        k += 1;
    }

    let pitch_rms = (pitch_sq / pitch_n.max(1) as f64).sqrt().to_degrees();
    eprintln!(
        "[{head_mode}] wrote {frame} frames to {outdir}  ·  measured trunk-pitch RMS = {pitch_rms:.2}°"
    );
}
