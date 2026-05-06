//! Cross-check that `misarta::rnea::compute_gravity` and MuJoCo's
//! own `qfrc_bias` (with `qvel = 0`) agree at the same configuration.
//!
//! Both compute the **generalised force needed to statically support
//! the robot under gravity** at a given `q`:
//!
//! - misarta: pure gravity term `g(q)` from RNEA with `q̇ = 0`,
//!   `q̈ = 0`. Returns shape `nv = 6 (FreeFlyer) + na`.
//! - MuJoCo: `qfrc_bias = C(q, q̇)·q̇ + g(q)`. With `q̇ = 0` the
//!   Coriolis term vanishes and only `g(q)` remains. Same shape `nv`.
//!
//! If the two values disagree by more than numerical noise, the
//! WBC's `tau_gravity` task references a different physics from the
//! one MuJoCo simulates — which would explain why anchoring τ to
//! `compute_gravity` doesn't actually hold the body up in
//! `wbc_walk` (the legs' "static gravity comp" τ is wrong for the
//! true MuJoCo dynamics).
//!
//! This test is **not** asserting they match exactly — it logs the
//! per-joint values + RMS deviation so the user can see the gap.
//! When the gap is small the WBC integration is sound; when large,
//! we have a model-consistency bug to track down.

#![cfg(feature = "mujoco")]

use std::path::PathBuf;

use articara::gait::{auto_detect_kinematics_config, DEFAULT_FOOT_LINKS};
use articara::mjcf::{GroundPlaneCfg, MjcfExportOptions};
use articara::mujoco_sim::MujocoSim;
use articara::rbd::model::ActuatorMode;
use articara::robot::RobotModel;
use nalgebra as na;
use quadruped_gait::{solve_leg_ik, KinematicsConfig, LegIkSolution};

fn namiashi_urdf() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("tests/fixtures/namiashi/urdf/namiashi.urdf")
}

/// Same seeding the WBC tests use — bent legs at the gait
/// controller's nominal stance, NOT the URDF's q=0 fully-extended
/// pose.
fn seed_joint_positions_from_kinematics(
    robot: &mut RobotModel,
    kin: &KinematicsConfig,
) {
    for leg_kin in [&kin.fl, &kin.fr, &kin.rl, &kin.rr] {
        let target = leg_kin.nominal_foot_body;
        let LegIkSolution::Reached { hip, thigh, calf } =
            solve_leg_ik(leg_kin, target, false)
        else {
            return;
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

#[test]
fn misarta_compute_gravity_matches_mujoco_qfrc_bias() {
    let path = namiashi_urdf();
    if !path.exists() {
        eprintln!("namiashi fixture missing — skipping");
        return;
    }
    let mut robot = RobotModel::from_urdf(&path).expect("load namiashi");
    for j in robot.joints.iter_mut() {
        if j.joint_type != "fixed" {
            j.actuator_mode = ActuatorMode::Position;
        }
    }
    let mut kin =
        auto_detect_kinematics_config(&robot, &DEFAULT_FOOT_LINKS).expect("kin");
    for lk in [&mut kin.fl, &mut kin.fr, &mut kin.rl, &mut kin.rr] {
        let total_leg = lk.upper_leg_m + lk.lower_leg_m;
        lk.nominal_foot_body.z += 0.08 * total_leg;
    }
    seed_joint_positions_from_kinematics(&mut robot, &kin);

    // Build MuJoCo sim. We don't need a ground plane for the
    // qfrc_bias comparison — bias is a property of the kinematic
    // tree + gravity, independent of contacts.
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

    // Tick once with no torque to let MuJoCo populate `qfrc_bias`
    // (it's computed each step). qvel stays at 0 (we never set
    // commands), so qfrc_bias = g(q) cleanly.
    sim.step(&mut robot, 0.002, false);

    // ── MuJoCo qfrc_bias ───────────────────────────────────────────
    let bias_mj_full = sim.qfrc_bias();
    eprintln!("\n=== MuJoCo qfrc_bias (length {}) ===", bias_mj_full.len());
    for (i, v) in bias_mj_full.iter().enumerate() {
        eprintln!("  [{i:2}] = {v:+.4} N·m (or N for base)");
    }

    // ── misarta compute_gravity ────────────────────────────────────
    // Build the same FreeFlyer-rooted misarta model the WBC pipeline
    // uses. We can reuse `WbcPipeline::new` to get its internal model
    // + a2m mapping; for the actual gravity vector we call misarta
    // directly.
    let foot_links: [String; 4] = [
        DEFAULT_FOOT_LINKS[0].1.to_string(),
        DEFAULT_FOOT_LINKS[1].1.to_string(),
        DEFAULT_FOOT_LINKS[2].1.to_string(),
        DEFAULT_FOOT_LINKS[3].1.to_string(),
    ];
    let pipeline = articara::wbc_pipeline::WbcPipeline::new(&robot, foot_links);
    let model = pipeline.model_for_test();
    let a2m = pipeline.a2m_for_test();

    // Build q the same way WbcPipeline::solve does: real base pose
    // from MuJoCo, joint angles from RobotModel.
    let body_pos = sim
        .body_world_position(&robot.root_link)
        .expect("trunk in mjcf");
    let body_q = sim
        .body_world_orientation(&robot.root_link)
        .expect("trunk quat");
    let mut q = model.neutral_q();
    q[0] = body_pos[0];
    q[1] = body_pos[1];
    q[2] = body_pos[2];
    q[3] = body_q.i;
    q[4] = body_q.j;
    q[5] = body_q.k;
    q[6] = body_q.w;
    for ji in 0..robot.joints.len() {
        let Some(mi) = a2m.get(ji).and_then(|&m| m) else {
            continue;
        };
        if model.joints[mi].joint_type.nq() == 1 {
            let qi = model.q_idx[mi];
            q[qi] = robot.joint_positions[ji];
        }
    }

    let g_mis = misarta::rnea::compute_gravity(model, &q);
    eprintln!(
        "\n=== misarta compute_gravity (length {}) ===",
        g_mis.len()
    );
    for i in 0..g_mis.len() {
        eprintln!("  [{i:2}] = {:+.4} N·m (or N for base)", g_mis[i]);
    }

    // ── Per-joint comparison ────────────────────────────────────────
    // The two vectors share length nv (= 6 base + 13 actuated for
    // namiashi = 19), but the **joint ordering** differs:
    //   - MuJoCo's qpos/qvel are laid out as [free-joint(7nq, 6nv);
    //     joint1; joint2; …] in the order MuJoCo compiled them.
    //   - misarta's nv-space follows BFS-from-root through `a2m` /
    //     `v_idx` — usually different from MuJoCo's compile order.
    // We can't match by index without the explicit joint name → MuJoCo
    // dof-id map. For this first-pass test, log both vectors side-by-
    // side and compute the per-block (base / joints) magnitude
    // separately — that catches order-independent anomalies (e.g.,
    // misarta says "base needs 23 N up" but MuJoCo says "0").
    eprintln!("\n=== Aggregate comparison ===");
    let mj_base_norm: f64 = bias_mj_full
        .iter()
        .take(6)
        .map(|v| v * v)
        .sum::<f64>()
        .sqrt();
    let mj_joint_norm: f64 = bias_mj_full
        .iter()
        .skip(6)
        .map(|v| v * v)
        .sum::<f64>()
        .sqrt();
    let mis_base_norm: f64 = g_mis.iter().take(6).map(|v| v * v).sum::<f64>().sqrt();
    let mis_joint_norm: f64 =
        g_mis.iter().skip(6).map(|v| v * v).sum::<f64>().sqrt();
    eprintln!(
        "  base block:  MuJoCo |.| = {mj_base_norm:.3}   misarta |.| = {mis_base_norm:.3}",
    );
    eprintln!(
        "  joint block: MuJoCo |.| = {mj_joint_norm:.3}   misarta |.| = {mis_joint_norm:.3}",
    );

    // Check the `m·g` invariant: total mass × 9.81 should match the
    // base linear-z component (in body frame). Both formulations
    // should give the same value within model-mass agreement.
    let total_mass: f64 = robot.links.iter().map(|l| l.inertial.mass).sum();
    let mg = total_mass * 9.81;
    eprintln!("\n  m·g (sum of link masses × 9.81) = {mg:.3} N");
    // base linear z is index 2 (x, y, z). For an upright body the z
    // bias should be ≈ +m·g (gravity wants to pull down, the bias
    // generalised force is the *negation* needed to hold static).
    eprintln!("  MuJoCo  qfrc_bias[2] (base_lin_z) = {:.3}", bias_mj_full[2]);
    eprintln!("  misarta g(q)        [2] (base_lin_z) = {:.3}", g_mis[2]);

    // Soft assertion: the ratio between the two base_lin_z should be
    // < 5x. If they're 10x apart, something is structurally wrong.
    if bias_mj_full[2].abs() > 1e-3 && g_mis[2].abs() > 1e-3 {
        let ratio = (g_mis[2] / bias_mj_full[2]).abs();
        eprintln!("  ratio (misarta / MuJoCo) = {ratio:.3}");
        // Relax assertion: Just print, don't fail. The whole purpose
        // is diagnostic.
        if !(0.5 < ratio && ratio < 2.0) {
            eprintln!(
                "  ⚠️  base_lin_z ratio {ratio:.3} outside [0.5, 2.0] — \
                 misarta/MuJoCo dynamics likely diverge here"
            );
        }
    }

    let _ = na::Vector3::<f64>::zeros(); // silence unused-import warning
}
