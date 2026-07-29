//! Diagnostic: weld kyo46rs's torso to the world, seed a FULL-BODY pose
//! (both legs crouched, both arms bent to a non-trivial pose), apply ONLY
//! misarta's per-joint gravity-compensation torque (recomputed every tick
//! from the current pose) to every actuated joint, and watch whether the
//! whole body holds still or sags/drifts.
//!
//! Extends `kyo46rs_arm_gravity_check.rs` (single arm, held PERFECTLY,
//! zero drift over 3s) and `kyo46rs_weld_balance_check.rs` (both legs
//! only, drifted substantially) to the combined case, so per-joint drift
//! can be compared side by side within the SAME run: is the earlier legs
//! drift specific to the leg chain/seed pose, or does adding the arms
//! change anything?
//!
//! Run with: `cargo run --features mujoco --example kyo46rs_fullbody_gravity_check`

#[cfg(feature = "mujoco")]
fn main() {
    use articara::mjcf::MjcfExportOptions;
    use articara::mujoco_sim::MujocoSim;
    use articara::rbd::model::ActuatorMode;
    use articara::robot::RobotModel;
    use articara::wbc_pipeline::build_floating_base_model;

    let urdf_path = std::path::Path::new(
        "/home/takara/work/dp/humanoid/kyo46rs_description/urdf/kyo46rs.urdf",
    );
    let mut robot = RobotModel::from_urdf(urdf_path).expect("load kyo46rs.urdf");

    // Same crouch seed as kyo46rs_squat.rs / kyo46rs_weld_balance_check.rs,
    // plus the same non-trivial arm pose as kyo46rs_arm_gravity_check.rs
    // (shoulder_pitch=0/elbow=0 is the trivial hanging-straight-down
    // equilibrium -- bending both is what makes "holds vs. drifts" a
    // real question rather than a no-op).
    let seed_pose = [
        ("left_hip_pitch_joint", -0.35),
        ("left_knee_joint", 0.70),
        ("left_ankle_pitch_joint", -0.45),
        ("right_hip_pitch_joint", -0.35),
        ("right_knee_joint", 0.70),
        ("right_ankle_pitch_joint", -0.45),
        ("left_shoulder_pitch_joint", -1.0),
        ("left_elbow_joint", 1.2),
        ("right_shoulder_pitch_joint", -1.0),
        ("right_elbow_joint", 1.2),
    ];
    for (name, q) in seed_pose {
        if let Some(&ji) = robot.joint_map.get(name) {
            robot.joint_positions[ji] = q;
        }
    }
    robot.rebuild_misarta_model();

    for j in robot.joints.iter_mut() {
        j.actuator_mode = ActuatorMode::Torque;
    }

    // Weld the torso so it can't fall to the ground -- isolates pure
    // link-chain dynamics (legs + arms, all cantilevered off a fixed
    // mount) from any floating-base/attitude/contact question, exactly
    // like the single-arm and legs-only weld tests.
    let opts = MjcfExportOptions {
        base_pos: Some([0.0, 0.0, 0.6]),
        base_locked_axes: [true; 6],
        ..MjcfExportOptions::default()
    };
    let mut sim = MujocoSim::new(&robot, opts).expect("MujocoSim::new");
    let mj_dt = sim.timestep();

    let (model, a2m, _link_to_idx) = build_floating_base_model(&robot);

    let compute_tau = |robot: &RobotModel| -> Vec<f64> {
        let mut q = model.neutral_q();
        q[6] = 1.0;
        q[2] = 0.6;
        for ji in 0..robot.joints.len() {
            let Some(mi) = a2m[ji] else { continue };
            if model.joints[mi].joint_type.nq() == 1 {
                q[model.q_idx[mi]] = robot.joint_positions[ji];
            }
        }
        let g_mis = misarta::rnea::compute_gravity(&model, &q);
        let mut tau = vec![0.0_f64; robot.joints.len()];
        for ji in 0..robot.joints.len() {
            let Some(mi) = a2m[ji] else { continue };
            if model.joints[mi].joint_type.nv() != 1 {
                continue;
            }
            let vi = model.v_idx[mi];
            if vi < 6 {
                continue;
            }
            tau[ji] = g_mis[vi];
        }
        tau
    };

    println!("=== kyo46rs full-body gravity-compensation hold (torso welded) ===");
    let tau0 = compute_tau(&robot);
    let bias0 = sim.qfrc_bias();
    for (name, _) in seed_pose {
        if let Some(&ji) = robot.joint_map.get(name) {
            let mj_val = sim.joint_dof_adr(name).map(|d| bias0[d]);
            println!(
                "  {name:<28} misarta tau (t=0) = {:+.4} N*m   mujoco qfrc_bias = {:?}",
                tau0[ji], mj_val
            );
        }
    }

    let q0: std::collections::HashMap<&str, f64> = seed_pose
        .iter()
        .map(|(name, _)| {
            let q = sim.joint_q_qd(name).map(|(q, _)| q).unwrap_or(0.0);
            (*name, q)
        })
        .collect();

    println!("\n=== Holding for 3.0 s, gravity-comp torque on ALL joints ===");
    let n_ticks = (3.0 / mj_dt) as u32;
    let report_every = (0.3 / mj_dt) as u32;
    let mut max_drift_per_joint: std::collections::HashMap<&str, f64> =
        seed_pose.iter().map(|(n, _)| (*n, 0.0)).collect();

    for tick in 0..n_ticks {
        let tau_cmd = compute_tau(&robot);
        sim.set_wbc_torques(&tau_cmd);
        sim.step_n_frames(&mut robot, 1, true);

        for (name, _) in seed_pose {
            if let Some((q, _)) = sim.joint_q_qd(name) {
                let drift = (q - q0[name]).abs();
                let e = max_drift_per_joint.get_mut(name).unwrap();
                if drift > *e {
                    *e = drift;
                }
            }
        }

        if tick % report_every == 0 {
            let t = tick as f64 * mj_dt;
            let mut worst_name = "";
            let mut worst_drift = -1.0_f64;
            for (name, _) in seed_pose {
                let d = (sim.joint_q_qd(name).map(|(q, _)| q).unwrap_or(0.0) - q0[name]).abs();
                if d > worst_drift {
                    worst_drift = d;
                    worst_name = name;
                }
            }
            println!("  t={t:5.2}s  worst joint = {worst_name:<28} drift = {worst_drift:.4} rad");
        }
    }

    println!("\n=== Final per-joint drift from seed pose (after 3.0 s) ===");
    println!("  {:<28} {:>8} {:>8} {:>8}", "joint", "seed", "final", "drift");
    for (name, target) in seed_pose {
        let (q, _qd) = sim.joint_q_qd(name).unwrap_or((f64::NAN, f64::NAN));
        println!("  {name:<28} {target:>+8.3} {q:>+8.4} {:>+8.4}", q - target);
    }

    println!("\n=== Max drift observed over the full 3.0 s (per joint) ===");
    let mut sorted: Vec<_> = max_drift_per_joint.into_iter().collect();
    sorted.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap());
    for (name, drift) in &sorted {
        println!("  {name:<28} max |drift| = {drift:.4} rad");
    }

    let worst = sorted.first().map(|(_, d)| *d).unwrap_or(0.0);
    println!(
        "\nverdict: {}",
        if worst < 0.05 {
            "HOLDS (gravity-comp alone is enough, whole body)"
        } else {
            "DRIFTS (gravity-comp alone is not enough for at least one joint)"
        }
    );
}

#[cfg(not(feature = "mujoco"))]
fn main() {
    eprintln!("This example requires the `mujoco` feature. Run with:");
    eprintln!("  cargo run --features mujoco --example kyo46rs_fullbody_gravity_check");
}
