//! Diagnostic: weld kyo46rs's torso to the world, seed the LEFT ARM at a
//! non-trivial (bent, not hanging-straight) pose, apply ONLY misarta's
//! per-joint gravity-compensation torque (`misarta::rnea::compute_gravity`,
//! recomputed every tick from the current pose) to the shoulder/elbow
//! actuators, and watch whether the arm holds still or sags/drifts.
//!
//! This mirrors `kyo46rs_weld_balance_check.rs`'s method but on the
//! simplest possible chain: a single 2-DOF serial arm hanging off a fixed
//! base, no legs, no contacts, no other tasks in the loop at all -- the
//! cleanest possible isolation of "does misarta's dynamics model of this
//! link chain match what MuJoCo simulates, closely enough that pure
//! gravity torque holds a bent pose statically."
//!
//! shoulder_pitch=0, elbow=0 is already the trivial hanging-straight-down
//! equilibrium (zero gravity torque needed), so this seeds a bent pose
//! (shoulder raised forward, elbow bent) where holding still is a real
//! test, not a no-op.
//!
//! Run with: `cargo run --features mujoco --example kyo46rs_arm_gravity_check`

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

    // Non-trivial seed: shoulder raised forward, elbow bent -- q=0 for
    // both is already the hanging-straight-down equilibrium (zero
    // gravity torque needed), which would be a no-op test.
    let arm_pose = [
        ("left_shoulder_pitch_joint", -1.0_f64),
        ("left_elbow_joint", 1.2_f64),
    ];
    for (name, q) in arm_pose {
        if let Some(&ji) = robot.joint_map.get(name) {
            robot.joint_positions[ji] = q;
        }
    }
    robot.rebuild_misarta_model();

    for j in robot.joints.iter_mut() {
        j.actuator_mode = ActuatorMode::Torque;
    }

    // Weld the torso -- no free joint, no ground/contacts needed; the
    // arm is cantilevered off a fixed mount, isolating pure link-chain
    // dynamics from any floating-base/attitude/contact question.
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

    println!("=== kyo46rs left-arm-only gravity-compensation hold ===");
    println!("Seed pose: shoulder_pitch = -1.000 rad, elbow = 1.200 rad");
    let tau0 = compute_tau(&robot);
    for (name, _) in arm_pose {
        if let Some(&ji) = robot.joint_map.get(name) {
            println!("  {name:<28} gravity-comp tau (t=0) = {:+.4} N*m", tau0[ji]);
        }
    }

    let q0: Vec<f64> = robot
        .joints
        .iter()
        .map(|j| sim.joint_q_qd(&j.name).map(|(q, _)| q).unwrap_or(0.0))
        .collect();

    // Only the left arm's two joints get the (continuously recomputed)
    // gravity-compensation torque; every other joint gets zero torque
    // (the base is welded, so nothing else needs to hold anything --
    // the point is to isolate this one arm's behaviour).
    println!("\n=== Holding for 3.0 s, gravity-comp torque on left arm ONLY ===");
    let n_ticks = (3.0 / mj_dt) as u32;
    let report_every = (0.2 / mj_dt) as u32;
    let mut max_drift: f64 = 0.0;
    for tick in 0..n_ticks {
        let tau_g = compute_tau(&robot);
        let mut tau_cmd = vec![0.0_f64; robot.joints.len()];
        for (name, _) in arm_pose {
            if let Some(&ji) = robot.joint_map.get(name) {
                tau_cmd[ji] = tau_g[ji];
            }
        }
        sim.set_wbc_torques(&tau_cmd);
        sim.step_n_frames(&mut robot, 1, true);

        if tick % report_every == 0 {
            let t = tick as f64 * mj_dt;
            let (sp_q, sp_qd) = sim.joint_q_qd("left_shoulder_pitch_joint").unwrap_or((f64::NAN, f64::NAN));
            let (el_q, el_qd) = sim.joint_q_qd("left_elbow_joint").unwrap_or((f64::NAN, f64::NAN));
            let drift = (sp_q - q0[*robot.joint_map.get("left_shoulder_pitch_joint").unwrap()]).abs()
                .max((el_q - q0[*robot.joint_map.get("left_elbow_joint").unwrap()]).abs());
            max_drift = max_drift.max(drift);
            println!(
                "  t={t:5.2}s  shoulder_pitch={sp_q:+.4} (qd={sp_qd:+.4})  elbow={el_q:+.4} (qd={el_qd:+.4})"
            );
        }
    }

    println!("\n=== Final result ===");
    for (name, target) in arm_pose {
        let (q, qd) = sim.joint_q_qd(name).unwrap_or((f64::NAN, f64::NAN));
        println!(
            "  {name:<28} seed={target:+.3} now={q:+.4} drift={:+.4} rad  qd={qd:+.4} rad/s",
            q - target
        );
    }
    println!("  max drift observed over 3.0s: {max_drift:.4} rad");
    println!(
        "  verdict: {}",
        if max_drift < 0.05 { "HOLDS (gravity-comp alone is enough)" } else { "DRIFTS (gravity-comp alone is not enough to hold this pose)" }
    );
}

#[cfg(not(feature = "mujoco"))]
fn main() {
    eprintln!("This example requires the `mujoco` feature. Run with:");
    eprintln!("  cargo run --features mujoco --example kyo46rs_arm_gravity_check");
}
