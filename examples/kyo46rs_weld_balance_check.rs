//! Diagnostic: weld kyo46rs's torso rigidly to the world (no free
//! joint at all -- `base_locked_axes: [true; 6]`), apply ONLY a
//! per-joint gravity-compensation torque to the leg actuators via
//! `set_wbc_torques`, and watch whether the crouch pose holds still
//! or drifts/sags. Runs the trial twice: once with misarta's
//! `compute_gravity(q)`, once with MuJoCo's own `qfrc_bias()` read at
//! the identical pose -- if misarta's number fails to hold the pose
//! but MuJoCo's own number does, the bug is in misarta's dynamics; if
//! BOTH fail identically, the bug is in how the torque gets applied
//! (actuator gearing, sign, etc.), not in either dynamics engine.
//!
//! This is a cleaner cross-check than `kyo46rs_gravity_check`'s
//! side-by-side number comparison: with the base welded there is no
//! floating-base integration, no contact-force allocation, and no
//! attitude/height task in the loop at all.
//!
//! Run with: `cargo run --features mujoco --example kyo46rs_weld_balance_check`

#[cfg(feature = "mujoco")]
fn run_trial(label: &str, use_mujoco_bias: bool) {
    use articara::mjcf::MjcfExportOptions;
    use articara::mujoco_sim::MujocoSim;
    use articara::rbd::model::ActuatorMode;
    use articara::robot::RobotModel;
    use articara::wbc_pipeline::build_floating_base_model;

    let urdf_path = std::path::Path::new(
        "/home/takara/work/dp/humanoid/kyo46rs_description/urdf/kyo46rs.urdf",
    );
    let mut robot = RobotModel::from_urdf(urdf_path).expect("load kyo46rs.urdf");

    let crouch = [
        ("left_hip_pitch_joint", -0.35),
        ("left_knee_joint", 0.70),
        ("left_ankle_pitch_joint", -0.45),
        ("right_hip_pitch_joint", -0.35),
        ("right_knee_joint", 0.70),
        ("right_ankle_pitch_joint", -0.45),
    ];
    for (name, q) in crouch {
        if let Some(&ji) = robot.joint_map.get(name) {
            robot.joint_positions[ji] = q;
        }
    }
    robot.rebuild_misarta_model();

    for j in robot.joints.iter_mut() {
        j.actuator_mode = ActuatorMode::Torque;
    }

    let opts = MjcfExportOptions {
        base_pos: Some([0.0, 0.0, 0.6]),
        base_locked_axes: [true; 6],
        ..MjcfExportOptions::default()
    };
    let mut sim = MujocoSim::new(&robot, opts).expect("MujocoSim::new");
    let mj_dt = sim.timestep();

    let (model, a2m, _link_to_idx) = build_floating_base_model(&robot);

    // Recompute gravity comp EVERY tick from the CURRENT joint angles
    // (matching how kyo46rs_squat.rs's P2 anchor actually works) rather
    // than freezing a single value computed once at t=0. A frozen
    // torque has no restoring tendency at all once the pose drifts
    // even slightly (the true gravity moment changes with angle, the
    // applied one doesn't) -- that alone would make ANY frozen-torque
    // hold eventually settle away from the target, independent of
    // whether the torque was numerically "correct" at t=0.
    let compute_tau = |robot: &articara::robot::RobotModel, sim: &MujocoSim| -> Vec<f64> {
        let mut q = model.neutral_q();
        q[6] = 1.0;
        q[2] = 0.6;
        for ji in 0..robot.joints.len() {
            let Some(mi) = a2m[ji] else { continue };
            if model.joints[mi].joint_type.nq() == 1 {
                q[model.q_idx[mi]] = robot.joint_positions[ji];
            }
        }
        let bias_mj = sim.qfrc_bias();
        let g_mis = misarta::rnea::compute_gravity(&model, &q);
        let mut tau = vec![0.0_f64; robot.joints.len()];
        for ji in 0..robot.joints.len() {
            if use_mujoco_bias {
                if let Some(dof) = sim.joint_dof_adr(&robot.joints[ji].name) {
                    tau[ji] = bias_mj[dof];
                }
                continue;
            }
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

    println!("\n########## trial: {label} ##########");
    let tau0 = compute_tau(&robot, &sim);
    for (name, _) in crouch {
        if let Some(&ji) = robot.joint_map.get(name) {
            println!("  {name:<28} applied tau (t=0) = {:+.4} N*m", tau0[ji]);
        }
    }

    let q0: Vec<f64> = robot
        .joints
        .iter()
        .map(|j| sim.joint_q_qd(&j.name).map(|(q, _)| q).unwrap_or(0.0))
        .collect();

    const KD: f64 = 0.6;
    let n_ticks = (2.0 / mj_dt) as u32;
    let report_every = (0.4 / mj_dt) as u32;
    for tick in 0..n_ticks {
        let mut tau_cmd = compute_tau(&robot, &sim);
        for (ji, joint) in robot.joints.iter().enumerate() {
            if let Some((_, qd)) = sim.joint_q_qd(&joint.name) {
                tau_cmd[ji] -= KD * qd;
            }
        }
        sim.set_wbc_torques(&tau_cmd);
        sim.step_n_frames(&mut robot, 1, true);
        if tick % report_every == 0 {
            let t = tick as f64 * mj_dt;
            let mut max_drift: f64 = 0.0;
            let mut worst = "";
            for (ji, joint) in robot.joints.iter().enumerate() {
                if let Some((q, _)) = sim.joint_q_qd(&joint.name) {
                    let drift = (q - q0[ji]).abs();
                    if drift > max_drift {
                        max_drift = drift;
                        worst = &joint.name;
                    }
                }
            }
            println!("  t={t:5.2}s  max |drift| = {max_drift:.4} rad  (worst joint: {worst})");
        }
    }

    println!("  --- final per-joint drift from crouch target ---");
    for (name, target) in crouch {
        let (q, qd) = sim.joint_q_qd(name).unwrap_or((f64::NAN, f64::NAN));
        println!(
            "  {name:<28} target={target:+.3} now={q:+.4} drift={:+.4} rad  qd={qd:+.4} rad/s",
            q - target
        );
    }
}

#[cfg(feature = "mujoco")]
fn main() {
    run_trial("misarta compute_gravity(q)", false);
    run_trial("MuJoCo's own qfrc_bias() at the same pose", true);
}

#[cfg(not(feature = "mujoco"))]
fn main() {
    eprintln!("This example requires the `mujoco` feature. Run with:");
    eprintln!("  cargo run --features mujoco --example kyo46rs_weld_balance_check");
}
