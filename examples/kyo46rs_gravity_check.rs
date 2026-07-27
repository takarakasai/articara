//! Diagnostic: does misarta's `compute_gravity(q)` match MuJoCo's own
//! `qfrc_bias` (== g(q) when qvel=0) at kyo46rs's crouch pose?
//!
//! `kyo46rs_squat`'s WBC uses `misarta::rnea::compute_gravity` both as
//! the P2 regularizer anchor AND (implicitly, via `nonlinear_effects`
//! in `h`) inside the EoM task that ties `tau` to the commanded
//! `qddot`. If misarta's model of the robot's mass/inertia disagrees
//! with what MuJoCo actually simulates, the WBC's solved torques are
//! systematically wrong from the very first tick -- which would
//! explain the ~5 rad/s unprovoked pitch-rate spike seen in the first
//! 30-40ms of `kyo46rs_squat`, before the attitude task has any
//! meaningful error to react to.
//!
//! Run with: `cargo run --features mujoco --example kyo46rs_gravity_check`

#[cfg(feature = "mujoco")]
fn main() {
    use articara::mjcf::{GroundPlaneCfg, MjcfExportOptions};
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
        j.actuator_mode = ActuatorMode::Position;
        j.actuator_kp = 40.0;
        j.actuator_kv = 2.0;
    }
    let opts = MjcfExportOptions {
        base_pos: Some([0.0, 0.0, 0.41]),
        ground_plane: Some(GroundPlaneCfg {
            z: 0.0,
            half_size: 2.0,
            roll: 0.0,
            pitch: 0.0,
        }),
        ..MjcfExportOptions::default()
    };
    let mut sim = MujocoSim::new(&robot, opts).expect("MujocoSim::new");
    let mj_dt = sim.timestep();

    // Settle to the crouch pose (same burn-in as kyo46rs_squat), then
    // hold there with qvel ~ 0 so qfrc_bias = C(q,v)v + g(q) collapses
    // to pure g(q) -- same rationale as
    // misarta_mujoco_gravity_consistency.rs's existing quadruped test.
    for (name, q) in crouch {
        if let Some(&ji) = robot.joint_map.get(name) {
            sim.set_position_target(ji, q);
        }
    }
    sim.step_n_frames(&mut robot, (0.15 / mj_dt) as u32, true);

    let (model, a2m, _link_to_idx) = build_floating_base_model(&robot);

    let body_pos = sim.body_world_position(&robot.root_link).expect("torso xpos");
    let body_quat = sim
        .body_world_orientation(&robot.root_link)
        .expect("torso xquat");
    let mut q = model.neutral_q();
    q[0] = body_pos[0];
    q[1] = body_pos[1];
    q[2] = body_pos[2];
    q[3] = body_quat.i;
    q[4] = body_quat.j;
    q[5] = body_quat.k;
    q[6] = body_quat.w;
    for ji in 0..robot.joints.len() {
        let Some(mi) = a2m[ji] else { continue };
        if model.joints[mi].joint_type.nq() == 1 {
            q[model.q_idx[mi]] = robot.joint_positions[ji];
        }
    }

    let g_mis = misarta::rnea::compute_gravity(&model, &q);
    let bias_mj = sim.qfrc_bias();

    // NOTE: the two engines use DIFFERENT free-joint DOF orderings --
    // misarta's motion_subspace is [angular(3); linear(3)] (confirmed
    // in misarta's joint.rs), while MuJoCo's native free-joint qvel
    // layout is [linear(3); angular(3)]. Label each side with its own
    // convention rather than a shared index.
    println!("=== base (free-flyer) block: misarta g(q) vs MuJoCo qfrc_bias ===");
    let mis_labels = ["ang_x", "ang_y", "ang_z", "lin_x", "lin_y", "lin_z"];
    let mj_labels = ["lin_x", "lin_y", "lin_z", "ang_x", "ang_y", "ang_z"];
    for i in 0..6 {
        println!(
            "  [{i}] misarta {:<6}={:+9.4}   mujoco {:<6}={:+9.4}",
            mis_labels[i], g_mis[i], mj_labels[i], bias_mj[i]
        );
    }

    println!("\n=== per actuated joint: misarta g(q) vs MuJoCo qfrc_bias ===");
    println!(
        "  {:<28} {:>12} {:>12} {:>10}",
        "joint", "misarta", "mujoco", "ratio"
    );
    let mut worst_ratio: f64 = 1.0;
    let mut worst_name = String::new();
    for ji in 0..robot.joints.len() {
        let Some(mi) = a2m[ji] else { continue };
        if model.joints[mi].joint_type.nv() != 1 {
            continue;
        }
        let vi = model.v_idx[mi];
        let name = &robot.joints[ji].name;
        let Some(dof) = sim.joint_dof_adr(name) else {
            println!("  {name:<28} (not in compiled MJCF)");
            continue;
        };
        let mis_val = g_mis[vi];
        let mj_val = bias_mj[dof];
        let ratio = if mj_val.abs() > 1e-3 {
            mis_val / mj_val
        } else {
            f64::NAN
        };
        println!("  {name:<28} {mis_val:+12.4} {mj_val:+12.4} {ratio:+10.3}");
        if ratio.is_finite() && (ratio - 1.0).abs() > (worst_ratio - 1.0).abs() {
            worst_ratio = ratio;
            worst_name = name.clone();
        }
    }
    println!("\nworst per-joint ratio: {worst_name} = {worst_ratio:.3} (1.0 = perfect agreement)");

    let total_mass: f64 = robot.links.iter().map(|l| l.inertial.mass).sum();
    println!("\nrobot total mass = {total_mass:.3} kg, m*g = {:.3} N", total_mass * 9.81);
    println!("misarta base lin_z g(q)[5] = {:+.4} N", g_mis[5]);
    println!("mujoco  base lin_z qfrc_bias[2] = {:+.4} N", bias_mj[2]);
}

#[cfg(not(feature = "mujoco"))]
fn main() {
    eprintln!("This example requires the `mujoco` feature. Run with:");
    eprintln!("  cargo run --features mujoco --example kyo46rs_gravity_check");
}
