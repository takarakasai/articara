//! Do MuJoCo's reported base orientation and misarta's FK of the SAME q agree?
//!
//! The trunk task takes its Jacobian from misarta and its error from MuJoCo.
//! Those must describe the same rotation -- misarta's q is synced from that
//! very quaternion each tick -- yet swapping the error source changes whether
//! kyo46rs stands. One of them is not what it claims to be.
#[cfg(feature = "mujoco")]
fn main() {
    use articara::mjcf::{GroundPlaneCfg, MjcfExportOptions};
    use articara::mujoco_sim::MujocoSim;
    use articara::robot::RobotModel;
    use articara::wbc_pipeline::build_floating_base_model;

    let which = std::env::var("ROBOT").unwrap_or_else(|_| "kyo46rs".into());
    let (urdf, root) = if which.starts_with("g1") {
        ("/home/takara/work/dp/articara/models/unitree_g1_src/robots/g1_description/g1_23dof.urdf", "pelvis")
    } else {
        ("/home/takara/work/dp/humanoid/kyo46rs_description/urdf/kyo46rs.urdf", "torso")
    };
    let mut robot = RobotModel::from_urdf(std::path::Path::new(urdf)).unwrap();
    robot.rebuild_misarta_model();
    let opts = MjcfExportOptions {
        base_pos: Some([0.0, 0.0, if which.starts_with("g1") { 0.9 } else { 0.5 }]),
        ground_plane: Some(GroundPlaneCfg { z: -5.0, half_size: 2.0, roll: 0.0, pitch: 0.0 }),
        timestep: Some(0.001),
        ..MjcfExportOptions::default()
    };
    let mut sim = MujocoSim::new(&robot, opts).expect("sim");
    let (model, a2m, _l2i) = build_floating_base_model(&robot);

    // Spin the base to a GENERAL orientation. Free fall left it upright, and
    // near identity a frame offset or a convention mismatch both round to
    // zero -- the comparison has to be made where roll, pitch and yaw are all
    // large and mutually coupled.
    {
        use articara::rbd::model::ActuatorMode;
        for j in robot.joints.iter_mut() {
            j.actuator_mode = ActuatorMode::Torque;
        }
    }
    println!("robot={which}  root={root}");
    println!("{:>7} {:>9} {:>9} {:>9} | {:>9} {:>9} {:>9} | {:>8}",
             "t", "mj_roll", "mj_pitch", "mj_yaw", "fk_roll", "fk_pitch", "fk_yaw", "max_diff");
    for step in 0..=6 {
        if step == 0 {
            // Spin it up about a skew axis so roll, pitch and yaw all become
            // large and coupled -- near identity a frame offset and a
            // convention mismatch both round to zero.
            sim.apply_external_force(root, [0.0, 0.0, 0.0], [7.0, -4.0, 9.0], 0.05);
        }
        sim.step_n_frames(&mut robot, 120, false);
        let p = sim.body_world_position(root).unwrap();
        let q_mj = sim.body_world_orientation(root).unwrap();
        let (mr, mp, my) = q_mj.euler_angles();

        // Build misarta q exactly as the controller does, then run its FK.
        let mut q: Vec<f64> = model.neutral_q();
        q[0] = p[0]; q[1] = p[1]; q[2] = p[2];
        q[3] = q_mj.i; q[4] = q_mj.j; q[5] = q_mj.k; q[6] = q_mj.w;
        for ji in 0..robot.joints.len() {
            let Some(mi) = a2m[ji] else { continue };
            if model.joints[mi].joint_type.nv() != 1 { continue; }
            q[model.q_idx[mi]] = robot.joint_positions[ji];
        }
        let data = misarta::fk::forward_kinematics(&model, &q);
        let rot = misarta::se3::rotation_matrix(&data.oMi[1]);
        let fr = rot[(2, 1)].atan2(rot[(2, 2)]);
        let fp = (-rot[(2, 0)]).asin();
        let fy = rot[(1, 0)].atan2(rot[(0, 0)]);
        let d = (mr - fr).abs().max((mp - fp).abs()).max((my - fy).abs());
        println!("{:7.3} {:9.4} {:9.4} {:9.4} | {:9.4} {:9.4} {:9.4} | {:8.2e}",
                 step as f64 * 0.12, mr, mp, my, fr, fp, fy, d);
    }
}

#[cfg(not(feature = "mujoco"))]
fn main() {}
