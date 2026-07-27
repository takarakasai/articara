//! Is the robot colliding with itself before anything even moves?
fn main() {
    use articara::mjcf::{GroundPlaneCfg, MjcfExportOptions};
    use articara::mujoco_sim::MujocoSim;
    use articara::robot::RobotModel;
    let mut robot = RobotModel::from_urdf(std::path::Path::new(
        "/home/takara/work/dp/humanoid/kyo46rs_description/urdf/kyo46rs.urdf",
    )).unwrap();
    for (n, q) in [("left_hip_pitch_joint",-0.35),("left_knee_joint",0.70),("left_ankle_pitch_joint",-0.35),
                   ("right_hip_pitch_joint",-0.35),("right_knee_joint",0.70),("right_ankle_pitch_joint",-0.35)] {
        if let Some(&ji) = robot.joint_map.get(n) { robot.joint_positions[ji] = q; }
    }
    if std::env::var("NO_ARM").is_ok() {
        for l in robot.links.iter_mut() {
            if l.name.contains("forearm") || l.name.contains("upper_arm") {
                l.collision_enabled = false;
            }
        }
    }
    robot.rebuild_misarta_model();
    let opts = MjcfExportOptions {
        base_pos: Some([0.0, 0.0, 0.60]),
        ground_plane: Some(GroundPlaneCfg { z: 0.0, half_size: 2.0, roll: 0.0, pitch: 0.0 }),
        timestep: Some(0.001),
        ..MjcfExportOptions::default()
    };
    let mut sim = MujocoSim::new(&robot, opts).expect("sim");
    sim.step_n_frames(&mut robot, 1, false);
    let cs = sim.contacts();
    println!("total contacts at the crouch spawn pose: {}", cs.len());
    let mut tot = 0.0;
    for c in &cs {
        let ground = c.body1.is_empty() || c.body2.is_empty();
        let f = (c.force_world[0].powi(2)+c.force_world[1].powi(2)+c.force_world[2].powi(2)).sqrt();
        tot += if ground { 0.0 } else { f };
        println!("  {:<22} <-> {:<22} |f| = {:9.2} N   {}",
            if c.body1.is_empty() {"WORLD"} else {&c.body1},
            if c.body2.is_empty() {"WORLD"} else {&c.body2},
            f, if ground {"(ground)"} else {"*** SELF ***"});
    }
    println!("\ntotal SELF-collision force: {tot:.1} N");
}
