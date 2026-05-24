//! End-to-end check: load Go2 MJCF → CHAMP gait controller → in-process
//! MuJoCo dynamics. Confirms the freshly-imported model with
//! class-inherited axes / actuators actually produces a stable standing
//! pose and a forward trot.
//!
//! Run with the `mujoco` feature: `cargo run --features mujoco --example go2_sim`.

#[cfg(feature = "mujoco")]
fn main() {
    use articara::gait::{auto_detect_kinematics_config, GaitController};
    use articara::mjcf::{self, GroundPlaneCfg, MjcfExportOptions};
    use articara::mujoco_sim::MujocoSim;
    use articara::rbd::model::ActuatorMode;
    use articara::robot::*;
    use nalgebra as na;
    use quadruped_gait::{GaitConfig, GaitMode, LegId, VelocityCmd};

    let path = std::path::Path::new("models/unitree_go2/go2.xml");
    let mut model = mjcf::import_mjcf(path).expect("Load Go2");

    // Add foot links (Go2's feet are geoms inside calf bodies, not bodies).
    let foot_offset = na::Vector3::new(-0.002_f32, 0.0, -0.213);
    for (leg, parent) in [
        ("FL_foot", "FL_calf"),
        ("FR_foot", "FR_calf"),
        ("RL_foot", "RL_calf"),
        ("RR_foot", "RR_calf"),
    ] {
        let origin = na::Isometry3::from_parts(
            na::Translation3::from(foot_offset),
            na::UnitQuaternion::identity(),
        );
        model
            .add_child(
                parent, leg, &format!("{leg}_fixed"), "fixed", origin,
                na::Vector3::z(),
                GeomData::Sphere { radius: 0.022 },
                [0.5, 0.5, 0.5, 1.0],
                0.0, 0.0,
            )
            .unwrap();
    }

    // Switch leg joints to Position mode so the gait's joint-angle target
    // is tracked by a PD inside MujocoSim. (As imported, motors are Torque
    // mode — fine for hand-supplied τ but we want PD here.) Tune Kp/Kv
    // to Menagerie-typical values for the Go2 (Kp=20, Kv=0.5).
    for j in model.joints.iter_mut().filter(|j| j.joint_type != "fixed") {
        j.actuator_mode = ActuatorMode::Position;
        j.actuator_kp = 60.0;
        j.actuator_kv = 1.0;
    }

    // Home keyframe: hip=0, thigh=0.9, calf=-1.8.
    let home_q = [
        ("FL_hip_joint", 0.0), ("FL_thigh_joint", 0.9), ("FL_calf_joint", -1.8),
        ("FR_hip_joint", 0.0), ("FR_thigh_joint", 0.9), ("FR_calf_joint", -1.8),
        ("RL_hip_joint", 0.0), ("RL_thigh_joint", 0.9), ("RL_calf_joint", -1.8),
        ("RR_hip_joint", 0.0), ("RR_thigh_joint", 0.9), ("RR_calf_joint", -1.8),
    ];
    for (name, q) in home_q.iter() {
        model.joint_positions[model.joint_map[*name]] = *q;
    }
    model.rebuild_misarta_model();

    // Auto-detect kinematics from the standing pose.
    let foot_links = [
        (LegId::FL, "FL_foot"), (LegId::FR, "FR_foot"),
        (LegId::RL, "RL_foot"), (LegId::RR, "RR_foot"),
    ];
    let kin = auto_detect_kinematics_config(&model, &foot_links)
        .unwrap_or_else(|errs| panic!("kin auto-detect: {errs:?}"));

    let ctrl_built =
        GaitController::build(&model, kin, GaitConfig::trot(), GaitMode::Champ).unwrap();
    let mut ctrl = ctrl_built;

    // Spawn the sim with a ground plane and the floating base at a height
    // matching Go2's keyframe (z=0.27).
    let opts = MjcfExportOptions {
        base_pos: Some([0.0, 0.0, 0.30]),
        ground_plane: Some(GroundPlaneCfg {
            z: 0.0, half_size: 5.0, roll: 0.0, pitch: 0.0,
        }),
        ..MjcfExportOptions::default()
    };
    let mut sim = MujocoSim::new(&model, opts).expect("MujocoSim::new");
    let mj_dt = sim.timestep();
    println!("MuJoCo timestep = {mj_dt} s");

    // Read trunk pose via MujocoSim's xpos/xquat lookups.
    let trunk = |sim: &MujocoSim| -> (na::Vector3<f64>, na::UnitQuaternion<f64>) {
        let p = sim.body_world_position("base").expect("base xpos");
        let r = sim.body_world_orientation("base").expect("base xquat");
        (na::Vector3::new(p[0], p[1], p[2]), r)
    };

    // Phase 1: hold the home pose for 0.5 s so the robot settles.
    println!("\n--- Phase 1: settle at home pose, 0.5 s ---");
    let (p0, _) = trunk(&sim);
    println!("  t=0.000 trunk=({:+.3}, {:+.3}, {:+.3})", p0.x, p0.y, p0.z);
    for (name, q) in home_q.iter() {
        sim.set_position_target(model.joint_map[*name], *q);
    }
    sim.step_n_frames(&mut model, (0.5 / mj_dt) as u32, true);
    let (p1, r1) = trunk(&sim);
    let rpy1 = r1.euler_angles();
    println!("  t=0.500 trunk=({:+.3}, {:+.3}, {:+.3})  rpy=({:+.3},{:+.3},{:+.3})",
        p1.x, p1.y, p1.z, rpy1.0, rpy1.1, rpy1.2);

    // Phase 2: enable gait, command vx=0.3 m/s, 2 s of physics.
    println!("\n--- Phase 2: vx=0.3 m/s trot, 2.0 s ---");
    ctrl.enable();
    ctrl.set_velocity_cmd(VelocityCmd { vx: 0.3, vy: 0.0, wz: 0.0 });

    let gait_dt = 0.005_f64;
    let total_phase2 = 2.0_f64;
    let n_ticks = (total_phase2 / gait_dt) as usize;
    let mut log_at = [0, n_ticks / 4, n_ticks / 2, 3 * n_ticks / 4, n_ticks - 1];
    log_at.sort();

    let mut max_roll: f64 = 0.0;
    let mut max_pitch: f64 = 0.0;
    let mut min_z: f64 = p1.z;
    for tick in 0..n_ticks {
        let (_out, targets, _ff) = ctrl.tick(gait_dt);
        for (ji, q) in targets.iter() {
            sim.set_position_target(*ji, *q);
        }
        sim.step_n_frames(&mut model, (gait_dt / mj_dt).max(1.0) as u32, true);
        let (p, r) = trunk(&sim);
        let rpy = r.euler_angles();
        max_roll = max_roll.max(rpy.0.abs());
        max_pitch = max_pitch.max(rpy.1.abs());
        min_z = min_z.min(p.z);
        if log_at.contains(&tick) {
            let t = (tick + 1) as f64 * gait_dt + 0.5;
            println!(
                "  t={t:.3} trunk=({:+.3}, {:+.3}, {:+.3})  rpy=({:+.3},{:+.3},{:+.3})",
                p.x, p.y, p.z, rpy.0, rpy.1, rpy.2,
            );
        }
    }

    let (pf, _) = trunk(&sim);
    println!("\n=== Result ===");
    println!("  start  trunk x={:+.3} y={:+.3} z={:+.3}", p1.x, p1.y, p1.z);
    println!("  end    trunk x={:+.3} y={:+.3} z={:+.3}", pf.x, pf.y, pf.z);
    println!("  Δx = {:+.3} m  (cmd vx=0.3 m/s × 2 s = 0.6 m expected)", pf.x - p1.x);
    println!("  Δy = {:+.3} m  (lateral drift)", pf.y - p1.y);
    println!("  min z = {min_z:.3} m  (lower = fell)");
    println!("  max |roll|  = {max_roll:.3} rad");
    println!("  max |pitch| = {max_pitch:.3} rad");
    let fell = min_z < 0.15;
    let walked = pf.x - p1.x > 0.10;
    println!(
        "  verdict: {}",
        if fell { "FELL" } else if walked { "WALKED" } else { "STOOD-BUT-DIDN'T-WALK" }
    );
}

#[cfg(not(feature = "mujoco"))]
fn main() {
    eprintln!("This example requires the `mujoco` feature. Run with:");
    eprintln!("  cargo run --features mujoco --example go2_sim");
}
