//! LinearCrawl パラメタ・スイープ: vx=0.05 m/s で歩かせたときの
//! body 揺らぎ (y / z / roll / pitch / yaw) を 5 種類の設定で計測して
//! 並べる。
//!
//! 実行: `cargo xtask run --features mujoco --example go2_linear_crawl_sweep`

#[cfg(feature = "mujoco")]
fn main() {
    use articara::gait::{auto_detect_kinematics_config, GaitController};
    use articara::mjcf::{GroundPlaneCfg, MjcfExportOptions};
    use articara::mujoco_sim::MujocoSim;
    use articara::rbd::model::ActuatorMode;
    use articara::robot::*;
    use nalgebra as na;
    use quadruped_gait::{GaitConfig, GaitMode, LegId, VelocityCmd};

    // (label, t3 [s], t4 [s], swing_h [m], Kp [N·m/rad])
    let configs: &[(&str, f64, f64, f64, f64)] = &[
        ("(0) baseline α=0.5  Kp=500           ", 0.2083, 0.2083, 0.005, 500.0),
        ("(1) α=0.75            Kp=500           ", 0.1042, 0.3125, 0.005, 500.0),
        ("(2) α=0.85            Kp=500           ", 0.0625, 0.3542, 0.005, 500.0),
        ("(3) α=0.85            Kp=1000          ", 0.0625, 0.3542, 0.005, 1000.0),
        ("(4) α=0.85 swing 0.003 Kp=1000         ", 0.0625, 0.3542, 0.003, 1000.0),
    ];

    let cmd_vx = 0.05_f64;
    let walk_s = 5.0_f64;
    let gait_dt = 0.005_f64;

    println!(
        "vx={cmd_vx} m/s, walk {walk_s} s per config.\n\
         label                                     |   Δx [m] |    |y|p |   Δz | |roll|max | |pitch|max | |yaw|max"
    );
    println!("-------------------------------------------+----------+--------+-------+-----------+------------+----------");

    for &(label, t3, t4, swing_h, kp) in configs {
        // Fresh load each iteration so previous run's residual pose
        // doesn't taint the next.
        let misa = std::path::Path::new("models/unitree_go2/go2.misa");
        let mut model = RobotModel::from_misa(misa).expect(".misa load");

        for j in model.joints.iter_mut().filter(|j| j.joint_type != "fixed") {
            j.actuator_mode = ActuatorMode::Position;
            j.actuator_kp = kp;
            j.actuator_kv = (kp / 100.0).max(2.0); // crude scale so Kv tracks Kp
        }

        let home_q = [
            ("FL_hip_joint", 0.0), ("FL_thigh_joint", 0.9), ("FL_calf_joint", -1.8),
            ("FR_hip_joint", 0.0), ("FR_thigh_joint", 0.9), ("FR_calf_joint", -1.8),
            ("RL_hip_joint", 0.0), ("RL_thigh_joint", 0.9), ("RL_calf_joint", -1.8),
            ("RR_hip_joint", 0.0), ("RR_thigh_joint", 0.9), ("RR_calf_joint", -1.8),
        ];
        for (name, q) in home_q.iter() {
            let ji = *model.joint_map.get(*name).unwrap();
            model.joint_positions[ji] = *q;
        }
        model.rebuild_misarta_model();

        let foot_links = [
            (LegId::FL, "FL_foot"), (LegId::FR, "FR_foot"),
            (LegId::RL, "RL_foot"), (LegId::RR, "RR_foot"),
        ];
        let kin = auto_detect_kinematics_config(&model, &foot_links).unwrap();

        let big_t = 4.0 * (t3 + t4);
        let alpha = t4 / (t3 + t4);
        let gait_cfg = GaitConfig::crawl()
            .with_cycle_period(big_t)
            .with_four_support_fraction(alpha)
            .with_swing_height(swing_h)
            // Disable the swing-foot feasibility cap: this sweep measures the
            // raw open-loop behaviour at each α, so the forward speed must
            // stay fixed at `cmd_vx` rather than being auto-reduced.
            .with_max_swing_foot_speed(0.0);
        let mut gc = GaitController::build(&model, kin, gait_cfg, GaitMode::LinearCrawl).unwrap();

        let opts = MjcfExportOptions {
            base_pos: Some([0.0, 0.0, 0.30]),
            ground_plane: Some(GroundPlaneCfg {
                z: 0.0, half_size: 5.0, roll: 0.0, pitch: 0.0,
            }),
            ..MjcfExportOptions::default()
        };
        let mut sim = MujocoSim::new(&model, opts).expect("MujocoSim::new");
        let mj_dt = sim.timestep();

        // Settle.
        for (name, q) in home_q.iter() {
            sim.set_position_target(model.joint_map[*name], *q);
        }
        sim.step_n_frames(&mut model, (0.5 / mj_dt) as u32, true);
        let p_start = sim.body_world_position("base").unwrap();
        let p_start = na::Vector3::new(p_start[0], p_start[1], p_start[2]);

        gc.set_velocity_cmd(VelocityCmd { vx: cmd_vx, vy: 0.0, wz: 0.0 });
        gc.enable();

        let mut max_roll: f64 = 0.0;
        let mut max_pitch: f64 = 0.0;
        let mut max_yaw: f64 = 0.0;
        let mut min_z = p_start.z;
        let mut max_z = p_start.z;
        let mut max_abs_y = 0.0_f64;

        let n_ticks = (walk_s / gait_dt) as usize;
        for _ in 0..n_ticks {
            let (_out, targets, _ff) = gc.tick(gait_dt);
            for (ji, q) in targets.iter() {
                sim.set_position_target(*ji, *q);
            }
            sim.step_n_frames(&mut model, (gait_dt / mj_dt).max(1.0) as u32, true);
            let p = sim.body_world_position("base").unwrap();
            let r = sim.body_world_orientation("base").unwrap();
            let rpy = r.euler_angles();
            max_roll = max_roll.max(rpy.0.abs());
            max_pitch = max_pitch.max(rpy.1.abs());
            max_yaw = max_yaw.max(rpy.2.abs());
            min_z = min_z.min(p[2]);
            max_z = max_z.max(p[2]);
            max_abs_y = max_abs_y.max((p[1] - p_start.y).abs());
        }

        let p_end = sim.body_world_position("base").unwrap();
        let dx = p_end[0] - p_start.x;
        let dz = max_z - min_z;
        println!(
            "{label}|  {dx:+.3}  | {max_abs_y:.4} | {dz:.4} |  {max_roll:.4}   |   {max_pitch:.4}   |  {max_yaw:.4}",
        );
    }
}

#[cfg(not(feature = "mujoco"))]
fn main() {
    eprintln!("Requires `mujoco` feature.");
}
