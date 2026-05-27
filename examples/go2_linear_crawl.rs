//! Go2 を [`quadruped_gait::GaitMode::LinearCrawl`] で走らせ、GUI と同じ
//! `articara::gait::GaitController::build` 経路で開ループ +X 直線歩容が
//! 再現できることを確認する。
//!
//! 仕様 (= GUI の Mode ドロップダウンで "Linear crawl (open-loop)" を選び、
//! `four_support_fraction` スライダを動かして同じ結果が得られるはず):
//!   1. Open-loop (IMU フィードバックなし)
//!   2. Crawl — 4 脚支持と 3 脚支持を繰り返し、1 サイクルで 4 脚を 1 本ずつ swing
//!   3. 体幹は +X 方向のみに移動 (Y/Z/roll/pitch/yaw はゼロ — 数式上)
//!   4. (3) のため体幹は常時 +X に等速で移動 (= 静止待機なし)
//!
//! 実行手順:
//!   - 事前に `cargo run --no-default-features --example go2_export_misa`
//!     で `models/unitree_go2/go2.misa` を生成しておく (脚リンク + Pose を込み)
//!   - `cargo xtask run --features mujoco --example go2_linear_crawl`

#[cfg(feature = "mujoco")]
fn main() {
    use articara::gait::{auto_detect_kinematics_config, GaitController};
    use articara::mjcf::{GroundPlaneCfg, MjcfExportOptions};
    use articara::mujoco_sim::MujocoSim;
    use articara::rbd::model::ActuatorMode;
    use articara::robot::*;
    use nalgebra as na;
    use quadruped_gait::{GaitConfig, GaitMode, LegId, VelocityCmd};

    let misa = std::path::Path::new("models/unitree_go2/go2.misa");
    let mut model = RobotModel::from_misa(misa)
        .unwrap_or_else(|e| panic!(".misa load failed ({}): {e}", misa.display()));
    println!(
        "Loaded {}: links={} joints={} (movable={})",
        misa.display(),
        model.links.len(),
        model.joints.len(),
        model.joints.iter().filter(|j| j.joint_type != "fixed").count(),
    );

    // ユーザ確認済の前進セットアップ: Position-PD で Kp=500。
    // (既定 actuator から強化することで足が地面を実際に蹴り返せる)
    for j in model.joints.iter_mut().filter(|j| j.joint_type != "fixed") {
        j.actuator_mode = ActuatorMode::Position;
        // Sweep-validated default (`examples/go2_linear_crawl_sweep.rs`)
        // for vx=0.05 m/s on Go2 — higher Kp tightens stance-leg PD and
        // reduces forward-velocity dropout during 3-support phases.
        j.actuator_kp = 1000.0;
        j.actuator_kv = 10.0;
    }

    // ホーム姿勢を seed (hip=0, thigh=0.9, calf=-1.8)。
    let home_q = [
        ("FL_hip_joint", 0.0), ("FL_thigh_joint", 0.9), ("FL_calf_joint", -1.8),
        ("FR_hip_joint", 0.0), ("FR_thigh_joint", 0.9), ("FR_calf_joint", -1.8),
        ("RL_hip_joint", 0.0), ("RL_thigh_joint", 0.9), ("RL_calf_joint", -1.8),
        ("RR_hip_joint", 0.0), ("RR_thigh_joint", 0.9), ("RR_calf_joint", -1.8),
    ];
    for (name, q) in home_q.iter() {
        let ji = *model.joint_map.get(*name)
            .unwrap_or_else(|| panic!("joint missing: {name}"));
        model.joint_positions[ji] = *q;
    }
    model.rebuild_misarta_model();

    let foot_links = [
        (LegId::FL, "FL_foot"), (LegId::FR, "FR_foot"),
        (LegId::RL, "RL_foot"), (LegId::RR, "RR_foot"),
    ];
    let kin = auto_detect_kinematics_config(&model, &foot_links)
        .unwrap_or_else(|errs| panic!("kin auto-detect: {errs:?}"));

    // ★ GUI と同じ articara::gait::GaitController::build 経路を使う。
    // GaitMode::LinearCrawl を渡すと内部で LinearCrawlGen が組まれる。
    let gait_cfg = GaitConfig::crawl()
        .with_cycle_period(1.0)
        .with_four_support_fraction(0.5)
        .with_swing_height(0.04);
    println!(
        "GaitMode::LinearCrawl  cycle={} s  α={}  swing_h={} m",
        gait_cfg.cycle_period_s, gait_cfg.four_support_fraction, gait_cfg.swing_height_m,
    );
    let mut gc = GaitController::build(&model, kin, gait_cfg, GaitMode::LinearCrawl)
        .expect("GaitController::build LinearCrawl");

    // Sim spawn — base は LinearCrawlGen が auto-detect した body_height に合わせる
    // (= 約 0.27 m, Go2 home pose の nominal_foot_body.z から逆算)。
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

    let trunk = |sim: &MujocoSim| -> (na::Vector3<f64>, na::UnitQuaternion<f64>) {
        let p = sim.body_world_position("base").expect("base xpos");
        let r = sim.body_world_orientation("base").expect("base xquat");
        (na::Vector3::new(p[0], p[1], p[2]), r)
    };

    // Phase 1: 0.5 s ホーム姿勢で着地待ち。
    println!("\n--- Phase 1: settle 0.5 s ---");
    let (p0, _) = trunk(&sim);
    println!("  t=0.000 trunk=({:+.3}, {:+.3}, {:+.3})", p0.x, p0.y, p0.z);
    for (name, q) in home_q.iter() {
        sim.set_position_target(model.joint_map[*name], *q);
    }
    sim.step_n_frames(&mut model, (0.5 / mj_dt) as u32, true);
    let (p1, r1) = trunk(&sim);
    let rpy1 = r1.euler_angles();
    println!(
        "  t=0.500 trunk=({:+.3}, {:+.3}, {:+.3})  rpy=({:+.3},{:+.3},{:+.3})",
        p1.x, p1.y, p1.z, rpy1.0, rpy1.1, rpy1.2,
    );

    // Phase 2: walk → stop → walk → stop の 4 区間で「停止中に足が上がらない」
    // ことを確認しつつ前進量も測る。
    let cmd_vx = 0.05_f64;
    let walk_s = 3.0_f64;
    let stop_s = 2.0_f64;
    println!(
        "\n--- Phase 2: walk({walk_s}s) → stop({stop_s}s) → walk({walk_s}s) → stop({stop_s}s) ---"
    );
    gc.enable();

    let gait_dt = 0.005_f64;
    let mut max_roll: f64 = 0.0;
    let mut max_pitch: f64 = 0.0;
    let mut max_yaw: f64 = 0.0;
    let mut min_z = p1.z;
    let mut max_z = p1.z;
    let mut max_abs_y = 0.0_f64;
    let mut t_clock = 0.5_f64; // continues from Phase 1's 0.5 s settle

    let mut run_segment = |label: &str, duration: f64, vx: f64,
        sim: &mut MujocoSim, gc: &mut GaitController,
        model: &mut articara::robot::RobotModel,
        t_clock: &mut f64, max_roll: &mut f64, max_pitch: &mut f64, max_yaw: &mut f64,
        min_z: &mut f64, max_z: &mut f64, max_abs_y: &mut f64,
    | {
        gc.set_velocity_cmd(VelocityCmd { vx, vy: 0.0, wz: 0.0 });
        let n = (duration / gait_dt) as usize;
        let mut foot_z_peak_during_segment = -1.0_f64;
        for tick in 0..n {
            let (_out, targets, _ff) = gc.tick(gait_dt);
            for (ji, q) in targets.iter() {
                sim.set_position_target(*ji, *q);
            }
            sim.step_n_frames(model, (gait_dt / mj_dt).max(1.0) as u32, true);
            let (p, r) = trunk(sim);
            let rpy = r.euler_angles();
            *max_roll = max_roll.max(rpy.0.abs());
            *max_pitch = max_pitch.max(rpy.1.abs());
            *max_yaw = max_yaw.max(rpy.2.abs());
            *min_z = min_z.min(p.z);
            *max_z = max_z.max(p.z);
            *max_abs_y = max_abs_y.max((p.y - p1.y).abs());

            // 追加メトリック: stop 中に最大の足 Z を見て、地面より上がっていないか
            // 確認する (target_body は IK 後にしか取れないので joint angle から
            // 簡易に推定する — calf joint angle が thigh joint からどれだけ
            // 曲げ戻ったかで足が上がっているかが分かる)。
            if vx == 0.0 {
                if let Some(p_foot) = sim.body_world_position("FL_calf") {
                    foot_z_peak_during_segment = foot_z_peak_during_segment.max(p_foot[2]);
                }
            }
            *t_clock += gait_dt;
            if tick == 0 || tick == n - 1 {
                println!(
                    "  [{label}] t={:.3} vx={vx:+.3} trunk=({:+.3},{:+.3},{:+.3}) rpy=({:+.3},{:+.3},{:+.3})",
                    *t_clock, p.x, p.y, p.z, rpy.0, rpy.1, rpy.2,
                );
            }
        }
        if vx == 0.0 && foot_z_peak_during_segment > 0.0 {
            println!(
                "    [stop check] FL_calf body max z = {foot_z_peak_during_segment:.4} m \
                 (= z of calf joint, not the foot; should stay close to nominal)",
            );
        }
    };

    run_segment("walk-1", walk_s, cmd_vx, &mut sim, &mut gc, &mut model,
        &mut t_clock, &mut max_roll, &mut max_pitch, &mut max_yaw,
        &mut min_z, &mut max_z, &mut max_abs_y);
    run_segment("stop-1", stop_s, 0.0, &mut sim, &mut gc, &mut model,
        &mut t_clock, &mut max_roll, &mut max_pitch, &mut max_yaw,
        &mut min_z, &mut max_z, &mut max_abs_y);
    run_segment("walk-2", walk_s, cmd_vx, &mut sim, &mut gc, &mut model,
        &mut t_clock, &mut max_roll, &mut max_pitch, &mut max_yaw,
        &mut min_z, &mut max_z, &mut max_abs_y);
    run_segment("stop-2", stop_s, 0.0, &mut sim, &mut gc, &mut model,
        &mut t_clock, &mut max_roll, &mut max_pitch, &mut max_yaw,
        &mut min_z, &mut max_z, &mut max_abs_y);

    let total_phase2 = walk_s * 2.0 + stop_s * 2.0;

    let (pf, _) = trunk(&sim);
    println!("\n=== Result ===");
    println!("  start trunk x={:+.3} y={:+.3} z={:+.3}", p1.x, p1.y, p1.z);
    println!("  end   trunk x={:+.3} y={:+.3} z={:+.3}", pf.x, pf.y, pf.z);
    println!(
        "  Δx = {:+.3} m  (effective walking time {:.1} s × vx={cmd_vx} m/s = {:.3} m expected)",
        pf.x - p1.x, walk_s * 2.0, cmd_vx * walk_s * 2.0,
    );
    println!("  Δy = {:+.3} m  (lateral drift)", pf.y - p1.y);
    println!(
        "  trunk z range = [{min_z:.3}, {max_z:.3}] m  Δz = {:.3} m  (vertical sway)",
        max_z - min_z,
    );
    println!("  trunk |y| peak = {max_abs_y:.3} m  (lateral sway)");
    println!("  max |roll|  = {max_roll:.3} rad");
    println!("  max |pitch| = {max_pitch:.3} rad");
    println!("  max |yaw|   = {max_yaw:.3} rad");
    let fell = min_z < 0.15;
    let walked = pf.x - p1.x > 0.05;
    println!(
        "  verdict: {}",
        if fell {
            "FELL"
        } else if walked {
            "WALKED"
        } else {
            "STOOD-BUT-DIDN'T-WALK"
        }
    );
}

#[cfg(not(feature = "mujoco"))]
fn main() {
    eprintln!("This example requires the `mujoco` feature. Run with:");
    eprintln!("  cargo xtask run --features mujoco --example go2_linear_crawl");
}
