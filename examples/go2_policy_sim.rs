//! Sim-deploy for the Go2 RL policy (ONNX): run the **identical**
//! `policy-runtime` I/O code that `go2-gait-runner` drives on the real robot,
//! but against articara's MuJoCo harness — the safe validation layer between
//! `policy --selftest` and hardware.
//!
//! What this proves before the robot moves:
//!   - observation assembly against a *physically consistent* sensor stream
//!     (projected gravity, body-frame gyro, joint reorder) — sign / order /
//!     scale mistakes show up as an immediate fall here, not on hardware;
//!   - the action path (`q_des = default + 0.5·action`, trained PD gains
//!     kp=25 / kd=0.5, 50 Hz inference) closes the loop and keeps the robot
//!     upright / tracking the command;
//!   - the Isaac → MuJoCo sim-to-sim gap (friction, actuator model) before
//!     paying the sim-to-real one.
//!
//! Run (model exported from Isaac Lab):
//!   cargo xtask run --features mujoco,policy-sim --example go2_policy_sim -- \
//!     --model policy.onnx [--vx 0.3] [--vy 0] [--wz 0] [--duration 8] \
//!     [--settle 0.5] [--csv sim.csv]
//!
//! The CSV has the same schema as the hardware runner's `--csv`, so the same
//! offline analysis applies to both.

#[cfg(all(feature = "mujoco", feature = "policy-sim"))]
fn main() {
    use articara::mjcf::{GroundPlaneCfg, MjcfExportOptions};
    use articara::mujoco_sim::MujocoSim;
    use articara::rbd::model::ActuatorMode;
    use articara::robot::RobotModel;
    use nalgebra as na;
    use policy_runtime::go2::*;
    use policy_runtime::{
        action_to_q_des_go2, build_obs, clamp_cmd, csv_header, obs_anomalies, write_csv_row,
        ObsInput, OnnxPolicy, N_OBS, POLICY_HZ,
    };

    // ── CLI ──────────────────────────────────────────────────────────────────
    let args: Vec<String> = std::env::args().collect();
    let get = |key: &str| -> Option<String> {
        args.iter()
            .position(|a| a == key)
            .and_then(|i| args.get(i + 1).cloned())
    };
    let Some(model_path) = get("--model") else {
        eprintln!(
            "usage: go2_policy_sim --model P.onnx [--vx V] [--vy V] [--wz V] \
             [--duration S] [--settle S] [--csv PATH]"
        );
        std::process::exit(2);
    };
    let cmd = clamp_cmd([
        get("--vx").and_then(|v| v.parse().ok()).unwrap_or(0.0),
        get("--vy").and_then(|v| v.parse().ok()).unwrap_or(0.0),
        get("--wz").and_then(|v| v.parse().ok()).unwrap_or(0.0),
    ]);
    let duration: f64 = get("--duration").and_then(|v| v.parse().ok()).unwrap_or(8.0);
    let settle: f64 = get("--settle").and_then(|v| v.parse().ok()).unwrap_or(0.5);
    let csv_path = get("--csv");

    // ── Model + policy ───────────────────────────────────────────────────────
    let misa = std::path::Path::new("models/unitree_go2/go2.misa");
    let mut model = RobotModel::from_misa(misa)
        .unwrap_or_else(|e| panic!(".misa load failed ({}): {e}", misa.display()));
    let policy = OnnxPolicy::load(&model_path).unwrap_or_else(|e| panic!("policy: {e}"));
    println!(
        "policy-sim: {} on {} (obs={N_OBS}, {POLICY_HZ} Hz, kp={POLICY_KP}, kd={POLICY_KD})",
        model_path,
        misa.display()
    );

    // Go2 motor order (FR,FL,RR,RL × hip/thigh/calf) → articara joint names.
    const GO2_MOTOR_JOINTS: [&str; 12] = [
        "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
        "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
        "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
        "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
    ];
    let joint_idx: Vec<usize> = GO2_MOTOR_JOINTS
        .iter()
        .map(|n| *model.joint_map.get(*n).unwrap_or_else(|| panic!("joint missing: {n}")))
        .collect();

    // The policy's nominal pose, in Go2 motor order.
    let mut default_go2 = [0.0f64; 12];
    for i in 0..12 {
        default_go2[ISAAC_TO_GO2[i]] = DEFAULT_ISAAC[i];
    }

    // Trained actuator gains — the whole point is to reproduce the deploy PD.
    for j in model.joints.iter_mut().filter(|j| j.joint_type != "fixed") {
        j.actuator_mode = ActuatorMode::Position;
        j.actuator_kp = POLICY_KP as f64;
        j.actuator_kv = POLICY_KD as f64;
    }
    for (g, ji) in joint_idx.iter().enumerate() {
        model.joint_positions[*ji] = default_go2[g];
    }
    model.rebuild_misarta_model();

    // ── Sim ──────────────────────────────────────────────────────────────────
    let opts = MjcfExportOptions {
        base_pos: Some([0.0, 0.0, 0.32]),
        ground_plane: Some(GroundPlaneCfg { z: 0.0, half_size: 5.0, roll: 0.0, pitch: 0.0 }),
        ..MjcfExportOptions::default()
    };
    let mut sim = MujocoSim::new(&model, opts).expect("MujocoSim::new");
    let mj_dt = sim.timestep();
    let infer_dt = 1.0 / POLICY_HZ;
    let frames_per_infer = (infer_dt / mj_dt).round().max(1.0) as u32;
    println!("policy-sim: mj_dt={mj_dt} s, {frames_per_infer} frames per inference");

    // Settle at the nominal pose so the PD has the feet planted.
    for (g, ji) in joint_idx.iter().enumerate() {
        sim.set_position_target(*ji, default_go2[g]);
    }
    sim.step_n_frames(&mut model, (settle / mj_dt) as u32, true);

    // Sensor adapter: MuJoCo → the runtime's Go2-convention snapshot.
    // (Angular velocity comes back in the world frame; the IMU gyro is body
    // frame, so rotate it back with the base orientation.)
    let obs_input = |sim: &MujocoSim, model: &RobotModel| -> ObsInput {
        let q_wb = sim.body_world_orientation("base").expect("base quat");
        let w_world = sim.body_world_angular_velocity("base").expect("base omega");
        let w_body = q_wb.inverse_transform_vector(&na::Vector3::new(
            w_world[0], w_world[1], w_world[2],
        ));
        let mut inp = ObsInput {
            gyro_rad_s: [w_body.x as f32, w_body.y as f32, w_body.z as f32],
            quat_wxyz: [
                q_wb.w as f32,
                q_wb.i as f32,
                q_wb.j as f32,
                q_wb.k as f32,
            ],
            ..Default::default()
        };
        for (g, name) in GO2_MOTOR_JOINTS.iter().enumerate() {
            let (q, dq) = sim.joint_q_qd(name).expect("joint state");
            inp.joint_q_go2[g] = q as f32;
            inp.joint_dq_go2[g] = dq as f32;
            let _ = model; // joint state comes straight from the sim
        }
        inp
    };

    // ── Policy loop (identical runtime path as the hardware runner) ─────────
    let mut csv = csv_path.as_ref().map(|p| {
        let mut f = std::fs::File::create(p).unwrap_or_else(|e| panic!("csv {p}: {e}"));
        use std::io::Write as _;
        writeln!(f, "{}", csv_header()).expect("csv header");
        println!("policy-sim: logging to {p}");
        f
    });
    let mut last_action = [0.0f64; 12];
    let mut anomaly_counts: std::collections::BTreeMap<&'static str, u64> = Default::default();
    let mut infer_us: Vec<u128> = Vec::new();
    let (p_start, _) = {
        let p = sim.body_world_position("base").expect("base pos");
        (na::Vector3::new(p[0], p[1], p[2]), ())
    };
    let mut max_roll = 0.0f64;
    let mut max_pitch = 0.0f64;
    let mut fell = false;

    let n_ticks = (duration / infer_dt).round() as usize;
    for tick in 0..n_ticks {
        let inp = obs_input(&sim, &model);
        let obs = build_obs(&inp, &cmd, &last_action);
        let anomalies = obs_anomalies(&obs);
        for a in &anomalies {
            *anomaly_counts.entry(a).or_insert(0) += 1;
        }
        let t0 = std::time::Instant::now();
        let action = policy.infer(&obs).unwrap_or_else(|e| panic!("infer: {e}"));
        let us = t0.elapsed().as_micros();
        infer_us.push(us);
        last_action = action;
        let q_des = action_to_q_des_go2(&action);
        for (g, ji) in joint_idx.iter().enumerate() {
            sim.set_position_target(*ji, q_des[g]);
        }
        if let Some(f) = csv.as_mut() {
            write_csv_row(f, tick as f64 * infer_dt, "sim", us, &obs, &action, &q_des, &anomalies)
                .expect("csv row");
        }
        sim.step_n_frames(&mut model, frames_per_infer, true);

        let r = sim.body_world_orientation("base").expect("base quat");
        let (roll, pitch, _yaw) = r.euler_angles();
        max_roll = max_roll.max(roll.abs());
        max_pitch = max_pitch.max(pitch.abs());
        let p = sim.body_world_position("base").expect("base pos");
        if p[2] < 0.12 || roll.abs() > 0.9 || pitch.abs() > 0.9 {
            println!(
                "policy-sim: FELL at t={:.2} s (z={:.3}, roll={:+.2}, pitch={:+.2})",
                tick as f64 * infer_dt,
                p[2],
                roll,
                pitch
            );
            fell = true;
            break;
        }
        if tick % (POLICY_HZ as usize) == 0 {
            println!(
                "  t={:5.2}s trunk=({:+.3},{:+.3},{:+.3}) rpy=({:+.2},{:+.2}) infer={us}us",
                tick as f64 * infer_dt,
                p[0],
                p[1],
                p[2],
                roll,
                pitch
            );
        }
    }

    // ── Summary ──────────────────────────────────────────────────────────────
    let p_end = sim.body_world_position("base").expect("base pos");
    let dx = p_end[0] - p_start.x;
    let dy = p_end[1] - p_start.y;
    let ran_s = infer_us.len() as f64 * infer_dt;
    println!("\n--- policy-sim summary ---");
    println!(
        "  cmd=[{:+.2},{:+.2},{:+.2}]  ran {:.2}s  moved dx={dx:+.3}m dy={dy:+.3}m \
         (mean vx={:+.3} m/s)",
        cmd[0],
        cmd[1],
        cmd[2],
        ran_s,
        dx / ran_s.max(1e-9)
    );
    println!("  max|roll|={:.2} rad  max|pitch|={:.2} rad  fell={fell}", max_roll, max_pitch);
    if !infer_us.is_empty() {
        let mut v = infer_us.clone();
        v.sort_unstable();
        println!(
            "  inference: mean {:.0}us p99 {}us max {}us over {} ticks",
            v.iter().sum::<u128>() as f64 / v.len() as f64,
            v[v.len() * 99 / 100 - (v.len() >= 100) as usize],
            v[v.len() - 1],
            v.len()
        );
    }
    if anomaly_counts.is_empty() {
        println!("  obs plausibility screen: no anomalies.");
    } else {
        println!("  OBS ANOMALIES:");
        for (k, n) in &anomaly_counts {
            println!("    {k}: {n} ticks");
        }
    }
    if fell {
        std::process::exit(1);
    }
}

#[cfg(not(all(feature = "mujoco", feature = "policy-sim")))]
fn main() {
    eprintln!(
        "this example needs: cargo xtask run --features mujoco,policy-sim \
         --example go2_policy_sim -- --model P.onnx"
    );
    std::process::exit(2);
}
