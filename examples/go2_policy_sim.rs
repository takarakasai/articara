//! Sim2sim: run an exported RL policy (ONNX) as the controller inside articara's
//! MuJoCo sim of the Go2, reproducing go2-gait-runner's `policy` deployment
//! contract. Validates a policy in a second physics engine (MuJoCo) before
//! hardware. Different engine than the Isaac/PhysX it was trained in.
//!
//! Run: `cargo run --features "mujoco onnx" --example go2_policy_sim -- \
//!        --vx 0.0 --secs 5`
//! Flags: --model <onnx> --mjcf <go2.xml> --vx/--vy/--wz <f> --secs <f> --hold

#[cfg(all(feature = "mujoco", feature = "onnx"))]
fn main() {
    use articara::gait::auto_detect_kinematics_config; // (only to mirror go2_sim setup path)
    use articara::mjcf::{self, GroundPlaneCfg, MjcfExportOptions};
    use articara::mujoco_sim::MujocoSim;
    use articara::policy::{
        build_obs, OnnxPolicy, ACTION_SCALE, DEFAULT_ISAAC, ISAAC_JOINT_NAMES, POLICY_KD, POLICY_KP,
    };
    use articara::rbd::model::ActuatorMode;
    use articara::robot::*;
    use nalgebra as na;
    use quadruped_gait::LegId;

    // ── args ────────────────────────────────────────────────────────────────
    let mut model_path =
        "/home/takara/work/dp/go2-gait-runner/models/policies/crawl_deploy.onnx".to_string();
    let mut mjcf_path =
        "/home/takara/work/dp/go2-gait-runner/models/unitree_go2/go2.xml".to_string();
    let (mut vx, mut vy, mut wz, mut secs) = (0.0f64, 0.0f64, 0.0f64, 5.0f64);
    let mut hold = false;
    let mut video: Option<String> = None;
    let mut it = std::env::args().skip(1);
    while let Some(a) = it.next() {
        match a.as_str() {
            "--model" => model_path = it.next().unwrap_or(model_path),
            "--mjcf" => mjcf_path = it.next().unwrap_or(mjcf_path),
            "--vx" => vx = it.next().and_then(|s| s.parse().ok()).unwrap_or(vx),
            "--vy" => vy = it.next().and_then(|s| s.parse().ok()).unwrap_or(vy),
            "--wz" => wz = it.next().and_then(|s| s.parse().ok()).unwrap_or(wz),
            "--secs" => secs = it.next().and_then(|s| s.parse().ok()).unwrap_or(secs),
            "--hold" => hold = true,
            "--video" => video = it.next(),
            _ => {}
        }
    }

    // ── model + sim (mirrors examples/go2_sim.rs) ────────────────────────────
    let mut model = mjcf::import_mjcf(std::path::Path::new(&mjcf_path)).expect("Load Go2 MJCF");
    let foot_offset = na::Vector3::new(-0.002_f32, 0.0, -0.213);
    for (leg, parent) in [
        ("FL_foot", "FL_calf"), ("FR_foot", "FR_calf"),
        ("RL_foot", "RL_calf"), ("RR_foot", "RR_calf"),
    ] {
        let origin = na::Isometry3::from_parts(
            na::Translation3::from(foot_offset), na::UnitQuaternion::identity());
        model.add_child(parent, leg, &format!("{leg}_fixed"), "fixed", origin,
            na::Vector3::z(), GeomData::Sphere { radius: 0.022 }, [0.5, 0.5, 0.5, 1.0], 0.0, 0.0)
            .unwrap();
    }
    // Position mode with the policy's trained PD gains (kp=25, kv=0.5) — the
    // on-board PD the action is fed to on the real robot.
    for j in model.joints.iter_mut().filter(|j| j.joint_type != "fixed") {
        j.actuator_mode = ActuatorMode::Position;
        j.actuator_kp = POLICY_KP;
        j.actuator_kv = POLICY_KD;
    }
    // Start at the MuJoCo home keyframe (hip=0, thigh=0.9, calf=-1.8).
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
    let foot_links = [
        (LegId::FL, "FL_foot"), (LegId::FR, "FR_foot"),
        (LegId::RL, "RL_foot"), (LegId::RR, "RR_foot"),
    ];
    let _ = auto_detect_kinematics_config(&model, &foot_links); // parity with go2_sim setup

    let opts = MjcfExportOptions {
        base_pos: Some([0.0, 0.0, 0.30]),
        ground_plane: Some(GroundPlaneCfg { z: 0.0, half_size: 5.0, roll: 0.0, pitch: 0.0 }),
        ..MjcfExportOptions::default()
    };
    let mut sim = MujocoSim::new(&model, opts).expect("MujocoSim::new");
    let mj_dt = sim.timestep();
    let decim = (0.02 / mj_dt).round().max(1.0) as u32; // 50 Hz policy
    println!("mj_dt={mj_dt}s decim={decim}  cmd=[{vx},{vy},{wz}]  hold={hold}");

    // Precompute joint indices (Isaac order) so the step loop never borrows `model`.
    let jidx: [usize; 12] = core::array::from_fn(|i| model.joint_map[ISAAC_JOINT_NAMES[i]]);

    // ── Phase 1: settle to the policy DEFAULT pose (0.5 s, no policy) ─────────
    for i in 0..12 {
        sim.set_position_target(jidx[i], DEFAULT_ISAAC[i]);
    }
    sim.step_n_frames(&mut model, (0.5 / mj_dt) as u32, true);

    // optional offscreen recording (one PNG per 50 Hz tick → mp4 via ffmpeg)
    #[cfg(feature = "record")]
    let frame_dir = "/tmp/go2_policy_frames";
    #[cfg(feature = "record")]
    let mut recorder = if video.is_some() {
        let _ = std::fs::remove_dir_all(frame_dir);
        Some(articara::record::Recorder::new(&sim, 640, 480, "base", 1.6, 90.0, -15.0, frame_dir)
            .expect("recorder init"))
    } else {
        None
    };
    #[cfg(not(feature = "record"))]
    if video.is_some() {
        eprintln!("note: --video needs the `record` feature (cargo run --features \"mujoco onnx record\" ...)");
    }

    // ── Phase 2: policy loop at 50 Hz ────────────────────────────────────────
    let policy = OnnxPolicy::load(&model_path).expect("load policy");
    let mut last_action = [0.0f32; 12];
    let n_ticks = (secs / 0.02) as usize;
    let mut min_z = f64::INFINITY;
    let mut max_tilt = 0.0f64;
    let p0 = sim.body_world_position("base").unwrap();
    for _ in 0..n_ticks {
        let quat = sim.body_world_orientation("base").unwrap(); // UnitQuaternion (w,x,y,z)
        let av_w = sim.body_world_angular_velocity("base").unwrap();
        let av_b = quat.inverse_transform_vector(&na::Vector3::new(av_w[0], av_w[1], av_w[2]));
        let g_b = quat.inverse_transform_vector(&na::Vector3::new(0.0, 0.0, -1.0));
        let mut q = [0.0f64; 12];
        let mut qd = [0.0f64; 12];
        for (i, n) in ISAAC_JOINT_NAMES.iter().enumerate() {
            let (qi, qdi) = sim.joint_q_qd(n).unwrap();
            q[i] = qi;
            qd[i] = qdi;
        }
        if hold {
            // calibration: hold default, ignore the policy (PD-to-default baseline)
            for i in 0..12 {
                sim.set_position_target(jidx[i], DEFAULT_ISAAC[i]);
            }
        } else {
            let obs = build_obs([av_b.x, av_b.y, av_b.z], [g_b.x, g_b.y, g_b.z],
                [vx, vy, wz], &q, &qd, &last_action);
            let a = policy.infer(&obs).expect("infer");
            last_action = a;
            for i in 0..12 {
                sim.set_position_target(jidx[i], DEFAULT_ISAAC[i] + ACTION_SCALE * a[i] as f64);
            }
        }
        sim.step_n_frames(&mut model, decim, true);
        #[cfg(feature = "record")]
        if let Some(r) = recorder.as_mut() {
            r.capture(&mut sim).expect("capture frame");
        }
        let p = sim.body_world_position("base").unwrap();
        let rpy = sim.body_world_orientation("base").unwrap().euler_angles();
        min_z = min_z.min(p[2]);
        max_tilt = max_tilt.max(rpy.0.abs().max(rpy.1.abs()));
    }

    #[cfg(feature = "record")]
    if let (Some(out), Some(r)) = (&video, &recorder) {
        let (dir, n) = (r.dir().to_string_lossy().to_string(), r.frame_count());
        let st = std::process::Command::new("ffmpeg")
            .args(["-y", "-framerate", "50", "-i", &format!("{dir}/%05d.png"),
                   "-pix_fmt", "yuv420p", out])
            .status();
        println!("  recorded {n} frames -> {out}  (ffmpeg ok={:?})", st.map(|s| s.success()));
    }
    let pf = sim.body_world_position("base").unwrap();
    let fell = min_z < 0.15;
    println!("=== sim2sim result ===");
    println!("  Δx={:+.3}m Δy={:+.3}m  final z={:.3}  min z={:.3}  max tilt={:.1}deg",
        pf[0] - p0[0], pf[1] - p0[1], pf[2], min_z, max_tilt.to_degrees());
    println!("  verdict: {}", if fell { "FELL" } else if pf[0] - p0[0] > 0.10 { "WALKED" } else { "STOOD" });
}

#[cfg(not(all(feature = "mujoco", feature = "onnx")))]
fn main() {
    eprintln!("requires both features. Run with:");
    eprintln!("  cargo run --features \"mujoco onnx\" --example go2_policy_sim");
}
