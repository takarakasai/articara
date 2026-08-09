//! Interactive, real-time keyboard teleop of the namiashi RL (PPO) stair-
//! climb policy -- the Rust-side counterpart to
//! `namiashi_staircase_5cm_teleop` (`tests/wbc_walk.rs`), so the SAME
//! keyboard muscle memory drives either controller on the SAME staircase,
//! in the SAME language/viewer, rather than one being a Rust `MjViewer`
//! window and the other a separate Python `mujoco.viewer` process.
//!
//! This is a direct, close-to-line-for-line Rust port of
//! `go2_rl/sim2sim_namiashi_mujoco.py` -- SAME constants (Isaac joint
//! order, default stance, DCMotor saturation/effort/velocity limits,
//! action scale, PD gains), SAME `dc_motor_clip` formula (an exact port of
//! `isaaclab.actuators.actuator_pd.DCMotor._clip_effort`, ported to Python
//! first and now here), SAME 45-d observation layout (ang_vel · grav ·
//! cmd · joint_pos_rel · joint_vel · last_action). Deliberately uses `ort`
//! (pyke's binding to the actual Microsoft onnxruntime C++ engine) rather
//! than `policy-runtime`'s `tract-onnx` (a different, pure-Rust ONNX
//! engine chosen there specifically for the real robot's aarch64
//! no-C-deps hardware-deploy constraint, which does not apply to a
//! desktop sim2sim/teleop tool) -- using the same engine as the Python
//! validation script means the Rust and Python paths share not just the
//! same weights but the same numerics, closing off a whole class of
//! engine-level discrepancy (the DCMotor-torque sim2sim bug earlier this
//! investigation is exactly the kind of subtle mismatch worth guarding
//! against on purpose).
//!
//! Limitation: only supports `history_length=1` policies (the base
//! 45-d-observation checkpoint, e.g. model_9393) -- no per-term history
//! stacking, unlike `sim2sim_namiashi_mujoco.py --history-length`.
//!
//! Run (needs a `libonnxruntime.so`/`.dylib`/`.dll` at runtime --
//! point `ORT_DYLIB_PATH` at the one already installed by go2_rl's
//! Python venv, e.g. `.../site-packages/onnxruntime/capi/libonnxruntime.so.1.28.0`):
//!   ORT_DYLIB_PATH=/path/to/libonnxruntime.so \
//!     cargo run --release --no-default-features --features "mujoco,mujoco-viewer,onnx" \
//!     --example namiashi_rl_teleop -- --onnx policy.onnx [--vx 0.8] [--vy 0] [--wz 0]
//!
//! Keys (identical to `namiashi_staircase_5cm_teleop` and
//! `sim2sim_namiashi_mujoco.py --interactive`):
//!   W/S or Up/Down          -- forward/back speed (vx)
//!   A/D or Left/Right       -- turn (wz)
//!   Q/E or PageUp/PageDown  -- strafe (vy)
//!   Space                   -- zero all three

#[cfg(all(feature = "mujoco", feature = "mujoco-viewer", feature = "onnx"))]
fn main() {
    use std::sync::{Arc, Mutex};

    use articara::mjcf::{MjcfExportOptions, StaircaseCfg};
    use articara::mujoco_sim::MujocoSim;
    use articara::rbd::model::ActuatorMode;
    use articara::robot::RobotModel;
    use nalgebra::Vector3;
    use ort::session::Session;
    use ort::value::TensorRef;

    // ── CLI ──────────────────────────────────────────────────────────────
    let args: Vec<String> = std::env::args().collect();
    let get = |key: &str| -> Option<String> {
        args.iter().position(|a| a == key).and_then(|i| args.get(i + 1).cloned())
    };
    let Some(onnx_path) = get("--onnx") else {
        eprintln!("usage: namiashi_rl_teleop --onnx P.onnx [--vx V] [--vy V] [--wz V]");
        std::process::exit(2);
    };
    let vx0: f64 = get("--vx").and_then(|v| v.parse().ok()).unwrap_or(0.8);
    let vy0: f64 = get("--vy").and_then(|v| v.parse().ok()).unwrap_or(0.0);
    let wz0: f64 = get("--wz").and_then(|v| v.parse().ok()).unwrap_or(0.0);

    // ── Constants (ported verbatim from sim2sim_namiashi_mujoco.py) ────────
    const ISAAC_NAMES: [&str; 12] = [
        "FL_hip_joint", "FR_hip_joint", "RL_hip_joint", "RR_hip_joint",
        "FL_thigh_joint", "FR_thigh_joint", "RL_thigh_joint", "RR_thigh_joint",
        "FL_calf_joint", "FR_calf_joint", "RL_calf_joint", "RR_calf_joint",
    ];
    const DEFAULT_ISAAC: [f64; 12] = [
        0.0, 0.0, 0.0, 0.0, 0.695, 0.695, 0.695, 0.695, -1.390, -1.390, -1.390, -1.390,
    ];
    // DCMotorCfg's THREE parameters, not a single flat torque clip -- see
    // sim2sim_namiashi_mujoco.py's own comment on SATURATION_EFFORT_ISAAC
    // for why the low-speed ceiling is EFFORT_LIMIT (rated), not
    // SATURATION_EFFORT (peak), and how big a sim2sim gap using the wrong
    // one opened up before that fix.
    const SATURATION_EFFORT: [f64; 12] = [2.5, 2.5, 2.5, 2.5, 2.5, 2.5, 2.5, 2.5, 3.88889, 3.88889, 3.88889, 3.88889];
    const EFFORT_LIMIT: [f64; 12] = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.5556, 1.5556, 1.5556, 1.5556];
    const VELOCITY_LIMIT: [f64; 12] =
        [33.5, 33.5, 33.5, 33.5, 33.5, 33.5, 33.5, 33.5, 21.5357, 21.5357, 21.5357, 21.5357];
    const ACTION_SCALE: f64 = 0.25;
    const KP: f64 = 25.0;
    const KD: f64 = 0.5;
    const ARM_DEFAULT: f64 = 0.0;
    const ARM_KP: f64 = 40.0;
    const ARM_KD: f64 = 2.0;
    const PHYSICS_DT: f64 = 0.005; // 200 Hz, matching the Python script's own recorded model.xml
    const INFER_HZ: f64 = 50.0;

    fn dc_motor_clip(effort: f64, joint_vel: f64, saturation_effort: f64, effort_limit: f64, velocity_limit: f64) -> f64 {
        let vel_at_effort_lim = velocity_limit * (1.0 + effort_limit / saturation_effort);
        let v = joint_vel.clamp(-vel_at_effort_lim, vel_at_effort_lim);
        let torque_speed_top = saturation_effort * (1.0 - v / velocity_limit);
        let torque_speed_bottom = saturation_effort * (-1.0 - v / velocity_limit);
        let max_effort = torque_speed_top.min(effort_limit);
        let min_effort = torque_speed_bottom.max(-effort_limit);
        effort.clamp(min_effort, max_effort)
    }

    // ── Robot + staircase (same fixture/geometry every WBC/MPC test in
    // tests/wbc_walk.rs uses) ───────────────────────────────────────────
    let misa = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("tests/fixtures/namiashi/namiashi_3p3_prop.misa");
    let mut robot = RobotModel::from_misa(&misa).unwrap_or_else(|e| panic!(".misa load failed ({}): {e}", misa.display()));
    let stairs = StaircaseCfg { rise_m: 0.05, run_m: 0.20, n_steps: 10, approach_m: 1.5, top_platform_m: 8.0, half_width_m: 6.0 };

    for (k, name) in ISAAC_NAMES.iter().enumerate() {
        let Some(&ji) = robot.joint_map.get(*name) else { panic!("joint missing: {name}") };
        robot.joints[ji].actuator_mode = ActuatorMode::Torque;
        robot.joint_positions[ji] = DEFAULT_ISAAC[k];
    }
    let arm_ji = robot.joint_map.get("arm_pitch_joint").copied();
    if let Some(ji) = arm_ji {
        robot.joints[ji].actuator_mode = ActuatorMode::Torque;
        robot.joint_positions[ji] = ARM_DEFAULT;
    }
    robot.rebuild_misarta_model();

    let opts = MjcfExportOptions {
        extra_worldbody_xml: Some(stairs.worldbody_xml()),
        add_actuators: true,
        timestep: Some(PHYSICS_DT),
        ..MjcfExportOptions::default()
    };
    let mut sim = MujocoSim::new(&robot, opts).expect("MujocoSim::new");
    sim.set_gravity_compensation(false); // Torque mode carries gravity itself -- see run_wbc_sim's Actuation::Torque comment
    let dt = sim.timestep();
    let decim = ((1.0 / INFER_HZ) / dt).round().max(1.0) as u32;
    println!("[namiashi_rl_teleop] dt={dt}s decim={decim} (-> {:.1} Hz inference)", 1.0 / (dt * decim as f64));

    // ── ONNX Runtime session (real onnxruntime via `ort`, not tract) ────
    let mut session = Session::builder()
        .unwrap_or_else(|e| panic!("Session::builder: {e}"))
        .commit_from_file(&onnx_path)
        .unwrap_or_else(|e| panic!("load {onnx_path}: {e}"));
    println!("[namiashi_rl_teleop] loaded {onnx_path}");

    // ── Settle: 0.5s of PD-hold at the default stance before engaging the
    // policy (mirrors sim2sim_namiashi_mujoco.py's own settle ramp). ────
    let settle_ticks = (0.5 / dt).round() as u32;
    for _ in 0..settle_ticks {
        for (k, name) in ISAAC_NAMES.iter().enumerate() {
            let ji = robot.joint_map[*name];
            let (q, dq) = sim.joint_q_qd(name).expect("leg joint state");
            let tau = dc_motor_clip(KP * (DEFAULT_ISAAC[k] - q) - KD * dq, dq, SATURATION_EFFORT[k], EFFORT_LIMIT[k], VELOCITY_LIMIT[k]);
            sim.set_torque_target(ji, tau);
        }
        if let Some(ji) = arm_ji {
            let (q, dq) = sim.joint_q_qd("arm_pitch_joint").expect("arm joint state");
            sim.set_torque_target(ji, (ARM_KP * (ARM_DEFAULT - q) - ARM_KD * dq).clamp(-6.865, 6.865));
        }
        sim.step(&mut robot, dt, true);
    }

    // ── Live teleop command + viewer, identical mechanism/keybindings to
    // namiashi_staircase_5cm_teleop (tests/wbc_walk.rs). ────────────────
    let live = Arc::new(Mutex::new([vx0, vy0, wz0]));
    let mut viewer = mujoco::viewer::MjViewer::launch_passive(sim.mj_model().clone(), 0).expect("launch MjViewer");
    {
        use mujoco::viewer::egui;
        const STEP: f64 = 0.02;
        const VX_MAX: f64 = 0.8;
        const VY_MAX: f64 = 0.3;
        const WZ_MAX: f64 = 1.0;
        let live = live.clone();
        viewer.add_ui_callback_detached(move |ctx| {
            let (up, down, left, right, strafe_l, strafe_r, reset) = ctx.input(|r| {
                (
                    r.key_down(egui::Key::W) || r.key_down(egui::Key::ArrowUp),
                    r.key_down(egui::Key::S) || r.key_down(egui::Key::ArrowDown),
                    r.key_down(egui::Key::A) || r.key_down(egui::Key::ArrowLeft),
                    r.key_down(egui::Key::D) || r.key_down(egui::Key::ArrowRight),
                    r.key_down(egui::Key::Q) || r.key_down(egui::Key::PageUp),
                    r.key_down(egui::Key::E) || r.key_down(egui::Key::PageDown),
                    r.key_pressed(egui::Key::Space),
                )
            });
            let mut cmd = live.lock().unwrap();
            if reset {
                *cmd = [0.0; 3];
                return;
            }
            if up { cmd[0] = (cmd[0] + STEP).min(VX_MAX); }
            if down { cmd[0] = (cmd[0] - STEP).max(-VX_MAX); }
            if left { cmd[2] = (cmd[2] + STEP).min(WZ_MAX); }
            if right { cmd[2] = (cmd[2] - STEP).max(-WZ_MAX); }
            if strafe_l { cmd[1] = (cmd[1] + STEP).min(VY_MAX); }
            if strafe_r { cmd[1] = (cmd[1] - STEP).max(-VY_MAX); }
        });
    }

    // ── Main loop: ONNX inference every `decim` physics ticks, held
    // between (matches sim2sim_namiashi_mujoco.py's own decimation). ───
    let mut last_action = [0.0f32; 12];
    let mut q_des_isaac = DEFAULT_ISAAC;
    let wall_start = std::time::Instant::now();
    let mut k: u64 = 0;
    loop {
        if k % decim as u64 == 0 {
            let ang_vel_w = sim.body_world_angular_velocity(&robot.root_link).expect("root omega");
            let q_wb = sim.body_world_orientation(&robot.root_link).expect("root quat");
            let ang_vel_b = q_wb.inverse_transform_vector(&Vector3::new(ang_vel_w[0], ang_vel_w[1], ang_vel_w[2]));
            let grav_b = q_wb.inverse_transform_vector(&Vector3::new(0.0, 0.0, -1.0));
            let cmd = *live.lock().unwrap();

            let mut obs = [0.0f32; 45];
            obs[0] = ang_vel_b.x as f32;
            obs[1] = ang_vel_b.y as f32;
            obs[2] = ang_vel_b.z as f32;
            obs[3] = grav_b.x as f32;
            obs[4] = grav_b.y as f32;
            obs[5] = grav_b.z as f32;
            obs[6] = cmd[0] as f32;
            obs[7] = cmd[1] as f32;
            obs[8] = cmd[2] as f32;
            for (i, name) in ISAAC_NAMES.iter().enumerate() {
                let (q, dq) = sim.joint_q_qd(name).expect("leg joint state");
                obs[9 + i] = (q - DEFAULT_ISAAC[i]) as f32;
                obs[21 + i] = dq as f32;
            }
            obs[33..45].copy_from_slice(&last_action);

            let input = ndarray::Array2::from_shape_vec((1, 45), obs.to_vec()).expect("obs shape");
            let outputs = session
                .run(ort::inputs![TensorRef::from_array_view(&input).expect("tensor view")])
                .unwrap_or_else(|e| panic!("inference: {e}"));
            let (_, action) = outputs[0].try_extract_tensor::<f32>().expect("extract action");
            last_action.copy_from_slice(action);
            for i in 0..12 {
                q_des_isaac[i] = DEFAULT_ISAAC[i] + ACTION_SCALE * action[i] as f64;
            }
        }

        for (i, name) in ISAAC_NAMES.iter().enumerate() {
            let ji = robot.joint_map[*name];
            let (q, dq) = sim.joint_q_qd(name).expect("leg joint state");
            let tau = dc_motor_clip(KP * (q_des_isaac[i] - q) - KD * dq, dq, SATURATION_EFFORT[i], EFFORT_LIMIT[i], VELOCITY_LIMIT[i]);
            sim.set_torque_target(ji, tau);
        }
        if let Some(ji) = arm_ji {
            let (q, dq) = sim.joint_q_qd("arm_pitch_joint").expect("arm joint state");
            sim.set_torque_target(ji, (ARM_KP * (ARM_DEFAULT - q) - ARM_KD * dq).clamp(-6.865, 6.865));
        }
        sim.step(&mut robot, dt, true);
        k += 1;

        // ~60 Hz render/sync cadence, independent of the finer physics dt.
        let render_decim = ((1.0 / 60.0) / dt).round().max(1.0) as u64;
        if k % render_decim == 0 {
            viewer.sync_data(sim.mj_data_mut());
            let _ = viewer.render();
            if !viewer.running() {
                break;
            }
        }
        let target = std::time::Duration::from_secs_f64(k as f64 * dt);
        let elapsed = wall_start.elapsed();
        if elapsed < target {
            std::thread::sleep(target - elapsed);
        }
    }
}

#[cfg(not(all(feature = "mujoco", feature = "mujoco-viewer", feature = "onnx")))]
fn main() {
    eprintln!(
        "this example needs: cargo run --release --no-default-features \
         --features mujoco,mujoco-viewer,onnx --example namiashi_rl_teleop -- --onnx P.onnx"
    );
    std::process::exit(2);
}
