//! kyo46rs plant sanity check: NO WBC at all.
//!
//! Hold the crouch pose with plain per-joint position PD and watch what
//! happens. The whole squat-controller effort is untunable until this
//! passes: kyo46rs_squat.rs's WBC assumes both feet are rigidly planted
//! and solves for contact forces on that basis, so if the bare plant
//! cannot even stand still under position control, every WBC number
//! downstream is measured against a robot that is already falling.
//!
//! Observed motivation: with position PD alone the robot holds z~0.45
//! for ~0.55 s, then collapses to z~0.08, and the base angular velocity
//! GROWS monotonically on the way down (wy: -0.25 -> +2.25 -> +5.38).
//! Growing oscillation, not a static topple -- and the CoM is comfortably
//! inside the support polygon (margin ~4 cm), so this is a dynamic
//! instability to be localised, not a bad stance.
//!
//! Prints the worst-offending joint each sample so the unstable DOF can
//! be identified rather than guessed at.
//!
//! Run with: `cargo run --features mujoco --example kyo46rs_stand_check`

#[cfg(feature = "mujoco")]
fn main() {
    use articara::mjcf::{GroundPlaneCfg, MjcfExportOptions};
    use articara::mujoco_sim::MujocoSim;
    use articara::rbd::model::ActuatorMode;
    use articara::robot::RobotModel;

    let env_f64 = |k: &str, d: f64| -> f64 {
        std::env::var(k).ok().and_then(|s| s.parse().ok()).unwrap_or(d)
    };
    let kp = env_f64("KP", 150.0);
    let kv = env_f64("KV", 8.0);
    let damping = env_f64("DAMPING", 0.15);
    let armature = env_f64("ARMATURE", 0.0005);
    let total_t = env_f64("T", 2.0);
    let hip_p = env_f64("HIP_PITCH", -0.35);
    let knee_q = env_f64("KNEE", 0.70);
    let ankle_p = env_f64("ANKLE_PITCH", -(0.70 - 0.35));
    // Substeps per controller call. The WBC runs at 3; the burn-in used
    // to run 25 in one go. Exposed so "is this a control-rate artifact?"
    // is answerable instead of arguable.
    let substeps = env_f64("SUBSTEPS", 1.0).max(1.0) as u32;

    let urdf_path = std::path::Path::new(
        "/home/takara/work/dp/humanoid/kyo46rs_description/urdf/kyo46rs.urdf",
    );
    let mut robot = RobotModel::from_urdf(urdf_path).expect("load kyo46rs.urdf");

    let crouch = [
        ("left_hip_pitch_joint", hip_p),
        ("left_knee_joint", knee_q),
        ("left_ankle_pitch_joint", ankle_p),
        ("right_hip_pitch_joint", hip_p),
        ("right_knee_joint", knee_q),
        ("right_ankle_pitch_joint", ankle_p),
    ];
    for (name, q) in crouch {
        if let Some(&ji) = robot.joint_map.get(name) {
            robot.joint_positions[ji] = q;
        }
    }
    robot.rebuild_misarta_model();

    // The URDF gives EVERY link a collision geom, and several are within
    // millimetres of each other (the torso box's bottom face and the
    // thigh box's top face are 4 mm apart). Non-adjacent pairs are not
    // excluded by MuJoCo's parent-child filter, so they can generate
    // spurious contact impulses. Only the soles should collide.
    let feet_only = std::env::var("FEET_ONLY").map(|v| v != "0").unwrap_or(false);
    if feet_only {
        for l in robot.links.iter_mut() {
            l.collision_enabled = l.name.ends_with("foot_link");
        }
    }

    for j in robot.joints.iter_mut() {
        j.actuator_mode = ActuatorMode::Position;
        j.actuator_kp = kp;
        j.actuator_kv = kv;
        j.joint_damping = damping;
        j.armature = armature;
    }

    const SOLE_BELOW_FOOT_ORIGIN: f64 = 0.059;
    const SOLE_CLEARANCE: f64 = 0.001;
    let dt_override = std::env::var("DT").ok().and_then(|v| v.parse::<f64>().ok());
    let make_opts = |z: f64| MjcfExportOptions {
        base_pos: Some([0.0, 0.0, z]),
        ground_plane: Some(GroundPlaneCfg { z: 0.0, half_size: 2.0, roll: 0.0, pitch: 0.0 }),
        timestep: dt_override,
        ..MjcfExportOptions::default()
    };
    let probe_z = 0.47;
    let spawn_z = {
        let probe = MujocoSim::new(&robot, make_opts(probe_z)).expect("probe sim");
        let foot_origin_z = probe.body_world_position("left_foot_link").expect("foot")[2];
        probe_z - ((foot_origin_z - SOLE_BELOW_FOOT_ORIGIN) - SOLE_CLEARANCE)
    };
    let mut sim = MujocoSim::new(&robot, make_opts(spawn_z)).expect("MujocoSim::new");
    let mj_dt = sim.timestep();
    println!(
        "kp={kp} kv={kv} damping={damping} armature={armature} substeps={substeps} \
         crouch=({hip_p:+.2},{knee_q:+.2},{ankle_p:+.2}) sum={:+.3} spawn_z={spawn_z:.4} dt={mj_dt}",
        hip_p + knee_q + ankle_p
    );

    // Set a target for EVERY joint, not just the crouch six. The roll
    // DOFs (hip_roll, ankle_roll) diverge monotonically from step 1 --
    // perfectly anti-symmetric left/right, before the soles even touch
    // -- which is what an unheld joint looks like.
    if std::env::var("ALL_TARGETS").map(|v| v != "0").unwrap_or(true) {
        for ji in 0..robot.joints.len() {
            sim.set_position_target(ji, robot.joint_positions[ji]);
        }
    }
    for (name, q) in crouch {
        if let Some(&ji) = robot.joint_map.get(name) {
            sim.set_position_target(ji, q);
        }
    }

    let names: Vec<String> = robot.joints.iter().map(|j| j.name.clone()).collect();

    // Step-by-step trace of the onset. The instability is already
    // visible 2 ms in (right_ankle_roll at -4 rad/s after ONE step),
    // which is far too early for any tipping dynamics -- so watch the
    // individual steps rather than 50 ms samples.
    if std::env::var("ONSET").is_ok() {
        for step in 0..12 {
            let z = sim.body_world_position(&robot.root_link).unwrap()[2];
            let lf = sim.body_world_position("left_foot_link").unwrap()[2];
            let mut worst: Vec<String> = Vec::new();
            for n in names.iter() {
                if let Some((q, qd)) = sim.joint_q_qd(n) {
                    if qd.abs() > 0.5 {
                        worst.push(format!("{n}(q={q:+.3},qd={qd:+.2})"));
                    }
                }
            }
            println!(
                "step {step:2} t={:.4} z={z:.5} sole={:.5} | {}",
                step as f64 * mj_dt,
                lf - SOLE_BELOW_FOOT_ORIGIN,
                if worst.is_empty() { "(all |qd|<0.5)".to_string() } else { worst.join(" ") }
            );
            sim.step_n_frames(&mut robot, 1, true);
        }
        return;
    }
    let n_steps = (total_t / (mj_dt * substeps as f64)) as u32;
    let sample_every = ((0.05 / (mj_dt * substeps as f64)) as u32).max(1);

    for step in 0..n_steps {
        sim.step_n_frames(&mut robot, substeps, true);
        if step % sample_every != 0 {
            continue;
        }
        let t = step as f64 * mj_dt * substeps as f64;
        let z = sim.body_world_position(&robot.root_link).unwrap()[2];
        let wv = sim.body_world_angular_velocity(&robot.root_link).unwrap();
        let (mut worst, mut worst_qd) = ("-", 0.0_f64);
        for (ji, n) in names.iter().enumerate() {
            let Some((_, qd)) = sim.joint_q_qd(n) else { continue };
            if qd.abs() > worst_qd.abs() {
                worst_qd = qd;
                worst = n;
                let _ = ji;
            }
        }
        let lfp = sim.body_world_position("left_foot_link").unwrap();
        let rfp = sim.body_world_position("right_foot_link").unwrap();
        let hr = |n: &str| sim.joint_q_qd(n).map(|(q, _)| q).unwrap_or(f64::NAN);
        println!(
            "t={t:5.2}  z={z:.4}  |w|={:.3}  worst={worst} qd={worst_qd:+.2}  foot_z=({:.4},{:.4})  hip_roll=({:+.3},{:+.3})  foot_y=({:+.4},{:+.4}) gap={:+.4}",
            (wv[0] * wv[0] + wv[1] * wv[1] + wv[2] * wv[2]).sqrt(),
            lfp[2], rfp[2],
            hr("left_hip_roll_joint"), hr("right_hip_roll_joint"),
            lfp[1], rfp[1], (lfp[1] - 0.019) - (rfp[1] + 0.019),
        );
        if z < 0.20 {
            println!("COLLAPSED at t={t:.2}");
            return;
        }
    }
    println!("SURVIVED {total_t}s standing");
}

#[cfg(not(feature = "mujoco"))]
fn main() {
    eprintln!("This example requires the `mujoco` feature. Run with:");
    eprintln!("  cargo run --features mujoco --example kyo46rs_stand_check");
}
