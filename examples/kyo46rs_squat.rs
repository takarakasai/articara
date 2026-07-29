//! kyo46rs (biped) squat-without-falling WBC demo.
//!
//! `articara::wbc_pipeline::WbcPipeline` and its `quadruped_gait::wbc`
//! dependency are hardcoded for exactly 4 legs/contacts, so this bypasses
//! that layer entirely and drives the biped directly with misa-wbc's
//! generic task catalogue + misarta's dynamics — the same primitives
//! `WbcPipeline` itself is built from, just assembled for 2 feet instead
//! of 4. Mirrors the quadruped "static stand" WBC tests' pattern (EoM +
//! per-foot zero-motion + friction cone), with a sinusoidal squat-height
//! + level-attitude reference as the balance task on top.
//!
//! Run with: `cargo run --features mujoco --example kyo46rs_squat`

#[cfg(feature = "mujoco")]
fn main() {
    use articara::mjcf::{GroundPlaneCfg, MjcfExportOptions};
    use articara::mujoco_sim::MujocoSim;
    use articara::rbd::model::ActuatorMode;
    use articara::robot::RobotModel;
    use articara::wbc_pipeline::build_floating_base_model;
    use misa_wbc::{tasks, AsAffine, Dynamics, Formulation, SolveConfig, Solver};
    use nalgebra as na;
    use std::f64::consts::PI;

    // ── Load + seed a bent-knee crouch pose ────────────────────────────
    // kyo46rs's q=0 (all-straight legs) is a kinematic singularity for a
    // floating-base squat controller, same reason go2_sim.rs/wbc_walk.rs
    // never start their quadrupeds at q=0. ~20 deg symmetric crouch.
    let urdf_path = std::path::Path::new(
        "/home/takara/work/dp/humanoid/kyo46rs_description/urdf/kyo46rs.urdf",
    );
    let mut robot = RobotModel::from_urdf(urdf_path).expect("load kyo46rs.urdf");

    // ── Runtime-tunable knobs (sweep without recompiling) ──────────────
    let env_f64 = |k: &str, d: f64| -> f64 {
        std::env::var(k).ok().and_then(|s| s.parse().ok()).unwrap_or(d)
    };
    // Sign of the attitude command fed into qddot[0..2].
    //
    // misarta's FreeFlyer motion subspace is `S = I6` in the BODY frame
    // with spatial row order [angular(3); linear(3)] (joint.rs:126-133,
    // cross-checked against jacobian.rs's write_joint_column, which
    // writes w into rows 0-2 and v into rows 3-5). So qddot[1] IS the
    // body-frame pitch angular acceleration, and a textbook PD driving
    // pitch -> 0 needs NO outer negation: +1.0 is the theoretically
    // correct sign.
    //
    // ...and yet -1.0 is what actually works. Re-tested from scratch
    // against BOTH a working solver (P2 force regulariser) and a stable
    // plant (1 ms timestep), the two things that invalidated the earlier
    // "confirmed empirically twice" claim: +1.0 diverges immediately
    // (wy -0.6 -> -20 rad/s in 10 ticks, textbook positive feedback)
    // while -1.0 holds wy bounded around -1..-1.8. So the inversion is
    // real, not an artifact of the bugs fixed today -- misarta's
    // qddot[1] and the measured body-frame d(omega_y)/dt genuinely carry
    // opposite signs here. Which frame convention produces that is still
    // unexplained; keeping the knob so it stays testable.
    let att_sign = env_f64("ATT_SIGN", 1.0);
    let use_xy = std::env::var("XY").map(|v| v != "0").unwrap_or(false);
    let height_first = std::env::var("HEIGHT_FIRST").map(|v| v != "0").unwrap_or(false);
    let use_post = std::env::var("POST").map(|v| v != "0").unwrap_or(false);
    let burnin_s = env_f64("BURNIN_S", 1.2);
    let burnin_kp = env_f64("BURNIN_KP", 150.0);
    let burnin_kv = env_f64("BURNIN_KV", 2.0);

    // Crouch seed. hip_pitch + knee + ankle_pitch MUST sum to 0, or the
    // foot link is not parallel to the floor and only its toe (or heel)
    // edge touches -- a line contact, not the flat patch every contact
    // task here assumes. The previous -0.35/0.70/-0.45 summed to -0.10
    // rad (5.7 deg of toe-down tilt), which is why NO burn-in stiffness
    // could hold the pose: swept kp from 40 to 2000 and the robot
    // collapsed to z~0.08 within 1 s in every single case. Not a gain
    // problem -- the stance itself was unstandable.
    let hip_p = env_f64("HIP_PITCH", -0.35);
    let knee_q = env_f64("KNEE", 0.70);
    let ankle_p = env_f64("ANKLE_PITCH", -(0.70 - 0.35));
    println!(
        "crouch seed: hip={hip_p:+.3} knee={knee_q:+.3} ankle={ankle_p:+.3} (sum={:+.4}, must be ~0 for a flat foot)",
        hip_p + knee_q + ankle_p
    );
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
    // Full seed pose, for the posture task below (the crouch six plus
    // every joint left at 0: hip_yaw/hip_roll/ankle_roll and the arms).
    let q_seed: Vec<f64> = robot.joint_positions.clone();

    // ── Spawn MuJoCo (Position mode first, for the settle burn-in) ─────
    // Robstride EduLite05 actuator model: joint_damping/armature aren't
    // in the official datasheet (misa-actuator/crates/robstride-protocol/
    // ref/el05_manual_en.md has no damping-coefficient or rotor-inertia
    // spec at all), so these are engineering placeholders, tuned via a
    // dedicated sweep in kyo46rs_njoint_check.rs: joint_damping=0 caused
    // periodic misa-wbc P1-level QP NumericalFailure (3.9-7.4% of ticks
    // at 6-joint tracking) and resonance-like growing overshoot; a sweep
    // from 0.01-0.15 found solver failures drop to exactly 0.0% at
    // damping >= 0.11 and stay there, so 0.15 keeps a margin above that
    // threshold. armature=0.0005 kg*m^2 is a geometric estimate for the
    // EL05's rotor (~60g, 15mm radius disc) reflected through its 9:1
    // gearbox (I*N^2). Applying both here to see whether they also
    // resolve THIS file's own still-unresolved t>~0.4s attitude
    // oscillation, not just the isolated njoint_check rig.
    const EL05_JOINT_DAMPING: f64 = 0.15;
    const EL05_ARMATURE: f64 = 0.0005;
    for j in robot.joints.iter_mut() {
        j.actuator_mode = ActuatorMode::Position;
        j.actuator_kp = burnin_kp;
        j.actuator_kv = burnin_kv;
        j.joint_damping = EL05_JOINT_DAMPING;
        j.armature = EL05_ARMATURE;
    }
    // ── Spawn height: MEASURED, not hand-derived ───────────────────────
    // The sole must start a hair above the floor. Getting this wrong is
    // not a cosmetic detail -- it invalidates the entire WBC, which
    // assumes both feet are rigidly planted and can deliver an arbitrary
    // support wrench. With the feet actually in the air the QP solves
    // against contact forces that do not exist, and the base does
    // whatever gravity says instead (observed: commanded qddot[1]=+6.8,
    // MuJoCo delivered -418).
    //
    // Both previous hand-derived constants were wrong, in opposite
    // directions, because the URDF's foot collision box is offset from
    // its link origin: `<collision><origin xyz="0.010 0 -0.0295"/>
    // <geometry><box size="0.098 0.038 0.059"/></geometry></collision>`
    // puts the box BOTTOM 0.0295+0.0295 = 0.059 m below the link origin.
    // `0.41` alone buried the soles ~4.8 cm INTO the floor (MuJoCo
    // resolved that penetration by launching the robot: trunk z 0.41 ->
    // 0.63, lin_vel.z +1.7 m/s). The `0.41 + 0.059 + 0.002` fix then
    // overshot the other way: 0.41 had been chosen to put the link
    // ORIGIN at z~0.011, not the sole at 0, so adding the full 0.059
    // left the soles 13 mm in the AIR.
    //
    // Stop hand-deriving it. Spawn once at a nominal height, measure
    // where the sole actually lands via FK, and re-spawn with the exact
    // correction applied. Self-correcting against any future change to
    // the crouch angles or the foot geometry.
    const SOLE_BELOW_FOOT_ORIGIN: f64 = 0.059;
    const SOLE_CLEARANCE: f64 = 0.001;
    // 1 ms, not MuJoCo's default 2 ms. MujocoSim's per-joint PD is an
    // EXPLICIT velocity feedback, stable only while kv < 2*I/dt. The
    // distal roll joints carry I = ixx + armature ~ 1.3e-4 + 5e-4 =
    // 6.3e-4 kg*m^2, which at dt=2 ms caps kv at 0.63 -- below even the
    // kv=2 this file used, so hip_roll/ankle_roll diverged from the
    // FIRST step (perfectly anti-symmetric left/right, before the soles
    // had even touched) and the robot buzzed itself over in ~0.65 s no
    // matter what the WBC did. Verified in kyo46rs_stand_check.rs with
    // position control only and NO WBC at all: dt=2 ms collapses at
    // t=0.65 while 1 ms / 0.5 ms / 0.25 ms all stand the full 5 s on
    // otherwise identical parameters.
    let sim_dt = env_f64("SIM_DT", 0.001);
    let make_opts = |z: f64| MjcfExportOptions {
        base_pos: Some([0.0, 0.0, z]),
        ground_plane: Some(GroundPlaneCfg { z: 0.0, half_size: 2.0, roll: 0.0, pitch: 0.0 }),
        timestep: Some(sim_dt),
        ..MjcfExportOptions::default()
    };
    let probe_z = 0.47;
    let spawn_z = {
        let probe = MujocoSim::new(&robot, make_opts(probe_z)).expect("MujocoSim::new (probe)");
        let foot_origin_z = probe.body_world_position("left_foot_link").expect("left_foot_link")[2];
        let sole_z = foot_origin_z - SOLE_BELOW_FOOT_ORIGIN;
        let z = probe_z - (sole_z - SOLE_CLEARANCE);
        println!(
            "spawn probe: at base_z={probe_z:.4} the sole sits at {sole_z:+.4} -> spawning at base_z={z:.4}"
        );
        z
    };
    let mut sim = MujocoSim::new(&robot, make_opts(spawn_z)).expect("MujocoSim::new");

    // ── Static-stability check: is the CoM even over the feet? ─────────
    // A position-controlled robot cannot balance -- it only holds joint
    // angles. If the crouch pose puts the CoM outside the support
    // polygon it WILL topple no matter how stiff the joints are (swept
    // burn-in kp 40..2000: collapsed to z~0.08 every time), and no
    // amount of WBC gain tuning downstream can be judged until this is
    // right. Sole corners come from the URDF collision box: centre
    // (0.010, 0, -0.0295), half extents (0.049, 0.019, 0.0295).
    {
        let (mut m_tot, mut cx, mut cy) = (0.0_f64, 0.0_f64, 0.0_f64);
        for link in robot.links.iter() {
            let m = link.inertial.mass;
            if m <= 0.0 {
                continue;
            }
            let Some(p) = sim.body_world_position(&link.name) else { continue };
            let Some(rot) = sim.body_world_orientation(&link.name) else { continue };
            let o = link.inertial.origin.translation.vector;
            let c = na::Vector3::new(p[0], p[1], p[2])
                + rot.to_rotation_matrix() * na::Vector3::new(o.x as f64, o.y as f64, o.z as f64);
            m_tot += m;
            cx += m * c.x;
            cy += m * c.y;
        }
        cx /= m_tot;
        cy /= m_tot;
        let (mut lo_x, mut hi_x) = (f64::INFINITY, f64::NEG_INFINITY);
        for foot in ["left_foot_link", "right_foot_link"] {
            let p = sim.body_world_position(foot).expect("foot pos");
            let rot = sim.body_world_orientation(foot).expect("foot rot").to_rotation_matrix();
            for sx in [-1.0_f64, 1.0] {
                for sy in [-1.0_f64, 1.0] {
                    let corner = na::Vector3::new(p[0], p[1], p[2])
                        + rot * na::Vector3::new(0.010 + sx * 0.049, sy * 0.019, -0.059);
                    lo_x = lo_x.min(corner.x);
                    hi_x = hi_x.max(corner.x);
                }
            }
        }
        let margin = (cx - lo_x).min(hi_x - cx);
        println!(
            "static stability: mass={m_tot:.3} kg  CoM=({cx:+.4},{cy:+.4})  support x=[{lo_x:+.4},{hi_x:+.4}]  margin={margin:+.4} m  -> {}",
            if margin > 0.0 { "INSIDE (statically standable)" } else { "OUTSIDE (will topple)" }
        );
    }
    let mj_dt = sim.timestep();
    println!("MuJoCo timestep = {mj_dt} s");
    {
        let rpy0 = sim.body_world_orientation(&robot.root_link).unwrap().euler_angles();
        let z0 = sim.body_world_position(&robot.root_link).unwrap()[2];
        println!("  t=0 (pre-burn-in) trunk z={z0:.3} rpy=({:+.3},{:+.3},{:+.3})", rpy0.0, rpy0.1, rpy0.2);
        for n in [
            "torso", "head_link",
            "left_hip_yaw_link", "left_hip_roll_link", "left_thigh_link", "left_shank_link", "left_ankle_pitch_link", "left_foot_link",
            "right_hip_yaw_link", "right_hip_roll_link", "right_thigh_link", "right_shank_link", "right_ankle_pitch_link", "right_foot_link",
            "left_upper_arm_link", "left_forearm_link", "right_upper_arm_link", "right_forearm_link",
        ] {
            println!("    {n} = {:?}", sim.body_world_position(n));
        }
    }

    // ── Build the WBC's floating-base misarta model ────────────────────
    // `build_floating_base_model` is generic (BFS over robot.joints, no
    // leg-count assumption) — reused verbatim from wbc_pipeline.rs.
    let (model, a2m, link_to_idx) = build_floating_base_model(&robot);
    let nv = model.nv;
    let na_count = nv - 6; // 16 actuated DOF
    println!("misarta floating-base model: nv={nv} na={na_count}");

    let left_foot_mi = *link_to_idx
        .get("left_foot_link")
        .expect("left_foot_link not found in kinematic tree");
    let right_foot_mi = *link_to_idx
        .get("right_foot_link")
        .expect("right_foot_link not found in kinematic tree");
    let trunk_mi = 1usize; // FreeFlyer's own body, by build_floating_base_model's convention

    // Per-actuated-joint torque limit (misarta v-index-6 order), from the
    // URDF's own <limit effort=.../> (Robstride Edulite05: 6 or 12 N*m
    // for the dual-motor knee/hip — see kyo46rs_description/README.md).
    let mut torque_max = na::DVector::from_element(na_count, 6.0);
    for ji in 0..robot.joints.len() {
        let Some(mi) = a2m[ji] else { continue };
        if model.joints[mi].joint_type.nv() != 1 {
            continue;
        }
        let vi = model.v_idx[mi];
        if vi < 6 {
            continue;
        }
        torque_max[vi - 6] = robot.joints[ji].effort.max(1.0);
    }

    // ── Burn-in: hold the crouch pose at Position-PD, 0.5 s ────────────
    // Hold EVERY joint, not just the crouch six. hip_roll has no
    // position feedback once the WBC takes over (P2 regularises torque,
    // and with both feet planted the closed chain means the posture
    // task cannot move it either), so whatever it drifts to during the
    // burn-in is what the WBC is stuck with for the rest of the run.
    for ji in 0..robot.joints.len() {
        sim.set_position_target(ji, q_seed[ji]);
    }
    let burnin_chunks = (burnin_s / 0.05).round().max(1.0) as u32;
    for chunk in 0..burnin_chunks {
        sim.step_n_frames(&mut robot, (0.05 / mj_dt) as u32, true);
        let wv = sim.body_world_angular_velocity(&robot.root_link).unwrap();
        let lv = sim.body_world_linear_velocity(&robot.root_link).unwrap();
        let z = sim.body_world_position(&robot.root_link).unwrap()[2];
        let (lk_q, lk_qd) = sim.joint_q_qd("left_knee_joint").unwrap_or((f64::NAN, f64::NAN));
        let (lhp_q, _) = sim.joint_q_qd("left_hip_pitch_joint").unwrap_or((f64::NAN, f64::NAN));
        let lfoot = sim.body_world_position("left_foot_link").unwrap_or([f64::NAN; 3]);
        println!(
            "  burn-in t={:.2}s z={z:.4} wvel=({:+.4},{:+.4},{:+.4}) lvel=({:+.4},{:+.4},{:+.4}) knee_q={lk_q:+.3} knee_qd={lk_qd:+.3} hip_p_q={lhp_q:+.3} left_foot_z={:.4}",
            (chunk + 1) as f64 * 0.05, wv[0], wv[1], wv[2], lv[0], lv[1], lv[2], lfoot[2],
        );
    }

    // Switch to Torque mode for the WBC-driven remainder (set_wbc_torques
    // bypasses per-joint PD regardless, but this makes the intent explicit).
    for j in robot.joints.iter_mut() {
        j.actuator_mode = ActuatorMode::Torque;
    }

    let z_hi = sim.body_world_position(&robot.root_link).expect("torso xpos")[2];
    // Horizontal reference, captured once the burn-in has settled. See
    // the P1xy task for why this is needed at all.
    let xy_ref = {
        let p = sim.body_world_position(&robot.root_link).expect("torso xpos");
        [p[0], p[1]]
    };
    let z_lo = z_hi * 0.85; // ~15% squat depth
    let z_mid = 0.5 * (z_hi + z_lo);
    let amp = 0.0; // DIAGNOSTIC: freeze height, static standing only
    let post_burnin_rpy = sim
        .body_world_orientation(&robot.root_link)
        .unwrap()
        .euler_angles();
    let post_burnin_wvel = sim.body_world_angular_velocity(&robot.root_link).unwrap();
    let post_burnin_lvel = sim.body_world_linear_velocity(&robot.root_link).unwrap();
    println!(
        "Standing (crouch-seed) trunk z = {z_hi:.3} m; squat target z_lo = {z_lo:.3} m; post-burn-in rpy=({:+.3},{:+.3},{:+.3})",
        post_burnin_rpy.0, post_burnin_rpy.1, post_burnin_rpy.2
    );
    {
        let hr = |n: &str| sim.joint_q_qd(n).map(|(q, _)| q).unwrap_or(f64::NAN);
        let lp = sim.body_world_position("left_foot_link").unwrap();
        let rp = sim.body_world_position("right_foot_link").unwrap();
        println!(
            "  post-burn-in hip_roll=({:+.3},{:+.3}) ankle_roll=({:+.3},{:+.3})  foot y=({:+.4},{:+.4}) inner-gap={:+.4} (spawn gap 0.102)",
            hr("left_hip_roll_joint"), hr("right_hip_roll_joint"),
            hr("left_ankle_roll_joint"), hr("right_ankle_roll_joint"),
            lp[1], rp[1], (lp[1] - 0.019) - (rp[1] + 0.019),
        );
    }
    println!(
        "  post-burn-in world ang_vel=({:+.4},{:+.4},{:+.4}) lin_vel=({:+.4},{:+.4},{:+.4})",
        post_burnin_wvel[0], post_burnin_wvel[1], post_burnin_wvel[2],
        post_burnin_lvel[0], post_burnin_lvel[1], post_burnin_lvel[2],
    );
    for (name, target) in [
        ("left_foot", sim.body_world_position("left_foot_link")),
        ("right_foot", sim.body_world_position("right_foot_link")),
    ] {
        println!("  {name} pos = {target:?}");
    }

    // ── WBC squat loop ──────────────────────────────────────────────────
    let mut solver = Solver::new();
    let cfg = SolveConfig::default();
    const PERIOD_S: f64 = 3.0;
    const N_CYCLES: u32 = 3;
    const FRICTION_MU: f64 = 0.6;
    // Attitude/height gains. The old 500/150 (and 3000/300) were tuned
    // during the broken-solver era and command accelerations far beyond
    // what the contacts can physically deliver: with both feet at the
    // same x, pitch authority is only the CoP travel along the sole,
    // |My| <= Lx*fz ~ 0.049*65 = 3.2 N*m, i.e. ~12 rad/s^2 against the
    // robot's ~0.27 kg*m^2 pitch inertia -- yet KD_ATT=150 alone turned
    // the +-2 rad/s of contact chatter in omega_y into +-300 rad/s^2 of
    // command. Defaults now sit inside the achievable envelope.
    let kp_z = env_f64("KP_Z", 3000.0);
    let kd_z = env_f64("KD_Z", 300.0);
    let kp_att = env_f64("KP_ATT", 500.0);
    let kd_att = env_f64("KD_ATT", 250.0);
    const FALL_Z_M: f64 = 0.30;
    const FALL_TILT_RAD: f64 = 0.52; // ~30 deg

    // DT must be an exact multiple of mj_dt: `mj_substeps` used to be
    // truncated (0.005/0.002 = 2.5 -> 2), so the WBC's own idea of
    // elapsed time (assumed 0.005 s/tick) silently drifted ~20% ahead of
    // the physics clock (actually advancing 0.004 s/tick) -- a growing
    // phase error between the commanded squat trajectory and reality.
    // Round to the nearest whole number of physics steps and derive DT
    // FROM that, so the two clocks can never disagree.
    let mj_substeps = (0.005 / mj_dt).round().max(1.0) as u32;
    let dt = mj_substeps as f64 * mj_dt;
    println!("control dt = {dt:.4} s ({mj_substeps} physics substeps/tick)");

    let total_t = PERIOD_S * N_CYCLES as f64;
    let n_ticks = (total_t / dt) as usize;

    // Trajectory log for offline video rendering (Python + MuJoCo replay,
    // since MujocoSim itself is a headless physics bridge with no
    // rendering/screenshot capability). One row per tick: t, base xyz,
    // base quat (w,x,y,z), then all 16 actuated joint angles by name.
    let log_joint_order: Vec<&str> = vec![
        "left_hip_yaw_joint", "left_hip_roll_joint", "left_hip_pitch_joint",
        "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
        "right_hip_yaw_joint", "right_hip_roll_joint", "right_hip_pitch_joint",
        "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
        "left_shoulder_pitch_joint", "left_elbow_joint",
        "right_shoulder_pitch_joint", "right_elbow_joint",
    ];
    let log_path = "/tmp/claude-1000/-home-takara-work/3288d9bb-da13-4665-bfb2-9595dd62f7ab/scratchpad/kyo46rs_squat_traj.csv";
    let mut log_file = std::fs::File::create(log_path).expect("create trajectory log");
    {
        use std::io::Write;
        write!(log_file, "t,x,y,z,qw,qx,qy,qz").unwrap();
        for name in &log_joint_order {
            write!(log_file, ",{name}").unwrap();
        }
        writeln!(log_file).unwrap();
    }

    let mut min_z = z_hi;
    let mut max_tilt: f64 = 0.0;
    let mut fell = false;

    // Attitude PD alone shows zero steady-state rejection of whatever
    // small constant bias/asymmetry the double-support stance carries
    // (roll observed to creep steadily in ONE direction rather than
    // oscillate, even under much higher Kp/Kd -- the classic signature
    // of PD "droop" under a persistent disturbance, not underdamping).
    // Add a modest integral term (anti-windup clamped) to null that bias.
    let ki_att = env_f64("KI_ATT", 0.0);
    const I_ATT_CLAMP: f64 = 0.15; // rad, limits integral windup
    let mut roll_i: f64 = 0.0;
    let mut pitch_i: f64 = 0.0;
    // Plant-response probe: commanded qddot[1] vs the QP's own solved
    // qddot[1] vs the acceleration MuJoCo actually produced. Separates
    // "the task isn't achieved" from "the task is achieved but the frame
    // convention is flipped" -- the two candidate explanations for the
    // attitude sign paradox.
    let mut prev_wy: Option<f64> = None;

    for tick in 0..n_ticks {
        let t = tick as f64 * dt;

        // ---- sync q, v from MuJoCo -------------------------------------
        let body_pos = sim.body_world_position(&robot.root_link).expect("torso xpos");
        let body_quat = sim
            .body_world_orientation(&robot.root_link)
            .expect("torso xquat");
        let v_lin_world = sim
            .body_world_linear_velocity(&robot.root_link)
            .expect("torso lin vel");
        let v_ang_world = sim
            .body_world_angular_velocity(&robot.root_link)
            .expect("torso ang vel");
        let r_wb = body_quat.to_rotation_matrix();
        let r_bw = r_wb.transpose();
        let v_lin_body = r_bw * na::Vector3::new(v_lin_world[0], v_lin_world[1], v_lin_world[2]);
        let v_ang_body = r_bw * na::Vector3::new(v_ang_world[0], v_ang_world[1], v_ang_world[2]);

        let mut q = model.neutral_q();
        q[0] = body_pos[0];
        q[1] = body_pos[1];
        q[2] = body_pos[2];
        q[3] = body_quat.i;
        q[4] = body_quat.j;
        q[5] = body_quat.k;
        q[6] = body_quat.w;
        let mut v = vec![0.0_f64; nv];
        v[0] = v_ang_body.x;
        v[1] = v_ang_body.y;
        v[2] = v_ang_body.z;
        v[3] = v_lin_body.x;
        v[4] = v_lin_body.y;
        v[5] = v_lin_body.z;
        for ji in 0..robot.joints.len() {
            let Some(mi) = a2m[ji] else { continue };
            if model.joints[mi].joint_type.nq() == 1 {
                q[model.q_idx[mi]] = robot.joint_positions[ji];
            }
            if model.joints[mi].joint_type.nv() == 1 {
                if let Some((_, qd)) = sim.joint_q_qd(&robot.joints[ji].name) {
                    const JOINT_V_MAX: f64 = 5.0;
                    v[model.v_idx[mi]] = qd.clamp(-JOINT_V_MAX, JOINT_V_MAX);
                }
            }
        }

        // ---- dynamics ----------------------------------------------------
        let mass = misarta::crba::crba(&model, &q);
        let h = misarta::rnea::nonlinear_effects(&model, &q, &v);
        let v_dvec = na::DVector::from_column_slice(&v);

        // Per-foot FULL 6-row (angular+linear) Jacobians, stacked into a
        // 12xnv contact Jacobian (nc=2 x 6D wrench each) -- upgraded from
        // linear-only (3 rows/foot). A linear-only "zero_contact_
        // acceleration" treats each foot as a PIVOT (free to rotate about
        // its fixed contact point), which lets the whole robot slowly
        // tip over that pivot even while box/friction constraints hold --
        // exactly the failure mode observed (a slow, unrecovered pitch
        // drift). The full 6D version makes the foot a truly rigid flat
        // contact (no rotation either), and pairs with `patch_contact`
        // below (CoP-in-sole constraint) instead of `friction_pyramid`
        // (which only bounds tangential force, not tipping moment).
        let mut j_contact = na::DMatrix::zeros(12, nv);
        let mut dj_v = na::DVector::zeros(12);
        for (slot, foot_mi) in [left_foot_mi, right_foot_mi].into_iter().enumerate() {
            let j_full = misarta::jacobian::compute_joint_jacobian(&model, &q, foot_mi);
            let dj_full =
                misarta::jacobian::compute_joint_jacobian_time_derivative(&model, &q, &v, foot_mi);
            let dj_v_full = &dj_full * &v_dvec;
            for r in 0..6 {
                for c in 0..nv {
                    j_contact[(6 * slot + r, c)] = j_full[(r, c)];
                }
                dj_v[6 * slot + r] = dj_v_full[r];
            }
        }

        // Trunk (base) task "Jacobian": rows [ang_x(roll), ang_y(pitch),
        // lin_z(height)]. NOT computed via `compute_joint_jacobian` — for
        // the FreeFlyer's own body, qddot[0..6] (body-frame [ang;lin]) IS
        // by definition its own spatial acceleration (that's what the
        // FreeFlyer's generalized coordinates mean), so the "Jacobian" is
        // the trivial selection qddot[1], qddot[0], qddot[5] with zero
        // bias — going through the general recursive Jacobian for the
        // base's own index risks a frame-convention mismatch (world vs.
        // body) against the [angular;linear] body-frame convention `v`
        // was built in above; this sidesteps that entirely.
        let _ = trunk_mi; // no longer used; kept for documentation context
        // Separate Jacobians for attitude (priority 1: don't fall over)
        // and height (priority 2: track the squat). Combining both into
        // one 3-row least-squares task let them compete for the same
        // torque budget -- pushing height gains up visibly made attitude
        // worse and vice versa, since the QP balances a single combined
        // residual rather than satisfying attitude first.
        let mut j_att = na::DMatrix::zeros(2, nv);
        j_att[(0, 0)] = 1.0; // roll
        j_att[(1, 1)] = 1.0; // pitch
        let dj_v_att = na::DVector::zeros(2);
        let mut j_height = na::DMatrix::zeros(1, nv);
        j_height[(0, 5)] = 1.0; // height
        let dj_v_height = na::DVector::zeros(1);

        // ---- squat reference (sinusoid between z_hi and z_lo) -----------
        let phase = 2.0 * PI * t / PERIOD_S;
        let z_ref = z_hi + amp * phase.cos();
        let zd_ref = -amp * (2.0 * PI / PERIOD_S) * phase.sin();
        let zdd_ref = -amp * (2.0 * PI / PERIOD_S).powi(2) * phase.cos();

        let (roll_meas, pitch_meas, _yaw_meas) = body_quat.euler_angles();
        let z_meas = body_pos[2];
        let zd_meas = v_lin_body.z;

        let az_cmd = zdd_ref + kd_z * (zd_ref - zd_meas) + kp_z * (z_ref - z_meas);
        // See the `att_sign` definition at the top for why this is now a
        // knob defaulting to +1.0 (no negation) rather than a hardcoded
        // -1.0: qddot[0..2] is the body-frame angular acceleration, so a
        // textbook PD needs no flip, and the old "empirically confirmed"
        // flip was measured against a solver that was failing every tick.
        roll_i = (roll_i + roll_meas * dt).clamp(-I_ATT_CLAMP, I_ATT_CLAMP);
        pitch_i = (pitch_i + pitch_meas * dt).clamp(-I_ATT_CLAMP, I_ATT_CLAMP);
        let a_roll_cmd = att_sign * (kd_att * (0.0 - v_ang_body.x) + kp_att * (0.0 - roll_meas) + ki_att * (0.0 - roll_i));
        let a_pitch_cmd = att_sign * (kd_att * (0.0 - v_ang_body.y) + kp_att * (0.0 - pitch_meas) + ki_att * (0.0 - pitch_i));
        let att_ref = na::DVector::from_vec(vec![a_roll_cmd, a_pitch_cmd]);
        let height_ref = na::DVector::from_vec(vec![az_cmd]);

        // ---- task stack ---------------------------------------------------
        let dyn_ctx = Dynamics::new(Formulation::Explicit, &mass, &h, &j_contact, na_count);

        // Per-foot 6-D wrench sub-selectors out of the stacked 12-size
        // force/wrench block (Var has no sub-slice method; build the
        // selection matrix ourselves, same pattern misa-wbc's own
        // tasks.rs uses internally). Wrench row order [m(3); f(3)]
        // matches the Jacobian's own [angular(3); linear(3)] rows, per
        // `patch_contact`'s doc comment.
        let forces = dyn_ctx.forces();
        let mut sel_left = na::DMatrix::zeros(6, forces.size());
        let mut sel_right = na::DMatrix::zeros(6, forces.size());
        for k in 0..6 {
            sel_left[(k, k)] = 1.0;
            sel_right[(k, 6 + k)] = 1.0;
        }
        // patch_contact's CoP box (|mx| <= Ly*fz, |my| <= Lx*fz) is only
        // the physical centre-of-pressure condition if the wrench's
        // moment is taken about the SOLE, in the sole's own frame. What
        // `forces` actually carries is the wrench dual to
        // `compute_joint_jacobian(.., foot_mi)` -- i.e. about the foot
        // LINK ORIGIN, in the WORLD frame. The link origin sits 0.059 m
        // ABOVE the sole, so a tangential friction force fx contributes
        // 0.059*fx of moment there that has nothing to do with where the
        // pressure centre is. Applying the box at the origin therefore
        // admits contact wrenches that no real foot can produce, and the
        // QP happily "achieves" base accelerations MuJoCo then refuses to
        // deliver (measured: commanded qddot[1] = -250, actual ~0).
        //
        // Transform to the sole before constraining. The EoM keeps using
        // the consistent (J_origin, w_origin) pair -- only the CoP test
        // moves reference point:
        //     m_sole = R^T (m_origin - r_world x f),  f_sole = R^T f
        // with r = sole centre - link origin. From the URDF collision box
        // (origin (0.010,0,-0.0295), z size 0.059) the bottom-face centre
        // is at (0.010, 0, -0.059) in the foot link frame.
        const SOLE_OFFSET_LOCAL: [f64; 3] = [0.010, 0.0, -0.059];
        let mut to_sole = |foot_link: &str, sel: &na::DMatrix<f64>| -> na::DMatrix<f64> {
            let rot = sim
                .body_world_orientation(foot_link)
                .expect("foot orientation")
                .to_rotation_matrix();
            let r_w = rot
                * na::Vector3::new(
                    SOLE_OFFSET_LOCAL[0],
                    SOLE_OFFSET_LOCAL[1],
                    SOLE_OFFSET_LOCAL[2],
                );
            let rt = rot.transpose();
            let rt = rt.matrix();
            let skew = na::Matrix3::new(
                0.0, -r_w.z, r_w.y,
                r_w.z, 0.0, -r_w.x,
                -r_w.y, r_w.x, 0.0,
            );
            let top_right = -(rt * skew);
            let mut t = na::DMatrix::zeros(6, 6);
            for i in 0..3 {
                for j in 0..3 {
                    t[(i, j)] = rt[(i, j)];
                    t[(i, 3 + j)] = top_right[(i, j)];
                    t[(3 + i, 3 + j)] = rt[(i, j)];
                }
            }
            t * sel
        };
        let sel_left_sole = to_sole("left_foot_link", &sel_left);
        let sel_right_sole = to_sole("right_foot_link", &sel_right);
        let w_left = &sel_left_sole * &forces.as_affine();
        let w_right = &sel_right_sole * &forces.as_affine();

        let j_left = j_contact.rows(0, 6).into_owned();
        let j_right = j_contact.rows(6, 6).into_owned();
        let dj_v_left = dj_v.rows(0, 6).into_owned();
        let dj_v_right = dj_v.rows(6, 6).into_owned();

        // Sole footprint from kyo46rs_description/README.md: 0.098 m
        // (length, x) x 0.038 m (width, y). cop_half = (Lx, Ly) where Lx
        // constrains the pitch-tipping moment (front/back, along the
        // foot's length) and Ly the roll-tipping moment (side/side,
        // along its width) -- per `patch_contact`'s doc comment.
        let sole_patch = tasks::ContactPatch {
            mu: FRICTION_MU,
            cop_half: (0.049, 0.019),
            mu_torsion: 0.05,
            f_max: 150.0,
        };

        let p0 = dyn_ctx
            .dynamics_task()
            .expect("Explicit formulation always keeps the EoM task")
            + tasks::zero_contact_acceleration(dyn_ctx.qddot(), &j_left, &dj_v_left)
            + tasks::zero_contact_acceleration(dyn_ctx.qddot(), &j_right, &dj_v_right)
            + tasks::patch_contact(&w_left, &sole_patch)
            + tasks::patch_contact(&w_right, &sole_patch)
            + tasks::box_bound(dyn_ctx.tau(), &torque_max);

        let p1 = tasks::cartesian_acceleration(dyn_ctx.qddot(), &j_att, &dj_v_att, &att_ref);
        let p1h = tasks::cartesian_acceleration(dyn_ctx.qddot(), &j_height, &dj_v_height, &height_ref);

        // ---- horizontal balance: hold the base over the feet ------------
        // THE OTHER MISSING TASK. P1 regulates base roll/pitch, P1h
        // regulates height -- so the base's horizontal position (x, y)
        // sits entirely in the null space and nothing pulls it back.
        // A level, correct-height base is not a balanced one: the QP is
        // free to shift the whole robot sideways and to unload one foot
        // completely, because neither costs it anything.
        //
        // That is exactly what happened. From a clean settled start the
        // robot holds roll/pitch inside +-0.06 for 0.25 s, while the
        // contact probe shows the LEFT foot's ground points decaying
        // 4 -> 3 -> 2 -> 1 -> 0 and its load going to zero: the QP walks
        // all 65 N onto the right foot. Once it is on one foot, roll
        // snaps +0.03 -> +0.37 in 50 ms and it topples.
        //
        // Proper humanoid balance regulates the CoM (or ZMP / capture
        // point). This is the cheap stand-in: for a robot holding one
        // pose the base and the CoM move together, so pinning the base's
        // x/y is most of the benefit for two extra rows. Errors are
        // rotated into the body frame to match qddot[3..5]'s convention.
        let kp_xy = env_f64("KP_XY", 300.0);
        let kd_xy = env_f64("KD_XY", 60.0);
        let e_world = na::Vector3::new(xy_ref[0] - body_pos[0], xy_ref[1] - body_pos[1], 0.0);
        let e_body = r_bw * e_world;
        let mut j_xy = na::DMatrix::zeros(2, nv);
        j_xy[(0, 3)] = 1.0;
        j_xy[(1, 4)] = 1.0;
        let xy_accel_ref = na::DVector::from_vec(vec![
            kp_xy * e_body.x + kd_xy * (0.0 - v_lin_body.x),
            kp_xy * e_body.y + kd_xy * (0.0 - v_lin_body.y),
        ]);
        let p1xy = tasks::cartesian_acceleration(
            dyn_ctx.qddot(),
            &j_xy,
            &na::DVector::zeros(2),
            &xy_accel_ref,
        );

        // ---- posture task: hold the seed pose in the null space ---------
        // THE MISSING TASK. P0/P1/P1h constrain the contacts and the
        // base's roll/pitch/height -- nothing anywhere constrained the
        // internal joint POSITIONS. P2 regularises tau (toward gravity
        // comp), which is a torque objective, so hip_roll / hip_yaw /
        // ankle_roll / the arms had no position feedback at all and were
        // free to drift wherever the null space took them.
        //
        // They drifted into each other: the contact probe showed the LEFT
        // foot colliding with the RIGHT foot on nearly every tick
        // (`SELF-COLLISION: L~right_foot_link`). The feet start 10.2 cm
        // apart at the inner faces, and ~0.15 rad of unopposed hip_roll
        // swings each foot ~5.6 cm inboard -- enough to close that gap
        // exactly. Those foot-on-foot impulses, plus the resulting
        // bouncing (measured ground contact points flickering 0..3 per
        // foot, both feet at ZERO force on many ticks), are what broke
        // the rigid-contact assumption the whole QP is built on.
        //
        // Sits below attitude and height so it only claims the null
        // space, and covers all 16 actuated joints -- the crouch three
        // per leg are already driven from above, so for them this is
        // just a tie-breaker that keeps them near the seed.
        let kp_post = env_f64("KP_POST", 400.0);
        let kd_post = env_f64("KD_POST", 40.0);
        let mut j_post = na::DMatrix::zeros(na_count, nv);
        let mut post_ref = na::DVector::zeros(na_count);
        for ji in 0..robot.joints.len() {
            let Some(mi) = a2m[ji] else { continue };
            if model.joints[mi].joint_type.nv() != 1 {
                continue;
            }
            let vi = model.v_idx[mi];
            if vi < 6 {
                continue;
            }
            j_post[(vi - 6, vi)] = 1.0;
            post_ref[vi - 6] = kp_post * (q_seed[ji] - robot.joint_positions[ji])
                + kd_post * (0.0 - v[vi]);
        }
        let p_post = tasks::cartesian_acceleration(
            dyn_ctx.qddot(),
            &j_post,
            &na::DVector::zeros(na_count),
            &post_ref,
        );

        // Gravity-comp torque anchor (prevents the QP's "contacts alone
        // balance gravity, tau -> 0" degenerate solution — same rationale
        // wbc_pipeline.rs documents for the quadruped case) + a weak
        // minimum-acceleration regularizer as the lowest-priority
        // tie-breaker.
        let g_full = misarta::rnea::compute_gravity(&model, &q);
        let mut tau_gravity = na::DVector::zeros(na_count);
        for ji in 0..robot.joints.len() {
            let Some(mi) = a2m[ji] else { continue };
            if model.joints[mi].joint_type.nv() != 1 {
                continue;
            }
            let vi = model.v_idx[mi];
            if vi < 6 {
                continue;
            }
            tau_gravity[vi - 6] = g_full[vi];
        }
        // Dropped the qddot->0 regularizer that used to sit alongside this:
        // it competes with P1's own qddot objective for any residual null-
        // space freedom, which could only ever add noise to the attitude
        // correction, never help it (P1 already outranks P2).
        //
        // ROOT CAUSE of this file's `level: 0` NumericalFailure (found via
        // kyo46rs_hanging_contact_check.rs, a welded-torso minimal rig):
        // without a target, the QP is indifferent to HOW the required
        // support wrench splits between the left and right foot -- the
        // EoM's residual is satisfied equally well by any split summing to
        // the same net wrench. That leaves one foot's fz free to drift
        // toward 0, exactly where patch_contact's linearized friction-
        // cone/CoP-box rows all become simultaneously (near-)active and
        // the active-set QP degenerates. Regularizing each foot's fz
        // toward its share of body weight breaks that degeneracy (100% ->
        // 0% NumericalFailure in the isolated welded-torso test).
        const G: f64 = 9.81;
        let total_mass: f64 = robot.links.iter().map(|l| l.inertial.mass).sum();
        let mut forces_nominal = na::DVector::zeros(forces.size());
        forces_nominal[5] = total_mass * G / 2.0;
        forces_nominal[6 + 5] = total_mass * G / 2.0;
        let p2 = tasks::regularize(dyn_ctx.tau(), &tau_gravity)
            + tasks::regularize(&dyn_ctx.forces(), &forces_nominal);

        // Ablation switches. With both feet planted, 12 of the 22 DOF
        // are already spoken for, so every extra task row competes for
        // the same 10 -- worth being able to turn each one off.
        // Attitude-above-height is not obviously right: with high
        // attitude gains the QP holds tilt well but lets z collapse,
        // and vice versa, so the two are competing for the same limited
        // contact authority. Make the order testable.
        let mut levels = if height_first {
            vec![p0, p1h, p1]
        } else {
            vec![p0, p1, p1h]
        };
        if use_xy {
            levels.push(p1xy);
        }
        if use_post {
            levels.push(p_post);
        }
        levels.push(p2);
        let sol = solver
                        .solve(&levels, &cfg)
            .unwrap_or_else(|e| panic!("wbc solve failed at t={t:.3}: {e}"));
        let extracted = dyn_ctx.extract(&sol.x);

        // ---- map tau back to robot.joints order, drive MuJoCo -----------
        let mut robot_taus = vec![0.0_f64; robot.joints.len()];
        for ji in 0..robot.joints.len() {
            let Some(mi) = a2m[ji] else { continue };
            if model.joints[mi].joint_type.nv() != 1 {
                continue;
            }
            let vi = model.v_idx[mi];
            if vi < 6 {
                continue;
            }
            robot_taus[ji] = extracted.tau[vi - 6];
        }
        // Foot link origins BEFORE the step -- the same reference point
        // the contact Jacobian (and hence the QP's wrench) uses, so the
        // measured wrench below is directly comparable to `forces`.
        let foot_p0 = [
            sim.body_world_position("left_foot_link").unwrap(),
            sim.body_world_position("right_foot_link").unwrap(),
        ];

        sim.set_wbc_torques(&robot_taus);
        sim.step_n_frames(&mut robot, mj_substeps, true);

        // ---- QP's ASSUMED contact wrench vs MuJoCo's ACTUAL ------------
        // The base is unactuated: its 6 EoM rows are driven purely by
        // Jc^T·f, so the WBC's predicted base acceleration is only as
        // good as its assumed f. Both the pitch and height probes show
        // the QP hitting its commanded qddot exactly while MuJoCo
        // delivers ~0, which points here. Rebuild MuJoCo's per-foot
        // wrench about the SAME reference point (foot link origin,
        // world frame, [m(3); f(3)] order) and compare term by term.
        if std::env::var("FPROBE").is_ok() && tick < 30 {
            let mut meas = [[0.0_f64; 6]; 2];
            let mut npts = [0usize; 2];
            let mut other: Vec<String> = Vec::new();
            for c in sim.contacts() {
                let idx = if c.body1 == "left_foot_link" || c.body2 == "left_foot_link" {
                    0
                } else if c.body1 == "right_foot_link" || c.body2 == "right_foot_link" {
                    1
                } else {
                    continue;
                };
                // Only ground contacts: the world body reports as "".
                // Without this a foot-vs-link self-collision would be
                // folded in as if it were ground reaction.
                let partner = if c.body1.ends_with("foot_link") { &c.body2 } else { &c.body1 };
                if !partner.is_empty() {
                    other.push(format!("{}~{}", if idx == 0 { "L" } else { "R" }, partner));
                    continue;
                }
                npts[idx] += 1;
                // MuJoCo reports the force on geom2's body; flip when the
                // foot is geom1 so `f` is always the force ON THE FOOT.
                let s = if c.body2.ends_with("foot_link") { 1.0 } else { -1.0 };
                let f = [
                    s * c.force_world[0],
                    s * c.force_world[1],
                    s * c.force_world[2],
                ];
                let o = foot_p0[idx];
                let r = [c.pos[0] - o[0], c.pos[1] - o[1], c.pos[2] - o[2]];
                let m = [
                    r[1] * f[2] - r[2] * f[1],
                    r[2] * f[0] - r[0] * f[2],
                    r[0] * f[1] - r[1] * f[0],
                ];
                for k in 0..3 {
                    meas[idx][k] += m[k];
                    meas[idx][3 + k] += f[k];
                }
            }
            let qp_fz = extracted.forces[5] + extracted.forces[11];
            let mj_fz = meas[0][5] + meas[1][5];
            println!(
                "    [f] pts=({},{})  fz  qp=({:+6.1},{:+6.1})={:+6.1}  mj=({:+6.1},{:+6.1})={:+6.1}  weight={:.1}  my qp=({:+.3},{:+.3}) mj=({:+.3},{:+.3}){}",
                npts[0], npts[1],
                extracted.forces[5], extracted.forces[11], qp_fz,
                meas[0][5], meas[1][5], mj_fz,
                total_mass * G,
                extracted.forces[1], extracted.forces[7],
                meas[0][1], meas[1][1],
                if other.is_empty() { String::new() } else { format!("  SELF-COLLISION: {}", other.join(",")) },
            );
            let lp = sim.body_world_position("left_foot_link").unwrap();
            let rp = sim.body_world_position("right_foot_link").unwrap();
            let hr = |n: &str| sim.joint_q_qd(n).map(|(q, _)| q).unwrap_or(f64::NAN);
            println!(
                "         foot y: L={:+.4} R={:+.4} inner-gap={:+.4} (touch at <=0)  hip_roll=({:+.3},{:+.3}) ankle_roll=({:+.3},{:+.3})",
                lp[1], rp[1],
                (lp[1] - 0.019) - (rp[1] + 0.019),
                hr("left_hip_roll_joint"), hr("right_hip_roll_joint"),
                hr("left_ankle_roll_joint"), hr("right_ankle_roll_joint"),
            );
        }

        // ---- plant-response probe (see prev_wy) --------------------------
        if tick < 40 && std::env::var("PROBE").is_ok() {
            let wv_after = sim.body_world_angular_velocity(&robot.root_link).unwrap();
            let q_after = sim.body_world_orientation(&robot.root_link).unwrap();
            let wy_after = (q_after.to_rotation_matrix().transpose()
                * na::Vector3::new(wv_after[0], wv_after[1], wv_after[2]))
            .y;
            let measured_acc = (wy_after - v_ang_body.y) / dt;
            let vz_world_after = sim.body_world_linear_velocity(&robot.root_link).unwrap();
            let vz_after = (q_after.to_rotation_matrix().transpose()
                * na::Vector3::new(vz_world_after[0], vz_world_after[1], vz_world_after[2]))
            .z;
            let measured_az = (vz_after - v_lin_body.z) / dt;
            println!(
                "    [probe] pitch: cmd={:+8.1} qp={:+8.1} meas={:+8.1} | height: cmd={:+8.1} qp={:+8.1} meas={:+8.1}  z={:.4}",
                a_pitch_cmd, extracted.qddot[1], measured_acc,
                az_cmd, extracted.qddot[5], measured_az, body_pos[2],
            );
            prev_wy = Some(wy_after);
        }
        let _ = &prev_wy;

        // ---- log this tick's full pose for offline video rendering -------
        {
            use std::io::Write;
            let cur_pos = sim.body_world_position(&robot.root_link).unwrap();
            let cur_quat = sim.body_world_orientation(&robot.root_link).unwrap();
            write!(
                log_file,
                "{t:.4},{:.5},{:.5},{:.5},{:.5},{:.5},{:.5},{:.5}",
                cur_pos[0], cur_pos[1], cur_pos[2], cur_quat.w, cur_quat.i, cur_quat.j, cur_quat.k
            )
            .unwrap();
            for name in &log_joint_order {
                let angle = robot
                    .joint_map
                    .get(*name)
                    .map(|&ji| robot.joint_positions[ji])
                    .unwrap_or(0.0);
                write!(log_file, ",{angle:.5}").unwrap();
            }
            writeln!(log_file).unwrap();
        }

        // ---- track fall / tilt --------------------------------------------
        let cur_z = sim.body_world_position(&robot.root_link).unwrap()[2];
        min_z = min_z.min(cur_z);
        let tilt = roll_meas.abs().max(pitch_meas.abs());
        max_tilt = max_tilt.max(tilt);
        if cur_z < FALL_Z_M || tilt > FALL_TILT_RAD {
            fell = true;
        }

        if tick % 10 == 0 {
            let lhp = robot.joint_map.get("left_hip_pitch_joint").map(|&i| robot_taus[i]).unwrap_or(0.0);
            let lk = robot.joint_map.get("left_knee_joint").map(|&i| robot_taus[i]).unwrap_or(0.0);
            let lap = robot.joint_map.get("left_ankle_pitch_joint").map(|&i| robot_taus[i]).unwrap_or(0.0);
            println!(
                "  t={t:6.3}  z={cur_z:+.3} (ref {z_ref:+.3})  roll={roll_meas:+.3} pitch={pitch_meas:+.3} wy={:+.3}  a_pitch_cmd={a_pitch_cmd:+.2} status={:?}  tau[hip_p,knee,ank_p]=({lhp:+.2},{lk:+.2},{lap:+.2})",
                v_ang_body.y, sol.status,
            );
        }
        if fell {
            println!("  FELL at t={t:.3} (z={cur_z:.3}, tilt={tilt:.3} rad)");
            break;
        }
    }

    println!("\n=== Result ===");
    println!("  z_hi (standing) = {z_hi:.3} m, z_lo (target crouch) = {z_lo:.3} m");
    println!("  min z reached   = {min_z:.3} m");
    println!("  max |roll|/|pitch| = {max_tilt:.3} rad");
    println!("  verdict: {}", if fell { "FELL" } else { "SQUATTED" });
}

#[cfg(not(feature = "mujoco"))]
fn main() {
    eprintln!("This example requires the `mujoco` feature. Run with:");
    eprintln!("  cargo run --features mujoco --example kyo46rs_squat");
}
