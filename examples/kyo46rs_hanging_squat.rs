//! kyo46rs squat-motion demo with the torso WELDED to the world (hung in
//! mid-air, `base_locked_axes: [true; 6]`) -- no ground, no feet contact,
//! no floating-base attitude/height problem at all. Drives the six leg
//! joints (hip_pitch/knee/ankle_pitch x2) through a sinusoidal crouch
//! motion using misa-wbc's actual hierarchical-QP task stack (not just
//! open-loop gravity comp), so this is a clean test of "can the WBC track
//! a joint-space squat trajectory" in isolation from every contact/
//! attitude complication `kyo46rs_squat.rs` has been fighting.
//!
//! Task stack:
//! - P0 (hard): equation-of-motion (Explicit formulation, ZERO contact
//!   rows -- nothing touches the ground) + torque box bound.
//! - P1: joint-space PD+feedforward tracking of the sinusoidal
//!   hip_pitch/knee/ankle_pitch targets (both legs), via a trivial
//!   qddot-index selector matrix -- the same pattern kyo46rs_squat.rs
//!   uses for its height/attitude tasks, just aimed at leg joint angles
//!   instead of base pose.
//! - P2: gravity-compensation regularizer (anchors idle joints -- arms,
//!   hip_yaw/roll, ankle_roll -- so they don't drift; see
//!   kyo46rs_fullbody_gravity_check.rs for why this alone holds well
//!   when the required torque isn't near-zero).
//!
//! Run with: `cargo run --features mujoco --example kyo46rs_hanging_squat`

#[cfg(feature = "mujoco")]
fn main() {
    use articara::mjcf::MjcfExportOptions;
    use articara::mujoco_sim::MujocoSim;
    use articara::rbd::model::ActuatorMode;
    use articara::robot::RobotModel;
    use articara::wbc_pipeline::build_floating_base_model;
    use misa_wbc::{tasks, Dynamics, Formulation, SolveConfig, Solver};
    use nalgebra as na;
    use std::f64::consts::PI;

    let urdf_path = std::path::Path::new(
        "/home/takara/work/dp/humanoid/kyo46rs_description/urdf/kyo46rs.urdf",
    );
    let mut robot = RobotModel::from_urdf(urdf_path).expect("load kyo46rs.urdf");

    // Mid-crouch seed (matches the pose already validated to hold well
    // under pure gravity comp in kyo46rs_fullbody_gravity_check.rs).
    let leg_joints = [
        ("left_hip_pitch_joint", -0.35, 1.0),
        ("left_knee_joint", 0.70, -1.0),
        ("left_ankle_pitch_joint", -0.45, 1.0),
        ("right_hip_pitch_joint", -0.35, 1.0),
        ("right_knee_joint", 0.70, -1.0),
        ("right_ankle_pitch_joint", -0.45, 1.0),
    ];
    // Also bend the arms to the non-trivial pose confirmed to hold
    // (kyo46rs_arm_gravity_check.rs) so the gravity-comp regularizer has
    // real work to do (not sitting at the degenerate q=0 hang) and isn't
    // itself a source of drift while we're only trying to test leg
    // tracking here.
    let arm_pose = [
        ("left_shoulder_pitch_joint", -1.0),
        ("left_elbow_joint", 1.2),
        ("right_shoulder_pitch_joint", -1.0),
        ("right_elbow_joint", 1.2),
    ];
    for (name, q, _sign) in leg_joints {
        if let Some(&ji) = robot.joint_map.get(name) {
            robot.joint_positions[ji] = q;
        }
    }
    for (name, q) in arm_pose {
        if let Some(&ji) = robot.joint_map.get(name) {
            robot.joint_positions[ji] = q;
        }
    }
    robot.rebuild_misarta_model();

    for j in robot.joints.iter_mut() {
        j.actuator_mode = ActuatorMode::Torque;
    }

    // Weld the torso mid-air -- no ground plane needed at all, nothing
    // ever touches anything.
    let opts = MjcfExportOptions {
        base_pos: Some([0.0, 0.0, 0.9]),
        base_locked_axes: [true; 6],
        ..MjcfExportOptions::default()
    };
    let mut sim = MujocoSim::new(&robot, opts).expect("MujocoSim::new");
    let mj_dt = sim.timestep();
    println!("MuJoCo timestep = {mj_dt} s");

    let (model, a2m, _link_to_idx) = build_floating_base_model(&robot);
    let nv = model.nv;
    let na_count = nv - 6;
    println!("misarta floating-base model: nv={nv} na={na_count} (base welded, never actuated)");

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

    // Per-leg-joint v-index (for the joint-space task selector) and the
    // seed angle each oscillates around.
    let leg_v_idx: Vec<(usize, f64, f64)> = leg_joints
        .iter()
        .map(|(name, seed, sign)| {
            let ji = *robot.joint_map.get(*name).expect("leg joint in URDF");
            let mi = a2m[ji].expect("leg joint mapped into misarta model");
            (model.v_idx[mi], *seed, *sign)
        })
        .collect();

    let mut solver = Solver::new();
    let cfg = SolveConfig::default();
    const PERIOD_S: f64 = 2.0;
    const N_CYCLES: u32 = 3;
    const SQUAT_AMP: f64 = 0.35; // rad, how far knee swings around its seed
    const KP_JOINT: f64 = 150.0;
    const KD_JOINT: f64 = 30.0;

    let mj_substeps = (0.005 / mj_dt).round().max(1.0) as u32;
    let dt = mj_substeps as f64 * mj_dt;
    println!("control dt = {dt:.4} s ({mj_substeps} physics substeps/tick)");
    let total_t = PERIOD_S * N_CYCLES as f64;
    let n_ticks = (total_t / dt) as usize;

    let log_path = "/tmp/claude-1000/-home-takara-work/3288d9bb-da13-4665-bfb2-9595dd62f7ab/scratchpad/kyo46rs_hanging_squat_traj.csv";
    let mut log_file = std::fs::File::create(log_path).expect("create trajectory log");
    let log_joint_order: Vec<&str> = vec![
        "left_hip_yaw_joint", "left_hip_roll_joint", "left_hip_pitch_joint",
        "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
        "right_hip_yaw_joint", "right_hip_roll_joint", "right_hip_pitch_joint",
        "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
        "left_shoulder_pitch_joint", "left_elbow_joint",
        "right_shoulder_pitch_joint", "right_elbow_joint",
    ];
    {
        use std::io::Write;
        write!(log_file, "t,x,y,z,qw,qx,qy,qz").unwrap();
        for name in &log_joint_order {
            write!(log_file, ",{name}").unwrap();
        }
        writeln!(log_file).unwrap();
    }

    let mut max_track_err: f64 = 0.0;

    for tick in 0..n_ticks {
        let t = tick as f64 * dt;

        // Base never moves (welded) -- q[0..7]/v[0..6] are the fixed
        // identity pose, no need to read them from MuJoCo every tick,
        // but do it anyway for symmetry with kyo46rs_squat.rs's pattern.
        let body_pos = sim.body_world_position(&robot.root_link).expect("torso xpos");
        let body_quat = sim.body_world_orientation(&robot.root_link).expect("torso xquat");
        let mut q = model.neutral_q();
        q[0] = body_pos[0];
        q[1] = body_pos[1];
        q[2] = body_pos[2];
        q[3] = body_quat.i;
        q[4] = body_quat.j;
        q[5] = body_quat.k;
        q[6] = body_quat.w;
        let mut v = vec![0.0_f64; nv];
        for ji in 0..robot.joints.len() {
            let Some(mi) = a2m[ji] else { continue };
            if model.joints[mi].joint_type.nq() == 1 {
                q[model.q_idx[mi]] = robot.joint_positions[ji];
            }
            if model.joints[mi].joint_type.nv() == 1 {
                if let Some((_, qd)) = sim.joint_q_qd(&robot.joints[ji].name) {
                    const JOINT_V_MAX: f64 = 10.0;
                    v[model.v_idx[mi]] = qd.clamp(-JOINT_V_MAX, JOINT_V_MAX);
                }
            }
        }

        let mass = misarta::crba::crba(&model, &q);
        let h = misarta::rnea::nonlinear_effects(&model, &q, &v);

        // The base is WELDED (an ideal mount, not a free-floating body):
        // it can supply whatever reaction wrench is needed, unlike a
        // real free-floating base whose EoM rows would need to net to
        // zero external force. Model that mount as a virtual 6-DOF
        // "contact" (J = identity on the base's own 6 rows) so the
        // solver has free reaction-force variables there instead of
        // forcing the base's 6 EoM rows to net to zero -- the Explicit
        // formulation's EoM task otherwise silently assumes an
        // unsupported free-floating base, which is wrong here and was
        // producing wildly incorrect qddot solutions.
        let mut j_contact = na::DMatrix::zeros(6, nv);
        for i in 0..6 {
            j_contact[(i, i)] = 1.0;
        }

        // Joint-space squat tracking: each leg joint oscillates around
        // its seed angle with a shared phase (all three joints per leg
        // move together, `sign` lets ankle/hip and knee bend opposite
        // ways so the crouch stays "natural" -- not that it matters
        // physically with the torso welded, just keeps the motion
        // looking like an actual squat in the rendered video).
        let phase = 2.0 * PI * t / PERIOD_S;
        let mut j_squat = na::DMatrix::zeros(leg_v_idx.len(), nv);
        let dj_v_squat = na::DVector::zeros(leg_v_idx.len());
        let mut accel_ref = na::DVector::zeros(leg_v_idx.len());
        for (row, (vidx, _seed, _sign)) in leg_v_idx.iter().enumerate() {
            j_squat[(row, *vidx)] = 1.0;
        }
        for (row, (_vidx, seed, sign)) in leg_v_idx.iter().enumerate() {
            let name = leg_joints[row].0;
            let ji = *robot.joint_map.get(name).unwrap();
            let q_meas = robot.joint_positions[ji];
            let q_ref = seed + sign * SQUAT_AMP * phase.cos();
            let qd_ref = -sign * SQUAT_AMP * (2.0 * PI / PERIOD_S) * phase.sin();
            let qdd_ref = -sign * SQUAT_AMP * (2.0 * PI / PERIOD_S).powi(2) * phase.cos();
            let qd_meas = v[leg_v_idx[row].0];
            accel_ref[row] = qdd_ref + KD_JOINT * (qd_ref - qd_meas) + KP_JOINT * (q_ref - q_meas);
            max_track_err = max_track_err.max((q_ref - q_meas).abs());
        }

        let dyn_ctx = Dynamics::new(Formulation::Explicit, &mass, &h, &j_contact, na_count);

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

        // A real weld is BOTH "free reaction force" (the j_contact rows
        // above) AND "prescribed zero acceleration" -- the base literally
        // cannot move. Without this, the solver is free to invent
        // nonzero base qddot that cancels the leg's own inertial term in
        // the actuated-row EoM via mass-matrix coupling, leaving tau
        // stuck at its gravity-comp value even while qddot_leg correctly
        // tracks the P1 target (confirmed: hand-computing tau from the
        // EoM using the solved qddot matched extracted.tau exactly --
        // the equations were self-consistent, just self-consistent with
        // a physically-wrong, unconstrained base acceleration).
        let mut j_base_fixed = na::DMatrix::zeros(6, nv);
        for i in 0..6 {
            j_base_fixed[(i, i)] = 1.0;
        }
        let dj_v_base_fixed = na::DVector::zeros(6);

        let p0 = dyn_ctx
            .dynamics_task()
            .expect("Explicit formulation always keeps the EoM task")
            + tasks::box_bound(dyn_ctx.tau(), &torque_max)
            + tasks::zero_contact_acceleration(dyn_ctx.qddot(), &j_base_fixed, &dj_v_base_fixed);
        let p1 = tasks::cartesian_acceleration(dyn_ctx.qddot(), &j_squat, &dj_v_squat, &accel_ref);
        let p2 = tasks::regularize(dyn_ctx.tau(), &tau_gravity);

        let sol = solver
            .solve(&[p0, p1, p2], &cfg)
            .unwrap_or_else(|e| panic!("wbc solve failed at t={t:.3}: {e}"));
        let extracted = dyn_ctx.extract(&sol.x);

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
        sim.set_wbc_torques(&robot_taus);
        sim.step_n_frames(&mut robot, mj_substeps, true);

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

        if tick % ((PERIOD_S / dt) as usize / 4).max(1) == 0 {
            let lk = robot.joint_positions[*robot.joint_map.get("left_knee_joint").unwrap()];
            let lhp = robot.joint_positions[*robot.joint_map.get("left_hip_pitch_joint").unwrap()];
            let knee_ref = leg_joints[1].1 + leg_joints[1].2 * SQUAT_AMP * phase.cos();
            println!(
                "  t={t:6.3}  left_knee={lk:+.3} (ref {knee_ref:+.3})  left_hip_pitch={lhp:+.3}  status={:?}",
                sol.status,
            );
        }
    }

    println!("\n=== Result ===");
    println!("  max joint tracking error over the run: {max_track_err:.4} rad");
    println!("  trajectory log written to {log_path}");
}

#[cfg(not(feature = "mujoco"))]
fn main() {
    eprintln!("This example requires the `mujoco` feature. Run with:");
    eprintln!("  cargo run --features mujoco --example kyo46rs_hanging_squat");
}
