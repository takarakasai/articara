//! kyo46rs single-joint controllability check: torso WELDED mid-air
//! (same rig as kyo46rs_hanging_squat.rs), but only ONE joint
//! (left_knee_joint) gets an active misa-wbc tracking task. Every other
//! joint is held purely by the P2 gravity-compensation regularizer (no
//! tracking task of its own) -- confirmed to hold well when seeded away
//! from a near-zero-torque pose (kyo46rs_fullbody_gravity_check.rs).
//!
//! This is the simplest possible controllability test for the WBC: one
//! actuated DOF, one tracking task, no cross-coupling from other active
//! tasks, no floating-base attitude/height coupling at all.
//!
//! Two phases:
//! 1. STEP response: seed at 0.70 rad, command a step to 1.05 rad, hold,
//!    then step back down to 0.35 rad -- reports rise time, overshoot,
//!    settling behaviour.
//! 2. SINUSOID tracking: same amplitude/period as kyo46rs_hanging_squat.rs
//!    for a direct before/after comparison against the 6-joint version.
//!
//! Run with: `cargo run --features mujoco --example kyo46rs_single_joint_check`

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

    const TARGET_JOINT: &str = "left_knee_joint";
    const SEED: f64 = 0.70;

    // Everything else seeded to the same crouch/arm pose validated to
    // hold well under gravity comp alone (kyo46rs_fullbody_gravity_check.rs)
    // -- these joints get NO tracking task, only the P2 regularizer.
    let hold_pose = [
        ("left_hip_pitch_joint", -0.35),
        ("left_ankle_pitch_joint", -0.45),
        ("right_hip_pitch_joint", -0.35),
        ("right_knee_joint", 0.70),
        ("right_ankle_pitch_joint", -0.45),
        ("left_shoulder_pitch_joint", -1.0),
        ("left_elbow_joint", 1.2),
        ("right_shoulder_pitch_joint", -1.0),
        ("right_elbow_joint", 1.2),
    ];
    if let Some(&ji) = robot.joint_map.get(TARGET_JOINT) {
        robot.joint_positions[ji] = SEED;
    }
    for (name, q) in hold_pose {
        if let Some(&ji) = robot.joint_map.get(name) {
            robot.joint_positions[ji] = q;
        }
    }
    robot.rebuild_misarta_model();

    for j in robot.joints.iter_mut() {
        j.actuator_mode = ActuatorMode::Torque;
    }

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

    let target_ji = *robot.joint_map.get(TARGET_JOINT).expect("target joint in URDF");
    let target_mi = a2m[target_ji].expect("target joint mapped into misarta model");
    let target_vidx = model.v_idx[target_mi];

    let mut solver = Solver::new();
    let cfg = SolveConfig::default();
    const KP_JOINT: f64 = 150.0;
    const KD_JOINT: f64 = 30.0;

    let mj_substeps = (0.005 / mj_dt).round().max(1.0) as u32;
    let dt = mj_substeps as f64 * mj_dt;
    println!("control dt = {dt:.4} s ({mj_substeps} physics substeps/tick)");

    // Shared per-tick solve, parameterised only by the target position/
    // velocity/acceleration reference for TARGET_JOINT.
    let mut run_tick = |robot: &mut RobotModel,
                        sim: &mut MujocoSim,
                        q_ref: f64,
                        qd_ref: f64,
                        qdd_ref: f64|
     -> (f64, f64, misa_wbc::SolveStatus) {
        let body_pos = sim.body_world_position(&robot.root_link).unwrap();
        let body_quat = sim.body_world_orientation(&robot.root_link).unwrap();
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

        let mut j_contact = na::DMatrix::zeros(6, nv);
        for i in 0..6 {
            j_contact[(i, i)] = 1.0;
        }

        let q_meas = robot.joint_positions[target_ji];
        let qd_meas = v[target_vidx];
        let accel_cmd = qdd_ref + KD_JOINT * (qd_ref - qd_meas) + KP_JOINT * (q_ref - q_meas);

        let mut j_single = na::DMatrix::zeros(1, nv);
        j_single[(0, target_vidx)] = 1.0;
        let dj_v_single = na::DVector::zeros(1);
        let accel_ref = na::DVector::from_vec(vec![accel_cmd]);

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
        let p1 = tasks::cartesian_acceleration(dyn_ctx.qddot(), &j_single, &dj_v_single, &accel_ref);
        let p2 = tasks::regularize(dyn_ctx.tau(), &tau_gravity);

        let sol = solver
            .solve(&[p0, p1, p2], &cfg)
            .unwrap_or_else(|e| panic!("wbc solve failed: {e}"));
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
        sim.step_n_frames(robot, mj_substeps, true);

        let q_now = robot.joint_positions[target_ji];
        let qd_now = sim.joint_q_qd(TARGET_JOINT).map(|(_, qd)| qd).unwrap_or(0.0);
        (q_now, qd_now, sol.status)
    };

    // ── Phase 1: step response ──────────────────────────────────────────
    println!("\n=== Phase 1: step response (target={TARGET_JOINT}) ===");
    let steps = [(SEED, 1.0), (1.05, 1.5), (0.35, 1.5)];
    let mut t = 0.0;
    let mut settle_reported = false;
    for (step_target, duration_s) in steps {
        println!("  step to {step_target:+.3} rad, holding {duration_s:.1}s");
        let n = (duration_s / dt) as usize;
        let mut overshoot: f64 = 0.0;
        let step_start_q = robot.joint_positions[target_ji];
        let rising = step_target > step_start_q;
        settle_reported = false;
        for i in 0..n {
            let (q_now, qd_now, status) = run_tick(&mut robot, &mut sim, step_target, 0.0, 0.0);
            t += dt;
            if rising {
                overshoot = overshoot.max(q_now - step_target);
            } else {
                overshoot = overshoot.max(step_target - q_now);
            }
            let err = (q_now - step_target).abs();
            if !settle_reported && err < 0.02 && i > 5 {
                println!("    settled within 0.02 rad at t={t:.3}s ({:.0} ms after step)", (i as f64 * dt) * 1000.0);
                settle_reported = true;
            }
            if i % ((0.5 / dt) as usize).max(1) == 0 {
                println!("    t={t:6.3}  q={q_now:+.4} qd={qd_now:+.3}  status={status:?}");
            }
        }
        println!("    max overshoot past target: {:.4} rad", overshoot.max(0.0));
    }

    // ── Phase 2: sinusoid tracking (matches kyo46rs_hanging_squat.rs) ──
    println!("\n=== Phase 2: sinusoid tracking (target={TARGET_JOINT}) ===");
    const PERIOD_S: f64 = 2.0;
    const N_CYCLES: u32 = 3;
    const AMP: f64 = 0.35;
    let total_t = PERIOD_S * N_CYCLES as f64;
    let n_ticks = (total_t / dt) as usize;
    let mut max_track_err: f64 = 0.0;
    let mut t2 = 0.0;
    for tick in 0..n_ticks {
        let phase = 2.0 * PI * t2 / PERIOD_S;
        let q_ref = SEED - AMP * phase.cos();
        let qd_ref = AMP * (2.0 * PI / PERIOD_S) * phase.sin();
        let qdd_ref = AMP * (2.0 * PI / PERIOD_S).powi(2) * phase.cos();
        let (q_now, _qd_now, status) = run_tick(&mut robot, &mut sim, q_ref, qd_ref, qdd_ref);
        max_track_err = max_track_err.max((q_now - q_ref).abs());
        if tick % ((PERIOD_S / dt) as usize / 4).max(1) == 0 {
            println!("  t={t2:6.3}  q={q_now:+.4} (ref {q_ref:+.4})  status={status:?}");
        }
        t2 += dt;
    }

    println!("\n=== Result ===");
    println!("  max sinusoid tracking error: {max_track_err:.4} rad");
}

#[cfg(not(feature = "mujoco"))]
fn main() {
    eprintln!("This example requires the `mujoco` feature. Run with:");
    eprintln!("  cargo run --features mujoco --example kyo46rs_single_joint_check");
}
