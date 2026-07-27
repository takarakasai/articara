//! kyo46rs hanging + contact check: torso WELDED to the world (same
//! rig as kyo46rs_njoint_check.rs, EL05 damping=0.15/armature=0.0005
//! already validated to give 0% solver failures for pure joint-space
//! tracking there), but now with a REAL ground plane and the feet
//! actually resting on it -- P0 gets the SAME zero_contact_acceleration
//! + patch_contact tasks kyo46rs_squat.rs uses, first for ONE foot
//! (N_FEET=1) then BOTH (N_FEET=2), with NO attitude/height task (the
//! base can't move, so there's nothing to correct) -- just gravity-comp
//! holding via P2, matching the already-proven-stable methodology.
//!
//! This isolates whether the `level: 0` NumericalFailure seen in the
//! FLOATING-base double-support tests (kyo46rs_double_support_check.rs,
//! kyo46rs_squat.rs) comes from the contact-task formulation itself
//! (zero_contact_acceleration + patch_contact), reproducible even with
//! the base welded and no attitude/height coupling at all, or whether
//! it only appears once the floating base is added back.
//!
//! Edit N_FEET (1 or 2) and rerun for each step.
//!
//! Run with: `cargo run --features mujoco --example kyo46rs_hanging_contact_check`

#[cfg(feature = "mujoco")]
fn main() {
    use articara::mjcf::{GroundPlaneCfg, MjcfExportOptions};
    use articara::mujoco_sim::MujocoSim;
    use articara::rbd::model::ActuatorMode;
    use articara::robot::RobotModel;
    use articara::wbc_pipeline::build_floating_base_model;
    use misa_wbc::{tasks, AsAffine, Dynamics, Formulation, SolveConfig, Solver};
    use nalgebra as na;

    // ── EDIT THIS: 1 = left foot only, 2 = both feet ───────────────────
    const N_FEET: usize = 2;

    let urdf_path = std::path::Path::new(
        "/home/takara/work/dp/humanoid/kyo46rs_description/urdf/kyo46rs.urdf",
    );
    let mut robot = RobotModel::from_urdf(urdf_path).expect("load kyo46rs.urdf");

    let seed = [
        ("left_hip_pitch_joint", -0.35),
        ("left_knee_joint", 0.70),
        ("left_ankle_pitch_joint", -0.45),
        ("right_hip_pitch_joint", -0.35),
        ("right_knee_joint", 0.70),
        ("right_ankle_pitch_joint", -0.45),
        ("left_shoulder_pitch_joint", -1.0),
        ("left_elbow_joint", 1.2),
        ("right_shoulder_pitch_joint", -1.0),
        ("right_elbow_joint", 1.2),
    ];
    for (name, q) in seed {
        if let Some(&ji) = robot.joint_map.get(name) {
            robot.joint_positions[ji] = q;
        }
    }
    robot.rebuild_misarta_model();

    const EL05_JOINT_DAMPING: f64 = 0.15;
    const EL05_ARMATURE: f64 = 0.0005;
    for j in robot.joints.iter_mut() {
        j.actuator_mode = ActuatorMode::Torque;
        j.joint_damping = EL05_JOINT_DAMPING;
        j.armature = EL05_ARMATURE;
    }

    // Torso WELDED (same as kyo46rs_njoint_check.rs) but at the SAME
    // spawn height kyo46rs_squat.rs uses (feet resting on a real ground
    // plane, not floating in mid-air with no ground at all).
    let opts = MjcfExportOptions {
        base_pos: Some([0.0, 0.0, 0.41 + 0.059 + 0.002]),
        base_locked_axes: [true; 6],
        ground_plane: Some(GroundPlaneCfg { z: 0.0, half_size: 2.0, roll: 0.0, pitch: 0.0 }),
        ..MjcfExportOptions::default()
    };
    let mut sim = MujocoSim::new(&robot, opts).expect("MujocoSim::new");
    let mj_dt = sim.timestep();
    println!("MuJoCo timestep = {mj_dt} s, N_FEET={N_FEET}");
    {
        let lfoot = sim.body_world_position("left_foot_link").unwrap();
        let rfoot = sim.body_world_position("right_foot_link").unwrap();
        println!("  t=0 (pre-burn-in) left_foot_z={:.4} right_foot_z={:.4}", lfoot[2], rfoot[2]);
    }

    let (model, a2m, link_to_idx) = build_floating_base_model(&robot);
    let nv = model.nv;
    let na_count = nv - 6;
    let left_foot_mi = *link_to_idx.get("left_foot_link").expect("left_foot_link");
    let right_foot_mi = *link_to_idx.get("right_foot_link").expect("right_foot_link");

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

    // Brief settle: hold the seed pose in Position mode for a moment so
    // any initial ground-contact impulse dissipates before switching to
    // WBC torque control (same rationale as kyo46rs_squat.rs's burn-in).
    for j in robot.joints.iter_mut() {
        j.actuator_mode = ActuatorMode::Position;
        j.actuator_kp = 40.0;
        j.actuator_kv = 2.0;
    }
    for (name, q) in seed {
        if let Some(&ji) = robot.joint_map.get(name) {
            sim.set_position_target(ji, q);
        }
    }
    sim.step_n_frames(&mut robot, (0.15 / mj_dt) as u32, true);
    for j in robot.joints.iter_mut() {
        j.actuator_mode = ActuatorMode::Torque;
    }
    {
        let lfoot = sim.body_world_position("left_foot_link").unwrap();
        let rfoot = sim.body_world_position("right_foot_link").unwrap();
        println!("post-burn-in: left_foot_z={:.4} right_foot_z={:.4}", lfoot[2], rfoot[2]);
    }

    let mut solver = Solver::new();
    let cfg = SolveConfig::default();
    const FRICTION_MU: f64 = 0.6;

    let mj_substeps = (0.005 / mj_dt).round().max(1.0) as u32;
    let dt = mj_substeps as f64 * mj_dt;
    println!("control dt = {dt:.4} s ({mj_substeps} physics substeps/tick)");
    let n_ticks = (3.0 / dt) as usize;

    let mut n_degraded: u32 = 0;
    let mut max_joint_drift: f64 = 0.0;
    let q0: Vec<f64> = seed.iter().map(|(name, _)| robot.joint_positions[*robot.joint_map.get(*name).unwrap()]).collect();

    for tick in 0..n_ticks {
        let t = tick as f64 * dt;

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
                    const JOINT_V_MAX: f64 = 5.0;
                    v[model.v_idx[mi]] = qd.clamp(-JOINT_V_MAX, JOINT_V_MAX);
                }
            }
        }

        let mass = misarta::crba::crba(&model, &q);
        let h = misarta::rnea::nonlinear_effects(&model, &q, &v);
        let v_dvec = na::DVector::from_column_slice(&v);

        let feet: &[usize] = if N_FEET == 1 { &[left_foot_mi] } else { &[left_foot_mi, right_foot_mi] };
        let nc = feet.len();

        // A welded base is BOTH "free reaction force" AND "prescribed
        // zero acceleration" (see kyo46rs_hanging_squat.rs) -- without
        // an explicit base-mount contact + zero_contact_acceleration on
        // qddot[0..6], the solver can hide torque behind a fictitious
        // nonzero base acceleration even while the feet's own contact
        // rows are satisfied (their Jacobians span the base columns
        // too, so "feet don't move" alone doesn't pin the base). Rows
        // 0..6 here are that virtual mount; foot contacts follow after.
        let mut j_contact = na::DMatrix::zeros(6 + 6 * nc, nv);
        let mut dj_v = na::DVector::zeros(6 + 6 * nc);
        for i in 0..6 {
            j_contact[(i, i)] = 1.0;
        }
        for (slot, &foot_mi) in feet.iter().enumerate() {
            let j_full = misarta::jacobian::compute_joint_jacobian(&model, &q, foot_mi);
            let dj_full = misarta::jacobian::compute_joint_jacobian_time_derivative(&model, &q, &v, foot_mi);
            let dj_v_full = &dj_full * &v_dvec;
            for r in 0..6 {
                for c in 0..nv {
                    j_contact[(6 + 6 * slot + r, c)] = j_full[(r, c)];
                }
                dj_v[6 + 6 * slot + r] = dj_v_full[r];
            }
        }

        let dyn_ctx = Dynamics::new(Formulation::Explicit, &mass, &h, &j_contact, na_count);

        let forces = dyn_ctx.forces();
        let sole_patch = tasks::ContactPatch { mu: FRICTION_MU, cop_half: (0.049, 0.019), mu_torsion: 0.05, f_max: 150.0 };

        let j_base = j_contact.rows(0, 6).into_owned();
        let dj_v_base = dj_v.rows(0, 6).into_owned();
        let mut p0 = dyn_ctx
            .dynamics_task()
            .expect("Explicit formulation always keeps the EoM task")
            + tasks::box_bound(dyn_ctx.tau(), &torque_max)
            + tasks::zero_contact_acceleration(dyn_ctx.qddot(), &j_base, &dj_v_base);
        for slot in 0..nc {
            let j_slot = j_contact.rows(6 + 6 * slot, 6).into_owned();
            let dj_v_slot = dj_v.rows(6 + 6 * slot, 6).into_owned();
            let mut sel = na::DMatrix::zeros(6, forces.size());
            for k in 0..6 {
                sel[(k, 6 + 6 * slot + k)] = 1.0;
            }
            let w_slot = &sel * &forces.as_affine();
            p0 = p0
                + tasks::zero_contact_acceleration(dyn_ctx.qddot(), &j_slot, &dj_v_slot)
                + tasks::patch_contact(&w_slot, &sole_patch);
        }

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
        // Without a target, the QP is indifferent between routing support
        // load through a real foot (bounded by patch_contact) and the
        // virtual base-mount (free, unconstrained) -- the EoM's equality
        // residual is satisfied equally well either way. That leaves the
        // foot's fz free to drift near 0, exactly where patch_contact's
        // linearized friction-cone/CoP-box rows all become simultaneously
        // (near-)active and the active-set QP degenerates. Give each real
        // foot a nonzero nominal fz (share of body weight) so the least-
        // squares split has a well-posed, non-degenerate optimum.
        const G: f64 = 9.81;
        let total_mass: f64 = robot.links.iter().map(|l| l.inertial.mass).sum();
        let mut forces_nominal = na::DVector::zeros(forces.size());
        for slot in 0..nc {
            forces_nominal[6 + 6 * slot + 5] = total_mass * G / nc as f64;
        }
        let p2 = tasks::regularize(dyn_ctx.tau(), &tau_gravity)
            + tasks::regularize(&dyn_ctx.forces(), &forces_nominal);

        let sol = solver
            .solve(&[p0, p2], &cfg)
            .unwrap_or_else(|e| panic!("wbc solve failed at t={t:.3}: {e}"));
        if !matches!(sol.status, misa_wbc::SolveStatus::Optimal) {
            n_degraded += 1;
        }
        let extracted = dyn_ctx.extract(&sol.x);

        if tick < 5 || (!matches!(sol.status, misa_wbc::SolveStatus::Optimal) && tick % 50 == 0) {
            let w = extracted.forces.rows(6, 6).into_owned();
            let (mx, my, mz, fx, fy, fz) = (w[0], w[1], w[2], w[3], w[4], w[5]);
            println!(
                "  [wrench foot0] t={t:6.3} status={:?} m=({mx:+.3},{my:+.3},{mz:+.3}) f=({fx:+.3},{fy:+.3},{fz:+.3})  |mx|<=Ly*fz? {} ({:.3}<={:.3})  |my|<=Lx*fz? {} ({:.3}<={:.3})",
                sol.status,
                mx.abs() <= sole_patch.cop_half.1 * fz + 1e-6,
                mx.abs(), sole_patch.cop_half.1 * fz,
                my.abs() <= sole_patch.cop_half.0 * fz + 1e-6,
                my.abs(), sole_patch.cop_half.0 * fz,
            );
        }

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

        for (i, (name, _)) in seed.iter().enumerate() {
            let ji = *robot.joint_map.get(*name).unwrap();
            let drift = (robot.joint_positions[ji] - q0[i]).abs();
            if drift > max_joint_drift {
                max_joint_drift = drift;
            }
        }

        if tick % 100 == 0 {
            let lfoot = sim.body_world_position("left_foot_link").unwrap();
            println!("  t={t:6.3}  left_foot_z={:.4}  max_joint_drift={max_joint_drift:.4}  status={:?}", lfoot[2], sol.status);
        }
    }

    println!("\n=== Result (N_FEET={N_FEET}, torso welded) ===");
    println!("  max joint drift from seed: {max_joint_drift:.4} rad");
    println!("  degraded (non-Optimal) solves: {n_degraded} / {n_ticks} ticks ({:.1}%)", 100.0 * n_degraded as f64 / n_ticks as f64);
}

#[cfg(not(feature = "mujoco"))]
fn main() {
    eprintln!("This example requires the `mujoco` feature. Run with:");
    eprintln!("  cargo run --features mujoco --example kyo46rs_hanging_contact_check");
}
