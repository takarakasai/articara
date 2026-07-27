//! kyo46rs single-leg-stance WBC balance test: same floating-base +
//! attitude/height task stack as kyo46rs_squat.rs, but with the RIGHT
//! leg lifted clear of the ground and P0's contact tasks reduced to
//! the LEFT foot only (6 contact rows instead of 12) -- the "increase
//! DOF one foot at a time" step requested after kyo46rs_squat.rs's
//! own double-support run kept hitting `level: 0` (contact-level)
//! solver NumericalFailure. This isolates whether that failure mode
//! is inherent to the EoM+attitude+height formulation itself, or
//! specific to the closed two-foot double-support loop.
//!
//! Left leg stays at the SAME crouch angles kyo46rs_squat.rs uses (so
//! the same base_pos spawn-height derivation still applies -- the
//! right leg's own kinematics don't affect the left foot's height at
//! all, independent chains off the same torso). Right leg is bent
//! further to lift its foot clear of the ground; a pre-burn-in
//! diagnostic print confirms the clearance.
//!
//! Run with: `cargo run --features mujoco --example kyo46rs_single_leg_stance`

#[cfg(feature = "mujoco")]
fn main() {
    use articara::mjcf::{GroundPlaneCfg, MjcfExportOptions};
    use articara::mujoco_sim::MujocoSim;
    use articara::rbd::model::ActuatorMode;
    use articara::robot::RobotModel;
    use articara::wbc_pipeline::build_floating_base_model;
    use misa_wbc::{tasks, AsAffine, Dynamics, Formulation, SolveConfig, Solver};
    use nalgebra as na;

    let urdf_path = std::path::Path::new(
        "/home/takara/work/dp/humanoid/kyo46rs_description/urdf/kyo46rs.urdf",
    );
    let mut robot = RobotModel::from_urdf(urdf_path).expect("load kyo46rs.urdf");

    // Left leg: same stance-crouch angles as kyo46rs_squat.rs (keeps the
    // same base_pos spawn-height formula valid). Right leg: bent further
    // to lift the foot clear of the ground.
    let left_stance = [
        ("left_hip_pitch_joint", -0.35),
        ("left_knee_joint", 0.70),
        ("left_ankle_pitch_joint", -0.45),
    ];
    let right_lifted = [
        ("right_hip_pitch_joint", -0.90),
        ("right_knee_joint", 1.60),
        ("right_ankle_pitch_joint", -0.30),
    ];
    let arm_pose = [
        ("left_shoulder_pitch_joint", -1.0),
        ("left_elbow_joint", 1.2),
        ("right_shoulder_pitch_joint", -1.0),
        ("right_elbow_joint", 1.2),
    ];
    for (name, q) in left_stance.iter().chain(right_lifted.iter()).chain(arm_pose.iter()) {
        if let Some(&ji) = robot.joint_map.get(*name) {
            robot.joint_positions[ji] = *q;
        }
    }
    robot.rebuild_misarta_model();

    // Robstride EduLite05 placeholders (see kyo46rs_squat.rs for the
    // full derivation/justification): damping=0.15 keeps a margin above
    // the 0.11 threshold found to eliminate P1-level solver
    // NumericalFailure in kyo46rs_njoint_check.rs's sweep.
    const EL05_JOINT_DAMPING: f64 = 0.15;
    const EL05_ARMATURE: f64 = 0.0005;
    for j in robot.joints.iter_mut() {
        j.actuator_mode = ActuatorMode::Position;
        j.actuator_kp = 40.0;
        j.actuator_kv = 2.0;
        j.joint_damping = EL05_JOINT_DAMPING;
        j.armature = EL05_ARMATURE;
    }

    let opts = MjcfExportOptions {
        // Same formula as kyo46rs_squat.rs -- valid because the LEFT
        // foot (the one this height must clear) uses the identical leg
        // angles; the right leg's geometry doesn't affect it.
        base_pos: Some([0.0, 0.0, 0.41 + 0.059 + 0.002]),
        ground_plane: Some(GroundPlaneCfg { z: 0.0, half_size: 2.0, roll: 0.0, pitch: 0.0 }),
        ..MjcfExportOptions::default()
    };
    let mut sim = MujocoSim::new(&robot, opts).expect("MujocoSim::new");
    let mj_dt = sim.timestep();
    println!("MuJoCo timestep = {mj_dt} s");
    {
        let lfoot = sim.body_world_position("left_foot_link").unwrap();
        let rfoot = sim.body_world_position("right_foot_link").unwrap();
        println!("  t=0 (pre-burn-in) left_foot_z={:.4}  right_foot_z={:.4}  clearance={:.4}", lfoot[2], rfoot[2], rfoot[2] - lfoot[2]);
    }

    let (model, a2m, link_to_idx) = build_floating_base_model(&robot);
    let nv = model.nv;
    let na_count = nv - 6;
    println!("misarta floating-base model: nv={nv} na={na_count}");

    let left_foot_mi = *link_to_idx.get("left_foot_link").expect("left_foot_link");

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

    for (name, q) in left_stance.iter().chain(right_lifted.iter()) {
        if let Some(&ji) = robot.joint_map.get(*name) {
            sim.set_position_target(ji, *q);
        }
    }
    sim.step_n_frames(&mut robot, (0.15 / mj_dt) as u32, true);

    for j in robot.joints.iter_mut() {
        j.actuator_mode = ActuatorMode::Torque;
    }

    let z_hi = sim.body_world_position(&robot.root_link).expect("torso xpos")[2];
    let post_rpy = sim.body_world_orientation(&robot.root_link).unwrap().euler_angles();
    let lfoot = sim.body_world_position("left_foot_link").unwrap();
    let rfoot = sim.body_world_position("right_foot_link").unwrap();
    println!(
        "post-burn-in: trunk z={z_hi:.3} rpy=({:+.3},{:+.3},{:+.3})  left_foot_z={:.4} right_foot_z={:.4} clearance={:.4}",
        post_rpy.0, post_rpy.1, post_rpy.2, lfoot[2], rfoot[2], rfoot[2] - lfoot[2]
    );

    let mut solver = Solver::new();
    let cfg = SolveConfig::default();
    const FRICTION_MU: f64 = 0.6;
    const KP_Z: f64 = 3000.0;
    const KD_Z: f64 = 300.0;
    const KP_ATT: f64 = 500.0;
    const KD_ATT: f64 = 150.0;
    const KI_ATT: f64 = 80.0;
    const I_ATT_CLAMP: f64 = 0.15;
    const FALL_Z_M: f64 = 0.30;
    const FALL_TILT_RAD: f64 = 0.52;

    let mj_substeps = (0.005 / mj_dt).round().max(1.0) as u32;
    let dt = mj_substeps as f64 * mj_dt;
    println!("control dt = {dt:.4} s ({mj_substeps} physics substeps/tick)");
    let total_t = 6.0;
    let n_ticks = (total_t / dt) as usize;

    let mut min_z = z_hi;
    let mut max_tilt: f64 = 0.0;
    let mut fell = false;
    let mut roll_i = 0.0_f64;
    let mut pitch_i = 0.0_f64;

    for tick in 0..n_ticks {
        let t = tick as f64 * dt;

        let body_pos = sim.body_world_position(&robot.root_link).unwrap();
        let body_quat = sim.body_world_orientation(&robot.root_link).unwrap();
        let v_lin_world = sim.body_world_linear_velocity(&robot.root_link).unwrap();
        let v_ang_world = sim.body_world_angular_velocity(&robot.root_link).unwrap();
        let r_bw = body_quat.to_rotation_matrix().transpose();
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

        let mass = misarta::crba::crba(&model, &q);
        let h = misarta::rnea::nonlinear_effects(&model, &q, &v);
        let v_dvec = na::DVector::from_column_slice(&v);

        // ONLY the left foot's 6D contact -- 6 rows, not 12.
        let mut j_contact = na::DMatrix::zeros(6, nv);
        let mut dj_v = na::DVector::zeros(6);
        {
            let j_full = misarta::jacobian::compute_joint_jacobian(&model, &q, left_foot_mi);
            let dj_full = misarta::jacobian::compute_joint_jacobian_time_derivative(&model, &q, &v, left_foot_mi);
            let dj_v_full = &dj_full * &v_dvec;
            for r in 0..6 {
                for c in 0..nv {
                    j_contact[(r, c)] = j_full[(r, c)];
                }
                dj_v[r] = dj_v_full[r];
            }
        }

        let mut j_att = na::DMatrix::zeros(2, nv);
        j_att[(0, 0)] = 1.0;
        j_att[(1, 1)] = 1.0;
        let dj_v_att = na::DVector::zeros(2);
        let mut j_height = na::DMatrix::zeros(1, nv);
        j_height[(0, 5)] = 1.0;
        let dj_v_height = na::DVector::zeros(1);

        let z_ref = z_hi; // static balance, no squat motion
        let (roll_meas, pitch_meas, _yaw) = body_quat.euler_angles();
        let z_meas = body_pos[2];
        let zd_meas = v_lin_body.z;
        let az_cmd = KD_Z * (0.0 - zd_meas) + KP_Z * (z_ref - z_meas);

        roll_i = (roll_i + roll_meas * dt).clamp(-I_ATT_CLAMP, I_ATT_CLAMP);
        pitch_i = (pitch_i + pitch_meas * dt).clamp(-I_ATT_CLAMP, I_ATT_CLAMP);
        let a_roll_cmd = -(KD_ATT * (0.0 - v_ang_body.x) + KP_ATT * (0.0 - roll_meas) + KI_ATT * (0.0 - roll_i));
        let a_pitch_cmd = -(KD_ATT * (0.0 - v_ang_body.y) + KP_ATT * (0.0 - pitch_meas) + KI_ATT * (0.0 - pitch_i));
        let att_ref = na::DVector::from_vec(vec![a_roll_cmd, a_pitch_cmd]);
        let height_ref = na::DVector::from_vec(vec![az_cmd]);

        let dyn_ctx = Dynamics::new(Formulation::Explicit, &mass, &h, &j_contact, na_count);

        let forces = dyn_ctx.forces();
        let sel_left = na::DMatrix::<f64>::identity(6, forces.size());
        let w_left = &sel_left * &forces.as_affine();

        let sole_patch = tasks::ContactPatch { mu: FRICTION_MU, cop_half: (0.049, 0.019), mu_torsion: 0.05, f_max: 150.0 };

        let p0 = dyn_ctx
            .dynamics_task()
            .expect("Explicit formulation always keeps the EoM task")
            + tasks::zero_contact_acceleration(dyn_ctx.qddot(), &j_contact, &dj_v)
            + tasks::patch_contact(&w_left, &sole_patch)
            + tasks::box_bound(dyn_ctx.tau(), &torque_max);

        let p1 = tasks::cartesian_acceleration(dyn_ctx.qddot(), &j_att, &dj_v_att, &att_ref);
        let p1h = tasks::cartesian_acceleration(dyn_ctx.qddot(), &j_height, &dj_v_height, &height_ref);

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
        let p2 = tasks::regularize(dyn_ctx.tau(), &tau_gravity);

        let sol = solver
            .solve(&[p0, p1, p1h, p2], &cfg)
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

        let cur_z = sim.body_world_position(&robot.root_link).unwrap()[2];
        min_z = min_z.min(cur_z);
        let tilt = roll_meas.abs().max(pitch_meas.abs());
        max_tilt = max_tilt.max(tilt);
        if cur_z < FALL_Z_M || tilt > FALL_TILT_RAD {
            fell = true;
        }

        if tick % 20 == 0 || tick < 10 {
            println!(
                "  t={t:6.3}  z={cur_z:+.3} (ref {z_ref:+.3})  roll={roll_meas:+.3} pitch={pitch_meas:+.3}  status={:?}",
                sol.status,
            );
        }
        if fell {
            println!("  FELL at t={t:.3} (z={cur_z:.3}, tilt={tilt:.3} rad)");
            break;
        }
    }

    println!("\n=== Result (single-leg stance, LEFT foot only) ===");
    println!("  z_hi (standing) = {z_hi:.3} m");
    println!("  min z reached   = {min_z:.3} m");
    println!("  max |roll|/|pitch| = {max_tilt:.3} rad");
    println!("  verdict: {}", if fell { "FELL" } else { "BALANCED" });
}

#[cfg(not(feature = "mujoco"))]
fn main() {
    eprintln!("This example requires the `mujoco` feature. Run with:");
    eprintln!("  cargo run --features mujoco --example kyo46rs_single_leg_stance");
}
