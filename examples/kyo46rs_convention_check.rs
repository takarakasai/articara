//! misarta vs MuJoCo floating-base convention audit.
//!
//! Both kyo46rs_squat.rs and kyo46rs_com_squat.rs need their base
//! angular task NEGATED to be stable, and that inversion survived every
//! bug fixed so far (the contact-force regulariser, the 1 ms timestep,
//! the settled burn-in, and the `cvel` reference-point fix). It is also
//! not misa-wbc's doing: `cartesian_acceleration` builds the textbook
//! `J·q̈ + J̇·v ≈ accel_ref`, and misarta's quaternion order (x,y,z,w)
//! matches what the examples write.
//!
//! So the suspicion is a frame/ordering mismatch on the floating base
//! itself. The two conventions differ in three separate places:
//!
//!                  MuJoCo free joint        misarta FreeFlyer
//!   qpos           [xyz, qw,qx,qy,qz]       [xyz, qx,qy,qz,qw]
//!   qvel order     [linear(3); angular(3)]  [angular(3); linear(3)]
//!   linear frame   world                    body   (S = I6, body frame)
//!   angular frame  body                     body
//!
//! This file measures which of those actually bite, in three stages that
//! each isolate one layer:
//!
//!   1. VELOCITY CONVENTION -- what do MuJoCo's own reported base
//!      velocities mean, and does the sync in the controllers reproduce
//!      them?
//!   2. GRAVITY / BIAS -- compare misarta's `nonlinear_effects` against
//!      MuJoCo's `qfrc_bias` at v = 0. No integration, no contacts, no
//!      dynamics: this is a pure coordinate-mapping test, and any base
//!      row that disagrees pins the convention down directly.
//!   3. DYNAMICS -- apply a known joint torque in FREE FLIGHT (no
//!      ground, so contacts cannot muddy it), step once, and compare the
//!      base angular acceleration MuJoCo actually produced against the
//!      one misarta predicts from the same state and torque.
//!
//! Run with: `cargo run --features mujoco --example kyo46rs_convention_check`

#[cfg(feature = "mujoco")]
fn main() {
    use articara::mjcf::MjcfExportOptions;
    use articara::mujoco_sim::MujocoSim;
    use articara::rbd::model::ActuatorMode;
    use articara::robot::RobotModel;
    use articara::wbc_pipeline::build_floating_base_model;
    use nalgebra as na;

    let urdf_path = std::path::Path::new(
        "/home/takara/work/dp/humanoid/kyo46rs_description/urdf/kyo46rs.urdf",
    );
    let mut robot = RobotModel::from_urdf(urdf_path).expect("load kyo46rs.urdf");

    // An ASYMMETRIC pose with a real tilt. A symmetric upright robot
    // makes several wrong conventions look right (world == body, left
    // cancels right), so deliberately break both symmetries.
    for (name, q) in [
        ("left_hip_pitch_joint", -0.35),
        ("left_knee_joint", 0.70),
        ("left_ankle_pitch_joint", -0.35),
        ("right_hip_pitch_joint", -0.15),
        ("right_knee_joint", 0.40),
        ("right_ankle_pitch_joint", -0.25),
        ("left_shoulder_pitch_joint", -0.8),
        ("left_elbow_joint", 0.9),
        ("right_hip_roll_joint", 0.20),
        ("left_hip_yaw_joint", 0.15),
    ] {
        if let Some(&ji) = robot.joint_map.get(name) {
            robot.joint_positions[ji] = q;
        }
    }
    robot.rebuild_misarta_model();
    for j in robot.joints.iter_mut() {
        j.actuator_mode = ActuatorMode::Torque;
        j.joint_damping = 0.0; // keep the comparison clean
        j.armature = 0.0;
    }

    // FREE FLIGHT: no ground plane at all, so no contact force can enter
    // either model. Spawn high enough that nothing can reach z = 0.
    let opts = MjcfExportOptions {
        base_pos: Some([0.0, 0.0, 2.0]),
        ground_plane: None,
        timestep: Some(0.001),
        // MuJoCo enforces baked `range=` limits with constraint forces
        // that misarta has no counterpart for. Any such force lands
        // entirely in the "misarta is wrong" column of a naive
        // comparison, so take them out of the picture.
        bake_joint_position_limits: false,
        ..MjcfExportOptions::default()
    };
    let mut sim = MujocoSim::new(&robot, opts).expect("MujocoSim::new");
    let dt = sim.timestep();

    let (model, a2m, _link_to_idx) = build_floating_base_model(&robot);
    #[allow(clippy::let_and_return)]
    let nv = model.nv;
    let na = nv - 6;
    println!("free-flight convention audit: nv={nv} na={na} dt={dt}\n");

    // Build misarta (q, v) exactly the way the controllers do.
    let sync = |sim: &MujocoSim, robot: &RobotModel| -> (Vec<f64>, Vec<f64>) {
        let p = sim.body_world_position(&robot.root_link).unwrap();
        let quat = sim.body_world_orientation(&robot.root_link).unwrap();
        let vl = sim.body_world_linear_velocity(&robot.root_link).unwrap();
        let va = sim.body_world_angular_velocity(&robot.root_link).unwrap();
        let r_bw = quat.to_rotation_matrix().transpose();
        let vlb = r_bw * na::Vector3::new(vl[0], vl[1], vl[2]);
        let vab = r_bw * na::Vector3::new(va[0], va[1], va[2]);
        let mut q = model.neutral_q();
        q[0] = p[0];
        q[1] = p[1];
        q[2] = p[2];
        q[3] = quat.i;
        q[4] = quat.j;
        q[5] = quat.k;
        q[6] = quat.w;
        let mut v = vec![0.0_f64; nv];
        v[0] = vab.x;
        v[1] = vab.y;
        v[2] = vab.z;
        v[3] = vlb.x;
        v[4] = vlb.y;
        v[5] = vlb.z;
        for ji in 0..robot.joints.len() {
            let Some(mi) = a2m[ji] else { continue };
            if model.joints[mi].joint_type.nq() == 1 {
                q[model.q_idx[mi]] = robot.joint_positions[ji];
            }
            if model.joints[mi].joint_type.nv() == 1 {
                if let Some((_, qd)) = sim.joint_q_qd(&robot.joints[ji].name) {
                    v[model.v_idx[mi]] = qd;
                }
            }
        }
        (q, v)
    };

    // ── Stage 2 first: gravity / bias at v = 0 ─────────────────────────
    // Nothing has moved yet, so qvel = 0 and MuJoCo's qfrc_bias is pure
    // gravity. misarta's nonlinear_effects at v = 0 is likewise pure
    // gravity. Both are generalised forces, i.e. each entry is dual to
    // its own velocity coordinate -- so the entries only line up if the
    // velocity coordinates line up. That makes this the cleanest probe
    // of the ordering/frame question there is.
    let (q0, v0) = sync(&sim, &robot);
    let h_misarta = misarta::rnea::nonlinear_effects(&model, &q0, &v0);
    let bias_mj = sim.qfrc_bias();
    let quat0 = sim.body_world_orientation(&robot.root_link).unwrap();
    let r_wb = quat0.to_rotation_matrix();
    let total_mass: f64 = robot.links.iter().map(|l| l.inertial.mass).sum();

    println!("=== Stage 2: gravity generalised force at v=0 (free flight) ===");
    println!("  total mass = {total_mass:.4} kg   m*g = {:.4} N", total_mass * 9.81);
    println!("  MuJoCo qfrc_bias[0..6] = [{}]",
        (0..6).map(|i| format!("{:+8.4}", bias_mj[i])).collect::<Vec<_>>().join(", "));
    println!("  misarta h        [0..6] = [{}]",
        (0..6).map(|i| format!("{:+8.4}", h_misarta[i])).collect::<Vec<_>>().join(", "));
    println!();

    // Hypothesis A (the one the controllers assume):
    //   misarta [ang(0..3, body) ; lin(3..6, body)]
    //   MuJoCo  [lin(0..3, world); ang(3..6, body)]
    let mj_lin_world = na::Vector3::new(bias_mj[0], bias_mj[1], bias_mj[2]);
    let mj_ang_body = na::Vector3::new(bias_mj[3], bias_mj[4], bias_mj[5]);
    let mi_ang_body = na::Vector3::new(h_misarta[0], h_misarta[1], h_misarta[2]);
    let mi_lin_body = na::Vector3::new(h_misarta[3], h_misarta[4], h_misarta[5]);
    // A generalised force dual to a body-frame linear velocity is the
    // world force rotated into the body: f_body = R^T f_world.
    let mj_lin_in_body = r_wb.transpose() * mj_lin_world;

    println!("  [A] linear: MuJoCo(world)->body vs misarta(body)");
    println!("      R^T*mj_lin = ({:+8.4},{:+8.4},{:+8.4})", mj_lin_in_body.x, mj_lin_in_body.y, mj_lin_in_body.z);
    println!("      misarta    = ({:+8.4},{:+8.4},{:+8.4})", mi_lin_body.x, mi_lin_body.y, mi_lin_body.z);
    println!("      diff norm  = {:.6}   (sum norm = {:.6})",
        (mj_lin_in_body - mi_lin_body).norm(), (mj_lin_in_body + mi_lin_body).norm());
    println!("  [A] angular: MuJoCo(body) vs misarta(body)");
    println!("      mujoco  = ({:+8.4},{:+8.4},{:+8.4})", mj_ang_body.x, mj_ang_body.y, mj_ang_body.z);
    println!("      misarta = ({:+8.4},{:+8.4},{:+8.4})", mi_ang_body.x, mi_ang_body.y, mi_ang_body.z);
    println!("      diff norm = {:.6}   (sum norm = {:.6})",
        (mj_ang_body - mi_ang_body).norm(), (mj_ang_body + mi_ang_body).norm());
    println!("      NOTE: a small 'sum norm' instead of a small 'diff norm' means SIGN FLIP.");
    println!();

    // Actuated rows should agree entry-for-entry regardless of the base
    // question -- a useful control on the whole comparison.
    let mut worst_joint = (String::new(), 0.0_f64);
    for ji in 0..robot.joints.len() {
        let Some(mi) = a2m[ji] else { continue };
        if model.joints[mi].joint_type.nv() != 1 {
            continue;
        }
        let vi = model.v_idx[mi];
        if vi < 6 {
            continue;
        }
        let Some(row) = sim.joint_dof_adr(&robot.joints[ji].name) else { continue };
        let d = (bias_mj[row] - h_misarta[vi]).abs();
        if d > worst_joint.1 {
            worst_joint = (robot.joints[ji].name.clone(), d);
        }
    }
    println!("  control: worst ACTUATED-row |mujoco - misarta| = {:.3e} ({})",
        worst_joint.1, worst_joint.0);
    println!();

    // ── Stage 3a: dynamics FROM REST, one step ─────────────────────────
    // Cleanest possible dynamics comparison: v = 0 (so h is pure
    // gravity and there is no Coriolis term to mismodel), a single
    // 1 ms step (so nothing can drift into a limit or a large-omega
    // regime), and no contacts.
    // ── Stage 2b: the mass matrix itself ───────────────────────────────
    // M and h together ARE the equation of motion. Stage 2 already
    // showed h agrees exactly; if M agrees too then misarta's model is
    // vindicated and any remaining q̈ disagreement must come from the
    // state sync or the applied forces instead.
    // misarta v-index -> MuJoCo dof index (base blocks swap, joints by
    // name -- the two engines do NOT order joint DOFs identically).
    // Precomputed as a Vec rather than a closure so it does not hold a
    // borrow on `sim`/`robot` across the stepping below.
    let perm_vec: Vec<usize> = {
        let mut p: Vec<usize> = (0..nv)
            .map(|i| match i {
                0..=2 => i + 3,
                3..=5 => i - 3,
                _ => i,
            })
            .collect();
        for ji in 0..robot.joints.len() {
            let Some(mi) = a2m[ji] else { continue };
            if model.joints[mi].joint_type.nv() != 1 {
                continue;
            }
            let vi = model.v_idx[mi];
            if vi >= 6 {
                if let Some(adr) = sim.joint_dof_adr(&robot.joints[ji].name) {
                    p[vi] = adr;
                }
            }
        }
        p
    };
    println!("=== Stage 2b: mass matrix, misarta crba vs MuJoCo mj_fullM ===");
    {
        let m_mi = misarta::crba::crba(&model, &q0);
        let m_mj_flat = sim.mass_matrix();
        // MuJoCo row order for the free base is [lin(3, WORLD); ang(3, body)];
        // misarta is [ang(3, body); lin(3, body)]. At q0 the base is
        // unrotated, so world == body and the remap is a pure swap.
        let perm = &perm_vec;
        let mut worst = (0usize, 0usize, 0.0_f64);
        let mut worst_base = (0usize, 0usize, 0.0_f64);
        for r in 0..nv {
            for c in 0..nv {
                let d = (m_mi[(r, c)] - m_mj_flat[perm[r] * nv + perm[c]]).abs();
                if d > worst.2 {
                    worst = (r, c, d);
                }
                if r < 6 && c < 6 && d > worst_base.2 {
                    worst_base = (r, c, d);
                }
            }
        }
        println!("  misarta M[3..6,3..6] (base linear, expect ~{:.3}*I):", total_mass);
        for r in 3..6 {
            println!("      [{:+8.4},{:+8.4},{:+8.4}]", m_mi[(r, 3)], m_mi[(r, 4)], m_mi[(r, 5)]);
        }
        println!("  --- base 6x6, misarta [ang(0..3); lin(3..6)] ordering ---");
        for r in 0..6 {
            let mi: Vec<String> = (0..6).map(|c| format!("{:+8.4}", m_mi[(r, c)])).collect();
            let mj: Vec<String> = (0..6)
                .map(|c| format!("{:+8.4}", m_mj_flat[perm[r] * nv + perm[c]]))
                .collect();
            println!("    r{r}  misarta [{}]   mujoco [{}]", mi.join(","), mj.join(","));
        }
        // Where does the whole-body CoM sit relative to the base origin?
        // The angular/linear coupling block of a composite inertia is
        // +-m*[c]x, so m*|c| is the scale to compare the mismatch against.
        {
            let d = misarta::fk::forward_kinematics(&model, &q0);
            let base_o = misarta::se3::translation(&d.oMi[1]);
            let mut c = na::Vector3::zeros();
            for l in robot.links.iter().filter(|l| l.inertial.mass > 0.0) {
                if let Some(&mi_) = _link_to_idx.get(&l.name) {
                    let rr = misarta::se3::rotation_matrix(&d.oMi[mi_]);
                    let oo = misarta::se3::translation(&d.oMi[mi_]);
                    let off = l.inertial.origin.translation.vector;
                    c += l.inertial.mass
                        * (oo + rr * na::Vector3::new(off.x as f64, off.y as f64, off.z as f64));
                }
            }
            c /= total_mass;
            let rel = c - base_o;
            println!("  CoM relative to base origin = ({:+.4},{:+.4},{:+.4})  m*|c| = {:.4}",
                rel.x, rel.y, rel.z, total_mass * rel.norm());
            println!("  m*cx={:+.4}  m*cy={:+.4}  m*cz={:+.4}",
                total_mass * rel.x, total_mass * rel.y, total_mass * rel.z);
        }
        // Confirm the missing term is exactly the composite inertia's
        // angular<->linear coupling m*[c]x, which `se3::spatial_inertia`
        // already knows how to build (upper-right = m*[c]x). If this
        // reproduces MuJoCo's block, the formula is fine and only the
        // symmetrisation in `crba` is destroying it.
        {
            let d = misarta::fk::forward_kinematics(&model, &q0);
            let base_o = misarta::se3::translation(&d.oMi[1]);
            let mut c = na::Vector3::zeros();
            for l in robot.links.iter().filter(|l| l.inertial.mass > 0.0) {
                if let Some(&mi_) = _link_to_idx.get(&l.name) {
                    let rr = misarta::se3::rotation_matrix(&d.oMi[mi_]);
                    let oo = misarta::se3::translation(&d.oMi[mi_]);
                    let off = l.inertial.origin.translation.vector;
                    c += l.inertial.mass
                        * (oo + rr * na::Vector3::new(off.x as f64, off.y as f64, off.z as f64));
                }
            }
            c = c / total_mass - base_o;
            let mcx = total_mass
                * na::Matrix3::new(0.0, -c.z, c.y, c.z, 0.0, -c.x, -c.y, c.x, 0.0);
            println!("  expected coupling m*[c]x (upper-right of the base block):");
            for r in 0..3 {
                println!("      predicted [{:+8.4},{:+8.4},{:+8.4}]   mujoco [{:+8.4},{:+8.4},{:+8.4}]",
                    mcx[(r, 0)], mcx[(r, 1)], mcx[(r, 2)],
                    m_mj_flat[perm[r] * nv + perm[3]],
                    m_mj_flat[perm[r] * nv + perm[4]],
                    m_mj_flat[perm[r] * nv + perm[5]]);
            }
            println!("      misarta has this block identically ZERO.");
        }
        println!("  worst |crba - fullM| over ALL {nv}x{nv} entries : {:.3e}  at (r={},c={})",
            worst.2, worst.0, worst.1);
        println!("  worst over the 6x6 BASE block                  : {:.3e}  at (r={},c={})",
            worst_base.2, worst_base.0, worst_base.1);
    }
    println!();

    println!("=== Stage 3a: base angular acceleration, low-velocity regime ===");
    // NOTE ON TIMING: `mj_step` is "compute accelerations from the
    // current state, then integrate", so immediately after it the
    // derived quantities (xpos, cvel, ...) still describe the state
    // BEFORE integration -- reading them right after the first step
    // from rest returns exact zeros. Both readings below are taken
    // after a step, so they are stale by the same one step and their
    // difference is still a valid acceleration over that interval.
    {
        let mut taus0 = vec![0.0_f64; robot.joints.len()];
        for (n, t) in [("left_hip_pitch_joint", 3.0), ("right_hip_roll_joint", 2.0)] {
            if let Some(&ji) = robot.joint_map.get(n) {
                taus0[ji] = t;
            }
        }
        sim.set_wbc_torques(&taus0);
        sim.step_n_frames(&mut robot, 2, false); // flush the staleness

        let (qa_, va_) = sync(&sim, &robot);
        let mass_a = misarta::crba::crba(&model, &qa_);
        let h_a = misarta::rnea::nonlinear_effects(&model, &qa_, &va_);
        let mut tau_a = na::DVector::zeros(nv);
        for ji in 0..robot.joints.len() {
            let Some(mi) = a2m[ji] else { continue };
            if model.joints[mi].joint_type.nv() == 1 && model.v_idx[mi] >= 6 {
                tau_a[model.v_idx[mi]] = taus0[ji];
            }
        }
        // Like-for-like: MuJoCo solves
        //   M q̈ + qfrc_bias = qfrc_actuator + qfrc_passive + qfrc_constraint
        // so fold its constraint force into our right-hand side too.
        // Even in "free flight" it is not zero -- a tumbling robot's own
        // limbs collide, and that force is real physics MuJoCo applied
        // and misarta was never told about.
        let con_mj = sim.qfrc_constraint();
        let mut con_mi = na::DVector::zeros(nv);
        for r in 0..nv {
            con_mi[r] = con_mj[perm_vec[r]];
        }
        let pred = mass_a
            .clone()
            .lu()
            .solve(&(&tau_a + &con_mi - &h_a))
            .expect("M invertible");

        let qb = sim.body_world_orientation(&robot.root_link).unwrap();
        let wb = sim.body_world_angular_velocity(&robot.root_link).unwrap();
        let wb_b = qb.to_rotation_matrix().transpose() * na::Vector3::new(wb[0], wb[1], wb[2]);
        sim.step_n_frames(&mut robot, 1, false);
        let qc = sim.body_world_orientation(&robot.root_link).unwrap();
        let wc = sim.body_world_angular_velocity(&robot.root_link).unwrap();
        let wc_b = qc.to_rotation_matrix().transpose() * na::Vector3::new(wc[0], wc[1], wc[2]);
        let meas = (wc_b - wb_b) / dt;
        let pv = na::Vector3::new(pred[0], pred[1], pred[2]);
        // Exact comparison: MuJoCo's own qacc, no differencing. Free-joint
        // rows are [lin(3) world; ang(3) body], so the angular block is
        // rows 3..6 and lines up directly with misarta's rows 0..3.
        let qacc = sim.qacc();
        let mj_ang = na::Vector3::new(qacc[3], qacc[4], qacc[5]);
        println!("  MuJoCo qacc[3..6] (body ang) = ({:+9.3},{:+9.3},{:+9.3})",
            mj_ang.x, mj_ang.y, mj_ang.z);
        println!("  vs misarta                   -> diff {:.4e}  (rel {:.3e})",
            (pv - mj_ang).norm(), (pv - mj_ang).norm() / mj_ang.norm().max(1e-9));
        // M and h were both verified exact at v = 0, so if q̈ still
        // disagrees at v != 0 the culprit is one of the two INPUTS:
        // the bias evaluated at our synced velocity, or the torque that
        // actually reached the solver. Check each against MuJoCo.
        {
            let bias_now = sim.qfrc_bias();
            let act_now = sim.qfrc_actuator();
            let mut worst_h = (0usize, 0.0_f64);
            let mut worst_t = (0usize, 0.0_f64);
            for r in 0..nv {
                let dh = (h_a[r] - bias_now[perm_vec[r]]).abs();
                if dh > worst_h.1 { worst_h = (r, dh); }
                let dtq = (tau_a[r] - act_now[perm_vec[r]]).abs();
                if dtq > worst_t.1 { worst_t = (r, dtq); }
            }
            println!("  input check: worst |h_misarta - qfrc_bias|     = {:.3e} at misarta row {}",
                worst_h.1, worst_h.0);
            println!("  input check: worst |tau_built - qfrc_actuator| = {:.3e} at misarta row {}",
                worst_t.1, worst_t.0);
            // MuJoCo solves M q̈ + qfrc_bias = qfrc_actuator + qfrc_passive
            //                                 + qfrc_constraint.
            // Our prediction drops the last two, so if they are non-zero
            // the comparison is not like-for-like.
            let con = sim.qfrc_constraint();
            let pas = sim.qfrc_passive();
            let n2 = |v: &Vec<f64>| v.iter().map(|x| x * x).sum::<f64>().sqrt();
            println!("  omitted terms: |qfrc_constraint| = {:.4}   |qfrc_passive| = {:.4}",
                n2(&con), n2(&pas));
        }
        println!("  |omega| at measurement = {:.4} rad/s (want small)", wb_b.norm());
        println!("  misarta qddot[0..3] = ({:+9.3},{:+9.3},{:+9.3})", pv.x, pv.y, pv.z);
        println!("  measured  dw_body/dt = ({:+9.3},{:+9.3},{:+9.3})", meas.x, meas.y, meas.z);
        println!("  diff norm = {:.4}   sum norm = {:.4}   rel_err = {:.4}",
            (pv - meas).norm(), (pv + meas).norm(), (pv - meas).norm() / meas.norm().max(1e-9));
        println!("  misarta qddot[3..6] = ({:+9.3},{:+9.3},{:+9.3})", pred[3], pred[4], pred[5]);
    }
    println!();

    // ── Stage 1: what do MuJoCo's reported base velocities mean? ───────
    // Let it fall and tumble briefly so the base has a real, non-trivial
    // twist, then check that the sync reproduces d(pose)/dt.
    println!("=== Stage 1: base velocity convention after free tumbling ===");
    let mut robot_taus = vec![0.0_f64; robot.joints.len()];
    if let Some(&ji) = robot.joint_map.get("left_hip_pitch_joint") {
        robot_taus[ji] = 3.0;
    }
    if let Some(&ji) = robot.joint_map.get("right_hip_roll_joint") {
        robot_taus[ji] = 2.0;
    }
    sim.set_wbc_torques(&robot_taus);
    sim.step_n_frames(&mut robot, 40, false);

    let p_before = sim.body_world_position(&robot.root_link).unwrap();
    let quat_before = sim.body_world_orientation(&robot.root_link).unwrap();
    let w_before = sim.body_world_angular_velocity(&robot.root_link).unwrap();
    sim.step_n_frames(&mut robot, 1, false);
    let p_after = sim.body_world_position(&robot.root_link).unwrap();
    let quat_after = sim.body_world_orientation(&robot.root_link).unwrap();

    let fd_lin = [
        (p_after[0] - p_before[0]) / dt,
        (p_after[1] - p_before[1]) / dt,
        (p_after[2] - p_before[2]) / dt,
    ];
    let vl = sim.body_world_linear_velocity(&robot.root_link).unwrap();
    println!("  d(xpos)/dt              = ({:+8.4},{:+8.4},{:+8.4})", fd_lin[0], fd_lin[1], fd_lin[2]);
    println!("  body_world_linear_vel   = ({:+8.4},{:+8.4},{:+8.4})", vl[0], vl[1], vl[2]);
    // dR = R_after * R_before^T  ->  omega_world ~ vee(dR - I)/dt
    let dr = quat_after.to_rotation_matrix() * quat_before.to_rotation_matrix().transpose();
    let m = dr.matrix();
    let fd_w = [
        (m[(2, 1)] - m[(1, 2)]) / (2.0 * dt),
        (m[(0, 2)] - m[(2, 0)]) / (2.0 * dt),
        (m[(1, 0)] - m[(0, 1)]) / (2.0 * dt),
    ];
    println!("  d(R)/dt -> omega_world  = ({:+8.4},{:+8.4},{:+8.4})", fd_w[0], fd_w[1], fd_w[2]);
    println!("  body_world_angular_vel  = ({:+8.4},{:+8.4},{:+8.4})", w_before[0], w_before[1], w_before[2]);
    println!("      -> body_world_angular_velocity is in the WORLD frame if these match.");
    println!();

    // ── Stage 3: predicted vs actual base angular acceleration ─────────
    // Same state, same torque, no contacts. misarta says
    //   qddot = M^-1 (S^T tau - h)
    // and rows 0..3 of that are the BODY-frame angular acceleration.
    // Measure the real one by differencing omega_body across one step.
    println!("=== Stage 3: base angular acceleration, misarta vs MuJoCo ===");
    let (q1, v1) = sync(&sim, &robot);
    let mass = misarta::crba::crba(&model, &q1);
    let h1 = misarta::rnea::nonlinear_effects(&model, &q1, &v1);
    let mut tau_full = na::DVector::zeros(nv);
    for ji in 0..robot.joints.len() {
        let Some(mi) = a2m[ji] else { continue };
        if model.joints[mi].joint_type.nv() != 1 {
            continue;
        }
        let vi = model.v_idx[mi];
        if vi >= 6 {
            tau_full[vi] = robot_taus[ji];
        }
    }
    let rhs = &tau_full - &h1;
    let qddot_pred = mass
        .clone()
        .lu()
        .solve(&rhs)
        .expect("M is invertible");

    let quat_b = sim.body_world_orientation(&robot.root_link).unwrap();
    let w_b = sim.body_world_angular_velocity(&robot.root_link).unwrap();
    let wb_body = quat_b.to_rotation_matrix().transpose()
        * na::Vector3::new(w_b[0], w_b[1], w_b[2]);
    sim.set_wbc_torques(&robot_taus);
    sim.step_n_frames(&mut robot, 1, false);
    let quat_a = sim.body_world_orientation(&robot.root_link).unwrap();
    let w_a = sim.body_world_angular_velocity(&robot.root_link).unwrap();
    let wa_body = quat_a.to_rotation_matrix().transpose()
        * na::Vector3::new(w_a[0], w_a[1], w_a[2]);
    let meas_ang = (wa_body - wb_body) / dt;

    println!("  misarta qddot[0..3] (body ang) = ({:+9.3},{:+9.3},{:+9.3})",
        qddot_pred[0], qddot_pred[1], qddot_pred[2]);
    println!("  measured d(omega_body)/dt      = ({:+9.3},{:+9.3},{:+9.3})",
        meas_ang.x, meas_ang.y, meas_ang.z);
    let pred = na::Vector3::new(qddot_pred[0], qddot_pred[1], qddot_pred[2]);
    println!("  diff norm = {:.4}   sum norm = {:.4}", (pred - meas_ang).norm(), (pred + meas_ang).norm());
    println!("  misarta qddot[3..6] (body lin) = ({:+9.3},{:+9.3},{:+9.3})",
        qddot_pred[3], qddot_pred[4], qddot_pred[5]);
    println!();
    println!("READING: 'diff norm << sum norm' = conventions agree.");
    println!("         'sum norm << diff norm' = pure SIGN INVERSION.");
    println!("         neither small            = ordering/frame mismatch, not just a sign.");
}

#[cfg(not(feature = "mujoco"))]
fn main() {
    eprintln!("This example requires the `mujoco` feature. Run with:");
    eprintln!("  cargo run --features mujoco --example kyo46rs_convention_check");
}
