//! kyo46rs squat/balance on a **centroidal** formulation.
//!
//! Successor to kyo46rs_squat.rs. That file regulates the BASE's
//! roll/pitch/height with separate PD tasks on `qddot[0..6]`, and it
//! plateaus at ~1.1 s before toppling: attitude and height end up
//! competing for the same limited contact authority (raise the attitude
//! gain and tilt is held but z collapses; lower it and z holds but it
//! tips), and nothing at all regulates the horizontal position, so the
//! QP is free to walk all 65 N onto one foot and roll off it.
//!
//! Base attitude is the wrong thing to regulate. What decides whether a
//! biped falls is the CoM (equivalently the ZMP / capture point): the
//! base's 6 DOF are unactuated, so the only handle on the CoM is the
//! contact wrench, bounded by unilaterality, friction and the CoP box.
//! So the task here is the CoM acceleration itself -- and the squat is
//! just a reference on its z component, which makes balance and squat
//! ONE task instead of two that fight.
//!
//! Two conventions worth stating, because both bit the predecessor:
//!
//! - The CoM Jacobian is assembled in the WORLD frame (misarta's
//!   `compute_joint_jacobian` is world-frame with row order
//!   `[angular(3); linear(3)]`), so the CoM task needs no body-frame
//!   conversion at all. That also sidesteps kyo46rs_squat.rs's
//!   unexplained attitude sign inversion, which only ever appeared on
//!   the body-frame `qddot[0..2]` rows.
//! - Trunk orientation likewise uses the trunk's world-frame angular
//!   Jacobian rather than raw `qddot[0..2]`, for the same reason.
//!
//! `J_com` is verified numerically every tick against a finite-difference
//! of the measured CoM (`COMCHK=1`) rather than trusted.
//!
//! Run with: `cargo run --features mujoco --example kyo46rs_com_squat`

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

    let env_f64 = |k: &str, d: f64| -> f64 {
        std::env::var(k).ok().and_then(|s| s.parse().ok()).unwrap_or(d)
    };
    let flag = |k: &str, d: bool| -> bool {
        std::env::var(k).map(|v| v != "0").unwrap_or(d)
    };

    // ── Crouch seed (hip+knee+ankle must sum to 0 for a flat sole) ─────
    let hip_p = env_f64("HIP_PITCH", -0.35);
    let knee_q = env_f64("KNEE", 0.70);
    let ankle_p = env_f64("ANKLE_PITCH", -(0.70 - 0.35));

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
    let q_seed: Vec<f64> = robot.joint_positions.clone();

    // EL05 placeholders + burn-in PD. kv must stay <= ~2 at dt=1 ms: the
    // per-joint PD is explicit velocity feedback, stable only while
    // kv < 2*I/dt, and the distal roll joints have I ~ 6e-4 kg*m^2.
    // Measured threshold (kyo46rs_stand_check.rs, position control only):
    // kv <= 2.0 stands 5 s, kv = 3.0 collapses at 0.65 s.
    const EL05_JOINT_DAMPING: f64 = 0.15;
    const EL05_ARMATURE: f64 = 0.0005;
    let burnin_kp = env_f64("BURNIN_KP", 150.0);
    let burnin_kv = env_f64("BURNIN_KV", 2.0);
    let burnin_s = env_f64("BURNIN_S", 1.2);
    for j in robot.joints.iter_mut() {
        j.actuator_mode = ActuatorMode::Position;
        j.actuator_kp = burnin_kp;
        j.actuator_kv = burnin_kv;
        j.joint_damping = EL05_JOINT_DAMPING;
        j.armature = EL05_ARMATURE;
    }

    // ── Spawn so the soles just touch: measured, not hand-derived ──────
    const SOLE_BELOW_FOOT_ORIGIN: f64 = 0.035;
    // Fore/aft centre of the sole in the foot link frame. MUST match the
    // URDF's foot collision box origin.
    const SOLE_CENTRE_X: f64 = 0.0;
    const SOLE_CLEARANCE: f64 = 0.001;
    let sim_dt = env_f64("SIM_DT", 0.001);
    let make_opts = |z: f64| MjcfExportOptions {
        base_pos: Some([0.0, 0.0, z]),
        ground_plane: Some(GroundPlaneCfg { z: 0.0, half_size: 2.0, roll: 0.0, pitch: 0.0 }),
        timestep: Some(sim_dt),
        ..MjcfExportOptions::default()
    };
    let probe_z = 0.47;
    let spawn_z = {
        let probe = MujocoSim::new(&robot, make_opts(probe_z)).expect("probe sim");
        let f = probe.body_world_position("left_foot_link").expect("foot")[2];
        probe_z - ((f - SOLE_BELOW_FOOT_ORIGIN) - SOLE_CLEARANCE)
    };
    {
        // Prove the base is genuinely FREE and the ground is real, rather
        // than trusting MjcfExportOptions::default(). Several sibling
        // examples in this directory deliberately weld the torso
        // (base_locked_axes: [true; 6]) and it would be easy to confuse
        // a suspended rig's result for a standing one.
        let xml = articara::mjcf::export_mjcf_with_options(&robot, make_opts(spawn_z));
        let base_free = xml.contains("<freejoint/>");
        let has_ground = xml.contains(r#"type="plane""#);
        println!("rig check: freejoint={base_free}  ground_plane={has_ground}");
        assert!(base_free, "base is NOT free -- this would be a suspended rig, not standing");
        assert!(has_ground, "no ground plane -- the feet would have nothing to push on");
    }
    let mut sim = MujocoSim::new(&robot, make_opts(spawn_z)).expect("MujocoSim::new");
    let mj_dt = sim.timestep();

    let (model, a2m, link_to_idx) = build_floating_base_model(&robot);
    let nv = model.nv;
    let na_count = nv - 6;
    let left_foot_mi = *link_to_idx.get("left_foot_link").expect("left_foot_link");
    let right_foot_mi = *link_to_idx.get("right_foot_link").expect("right_foot_link");
    let trunk_mi = 1usize; // the FreeFlyer's own body

    // Links that carry mass, paired with their misarta index and the CoM
    // offset in the link frame -- everything `J_com` needs.
    struct MassLink {
        mi: usize,
        m: f64,
        com_local: na::Vector3<f64>,
    }
    let mass_links: Vec<MassLink> = robot
        .links
        .iter()
        .filter(|l| l.inertial.mass > 0.0)
        .filter_map(|l| {
            link_to_idx.get(&l.name).map(|&mi| {
                let o = l.inertial.origin.translation.vector;
                MassLink {
                    mi,
                    m: l.inertial.mass,
                    com_local: na::Vector3::new(o.x as f64, o.y as f64, o.z as f64),
                }
            })
        })
        .collect();
    let total_mass: f64 = mass_links.iter().map(|l| l.m).sum();
    const G: f64 = 9.81;
    println!(
        "centroidal model: nv={nv} na={na_count} mass_links={} total_mass={total_mass:.3} kg  dt={mj_dt}",
        mass_links.len()
    );

    let mut torque_max = na::DVector::from_element(na_count, 6.0);
    for ji in 0..robot.joints.len() {
        let Some(mi) = a2m[ji] else { continue };
        if model.joints[mi].joint_type.nv() != 1 {
            continue;
        }
        let vi = model.v_idx[mi];
        if vi >= 6 {
            torque_max[vi - 6] = robot.joints[ji].effort.max(1.0);
        }
    }

    // ── Settle with the base WELDED, then hand a clean pose over ───────
    // A free-standing position-controlled biped is laterally unstable:
    // it has no balance control, so any numerical asymmetry grows and the
    // whole robot slides sideways. Settling in place therefore lands the
    // CoM somewhere arbitrary — measured across burn-in lengths of
    // 0.3/0.6/0.9/1.2/1.6 s it drifted to com_y between -0.024 and
    // -0.090 m, and since the lateral support only spans +-0.089 m the
    // run's survival flipped with it. That makes every downstream
    // comparison a coin toss rather than a measurement.
    //
    // The model is symmetric, so the correct settled state is symmetric.
    // Split the two problems: settle the JOINTS against the floor with
    // the base held (the well-posed half), then start the real free-base
    // run from those angles, centred. Same two-pass idea as the spawn
    // probe above.
    {
        let mut settle_robot = robot.clone();
        let settle_opts = MjcfExportOptions {
            base_locked_axes: [true; 6],
            ..make_opts(spawn_z)
        };
        let mut settle = MujocoSim::new(&settle_robot, settle_opts).expect("settle sim");
        for ji in 0..settle_robot.joints.len() {
            settle.set_position_target(ji, q_seed[ji]);
        }
        settle.step_n_frames(&mut settle_robot, (burnin_s / mj_dt) as u32, true);
        for ji in 0..robot.joints.len() {
            robot.joint_positions[ji] = settle_robot.joint_positions[ji];
        }
        let worst = (0..robot.joints.len())
            .map(|ji| (robot.joint_positions[ji] - q_seed[ji]).abs())
            .fold(0.0_f64, f64::max);
        println!("settled (base welded) for {burnin_s}s: max joint move from seed = {worst:.4} rad");
    }
    // Re-spawn free-based at the settled pose, centred at the origin.
    let mut sim = MujocoSim::new(&robot, make_opts(spawn_z)).expect("MujocoSim::new (run)");
    for ji in 0..robot.joints.len() {
        sim.set_position_target(ji, robot.joint_positions[ji]);
    }
    // Brief hold so the contacts engage before torque control starts.
    sim.step_n_frames(&mut robot, (0.05 / mj_dt) as u32, true);
    for j in robot.joints.iter_mut() {
        j.actuator_mode = ActuatorMode::Torque;
    }
    {
        let hr = |n: &str| sim.joint_q_qd(n).map(|(q, _)| q).unwrap_or(f64::NAN);
        let lp = sim.body_world_position("left_foot_link").unwrap();
        let rp = sim.body_world_position("right_foot_link").unwrap();
        let rpy = sim.body_world_orientation(&robot.root_link).unwrap().euler_angles();
        println!(
            "post-burn-in: rpy=({:+.3},{:+.3},{:+.3}) hip_roll=({:+.3},{:+.3}) foot inner-gap={:+.4}",
            rpy.0, rpy.1, rpy.2,
            hr("left_hip_roll_joint"), hr("right_hip_roll_joint"),
            (lp[1] - 0.019) - (rp[1] + 0.019),
        );
    }

    let mut solver = Solver::new();
    let cfg = SolveConfig::default();
    const FRICTION_MU: f64 = 0.6;
    let kp_com = env_f64("KP_COM", 300.0);
    let kd_com = env_f64("KD_COM", 40.0);
    let kp_trunk = env_f64("KP_TRUNK", 200.0);
    let kd_trunk = env_f64("KD_TRUNK", 40.0);
    let kp_post = env_f64("KP_POST", 100.0);
    let kd_post = env_f64("KD_POST", 20.0);
    let use_post = flag("POST", true);
    let trunk_sign = env_f64("TRUNK_SIGN", 1.0);
    // Shrink the admissible CoP box to keep a margin: riding the exact
    // edge means the next disturbance makes P0 infeasible outright.
    let cop_frac = env_f64("COP_FRAC", 1.0);
    let com_sign = env_f64("COM_SIGN", 1.0);
    let comchk = flag("COMCHK", false);
    let period_s = env_f64("PERIOD_S", 3.0);
    let squat_amp = env_f64("AMP", 0.0); // 0 = hold still; >0 = squat
    // Single-support mode: shift the CoM over the left foot, then release
    // the right foot's contact and lift it. Loads hip_pitch/hip_roll the
    // way a squat never does, which is what the "does hip_pitch really
    // need two motors" question actually needs measuring against.
    let lift_leg = flag("LIFT", false);
    // Static fore/aft CoM offset, both feet down. Loads hip_pitch the way
    // stance does, and stays inside a regime the QP can actually solve.
    let com_dx = env_f64("COM_DX", 0.0);
    let t_shift = env_f64("T_SHIFT", 3.0);   // seconds spent moving the CoM across
    let lift_h = env_f64("LIFT_H", 0.04);    // swing-foot clearance, m
    let kp_sw = env_f64("KP_SWING", 400.0);
    let kd_sw = env_f64("KD_SWING", 40.0);
    let total_t = env_f64("T", 6.0);

    let mj_substeps = (0.005 / mj_dt).round().max(1.0) as u32;
    let dt = mj_substeps as f64 * mj_dt;
    let n_ticks = (total_t / dt) as usize;

    // Helper: world-frame CoM position from an FK snapshot.
    let com_of = |data: &misarta::data::Data<f64>| -> na::Vector3<f64> {
        let mut c = na::Vector3::zeros();
        for l in &mass_links {
            let r = misarta::se3::rotation_matrix(&data.oMi[l.mi]);
            let o = misarta::se3::translation(&data.oMi[l.mi]);
            c += l.m * (o + r * l.com_local);
        }
        c / total_mass
    };

    // ── Where does the CoM actually sit inside the footprint? ──────────
    // The fore/aft split of the sole about the ankle sets how far the
    // centre of pressure can travel each way, and that only helps if it
    // is matched to which way the robot actually tends to fall. Copying
    // the human 25/75 split assumes a human's stance, where the CoM sits
    // well forward of the ankle; it is the wrong trade if this robot's
    // CoM sits level with or behind its ankles.
    {
        let d0 = misarta::fk::forward_kinematics(&model, &{
            let p = sim.body_world_position(&robot.root_link).unwrap();
            let qq = sim.body_world_orientation(&robot.root_link).unwrap();
            let mut q = model.neutral_q();
            q[0] = p[0]; q[1] = p[1]; q[2] = p[2];
            q[3] = qq.i; q[4] = qq.j; q[5] = qq.k; q[6] = qq.w;
            for ji in 0..robot.joints.len() {
                if let Some(mi) = a2m[ji] {
                    if model.joints[mi].joint_type.nq() == 1 {
                        q[model.q_idx[mi]] = robot.joint_positions[ji];
                    }
                }
            }
            q
        });
        let com0 = com_of(&d0);
        let ankle_x = 0.5
            * (misarta::se3::translation(&d0.oMi[left_foot_mi]).x
                + misarta::se3::translation(&d0.oMi[right_foot_mi]).x);
        let (cx, half) = (SOLE_CENTRE_X, 0.049);
        let (back, front) = (ankle_x + cx - half, ankle_x + cx + half);
        println!(
            "footprint: ankle x={ankle_x:+.4}  sole x=[{back:+.4},{front:+.4}]  CoM x={:+.4}",
            com0.x
        );
        println!(
            "  CoM is {:+.1} mm relative to the ankle;  margin back {:.1} mm / front {:.1} mm",
            (com0.x - ankle_x) * 1000.0,
            (com0.x - back) * 1000.0,
            (front - com0.x) * 1000.0
        );
        let centred_cx = com0.x - ankle_x;
        println!(
            "  sole centre that would put the CoM mid-footprint: x = {centred_cx:+.4} (currently {cx:+.4})"
        );
    }


    // Trajectory log for offline rendering (same column layout
    // kyo46rs_squat.rs uses, so the replay tooling is shared).
    let log_joint_order: Vec<&str> = vec![
        "left_hip_yaw_joint", "left_hip_roll_joint", "left_hip_pitch_joint",
        "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
        "right_hip_yaw_joint", "right_hip_roll_joint", "right_hip_pitch_joint",
        "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
        "left_shoulder_pitch_joint", "left_elbow_joint",
        "right_shoulder_pitch_joint", "right_elbow_joint",
    ];
    let mut log_file = std::env::var("TRAJ_CSV").ok().map(|path| {
        use std::io::Write;
        let mut f = std::fs::File::create(&path).expect("create trajectory log");
        write!(f, "t,x,y,z,qw,qx,qy,qz,com_x,com_y,com_z,com_ref_z,tilt,com_ref_y,n_stance,swing_z").unwrap();
        for n in &log_joint_order {
            write!(f, ",{n}").unwrap();
        }
        // WBC-commanded torque per joint, plus the joint's effort limit, so
        // a replay can show demand against capability and make saturation
        // visible rather than silently clipped.
        for n in &log_joint_order {
            write!(f, ",tau_{n}").unwrap();
        }
        for n in &log_joint_order {
            write!(f, ",lim_{n}").unwrap();
        }
        writeln!(f).unwrap();
        println!("logging trajectory to {path}");
        f
    });

    let mut com_ref0: Option<na::Vector3<f64>> = None;
    let mut swing_home_cell: Option<na::Vector3<f64>> = None;
    let mut prev_com: Option<na::Vector3<f64>> = None;
    let mut prev_body_pos: Option<[f64; 3]> = None;
    let mut n_degraded = 0u32;
    let mut fell = false;
    let mut min_z = f64::INFINITY;
    let mut max_tilt: f64 = 0.0;
    let mut max_jcom_err: f64 = 0.0;

    for tick in 0..n_ticks {
        let t = tick as f64 * dt;

        // ---- sync state ------------------------------------------------
        let body_pos = sim.body_world_position(&robot.root_link).unwrap();
        let body_quat = sim.body_world_orientation(&robot.root_link).unwrap();
        let v_lin_w = sim.body_world_linear_velocity(&robot.root_link).unwrap();
        let v_ang_w = sim.body_world_angular_velocity(&robot.root_link).unwrap();
        let r_wb = body_quat.to_rotation_matrix();
        let r_bw = r_wb.transpose();
        let v_lin_body = r_bw * na::Vector3::new(v_lin_w[0], v_lin_w[1], v_lin_w[2]);
        let v_ang_body = r_bw * na::Vector3::new(v_ang_w[0], v_ang_w[1], v_ang_w[2]);

        // Is `body_world_linear_velocity` actually the velocity of the
        // body ORIGIN (what `body_world_position` reports)? It reads
        // MuJoCo's `cvel`, whose linear part is expressed in the c-frame
        // -- world-aligned axes but origin at the subtree CoM, not at
        // xpos. If so it differs from d(xpos)/dt by omega x (xpos - com).
        if flag("VELCHK", false) {
            if let Some(pp) = prev_body_pos {
                let fd: [f64; 3] = [
                    (body_pos[0] - pp[0]) / dt,
                    (body_pos[1] - pp[1]) / dt,
                    (body_pos[2] - pp[2]) / dt,
                ];
                if tick % 20 == 0 {
                    println!(
                        "  [velchk] d(xpos)/dt=({:+.4},{:+.4},{:+.4})  cvel_lin=({:+.4},{:+.4},{:+.4})",
                        fd[0], fd[1], fd[2], v_lin_w[0], v_lin_w[1], v_lin_w[2]
                    );
                }
            }
        }
        prev_body_pos = Some(body_pos);

        let mut q = model.neutral_q();
        q[0] = body_pos[0];
        q[1] = body_pos[1];
        q[2] = body_pos[2];
        q[3] = body_quat.i;
        q[4] = body_quat.j;
        q[5] = body_quat.k;
        q[6] = body_quat.w;
        let mut v = vec![0.0_f64; nv];
        // FreeFlyer motion subspace is I6 in the BODY frame with row
        // order [angular; linear] (misarta joint.rs), so v[0..3] is
        // omega_body and v[3..6] is v_body.
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
                    v[model.v_idx[mi]] = qd.clamp(-5.0, 5.0);
                }
            }
        }
        let v_dvec = na::DVector::from_column_slice(&v);

        let mass = misarta::crba::crba(&model, &q);
        let h = misarta::rnea::nonlinear_effects(&model, &q, &v);
        let data = misarta::fk::forward_kinematics(&model, &q);

        // ---- CoM position, Jacobian and bias, all world-frame ----------
        // Per link: take its parent joint's world Jacobian (rows 0..3 =
        // angular, 3..6 = linear, at the JOINT origin) and shift the
        // linear part out to the link's own CoM,
        //     v_c = v_o + omega x r      =>  J_lin_c = J_lin_o - [r]x J_ang
        // then mass-average. The bias picks up the centripetal term:
        //     dJv_lin_c = dJv_lin_o - [r]x dJv_ang + omega x (omega x r)
        let com = com_of(&data);
        let mut j_com = na::DMatrix::zeros(3, nv);
        let mut djv_com = na::Vector3::zeros();
        for l in &mass_links {
            let rot = misarta::se3::rotation_matrix(&data.oMi[l.mi]);
            let r = rot * l.com_local;
            let skew = na::Matrix3::new(
                0.0, -r.z, r.y,
                r.z, 0.0, -r.x,
                -r.y, r.x, 0.0,
            );
            let j = misarta::jacobian::compute_joint_jacobian_from_data(&model, &q, &data, l.mi);
            let j_ang = j.rows(0, 3).into_owned();
            let j_lin = j.rows(3, 3).into_owned();
            let j_lin_c = &j_lin - &skew * &j_ang;
            j_com += l.m * j_lin_c;

            let dj = misarta::jacobian::compute_joint_jacobian_time_derivative(&model, &q, &v, l.mi);
            let djv = &dj * &v_dvec;
            let djv_ang = na::Vector3::new(djv[0], djv[1], djv[2]);
            let djv_lin = na::Vector3::new(djv[3], djv[4], djv[5]);
            let omega = &j_ang * &v_dvec;
            let omega = na::Vector3::new(omega[0], omega[1], omega[2]);
            djv_com += l.m * (djv_lin - skew * djv_ang + omega.cross(&omega.cross(&r)));
        }
        j_com /= total_mass;
        djv_com /= total_mass;

        // One-shot column-wise check of J_com against finite differences
        // on the joint coordinates (the base columns need quaternion
        // integration, so they are checked via the running J*v vs
        // d(com)/dt comparison below instead).
        if tick == 0 && flag("COLCHK", false) {
            const EPS: f64 = 1e-6;
            let mut worst = (0usize, 0.0_f64, String::new());
            for ji in 0..robot.joints.len() {
                let Some(mi) = a2m[ji] else { continue };
                if model.joints[mi].joint_type.nv() != 1 {
                    continue;
                }
                let (qi, vi) = (model.q_idx[mi], model.v_idx[mi]);
                if vi < 6 {
                    continue;
                }
                let mut qp = q.clone();
                qp[qi] += EPS;
                let fd = (com_of(&misarta::fk::forward_kinematics(&model, &qp)) - com) / EPS;
                let col = na::Vector3::new(j_com[(0, vi)], j_com[(1, vi)], j_com[(2, vi)]);
                let e = (fd - col).norm();
                if e > worst.1 {
                    worst = (vi, e, robot.joints[ji].name.clone());
                }
                if e > 1e-4 {
                    println!(
                        "  [colchk] {:<28} v{vi}: fd=({:+.5},{:+.5},{:+.5}) J=({:+.5},{:+.5},{:+.5}) err={e:.2e}",
                        robot.joints[ji].name, fd.x, fd.y, fd.z, col.x, col.y, col.z
                    );
                }
            }
            println!("  [colchk] worst joint column: {} (v{}) err={:.3e}", worst.2, worst.0, worst.1);
        }

        let com_vel = &j_com * &v_dvec;
        let com_vel = na::Vector3::new(com_vel[0], com_vel[1], com_vel[2]);

        // Verify J_com against a finite difference of the measured CoM
        // rather than trusting the shift algebra.
        if let Some(pc) = prev_com {
            let fd = (com - pc) / dt;
            let err = (fd - com_vel).norm() / fd.norm().max(1e-3);
            max_jcom_err = max_jcom_err.max(err);
            if comchk && tick % 20 == 0 {
                println!(
                    "  [Jcom] fd=({:+.4},{:+.4},{:+.4})  J*v=({:+.4},{:+.4},{:+.4})  rel_err={err:.4}",
                    fd.x, fd.y, fd.z, com_vel.x, com_vel.y, com_vel.z
                );
            }
        }
        prev_com = Some(com);
        let com_ref0 = *com_ref0.get_or_insert(com);
        // Freeze the swing foot's start pose on the first tick so the lift
        // target does not chase the foot as it moves.
        let swing_home =
            *swing_home_cell.get_or_insert(misarta::se3::translation(&data.oMi[right_foot_mi]));

        // ---- contacts --------------------------------------------------
        // In single-support the right foot leaves the ground, so it must
        // also leave the contact set -- keeping its rows would have the QP
        // solve against a reaction force that no longer exists.
        let single = lift_leg && t >= t_shift;
        let stance: Vec<usize> = if single {
            vec![left_foot_mi]
        } else {
            vec![left_foot_mi, right_foot_mi]
        };
        let nc = stance.len();
        let mut j_contact = na::DMatrix::zeros(6 * nc, nv);
        let mut dj_v = na::DVector::zeros(6 * nc);
        for (slot, foot_mi) in stance.iter().copied().enumerate() {
            let jf = misarta::jacobian::compute_joint_jacobian_from_data(&model, &q, &data, foot_mi);
            let djf = misarta::jacobian::compute_joint_jacobian_time_derivative(&model, &q, &v, foot_mi);
            let djv = &djf * &v_dvec;
            for r in 0..6 {
                for c in 0..nv {
                    j_contact[(6 * slot + r, c)] = jf[(r, c)];
                }
                dj_v[6 * slot + r] = djv[r];
            }
        }

        let dyn_ctx = Dynamics::new(Formulation::Explicit, &mass, &h, &j_contact, na_count);
        let forces = dyn_ctx.forces();

        // patch_contact's CoP box is only the real centre-of-pressure
        // condition about the SOLE in the sole's frame; `forces` is about
        // the foot LINK ORIGIN, 0.059 m higher, where a tangential fx
        // fakes 0.059*fx of moment. Transform before constraining.
        // MUST track the URDF's foot collision box. The CoP box is
        // centred on this point, so a stale value silently constrains
        // the pressure centre about the wrong place -- moving the sole
        // in the model and forgetting this makes the QP defend a
        // footprint the robot no longer has.
        const SOLE_OFFSET_LOCAL: [f64; 3] = [SOLE_CENTRE_X, 0.0, -0.035];
        let sole_patch = tasks::ContactPatch {
            mu: FRICTION_MU,
            cop_half: (0.049 * cop_frac, 0.019 * cop_frac),
            mu_torsion: 0.05,
            f_max: 150.0,
        };
        let mut p0 = dyn_ctx
            .dynamics_task()
            .expect("Explicit keeps the EoM task")
            + tasks::box_bound(dyn_ctx.tau(), &torque_max);
        for (slot, foot_mi) in stance.iter().copied().enumerate() {
            let js = j_contact.rows(6 * slot, 6).into_owned();
            let djvs = dj_v.rows(6 * slot, 6).into_owned();
            let rot = misarta::se3::rotation_matrix(&data.oMi[foot_mi]);
            let r_w = rot
                * na::Vector3::new(SOLE_OFFSET_LOCAL[0], SOLE_OFFSET_LOCAL[1], SOLE_OFFSET_LOCAL[2]);
            let rt = rot.transpose();
            let skew = na::Matrix3::new(
                0.0, -r_w.z, r_w.y,
                r_w.z, 0.0, -r_w.x,
                -r_w.y, r_w.x, 0.0,
            );
            let top_right = -(rt * skew);
            let mut sel = na::DMatrix::zeros(6, forces.size());
            for i in 0..3 {
                for jj in 0..3 {
                    sel[(i, 6 * slot + jj)] = rt[(i, jj)];
                    sel[(i, 6 * slot + 3 + jj)] = top_right[(i, jj)];
                    sel[(3 + i, 6 * slot + 3 + jj)] = rt[(i, jj)];
                }
            }
            let w_sole = &sel * &forces.as_affine();
            p0 = p0
                + tasks::zero_contact_acceleration(dyn_ctx.qddot(), &js, &djvs)
                + tasks::patch_contact(&w_sole, &sole_patch);
        }

        // ---- P1: the CoM task = balance (x,y) AND squat (z) ------------
        let phase = 2.0 * PI * t / period_s;
        let z_ref = com_ref0.z - squat_amp * (1.0 - phase.cos()) * 0.5;
        let zd_ref = -squat_amp * 0.5 * (2.0 * PI / period_s) * phase.sin();
        let zdd_ref = -squat_amp * 0.5 * (2.0 * PI / period_s).powi(2) * phase.cos();
        // Move the CoM over the stance foot BEFORE releasing the other one.
        let y_ref = if lift_leg {
            let stance_y = misarta::se3::translation(&data.oMi[left_foot_mi]).y;
            let a = (t / t_shift).clamp(0.0, 1.0);
            let a = 0.5 - 0.5 * (PI * a).cos();          // smooth ramp
            com_ref0.y + a * (stance_y - com_ref0.y)
        } else {
            com_ref0.y
        };
        let lean = com_dx * (t / 2.0).clamp(0.0, 1.0);   // ramp in over 2 s
        let c_ref = na::Vector3::new(com_ref0.x + lean, y_ref, z_ref);
        let cd_ref = na::Vector3::new(0.0, 0.0, zd_ref);
        let cdd_ref = na::Vector3::new(0.0, 0.0, zdd_ref);
        let a_com = com_sign * (cdd_ref + kd_com * (cd_ref - com_vel) + kp_com * (c_ref - com));
        let com_accel_ref = na::DVector::from_vec(vec![a_com.x, a_com.y, a_com.z]);
        let p1 = tasks::cartesian_acceleration(
            dyn_ctx.qddot(),
            &j_com,
            &na::DVector::from_vec(vec![djv_com.x, djv_com.y, djv_com.z]),
            &com_accel_ref,
        );

        // ---- P2: trunk upright, via the WORLD-frame angular Jacobian ---
        // Not qddot[0..2]: those are body-frame and carry the sign
        // inversion kyo46rs_squat.rs never explained. Rows 0..3 of the
        // trunk's own world Jacobian map qddot to world angular
        // acceleration, which matches the world-frame roll/pitch error.
        let j_trunk = misarta::jacobian::compute_joint_jacobian_from_data(&model, &q, &data, trunk_mi);
        let dj_trunk = misarta::jacobian::compute_joint_jacobian_time_derivative(&model, &q, &v, trunk_mi);
        let djv_trunk = &dj_trunk * &v_dvec;
        let (roll, pitch, _yaw) = body_quat.euler_angles();
        let mut j_rp = na::DMatrix::zeros(2, nv);
        for c in 0..nv {
            j_rp[(0, c)] = j_trunk[(0, c)];
            j_rp[(1, c)] = j_trunk[(1, c)];
        }
        // Same unexplained inversion kyo46rs_squat.rs hit, and it
        // SURVIVES the cvel fix: re-tested with the base velocity now
        // correct, +1.0 still diverges (0.185 s) and -1.0 still does not
        // (0.525 s). So it is an independent angular-convention
        // mismatch between misarta's model and MuJoCo, not a knock-on of
        // the velocity bug. The CoM task by contrast needs NO flip --
        // com_sign=-1 merely postpones the fall by turning the task into
        // slow positive feedback (CoM z drifts 0.2939 -> 0.3388 instead
        // of tracking), which is why it is not the default despite
        // "surviving" longer.
        let rp_ref = na::DVector::from_vec(vec![
            trunk_sign * (kp_trunk * (0.0 - roll) + kd_trunk * (0.0 - v_ang_w[0])),
            trunk_sign * (kp_trunk * (0.0 - pitch) + kd_trunk * (0.0 - v_ang_w[1])),
        ]);
        let p2 = tasks::cartesian_acceleration(
            dyn_ctx.qddot(),
            &j_rp,
            &na::DVector::from_vec(vec![djv_trunk[0], djv_trunk[1]]),
            &rp_ref,
        );

        // ---- P3: weak posture, so the null space does not wander -------
        let mut j_post = na::DMatrix::zeros(na_count, nv);
        let mut post_ref = na::DVector::zeros(na_count);
        for ji in 0..robot.joints.len() {
            let Some(mi) = a2m[ji] else { continue };
            if model.joints[mi].joint_type.nv() != 1 {
                continue;
            }
            let vi = model.v_idx[mi];
            if vi >= 6 {
                j_post[(vi - 6, vi)] = 1.0;
                post_ref[vi - 6] =
                    kp_post * (q_seed[ji] - robot.joint_positions[ji]) + kd_post * (0.0 - v[vi]);
            }
        }
        let p3 = tasks::cartesian_acceleration(
            dyn_ctx.qddot(),
            &j_post,
            &na::DVector::zeros(na_count),
            &post_ref,
        );

        // ---- lowest: gravity-comp torque + even weight split -----------
        let g_full = misarta::rnea::compute_gravity(&model, &q);
        let mut tau_gravity = na::DVector::zeros(na_count);
        for ji in 0..robot.joints.len() {
            let Some(mi) = a2m[ji] else { continue };
            if model.joints[mi].joint_type.nv() != 1 {
                continue;
            }
            let vi = model.v_idx[mi];
            if vi >= 6 {
                tau_gravity[vi - 6] = g_full[vi];
            }
        }
        let mut forces_nominal = na::DVector::zeros(forces.size());
        for slot in 0..nc {
            forces_nominal[6 * slot + 5] = total_mass * G / nc as f64;
        }
        let p_reg = tasks::regularize(dyn_ctx.tau(), &tau_gravity)
            + tasks::regularize(&dyn_ctx.forces(), &forces_nominal);

        // Swing foot: hold it at a clearance above where it started.
        let p_swing = if single {
            let jf = misarta::jacobian::compute_joint_jacobian_from_data(&model, &q, &data, right_foot_mi);
            let djf = misarta::jacobian::compute_joint_jacobian_time_derivative(&model, &q, &v, right_foot_mi);
            let djv = &djf * &v_dvec;
            let pos = misarta::se3::translation(&data.oMi[right_foot_mi]);
            let vel = &jf.rows(3, 3).into_owned() * &v_dvec;
            let tgt = swing_home + na::Vector3::new(0.0, 0.0, lift_h);
            let a = kp_sw * (tgt - pos) - kd_sw * na::Vector3::new(vel[0], vel[1], vel[2]);
            Some(tasks::cartesian_acceleration(
                dyn_ctx.qddot(),
                &jf.rows(3, 3).into_owned(),
                &na::DVector::from_vec(vec![djv[3], djv[4], djv[5]]),
                &na::DVector::from_vec(vec![a.x, a.y, a.z]),
            ))
        } else {
            None
        };

        let mut levels = vec![p0, p1, p2];
        if let Some(ps) = p_swing {
            levels.push(ps);
        }
        if use_post {
            levels.push(p3);
        }
        levels.push(p_reg);
        let sol = solver
            .solve(&levels, &cfg)
            .unwrap_or_else(|e| panic!("wbc solve failed at t={t:.3}: {e}"));
        if !matches!(sol.status, misa_wbc::SolveStatus::Optimal) {
            n_degraded += 1;
            if n_degraded <= 6 || tick % 200 == 0 {
                println!("    [degraded] t={t:6.3} nc={nc} status={:?}", sol.status);
            }
        }
        let extracted = dyn_ctx.extract(&sol.x);

        let mut robot_taus = vec![0.0_f64; robot.joints.len()];
        for ji in 0..robot.joints.len() {
            let Some(mi) = a2m[ji] else { continue };
            if model.joints[mi].joint_type.nv() != 1 {
                continue;
            }
            let vi = model.v_idx[mi];
            if vi >= 6 {
                robot_taus[ji] = extracted.tau[vi - 6];
            }
        }
        // NO_TORQUE=1 sends zeros: a free base on a real floor must
        // collapse. If it does not, something is holding the robot.
        if flag("NO_TORQUE", false) {
            robot_taus.iter_mut().for_each(|t| *t = 0.0);
        }
        sim.set_wbc_torques(&robot_taus);
        sim.step_n_frames(&mut robot, mj_substeps, true);

        if let Some(f) = log_file.as_mut() {
            use std::io::Write;
            let p = sim.body_world_position(&robot.root_link).unwrap();
            let qq = sim.body_world_orientation(&robot.root_link).unwrap();
            write!(
                f,
                "{t:.4},{:.5},{:.5},{:.5},{:.5},{:.5},{:.5},{:.5},{:.5},{:.5},{:.5},{z_ref:.5},{:.5}",
                p[0], p[1], p[2], qq.w, qq.i, qq.j, qq.k,
                com.x, com.y, com.z,
                roll.abs().max(pitch.abs())
            )
            .unwrap();
            let sw = sim.body_world_position("right_foot_link").unwrap()[2];
            write!(f, ",{y_ref:.5},{nc},{sw:.5}").unwrap();
            for n in &log_joint_order {
                let a = robot.joint_map.get(*n).map(|&ji| robot.joint_positions[ji]).unwrap_or(0.0);
                write!(f, ",{a:.5}").unwrap();
            }
            for n in &log_joint_order {
                let tq = robot.joint_map.get(*n).map(|&ji| robot_taus[ji]).unwrap_or(0.0);
                write!(f, ",{tq:.5}").unwrap();
            }
            for n in &log_joint_order {
                let lm = robot.joint_map.get(*n).map(|&ji| robot.joints[ji].effort).unwrap_or(0.0);
                write!(f, ",{lm:.3}").unwrap();
            }
            writeln!(f).unwrap();
        }

        let cur_z = sim.body_world_position(&robot.root_link).unwrap()[2];
        min_z = min_z.min(cur_z);
        let tilt = roll.abs().max(pitch.abs());
        max_tilt = max_tilt.max(tilt);
        if tick % 20 == 0 {
            println!(
                "  t={t:6.3}  com=({:+.4},{:+.4},{:+.4}) ref_z={z_ref:+.4}  roll={roll:+.3} pitch={pitch:+.3}  status={:?}",
                com.x, com.y, com.z, sol.status
            );
        }
        if cur_z < 0.30 || tilt > 0.52 {
            println!("  FELL at t={t:.3} (z={cur_z:.3}, tilt={tilt:.3})");
            fell = true;
            break;
        }
    }

    println!("\n=== Result (centroidal) ===");
    println!("  max |J_com*v - d(com)/dt| relative error: {max_jcom_err:.4}");
    println!("  min trunk z = {min_z:.3}   max tilt = {max_tilt:.3} rad");
    println!("  degraded solves: {n_degraded}");
    println!("  verdict: {}", if fell { "FELL" } else { "SURVIVED" });
}

#[cfg(not(feature = "mujoco"))]
fn main() {
    eprintln!("This example requires the `mujoco` feature. Run with:");
    eprintln!("  cargo run --features mujoco --example kyo46rs_com_squat");
}
