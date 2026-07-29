//! kyo46rs progressive N-joint controllability check: same welded-torso
//! rig as kyo46rs_single_joint_check.rs / kyo46rs_hanging_squat.rs, but
//! with a CONFIGURABLE set of jointly-tracked joints (`JOINT_SET` below)
//! so the same sinusoid-tracking test can be rerun at 1, 2, 3, 6, ...
//! active DOF to find where solver failures / tracking degradation
//! actually start appearing, rather than jumping straight from "1 joint,
//! zero failures" to "6 joints, periodic NumericalFailure".
//!
//! Edit JOINT_SET and rerun for each step of the sweep:
//!   1 joint:  ["left_knee_joint"]                                    (baseline, confirmed 0 failures)
//!   2 joints: ["left_hip_pitch_joint", "left_knee_joint"]
//!   3 joints: ["left_hip_pitch_joint", "left_knee_joint", "left_ankle_pitch_joint"]  (one full leg)
//!   6 joints: both legs' hip_pitch/knee/ankle_pitch                  (kyo46rs_hanging_squat.rs's case)
//!
//! Every joint NOT in JOINT_SET is held only by the P2 gravity-comp
//! regularizer (same as the single-joint test). All tracked joints
//! share the SAME sinusoid amplitude/period/gains for a fair, direct
//! comparison across sweep steps.
//!
//! Run with: `cargo run --features mujoco --example kyo46rs_njoint_check`

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

    // ── EDIT THIS to sweep 1 -> 2 -> 3 -> 6 joints ─────────────────────
    const JOINT_SET: &[&str] = &[
        "left_hip_pitch_joint", "left_knee_joint", "left_ankle_pitch_joint",
        "right_hip_pitch_joint", "right_knee_joint", "right_ankle_pitch_joint",
    ];

    let urdf_path = std::path::Path::new(
        "/home/takara/work/dp/humanoid/kyo46rs_description/urdf/kyo46rs.urdf",
    );
    let mut robot = RobotModel::from_urdf(urdf_path).expect("load kyo46rs.urdf");

    // Full crouch+arms seed (same as kyo46rs_hanging_squat.rs /
    // kyo46rs_fullbody_gravity_check.rs) regardless of which subset is
    // actively tracked -- untracked joints in this set fall back to
    // pure gravity-comp holding, already validated to work well here.
    let full_seed: [(&str, f64); 10] = [
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
    for (name, q) in full_seed {
        if let Some(&ji) = robot.joint_map.get(name) {
            robot.joint_positions[ji] = q;
        }
    }
    robot.rebuild_misarta_model();

    // Robstride EduLite05 actuator model: the official manual
    // (misa-actuator/crates/robstride-protocol/ref/el05_manual_en.md)
    // publishes mass (242 g), 9:1 gear ratio, pole/phase count, but
    // NO damping coefficient or rotor-inertia (armature) spec -- these
    // simply aren't in the datasheet. Values below are engineering
    // placeholders, not catalog numbers:
    //
    // - joint_damping = 0.3 N*m*s/rad: validated empirically in this
    //   project (kyo46rs_njoint_check.rs sweep) -- eliminated ALL
    //   solver NumericalFailure at the original (resonant) 2.0s squat
    //   period, vs. 3.9-7.4% failure rate with zero damping.
    // - armature = 0.0005 kg*m^2: geometric estimate for the rotor
    //   reflected through the 9:1 gearbox. Assuming ~60 g of the 242 g
    //   total is rotor (magnets + back-iron hub, the rest being
    //   housing/gearbox/encoder/driver) modeled as a thin disc of
    //   radius 15 mm (body OD is 46 mm): I_rotor = 0.5*m*r^2 =
    //   0.5*0.06*0.015^2 ~= 6.75e-6 kg*m^2, reflected at the output
    //   joint by gear_ratio^2 = 81 -> ~5.5e-4 kg*m^2, rounded.
    // Runtime-overridable via EL05_DAMPING env var (used for the 0.01-0.15
    // sweep). Default 0.15: the sweep found solver NumericalFailure drops
    // to exactly 0.0% at damping >= 0.11 (and stays there through 0.15,
    // the top of the tested range); 0.15 keeps a margin above that
    // threshold rather than sitting right on it.
    let el05_joint_damping: f64 = std::env::var("EL05_DAMPING")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(0.15);
    const EL05_ARMATURE: f64 = 0.0005;
    for j in robot.joints.iter_mut() {
        j.actuator_mode = ActuatorMode::Torque;
        j.joint_damping = el05_joint_damping;
        j.armature = EL05_ARMATURE;
    }

    let opts = MjcfExportOptions {
        base_pos: Some([0.0, 0.0, 0.9]),
        base_locked_axes: [true; 6],
        ..MjcfExportOptions::default()
    };
    let mut sim = MujocoSim::new(&robot, opts).expect("MujocoSim::new");
    let mj_dt = sim.timestep();
    println!("MuJoCo timestep = {mj_dt} s");
    println!("JOINT_SET ({} joints): {:?}", JOINT_SET.len(), JOINT_SET);

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

    // Seed angle each tracked joint oscillates around, its misarta
    // v-index, and a sign so hip/ankle bend opposite the knee (purely
    // cosmetic -- keeps the motion looking like a natural squat when
    // more than one joint per leg is tracked).
    let tracked: Vec<(usize, usize, f64, f64)> = JOINT_SET
        .iter()
        .map(|&name| {
            let ji = *robot.joint_map.get(name).unwrap_or_else(|| panic!("{name} not in URDF"));
            let mi = a2m[ji].unwrap_or_else(|| panic!("{name} not mapped into misarta model"));
            let seed = full_seed
                .iter()
                .find(|(n, _)| *n == name)
                .map(|(_, q)| *q)
                .unwrap_or(0.0);
            let sign = if name.contains("knee") { -1.0 } else { 1.0 };
            (ji, model.v_idx[mi], seed, sign)
        })
        .collect();

    let mut solver = Solver::new();
    let cfg = SolveConfig::default();
    const KP_JOINT: f64 = 150.0;
    const KD_JOINT: f64 = 30.0;
    const PERIOD_S: f64 = 2.0;
    const N_CYCLES: u32 = 3;
    const AMP: f64 = 0.35;

    let mj_substeps = (0.005 / mj_dt).round().max(1.0) as u32;
    let dt = mj_substeps as f64 * mj_dt;
    println!("control dt = {dt:.4} s ({mj_substeps} physics substeps/tick)");
    let total_t = PERIOD_S * N_CYCLES as f64;
    let n_ticks = (total_t / dt) as usize;

    // (b) fallback: when the P1-level sub-QP goes Degraded, its qddot
    // (and hence tau) can be wildly wrong (confirmed: up to 100-350
    // rad/s^2 off the requested accel_ref during those specific ticks,
    // vs. an exact match when Optimal) -- hold the LAST GOOD (Optimal)
    // torque instead of applying that transient garbage.
    let mut prev_good_taus: Option<Vec<f64>> = None;
    let mut n_fallback_used: u32 = 0;

    // Time-series log for charting: t, then (q, q_ref) pairs for every
    // tracked joint in JOINT_SET order.
    let log_path = "/tmp/claude-1000/-home-takara-work/3288d9bb-da13-4665-bfb2-9595dd62f7ab/scratchpad/njoint_timeseries.csv";
    let mut log_file = std::fs::File::create(log_path).expect("create trajectory log");
    {
        use std::io::Write;
        write!(log_file, "t").unwrap();
        for name in JOINT_SET {
            write!(log_file, ",{name}_q,{name}_ref").unwrap();
        }
        writeln!(log_file).unwrap();
    }

    let mut max_track_err_per_joint = vec![0.0_f64; tracked.len()];
    let mut max_tau_abs_per_joint = vec![0.0_f64; tracked.len()];
    let mut sat_count_per_joint = vec![0u32; tracked.len()];
    let mut max_qddot_gap_per_joint = vec![0.0_f64; tracked.len()];
    let mut n_degraded: u32 = 0;

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

        let phase = 2.0 * PI * t / PERIOD_S;
        let mut j_tracked = na::DMatrix::zeros(tracked.len(), nv);
        let dj_v_tracked = na::DVector::zeros(tracked.len());
        let mut accel_ref = na::DVector::zeros(tracked.len());
        let mut q_ref_this_tick = vec![0.0_f64; tracked.len()];
        for (row, (ji, vidx, seed, sign)) in tracked.iter().enumerate() {
            j_tracked[(row, *vidx)] = 1.0;
            let q_ref = seed + sign * AMP * phase.cos();
            let qd_ref = -sign * AMP * (2.0 * PI / PERIOD_S) * phase.sin();
            let qdd_ref = -sign * AMP * (2.0 * PI / PERIOD_S).powi(2) * phase.cos();
            let q_meas = robot.joint_positions[*ji];
            let qd_meas = v[*vidx];
            accel_ref[row] = qdd_ref + KD_JOINT * (qd_ref - qd_meas) + KP_JOINT * (q_ref - q_meas);
            q_ref_this_tick[row] = q_ref;
            let err = (q_ref - q_meas).abs();
            if err > max_track_err_per_joint[row] {
                max_track_err_per_joint[row] = err;
            }
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
        let p1 = tasks::cartesian_acceleration(dyn_ctx.qddot(), &j_tracked, &dj_v_tracked, &accel_ref);
        let p2 = tasks::regularize(dyn_ctx.tau(), &tau_gravity);

        let sol = solver
            .solve(&[p0, p1, p2], &cfg)
            .unwrap_or_else(|e| panic!("wbc solve failed at t={t:.3}: {e}"));
        if !matches!(sol.status, misa_wbc::SolveStatus::Optimal) {
            n_degraded += 1;
        }
        let extracted = dyn_ctx.extract(&sol.x);

        // Did the solved qddot actually achieve what P1 asked for
        // (accel_ref), or did it settle for something else (e.g.
        // because P2's regularizer or another joint's competing demand
        // pulled the least-squares solution away from it)?
        let mut qddot_gap_this_tick = vec![0.0_f64; tracked.len()];
        for (row, (_ji, vidx, _seed, _sign)) in tracked.iter().enumerate() {
            let gap = (extracted.qddot[*vidx] - accel_ref[row]).abs();
            qddot_gap_this_tick[row] = gap;
            if gap > max_qddot_gap_per_joint[row] {
                max_qddot_gap_per_joint[row] = gap;
            }
        }
        if !matches!(sol.status, misa_wbc::SolveStatus::Optimal) {
            let worst_gap = qddot_gap_this_tick.iter().cloned().fold(0.0, f64::max);
            println!("    [degraded] t={t:.3} status={:?} worst_qddot_gap={worst_gap:.2}", sol.status);
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

        // (b) fallback: only trust this tick's solve if it reached
        // Optimal; otherwise reapply the last known-good torque instead
        // of the (confirmed) garbage qddot/tau a Degraded solve can
        // produce.
        let taus_to_apply = if matches!(sol.status, misa_wbc::SolveStatus::Optimal) {
            prev_good_taus = Some(robot_taus.clone());
            &robot_taus
        } else {
            n_fallback_used += 1;
            prev_good_taus.as_ref().unwrap_or(&robot_taus)
        };
        sim.set_wbc_torques(taus_to_apply);
        sim.step_n_frames(&mut robot, mj_substeps, true);

        {
            use std::io::Write;
            write!(log_file, "{t:.4}").unwrap();
            for (row, (ji, _, _, _)) in tracked.iter().enumerate() {
                write!(log_file, ",{:.5},{:.5}", robot.joint_positions[*ji], q_ref_this_tick[row]).unwrap();
            }
            writeln!(log_file).unwrap();
        }

        for (row, (ji, _, _, _)) in tracked.iter().enumerate() {
            let tau_abs = taus_to_apply[*ji].abs();
            if tau_abs > max_tau_abs_per_joint[row] {
                max_tau_abs_per_joint[row] = tau_abs;
            }
            let limit = torque_max[model.v_idx[a2m[*ji].unwrap()] - 6];
            if tau_abs > 0.98 * limit {
                sat_count_per_joint[row] += 1;
            }
        }

        if tick % ((PERIOD_S / dt) as usize / 2).max(1) == 0 {
            let report: Vec<String> = tracked
                .iter()
                .enumerate()
                .map(|(row, (ji, _, _, _))| format!("{}={:+.3}", JOINT_SET[row], robot.joint_positions[*ji]))
                .collect();
            let gaps: Vec<String> = qddot_gap_this_tick
                .iter()
                .map(|g| format!("{g:.2}"))
                .collect();
            println!(
                "  t={t:6.3}  {}  qddot_gap=[{}]  status={:?}",
                report.join(" "),
                gaps.join(","),
                sol.status
            );
        }
    }

    println!("\n=== Result ({} joints tracked) ===", tracked.len());
    for (row, (ji, ..)) in tracked.iter().enumerate() {
        let limit = torque_max[model.v_idx[a2m[*ji].unwrap()] - 6];
        println!(
            "  {:<28} max err={:.4} rad   max |tau|={:.2} / {:.1} N*m   saturated {:.1}% of ticks   max qddot_gap={:.3} rad/s^2",
            JOINT_SET[row],
            max_track_err_per_joint[row],
            max_tau_abs_per_joint[row],
            limit,
            100.0 * sat_count_per_joint[row] as f64 / n_ticks as f64,
            max_qddot_gap_per_joint[row],
        );
    }
    println!("  degraded (non-Optimal) solves: {n_degraded} / {n_ticks} ticks ({:.1}%)", 100.0 * n_degraded as f64 / n_ticks as f64);
    println!("  fallback (held last-good torque) used: {n_fallback_used} / {n_ticks} ticks ({:.1}%)", 100.0 * n_fallback_used as f64 / n_ticks as f64);
}

#[cfg(not(feature = "mujoco"))]
fn main() {
    eprintln!("This example requires the `mujoco` feature. Run with:");
    eprintln!("  cargo run --features mujoco --example kyo46rs_njoint_check");
}
