//! Flight probe: what the WBC does with an EMPTY contact set.
//!
//! The walker never hands the QP zero contacts (`ContactCorrection::update`
//! re-inserts a foot, gait.rs). Before building a jump, this measures whether
//! the QP is even well-posed in flight and what it asks for:
//!
//! 1. stand on both feet (CoM hold, trunk, posture) until `T_LAUNCH`;
//! 2. throw the robot up with a world-frame force on the pelvis
//!    (`LAUNCH_V` m/s of take-off velocity over `LAUNCH_DUR` s);
//! 3. while no foot carries `F_ON` N (measured, `ON_TICKS` ticks of
//!    hysteresis), solve with nc = 0: EoM (hard) + torque box, then trunk
//!    attitude and posture. `FLIGHT_KEEP_COM=1` also keeps the stance CoM
//!    task, which flight cannot satisfy, to see what an inconsistent task does;
//! 4. on touchdown, return to the stance tasks with the CoM target at the
//!    touchdown point (x, y) and the stand height.
//!
//! Reported for the flight ticks: solve status, |predicted CoM accel - g|
//! (from the QP's own qddot, i.e. is its base dynamics ballistic),
//! |measured CoM accel - g|, drift of the centroidal angular momentum
//! (measured) and its predicted rate (should be 0), torque-box use, and the
//! landing outcome.
//!
//! ```text
//! ROBOT=g1 cargo run --release --features mujoco --example g1_flight_probe
//! ROBOT=g1 LAUNCH_V=1.2 FLIGHT_KEEP_COM=1 cargo run ... --example g1_flight_probe
//! ```

#[cfg(feature = "mujoco")]
fn main() {
    use articara::biped::actuate::{CommandPolicy, DegradedTally, gravity_plus_posture, write_to_plant};
    use articara::biped::contact::{Anchors, contact_jacobians};
    use articara::biped::log::measure_contacts;
    use articara::biped::profile;
    use articara::biped::rig::{BipedRig, CtrlMode, G, RigOptions};
    use articara::biped::tasks as bt;
    use misa_wbc::{Dynamics, Formulation, SolveConfig, Solver, tasks};
    use nalgebra as na;

    let env_f64 = |k: &str, d: f64| -> f64 { std::env::var(k).ok().and_then(|s| s.parse().ok()).unwrap_or(d) };
    let flag = |k: &str, d: bool| -> bool { std::env::var(k).map(|v| v != "0").unwrap_or(d) };

    let mut prof = profile::by_name(&std::env::var("ROBOT").unwrap_or_else(|_| "g1".into()));
    if let Ok(u) = std::env::var("URDF") {
        prof.urdf = Box::leak(u.into_boxed_str());
        println!("URDF override: {}", prof.urdf);
    }
    let mut o = RigOptions::from_profile(&prof);
    o.knee = env_f64("KNEE", prof.knee_seed);
    o.hip_pitch = env_f64("HIP_PITCH", -o.knee / 2.0);
    o.ankle_pitch = env_f64("ANKLE_PITCH", -o.knee / 2.0);
    o.joint_damping = env_f64("JOINT_DAMPING", prof.joint_damping);
    o.armature = env_f64("ARMATURE", prof.armature);
    o.burnin_kp = env_f64("BURNIN_KP", prof.burnin_kp);
    o.burnin_kv = env_f64("BURNIN_KV", prof.burnin_kv);
    o.burnin_s = env_f64("BURNIN_S", prof.burnin_s);
    o.run_kp = env_f64("RUN_KP", prof.burnin_kp);
    o.run_kv = env_f64("RUN_KV", prof.burnin_kv);
    o.ctrl_mode = CtrlMode::from_env_name(&std::env::var("CTRL_MODE").unwrap_or_else(|_| "torque".into()));
    o.sim_dt = env_f64("SIM_DT", 0.001);
    o.mu_ground = env_f64("MU_GROUND", 0.7);
    o.probe_z = env_f64("PROBE_Z", prof.probe_z);
    let sole_half_l = env_f64("SOLE_HALF_L", prof.cop_half.0);
    let sole_half_w = env_f64("SOLE_HALF_W", prof.cop_half.1);
    let mu_ground = o.mu_ground;
    let ctrl_mode = o.ctrl_mode;

    let mut rig = BipedRig::build(prof, &o);
    let nv = rig.nv;
    let na_count = rig.na;
    let mj_dt = rig.mj_dt;
    let feet = [rig.left_foot_mi(), rig.right_foot_mi()];
    let total_mass = rig.total_mass;
    let weight = total_mass * G;
    let f_max_per_foot = env_f64("F_MAX", 2.3 * weight);

    let mut solver = Solver::new();
    let mut cfg = SolveConfig::default();
    // WBC_STRATEGY=weighted: the walker's solver (level 0 hard, the rest one
    // weighted QP with the walker's default per-level weights).
    let weighted = std::env::var("WBC_STRATEGY").as_deref() == Ok("weighted");
    if weighted {
        cfg.strategy = misa_wbc::HqpStrategy::WeightedQp { level_ratio: 1 };
    }
    let friction_mu = env_f64("friction_mu", mu_ground * prof.friction_margin);
    let kp_com = env_f64("KP_COM", 300.0);
    let kd_com = env_f64("KD_COM", 80.0);
    let kp_trunk = env_f64("KP_TRUNK", 200.0);
    let kd_trunk = env_f64("KD_TRUNK", 40.0);
    let kp_post = env_f64("KP_POST", 100.0);
    let kd_post = env_f64("KD_POST", 20.0);
    let kp_c = env_f64("KP_CONTACT", 400.0);
    let kd_c = env_f64("KD_CONTACT", 2.0 * kp_c.sqrt());
    let hold_kp = env_f64("HOLD_KP", prof.hold_kp);
    let hold_kd = env_f64("HOLD_KD", prof.hold_kd);
    let total_t = env_f64("T", 4.0);
    let t_launch = env_f64("T_LAUNCH", 2.0);
    let launch_v = env_f64("LAUNCH_V", 1.0);
    let launch_dur = env_f64("LAUNCH_DUR", 0.10).max(1e-3);
    let f_on = env_f64("F_ON", 0.05 * weight);
    let on_ticks = env_f64("ON_TICKS", 2.0) as u32;
    let flight_trunk = flag("FLIGHT_TRUNK", true);
    let flight_post = flag("FLIGHT_POST", true);
    let flight_keep_com = flag("FLIGHT_KEEP_COM", false);
    let verbose = flag("VERBOSE", false);

    let ctrl_dt = env_f64("CTRL_DT", prof.ctrl_dt);
    let mj_substeps = (ctrl_dt / mj_dt).round().max(1.0) as u32;
    let dt = mj_substeps as f64 * mj_dt;
    let n_ticks = (total_t / dt) as usize;
    println!("flight probe: m={total_mass:.2} kg, launch {launch_v:.2} m/s over {launch_dur:.3} s at t={t_launch:.2}, F_ON {f_on:.1} N, ctrl {:.0} Hz", 1.0 / dt);

    let contact_cfg = bt::ContactCfg {
        kp_c,
        kd_c,
        anchor_leak: env_f64("ANCHOR_LEAK", 0.2),
        anchor_leak_rot: 0.0,
        sole_offset_local: [prof.sole_centre_x, 0.0, -prof.sole_below_origin],
        friction_mu,
        cop_half: (sole_half_l, sole_half_w),
        mu_torsion: 0.05,
        f_max_per_foot,
        no_patch: false,
        dt,
    };
    let mut policy = CommandPolicy::new(rig.robot.joints.len(), 999, 8, false, 0);
    let mut tally = DegradedTally::new();
    let mut anchors = Anchors::new();

    // Contact state from the measured load, with hysteresis per foot.
    let mut in_contact = [true, true];
    let mut flip_count = [0u32; 2];
    let mut com_ref: Option<na::Vector3<f64>> = None;
    let mut stand_z = None;
    let mut launched = false;
    let mut prev_com_vel: Option<na::Vector3<f64>> = None;
    let mut flight_t0: Option<f64> = None;
    let mut h_ang0: Option<na::Vector3<f64>> = None;
    let mut flight_stats_t0: Option<f64> = None;
    // Flight statistics.
    let (mut n_flight, mut n_flight_degraded) = (0u32, 0u32);
    let (mut max_pred_err, mut max_meas_err, mut max_dh_pred, mut max_h_drift, mut max_tau_use) =
        (0.0_f64, 0.0_f64, 0.0_f64, 0.0_f64, 0.0_f64);
    let mut flight_spans: Vec<(f64, f64)> = Vec::new();
    let mut touchdown_t: Option<f64> = None;
    let mut fell = false;
    let mut max_tilt_after = 0.0_f64;

    for tick in 0..n_ticks {
        let t = tick as f64 * dt;
        let st = rig.sync();
        let (q, v, v_dvec, data) = (&st.q, &st.v, &st.v_dvec, &st.data);
        let com = st.com;
        let com_vel = st.com_vel;
        let stand_z = *stand_z.get_or_insert(com.z);

        if !launched && t >= t_launch {
            launched = true;
            let fz = weight + total_mass * launch_v / launch_dur;
            rig.sim.apply_external_force(&rig.robot.root_link, [0.0, 0.0, fz], [0.0; 3], launch_dur);
            println!("  [launch] t={t:.3} pelvis force {fz:.1} N for {launch_dur:.3} s");
        }

        // ---- measured contact state -------------------------------------
        let measured = measure_contacts(&rig, data, friction_mu);
        for s in 0..2 {
            let loaded = measured.cop[s][2] > f_on;
            if loaded != in_contact[s] {
                flip_count[s] += 1;
                if flip_count[s] >= on_ticks {
                    in_contact[s] = loaded;
                    flip_count[s] = 0;
                    if !loaded {
                        anchors.release(s);
                    }
                }
            } else {
                flip_count[s] = 0;
            }
        }
        let stance: Vec<usize> = (0..2).filter(|&s| in_contact[s]).map(|s| feet[s]).collect();
        let nc = stance.len();
        let flight = nc == 0;
        match (flight, flight_t0) {
            (true, None) => {
                flight_t0 = Some(t);
                h_ang0 = None;
                println!("  [lift-off] t={t:.3} com z {:.4} vz {:+.3}", com.z, com_vel.z);
            }
            (false, Some(t0)) => {
                flight_spans.push((t0, t));
                flight_t0 = None;
                touchdown_t.get_or_insert(t);
                com_ref = Some(na::Vector3::new(com.x, com.y, stand_z));
                println!("  [touchdown] t={t:.3} after {:.3} s, com z {:.4} vz {:+.3}, feet {:?}", t - t0, com.z, com_vel.z, in_contact);
            }
            _ => {}
        }
        let c_ref = *com_ref.get_or_insert(com);

        // ---- QP -------------------------------------------------------------
        let (j_contact, dj_v) = contact_jacobians(&rig.model, q, v, data, v_dvec, &stance, nv);
        let dyn_ctx = Dynamics::new(Formulation::Explicit, &st.mass, &st.h, &j_contact, na_count);
        let eom_task = dyn_ctx.dynamics_task().expect("Explicit keeps the EoM task");
        let tau_box = tasks::box_bound(dyn_ctx.tau(), &rig.torque_max);
        let mut levels: Vec<misa_wbc::Task> = vec![eom_task];
        let mut names: Vec<&str> = vec!["eom"];
        if flight {
            levels.push(tau_box);
            names.push("torque box");
        } else {
            let p0 = bt::contact_level(
                &dyn_ctx, tau_box, &j_contact, &dj_v, data, v_dvec, &stance, feet[0], &mut anchors,
                &|_| 1.0, &contact_cfg, &|_| bt::ContactMode::Rigid, &|_, _| None,
            );
            levels.push(p0.task);
            names.push("contact+cones");
        }
        if !flight || flight_keep_com {
            let a_com = kd_com * (-com_vel) + kp_com * (c_ref - com);
            levels.push(bt::com(dyn_ctx.qddot(), &st.j_com, &st.djv_com, &a_com));
            names.push("com");
        }
        let j_trunk = misarta::jacobian::compute_joint_jacobian_from_data(&rig.model, q, data, rig.trunk_mi);
        let djv_trunk = &misarta::jacobian::compute_joint_jacobian_time_derivative(&rig.model, q, v, rig.trunk_mi) * v_dvec;
        let (roll, pitch, _) = st.body_quat.euler_angles();
        if !flight || flight_trunk {
            let rp_ref = bt::trunk_rp_ref(roll, pitch, &st.v_ang_w, kp_trunk, kd_trunk, 0.0, 1.0);
            levels.push(bt::trunk(dyn_ctx.qddot(), &j_trunk, &djv_trunk, &rp_ref, nv));
            names.push("trunk");
        }
        if !flight || flight_post {
            levels.push(bt::posture(
                dyn_ctx.qddot(), &rig.actuated(), &rig.robot.joint_positions, &rig.q_seed, v,
                kp_post, kd_post, na_count, nv,
            ));
            names.push("posture");
        }
        let tau_gravity = rig.gravity_torque(q);
        let shares = vec![1.0; nc];
        let forces_nominal = bt::force_nominal(dyn_ctx.forces().size(), &shares, weight);
        // In flight there is no weight to carry: regularise torque to zero.
        let tau_reg_target = if flight { na::DVector::zeros(tau_gravity.len()) } else { tau_gravity.clone() };
        levels.push(bt::regulariser(dyn_ctx.tau(), &tau_reg_target, &dyn_ctx.forces(), &forces_nominal));
        names.push("regularise");

        if weighted {
            let nd = levels[0].n_decision();
            for (lvl, name) in levels.iter_mut().zip(&names).skip(1) {
                let w = match *name {
                    "com" => 1.0,
                    "trunk" => 0.3,
                    "posture" => 0.01,
                    "regularise" => 1e-4,
                    _ => 0.1,
                };
                let taken = std::mem::replace(lvl, misa_wbc::Task::empty(nd));
                *lvl = taken.weight(w);
            }
        }
        let sol = solver.solve(&levels, &cfg).unwrap_or_else(|e| panic!("wbc solve failed at t={t:.3} (nc={nc}): {e}"));
        tally.observe(&sol.status, t, tick, nc, &names);
        let ex = dyn_ctx.extract(&sol.x);

        // ---- flight diagnostics --------------------------------------------
        // Stats only once the launch force is off: while it acts, the CoM is
        // not ballistic and the pelvis force puts a torque about the CoM.
        // ... and only on ticks where the plant itself has no foot contact:
        // the ON_TICKS hysteresis otherwise lets the touchdown impact in.
        let truly_airborne = measured.cop[0][2] <= 0.0 && measured.cop[1][2] <= 0.0;
        if flight && truly_airborne && t >= t_launch + launch_dur + dt {
            if h_ang0.is_none() || flight_stats_t0.is_none() {
                let cmm = misarta::centroidal::compute_centroidal_momentum_matrix(&rig.model, q);
                let h = &cmm * v_dvec;
                h_ang0 = Some(na::Vector3::new(h[0], h[1], h[2]));
                flight_stats_t0 = Some(t);
                prev_com_vel = None;
            }
            n_flight += 1;
            if !matches!(sol.status, misa_wbc::SolveStatus::Optimal) {
                n_flight_degraded += 1;
            }
            let g = na::Vector3::new(0.0, 0.0, -G);
            let a_pred = &st.j_com * &ex.qddot + st.djv_com;
            let pred_err = (na::Vector3::new(a_pred[0], a_pred[1], a_pred[2]) - g).norm();
            max_pred_err = max_pred_err.max(pred_err);
            let mut meas_err = 0.0;
            if let Some(pv) = prev_com_vel {
                meas_err = ((com_vel - pv) / dt - g).norm();
                max_meas_err = max_meas_err.max(meas_err);
            }
            if verbose {
                let selfc = rig.sim.contacts().iter().filter(|c| !c.body1.is_empty() && !c.body2.is_empty()).count();
                let anyc = rig.sim.contacts().len();
                let cmm = misarta::centroidal::compute_centroidal_momentum_matrix(&rig.model, q);
                let h = &cmm * v_dvec;
                let drift = h_ang0.map_or(0.0, |h0| (na::Vector3::new(h[0], h[1], h[2]) - h0).norm());
                let (r, p, _) = st.body_quat.euler_angles();
                println!("  [air] t={t:.4} |a_meas-g|={meas_err:.3} |h-h0|={drift:.4} roll={r:+.4} pitch={p:+.4} contacts={anyc} self={selfc}");
            }
            prev_com_vel = Some(com_vel);
            let cmm = misarta::centroidal::compute_centroidal_momentum_matrix(&rig.model, q);
            let dcmm_v = misarta::centroidal::compute_cmm_dot_times_v(&rig.model, q, v);
            let h = &cmm * v_dvec;
            let dh = &cmm * &ex.qddot + &dcmm_v;
            max_dh_pred = max_dh_pred.max(na::Vector3::new(dh[0], dh[1], dh[2]).norm());
            if let Some(h0) = h_ang0 {
                max_h_drift = max_h_drift.max((na::Vector3::new(h[0], h[1], h[2]) - h0).norm());
            }
            for (i, tmax) in rig.torque_max.iter().enumerate() {
                if *tmax > 0.0 {
                    max_tau_use = max_tau_use.max(ex.tau[i].abs() / tmax);
                }
            }
            if verbose {
                println!(
                    "  [flight] t={t:.3} status={:?} |a_pred-g|={pred_err:.3} |dh_ang_pred|={:.3} roll={roll:+.3} pitch={pitch:+.3}",
                    sol.status,
                    na::Vector3::new(dh[0], dh[1], dh[2]).norm()
                );
            }
        }
        if !flight {
            flight_stats_t0 = None;
            prev_com_vel = None;
        }

        // ---- actuate ----------------------------------------------------------
        let mut robot_taus = vec![0.0_f64; rig.robot.joints.len()];
        for (ji, vi) in rig.actuated() {
            robot_taus[ji] = ex.tau[vi - 6];
        }
        {
            let rig_ref = &rig;
            let tg = &tau_gravity;
            let fallback = move |out: &mut [f64]| gravity_plus_posture(rig_ref, tg, v, hold_kp, hold_kd, out);
            policy.apply(&sol.status, &mut robot_taus, t, dt, nc.max(1), &fallback);
        }
        write_to_plant(&mut rig, ctrl_mode, &robot_taus, &ex.qddot, v, dt);
        rig.step(mj_substeps);

        let tilt = roll.abs().max(pitch.abs());
        if touchdown_t.is_some() {
            max_tilt_after = max_tilt_after.max(tilt);
        }
        if tick % 40 == 0 {
            println!("  t={t:6.3} nc={nc} com=({:+.3},{:+.3},{:+.4}) vz={:+.3} roll={roll:+.3} pitch={pitch:+.3} status={:?}", com.x, com.y, com.z, com_vel.z, sol.status);
        }
        let cur_z = rig.sim.body_world_position(&rig.robot.root_link).unwrap()[2];
        if cur_z < 0.30 || tilt > 0.52 {
            println!("  FELL at t={t:.3} (z={cur_z:.3}, tilt={tilt:.3})");
            fell = true;
            break;
        }
    }

    println!("\n=== Flight probe ===");
    println!("  flight spans: {:?}", flight_spans.iter().map(|(a, b)| format!("{a:.3}-{b:.3} ({:.3} s)", b - a)).collect::<Vec<_>>());
    println!("  flight ticks: {n_flight}, non-optimal solves in flight: {n_flight_degraded}");
    println!("  max |predicted CoM accel - g| in flight: {max_pred_err:.4} m/s^2  (QP base dynamics ballistic if ~0)");
    println!("  max |measured CoM accel - g| in flight:  {max_meas_err:.4} m/s^2");
    println!("  max |predicted d(h_ang)/dt| in flight:   {max_dh_pred:.4} N*m  (conservation if ~0)");
    println!("  max |h_ang - h_ang(lift-off)| measured:  {max_h_drift:.4} kg*m^2/s");
    println!("  max torque-box use in flight: {max_tau_use:.2}");
    println!("  max tilt after touchdown: {max_tilt_after:.3} rad");
    println!("  degraded solves (all): {}", tally.n);
    tally.report();
    println!("  verdict: {}", if fell { "FELL" } else if flight_spans.is_empty() { "NO FLIGHT" } else { "LANDED" });
}

#[cfg(not(feature = "mujoco"))]
fn main() {
    eprintln!("This example requires the `mujoco` feature.");
}
