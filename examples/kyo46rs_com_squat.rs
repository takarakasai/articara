//! Biped WBC: centre-of-mass squat and single-leg stance.
//!
//! Controls the CoM. The older version that held base roll/pitch/height
//! separately (`kyo46rs_squat.rs`, kept for comparison) plateaued because the
//! attitude task and the height task compete for the same contact authority.
//! The base's 6 DoF are unactuated, so the only handle on balance is the
//! contact wrench, and a squat is nothing but a command on CoM z -- which
//! collapses two tasks into one.
//!
//! Task levels:
//!
//! | level | contents |
//! |---|---|
//! | P0 | EoM + contact (Baumgarte) + `patch_contact` + torque box |
//! | P1 | CoM acceleration (3 rows) |
//! | P2 | trunk attitude, WORLD-frame angular Jacobian (2 rows) |
//! | P3 | swing clearance (single support only, z row only) |
//! | P4 | posture hold |
//! | P5 | tau -> gravity comp, contact force -> nominal split |
//!
//! Reproduce:
//!
//! ```text
//! cargo run --features mujoco --example kyo46rs_com_squat                    # stand
//! AMP=0.07 cargo run --features mujoco --example kyo46rs_com_squat           # squat
//! KNEE=1.10 HIP_PITCH=-0.55 ANKLE_PITCH=-0.55 LIFT=1 T=40 \
//!   cargo run --features mujoco --example kyo46rs_com_squat                  # single leg
//! ```
//!
//! When changing `KNEE`, always set `HIP_PITCH = ANKLE_PITCH = -KNEE/2` with
//! it: the three must sum to zero or the sole is not parallel to the floor
//! and the robot stands on an edge.
//!
//! The machinery lives in [`articara::biped`]; this file is the reference
//! generation and the level ordering, which is what actually differs between
//! experiments. See `doc/kyo46rs_biped_wbc.md` for what is measured, and in
//! particular for the list of results that turned out to be artifacts.

#[cfg(feature = "mujoco")]
fn main() {
    use articara::biped::actuate::{gravity_plus_posture, write_to_plant, CommandPolicy, DegradedTally};
    use articara::biped::contact::{contact_jacobians, cop_from_sole_wrench, Anchors};
    use articara::biped::log::{measure_contacts, Row, TrajLog};
    use articara::biped::profile;
    use articara::biped::rig::{BipedRig, CtrlMode, RigOptions, G};
    use articara::biped::tasks as bt;
    use misa_wbc::{tasks, Dynamics, Formulation, SolveConfig, Solver};
    use nalgebra as na;
    use std::f64::consts::PI;

    let env_f64 = |k: &str, d: f64| -> f64 {
        std::env::var(k).ok().and_then(|s| s.parse().ok()).unwrap_or(d)
    };
    let flag = |k: &str, d: bool| -> bool { std::env::var(k).map(|v| v != "0").unwrap_or(d) };

    let prof = profile::by_name(&std::env::var("ROBOT").unwrap_or_default());

    // ── Rig bring-up ───────────────────────────────────────────────────
    let mut o = RigOptions::from_profile(&prof);
    o.knee = env_f64("KNEE", prof.knee_seed);
    o.hip_pitch = env_f64("HIP_PITCH", -o.knee / 2.0);
    o.ankle_pitch = env_f64("ANKLE_PITCH", -o.knee / 2.0);
    o.joint_damping = env_f64("JOINT_DAMPING", prof.joint_damping);
    o.armature = env_f64("ARMATURE", prof.armature);
    o.torque_scale = env_f64("TORQUE_SCALE", 1.0);
    o.burnin_kp = env_f64("BURNIN_KP", prof.burnin_kp);
    o.burnin_kv = env_f64("BURNIN_KV", prof.burnin_kv);
    o.burnin_s = env_f64("BURNIN_S", prof.burnin_s);
    o.run_kp = env_f64("RUN_KP", prof.burnin_kp);
    o.run_kv = env_f64("RUN_KV", prof.burnin_kv);
    o.ctrl_mode = CtrlMode::from_env_name(
        &std::env::var("CTRL_MODE").unwrap_or_else(|_| "torque".into()),
    );
    o.sim_dt = env_f64("SIM_DT", 0.001);
    o.mu_ground = env_f64("MU_GROUND", 0.7);
    o.probe_z = env_f64("PROBE_Z", prof.probe_z);
    // DIAGNOSTIC ONLY. See the self-collision counter below: single-leg
    // stance spends 97% of its ticks with a forearm resting on a hip block,
    // and this is how to ask what the result looks like without it.
    if flag("NO_ARM_COLLIDE", false) {
        o.uncollide_links = vec!["forearm", "upper_arm"];
    }

    let sole_half_l = env_f64("SOLE_HALF_L", prof.cop_half.0);
    let sole_half_w = env_f64("SOLE_HALF_W", prof.cop_half.1);
    let mu_ground = o.mu_ground;
    let ctrl_mode = o.ctrl_mode;

    let mut rig = BipedRig::build(prof, &o);
    let nv = rig.nv;
    let na_count = rig.na;
    let mj_dt = rig.mj_dt;
    let left_foot_mi = rig.left_foot_mi();
    let right_foot_mi = rig.right_foot_mi();
    let total_mass = rig.total_mass;

    // Per-foot vertical force ceiling on patch_contact.
    //
    // Scale it with the machine, because the thing it has to be consistent
    // with already is: the P5 regulariser asks each stance foot for
    // total_mass*G/nc, which in SINGLE support is the whole weight. A cap
    // below that puts the lowest-priority target outside the highest-priority
    // constraint, and the QP resolves the contradiction with slack --
    // silently, reported as Optimal. On G1 the old flat 150 N was 45% of body
    // weight and the solved fz breached it on 21-68% of ticks, peaking at
    // 177 N.
    let f_max_scale = env_f64("F_MAX_SCALE", 2.3);
    let f_max_per_foot = env_f64("F_MAX", f_max_scale * total_mass * G);
    println!(
        "  contact f_max per foot: {f_max_per_foot:.1} N  (weight {:.1} N, \
         single-support nominal {:.1} N)",
        total_mass * G,
        total_mass * G
    );
    assert!(
        f_max_per_foot >= total_mass * G,
        "f_max ({f_max_per_foot:.1} N) is below the single-support nominal \
         ({:.1} N): the force regulariser would be asking for something the \
         contact cone forbids, and the QP would break the cone with slack \
         while still reporting Optimal",
        total_mass * G
    );

    // ── Control-law knobs ──────────────────────────────────────────────
    let mut solver = Solver::new();
    let cfg = SolveConfig::default();
    // Derived from the plant's friction, never written independently: two
    // separate literals is how they drift apart without anyone noticing.
    let friction_mu: f64 = env_f64("friction_mu", mu_ground * prof.friction_margin);
    let kp_com = env_f64("KP_COM", 300.0);
    let kd_com = env_f64("KD_COM", 80.0);
    let kp_trunk = env_f64("KP_TRUNK", 200.0);
    let kd_trunk = env_f64("KD_TRUNK", 40.0);
    let kp_post = env_f64("KP_POST", 100.0);
    let kd_post = env_f64("KD_POST", 20.0);
    let use_post = flag("POST", true);
    let trunk_sign = env_f64("TRUNK_SIGN", 1.0);
    let use_trunk = flag("TRUNK", true);
    // LATCH_STANCE=0 restores reading the stance foot's y every tick, which
    // is how the reference came to depend on the thing it is supposed to be
    // steering.
    let latch_stance = flag("LATCH_STANCE", true);
    // Put the equation of motion on its own top level so the null-space
    // cascade enforces it exactly, instead of letting it be traded against
    // the cones in a shared least-squares objective.
    let eom_hard = flag("EOM_HARD", true);
    let trunk_dead = env_f64("TRUNK_DEAD", 0.0);
    let trunk_late = flag("TRUNK_LATE", false);
    let cop_frac = env_f64("COP_FRAC", 1.0);
    let com_sign = env_f64("COM_SIGN", 1.0);
    let comchk = flag("COMCHK", false);
    let period_s = env_f64("PERIOD_S", 3.0);
    let squat_amp = env_f64("AMP", 0.0); // 0 = hold still; >0 = squat
    let lift_leg = flag("LIFT", false);
    let com_dx = env_f64("COM_DX", 0.0);
    let t_shift = env_f64("T_SHIFT", 3.0);
    let lift_h = env_f64("LIFT_H", prof.lift_h);
    let lift_ramp = env_f64("LIFT_RAMP", 1.0);
    let unload_ramp = env_f64("UNLOAD_RAMP", 0.0);
    let hold_kp = env_f64("HOLD_KP", prof.hold_kp);
    let hold_kd = env_f64("HOLD_KD", prof.hold_kd);
    let hold_bridge = env_f64("HOLD_BRIDGE", 8.0) as u32;
    let fallback_max_level = env_f64("FALLBACK_LEVEL", 999.0) as usize;
    let lat_share = flag("LAT_SHARE", true);
    let hold_last = flag("HOLD_LAST", false);
    let blend_ticks = env_f64("BLEND_TICKS", 0.0) as u32;
    let swing_z_only = flag("SWING_ZONLY", true);
    let kp_sw = env_f64("KP_SWING", prof.kp_swing);
    let kd_sw = env_f64("KD_SWING", prof.kd_swing);
    let total_t = env_f64("T", 6.0);
    let anchor_leak = env_f64("ANCHOR_LEAK", 0.2);
    let anchor_leak_rot = env_f64("ANCHOR_LEAK_ROT", 0.0);
    //
    // 1600 was chosen when Baumgarte was first added, to stop a sole rolling
    // to 19 deg, and never revisited. It is far too stiff once a single foot
    // carries everything: the same gain that is reasonable spread over two
    // contacts slams one, and on G1 it spiked the contact force to 635 N
    // against a 335 N robot and threw 26.5 deg of roll into it 60 ms after
    // lift-off. At 400 (critical damping kept, kd = 2*sqrt(kp)) the spike
    // falls to 434 N and the roll to 0.1 deg -- G1 holds single support with
    // zero degraded solves. kyo46rs improves too (degraded 23 -> 9).
    let kp_c = env_f64("KP_CONTACT", 400.0);
    let kd_c = env_f64("KD_CONTACT", 2.0 * kp_c.sqrt());
    let no_patch = flag("NO_PATCH", false);
    let attitude_from_fk = flag("ATT_FK", false);

    // Control period, distinct from the physics step. The plant runs at
    // SIM_DT (1 ms, forced by the explicit joint PD's kv < 2I/dt limit); the
    // WBC runs every CTRL_DT and MuJoCo is stepped CTRL_DT/SIM_DT times in
    // between.
    let ctrl_dt = env_f64("CTRL_DT", prof.ctrl_dt);
    let mj_substeps = (ctrl_dt / mj_dt).round().max(1.0) as u32;
    let dt = mj_substeps as f64 * mj_dt;
    println!("control: {:.1} kHz plant / {:.0} Hz WBC ({mj_substeps} substeps per tick)",
             1e-3 / mj_dt, 1.0 / dt);
    let n_ticks = (total_t / dt) as usize;

    rig.report_footprint(sole_half_l);

    let contact_cfg = bt::ContactCfg {
        kp_c,
        kd_c,
        anchor_leak,
        anchor_leak_rot,
        sole_offset_local: [prof.sole_centre_x, 0.0, -prof.sole_below_origin],
        friction_mu,
        cop_half: (sole_half_l * cop_frac, sole_half_w * cop_frac),
        mu_torsion: 0.05,
        f_max_per_foot,
        no_patch,
        dt,
    };

    let mut log = TrajLog::create(std::env::var("TRAJ_CSV").ok(), prof.log_joints.to_vec(), &[]);
    let mut policy = CommandPolicy::new(
        rig.robot.joints.len(),
        fallback_max_level,
        hold_bridge,
        hold_last,
        blend_ticks,
    );
    let mut tally = DegradedTally::new();
    let mut anchors = Anchors::new();

    let mut com_ref0: Option<na::Vector3<f64>> = None;
    let mut swing_home_cell: Option<na::Vector3<f64>> = None;
    let mut stance_y_cell: Option<f64> = None;
    let mut prev_com: Option<na::Vector3<f64>> = None;
    let mut prev_body_pos: Option<[f64; 3]> = None;
    let mut fell = false;
    let mut min_z = f64::INFINITY;
    let mut max_tilt: f64 = 0.0;
    let mut max_jcom_err: f64 = 0.0;
    let mut n_selfcollide = 0u32;
    let mut max_selfcollide_f: f64 = 0.0;

    for tick in 0..n_ticks {
        let t = tick as f64 * dt;

        // ---- sync state ------------------------------------------------
        let st = rig.sync();
        let (q, v, v_dvec, data) = (&st.q, &st.v, &st.v_dvec, &st.data);
        let body_quat = st.body_quat;
        let v_ang_w = st.v_ang_w;
        let com = st.com;
        let com_vel = st.com_vel;

        // Is `body_world_linear_velocity` actually the velocity of the body
        // ORIGIN (what `body_world_position` reports)? It reads MuJoCo's
        // `cvel`, whose linear part is expressed in the c-frame -- world-
        // aligned axes but origin at the subtree CoM, not at xpos.
        if flag("VELCHK", false) {
            if let Some(pp) = prev_body_pos {
                let fd: [f64; 3] = [
                    (st.body_pos[0] - pp[0]) / dt,
                    (st.body_pos[1] - pp[1]) / dt,
                    (st.body_pos[2] - pp[2]) / dt,
                ];
                if tick % 20 == 0 {
                    let vl = rig.sim.body_world_linear_velocity(&rig.robot.root_link).unwrap();
                    println!(
                        "  [velchk] d(xpos)/dt=({:+.4},{:+.4},{:+.4})  cvel_lin=({:+.4},{:+.4},{:+.4})",
                        fd[0], fd[1], fd[2], vl[0], vl[1], vl[2]
                    );
                }
            }
        }
        prev_body_pos = Some(st.body_pos);

        // One-shot column-wise check of J_com against finite differences on
        // the joint coordinates (the base columns need quaternion
        // integration, so they are checked via the running J*v vs d(com)/dt
        // comparison below instead).
        if tick == 0 && flag("COLCHK", false) {
            const EPS: f64 = 1e-6;
            let mut worst = (0usize, 0.0_f64, String::new());
            for (ji, vi) in rig.actuated() {
                let qi = rig.model.q_idx[rig.a2m[ji].unwrap()];
                let mut qp = q.clone();
                qp[qi] += EPS;
                let fd = (rig.com_of(&misarta::fk::forward_kinematics(&rig.model, &qp)) - com) / EPS;
                let col = na::Vector3::new(st.j_com[(0, vi)], st.j_com[(1, vi)], st.j_com[(2, vi)]);
                let e = (fd - col).norm();
                if e > worst.1 {
                    worst = (vi, e, rig.robot.joints[ji].name.clone());
                }
                if e > 1e-4 {
                    println!(
                        "  [colchk] {:<28} v{vi}: fd=({:+.5},{:+.5},{:+.5}) J=({:+.5},{:+.5},{:+.5}) err={e:.2e}",
                        rig.robot.joints[ji].name, fd.x, fd.y, fd.z, col.x, col.y, col.z
                    );
                }
            }
            println!("  [colchk] worst joint column: {} (v{}) err={:.3e}", worst.2, worst.0, worst.1);
        }

        // Verify J_com against a finite difference of the measured CoM rather
        // than trusting the shift algebra.
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
        // In single support the right foot leaves the ground, so it must also
        // leave the contact set -- keeping its rows would have the QP solve
        // against a reaction force that no longer exists.
        let single = lift_leg && t >= t_shift;
        let stance: Vec<usize> = if single {
            vec![left_foot_mi]
        } else {
            vec![left_foot_mi, right_foot_mi]
        };
        let nc = stance.len();
        // Unloading ramp. Dropping the swing foot's rows in a single tick
        // hands its whole share of the load to a stance foot whose CoP box is
        // already pinned at the lateral edge, and the level-0 cone loses its
        // interior in one step. Ramping the swing foot's force ceiling to
        // zero *before* it leaves the set lets the remaining box tighten
        // gradually.
        let unload = if lift_leg && unload_ramp > 0.0 {
            ((t - (t_shift - unload_ramp)) / unload_ramp).clamp(0.0, 1.0)
        } else {
            0.0
        };
        let load_share = move |foot_mi: usize| -> f64 {
            if foot_mi == right_foot_mi {
                1.0 - unload
            } else {
                1.0
            }
        };
        if single {
            anchors.release(1); // right foot is swinging; forget its anchor
        }
        let (j_contact, dj_v) =
            contact_jacobians(&rig.model, q, v, data, v_dvec, &stance, nv);

        // Level 0's equality block, assembled explicitly so its conditioning
        // can be watched. A Clarabel NumericalFailure at level 0 is a
        // conditioning failure, so this is the matrix to look at.
        if flag("CONDCHK", false) && tick % 5 == 0 {
            let nx = nv + 6 * nc + na_count;
            let mut a0 = na::DMatrix::zeros(nv + 6 * nc, nx);
            for r in 0..nv {
                for c in 0..nv {
                    a0[(r, c)] = st.mass[(r, c)];
                }
                for c in 0..6 * nc {
                    a0[(r, nv + c)] = -j_contact[(c, r)];
                }
            }
            for i in 0..na_count {
                a0[(6 + i, nv + 6 * nc + i)] = -1.0;
            }
            for r in 0..6 * nc {
                for c in 0..nv {
                    a0[(nv + r, c)] = j_contact[(r, c)];
                }
            }
            let sv = a0.singular_values();
            let (mx, mn) = (sv.max(), sv.min());
            let jsv = j_contact.clone().singular_values();
            println!(
                "  [cond] t={t:6.3} nc={nc}  A0 cond={:10.1} (sigma_min {:.3e})   Jc cond={:8.1}",
                mx / mn, mn, jsv.max() / jsv.min()
            );
        }

        let dyn_ctx = Dynamics::new(Formulation::Explicit, &st.mass, &st.h, &j_contact, na_count);
        let forces = dyn_ctx.forces();

        // The equation of motion is `Task::soft_eq` -- a least-squares term,
        // not a constraint. Sharing a level with the cones and the torque box
        // means the solver may trade physics against them, and it does: the
        // objective there is 1/2||EoM residual||^2 + 1/2||contact accel
        // residual||^2 + 1/2||slack||^2, summed with no weights across N*m,
        // m/s^2 and N. HoQp has no hard-equality facility, but it does not
        // need one: give the EoM a level of its own ABOVE everything else and
        // its least-squares problem has nothing to trade against, so the
        // residual goes to zero -- and every later level is then confined to
        // null(A_eom), which preserves it exactly.
        let eom_task = dyn_ctx.dynamics_task().expect("Explicit keeps the EoM task");
        let base = if eom_hard {
            tasks::box_bound(dyn_ctx.tau(), &rig.torque_max)
        } else {
            eom_task.clone() + tasks::box_bound(dyn_ctx.tau(), &rig.torque_max)
        };
        let p0 = bt::contact_level(
            &dyn_ctx,
            base,
            &j_contact,
            &dj_v,
            data,
            v_dvec,
            &stance,
            left_foot_mi,
            &mut anchors,
            &load_share,
            &contact_cfg,
        );

        // ---- P1: the CoM task = balance (x,y) AND squat (z) ------------
        let phase = 2.0 * PI * t / period_s;
        let z_ref = com_ref0.z - squat_amp * (1.0 - phase.cos()) * 0.5;
        let zd_ref = -squat_amp * 0.5 * (2.0 * PI / period_s) * phase.sin();
        let zdd_ref = -squat_amp * 0.5 * (2.0 * PI / period_s).powi(2) * phase.cos();
        // Move the CoM over the stance foot BEFORE releasing the other one.
        //
        // The target is LATCHED on the first tick, not re-read from the foot
        // every tick. Reading it live puts the plant inside the reference:
        // the foot link origin sits above the sole, so the moment the ankle
        // rolls the origin swings sideways and the CoM target swings with it.
        // Measured on G1, the stance foot rolled 4.6 mm 0.2 s after lift-off
        // and the target jumped 37 mm inboard in two ticks; the QP tracked it
        // faithfully, with zero degraded solves, straight into a fall. A
        // balance target has to be a fixed point in the world -- if the foot
        // moves, that is a disturbance to reject, not a new goal to chase.
        let stance_y = *stance_y_cell
            .get_or_insert_with(|| misarta::se3::translation(&data.oMi[left_foot_mi]).y);
        let y_ref = if lift_leg {
            let stance_y = if latch_stance {
                stance_y
            } else {
                misarta::se3::translation(&data.oMi[left_foot_mi]).y
            };
            let a = (t / t_shift).clamp(0.0, 1.0);
            let a = 0.5 - 0.5 * (PI * a).cos(); // smooth ramp
            com_ref0.y + a * (stance_y - com_ref0.y)
        } else {
            com_ref0.y
        };
        let lean = com_dx * (t / 2.0).clamp(0.0, 1.0); // ramp in over 2 s
        let c_ref = na::Vector3::new(com_ref0.x + lean, y_ref, z_ref);
        let cd_ref = na::Vector3::new(0.0, 0.0, zd_ref);
        let cdd_ref = na::Vector3::new(0.0, 0.0, zdd_ref);
        let a_com = com_sign * (cdd_ref + kd_com * (cd_ref - com_vel) + kp_com * (c_ref - com));
        let p1 = bt::com(dyn_ctx.qddot(), &st.j_com, &st.djv_com, &a_com);

        // ---- P2: trunk upright, via the WORLD-frame angular Jacobian ---
        let j_trunk =
            misarta::jacobian::compute_joint_jacobian_from_data(&rig.model, q, data, rig.trunk_mi);
        let dj_trunk =
            misarta::jacobian::compute_joint_jacobian_time_derivative(&rig.model, q, v, rig.trunk_mi);
        let djv_trunk = &dj_trunk * v_dvec;
        // The Jacobian comes from misarta's FK, so on the face of it the
        // ERROR should too -- reading the base attitude from MuJoCo mixes two
        // sources. Measured, it is worse: kyo46rs single-leg goes SURVIVED ->
        // FELL (46 degraded / 0.146 rad -> 77 / 0.559). The two should agree
        // exactly, since misarta's q was synced from that same quaternion one
        // line earlier, and the fact that swapping them changes the outcome
        // at all says they do NOT. Unresolved; default keeps what works.
        let (roll, pitch) = if rig.trunk_from_base && !attitude_from_fk {
            let (r, p, _) = body_quat.euler_angles();
            (r, p)
        } else {
            let rot = misarta::se3::rotation_matrix(&data.oMi[rig.trunk_mi]);
            (rot[(2, 1)].atan2(rot[(2, 2)]), (-rot[(2, 0)]).asin())
        };
        let rp_ref = bt::trunk_rp_ref(roll, pitch, &v_ang_w, kp_trunk, kd_trunk, trunk_dead, trunk_sign);
        let p2 = bt::trunk(dyn_ctx.qddot(), &j_trunk, &djv_trunk, &rp_ref, nv);

        // ---- P4: weak posture, so the null space does not wander -------
        let p3 = bt::posture(
            dyn_ctx.qddot(),
            &rig.actuated(),
            &rig.robot.joint_positions,
            &rig.q_seed,
            v,
            kp_post,
            kd_post,
            na_count,
            nv,
        );

        // ---- lowest: gravity-comp torque + nominal weight split --------
        let tau_gravity = rig.gravity_torque(q);
        // Ask each foot for the load that PUTS the net CoP under the CoM
        // reference, not for an equal share. An equal-share target is the
        // reason the CoP box saturated: the CoM task can be met either by
        // transferring load between the feet or by walking the CoP outward,
        // those are interchangeable in the task's null space, and a 50/50
        // force target makes the regulariser pick the second one every time.
        let lat: Vec<f64> = if lat_share {
            let ys: Vec<f64> = stance
                .iter()
                .map(|&mi| misarta::se3::translation(&data.oMi[mi]).y)
                .collect();
            match ys.len() {
                2 => {
                    let (y0, y1) = (ys[0], ys[1]);
                    let a = if (y0 - y1).abs() > 1e-6 {
                        ((y_ref - y1) / (y0 - y1)).clamp(0.0, 1.0)
                    } else {
                        0.5
                    };
                    vec![a, 1.0 - a]
                }
                _ => vec![1.0; ys.len()],
            }
        } else {
            vec![1.0; nc]
        };
        let shares: Vec<f64> = stance
            .iter()
            .copied()
            .zip(&lat)
            .map(|(mi, l)| load_share(mi) * l)
            .collect();
        let forces_nominal = bt::force_nominal(forces.size(), &shares, total_mass * G);
        let p_reg = bt::regulariser(dyn_ctx.tau(), &tau_gravity, &dyn_ctx.forces(), &forces_nominal);

        // ---- swing foot: hold it at a clearance above where it started --
        let p_swing = if single {
            let jf = misarta::jacobian::compute_joint_jacobian_from_data(
                &rig.model, q, data, right_foot_mi,
            );
            let djf = misarta::jacobian::compute_joint_jacobian_time_derivative(
                &rig.model, q, v, right_foot_mi,
            );
            let djv = &djf * v_dvec;
            let pos = misarta::se3::translation(&data.oMi[right_foot_mi]);
            let vel3 = &jf.rows(3, 3).into_owned() * v_dvec;
            let vel = na::Vector3::new(vel3[0], vel3[1], vel3[2]);
            // Ramp the clearance in rather than stepping it: releasing the
            // contact and jumping the target 40 mm in the same tick is a step
            // input, and its reaction lands straight on the stance foot's
            // narrow CoP budget.
            let a_lift = ((t - t_shift) / lift_ramp).clamp(0.0, 1.0);
            let a_lift = 0.5 - 0.5 * (PI * a_lift).cos();
            let tgt = swing_home + na::Vector3::new(0.0, 0.0, lift_h * a_lift);
            Some(bt::swing(
                dyn_ctx.qddot(),
                &jf,
                &djv,
                &pos,
                &vel,
                &tgt,
                &na::Vector3::zeros(),
                kp_sw,
                kp_sw,
                kd_sw,
                if swing_z_only { bt::SwingAxes::ZOnly } else { bt::SwingAxes::Xyz },
            ))
        } else {
            None
        };

        // Names alongside, because the INDEX moves: swing only exists in
        // single support and trunk/posture are switchable, so "level 4" is
        // the regulariser in double support and posture in single. Reading
        // the number without the configuration has already caused one
        // misreading in this file's history.
        let mut levels: Vec<misa_wbc::Task> = Vec::new();
        let mut level_names: Vec<&str> = Vec::new();
        if eom_hard {
            levels.push(eom_task);
            level_names.push("eom");
        }
        levels.push(p0.task);
        level_names.push(if eom_hard { "contact+cones" } else { "dynamics+contact+cones" });
        levels.push(p1);
        level_names.push("com");
        let mut p2_late = None;
        if use_trunk {
            if trunk_late {
                p2_late = Some(p2);
            } else {
                levels.push(p2);
                level_names.push("trunk");
            }
        }
        if let Some(ps) = p_swing {
            levels.push(ps);
            level_names.push("swing");
        }
        if use_post {
            levels.push(p3);
            level_names.push("posture");
        }
        if let Some(pt) = p2_late {
            levels.push(pt);
            level_names.push("trunk(late)");
        }
        levels.push(p_reg);
        level_names.push("regularise");

        let sol = solver
            .solve(&levels, &cfg)
            .unwrap_or_else(|e| panic!("wbc solve failed at t={t:.3}: {e}"));
        tally.observe(&sol.status, t, tick, nc, &level_names);
        let extracted = dyn_ctx.extract(&sol.x);

        // Where did the QP actually put the centre of pressure, and how much
        // of the box was left?
        if flag("COPCHK", false) && tick % 5 == 0 {
            let (lx, ly) = (sole_half_l * cop_frac, sole_half_w * cop_frac);
            let mut parts = Vec::new();
            for (slot, sel) in p0.sole_sel.iter().enumerate() {
                let w = sel * &extracted.forces;
                match cop_from_sole_wrench(&w) {
                    None => parts.push(format!("foot{slot}: fz~0")),
                    Some((cx, cy, fz)) => parts.push(format!(
                        "foot{slot} fz={fz:6.1}N cop=({:+6.1},{:+6.1})mm  use=({:5.2},{:5.2})",
                        cx * 1e3, cy * 1e3, cx.abs() / lx, cy.abs() / ly
                    )),
                }
            }
            println!("  [cop] t={t:6.3} nc={nc}  {}", parts.join("   "));
        }

        // CoP per foot in that foot's sole frame, side 0 = left, 1 = right,
        // as [x, y, fz]. The force variables themselves are WORLD frame (the
        // selection is what rotates them into the sole), so `f_qp_w` is
        // directly comparable with what MuJoCo's contacts sum to -- same
        // frame, same instant, same q.
        let mut cop_qp = [[0.0_f64; 3]; 2];
        let mut f_qp_w = [[0.0_f64; 3]; 2];
        for (slot, foot_mi) in stance.iter().copied().enumerate() {
            let side = rig.side_of(foot_mi);
            let w = &p0.sole_sel[slot] * &extracted.forces;
            if let Some((cx, cy, fz)) = cop_from_sole_wrench(&w) {
                if fz > 1e-6 {
                    cop_qp[side] = [cx, cy, fz];
                }
            }
            for k in 0..3 {
                f_qp_w[side][k] = extracted.forces[6 * slot + 3 + k];
            }
        }

        let measured = measure_contacts(&rig, data, friction_mu);

        // Self-collision, EVERY tick, not just at spawn.
        //
        // The spawn assert catches a robot braced against itself before it
        // moves. It cannot catch one that folds into itself while leaning,
        // and that is the pose single-leg stance spends its whole life in.
        // Trap 1 in doc/kyo46rs_biped_wbc.md is exactly this failure being
        // reported as a success, so the guard has to cover the motion too.
        {
            let hits: Vec<(String, String, f64)> = rig
                .sim
                .contacts()
                .into_iter()
                .filter(|c| !c.body1.is_empty() && !c.body2.is_empty())
                .map(|c| (c.body1.clone(), c.body2.clone(), c.force_mag))
                .collect();
            if !hits.is_empty() {
                n_selfcollide += 1;
                if n_selfcollide == 1 {
                    let d: Vec<String> = hits.iter().map(|(a, b, f)| format!("{a} <-> {b} ({f:.1} N)")).collect();
                    println!("  [self-collision] first at t={t:6.3}  {}", d.join(", "));
                }
                max_selfcollide_f = max_selfcollide_f.max(hits.iter().map(|(_, _, f)| *f).fold(0.0, f64::max));
            }
        }

        let mut robot_taus = vec![0.0_f64; rig.robot.joints.len()];
        for (ji, vi) in rig.actuated() {
            robot_taus[ji] = extracted.tau[vi - 6];
        }
        // NO_TORQUE=1 sends zeros: a free base on a real floor must collapse.
        // If it does not, something is holding the robot.
        if flag("NO_TORQUE", false) {
            robot_taus.iter_mut().for_each(|t| *t = 0.0);
        }
        {
            let rig_ref = &rig;
            let tg = &tau_gravity;
            let vv = v;
            let fallback = move |out: &mut [f64]| {
                gravity_plus_posture(rig_ref, tg, vv, hold_kp, hold_kd, out);
            };
            policy.apply(&sol.status, &mut robot_taus, t, dt, nc, &fallback);
        }

        // The row must be built before the plant steps: `com`, `data` and the
        // contact measurement all describe the state the torque was computed
        // FOR, and mixing in post-step positions would put two different
        // instants on one line.
        let trot = misarta::se3::rotation_matrix(&data.oMi[rig.trunk_mi]);
        let trunk_tilt = trot[(2, 1)].atan2(trot[(2, 2)]).abs().max((-trot[(2, 0)]).asin().abs());

        write_to_plant(&mut rig, ctrl_mode, &robot_taus, &extracted.qddot, v, dt);
        rig.step(mj_substeps);

        if log.is_enabled() {
            // What the contact constraint is ASKING the stance foot to do
            // (Baumgarte accel_ref, vertical row) against what the foot is
            // actually doing. Both feet unload together at the failure and
            // the CoM falls at the same time, which is only consistent with
            // the legs shortening -- so watch the demand, not just the result.
            let foot_z = rig.sim.body_world_position(prof.foot_links[0]).unwrap()[2];
            let foot_vz = rig
                .sim
                .body_world_linear_velocity(prof.foot_links[0])
                .map(|v| v[2])
                .unwrap_or(0.0);
            let swing_z = rig.sim.body_world_position(prof.foot_links[1]).unwrap()[2];
            let row = Row {
                t,
                com,
                com_ref_z: z_ref,
                com_ref_y: y_ref,
                tilt: roll.abs().max(pitch.abs()),
                trunk_tilt,
                n_stance: nc,
                foot_z,
                swing_z,
                foot_vz,
                acc_dbg: p0.acc_dbg,
                a_com,
                rp_ref,
                degraded: !matches!(sol.status, misa_wbc::SolveStatus::Optimal),
                cop_box: (sole_half_l * cop_frac, sole_half_w * cop_frac),
                cop_qp,
                f_qp_w,
                measured: &measured,
                taus: &robot_taus,
                extra: &[],
            };
            log.write(&rig, &row);
        }

        let cur_z = rig.sim.body_world_position(&rig.robot.root_link).unwrap()[2];
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
    println!("  degraded solves: {}", tally.n);
    tally.report();
    println!(
        "  self-collision ticks: {n_selfcollide}  (peak {max_selfcollide_f:.1} N) -- nonzero \
         means part of the load went through a contact the QP has no model for"
    );
    println!("  verdict: {}", if fell { "FELL" } else { "SURVIVED" });
}

#[cfg(not(feature = "mujoco"))]
fn main() {
    eprintln!("This example requires the `mujoco` feature. Run with:");
    eprintln!("  cargo run --features mujoco --example kyo46rs_com_squat");
}
