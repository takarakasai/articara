//! articara's stack of QP levels behind the walker: [`TaskStack`] implements
//! misa-biped's [`WbcStack`] with the level builders of [`super::tasks`] and
//! [`super::contact`] and a stateful [`misa_wbc::Solver`].
//!
//! Moved on 2026-10-30 from phase 11 of the G1 walker
//! (`examples/kyo46rs_walk/walker.rs`, S12 of
//! doc/from_claude/20261029_walker_extraction_plan.md). The body of
//! [`TaskStack::solve`] is that phase's, statement for statement: the
//! dynamics, the torque box and the joint-limit CBF, the contact level (which
//! bleeds the anchors), the CoM, trunk, momentum, MOM_CMP, arm-hold and
//! posture levels, the nominal forces (the MPC's wrenches through the
//! contact selectors) and the regulariser, the swing level; their order and
//! weights; the solve, the physical gate, the QP_DIAG_DIR capture and the
//! WBC_LEVEL_DEBUG_T print. What the walker used to hand it through closures
//! (the load share, rigid or managed contact, the slope's heel cut) arrives
//! per side in the request; the anchors are the stack's own and the walker's
//! anchor decisions arrive as events, applied in order before the contact
//! level is built -- the same order as before (lift-off / touchdown, then a
//! confirmed landing, then the contact level's leak), since nothing between
//! the two read the anchors.
//!
//! The levels' ORDER (MOM_LEVEL, ARM_BEFORE_SWING, posture / trunk on or
//! off) and weights are a control decision that lives here for now (plan A
//! of §2.6).
use super::contact::{Anchors, contact_jacobians};
use super::tasks as bt;
use misa_biped::config::{Config, LevelWeights};
use misa_biped::dcm::G;
use misa_biped::wbc::{
    AnchorEvent, ContactMode, ContactParams, QpTimings, SwingAxes, WbcRequest, WbcSolution, WbcStack,
};
use misa_wbc::{Dynamics, Formulation, SolveConfig, tasks};
use nalgebra as na;

/// articara's contact configuration from the walker's contact model.
pub fn contact_cfg(p: &ContactParams) -> bt::ContactCfg {
    bt::ContactCfg {
        kp_c: p.kp_c,
        kd_c: p.kd_c,
        anchor_leak: p.anchor_leak,
        anchor_leak_rot: p.anchor_leak_rot,
        sole_offset_local: p.sole_offset_local,
        friction_mu: p.friction_mu,
        cop_half: p.cop_half,
        mu_torsion: p.mu_torsion,
        f_max_per_foot: p.f_max_per_foot,
        no_patch: p.no_patch,
        dt: p.dt,
    }
}

fn contact_mode(m: ContactMode) -> bt::ContactMode {
    match m {
        ContactMode::Rigid => bt::ContactMode::Rigid,
        ContactMode::Managed { tangent_weight, rotation_weight, normal_weight, normal_accel_max } => {
            bt::ContactMode::Managed { tangent_weight, rotation_weight, normal_weight, normal_accel_max }
        }
    }
}

fn swing_axes(a: SwingAxes) -> bt::SwingAxes {
    match a {
        SwingAxes::ZOnly => bt::SwingAxes::ZOnly,
        SwingAxes::Xyz => bt::SwingAxes::Xyz,
        SwingAxes::XyzYaw => bt::SwingAxes::XyzYaw,
        SwingAxes::XyzRollPitch => bt::SwingAxes::XyzRollPitch,
        SwingAxes::XyzYawPitch => bt::SwingAxes::XyzYawPitch,
        SwingAxes::Pose => bt::SwingAxes::Pose,
    }
}

/// QP_DIAG_DIR: snapshots of the task stack for offline numerical diagnosis
/// (`tick_NNNNNN.json`, numbered by `count`). QP_CAPTURE_*: opt-in continuous
/// sampling between `start` and `end` every `stride` ticks, at most `max`;
/// otherwise the legacy capture of the first 8 swing-level failures after
/// 2 s.
#[derive(Clone, Debug, Default)]
pub struct QpDump {
    pub dir: Option<String>,
    pub count: usize,
    pub capture_all: bool,
    pub start: f64,
    pub end: f64,
    pub stride: usize,
    pub max: usize,
}

/// What the stack's diagnostics are asked for (the harness's sinks): the
/// physical violation measured every tick (QP_RUNTIME_CSV reports it) and
/// the QP_DIAG_DIR snapshots. Neither changes the solve.
#[derive(Clone, Debug, Default)]
pub struct StackDiag {
    pub measure_violation: bool,
    pub dump: QpDump,
}

/// The stack's knobs, fixed at start-up.
pub struct TaskStackConfig {
    pub solve: SolveConfig,
    pub contact: bt::ContactCfg,
    /// JLIM: the joint-limit CBF beside the torque box.
    pub joint_limits: Option<misa_wbc::tasks::JointLimitCbf>,
    /// WBC_EOM_HARD: the equation of motion as its own top level.
    pub eom_hard: bool,
    /// QP_BENCH_GATE (or a native backend): every solve is validated
    /// against its physical rows and a violation is reported as degraded.
    /// This is the only switch that changes the result; `diag` only adds
    /// measurements.
    pub validate: bool,
    /// WBC_STRATEGY=weighted, with the per-level weights (WBCW_*).
    pub weighted: bool,
    pub level_weights: LevelWeights,
    /// The posture level (POSTURE) and its gains.
    pub posture: bool,
    pub kp_post: f64,
    pub kd_post: f64,
    /// The trunk level (TRUNK).
    pub trunk: bool,
    /// The arm hold's gains and whether it sits above the swing.
    pub kp_arm: f64,
    pub kd_arm: f64,
    pub arm_before_swing: bool,
    /// MOM_LEVEL: where the momentum level sits.
    pub mom_level: String,
    /// MOM_CMP_W: MOM_CMP's weight inside the CoM level.
    pub mom_cmp_w: f64,
    /// The physical rows' scales: the robot's mass and the sole's half
    /// length; and the control period (the snapshots report it).
    pub total_mass: f64,
    pub sole_half_l: f64,
    pub dt: f64,
    /// WBC_LEVEL_DEBUG_T.
    pub level_debug_t: Option<f64>,
    pub diag: StackDiag,
}

impl TaskStackConfig {
    /// The knobs as the walker's [`Config`] sets them; the solver's settings,
    /// the contact model, the CBF (used when JLIM is on), the physical
    /// scales and the diagnostics come from the harness's start-up.
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        wcfg: &Config,
        solve: SolveConfig,
        contact: &ContactParams,
        jlim_cfg: &misa_wbc::tasks::JointLimitCbf,
        total_mass: f64,
        sole_half_l: f64,
        dt: f64,
        diag: StackDiag,
    ) -> Self {
        TaskStackConfig {
            solve,
            contact: contact_cfg(contact),
            joint_limits: wcfg.joint_limit.enabled.then(|| jlim_cfg.clone()),
            eom_hard: wcfg.wbc.eom_hard,
            validate: wcfg.wbc.qp_gate(),
            weighted: matches!(wcfg.wbc.strategy, misa_wbc::HqpStrategy::WeightedQp { .. }),
            level_weights: wcfg.wbc.level_weights.clone(),
            posture: wcfg.gains.posture,
            kp_post: wcfg.gains.kp_post,
            kd_post: wcfg.gains.kd_post,
            trunk: wcfg.gains.trunk,
            kp_arm: wcfg.arm.kp,
            kd_arm: wcfg.arm.kd,
            arm_before_swing: wcfg.arm.before_swing,
            mom_level: wcfg.arm.mom_level.clone(),
            mom_cmp_w: wcfg.mom_cmp.w,
            total_mass,
            sole_half_l,
            dt,
            level_debug_t: wcfg.debug.wbc_level_t,
            diag,
        }
    }
}

/// The stack: its knobs, the solver (warm data), the QP structure of the
/// last tick (a change resets the solver) and the contact anchors.
pub struct TaskStack {
    pub cfg: TaskStackConfig,
    solver: misa_wbc::Solver,
    qp_previous_structure: Option<(Vec<usize>, Vec<(usize, usize, usize)>)>,
    anchors: Anchors,
}

impl TaskStack {
    pub fn new(cfg: TaskStackConfig, solver: misa_wbc::Solver) -> Self {
        TaskStack { cfg, solver, qp_previous_structure: None, anchors: Anchors::new() }
    }

    pub fn anchors(&self) -> &Anchors {
        &self.anchors
    }
}

impl WbcStack for TaskStack {
    fn anchor(&self, side: usize) -> Option<(na::Vector3<f64>, na::Matrix3<f64>)> {
        self.anchors.a[side]
    }

    fn solve(&mut self, req: &WbcRequest) -> WbcSolution {
        // The anchor changes the walker decided this tick, in its order.
        for ev in req.anchor_events {
            match ev {
                AnchorEvent::Release(side) => self.anchors.release(*side),
                AnchorEvent::Touchdown { side, pos, rot } => self.anchors.touchdown(*side, *pos, *rot),
            }
        }
        // `cfg` borrows the solver's settings alone: the QP_DIAG capture
        // below counts in `self.cfg.diag` while it reads them.
        let cfg = &self.cfg.solve;
        let scfg = &self.cfg;
        let eom_hard = scfg.eom_hard;
        let jlim = scfg.joint_limits.is_some();
        let kp_post = scfg.kp_post;
        let kd_post = scfg.kd_post;
        let mom_cmp_w = scfg.mom_cmp_w;
        let use_trunk = scfg.trunk;
        let mom_level = scfg.mom_level.clone();
        let arm_before_swing = scfg.arm_before_swing;
        let use_post = scfg.posture;
        let kp_arm = scfg.kp_arm;
        let kd_arm = scfg.kd_arm;
        let (dt, total_mass) = (scfg.dt, scfg.total_mass);
        let sole_half_l = scfg.sole_half_l;
        let robot = req.robot;
        let (nv, na_count) = (robot.nv, robot.na);
        let contact_cfg = &scfg.contact;
        let (qp_gate, weighted_levels) = (scfg.validate, scfg.weighted);
        let (t, tick, support) = (req.t, req.tick, req.support);
        let st = req.state;
        let (q, v, v_dvec, data) = (&st.q, &st.v, &st.v_dvec, &st.data);
        let stance_sides = req.stance_sides;
        let stance = req.stance;
        let load_share = |mi: usize| -> f64 { req.load_share[usize::from(mi != robot.foot_mi[0])] };
        let a_com = req.a_com;
        let (j_trunk, djv_trunk, rp_ref) = (req.trunk.j, req.trunk.djv, req.trunk.rp_ref);
        let coupled_active = req.coupled;
        let (tau_gravity, shares) = (req.tau_gravity, req.shares);
        let nc = stance.len();

        let (j_contact, dj_v) = contact_jacobians(&robot.model, q, v, data, v_dvec, stance, nv);
        let dyn_ctx = Dynamics::new(Formulation::Explicit, &st.mass, &st.h, &j_contact, na_count);
        let forces = dyn_ctx.forces();

        let eom_task = dyn_ctx
            .dynamics_task()
            .expect("Explicit keeps the EoM task");
        let base = if eom_hard {
            tasks::box_bound(dyn_ctx.tau(), &robot.torque_max)
        } else {
            eom_task.clone() + tasks::box_bound(dyn_ctx.tau(), &robot.torque_max)
        };
        let base = if let Some(jlim_cfg) = &scfg.joint_limits {
            // Live actuated-row position/velocity, read through model indices.
            let actuated = robot.actuated();
            let mut q_act = na::DVector::zeros(na_count);
            let mut v_act = na::DVector::zeros(na_count);
            for &(ji, vi) in &actuated {
                if let Some(mi) = robot.a2m[ji] {
                    q_act[vi - 6] = q[robot.model.q_idx[mi]];
                }
                v_act[vi - 6] = v[vi];
            }
            base + bt::joint_limits(
                dyn_ctx.qddot(),
                &actuated,
                &q_act,
                &v_act,
                jlim_cfg,
                na_count,
                nv,
            )
        } else {
            base
        };
        let p0 = bt::contact_level(
            &dyn_ctx,
            base,
            &j_contact,
            &dj_v,
            data,
            v_dvec,
            stance,
            robot.foot_mi[0],
            &mut self.anchors,
            &load_share,
            contact_cfg,
            // Managed or rigid, as the walker asked (STAIR_MANAGED_*: the
            // rate trigger's window).
            &|side: usize| contact_mode(req.contact_mode[side]),
            // The walker's CoP box for the foot (SLOPE_HEEL_CUT on a known
            // upslope), or the full sole.
            &|side: usize, _rot: &na::Matrix3<f64>| req.cop_patch[side],
        );

        let p1 = bt::com(dyn_ctx.qddot(), &st.j_com, &st.djv_com, &a_com);

        let p2 = bt::trunk_rpy(
            dyn_ctx.qddot(),
            j_trunk,
            djv_trunk,
            &rp_ref,
            req.trunk.yaw,
            nv,
        );


        // ---- the momentum task, MOM_CMP and the arm hold
        let p_mom = req.momentum.and_then(|m| {
            bt::angular_momentum(dyn_ctx.qddot(), &m.cmm, &m.bias, &m.h, &m.h_ref, &m.dh_ref, m.kp, m.axes, nv)
        });
        let p_cmp = req.mom_cmp.as_ref().and_then(|c| {
            bt::angular_momentum(dyn_ctx.qddot(), c.cmm, &c.r.bias, &c.r.h, &c.r.h_ref, &c.r.dh, 0.0, [true, true, false], nv)
        });
        let mut p_arm = req.arm_hold.map(|a| {
            bt::posture(dyn_ctx.qddot(), &a.arms, req.joint_pos, &a.q_ref, v, kp_arm, kd_arm, na_count, nv)
        });
        // ---- P4: posture ------------------------------------------------
        let p3 = bt::posture(
            dyn_ctx.qddot(),
            &robot.actuated(),
            req.joint_pos,
            &robot.q_seed,
            v,
            kp_post,
            kd_post,
            na_count,
            nv,
        );


        let mut forces_nominal = bt::force_nominal(forces.size(), shares, total_mass * G);
        if let Some(mpc) = req.mpc.as_ref() {
            let command = mpc.wrench;
            // contact_level's selector maps the native WBC wrench block
            // to the sole-centre frame used by the MPC. Invert its 6x6
            // slot block so the regulariser receives the same wrench.
            for (slot, &side) in stance_sides.iter().enumerate() {
                let selector = p0.sole_sel[slot].view((0, 6 * slot), (6, 6)).into_owned();
                if let Some(native) = selector.lu().solve(&command[side]) {
                    // MPC_SOLE_AUTHORITY: REPLACE the nominal instead of
                    // blending into it.
                    //
                    // `forces_nominal` is not a neutral regulariser
                    // target: it is built by splitting the load so the net
                    // CoP lands where the DCM controller asked (see above).
                    // Blending therefore averages TWO independent CoP
                    // decisions and realises neither -- which is why every
                    // blend fraction failed identically, down to 1/8 of
                    // the shipped one, with the fall time unmoved
                    // (measured 2026-09-25). BD and IHMC both keep one
                    // authority over the CoP. This hands it to the MPC.
                    //
                    // Restricted to double support by default: in single
                    // support the split is determined anyway (the two
                    // agree to 0.3 N there), so there is nothing to win
                    // and a failed MPC solve would have nowhere to fall
                    // back to.
                    // (MPC_AUTHORITY_ALL, authority in single support
                    // too, is removed.)
                    let authority = mpc.authority;
                    let w = if authority { 1.0 } else { mpc.blend };
                    let old = forces_nominal.rows(6 * slot, 6).into_owned();
                    let blended = old * (1.0 - w) + native * w;
                    forces_nominal.rows_mut(6 * slot, 6).copy_from(&blended);
                }
            }
        }
        let p_wrench_coupled = coupled_active.then(|| {
            tasks::regularize(&dyn_ctx.forces(), &forces_nominal)
                // 1.0: the default of STAIR_WRENCH_LEVEL_WEIGHT (removed).
                .weight(1.0 / (total_mass * G).powi(2))
        });
        let p_reg = if coupled_active {
            tasks::regularize(dyn_ctx.tau(), tau_gravity)
        } else {
            bt::regulariser(
                dyn_ctx.tau(),
                tau_gravity,
                &dyn_ctx.forces(),
                &forces_nominal,
            )
        };


        // ---- the swing level
        let p_swing = req.swing.map(|s| match s.acc {
            Some(acc) => bt::swing_with_pose_acceleration(
                dyn_ctx.qddot(),
                &s.jf,
                &s.djv,
                &s.pos,
                &s.vel,
                &s.tgt,
                &s.tgt_v,
                s.kp_xy,
                s.kp_z,
                s.kd,
                swing_axes(s.axes),
                s.rot,
                &acc,
            ),
            None => bt::swing_with_pose(
                dyn_ctx.qddot(),
                &s.jf,
                &s.djv,
                &s.pos,
                &s.vel,
                &s.tgt,
                &s.tgt_v,
                s.kp_xy,
                s.kp_z,
                s.kd,
                swing_axes(s.axes),
                s.rot,
            ),
        });

        // Keep the unweighted task for post-solve diagnostics; `p_swing`
        // itself is moved into the hierarchy below.
        let p_swing_diag = p_swing.clone();
        // Preserve original physical rows before task weighting / HQP slack:
        // to validate the solve (the gate), or because a diagnostic asked for
        // the violation or the snapshots (they do not change the solve).
        let keep_physical = scfg.diag.dump.dir.is_some() || qp_gate || scfg.diag.measure_violation;
        let qp_physical = keep_physical.then(|| {
            let mut units = vec!["N*m"; 2 * na_count];
            let mut scales = vec![(total_mass * G * sole_half_l).max(1.0); 2 * na_count];
            if jlim {
                units.extend(vec!["rad/s^2"; 2 * na_count]);
                scales.extend(vec![1.0; 2 * na_count]);
            }
            for _ in stance_sides {
                units.extend([
                    "N", "N", "N", "N", "N", "N", "N*m", "N*m", "N*m", "N*m", "N*m", "N*m",
                ]);
                scales.extend(vec![(total_mass * G).max(1.0); 6]);
                scales.extend(vec![(total_mass * G * sole_half_l).max(1.0); 6]);
            }
            // Optional per-foot normal force floor follows the contact rows.
            while units.len() < p0.task.d.nrows() {
                units.push("N");
                scales.push((total_mass * G).max(1.0));
            }
            assert_eq!(units.len(), p0.task.d.nrows(), "physical row metadata");
            (
                eom_task.clone(),
                p0.task.d.clone(),
                p0.task.f.clone(),
                units,
                scales,
            )
        });
        let mut levels: Vec<misa_wbc::Task> = Vec::new();
        let mut level_names: Vec<&'static str> = Vec::new();
        if eom_hard {
            levels.push(eom_task);
            level_names.push("eom");
        }
        let nd = p0.task.n_decision();
        levels.push(p0.task);
        level_names.push(if eom_hard {
            "contact+cones"
        } else {
            "dynamics+contact+cones"
        });
        // The coupled level's weights are the defaults of the removed
        // STAIR_WRENCH_COM_WEIGHT (10) and STAIR_WRENCH_MOM_WEIGHT (0.2);
        // STAIR_WRENCH_TRUNK_COUPLED (trunk merged too, default off) is gone.
        let mut p1_centroidal = if coupled_active {
            p1.weight(10.0)
        } else {
            p1
        };
        let mut mom_pending = p_mom;
        if let Some(pw) = p_wrench_coupled {
            p1_centroidal = p1_centroidal + pw;
            if let Some(pm) = mom_pending.take() {
                p1_centroidal = p1_centroidal + pm.weight(0.2);
            }
        }
        if let Some(pc) = p_cmp {
            p1_centroidal = p1_centroidal + pc.weight(mom_cmp_w);
        }
        levels.push(p1_centroidal);
        level_names.push(if coupled_active { "com+momentum+wrench" } else { "com" });
        if use_trunk {
            levels.push(p2);
            level_names.push("trunk");
        }
        // Where the momentum level sits in the stack. The task is an
        // EQUALITY at its level, so `kp` does not control how much of the
        // null space it takes -- even `kp=0` still demands `ḣ_ang = 0`, i.e.
        // "never change roll momentum", which a walk cannot honour.
        // Placement is therefore the real knob:
        //   `swing_before` / `swing_after` -- above posture, all authority
        //   `post_after`                   -- below posture, only what is left
        //   `post_merge`                   -- SAME level as posture, so the two
        //                                     trade off inside one least-squares
        //                                     instead of one strictly preceding
        //                                     the other
        let push_mom = |at: &str,
                        levels: &mut Vec<misa_wbc::Task>,
                        names: &mut Vec<&'static str>,
                        pending: &mut Option<misa_wbc::Task>| {
            if mom_level == at {
                if let Some(pm) = pending.take() {
                    levels.push(pm);
                    names.push("momentum");
                }
            }
        };
        push_mom(
            "swing_before",
            &mut levels,
            &mut level_names,
            &mut mom_pending,
        );
        // ARM_BEFORE_SWING: keep arm posture above swing (but below
        // CoM/trunk). Default stays below swing.
        if arm_before_swing {
            if let Some(pa) = p_arm.take() {
                levels.push(pa);
                level_names.push("arm hold(before swing)");
            }
        }
        if let Some(ps) = p_swing {
            levels.push(ps);
            level_names.push("swing");
        }
        push_mom(
            "swing_after",
            &mut levels,
            &mut level_names,
            &mut mom_pending,
        );
        if let Some(pa) = p_arm {
            levels.push(pa);
            level_names.push("arm hold");
        }
        if use_post {
            let merged = match (mom_level.as_str(), mom_pending.take()) {
                ("post_merge", Some(pm)) => {
                    level_names.push("posture+momentum");
                    p3 + pm
                }
                (_, back) => {
                    mom_pending = back;
                    level_names.push("posture");
                    p3
                }
            };
            levels.push(merged);
        }
        push_mom(
            "post_after",
            &mut levels,
            &mut level_names,
            &mut mom_pending,
        );
        levels.push(p_reg);
        level_names.push("regularise");
        // Weighted single QP (WBC_STRATEGY=weighted): each level below the
        // hard physics level gets a PER-TASK weight, because a geometric
        // ratio cannot express "swing as strong as the CoM, regularisation
        // far weaker" (measured 2026-09-24: ratios 1..1000 all fell in the
        // first step or before it). Override one with WBCW_<NAME> where NAME
        // is the level name upper-cased with non-alphanumerics as '_'
        // (e.g. WBCW_COM, WBCW_SWING, WBCW_POSTURE_MOMENTUM); the table was
        // read once at start-up and lists every name this stack produces
        // (`LevelWeights::NAMES`), so the default is never reached.
        // (WBC_WEIGHTED_OFF_STAIRS, the strict cascade inside the stair
        // zone, was not adopted: weighted everywhere = 112/112 on the flat
        // push bench but 0/192 on the stairs, measured 2026-09-24; removed.)
        if weighted_levels {
            for (k, lvl) in levels.iter_mut().enumerate().skip(1) {
                let name = level_names.get(k).copied().unwrap_or("");
                let wk = scfg.level_weights.get(name).unwrap_or_else(|| LevelWeights::default_for(name));
                let taken = std::mem::replace(lvl, misa_wbc::Task::empty(nd));
                *lvl = taken.weight(wk);
            }
        }

        let qp_structure = (
            stance_sides.to_vec(),
            levels
                .iter()
                .map(|v| (v.a.ncols(), v.a.nrows(), v.d.nrows()))
                .collect::<Vec<_>>(),
        );
        let qp_reset = self.qp_previous_structure
            .as_ref()
            .is_some_and(|old| old != &qp_structure);
        // The stateful solver keeps a workspace per level and only resets it
        // when the QP STRUCTURE changes. (WBC_QP_RESET_EVERY_TICK, dropping
        // the warm start every tick, was an experiment and is removed.)
        if qp_reset {
            self.solver.reset();
        }
        self.qp_previous_structure = Some(qp_structure);
        let qp_solve_start = std::time::Instant::now();
        let mut sol = self.solver
            .solve(&levels, cfg)
            .unwrap_or_else(|e| panic!("wbc solve failed at t={t:.3}: {e}"));
        let qp_solve_ns = qp_solve_start.elapsed().as_nanos();
        // WBC_LEVEL_DEBUG_T=<t>: print each level's least-squares residual
        // at the ticks within 11 ms of t (which task the solve could not
        // satisfy, and by how much).
        if let Some(tw) = scfg.level_debug_t
            && (t - tw).abs() < 0.011
        {
            let parts: Vec<String> = levels
                .iter()
                .enumerate()
                .map(|(k, lv)| {
                    let r = if lv.a.nrows() > 0 { (&lv.a * &sol.x - &lv.b).norm() } else { 0.0 };
                    let viol = if lv.d.nrows() > 0 {
                        (&lv.d * &sol.x - &lv.f).iter().fold(0.0_f64, |m, v| m.max(*v))
                    } else {
                        0.0
                    };
                    format!("{}={:.3e}/{:.1e}", level_names.get(k).copied().unwrap_or("?"), r, viol.max(0.0))
                })
                .collect();
            println!("  [wbc-levels] t={t:.3} support={support:?} {}", parts.join(" "));
        }
        let qp_check_start = std::time::Instant::now();
        let mut qp_physical_violation = 0.0_f64;
        if let Some((eom, d, f, _, scales)) = &qp_physical {
            for (i, value) in (&eom.a * &sol.x - &eom.b).iter().enumerate() {
                let scale = if i < 3 || i >= 6 {
                    total_mass * G * sole_half_l
                } else {
                    total_mass * G
                };
                qp_physical_violation = if value.is_finite() {
                    qp_physical_violation.max(value.abs() / scale.max(1.0))
                } else {
                    f64::INFINITY
                };
            }
            for (value, scale) in (d * &sol.x - f).iter().zip(scales) {
                qp_physical_violation = if value.is_finite() {
                    qp_physical_violation.max(*value / scale)
                } else {
                    f64::INFINITY
                };
            }
            if sol.x.iter().any(|v| !v.is_finite()) {
                qp_physical_violation = f64::INFINITY;
            }
        }
        let raw_almost = sol
            .level_diagnostics
            .iter()
            .flatten()
            .any(|d| d.raw_status == "AlmostSolved");
        let qp_rejected = qp_gate
            && (qp_physical_violation > 1e-6
                || !matches!(sol.status, misa_wbc::SolveStatus::Optimal)
                || raw_almost);
        if qp_rejected {
            // Route through the existing degraded-control policy; no new fallback law.
            sol.status = misa_wbc::SolveStatus::Degraded {
                level: 0,
                status: misa_wbc::QpStatus::NumericalFailure,
            };
        }
        let qp_check_ns = qp_check_start.elapsed().as_nanos();
        // Did the trunk task actually get what it asked for?
        //
        // The pitch diverges at lambda = 29.4 /s while the trunk task commands
        // an angular acceleration with kp=200, kd=40 -- poles at -5.9 and -34
        // if the command is MET. So either the command is not met (a higher
        // level is eating it) or the measured divergence is impossible. This
        // separates the two without guessing.
        let trunk_acc_rp = {
            let qdd = dyn_ctx.qddot().eval(&sol.x);
            let jr = j_trunk.rows(0, 2).into_owned();
            let acc = &jr * &qdd + djv_trunk.rows(0, 2).into_owned();
            [acc[0], acc[1]]
        };
        let degraded = !matches!(sol.status, misa_wbc::SolveStatus::Optimal);
        let qp_capture_timer = std::time::Instant::now();
        let dump = &mut self.cfg.diag.dump;
        if let Some(dir) = &dump.dir {
            let legacy_failure = t >= 2.0
                && dump.count < 8
                && matches!(sol.status,
                misa_wbc::SolveStatus::Degraded { level, .. } if level_names.get(level) == Some(&"swing"));
            let continuous = dump.capture_all
                && t >= dump.start
                && t <= dump.end
                && tick % dump.stride == 0
                && dump.count < dump.max;
            if continuous || (!dump.capture_all && legacy_failure) {
                let matrix = |m: &na::DMatrix<f64>| serde_json::json!({"rows":m.nrows(),"cols":m.ncols(),"data":m.as_slice()});
                let tasks: Vec<_> = levels.iter().zip(&level_names).map(|(task,name)| serde_json::json!({
                    "name":name,"a":matrix(&task.a),"b":task.b.as_slice(),"d":matrix(&task.d),"f":task.f.as_slice(),
                    "equality_semantics":"lexicographic_least_squares", "inequality_semantics":"hierarchical_slack",
                    "row_units":"weighted_task_units", "stored_scale":1.0})).collect();
                let variables: Vec<_> = dyn_ctx.layout().names().map(|name| {
                    let var=dyn_ctx.layout().var(name);
                    serde_json::json!({"name":name,"offset":var.offset(),"size":var.size(),
                        "components":match name {"qddot"=>"base angular rad/s^2, base linear m/s^2, joints rad/s^2", "f"=>"world link-origin [mx,my,mz,fx,fy,fz] per contact", _=>"joint torque N*m"}})
                }).collect();
                let physical = qp_physical.as_ref().map(|(eom,d,f,units,scales)| serde_json::json!({
                    "eom":{"a":matrix(&eom.a),"b":eom.b.as_slice(),"units":"base moment N*m, base force N, joint moment N*m"},
                    "inequalities":{"d":matrix(d),"f":f.as_slice(),"units":units,"normalization_scales":scales,
                        "semantics":"physical_hard_despite_hqp_slack","torque_rows":2*na_count,"cbf_rows":if jlim {2*na_count} else {0}},
                    "body_weight_N":total_mass*G,"reference_length_m":sole_half_l}));
                let snapshot = serde_json::json!({"schema_version":2,"t":t,"tick":tick,"ctrl_dt":dt,
                    "status":format!("{:?}",sol.status),"backend":format!("{:?}",cfg.backend),
                    "layer_statuses":sol.level_statuses.iter().map(|s| format!("{s:?}")).collect::<Vec<_>>(),
                    "layer_raw_statuses":sol.level_diagnostics.iter().map(|d| d.as_ref().map(|d| &d.raw_status)).collect::<Vec<_>>(),
                    "prox_weight":cfg.prox_weight,"tasks":tasks,"solution":sol.x.as_slice(),"variables":variables,
                    "contact_ids":stance_sides,"planned_support":format!("{:?}",support),"phase":"None",
                    "measured_fz_N":req.contact_fz,"physical_groups":physical,"solve_ns":qp_solve_ns,
                    "check_ns":qp_check_ns,"normalized_physical_violation":qp_physical_violation});
                std::fs::create_dir_all(dir).expect("QP_DIAG_DIR");
                std::fs::write(
                    format!("{dir}/tick_{:06}.json", dump.count),
                    serde_json::to_vec(&snapshot).unwrap(),
                )
                .expect("write QP snapshot");
                dump.count += 1;
            }
        }
        let qp_capture_ns = qp_capture_timer.elapsed().as_nanos();
        let extracted = dyn_ctx.extract(&sol.x);
        // The solved wrench per stance slot in the sole-centre frame, and
        // the swing level's residual (read by the walker's diagnostics).
        let sole_wrench: Vec<na::DVector<f64>> =
            (0..nc).map(|slot| &p0.sole_sel[slot] * &extracted.forces).collect();
        let swing_residual = p_swing_diag.as_ref().map(|task| &task.a * &sol.x - &task.b);
        WbcSolution {
            sol,
            extracted,
            level_names,
            forces_nominal,
            sole_wrench,
            acc_dbg: p0.acc_dbg,
            swing_residual,
            trunk_acc_rp,
            timings: QpTimings { solve_ns: qp_solve_ns, check_ns: qp_check_ns, capture_ns: qp_capture_ns },
            physical_violation: qp_physical_violation,
            reset: qp_reset,
            rejected: qp_rejected,
            degraded,
        }
    }
}
