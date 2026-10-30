//! Biped WBC: stepping in place.
//!
//! Same hierarchical QP as `kyo46rs_com_squat`, with the three things that
//! made single-leg STANCE work replaced by things that can also WALK:
//!
//! | standing | stepping |
//! |---|---|
//! | lateral CoM target latched on tick 0 | DCM reference from the footstep plan |
//! | swing foot constrained in z only | full 3-D swing trajectory |
//! | contact anchors frozen at first touch | re-anchored at every touchdown |
//!
//! The common thread is that each frozen quantity becomes a PLANNED one. It
//! must not become a MEASURED one: reading the stance foot's position every
//! tick is what put the plant inside its own reference and cost a fall with
//! zero degraded solves (`doc/kyo46rs_biped_wbc.md` section 9.2). Everything
//! the reference is built from is captured once, from the settled pose, in
//! world coordinates.
//!
//! Staged bring-up, because each stage isolates one failure mode:
//!
//! ```text
//! NO_LIFT=1 T=25   # W1: weight shift only, both feet never leave the ground
//! LIFT_H=0  T=25   # W2: contact set switches, but the swing foot does not rise
//! T=25             # W3: the real thing
//! ```


#[cfg(feature = "mujoco")]
mod config_env;
#[cfg(feature = "mujoco")]
mod diag;
#[cfg(feature = "mujoco")]
mod metrics;
#[cfg(feature = "mujoco")]
mod scene;

// The walker (the control) is misa-biped's; this example is its harness:
// the plant and the scenario (scene.rs), the walker's knobs from the
// environment (config_env.rs), the measurements and the report
// (metrics.rs), the outputs besides stdout (diag.rs), and here the loop in
// the order of §2.5 of doc/from_claude/20261029_walker_extraction_plan.md.
#[cfg(feature = "mujoco")]
use misa_biped::walker;
#[cfg(feature = "mujoco")]
use nalgebra as na;
#[cfg(feature = "mujoco")]
use misa_biped::recovery::RefLib;

/// Read a push-recovery reference from scripts/trajopt/push_recovery.py
/// (JSON) into [`RefLib`] (misa-biped does not read JSON).
/// `REF_LIBRARY=<file>` loads one; see doc/from_claude/20261005_trajopt_reference_plan.md §9.
#[cfg(feature = "mujoco")]
fn load_ref_lib(path: &str) -> RefLib {
    let v: serde_json::Value = serde_json::from_str(&std::fs::read_to_string(path).expect("REF_LIBRARY file"))
        .expect("REF_LIBRARY json");
    let f = |x: &serde_json::Value| x.as_f64().unwrap();
    let vec3 = |a: &serde_json::Value| {
        a.as_array().unwrap().iter().map(|r| {
            let r = r.as_array().unwrap();
            na::Vector3::new(f(&r[0]), f(&r[1]), f(&r[2]))
        }).collect::<Vec<_>>()
    };
    let sole_l = vec3(&v["sole_l"]);
    let stance_y0 = sole_l[0].y;
    let sole_r = vec3(&v["sole_r"]);
    let swing_z0 = sole_r[0].z - sole_l[0].z;
    let com = vec3(&v["com"]);
    let fz: [Vec<f64>; 2] = {
        let fa = v["f"].as_array().map(|a| a.to_vec()).unwrap_or_default();
        let per = |side: usize| fa.iter().map(|k| k[side].as_array().unwrap().iter().map(|c| f(&c[2])).sum::<f64>()).collect::<Vec<f64>>();
        [per(0), per(1)]
    };
    RefLib {
        com,
        fz,
        swing_z0,
        swing_phase: v["swing_phase"].as_f64().unwrap_or(0.5),
        impulse_deg: v["impulse_deg"].as_f64().unwrap_or(90.0),
        landed: v["landed"].as_bool().unwrap_or(false),
        vx: v["vx"].as_f64().unwrap_or(0.12),
        phases: v["T"].as_array().unwrap().iter().map(f).collect(),
        t: v["t"].as_array().unwrap().iter().map(f).collect(),
        sole: [sole_l, sole_r],
        stance_y0,
    }
}

/// Environment knobs: a number (unset or unparsable = `d`), and a switch
/// (set to anything but "0" = on). Free functions rather than closures in
/// `main`, so the scenario builders below read them the same way.
#[cfg(feature = "mujoco")]
fn env_f64(k: &str, d: f64) -> f64 {
    std::env::var(k).ok().and_then(|s| s.parse().ok()).unwrap_or(d)
}
#[cfg(feature = "mujoco")]
fn flag(k: &str, d: bool) -> bool {
    std::env::var(k).map(|v| v != "0").unwrap_or(d)
}

/// The command (VX / VY / WZ, SQUAT_*, VX2*, JAB*), read here and woven into
/// the walker's plan at start-up.
#[cfg(feature = "mujoco")]
fn command_from_env(wcfg: &misa_biped::config::Config) -> walker::CommandSchedule {
    // Jab: sweep one arm from the guard to an extended pose and back, in the
    // order `JAB_PATTERN` gives (L/R, e.g. "LLRR" = left twice then right
    // twice). One character per `JAB_PERIOD`. Needs ARM_HOLD to have anything
    // to drive. Held off until `JAB_START` so the burn-in and the stride ramp
    // are not measuring a punch.
    let jab = flag("JAB", false);
    let jab_period = env_f64("JAB_PERIOD", 0.55);
    let jab_start = env_f64("JAB_START", 2.5);
    let jab_pitch = env_f64("JAB_PITCH", -1.3963); // -80 deg: arm out front
    let jab_elbow = env_f64("JAB_ELBOW", -0.1745); // -10 deg: nearly straight
    let jab_pattern = std::env::var("JAB_PATTERN").unwrap_or_else(|_| "LLRR".into());
    walker::CommandSchedule {
        // STRIDE is kept as a shorthand for VX in m/step.
        vx: env_f64("VX", wcfg.gait.stride / (wcfg.gait.t_ss + wcfg.gait.t_ds)),
        vy: env_f64("VY", 0.0),
        wz: env_f64("WZ", 0.0),
        // Crouch/rise while doing it, to load the sagittal chain during
        // locomotion rather than only when standing.
        squat_amp: env_f64("SQUAT_AMP", 0.0),
        squat_period: env_f64("SQUAT_PERIOD", 3.0),
        // Speed-change benchmark: from step VX2_AT the forward speed ramps
        // to VX2 over VX2_RAMP steps (gait::FootstepPlan::velocity_with_change).
        // Unset = the fixed-speed plan, bit-identical.
        vx2: std::env::var("VX2").ok().and_then(|v| v.parse::<f64>().ok()).map(|vx2| {
            (env_f64("VX2_AT", 8.0).max(0.0) as usize, vx2, env_f64("VX2_RAMP", 3.0).max(0.0) as usize)
        }),
        jab,
        jab_start,
        jab_period,
        jab_pattern,
        jab_pitch,
        jab_elbow,
    }
}

/// Between the solve and the plant: the degraded-solve policy and whether a
/// good command has been seen yet (harness side; see `CommandPolicy`).
#[cfg(feature = "mujoco")]
struct CommandGuard {
    policy: articara::biped::actuate::CommandPolicy,
    qp_has_good_command: bool,
}

/// The task stack's solver settings and its joint-limit CBF, from the
/// walker's Config (the WBC knobs) and the robot's limits. Checks the gates
/// against the control mode, and prints the CBF's operating band when
/// `JLIM_V_FRAC` slows it.
#[cfg(feature = "mujoco")]
fn stack_settings(
    wcfg: &misa_biped::config::Config,
    robot: &misa_biped::robot::RobotModel,
    ctrl_mode: articara::biped::rig::CtrlMode,
) -> (misa_wbc::SolveConfig, misa_wbc::tasks::JointLimitCbf) {
    use articara::biped::rig::CtrlMode;
    use misa_wbc::{SolveConfig, tasks};
    let na_count = robot.na;
    // Additional WBC backends must never bypass the independent acceptance gate.
    // QP_BENCH_GATE also enables the identical gate for existing baselines.
    let qp_gate = wcfg.wbc.qp_gate();
    assert!(
        !qp_gate || ctrl_mode == CtrlMode::Torque,
        "QP_BENCH_GATE requires torque control"
    );
    let mpc_gate = qp_gate || wcfg.mpc.bench_gate;
    assert!(
        !mpc_gate || ctrl_mode == CtrlMode::Torque,
        "MPC_BENCH_GATE requires torque control"
    );
    let feasibility_tol = wcfg.wbc.feas_tol;
    assert!(
        feasibility_tol.is_finite() && feasibility_tol > 0.0,
        "WBC_FEAS_TOL must be positive and finite"
    );
    let mut cfg = SolveConfig {
        feasibility_tol,
        validation_tol: qp_gate.then_some(1e-6),
        ..SolveConfig::default()
    };
    // Priority resolution. Default = the strict null-space cascade (shipped).
    // `weighted` = one QP with level k>=1 weighted WBC_WEIGHT_RATIO^-(k-1),
    // level 0 hard (misa_wbc::HqpStrategy::WeightedQp) -- plan v2 P4.
    // (Per-task weights are applied to the levels themselves, WBCW_*; the
    // `TaskStack` reads the strategy and the weights from `wcfg`.)
    cfg.strategy = wcfg.wbc.strategy;
    if let Some(backend) = wcfg.wbc.backend {
        cfg.backend = backend;
    }
    // Joint position/velocity safety limits (misa_wbc::tasks::joint_limit_cbf).
    // Off by default: only the torque box is enforced today, which is why a
    // redundant leg could be commanded into a solution only realisable by
    // running a joint past its physical stop -- measured on ankle_roll at
    // 196% of its URDF limit while falling (doc Sec.18.5). Added at the same
    // priority as the torque box (`base`, below), since both are "never
    // violate" safety constraints rather than tracked objectives. Gains are
    // a first guess, unverified on this robot -- doc Sec.21.7/25.4.
    let jlim = wcfg.joint_limit.enabled;
    let [jlim_alpha1, jlim_alpha2, jlim_alpha3] = wcfg.joint_limit.alpha;
    let jlim_amax = wcfg.joint_limit.a_max;
    let mut jlim_v_max = robot.v_max.clone();
    // Slow the CBF's velocity limit below the mechanical one. (The position
    // band JLIM_Q_MARGIN that shared this block was not adopted and is
    // removed; the message keeps its 0 deg.)
    let jlim_v_frac = wcfg.joint_limit.v_frac;
    if jlim_v_frac < 1.0 {
        assert!(jlim, "JLIM_V_FRAC requires JLIM=1");
        for row in 0..na_count {
            jlim_v_max[row] *= jlim_v_frac;
        }
        println!(
            "CBF operating band: {:.1} deg inside every position stop, v x{:.2}",
            0.0,
            jlim_v_frac
        );
    }
    let jlim_cfg = tasks::JointLimitCbf {
        q_min: robot.q_min.clone(),
        q_max: robot.q_max.clone(),
        v_max: jlim_v_max,
        a_max: na::DVector::from_element(na_count, jlim_amax),
        alpha1: na::DVector::from_element(na_count, jlim_alpha1),
        alpha2: na::DVector::from_element(na_count, jlim_alpha2),
        alpha3: na::DVector::from_element(na_count, jlim_alpha3),
    };
    (cfg, jlim_cfg)
}

/// The start-up checks on the walker's Config that the harness keeps (the
/// walker runs its own in `WalkerInit::new`).
#[cfg(feature = "mujoco")]
fn check_config(wcfg: &misa_biped::config::Config) {
    let com_replan_forward_gain = wcfg.com_plan.replan_forward_gain;
    assert!(com_replan_forward_gain.is_finite() && (0.0..=1.0).contains(&com_replan_forward_gain));
    let mpc_footstep = wcfg.mpc_footstep.on;
    // The one combination allowed: the contact_plan adjuster with ALL its
    // position boxes at zero is a pure step-RETIMER (its shift is exactly
    // zero, so it never touches the landing), and the MPC landing variable
    // has no timing freedom. Measured 2026-09-27 (design note, FX): the
    // landing variable matches v4.1 at 8 N.s but takes 0/24 at 11 N.s where
    // v4.1's adjuster shortens the single support (CP_T_FROM_X).
    let contact_plan_on = wcfg.contact_plan.on;
    let contact_plan_cfg = wcfg.contact_plan.planner();
    let contact_plan_timing_only = contact_plan_on
        && contact_plan_cfg.max_x == 0.0
        && contact_plan_cfg.max_y == 0.0
        && contact_plan_cfg.max_y_in == 0.0;
    assert!(
        !(mpc_footstep && contact_plan_on) || contact_plan_timing_only,
        "MPC_FOOTSTEP and the contact_plan adjuster are two authorities over the same landing; turn one off (or make the adjuster timing-only: CP_MAX_X=CP_MAX_Y=CP_MAX_Y_IN=0)"
    );
    let com_xy_jerk_max = wcfg.dcm.com_xy_jerk_max;
    assert!(
        com_xy_jerk_max.is_finite() && com_xy_jerk_max > 0.0 || com_xy_jerk_max == f64::INFINITY,
        "COM_XY_JERK_MAX must be positive"
    );
}

#[cfg(feature = "mujoco")]
fn main() {
    use articara::biped::actuate::{CommandPolicy, DegradedTally, gravity_plus_posture, write_to_plant};
    use articara::biped::gait::Support;
    use articara::biped::log::{TrajLog, read_contacts};
    use articara::biped::profile;
    use articara::biped::rig::BipedRig;
    use articara::biped::wbc_stack::{StackDiag, TaskStack, TaskStackConfig};
    use diag::DiagSinks;
    use metrics::RunMetrics;
    use misa_biped::robot::Observation;
    use misa_wbc::Solver;

    // ROUGH_MODE=1: the rough-terrain preset (config_env::rough_mode). The
    // walker's Config reads its four values over the knobs; the harness only
    // says so, first.
    if let Some(rough) = config_env::rough_mode(&|k| std::env::var(k).ok()) {
        println!("rough mode: SWING_Z_HOLD=1 SWING_Z_DROP=0.50,1.0 SWING_XY_DELAY=0.15 LIFT_H={}", rough[3].1);
    }

    let prof = profile::by_name(&std::env::var("ROBOT").unwrap_or_default());
    // The walker's knobs (misa_biped::config), read once from the
    // environment through the table in config_env.rs. The rest of this
    // start-up reads the rig, the scenario, the command and the diagnostics.
    let wcfg = config_env::config_from_kv(&prof, &|k| std::env::var(k).ok());
    // WALKER_CONFIG_DUMP=<path>: write the Config as read (`{:#?}`), for
    // checking a controls file against a preset. Opt-in and silent: the log
    // is compared byte for byte.
    if let Ok(path) = std::env::var("WALKER_CONFIG_DUMP") {
        std::fs::write(&path, format!("{wcfg:#?}\n")).expect("WALKER_CONFIG_DUMP");
    }


    // The plant (its build options and the profile's overrides).
    let (mut o, prof) = scene::plant_from_env(prof);
    // Leg reach (src/biped/reach.rs). `LEG_REACH` overrides the profile's
    // measured length; the margin is what a PLAN may ask for. Used for the
    // diagnostic columns always, and for the footstep bounds only when
    // REACH_LIMIT is on.
    let leg_reach = articara::biped::reach::LegReach {
        max: env_f64("LEG_REACH", prof.leg_reach).max(0.0),
        margin: wcfg.descent.leg_reach_margin,
    };
    // The sole's half-size (SOLE_HALF_L / SOLE_HALF_W over the profile's).
    let prof = config_env::profile_from_kv(prof, &|k| std::env::var(k).ok());
    let (sole_half_l, sole_half_w) = prof.cop_half;
    let mu_ground = o.mu_ground;
    let ctrl_mode = o.ctrl_mode;

    // The scene around the robot: the sparring target, the staircase or
    // slope of the planner's terrain model, and the unmodelled bumps.
    let scene::Scene { stairs, spar_wall } = scene::build_scene(&mut o);
    let mut rig = BipedRig::build(prof, &o);
    // The model the walker computes with (misa_biped::robot): the control
    // reads it, the harness keeps the plant (`rig`).
    let robot = rig.walker_model();
    // Model indices of the links the thighs hang from, for the reach columns.
    let hip_mi: Option<[usize; 2]> = prof.hip_links.map(|names| {
        names.map(|n| {
            *rig.link_to_idx
                .get(n)
                .unwrap_or_else(|| panic!("profile names hip link {n}, which this robot lacks"))
        })
    });
    let mj_dt = rig.mj_dt;
    let total_mass = robot.total_mass;

    // The WBC's solver settings and joint-limit CBF (the stack is built
    // below, after the walker's plan).
    let solver = Solver::new();
    let (solve_cfg, jlim_cfg) = stack_settings(&wcfg, &robot, ctrl_mode);
    let friction_mu: f64 = wcfg.wbc.friction_mu(mu_ground, prof.friction_margin);
    let total_t = env_f64("T", 25.0);

    // The disturbance (`PushScenario`) and what is measured about it.
    let push = scene::PushScenario::from_env(prof.root_link);
    let push_th = metrics::PushThresholds::from_env();

    let ctrl_dt = wcfg.wbc.ctrl_dt;
    let mj_substeps = (ctrl_dt / mj_dt).round().max(1.0) as u32;
    let dt = mj_substeps as f64 * mj_dt;
    let n_ticks = (total_t / dt) as usize;
    // Optional physics-rate audit. The normal trajectory checks only WBC ticks;
    // the simulator trace already contains q, qdot and applied torque after
    // every MuJoCo step, so no plant or controller behavior needs to change.
    let substep_audit_path = std::env::var("SUBSTEP_LIMIT_AUDIT_JSON").ok();
    if substep_audit_path.is_some() {
        rig.sim.set_trace_max(rig.sim.trace_len().max(mj_substeps as usize + 1));
    }
    println!(
        "control: {:.1} kHz plant / {:.0} Hz WBC ({mj_substeps} substeps per tick)",
        1e-3 / mj_dt,
        1.0 / dt
    );

    rig.report_footprint(sole_half_l);

    // ── The walker: its plan, captured ONCE from the settled pose, and
    // its initial state (the plan's lines are printed here).
    let command = command_from_env(&wcfg);
    // REF_LIBRARY: a push-recovery reference (scripts/trajopt). Engaged by
    // the detector (REF_TRIG_DCM) for its whole duration: the
    // gait is retimed to the reference's phases, the next two footfalls are
    // placed where it lands, the swing foot follows its sole trajectory, and
    // landings are taken by measured contact (CONTACT_EVENT) meanwhile. The
    // MPC and WBC otherwise run as usual on the re-planned DCM. Default
    // unset = bit-identical.
    // Several files (comma-separated) make a library: at the push the one
    // whose initial swing-sole height is closest to the measured one is used
    // (the phase of the swing is what the references differ in).
    let ref_libs: Vec<RefLib> = std::env::var("REF_LIBRARY")
        .ok()
        .map(|p| p.split(',').filter(|x| !x.is_empty()).map(load_ref_lib).collect())
        .unwrap_or_default();
    let ref_any = !ref_libs.is_empty();
    let init = walker::WalkerInit::new(
        wcfg.clone(),
        robot,
        walker::WalkerSetup {
            dt,
            t_end: total_t,
            terrain: stairs,
            command,
            recovery_refs: ref_libs,
            friction_mu,
            init_fk: rig.fk_now(),
            leg_reach,
            hip_mi,
        },
        &mut diag::StdSink,
    );

    // Every output of the run but stdout: TRAJ_CSV, then the opt-in
    // diagnostics (`DiagSinks`).
    let mut diag = DiagSinks::open(
        TrajLog::create(std::env::var("TRAJ_CSV").ok(), prof.log_joints.to_vec(), &diag::EXTRA_COLS),
        std::env::var("TICK_AUDIT_CSV")
            .ok()
            .map(|path| articara::biped::tick_audit::TickAudit::create(&path, &rig)),
        substep_audit_path,
        &rig,
    );
    let policy = CommandPolicy::new(
        rig.robot.joints.len(),
        wcfg.wbc.fallback_level,
        wcfg.wbc.hold_bridge,
        false, // hold_last (HOLD_LAST, not adopted)
        0,     // blend_ticks (BLEND_TICKS, not adopted)
    );
    let mut tally = DegradedTally::new();
    // FIX_*: the corrective-impulse probe (see `FixProbe`).
    let fix = scene::FixProbe::from_env();
    check_config(&wcfg);
    let ankle = metrics::AnkleRoll::new(&rig);

    // What the run measures about itself (achievement, extremes, the push
    // response), for the report after the loop.
    let p0_body = rig.sim.body_world_position(&rig.robot.root_link).unwrap();
    let yaw0_body = rig
        .sim
        .body_world_orientation(&rig.robot.root_link)
        .unwrap()
        .euler_angles()
        .2;
    let gp = articara::biped::gait::GaitParams {
        t_start: wcfg.gait.t_start,
        t_ds: wcfg.gait.t_ds,
        t_ss: wcfg.gait.t_ss,
        n_steps: wcfg.gait.n_steps,
        first_swing: wcfg.gait.first_swing,
        t_end: total_t,
    };
    // Time at which the last step ends, so achievement is measured over the
    // WALK and not diluted by the settle.
    let t_walk_end = init.plan().gait
        .slices
        .iter()
        .rposition(|s| matches!(s.support, Support::Single { .. }))
        .map(|i| init.plan().gait.slices[i].t1 + gp.t_ds)
        // No steps at all (a standing squat): the "walk" is the whole run, so
        // the achievement window is the whole run. Falling back to 0 made the
        // window 1 microsecond wide and turned a millimetre of drift into
        // 56 m/s.
        .unwrap_or(total_t);
    // Achievement is measured from the end of the stride ramp, not from
    // standing. Including the ramp charges every case a fixed shortfall that
    // never recovers -- it read 91% for every forward speed, and 47% for a
    // turn that was in fact holding 0.195 of a commanded 0.200 rad/s.
    let t_ramp_end = gp.t_start + (wcfg.gait.stride_ramp as f64) * (gp.t_ss + gp.t_ds);
    let mut m = RunMetrics::new(
        rig.robot.joints.len(),
        metrics::RunSetup {
            th: push_th,
            total_mass,
            sole_half: (sole_half_l, sole_half_w),
            friction_mu,
            mu_ground,
            dt,
            p0_body,
            yaw0_body,
            t_ramp_end,
            t_walk_end,
        },
    );
    // ACT_DELAY_TICKS=n: the plant receives the torques commanded n
    // control ticks ago (actuation latency, model-mismatch bench).
    // 0 = off = this tick's torques.
    let act_delay_ticks = env_f64("ACT_DELAY_TICKS", 0.0).max(0.0) as usize;
    let mut act_delay_buf: std::collections::VecDeque<Vec<f64>> = std::collections::VecDeque::new();

    // The WBC: the stack of QP levels (the solver, its warm start and the
    // contact anchors are its own), with the QP_DIAG_DIR settings and
    // whether QP_RUNTIME_CSV wants the physical violation measured.
    let stack = TaskStack::new(
        TaskStackConfig::new(
            &wcfg,
            solve_cfg,
            init.contact_params(),
            &jlim_cfg,
            total_mass,
            sole_half_l,
            dt,
            StackDiag {
                measure_violation: diag.qp_runtime_log.is_some(),
                dump: std::mem::take(&mut diag.qp_dump),
            },
        ),
        solver,
    );
    let mut guard = CommandGuard { policy, qp_has_good_command: false };
    let mut walker = init.with_stack(stack);

    // The walker's log lines go straight to stdout / stderr.
    let mut out = diag::StdSink;
    for tick in 0..n_ticks {
        let qp_control_start = std::time::Instant::now();
        let t = tick as f64 * dt;
        m.begin_tick(t, tally.n);
        // The state estimate (the sensor model, QD_CLAMP); the walker
        // assembles the dynamics and kinematics from it (S14).
        let est = rig.sense();
        // Ground truth first: the phase logic, the contact set and the tasks
        // all solve against this, so it is read before any of them. The CoP
        // in the sole frame needs the FK, so the measurement is finished
        // after the step (the plant's contacts do not change in between).
        let contacts = read_contacts(&rig);
        let measured_pre = contacts.fz();
        // What the walker is given this tick: the estimate, the vertical
        // contact forces, and the plant's joint angles (read by the posture
        // levels; `misa_biped::robot::Observation`).
        let obs = Observation {
            tick,
            estimate: &est,
            contact_fz: measured_pre,
            joint_pos_plant: &rig.robot.joint_positions,
        };
        diag.contact_dump(&rig, t);

        // ---- the walker's control (phases 1-12): its lines are printed as
        // they are decided, its records go to `diag`. It hands back the
        // state it assembled from the estimate.
        let walker::StepOutput {
            command: walker::Command { sol, extracted, tau_gravity, rejected: qp_rejected, degraded },
            record: rec,
            state: st,
        } = walker.step(&obs, &mut out, &mut diag);
        let measured = contacts.finish(&rig, &st.data, friction_mu);
        tally.observe(&sol.status, t, tick, rec.nc, &rec.level_names);
        let k = diag::Tick { t, dt, rec: &rec, st: &st, measured_pre, measured: &measured, degraded, total_mass };
        diag.transfer(&k, &rig, walker.stats());
        // The run's measurements from the record and the plant; the FIX
        // probe fires at the commitment instant or at its time after the
        // push (the measurement decides, the scenario applies it).
        let h_roll = || (walker.robot().cmm(&st) * &st.v_dvec)[0];
        if let Some(at_commitment) = m.observe_step(t, &rec, &measured, &h_roll, &fix) {
            fix.fire(&mut rig.sim, &push.link, total_mass, t, at_commitment);
        }
        // Self-collision, watched EVERY tick rather than only at spawn.
        m.observe_plant(&rig, t);

        let mut robot_taus = vec![0.0_f64; rig.robot.joints.len()];
        for (ji, vi) in rig.actuated() {
            robot_taus[ji] = extracted.tau[vi - 6];
        }
        {
            let rig_ref = &rig;
            let tg = &tau_gravity;
            let (v, hold_kp, hold_kd) = (&st.v, wcfg.wbc.hold_kp, wcfg.wbc.hold_kd);
            let fallback =
                move |out: &mut [f64]| gravity_plus_posture(rig_ref, tg, v, hold_kp, hold_kd, out);
            guard.policy.apply(&sol.status, &mut robot_taus, t, dt, rec.nc, &fallback);
            // Existing policy has no held command on its first failed tick.
            // Use its existing recomputed fallback instead of applying rejected QP torques.
            if qp_rejected && !guard.qp_has_good_command {
                fallback(&mut robot_taus);
            }
            if matches!(sol.status, misa_wbc::SolveStatus::Optimal) {
                guard.qp_has_good_command = true;
            }
        }
        // The fallback bypasses the QP torque box. Match the plant's
        // actuator range before sending and logging either control path.
        for (ji, vi) in rig.actuated() {
            let bound = rig.torque_max[vi - 6];
            robot_taus[ji] = robot_taus[ji].clamp(-bound, bound);
        }
        if guard.policy.consecutive_degraded() >= 10 {
            m.n_open_loop += 1;
        }
        let ankle_roll = m.observe_ankle(&ankle, &rig);
        m.observe_steps(&rig, t, &rec, &measured, degraded);

        // ---- disturbance: fire once, at the requested phase (`fire_push`).
        if m.push_at_t.is_none()
            && scene::fire_push(
                &mut rig.sim,
                &push,
                &scene::PushTick {
                    t,
                    step_idx: rec.step_idx,
                    support: rec.support,
                    slice_frac: rec.slice_frac,
                    slice_t1: rec.slice.t1,
                },
                total_mass,
                walker.omega(),
                walker.command().jab.then_some(walker.command()),
            )
        {
            m.push_at_t = Some(t);
            m.push_at_step = rec.step_idx;
            m.push_degraded_at = tally.n;
        }
        // The end of the tick for the walker (phase 13): a recovery
        // reference whose time is up hands the feet back to the gait.
        walker.end_tick(&mut out);

        m.support_prev = Some(rec.support);
        let pre = diag.before_plant(&k, &rig, walker.stats(), &sol, &extracted, &push.link, qp_control_start);
        if act_delay_ticks > 0 {
            act_delay_buf.push_back(robot_taus.clone());
            while act_delay_buf.len() > act_delay_ticks + 1 {
                act_delay_buf.pop_front();
            }
            let delayed = act_delay_buf.front().cloned().unwrap_or_else(|| robot_taus.clone());
            write_to_plant(&mut rig, ctrl_mode, &delayed, &extracted.qddot, &st.v, dt);
        } else {
            write_to_plant(&mut rig, ctrl_mode, &robot_taus, &extracted.qddot, &st.v, dt);
        }
        rig.step(mj_substeps);
        diag.after_plant(
            &k,
            &rig,
            &walker,
            &pre,
            &extracted,
            &robot_taus,
            mj_substeps,
            &jlim_cfg,
            friction_mu,
            ankle_roll,
            [m.hit_f_now, m.ep_imp, m.hit_impulse],
        );

        let (cur_z, tilt) = m.observe_pose(&rig, t, &rec);
        if tick % 40 == 0 {
            let sup = match rec.support {
                Support::Double => "DS  ".to_string(),
                Support::Single { stance, .. } => {
                    format!("SS{}", if stance == 0 { "L" } else { "R" })
                }
            };
            let com = st.com;
            println!(
                "  t={t:6.3} {sup} step={step_idx:3}  com=({:+.4},{:+.4},{:+.4})  \
                 xi=({:+.4},{:+.4}) err={:.4}  zmp=({:+.4},{:+.4})  cop_use={cop_use:.2}  {:?}",
                com.x, com.y, com.z, rec.xi.x, rec.xi.y, rec.dcm_err, rec.p_cmd.x, rec.p_cmd.y, sol.status,
                step_idx = rec.step_idx, cop_use = rec.cop_use
            );
        }
        // Height is judged against the surface under the body, or a robot
        // that WALKED down a staircase would be scored as having fallen it.
        let floor_z = stairs.as_ref().map_or(0.0, |s| s.h(st.com.x));
        if cur_z - floor_z < 0.30 || tilt > 0.52 {
            println!("  FELL at t={t:.3} (z={cur_z:.3}, floor={floor_z:.3}, tilt={tilt:.3})");
            m.fell = true;
            break;
        }
    }
    metrics::report(m, &walker, &tally, &rig, &push, spar_wall, ref_any);
    diag.finish(&rig, n_ticks, mj_substeps, mj_dt);
}

#[cfg(not(feature = "mujoco"))]
fn main() {
    eprintln!("This example requires the `mujoco` feature. Run with:");
    eprintln!("  cargo run --features mujoco --example kyo46rs_walk");
}
