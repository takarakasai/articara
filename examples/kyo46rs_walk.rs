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
fn main() {
    use articara::biped::actuate::{gravity_plus_posture, write_to_plant, CommandPolicy, DegradedTally};
    use articara::biped::contact::{contact_jacobians, cop_from_sole_wrench, Anchors};
    use articara::biped::dcm::{com_accel_xy, commanded_zmp, dcm_of, DcmPlan, SupportBox};
    use articara::biped::gait::{
        swing_position, swing_velocity, ContactCorrection, FootstepPlan, Footsteps, GaitParams,
        GaitPlan, Support,
    };
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
    o.ctrl_mode =
        CtrlMode::from_env_name(&std::env::var("CTRL_MODE").unwrap_or_else(|_| "torque".into()));
    o.sim_dt = env_f64("SIM_DT", 0.001);
    o.mu_ground = env_f64("MU_GROUND", 0.7);
    o.probe_z = env_f64("PROBE_Z", prof.probe_z);
    // Swing the arms out of the frontal plane. Measured: with the arms at
    // zero the forearm collides with the hip block as soon as the robot rolls
    // to step -- 301 ticks of it, peaking at 210 N, 3.2x body weight, all of
    // it invisible to the QP. The spawn-time self-collision assert passes,
    // because standing still is exactly the pose that clears.
    if flag("NO_ARM_COLLIDE", false) {
        o.uncollide_links = vec!["forearm", "upper_arm"];
    }
    // Narrow the stance by seeding hip roll. The lean the CoM must make to
    // get over one sole is atan(half_stance / leg_length), and ankle_roll has
    // to supply it while the sole stays flat -- so stance width and ankle_roll
    // travel are the same constraint seen twice.
    let hip_roll_seed = env_f64("HIP_ROLL_SEED", 0.0);
    let arm_pitch = env_f64("ARM_PITCH", -1.40);
    let elbow = env_f64("ELBOW", 0.0);
    if arm_pitch != 0.0 || elbow != 0.0 {
        o.extra_seed = vec![
            ("left_shoulder_pitch_joint", arm_pitch),
            ("right_shoulder_pitch_joint", arm_pitch),
            ("left_elbow_joint", elbow),
            ("right_elbow_joint", elbow),
        ];
    }
    if hip_roll_seed != 0.0 {
        // Signs mirror: both hips roll inward to bring the feet together, and
        // the ankles roll back so the soles stay flat on the floor.
        o.extra_seed.push((prof.hip_roll[0], -hip_roll_seed));
        o.extra_seed.push((prof.hip_roll[1], hip_roll_seed));
        o.extra_seed.push(("left_ankle_roll_joint", hip_roll_seed));
        o.extra_seed.push(("right_ankle_roll_joint", -hip_roll_seed));
    }

    // URDF override, for sweeping model variants without editing the shipped
    // package. Leaked because Profile stores &'static str; this runs once.
    let mut prof = prof;
    if let Ok(u) = std::env::var("URDF") {
        prof.urdf = Box::leak(u.into_boxed_str());
        println!("URDF override: {}", prof.urdf);
    }
    // A variant that adds a waist splits `torso` into a pelvis carrying the
    // legs and an upper body carrying the arms -- the same shape as G1, where
    // the FreeFlyer is the pelvis and the thing held upright is above the
    // waist. Both names have to move together or P2 regulates one body using
    // another's Jacobian.
    if let Ok(l) = std::env::var("ROOT_LINK") {
        prof.root_link = Box::leak(l.into_boxed_str());
    }
    if let Ok(l) = std::env::var("TRUNK_LINK") {
        prof.trunk_link = Some(Box::leak(l.into_boxed_str()));
    }
    let sole_half_l = env_f64("SOLE_HALF_L", prof.cop_half.0);
    let sole_half_w = env_f64("SOLE_HALF_W", prof.cop_half.1);
    let mu_ground = o.mu_ground;
    let ctrl_mode = o.ctrl_mode;

    let mut rig = BipedRig::build(prof, &o);
    let nv = rig.nv;
    let na_count = rig.na;
    let mj_dt = rig.mj_dt;
    let total_mass = rig.total_mass;

    let f_max_scale = env_f64("F_MAX_SCALE", 2.3);
    let f_max_per_foot = env_f64("F_MAX", f_max_scale * total_mass * G);
    assert!(
        f_max_per_foot >= total_mass * G,
        "f_max ({f_max_per_foot:.1} N) is below the single-support nominal ({:.1} N)",
        total_mass * G
    );

    // ── Control-law knobs ──────────────────────────────────────────────
    let mut solver = Solver::new();
    let cfg = SolveConfig::default();
    let friction_mu: f64 = env_f64("friction_mu", mu_ground * prof.friction_margin);
    let kp_com = env_f64("KP_COM", 300.0);
    let kd_com = env_f64("KD_COM", 80.0);
    let kp_trunk = env_f64("KP_TRUNK", 200.0);
    let kd_trunk = env_f64("KD_TRUNK", 40.0);
    let kp_post = env_f64("KP_POST", 100.0);
    let kd_post = env_f64("KD_POST", 20.0);
    let use_post = flag("POST", true);
    let use_trunk = flag("TRUNK", true);
    let trunk_sign = env_f64("TRUNK_SIGN", 1.0);
    let trunk_dead = env_f64("TRUNK_DEAD", 0.0);
    let trunk_late = flag("TRUNK_LATE", false);
    let eom_hard = flag("EOM_HARD", true);
    let cop_frac = env_f64("COP_FRAC", 1.0);
    let hold_kp = env_f64("HOLD_KP", prof.hold_kp);
    let hold_kd = env_f64("HOLD_KD", prof.hold_kd);
    let hold_bridge = env_f64("HOLD_BRIDGE", 8.0) as u32;
    let fallback_max_level = env_f64("FALLBACK_LEVEL", 999.0) as usize;
    let hold_last = flag("HOLD_LAST", false);
    let blend_ticks = env_f64("BLEND_TICKS", 0.0) as u32;
    let anchor_leak = env_f64("ANCHOR_LEAK", 0.2);
    let anchor_leak_rot = env_f64("ANCHOR_LEAK_ROT", 0.0);
    // Contact-stabiliser (Baumgarte) gain. 1600 was picked for a two-foot
    // stance and concentrated on one foot it kicks the robot off the ground --
    // the same finding that fixed G1's single-leg stance (6ba64d2). Measured
    // here, stepping in place at a 99.4 mm stance, T=40 / N_STEPS=40:
    //
    //   kp_c   60  100  120  150  180  220  250  400  600  1600
    //   steps   9   16   17   25   10   13   17    9    6     3
    //
    // 150 is the DEFAULT, not a tuned value. The sweep is non-monotone and
    // both neighbours drop hard, and the sensitivity is sharper than that:
    // kd_c = 24 gives 25 steps where kd_c = 2*sqrt(150) = 24.495 gives 18.
    // A 2% change in the damping is worth 7 steps, so no single number here
    // is a result. What the data supports is "1600 is wrong and anything in
    // 60-250 is several times better".
    let kp_c = env_f64("KP_CONTACT", 150.0);
    // Critically damped against kp_c. Moving the two independently is how the
    // pair drifted apart in the first place: kp_c=400 with the old kd_c=80
    // gives 4 steps, and the same kp_c with 2*sqrt(kp_c) gives 9.
    let kd_c = env_f64("KD_CONTACT", 2.0 * kp_c.sqrt());
    let no_patch = flag("NO_PATCH", false);

    // Swing. kp is split by axis: z is clearance and must not scuff, x/y is
    // placement, and while the stance leg is paying the reaction for both it
    // is x/y that can afford to be soft. Holding the swing foot's world x,y
    // HARD is what toppled the single-leg experiment (section 4.2) -- there
    // the target was frozen while the body translated 70 mm, which is a
    // kinematic conflict. Here the target is a planned footstep, so the
    // conflict is gone in principle; the split gain is how much that is
    // trusted in practice.
    let kp_sw_z = env_f64("KP_SWING", prof.kp_swing);
    let kp_sw_xy = env_f64("KP_SWING_XY", 0.25 * kp_sw_z);
    let kd_sw = env_f64("KD_SWING", prof.kd_swing);
    let lift_h = env_f64("LIFT_H", 0.5 * prof.lift_h);

    // DCM tracking. k is dimensionless: the DCM error decays as
    // exp(-k_dcm * omega * t), so 2.0 is two e-folds per DCM time constant.
    let k_dcm = env_f64("K_DCM", 2.0);
    // Keep the commanded ZMP inside the support polygon, shrunk by this
    // factor. Riding the exact edge means the next disturbance makes P0
    // infeasible outright.
    let zmp_margin = env_f64("ZMP_MARGIN", 0.8);
    let clamp_zmp = flag("CLAMP_ZMP", true);

    let total_t = env_f64("T", 25.0);
    let gp = GaitParams {
        t_start: env_f64("T_START", 2.0),
        t_ds: env_f64("T_DS", 0.20),
        t_ss: env_f64("T_SS", 0.45),
        n_steps: env_f64("N_STEPS", 20.0) as usize,
        first_swing: env_f64("FIRST_SWING", 1.0) as usize,
        t_end: total_t,
    };
    // W1: run the schedule (so the ZMP reference moves) but never actually
    // release a foot. Isolates "can the balance reference move at all" from
    // everything that touchdown brings with it.
    let no_lift = flag("NO_LIFT", false);

    // ---- disturbance: one rectangular push at a chosen gait phase --------
    //
    // Sized in IMPULSE (N*s), not force, because that is the quantity the
    // machine's recovery budget is denominated in: an ankle strategy can
    // absorb `m * omega * d_cop` of it and no more, which on this robot is
    // 6.63 * 5.585 * 0.019 = 0.70 N*s sideways against 1.8 N*s fore-aft.
    // Quoting a force instead hides the duration and makes two runs with the
    // same "10 N push" incomparable.
    let push_imp = env_f64("PUSH_IMPULSE", 0.0);
    // Horizontal direction, degrees: 0 = +x (forward), 90 = +y (left). The
    // whole circle is reachable, so diagonals are one number rather than a
    // pair of components that have to be kept consistent with each other.
    let push_deg = env_f64("PUSH_DEG", 90.0);
    // Yaw impulse (N*m*s) about +z, independent of the linear one so a pure
    // twist is expressible.
    let push_yaw_imp = env_f64("PUSH_YAW_IMPULSE", 0.0);
    let push_dt = env_f64("PUSH_DT", 0.10).max(1e-6);
    // WHEN. Not a wall-clock time: what the machine can absorb differs by
    // several times between double and single support, so a ladder that does
    // not pin the phase measures the phase as much as the impulse. Fire on
    // the first tick that is inside step `PUSH_STEP`, in the requested
    // support, at or past `PUSH_AT` of the way through that slice.
    let push_step = env_f64("PUSH_STEP", 5.0) as usize;
    let push_at = env_f64("PUSH_AT", 0.5).clamp(0.0, 1.0);
    // "ds" | "ss" | "any" -- which support phase to catch.
    let push_support = std::env::var("PUSH_SUPPORT").unwrap_or_else(|_| "ss".into());
    // Recovery is declared when the DCM error comes back under this and
    // STAYS under it -- a single tick dipping below the line is the error
    // crossing zero on its way past, not a recovery.
    let push_recover_m = env_f64("PUSH_RECOVER_MM", 20.0) * 1e-3;
    // How long it has to stay under. The DCM time constant is 1/omega =
    // 0.179 s on this machine, so anything shorter than a couple of those is
    // indistinguishable from the error passing through the band.
    let push_recover_s = env_f64("PUSH_RECOVER_S", 0.5);
    let push_link = std::env::var("PUSH_LINK").unwrap_or_else(|_| prof.root_link.to_string());

    let ctrl_dt = env_f64("CTRL_DT", prof.ctrl_dt);
    let mj_substeps = (ctrl_dt / mj_dt).round().max(1.0) as u32;
    let dt = mj_substeps as f64 * mj_dt;
    let n_ticks = (total_t / dt) as usize;
    println!(
        "control: {:.1} kHz plant / {:.0} Hz WBC ({mj_substeps} substeps per tick)",
        1e-3 / mj_dt,
        1.0 / dt
    );

    rig.report_footprint(sole_half_l);

    // ── Plan, captured ONCE from the settled pose ──────────────────────
    let d0 = rig.fk_now();
    let com0 = rig.com_of(&d0);
    let steps = Footsteps::from_fk(&d0, rig.foot_mi, prof.sole_centre_x, prof.sole_below_origin);
    // The orientation each foot is PLANNED to land in, captured once from the
    // settled pose. Stepping in place does not turn, so this never changes --
    // which is the point: it is a plan, not a measurement, and a swing foot
    // that drifts is correcting toward it rather than tracking its own drift.
    let foot_rot0: [na::Matrix3<f64>; 2] =
        std::array::from_fn(|s| misarta::se3::rotation_matrix(&d0.oMi[rig.foot_mi[s]]));
    // Rotate the nominal foot orientation to a planned yaw. A turn moves the
    // footstep AND the direction the foot points; commanding the original
    // orientation would land the foot square to the world under a turned body.
    let foot_rot_at = |side: usize, yaw: f64| -> na::Matrix3<f64> {
        let d = yaw - foot_rot0[side][(1, 0)].atan2(foot_rot0[side][(0, 0)]);
        na::Rotation3::from_euler_angles(0.0, 0.0, d).matrix() * foot_rot0[side]
    };
    // Swing orientation gains. Without this task the foot's three rotational
    // DoF are free and the QP twists the leg: measured, hip_yaw walked to its
    // +-30 deg stop over 16 steps and the feet landed yawed by up to 31 deg.
    let _yaw0 = {
        let q = rig.sim.body_world_orientation(&rig.robot.root_link).unwrap();
        q.euler_angles().2
    };
    let kp_sw_rot = env_f64("KP_SWING_ROT", 400.0);
    let kd_sw_rot = env_f64("KD_SWING_ROT", 40.0);
    // 0 = translation only, 1 = + yaw (4 rows), 2 = full pose (6 rows).
    let swing_rot_mode = env_f64("SWING_ROT", 1.0) as u32;
    // Regulate the trunk's yaw as well as its roll and pitch.
    let trunk_yaw = flag("TRUNK_YAW", true);
    let kp_yaw = env_f64("KP_YAW", 200.0);
    let kd_yaw = env_f64("KD_YAW", 40.0);
    let mut gait = GaitPlan::new(&gp);
    let z_com = env_f64("Z_COM", com0.z);
    // How far the ZMP is asked to travel laterally, as a fraction of the
    // half stance width. 1.0 is true single support (all load on one sole);
    // anything less is only reachable with both feet down, so it is a W1
    // instrument, not a gait parameter.
    let zmp_lat_scale = env_f64("ZMP_LAT_SCALE", 1.0);
    // Forward travel per step, in metres of BODY advance. 0 is stepping in
    // place, which stays a special case of this rather than a separate path.
    // A commanded speed maps to it as stride = v * (t_ss + t_ds).
    // Velocity command. STRIDE is kept as a shorthand for VX in m/step.
    let t_step = gp.t_ss + gp.t_ds;
    let stride = env_f64("STRIDE", 0.0);
    let vx = env_f64("VX", stride / t_step);
    let vy = env_f64("VY", 0.0);
    let wz = env_f64("WZ", 0.0);
    // Crouch/rise while doing it, to load the sagittal chain during locomotion
    // rather than only when standing.
    let squat_amp = env_f64("SQUAT_AMP", 0.0);
    let squat_period = env_f64("SQUAT_PERIOD", 3.0);
    // Steps over which the stride ramps in from standing.
    let stride_ramp = env_f64("STRIDE_RAMP", 6.0) as usize;
    // ALTERNATE=0 restores the step-and-close plan, for A/B.
    let footsteps = if flag("ALTERNATE", true) {
        FootstepPlan::velocity(&gait, &steps, vx, vy, wz, t_step, stride_ramp)
    } else {
        FootstepPlan::ramped_stride(&gait, &steps, vx * t_step, stride_ramp)
    };
    let dcm_plan = DcmPlan::from_footsteps(&gait, &footsteps, z_com, zmp_lat_scale);
    let omega = dcm_plan.omega;
    println!(
        "gait: {} steps, DS {:.2}s / SS {:.2}s (start {:.2}s), first swing = {}",
        gp.n_steps,
        gp.t_ds,
        gp.t_ss,
        gp.t_start,
        if gp.first_swing == 0 { "left" } else { "right" }
    );
    if zmp_lat_scale != 1.0 {
        println!("ZMP_LAT_SCALE={zmp_lat_scale}: the plan does NOT reach true single support");
    }
    println!(
        "lipm: z_com={z_com:.4} m  omega={omega:.3} rad/s (1/omega={:.3} s)  k_dcm={k_dcm}",
        1.0 / omega
    );
    println!(
        "footsteps (sole centres, world): L=({:+.4},{:+.4})  R=({:+.4},{:+.4})  stance width={:.1} mm",
        steps.sole[0].x, steps.sole[0].y, steps.sole[1].x, steps.sole[1].y,
        (steps.sole[0].y - steps.sole[1].y).abs() * 1e3
    );
    if vx != 0.0 || vy != 0.0 || wz != 0.0 || squat_amp != 0.0 {
        println!(
            "command: vx={vx:+.3} vy={vy:+.3} m/s  wz={wz:+.3} rad/s  squat={:.0} mm @ {squat_period:.1} s",
            squat_amp * 1e3
        );
    }
    if stride != 0.0 {
        println!(
            "stride: {:.3} m/step -> {:.3} m/s commanded, {:.2} m of travel planned over {} steps",
            stride,
            stride / (gp.t_ss + gp.t_ds),
            footsteps.travel_x(),
            gp.n_steps
        );
    }
    if no_lift {
        println!("NO_LIFT: the schedule drives the ZMP reference, but both feet stay in contact");
    }

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

    let extra_cols = [
        "xi_x", "xi_y", "xi_ref_x", "xi_ref_y", "zmp_ref_x", "zmp_ref_y",
        "zmp_cmd_x", "zmp_cmd_y", "zmp_clamp", "dcm_err", "support", "step",
        "share_l", "share_r", "swing_tgt_x", "swing_tgt_y", "swing_tgt_z",
        "ankle_roll_l", "ankle_roll_r",
    ];
    let mut log = TrajLog::create(
        std::env::var("TRAJ_CSV").ok(),
        prof.log_joints.to_vec(),
        &extra_cols,
    );
    let mut policy = CommandPolicy::new(
        rig.robot.joints.len(),
        fallback_max_level,
        hold_bridge,
        hold_last,
        blend_ticks,
    );
    let mut tally = DegradedTally::new();
    let mut anchors = Anchors::new();
    // Solve against the contact set the FEET report, not the one the schedule
    // assumes. CONTACT_DRIVEN=0 restores the schedule-only behaviour for A/B.
    let contact_driven = flag("CONTACT_DRIVEN", true);
    let mut correction = ContactCorrection::new(total_mass * G, env_f64("CONTACT_TICKS", 2.0) as u32);
    // Capture-point footstep adaptation. Default OFF because it was measured
    // and does not earn its place: the ZMP clamp it exists to replace turns
    // out to be a SYMPTOM, not the cause. See doc section 13.7 -- at the
    // sustainable speed the clamp never binds and this fires zero times, and
    // above it the clamp binds once, at the collapse, when a 67 mm step
    // adjustment is already too late. Kept because the mechanism is right and
    // will be needed once whatever actually limits the speed is found.
    let adapt = flag("ADAPT_STEP", false);
    // Reachability. Not a tuning knob -- past this the leg cannot get there,
    // and a target it cannot reach is worse than a nominal one.
    let adapt_max_x = env_f64("ADAPT_MAX_X", 0.06);
    let adapt_max_y = env_f64("ADAPT_MAX_Y", 0.03);
    let mut footsteps = footsteps;
    let mut dcm_plan = dcm_plan;
    let mut adapt_now = na::Vector2::zeros();
    // How much of `adapt_now` is already committed to the footstep plan.
    //
    // The adaptation has to be IN the plan, not added on top of it. If the
    // foot target moves but the DCM reference still describes the nominal
    // footstep, the ZMP feedback and the step adjustment see the same error
    // and fight over it -- which is exactly what made the first two attempts
    // at this worse than doing nothing (section 13.7). Every method in the
    // literature regenerates the reference from the adapted decision
    // variables: Khadiv et al. re-solve every control cycle and the solution
    // IS the reference, so the CoP layer only ever sees the residual.
    let mut adapt_applied = na::Vector2::zeros();
    // Rebuild the plan continuously rather than only at touchdown.
    let adapt_live = flag("ADAPT_LIVE", true);
    // ---- contact-driven phase transitions -----------------------------
    //
    // The schedule says when the step ENDS; the ground says when it actually
    // ended. Measured on this robot, the swing foot lands about 30 ms early
    // on a 0.35 s single support -- 8.6% of the phase, every step, same sign
    // -- and a time-based schedule keeps commanding a swing that has already
    // finished while the QP solves against a contact set that is a step
    // behind. Terminating the swing on contact is what a passive-ankle biped
    // does in hardware (Kim et al. 2019, arXiv:1901.08100: contact switches
    // "terminate swing foot motion controls when the swing foot touches the
    // ground earlier than anticipated").
    let phase_by_contact = flag("PHASE_BY_CONTACT", false);
    // A late foot gets the step extended rather than the contact set being
    // told a lie, but not forever -- past this the swing is not coming down
    // and holding the phase would only stall the robot in single support.
    let phase_extend_max = env_f64("PHASE_EXTEND", 0.5);
    // A touchdown is only believed in the last part of the swing. The naive
    // version -- "any load above 10% of body weight ends the step" -- fires on
    // a mid-swing scuff, and measured, it cut 61 ms off every 350 ms step and
    // took the gait from 200 steps to 46. The foot brushing the ground on its
    // way past is not the end of the step.
    let phase_min_frac = env_f64("PHASE_MIN_FRAC", 0.75);
    // ...and it has to be a real load, held for a few ticks, not a graze.
    let phase_load_frac = env_f64("PHASE_LOAD", 0.30);
    let phase_ticks = env_f64("PHASE_TICKS", 3.0) as u32;
    let mut land_count = 0u32;
    let mut n_early = 0u32;
    let mut n_late = 0u32;
    let mut retime_total = 0.0_f64;
    let mut max_adapt: f64 = 0.0;
    let mut n_adapt = 0u32;
    let mut n_replan = 0u32;
    let mut n_corrected = 0u32;

    // ankle_roll range of motion. The model review measured that holding the
    // sole flat already consumes 82% of the +-20 deg travel, and stepping
    // adds lateral sway on top. If this saturates, that is a MODEL limit and
    // the answer is a wider joint, not a different gain.
    let ankle_roll_names = ["left_ankle_roll_joint", "right_ankle_roll_joint"];
    let ankle_roll_lim: [f64; 2] = std::array::from_fn(|i| {
        rig.robot
            .joint_map
            .get(ankle_roll_names[i])
            // The tighter side, so "82% of limit" never flatters an
            // asymmetric joint.
            .map(|&ji| rig.robot.joints[ji].lower.abs().min(rig.robot.joints[ji].upper.abs()))
            .unwrap_or(f64::INFINITY)
    });
    println!(
        "ankle_roll limit: +-{:.1} deg / +-{:.1} deg",
        ankle_roll_lim[0].to_degrees(),
        ankle_roll_lim[1].to_degrees()
    );

    // Per-step accounting, printed at the end. A run-total hides the thing
    // that matters, which is whether the steps are all alike -- a periodic
    // orbit -- or drifting.
    struct StepStat {
        idx: usize,
        slice: usize,
        t0: f64,
        stance: usize,
        dcm_err_max: f64,
        cop_use_max: f64,
        fz_peak: f64,
        degraded: u32,
        td_vz: f64,
        swing_apex: f64,
    }
    let mut step_stats: Vec<StepStat> = Vec::new();

    let mut prev_support = gait.support_at(0.0);
    let mut swing_lift_off: Option<na::Vector3<f64>> = None;
    let p0_body = rig.sim.body_world_position(&rig.robot.root_link).unwrap();
    let yaw0_body = rig.sim.body_world_orientation(&rig.robot.root_link).unwrap().euler_angles().2;
    // Time at which the last step ends, so achievement is measured over the
    // WALK and not diluted by the settle.
    let t_walk_end = gait
        .slices
        .iter()
        .rposition(|s| matches!(s.support, Support::Single { .. }))
        .map(|i| gait.slices[i].t1 + gp.t_ds)
        // No steps at all (a standing squat): the "walk" is the whole run, so
        // the achievement window is the whole run. Falling back to 0 made the
        // window 1 microsecond wide and turned a millimetre of drift into
        // 56 m/s.
        .unwrap_or(total_t);
    let mut walk_end_state: Option<([f64; 3], f64, f64)> = None;
    // Achievement is measured from the end of the stride ramp, not from
    // standing. Including the ramp charges every case a fixed shortfall that
    // never recovers -- it read 91% for every forward speed, and 47% for a
    // turn that was in fact holding 0.195 of a commanded 0.200 rad/s.
    let t_ramp_end = gp.t_start + (stride_ramp as f64) * t_step;
    let mut ramp_end_state: Option<([f64; 3], f64, f64)> = None;
    // Last state before the loop ended, so a case that FELL reports the rate
    // it was actually holding rather than a post-fall pose divided by a
    // duration that never happened.
    let mut last_state: ([f64; 3], f64, f64) = (p0_body, yaw0_body, 0.0);
    // Command tracking is integrated in the BODY frame, tick by tick, rather
    // than taken as a displacement in the start frame. Two reasons, both
    // measured: euler yaw wraps at +-pi, so a turn past 180 deg read as
    // -69% of its command; and a curved path's chord is not its arc, so
    // vx+wz together read 56% on a run whose speed was right.
    let mut travel = [0.0f64; 3]; // body-frame x, y, unwrapped yaw
    let mut prev_pose: Option<([f64; 3], f64)> = None;
    let mut fell = false;
    let mut min_z = f64::INFINITY;
    let mut max_tilt: f64 = 0.0;
    let mut max_dcm_err: f64 = 0.0;
    let mut max_cop_use: f64 = 0.0;
    let mut max_zmp_clamp: f64 = 0.0;
    let mut max_ankle_roll_use: f64 = 0.0;
    let mut n_open_loop = 0u32;
    let mut n_selfcollide = 0u32;
    let mut max_selfcollide_f: f64 = 0.0;
    // ---- disturbance response ------------------------------------------
    // Whole-run maxima say nothing about a push: they are dominated by the
    // walk itself. Everything about the recovery has to be measured from the
    // moment of the push forward, which is what these hold.
    let mut push_at_t: Option<f64> = None;
    let mut push_at_step: usize = 0;
    let mut push_dcm_err_max: f64 = 0.0;
    let mut push_tilt_max: f64 = 0.0;
    let mut push_degraded_at: u32 = 0;
    let mut push_cop_max: f64 = 0.0;
    let mut push_slip_wbc_max: f64 = 0.0;
    let mut push_slip_plant_max: f64 = 0.0;
    let mut push_slip_ticks: u32 = 0;
    // Landing impact and the unloading that follows it. Measured on the
    // straddling pair at 0.20 (fell) against 0.15 and 0.30 (both survived):
    // the faller spiked to 2.77x body weight and then spent 100 ms with the
    // two feet carrying less than half the robot's weight between them --
    // effectively ballistic -- while both survivors never dropped below 0.63x
    // and peaked at 1.22x / 1.41x. Neither friction use nor CoP use nor
    // ankle_roll range separated those three runs; this did.
    let mut push_fz_total_min: f64 = f64::INFINITY;
    let mut push_fz_total_max: f64 = 0.0;
    let mut push_unloaded_ticks: u32 = 0;
    let mut push_ankle_max: f64 = 0.0;
    let mut push_exceeded = false;
    let mut push_last_exceed: Option<f64> = None;
    let mut t_last: f64 = 0.0;

    for tick in 0..n_ticks {
        let t = tick as f64 * dt;
        t_last = t;
        let st = rig.sync();
        let (q, v, v_dvec, data) = (&st.q, &st.v, &st.v_dvec, &st.data);
        // Ground truth first: the phase logic, the contact set and the tasks
        // all solve against this, so it is read before any of them.
        let measured_c = measure_contacts(&rig, data, friction_mu);
        let measured_pre = [measured_c.f_w[0][2], measured_c.f_w[1][2]];
        let com = st.com;
        let com_vel = st.com_vel;

        // ---- contact-driven retiming, before anything reads the phase ---
        if phase_by_contact {
            let i = gait.index_at(t);
            if let Support::Single { swing, .. } = gait.slices[i].support {
                let loaded = measured_pre[swing] > phase_load_frac * total_mass * G;
                land_count = if loaded { land_count + 1 } else { 0 };
                let nominal_end = gait.slices[i].t1;
                let frac = gait.slices[i].frac(t);
                let landed = land_count >= phase_ticks && frac >= phase_min_frac;
                if landed && t < nominal_end - 1e-9 {
                    // Early: the step is over, so say so.
                    retime_total += gait.retime(i, t).abs();
                    n_early += 1;
                    land_count = 0;
                    dcm_plan = DcmPlan::from_footsteps(&gait, &footsteps, z_com, zmp_lat_scale);
                } else if !loaded && t >= nominal_end - 1e-9 {
                    // Late: hold the phase open until the foot arrives.
                    let limit = gait.slices[i].t0
                        + (nominal_end - gait.slices[i].t0) * (1.0 + phase_extend_max);
                    if t + dt <= limit {
                        retime_total += gait.retime(i, t + dt).abs();
                        n_late += 1;
                        dcm_plan =
                            DcmPlan::from_footsteps(&gait, &footsteps, z_com, zmp_lat_scale);
                    }
                }
            }
        }

        // ---- schedule -> contact set -----------------------------------
        let (slice, slice_frac) = gait.at(t);
        let support = if no_lift { Support::Double } else { slice.support };
        let step_idx = gait.steps_taken(t.max(1e-9));
        let slice_idx = gait.index_at(t);
        // Everything below reads the footsteps for THIS slice. With stride 0
        // they are the same pair every time and this is the old behaviour.
        let steps = *footsteps.at_slice(slice_idx);

        // Touchdown and lift-off are the only two events that may edit an
        // anchor. Doing it on the schedule (rather than lazily, the way
        // standing did with get_or_insert) is what makes a foot that has been
        // somewhere else since stop arguing for where it used to be.
        if support != prev_support {
            match (prev_support, support) {
                (Support::Double, Support::Single { swing, .. }) => {
                    anchors.release(swing);
                    let p = misarta::se3::translation(&data.oMi[rig.foot_mi[swing]]);
                    swing_lift_off = Some(p);
                }
                (Support::Single { swing, .. }, Support::Double) => {
                    // The foot has landed somewhere other than nominal, so
                    // every future segment is relative to where it ACTUALLY
                    // is. Rebuilding the DCM plan here is safe rather than
                    // discontinuous: a segment's influence on the present
                    // reference decays by exp(-omega*T) per step -- about a
                    // factor of 7 at these timings -- so a few mm one step
                    // ahead moves the current reference by a fraction of a
                    // millimetre.
                    if adapt && adapt_now.norm() > 1e-9 {
                        if !adapt_live {
                            footsteps.shift_from(slice_idx, swing, adapt_now);
                            dcm_plan =
                                DcmPlan::from_footsteps(&gait, &footsteps, z_com, zmp_lat_scale);
                        }
                        max_adapt = max_adapt.max(adapt_now.norm());
                        n_adapt += 1;
                    }
                    adapt_now = na::Vector2::zeros();
                    adapt_applied = na::Vector2::zeros();
                    // Re-anchor the landed foot where it ACTUALLY is, not
                    // where it was planned to be: the anchor's job is to stop
                    // drift from here, and seeding it with a position the
                    // foot is not at demands a force to close a gap that the
                    // contact cannot close.
                    let mi = rig.foot_mi[swing];
                    let p = misarta::se3::translation(&data.oMi[mi]);
                    let r = misarta::se3::rotation_matrix(&data.oMi[mi]);
                    anchors.touchdown(swing, p, r);
                    swing_lift_off = None;
                }
                _ => {}
            }
            prev_support = support;
        }

        let fz_meas = [measured_pre[0], measured_pre[1]];
        let measured = measured_c;
        let stance_sides: Vec<usize> = if contact_driven {
            let (sides, fresh) = correction.update(support, fz_meas);
            for side in fresh {
                // A foot that has just become load-bearing needs a fresh
                // anchor: inheriting the stale one demands a force to close a
                // gap the contact cannot close.
                //
                // Take x, y and orientation from the measurement, but put z on
                // the GROUND. A contact anchor describes where the foot meets
                // the floor, and admitting a foot that is still 3-5 mm up --
                // which is exactly what a bouncing touchdown looks like --
                // anchors it in mid-air, so the Baumgarte term then works to
                // HOLD IT THERE at kp_c = 1600 while the QP plans a reaction
                // that does not exist. Measured, that is what turned a landed
                // foot into a hovering one for the rest of the run.
                let mi = rig.foot_mi[side];
                let mut p = misarta::se3::translation(&data.oMi[mi]);
                p.z = steps.sole[side].z + prof.sole_below_origin;
                anchors.touchdown(side, p, misarta::se3::rotation_matrix(&data.oMi[mi]));
            }
            let sched: Vec<usize> = (0..2).filter(|&s| support.is_stance(s)).collect();
            if sides != sched {
                n_corrected += 1;
            }
            sides
        } else {
            (0..2).filter(|&s| support.is_stance(s)).collect()
        };
        let stance: Vec<usize> = stance_sides.iter().map(|&s| rig.foot_mi[s]).collect();
        let nc = stance.len();

        // Load ramps across double support. The foot that just landed takes
        // its share on gradually and the foot about to lift gives it up
        // gradually: the CoP box is |m| <= L*fz, so ramping fz is the only
        // way to shrink a foot's authority without a step change in the
        // constraint set.
        let (landing, lifting) = if matches!(support, Support::Double) && !no_lift {
            let i = gait.index_at(t);
            let landed = if i > 0 { gait.slices[i - 1].support.swing() } else { None };
            let next = gait.slices.get(i + 1).and_then(|s| s.support.swing());
            (landed, next)
        } else {
            (None, None)
        };
        let ramp_frac = env_f64("LOAD_RAMP_FRAC", 0.6).clamp(0.05, 1.0);
        let f_share = |side: usize| -> f64 {
            if let Some(l) = landing {
                if l == side {
                    return (slice_frac / ramp_frac).clamp(0.0, 1.0);
                }
            }
            if let Some(l) = lifting {
                if l == side {
                    return ((1.0 - slice_frac) / ramp_frac).clamp(0.0, 1.0);
                }
            }
            1.0
        };
        let foot_mi = rig.foot_mi;
        let load_share = move |mi: usize| -> f64 {
            let side = usize::from(mi != foot_mi[0]);
            f_share(side).max(0.02)
        };

        let (j_contact, dj_v) = contact_jacobians(&rig.model, q, v, data, v_dvec, &stance, nv);
        let dyn_ctx = Dynamics::new(Formulation::Explicit, &st.mass, &st.h, &j_contact, na_count);
        let forces = dyn_ctx.forces();

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
            rig.foot_mi[0],
            &mut anchors,
            &load_share,
            &contact_cfg,
        );

        // ---- P1: DCM-tracking CoM acceleration -------------------------
        let xi = dcm_of(&com, &com_vel, omega);
        let r = dcm_plan.reference(t);
        let dcm_err = (xi - r.xi).norm();
        max_dcm_err = max_dcm_err.max(dcm_err);
        // Post-push accumulation. The push fires late in the tick, after this
        // point, so the firing tick's error is still the undisturbed one and
        // the window starts one tick later -- which is what we want.
        if push_at_t.is_some() {
            push_dcm_err_max = push_dcm_err_max.max(dcm_err);
            // Recovery is decided BACKWARD from the end of the run, from the
            // LAST time the error was out of tolerance -- not forward from the
            // first time it dipped back in.
            //
            // Both cheaper definitions are wrong, and each was measured wrong
            // here before this comment existed. Counting the first N ticks
            // under the line credits recovery 5 ms after the shove, because
            // the impulse has not reached the CoM yet. Counting the first
            // N-tick dwell after the error exceeds the line credits recovery
            // at 5.150 s to a robot that fell at 5.915 s, because the error is
            // not monotonic on the way down and crosses the band on the way
            // past. Only "went under and STAYED under" is a recovery.
            if dcm_err >= push_recover_m {
                push_exceeded = true;
                push_last_exceed = Some(t);
            }
        }
        // ---- capture-point footstep adaptation --------------------------
        // Steer the NEXT footstep on the DCM's predicted position at
        // touchdown. Recomputed every tick; the prediction horizon shrinks to
        // zero as the step runs out, so the target converges rather than
        // chasing early-step noise.
        let p_raw = commanded_zmp(&xi, &r, omega, k_dcm);
        // Clamp against the support that is REALLY there, which in NO_LIFT is
        // both feet even where the plan says one.
        let sbox = SupportBox::from_stance_yawed(
            &steps,
            support,
            (sole_half_l * cop_frac, sole_half_w * cop_frac),
            zmp_margin,
        );
        let (p_cmd, clamped) = if clamp_zmp { sbox.clamp(&p_raw) } else { (p_raw, 0.0) };
        max_zmp_clamp = max_zmp_clamp.max(clamped);

        // ---- capture-point footstep adaptation --------------------------
        //
        // Trigger on ZMP SATURATION, not on a DCM prediction.
        //
        // The obvious formulation -- predict the DCM at touchdown open-loop
        // and shift the foot by the error -- is wrong here and was measured
        // to be: it ignores that the ZMP feedback is already pulling the DCM
        // back, so the two corrections fight. At the head of a single support
        // the prediction carries exp(omega*0.35) = 7, turning a 5 mm tracking
        // error into a 35 mm step adjustment, and the gait went from 200
        // steps to 190 at the sustainable speed and 26 to 14 above it.
        //
        // The foot should move only when the ZMP CANNOT do the job. The clamp
        // deficit `p_raw - p_cmd` is exactly that: the pressure the controller
        // asked for and the support polygon refused. Move the next footstep
        // by it, and the next polygon contains what this one could not.
        if adapt && matches!(support, Support::Single { .. }) && clamped > 1e-6 {
            let d = p_raw - p_cmd;
            let want = na::Vector2::new(
                d.x.clamp(-adapt_max_x, adapt_max_x),
                d.y.clamp(-adapt_max_y, adapt_max_y),
            );
            // Keep the largest demand seen this step rather than the latest:
            // the deficit is transient and the foot has to be placed once.
            if want.norm() > adapt_now.norm() {
                adapt_now = want;
            }
        }
        // Commit the change to the plan NOW, so `xi_ref` below describes the
        // footstep the foot is actually going to.
        if adapt_live {
            if let Support::Single { swing, .. } = support {
                let d = adapt_now - adapt_applied;
                if d.norm() > 1e-9 {
                    footsteps.shift_from(slice_idx, swing, d);
                    dcm_plan = DcmPlan::from_footsteps(&gait, &footsteps, z_com, zmp_lat_scale);
                    adapt_applied = adapt_now;
                    n_replan += 1;
                }
            }
        }
        let a_xy = com_accel_xy(&com, &p_cmd, omega);
        // z stays a PD on the nominal height: the LIPM says nothing about it,
        // and a constant-height assumption is exactly what makes omega a
        // constant in the first place.
        // Height reference. `omega` stays at the nominal height: the LIPM is
        // derived for a constant one, and re-deriving it per tick is the
        // variable-height DCM formulation, which this is not. The squat is
        // therefore a disturbance the horizontal loop has to survive, which is
        // the point of having it in the benchmark.
        let (z_ref, zd_ref, zdd_ref) = if squat_amp > 0.0 {
            let w = 2.0 * PI / squat_period;
            let ph = w * t;
            (
                z_com - squat_amp * (1.0 - ph.cos()) * 0.5,
                -squat_amp * 0.5 * w * ph.sin(),
                -squat_amp * 0.5 * w * w * ph.cos(),
            )
        } else {
            (z_com, 0.0, 0.0)
        };
        let a_z = zdd_ref + kp_com * (z_ref - com.z) + kd_com * (zd_ref - com_vel.z);
        let a_com = na::Vector3::new(a_xy.x, a_xy.y, a_z);
        let p1 = bt::com(dyn_ctx.qddot(), &st.j_com, &st.djv_com, &a_com);

        // ---- P2: trunk upright -----------------------------------------
        let j_trunk =
            misarta::jacobian::compute_joint_jacobian_from_data(&rig.model, q, data, rig.trunk_mi);
        let dj_trunk = misarta::jacobian::compute_joint_jacobian_time_derivative(
            &rig.model, q, v, rig.trunk_mi,
        );
        let djv_trunk = &dj_trunk * v_dvec;
        let (roll, pitch) = if rig.trunk_from_base {
            let (r, p, _) = st.body_quat.euler_angles();
            (r, p)
        } else {
            let rot = misarta::se3::rotation_matrix(&data.oMi[rig.trunk_mi]);
            (rot[(2, 1)].atan2(rot[(2, 2)]), (-rot[(2, 0)]).asin())
        };
        let _ = (roll, pitch);
        // Yaw error against the pose the plan was built in, not against the
        // last tick: the reference is a fixed world direction.
        // Trunk yaw follows the PLAN, not the initial heading.
        //
        // Regulating to `yaw0` with a zero rate reference is right for
        // straight walking -- it is what stopped the legs twisting to their
        // hip_yaw stops -- and it makes turning impossible, because the task
        // then fights the commanded turn directly. Measured: every turn
        // command in the benchmark fell, down to 0.05 rad/s. The planned
        // heading is the mean of the two footstep yaws, which already carries
        // the stride ramp.
        // WHEN the body turns matters as much as how much.
        //
        // Yawing during SINGLE support has to be paid for out of the stance
        // sole's torsional friction -- mu_torsion 0.05 against 65 N is
        // 3.25 N*m, on a 38 mm-wide foot. Yawing during DOUBLE support is paid
        // for by a couple between two feet 99.4 mm apart, which is a different
        // order of authority. So hold the heading through single support and
        // turn while both feet are down.
        //
        // `at_slice(i)` already has the swing foot at its LANDING yaw, so
        // reading it during single support asks the trunk to rotate before the
        // foot that will carry the rotation is even down.
        let turn_in_ds = flag("TURN_IN_DS", true);
        let yaw_plan = {
            let mean = |i: usize| {
                let y = footsteps.at_slice(i).yaw;
                0.5 * (y[0] + y[1])
            };
            let prev = mean(slice_idx.saturating_sub(1));
            if !turn_in_ds {
                mean(slice_idx)
            } else if matches!(support, Support::Single { .. }) {
                prev
            } else {
                // Rotate across the double support.
                prev + (mean(slice_idx) - prev) * slice_frac
            }
        };
        // Rate reference follows the same shape as the heading: zero while a
        // single foot is carrying the robot, the whole turn compressed into DS.
        let wz_ref = if turn_in_ds {
            if matches!(support, Support::Single { .. }) { 0.0 } else { wz * (t_step / gp.t_ds.max(1e-6)) }
        } else {
            wz
        };
        // Now an absolute target heading -- trunk_ori_ref forms the error.
        let yaw_ref = trunk_yaw.then_some(yaw_plan);
        // One world-frame rotation-vector error for all three rows. The
        // roll/pitch rows are frame-correct at any heading this way; the old
        // Euler form silently rotated its own correction away.
        let ori = bt::trunk_ori_ref(
            &st.body_quat,
            yaw_ref,
            &st.v_ang_w,
            bt::TrunkGains {
                kp: kp_trunk,
                kd: kd_trunk,
                deadband: trunk_dead,
                sign: trunk_sign,
                kp_yaw,
                kd_yaw,
                wz_ref,
            },
        );
        let rp_ref = [ori[0], ori[1]];
        let p2 = bt::trunk_rpy(
            dyn_ctx.qddot(),
            &j_trunk,
            &djv_trunk,
            &rp_ref,
            yaw_ref.map(|_| ori[2]),
            nv,
        );

        // ---- P4: posture ------------------------------------------------
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

        // ---- lowest: gravity comp + the load split the ZMP implies -------
        let tau_gravity = rig.gravity_torque(q);
        // Split the nominal load so that the net CoP lands where the DCM
        // controller asked for it. Projecting p_cmd onto the line between the
        // sole centres generalises the y-interpolation the standing version
        // used, and it is the same argument: an equal-share target makes the
        // regulariser walk the CoP outward instead of transferring load.
        let shares: Vec<f64> = match nc {
            2 => {
                let s0 = steps.xy(0);
                let s1 = steps.xy(1);
                let d = s0 - s1;
                let a = if d.norm_squared() > 1e-12 {
                    ((p_cmd - s1).dot(&d) / d.norm_squared()).clamp(0.0, 1.0)
                } else {
                    0.5
                };
                vec![a * load_share(stance[0]), (1.0 - a) * load_share(stance[1])]
            }
            _ => vec![1.0; nc],
        };
        let forces_nominal = bt::force_nominal(forces.size(), &shares, total_mass * G);
        let p_reg =
            bt::regulariser(dyn_ctx.tau(), &tau_gravity, &dyn_ctx.forces(), &forces_nominal);

        // ---- P3: the foot that is NOT in contact ------------------------
        //
        // EVERY foot outside the contact set needs a Cartesian task. A foot
        // that is in neither is unconstrained except by the weak posture
        // level, and the QP will happily use the free leg as a reaction mass
        // for the CoM task: measured, a foot that had just landed was flung
        // to 80 mm and stayed there for a second while the robot toppled on
        // the other leg. "The schedule says double support" is not a task.
        let free_side: Option<usize> = (0..2).find(|s| !stance_sides.contains(s));
        let mut swing_tgt = na::Vector3::zeros();
        let p_swing = match (free_side, swing_lift_off) {
            (None, _) => None,
            // The scheduled swing, following its planned arc.
            (Some(side), Some(lift_off)) if support.swing() == Some(side) => {
                let mi = rig.foot_mi[side];
                let jf =
                    misarta::jacobian::compute_joint_jacobian_from_data(&rig.model, q, data, mi);
                let djf =
                    misarta::jacobian::compute_joint_jacobian_time_derivative(&rig.model, q, v, mi);
                let djv = &djf * v_dvec;
                let pos = misarta::se3::translation(&data.oMi[mi]);
                let vel3 = &jf.rows(3, 3).into_owned() * v_dvec;
                let vel = na::Vector3::new(vel3[0], vel3[1], vel3[2]);
                // Land on the PLANNED footstep. Its z stays the lift-off z
                // rather than the plan's: the ground is flat, so they agree to
                // within a millimetre, and using the measured one keeps a foot
                // that started slightly high from being driven into the floor.
                // With stride 0 this is exactly "land where you took off".
                // Read the plan, which already carries the adaptation when
                // ADAPT_LIVE is on. Adding `adapt_now` on top of a plan that
                // has been shifted would apply it twice.
                let plan_xy = footsteps.at_slice(slice_idx).sole[side];
                let extra = if adapt_live { na::Vector2::zeros() } else { adapt_now };
                let touch_down = na::Vector3::new(
                    plan_xy.x + extra.x,
                    plan_xy.y + extra.y,
                    lift_off.z,
                );
                let tgt = swing_position(lift_off, touch_down, lift_h, slice_frac);
                let tgt_v =
                    swing_velocity(lift_off, touch_down, lift_h, slice_frac, slice.duration());
                swing_tgt = tgt;
                let om3 = &jf.rows(0, 3).into_owned() * v_dvec;
                Some(bt::swing_with_pose(
                    dyn_ctx.qddot(),
                    &jf,
                    &djv,
                    &pos,
                    &vel,
                    &tgt,
                    &tgt_v,
                    kp_sw_xy,
                    kp_sw_z,
                    kd_sw,
                    match swing_rot_mode {
                        0 => bt::SwingAxes::Xyz,
                        2 => bt::SwingAxes::Pose,
                        _ => bt::SwingAxes::XyzYaw,
                    },
                    Some((
                        misarta::se3::rotation_matrix(&data.oMi[mi]),
                        foot_rot_at(side, footsteps.at_slice(slice_idx).yaw[side]),
                        na::Vector3::new(om3[0], om3[1], om3[2]),
                        kp_sw_rot,
                        kd_sw_rot,
                    )),
                ))
            }
            // Out of contact for any other reason -- it has not re-established
            // yet, or it bounced. Put it back on its planned footstep. The
            // target is the foot LINK origin, which sits `sole_below_origin`
            // above the sole.
            (Some(side), _) => {
                let mi = rig.foot_mi[side];
                let jf =
                    misarta::jacobian::compute_joint_jacobian_from_data(&rig.model, q, data, mi);
                let djf =
                    misarta::jacobian::compute_joint_jacobian_time_derivative(&rig.model, q, v, mi);
                let djv = &djf * v_dvec;
                let pos = misarta::se3::translation(&data.oMi[mi]);
                let vel3 = &jf.rows(3, 3).into_owned() * v_dvec;
                let vel = na::Vector3::new(vel3[0], vel3[1], vel3[2]);
                let tgt = steps.sole[side] + na::Vector3::new(0.0, 0.0, prof.sole_below_origin);
                swing_tgt = tgt;
                let om3 = &jf.rows(0, 3).into_owned() * v_dvec;
                Some(bt::swing_with_pose(
                    dyn_ctx.qddot(),
                    &jf,
                    &djv,
                    &pos,
                    &vel,
                    &tgt,
                    &na::Vector3::zeros(),
                    kp_sw_xy,
                    kp_sw_z,
                    kd_sw,
                    match swing_rot_mode {
                        0 => bt::SwingAxes::Xyz,
                        2 => bt::SwingAxes::Pose,
                        _ => bt::SwingAxes::XyzYaw,
                    },
                    Some((
                        misarta::se3::rotation_matrix(&data.oMi[mi]),
                        foot_rot_at(side, footsteps.at_slice(slice_idx).yaw[side]),
                        na::Vector3::new(om3[0], om3[1], om3[2]),
                        kp_sw_rot,
                        kd_sw_rot,
                    )),
                ))
            }
        };

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
        let degraded = !matches!(sol.status, misa_wbc::SolveStatus::Optimal);
        tally.observe(&sol.status, t, tick, nc, &level_names);
        let extracted = dyn_ctx.extract(&sol.x);

        let mut cop_qp = [[0.0_f64; 3]; 2];
        let mut f_qp_w = [[0.0_f64; 3]; 2];
        let mut cop_use = 0.0_f64;
        for (slot, mi) in stance.iter().copied().enumerate() {
            let side = rig.side_of(mi);
            let w = &p0.sole_sel[slot] * &extracted.forces;
            if let Some((cx, cy, fz)) = cop_from_sole_wrench(&w) {
                if fz > 1e-6 {
                    cop_qp[side] = [cx, cy, fz];
                    // Only count a foot that is actually carrying something.
                    // A foot at 1 N with its CoP in the box corner is a
                    // 0.07 N*m wrench, and reporting that as a saturated
                    // constraint hides the one that matters.
                    if fz > 0.1 * total_mass * G {
                        cop_use = cop_use
                            .max((cx.abs() / (sole_half_l * cop_frac))
                                .max(cy.abs() / (sole_half_w * cop_frac)));
                    }
                }
            }
            for k in 0..3 {
                f_qp_w[side][k] = extracted.forces[6 * slot + 3 + k];
            }
        }
        max_cop_use = max_cop_use.max(cop_use);
        if push_at_t.is_some() {
            push_cop_max = push_cop_max.max(cop_use);
            // Friction utilisation, against BOTH denominators, because they
            // answer different questions and swapping them silently is how a
            // friction sweep comes to measure its own assumption.
            //
            //   ...wbc   -- tangential / (friction_mu * fz): what the WBC
            //               thinks it is allowed. Sweeping `friction_mu` moves
            //               this by construction, so it cannot show whether
            //               the foot slipped.
            //   ...plant -- tangential / (mu_ground * fz): what the ground
            //               actually offers. Above 1.0 the foot IS sliding,
            //               regardless of what the controller assumed.
            let fz_total = measured.f_w[0][2] + measured.f_w[1][2];
            push_fz_total_min = push_fz_total_min.min(fz_total);
            push_fz_total_max = push_fz_total_max.max(fz_total);
            if fz_total < 0.5 * total_mass * G {
                push_unloaded_ticks += 1;
            }
            for side in 0..2 {
                let fz = measured.f_w[side][2];
                if fz <= 1.0 {
                    continue; // an unloaded foot has no meaningful ratio
                }
                let tan = (measured.f_w[side][0].powi(2)
                    + measured.f_w[side][1].powi(2))
                    .sqrt();
                push_slip_wbc_max = push_slip_wbc_max.max(tan / (friction_mu * fz));
                push_slip_plant_max = push_slip_plant_max.max(tan / (mu_ground * fz));
                // AT the cone, not past it: MuJoCo's contact solver never
                // returns a force outside the cone, so `tan > mu*fz` is only
                // ever true by floating-point luck and counted 0 ticks on a
                // run that slid for 150 ms. Riding the boundary IS sliding.
                if tan >= 0.98 * mu_ground * fz {
                    push_slip_ticks += 1;
                }
            }
        }

        // Self-collision, watched EVERY tick rather than only at spawn.
        //
        // The spawn assert catches a robot that is braced against itself
        // before it moves; it says nothing about one that folds into itself
        // mid-step. That has already happened on this machine once -- a
        // stance-width sweep found 130 mm topples the robot because the FEET
        // touch -- and a contact the QP has no model for is indistinguishable
        // in the logs from a control failure.
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
                if n_selfcollide <= 8 {
                    let d: Vec<String> = hits
                        .iter()
                        .map(|(a, b, f)| format!("{a} <-> {b} ({f:.1} N)"))
                        .collect();
                    println!("  [self-collision] t={t:6.3}  {}", d.join(", "));
                }
                max_selfcollide_f =
                    max_selfcollide_f.max(hits.iter().map(|(_, _, f)| *f).fold(0.0, f64::max));
            }
        }

        let mut robot_taus = vec![0.0_f64; rig.robot.joints.len()];
        for (ji, vi) in rig.actuated() {
            robot_taus[ji] = extracted.tau[vi - 6];
        }
        {
            let rig_ref = &rig;
            let tg = &tau_gravity;
            let fallback =
                move |out: &mut [f64]| gravity_plus_posture(rig_ref, tg, v, hold_kp, hold_kd, out);
            policy.apply(&sol.status, &mut robot_taus, t, dt, nc, &fallback);
        }
        if policy.consecutive_degraded() >= 10 {
            n_open_loop += 1;
        }

        // ankle_roll travel, against the joint's own limit.
        let ankle_roll: [f64; 2] = std::array::from_fn(|i| {
            rig.robot
                .joint_map
                .get(ankle_roll_names[i])
                .map(|&ji| rig.robot.joint_positions[ji])
                .unwrap_or(0.0)
        });
        for i in 0..2 {
            if ankle_roll_lim[i].is_finite() && ankle_roll_lim[i] > 1e-6 {
                let use_i = ankle_roll[i].abs() / ankle_roll_lim[i];
                max_ankle_roll_use = max_ankle_roll_use.max(use_i);
                if push_at_t.is_some() {
                    push_ankle_max = push_ankle_max.max(use_i);
                }
            }
        }

        // Per-step accounting, keyed on the single-support phases.
        if matches!(support, Support::Single { .. }) {
            let stance_side = match support {
                Support::Single { stance, .. } => stance,
                Support::Double => unreachable!(),
            };
            if step_stats.last().map(|s| s.slice) != Some(slice_idx) {
                step_stats.push(StepStat {
                    idx: step_idx,
                    slice: slice_idx,
                    t0: t,
                    stance: stance_side,
                    dcm_err_max: 0.0,
                    cop_use_max: 0.0,
                    fz_peak: 0.0,
                    degraded: 0,
                    td_vz: 0.0,
                    swing_apex: 0.0,
                });
            }
            let s = step_stats.last_mut().unwrap();
            s.dcm_err_max = s.dcm_err_max.max(dcm_err);
            s.cop_use_max = s.cop_use_max.max(cop_use);
            s.fz_peak = s.fz_peak.max(measured.f_w[0][2] + measured.f_w[1][2]);
            s.degraded += u32::from(degraded);
            if let Some(sw) = support.swing() {
                let z = rig.sim.body_world_position(prof.foot_links[sw]).unwrap()[2];
                s.swing_apex = s.swing_apex.max(z - prof.sole_below_origin);
                s.td_vz = rig
                    .sim
                    .body_world_linear_velocity(prof.foot_links[sw])
                    .map(|v| v[2])
                    .unwrap_or(0.0);
            }
        }

        let trot = misarta::se3::rotation_matrix(&data.oMi[rig.trunk_mi]);
        let trunk_tilt =
            trot[(2, 1)].atan2(trot[(2, 2)]).abs().max((-trot[(2, 0)]).asin().abs());

        // ---- disturbance: fire once, at the requested phase --------------
        // After the plant write and before the plant step, so the push lands
        // on the same tick boundary as the torques it has to fight, and the
        // controller sees the consequence on the NEXT tick's measurement --
        // which is the causal order a real shove has.
        if push_at_t.is_none() && (push_imp.abs() > 0.0 || push_yaw_imp.abs() > 0.0) {
            let phase_ok = match push_support.as_str() {
                "ds" => matches!(support, Support::Double),
                "ss" => matches!(support, Support::Single { .. }),
                _ => true,
            };
            if step_idx == push_step && phase_ok && slice_frac >= push_at {
                let th = push_deg.to_radians();
                let f = push_imp / push_dt;
                let (fx, fy) = (f * th.cos(), f * th.sin());
                let tz = push_yaw_imp / push_dt;
                rig.sim
                    .apply_external_force(&push_link, [fx, fy, 0.0], [0.0, 0.0, tz], push_dt);
                let dv = push_imp / total_mass;
                println!(
                    "  PUSH at t={t:.3} step={step_idx} {} frac={slice_frac:.2}: \
                     {push_imp:.3} N*s at {push_deg:.0} deg \
                     (F=[{fx:.1}, {fy:.1}] N for {push_dt:.3} s) -> dv={dv:.3} m/s, \
                     capture-point shift {:.1} mm",
                    match support {
                        Support::Double => "DS".to_string(),
                        Support::Single { stance, .. } =>
                            format!("SS/{}", if stance == 0 { "L" } else { "R" }),
                    },
                    dv / omega * 1e3,
                );
                // If the pulse outlives the slice it was fired into, the run
                // is not measuring what the label says. Firing 0.10 s into a
                // 0.20 s double-support at frac 0.5 puts the tail of the pulse
                // exactly on liftoff, and the resulting "double support"
                // number came out BELOW the single-support one -- the phase
                // was being measured, not the impulse. Say so rather than
                // letting the row be compared with the others.
                let slice_left = slice.t1 - t;
                if push_dt > slice_left {
                    println!(
                        "  PUSH WARNING: the {push_dt:.3} s pulse outlasts this \
                         slice by {:.3} s, so it spills into the next support \
                         phase. Lower PUSH_AT or PUSH_DT to keep the \
                         disturbance inside one phase.",
                        push_dt - slice_left
                    );
                }
                push_at_t = Some(t);
                push_at_step = step_idx;
                push_degraded_at = tally.n;
            }
        }

        write_to_plant(&mut rig, ctrl_mode, &robot_taus, &extracted.qddot, v, dt);
        rig.step(mj_substeps);

        if log.is_enabled() {
            let foot_z = rig.sim.body_world_position(prof.foot_links[0]).unwrap()[2];
            let foot_vz = rig
                .sim
                .body_world_linear_velocity(prof.foot_links[0])
                .map(|v| v[2])
                .unwrap_or(0.0);
            let swing_z = rig.sim.body_world_position(prof.foot_links[1]).unwrap()[2];
            let support_code = match support {
                Support::Double => 0.0,
                Support::Single { stance, .. } => 1.0 + stance as f64,
            };
            let extra = [
                xi.x, xi.y, r.xi.x, r.xi.y, r.p.x, r.p.y,
                p_cmd.x, p_cmd.y, clamped, dcm_err, support_code, step_idx as f64,
                *shares.first().unwrap_or(&0.0), *shares.get(1).unwrap_or(&0.0),
                swing_tgt.x, swing_tgt.y, swing_tgt.z,
                ankle_roll[0], ankle_roll[1],
            ];
            let row = Row {
                t,
                com,
                com_ref_z: z_ref,
                // The renderer's `com_ref_y` panel is the lateral reference;
                // for a walk that is the DCM target, not a fixed point.
                com_ref_y: r.xi.y,
                tilt: roll.abs().max(pitch.abs()),
                trunk_tilt,
                n_stance: nc,
                foot_z,
                swing_z,
                foot_vz,
                acc_dbg: p0.acc_dbg,
                a_com,
                rp_ref,
                degraded,
                cop_box: (sole_half_l * cop_frac, sole_half_w * cop_frac),
                cop_qp,
                f_qp_w,
                measured: &measured,
                taus: &robot_taus,
                extra: &extra,
            };
            log.write(&rig, &row);
        }

        if !fell {
            let p = rig.sim.body_world_position(&rig.robot.root_link).unwrap();
            let y = rig.sim.body_world_orientation(&rig.robot.root_link).unwrap().euler_angles().2;
            if let Some((pp, py)) = prev_pose {
                if t >= t_ramp_end {
                    let (dx, dy) = (p[0] - pp[0], p[1] - pp[1]);
                    let (c, sn) = (py.cos(), py.sin());
                    travel[0] += dx * c + dy * sn;
                    travel[1] += -dx * sn + dy * c;
                    // Shortest-arc difference, so the accumulator never wraps.
                    let mut dy_aw = y - py;
                    while dy_aw > std::f64::consts::PI { dy_aw -= std::f64::consts::TAU; }
                    while dy_aw < -std::f64::consts::PI { dy_aw += std::f64::consts::TAU; }
                    travel[2] += dy_aw;
                }
            }
            prev_pose = Some((p, y));
            last_state = (p, y, t);
        }
        if ramp_end_state.is_none() && t >= t_ramp_end {
            let p = rig.sim.body_world_position(&rig.robot.root_link).unwrap();
            let y = rig.sim.body_world_orientation(&rig.robot.root_link).unwrap().euler_angles().2;
            ramp_end_state = Some((p, y, t));
        }
        if walk_end_state.is_none() && t >= t_walk_end {
            let p = rig.sim.body_world_position(&rig.robot.root_link).unwrap();
            let y = rig.sim.body_world_orientation(&rig.robot.root_link).unwrap().euler_angles().2;
            walk_end_state = Some((p, y, t));
        }
        let cur_z = rig.sim.body_world_position(&rig.robot.root_link).unwrap()[2];
        min_z = min_z.min(cur_z);
        let tilt = roll.abs().max(pitch.abs());
        max_tilt = max_tilt.max(tilt);
        if push_at_t.is_some() {
            push_tilt_max = push_tilt_max.max(tilt);
        }
        if tick % 40 == 0 {
            let sup = match support {
                Support::Double => "DS  ".to_string(),
                Support::Single { stance, .. } => {
                    format!("SS{}", if stance == 0 { "L" } else { "R" })
                }
            };
            println!(
                "  t={t:6.3} {sup} step={step_idx:3}  com=({:+.4},{:+.4},{:+.4})  \
                 xi=({:+.4},{:+.4}) err={:.4}  zmp=({:+.4},{:+.4})  cop_use={cop_use:.2}  {:?}",
                com.x, com.y, com.z, xi.x, xi.y, dcm_err, p_cmd.x, p_cmd.y, sol.status
            );
        }
        if cur_z < 0.30 || tilt > 0.52 {
            println!("  FELL at t={t:.3} (z={cur_z:.3}, tilt={tilt:.3})");
            fell = true;
            break;
        }
    }

    println!("\n=== Per-step ===");
    println!("  step  t0     stance  dcm_err  cop_use  fz_peak  degraded  apex_mm  td_vz");
    for s in &step_stats {
        println!(
            "  {:4}  {:5.2}  {:>6}  {:7.4}  {:7.2}  {:7.1}  {:8}  {:7.1}  {:+.3}",
            s.idx,
            s.t0,
            if s.stance == 0 { "L" } else { "R" },
            s.dcm_err_max,
            s.cop_use_max,
            s.fz_peak,
            s.degraded,
            s.swing_apex * 1e3,
            s.td_vz
        );
    }

    println!("\n=== Result (walk) ===");
    println!("  steps taken: {}", step_stats.len());
    {
        // Achieved against COMMANDED, on every axis that was commanded, and
        // measured over the walk rather than to the end of the log. A gait
        // that goes half as far as it was told looks perfectly stable while
        // doing it -- and a travel metric that only reads x says "100%" for a
        // sideways command that went nowhere, which is worse than no metric.
        let (pe, ye, te) = walk_end_state.unwrap_or(last_state);
        let (_, _, t0) = ramp_end_state.unwrap_or((p0_body, yaw0_body, gp.t_start));
        let _ = (pe, ye);
        let dur = (te - t0).max(1e-6);
        let ach = [
            travel[0] / dur,
            travel[1] / dur,
            travel[2] / dur,
        ];
        let cmd = [vx, vy, wz];
        let names = ["vx", "vy", "wz"];
        let units = ["m/s", "m/s", "rad/s"];
        let mut parts = Vec::new();
        for k in 0..3 {
            if cmd[k].abs() < 1e-9 && ach[k].abs() < 1e-3 {
                continue;
            }
            let pct = if cmd[k].abs() > 1e-9 { 100.0 * ach[k] / cmd[k] } else { f64::NAN };
            // An uncommanded axis has no percentage -- it is cross-coupling,
            // and printing "(0%)" for it makes the pass criterion read it as a
            // failure.
            if pct.is_nan() {
                parts.push(format!("{}: {:+.4} {} (drift)", names[k], ach[k], units[k]));
            } else {
                parts.push(format!(
                    "{}: {:+.4} / {:+.4} {} ({:.0}%)",
                    names[k], ach[k], cmd[k], units[k], pct
                ));
            }
        }
        if parts.is_empty() {
            parts.push("no command".into());
        }
        println!(
            "  achieved after the ramp ({:.1} s): {}",
            dur,
            parts.join("   ")
        );
    }
    println!("  min trunk z = {min_z:.3}   max tilt = {max_tilt:.3} rad");
    println!("  max |xi - xi_ref| = {:.1} mm", max_dcm_err * 1e3);
    println!("  max CoP box use = {max_cop_use:.2}  (feet carrying >10% of weight)");
    println!("  max ZMP clamp = {:.1} mm", max_zmp_clamp * 1e3);
    println!("  max ankle_roll use = {:.0}% of limit", max_ankle_roll_use * 100.0);
    println!("  degraded solves: {}", tally.n);
    tally.report();
    println!("  open-loop ticks: {n_open_loop}");
    println!("  contact-set corrections: {n_corrected} ticks where the feet disagreed with the schedule");
    println!(
        "  phase retimed by contact: {n_early} early / {n_late} late ticks, {:.0} ms total",
        retime_total * 1e3
    );
    println!(
        "  footstep adaptations: {n_adapt} steps moved, largest {:.1} mm, {n_replan} live replans",
        max_adapt * 1e3
    );
    println!(
        "  self-collision ticks: {n_selfcollide}  (peak {max_selfcollide_f:.1} N)          -- any nonzero value means the QP was solving against a contact it has no model for"
    );
    if let Some(tp) = push_at_t {
        println!("\n=== Disturbance ===");
        println!(
            "  push: {push_imp:.3} N*s at {push_deg:.0} deg, {push_dt:.3} s, \
             t={tp:.3} step={push_at_step} ({push_support}, frac>={push_at:.2})"
        );
        // The ankle-strategy budget, from the CoP box this run is actually
        // using. Printed next to the impulse so a row that survived on
        // stepping alone is distinguishable from one the feet could have
        // absorbed standing still -- the two are different results.
        // How far the CoP can travel along the push direction before it leaves
        // the sole. For a rectangular box that is where the ray exits, i.e.
        // min over the axes of half_extent / |component| -- NOT the half
        // extent of whichever axis is dominant, which is only right for the
        // two axis-aligned cases and gets a 45 deg push wrong by 1.8x.
        let (ux, uy) = (push_deg.to_radians().cos(), push_deg.to_radians().sin());
        let d_cop = [
            if ux.abs() > 1e-9 { sole_half_l / ux.abs() } else { f64::INFINITY },
            if uy.abs() > 1e-9 { sole_half_w / uy.abs() } else { f64::INFINITY },
        ]
        .into_iter()
        .fold(f64::INFINITY, f64::min);
        let ankle_budget = total_mass * omega * d_cop;
        println!(
            "  ankle-strategy budget in that direction: {ankle_budget:.2} N*s \
             ({:.1}x exceeded)",
            push_imp / ankle_budget.max(1e-9)
        );
        println!("  peak |xi - xi_ref| after the push = {:.1} mm", push_dcm_err_max * 1e3);
        println!("  peak tilt after the push = {push_tilt_max:.3} rad");
        println!("  degraded solves after the push: {}", tally.n - push_degraded_at);
        // What was actually saturating when it let go. On this machine the DCM
        // error stays small right up to the limit and then the robot falls, so
        // the error is not the early warning -- these two are.
        println!("  peak CoP box use after the push = {push_cop_max:.2}");
        println!(
            "  peak friction use after the push = {push_slip_plant_max:.2} of the \
             PLANT cone (mu={mu_ground:.2}), {push_slip_wbc_max:.2} of what the WBC \
             assumed (mu={friction_mu:.2})"
        );
        println!(
            "  sliding ticks after the push: {push_slip_ticks} ({:.0} ms)  -- a loaded \
             foot riding the plant's friction cone",
            push_slip_ticks as f64 * dt * 1e3
        );
        println!(
            "  total vertical ground force after the push: min {:.1} N ({:.2}x weight), \
             peak {push_fz_total_max:.1} N ({:.2}x weight)",
            push_fz_total_min,
            push_fz_total_min / (total_mass * G),
            push_fz_total_max / (total_mass * G),
        );
        println!(
            "  unloaded ticks after the push: {push_unloaded_ticks} ({:.0} ms below half \
             body weight)  -- the machine is ballistic and the QP is planning \
             against contacts that are not carrying it",
            push_unloaded_ticks as f64 * dt * 1e3
        );
        println!(
            "  peak ankle_roll use after the push = {:.0}% of limit",
            push_ankle_max * 100.0
        );
        let settled_for = push_last_exceed.map(|te| t_last - te).unwrap_or(f64::INFINITY);
        if fell {
            println!("  NOT RECOVERED: fell");
        } else if !push_exceeded {
            println!(
                "  ABSORBED: the DCM error never left the {:.0} mm band at all",
                push_recover_m * 1e3
            );
        } else if settled_for >= push_recover_s {
            let te = push_last_exceed.unwrap();
            println!(
                "  RECOVERED at t={te:.3} ({:.3} s, {} steps after the push) -- \
                 last out-of-band tick, then under {:.0} mm for {settled_for:.2} s \
                 to the end of the run",
                te - tp,
                gait.steps_taken(te.max(1e-9)).saturating_sub(push_at_step),
                push_recover_m * 1e3,
            );
        } else {
            println!(
                "  NOT RECOVERED: survived, but the DCM error was still outside \
                 the {:.0} mm band {settled_for:.2} s before the run ended \
                 (needs {push_recover_s:.2} s) -- lengthen T to decide this row",
                push_recover_m * 1e3
            );
        }
    }
    println!("  verdict: {}", if fell { "FELL" } else { "SURVIVED" });
}

#[cfg(not(feature = "mujoco"))]
fn main() {
    eprintln!("This example requires the `mujoco` feature. Run with:");
    eprintln!("  cargo run --features mujoco --example kyo46rs_walk");
}
