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
    // Swing orientation gains. Without this task the foot's three rotational
    // DoF are free and the QP twists the leg: measured, hip_yaw walked to its
    // +-30 deg stop over 16 steps and the feet landed yawed by up to 31 deg.
    let yaw0 = {
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
    let gait = GaitPlan::new(&gp);
    let z_com = env_f64("Z_COM", com0.z);
    // How far the ZMP is asked to travel laterally, as a fraction of the
    // half stance width. 1.0 is true single support (all load on one sole);
    // anything less is only reachable with both feet down, so it is a W1
    // instrument, not a gait parameter.
    let zmp_lat_scale = env_f64("ZMP_LAT_SCALE", 1.0);
    // Forward travel per step, in metres of BODY advance. 0 is stepping in
    // place, which stays a special case of this rather than a separate path.
    // A commanded speed maps to it as stride = v * (t_ss + t_ds).
    let stride = env_f64("STRIDE", 0.0);
    // Steps over which the stride ramps in from standing.
    let stride_ramp = env_f64("STRIDE_RAMP", 6.0) as usize;
    // ALTERNATE=0 restores the step-and-close plan, for A/B.
    let footsteps = if flag("ALTERNATE", true) {
        FootstepPlan::alternating(&gait, &steps, stride, stride_ramp)
    } else {
        FootstepPlan::ramped_stride(&gait, &steps, stride, stride_ramp)
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
    let mut max_adapt: f64 = 0.0;
    let mut n_adapt = 0u32;
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
    let x0_body = rig.sim.body_world_position(&rig.robot.root_link).unwrap()[0];
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

    for tick in 0..n_ticks {
        let t = tick as f64 * dt;
        let st = rig.sync();
        let (q, v, v_dvec, data) = (&st.q, &st.v, &st.v_dvec, &st.data);
        let com = st.com;
        let com_vel = st.com_vel;

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
                        footsteps.shift_from(slice_idx, swing, adapt_now);
                        dcm_plan =
                            DcmPlan::from_footsteps(&gait, &footsteps, z_com, zmp_lat_scale);
                        max_adapt = max_adapt.max(adapt_now.norm());
                        n_adapt += 1;
                    }
                    adapt_now = na::Vector2::zeros();
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

        // Ground truth first: everything below solves against this, so it has
        // to be read before the tasks are built, not after the solve.
        let measured = measure_contacts(&rig, data, friction_mu);
        let fz_meas = [measured.f_w[0][2], measured.f_w[1][2]];
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
        // ---- capture-point footstep adaptation --------------------------
        // Steer the NEXT footstep on the DCM's predicted position at
        // touchdown. Recomputed every tick; the prediction horizon shrinks to
        // zero as the step runs out, so the target converges rather than
        // chasing early-step noise.
        let p_raw = commanded_zmp(&xi, &r, omega, k_dcm);
        // Clamp against the support that is REALLY there, which in NO_LIFT is
        // both feet even where the plan says one.
        let sbox = SupportBox::from_stance(
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
        let a_xy = com_accel_xy(&com, &p_cmd, omega);
        // z stays a PD on the nominal height: the LIPM says nothing about it,
        // and a constant-height assumption is exactly what makes omega a
        // constant in the first place.
        let a_z = kp_com * (z_com - com.z) + kd_com * (0.0 - com_vel.z);
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
        let rp_ref =
            bt::trunk_rp_ref(roll, pitch, &st.v_ang_w, kp_trunk, kd_trunk, trunk_dead, trunk_sign);
        // Yaw error against the pose the plan was built in, not against the
        // last tick: the reference is a fixed world direction.
        let yaw_ref = trunk_yaw.then(|| {
            let (_, _, yaw) = st.body_quat.euler_angles();
            kp_yaw * (yaw0 - yaw) + kd_yaw * (0.0 - st.v_ang_w[2])
        });
        let p2 = bt::trunk_rpy(dyn_ctx.qddot(), &j_trunk, &djv_trunk, &rp_ref, yaw_ref, nv);

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
                let plan_xy = steps.sole[side];
                let touch_down = na::Vector3::new(
                    plan_xy.x + adapt_now.x,
                    plan_xy.y + adapt_now.y,
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
                        foot_rot0[side],
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
                        foot_rot0[side],
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
                max_ankle_roll_use = max_ankle_roll_use.max(ankle_roll[i].abs() / ankle_roll_lim[i]);
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
                com_ref_z: z_com,
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

        let cur_z = rig.sim.body_world_position(&rig.robot.root_link).unwrap()[2];
        min_z = min_z.min(cur_z);
        let tilt = roll.abs().max(pitch.abs());
        max_tilt = max_tilt.max(tilt);
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
        // Commanded against achieved travel. A gait that "walks" at half the
        // commanded speed is the classic footstep bookkeeping error, and it
        // looks perfectly stable while doing it.
        let x_end = rig.sim.body_world_position(&rig.robot.root_link).unwrap()[0];
        println!(
            "  travel: planned {:.3} m, body moved {:.3} m ({:.0}%)",
            footsteps.travel_x(),
            x_end - x0_body,
            if footsteps.travel_x() > 1e-6 {
                100.0 * (x_end - x0_body) / footsteps.travel_x()
            } else {
                100.0
            }
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
        "  footstep adaptations: {n_adapt} steps moved, largest {:.1} mm",
        max_adapt * 1e3
    );
    println!(
        "  self-collision ticks: {n_selfcollide}  (peak {max_selfcollide_f:.1} N)          -- any nonzero value means the QP was solving against a contact it has no model for"
    );
    println!("  verdict: {}", if fell { "FELL" } else { "SURVIVED" });
}

#[cfg(not(feature = "mujoco"))]
fn main() {
    eprintln!("This example requires the `mujoco` feature. Run with:");
    eprintln!("  cargo run --features mujoco --example kyo46rs_walk");
}
