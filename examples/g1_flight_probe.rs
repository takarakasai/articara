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
    use articara::biped::log::{Row, TrajLog, measure_contacts};
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
    // INIT_JITTER / INIT_SEED: the walker's per-member posture jitter (each
    // leg joint of the build seed +- INIT_JITTER rad, SplitMix64/xorshift on
    // INIT_SEED), so a battery's members are different situations. 0 = off.
    let init_jitter = env_f64("INIT_JITTER", 0.0).abs();
    if init_jitter > 0.0 {
        let mut state = {
            let mut z = (env_f64("INIT_SEED", 1.0) as u64).wrapping_mul(0x9E37_79B9_7F4A_7C15);
            z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
            z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
            (z ^ (z >> 31)) | 1
        };
        let mut next = || {
            state ^= state << 13;
            state ^= state >> 7;
            state ^= state << 17;
            ((state >> 11) as f64 / (1u64 << 53) as f64) * 2.0 - 1.0
        };
        for name in [
            "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint", "left_knee_joint",
            "left_ankle_pitch_joint", "left_ankle_roll_joint", "right_hip_pitch_joint", "right_hip_roll_joint",
            "right_hip_yaw_joint", "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
        ] {
            o.extra_seed_delta.push((name, init_jitter * next()));
        }
    }
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
    // JUMP_V > 0: a self-propelled vertical jump instead of the pelvis force.
    // From T_LAUNCH: crouch CROUCH m below the stand height over JUMP_T_CROUCH
    // s (quintic), hold JUMP_T_HOLD s, then accelerate the CoM upward at a
    // constant rate so that it reaches JUMP_V m/s at JUMP_Z_TO m relative to
    // the stand height, and from that tick schedule flight: the contact set
    // is empty (the flight tasks take over) until a foot is loaded again
    // after both have left the ground.
    let jump_v = env_f64("JUMP_V", 0.0);
    // DIAG_H=1: decompose the pitch angular momentum through the jump. The
    // only thing that changes the total is the ground's moment about the
    // CoM, M_y = -z_c f_x - (x_cop - x_c) f_z (horizontal force x height,
    // CoP offset x vertical force), integrated from the crouch; and the CMM
    // row split by joint group says which motions carry it.
    let diag_h = flag("DIAG_H", false);
    let mut h_int = [0.0_f64; 2]; // [horizontal-force part, CoP-offset part]
    let mut h_start: Option<f64> = None;
    // DIAG_HX=1: the same for the ROLL momentum h_x, split by phase (crouch,
    // hold, push, release, loaded after the release) and printed once at
    // lift-off. The ground's roll moment about the CoM is
    // M_x = sum_k (p_k,y - c_y) f_z,k + (c_z - p_k,z) f_y,k, split as
    // [L/R vertical-force difference (sole centre about the feet's midpoint),
    //  CoP inside the sole, CoM off the feet's midpoint, lateral force x
    //  height]. Measured 2026-10-25: |h_x| at lift-off decides the falls.
    let diag_hx = flag("DIAG_HX", false);
    let mut hx_int = [[0.0_f64; 4]; 5];
    let mut hx_at = [f64::NAN; 6];
    let mut hx_fz = [[0.0_f64; 2]; 5]; // integral of (fz_L - fz_R) dt, and of dt
    let mut hx_done = false;
    // JUMP_PUSH_R: the push acceleration falls linearly to R times its start
    // value, keeping the push distance and the take-off speed (R = 1 = the
    // constant acceleration). The leg's vertical authority falls as it
    // straightens; a constant demand runs into that at the end of the push.
    let jump_push_r = env_f64("JUMP_PUSH_R", 1.0).clamp(0.0, 1.0);
    // JUMP_HEEL_CUT (m): during the push and release, move the heel edge of
    // each sole's CoP box forward by this much (toe edge kept). Measured
    // 2026-10-21: the CoP ran to the heel edge at the end of the push and
    // the foot rolled back, leaving h_y = 3-5 kg m^2/s at take-off. 0 = off.
    let jump_heel_cut = env_f64("JUMP_HEEL_CUT", 0.0).max(0.0);
    // MOM_TASK=1: in stance, drive the centroidal angular momentum to zero
    // (dh = -KP_MOM h, weight WBCW_MOMENTUM, default 0.1 as in the walker).
    // Measured 2026-10-21: the push-off without it left the jump with
    // h_y = +5.2 kg m^2/s, the body pitched forward in flight and landed on
    // one foot 9 cm behind the CoM.
    let mom_task = flag("MOM_TASK", false);
    let kp_mom = env_f64("KP_MOM", 20.0);
    let mom_w = env_f64("WBCW_MOMENTUM", 0.1);
    let crouch = env_f64("CROUCH", 0.12);
    let jump_t_crouch = env_f64("JUMP_T_CROUCH", 0.8).max(0.05);
    let jump_t_hold = env_f64("JUMP_T_HOLD", 0.3).max(0.0);
    let jump_z_to = env_f64("JUMP_Z_TO", 0.02);
    let mut push_end: Option<f64> = None;
    // JUMP_RELEASE=1: after the push reaches JUMP_V, command the CoM to fall
    // at g (zero ground force) with the contact rows still in, and enter
    // flight only when both feet are measured unloaded (or after
    // JUMP_T_RELEASE s). Measured 2026-10-21: scheduling flight at the end of
    // the push, with the feet still pressing, put h_y = +5 kg m^2/s into the
    // jump in the first 25 ms.
    let jump_release = flag("JUMP_RELEASE", false);
    let jump_t_release = env_f64("JUMP_T_RELEASE", 0.08).max(0.0);
    // JUMP_ABORT=1: if the release times out with a foot still loaded (the
    // push was too weak to leave the ground), do not schedule flight -- stay
    // in stance and absorb from the current state. Measured 2026-10-21: the
    // scheduled flight on the ground, with no touchdown to end it, felled
    // 44/48 of the JUMP_V=1.0 battery.
    let jump_abort = flag("JUMP_ABORT", false);
    let mut release_t0: Option<(f64, f64, f64)> = None;
    // RELEASE_FIRST_OFF=1: end the release as soon as EITHER foot is
    // unloaded (not both): the flight tasks then lift the other foot.
    // Measured 2026-10-25 (DIAG_HX): 91% of the lift-off roll momentum is
    // made in the release, by the foot left on the ground pushing sideways;
    // releases under 25 ms fell 21/115, longer ones 50/93.
    let release_first_off = flag("RELEASE_FIRST_OFF", false);
    let wbcw_contact = env_f64("WBCW_CONTACT", 0.1);
    let wbcw_contact_rest = std::env::var("WBCW_CONTACT_REST").ok().and_then(|s| s.parse::<f64>().ok());
    let wbcw_contact_rest_after = env_f64("WBCW_CONTACT_REST_AFTER", 0.5);
    // WBCW_CONTACT_REST_SETTLE_V (m/s, 0 = off): after landing, switch to
    // the rest weight only once the CoM speed is below this and the trunk
    // roll and pitch below WBCW_CONTACT_REST_SETTLE_TILT rad (and at least
    // WBCW_CONTACT_REST_AFTER s after touchdown); latched from then on.
    // Measured 2026-10-25: a timed switch keeps JUMP_V 1.0/1.5 standing
    // (47-48/48) but drops 2.0 38->9, whose recovery from the backward
    // landing still needs the contact weight 1.
    let rest_settle_v = env_f64("WBCW_CONTACT_REST_SETTLE_V", 0.0);
    let rest_settle_tilt = env_f64("WBCW_CONTACT_REST_SETTLE_TILT", 0.03);
    let mut rest_settled = false;
    // WBCW_CONTACT_REST_LAUNCH=1: the jump's window (weight WBCW_CONTACT)
    // starts at T_LAUNCH (the crouch) instead of the push. Measured
    // 2026-10-25: the rest weight through the crouch and hold raises the
    // take-off speed (JUMP_V 2.0: 1.23 -> 1.33 m/s) and the landing with it,
    // and 2.0 fell 38 -> 20 whatever the switch after landing.
    let rest_launch = flag("WBCW_CONTACT_REST_LAUNCH", false);
    // DIAG_LAND=1: for 3 s after the first touchdown, every 20 ms: CoM x
    // (actual, reference, velocity), the CoM task's commanded x acceleration
    // against the one the QP solved for, trunk pitch and rate, and per foot
    // CoP x and load -- where the post-landing forward run comes from.
    let diag_land = flag("DIAG_LAND", false);
    let trunk_land_vz = env_f64("WBCW_TRUNK_LAND_VZ", 0.0);
    let kp_trunk_land = env_f64("KP_TRUNK_LAND", kp_trunk);
    let kd_trunk_land = env_f64("KD_TRUNK_LAND", kd_trunk);
    let mut diag_com_cmd: Option<(na::Vector3<f64>, na::Vector3<f64>)> = None; // (ref, a_cmd)
    let mut air_sched = false;
    let mut left_ground = false;
    let f_on = env_f64("F_ON", 0.05 * weight);
    let on_ticks = env_f64("ON_TICKS", 2.0) as u32;
    // Touchdown is admitted after LAND_TICKS loaded ticks (lift-off keeps
    // ON_TICKS): every tick of a landing handled as flight is a tick of
    // impact the QP does not see.
    let land_ticks = env_f64("LAND_TICKS", 1.0) as u32;
    // LAND_ABSORB=1: after touchdown the CoM follows a quintic from the
    // touchdown state (position, velocity) to rest over LAND_T s (stopping
    // distance v*T/2, peak deceleration 1.5 v/T), holds LAND_HOLD s, then
    // rises to the stand height over LAND_RISE s -- with the reference's own
    // velocity and acceleration fed forward. Off = the probe's original
    // landing: hold (x, y, stand height) at rest from the first tick.
    let land_absorb = flag("LAND_ABSORB", false);
    let land_t = env_f64("LAND_T", 0.25).max(0.02);
    let land_hold = env_f64("LAND_HOLD", 0.3).max(0.0);
    let land_rise = env_f64("LAND_RISE", 1.0).max(0.05);
    // LAND_STEP=1: after touchdown, if the DCM (c + v/omega) leaves the
    // feet's fore-aft support (within LAND_STEP_MARGIN of an edge), take one
    // capture step: the foot farther from the DCM's lateral position swings
    // for LAND_STEP_T s to where the DCM will be at its touchdown (CoP held
    // at the stance sole centre), clamped to +-LAND_STEP_MAX from the stance
    // foot; the CoM's horizontal reference follows the LIPM meanwhile.
    // Measured 2026-10-21: jumps that land fall >1 s later, the backward
    // velocity left at touchdown carrying the CoM out over the heels with no
    // step to stop it. Off = bit-identical.
    let land_step = flag("LAND_STEP", false);
    let land_latch = flag("LAND_LATCH", false);
    // DIAG_TD=1: after the first flight, print each foot's first loaded tick
    // with both feet's sole height and roll / pitch (diagnostic only).
    let diag_td = flag("DIAG_TD", false);
    let mut first_load: [Option<f64>; 2] = [None, None];
    let land_step_t = env_f64("LAND_STEP_T", 0.35).max(0.1);
    let land_step_h = env_f64("LAND_STEP_H", 0.04).max(0.0);
    let land_step_margin = env_f64("LAND_STEP_MARGIN", 0.02);
    let land_step_max = env_f64("LAND_STEP_MAX", 0.35).max(0.0);
    let land_step_bx = env_f64("LAND_STEP_BX", 0.0);
    let land_step_n = env_f64("LAND_STEP_N", 1.0).max(0.0) as usize;
    let land_step_w = env_f64("LAND_STEP_W", 1.0);
    let kp_ls = env_f64("KP_LAND_STEP", 400.0);
    let kd_ls = env_f64("KD_LAND_STEP", 40.0);
    // (swing side, start time, start position, target position, stance sole centre xy)
    let mut step_st: Option<(usize, f64, na::Vector3<f64>, na::Vector3<f64>, na::Vector2<f64>)> = None;
    let mut steps_done = 0usize;
    // quintic p0,v0 -> p1 at rest over T (a0 = a1 = 0): (p, v, a) at t
    let quintic = |p0: f64, v0: f64, p1: f64, tt: f64, t: f64| -> (f64, f64, f64) {
        let t = t.clamp(0.0, tt);
        let h = p1 - p0;
        let c3 = (20.0 * h - 12.0 * v0 * tt) / (2.0 * tt.powi(3));
        let c4 = (-30.0 * h + 16.0 * v0 * tt) / (2.0 * tt.powi(4));
        let c5 = (12.0 * h - 6.0 * v0 * tt) / (2.0 * tt.powi(5));
        (
            p0 + v0 * t + c3 * t.powi(3) + c4 * t.powi(4) + c5 * t.powi(5),
            v0 + 3.0 * c3 * t.powi(2) + 4.0 * c4 * t.powi(3) + 5.0 * c5 * t.powi(4),
            6.0 * c3 * t + 12.0 * c4 * t.powi(2) + 20.0 * c5 * t.powi(3),
        )
    };
    let mut land: Option<(f64, na::Vector3<f64>, na::Vector3<f64>)> = None;
    let mut min_com_z_after = f64::INFINITY;
    let (mut peak_grf_after, mut peak_tau_after, mut td_vz) = (0.0_f64, 0.0_f64, 0.0_f64);
    let flight_trunk = flag("FLIGHT_TRUNK", true);
    let flight_post = flag("FLIGHT_POST", true);
    let flight_keep_com = flag("FLIGHT_KEEP_COM", false);
    // FLIGHT_FEET=1: in flight, hold each foot at its lift-off offset from
    // the CoM (position, Xyz rows, weight FLIGHT_FEET_W above the trunk's
    // 0.3). Without it the trunk task keeps the trunk upright against any
    // lift-off angular momentum and the legs take the rotation: measured
    // 2026-10-21, both feet 175 mm behind the CoM at touchdown.
    let flight_feet = flag("FLIGHT_FEET", false);
    let flight_feet_w = env_f64("FLIGHT_FEET_W", 1.0);
    let kp_ff = env_f64("KP_FLIGHT_FEET", 400.0);
    let kd_ff = env_f64("KD_FLIGHT_FEET", 40.0);
    let flight_feet_flat = flag("FLIGHT_FEET_FLAT", false);
    let flight_feet_level = flag("FLIGHT_FEET_LEVEL", false);
    // FOOT_RESEAT=1: on the ground after the first touchdown, a foot that is
    // out of the contact set (and not the landing step's swing foot) gets a
    // task that brings it straight down, sole level, to the floor under
    // where it left contact (5 mm below the standing foot height, so it
    // presses), until the measured load puts it back in the contact set.
    // Off = bit-identical.
    let foot_reseat = flag("FOOT_RESEAT", false);
    let jump_mom_x_w = env_f64("JUMP_MOM_X_W", 0.0).max(0.0);
    let jump_fz_sym_w = env_f64("JUMP_FZ_SYM_W", 0.0).max(0.0);
    let foot_reseat_w = env_f64("FOOT_RESEAT_W", 1.0);
    let mut reseat_xy: [Option<na::Vector2<f64>>; 2] = [None, None];
    let mut foot_stand_z: Option<[f64; 2]> = None;
    let kp_ffr = env_f64("KP_FLIGHT_FEET_ROT", 400.0);
    let kd_ffr = env_f64("KD_FLIGHT_FEET_ROT", 40.0);
    let mut feet_off: Option<[na::Vector3<f64>; 2]> = None;
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
    // TRAJ_CSV: the walker's trajectory format, for render_com_squat_video.py.
    let mut log = TrajLog::create(std::env::var("TRAJ_CSV").ok(), prof.log_joints.to_vec(), &["flight", "com_vz"]);

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

        if jump_v <= 0.0 && !launched && t >= t_launch {
            launched = true;
            let fz = weight + total_mass * launch_v / launch_dur;
            // MuJoCo applies xfrc_applied at the pelvis' own CoM, not the
            // robot's: LAUNCH_THRU_COM=1 (default) adds the couple that moves
            // the line of action through the robot CoM, so the launch leaves
            // no angular momentum and the probe measures the controller, not
            // the harness (without it: 175 mm foot drift / forward fall).
            let mut torque = [0.0; 3];
            if flag("LAUNCH_THRU_COM", true) {
                if let Some(l) = rig.mass_links.iter().find(|l| l.mi == 1) {
                    let rot = misarta::se3::rotation_matrix(&data.oMi[1]);
                    let pc = misarta::se3::translation(&data.oMi[1]) + rot * l.com_local;
                    let d = com - pc;
                    let tq = d.cross(&na::Vector3::new(0.0, 0.0, fz));
                    torque = [tq.x, tq.y, tq.z];
                    println!("  [launch] pelvis CoM offset from robot CoM ({:+.3},{:+.3},{:+.3}) -> couple ({:+.2},{:+.2},{:+.2}) N*m", -d.x, -d.y, -d.z, tq.x, tq.y, tq.z);
                }
            }
            rig.sim.apply_external_force(&rig.robot.root_link, [0.0, 0.0, fz], torque, launch_dur);
            println!("  [launch] t={t:.3} pelvis force {fz:.1} N for {launch_dur:.3} s");
        }

        // ---- measured contact state -------------------------------------
        let measured = measure_contacts(&rig, data, friction_mu);
        for s in 0..2 {
            let loaded = measured.cop[s][2] > f_on;
            // LAND_LATCH=1: after the first touchdown a foot that has landed
            // stays in the contact set when its load drops (unless it is the
            // landing step's swing foot). Measured 2026-10-24: after landing
            // the measured set chattered between one and two feet for
            // seconds, and the unloaded foot had no task at all (flight feet
            // only in flight) -- it wandered and the CoM rocked +-4 cm and
            // drifted backward until the fall.
            if land_latch && !loaded && in_contact[s] && touchdown_t.is_some() && step_st.is_none_or(|x| x.0 != s) {
                flip_count[s] = 0;
                continue;
            }
            if loaded != in_contact[s] {
                flip_count[s] += 1;
                if flip_count[s] >= if loaded { land_ticks } else { on_ticks } {
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
        if air_sched {
            let unloaded = measured.cop[0][2] <= f_on && measured.cop[1][2] <= f_on;
            if diag_hx && unloaded && !left_ground {
                let cmm = misarta::centroidal::compute_centroidal_momentum_matrix(&rig.model, q);
                let h = &cmm * v_dvec;
                println!("  [airborne] t={t:.3} both feet unloaded: h_ang ({:+.3},{:+.3},{:+.3}) com vz {:+.3}", h[0], h[1], h[2], com_vel.z);
            }
            left_ground |= unloaded;
            let loaded = (0..2).any(|k| measured.cop[k][2] > f_on);
            if left_ground && loaded {
                air_sched = false;
                for k in 0..2 {
                    in_contact[k] = measured.cop[k][2] > f_on;
                    flip_count[k] = 0;
                }
            } else {
                for k in 0..2 {
                    if in_contact[k] {
                        anchors.release(k);
                    }
                    in_contact[k] = false;
                    flip_count[k] = 0;
                }
            }
        }
        if let Some((sw, t0s, _, _, _)) = step_st {
            // The swing foot is out of the contact set until it has swung
            // half way and carries load again; then the step is over.
            let sstep = (t - t0s) / land_step_t;
            if sstep >= 0.5 && measured.cop[sw][2] > f_on {
                in_contact[sw] = true;
                flip_count[sw] = 0;
                step_st = None;
                steps_done += 1;
                land = Some((t, com, com_vel));
                let fx = misarta::se3::translation(&data.oMi[feet[sw]]);
                println!("  [step-down] t={t:.3} after {:.3} s, foot ({:+.3},{:+.3}), com ({:+.3},{:+.3}) v ({:+.3},{:+.3})", t - t0s, fx.x, fx.y, com.x, com.y, com_vel.x, com_vel.y);
            } else {
                if in_contact[sw] {
                    anchors.release(sw);
                }
                in_contact[sw] = false;
                flip_count[sw] = 0;
            }
        }
        if diag_td && (!flight_spans.is_empty() || flight_t0.is_some()) {
            for k in 0..2 {
                if first_load[k].is_none() && measured.cop[k][2] > f_on {
                    first_load[k] = Some(t);
                    let pose = |j: usize| {
                        let m = misarta::se3::rotation_matrix(&data.oMi[feet[j]]);
                        let (r, p, _) = na::Rotation3::from_matrix_unchecked(m).euler_angles();
                        (misarta::se3::translation(&data.oMi[feet[j]]).z - prof.sole_below_origin, r, p, measured.cop[j][2])
                    };
                    let (z0, r0, p0, f0) = pose(0);
                    let (z1, r1, p1, f1) = pose(1);
                    println!("  [td-foot] {} first load t={t:.3} | L sole z {z0:+.4} roll {r0:+.3} pitch {p0:+.3} fz {f0:.0} | R sole z {z1:+.4} roll {r1:+.3} pitch {p1:+.3} fz {f1:.0}", if k == 0 { "L" } else { "R" });
                }
            }
        }
        let fz_stand = *foot_stand_z.get_or_insert([misarta::se3::translation(&data.oMi[feet[0]]).z, misarta::se3::translation(&data.oMi[feet[1]]).z]);
        for k in 0..2 {
            if in_contact[k] || flight_t0.is_some() {
                reseat_xy[k] = None;
            } else if reseat_xy[k].is_none() {
                reseat_xy[k] = Some(misarta::se3::translation(&data.oMi[feet[k]]).xy());
            }
        }
        let stance: Vec<usize> = (0..2).filter(|&s| in_contact[s]).map(|s| feet[s]).collect();
        let nc = stance.len();
        let flight = nc == 0;
        match (flight, flight_t0) {
            (true, None) => {
                flight_t0 = Some(t);
                h_ang0 = None;
                feet_off = Some(std::array::from_fn(|k| misarta::se3::translation(&data.oMi[feet[k]]) - com));
                // FLIGHT_FEET_LEVEL=1: hold both feet at the LOWER foot's
                // height below the CoM, so they meet a flat floor together.
                // Measured 2026-10-24 (flat feet): the fallers touched down
                // 70 ms apart (survivors 20 ms) with an 8.9 mm height gap.
                if flight_feet_level && let Some(off) = feet_off.as_mut() {
                    let zl = off[0].z.min(off[1].z);
                    off[0].z = zl;
                    off[1].z = zl;
                }
                let fx: Vec<String> = (0..2).map(|k| format!("{:+.3}", misarta::se3::translation(&data.oMi[feet[k]]).x)).collect();
                let fz: Vec<String> = (0..2).map(|k| format!("{:+.3}", misarta::se3::translation(&data.oMi[feet[k]]).z)).collect();
                let cmm = misarta::centroidal::compute_centroidal_momentum_matrix(&rig.model, q);
                let h = &cmm * v_dvec;
                let (r, pch, _) = st.body_quat.euler_angles();
                println!("  [lift-off] t={t:.3} com ({:+.3}, z {:.4}) v ({:+.3}, {:+.3}), foot x {:?} z {:?}, h_ang ({:+.3},{:+.3},{:+.3}), roll {r:+.3} pitch {pch:+.3}", com.x, com.z, com_vel.x, com_vel.z, fx, fz, h[0], h[1], h[2]);
            }
            (false, Some(t0)) => {
                flight_spans.push((t0, t));
                flight_t0 = None;
                if touchdown_t.is_none() {
                    td_vz = com_vel.z;
                }
                touchdown_t.get_or_insert(t);
                com_ref = Some(na::Vector3::new(com.x, com.y, stand_z));
                if land_absorb {
                    land = Some((t, com, com_vel));
                }
                let fx: Vec<String> = (0..2).map(|k| format!("{:+.3}", misarta::se3::translation(&data.oMi[feet[k]]).x)).collect();
                println!("  [touchdown] t={t:.3} after {:.3} s, com ({:+.3}, z {:.4}) v ({:+.3}, {:+.3}), foot x {:?}, feet {:?}", t - t0, com.x, com.z, com_vel.x, com_vel.z, fx, in_contact);
            }
            _ => {}
        }
        let c_ref = *com_ref.get_or_insert(com);
        if land_step
            && step_st.is_none()
            && steps_done < land_step_n
            && in_contact[0]
            && in_contact[1]
            && touchdown_t.is_some_and(|t0| t - t0 >= 0.05 && t - t0 <= 2.0)
        {
            let foot = |k: usize| misarta::se3::translation(&data.oMi[feet[k]]);
            let centre = |k: usize| na::Vector2::new(foot(k).x + prof.sole_centre_x, foot(k).y);
            let z_sole = 0.5 * (foot(0).z + foot(1).z) - prof.sole_below_origin;
            let omega = (G / (com.z - z_sole).max(0.2)).sqrt();
            let xi = com.xy() + com_vel.xy() / omega;
            let lo = centre(0).x.min(centre(1).x) - sole_half_l;
            let hi = centre(0).x.max(centre(1).x) + sole_half_l;
            if xi.x < lo + land_step_margin || xi.x > hi - land_step_margin {
                let st_side = if (centre(0).y - xi.y).abs() <= (centre(1).y - xi.y).abs() { 0 } else { 1 };
                let sw = 1 - st_side;
                let p_st = centre(st_side);
                let xi_t = p_st + (xi - p_st) * (omega * land_step_t).exp();
                let dx = (xi_t.x + land_step_bx - p_st.x).clamp(-land_step_max, land_step_max);
                let p0 = foot(sw);
                let p1 = na::Vector3::new(p_st.x + dx - prof.sole_centre_x, p0.y, p0.z);
                anchors.release(sw);
                in_contact[sw] = false;
                step_st = Some((sw, t, p0, p1, p_st));
                println!("  [step] t={t:.3} xi ({:+.3},{:+.3}) outside x [{lo:+.3},{hi:+.3}]: swing {} from x {:+.3} to {:+.3} (xi at T {:+.3}), stance centre ({:+.3},{:+.3})", xi.x, xi.y, if sw == 0 { "L" } else { "R" }, p0.x, p1.x, xi_t.x, p_st.x, p_st.y);
            }
        }

        if verbose && jump_v > 0.0 && !flight && t >= t_launch && push_end.is_none() {
            let cmm = misarta::centroidal::compute_centroidal_momentum_matrix(&rig.model, q);
            let h = &cmm * v_dvec;
            let (_, pch, _) = st.body_quat.euler_angles();
            let fx = misarta::se3::translation(&data.oMi[feet[0]]).x;
            println!("  [push] t={t:.3} com x {:+.4} z {:.4} vz {:+.3} h_y {:+.3} pitch {pch:+.4} cop_x L {:+.4} R {:+.4} fz {:.0} {:.0} foot x {fx:+.4}",
                com.x, com.z, com_vel.z, h[1], measured.cop[0][0], measured.cop[1][0], measured.cop[0][2], measured.cop[1][2]);
        }
        if diag_h && jump_v > 0.0 && t >= t_launch {
            let cmm = misarta::centroidal::compute_centroidal_momentum_matrix(&rig.model, q);
            let mut grp = [0.0_f64; 5]; // base, left leg, right leg, waist, arms
            for c in 0..nv {
                let g = if c < 6 {
                    0
                } else {
                    let name = rig.actuated().iter().find(|&&(_, vi)| vi == c).map(|&(ji, _)| rig.robot.joints[ji].name.as_str()).unwrap_or("");
                    if name.starts_with("left_") && (name.contains("hip") || name.contains("knee") || name.contains("ankle")) {
                        1
                    } else if name.starts_with("right_") && (name.contains("hip") || name.contains("knee") || name.contains("ankle")) {
                        2
                    } else if name.contains("waist") || name.contains("torso") {
                        3
                    } else {
                        4
                    }
                };
                grp[g] += cmm[(1, c)] * v[c];
            }
            let h_y: f64 = grp.iter().sum();
            let (plant_h, plant_vx, plant_vz) = rig.sim.subtree_momentum(&rig.robot.root_link).map_or((f64::NAN, f64::NAN, f64::NAN), |(am, lv)| (am[1], lv[0], lv[2]));
            let h0 = *h_start.get_or_insert(h_y);
            if !flight {
                for k in 0..2 {
                    let f = measured.f_w[k];
                    if f[2] > 0.0 {
                        let px = measured.patch_w[k][0];
                        h_int[0] += -com.z * f[0] * dt;
                        h_int[1] += -(px - com.x) * f[2] * dt;
                    }
                }
            }
            if tick % 5 == 0 && push_end.map_or(true, |te| t < te + 0.05) && t >= t_launch + jump_t_crouch + jump_t_hold - 0.05 {
                println!(
                    "  [h] t={t:.3} {} h_y {:+.3} plant {plant_h:+.3} vcom model ({:+.3},{:+.3}) plant ({plant_vx:+.3},{plant_vz:+.3}) (d {:+.3}; ground int: fx*h {:+.3} cop*fz {:+.3} sum {:+.3}) | base {:+.3} Lleg {:+.3} Rleg {:+.3} waist {:+.3} arms {:+.3} | fz {:.0} {:.0} fx {:+.1} {:+.1}",
                    if flight { "AIR" } else { "GND" }, h_y, com_vel.x, com_vel.z, h_y - h0, h_int[0], h_int[1], h_int[0] + h_int[1],
                    grp[0], grp[1], grp[2], grp[3], grp[4],
                    measured.f_w[0][2], measured.f_w[1][2], measured.f_w[0][0], measured.f_w[1][0]
                );
            }
        }
        if diag_hx && jump_v > 0.0 && t >= t_launch && !hx_done {
            let cmm = misarta::centroidal::compute_centroidal_momentum_matrix(&rig.model, q);
            let h_x: f64 = (0..nv).map(|c| cmm[(0, c)] * v[c]).sum();
            if flight {
                hx_at[5] = h_x;
                hx_done = true;
                let mut grp = [0.0_f64; 5]; // base, left leg, right leg, waist, arms
                for c in 0..nv {
                    let g = if c < 6 {
                        0
                    } else {
                        let name = rig.actuated().iter().find(|&&(_, vi)| vi == c).map(|&(ji, _)| rig.robot.joints[ji].name.as_str()).unwrap_or("");
                        if name.starts_with("left_") && (name.contains("hip") || name.contains("knee") || name.contains("ankle")) {
                            1
                        } else if name.starts_with("right_") && (name.contains("hip") || name.contains("knee") || name.contains("ankle")) {
                            2
                        } else if name.contains("waist") || name.contains("torso") {
                            3
                        } else {
                            4
                        }
                    };
                    grp[g] += cmm[(0, c)] * v[c];
                }
                println!("  [hx] lift-off h_x {h_x:+.3} | base {:+.3} Lleg {:+.3} Rleg {:+.3} waist {:+.3} arms {:+.3}", grp[0], grp[1], grp[2], grp[3], grp[4]);
                for (ph, name) in ["crouch", "hold", "push", "release", "loaded"].iter().enumerate() {
                    let d = hx_at[ph + 1..].iter().find(|x| x.is_finite()).copied().unwrap_or(h_x) - hx_at[ph];
                    let m = &hx_int[ph];
                    println!(
                        "  [hx] {name:7} dt {:.3} dh_x {:+.3} ground {:+.3} = fzdiff {:+.3} cop {:+.3} com {:+.3} fy*h {:+.3} | mean fzL-fzR {:+.1}",
                        hx_fz[ph][1], if hx_at[ph].is_finite() { d } else { f64::NAN }, m.iter().sum::<f64>(), m[0], m[1], m[2], m[3],
                        hx_fz[ph][0] / hx_fz[ph][1].max(1e-9)
                    );
                }
            } else {
                let ph = if t < t_launch + jump_t_crouch {
                    0
                } else if t < t_launch + jump_t_crouch + jump_t_hold {
                    1
                } else if push_end.is_none() && release_t0.is_none() {
                    2
                } else if push_end.is_none() {
                    3
                } else {
                    4
                };
                if !hx_at[ph].is_finite() {
                    hx_at[ph] = h_x;
                }
                let sole = |k: usize| {
                    let o = misarta::se3::translation(&data.oMi[feet[k]]);
                    let r = misarta::se3::rotation_matrix(&data.oMi[feet[k]]);
                    o + r * na::Vector3::new(prof.sole_centre_x, 0.0, -prof.sole_below_origin)
                };
                let s = [sole(0), sole(1)];
                let y_mid = 0.5 * (s[0].y + s[1].y);
                for k in 0..2 {
                    let f = measured.f_w[k];
                    if f[2] > 0.0 {
                        let py = measured.patch_w[k][1];
                        hx_int[ph][0] += (s[k].y - y_mid) * f[2] * dt;
                        hx_int[ph][1] += (py - s[k].y) * f[2] * dt;
                        hx_int[ph][2] += (y_mid - com.y) * f[2] * dt;
                        hx_int[ph][3] += (com.z - s[k].z) * f[1] * dt;
                    }
                }
                hx_fz[ph][0] += (measured.f_w[0][2] - measured.f_w[1][2]) * dt;
                hx_fz[ph][1] += dt;
            }
        }
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
                &|_| 1.0, &contact_cfg, &|_| bt::ContactMode::Rigid,
                &|_, _| {
                    let in_push = jump_v > 0.0 && push_end.is_none() && t >= t_launch + jump_t_crouch + jump_t_hold;
                    (in_push && jump_heel_cut > 0.0).then(|| {
                        let c = jump_heel_cut.min(sole_half_l);
                        ([prof.sole_centre_x + 0.5 * c, 0.0, -prof.sole_below_origin], ((sole_half_l - 0.5 * c).max(0.005), sole_half_w))
                    })
                },
            );
            levels.push(p0.task);
            names.push("contact+cones");
        }
        if !flight || flight_keep_com {
            let (mut c_ref, mut cd_ref, mut cdd_ref) = match land {
                Some((t0, c0, v0)) => {
                    let s = t - t0;
                    let mut c = na::Vector3::zeros();
                    let mut cd = na::Vector3::zeros();
                    let mut cdd = na::Vector3::zeros();
                    for i in 0..3 {
                        let p1 = c0[i] + 0.5 * v0[i] * land_t;
                        let (p, vv, aa) = quintic(c0[i], v0[i], p1, land_t, s);
                        c[i] = p;
                        cd[i] = vv;
                        cdd[i] = aa;
                    }
                    let s_rise = s - land_t - land_hold;
                    if s_rise > 0.0 {
                        let z_low = c0.z + 0.5 * v0.z * land_t;
                        let (p, vv, aa) = quintic(z_low, 0.0, stand_z, land_rise, s_rise);
                        c.z = p;
                        cd.z = vv;
                        cdd.z = aa;
                    }
                    (c, cd, cdd)
                }
                None if jump_v > 0.0 && t >= t_launch && push_end.is_none() => {
                    // crouch, hold, push off
                    let z_low = stand_z - crouch;
                    let s = t - t_launch;
                    let (mut z, mut vz, mut az) = quintic(stand_z, 0.0, z_low, jump_t_crouch, s);
                    let s_push = s - jump_t_crouch - jump_t_hold;
                    if s_push > 0.0 {
                        let dz = stand_z + jump_z_to - z_low;
                        // a(s) = a0 (1 - (1 - r) s / T): v(T) = a0 T (1 + r) / 2,
                        // z(T) = a0 T^2 (2 + r) / 6.
                        let r = jump_push_r;
                        let (t_push, a) = if r < 1.0 {
                            let t_push = 3.0 * dz * (1.0 + r) / ((2.0 + r) * jump_v);
                            let a0 = 2.0 * jump_v / ((1.0 + r) * t_push);
                            let sp = s_push.min(t_push);
                            let k = (1.0 - r) / t_push;
                            z = z_low + a0 * (0.5 * sp * sp - k * sp.powi(3) / 6.0);
                            vz = a0 * (sp - 0.5 * k * sp * sp);
                            az = a0 * (1.0 - k * sp);
                            (t_push, a0)
                        } else {
                            let a = jump_v * jump_v / (2.0 * dz);
                            z = z_low + 0.5 * a * s_push * s_push;
                            vz = a * s_push;
                            az = a;
                            (jump_v / a, a)
                        };
                        if s_push >= t_push && jump_release {
                            let (t0, z0, v0) = *release_t0.get_or_insert((t, z, vz));
                            let sr = t - t0;
                            z = z0 + v0 * sr - 0.5 * G * sr * sr;
                            vz = v0 - G * sr;
                            az = -G;
                            let unloaded = measured.cop[0][2] <= f_on && measured.cop[1][2] <= f_on;
                            let one_off = release_first_off && !unloaded && (measured.cop[0][2] <= f_on || measured.cop[1][2] <= f_on);
                            if one_off {
                                push_end = Some(t);
                                air_sched = true;
                                left_ground = false;
                                println!("  [release] t={t:.3} after {sr:.3} s (one foot unloaded: fz {:.0} {:.0})", measured.cop[0][2], measured.cop[1][2]);
                            } else if !unloaded && sr >= jump_t_release && jump_abort {
                                push_end = Some(t);
                                land = Some((t, com, com_vel));
                                println!("  [abort] t={t:.3} feet still loaded after {sr:.3} s: no flight, absorb in stance");
                            } else if unloaded || sr >= jump_t_release {
                                push_end = Some(t);
                                air_sched = true;
                                left_ground = false;
                                println!("  [release] t={t:.3} after {sr:.3} s ({})", if unloaded { "feet unloaded" } else { "timeout" });
                            }
                        } else if s_push >= t_push {
                            push_end = Some(t);
                            air_sched = true;
                            left_ground = false;
                            println!("  [take-off] t={t:.3} scheduled flight: com z {:.4} vz {:+.3} (ref {:.4} {:+.3}), push {t_push:.3} s at {a:.2} m/s^2", com.z, com_vel.z, z, vz);
                        }
                    }
                    (na::Vector3::new(c_ref.x, c_ref.y, z), na::Vector3::new(0.0, 0.0, vz), na::Vector3::new(0.0, 0.0, az))
                }
                None => (c_ref, na::Vector3::zeros(), na::Vector3::zeros()),
            };
            if let Some((_, _, _, _, p_st)) = step_st {
                // Single support on the stance sole: the CoM's horizontal
                // motion is the LIPM's own (no feedback to fight the step).
                let foot_z = misarta::se3::translation(&data.oMi[feet[0]]).z.min(misarta::se3::translation(&data.oMi[feet[1]]).z) - prof.sole_below_origin;
                let w2 = G / (com.z - foot_z).max(0.2);
                c_ref.x = com.x;
                c_ref.y = com.y;
                cd_ref.x = com_vel.x;
                cd_ref.y = com_vel.y;
                cdd_ref.x = w2 * (com.x - p_st.x);
                cdd_ref.y = w2 * (com.y - p_st.y);
            }
            let a_com = cdd_ref + kd_com * (cd_ref - com_vel) + kp_com * (c_ref - com);
            diag_com_cmd = Some((c_ref, a_com));
            levels.push(bt::com(dyn_ctx.qddot(), &st.j_com, &st.djv_com, &a_com));
            names.push("com");
        }
        let j_trunk = misarta::jacobian::compute_joint_jacobian_from_data(&rig.model, q, data, rig.trunk_mi);
        let djv_trunk = &misarta::jacobian::compute_joint_jacobian_time_derivative(&rig.model, q, v, rig.trunk_mi) * v_dvec;
        let (roll, pitch, _) = st.body_quat.euler_angles();
        if !flight || flight_trunk {
            // KP/KD_TRUNK_LAND: the trunk gains after a landing faster than
            // WBCW_TRUNK_LAND_VZ (default = KP/KD_TRUNK). Measured
            // 2026-10-28: the weight 0.03 frees the CoM but lets the trunk
            // pitch on from 0.12 to 0.52 rad in 0.5 s (13 of 15 2.0 falls).
            let hard_land = touchdown_t.is_some() && (trunk_land_vz <= 0.0 || td_vz < -trunk_land_vz);
            let (kp_t, kd_t) = if hard_land { (kp_trunk_land, kd_trunk_land) } else { (kp_trunk, kd_trunk) };
            let rp_ref = bt::trunk_rp_ref(roll, pitch, &st.v_ang_w, kp_t, kd_t, 0.0, 1.0);
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
        if !flight && mom_task {
            let cmm = misarta::centroidal::compute_centroidal_momentum_matrix(&rig.model, q);
            let dcmm_v = misarta::centroidal::compute_cmm_dot_times_v(&rig.model, q, v);
            let h_now = na::DVector::from_column_slice((&cmm * v_dvec).as_slice());
            let dcmm_v = na::DVector::from_column_slice(dcmm_v.as_slice());
            let cmm = na::DMatrix::from_column_slice(cmm.nrows(), cmm.ncols(), cmm.as_slice());
            if let Some(tk) = bt::angular_momentum(dyn_ctx.qddot(), &cmm, &dcmm_v, &h_now, &[0.0; 3], &[0.0; 3], kp_mom, [true; 3], nv) {
                levels.push(tk);
                names.push("momentum");
            }
            // JUMP_MOM_X_W: during the push only, an extra ROLL-only
            // momentum task with this weight. Measured 2026-10-25: the roll
            // momentum at lift-off decides the remaining falls (|h_x| median
            // 0.18 survivors vs 0.51 falls; one foot then lands first).
            let in_push_now = jump_v > 0.0 && push_end.is_none() && t >= t_launch + jump_t_crouch + jump_t_hold;
            if jump_mom_x_w > 0.0 && in_push_now
                && let Some(tk) = bt::angular_momentum(dyn_ctx.qddot(), &cmm, &dcmm_v, &h_now, &[0.0; 3], &[0.0; 3], kp_mom, [true, false, false], nv)
            {
                levels.push(tk);
                names.push("momentum roll");
            }
        }
        // JUMP_FZ_SYM_W: during the push only, with both feet in contact,
        // ask for equal vertical forces on the two feet (fz_L - fz_R = 0,
        // rows 5 and 11 of the stacked (m, f) wrenches) with this weight.
        if jump_fz_sym_w > 0.0 && nc == 2 && jump_v > 0.0 && push_end.is_none() && t >= t_launch + jump_t_crouch + jump_t_hold {
            let fa = misa_wbc::AsAffine::as_affine(&dyn_ctx.forces());
            let a = (fa.m().row(5) - fa.m().row(11)).into_owned();
            let b = -(fa.q()[5] - fa.q()[11]);
            levels.push(misa_wbc::Task::equality(na::DMatrix::from_rows(&[a]), na::DVector::from_vec(vec![b])));
            names.push("fz sym");
        }
        if flight && flight_feet && let Some(off) = feet_off {
            for k in 0..2 {
                let jf = misarta::jacobian::compute_joint_jacobian_from_data(&rig.model, q, data, feet[k]);
                let djv = &misarta::jacobian::compute_joint_jacobian_time_derivative(&rig.model, q, v, feet[k]) * v_dvec;
                let pos = misarta::se3::translation(&data.oMi[feet[k]]);
                let vel3 = &jf.rows(3, 3).into_owned() * v_dvec;
                let vel = na::Vector3::new(vel3[0], vel3[1], vel3[2]);
                if flight_feet_flat {
                    // FLIGHT_FEET_FLAT=1: also hold the sole level (roll and
                    // pitch to 0, yaw as it is), so both feet meet the floor
                    // flat. Measured 2026-10-24: with position-only feet they
                    // touched down pitched 11-13 deg and rolled 6-8 deg.
                    let rot = misarta::se3::rotation_matrix(&data.oMi[feet[k]]);
                    let yaw = rot[(1, 0)].atan2(rot[(0, 0)]);
                    let rot_tgt = *na::Rotation3::from_axis_angle(&na::Vector3::z_axis(), yaw).matrix();
                    let w3 = &jf.rows(0, 3).into_owned() * v_dvec;
                    let omega = na::Vector3::new(w3[0], w3[1], w3[2]);
                    levels.push(bt::swing_with_pose(
                        dyn_ctx.qddot(), &jf, &djv, &pos, &vel, &(com + off[k]), &com_vel,
                        kp_ff, kp_ff, kd_ff, bt::SwingAxes::XyzRollPitch,
                        Some((rot, rot_tgt, omega, kp_ffr, kd_ffr)),
                    ));
                } else {
                    levels.push(bt::swing(
                        dyn_ctx.qddot(), &jf, &djv, &pos, &vel, &(com + off[k]), &com_vel,
                        kp_ff, kp_ff, kd_ff, bt::SwingAxes::Xyz,
                    ));
                }
                names.push("flight feet");
            }
        }
        if foot_reseat && !flight && touchdown_t.is_some() && flight_t0.is_none() {
            for k in 0..2 {
                if in_contact[k] || step_st.is_some_and(|x| x.0 == k) {
                    continue;
                }
                let Some(xy) = reseat_xy[k] else { continue };
                let jf = misarta::jacobian::compute_joint_jacobian_from_data(&rig.model, q, data, feet[k]);
                let djv = &misarta::jacobian::compute_joint_jacobian_time_derivative(&rig.model, q, v, feet[k]) * v_dvec;
                let pos = misarta::se3::translation(&data.oMi[feet[k]]);
                let vel3 = &jf.rows(3, 3).into_owned() * v_dvec;
                let vel = na::Vector3::new(vel3[0], vel3[1], vel3[2]);
                let rot = misarta::se3::rotation_matrix(&data.oMi[feet[k]]);
                let yaw = rot[(1, 0)].atan2(rot[(0, 0)]);
                let rot_tgt = *na::Rotation3::from_axis_angle(&na::Vector3::z_axis(), yaw).matrix();
                let w3 = &jf.rows(0, 3).into_owned() * v_dvec;
                let omega = na::Vector3::new(w3[0], w3[1], w3[2]);
                let tgt = na::Vector3::new(xy.x, xy.y, fz_stand[k] - 0.005);
                levels.push(bt::swing_with_pose(
                    dyn_ctx.qddot(), &jf, &djv, &pos, &vel, &tgt, &na::Vector3::zeros(),
                    kp_ff, kp_ff, kd_ff, bt::SwingAxes::XyzRollPitch,
                    Some((rot, rot_tgt, omega, kp_ffr, kd_ffr)),
                ));
                names.push("reseat");
            }
        }
        if let Some((sw, t0s, p0, p1, _)) = step_st {
            let s = ((t - t0s) / land_step_t).clamp(0.0, 1.0);
            let b = s * s * s * (10.0 - 15.0 * s + 6.0 * s * s);
            let db = if (t - t0s) / land_step_t < 1.0 { 30.0 * s * s * (1.0 - s) * (1.0 - s) / land_step_t } else { 0.0 };
            let lift = if s < 1.0 { land_step_h * (std::f64::consts::PI * s).sin() } else { -0.005 };
            let dlift = if s < 1.0 { land_step_h * std::f64::consts::PI * (std::f64::consts::PI * s).cos() / land_step_t } else { 0.0 };
            let tgt = p0 + (p1 - p0) * b + na::Vector3::new(0.0, 0.0, lift);
            let tgt_v = (p1 - p0) * db + na::Vector3::new(0.0, 0.0, dlift);
            let jf = misarta::jacobian::compute_joint_jacobian_from_data(&rig.model, q, data, feet[sw]);
            let djv = &misarta::jacobian::compute_joint_jacobian_time_derivative(&rig.model, q, v, feet[sw]) * v_dvec;
            let pos = misarta::se3::translation(&data.oMi[feet[sw]]);
            let vel3 = &jf.rows(3, 3).into_owned() * v_dvec;
            let vel = na::Vector3::new(vel3[0], vel3[1], vel3[2]);
            levels.push(bt::swing(dyn_ctx.qddot(), &jf, &djv, &pos, &vel, &tgt, &tgt_v, kp_ls, kp_ls, kd_ls, bt::SwingAxes::Xyz));
            names.push("step swing");
        }
        let tau_gravity = rig.gravity_torque(q);
        let shares = vec![1.0; nc];
        let forces_nominal = bt::force_nominal(dyn_ctx.forces().size(), &shares, weight);
        // In flight there is no weight to carry: regularise torque to zero.
        let tau_reg_target = if flight { na::DVector::zeros(tau_gravity.len()) } else { tau_gravity.clone() };
        levels.push(bt::regulariser(dyn_ctx.tau(), &tau_reg_target, &dyn_ctx.forces(), &forces_nominal));
        names.push("regularise");

        // WBCW_CONTACT_REST: the contact weight outside the push and release
        // and outside WBCW_CONTACT_REST_AFTER s from the first touchdown
        // (default = WBCW_CONTACT). Measured 2026-10-25: with
        // WBCW_CONTACT=1 the double-support stand rocks from one sole edge to
        // the other (CoP y +-25 mm every ~40 ms, ankle roll +-0.08 rad) and
        // after landing one foot unloads and the body topples; 0.1 stands
        // still.
        if rest_settle_v > 0.0 && !rest_settled && !flight
            && touchdown_t.is_some_and(|t0| t - t0 >= wbcw_contact_rest_after)
            && com_vel.norm() < rest_settle_v && roll.abs().max(pitch.abs()) < rest_settle_tilt
        {
            rest_settled = true;
            println!("  [settled] t={t:.3} after {:.3} s: contact weight -> rest", t - touchdown_t.unwrap_or(t));
        }
        let landed_busy = if rest_settle_v > 0.0 {
            touchdown_t.is_some() && !rest_settled
        } else {
            touchdown_t.is_some_and(|t0| t - t0 < wbcw_contact_rest_after)
        };
        let jump_from = if rest_launch { t_launch } else { t_launch + jump_t_crouch + jump_t_hold };
        let in_jump = (jump_v > 0.0 && push_end.is_none() && t >= jump_from) || landed_busy;
        let contact_rest = wbcw_contact_rest.is_some() && !in_jump;
        if weighted {
            let nd = levels[0].n_decision();
            for (lvl, name) in levels.iter_mut().zip(&names).skip(1) {
                // WBCW_TRUNK_PUSH: the trunk weight during the push and the
                // release only (default = WBCW_TRUNK).
                let in_push = jump_v > 0.0 && !flight && push_end.is_none() && t >= t_launch + jump_t_crouch + jump_t_hold;
                let w = match *name {
                    "com" => env_f64("WBCW_COM", 1.0),
                    "flight feet" => flight_feet_w,
                    "step swing" => land_step_w,
                    "reseat" => foot_reseat_w,
                    "momentum roll" => jump_mom_x_w,
                    "fz sym" => jump_fz_sym_w,
                    "momentum" => mom_w,
                    "contact+cones" if contact_rest => wbcw_contact_rest.unwrap_or(wbcw_contact),
                    "contact+cones" => wbcw_contact,
                    "trunk" if in_push => env_f64("WBCW_TRUNK_PUSH", env_f64("WBCW_TRUNK", 0.3)),
                    // WBCW_TRUNK_LAND: the trunk weight from the first
                    // touchdown on (default = WBCW_TRUNK). Measured
                    // 2026-10-28 (DIAG_LAND): JUMP_V 2.0 lands pitched
                    // 0.13 rad; righting the trunk at 0.3 overrules the CoM
                    // task (commanded -16 m/s^2 back, solved -0.1), the CoM
                    // runs 7 cm forward of its reference to the toe edge.
                    // WBCW_TRUNK_LAND_VZ (m/s, 0 = always): only after a
                    // landing faster than this. Measured 2026-10-28: 0.1
                    // after every landing drops JUMP_V 1.0 38 -> 27.
                    "trunk" if touchdown_t.is_some() && (trunk_land_vz <= 0.0 || td_vz < -trunk_land_vz) => {
                        env_f64("WBCW_TRUNK_LAND", env_f64("WBCW_TRUNK", 0.3))
                    }
                    "trunk" => env_f64("WBCW_TRUNK", 0.3),
                    "posture" => env_f64("WBCW_POSTURE", 0.01),
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
        if diag_land && tick % 10 == 0 && let Some(t0) = touchdown_t && t - t0 <= 3.0 {
            let a_sol = &st.j_com * &ex.qddot + st.djv_com;
            let (cr, ac) = diag_com_cmd.unwrap_or((com, na::Vector3::zeros()));
            let feet_x: Vec<f64> = (0..2).map(|k| misarta::se3::translation(&data.oMi[feet[k]]).x + prof.sole_centre_x).collect();
            println!(
                "  [land] t+{:.2} nc={nc} com x {:+.3} ref {:+.3} vx {:+.3} | ax cmd {:+.2} sol {:+.2} | pitch {:+.3} rate {:+.3} | sole x {:+.3} {:+.3} cop x {:+.3} {:+.3} fz {:.0} {:.0}",
                t - t0, com.x, cr.x, com_vel.x, ac.x, a_sol[0], pitch, st.v_ang_w[1],
                feet_x[0], feet_x[1], measured.cop[0][0], measured.cop[1][0], measured.cop[0][2], measured.cop[1][2]
            );
        }

        if verbose && jump_v > 0.0 && !flight && t >= t_launch && push_end.is_none() && tick % 5 == 0 {
            let mut use_j: Vec<(f64, String)> = rig.actuated().iter().map(|&(ji, vi)| {
                let tm = rig.torque_max[vi - 6];
                (if tm > 0.0 { ex.tau[vi - 6].abs() / tm } else { 0.0 }, format!("{}={:+.0}/{:.0}", rig.robot.joints[ji].name.replace("_joint", ""), ex.tau[vi - 6], tm))
            }).collect();
            use_j.sort_by(|a, b| b.0.partial_cmp(&a.0).unwrap());
            println!("  [tau] t={t:.3} {}", use_j.iter().take(4).map(|x| format!("{} ({:.2})", x.1, x.0)).collect::<Vec<_>>().join("  "));
        }
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
        if log.is_enabled() {
            let foot_z = rig.sim.body_world_position(prof.foot_links[0]).unwrap()[2];
            let swing_z = rig.sim.body_world_position(prof.foot_links[1]).unwrap()[2];
            let row = Row {
                t,
                com,
                com_ref_z: c_ref.z,
                com_ref_y: c_ref.y,
                tilt: roll.abs().max(pitch.abs()),
                trunk_tilt: roll.abs().max(pitch.abs()),
                n_stance: nc,
                foot_z,
                swing_z,
                foot_vz: 0.0,
                acc_dbg: [0.0; 2],
                a_com: na::Vector3::zeros(),
                rp_ref: [0.0; 2],
                degraded: !matches!(sol.status, misa_wbc::SolveStatus::Optimal),
                cop_box: (sole_half_l, sole_half_w),
                cop_qp: [[0.0; 3]; 2],
                f_qp_w: [[0.0; 3]; 2],
                measured: &measured,
                taus: &robot_taus,
                extra: &[if flight { 1.0 } else { 0.0 }, com_vel.z],
            };
            log.write(&rig, &row);
        }
        write_to_plant(&mut rig, ctrl_mode, &robot_taus, &ex.qddot, v, dt);
        rig.step(mj_substeps);

        let tilt = roll.abs().max(pitch.abs());
        if touchdown_t.is_some() {
            max_tilt_after = max_tilt_after.max(tilt);
            min_com_z_after = min_com_z_after.min(com.z);
            if touchdown_t.is_some_and(|t0| t - t0 < 1.0) {
                peak_grf_after = peak_grf_after.max(measured.cop[0][2] + measured.cop[1][2]);
                for (i, tmax) in rig.torque_max.iter().enumerate() {
                    if *tmax > 0.0 {
                        peak_tau_after = peak_tau_after.max(ex.tau[i].abs() / tmax);
                    }
                }
            }
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
    println!("  max tilt after touchdown: {max_tilt_after:.3} rad, min CoM z after touchdown {min_com_z_after:.3} (stand {:.3})", stand_z.unwrap_or(0.0));
    println!("  first touchdown vz {td_vz:+.3} m/s, peak GRF in the first 1 s {:.2} x weight, peak torque-box use {peak_tau_after:.2}", peak_grf_after / weight);
    println!("  degraded solves (all): {}", tally.n);
    tally.report();
    println!("  landing steps: {steps_done}");
    println!("  verdict: {}", if fell { "FELL" } else if flight_spans.is_empty() { "NO FLIGHT" } else { "LANDED" });
}

#[cfg(not(feature = "mujoco"))]
fn main() {
    eprintln!("This example requires the `mujoco` feature.");
}
