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
    // JUMP_BOX="x,h[,len]": a box welded to the world, its front face at
    // world x (m), h m high, len m long (default 2) and 2 m wide, to jump
    // onto. Named stair_* so a foot on it counts as a stance contact.
    // Unset = no box (bit-identical).
    let jump_box: Option<(f64, f64, f64)> = std::env::var("JUMP_BOX").ok().map(|s| {
        let p: Vec<f64> = s.split(',').map(|x| x.trim().parse().unwrap()).collect();
        (p[0], p[1], p.get(2).copied().unwrap_or(2.0))
    });
    if let Some((bx, bh, bl)) = jump_box {
        o.extra_worldbody = Some(format!(
            "\n    <body name=\"stair_jump_box\" pos=\"{:.4} 0 {:.4}\">\
             \n      <geom name=\"stair_jump_box_geom\" type=\"box\" size=\"{:.4} 1.0 {:.4}\" rgba=\"0.55 0.57 0.62 1\"/>\n    </body>",
            bx + 0.5 * bl, 0.5 * bh, 0.5 * bl, 0.5 * bh
        ));
        println!("jump box: front face x {bx:+.3}, height {bh:.3}, length {bl:.3}");
        // a box under the start (front face behind the feet): spawn on it,
        // 1 cm up (INIT_JITTER otherwise puts a sole into it)
        if bx < -0.2 {
            o.spawn_z_offset = bh + 0.01;
        }
    }
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
    let mut t_launch = env_f64("T_LAUNCH", 2.0);
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
    let mut crouch = env_f64("CROUCH", 0.12);
    let jump_t_crouch = env_f64("JUMP_T_CROUCH", 0.8).max(0.05);
    let jump_t_hold = env_f64("JUMP_T_HOLD", 0.3).max(0.0);
    let jump_z_to = env_f64("JUMP_Z_TO", 0.02);
    // JUMP_VX (m/s, 0 = off): a forward jump. The push runs along a line
    // tilted forward -- the CoM's x acceleration is JUMP_VX / JUMP_V times
    // the vertical one, so it leaves with (JUMP_VX, JUMP_V) -- and the
    // release keeps x ballistic. JUMP_LEAN (m, 0 = off): shift the CoM by
    // this much in x during the crouch (quintic over JUMP_T_CROUCH).
    // JUMP_VX_TAB / JUMP_LEAN_TAB ("V:value,V:value,..."): pick JUMP_VX /
    // JUMP_LEAN from JUMP_V by piecewise-linear interpolation (clamped at
    // the ends), so one setting covers every jump height. Measured
    // 2026-11-03: one fixed (VX, LEAN) gives the most lift-off vx to the
    // shortest flight (VX/V tilts the push line), which cannot be caught.
    let tab = |k: &str| -> Option<Vec<(f64, f64)>> {
        let s = std::env::var(k).ok()?;
        let mut t: Vec<(f64, f64)> = s
            .split(',')
            .map(|p| {
                let (a, b) = p.split_once(':').unwrap_or_else(|| panic!("{k}: expected V:value, got {p:?}"));
                (a.trim().parse().unwrap(), b.trim().parse().unwrap())
            })
            .collect();
        t.sort_by(|a, b| a.0.total_cmp(&b.0));
        Some(t)
    };
    let interp = |t: &[(f64, f64)], v: f64| -> f64 {
        if v <= t[0].0 {
            return t[0].1;
        }
        for w in t.windows(2) {
            if v <= w[1].0 {
                return w[0].1 + (w[1].1 - w[0].1) * (v - w[0].0) / (w[1].0 - w[0].0);
            }
        }
        t[t.len() - 1].1
    };
    let jump_vx = tab("JUMP_VX_TAB").map_or_else(|| env_f64("JUMP_VX", 0.0), |t| interp(&t, jump_v));
    let jump_lean = tab("JUMP_LEAN_TAB").map_or_else(|| env_f64("JUMP_LEAN", 0.0), |t| interp(&t, jump_v));
    // CROUCH_TAB ("V:depth,..."): CROUCH from JUMP_V the same way. Measured
    // 2026-11-07: at JUMP_V 1.0 the 0.18 m crouch makes a 0.31 s push; on
    // two feet the pelvis pitches back through it (-0.45 vs -0.26 rad) and
    // the repeated jumps fall; 0.12 m (push ~0.2 s) takes 99 -> 142 of 144.
    if let Some(t) = tab("CROUCH_TAB") {
        crouch = interp(&t, jump_v);
        println!("jump tables: JUMP_V {jump_v} -> CROUCH {crouch:.4}");
    }
    if std::env::var("JUMP_VX_TAB").is_ok() || std::env::var("JUMP_LEAN_TAB").is_ok() {
        println!("jump tables: JUMP_V {jump_v} -> JUMP_VX {jump_vx:.3} JUMP_LEAN {jump_lean:.4}");
    }
    // JUMP_FWD_TARGET (m, unset = off): a forward jump that aims the soles'
    // midpoint this far ahead of where it stood at T_LAUNCH. JUMP_VX and
    // JUMP_LEAN are scaled by one factor s, read backwards off JUMP_FWD_TAB
    // ("V|s:dx,s:dx,...;V|..."; the soles' move one jump makes against s,
    // per JUMP_V; piecewise linear, interpolated in JUMP_V, clamped).
    // With JUMP_REPEAT, the jumps go on until, at a re-arm, the soles are
    // within JUMP_FWD_TOL m (default 0.05) of the target; each one's s is
    // read off the table for the distance still to go.
    // DIAG_FWD_FEET=1 prints the soles' move without a target. Measured
    // 2026-11-08 (TB6 tables times s, final CoM dx): 1.5 -0.10 / +0.07 /
    // +0.32 m and 2.0 -0.13 / +0.10 / +0.35 m at s 0 / 0.5 / 1.
    let jump_fwd_target = std::env::var("JUMP_FWD_TARGET").ok().map(|s| s.trim().parse::<f64>().unwrap());
    let jump_fwd_tab = vtab("JUMP_FWD_TAB");
    let fwd_s_for = |dx: f64| -> f64 {
        let tab = jump_fwd_tab.as_ref().expect("JUMP_FWD_TARGET needs JUMP_FWD_TAB");
        let ss: Vec<(f64, f64)> = tab.iter().map(|(v, pts)| (*v, inv_pw(pts, dx))).collect();
        interp(&ss, jump_v)
    };
    let mut fwd_s = jump_fwd_target.map_or(1.0, fwd_s_for);
    let jump_fwd_tol = env_f64("JUMP_FWD_TOL", 0.05);
    let mut fwd_reached = false;
    // JUMP_BOX_GAP=g (m, unset = off; needs JUMP_BOX, JUMP_FWD_TAB and
    // JUMP_REPEAT): hop on the flat (scale read off JUMP_FWD_TAB) until the
    // gap from the front toe (sole centre + SOLE_HALF_L) to the box's front
    // face is within JUMP_BOX_GAP_TOL (default 0.03) of g, then jump onto the
    // box at scale JUMP_BOX_S (default 1); no more jumps once a landing is on
    // the box. Measured 2026-11-08 (TB6 single jump, tuck 0.05, 5 cm box):
    // gap 0.03 clips the toe on the way up (1.5 8/16), 0.08 stands 14/16 at
    // both 1.5 and 2.0, 0.13 lands the heel short (1.5 0/8).
    let jump_box_gap = std::env::var("JUMP_BOX_GAP").ok().map(|s| s.trim().parse::<f64>().unwrap());
    let jump_box_gap_tol = env_f64("JUMP_BOX_GAP_TOL", 0.03);
    let jump_box_s = env_f64("JUMP_BOX_S", 1.0);
    // JUMP_BOX_TUCK=d (m, 0 = off): FLIGHT_FEET_TUCK added on the jump onto
    // the box only (JUMP_BOX_GAP). Measured 2026-11-08: with the tuck on
    // every jump, the short backward hops that close the gap fell (5 cm box
    // 0.20 m ahead, 2.0: 15/16 fell on the first hop).
    let jump_box_tuck = env_f64("JUMP_BOX_TUCK", 0.0);
    // JUMP_BOX_STIFF=1: the jump onto the box crouches on the stiff contact
    // like a first jump, even with JUMP_REPEAT_SOFT_CROUCH. Measured
    // 2026-11-08 (5 cm box after flat hops, 2.0): on the soft contact the
    // CoM starts the push +0.018-0.021 m ahead of the soles' midpoint for a
    // reference of +0.061 (first jump: +0.040 for +0.050), lift-off vx
    // 0.44-0.55 against 0.62, and the heel lands short of the box edge.
    let jump_box_stiff = flag("JUMP_BOX_STIFF", false);
    // JUMP_BOX_CROUCH_W (unset = off): the contact level's weight through the
    // crouch and hold of the jump onto the box after a re-arm, in place of
    // WBCW_CONTACT_REST (JUMP_REPEAT_SOFT_CROUCH). The level holds the CoP
    // box too. Measured 2026-11-08 (DIAG_CROUCH, 2.0, 0.20 m box): at 0.1
    // the CoM follows the lean to +0.047 m ahead of the sole centres at
    // 0.13 m/s, the plan puts the CoP past the toe edge to brake, the feet
    // tip and the CoM rocks back to +0.018 at the push.
    let jump_box_crouch_w = std::env::var("JUMP_BOX_CROUCH_W").ok().map(|s| s.trim().parse::<f64>().unwrap());
    let mut box_jump_now = false;
    // (toe to box, scale, onto the box)
    let box_gap_s = move |toe_x: f64| -> (f64, f64, bool) {
        let (bx, g) = (jump_box.expect("JUMP_BOX_GAP needs JUMP_BOX").0, jump_box_gap.unwrap());
        let gap = bx - toe_x;
        if (gap - g).abs() <= jump_box_gap_tol { (gap, jump_box_s, true) } else { (gap, fwd_s_for(gap - g), false) }
    };
    let sole_cx = prof.sole_centre_x;
    let front_toe_x = move |o_mi: &[misarta::se3::SE3<f64>]| -> f64 {
        (0..2)
            .map(|k| misarta::se3::translation(&o_mi[feet[k]])[0] + misarta::se3::rotation_matrix(&o_mi[feet[k]])[(0, 0)] * sole_cx)
            .fold(f64::NEG_INFINITY, f64::max)
            + sole_half_l
    };
    let diag_fwd_feet = jump_fwd_target.is_some() || jump_box_gap.is_some() || flag("DIAG_FWD_FEET", false);
    if let Some(dx) = jump_fwd_target {
        println!("jump forward target {dx:+.3} m: scale {fwd_s:.4} -> JUMP_VX {:.3} JUMP_LEAN {:.4}", jump_vx * fwd_s, jump_lean * fwd_s);
    }
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
    // AIR_ABORT_T (s, 0 = off): after the release has scheduled flight, if
    // no tick has had both feet unloaded within this long and a foot is
    // loaded, it was not a flight: return to stance and absorb (JUMP_ABORT
    // for the RELEASE_FIRST_OFF path). Measured 2026-11-02: two JUMP_V 1.0
    // members released on one foot, reloaded both 4 ms later and collapsed
    // under the flight tasks; release -> both feet off is <= 42 ms in every
    // member that flew.
    let air_abort_t = env_f64("AIR_ABORT_T", 0.0).max(0.0);
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
    // DIAG_CROUCH=1: the [land] line every 10 ticks through each jump's
    // crouch and hold too (label [crouch], time from T_LAUNCH), with the
    // contact weight's state.
    let diag_crouch = flag("DIAG_CROUCH", false);
    let trunk_land_vz = env_f64("WBCW_TRUNK_LAND_VZ", 0.0);
    // WBCW_TRUNK_LAND_T (s, default: for good): WBCW_TRUNK_LAND only this
    // long after the touchdown, then WBCW_TRUNK again. Measured 2026-11-03
    // (JUMP_REPEAT): the robot waits for the next jump pitched 0.15 rad
    // forward under 0.03, and the step back to 0.3 at the re-arm rights it
    // by running the CoM backward off the heels.
    let trunk_land_t = env_f64("WBCW_TRUNK_LAND_T", f64::INFINITY);
    let kp_trunk_land = env_f64("KP_TRUNK_LAND", kp_trunk);
    let kd_trunk_land = env_f64("KD_TRUNK_LAND", kd_trunk);
    let mut diag_com_cmd: Option<(na::Vector3<f64>, na::Vector3<f64>)> = None; // (ref, a_cmd)
    // DIAG_PUSHX=1: from the push start to 30 ms after the push ends, every
    // 10 ms: the CoM x reference (position, velocity, acceleration) against
    // the CoM, and the x acceleration at four stages -- the CoM task's
    // command, the QP's solved CoM acceleration (J qdd + dJ v), the QP's own
    // contact forces (sum f_x / m), and the plant's measured forces -- plus
    // the QP's planned contact acceleration (the soft contact's leak), trunk
    // pitch and rate, and CoP x per foot. Where the take-off vx goes missing.
    let diag_pushx = flag("DIAG_PUSHX", false);
    let mut diag_com_vref: Option<(na::Vector3<f64>, na::Vector3<f64>)> = None; // (vel ref, acc ref)
    // JUMP_MPC=1: plan the push's sagittal motion instead of scripting it.
    // From JUMP_MPC_FROM s after T_LAUNCH to the end of the push, every
    // JUMP_MPC_EVERY ticks, from the measured CoM x, vx and h_y: the CoM x
    // acceleration and the net CoP x per JUMP_MPC_DT node (`plan_push`),
    // with the vertical plan (crouch, hold, push) kept as scripted, so the
    // vertical force is known and the problem is a convex QP. Goal: lift-off
    // vx = JUMP_MPC_VX (default JUMP_VX) and h_y = 0, the CoP inside the
    // soles less JUMP_MPC_COP_MARGIN, the CoM within JUMP_MPC_X_FWD ahead of
    // / JUMP_MPC_X_BACK behind the sole centres. The plan replaces the CoM x
    // reference (coasting through the release) and the momentum task's
    // pitch reference (h_y and its rate, fed forward). Measured 2026-11-02:
    // the scripted push pins the CoP on the heel edge and the lift-off vx
    // tops out at 0.46 m/s for a 1.2 command. Off = bit-identical.
    let jump_mpc = flag("JUMP_MPC", false);
    let jump_mpc_from = env_f64("JUMP_MPC_FROM", 0.0).max(0.0);
    let jump_mpc_dt = env_f64("JUMP_MPC_DT", 0.02).max(0.004);
    let jump_mpc_every = (env_f64("JUMP_MPC_EVERY", 5.0) as usize).max(1);
    let jump_mpc_meas = flag("JUMP_MPC_MEAS", false);
    let diag_mpc = flag("DIAG_MPC", false);
    let mut push_cfg = PushPlanCfg {
        vx_target: env_f64("JUMP_MPC_VX", jump_vx * fwd_s),
        w_v: env_f64("JUMP_MPC_WV", 1e3),
        w_h: env_f64("JUMP_MPC_WH", 1e2),
        w_h_run: env_f64("JUMP_MPC_WH_RUN", 1.0),
        w_a: env_f64("JUMP_MPC_WA", 0.01),
        w_da: env_f64("JUMP_MPC_WDA", 0.1),
        w_dp: env_f64("JUMP_MPC_WDP", 1e2),
        mu: env_f64("JUMP_MPC_MU", 0.8 * friction_mu),
        cop_lo: 0.0,
        cop_hi: 0.0,
        x_lo: 0.0,
        x_hi: 0.0,
    };
    let jump_mpc_vx_set = std::env::var("JUMP_MPC_VX").is_ok();
    let jump_mpc_cop_margin = env_f64("JUMP_MPC_COP_MARGIN", 0.02);
    let jump_mpc_x_fwd = env_f64("JUMP_MPC_X_FWD", 0.12);
    let jump_mpc_x_back = env_f64("JUMP_MPC_X_BACK", 0.06);
    let mut push_plan: Option<PushPlan> = None;
    // JUMP_H_NAT=1: from T_LAUNCH to the push, the momentum task tracks the
    // pitch momentum the planned motion itself carries instead of zero --
    // the h of the joint velocities that move the CoM at its reference
    // velocity with both feet still and the trunk's orientation held (base,
    // legs and waist; min-norm, the arms still). Measured 2026-11-03: folding
    // the legs with the trunk upright carries h_y ~ 2.8 vz; asking for 0
    // instead needs a ground moment that, with the contact stiff, dragged the
    // CoM 5 cm forward and kept the crouch 9 cm shallow. JUMP_H_NAT_PUSH=1:
    // also in the push, scaled from 1 at its start to 0 at its end (=0: the
    // push keeps 0, as before). Off = bit-identical.
    let jump_h_nat = flag("JUMP_H_NAT", false);
    let jump_h_nat_push = flag("JUMP_H_NAT_PUSH", false);
    // JUMP_H_NAT_XY=1: track the natural momentum's x component too (else
    // h_x is held at 0). Measured 2026-11-07 (repeated turning jumps): facing
    // 0.7-1.2 rad, the crouch's natural momentum about the body's lateral
    // axis is partly world x; holding that at 0 rolls the body (lift-off
    // body-frame h_x -1.4 vs -0.7 on the first jump) and a foot leaves early.
    let jump_h_nat_xy = flag("JUMP_H_NAT_XY", false);
    let mut h_nat_prev: Option<f64> = None;
    let mut h_nat_prev_x: Option<f64> = None;
    let diag_hnat = flag("DIAG_HNAT", false);
    // DIAG_PUSHF=1: from 20 ms before the push to 10 ms after it ends, every
    // DIAG_PUSHF_EVERY ticks, per foot (L | R): in the contact set, the QP's
    // vertical force against the measured one, the measured CoP (x, y), the
    // QP's planned contact acceleration z, the sole's height, roll and
    // pitch, and the knee / ankle pitch / ankle roll torque and angle
    // (with the angle's distance to its nearer limit); plus h_x and the
    // CoM's y and vy. Measured 2026-11-03: every forward push (VX 1.2) ends
    // on one foot, the vertical push rarely; why one foot goes.
    let diag_pushf = flag("DIAG_PUSHF", false);
    let diag_pushf_every = (env_f64("DIAG_PUSHF_EVERY", 1.0) as usize).max(1);
    let mut mpc_fails = 0usize;
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
    // PUSH_LATCH=1: during the push (until the scripted push ends) a foot
    // stays in the contact set when its load drops. Measured 2026-11-06: at
    // JUMP_V 1.0 one foot dropped out 0.10-0.16 s into the 0.31 s push and
    // the failing repeat jumps chattered 2-1-2-1 between the feet, pushing
    // off one foot then the other (lift-off |h_x| ~1).
    let push_latch = flag("PUSH_LATCH", false);
    let mut push_phase = false;
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
    // LAND_REF_FOLLOW=1: the landing reference's horizontal part moves with
    // the feet -- shifted by how far the two sole centres' midpoint has
    // moved since the landing began. Measured 2026-11-02 (DIAG_LAND, the
    // JUMP_V 2.0 true falls): the trunk pitches forward to 0.54 rad, the CoP
    // sits on the toe edge, the feet slide 7 cm forward 1.0-1.4 s after
    // touchdown, and the reference left in the world is then on the heel
    // edge: the CoM task pulls the CoM back over it and the robot falls
    // backward ~2.5 s after landing. Off = bit-identical.
    // LAND_REF_FOLLOW=2: follow only the feet in contact -- the reference
    // moves by the mean displacement, tick to tick, of the loaded soles
    // (none loaded: no move). Measured 2026-11-02: with =1 the JUMP_V 1.0
    // members that land on one foot in a row of short hops lost 6/96: the
    // unloaded foot wandered 7-8 cm back on the floor and took the
    // reference with it.
    let land_ref_follow = env_f64("LAND_REF_FOLLOW", 0.0) as u32;
    // LAND_GROUND_Z=1 (default on with JUMP_BOX): at each touchdown the
    // stand height and the standing foot heights move with the surface the
    // robot lands on -- the loaded feet's height then minus at the start;
    // at each re-arm, both feet's. Off = bit-identical.
    let land_ground_z = flag("LAND_GROUND_Z", jump_box.is_some());
    // JUMP_BOX_CLEAR=c (m, needs JUMP_BOX; unset = off): in flight, a foot
    // whose heel (sole centre - SOLE_HALF_L - JUMP_BOX_CLEAR_X) is not yet
    // past the box's front face is held with its sole at least c above the
    // box top; past it, the foot hangs from the CoM as on flat ground.
    // Measured 2026-11-08: hanging feet clip the box edge on the way up
    // (5 cm box 0.20-0.25 m ahead, 1.5: 1/8 and 6/8 standing); a constant
    // FLIGHT_FEET_TUCK clears it but lands with the legs 5 cm more bent and
    // one member then drifts backward off the box.
    let jump_box_clear = std::env::var("JUMP_BOX_CLEAR").ok().map(|s| s.trim().parse::<f64>().unwrap());
    let jump_box_clear_x = env_f64("JUMP_BOX_CLEAR_X", 0.03);
    let mut ground_dz = 0.0_f64;
    let mut land_sole0: Option<(f64, na::Vector2<f64>)> = None;
    // (landing start, each sole's last position, accumulated shift)
    let mut land_follow: Option<(f64, [na::Vector2<f64>; 2], na::Vector2<f64>)> = None;
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
    // FLIGHT_FEET_YAW=1 (with FLIGHT_FEET_FLAT): hold each sole's yaw at its
    // lift-off yaw instead of letting it turn. Measured 2026-11-03: the jump
    // leaves with h_z ~0.1-0.2 and nothing holds yaw in flight; the light
    // legs take it and the feet land turned 0.3-0.7 rad, so the next jump
    // pushes sideways. Off = bit-identical.
    let flight_feet_yaw = flag("FLIGHT_FEET_YAW", false);
    let mut feet_yaw0: [f64; 2] = [0.0; 2];
    // FLIGHT_TRUNK_YAW=w (0 = off): in flight, a yaw-only pelvis task (weight
    // w, gains KP/KD_TRUNK) holding the pelvis' lift-off yaw. Measured
    // 2026-11-03: with the feet's yaw held the trunk took the twist (-0.37
    // rad in flight) and, back on the ground, turning it back twisted a
    // foot 0.4 rad on the floor.
    let flight_trunk_yaw = env_f64("FLIGHT_TRUNK_YAW", 0.0).max(0.0);
    let mut trunk_yaw0 = 0.0_f64;
    // STANCE_TRUNK_YAW=w (0 = off): on the ground from the first touchdown
    // on, a yaw-only pelvis task (weight w) toward the mean yaw of the two
    // soles. Measured 2026-11-03 (JUMP_REPEAT): after a landing the pelvis
    // and the feet differ by 0.2-0.3 rad of yaw; the posture task's hip-yaw
    // pull then twists the feet on the floor in the next crouch, a foot
    // unloads and the CoM runs backward.
    let stance_trunk_yaw = env_f64("STANCE_TRUNK_YAW", 0.0).max(0.0);
    let mut ever_landed = false;
    // FLIGHT_FEET_CAPTURE=k (0 = off): in flight, carry both sole centres
    // from their lift-off offset to k * vx / omega ahead of the CoM (the
    // stopping point of a LIPM at the stand height), smoothstep over
    // FLIGHT_FEET_CAPTURE_T s from lift-off. For a forward jump the feet
    // leave behind the CoM; landing there pitches the body forward.
    let flight_feet_capture = env_f64("FLIGHT_FEET_CAPTURE", 0.0);
    let flight_feet_capture_t = env_f64("FLIGHT_FEET_CAPTURE_T", 0.12).max(1e-3);
    // FLIGHT_FEET_WIDTH=w (m, 0 = off): in flight, also carry the soles'
    // lateral offsets from the CoM to +w/2 (left) and -w/2 (right),
    // smoothstep over FLIGHT_FEET_CAPTURE_T s from lift-off. Measured
    // 2026-11-03 (JUMP_REPEAT): the stance narrows with every jump (0.236 ->
    // 0.18-0.20 -> 0.16 m), the narrow ones fail with a large lift-off h_x.
    // With FLIGHT_FEET_TURN and JUMP_REPEAT_HEADING the offsets are across
    // the jump's heading.
    let flight_feet_width = env_f64("FLIGHT_FEET_WIDTH", 0.0).max(0.0);
    // FLIGHT_FEET_LEVEL=1: at lift-off, set both feet's height below the
    // CoM to the lower one's. =2: instead lower the higher foot to the lower
    // one over FLIGHT_FEET_CAPTURE_T s (smoothstep). Measured 2026-11-06:
    // at JUMP_V 1.0 the push ends on one foot with the other swung up
    // ~7 cm; it is still up at touchdown after the 0.17 s flight, lands
    // 60 ms later and ahead, and the next jump starts from feet staggered by
    // ~6 cm (2.0: ~1 cm).
    let flight_feet_level_n = env_f64("FLIGHT_FEET_LEVEL", 0.0);
    let flight_feet_level = flight_feet_level_n > 0.0 && flight_feet_level_n < 2.0;
    let flight_feet_level_smooth = flight_feet_level_n >= 2.0;
    // FLIGHT_FEET_TUCK=d (m, 0 = off): in flight, raise both feet d above
    // their lift-off offset from the CoM (smoothstep over FLIGHT_FEET_TUCK_T
    // s from lift-off), so the floor is met when the CoM has fallen d below
    // where the legs alone would meet it. Measured 2026-11-03: the feet hang
    // at their lift-off length and touch down near the apex (CoM 3-7 cm
    // above lift-off); a weak push (vz ~1.0) then flies 0.1 s, too short to
    // carry the feet forward, and those land behind the CoM and fall.
    let flight_feet_tuck = env_f64("FLIGHT_FEET_TUCK", 0.0);
    let flight_feet_tuck_t = env_f64("FLIGHT_FEET_TUCK_T", 0.06).max(1e-3);
    // FLIGHT_FEET_TUCK_TF=T (s, 0 = off): choose the tuck per flight at
    // lift-off so the feet meet the floor T s after lift-off: the CoM height
    // at which the held feet would touch (standing foot height minus the
    // lift-off offset) minus the ballistic CoM height at T, clamped to
    // [0, FLIGHT_FEET_TUCK_MAX]; added to FLIGHT_FEET_TUCK. Only a weak push
    // (short flight) gets a tuck: a longer flight with the lift-off pitch
    // momentum over-rotates the body forward (measured 2026-11-03).
    let flight_feet_tuck_tf = env_f64("FLIGHT_FEET_TUCK_TF", 0.0);
    let flight_feet_tuck_max = env_f64("FLIGHT_FEET_TUCK_MAX", 0.08);
    let mut tuck_adapt: Option<(f64, f64)> = None;
    // DIAG_FFEET=1: in flight, every 5 ticks, each foot's target and actual
    // offset from the CoM (x, z), and the trunk pitch.
    let diag_ffeet = flag("DIAG_FFEET", false);
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
    // JUMP_REPEAT=n (0 = off): after a jump has ended in stance (touchdown,
    // abort or air-abort, with LAND_ABSORB) and JUMP_REPEAT_AFTER s have
    // passed with both feet loaded, jump again from where the landing
    // reference holds the CoM, n more times. Each jump starts like the first:
    // the per-jump state (push, release, landing, touchdown window) is
    // cleared and T_LAUNCH moves to that tick. Off = bit-identical.
    let jump_repeat = env_f64("JUMP_REPEAT", 0.0).max(0.0) as usize;
    let jump_repeat_after = env_f64("JUMP_REPEAT_AFTER", 2.0).max(0.0);
    let mut repeats_done = 0usize;
    let mut min_share = 1.0_f64; // JUMP_REPEAT diagnostics: lighter foot's share of the load, min over 0.1 s
    // JUMP_REPEAT_RECENTRE=1: in a repeated jump's crouch, also carry the
    // CoM reference (quintic, with the crouch) from where the landing held
    // it to the soles' midpoint plus the CoM's offset from it at the first
    // jump's T_LAUNCH. Measured 2026-11-03: the landing leaves the CoM ~3 cm
    // behind the soles' midpoint (first jump: 1 cm); the second jumps that
    // succeed are the ones whose CoM was further forward.
    // =2: the target is taken from the soles' current midpoint every tick
    // (the feet move on the soft contact through the crouch; measured
    // 2026-11-03: the failing second jumps started the push with the CoM
    // 1.3 cm sideways of the soles' midpoint, the good ones 0.3 cm).
    let jump_repeat_recentre_n = env_f64("JUMP_REPEAT_RECENTRE", 0.0) as u32;
    let jump_repeat_recentre = jump_repeat_recentre_n >= 1;
    let mut stand_off: Option<na::Vector2<f64>> = None;
    let mut rearm_shift = na::Vector2::<f64>::zeros();
    // A repeated jump's crouch starts from the landing reference's height
    // at the re-arm (with LAND_RISE long, the robot stays low between jumps:
    // measured 2026-11-03, rising back to the stand height on the displaced
    // feet rocks the stand from sole edge to sole edge).
    let mut crouch_z0: Option<f64> = None;
    // JUMP_REPEAT_SOFT_CROUCH=1: in a repeated jump, the contact weight
    // stays at WBCW_CONTACT_REST through the crouch and hold (as without
    // WBCW_CONTACT_REST_LAUNCH); the first jump is unchanged. Measured
    // 2026-11-03: on the displaced feet after a landing, the stiff contact
    // rocks the load from foot to foot through the crouch (lighter foot's
    // share 0.00-0.10; the first jump's crouch 0.50).
    let jump_repeat_soft_crouch = flag("JUMP_REPEAT_SOFT_CROUCH", false);
    // JUMP_REPEAT_HEADING=1: a repeated jump goes along the soles' mean yaw
    // at the re-arm, not world x: the lean, the push's forward part, the
    // recentring offset (the first jump's, turned by that yaw) and the
    // flight feet's capture are laid out in that heading. Measured
    // 2026-11-03: the feet stand turned 0.1-0.3 rad after a landing, and a
    // world-x push left the second jump with |h_x| 1.2-1.5 (first: ~0.05).
    let jump_repeat_heading = flag("JUMP_REPEAT_HEADING", false);
    let mut heading: Option<f64> = None;
    // JUMP_TURN_HZ (kg m^2/s, 0 = off): a turning jump. Through the push the
    // momentum task's yaw reference ramps linearly from 0 to this; it is
    // held through the release until touchdown (flight conserves it).
    // FLIGHT_FEET_TURN=1: in flight the feet's yaw targets and their offsets
    // from the CoM turn with the pelvis' yaw since lift-off, so the legs go
    // round with the body instead of holding the lift-off yaw.
    // JUMP_TURN_T0 (s, default 0): start the ramp this long before the push
    // (in the hold), so the yaw momentum is built over a longer time.
    let jump_turn_hz = env_f64("JUMP_TURN_HZ", 0.0);
    let jump_turn_t0 = env_f64("JUMP_TURN_T0", 0.0).max(0.0);
    let flight_feet_turn = flag("FLIGHT_FEET_TURN", false);
    // JUMP_TURN_LAND_T (s, 0 = off): after a turning jump's touchdown the
    // momentum task's yaw reference falls linearly from the touchdown h_z to
    // 0 over this time, instead of to 0 at once. Measured 2026-11-07
    // (repeated turning jumps): the feet land parallel (toe angle 0.02-0.05
    // rad) and then twist on the floor (0.09-0.15 rad, p90 0.3) while
    // h_z ~1.3 is stopped; the next jump starts from a narrow, staggered
    // stance (2.0: narrowest third fails 15/27, widest 5/28).
    let jump_turn_land_t = env_f64("JUMP_TURN_LAND_T", 0.0).max(0.0);
    // JUMP_TURN_FREE_YAW=1: STANCE_TRUNK_YAW is off while the turn's yaw
    // ramp runs (push start to touchdown). Measured 2026-11-07: with it on
    // through the push, the jumps after the first turn 16-28 deg (first 42).
    let jump_turn_free_yaw = flag("JUMP_TURN_FREE_YAW", false);
    // TRUNK_RP_YAWED=1: the trunk task's roll / pitch error (ZYX Euler, about
    // the yawed axes) is turned by the yaw into world x / y before it meets
    // the world angular Jacobian; bt::trunk_rp_ref applies it to world x / y
    // as is, right only near yaw 0. Measured 2026-11-07 (repeated turning
    // jumps): the push fails once the robot faces ~90 deg whatever the jump
    // (free yaw: the 3rd at 86 deg; with STANCE_TRUNK_YAW: the 4th at 93).
    let trunk_rp_yawed = flag("TRUNK_RP_YAWED", false);
    // JUMP_TURN_TARGET (deg, unset = off): turning jumps aim at this total
    // turn of the soles' mean yaw, counted from the first push's start. Each
    // jump's yaw momentum is the turn still to go over JUMP_TURN_GAIN (deg
    // per kg m^2/s; JUMP_TURN_GAIN_TAB "V:gain,..." by JUMP_V), clamped to
    // |JUMP_TURN_HZ|; at a re-arm with less than JUMP_TURN_TOL deg to go, no
    // more jumps. Measured 2026-11-07: at h_z 2 the soles turn 40-46 deg on
    // the first jump and 44-50 on the later ones (1.5 / 2.0).
    let jump_turn_target = std::env::var("JUMP_TURN_TARGET").ok().map(|s| s.trim().parse::<f64>().unwrap().to_radians());
    let jump_turn_gain = tab("JUMP_TURN_GAIN_TAB").map_or_else(|| env_f64("JUMP_TURN_GAIN", 24.0), |t| interp(&t, jump_v)).to_radians();
    let jump_turn_tol = env_f64("JUMP_TURN_TOL", 5.0).to_radians();
    // JUMP_TURN_TAB ("V|h_z:deg,h_z:deg,...;V|..."): for the jumps after the
    // first, the turn one jump makes against its h_z (in the target's
    // direction, after a jump that turned that way), per JUMP_V; the h_z is
    // read off it backwards (piecewise linear, interpolated in JUMP_V,
    // clamped), in place of JUMP_TURN_GAIN. Measured 2026-11-08: a small
    // h_z still turns the robot on 7-8 deg the way it was turning (1.5: h_z
    // +0.25 -> +12, -0.25 -> -6, 2 -> +44 deg), so the last jumps
    // overshoot and add a jump back.
    let jump_turn_tab: Option<Vec<(f64, Vec<(f64, f64)>)>> = std::env::var("JUMP_TURN_TAB").ok().map(|s| {
        let mut t: Vec<(f64, Vec<(f64, f64)>)> = s
            .split(';')
            .map(|blk| {
                let (v, pts) = blk.split_once('|').unwrap_or_else(|| panic!("JUMP_TURN_TAB: expected V|h:deg,..., got {blk:?}"));
                let mut pts: Vec<(f64, f64)> = pts
                    .split(',')
                    .map(|p| {
                        let (a, b) = p.split_once(':').unwrap_or_else(|| panic!("JUMP_TURN_TAB: expected h:deg, got {p:?}"));
                        (a.trim().parse().unwrap(), b.trim().parse::<f64>().unwrap().to_radians())
                    })
                    .collect();
                pts.sort_by(|a, b| a.0.total_cmp(&b.0));
                (v.trim().parse().unwrap(), pts)
            })
            .collect();
        t.sort_by(|a, b| a.0.total_cmp(&b.0));
        t
    });
    // h_z for a jump still to turn `rem` (rad): the first jump (or without
    // JUMP_TURN_TAB) by the gain, the later ones off the table
    let turn_hz_for = |rem: f64, tgt: f64, first: bool| -> f64 {
        let cap = jump_turn_hz.abs();
        let Some(tab) = jump_turn_tab.as_ref().filter(|_| !first) else {
            return (rem / jump_turn_gain).clamp(-cap, cap);
        };
        let s = if tgt < 0.0 { -1.0 } else { 1.0 };
        let inv = |pts: &[(f64, f64)], y: f64| -> f64 {
            let (mut lo, mut hi) = (pts[0], pts[pts.len() - 1]);
            if y <= lo.1 {
                return lo.0;
            }
            if y >= hi.1 {
                return hi.0;
            }
            for w in pts.windows(2) {
                if y <= w[1].1 {
                    (lo, hi) = (w[0], w[1]);
                    break;
                }
            }
            lo.0 + (hi.0 - lo.0) * (y - lo.1) / (hi.1 - lo.1)
        };
        let hs: Vec<(f64, f64)> = tab.iter().map(|(v, pts)| (*v, inv(pts, s * rem))).collect();
        (s * interp(&hs, jump_v)).clamp(-cap, cap)
    };
    let wrap_pi = |a: f64| (a + std::f64::consts::PI).rem_euclid(2.0 * std::f64::consts::PI) - std::f64::consts::PI;
    let mut turn_hz_now = jump_turn_hz;
    let mut turn_feet: Option<(f64, f64)> = None; // soles' mean yaw at the last count, turn so far
    let mut turn_reached = false;
    let mut turn_jumps = 0usize;
    let mut turn_last_feet = f64::NAN;
    let mut turn_td_hz = 0.0_f64;
    // pelvis yaw at the ramp start, lift-off and touchdown (first flight of
    // the last jump); the flight feet turn by the pelvis' yaw since the ramp
    // start, so the twist the push leaves between pelvis and feet comes out.
    let mut turn_yaw = [f64::NAN; 3];
    let mut turn_last_yaw = f64::NAN;
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
        // MU_TORSION (m): the QP's torsion bound |mz| <= MU_TORSION fz per
        // foot. The G1 soles meet the floor at two points (condim 3), so the
        // simulator's yaw friction is coupled with the sliding force.
        mu_torsion: env_f64("MU_TORSION", 0.05),
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
    let fall_tilt = env_f64("FALL_TILT", 0.52);
    let mut max_tilt_after = 0.0_f64;
    // forward jump summary: CoM x at T_LAUNCH, first lift-off vx, first
    // touchdown CoM x, last CoM x
    let (mut fwd_x0, mut fwd_lo_vx, mut fwd_td_x, mut fwd_last_x) = (None::<f64>, None::<f64>, None::<f64>, 0.0_f64);
    // soles' midpoint x at T_LAUNCH and now (DIAG_FWD_FEET / JUMP_FWD_TARGET)
    let (mut fwd_feet0, mut fwd_feet_last) = (None::<f64>, f64::NAN);

    for tick in 0..n_ticks {
        let t = tick as f64 * dt;
        let st = rig.sync();
        let (q, v, v_dvec, data) = (&st.q, &st.v, &st.v_dvec, &st.data);
        let com = st.com;
        let com_vel = st.com_vel;
        let stand_z0 = *stand_z.get_or_insert(com.z);
        let mut stand_z = stand_z0 + ground_dz;
        if t >= t_launch {
            if jump_repeat > 0 && fwd_x0.is_none() {
                let sole = |k: usize| {
                    let o = misarta::se3::translation(&st.data.oMi[feet[k]]);
                    let m = misarta::se3::rotation_matrix(&st.data.oMi[feet[k]]);
                    o + m * na::Vector3::new(prof.sole_centre_x, 0.0, 0.0)
                };
                let (s0, s1) = (sole(0), sole(1));
                let mid = 0.5 * (s0 + s1);
                stand_off = Some(com.xy() - mid.xy());
                println!("  [jump-1] t={t:.3} com ({:+.3},{:+.3}) | soles L ({:+.3},{:+.3}) R ({:+.3},{:+.3}) | com-mid ({:+.3},{:+.3})", com.x, com.y, s0.x, s0.y, s1.x, s1.y, com.x - mid.x, com.y - mid.y);
            }
            fwd_x0.get_or_insert(com.x);
        }
        fwd_last_x = com.x;
        if diag_fwd_feet {
            let x = soles_mid_x(&st.data.oMi, &feet, prof.sole_centre_x);
            fwd_feet_last = x;
            if t >= t_launch {
                if fwd_feet0.is_none() && jump_box_gap.is_some() {
                    let (gap, s, onto) = box_gap_s(front_toe_x(&st.data.oMi));
                    (fwd_s, box_jump_now) = (s, onto);
                    push_cfg.vx_target = env_f64("JUMP_MPC_VX", jump_vx * fwd_s);
                    println!("  [box-gap] t={t:.3} toe to box {gap:+.3} m: scale {fwd_s:.4}");
                }
                fwd_feet0.get_or_insert(x);
            }
        }

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
            if push_latch && push_phase && !loaded && in_contact[s] {
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
            } else if air_abort_t > 0.0 && !left_ground && loaded && push_end.is_some_and(|te| t - te >= air_abort_t) {
                // Never both feet off: not a flight. Back to stance without
                // counting a flight span, and absorb from here (as JUMP_ABORT).
                air_sched = false;
                flight_t0 = None;
                for k in 0..2 {
                    in_contact[k] = measured.cop[k][2] > f_on;
                    flip_count[k] = 0;
                }
                land = Some((t, com, com_vel));
                println!("  [air-abort] t={t:.3} no instant with both feet off {air_abort_t:.3} s after the release: no flight, absorb in stance");
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
        if jump_repeat > 0 {
            let (fl, fr) = (measured.cop[0][2].max(0.0), measured.cop[1][2].max(0.0));
            if fl + fr > 0.5 * weight {
                min_share = min_share.min(fl.min(fr) / (fl + fr));
            }
        }
        if (jump_repeat > 0 || jump_turn_hz != 0.0) && tick % 50 == 0 {
            let yaw = |k: usize| {
                let m = misarta::se3::rotation_matrix(&data.oMi[feet[k]]);
                m[(1, 0)].atan2(m[(0, 0)])
            };
            let (_, _, py) = st.body_quat.euler_angles();
            let cmm = misarta::centroidal::compute_centroidal_momentum_matrix(&rig.model, q);
            let hz = (&cmm * v_dvec)[2];
            println!("  [yaw] t={t:.3} nc {} pelvis {py:+.3} feet {:+.3} {:+.3} h_z {hz:+.3} | min load share {min_share:.2}", (0..2).filter(|&k| in_contact[k]).count(), yaw(0), yaw(1));
            min_share = 1.0;
        }
        if let Some(tgt) = jump_turn_target
            && !turn_reached
            && jump_repeat > repeats_done && jump_v > 0.0 && !air_sched && flight_t0.is_none() && in_contact[0] && in_contact[1]
            && let Some((tl, _, _)) = land
            && t - tl >= jump_repeat_after
            && let Some((y_last, done)) = turn_feet
        {
            let y = soles_mean_yaw(&data.oMi, &feet);
            let done = done + wrap_pi(y - y_last);
            turn_feet = Some((y, done));
            let rem = tgt - done;
            if rem.abs() < jump_turn_tol {
                turn_reached = true;
                println!("  [turn-target] t={t:.3} turned {:+.1} deg, {:+.1} to go: no more jumps", done.to_degrees(), rem.to_degrees());
            } else {
                turn_hz_now = turn_hz_for(rem, tgt, false);
                println!("  [turn-target] t={t:.3} turned {:+.1} deg, {:+.1} to go: h_z {turn_hz_now:+.3}", done.to_degrees(), rem.to_degrees());
            }
        }
        if let Some(tgt) = jump_fwd_target
            && !fwd_reached
            && jump_repeat > repeats_done && jump_v > 0.0 && !air_sched && flight_t0.is_none() && in_contact[0] && in_contact[1]
            && let Some((tl, _, _)) = land
            && t - tl >= jump_repeat_after
            && let Some(x0) = fwd_feet0
        {
            let done = soles_mid_x(&data.oMi, &feet, prof.sole_centre_x) - x0;
            let rem = tgt - done;
            if rem.abs() < jump_fwd_tol {
                fwd_reached = true;
                println!("  [fwd-target] t={t:.3} moved {done:+.3} m, {rem:+.3} to go: no more jumps");
            } else {
                fwd_s = fwd_s_for(rem);
                if !jump_mpc_vx_set {
                    push_cfg.vx_target = jump_vx * fwd_s;
                }
                println!("  [fwd-target] t={t:.3} moved {done:+.3} m, {rem:+.3} to go: scale {fwd_s:.4}");
            }
        }
        if let (Some(_), Some((_, bh, _))) = (jump_box_gap, jump_box)
            && !fwd_reached
            && jump_repeat > repeats_done && jump_v > 0.0 && !air_sched && flight_t0.is_none() && in_contact[0] && in_contact[1]
            && let Some((tl, _, _)) = land
            && t - tl >= jump_repeat_after
        {
            if ground_dz > 0.5 * bh {
                fwd_reached = true;
                println!("  [box-gap] t={t:.3} on the box ({ground_dz:+.3} m): no more jumps");
            } else {
                let (gap, s, onto) = box_gap_s(front_toe_x(&data.oMi));
                (fwd_s, box_jump_now) = (s, onto);
                if !jump_mpc_vx_set {
                    push_cfg.vx_target = jump_vx * fwd_s;
                }
                println!("  [box-gap] t={t:.3} toe to box {gap:+.3} m: scale {fwd_s:.4}");
            }
        }
        if jump_repeat > repeats_done && !turn_reached && !fwd_reached && jump_v > 0.0 && !air_sched && flight_t0.is_none() && in_contact[0] && in_contact[1]
            && let Some((tl, _, _)) = land
            && t - tl >= jump_repeat_after
        {
            repeats_done += 1;
            if land_ground_z && let Some(f0) = foot_stand_z {
                ground_dz = 0.5 * (0..2).map(|k| misarta::se3::translation(&data.oMi[feet[k]]).z - f0[k]).sum::<f64>();
                stand_z = stand_z0 + ground_dz;
                println!("  [ground] t={t:.3} re-arm on {ground_dz:+.4} m above the start");
            }
            let r = diag_com_cmd.map_or(com, |(c, _)| c);
            com_ref = Some(na::Vector3::new(r.x, r.y, stand_z));
            let sole = |k: usize| {
                let o = misarta::se3::translation(&data.oMi[feet[k]]);
                let m = misarta::se3::rotation_matrix(&data.oMi[feet[k]]);
                (o + m * na::Vector3::new(prof.sole_centre_x, 0.0, 0.0), m[(1, 0)].atan2(m[(0, 0)]))
            };
            let ((s0, y0), (s1, y1)) = (sole(0), sole(1));
            let mid = 0.5 * (s0 + s1);
            let (rr, pp, _) = st.body_quat.euler_angles();
            println!(
                "  [jump-again] t={t:.3} jump {} of {}: com ({:+.3},{:+.3}, z {:.4}) ref ({:+.3},{:+.3}) v ({:+.3},{:+.3}) | soles L ({:+.3},{:+.3}) R ({:+.3},{:+.3}) yaw {y0:+.3} {y1:+.3} | com-mid ({:+.3},{:+.3}) | roll {rr:+.3} pitch {pp:+.3}",
                repeats_done + 1, jump_repeat + 1, com.x, com.y, com.z, r.x, r.y, com_vel.x, com_vel.z, s0.x, s0.y, s1.x, s1.y, com.x - mid.x, com.y - mid.y
            );
            if jump_repeat_heading {
                let (sy, cy) = (y0.sin() + y1.sin(), y0.cos() + y1.cos());
                heading = Some(sy.atan2(cy));
                println!("  [heading] {:+.3} rad", sy.atan2(cy));
            }
            if jump_repeat_recentre && let Some(off) = stand_off {
                let off = na::Rotation2::new(heading.unwrap_or(0.0)) * off;
                rearm_shift = mid.xy() + off - r.xy();
                if let Some(psi) = heading {
                    // the crouch lays the shift out in the heading frame
                    rearm_shift = na::Rotation2::new(-psi) * rearm_shift;
                }
                println!("  [recentre] shift ({:+.3},{:+.3}) over the crouch", rearm_shift.x, rearm_shift.y);
            }
            crouch_z0 = Some(r.z);
            turn_yaw = [f64::NAN; 3];
            t_launch = t;
            push_end = None;
            release_t0 = None;
            land = None;
            touchdown_t = None;
            push_plan = None;
            h_nat_prev = None;
            h_nat_prev_x = None;
            rest_settled = false;
            first_load = [None, None];
            hx_done = false;
            hx_int = [[0.0; 4]; 5];
            hx_at = [f64::NAN; 6];
            hx_fz = [[0.0; 2]; 5];
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
        let fz_stand0 = *foot_stand_z.get_or_insert([misarta::se3::translation(&data.oMi[feet[0]]).z, misarta::se3::translation(&data.oMi[feet[1]]).z]);
        let mut fz_stand = fz_stand0.map(|z| z + ground_dz);
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
                fwd_lo_vx.get_or_insert(com_vel.x);
                h_ang0 = None;
                feet_off = Some(std::array::from_fn(|k| misarta::se3::translation(&data.oMi[feet[k]]) - com));
                trunk_yaw0 = st.body_quat.euler_angles().2;
                if touchdown_t.is_none() {
                    turn_yaw[1] = trunk_yaw0;
                }
                if flight_feet_yaw {
                    feet_yaw0 = std::array::from_fn(|k| {
                        let m = misarta::se3::rotation_matrix(&data.oMi[feet[k]]);
                        m[(1, 0)].atan2(m[(0, 0)])
                    });
                }
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
                    fwd_td_x = Some(com.x);
                }
                // the first touchdown of a jump only: a later 2 ms "flight"
                // with a heel over the box edge measures the tilted foot
                if land_ground_z && touchdown_t.is_none() {
                    let dz: Vec<f64> = (0..2).filter(|&k| in_contact[k]).map(|k| misarta::se3::translation(&data.oMi[feet[k]]).z - fz_stand0[k]).collect();
                    if !dz.is_empty() {
                        ground_dz = dz.iter().sum::<f64>() / dz.len() as f64;
                        stand_z = stand_z0 + ground_dz;
                        fz_stand = fz_stand0.map(|z| z + ground_dz);
                    }
                    println!("  [ground] t={t:.3} landed {ground_dz:+.4} m above the start (feet {dz:.4?})");
                }
                touchdown_t.get_or_insert(t);
                ever_landed = true;
                com_ref = Some(na::Vector3::new(com.x, com.y, stand_z));
                if land_absorb {
                    land = Some((t, com, com_vel));
                }
                let fx: Vec<String> = (0..2).map(|k| format!("{:+.3}", misarta::se3::translation(&data.oMi[feet[k]]).x)).collect();
                println!("  [touchdown] t={t:.3} after {:.3} s, com ({:+.3}, z {:.4}) v ({:+.3}, {:+.3}), foot x {:?}, feet {:?}", t - t0, com.x, com.z, com_vel.x, com_vel.z, fx, in_contact);
                if jump_turn_hz != 0.0 && turn_yaw[2].is_nan() {
                    turn_yaw[2] = st.body_quat.euler_angles().2;
                    let fy: Vec<String> = (0..2)
                        .map(|k| {
                            let m = misarta::se3::rotation_matrix(&data.oMi[feet[k]]);
                            format!("{:+.3}", m[(1, 0)].atan2(m[(0, 0)]))
                        })
                        .collect();
                    let cmm = misarta::centroidal::compute_centroidal_momentum_matrix(&rig.model, q);
                    let h = &cmm * v_dvec;
                    turn_td_hz = h[2];
                    println!("  [turn] touchdown pelvis yaw {:+.3} (lift-off {:+.3}, push start {:+.3}), feet yaw {:?}, h_z {:+.3}, yaw rate {:+.3}", turn_yaw[2], turn_yaw[1], turn_yaw[0], fy, h[2], st.v_ang_w[2]);
                }
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
        let mut mpc_h: Option<(f64, f64)> = None; // JUMP_MPC: planned (h_y, dh_y)
        push_phase = false;
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
                    let sole = |k: usize| {
                        let o = misarta::se3::translation(&data.oMi[feet[k]]);
                        let r = misarta::se3::rotation_matrix(&data.oMi[feet[k]]);
                        (o + r * na::Vector3::new(prof.sole_centre_x, 0.0, 0.0)).xy()
                    };
                    if land_ref_follow == 2 {
                        let now = [sole(0), sole(1)];
                        if land_follow.is_none_or(|(tt, _, _)| tt != t0) {
                            land_follow = Some((t0, now, na::Vector2::zeros()));
                        }
                        if let Some((_, last, shift)) = land_follow.as_mut() {
                            let on: Vec<usize> = (0..2).filter(|&k| in_contact[k]).collect();
                            if !on.is_empty() {
                                let d: na::Vector2<f64> = on.iter().map(|&k| now[k] - last[k]).sum();
                                *shift += d / on.len() as f64;
                            }
                            *last = now;
                            c.x += shift.x;
                            c.y += shift.y;
                        }
                    } else if land_ref_follow == 1 {
                        let mid = 0.5 * (sole(0) + sole(1));
                        if land_sole0.is_none_or(|(tt, _)| tt != t0) {
                            land_sole0 = Some((t0, mid));
                        }
                        let m0 = land_sole0.map_or(mid, |(_, m)| m);
                        c.x += mid.x - m0.x;
                        c.y += mid.y - m0.y;
                    }
                    (c, cd, cdd)
                }
                None if jump_v > 0.0 && t >= t_launch && push_end.is_none() => {
                    // crouch, hold, push off
                    let z_low = stand_z - crouch;
                    let s = t - t_launch;
                    let (mut z, mut vz, mut az) = quintic(crouch_z0.unwrap_or(stand_z), 0.0, z_low, jump_t_crouch, s);
                    if jump_repeat_recentre_n >= 2 && repeats_done > 0 && heading.is_none() && let Some(off) = stand_off {
                        // live target: the shift from the soles' midpoint now
                        let sole = |k: usize| {
                            let o = misarta::se3::translation(&data.oMi[feet[k]]);
                            let m = misarta::se3::rotation_matrix(&data.oMi[feet[k]]);
                            (o + m * na::Vector3::new(prof.sole_centre_x, 0.0, 0.0)).xy()
                        };
                        rearm_shift = 0.5 * (sole(0) + sole(1)) + off - c_ref.xy();
                    }
                    let lean_x = jump_lean * fwd_s + rearm_shift.x;
                    let (mut x, mut vx, mut ax) = quintic(0.0, 0.0, lean_x, jump_t_crouch, s);
                    let (y, vy, ay) = quintic(0.0, 0.0, rearm_shift.y, jump_t_crouch, s);
                    let kx = jump_vx * fwd_s / jump_v;
                    let s_push = s - jump_t_crouch - jump_t_hold;
                    if jump_repeat > 0 && s_push > 0.0 && s_push <= dt + 1e-9 {
                        let sole = |k: usize| {
                            let o = misarta::se3::translation(&data.oMi[feet[k]]);
                            let m = misarta::se3::rotation_matrix(&data.oMi[feet[k]]);
                            (o + m * na::Vector3::new(prof.sole_centre_x, 0.0, 0.0)).xy()
                        };
                        let mid = 0.5 * (sole(0) + sole(1));
                        println!(
                            "  [push-start] t={t:.3} com-mid ({:+.3},{:+.3}) ref-mid ({:+.3},{:+.3}) | soles L ({:+.3},{:+.3}) R ({:+.3},{:+.3})",
                            com.x - mid.x, com.y - mid.y, c_ref.x + x - mid.x, c_ref.y + y - mid.y, sole(0).x, sole(0).y, sole(1).x, sole(1).y
                        );
                    }
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
                        push_phase = s_push < t_push;
                        x += kx * (z - z_low);
                        vx += kx * vz;
                        ax += kx * az;
                        if s_push >= t_push && jump_release {
                            let (t0, z0, v0) = *release_t0.get_or_insert((t, z, vz));
                            let sr = t - t0;
                            z = z0 + v0 * sr - 0.5 * G * sr * sr;
                            vz = v0 - G * sr;
                            az = -G;
                            x = lean_x + kx * (z0 - z_low) + kx * v0 * sr;
                            vx = kx * v0;
                            ax = 0.0;
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
                    if jump_mpc && s >= jump_mpc_from {
                        let dz = stand_z + jump_z_to - z_low;
                        let r = jump_push_r;
                        let (t_push, a0, kk) = if r < 1.0 {
                            let tp = 3.0 * dz * (1.0 + r) / ((2.0 + r) * jump_v);
                            (tp, 2.0 * jump_v / ((1.0 + r) * tp), (1.0 - r) / tp)
                        } else {
                            let a = jump_v * jump_v / (2.0 * dz);
                            (jump_v / a, a, 0.0)
                        };
                        let s_end = jump_t_crouch + jump_t_hold + t_push;
                        // the scripted vertical plan: (z, az) at s
                        let vert = |s: f64| -> (f64, f64) {
                            if s < jump_t_crouch {
                                let (z, _, az) = quintic(stand_z, 0.0, z_low, jump_t_crouch, s);
                                (z, az)
                            } else if s < jump_t_crouch + jump_t_hold {
                                (z_low, 0.0)
                            } else {
                                let sp = (s - jump_t_crouch - jump_t_hold).min(t_push);
                                (z_low + a0 * (0.5 * sp * sp - kk * sp.powi(3) / 6.0), a0 * (1.0 - kk * sp))
                            }
                        };
                        let rem = s_end - s;
                        if rem > 0.5 * jump_mpc_dt && (push_plan.is_none() || tick % jump_mpc_every == 0) {
                            let nn = (rem / jump_mpc_dt).ceil().max(1.0) as usize;
                            let dtn = rem / nn as f64;
                            let foot = |k: usize| misarta::se3::translation(&data.oMi[feet[k]]);
                            let z_g = 0.5 * (foot(0).z + foot(1).z) - prof.sole_below_origin;
                            let xs = 0.5 * (foot(0).x + foot(1).x) + prof.sole_centre_x;
                            let (zeta, fzv): (Vec<f64>, Vec<f64>) = (0..nn)
                                .map(|k| {
                                    let (z, az) = vert(s + (k as f64 + 0.5) * dtn);
                                    (z - z_g, total_mass * (G + az))
                                })
                                .unzip();
                            push_cfg.cop_lo = xs - sole_half_l + jump_mpc_cop_margin;
                            push_cfg.cop_hi = xs + sole_half_l - jump_mpc_cop_margin;
                            push_cfg.x_lo = xs - jump_mpc_x_back;
                            push_cfg.x_hi = xs + jump_mpc_x_fwd;
                            let cmm = misarta::centroidal::compute_centroidal_momentum_matrix(&rig.model, q);
                            let h_y = (&cmm * v_dvec)[1];
                            // JUMP_MPC_MEAS=0: replan from the current plan's own
                            // state at t (the WBC's PD tracks the plan); =1 from
                            // the measured CoM x, vx and h_y. The first plan is
                            // always from the measured state.
                            let (x0, v0, h0) = match (&push_plan, jump_mpc_meas) {
                                (Some(pl), false) => {
                                    let (px, pv, _, ph, _) = pl.eval(t);
                                    (px, pv, ph)
                                }
                                _ => (com.x, com_vel.x, h_y),
                            };
                            match plan_push(t, dtn, x0, v0, h0, total_mass, &zeta, &fzv, &push_cfg) {
                                Ok(pl) => {
                                    if diag_mpc {
                                        let (_, ve, _, he, _) = pl.eval(t + rem);
                                        let pmin = pl.p.iter().cloned().fold(f64::INFINITY, f64::min) - xs;
                                        let pmax = pl.p.iter().cloned().fold(f64::NEG_INFINITY, f64::max) - xs;
                                        let xmax = (0..=nn).map(|k| pl.eval(t + k as f64 * dtn).0).fold(f64::NEG_INFINITY, f64::max) - xs;
                                        println!(
                                            "  [mpc] t={t:.3} n {nn} rem {rem:.3} | now x-xs {:+.3} vx {:+.3} h_y {h_y:+.3} | plan a0 {:+.2} p0-xs {:+.3} p-xs [{pmin:+.3},{pmax:+.3}] max x-xs {xmax:+.3} | end vx {ve:+.3} h_y {he:+.3}",
                                            com.x - xs, com_vel.x, pl.a[0], pl.p[0] - xs
                                        );
                                    }
                                    push_plan = Some(pl);
                                }
                                Err(e) => {
                                    mpc_fails += 1;
                                    if diag_mpc {
                                        println!("  [mpc] t={t:.3} solve failed ({mpc_fails}): {e}");
                                    }
                                }
                            }
                        }
                        if let Some(pl) = &push_plan {
                            let (px, pv, pa, ph, pdh) = pl.eval(t);
                            x = px - c_ref.x;
                            vx = pv;
                            ax = pa;
                            mpc_h = Some((ph, pdh));
                        }
                    }
                    if let Some(psi) = heading {
                        let rot = na::Rotation2::new(psi);
                        let (p, pv, pa) = (rot * na::Vector2::new(x, y), rot * na::Vector2::new(vx, vy), rot * na::Vector2::new(ax, ay));
                        (na::Vector3::new(c_ref.x + p.x, c_ref.y + p.y, z), na::Vector3::new(pv.x, pv.y, vz), na::Vector3::new(pa.x, pa.y, az))
                    } else {
                        (na::Vector3::new(c_ref.x + x, c_ref.y + y, z), na::Vector3::new(vx, vy, vz), na::Vector3::new(ax, ay, az))
                    }
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
            diag_com_vref = Some((cd_ref, cdd_ref));
            levels.push(bt::com(dyn_ctx.qddot(), &st.j_com, &st.djv_com, &a_com));
            names.push("com");
        }
        let j_trunk = misarta::jacobian::compute_joint_jacobian_from_data(&rig.model, q, data, rig.trunk_mi);
        let djv_trunk = &misarta::jacobian::compute_joint_jacobian_time_derivative(&rig.model, q, v, rig.trunk_mi) * v_dvec;
        let (roll, pitch, yaw_now) = st.body_quat.euler_angles();
        turn_last_yaw = yaw_now;
        if jump_turn_target.is_some() {
            turn_last_feet = soles_mean_yaw(&data.oMi, &feet);
        }
        if !flight || flight_trunk {
            // KP/KD_TRUNK_LAND: the trunk gains after a landing faster than
            // WBCW_TRUNK_LAND_VZ (default = KP/KD_TRUNK). Measured
            // 2026-10-28: the weight 0.03 frees the CoM but lets the trunk
            // pitch on from 0.12 to 0.52 rad in 0.5 s (13 of 15 2.0 falls).
            let hard_land = touchdown_t.is_some() && (trunk_land_vz <= 0.0 || td_vz < -trunk_land_vz);
            let (kp_t, kd_t) = if hard_land { (kp_trunk_land, kd_trunk_land) } else { (kp_trunk, kd_trunk) };
            let rp_ref = if trunk_rp_yawed {
                let (sy, cy) = yaw_now.sin_cos();
                let (er, ep) = (-kp_t * roll, -kp_t * pitch);
                [cy * er - sy * ep - kd_t * st.v_ang_w[0], sy * er + cy * ep - kd_t * st.v_ang_w[1]]
            } else {
                bt::trunk_rp_ref(roll, pitch, &st.v_ang_w, kp_t, kd_t, 0.0, 1.0)
            };
            levels.push(bt::trunk(dyn_ctx.qddot(), &j_trunk, &djv_trunk, &rp_ref, nv));
            names.push("trunk");
        }
        let turn_ramp = jump_turn_hz != 0.0 && jump_v > 0.0 && t - t_launch >= jump_t_crouch + jump_t_hold - jump_turn_t0 && touchdown_t.is_none();
        let stance_yaw = !flight && stance_trunk_yaw > 0.0 && ever_landed && !(jump_turn_free_yaw && turn_ramp);
        if (flight && flight_trunk_yaw > 0.0) || stance_yaw {
            let tgt = if flight {
                trunk_yaw0
            } else {
                let (sy, cy) = (0..2).fold((0.0, 0.0), |(a, b), k| {
                    let m = misarta::se3::rotation_matrix(&data.oMi[feet[k]]);
                    let y = m[(1, 0)].atan2(m[(0, 0)]);
                    (a + y.sin(), b + y.cos())
                });
                sy.atan2(cy)
            };
            let mut e = tgt - st.body_quat.euler_angles().2;
            e = (e + std::f64::consts::PI).rem_euclid(2.0 * std::f64::consts::PI) - std::f64::consts::PI;
            let a = kp_trunk * e - kd_trunk * st.v_ang_w[2];
            // The pelvis (root) -- where body_quat and v_ang_w are measured.
            // On G1 the trunk task's body is torso_link, past the waist yaw
            // joint: driving its yaw while measuring the pelvis diverged.
            let j_base = misarta::jacobian::compute_joint_jacobian_from_data(&rig.model, q, data, 1);
            let djv_base = &misarta::jacobian::compute_joint_jacobian_time_derivative(&rig.model, q, v, 1) * v_dvec;
            levels.push(tasks::cartesian_acceleration(
                dyn_ctx.qddot(),
                &j_base.rows(2, 1).into_owned(),
                &na::DVector::from_vec(vec![djv_base[2]]),
                &na::DVector::from_vec(vec![a]),
            ));
            names.push("trunk yaw");
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
            let (mut h_ref_m, mut dh_ref_m) = mpc_h.map_or(([0.0; 3], [0.0; 3]), |(h, dh)| ([0.0, h, 0.0], [0.0, dh, 0.0]));
            let s_j = t - t_launch;
            let t_ch = jump_t_crouch + jump_t_hold;
            let nat_on = jump_h_nat && mpc_h.is_none() && jump_v > 0.0 && s_j >= 0.0 && push_end.is_none() && land.is_none()
                && (s_j < t_ch || jump_h_nat_push);
            if nat_on && let Some((cd_ref, _)) = diag_com_vref {
                // unknowns: base (6), legs and waist; rows: both feet (12),
                // trunk angular velocity (3), CoM velocity (3)
                let mut cols: Vec<usize> = (0..6).collect();
                for &(ji, vi) in rig.actuated().iter() {
                    let name = rig.robot.joints[ji].name.as_str();
                    if name.contains("hip") || name.contains("knee") || name.contains("ankle") || name.contains("waist") {
                        cols.push(vi);
                    }
                }
                let jl = misarta::jacobian::compute_joint_jacobian_from_data(&rig.model, q, data, feet[0]);
                let jr = misarta::jacobian::compute_joint_jacobian_from_data(&rig.model, q, data, feet[1]);
                let mut a = na::DMatrix::<f64>::zeros(18, cols.len());
                let mut b = na::DVector::<f64>::zeros(18);
                for (ci, &c) in cols.iter().enumerate() {
                    for r in 0..6 {
                        a[(r, ci)] = jl[(r, c)];
                        a[(6 + r, ci)] = jr[(r, c)];
                    }
                    for r in 0..3 {
                        a[(12 + r, ci)] = j_trunk[(r, c)];
                        a[(15 + r, ci)] = st.j_com[(r, c)];
                    }
                }
                for r in 0..3 {
                    b[15 + r] = cd_ref[r];
                }
                if let Ok(x) = a.clone().svd(true, true).solve(&b, 1e-9) {
                    let mut qd = na::DVector::<f64>::zeros(nv);
                    for (ci, &c) in cols.iter().enumerate() {
                        qd[c] = x[ci];
                    }
                    let hq = &cmm * &qd;
                    let hn = hq[1];
                    let scale = if s_j < t_ch { 1.0 } else {
                        let dz = stand_z + jump_z_to - (stand_z - crouch);
                        let r = jump_push_r;
                        let t_push = if r < 1.0 { 3.0 * dz * (1.0 + r) / ((2.0 + r) * jump_v) } else { 2.0 * dz / jump_v };
                        (1.0 - (s_j - t_ch) / t_push).clamp(0.0, 1.0)
                    };
                    let h = scale * hn;
                    let dh = h_nat_prev.map_or(0.0, |hp| (h - hp) / dt);
                    h_nat_prev = Some(h);
                    h_ref_m[1] = h;
                    dh_ref_m[1] = dh;
                    if jump_h_nat_xy {
                        let hx = scale * hq[0];
                        dh_ref_m[0] = h_nat_prev_x.map_or(0.0, |hp| (hx - hp) / dt);
                        h_nat_prev_x = Some(hx);
                        h_ref_m[0] = hx;
                    }
                    if diag_hnat && tick % 10 == 0 {
                        println!("  [hnat] t={t:.3} h_nat {hn:+.3} ref {h:+.3} dh {dh:+.2} h_y {:+.3} vz_ref {:+.3} | com x {:+.4} z {:.4}", h_now[1], cd_ref.z, com.x, com.z);
                    }
                }
            }
            if jump_turn_hz != 0.0 && jump_v > 0.0 && s_j >= t_ch - jump_turn_t0 && touchdown_t.is_none() {
                if turn_yaw[0].is_nan() {
                    turn_yaw[0] = st.body_quat.euler_angles().2;
                    turn_jumps += 1;
                    if let Some(tgt) = jump_turn_target
                        && turn_feet.is_none()
                    {
                        turn_feet = Some((soles_mean_yaw(&data.oMi, &feet), 0.0));
                        turn_hz_now = turn_hz_for(tgt, tgt, true);
                        println!("  [turn-target] t={t:.3} target {:+.1} deg: h_z {turn_hz_now:+.3}", tgt.to_degrees());
                    }
                }
                let dz = crouch + jump_z_to;
                let r = jump_push_r;
                let t_push = if r < 1.0 { 3.0 * dz * (1.0 + r) / ((2.0 + r) * jump_v) } else { 2.0 * dz / jump_v };
                let tr = t_push + jump_turn_t0;
                let p = (s_j - t_ch + jump_turn_t0) / tr;
                h_ref_m[2] = turn_hz_now * p.clamp(0.0, 1.0);
                dh_ref_m[2] = if p < 1.0 { turn_hz_now / tr } else { 0.0 };
            }
            if jump_turn_hz != 0.0 && jump_turn_land_t > 0.0 && let Some(td) = touchdown_t
                && t - td < jump_turn_land_t
            {
                h_ref_m[2] = turn_td_hz * (1.0 - (t - td) / jump_turn_land_t);
                dh_ref_m[2] = -turn_td_hz / jump_turn_land_t;
            }
            let cmm = na::DMatrix::from_column_slice(cmm.nrows(), cmm.ncols(), cmm.as_slice());
            if let Some(tk) = bt::angular_momentum(dyn_ctx.qddot(), &cmm, &dcmm_v, &h_now, &h_ref_m, &dh_ref_m, kp_mom, [true; 3], nv) {
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
        if flight && flight_feet && let Some(mut off) = feet_off {
            if flight_feet_capture > 0.0 && let Some(t0f) = flight_t0 {
                let h = (stand_z - (fz_stand[0].min(fz_stand[1]) - prof.sole_below_origin)).max(0.3);
                let b = ((t - t0f) / flight_feet_capture_t).clamp(0.0, 1.0);
                let b = b * b * (3.0 - 2.0 * b);
                if let Some(psi) = heading {
                    let d = na::Vector3::new(psi.cos(), psi.sin(), 0.0);
                    let tgt = flight_feet_capture * com_vel.dot(&d) / (G / h).sqrt() - prof.sole_centre_x;
                    for o in off.iter_mut() {
                        let along = o.dot(&d);
                        *o += d * ((tgt - along) * b);
                    }
                } else {
                    let tgt = flight_feet_capture * com_vel.x / (G / h).sqrt() - prof.sole_centre_x;
                    for o in off.iter_mut() {
                        o.x += (tgt - o.x) * b;
                    }
                }
            }
            if flight_feet_width > 0.0 && let Some(t0f) = flight_t0 {
                let b = ((t - t0f) / flight_feet_capture_t).clamp(0.0, 1.0);
                let b = b * b * (3.0 - 2.0 * b);
                // FLIGHT_FEET_TURN with a heading: across the heading, not
                // world y (a repeated turning jump starts turned).
                let psi = if flight_feet_turn { heading.unwrap_or(0.0) } else { 0.0 };
                let n = na::Vector3::new(-psi.sin(), psi.cos(), 0.0);
                for (k, o) in off.iter_mut().enumerate() {
                    let tgt = if k == 0 { 0.5 } else { -0.5 } * flight_feet_width;
                    if psi == 0.0 {
                        o.y += (tgt - o.y) * b;
                    } else {
                        *o += n * ((tgt - o.dot(&n)) * b);
                    }
                }
            }
            if flight_feet_level_smooth && let Some(t0f) = flight_t0 {
                let b = ((t - t0f) / flight_feet_capture_t).clamp(0.0, 1.0);
                let b = b * b * (3.0 - 2.0 * b);
                let low = off[0].z.min(off[1].z);
                for o in off.iter_mut() {
                    o.z += (low - o.z) * b;
                }
            }
            let dpsi_turn = if flight_feet_turn {
                let d = st.body_quat.euler_angles().2 - if turn_yaw[0].is_nan() { trunk_yaw0 } else { turn_yaw[0] };
                (d + std::f64::consts::PI).rem_euclid(2.0 * std::f64::consts::PI) - std::f64::consts::PI
            } else {
                0.0
            };
            if flight_feet_turn {
                let rz = na::Rotation3::from_axis_angle(&na::Vector3::z_axis(), dpsi_turn);
                for o in off.iter_mut() {
                    *o = rz * *o;
                }
            }
            if flight_feet_tuck_tf > 0.0 && let Some(t0f) = flight_t0 && tuck_adapt.is_none_or(|x| x.0 != t0f) {
                let tf = flight_feet_tuck_tf;
                let z_nat = (0..2).map(|k| fz_stand[k] - off[k].z).fold(f64::NEG_INFINITY, f64::max);
                let z_star = com.z + com_vel.z * tf - 0.5 * G * tf * tf;
                let d = (z_nat - z_star).clamp(0.0, flight_feet_tuck_max);
                tuck_adapt = Some((t0f, d));
                println!("  [tuck] t={t:.3} com z {:.4} vz {:+.3}: feet meet the floor at com z {z_nat:.4}, ballistic at {tf:.3} s {z_star:.4} -> tuck {d:.4}", com.z, com_vel.z);
            }
            let tuck = flight_feet_tuck + if box_jump_now { jump_box_tuck } else { 0.0 } + if flight_feet_tuck_tf > 0.0 { tuck_adapt.map_or(0.0, |x| x.1) } else { 0.0 };
            if tuck != 0.0 && let Some(t0f) = flight_t0 {
                let b = ((t - t0f) / flight_feet_tuck_t).clamp(0.0, 1.0);
                let b = b * b * (3.0 - 2.0 * b);
                for o in off.iter_mut() {
                    o.z += tuck * b;
                }
            }
            if diag_ffeet && tick % 5 == 0 {
                let rel: Vec<String> = (0..2)
                    .map(|k| {
                        let p = misarta::se3::translation(&data.oMi[feet[k]]) - com;
                        format!("tgt ({:+.3},{:+.3}) act ({:+.3},{:+.3})", off[k].x, off[k].z, p.x, p.z)
                    })
                    .collect();
                let (_, pch, _) = st.body_quat.euler_angles();
                println!("  [ffeet] t={t:.3} com x {:+.3} z {:.4} vx {:+.3} pitch {pch:+.3} | L {} | R {}", com.x, com.z, com_vel.x, rel[0], rel[1]);
            }
            for k in 0..2 {
                let jf = misarta::jacobian::compute_joint_jacobian_from_data(&rig.model, q, data, feet[k]);
                let djv = &misarta::jacobian::compute_joint_jacobian_time_derivative(&rig.model, q, v, feet[k]) * v_dvec;
                let pos = misarta::se3::translation(&data.oMi[feet[k]]);
                let vel3 = &jf.rows(3, 3).into_owned() * v_dvec;
                let vel = na::Vector3::new(vel3[0], vel3[1], vel3[2]);
                let mut p_tgt = com + off[k];
                if let (Some(c), Some((bx, bh, _))) = (jump_box_clear, jump_box)
                    && pos.x + prof.sole_centre_x - sole_half_l - jump_box_clear_x < bx
                {
                    p_tgt.z = p_tgt.z.max(bh + c + prof.sole_below_origin);
                }
                if flight_feet_flat {
                    // FLIGHT_FEET_FLAT=1: also hold the sole level (roll and
                    // pitch to 0, yaw as it is), so both feet meet the floor
                    // flat. Measured 2026-10-24: with position-only feet they
                    // touched down pitched 11-13 deg and rolled 6-8 deg.
                    let rot = misarta::se3::rotation_matrix(&data.oMi[feet[k]]);
                    let yaw = if flight_feet_yaw { feet_yaw0[k] + dpsi_turn } else { rot[(1, 0)].atan2(rot[(0, 0)]) };
                    let rot_tgt = *na::Rotation3::from_axis_angle(&na::Vector3::z_axis(), yaw).matrix();
                    let w3 = &jf.rows(0, 3).into_owned() * v_dvec;
                    let omega = na::Vector3::new(w3[0], w3[1], w3[2]);
                    levels.push(bt::swing_with_pose(
                        dyn_ctx.qddot(), &jf, &djv, &pos, &vel, &p_tgt, &com_vel,
                        kp_ff, kp_ff, kd_ff, if flight_feet_yaw { bt::SwingAxes::Pose } else { bt::SwingAxes::XyzRollPitch },
                        Some((rot, rot_tgt, omega, kp_ffr, kd_ffr)),
                    ));
                } else {
                    levels.push(bt::swing(
                        dyn_ctx.qddot(), &jf, &djv, &pos, &vel, &p_tgt, &com_vel,
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
        let jump_from = if rest_launch && !(jump_repeat_soft_crouch && repeats_done > 0 && !(jump_box_stiff && box_jump_now)) { t_launch } else { t_launch + jump_t_crouch + jump_t_hold };
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
                    "trunk yaw" if flight => flight_trunk_yaw,
                    "trunk yaw" => stance_trunk_yaw,
                    "step swing" => land_step_w,
                    "reseat" => foot_reseat_w,
                    "momentum roll" => jump_mom_x_w,
                    "fz sym" => jump_fz_sym_w,
                    // WBCW_MOMENTUM_PUSH: the momentum weight during the push
                    // and the release only (default = WBCW_MOMENTUM).
                    "momentum" if in_push => env_f64("WBCW_MOMENTUM_PUSH", mom_w),
                    "momentum" => mom_w,
                    "contact+cones" if contact_rest && box_jump_now && repeats_done > 0 && jump_box_crouch_w.is_some() && jump_v > 0.0 && t >= t_launch && push_end.is_none() => {
                        jump_box_crouch_w.unwrap()
                    }
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
                    "trunk" if touchdown_t.is_some_and(|t0| t - t0 < trunk_land_t) && (trunk_land_vz <= 0.0 || td_vz < -trunk_land_vz) => {
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
        let crouch_t0 = (diag_crouch && jump_v > 0.0 && push_end.is_none() && t >= t_launch && t < t_launch + jump_t_crouch + jump_t_hold + 0.2).then_some(t_launch);
        if (diag_land || crouch_t0.is_some()) && tick % 10 == 0 && let Some(t0) = crouch_t0.or(touchdown_t.filter(|&t0| diag_land && t - t0 <= 3.0)) {
            let a_sol = &st.j_com * &ex.qddot + st.djv_com;
            let (cr, ac) = diag_com_cmd.unwrap_or((com, na::Vector3::zeros()));
            let feet_x: Vec<f64> = (0..2).map(|k| misarta::se3::translation(&data.oMi[feet[k]]).x + prof.sole_centre_x).collect();
            let label = if crouch_t0.is_some() { if contact_rest { "crouch soft" } else { "crouch stiff" } } else { "land" };
            println!(
                "  [{label}] t+{:.2} nc={nc} com x {:+.3} ref {:+.3} vx {:+.3} | ax cmd {:+.2} sol {:+.2} | pitch {:+.3} rate {:+.3} | sole x {:+.3} {:+.3} cop x {:+.3} {:+.3} fz {:.0} {:.0}",
                t - t0, com.x, cr.x, com_vel.x, ac.x, a_sol[0], pitch, st.v_ang_w[1],
                feet_x[0], feet_x[1], measured.cop[0][0], measured.cop[1][0], measured.cop[0][2], measured.cop[1][2]
            );
        }

        if diag_pushf && jump_v > 0.0 && tick % diag_pushf_every == 0 && t >= t_launch + jump_t_crouch + jump_t_hold - 0.02
            && push_end.is_none_or(|te| t <= te + 0.01)
        {
            let sides: Vec<usize> = (0..2).filter(|&k| in_contact[k]).collect();
            let nfc = ex.forces.len() / 6;
            let acc_c = if nfc > 0 { &j_contact * &ex.qddot + &dj_v } else { na::DVector::zeros(0) };
            let cmm = misarta::centroidal::compute_centroidal_momentum_matrix(&rig.model, q);
            let h_x = (&cmm * v_dvec)[0];
            let joint = |side: &str, part: &str| -> String {
                rig.actuated()
                    .iter()
                    .find(|&&(ji, _)| rig.robot.joints[ji].name == format!("{side}_{part}_joint"))
                    .map_or("?".into(), |&(ji, vi)| {
                        let j = &rig.robot.joints[ji];
                        let qa = rig.robot.joint_positions[ji];
                        let room = (qa - j.lower).min(j.upper - qa);
                        format!("{:+.0}/{:.0}@{qa:+.2}({room:.2})", ex.tau[vi - 6], rig.torque_max[vi - 6])
                    })
            };
            let foot_str = |k: usize, side: &str| -> String {
                let (fq, az) = match sides.iter().position(|&x| x == k) {
                    Some(i) if 6 * i + 5 < ex.forces.len() => (ex.forces[6 * i + 5], acc_c[6 * i + 5]),
                    _ => (f64::NAN, f64::NAN),
                };
                let m = misarta::se3::rotation_matrix(&data.oMi[feet[k]]);
                let (r, pch, _) = na::Rotation3::from_matrix_unchecked(m).euler_angles();
                let z = misarta::se3::translation(&data.oMi[feet[k]]).z - prof.sole_below_origin;
                format!(
                    "{} fz qp {fq:4.0} mj {:4.0} cop ({:+.3},{:+.3}) az {az:+.2} sole z {z:+.4} r {r:+.3} p {pch:+.3} | knee {} apitch {} aroll {}",
                    if in_contact[k] { "C" } else { "-" }, measured.cop[k][2], measured.cop[k][0], measured.cop[k][1],
                    joint(side, "knee"), joint(side, "ankle_pitch"), joint(side, "ankle_roll")
                )
            };
            println!(
                "  [pushf] t={t:.3} {} h_x {h_x:+.3} com y {:+.4} vy {:+.3} || L {} || R {}",
                if release_t0.is_some() { "REL" } else { "PSH" }, com.y, com_vel.y, foot_str(0, "left"), foot_str(1, "right")
            );
        }
        if diag_pushx && jump_v > 0.0 && tick % 10 == 0 && t >= t_launch + jump_t_crouch + jump_t_hold - 0.02
            && push_end.is_none_or(|te| t <= te + 0.03)
        {
            let a_sol = &st.j_com * &ex.qddot + st.djv_com;
            let (cr, ac) = diag_com_cmd.unwrap_or((com, na::Vector3::zeros()));
            let (vr, ar) = diag_com_vref.unwrap_or((na::Vector3::zeros(), na::Vector3::zeros()));
            let m = total_mass;
            let nfc = ex.forces.len() / 6;
            let fx_qp: f64 = (0..nfc).map(|i| ex.forces[6 * i + 3]).sum();
            let fx_mj = measured.f_w[0][0] + measured.f_w[1][0];
            let acc_c = if nfc > 0 { &j_contact * &ex.qddot + &dj_v } else { na::DVector::zeros(0) };
            let leak: Vec<String> = (0..nfc).map(|i| format!("{:+.2}/{:+.2}", acc_c[6 * i + 3], acc_c[6 * i + 5])).collect();
            println!(
                "  [pushx] t={t:.3} {} ref x {:+.3} vx {:+.3} ax {:+.2} | com x {:+.3} vx {:+.3} | ax cmd {:+.2} sol {:+.2} qpF {:+.2} plant {:+.2} | contact acc x/z [{}] | pitch {:+.3} rate {:+.2} | cop x {:+.3} {:+.3} fz {:.0} {:.0}",
                if flight { "AIR" } else if release_t0.is_some() { "REL" } else { "PSH" },
                cr.x, vr.x, ar.x, com.x, com_vel.x, ac.x, a_sol[0], fx_qp / m, fx_mj / m,
                leak.join(" "), pitch, st.v_ang_w[1],
                measured.cop[0][0], measured.cop[1][0], measured.cop[0][2], measured.cop[1][2]
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
            let fallback = move |out: &mut [f64]| gravity_plus_posture(rig_ref, &rig_ref.robot.joint_positions, tg, v, hold_kp, hold_kd, out);
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
        let q_joint = rig.robot.joint_positions.clone();
        write_to_plant(&mut rig, ctrl_mode, &q_joint, &robot_taus, &ex.qddot, v, dt);
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
        // FALL_TILT: the tilt verdict [rad] (default 0.52). Raised, it tells
        // a torso that leans past 0.52 and comes back from one that falls.
        if cur_z < 0.30 || tilt > fall_tilt {
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
    if let Some(x0) = fwd_x0 {
        println!(
            "  forward: lift-off vx {:+.3}, touchdown CoM dx {:+.3}, final CoM dx {:+.3} m",
            fwd_lo_vx.unwrap_or(f64::NAN), fwd_td_x.map_or(f64::NAN, |x| x - x0), fwd_last_x - x0
        );
    }
    if let Some(x0) = fwd_feet0 {
        let d = fwd_feet_last - x0;
        match jump_fwd_target {
            Some(tgt) => println!(
                "  forward target {tgt:+.3} m (last scale {fwd_s:.4}): soles moved {d:+.3} m at the end (error {:+.3}), {} jumps, reached {fwd_reached}",
                d - tgt,
                repeats_done + 1
            ),
            None => println!("  forward: soles moved {d:+.3} m at the end"),
        }
        if jump_box_gap.is_some() {
            println!("  box gap: {} jumps, on the box {fwd_reached}", repeats_done + 1);
        }
    }
    if jump_turn_hz != 0.0 {
        println!(
            "  turn: pelvis yaw push start {:+.3}, lift-off {:+.3}, touchdown {:+.3}, final {:+.3} | in flight {:+.3}, push start to final {:+.3} rad",
            turn_yaw[0], turn_yaw[1], turn_yaw[2], turn_last_yaw, turn_yaw[2] - turn_yaw[1], turn_last_yaw - turn_yaw[0]
        );
    }
    if let Some(tgt) = jump_turn_target {
        let fin = turn_feet.map_or(f64::NAN, |(y, done)| done + wrap_pi(turn_last_feet - y));
        println!(
            "  turn target {:+.1} deg: soles turned {:+.1} deg at the end (error {:+.1}), {turn_jumps} turning jumps, reached {turn_reached}",
            tgt.to_degrees(), fin.to_degrees(), (fin - tgt).to_degrees()
        );
    }
    println!("  max tilt after touchdown: {max_tilt_after:.3} rad, min CoM z after touchdown {min_com_z_after:.3} (stand {:.3})", stand_z.unwrap_or(0.0));
    println!("  first touchdown vz {td_vz:+.3} m/s, peak GRF in the first 1 s {:.2} x weight, peak torque-box use {peak_tau_after:.2}", peak_grf_after / weight);
    println!("  degraded solves (all): {}", tally.n);
    tally.report();
    println!("  landing steps: {steps_done}");
    if jump_mpc {
        println!("  push MPC failed solves: {mpc_fails}");
    }
    if jump_repeat > 0 {
        println!("  repeat: {} jumps started, {} flights", repeats_done + 1, flight_spans.len());
    }
    println!("  verdict: {}", if fell { "FELL" } else if flight_spans.is_empty() { "NO FLIGHT" } else { "LANDED" });
}

/// The sagittal push plan of `JUMP_MPC`: per node, the CoM's x acceleration
/// and the net CoP x, piecewise constant, from the solve time to the end of
/// the push.
#[cfg(feature = "mujoco")]
#[derive(Clone, Debug)]
struct PushPlan {
    t0: f64,
    dt: f64,
    x0: f64,
    v0: f64,
    h0: f64,
    m: f64,
    a: Vec<f64>,
    p: Vec<f64>,
    /// CoM height above the soles and vertical force at each node's midpoint.
    zeta: Vec<f64>,
    fz: Vec<f64>,
}

#[cfg(feature = "mujoco")]
impl PushPlan {
    /// (x, vx, ax, h_y, dh_y) at time t; past the last node the end state
    /// coasts (zero acceleration and moment).
    fn eval(&self, t: f64) -> (f64, f64, f64, f64, f64) {
        let (mut x, mut v, mut h) = (self.x0, self.v0, self.h0);
        let mut s = (t - self.t0).max(0.0);
        for k in 0..self.a.len() {
            let d = s.min(self.dt);
            let (a, p) = (self.a[k], self.p[k]);
            let dh = |x: f64| -self.zeta[k] * self.m * a - (p - x) * self.fz[k];
            if s <= self.dt {
                let xm = x + 0.5 * v * d + a * d * d / 8.0;
                h += d * dh(xm);
                x += v * d + 0.5 * a * d * d;
                v += a * d;
                return (x, v, a, h, dh(x));
            }
            let xm = x + 0.5 * v * d + a * d * d / 8.0;
            h += d * dh(xm);
            x += v * d + 0.5 * a * d * d;
            v += a * d;
            s -= d;
        }
        (x + v * s, v, 0.0, h, 0.0)
    }
}

/// Weights and bounds of the `JUMP_MPC` push planner.
#[cfg(feature = "mujoco")]
#[derive(Clone, Debug)]
struct PushPlanCfg {
    vx_target: f64,
    w_v: f64,
    w_h: f64,
    w_h_run: f64,
    w_a: f64,
    w_da: f64,
    w_dp: f64,
    mu: f64,
    /// CoP box in world x (net, both soles side by side).
    cop_lo: f64,
    cop_hi: f64,
    /// CoM x box in world x.
    x_lo: f64,
    x_hi: f64,
}

/// Plan the CoM's x acceleration and the net CoP x from the state now to the
/// end of the push, given the vertical plan (CoM height above the soles and
/// vertical force at each node's midpoint, from the script). With the
/// vertical fixed the pitch momentum is linear in the plan:
///     dh_y/dt = -zeta f_x - (p - x) f_z,   f_x = m a
/// so the push is one convex QP: reach `vx_target` with h_y = 0 at the end,
/// the CoP inside the soles and the friction cone, the CoM inside its box.
#[cfg(feature = "mujoco")]
#[allow(clippy::too_many_arguments)]
fn plan_push(
    t0: f64,
    dt: f64,
    x0: f64,
    v0: f64,
    h0: f64,
    m: f64,
    zeta: &[f64],
    fz: &[f64],
    c: &PushPlanCfg,
) -> Result<PushPlan, String> {
    use nalgebra as na;
    let n = zeta.len();
    if n == 0 || fz.len() != n {
        return Err("empty horizon".into());
    }
    let nu = 2 * n; // [a_0..a_{n-1}, p_0..p_{n-1}]
    // Affine maps (row, constant) of the states at the start of each node.
    type Aff = (na::DVector<f64>, f64);
    let mut xs: Vec<Aff> = Vec::with_capacity(n + 1);
    let mut vs: Vec<Aff> = Vec::with_capacity(n + 1);
    let mut hs: Vec<Aff> = Vec::with_capacity(n + 1);
    let (mut x, mut v, mut h) = ((na::DVector::zeros(nu), x0), (na::DVector::zeros(nu), v0), (na::DVector::zeros(nu), h0));
    for k in 0..n {
        xs.push(x.clone());
        vs.push(v.clone());
        hs.push(h.clone());
        // midpoint x: x + v dt/2 + a dt^2/8
        let mut xm = (&x.0 + &v.0 * (0.5 * dt), x.1 + v.1 * 0.5 * dt);
        xm.0[k] += dt * dt / 8.0;
        // h += dt (-zeta m a - f p + f xm)
        let mut hn = (&h.0 + &xm.0 * (dt * fz[k]), h.1 + dt * fz[k] * xm.1);
        hn.0[k] -= dt * zeta[k] * m;
        hn.0[n + k] -= dt * fz[k];
        let mut xn = (&x.0 + &v.0 * dt, x.1 + v.1 * dt);
        xn.0[k] += 0.5 * dt * dt;
        let mut vn = v.clone();
        vn.0[k] += dt;
        x = xn;
        v = vn;
        h = hn;
    }
    xs.push(x);
    vs.push(v);
    hs.push(h);

    let mut hm = na::DMatrix::<f64>::identity(nu, nu) * 1e-9;
    let mut g = na::DVector::<f64>::zeros(nu);
    let mut add = |r: &na::DVector<f64>, d: f64, tgt: f64, w: f64| {
        hm += r * r.transpose() * (2.0 * w);
        g += r * (2.0 * w * (d - tgt));
    };
    add(&vs[n].0, vs[n].1, c.vx_target, c.w_v);
    add(&hs[n].0, hs[n].1, 0.0, c.w_h);
    for k in 1..n {
        add(&hs[k].0, hs[k].1, 0.0, c.w_h_run * dt);
    }
    for k in 0..n {
        let mut r = na::DVector::zeros(nu);
        r[k] = 1.0;
        add(&r, 0.0, 0.0, c.w_a * dt);
        if k > 0 {
            let mut r = na::DVector::zeros(nu);
            r[k] = 1.0;
            r[k - 1] = -1.0;
            add(&r, 0.0, 0.0, c.w_da);
            let mut r = na::DVector::zeros(nu);
            r[n + k] = 1.0;
            r[n + k - 1] = -1.0;
            add(&r, 0.0, 0.0, c.w_dp);
        }
    }
    // A u <= b
    let rows = 4 * n + 2 * n;
    let mut a = na::DMatrix::<f64>::zeros(rows, nu);
    let mut b = na::DVector::<f64>::zeros(rows);
    let mut i = 0;
    for k in 0..n {
        let amax = c.mu * fz[k].max(0.0) / m;
        a[(i, k)] = 1.0;
        b[i] = amax;
        a[(i + 1, k)] = -1.0;
        b[i + 1] = amax;
        a[(i + 2, n + k)] = 1.0;
        b[i + 2] = c.cop_hi;
        a[(i + 3, n + k)] = -1.0;
        b[i + 3] = -c.cop_lo;
        i += 4;
    }
    for k in 1..=n {
        let (r, d) = &xs[k];
        a.row_mut(i).copy_from(&r.transpose());
        b[i] = c.x_hi - d;
        a.row_mut(i + 1).copy_from(&(-r).transpose());
        b[i + 1] = d - c.x_lo;
        i += 2;
    }
    // The built-in interior point: the active set's cold Phase-1 reported
    // feasible plans infeasible (2026-11-03, "Infeasible after 0 iterations").
    let qp = misa_wbc::qp::QpConfig { solver: misa_wbc::qp::QpSolver::Ipm, max_iters: 200, ..Default::default() };
    let sol = misa_wbc::qp::solve_qp(&hm, &g, None, None, Some(&a), Some(&b), None, &qp);
    if sol.status != misa_wbc::qp::QpStatus::Optimal {
        return Err(format!("{:?} after {} iterations", sol.status, sol.iterations));
    }
    Ok(PushPlan {
        t0,
        dt,
        x0,
        v0,
        h0,
        m,
        a: sol.x.rows(0, n).iter().copied().collect(),
        p: sol.x.rows(n, n).iter().copied().collect(),
        zeta: zeta.to_vec(),
        fz: fz.to_vec(),
    })
}

/// The soles' mean yaw (rad), from the feet frames.
#[cfg(feature = "mujoco")]
/// x of the soles' midpoint (sole centres).
fn soles_mid_x(o_mi: &[misarta::se3::SE3<f64>], feet: &[usize; 2], sole_centre_x: f64) -> f64 {
    let sole = |k: usize| {
        let o = misarta::se3::translation(&o_mi[feet[k]]);
        let m = misarta::se3::rotation_matrix(&o_mi[feet[k]]);
        o[0] + m[(0, 0)] * sole_centre_x
    };
    0.5 * (sole(0) + sole(1))
}

/// A table per JUMP_V, "V|a:b,a:b,...;V|...", sorted by V and by a.
fn vtab(key: &str) -> Option<Vec<(f64, Vec<(f64, f64)>)>> {
    let s = std::env::var(key).ok()?;
    let mut t: Vec<(f64, Vec<(f64, f64)>)> = s
        .split(';')
        .map(|blk| {
            let (v, pts) = blk.split_once('|').unwrap_or_else(|| panic!("{key}: expected V|a:b,..., got {blk:?}"));
            let mut pts: Vec<(f64, f64)> = pts
                .split(',')
                .map(|p| {
                    let (a, b) = p.split_once(':').unwrap_or_else(|| panic!("{key}: expected a:b, got {p:?}"));
                    (a.trim().parse().unwrap(), b.trim().parse().unwrap())
                })
                .collect();
            pts.sort_by(|a, b| a.0.total_cmp(&b.0));
            (v.trim().parse().unwrap(), pts)
        })
        .collect();
    t.sort_by(|a, b| a.0.total_cmp(&b.0));
    Some(t)
}

/// The a at which the piecewise-linear b(a) (rising) reaches y, clamped.
fn inv_pw(pts: &[(f64, f64)], y: f64) -> f64 {
    let (mut lo, mut hi) = (pts[0], pts[pts.len() - 1]);
    if y <= lo.1 {
        return lo.0;
    }
    if y >= hi.1 {
        return hi.0;
    }
    for w in pts.windows(2) {
        if y <= w[1].1 {
            (lo, hi) = (w[0], w[1]);
            break;
        }
    }
    lo.0 + (hi.0 - lo.0) * (y - lo.1) / (hi.1 - lo.1)
}

fn soles_mean_yaw(o_mi: &[misarta::se3::SE3<f64>], feet: &[usize; 2]) -> f64 {
    let (mut sy, mut cy) = (0.0, 0.0);
    for &k in feet {
        let m = misarta::se3::rotation_matrix(&o_mi[k]);
        let y = m[(1, 0)].atan2(m[(0, 0)]);
        sy += y.sin();
        cy += y.cos();
    }
    sy.atan2(cy)
}

#[cfg(not(feature = "mujoco"))]
fn main() {
    eprintln!("This example requires the `mujoco` feature.");
}
