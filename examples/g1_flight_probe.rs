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
    let mut jump_t_crouch = env_f64("JUMP_T_CROUCH", 0.8).max(0.05);
    let mut jump_t_hold = env_f64("JUMP_T_HOLD", 0.3).max(0.0);
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
    // JUMP_VY (m/s, + = left) and JUMP_LEAN_Y (m): the sideways take-off
    // speed and the CoM's shift over the crouch, the lateral twins of JUMP_VX
    // and JUMP_LEAN (the push line tilts by JUMP_VY / JUMP_V in y). In the
    // heading frame when there is one. Both 0 (unset) = off, bit-identical.
    let jump_vy = env_f64("JUMP_VY", 0.0);
    let jump_lean_y = env_f64("JUMP_LEAN_Y", 0.0);
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
    let fwd_s_at = |dx: f64, v: f64| -> f64 {
        let tab = jump_fwd_tab.as_ref().expect("JUMP_FWD_TARGET needs JUMP_FWD_TAB");
        let ss: Vec<(f64, f64)> = tab.iter().map(|(tv, pts)| (*tv, inv_pw(pts, dx))).collect();
        interp(&ss, v)
    };
    let fwd_s_for = |dx: f64| -> f64 { fwd_s_at(dx, jump_v) };
    // JUMP_FWD_DROP_TAB ("h:dx,h:dx,...", unset = off): standing on the box
    // at the start (JUMP_BOX's front face behind the feet), the first jump
    // aims dx m (read off the box height h, piecewise-linear and clamped)
    // short of JUMP_FWD_TARGET -- the drop lengthens the flight past what
    // JUMP_FWD_TAB, measured on the flat, knows. Measured 2026-11-13 (jb82):
    // off a 10/20/30/40/50 cm box the first jump overshoots by a median
    // +0.03/+0.05/+0.07/+0.09/+0.10 m, and the backward correction hop that
    // follows is FY's weak point (28 of the 59 falls at 30-50 cm).
    let jump_fwd_drop = tab("JUMP_FWD_DROP_TAB").zip(jump_box.filter(|b| b.0 < -0.2)).map_or(0.0, |(t, b)| interp(&t, b.1));
    let mut fwd_s = jump_fwd_target.map_or(1.0, |dx| fwd_s_for(dx - jump_fwd_drop));
    if jump_fwd_drop != 0.0 {
        println!("jump forward drop: first jump aims {jump_fwd_drop:.3} m short");
    }
    let jump_fwd_tol = env_f64("JUMP_FWD_TOL", 0.05);
    let mut fwd_reached = false;
    // JUMP_LAT_TARGET (m, + = left, unset = off): the sideways twin of
    // JUMP_FWD_TARGET: aim the soles' midpoint this far to the side of where
    // it stood at T_LAUNCH. JUMP_VY is read backwards off JUMP_LAT_TAB
    // ("V|vy:dy,...;V|..."; the soles' sideways move one jump makes against
    // JUMP_VY, interpolated in JUMP_V), and with JUMP_REPEAT the jumps go on
    // until, at a re-arm, the soles are within JUMP_LAT_TOL m (default 0.05)
    // of the target. Measured 2026-11-15 (jb102-104, on TB6's tables times
    // 0.25): one jump moves the soles +-0.14 m at JUMP_VY +-0.6 and stands
    // 15-16 of 16; from 0.8 the push ends on the trailing foot alone (vy
    // 0.82 against 0.33) and lands on one sliding foot.
    let jump_lat_target = std::env::var("JUMP_LAT_TARGET").ok().map(|s| s.trim().parse::<f64>().unwrap());
    let jump_lat_tab = vtab("JUMP_LAT_TAB");
    let lat_vy_for = |dy: f64| -> f64 {
        let tab = jump_lat_tab.as_ref().expect("JUMP_LAT_TARGET needs JUMP_LAT_TAB");
        let ss: Vec<(f64, f64)> = tab.iter().map(|(tv, pts)| (*tv, inv_pw(pts, dy))).collect();
        interp(&ss, jump_v)
    };
    let jump_lat_tol = env_f64("JUMP_LAT_TOL", 0.05);
    let mut jvy = jump_lat_target.map_or(jump_vy, lat_vy_for);
    if let Some(dy) = jump_lat_target {
        println!("jump lateral target {dy:+.3} m: JUMP_VY {jvy:+.3}");
    }
    let mut lat_reached = false;
    let jump_lat = jump_vy != 0.0 || jump_lean_y != 0.0 || jump_lat_target.is_some();
    // JUMP_FWD_TARGET and JUMP_LAT_TARGET together: every re-arm checks both
    // (an axis within its tolerance jumps at JUMP_VY 0 / at the scale its
    // small remainder reads), and the jumps stop once both are within.
    let fwd_lat_both = jump_fwd_target.is_some() && jump_lat_target.is_some();
    // JUMP_PLANE_SEQ=1 (with both targets): jump the sideways distance first
    // at the scale JUMP_FWD_TAB reads for 0 m, then the forward one. Measured
    // 2026-11-15 (jb108): both at once, the backward hop (scale 0) with a
    // sideways push falls after the second landing (-0.3, +0.3: 3 / 7 of 16
    // at 1.5) and at 2.0 the full forward jump with JUMP_VY -0.6 falls on
    // the first landing (+0.5, -0.3: 7 / 12 of 16); each axis alone stands
    // 15-16 of 16.
    let plane_seq = fwd_lat_both && flag("JUMP_PLANE_SEQ", false);
    let mut plane_done = false;
    if plane_seq {
        fwd_s = fwd_s_for(0.0);
        println!("jump plane seq: sideways first at scale {fwd_s:.4}");
    }
    let (mut lat_feet0, mut lat_feet_last): (Option<f64>, f64) = (None, 0.0);
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
    // JUMP_BOX_S_TAB ("h:s,h:s,...", unset = off): JUMP_BOX_S read off the
    // box height (JUMP_BOX's second value), piecewise-linear and clamped at
    // the ends. Measured 2026-11-12 (jb79scr, HY on a 20 cm box, the 18
    // that landed at the edge): 1.2 stood 4, 1.35 stood 9, 1.5 stood 0
    // (the toe meets the box face).
    let jump_box_s = tab("JUMP_BOX_S_TAB").map_or(jump_box_s, |t| interp(&t, jump_box.map_or(0.0, |b| b.1)));
    // JUMP_BOX_TUCK=d (m, 0 = off): FLIGHT_FEET_TUCK added on the jump onto
    // the box only (JUMP_BOX_GAP). Measured 2026-11-08: with the tuck on
    // every jump, the short backward hops that close the gap fell (5 cm box
    // 0.20 m ahead, 2.0: 15/16 fell on the first hop).
    let jump_box_tuck = env_f64("JUMP_BOX_TUCK", 0.0);
    // JUMP_BOX_TUCK_TAB ("h:d,h:d,...", unset = off): JUMP_BOX_TUCK read
    // off the box height (JUMP_BOX's second value), piecewise-linear and
    // clamped at the ends like JUMP_VX_TAB.
    let jump_box_tuck = tab("JUMP_BOX_TUCK_TAB").map_or(jump_box_tuck, |t| interp(&t, jump_box.map_or(0.0, |b| b.1)));
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
    // JUMP_BOX_V (m/s, unset = JUMP_V) and JUMP_BOX_CROUCH (m, unset =
    // CROUCH): the push's take-off speed and the crouch depth of the jump
    // onto the box only (JUMP_VX and the tables stay those of JUMP_V; the
    // push line is tilted so vx is kept). Measured 2026-11-09 (jb14, BX on a
    // 7.5 / 10 cm box): lift-off vz 1.16 m/s at 1.5 (CoM up 6.9 cm) and 1.40
    // at 2.0 (10.0 cm); the toe hits the box face or corner.
    // JUMP_BOX_V_TAB ("h:v,h:v,...", unset = off): JUMP_BOX_V read off the
    // box height (JUMP_BOX's second value) in place of JUMP_BOX_V. Measured
    // 2026-11-11 (jb71, BH): 3.0 leaves the toe on the face of a 15 cm box
    // (34 / 39 of 96 stand), 3.5 stands 67 / 75 there but on a 12.5 cm box
    // the trunk lands pitched further forward, the rise brings the CoM back
    // behind the ankles and 1.5 drops from 79 to 62 of 96.
    let jump_box_v = tab("JUMP_BOX_V_TAB")
        .map(|t| interp(&t, jump_box.map_or(0.0, |b| b.1)))
        .or_else(|| std::env::var("JUMP_BOX_V").ok().map(|s| s.trim().parse::<f64>().unwrap()));
    let jump_box_crouch = std::env::var("JUMP_BOX_CROUCH").ok().map(|s| s.trim().parse::<f64>().unwrap());
    let crouch_base = crouch;
    // JUMP_BOX_V_TABS=1: read JUMP_VX_TAB and JUMP_LEAN_TAB at JUMP_BOX_V
    // too, for the jump onto the box. Measured 2026-11-09 (jb15, 10 cm box):
    // with the tables at JUMP_V 1.5 the push line tilts less as JUMP_BOX_V
    // rises (vx 0.60 / 0.50 / 0.39 m/s at JUMP_V / 2.5 / 3.0) and the heel
    // lands on the edge.
    let jump_box_v_tabs = flag("JUMP_BOX_V_TABS", false);
    let (box_vx, box_lean) = match jump_box_v {
        Some(v) if jump_box_v_tabs => (
            tab("JUMP_VX_TAB").map_or(jump_vx, |t| interp(&t, v)),
            tab("JUMP_LEAN_TAB").map_or(jump_lean, |t| interp(&t, v)),
        ),
        _ => (jump_vx, jump_lean),
    };
    // the take-off speed, forward speed and lean of the jump now (JUMP_V's
    // but for the jump onto the box)
    let (mut jv, mut jvx, mut jlean) = (jump_v, jump_vx, jump_lean);
    // JUMP_FWD_BACK_V (m/s, unset = off; with JUMP_FWD_TARGET): a jump with
    // the distance still to go behind the soles (rem < 0) takes off at this
    // speed in place of JUMP_V; JUMP_VX_TAB, JUMP_LEAN_TAB, CROUCH_TAB and
    // JUMP_FWD_TAB are read at it. Measured 2026-11-14 (RDB, 30-50 cm, the
    // backward correction hop after the drop): 3 of 183 fall at 1.5 and 20
    // of 178 at 2.0, those with lift-off vz 1.35 (standing 1.10) and h_y
    // +0.17 (+0.45); on the flat FY's -0.3 m stands 14 / 12 of 16 at 2.0.
    let jump_fwd_back = std::env::var("JUMP_FWD_BACK_V").ok().map(|s| {
        let v = s.trim().parse::<f64>().unwrap();
        let at = |name: &str, d: f64| tab(name).map_or(d, |t| interp(&t, v));
        (v, at("CROUCH_TAB", crouch_base), at("JUMP_VX_TAB", jump_vx), at("JUMP_LEAN_TAB", jump_lean))
    });
    if let (Some(b), Some(dx)) = (jump_fwd_back, jump_fwd_target)
        && dx - jump_fwd_drop < 0.0
        && !plane_seq
    {
        (jv, crouch, jvx, jlean) = b;
        fwd_s = fwd_s_at(dx - jump_fwd_drop, jv);
        println!("jump forward back: JUMP_V {jv} CROUCH {crouch:.4}: scale {fwd_s:.4} -> JUMP_VX {:.3} JUMP_LEAN {:.4}", jvx * fwd_s, jlean * fwd_s);
    }
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
    // KP/KD_TRUNK_LAND_BOX: the trunk gains after the landing of a jump onto
    // the box (JUMP_BOX; default = KP/KD_TRUNK_LAND). Measured 2026-11-10
    // (jb20, 5 cm box, JUMP_V 2.0): the box landing comes in pitched
    // 0.6-0.8 rad, the rise rights the trunk at -1.2 rad/s, throws the CoM
    // 5 cm behind the sole centre and the feet roll back on the heel edge
    // (11 of 12 falls); KP/KD_TRUNK_LAND 60/15 stood all 12 and 2.0 went
    // 51 -> 63/64, but on every landing it also changed the flat jumps
    // before the box (the box take-off at 1.5 fell 3 members short).
    let kp_trunk_land_box = env_f64("KP_TRUNK_LAND_BOX", kp_trunk_land);
    let kd_trunk_land_box = env_f64("KD_TRUNK_LAND_BOX", kd_trunk_land);
    // KD_TRUNK_LAND_BOX_TAB ("h:kd,h:kd,...", unset = off): KD_TRUNK_LAND_BOX
    // read off the box height (JUMP_BOX's second value), piecewise-linear and
    // clamped at the ends like JUMP_VX_TAB. Measured 2026-11-11 (jb76scr, TT
    // on a 20 cm box): of the 57 that landed past the edge and fell back in
    // the rise at KD 150, KD 200 stood 34 and 250 stood 40.
    let kd_trunk_land_box = tab("KD_TRUNK_LAND_BOX_TAB").map_or(kd_trunk_land_box, |t| interp(&t, jump_box.map_or(0.0, |b| b.1)));
    // KP/KD_TRUNK_LAND_DROP: the trunk gains after a landing at least
    // LAND_DROP_H m (default 0.05) below the surface the jump took off from
    // (needs LAND_GROUND_Z; default = KP/KD_TRUNK_LAND). Measured 2026-11-13
    // (jb80-81, jumping off a 30-40 cm box): the CoM sinks to 0.41-0.44 m
    // and, rising, runs 5-8 cm behind the sole centre and on off the heels
    // (34 of 128 stood); KP/KD_TRUNK_LAND 60/150 stood 111 of 128 but on
    // every landing it also changes the flat hops.
    let kp_trunk_land_drop = env_f64("KP_TRUNK_LAND_DROP", kp_trunk_land);
    let kd_trunk_land_drop = env_f64("KD_TRUNK_LAND_DROP", kd_trunk_land);
    // KP/KD_TRUNK_LAND_DROP_ROLL: the roll gains alone after such a landing
    // (default = KP/KD_TRUNK_LAND_DROP; needs TRUNK_RP_YAWED). Measured
    // 2026-11-14 (jb92): at 60/150 the landing roll of 0.2 rad decays over
    // ~2 s, the CoM rests over one foot, and a quick rise unloads the other.
    let kp_trunk_land_drop_roll = env_f64("KP_TRUNK_LAND_DROP_ROLL", kp_trunk_land_drop);
    let kd_trunk_land_drop_roll = env_f64("KD_TRUNK_LAND_DROP_ROLL", kd_trunk_land_drop);
    let land_drop_h = env_f64("LAND_DROP_H", 0.05);
    let mut drop_land = false;
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
        vx_target: env_f64("JUMP_MPC_VX", jvx * fwd_s),
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
    // LAND_HOLD_DROP / LAND_RISE_DROP (s, unset = LAND_HOLD / LAND_RISE): the
    // same, on a landing at least LAND_DROP_H below the surface the jump left
    // (as KP/KD_TRUNK_LAND_DROP). Measured 2026-11-14 (jb88, DD off 30-50 cm,
    // 1.5, toe 2 cm from the edge): the members that fall back keep the CoM
    // over the soles through the sink (-0.03 m at 0.6 s) and lose it in the
    // rise (0.41 -> 0.60 m by 1.1 s): the trunk pitches back +0.10 -> -0.13
    // rad and the CoM goes to -0.095 (standing ones -0.046); moving the
    // landing reference forward (LAND_REF_DROP_DX 0.03) leaves it at -0.095.
    let land_hold_drop = env_f64("LAND_HOLD_DROP", land_hold).max(0.0);
    let land_rise_drop = env_f64("LAND_RISE_DROP", land_rise).max(0.05);
    // LAND_RISE_DROP_H (m, unset = LAND_DROP_H): LAND_HOLD_DROP and
    // LAND_RISE_DROP only on a landing at least this far below the surface
    // the jump left. Measured 2026-11-14 (jb91, DD without the correction
    // hop, 2 seeds, 144 a height): HOLD 0.1 + RISE 0.7 stands 123 / 131 /
    // 131 / 130 / 128 off 10 / 20 / 30 / 40 / 50 cm against 135 / 134 / 131
    // / 123 / 114.
    let land_rise_drop_h = env_f64("LAND_RISE_DROP_H", land_drop_h);
    // LAND_RISE_DROP_DECEL (s, unset = off): on those landings the rise
    // speeds up over LAND_RISE_DROP / 2 as before but slows down over this
    // long (the two halves of rest-to-rest quintics, joined at the top speed).
    // Measured 2026-11-14 (jb92-93): with RISE 0.7 a foot unloads and lifts
    // 0.8-1.0 s after touchdown on 272 of 363 landings (DD 9), as the rise
    // brakes, and is set down wide (>0.28 m on 110) or turned; the hop from
    // there falls. Putting the roll gains back (KP/KD_TRUNK_LAND_DROP_ROLL)
    // takes the roll out but not the lift.
    let land_rise_drop_decel = std::env::var("LAND_RISE_DROP_DECEL").ok().map(|s| s.trim().parse::<f64>().unwrap().max(0.05));
    let mut drop_dz = 0.0_f64; // how far below the take-off surface the last first touchdown was
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
    // LAND_REF_SOLES=T (s, unset = off; needs JUMP_REPEAT for the stand
    // offset): from LAND_T after a touchdown, carry the landing reference's
    // horizontal part over T s (quintic weight) to the soles' current
    // midpoint plus the CoM's offset from it at the first jump. Measured
    // 2026-11-09 (jb16-17, JUMP_BOX_V on a 5-10 cm box): the reference is
    // c0 + v0 LAND_T / 2 and never looks at the feet, which land ahead of
    // the CoM on a box; the falls that land well past the edge drift back
    // (CoM - ankle origin +0.00 at 0.8 s, -0.05 to -0.09 at 1.5 s; standing
    // runs +0.03 to +0.08 and +-0.016) and fall backward.
    let land_ref_soles = std::env::var("LAND_REF_SOLES").ok().map(|s| s.trim().parse::<f64>().unwrap().max(0.02));
    // LAND_REF_SOLES_DROP=T (s, unset = LAND_REF_SOLES) and LAND_REF_DROP_DX=d
    // (m, default 0): the same, on a landing at least LAND_DROP_H below the
    // surface the jump left (as KP/KD_TRUNK_LAND_DROP), with the goal d m
    // further along world x. Measured 2026-11-14 (jb86, DD off 30-50 cm,
    // h_y < 0.3): the members that fall back and those that stand have the
    // CoM at the same place over the soles at touchdown and at the bottom
    // of the sink (-0.07, -0.01 m); 1.1 s later it is back at -0.084 against
    // -0.048. jb81 (LAND_REF_SOLES 0.5 on every landing, 30-40 cm): 34 -> 69
    // of 128 standing alone, 111 -> 116 with the trunk gains.
    let land_ref_soles_drop = std::env::var("LAND_REF_SOLES_DROP").ok().map(|s| s.trim().parse::<f64>().unwrap().max(0.02)).or(land_ref_soles);
    let land_ref_drop_dx = env_f64("LAND_REF_DROP_DX", 0.0);
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
    // JUMP_BOX_LAND_X_MIN=m (m, unset = off; needs JUMP_BOX): in flight on
    // the jump onto the box, hold each foot's target (the ankle origin) at
    // least m past the box's front face, once the CoM is coming down (from
    // lift-off on, the feet swing forward low and hit the box face). Measured 2026-11-12 (HY, 20 cm
    // box): the landings at the edge (ankle +0.00-0.06 past the face, 13 of
    // 17 fell) are the weak take-offs (vx 0.46-0.49) and the small-h_y ones
    // whose feet JUMP_BOX_CAPTURE_HY_TAB brought under the CoM. It does not
    // help (jb79scr, 18 of them: 4 stood, 0.07 / 0.08 / 0.10: 5 / 4 / 5):
    // the feet land further in but the CoM does not, 8-10 cm behind the
    // ankles, and the rise falls back. The CoM's travel is short.
    let jump_box_land_x_min = std::env::var("JUMP_BOX_LAND_X_MIN").ok().map(|s| s.trim().parse::<f64>().unwrap());
    // JUMP_DROP_CLEAR=c and JUMP_DROP_LAND_X_MIN=m (m, unset = off; need a
    // JUMP_BOX under the start): in flight from the box top (the surface
    // the robot took off from is the box), a foot whose heel (sole centre -
    // SOLE_HALF_L - JUMP_BOX_CLEAR_X) is not yet past the box's back edge
    // (x + len) is held with its sole at least c above the box top, and
    // each foot's target (the ankle origin) is held at least m past that
    // edge. Measured 2026-11-13 (DD, 30-50 cm): cutting the first jump's
    // target by its overshoot slows the take-off (vx 0.62 -> 0.49 at 1.5)
    // and the trailing foot lands on the box top (228 -> 48 standing).
    let jump_drop_clear = std::env::var("JUMP_DROP_CLEAR").ok().map(|s| s.trim().parse::<f64>().unwrap());
    let jump_drop_land_x_min = std::env::var("JUMP_DROP_LAND_X_MIN").ok().map(|s| s.trim().parse::<f64>().unwrap());
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
    // HOP_FEET_YAW_FROM=n (unset = off): FLIGHT_FEET_YAW and FLIGHT_TRUNK_YAW
    // only from jump n of a JUMP_REPEAT on. Measured 2026-11-16 (one-leg
    // hops, HL01, 5 sets): the stance foot's yaw at a re-arm predicts the
    // next hop's fall (jump 3: +0.26 fell vs +0.06 stood); most of it comes
    // from the first flight (+0.13 while the pelvis untwists), and holding
    // it there twists the first landing (HYL01, 7 first-hop falls).
    let hop_yaw_from = std::env::var("HOP_FEET_YAW_FROM").ok().map(|s| s.parse::<usize>().expect("HOP_FEET_YAW_FROM"));
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
    // JUMP_BOX_CAPTURE_TAB ("h:k,h:k,...", unset = off): FLIGHT_FEET_CAPTURE
    // on the jump onto the box only, read off the box height (JUMP_BOX's
    // second value), piecewise-linear and clamped at the ends. Measured
    // 2026-11-12 (TK, 20 cm box): the box landing comes in with the ankles
    // 4 cm ahead of the CoM, the legs 25 cm short of standing, and the
    // members with a small lift-off pitch momentum (h_y <= 0.6) cannot
    // carry the CoM over the feet in the rise and fall back (27 of 48).
    let jump_box_capture = tab("JUMP_BOX_CAPTURE_TAB").map(|t| interp(&t, jump_box.map_or(0.0, |b| b.1)));
    // JUMP_BOX_CAPTURE_HY_TAB ("hy:k,hy:k,...", unset = off): on the jump
    // onto a box at least JUMP_BOX_CAPTURE_HY_H m high (default 0), the
    // FLIGHT_FEET_CAPTURE read off the pitch momentum h_y at lift-off
    // (piecewise-linear, clamped at the ends), in place of the above.
    // Measured 2026-11-12 (jb77, TK on a 20 cm box, landings past the edge):
    // k 0.55 / 0.25 fell 24/44 / 0/40 of the members with h_y <= 0.6 but
    // 5/20 / 10/13 of those with h_y >= 1.0; one k cannot serve both.
    let jump_box_capture_hy = tab("JUMP_BOX_CAPTURE_HY_TAB").filter(|_| jump_box.map_or(0.0, |b| b.1) >= env_f64("JUMP_BOX_CAPTURE_HY_H", 0.0));
    // JUMP_DROP_CAPTURE_HY_TAB ("hy:k,hy:k,...", unset = off; needs a
    // JUMP_BOX under the start): the same, on a jump from the box top (the
    // surface the robot took off from is the box). Measured 2026-11-14
    // (jb86, DD without the correction hop, 30-50 cm): the members that fall
    // back after the first landing are the ones with a small h_y at
    // lift-off (1.5, toe 2 cm from the edge: 23 fell, h_y median 0.205, all
    // below 0.3; 38 stood, 0.272), not the slow ones (touchdown vx 0.50
    // either way).
    let jump_drop_capture_hy = tab("JUMP_DROP_CAPTURE_HY_TAB").filter(|_| jump_box.is_some_and(|b| b.0 < -0.2));
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
    let hop_mom_z_w = env_f64("HOP_MOM_Z_W", 0.0).max(0.0);
    let jump_fz_sym_w = env_f64("JUMP_FZ_SYM_W", 0.0).max(0.0);
    let foot_reseat_w = env_f64("FOOT_RESEAT_W", 1.0);
    let mut reseat_xy: [Option<na::Vector2<f64>>; 2] = [None, None];
    let mut foot_stand_z: Option<[f64; 2]> = None;
    let kp_ffr = env_f64("KP_FLIGHT_FEET_ROT", 400.0);
    let kd_ffr = env_f64("KD_FLIGHT_FEET_ROT", 40.0);
    // HOP_LEG=L|R (unset = off): hop on that leg alone. From HOP_SHIFT_T0
    // the CoM reference moves over HOP_SHIFT_T s to the stance sole's centre
    // (HOP_COM_DY m further toward the free side); then the free foot
    // leaves the contact set for good and rises HOP_LIFT_H m over
    // HOP_LIFT_T s, sole level (task "hop foot", weight HOP_FOOT_W, gains
    // KP/KD_HOP_FOOT), and is held at that offset from the stance foot
    // outside flight (in flight the flight-feet task holds it). The free
    // foot's lost load never counts as a release (RELEASE_FIRST_OFF). Set
    // T_LAUNCH after the lift. Off = bit-identical.
    let mut hop_leg: Option<usize> = std::env::var("HOP_LEG").ok().map(|s| match s.as_str() {
        "L" | "l" => 0,
        "R" | "r" => 1,
        _ => panic!("HOP_LEG must be L or R"),
    });
    let mut hop_free = hop_leg.map(|s| 1 - s);
    let hop_shift_t0 = env_f64("HOP_SHIFT_T0", 0.3);
    let hop_shift_t = env_f64("HOP_SHIFT_T", 1.0);
    let hop_lift_t = env_f64("HOP_LIFT_T", 0.5);
    let hop_lift_h = env_f64("HOP_LIFT_H", 0.08);
    let hop_com_dy = env_f64("HOP_COM_DY", 0.0);
    // HOP_CROUCH_DY (m, with HOP_LEG, unset = 0): in the crouch and push of
    // every re-armed hop (not the first, not the landing or flight), the CoM
    // reference moves this far toward the free side (laid out with the
    // crouch like JUMP_LEAN_Y). Measured 2026-11-16 (HL01, 5 sets): the free
    // foot rests on the floor through the crouch (~30 N, 22 cm inward,
    // unmodelled), the effective CoP sits ~2 cm inward of the stance sole's
    // centre and the stance CoP pins on its outer edge; the first crouch has
    // the CoM 1.4 cm inward and holds, the re-armed ones start at the sole's
    // centre and drift out (plants lighter than the model launch outward).
    // HOP_COM_DY 0.01 fixes the crouch for every mass but also places the
    // landing foot and the landing reference, and the falls move there.
    let hop_crouch_dy = env_f64("HOP_CROUCH_DY", 0.0);
    let hop_foot_w = env_f64("HOP_FOOT_W", 1.0);
    let kp_hop = env_f64("KP_HOP_FOOT", 400.0);
    let kd_hop = env_f64("KD_HOP_FOOT", 40.0);
    // KP_HOP_FOOT_Z: the lifted foot's height gain alone (unset = KP_HOP_FOOT).
    // Measured 2026-11-15 (JUMP_REPEAT): at HOP_FOOT_W 0.03 the foot follows
    // the posture down in the crouch and rests on the floor; in a repeated
    // hop it scuffs (fz ~40 N) and rolls the stance sole onto its edge.
    let kp_hop_z = env_f64("KP_HOP_FOOT_Z", kp_hop);
    // HOP_PUSH_PELVIS=1: during the push (after the crouch and the hold, up
    // to the release), hold the lifted foot where it was in the PELVIS frame
    // at the push start (task "hop foot push", weight HOP_PUSH_FOOT_W, unset =
    // HOP_FOOT_W). Measured 2026-11-15 (JUMP_REPEAT): with the foot held in
    // the world at weight 0.03 it swings 9 cm forward in the push (h_z
    // +0.69 from the lifted leg), the pelvis turns the other way and a hop
    // lands with the pelvis and the held foot yaw 0.46 rad apart.
    let hop_push_pelvis = flag("HOP_PUSH_PELVIS", false);
    let hop_push_foot_w = env_f64("HOP_PUSH_FOOT_W", hop_foot_w);
    let mut hop_rel_pelvis: Option<na::Vector3<f64>> = None;
    // HOP_TRUNK_YAW=w (with HOP_LEG, 0 = off): on the ground from the start
    // (not only after a landing like STANCE_TRUNK_YAW), a yaw-only pelvis
    // task toward the STANCE sole's yaw (not the two soles' mean: the lifted
    // foot's yaw is free). Measured 2026-11-15: in the one-leg crouch the
    // pelvis turns 0.08 rad off the stance foot before the push.
    // HOP_MOM_LAND_W=w (with HOP_LEG): the momentum weight on the ground for
    // HOP_MOM_LAND_T s (default 2) from each touchdown (default = WBCW_MOMENTUM).
    // Measured 2026-11-15 (JUMP_REPEAT): after a one-leg landing the CoP sits
    // on the sole's inner edge and the CoM walks over it; the CoM task asks
    // +11 m/s^2 outward and gets ~0, the momentum task (10) forbidding the
    // hip strategy that is all that is left.
    let hop_mom_land_w = std::env::var("HOP_MOM_LAND_W").ok().map(|s| s.parse::<f64>().expect("HOP_MOM_LAND_W"));
    let hop_mom_land_t = env_f64("HOP_MOM_LAND_T", 2.0);
    // HOP_COP_CENTRE=k (1/s, unset/0 = off; with HOP_LEG): on the one stance
    // foot (not in the push, not in the first HOP_COP_T0 s after a
    // touchdown), move the CoM reference across the sole at k times the
    // stance CoP's lateral offset from the sole's centre, the other way;
    // the offset (clamped to HOP_COP_MAX m) carries over from hop to hop.
    // Measured 2026-11-16 (HL01, 5 sets): from the second hop on the CoP
    // sits on the sole's outer edge in the crouch (+2.7 cm, the scuffing
    // free foot carrying load inward) and the foot rolls out 0.08 rad;
    // plants lighter than the model fall out of it (112/175 against
    // 223/225). A fixed HOP_COM_DY 0.01 rescues the light ones (151) and
    // tips the heavy ones inward (194).
    let hop_cop_k = if hop_leg.is_some() { env_f64("HOP_COP_CENTRE", 0.0).max(0.0) } else { 0.0 };
    let hop_cop_t0 = env_f64("HOP_COP_T0", 0.3);
    let hop_cop_max = env_f64("HOP_COP_MAX", 0.03);
    let mut hop_cop_off = na::Vector2::<f64>::zeros();
    let hop_trunk_yaw = if hop_leg.is_some() { env_f64("HOP_TRUNK_YAW", 0.0).max(0.0) } else { 0.0 };
    let hop_t_lift = hop_shift_t0 + hop_shift_t;
    let hop_foot_com = flag("HOP_FOOT_COM", false);
    let hop_flight_y = flag("HOP_FLIGHT_Y", false);
    // HOP_FLIGHT_TOE=rad (with HOP_LEG and FLIGHT_FEET_YAW): in flight, turn
    // the stance foot to this toe-out yaw from the START heading (world yaw
    // 0) instead of holding its lift-off yaw. Measured 2026-11-15 (JUMP_REPEAT,
    // 347 landings without FLIGHT_FEET_YAW): toe-out -0.1..+0.2 at touchdown
    // never fell (250), toe-in past -0.1 fell 20 of 42; the free yaw lands
    // toe-out ~+0.1 on the first hop and then drifts.
    let hop_flight_toe = std::env::var("HOP_FLIGHT_TOE").ok().map(|s| s.parse::<f64>().expect("HOP_FLIGHT_TOE"));
    // HOP_FLIGHT_Y_CAPTURE: the lateral capture gain of HOP_FLIGHT_Y (unset
    // = FLIGHT_FEET_CAPTURE). Measured 2026-11-15 (JUMP_REPEAT): at 0.55 a
    // hop that lands with vy 0.2 m/s puts the sole 2.4 cm past the CoM where
    // 5 cm was needed; the CoP sits on the sole's edge and the CoM walks
    // over it in ~1 s.
    let hop_flight_y_capture = env_f64("HOP_FLIGHT_Y_CAPTURE", flight_feet_capture);
    // HOP_ALT=1 (with HOP_LEG): alternate the legs, the first step toward
    // running. At every lift-off the lifted foot becomes the stance foot:
    // in flight it comes down to the old stance foot's lift-off height and
    // the old stance foot goes HOP_LIFT_H up (over FLIGHT_FEET_CAPTURE_T);
    // after the touchdown the new lifted foot is held where it is relative
    // to the new stance foot. HOP_ALT_VY (m/s): lateral lift-off speed
    // toward the lifted foot (the push line tilts as with JUMP_VY).
    // HOP_ALT_LEAN_Y (m): the CoM's shift toward the lifted foot over every
    // crouch, the first one too. HOP_ALT_W (m, 0 = off): in flight, carry
    // the lifted foot to this far beside the new stance foot (same x)
    // instead of keeping its lift-off offset. Measured 2026-11-19: the
    // lifted foot swings ~15 cm ahead in the push, the stance foot's x
    // capture then took it 15 cm behind, and the second landing put the
    // feet 8 cm apart. The first lift (HOP_LIFT_T) also brings the lifted
    // foot to this width. Off = bit-identical.
    let hop_alt = hop_leg.is_some() && flag("HOP_ALT", false);
    let hop_alt_w = env_f64("HOP_ALT_W", 0.0);
    let hop_alt_vy = env_f64("HOP_ALT_VY", 0.0);
    let hop_alt_lean_y = env_f64("HOP_ALT_LEAN_Y", 0.0);
    let mut hop_alt_z: Option<[f64; 2]> = None; // flight targets of the feet's z offsets from the CoM
    let mut hop_alt_rel_pending = false;
    // HOP_RUN=1 (with HOP_ALT): alternate without stopping. After a real
    // flight (>= 0.05 s) the next jump re-arms HOP_RUN_AFTER s after the
    // touchdown, with its crouch (HOP_RUN_T_CROUCH, hold HOP_RUN_T_HOLD)
    // starting from the CoM's velocity there (z and x: the dip absorbs the
    // landing). The stance's lateral reference is planned at the re-arm:
    // the LIPM along the dip and the push (omega^2 = (g + z'') / height,
    // integrated), about one CoP held within HOP_RUN_COP of the stance sole
    // centre and chosen to put the lift-off DCM HOP_RUN_XI toward the
    // lifted foot; no lean, the push tilted by HOP_RUN_VY (default 0)
    // instead of HOP_ALT_VY. In flight the new stance foot's sole centre
    // goes HOP_RUN_DCM outward of the lateral capture point, at least
    // HOP_RUN_MIN_W from where the old stance foot left, and to
    // HOP_RUN_CAPTURE_X times the fore-aft one. Only the first flight after
    // a push swaps the feet. Measured 2026-11-20: stopping on one foot after
    // the lateral carry had a +-1 cm window that the friction moved (§55);
    // the dip and the push grow a DCM offset ~13x over a 0.45 s stance, so
    // the touchdown DCM must sit within about -1.4..+2.3 cm of the sole.
    // Off = bit-identical.
    let hop_run = hop_alt && flag("HOP_RUN", false);
    let hop_run_dcm = env_f64("HOP_RUN_DCM", 0.02);
    let hop_run_min_w = env_f64("HOP_RUN_MIN_W", 0.10);
    let hop_run_vy = env_f64("HOP_RUN_VY", 0.0);
    let hop_run_xi = env_f64("HOP_RUN_XI", 0.09);
    let hop_run_t_crouch = env_f64("HOP_RUN_T_CROUCH", 0.25).max(0.05);
    let hop_run_t_hold = env_f64("HOP_RUN_T_HOLD", 0.0).max(0.0);
    // HOP_RUN_AFTER (s): re-arm this long after the touchdown. Measured
    // 2026-11-20: from the touchdown velocity the impact took vz -1.13 ->
    // -0.82 in 20 ms, the dip's PD asked -25 m/s^2 to catch the plan up and
    // the stance foot's load fell 1273 -> 174 N.
    let hop_run_after = env_f64("HOP_RUN_AFTER", 0.04).max(0.0);
    // HOP_RUN_CAPTURE_X: the stance foot's fore-aft capture gain in flight
    // (FLIGHT_FEET_CAPTURE otherwise). Measured 2026-11-20: at 0.55 the hops
    // went on at a steady -0.36 m/s.
    let hop_run_capture_x = env_f64("HOP_RUN_CAPTURE_X", 1.0);
    let hop_run_cop = env_f64("HOP_RUN_COP", 0.02);
    let mut hop_run_v0 = na::Vector3::<f64>::zeros(); // CoM velocity at the re-arm
    let mut hop_run_p = 0.0_f64; // stance sole centre y at the re-arm
    let mut hop_run_plan: Vec<(f64, f64, f64)> = vec![(0.0, 0.0, 0.0)]; // stance lateral plan per tick: y - y0, vy, ay
    let mut hop_run_old_y: Option<f64> = None; // old stance foot y at the lift-off
    let hop_land_ref = std::env::var("HOP_LAND_REF").ok().and_then(|s| s.parse::<f64>().ok());
    let mut hop_rel_com: Option<na::Vector3<f64>> = None;
    let hop_kp_com_y = env_f64("HOP_KP_COM_Y", env_f64("KP_COM", 300.0));
    let hop_kd_com_y = env_f64("HOP_KD_COM_Y", env_f64("KD_COM", 80.0));
    let mut hop_c0: Option<(na::Vector2<f64>, na::Vector2<f64>)> = None; // CoM ref at the shift start, goal
    let mut hop_p0: Option<na::Vector3<f64>> = None; // free foot at the lift start
    let mut hop_rel: Option<na::Vector3<f64>> = None; // free foot - stance foot after the lift
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
    // With HOP_LEG the heading is the stance sole's yaw alone.
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
    let mut lift_off_hy = 0.0_f64; // h_y at the last lift-off
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
        let (flight_feet_yaw, flight_trunk_yaw) =
            if hop_yaw_from.is_some_and(|n| repeats_done + 1 < n) { (false, 0.0) } else { (flight_feet_yaw, flight_trunk_yaw) };
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
        if jump_lat {
            let y = soles_mid_y(&st.data.oMi, &feet, prof.sole_centre_x);
            lat_feet_last = y;
            if t >= t_launch {
                lat_feet0.get_or_insert(y);
            }
        }
        if diag_fwd_feet {
            let x = soles_mid_x(&st.data.oMi, &feet, prof.sole_centre_x);
            fwd_feet_last = x;
            if t >= t_launch {
                if fwd_feet0.is_none() && jump_box_gap.is_some() {
                    let (gap, s, onto) = box_gap_s(front_toe_x(&st.data.oMi));
                    (fwd_s, box_jump_now) = (s, onto);
                    (jv, crouch, jvx, jlean) = if box_jump_now { (jump_box_v.unwrap_or(jump_v), jump_box_crouch.unwrap_or(crouch_base), box_vx, box_lean) } else { (jump_v, crouch_base, jump_vx, jump_lean) };
                    push_cfg.vx_target = env_f64("JUMP_MPC_VX", jvx * fwd_s);
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
        if let Some(fr) = hop_free && t >= hop_t_lift {
            if in_contact[fr] {
                anchors.release(fr);
                let p = misarta::se3::translation(&data.oMi[feet[fr]]);
                println!("  [hop-lift] t={t:.3} free foot {} out of the contact set at ({:+.3},{:+.3},{:+.3}), com ({:+.3},{:+.3}) fz {:.0} {:.0}", if fr == 0 { "L" } else { "R" }, p.x, p.y, p.z, com.x, com.y, measured.cop[0][2], measured.cop[1][2]);
            }
            in_contact[fr] = false;
            flip_count[fr] = 0;
        }
        // a one-leg hop (HOP_LEG) re-arms on the stance foot alone
        let rearm_contact = match hop_leg { Some(sl) => in_contact[sl], None => in_contact[0] && in_contact[1] };
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
        // HOP_RUN: HOP_RUN_AFTER after a real flight (a 2 ms one in the
        // crouch re-armed the jump)
        let rearm_after = if hop_run && flight_spans.last().is_some_and(|&(a, b)| b - a >= 0.05) { hop_run_after } else { jump_repeat_after };
        if let Some(tgt) = jump_turn_target
            && !turn_reached
            && jump_repeat > repeats_done && jump_v > 0.0 && !air_sched && flight_t0.is_none() && rearm_contact
            && let Some((tl, _, _)) = land
            && t - tl >= rearm_after
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
            && (!fwd_reached || (fwd_lat_both && !plane_done))
            && jump_repeat > repeats_done && jump_v > 0.0 && !air_sched && flight_t0.is_none() && rearm_contact
            && let Some((tl, _, _)) = land
            && t - tl >= rearm_after
            && let Some(x0) = fwd_feet0
        {
            let done = soles_mid_x(&data.oMi, &feet, prof.sole_centre_x) - x0;
            let rem_x = tgt - done;
            if fwd_lat_both {
                fwd_reached = rem_x.abs() < jump_fwd_tol;
            }
            // JUMP_PLANE_SEQ: the sideways axis first, at the scale that
            // moves the soles 0 in x
            let lat_pending = plane_seq
                && jump_lat_target.zip(lat_feet0).is_some_and(|(ty, y0)| (ty - (soles_mid_y(&data.oMi, &feet, prof.sole_centre_x) - y0)).abs() >= jump_lat_tol);
            let rem = if lat_pending { 0.0 } else { rem_x };
            if rem.abs() < jump_fwd_tol && !fwd_lat_both {
                fwd_reached = true;
                println!("  [fwd-target] t={t:.3} moved {done:+.3} m, {rem:+.3} to go: no more jumps");
            } else if let Some(b) = jump_fwd_back {
                (jv, crouch, jvx, jlean) = if rem < 0.0 { b } else { (jump_v, crouch_base, jump_vx, jump_lean) };
                fwd_s = fwd_s_at(rem, jv);
                if !jump_mpc_vx_set {
                    push_cfg.vx_target = jvx * fwd_s;
                }
                println!("  [fwd-target] t={t:.3} moved {done:+.3} m, {rem:+.3} to go: scale {fwd_s:.4} at JUMP_V {jv}");
            } else {
                fwd_s = fwd_s_for(rem);
                if !jump_mpc_vx_set {
                    push_cfg.vx_target = jump_vx * fwd_s;
                }
                println!("  [fwd-target] t={t:.3} moved {done:+.3} m, {rem:+.3} to go: scale {fwd_s:.4}");
            }
        }
        if let Some(tgt) = jump_lat_target
            && (!lat_reached || (fwd_lat_both && !plane_done))
            && jump_repeat > repeats_done && jump_v > 0.0 && !air_sched && flight_t0.is_none() && rearm_contact
            && let Some((tl, _, _)) = land
            && t - tl >= rearm_after
            && let Some(y0) = lat_feet0
        {
            let done = soles_mid_y(&data.oMi, &feet, prof.sole_centre_x) - y0;
            let rem = tgt - done;
            if rem.abs() < jump_lat_tol {
                lat_reached = true;
                if fwd_lat_both {
                    jvy = 0.0;
                    println!("  [lat-target] t={t:.3} moved {done:+.3} m, {rem:+.3} to go: reached, JUMP_VY 0");
                } else {
                    println!("  [lat-target] t={t:.3} moved {done:+.3} m, {rem:+.3} to go: no more jumps");
                }
            } else {
                lat_reached = false;
                jvy = lat_vy_for(rem);
                println!("  [lat-target] t={t:.3} moved {done:+.3} m, {rem:+.3} to go: JUMP_VY {jvy:+.3}");
            }
        }
        // both targets: checked once per re-arm, done once both are within
        if fwd_lat_both && fwd_reached && lat_reached {
            plane_done = true;
        }
        if let (Some(_), Some((_, bh, _))) = (jump_box_gap, jump_box)
            && !fwd_reached
            && jump_repeat > repeats_done && jump_v > 0.0 && !air_sched && flight_t0.is_none() && rearm_contact
            && let Some((tl, _, _)) = land
            && t - tl >= rearm_after
        {
            if ground_dz > 0.5 * bh {
                fwd_reached = true;
                println!("  [box-gap] t={t:.3} on the box ({ground_dz:+.3} m): no more jumps");
            } else {
                let (gap, s, onto) = box_gap_s(front_toe_x(&data.oMi));
                (fwd_s, box_jump_now) = (s, onto);
                (jv, crouch, jvx, jlean) = if box_jump_now { (jump_box_v.unwrap_or(jump_v), jump_box_crouch.unwrap_or(crouch_base), box_vx, box_lean) } else { (jump_v, crouch_base, jump_vx, jump_lean) };
                if !jump_mpc_vx_set {
                    push_cfg.vx_target = jvx * fwd_s;
                }
                println!("  [box-gap] t={t:.3} toe to box {gap:+.3} m: scale {fwd_s:.4}");
            }
        }
        if jump_repeat > repeats_done && !turn_reached
            && !(fwd_reached && (jump_lat_target.is_none() || lat_reached))
            && !(lat_reached && (jump_fwd_target.is_none() || fwd_reached))
            && jump_v > 0.0 && !air_sched && flight_t0.is_none() && rearm_contact
            && let Some((tl, _, _)) = land
            && t - tl >= rearm_after
        {
            repeats_done += 1;
            if land_ground_z && let Some(f0) = foot_stand_z {
                ground_dz = 0.5 * (0..2).map(|k| misarta::se3::translation(&data.oMi[feet[k]]).z - f0[k]).sum::<f64>();
                stand_z = stand_z0 + ground_dz;
                println!("  [ground] t={t:.3} re-arm on {ground_dz:+.4} m above the start");
            }
            let mut r = diag_com_cmd.map_or(com, |(c, _)| c);
            if hop_cop_k > 0.0 {
                // the commanded reference carries HOP_COP_CENTRE's offset,
                // which is added again on top of the base
                r.x -= hop_cop_off.x;
                r.y -= hop_cop_off.y;
                println!("  [hop-cop] t={t:.3} offset ({:+.4},{:+.4})", hop_cop_off.x, hop_cop_off.y);
            }
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
                // a one-leg hop (HOP_LEG) goes along the stance sole alone:
                // the lifted foot's yaw is free (0.2-0.3 rad off by the
                // third hop, HOP_CROUCH_DY 0.02)
                let (sy, cy) = match hop_leg {
                    Some(sl) => { let y = if sl == 0 { y0 } else { y1 }; (y.sin(), y.cos()) }
                    None => (y0.sin() + y1.sin(), y0.cos() + y1.cos()),
                };
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
            if hop_run && let Some(sl) = hop_leg {
                // the dip from the landing speed: a crouch from rest this
                // short asks more than g down and the stance foot flickers off
                jump_t_crouch = hop_run_t_crouch;
                jump_t_hold = hop_run_t_hold;
                hop_run_v0 = com_vel;
                let ps = if sl == 0 { s0.y } else { s1.y };
                // this stance's vertical plan (dip from the touchdown
                // velocity, hold, push) and the lateral LIPM along it,
                // y'' = (g + z'') / (z - floor) (y - p)
                let z_low = stand_z - crouch;
                let floor = foot_stand_z.map_or(0.0, |f| f[sl] + ground_dz) - prof.sole_below_origin;
                let dzp = stand_z + jump_z_to - z_low;
                let rr = jump_push_r;
                let (t_push, a0) = if rr < 1.0 {
                    let tp = 3.0 * dzp * (1.0 + rr) / ((2.0 + rr) * jv);
                    (tp, 2.0 * jv / ((1.0 + rr) * tp))
                } else {
                    let a = jv * jv / (2.0 * dzp);
                    (jv / a, a)
                };
                let kk = if rr < 1.0 { (1.0 - rr) / t_push } else { 0.0 };
                let t_s = jump_t_crouch + jump_t_hold + t_push;
                let n = (t_s / dt).ceil() as usize;
                // basis solutions A (y 1, v 0) and B (y 0, v 1), and w^2
                let mut basis = Vec::with_capacity(n + 1);
                let (mut a, mut ad, mut b, mut bd) = (1.0, 0.0, 0.0, 1.0);
                for i in 0..=n {
                    let s = i as f64 * dt;
                    let (z, az) = if s < jump_t_crouch {
                        let (z, _, az) = quintic(r.z, com_vel.z, z_low, jump_t_crouch, s);
                        (z, az)
                    } else if s < jump_t_crouch + jump_t_hold {
                        (z_low, 0.0)
                    } else {
                        let sp = (s - jump_t_crouch - jump_t_hold).min(t_push);
                        (z_low + a0 * (0.5 * sp * sp - kk * sp.powi(3) / 6.0), a0 * (1.0 - kk * sp))
                    };
                    let w2 = (G + az).max(0.0) / (z - floor).max(0.3);
                    basis.push((a, ad, b, bd, w2));
                    ad += w2 * a * dt;
                    a += ad * dt;
                    bd += w2 * b * dt;
                    b += bd * dt;
                }
                // the CoP that puts the lift-off DCM (nominal omega)
                // HOP_RUN_XI toward the lifted foot, within the sole
                let w0 = (G / (stand_z - floor).max(0.3)).sqrt();
                let (ae, ade, be, bde, _) = basis[n];
                let (ka, kb) = (ae + ade / w0, be + bde / w0);
                let toward = if sl == 0 { -1.0 } else { 1.0 };
                let (y0, v0) = (r.y, com_vel.y);
                let p = ((ps + toward * hop_run_xi - ka * y0 - kb * v0) / (1.0 - ka)).clamp(ps - hop_run_cop, ps + hop_run_cop);
                hop_run_p = ps;
                hop_run_plan = basis
                    .iter()
                    .map(|&(a, ad, b, bd, w2)| {
                        let y = p + a * (y0 - p) + b * v0;
                        (y - y0, ad * (y0 - p) + bd * v0, w2 * (y - p))
                    })
                    .collect();
                let &(ye, vye, _) = hop_run_plan.last().unwrap();
                println!(
                    "  [run] t={t:.3} re-arm on {}: com y - sole {:+.4} vy {:+.3} vz {:+.3} dcm - sole {:+.4} | stance {t_s:.3} s, cop - sole {:+.4}, plan lift-off y - sole {:+.4} vy {:+.3}",
                    if sl == 0 { "L" } else { "R" }, y0 - ps, v0, com_vel.z, y0 + v0 / w0 - ps, p - ps, y0 + ye - ps, vye
                );
            }
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
                if flight_feet_yaw && let (Some(sl), Some(toe)) = (hop_leg, hop_flight_toe) {
                    feet_yaw0[sl] = if sl == 0 { toe } else { -toe };
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
                lift_off_hy = h[1];
                let (r, pch, _) = st.body_quat.euler_angles();
                println!("  [lift-off] t={t:.3} com ({:+.3}, z {:.4}) v ({:+.3}, {:+.3}), foot x {:?} z {:?}, h_ang ({:+.3},{:+.3},{:+.3}), roll {r:+.3} pitch {pch:+.3}", com.x, com.z, com_vel.x, com_vel.z, fx, fz, h[0], h[1], h[2]);
                // HOP_RUN: only the first flight after a push swaps (a bounce
                // in the dip or a fall would swap the feet back)
                let alt_swap = !hop_run || (touchdown_t.is_none() && t >= t_launch + jump_t_crouch + jump_t_hold);
                if hop_alt && alt_swap && let (Some(sl), Some(fr), Some(off)) = (hop_leg, hop_free, feet_off) {
                    if hop_run {
                        hop_run_old_y = Some(misarta::se3::translation(&data.oMi[feet[sl]]).y);
                        let (vy, h) = (com_vel.y, (stand_z - (fz_stand[0].min(fz_stand[1]) - prof.sole_below_origin)).max(0.3));
                        println!("  [run] t={t:.3} lift-off {}: com y - sole {:+.4} vy {:+.3} dcm - sole {:+.4}", if sl == 0 { "L" } else { "R" }, com.y - hop_run_p, vy, com.y + vy / (G / h).sqrt() - hop_run_p);
                    }
                    let mut zt = [0.0; 2];
                    zt[fr] = off[sl].z;
                    zt[sl] = off[sl].z + hop_lift_h;
                    hop_alt_z = Some(zt);
                    hop_leg = Some(fr);
                    hop_free = Some(sl);
                    hop_p0 = None;
                    hop_rel = None;
                    hop_rel_com = None;
                    hop_alt_rel_pending = true;
                    let fy: Vec<String> = (0..2).map(|k| format!("{:+.3}", misarta::se3::translation(&data.oMi[feet[k]]).y)).collect();
                    println!("  [hop-alt] t={t:.3} stance {} -> {}, com y {:+.3} vy {:+.3}, foot y {:?}", if sl == 0 { "L" } else { "R" }, if fr == 0 { "L" } else { "R" }, com.y, com_vel.y, fy);
                }
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
                        let gz = dz.iter().sum::<f64>() / dz.len() as f64;
                        drop_land = gz - ground_dz < -land_drop_h;
                        drop_dz = ground_dz - gz;
                        ground_dz = gz;
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
        if let Some(sl) = hop_leg && touchdown_t.is_none() && t < t_launch && t >= hop_shift_t0 {
            let (c0, goal) = *hop_c0.get_or_insert_with(|| {
                let o = misarta::se3::translation(&data.oMi[feet[sl]]);
                let m = misarta::se3::rotation_matrix(&data.oMi[feet[sl]]);
                let c = (o + m * na::Vector3::new(prof.sole_centre_x, 0.0, 0.0)).xy();
                let dy = if sl == 0 { -hop_com_dy } else { hop_com_dy };
                let c0 = com_ref.unwrap_or(com).xy();
                println!("  [hop-shift] t={t:.3} stance {} sole centre ({:+.3},{:+.3}): com ref ({:+.3},{:+.3}) -> ({:+.3},{:+.3}) over {hop_shift_t:.2} s", if sl == 0 { "L" } else { "R" }, c.x, c.y, c0.x, c0.y, c.x, c.y + dy);
                (c0, na::Vector2::new(c.x, c.y + dy))
            });
            let (w, _, _) = quintic(0.0, 0.0, 1.0, hop_shift_t, t - hop_shift_t0);
            let r = com_ref.get_or_insert(com);
            r.x = c0.x + w * (goal.x - c0.x);
            r.y = c0.y + w * (goal.y - c0.y);
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
                if hop_leg.is_some() && flag("DIAG_HOP", false) {
                    // the same split for h_z (the hop's lift-off yaw)
                    let mut gz = [0.0_f64; 5];
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
                        gz[g] += cmm[(2, c)] * v[c];
                    }
                    println!("  [hz] lift-off h_z {:+.3} | base {:+.3} Lleg {:+.3} Rleg {:+.3} waist {:+.3} arms {:+.3}", gz.iter().sum::<f64>(), gz[0], gz[1], gz[2], gz[3], gz[4]);
                }
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
                    let deep = drop_land && drop_dz > land_rise_drop_h;
                    let (lh, lr) = if deep { (land_hold_drop, land_rise_drop) } else { (land_hold, land_rise) };
                    let s_rise = s - land_t - lh;
                    if s_rise > 0.0 {
                        let z_low = c0.z + 0.5 * v0.z * land_t;
                        let (p, vv, aa) = if deep && let Some(td) = land_rise_drop_decel {
                            let ta = 0.5 * lr;
                            let d1 = (stand_z - z_low) * ta / (ta + td);
                            let d2 = stand_z - z_low - d1;
                            if s_rise < ta {
                                quintic(z_low, 0.0, z_low + 2.0 * d1, 2.0 * ta, s_rise)
                            } else {
                                quintic(stand_z - 2.0 * d2, 0.0, stand_z, 2.0 * td, td + s_rise - ta)
                            }
                        } else {
                            quintic(z_low, 0.0, stand_z, lr, s_rise)
                        };
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
                    if let (Some(tb), Some(off)) = (if drop_land { land_ref_soles_drop } else { land_ref_soles }, stand_off) && s > land_t {
                        let mut goal = 0.5 * (sole(0) + sole(1)) + off;
                        if drop_land {
                            goal.x += land_ref_drop_dx;
                        }
                        let (w, dw, ddw) = quintic(0.0, 0.0, 1.0, tb, s - land_t);
                        for i in 0..2 {
                            let e = goal[i] - c[i];
                            c[i] += w * e;
                            cd[i] = (1.0 - w) * cd[i] + dw * e;
                            cdd[i] = (1.0 - w) * cdd[i] + ddw * e;
                        }
                    }
                    // HOP_LAND_REF (s, with HOP_LEG): from the touchdown, carry
                    // the reference's horizontal part to the stance sole's
                    // centre (HOP_COM_DY toward the free side) over this time.
                    if let (Some(sl), Some(tb)) = (hop_leg, hop_land_ref) {
                        let mut goal = sole(sl);
                        goal.y += if sl == 0 { -hop_com_dy } else { hop_com_dy };
                        let (w, dw, ddw) = quintic(0.0, 0.0, 1.0, tb, s);
                        for i in 0..2 {
                            let e = goal[i] - c[i];
                            c[i] += w * e;
                            cd[i] = (1.0 - w) * cd[i] + dw * e;
                            cdd[i] = (1.0 - w) * cdd[i] + ddw * e;
                        }
                    }
                    (c, cd, cdd)
                }
                None if jump_v > 0.0 && t >= t_launch && push_end.is_none() => {
                    // crouch, hold, push off
                    let z_low = stand_z - crouch;
                    let s = t - t_launch;
                    let run_now = hop_run && repeats_done > 0;
                    let run_v0 = if run_now { hop_run_v0 } else { na::Vector3::zeros() };
                    let (mut z, mut vz, mut az) = quintic(crouch_z0.unwrap_or(stand_z), run_v0.z, z_low, jump_t_crouch, s);
                    if jump_repeat_recentre_n >= 2 && repeats_done > 0 && heading.is_none() && let Some(off) = stand_off {
                        // live target: the shift from the soles' midpoint now
                        let sole = |k: usize| {
                            let o = misarta::se3::translation(&data.oMi[feet[k]]);
                            let m = misarta::se3::rotation_matrix(&data.oMi[feet[k]]);
                            (o + m * na::Vector3::new(prof.sole_centre_x, 0.0, 0.0)).xy()
                        };
                        rearm_shift = 0.5 * (sole(0) + sole(1)) + off - c_ref.xy();
                    }
                    let lean_x = jlean * fwd_s + rearm_shift.x;
                    let (mut x, mut vx, mut ax) = quintic(0.0, run_v0.x, lean_x, jump_t_crouch, s);
                    let mut lean_y = jump_lean_y + rearm_shift.y;
                    if hop_crouch_dy != 0.0 && repeats_done > 0 && let Some(sl) = hop_leg {
                        lean_y += if sl == 0 { -hop_crouch_dy } else { hop_crouch_dy };
                    }
                    // HOP_ALT: toward the lifted foot (+y for the left one)
                    let alt_sign = match hop_leg { Some(0) => -1.0, _ => 1.0 };
                    if hop_alt {
                        lean_y += alt_sign * hop_alt_lean_y;
                    }
                    let (mut y, mut vy, mut ay) = quintic(0.0, 0.0, lean_y, jump_t_crouch, s);
                    // HOP_RUN: the stance plan made at the re-arm (y relative
                    // to the re-arm CoM), straight on at its end speed after
                    let run_y = |s: f64| -> (f64, f64, f64) {
                        let n = hop_run_plan.len() - 1;
                        let i = ((s / dt).round() as usize).min(n);
                        let (y, vy, ay) = hop_run_plan[i];
                        if i < n { (y, vy, ay) } else { (y + vy * (s - n as f64 * dt).max(0.0), vy, 0.0) }
                    };
                    if run_now {
                        lean_y = 0.0;
                        (y, vy, ay) = run_y(s);
                    }
                    let ky = if run_now { alt_sign * hop_run_vy / jv } else if hop_alt { alt_sign * hop_alt_vy / jv } else { jvy / jv };
                    let jump_lat = jump_lat || hop_alt;
                    let kx = jvx * fwd_s / jv;
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
                            let t_push = 3.0 * dz * (1.0 + r) / ((2.0 + r) * jv);
                            let a0 = 2.0 * jv / ((1.0 + r) * t_push);
                            let sp = s_push.min(t_push);
                            let k = (1.0 - r) / t_push;
                            z = z_low + a0 * (0.5 * sp * sp - k * sp.powi(3) / 6.0);
                            vz = a0 * (sp - 0.5 * k * sp * sp);
                            az = a0 * (1.0 - k * sp);
                            (t_push, a0)
                        } else {
                            let a = jv * jv / (2.0 * dz);
                            z = z_low + 0.5 * a * s_push * s_push;
                            vz = a * s_push;
                            az = a;
                            (jv / a, a)
                        };
                        push_phase = s_push < t_push;
                        x += kx * (z - z_low);
                        vx += kx * vz;
                        ax += kx * az;
                        if jump_lat {
                            y += ky * (z - z_low);
                            vy += ky * vz;
                            ay += ky * az;
                        }
                        if s_push >= t_push && jump_release {
                            let (t0, z0, v0) = *release_t0.get_or_insert((t, z, vz));
                            let sr = t - t0;
                            z = z0 + v0 * sr - 0.5 * G * sr * sr;
                            vz = v0 - G * sr;
                            az = -G;
                            x = lean_x + kx * (z0 - z_low) + kx * v0 * sr;
                            vx = kx * v0;
                            ax = 0.0;
                            if jump_lat && run_now {
                                let (yl, vyl, ayl) = run_y(s);
                                y = yl + ky * (z0 - z_low) + ky * v0 * sr;
                                vy = vyl + ky * v0;
                                ay = ayl;
                            } else if jump_lat {
                                y = lean_y + ky * (z0 - z_low) + ky * v0 * sr;
                                vy = ky * v0;
                                ay = 0.0;
                            }
                            let unloaded = measured.cop[0][2] <= f_on && measured.cop[1][2] <= f_on;
                            let one_off = release_first_off && hop_free.is_none() && !unloaded && (measured.cop[0][2] <= f_on || measured.cop[1][2] <= f_on);
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
                            let tp = 3.0 * dz * (1.0 + r) / ((2.0 + r) * jv);
                            (tp, 2.0 * jv / ((1.0 + r) * tp), (1.0 - r) / tp)
                        } else {
                            let a = jv * jv / (2.0 * dz);
                            (jv / a, a, 0.0)
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
            if hop_cop_k > 0.0 && let (Some(sl), Some(fr)) = (hop_leg, hop_free) && !flight {
                let pushing = jump_v > 0.0 && t >= t_launch + jump_t_crouch + jump_t_hold && push_end.is_none();
                let settled = touchdown_t.is_none_or(|t0| t - t0 >= hop_cop_t0);
                let m = misarta::se3::rotation_matrix(&data.oMi[feet[sl]]);
                let side = na::Vector2::new(m[(0, 1)], m[(1, 1)]);
                if !pushing && settled && in_contact[sl] && !in_contact[fr] && measured.cop[sl][2] > f_on {
                    hop_cop_off -= side * (hop_cop_k * measured.cop[sl][1] * dt);
                    let n = hop_cop_off.norm();
                    if n > hop_cop_max {
                        hop_cop_off *= hop_cop_max / n;
                    }
                }
                // eased in over HOP_COP_T0 after a touchdown (the landing
                // reference starts at the touchdown CoM)
                let a = touchdown_t.map_or(1.0, |t0| ((t - t0) / hop_cop_t0).clamp(0.0, 1.0));
                c_ref.x += a * hop_cop_off.x;
                c_ref.y += a * hop_cop_off.y;
            }
            let a_com = cdd_ref + kd_com * (cd_ref - com_vel) + kp_com * (c_ref - com);
            // HOP_KP/KD_COM_Y (with HOP_LEG; default = KP/KD_COM): the CoM's
            // lateral gains on the one stance foot.
            let a_com = if hop_leg.is_some() {
                let mut a = a_com;
                a.y = cdd_ref.y + hop_kd_com_y * (cd_ref.y - com_vel.y) + hop_kp_com_y * (c_ref.y - com.y);
                a
            } else {
                a_com
            };
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
            let (kp_t, kd_t) = if hard_land && box_jump_now {
                (kp_trunk_land_box, kd_trunk_land_box)
            } else if hard_land && drop_land {
                (kp_trunk_land_drop, kd_trunk_land_drop)
            } else if hard_land {
                (kp_trunk_land, kd_trunk_land)
            } else {
                (kp_trunk, kd_trunk)
            };
            let drop_roll = hard_land && !box_jump_now && drop_land && (kp_trunk_land_drop_roll != kp_t || kd_trunk_land_drop_roll != kd_t);
            let rp_ref = if trunk_rp_yawed && drop_roll {
                // roll and pitch about the heading, each with its own gains
                let (sy, cy) = yaw_now.sin_cos();
                let (wr, wp) = (cy * st.v_ang_w[0] + sy * st.v_ang_w[1], -sy * st.v_ang_w[0] + cy * st.v_ang_w[1]);
                let er = -kp_trunk_land_drop_roll * roll - kd_trunk_land_drop_roll * wr;
                let ep = -kp_t * pitch - kd_t * wp;
                [cy * er - sy * ep, sy * er + cy * ep]
            } else if trunk_rp_yawed {
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
        let hop_yaw = !flight && hop_trunk_yaw > 0.0 && !stance_yaw;
        if (flight && flight_trunk_yaw > 0.0) || stance_yaw || hop_yaw {
            let tgt = if flight {
                trunk_yaw0
            } else if hop_yaw && let Some(sl) = hop_leg {
                let m = misarta::se3::rotation_matrix(&data.oMi[feet[sl]]);
                m[(1, 0)].atan2(m[(0, 0)])
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
            names.push(if hop_yaw { "trunk yaw hop" } else { "trunk yaw" });
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
                        let t_push = if r < 1.0 { 3.0 * dz * (1.0 + r) / ((2.0 + r) * jv) } else { 2.0 * dz / jv };
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
                let t_push = if r < 1.0 { 3.0 * dz * (1.0 + r) / ((2.0 + r) * jv) } else { 2.0 * dz / jv };
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
            // HOP_MOM_Z_W (with HOP_LEG): during the push only, a YAW-only
            // momentum task toward 0 with this weight. Measured 2026-11-15:
            // a hop leaves with h_z +-0.15 (the lifted leg swings 9 cm
            // forward, +0.69), lands with the feet's yaw held
            // (FLIGHT_FEET_YAW) and the low-friction members slip on the
            // landing twist.
            if hop_leg.is_some() && hop_mom_z_w > 0.0 && in_push_now
                && let Some(tk) = bt::angular_momentum(dyn_ctx.qddot(), &cmm, &dcmm_v, &h_now, &[0.0; 3], &[0.0; 3], kp_mom, [false, false, true], nv)
            {
                levels.push(tk);
                names.push("momentum yaw");
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
            let flight_feet_capture = if box_jump_now {
                jump_box_capture_hy.as_ref().map(|t| interp(t, lift_off_hy)).or(jump_box_capture).unwrap_or(flight_feet_capture)
            } else if let Some(t) = jump_drop_capture_hy.as_ref() && ground_dz > -0.02 {
                interp(t, lift_off_hy)
            } else {
                flight_feet_capture
            };
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
                } else if let Some(sl) = hop_leg {
                    // one-leg hop: the stance foot to the capture point, the
                    // lifted foot shifted with it (not swung to the same x)
                    let k = if hop_run { hop_run_capture_x } else { flight_feet_capture };
                    let tgt = k * com_vel.x / (G / h).sqrt() - prof.sole_centre_x;
                    let d = (tgt - off[sl].x) * b;
                    for o in off.iter_mut() {
                        o.x += d;
                    }
                } else {
                    let tgt = flight_feet_capture * com_vel.x / (G / h).sqrt() - prof.sole_centre_x;
                    for o in off.iter_mut() {
                        o.x += (tgt - o.x) * b;
                    }
                }
            }
            // HOP_FLIGHT_Y=1 (with HOP_LEG): in flight, carry the stance
            // foot's lateral offset to FLIGHT_FEET_CAPTURE times the CoM's
            // lateral capture offset (vy / omega) over FLIGHT_FEET_CAPTURE_T,
            // the lifted foot along with it.
            if hop_flight_y && let (Some(sl), Some(t0f)) = (hop_leg, flight_t0) {
                let h = (stand_z - (fz_stand[0].min(fz_stand[1]) - prof.sole_below_origin)).max(0.3);
                let b = ((t - t0f) / flight_feet_capture_t).clamp(0.0, 1.0);
                let b = b * b * (3.0 - 2.0 * b);
                let mut tgt = hop_flight_y_capture * com_vel.y / (G / h).sqrt() + if sl == 0 { hop_com_dy } else { -hop_com_dy };
                if hop_run && let Some(oy) = hop_run_old_y {
                    // outward of the capture point, clear of the old foot
                    tgt = if sl == 0 { (tgt + hop_run_dcm).max(oy + hop_run_min_w - com.y) } else { (tgt - hop_run_dcm).min(oy - hop_run_min_w - com.y) };
                    // the sole centre, not the foot origin (a yawed foot
                    // carries its sole centre sideways)
                    let m = misarta::se3::rotation_matrix(&data.oMi[feet[sl]]);
                    tgt -= m[(1, 0)] * prof.sole_centre_x;
                }
                let d = (tgt - off[sl].y) * b;
                for o in off.iter_mut() {
                    o.y += d;
                }
                if hop_run && tick % 10 == 0 {
                    let p = misarta::se3::translation(&data.oMi[feet[sl]]);
                    println!("  [run-air] t={t:.3} stance-to-be y target {:+.4} at {:+.4} (com {:+.4} vy {:+.3})", com.y + off[sl].y, p.y, com.y, com_vel.y);
                }
            }
            if let (Some(zt), Some(t0f)) = (hop_alt_z, flight_t0) {
                let b = ((t - t0f) / flight_feet_capture_t).clamp(0.0, 1.0);
                let b = b * b * (3.0 - 2.0 * b);
                for (k, o) in off.iter_mut().enumerate() {
                    o.z += (zt[k] - o.z) * b;
                }
                if hop_alt_w > 0.0 && let (Some(sl), Some(fr)) = (hop_leg, hop_free) {
                    // HOP_ALT_W: the lifted foot beside the new stance foot
                    let y = off[sl].y + if fr == 0 { hop_alt_w } else { -hop_alt_w };
                    off[fr].x += (off[sl].x - off[fr].x) * b;
                    off[fr].y += (y - off[fr].y) * b;
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
                if box_jump_now && com_vel.z < 0.0 && let (Some(m), Some((bx, _, _))) = (jump_box_land_x_min, jump_box) {
                    p_tgt.x = p_tgt.x.max(bx + m);
                }
                if let Some((bx, bh, bl)) = jump_box && bx < -0.2 && ground_dz > -0.02 {
                    if let Some(c) = jump_drop_clear && pos.x + prof.sole_centre_x - sole_half_l - jump_box_clear_x < bx + bl {
                        p_tgt.z = p_tgt.z.max(bh + c + prof.sole_below_origin);
                    }
                    if let Some(m) = jump_drop_land_x_min {
                        p_tgt.x = p_tgt.x.max(bx + bl + m);
                    }
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
                if in_contact[k] || step_st.is_some_and(|x| x.0 == k) || hop_free == Some(k) {
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
        if let (Some(sl), Some(fr)) = (hop_leg, hop_free) && !flight && t >= hop_t_lift {
            let jf = misarta::jacobian::compute_joint_jacobian_from_data(&rig.model, q, data, feet[fr]);
            let djv = &misarta::jacobian::compute_joint_jacobian_time_derivative(&rig.model, q, v, feet[fr]) * v_dvec;
            let pos = misarta::se3::translation(&data.oMi[feet[fr]]);
            let vel3 = &jf.rows(3, 3).into_owned() * v_dvec;
            let vel = na::Vector3::new(vel3[0], vel3[1], vel3[2]);
            let p_st = misarta::se3::translation(&data.oMi[feet[sl]]);
            if hop_alt_rel_pending {
                // HOP_ALT: the old stance foot, lifted in flight, is held
                // where it touched down relative to the new stance foot
                hop_alt_rel_pending = false;
                hop_p0 = Some(pos);
                hop_rel = Some(pos - p_st);
                println!("  [hop-alt-free] t={t:.3} free foot {} at ({:+.3},{:+.3},{:+.3}), rel ({:+.3},{:+.3},{:+.3})", if fr == 0 { "L" } else { "R" }, pos.x, pos.y, pos.z, pos.x - p_st.x, pos.y - p_st.y, pos.z - p_st.z);
            }
            let p0 = *hop_p0.get_or_insert(pos);
            let s = t - hop_t_lift;
            let hop_alt_dy = if hop_alt && hop_alt_w > 0.0 { p_st.y + if fr == 0 { hop_alt_w } else { -hop_alt_w } - p0.y } else { 0.0 };
            let hop_in_push = hop_push_pelvis && jump_v > 0.0 && push_end.is_none() && t >= t_launch + jump_t_crouch + jump_t_hold;
            if !hop_in_push {
                hop_rel_pelvis = None;
            }
            let (tgt, tgt_v) = if hop_in_push {
                let pp = misarta::se3::translation(&data.oMi[1]);
                let rp = misarta::se3::rotation_matrix(&data.oMi[1]);
                let rel = *hop_rel_pelvis.get_or_insert_with(|| {
                    let rel = rp.transpose() * (pos - pp);
                    println!("  [hop-push-pelvis] t={t:.3} free foot at ({:+.3},{:+.3},{:+.3}), in the pelvis frame ({:+.3},{:+.3},{:+.3})", pos.x, pos.y, pos.z, rel.x, rel.y, rel.z);
                    rel
                });
                let jp = misarta::jacobian::compute_joint_jacobian_from_data(&rig.model, q, data, 1);
                let tw = &jp * v_dvec;
                let (w_p, v_p) = (na::Vector3::new(tw[0], tw[1], tw[2]), na::Vector3::new(tw[3], tw[4], tw[5]));
                let r = rp * rel;
                (pp + r, v_p + w_p.cross(&r))
            } else if s < hop_lift_t && hop_rel.is_none() {
                let (h, dh, _) = quintic(0.0, 0.0, hop_lift_h, hop_lift_t, s);
                // HOP_ALT_W: the first lift also brings the foot to that
                // width beside the stance foot
                let k = h / hop_lift_h;
                (p0 + na::Vector3::new(0.0, k * hop_alt_dy, h), na::Vector3::new(0.0, dh / hop_lift_h * hop_alt_dy, dh))
            } else {
                let rel = *hop_rel.get_or_insert_with(|| {
                    let rel = p0 + na::Vector3::new(0.0, hop_alt_dy, hop_lift_h) - p_st;
                    println!("  [hop-lifted] t={t:.3} free foot at ({:+.3},{:+.3},{:+.3}) (target rel ({:+.3},{:+.3},{:+.3})), com ({:+.3},{:+.3}) stance sole ({:+.3},{:+.3})", pos.x, pos.y, pos.z, rel.x, rel.y, rel.z, com.x, com.y, p_st.x + prof.sole_centre_x, p_st.y);
                    rel
                });
                if hop_foot_com {
                    // HOP_FOOT_COM=1: the lifted foot's offset from the CoM
                    // instead (it comes down with the crouch)
                    let rc = *hop_rel_com.get_or_insert(p_st + rel - com);
                    (com + rc, com_vel)
                } else {
                    (p_st + rel, na::Vector3::zeros())
                }
            };
            let rot = misarta::se3::rotation_matrix(&data.oMi[feet[fr]]);
            let yaw = rot[(1, 0)].atan2(rot[(0, 0)]);
            let rot_tgt = *na::Rotation3::from_axis_angle(&na::Vector3::z_axis(), yaw).matrix();
            let w3 = &jf.rows(0, 3).into_owned() * v_dvec;
            let omega = na::Vector3::new(w3[0], w3[1], w3[2]);
            levels.push(bt::swing_with_pose(
                dyn_ctx.qddot(), &jf, &djv, &pos, &vel, &tgt, &tgt_v,
                kp_hop, kp_hop_z, kd_hop, bt::SwingAxes::XyzRollPitch,
                Some((rot, rot_tgt, omega, kp_ffr, kd_ffr)),
            ));
            names.push(if hop_in_push { "hop foot push" } else { "hop foot" });
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
                    "hop foot" => hop_foot_w,
                    "hop foot push" => hop_push_foot_w,
                    "trunk yaw hop" => hop_trunk_yaw,
                    "momentum roll" => jump_mom_x_w,
                    "momentum yaw" => hop_mom_z_w,
                    "fz sym" => jump_fz_sym_w,
                    // WBCW_MOMENTUM_PUSH: the momentum weight during the push
                    // and the release only (default = WBCW_MOMENTUM).
                    "momentum" if hop_mom_land_w.is_some() && hop_leg.is_some() && !flight && touchdown_t.is_some_and(|t0| t - t0 < hop_mom_land_t) => {
                        hop_mom_land_w.unwrap()
                    }
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

        // DIAG_HOP=1 (with HOP_LEG): every 20 ms from the shift on, the
        // lateral state on the stance foot.
        if let Some(sl) = hop_leg && flag("DIAG_HOP", false) && tick % 10 == 0 && t >= hop_shift_t0 && touchdown_t.is_none_or(|t0| t - t0 <= 1.5) {
            let a_sol = &st.j_com * &ex.qddot + st.djv_com;
            let (cr, ac) = diag_com_cmd.unwrap_or((com, na::Vector3::zeros()));
            let cmm = misarta::centroidal::compute_centroidal_momentum_matrix(&rig.model, q);
            let h = &cmm * v_dvec;
            let m = misarta::se3::rotation_matrix(&data.oMi[feet[sl]]);
            let (r, _, _) = na::Rotation3::from_matrix_unchecked(m).euler_angles();
            let o = misarta::se3::translation(&data.oMi[feet[sl]]);
            let side = if sl == 0 { "left" } else { "right" };
            let tq = |part: &str| -> String {
                rig.actuated()
                    .iter()
                    .find(|&&(ji, _)| rig.robot.joints[ji].name == format!("{side}_{part}_joint"))
                    .map_or("?".into(), |&(_, vi)| format!("{:+.0}/{:.0}", ex.tau[vi - 6], rig.torque_max[vi - 6]))
            };
            println!(
                "  [hop] t={t:.3} nc={nc} {} com y {:+.4} ref {:+.4} vy {:+.3} ay cmd {:+.2} sol {:+.2} | z {:.3} vz {:+.3} az cmd {:+.2} sol {:+.2} | h_x {:+.3} roll {:+.3} | sole y {:+.4} roll {r:+.3} cop ({:+.3},{:+.3}) fz {:.0} | hip roll {} ankle roll {} ankle pitch {} | free z {:+.3} tgt {:+.3}",
                if flight { "AIR" } else if push_phase { "PSH" } else { "GND" },
                com.y, cr.y, com_vel.y, ac.y, a_sol[1], com.z, com_vel.z, ac.z, a_sol[2], h[0], roll, o.y, measured.cop[sl][0], measured.cop[sl][1], measured.cop[sl][2],
                tq("hip_roll"), tq("ankle_roll"), tq("ankle_pitch"),
                misarta::se3::translation(&data.oMi[feet[1 - sl]]).z, hop_rel.map_or(f64::NAN, |rel| o.z + rel.z)
            );
            let mut use_: Vec<(f64, String)> = rig
                .actuated()
                .iter()
                .map(|&(ji, vi)| (ex.tau[vi - 6].abs() / rig.torque_max[vi - 6], format!("{} {:+.0}/{:.0}", rig.robot.joints[ji].name.trim_end_matches("_joint"), ex.tau[vi - 6], rig.torque_max[vi - 6])))
                .collect();
            use_.sort_by(|a, b| b.0.partial_cmp(&a.0).unwrap());
            println!("  [hop-tau] t={t:.3} {}", use_.iter().take(4).map(|(u, s)| format!("{s} ({u:.2})")).collect::<Vec<_>>().join(", "));
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
    if let Some(y0) = lat_feet0 {
        let d = lat_feet_last - y0;
        match jump_lat_target {
            Some(tgt) => println!("  lateral target {tgt:+.3} m (last JUMP_VY {jvy:+.3}): soles moved {d:+.3} m at the end (error {:+.3}), {} jumps, reached {lat_reached}", d - tgt, repeats_done + 1),
            None => println!("  lateral: soles moved {d:+.3} m at the end, {} jumps", repeats_done + 1),
        }
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
fn soles_mid_y(o_mi: &[misarta::se3::SE3<f64>], feet: &[usize; 2], sole_centre_x: f64) -> f64 {
    let sole = |k: usize| {
        let o = misarta::se3::translation(&o_mi[feet[k]]);
        let m = misarta::se3::rotation_matrix(&o_mi[feet[k]]);
        o[1] + m[(1, 0)] * sole_centre_x
    };
    0.5 * (sole(0) + sole(1))
}

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
