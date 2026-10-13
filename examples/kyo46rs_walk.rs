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
#[cfg(feature = "mujoco")]
use nalgebra as na;

/// A recovery reference from scripts/trajopt/push_recovery.py (JSON): phase
/// durations, node times, sole-centre trajectories and the CoM, all in the
/// optimiser's frame (left foot stance at y = `stance_y0`, push toward +y).
/// `REF_LIBRARY=<file>` loads one; see doc/from_claude/20261005_trajopt_reference_plan.md §9.
#[cfg(feature = "mujoco")]
#[derive(Clone)]
struct RefLib {
    phases: Vec<f64>,
    swing_z0: f64,
    /// Swing phase (fraction of the single support) this reference is meant for.
    swing_phase: f64,
    /// Direction of the push this reference answers [deg]: 90 = toward the stance foot, 0 = forward.
    impulse_deg: f64,
    /// Solved from the LANDED state (the swing foot on the ground, unloaded):
    /// the re-step reference, engaged by RESTEP and never by the detectors.
    landed: bool,
    t: Vec<f64>,
    sole: [Vec<na::Vector3<f64>>; 2],
    /// CoM at the nodes, and the vertical force of each foot per interval.
    com: Vec<na::Vector3<f64>>,
    fz: [Vec<f64>; 2],
    stance_y0: f64,
}

#[cfg(feature = "mujoco")]
impl RefLib {
    fn load(path: &str) -> RefLib {
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
            phases: v["T"].as_array().unwrap().iter().map(f).collect(),
            t: v["t"].as_array().unwrap().iter().map(f).collect(),
            sole: [sole_l, sole_r],
            stance_y0,
        }
    }
    fn total(&self) -> f64 {
        self.phases.iter().sum()
    }
    fn interp(&self, xs: &[na::Vector3<f64>], tau: f64) -> na::Vector3<f64> {
        let t = &self.t;
        if tau <= t[0] {
            return xs[0];
        }
        for k in 0..t.len() - 1 {
            if tau <= t[k + 1] {
                let a = ((tau - t[k]) / (t[k + 1] - t[k]).max(1e-9)).clamp(0.0, 1.0);
                return xs[k] * (1.0 - a) + xs[k + 1] * a;
            }
        }
        *xs.last().unwrap()
    }
    /// Reference DCM (xy) at `tau`: c + c_dot / omega, c_dot by finite difference.
    fn dcm_at(&self, tau: f64, omega: f64) -> na::Vector2<f64> {
        let h = 0.01;
        let c = self.interp(&self.com, tau);
        let cd = (self.interp(&self.com, tau + h) - self.interp(&self.com, (tau - h).max(0.0))) / (h + (tau - (tau - h).max(0.0)));
        (c + cd / omega).xy()
    }
    /// Reference net CoP (xy) at `tau`: the soles weighted by their vertical force.
    fn cop_at(&self, tau: f64) -> na::Vector2<f64> {
        let t = &self.t;
        let mut k = 0usize;
        while k + 1 < self.fz[0].len() && tau > t[k + 1] {
            k += 1;
        }
        let (fl, fr) = (self.fz[0][k].max(0.0), self.fz[1][k].max(0.0));
        let (pl, pr) = (self.sole_at(0, tau), self.sole_at(1, tau));
        if fl + fr < 1.0 {
            return (0.5 * (pl + pr)).xy();
        }
        ((pl * fl + pr * fr) / (fl + fr)).xy()
    }
    /// Sole centre of reference foot `side` at reference time `tau`, linearly
    /// interpolated between nodes and held at the ends.
    fn sole_at(&self, side: usize, tau: f64) -> na::Vector3<f64> {
        let t = &self.t;
        if tau <= t[0] {
            return self.sole[side][0];
        }
        for k in 0..t.len() - 1 {
            if tau <= t[k + 1] {
                let a = ((tau - t[k]) / (t[k + 1] - t[k]).max(1e-9)).clamp(0.0, 1.0);
                return self.sole[side][k] * (1.0 - a) + self.sole[side][k + 1] * a;
            }
        }
        *self.sole[side].last().unwrap()
    }
}

/// An engaged reference: when it started, which walker foot plays the
/// reference's LEFT (stance) foot, and the frame that maps the reference into
/// the world (origin = measured stance sole at engagement, `mirror` = -1 when
/// the stance is the walker's right foot).
#[cfg(feature = "mujoco")]
#[derive(Clone, Copy)]
struct RefMode {
    t0: f64,
    /// Reference time already elapsed at engagement (the detector fires
    /// some tens of ms after the push; the reference starts at the push).
    tau0: f64,
    stance: usize,
    origin: na::Vector3<f64>,
    mirror: f64,
}

#[cfg(feature = "mujoco")]
impl RefMode {
    fn world(&self, lib: &RefLib, p: na::Vector3<f64>) -> na::Vector3<f64> {
        self.origin + na::Vector3::new(p.x, self.mirror * (p.y - lib.stance_y0), p.z)
    }
    fn world_xy(&self, lib: &RefLib, p: na::Vector2<f64>) -> na::Vector2<f64> {
        self.world(lib, na::Vector3::new(p.x, p.y, 0.0)).xy()
    }
    /// Walker foot index that plays reference foot `ref_side` (0 = its stance).
    fn foot(&self, ref_side: usize) -> usize {
        if ref_side == 0 { self.stance } else { 1 - self.stance }
    }
}

fn main() {
    use articara::biped::actuate::{
        CommandPolicy, DegradedTally, gravity_plus_posture, write_to_plant,
    };
    use articara::biped::contact::{Anchors, contact_jacobians, cop_from_sole_wrench};
    use articara::biped::dcm::{
        DcmPlan, StepTiming, SupportBox, com_accel_xy, commanded_zmp, dcm_of, step_timing,
    };
    use articara::biped::gait::{
        ContactCorrection, ContactPhase, FootstepPlan, Footsteps, GaitParams, GaitPlan, Support,
        swing_position, swing_velocity,
    };
    use articara::biped::log::{Row, TrajLog, measure_contacts};
    use articara::biped::profile;
    use articara::biped::rig::{BipedRig, CtrlMode, G, RigOptions};
    use articara::biped::tasks as bt;
    use misa_wbc::{AsAffine, Dynamics, Formulation, SolveConfig, Solver, Task, tasks};
    use nalgebra as na;
    use std::f64::consts::PI;
    use std::io::Write;

    let env_f64 = |k: &str, d: f64| -> f64 {
        std::env::var(k)
            .ok()
            .and_then(|s| s.parse().ok())
            .unwrap_or(d)
    };
    let flag = |k: &str, d: bool| -> bool { std::env::var(k).map(|v| v != "0").unwrap_or(d) };

    // ---- phase-scoped gains ----------------------------------------------
    //
    // Every gain in this example is shared by the whole run, and the approach
    // walk is tuned to their current values. Measured on the fixed-arm
    // one-step, that made five separate hypotheses about the stair transfer
    // untestable rather than false: raising `KP_TRUNK` to 600/1500 fell at
    // 8.11/3.73 s, `COP_FRAC` 0.85/0.70 at 10.09/5.36 s, `JLIM_AMAX`
    // 600/2000 at 3.79/3.78 s, `COM_LAND_PREDICT_FRAC` 0.5/0 at 12.22 s, and
    // level landing confirmation at 5.66 s -- all of them BEFORE reaching the
    // stair, so none of them was a verdict on the transfer.
    //
    // `NAME_XFER` applies while a stair transfer is running, `NAME_POST`
    // after one has completed. Each defaults to `NAME`, so leaving them unset
    // reproduces any existing recipe exactly.
    #[derive(Clone, Copy, PartialEq, Eq)]
    enum RunPhase {
        Approach,
        Transfer,
        Post,
    }
    #[derive(Clone, Copy)]
    struct PhaseGain {
        base: f64,
        xfer: f64,
        post: f64,
    }
    impl PhaseGain {
        fn at(&self, p: RunPhase) -> f64 {
            match p {
                RunPhase::Approach => self.base,
                RunPhase::Transfer => self.xfer,
                RunPhase::Post => self.post,
            }
        }
        fn scoped(&self) -> bool {
            self.xfer != self.base || self.post != self.base
        }
    }
    let phase_gain = |name: &str, base: f64| -> PhaseGain {
        PhaseGain {
            base,
            xfer: env_f64(&format!("{name}_XFER"), base),
            post: env_f64(&format!("{name}_POST"), base),
        }
    };

    let prof = profile::by_name(&std::env::var("ROBOT").unwrap_or_default());

    let mut o = RigOptions::from_profile(&prof);
    o.knee = env_f64("KNEE", prof.knee_seed);
    o.hip_pitch = env_f64("HIP_PITCH", -o.knee / 2.0);
    o.ankle_pitch = env_f64("ANKLE_PITCH", -o.knee / 2.0);
    o.joint_damping = env_f64("JOINT_DAMPING", prof.joint_damping);
    // Per-joint viscous damping as a DESIGN input, e.g.
    // DAMPING_OVERRIDES="elbow=0.25,ankle_pitch=0.4" (name substring = value,
    // N*m*s/rad). Reaches plant and QP model consistently -- see RigOptions.
    if let Ok(s) = std::env::var("DAMPING_OVERRIDES") {
        for part in s.split(',').filter(|p| !p.trim().is_empty()) {
            let (pat, val) = part
                .split_once('=')
                .expect("DAMPING_OVERRIDES wants name=value[,name=value]");
            o.joint_damping_overrides.push((
                pat.trim().to_string(),
                val.trim().parse().expect("damping value"),
            ));
        }
    }
    o.armature = env_f64("ARMATURE", prof.armature);
    // Override selected joints without changing the other motor groups.
    // Example: ARMATURE_OVERRIDES="ankle_roll=0.005".
    if let Ok(s) = std::env::var("ARMATURE_OVERRIDES") {
        for part in s.split(',').filter(|p| !p.trim().is_empty()) {
            let (pat, val) = part
                .split_once('=')
                .expect("ARMATURE_OVERRIDES wants name=value[,name=value]");
            let value: f64 = val.trim().parse().expect("armature value");
            assert!(
                value.is_finite() && value >= 0.0,
                "armature override must be finite and nonnegative"
            );
            o.joint_armature_overrides
                .push((pat.trim().to_string(), value));
        }
    }
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
    if let Ok(value) = std::env::var("FOOT_SOLIMP") {
        let values: Vec<f64> = value
            .split(',')
            .map(|part| {
                part.trim()
                    .parse()
                    .expect("FOOT_SOLIMP values must be numbers")
            })
            .collect();
        assert!(values.len() == 3, "FOOT_SOLIMP wants d0,d_width,width");
        assert!(
            values[0] >= 0.0
                && values[0] <= 1.0
                && values[1] >= 0.0
                && values[1] <= 1.0
                && values[2] > 0.0,
            "FOOT_SOLIMP requires d0,d_width in [0,1] and width > 0"
        );
        o.foot_solimp = Some([values[0], values[1], values[2]]);
    }
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
    // Abduct the arms. The v6 shoulder roll exists precisely to open the
    // forearm <-> hip_roll_block clearance Sec.10.4 measured at 2.5 mm, but
    // nothing on the control side commands it -- the posture task only holds
    // it near its seed, so at seed 0 the arm keeps the pitch-only geometry
    // and now has a free axis to swing further in along. Positive is outward
    // on the left, negative on the right (mirrored limits in the URDF).
    // Setting it on a pre-v6 URDF panics in the rig, which is the honest
    // outcome: the seed cannot be applied to a joint that does not exist.
    let shoulder_roll_seed = env_f64("SHOULDER_ROLL", 0.0);
    if shoulder_roll_seed != 0.0 {
        o.extra_seed
            .push(("left_shoulder_roll_joint", shoulder_roll_seed));
        o.extra_seed
            .push(("right_shoulder_roll_joint", -shoulder_roll_seed));
    }
    if hip_roll_seed != 0.0 {
        // Signs mirror: both hips roll inward to bring the feet together, and
        // the ankles roll back so the soles stay flat on the floor.
        o.extra_seed.push((prof.hip_roll[0], -hip_roll_seed));
        o.extra_seed.push((prof.hip_roll[1], hip_roll_seed));
        o.extra_seed.push(("left_ankle_roll_joint", hip_roll_seed));
        o.extra_seed
            .push(("right_ankle_roll_joint", -hip_roll_seed));
    }

    // Seeded jitter on the INITIAL POSTURE of the twelve leg joints.
    //
    // The verification harnesses used to make their "members" by moving KD_COM
    // by 1e-9, which samples the chaotic spread around ONE scenario rather
    // than a distribution of situations. Measured 2026-09-24: under the
    // weighted QP -- which is far less chaotic -- all sixteen members of a
    // push condition then fell at the same millisecond, i.e. N=16 was really
    // N=1. A jitter on the posture the robot is built in is a real initial
    // condition, and it is the cheapest one that survives a controller that
    // does not amplify rounding. INIT_JITTER = 0 (default) adds nothing and
    // keeps every recipe bit-for-bit.
    let init_jitter = env_f64("INIT_JITTER", 0.0).abs();
    if init_jitter > 0.0 {
        // A small deterministic generator, so a run is reproducible from
        // (INIT_SEED, INIT_JITTER) alone and no RNG crate enters the example.
        // SplitMix64 on the seed first: `seed | 1` would give 2 and 3 the
        // same stream, which is how members 2/3, 4/5 and 6/7 came out
        // identical on the first try.
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
        // Added, never assigned: `extra_seed` would erase the profile's own
        // posture (bent knees, hip pitch) as well as the stance-width seed.
        for name in [
            "left_hip_pitch_joint",
            "left_hip_roll_joint",
            "left_hip_yaw_joint",
            "left_knee_joint",
            "left_ankle_pitch_joint",
            "left_ankle_roll_joint",
            "right_hip_pitch_joint",
            "right_hip_roll_joint",
            "right_hip_yaw_joint",
            "right_knee_joint",
            "right_ankle_pitch_joint",
            "right_ankle_roll_joint",
        ] {
            o.extra_seed_delta.push((name, init_jitter * next()));
        }
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
    // Leg reach (src/biped/reach.rs). `LEG_REACH` overrides the profile's
    // measured length; the margin is what a PLAN may ask for. Used for the
    // diagnostic columns always, and for the footstep bounds only when
    // REACH_LIMIT is on.
    let leg_reach = articara::biped::reach::LegReach {
        max: env_f64("LEG_REACH", prof.leg_reach).max(0.0),
        margin: env_f64("LEG_REACH_MARGIN", 0.95).clamp(0.1, 1.0),
    };
    let sole_half_l = env_f64("SOLE_HALF_L", prof.cop_half.0);
    let sole_half_w = env_f64("SOLE_HALF_W", prof.cop_half.1);
    let mu_ground = o.mu_ground;
    let ctrl_mode = o.ctrl_mode;

    // A thing to hit. `SPAR_WALL` is where its FRONT FACE goes, in metres
    // along +x; the body is welded to the world, which is the upper bound on
    // what a punch can transfer -- an opponent that gives way takes less.
    // Doc Sec.42.4 could not tell an impact from a lean because there was
    // nothing in the scene to make contact with.
    let spar_wall = env_f64("SPAR_WALL", 0.0);
    if spar_wall > 0.0 {
        // Height and width are MEASURED, not guessed: standing, the base sits
        // at z = 0.477 and the extended fist passes through (x 0.221, z 0.558)
        // with the two fists at y = +-0.115. A first attempt put the box at
        // z = 0.43 half a torso-height too low and 0.10 too narrow, and it
        // recorded zero contacts while looking like a null result.
        let cx = spar_wall + 0.05;
        let cz = env_f64("SPAR_WALL_Z", 0.52);
        o.extra_worldbody = Some(format!(
            "\n    <body name=\"spar_target\" pos=\"{cx:.4} 0 {cz:.4}\">\
             \n      <geom name=\"spar_target_geom\" type=\"box\" size=\"0.05 0.18 0.16\" \
             rgba=\"0.75 0.30 0.22 1\"/>\n    </body>"
        ));
    }
    // ---- stairs ---------------------------------------------------------
    // `STAIRS="n,riser,tread[,start_x]"` puts a flight of steps across +x.
    // The default proportions elsewhere (scripts) follow the human ratio:
    // hip-pitch height 0.276 m against a human's ~0.90 scales a 170/270 mm
    // stair to roughly 52/85 mm. A NEGATIVE riser descends: the robot spawns
    // on a platform `n*|riser|` high and the treads step down to the plane.
    use articara::biped::terrain::Stairs;
    // Landing length past the last tread. Default keeps the historical 0.60 m
    // so kyo46rs stays bit-identical; G1 runs must set STAIR_LANDING_LEN long
    // enough for the planned walk-out (>= 3.0 m for a 28-step plan). See
    // doc/from_claude/20260923_landing_edge.md.
    let stair_landing_len = env_f64("STAIR_LANDING_LEN", 0.60);
    let stairs: Option<Stairs> = std::env::var("STAIRS")
        .ok()
        .map(|s| {
            // STAIRS_DOWN=n2: an up-and-over course -- after the landing
            // (STAIR_LANDING_LEN) a second flight of n2 steps descends with
            // the same riser and tread. Ascending STAIRS only. 0 = off.
            let mut st = Stairs::parse(&s, stair_landing_len);
            if st.riser > 0.0 {
                st.down_n = (env_f64("STAIRS_DOWN", 0.0).max(0.0) as usize).min(st.n);
            }
            st
        });
    if let Some(st) = &stairs {
        let half_w = env_f64("STAIR_HALF_W", 0.30);
        let mut wb = String::new();
        let mut add_box = |name: String, x0: f64, x1: f64, top: f64| {
            if top <= 1e-9 {
                return;
            }
            wb.push_str(&format!(
                "\n    <body name=\"{name}\" pos=\"{:.4} 0 {:.4}\">\
                 \n      <geom name=\"{name}_geom\" type=\"box\" size=\"{:.4} {half_w:.4} {:.4}\" \
                 rgba=\"0.55 0.57 0.62 1\"/>\n    </body>",
                0.5 * (x0 + x1),
                0.5 * top,
                0.5 * (x1 - x0),
                0.5 * top
            ));
        };
        if st.riser < 0.0 {
            // The platform the robot starts on, reaching back under the spawn.
            add_box(
                "stair_platform".into(),
                st.start - 0.55,
                st.start,
                st.base(),
            );
        }
        for k in 1..=st.n {
            let x0 = st.start + (k - 1) as f64 * st.tread;
            add_box(
                format!("stair_{k}"),
                x0,
                x0 + st.tread,
                st.base() + k as f64 * st.riser,
            );
        }
        if st.riser > 0.0 {
            // Somewhere to stop: a landing as high as the last tread.
            //
            // LENGTH. `Stairs::h` clamps its level to `n`, i.e. the planner
            // believes the top surface continues forever, and the walk-out
            // ladder (`pos_of` for `lvl > n`) keeps placing footfalls on it.
            // The physical landing used to end 0.60 m past the last tread.
            // Measured 2026-09-23 on G1 (T=40, N=48 x 4 risers): every
            // walk-out fell at x = 2.8..3.1 m, i.e. AT THIS EDGE -- the
            // stance sole rolls off it at half support and the next swing
            // waits in the air for a tread that is not there. Survival
            // scaled with the riser (4 mm 46/48, 60 mm 0/48) because the
            // riser is the height of the drop. The T=30 tests of the two
            // days before stopped ~1 s short of the edge and never saw it.
            // Default keeps the historical 0.60 m so kyo46rs stays
            // bit-identical; G1 runs must set STAIR_LANDING_LEN long enough
            // for the planned walk-out (>= 3.0 m for a 28-step plan).
            let x1 = st.start + st.n as f64 * st.tread;
            add_box(
                "stair_landing".into(),
                x1,
                x1 + st.landing_len,
                st.n as f64 * st.riser,
            );
            // The descending flight of an up-and-over course, and the lower
            // landing it ends on when it does not return to the ground.
            let le = x1 + st.landing_len;
            for k in 1..=st.down_n {
                add_box(
                    format!("stair_down_{k}"),
                    le + (k - 1) as f64 * st.tread,
                    le + k as f64 * st.tread,
                    (st.n - k) as f64 * st.riser,
                );
            }
            if st.down_n > 0 && st.down_n < st.n {
                let x2 = le + st.down_n as f64 * st.tread;
                add_box("stair_lower_landing".into(), x2, x2 + 4.0, (st.n - st.down_n) as f64 * st.riser);
            }
        }
        o.extra_worldbody = Some(match o.extra_worldbody.take() {
            Some(s) => s + wb.as_str(),
            None => wb,
        });
        o.spawn_z_offset = st.base();
        // Extra spawn clearance [m] for a descent platform: the spawn height
        // is probed in the neutral posture, and INIT_JITTER then lowers one
        // sole into the platform box, which the spawn self-collision check
        // rejects (2026-10-02: 36/48 descent members panicked at spawn).
        // 0 = off = shipped.
        o.spawn_z_offset += env_f64("SPAWN_Z_EXTRA", 0.0).max(0.0);
        println!(
            "stairs: n={} riser={:+.4} tread={:.4} start={:.2} (spawn platform {:.4})",
            st.n,
            st.riser,
            st.tread,
            st.start,
            st.base()
        );
    }
    let mut rig = BipedRig::build(prof, &o);
    // Model indices of the links the thighs hang from, for the reach columns.
    let hip_mi: Option<[usize; 2]> = prof.hip_links.map(|names| {
        names.map(|n| {
            *rig.link_to_idx
                .get(n)
                .unwrap_or_else(|| panic!("profile names hip link {n}, which this robot lacks"))
        })
    });
    let nv = rig.nv;
    let na_count = rig.na;
    let mj_dt = rig.mj_dt;
    let total_mass = rig.total_mass;

    let f_max_scale = env_f64("F_MAX_SCALE", 2.3);
    let f_max_per_foot = env_f64("F_MAX", f_max_scale * total_mass * G);
    // The wrench MPC's own per-foot ceiling, as a multiple of body weight.
    // Default = the WBC's (F_MAX), bit-identical. Measured 2026-10-01
    // (design note Sec.14): with the CoP-capture rows the MPC answers a
    // backward push by loading the front foot to 445 N (1.3 x body weight)
    // and unloading the rear -- the rows linearise the CoP at the reference
    // fz, so excess vertical force buys an apparent heel-ward CoP for free,
    // and the WBC's regulariser follows the front loading. 1.0 = no foot
    // may carry more than the body weight.
    let mpc_f_max_per_foot = {
        let scale = env_f64("MPC_F_MAX_SCALE", f64::NAN);
        if scale.is_finite() && scale > 0.0 {
            scale * total_mass * G
        } else {
            f_max_per_foot
        }
    };
    assert!(
        f_max_per_foot >= total_mass * G,
        "f_max ({f_max_per_foot:.1} N) is below the single-support nominal ({:.1} N)",
        total_mass * G
    );

    // ── Control-law knobs ──────────────────────────────────────────────
    let mut solver = Solver::new();
    // Capture immutable task stacks for offline numerical diagnosis only.
    let qp_diag_dir = std::env::var("QP_DIAG_DIR").ok();
    let mut qp_diag_count = 0usize;
    // Opt-in continuous sampling; legacy failure-only capture remains default.
    let qp_capture_all = flag("QP_CAPTURE_ALL", false);
    let qp_capture_start = env_f64("QP_CAPTURE_START", 0.0);
    let qp_capture_end = env_f64("QP_CAPTURE_END", f64::INFINITY);
    let qp_capture_stride = env_f64("QP_CAPTURE_STRIDE", 1.0).max(1.0) as usize;
    let qp_capture_max = env_f64("QP_CAPTURE_MAX", 20000.0).max(1.0) as usize;
    // Additional WBC backends must never bypass the independent acceptance gate.
    // QP_BENCH_GATE also enables the identical gate for existing baselines.
    let qp_gate = flag("QP_BENCH_GATE", false)
        || matches!(
            std::env::var("WBC_QP_BACKEND").as_deref(),
            Ok("proxqp" | "eiquadprog" | "qpoases")
        );
    assert!(
        !qp_gate || ctrl_mode == CtrlMode::Torque,
        "QP_BENCH_GATE requires torque control"
    );
    let mpc_gate = qp_gate || flag("MPC_BENCH_GATE", false);
    assert!(
        !mpc_gate || ctrl_mode == CtrlMode::Torque,
        "MPC_BENCH_GATE requires torque control"
    );
    // Experimental use of an independently checked prefix when only a
    // lower-priority HQP refinement fails. The command policy still receives
    // the true degraded status; FALLBACK_LEVEL decides whether to apply it.
    let prefix_min_level = env_f64("WBC_PREFIX_MIN_LEVEL", 999.0).max(0.0) as usize;
    let repair_original = flag("WBC_REPAIR_ORIGINAL", false);
    assert!(
        !repair_original || qp_gate,
        "WBC_REPAIR_ORIGINAL requires QP_BENCH_GATE"
    );
    let mut qp_has_good_command = false;
    let mut qp_previous_structure: Option<(Vec<usize>, Vec<(usize, usize, usize)>)> = None;
    let mut qp_runtime_log = std::env::var("QP_RUNTIME_CSV").ok().map(|p| {
        use std::io::Write;
        let mut f = std::io::BufWriter::new(std::fs::File::create(p).expect("QP_RUNTIME_CSV"));
        writeln!(f, "t,solve_ns,check_ns,status,normalized_physical_violation,reset,deadline_miss,control_ns,mpc_age_s,mpc_solves,mpc_failures,prefix_applied,repair_applied,repair_scaled_delta,repair_ns").unwrap();
        f
    });
    // The MPC update period and the 5 ms WBC tick are separate deadlines.
    let mut mpc_runtime_log = std::env::var("MPC_RUNTIME_CSV").ok().map(|p| {
        use std::io::Write;
        let mut f = std::io::BufWriter::new(std::fs::File::create(p).expect("MPC_RUNTIME_CSV"));
        writeln!(f, "t,phase,contact_mask,backend,horizon,update_period_s,solve_ns,deadline_miss,accepted,previous_age_s").unwrap();
        f
    });
    // Opt-in centroidal diagnostic: the arm columns measure momentum from arm
    // joint motion, while base-motion inertia remains in the total-minus-arm term.
    let mut mom_diag_log = std::env::var("MOM_DIAG_CSV").ok().map(|p| {
        use std::io::Write;
        let mut f = std::io::BufWriter::new(std::fs::File::create(p).expect("MOM_DIAG_CSV"));
        writeln!(f, "t,h_total_x,h_total_y,h_total_z,h_arm_joint_x,h_arm_joint_y,h_arm_joint_z,h_other_x,h_other_y,h_other_z,fz_measured_l,fz_measured_r,fz_qp_l,fz_qp_r,n_stance").unwrap();
        f
    });
    // Six-axis instrumentation is additive; writes/conversions are outside control_ns.
    let mut qp_wrench_log = std::env::var("QP_WRENCH_CSV").ok().map(|p| {
        use std::io::Write;
        let mut f = std::io::BufWriter::new(std::fs::File::create(p).expect("QP_WRENCH_CSV"));
        write!(f, "t,side,planned_support,wbc_contact,mpc_valid,planned_valid,wbc_valid,measured_valid,wbc_degraded,mpc_age_s").unwrap();
        for source in ["planned", "mpc", "wbc", "measured"] {
            for component in ["mx", "my", "mz", "fx", "fy", "fz"] { write!(f, ",{source}_{component}").unwrap(); }
        }
        writeln!(f).unwrap();
        f
    });
    // Optional objective term (prox_weight/2)*||x-x_previous||^2, default 0.
    // This changes the mathematical objective; it is independent of pure
    // workspace/primal-dual warm starts, which can be used with weight 0.
    // Numerical comparisons and the independent original-QP gate require 0.
    let prox_weight = env_f64("PROX_WEIGHT", 0.0);
    let qp_reset_every_tick = flag("WBC_QP_RESET_EVERY_TICK", false);
    let feasibility_tol = env_f64("WBC_FEAS_TOL", SolveConfig::default().feasibility_tol);
    assert!(
        feasibility_tol.is_finite() && feasibility_tol > 0.0,
        "WBC_FEAS_TOL must be positive and finite"
    );
    let mut cfg = SolveConfig {
        prox_weight,
        feasibility_tol,
        validation_tol: qp_gate.then_some(1e-6),
        hard_first_inequalities: flag("WBC_HARD_PHYSICAL", false),
        ..SolveConfig::default()
    };
    // Priority resolution. Default = the strict null-space cascade (shipped).
    // `weighted` = one QP with level k>=1 weighted WBC_WEIGHT_RATIO^-(k-1),
    // level 0 hard (misa_wbc::HqpStrategy::WeightedQp) -- plan v2 P4.
    let weighted_levels = std::env::var("WBC_STRATEGY").as_deref() == Ok("weighted");
    let weighted_off_stairs = flag("WBC_WEIGHTED_OFF_STAIRS", false);
    match std::env::var("WBC_STRATEGY").as_deref() {
        Ok("weighted") => {
            // Per-task weights are applied to the levels themselves (see
            // WBCW_* below); the ratio defaults to 1 so they are not scaled
            // again by position.
            cfg.strategy = misa_wbc::HqpStrategy::WeightedQp {
                level_ratio: env_f64("WBC_WEIGHT_RATIO", 1.0).max(1.0) as u32,
            };
        }
        Ok("force_budget") => cfg.strategy = misa_wbc::HqpStrategy::ForceBudgetCascade,
        Ok("null_space") | Err(_) => {}
        Ok(other) => panic!("unsupported WBC_STRATEGY: {other}"),
    }
    if let Ok(name) = std::env::var("WBC_QP_BACKEND") {
        cfg.backend = match name.as_str() {
            "clarabel" => misa_wbc::QpSolver::Clarabel,
            "active_set" => misa_wbc::QpSolver::ActiveSet,
            "proxqp" => misa_wbc::QpSolver::Proxqp,
            "eiquadprog" => misa_wbc::QpSolver::Eiquadprog,
            "qpoases" => misa_wbc::QpSolver::Qpoases,
            _ => panic!("unsupported WBC_QP_BACKEND: {name}"),
        };
    }
    // Joint position/velocity safety limits (misa_wbc::tasks::joint_limit_cbf).
    // Off by default: only the torque box is enforced today, which is why a
    // redundant leg could be commanded into a solution only realisable by
    // running a joint past its physical stop -- measured on ankle_roll at
    // 196% of its URDF limit while falling (doc Sec.18.5). Added at the same
    // priority as the torque box (`base`, below), since both are "never
    // violate" safety constraints rather than tracked objectives. Gains are
    // a first guess, unverified on this robot -- doc Sec.21.7/25.4.
    let jlim = flag("JLIM", false);
    let jlim_alpha1 = env_f64("JLIM_ALPHA1", 10.0);
    let jlim_alpha2 = env_f64("JLIM_ALPHA2", 10.0);
    let jlim_alpha3 = env_f64("JLIM_ALPHA3", 10.0);
    let jlim_amax_g = phase_gain("JLIM_AMAX", env_f64("JLIM_AMAX", 50.0));
    let jlim_amax = jlim_amax_g.base;
    // Optional diagnostic operating envelope for shoulder pitch.  This uses
    // the same hard CBF as the physical joint limits, but intersects the URDF
    // range with a symmetric band around the spawn/posture seed.  It answers
    // whether stair success actually requires the roughly 100 degree arm
    // excursions seen in the unconstrained solution.  Zero keeps the old
    // physical-limit-only behavior.
    let arm_pitch_band = env_f64("ARM_PITCH_BAND", 0.0);
    let arm_pitch_vmax = env_f64("ARM_PITCH_VMAX", 0.0);
    let arm_pitch_amax = env_f64("ARM_PITCH_AMAX", 0.0);
    assert!(
        arm_pitch_band.is_finite() && arm_pitch_band >= 0.0,
        "ARM_PITCH_BAND must be finite and non-negative"
    );
    assert!(
        arm_pitch_vmax.is_finite() && arm_pitch_vmax >= 0.0,
        "ARM_PITCH_VMAX must be finite and non-negative"
    );
    assert!(
        arm_pitch_amax.is_finite() && arm_pitch_amax >= 0.0,
        "ARM_PITCH_AMAX must be finite and non-negative"
    );
    let mut jlim_q_min = rig.q_min.clone();
    let mut jlim_q_max = rig.q_max.clone();
    let mut jlim_v_max = rig.v_max.clone();
    let mut jlim_a_max = na::DVector::from_element(na_count, jlim_amax);
    // Operate the CBF INSIDE the mechanical limits, for EVERY joint. Without
    // this, widening a joint's URDF range widens the safety constraint with
    // it, so the machine gains permission rather than margin -- measured by
    // the Codex thread, ankle_roll +-20 -> +-30 -> +-40 deg made the in-place
    // test fall EARLIER at every widening. Reserving a band the controller
    // will not enter is what makes a wider mechanism a SAFER one, and it is
    // the prerequisite for the morphology study's wider ankle.
    // (`ARM_PITCH_BAND` below narrows the shoulders further, and still wins.)
    let jlim_q_margin_g = phase_gain("JLIM_Q_MARGIN", env_f64("JLIM_Q_MARGIN", 0.0));
    let jlim_q_margin = jlim_q_margin_g.base.max(0.0);
    if jlim_q_margin_g.scoped() {
        assert!(jlim, "JLIM_Q_MARGIN_XFER / _POST require JLIM=1");
    }
    let jlim_v_frac = env_f64("JLIM_V_FRAC", 1.0).clamp(0.05, 1.0);
    if jlim_q_margin > 0.0 || jlim_v_frac < 1.0 {
        assert!(jlim, "JLIM_Q_MARGIN / JLIM_V_FRAC require JLIM=1");
        for row in 0..na_count {
            let mid = 0.5 * (jlim_q_min[row] + jlim_q_max[row]);
            let m = jlim_q_margin.min(0.45 * (jlim_q_max[row] - jlim_q_min[row]));
            jlim_q_min[row] = (jlim_q_min[row] + m).min(mid);
            jlim_q_max[row] = (jlim_q_max[row] - m).max(mid);
            jlim_v_max[row] *= jlim_v_frac;
        }
        println!(
            "CBF operating band: {:.1} deg inside every position stop, v x{:.2}",
            jlim_q_margin.to_degrees(),
            jlim_v_frac
        );
    }
    if arm_pitch_band > 0.0 || arm_pitch_vmax > 0.0 || arm_pitch_amax > 0.0 {
        assert!(jlim, "shoulder operating limits require JLIM=1");
        let mut narrowed = 0usize;
        for (ji, vi) in rig.actuated() {
            if rig.robot.joints[ji].name.ends_with("shoulder_pitch_joint") {
                let row = vi - 6;
                let centre = rig.q_seed[ji];
                if arm_pitch_band > 0.0 {
                    jlim_q_min[row] = jlim_q_min[row].max(centre - arm_pitch_band);
                    jlim_q_max[row] = jlim_q_max[row].min(centre + arm_pitch_band);
                }
                if arm_pitch_vmax > 0.0 {
                    jlim_v_max[row] = jlim_v_max[row].min(arm_pitch_vmax);
                }
                if arm_pitch_amax > 0.0 {
                    jlim_a_max[row] = jlim_a_max[row].min(arm_pitch_amax);
                }
                assert!(
                    jlim_q_min[row] < jlim_q_max[row],
                    "ARM_PITCH_BAND leaves no range for {}",
                    rig.robot.joints[ji].name
                );
                narrowed += 1;
            }
        }
        assert!(
            narrowed == 2,
            "ARM_PITCH_BAND expected two shoulder pitch joints, found {narrowed}"
        );
        println!(
            "shoulder pitch operating limits: band={} deg  vmax={} rad/s  amax={} rad/s^2",
            if arm_pitch_band > 0.0 {
                arm_pitch_band.to_degrees()
            } else {
                f64::INFINITY
            },
            if arm_pitch_vmax > 0.0 {
                arm_pitch_vmax
            } else {
                f64::INFINITY
            },
            if arm_pitch_amax > 0.0 {
                arm_pitch_amax
            } else {
                f64::INFINITY
            }
        );
    }
    let mut jlim_cfg = tasks::JointLimitCbf {
        q_min: jlim_q_min,
        q_max: jlim_q_max,
        v_max: jlim_v_max,
        a_max: jlim_a_max,
        alpha1: na::DVector::from_element(na_count, jlim_alpha1),
        alpha2: na::DVector::from_element(na_count, jlim_alpha2),
        alpha3: na::DVector::from_element(na_count, jlim_alpha3),
    };
    let friction_mu: f64 = env_f64("friction_mu", mu_ground * prof.friction_margin);
    let kp_com = env_f64("KP_COM", 300.0);
    let mut kd_com = env_f64("KD_COM", 80.0);
    let kp_trunk_g = phase_gain("KP_TRUNK", env_f64("KP_TRUNK", 200.0));
    let mut kd_trunk_g = phase_gain("KD_TRUNK", env_f64("KD_TRUNK", 40.0));
    let kp_post = env_f64("KP_POST", 100.0);
    let kd_post = env_f64("KD_POST", 20.0);
    let use_post = flag("POST", true);
    // Hold the arms, at their own level ABOVE posture.
    //
    // Sec.29.6 measured the v6 arm swinging 111 deg peak-to-peak at the
    // shoulder roll and dragging pitch and elbow to 64/26 deg with it, where
    // every pre-v6 model and `v6fixed` sit at 2-10 deg. The posture level is
    // the only thing regulating any of it, and it runs at one uniform gain
    // BELOW swing -- for the legs that is harmless because the contact/CoM/
    // swing tasks pin them anyway, and for an arm in every one of those
    // tasks' null space it is the whole budget.
    //
    // NOT free, though it looked like it should be: the arms carry no load, so
    // a level of their own reads like it can only take null space nothing else
    // wanted. Measured (Sec.30.3), the same task on v4 -- a model with no
    // shoulder roll and no problem -- costs 22/28 -> 16/28. On v6 it is a net
    // win only because the flailing was costing more than the level does.
    // Default OFF for that reason.
    let arm_hold = flag("ARM_HOLD", false);
    // Experimental priority: keep arm posture above swing (but below CoM/trunk).
    // Default stays below swing; compare full closed-loop motion before adoption.
    let arm_before_swing = flag("ARM_BEFORE_SWING", false);
    let kp_arm = env_f64("KP_ARM", 400.0);
    // BDI/IHMC 方式: 運動量と腕を、重心と「同じレベル」の重み付き目的として載せる。
    //
    // 現行の階層は 接触 -> 重心/体幹 -> 遊脚 -> 腕保持 -> 姿勢 で、腕保持は遊脚の
    // null space の中でしか動けない。G1 自由腕で N=16 実測: `ARM_HOLD=1` かつ
    // `KP_ARM=752` でも腕は 186-215 deg 振れ、`KP_ARM` を 4 倍にすると 202 deg へ
    // 「増える」。慣性 40 倍・粘性 10 倍でも抑えられない
    // (doc/from_claude/20260919_g1_free_arm.md)。優先度の下にある限り、腕は
    // 最適化の対象ではなく上位タスクの副産物になる。
    //
    // Koolen らの Atlas は単一 QP の重み付き目的で、主タスクが運動量変化率、
    // 骨盤高さ・胸姿勢・手先・腕配置は「同じ QP の重み項」。Boston Dynamics も
    // パルクール期の「総重心と慣性だけ」から「全関節の運動・全リンクの運動量」へ
    // モデルを広げ、腕脚で慣性を変える制御を単一の MPC 最適化に入れたと述べている。
    //
    // どちらも既定 0 = 従来の階層のまま。
    let mom_merge = env_f64("MOM_MERGE", 0.0).max(0.0);
    let arm_merge = env_f64("ARM_MERGE", 0.0).max(0.0);
    let kd_arm = env_f64("KD_ARM", 40.0);
    // kd-only damping on the elbow joints (0 = off). See the arm level.
    let elbow_kd = env_f64("ELBOW_KD", 0.0);
    // WHICH arm rows to hold. Holding all of them keeps a commanded pose but
    // costs the top lateral rung: at VY=0.073 the held arm survives 1 of 16
    // probes where the free arm survives 16 (Sec.38). The free arm keeps the
    // guard's SHAPE anyway -- measured, the elbow sits at -128 deg against a
    // -130 seed and travels 23 deg, while the shoulder swings 132 deg about
    // its seed. So the shoulder swing is what the walk wants and the elbow
    // fold is what the guard wants, and they are separable.
    //   all      -- shoulder + elbow (what Sec.30 measured)
    //   elbow    -- fold held, shoulder free to swing
    //   shoulder -- the opposite, for symmetry of the experiment
    let arm_hold_joints = std::env::var("ARM_HOLD_JOINTS").unwrap_or_else(|_| "all".into());
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
    // Centroidal angular-momentum task (doc Sec.21.3, reopened by the v6
    // shoulder roll -- see `bt::angular_momentum`). `MOM_AXES` is a 3-char
    // mask over roll/pitch/yaw, e.g. "x--" for roll only. Default OFF: it
    // costs two extra CMM evaluations a tick and has never been measured.
    let use_mom = flag("MOM", false);
    let kp_mom = env_f64("KP_MOM", 10.0);
    let mom_axes = {
        let s = std::env::var("MOM_AXES").unwrap_or_else(|_| "x--".into());
        let b = s.as_bytes();
        [
            b.first().is_some_and(|c| *c == b'x'),
            b.get(1).is_some_and(|c| *c == b'y'),
            b.get(2).is_some_and(|c| *c == b'z'),
        ]
    };
    // Take the momentum task's reference from the wrench MPC instead of zero.
    //
    // `bt::angular_momentum` says `J_ang qdd + bias = dh_ref - kp (h - h_ref)`.
    // With both references zero it says "never change angular momentum", which
    // a walk cannot honour: the legs change it by construction, so the solver
    // spends the cheapest joints it owns -- the arms -- cancelling them.
    // Measured 2026-09-20 on the free-arm G1 (doc/from_claude/20260920_bdi_task_structure.md):
    // enabling the task took arm travel from 1.8 deg to 272-297 deg and
    // sweeping `KP_MOM` over 60x did not move it, because the REFERENCE, not
    // the gain, is what forbids the motion. That doc's conclusion was that a
    // planner had to supply `h(t)` first, and that this example could not be
    // connected to one because it runs its own DCM gait.
    //
    // The closed-loop wrench MPC (2026-09-22) is that planner, and it plans
    // THIS gait: its centroidal state carries angular momentum, so its
    // terminal value is the momentum this motion is heading for under the
    // contact wrenches it just solved for. Setting `h_ref = h_now` cancels the
    // P term and leaves `dh_ref` = the MPC's planned rate, i.e. "change
    // angular momentum the way the plan says", which the legs can satisfy on
    // their own -- so the arms are no longer asked to cancel anything.
    //
    // Default OFF, and inert unless `MOM` and `STAIR_WRENCH_MPC` are both on.
    let mom_ref_mpc = flag("MOM_REF_MPC", false);
    // Where the level sits in the stack. The task is an EQUALITY at its
    // level, so `kp` does not control how much of the null space it takes --
    // even `kp=0` still demands `ḣ_ang = 0`, i.e. "never change roll momentum",
    // which a walk cannot honour. Placement is therefore the real knob:
    //   `swing_before` / `swing_after` -- above posture, all authority
    //   `post_after`                   -- below posture, only what is left
    //   `post_merge`                   -- SAME level as posture, so the two
    //                                     trade off inside one least-squares
    //                                     instead of one strictly preceding
    //                                     the other
    let mom_level = std::env::var("MOM_LEVEL").unwrap_or_else(|_| "swing_after".into());
    // 角運動量タスクが「追従」する参照の作り方。
    //   zero         : h_ref = 0 = 従来(抑制)。歩行と両立しないことを実測済み。
    //   arms_neutral : h_ref = h - A_arms·v_arms、つまり残差が「腕の寄与そのもの」に
    //                  なる。タスクは「腕の角運動量寄与をゼロに。脚の分には触るな」。
    // 本来は A2 の可行な全身参照から h_ref(t) を取るべきで、これはその足場。
    let mom_ref_mode = std::env::var("MOM_REF").unwrap_or_else(|_| "zero".into());
    // Ask the ARMS for roll angular momentum when the lateral DCM escapes.
    //
    // This is the eCMP/VRP mechanism (Englsberger IROS 2013) written as a task
    // reference. The centroidal moment pivot is the ZMP displaced by the rate
    // of change of angular momentum,
    //
    //     CMP_y = ZMP_y - h_dot_x / (m g)
    //
    // and the DCM is driven by the CMP, not the ZMP. So when the ZMP is pinned
    // to the lateral sole edge -- measured as the terminal state of every
    // 60 mm-riser fall, `zmp_y` clamped at -0.0919 for 2 s -- the only pivot
    // left is angular momentum. Driving the DCM back toward its reference in
    // +y needs CMP_y to move +y, hence h_dot_x NEGATIVE: the gain is applied
    // with a minus sign below.
    //
    // Only meaningful with `MOM_REF=arms_neutral`, where the task's Jacobian
    // is masked to the arm columns: the reference then says "the ARMS shall
    // produce this roll momentum rate" and the legs' own momentum is left
    // alone. With the unmasked CMM the same reference would instead forbid the
    // legs the momentum they make by construction -- the failure measured
    // 2026-09-20 (arm travel 1.8 -> 297 deg, unmoved by a 60x gain sweep).
    //
    // Default 0.0 = the zero reference, i.e. previous behaviour.
    let mom_arm_dcm = env_f64("MOM_ARM_DCM", 0.0);
    // Ceiling on the requested rate. The arms are light and near the CoM, so a
    // large request is one the solver cannot fill and will chase to the joint
    // limits; this bounds the ask rather than the gain.
    let mom_arm_dcm_max = env_f64("MOM_ARM_DCM_MAX", 20.0).max(0.0);
    // Sagittal counterpart of MOM_ARM_DCM: a forward/backward DCM error asks
    // for a PITCH angular-momentum rate (arms swinging in the sagittal
    // plane), dh_ref[1] = MOM_ARM_DCM_X * (xi.x - xi_ref.x), clamped to
    // MOM_ARM_DCM_X_MAX. Needs the pitch axis in MOM_AXES ("xy-"). 0 = off.
    let mom_arm_dcm_x = env_f64("MOM_ARM_DCM_X", 0.0);
    let mom_arm_dcm_x_max = env_f64("MOM_ARM_DCM_X_MAX", 20.0).max(0.0);
    // MOM_CMP: the hip strategy, written as the centroidal moment pivot.
    // When the DCM law's pressure point saturates at the support edge, the
    // deficit `p_raw - p_cmd` is pressure the polygon cannot give; the same
    // horizontal force follows from an angular-momentum rate instead
    // (CMP_y = CoP_y - h_dot_x / (m g), CMP_x = CoP_x + h_dot_y / (m g)).
    // This asks the WHOLE body (unmasked CMM, roll and pitch) for
    //     h_dot = MOM_CMP * m g * (-deficit_y, +deficit_x, 0),
    // capped at MOM_CMP_MAX [N m], with no P term, as a weighted task in
    // the centroidal level. The arms-only MOM_ARM_DCM task stays as it is.
    // Design: doc/from_claude/20261012_restep_design.md Sec.6 (the WBC's
    // lateral authority ends at the sole edge). 0 = off, bit-identical.
    let mom_cmp = env_f64("MOM_CMP", 0.0).max(0.0);
    let mom_cmp_max = env_f64("MOM_CMP_MAX", 20.0).max(0.0);
    let mom_cmp_dead = env_f64("MOM_CMP_DEAD", 0.003).max(0.0);
    let mom_cmp_w = env_f64("MOM_CMP_W", 1.0).max(0.0);
    // Only while a recovery reference is engaged.
    let mom_cmp_ref_only = flag("MOM_CMP_REF_ONLY", false);
    // Sign check (+1 = the derivation above) and a cap on the angular
    // momentum the strategy may build up [N m s]: a hip strategy is a
    // transient -- past the cap the rate in that direction is cut to zero
    // and a P term winds the momentum back (MOM_CMP_KH [1/s]).
    let mom_cmp_sign = env_f64("MOM_CMP_SIGN", 1.0);
    let mom_cmp_hmax = env_f64("MOM_CMP_HMAX", 0.0).max(0.0);
    let mom_cmp_kh = env_f64("MOM_CMP_KH", 0.0).max(0.0);
    // Off inside a stair zone with a real riser (the flat harness's 0 mm
    // stair does not count). Measured 2026-10-13 (N=48): the lean took the
    // pushes 301 -> 326 but the descents 191 -> 166, 3-step 142 -> 101,
    // the up-and-over 2/4 cm 48 -> 15 -- on a descent the pressure
    // saturates by design and leaning on it breaks the step. Ascents and
    // the flat were unchanged. true = lean on stairs too.
    let mom_cmp_stairs = flag("MOM_CMP_STAIRS", false);
    let mut n_mom_cmp_ticks = 0u32;
    let mut mom_cmp_peak = 0.0_f64;
    let use_trunk = flag("TRUNK", true);
    let trunk_sign = env_f64("TRUNK_SIGN", 1.0);
    let trunk_dead = env_f64("TRUNK_DEAD", 0.0);
    let trunk_late = flag("TRUNK_LATE", false);
    let eom_hard = flag("EOM_HARD", true);
    let cop_frac_g = phase_gain("COP_FRAC", env_f64("COP_FRAC", 1.0));
    let cop_frac = cop_frac_g.base;
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
    let kp_sw_z_g = phase_gain("KP_SWING", env_f64("KP_SWING", prof.kp_swing));
    let kp_sw_xy_g = phase_gain("KP_SWING_XY", env_f64("KP_SWING_XY", 0.25 * kp_sw_z_g.base));
    let kd_sw_g = phase_gain("KD_SWING", env_f64("KD_SWING", prof.kd_swing));
    // Phase-scoped, because the step that follows the stair transfer wants a
    // different one from the approach. Measured with N=16 ensembles
    // (doc/from_claude/20260919_approach_reach_rate.md): in 6 of the 7 members
    // that complete the transfer, the receiving sole sits exactly LIFT_H = 30 mm
    // up -- at the apex of its own lift -- at the instant the CoM crosses the
    // stance foot's lateral support edge. It is not that the contact fails to
    // establish; the foot has not started down. But lowering LIFT_H globally
    // takes transfer completion from 7/16 to 0/16, because the APPROACH needs
    // that clearance. So scope it: LIFT_H_XFER / LIFT_H_POST.
    let lift_h_g = phase_gain("LIFT_H", env_f64("LIFT_H", 0.5 * prof.lift_h));
    // Shorten only the first LEVEL swing after a completed stair transfer.
    // This changes the foot's clock, not the preplanned CoM/ZMP schedule.
    // 1.0 keeps the original trajectory and contact confirmation unchanged.
    let first_post_swing_fraction = env_f64("STAIR_FIRST_POST_SWING_FRACTION", 1.0);
    assert!(
        first_post_swing_fraction.is_finite() && (0.5..=1.0).contains(&first_post_swing_fraction),
        "STAIR_FIRST_POST_SWING_FRACTION must be in [0.5, 1.0]"
    );

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
    // Early window. Every post-push maximum below is reported TWICE: once over
    // this window and once over the rest of the run.
    //
    // WHY: the whole-run versions measure the crash. On the 0.20 N*s run that
    // falls, the 2.77x "landing impact" peak lands 0.69 s after touchdown with
    // the trunk already at 28 deg -- that is the robot hitting the floor, not
    // the disturbance. The unloading has the same problem: the total ground
    // force first drops below half body weight at t=5.40, by which point the
    // tilt has already run to 0.122 rad. A quantity that only separates
    // survivors from fallers over the whole run may be restating "it fell".
    // Windowed to the first 1/omega or so after the push, a quantity that
    // still separates is a PRECURSOR, and only that is usable for judging a
    // change.
    let push_window_s = env_f64("PUSH_WINDOW_S", 0.20);
    // ---- weight-transfer quality ---------------------------------------
    //
    // What actually separated the 0.20 N*s run that falls from the 0.30 that
    // survives, tick by tick: after the support changes, the survivor's CoP
    // leaves the sole EDGE within 10 ms and settles 2-4 mm from the sole
    // centre, and the total ground force locks at 1.00x. The faller's CoP
    // stays pinned at 18-19 mm -- the edge of a 19 mm half-width sole -- for
    // the whole rest of the run, while the force oscillates between 0 and
    // 1.97x.
    //
    // A CoP on the edge is a foot that cannot modulate its own pressure. The
    // peak `cop_use` does not show this (it reads 1.00 in benign runs too);
    // what matters is whether the foot comes OFF the edge, so this measures
    // TIME AT THE EDGE, anchored on the support change rather than on the
    // clock, so the number is not contaminated by however the fall unfolds.
    let edge_frac = env_f64("PUSH_EDGE_FRAC", 0.9);
    let transfer_s = env_f64("PUSH_TRANSFER_S", 0.10);
    let push_link = std::env::var("PUSH_LINK").unwrap_or_else(|_| prof.root_link.to_string());

    let ctrl_dt = env_f64("CTRL_DT", prof.ctrl_dt);
    let mj_substeps = (ctrl_dt / mj_dt).round().max(1.0) as u32;
    let dt = mj_substeps as f64 * mj_dt;
    let n_ticks = (total_t / dt) as usize;
    // Optional physics-rate audit. The normal trajectory checks only WBC ticks;
    // the simulator trace already contains q, qdot and applied torque after
    // every MuJoCo step, so no plant or controller behavior needs to change.
    let substep_audit_path = std::env::var("SUBSTEP_LIMIT_AUDIT_JSON").ok();
    let mut substep_max_excess = vec![[0.0_f64; 3]; rig.robot.joints.len()];
    let mut substep_extrema = vec![[f64::INFINITY, f64::NEG_INFINITY, 0.0, 0.0]; rig.robot.joints.len()];
    let mut substep_samples = 0_usize;
    let mut substep_nonfinite = 0_usize;
    if substep_audit_path.is_some() {
        rig.sim.set_trace_max(rig.sim.trace_len().max(mj_substeps as usize + 1));
    }
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
        let q = rig
            .sim
            .body_world_orientation(&rig.robot.root_link)
            .unwrap();
        q.euler_angles().2
    };
    let kp_sw_rot_g = phase_gain("KP_SWING_ROT", env_f64("KP_SWING_ROT", 400.0));
    let kd_sw_rot_g = phase_gain("KD_SWING_ROT", env_f64("KD_SWING_ROT", 40.0));
    // Multiplier on kd_sw_rot through a stair swing's drop phase (1 = off).
    let land_kd_rot_boost = env_f64("LAND_KD_ROT_BOOST", 1.0);
    // 0 = translation, 1 = +yaw, 2 = full pose, 3 = XYZ+roll+pitch (no yaw).
    let stair_absolute_td = flag("STAIR_ABSOLUTE_TD", false);
    let stair_swing_ff = flag("STAIR_SWING_FF", false);
    // Controlled ablation: retain the cubic profile and add its analytic acceleration.
    let tracking_cubic_ff = flag("TRACKING_CUBIC_FF", false);
    let tracking_ff_scale = env_f64("TRACKING_FF_SCALE", 1.0);
    // Isolation experiment: use stair rise/hold/lower timing even on flat ground.
    // LIFT_H is the full rise above lift-off when both surfaces are level.
    let stair_swing_profile = flag("STAIR_SWING_PROFILE", false);
    let swing_rot_mode_g = phase_gain("SWING_ROT", env_f64("SWING_ROT", 1.0));
    // Regulate the trunk's yaw as well as its roll and pitch.
    let trunk_yaw = flag("TRUNK_YAW", true);
    let kp_yaw = env_f64("KP_YAW", 200.0);
    let kd_yaw = env_f64("KD_YAW", 40.0);
    let mut gait = GaitPlan::new(&gp);
    // A separate hold AFTER the normal inter-step pressure transfer. Extending
    // T_DS itself changes the transfer slope and proved non-robust for fixed
    // arms: 20 ms changes swapped survival between the 0.5/0.25 ms plants.
    // Consecutive DS slices make this one a constant-ZMP segment, so it adds
    // settling time without changing the validated transfer duration.
    let ds_settle = env_f64("DS_SETTLE", 0.0);
    let ds_settle_after_step = env_f64("DS_SETTLE_AFTER_STEP", 1.0).max(1.0) as usize;
    assert!(
        ds_settle.is_finite() && ds_settle >= 0.0,
        "DS_SETTLE must be finite and non-negative"
    );
    let n_ds_settles = gait.insert_ds_settles(ds_settle, ds_settle_after_step);
    if n_ds_settles > 0 {
        println!(
            "inter-step settle: {n_ds_settles} mid-foot holds x {ds_settle:.3}s, after step {ds_settle_after_step}"
        );
    }
    // Z_COM_OFFSET [m]: the walking CoM height RELATIVE to the measured
    // initial one. `Z_COM` is an absolute world height, which on a descent
    // (spawn on a platform) silently adds the platform height to the crouch
    // -- measured 2026-10-05: Z_COM=0.66 took every descent below 6 cm to
    // 0/48. 0 = off = bit-identical.
    let z_com = env_f64("Z_COM", com0.z + env_f64("Z_COM_OFFSET", 0.0));
    // Carry a centroidal DAMPING gain across robots by the LIPM timescale.
    //
    // Task-space gains here are absolute (1/s^2 for kp, 1/s for kd), so a
    // recipe does not transfer: the kyo46rs gains put G1 on the floor in two
    // steps. Setting `GAIN_OMEGA_REF` to the omega the gains were tuned at
    // (kyo46rs: 5.379) scales kd by omega/omega_ref. 0 = off = shipped.
    //
    // Only the centroidal damping, because that is all the measurement
    // supports. Against the G1 CEM optimum (doc/from_claude/
    // 20260919_g1_transfer_probe.md), omega ratio 0.691:
    //
    //     KD_COM    predicted 55.3, found 54.5   (x0.99)
    //     KD_TRUNK  predicted 27.6, found 24.4   (x0.88)
    //     KD_SWING  predicted 27.6, found 45.0   (x1.63)  <- excluded
    //     KP_COM    predicted 143,  found 355    (x2.48)  <- excluded
    //     KP_SWING  predicted 191,  found 340    (x1.78)  <- excluded
    //     KP_TRUNK  predicted 95,   found 185    (x1.94)  <- excluded
    //
    // The stiffnesses come out near their UNSCALED values, which reads as the
    // realizable stiffness being set by torque over inertia rather than by the
    // pendulum -- so scaling them by omega is not just unsupported, it is
    // pointing the wrong way. One robot pair, one optimum: treat as a working
    // rule, not a law.
    let gain_omega_ref = env_f64("GAIN_OMEGA_REF", 0.0).max(0.0);
    if gain_omega_ref > 0.0 {
        let scale = (G / z_com).sqrt() / gain_omega_ref;
        kd_com *= scale;
        kd_trunk_g.base *= scale;
        kd_trunk_g.xfer *= scale;
        kd_trunk_g.post *= scale;
        println!(
            "GAIN_OMEGA_REF={gain_omega_ref}: omega={:.3}, centroidal kd x{scale:.3} \
             (KD_COM {kd_com:.1}, KD_TRUNK {:.1})",
            (G / z_com).sqrt(),
            kd_trunk_g.base,
        );
    }
    // How far the ZMP is asked to travel laterally, as a fraction of the
    // half stance width. 1.0 is true single support (all load on one sole);
    // anything less is only reachable with both feet down, so it is a W1
    // instrument, not a gait parameter.
    let zmp_lat_scale_g = phase_gain("ZMP_LAT_SCALE", env_f64("ZMP_LAT_SCALE", 1.0));
    let zmp_lat_scale = zmp_lat_scale_g.base;
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
    // Speed-change benchmark: from step VX2_AT the forward speed ramps to
    // VX2 over VX2_RAMP steps (gait::FootstepPlan::velocity_with_change).
    // Unset = the fixed-speed plan, bit-identical.
    let vx2_change = std::env::var("VX2").ok().and_then(|v| v.parse::<f64>().ok()).map(|vx2| {
        (env_f64("VX2_AT", 8.0).max(0.0) as usize, vx2, env_f64("VX2_RAMP", 3.0).max(0.0) as usize)
    });
    // VX2_STRIDE_MAX [m/step]: a speed the fixed step time would turn into a
    // longer stride than this is reached by STEPPING FASTER instead -- the
    // footfalls are planned at the capped stride and each step from VX2_AT
    // on (its single support and the double support after it) is shortened
    // by stride_max / (v * t_step). Measured 2026-10-03 on G1 v6.3 with the
    // step time fixed at 1.21 s: 0.25 m/s 47/48, 0.30 m/s 12/48, 0.35 m/s
    // 1/48 -- the stride grows to 0.36-0.42 m and the landing leg is at
    // 99.8% of its length. VX2_TSCALE_MIN bounds the shortening. 0 = off.
    let vx2_stride_max = env_f64("VX2_STRIDE_MAX", 0.0).max(0.0);
    let vx2_tscale_min = env_f64("VX2_TSCALE_MIN", 0.4).clamp(0.1, 1.0);
    // VX2_SCHED: ONE speed schedule instead of the two hand-set knobs.
    // Measured 2026-10-14 (doc/from_claude/20261014_speed_design.md): the
    // swing's peak acceleration goes as v / T_step, so a speed is reached
    // by the LONGEST stride the legs allow (cap VXS_STRIDE_MAX) and only
    // then by cadence; a longer stride needs a lower CoM to stay within
    // leg reach, z_drop = 0.024 + VXS_Z_SLOPE (stride - 0.35) above a
    // 0.35 m stride, fading to 0 below 0.25 m. Reproduces the measured
    // settings (0.35/0.024 to 0.50 m/s, 0.45/0.06 at 0.55). The swing
    // acceleration bound v / T <= 0.7 then puts the ceiling at ~0.56 m/s
    // with a 0.45 m stride. Unset = the two knobs, bit-identical.
    let vx2_sched = flag("VX2_SCHED", false);
    // VX2_REANCHOR: at every entry into single support, translate the rest
    // of the footstep plan so the stance foot's planned position is where
    // the foot actually is. The plan is a position-vs-time schedule at the
    // commanded speed; after a recovery stalls the robot it runs ahead and
    // the next landings become lunges (measured 2026-10-14 at 0.30 m/s
    // after a lateral-push reference: a 0.65 m step, then the fall). Only
    // with a speed command (VX2); unset = bit-identical.
    let vx2_reanchor = flag("VX2_REANCHOR", false);
    let mut n_reanchor = 0u32;
    let mut reanchor_total = 0.0_f64;
    let (vx2_stride_max, vx2_z_drop_sched) = match vx2_change {
        Some((_, vx2, _)) if vx2_sched => {
            // The cap itself rises with speed: a 0.42 m stride at the
            // nominal cadence (0.35 m/s) stretches the stance leg over a
            // 0.84 s single support and falls even 5 cm lower, while the
            // same stride at 0.55 m/s (0.82 s step) walks. 0.35 m up to
            // VXS_V_LONG, then +VXS_CAP_SLOPE per m/s to VXS_STRIDE_MAX.
            let cap_hi = env_f64("VXS_STRIDE_MAX", 0.45).max(0.05);
            let cap = (0.35 + env_f64("VXS_CAP_SLOPE", 2.0) * (vx2 - env_f64("VXS_V_LONG", 0.45))).clamp(0.35, cap_hi);
            let stride = (vx2 * t_step).min(cap);
            let zd = if stride >= 0.35 {
                0.024 + env_f64("VXS_Z_SLOPE", 0.36).max(0.0) * (stride - 0.35)
            } else {
                0.024 * ((stride - 0.25) / 0.10).clamp(0.0, 1.0)
            }
            .min(env_f64("VXS_Z_MAX", 0.08))
            // VXS_Z_EXTRA: more drop on top of the schedule (the landing leg
            // at speed is at 0.99 of its length and has no lateral authority
            // against a push; measured 2026-10-14). 0 = the schedule alone.
            + env_f64("VXS_Z_EXTRA", 0.0).max(0.0);
            println!("speed schedule: v={vx2:.2} m/s -> stride {stride:.3} m (cap {cap:.2}), step {:.3} s, CoM drop {zd:.3} m", stride / vx2.max(1e-6));
            (cap, Some(zd))
        }
        _ => (vx2_stride_max, None),
    };
    // VX2_DS_SCALE: an extra factor on the DOUBLE-support slices of the
    // sped-up steps. Measured 2026-10-14 at 0.55 m/s with a 0.45 m stride
    // (mu 0.60): at the end of the double support the trailing foot is
    // 0.45 m behind the CoM, its force needs a shear/normal ratio of ~0.7,
    // and it slips 5 cm backward at toe-off; the next landing then falls
    // short. Unloading it sooner (a shorter DS) keeps the trailing force
    // inside the cone. 1 = the shipped equal scaling, bit-identical.
    let vx2_ds_scale = env_f64("VX2_DS_SCALE", 1.0).clamp(0.1, 1.0);
    let vx2_change = match vx2_change {
        Some((at, vx2, ramp2)) if vx2_stride_max > 0.0 && vx2 * t_step > vx2_stride_max => {
            // Per-step time scale along the ramp, then the capped plan speed.
            let mut k = 0usize;
            let mut i = 0usize;
            while i < gait.slices.len() {
                if matches!(gait.slices[i].support, Support::Single { .. }) {
                    if k >= at {
                        let g = if ramp2 == 0 { 1.0 } else { ((k - at + 1) as f64 / ramp2 as f64).min(1.0) };
                        let v_k = vx + (vx2 - vx) * g;
                        let scale = (vx2_stride_max / (v_k * t_step)).clamp(vx2_tscale_min, 1.0);
                        // The single support and EVERY double-support slice
                        // after it (transfer + DS_SETTLE hold).
                        let mut j = i;
                        while j < gait.slices.len()
                            && (j == i || matches!(gait.slices[j].support, Support::Double))
                        {
                            let t0 = gait.slices[j].t0;
                            let d = gait.slices[j].duration() * scale * if j == i { 1.0 } else { vx2_ds_scale };
                            gait.retime(j, t0 + d);
                            j += 1;
                        }
                    }
                    k += 1;
                }
                i += 1;
            }
            println!("speed change: stride capped at {vx2_stride_max:.3} m, steps from {at} shortened (>= x{vx2_tscale_min:.2})");
            // The position plan advances `v * t_step` per step, so the capped
            // stride is the speed stride_max / t_step in the plan's units.
            Some((at, (vx2_stride_max / t_step).max(vx), ramp2))
        }
        other => other,
    };
    // VX2_Z_DROP [m]: lower the walking CoM height by this much from one step
    // before the speed change on. Measured 2026-10-05: the 0.45 m/s wall is
    // leg reach (stance leg at 99%, the heel lifts and the foot slips), and
    // 24 mm lower takes it from 7/48 to 48/48 -- but lowering the CoM for
    // the whole run trades descents and lateral pushes away. Tied to the
    // speed command it leaves every other bench bit-identical. 0 = off.
    let vx2_z_drop = vx2_z_drop_sched.unwrap_or_else(|| env_f64("VX2_Z_DROP", 0.0).max(0.0));
    let mut vx2_z_track = 0.0_f64;
    let push_z_drop = env_f64("PUSH_Z_DROP", 0.0).max(0.0);
    let push_z_drop_trig = env_f64("PUSH_Z_DROP_TRIG", 0.03).max(0.0);
    let push_z_drop_t = env_f64("PUSH_Z_DROP_T", 1.5).max(0.0);
    let push_z_drop_lat_max = env_f64("PUSH_Z_DROP_LAT_MAX", 0.02).max(0.0);
    let mut push_z_drop_until = f64::NEG_INFINITY;
    let mut n_push_z_drop = 0u32;
    let mut push_back_hist: std::collections::VecDeque<(f64, f64)> = std::collections::VecDeque::new();
    let vx2_z_from: Option<f64> = match vx2_change {
        Some((at, _, _)) if vx2_z_drop > 0.0 => gait
            .slices
            .iter()
            .filter(|sl| matches!(sl.support, Support::Single { .. }))
            .nth(at.saturating_sub(1))
            .map(|sl| sl.t0),
        _ => None,
    };
    let mut footsteps = if flag("ALTERNATE", true) {
        FootstepPlan::velocity_with_change(&gait, &steps, vx, vy, wz, t_step, stride_ramp, vx2_change)
    } else {
        FootstepPlan::ramped_stride(&gait, &steps, vx * t_step, stride_ramp)
    };
    // Stairs: rebuild the flat-ground footstep plan at tread centres. Terrain
    // enters the controller HERE and nowhere else -- the ZMP plan, the swing
    // touchdown, the per-slice anchors and the support polygon all read this
    // plan, so one edit moves all of them consistently.
    if let Some(st) = &stairs {
        // Rebuild the on-stairs part of the plan DETERMINISTICALLY instead
        // of snapping the velocity plan's wishes to the tread grid. Snapping
        // was tried first and quantised different nominal targets onto the
        // same spot: the approach degenerated into stepping against the
        // first riser and even a 20 mm riser fell. The sequence a stair
        // wants is an approach followed by alternating feet on successive
        // treads, with one riser of body advance per step.
        // Toe margin to a riser face. 10 mm was not enough: the heel-to-toe
        // pressure roll pushes the stance toe forward and it ground into the
        // first riser at 46 N while the climb was being launched off it.
        // The 20 mm default is 41% of kyo46rs's 49 mm sole half-length but only
        // 24% of G1's 85 mm, so the same number means two different things.
        // Measured 2026-09-20 on G1: the last approach footfall lands ON this
        // stand line, which puts the swing toe 20 mm from the riser face -- the
        // ankle advances 20 mm and the toe is already at the riser while the
        // foot has barely left the ground. Touchdown was detected with the foot
        // still RISING (td_vz +0.114), DCM error went 0.0042 -> 0.1857, cop_use
        // saturated and fz nearly doubled. Overridable so a longer-footed robot
        // can buy the clearance; the default keeps kyo46rs bit-identical.
        let m = sole_half_l + env_f64("STAIR_TOE_MARGIN", 0.020);
        let a_pos = st.start - m; // stand line: toes just clear of riser 1
        // Widen the on-stairs footfalls. Foot-over-foot passes the swing
        // shank beside the stance foot at tread height, and the measured
        // clearance was negative: left_shank <-> right_foot at 75 N on the
        // second climb. Walking a little wide opens it.
        let y_out = env_f64("STAIR_Y_OUT", 0.010);
        let stair_gait_pair = std::env::var("STAIR_GAIT")
            .map(|v| v == "pair")
            .unwrap_or(false);
        let n = st.n as i64;
        let land = st.start + st.n as f64 * st.tread + m;
        // Aim tread landings BEHIND the centre. The measured touchdown
        // overshoots its target by ~45 mm in x (travel-phase momentum); on
        // the one-step case the tread and the landing form one continuous
        // surface and the overshoot was harmless, but on a real flight each
        // tread is a 110 mm island and centre+45 is the nosing. Biasing the
        // TARGET rearward spends the sole's rear margin on the overshoot.
        let mut td_bias = env_f64("STAIR_TD_BIAS", 0.0);
        // Descent-only rearward bias. Measured 2026-10-03 on G1 (v6.2, two
        // steps down, 60 mm risers, 29/48): the step-down landing overshoots
        // its goal by ~21 mm in x, a 170 mm sole on a 211 mm tread has 20 mm
        // to spare, so the toe contact spheres land PAST the nosing and the
        // stance foot pitches over the edge during the next swing. Default =
        // STAIR_TD_BIAS, i.e. bit-identical.
        // Applied only to treads that HAVE a nosing ahead (levels 1..n-1):
        // the last level of a descent is the ground, and biasing it rearward
        // walks the heel toward the riser face behind for nothing (measured:
        // one step down 192 -> 188, 150 mm 18 -> 0 with the bias on it).
        // Ascent twin of the descent bias: treads with a RISER AHEAD (levels
        // 1..n-1 of a flight going up). Measured 2026-10-03 on G1 v6.3, three
        // steps up: the landing overshoots by ~30 mm, the toe ends past the
        // tread (20 mm to spare) and comes down ON the next step -- at 4 and
        // 6 cm the sole rests half on it (centre 20 mm high, 0/48), at 8 cm
        // the riser face stops the toe (48/48). Default = STAIR_TD_BIAS, so
        // the one-step benches are bit-identical.
        let td_bias_asc_mid = env_f64("STAIR_TD_BIAS_ASC_MID", td_bias);
        let td_bias_desc = if st.riser < 0.0 || st.down_n > 0 {
            env_f64("STAIR_TD_BIAS_DESC", td_bias)
        } else {
            td_bias
        };
        if flag("STAIR_FULL_SOLE", false) {
            let margin = env_f64("STAIR_SOLE_MARGIN", 0.002);
            let available = 0.5 * st.tread - sole_half_l - margin;
            assert!(
                margin.is_finite() && margin >= 0.0 && available >= 0.0,
                "tread cannot contain the requested full sole and margin"
            );
            let requested = td_bias;
            td_bias = td_bias.clamp(-available, available);
            println!("full-sole plan: bias {requested:.6} -> {td_bias:.6} m, margin {margin:.4} m");
        }
        // Stride the walk-out takes on the landing, as a fraction of the tread.
        // Half a tread is 1.31x kyo46rs's 42 mm cruise stride (an acceleration)
        // but only 0.43x G1's 242 mm (a halving in one step). Measured
        // 2026-09-20 on G1: with the stair itself removed (riser 0.000) the run
        // still fell at 3.98 m -- on the landing, at exactly this deceleration
        // -- while the same recipe without STAIRS walked 6.23 m. `pos_of` and
        // `lvl_of` must read the SAME value or the grade and its inverse
        // disagree. The default keeps kyo46rs bit-identical.
        let landing_step = env_f64("STAIR_LANDING_FRAC", 0.5).max(1e-3) * st.tread;
        let pos_of = |lvl: i64| -> f64 {
            if lvl <= 0 {
                a_pos
            } else if lvl > n {
                // KEEP WALKING on the landing at half-tread strides instead
                // of pinning both feet to one point. The pinned version made
                // every run that reached the top fall THERE: the climb ends
                // at ~0.15 m/s and a frozen dwell asks that momentum to die
                // in one step -- measured, all eight one-step-ascent
                // configurations fell at x = 0.43..0.49, i.e. on the
                // landing, not on the stair.
                land + (lvl - n - 1) as f64 * landing_step
            } else {
                st.start + (lvl as f64 - 0.5) * st.tread
                    - if st.riser < 0.0 && lvl < n {
                        td_bias_desc
                    } else if st.riser > 0.0 && lvl < n {
                        td_bias_asc_mid
                    } else {
                        td_bias
                    }
            }
        };
        let lvl_of = |x: f64| -> i64 {
            // Landing positions grade upward too, or the walk-out targets
            // would converge on one point and re-create the pinned stop.
            if x >= st.start + st.n as f64 * st.tread {
                // `pos_of` and this inverse must agree on the positions
                // `pos_of` itself generates, and in floating point they do not:
                // measured 2026-09-22, `pos_of(3)` produced a position whose
                // `(x - land) / landing_step` evaluates to 0.999999999999999,
                // so `floor` returned 0 and the point mapped back to level 2.
                // The next swing then aimed at `pos_of(3)` again and BOTH feet
                // were pinned to one x -- the walk-out stop this grading exists
                // to prevent. Nudging by an epsilon before the floor makes the
                // round trip exact for every level (verified 2..5).
                return n + 1
                    + (((x - land) / landing_step + 1e-9).floor() as i64).max(0);
            }
            if x < st.start - 0.5 * m {
                0
            } else if x >= st.start + st.n as f64 * st.tread {
                n + 1
            } else {
                (((x - st.start) / st.tread) as i64 + 1).clamp(1, n)
            }
        };
        // Land one foot ON the stand line before the first tread, instead of
        // jumping to the tread from wherever the approach grid happens to stop.
        //
        // OPT-IN (default off): the comment below records that squaring up was
        // measured to fell kyo46rs, and that result stands for kyo46rs's
        // geometry, where the un-squared jump is already small. G1 is the case
        // it was not measured on. Measured 2026-09-21: G1 cruises at 242 mm,
        // its last approach footfall lands at 1.739 m and tread 1 sits at
        // 2.106 m, so the un-squared jump is 367 mm -- half again the stride
        // the machine has been shown to take (242 mm, 16/16 under the 0.5 ms
        // audit). Squaring splits it into 141 mm and 226 mm, both inside that.
        // The knob exists so the trade (one short step vs one oversized one)
        // is a measured choice per robot rather than a constant.
        let align_stand_line = flag("STAIR_ALIGN_STAND_LINE", false);
        // Up-and-over course (STAIRS_DOWN): the footfall positions by level,
        // walked with explicit per-foot levels instead of the `lvl_of`
        // inverse. Level 0 = stand line, 1..n = up treads, then the landing
        // at `landing_step` strides up to a second stand line before the
        // descending flight, the down treads (rearward bias on the ones with
        // a nosing ahead), and the walk-out below.
        let course: Option<Vec<f64>> = (st.down_n > 0).then(|| {
            let le = st.landing_end();
            // Stand line before the descending flight: the toe must stay
            // short of the landing's edge. Measured 2026-10-03 (4 cm course,
            // v6 and v6.2 alike): with the toe AT the edge (the ascent's
            // STAIR_TOE_MARGIN=0 reused) the landing overshoots by 7 mm, the
            // toe spheres hang over the edge and the sole centre sinks 20 mm.
            let stand2 = le - sole_half_l - env_f64("STAIR_DESC_STAND_MARGIN", 0.03).max(0.0);
            let mut v = vec![a_pos];
            for k in 1..=n {
                v.push(pos_of(k));
            }
            let mut x = land;
            while x < stand2 - 0.5 * landing_step {
                v.push(x);
                x += landing_step;
            }
            v.push(stand2);
            for k in 1..=st.down_n {
                let nosing_ahead = k < st.down_n;
                v.push(
                    le + (k as f64 - 0.5) * st.tread
                        - if nosing_ahead { td_bias_desc } else { td_bias },
                );
            }
            let last = *v.last().unwrap();
            for j in 1..=60 {
                v.push(last + j as f64 * landing_step);
            }
            v
        });
        let mut course_lvl = [0usize; 2];
        let mut aligned = false;
        let mut cur = footsteps.initial;
        let mut ladder: Vec<(usize, f64, f64)> = Vec::new();
        for (i, sl) in gait.slices.iter().enumerate() {
            if let Support::Single { swing, .. } = sl.support {
                let stance = 1 - swing;
                let nom_x = footsteps.at[i].sole[swing].x;
                // FOOT OVER FOOT, with no pause at the bottom: the first
                // swing whose nominal target would cross the stand line goes
                // straight to tread 1, and every later one to the level
                // above its stance foot -- alternate feet on alternate
                // treads. Two gaits were tried and rejected first: pair-up
                // (both feet per tread) alternates a 110 mm ZMP jump with a
                // 0 mm one, a stop-and-go CoM shuttle whose amplitude does
                // not fit on a 98 mm sole; and squaring both feet up on a
                // stand line before the first riser inserts the same
                // stop-start at the bottom -- measured, THAT is what felled
                // even a 20 mm riser, height-independently.
                // STAIR_GAIT=pair revisits the rejected gait WITH the tools
                // that did not exist at rejection time: ZMP_ROLL bridges
                // half the dwell jump inside the sole, and the measured
                // killer of foot-over-foot is its 220-270 mm swings -- every
                // one lands short, high and late (dead-stretched leg waiting
                // for the body), and the next lift-off inherits the mess.
                // Pair-up halves the swing to ~110 mm.
                let tx = if nom_x < a_pos - 1e-9 {
                    nom_x // flat approach: the velocity plan's own wish
                } else if align_stand_line && !aligned {
                    // First swing that would overshoot the stand line: put it
                    // ON the line. Only ever fires once; the next swing takes
                    // the tread from a known start, so both halves of the
                    // entry are bounded by the stand-line offset and half a
                    // tread rather than by where the approach grid landed.
                    aligned = true;
                    a_pos
                } else if let Some(v) = &course {
                    let l = (course_lvl[stance] + 1).min(v.len() - 1);
                    course_lvl[swing] = l;
                    v[l]
                } else if stair_gait_pair {
                    let sk = lvl_of(cur.sole[stance].x);
                    let wk = lvl_of(cur.sole[swing].x);
                    if wk < sk { pos_of(sk) } else { pos_of(sk + 1) }
                } else {
                    pos_of(lvl_of(cur.sole[stance].x) + 1)
                };
                cur.sole[swing].x = tx;
                cur.sole[swing].y = footsteps.at[i].sole[swing].y
                    + if nom_x < a_pos - 1e-9 {
                        0.0
                    } else if swing == 0 {
                        y_out
                    } else {
                        -y_out
                    };
                cur.sole[swing].z = st.h(tx);
                cur.yaw[swing] = footsteps.at[i].yaw[swing];
                ladder.push((swing, tx, cur.sole[swing].z));
            }
            footsteps.at[i] = cur;
        }
        let s: Vec<String> = ladder
            .iter()
            .map(|(sw, x, z)| format!("{}({:.3},{:.3})", if *sw == 0 { "L" } else { "R" }, x, z))
            .collect();
        println!("stairs footfalls: {}", s.join(" "));
    }
    // Reserve swing time before the approach DCM is generated. Waiting until
    // touchdown to discover a longer step can already put the DCM beyond the
    // stance sole's capture range. Optionally also reserve time for long
    // same-height swings, including the transition onto the final landing.
    if flag("COM_PREPLAN", flag("COM_REPLAN", false)) {
        let vx_limit = env_f64("COM_LAND_VXY", 0.35).max(0.01);
        let vz_limit = env_f64("COM_LAND_VZ", 0.20).max(0.01);
        let max_ss = env_f64("COM_STAIR_SS_MAX", 0.80).max(gp.t_ss);
        // Stretch the double support preceding each DESCENT swing to at
        // least this many seconds (0 = off).
        let com_desc_ds = env_f64("COM_DESC_DS", 0.0);
        // Stretch the double support AFTER each descent swing: the toe has
        // touched, and the ZMP transfer must crawl while the sole rolls
        // flat under light load (0 = off).
        let com_desc_xfer = env_f64("COM_DESC_XFER", 0.0);
        let xy_min = env_f64("COM_PREPLAN_XY_MIN", f64::INFINITY);
        for i in 1..gait.slices.len() {
            if let Some(side) = gait.slices[i].support.swing() {
                let delta = footsteps.at_slice(i).sole[side] - footsteps.at_slice(i - 1).sole[side];
                if delta.z.abs() > 0.005 || delta.xy().norm() > xy_min {
                    let duration =
                        (delta.xy().norm() / vx_limit).max(delta.z.abs() / vz_limit) + 0.05;
                    let new_t1 = gait.slices[i].t0 + duration.clamp(gp.t_ss, max_ss);
                    // Say so. The landing-speed sweep that motivated this
                    // line produced four bit-identical falls, and whether
                    // the extension had fired at all was not observable.
                    println!(
                        "preplan: slice {i} swing {} d=({:.3},{:+.3}) SS {:.2} -> {:.2}s",
                        if side == 0 { "L" } else { "R" },
                        delta.xy().norm(),
                        delta.z,
                        gait.slices[i].duration(),
                        new_t1 - gait.slices[i].t0
                    );
                    gait.retime(i, new_t1);
                    // A DESCENT swing also buys time in the DOUBLE support
                    // BEFORE it: the pelvis pre-lowers a full riser there on
                    // both knees (see the height reference's look-ahead),
                    // and at the stock 0.35 s the rate limit only yields
                    // ~17 mm of the 52. A person does the same thing --
                    // pauses at the nosing, knees bending, then steps down.
                    if delta.z < -0.005 && com_desc_xfer > 0.0 && i + 1 < gait.slices.len() {
                        let next_end = gait.slices[i + 1].t0
                            + gait.slices[i + 1].duration().max(com_desc_xfer);
                        gait.retime(i + 1, next_end);
                        println!(
                            "preplan: slice {} DS after descent -> {:.2}s",
                            i + 1,
                            gait.slices[i + 1].duration()
                        );
                    }
                    if delta.z < -0.005 && com_desc_ds > 0.0 && i >= 1 {
                        let prev_end =
                            gait.slices[i - 1].t0 + gait.slices[i - 1].duration().max(com_desc_ds);
                        gait.retime(i - 1, prev_end);
                        println!(
                            "preplan: slice {} DS before descent -> {:.2}s",
                            i - 1,
                            gait.slices[i - 1].duration()
                        );
                    }
                }
            }
        }
    }
    // The LIPM height is the CoM's height ABOVE THE WALKING SURFACE, which
    // on a descent platform is not its world z.
    // The LIPM height must be the CoM's height above the walking surface AS
    // FLOWN, which includes the stair crouch: omega = sqrt(g/z) and the crouch
    // is up to 90 mm of a 350 mm height, so ignoring it leaves the DCM plan
    // 10-17% slow exactly when the machine is lowest and least stable.
    let stair_crouch = env_f64("STAIR_CROUCH", 0.0);
    let z_lip = (z_com
        - stairs.as_ref().map_or(0.0, |st| st.base())
        - env_f64("STAIR_CROUCH", 0.0).max(0.0))
    .max(0.05);
    // Heel-to-toe travel of the planned pressure inside each stance (see
    // DcmPlan::from_footsteps_rolled). On for stairs by default: the tread
    // pitch fixes the footstep pitch at ~2x what the commanded speed builds
    // momentum for, and the sole's own length is where the difference has
    // to come from. Must stay inside the CoP box (cop_half.0 * COP_FRAC).
    let zmp_roll = env_f64("ZMP_ROLL", if stairs.is_some() { 0.030 } else { 0.0 });
    // A finite DCM preview makes early walking references independent of the
    // requested total step count. Zero retains the full terminal-rest plan.
    let dcm_preview_horizon = env_f64("DCM_PREVIEW_HORIZON", 0.0);
    // Continuity guard on the DCM reference across plan rebuilds.
    //
    // `dcm_plan` is rebuilt at a dozen sites (contact-driven retime, landing
    // extension, footstep adaptation, transfer arming, ...). The rebuilt plan
    // is continuous in its OWN timeline, but when the rebuild moves a slice
    // boundary to before `now` the reference read at `now` switches from the
    // old slice's value to the new slice's value in one tick. Measured
    // 2026-09-23 on G1 (T=40, landing 3.0 m, N=48): at the tick the swing
    // foot touched the tread the single support was cut by contact and
    // `xi_ref` went from over the stance foot (1.896, +0.073) to the
    // double-support rest point (1.998, +0.003) -- 12 cm in 5 ms. A one-tick
    // jump above 5 cm separated fallers from survivors at every riser, with
    // and without the stair transfer: 23/25 vs 0/23 at 40 mm (transfer off),
    // 21/23 vs 2/25 (transfer on). `commanded_zmp` turns the jump into a
    // saturated ZMP command and the DCM error never recovers.
    //
    // With `DCM_REF_BLEND_T > 0`, a jump larger than `DCM_REF_BLEND_MIN` is
    // not applied at once: the reference is blended from the pre-jump value
    // to the plan's value over that many seconds (smoothstep), and `p` is
    // re-derived from the blended `xi`. Default 0 = off, every recipe
    // reproduces bit for bit.
    let dcm_ref_blend_t = env_f64("DCM_REF_BLEND_T", 0.0).max(0.0);
    let dcm_ref_blend_min = env_f64("DCM_REF_BLEND_MIN", 0.02).max(0.0);
    let dcm_plan =
        DcmPlan::from_footsteps_rolled(&gait, &footsteps, z_lip, zmp_lat_scale, zmp_roll);
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
        steps.sole[0].x,
        steps.sole[0].y,
        steps.sole[1].x,
        steps.sole[1].y,
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

    let mut contact_cfg = bt::ContactCfg {
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
        "xi_x",
        "xi_y",
        "xi_ref_x",
        "xi_ref_y",
        "zmp_ref_x",
        "zmp_ref_y",
        "zmp_cmd_x",
        "zmp_cmd_y",
        "zmp_clamp",
        "dcm_err",
        "support",
        "step",
        "share_l",
        "share_r",
        "swing_tgt_x",
        "swing_tgt_y",
        "swing_tgt_z",
        "ankle_roll_l",
        "ankle_roll_r",
        // The disturbance itself, so a replay can DRAW it instead of the
        // viewer having to be told separately what was applied. Zero except
        // while a pulse is live.
        "push_fx",
        "push_fy",
        // Contact with `SPAR_WALL`'s target: this tick's force, the impulse
        // delivered by the punch in progress, and the run total.
        "hit_f",
        "hit_imp_punch",
        "hit_imp_total",
        // Pre-step FK, time-aligned with measured contact forces and goals.
        "sole_l_x",
        "sole_l_y",
        "sole_l_z",
        "sole_r_x",
        "sole_r_y",
        "sole_r_z",
        "anchor_l_y",
        "anchor_r_y",
        "anchor_err_l_y",
        "anchor_err_r_y",
        "foot_v_l_x",
        "foot_v_l_y",
        "foot_v_r_x",
        "foot_v_r_y",
        "contact_activation_l",
        "contact_activation_r",
        "contact_force_activation_l",
        "contact_force_activation_r",
        "load_chatter_ema",
        "goal_l_x",
        "goal_l_y",
        "goal_l_z",
        "goal_r_x",
        "goal_r_y",
        "goal_r_z",
        "com_replan",
        "preview_x",
        "preview_y",
        "preview_vx",
        "preview_vy",
        "preview_terminal_error",
        "preview_solves",
        "preview_failures",
        "landing_delay",
        "stair_transfer",
        "stair_transfer_phase",
        "stair_transfer_alpha",
        "stair_transfer_high",
        "ffdc_dz",
        "ffdc_err",
        "capture_error",
        "capture_tilt",
        "trunk_roll",
        "trunk_pitch",
        "trunk_cmd_pitch",
        "trunk_acc_pitch",
        "trunk_cmd_roll",
        "trunk_acc_roll",
        "transfer_guard",
        "wrench_mpc",
        "wrench_mpc_fz_l",
        "wrench_mpc_fz_r",
        "mpc_cop_x",
        "mpc_cop_y",
        "mpc_fs_x",
        "mpc_fs_y",
        "wbc_cop_x",
        "wbc_cop_y",
        "wrench_mpc_solves",
        "wrench_mpc_failures",
        "swing_pos_err_xy",
        "swing_pos_err_z",
        "swing_task_residual_rms",
        "swing_res_z",
        "swing_res_rows",
        "swing_cmd_acc_z",
        "swing_achieved_acc_z",
        "tau_use_max",
        "tau_saturated_count",
        // Leg reach (src/biped/reach.rs): |hip - sole| / leg length, per side,
        // and the hip heights the bound is computed from. Zero when the
        // profile has no measured leg. Logged unconditionally because every
        // reach CLAIM about a fall is worthless without it.
        "reach_l",
        "reach_r",
        "hip_l_z",
        "hip_r_z",
        "stride_max",
    ];
    let mut log = TrajLog::create(
        std::env::var("TRAJ_CSV").ok(),
        prof.log_joints.to_vec(),
        &extra_cols,
    );
    let mut tick_audit = std::env::var("TICK_AUDIT_CSV")
        .ok()
        .map(|path| articara::biped::tick_audit::TickAudit::create(&path, &rig));
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
    let mut correction =
        ContactCorrection::new(total_mass * G, env_f64("CONTACT_TICKS", 2.0) as u32);
    let contact_on_bw = env_f64("CONTACT_ON_BW", 0.10);
    assert!(
        contact_on_bw.is_finite() && contact_on_bw > 0.02 && contact_on_bw <= 1.0,
        "CONTACT_ON_BW must exceed the 0.02 BW release threshold and be <= 1"
    );
    correction.on_n = contact_on_bw * total_mass * G;
    correction.scheduled_reentry = flag("CONTACT_SCHEDULED_REENTRY", false);
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
    // P2-1 contact planner (doc/from_claude/20260923_p2_contact_planner_design_v2.md).
    // Predictive replacement for ADAPT_STEP/ADAPT_TIME: picks the landing
    // shift AND the end of the current single support so the predicted
    // end-of-step DCM meets the plan's. Default OFF. Keeps its own state --
    // never shares `adapt_now`/`adapt_applied` (the A/B would be polluted).
    let contact_plan_on = flag("CONTACT_PLAN", false);
    let contact_plan_cfg = articara::biped::contact_plan::Config {
        t_min_frac: env_f64("CP_T_MIN", 0.5),
        // v0.1: never EXTEND the single support -- a long single support is
        // mechanism B itself. Measured v0 (T_max 1.5): the planner fired only
        // in already-disturbed runs and then retimed every tick (median 139-479
        // retimes per run); those runs all fell, the quiet ones survived ~95%.
        t_max_frac: env_f64("CP_T_MAX", 1.0),
        dt_grid: env_f64("CP_DT", 0.01).max(0.001),
        max_x: env_f64("CP_MAX_X", adapt_max_x),
        max_y: env_f64("CP_MAX_Y", adapt_max_y),
        max_y_in: env_f64("CP_MAX_Y_IN", env_f64("CP_MAX_Y", adapt_max_y)),
        timing_from_x: flag("CP_T_FROM_X", false),
        timing_lat_outward: flag("CP_T_LAT_OUT", false),
        j: env_f64("CP_J", 0.5).max(0.05),
        w_shift: env_f64("CP_W_SHIFT", 0.0).max(0.0),
        w_u: env_f64("CP_W_U", 1.0),
        w_t: env_f64("CP_W_T", 1.0),
        w_res: env_f64("CP_W_RES", 100.0),
        dead_u: env_f64("CP_DEAD_U", 0.002),
        dead_t: env_f64("CP_DEAD_T", 0.005),
    };
    // Only plan when the DCM is actually off the plan (same reason as
    // ADAPT_TIME_TRIG: the undisturbed gait must reproduce bit for bit).
    let contact_plan_trig = env_f64("CP_TRIG", 0.015);
    let cp_track = flag("CP_TRACK", false);
    let cp_j_auto = flag("CP_J_AUTO", false);
    let cp_touchdown = flag("CP_TOUCHDOWN", false);
    let mut footsteps = footsteps;
    // Terrain height under the support, slewed (below) so the CoM height
    // reference climbs a riser over about a step instead of jumping at a
    // slice boundary. Starts at the spawn platform's height.
    let mut terrain_track = stairs.as_ref().map_or(0.0, |st| st.base());
    let terrain_base = terrain_track;
    // Risers per step the height reference is allowed to move. Small spreads
    // one riser's climb over several steps; large finishes it within the
    // swing that takes the tread.
    let stair_climb_rate = env_f64("STAIR_CLIMB_RATE", 1.5);
    // Descent: ride the minimum of the two planned sole heights (see the
    // height-reference block). Off by default for lineage comparability.
    let stair_desc_min = flag("STAIR_DESC_MIN", false);
    // Crouch while on stairs. Measured on the one-step mount: the swing
    // target sits 212 mm from the hip -- the leg's full thigh+shank length
    // -- so the foot hovered 40 mm short waiting for the body, landed after
    // the schedule, and the anchor slam felled the run. Lowering the hip
    // 25 mm trades that dead-stretched reach for ~+39 mm of horizontal
    // range, the way a person bends their knees on a staircase. Slewed in
    // so the reference does not step.
    // OPT-IN (default 0), not stairs-conditional: the bench's `flat` case
    // keeps a staircase parked at x = 5 m, so a stairs-conditional default
    // silently crouched every flat baseline -- measured by the Codex
    // thread, it felled their reference runs before they pinned it to 0.
    let mut crouch_track = 0.0_f64;
    // Phase windows of the stair swing, `rise_end,travel_start,travel_end,
    // drop_start` (see gait::stair_swing_windows). The default is the
    // shipped profile.
    // Landing overdrive of the stair swing (metres COMMANDED below the
    // surface). See gait::stair_swing_sink.
    let stair_td_sink_g = phase_gain("STAIR_TD_SINK", env_f64("STAIR_TD_SINK", 0.004));
    let stair_land_sink = env_f64("STAIR_LAND_SINK", 0.004).max(0.0);
    // Toe-first descent landing: pitch the swing sole toe-down by this many
    // radians through the DROP phase of a DESCENDING stair swing. The toe
    // then meets the lower tread first (plantarflexion buys ~half_l*sin
    // of reach) and the sole rolls flat as load transfers, the way a person
    // steps down. 0 = off.
    let stair_desc_toe = env_f64("STAIR_DESC_TOE", 0.0);
    // Anchor landings at the PLAN rotation (flat sole) instead of the
    // measured one. Needed for toe-first descent landings (a measured
    // anchor freezes the sole on its toe); breaks the validated ascent
    // recipes if applied there, so strictly opt-in.
    let stair_anchor_flat = flag("STAIR_ANCHOR_FLAT", false);
    // Shrink the CoP box to the contact edge while a sole is pitched
    // (rolling contact). See contact_level's patch_mod. Opt-in.
    let stair_roll_patch = flag("STAIR_ROLL_PATCH", false);
    // Damp the LOAD SPLIT between two stance feet. Measured on the fixed-arm
    // stair transfer: with both feet down at different heights the QP has a
    // redundant load distribution, and it oscillates -- fz went
    // 15/55 -> 79/17 -> 26/67 N on a ~50 ms period, the CoM stopped rising
    // and the robot toppled. Free arms damp this implicitly (load-share
    // changes 3.6%/tick vs 9.0%/tick fixed), which is what the arms were
    // really doing for stairs. The split lives in the NULL SPACE of the
    // contact/CoM tasks, so a term just above the regulariser reaches it
    // without competing with them. Reference is the previous tick's MEASURED
    // normal force, so it damps rather than commands.
    //
    // FALSIFIED, kept only so the experiment is not repeated. Both forms are
    // WORSE than undamped at every weight tried (0.05 / 0.2 / 0.5):
    //   - first difference (track the previous fz): resists the INTENDED
    //     transfer too; fell at 2.9-7.9 s against a 13.0 s baseline.
    //   - second difference (track 2*fz_prev - fz_prev2, so a steady ramp is
    //     free): differentiating a measured contact force twice amplifies its
    //     noise; weight 0.5 could not even stand (fell at 0.5 s).
    // A load-split damper needs a reference that is not built from measured
    // force differences -- a planned share, or a filtered state estimate.
    let contact_fz_rate = env_f64("CONTACT_FZ_RATE", 0.0).max(0.0);
    // Planned load-split damping. Move the low-priority WBC fz target from
    // the previous WBC solution toward the smooth MPC command by at most this
    // fraction of body weight per tick. This uses neither a hard constraint
    // nor a differentiated force sensor.
    let contact_fz_plan_slew_bw = env_f64("CONTACT_FZ_PLAN_SLEW_BW", 0.0).max(0.0);
    let contact_fz_plan_weight = env_f64("CONTACT_FZ_PLAN_WEIGHT", 0.0).max(0.0);
    // Opt-in soft load target for either sole during double support. This
    // changes the CoM-level objective; it is not a hard contact-force bound.
    let ds_fz_floor_weight = env_f64("DS_FZ_FLOOR_WEIGHT", 0.0).max(0.0);
    let ds_fz_floor_bw = env_f64("DS_FZ_FLOOR_BW", 0.12).max(0.0);
    let stair_swing_windows: [f64; 4] = {
        let s = std::env::var("STAIR_SWING_WINDOWS").unwrap_or_default();
        let v: Vec<f64> = s.split(',').filter_map(|x| x.trim().parse().ok()).collect();
        if v.len() == 4 {
            [v[0], v[1], v[2], v[3]]
        } else {
            [0.30, 0.25, 0.70, 0.60]
        }
    };
    let mut dcm_plan = dcm_plan;
    let mut dcm_plan_lat_scale = zmp_lat_scale;
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
    let mut cp_applied = na::Vector2::zeros();
    let cp_shift_next = flag("CP_SHIFT_NEXT", false);
    let cp_tread_box = flag("CP_TREAD_BOX", false);
    let mut cp_slice: Option<usize> = None;
    let mut cp_retimed_slice: Option<usize> = None;
    let mut n_cp_shift = 0u32;
    let mut n_cp_retime = 0u32;
    let mut n_mpc_pcmd = 0u64;
    let mut cp_retime_total = 0.0_f64;
    // Rebuild the plan continuously rather than only at touchdown.
    let adapt_live = flag("ADAPT_LIVE", true);
    // ---- step TIMING adaptation ----------------------------------------
    //
    // Independent of `ADAPT_STEP`, which moves WHERE the next foot goes.
    // This moves WHEN it gets there, by inverting the single-support DCM
    // closed form for the remaining time (see `dcm::step_timing`). Default
    // OFF: `t_ss` has been a constant for every result in this doc, and a
    // schedule that breathes changes the swing trajectory's duration, the
    // ZMP plan and the per-step accounting all at once.
    let adapt_time = flag("ADAPT_TIME", false);
    // Which axis the (over-determined) solve honours. 1 = lateral, because
    // that is the axis this machine falls on -- +-19 mm of CoP box against
    // +-49 mm fore-aft, and every failure traced in doc Sec.19 was the
    // stance CoP pinned to the LATERAL edge.
    let adapt_time_axis = env_f64("ADAPT_TIME_AXIS", 1.0) as usize;
    // How far the solve may move the end of the current single support, as a
    // fraction of the nominal `t_ss`. The swing trajectory is re-evaluated
    // against the new duration, so a large cut asks the leg for a speed it
    // does not have; a large extension leaves the machine on one foot longer
    // than the plan ever tested.
    let adapt_time_min = env_f64("ADAPT_TIME_MIN", 0.6);
    let adapt_time_max = env_f64("ADAPT_TIME_MAX", 1.4);
    // Deadband: below this the retime is not worth the plan rebuild it costs.
    let adapt_time_dead = env_f64("ADAPT_TIME_DEAD", 0.005);
    // Trigger: only solve when the DCM is actually off its reference by this
    // much. Without it the solve fires on EVERY tick of an undisturbed walk --
    // 54-60 ticks and ~500 ms of accumulated retiming with no disturbance at
    // all -- because the measured DCM never matches `xi_eos` exactly, so the
    // solver always has a few milliseconds to ask for. That is not disturbance
    // rejection, it is a slow rewrite of the gait, and it cost 8 of the 42
    // cases in the command benchmark (34/42 -> 26/42) before this existed.
    // The nominal gait has to be preserved bit-for-bit when nothing is wrong.
    let adapt_time_trig = env_f64("ADAPT_TIME_TRIG", 0.015);
    // Reduced-step reflex. `StepTiming::Unreachable` says the DCM is outboard
    // of the stance ZMP and no waiting brings it back -- measured on 75 of 83
    // ticks in a run that falls, against 0 in runs that survive. Standing on a
    // foot whose CoP is pinned to the sole edge is strictly dominated by
    // standing on two, so end the step as early as the swing trajectory allows
    // and let the wider polygon take over. Default OFF so every number already
    // measured with ADAPT_TIME reproduces bit-for-bit.
    let adapt_time_cut = flag("ADAPT_TIME_CUT", false);
    let mut n_time_cut = 0u32;
    // Reactive widen. `ADAPT_TIME_CUT` alone (same `Unreachable` trigger)
    // measured as no effect (doc Sec.21.5) -- it only moves WHEN the step
    // ends, not WHERE the next one lands, and the resulting support polygon
    // is the same nominal width either way. This is the untried half: bias
    // the swing foot's landing OUTWARD (away from the centreline) by a fixed
    // amount when `Unreachable` fires, growing the polygon instead of just
    // reaching it sooner. Independent flag from `ADAPT_TIME_CUT` so cut-only,
    // wide-only and cut+wide are separately measurable. ALWAYS outward,
    // unlike `ADAPT_STEP` (Sec.19.6, harmful over 336 points): that chases
    // the ZMP clamp deficit in whichever direction it points and reacts to
    // small, benign saturation; this fires only on the much cleaner
    // `Unreachable` signal (Sec.21.2: 75/83 ticks in a falling run, ~0 in
    // survivors) and only ever grows the polygon. Fires at most once per
    // step (`last_wide_slice` guard) -- doc Sec.25.4.
    let adapt_time_wide = flag("ADAPT_TIME_WIDE", false);
    let adapt_time_widen = env_f64("ADAPT_TIME_WIDEN", 0.02);
    let mut n_time_wide = 0u32;
    let mut last_wide_slice: Option<usize> = None;
    // ---- variable double support ---------------------------------------
    //
    // `t_ds` has been a constant since the gait was written, so the tick at
    // which the plan commits the whole machine to one foot is chosen before
    // the disturbance happens. Measured at that instant, on the lateral cell
    // with no adaptation of any kind, the DCM's offset from the foot being
    // committed to separates the outcomes completely over 8 impulses:
    //
    //   survived  +7.7  +7.8  +3.3  +8.9 mm
    //   fell      -4.3 -11.9         (committed LATE, the DCM is already
    //                                 outboard of the new stance foot)
    //   fell     +23.2 +23.9         (committed EARLY, the DCM has not
    //                                 travelled far enough yet)
    //
    // against a periodic-walk nominal of b_y = l_p / (1 + exp(omega t_ss)) =
    // 12.3 mm. So end double support when the DCM ARRIVES, not when the clock
    // says. This is the mirror of the reduced-step reflex (which shortens
    // single support AFTER commitment and measured as no effect, Sec.21.5) --
    // the damage is done at the commitment itself.
    let adapt_ds = flag("ADAPT_DS", false);
    // Same discipline as `ADAPT_TIME_TRIG`: without a gate this fires on an
    // undisturbed walk too and quietly rewrites the nominal gait, which cost
    // 8 of the 42 command-benchmark cases when step timing did it.
    let adapt_ds_trig = env_f64("ADAPT_DS_TRIG", 0.015);
    // 0.5 was the first guess and it bound: a run whose DCM was already
    // 11.9 mm OUTBOARD at commitment could only be pulled back to +0.8 mm
    // because double support could not be cut below 0.10 s.
    let adapt_ds_min = env_f64("ADAPT_DS_MIN", 0.25);
    let adapt_ds_max = env_f64("ADAPT_DS_MAX", 2.0);
    // ---- corrective impulse at the commitment instant -------------------
    //
    // A causal probe, not another correlate. Six quantities have now been
    // found that separate survivors from fallers; the three that could be
    // moved changed nothing (Sec.22.5). Separation cannot tell a cause you can
    // act on from a symptom, because a trajectory heading for a fall departs
    // from a healthy one in every quantity at once.
    //
    // So intervene instead: at the instant the plan commits to one foot, add
    // an impulse that changes the CoM velocity by a chosen amount along a
    // chosen axis, and sweep it. If some value saves the run, that axis
    // carries the fatal information and the control problem is to produce that
    // velocity change. If no value on either axis saves it, the state at
    // commitment is not where the cause lives, and the search moves on
    // without another round of correlate-chasing.
    let fix_dv = env_f64("FIX_DV", 0.0);
    let fix_deg = env_f64("FIX_DEG", 90.0);
    let fix_dt = env_f64("FIX_DT", 0.02);
    // Angular version of the same probe. A linear impulse cannot touch the
    // attitude/angular-rate part of the state, and sweeping +-0.15 m/s of CoM
    // velocity -- five times the disturbance that caused the fall -- saved 1
    // of 4 runs. So the next place to look is the part a linear impulse
    // cannot reach. `FIX_TAU` is an ANGULAR impulse [N*m*s] about the world
    // axis named by `FIX_TAU_AXIS` (0 = roll, 1 = pitch, 2 = yaw).
    let fix_tau = env_f64("FIX_TAU", 0.0);
    let fix_tau_axis = env_f64("FIX_TAU_AXIS", 0.0) as usize;
    // WHEN to apply the corrective impulse, as seconds after the push.
    // Negative (the default) means "at the commitment instant", which is where
    // the first two probes fired -- and neither +-0.15 m/s of CoM velocity nor
    // +-1.5 N*m*s of angular impulse saved 3 of the 4 runs there. By then it
    // is too late. Sweeping this backwards towards the push finds the boundary
    // of where it is still early enough, which is a control BANDWIDTH
    // requirement rather than another correlate.
    let fix_at_s = env_f64("FIX_AT_S", -1.0);
    let mut fix_fired = false;
    let mut n_ds_retime = 0u32;
    let mut ds_retime_max: f64 = 0.0;
    // Why the timing solve declined, per tick. A reviewer's claim worth
    // checking: `step_timing` returns None when the DCM is on the far side of
    // the ZMP, which may be exactly the lateral-outboard case that falls -- in
    // which case the measured +27 survivals came from fixing runs that were
    // merely late, not from saving runs that were going over.
    let mut n_time_solved = 0u32;
    let mut n_time_wrongside = 0u32;
    let mut n_time_clamped = 0u32;
    let mut n_retime_dcm = 0u32;
    let mut retime_dcm_total = 0.0f64;
    let mut retime_dcm_max: f64 = 0.0;
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
    let com_replan = flag("COM_REPLAN", false);
    let com_replan_lateral = flag("COM_REPLAN_LATERAL", false);
    let com_replan_forward_gain = env_f64("COM_REPLAN_FORWARD_GAIN", 1.0);
    assert!(com_replan_forward_gain.is_finite() && (0.0..=1.0).contains(&com_replan_forward_gain));
    let com_replan_lateral_gain = env_f64("COM_REPLAN_LATERAL_GAIN", 1.0);
    assert!(com_replan_lateral_gain.is_finite() && (0.0..=1.0).contains(&com_replan_lateral_gain));
    let com_replan_always = flag("COM_REPLAN_ALWAYS", false);
    let com_replan_dt = env_f64("COM_REPLAN_DT", 0.02).max(dt);
    // Preserve the requested mean update rate on a discrete control clock.
    // Opt in to avoid changing existing stair timing experiments.
    let com_preview_phase_lock = flag("COM_PREVIEW_PHASE_LOCK", false);
    let com_preview_dt = env_f64("COM_PREVIEW_DT", 0.05).max(dt);
    let com_preview_n = env_f64("COM_PREVIEW_N", 20.0).max(2.0) as usize;
    let com_preview_smooth = env_f64("COM_PREVIEW_SMOOTH", 0.1).max(0.0);
    let com_height_replan = flag("COM_HEIGHT_REPLAN", true);
    let com_height_time = env_f64("COM_HEIGHT_TIME", 0.60).max(0.10);
    // Force on the swing sole that counts toward confirming a landing.
    // Historically 5% of body weight (17 N on G1). Measured 2026-09-24 on the
    // stair swing: the foot rests on the tread for 1.4-2.8 s carrying a
    // MEDIAN of 10 N, because the swing task still holds it 4 mm above where
    // it physically sits (swing_pos_err_z = +4 mm) -- 10 N never reaches the
    // 17 N gate, and only impact chatter (spikes to 464 N) does. Default
    // reproduces the constant.
    let land_confirm_fz_frac = env_f64("LAND_CONFIRM_FZ_FRAC", 0.05).max(0.0);
    let mut landing_timing =
        articara::biped::preview::LandingTiming::new(land_confirm_fz_frac * total_mass * G);
    // P2-2: confirm a landing on real load (see LandingTiming::load_confirm).
    // Default OFF. When it confirms, the plan is moved onto the actual foot
    // for THIS landing only (`shift_next`), so the xy error is absorbed by
    // the plan instead of being waited out.
    let land_by_load = flag("LAND_BY_LOAD", false);
    // Admit a CONFIRMED landing to the contact set on the schedule instead
    // of waiting for the 10% BW load hysteresis (`ContactCorrection::admit`).
    // The landing gate already certified the sole on its tread (xy/z within
    // tolerance, fz above the gate); what it cannot certify is LOAD, because
    // load needs the QP to model the contact first. Default off = shipped.
    let contact_admit_confirmed = flag("CONTACT_ADMIT_CONFIRMED", false);
    let contact_admit_scheduled = flag("CONTACT_ADMIT_SCHEDULED", false);
    let contact_admit_scheduled_off_stairs = flag("CONTACT_ADMIT_SCHEDULED_OFF_STAIRS", false);
    let mut last_scheduled_admit: [Option<usize>; 2] = [None; 2];
    let com_height_ds_frac: Option<f64> = std::env::var("COM_HEIGHT_DS_FRAC")
        .ok()
        .and_then(|v| v.parse::<f64>().ok())
        .map(|f| f.clamp(0.0, 1.0));
    // A landing that ends by the extension TIMEOUT counts too when the sole
    // is within this of its planned tread height: measured 2026-09-24, the
    // champion's stair landings mostly end that way (retime median 2.16 s =
    // 1.0 s swing + 1.08 s extension) with the sole on the tread all along.
    let contact_admit_z = env_f64("CONTACT_ADMIT_Z", 0.010).max(0.0);
    let gait_dump_t: Option<(f64, f64)> = std::env::var("GAIT_DUMP_T").ok().and_then(|v| {
        let mut it = v.split(',').map(|x| x.trim().parse::<f64>().ok());
        Some((it.next()??, it.next()??))
    });
    let contact_dump_t: Option<(f64, f64)> = std::env::var("CONTACT_DUMP_T").ok().and_then(|v| {
        let mut it = v.split(',').map(|x| x.trim().parse::<f64>().ok());
        Some((it.next()??, it.next()??))
    });
    // Run the DS_FEASIBLE re-plan on a timeout end as well (it is keyed on a
    // non-zero retime, which a timeout never produces). Default off.
    let ds_feasible_on_timeout = flag("DS_FEASIBLE_ON_TIMEOUT", false);
    // Start the double support's DCM reference AT THE MEASURED DCM. The
    // plan's DS segment begins at the DCM the single support was supposed
    // to build up; a landing that ends early (confirmed while the plan's
    // extended swing still had 0.3-0.8 s to run) leaves the robot 76-108 mm
    // behind that, and the DCM controller then drives the CoM forward to
    // catch up (measured 2026-09-24: 0.26 m/s in 0.8 s, then a fall on the
    // new foot). Solve the constant DS ZMP p that makes xi(t0) = xi_meas:
    //     xi_meas = p + (xi_eos - p) e^{-wT}  ->  p = (xi_meas - xi_eos e^{-wT}) / (1 - e^{-wT})
    // clamped to the DS box; when the clamp binds, T is extended by the
    // closed-form step timing and p re-solved. Written into the plan on
    // every landing end (not only when a retime was needed). Default off.
    let ds_feasible_continuous = flag("DS_FEASIBLE_CONTINUOUS", false);
    // Ramp variant: the double-support ZMP ramps from a solved p0 to the
    // next stance's own entry point p1, so the new foot takes load INSIDE
    // the double support. With the constant solve above the ZMP sits behind
    // the leading foot for the whole DS (measured 2026-09-24: the admitted
    // foot carried 0-6 N until single support). For p(t) = p0 + v t:
    //     xi(0) = p0 A + p1 B + xi_eos e^{-wT},
    //     A = 1 - (1 - e^{-wT})/(wT),  B = (1 - e^{-wT})/(wT) - e^{-wT}
    // solved for p0 and clamped to the DS box. Default off.
    let ds_feasible_ramp = flag("DS_FEASIBLE_RAMP", false);
    // While a stair landing WAITS on one leg, hold the DCM reference at the
    // stance sole instead of pre-loading it toward the next foot. The plan's
    // backward recursion makes the swing's end DCM whatever the following
    // double support needs to reach the next stance -- 50-100 mm ahead of
    // the stance sole -- so a 2.1 s extended stance ends with the CoP on the
    // toe and the ankle pitch at its 35 N.m limit (measured 2026-09-24 on
    // the champion at 80 mm: xi tracked xi_ref to within 5 mm all the way
    // to the toe, then the swing leg was thrown up as the QP's hip strategy
    // and the run fell at 27.4 s). With the hold, the double support's ZMP
    // is solved so the swing's end DCM equals the stance's rolled end point
    // (xi_dot = 0 there), clamped to the planned two-foot box; the carry to
    // the next foot then happens inside the double support, where two feet
    // can pay for it. Default off = shipped plan.
    let land_wait_hold = flag("LAND_WAIT_HOLD", false);
    // Lower the CoM height target while the feet are in the stair zone.
    // Measured 2026-09-24 on G1: the second step onto an 80 mm riser puts
    // the swing sole 0.20 m ahead of the stance sole with the CoM 0.684 m
    // above the tread -- a 0.71 m reach, the leg's full length. The knee
    // arrives straight (0.01-0.06 rad), the swing task can no longer push
    // the sole down (residual 20 -> 600), and the QP throws the leg up.
    // The 0.233 m landing stride (STAIR_LANDING_FRAC) has the same reach.
    // Zone: mid-feet x from `start - lead` to `landing_start + tail`. The
    // target moves through the usual `COM_HEIGHT_TIME` retarget, so it is
    // smooth. Default 0 = shipped height.
    // (Distinct from the older global `STAIR_CROUCH`, which lowers the CoM
    // from t=0 at 50 mm/s and rescales the LIPM height; measured 2026-09-24
    // that one fells the flat approach at t=5 on G1.)
    // Cap the CoM height by the leg reach (see the use site). 1 = on.
    // The floor stops a pathological plan from folding the robot up.
    let reach_height = env_f64("REACH_HEIGHT", 0.0);
    let reach_height_floor = env_f64("REACH_HEIGHT_FLOOR", 0.45).max(0.1);
    // Aim the CoM height at the LOWER level a descent is about to step
    // onto, before the swing. Measured 2026-10-03 on G1 (v6 + descent
    // extension, 80 mm step-down, 45 fallers of 48): the CoM reference
    // follows the STANCE soles, so it stays at platform height through the
    // whole step-down swing; the swing leg reaches 99.7-99.9% of its length
    // and stops 15-20 mm above the lower surface; the schedule then admits
    // the airborne foot, whose orientation is uncontrolled for ~80 ms while
    // it is loaded, and the corner it happens to land on decides the run
    // (heel-outer corner -> survives, toe-outer corner -> slides 12 cm
    // forward on the toe and falls). Fraction of the drop to the coming
    // landing that the height reference takes in advance: 1 = the full
    // riser, through the usual COM_HEIGHT_TIME retarget. Only a landing
    // BELOW the stance soles counts, so ascents and level walking are
    // bit-identical. Default 0 = off = shipped.
    let com_height_desc_lead = env_f64("COM_HEIGHT_DESC_LEAD", 0.0).clamp(0.0, 1.0);
    // Geometric companion of the lead: cap the CoM height so that the LOWER
    // landing a descent swing is heading for is within this fraction of the
    // leg length at touchdown, with the CoM assumed midway between the
    // stance sole and the landing. Measured 2026-10-03 on the up-and-over
    // course (8 cm, 2/48): the stand line before the edge makes the
    // step-down 3 cm longer than on the bare descent bench, the fixed half-
    // riser lead leaves the leg at 0.99 at touchdown, the foot arrives
    // unloaded and is dragged 16 cm. Only a landing below the stance soles
    // counts, so ascents and level walking are bit-identical. 0 = off.
    let com_height_desc_reach = env_f64("COM_HEIGHT_DESC_REACH", 0.0).clamp(0.0, 1.0);
    let stair_zone_crouch = env_f64("STAIR_ZONE_CROUCH", 0.0).clamp(0.0, 0.15);
    let stair_crouch_lead = env_f64("STAIR_ZONE_CROUCH_LEAD", 0.30).max(0.0);
    let stair_crouch_tail = env_f64("STAIR_ZONE_CROUCH_TAIL", 0.60).max(0.0);
    // How far past the landing start the STRICT cascade stays in charge
    // under WBC_WEIGHTED_OFF_STAIRS. Defaults to the crouch tail (the
    // behaviour measured first).
    let weighted_zone_tail = env_f64("WBC_WEIGHTED_ZONE_TAIL", stair_crouch_tail).max(0.0);
    let mut n_wait_hold = 0u32;
    // Per side: the DOUBLE-SUPPORT slice that follows a certified landing.
    // The foot is a planned contact for that slice and is re-admitted
    // whenever the load hysteresis drops it there; cleared afterwards.
    let mut confirmed_admit: [Option<usize>; 2] = [None; 2];
    let mut n_confirmed_admissions = 0u32;
    landing_timing.load_confirm = land_by_load;
    // Event-driven contact (doc/from_claude/20261004_event_driven_contact_design.md):
    // touchdown = measured load with the sole at its planned height (no xy
    // gate), the plan is moved onto the foot, a descending swing ends at the
    // event, a rising one keeps COM_LAND_EXTEND_RISE as an explicit dwell,
    // and a swing that has not touched by its deadline probes down instead
    // of timing out. Default off = bit-identical.
    // REF_LIBRARY: a push-recovery reference (scripts/trajopt). Engaged at
    // the push (REF_TRIG_AT_PUSH=1, bench use) for its whole duration: the
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
        .map(|p| p.split(',').filter(|x| !x.is_empty()).map(RefLib::load).collect())
        .unwrap_or_default();
    let mut ref_lib: Option<RefLib> = None;
    let ref_any = !ref_libs.is_empty();
    let ref_trig_at_push = flag("REF_TRIG_AT_PUSH", false);
    let ref_swing = flag("REF_SWING", true);
    let ref_reset_stance = flag("REF_RESET_STANCE", true);
    // The re-centring after a reference skipped the swing already under way
    // when the reference ended (the gait's first step then went to the
    // pre-push plan: 0.38 m ahead, 0.28 m wide). With this on, that swing is
    // re-centred too, landing REF_RESET_LEAD [m] ahead of the feet's midpoint.
    let ref_reset_incl_current = flag("REF_RESET_INCL_CURRENT", false);
    let ref_reset_lead = env_f64("REF_RESET_LEAD", 0.0);
    // Detector: lateral DCM error toward the stance side [m] that engages a
    // reference (0 = off; the bench can trigger at the push instead).
    let ref_trig_dcm = env_f64("REF_TRIG_DCM", 0.0).max(0.0);
    // Forward-push detector: DCM error ahead of the plan [m] (0 = off);
    // engages a reference solved for a forward push (`impulse_deg` 0).
    let ref_trig_dcm_x = env_f64("REF_TRIG_DCM_X", 0.0).max(0.0);
    // Bench trigger: only pushes toward the stance side of at least this
    // impulse engage (pushes toward the swing side the gait absorbs itself:
    // measured 24/24 at 3-6 N s, and a crossover there is the wrong move).
    let ref_trig_min_imp = env_f64("REF_TRIG_MIN_IMPULSE", 4.5);
    let ref_cooldown = env_f64("REF_COOLDOWN", 1.0).max(0.0);
    // 0 = enter the reference at its start even when the detector fired late
    // (for references solved with the detection delay in their initial state).
    let ref_delay_comp = flag("REF_DELAY_COMP", true);
    // 1 = while a reference is engaged the MPC's DCM and net-CoP reference
    // schedules come from the reference (its CoM and contact forces) instead
    // of the re-planned LIPM.
    let ref_mpc = flag("REF_MPC", false);
    let ref_wbc = flag("REF_WBC", false);
    // For a LANDED (re-step) reference the WBC's DCM law follows the
    // reference's DCM/CoP (as REF_WBC does for every reference). Measured
    // 2026-10-13 on ld7_a: without it the walker's CoM x lagged the
    // reference by 6 cm at the second landing and 12 cm at the end and the
    // robot fell backward; with it the CoM tracks within 3 cm.
    let ref_wbc_landed = flag("REF_WBC_LANDED", true);
    let mut ref_cooldown_until = f64::NEG_INFINITY;
    let mut ref_engage_pending: Option<f64> = None;
    // The pending engagement wants a landed-state (re-step) reference.
    let mut ref_want_landed = false;
    let mut ref_lat_quiet_t = f64::NEG_INFINITY;
    let ref_trig_pred = env_f64("REF_TRIG_PRED", 0.0).max(0.0);
    let mut ref_lat_hist: std::collections::VecDeque<(f64, f64)> = std::collections::VecDeque::new();
    // No reference when the push lands this late in the swing (the walker
    // copes on its own there; measured in-place 5 N s at phase 0.7: 3/3).
    let ref_trig_max_frac = env_f64("REF_TRIG_MAX_FRAC", 1.0);
    let ref_post_hold = env_f64("REF_POST_HOLD", 0.0).max(0.0);
    let mut ref_mode: Option<RefMode> = None;
    let mut ref_engagements = 0u32;
    let event_mode_cfg = flag("CONTACT_EVENT", false);
    landing_timing.event_mode = flag("CONTACT_EVENT", false);
    landing_timing.event_z_tol = env_f64("CONTACT_EVENT_Z", 0.003).max(0.0);
    landing_timing.probe_speed = env_f64("CONTACT_PROBE_VZ", 0.05).max(0.0);
    landing_timing.probe_max = env_f64("CONTACT_PROBE_MAX", 0.03).max(0.0);
    let contact_event_shift = flag("CONTACT_EVENT_SHIFT", true);
    // 0 = a descending swing keeps its extension as a dwell too (double
    // support starts at the deadline, as the historical timeout did).
    let contact_event_end_on_touch = flag("CONTACT_EVENT_END_ON_TOUCH", true);
    landing_timing.load_confirm_threshold = env_f64("LAND_BY_LOAD_FRAC", 0.20) * total_mass * G;
    landing_timing.load_confirm_ticks = env_f64("LAND_BY_LOAD_TICKS", 3.0).max(1.0) as u32;
    let mut n_land_by_load_shift = 0u32;
    // P2-3': after a landing confirmation cuts the single support at `t`, the
    // double support that follows is the nominal 0.38 s no matter where the
    // DCM is. Measured 2026-09-23/24 (position gate, load gate, transfer,
    // reference blend -- every path): the rebuilt reference asks the DCM to
    // travel from over the stance foot to the next step's start inside that
    // DS, which needs a ZMP outside both soles; the command saturates and the
    // error (0.13 m at the cut) never recovers. So size the DS from the
    // measured DCM: the closed-form time for a ZMP inside the double-support
    // box to bring xi to the plan's DS-end value (`dcm::step_timing`), taken
    // per axis and clamped to [nominal, DS_FEASIBLE_MAX]. Default OFF.
    let ds_feasible = flag("DS_FEASIBLE", false);
    // DS_CAPTURE: end the double support EARLY when the DCM has left the
    // double-support box along x (no pressure point inside it can hold
    // it: 0-step capture is gone, only the next step can). Measured
    // 2026-10-11 on the late-phase forward push (11 N*s at swing 0.63):
    // the foot lands short, then the nominal 0.37 s double support runs
    // while the CoM walks past the front foot, and the catch step comes
    // 0.9 s after the push -- too late (12/12 falls). The DS end moves to
    // `t0 + DS_CAPTURE_T_MIN` (not before the landed foot carries
    // DS_CAPTURE_LOAD of the weight), once per slice; the next single
    // support is then shortened by the contact_plan and its landing by
    // the MPC as usual. DS_FEASIBLE (which only lengthens, when the DS
    // pressure CAN still catch the DCM) is complementary. Default off.
    let ds_capture = flag("DS_CAPTURE", false);
    // RESTEP: when the DCM has run past the LOADED foot's outer edge toward
    // the stance side while the just-landed foot carries no load, the
    // double support is wasted (the load never transfers) -- end it now and
    // swing the landed foot again, across to the stance side (the cross
    // step the recovery references take, for the late phases where no
    // reference fires). Design: doc/from_claude/20261012_restep_design.md.
    let restep = flag("RESTEP", false);
    let restep_margin = env_f64("RESTEP_MARGIN", 0.0);
    let restep_load_max = env_f64("RESTEP_LOAD_MAX", 0.15).clamp(0.0, 1.0);
    let restep_t_ss = env_f64("RESTEP_T_SS", 0.30).max(0.1);
    let restep_t_ds = env_f64("RESTEP_T_DS", 0.15).max(0.02);
    let restep_x = env_f64("RESTEP_X", 0.05);
    let restep_cross = env_f64("RESTEP_CROSS", 0.03);
    let restep_debug = std::env::var("RESTEP_DEBUG").is_ok();
    // Inward (toward-stance) landing bound allowed for the re-step and the
    // RESTEP_Y_IN_STEPS single supports after it: the lateral DCM left over
    // after the cross can only be caught by landing further inward, which
    // the shipped ban (MPC_FOOTSTEP_MAX_Y_IN=0) forbids.
    let restep_y_in = env_f64("RESTEP_Y_IN", 0.06).max(0.0);
    // The step AFTER the re-step is shortened too (single support
    // RESTEP_T_SS2, its double support RESTEP_T_DS): measured 2026-10-12,
    // with the nominal 0.84 s swing the touchdown lies beyond the MPC's
    // 0.4 s horizon, so no landing variable exists until the last 0.4 s
    // and the lateral residual left by the cross is never caught.
    let restep_t_ss2 = env_f64("RESTEP_T_SS2", 0.35).max(0.1);
    // How many steps after the re-step get that shortening.
    let restep_follow = env_f64("RESTEP_FOLLOW", 1.0).max(0.0) as usize;
    // Drive the re-step with a landed-state reference from REF_LIBRARY
    // (engaged through the pending path at the inserted single support)
    // instead of the hand-made cross + re-centre.
    let restep_ref = flag("RESTEP_REF", true) && ref_libs.iter().any(|l| l.landed);
    let restep_y_in_steps = env_f64("RESTEP_Y_IN_STEPS", 2.0).max(0.0) as usize;
    let mut restep_last: Option<usize> = None;
    let mut n_restep = 0u32;
    // "Hopeless": the time the DS pressure needs along x exceeds this
    // multiple of the remaining double support.
    let ds_capture_hopeless = env_f64("DS_CAPTURE_HOPELESS", 1.5).max(1.0);
    // Only shorten when it saves at least this much [s].
    let ds_capture_min_gain = env_f64("DS_CAPTURE_MIN_GAIN", 0.05).max(0.0);
    // Fire only when the DCM has run AHEAD of the DS-end target (a forward
    // push). Measured 2026-10-11 (N=48, 4 conditions): without the gate the
    // forward push went 36 -> 45 but the backward one 34 -> 32 -- stepping
    // sooner backward does not help a leg that has no authority (the
    // backward failure, design note 10-10). Default on.
    let ds_capture_fwd_only = flag("DS_CAPTURE_FWD_ONLY", true);
    // The forward excursion must exceed this [m] (noise floor).
    let ds_capture_margin = env_f64("DS_CAPTURE_MARGIN", 0.01);
    let ds_capture_t_min = env_f64("DS_CAPTURE_T_MIN", 0.15).max(0.02);
    let ds_capture_load = env_f64("DS_CAPTURE_LOAD", 0.4).clamp(0.0, 1.0);
    let ds_capture_debug = std::env::var("DS_CAPTURE_DEBUG").is_ok();
    let mut dsc_retimed_last: Option<usize> = None;
    let mut n_ds_capture = 0u32;
    let mut ds_capture_total = 0.0_f64;
    let ds_feasible_max = env_f64("DS_FEASIBLE_MAX", 1.5);
    // Also write the chosen pivot into the plan (see DcmPlan::set_segment_zmp).
    let ds_feasible_plan = flag("DS_FEASIBLE_PLAN", false);
    // Pivot box for the DS sizing: "both" = the double-support box (the
    // historical choice), "stance" = the OLD stance sole only. At the cut the
    // new foot carries ~25% of the weight (measured), so a pivot under it is
    // not realisable until the load has moved; the stance-only box is what
    // the pressure can reach NOW. With DS_FEASIBLE_PLAN the plan's DS ZMP
    // then ramps from the stance-box pivot to the full-box pivot.
    let ds_feasible_box_stance = std::env::var("DS_FEASIBLE_BOX").map(|v| v == "stance").unwrap_or(false);
    // Load share through double support as a time function for the wrench
    // MPC (walking branch): from the MEASURED share at the tick to the share
    // the next single support implies, smoothstep over the DS. Default off =
    // the 0.5/0.5 constant.
    let load_share_ramp = flag("LOAD_SHARE_RAMP", false);
    let mut n_ds_feasible = 0u32;
    let mut n_ds_continuous = 0u32;
    let mut ds_feasible_total = 0.0_f64;
    let land_frac_g = phase_gain(
        "COM_LAND_PREDICT_FRAC",
        env_f64("COM_LAND_PREDICT_FRAC", 0.0),
    );
    landing_timing.min_fraction = land_frac_g.base.clamp(0.0, 1.0);
    let land_extend_base = env_f64("COM_LAND_EXTEND", 0.25).max(0.0);
    // Extra landing-deadline slack for the swing that CHANGES HEIGHT, i.e. the
    // step onto a tread. Zero (the default) means "same as everywhere else".
    //
    // Measured 2026-09-22 on G1: the step onto a 20 mm tread needs roughly a
    // second of slack before its load crosses the confirm threshold -- the foot
    // rests on the tread at 4-6 N for ~500 ms first -- but applying that slack
    // to EVERY swing stalls the gait clock afterwards. With
    // `COM_LAND_EXTEND=1.08` the walk reached the tread 15/16 and then died:
    // 2 landing retimes totalling 2.16 s, the stair swing taking 2.035 s
    // against a planned 0.681, and the swings after it collapsing to
    // 1.075 / 0.500 / 0.260 / 0.035 s. With the default 0.25 it never reaches
    // the tread at all (0/16). The two ends of that trade want different
    // numbers, so they get different numbers.
    //
    // `RunPhase` cannot express this: it keys off `stair_transfer`, which only
    // exists once a transfer has ARMED, and the transfer is exactly what fails
    // to arm here.
    let land_extend_rise = env_f64("COM_LAND_EXTEND_RISE", 0.0).max(0.0);
    // Landing-deadline slack for a swing that steps DOWN. Default = the rise
    // value (bit-identical). Measured 2026-10-03 on a 60 mm descent: the
    // 1.08 s rise slack made the stepping-down foot hover on the lower
    // surface unloaded while the body walked on (2 timeouts, 2.16 s net),
    // and the next swing started with 22 N on the new stance -- a fall at
    // 3/48. With 0 the same member walked out.
    let land_extend_desc = env_f64("COM_LAND_EXTEND_DESC", land_extend_rise).max(0.0);
    // xy tolerance of the landing confirmation for DESCENDING swings only.
    // Measured 2026-10-03 on the up-and-over course (8 cm, 2/48): the long
    // step-down swing overshoots its goal by ~23 mm in x, the 12 mm gate
    // never confirms a foot that is flat on the ground, the swing times out
    // and the MPC then moves the landing 15 cm while the foot is down and
    // drags it. Unset = LAND_CONFIRM_XY (bit-identical).
    let land_confirm_xy_desc_cp = flag("LAND_CONFIRM_XY_DESC_CP", false);
    // Keep the descent extension on swings the contact planner retimed.
    // Measured 2026-10-04 (course, swing feed-forward on): the step-down foot
    // is 3 mm above the ground at the retimed deadline, the extension is 0
    // there, the swing times out and the MPC moves the landing 4 cm under a
    // foot that touches down 15 ms later. Default off = bit-identical.
    // (A flag in the first version, which kept the whole 0.25 s; measured:
    // 3 steps down 142 -> 133, the foot rests unloaded for the full wait.
    // Now the extension itself, in seconds, for those swings only.)
    let land_extend_desc_cp = env_f64("COM_LAND_EXTEND_DESC_CP", 0.0).max(0.0);
    // Force gate of the confirmation for DESCENDING swings, as a fraction
    // of body weight. Measured 2026-10-04 (3 steps down, feed-forward on):
    // the foot sits flat on the tread within 5 mm of its goal carrying 12 N
    // -- under the 17 N gate, because the swing task still holds the leg --
    // and waits out the whole extension. Unset = LAND_CONFIRM_FZ_FRAC.
    let land_confirm_fz_desc: Option<f64> = std::env::var("LAND_CONFIRM_FZ_DESC_FRAC").ok().and_then(|v| v.parse::<f64>().ok()).map(|f| f.max(0.0) * total_mass * G);
    // Forward tolerance of the confirmation for RISING swings. Measured
    // 2026-10-04: the champion's ascent never confirms a landing (the swing
    // overshoots by 15-26 mm, outside the 12 mm gate) and so always waits out
    // COM_LAND_EXTEND_RISE -- the CEM tuned that 1.08 s as a fixed dwell.
    // With the swing feed-forward (TRACKING_CUBIC_FF) the landing is within
    // 6 mm, confirms at once, double support starts 1 s early and the DCM
    // error runs to 0.1 (1 step up 192 -> 114). 0 = never confirm a rising
    // swing, i.e. keep that dwell explicitly. Unset = LAND_CONFIRM_XY.
    let land_confirm_xy_rise: Option<f64> = std::env::var("LAND_CONFIRM_XY_RISE").ok().and_then(|v| v.parse().ok());
    // z tolerance that goes with the wider xy one: the 10 mm default lets
    // the confirmation fire on the HEEL STRIKE of a toe-up foot (sole centre
    // still 8 mm high), 60 ms before the sole is flat (measured: 2 steps x
    // 6 cm 46 -> 33 with the wide xy alone). Unset = LAND_CONFIRM_Z.
    landing_timing.z_tol_override = std::env::var("LAND_CONFIRM_Z_DESC").ok().and_then(|v| v.parse().ok());
    let land_confirm_xy_desc: Option<f64> = std::env::var("LAND_CONFIRM_XY_DESC").ok().and_then(|v| v.parse().ok());
    landing_timing.max_extension = land_extend_base;
    landing_timing.speed_z = env_f64("COM_LAND_VZ", 0.20).max(0.01);
    landing_timing.speed_xy = env_f64("COM_LAND_VXY", 0.35).max(0.01);
    // Confirm level landings, but only AFTER a stair transfer has finished.
    //
    // Enabling it for the whole run breaks the approach: the confirmation
    // wants 12 mm of xy and 10 mm of z accuracy plus 5% of body weight, which
    // an ordinary walking step does not reach inside its slice, so it times
    // out repeatedly and drags the schedule -- measured, the approach fell at
    // 5.66 s instead of reaching the stair. The gap it exists to close is the
    // one AFTER the ascent, where every remaining step is level and nothing
    // confirms the landing at all. See `LandingTiming::level_steps`.
    let land_confirm_level = flag("LAND_CONFIRM_LEVEL", false);
    // Contact-triggered stair transfer.  A confirmed landing opens one DS
    // phase and gives pressure, CoM height and vertical load the SAME clock.
    // It is opt-in while the fixed-arm staircase experiment is validated.
    let stair_transfer_enabled = flag("STAIR_TRANSFER", false);
    let stair_capture_time = env_f64("STAIR_CAPTURE_TIME", 0.80).max(0.10);
    let stair_capture_from_xi = flag("STAIR_CAPTURE_FROM_XI", false);
    // Capture phase tracks the MEASURED load instead of demanding pressure
    // where there is none. At arming the newly landed (high) foot carries
    // ~25% of the weight (measured 2026-09-24), yet the historical Capture
    // moves the ZMP reference to the capture point at once and asks the MPC
    // for a 50/50 split at once; every path that kept that demand (reference
    // continuity, DS sizing, pivot in the plan) left 55-59 of 62 armed runs
    // on the floor. With this on, the reference pressure point is
    //     p_ref = p_low + (capture_target - p_low) * s
    // where s is the (EMA-smoothed) share of weight on the high foot, and the
    // MPC's Capture split goes from s to 0.5 over STAIR_CAPTURE_TIME instead
    // of starting at 0.5. Default off.
    let stair_capture_load_track = flag("STAIR_CAPTURE_LOAD_TRACK", false);
    let stair_capture_share_tau = env_f64("STAIR_CAPTURE_SHARE_TAU", 0.08).max(1e-3);
    // How far the Capture reference may LEAD the measured share. 0 = pure
    // tracking (measured 2026-09-24: no drive, the load never moves, 0 of 67
    // armed runs survived); 1 = the driven ramp alone. The reference share is
    //     s_ref = min(ramp(t), s_meas + lead)
    // so it pulls the load along but never asks for more than `lead` ahead
    // of what is actually there.
    let stair_capture_lead = env_f64("STAIR_CAPTURE_LEAD", 0.0).clamp(0.0, 1.0);
    // How much of `dt` the transfer clock advances while a guard fails.
    // Historically 0: the clock STOPS, so a Capture whose guards fail on
    // 45% of ticks (measured 2026-09-22) barely progresses, and anything
    // keyed to the clock -- the driven load ramp, `STAIR_CAPTURE_LEAD` --
    // never fires (measured 2026-09-24: lead 0.15 and 0.30 bit-identical to
    // no lead). The phase TRANSITIONS keep their guard conditions; only the
    // drive keeps moving, at this fraction of speed. Default 0 = historical.
    let stair_clock_guard_slow = env_f64("STAIR_CLOCK_GUARD_SLOW", 0.0).clamp(0.0, 1.0);
    // Transfer state machine variant. "capture" = historical Capture -> Rise
    // -> Commit. "rise_first" = skip Capture: rise immediately with BOTH feet
    // down, the load share and the height driven by the phase clock, and
    // release the low foot only once the high foot carries
    // STAIR_RELEASE_SHARE of the weight. Rise -> Commit then judges the DCM
    // against the DOUBLE-support box while the low foot is held.
    //
    // Why (measured 2026-09-24): Capture -> Rise requires the measured DCM
    // to sit inside the HIGH foot's CoP box (capture_error <= 3 mm), i.e. the
    // DCM must be fully captured over a foot that carries 9% of the weight
    // at arming (median of 67 armed runs). Driving it there in a 0.4 s
    // double support saturates the ZMP; not driving it (load tracking)
    // times Capture out at 4 s. Neither can satisfy the gate. Lifting while
    // the DCM is still between the feet removes the gate instead of fighting
    // it. Default "capture" reproduces every existing recipe.
    let stair_transfer_rise_first = std::env::var("STAIR_TRANSFER_MODE").map(|v| v == "rise_first").unwrap_or(false);
    let stair_release_share = env_f64("STAIR_RELEASE_SHARE", 0.80).clamp(0.3, 1.0);
    // The release must not fire on the landing impact transient (measured
    // 2026-09-24: 98% on the landing foot 5 ms after arming, ~9% a few ticks
    // later). Smooth the share with this time constant and require the
    // transfer clock to have run at least STAIR_RELEASE_MIN_T.
    let stair_release_tau = env_f64("STAIR_RELEASE_TAU", 0.10).max(1e-3);
    let stair_release_min_t = env_f64("STAIR_RELEASE_MIN_T", 0.30).max(0.0);
    let stair_rise_time = env_f64("STAIR_RISE_TIME", 1.50).max(0.10);
    // Optional unload interval before Rise; zero preserves the baseline.
    let stair_release_time = env_f64("STAIR_CAPTURE_RELEASE_TIME", 0.0).max(0.0);
    let stair_release_guard = flag("STAIR_CAPTURE_RELEASE_GUARD", false);
    let stair_commit_time = env_f64("STAIR_COMMIT_TIME", 0.40).max(0.10);
    // Diagnostic separation of tread hold from the return to walking.
    let stair_post_commit_hold = env_f64("STAIR_POST_COMMIT_HOLD", 0.0).max(0.0);
    let stair_post_walk_settle = flag("STAIR_POST_WALK_SETTLE", false);
    let stair_post_walk_terminal_step = flag("STAIR_POST_WALK_TERMINAL_STEP", false);
    let stair_post_walk_terminal_gap = env_f64("STAIR_POST_WALK_TERMINAL_GAP", 0.065).max(0.060);
    let stair_post_walk_settle_ramp = env_f64("STAIR_POST_WALK_SETTLE_RAMP", 1.00).max(dt);
    let stair_post_walk_settle_capture_hold =
        env_f64("STAIR_POST_WALK_SETTLE_CAPTURE_HOLD", 0.50).max(0.0);
    let stair_post_walk_settle_steps =
        env_f64("STAIR_POST_WALK_SETTLE_STEPS", 2.0).max(1.0) as usize;
    let stair_post_walk_settle_load = env_f64("STAIR_POST_WALK_SETTLE_LOAD", 0.10).clamp(0.0, 0.40);
    let stair_post_walk_settle_dwell = env_f64("STAIR_POST_WALK_SETTLE_DWELL", 0.05).max(0.0);
    let stair_post_walk_settle_fz_weight =
        env_f64("STAIR_POST_WALK_SETTLE_FZ_WEIGHT", 1.0).max(0.0);
    let stair_trunk_post_time = env_f64("STAIR_TRUNK_POST_TIME", 0.0).max(0.0);
    let stair_height_handover = flag("STAIR_HEIGHT_HANDOVER", false);
    let stair_consume_rise_step = flag("STAIR_CONSUME_RISE_STEP", false);
    // Crouch this far below the Commit terminal height when the transfer
    // hands back to the gait.
    //
    // Measured on the fixed-arm one-step: the step after the ascent never
    // lands. Its target descends 24 mm and the foot follows only 10 of them,
    // swing task residual 3.9 (m/s^2 RMS) while the leg torques stay under
    // 1 N*m of their 6-14 -- so it is not torque, and the residual falls to
    // 1e-6 the moment the target stops moving. The knee is at 15 deg, about
    // 99% of full extension: the swing leg is near its kinematic singularity
    // and cannot put the foot down. Standing 52 mm taller on the landing
    // than the gait's own nominal is what puts it there. 0 = off (identical
    // to the shipped `STAIR_HEIGHT_HANDOVER` seed).
    let stair_post_z = env_f64("STAIR_POST_Z", 0.0).clamp(0.0, 0.08);
    // Post-transfer DS may release only after the next stance foot is loaded.
    let stair_resume_contact_gate = flag("STAIR_RESUME_CONTACT_GATE", false);
    // Hold the lateral DCM over the measured stance foot while a
    // post-transfer single-support phase waits for its receiver to load.
    let stair_resume_ss_dcm_hold = flag("STAIR_RESUME_SS_DCM_HOLD", false);
    let stair_resume_ss_hold_max = env_f64("STAIR_RESUME_SS_HOLD_MAX", 1.0).max(dt);
    let stair_resume_ss_hold_frac = env_f64("STAIR_RESUME_SS_HOLD_FRAC", 0.65).clamp(0.5, 1.0);
    let stair_resume_ss_sink = env_f64("STAIR_RESUME_SS_SINK", 0.002).clamp(0.0, 0.010);
    let stair_resume_ss_load = env_f64("STAIR_RESUME_SS_LOAD", 0.10).clamp(0.02, 0.80);
    let stair_resume_ss_admit_load =
        env_f64("STAIR_RESUME_SS_ADMIT_LOAD", 0.015).clamp(0.005, 0.10);
    // Bridge the SS-held DCM to the ordinary DS plan from the measured state.
    let stair_resume_ds_capture = flag("STAIR_RESUME_DS_CAPTURE", false);
    let stair_resume_ds_capture_time = env_f64("STAIR_RESUME_DS_CAPTURE_TIME", 0.50).max(dt);
    let stair_resume_ds_capture_after =
        env_f64("STAIR_RESUME_DS_CAPTURE_AFTER", 2.0).max(1.0) as u32;
    let stair_resume_gate_load = env_f64("STAIR_RESUME_GATE_LOAD", 0.65).clamp(0.05, 1.5);
    let stair_resume_gate_tilt = env_f64("STAIR_RESUME_GATE_TILT", 0.08).max(0.01);
    let stair_resume_gate_dwell = env_f64("STAIR_RESUME_GATE_DWELL", 0.05).max(dt);
    let stair_resume_sole_pose = flag("STAIR_RESUME_SOLE_POSE", false);
    let stair_resume_swing_priority = flag("STAIR_RESUME_SWING_PRIORITY", false);
    let stair_resume_swing_couple = env_f64("STAIR_RESUME_SWING_COUPLE", 0.0).max(0.0);
    let mut resume_gate_slice = None;
    let mut resume_gate_confirmed_s = 0.0_f64;
    let mut resume_gate_wait_started: Option<f64> = None;
    // source slice, receiver, frozen lateral DCM, activation time
    let mut resume_ss_dcm_hold: Option<(usize, usize, f64, f64)> = None;
    let mut resume_ss_dcm_ready_s = 0.0_f64;
    let mut resume_ss_contact_admitted: Option<(usize, usize)> = None;
    let mut resume_ss_hold_finished_slice: Option<usize> = None;
    // release time, measured lateral DCM at release
    let mut resume_ds_capture: Option<(f64, f64)> = None;
    let mut resume_ss_releases = 0_u32;
    let mut stair_trunk_post_until = 0.0_f64;
    let stair_capture_inboard = env_f64("STAIR_CAPTURE_INBOARD", 0.012).max(0.0);
    let stair_capture_margin = env_f64("STAIR_CAPTURE_MARGIN", 0.70).clamp(0.1, 1.0);
    let stair_capture_tol = env_f64("STAIR_CAPTURE_TOL", 0.003).max(0.0);
    // Capacity the LOW foot keeps at the end of Capture. Measured on the
    // fixed-arm one-step: through Capture the HIGH foot sits at 0-3 N for
    // 250 ms and then takes everything in 40 ms -- total support falls to
    // 0.65 mg (briefly falling), a 146 N slam follows, and it launches.
    // Nothing asks the high foot to take load: `f_share` returns 1.0 for
    // both feet during a transfer, and `TouchdownLoadRamp` serves the LOW
    // foot's re-contact in Rise, not the high foot's initial loading here.
    // Squeezing the low foot's CAPACITY is the existing handle (the CoP box
    // is |m| <= L*fz). 1.0 = off (shipped behaviour).
    let stair_capture_low_cap = env_f64("STAIR_CAPTURE_LOW_CAP", 1.0).clamp(0.05, 1.0);
    // Foot force difference control (Caron, Kheddar, Tempier, ICRA 2019 --
    // HRP-4 climbs 18.5 cm x 5 steps with it). Squeezing the low foot's
    // capacity above, and the nominal-force reference below, are both OPEN
    // LOOP: nothing checks whether the high foot actually took the load.
    // Measured on the fixed-arm one-step, it does not -- the reference ramps
    // for 220 ms at 0-1.6 N and then the whole handover happens in 15 ms.
    //
    // HRP-4 closes the loop on the RELATIVE FOOT HEIGHT, not on the force
    // command, because a unilateral contact answers "push down" and ignores
    // "please carry more". Their gain is A_dfz = 1e-4 m/(N*s) on a 39 kg
    // robot; scale by mass when porting. Their law needs a separate 1 Hz
    // vertical drift compensator; applying the correction as +-dz/2 keeps
    // the mean foot height fixed by construction, so it does not.
    //
    // 0 = off (shipped behaviour).
    let stair_ffdc = env_f64("STAIR_FFDC", 0.0).max(0.0);
    let stair_ffdc_clamp = env_f64("STAIR_FFDC_CLAMP", 0.010).max(0.0);
    // Rate at which the offset bleeds back to zero when FFDC is not driving,
    // so a released foot does not inherit a stale bias.
    let stair_ffdc_leak = env_f64("STAIR_FFDC_LEAK", 4.0).max(0.0);
    let stair_capture_k = env_f64("STAIR_CAPTURE_K", 1.0).max(0.1);
    let stair_transfer_latch = flag("STAIR_TRANSFER_LATCH", false);
    let stair_transfer_timeout = env_f64("STAIR_TRANSFER_TIMEOUT", 4.0)
        .max(stair_capture_time + stair_rise_time + stair_commit_time);
    let stair_transfer_dcm_tol = env_f64("STAIR_TRANSFER_DCM", 0.025).max(0.001);
    let stair_transfer_z_tol = env_f64("STAIR_TRANSFER_Z", 0.015).max(0.001);
    let stair_rise_load = env_f64("STAIR_RISE_LOAD", 0.45).clamp(0.05, 0.95);
    let stair_commit_load = env_f64("STAIR_COMMIT_LOAD", 0.80).clamp(0.05, 0.98);
    // Fraction of the walk's DCM drift carried through a transfer. See the
    // use site; 0 reproduces the historical behaviour exactly.
    let stair_transfer_drift = env_f64("STAIR_TRANSFER_DRIFT", 0.0).clamp(0.0, 2.0);
    // Finite double-support extension at transfer arm (0 = to end of run).
    let stair_transfer_hold = env_f64("STAIR_TRANSFER_HOLD", 0.0).max(0.0);
    // Keep the walking DCM plan through a transfer instead of rebuilding it.
    let stair_transfer_keep_dcm = flag("STAIR_TRANSFER_KEEP_DCM", false);
    let stair_transfer_tilt = env_f64("STAIR_TRANSFER_TILT", 0.12).max(0.01);
    let stair_transfer_fz_guard = env_f64("STAIR_TRANSFER_FZ_GUARD", 1.60).max(1.0);
    let stair_transfer_ticks = env_f64("STAIR_TRANSFER_TICKS", 4.0).max(1.0) as u32;
    // Full per-foot wrench MPC for the three-phase stair transfer.
    // Opt-in until the fixed-arm staircase benchmark has been revalidated.
    // Command the swing to the ANKLE pose the footstep plan implies, rather
    // than to the sole pose directly. See the use site for the measurement.
    let stair_td_sole_frame = flag("STAIR_TD_SOLE_FRAME", false);
    // Keep the sole flat through a height-changing swing. See the use sites.
    let stair_rise_sole_pose = flag("STAIR_RISE_SOLE_POSE", false);
    let stair_rise_sole_pose_min_dz = env_f64("STAIR_RISE_SOLE_POSE_MIN_DZ", 0.0).max(0.0);
    let stair_wrench_mpc = flag("STAIR_WRENCH_MPC", false);
    // Run the wrench MPC outside a stair transfer too.
    //
    // The block below is gated on `stair_transfer_cmd.is_some()`, so the MPC
    // solves only while a transfer is armed. Measured 2026-09-22: at 60 and
    // 80 mm risers no transfer armed in 16 of 16 runs, i.e. the best
    // high-riser figures of that day were obtained with the MPC executing
    // ZERO ticks. A predictive layer that only runs during a hand-written
    // primitive cannot carry a momentum reference or a foothold decision, so
    // this is infrastructure for both rather than a result on its own.
    //
    // Default OFF: the walking branch builds its own references (below) and
    // must be measured against the transfer-only lineage before it replaces
    // it.
    let wrench_mpc_always = flag("WRENCH_MPC_ALWAYS", false);
    // ... and let its solution reach the WBC outside a transfer. Measured
    // 2026-09-24: every consumer of `wrench_mpc_command` (accel, coupled
    // level, forces_nominal blend, fz plan) is conditioned on a transfer, so
    // ALWAYS alone solved 1500 times per run and discarded the result; a
    // load-share change in the MPC input was bit-identical at the output.
    // With this on, the coupled regulariser level and the forces_nominal
    // blend follow the MPC whenever it has a fresh solution. Default off.
    let wrench_mpc_apply = flag("WRENCH_MPC_APPLY", false) && wrench_mpc_always;
    let mpc_target_preview = flag("MPC_TARGET_PREVIEW", false);
    // Show the MPC the same double-support load ramp the WBC applies.
    let mpc_load_schedule = flag("MPC_LOAD_SCHEDULE", false);
    // Align the fz schedules with `contact_schedule` (node centre).
    let mpc_node_centre = flag("MPC_NODE_CENTRE", false);
    // Give the MPC the CoP outright instead of averaging it with the DCM
    // controller's (see the use site).
    // Weight on "the pressure sits where the DCM plan says", inside the MPC.
    // 0 = off; the MPC then has no opinion about the CoP (see wrench_mpc.rs).
    let mpc_cop_weight = env_f64("MPC_COP_W", 0.0).max(0.0);
    let mpc_net_cop_weight = env_f64("MPC_NET_COP_W", 0.0).max(0.0);
    // Give the MPC the SAME pressure reference the WBC is executing.
    //
    // The net-CoP schedule below is read off the open-loop plan,
    // `dcm_plan.reference(at).p`. The WBC does not track that: it tracks
    // `p_cmd`, the plan plus DCM feedback, clamped to the real support.
    // Measured 2026-09-25 the two differ by 5.6 mm (median, double support),
    // and measured 2026-09-26 -- once the `wbc_cop` column's frame bias was
    // fixed (Sec.43) -- the MPC's own pressure sits 9 mm ahead of `p_cmd` in
    // double support while the WBC sits within 5 mm of it. Handing the MPC
    // authority injects that 9 mm. This shifts the whole schedule by the
    // current feedback offset (p_cmd - plan(t)) so node 0 asks for what the
    // WBC is asking for; later nodes keep the plan's shape. 0 = off = the
    // shipped schedule, bit-identical (adds an exact zero).
    let mpc_cop_ref_cmd = flag("MPC_COP_REF_CMD", false);
    // Cap on that shift's length [m]. After a push p_cmd can sit 8 cm beyond
    // the front toe; fed raw into the MPC's net-CoP reference it pins the CoP
    // to the toe for the whole double support and over-brakes (2026-09-27,
    // V5A i8_d0/m01: the DCM error reverses sign and the robot falls
    // backwards in the next single support). The WBC's own use of p_cmd is
    // clamped to the support polygon; this is the MPC-side analogue.
    // inf = off = the raw shift, bit-identical.
    let mpc_cop_ref_cmd_max = env_f64("MPC_COP_REF_CMD_MAX", f64::INFINITY).max(0.0);
    // 0 = off, 1 = seed the FIRST solve from the measured load, 2 = anchor
    // EVERY solve to the measured load (the stair-transfer ramp's rule, made
    // general): the smoothness term then says "do not jump away from what the
    // robot is actually carrying" instead of "do not jump away from what the
    // MPC said last time", which is what makes it lag every load transfer.
    let mpc_seed_previous = env_f64("MPC_SEED_PREVIOUS", 0.0) as i32;
    // MPC_FOOTSTEP: the swing foot's landing becomes a decision variable of
    // the wrench MPC (`wrench_mpc::Landing`, one SQP step about the planned
    // load). The chosen offset is applied the way the contact_plan adjuster
    // applies its shift -- `shift_next` (non-accumulating) and a rebuilt DCM
    // plan -- and the two are mutually exclusive: both on would be two
    // authorities over the same landing. Off by default.
    // Step-4 body (doc/from_claude/20260926_sqp_mpc_design.md): 2a moment
    // arm about the previous solve's predicted CoM, 2b penalise L-dot.
    let mpc_arm_about_prior = flag("MPC_ARM_ABOUT_PRIOR", false);
    let mpc_w_ldot = env_f64("MPC_W_LDOT", 0.0).max(0.0);
    // Step-4 body, stage 1 (design note Sec.9): the landing recaptures the DCM.
    let mpc_landing_capture_w = env_f64("MPC_LANDING_CAPTURE_W", 0.0).max(0.0);
    // Longer horizon for the landing-capture rows only (nodes); the
    // reference schedules are built this long. 0 = the wrench horizon.
    let mpc_landing_capture_n = env_f64("MPC_LANDING_CAPTURE_N", 0.0).max(0.0) as usize;
    // Decide the landing lexicographically (capture first, exactly, inside
    // the box; then the wrench MPC with it fixed) instead of through the
    // penalty weight above, which then only has to be > 0 to switch the
    // capture inputs on. See wrench_mpc::Config::landing_lexicographic.
    let mpc_landing_lex = flag("MPC_LANDING_LEX", false);
    // DCM-capture rows on the MPC's own net CoP (every node), see
    // wrench_mpc::Config::capture_cop_weight. Needs MPC_LANDING_CAPTURE_W > 0
    // for the capture inputs. 0 = off = shipped.
    let mpc_capture_cop_w = env_f64("MPC_CAPTURE_COP_W", 0.0).max(0.0);
    // Lexicographic CoP (`wrench_mpc::Config::capture_cop_lexicographic`):
    // the net CoP is planned first as a LIPM recapture inside the box the
    // scheduled split can reach, then held by equality. Replaces the penalty
    // capture rows (MPC_CAPTURE_COP_W), which the MPC satisfied through the
    // load split (2026-10-01, design note Sec.14). Off = shipped.
    let mpc_capture_lex = flag("MPC_CAPTURE_LEX", false);
    // Step timing as a decision of the lexicographic landing
    // (`wrench_mpc::Config::landing_timing`): the touchdown may be brought
    // forward to MPC_LANDING_TMIN_FRAC of the single support, at a cost of
    // MPC_LANDING_TIME_W per second^2 of shortening. The chosen time is
    // applied through the same retime the contact_plan adjuster uses (so
    // run with CONTACT_PLAN=0 to compare). Off = shipped.
    let mpc_landing_time = flag("MPC_LANDING_TIME", false);
    // Emergency touchdown for a push TOWARD the stance foot: when the lateral
    // DCM error points at the stance side by more than this [m], the swing
    // is brought down at CP_LAT_EMERGENCY_TMIN of its remaining time, once
    // per step. Measured 2026-10-02 (design note Sec.18): 7 N.s toward the
    // stance side fell 35/35 -- the sole's +-27 mm cannot hold a 54 mm
    // capture-point shift and the swing foot was still 0.9 s from landing on
    // its own side; no landing geometry and no x-driven retime changed a
    // single run. A double support NOW is the only polygon that contains
    // the CoM. 0 = off = shipped.
    let cp_lat_emergency = env_f64("CP_LAT_EMERGENCY", 0.0).max(0.0);
    let cp_lat_emergency_tmin = env_f64("CP_LAT_EMERGENCY_TMIN", 0.3).clamp(0.0, 1.0);
    let mut lat_emg_slice: Option<usize> = None;
    let mut n_lat_emg = 0u32;
    let mpc_landing_time_w = env_f64("MPC_LANDING_TIME_W", 1e-3).max(0.0);
    let mpc_landing_tmin_frac = env_f64("MPC_LANDING_TMIN_FRAC", 0.5).clamp(0.0, 1.0);
    let mut n_mpc_retime = 0u32;
    let mut mpc_retimed_slice: Option<usize> = None;
    // The lexicographic CoP writes equality rows at every node. The default
    // active-set backend reports Infeasible for those with any positive
    // minimum load (wrench_mpc test
    // `a_sideways_net_cop_equality_is_feasible_on_clarabel_but_not_the_active_set`),
    // so this mode runs on Clarabel unless MPC_QP_BACKEND says otherwise.
    let mpc_capture_lex_wp = env_f64("MPC_CAPTURE_LEX_WP", 0.1).max(0.0);
    let mpc_capture_lex_ws = env_f64("MPC_CAPTURE_LEX_WS", 0.1).max(0.0);
    let mpc_capture_lex_margin = env_f64("MPC_CAPTURE_LEX_MARGIN", 0.005).max(0.0);
    // Fraction of the WBC's commanded pressure point taken from the MPC's
    // node-0 net CoP (see the use site). Step 3 of the BD roadmap: the WBC
    // follows the MPC. 0 = off = shipped.
    let mpc_pcmd = env_f64("MPC_PCMD", 0.0).clamp(0.0, 1.0);
    // With MPC_PCMD: BD's "plan + fast feedback". The MPC is solved every
    // 20 ms and its node-0 CoP is otherwise held between solves, while the
    // DCM law it replaces looked at xi every tick. Measured 2026-09-27
    // (design note Sec.14, N=24): that hold loses the strong pushes
    // (1e7: z=-3.13, backward 7->1). Here the solve-time DCM xi_s and the
    // MPC's CoP p give the DCM the MPC expects now,
    //     xi_pred(t) = p + (xi_s - p) exp(omega (t - t_s)),
    // and the commanded pressure point corrects only the departure from it,
    //     p_cmd = p + (1 + k_dcm) (xi_meas - xi_pred),
    // which makes that departure decay as exp(-k_dcm omega t) (same gain
    // as the DCM law, dcm::commanded_zmp). Off = the held CoP = shipped.
    let mpc_pcmd_fb = flag("MPC_PCMD_FB", false);
    // Sensor-delay compensation of the DCM (see the use site). Seconds.
    let dcm_lead_s = env_f64("DCM_LEAD_S", 0.0).max(0.0);
    let mut prev_p_cmd: Option<na::Vector2<f64>> = None;
    // With MPC_PCMD: take the MPC's node-0 horizontal CoM ACCELERATION and
    // hand the WBC the LIPM pressure point that produces it,
    //     p = c - a_xy / omega^2,
    // instead of the moment-based net CoP of the MPC's wrench. The DCM obeys
    // the horizontal force, not the pressure point: with free horizontal
    // forces and angular momentum the SRBD's CoP and its CoM acceleration
    // are different things, and the CoP-capture rows were buying a heel-ward
    // CoP with front-foot load while the acceleration stayed short
    // (2026-10-01, design note Sec.14). Off = the net CoP = shipped.
    let mpc_pcmd_from_acc = flag("MPC_PCMD_FROM_ACC", false);
    // Fix the MPC's double-support vertical split to the schedule
    // (`wrench_mpc::Config::fz_split_equality`). Off = shipped.
    let mpc_fz_split_eq = flag("MPC_FZ_SPLIT_EQ", false);
    // 1 = shipped: the landing offset also enters the momentum and net-CoP
    // rows as `dp x F`. 0 = capture rows and landing cost only (2026-09-27:
    // the moment coupling made the landing jitter +-30-47 mm on
    // undisturbed steps, see the design note).
    let mpc_footstep_moment = flag("MPC_FOOTSTEP_MOMENT", true);
    let mpc_footstep = flag("MPC_FOOTSTEP", false);
    // The one combination allowed: the contact_plan adjuster with ALL its
    // position boxes at zero is a pure step-RETIMER (its shift is exactly
    // zero, so it never touches the landing), and the MPC landing variable
    // has no timing freedom. Measured 2026-09-27 (design note, FX): the
    // landing variable matches v4.1 at 8 N.s but takes 0/24 at 11 N.s where
    // v4.1's adjuster shortens the single support (CP_T_FROM_X).
    // CP_TREAD_BOX snaps the landing after the box clamp, so it can move the
    // foot with every box at zero: not timing-only.
    let contact_plan_timing_only = contact_plan_on
        && contact_plan_cfg.max_x == 0.0
        && contact_plan_cfg.max_y == 0.0
        && contact_plan_cfg.max_y_in == 0.0
        && !flag("CP_TREAD_BOX", false);
    assert!(
        !(mpc_footstep && contact_plan_on) || contact_plan_timing_only,
        "MPC_FOOTSTEP and the contact_plan adjuster are two authorities over the same landing; turn one off (or make the adjuster timing-only: CP_MAX_X=CP_MAX_Y=CP_MAX_Y_IN=0)"
    );
    let mpc_footstep_w = env_f64("MPC_FOOTSTEP_W", 100.0).max(0.0);
    // Freeze the landing variable once the swing clock passes this fraction
    // of the single support (>1 = during a late-contact extension). Measured
    // 2026-09-27 (design note, wide holdout 8 cm m01): while the foot hovers
    // over the first tread waiting for contact, the schedule already has it
    // down (first_node = 0) and the capture rows (1e6) throw the landing to
    // its box (-54, +80 mm), pulling the foot off the nosing. The swing
    // cannot follow a late shift anyway. inf = off = shipped behaviour.
    let mpc_footstep_freeze_frac = env_f64("MPC_FOOTSTEP_FREEZE_FRAC", f64::INFINITY);
    // Give the capture rows the plan's own sensitivity to the landing
    // (`Input::reference_dcm_sensitivity`), differenced from the real DCM
    // plan the way CP_J_AUTO does: shift the landing by 2 cm per axis,
    // rebuild the rolled plan, read the reference DCM at every node. Off =
    // the shipped rows, bit-identical.
    let mpc_landing_capture_j = flag("MPC_LANDING_CAPTURE_J", false);
    // The MPC is given the plan AFTER this slice's earlier landing shifts
    // (forecast contact, net-CoP and DCM schedules), so its answer is an
    // INCREMENT. The shipped application treats it as the total since the
    // slice started (`d = dp - applied`), which retracts every shift on the
    // next solve (code review 2026-09-27). 1 = apply it as an increment, with
    // the landing cost and box on the accumulated total. 0 = shipped.
    let mpc_footstep_increment = flag("MPC_FOOTSTEP_INCREMENT", false);
    // Largest BACKWARD landing shift allowed when the landing is on a
    // different level from the lift-off (a stair step, either direction).
    // Measured 2026-09-27 (wide holdout 8 cm m33): the MPC moved the first
    // tread landing 20 mm back, the heel overhung 35 mm instead of 15, and the
    // robot fell backward in the next double support. A box, not a snap
    // (CP_TREAD_BOX showed snapping to an edge is harmful). Default = the
    // symmetric box (MPC_FOOTSTEP_MAX_X), bit-identical.
    let mpc_footstep_stair_back = env_f64("MPC_FOOTSTEP_STAIR_BACK", f64::INFINITY).max(0.0);
    // Terrain as a COST in the lexicographic landing (`Landing::terrain_x`):
    // on a staircase the landing's total x pays this weight times its
    // squared distance outside the sole window of the surface the nominal
    // landing is over (minus MPC_FOOTSTEP_TERRAIN_MARGIN). 0 = off.
    let mpc_footstep_terrain_w = env_f64("MPC_FOOTSTEP_TERRAIN_W", 0.0).max(0.0);
    // Capture FLOOR on the lexicographic landing (fraction of the sole
    // half-length, 0 = off): predict the DCM at the end of the coming double
    // support with the pressure held at the stance foot (the new foot's load
    // is not admitted before that), and require the landing to put the sole
    // under it -- behind it for a backward push, ahead for a forward one.
    // The capture rows alone match the plan's DCM, which after a strong push
    // early in swing asked for a step beside the stance foot (design note
    // Sec.18: -106 mm where -230 was needed).
    let mpc_footstep_capture_floor = env_f64("MPC_FOOTSTEP_CAPTURE_FLOOR", 0.0).max(0.0);
    let mpc_footstep_terrain_margin = env_f64("MPC_FOOTSTEP_TERRAIN_MARGIN", 0.01).max(0.0);
    // Largest lateral shift TOWARD the stance foot (a narrower step). The
    // contact_plan adjuster forbids it in v4.1 (CP_MAX_Y_IN=0); the MPC box
    // allowed it. Measured 2026-09-27 (D+J+T, 11 N.s backward, m32): the
    // landing moved 27 mm inward, the lateral DCM error then ran away
    // -19 -> -126 mm and the robot fell. Default = MPC_FOOTSTEP_MAX_Y
    // (symmetric box), bit-identical.
    let mpc_footstep_max_y_in = env_f64("MPC_FOOTSTEP_MAX_Y_IN", f64::INFINITY).max(0.0);
    // Allow an inward (narrowing) landing shift only down to this lateral
    // gap between the stance sole and the swing landing [m]. What the
    // inward ban protects against is a narrow or crossing stance, so bound
    // the gap itself rather than the shift. Measured 2026-09-27: the flat
    // nominal gap is ~0.11 m. Combined with MPC_FOOTSTEP_MAX_Y_IN (the
    // smaller inward bound wins). 0 = off (no width floor) = shipped.
    let mpc_footstep_min_width = env_f64("MPC_FOOTSTEP_MIN_WIDTH", 0.0).max(0.0);
    // Inward bound on a step the contact_plan timing adjuster has SHORTENED.
    // Measured 2026-09-27 (v5, 11 N.s forward, 11 members): with inward
    // allowed the pushed step lands 60-80 mm forward AND 20-35 mm inward and
    // the next step needs only ~7 mm; v5 (inward 0) lands the same step
    // straight ahead and the next one is thrown to the box edges. A shortened
    // single support ends before the lateral DCM swings out, so the next
    // support belongs inward. The harmful inward shift after a backward push
    // was on the step AFTER the push, not a retimed one. This relaxes the
    // inward bound to at least this value on retimed steps only.
    // 0 = off = shipped.
    let mpc_footstep_y_in_retimed = env_f64("MPC_FOOTSTEP_Y_IN_RETIMED", 0.0).max(0.0);
    // Inward bound while the SAGITTAL DCM error is forward by at least
    // MPC_FOOTSTEP_Y_IN_FWD_TRIG [m] (world x; the walk heads +x). Measured
    // 2026-09-27: the inward shifts that save 11 N.s forward pushes happen
    // at x error +10..+17 mm; the harmful ones after backward pushes at
    // x error -16..-34 mm (push step) or ~0 with a large lateral error (the
    // next step). The retimed-step rule above does not separate them (a
    // backward push shortens steps too). 0 = off = shipped.
    let mpc_footstep_y_in_fwd = env_f64("MPC_FOOTSTEP_Y_IN_FWD", 0.0).max(0.0);
    let mpc_footstep_y_in_fwd_trig = env_f64("MPC_FOOTSTEP_Y_IN_FWD_TRIG", 0.005);
    // The same gate for a RISING swing (the landing more than 5 mm above the
    // lift-off, i.e. a stair step). Stairs want an early freeze (the hover
    // above the tread), flat pushes want a late one (a push at 0.75 of the
    // swing still needs the landing). Default = the flat value.
    let mpc_footstep_freeze_frac_rise =
        env_f64("MPC_FOOTSTEP_FREEZE_FRAC_RISE", mpc_footstep_freeze_frac);
    let mpc_fs_debug = std::env::var("MPC_FS_DEBUG").is_ok();
    // Arm the landing variable only once the DCM error of this single
    // support reaches this value [m] (latched for the rest of the step), the
    // rule the contact_plan adjuster uses (CP_TRIG): an undisturbed step
    // then solves exactly the fixed-contact problem. Measured 2026-09-27:
    // with the variable always present, D+J loses 6-8 cm wide-holdout
    // stairs on two independent seeds (pooled 1 better / 10 worse) while
    // the variable itself stays near 0 where the runs part -- the cost is
    // its presence, not its choice. 0 = always armed = shipped.
    let mpc_footstep_trig = env_f64("MPC_FOOTSTEP_TRIG", 0.0).max(0.0);
    // Keep the variable armed for this many FOLLOWING steps once the trigger
    // fires. Measured 2026-09-27 (D+J+T, 11 N.s backward, m32): the step
    // after the push recovery starts below the threshold, the DCM error
    // creeps +4 -> +9 mm, the trigger fires only at the end of the swing and
    // the landing jumps 32 mm sideways at once; always-armed D+J corrects the
    // same step continuously and survives. 0 = arm only the step that
    // triggered = shipped.
    let mpc_footstep_trig_hold = env_f64("MPC_FOOTSTEP_TRIG_HOLD", 0.0).max(0.0) as usize;
    // VXS_BOX_GAIN [1/(m/s)]: with a speed command the landing box grows
    // by (1 + gain * (v - 0.12)): a push at speed needs a capture step
    // scaled with the stride (measured 2026-10-14, the 0.12 m/s box
    // clipped every lateral recovery at 0.30-0.50 m/s). 0 = fixed box.
    let vxs_box_scale = match vx2_change {
        Some((_, v, _)) => 1.0 + env_f64("VXS_BOX_GAIN", 0.0).max(0.0) * (v - 0.12).max(0.0),
        None => 1.0,
    };
    let mpc_footstep_max = [
        env_f64("MPC_FOOTSTEP_MAX_X", 0.06).max(0.0) * vxs_box_scale,
        env_f64("MPC_FOOTSTEP_MAX_Y", 0.03).max(0.0) * vxs_box_scale,
    ];
    // Weight on tracking the PLAN'S DCM inside the MPC. Without it the MPC's
    // implied feedback law is not a DCM law -- measured 2026-09-25, against a
    // 49 mm divergence the shipped cost moves the pressure 2.7 mm, where a
    // DCM law moves it past 49 mm -- so the MPC and the WBC were running
    // different feedback on the same plant and their double-support force
    // split would not close below 33 N. 0 = off = bit-identical.
    let mpc_dcm_weight = env_f64("MPC_DCM_W", 0.0).max(0.0);
    let mpc_net_cop_eq = flag("MPC_NET_COP_EQ", false);
    let mpc_w_pos = env_f64("MPC_W_POS", 20.0).max(0.0);
    let mpc_w_vel = env_f64("MPC_W_VEL", 4.0).max(0.0);
    let mpc_w_mom = env_f64("MPC_W_MOM", 2.0).max(0.0);
    let mpc_w_acc = env_f64("MPC_W_ACC", 6.0).max(0.0);
    let mpc_w_wrench = env_f64("MPC_W_WRENCH", 2e-4).max(0.0);
    let mpc_w_smooth = env_f64("MPC_W_SMOOTH", 1e-3).max(0.0);
    let mpc_sole_authority = flag("MPC_SOLE_AUTHORITY", false);
    // MPC_REACH: the CoM height becomes the MPC's to decide. Each node gets
    // a cap = what the legs can reach from the feet the schedule has down
    // (sole + sqrt(L^2 - d^2) + hip drop, L = MPC_REACH x the straight
    // leg, d the horizontal hip-to-sole distance at the PREVIOUS solve's
    // predicted CoM), the MPC's vertical target is the plan height itself
    // (no PD extrapolation), and the WBC's vertical PD then tracks the
    // MPC's predicted height (MPC_REACH_TRACK). The three height heuristics
    // (COM_HEIGHT_DESC_LEAD, PUSH_Z_DROP, VX2_Z_DROP) are what this is
    // meant to retire. 0 = off, bit-identical.
    let mpc_reach = env_f64("MPC_REACH", 0.0).clamp(0.0, 1.0);
    let mpc_reach_w = env_f64("MPC_REACH_W", 1e6).max(0.0);
    let mpc_reach_max_drop = env_f64("MPC_REACH_MAX_DROP", 0.06).max(0.0);
    // Only a foot scheduled to carry at least this fraction of the weight
    // at a node constrains the height there. Measured 2026-10-10: without
    // it the TRAILING leg at the end of every double support (0.2 m behind
    // the CoM, unloading, reach 0.97) pulled the height down 3 cm each step.
    let mpc_reach_min_load = env_f64("MPC_REACH_MIN_LOAD", 0.3).clamp(0.0, 1.0);
    // Only a loaded foot AHEAD of the predicted CoM (by more than this
    // [m] along +x) constrains the height. Measured 2026-10-10 (N=48,
    // margin 0.95, PUSH_Z_DROP off): the cap on every loaded leg took the
    // backward push 34 -> 44 but the forward push 36 -> 25 -- the same
    // trade as lowering Z_COM outright. Behind the CoM a stretched leg is
    // the trailing one, and lowering there only speeds the divergence
    // (omega) the forward step has to catch. Negative = off (all feet).
    let mpc_reach_ahead = env_f64("MPC_REACH_AHEAD", -1.0);
    // Measure the hip-to-sole distance along x only. Measured 2026-10-10
    // (0.96 / ahead 0): the full distance took the lateral pushes 152 ->
    // 141 (1/12) -- a sideways displacement lowered the CoM, and lowering
    // costs the lateral recovery as it costs the forward one.
    let mpc_reach_sagittal = flag("MPC_REACH_SAGITTAL", false);
    // How the WBC's vertical follows the cap: 1 = its PD tracks the MPC's
    // predicted height (measured 2026-10-10: the prediction is the measured
    // load continued -- the smoothness anchor outweighs the height cost by
    // 1e5 -- so the PD had nothing to restore and the height wandered 3 cm
    // per step); 2 = the height reference itself is lowered to the lowest
    // cap in the horizon, slewed at MPC_REACH_SLEW [m/s] (default).
    let mpc_reach_track = env_f64("MPC_REACH_TRACK", 2.0) as i32;
    let mpc_reach_slew = env_f64("MPC_REACH_SLEW", 0.05).max(0.001);
    // Lowest cap of the last solve, and the drop currently applied.
    let mut mpc_reach_cap_min = f64::INFINITY;
    let mut mpc_reach_drop = 0.0_f64;
    let mpc_reach_debug = std::env::var("MPC_REACH_DEBUG").is_ok();
    let mpc_authority_ds_only = !flag("MPC_AUTHORITY_ALL", false);
    let ramp_frac_mpc = env_f64("LOAD_RAMP_FRAC", 0.6).clamp(0.05, 1.0);
    let stair_wrench_dt = env_f64("STAIR_WRENCH_DT", 0.02).max(dt);
    // Optional age limit for a held MPC result after a failed update.
    // Zero preserves the historical hold behavior; use with event replanning.
    let stair_wrench_max_age = env_f64("STAIR_WRENCH_MAX_AGE", 0.0).max(0.0);
    let stair_wrench_horizon = env_f64("STAIR_WRENCH_N", 8.0).max(2.0) as usize;
    let stair_wrench_preview_dt = env_f64("STAIR_WRENCH_PREVIEW_DT", 0.05).max(dt);
    let stair_wrench_low_min = env_f64("STAIR_WRENCH_LOW_MIN", 0.12).clamp(0.0, 0.45);
    let stair_wrench_high_min = env_f64("STAIR_WRENCH_HIGH_MIN", 0.05).clamp(0.0, 0.45);
    let stair_touchdown_load_time = env_f64("STAIR_TOUCHDOWN_LOAD_TIME", 0.50).max(0.05);
    let stair_touchdown_high_share = env_f64("STAIR_TOUCHDOWN_HIGH_SHARE", 0.25).clamp(0.05, 0.45);
    let stair_wrench_accel = flag("STAIR_WRENCH_ACCEL", false);
    let stair_wrench_accel_xy = flag("STAIR_WRENCH_ACCEL_XY", false);
    // MPC_XI_REF: keep the WBC's DCM law, but take its REFERENCE (xi_ref,
    // xi_dot_ref) from the MPC's predicted CoM trajectory instead of the
    // open-loop DCM plan. This is the "WBC tracks the MPC trajectory" shape:
    // the MPC decides where the CoM goes, the 200 Hz DCM law realises it.
    // Handing over the node-0 acceleration directly did not work (Sec.47).
    let mpc_xi_ref = flag("MPC_XI_REF", false);
    // Look-ahead [s] into the MPC trajectory for the reference. At 0 the
    // reference is the prediction AT t, which starts from the measured state
    // and therefore equals it: the DCM law's error term vanishes and only the
    // reference-rate term is left, which is positive feedback -- measured
    // 2026-09-26 the standing robot ran away in 1.2 s with xi_ref == xi at
    // every tick. Sampling one node or more ahead keeps the error term.
    let mpc_xi_ref_lead = env_f64("MPC_XI_REF_LEAD", 0.0).max(0.0);
    let mpc_xi_ref_transfer_plan = flag("MPC_XI_REF_TRANSFER_PLAN", false);
    let stair_wrench_floor = flag("STAIR_WRENCH_FLOOR", false);
    // (B) Do not hand the whole load to the high foot by the end of Capture.
    //
    // Measured: the trailing foot sits BEHIND and BELOW the centre of mass, so
    // its vertical force is a nose-UP moment about the CoM. As Capture drives
    // alpha past ~0.85 that force goes away, the high foot's CoP swings from
    // its heel (-40 mm) to its toe (+46 mm of a +53 mm sole) to supply the
    // moment alone, saturates, and the trunk pitches over the toe: 0.1 deg ->
    // 25 deg in 0.28 s while the leg torques never exceed a quarter of their
    // limit. It is CoP authority that runs out, not torque.
    //
    // The floor that fixes this already exists -- it is the minimum normal
    // force of HRP-4's wrench distribution QP (Caron et al., ICRA 2019), and
    // `STAIR_WRENCH_FLOOR` implements it -- but it is reachable only with
    // `STAIR_WRENCH_MPC` on. This flag lets the floor run without the MPC.
    let stair_capture_floor = flag("STAIR_CAPTURE_FLOOR", false);
    // Ankle-moment regularisation, the second term of HRP-4's wrench
    // distribution QP (weight 100, against 10000 for net-wrench tracking).
    // Its job there is to remove the redundancy of splitting one net wrench
    // between two soles. Measured here, that redundancy is not benign: on the
    // non-coplanar double support the CoP of BOTH soles flips by 20-32 mm RMS
    // every 5 ms IN THE PLANT (`cop_mj`, not just the QP's own solution),
    // while the trunk pitch grows at lambda = 29.4 /s -- doubling every 24 ms,
    // 5.4x the ankle-pivot inverted pendulum rate sqrt(g/h) = 5.4 /s. A body
    // that topples does it at the pendulum rate; this is a closed-loop mode.
    // Weight is in units of (m g L)^2. 0 = off.
    let stair_ankle_reg = env_f64("STAIR_ANKLE_REG", 0.0).max(0.0);
    // Let the trunk compete with the CoM instead of living under it.
    //
    // Measured on the fixed-arm one-step: up to t=12.550 the trunk task's
    // residual is EXACTLY 0.00 -- the WBC delivers the commanded angular
    // acceleration. From t=12.575 the task asks for -9, -23, -39, ... -132
    // rad/s^2 (nose UP, correctly fighting the pitch) and the solution
    // delivers +9, +13, +29, ... +14 -- the OPPOSITE SIGN, residual RMS 76.
    // The trunk is not failing to be tracked; it is being overruled.
    //
    // The level above it is the CoM. With both soles rigidly anchored, the
    // only way left to carry the CoM forward onto the tread is to lean: a
    // 25 deg pitch of a trunk holding 27.4% of the mass moves the CoM about
    // 17 mm, and the CoM command asks for 24 mm over that window. The
    // hierarchy therefore spends the trunk to buy CoM tracking.
    //
    // `STAIR_WRENCH_TRUNK_COUPLED` already merges the two into one weighted
    // level, but only with the wrench MPC on. This is the same merge for the
    // transfer without it. Weight is relative to the CoM rows. 0 = off.
    let stair_trunk_couple = env_f64("STAIR_TRUNK_COUPLE", 0.0).max(0.0);
    // Low-foot floor during CAPTURE only, as a fraction of body weight.
    // Negative = fall back to STAIR_WRENCH_LOW_MIN (so this is inert unless
    // asked for).
    let stair_capture_low_min = env_f64("STAIR_CAPTURE_LOW_MIN", -1.0).min(0.45);
    let stair_wrench_blend = env_f64("STAIR_WRENCH_BLEND", 0.25).clamp(0.0, 1.0);
    let stair_wrench_coupled = flag("STAIR_WRENCH_COUPLED", true);
    let stair_wrench_level_weight = env_f64("STAIR_WRENCH_LEVEL_WEIGHT", 1.0).max(0.0);
    let stair_wrench_com_weight = env_f64("STAIR_WRENCH_COM_WEIGHT", 10.0).max(0.0);
    let stair_wrench_mom_weight = env_f64("STAIR_WRENCH_MOM_WEIGHT", 0.2).max(0.0);
    let stair_wrench_trunk_weight = env_f64("STAIR_WRENCH_TRUNK_WEIGHT", 2.0).max(0.0);
    let stair_wrench_trunk_coupled = flag("STAIR_WRENCH_TRUNK_COUPLED", false);
    let stair_managed_contact = flag("STAIR_MANAGED_CONTACT", true);
    let stair_managed_rise = flag("STAIR_MANAGED_RISE", false);
    let stair_managed_fz = env_f64("STAIR_MANAGED_FZ", 0.20).clamp(0.0, 0.8);
    let stair_managed_tangent = env_f64("STAIR_MANAGED_TANGENT", 0.03).clamp(0.0, 1.0);
    let stair_managed_rotation = env_f64("STAIR_MANAGED_ROTATION", 0.05).clamp(0.0, 1.0);
    let stair_managed_az_max = env_f64("STAIR_MANAGED_AZ_MAX", 15.0).max(0.1);
    // Trigger Managed on the RATE of the normal force, not only its level.
    // Measured on the fixed-arm stair transfer: during the chatter fz swings
    // 15 <-> 78 N, so a level test catches only the dips and leaves the
    // contact rigid through the rest of the oscillation. The chatter runs at
    // ~1200 N/s against ~200 N/s for an intended transfer, so the rate
    // separates them. 0 = off (shipped behaviour).
    let stair_managed_rate = env_f64("STAIR_MANAGED_RATE", 0.0).max(0.0);
    // Hold Managed this long once tripped, so the trigger does not itself
    // chatter at the rate it is suppressing.
    let stair_managed_latch = env_f64("STAIR_MANAGED_LATCH", 0.10).max(0.0);
    // Couple the plant's measured contact set into both the horizon and WBC.
    // An absent foot is recovered kinematically and cannot provide a wrench
    // until ContactCorrection's load hysteresis admits it again.
    let stair_hybrid_contact = flag("STAIR_HYBRID_CONTACT", true);
    // Smooth the newly landed foot's authority instead of switching its full
    // contact wrench and acceleration task on/off with every load sample.
    let stair_continuous_contact = flag("STAIR_CONTINUOUS_CONTACT", false);
    // Which foot the Commit phase settles the CoM onto.
    //
    // The default (false) settles onto `1 - high`, the trailing foot still on
    // the lower surface. Measured 2026-09-22 on G1, riser 40 mm: `high` is the
    // foot that just LANDED on the tread, and the gait's next single support
    // is stance=high, swing=1-high -- so the default drives the CoM onto the
    // very foot that lifts a tick later. Both runs that ever reached Commit
    // (STAIR_CONTINUOUS_CONTACT=1, the first completions G1 has produced) fell
    // sideways within 1.0 s of `complete`, CoM y running out to +0.27 m with
    // the ZMP pinned at the stance sole's inner edge. Setting this true commits
    // onto the high foot instead, which is the one that stays down.
    //
    // OPT-IN so kyo46rs stays bit-identical: kyo46rs has never completed a
    // transfer either, so the default is untested rather than known-good, but
    // changing it silently would move a robot this was not measured on.
    let stair_commit_on_high = flag("STAIR_COMMIT_ON_HIGH", false);
    // Ramp a NEW contact in over this many seconds, on any ground, with no
    // stair transfer in sight. 0 = off = shipped behaviour.
    //
    // Everything Managed above lives inside a stair transfer
    // (`stair_wrench_mpc && stair_managed_contact && stair_transfer_cmd...`),
    // so on flat ground every contact is admitted Rigid in a single tick.
    // Measured on the G1 bring-up (doc/from_claude/20260919_g1_transfer_probe.md):
    // at the 1 -> 2 contact-set change the QP's total vertical force collapsed
    // 1.03 -> 0.45 BW for one tick and took 50 ms to climb back, both feet flat
    // on the floor the whole time. The CoM drop that opened made the CoM task
    // demand +15 m/s^2 -- 1.5 g upward -- and the QP paid for it with a 1.81 BW
    // slam that launched the swing leg to 0.82 m. kyo46rs never dips below
    // 0.85 BW walking on the flat, which is why this went unseen there.
    let touchdown_ramp_g = phase_gain("TOUCHDOWN_RAMP", env_f64("TOUCHDOWN_RAMP", 0.0));
    // Show the QP a scheduled-SWING foot that is nevertheless carrying load.
    //
    // `ContactCorrection` drops a commanded swing foot from the contact set
    // unconditionally, and for good reason -- keeping it as a rigid contact
    // pins it and the swing task never runs (20 steps became a shuffle, apex
    // 0.2-1.2 mm). But the QP then solves as if the foot were free while the
    // plant pushes back through it. Measured on the G1 bring-up: at t=3.050
    // the left sole carried 0.361 BW with `n_stance == 1`, and the tick the
    // QP solved against that missing reaction is the tick total support
    // collapsed 1.03 -> 0.45 BW.
    //
    // So admit it, but only as a WEIGHTED contact (ContactMode::Managed is a
    // weighted objective, not a constraint), at `SWING_CONTACT_SOFT` x the
    // usual weights, with a force capacity of `SWING_CONTACT_CAP`. Weak enough
    // to leave the swing its null space, present enough that the reaction is
    // in the model. 0 = off = shipped behaviour.
    let swing_contact_soft_g = phase_gain("SWING_CONTACT_SOFT", env_f64("SWING_CONTACT_SOFT", 0.0));
    let swing_contact_soft_after = env_f64("SWING_CONTACT_SOFT_AFTER", 0.0).max(0.0) as u32;
    let swing_contact_on = env_f64("SWING_CONTACT_ON", 0.10).clamp(0.01, 1.0);
    let swing_contact_cap = env_f64("SWING_CONTACT_CAP", 0.30).clamp(0.02, 1.0);
    // This ramp is planned from the touchdown event. It deliberately does not
    // follow fz: feeding measured load back into its own capacity made a
    // positive-feedback loop during contact chatter.
    let stair_contact_activation_time = env_f64("STAIR_CONTACT_ACTIVATION_TIME", 0.40).max(dt);
    let stair_contact_min = env_f64("STAIR_CONTACT_MIN", 0.03).clamp(0.0, 0.25);
    // Rise may commit only after the measured left/right load split has
    // settled. Zero disables this gate for baseline compatibility.
    let stair_load_chatter_max = env_f64("STAIR_LOAD_CHATTER_MAX", 0.0).max(0.0);
    let stair_load_chatter_tau = env_f64("STAIR_LOAD_CHATTER_TAU", 0.20).max(dt);
    let stair_recontact_load = env_f64("STAIR_RECONTACT_LOAD", 0.05).clamp(0.01, 0.30);
    let stair_recontact_alpha = env_f64("STAIR_RECONTACT_ALPHA", 0.80).clamp(0.50, 0.98);
    let stair_recontact_z = env_f64("STAIR_RECONTACT_Z", 0.004).max(0.001);
    let stair_recontact_angle = env_f64("STAIR_RECONTACT_ANGLE", 0.10).max(0.01);
    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    enum StairTransferPhase {
        Capture,
        Rise,
        Commit,
    }
    impl StairTransferPhase {
        fn code(self) -> f64 {
            match self {
                Self::Capture => 1.0,
                Self::Rise => 2.0,
                Self::Commit => 3.0,
            }
        }
    }
    #[derive(Clone, Copy, Debug)]
    struct TouchdownLoadRamp {
        high: usize,
        contact_t: f64,
        start_share: [f64; 2],
        support_target: na::Vector2<f64>,
        seed_previous: bool,
    }
    struct ActiveStairTransfer {
        ds_slice: usize,
        high: usize,
        phase: StairTransferPhase,
        plan: articara::biped::preview::StairTransferPlan,
        capture_target: na::Vector2<f64>,
        /// DCM reference velocity the walk had at the moment this transfer
        /// armed. See the `STAIR_TRANSFER_DRIFT` use site.
        arm_xi_dot: na::Vector2<f64>,
        target_z: f64,
        clock: f64,
        deadline: f64,
        ready_ticks: u32,
        paused_ticks: u32,
        timed_out: bool,
        hold_started: Option<f64>,
        /// rise_first: the low foot is still anchored (not yet released).
        low_held: bool,
        /// rise_first: share of weight on the high foot at arming.
        arm_share: f64,
        /// rise_first: smoothed share of weight on the high foot.
        share_ema: f64,
    }
    #[derive(Clone, Copy, Debug)]
    struct PostWalkSettle {
        start: f64,
        start_xi: na::Vector2<f64>,
        target: na::Vector2<f64>,
        z: f64,
    }
    let mut stair_transfer: Option<ActiveStairTransfer> = None;
    let mut touchdown_load_ramp: Option<TouchdownLoadRamp> = None;
    let mut stair_capture_share: Option<f64> = None; // EMA of the high foot's share of weight
    let mut stair_capture_p_prev: Option<na::Vector2<f64>> = None;
    let mut stair_transfer_started = 0u32;
    let mut stair_transfer_completed = 0u32;
    let mut first_post_swing_slice: Option<usize> = None;
    let mut stair_transfer_timeouts = 0u32;
    let mut post_transfer_steps: Option<usize> = None;
    let mut post_transfer_touchdowns: Option<u32> = None;
    let mut post_walk_settle: Option<PostWalkSettle> = None;
    let mut post_settle_contacts_latched = false;
    let mut post_walk_settle_ready_s = 0.0_f64;
    let mut replan_engaged = false;
    let mut height_plan = articara::biped::preview::HeightPlan::constant(z_com);
    let mut preview_p = steps.mid_xy();
    // The CoM preview QP's horizontal trajectory, kept whole with the time it
    // was solved at, so the MPC can read its own horizon out of it.
    let mut preview_traj: Option<(f64, Vec<na::Vector2<f64>>, Vec<na::Vector2<f64>>)> = None;
    let mut preview_c = com0.xy();
    let mut preview_v = na::Vector2::zeros();
    let mut preview_next = 0.0;
    let mut preview_count = 0u32;
    let mut preview_failures = 0u32;
    let mut preview_error = 0.0;
    let mut com_preview_runtime = std::env::var("COM_PREVIEW_RUNTIME_CSV").ok().map(|path| {
        let mut writer = std::io::BufWriter::new(
            std::fs::File::create(path).expect("COM_PREVIEW_RUNTIME_CSV"),
        );
        writeln!(writer, "t,solve_ns,deadline_miss_50ms,accepted,terminal_error_m,applied_lateral,horizon").unwrap();
        writer
    });
    let mut landing_retimes = 0u32;
    let mut landing_delay = 0.0;
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
    let mut contact_phase = ContactPhase::new(
        phase_min_frac,
        phase_load_frac * total_mass * G,
        phase_ticks,
        phase_extend_max,
    );
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
            .map(|&ji| {
                rig.robot.joints[ji]
                    .lower
                    .abs()
                    .min(rig.robot.joints[ji].upper.abs())
            })
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
    let yaw0_body = rig
        .sim
        .body_world_orientation(&rig.robot.root_link)
        .unwrap()
        .euler_angles()
        .2;
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
    // Per-joint travel, so a joint that swings without hitting anything is
    // still visible. Sec.30 found the v6 arm at 111 deg peak-to-peak in a clip
    // that logged 44 self-collision ticks: the collision count is a count of
    // collisions, and it was being read as if it were a measure of behaviour.
    let mut q_lo: Vec<f64> = vec![f64::INFINITY; rig.robot.joints.len()];
    let mut q_hi: Vec<f64> = vec![f64::NEG_INFINITY; rig.robot.joints.len()];
    // Hits on `spar_target`, kept OUT of the self-collision tally: a contact
    // with the thing the robot is aiming at is not the QP solving against a
    // contact it has no model for, which is what that counter means.
    let mut hit_ticks = 0u32;
    let mut hit_peak_f: f64 = 0.0;
    let mut hit_impulse: f64 = 0.0;
    let mut hit_first_t: Option<f64> = None;
    let mut hit_last_t: f64 = 0.0;
    // Per-PUNCH, not per-run: the totals above sum every contact in the run,
    // and "how long does one punch stay on the target" is the question Sec.42.4
    // actually asked. An episode is a contiguous run of ticks in contact.
    let mut hit_episodes: Vec<(f64, f64)> = Vec::new(); // (duration s, impulse N*s)
    let mut ep_t0: Option<f64> = None;
    let mut ep_imp: f64 = 0.0;
    // This tick's force on the target, hoisted so the CSV can carry it: a
    // replay that draws the punch should not have to be told separately
    // what it delivered (same reason `push_fx`/`push_fy` are logged).
    let mut hit_f_now: f64 = 0.0;
    let mut fz_prev: [f64; 2] = [0.0; 2];
    let mut fz_prev2: [f64; 2] = [0.0; 2];
    let mut wbc_fz_previous: [Option<f64>; 2] = [None; 2];
    let mut fz_rate_ema: [f64; 2] = [0.0; 2];
    let mut managed_until: [f64; 2] = [f64::NEG_INFINITY; 2];
    // When each side last ENTERED the contact set; None while it is out.
    let mut touchdown_since: [Option<f64>; 2] = [None; 2];
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
    // Early-window counterparts (see `push_window_s`).
    let mut win_dcm_err_max: f64 = 0.0;
    let mut win_tilt_max: f64 = 0.0;
    let mut win_cop_max: f64 = 0.0;
    let mut win_slip_plant_max: f64 = 0.0;
    let mut win_slip_ticks: u32 = 0;
    let mut win_fz_total_min: f64 = f64::INFINITY;
    let mut win_fz_total_max: f64 = 0.0;
    let mut win_unloaded_ticks: u32 = 0;
    let mut win_degraded_at: Option<u32> = None;
    let mut win_degraded_end: u32 = 0;
    // First support change after the push, and the CoP-edge accounting over
    // the `transfer_s` that follows it.
    let mut transfer_t: Option<f64> = None;
    let mut support_prev: Option<Support> = None;
    let mut xfer_edge_ticks: u32 = 0;
    let mut xfer_ticks: u32 = 0;
    let mut xfer_leave_t: Option<f64> = None;
    let mut xfer_wrongfoot_ticks: u32 = 0;
    // Second single-support entry after the push. The first-window metric is
    // anchored to a transition that step-timing adaptation MOVES, so it scores
    // that intervention badly by construction (+27/-1 survivals against only
    // 36/29 on the metric). Measuring the next step too separates "the metric
    // is truncated" from "the intervention only helps survival".
    let mut transfer2_t: Option<f64> = None;
    let mut xfer2_edge_ticks: u32 = 0;
    let mut xfer2_ticks: u32 = 0;
    // Centroidal angular momentum about the roll axis. If damping it is going
    // to help, it has to be large and growing in runs that fall -- and the
    // arms are shoulder_pitch + elbow, both PITCH, so they cannot make roll
    // momentum at all. Measured before implementing the task, not after.
    let mut h_roll_max: f64 = 0.0;
    let mut h_roll_at_xfer: f64 = 0.0;
    // DCM offset relative to the foot the plan is about to commit to, at the
    // instant of commitment. The Khadiv/Englsberger nominal for a periodic
    // walk is b_y = l_p / (1 + exp(omega * t_ss)) with the sign alternating;
    // entering single support at the wrong sign or magnitude is the candidate
    // mechanism for the failures traced in Sec.19-21. Zero new machinery is
    // needed to check it -- the footstep plan and the measured DCM are both
    // already here.
    let mut xfer_dcm_off = [0.0f64; 2];
    let mut xfer_stance: i32 = -1;
    let mut push_ankle_max: f64 = 0.0;
    let mut push_exceeded = false;
    let mut push_last_exceed: Option<f64> = None;
    let mut t_last: f64 = 0.0;
    // Optional norm bound on the change in commanded horizontal CoM
    // acceleration.  The DCM/ZMP reference is acceleration-continuous, but
    // its piecewise-linear ZMP slope changes at support boundaries, producing
    // a jerk step.  Limiting that step preserves the full DCM feedback law
    // while giving contacts and counterweight joints time to respond.
    let com_xy_jerk_max = env_f64("COM_XY_JERK_MAX", f64::INFINITY);
    assert!(
        com_xy_jerk_max.is_finite() && com_xy_jerk_max > 0.0 || com_xy_jerk_max == f64::INFINITY,
        "COM_XY_JERK_MAX must be positive"
    );
    let mut previous_a_xy: Option<na::Vector2<f64>> = None;
    // Preserve the command actually sent on the previous control tick so a
    // contact-triggered transfer starts continuously from the live plan.
    let mut last_single_p_cmd: Option<na::Vector2<f64>> = None;
    let mut dcm_ref_prev: Option<na::Vector2<f64>> = None;
    // (pre-jump xi, plan xi at t0, t0). offset(s) = (pre - plan0) * (1 - smoothstep((s - t0)/T))
    let mut dcm_ref_blend: Option<(na::Vector2<f64>, na::Vector2<f64>, f64)> = None;
    // Same for the reference VELOCITY: (pre-jump xi_dot, plan xi_dot at t0).
    // `commanded_zmp` feeds `-xi_dot_ref/omega` forward, and at the cut the
    // rebuilt double-support segment carries a large xi_dot -- measured
    // 2026-09-23: with only `xi` blended, p_cmd still went to the sole edge.
    let mut dcm_ref_blend_dot: Option<(na::Vector2<f64>, na::Vector2<f64>)> = None;
    let mut dcm_ref_prev_dot: Option<na::Vector2<f64>> = None;
    // Optional P0 diagnostic: sampled entirely before stepping the plant.
    // A command computed at t is compared with the observation at t; its
    // physical response must be evaluated at subsequent observation times.
    let mut transfer_diag = std::env::var("TRANSFER_DIAG_CSV").ok().map(|path| {
        let mut file = std::io::BufWriter::new(std::fs::File::create(path).expect("transfer diagnostic CSV"));
        writeln!(file, "t,dt,bw,phase,high,alpha,stance_mask,measured_mask,mpc_valid,mpc_age,mpc_failed,mpc_ref_l,mpc_ref_r,mpc_fz_l,mpc_fz_r,nominal_world_fz_l,nominal_world_fz_r,qp_world_fz_l,qp_world_fz_r,measured_world_fz_l,measured_world_fz_r,trunk_pitch,trunk_roll,qp_cop_l_x,qp_cop_r_x,measured_cop_l_x,measured_cop_r_x,degraded,ffdc_dz").unwrap();
        file
    });
    // Opt-in acceleration-level audit at the right ankle-roll stop. The CSV
    // samples the state and QP decision in the same tick; the plant response
    // appears at later samples, so command-vs-response causality stays clear.
    let mut ankle_cbf_diag = std::env::var("ANKLE_CBF_DIAG_CSV").ok().map(|path| {
        let mut file = std::io::BufWriter::new(
            std::fs::File::create(path).expect("ankle CBF diagnostic CSV"),
        );
        writeln!(file, "t,q,v,q_max,cbf_upper,qp_qdd,measured_fz_r,qp_fx_r,qp_fy_r,qp_fz_r,measured_fx_r,measured_fy_r,post_q,post_v,mj_qacc,mj_qfrc_constraint,mj_qfrc_passive,mj_qfrc_actuator,command_tau").unwrap();
        file
    });
    let ankle_cbf_indices = rig.actuated().into_iter().find_map(|(ji, vi)| {
        (rig.robot.joints[ji].name == "right_ankle_roll_joint").then(|| {
            (
                rig.a2m[ji].expect("ankle misarta joint"),
                vi,
                ji,
                rig.sim
                    .joint_dof_adr("right_ankle_roll_joint")
                    .expect("ankle MuJoCo joint"),
            )
        })
    });
    let mut diag_mpc_reference = [f64::NAN; 2];
    let mut diag_mpc_time: Option<f64> = None;
    // Where the MPC's own answer puts the NET pressure of both feet, in
    // world. The split alone does not determine this -- each sole's own CoP
    // moves it too -- so it has to be computed where the wrenches and their
    // contact frames are both in hand.
    let mut diag_mpc_net_cop = [f64::NAN; 2];
    // The measured DCM at the moment of that solve (MPC_PCMD_FB).
    let mut diag_mpc_xi: Option<na::Vector2<f64>> = None;
    // The same quantity for the forces the WBC actually asked the robot for.
    let mut diag_wbc_net_cop = [f64::NAN; 2];
    // P2 opt-in: a held command belongs to one phase/contact topology.
    let wrench_event_replan = flag("STAIR_WRENCH_EVENT_REPLAN", false);
    let mut wrench_command_context: Option<(u8, u8, usize)> = None;
    let mut wrench_mpc_next = 0.0;
    let mpc_backend_override =
        std::env::var("MPC_QP_BACKEND")
            .ok()
            .map(|name| match name.as_str() {
                "active_set" => misa_wbc::QpSolver::ActiveSet,
                "clarabel" => misa_wbc::QpSolver::Clarabel,
                "proxqp" => misa_wbc::QpSolver::Proxqp,
                "osqp" => misa_wbc::QpSolver::Osqp,
                "hpipm" => misa_wbc::QpSolver::Hpipm,
                _ => panic!("unsupported MPC_QP_BACKEND: {name}"),
            });
    let mpc_warm = flag("MPC_QP_WARM", false);
    let mpc_diag_dir = std::env::var("MPC_DIAG_DIR").ok();
    let mut mpc_diag_count = 0usize;
    let mut mpc_workspace = articara::biped::wrench_mpc::Workspace::default();
    let mut wrench_mpc_command: Option<[articara::biped::wrench_mpc::Wrench; 2]> = None;
    // Preserve the physical origin/axes of the held MPC forecast for diagnostic shifts.
    let mut wrench_mpc_frames: Option<[articara::biped::wrench_mpc::Contact; 2]> = None;
    let mut wrench_mpc_acceleration = na::Vector3::zeros();
    // MPC_FOOTSTEP bookkeeping: which slice's landing is being moved and how
    // much of the MPC's offset has already been applied to it (the same
    // non-ratcheting rule as `cp_applied`).
    let mut mpc_fs_slice: Option<usize> = None;
    let mut mpc_fs_applied = na::Vector2::zeros();
    // (first slice, last slice) the landing variable is armed for.
    let mut mpc_fs_armed: Option<(usize, usize)> = None;
    let mut mpc_fs_last = na::Vector2::zeros();
    // (solve time, CoM at solve, CoM velocity at solve, predicted [c, v] per node end)
    let mut wrench_mpc_traj: Option<(f64, na::Vector3<f64>, na::Vector3<f64>, Vec<[na::Vector3<f64>; 2]>)> = None;
    // Planned dh over the MPC horizon, held between MPC updates. Zero until
    // the first solution, so a tick before the MPC has run reproduces the
    // zero-reference behaviour exactly.
    let mut wrench_mpc_momentum_rate = na::Vector3::zeros();
    let mut wrench_mpc_solves = 0u32;
    let mut wrench_mpc_failures = 0u32;
    let mut wrench_mpc_soft = 0u32;
    let mut contact_activation = [1.0_f64; 2];
    let mut contact_force_activation = [1.0_f64; 2];
    // FFDC state: the integrated relative-height correction, and the load
    // split the QP was asked for on the PREVIOUS tick. HRP-4 feeds back the
    // distribution QP's own output; `forces_nominal` is built after the
    // contact level that consumes the anchors, so the reference is one WBC
    // tick (5 ms) old. That lag is far inside an admittance whose gain moves
    // the feet at millimetres per second.
    let mut ffdc_dz = 0.0_f64;
    let mut ffdc_ref_share: Option<[f64; 2]> = None;
    let mut ffdc_err_log = 0.0_f64;
    let mut transfer_pitch_previous: Option<f64> = None;
    let mut capture_error_log = -1.0_f64;
    let mut capture_tilt_log = 0.0_f64;
    let mut trunk_cmd_rp = [0.0_f64; 2];
    let mut trunk_acc_rp = [0.0_f64; 2];
    let mut transfer_guard_log = -1.0_f64;
    let mut load_share_previous = 0.5_f64;
    let mut load_chatter_ema = 0.0_f64;

    for tick in 0..n_ticks {
        let qp_control_start = std::time::Instant::now();
        let mut mpc_capture_ns = 0u128;
        let t = tick as f64 * dt;
        t_last = t;
        // Which window this tick belongs to. `stair_transfer` still holds
        // last tick's state here, which is what "are we in a transfer" means.
        let run_phase = if stair_transfer.is_some() {
            RunPhase::Transfer
        } else if stair_transfer_completed > 0 {
            RunPhase::Post
        } else {
            RunPhase::Approach
        };
        // Shadow the scoped gains for the rest of the tick. Each branch is
        // skipped entirely when nothing overrides it, so an unscoped run
        // takes exactly the path it took before.
        let cop_frac = cop_frac_g.at(run_phase);
        let kp_sw_z = kp_sw_z_g.at(run_phase);
        let kp_sw_xy = kp_sw_xy_g.at(run_phase);
        let kd_sw = kd_sw_g.at(run_phase);
        let kp_trunk = kp_trunk_g.at(run_phase);
        let kd_trunk = kd_trunk_g.at(run_phase);
        let kp_sw_rot = kp_sw_rot_g.at(run_phase);
        let kd_sw_rot = kd_sw_rot_g.at(run_phase);
        let swing_rot_mode = swing_rot_mode_g.at(run_phase) as u32;
        let stair_td_sink = stair_td_sink_g.at(run_phase);
        let touchdown_ramp = touchdown_ramp_g.at(run_phase).max(0.0);
        let swing_contact_soft = if resume_ss_releases >= swing_contact_soft_after {
            swing_contact_soft_g.at(run_phase).clamp(0.0, 1.0)
        } else {
            0.0
        };
        let zmp_lat_scale = zmp_lat_scale_g.at(run_phase);
        if zmp_lat_scale_g.scoped() && zmp_lat_scale != dcm_plan_lat_scale {
            dcm_plan =
                DcmPlan::from_footsteps_rolled(&gait, &footsteps, z_lip, zmp_lat_scale, zmp_roll);
            dcm_plan_lat_scale = zmp_lat_scale;
            preview_next = t;
        }
        if cop_frac_g.scoped() {
            contact_cfg.cop_half = (sole_half_l * cop_frac, sole_half_w * cop_frac);
        }
        if jlim_amax_g.scoped() {
            jlim_cfg.a_max.fill(jlim_amax_g.at(run_phase));
        }
        if jlim_q_margin_g.scoped() {
            let margin = jlim_q_margin_g.at(run_phase).max(0.0);
            for row in 0..na_count {
                let mid = 0.5 * (rig.q_min[row] + rig.q_max[row]);
                let m = margin.min(0.45 * (rig.q_max[row] - rig.q_min[row]));
                jlim_cfg.q_min[row] = (rig.q_min[row] + m).min(mid);
                jlim_cfg.q_max[row] = (rig.q_max[row] - m).max(mid);
            }
            if arm_pitch_band > 0.0 {
                for (ji, vi) in rig.actuated() {
                    if rig.robot.joints[ji].name.ends_with("shoulder_pitch_joint") {
                        let row = vi - 6;
                        let centre = rig.q_seed[ji];
                        jlim_cfg.q_min[row] = jlim_cfg.q_min[row].max(centre - arm_pitch_band);
                        jlim_cfg.q_max[row] = jlim_cfg.q_max[row].min(centre + arm_pitch_band);
                    }
                }
            }
        }
        if land_frac_g.scoped() {
            landing_timing.min_fraction = land_frac_g.at(run_phase).clamp(0.0, 1.0);
        }
        if land_extend_rise > 0.0 {
            // Same test the landing detector itself uses to decide a swing is
            // a stair swing (`preview.rs`: the planned sole height changes by
            // more than 5 mm), so the slack applies to exactly the swings that
            // need it and to no others.
            let i = gait.index_at(t);
            let dz_swing = gait.slices[i].support.swing().map_or(0.0, |side| {
                footsteps.at_slice(i).sole[side].z
                    - footsteps.at_slice(i.saturating_sub(1)).sole[side].z
            });
            let rising = dz_swing.abs() > 0.005;
            let descending = dz_swing < -0.005;
            // LAND_CONFIRM_XY_DESC_CP=1 restricts it to swings whose end the
            // contact planner has retimed (no landing extension there, so a
            // missed confirmation goes straight to the timeout). Measured:
            // applied to every descending swing it removes the 0.25 s dwell
            // the bare descents rely on (2 steps x 6 cm 46 -> 34).
            landing_timing.load_threshold_override = if descending { land_confirm_fz_desc } else { None };
            landing_timing.event_end_on_touch = descending && contact_event_end_on_touch;
            landing_timing.xy_tol_override = if descending
                && (!land_confirm_xy_desc_cp || (contact_plan_on && cp_retimed_slice == Some(i)))
            {
                land_confirm_xy_desc
            } else if rising && !descending {
                land_confirm_xy_rise
            } else {
                None
            };
            // The contact planner owns the end of a step it has retimed; the
            // landing extension must not re-open it (assigned every tick, so
            // the planner's own zeroing would be undone here otherwise).
            landing_timing.max_extension = if contact_plan_on && cp_retimed_slice == Some(i) {
                if descending { land_extend_desc_cp } else { 0.0 }
            } else if descending {
                land_extend_desc
            } else if rising {
                land_extend_rise
            } else {
                land_extend_base
            };
        }
        // Read before the push fires later in this tick, so the window opens
        // on the tick AFTER the impulse -- the same convention the whole-run
        // accumulators use.
        let in_push_window = push_at_t.map(|tp| t - tp <= push_window_s).unwrap_or(false);
        if in_push_window {
            if win_degraded_at.is_none() {
                win_degraded_at = Some(tally.n);
            }
            win_degraded_end = tally.n;
        }
        let st = rig.sync();
        let (q, v, v_dvec, data) = (&st.q, &st.v, &st.v_dvec, &st.data);
        // Ground truth first: the phase logic, the contact set and the tasks
        // all solve against this, so it is read before any of them.
        let measured_c = measure_contacts(&rig, data, friction_mu);
        let measured_pre = [measured_c.f_w[0][2], measured_c.f_w[1][2]];
        // CONTACT_DUMP_T="t0,t1": print every MuJoCo contact (bodies, world
        // position, world force) on the ticks inside the window. Diagnostic
        // only; unset = nothing.
        if let Some((d0, d1)) = contact_dump_t
            && t >= d0
            && t <= d1
        {
            for c in rig.sim.contacts() {
                println!(
                    "  [con] t={t:.3} {} <-> {} pos=({:+.4},{:+.4},{:+.4}) f=({:+.1},{:+.1},{:+.1})",
                    if c.body1.is_empty() { "world" } else { c.body1.as_str() },
                    if c.body2.is_empty() { "world" } else { c.body2.as_str() },
                    c.pos[0], c.pos[1], c.pos[2], c.force_world[0], c.force_world[1], c.force_world[2]
                );
            }
        }
        let measured_total = measured_pre[0].max(0.0) + measured_pre[1].max(0.0);
        let measured_left_share = if measured_total > 1e-6 {
            measured_pre[0].max(0.0) / measured_total
        } else {
            load_share_previous
        };
        if touchdown_load_ramp.is_some() {
            let change = (measured_left_share - load_share_previous).abs();
            let alpha = (ctrl_dt / stair_load_chatter_tau).clamp(0.0, 1.0);
            load_chatter_ema += alpha * (change - load_chatter_ema);
        }
        load_share_previous = measured_left_share;
        let com = st.com;
        let com_vel = st.com_vel;

        if com_replan {
            let actual = Footsteps::from_fk(
                data,
                rig.foot_mi,
                prof.sole_centre_x,
                prof.sole_below_origin,
            );
            if vx2_reanchor
                && vx2_change.is_some()
                && ref_mode.is_none()
                && let (Some(Support::Double), Support::Single { stance, swing }) = (support_prev, gait.support_at(t))
            {
                let i = gait.index_at(t);
                let d = actual.sole[stance].xy() - footsteps.at_slice(i).sole[stance].xy();
                if d.norm() > 0.002 {
                    footsteps.shift_from(i, stance, d);
                    footsteps.shift_from(i, swing, d);
                    dcm_plan = DcmPlan::from_footsteps_rolled(&gait, &footsteps, z_lip, zmp_lat_scale, zmp_roll);
                    n_reanchor += 1;
                    reanchor_total += d.norm();
                }
            }
            landing_timing.level_steps = land_confirm_level && stair_transfer_completed > 0;
            let touchdowns_before = landing_timing.touchdowns;
            let timeouts_before = landing_timing.timeouts;
            let shift = landing_timing.update(&mut gait, &footsteps, &actual, measured_pre, t, dt);
            let landing_timed_out = landing_timing.timeouts > timeouts_before;
            // The slice that just ended is the timer's clock slice, not
            // `gait.index_at(t)`: on a timeout tick the index already points
            // at the double support, whose swing side is None.
            if contact_admit_confirmed
                && (landing_timing.touchdowns > touchdowns_before || landing_timed_out)
                && let Some((ended, _, _)) = landing_timing.clock
                && let Some(sw) = gait.slices[ended].support.swing()
                && (footsteps.at_slice(ended).sole[sw].z - actual.sole[sw].z).abs() < contact_admit_z
            {
                confirmed_admit[sw] = Some(ended + 1);
                if std::env::var("CONTACT_ADMIT_DEBUG").is_ok() {
                    println!("  [adm] t={t:.3} certificate side={sw} ds_slice={} (confirm={} timeout={})", ended + 1, landing_timing.touchdowns > touchdowns_before, landing_timed_out);
                }
            }
            if let Some((li, lsw, err_xy)) = landing_timing.last_event.take()
                && contact_event_shift
                && err_xy.norm() > 1e-4
            {
                // plan - actual = err  ->  move the plan by -err onto the foot.
                footsteps.shift_next(li, lsw, -err_xy);
                dcm_plan = DcmPlan::from_footsteps_rolled(
                    &gait,
                    &footsteps,
                    z_lip,
                    zmp_lat_scale,
                    zmp_roll,
                );
            }
            if land_by_load
                && let Some((li, lsw, err_xy)) = landing_timing.last_load_confirm.take()
                && err_xy.norm() > 1e-4
            {
                if std::env::var("LAND_BY_LOAD_DEBUG").is_ok() {
                    let pl = footsteps.at_slice(li).sole[lsw];
                    let ac = actual.sole[lsw];
                    println!(
                        "  [lbl] t={t:.3} slice={li} side={lsw} plan=({:.3},{:+.3},{:+.3}) actual=({:.3},{:+.3},{:+.3}) err_xy=({:+.3},{:+.3}) fz=({:.0},{:.0})",
                        pl.x, pl.y, pl.z, ac.x, ac.y, ac.z, err_xy.x, err_xy.y, measured_pre[0], measured_pre[1]
                    );
                }
                // plan - actual = err  ->  move the plan by -err onto the foot.
                footsteps.shift_next(li, lsw, -err_xy);
                dcm_plan = DcmPlan::from_footsteps_rolled(
                    &gait,
                    &footsteps,
                    z_lip,
                    zmp_lat_scale,
                    zmp_roll,
                );
                n_land_by_load_shift += 1;
            }
            if shift != 0.0 || (ds_feasible_on_timeout && landing_timed_out) {
                if shift != 0.0 {
                    landing_retimes += 1;
                    landing_delay += -shift;
                }
                dcm_plan = DcmPlan::from_footsteps_rolled(
                    &gait,
                    &footsteps,
                    z_lip,
                    zmp_lat_scale,
                    zmp_roll,
                );
                preview_next = t;
                if land_wait_hold
                    && let Some((li, _, _)) = landing_timing.clock
                    && gait.index_at(t) == li
                    && gait.slices[li].support.swing().is_some()
                    && li + 1 < gait.slices.len()
                    && matches!(gait.slices[li + 1].support, Support::Double)
                    && li + 1 < dcm_plan.segs.len()
                {
                    let seg_ds = li + 1;
                    let hold = dcm_plan.segs[li].p1;
                    let d = (-omega * dcm_plan.segs[seg_ds].duration).exp();
                    let target = dcm_plan.xi_eos[seg_ds];
                    let ds_box = SupportBox::from_stance_yawed(
                        footsteps.at_slice(seg_ds),
                        Support::Double,
                        (sole_half_l * cop_frac, sole_half_w * cop_frac),
                        zmp_margin,
                    );
                    let p = ds_box.clamp(&((hold - target * d) / (1.0 - d))).0;
                    dcm_plan.set_segment_zmp(seg_ds, p);
                    n_wait_hold += 1;
                }
                if ds_feasible {
                    let ds = gait.index_at(t + 1e-6);
                    if matches!(gait.slices[ds].support, Support::Double) {
                        let xi_meas = dcm_of(&com, &com_vel, omega);
                        let seg = dcm_plan.index_at(t + 1e-6);
                        let target = dcm_plan.xi_eos[seg];
                        let ds_box_full = SupportBox::from_stance_yawed(
                            &actual,
                            Support::Double,
                            (sole_half_l * cop_frac, sole_half_w * cop_frac),
                            zmp_margin,
                        );
                        // The support that just ended tells which foot carries the load.
                        let prev_sw = ds
                            .checked_sub(1)
                            .and_then(|j| gait.slices[j].support.swing());
                        let ds_box = match (ds_feasible_box_stance, prev_sw) {
                            (true, Some(sw)) => SupportBox::from_stance_yawed(
                                &actual,
                                Support::Single { stance: 1 - sw, swing: sw },
                                (sole_half_l * cop_frac, sole_half_w * cop_frac),
                                zmp_margin,
                            ),
                            _ => SupportBox { lo: ds_box_full.lo, hi: ds_box_full.hi },
                        };
                        // Pivot: a box point on the far side of the DCM from
                        // the target, so the DCM is pushed TOWARD the target
                        // (`xi_dot = omega (xi - p)`). Clamping the DCM itself
                        // put the pivot on the DCM when it was inside the box
                        // (gap zero, `Degenerate`) and the branch never fired
                        // -- measured 2026-09-24, 0 retimes in 384 runs.
                        let nominal = gait.slices[ds].duration();
                        let p_continuous = |tt: f64| -> na::Vector2<f64> {
                            let d = (-omega * tt).exp();
                            ds_box.clamp(&((xi_meas - target * d) / (1.0 - d))).0
                        };
                        let p = if ds_feasible_continuous {
                            p_continuous(nominal)
                        } else {
                            ds_box.clamp(&(xi_meas * 2.0 - target)).0
                        };
                        let mut need = 0.0_f64;
                        for axis in 0..2 {
                            if let StepTiming::Time(tt) = step_timing(&xi_meas, &target, &p, omega, axis) {
                                need = need.max(tt);
                            }
                        }
                        let want = need.clamp(nominal, ds_feasible_max);
                        if ds_feasible_continuous {
                            if want > nominal + 1e-3 {
                                let dd = gait.retime(ds, gait.slices[ds].t0 + want);
                                n_ds_feasible += 1;
                                ds_feasible_total += dd.abs();
                                dcm_plan = DcmPlan::from_footsteps_rolled(
                                    &gait,
                                    &footsteps,
                                    z_lip,
                                    zmp_lat_scale,
                                    zmp_roll,
                                );
                            }
                            let seg_ds = dcm_plan.index_at(gait.slices[ds].t0 + 1e-6);
                            let p_final = if ds_feasible_ramp && seg_ds + 1 < dcm_plan.segs.len() {
                                let tt = gait.slices[ds].duration();
                                let wt = omega * tt;
                                let d = (-wt).exp();
                                let b = (1.0 - d) / wt - d;
                                let a = 1.0 - (1.0 - d) / wt;
                                let p1 = dcm_plan.segs[seg_ds + 1].p0;
                                let p0 = ds_box.clamp(&((xi_meas - target * d - p1 * b) / a)).0;
                                dcm_plan.set_segment_zmp_ramp(seg_ds, p0, p1);
                                p0
                            } else {
                                let p = p_continuous(gait.slices[ds].duration());
                                dcm_plan.set_segment_zmp(seg_ds, p);
                                p
                            };
                            n_ds_continuous += 1;
                            if std::env::var("DS_FEASIBLE_DEBUG").is_ok() {
                                let r = dcm_plan.reference(gait.slices[ds].t0 + 1e-6);
                                println!("  [dsc] t={t:.3} ds={ds} xi=({:.3},{:+.3}) target=({:.3},{:+.3}) p=({:.3},{:+.3}) T={:.3} xi_ref(t0)=({:.3},{:+.3})", xi_meas.x, xi_meas.y, target.x, target.y, p_final.x, p_final.y, gait.slices[ds].duration(), r.xi.x, r.xi.y);
                            }
                        } else if want > nominal + 1e-3 {
                            let dd = gait.retime(ds, gait.slices[ds].t0 + want);
                            n_ds_feasible += 1;
                            ds_feasible_total += dd.abs();
                            dcm_plan = DcmPlan::from_footsteps_rolled(
                                &gait,
                                &footsteps,
                                z_lip,
                                zmp_lat_scale,
                                zmp_roll,
                            );
                            if ds_feasible_plan {
                                let seg_ds = dcm_plan.index_at(gait.slices[ds].t0 + 1e-6);
                                if ds_feasible_box_stance {
                                    // Pressure starts where the load is and
                                    // ends where the full DS box allows.
                                    let p_end = ds_box_full.clamp(&(xi_meas * 2.0 - target)).0;
                                    dcm_plan.set_segment_zmp_ramp(seg_ds, p, p_end);
                                } else {
                                    dcm_plan.set_segment_zmp(seg_ds, p);
                                }
                            }
                        }
                        if std::env::var("DS_FEASIBLE_DEBUG").is_ok() {
                            println!("  [dsf] t={t:.3} ds={ds} xi=({:.3},{:+.3}) target=({:.3},{:+.3}) p=({:.3},{:+.3}) need={need:.3} nominal={nominal:.3} -> {want:.3}", xi_meas.x, xi_meas.y, target.x, target.y, p.x, p.y);
                        }
                    }
                }
            }
            if restep && ref_mode.is_none() {
                let ds = gait.index_at(t + 1e-6);
                let is_ds = |j: usize| matches!(gait.slices[j].support, Support::Double);
                if is_ds(ds) && !restep_last.is_some_and(|l| l >= ds) && ds > 0 {
                    let mut first = ds;
                    while first > 0 && is_ds(first - 1) {
                        first -= 1;
                    }
                    let mut last = ds;
                    while last + 1 < gait.slices.len() && is_ds(last + 1) {
                        last += 1;
                    }
                    if let Some(landed) = first.checked_sub(1).and_then(|j| gait.slices[j].support.swing())
                        && last + 1 < gait.slices.len()
                    {
                        let loaded = 1 - landed;
                        let xi_meas = dcm_of(&com, &com_vel, omega);
                        let sole = actual.sole[loaded];
                        // Toward the loaded (stance) side is +y for the left foot.
                        let sgn = if loaded == 0 { 1.0 } else { -1.0 };
                        // Outer edge of the loaded sole's pressure box (ZMP_MARGIN
                        // is a fraction, not metres: go through SupportBox).
                        let sbox1 = SupportBox::from_stance_yawed(
                            &actual,
                            Support::Single { stance: loaded, swing: landed },
                            (sole_half_l * cop_frac, sole_half_w * cop_frac),
                            zmp_margin,
                        );
                        let edge = if loaded == 0 { sbox1.hi.y } else { sbox1.lo.y };
                        let beyond = (xi_meas.y - edge) * sgn;
                        let landed_load = fz_prev[landed];
                        if restep_debug && beyond > -0.02 {
                            println!("  [restep?] t={t:.3} ds run {first}..{last} loaded={loaded} xi.y={:+.3} edge={edge:+.3} beyond={beyond:+.3} landed load={landed_load:.0}", xi_meas.y);
                        }
                        if beyond > restep_margin && landed_load < restep_load_max * total_mass * G {
                            // End the double support now.
                            let t_end = t + 2.0 * dt;
                            let remaining = (gait.slices[last].t1 - t).max(1e-6);
                            let f = ((t_end - t) / remaining).min(1.0);
                            let targets: Vec<(usize, f64)> =
                                (ds..=last).map(|j| (j, t + (gait.slices[j].t1 - t) * f)).collect();
                            for (j, nt) in targets {
                                gait.retime(j, nt);
                            }
                            // Insert the re-step: the landed foot swings again
                            // across to the stance side, then a short DS.
                            let i_ss = gait.insert_after(last, Support::Single { stance: loaded, swing: landed }, restep_t_ss);
                            let i_ds = gait.insert_after(i_ss, Support::Double, restep_t_ds);
                            let cur = *footsteps.at_slice(last);
                            let mut fs = cur;
                            fs.sole[landed] = na::Vector3::new(sole.x + restep_x, sole.y + sgn * restep_cross, sole.z);
                            footsteps.insert_after(last, fs);
                            footsteps.insert_after(i_ss, fs);
                            if restep_ref {
                                // The reference retimes the phases, sets both
                                // landings and re-centres on exit.
                                ref_engage_pending = Some(t);
                                ref_want_landed = true;
                            }
                            // Re-centre every later footfall on the feet as
                            // they will be after the cross, at nominal width.
                            let mid = 0.5 * (fs.sole[0].xy() + fs.sole[1].xy());
                            let half_w = 0.5 * (footsteps.initial.sole[0].y - footsteps.initial.sole[1].y).abs();
                            for side in 0..2 {
                                if restep_ref {
                                    break;
                                }
                                let want = na::Vector2::new(mid.x, mid.y + if side == 0 { half_w } else { -half_w });
                                if let Some(j) = (i_ds + 1..gait.slices.len()).find(|&j| gait.slices[j].support.swing() == Some(side)) {
                                    let d = want - footsteps.at_slice(j).sole[side].xy();
                                    footsteps.shift_from(j, side, d);
                                }
                            }
                            // Shorten the following RESTEP_FOLLOW steps as well
                            // (single support to RESTEP_T_SS2, their double
                            // supports to RESTEP_T_DS).
                            let mut j = i_ds + 1;
                            for _ in 0..(if restep_ref { 0 } else { restep_follow }) {
                                if j >= gait.slices.len() || !matches!(gait.slices[j].support, Support::Single { .. }) {
                                    break;
                                }
                                let t0j = gait.slices[j].t0;
                                gait.retime(j, t0j + restep_t_ss2);
                                let k0 = j + 1;
                                let mut k = k0;
                                while k < gait.slices.len() && matches!(gait.slices[k].support, Support::Double) {
                                    k += 1;
                                }
                                if k > k0 {
                                    let tot = gait.slices[k - 1].t1 - gait.slices[k0].t0;
                                    let f = restep_t_ds / tot.max(1e-6);
                                    let base = gait.slices[k0].t0;
                                    let targets: Vec<(usize, f64)> =
                                        (k0..k).map(|q| (q, base + (gait.slices[q].t1 - base) * f)).collect();
                                    for (q, nt) in targets {
                                        gait.retime(q, nt);
                                    }
                                }
                                j = k;
                            }
                            dcm_plan = DcmPlan::from_footsteps_rolled(&gait, &footsteps, z_lip, zmp_lat_scale, zmp_roll);
                            restep_last = Some(i_ds);
                            n_restep += 1;
                            println!(
                                "  [restep] t={t:.3} foot {landed} re-steps across: DS ends {t_end:.3}, swing [{:.3},{:.3}] to ({:.3},{:+.3}), DS to {:.3}",
                                gait.slices[i_ss].t0, gait.slices[i_ss].t1, fs.sole[landed].x, fs.sole[landed].y, gait.slices[i_ds].t1
                            );
                        }
                    }
                }
            }
            if ds_capture {
                let ds = gait.index_at(t + 1e-6);
                let is_ds = |j: usize| matches!(gait.slices[j].support, Support::Double);
                // Not while a recovery reference owns the phase durations.
                if is_ds(ds) && ref_mode.is_none() && !dsc_retimed_last.is_some_and(|l| l >= ds) {
                    // The whole run of double-support slices the clock is in
                    // (a DS is split into ramp / hold / ramp slices).
                    let mut first = ds;
                    while first > 0 && is_ds(first - 1) {
                        first -= 1;
                    }
                    let mut last = ds;
                    while last + 1 < gait.slices.len() && is_ds(last + 1) {
                        last += 1;
                    }
                    let t0_run = gait.slices[first].t0;
                    let t1_run = gait.slices[last].t1;
                    let remaining = t1_run - t;
                    let xi_meas = dcm_of(&com, &com_vel, omega);
                    let seg_last = dcm_plan.index_at(t1_run - 1e-6);
                    let target = dcm_plan.xi_eos[seg_last];
                    let ds_box = SupportBox::from_stance_yawed(
                        &actual,
                        Support::Double,
                        (sole_half_l * cop_frac, sole_half_w * cop_frac),
                        zmp_margin,
                    );
                    // The pivot that would bring the DCM to the DS-end target
                    // in the remaining time, clamped to the box; the time it
                    // then actually needs along x (DS_FEASIBLE's own test).
                    let dcy = (-omega * remaining.max(1e-3)).exp();
                    let p = ds_box.clamp(&((xi_meas - target * dcy) / (1.0 - dcy))).0;
                    // Degenerate = the DCM is already at the target (or the
                    // pivot sits on it): nothing to do, NOT hopeless. Measured
                    // 2026-10-11 (7@90 m19): a 0.1 mm gap was read as
                    // unreachable and a healthy double support lost 300 ms.
                    let need = match step_timing(&xi_meas, &target, &p, omega, 0) {
                        StepTiming::Time(tt) => tt,
                        StepTiming::Degenerate => 0.0,
                        StepTiming::Unreachable => f64::INFINITY,
                    };
                    let hopeless = remaining > 1e-3
                        && need > remaining * ds_capture_hopeless
                        && (!ds_capture_fwd_only || xi_meas.x > target.x + ds_capture_margin);
                    let prev_sw = first.checked_sub(1).and_then(|j| gait.slices[j].support.swing());
                    let landed_load = prev_sw.map_or(0.0, |sw| fz_prev[sw]);
                    let earliest = (t0_run + ds_capture_t_min).max(t + 2.0 * dt);
                    if ds_capture_debug {
                        println!(
                            "  [dsc-cap?] t={t:.3} run {first}..{last} [{t0_run:.3},{t1_run:.3}] xi.x={:.3} target.x={:.3} p.x={:.3} need={need:.3} rem={remaining:.3} hopeless={hopeless} load={landed_load:.0} earliest={earliest:.3}",
                            xi_meas.x, target.x, p.x
                        );
                    }
                    if hopeless
                        && landed_load >= ds_capture_load * total_mass * G
                        && earliest < t1_run - ds_capture_min_gain
                    {
                        let f = (earliest - t) / remaining;
                        let targets: Vec<(usize, f64)> =
                            (ds..=last).map(|j| (j, t + (gait.slices[j].t1 - t) * f)).collect();
                        for (j, nt) in targets {
                            ds_capture_total += gait.retime(j, nt).abs();
                        }
                        dsc_retimed_last = Some(last);
                        n_ds_capture += 1;
                        dcm_plan = DcmPlan::from_footsteps_rolled(&gait, &footsteps, z_lip, zmp_lat_scale, zmp_roll);
                        if ds_capture_debug {
                            println!("  [dsc-cap] t={t:.3} DS end {t1_run:.3} -> {earliest:.3}");
                        }
                    }
                }
            }
            if !replan_engaged && (com_replan_always || landing_timing.clock.is_some()) {
                // The ZMP command is built from THIS reference, not from `r`
                // below -- measured 2026-09-23: with the blend applied only to
                // `r`, `xi_ref` in the log was smooth but `zmp_cmd` was
                // identical to the digit with and without it. Apply the same
                // continuity guard here, using last tick's state.
                let mut cmd_ref = dcm_plan.reference_preview(t, dcm_preview_horizon);
                if dcm_ref_blend_t > 0.0 {
                    if let Some(prev) = dcm_ref_prev
                        && dcm_ref_blend.is_none()
                        && (cmd_ref.xi - prev).norm() > dcm_ref_blend_min
                    {
                        dcm_ref_blend = Some((prev, cmd_ref.xi, t));
                        dcm_ref_blend_dot = dcm_ref_prev_dot.map(|pd| (pd, cmd_ref.xi_dot));
                    }
                    if let Some((from, plan0, t0)) = dcm_ref_blend {
                        let u = ((t - t0) / dcm_ref_blend_t).clamp(0.0, 1.0);
                        let a = u * u * (3.0 - 2.0 * u);
                        let off = (from - plan0) * (1.0 - a);
                        cmd_ref.xi += off;
                        cmd_ref.p += off;
                        if let Some((fd, pd0)) = dcm_ref_blend_dot {
                            cmd_ref.xi_dot += (fd - pd0) * (1.0 - a);
                        }
                    }
                }
                preview_p = commanded_zmp(
                    &dcm_of(&com, &com_vel, omega),
                    &cmd_ref,
                    omega,
                    k_dcm,
                );
                height_plan = articara::biped::preview::HeightPlan::constant(
                    z_com + terrain_track - terrain_base,
                );
                replan_engaged = true;
            }
            if stair_transfer_enabled
                && stairs.is_some()
                && landing_timing.touchdowns > touchdowns_before
                // A same-height landing must not create another stair Rise.
                && landing_timing.clock.is_some_and(|(i, _, _)| {
                    gait.slices[i].support.swing().is_some_and(|side|
                        (footsteps.at_slice(i).sole[side].z
                            - footsteps.at_slice(i.saturating_sub(1)).sole[side].z).abs() > 0.005)
                })
                && stair_transfer.is_none()
            {
                let swing_slice = landing_timing
                    .clock
                    .expect("touchdown keeps its swing clock")
                    .0;
                let high = gait.slices[swing_slice]
                    .support
                    .swing()
                    .expect("touchdown event belongs to single support");
                let ds_slice = gait.index_at(t);
                if matches!(gait.slices[ds_slice].support, Support::Double) {
                    let low = 1 - high;
                    let toward_low = actual.xy(low) - actual.xy(high);
                    let capture_target = if toward_low.norm_squared() > 1e-12 {
                        actual.xy(high)
                            + toward_low.normalize() * stair_capture_inboard.min(toward_low.norm())
                    } else {
                        actual.xy(high)
                    };
                    let target_z = z_com + actual.sole[high].z - terrain_base;
                    // Where the transfer's DCM reference STARTS.
                    //
                    // `plan.at()` feeds the feedback law as BOTH `xi` and `p`
                    // (see `transfer_ref` below), so `p0` is not only the
                    // first ZMP reference of the Capture phase but also the
                    // first DCM reference. Historically it was the last
                    // commanded ZMP of the single support, i.e. a point
                    // between the feet -- and the walking DCM reference is
                    // not there. Measured 2026-09-23 on G1 (T=40, landing
                    // 3.0 m, 38 armed runs at 20/40 mm): at the arming tick
                    // `xi_ref` jumped from (1.908, +0.071) to (1.998, +0.004),
                    // 9 cm ahead and 7 cm across, while the real DCM stayed
                    // put. `commanded_zmp` then asked for a 34-43 cm ZMP
                    // displacement (clamped at the sole edge) and the error
                    // grew to 0.43 m through a 4 s double-support hold; 59 of
                    // the 67 armed runs fell, against 39 of 125 unarmed ones.
                    // Every one of the 38 showed the same jump to the digit.
                    //
                    // OPT-IN: start the DCM reference from the WALK's own
                    // reference at this instant, so the plan is continuous and
                    // the capture target is approached over `stair_capture_time`
                    // rather than demanded at once. Default keeps the ZMP
                    // point so every existing recipe reproduces.
                    let p0 = if stair_capture_from_xi {
                        dcm_plan.reference(t).xi
                    } else {
                        last_single_p_cmd.unwrap_or_else(|| actual.xy(low))
                    };
                    let plan = articara::biped::preview::StairTransferPlan {
                        start: 0.0,
                        duration: stair_capture_time,
                        p0,
                        p1: capture_target,
                        z0: com.z,
                        z1: com.z,
                    };
                    // Keep both feet scheduled until the measured state says
                    // the transfer is complete.  The deadline records a miss;
                    // it does not release the low foot into an unsafe step.
                    //
                    // The default stretches the double-support slice to the END
                    // OF THE RUN and rebuilds the DCM plan from it. A double
                    // support holds its ZMP, so the rebuilt reference stops
                    // advancing -- measured 2026-09-22 on G1, the DCM reference
                    // went from +0.0311 m/s to +0.0098 m/s at the instant a
                    // transfer armed, while the machine kept walking, and the
                    // real DCM ran 135 mm past it within a second. The sole is
                    // 85 mm half-length, so no ZMP inside the foot can pull
                    // that back: every member that armed a transfer fell, and
                    // every one that did not survived (6 and 10 of 16).
                    //
                    // kyo46rs does not see it. At 0.03 m/s and a 42 mm stride
                    // its walking DCM barely moves, so "hold" and "keep
                    // walking" are nearly the same instruction; at G1's
                    // 0.15-0.18 m/s they are not.
                    //
                    // Two opt-in ways out, both defaulting to the historical
                    // behaviour. The literature on this is about not freezing:
                    // arXiv:2505.13715 makes step timing a decision variable
                    // and switches the foot-timing schedule on tracking error,
                    // and TOPP-MPC (arXiv:1609.04600) treats the timing itself
                    // as the problem. BDI's public writing on Atlas describes
                    // the prediction model but says nothing about contact
                    // scheduling, so it is not evidence either way here.
                    //
                    // `STAIR_TRANSFER_HOLD`: extend the double support by a
                    // FINITE amount rather than to the end of the run, so the
                    // rebuilt plan still walks out of the transfer.
                    let hold_until = if stair_transfer_hold > 0.0 {
                        t + stair_transfer_hold
                    } else {
                        total_t
                    };
                    gait.retime(ds_slice, hold_until.max(t + stair_transfer_timeout + dt));
                    // `STAIR_TRANSFER_KEEP_DCM`: leave the walking DCM plan
                    // alone and let the transfer own only the load split. The
                    // reference then keeps its forward motion and the transfer
                    // stops being a step change in the CoM command.
                    if !stair_transfer_keep_dcm {
                        dcm_plan = DcmPlan::from_footsteps_rolled(
                            &gait,
                            &footsteps,
                            z_lip,
                            zmp_lat_scale,
                            zmp_roll,
                        );
                    }
                    preview_next = t;
                    touchdown_load_ramp = None;
                    let total_now = (measured_pre[0] + measured_pre[1]).max(1e-6);
                    let arm_share = (measured_pre[high] / total_now).clamp(0.0, 1.0);
                    let (phase, plan) = if stair_transfer_rise_first {
                        (
                            StairTransferPhase::Rise,
                            articara::biped::preview::StairTransferPlan {
                                start: 0.0,
                                duration: stair_rise_time,
                                // Continuous with the walk's own reference.
                                p0: dcm_plan.reference(t).xi,
                                p1: capture_target,
                                z0: com.z,
                                z1: target_z,
                            },
                        )
                    } else {
                        (StairTransferPhase::Capture, plan)
                    };
                    stair_transfer = Some(ActiveStairTransfer {
                        ds_slice,
                        high,
                        phase,
                        plan,
                        capture_target,
                        // The walk's DCM reference velocity at this instant.
                        // `StairTransferPlan` interpolates to a capture point
                        // and holds, which stops the reference dead; keeping
                        // this lets `STAIR_TRANSFER_DRIFT` put the walk's own
                        // motion back on top of it.
                        arm_xi_dot: dcm_plan.reference(t).xi_dot,
                        target_z,
                        clock: 0.0,
                        deadline: t + stair_transfer_timeout,
                        ready_ticks: 0,
                        paused_ticks: 0,
                        timed_out: false,
                        hold_started: None,
                        low_held: stair_transfer_rise_first,
                        arm_share,
                        share_ema: arm_share,
                    });
                    stair_transfer_started += 1;
                    println!(
                        "  STAIR TRANSFER start t={t:.3} high={} capture=({:.3},{:.3}) z={:.3}->{:.3} phases={:.2}/{:.2}/{:.2}s",
                        if high == 0 { "L" } else { "R" },
                        capture_target.x,
                        capture_target.y,
                        com.z,
                        target_z,
                        stair_capture_time,
                        stair_rise_time,
                        stair_commit_time,
                    );
                }
            }
        }

        // The transfer has three causal phases. Its internal clock only
        // advances while measured contact and the previous WBC solve are
        // healthy, so an impact spike cannot advance the height trajectory.
        let mut finish_transfer: Option<(usize, usize, f64, u32)> = None;
        if let Some(active) = stair_transfer.as_mut() {
            let actual = Footsteps::from_fk(
                data,
                rig.foot_mi,
                prof.sole_centre_x,
                prof.sole_below_origin,
            );
            let high_support = Support::Single {
                stance: active.high,
                swing: 1 - active.high,
            };
            let high_box = SupportBox::from_stance_yawed(
                &actual,
                high_support,
                (sole_half_l * cop_frac, sole_half_w * cop_frac),
                zmp_margin * stair_capture_margin,
            );
            let xi_now = dcm_of(&com, &com_vel, omega);
            let capture_error = high_box.clamp(&xi_now).1;
            capture_error_log = capture_error;
            let next_support = 1 - active.high;
            // The foot Commit settles onto; see STAIR_COMMIT_ON_HIGH.
            let commit_side = if stair_commit_on_high {
                active.high
            } else {
                next_support
            };
            let next_box = SupportBox::from_stance_yawed(
                &actual,
                Support::Single {
                    stance: commit_side,
                    swing: 1 - commit_side,
                },
                (sole_half_l * cop_frac, sole_half_w * cop_frac),
                zmp_margin * stair_capture_margin,
            );
            let commit_error = next_box.clamp(&xi_now).1;
            let trot_now = misarta::se3::rotation_matrix(&data.oMi[rig.trunk_mi]);
            let transfer_tilt_now = trot_now[(2, 1)]
                .atan2(trot_now[(2, 2)])
                .abs()
                .max((-trot_now[(2, 0)]).asin().abs());
            let signed_pitch = (-trot_now[(2, 0)]).clamp(-1.0, 1.0).asin();
            let pitch_rate = transfer_pitch_previous.map_or(0.0, |old| (signed_pitch - old) / dt);
            transfer_pitch_previous = Some(signed_pitch);
            let total_fz_now = measured_pre[0] + measured_pre[1];
            let contact_pose_ok = |side: usize| {
                let sole = actual.sole[side];
                let surface_z = stairs.as_ref().map_or(0.0, |st| st.h(sole.x));
                let rot = misarta::se3::rotation_matrix(&data.oMi[rig.foot_mi[side]]);
                let sole_tilt = rot[(2, 1)]
                    .atan2(rot[(2, 2)])
                    .abs()
                    .max((-rot[(2, 0)]).asin().abs());
                measured_pre[side] >= stair_recontact_load * total_mass * G
                    && (sole.z - surface_z).abs() <= stair_recontact_z
                    && sole_tilt <= stair_recontact_angle
            };
            let phase_contacts_ok = match active.phase {
                // Capture must be allowed to roll a newly landed sole flat.
                // Rise waits for both rectangular support patches to exist.
                StairTransferPhase::Capture => true,
                StairTransferPhase::Rise => {
                    if touchdown_load_ramp.is_some_and(|ramp| ramp.high == active.high) {
                        contact_pose_ok(next_support)
                    } else {
                        contact_pose_ok(active.high)
                    }
                }
                StairTransferPhase::Commit => {
                    contact_pose_ok(active.high) && contact_pose_ok(next_support)
                }
            };
            let guard_ok = transfer_tilt_now <= stair_transfer_tilt
                && total_fz_now <= stair_transfer_fz_guard * total_mass * G
                && policy.consecutive_degraded() == 0
                && (!stair_hybrid_contact || phase_contacts_ok);
            capture_tilt_log = transfer_tilt_now;
            transfer_guard_log = f64::from(
                u8::from(transfer_tilt_now <= stair_transfer_tilt)
                    + 2 * u8::from(total_fz_now <= stair_transfer_fz_guard * total_mass * G)
                    + 4 * u8::from(policy.consecutive_degraded() == 0)
                    + 8 * u8::from(!stair_hybrid_contact || phase_contacts_ok),
            );
            if guard_ok {
                active.clock += dt;
            } else {
                active.clock += dt * stair_clock_guard_slow;
                active.paused_ticks += 1;
            }
            if active.low_held {
                let total = (measured_pre[0] + measured_pre[1]).max(1e-6);
                let sh = (measured_pre[active.high] / total).clamp(0.0, 1.0);
                let al = (dt / stair_release_tau).clamp(0.0, 1.0);
                active.share_ema += (sh - active.share_ema) * al;
            }
            if active.low_held
                && active.phase == StairTransferPhase::Rise
                && active.clock >= stair_release_min_t
                && active.share_ema >= stair_release_share
            {
                let low = 1 - active.high;
                anchors.release(low);
                swing_lift_off = Some(misarta::se3::translation(&data.oMi[rig.foot_mi[low]]));
                active.low_held = false;
                println!(
                    "  STAIR TRANSFER rise_first release t={t:.3} clock={:.3} high_share_ema={:.2}",
                    active.clock, active.share_ema
                );
            }

            let phase_done = active.clock >= active.plan.duration;
            let phase_ready = match active.phase {
                StairTransferPhase::Capture => {
                    let unload_done = stair_release_time <= 0.0
                        || active.clock >= active.plan.duration + stair_release_time;
                    let release_ok = !stair_release_guard
                        || (measured_pre[1 - active.high] <= 0.05 * total_mass * G
                            && measured_pre[active.high] >= 0.80 * total_mass * G
                            && transfer_tilt_now <= 0.10
                            && pitch_rate.abs() <= 0.30
                            && contact_pose_ok(active.high)
                            && measured_c.cop[active.high][0].abs() <= 0.90 * sole_half_l);
                    phase_done
                        && unload_done
                        && release_ok
                        && capture_error <= stair_capture_tol
                        && guard_ok
                }
                StairTransferPhase::Rise => {
                    let low = 1 - active.high;
                    let touchdown = touchdown_load_ramp.filter(|ramp| ramp.high == active.high);
                    let support_target = touchdown
                        .map_or_else(|| actual.xy(active.high), |ramp| ramp.support_target);
                    // The event target is the convex pressure point implied
                    // by the planned split, so it is inside double support.
                    let support_error = if touchdown.is_some() {
                        0.0
                    } else if active.low_held {
                        // rise_first with the low foot still down: the DCM
                        // only has to be inside the DOUBLE-support box.
                        let ds_box = SupportBox::from_stance_yawed(
                            &actual,
                            Support::Double,
                            (sole_half_l * cop_frac, sole_half_w * cop_frac),
                            zmp_margin,
                        );
                        ds_box.clamp(&xi_now).1
                    } else {
                        capture_error
                    };
                    let load_side = if touchdown.is_some() {
                        low
                    } else {
                        active.high
                    };
                    phase_done
                        && !active.low_held
                        && (!stair_continuous_contact || contact_activation[low] >= 1.0 - 1e-6)
                        && (stair_load_chatter_max <= 0.0
                            || load_chatter_ema <= stair_load_chatter_max)
                        && measured_pre[load_side] >= stair_rise_load * total_mass * G
                        && contact_pose_ok(low)
                        && support_error <= stair_capture_tol
                        && (xi_now - support_target).norm() <= stair_transfer_dcm_tol
                        && (com.z - active.target_z).abs() <= stair_transfer_z_tol
                        && guard_ok
                }
                StairTransferPhase::Commit => {
                    phase_done
                        && measured_pre[commit_side] >= stair_commit_load * total_mass * G
                        && commit_error <= stair_capture_tol
                        && (xi_now.y - actual.xy(commit_side).y).abs() <= stair_transfer_dcm_tol
                        && (com.z - active.target_z).abs() <= stair_transfer_z_tol
                        && guard_ok
                }
            };
            active.ready_ticks = if phase_ready {
                active.ready_ticks + 1
            } else {
                0
            };

            if active.ready_ticks >= stair_transfer_ticks {
                match active.phase {
                    StairTransferPhase::Capture => {
                        active.phase = StairTransferPhase::Rise;
                        touchdown_load_ramp = None;
                        active.plan = articara::biped::preview::StairTransferPlan {
                            start: 0.0,
                            duration: stair_rise_time,
                            p0: active.capture_target,
                            p1: active.capture_target,
                            z0: com.z,
                            z1: active.target_z,
                        };
                        active.clock = 0.0;
                        active.ready_ticks = 0;
                        let low = 1 - active.high;
                        anchors.release(low);
                        swing_lift_off =
                            Some(misarta::se3::translation(&data.oMi[rig.foot_mi[low]]));
                        preview_next = t;
                        println!(
                            "  STAIR TRANSFER capture->rise t={t:.3} dcm_box={capture_error:.4} load={:.2} tilt={transfer_tilt_now:.3}",
                            measured_pre[active.high] / (total_mass * G),
                        );
                    }
                    StairTransferPhase::Rise => {
                        active.phase = StairTransferPhase::Commit;
                        active.plan = articara::biped::preview::StairTransferPlan {
                            start: 0.0,
                            duration: stair_commit_time,
                            // Where the ramp STARTS. The default reads the
                            // capture target, i.e. where Capture asked the CoM
                            // to be. Measured 2026-09-22 on G1 riser 40 mm, it
                            // is not where the CoM IS: Capture's gate checks
                            // that the DCM lies inside the high foot's support
                            // box, which a CoM sitting midway between the feet
                            // satisfies, and Rise then raises z without moving
                            // y. At rise->commit the CoM was at y=-0.003 while
                            // the capture target was y=-0.057, so the ramp
                            // opened with a 54 mm step the body never took.
                            // Committing onto the high foot starts from the
                            // measured CoM instead, so the whole ramp is a move
                            // the body is actually asked to make.
                            p0: if stair_commit_on_high {
                                com.xy()
                            } else {
                                touchdown_load_ramp
                                    .filter(|ramp| ramp.high == active.high)
                                    .map_or(active.capture_target, |ramp| ramp.support_target)
                            },
                            // The lead foot is the next swing foot. Commit
                            // onto the newly landed trailing foot so the lead
                            // foot is unloaded for the following tread.
                            p1: actual.xy(commit_side),
                            z0: active.target_z,
                            z1: active.target_z,
                        };
                        active.clock = 0.0;
                        active.ready_ticks = 0;
                        preview_next = t;
                        println!(
                            "  STAIR TRANSFER rise->commit t={t:.3} zerr={:.4} next_load={:.2} tilt={transfer_tilt_now:.3}",
                            (com.z - active.target_z).abs(),
                            measured_pre[commit_side] / (total_mass * G),
                        );
                    }
                    StairTransferPhase::Commit => {
                        let hold_start = *active.hold_started.get_or_insert_with(|| {
                            if stair_post_commit_hold > 0.0 {
                                println!("  STAIR TRANSFER hold t={t:.3} duration={stair_post_commit_hold:.3}");
                            }
                            t
                        });
                        if t - hold_start >= stair_post_commit_hold {
                            finish_transfer = Some((
                                active.ds_slice,
                                active.high,
                                active.target_z,
                                active.paused_ticks,
                            ));
                        }
                    }
                }
            }

            if !active.timed_out && t >= active.deadline {
                active.timed_out = true;
                stair_transfer_timeouts += 1;
                println!(
                    "  STAIR TRANSFER timeout t={t:.3} phase={:?} load={:.2} dcm_box={capture_error:.4} zerr={:.4} tilt={transfer_tilt_now:.3} pauses={}",
                    active.phase,
                    measured_pre[active.high] / (total_mass * G),
                    (com.z - active.target_z).abs(),
                    active.paused_ticks,
                );
            }
        }
        if let Some((ds_slice, high, target_z, paused_ticks)) = finish_transfer {
            terrain_track = target_z - z_com + terrain_base;
            if stair_height_handover {
                // Commit ends at rest. Resume the ordinary height planner
                // from that same reference, not its pre-Capture segment.
                height_plan =
                    articara::biped::preview::HeightPlan::constant(target_z - stair_post_z);
            }
            // Rise has already executed the following trailing-foot swing;
            // Commit has executed its load transfer. Resume on the next
            // leading-foot swing without changing footstep array indices.
            let consumed = if stair_consume_rise_step
                && stair_hybrid_contact
                && gait
                    .slices
                    .get(ds_slice + 1)
                    .is_some_and(|s| s.support.swing() == Some(1 - high))
                && gait
                    .slices
                    .get(ds_slice + 2)
                    .is_some_and(|s| s.support == Support::Double)
            {
                gait.slices[ds_slice + 1].duration() + gait.slices[ds_slice + 2].duration()
            } else {
                0.0
            };
            let shift = gait.retime(ds_slice, t - consumed);
            if stair_post_walk_terminal_step {
                let resume_i = gait.index_at(t + dt);
                let terminal_side = gait
                    .slices
                    .iter()
                    .skip(resume_i)
                    .find_map(|slice| slice.support.swing())
                    .unwrap_or(1 - high);
                let stance_side = 1 - terminal_side;
                let actual = Footsteps::from_fk(
                    data,
                    rig.foot_mi,
                    prof.sole_centre_x,
                    prof.sole_below_origin,
                );
                let desired_y = actual.sole[stance_side].y
                    + if terminal_side == 0 {
                        stair_post_walk_terminal_gap
                    } else {
                        -stair_post_walk_terminal_gap
                    };
                let dy = desired_y - footsteps.at_slice(resume_i).sole[terminal_side].y;
                footsteps.shift_from(resume_i, terminal_side, na::Vector2::new(0.0, dy));
                println!(
                    "  STAIR TERMINAL STEP side={} y={desired_y:.3} shift={dy:+.3}",
                    if terminal_side == 0 { "L" } else { "R" },
                );
            }
            dcm_plan =
                DcmPlan::from_footsteps_rolled(&gait, &footsteps, z_lip, zmp_lat_scale, zmp_roll);
            preview_next = t;
            stair_transfer_completed += 1;
            first_post_swing_slice = if first_post_swing_fraction < 1.0 && !stair_swing_profile {
                let next = gait.index_at(t + dt);
                gait.slices
                    .iter()
                    .enumerate()
                    .skip(next)
                    .find_map(|(i, slice)| slice.support.swing().map(|side| (i, side)))
                    .and_then(|(i, side)| {
                        let dz = footsteps.at_slice(i).sole[side].z
                            - footsteps.at_slice(i.saturating_sub(1)).sole[side].z;
                        (dz.abs() <= 0.005).then_some(i)
                    })
            } else {
                None
            };
            landing_timing.early_confirm_slice =
                first_post_swing_slice.map(|i| (i, 0.65 * first_post_swing_fraction));
            post_transfer_steps = Some(gait.steps_taken(t.max(1e-9)));
            post_transfer_touchdowns = Some(landing_timing.touchdowns);
            stair_trunk_post_until = t + stair_trunk_post_time;
            println!(
                "  STAIR TRANSFER complete t={t:.3} high={} shift={shift:+.3}s pauses={}",
                if high == 0 { "L" } else { "R" },
                paused_ticks,
            );
            stair_transfer = None;
            touchdown_load_ramp = None;
        }
        if stair_transfer.is_none() {
            stair_capture_share = None;
            stair_capture_p_prev = None;
        }
        let stair_transfer_cmd = stair_transfer
            .as_ref()
            .map(|active| (active.high, active.phase, active.plan.at(active.clock)));

        // ---- contact-driven retiming, before anything reads the phase ---
        if phase_by_contact && !com_replan {
            let shift = contact_phase.update(&mut gait, t, dt, measured_pre);
            if shift != 0.0 {
                retime_total += shift.abs();
                if shift > 0.0 {
                    n_early += 1;
                } else {
                    n_late += 1;
                }
                dcm_plan = DcmPlan::from_footsteps_rolled(
                    &gait,
                    &footsteps,
                    z_lip,
                    zmp_lat_scale,
                    zmp_roll,
                );
            }
        }

        // Check before the schedule is read, including the last DS tick.
        // Keep the original support foot until the receiver has carried
        // load continuously for 50 ms with a flat sole on the surface.
        if stair_resume_contact_gate && stair_transfer_completed > 0 && stair_transfer.is_none() {
            let i = gait.index_at(t);
            if gait.slices[i].support == Support::Double {
                if let Some(Support::Single {
                    stance: receiver, ..
                }) = gait.slices.get(i + 1).map(|s| s.support)
                {
                    if resume_gate_slice != Some(i) {
                        resume_gate_slice = Some(i);
                        resume_gate_confirmed_s = 0.0;
                        resume_gate_wait_started = None;
                    }
                    let actual = Footsteps::from_fk(
                        data,
                        rig.foot_mi,
                        prof.sole_centre_x,
                        prof.sole_below_origin,
                    );
                    let sole = actual.sole[receiver];
                    let surface = stairs.as_ref().map_or(0.0, |st| st.h(sole.x));
                    let rot = misarta::se3::rotation_matrix(&data.oMi[rig.foot_mi[receiver]]);
                    let tilt = rot[(2, 1)]
                        .atan2(rot[(2, 2)])
                        .abs()
                        .max((-rot[(2, 0)]).clamp(-1.0, 1.0).asin().abs());
                    let ready = measured_pre[receiver] >= stair_resume_gate_load * total_mass * G
                        && (sole.z - surface).abs() <= 0.004
                        && tilt <= stair_resume_gate_tilt;
                    resume_gate_confirmed_s = if ready {
                        resume_gate_confirmed_s + dt
                    } else {
                        0.0
                    };
                    if t + dt >= gait.slices[i].t1 - 1e-9
                        && resume_gate_confirmed_s < stair_resume_gate_dwell
                    {
                        let start = *resume_gate_wait_started.get_or_insert_with(|| {
                            println!("  STAIR RESUME waiting t={t:.3} receiver={receiver} load={:.3} zerr={:.4} tilt={tilt:.3}", measured_pre[receiver]/(total_mass*G), sole.z-surface);
                            t
                        });
                        if t - start >= 1.0 {
                            println!("  STAIR RESUME failed: contact timeout t={t:.3}");
                            break;
                        }
                        gait.retime(i, t + 2.0 * dt);
                        dcm_plan = DcmPlan::from_footsteps_rolled(
                            &gait,
                            &footsteps,
                            z_lip,
                            zmp_lat_scale,
                            zmp_roll,
                        );
                        preview_next = t;
                    }
                }
            }
        }

        if stair_post_walk_settle
            && post_walk_settle.is_none()
            && stair_transfer_completed > 0
            && stair_transfer.is_none()
            && post_transfer_steps.map_or(false, |n| {
                gait.steps_taken(t.max(1e-9)) >= n + stair_post_walk_settle_steps
            })
        {
            let i = gait.index_at(t);
            let both_loaded = measured_pre
                .iter()
                .all(|fz| *fz >= stair_post_walk_settle_load * total_mass * G)
                && measured_total >= 0.80 * total_mass * G;
            if gait.slices[i].support == Support::Double && both_loaded {
                post_walk_settle_ready_s += dt;
            } else {
                post_walk_settle_ready_s = 0.0;
            }
            if gait.slices[i].support == Support::Double
                && both_loaded
                && post_walk_settle_ready_s + 1e-12 >= stair_post_walk_settle_dwell
            {
                let actual = Footsteps::from_fk(
                    data,
                    rig.foot_mi,
                    prof.sole_centre_x,
                    prof.sole_below_origin,
                );
                let target = 0.5 * (actual.xy(0) + actual.xy(1));
                post_walk_settle = Some(PostWalkSettle {
                    start: t,
                    start_xi: dcm_of(&com, &com_vel, omega),
                    target,
                    z: com.z,
                });
                gait.retime(i, total_t);
                dcm_plan = DcmPlan::from_footsteps_rolled(
                    &gait,
                    &footsteps,
                    z_lip,
                    zmp_lat_scale,
                    zmp_roll,
                );
                preview_next = t;
                println!(
                    "  STAIR POST SETTLE start t={t:.3} target=({:.3},{:.3}) z={:.3} load={:.2}/{:.2}",
                    target.x,
                    target.y,
                    com.z,
                    measured_pre[0] / (total_mass * G),
                    measured_pre[1] / (total_mass * G)
                );
            }
        }

        if let Some((source_slice, _, _, _)) = resume_ss_dcm_hold {
            let hold_end = t + 2.0 * dt;
            if gait.slices[source_slice].t1 < hold_end - 1e-9 {
                gait.retime(source_slice, hold_end);
                dcm_plan = DcmPlan::from_footsteps_rolled(
                    &gait,
                    &footsteps,
                    z_lip,
                    zmp_lat_scale,
                    zmp_roll,
                );
                preview_next = t;
            }
        }

        // ---- schedule -> contact set -----------------------------------
        // By value, not by reference: `Slice` is `Copy`, and holding a
        // borrow on `gait` here is what stops the step-timing adaptation
        // below from retiming it.
        let (slice, slice_frac) = {
            let (sl, f) = gait.at(t);
            (*sl, f)
        };
        let support = if no_lift || post_walk_settle.is_some() {
            Support::Double
        } else {
            slice.support
        };
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
                            dcm_plan = DcmPlan::from_footsteps_rolled(
                                &gait,
                                &footsteps,
                                z_lip,
                                zmp_lat_scale,
                                zmp_roll,
                            );
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
                    let mut p = misarta::se3::translation(&data.oMi[mi]);
                    // Rotation from the PLAN (flat sole at the planned yaw),
                    // not from the measurement. The anchor's rotation is
                    // deliberately leak-free ("roll cannot be conceded"), so
                    // anchoring a toe-first landing at its measured pitch
                    // HOLDS the foot on its toe -- measured on the descent,
                    // the sole stayed at +27..+47 deg after touchdown, the
                    // robot vaulted over the toe and fell forward. Anchoring
                    // flat makes the Baumgarte roll the heel down instead,
                    // which is what a toe-first landing is FOR.
                    let r = if stair_anchor_flat {
                        foot_rot_at(swing, footsteps.at_slice(slice_idx).yaw[swing])
                    } else {
                        misarta::se3::rotation_matrix(&data.oMi[mi])
                    };
                    // x, y and orientation from the measurement -- but z on
                    // the WALKING SURFACE (the plan's sole height), same as
                    // the contact-driven branch below. Anchoring a foot at
                    // its measured z was fine when landings finished on
                    // time; a stair swing that is still 20 mm up at the
                    // slice boundary was being anchored IN MID-AIR, held
                    // there by Baumgarte, and never loaded -- the double
                    // support then had one real foot, and the next lift-off
                    // fell. Measured: the entire knife-edge sensitivity of
                    // the 4-step recipe was "does this landing finish
                    // before the boundary".
                    let p_meas_z = p.z;
                    p.z = steps.sole[swing].z + prof.sole_below_origin;
                    if flag("DEBUG_ANCHOR", false) {
                        eprintln!(
                            "  [anchor] t={t:.3} swing {} touchdown: meas z {:.4} -> plan z {:.4}",
                            if swing == 0 { "L" } else { "R" },
                            p_meas_z,
                            p.z
                        );
                    }
                    anchors.touchdown(swing, p, r);
                    swing_lift_off = None;
                }
                _ => {}
            }
            prev_support = support;
        }

        let fz_meas = [measured_pre[0], measured_pre[1]];
        if stair_managed_rate > 0.0 {
            for side in 0..2 {
                // EMA over ~4 ticks: one noisy sample must not trip it.
                let r = (fz_meas[side] - fz_prev[side]).abs() / dt.max(1e-9);
                fz_rate_ema[side] += 0.25 * (r - fz_rate_ema[side]);
                if fz_rate_ema[side] > stair_managed_rate {
                    managed_until[side] = t + stair_managed_latch;
                }
            }
        }
        fz_prev2 = fz_prev;
        fz_prev = fz_meas;
        let measured = measured_c;
        let mut stance_sides: Vec<usize> = if contact_driven {
            let (mut sides, mut fresh) = correction.update(support, fz_meas);
            // Planned admission on ANY scheduled touchdown, flat included:
            // entering a double support, a foot the schedule now calls
            // stance whose sole is within CONTACT_ADMIT_Z of its planned
            // height is certified for that double support. Measured
            // 2026-09-24 under the weighted QP: a gentle flat touchdown
            // carried 5 N, stayed under the 10% BW hysteresis for the whole
            // double support, and the QP balanced on the rear foot alone.
            // CONTACT_ADMIT_SCHEDULED_OFF_STAIRS=1: only outside the stair
            // zone (the stair landings have their own certificate path).
            let in_stair_zone = stairs.as_ref().is_some_and(|st| {
                let x = steps.mid_xy().x;
                x >= st.start - stair_crouch_lead && x <= st.landing_start() + stair_crouch_tail
            });
            if contact_admit_scheduled
                && !(contact_admit_scheduled_off_stairs && in_stair_zone)
                && matches!(support, Support::Double)
            {
                let now_slice = gait.index_at(t);
                if let Some(prev) = now_slice.checked_sub(1)
                    && let Some(landed) = gait.slices[prev].support.swing()
                    && confirmed_admit[landed].is_none()
                    && !sides.contains(&landed)
                    && last_scheduled_admit[landed] != Some(now_slice)
                {
                    let mi = rig.foot_mi[landed];
                    let sole_z = misarta::se3::translation(&data.oMi[mi]).z - prof.sole_below_origin;
                    if (sole_z - steps.sole[landed].z).abs() < contact_admit_z {
                        confirmed_admit[landed] = Some(now_slice);
                        last_scheduled_admit[landed] = Some(now_slice);
                    }
                }
            }
            if contact_admit_confirmed || contact_admit_scheduled {
                let now_slice = gait.index_at(t);
                for side in 0..2 {
                    let Some(ds_slice) = confirmed_admit[side] else { continue };
                    if now_slice > ds_slice {
                        // The double support after the landing is over.
                        confirmed_admit[side] = None;
                        continue;
                    }
                    // Only inside that double support (not on the confirm
                    // tick itself, whose schedule may still say "swing").
                    // Re-admit as often as the hysteresis drops the foot:
                    // measured 2026-09-24, a confirmed landing is admitted
                    // by its own 106 N impact, bounces under the 2% BW
                    // release threshold two ticks later, and then rests at
                    // 30 N -- under the 10% BW re-entry -- for the rest of
                    // the double support. The lifting foot at the END of a
                    // later double support is not covered by this: its
                    // certificate expired with its own landing's DS.
                    if now_slice == ds_slice
                        && support.is_stance(side)
                        && !sides.contains(&side)
                        && correction.admit(side)
                    {
                        sides.push(side);
                        sides.sort_unstable();
                        fresh.push(side);
                        n_confirmed_admissions += 1;
                        if std::env::var("CONTACT_ADMIT_DEBUG").is_ok() {
                            let mi = rig.foot_mi[side];
                            let sole_z = misarta::se3::translation(&data.oMi[mi]).z - prof.sole_below_origin;
                            println!(
                                "  [adm] t={t:.3} side={side} sole_z={sole_z:.4} plan_z={:.4} fz=({:.0},{:.0}) support={support:?}",
                                steps.sole[side].z, fz_meas[0], fz_meas[1]
                            );
                        }
                    }
                }
            }
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
                // Plan rotation for the same reason as the scheduled path
                // above: a toe-first contact must not freeze the sole on
                // its toe.
                let rr = if stair_anchor_flat {
                    foot_rot_at(side, steps.yaw[side])
                } else {
                    misarta::se3::rotation_matrix(&data.oMi[mi])
                };
                anchors.touchdown(side, p, rr);
            }
            let sched: Vec<usize> = (0..2).filter(|&s| support.is_stance(s)).collect();
            if sides != sched {
                n_corrected += 1;
            }
            sides
        } else {
            (0..2).filter(|&s| support.is_stance(s)).collect()
        };
        if (stair_transfer_latch
            || (stair_wrench_mpc && stair_managed_contact && !stair_hybrid_contact))
            && stair_transfer_cmd.is_some()
        {
            // Touchdown already passed force, position and height checks.
            // Keep both feet in the contact model until Commit succeeds;
            // otherwise unloading removes the low foot and collapses the
            // support polygon before the controller can reload it.
            stance_sides = vec![0, 1];
        }
        if let Some((high, StairTransferPhase::Rise, rf)) = stair_transfer_cmd
            && stair_hybrid_contact
        {
            let low = 1 - high;
            // Once capture has placed the body over the step, the trailing
            // foot cannot remain on the lower floor through the body rise.
            // Release it, follow the next planned stair swing, and admit it
            // again only after late-swing measured contact.
            let actual = Footsteps::from_fk(
                data,
                rig.foot_mi,
                prof.sole_centre_x,
                prof.sole_below_origin,
            );
            let sole = actual.sole[low];
            let surface_z = stairs.as_ref().map_or(0.0, |st| st.h(sole.x));
            let rotation = misarta::se3::rotation_matrix(&data.oMi[rig.foot_mi[low]]);
            let sole_tilt = rotation[(2, 1)]
                .atan2(rotation[(2, 2)])
                .abs()
                .max((-rotation[(2, 0)]).asin().abs());
            if stair_continuous_contact {
                contact_activation[high] = 1.0;
                if let Some(ramp) = touchdown_load_ramp.filter(|ramp| ramp.high == high) {
                    let x = ((t - ramp.contact_t) / stair_contact_activation_time).clamp(0.0, 1.0);
                    let beta = x * x * (3.0 - 2.0 * x);
                    contact_activation[low] =
                        ramp.start_share[low] + (1.0 - ramp.start_share[low]) * beta;
                } else {
                    contact_activation[low] = 0.0;
                }
            }
            let low_ready = rf.alpha >= stair_recontact_alpha
                && measured_pre[low] >= stair_recontact_load * total_mass * G
                && (sole.z - surface_z).abs() <= stair_recontact_z
                && sole_tilt <= stair_recontact_angle;
            let toe_contact = rf.alpha >= stair_recontact_alpha
                && stance_sides.contains(&low)
                && measured_pre[low] >= correction.on_n;
            let measured_contact_admitted = low_ready || toe_contact;
            if stair_continuous_contact {
                contact_force_activation[high] = 1.0;
                contact_force_activation[low] = if measured_contact_admitted {
                    contact_activation[low]
                } else {
                    0.0
                };
            }
            let continuously_active = stair_continuous_contact
                && touchdown_load_ramp.is_some_and(|ramp| ramp.high == high)
                && contact_activation[low] >= stair_contact_min;
            if measured_contact_admitted || continuously_active {
                // A measured toe/heel contact enters the dynamics immediately
                // with a narrow patch. Keep a flat planned anchor so Managed
                // contact rolls the remaining sole down instead of freezing
                // the measured toe-first attitude.
                let mut anchor = misarta::se3::translation(&data.oMi[rig.foot_mi[low]]);
                anchor.z = surface_z + prof.sole_below_origin;
                anchors.touchdown(low, anchor, foot_rot_at(low, steps.yaw[low]));
                stance_sides = vec![high, low];
                if touchdown_load_ramp.is_none() {
                    let total = measured_pre[0].max(0.0) + measured_pre[1].max(0.0);
                    let low_share = if total > 1e-6 {
                        (measured_pre[low].max(0.0) / total).clamp(0.0, 1.0)
                    } else {
                        0.0
                    };
                    let mut start_share = [0.0; 2];
                    start_share[low] = low_share;
                    start_share[high] = 1.0 - low_share;
                    if stair_continuous_contact {
                        contact_activation[low] = low_share.max(stair_contact_min);
                        contact_force_activation[low] = contact_activation[low];
                    }
                    load_share_previous = low_share;
                    load_chatter_ema = 1.0;
                    touchdown_load_ramp = Some(TouchdownLoadRamp {
                        high,
                        contact_t: t,
                        start_share,
                        support_target: actual.xy(high) * stair_touchdown_high_share
                            + actual.xy(low) * (1.0 - stair_touchdown_high_share),
                        seed_previous: true,
                    });
                    println!(
                        "  STAIR TOUCHDOWN load-ramp t={t:.3} low={} measured={:.2}/{:.2}",
                        if low == 0 { "L" } else { "R" },
                        start_share[low],
                        start_share[high],
                    );
                }
            } else {
                stance_sides = vec![high];
            }
        }
        if let Some((high, StairTransferPhase::Commit, _)) = stair_transfer_cmd
            && stair_hybrid_contact
        {
            // Rise exits only after both soles pass pose and load checks. A
            // touchdown impulse can unload the high foot for a few samples;
            // dropping it here makes WBC single-support on the low foot while
            // the MPC still plans the high foot as primary support.
            stance_sides = vec![high, 1 - high];
        }
        if post_walk_settle.is_some() {
            post_settle_contacts_latched |= stance_sides.len() == 2;
            if post_settle_contacts_latched {
                stance_sides = vec![0, 1];
            }
        }
        // A receiver admitted for the previous hold hands back to the normal
        // scheduled/contact-driven path as soon as that hold releases.
        if resume_ss_dcm_hold.is_none() {
            resume_ss_contact_admitted = None;
        }
        // Break the landing catch-22: the receiver cannot reach the release
        // load while it remains a swing task, but admitting it before it has
        // made physical contact creates an imaginary support. A small,
        // measured force plus the same sole-pose gate arms a compliant
        // contact. From this point it is no longer a swing task.
        if let Some((source_slice, receiver, _, _)) = resume_ss_dcm_hold {
            let actual = Footsteps::from_fk(
                data,
                rig.foot_mi,
                prof.sole_centre_x,
                prof.sole_below_origin,
            );
            let sole = actual.sole[receiver];
            let surface = stairs.as_ref().map_or(0.0, |st| st.h(sole.x));
            let rot = misarta::se3::rotation_matrix(&data.oMi[rig.foot_mi[receiver]]);
            let tilt = rot[(2, 1)]
                .atan2(rot[(2, 2)])
                .abs()
                .max((-rot[(2, 0)]).clamp(-1.0, 1.0).asin().abs());
            if resume_ss_contact_admitted.is_none()
                && measured_pre[receiver] >= stair_resume_ss_admit_load * total_mass * G
                && (sole.z - surface).abs() <= 0.004
                && tilt <= stair_resume_gate_tilt
            {
                let mut anchor = misarta::se3::translation(&data.oMi[rig.foot_mi[receiver]]);
                anchor.z = surface + prof.sole_below_origin;
                anchors.touchdown(receiver, anchor, foot_rot_at(receiver, steps.yaw[receiver]));
                resume_ss_contact_admitted = Some((source_slice, receiver));
                println!(
                    "  STAIR RESUME SS contact t={t:.3} receiver={receiver} load={:.3} zerr={:.4}",
                    measured_pre[receiver] / (total_mass * G),
                    sole.z - surface,
                );
            }
            if resume_ss_contact_admitted == Some((source_slice, receiver))
                && !stance_sides.contains(&receiver)
            {
                stance_sides.push(receiver);
            }
        }
        // Admit a loaded swing foot as a soft contact (SWING_CONTACT_SOFT).
        // Last, so every stair rule above still decides the real contact set.
        // The anchor is rewritten every tick at the foot's own position with z
        // on the plan surface: a soft contact is here to tell the QP a
        // reaction exists, not to pull the foot anywhere, and a stale anchor
        // is exactly what strands a foot in mid-air at kp_c = 1600.
        let mut soft_sides = [false; 2];
        if swing_contact_soft > 0.0 {
            for side in 0..2 {
                if support.swing() == Some(side)
                    && !stance_sides.contains(&side)
                    && fz_meas[side] >= swing_contact_on * total_mass * G
                {
                    let mi = rig.foot_mi[side];
                    let mut p = misarta::se3::translation(&data.oMi[mi]);
                    p.z = steps.sole[side].z + prof.sole_below_origin;
                    anchors.touchdown(side, p, foot_rot_at(side, steps.yaw[side]));
                    stance_sides.push(side);
                    soft_sides[side] = true;
                }
            }
        }
        // Touchdown ramp bookkeeping, after `stance_sides` is final for the
        // tick. A side that leaves the set forgets its clock, so a foot that
        // chatters in and out re-ramps each time rather than latching admitted.
        for side in 0..2 {
            if stance_sides.contains(&side) {
                touchdown_since[side].get_or_insert(t);
            } else {
                touchdown_since[side] = None;
            }
        }
        let stance: Vec<usize> = stance_sides.iter().map(|&s| rig.foot_mi[s]).collect();
        let nc = stance.len();

        // Load ramps across double support. The foot that just landed takes
        // its share on gradually and the foot about to lift gives it up
        // gradually: the CoP box is |m| <= L*fz, so ramping fz is the only
        // way to shrink a foot's authority without a step change in the
        // constraint set.
        let (landing, lifting) = if matches!(support, Support::Double) && !no_lift {
            let i = gait.index_at(t);
            let landed = if i > 0 {
                gait.slices[i - 1].support.swing()
            } else {
                None
            };
            let next = gait.slices.get(i + 1).and_then(|s| s.support.swing());
            (landed, next)
        } else {
            (None, None)
        };
        let ramp_frac = env_f64("LOAD_RAMP_FRAC", 0.6).clamp(0.05, 1.0);
        let f_share = |side: usize| -> f64 {
            if soft_sides[side] {
                // A capacity, not a target: the cone and CoP box shrink with
                // it, so the QP can account for a reaction here without being
                // able to lean on the foot it is still trying to lift.
                return swing_contact_cap;
            }
            if let Some((high, phase, rf)) = stair_transfer_cmd {
                if phase == StairTransferPhase::Capture
                    && stair_capture_low_cap < 1.0
                    && side == 1 - high
                {
                    // Hand the load over on a schedule instead of leaving the
                    // split to the QP. See STAIR_CAPTURE_LOW_CAP above.
                    return (1.0 - (1.0 - stair_capture_low_cap) * rf.alpha).max(0.02);
                }
            }
            if stair_transfer_cmd.is_some() {
                // This is a contact-force CAPACITY, not the desired split.
                // With continuous contact the same estimator that weights the
                // acceleration task also shrinks this foot's wrench cone.
                return if stair_continuous_contact {
                    contact_force_activation[side]
                } else {
                    1.0
                };
            }
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

        let eom_task = dyn_ctx
            .dynamics_task()
            .expect("Explicit keeps the EoM task");
        let base = if eom_hard {
            tasks::box_bound(dyn_ctx.tau(), &rig.torque_max)
        } else {
            eom_task.clone() + tasks::box_bound(dyn_ctx.tau(), &rig.torque_max)
        };
        let base = if jlim {
            // Live actuated-row position/velocity, read through model indices.
            let actuated = rig.actuated();
            let mut q_act = na::DVector::zeros(na_count);
            let mut v_act = na::DVector::zeros(na_count);
            for &(ji, vi) in &actuated {
                if let Some(mi) = rig.a2m[ji] {
                    q_act[vi - 6] = q[rig.model.q_idx[mi]];
                }
                v_act[vi - 6] = v[vi];
            }
            base + bt::joint_limits(
                dyn_ctx.qddot(),
                &actuated,
                &q_act,
                &v_act,
                &jlim_cfg,
                na_count,
                nv,
            )
        } else {
            base
        };
        // --- FFDC: close the loop on the measured load split -------------
        // dz_dot = A * [(f_hi - f_lo)_ref - (f_hi - f_lo)_meas], then the
        // anchors move +-dz/2 so the under-loaded foot is pressed down and
        // the mean foot height is untouched.
        if stair_ffdc > 0.0
            && let Some((high, _, _)) = stair_transfer_cmd
            && let Some(ref_share) = ffdc_ref_share
            && stance_sides.len() == 2
        {
            let low = 1 - high;
            let meas_diff = measured_pre[high].max(0.0) - measured_pre[low].max(0.0);
            let ref_diff = ref_share[high] - ref_share[low];
            let err = ref_diff - meas_diff;
            ffdc_err_log = err;
            ffdc_dz = (ffdc_dz + stair_ffdc * err * dt).clamp(-stair_ffdc_clamp, stair_ffdc_clamp);
            anchors.bias_z[high] = -0.5 * ffdc_dz;
            anchors.bias_z[low] = 0.5 * ffdc_dz;
        } else {
            // Bleed back so a foot that leaves the transfer -- or lands for
            // the next step -- does not inherit a stale offset.
            let decay = (1.0 - stair_ffdc_leak * dt).clamp(0.0, 1.0);
            ffdc_dz *= decay;
            ffdc_err_log = 0.0;
            for b in anchors.bias_z.iter_mut() {
                *b *= decay;
            }
        }
        let mut p0 = bt::contact_level(
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
            &|side: usize| {
                if resume_ss_contact_admitted.is_some_and(|(_, receiver)| receiver == side) {
                    return bt::ContactMode::Managed {
                        tangent_weight: stair_managed_tangent,
                        rotation_weight: stair_managed_rotation,
                        normal_weight: 1.0,
                        normal_accel_max: stair_managed_az_max,
                    };
                }
                // A soft-admitted swing foot: weighted down so the swing task
                // keeps its null space (see SWING_CONTACT_SOFT).
                if soft_sides[side] {
                    return bt::ContactMode::Managed {
                        tangent_weight: stair_managed_tangent * swing_contact_soft,
                        rotation_weight: stair_managed_rotation * swing_contact_soft,
                        normal_weight: swing_contact_soft,
                        normal_accel_max: stair_managed_az_max,
                    };
                }
                // Ramp a fresh contact in regardless of terrain. Ahead of the
                // stair branch because it is the general case; `TOUCHDOWN_RAMP`
                // defaults to 0, which skips this entirely.
                if touchdown_ramp > 0.0 {
                    if let Some(since) = touchdown_since[side] {
                        let u = ((t - since) / touchdown_ramp).clamp(0.0, 1.0);
                        if u < 1.0 {
                            let a = u * u * (3.0 - 2.0 * u);
                            return bt::ContactMode::Managed {
                                tangent_weight: stair_managed_tangent * a,
                                rotation_weight: stair_managed_rotation * a,
                                normal_weight: a,
                                normal_accel_max: stair_managed_az_max,
                            };
                        }
                    }
                }
                let managed_touchdown = stair_transfer_cmd.is_some_and(|(high, phase, rf)| {
                    stair_hybrid_contact
                        && (phase == StairTransferPhase::Commit
                            || (phase == StairTransferPhase::Rise
                                && rf.alpha >= stair_recontact_alpha))
                        && side == 1 - high
                });
                if stair_wrench_mpc
                    && stair_managed_contact
                    && (managed_touchdown
                        || (stair_transfer_cmd.is_some_and(|(_, phase, _)| {
                            phase == StairTransferPhase::Capture || stair_managed_rise
                        }) && measured_pre[side] < stair_managed_fz * total_mass * G)
                        // Rate trigger (see STAIR_MANAGED_RATE): the level
                        // test misses the chatter, which swings fz 15<->78 N.
                        || t < managed_until[side])
                {
                    let activation = if stair_continuous_contact {
                        contact_activation[side]
                    } else {
                        1.0
                    };
                    bt::ContactMode::Managed {
                        tangent_weight: stair_managed_tangent * activation,
                        rotation_weight: stair_managed_rotation * activation,
                        normal_weight: activation,
                        normal_accel_max: stair_managed_az_max,
                    }
                } else {
                    bt::ContactMode::Rigid
                }
            },
            &|_side: usize, rot: &na::Matrix3<f64>| {
                if !stair_roll_patch
                    && !(stair_wrench_mpc && stair_hybrid_contact && stair_transfer_cmd.is_some())
                {
                    return None;
                }
                // World pitch of the sole's forward axis: toe-down positive.
                let s_pitch = -rot[(2, 0)];
                if s_pitch.abs() < 0.087 {
                    return None; // < ~5 deg: treat as flat, full sole
                }
                // Line contact at the leading (toe) or trailing (heel)
                // edge: a narrow CoP strip CENTRED ON THE PATCH, not on the
                // sole. w = 12 mm of effective strip.
                let w = 0.012;
                let x_edge = (sole_half_l - 0.5 * w) * s_pitch.signum();
                Some((
                    [prof.sole_centre_x + x_edge, 0.0, -prof.sole_below_origin],
                    (0.5 * w, contact_cfg.cop_half.1),
                ))
            },
        );
        if ((stair_wrench_mpc && stair_wrench_floor) || stair_capture_floor)
            && stair_transfer_cmd.is_some()
            && nc == 2
        {
            let (high, phase, rf) = stair_transfer_cmd.unwrap();
            let low = 1 - high;
            let capture_low = if stair_capture_low_min >= 0.0 {
                stair_capture_low_min
            } else {
                stair_wrench_low_min
            };
            let low_min_share = match phase {
                StairTransferPhase::Commit => stair_wrench_low_min * (1.0 - rf.alpha),
                StairTransferPhase::Capture => capture_low,
                _ => stair_wrench_low_min,
            };
            let mut force_floor = Task::empty(p0.task.n_decision());
            for (slot, &side) in stance_sides.iter().enumerate() {
                let sole = &p0.sole_sel[slot] * &dyn_ctx.forces().as_affine();
                let mut pick_fz = na::DMatrix::zeros(1, 6);
                pick_fz[(0, 5)] = 1.0;
                let fz = &pick_fz * &sole;
                let min_share = if side == low {
                    low_min_share
                } else {
                    stair_wrench_high_min
                };
                force_floor = force_floor
                    + Task::ge(
                        &fz,
                        &na::DVector::from_element(1, min_share * total_mass * G),
                    );
            }
            p0.task = p0.task.clone() + force_floor;
        }

        // ---- P1: DCM-tracking CoM acceleration -------------------------
        let xi = {
            // DCM_LEAD_S: compensate a measurement delay by propagating the
            // measured DCM forward under the pressure point commanded last
            // tick, xi(t+L) = p + (xi - p) e^{omega L}. 0 = off = shipped.
            let raw = dcm_of(&com, &com_vel, omega);
            match (dcm_lead_s > 0.0, prev_p_cmd) {
                (true, Some(p)) => p + (raw - p) * (omega * dcm_lead_s).exp(),
                _ => raw,
            }
        };
        let mut r = dcm_plan.reference_preview(t, dcm_preview_horizon);
        // REF_WBC=1: while a reference is engaged the DCM law tracks the
        // reference's own DCM and CoP (its CoM and contact forces) instead of
        // the re-planned LIPM.
        if let (Some(mode), Some(lib)) = (ref_mode, ref_lib.as_ref())
            && (ref_wbc || (ref_wbc_landed && lib.landed))
        {
            let tau = t - mode.t0 + mode.tau0;
            if tau <= lib.total() {
                let xi = mode.world_xy(lib, lib.dcm_at(tau, omega));
                let xi1 = mode.world_xy(lib, lib.dcm_at(tau + 0.01, omega));
                r.xi = xi;
                r.xi_dot = (xi1 - xi) / 0.01;
                r.p = mode.world_xy(lib, lib.cop_at(tau));
            }
        }
        // During a stair transfer the MPC's contact schedule falls back to
        // "repeat the current stance" (see the Input construction), so its
        // predicted trajectory is not a usable reference there: measured
        // 2026-09-26, MPC_XI_REF walked the flat approach and fell at the
        // step in 190/192 members. MPC_XI_REF_TRANSFER_PLAN hands the
        // reference back to the plan for the transfer.
        if mpc_xi_ref
            && !(mpc_xi_ref_transfer_plan && stair_transfer_cmd.is_some())
            && let Some((t0, c0, v0, ref traj)) = wrench_mpc_traj
            && !traj.is_empty()
        {
            // Knots: k=0 is the measured state at the solve, k=i+1 is the
            // MPC's prediction at the end of node i. Linear between knots.
            let dtn = stair_wrench_preview_dt;
            let tau = (t - t0 + mpc_xi_ref_lead).min(traj.len() as f64 * dtn);
            if tau >= 0.0 {
                let kf = ((tau / dtn).floor() as usize).min(traj.len() - 1);
                let (ca, va) = if kf == 0 { (c0, v0) } else { (traj[kf - 1][0], traj[kf - 1][1]) };
                let (cb, vb) = (traj[kf][0], traj[kf][1]);
                let a = ((tau - kf as f64 * dtn) / dtn).clamp(0.0, 1.0);
                let xi_a = ca.xy() + va.xy() / omega;
                let xi_b = cb.xy() + vb.xy() / omega;
                r.xi = xi_a * (1.0 - a) + xi_b * a;
                r.xi_dot = (xi_b - xi_a) / dtn;
            }
        }
        let mut resume_ss_hold_active = false;
        if stair_resume_ss_dcm_hold && stair_transfer_completed > 0 && stair_transfer.is_none() {
            if resume_ss_dcm_hold.is_none()
                && resume_ss_hold_finished_slice != Some(slice_idx)
                && post_transfer_touchdowns.is_some_and(|n| landing_timing.touchdowns > n)
                && let Some((clock_i, start, duration)) = landing_timing.clock
                && clock_i == slice_idx
                && let Support::Single { stance, swing } = support
                && t + 1e-9 >= start + duration * stair_resume_ss_hold_frac
            {
                let actual = Footsteps::from_fk(
                    data,
                    rig.foot_mi,
                    prof.sole_centre_x,
                    prof.sole_below_origin,
                );
                let stance_box = SupportBox::from_stance_yawed(
                    &actual,
                    Support::Single { stance, swing },
                    (sole_half_l * cop_frac, sole_half_w * cop_frac),
                    zmp_margin,
                );
                let hold_y = stance_box.clamp(&r.xi).0.y;
                resume_ss_dcm_hold = Some((slice_idx, swing, hold_y, t));
                resume_ss_dcm_ready_s = 0.0;
                let sole = actual.sole[swing];
                let surface = stairs.as_ref().map_or(0.0, |st| st.h(sole.x));
                println!(
                    "  STAIR RESUME SS DCM hold t={t:.3} receiver={} y={hold_y:.3} load={:.3} zerr={:.4}",
                    swing,
                    measured_pre[swing] / (total_mass * G),
                    sole.z - surface,
                );
            }
            if let Some((source_slice, receiver, hold_y, hold_start)) = resume_ss_dcm_hold {
                let actual = Footsteps::from_fk(
                    data,
                    rig.foot_mi,
                    prof.sole_centre_x,
                    prof.sole_below_origin,
                );
                let sole = actual.sole[receiver];
                let surface = stairs.as_ref().map_or(0.0, |st| st.h(sole.x));
                let rot = misarta::se3::rotation_matrix(&data.oMi[rig.foot_mi[receiver]]);
                let tilt = rot[(2, 1)]
                    .atan2(rot[(2, 2)])
                    .abs()
                    .max((-rot[(2, 0)]).clamp(-1.0, 1.0).asin().abs());
                let ready = measured_pre[receiver] >= stair_resume_ss_load * total_mass * G
                    && (sole.z - surface).abs() <= 0.004
                    && tilt <= stair_resume_gate_tilt;
                resume_ss_dcm_ready_s = if ready {
                    resume_ss_dcm_ready_s + dt
                } else {
                    0.0
                };
                if resume_ss_dcm_ready_s + 1e-12 >= stair_resume_gate_dwell {
                    println!(
                        "  STAIR RESUME SS DCM release t={t:.3} receiver={receiver} load={:.3}",
                        measured_pre[receiver] / (total_mass * G),
                    );
                    resume_ss_hold_finished_slice = Some(source_slice);
                    resume_ss_dcm_hold = None;
                    resume_ss_releases += 1;
                    if stair_resume_ds_capture
                        && resume_ss_releases >= stair_resume_ds_capture_after
                    {
                        resume_ds_capture = Some((t, xi.y));
                    }
                } else if t - hold_start >= stair_resume_ss_hold_max {
                    println!(
                        "  STAIR RESUME SS DCM timeout t={t:.3} receiver={receiver} load={:.3}",
                        measured_pre[receiver] / (total_mass * G),
                    );
                    resume_ss_hold_finished_slice = Some(source_slice);
                    resume_ss_dcm_hold = None;
                } else {
                    r.xi.y = hold_y;
                    r.xi_dot.y = 0.0;
                    r.p.y = hold_y;
                    resume_ss_hold_active = true;
                }
            }
        } else {
            resume_ss_dcm_hold = None;
            resume_ss_dcm_ready_s = 0.0;
        }
        let mut resume_ds_capture_active = false;
        if let Some((release_t, release_y)) = resume_ds_capture {
            let elapsed = t - release_t;
            if elapsed >= stair_resume_ds_capture_time {
                resume_ds_capture = None;
            } else if support == Support::Double {
                let u = (elapsed / stair_resume_ds_capture_time).clamp(0.0, 1.0);
                let blend = u * u * (3.0 - 2.0 * u);
                let blend_rate = 6.0 * u * (1.0 - u) / stair_resume_ds_capture_time;
                let nominal_y = r.xi.y;
                r.xi.y = release_y + (nominal_y - release_y) * blend;
                r.xi_dot.y = r.xi_dot.y * blend + (nominal_y - release_y) * blend_rate;
                r.p.y = r.xi.y - r.xi_dot.y / omega;
                resume_ds_capture_active = true;
            }
        }
        if dcm_ref_blend_t > 0.0 {
            if let Some(prev) = dcm_ref_prev
                && dcm_ref_blend.is_none()
                && (r.xi - prev).norm() > dcm_ref_blend_min
            {
                dcm_ref_blend = Some((prev, r.xi, t));
                dcm_ref_blend_dot = dcm_ref_prev_dot.map(|pd| (pd, r.xi_dot));
            }
            if let Some((from, plan0, t0)) = dcm_ref_blend {
                let u = ((t - t0) / dcm_ref_blend_t).clamp(0.0, 1.0);
                let a = u * u * (3.0 - 2.0 * u);
                let off = (from - plan0) * (1.0 - a);
                r.xi += off;
                r.p += off;
                if let Some((fd, pd0)) = dcm_ref_blend_dot {
                    r.xi_dot += (fd - pd0) * (1.0 - a);
                }
                if u >= 1.0 {
                    dcm_ref_blend = None;
                    dcm_ref_blend_dot = None;
                }
            }
            dcm_ref_prev = Some(r.xi);
            dcm_ref_prev_dot = Some(r.xi_dot);
        }
        let dcm_err = (xi - r.xi).norm();
        // Backward-dominant only, and never under an engaged recovery
        // reference (measured: firing on lateral pushes cost the lateral
        // ladder 136 -> 129 while the reference was running).
        let back_err = (xi - r.xi).x;
        push_back_hist.push_back((t, back_err));
        while push_back_hist.len() > 1 && t - push_back_hist.front().unwrap().0 > 0.02 + 1e-9 {
            push_back_hist.pop_front();
        }
        let back_rate = push_back_hist.front().map_or(0.0, |&(tp, bp)| if t - tp >= 0.015 { (back_err - bp) / (t - tp) } else { 0.0 });
        let back_pred = if back_err < -0.6 * push_z_drop_trig { back_err + back_rate.min(0.0) * ref_trig_pred } else { back_err };
        if push_z_drop > 0.0
            && back_pred < -push_z_drop_trig
            && (xi - r.xi).y.abs() < push_z_drop_lat_max
            && ref_mode.is_none()
            && t >= push_z_drop_until
        {
            push_z_drop_until = t + push_z_drop_t;
            n_push_z_drop += 1;
            println!("  [zdrop] t={t:.3} backward DCM error {:+.3} m -> CoM {:.3} m lower for {push_z_drop_t:.2} s", (xi - r.xi).x, push_z_drop);
        }
        // Engage a recovery reference: on request from the push harness
        // (REF_TRIG_AT_PUSH) or when the DCM error toward the STANCE side
        // exceeds REF_TRIG_DCM in single support (the detector).
        if ref_any
            && ref_mode.is_none()
            && t >= ref_cooldown_until
            && slice_frac <= ref_trig_max_frac
            && let Support::Single { stance, swing } = support
        {
            let lateral = (xi - r.xi).y * if stance == 0 { 1.0 } else { -1.0 };
            // REF_TRIG_PRED: fire on the error predicted REF_TRIG_PRED
            // seconds ahead (error + rate * horizon), the rate from a short
            // finite difference. Cuts the detection delay at the cost of
            // reacting to the error's slope.
            // Rate over a 20 ms window (a single-tick difference of the
            // measured DCM is noise: the first version fired on every push
            // direction and took the lateral ladder 136 -> 67).
            ref_lat_hist.push_back((t, lateral));
            while ref_lat_hist.len() > 1 && t - ref_lat_hist.front().unwrap().0 > 0.02 + 1e-9 {
                ref_lat_hist.pop_front();
            }
            let lateral_rate = ref_lat_hist.front().map_or(0.0, |&(tp, lp)| if t - tp >= 0.015 { (lateral - lp) / (t - tp) } else { 0.0 });
            // The raw error must already be most of the way there.
            let lateral_pred = if lateral > 0.6 * ref_trig_dcm { lateral + lateral_rate.max(0.0) * ref_trig_pred } else { lateral };
            if lateral.abs() < 0.005 {
                ref_lat_quiet_t = t;
            }
            let forward = (xi - r.xi).x;
            let pending = ref_engage_pending.is_some_and(|tp| t - tp <= 0.2);
            let lat_fire = ref_trig_dcm > 0.0 && lateral_pred > ref_trig_dcm && ref_libs.iter().any(|l| l.impulse_deg == 90.0);
            let fwd_fire = ref_trig_dcm_x > 0.0 && forward > ref_trig_dcm_x && ref_libs.iter().any(|l| l.impulse_deg == 0.0);
            if pending || lat_fire || fwd_fire {
                let want_deg = if fwd_fire && !lat_fire { 0.0 } else { 90.0 };
                ref_engage_pending = None;
                // The push happened when the lateral error last was quiet;
                // the reference is entered that far in.
                let tau0 = if pending || !ref_delay_comp { 0.0 } else { (t - ref_lat_quiet_t).clamp(0.0, 0.3) };
                println!("  [ref] t={t:.3} trigger: {} (lateral DCM error toward stance {lateral:+.3} m, forward {forward:+.3} m, dir {want_deg:.0}, tau0 {tau0:.3})", if pending { "push" } else if fwd_fire && !lat_fire { "dcm-x" } else { "dcm" });
                let actual = Footsteps::from_fk(data, rig.foot_mi, prof.sole_centre_x, prof.sole_below_origin);
                // Pick the reference meant for the nearest swing phase.
                // (Selecting by the swing foot's height was tried first:
                // on the flat in-place gait the apex comes late, so the
                // height does not order the phases.)
                // Phase AT THE PUSH (the detector fires tau0 later).
                let frac_push = slice_frac - tau0 / gait.slices[slice_idx].duration().max(1e-6);
                let pick = (0..ref_libs.len())
                    .filter(|&k| ref_libs[k].impulse_deg == want_deg && ref_libs[k].landed == ref_want_landed)
                    .min_by(|&a, &b| (ref_libs[a].swing_phase - frac_push).abs().partial_cmp(&(ref_libs[b].swing_phase - frac_push).abs()).unwrap())
                    .unwrap_or(0);
                ref_want_landed = false;
                ref_lib = Some(ref_libs[pick].clone());
                let lib = ref_lib.as_ref().unwrap();
                println!("  [ref] swing phase {slice_frac:.2} -> reference {} (meant for {:.2})", pick, lib.swing_phase);
                let mode = RefMode { t0: t, tau0, stance, origin: actual.sole[stance], mirror: if stance == 0 { 1.0 } else { -1.0 } };
                // Phases: this single support ends after phases[0]; the
                // double support after it (every DS slice up to the next
                // single support) gets phases[1]; the next single support
                // phases[2]; the double support after that phases[3].
                let i0 = slice_idx;
                let mut j = i0;
                let mut ph = 0usize;
                let mut t_cursor = t;
                let mut first_ss_after: Option<usize> = None;
                while j < gait.slices.len() && ph < lib.phases.len() {
                    let is_ss = matches!(gait.slices[j].support, Support::Single { .. });
                    // Count the slices of this phase (DS phases may be split).
                    let mut jend = j + 1;
                    if !is_ss {
                        while jend < gait.slices.len() && matches!(gait.slices[jend].support, Support::Double) {
                            jend += 1;
                        }
                    }
                    let n = (jend - j) as f64;
                    for jj in j..jend {
                        let remaining = if ph == 0 { (lib.phases[0] - tau0).max(0.05) } else { lib.phases[ph] };
                        let end = t_cursor + remaining / n;
                        gait.retime(jj, end);
                        t_cursor = end;
                    }
                    if is_ss && j != i0 && first_ss_after.is_none() {
                        first_ss_after = Some(j);
                    }
                    j = jend;
                    ph += 1;
                }
                // Footfalls: every single-support phase of the reference
                // lands its swing foot (reference right, left, right, ...)
                // where the reference puts it at the end of that phase.
                let mut t_end = 0.0;
                let mut jj = i0;
                let mut ref_sw = 1usize;
                let mut land1 = na::Vector3::zeros();
                let mut land2 = na::Vector3::zeros();
                for (ph, dur) in lib.phases.iter().enumerate() {
                    t_end += dur;
                    if ph % 2 == 0 {
                        // find the single-support slice of this phase
                        while jj < gait.slices.len() && gait.slices[jj].support.swing().is_none() {
                            jj += 1;
                        }
                        if jj >= gait.slices.len() {
                            break;
                        }
                        let foot = mode.foot(ref_sw);
                        let land = mode.world(lib, lib.sole_at(ref_sw, t_end));
                        let d = (land - footsteps.at_slice(jj).sole[foot]).xy();
                        footsteps.shift_next(jj, foot, d);
                        if ph == 0 { land1 = land; } else if ph == 2 { land2 = land; }
                        ref_sw = 1 - ref_sw;
                        jj += 1;
                    }
                }
                let _ = first_ss_after;
                dcm_plan = DcmPlan::from_footsteps_rolled(&gait, &footsteps, z_lip, zmp_lat_scale, zmp_roll);
                landing_timing.event_mode = true;
                ref_mode = Some(mode);
                ref_engagements += 1;
                println!(
                    "  [ref] t={t:.3} engaged: stance={} mirror={:+.0} origin=({:.3},{:.3}) phases={:?} land1=({:.3},{:.3}) land2=({:.3},{:.3}) total={:.3}",
                    stance, mode.mirror, mode.origin.x, mode.origin.y, lib.phases, land1.x, land1.y,
                    land2.x, land2.y, lib.total()
                );
            
            }
        }
        max_dcm_err = max_dcm_err.max(dcm_err);
        // Post-push accumulation. The push fires late in the tick, after this
        // point, so the firing tick's error is still the undisturbed one and
        // the window starts one tick later -- which is what we want.
        if in_push_window {
            win_dcm_err_max = win_dcm_err_max.max(dcm_err);
        }
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
        // ---- variable double support -------------------------------------
        // Ends the CURRENT double support the moment the measured DCM reaches
        // the nominal offset from the foot the next slice commits to.
        if adapt_ds && matches!(support, Support::Double) && dcm_err >= adapt_ds_trig {
            let i = gait.index_at(t);
            if let Some(next) = gait.slices.get(i + 1).copied() {
                if let Support::Single { stance, .. } = next.support {
                    let u = steps.xy(stance);
                    // Inboard is the direction of the OTHER foot, so the sign
                    // of the nominal offset follows the stance side rather
                    // than being hard-coded.
                    let other = steps.xy(1 - stance);
                    let inboard = (other.y - u.y).signum();
                    let b_y = (other.y - u.y).abs() / (1.0 + (omega * gp.t_ss).exp());
                    let off = (xi.y - u.y) * inboard;
                    let t0 = gait.slices[i].t0;
                    let lo = t0 + gp.t_ds * adapt_ds_min;
                    let hi = t0 + gp.t_ds * adapt_ds_max;
                    // The DCM has arrived: commit now. Or it has NOT arrived
                    // and the clock is about to commit anyway: hold the phase
                    // open. Both directions are needed -- the first version
                    // only shortened, and the two runs that failed by
                    // committing EARLY (+23 mm, the DCM not yet close enough)
                    // never triggered it at all.
                    // Open the phase to its maximum and let the arrival
                    // branch close it. Extending by one tick at a time does
                    // not work: `retime(i, t + dt)` puts the boundary exactly
                    // on the next tick, `index_at` then returns the FOLLOWING
                    // slice, and the branch never runs again -- measured as
                    // exactly one retime per run and an unchanged commitment
                    // offset.
                    let new_end = if off <= b_y {
                        Some(t.clamp(lo, hi))
                    } else if gait.slices[i].t1 < hi - 1e-9 {
                        Some(hi)
                    } else {
                        None
                    };
                    if let Some(ne) = new_end
                        && ref_mode.is_none()
                    {
                        if (gait.slices[i].t1 - ne).abs() > 1e-4 {
                            let d = gait.retime(i, ne);
                            n_ds_retime += 1;
                            ds_retime_max = ds_retime_max.max(d.abs());
                            dcm_plan = DcmPlan::from_footsteps_rolled(
                                &gait,
                                &footsteps,
                                z_lip,
                                zmp_lat_scale,
                                zmp_roll,
                            );
                        }
                    }
                }
            }
        }

        // ---- step-timing adaptation --------------------------------------
        // Placed after the DCM reference and before the footstep adaptation,
        // so the two never argue over the same tick: timing reads the plan's
        // own `xi_eos` as its target, which placement would have moved.
        if adapt_time && matches!(support, Support::Single { .. }) && dcm_err >= adapt_time_trig {
            let i = gait.index_at(t);
            let nominal_end = gait.slices[i].t1;
            let target = dcm_plan.eos_at(t);
            let solved = step_timing(&xi, &target, &r.p, omega, adapt_time_axis);
            match solved {
                StepTiming::Time(_) => n_time_solved += 1,
                StepTiming::Unreachable => n_time_wrongside += 1,
                StepTiming::Degenerate => {}
            }
            // The reflex: unreachable means "stop waiting", not "do nothing".
            if adapt_time_cut && solved == StepTiming::Unreachable {
                let slice_t0 = gait.slices[i].t0;
                let new_end = slice_t0 + gp.t_ss * adapt_time_min;
                if nominal_end - new_end > adapt_time_dead {
                    let d = gait.retime(i, new_end);
                    n_time_cut += 1;
                    retime_dcm_total += d.abs();
                    retime_dcm_max = retime_dcm_max.max(d.abs());
                    dcm_plan = DcmPlan::from_footsteps_rolled(
                        &gait,
                        &footsteps,
                        z_lip,
                        zmp_lat_scale,
                        zmp_roll,
                    );
                }
            }
            // The other half of the reflex: unreachable also means "the next
            // footstep should grow the polygon". `swing == 0` is LEFT (URDF:
            // left leg at positive y), so its landing moves further +y and
            // RIGHT's moves further -y -- both away from the centreline.
            if adapt_time_wide && solved == StepTiming::Unreachable && last_wide_slice != Some(i) {
                let swing = support.swing().expect("Support::Single has a swing side");
                let outward = if swing == 0 { 1.0 } else { -1.0 };
                let d = na::Vector2::new(0.0, outward * adapt_time_widen);
                footsteps.shift_from(i, swing, d);
                dcm_plan = DcmPlan::from_footsteps_rolled(
                    &gait,
                    &footsteps,
                    z_lip,
                    zmp_lat_scale,
                    zmp_roll,
                );
                last_wide_slice = Some(i);
                n_time_wide += 1;
            }
            if let StepTiming::Time(tau) = solved {
                let lo = gp.t_ss * adapt_time_min;
                let hi = gp.t_ss * adapt_time_max;
                // Clamp on the SLICE duration, not on tau alone: the solve is
                // "how much longer from now", and a step already 90% done can
                // legitimately want 10 ms while still being a full-length step.
                let slice_t0 = gait.slices[i].t0;
                let new_end = (t + tau).clamp(slice_t0 + lo, slice_t0 + hi);
                if ((t + tau) - new_end).abs() > 1e-9 {
                    n_time_clamped += 1;
                }
                let shift = nominal_end - new_end;
                if shift.abs() > adapt_time_dead {
                    let d = gait.retime(i, new_end);
                    n_retime_dcm += 1;
                    retime_dcm_total += d.abs();
                    retime_dcm_max = retime_dcm_max.max(d.abs());
                    dcm_plan = DcmPlan::from_footsteps_rolled(
                        &gait,
                        &footsteps,
                        z_lip,
                        zmp_lat_scale,
                        zmp_roll,
                    );
                }
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
        let (mut p_cmd, clamped) = if clamp_zmp {
            sbox.clamp(&p_raw)
        } else {
            (p_raw, 0.0)
        };
        max_zmp_clamp = max_zmp_clamp.max(clamped);
        // Pressure the polygon refused (world x, y), for MOM_CMP.
        let zmp_deficit = p_raw - p_cmd;

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
        if contact_plan_on
            && ref_mode.is_none()
            && let Support::Single { swing, .. } = support
            && dcm_err >= contact_plan_trig
        {
            let i = gait.index_at(t);
            if cp_slice != Some(i) {
                cp_slice = Some(i);
                cp_applied = na::Vector2::zeros();
            }
            let seg = &dcm_plan.segs[dcm_plan.index_at(t)];
            let sit = articara::biped::contact_plan::Situation {
                xi,
                p: seg.at(t),
                p_dot: seg.slope(),
                omega,
                t,
                nominal_end: gait.slices[i].t1,
                xi_eos_plan: dcm_plan.eos_at(t),
                swing_sign: if swing == 0 { 1.0 } else { -1.0 },
            };
            // How far the plan's end-of-step DCM moves when this landing
            // moves: the planner divides the residual by it, so a wrong
            // value scales every correction. The shipped 0.5 ("the DCM sits
            // midway between the feet") measures 0.646 against the real
            // backward recursion with this gait, i.e. 29% too small. With
            // CP_J_AUTO it is differenced from the plan itself instead.
            let mut cfg_now = contact_plan_cfg;
            if cp_j_auto {
                let du = 0.02;
                let mut probe = footsteps.clone();
                probe.shift_from(i, swing, na::Vector2::new(du, 0.0));
                let moved = DcmPlan::from_footsteps_rolled(
                    &gait,
                    &probe,
                    z_lip,
                    zmp_lat_scale,
                    zmp_roll,
                );
                let j = (moved.xi_eos[dcm_plan.index_at(t)].x - sit.xi_eos_plan.x) / du;
                if j.is_finite() && j > 0.05 {
                    cfg_now.j = j;
                }
            }
            // CP_TOUCHDOWN: put the foot where the DCM will BE when it
            // lands, instead of chasing the plan's own end-of-step value
            // through a sensitivity (contact_plan::plan_touchdown). The
            // landing's real channel is that the foot becomes the pressure
            // point, and the double support that follows is where that first
            // acts -- 3.5x the authority, with the sign the physics asks for.
            let dec = if cp_touchdown && i + 1 < gait.slices.len() {
                let ds = &gait.slices[i + 1];
                let seg_next = dcm_plan.index_at(gait.slices[i].t1 + ds.duration() + 1e-6);
                let nxt = footsteps.at_slice(i);
                let u_nom = dcm_plan.segs[seg_next].p0;
                let td = articara::biped::contact_plan::Touchdown {
                    ds_duration: ds.duration(),
                    // The next single support's pressure enters at p0 and
                    // leaves at p1; the landing "is" the midpoint of those.
                    entry_offset: dcm_plan.segs[seg_next].p0
                        - 0.5 * (dcm_plan.segs[seg_next].p0 + dcm_plan.segs[seg_next].p1),
                    u_nom: 0.5 * (dcm_plan.segs[seg_next].p0 + dcm_plan.segs[seg_next].p1),
                    // Where the plan wants the DCM when that support starts,
                    // relative to the landing.
                    b_nom: dcm_plan.reference(gait.slices[i].t1 + ds.duration() + 1e-6).xi
                        - 0.5 * (dcm_plan.segs[seg_next].p0 + dcm_plan.segs[seg_next].p1),
                };
                let _ = (nxt, u_nom);
                articara::biped::contact_plan::plan_touchdown(&sit, &td, &cfg_now)
            } else {
                articara::biped::contact_plan::plan(&sit, &cfg_now)
            };
            if std::env::var("CP_DEBUG").is_ok() {
                println!(
                    "  [cp] t={t:.3} slice={i} err={dcm_err:.4} xi=({:.3},{:+.3}) eos_plan=({:.3},{:+.3}) pred_nom=({:.3},{:+.3}) shift=({:+.3},{:+.3}) dT={:+.3} unmet={:.4} cost={:.3}",
                    xi.x, xi.y, sit.xi_eos_plan.x, sit.xi_eos_plan.y,
                    articara::biped::contact_plan::predict(&sit, (sit.nominal_end - t).max(0.0)).x,
                    articara::biped::contact_plan::predict(&sit, (sit.nominal_end - t).max(0.0)).y,
                    dec.shift.x, dec.shift.y, dec.new_end.map_or(0.0, |e| e - sit.nominal_end), dec.unmet, dec.cost
                );
            }
            // CP_TREAD_BOX: keep the whole sole on the surface it lands on.
            //
            // MEASURED HARMFUL 2026-09-25. Kept off, and kept here, so the
            // next attempt does not take this shape again.
            //
            // The premise looked sound: a 0.211 m tread minus a 0.170 m sole
            // leaves the sole centre a 41 mm window, the adjuster has +-60 mm
            // of authority in x, and per-cycle adjustment did land 16/24
            // fully on the tread against 20/24 throttled.
            //
            // But the premise was never checked, and it is false. Landing
            // fully on the tread DOES NOT PREDICT SURVIVAL (n=32 at the 60 mm
            // riser): on-tread 18 survived / 5 fell, off-tread 8 survived /
            // 1 fell. A sole hanging over the nosing is carried by the
            // contact it does have. Snapping to the window edge then makes
            // things worse -- it commands the landing AT a boundary, where
            // any swing tracking error puts it outside -- and survival fell
            // 26/32 -> 17/32.
            //
            // A footstep optimisation still wants terrain awareness, but as a
            // cost with margin, not as a hard snap to an edge.
            let mut dec = dec;
            if cp_tread_box
                && let Some(st) = stairs.as_ref()
            {
                // Against the surface the ADJUSTED landing lands on, not the
                // one the nominal is over: clamping to the latter pins a foot
                // reaching for the tread behind the riser and forbids the
                // climb outright (measured 2026-09-25: on-tread landings went
                // 16/24 -> 11/24, worse than no constraint at all).
                let nominal = footsteps.at_slice(i).sole[swing].x;
                let (snapped, _) =
                    st.snap_to_sole_window(nominal + dec.shift.x, sole_half_l);
                dec.shift.x = snapped - nominal;
            }
            let mut rebuilt = false;
            // Like ADAPT_STEP: place the foot once per step, keeping the
            // largest demand seen, instead of re-placing it every tick.
            //
            // That is a RATCHET: the landing only ever moves further from
            // nominal. With the shipped 15 mm trigger it fires a handful of
            // times in an emergency and the ratchet is harmless, but run
            // continuously it walks the foot out to its bound and the stair
            // approach dies (measured 2026-09-24 on the randomised stairs:
            // 0/192 at every shift weight, falling at 21-23 s). CP_TRACK
            // makes the landing FOLLOW the current decision instead, which
            // is what a step adjustment recomputed every cycle has to do.
            let d = if cp_track {
                dec.shift - cp_applied
            } else if dec.shift.norm() > cp_applied.norm() {
                dec.shift - cp_applied
            } else {
                na::Vector2::zeros()
            };
            if d.norm() > 1e-9 && !landing_timing.landing_closed(i) {
                // CP_SHIFT_NEXT: move THIS landing, not every later one.
                //
                // `shift_from` shifts the swing foot's landing in slice `i`
                // AND every later slice, so the adjusted position becomes the
                // nominal that the NEXT step's residual is measured against.
                // The per-step bound (`max_y` = 30 mm) is then a bound on the
                // increment, not on the total, and the foot walks outward one
                // bound per step: measured 2026-09-25 under continuous firing
                // the lateral spacing went 127 -> 153 mm median, 180 mm peak,
                // against a 127 mm nominal and a 30 mm limit. That drift --
                // not the reference discontinuity the earlier design note
                // assumed (measured: the reference rate of change grows only
                // 0.104 -> 0.139 m/s, and its peak FALLS) -- is what a step
                // adjustment recomputed every cycle actually does wrong.
                if cp_shift_next {
                    footsteps.shift_next(i, swing, d);
                } else {
                    footsteps.shift_from(i, swing, d);
                }
                cp_applied = dec.shift;
                n_cp_shift += 1;
                rebuilt = true;
            }
            if let Some(ne) = dec.new_end
                && ne > t + dt
                && cp_retimed_slice != Some(i)
            {
                let dd = gait.retime(i, ne);
                n_cp_retime += 1;
                cp_retime_total += dd.abs();
                cp_retimed_slice = Some(i);
                // The planner has now set this step's end; the landing
                // extension must not move it again (the two would fight
                // over the same boundary, and extension IS mechanism B).
                landing_timing.max_extension = 0.0;
                rebuilt = true;
            }
            if rebuilt {
                dcm_plan = DcmPlan::from_footsteps_rolled(
                    &gait,
                    &footsteps,
                    z_lip,
                    zmp_lat_scale,
                    zmp_roll,
                );
                n_replan += 1;
            }
        }
        if !contact_plan_on && adapt && matches!(support, Support::Single { .. }) && clamped > 1e-6 {
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
        if adapt_live && !contact_plan_on {
            if let Support::Single { swing, .. } = support {
                let d = adapt_now - adapt_applied;
                if d.norm() > 1e-9 {
                    footsteps.shift_from(slice_idx, swing, d);
                    dcm_plan = DcmPlan::from_footsteps_rolled(
                        &gait,
                        &footsteps,
                        z_lip,
                        zmp_lat_scale,
                        zmp_roll,
                    );
                    adapt_applied = adapt_now;
                    n_replan += 1;
                }
            }
        }
        if replan_engaged {
            let actual = Footsteps::from_fk(
                data,
                rig.foot_mi,
                prof.sole_centre_x,
                prof.sole_below_origin,
            );
            let measured_support = if stance_sides.len() == 2 {
                Support::Double
            } else {
                let side = stance_sides[0];
                Support::Single {
                    stance: side,
                    swing: 1 - side,
                }
            };
            let current_box = SupportBox::from_stance_yawed(
                &actual,
                measured_support,
                (sole_half_l * cop_frac, sole_half_w * cop_frac),
                zmp_margin,
            );
            if t + 1e-9 >= preview_next {
                let mut samples = Vec::with_capacity(com_preview_n);
                for k in 0..com_preview_n {
                    let begin = t + k as f64 * com_preview_dt;
                    let end = begin + com_preview_dt;
                    let f0 = articara::biped::preview::forecast_feet(
                        &gait,
                        &footsteps,
                        &actual,
                        &stance_sides,
                        t,
                        begin,
                    );
                    let f1 = articara::biped::preview::forecast_feet(
                        &gait,
                        &footsteps,
                        &actual,
                        &stance_sides,
                        t,
                        end - 1e-9,
                    );
                    let b0 = SupportBox::from_stance_yawed(
                        &f0,
                        gait.support_at(begin),
                        (sole_half_l * cop_frac, sole_half_w * cop_frac),
                        zmp_margin,
                    );
                    let b1 = SupportBox::from_stance_yawed(
                        &f1,
                        gait.support_at(end - 1e-9),
                        (sole_half_l * cop_frac, sole_half_w * cop_frac),
                        zmp_margin,
                    );
                    let mut bounds = SupportBox {
                        lo: b0.lo.sup(&b1.lo),
                        hi: b0.hi.inf(&b1.hi),
                    };
                    if k == 0 {
                        bounds = SupportBox {
                            lo: current_box.lo,
                            hi: current_box.hi,
                        };
                    }
                    let (zmp_ref, xi_ref) = if let Some(active) = stair_transfer.as_ref() {
                        // Receding-horizon transfer: the common contact clock
                        // supplies the moving capture target, while the QP
                        // chooses a feasible ZMP from the measured state.
                        // Same drift as the feedback path below: the capture
                        // plan holds a point, the walk does not. This is the
                        // sample stream the CoM preview QP optimises against,
                        // so leaving it un-drifted pins the whole horizon --
                        // measured, the logged `xi_ref_x` advanced at
                        // +0.0098 m/s during a transfer against +0.0542 m/s
                        // without one, and correcting only the feedback law
                        // left that number unchanged.
                        let d0 = active.arm_xi_dot
                            * (stair_transfer_drift
                                * (active.clock + (begin - t) + 0.5 * com_preview_dt));
                        let d1 =
                            active.arm_xi_dot * (stair_transfer_drift * (active.clock + (end - t)));
                        (
                            active
                                .plan
                                .at(active.clock + (begin - t) + 0.5 * com_preview_dt)
                                .p
                                + d0,
                            active.plan.at(active.clock + (end - t)).p + d1,
                        )
                    } else {
                        // Continuity guard (DCM_REF_BLEND_T), applied to the
                        // sample stream the CoM preview QP actually optimises
                        // against -- measured 2026-09-23: blending `r` and
                        // `preview_p` alone left `zmp_cmd` identical to the
                        // digit, because THIS is the command path.
                        let off_at = |s_at: f64| -> na::Vector2<f64> {
                            match dcm_ref_blend {
                                Some((from, plan0, t0)) if dcm_ref_blend_t > 0.0 => {
                                    let u = ((s_at - t0) / dcm_ref_blend_t).clamp(0.0, 1.0);
                                    (from - plan0) * (1.0 - u * u * (3.0 - 2.0 * u))
                                }
                                _ => na::Vector2::zeros(),
                            }
                        };
                        (
                            dcm_plan
                                .reference_preview(
                                    begin + 0.5 * com_preview_dt,
                                    dcm_preview_horizon,
                                )
                                .p
                                + off_at(begin + 0.5 * com_preview_dt),
                            dcm_plan.reference_preview(end, dcm_preview_horizon).xi + off_at(end),
                        )
                    };
                    samples.push(articara::biped::preview::PreviewSample {
                        duration: com_preview_dt,
                        bounds,
                        zmp: zmp_ref,
                        xi: xi_ref,
                    });
                }
                if dcm_ref_blend_t > 0.0 && std::env::var("DCM_REF_BLEND_DEBUG").is_ok() {
                    let (ba, bf, bp, bt) = match dcm_ref_blend {
                        Some((f, p0, t0)) => (1, f, p0, t0),
                        None => (0, na::Vector2::zeros(), na::Vector2::zeros(), 0.0),
                    };
                    println!(
                        "  [blend] t={t:.3} active={ba} from=({:.3},{:+.3}) plan0=({:.3},{:+.3}) t0={bt:.3} s0.xi=({:.3},{:+.3}) s0.zmp=({:.3},{:+.3}) preview_p=({:.3},{:+.3}) p_cmd=({:.3},{:+.3})",
                        bf.x, bf.y, bp.x, bp.y, samples[0].xi.x, samples[0].xi.y, samples[0].zmp.x, samples[0].zmp.y,
                        preview_p.x, preview_p.y, p_cmd.x, p_cmd.y
                    );
                }
                let preview_solve_start = std::time::Instant::now();
                let preview_solution = articara::biped::preview::replan(
                    omega,
                    com.xy(),
                    com_vel.xy(),
                    preview_p,
                    &samples,
                    com_preview_smooth,
                );
                let preview_solve_ns = preview_solve_start.elapsed().as_nanos();
                let preview_accepted = preview_solution.is_some();
                match preview_solution {
                    Some(plan) => {
                        preview_p = plan.zmp[0];
                        preview_c = plan.com[0];
                        preview_v = plan.velocity[0];
                        preview_error = plan.terminal_error;
                        preview_count += 1;
                        // Keep the whole horizontal trajectory, not just its
                        // first node: the wrench MPC needs a TARGET, and the
                        // only horizontal plan in the stack is this one. See
                        // `mpc_target_preview` at the MPC input.
                        preview_traj = Some((t, plan.com.clone(), plan.velocity.clone()));
                    }
                    None => {
                        preview_p = current_box.clamp(&p_cmd).0;
                        preview_failures += 1;
                    }
                }
                if let Some(writer) = &mut com_preview_runtime {
                    writeln!(writer, "{t:.6},{preview_solve_ns},{},{},{:.9},{},{}",
                        u8::from(preview_solve_ns > 50_000_000),
                        u8::from(preview_accepted),
                        preview_error,
                        u8::from(com_replan_lateral),
                        com_preview_n,
                    ).unwrap();
                }
                preview_next = if com_preview_phase_lock {
                    (preview_next + com_replan_dt).max(t + dt)
                } else {
                    t + com_replan_dt
                };
            }
            let optimized = current_box.clamp(&preview_p).0;
            p_cmd.x += com_replan_forward_gain * (optimized.x - p_cmd.x);
            if com_replan_lateral {
                p_cmd.y += com_replan_lateral_gain * (optimized.y - p_cmd.y);
            }
        }
        if resume_ss_hold_active || resume_ds_capture_active {
            let actual = Footsteps::from_fk(
                data,
                rig.foot_mi,
                prof.sole_centre_x,
                prof.sole_below_origin,
            );
            let measured_support = if stance_sides.len() == 2 {
                Support::Double
            } else {
                let stance = stance_sides[0];
                Support::Single {
                    stance,
                    swing: 1 - stance,
                }
            };
            let hold_box = SupportBox::from_stance_yawed(
                &actual,
                measured_support,
                (sole_half_l * cop_frac, sole_half_w * cop_frac),
                zmp_margin,
            );
            p_cmd.y = hold_box.clamp(&commanded_zmp(&xi, &r, omega, k_dcm)).0.y;
        }
        if let Some((_high, _phase, _rf)) = stair_transfer_cmd {
            // The transfer plan was created from measured foot positions at
            // touchdown.  Clamp once more against the measured support box,
            // then use it for both DCM acceleration and force allocation.
            let actual = Footsteps::from_fk(
                data,
                rig.foot_mi,
                prof.sole_centre_x,
                prof.sole_below_origin,
            );
            let measured_support = if stance_sides.len() == 2 {
                Support::Double
            } else {
                let side = stance_sides[0];
                Support::Single {
                    stance: side,
                    swing: 1 - side,
                }
            };
            let transfer_box = SupportBox::from_stance_yawed(
                &actual,
                measured_support,
                (sole_half_l * cop_frac, sole_half_w * cop_frac),
                zmp_margin,
            );
            p_cmd = if let Some(active) = stair_transfer.as_ref() {
                // Use one feedback law across Capture, Rise and Commit.
                // Switching back to the generic preview at Capture->Rise
                // produced a measured 60 mm ZMP command jump.
                let rf = active.plan.at(active.clock);
                let rf_next = active.plan.at(active.clock + ctrl_dt);
                let event_p = |base: na::Vector2<f64>, at: f64| {
                    if active.phase == StairTransferPhase::Rise
                        && let Some(ramp) =
                            touchdown_load_ramp.filter(|ramp| ramp.high == active.high)
                    {
                        let x = ((at - ramp.contact_t) / stair_touchdown_load_time).clamp(0.0, 1.0);
                        let beta = x * x * (3.0 - 2.0 * x);
                        active.capture_target + (ramp.support_target - active.capture_target) * beta
                    } else {
                        base
                    }
                };
                let mut p_ref = event_p(rf.p, t);
                let mut p_ref_next = event_p(rf_next.p, t + ctrl_dt);
                if stair_capture_load_track && active.phase == StairTransferPhase::Capture {
                    let total = (measured_pre[0] + measured_pre[1]).max(1e-6);
                    let s_meas = (measured_pre[active.high] / total).clamp(0.0, 1.0);
                    let a = (ctrl_dt / stair_capture_share_tau).clamp(0.0, 1.0);
                    let sh = match stair_capture_share {
                        Some(prev) => prev + (s_meas - prev) * a,
                        None => s_meas,
                    };
                    stair_capture_share = Some(sh);
                    let p_low = actual.xy(1 - active.high);
                    // Driven ramp from the share at arming toward 0.5 over the
                    // capture time, limited to `lead` ahead of the measurement.
                    let u = (active.clock / stair_capture_time).clamp(0.0, 1.0);
                    let mut ramp = sh.min(0.5) + (0.5 - sh.min(0.5)) * (u * u * (3.0 - 2.0 * u));
                    // Past the capture time the historical release moves the
                    // load to 0.95 on the high foot (Capture->Rise needs low
                    // <= 0.05 mg and high >= 0.80 mg). The reference must go
                    // there too: capped at 0.5 it contradicted the load
                    // schedule and Capture timed out at 4 s in every armed run
                    // (measured 2026-09-24, s_ref pinned at 0.500 while the
                    // measured share reached 1.0).
                    if stair_release_time > 0.0 {
                        let w = ((active.clock - stair_capture_time) / stair_release_time).clamp(0.0, 1.0);
                        ramp += (0.95 - ramp) * (w * w * (3.0 - 2.0 * w));
                    }
                    let s_ref = ramp.max(sh).min(sh + stair_capture_lead).clamp(0.0, 1.0);
                    let tracked = p_low + (active.capture_target - p_low) * s_ref;
                    if std::env::var("STAIR_CAPTURE_DEBUG").is_ok() {
                        println!("  [cap] t={t:.3} clock={:.3} s_meas={s_meas:.3} sh={sh:.3} ramp={ramp:.3} s_ref={s_ref:.3} lead={stair_capture_lead:.2}", active.clock);
                    }
                    p_ref_next = tracked + (tracked - stair_capture_p_prev.unwrap_or(tracked));
                    p_ref = tracked;
                    stair_capture_p_prev = Some(tracked);
                }
                // Carry the walk's own DCM drift through the transfer.
                //
                // `plan.at()` interpolates from where the CoM was at arming to
                // a capture target and then holds, so `xi_dot` decays to zero
                // while the machine is still walking forward. `commanded_zmp`
                // uses only `xi` and `xi_dot`, so the reference simply stops.
                //
                // Measured 2026-09-22 on G1 (20 mm riser, CEM15): without a
                // transfer the DCM reference advances at +0.0542 m/s; with one
                // it advances at +0.0098 m/s, a factor of 5.5. The real DCM
                // keeps going, so the error runs 0.0006 -> 0.0122 -> 0.1143 ->
                // 0.1346 m within a second -- past the 85 mm sole half-length,
                // i.e. beyond what any ZMP inside the foot can pull back. Every
                // member that armed a transfer fell; every member that did not
                // survived (6 and 10 of 16).
                //
                // kyo46rs does not see this: at 0.03 m/s and a 42 mm stride its
                // walking DCM barely moves, so "hold the capture point" and
                // "keep walking" are nearly the same instruction. At G1's
                // 0.15-0.18 m/s they are not.
                //
                // OPT-IN via `STAIR_TRANSFER_DRIFT` (0 = the historical
                // behaviour, so every existing recipe is unchanged). The drift
                // is the DCM reference velocity the walk had when the transfer
                // armed, applied for the transfer's own elapsed time.
                let drift = if stair_transfer_drift > 0.0 {
                    active.arm_xi_dot * (stair_transfer_drift * active.clock)
                } else {
                    na::Vector2::zeros()
                };
                let transfer_ref = articara::biped::dcm::DcmRef {
                    xi: p_ref + drift,
                    xi_dot: (p_ref_next - p_ref) / ctrl_dt
                        + active.arm_xi_dot * stair_transfer_drift,
                    p: p_ref + drift,
                };
                let transfer_p = commanded_zmp(
                    &dcm_of(&com, &com_vel, omega),
                    &transfer_ref,
                    omega,
                    stair_capture_k,
                );
                transfer_box.clamp(&transfer_p).0
            } else {
                transfer_box.clamp(&preview_p).0
            };
        }
        if let Some(settle) = post_walk_settle {
            let u = ((t - settle.start - stair_post_walk_settle_capture_hold)
                / stair_post_walk_settle_ramp)
                .clamp(0.0, 1.0);
            let alpha = u * u * (3.0 - 2.0 * u);
            let alpha_dot = 6.0 * u * (1.0 - u) / stair_post_walk_settle_ramp;
            let xi_ref = settle.start_xi + (settle.target - settle.start_xi) * alpha;
            let xi_dot = (settle.target - settle.start_xi) * alpha_dot;
            let settle_ref = articara::biped::dcm::DcmRef {
                xi: xi_ref,
                xi_dot,
                p: xi_ref - xi_dot / omega,
            };
            let actual = Footsteps::from_fk(
                data,
                rig.foot_mi,
                prof.sole_centre_x,
                prof.sole_below_origin,
            );
            let settle_support = if stance_sides.len() == 2 {
                Support::Double
            } else {
                let side = stance_sides[0];
                Support::Single {
                    stance: side,
                    swing: 1 - side,
                }
            };
            let settle_box = SupportBox::from_stance_yawed(
                &actual,
                settle_support,
                (sole_half_l * cop_frac, sole_half_w * cop_frac),
                zmp_margin,
            );
            p_cmd = settle_box
                .clamp(&commanded_zmp(
                    &dcm_of(&com, &com_vel, omega),
                    &settle_ref,
                    omega,
                    k_dcm,
                ))
                .0;
        }
        // MPC_PCMD: the WBC's commanded pressure point comes from the MPC
        // (its node-0 net CoP from the latest solve, clamped to the same
        // support box as the DCM law) instead of the WBC's own DCM law -- the
        // MPC then decides where the pressure is in single support too, and
        // the WBC's CoM task realises it through c_ddot = w^2 (c - p). The
        // value is blended in by this fraction, after every other branch.
        // Not during a stair transfer or the post-walk settle (their own
        // laws). Only a solve younger than 2 MPC periods is used; otherwise
        // the DCM law stands. 0 = off = shipped.
        if mpc_pcmd > 0.0
            && stair_transfer.is_none()
            && post_walk_settle.is_none()
            && diag_mpc_time.is_some_and(|last| t - last <= 2.0 * stair_wrench_dt + 1e-9)
            && diag_mpc_net_cop.iter().all(|v| v.is_finite())
        {
            let m = if mpc_pcmd_from_acc {
                com.xy() - wrench_mpc_acceleration.xy() / (omega * omega)
            } else {
                na::Vector2::new(diag_mpc_net_cop[0], diag_mpc_net_cop[1])
            };
            let mut m = sbox.clamp(&m).0;
            if mpc_pcmd_fb
                && let (Some(xi_s), Some(t_s)) = (diag_mpc_xi, diag_mpc_time)
            {
                let xi_pred = m + (xi_s - m) * (omega * (t - t_s)).exp();
                let e = dcm_of(&com, &com_vel, omega) - xi_pred;
                m = sbox.clamp(&(m + e * (1.0 + k_dcm))).0;
            }
            p_cmd += (m - p_cmd) * mpc_pcmd;
            n_mpc_pcmd += 1;
        }
        if matches!(support, Support::Single { .. }) {
            last_single_p_cmd = Some(p_cmd);
        }
        // Variable-height form when a stair is in the scene: the LIPM's
        // constant-height omega is ~20% too stiff while the CoM is being
        // driven DOWN a riser. Gated so level and validated runs are bit-
        // identical (VH_DCM=0 restores the classic form).
        prev_p_cmd = Some(p_cmd);
        let a_xy_raw = if stairs.is_some() && flag("VH_DCM", false) {
            // Height above the surface the pressure actually acts on, which
            // on a stair is the STANCE sole -- not the fixed `z_lip` omega
            // was built from. Measured on a descent: omega^2 claims 32.9
            // while the true g/h is 39.7, a 21% error in the commanded ZMP
            // response, in the unsafe direction (too soft).
            let surf = if stance_sides.is_empty() {
                0.0
            } else {
                stance_sides.iter().map(|&s| steps.sole[s].z).sum::<f64>()
                    / stance_sides.len() as f64
            };
            articara::biped::dcm::com_accel_xy_vh(&com, &p_cmd, omega, 0.0, surf)
        } else {
            com_accel_xy(&com, &p_cmd, omega)
        };
        let a_xy = if let Some(previous) = previous_a_xy {
            let delta = a_xy_raw - previous;
            let max_delta = com_xy_jerk_max * dt;
            if delta.norm() > max_delta {
                previous + delta * (max_delta / delta.norm())
            } else {
                a_xy_raw
            }
        } else {
            a_xy_raw
        };
        previous_a_xy = Some(a_xy);
        // z stays a PD on the nominal height: the LIPM says nothing about it,
        // and a constant-height assumption is exactly what makes omega a
        // constant in the first place.
        // Height reference. `omega` stays at the nominal height: the LIPM is
        // derived for a constant one, and re-deriving it per tick is the
        // variable-height DCM formulation, which this is not. The squat is
        // therefore a disturbance the horizontal loop has to survive, which is
        // the point of having it in the benchmark.
        // Stairs: the height reference rides the terrain. The target is the
        // mean of the two planned sole heights for THIS slice, slewed at
        // 1.5 risers per step so a riser is absorbed over roughly one step.
        // omega stays at the flat-ground value -- same argument as the squat
        // below: this is not the variable-height DCM formulation, and the
        // height change is a disturbance the horizontal loop must survive.
        let zd_terrain = if let Some(st) = &stairs {
            // A DS-only ramp was tried, but left the height reference a
            // riser behind the feet. Slew through both support phases.
            // Foot-over-foot climbs one riser per step, so the rate limit
            // turns the staircase of slice targets into a near-constant
            // slope; no gating on support phase (a DS-only ramp was tried
            // and left the reference a riser behind the feet by tread 2).
            let fs = footsteps.at_slice(slice_idx);
            // STAIR_DESC_MIN=1: descend toward the LOWER of the two planned
            // soles instead of their mean. The mean keeps the hip half a
            // riser high exactly while the swing foot needs to reach a full
            // riser down -- measured on the one-step descent, the foot
            // stalled 46 mm above the lower surface at schedule end. Riding
            // the minimum is the stance knee actively lowering the pelvis
            // through the step, the way a person walks downstairs; it also
            // removes the mean's upward bounce when the pair completes.
            let tgt_h = if stair_desc_min && st.riser < 0.0 {
                // Look ONE SLICE AHEAD: the double support BEFORE a descent
                // swing already rides down to the coming lower level, so
                // both knees share the 52 mm of lowering quasi-statically
                // and the swing starts with the pelvis low. Lowering during
                // the single support instead leaves the reach to the last
                // 30 mm of the swing, where the QP diagnosis shows the leg
                // at sigma_min = 0.004 -- a kinematic singularity -- and
                // the swing level goes Infeasible with |x| ~ 430.
                let cur = fs.sole[0].z.min(fs.sole[1].z);
                let nxt = footsteps
                    .at
                    .get(slice_idx + 1)
                    .map(|f| f.sole[0].z.min(f.sole[1].z))
                    .unwrap_or(cur);
                cur.min(nxt)
            } else {
                0.5 * (fs.sole[0].z + fs.sole[1].z)
            };
            let rate = stair_climb_rate * st.riser.abs() / t_step.max(1e-6);
            let dh = (tgt_h - terrain_track).clamp(-rate * dt, rate * dt);
            terrain_track += dh;
            // The stair crouch (see STAIR_CROUCH above) eases in at 50 mm/s.
            let dc = (stair_crouch - crouch_track).clamp(-0.05 * dt, 0.05 * dt);
            crouch_track += dc;
            (dh - dc) / dt
        } else {
            0.0
        };
        let (z_ref, zd_ref, zdd_ref) = if let Some(settle) = post_walk_settle {
            (settle.z, 0.0, 0.0)
        } else if let Some((_high, _phase, rf)) = stair_transfer_cmd {
            (rf.z, rf.zd, rf.zdd)
        } else if replan_engaged && com_height_replan && landing_timing.clock.is_some() {
            // Where the CoM height aims while BOTH feet count: historically
            // the mean sole height. Measured 2026-09-24 on the 80 mm riser
            // with the landed foot admitted at the start of double support
            // (CONTACT_ADMIT_CONFIRMED): the half-rise then completes before
            // the trailing leg has left the lower tread, that leg reaches
            // single support with its knee at 0.09 rad (champion: 0.34), and
            // the next swing task has no authority left -- the foot launches
            // to 1.2 m. COM_HEIGHT_DS_FRAC = 0 keeps the CoM at the lower
            // sole through double support and rises on the new foot. Unset =
            // the mean, computed exactly as before.
            let h = match com_height_ds_frac {
                Some(frac) if stance_sides.len() == 2 => {
                    let z0 = steps.sole[stance_sides[0]].z;
                    let z1 = steps.sole[stance_sides[1]].z;
                    z0.min(z1) + frac * (z0.max(z1) - z0.min(z1))
                }
                _ => {
                    stance_sides
                        .iter()
                        .map(|&side| steps.sole[side].z)
                        .sum::<f64>()
                        / stance_sides.len() as f64
                }
            };
            let h = if com_height_desc_lead > 0.0 {
                // The landing this swing is heading for, or -- in a double
                // support -- the landing of the swing that follows it.
                let i = gait.index_at(t);
                let landing_z = (i..gait.slices.len().min(i + 4))
                    .find_map(|j| gait.slices[j].support.swing().map(|sw| (j, sw)))
                    .map(|(j, sw)| footsteps.at_slice(j).sole[sw].z);
                match landing_z {
                    Some(lz) if lz < h - 1e-6 => h - com_height_desc_lead * (h - lz),
                    _ => h,
                }
            } else {
                h
            };
            let crouch = match (&stairs, stair_zone_crouch > 0.0) {
                (Some(st), true) => {
                    let x = steps.mid_xy().x;
                    if (x >= st.start - stair_crouch_lead && x <= st.landing_start() + stair_crouch_tail)
                        || (st.down_n > 0
                            && x >= st.landing_end() - stair_crouch_lead
                            && x <= st.landing_end() + st.down_n as f64 * st.tread + stair_crouch_tail)
                    {
                        stair_zone_crouch
                    } else {
                        0.0
                    }
                }
                _ => 0.0,
            };
            let mut target = if crouch > 0.0 {
                z_com + h - terrain_base - crouch
            } else {
                z_com + h - terrain_base
            };
            // Cap the height by what the LEGS can reach (src/biped/reach.rs).
            //
            // Measured 2026-09-24 with the new `reach_*` columns: this robot
            // walks with its legs 92-99% extended and is at 0.97-0.99 when it
            // falls. A leg that straight has no vertical authority left, which
            // is why the swing residual explodes at the knee singularity and
            // why the fixed 4 cm stair crouch helped at all -- it was this
            // cap, guessed as a constant. Here it is computed: the CoM may not
            // sit higher than the point from which every sole the plan is
            // standing on (or about to stand on) is still within `margin` of
            // the leg's length.
            //
            // The horizontal lever is measured from the CoM, and the leg hangs
            // from a hip roughly `hip_drop` below it, so the cap is written
            // about the hip and lifted back to the CoM. Default off.
            if com_height_desc_reach > 0.0 && leg_reach.max > 0.0 {
                let i = gait.index_at(t);
                if let Some((j, sw)) = (i..gait.slices.len().min(i + 4))
                    .find_map(|j| gait.slices[j].support.swing().map(|sw| (j, sw)))
                {
                    let landing = footsteps.at_slice(j).sole[sw];
                    let stance = steps.sole[1 - sw];
                    let hip_drop = hip_mi
                        .map(|mis| {
                            mis.iter()
                                .map(|&mi| com.z - misarta::se3::translation(&data.oMi[mi]).z)
                                .fold(f64::INFINITY, f64::min)
                        })
                        .unwrap_or(f64::NAN);
                    if landing.z < stance.z - 0.005 && hip_drop.is_finite() {
                        let d = 0.5 * (landing.xy() - stance.xy()).norm();
                        let span = leg_reach.max * com_height_desc_reach;
                        if d < span {
                            target = target.min(landing.z + (span * span - d * d).sqrt() + hip_drop);
                        }
                    }
                }
            }
            if reach_height > 0.0 && leg_reach.max > 0.0 {
                // The hip heights are computed later in the tick for the log;
                // here the drop is read straight off the model.
                let hip_drop = hip_mi
                    .map(|mis| {
                        mis.iter()
                            .map(|&mi| com.z - misarta::se3::translation(&data.oMi[mi]).z)
                            .fold(f64::INFINITY, f64::min)
                    })
                    .unwrap_or(f64::NAN);
                if hip_drop.is_finite() {
                    let mut cap = f64::INFINITY;
                    // Both feet the plan believes in now, plus the landing the
                    // swing is heading for: the bound has to hold at touchdown,
                    // not only at this instant.
                    let i = gait.index_at(t);
                    let next = footsteps.at_slice(i);
                    for sole in stance_sides
                        .iter()
                        .map(|&s| steps.sole[s])
                        .chain(gait.slices[i].support.swing().map(|s| next.sole[s]))
                    {
                        let d = (com.xy() - sole.xy()).norm();
                        let span = leg_reach.usable();
                        if d < span {
                            cap = cap.min(sole.z + (span * span - d * d).sqrt() + hip_drop);
                        }
                    }
                    if cap.is_finite() {
                        target = target.min(cap).max(reach_height_floor);
                    }
                }
            }
            if (target - height_plan.target).abs() > 1e-6 {
                height_plan.retarget(t, target, com_height_time);
            }
            height_plan.at(t)
        } else if squat_amp > 0.0 {
            let w = 2.0 * PI / squat_period;
            let ph = w * t;
            (
                z_com - squat_amp * (1.0 - ph.cos()) * 0.5,
                -squat_amp * 0.5 * w * ph.sin(),
                -squat_amp * 0.5 * w * w * ph.cos(),
            )
        } else {
            (
                z_com + (terrain_track - terrain_base) - crouch_track,
                zd_terrain,
                0.0,
            )
        };
        // VX2_Z_DROP: eased in at 50 mm/s on top of whichever height
        // reference is live (the flat walk never enters the replan branch).
        // PUSH_Z_DROP: lower the CoM for PUSH_Z_DROP_T after a BACKWARD
        // push (DCM error behind the plan by more than PUSH_Z_DROP_TRIG).
        // Measured 2026-10-05: 24 mm lower for the whole run doubles the
        // backward-push survival (21 -> 43/48) but trades descents away;
        // tied to the detection it leaves the other benches alone.
        let z_drop_target = (if vx2_z_from.is_some_and(|t0| t >= t0) { vx2_z_drop } else { 0.0 })
            .max(if t < push_z_drop_until { push_z_drop } else { 0.0 });
        let (z_ref, zd_ref) = if z_drop_target > 0.0 || vx2_z_track > 0.0 {
            let d = (z_drop_target - vx2_z_track).clamp(-0.05 * dt, 0.05 * dt);
            vx2_z_track += d;
            (z_ref - vx2_z_track, zd_ref - d / dt)
        } else {
            (z_ref, zd_ref)
        };
        let mpc_reach_on = mpc_reach > 0.0
            && leg_reach.max > 0.0
            && hip_mi.is_some()
            && stair_transfer_cmd.is_none()
            && wrench_mpc_apply;
        let mut mpc_reach_z: Option<f64> = None;
        // MPC_REACH_TRACK=2: lower the height reference to the lowest cap
        // the last solve saw, slewed; the nominal is kept for the cap floor.
        let z_ref_nominal = z_ref;
        let (z_ref, zd_ref) = if mpc_reach_on && mpc_reach_track == 2 {
            let want = (z_ref - mpc_reach_cap_min).max(0.0);
            let d = (want - mpc_reach_drop).clamp(-mpc_reach_slew * dt, mpc_reach_slew * dt);
            mpc_reach_drop += d;
            (z_ref - mpc_reach_drop, zd_ref - d / dt)
        } else {
            mpc_reach_drop = 0.0;
            (z_ref, zd_ref)
        };
        let a_z = zdd_ref + kp_com * (z_ref - com.z) + kd_com * (zd_ref - com_vel.z);
        let mut a_com = na::Vector3::new(a_xy.x, a_xy.y, a_z);

        // Replan both sole wrenches from the measured centroidal state. The
        // first horizon command becomes the WBC force regularisation target;
        // its resultant acceleration also becomes the CoM task reference.
        // This removes the old conflict where ZMP, height and load split were
        // generated by independent controllers.
        if stair_wrench_mpc && (stair_transfer_cmd.is_some() || wrench_mpc_always) {
            if stair_wrench_max_age > 0.0
                && wrench_mpc_command.is_some()
                && diag_mpc_time.is_some_and(|last| t - last > stair_wrench_max_age + 1e-9)
            {
                wrench_mpc_command = None;
                wrench_mpc_frames = None;
                diag_mpc_time = None;
                diag_mpc_reference = [f64::NAN; 2];
            }
            if wrench_event_replan && stair_transfer_cmd.is_some() {
                let (high, phase, _) = stair_transfer_cmd.unwrap();
                let mask = u8::from(stance_sides.contains(&0))
                    | (u8::from(stance_sides.contains(&1)) << 1);
                let context = (phase.code() as u8, mask, high);
                if wrench_command_context != Some(context) {
                    // Never reuse a wrench from the old support set if the
                    // new solve fails; generic WBC remains the fallback.
                    wrench_mpc_command = None;
                    diag_mpc_time = None;
                    diag_mpc_reference = [f64::NAN; 2];
                    wrench_mpc_next = t;
                    wrench_command_context = Some(context);
                }
            }
            if t + 1e-9 >= wrench_mpc_next {
                let horizon_time = stair_wrench_preview_dt * stair_wrench_horizon as f64;
                let weight = total_mass * G;
                // Where the MPC is asked to be at the end of its horizon.
                //
                // Historically this was the state PROPAGATED from now at the
                // DCM controller's current acceleration -- i.e. "wherever we
                // are already heading", which is a prediction, not a target.
                // An MPC given its own prediction has nothing to correct, and
                // measured 2026-09-24 the walking branch's horizontal target
                // carried no plan at all (only the height came from one).
                //
                // MPC_TARGET_PREVIEW takes the horizontal target from the CoM
                // preview QP instead, which is the stack's actual horizontal
                // plan: its nodes are the ZMP/CoM trajectory that satisfies
                // the DCM reference inside the support polygons. The MPC then
                // has something to disagree with.
                let mut target_com =
                    com + com_vel * horizon_time + a_com * (0.5 * horizon_time * horizon_time);
                let mut target_velocity = com_vel + a_com * horizon_time;
                if mpc_reach_on {
                    // The plan height itself: the MPC, not the WBC's PD,
                    // decides how to get there.
                    target_com.z = z_ref + zd_ref * horizon_time;
                    target_velocity.z = zd_ref;
                }
                if mpc_target_preview
                    && let Some((t0, ref pc, ref pv)) = preview_traj
                {
                    let k = ((t + horizon_time - t0) / com_preview_dt).round();
                    if k >= 0.0 && (k as usize) < pc.len() {
                        let k = k as usize;
                        target_com.x = pc[k].x;
                        target_com.y = pc[k].y;
                        target_velocity.x = pv[k].x;
                        target_velocity.y = pv[k].y;
                    }
                }
                // The walking branch takes `high` as the CURRENT stance foot
                // purely so the shared code below has a name for "the loaded
                // side"; with no transfer there is no high/low step, and the
                // schedules are rebuilt from the gait instead (see the `else`
                // arm of `reference_fz_schedule` below).
                let (high, phase, rf) = match stair_transfer_cmd {
                    Some(cmd) => cmd,
                    None => (
                        usize::from(!stance_sides.contains(&0)),
                        StairTransferPhase::Capture,
                        articara::biped::preview::StairTransferRef {
                            alpha: 0.0,
                            p: com.xy(),
                            z: com.z,
                            zd: 0.0,
                            zdd: 0.0,
                        },
                    ),
                };
                let rf_horizon = stair_transfer
                    .as_ref()
                    .map(|active| active.plan.at(active.clock + horizon_time))
                    .unwrap_or(rf);
                if stair_transfer_cmd.is_some() {
                    // Horizontal motion comes from the DCM acceleration. The
                    // transfer plan's p is a capture/ZMP target, not a CoM
                    // target. Height is the exception: it is the shared causal
                    // reference.
                    target_com.z = rf_horizon.z;
                    target_velocity.z = rf_horizon.zd;
                } else {
                    // No transfer: the height reference is the walk's own.
                    let (hz, hzd, _) = height_plan.at(t + horizon_time);
                    target_com.z = hz;
                    target_velocity.z = hzd;
                }
                let low = 1 - high;
                let phase_clock = stair_transfer.as_ref().map_or(0.0, |active| active.clock);
                let active_rise_first_share = stair_transfer
                    .as_ref()
                    .filter(|a| a.low_held || stair_transfer_rise_first)
                    .map(|a| a.arm_share);
                let smoothstep = |x: f64| {
                    let x = x.clamp(0.0, 1.0);
                    x * x * (3.0 - 2.0 * x)
                };
                let mut reference_fz_schedule = Vec::with_capacity(stair_wrench_horizon);
                let mut min_fz_schedule = Vec::with_capacity(stair_wrench_horizon);
                for k in 0..stair_wrench_horizon {
                    // Sample the gait at the node's CENTRE, the same instant
                    // `contact_schedule` uses.
                    //
                    // These two schedules used to disagree by half a node:
                    // this one read `k * dt`, the contact set read
                    // `(k + 0.5) * dt`. A node that straddles a lift-off then
                    // gets "swing" from one and "double support" from the
                    // other, and the MPC is handed a minimum vertical force on
                    // a foot it has been told is in the air. Measured
                    // 2026-09-24 with the MPC applied: the first node at
                    // t=22.06 said contact [false, true] and min_fz
                    // [40.2, 0.0] -- infeasible, the solve failed, the walker
                    // held the last wrench across a support change and put
                    // 770 N through the wrong foot.
                    let stage_dt = if mpc_node_centre {
                        (k as f64 + 0.5) * stair_wrench_preview_dt
                    } else {
                        k as f64 * stair_wrench_preview_dt
                    };
                    // No transfer: the load split is the GAIT's, read forward
                    // along the horizon. Double support is an even split;
                    // single support puts the whole weight on the stance sole.
                    // This is the schedule the walk is already executing, so
                    // the MPC predicts the motion rather than a primitive.
                    let walking_share = stair_transfer_cmd.is_none().then(|| {
                        let idx = gait.index_at(t + stage_dt);
                        match gait.slices[idx].support.swing() {
                            None if load_share_ramp => {
                                let sl = gait.slices[idx];
                                let total = (measured_pre[0] + measured_pre[1]).max(1e-6);
                                let share0 = (measured_pre[high] / total).clamp(0.0, 1.0);
                                // Where the DS is heading: 0 if `high` swings next, else 1.
                                let share1 = gait
                                    .slices
                                    .get(idx + 1)
                                    .and_then(|n| n.support.swing())
                                    .map_or(0.5, |sw| if sw == high { 0.0 } else { 1.0 });
                                let u = sl.frac(t + stage_dt);
                                let a = u * u * (3.0 - 2.0 * u);
                                share0 + (share1 - share0) * a
                            }
                            None => 0.5,
                            Some(sw) => {
                                if sw == high {
                                    0.0
                                } else {
                                    1.0
                                }
                            }
                        }
                    });
                    let (high_share, touchdown_beta): (f64, f64) =
                        if let Some(sh) = walking_share {
                            (sh, 1.0)
                        } else {
                        match phase {
                        StairTransferPhase::Capture => {
                            // Keep the pre-touchdown controller identical to
                            // the validated baseline. The event ramp begins
                            // only after physical contact is admitted.
                            if stair_capture_load_track {
                                // From the measured share toward 0.5 over the
                                // capture time, then the historical release.
                                let sh = stair_capture_share.unwrap_or(0.5);
                                let u = smoothstep((phase_clock + stage_dt) / stair_capture_time);
                                let base = (sh + (0.5 - sh) * u).min(sh + stair_capture_lead).max(sh.min(0.5));
                                if stair_release_time > 0.0 {
                                    let duration = stair_transfer.as_ref().unwrap().plan.duration;
                                    let beta = smoothstep(
                                        (phase_clock + stage_dt - duration) / stair_release_time,
                                    );
                                    (base + (0.95 - base) * beta, 1.0)
                                } else {
                                    (base, 1.0)
                                }
                            } else if stair_release_time > 0.0 {
                                let duration = stair_transfer.as_ref().unwrap().plan.duration;
                                let beta = smoothstep(
                                    (phase_clock + stage_dt - duration) / stair_release_time,
                                );
                                (0.50 + 0.05 * rf.alpha + 0.45 * beta, 1.0)
                            } else {
                                (0.50 + 0.05 * rf.alpha, 1.0)
                            }
                        }
                        StairTransferPhase::Rise => {
                            if let Some(ramp) = touchdown_load_ramp.filter(|ramp| ramp.high == high)
                            {
                                let beta = smoothstep(
                                    (t + stage_dt - ramp.contact_t) / stair_touchdown_load_time,
                                );
                                (
                                    ramp.start_share[high]
                                        + (stair_touchdown_high_share - ramp.start_share[high])
                                            * beta,
                                    beta,
                                )
                            } else if active_rise_first_share.is_some() {
                                // rise_first: the split follows the phase
                                // clock from the share at arming to all on the
                                // high foot; the low foot's floor fades with it.
                                let s0 = active_rise_first_share.unwrap();
                                let u = smoothstep((phase_clock + stage_dt) / stair_rise_time);
                                (s0 + (1.0 - s0) * u, 1.0 - u)
                            } else {
                                (1.0, 0.0)
                            }
                        }
                        StairTransferPhase::Commit => {
                            // Rise already moved support directly to the new
                            // foot; Commit only finishes the last 5% unload.
                            let alpha = smoothstep((phase_clock + stage_dt) / stair_commit_time);
                            (
                                stair_touchdown_high_share
                                    + (0.10 - stair_touchdown_high_share) * alpha,
                                1.0,
                            )
                        }
                        }
                        };
                    let mut reference = [0.0; 2];
                    let mut minimum = [0.0; 2];
                    reference[high] = high_share * weight;
                    reference[low] = (1.0 - high_share) * weight;
                    if let Some(sh) = walking_share {
                        minimum[high] = stair_wrench_low_min * weight * sh;
                        minimum[low] = stair_wrench_low_min * weight * (1.0 - sh);
                    } else {
                        match phase {
                        StairTransferPhase::Capture => {
                            minimum[high] = stair_wrench_high_min * weight;
                            let beta = if stair_release_time > 0.0 {
                                let duration = stair_transfer.as_ref().unwrap().plan.duration;
                                smoothstep((phase_clock + stage_dt - duration) / stair_release_time)
                            } else {
                                0.0
                            };
                            minimum[low] = stair_wrench_low_min * weight * (1.0 - beta);
                        }
                        StairTransferPhase::Rise => {
                            minimum[high] = stair_wrench_high_min * weight;
                            minimum[low] = stair_wrench_low_min * touchdown_beta * weight;
                        }
                        StairTransferPhase::Commit => {
                            let alpha = smoothstep((phase_clock + stage_dt) / stair_commit_time);
                            minimum[high] = stair_wrench_low_min * (1.0 - alpha) * weight;
                            minimum[low] = stair_wrench_low_min * weight;
                        }
                        }
                    }
                    for side in 0..2 {
                        let activation = if stair_continuous_contact {
                            contact_force_activation[side]
                        } else {
                            1.0
                        };
                        let capacity = f_max_per_foot * activation;
                        minimum[side] = minimum[side].min(capacity);
                        reference[side] = reference[side].max(minimum[side]).min(capacity);
                    }
                    reference_fz_schedule.push(reference);
                    min_fz_schedule.push(minimum);
                }
                let actual = Footsteps::from_fk(
                    data,
                    rig.foot_mi,
                    prof.sole_centre_x,
                    prof.sole_below_origin,
                );
                let mut previous =
                    wrench_mpc_command.unwrap_or([articara::biped::wrench_mpc::Wrench::zeros(); 2]);
                // MPC_SEED_PREVIOUS: start the smoothness anchor at the
                // MEASURED load instead of at zero.
                //
                // The smoothness term penalises distance from `previous`, and
                // on the first solve `previous` is zero. With the weight that
                // kills the chatter (MPC_W_SMOOTH=10, 1e4x the default) the
                // commanded wrench then ramps up at ~150 N/s: measured
                // 2026-09-26, 148 N total at t=1.0 s and 113 N on the stance
                // leg as the first single support began, against 353 N the
                // WBC needed. Under MPC_SOLE_AUTHORITY that under-load IS the
                // regulariser target; the welded robot absorbed it (151/192),
                // the free-arm robot touched down early at 2.6 s and fell at
                // 3.2 s in half the members. The stair-transfer ramp already
                // seeds `previous` this way (`seed_previous`); this does the
                // same for the first solve of a walk. Off = zero seed = the
                // shipped behaviour.
                if mpc_seed_previous == 2 || (mpc_seed_previous == 1 && wrench_mpc_command.is_none()) {
                    for side in 0..2 {
                        previous[side][5] = measured_pre[side].max(0.0);
                    }
                }
                if let Some(ramp) = touchdown_load_ramp.as_mut()
                    && ramp.high == high
                    && ramp.seed_previous
                {
                    previous = [articara::biped::wrench_mpc::Wrench::zeros(); 2];
                    for side in 0..2 {
                        previous[side][5] = measured_pre[side].max(0.0);
                    }
                    ramp.seed_previous = false;
                }
                let mut contacts: Vec<articara::biped::wrench_mpc::Contact> = Vec::with_capacity(2);
                for side in 0..2 {
                    let rotation = misarta::se3::rotation_matrix(&data.oMi[rig.foot_mi[side]]);
                    let s_pitch = -rotation[(2, 0)];
                    let rolling = stair_hybrid_contact && s_pitch.abs() >= 0.087;
                    let cop_half = if rolling {
                        (0.006, contact_cfg.cop_half.1)
                    } else {
                        contact_cfg.cop_half
                    };
                    // The WBC selector and MPC wrench must use the same
                    // point. During rolling contact that point is the toe or
                    // heel strip centre rather than the sole centre.
                    let patch_x = if rolling {
                        (sole_half_l - cop_half.0) * s_pitch.signum()
                    } else {
                        0.0
                    };
                    let position =
                        actual.sole[side] + rotation * na::Vector3::new(patch_x, 0.0, 0.0);
                    contacts.push(articara::biped::wrench_mpc::Contact {
                        position,
                        rotation,
                        cop_half,
                    });
                }
                // A foot in the air now but scheduled to be in contact within
                // the horizon: place its MPC contact at the forecast landing
                // rather than where it is mid-swing.
                if stair_transfer_cmd.is_none() && wrench_mpc_apply {
                    let end = t + stair_wrench_horizon as f64 * stair_wrench_preview_dt;
                    let fc = articara::biped::preview::forecast_feet(
                        &gait,
                        &footsteps,
                        &actual,
                        &stance_sides,
                        t,
                        end - 1e-9,
                    );
                    for side in 0..2 {
                        if !stance_sides.contains(&side) {
                            let yaw = fc.yaw[side];
                            let (sy, cy) = yaw.sin_cos();
                            let rot = na::Matrix3::new(cy, -sy, 0.0, sy, cy, 0.0, 0.0, 0.0, 1.0);
                            contacts[side].position = fc.sole[side];
                            contacts[side].rotation = rot;
                        }
                    }
                }
                let cmm = misarta::centroidal::compute_centroidal_momentum_matrix(&rig.model, q);
                let h = &cmm * v_dvec;
                // MPC_FOOTSTEP: in single support, the swing foot's landing is
                // a variable from the first node at which the schedule has it
                // down.
                // Inward (toward-stance) bound on the landing shift and which
                // sign of y is inward: MPC_FOOTSTEP_MAX_Y_IN, and with
                // MPC_FOOTSTEP_MIN_WIDTH also whatever keeps the NOMINAL
                // landing (this slice's applied shift removed) that wide.
                let inward_bound = |i: usize| -> (f64, bool) {
                    let (gap, plus) = support
                        .swing()
                        .map(|sw| landing_inward(&footsteps, i, sw))
                        .unwrap_or((0.0, true));
                    let mut b = mpc_footstep_max_y_in;
                    if mpc_footstep_min_width > 0.0 {
                        let applied = if mpc_fs_slice == Some(i) { mpc_fs_applied.y } else { 0.0 };
                        // Toward stance is +y when plus: the applied shift
                        // already narrowed the gap by `applied` (signed).
                        let nominal_gap = if plus { gap + applied } else { gap - applied };
                        b = b.min((nominal_gap - mpc_footstep_min_width).max(0.0));
                    }
                    if mpc_footstep_y_in_retimed > 0.0 && cp_retimed_slice == Some(i) {
                        b = b.max(mpc_footstep_y_in_retimed);
                    }
                    // Around a re-step: its own swing (restep_last - 1) and the
                    // next RESTEP_Y_IN_STEPS single supports (two slices each).
                    if let Some(l) = restep_last
                        && i + 1 >= l
                        && i <= l + 2 * restep_y_in_steps
                    {
                        b = b.max(restep_y_in);
                    }
                    if mpc_footstep_y_in_fwd > 0.0
                        && xi.x - dcm_plan.reference(t).xi.x >= mpc_footstep_y_in_fwd_trig
                    {
                        b = b.max(mpc_footstep_y_in_fwd);
                    }
                    (b, plus)
                };
                let landing = if mpc_footstep
                    && stair_transfer_cmd.is_none()
                    && wrench_mpc_apply
                    && let Support::Single { swing, .. } = support
                    && {
                        let i = gait.index_at(t);
                        if mpc_footstep_trig > 0.0
                            && dcm_err >= mpc_footstep_trig
                            && !mpc_fs_armed.is_some_and(|(a, u)| a <= i && i <= u)
                        {
                            // A step is two slices (single + double support).
                            mpc_fs_armed = Some((i, i + 2 * mpc_footstep_trig_hold));
                        }
                        mpc_footstep_trig == 0.0
                            || mpc_fs_armed.is_some_and(|(a, u)| a <= i && i <= u)
                    }
                    && {
                        // The single support's own clock. LandingTiming
                        // extends this slice through a late contact (so
                        // `index_at` stays on it) but keeps the clock's
                        // NOMINAL duration, so the fraction passes 1 while
                        // the foot hovers; a contact_plan retime SHORTENS the
                        // live slice. The shorter of the two covers both.
                        let i = gait.index_at(t);
                        let sl = &gait.slices[i];
                        let (start, duration) = landing_timing
                            .clock
                            .filter(|(k, _, _)| *k == i)
                            .map(|(_, start, duration)| (start, duration.min(sl.t1 - start)))
                            .unwrap_or((sl.t0, sl.duration()));
                        let frac = (t - start) / duration.max(1e-9);
                        // Upward only: the hover-above-the-tread case.
                        let rising = footsteps.at_slice(i).sole[swing].z
                            - footsteps.at_slice(i.saturating_sub(1)).sole[swing].z
                            > 0.005;
                        if mpc_fs_debug {
                            println!(
                                "  [fs-gate] t={t:.3} i={i} frac={frac:.3} rising={rising} clock={:?}",
                                landing_timing.clock
                            );
                        }
                        // Event mode: no landing moves while the foot is
                        // probing for the ground (measured 2026-10-04: the
                        // goal moved 5-12 cm under a foot 2 cm from contact).
                        !landing_timing.landing_closed(i)
                            && ref_mode.is_none()
                            && frac < if rising {
                                mpc_footstep_freeze_frac_rise
                            } else {
                                mpc_footstep_freeze_frac
                            }
                    }
                {
                    (0..stair_wrench_horizon)
                        .find(|&k| {
                            let at = t + (k as f64 + 0.5) * stair_wrench_preview_dt;
                            match gait.support_at(at) {
                                Support::Double => true,
                                Support::Single { swing: sw, .. } => sw != swing,
                            }
                        })
                        .map(|first_node| articara::biped::wrench_mpc::Landing {
                            foot: swing,
                            first_node,
                            min_first_node: if mpc_landing_time {
                                // Like the contact_plan: at least this
                                // fraction of the REMAINING single support
                                // (the swing has to come down).
                                let i = gait.index_at(t);
                                let sl = &gait.slices[i];
                                let earliest = t + (sl.t1 - t).max(0.0) * mpc_landing_tmin_frac;
                                let k = ((earliest - t) / stair_wrench_preview_dt - 0.5).ceil().max(1.0) as usize;
                                k.min(first_node)
                            } else {
                                first_node
                            },
                            position: [contacts[swing].position.x, contacts[swing].position.y],
                            terrain_x: (mpc_footstep_terrain_w > 0.0)
                                .then(|| stairs.as_ref())
                                .flatten()
                                .and_then(|st| {
                                    let prior = mpc_fs_applied.x;
                                    let xn = contacts[swing].position.x + prior;
                                    st.sole_window(xn, sole_half_l).map(|(lo, hi)| {
                                        let m = mpc_footstep_terrain_margin.min((hi - lo) * 0.5);
                                        (lo + m, hi - m, mpc_footstep_terrain_w)
                                    })
                                }),
                            weight: mpc_footstep_w,
                            max: {
                                let i = gait.index_at(t);
                                let (inward, inward_is_plus) = inward_bound(i);
                                let mut m = mpc_footstep_max;
                                if inward_is_plus {
                                    m[1] = m[1].min(inward);
                                }
                                m
                            },
                            prior: if mpc_footstep_increment
                                && mpc_fs_slice == Some(gait.index_at(t))
                            {
                                [mpc_fs_applied.x, mpc_fs_applied.y]
                            } else {
                                [0.0; 2]
                            },
                            min: {
                                let i = gait.index_at(t);
                                let stair = (footsteps.at_slice(i).sole[swing].z
                                    - footsteps.at_slice(i.saturating_sub(1)).sole[swing].z)
                                    .abs()
                                    > 0.005;
                                let mut m = mpc_footstep_max;
                                if stair {
                                    m[0] = m[0].min(mpc_footstep_stair_back);
                                }
                                let (inward, inward_is_plus) = inward_bound(i);
                                if !inward_is_plus {
                                    m[1] = m[1].min(inward);
                                }
                                m
                            },
                        })
                        .map(|mut land| {
                            if mpc_footstep_capture_floor > 0.0 {
                                let i = gait.index_at(t);
                                let sl = &gait.slices[i];
                                let seg = &dcm_plan.segs[dcm_plan.index_at(t)];
                                let rem = (sl.t1 - t).max(0.0);
                                let p0 = seg.at(t);
                                let pd = seg.slope();
                                let v_over_w = pd / omega;
                                let xi1 = p0 + pd * rem + v_over_w + (xi - p0 - v_over_w) * (omega * rem).exp();
                                let p1 = p0 + pd * rem;
                                let t_ds = gait.slices.get(i + 1).map_or(0.0, |n| n.duration());
                                let xi2 = p1 + (xi1 - p1) * (omega * t_ds).exp();
                                let reach = sole_half_l * mpc_footstep_capture_floor;
                                let base = land.position[0] + land.prior[0];
                                // Backward: the sole's front edge must reach back to xi2.
                                let need_back = base - (xi2.x + reach);
                                if need_back > 0.0 {
                                    land.max[0] = land.max[0].min(-need_back);
                                }
                                // Forward: the sole's rear edge must reach xi2.
                                let need_fwd = (xi2.x - reach) - base;
                                if need_fwd > 0.0 {
                                    land.min[0] = land.min[0].min(-need_fwd);
                                }
                                if std::env::var("MPC_FS_DEBUG_LANDING").is_ok() {
                                    println!("  [fs-floor] t={t:.3} rem={rem:.3} t_ds={t_ds:.3} xi1={:.3} xi2={:.3} base={base:.3} need_back={need_back:+.3} need_fwd={need_fwd:+.3} max={:?} min={:?}", xi1.x, xi2.x, land.max, land.min);
                                }
                            }
                            land
                        })
                } else {
                    None
                };
                // MPC_REACH: per-node height cap from the feet the schedule
                // has down at that node and the previous solve's CoM.
                let com_z_max = mpc_reach_on.then(|| {
                    let l_u = leg_reach.max * mpc_reach;
                    let hips = hip_mi.unwrap().map(|mi| {
                        let p = misarta::se3::translation(&data.oMi[mi]);
                        (na::Vector2::new(p.x - com.x, p.y - com.y), com.z - p.z)
                    });
                    (0..stair_wrench_horizon)
                        .map(|k| {
                            let at = t + (k as f64 + 0.5) * stair_wrench_preview_dt;
                            let tau = (k as f64 + 1.0) * stair_wrench_preview_dt;
                            let c_xy = wrench_mpc_traj
                                .as_ref()
                                .and_then(|(_, _, _, tr)| tr.get(k))
                                .map(|[c, _]| c.xy())
                                .unwrap_or_else(|| com.xy() + com_vel.xy() * tau);
                            let down = match gait.support_at(at) {
                                Support::Double => [true, true],
                                Support::Single { swing, .. } => [swing != 0, swing != 1],
                            };
                            let mut cap = f64::INFINITY;
                            for f in 0..2 {
                                let sole = contacts[f].position;
                                if !down[f]
                                    || reference_fz_schedule[k][f] < mpc_reach_min_load * total_mass * G
                                    || (mpc_reach_ahead >= 0.0 && sole.x - c_xy.x < mpc_reach_ahead)
                                {
                                    continue;
                                }
                                let (off, drop) = hips[f];
                                let d = if mpc_reach_sagittal {
                                    (c_xy.x + off.x - sole.x).abs()
                                } else {
                                    (c_xy + off - sole.xy()).norm()
                                };
                                let span = (l_u * l_u - d * d).max(0.0).sqrt();
                                cap = cap.min(sole.z + span + drop);
                            }
                            // A node with no qualifying foot has no cap; keep it
                            // finite (the MPC rejects non-finite inputs -- measured
                            // 2026-10-10: the ahead-only gate left such nodes at
                            // +inf and every solve failed, 36 -> 0/48).
                            cap.max(z_ref_nominal - mpc_reach_max_drop).min(z_ref_nominal + 1.0)
                        })
                        .collect::<Vec<f64>>()
                });
                if let Some(c) = &com_z_max {
                    mpc_reach_cap_min = c.iter().cloned().fold(f64::INFINITY, f64::min);
                }
                if mpc_reach_debug && let Some(c) = &com_z_max {
                    let (k, m) = c.iter().enumerate().fold((0, f64::INFINITY), |a, (k, &v)| if v < a.1 { (k, v) } else { a });
                    if m < z_ref - 0.003 {
                        println!("  [reach] t={t:.3} cap min {m:.3} at node {k} (z_ref {z_ref:.3}, com {:.3}) caps {}", com.z, c.iter().map(|v| format!("{v:.3}")).collect::<Vec<_>>().join(" "));
                    }
                }
                let input = articara::biped::wrench_mpc::Input {
                    com,
                    velocity: com_vel,
                    angular_momentum: na::Vector3::new(h[0], h[1], h[2]),
                    target_com,
                    target_velocity,
                    target_acceleration: if mpc_reach_on {
                        na::Vector3::new(a_com.x, a_com.y, zdd_ref)
                    } else {
                        a_com
                    },
                    com_z_max,
                    contacts: [contacts[0].clone(), contacts[1].clone()],
                    // Walking branch: the contact SET over the horizon comes
                    // from the gait, node by node (lift-off and landing inside
                    // the horizon), the same way the CoM preview builds its
                    // support boxes. Repeating the current stance for 0.4 s
                    // -- the historical construction, kept for transfers --
                    // gave a solution that, once applied (WRENCH_MPC_APPLY),
                    // could not walk at all (0/192, measured 2026-09-24).
                    contact_schedule: if stair_transfer_cmd.is_none() && wrench_mpc_apply {
                        (0..stair_wrench_horizon)
                            .map(|k| {
                                let at = t + (k as f64 + 0.5) * stair_wrench_preview_dt;
                                match gait.support_at(at) {
                                    Support::Double => [true, true],
                                    Support::Single { swing, .. } => [swing != 0, swing != 1],
                                }
                            })
                            .collect()
                    } else {
                        vec![
                            [stance_sides.contains(&0), stance_sides.contains(&1)];
                            stair_wrench_horizon
                        ]
                    },
                    // What fraction of its capacity each foot may use at
                    // each node.
                    //
                    // This used to be a constant 1.0 while walking, i.e. the
                    // MPC solved as if both feet were fully available for the
                    // whole double support -- while the WBC was ramping them
                    // through `LOAD_RAMP_FRAC` the entire time. Measured
                    // 2026-09-24 with the MPC applied: at a landing the MPC
                    // asked for 127/276 N and the QP produced 40/301, the
                    // mismatch held for 0.5 s, and the run then took 782 N on
                    // one foot and fell. The two were solving against
                    // different contact models. MPC_LOAD_SCHEDULE gives the
                    // MPC the WBC's own ramp.
                    activation_schedule: if mpc_load_schedule && stair_transfer_cmd.is_none() {
                        (0..stair_wrench_horizon)
                            .map(|k| {
                                let at = t + (k as f64 + 0.5) * stair_wrench_preview_dt;
                                let (sl, frac) = gait.at(at);
                                match sl.support {
                                    Support::Double => {
                                        let i = gait.index_at(at);
                                        let landed = i
                                            .checked_sub(1)
                                            .and_then(|j| gait.slices[j].support.swing());
                                        let next =
                                            gait.slices.get(i + 1).and_then(|s| s.support.swing());
                                        let mut a = [1.0_f64; 2];
                                        if let Some(l) = landed {
                                            a[l] = (frac / ramp_frac_mpc).clamp(0.0, 1.0);
                                        }
                                        if let Some(l) = next {
                                            a[l] = ((1.0 - frac) / ramp_frac_mpc).clamp(0.0, 1.0);
                                        }
                                        a
                                    }
                                    Support::Single { stance, swing } => {
                                        let mut a = [0.0_f64; 2];
                                        a[stance] = 1.0;
                                        a[swing] = 0.0;
                                        a
                                    }
                                }
                            })
                            .collect()
                    } else {
                        vec![
                            if stair_continuous_contact {
                                contact_force_activation
                            } else {
                                [1.0; 2]
                            };
                            stair_wrench_horizon
                        ]
                    },
                    reference_fz_schedule,
                    // Where the walk wants the pressure, node by node, in
                    // each sole's own frame. Without it the MPC has no
                    // opinion about the CoP and picks the split its wrench
                    // terms like, which is a different CoP from the one the
                    // DCM plan is steering by (measured 2026-09-25: totals
                    // agree to 1%, the split differs by 40-55 N, and applying
                    // the MPC's answer cannot walk at any blend).
                    reference_cop_schedule: None,
                    // Where the walk wants the NET pressure of both feet,
                    // node by node, in world. This is what the DCM plan
                    // actually constrains; asking each sole for its own CoP
                    // made the disagreement worse (43 -> 63 N, 2026-09-25).
                    reference_net_cop_schedule: (mpc_net_cop_weight > 0.0
                        || mpc_net_cop_eq)
                        .then(|| {
                        let shift = if mpc_cop_ref_cmd {
                            let raw = p_cmd - dcm_plan.reference(t).p;
                            let n = raw.norm();
                            if n > mpc_cop_ref_cmd_max && n > 0.0 {
                                raw * (mpc_cop_ref_cmd_max / n)
                            } else {
                                raw
                            }
                        } else {
                            na::Vector2::zeros()
                        };
                        (0..stair_wrench_horizon.max(mpc_landing_capture_n))
                            .map(|k| {
                                let at = t + (k as f64 + 0.5) * stair_wrench_preview_dt;
                                if let (Some(mode), Some(lib)) = (ref_mode, ref_lib.as_ref())
                                    && ref_mpc
                                    && at - mode.t0 + mode.tau0 <= lib.total()
                                {
                                    mode.world_xy(lib, lib.cop_at(at - mode.t0 + mode.tau0))
                                } else {
                                    dcm_plan.reference(at).p + shift
                                }
                            })
                            .collect()
                    }),
                    reference_dcm_schedule: (mpc_dcm_weight > 0.0
                        || mpc_landing_capture_w > 0.0
                        || mpc_capture_lex)
                        .then(|| {
                        (0..stair_wrench_horizon.max(mpc_landing_capture_n))
                            .map(|k| {
                                let at = t + (k as f64 + 0.5) * stair_wrench_preview_dt;
                                if let (Some(mode), Some(lib)) = (ref_mode, ref_lib.as_ref())
                                    && ref_mpc
                                    && at - mode.t0 + mode.tau0 <= lib.total()
                                {
                                    mode.world_xy(lib, lib.dcm_at(at - mode.t0 + mode.tau0, omega))
                                } else {
                                    dcm_plan.reference(at).xi
                                }
                            })
                            .collect()
                    }),
                    min_fz_schedule,
                    previous,
                    landing,
                    dcm_now: (mpc_landing_capture_w > 0.0 || mpc_capture_lex).then_some(xi),
                    reference_dcm_sensitivity: (mpc_landing_capture_j
                        && mpc_landing_capture_w > 0.0)
                        .then(|| landing)
                        .flatten()
                        .map(|land| {
                            let i = gait.index_at(t);
                            let du = 0.02;
                            let mut cols = [Vec::new(), Vec::new()];
                            for axis in 0..2 {
                                let mut probe = footsteps.clone();
                                let mut d = na::Vector2::zeros();
                                d[axis] = du;
                                probe.shift_next(i, land.foot, d);
                                let moved = DcmPlan::from_footsteps_rolled(
                                    &gait,
                                    &probe,
                                    z_lip,
                                    zmp_lat_scale,
                                    zmp_roll,
                                );
                                cols[axis] = (0..stair_wrench_horizon.max(mpc_landing_capture_n))
                                    .map(|k| {
                                        let at = t + (k as f64 + 0.5) * stair_wrench_preview_dt;
                                        (moved.reference(at).xi - dcm_plan.reference(at).xi) / du
                                    })
                                    .collect();
                            }
                            (0..stair_wrench_horizon.max(mpc_landing_capture_n))
                                .map(|k| {
                                    na::Matrix2::from_columns(&[cols[0][k], cols[1][k]])
                                })
                                .collect()
                        }),
                    prior_com: (mpc_arm_about_prior)
                        .then(|| wrench_mpc_traj.as_ref().map(|(_, _, _, tr)| tr.iter().map(|[c, _]| *c).collect()))
                        .flatten(),
                };
                let mut config = articara::biped::wrench_mpc::Config {
                    horizon: stair_wrench_horizon,
                    dt: stair_wrench_preview_dt,
                    mass: total_mass,
                    gravity: G,
                    friction_mu,
                    cop_half: contact_cfg.cop_half,
                    torsion_mu: contact_cfg.mu_torsion,
                    max_fz: mpc_f_max_per_foot,
                    validate_solution: mpc_gate,
                    // The cost weights, so the ones that compete with the CoP
                    // term can be measured against it. Defaults are the
                    // shipped values, so leaving them alone is bit-identical.
                    position_weight: mpc_w_pos,
                    velocity_weight: mpc_w_vel,
                    momentum_weight: mpc_w_mom,
                    acceleration_weight: mpc_w_acc,
                    wrench_weight: mpc_w_wrench,
                    dcm_weight: mpc_dcm_weight,
                    omega,
                    landing_capture_weight: mpc_landing_capture_w,
                    landing_capture_horizon: mpc_landing_capture_n,
                    landing_lexicographic: mpc_landing_lex,
                    landing_timing: mpc_landing_time,
                    landing_time_weight: mpc_landing_time_w,
                    landing_time_from_x: flag("MPC_LANDING_TIME_FROM_X", true),
                    landing_time_gain_min: env_f64("MPC_LANDING_TIME_GAIN", 1e-3).max(0.0),
                    fz_split_equality: mpc_fz_split_eq,
                    capture_cop_weight: mpc_capture_cop_w,
                    capture_cop_lexicographic: mpc_capture_lex,
                    capture_cop_wp: mpc_capture_lex_wp,
                    capture_cop_ws: mpc_capture_lex_ws,
                    capture_cop_margin: mpc_capture_lex_margin,
                    capture_cop_net_eq: flag("MPC_CAPTURE_LEX_NET_EQ", true),
                    capture_cop_total_eq: flag("MPC_CAPTURE_LEX_TOTAL_EQ", true),
                    capture_cop_box_ignores_min: flag("MPC_CAPTURE_LEX_IGNORE_MIN", false),
                    capture_cop_force_margin: env_f64("MPC_CAPTURE_LEX_FMARGIN", 0.1),
                    landing_moment_coupling: mpc_footstep_moment,
                    arm_about_prior: mpc_arm_about_prior,
                    ldot_weight: mpc_w_ldot,
                    smooth_weight: mpc_w_smooth,
                    cop_weight: mpc_cop_weight,
                    net_cop_weight: mpc_net_cop_weight,
                    net_cop_equality: mpc_net_cop_eq,
                    reach_weight: mpc_reach_w,
                    ..Default::default()
                };
                config.qp.absolute_gap = mpc_gate;
                if mpc_capture_lex {
                    config.qp.solver = misa_wbc::QpSolver::Clarabel;
                }
                // Interior-point solutions carry complementarity slack at the
                // gap setting; the KKT gate's 1e-6 rejected every one.
                config.kkt_complementarity_tolerance = env_f64("MPC_KKT_COMP_TOL", if mpc_capture_lex { 1e-2 } else { 1e-6 });
                if let Some(backend) = mpc_backend_override {
                    config.qp.solver = backend;
                }
                if let Some(dir) = &mpc_diag_dir {
                    let capture_start = std::time::Instant::now();
                    let contacts: Vec<_> = input.contacts.iter().map(|c| serde_json::json!({
                        "position":c.position.as_slice(),"rotation":c.rotation.as_slice(),"cop_half":c.cop_half})).collect();
                    let dump = serde_json::json!({"schema":1,"t":t,"ctrl_dt":dt,"mpc_update_dt":stair_wrench_dt,
                        "phase":format!("{phase:?}"),"input":{
                            "com":input.com.as_slice(),"velocity":input.velocity.as_slice(),"angular_momentum":input.angular_momentum.as_slice(),
                            "target_com":input.target_com.as_slice(),"target_velocity":input.target_velocity.as_slice(),"target_acceleration":input.target_acceleration.as_slice(),
                            "contacts":contacts,"contact_schedule":input.contact_schedule,"activation_schedule":input.activation_schedule,
                            "reference_fz_schedule":input.reference_fz_schedule,"min_fz_schedule":input.min_fz_schedule,
                            "previous":[input.previous[0].as_slice(),input.previous[1].as_slice()]},
                        "config":{"horizon":config.horizon,"dt":config.dt,"mass":config.mass,"gravity":config.gravity,
                            "friction_mu":config.friction_mu,"cop_half":config.cop_half,"torsion_mu":config.torsion_mu,"max_fz":config.max_fz,
                            "position_weight":config.position_weight,"velocity_weight":config.velocity_weight,"momentum_weight":config.momentum_weight,
                            "acceleration_weight":config.acceleration_weight,"wrench_weight":config.wrench_weight,"smooth_weight":config.smooth_weight,"validate_solution":config.validate_solution}});
                    std::fs::create_dir_all(dir).expect("MPC_DIAG_DIR");
                    std::fs::write(
                        format!("{dir}/mpc_{mpc_diag_count:06}.json"),
                        serde_json::to_vec(&dump).unwrap(),
                    )
                    .expect("MPC snapshot");
                    mpc_diag_count += 1;
                    mpc_capture_ns = capture_start.elapsed().as_nanos();
                }
                let previous_age = diag_mpc_time.map_or(-1.0, |stamp| t - stamp);
                let mpc_solve_start = std::time::Instant::now();
                let mpc_result = articara::biped::wrench_mpc::solve_with_workspace(
                    &input,
                    &config,
                    if mpc_warm {
                        Some(&mut mpc_workspace)
                    } else {
                        None
                    },
                );
                let mpc_solve_ns = mpc_solve_start.elapsed().as_nanos();
                if let Some(file) = &mut mpc_runtime_log {
                    use std::io::Write;
                    let mask = u8::from(stance_sides.contains(&0))
                        | (u8::from(stance_sides.contains(&1)) << 1);
                    writeln!(file, "{t:.9},{phase:?},{mask},{:?},{},{stair_wrench_dt:.9},{mpc_solve_ns},{},{},{previous_age:.9}",
                        config.qp.solver, config.horizon,
                        mpc_solve_ns > (stair_wrench_dt * 1e9) as u128,
                        mpc_result.is_some()).unwrap();
                }
                match mpc_result {
                    Some(solution) => {
                        let mut by_side = [articara::biped::wrench_mpc::Wrench::zeros(); 2];
                        by_side.copy_from_slice(&solution.sole_wrenches);
                        diag_mpc_reference = input.reference_fz_schedule[0];
                        diag_mpc_time = Some(t);
                        diag_mpc_xi = Some(dcm_of(&com, &com_vel, omega));
                        wrench_mpc_command = Some(by_side);
                        {
                            let (mut mx, mut my, mut fz) = (0.0, 0.0, 0.0);
                            for foot in 0..2 {
                                let c = &input.contacts[foot];
                                let w = by_side[foot];
                                let m = c.rotation * na::Vector3::new(w[0], w[1], w[2]);
                                let f = c.rotation * na::Vector3::new(w[3], w[4], w[5]);
                                // Moment of this contact about the world origin,
                                // horizontal components only.
                                mx += m.x + c.position.y * f.z - c.position.z * f.y;
                                my += m.y - c.position.x * f.z + c.position.z * f.x;
                                fz += f.z;
                            }
                            diag_mpc_net_cop = if fz > 1.0 {
                                [-my / fz, mx / fz]
                            } else {
                                [f64::NAN; 2]
                            };
                        }
                        if qp_wrench_log.is_some() {
                            wrench_mpc_frames = Some(input.contacts.clone());
                        }
                        wrench_mpc_acceleration = solution.acceleration;
                        wrench_mpc_traj = Some((t, com, com_vel, solution.trajectory.clone()));
                        // MPC_LANDING_TIME: the touchdown node the landing
                        // stage chose, applied as a retime of this single
                        // support (once per slice, like the contact_plan).
                        if mpc_landing_time
                            && let (Some(k_td), Some(land)) = (solution.landing_node, landing)
                            && k_td < land.first_node
                        {
                            let i = gait.index_at(t);
                            let ne = t + (k_td as f64 + 0.5) * stair_wrench_preview_dt;
                            if ne > t + dt && mpc_retimed_slice != Some(i) && matches!(support, Support::Single { .. }) {
                                gait.retime(i, ne);
                                n_mpc_retime += 1;
                                mpc_retimed_slice = Some(i);
                                cp_retimed_slice = Some(i);
                                landing_timing.max_extension = 0.0;
                                dcm_plan = DcmPlan::from_footsteps_rolled(&gait, &footsteps, z_lip, zmp_lat_scale, zmp_roll);
                                n_replan += 1;
                            }
                        }
                        if cp_lat_emergency > 0.0
                            && let Support::Single { swing, .. } = support
                        {
                            let i = gait.index_at(t);
                            let (_, plus) = landing_inward(&footsteps, i, swing);
                            let toward_stance = if plus { 1.0 } else { -1.0 };
                            let e_y = (xi.y - dcm_plan.reference(t).xi.y) * toward_stance;
                            let t1 = gait.slices[i].t1;
                            let ne = t + (t1 - t).max(0.0) * cp_lat_emergency_tmin;
                            if std::env::var("MPC_FS_DEBUG_LANDING").is_ok() && e_y > cp_lat_emergency {
                                println!("  [lat-emergency?] t={t:.3} e_y={e_y:+.3} plus={plus} i={i} lat_emg_slice={lat_emg_slice:?} cp_retimed_slice={cp_retimed_slice:?} ne={ne:.3} t1={t1:.3}");
                            }
                            // An emergency may shorten a step the contact
                            // plan already retimed; it only ever brings the
                            // end closer.
                            if e_y > cp_lat_emergency
                                && lat_emg_slice != Some(i)
                                && ne > t + dt
                                && t1 - ne > 0.02
                            {
                                gait.retime(i, ne);
                                n_lat_emg += 1;
                                lat_emg_slice = Some(i);
                                cp_retimed_slice = Some(i);
                                // Unlike the planner's retime, keep the
                                // landing extension: the foot must really
                                // be down before the next step starts
                                // (measured 2026-10-02 06:10: with the
                                // extension zeroed the schedule moved on
                                // with the swing still 35 mm up).
                                dcm_plan = DcmPlan::from_footsteps_rolled(&gait, &footsteps, z_lip, zmp_lat_scale, zmp_roll);
                                n_replan += 1;
                                if std::env::var("MPC_FS_DEBUG_LANDING").is_ok() {
                                    println!("  [lat-emergency] t={t:.3} e_y_toward_stance={e_y:+.3} new_end={ne:.3} (was {t1:.3})");
                                }
                            }
                        }
                        if solution.soft_fallback {
                            wrench_mpc_soft += 1;
                            if std::env::var("MPC_LEX_DEBUG_FB").is_ok() && wrench_mpc_soft <= 4 {
                                let plan = solution.cop_plan.as_ref().map(|p| (p[0].x, p[0].y));
                                println!(
                                    "  [lex-soft] t={t:.3} support={support:?} plan0={plan:?} contact0={:?} pos=({:.3},{:.3})/({:.3},{:.3}) min={:?} ref={:?} land={:?}",
                                    input.contact_schedule[0], input.contacts[0].position.x, input.contacts[0].position.y,
                                    input.contacts[1].position.x, input.contacts[1].position.y, input.min_fz_schedule[0],
                                    input.reference_fz_schedule[0], input.landing.map(|l| (l.foot, l.first_node))
                                );
                                if let Some(plan) = solution.cop_plan.as_ref() {
                                    println!("    plan: {}", plan.iter().map(|p| format!("({:.3},{:.3})", p.x, p.y)).collect::<Vec<_>>().join(" "));
                                    println!("    contact: {}", input.contact_schedule.iter().map(|c| format!("{}{}", c[0] as u8, c[1] as u8)).collect::<Vec<_>>().join(" "));
                                    println!("    act: {}", input.activation_schedule.iter().map(|a| format!("{:.2}/{:.2}", a[0], a[1])).collect::<Vec<_>>().join(" "));
                                    println!("    ref: {}", input.reference_fz_schedule.iter().map(|a| format!("{:.0}/{:.0}", a[0], a[1])).collect::<Vec<_>>().join(" "));
                                    println!("    min: {}", input.min_fz_schedule.iter().map(|a| format!("{:.0}/{:.0}", a[0], a[1])).collect::<Vec<_>>().join(" "));
                                }
                            }
                        }
                        if let Ok(tw) = std::env::var("MPC_LEX_DEBUG_T")
                            && let Ok(tw) = tw.parse::<f64>()
                            && (t - tw).abs() < 0.011
                            && let Some(plan) = solution.cop_plan.as_ref()
                        {
                            println!("  [lex] t={t:.3} xi0={:?} support={support:?}", input.dcm_now.map(|v| (v.x, v.y)));
                            for k in 0..plan.len() {
                                let xr = input.reference_dcm_schedule.as_ref().map(|v| v[k]);
                                let pb = input.reference_net_cop_schedule.as_ref().map(|v| v[k]);
                                println!(
                                    "    k{k} contact={:?} ref_fz={:?} min={:?} xi_ref=({:.3},{:.3}) pbar=({:.3},{:.3}) plan=({:.3},{:.3}) pos=({:.3},{:.3})/({:.3},{:.3})",
                                    input.contact_schedule[k], input.reference_fz_schedule[k], input.min_fz_schedule[k],
                                    xr.map_or(0.0, |v| v.x), xr.map_or(0.0, |v| v.y), pb.map_or(0.0, |v| v.x), pb.map_or(0.0, |v| v.y),
                                    plan[k].x, plan[k].y, input.contacts[0].position.x, input.contacts[0].position.y, input.contacts[1].position.x, input.contacts[1].position.y
                                );
                            }
                        }
                        if let Some(dp) = solution.landing_shift
                            && let Some(land) = landing
                            && !landing_timing.landing_closed(gait.index_at(t))
                        {
                            let i = gait.index_at(t);
                            if mpc_fs_slice != Some(i) {
                                mpc_fs_slice = Some(i);
                                mpc_fs_applied = na::Vector2::zeros();
                            }
                            let (d, total) = if mpc_footstep_increment {
                                (dp, mpc_fs_applied + dp)
                            } else {
                                (dp - mpc_fs_applied, dp)
                            };
                            mpc_fs_last = total;
                            if std::env::var("MPC_FS_DEBUG_LANDING").is_ok() {
                                println!(
                                    "  [fs-decision] t={t:.3} first_node={} prior=({:+.3},{:+.3}) dp=({:+.3},{:+.3}) total=({:+.3},{:+.3}) max={:?} min={:?} xi-ref=({:+.3},{:+.3})",
                                    land.first_node, land.prior[0], land.prior[1], dp.x, dp.y, total.x, total.y, land.max, land.min,
                                    xi.x - dcm_plan.reference(t).xi.x, xi.y - dcm_plan.reference(t).xi.y
                                );
                            }
                            if d.norm() > 1e-6 {
                                footsteps.shift_next(i, land.foot, d);
                                mpc_fs_applied = total;
                                dcm_plan = DcmPlan::from_footsteps_rolled(
                                    &gait,
                                    &footsteps,
                                    z_lip,
                                    zmp_lat_scale,
                                    zmp_roll,
                                );
                                n_replan += 1;
                            }
                        }
                        if std::env::var("MPC_TRAJ_DEBUG").is_ok() && t < 2.0 {
                            let xi0 = com.xy() + com_vel.xy() / omega;
                            let r0 = dcm_plan.reference(t);
                            print!("  [traj] t={t:.3} xi_meas=({:+.3},{:+.3}) xi_plan=({:+.3},{:+.3}) tgt=({:+.3},{:+.3}) |",
                                xi0.x, xi0.y, r0.xi.x, r0.xi.y, target_com.x, target_com.y);
                            for (k, [c, v]) in solution.trajectory.iter().enumerate() {
                                let xi = c.xy() + v.xy() / omega;
                                let rp = dcm_plan.reference(t + (k as f64 + 1.0) * stair_wrench_preview_dt);
                                print!(" k{k}: pred({:+.3},{:+.3}) plan({:+.3},{:+.3})", xi.x, xi.y, rp.xi.x, rp.xi.y);
                            }
                            println!();
                        }
                        // The horizon the terminal state sits at. `Solution`
                        // carries only the terminal momentum, not a per-node
                        // series, so the reference is the mean rate over it.
                        let horizon_t = stair_wrench_preview_dt
                            * stair_wrench_horizon as f64;
                        wrench_mpc_momentum_rate = (solution.terminal_momentum
                            - input.angular_momentum)
                            / horizon_t;
                        wrench_mpc_solves += 1;
                    }
                    None => {
                        wrench_mpc_failures += 1;
                    }
                }
                wrench_mpc_next = t + stair_wrench_dt;
            }
            if stair_wrench_accel && wrench_mpc_command.is_some() {
                // STAIR_WRENCH_ACCEL_XY: take only the HORIZONTAL acceleration
                // from the MPC and keep the WBC's own vertical reference.
                //
                // With MPC_SEED_PREVIOUS=2 the MPC's smoothness term is
                // anchored to the MEASURED load. Handing its full acceleration
                // to the CoM task closes a loop through the vertical axis:
                // a touchdown transient lowers the measured load, the MPC
                // (anchored to it) commands less total force, the CoM task
                // now asks for that as a downward acceleration, the robot
                // sinks, the measured load drops further. Measured
                // 2026-09-26 on the v1 baseline: a_z reference -0.9 at
                // 2.50 s, -9.9 m/s^2 (free fall) at 2.75 s with the MPC
                // asking 157 N total, CoM height 0.68 -> 0.30 m at 2.95 s.
                // The horizontal components carry the DCM-relevant content
                // and have no such loop.
                if stair_wrench_accel_xy {
                    a_com.x = wrench_mpc_acceleration.x;
                    a_com.y = wrench_mpc_acceleration.y;
                } else {
                    a_com = wrench_mpc_acceleration;
                }
            }
            // MPC_REACH_TRACK: the WBC's vertical PD follows the MPC's
            // predicted height (position and velocity at the end of node 0,
            // interpolated to now) instead of the heuristic reference.
            if mpc_reach_on
                && mpc_reach_track == 1
                && let Some((t0, c0, v0, tr)) = wrench_mpc_traj.as_ref()
                && let Some([c1, v1]) = tr.first()
            {
                let s = ((t - t0) / stair_wrench_preview_dt).clamp(0.0, 1.0);
                let z_m = c0.z + (c1.z - c0.z) * s;
                let zd_m = v0.z + (v1.z - v0.z) * s;
                a_com.z = zdd_ref + kp_com * (z_m - com.z) + kd_com * (zd_m - com_vel.z);
                mpc_reach_z = Some(z_m);
            }
        } else {
            wrench_mpc_command = None;
            wrench_command_context = None;
            wrench_mpc_next = t;
        }
        let p1 = bt::com(dyn_ctx.qddot(), &st.j_com, &st.djv_com, &a_com);

        // ---- P2: trunk upright -----------------------------------------
        let j_trunk =
            misarta::jacobian::compute_joint_jacobian_from_data(&rig.model, q, data, rig.trunk_mi);
        let dj_trunk = misarta::jacobian::compute_joint_jacobian_time_derivative(
            &rig.model,
            q,
            v,
            rig.trunk_mi,
        );
        let djv_trunk = &dj_trunk * v_dvec;
        let (roll, pitch, trunk_quat, trunk_omega_w) = if rig.trunk_from_base {
            let (r, p, _) = st.body_quat.euler_angles();
            (r, p, st.body_quat, st.v_ang_w)
        } else {
            let rot = misarta::se3::rotation_matrix(&data.oMi[rig.trunk_mi]);
            let omega = j_trunk.rows(0, 3) * v_dvec;
            let rotation = na::Rotation3::from_matrix_unchecked(rot);
            (
                rotation[(2, 1)].atan2(rotation[(2, 2)]),
                (-rotation[(2, 0)]).asin(),
                na::UnitQuaternion::from_rotation_matrix(&rotation),
                [omega[0], omega[1], omega[2]],
            )
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
            if matches!(support, Support::Single { .. }) {
                0.0
            } else {
                wz * (t_step / gp.t_ds.max(1e-6))
            }
        } else {
            wz
        };
        // Now an absolute target heading -- trunk_ori_ref forms the error.
        let yaw_ref = trunk_yaw.then_some(yaw_plan);
        // One world-frame rotation-vector error for all three rows. The
        // roll/pitch rows are frame-correct at any heading this way; the old
        // Euler form silently rotated its own correction away.
        let ori = bt::trunk_ori_ref(
            &trunk_quat,
            yaw_ref,
            &trunk_omega_w,
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

        // ---- centroidal angular momentum --------------------------------
        //
        // `A_G` and `Ȧ_G v` are recomputed here rather than cached: the CMM is
        // a function of q, and this loop's whole point is that q moves.
        let coupled_active = stair_wrench_mpc
            && stair_wrench_coupled
            && (stair_transfer_cmd.is_some() || wrench_mpc_apply)
            && wrench_mpc_command.is_some()
            && nc > 0;
        let p_mom = if use_mom || coupled_active {
            let cmm = misarta::centroidal::compute_centroidal_momentum_matrix(&rig.model, q);
            let dcmm_v = misarta::centroidal::compute_cmm_dot_times_v(&rig.model, q, v);
            let h_now = &cmm * v_dvec;
            // arms_neutral: 「腕が角運動量を生むな。脚の分には触るな」。
            //
            // 参照だけ差し替えるのでは表現できない ── ヤコビアンが全 CMM 行の
            // ままだと、式は ḣ_total = -kp·h_arms という意味を成さない目的になる
            // (実測: 腕の振れが 299.8 -> 318.2 deg と増えた)。腕の列だけを残した
            // CMM を渡し、状態も腕の寄与だけにする。
            //
            // Ȧ·v の腕成分は直接得られないのでゼロとする。減衰型の目的としては
            // 許容できる近似で、本来 A2 が h_ref(t) を供給すれば不要になる経路。
            let arm_cols: Vec<usize> = rig
                .actuated()
                .into_iter()
                .filter(|&(ji, _)| {
                    let n = &rig.robot.joints[ji].name;
                    n.contains("shoulder") || n.contains("elbow") || n.contains("wrist")
                })
                .map(|(_, vi)| vi)
                .collect();
            let arms_neutral = mom_ref_mode == "arms_neutral" && !arm_cols.is_empty();
            let (cmm_use, h_use, bias_use) = if arms_neutral {
                let mut masked = na::DMatrix::zeros(cmm.nrows(), cmm.ncols());
                for &vi in &arm_cols {
                    masked.set_column(vi, &cmm.column(vi));
                }
                let h_arm = &masked * v_dvec;
                let zero = na::DVector::zeros(cmm.nrows());
                (masked, h_arm, zero)
            } else {
                (
                    cmm.clone(),
                    h_now.clone(),
                    na::DVector::from_row_slice(dcmm_v.as_slice()),
                )
            };
            // `h_ref = h_now` cancels the P term so only the planned rate
            // remains; the masked `arms_neutral` mode keeps its own reference
            // (there `h_use` is the ARMS' contribution alone, and zero is the
            // right thing to ask of it).
            // Only while the MPC is actually running and its solution is
            // fresh. It runs only during a transfer (line ~4645), so outside
            // one the stored rate is stale and the reference falls back to
            // zero -- the previous behaviour -- rather than commanding a rate
            // the plan no longer holds.
            let use_mpc_ref = mom_ref_mpc
                && !arms_neutral
                && diag_mpc_time
                    .is_some_and(|last| t - last <= stair_wrench_max_age + 1e-9);
            let h_ref = if use_mpc_ref {
                [h_use[0], h_use[1], h_use[2]]
            } else {
                [0.0_f64; 3]
            };
            // A2 の計画参照(`h`/`dh` 5701 節点)が差さる口。現状ゼロ = P 項のみで従来と同一。
            // ここに時変参照を入れるには、この example が参照と同じ動作を実行している
            // 必要がある(いまは自前の DCM/足運び歩容なので、そのままでは接続できない)。
            let mut dh_ref = if use_mpc_ref {
                [
                    wrench_mpc_momentum_rate[0],
                    wrench_mpc_momentum_rate[1],
                    wrench_mpc_momentum_rate[2],
                ]
            } else {
                [0.0_f64; 3]
            };
            if arms_neutral && mom_arm_dcm != 0.0 {
                dh_ref[0] = (-mom_arm_dcm * (xi.y - r.xi.y))
                    .clamp(-mom_arm_dcm_max, mom_arm_dcm_max);
            }
            if arms_neutral && mom_arm_dcm_x != 0.0 {
                dh_ref[1] = (mom_arm_dcm_x * (xi.x - r.xi.x))
                    .clamp(-mom_arm_dcm_x_max, mom_arm_dcm_x_max);
            }
            let dh_ref = dh_ref;
            bt::angular_momentum(
                dyn_ctx.qddot(),
                &cmm_use,
                &bias_use,
                &h_use,
                &h_ref,
                &dh_ref,
                kp_mom,
                if coupled_active { [true; 3] } else { mom_axes },
                nv,
            )
        } else {
            None
        };

        // ---- hip strategy: centroidal moment pivot (MOM_CMP) --------------
        let in_stair_zone = match &stairs {
            Some(st) if st.riser.abs() > 0.005 => {
                let x = steps.mid_xy().x;
                (x >= st.start - stair_crouch_lead && x <= st.landing_start() + stair_crouch_tail)
                    || (st.down_n > 0
                        && x >= st.landing_end() - stair_crouch_lead
                        && x <= st.landing_end() + st.down_n as f64 * st.tread + stair_crouch_tail)
            }
            _ => false,
        };
        let p_cmp = if mom_cmp > 0.0
            && (!mom_cmp_ref_only || ref_mode.is_some())
            && (mom_cmp_stairs || !in_stair_zone)
            && zmp_deficit.norm() > mom_cmp_dead
        {
            let cmm = misarta::centroidal::compute_centroidal_momentum_matrix(&rig.model, q);
            let dcmm_v = misarta::centroidal::compute_cmm_dot_times_v(&rig.model, q, v);
            let h_now = &cmm * v_dvec;
            let mg = total_mass * G;
            let mut dh = [
                (-mom_cmp_sign * mom_cmp * mg * zmp_deficit.y).clamp(-mom_cmp_max, mom_cmp_max),
                (mom_cmp_sign * mom_cmp * mg * zmp_deficit.x).clamp(-mom_cmp_max, mom_cmp_max),
                0.0,
            ];
            for a in 0..2 {
                if mom_cmp_hmax > 0.0 && h_now[a].abs() > mom_cmp_hmax && dh[a] * h_now[a] > 0.0 {
                    dh[a] = 0.0;
                }
                dh[a] -= mom_cmp_kh * h_now[a];
            }
            n_mom_cmp_ticks += 1;
            mom_cmp_peak = mom_cmp_peak.max(dh[0].abs()).max(dh[1].abs());
            bt::angular_momentum(
                dyn_ctx.qddot(),
                &cmm,
                &na::DVector::from_row_slice(dcmm_v.as_slice()),
                &h_now,
                &[h_now[0], h_now[1], h_now[2]],
                &dh,
                0.0,
                [true, true, false],
                nv,
            )
        } else {
            None
        };

        // ---- arm hold ----------------------------------------------------
        //
        // Same task as posture, restricted to the arm rows. Rows outside the
        // filter stay all-zero with a zero reference, i.e. `0 = 0`, which the
        // solver satisfies for free -- so this really is "posture, arms only,
        // at its own gain".
        // Jab. The arm-hold reference stops being the seed and becomes a
        // function of time for ONE arm at a time, out and back along a raised
        // cosine so the reversal has no corner.
        //
        // The punching arm needs its ELBOW driven too, and Sec.38 measured that
        // holding both rows of both arms costs four cells. So the elbow is
        // added to the held set only for the arm that is currently punching,
        // and only while it is: the other arm keeps whatever
        // `ARM_HOLD_JOINTS` asked for.
        let (jab_side, jab_a) = if jab && t >= jab_start {
            let ph = (t - jab_start) / jab_period;
            let idx = ph.floor().max(0.0) as usize;
            let side = if jab_pattern
                .chars()
                .nth(idx % jab_pattern.chars().count())
                .is_some_and(|c| c.eq_ignore_ascii_case(&'l'))
            {
                0
            } else {
                1
            };
            (Some(side), 0.5 * (1.0 - (2.0 * PI * ph.fract()).cos()))
        } else {
            (None, 0.0)
        };
        let p_arm = if arm_hold {
            let jab_names: Option<(&str, &str)> = jab_side.map(|s| {
                if s == 0 {
                    ("left_shoulder_pitch_joint", "left_elbow_joint")
                } else {
                    ("right_shoulder_pitch_joint", "right_elbow_joint")
                }
            });
            let arms: Vec<(usize, usize)> = rig
                .actuated()
                .into_iter()
                .filter(|&(ji, _)| {
                    let n = rig.robot.joints[ji].name.as_str();
                    let base = match arm_hold_joints.as_str() {
                        "elbow" => n.contains("elbow"),
                        "shoulder" => n.contains("shoulder"),
                        _ => n.contains("shoulder") || n.contains("elbow"),
                    };
                    base || jab_names.is_some_and(|(_, e)| n == e)
                })
                .collect();
            // Reference = the seed, with the punching arm's two rows swept
            // toward the extended pose.
            let mut q_ref = rig.q_seed.clone();
            if let Some((sh, el)) = jab_names {
                for (name, target) in [(sh, jab_pitch), (el, jab_elbow)] {
                    if let Some(&ji) = rig.robot.joint_map.get(name) {
                        q_ref[ji] += jab_a * (target - q_ref[ji]);
                    }
                }
            }
            (!arms.is_empty()).then(|| {
                bt::posture(
                    dyn_ctx.qddot(),
                    &arms,
                    &rig.robot.joint_positions,
                    &q_ref,
                    v,
                    kp_arm,
                    kd_arm,
                    na_count,
                    nv,
                )
            })
        } else {
            None
        };
        // Pure DAMPING on the elbows: kd only, kp = 0, merged into the arm
        // level. The landing jerk of a stair swing whips the free forearm
        // into its flexion stop (30-35 ms, 1.3-1.7 mrad past the URDF limit
        // -- the last failing cell of the capability gate), and every
        // position-holding cure re-broke the walk the way Sec.38 measured.
        // A damping row asserts no target posture, so it has no null-space
        // demand to steal; it only bleeds the jolt.
        let mut p_arm = if elbow_kd > 0.0 {
            let elbows: Vec<(usize, usize)> = rig
                .actuated()
                .into_iter()
                .filter(|&(ji, _)| rig.robot.joints[ji].name.contains("elbow"))
                .collect();
            let damp = bt::posture(
                dyn_ctx.qddot(),
                &elbows,
                &rig.robot.joint_positions,
                &rig.q_seed,
                v,
                0.0,
                elbow_kd,
                na_count,
                nv,
            );
            Some(match p_arm {
                Some(pa) => pa + damp,
                None => damp,
            })
        } else {
            p_arm
        };

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
        let mut forces_nominal = bt::force_nominal(forces.size(), &shares, total_mass * G);
        if let Some(command) = wrench_mpc_command.as_ref() {
            if (stair_transfer_cmd.is_some() || wrench_mpc_apply) && nc > 0 {
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
                        let authority = mpc_sole_authority
                            && (!mpc_authority_ds_only || nc == 2);
                        let w = if authority { 1.0 } else { stair_wrench_blend };
                        let old = forces_nominal.rows(6 * slot, 6).into_owned();
                        let blended = old * (1.0 - w) + native * w;
                        forces_nominal.rows_mut(6 * slot, 6).copy_from(&blended);
                    }
                }
            }
        }
        // Hand this tick's requested split to the FFDC loop, which runs
        // before the contact level on the next tick.
        if stair_ffdc > 0.0 {
            let mut share = [0.0_f64; 2];
            for (slot, &side) in stance_sides.iter().enumerate() {
                if 6 * slot + 5 < forces_nominal.len() {
                    share[side] = forces_nominal[6 * slot + 5];
                }
            }
            ffdc_ref_share = (stance_sides.len() == 2).then_some(share);
        }
        let p_wrench_coupled = coupled_active.then(|| {
            tasks::regularize(&dyn_ctx.forces(), &forces_nominal)
                .weight(stair_wrench_level_weight / (total_mass * G).powi(2))
        });
        let p_fz_plan_coupled = if contact_fz_plan_weight > 0.0
            && contact_fz_plan_slew_bw > 0.0
            && touchdown_load_ramp.is_some()
            && nc == 2
            && let Some(command) = wrench_mpc_command.as_ref()
        {
            let body_weight = total_mass * G;
            let delta = contact_fz_plan_slew_bw * body_weight;
            let mut sel_fz = na::DMatrix::zeros(p0.sole_sel.len(), forces.size());
            let mut ref_fz = na::DVector::zeros(p0.sole_sel.len());
            for (slot, sel) in p0.sole_sel.iter().enumerate() {
                for j in 0..forces.size() {
                    sel_fz[(slot, j)] = sel[(5, j)] / body_weight;
                }
                let side = usize::from(stance[slot] != rig.foot_mi[0]);
                let desired = command[side][5].max(0.0);
                let target = wbc_fz_previous[side].map_or(desired, |previous| {
                    previous + (desired - previous).clamp(-delta, delta)
                });
                ref_fz[slot] = target / body_weight;
            }
            let expr = &sel_fz * &forces.as_affine();
            Some(tasks::track(&expr, &ref_fz).weight(contact_fz_plan_weight))
        } else {
            None
        };
        let p_post_settle_fz =
            if post_walk_settle.is_some() && nc == 2 && stair_post_walk_settle_fz_weight > 0.0 {
                let body_weight = total_mass * G;
                let mut sel_fz = na::DMatrix::zeros(2, forces.size());
                let mut ref_fz = na::DVector::zeros(2);
                for (slot, sel) in p0.sole_sel.iter().enumerate() {
                    for j in 0..forces.size() {
                        sel_fz[(slot, j)] = sel[(5, j)] / body_weight;
                    }
                    ref_fz[slot] = forces_nominal[6 * slot + 5] / body_weight;
                }
                let expr = &sel_fz * &forces.as_affine();
                Some(tasks::track(&expr, &ref_fz).weight(stair_post_walk_settle_fz_weight))
            } else {
                None
            };
        let p_reg = if coupled_active {
            tasks::regularize(dyn_ctx.tau(), &tau_gravity)
        } else {
            bt::regulariser(
                dyn_ctx.tau(),
                &tau_gravity,
                &dyn_ctx.forces(),
                &forces_nominal,
            )
        };

        // ---- P3: the foot that is NOT in contact ------------------------
        //
        // EVERY foot outside the contact set needs a Cartesian task. A foot
        // that is in neither is unconstrained except by the weak posture
        // level, and the QP will happily use the free leg as a reaction mass
        // for the CoM task: measured, a foot that had just landed was flung
        // to 80 mm and stayed there for a second while the robot toppled on
        // the other leg. "The schedule says double support" is not a task.
        let free_side: Option<usize> = (0..2).find(|s| !stance_sides.contains(s));
        let transfer_swing = stair_transfer_cmd.and_then(|(high, phase, rf)| {
            (phase == StairTransferPhase::Rise).then_some((1 - high, rf))
        });
        let mut swing_tgt = na::Vector3::zeros();
        let mut swing_cmd_acc_z = 0.0_f64;
        let mut p_swing = match (free_side, swing_lift_off) {
            (None, _) => None,
            // The scheduled swing, following its planned arc.
            (Some(side), Some(lift_off))
                if support.swing() == Some(side)
                    || transfer_swing.is_some_and(|(transfer_side, _)| transfer_side == side) =>
            {
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
                let plan_slice_idx = if transfer_swing.is_some_and(|(s, _)| s == side) {
                    gait.slices
                        .iter()
                        .enumerate()
                        .skip(slice_idx + 1)
                        .find(|(_, future)| future.support.swing() == Some(side))
                        .map_or(slice_idx, |(i, _)| i)
                } else {
                    slice_idx
                };
                let plan_xy = footsteps.at_slice(plan_slice_idx).sole[side];
                let extra = if adapt_live {
                    na::Vector2::zeros()
                } else {
                    adapt_now
                };
                // On stairs the surfaces differ in height; carry the CHANGE
                // in surface height on top of the measured lift-off z, so a
                // foot that started a few millimetres high keeps its offset
                // instead of being driven into the tread. Flat ground makes
                // dz zero and this is exactly the old `lift_off.z`.
                let dz = if stair_absolute_td && stairs.is_some() {
                    plan_xy.z - footsteps.at_slice(plan_slice_idx.saturating_sub(1)).sole[side].z
                } else {
                    stairs
                        .as_ref()
                        .map_or(0.0, |st| st.h(plan_xy.x + extra.x) - st.h(lift_off.x))
                };
                let touch_down = if stair_absolute_td && dz.abs() > 1e-9 {
                    // The target is a surveyed tread, not the measured height
                    // of the previous (possibly rocking) foot plus a riser.
                    let offset = foot_rot_at(side, footsteps.at_slice(plan_slice_idx).yaw[side])
                        * na::Vector3::new(prof.sole_centre_x, 0.0, -prof.sole_below_origin);
                    plan_xy + na::Vector3::new(extra.x, extra.y, 0.0) - offset
                } else {
                    // `plan_xy` is the SOLE centre; the swing task tracks the
                    // ANKLE. On a robot whose sole centre sits under the ankle
                    // the two coincide and the difference is exactly zero --
                    // kyo46rs has `sole_centre_x: 0.0`. G1 has 0.035, so the
                    // swing is commanded 35 mm beyond where the footstep plan
                    // wants the sole, and the landing check (which compares
                    // sole to sole, `preview.rs`) sees that as a miss it can
                    // never confirm: measured 2026-09-21, the sole lands 62 mm
                    // long against a 12 mm tolerance and the stair transfer
                    // therefore never arms.
                    //
                    // `STAIR_ABSOLUTE_TD` already subtracts this offset, but it
                    // also switches z from "lift-off height plus riser" to the
                    // surveyed tread, and that second change is what fells the
                    // G1 approach (measured: x = 0.21 m). This knob is the
                    // frame correction ALONE, in xy -- z is already an ankle
                    // height here, being lift-off plus a surface delta.
                    // OPT-IN: zero on kyo46rs either way, so the default keeps
                    // every existing result bit-identical.
                    let mut td = na::Vector3::new(
                        plan_xy.x + extra.x, plan_xy.y + extra.y, lift_off.z + dz);
                    if stair_td_sole_frame {
                        let off = foot_rot_at(side, footsteps.at_slice(plan_slice_idx).yaw[side])
                            * na::Vector3::new(prof.sole_centre_x, 0.0, 0.0);
                        td.x -= off.x;
                        td.y -= off.y;
                    }
                    td
                };
                // Flat ground keeps the shipped arc. A stair swing cannot:
                // its xy smoothstep advances while the sin^2 bump is still
                // low, and the TOE (sole half-length ahead of the centre)
                // reaches the riser FACE a few percent into the swing --
                // watched in render, the toe slams the wall at ~5 mm of
                // lift and the "climb" lands at the riser's base. So climb
                // in phases instead: rise to clearance FIRST, translate at
                // height, drop onto the tread last. Each phase edge uses
                // smoothstep, so velocity is zero at every joint and the
                // feedforward stays continuous.
                let (swing_fraction, swing_duration) = if let Some((transfer_side, rf)) =
                    transfer_swing
                    && transfer_side == side
                {
                    (rf.alpha, stair_rise_time)
                } else if com_replan {
                    landing_timing
                        .clock
                        .filter(|(i, _, _)| *i == slice_idx)
                        .map(|(_, start, duration)| {
                            (((t - start) / duration).clamp(0.0, 1.0), duration)
                        })
                        .unwrap_or((slice_frac, slice.duration()))
                } else if let Some((t0, dur)) = contact_phase.frozen_clock(slice_idx) {
                    // A late-contact extension must not rewind the swing:
                    // keep the ORIGINAL clock and hold at the endpoint
                    // (frac clamps to 1) while the schedule waits for load.
                    ((((t - t0) / dur).clamp(0.0, 1.0)), dur)
                } else {
                    (slice_frac, slice.duration())
                };
                let (swing_fraction, swing_duration) = if first_post_swing_slice == Some(slice_idx)
                    && first_post_swing_fraction < 1.0
                    && dz.abs() <= 0.005
                    && !stair_swing_profile
                    && transfer_swing.is_none()
                {
                    (
                        (swing_fraction / first_post_swing_fraction).min(1.0),
                        swing_duration * first_post_swing_fraction,
                    )
                } else {
                    (swing_fraction, swing_duration)
                };
                let (mut tgt, mut tgt_v, tgt_a) =
                    if (dz.abs() > 1e-9 || stair_swing_profile) && stair_swing_ff {
                        articara::biped::gait::stair_swing_sink(
                            lift_off,
                            touch_down,
                            lift_h_g.at(run_phase),
                            swing_fraction,
                            swing_duration,
                            stair_swing_windows,
                            stair_td_sink,
                        )
                    } else if dz.abs() > 1e-9 || stair_swing_profile {
                        let f = swing_fraction;
                        let dur = swing_duration;
                        let ss = |u: f64| -> (f64, f64) {
                            let u = u.clamp(0.0, 1.0);
                            (u * u * (3.0 - 2.0 * u), 6.0 * u * (1.0 - u))
                        };
                        let z_max = lift_off.z.max(touch_down.z) + lift_h_g.at(run_phase);
                        // rise [0,0.30] -> travel [0.25,0.70] -> drop [0.60,0.95].
                        // The drop OVERSHOOTS the surface by a few millimetres
                        // and ends before the slice does: a profile that arrives
                        // exactly at the tread with exactly zero velocity at
                        // frac = 1.0 leaves no margin for tracking lag, and the
                        // measured result was a foot still hovering when the
                        // schedule declared double support -- the anchor then
                        // slams it down (197 N spike) and the step after dies.
                        // How far INTO the tread the default stair swing aims.
                        // Historically 4 mm. Measured 2026-09-24: with 4 mm the
                        // foot rests on the tread carrying a median of 10 N for
                        // 1.4-2.8 s (the task still holds the leg's weight) and
                        // the landing gate (17 N) is met only by impact chatter;
                        // the feed-forward profile with 15 mm loaded it at
                        // 102 N within 0.07 s. Knob so the depth can be set on
                        // THIS profile; default reproduces the constant.
                        let z_land = touch_down.z - stair_land_sink;
                        let (su, dsu) = ss((f - 0.25) / 0.45);
                        let (ru, dru) = ss(f / 0.30);
                        let (du, ddu) = ss((f - 0.60) / 0.35);
                        let xy = lift_off.xy() + (touch_down.xy() - lift_off.xy()) * su;
                        let z = lift_off.z + (z_max - lift_off.z) * ru + (z_land - z_max) * du;
                        let vxy = (touch_down.xy() - lift_off.xy()) * (dsu / 0.45 / dur);
                        let vz = (z_max - lift_off.z) * (dru / 0.30 / dur)
                            + (z_land - z_max) * (ddu / 0.35 / dur);
                        (
                            na::Vector3::new(xy.x, xy.y, z),
                            na::Vector3::new(vxy.x, vxy.y, vz),
                            if tracking_cubic_ff {
                                // Exact second derivative of the unchanged cubic profile.
                                let dd = |u: f64, span: f64| {
                                    if u > 0.0 && u < 1.0 {
                                        (6.0 - 12.0 * u) / (span * dur).powi(2)
                                    } else {
                                        0.0
                                    }
                                };
                                let axy =
                                    (touch_down.xy() - lift_off.xy()) * dd((f - 0.25) / 0.45, 0.45);
                                let az = (z_max - lift_off.z) * dd(f / 0.30, 0.30)
                                    + (z_land - z_max) * dd((f - 0.60) / 0.35, 0.35);
                                na::Vector3::new(axy.x, axy.y, az)
                            } else {
                                na::Vector3::zeros()
                            },
                        )
                    } else {
                        (
                            swing_position(
                                lift_off,
                                touch_down,
                                lift_h_g.at(run_phase),
                                swing_fraction,
                            ),
                            swing_velocity(
                                lift_off,
                                touch_down,
                                lift_h_g.at(run_phase),
                                swing_fraction,
                                swing_duration,
                            ),
                            na::Vector3::zeros(),
                        )
                    };
                if resume_ss_dcm_hold.is_some_and(|(_, receiver, _, _)| receiver == side) {
                    tgt.z -= stair_resume_ss_sink;
                }
                if let Some(mode) = ref_mode
                    && let Some(lib) = &ref_lib
                    && ref_swing
                {
                    // Follow the reference's sole trajectory for this foot
                    // (the reference's right foot is the one that swings
                    // first). Ankle target = sole centre minus the sole offset.
                    let ref_side = if side == mode.stance { 0 } else { 1 };
                    // The CoM dynamics ran on during the detection delay, but
                    // the swing foot did not: compress the first swing's path
                    // into the time that remains instead of skipping its start.
                    let elapsed = t - mode.t0;
                    let first = (lib.phases[0] - mode.tau0).max(0.05);
                    let warp = if ref_side == 1 && elapsed < first { lib.phases[0] / first } else { 1.0 };
                    let tau = if ref_side == 1 && elapsed < first { elapsed * warp } else { elapsed + mode.tau0 };
                    let sole_w = mode.world(lib, lib.sole_at(ref_side, tau));
                    let sole_w1 = mode.world(lib, lib.sole_at(ref_side, tau + dt * warp));
                    let offset = foot_rot_at(side, footsteps.at_slice(slice_idx).yaw[side])
                        * na::Vector3::new(prof.sole_centre_x, 0.0, -prof.sole_below_origin);
                    tgt = sole_w - offset;
                    tgt_v = (sole_w1 - sole_w) / dt;
                }
                if landing_timing.probe_depth > 0.0
                    && landing_timing.clock.is_some_and(|(ci, _, _)| ci == slice_idx)
                {
                    tgt.z -= landing_timing.probe_depth;
                }
                swing_tgt = tgt;
                swing_cmd_acc_z = tgt_a.z * tracking_ff_scale
                    + kp_sw_z * (tgt.z - pos.z)
                    + kd_sw * (tgt_v.z - vel.z);
                let om3 = &jf.rows(0, 3).into_owned() * v_dvec;
                Some(bt::swing_with_pose_acceleration(
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
                    if transfer_swing.is_some_and(|(transfer_side, _)| transfer_side == side)
                        || (stair_resume_sole_pose && stair_transfer_completed > 0)
                        // A swing that CHANGES HEIGHT -- the step onto a tread
                        // -- whether or not a transfer ever armed.
                        //
                        // OPT-IN. Measured 2026-09-22 on G1 with the default
                        // four-row `XyzYaw` swing: the foot reaches the tread
                        // with its heel 33 mm past the nosing and nothing
                        // holding its pitch, so the heel becomes a pivot --
                        // ankle pitch runs -18 to -43 deg in 700 ms, the
                        // measured CoP sits at -87 mm on an 85 mm half-sole
                        // (off the heel), the load never exceeds 90 N of a
                        // 334 N body, and the landing check correctly refuses
                        // to confirm a foot that is not on the surface.
                        // Constraining roll and pitch for EVERY swing is not
                        // the answer: `SWING_ROT=3` costs a row of null space
                        // the CEM-tuned gains do not have, and the same recipe
                        // then falls at 3.37 s on flat ground, two steps in.
                        // STAIR_RISE_SOLE_POSE_MIN_DZ: only pay the two extra
                        // swing rows for a step that needs them. At a 1 cm
                        // riser the free-arm recipe (arm hold above swing =
                        // less null space) went 7/24 with this on and 18/24
                        // with it off, while it is load-bearing for deep
                        // treads (measured 2026-09-26, Sec.54). 0 = any dz,
                        // the shipped behaviour.
                        || (stair_rise_sole_pose
                            && dz.abs() > 1e-9
                            && dz.abs() >= stair_rise_sole_pose_min_dz)
                    {
                        // Full sole orientation is essential only for the deep
                        // stair transfer. Applying it to the approach made the
                        // feet collide, while leaving roll free lets the swing
                        // ankle run to its stop and throws the foot sideways.
                        bt::SwingAxes::XyzRollPitch
                    } else {
                        match swing_rot_mode {
                            0 => bt::SwingAxes::Xyz,
                            2 => bt::SwingAxes::Pose,
                            3 => bt::SwingAxes::XyzRollPitch,
                            _ => bt::SwingAxes::XyzYaw,
                        }
                    },
                    {
                        let mut rot_tgt =
                            foot_rot_at(side, footsteps.at_slice(plan_slice_idx).yaw[side]);
                        if stair_desc_toe > 0.0 && dz < -1e-9 {
                            // Start the toe-down EARLY. Measured on the
                            // one-step descent: ramping it from the drop
                            // phase only reached +16.6 deg by the time the
                            // sole met the nosing, and the heel needs >=35
                            // deg of pitch to clear a 52 mm riser with a
                            // 98 mm foot -- so the foot landed perched on
                            // the edge of the step it was leaving, every
                            // time. `STAIR_DESC_TOE_START` is the swing
                            // fraction at which the pitch begins.
                            let w3 = env_f64("STAIR_DESC_TOE_START", stair_swing_windows[3]);
                            let u = ((swing_fraction - w3) / (0.95 - w3).max(1e-6)).clamp(0.0, 1.0);
                            let s_ = u * u * (3.0 - 2.0 * u);
                            rot_tgt = rot_tgt
                                * na::Rotation3::from_axis_angle(
                                    &na::Vector3::y_axis(),
                                    stair_desc_toe * s_,
                                )
                                .into_inner();
                        }
                        Some((
                            misarta::se3::rotation_matrix(&data.oMi[mi]),
                            rot_tgt,
                            na::Vector3::new(om3[0], om3[1], om3[2]),
                            kp_sw_rot,
                            // Extra rotational damping through the DROP phase of
                            // a stair swing: the touchdown slap is what pushes
                            // the landing ankle's difference-velocity past its
                            // URDF limit (+2..3 rad/s in the failing gate cell),
                            // and it is a rate spike, not a pose error.
                            if (dz.abs() > 1e-9 || stair_swing_profile)
                                && swing_fraction > stair_swing_windows[3]
                            {
                                kd_sw_rot * land_kd_rot_boost
                            } else {
                                kd_sw_rot
                            },
                        ))
                    },
                    &(tgt_a * tracking_ff_scale),
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
                swing_cmd_acc_z = kp_sw_z * (tgt.z - pos.z) - kd_sw * vel.z;
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
                    if transfer_swing.is_some_and(|(transfer_side, _)| transfer_side == side)
                        || (stair_resume_sole_pose && stair_transfer_completed > 0)
                    {
                        // Full sole orientation is essential only for the deep
                        // stair transfer. Applying it to the approach made the
                        // feet collide, while leaving roll free lets the swing
                        // ankle run to its stop and throws the foot sideways.
                        bt::SwingAxes::XyzRollPitch
                    } else {
                        match swing_rot_mode {
                            0 => bt::SwingAxes::Xyz,
                            2 => bt::SwingAxes::Pose,
                            3 => bt::SwingAxes::XyzRollPitch,
                            _ => bt::SwingAxes::XyzYaw,
                        }
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

        // Keep the unweighted task for post-solve diagnostics. `p_swing` may
        // be moved into an earlier recovery level below, so retain a clone
        // before assembling the hierarchy.
        let p_swing_diag = p_swing.clone();
        // Preserve original physical rows before task weighting / HQP slack.
        let qp_physical =
            (qp_diag_dir.is_some() || qp_gate || qp_runtime_log.is_some()).then(|| {
                let mut units = vec!["N*m"; 2 * na_count];
                let mut scales = vec![(total_mass * G * sole_half_l).max(1.0); 2 * na_count];
                if jlim {
                    units.extend(vec!["rad/s^2"; 2 * na_count]);
                    scales.extend(vec![1.0; 2 * na_count]);
                }
                if !contact_cfg.no_patch {
                    for _ in &stance_sides {
                        units.extend([
                            "N", "N", "N", "N", "N", "N", "N*m", "N*m", "N*m", "N*m", "N*m", "N*m",
                        ]);
                        scales.extend(vec![(total_mass * G).max(1.0); 6]);
                        scales.extend(vec![(total_mass * G * sole_half_l).max(1.0); 6]);
                    }
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
        let mut level_names: Vec<&str> = Vec::new();
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
        let managed_recovery = (stair_wrench_mpc
            && stair_managed_contact
            && stair_transfer_cmd.is_some()
            && free_side.is_some())
            || (stair_resume_swing_priority
                && stair_transfer_completed > 0
                && stair_transfer_cmd.is_none()
                && free_side.is_some());
        if managed_recovery {
            if let Some(ps) = p_swing.take() {
                levels.push(ps);
                level_names.push("contact recovery");
            }
        }
        let mut p1_centroidal = if coupled_active {
            p1.weight(stair_wrench_com_weight)
        } else {
            p1
        };
        let mut mom_pending = p_mom;
        if let Some(pw) = p_wrench_coupled {
            p1_centroidal = p1_centroidal + pw;
            if let Some(pm) = mom_pending.take() {
                p1_centroidal = p1_centroidal + pm.weight(stair_wrench_mom_weight);
            }
            if use_trunk && stair_wrench_trunk_coupled {
                p1_centroidal = p1_centroidal + p2.clone().weight(stair_wrench_trunk_weight);
            }
        }
        if let Some(pf) = p_fz_plan_coupled {
            p1_centroidal = p1_centroidal + pf;
        }
        if let Some(pf) = p_post_settle_fz {
            p1_centroidal = p1_centroidal + pf;
        }
        if ds_fz_floor_weight > 0.0 && nc == 2 {
            let body_weight = total_mass * G;
            for slot in 0..2 {
                if forces_nominal[6 * slot + 5] >= ds_fz_floor_bw * body_weight {
                    continue;
                }
                let mut selector = na::DMatrix::zeros(1, forces.size());
                for j in 0..forces.size() {
                    selector[(0, j)] = p0.sole_sel[slot][(5, j)] / body_weight;
                }
                let target = na::DVector::from_element(1, ds_fz_floor_bw);
                let expr = &selector * &forces.as_affine();
                p1_centroidal =
                    p1_centroidal + tasks::track(&expr, &target).weight(ds_fz_floor_weight);
            }
        }
        if let Some(pc) = p_cmp {
            p1_centroidal = p1_centroidal + pc.weight(mom_cmp_w);
        }
        let trunk_couple_now = use_trunk
            && stair_trunk_couple > 0.0
            && (stair_transfer_cmd.is_some() || t < stair_trunk_post_until)
            && !(coupled_active && stair_wrench_trunk_coupled);
        if trunk_couple_now {
            p1_centroidal = p1_centroidal + p2.clone().weight(stair_trunk_couple);
        }
        if stair_resume_swing_couple > 0.0
            && stair_transfer_completed > 0
            && stair_transfer_cmd.is_none()
            && free_side.is_some()
        {
            if let Some(ps) = p_swing.take() {
                p1_centroidal = p1_centroidal + ps.weight(stair_resume_swing_couple);
            }
        }
        // 運動量と腕を重心レベルへ合流(BDI/IHMC 方式)。既定 0 で無効。
        let mut merged_mom = false;
        if mom_merge > 0.0 {
            if let Some(pm) = mom_pending.take() {
                p1_centroidal = p1_centroidal + pm.weight(mom_merge);
                merged_mom = true;
            }
        }
        let mut merged_arm = false;
        if arm_merge > 0.0 {
            if let Some(pa) = p_arm.take() {
                p1_centroidal = p1_centroidal + pa.weight(arm_merge);
                merged_arm = true;
            }
        }
        levels.push(p1_centroidal);
        level_names.push(match (coupled_active, merged_mom, merged_arm) {
            (true, _, _) => "com+momentum+wrench",
            (false, true, true) => "com+momentum+arm",
            (false, true, false) => "com+momentum",
            (false, false, true) => "com+arm",
            (false, false, false) => "com",
        });
        let mut p2_late = None;
        if use_trunk && !(coupled_active && stair_wrench_trunk_coupled) && !trunk_couple_now {
            if trunk_late {
                p2_late = Some(p2);
            } else {
                levels.push(p2);
                level_names.push("trunk");
            }
        }
        let mut push_mom = |at: &str,
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
        if let Some(pt) = p2_late {
            levels.push(pt);
            level_names.push("trunk(late)");
        }
        // Load-split damping (see CONTACT_FZ_RATE above). Row 5 of each
        // sole selection is that foot's normal force.
        if contact_fz_rate > 0.0 && stance.len() > 1 && !p0.sole_sel.is_empty() {
            let mut sel_fz = na::DMatrix::zeros(p0.sole_sel.len(), forces.size());
            let mut ref_fz = na::DVector::zeros(p0.sole_sel.len());
            for (slot, sel) in p0.sole_sel.iter().enumerate() {
                for j in 0..forces.size() {
                    sel_fz[(slot, j)] = sel[(5, j)] * contact_fz_rate;
                }
                let side = usize::from(stance[slot] != rig.foot_mi[0]);
                // SECOND difference, not first: the reference is the linear
                // extrapolation of the last two ticks, so a steady load
                // ramp costs nothing and only a CHANGE of rate is damped.
                // Tracking fz_prev directly (first difference) was tried and
                // resisted the intended transfer -- every weight fell
                // EARLIER than the undamped baseline.
                ref_fz[slot] = (2.0 * fz_prev[side] - fz_prev2[side]).max(0.0) * contact_fz_rate;
            }
            let expr = &sel_fz * &forces.as_affine();
            levels.push(tasks::track(&expr, &ref_fz));
            level_names.push("load split");
        }
        if stair_ankle_reg > 0.0 && stair_transfer_cmd.is_some() && nc > 0 {
            // Drive the sole-frame roll/pitch moments toward zero, i.e. ask
            // for the CoP at the sole centre, and let everything above
            // override it where balance actually needs the moment.
            let mut pick = na::DMatrix::zeros(2, 6);
            pick[(0, 0)] = 1.0;
            pick[(1, 1)] = 1.0;
            let mut ankle = Task::empty(nd);
            for slot in 0..stance_sides.len() {
                let sole = &p0.sole_sel[slot] * &forces.as_affine();
                ankle = ankle + Task::soft_eq(&(&pick * &sole));
            }
            let scale = (total_mass * G * sole_half_l).powi(2).max(1e-9);
            levels.push(ankle.weight(stair_ankle_reg / scale));
            level_names.push("ankle moment");
        }
        levels.push(p_reg);
        level_names.push("regularise");
        // Weighted single QP (WBC_STRATEGY=weighted): each level below the
        // hard physics level gets a PER-TASK weight, because a geometric
        // ratio cannot express "swing as strong as the CoM, regularisation
        // far weaker" (measured 2026-09-24: ratios 1..1000 all fell in the
        // first step or before it). Override one with WBCW_<NAME> where NAME
        // is the level name upper-cased with non-alphanumerics as '_'
        // (e.g. WBCW_COM, WBCW_SWING, WBCW_POSTURE_MOMENTUM).
        // WBC_WEIGHTED_OFF_STAIRS=1: the weighted QP only while the feet are
        // outside the stair zone (the zone STAIR_ZONE_CROUCH uses); inside
        // it the strict cascade the stair machinery was built on. Measured
        // 2026-09-24: weighted everywhere = 112/112 on the flat push bench
        // but 0/192 on the stairs.
        let weighted_now = weighted_levels
            && !(weighted_off_stairs
                && stairs.as_ref().is_some_and(|st| {
                    let x = steps.mid_xy().x;
                    x >= st.start - stair_crouch_lead && x <= st.landing_start() + weighted_zone_tail
                }));
        let mut cfg_now = cfg.clone();
        if weighted_levels && !weighted_now {
            cfg_now.strategy = misa_wbc::HqpStrategy::NullSpace;
        }
        if weighted_now {
            for (k, lvl) in levels.iter_mut().enumerate().skip(1) {
                let name = level_names.get(k).copied().unwrap_or("");
                let key: String = name
                    .chars()
                    .map(|c| if c.is_ascii_alphanumeric() { c.to_ascii_uppercase() } else { '_' })
                    .collect();
                let default = match name {
                    "com" | "com+momentum" | "com+arm" | "com+momentum+arm" | "com+momentum+wrench" => 1.0,
                    "swing" | "contact recovery" => 1.0,
                    "trunk" | "trunk(late)" => 0.3,
                    "momentum" => 0.1,
                    "posture" | "posture+momentum" => 0.01,
                    "regularise" => 1e-4,
                    _ => 0.1,
                };
                let wk = env_f64(&format!("WBCW_{key}"), default);
                let taken = std::mem::replace(lvl, misa_wbc::Task::empty(nd));
                *lvl = taken.weight(wk);
            }
        }

        let qp_structure = (
            stance_sides.clone(),
            levels
                .iter()
                .map(|v| (v.a.ncols(), v.a.nrows(), v.d.nrows()))
                .collect::<Vec<_>>(),
        );
        let qp_reset = qp_previous_structure
            .as_ref()
            .is_some_and(|old| old != &qp_structure);
        // Drop the primal-dual warm start every tick.
        //
        // The stateful solver keeps a workspace per level and only resets it
        // when the QP STRUCTURE changes, so a working set from the previous
        // tick is carried into a problem whose data has moved. Measured
        // 2026-09-25: the HQP reports `Infeasible` on 5-56 ticks per run, and
        // WHICH controller trips it predicts which one falls -- across 192
        // paired members, the side with ~50 infeasible ticks is the side that
        // goes down. This isolates the warm start as the cause, or clears it.
        if qp_reset || qp_reset_every_tick {
            solver.reset();
        }
        qp_previous_structure = Some(qp_structure);
        let qp_solve_start = std::time::Instant::now();
        let mut sol = solver
            .solve(&levels, &cfg_now)
            .unwrap_or_else(|e| panic!("wbc solve failed at t={t:.3}: {e}"));
        let qp_solve_ns = qp_solve_start.elapsed().as_nanos();
        // WBC_LEVEL_DEBUG_T=<t>: print each level's least-squares residual
        // at the ticks within 11 ms of t (which task the solve could not
        // satisfy, and by how much).
        if let Ok(tw) = std::env::var("WBC_LEVEL_DEBUG_T")
            && let Ok(tw) = tw.parse::<f64>()
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
        let mut repair_applied = false;
        let mut repair_scaled_delta = 0.0_f64;
        let repair_start = std::time::Instant::now();
        if repair_original
            && (matches!(sol.status, misa_wbc::SolveStatus::Optimal)
                || matches!(sol.status, misa_wbc::SolveStatus::Degraded { level, .. }
                if level >= prefix_min_level && prefix_min_level < 999))
        {
            if let Some((eom, d, f, _, scales)) = &qp_physical {
                let iq_violation = (d * &sol.x - f)
                    .iter()
                    .zip(scales)
                    .fold(0.0_f64, |max, (v, scale)| max.max(*v / scale));
                if iq_violation > 1e-6 && iq_violation.is_finite() {
                    let mut h = na::DMatrix::identity(sol.x.len(), sol.x.len());
                    let mut variable_scales = vec![5.0_f64; sol.x.len()];
                    let force_var = dyn_ctx.layout().var("f");
                    for i in 0..force_var.size() {
                        variable_scales[force_var.offset() + i] = if i % 6 < 3 {
                            (total_mass * G * sole_half_l).max(1.0)
                        } else {
                            (total_mass * G).max(1.0)
                        };
                    }
                    let tau_var = dyn_ctx.layout().var("tau");
                    for i in 0..tau_var.size() {
                        variable_scales[tau_var.offset() + i] = rig.torque_max[i].max(1.0);
                    }
                    for i in 0..sol.x.len() {
                        h[(i, i)] = 1.0 / variable_scales[i].powi(2);
                    }
                    let c = -&h * &sol.x;
                    let repair_cfg = misa_wbc::QpConfig {
                        solver: misa_wbc::QpSolver::Clarabel,
                        feasibility_tol: 1e-9,
                        optimality_tol: 1e-8,
                        absolute_gap: true,
                        ..Default::default()
                    };
                    let repaired = misa_wbc::solve_qp(
                        &h,
                        &c,
                        Some(&eom.a),
                        Some(&eom.b),
                        Some(d),
                        Some(f),
                        None,
                        &repair_cfg,
                    );
                    let qualified = repaired.formally_solved()
                        && misa_wbc::qp_check::check(
                            &h,
                            &c,
                            Some((&eom.a, &eom.b)),
                            Some((d, f)),
                            &repaired,
                        )
                        .accepted(1e-6);
                    if qualified {
                        repair_scaled_delta = (&repaired.x - &sol.x)
                            .iter()
                            .zip(&variable_scales)
                            .map(|(delta, scale)| (delta / scale).abs())
                            .fold(0.0_f64, f64::max);
                        if repair_scaled_delta <= 0.01 {
                            sol.x = repaired.x;
                            repair_applied = true;
                        }
                    }
                }
            }
        }
        let repair_ns = repair_start.elapsed().as_nanos();
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
        let prefix_eligible = qp_gate
            && (cfg.hard_first_inequalities || repair_original)
            && prefix_min_level < 999
            && qp_physical_violation <= 1e-6
            && !raw_almost
            && matches!(sol.status, misa_wbc::SolveStatus::Degraded { level, .. }
                if level >= prefix_min_level)
            && sol
                .level_statuses
                .iter()
                .take(usize::from(eom_hard) + 1)
                .all(|s| *s == misa_wbc::QpStatus::Optimal);
        let qp_rejected = qp_gate
            && (qp_physical_violation > 1e-6
                || (!prefix_eligible && !matches!(sol.status, misa_wbc::SolveStatus::Optimal))
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
        {
            let qdd = dyn_ctx.qddot().eval(&sol.x);
            let jr = j_trunk.rows(0, 2).into_owned();
            let acc = &jr * &qdd + djv_trunk.rows(0, 2).into_owned();
            trunk_cmd_rp = rp_ref;
            trunk_acc_rp = [acc[0], acc[1]];
        }
        let degraded = !matches!(sol.status, misa_wbc::SolveStatus::Optimal);
        let qp_capture_timer = std::time::Instant::now();
        if let Some(dir) = &qp_diag_dir {
            let legacy_failure = t >= 2.0
                && qp_diag_count < 8
                && matches!(sol.status,
                misa_wbc::SolveStatus::Degraded { level, .. } if level_names.get(level) == Some(&"swing"));
            let continuous = qp_capture_all
                && t >= qp_capture_start
                && t <= qp_capture_end
                && tick % qp_capture_stride == 0
                && qp_diag_count < qp_capture_max;
            if continuous || (!qp_capture_all && legacy_failure) {
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
                let dump = serde_json::json!({"schema_version":2,"t":t,"tick":tick,"ctrl_dt":dt,
                    "status":format!("{:?}",sol.status),"backend":format!("{:?}",cfg.backend),
                    "layer_statuses":sol.level_statuses.iter().map(|s| format!("{s:?}")).collect::<Vec<_>>(),
                    "layer_raw_statuses":sol.level_diagnostics.iter().map(|d| d.as_ref().map(|d| &d.raw_status)).collect::<Vec<_>>(),
                    "prox_weight":prox_weight,"tasks":tasks,"solution":sol.x.as_slice(),"variables":variables,
                    "contact_ids":stance_sides,"planned_support":format!("{:?}",support),"phase":format!("{:?}",stair_transfer_cmd.map(|(_,p,_)|p)),
                    "measured_fz_N":measured_pre,"physical_groups":physical,"solve_ns":qp_solve_ns,
                    "check_ns":qp_check_ns,"normalized_physical_violation":qp_physical_violation});
                std::fs::create_dir_all(dir).expect("QP_DIAG_DIR");
                std::fs::write(
                    format!("{dir}/tick_{qp_diag_count:06}.json"),
                    serde_json::to_vec(&dump).unwrap(),
                )
                .expect("write QP snapshot");
                qp_diag_count += 1;
            }
        }
        let qp_capture_ns = qp_capture_timer.elapsed().as_nanos();
        let swing_task_residual_rms = p_swing_diag.as_ref().map_or(0.0, |task| {
            if task.a.nrows() == 0 {
                0.0
            } else {
                (&task.a * &sol.x - &task.b).norm() / (task.a.nrows() as f64).sqrt()
            }
        });
        // Which ROW of the swing task is being refused. `z` is the last row
        // in every `SwingAxes` layout, and after the ascent the step that
        // never lands is exactly the one whose z target is descending.
        let (swing_res_z, swing_res_rows) = p_swing_diag.as_ref().map_or((0.0, 0.0), |task| {
            if task.a.nrows() == 0 {
                (0.0, 0.0)
            } else {
                let r = &task.a * &sol.x - &task.b;
                (r[r.len() - 1], task.a.nrows() as f64)
            }
        });
        // Leg reach this tick. `steps.sole[side]` is the PLANNED sole, so the
        // measured sole comes from FK like everywhere else in this loop.
        let mut reach_used = [0.0_f64; 2];
        let mut hip_z = [0.0_f64; 2];
        let mut stride_max_now = 0.0_f64;
        if let Some(mis) = hip_mi
            && leg_reach.max > 0.0
        {
            let mut soles = [na::Vector3::zeros(); 2];
            for side in 0..2 {
                let hip = misarta::se3::translation(&data.oMi[mis[side]]);
                let mut sole = misarta::se3::translation(&data.oMi[rig.foot_mi[side]]);
                sole.z -= prof.sole_below_origin;
                reach_used[side] = leg_reach.used(&hip, &sole);
                hip_z[side] = hip.z;
                soles[side] = sole;
            }
            // The stride the two legs allow at this instant, with the hip
            // midway (see reach::max_stride).
            let hip_mid = 0.5 * (hip_z[0] + hip_z[1]);
            stride_max_now = leg_reach.max_stride(hip_mid, soles[0].z, soles[1].z);
        }
        // `swing_res_z = achieved - commanded`: misa-wbc Cartesian residual.
        let swing_achieved_acc_z = swing_cmd_acc_z + swing_res_z;
        let (swing_pos_err_xy, swing_pos_err_z) = free_side.map_or((0.0, 0.0), |side| {
            let pos = misarta::se3::translation(&data.oMi[rig.foot_mi[side]]);
            (
                (pos.xy() - swing_tgt.xy()).norm(),
                (pos.z - swing_tgt.z).abs(),
            )
        });
        tally.observe(&sol.status, t, tick, nc, &level_names);
        let extracted = dyn_ctx.extract(&sol.x);

        let mut cop_qp = [[0.0_f64; 3]; 2];
        let mut wbc_net_cop_acc = [f64::NAN; 3];
        let mut f_qp_w = [[0.0_f64; 3]; 2];
        let mut wbc_fz_now: [Option<f64>; 2] = [None; 2];
        let mut cop_use = 0.0_f64;
        for (slot, mi) in stance.iter().copied().enumerate() {
            let side = rig.side_of(mi);
            let w = &p0.sole_sel[slot] * &extracted.forces;
            if w[5].is_finite() {
                wbc_fz_now[side] = Some(w[5].max(0.0));
            }
            if let Some((cx, cy, fz)) = cop_from_sole_wrench(&w) {
                if fz > 1e-6 {
                    cop_qp[side] = [cx, cy, fz];
                    // Same net pressure point as `diag_mpc_net_cop`, for the
                    // forces the WBC asked for, so the two are comparable
                    // against the commanded ZMP.
                    {
                        // `cop_from_sole_wrench` gives the point in the SOLE
                        // CENTRE frame, and `oMi` is the ankle. On G1 the sole
                        // centre sits `sole_centre_x` = 35 mm ahead of the
                        // ankle, so adding the two directly biased this column
                        // by -35 mm in x -- which read as "the WBC puts its
                        // pressure 32 mm behind the command" and was reported
                        // that way on 2026-09-25 (mpc_in_the_loop.md Sec.29).
                        // In single support the same column read -37.5 mm,
                        // which a CoP confined to one sole cannot do; that is
                        // how the bias was caught (2026-09-26). Same
                        // construction as `Footsteps::from_fk`.
                        let pose = &data.oMi[mi];
                        let rot = misarta::se3::rotation_matrix(pose);
                        let origin = misarta::se3::translation(pose);
                        let point = origin
                            + rot
                                * na::Vector3::new(
                                    prof.sole_centre_x + cx,
                                    cy,
                                    -prof.sole_below_origin,
                                );
                        if !wbc_net_cop_acc[2].is_finite() {
                            wbc_net_cop_acc = [0.0; 3];
                        }
                        wbc_net_cop_acc[0] += point.x * fz;
                        wbc_net_cop_acc[1] += point.y * fz;
                        wbc_net_cop_acc[2] += fz;
                    }
                    // Only count a foot that is actually carrying something.
                    // A foot at 1 N with its CoP in the box corner is a
                    // 0.07 N*m wrench, and reporting that as a saturated
                    // constraint hides the one that matters.
                    if fz > 0.1 * total_mass * G {
                        cop_use = cop_use.max(
                            (cx.abs() / (sole_half_l * cop_frac))
                                .max(cy.abs() / (sole_half_w * cop_frac)),
                        );
                    }
                }
            }
            for k in 0..3 {
                f_qp_w[side][k] = extracted.forces[6 * slot + 3 + k];
            }
        }
        diag_wbc_net_cop = if wbc_net_cop_acc[2] > 1.0 {
            [
                wbc_net_cop_acc[0] / wbc_net_cop_acc[2],
                wbc_net_cop_acc[1] / wbc_net_cop_acc[2],
            ]
        } else {
            [f64::NAN; 2]
        };
        if let Some(file) = transfer_diag.as_mut() {
            let mut nominal = [0.0; 2];
            let mut mask = 0u8;
            for (slot, &side) in stance_sides.iter().enumerate() {
                mask |= 1 << side;
                // Native WBC force blocks use world axes (as f_qp_w above).
                nominal[side] = forces_nominal[6 * slot + 5];
            }
            let bw = total_mass * G;
            // Load threshold is evidence of support, not a contact detector.
            let measured_mask = u8::from(measured_pre[0] > 0.05 * bw)
                | (u8::from(measured_pre[1] > 0.05 * bw) << 1);
            let rot = misarta::se3::rotation_matrix(&data.oMi[rig.trunk_mi]);
            let mpc_valid = wrench_mpc_command.is_some();
            let values = [
                t,
                dt,
                bw,
                stair_transfer_cmd.map_or(0.0, |(_, phase, _)| phase.code()),
                stair_transfer_cmd.map_or(-1.0, |(high, _, _)| high as f64),
                stair_transfer_cmd.map_or(0.0, |(_, _, rf)| rf.alpha),
                mask as f64,
                measured_mask as f64,
                f64::from(mpc_valid),
                if mpc_valid {
                    diag_mpc_time.map_or(f64::NAN, |ts| t - ts)
                } else {
                    f64::NAN
                },
                wrench_mpc_failures as f64,
                if mpc_valid {
                    diag_mpc_reference[0]
                } else {
                    f64::NAN
                },
                if mpc_valid {
                    diag_mpc_reference[1]
                } else {
                    f64::NAN
                },
                wrench_mpc_command.as_ref().map_or(f64::NAN, |w| w[0][5]),
                wrench_mpc_command.as_ref().map_or(f64::NAN, |w| w[1][5]),
                nominal[0],
                nominal[1],
                f_qp_w[0][2],
                f_qp_w[1][2],
                measured_pre[0],
                measured_pre[1],
                (-rot[(2, 0)]).clamp(-1.0, 1.0).asin(),
                rot[(2, 1)].atan2(rot[(2, 2)]),
                cop_qp[0][0],
                cop_qp[1][0],
                measured_c.cop[0][0],
                measured_c.cop[1][0],
                f64::from(degraded),
                ffdc_dz,
            ];
            for (i, value) in values.iter().enumerate() {
                if i > 0 {
                    write!(file, ",").unwrap();
                }
                write!(file, "{value:.9}").unwrap();
            }
            writeln!(file).unwrap();
        }
        wbc_fz_previous = wbc_fz_now;
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
            // The first support change after the push opens the transfer
            // window. `support` is the SCHEDULE's opinion; that is the right
            // anchor, because the question is whether the machine completed
            // the transfer the plan asked for.
            // The anchor is entry into SINGLE support -- the tick the plan
            // commits the whole machine to one foot. The other transition
            // (single -> double) does not discriminate: at the first one after
            // the push both the 0.20 that falls and the 0.30 that survives sit
            // on the edge for 21/21 ticks, because in double support the CoP
            // of each sole rides its inner edge by construction. It is the
            // commitment to one foot that the disturbed machine either
            // completes or does not.
            if transfer_t.is_none() {
                if let (Some(Support::Double), Support::Single { .. }) = (support_prev, support) {
                    transfer_t = Some(t);
                    if (fix_dv.abs() > 0.0 || fix_tau.abs() > 0.0) && !fix_fired {
                        let th = fix_deg.to_radians();
                        let f = total_mass * fix_dv / fix_dt;
                        let mut tq = [0.0; 3];
                        if fix_tau_axis < 3 {
                            tq[fix_tau_axis] = fix_tau / fix_dt;
                        }
                        rig.sim.apply_external_force(
                            &push_link,
                            [f * th.cos(), f * th.sin(), 0.0],
                            tq,
                            fix_dt,
                        );
                        fix_fired = true;
                        println!(
                            "  FIX at t={t:.3} (commitment): dv={fix_dv:+.3} m/s at \
                             {fix_deg:.0} deg, angular {fix_tau:+.4} N*m*s about axis \
                             {fix_tau_axis}, over {fix_dt:.3} s"
                        );
                    }
                    // 6-D centroidal momentum h = A_G * v; rows 0..3 are
                    // angular, and row 0 is the roll axis.
                    let ag =
                        misarta::centroidal::compute_centroidal_momentum_matrix(&rig.model, &st.q);
                    h_roll_at_xfer = (&ag * v_dvec)[0];
                    if let Support::Single { stance, .. } = support {
                        let u = steps.xy(stance);
                        xfer_dcm_off = [xi.x - u.x, xi.y - u.y];
                        xfer_stance = stance as i32;
                    }
                }
            } else if transfer2_t.is_none() && t > transfer_t.unwrap() + 1e-9 {
                if let (Some(Support::Double), Support::Single { .. }) = (support_prev, support) {
                    transfer2_t = Some(t);
                }
            }
            // Time-triggered corrective impulse (see `fix_at_s`).
            if fix_at_s >= 0.0
                && !fix_fired
                && (fix_dv.abs() > 0.0 || fix_tau.abs() > 0.0)
                && push_at_t.map(|tp0| t - tp0 >= fix_at_s).unwrap_or(false)
            {
                let th = fix_deg.to_radians();
                let f = total_mass * fix_dv / fix_dt;
                let mut tq = [0.0; 3];
                if fix_tau_axis < 3 {
                    tq[fix_tau_axis] = fix_tau / fix_dt;
                }
                rig.sim.apply_external_force(
                    &push_link,
                    [f * th.cos(), f * th.sin(), 0.0],
                    tq,
                    fix_dt,
                );
                fix_fired = true;
                println!(
                    "  FIX at t={t:.3} (+{fix_at_s:.2} s after the push): \
                     dv={fix_dv:+.3} m/s at {fix_deg:.0} deg, angular {fix_tau:+.3}"
                );
            }
            if push_at_t.map(|tp0| t - tp0 <= 1.0).unwrap_or(false) {
                let ag = misarta::centroidal::compute_centroidal_momentum_matrix(&rig.model, &st.q);
                h_roll_max = h_roll_max.max((&ag * v_dvec)[0].abs());
            }
            if let Some(t2) = transfer2_t {
                if t - t2 <= transfer_s {
                    xfer2_ticks += 1;
                    let on_edge2 = (0..2).any(|side| {
                        measured.cop[side][2] > 0.1 * total_mass * G
                            && measured.cop[side][1].abs() >= edge_frac * sole_half_w
                    });
                    if on_edge2 {
                        xfer2_edge_ticks += 1;
                    }
                }
            }
            if let Some(t0) = transfer_t {
                if t - t0 <= transfer_s {
                    xfer_ticks += 1;
                    let mut on_edge = false;
                    for side in 0..2 {
                        let fz = measured.cop[side][2];
                        if fz <= 0.1 * total_mass * G {
                            continue;
                        }
                        if measured.cop[side][1].abs() >= edge_frac * sole_half_w {
                            on_edge = true;
                        }
                        // A foot the schedule says is swinging, carrying load.
                        if !support.is_stance(side) {
                            xfer_wrongfoot_ticks += 1;
                        }
                    }
                    if on_edge {
                        xfer_edge_ticks += 1;
                    } else if xfer_leave_t.is_none() {
                        xfer_leave_t = Some(t);
                    }
                }
            }
            let fz_total = measured.f_w[0][2] + measured.f_w[1][2];
            push_fz_total_min = push_fz_total_min.min(fz_total);
            push_fz_total_max = push_fz_total_max.max(fz_total);
            if fz_total < 0.5 * total_mass * G {
                push_unloaded_ticks += 1;
            }
            if in_push_window {
                win_cop_max = win_cop_max.max(cop_use);
                win_fz_total_min = win_fz_total_min.min(fz_total);
                win_fz_total_max = win_fz_total_max.max(fz_total);
                if fz_total < 0.5 * total_mass * G {
                    win_unloaded_ticks += 1;
                }
            }
            for side in 0..2 {
                let fz = measured.f_w[side][2];
                if fz <= 1.0 {
                    continue; // an unloaded foot has no meaningful ratio
                }
                let tan = (measured.f_w[side][0].powi(2) + measured.f_w[side][1].powi(2)).sqrt();
                push_slip_wbc_max = push_slip_wbc_max.max(tan / (friction_mu * fz));
                push_slip_plant_max = push_slip_plant_max.max(tan / (mu_ground * fz));
                if in_push_window {
                    win_slip_plant_max = win_slip_plant_max.max(tan / (mu_ground * fz));
                    if tan >= 0.98 * mu_ground * fz {
                        win_slip_ticks += 1;
                    }
                }
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
        for ji in 0..rig.robot.joints.len() {
            let qj = rig.robot.joint_positions[ji];
            q_lo[ji] = q_lo[ji].min(qj);
            q_hi[ji] = q_hi[ji].max(qj);
        }
        {
            let all: Vec<(String, String, f64)> = rig
                .sim
                .contacts()
                .into_iter()
                .filter(|c| !c.body1.is_empty() && !c.body2.is_empty())
                .map(|c| (c.body1.clone(), c.body2.clone(), c.force_mag))
                .collect();
            let is_target = |a: &str, b: &str| a == "spar_target" || b == "spar_target";
            // Feet on staircase scenery are stance, not "the QP solving
            // against a contact it has no model for" -- measure_contacts now
            // counts them as ground. Anything ELSE against a stair (a shin,
            // a toe grinding a riser face) stays in the tally.
            let foot = |n: &str| n == prof.foot_links[0] || n == prof.foot_links[1];
            let is_stairs = |a: &str, b: &str| {
                (a.starts_with("stair") && foot(b)) || (b.starts_with("stair") && foot(a))
            };
            let f_hit: f64 = all
                .iter()
                .filter(|(a, b, _)| is_target(a, b))
                .map(|(_, _, f)| *f)
                .sum();
            hit_f_now = f_hit;
            if f_hit > 0.0 {
                hit_ticks += 1;
                hit_peak_f = hit_peak_f.max(f_hit);
                hit_impulse += f_hit * dt;
                hit_first_t.get_or_insert(t);
                hit_last_t = t;
                ep_t0.get_or_insert(t);
                ep_imp += f_hit * dt;
            } else if let Some(t0) = ep_t0.take() {
                hit_episodes.push((t - t0, ep_imp));
                ep_imp = 0.0;
            }
            let hits: Vec<(String, String, f64)> = all
                .into_iter()
                .filter(|(a, b, _)| !is_target(a, b) && !is_stairs(a, b))
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
        let prefix_applied;
        {
            let rig_ref = &rig;
            let tg = &tau_gravity;
            let fallback =
                move |out: &mut [f64]| gravity_plus_posture(rig_ref, tg, v, hold_kp, hold_kd, out);
            let fallback_active = policy.apply(&sol.status, &mut robot_taus, t, dt, nc, &fallback);
            prefix_applied = prefix_eligible && !fallback_active;
            // Existing policy has no held command on its first failed tick.
            // Use its existing recomputed fallback instead of applying rejected QP torques.
            if qp_rejected && !qp_has_good_command {
                fallback(&mut robot_taus);
            }
            if matches!(sol.status, misa_wbc::SolveStatus::Optimal) {
                qp_has_good_command = true;
            }
        }
        // The fallback bypasses the QP torque box. Match the plant's
        // actuator range before sending and logging either control path.
        for (ji, vi) in rig.actuated() {
            let bound = rig.torque_max[vi - 6];
            robot_taus[ji] = robot_taus[ji].clamp(-bound, bound);
        }
        if policy.consecutive_degraded() >= 10 {
            n_open_loop += 1;
        }
        let mut tau_use_max = 0.0_f64;
        let mut tau_saturated_count = 0usize;
        for (ji, vi) in rig.actuated() {
            let ratio = robot_taus[ji].abs() / rig.torque_max[vi - 6].max(1e-9);
            tau_use_max = tau_use_max.max(ratio);
            tau_saturated_count += usize::from(ratio >= 0.98);
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
        let trunk_tilt = trot[(2, 1)]
            .atan2(trot[(2, 2)])
            .abs()
            .max((-trot[(2, 0)]).asin().abs());
        let trunk_tilt_roll = trot[(2, 1)].atan2(trot[(2, 2)]);
        let trunk_tilt_pitch = (-trot[(2, 0)]).asin();

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
                // Where the jab was when the shove landed. The jab runs on its
                // own clock, unrelated to the gait, so without this line a
                // push-vs-jab comparison is averaging over an unrecorded phase
                // -- and "pushed at full extension" and "pushed back in guard"
                // are not the same experiment.
                if jab {
                    let ph = ((t - jab_start) / jab_period).max(0.0);
                    let side = if jab_pattern
                        .chars()
                        .nth(ph.floor() as usize % jab_pattern.chars().count())
                        .is_some_and(|c| c.eq_ignore_ascii_case(&'l'))
                    {
                        "L"
                    } else {
                        "R"
                    };
                    let a = 0.5 * (1.0 - (2.0 * PI * ph.fract()).cos());
                    println!(
                        "  PUSH JAB PHASE: side={side} phase={:.3} extension={a:.3} \
                         (0 = in guard, 1 = fully extended)",
                        ph.fract()
                    );
                }
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
                if ref_any
                    && ref_trig_at_push
                    && push_imp >= ref_trig_min_imp
                    && let Support::Single { stance, .. } = support
                    && (fy > 0.0) == (stance == 0)
                {
                    ref_engage_pending = Some(t);
                }
            }
        }
        if let Some(mode) = ref_mode
            && let Some(lib) = &ref_lib
            && t >= mode.t0 + lib.total() - mode.tau0
        {
            ref_mode = None;
            ref_cooldown_until = t + ref_cooldown;
            landing_timing.event_mode = event_mode_cfg;
            // The reference leaves the feet where the recovery put them
            // (in tandem, narrow). Every later footfall is re-centred on the
            // feet as they are now, at the nominal stance width, so the gait
            // that resumes is the nominal one and not a stride-0 echo of the
            // recovery (measured: the echo fell 1.7 s after the reference).
            if ref_reset_stance {
                let i = gait.index_at(t);
                let cur = *footsteps.at_slice(i);
                let mid = 0.5 * (cur.sole[0].xy() + cur.sole[1].xy());
                let half_w = 0.5 * (footsteps.initial.sole[0].y - footsteps.initial.sole[1].y).abs();
                let j0 = if ref_reset_incl_current { i } else { i + 1 };
                for side in 0..2 {
                    let mut want = na::Vector2::new(mid.x, mid.y + if side == 0 { half_w } else { -half_w });
                    if let Some(j) = (j0..gait.slices.len()).find(|&j| gait.slices[j].support.swing() == Some(side)) {
                        if j == i {
                            want.x += ref_reset_lead;
                            println!("  [ref] t={t:.3} re-centre the swing under way: foot {side} -> ({:.3},{:.3})", want.x, want.y);
                        }
                        let d = want - footsteps.at_slice(j).sole[side].xy();
                        footsteps.shift_from(j, side, d);
                    }
                }
                dcm_plan = DcmPlan::from_footsteps_rolled(&gait, &footsteps, z_lip, zmp_lat_scale, zmp_roll);
            }
            // Let the DCM settle on the new (tandem) stance before stepping
            // again: lengthen the first double support after the reference.
            if ref_post_hold > 0.0 {
                let i = gait.index_at(t);
                if let Some(j) = (i..gait.slices.len()).find(|&j| matches!(gait.slices[j].support, Support::Double)) {
                    let end = gait.slices[j].t1 + ref_post_hold;
                    gait.retime(j, end);
                    dcm_plan = DcmPlan::from_footsteps_rolled(&gait, &footsteps, z_lip, zmp_lat_scale, zmp_roll);
                }
            }
            println!("  [ref] t={t:.3} reference over, back to the gait");
        }

        support_prev = Some(support);
        // What the disturbance is doing RIGHT NOW, for the replay to draw.
        // Read from the sim's own pulse list rather than recomputed here, so
        // the arrow cannot disagree with the force the plant actually saw.
        let push_now = rig
            .sim
            .external_force_pulses()
            .iter()
            .find(|p| p.link_name == push_link)
            .map(|p| [p.force[0], p.force[1]])
            .unwrap_or([0.0, 0.0]);
        let audit_pre = tick_audit.as_ref().map(|audit| audit.sample(&rig));
        if let Some(file) = &mut qp_runtime_log {
            use std::io::Write;
            let control_ns = qp_control_start
                .elapsed()
                .as_nanos()
                .saturating_sub(qp_capture_ns)
                .saturating_sub(mpc_capture_ns);
            writeln!(file, "{t:.9},{qp_solve_ns},{qp_check_ns},\"{:?}\",{qp_physical_violation:.12e},{qp_reset},{},{control_ns},{},{wrench_mpc_solves},{wrench_mpc_failures},{prefix_applied},{repair_applied},{repair_scaled_delta:.12e},{repair_ns}", sol.status,
                control_ns > (dt * 1e9) as u128, diag_mpc_time.map_or(-1.0, |stamp| t-stamp)).unwrap();
        }
        if let Some(file) = &mut qp_wrench_log {
            use articara::biped::log::contact_wrench_in_sole;
            use std::io::Write;
            // All four streams are expressed about the same current physical sole
            // centre and actual MuJoCo foot axes, before the plant step. MPC is
            // the held forecast at its solve-time patch, shifted/rotated here.
            for side in 0..2 {
                let link = prof.foot_links[side];
                let pose = rig
                    .sim
                    .body_world_position(link)
                    .zip(rig.sim.body_world_orientation(link));
                let mut planned = na::SVector::<f64, 6>::zeros();
                let mut wbc = na::SVector::<f64, 6>::zeros();
                let mut mpc = na::SVector::<f64, 6>::repeat(f64::NAN);
                let slot = stance_sides.iter().position(|s| *s == side);
                if let Some((position, orientation)) = &pose {
                    let rotation = orientation.to_rotation_matrix().into_inner();
                    let origin = na::Vector3::from_column_slice(position);
                    let sole = origin
                        + rotation
                            * na::Vector3::new(prof.sole_centre_x, 0.0, -prof.sole_below_origin);
                    if let Some(slot) = slot {
                        let force = na::Vector3::from_column_slice(
                            &extracted.forces.as_slice()[6 * slot + 3..6 * slot + 6],
                        );
                        let moment = na::Vector3::from_column_slice(
                            &extracted.forces.as_slice()[6 * slot..6 * slot + 3],
                        );
                        wbc = contact_wrench_in_sole(&origin, &force, &moment, &sole, &rotation);
                        let force = na::Vector3::from_column_slice(
                            &forces_nominal.as_slice()[6 * slot + 3..6 * slot + 6],
                        );
                        let moment = na::Vector3::from_column_slice(
                            &forces_nominal.as_slice()[6 * slot..6 * slot + 3],
                        );
                        planned =
                            contact_wrench_in_sole(&origin, &force, &moment, &sole, &rotation);
                    }
                    if let (Some(command), Some(frames)) = (&wrench_mpc_command, &wrench_mpc_frames)
                    {
                        let source = &frames[side];
                        let force = source.rotation * command[side].fixed_rows::<3>(3);
                        let moment = source.rotation * command[side].fixed_rows::<3>(0);
                        mpc = contact_wrench_in_sole(
                            &source.position,
                            &force,
                            &moment,
                            &sole,
                            &rotation,
                        );
                    }
                }
                let valid =
                    |w: &na::SVector<f64, 6>| pose.is_some() && w.iter().all(|v| v.is_finite());
                let planned_support = match support {
                    Support::Double => true,
                    Support::Single { stance, .. } => stance == side,
                };
                write!(
                    file,
                    "{t:.9},{side},{planned_support},{},{},{},{},{},{degraded},{}",
                    slot.is_some(),
                    valid(&mpc),
                    valid(&planned),
                    valid(&wbc),
                    measured_c.sole_wrench_valid[side],
                    diag_mpc_time.map_or(-1.0, |stamp| t - stamp)
                )
                .unwrap();
                for wrench in [
                    planned.as_slice(),
                    mpc.as_slice(),
                    wbc.as_slice(),
                    &measured_c.sole_wrench[side],
                ] {
                    for component in wrench {
                        write!(file, ",{component:.12e}").unwrap();
                    }
                }
                writeln!(file).unwrap();
            }
        }
        write_to_plant(&mut rig, ctrl_mode, &robot_taus, &extracted.qddot, v, dt);
        rig.step(mj_substeps);
        if substep_audit_path.is_some() {
            let tail = rig.sim.trace_len().saturating_sub(mj_substeps as usize);
            for frame in rig.sim.trace().skip(tail) {
                substep_samples += 1;
                for (ji, joint) in rig.robot.joints.iter().enumerate() {
                    if joint.joint_type != "revolute" { continue; }
                    let (q, qd, tau) = (frame.q[ji], frame.qvel[ji], frame.tau[ji]);
                    if !q.is_finite() || !qd.is_finite() || !tau.is_finite() {
                        substep_nonfinite += 1;
                        continue;
                    }
                    let extrema = &mut substep_extrema[ji];
                    extrema[0] = extrema[0].min(q);
                    extrema[1] = extrema[1].max(q);
                    extrema[2] = extrema[2].max(qd.abs());
                    extrema[3] = extrema[3].max(tau.abs());
                    let excess = &mut substep_max_excess[ji];
                    excess[0] = excess[0].max((joint.lower - q).max(q - joint.upper).max(0.0));
                    excess[1] = excess[1].max((qd.abs() - joint.velocity).max(0.0));
                    excess[2] = excess[2].max((tau.abs() - joint.effort).max(0.0));
                }
            }
        }
        if let (Some(audit), Some(pre)) = (tick_audit.as_mut(), audit_pre.as_ref()) {
            // step() refreshes MuJoCo with forward(): these are post-state
            // forces. Only f_w is used, independent of stale `data` CoP.
            let last = measure_contacts(&rig, data, friction_mu);
            audit.write(
                &rig,
                t,
                mj_substeps as f64 * rig.sim.timestep(),
                pre,
                &robot_taus,
                measured_pre,
                [f_qp_w[0][2], f_qp_w[1][2]],
                [last.f_w[0][2], last.f_w[1][2]],
            );
        }

        if let (Some(file), Some((mi, vi, ji, mjvi))) = (ankle_cbf_diag.as_mut(), ankle_cbf_indices)
        {
            let row = vi - 6;
            let position = q[rig.model.q_idx[mi]];
            let velocity = v[vi];
            let a1 = jlim_cfg.alpha1[row];
            let a2 = jlim_cfg.alpha2[row];
            let upper = -(a1 + a2) * velocity + a1 * a2 * (jlim_cfg.q_max[row] - position);
            let (post_q, post_v) = rig
                .sim
                .joint_q_qd("right_ankle_roll_joint")
                .expect("ankle joint state after stepping");
            // MuJoCo force and acceleration entries are from the final physics
            // substep under this tick's held command. Pre/post velocity spans
            // the complete control interval instead.
            writeln!(file,
                "{t:.6},{position:.9},{velocity:.9},{:.9},{upper:.9},{:.9},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{post_q:.9},{post_v:.9},{:.9},{:.9},{:.9},{:.9},{:.9}",
                jlim_cfg.q_max[row], extracted.qddot[vi], measured_pre[1],
                f_qp_w[1][0], f_qp_w[1][1], f_qp_w[1][2],
                measured_c.f_w[1][0], measured_c.f_w[1][1],
                rig.sim.qacc()[mjvi], rig.sim.qfrc_constraint()[mjvi],
                rig.sim.qfrc_passive()[mjvi], rig.sim.qfrc_actuator()[mjvi], robot_taus[ji],
            ).unwrap();
        }

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
            let actual_feet = Footsteps::from_fk(
                data,
                rig.foot_mi,
                prof.sole_centre_x,
                prof.sole_below_origin,
            );
            let [al, ar] = actual_feet.sole;
            let [gl, gr] = steps.sole;
            let link_l = misarta::se3::translation(&data.oMi[rig.foot_mi[0]]);
            let link_r = misarta::se3::translation(&data.oMi[rig.foot_mi[1]]);
            let anchor_l_y = anchors.a[0].map_or(f64::NAN, |(p, _)| p.y);
            let anchor_r_y = anchors.a[1].map_or(f64::NAN, |(p, _)| p.y);
            let foot_v_l = rig
                .sim
                .body_world_linear_velocity(prof.foot_links[0])
                .unwrap_or([0.0; 3]);
            let foot_v_r = rig
                .sim
                .body_world_linear_velocity(prof.foot_links[1])
                .unwrap_or([0.0; 3]);
            let extra = [
                xi.x,
                xi.y,
                r.xi.x,
                r.xi.y,
                r.p.x,
                r.p.y,
                p_cmd.x,
                p_cmd.y,
                clamped,
                dcm_err,
                support_code,
                step_idx as f64,
                *shares.first().unwrap_or(&0.0),
                *shares.get(1).unwrap_or(&0.0),
                swing_tgt.x,
                swing_tgt.y,
                swing_tgt.z,
                ankle_roll[0],
                ankle_roll[1],
                push_now[0],
                push_now[1],
                hit_f_now,
                ep_imp,
                hit_impulse,
                al.x,
                al.y,
                al.z,
                ar.x,
                ar.y,
                ar.z,
                anchor_l_y,
                anchor_r_y,
                anchor_l_y - link_l.y,
                anchor_r_y - link_r.y,
                foot_v_l[0],
                foot_v_l[1],
                foot_v_r[0],
                foot_v_r[1],
                contact_activation[0],
                contact_activation[1],
                contact_force_activation[0],
                contact_force_activation[1],
                load_chatter_ema,
                gl.x,
                gl.y,
                gl.z,
                gr.x,
                gr.y,
                gr.z,
                if replan_engaged { 1.0 } else { 0.0 },
                preview_c.x,
                preview_c.y,
                preview_v.x,
                preview_v.y,
                preview_error,
                preview_count as f64,
                preview_failures as f64,
                landing_delay,
                if stair_transfer_cmd.is_some() {
                    1.0
                } else {
                    0.0
                },
                stair_transfer_cmd.map_or(0.0, |(_, phase, _)| phase.code()),
                stair_transfer_cmd.map_or(0.0, |(_, _, rf)| rf.alpha),
                stair_transfer_cmd.map_or(-1.0, |(high, _, _)| high as f64),
                ffdc_dz,
                ffdc_err_log,
                capture_error_log,
                capture_tilt_log,
                trunk_tilt_roll,
                trunk_tilt_pitch,
                trunk_cmd_rp[1],
                trunk_acc_rp[1],
                trunk_cmd_rp[0],
                trunk_acc_rp[0],
                transfer_guard_log,
                if wrench_mpc_command.is_some() {
                    1.0
                } else {
                    0.0
                },
                wrench_mpc_command.as_ref().map_or(0.0, |w| w[0][5]),
                wrench_mpc_command.as_ref().map_or(0.0, |w| w[1][5]),
                diag_mpc_net_cop[0],
                diag_mpc_net_cop[1],
                mpc_fs_last.x,
                mpc_fs_last.y,
                diag_wbc_net_cop[0],
                diag_wbc_net_cop[1],
                wrench_mpc_solves as f64,
                wrench_mpc_failures as f64,
                swing_pos_err_xy,
                swing_pos_err_z,
                swing_task_residual_rms,
                swing_res_z,
                swing_res_rows,
                swing_cmd_acc_z,
                swing_achieved_acc_z,
                tau_use_max,
                tau_saturated_count as f64,
                reach_used[0],
                reach_used[1],
                hip_z[0],
                hip_z[1],
                stride_max_now,
            ];
            if let Some(file) = &mut mom_diag_log {
                use std::io::Write;
                let ag = misarta::centroidal::compute_centroidal_momentum_matrix(&rig.model, &st.q);
                let h = &ag * v_dvec;
                let mut h_arm = na::DVector::zeros(h.len());
                for (ji, vi) in rig.actuated() {
                    let name = &rig.robot.joints[ji].name;
                    if name.contains("shoulder") || name.contains("elbow") || name.contains("wrist")
                    {
                        h_arm += ag.column(vi) * v_dvec[vi];
                    }
                }
                writeln!(file, "{t:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{}",
                    h[0],h[1],h[2],h_arm[0],h_arm[1],h_arm[2],
                    h[0]-h_arm[0],h[1]-h_arm[1],h[2]-h_arm[2],
                    measured.cop[0][2],measured.cop[1][2],cop_qp[0][2],cop_qp[1][2],nc).unwrap();
            }
            // GAIT_DUMP_T="t0,t1": one line per tick with the gait slice the
            // clock is on against the contact-driven stance set.
            if let Some((d0, d1)) = gait_dump_t
                && t >= d0
                && t <= d1
            {
                let i = gait.index_at(t);
                let sl = &gait.slices[i];
                println!(
                    "  [gait] t={t:.3} slice {i} {:?} [{:.3},{:.3}] stance={:?} fz=({:.0},{:.0}) xi.x={:.3} ref.x={:.3}",
                    sl.support, sl.t0, sl.t1, stance_sides, fz_meas[0], fz_meas[1], xi.x, r.xi.x
                );
            }
            let row = Row {
                t,
                com,
                com_ref_z: mpc_reach_z.unwrap_or(z_ref),
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
            let y = rig
                .sim
                .body_world_orientation(&rig.robot.root_link)
                .unwrap()
                .euler_angles()
                .2;
            if let Some((pp, py)) = prev_pose {
                if t >= t_ramp_end {
                    let (dx, dy) = (p[0] - pp[0], p[1] - pp[1]);
                    let (c, sn) = (py.cos(), py.sin());
                    travel[0] += dx * c + dy * sn;
                    travel[1] += -dx * sn + dy * c;
                    // Shortest-arc difference, so the accumulator never wraps.
                    let mut dy_aw = y - py;
                    while dy_aw > std::f64::consts::PI {
                        dy_aw -= std::f64::consts::TAU;
                    }
                    while dy_aw < -std::f64::consts::PI {
                        dy_aw += std::f64::consts::TAU;
                    }
                    travel[2] += dy_aw;
                }
            }
            prev_pose = Some((p, y));
            last_state = (p, y, t);
        }
        if ramp_end_state.is_none() && t >= t_ramp_end {
            let p = rig.sim.body_world_position(&rig.robot.root_link).unwrap();
            let y = rig
                .sim
                .body_world_orientation(&rig.robot.root_link)
                .unwrap()
                .euler_angles()
                .2;
            ramp_end_state = Some((p, y, t));
        }
        if walk_end_state.is_none() && t >= t_walk_end {
            let p = rig.sim.body_world_position(&rig.robot.root_link).unwrap();
            let y = rig
                .sim
                .body_world_orientation(&rig.robot.root_link)
                .unwrap()
                .euler_angles()
                .2;
            walk_end_state = Some((p, y, t));
        }
        let cur_z = rig.sim.body_world_position(&rig.robot.root_link).unwrap()[2];
        min_z = min_z.min(cur_z);
        let tilt = roll.abs().max(pitch.abs());
        max_tilt = max_tilt.max(tilt);
        if push_at_t.is_some() {
            push_tilt_max = push_tilt_max.max(tilt);
        }
        if in_push_window {
            win_tilt_max = win_tilt_max.max(tilt);
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
        // Height is judged against the surface under the body, or a robot
        // that WALKED down a staircase would be scored as having fallen it.
        let floor_z = if let Some((_high, _phase, rf)) = stair_transfer_cmd {
            // While the feet straddle two levels, the terrain under CoM.x is
            // already the upper tread even though the common transfer clock
            // deliberately raises the body gradually.  Judge clearance from
            // that same clock; at alpha=1 this equals the upper tread.
            terrain_base + (rf.z - z_com)
        } else {
            stairs.as_ref().map_or(0.0, |st| st.h(com.x))
        };
        if cur_z - floor_z < 0.30 || tilt > 0.52 {
            println!("  FELL at t={t:.3} (z={cur_z:.3}, floor={floor_z:.3}, tilt={tilt:.3})");
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

    println!(
        "COM REPLAN: {preview_count} solves / {preview_failures} failures; {landing_retimes} landing retimes ({landing_delay:.3}s net), {} confirmed / {} timeouts",
        landing_timing.touchdowns, landing_timing.timeouts
    );
    println!(
        "STAIR TRANSFER: {stair_transfer_started} started / {stair_transfer_completed} completed / {stair_transfer_timeouts} timeouts / {} active",
        usize::from(stair_transfer.is_some()),
    );
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
        let ach = [travel[0] / dur, travel[1] / dur, travel[2] / dur];
        let cmd = [vx, vy, wz];
        let names = ["vx", "vy", "wz"];
        let units = ["m/s", "m/s", "rad/s"];
        let mut parts = Vec::new();
        for k in 0..3 {
            if cmd[k].abs() < 1e-9 && ach[k].abs() < 1e-3 {
                continue;
            }
            let pct = if cmd[k].abs() > 1e-9 {
                100.0 * ach[k] / cmd[k]
            } else {
                f64::NAN
            };
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
    println!(
        "  max ankle_roll use = {:.0}% of limit",
        max_ankle_roll_use * 100.0
    );
    println!("  degraded solves: {}", tally.n);
    tally.report();
    println!("  open-loop ticks: {n_open_loop}");
    println!(
        "  contact-set corrections: {n_corrected} ticks where the feet disagreed with the schedule"
    );
    println!(
        "  phase retimed by contact: {n_early} early / {n_late} late ticks, {:.0} ms total",
        retime_total * 1e3
    );
    if contact_admit_confirmed || contact_admit_scheduled {
        println!("  confirmed-landing admissions: {n_confirmed_admissions}");
    }
    println!(
        "  footstep adaptations: {n_adapt} steps moved, largest {:.1} mm, {n_replan} live replans",
        // (contact planner counters follow)
        max_adapt * 1e3
    );
    if contact_plan_on {
        println!("  contact plan: {n_cp_shift} shifts, {n_cp_retime} retimes ({:.0} ms total)", cp_retime_total * 1e3);
    }
    if ds_feasible {
        println!("  ds feasible: {n_ds_feasible} retimes ({:.0} ms total)", ds_feasible_total * 1e3);
        println!("  ds capture: {n_ds_capture} early ends ({:.0} ms total)", ds_capture_total * 1e3);
        println!("  re-steps: {n_restep}");
        println!("  plan re-anchors: {n_reanchor} ({:.0} mm total)", reanchor_total * 1e3);
        println!("  hip strategy (MOM_CMP): {n_mom_cmp_ticks} ticks, peak rate {mom_cmp_peak:.1} N m");
        if ds_feasible_continuous {
            println!("  ds continuous: {n_ds_continuous} plan writes");
        }
    }
    if land_wait_hold {
        println!("  landing wait hold: {n_wait_hold} plan writes");
    }
    if push_z_drop > 0.0 {
        println!("  push CoM drops: {n_push_z_drop}");
    }
    if ref_any {
        println!("  reference engagements: {ref_engagements}");
    }
    if landing_timing.event_mode {
        println!(
            "  contact events: {} touchdowns by event, {} probes, {} timeouts",
            landing_timing.events, landing_timing.probes, landing_timing.timeouts
        );
    }
    if land_by_load {
        println!("  landing by load: {} confirmed, {n_land_by_load_shift} plan shifts", landing_timing.load_confirm_count);
    }
    println!(
        "  step-timing adaptations: {n_retime_dcm} ticks retimed, {:.0} ms total, \
         largest single shift {:.0} ms",
        retime_dcm_total * 1e3,
        retime_dcm_max * 1e3
    );
    println!("  reduced-step cuts: {n_time_cut} ticks (unreachable -> end the step now)");
    println!(
        "  reactive widen: {n_time_wide} steps ({:.0} mm, unreachable -> land further out)",
        adapt_time_widen * 1e3
    );
    println!(
        "  double-support retimes: {n_ds_retime} ticks, largest shift {:.0} ms",
        ds_retime_max * 1e3
    );
    println!(
        "  step-timing solve: {n_time_solved} solved / {n_time_wrongside} declined \
         (DCM on the far side of the ZMP -- no positive time reaches the target), \
         {n_time_clamped} clamped to the duration bounds"
    );
    {
        // Arms first and by name, because they are the ones nothing else
        // constrains; then the worst joint anywhere, so a leg that starts
        // wandering is not silently omitted by an arm-shaped filter.
        let pp = |ji: usize| (q_hi[ji] - q_lo[ji]).to_degrees();
        let mut arms: Vec<(f64, &str)> = (0..rig.robot.joints.len())
            .filter(|&ji| {
                let n = rig.robot.joints[ji].name.as_str();
                (n.contains("shoulder") || n.contains("elbow")) && q_hi[ji] > q_lo[ji]
            })
            .map(|ji| (pp(ji), rig.robot.joints[ji].name.as_str()))
            .collect();
        arms.sort_by(|a, b| b.0.partial_cmp(&a.0).unwrap());
        let worst = (0..rig.robot.joints.len())
            .filter(|&ji| q_hi[ji] > q_lo[ji])
            .map(|ji| (pp(ji), rig.robot.joints[ji].name.as_str()))
            .fold((0.0_f64, ""), |acc, x| if x.0 > acc.0 { x } else { acc });
        let arm_max = arms.first().map(|a| a.0).unwrap_or(0.0);
        println!(
            "  joint travel peak-to-peak: arms {arm_max:.1} deg ({}), worst overall \
             {:.1} deg ({})          -- an arm that swings without touching anything \
             is invisible to the self-collision count above",
            arms.first().map(|a| a.1).unwrap_or("none"),
            worst.0,
            worst.1
        );
        for (d, n) in arms.iter().take(3) {
            println!("    {n:32} {d:7.1} deg");
        }
    }
    if spar_wall > 0.0 {
        let span = hit_first_t.map(|t0| hit_last_t - t0).unwrap_or(0.0);
        println!(
            "  target hits: {hit_ticks} ticks, first contact at t={:.3}, span {:.3} s, \
             peak {hit_peak_f:.1} N, transferred impulse {hit_impulse:.4} N*s",
            hit_first_t.unwrap_or(f64::NAN),
            span
        );
        if !hit_episodes.is_empty() {
            let n = hit_episodes.len() as f64;
            let dmean = hit_episodes.iter().map(|e| e.0).sum::<f64>() / n;
            let dmax = hit_episodes.iter().map(|e| e.0).fold(0.0, f64::max);
            let imean = hit_episodes.iter().map(|e| e.1).sum::<f64>() / n;
            let imax = hit_episodes.iter().map(|e| e.1).fold(0.0, f64::max);
            println!(
                "  per punch ({} contacts): duration mean {dmean:.3} s / max {dmax:.3} s, \
                 impulse mean {imean:.3} / max {imax:.3} N*s",
                hit_episodes.len()
            );
        }
    }
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
            if ux.abs() > 1e-9 {
                sole_half_l / ux.abs()
            } else {
                f64::INFINITY
            },
            if uy.abs() > 1e-9 {
                sole_half_w / uy.abs()
            } else {
                f64::INFINITY
            },
        ]
        .into_iter()
        .fold(f64::INFINITY, f64::min);
        let ankle_budget = total_mass * omega * d_cop;
        println!(
            "  ankle-strategy budget in that direction: {ankle_budget:.2} N*s \
             ({:.1}x exceeded)",
            push_imp / ankle_budget.max(1e-9)
        );
        // ---- the early window: everything below the push, before the fall --
        println!("  --- first {push_window_s:.2} s after the push (precursor window) ---");
        match transfer_t {
            Some(t0) => {
                println!(
                    "  weight transfer at t={t0:.3} ({:.3} s after the push), \
                     first {transfer_s:.2} s of it:",
                    t0 - tp
                );
                println!(
                    "    CoP on the sole edge (|y| >= {:.0}% of half-width) for \
                     {xfer_edge_ticks}/{xfer_ticks} ticks ({:.0}%)",
                    edge_frac * 100.0,
                    100.0 * xfer_edge_ticks as f64 / xfer_ticks.max(1) as f64,
                );
                match xfer_leave_t {
                    Some(tl) => println!(
                        "    left the edge {:.0} ms after the transfer",
                        (tl - t0) * 1e3
                    ),
                    None => println!(
                        "    NEVER left the edge inside the window -- the stance \
                         foot could not modulate its own pressure"
                    ),
                }
                println!(
                    "    load on a foot the schedule says is swinging: \
                     {xfer_wrongfoot_ticks} ticks"
                );
                if xfer2_ticks > 0 {
                    println!(
                        "    SECOND single support: CoP on the edge for \
                         {xfer2_edge_ticks}/{xfer2_ticks} ticks ({:.0}%)",
                        100.0 * xfer2_edge_ticks as f64 / xfer2_ticks as f64
                    );
                }
                println!(
                    "    centroidal roll momentum: {h_roll_at_xfer:+.4} kg*m^2/s at the \
                     transfer, peak |h_x| {h_roll_max:.4} in the first second"
                );
                let b_nom =
                    (steps.sole[0].y - steps.sole[1].y).abs() / (1.0 + (omega * gp.t_ss).exp());
                println!(
                    "    DCM offset from the committed stance foot: \
                     x {:+.1} mm, y {:+.1} mm (stance {}), nominal |b_y| = {:.1} mm",
                    xfer_dcm_off[0] * 1e3,
                    xfer_dcm_off[1] * 1e3,
                    if xfer_stance == 0 { "L" } else { "R" },
                    b_nom * 1e3,
                );
            }
            None => println!("  no support change after the push (run ended first)"),
        }
        println!("  win peak |xi - xi_ref| = {:.1} mm", win_dcm_err_max * 1e3);
        println!("  win peak tilt = {win_tilt_max:.3} rad");
        println!("  win peak CoP box use = {win_cop_max:.2}");
        println!(
            "  win peak friction use = {win_slip_plant_max:.2} of the plant cone, \
             sliding {win_slip_ticks} ticks ({:.0} ms)",
            win_slip_ticks as f64 * dt * 1e3
        );
        println!(
            "  win total vertical ground force: min {:.1} N ({:.2}x weight), \
             peak {win_fz_total_max:.1} N ({:.2}x weight)",
            if win_fz_total_min.is_finite() {
                win_fz_total_min
            } else {
                0.0
            },
            if win_fz_total_min.is_finite() {
                win_fz_total_min / (total_mass * G)
            } else {
                0.0
            },
            win_fz_total_max / (total_mass * G),
        );
        println!(
            "  win unloaded ticks: {win_unloaded_ticks} ({:.0} ms below half body weight)",
            win_unloaded_ticks as f64 * dt * 1e3
        );
        println!(
            "  win degraded solves: {}",
            win_degraded_end.saturating_sub(win_degraded_at.unwrap_or(win_degraded_end))
        );
        println!("  --- whole run after the push (contaminated by the fall) ---");
        println!(
            "  peak |xi - xi_ref| after the push = {:.1} mm",
            push_dcm_err_max * 1e3
        );
        println!("  peak tilt after the push = {push_tilt_max:.3} rad");
        println!(
            "  degraded solves after the push: {}",
            tally.n - push_degraded_at
        );
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
        let settled_for = push_last_exceed
            .map(|te| t_last - te)
            .unwrap_or(f64::INFINITY);
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
    if let Some(path) = substep_audit_path {
        let joints: Vec<_> = rig.robot.joints.iter().enumerate()
            .filter(|(_, joint)| joint.joint_type == "revolute")
            .map(|(ji, joint)| serde_json::json!({
                "name": joint.name, "limits": {"lower_rad": joint.lower, "upper_rad": joint.upper,
                    "velocity_rad_s": joint.velocity, "effort_nm": joint.effort},
                "observed": {"min_q_rad": substep_extrema[ji][0], "max_q_rad": substep_extrema[ji][1],
                    "max_abs_qd_rad_s": substep_extrema[ji][2], "max_abs_tau_nm": substep_extrema[ji][3]},
                "excess": {"position_rad": substep_max_excess[ji][0],
                    "velocity_rad_s": substep_max_excess[ji][1], "effort_nm": substep_max_excess[ji][2]}
            })).collect();
        let passed = substep_nonfinite == 0 && substep_samples == n_ticks * mj_substeps as usize
            && rig.robot.joints.iter().enumerate()
                .filter(|(_, joint)| joint.joint_type == "revolute")
                .all(|(ji, _)| substep_max_excess[ji] == [0.0; 3]);
        let report = serde_json::json!({"schema": 1, "scope": "MuJoCo physics-step trace after each step",
            "physics_dt_s": mj_dt, "expected_samples": n_ticks * mj_substeps as usize,
            "samples": substep_samples, "nonfinite_values": substep_nonfinite,
            "passed": passed, "joints": joints});
        std::fs::write(path, serde_json::to_vec_pretty(&report).unwrap()).expect("SUBSTEP_LIMIT_AUDIT_JSON");
    }
    if mpc_capture_lex {
        println!("  mpc lex: {wrench_mpc_soft} solves fell back to the soft net-CoP rows");
    }
    if mpc_landing_time {
        println!("  mpc landing time: {n_mpc_retime} steps shortened by the landing stage");
    }
    if cp_lat_emergency > 0.0 {
        println!("  lateral emergency touchdown: {n_lat_emg} steps");
    }
    if mpc_pcmd > 0.0 {
        println!("  mpc pcmd: {n_mpc_pcmd} ticks took the MPC pressure point (fraction {mpc_pcmd})");
    }
    println!("  verdict: {}", if fell { "FELL" } else { "SURVIVED" });
}

#[cfg(not(feature = "mujoco"))]
fn main() {
    eprintln!("This example requires the `mujoco` feature. Run with:");
    eprintln!("  cargo run --features mujoco --example kyo46rs_walk");
}

/// Which lateral direction is "inward" (toward the stance foot) for the
/// landing of `swing` planned at slice `i`: returns (lateral gap, true when
/// inward is +y). The gap is |stance.y - landing.y| in world y.
fn landing_inward(
    footsteps: &articara::biped::gait::FootstepPlan,
    i: usize,
    swing: usize,
) -> (f64, bool) {
    let f = footsteps.at_slice(i);
    let stance = 1 - swing;
    let dy = f.sole[stance].y - f.sole[swing].y;
    (dy.abs(), dy > 0.0)
}
