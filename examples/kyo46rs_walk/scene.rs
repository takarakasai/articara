//! The scenario: what the world is, apart from what the robot knows.
//!
//! The plant's build options (`plant_from_env`), the scenery appended to the
//! MJCF (`build_scene`: the sparring target, the staircase or slope, the
//! unmodelled bumps), the one push (`PushScenario`, `fire_push`, with the
//! jab's phase at the push) and the FIX probe (`FixProbe`). Only the
//! staircase (`Scene::stairs`, the planner's terrain model) reaches the
//! walker. Moved out of main.rs in S15 of
//! doc/from_claude/20261029_walker_extraction_plan.md, expression for
//! expression.
use crate::{env_f64, flag};
use articara::biped::profile::Profile;
use articara::biped::rig::{CtrlMode, RigOptions};
use misa_biped::walker::CommandSchedule;

/// The plant's build options from the environment (the joint seeds and
/// damping, armature, the burn-in, the control mode, the ground's friction
/// and contact, the arm and stance seeds, `INIT_JITTER`), and the profile
/// with its `URDF` / `ROOT_LINK` / `TRUNK_LINK` overrides (printing the
/// first). Read in this order, after the walker's Config.
pub fn plant_from_env(prof: Profile) -> (RigOptions, Profile) {
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
    // MJ_NOSLIP=N: MuJoCo's noslip post-pass. Measured 2026-10-15 on the
    // slope bench: a loaded stance foot creeps 0.2-0.5 cm/s along a 3-5 deg
    // slope with the default solver (flat: 0.03), which is the solver's
    // soft-contact artefact, not the controller. Unset = plant unchanged.
    o.noslip_iterations = std::env::var("MJ_NOSLIP").ok().and_then(|v| v.parse::<u32>().ok()).filter(|&n| n > 0);
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
    (o, prof)
}

/// What `build_scene` put into the world besides the robot. `stairs` is also
/// the planner's terrain model; `spar_wall` (0 = none) is read back by the
/// target-hit report.
pub struct Scene {
    pub stairs: Option<articara::biped::terrain::Stairs>,
    pub spar_wall: f64,
}

/// The scenario's scenery, as MJCF appended to `o.extra_worldbody` (and the
/// spawn height it implies): `SPAR_WALL`, `TERRAIN_SLOPE` / `STAIRS`, then
/// `TERRAIN_BUMPS`, in that order, printing as it goes. Kept out of `main` so
/// the scenario (what the world is) reads apart from the controller (what the
/// robot knows); only `stairs` reaches the controller.
pub fn build_scene(o: &mut RigOptions) -> Scene {
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
    // TERRAIN_SLOPE="deg[,start[,run]]": a known slope (in the planner's
    // terrain model) instead of a flight. Takes precedence over STAIRS.
    let terrain_slope: Option<Stairs> = std::env::var("TERRAIN_SLOPE").ok().map(|v| {
        let f: Vec<f64> = v.split(',').filter_map(|x| x.trim().parse().ok()).collect();
        Stairs::slope(f.first().copied().unwrap_or(0.0), f.get(1).copied().unwrap_or(1.0), f.get(2).copied().unwrap_or(5.0), stair_landing_len)
    });
    let stairs: Option<Stairs> = terrain_slope.or_else(|| std::env::var("STAIRS")
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
        }));
    if let Some(st) = &stairs {
        let half_w = env_f64("STAIR_HALF_W", 0.30);
        let mut wb = String::new();
        if st.n == 0 && st.slope != 0.0 {
            // A rotated slab whose top surface starts at (start, base) and
            // rises `slope` per metre over `slope_len`, then a flat landing.
            let th = st.slope.atan();
            let thick = 0.10;
            let len = st.slope_len / th.cos();
            let (cx, cz) = (
                st.start + 0.5 * st.slope_len + 0.5 * thick * th.sin(),
                st.base() + 0.5 * st.slope_len * st.slope - 0.5 * thick * th.cos(),
            );
            wb.push_str(&format!(
                "\n    <body name=\"stair_slope\" pos=\"{cx:.4} 0 {cz:.4}\" euler=\"0 {:.4} 0\">\
                 \n      <geom name=\"stair_slope_geom\" type=\"box\" size=\"{:.4} {half_w:.4} {:.4}\" rgba=\"0.55 0.57 0.62 1\"/>\n    </body>",
                // The exporter declares `<compiler angle="radian"/>`, so
                // euler is in radians (a degrees value here rotated the slab
                // by -3 rad = -172 deg and put its surface nowhere, 2026-10-15).
                -th, 0.5 * len, 0.5 * thick
            ));
            let top = st.base() + st.slope * st.slope_len;
            let x1 = st.start + st.slope_len;
            let mut add_flat = |name: &str, x0: f64, x1: f64, top: f64| {
                if top > 1e-9 {
                    wb.push_str(&format!(
                        "\n    <body name=\"{name}\" pos=\"{:.4} 0 {:.4}\">\n      <geom name=\"{name}_geom\" type=\"box\" size=\"{:.4} {half_w:.4} {:.4}\" rgba=\"0.55 0.57 0.62 1\"/>\n    </body>",
                        0.5 * (x0 + x1), 0.5 * top, 0.5 * (x1 - x0), 0.5 * top
                    ));
                }
            };
            add_flat("stair_slope_landing", x1, x1 + 4.0, top);
            // The spawn platform reaches back under the spawn point (x ~ 0).
            add_flat("stair_slope_platform", (st.start - 0.60).min(-1.0), st.start, st.base());
            o.extra_worldbody = Some(match o.extra_worldbody.take() { Some(s) => s + wb.as_str(), None => wb.clone() });
            if std::env::var("TERRAIN_DEBUG").is_ok() {
                println!("terrain mjcf:{wb}");
            }
            o.spawn_z_offset = st.base() + env_f64("SPAWN_Z_EXTRA", 0.0).max(0.0);
            println!("slope: {:+.2} deg from x={:.2} over {:.2} m (spawn platform {:.4})", th.to_degrees(), st.start, st.slope_len, st.base());
        } else {
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
    }
    // TERRAIN_BUMPS="n,h_max,len,x0,x1": n boxes of random height (0, h_max]
    // and length len x (0.5..1.5), at random x in [x0, x1], full width, NOT
    // in the planner's terrain model -- robustness to unmodelled ground.
    // TERRAIN_SEED seeds the draw (the bench sets it per member).
    if let Ok(v) = std::env::var("TERRAIN_BUMPS") {
        let f: Vec<f64> = v.split(',').filter_map(|x| x.trim().parse().ok()).collect();
        let (n, h_max, len, x0, x1) = (
            f.first().copied().unwrap_or(0.0) as usize,
            f.get(1).copied().unwrap_or(0.02),
            f.get(2).copied().unwrap_or(0.25),
            f.get(3).copied().unwrap_or(1.0),
            f.get(4).copied().unwrap_or(4.0),
        );
        let half_w = env_f64("STAIR_HALF_W", 0.30);
        let mut seed = (env_f64("TERRAIN_SEED", 1.0).max(1.0) as u64).wrapping_mul(0x9E37_79B9_7F4A_7C15) | 1;
        let mut rnd = || {
            seed ^= seed << 13;
            seed ^= seed >> 7;
            seed ^= seed << 17;
            (seed >> 11) as f64 / (1u64 << 53) as f64
        };
        let mut wb = String::new();
        let mut log = Vec::new();
        for k in 0..n {
            let h = h_max * (0.3 + 0.7 * rnd());
            let l = len * (0.5 + rnd());
            let xs = x0 + rnd() * (x1 - x0 - l).max(0.0);
            wb.push_str(&format!(
                "\n    <body name=\"stair_bump_{k}\" pos=\"{:.4} 0 {:.4}\">\n      <geom name=\"stair_bump_{k}_geom\" type=\"box\" size=\"{:.4} {half_w:.4} {:.4}\" rgba=\"0.62 0.55 0.50 1\"/>\n    </body>",
                xs + 0.5 * l, 0.5 * h, 0.5 * l, 0.5 * h
            ));
            log.push(format!("[{xs:.2}+{l:.2} h{:.0}mm]", h * 1e3));
        }
        o.extra_worldbody = Some(match o.extra_worldbody.take() { Some(s) => s + wb.as_str(), None => wb });
        println!("bumps (unmodelled): {}", log.join(" "));
    }
    Scene { stairs, spar_wall }
}

/// One rectangular push at a chosen gait phase (`PUSH_*`): the scenario's
/// disturbance, fired once by `fire_push`. The measurement thresholds
/// (`PUSH_RECOVER_*`, `PUSH_WINDOW_S`, ...) stay with the measurements.
pub struct PushScenario {
    /// Sized in IMPULSE (N*s), not force, because that is the quantity the
    /// machine's recovery budget is denominated in: an ankle strategy can
    /// absorb `m * omega * d_cop` of it and no more, which on this robot is
    /// 6.63 * 5.585 * 0.019 = 0.70 N*s sideways against 1.8 N*s fore-aft.
    /// Quoting a force instead hides the duration and makes two runs with the
    /// same "10 N push" incomparable.
    pub imp: f64,
    /// Horizontal direction, degrees: 0 = +x (forward), 90 = +y (left). The
    /// whole circle is reachable, so diagonals are one number rather than a
    /// pair of components that have to be kept consistent with each other.
    pub deg: f64,
    /// Yaw impulse (N*m*s) about +z, independent of the linear one so a pure
    /// twist is expressible.
    pub yaw_imp: f64,
    pub dt: f64,
    /// WHEN. Not a wall-clock time: what the machine can absorb differs by
    /// several times between double and single support, so a ladder that does
    /// not pin the phase measures the phase as much as the impulse. Fire on
    /// the first tick that is inside step `step`, in the requested
    /// support, at or past `at` of the way through that slice.
    pub step: usize,
    pub at: f64,
    /// "ds" | "ss" | "any" -- which support phase to catch.
    pub support: String,
    /// The link the push (and the FIX probe) is applied to.
    pub link: String,
}

impl PushScenario {
    pub fn from_env(root_link: &str) -> Self {
        Self {
            imp: env_f64("PUSH_IMPULSE", 0.0),
            deg: env_f64("PUSH_DEG", 90.0),
            yaw_imp: env_f64("PUSH_YAW_IMPULSE", 0.0),
            dt: env_f64("PUSH_DT", 0.10).max(1e-6),
            step: env_f64("PUSH_STEP", 5.0) as usize,
            at: env_f64("PUSH_AT", 0.5).clamp(0.0, 1.0),
            support: std::env::var("PUSH_SUPPORT").unwrap_or_else(|_| "ss".into()),
            link: std::env::var("PUSH_LINK").unwrap_or_else(|_| root_link.to_string()),
        }
    }
}

/// Where the gait is on this tick, as far as the push trigger cares.
pub struct PushTick {
    pub t: f64,
    pub step_idx: usize,
    pub support: articara::biped::gait::Support,
    pub slice_frac: f64,
    /// End of the current slice (the pulse-spill warning).
    pub slice_t1: f64,
}

/// Fire the scenario's push once, at the requested phase. Called after the
/// plant write and before the plant step, so the push lands
/// on the same tick boundary as the torques it has to fight, and the
/// controller sees the consequence on the NEXT tick's measurement --
/// which is the causal order a real shove has. The caller asks only while
/// nothing has fired yet; true = fired on this tick (the caller records it).
/// `jab` is the command when it jabs (the push line says where the jab was).
pub fn fire_push(
    sim: &mut articara::mujoco_sim::MujocoSim,
    push: &PushScenario,
    tick: &PushTick,
    total_mass: f64,
    omega: f64,
    jab: Option<&CommandSchedule>,
) -> bool {
    use articara::biped::gait::Support;
    let &PushScenario { imp: push_imp, deg: push_deg, yaw_imp: push_yaw_imp, dt: push_dt, step: push_step, at: push_at, .. } = push;
    let &PushTick { t, step_idx, support, slice_frac, slice_t1 } = tick;
    if !(push_imp.abs() > 0.0 || push_yaw_imp.abs() > 0.0) {
        return false;
    }
    let phase_ok = match push.support.as_str() {
        "ds" => matches!(support, Support::Double),
        "ss" => matches!(support, Support::Single { .. }),
        _ => true,
    };
    if step_idx == push_step && phase_ok && slice_frac >= push_at {
        let th = push_deg.to_radians();
        let f = push_imp / push_dt;
        let (fx, fy) = (f * th.cos(), f * th.sin());
        let tz = push_yaw_imp / push_dt;
        sim.apply_external_force(&push.link, [fx, fy, 0.0], [0.0, 0.0, tz], push_dt);
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
                Support::Flight { .. } => "FL".to_string(),
            },
            dv / omega * 1e3,
        );
        // Where the jab was when the shove landed. The jab runs on its
        // own clock, unrelated to the gait, so without this line a
        // push-vs-jab comparison is averaging over an unrecorded phase
        // -- and "pushed at full extension" and "pushed back in guard"
        // are not the same experiment.
        if let Some(jab) = jab {
            let j = jab.jab_at(t);
            let side = if j.side == 0 { "L" } else { "R" };
            println!(
                "  PUSH JAB PHASE: side={side} phase={:.3} extension={:.3} \
                 (0 = in guard, 1 = fully extended)",
                j.phase, j.extension
            );
        }
        // If the pulse outlives the slice it was fired into, the run
        // is not measuring what the label says. Firing 0.10 s into a
        // 0.20 s double-support at frac 0.5 puts the tail of the pulse
        // exactly on liftoff, and the resulting "double support"
        // number came out BELOW the single-support one -- the phase
        // was being measured, not the impulse. Say so rather than
        // letting the row be compared with the others.
        let slice_left = slice_t1 - t;
        if push_dt > slice_left {
            println!(
                "  PUSH WARNING: the {push_dt:.3} s pulse outlasts this \
                 slice by {:.3} s, so it spills into the next support \
                 phase. Lower PUSH_AT or PUSH_DT to keep the \
                 disturbance inside one phase.",
                push_dt - slice_left
            );
        }
        return true;
    }
    false
}

/// The corrective impulse at the commitment instant (`FIX_*`).
///
/// A causal probe, not another correlate. Six quantities have now
/// been found that separate survivors from fallers; the three that could be
/// moved changed nothing (Sec.22.5). Separation cannot tell a cause you can
/// act on from a symptom, because a trajectory heading for a fall departs
/// from a healthy one in every quantity at once.
///
/// So intervene instead: at the instant the plan commits to one foot, add
/// an impulse that changes the CoM velocity by a chosen amount along a
/// chosen axis, and sweep it. If some value saves the run, that axis
/// carries the fatal information and the control problem is to produce that
/// velocity change. If no value on either axis saves it, the state at
/// commitment is not where the cause lives, and the search moves on
/// without another round of correlate-chasing.
pub struct FixProbe {
    pub dv: f64,
    pub deg: f64,
    pub dt: f64,
    /// Angular version of the same probe. A linear impulse cannot touch the
    /// attitude/angular-rate part of the state, and sweeping +-0.15 m/s of CoM
    /// velocity -- five times the disturbance that caused the fall -- saved 1
    /// of 4 runs. So the next place to look is the part a linear impulse
    /// cannot reach. `tau` is an ANGULAR impulse [N*m*s] about the world
    /// axis named by `tau_axis` (0 = roll, 1 = pitch, 2 = yaw).
    pub tau: f64,
    pub tau_axis: usize,
    /// WHEN to apply the corrective impulse, as seconds after the push.
    /// Negative (the default) means "at the commitment instant", which is where
    /// the first two probes fired -- and neither +-0.15 m/s of CoM velocity nor
    /// +-1.5 N*m*s of angular impulse saved 3 of the 4 runs there. By then it
    /// is too late. Sweeping this backwards towards the push finds the boundary
    /// of where it is still early enough, which is a control BANDWIDTH
    /// requirement rather than another correlate.
    pub at_s: f64,
}

impl FixProbe {
    pub fn from_env() -> Self {
        Self {
            dv: env_f64("FIX_DV", 0.0),
            deg: env_f64("FIX_DEG", 90.0),
            dt: env_f64("FIX_DT", 0.02),
            tau: env_f64("FIX_TAU", 0.0),
            tau_axis: env_f64("FIX_TAU_AXIS", 0.0) as usize,
            at_s: env_f64("FIX_AT_S", -1.0),
        }
    }

    pub fn armed(&self) -> bool {
        self.dv.abs() > 0.0 || self.tau.abs() > 0.0
    }

    /// Apply the impulse to `link` and print it; `at_commitment` picks which
    /// of the two triggers fired (the two lines differ).
    pub fn fire(&self, sim: &mut articara::mujoco_sim::MujocoSim, link: &str, total_mass: f64, t: f64, at_commitment: bool) {
        let &FixProbe { dv: fix_dv, deg: fix_deg, dt: fix_dt, tau: fix_tau, tau_axis: fix_tau_axis, at_s: fix_at_s } = self;
        let th = fix_deg.to_radians();
        let f = total_mass * fix_dv / fix_dt;
        let mut tq = [0.0; 3];
        if fix_tau_axis < 3 {
            tq[fix_tau_axis] = fix_tau / fix_dt;
        }
        sim.apply_external_force(link, [f * th.cos(), f * th.sin(), 0.0], tq, fix_dt);
        if at_commitment {
            println!(
                "  FIX at t={t:.3} (commitment): dv={fix_dv:+.3} m/s at \
                 {fix_deg:.0} deg, angular {fix_tau:+.4} N*m*s about axis \
                 {fix_tau_axis}, over {fix_dt:.3} s"
            );
        } else {
            println!(
                "  FIX at t={t:.3} (+{fix_at_s:.2} s after the push): \
                 dv={fix_dv:+.3} m/s at {fix_deg:.0} deg, angular {fix_tau:+.3}"
            );
        }
    }
}
