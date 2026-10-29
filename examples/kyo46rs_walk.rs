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
#[cfg(feature = "mujoco")]
use misa_biped::recovery::{RefLib, RefMode};

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

/// The control loop's debug prints. Each was an environment read inside the
/// tick (or a `let` scattered among the knobs); the environment does not
/// change during a run, so reading them once is the same switch.
#[cfg(feature = "mujoco")]
struct DebugFlags {
    /// CONTACT_ADMIT_DEBUG: the `[adm]` lines (admit certificates).
    contact_admit: bool,
    /// DS_FEASIBLE_DEBUG: the `[ds]`/`[dsf]` lines.
    ds_feasible: bool,
    /// DS_CAPTURE_DEBUG: the `[dsc-cap]` lines.
    ds_capture: bool,
    /// DEBUG_ANCHOR=1: the `[anchor]` lines (a switch: "0" = off).
    anchor: bool,
    /// CP_DEBUG: the `[cp]` lines (contact_plan).
    cp: bool,
    /// PUSH_REPLAN_DEBUG: the `[replan]` lines.
    push_replan: bool,
    /// MPC_FS_CROSS_DEBUG: the `[fs-cross]` lines (the crossing gate).
    mpc_fs_cross: bool,
    /// MPC_FS_DEBUG_LANDING: the `[fs-decision]` lines.
    mpc_fs_landing: bool,
    /// MPC_TRAJ_DEBUG: the `[traj]` lines (before t = 2 s).
    mpc_traj: bool,
    /// DIAG_REPLAN=1: print each live replan with its site (a switch).
    replan: bool,
    /// WBC_LEVEL_DEBUG_T=<t>: the `[wbc-levels]` lines within 11 ms of t.
    /// Unparsable = None, as before (silently off).
    wbc_level_t: Option<f64>,
}

#[cfg(feature = "mujoco")]
impl DebugFlags {
    fn from_env() -> Self {
        let set = |k: &str| std::env::var(k).is_ok();
        Self {
            contact_admit: set("CONTACT_ADMIT_DEBUG"),
            ds_feasible: set("DS_FEASIBLE_DEBUG"),
            ds_capture: set("DS_CAPTURE_DEBUG"),
            anchor: flag("DEBUG_ANCHOR", false),
            cp: set("CP_DEBUG"),
            push_replan: set("PUSH_REPLAN_DEBUG"),
            mpc_fs_cross: set("MPC_FS_CROSS_DEBUG"),
            mpc_fs_landing: set("MPC_FS_DEBUG_LANDING"),
            mpc_traj: set("MPC_TRAJ_DEBUG"),
            replan: flag("DIAG_REPLAN", false),
            wbc_level_t: std::env::var("WBC_LEVEL_DEBUG_T").ok().and_then(|v| v.parse::<f64>().ok()),
        }
    }
}

/// WBCW_<NAME>: the weighted strategy's weight of one WBC level, NAME being
/// the level name upper-cased with non-alphanumerics as '_' (e.g. WBCW_COM,
/// WBCW_SWING, WBCW_POSTURE_MOMENTUM). The default depends on the level.
#[cfg(feature = "mujoco")]
fn level_weight_from_env(name: &str) -> f64 {
    let key: String = name
        .chars()
        .map(|c| if c.is_ascii_alphanumeric() { c.to_ascii_uppercase() } else { '_' })
        .collect();
    let default = match name {
        "com" | "com+momentum+wrench" => 1.0,
        "swing" | "contact recovery" => 1.0,
        "trunk" => 0.3,
        "momentum" => 0.1,
        "posture" | "posture+momentum" => 0.01,
        "regularise" => 1e-4,
        _ => 0.1,
    };
    env_f64(&format!("WBCW_{key}"), default)
}

/// The per-level weights of the weighted strategy, read once for every level
/// name the stack in `main` produces (they were read every tick). An array,
/// not a map, so a lookup cannot depend on an iteration order; a name not in
/// the table is read on the spot, as before.
#[cfg(feature = "mujoco")]
struct LevelWeights([(&'static str, f64); 13]);

#[cfg(feature = "mujoco")]
impl LevelWeights {
    const NAMES: [&'static str; 13] = [
        "eom",
        "contact+cones",
        "dynamics+contact+cones",
        "com",
        "com+momentum+wrench",
        "trunk",
        "momentum",
        "arm hold(before swing)",
        "swing",
        "arm hold",
        "posture",
        "posture+momentum",
        "regularise",
    ];

    fn from_env() -> Self {
        Self(Self::NAMES.map(|name| (name, level_weight_from_env(name))))
    }

    fn get(&self, name: &str) -> f64 {
        self.0
            .iter()
            .find(|(n, _)| *n == name)
            .map_or_else(|| level_weight_from_env(name), |&(_, w)| w)
    }
}

/// What `build_scene` put into the world besides the robot. `stairs` is also
/// the planner's terrain model; `spar_wall` (0 = none) is read back by the
/// target-hit report.
#[cfg(feature = "mujoco")]
struct Scene {
    stairs: Option<articara::biped::terrain::Stairs>,
    spar_wall: f64,
}

/// The scenario's scenery, as MJCF appended to `o.extra_worldbody` (and the
/// spawn height it implies): `SPAR_WALL`, `TERRAIN_SLOPE` / `STAIRS`, then
/// `TERRAIN_BUMPS`, in that order, printing as it goes. Kept out of `main` so
/// the scenario (what the world is) reads apart from the controller (what the
/// robot knows); only `stairs` reaches the controller.
#[cfg(feature = "mujoco")]
fn build_scene(o: &mut articara::biped::rig::RigOptions) -> Scene {
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
#[cfg(feature = "mujoco")]
struct PushScenario {
    /// Sized in IMPULSE (N*s), not force, because that is the quantity the
    /// machine's recovery budget is denominated in: an ankle strategy can
    /// absorb `m * omega * d_cop` of it and no more, which on this robot is
    /// 6.63 * 5.585 * 0.019 = 0.70 N*s sideways against 1.8 N*s fore-aft.
    /// Quoting a force instead hides the duration and makes two runs with the
    /// same "10 N push" incomparable.
    imp: f64,
    /// Horizontal direction, degrees: 0 = +x (forward), 90 = +y (left). The
    /// whole circle is reachable, so diagonals are one number rather than a
    /// pair of components that have to be kept consistent with each other.
    deg: f64,
    /// Yaw impulse (N*m*s) about +z, independent of the linear one so a pure
    /// twist is expressible.
    yaw_imp: f64,
    dt: f64,
    /// WHEN. Not a wall-clock time: what the machine can absorb differs by
    /// several times between double and single support, so a ladder that does
    /// not pin the phase measures the phase as much as the impulse. Fire on
    /// the first tick that is inside step `step`, in the requested
    /// support, at or past `at` of the way through that slice.
    step: usize,
    at: f64,
    /// "ds" | "ss" | "any" -- which support phase to catch.
    support: String,
    /// The link the push (and the FIX probe) is applied to.
    link: String,
}

#[cfg(feature = "mujoco")]
impl PushScenario {
    fn from_env(root_link: &str) -> Self {
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
#[cfg(feature = "mujoco")]
struct PushTick {
    t: f64,
    step_idx: usize,
    support: articara::biped::gait::Support,
    slice_frac: f64,
    /// End of the current slice (the pulse-spill warning).
    slice_t1: f64,
}

/// The jab's own clock (`JAB_START`, `JAB_PERIOD`, `JAB_PATTERN`), for the
/// push line that says where the jab was when the shove landed.
#[cfg(feature = "mujoco")]
struct JabClock<'a> {
    start: f64,
    period: f64,
    pattern: &'a str,
}

/// Fire the scenario's push once, at the requested phase. Called after the
/// plant write and before the plant step, so the push lands
/// on the same tick boundary as the torques it has to fight, and the
/// controller sees the consequence on the NEXT tick's measurement --
/// which is the causal order a real shove has. The caller asks only while
/// nothing has fired yet; true = fired on this tick (the caller records it).
#[cfg(feature = "mujoco")]
fn fire_push(
    sim: &mut articara::mujoco_sim::MujocoSim,
    push: &PushScenario,
    tick: &PushTick,
    total_mass: f64,
    omega: f64,
    jab: Option<JabClock>,
) -> bool {
    use articara::biped::gait::Support;
    use std::f64::consts::PI;
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
            },
            dv / omega * 1e3,
        );
        // Where the jab was when the shove landed. The jab runs on its
        // own clock, unrelated to the gait, so without this line a
        // push-vs-jab comparison is averaging over an unrecorded phase
        // -- and "pushed at full extension" and "pushed back in guard"
        // are not the same experiment.
        if let Some(jab) = jab {
            let ph = ((t - jab.start) / jab.period).max(0.0);
            let side = if jab.pattern
                .chars()
                .nth(ph.floor() as usize % jab.pattern.chars().count())
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
#[cfg(feature = "mujoco")]
struct FixProbe {
    dv: f64,
    deg: f64,
    dt: f64,
    /// Angular version of the same probe. A linear impulse cannot touch the
    /// attitude/angular-rate part of the state, and sweeping +-0.15 m/s of CoM
    /// velocity -- five times the disturbance that caused the fall -- saved 1
    /// of 4 runs. So the next place to look is the part a linear impulse
    /// cannot reach. `tau` is an ANGULAR impulse [N*m*s] about the world
    /// axis named by `tau_axis` (0 = roll, 1 = pitch, 2 = yaw).
    tau: f64,
    tau_axis: usize,
    /// WHEN to apply the corrective impulse, as seconds after the push.
    /// Negative (the default) means "at the commitment instant", which is where
    /// the first two probes fired -- and neither +-0.15 m/s of CoM velocity nor
    /// +-1.5 N*m*s of angular impulse saved 3 of the 4 runs there. By then it
    /// is too late. Sweeping this backwards towards the push finds the boundary
    /// of where it is still early enough, which is a control BANDWIDTH
    /// requirement rather than another correlate.
    at_s: f64,
}

#[cfg(feature = "mujoco")]
impl FixProbe {
    fn from_env() -> Self {
        Self {
            dv: env_f64("FIX_DV", 0.0),
            deg: env_f64("FIX_DEG", 90.0),
            dt: env_f64("FIX_DT", 0.02),
            tau: env_f64("FIX_TAU", 0.0),
            tau_axis: env_f64("FIX_TAU_AXIS", 0.0) as usize,
            at_s: env_f64("FIX_AT_S", -1.0),
        }
    }

    fn armed(&self) -> bool {
        self.dv.abs() > 0.0 || self.tau.abs() > 0.0
    }

    /// Apply the impulse to `link` and print it; `at_commitment` picks which
    /// of the two triggers fired (the two lines differ).
    fn fire(&self, sim: &mut articara::mujoco_sim::MujocoSim, link: &str, total_mass: f64, t: f64, at_commitment: bool) {
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

/// Per-step accounting, printed at the end. A run-total hides the thing
/// that matters, which is whether the steps are all alike -- a periodic
/// orbit -- or drifting.
#[cfg(feature = "mujoco")]
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

/// What the run measures about itself: achievement, extremes, the target
/// hits, the push response and the weight transfer after it. Harness state:
/// written from the plant, the solve and the plan, read by the report after
/// the loop, and never by the controller -- except `push_at_t`, which also
/// gates the FIX probe and the push-window bookkeeping (both scenario).
#[cfg(feature = "mujoco")]
struct RunMetrics {
    step_stats: Vec<StepStat>,
    walk_end_state: Option<([f64; 3], f64, f64)>,
    /// Pose at the end of the stride ramp (`t_ramp_end`): achievement is
    /// measured from there.
    ramp_end_state: Option<([f64; 3], f64, f64)>,
    /// Last state before the loop ended, so a case that FELL reports the rate
    /// it was actually holding rather than a post-fall pose divided by a
    /// duration that never happened.
    last_state: ([f64; 3], f64, f64),
    /// Command tracking is integrated in the BODY frame, tick by tick, rather
    /// than taken as a displacement in the start frame. Two reasons, both
    /// measured: euler yaw wraps at +-pi, so a turn past 180 deg read as
    /// -69% of its command; and a curved path's chord is not its arc, so
    /// vx+wz together read 56% on a run whose speed was right.
    travel: [f64; 3], // body-frame x, y, unwrapped yaw
    prev_pose: Option<([f64; 3], f64)>,
    fell: bool,
    min_z: f64,
    max_tilt: f64,
    max_dcm_err: f64,
    max_cop_use: f64,
    max_zmp_clamp: f64,
    max_ankle_roll_use: f64,
    n_open_loop: u32,
    /// Per-joint travel, so a joint that swings without hitting anything is
    /// still visible. Sec.30 found the v6 arm at 111 deg peak-to-peak in a clip
    /// that logged 44 self-collision ticks: the collision count is a count of
    /// collisions, and it was being read as if it were a measure of behaviour.
    q_lo: Vec<f64>,
    q_hi: Vec<f64>,
    /// Hits on `spar_target`, kept OUT of the self-collision tally: a contact
    /// with the thing the robot is aiming at is not the QP solving against a
    /// contact it has no model for, which is what that counter means.
    hit_ticks: u32,
    hit_peak_f: f64,
    hit_impulse: f64,
    hit_first_t: Option<f64>,
    hit_last_t: f64,
    /// Per-PUNCH, not per-run: the totals above sum every contact in the run,
    /// and "how long does one punch stay on the target" is the question Sec.42.4
    /// actually asked. An episode is a contiguous run of ticks in contact.
    hit_episodes: Vec<(f64, f64)>, // (duration s, impulse N*s)
    ep_t0: Option<f64>,
    ep_imp: f64,
    /// This tick's force on the target, hoisted so the CSV can carry it: a
    /// replay that draws the punch should not have to be told separately
    /// what it delivered (same reason `push_fx`/`push_fy` are logged).
    hit_f_now: f64,
    n_selfcollide: u32,
    max_selfcollide_f: f64,
    // ---- disturbance response ------------------------------------------
    /// Whole-run maxima say nothing about a push: they are dominated by the
    /// walk itself. Everything about the recovery has to be measured from the
    /// moment of the push forward, which is what these hold.
    push_at_t: Option<f64>,
    push_at_step: usize,
    push_dcm_err_max: f64,
    push_tilt_max: f64,
    push_degraded_at: u32,
    push_cop_max: f64,
    push_slip_wbc_max: f64,
    push_slip_plant_max: f64,
    push_slip_ticks: u32,
    /// Landing impact and the unloading that follows it. Measured on the
    /// straddling pair at 0.20 (fell) against 0.15 and 0.30 (both survived):
    /// the faller spiked to 2.77x body weight and then spent 100 ms with the
    /// two feet carrying less than half the robot's weight between them --
    /// effectively ballistic -- while both survivors never dropped below 0.63x
    /// and peaked at 1.22x / 1.41x. Neither friction use nor CoP use nor
    /// ankle_roll range separated those three runs; this did.
    push_fz_total_min: f64,
    push_fz_total_max: f64,
    push_unloaded_ticks: u32,
    /// Early-window counterparts (see `push_window_s`).
    win_dcm_err_max: f64,
    win_tilt_max: f64,
    win_cop_max: f64,
    win_slip_plant_max: f64,
    win_slip_ticks: u32,
    win_fz_total_min: f64,
    win_fz_total_max: f64,
    win_unloaded_ticks: u32,
    win_degraded_at: Option<u32>,
    win_degraded_end: u32,
    /// First support change after the push, and the CoP-edge accounting over
    /// the `transfer_s` that follows it.
    transfer_t: Option<f64>,
    support_prev: Option<articara::biped::gait::Support>,
    xfer_edge_ticks: u32,
    xfer_ticks: u32,
    xfer_leave_t: Option<f64>,
    xfer_wrongfoot_ticks: u32,
    /// Second single-support entry after the push. The first-window metric is
    /// anchored to a transition that step-timing adaptation MOVES, so it scores
    /// that intervention badly by construction (+27/-1 survivals against only
    /// 36/29 on the metric). Measuring the next step too separates "the metric
    /// is truncated" from "the intervention only helps survival".
    transfer2_t: Option<f64>,
    xfer2_edge_ticks: u32,
    xfer2_ticks: u32,
    /// Centroidal angular momentum about the roll axis. If damping it is going
    /// to help, it has to be large and growing in runs that fall -- and the
    /// arms are shoulder_pitch + elbow, both PITCH, so they cannot make roll
    /// momentum at all. Measured before implementing the task, not after.
    h_roll_max: f64,
    h_roll_at_xfer: f64,
    /// DCM offset relative to the foot the plan is about to commit to, at the
    /// instant of commitment. The Khadiv/Englsberger nominal for a periodic
    /// walk is b_y = l_p / (1 + exp(omega * t_ss)) with the sign alternating;
    /// entering single support at the wrong sign or magnitude is the candidate
    /// mechanism for the failures traced in Sec.19-21. Zero new machinery is
    /// needed to check it -- the footstep plan and the measured DCM are both
    /// already here.
    xfer_dcm_off: [f64; 2],
    xfer_stance: i32,
    push_ankle_max: f64,
    push_exceeded: bool,
    push_last_exceed: Option<f64>,
    t_last: f64,
    /// The FIX probe fires at most once per run.
    fix_fired: bool,
}

#[cfg(feature = "mujoco")]
impl RunMetrics {
    /// Everything zero / empty, except the joint-travel extremes (sized by
    /// `n_joints`) and `last_state`, which starts at the spawn pose.
    fn new(n_joints: usize, p0_body: [f64; 3], yaw0_body: f64) -> Self {
        Self {
            step_stats: Vec::new(),
            walk_end_state: None,
            ramp_end_state: None,
            last_state: (p0_body, yaw0_body, 0.0),
            travel: [0.0f64; 3],
            prev_pose: None,
            fell: false,
            min_z: f64::INFINITY,
            max_tilt: 0.0,
            max_dcm_err: 0.0,
            max_cop_use: 0.0,
            max_zmp_clamp: 0.0,
            max_ankle_roll_use: 0.0,
            n_open_loop: 0,
            q_lo: vec![f64::INFINITY; n_joints],
            q_hi: vec![f64::NEG_INFINITY; n_joints],
            hit_ticks: 0,
            hit_peak_f: 0.0,
            hit_impulse: 0.0,
            hit_first_t: None,
            hit_last_t: 0.0,
            hit_episodes: Vec::new(),
            ep_t0: None,
            ep_imp: 0.0,
            hit_f_now: 0.0,
            n_selfcollide: 0,
            max_selfcollide_f: 0.0,
            push_at_t: None,
            push_at_step: 0,
            push_dcm_err_max: 0.0,
            push_tilt_max: 0.0,
            push_degraded_at: 0,
            push_cop_max: 0.0,
            push_slip_wbc_max: 0.0,
            push_slip_plant_max: 0.0,
            push_slip_ticks: 0,
            push_fz_total_min: f64::INFINITY,
            push_fz_total_max: 0.0,
            push_unloaded_ticks: 0,
            win_dcm_err_max: 0.0,
            win_tilt_max: 0.0,
            win_cop_max: 0.0,
            win_slip_plant_max: 0.0,
            win_slip_ticks: 0,
            win_fz_total_min: f64::INFINITY,
            win_fz_total_max: 0.0,
            win_unloaded_ticks: 0,
            win_degraded_at: None,
            win_degraded_end: 0,
            transfer_t: None,
            support_prev: None,
            xfer_edge_ticks: 0,
            xfer_ticks: 0,
            xfer_leave_t: None,
            xfer_wrongfoot_ticks: 0,
            transfer2_t: None,
            xfer2_edge_ticks: 0,
            xfer2_ticks: 0,
            h_roll_max: 0.0,
            h_roll_at_xfer: 0.0,
            xfer_dcm_off: [0.0f64; 2],
            xfer_stance: -1,
            push_ankle_max: 0.0,
            push_exceeded: false,
            push_last_exceed: None,
            t_last: 0.0,
            fix_fired: false,
        }
    }
}

/// Where the run writes, besides stdout: TRAJ_CSV (`traj`), and the opt-in
/// diagnostics -- the `*_CSV` files, the QP / MPC snapshot directories and
/// the physics-rate audit. Harness state: the controller writes into some of
/// them on the way, but none of them is read back by it (except that
/// `qp_runtime_log` / `qp_diag_dir` being set makes the solve keep its
/// physical rows, see `qp_physical`). Opened together in `open`; a file that
/// is not asked for is `None`.
#[cfg(feature = "mujoco")]
struct DiagSinks {
    traj: articara::biped::log::TrajLog,
    /// TICK_AUDIT_CSV: per-joint pre/post state around every plant step.
    tick_audit: Option<articara::biped::tick_audit::TickAudit>,
    /// QP_DIAG_DIR: capture immutable task stacks for offline numerical
    /// diagnosis only (`tick_NNNNNN.json`, numbered by `qp_diag_count`).
    qp_diag_dir: Option<String>,
    qp_diag_count: usize,
    /// QP_CAPTURE_*: opt-in continuous sampling; legacy failure-only capture
    /// remains default.
    qp_capture_all: bool,
    qp_capture_start: f64,
    qp_capture_end: f64,
    qp_capture_stride: usize,
    qp_capture_max: usize,
    /// QP_RUNTIME_CSV: per tick, the WBC solve's timing and status.
    qp_runtime_log: Option<Csv>,
    /// MPC_RUNTIME_CSV: per MPC solve. The MPC update period and the 5 ms
    /// WBC tick are separate deadlines.
    mpc_runtime_log: Option<Csv>,
    /// MOM_DIAG_CSV: opt-in centroidal diagnostic: the arm columns measure
    /// momentum from arm joint motion, while base-motion inertia remains in
    /// the total-minus-arm term.
    mom_diag_log: Option<Csv>,
    /// QP_WRENCH_CSV: six-axis instrumentation is additive; writes/conversions
    /// are outside control_ns.
    qp_wrench_log: Option<Csv>,
    /// SUBSTEP_LIMIT_AUDIT_JSON: optional physics-rate audit. The normal
    /// trajectory checks only WBC ticks; the simulator trace already contains
    /// q, qdot and applied torque after every MuJoCo step, so no plant or
    /// controller behavior needs to change.
    substep_audit_path: Option<String>,
    substep_max_excess: Vec<[f64; 3]>,
    substep_extrema: Vec<[f64; 4]>,
    substep_samples: usize,
    substep_nonfinite: usize,
    /// COM_PREVIEW_RUNTIME_CSV: per CoM-preview solve.
    com_preview_runtime: Option<Csv>,
    /// TRANSFER_DIAG_CSV: optional P0 diagnostic, sampled entirely before
    /// stepping the plant. A command computed at t is compared with the
    /// observation at t; its physical response must be evaluated at
    /// subsequent observation times.
    transfer_diag: Option<Csv>,
    /// ANKLE_CBF_DIAG_CSV: opt-in acceleration-level audit at the right
    /// ankle-roll stop. The CSV samples the state and QP decision in the same
    /// tick; the plant response appears at later samples, so
    /// command-vs-response causality stays clear.
    ankle_cbf_diag: Option<Csv>,
    /// PREVIEW_TRAJ_CSV: every accepted LIPM CoM-preview solve (the
    /// bounded-ZMP receding-horizon planner of COM_REPLAN, src/biped/preview.rs),
    /// one row per horizon sample: the planned ZMP, the predicted CoM at the
    /// sample end and the reference ZMP the plan was given. For the summary
    /// videos (MPC_CSV in examples/render_com_squat_video.py).
    preview_traj_file: Option<Csv>,
    /// MPC_TRAJ_CSV: one row per wrench-MPC solve with what the MPC
    /// planned -- the CoM it starts from and aims at, its predicted CoM at the
    /// end of every node, the net CoP of node 0 and the per-node net-CoP
    /// reference, each contact's origin and planned normal force, and the
    /// landing variable (foot, shift). For the summary videos (drawn by
    /// examples/render_com_squat_video.py with MPC_CSV). The controller and
    /// TRAJ_CSV are untouched either way.
    mpc_traj_file: Option<Csv>,
    /// MPC_DIAG_DIR: every MPC solve's input and config
    /// (`mpc_NNNNNN.json`, numbered by `mpc_diag_count`).
    mpc_diag_dir: Option<String>,
    mpc_diag_count: usize,
}

/// One opt-in diagnostic CSV.
#[cfg(feature = "mujoco")]
type Csv = std::io::BufWriter<std::fs::File>;

#[cfg(feature = "mujoco")]
impl DiagSinks {
    /// `traj` and `tick_audit` are made by the caller (TrajLog prints its
    /// path at that point of the start-up log); the rest is opened here from
    /// the environment, each CSV with its header line.
    fn open(
        traj: articara::biped::log::TrajLog,
        tick_audit: Option<articara::biped::tick_audit::TickAudit>,
        substep_audit_path: Option<String>,
        n_joints: usize,
    ) -> Self {
        use std::io::Write;
        let csv = |key: &str, what: &str, header: &dyn Fn(&mut Csv)| -> Option<Csv> {
            std::env::var(key).ok().map(|path| {
                let mut f = std::io::BufWriter::new(std::fs::File::create(path).expect(what));
                header(&mut f);
                f
            })
        };
        Self {
            traj,
            tick_audit,
            qp_diag_dir: std::env::var("QP_DIAG_DIR").ok(),
            qp_diag_count: 0,
            qp_capture_all: flag("QP_CAPTURE_ALL", false),
            qp_capture_start: env_f64("QP_CAPTURE_START", 0.0),
            qp_capture_end: env_f64("QP_CAPTURE_END", f64::INFINITY),
            qp_capture_stride: env_f64("QP_CAPTURE_STRIDE", 1.0).max(1.0) as usize,
            qp_capture_max: env_f64("QP_CAPTURE_MAX", 20000.0).max(1.0) as usize,
            qp_runtime_log: csv("QP_RUNTIME_CSV", "QP_RUNTIME_CSV", &|f| {
                writeln!(f, "t,solve_ns,check_ns,status,normalized_physical_violation,reset,deadline_miss,control_ns,mpc_age_s,mpc_solves,mpc_failures,prefix_applied,repair_applied,repair_scaled_delta,repair_ns").unwrap();
            }),
            mpc_runtime_log: csv("MPC_RUNTIME_CSV", "MPC_RUNTIME_CSV", &|f| {
                writeln!(f, "t,phase,contact_mask,backend,horizon,update_period_s,solve_ns,deadline_miss,accepted,previous_age_s").unwrap();
            }),
            mom_diag_log: csv("MOM_DIAG_CSV", "MOM_DIAG_CSV", &|f| {
                writeln!(f, "t,h_total_x,h_total_y,h_total_z,h_arm_joint_x,h_arm_joint_y,h_arm_joint_z,h_other_x,h_other_y,h_other_z,fz_measured_l,fz_measured_r,fz_qp_l,fz_qp_r,n_stance").unwrap();
            }),
            qp_wrench_log: csv("QP_WRENCH_CSV", "QP_WRENCH_CSV", &|f| {
                write!(f, "t,side,planned_support,wbc_contact,mpc_valid,planned_valid,wbc_valid,measured_valid,wbc_degraded,mpc_age_s").unwrap();
                for source in ["planned", "mpc", "wbc", "measured"] {
                    for component in ["mx", "my", "mz", "fx", "fy", "fz"] { write!(f, ",{source}_{component}").unwrap(); }
                }
                writeln!(f).unwrap();
            }),
            substep_audit_path,
            substep_max_excess: vec![[0.0_f64; 3]; n_joints],
            substep_extrema: vec![[f64::INFINITY, f64::NEG_INFINITY, 0.0, 0.0]; n_joints],
            substep_samples: 0,
            substep_nonfinite: 0,
            com_preview_runtime: csv("COM_PREVIEW_RUNTIME_CSV", "COM_PREVIEW_RUNTIME_CSV", &|f| {
                writeln!(f, "t,solve_ns,deadline_miss_50ms,accepted,terminal_error_m,applied_lateral,horizon").unwrap();
            }),
            transfer_diag: csv("TRANSFER_DIAG_CSV", "transfer diagnostic CSV", &|f| {
                writeln!(f, "t,dt,bw,phase,high,alpha,stance_mask,measured_mask,mpc_valid,mpc_age,mpc_failed,mpc_ref_l,mpc_ref_r,mpc_fz_l,mpc_fz_r,nominal_world_fz_l,nominal_world_fz_r,qp_world_fz_l,qp_world_fz_r,measured_world_fz_l,measured_world_fz_r,trunk_pitch,trunk_roll,qp_cop_l_x,qp_cop_r_x,measured_cop_l_x,measured_cop_r_x,degraded,ffdc_dz").unwrap();
            }),
            ankle_cbf_diag: csv("ANKLE_CBF_DIAG_CSV", "ankle CBF diagnostic CSV", &|f| {
                writeln!(f, "t,q,v,q_max,cbf_upper,qp_qdd,measured_fz_r,qp_fx_r,qp_fy_r,qp_fz_r,measured_fx_r,measured_fy_r,post_q,post_v,mj_qacc,mj_qfrc_constraint,mj_qfrc_passive,mj_qfrc_actuator,command_tau").unwrap();
            }),
            preview_traj_file: csv("PREVIEW_TRAJ_CSV", "PREVIEW_TRAJ_CSV", &|f| {
                writeln!(f, "t,k,dur,zmp_x,zmp_y,c_x,c_y,rzmp_x,rzmp_y,lo_x,lo_y,hi_x,hi_y").unwrap();
            }),
            mpc_traj_file: csv("MPC_TRAJ_CSV", "MPC_TRAJ_CSV", &|f| {
                let mut h = String::from("t,com_x,com_y,com_z,tgt_x,tgt_y,tgt_z");
                for k in 0..16 {
                    h += &format!(",c{k}_x,c{k}_y,c{k}_z");
                }
                h += ",cop0_x,cop0_y";
                for k in 0..16 {
                    h += &format!(",rcop{k}_x,rcop{k}_y");
                }
                h += ",pl_x,pl_y,pl_z,pr_x,pr_y,pr_z,fz_l,fz_r,on_l,on_r,land_foot,land_dx,land_dy,n_nodes";
                writeln!(f, "{h}").unwrap();
            }),
            mpc_diag_dir: std::env::var("MPC_DIAG_DIR").ok(),
            mpc_diag_count: 0,
        }
    }
}

/// The controller's state carried from tick to tick (the planned
/// schedule, footsteps and DCM aside, which are the plan). Each field is the
/// `let` of the same name before the loop, which gives its initial value;
/// gathered here so what the controller remembers is one thing.
#[cfg(feature = "mujoco")]
struct CtlState {
    /// PUSH_YIELD has slowed the walk (once per run).
    push_yield_done: bool,
    /// Speed-scheduled CoM drop, slewed.
    vx2_z_track: articara::biped::height::SlewedDrop,
    /// PUSH_Z_DROP holds the lowered CoM until this time.
    push_z_drop_until: f64,
    /// Measured-contact correction of the planned contact set.
    correction: articara::biped::gait::ContactCorrection,
    /// Terrain height under the support, slewed.
    terrain_track: f64,
    /// contact_plan: the landing shift applied to `cp_slice`, and the slice
    /// whose end it has retimed (it then owns that end).
    cp_applied: na::Vector2<f64>,
    cp_slice: Option<usize>,
    cp_retimed_slice: Option<usize>,
    /// Contact admission: the last scheduled admit, the sole height measured
    /// at it, and the double support each foot is certified for.
    last_scheduled_admit: [Option<usize>; 2],
    admit_measured_z: [Option<f64>; 2],
    confirmed_admit: [Option<usize>; 2],
    /// Push-recovery reference: the one picked, the cooldown, the lateral
    /// quiet time, and the engaged reference's frame.
    ref_lib: Option<RefLib>,
    ref_cooldown_until: f64,
    ref_lat_quiet_t: f64,
    ref_mode: Option<RefMode>,
    /// DS_CAPTURE retimes a double support once.
    dsc_retimed_last: Option<usize>,
    /// PUSH_REPLAN: (slice, original footsteps, original DCM plan, original
    /// landing, last applied landing), the slice it owns, and the hold shift.
    push_replan_state: Option<(
        usize,
        articara::biped::gait::FootstepPlan,
        articara::biped::dcm::DcmPlan,
        na::Vector2<f64>,
        Option<na::Vector2<f64>>,
    )>,
    push_replan_slice: Option<usize>,
    push_replan_shift: f64,
    /// (slice, inward is +y) latched when the MPC crossing gate opens.
    mpc_fs_cross_latched: Option<(usize, bool)>,
    /// COM_REPLAN has started (the height plan was rebuilt).
    replan_engaged: bool,
    /// CoM preview: the reference ZMP, the predicted CoM and velocity, and
    /// the time of the next solve.
    preview_p: na::Vector2<f64>,
    preview_c: na::Vector2<f64>,
    preview_v: na::Vector2<f64>,
    preview_next: f64,
    /// Support of the previous tick (anchors are released and set on a change).
    prev_support: articara::biped::gait::Support,
    /// Where the swing sole left the ground.
    swing_lift_off: Option<na::Vector3<f64>>,
    /// Load-change rate per foot (the managed-contact trigger).
    fz_prev: [f64; 2],
    fz_rate_ema: [f64; 2],
    managed_until: [f64; 2],
    /// Last commanded horizontal CoM acceleration (COM_XY_JERK_MAX).
    previous_a_xy: Option<na::Vector2<f64>>,
    /// Wrench MPC: the time of the next solve and the held answer.
    wrench_mpc_next: f64,
    wrench_mpc_command: Option<[articara::biped::wrench_mpc::Wrench; 2]>,
    /// MPC_FOOTSTEP: the slice whose landing is being moved, how much of the
    /// MPC's offset is applied, the (first, last) slice it is armed for, and
    /// the last total.
    mpc_fs_slice: Option<usize>,
    mpc_fs_applied: na::Vector2<f64>,
    mpc_fs_armed: Option<(usize, usize)>,
    mpc_fs_last: na::Vector2<f64>,
}

/// The WBC's own state: the solver (warm data), the QP structure of the
/// last tick (a change resets the solver) and the contact anchors.
#[cfg(feature = "mujoco")]
struct WbcState {
    solver: misa_wbc::Solver,
    qp_previous_structure: Option<(Vec<usize>, Vec<(usize, usize, usize)>)>,
    anchors: articara::biped::contact::Anchors,
}

/// Between the solve and the plant: the degraded-solve policy and whether a
/// good command has been seen yet (harness side; see `CommandPolicy`).
#[cfg(feature = "mujoco")]
struct CommandGuard {
    policy: articara::biped::actuate::CommandPolicy,
    qp_has_good_command: bool,
}

/// The controller's counters, reported after the loop (`DegradedTally`, the
/// harness's, stays apart). All start at zero.
#[cfg(feature = "mujoco")]
#[derive(Default)]
struct WalkerStats {
    n_mom_cmp_ticks: u32,
    mom_cmp_peak: f64,
    n_push_z_drop: u32,
    n_cp_shift: u32,
    n_cp_retime: u32,
    cp_retime_total: f64,
    n_confirmed_admissions: u32,
    ref_engagements: u32,
    n_ds_capture: u32,
    ds_capture_total: f64,
    n_ds_feasible: u32,
    n_ds_continuous: u32,
    ds_feasible_total: f64,
    n_push_replan: u32,
    n_mpc_fs_cross: u32,
    preview_count: u32,
    preview_failures: u32,
    preview_error: f64,
    landing_retimes: u32,
    landing_delay: f64,
    n_replan: u32,
    n_corrected: u32,
    wrench_mpc_solves: u32,
    wrench_mpc_failures: u32,
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
        ContactCorrection, FootstepPlan, Footsteps, GaitParams, GaitPlan, Support,
        swing_position, swing_velocity,
    };
    use articara::biped::log::{Row, TrajLog, measure_contacts};
    use articara::biped::profile;
    use articara::biped::rig::{BipedRig, CtrlMode, G, RigOptions};
    use articara::biped::tasks as bt;
    use misa_biped::plan::{DcmParams, Plan};
    use misa_wbc::{Dynamics, Formulation, SolveConfig, Solver, tasks};
    use nalgebra as na;
    use std::f64::consts::PI;
    use std::io::Write;

    // ROUGH_MODE=1: the rough-terrain preset for an operator who knows the
    // ground is uneven. Lift fully before advancing (SWING_Z_HOLD with the
    // drop starting at 0.50 so early landings still work), a 0.15 start
    // delay, and an 8 cm apex. Measured 2026-10-16/17 (N=48, v6.10):
    // unmodelled bumps 10/20/30 mm = 48/46/48 (champion 35/13/5), but the
    // lateral pushes at 0.12 m/s go 157 -> 150, so it is not the default.
    // The mode WINS over the four knobs (the champion's controls carry a
    // CEM-tuned LIFT_H of 0.051, which the mode must replace); the apex is
    // ROUGH_MODE_LIFT_H (0.08).
    if std::env::var("ROUGH_MODE").map(|v| v == "1").unwrap_or(false) {
        // On a KNOWN slope the 8 cm apex falls on +3 deg (measured
        // 2026-10-17, LIFT_H=0.08 alone: 16.0 s), 6 cm walks: the mode
        // uses 6 cm there unless ROUGH_MODE_LIFT_H says otherwise.
        let lift = std::env::var("ROUGH_MODE_LIFT_H")
            .unwrap_or_else(|_| if std::env::var("TERRAIN_SLOPE").is_ok() { "0.06".into() } else { "0.08".into() });
        for (k, v) in [("SWING_Z_HOLD", "1"), ("SWING_Z_DROP", "0.50,1.0"), ("SWING_XY_DELAY", "0.15"), ("LIFT_H", lift.as_str())] {
            // SAFETY: single-threaded here, before any reader of these.
            unsafe { std::env::set_var(k, v) };
        }
        println!("rough mode: SWING_Z_HOLD=1 SWING_Z_DROP=0.50,1.0 SWING_XY_DELAY=0.15 LIFT_H={lift}");
    }

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

    // The scene around the robot: the sparring target, the staircase or
    // slope of the planner's terrain model, and the unmodelled bumps.
    let Scene { stairs, spar_wall } = build_scene(&mut o);
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
    assert!(
        f_max_per_foot >= total_mass * G,
        "f_max ({f_max_per_foot:.1} N) is below the single-support nominal ({:.1} N)",
        total_mass * G
    );

    // ── Control-law knobs ──────────────────────────────────────────────
    let solver = Solver::new();
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
    let qp_has_good_command = false;
    let qp_previous_structure: Option<(Vec<usize>, Vec<(usize, usize, usize)>)> = None;
    let feasibility_tol = env_f64("WBC_FEAS_TOL", SolveConfig::default().feasibility_tol);
    // WBCW_<NAME>: the weighted strategy's per-level weights.
    let level_weights = LevelWeights::from_env();
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
    let weighted_levels = std::env::var("WBC_STRATEGY").as_deref() == Ok("weighted");
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
    let jlim_amax = env_f64("JLIM_AMAX", 50.0);
    let mut jlim_v_max = rig.v_max.clone();
    // Slow the CBF's velocity limit below the mechanical one. (The position
    // band JLIM_Q_MARGIN that shared this block was not adopted and is
    // removed; the message keeps its 0 deg.)
    let jlim_v_frac = env_f64("JLIM_V_FRAC", 1.0).clamp(0.05, 1.0);
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
        q_min: rig.q_min.clone(),
        q_max: rig.q_max.clone(),
        v_max: jlim_v_max,
        a_max: na::DVector::from_element(na_count, jlim_amax),
        alpha1: na::DVector::from_element(na_count, jlim_alpha1),
        alpha2: na::DVector::from_element(na_count, jlim_alpha2),
        alpha3: na::DVector::from_element(na_count, jlim_alpha3),
    };
    let friction_mu: f64 = env_f64("friction_mu", mu_ground * prof.friction_margin);
    let kp_com = env_f64("KP_COM", 300.0);
    let kd_com = env_f64("KD_COM", 80.0);
    let kp_trunk = env_f64("KP_TRUNK", 200.0);
    let kd_trunk = env_f64("KD_TRUNK", 40.0);
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
    // (MOM_MERGE / ARM_MERGE, the momentum and arm tasks merged into the
    // CoM level as weighted objectives in the BDI/IHMC manner, were not
    // adopted: with the MPC's authority ARM_MERGE falls at once. Removed.)
    let kd_arm = env_f64("KD_ARM", 40.0);
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
    // the flat were unchanged. (MOM_CMP_STAIRS, lean on stairs too, is
    // removed.)
    let use_trunk = flag("TRUNK", true);
    let trunk_sign = env_f64("TRUNK_SIGN", 1.0);
    let eom_hard = flag("EOM_HARD", true);
    let cop_frac = env_f64("COP_FRAC", 1.0);
    let hold_kp = env_f64("HOLD_KP", prof.hold_kp);
    let hold_kd = env_f64("HOLD_KD", prof.hold_kd);
    let hold_bridge = env_f64("HOLD_BRIDGE", 8.0) as u32;
    let fallback_max_level = env_f64("FALLBACK_LEVEL", 999.0) as usize;
    let anchor_leak = env_f64("ANCHOR_LEAK", 0.2);
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
    // VH_DCM=1: the height-varying DCM law on a modelled terrain (see its use).
    let vh_dcm = flag("VH_DCM", false);
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
    // TURN_IN_DS: the trunk turns toward a new heading during the double
    // support, not the single support before it (see its use).
    let turn_in_ds = flag("TURN_IN_DS", true);

    // ---- disturbance: one rectangular push at a chosen gait phase --------
    // What is fired and when is `PushScenario`; the thresholds below are
    // what is measured about it.
    let push = PushScenario::from_env(prof.root_link);
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

    let ctrl_dt = env_f64("CTRL_DT", prof.ctrl_dt);
    // DIAG_VZ_BIAS_MPC / DIAG_VZ_BIAS_HEIGHT (m/s): a vertical CoM velocity
    // error seen by ONE consumer only -- the wrench MPC's initial state, or
    // the CoM height PD. Diagnostic for where SENSE_VEL_BIAS_Z acts; 0 = off.
    let diag_vz_bias_mpc = env_f64("DIAG_VZ_BIAS_MPC", 0.0);
    let diag_vz_bias_height = env_f64("DIAG_VZ_BIAS_HEIGHT", 0.0);
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
    let kp_sw_rot = env_f64("KP_SWING_ROT", 400.0);
    let kd_sw_rot = env_f64("KD_SWING_ROT", 40.0);
    // 0 = translation, 1 = +yaw, 2 = full pose, 3 = XYZ+roll+pitch (no yaw).
    let stair_absolute_td = flag("STAIR_ABSOLUTE_TD", false);
    // STAIR_ABSOLUTE_TD_DESC=1: the surveyed-tread touchdown height for
    // DESCENDING swings only. Measured 2026-10-19 with the acceleration
    // feedforward (which tracks the target to a few mm): a descent target
    // built from the lift-off height sits 6 cm above the lower tread when
    // the foot lifted off hovering, the precise swing then never touches and
    // the 60 mm round trip falls; the absolute height fixes that, but on
    // the 80 mm ascent's top landing it costs the run (foot lands 4-10 cm
    // short, the 4-step 80 mm ascent 48 -> fall). Descent-only keeps both.
    let stair_absolute_td_desc = flag("STAIR_ABSOLUTE_TD_DESC", false);
    let stair_swing_ff = flag("STAIR_SWING_FF", false);
    // Controlled ablation: retain the cubic profile and add its analytic acceleration.
    let tracking_cubic_ff = flag("TRACKING_CUBIC_FF", false);
    // TRACKING_FF_MIN_RISER [m]: apply the cubic acceleration feedforward
    // only on a staircase whose riser is at least this (0 = every stair
    // swing). Measured 2026-10-20: the feedforward takes the -150 mm descent
    // from 18 to 41/48 but, applied to every stair swing, needs the hand
    // retreat biases removed and then costs the 60/80 mm descents 3-7
    // members (round trip 192 -> 185). The gate is the RISER, not the
    // swing's height change: each foot climbs two risers per swing, so 60 mm
    // stairs already give 0.12 while the one-step deep bench gives 0.10-0.15.
    let tracking_ff_min_riser = env_f64("TRACKING_FF_MIN_RISER", 0.0).max(0.0);
    let tracking_ff_riser_ok = stairs.as_ref().is_some_and(|st| st.riser.abs() >= tracking_ff_min_riser);
    let tracking_ff_scale = env_f64("TRACKING_FF_SCALE", 1.0);
    let swing_rot_mode = env_f64("SWING_ROT", 1.0) as u32;
    // STAIR_ZONE_SWING_ROT=mode: the SWING_ROT mode used only while the
    // mid-feet point is inside a known staircase's zone (the same zone as
    // STAIR_ZONE_CROUCH, including its lead/tail). Measured 2026-10-20 on
    // kyo46rs (N=24): holding the swing foot's pitch (mode 3) is what lets
    // it take the 0.13-0.16 m stair step (8 -> 12 of 12) and the 10 mm
    // descent (1 -> 24 of 24 with the zone crouch), but applied everywhere
    // it costs the 0.9 N.s forward push (21 -> 16). 0 = off = SWING_ROT
    // everywhere, bit-identical.
    let stair_zone_swing_rot = env_f64("STAIR_ZONE_SWING_ROT", 0.0).max(0.0) as u32;
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
    // Speed-change benchmark: from step VX2_AT the forward speed ramps to
    // VX2 over VX2_RAMP steps (gait::FootstepPlan::velocity_with_change).
    // Unset = the fixed-speed plan, bit-identical.
    let vx2_change = std::env::var("VX2").ok().and_then(|v| v.parse::<f64>().ok()).map(|vx2| {
        (env_f64("VX2_AT", 8.0).max(0.0) as usize, vx2, env_f64("VX2_RAMP", 3.0).max(0.0) as usize)
    });
    // A speed the fixed step time would turn into a longer stride than the
    // VX2_SCHED cap is reached by STEPPING FASTER instead -- the footfalls
    // are planned at the capped stride and each step from VX2_AT on (its
    // single support and the double support after it) is shortened by
    // stride_max / (v * t_step). Measured 2026-10-03 on G1 v6.3 with the
    // step time fixed at 1.21 s: 0.25 m/s 47/48, 0.30 m/s 12/48, 0.35 m/s
    // 1/48 -- the stride grows to 0.36-0.42 m and the landing leg is at
    // 99.8% of its length. VX2_TSCALE_MIN bounds the shortening. (The
    // hand-set cap VX2_STRIDE_MAX is removed.)
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
    // with a 0.45 m stride. (The two hand-set knobs, VX2_STRIDE_MAX and
    // VX2_Z_DROP, are removed.) Unset = no cap and no drop.
    let vx2_sched = flag("VX2_SCHED", false);
    // (VX2_REANCHOR, re-anchoring the plan on the stance foot at every
    // single support under a speed command, is removed.)
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
            // VXS_Z_MIN: a floor on the scheduled drop once a speed command
            // is active. Measured 2026-10-16 (push x speed, N=48, v6.10):
            // at 0.30 m/s the table drops 0.024 and the pushes go 77/192;
            // +0.02 gives 122, +0.04 gives 127 (backward 11 N.s 14 -> 48).
            // At 0.50 the table already drops 0.06 and more removes lateral
            // margin (swing-side 5 N.s 11 -> 6). 0 = the table alone.
            .max(env_f64("VXS_Z_MIN", 0.0).max(0.0));
            // (VXS_Z_EXTRA, more drop on top of the schedule, is removed.)
            println!("speed schedule: v={vx2:.2} m/s -> stride {stride:.3} m (cap {cap:.2}), step {:.3} s, CoM drop {zd:.3} m", stride / vx2.max(1e-6));
            (cap, Some(zd))
        }
        _ => (0.0, None),
    };
    // PUSH_YIELD_V [m/s]: when a push is detected during a speed command
    // (DCM error above PUSH_YIELD_TRIG), re-plan the remaining steps at this
    // lower speed with the schedule's stride cap and step time, once per
    // run. Measured 2026-10-16: the pushes at 0.50 m/s are lost 2-3 steps
    // after the push through the lateral authority, and the same pushes at
    // 0.30 survive 133/192. 0 = off (bit-identical).
    let push_yield_v = env_f64("PUSH_YIELD_V", 0.0).max(0.0);
    let push_yield_trig = env_f64("PUSH_YIELD_TRIG", 0.03).max(0.0);
    let push_yield_no_fwd = flag("PUSH_YIELD_NO_FWD", false);
    let push_yield_vmin = env_f64("PUSH_YIELD_VMIN", 0.0).max(0.0);
    let push_yield_done = false;
    let vx2_cmd_speed: Option<f64> = vx2_change.map(|(_, v, _)| v);
    let vxs_cap_of = |v: f64| -> f64 {
        let cap_hi = env_f64("VXS_STRIDE_MAX", 0.45).max(0.05);
        (0.35 + env_f64("VXS_CAP_SLOPE", 2.0) * (v - env_f64("VXS_V_LONG", 0.45))).clamp(0.35, cap_hi)
    };
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
                            // (VX2_DS_SCALE, an extra factor on the double
                            // supports, defaulted to 1 and is removed.)
                            let d = gait.slices[j].duration() * scale;
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
    // Lower the walking CoM height by the schedule's drop from one step
    // before the speed change on. Measured 2026-10-05: the 0.45 m/s wall is
    // leg reach (stance leg at 99%, the heel lifts and the foot slips), and
    // 24 mm lower takes it from 7/48 to 48/48 -- but lowering the CoM for
    // the whole run trades descents and lateral pushes away. Tied to the
    // speed command it leaves every other bench bit-identical. (The
    // hand-set VX2_Z_DROP is removed.)
    let vx2_z_drop = vx2_z_drop_sched.unwrap_or(0.0);
    let vx2_z_track = articara::biped::height::SlewedDrop::default();
    let push_z_drop = env_f64("PUSH_Z_DROP", 0.0).max(0.0);
    let push_z_drop_trig = env_f64("PUSH_Z_DROP_TRIG", 0.03).max(0.0);
    let push_z_drop_t = env_f64("PUSH_Z_DROP_T", 1.5).max(0.0);
    let push_z_drop_lat_max = env_f64("PUSH_Z_DROP_LAT_MAX", 0.02).max(0.0);
    let push_z_drop_until = f64::NEG_INFINITY;
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
        let slope_keep_stride = flag("SLOPE_KEEP_STRIDE", false);
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
                // SLOPE_KEEP_STRIDE=1: on a known slope (no treads) keep the
                // velocity plan's stride and only lift each footfall to the
                // surface. The tread grid otherwise re-lays every footfall at
                // the landing stride (G1: 0.199 m vs the commanded 0.149, a
                // 35% faster climb) -- measured 2026-10-19 as the root of the
                // pushes-on-a-slope collapse (late, long, shortening steps).
                let keep_nominal = nom_x < a_pos - 1e-9 || (slope_keep_stride && st.n == 0 && st.slope != 0.0);
                let tx = if keep_nominal {
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
                    + if keep_nominal {
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
        // (COM_DESC_DS / COM_DESC_XFER, the double supports before and
        // after a descent swing stretched, are removed.)
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
                }
            }
        }
    }
    // The LIPM height is the CoM's height ABOVE THE WALKING SURFACE, which
    // on a descent platform is not its world z.
    // (STAIR_CROUCH, a fixed crouch on stairs that this height also took
    // out, was opt-in and is removed; STAIR_ZONE_CROUCH is the one in use.)
    let z_lip = (z_com - stairs.as_ref().map_or(0.0, |st| st.base())).max(0.05);
    // Heel-to-toe travel of the planned pressure inside each stance (see
    // DcmPlan::from_footsteps_rolled). On for stairs by default: the tread
    // pitch fixes the footstep pitch at ~2x what the commanded speed builds
    // momentum for, and the sole's own length is where the difference has
    // to come from. Must stay inside the CoP box (cop_half.0 * COP_FRAC).
    let zmp_roll = env_f64("ZMP_ROLL", if stairs.is_some() { 0.030 } else { 0.0 });
    // (DCM_PREVIEW_HORIZON, a finite DCM preview, and DCM_REF_BLEND_T / _MIN,
    // a continuity guard that blended a jump of the DCM reference across a
    // plan rebuild, were not adopted and are removed. Measured 2026-09-23
    // on G1: a one-tick jump of xi_ref above 5 cm separated fallers from
    // survivors at every riser -- the guard was the experiment on it.)
    // Force on the swing sole that counts toward confirming a landing.
    // Historically 5% of body weight (17 N on G1). Measured 2026-09-24 on the
    // stair swing: the foot rests on the tread for 1.4-2.8 s carrying a
    // MEDIAN of 10 N, because the swing task still holds it 4 mm above where
    // it physically sits (swing_pos_err_z = +4 mm) -- 10 N never reaches the
    // 17 N gate, and only impact chatter (spikes to 464 N) does. Default
    // reproduces the constant.
    let land_confirm_fz_frac = env_f64("LAND_CONFIRM_FZ_FRAC", 0.05).max(0.0);
    // The plan from here on: the schedule and footsteps built above, their
    // DCM reference (built now), the CoM height plan (level at z_com until
    // COM_REPLAN retargets it) and the landing clock (its knobs are set
    // below). Every later edit goes through it.
    let mut plan = Plan::new(
        gait,
        footsteps,
        DcmParams { z_lip, lat_scale: zmp_lat_scale, roll: zmp_roll },
        articara::biped::preview::HeightPlan::constant(z_com),
        articara::biped::preview::LandingTiming::new(land_confirm_fz_frac * total_mass * G),
    );
    let omega = plan.dcm.omega;
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
            plan.footsteps.travel_x(),
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
        anchor_leak_rot: 0.0,
        sole_offset_local: [prof.sole_centre_x, 0.0, -prof.sole_below_origin],
        friction_mu,
        cop_half: (sole_half_l * cop_frac, sole_half_w * cop_frac),
        mu_torsion: 0.05,
        f_max_per_foot,
        no_patch: false,
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
    // Every output of the run but stdout: TRAJ_CSV, then the opt-in
    // diagnostics (`DiagSinks`).
    let mut diag = DiagSinks::open(
        TrajLog::create(std::env::var("TRAJ_CSV").ok(), prof.log_joints.to_vec(), &extra_cols),
        std::env::var("TICK_AUDIT_CSV")
            .ok()
            .map(|path| articara::biped::tick_audit::TickAudit::create(&path, &rig)),
        substep_audit_path,
        rig.robot.joints.len(),
    );
    let policy = CommandPolicy::new(
        rig.robot.joints.len(),
        fallback_max_level,
        hold_bridge,
        false, // hold_last (HOLD_LAST, not adopted)
        0,     // blend_ticks (BLEND_TICKS, not adopted)
    );
    let mut tally = DegradedTally::new();
    let anchors = Anchors::new();
    // Solve against the contact set the FEET report, not the one the schedule
    // assumes. CONTACT_DRIVEN=0 restores the schedule-only behaviour for A/B.
    let contact_driven = flag("CONTACT_DRIVEN", true);
    let mut correction =
        ContactCorrection::new(total_mass * G, env_f64("CONTACT_TICKS", 2.0) as u32);
    let contact_on_bw = env_f64("CONTACT_ON_BW", 0.10);
    // Double-support load ramp: the fraction of the double support over
    // which the landed foot takes the load.
    let ramp_frac = env_f64("LOAD_RAMP_FRAC", 0.6).clamp(0.05, 1.0);
    assert!(
        contact_on_bw.is_finite() && contact_on_bw > 0.02 && contact_on_bw <= 1.0,
        "CONTACT_ON_BW must exceed the 0.02 BW release threshold and be <= 1"
    );
    correction.on_n = contact_on_bw * total_mass * G;
    // P2-1 contact planner (doc/from_claude/20260923_p2_contact_planner_design_v2.md).
    // Predictive replacement for ADAPT_STEP/ADAPT_TIME (both removed, never
    // adopted): picks the landing shift AND the end of the current single
    // support so the predicted end-of-step DCM meets the plan's. Default OFF.
    let contact_plan_on = flag("CONTACT_PLAN", false);
    let contact_plan_cfg = articara::biped::contact_plan::Config {
        t_min_frac: env_f64("CP_T_MIN", 0.5),
        // v0.1: never EXTEND the single support -- a long single support is
        // mechanism B itself. Measured v0 (T_max 1.5): the planner fired only
        // in already-disturbed runs and then retimed every tick (median 139-479
        // retimes per run); those runs all fell, the quiet ones survived ~95%.
        t_max_frac: env_f64("CP_T_MAX", 1.0),
        dt_grid: env_f64("CP_DT", 0.01).max(0.001),
        // Defaults were ADAPT_MAX_X / ADAPT_MAX_Y (removed with ADAPT_STEP).
        max_x: env_f64("CP_MAX_X", 0.06),
        max_y: env_f64("CP_MAX_Y", 0.03),
        max_y_in: env_f64("CP_MAX_Y_IN", env_f64("CP_MAX_Y", 0.03)),
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
    // Only plan when the DCM is actually off the plan (same reason as the
    // removed ADAPT_TIME_TRIG: the undisturbed gait must reproduce bit for
    // bit).
    let contact_plan_trig = env_f64("CP_TRIG", 0.015);
    // Terrain height under the support, slewed (below) so the CoM height
    // reference climbs a riser over about a step instead of jumping at a
    // slice boundary. Starts at the spawn platform's height.
    let terrain_track = stairs.as_ref().map_or(0.0, |st| st.base());
    let terrain_base = terrain_track;
    // Risers per step the height reference is allowed to move. Small spreads
    // one riser's climb over several steps; large finishes it within the
    // swing that takes the tread.
    let stair_climb_rate = env_f64("STAIR_CLIMB_RATE", 1.5);
    // (STAIR_DESC_MIN, riding the lower of the two soles down a descent, and
    // STAIR_CROUCH, a fixed crouch on stairs, were opt-in and are removed.)
    // Phase windows of the stair swing, `rise_end,travel_start,travel_end,
    // drop_start` (see gait::stair_swing_windows). The default is the
    // shipped profile.
    // Landing overdrive of the stair swing (metres COMMANDED below the
    // surface). See gait::stair_swing_sink.
    let stair_td_sink = env_f64("STAIR_TD_SINK", 0.004);
    let stair_land_sink = env_f64("STAIR_LAND_SINK", 0.004).max(0.0);
    // (STAIR_DESC_TOE, a toe-first descent landing, and SWING_TOE_UP / _W,
    // the toe lifted through every swing, were not adopted and are removed.)
    // SWING_XY_DELAY / SWING_Z_HOLD / SWING_Z_DROP: ROUGH_MODE sets them.
    let swing_xy_delay = env_f64("SWING_XY_DELAY", 0.0).clamp(0.0, 0.6);
    let swing_z_hold = flag("SWING_Z_HOLD", false);
    let swing_z_hold_min_dur = env_f64("SWING_Z_HOLD_MIN_DUR", 0.0).clamp(0.0, 1.0);
    let swing_z_drop: [f64; 2] = {
        let v: Vec<f64> = std::env::var("SWING_Z_DROP")
            .ok()
            .map(|s| s.split(',').filter_map(|x| x.trim().parse().ok()).collect())
            .unwrap_or_default();
        [v.first().copied().unwrap_or(0.60).clamp(0.3, 0.95), v.get(1).copied().unwrap_or(0.95).clamp(0.5, 1.0)]
    };
    // (SWING_CAUTIOUS_ERR, STAIR_ANCHOR_FLAT and STAIR_ROLL_PATCH -- the
    // latter two for the toe-first descent -- are removed.)
    let slope_heel_cut = env_f64("SLOPE_HEEL_CUT", 0.0).max(0.0);
    let slope_heel_cut_max = env_f64("SLOPE_HEEL_CUT_MAX", 0.05).clamp(0.0, 0.08);
    // (CONTACT_FZ_RATE, a load-split damper on the measured normal force,
    // was FALSIFIED -- worse than undamped at every weight -- and is
    // removed, as is DS_FZ_FLOOR_WEIGHT / _BW, a soft double-support load
    // floor that was not adopted.)
    let stair_swing_windows: [f64; 4] = {
        let s = std::env::var("STAIR_SWING_WINDOWS").unwrap_or_default();
        let v: Vec<f64> = s.split(',').filter_map(|x| x.trim().parse().ok()).collect();
        if v.len() == 4 {
            [v[0], v[1], v[2], v[3]]
        } else {
            [0.30, 0.25, 0.70, 0.60]
        }
    };
    let cp_applied = na::Vector2::zeros();
    let cp_slice: Option<usize> = None;
    let cp_retimed_slice: Option<usize> = None;
    // FIX_*: the corrective-impulse probe (see `FixProbe`).
    let fix = FixProbe::from_env();
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
    let com_replan_forward_gain = env_f64("COM_REPLAN_FORWARD_GAIN", 1.0);
    assert!(com_replan_forward_gain.is_finite() && (0.0..=1.0).contains(&com_replan_forward_gain));
    let com_replan_dt = env_f64("COM_REPLAN_DT", 0.02).max(dt);
    let com_preview_dt = env_f64("COM_PREVIEW_DT", 0.05).max(dt);
    let com_preview_n = env_f64("COM_PREVIEW_N", 20.0).max(2.0) as usize;
    let com_preview_smooth = env_f64("COM_PREVIEW_SMOOTH", 0.1).max(0.0);
    let com_height_replan = flag("COM_HEIGHT_REPLAN", true);
    let com_height_time = env_f64("COM_HEIGHT_TIME", 0.60).max(0.10);
    // The confirmation's xy / z tolerances (see LandingTiming::update).
    plan.landing.xy_tol = env_f64("LAND_CONFIRM_XY", plan.landing.xy_tol);
    plan.landing.z_tol = env_f64("LAND_CONFIRM_Z", plan.landing.z_tol);
    // (P2-2 LAND_BY_LOAD, confirming a landing on real load, was not
    // adopted and is removed.)
    // Admit a CONFIRMED landing to the contact set on the schedule instead
    // of waiting for the 10% BW load hysteresis (`ContactCorrection::admit`).
    // The landing gate already certified the sole on its tread (xy/z within
    // tolerance, fz above the gate); what it cannot certify is LOAD, because
    // load needs the QP to model the contact first. Default off = shipped.
    let contact_admit_confirmed = flag("CONTACT_ADMIT_CONFIRMED", false);
    let contact_admit_scheduled = flag("CONTACT_ADMIT_SCHEDULED", false);
    let last_scheduled_admit: [Option<usize>; 2] = [None; 2];
    // A landing that ends by the extension TIMEOUT counts too when the sole
    // is within this of its planned tread height: measured 2026-09-24, the
    // champion's stair landings mostly end that way (retime median 2.16 s =
    // 1.0 s swing + 1.08 s extension) with the sole on the tread all along.
    let contact_admit_z = env_f64("CONTACT_ADMIT_Z", 0.010).max(0.0);
    // CONTACT_ADMIT_LOADED="z_tol,fz_frac": a scheduled landing whose sole
    // is OFF its planned height by more than CONTACT_ADMIT_Z but within
    // z_tol, and which measures at least fz_frac of body weight, is admitted
    // too, anchored at its MEASURED height. Measured 2026-10-15 on the
    // unmodelled-bump bench: a foot landing on a 12 mm bump failed the 10 mm
    // gate, carried 27 N (under the 10% BW hysteresis) for the whole double
    // support while the QP balanced on the rear foot (n_stance = 1), the DCM
    // error grew, the next swing was retimed to 0.2 s and stomped at 1860 N.
    // Unset = the 10 mm gate alone (bit-identical).
    let contact_admit_loaded: Option<(f64, f64)> = std::env::var("CONTACT_ADMIT_LOADED").ok().and_then(|v| {
        let mut it = v.split(',').map(|x| x.trim().parse::<f64>().ok());
        Some((it.next()??.max(0.0), it.next()??.max(0.0)))
    });
    let admit_measured_z: [Option<f64>; 2] = [None; 2];
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
    // (LAND_WAIT_HOLD, holding the DCM reference at the stance sole while a
    // stair landing waits, was not adopted and is removed.)
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
    // (Distinct from the older global `STAIR_CROUCH` (removed), which
    // lowered the CoM from t=0 at 50 mm/s and rescaled the LIPM height;
    // measured 2026-09-24 that one fells the flat approach at t=5 on G1.)
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
    let stair_zone_crouch = env_f64("STAIR_ZONE_CROUCH", 0.0).clamp(0.0, 0.15);
    let stair_crouch_lead = env_f64("STAIR_ZONE_CROUCH_LEAD", 0.30).max(0.0);
    let stair_crouch_tail = env_f64("STAIR_ZONE_CROUCH_TAIL", 0.60).max(0.0);
    // Per side: the DOUBLE-SUPPORT slice that follows a certified landing.
    // The foot is a planned contact for that slice and is re-admitted
    // whenever the load hysteresis drops it there; cleared afterwards.
    let confirmed_admit: [Option<usize>; 2] = [None; 2];
    // Event-driven contact (doc/from_claude/20261004_event_driven_contact_design.md):
    // touchdown = measured load with the sole at its planned height (no xy
    // gate), the plan is moved onto the foot, a descending swing ends at the
    // event, a rising one keeps COM_LAND_EXTEND_RISE as an explicit dwell,
    // and a swing that has not touched by its deadline probes down instead
    // of timing out. Default off = bit-identical.
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
    let ref_lib: Option<RefLib> = None;
    let ref_any = !ref_libs.is_empty();
    let ref_swing = flag("REF_SWING", true);
    let ref_reset_stance = flag("REF_RESET_STANCE", true);
    // The re-centring after a reference skipped the swing already under way
    // when the reference ended (the gait's first step then went to the
    // pre-push plan: 0.38 m ahead, 0.28 m wide). With this on, that swing is
    // re-centred too, at the feet's midpoint (REF_RESET_LEAD, a lead ahead
    // of it, is removed).
    let ref_reset_incl_current = flag("REF_RESET_INCL_CURRENT", false);
    // Detector: lateral DCM error toward the stance side [m] that engages a
    // reference (0 = off). (The bench trigger at the push, REF_TRIG_AT_PUSH /
    // REF_TRIG_MIN_IMPULSE -- an oracle a robot does not have -- and the
    // forward-push detector REF_TRIG_DCM_X are removed.)
    let ref_trig_dcm = env_f64("REF_TRIG_DCM", 0.0).max(0.0);
    let ref_cooldown = env_f64("REF_COOLDOWN", 1.0).max(0.0);
    // 0 = enter the reference at its start even when the detector fired late
    // (for references solved with the detection delay in their initial state).
    let ref_delay_comp = flag("REF_DELAY_COMP", true);
    // (REF_MPC / REF_WBC, the MPC's and the WBC's references taken from
    // the reference library for every reference, were not adopted and are
    // removed.)
    let ref_cooldown_until = f64::NEG_INFINITY;
    let ref_lat_quiet_t = f64::NEG_INFINITY;
    // No reference when the push lands this late in the swing (the walker
    // copes on its own there; measured in-place 5 N s at phase 0.7: 3/3).
    let ref_trig_max_frac = env_f64("REF_TRIG_MAX_FRAC", 1.0);
    let ref_mode: Option<RefMode> = None;
    let event_mode_cfg = flag("CONTACT_EVENT", false);
    plan.landing.event_mode = flag("CONTACT_EVENT", false);
    plan.landing.event_z_tol = env_f64("CONTACT_EVENT_Z", 0.003).max(0.0);
    plan.landing.probe_speed = env_f64("CONTACT_PROBE_VZ", 0.05).max(0.0);
    plan.landing.probe_max = env_f64("CONTACT_PROBE_MAX", 0.03).max(0.0);
    let contact_event_shift = flag("CONTACT_EVENT_SHIFT", true);
    // 0 = a descending swing keeps its extension as a dwell too (double
    // support starts at the deadline, as the historical timeout did).
    let contact_event_end_on_touch = flag("CONTACT_EVENT_END_ON_TOUCH", true);
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
    // (DS_CAPTURE_LAT, the same test along y, was not adopted and is
    // removed.)
    // The forward excursion must exceed this [m] (noise floor).
    let ds_capture_margin = env_f64("DS_CAPTURE_MARGIN", 0.01);
    let ds_capture_t_min = env_f64("DS_CAPTURE_T_MIN", 0.15).max(0.02);
    let ds_capture_load = env_f64("DS_CAPTURE_LOAD", 0.4).clamp(0.0, 1.0);
    // (LIFT_GATE_FRAC / _MAX, holding a double support while the lateral DCM
    // is still over the lifting foot, were not adopted and are removed.)
    let dsc_retimed_last: Option<usize> = None;
    let ds_feasible_max = env_f64("DS_FEASIBLE_MAX", 1.5);
    // Pivot box for the DS sizing: "both" = the double-support box (the
    // historical choice), "stance" = the OLD stance sole only. At the cut the
    // new foot carries ~25% of the weight (measured), so a pivot under it is
    // not realisable until the load has moved; the stance-only box is what
    // the pressure can reach NOW.
    let ds_feasible_box_stance = std::env::var("DS_FEASIBLE_BOX").map(|v| v == "stance").unwrap_or(false);
    plan.landing.min_fraction = env_f64("COM_LAND_PREDICT_FRAC", 0.0).clamp(0.0, 1.0);
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
    let land_extend_rise = env_f64("COM_LAND_EXTEND_RISE", 0.0).max(0.0);
    // STEP_DZ_MIN: planned sole-height change above which a swing is a
    // stair swing (rise/descent dwell and confirmation, stair landing box,
    // landing-timing activation). 0.005 = shipped, bit-identical; a known
    // slope (TERRAIN_SLOPE, 10-15 mm per step) sets ~0.03 so its steps are
    // level steps (measured 2026-10-15: at 0.005 every step waited the
    // 1.08 s rise dwell and the walk collapsed). STEP_DZ_SWING_MIN does the
    // same for the swing PROFILE choice (stair quintic vs smoothstep).
    let step_dz_min = env_f64("STEP_DZ_MIN", 0.005).max(0.0);
    let step_dz_swing_min = env_f64("STEP_DZ_SWING_MIN", 1e-9).max(0.0);
    // TERRAIN_PITCH=1: on a known slope the swing foot's orientation target
    // is pitched to the surface at its landing (toe up on an upslope) and
    // pitch is tracked (SwingAxes::XyzRollPitch). A level sole meets a 5 deg
    // slope on its toe or heel only (measured 2026-10-15: the sole centre
    // hovered 7 mm above the surface with no load, then the stance rolled).
    // Off = shipped, bit-identical.
    let terrain_pitch = flag("TERRAIN_PITCH", false);
    let terrain_pitch_yaw = flag("TERRAIN_PITCH_YAW", false);
    plan.landing.dz_min = step_dz_min;
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
    // (LAND_CONFIRM_XY_DESC_CP, COM_LAND_EXTEND_DESC_CP and
    // LAND_CONFIRM_FZ_DESC_FRAC -- variants of the descent confirmation --
    // and LAND_CONFIRM_XY_RISE -- confirming rising swings, which ended the
    // CEM's 1.08 s ascent dwell (1 step up 192 -> 114) -- were not adopted
    // and are removed.)
    // z tolerance that goes with the wider xy one: the 10 mm default lets
    // the confirmation fire on the HEEL STRIKE of a toe-up foot (sole centre
    // still 8 mm high), 60 ms before the sole is flat (measured: 2 steps x
    // 6 cm 46 -> 33 with the wide xy alone). Unset = LAND_CONFIRM_Z.
    plan.landing.z_tol_override = std::env::var("LAND_CONFIRM_Z_DESC").ok().and_then(|v| v.parse().ok());
    let land_confirm_xy_desc: Option<f64> = std::env::var("LAND_CONFIRM_XY_DESC").ok().and_then(|v| v.parse().ok());
    plan.landing.max_extension = land_extend_base;
    plan.landing.speed_z = env_f64("COM_LAND_VZ", 0.20).max(0.01);
    plan.landing.speed_xy = env_f64("COM_LAND_VXY", 0.35).max(0.01);
    // Full per-foot wrench MPC for the three-phase stair transfer.
    // Opt-in until the fixed-arm staircase benchmark has been revalidated.
    // Command the swing to the ANKLE pose the footstep plan implies, rather
    // than to the sole pose directly. See the use site for the measurement.
    let stair_td_sole_frame = flag("STAIR_TD_SOLE_FRAME", false);
    let stair_wrench_mpc = flag("STAIR_WRENCH_MPC", false);
    // Run the wrench MPC outside a stair transfer too.
    //
    // The block below was gated on a running stair transfer (since removed),
    // so the MPC solved only while a transfer was armed. Measured
    // 2026-09-22: at 60 and 80 mm risers no transfer armed in 16 of 16 runs,
    // i.e. the best high-riser figures of that day were obtained with the
    // MPC executing ZERO ticks. A predictive layer that only runs during a
    // hand-written primitive cannot carry a momentum reference or a foothold
    // decision, so this is infrastructure for both rather than a result on
    // its own.
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
    // Align the fz schedules with `contact_schedule` (node centre).
    let mpc_node_centre = flag("MPC_NODE_CENTRE", false);
    let mpc_net_cop_weight = env_f64("MPC_NET_COP_W", 0.0).max(0.0);
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
    // Step-4 body, stage 1 (design note Sec.9): the landing recaptures the DCM.
    let mpc_landing_capture_w = env_f64("MPC_LANDING_CAPTURE_W", 0.0).max(0.0);
    // Decide the landing lexicographically (capture first, exactly, inside
    // the box; then the wrench MPC with it fixed) instead of through the
    // penalty weight above, which then only has to be > 0 to switch the
    // capture inputs on. See wrench_mpc::Config::landing_lexicographic.
    let mpc_landing_lex = flag("MPC_LANDING_LEX", false);
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
    let contact_plan_timing_only = contact_plan_on
        && contact_plan_cfg.max_x == 0.0
        && contact_plan_cfg.max_y == 0.0
        && contact_plan_cfg.max_y_in == 0.0;
    assert!(
        !(mpc_footstep && contact_plan_on) || contact_plan_timing_only,
        "MPC_FOOTSTEP and the contact_plan adjuster are two authorities over the same landing; turn one off (or make the adjuster timing-only: CP_MAX_X=CP_MAX_Y=CP_MAX_Y_IN=0)"
    );
    let mpc_footstep_w = env_f64("MPC_FOOTSTEP_W", 100.0).max(0.0);
    // Give the capture rows the plan's own sensitivity to the landing
    // (`Input::reference_dcm_sensitivity`), differenced from the real DCM
    // plan the way CP_J_AUTO does: shift the landing by 2 cm per axis,
    // rebuild the rolled plan, read the reference DCM at every node. Off =
    // the shipped rows, bit-identical.
    let mpc_landing_capture_j = flag("MPC_LANDING_CAPTURE_J", false);
    // Largest lateral shift TOWARD the stance foot (a narrower step). The
    // contact_plan adjuster forbids it in v4.1 (CP_MAX_Y_IN=0); the MPC box
    // allowed it. Measured 2026-09-27 (D+J+T, 11 N.s backward, m32): the
    // landing moved 27 mm inward, the lateral DCM error then ran away
    // -19 -> -126 mm and the robot fell. Default = MPC_FOOTSTEP_MAX_Y
    // (symmetric box), bit-identical.
    let mpc_footstep_max_y_in = env_f64("MPC_FOOTSTEP_MAX_Y_IN", f64::INFINITY).max(0.0);
    // Inward bound while the SAGITTAL DCM error is forward by at least
    // MPC_FOOTSTEP_Y_IN_FWD_TRIG [m] (world x; the walk heads +x). Measured
    // 2026-09-27: the inward shifts that save 11 N.s forward pushes happen
    // at x error +10..+17 mm; the harmful ones after backward pushes at
    // x error -16..-34 mm (push step) or ~0 with a large lateral error (the
    // next step). The retimed-step rule above does not separate them (a
    // backward push shortens steps too). 0 = off = shipped.
    let mpc_footstep_y_in_fwd = env_f64("MPC_FOOTSTEP_Y_IN_FWD", 0.0).max(0.0);
    // MPC_FOOTSTEP_CROSS_Y [m]: let the MPC landing CROSS the stance foot
    // when the DCM has run out past the stance foot on the stance side. In
    // single support, once the measured DCM is more than
    // MPC_FOOTSTEP_CROSS_EX [m] beyond the stance sole centre AWAY from the
    // swing foot, the inward bound and the inward side of the y box become
    // this value for the rest of that single support (latched). The capture
    // cost then picks the crossing landing. Measured
    // 2026-10-20 on G1 v6.11: 21 of 24 lateral-push falls are late-phase
    // (0.58-0.75) pushes toward the stance foot, where the landing stays put
    // with the DCM 9-11 cm past the new stance sole; the inward ban
    // (MPC_FOOTSTEP_MAX_Y_IN=0, set after a backward-push runaway on 09-27)
    // and the 0.08 m box leave the MPC no crossing step (doc
    // 20261014_push_x_speed.md Sec.16). The gate keeps the ban for every
    // other case. 0 = off = bit-identical.
    let mpc_footstep_cross_y = env_f64("MPC_FOOTSTEP_CROSS_Y", 0.0).max(0.0);
    let mpc_footstep_cross_ex = env_f64("MPC_FOOTSTEP_CROSS_EX", 0.02);
    // MPC_FOOTSTEP_CROSS_FLOOR [fraction of the sole half width]: while
    // crossing, a lower bound on the inward shift from the LIPM: the DCM at
    // the end of this single support, with the stance CoP held at the
    // stance sole's outer edge (the best the ankle can do), must lie within
    // this fraction of the half width of the landing sole. (Its x twin,
    // MPC_FOOTSTEP_CAPTURE_FLOOR, is removed.) Measured 2026-10-20: with
    // only the bound opened the MPC lands at y +0.02..+0.04 for a DCM at
    // +0.09 (stance sole +0.05) -- the SRBD model has no attitude and
    // believes it can hold the excursion. 0 = off.
    let mpc_footstep_cross_floor = env_f64("MPC_FOOTSTEP_CROSS_FLOOR", 0.0).max(0.0);
    // MPC_FOOTSTEP_CROSS_HOLD_Z [m]: while crossing, hold the swing foot at
    // least this high above its landing until it is within
    // MPC_FOOTSTEP_CROSS_HOLD_XY [m] of the landing horizontally. Measured
    // 2026-10-20: with the crossing landing fixed the foot still came down on
    // the nominal clock 5 cm short of it (the lateral travel lags), touched
    // at y +0.01 and slid. The landing check then extends the single support
    // (LandingTiming) instead of the foot landing short. 0 = off.
    let mpc_footstep_cross_hold_z = env_f64("MPC_FOOTSTEP_CROSS_HOLD_Z", 0.0).max(0.0);
    let mpc_footstep_cross_hold_xy = env_f64("MPC_FOOTSTEP_CROSS_HOLD_XY", 0.02).max(0.0);
    // PUSH_REPLAN=trig [m]: ONE re-planner for the step after a push
    // (doc 20261014_push_x_speed.md Sec.20.3). In single support, the DCM
    // error against the plan that was in force when the support began,
    // carried to touchdown (x e^{omega rem}), is the landing shift the
    // capture step needs; once it exceeds `trig` the re-planner owns this
    // landing: original landing + that shift, clamped to the leg's reach
    // (PUSH_REPLAN_MAX_FWD / _MAX_BACK, a lateral gap of at least
    // PUSH_REPLAN_MIN_W; its own crossing, PUSH_REPLAN_CROSS / _AHEAD, was
    // removed), and the rest of the
    // plan is re-laid from it (`FootstepPlan::replan_from_landing`:
    // PUSH_REPLAN_STAND steps in place, then PUSH_REPLAN_RAMP steps back to
    // the original stride). The MPC landing variable is off for that swing.
    // Re-evaluated every tick until PUSH_REPLAN_FREEZE of the swing. 0 = off.
    let push_replan_trig = env_f64("PUSH_REPLAN", 0.0).max(0.0);
    let push_replan_stand = env_f64("PUSH_REPLAN_STAND", 2.0).max(0.0) as usize;
    let push_replan_ramp = env_f64("PUSH_REPLAN_RAMP", 2.0).max(0.0) as usize;
    let push_replan_freeze = env_f64("PUSH_REPLAN_FREEZE", 0.85);
    let push_replan_max_fwd = env_f64("PUSH_REPLAN_MAX_FWD", 0.35).max(0.0);
    let push_replan_max_back = env_f64("PUSH_REPLAN_MAX_BACK", 0.30).max(0.0);
    let push_replan_min_w = env_f64("PUSH_REPLAN_MIN_W", 0.12).max(0.0);
    let push_replan_hold = env_f64("PUSH_REPLAN_HOLD", 0.06).max(0.0);
    // PUSH_REPLAN_BACK_ONLY=1: engage only on a backward-dominant error
    // (behind, and more than sideways). PUSH_REPLAN_VMAX [m/s]: never while a
    // VX2 speed command above this is in force (the speed yield re-plans
    // there). Measured 2026-10-21 (tu42, all directions): backward 14 N.s
    // 9 -> 42, but forward / lateral 3-5 members lost each and 0.50 m/s
    // backward 44 -> 21.
    let push_replan_back_only = flag("PUSH_REPLAN_BACK_ONLY", false);
    let push_replan_vmax = env_f64("PUSH_REPLAN_VMAX", f64::INFINITY).max(0.0);
    // The current DCM error must also exceed this [m] (a 2 mm error at the
    // start of a long first support carried 0.8 s forward read as 4 cm).
    let push_replan_emin = env_f64("PUSH_REPLAN_EMIN", 0.015).max(0.0);
    // Widest outward step [m] from the stance foot.
    let push_replan_max_w = env_f64("PUSH_REPLAN_MAX_W", 0.40).max(0.0);
    // PUSH_REPLAN_ANKLE="ax,ay": fraction of the sole's half length / width
    // the ankle is credited with holding before the error is carried forward.
    // Measured 2026-10-21: crediting the full 85 mm half length took the
    // backward 14 N.s fallers from 8/12 to 0/12 (a heel CoP does not hold a
    // backward push); laterally it keeps a small error from reading as a 1 m
    // step. Default 0 / 1.
    let push_replan_ankle: [f64; 2] = {
        let v: Vec<f64> = std::env::var("PUSH_REPLAN_ANKLE")
            .ok()
            .map(|s| s.split(',').filter_map(|x| x.trim().parse().ok()).collect())
            .unwrap_or_default();
        [v.first().copied().unwrap_or(0.0).clamp(0.0, 1.0), v.get(1).copied().unwrap_or(1.0).clamp(0.0, 1.0)]
    };
    // (slice, original footsteps, original DCM plan, original landing, last applied landing)
    let push_replan_state: Option<(usize, FootstepPlan, DcmPlan, na::Vector2<f64>, Option<na::Vector2<f64>>)> = None;
    let push_replan_slice: Option<usize> = None;
    let push_replan_shift = 0.0_f64;
    // (slice, inward is +y) latched when the crossing gate opens: once the
    // landing has crossed, `landing_inward` would read the sign the other way.
    let mpc_fs_cross_latched: Option<(usize, bool)> = None;
    let mpc_footstep_y_in_fwd_trig = env_f64("MPC_FOOTSTEP_Y_IN_FWD_TRIG", 0.005);
    // Arm the landing variable only once the DCM error of this single
    // support reaches this value [m] (latched for the rest of the step), the
    // rule the contact_plan adjuster uses (CP_TRIG): an undisturbed step
    // then solves exactly the fixed-contact problem. Measured 2026-09-27:
    // with the variable always present, D+J loses 6-8 cm wide-holdout
    // stairs on two independent seeds (pooled 1 better / 10 worse) while
    // the variable itself stays near 0 where the runs part -- the cost is
    // its presence, not its choice. 0 = always armed = shipped.
    let mpc_footstep_trig = env_f64("MPC_FOOTSTEP_TRIG", 0.0).max(0.0);
    // (MPC_FOOTSTEP_TRIG_HOLD, arming the following steps too, and
    // VXS_BOX_GAIN, a box grown with the speed command, are removed.)
    let mpc_footstep_max = [
        env_f64("MPC_FOOTSTEP_MAX_X", 0.06).max(0.0),
        env_f64("MPC_FOOTSTEP_MAX_Y", 0.03).max(0.0),
    ];
    let mpc_w_pos = env_f64("MPC_W_POS", 20.0).max(0.0);
    let mpc_w_vel = env_f64("MPC_W_VEL", 4.0).max(0.0);
    let mpc_w_mom = env_f64("MPC_W_MOM", 2.0).max(0.0);
    let mpc_w_acc = env_f64("MPC_W_ACC", 6.0).max(0.0);
    let mpc_w_wrench = env_f64("MPC_W_WRENCH", 2e-4).max(0.0);
    let mpc_w_smooth = env_f64("MPC_W_SMOOTH", 1e-3).max(0.0);
    let mpc_sole_authority = flag("MPC_SOLE_AUTHORITY", false);
    let stair_wrench_dt = env_f64("STAIR_WRENCH_DT", 0.02).max(dt);
    let stair_wrench_horizon = env_f64("STAIR_WRENCH_N", 8.0).max(2.0) as usize;
    let stair_wrench_preview_dt = env_f64("STAIR_WRENCH_PREVIEW_DT", 0.05).max(dt);
    let stair_wrench_low_min = env_f64("STAIR_WRENCH_LOW_MIN", 0.12).clamp(0.0, 0.45);
    let stair_wrench_blend = env_f64("STAIR_WRENCH_BLEND", 0.25).clamp(0.0, 1.0);
    let stair_wrench_coupled = flag("STAIR_WRENCH_COUPLED", true);
    let stair_managed_contact = flag("STAIR_MANAGED_CONTACT", true);
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
    // Ramp a NEW contact in over this many seconds, on any ground, with no
    // stair transfer in sight. 0 = off = shipped behaviour.
    //
    // Apart from the rate trigger (`STAIR_MANAGED_RATE`), Managed contact
    // lived inside the stair transfer (removed), so on flat ground every
    // contact is admitted Rigid in a single tick.
    // Measured on the G1 bring-up (doc/from_claude/20260919_g1_transfer_probe.md):
    // at the 1 -> 2 contact-set change the QP's total vertical force collapsed
    // 1.03 -> 0.45 BW for one tick and took 50 ms to climb back, both feet flat
    // on the floor the whole time. The CoM drop that opened made the CoM task
    // demand +15 m/s^2 -- 1.5 g upward -- and the QP paid for it with a 1.81 BW
    // slam that launched the swing leg to 0.82 m. kyo46rs never dips below
    // 0.85 BW walking on the flat, which is why this went unseen there.
    // (TOUCHDOWN_RAMP, ramping a new contact in as a Managed one, and
    // SWING_CONTACT_SOFT / _ON / _CAP, admitting a loaded swing foot as a
    // weighted contact, were not adopted and are removed.)
    let replan_engaged = false;
    let preview_p = steps.mid_xy();
    // The CoM preview QP's horizontal trajectory, kept whole with the time it
    // was solved at, so the MPC can read its own horizon out of it.
    let preview_c = com0.xy();
    let preview_v = na::Vector2::zeros();
    let preview_next = 0.0;
    // (PHASE_BY_CONTACT / PHASE_EXTEND / PHASE_MIN_FRAC / PHASE_LOAD /
    // PHASE_TICKS, the phase clock advanced by measured contact, are removed;
    // its summary line keeps the zeros it printed when off.)
    // The control loop's debug prints (`DIAG_REPLAN`, `*_DEBUG`, ...), read
    // once here.
    let debug = DebugFlags::from_env();
    let push_replan_slope_lead = env_f64("PUSH_REPLAN_SLOPE_LEAD", 0.0).max(0.0);

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

    let prev_support = plan.gait.support_at(0.0);
    let swing_lift_off: Option<na::Vector3<f64>> = None;
    let p0_body = rig.sim.body_world_position(&rig.robot.root_link).unwrap();
    let yaw0_body = rig
        .sim
        .body_world_orientation(&rig.robot.root_link)
        .unwrap()
        .euler_angles()
        .2;
    // Time at which the last step ends, so achievement is measured over the
    // WALK and not diluted by the settle.
    let t_walk_end = plan.gait
        .slices
        .iter()
        .rposition(|s| matches!(s.support, Support::Single { .. }))
        .map(|i| plan.gait.slices[i].t1 + gp.t_ds)
        // No steps at all (a standing squat): the "walk" is the whole run, so
        // the achievement window is the whole run. Falling back to 0 made the
        // window 1 microsecond wide and turned a millimetre of drift into
        // 56 m/s.
        .unwrap_or(total_t);
    // Achievement is measured from the end of the stride ramp, not from
    // standing. Including the ramp charges every case a fixed shortfall that
    // never recovers -- it read 91% for every forward speed, and 47% for a
    // turn that was in fact holding 0.195 of a commanded 0.200 rad/s.
    let t_ramp_end = gp.t_start + (stride_ramp as f64) * t_step;
    let fz_prev: [f64; 2] = [0.0; 2];
    let fz_rate_ema: [f64; 2] = [0.0; 2];
    let managed_until: [f64; 2] = [f64::NEG_INFINITY; 2];
    // What the run measured about itself (achievement, extremes, the
    // push response), for the report after the loop.
    let mut m = RunMetrics::new(rig.robot.joints.len(), p0_body, yaw0_body);
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
    let previous_a_xy: Option<na::Vector2<f64>> = None;
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
    // The same quantity for the forces the WBC actually asked the robot for.
    let mut diag_wbc_net_cop = [f64::NAN; 2];
    let wrench_mpc_next = 0.0;
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
    // KKT complementarity tolerance of the MPC's solution check (see its use).
    let mpc_kkt_comp_tol = env_f64("MPC_KKT_COMP_TOL", 1e-6);
    let act_delay_ticks = env_f64("ACT_DELAY_TICKS", 0.0).max(0.0) as usize;
    let mut act_delay_buf: std::collections::VecDeque<Vec<f64>> = std::collections::VecDeque::new();
    let wrench_mpc_command: Option<[articara::biped::wrench_mpc::Wrench; 2]> = None;
    // Preserve the physical origin/axes of the held MPC forecast for diagnostic shifts.
    let mut wrench_mpc_frames: Option<[articara::biped::wrench_mpc::Contact; 2]> = None;
    // MPC_FOOTSTEP bookkeeping: which slice's landing is being moved and how
    // much of the MPC's offset has already been applied to it (the same
    // non-ratcheting rule as `cp_applied`).
    let mpc_fs_slice: Option<usize> = None;
    let mpc_fs_applied = na::Vector2::zeros();
    // (first slice, last slice) the landing variable is armed for.
    let mpc_fs_armed: Option<(usize, usize)> = None;
    let mpc_fs_last = na::Vector2::zeros();
    let mut trunk_cmd_rp = [0.0_f64; 2];
    let mut trunk_acc_rp = [0.0_f64; 2];

    // The controller's state from here on (its initial values are the
    // `let`s above, next to the knobs that configure them), the WBC's
    // own, the command guard's, and the controller's counters.
    let mut ctl = CtlState {
        push_yield_done,
        vx2_z_track,
        push_z_drop_until,
        correction,
        terrain_track,
        cp_applied,
        cp_slice,
        cp_retimed_slice,
        last_scheduled_admit,
        admit_measured_z,
        confirmed_admit,
        ref_lib,
        ref_cooldown_until,
        ref_lat_quiet_t,
        ref_mode,
        dsc_retimed_last,
        push_replan_state,
        push_replan_slice,
        push_replan_shift,
        mpc_fs_cross_latched,
        replan_engaged,
        preview_p,
        preview_c,
        preview_v,
        preview_next,
        prev_support,
        swing_lift_off,
        fz_prev,
        fz_rate_ema,
        managed_until,
        previous_a_xy,
        wrench_mpc_next,
        wrench_mpc_command,
        mpc_fs_slice,
        mpc_fs_applied,
        mpc_fs_armed,
        mpc_fs_last,
    };
    let mut wbc_state = WbcState {
        solver,
        qp_previous_structure,
        anchors,
    };
    let mut guard = CommandGuard {
        policy,
        qp_has_good_command,
    };
    let mut stats = WalkerStats::default();

    for tick in 0..n_ticks {
        let qp_control_start = std::time::Instant::now();
        let mut mpc_capture_ns = 0u128;
        let t = tick as f64 * dt;
        m.t_last = t;
        if land_extend_rise > 0.0 {
            // Same test the landing detector itself uses to decide a swing is
            // a stair swing (`preview.rs`: the planned sole height changes by
            // more than 5 mm), so the slack applies to exactly the swings that
            // need it and to no others.
            let i = plan.gait.index_at(t);
            let dz_swing = plan.gait.slices[i].support.swing().map_or(0.0, |side| {
                plan.footsteps.at_slice(i).sole[side].z
                    - plan.footsteps.at_slice(i.saturating_sub(1)).sole[side].z
            });
            let rising = dz_swing.abs() > step_dz_min;
            let descending = dz_swing < -step_dz_min;
            plan.landing.event_end_on_touch = descending && contact_event_end_on_touch;
            plan.landing.xy_tol_override = if descending { land_confirm_xy_desc } else { None };
            // The contact planner owns the end of a step it has retimed; the
            // landing extension must not re-open it (assigned every tick, so
            // the planner's own zeroing would be undone here otherwise).
            plan.landing.max_extension = if contact_plan_on && ctl.cp_retimed_slice == Some(i) {
                0.0
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
        let in_push_window = m.push_at_t.map(|tp| t - tp <= push_window_s).unwrap_or(false);
        if in_push_window {
            if m.win_degraded_at.is_none() {
                m.win_degraded_at = Some(tally.n);
            }
            m.win_degraded_end = tally.n;
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
        let com = st.com;
        let com_vel = st.com_vel;

        if com_replan {
            let actual = Footsteps::from_fk(
                data,
                rig.foot_mi,
                prof.sole_centre_x,
                prof.sole_below_origin,
            );
            let touchdowns_before = plan.landing.touchdowns;
            let timeouts_before = plan.landing.timeouts;
            let shift = plan.landing.update(&mut plan.gait, &plan.footsteps, &actual, measured_pre, t, dt);
            let landing_timed_out = plan.landing.timeouts > timeouts_before;
            // The slice that just ended is the timer's clock slice, not
            // `gait.index_at(t)`: on a timeout tick the index already points
            // at the double support, whose swing side is None.
            if contact_admit_confirmed
                && (plan.landing.touchdowns > touchdowns_before || landing_timed_out)
                && let Some((ended, _, _)) = plan.landing.clock
                && let Some(sw) = plan.gait.slices[ended].support.swing()
                && (plan.footsteps.at_slice(ended).sole[sw].z - actual.sole[sw].z).abs() < contact_admit_z
            {
                ctl.confirmed_admit[sw] = Some(ended + 1);
                if debug.contact_admit {
                    println!("  [adm] t={t:.3} certificate side={sw} ds_slice={} (confirm={} timeout={})", ended + 1, plan.landing.touchdowns > touchdowns_before, landing_timed_out);
                }
            }
            if let Some((li, lsw, err_xy)) = plan.landing.last_event.take()
                && contact_event_shift
                && err_xy.norm() > 1e-4
            {
                // plan - actual = err  ->  move the plan by -err onto the foot.
                plan.footsteps.shift_next(li, lsw, -err_xy);
                plan.rebuild_dcm();
            }
            if shift != 0.0 || (ds_feasible_on_timeout && landing_timed_out) {
                if shift != 0.0 {
                    stats.landing_retimes += 1;
                    stats.landing_delay += -shift;
                }
                plan.rebuild_dcm();
                ctl.preview_next = t;
                if ds_feasible {
                    let ds = plan.gait.index_at(t + 1e-6);
                    if matches!(plan.gait.slices[ds].support, Support::Double) {
                        let xi_meas = dcm_of(&com, &com_vel, omega);
                        let seg = plan.dcm.index_at(t + 1e-6);
                        let target = plan.dcm.xi_eos[seg];
                        let ds_box_full = SupportBox::from_stance_yawed(
                            &actual,
                            Support::Double,
                            (sole_half_l * cop_frac, sole_half_w * cop_frac),
                            zmp_margin,
                        );
                        // The support that just ended tells which foot carries the load.
                        let prev_sw = ds
                            .checked_sub(1)
                            .and_then(|j| plan.gait.slices[j].support.swing());
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
                        let nominal = plan.gait.slices[ds].duration();
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
                                let dd = plan.gait.retime(ds, plan.gait.slices[ds].t0 + want);
                                stats.n_ds_feasible += 1;
                                stats.ds_feasible_total += dd.abs();
                                plan.rebuild_dcm();
                            }
                            let seg_ds = plan.dcm.index_at(plan.gait.slices[ds].t0 + 1e-6);
                            let p_final = if ds_feasible_ramp && seg_ds + 1 < plan.dcm.segs.len() {
                                let tt = plan.gait.slices[ds].duration();
                                let wt = omega * tt;
                                let d = (-wt).exp();
                                let b = (1.0 - d) / wt - d;
                                let a = 1.0 - (1.0 - d) / wt;
                                let p1 = plan.dcm.segs[seg_ds + 1].p0;
                                let p0 = ds_box.clamp(&((xi_meas - target * d - p1 * b) / a)).0;
                                plan.dcm.set_segment_zmp_ramp(seg_ds, p0, p1);
                                p0
                            } else {
                                let p = p_continuous(plan.gait.slices[ds].duration());
                                plan.dcm.set_segment_zmp(seg_ds, p);
                                p
                            };
                            stats.n_ds_continuous += 1;
                            if debug.ds_feasible {
                                let r = plan.dcm.reference(plan.gait.slices[ds].t0 + 1e-6);
                                println!("  [dsc] t={t:.3} ds={ds} xi=({:.3},{:+.3}) target=({:.3},{:+.3}) p=({:.3},{:+.3}) T={:.3} xi_ref(t0)=({:.3},{:+.3})", xi_meas.x, xi_meas.y, target.x, target.y, p_final.x, p_final.y, plan.gait.slices[ds].duration(), r.xi.x, r.xi.y);
                            }
                        } else if want > nominal + 1e-3 {
                            let dd = plan.gait.retime(ds, plan.gait.slices[ds].t0 + want);
                            stats.n_ds_feasible += 1;
                            stats.ds_feasible_total += dd.abs();
                            plan.rebuild_dcm();
                        }
                        if debug.ds_feasible {
                            println!("  [dsf] t={t:.3} ds={ds} xi=({:.3},{:+.3}) target=({:.3},{:+.3}) p=({:.3},{:+.3}) need={need:.3} nominal={nominal:.3} -> {want:.3}", xi_meas.x, xi_meas.y, target.x, target.y, p.x, p.y);
                        }
                    }
                }
            }
            if ds_capture {
                let ds = plan.gait.index_at(t + 1e-6);
                let is_ds = |j: usize| matches!(plan.gait.slices[j].support, Support::Double);
                // Not while a recovery reference owns the phase durations.
                if is_ds(ds) && ctl.ref_mode.is_none() && !ctl.dsc_retimed_last.is_some_and(|l| l >= ds) {
                    // The whole run of double-support slices the clock is in
                    // (a DS is split into ramp / hold / ramp slices).
                    let mut first = ds;
                    while first > 0 && is_ds(first - 1) {
                        first -= 1;
                    }
                    let mut last = ds;
                    while last + 1 < plan.gait.slices.len() && is_ds(last + 1) {
                        last += 1;
                    }
                    let t0_run = plan.gait.slices[first].t0;
                    let t1_run = plan.gait.slices[last].t1;
                    let remaining = t1_run - t;
                    let xi_meas = dcm_of(&com, &com_vel, omega);
                    let seg_last = plan.dcm.index_at(t1_run - 1e-6);
                    let target = plan.dcm.xi_eos[seg_last];
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
                    let hopeless_x = remaining > 1e-3
                        && need > remaining * ds_capture_hopeless
                        && (!ds_capture_fwd_only || xi_meas.x > target.x + ds_capture_margin);
                    let hopeless = hopeless_x;
                    let prev_sw = first.checked_sub(1).and_then(|j| plan.gait.slices[j].support.swing());
                    let landed_load = prev_sw.map_or(0.0, |sw| ctl.fz_prev[sw]);
                    let earliest = (t0_run + ds_capture_t_min).max(t + 2.0 * dt);
                    if debug.ds_capture {
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
                            (ds..=last).map(|j| (j, t + (plan.gait.slices[j].t1 - t) * f)).collect();
                        for (j, nt) in targets {
                            stats.ds_capture_total += plan.gait.retime(j, nt).abs();
                        }
                        ctl.dsc_retimed_last = Some(last);
                        stats.n_ds_capture += 1;
                        plan.rebuild_dcm();
                        if debug.ds_capture {
                            println!("  [dsc-cap] t={t:.3} DS end {t1_run:.3} -> {earliest:.3}");
                        }
                    }
                }
            }
            // (COM_REPLAN_ALWAYS, engaging before the first landing clock, is
            // removed.)
            if !ctl.replan_engaged && plan.landing.clock.is_some() {
                let cmd_ref = plan.dcm.reference(t);
                ctl.preview_p = commanded_zmp(
                    &dcm_of(&com, &com_vel, omega),
                    &cmd_ref,
                    omega,
                    k_dcm,
                );
                plan.height = articara::biped::preview::HeightPlan::constant(
                    z_com + ctl.terrain_track - terrain_base,
                );
                ctl.replan_engaged = true;
            }
        }

        // ---- schedule -> contact set -----------------------------------
        // By value, not by reference: `Slice` is `Copy`, and holding a
        // borrow on `gait` here is what stops the step-timing adaptation
        // below from retiming it.
        let (slice, slice_frac) = {
            let (sl, f) = plan.gait.at(t);
            (*sl, f)
        };
        let support = if no_lift {
            Support::Double
        } else {
            slice.support
        };
        let step_idx = plan.gait.steps_taken(t.max(1e-9));
        let slice_idx = plan.gait.index_at(t);
        // Everything below reads the footsteps for THIS slice. With stride 0
        // they are the same pair every time and this is the old behaviour.
        let steps = *plan.footsteps.at_slice(slice_idx);
        let swing_rot_mode = if stair_zone_swing_rot > 0
            && stairs.as_ref().is_some_and(|st| {
                articara::biped::height::stair_zone_crouch(steps.mid_xy().x, st, stair_crouch_lead, stair_crouch_tail, 1.0) > 0.0
            }) {
            stair_zone_swing_rot
        } else {
            swing_rot_mode
        };

        // Touchdown and lift-off are the only two events that may edit an
        // anchor. Doing it on the schedule (rather than lazily, the way
        // standing did with get_or_insert) is what makes a foot that has been
        // somewhere else since stop arguing for where it used to be.
        if support != ctl.prev_support {
            match (ctl.prev_support, support) {
                (Support::Double, Support::Single { swing, .. }) => {
                    wbc_state.anchors.release(swing);
                    let p = misarta::se3::translation(&data.oMi[rig.foot_mi[swing]]);
                    ctl.swing_lift_off = Some(p);
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
                    // (STAIR_ANCHOR_FLAT, the plan rotation, is removed.)
                    let r = misarta::se3::rotation_matrix(&data.oMi[mi]);
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
                    if debug.anchor {
                        eprintln!(
                            "  [anchor] t={t:.3} swing {} touchdown: meas z {:.4} -> plan z {:.4}",
                            if swing == 0 { "L" } else { "R" },
                            p_meas_z,
                            p.z
                        );
                    }
                    wbc_state.anchors.touchdown(swing, p, r);
                    ctl.swing_lift_off = None;
                }
                _ => {}
            }
            ctl.prev_support = support;
        }

        let fz_meas = [measured_pre[0], measured_pre[1]];
        if stair_managed_rate > 0.0 {
            for side in 0..2 {
                // EMA over ~4 ticks: one noisy sample must not trip it.
                let r = (fz_meas[side] - ctl.fz_prev[side]).abs() / dt.max(1e-9);
                ctl.fz_rate_ema[side] += 0.25 * (r - ctl.fz_rate_ema[side]);
                if ctl.fz_rate_ema[side] > stair_managed_rate {
                    ctl.managed_until[side] = t + stair_managed_latch;
                }
            }
        }
        ctl.fz_prev = fz_meas;
        let measured = measured_c;
        let stance_sides: Vec<usize> = if contact_driven {
            let (mut sides, mut fresh) = ctl.correction.update(support, fz_meas);
            // Planned admission on ANY scheduled touchdown, flat included:
            // entering a double support, a foot the schedule now calls
            // stance whose sole is within CONTACT_ADMIT_Z of its planned
            // height is certified for that double support. Measured
            // 2026-09-24 under the weighted QP: a gentle flat touchdown
            // carried 5 N, stayed under the 10% BW hysteresis for the whole
            // double support, and the QP balanced on the rear foot alone.
            // (CONTACT_ADMIT_SCHEDULED_OFF_STAIRS, only outside the stair
            // zone, is removed.)
            if contact_admit_scheduled
                && matches!(support, Support::Double)
            {
                let now_slice = plan.gait.index_at(t);
                if let Some(prev) = now_slice.checked_sub(1)
                    && let Some(landed) = plan.gait.slices[prev].support.swing()
                    && ctl.confirmed_admit[landed].is_none()
                    && !sides.contains(&landed)
                    && ctl.last_scheduled_admit[landed] != Some(now_slice)
                {
                    let mi = rig.foot_mi[landed];
                    let sole_z = misarta::se3::translation(&data.oMi[mi]).z - prof.sole_below_origin;
                    let dz_plan = (sole_z - steps.sole[landed].z).abs();
                    if dz_plan < contact_admit_z {
                        ctl.confirmed_admit[landed] = Some(now_slice);
                        ctl.last_scheduled_admit[landed] = Some(now_slice);
                        ctl.admit_measured_z[landed] = None;
                    } else if let Some((z_tol, fz_frac)) = contact_admit_loaded
                        && dz_plan < z_tol
                        && fz_meas[landed] >= fz_frac * total_mass * G
                    {
                        ctl.confirmed_admit[landed] = Some(now_slice);
                        ctl.last_scheduled_admit[landed] = Some(now_slice);
                        ctl.admit_measured_z[landed] = Some(sole_z);
                        if debug.contact_admit {
                            println!("  [adm-loaded] t={t:.3} side={landed} sole_z={sole_z:.4} plan_z={:.4} fz={:.0}", steps.sole[landed].z, fz_meas[landed]);
                        }
                    }
                }
            }
            if contact_admit_confirmed || contact_admit_scheduled {
                let now_slice = plan.gait.index_at(t);
                for side in 0..2 {
                    let Some(ds_slice) = ctl.confirmed_admit[side] else { continue };
                    if now_slice > ds_slice {
                        // The double support after the landing is over.
                        ctl.confirmed_admit[side] = None;
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
                        && ctl.correction.admit(side)
                    {
                        sides.push(side);
                        sides.sort_unstable();
                        fresh.push(side);
                        stats.n_confirmed_admissions += 1;
                        if debug.contact_admit {
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
                // A landing admitted on an unmodelled surface (see
                // CONTACT_ADMIT_LOADED) is anchored where it actually is.
                p.z = ctl.admit_measured_z[side].unwrap_or(steps.sole[side].z) + prof.sole_below_origin;
                let rr = misarta::se3::rotation_matrix(&data.oMi[mi]);
                wbc_state.anchors.touchdown(side, p, rr);
            }
            let sched: Vec<usize> = (0..2).filter(|&s| support.is_stance(s)).collect();
            if sides != sched {
                stats.n_corrected += 1;
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
            let i = plan.gait.index_at(t);
            let landed = if i > 0 {
                plan.gait.slices[i - 1].support.swing()
            } else {
                None
            };
            let next = plan.gait.slices.get(i + 1).and_then(|s| s.support.swing());
            (landed, next)
        } else {
            (None, None)
        };
        // (LOAD_RAMP_TD, keeping the share the landed foot already carries,
        // and LOAD_RAMP_CONTACT_T, ramping from the contact-entry tick too,
        // were not adopted and are removed.)
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
        let p0 = bt::contact_level(
            &dyn_ctx,
            base,
            &j_contact,
            &dj_v,
            data,
            v_dvec,
            &stance,
            rig.foot_mi[0],
            &mut wbc_state.anchors,
            &load_share,
            &contact_cfg,
            &|side: usize| {
                if stair_wrench_mpc
                    && stair_managed_contact
                    // Rate trigger (see STAIR_MANAGED_RATE): the level
                    // test misses the chatter, which swings fz 15<->78 N.
                    && t < ctl.managed_until[side]
                {
                    bt::ContactMode::Managed {
                        tangent_weight: stair_managed_tangent,
                        rotation_weight: stair_managed_rotation,
                        normal_weight: 1.0,
                        normal_accel_max: stair_managed_az_max,
                    }
                } else {
                    bt::ContactMode::Rigid
                }
            },
            &|_side: usize, _rot: &na::Matrix3<f64>| {
                // SLOPE_HEEL_CUT [m/deg]: on a known UPSLOPE, during double
                // support, move the REAR foot's CoP box forward by
                // cut * slope_deg and shrink it by the same, so its heel
                // region is excluded. Measured 2026-10-17 (threshold table,
                // terrain bench doc Sec.10): the rear foot's CoP goes to the
                // heel in proportion to the slope (-12 mm/deg), reaches the
                // heel edge at 5 deg and the foot lifts off 3/4 of the way
                // through the double support. With the heel excluded the QP
                // has to realise the same net CoP by loading the front
                // foot, which is what the climb needs. 0 = off.
                if slope_heel_cut > 0.0
                    && matches!(support, Support::Double)
                    && let Some(st) = &stairs
                    && st.n == 0
                    && st.slope > 0.0
                    && lifting == Some(_side)
                    // ... and only while that foot stands ON the slope (the
                    // cut applied on the flat approach at 7 deg emptied the
                    // box and fell at 3.3 s, 2026-10-17).
                    && {
                        let x = steps.sole[_side].x;
                        x >= st.start && x <= st.start + st.slope_len
                    }
                {
                    let deg = st.slope.atan().to_degrees();
                    let d = (slope_heel_cut * deg).min(slope_heel_cut_max);
                    return Some((
                        [prof.sole_centre_x + d, 0.0, -prof.sole_below_origin],
                        ((sole_half_l - d).max(0.005), contact_cfg.cop_half.1),
                    ));
                }
                // (STAIR_ROLL_PATCH, a narrow strip at the edge of a pitched
                // sole, is removed.)
                None
            },
        );

        // ---- P1: DCM-tracking CoM acceleration -------------------------
        // (DCM_LEAD_S, a sensor-delay lead on the measured DCM, is removed.)
        let xi = dcm_of(&com, &com_vel, omega);
        let mut r = plan.dcm.reference(t);
        // For a LANDED (re-step) reference the DCM law tracks the
        // reference's own DCM and CoP (its CoM and contact forces) instead of
        // the re-planned LIPM. Measured 2026-10-13 on ld7_a: without it the
        // walker's CoM x lagged the reference by 6 cm at the second landing
        // and 12 cm at the end and the robot fell backward; with it the CoM
        // tracks within 3 cm. (REF_WBC_LANDED, default on, is removed with
        // REF_WBC; the detectors only pick a landed reference when the
        // library has no other.)
        if let (Some(mode), Some(lib)) = (ctl.ref_mode, ctl.ref_lib.as_ref())
            && lib.landed
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
        let dcm_err = (xi - r.xi).norm();
        if push_yield_v > 0.0
            && !ctl.push_yield_done
            && vx2_sched
            && let Some(v_hi) = vx2_cmd_speed
            && v_hi > push_yield_v
            // PUSH_YIELD_VMIN: only from this commanded speed up (at 0.30 m/s
            // the lowering alone handles the pushes and the yield cost the
            // backward 11 N.s 48 -> 46, 2026-10-16). 0 = any speed.
            && v_hi >= push_yield_vmin
            && vx2_z_from.is_some_and(|t0| t >= t0)
            && dcm_err > push_yield_trig
            // PUSH_YIELD_NO_FWD=1: never yield on a FORWARD error. Measured
            // 2026-10-16 at 0.50 m/s (N=48): yielding to 0.20 took the
            // backward 11 N.s from 19 to 44 but the forward 11 N.s from 45
            // to 34 -- slowing the plan under a forward push leaves the body
            // ahead of it.
            && !(push_yield_no_fwd && (xi.x - r.xi.x) > 0.5 * push_yield_trig)
        {
            ctl.push_yield_done = true;
            let v_lo = push_yield_v;
            let stride_hi = (v_hi * t_step).min(vxs_cap_of(v_hi));
            let stride_lo = (v_lo * t_step).min(vxs_cap_of(v_lo));
            let scale_hi = (stride_hi / (v_hi * t_step)).clamp(vx2_tscale_min, 1.0);
            let scale_lo = (stride_lo / (v_lo * t_step)).clamp(vx2_tscale_min, 1.0);
            let ratio = scale_lo / scale_hi;
            // Every future landing advances by stride_lo instead of
            // stride_hi: the n-th future step is pulled back n * delta.
            let delta = stride_hi - stride_lo;
            let i_now = plan.gait.index_at(t);
            let mut n = 0usize;
            let mut shift = [0.0_f64; 2];
            for j in (i_now + 1)..plan.gait.slices.len() {
                if let Some(sw) = plan.gait.slices[j].support.swing() {
                    n += 1;
                    shift[sw] = n as f64 * delta;
                }
                if j < plan.footsteps.at.len() {
                    for side in 0..2 {
                        plan.footsteps.at[j].sole[side].x -= shift[side];
                    }
                }
            }
            // And every future slice runs at the slower speed's duration.
            for j in (i_now + 1)..plan.gait.slices.len() {
                let t0 = plan.gait.slices[j].t0;
                let d = plan.gait.slices[j].duration() * ratio;
                plan.gait.retime(j, t0 + d);
            }
            plan.rebuild_dcm();
            stats.n_replan += 1;
            if debug.replan { println!("  [diag-replan] site 0 t={t:.3} (line 6798)"); }
            println!("  [yield] t={t:.3} dcm_err={dcm_err:.3}: speed {v_hi:.2} -> {v_lo:.2} m/s, stride {stride_hi:.3} -> {stride_lo:.3}, slices x{ratio:.3}, {n} steps re-planned");
        }
        if push_replan_trig > 0.0 {
            let i = plan.gait.index_at(t);
            let sl = plan.gait.slices[i].clone();
            match sl.support {
                Support::Single { swing, stance }
                    if ctl.ref_mode.is_none()
                        // Not on a known staircase (its zone, with the crouch
                        // lead / tail): a deep step-down reads as a backward
                        // error and the re-plan cost the -150 mm descent
                        // 41 -> 30 (measured 2026-10-22).
                        && !stairs.as_ref().is_some_and(|st| {
                            // A zero-riser "stair" (the flat benches park one
                            // at x = 2 m) is not a staircase.
                            (st.riser.abs() > 1e-6 || st.down_n > 0)
                                && articara::biped::height::stair_zone_crouch(
                                plan.footsteps.at_slice(i).mid_xy().x,
                                st,
                                stair_crouch_lead,
                                stair_crouch_tail,
                                1.0,
                            ) > 0.0
                        })
                        // PUSH_REPLAN_SLOPE_LEAD (m): nor on a known slope,
                        // from this far before its start to its end. The
                        // zero-riser test above lets a slope through, and
                        // there the re-plan costs the backward 11 N.s 18 -> 4
                        // (measured 2026-10-19: the re-laid plan meets the
                        // slope entry with a backward DCM error, and the
                        // second firing steps back off the slope). 0 = off.
                        && !(push_replan_slope_lead > 0.0
                            && stairs.as_ref().is_some_and(|st| {
                                let x = plan.footsteps.at_slice(i).mid_xy().x;
                                st.n == 0
                                    && st.slope != 0.0
                                    && x >= st.start - push_replan_slope_lead
                                    && x <= st.start + st.slope_len
                            })) =>
                {
                    // The plan in force when this single support began.
                    if ctl.push_replan_state.as_ref().is_none_or(|st| st.0 != i) {
                        let base = plan.footsteps.at_slice(i).xy(swing);
                        ctl.push_replan_state = Some((i, plan.footsteps.clone(), plan.dcm.clone(), base, None));
                    }
                    let frac = (t - sl.t0) / sl.duration().max(1e-9);
                    let rem = (sl.t1 - t).clamp(0.0, gp.t_ss);
                    if let Some((_, orig_fs, orig_plan, base, last)) = ctl.push_replan_state.as_mut()
                        && frac < push_replan_freeze
                    {
                        let e = xi - orig_plan.reference(t).xi;
                        // The ankle can hold an error up to the sole's half
                        // size by moving the CoP; only the rest diverges.
                        let half = na::Vector2::new(sole_half_l * push_replan_ankle[0], sole_half_w * push_replan_ankle[1]);
                        let grow = (omega * rem).exp();
                        let d = na::Vector2::new(
                            {
                                let dp = e.x.clamp(-half.x, half.x);
                                dp + (e.x - dp) * grow
                            },
                            {
                                let dp = e.y.clamp(-half.y, half.y);
                                dp + (e.y - dp) * grow
                            },
                        );
                        let v_now = (vx2_sched && vx2_z_from.is_some_and(|t0| t >= t0))
                            .then_some(vx2_cmd_speed)
                            .flatten();
                        let back_ok = (!push_replan_back_only || (e.x < 0.0 && e.y.abs() < -e.x))
                            && !v_now.is_some_and(|v| v > push_replan_vmax);
                        // (PUSH_REPLAN_LAT, the outward lateral step too, is
                        // removed.)
                        if ctl.push_replan_slice == Some(i)
                            || (back_ok && d.norm() > push_replan_trig && e.norm() > push_replan_emin)
                        {
                            let st = orig_fs.at_slice(i).xy(stance);
                            let mut l = *base + d;
                            l.x = l.x.clamp(st.x - push_replan_max_back, st.x + push_replan_max_fwd);
                            // Lateral: +1 when the swing foot normally lands at +y of the stance foot.
                            let side = if base.y > st.y { 1.0 } else { -1.0 };
                            let w = (l.y - st.y) * side;
                            // Never widen beyond what the plan already had: the
                            // floor is the smaller of MIN_W and the planned gap
                            // (G1 walks at ~0.104 m; a 0.12 floor widened every
                            // backward re-plan by 2 cm, measured 2026-10-21).
                            let min_w = push_replan_min_w.min(((base.y - st.y) * side).max(0.0));
                            // Inward of the minimum gap (a crossing): leave this
                            // landing to the MPC (its crossing gate,
                            // MPC_FOOTSTEP_CROSS_*) -- do not engage, and once
                            // engaged keep the last landing.
                            let inward_skip = w < 0.0;
                            let w = if w >= min_w {
                                w.min(push_replan_max_w)
                            } else {
                                min_w
                            };
                            l.y = st.y + side * w;
                            if !inward_skip && last.is_none_or(|p| (p - l).norm() > 0.005) {
                                let mut fs = orig_fs.clone();
                                fs.replan_from_landing(&plan.gait, i, swing, l, push_replan_stand, push_replan_ramp);
                                plan.footsteps = fs;
                                plan.rebuild_dcm();
                                if ctl.push_replan_slice != Some(i) {
                                    stats.n_push_replan += 1;
                                    stats.n_replan += 1;
                                    if debug.replan { println!("  [diag-replan] site 1 t={t:.3} (line 6901)"); }
                                }
                                ctl.push_replan_slice = Some(i);
                                ctl.push_replan_shift = (l - *base).norm();
                                if debug.push_replan {
                                    println!("  [replan] t={t:.3} slice {i} frac={frac:.2} e=({:+.3},{:+.3}) d=({:+.3},{:+.3}) landing ({:.3},{:+.3}) -> ({:.3},{:+.3})", e.x, e.y, d.x, d.y, base.x, base.y, l.x, l.y);
                                }
                                *last = Some(l);
                            }
                        }
                    }
                }
                _ => {}
            }
        }
        // Backward-dominant only, and never under an engaged recovery
        // reference (measured: firing on lateral pushes cost the lateral
        // ladder 136 -> 129 while the reference was running).
        // (The rate-predicted error, REF_TRIG_PRED, is removed: at its
        // default 0 the prediction was the error itself.)
        let back_err = (xi - r.xi).x;
        if push_z_drop > 0.0
            && back_err < -push_z_drop_trig
            && (xi - r.xi).y.abs() < push_z_drop_lat_max
            && ctl.ref_mode.is_none()
            && t >= ctl.push_z_drop_until
        {
            ctl.push_z_drop_until = t + push_z_drop_t;
            stats.n_push_z_drop += 1;
            println!("  [zdrop] t={t:.3} backward DCM error {:+.3} m -> CoM {:.3} m lower for {push_z_drop_t:.2} s", (xi - r.xi).x, push_z_drop);
        }
        // Engage a recovery reference when the DCM error toward the STANCE
        // side exceeds REF_TRIG_DCM in single support (the detector).
        // (REF_SLOPE_GATE, no reference on a known slope, is removed.)
        if ref_any
            && ctl.ref_mode.is_none()
            && t >= ctl.ref_cooldown_until
            && slice_frac <= ref_trig_max_frac
            && let Support::Single { stance, swing } = support
        {
            let lateral = (xi - r.xi).y * if stance == 0 { 1.0 } else { -1.0 };
            if lateral.abs() < 0.005 {
                ctl.ref_lat_quiet_t = t;
            }
            let forward = (xi - r.xi).x;
            // The library's lateral-push references (`impulse_deg` 90).
            let want_deg = 90.0;
            if ref_trig_dcm > 0.0 && lateral > ref_trig_dcm && ref_libs.iter().any(|l| l.impulse_deg == want_deg) {
                // The push happened when the lateral error last was quiet;
                // the reference is entered that far in.
                let tau0 = if !ref_delay_comp { 0.0 } else { (t - ctl.ref_lat_quiet_t).clamp(0.0, 0.3) };
                println!("  [ref] t={t:.3} trigger: dcm (lateral DCM error toward stance {lateral:+.3} m, forward {forward:+.3} m, dir {want_deg:.0}, tau0 {tau0:.3})");
                let actual = Footsteps::from_fk(data, rig.foot_mi, prof.sole_centre_x, prof.sole_below_origin);
                // Pick the reference meant for the nearest swing phase.
                // (Selecting by the swing foot's height was tried first:
                // on the flat in-place gait the apex comes late, so the
                // height does not order the phases.)
                // Phase AT THE PUSH (the detector fires tau0 later).
                let frac_push = slice_frac - tau0 / plan.gait.slices[slice_idx].duration().max(1e-6);
                // Nearest walking speed first (the commanded speed once the
                // speed change has ramped in, else the nominal), then phase.
                let v_now = match vx2_change {
                    Some((at, v2, ramp2)) if plan.gait.steps_taken(t) >= at + ramp2 => v2,
                    _ => vx,
                };
                let pick = (0..ref_libs.len())
                    .filter(|&k| ref_libs[k].impulse_deg == want_deg && !ref_libs[k].landed)
                    .min_by(|&a, &b| {
                        let da = ((ref_libs[a].vx - v_now).abs() * 10.0, (ref_libs[a].swing_phase - frac_push).abs());
                        let db = ((ref_libs[b].vx - v_now).abs() * 10.0, (ref_libs[b].swing_phase - frac_push).abs());
                        (da.0 + da.1).partial_cmp(&(db.0 + db.1)).unwrap()
                    })
                    .unwrap_or(0);
                ctl.ref_lib = Some(ref_libs[pick].clone());
                let lib = ctl.ref_lib.as_ref().unwrap();
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
                while j < plan.gait.slices.len() && ph < lib.phases.len() {
                    let is_ss = matches!(plan.gait.slices[j].support, Support::Single { .. });
                    // Count the slices of this phase (DS phases may be split).
                    let mut jend = j + 1;
                    if !is_ss {
                        while jend < plan.gait.slices.len() && matches!(plan.gait.slices[jend].support, Support::Double) {
                            jend += 1;
                        }
                    }
                    let n = (jend - j) as f64;
                    for jj in j..jend {
                        let remaining = if ph == 0 { (lib.phases[0] - tau0).max(0.05) } else { lib.phases[ph] };
                        let end = t_cursor + remaining / n;
                        plan.gait.retime(jj, end);
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
                        while jj < plan.gait.slices.len() && plan.gait.slices[jj].support.swing().is_none() {
                            jj += 1;
                        }
                        if jj >= plan.gait.slices.len() {
                            break;
                        }
                        let foot = mode.foot(ref_sw);
                        let land = mode.world(lib, lib.sole_at(ref_sw, t_end));
                        let d = (land - plan.footsteps.at_slice(jj).sole[foot]).xy();
                        plan.footsteps.shift_next(jj, foot, d);
                        if ph == 0 { land1 = land; } else if ph == 2 { land2 = land; }
                        ref_sw = 1 - ref_sw;
                        jj += 1;
                    }
                }
                let _ = first_ss_after;
                plan.rebuild_dcm();
                plan.landing.event_mode = true;
                ctl.ref_mode = Some(mode);
                stats.ref_engagements += 1;
                println!(
                    "  [ref] t={t:.3} engaged: stance={} mirror={:+.0} origin=({:.3},{:.3}) phases={:?} land1=({:.3},{:.3}) land2=({:.3},{:.3}) total={:.3}",
                    stance, mode.mirror, mode.origin.x, mode.origin.y, lib.phases, land1.x, land1.y,
                    land2.x, land2.y, lib.total()
                );
            
            }
        }
        m.max_dcm_err = m.max_dcm_err.max(dcm_err);
        // Post-push accumulation. The push fires late in the tick, after this
        // point, so the firing tick's error is still the undisturbed one and
        // the window starts one tick later -- which is what we want.
        if in_push_window {
            m.win_dcm_err_max = m.win_dcm_err_max.max(dcm_err);
        }
        if m.push_at_t.is_some() {
            m.push_dcm_err_max = m.push_dcm_err_max.max(dcm_err);
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
                m.push_exceeded = true;
                m.push_last_exceed = Some(t);
            }
        }

        // ---- commanded ZMP ----------------------------------------------
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
        m.max_zmp_clamp = m.max_zmp_clamp.max(clamped);
        // Pressure the polygon refused (world x, y), for MOM_CMP.
        let zmp_deficit = p_raw - p_cmd;

        // ---- contact plan (CONTACT_PLAN) --------------------------------
        if contact_plan_on
            && ctl.ref_mode.is_none()
            && let Support::Single { swing, .. } = support
            && dcm_err >= contact_plan_trig
        {
            let i = plan.gait.index_at(t);
            if ctl.cp_slice != Some(i) {
                ctl.cp_slice = Some(i);
                ctl.cp_applied = na::Vector2::zeros();
            }
            let seg = &plan.dcm.segs[plan.dcm.index_at(t)];
            let sit = articara::biped::contact_plan::Situation {
                xi,
                p: seg.at(t),
                p_dot: seg.slope(),
                omega,
                t,
                nominal_end: plan.gait.slices[i].t1,
                xi_eos_plan: plan.dcm.eos_at(t),
                swing_sign: if swing == 0 { 1.0 } else { -1.0 },
            };
            // (CP_J_AUTO, the sensitivity differenced from the plan, and
            // CP_TOUCHDOWN, planning for the DCM at touchdown, were not
            // adopted and are removed.)
            let dec = articara::biped::contact_plan::plan(&sit, &contact_plan_cfg);
            if debug.cp {
                println!(
                    "  [cp] t={t:.3} slice={i} err={dcm_err:.4} xi=({:.3},{:+.3}) eos_plan=({:.3},{:+.3}) pred_nom=({:.3},{:+.3}) shift=({:+.3},{:+.3}) dT={:+.3} unmet={:.4} cost={:.3}",
                    xi.x, xi.y, sit.xi_eos_plan.x, sit.xi_eos_plan.y,
                    articara::biped::contact_plan::predict(&sit, (sit.nominal_end - t).max(0.0)).x,
                    articara::biped::contact_plan::predict(&sit, (sit.nominal_end - t).max(0.0)).y,
                    dec.shift.x, dec.shift.y, dec.new_end.map_or(0.0, |e| e - sit.nominal_end), dec.unmet, dec.cost
                );
            }
            // (CP_TREAD_BOX, snapping the landing into the tread's sole
            // window, MEASURED HARMFUL 2026-09-25: landing fully on the tread
            // does not predict survival at the 60 mm riser, and snapping to
            // the window edge took 26/32 -> 17/32. Removed; terrain awareness
            // belongs in a cost with margin, not a snap to an edge.)
            let mut rebuilt = false;
            // Like ADAPT_STEP did: place the foot once per step, keeping the
            // largest demand seen, instead of re-placing it every tick.
            //
            // That is a RATCHET: the landing only ever moves further from
            // nominal. With the shipped 15 mm trigger it fires a handful of
            // times in an emergency and the ratchet is harmless, but run
            // continuously it walks the foot out to its bound and the stair
            // approach dies (measured 2026-09-24 on the randomised stairs:
            // 0/192 at every shift weight, falling at 21-23 s). (CP_TRACK,
            // following the current decision instead, is removed: with the
            // champion's CP_MAX_*=0 the shift is always zero and both rules
            // give zero.)
            let d = if dec.shift.norm() > ctl.cp_applied.norm() {
                dec.shift - ctl.cp_applied
            } else {
                na::Vector2::zeros()
            };
            if d.norm() > 1e-9 && !plan.landing.landing_closed(i) {
                // `shift_from` shifts the swing foot's landing in slice `i`
                // AND every later slice, so the adjusted position becomes the
                // nominal that the NEXT step's residual is measured against.
                // Under continuous firing the foot walks outward one bound per
                // step (measured 2026-09-25: lateral spacing 127 -> 153 mm
                // median against a 30 mm limit). CP_SHIFT_NEXT, moving only
                // this landing, was not adopted and is removed.
                plan.footsteps.shift_from(i, swing, d);
                ctl.cp_applied = dec.shift;
                stats.n_cp_shift += 1;
                rebuilt = true;
            }
            if let Some(ne) = dec.new_end
                && ne > t + dt
                && ctl.cp_retimed_slice != Some(i)
            {
                let dd = plan.gait.retime(i, ne);
                stats.n_cp_retime += 1;
                stats.cp_retime_total += dd.abs();
                ctl.cp_retimed_slice = Some(i);
                // The planner has now set this step's end; the landing
                // extension must not move it again (the two would fight
                // over the same boundary, and extension IS mechanism B).
                plan.landing.max_extension = 0.0;
                rebuilt = true;
            }
            if rebuilt {
                plan.rebuild_dcm();
                stats.n_replan += 1;
                if debug.replan { println!("  [diag-replan] site 2 t={t:.3} (line 7454)"); }
            }
        }
        if ctl.replan_engaged {
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
            if t + 1e-9 >= ctl.preview_next {
                let mut samples = Vec::with_capacity(com_preview_n);
                for k in 0..com_preview_n {
                    let begin = t + k as f64 * com_preview_dt;
                    let end = begin + com_preview_dt;
                    let f0 = articara::biped::preview::forecast_feet(
                        &plan.gait,
                        &plan.footsteps,
                        &actual,
                        &stance_sides,
                        t,
                        begin,
                    );
                    let f1 = articara::biped::preview::forecast_feet(
                        &plan.gait,
                        &plan.footsteps,
                        &actual,
                        &stance_sides,
                        t,
                        end - 1e-9,
                    );
                    let b0 = SupportBox::from_stance_yawed(
                        &f0,
                        plan.gait.support_at(begin),
                        (sole_half_l * cop_frac, sole_half_w * cop_frac),
                        zmp_margin,
                    );
                    let b1 = SupportBox::from_stance_yawed(
                        &f1,
                        plan.gait.support_at(end - 1e-9),
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
                    let (zmp_ref, xi_ref) = (
                        plan.dcm.reference(begin + 0.5 * com_preview_dt).p,
                        plan.dcm.reference(end).xi,
                    );
                    samples.push(articara::biped::preview::PreviewSample {
                        duration: com_preview_dt,
                        bounds,
                        zmp: zmp_ref,
                        xi: xi_ref,
                    });
                }
                let preview_solve_start = std::time::Instant::now();
                let preview_solution = articara::biped::preview::replan(
                    omega,
                    com.xy(),
                    com_vel.xy(),
                    ctl.preview_p,
                    &samples,
                    com_preview_smooth,
                );
                let preview_solve_ns = preview_solve_start.elapsed().as_nanos();
                let preview_accepted = preview_solution.is_some();
                match preview_solution {
                    Some(plan) => {
                        ctl.preview_p = plan.zmp[0];
                        ctl.preview_c = plan.com[0];
                        ctl.preview_v = plan.velocity[0];
                        stats.preview_error = plan.terminal_error;
                        stats.preview_count += 1;
                        if let Some(f) = diag.preview_traj_file.as_mut() {
                            use std::io::Write as _;
                            for (k, smp) in samples.iter().enumerate() {
                                let (Some(z), Some(c)) = (plan.zmp.get(k), plan.com.get(k)) else { break };
                                writeln!(
                                    f,
                                    "{t:.4},{k},{:.4},{:.5},{:.5},{:.5},{:.5},{:.5},{:.5},{:.5},{:.5},{:.5},{:.5}",
                                    smp.duration, z.x, z.y, c.x, c.y, smp.zmp.x, smp.zmp.y,
                                    smp.bounds.lo.x, smp.bounds.lo.y, smp.bounds.hi.x, smp.bounds.hi.y
                                )
                                .unwrap();
                            }
                        }
                    }
                    None => {
                        ctl.preview_p = current_box.clamp(&p_cmd).0;
                        stats.preview_failures += 1;
                    }
                }
                if let Some(writer) = &mut diag.com_preview_runtime {
                    writeln!(writer, "{t:.6},{preview_solve_ns},{},{},{:.9},{},{}",
                        u8::from(preview_solve_ns > 50_000_000),
                        u8::from(preview_accepted),
                        stats.preview_error,
                        0, // applied_lateral: COM_REPLAN_LATERAL (removed) was off
                        com_preview_n,
                    ).unwrap();
                }
                // (COM_PREVIEW_PHASE_LOCK, a discrete-clock update rate, is
                // removed.)
                ctl.preview_next = t + com_replan_dt;
            }
            let optimized = current_box.clamp(&ctl.preview_p).0;
            // Forward only (COM_REPLAN_LATERAL and its gain are removed).
            p_cmd.x += com_replan_forward_gain * (optimized.x - p_cmd.x);
        }
        // Variable-height form when a stair is in the scene: the LIPM's
        // constant-height omega is ~20% too stiff while the CoM is being
        // driven DOWN a riser. Gated so level and validated runs are bit-
        // identical (VH_DCM=0 restores the classic form).
        let a_xy_raw = if stairs.is_some() && vh_dcm {
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
        let a_xy = if let Some(previous) = ctl.previous_a_xy {
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
        ctl.previous_a_xy = Some(a_xy);
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
            let fs = plan.footsteps.at_slice(slice_idx);
            // The mean planned sole height (STAIR_DESC_MIN, the lower of the
            // two on a descent, is removed).
            let tgt_h = 0.5 * (fs.sole[0].z + fs.sole[1].z);
            // A slope (TERRAIN_SLOPE) has no riser: its rise per step is
            // the grade times the nominal stride, else the rate is zero and
            // the height reference never leaves the spawn level (measured
            // 2026-10-15 on +3 deg: CoM 0.683 absolute for the whole climb).
            let riser_eff = if st.n == 0 && st.slope != 0.0 {
                st.slope.abs() * (vx.abs() * t_step).max(0.10)
            } else {
                st.riser.abs()
            };
            let rate = stair_climb_rate * riser_eff / t_step.max(1e-6);
            let dh = (tgt_h - ctl.terrain_track).clamp(-rate * dt, rate * dt);
            ctl.terrain_track += dh;
            dh / dt
        } else {
            0.0
        };
        let (z_ref, zd_ref, zdd_ref) = if ctl.replan_engaged && com_height_replan && plan.landing.clock.is_some() {
            // Where the CoM height aims while BOTH feet count: historically
            // the mean sole height. Measured 2026-09-24 on the 80 mm riser
            // with the landed foot admitted at the start of double support
            // (CONTACT_ADMIT_CONFIRMED): the half-rise then completes before
            // the trailing leg has left the lower tread, that leg reaches
            // single support with its knee at 0.09 rad (champion: 0.34), and
            // the next swing task has no authority left -- the foot launches
            // to 1.2 m. (COM_HEIGHT_DS_FRAC, keeping the CoM at the lower
            // sole through double support, was not adopted and is removed;
            // the level is the mean.)
            // All height rules live in src/biped/height.rs (one place for the
            // stance level, the descent lead, the stair crouch, the reach
            // caps and the slewed drops); the arithmetic is the one used
            // inline before 2026-10-19, bit-identical.
            use articara::biped::height as hr;
            let soles_z: Vec<f64> = stance_sides.iter().map(|&side| steps.sole[side].z).collect();
            let h = hr::stance_level(&soles_z, None);
            // The landing this swing is heading for, or -- in a double
            // support -- the landing of the swing that follows it.
            let next_landing = {
                let i = plan.gait.index_at(t);
                (i..plan.gait.slices.len().min(i + 4))
                    .find_map(|j| plan.gait.slices[j].support.swing().map(|sw| (j, sw)))
                    .map(|(j, sw)| (sw, plan.footsteps.at_slice(j).sole[sw]))
            };
            let h = hr::descent_lead(h, next_landing.map(|(_, l)| l.z), com_height_desc_lead);
            let crouch = match &stairs {
                Some(st) => hr::stair_zone_crouch(steps.mid_xy().x, st, stair_crouch_lead, stair_crouch_tail, stair_zone_crouch),
                None => 0.0,
            };
            let target = if crouch > 0.0 {
                z_com + h - terrain_base - crouch
            } else {
                z_com + h - terrain_base
            };
            // (REACH_HEIGHT / _FLOOR, the height capped by the legs' reach
            // over every sole, and COM_HEIGHT_DESC_REACH, the same cap for
            // the landing of a descent, were not adopted and are removed.)
            if (target - plan.height.target).abs() > 1e-6 {
                plan.height.retarget(t, target, com_height_time);
            }
            plan.height.at(t)
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
                z_com + (ctl.terrain_track - terrain_base),
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
        let z_drop_target = articara::biped::height::drop_target(
            if vx2_z_from.is_some_and(|t0| t >= t0) { vx2_z_drop } else { 0.0 },
            if t < ctl.push_z_drop_until { push_z_drop } else { 0.0 },
        );
        let (z_ref, zd_ref) = if z_drop_target > 0.0 || ctl.vx2_z_track.track > 0.0 {
            let d = ctl.vx2_z_track.step(z_drop_target, 0.05, dt);
            (z_ref - ctl.vx2_z_track.track, zd_ref - d / dt)
        } else {
            (z_ref, zd_ref)
        };
        // (MPC_REACH, the CoM height capped by the MPC from the legs' reach,
        // was not adopted and is removed.)
        let a_z = zdd_ref + kp_com * (z_ref - com.z) + kd_com * (zd_ref - com_vel.z - diag_vz_bias_height);
        let a_com = na::Vector3::new(a_xy.x, a_xy.y, a_z);

        // Replan both sole wrenches from the measured centroidal state. The
        // first horizon command becomes the WBC force regularisation target;
        // its resultant acceleration also becomes the CoM task reference.
        // This removes the old conflict where ZMP, height and load split were
        // generated by independent controllers.
        if stair_wrench_mpc && wrench_mpc_always {
            // A held result has no age limit (STAIR_WRENCH_MAX_AGE removed).
            if t + 1e-9 >= ctl.wrench_mpc_next {
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
                // (MPC_TARGET_PREVIEW, the horizontal target from the CoM
                // preview QP, was not adopted and is removed.)
                let mut target_com =
                    com + com_vel * horizon_time + a_com * (0.5 * horizon_time * horizon_time);
                let mut target_velocity = com_vel + a_com * horizon_time;
                // `high` is the CURRENT stance foot, purely so the code below
                // has a name for "the loaded side"; the schedules are built
                // from the gait.
                let high = usize::from(!stance_sides.contains(&0));
                // The height reference is the walk's own.
                let (hz, hzd, _) = plan.height.at(t + horizon_time);
                target_com.z = hz;
                target_velocity.z = hzd;
                let low = 1 - high;
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
                    // The load split is the GAIT's, read forward along the
                    // horizon. Double support is an even split; single
                    // support puts the whole weight on the stance sole. This
                    // is the schedule the walk is already executing, so the
                    // MPC predicts the motion rather than a primitive.
                    let high_share = {
                        let idx = plan.gait.index_at(t + stage_dt);
                        match plan.gait.slices[idx].support.swing() {
                            None => 0.5,
                            Some(sw) => {
                                if sw == high {
                                    0.0
                                } else {
                                    1.0
                                }
                            }
                        }
                    };
                    let mut reference = [0.0; 2];
                    let mut minimum = [0.0; 2];
                    reference[high] = high_share * weight;
                    reference[low] = (1.0 - high_share) * weight;
                    minimum[high] = stair_wrench_low_min * weight * high_share;
                    minimum[low] = stair_wrench_low_min * weight * (1.0 - high_share);
                    for side in 0..2 {
                        let capacity = f_max_per_foot;
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
                    ctl.wrench_mpc_command.unwrap_or([articara::biped::wrench_mpc::Wrench::zeros(); 2]);
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
                if mpc_seed_previous == 2 || (mpc_seed_previous == 1 && ctl.wrench_mpc_command.is_none()) {
                    for side in 0..2 {
                        previous[side][5] = measured_pre[side].max(0.0);
                    }
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
                if wrench_mpc_apply {
                    let end = t + stair_wrench_horizon as f64 * stair_wrench_preview_dt;
                    let fc = articara::biped::preview::forecast_feet(
                        &plan.gait,
                        &plan.footsteps,
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
                // sign of y is inward: MPC_FOOTSTEP_MAX_Y_IN. (The v6.13
                // backward parts MPC_FOOTSTEP_SHIFT_REST / _BACK_ONLY /
                // _BACK_PERSIST / _FLOOR_BACK_ERR / _BACK_VMAX / _HOLD_SHIFT /
                // _CAPTURE_FLOOR went to PUSH_REPLAN in v6.14 and are removed.)
                // MPC_FOOTSTEP_CROSS_Y gate (see its declaration).
                let cross_now: Option<(usize, bool)> = if mpc_footstep_cross_y > 0.0
                    && let Support::Single { swing: sw_c, .. } = support
                {
                    let i = plan.gait.index_at(t);
                    if let Some((li, lp)) = ctl.mpc_fs_cross_latched
                        && li == i
                    {
                        Some((li, lp))
                    } else {
                        let stance_c = 1 - sw_c;
                        let f = plan.footsteps.at_slice(i);
                        // The stance foot lies at +y of the swing foot.
                        let plus = f.sole[stance_c].y > f.sole[sw_c].y;
                        let ex = (xi.y - f.sole[stance_c].y) * if plus { 1.0 } else { -1.0 };
                        if ex > mpc_footstep_cross_ex {
                            ctl.mpc_fs_cross_latched = Some((i, plus));
                            stats.n_mpc_fs_cross += 1;
                            if debug.mpc_fs_cross {
                                println!("  [fs-cross] t={t:.3} slice {i} swing {sw_c} DCM {ex:+.3} m past the stance sole");
                            }
                            Some((i, plus))
                        } else {
                            None
                        }
                    }
                } else {
                    None
                };
                let inward_bound = |i: usize| -> (f64, bool) {
                    let (_, plus) = support
                        .swing()
                        .map(|sw| landing_inward(&plan.footsteps, i, sw))
                        .unwrap_or((0.0, true));
                    let mut b = mpc_footstep_max_y_in;
                    if mpc_footstep_y_in_fwd > 0.0
                        && xi.x - plan.dcm.reference(t).xi.x >= mpc_footstep_y_in_fwd_trig
                    {
                        b = b.max(mpc_footstep_y_in_fwd);
                    }
                    if let Some((ci, cplus)) = cross_now
                        && ci == i
                    {
                        return (b.max(mpc_footstep_cross_y), cplus);
                    }
                    (b, plus)
                };
                let landing = if mpc_footstep
                    && ctl.push_replan_slice != Some(plan.gait.index_at(t))
                    && wrench_mpc_apply
                    && let Support::Single { swing, .. } = support
                    && {
                        let i = plan.gait.index_at(t);
                        if mpc_footstep_trig > 0.0
                            && dcm_err >= mpc_footstep_trig
                            && !ctl.mpc_fs_armed.is_some_and(|(a, u)| a <= i && i <= u)
                        {
                            // A step is two slices (single + double support).
                            ctl.mpc_fs_armed = Some((i, i));
                        }
                        mpc_footstep_trig == 0.0
                            || ctl.mpc_fs_armed.is_some_and(|(a, u)| a <= i && i <= u)
                    }
                    && {
                        // (The freeze by swing phase, MPC_FOOTSTEP_FREEZE_FRAC /
                        // _RISE, defaulted to inf = never and is removed.)
                        // Event mode: no landing moves while the foot is
                        // probing for the ground (measured 2026-10-04: the
                        // goal moved 5-12 cm under a foot 2 cm from contact).
                        !plan.landing.landing_closed(plan.gait.index_at(t)) && ctl.ref_mode.is_none()
                    }
                {
                    (0..stair_wrench_horizon)
                        .find(|&k| {
                            let at = t + (k as f64 + 0.5) * stair_wrench_preview_dt;
                            match plan.gait.support_at(at) {
                                Support::Double => true,
                                Support::Single { swing: sw, .. } => sw != swing,
                            }
                        })
                        .map(|first_node| articara::biped::wrench_mpc::Landing {
                            foot: swing,
                            first_node,
                            min_first_node: first_node,
                            position: [contacts[swing].position.x, contacts[swing].position.y],
                            // MPC_FOOTSTEP_TERRAIN_W (terrain as a cost) removed.
                            terrain_x: None,
                            weight: mpc_footstep_w,
                            max: {
                                let i = plan.gait.index_at(t);
                                let (inward, inward_is_plus) = inward_bound(i);
                                let mut m = mpc_footstep_max;
                                if inward_is_plus {
                                    m[1] = if cross_now.is_some_and(|(ci, _)| ci == i) { inward } else { m[1].min(inward) };
                                }
                                m
                            },
                            // The answer is the total since the slice began
                            // (MPC_FOOTSTEP_INCREMENT removed).
                            prior: [0.0; 2],
                            min: {
                                let i = plan.gait.index_at(t);
                                // (MPC_FOOTSTEP_STAIR_BACK, inf by default, removed.)
                                let mut m = mpc_footstep_max;
                                let (inward, inward_is_plus) = inward_bound(i);
                                if !inward_is_plus {
                                    m[1] = if cross_now.is_some_and(|(ci, _)| ci == i) { inward } else { m[1].min(inward) };
                                }
                                m
                            },
                        })
                        .map(|mut land| {
                            // Crossing: the LIPM capture floor on the inward shift.
                            if mpc_footstep_cross_floor > 0.0
                                && let Some((ci, cplus)) = cross_now
                                && ci == plan.gait.index_at(t)
                            {
                                let sl = &plan.gait.slices[ci];
                                let rem = (sl.t1 - t).max(0.0);
                                let sgn = if cplus { 1.0 } else { -1.0 };
                                let stance_y = plan.footsteps.at_slice(ci).sole[1 - land.foot].y;
                                let edge = stance_y + sgn * sole_half_w;
                                let xi_td = edge + (xi.y - edge) * (omega * rem).exp();
                                // The ORIGINAL landing: the contact position already
                                // carries the shift applied so far this slice, and
                                // the box acts on the total shift from the original.
                                let applied_y = if ctl.mpc_fs_slice == Some(ci) { ctl.mpc_fs_applied.y } else { 0.0 };
                                let base = land.position[1] - applied_y;
                                // Inward shift needed for the landing to come within
                                // floor * half width of xi_td.
                                let need = (xi_td - sgn * mpc_footstep_cross_floor * sole_half_w - base) * sgn;
                                if need > 0.0 {
                                    if cplus {
                                        land.min[1] = land.min[1].min(-need);
                                        land.max[1] = land.max[1].max(need);
                                    } else {
                                        land.max[1] = land.max[1].min(-need);
                                        land.min[1] = land.min[1].max(need);
                                    }
                                }
                                if debug.mpc_fs_cross {
                                    println!("  [fs-cross-floor] t={t:.3} rem={rem:.3} xi.y={:+.3} xi_td={xi_td:+.3} base={base:+.3} need={need:+.3} max={:?} min={:?}", xi.y, land.max, land.min);
                                }
                            }
                            land
                        })
                } else {
                    None
                };
                let input = articara::biped::wrench_mpc::Input {
                    com,
                    velocity: if diag_vz_bias_mpc != 0.0 {
                        com_vel + na::Vector3::new(0.0, 0.0, diag_vz_bias_mpc)
                    } else {
                        com_vel
                    },
                    angular_momentum: na::Vector3::new(h[0], h[1], h[2]),
                    target_com,
                    target_velocity,
                    target_acceleration: a_com,
                    com_z_max: None,
                    contacts: [contacts[0].clone(), contacts[1].clone()],
                    // Walking branch: the contact SET over the horizon comes
                    // from the gait, node by node (lift-off and landing inside
                    // the horizon), the same way the CoM preview builds its
                    // support boxes. Repeating the current stance for 0.4 s
                    // -- the historical construction, kept for transfers --
                    // gave a solution that, once applied (WRENCH_MPC_APPLY),
                    // could not walk at all (0/192, measured 2026-09-24).
                    contact_schedule: if wrench_mpc_apply {
                        (0..stair_wrench_horizon)
                            .map(|k| {
                                let at = t + (k as f64 + 0.5) * stair_wrench_preview_dt;
                                match plan.gait.support_at(at) {
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
                    // each node: all of it. (MPC_LOAD_SCHEDULE, the WBC's
                    // double-support load ramp shown to the MPC, was not
                    // adopted and is removed.)
                    activation_schedule: vec![[1.0; 2]; stair_wrench_horizon],
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
                    // (MPC_COP_REF_CMD, the schedule shifted to the WBC's
                    // p_cmd, was not adopted -- the champion sets 0 -- and is
                    // removed; it added an exact zero when off.)
                    reference_net_cop_schedule: (mpc_net_cop_weight > 0.0)
                        .then(|| {
                        (0..stair_wrench_horizon)
                            .map(|k| {
                                let at = t + (k as f64 + 0.5) * stair_wrench_preview_dt;
                                plan.dcm.reference(at).p
                            })
                            .collect()
                    }),
                    reference_dcm_schedule: (mpc_landing_capture_w > 0.0)
                        .then(|| {
                        (0..stair_wrench_horizon)
                            .map(|k| {
                                let at = t + (k as f64 + 0.5) * stair_wrench_preview_dt;
                                plan.dcm.reference(at).xi
                            })
                            .collect()
                    }),
                    min_fz_schedule,
                    previous,
                    landing,
                    dcm_now: (mpc_landing_capture_w > 0.0).then_some(xi),
                    reference_dcm_sensitivity: (mpc_landing_capture_j
                        && mpc_landing_capture_w > 0.0)
                        .then(|| landing)
                        .flatten()
                        .map(|land| {
                            let i = plan.gait.index_at(t);
                            let du = 0.02;
                            let mut cols = [Vec::new(), Vec::new()];
                            for axis in 0..2 {
                                let mut probe = plan.footsteps.clone();
                                let mut d = na::Vector2::zeros();
                                d[axis] = du;
                                // The sensitivity describes the plan the shift
                                // is actually applied to.
                                probe.shift_next(i, land.foot, d);
                                let moved = plan.dcm_for(&probe);
                                cols[axis] = (0..stair_wrench_horizon)
                                    .map(|k| {
                                        let at = t + (k as f64 + 0.5) * stair_wrench_preview_dt;
                                        (moved.reference(at).xi - plan.dcm.reference(at).xi) / du
                                    })
                                    .collect();
                            }
                            (0..stair_wrench_horizon)
                                .map(|k| {
                                    na::Matrix2::from_columns(&[cols[0][k], cols[1][k]])
                                })
                                .collect()
                        }),
                    // MPC_ARM_ABOUT_PRIOR (removed) was the only user.
                    prior_com: None,
                };
                let mut config = articara::biped::wrench_mpc::Config {
                    horizon: stair_wrench_horizon,
                    dt: stair_wrench_preview_dt,
                    mass: total_mass,
                    gravity: G,
                    friction_mu,
                    cop_half: contact_cfg.cop_half,
                    torsion_mu: contact_cfg.mu_torsion,
                    // (MPC_F_MAX_SCALE, a ceiling of its own, is removed:
                    // its default NaN meant the WBC's.)
                    max_fz: f_max_per_foot,
                    validate_solution: mpc_gate,
                    // The cost weights, so the ones that compete with the CoP
                    // term can be measured against it. Defaults are the
                    // shipped values, so leaving them alone is bit-identical.
                    // The removed experiments (MPC_DCM_W, MPC_LANDING_CAPTURE_N,
                    // MPC_LANDING_TIME*, MPC_FZ_SPLIT_EQ, MPC_CAPTURE_COP_W,
                    // MPC_CAPTURE_LEX*, MPC_ARM_ABOUT_PRIOR, MPC_W_LDOT,
                    // MPC_COP_W, MPC_NET_COP_EQ, MPC_REACH_W) keep `Config::default()`,
                    // which is what their defaults were.
                    position_weight: mpc_w_pos,
                    velocity_weight: mpc_w_vel,
                    momentum_weight: mpc_w_mom,
                    acceleration_weight: mpc_w_acc,
                    wrench_weight: mpc_w_wrench,
                    omega,
                    landing_capture_weight: mpc_landing_capture_w,
                    landing_lexicographic: mpc_landing_lex,
                    landing_moment_coupling: mpc_footstep_moment,
                    smooth_weight: mpc_w_smooth,
                    net_cop_weight: mpc_net_cop_weight,
                    ..Default::default()
                };
                config.qp.absolute_gap = mpc_gate;
                // Interior-point solutions carry complementarity slack at the
                // gap setting; the KKT gate's 1e-6 rejected every one.
                config.kkt_complementarity_tolerance = mpc_kkt_comp_tol;
                if let Some(backend) = mpc_backend_override {
                    config.qp.solver = backend;
                }
                if let Some(dir) = &diag.mpc_diag_dir {
                    let capture_start = std::time::Instant::now();
                    let contacts: Vec<_> = input.contacts.iter().map(|c| serde_json::json!({
                        "position":c.position.as_slice(),"rotation":c.rotation.as_slice(),"cop_half":c.cop_half})).collect();
                    // "phase" was the stair transfer's phase (removed); the walk
                    // always wrote its placeholder, `Capture`.
                    let dump = serde_json::json!({"schema":1,"t":t,"ctrl_dt":dt,"mpc_update_dt":stair_wrench_dt,
                        "phase":"Capture","input":{
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
                        format!("{dir}/mpc_{:06}.json", diag.mpc_diag_count),
                        serde_json::to_vec(&dump).unwrap(),
                    )
                    .expect("MPC snapshot");
                    diag.mpc_diag_count += 1;
                    mpc_capture_ns = capture_start.elapsed().as_nanos();
                }
                let previous_age = diag_mpc_time.map_or(-1.0, |stamp| t - stamp);
                let mpc_solve_start = std::time::Instant::now();
                let mpc_result = articara::biped::wrench_mpc::solve_with_workspace(
                    &input,
                    &config,
                    None, // no warm start (MPC_QP_WARM removed)
                );
                let mpc_solve_ns = mpc_solve_start.elapsed().as_nanos();
                if let Some(file) = &mut diag.mpc_runtime_log {
                    use std::io::Write;
                    let mask = u8::from(stance_sides.contains(&0))
                        | (u8::from(stance_sides.contains(&1)) << 1);
                    // Second column: the removed stair transfer's phase,
                    // `Capture` being the walk's placeholder.
                    writeln!(file, "{t:.9},Capture,{mask},{:?},{},{stair_wrench_dt:.9},{mpc_solve_ns},{},{},{previous_age:.9}",
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
                        ctl.wrench_mpc_command = Some(by_side);
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
                        if diag.qp_wrench_log.is_some() {
                            wrench_mpc_frames = Some(input.contacts.clone());
                        }
                        if let Some(f) = diag.mpc_traj_file.as_mut() {
                            use std::io::Write as _;
                            let nan = f64::NAN;
                            let mut row = format!(
                                "{t:.4},{:.5},{:.5},{:.5},{:.5},{:.5},{:.5}",
                                com.x, com.y, com.z, input.target_com.x, input.target_com.y, input.target_com.z
                            );
                            for k in 0..16 {
                                match solution.trajectory.get(k) {
                                    Some(n) => row += &format!(",{:.5},{:.5},{:.5}", n[0].x, n[0].y, n[0].z),
                                    None => row += &format!(",{nan},{nan},{nan}"),
                                }
                            }
                            row += &format!(",{:.5},{:.5}", diag_mpc_net_cop[0], diag_mpc_net_cop[1]);
                            for k in 0..16 {
                                match input.reference_net_cop_schedule.as_ref().and_then(|v| v.get(k)) {
                                    Some(c) => row += &format!(",{:.5},{:.5}", c.x, c.y),
                                    None => row += &format!(",{nan},{nan}"),
                                }
                            }
                            for foot in 0..2 {
                                let p = input.contacts[foot].position;
                                row += &format!(",{:.5},{:.5},{:.5}", p.x, p.y, p.z);
                            }
                            let on = input.contact_schedule.first().copied().unwrap_or([false; 2]);
                            let (lf, ld) = match (&landing, solution.landing_shift) {
                                (Some(l), Some(d)) => (l.foot as i64, d),
                                (Some(l), None) => (l.foot as i64, na::Vector2::zeros()),
                                _ => (-1, na::Vector2::zeros()),
                            };
                            row += &format!(
                                ",{:.3},{:.3},{},{},{lf},{:.5},{:.5},{}",
                                by_side[0][5], by_side[1][5], on[0] as u8, on[1] as u8, ld.x, ld.y, solution.trajectory.len()
                            );
                            writeln!(f, "{row}").unwrap();
                        }
                        // (The lexicographic CoP's soft fallback and its
                        // MPC_LEX_DEBUG_* prints went with MPC_CAPTURE_LEX.)
                        if let Some(dp) = solution.landing_shift
                            && let Some(land) = landing
                            && !plan.landing.landing_closed(plan.gait.index_at(t))
                        {
                            let i = plan.gait.index_at(t);
                            if ctl.mpc_fs_slice != Some(i) {
                                ctl.mpc_fs_slice = Some(i);
                                ctl.mpc_fs_applied = na::Vector2::zeros();
                            }
                            let (d, total) = (dp - ctl.mpc_fs_applied, dp);
                            ctl.mpc_fs_last = total;
                            if debug.mpc_fs_landing {
                                println!(
                                    "  [fs-decision] t={t:.3} first_node={} prior=({:+.3},{:+.3}) dp=({:+.3},{:+.3}) total=({:+.3},{:+.3}) max={:?} min={:?} xi-ref=({:+.3},{:+.3})",
                                    land.first_node, land.prior[0], land.prior[1], dp.x, dp.y, total.x, total.y, land.max, land.min,
                                    xi.x - plan.dcm.reference(t).xi.x, xi.y - plan.dcm.reference(t).xi.y
                                );
                            }
                            if d.norm() > 1e-6 {
                                plan.footsteps.shift_next(i, land.foot, d);
                                ctl.mpc_fs_applied = total;
                                plan.rebuild_dcm();
                                stats.n_replan += 1;
                                if debug.replan { println!("  [diag-replan] site 6 t={t:.3} (line 9380)"); }
                            }
                        }
                        if debug.mpc_traj && t < 2.0 {
                            let xi0 = com.xy() + com_vel.xy() / omega;
                            let r0 = plan.dcm.reference(t);
                            print!("  [traj] t={t:.3} xi_meas=({:+.3},{:+.3}) xi_plan=({:+.3},{:+.3}) tgt=({:+.3},{:+.3}) |",
                                xi0.x, xi0.y, r0.xi.x, r0.xi.y, target_com.x, target_com.y);
                            for (k, [c, v]) in solution.trajectory.iter().enumerate() {
                                let xi = c.xy() + v.xy() / omega;
                                let rp = plan.dcm.reference(t + (k as f64 + 1.0) * stair_wrench_preview_dt);
                                print!(" k{k}: pred({:+.3},{:+.3}) plan({:+.3},{:+.3})", xi.x, xi.y, rp.xi.x, rp.xi.y);
                            }
                            println!();
                        }
                        stats.wrench_mpc_solves += 1;
                    }
                    None => {
                        stats.wrench_mpc_failures += 1;
                    }
                }
                ctl.wrench_mpc_next = t + stair_wrench_dt;
            }
        } else {
            ctl.wrench_mpc_command = None;
            ctl.wrench_mpc_next = t;
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
        let yaw_plan = {
            let mean = |i: usize| {
                let y = plan.footsteps.at_slice(i).yaw;
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
                deadband: 0.0,
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
            && wrench_mpc_apply
            && ctl.wrench_mpc_command.is_some()
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
            // Zero references (MOM_REF_MPC, the MPC's planned rate as the
            // reference, was not adopted and is removed); the arms-only DCM
            // ask below is the one rate this task is given.
            let h_ref = [0.0_f64; 3];
            let mut dh_ref = [0.0_f64; 3];
            if arms_neutral && mom_arm_dcm != 0.0 {
                dh_ref[0] = (-mom_arm_dcm * (xi.y - r.xi.y))
                    .clamp(-mom_arm_dcm_max, mom_arm_dcm_max);
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
            && !in_stair_zone
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
            stats.n_mom_cmp_ticks += 1;
            stats.mom_cmp_peak = stats.mom_cmp_peak.max(dh[0].abs()).max(dh[1].abs());
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
        let mut p_arm = if arm_hold {
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
        if let Some(command) = ctl.wrench_mpc_command.as_ref() {
            if wrench_mpc_apply && nc > 0 {
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
                        let authority = mpc_sole_authority && nc == 2;
                        let w = if authority { 1.0 } else { stair_wrench_blend };
                        let old = forces_nominal.rows(6 * slot, 6).into_owned();
                        let blended = old * (1.0 - w) + native * w;
                        forces_nominal.rows_mut(6 * slot, 6).copy_from(&blended);
                    }
                }
            }
        }
        let p_wrench_coupled = coupled_active.then(|| {
            tasks::regularize(&dyn_ctx.forces(), &forces_nominal)
                // 1.0: the default of STAIR_WRENCH_LEVEL_WEIGHT (removed).
                .weight(1.0 / (total_mass * G).powi(2))
        });
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
        let mut swing_tgt = na::Vector3::zeros();
        let mut swing_cmd_acc_z = 0.0_f64;
        let p_swing = match (free_side, ctl.swing_lift_off) {
            (None, _) => None,
            // The scheduled swing, following its planned arc.
            (Some(side), Some(lift_off))
                if support.swing() == Some(side) =>
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
                let plan_xy = plan.footsteps.at_slice(slice_idx).sole[side];
                // On stairs the surfaces differ in height; carry the CHANGE
                // in surface height on top of the measured lift-off z, so a
                // foot that started a few millimetres high keeps its offset
                // instead of being driven into the tread. Flat ground makes
                // dz zero and this is exactly the old `lift_off.z`.
                // The plan's own height change decides "descending" for the
                // descent-only absolute target (the measured-lift-off form
                // of dz is what it replaces, so it cannot be read first).
                let plan_dz_sign = stairs
                    .as_ref()
                    .map_or(0.0, |st| st.h(plan_xy.x) - st.h(lift_off.x));
                let dz = if (stair_absolute_td || (stair_absolute_td_desc && plan_dz_sign < -1e-9)) && stairs.is_some() {
                    plan_xy.z - plan.footsteps.at_slice(slice_idx.saturating_sub(1)).sole[side].z
                } else {
                    stairs
                        .as_ref()
                        .map_or(0.0, |st| st.h(plan_xy.x) - st.h(lift_off.x))
                };
                let touch_down = if (stair_absolute_td || (stair_absolute_td_desc && dz < -1e-9)) && dz.abs() > 1e-9 {
                    // The target is a surveyed tread, not the measured height
                    // of the previous (possibly rocking) foot plus a riser.
                    let offset = foot_rot_at(side, plan.footsteps.at_slice(slice_idx).yaw[side])
                        * na::Vector3::new(prof.sole_centre_x, 0.0, -prof.sole_below_origin);
                    plan_xy - offset
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
                    let mut td = na::Vector3::new(plan_xy.x, plan_xy.y, lift_off.z + dz);
                    if stair_td_sole_frame {
                        let off = foot_rot_at(side, plan.footsteps.at_slice(slice_idx).yaw[side])
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
                let (swing_fraction, swing_duration) = if com_replan {
                    plan.landing
                        .clock
                        .filter(|(i, _, _)| *i == slice_idx)
                        .map(|(_, start, duration)| {
                            (((t - start) / duration).clamp(0.0, 1.0), duration)
                        })
                        .unwrap_or((slice_frac, slice.duration()))
                } else {
                    (slice_frac, slice.duration())
                };
                let (mut tgt, mut tgt_v, tgt_a) =
                    if dz.abs() > step_dz_swing_min && stair_swing_ff {
                        articara::biped::gait::stair_swing_sink(
                            lift_off,
                            touch_down,
                            lift_h,
                            swing_fraction,
                            swing_duration,
                            stair_swing_windows,
                            stair_td_sink,
                        )
                    } else if dz.abs() > 1e-9 {
                        let f = swing_fraction;
                        let dur = swing_duration;
                        let ss = |u: f64| -> (f64, f64) {
                            let u = u.clamp(0.0, 1.0);
                            (u * u * (3.0 - 2.0 * u), 6.0 * u * (1.0 - u))
                        };
                        let z_max = lift_off.z.max(touch_down.z) + lift_h;
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
                            if tracking_cubic_ff && tracking_ff_riser_ok {
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
                        let mut pos0 = swing_position(
                            lift_off,
                            touch_down,
                            lift_h,
                            swing_fraction,
                        );
                        let mut vel0 = swing_velocity(
                            lift_off,
                            touch_down,
                            lift_h,
                            swing_fraction,
                            swing_duration,
                        );
                        // Level swings only: a swing that changes height
                        // (the shipped arc interpolates z linearly under its
                        // bump) would hold the apex and drop the whole change
                        // in the last 35% -- measured 2026-10-15, the 100 mm
                        // descent stomped at 1376 N and fell on every member.
                        // ... and only away from known stairs: inside the
                        // stair zone the validated stair machinery (landing
                        // clocks, descent lead) owns the swing; with the hold
                        // active there the 100 mm descent started its swing
                        // 20 cm ahead of the foot and fell on every member.
                        // ... and only for a swing on its NOMINAL clock: a
                        // push retimes the swing shorter, and the held apex
                        // then has to come down in the last 35% of a short
                        // swing. Measured 2026-10-16 on the 24 worst backward
                        // and lateral push members: hold unconditional 17 -> 11,
                        // start delay alone 17 -> 17 (SWING_Z_HOLD_MIN_DUR,
                        // fraction of the nominal single support; 0 = always).
                        if swing_z_hold
                            && !in_stair_zone
                            && (touch_down.z - lift_off.z).abs() < 0.02
                            && swing_duration >= swing_z_hold_min_dur * gp.t_ss
                        {
                            // SWING_Z_HOLD: the three-phase z of
                            // STAIR_SWING_PROFILE (rise [0,0.30] to the apex,
                            // hold, drop [0.60,0.95]) with the SHIPPED xy.
                            // Measured 2026-10-15 on the unmodelled-bump bench
                            // (joint-sum foot pitch): the swing foot hangs
                            // 20 deg toe-down, so with the sin^2 lift the toe
                            // is ~1 cm up when it reaches a 16 mm edge; the
                            // three-phase lift is at 5 cm before x moves and
                            // clears. Its compressed xy is what cost the push
                            // family (326 -> 235), so only z is taken.
                            let f = swing_fraction;
                            let dur = swing_duration.max(1e-6);
                            let ss = |u: f64| -> (f64, f64) {
                                let u = u.clamp(0.0, 1.0);
                                (u * u * (3.0 - 2.0 * u), 6.0 * u * (1.0 - u))
                            };
                            let z_max = lift_off.z.max(touch_down.z) + lift_h;
                            // SWING_Z_DROP="d0,d1": the drop window (default
                            // 0.60..0.95 as in the three-phase profile; 1.0
                            // lands on the slice end like the shipped arc).
                            let (d0, d1) = (swing_z_drop[0], swing_z_drop[1].max(swing_z_drop[0] + 0.05));
                            let (ru, dru) = ss(f / 0.30);
                            let (du, ddu) = ss((f - d0) / (d1 - d0));
                            pos0.z = lift_off.z + (z_max - lift_off.z) * ru + (touch_down.z - z_max) * du;
                            vel0.z = (z_max - lift_off.z) * (dru / 0.30 / dur)
                                + (touch_down.z - z_max) * (ddu / (d1 - d0) / dur);
                        }
                        if swing_xy_delay > 0.0 {
                            // (SWING_XY_END, ending the horizontal motion
                            // before 1, is removed.)
                            // SWING_XY_DELAY: hold the horizontal motion for
                            // the first `delay` of the swing while the shipped
                            // sin^2 lift runs, then smoothstep the xy over the
                            // rest. The unmodelled-bump bench measured the
                            // toe catching a 15 mm edge as soon as the swing
                            // starts (x advances before z has risen); the
                            // three-phase STAIR_SWING_PROFILE fixes that but
                            // costs the push family 326 -> 235 (2026-10-15).
                            // This keeps the lift and landing timing as
                            // shipped. 0 = off (bit-identical).
                            let d = swing_xy_delay;
                            let e = 1.0_f64.max(d + 0.05);
                            let u = ((swing_fraction - d) / (e - d)).clamp(0.0, 1.0);
                            let s_ = (3.0 - 2.0 * u) * u * u;
                            let ds = 6.0 * u * (1.0 - u) / ((e - d) * swing_duration.max(1e-6));
                            let dxy = touch_down.xy() - lift_off.xy();
                            pos0.x = lift_off.x + dxy.x * s_;
                            pos0.y = lift_off.y + dxy.y * s_;
                            vel0.x = dxy.x * ds;
                            vel0.y = dxy.y * ds;
                        }
                        (pos0, vel0, na::Vector3::zeros())
                    };
                let replan_hold = push_replan_trig > 0.0
                    && ctl.push_replan_slice == Some(slice_idx)
                    && ctl.push_replan_shift > push_replan_hold;
                // 0.05: the default of PUSH_REPLAN_HOLD_Z (removed; the
                // champion turns the hold off with PUSH_REPLAN_HOLD=10).
                let hold_z = if replan_hold { 0.05 } else { mpc_footstep_cross_hold_z };
                if hold_z > 0.0
                    && (replan_hold || ctl.mpc_fs_cross_latched.is_some_and(|(ci, _)| ci == slice_idx))
                {
                    let foot = misarta::se3::translation(&data.oMi[rig.foot_mi[side]]);
                    if (touch_down.xy() - foot.xy()).norm() > mpc_footstep_cross_hold_xy {
                        let hold = touch_down.z + hold_z;
                        if tgt.z < hold {
                            tgt.z = hold;
                            tgt_v.z = tgt_v.z.max(0.0);
                        }
                    }
                }
                if let Some(mode) = ctl.ref_mode
                    && let Some(lib) = &ctl.ref_lib
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
                    let offset = foot_rot_at(side, plan.footsteps.at_slice(slice_idx).yaw[side])
                        * na::Vector3::new(prof.sole_centre_x, 0.0, -prof.sole_below_origin);
                    tgt = sole_w - offset;
                    tgt_v = (sole_w1 - sole_w) / dt;
                }
                if plan.landing.probe_depth > 0.0
                    && plan.landing.clock.is_some_and(|(ci, _, _)| ci == slice_idx)
                {
                    tgt.z -= plan.landing.probe_depth;
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
                    // (STAIR_RISE_SOLE_POSE / _MIN_DZ, roll and pitch held
                    // through a height-changing swing, were opt-in -- the
                    // champion sets 0 -- and are removed.)
                    if terrain_pitch
                        && stairs.as_ref().is_some_and(|st| st.pitch_at(plan.footsteps.at_slice(slice_idx).sole[side].x) != 0.0)
                    {
                        // TERRAIN_PITCH_YAW=1: keep the walking default's yaw
                        // and add pitch only, roll free (see SwingAxes::XyzYawPitch).
                        if terrain_pitch_yaw { bt::SwingAxes::XyzYawPitch } else { bt::SwingAxes::XyzRollPitch }
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
                            foot_rot_at(side, plan.footsteps.at_slice(slice_idx).yaw[side]);
                        if terrain_pitch && let Some(st) = &stairs {
                            let th = st.pitch_at(plan.footsteps.at_slice(slice_idx).sole[side].x);
                            if th != 0.0 {
                                // Toe up by the slope angle: rotate about world y by -th.
                                rot_tgt = na::Rotation3::from_axis_angle(&na::Vector3::y_axis(), -th).matrix() * rot_tgt;
                            }
                        }
                        Some((
                            misarta::se3::rotation_matrix(&data.oMi[mi]),
                            rot_tgt,
                            na::Vector3::new(om3[0], om3[1], om3[2]),
                            kp_sw_rot,
                            // (LAND_KD_ROT_BOOST, a multiplier through a stair
                            // swing's drop phase, defaulted to 1 and is removed.)
                            kd_sw_rot,
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
                    match swing_rot_mode {
                        0 => bt::SwingAxes::Xyz,
                        2 => bt::SwingAxes::Pose,
                        3 => bt::SwingAxes::XyzRollPitch,
                        _ => bt::SwingAxes::XyzYaw,
                    },
                    Some((
                        misarta::se3::rotation_matrix(&data.oMi[mi]),
                        foot_rot_at(side, plan.footsteps.at_slice(slice_idx).yaw[side]),
                        na::Vector3::new(om3[0], om3[1], om3[2]),
                        kp_sw_rot,
                        kd_sw_rot,
                    )),
                ))
            }
        };

        // Keep the unweighted task for post-solve diagnostics; `p_swing`
        // itself is moved into the hierarchy below.
        let p_swing_diag = p_swing.clone();
        // Preserve original physical rows before task weighting / HQP slack.
        let qp_physical =
            (diag.qp_diag_dir.is_some() || qp_gate || diag.qp_runtime_log.is_some()).then(|| {
                let mut units = vec!["N*m"; 2 * na_count];
                let mut scales = vec![(total_mass * G * sole_half_l).max(1.0); 2 * na_count];
                if jlim {
                    units.extend(vec!["rad/s^2"; 2 * na_count]);
                    scales.extend(vec![1.0; 2 * na_count]);
                }
                for _ in &stance_sides {
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
        levels.push(p_reg);
        level_names.push("regularise");
        // Weighted single QP (WBC_STRATEGY=weighted): each level below the
        // hard physics level gets a PER-TASK weight, because a geometric
        // ratio cannot express "swing as strong as the CoM, regularisation
        // far weaker" (measured 2026-09-24: ratios 1..1000 all fell in the
        // first step or before it). Override one with WBCW_<NAME> where NAME
        // is the level name upper-cased with non-alphanumerics as '_'
        // (e.g. WBCW_COM, WBCW_SWING, WBCW_POSTURE_MOMENTUM).
        // (WBC_WEIGHTED_OFF_STAIRS, the strict cascade inside the stair
        // zone, was not adopted: weighted everywhere = 112/112 on the flat
        // push bench but 0/192 on the stairs, measured 2026-09-24; removed.)
        if weighted_levels {
            for (k, lvl) in levels.iter_mut().enumerate().skip(1) {
                let name = level_names.get(k).copied().unwrap_or("");
                let wk = level_weights.get(name);
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
        let qp_reset = wbc_state.qp_previous_structure
            .as_ref()
            .is_some_and(|old| old != &qp_structure);
        // The stateful solver keeps a workspace per level and only resets it
        // when the QP STRUCTURE changes. (WBC_QP_RESET_EVERY_TICK, dropping
        // the warm start every tick, was an experiment and is removed.)
        if qp_reset {
            wbc_state.solver.reset();
        }
        wbc_state.qp_previous_structure = Some(qp_structure);
        let qp_solve_start = std::time::Instant::now();
        let mut sol = wbc_state.solver
            .solve(&levels, &cfg)
            .unwrap_or_else(|e| panic!("wbc solve failed at t={t:.3}: {e}"));
        let qp_solve_ns = qp_solve_start.elapsed().as_nanos();
        // WBC_LEVEL_DEBUG_T=<t>: print each level's least-squares residual
        // at the ticks within 11 ms of t (which task the solve could not
        // satisfy, and by how much).
        if let Some(tw) = debug.wbc_level_t
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
        // WBC_REPAIR_ORIGINAL / WBC_PREFIX_MIN_LEVEL (repair the failed QP,
        // or command a checked prefix) were experiments and are removed;
        // QP_RUNTIME_CSV keeps their columns at the off values.
        let repair_applied = false;
        let repair_scaled_delta = 0.0_f64;
        let repair_ns = 0u128;
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
        {
            let qdd = dyn_ctx.qddot().eval(&sol.x);
            let jr = j_trunk.rows(0, 2).into_owned();
            let acc = &jr * &qdd + djv_trunk.rows(0, 2).into_owned();
            trunk_cmd_rp = rp_ref;
            trunk_acc_rp = [acc[0], acc[1]];
        }
        let degraded = !matches!(sol.status, misa_wbc::SolveStatus::Optimal);
        let qp_capture_timer = std::time::Instant::now();
        if let Some(dir) = &diag.qp_diag_dir {
            let legacy_failure = t >= 2.0
                && diag.qp_diag_count < 8
                && matches!(sol.status,
                misa_wbc::SolveStatus::Degraded { level, .. } if level_names.get(level) == Some(&"swing"));
            let continuous = diag.qp_capture_all
                && t >= diag.qp_capture_start
                && t <= diag.qp_capture_end
                && tick % diag.qp_capture_stride == 0
                && diag.qp_diag_count < diag.qp_capture_max;
            if continuous || (!diag.qp_capture_all && legacy_failure) {
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
                    "prox_weight":cfg.prox_weight,"tasks":tasks,"solution":sol.x.as_slice(),"variables":variables,
                    "contact_ids":stance_sides,"planned_support":format!("{:?}",support),"phase":"None",
                    "measured_fz_N":measured_pre,"physical_groups":physical,"solve_ns":qp_solve_ns,
                    "check_ns":qp_check_ns,"normalized_physical_violation":qp_physical_violation});
                std::fs::create_dir_all(dir).expect("QP_DIAG_DIR");
                std::fs::write(
                    format!("{dir}/tick_{:06}.json", diag.qp_diag_count),
                    serde_json::to_vec(&dump).unwrap(),
                )
                .expect("write QP snapshot");
                diag.qp_diag_count += 1;
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
        let mut cop_use = 0.0_f64;
        for (slot, mi) in stance.iter().copied().enumerate() {
            let side = rig.side_of(mi);
            let w = &p0.sole_sel[slot] * &extracted.forces;
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
        if let Some(file) = diag.transfer_diag.as_mut() {
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
            let mpc_valid = ctl.wrench_mpc_command.is_some();
            let values = [
                t,
                dt,
                bw,
                // phase / high / alpha of the removed stair transfer (idle).
                0.0,
                -1.0,
                0.0,
                mask as f64,
                measured_mask as f64,
                f64::from(mpc_valid),
                if mpc_valid {
                    diag_mpc_time.map_or(f64::NAN, |ts| t - ts)
                } else {
                    f64::NAN
                },
                stats.wrench_mpc_failures as f64,
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
                ctl.wrench_mpc_command.as_ref().map_or(f64::NAN, |w| w[0][5]),
                ctl.wrench_mpc_command.as_ref().map_or(f64::NAN, |w| w[1][5]),
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
                0.0, // ffdc_dz: the removed FFDC loops left it at 0
            ];
            for (i, value) in values.iter().enumerate() {
                if i > 0 {
                    write!(file, ",").unwrap();
                }
                write!(file, "{value:.9}").unwrap();
            }
            writeln!(file).unwrap();
        }
        m.max_cop_use = m.max_cop_use.max(cop_use);
        if m.push_at_t.is_some() {
            m.push_cop_max = m.push_cop_max.max(cop_use);
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
            if m.transfer_t.is_none() {
                if let (Some(Support::Double), Support::Single { .. }) = (m.support_prev, support) {
                    m.transfer_t = Some(t);
                    if fix.armed() && !m.fix_fired {
                        fix.fire(&mut rig.sim, &push.link, total_mass, t, true);
                        m.fix_fired = true;
                    }
                    // 6-D centroidal momentum h = A_G * v; rows 0..3 are
                    // angular, and row 0 is the roll axis.
                    let ag =
                        misarta::centroidal::compute_centroidal_momentum_matrix(&rig.model, &st.q);
                    m.h_roll_at_xfer = (&ag * v_dvec)[0];
                    if let Support::Single { stance, .. } = support {
                        let u = steps.xy(stance);
                        m.xfer_dcm_off = [xi.x - u.x, xi.y - u.y];
                        m.xfer_stance = stance as i32;
                    }
                }
            } else if m.transfer2_t.is_none() && t > m.transfer_t.unwrap() + 1e-9 {
                if let (Some(Support::Double), Support::Single { .. }) = (m.support_prev, support) {
                    m.transfer2_t = Some(t);
                }
            }
            // Time-triggered corrective impulse (see `fix_at_s`).
            if fix.at_s >= 0.0
                && !m.fix_fired
                && fix.armed()
                && m.push_at_t.map(|tp0| t - tp0 >= fix.at_s).unwrap_or(false)
            {
                fix.fire(&mut rig.sim, &push.link, total_mass, t, false);
                m.fix_fired = true;
            }
            if m.push_at_t.map(|tp0| t - tp0 <= 1.0).unwrap_or(false) {
                let ag = misarta::centroidal::compute_centroidal_momentum_matrix(&rig.model, &st.q);
                m.h_roll_max = m.h_roll_max.max((&ag * v_dvec)[0].abs());
            }
            if let Some(t2) = m.transfer2_t {
                if t - t2 <= transfer_s {
                    m.xfer2_ticks += 1;
                    let on_edge2 = (0..2).any(|side| {
                        measured.cop[side][2] > 0.1 * total_mass * G
                            && measured.cop[side][1].abs() >= edge_frac * sole_half_w
                    });
                    if on_edge2 {
                        m.xfer2_edge_ticks += 1;
                    }
                }
            }
            if let Some(t0) = m.transfer_t {
                if t - t0 <= transfer_s {
                    m.xfer_ticks += 1;
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
                            m.xfer_wrongfoot_ticks += 1;
                        }
                    }
                    if on_edge {
                        m.xfer_edge_ticks += 1;
                    } else if m.xfer_leave_t.is_none() {
                        m.xfer_leave_t = Some(t);
                    }
                }
            }
            let fz_total = measured.f_w[0][2] + measured.f_w[1][2];
            m.push_fz_total_min = m.push_fz_total_min.min(fz_total);
            m.push_fz_total_max = m.push_fz_total_max.max(fz_total);
            if fz_total < 0.5 * total_mass * G {
                m.push_unloaded_ticks += 1;
            }
            if in_push_window {
                m.win_cop_max = m.win_cop_max.max(cop_use);
                m.win_fz_total_min = m.win_fz_total_min.min(fz_total);
                m.win_fz_total_max = m.win_fz_total_max.max(fz_total);
                if fz_total < 0.5 * total_mass * G {
                    m.win_unloaded_ticks += 1;
                }
            }
            for side in 0..2 {
                let fz = measured.f_w[side][2];
                if fz <= 1.0 {
                    continue; // an unloaded foot has no meaningful ratio
                }
                let tan = (measured.f_w[side][0].powi(2) + measured.f_w[side][1].powi(2)).sqrt();
                m.push_slip_wbc_max = m.push_slip_wbc_max.max(tan / (friction_mu * fz));
                m.push_slip_plant_max = m.push_slip_plant_max.max(tan / (mu_ground * fz));
                if in_push_window {
                    m.win_slip_plant_max = m.win_slip_plant_max.max(tan / (mu_ground * fz));
                    if tan >= 0.98 * mu_ground * fz {
                        m.win_slip_ticks += 1;
                    }
                }
                // AT the cone, not past it: MuJoCo's contact solver never
                // returns a force outside the cone, so `tan > mu*fz` is only
                // ever true by floating-point luck and counted 0 ticks on a
                // run that slid for 150 ms. Riding the boundary IS sliding.
                if tan >= 0.98 * mu_ground * fz {
                    m.push_slip_ticks += 1;
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
            m.q_lo[ji] = m.q_lo[ji].min(qj);
            m.q_hi[ji] = m.q_hi[ji].max(qj);
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
            m.hit_f_now = f_hit;
            if f_hit > 0.0 {
                m.hit_ticks += 1;
                m.hit_peak_f = m.hit_peak_f.max(f_hit);
                m.hit_impulse += f_hit * dt;
                m.hit_first_t.get_or_insert(t);
                m.hit_last_t = t;
                m.ep_t0.get_or_insert(t);
                m.ep_imp += f_hit * dt;
            } else if let Some(t0) = m.ep_t0.take() {
                m.hit_episodes.push((t - t0, m.ep_imp));
                m.ep_imp = 0.0;
            }
            let hits: Vec<(String, String, f64)> = all
                .into_iter()
                .filter(|(a, b, _)| !is_target(a, b) && !is_stairs(a, b))
                .collect();
            if !hits.is_empty() {
                m.n_selfcollide += 1;
                if m.n_selfcollide <= 8 {
                    let d: Vec<String> = hits
                        .iter()
                        .map(|(a, b, f)| format!("{a} <-> {b} ({f:.1} N)"))
                        .collect();
                    println!("  [self-collision] t={t:6.3}  {}", d.join(", "));
                }
                m.max_selfcollide_f =
                    m.max_selfcollide_f.max(hits.iter().map(|(_, _, f)| *f).fold(0.0, f64::max));
            }
        }

        let mut robot_taus = vec![0.0_f64; rig.robot.joints.len()];
        for (ji, vi) in rig.actuated() {
            robot_taus[ji] = extracted.tau[vi - 6];
        }
        // The checked-prefix path (WBC_PREFIX_MIN_LEVEL) is removed.
        let prefix_applied = false;
        {
            let rig_ref = &rig;
            let tg = &tau_gravity;
            let fallback =
                move |out: &mut [f64]| gravity_plus_posture(rig_ref, tg, v, hold_kp, hold_kd, out);
            guard.policy.apply(&sol.status, &mut robot_taus, t, dt, nc, &fallback);
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
                m.max_ankle_roll_use = m.max_ankle_roll_use.max(use_i);
                if m.push_at_t.is_some() {
                    m.push_ankle_max = m.push_ankle_max.max(use_i);
                }
            }
        }

        // Per-step accounting, keyed on the single-support phases.
        if matches!(support, Support::Single { .. }) {
            let stance_side = match support {
                Support::Single { stance, .. } => stance,
                Support::Double => unreachable!(),
            };
            if m.step_stats.last().map(|s| s.slice) != Some(slice_idx) {
                m.step_stats.push(StepStat {
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
            let s = m.step_stats.last_mut().unwrap();
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

        // ---- disturbance: fire once, at the requested phase (`fire_push`).
        if m.push_at_t.is_none()
            && fire_push(
                &mut rig.sim,
                &push,
                &PushTick { t, step_idx, support, slice_frac, slice_t1: slice.t1 },
                total_mass,
                omega,
                jab.then_some(JabClock { start: jab_start, period: jab_period, pattern: &jab_pattern }),
            )
        {
            m.push_at_t = Some(t);
            m.push_at_step = step_idx;
            m.push_degraded_at = tally.n;
        }
        if let Some(mode) = ctl.ref_mode
            && let Some(lib) = &ctl.ref_lib
            && t >= mode.t0 + lib.total() - mode.tau0
        {
            ctl.ref_mode = None;
            ctl.ref_cooldown_until = t + ref_cooldown;
            plan.landing.event_mode = event_mode_cfg;
            // The reference leaves the feet where the recovery put them
            // (in tandem, narrow). Every later footfall is re-centred on the
            // feet as they are now, at the nominal stance width, so the gait
            // that resumes is the nominal one and not a stride-0 echo of the
            // recovery (measured: the echo fell 1.7 s after the reference).
            if ref_reset_stance {
                let i = plan.gait.index_at(t);
                let cur = *plan.footsteps.at_slice(i);
                let mid = 0.5 * (cur.sole[0].xy() + cur.sole[1].xy());
                let half_w = 0.5 * (plan.footsteps.initial.sole[0].y - plan.footsteps.initial.sole[1].y).abs();
                let j0 = if ref_reset_incl_current { i } else { i + 1 };
                for side in 0..2 {
                    let want = na::Vector2::new(mid.x, mid.y + if side == 0 { half_w } else { -half_w });
                    if let Some(j) = (j0..plan.gait.slices.len()).find(|&j| plan.gait.slices[j].support.swing() == Some(side)) {
                        if j == i {
                            println!("  [ref] t={t:.3} re-centre the swing under way: foot {side} -> ({:.3},{:.3})", want.x, want.y);
                        }
                        let d = want - plan.footsteps.at_slice(j).sole[side].xy();
                        plan.footsteps.shift_from(j, side, d);
                    }
                }
                plan.rebuild_dcm();
            }
            println!("  [ref] t={t:.3} reference over, back to the gait");
        }

        m.support_prev = Some(support);
        // What the disturbance is doing RIGHT NOW, for the replay to draw.
        // Read from the sim's own pulse list rather than recomputed here, so
        // the arrow cannot disagree with the force the plant actually saw.
        let push_now = rig
            .sim
            .external_force_pulses()
            .iter()
            .find(|p| p.link_name == push.link)
            .map(|p| [p.force[0], p.force[1]])
            .unwrap_or([0.0, 0.0]);
        let audit_pre = diag.tick_audit.as_ref().map(|audit| audit.sample(&rig));
        if let Some(file) = &mut diag.qp_runtime_log {
            use std::io::Write;
            let control_ns = qp_control_start
                .elapsed()
                .as_nanos()
                .saturating_sub(qp_capture_ns)
                .saturating_sub(mpc_capture_ns);
            writeln!(file, "{t:.9},{qp_solve_ns},{qp_check_ns},\"{:?}\",{qp_physical_violation:.12e},{qp_reset},{},{control_ns},{},{wrench_mpc_solves},{wrench_mpc_failures},{prefix_applied},{repair_applied},{repair_scaled_delta:.12e},{repair_ns}", sol.status,
                control_ns > (dt * 1e9) as u128, diag_mpc_time.map_or(-1.0, |stamp| t-stamp),
                wrench_mpc_solves = stats.wrench_mpc_solves, wrench_mpc_failures = stats.wrench_mpc_failures).unwrap();
        }
        if let Some(file) = &mut diag.qp_wrench_log {
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
                    if let (Some(command), Some(frames)) = (&ctl.wrench_mpc_command, &wrench_mpc_frames)
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
        // ACT_DELAY_TICKS=n: the plant receives the torques commanded n
        // control ticks ago (actuation latency, model-mismatch bench).
        // 0 = off = this tick's torques.
        if act_delay_ticks > 0 {
            act_delay_buf.push_back(robot_taus.clone());
            while act_delay_buf.len() > act_delay_ticks + 1 {
                act_delay_buf.pop_front();
            }
            let delayed = act_delay_buf.front().cloned().unwrap_or_else(|| robot_taus.clone());
            write_to_plant(&mut rig, ctrl_mode, &delayed, &extracted.qddot, v, dt);
        } else {
            write_to_plant(&mut rig, ctrl_mode, &robot_taus, &extracted.qddot, v, dt);
        }
        rig.step(mj_substeps);
        if diag.substep_audit_path.is_some() {
            let tail = rig.sim.trace_len().saturating_sub(mj_substeps as usize);
            for frame in rig.sim.trace().skip(tail) {
                diag.substep_samples += 1;
                for (ji, joint) in rig.robot.joints.iter().enumerate() {
                    if joint.joint_type != "revolute" { continue; }
                    let (q, qd, tau) = (frame.q[ji], frame.qvel[ji], frame.tau[ji]);
                    if !q.is_finite() || !qd.is_finite() || !tau.is_finite() {
                        diag.substep_nonfinite += 1;
                        continue;
                    }
                    let extrema = &mut diag.substep_extrema[ji];
                    extrema[0] = extrema[0].min(q);
                    extrema[1] = extrema[1].max(q);
                    extrema[2] = extrema[2].max(qd.abs());
                    extrema[3] = extrema[3].max(tau.abs());
                    let excess = &mut diag.substep_max_excess[ji];
                    excess[0] = excess[0].max((joint.lower - q).max(q - joint.upper).max(0.0));
                    excess[1] = excess[1].max((qd.abs() - joint.velocity).max(0.0));
                    excess[2] = excess[2].max((tau.abs() - joint.effort).max(0.0));
                }
            }
        }
        if let (Some(audit), Some(pre)) = (diag.tick_audit.as_mut(), audit_pre.as_ref()) {
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

        if let (Some(file), Some((mi, vi, ji, mjvi))) = (diag.ankle_cbf_diag.as_mut(), ankle_cbf_indices)
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

        if diag.traj.is_enabled() {
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
            let anchor_l_y = wbc_state.anchors.a[0].map_or(f64::NAN, |(p, _)| p.y);
            let anchor_r_y = wbc_state.anchors.a[1].map_or(f64::NAN, |(p, _)| p.y);
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
                m.hit_f_now,
                m.ep_imp,
                m.hit_impulse,
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
                // contact_activation_l/r, contact_force_activation_l/r,
                // load_chatter_ema: written only by the removed stair
                // transfer; these are the values it left them at.
                1.0,
                1.0,
                1.0,
                1.0,
                0.0,
                gl.x,
                gl.y,
                gl.z,
                gr.x,
                gr.y,
                gr.z,
                if ctl.replan_engaged { 1.0 } else { 0.0 },
                ctl.preview_c.x,
                ctl.preview_c.y,
                ctl.preview_v.x,
                ctl.preview_v.y,
                stats.preview_error,
                stats.preview_count as f64,
                stats.preview_failures as f64,
                stats.landing_delay,
                // stair_transfer / _phase / _alpha / _high: the removed
                // three-phase transfer's columns, kept at its idle values so
                // the column layout does not move.
                0.0,
                0.0,
                0.0,
                -1.0,
                0.0, // ffdc_dz (removed DS_FFDC / STAIR_FFDC)
                0.0, // ffdc_err (removed DS_FFDC / STAIR_FFDC)
                -1.0, // capture_error (removed stair transfer)
                0.0,  // capture_tilt (removed stair transfer)
                trunk_tilt_roll,
                trunk_tilt_pitch,
                trunk_cmd_rp[1],
                trunk_acc_rp[1],
                trunk_cmd_rp[0],
                trunk_acc_rp[0],
                -1.0, // transfer_guard (removed stair transfer)
                if ctl.wrench_mpc_command.is_some() {
                    1.0
                } else {
                    0.0
                },
                ctl.wrench_mpc_command.as_ref().map_or(0.0, |w| w[0][5]),
                ctl.wrench_mpc_command.as_ref().map_or(0.0, |w| w[1][5]),
                diag_mpc_net_cop[0],
                diag_mpc_net_cop[1],
                ctl.mpc_fs_last.x,
                ctl.mpc_fs_last.y,
                diag_wbc_net_cop[0],
                diag_wbc_net_cop[1],
                stats.wrench_mpc_solves as f64,
                stats.wrench_mpc_failures as f64,
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
            if let Some(file) = &mut diag.mom_diag_log {
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
                let i = plan.gait.index_at(t);
                let sl = &plan.gait.slices[i];
                println!(
                    "  [gait] t={t:.3} slice {i} {:?} [{:.3},{:.3}] stance={:?} fz=({:.0},{:.0}) xi.x={:.3} ref.x={:.3}",
                    sl.support, sl.t0, sl.t1, stance_sides, fz_meas[0], fz_meas[1], xi.x, r.xi.x
                );
            }
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
            diag.traj.write(&rig, &row);
        }

        if !m.fell {
            let p = rig.sim.body_world_position(&rig.robot.root_link).unwrap();
            let y = rig
                .sim
                .body_world_orientation(&rig.robot.root_link)
                .unwrap()
                .euler_angles()
                .2;
            if let Some((pp, py)) = m.prev_pose {
                if t >= t_ramp_end {
                    let (dx, dy) = (p[0] - pp[0], p[1] - pp[1]);
                    let (c, sn) = (py.cos(), py.sin());
                    m.travel[0] += dx * c + dy * sn;
                    m.travel[1] += -dx * sn + dy * c;
                    // Shortest-arc difference, so the accumulator never wraps.
                    let mut dy_aw = y - py;
                    while dy_aw > std::f64::consts::PI {
                        dy_aw -= std::f64::consts::TAU;
                    }
                    while dy_aw < -std::f64::consts::PI {
                        dy_aw += std::f64::consts::TAU;
                    }
                    m.travel[2] += dy_aw;
                }
            }
            m.prev_pose = Some((p, y));
            m.last_state = (p, y, t);
        }
        if m.ramp_end_state.is_none() && t >= t_ramp_end {
            let p = rig.sim.body_world_position(&rig.robot.root_link).unwrap();
            let y = rig
                .sim
                .body_world_orientation(&rig.robot.root_link)
                .unwrap()
                .euler_angles()
                .2;
            m.ramp_end_state = Some((p, y, t));
        }
        if m.walk_end_state.is_none() && t >= t_walk_end {
            let p = rig.sim.body_world_position(&rig.robot.root_link).unwrap();
            let y = rig
                .sim
                .body_world_orientation(&rig.robot.root_link)
                .unwrap()
                .euler_angles()
                .2;
            m.walk_end_state = Some((p, y, t));
        }
        let cur_z = rig.sim.body_world_position(&rig.robot.root_link).unwrap()[2];
        m.min_z = m.min_z.min(cur_z);
        let tilt = roll.abs().max(pitch.abs());
        m.max_tilt = m.max_tilt.max(tilt);
        if m.push_at_t.is_some() {
            m.push_tilt_max = m.push_tilt_max.max(tilt);
        }
        if in_push_window {
            m.win_tilt_max = m.win_tilt_max.max(tilt);
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
        let floor_z = stairs.as_ref().map_or(0.0, |st| st.h(com.x));
        if cur_z - floor_z < 0.30 || tilt > 0.52 {
            println!("  FELL at t={t:.3} (z={cur_z:.3}, floor={floor_z:.3}, tilt={tilt:.3})");
            m.fell = true;
            break;
        }
    }
    let WalkerStats { n_mom_cmp_ticks, mom_cmp_peak, n_push_z_drop, n_cp_shift, n_cp_retime, cp_retime_total, n_confirmed_admissions, ref_engagements, n_ds_capture, ds_capture_total, n_ds_feasible, n_ds_continuous, ds_feasible_total, n_push_replan, n_mpc_fs_cross, preview_count, preview_failures, landing_retimes, landing_delay, n_replan, n_corrected, .. } = stats;
    let RunMetrics { step_stats, walk_end_state, ramp_end_state, last_state, travel, fell, min_z, max_tilt, max_dcm_err, max_cop_use, max_zmp_clamp, max_ankle_roll_use, n_open_loop, q_lo, q_hi, hit_ticks, hit_peak_f, hit_impulse, hit_first_t, hit_last_t, hit_episodes, n_selfcollide, max_selfcollide_f, push_at_t, push_at_step, push_dcm_err_max, push_tilt_max, push_degraded_at, push_cop_max, push_slip_wbc_max, push_slip_plant_max, push_slip_ticks, push_fz_total_min, push_fz_total_max, push_unloaded_ticks, win_dcm_err_max, win_tilt_max, win_cop_max, win_slip_plant_max, win_slip_ticks, win_fz_total_min, win_fz_total_max, win_unloaded_ticks, win_degraded_at, win_degraded_end, transfer_t, xfer_edge_ticks, xfer_ticks, xfer_leave_t, xfer_wrongfoot_ticks, xfer2_edge_ticks, xfer2_ticks, h_roll_max, h_roll_at_xfer, xfer_dcm_off, xfer_stance, push_ankle_max, push_exceeded, push_last_exceed, t_last, .. } = m;

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
        plan.landing.touchdowns, plan.landing.timeouts
    );
    // The three-phase stair transfer was removed (never adopted, off by
    // default); the line stays so existing log parsers keep their layout.
    println!("STAIR TRANSFER: 0 started / 0 completed / 0 timeouts / 0 active");
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
    // PHASE_BY_CONTACT is removed; the line keeps its off values.
    println!("  phase retimed by contact: 0 early / 0 late ticks, 0 ms total");
    if contact_admit_confirmed || contact_admit_scheduled {
        println!("  confirmed-landing admissions: {n_confirmed_admissions}");
    }
    // ADAPT_STEP / ADAPT_TIME / ADAPT_DS were removed (never adopted, off by
    // default); their summary lines stay, at the zeros they printed, so
    // existing log parsers keep their layout.
    println!("  footstep adaptations: 0 steps moved, largest 0.0 mm, {n_replan} live replans");
    if contact_plan_on {
        println!("  contact plan: {n_cp_shift} shifts, {n_cp_retime} retimes ({:.0} ms total)", cp_retime_total * 1e3);
    }
    if ds_feasible {
        println!("  ds feasible: {n_ds_feasible} retimes ({:.0} ms total)", ds_feasible_total * 1e3);
        println!("  ds capture: {n_ds_capture} early ends ({:.0} ms total)", ds_capture_total * 1e3);
        // RESTEP (removed, never adopted) re-stepped nothing by default.
        println!("  re-steps: 0");
        // VX2_REANCHOR (removed) re-anchored nothing by default.
        println!("  plan re-anchors: 0 (0 mm total)");
        println!("  hip strategy (MOM_CMP): {n_mom_cmp_ticks} ticks, peak rate {mom_cmp_peak:.1} N m");
        if ds_feasible_continuous {
            println!("  ds continuous: {n_ds_continuous} plan writes");
        }
    }
    if push_z_drop > 0.0 {
        println!("  push CoM drops: {n_push_z_drop}");
    }
    if ref_any {
        println!("  reference engagements: {ref_engagements}");
    }
    if plan.landing.event_mode {
        println!(
            "  contact events: {} touchdowns by event, {} probes, {} timeouts",
            plan.landing.events, plan.landing.probes, plan.landing.timeouts
        );
    }
    println!("  step-timing adaptations: 0 ticks retimed, 0 ms total, largest single shift 0 ms");
    println!("  reduced-step cuts: 0 ticks (unreachable -> end the step now)");
    // 20 mm: the ADAPT_TIME_WIDEN default.
    println!("  reactive widen: 0 steps (20 mm, unreachable -> land further out)");
    println!("  double-support retimes: 0 ticks, largest shift 0 ms");
    println!(
        "  step-timing solve: 0 solved / 0 declined \
         (DCM on the far side of the ZMP -- no positive time reaches the target), \
         0 clamped to the duration bounds"
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
        let PushScenario { imp: push_imp, deg: push_deg, dt: push_dt, at: push_at, support: push_support, .. } = &push;
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
                plan.gait.steps_taken(te.max(1e-9)).saturating_sub(push_at_step),
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
    if let Some(path) = diag.substep_audit_path {
        let joints: Vec<_> = rig.robot.joints.iter().enumerate()
            .filter(|(_, joint)| joint.joint_type == "revolute")
            .map(|(ji, joint)| serde_json::json!({
                "name": joint.name, "limits": {"lower_rad": joint.lower, "upper_rad": joint.upper,
                    "velocity_rad_s": joint.velocity, "effort_nm": joint.effort},
                "observed": {"min_q_rad": diag.substep_extrema[ji][0], "max_q_rad": diag.substep_extrema[ji][1],
                    "max_abs_qd_rad_s": diag.substep_extrema[ji][2], "max_abs_tau_nm": diag.substep_extrema[ji][3]},
                "excess": {"position_rad": diag.substep_max_excess[ji][0],
                    "velocity_rad_s": diag.substep_max_excess[ji][1], "effort_nm": diag.substep_max_excess[ji][2]}
            })).collect();
        let passed = diag.substep_nonfinite == 0 && diag.substep_samples == n_ticks * mj_substeps as usize
            && rig.robot.joints.iter().enumerate()
                .filter(|(_, joint)| joint.joint_type == "revolute")
                .all(|(ji, _)| diag.substep_max_excess[ji] == [0.0; 3]);
        let report = serde_json::json!({"schema": 1, "scope": "MuJoCo physics-step trace after each step",
            "physics_dt_s": mj_dt, "expected_samples": n_ticks * mj_substeps as usize,
            "samples": diag.substep_samples, "nonfinite_values": diag.substep_nonfinite,
            "passed": passed, "joints": joints});
        std::fs::write(path, serde_json::to_vec_pretty(&report).unwrap()).expect("SUBSTEP_LIMIT_AUDIT_JSON");
    }
    if mpc_footstep_cross_y > 0.0 {
        println!("  mpc crossing gate: {n_mpc_fs_cross} single supports opened");
    }
    if push_replan_trig > 0.0 {
        println!("  push re-plans: {n_push_replan}");
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
