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
use nalgebra as na;
#[cfg(feature = "mujoco")]
use misa_biped::recovery::RefLib;

#[cfg(feature = "mujoco")]
mod config_env;
#[cfg(feature = "mujoco")]
mod walker;

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
/// them on the way, but none of them is read back by it. Opened together in
/// `open`; a file that is not asked for is `None`. The QP_DIAG_DIR settings
/// are handed to the `TaskStack`, which writes the snapshots (and keeps the
/// physical rows for them, or for QP_RUNTIME_CSV's violation column, without
/// changing the solve).
#[cfg(feature = "mujoco")]
struct DiagSinks {
    traj: articara::biped::log::TrajLog,
    /// TICK_AUDIT_CSV: per-joint pre/post state around every plant step.
    tick_audit: Option<articara::biped::tick_audit::TickAudit>,
    /// QP_DIAG_DIR / QP_CAPTURE_*: the task-stack snapshots (taken by the
    /// `TaskStack` when it is built).
    qp_dump: articara::biped::wbc_stack::QpDump,
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
            qp_dump: articara::biped::wbc_stack::QpDump {
                dir: std::env::var("QP_DIAG_DIR").ok(),
                count: 0,
                capture_all: flag("QP_CAPTURE_ALL", false),
                start: env_f64("QP_CAPTURE_START", 0.0),
                end: env_f64("QP_CAPTURE_END", f64::INFINITY),
                stride: env_f64("QP_CAPTURE_STRIDE", 1.0).max(1.0) as usize,
                max: env_f64("QP_CAPTURE_MAX", 20000.0).max(1.0) as usize,
            },
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

/// The walker's diagnostic records (`misa_biped::sink::WalkerDiag`), written
/// to the CSV / JSON files above in the formats the control used to write
/// them itself.
#[cfg(feature = "mujoco")]
impl misa_biped::sink::WalkerDiag for DiagSinks {
    fn preview_solved(
        &mut self,
        t: f64,
        samples: &[articara::biped::preview::PreviewSample],
        plan: &articara::biped::preview::ComPreview,
    ) {
        if let Some(f) = self.preview_traj_file.as_mut() {
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

    fn preview_runtime(&mut self, t: f64, preview_solve_ns: u128, preview_accepted: bool, terminal_error: f64, com_preview_n: usize) {
        if let Some(writer) = &mut self.com_preview_runtime {
            use std::io::Write as _;
            writeln!(writer, "{t:.6},{preview_solve_ns},{},{},{:.9},{},{}",
                u8::from(preview_solve_ns > 50_000_000),
                u8::from(preview_accepted),
                terminal_error,
                0, // applied_lateral: COM_REPLAN_LATERAL (removed) was off
                com_preview_n,
            ).unwrap();
        }
    }

    fn wants_mpc_input(&self) -> bool {
        self.mpc_diag_dir.is_some()
    }

    fn mpc_input(
        &mut self,
        t: f64,
        dt: f64,
        stair_wrench_dt: f64,
        input: &articara::biped::wrench_mpc::Input,
        config: &articara::biped::wrench_mpc::Config,
    ) {
        let Some(dir) = &self.mpc_diag_dir else { return };
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
            format!("{dir}/mpc_{:06}.json", self.mpc_diag_count),
            serde_json::to_vec(&dump).unwrap(),
        )
        .expect("MPC snapshot");
        self.mpc_diag_count += 1;
    }

    fn mpc_runtime(
        &mut self,
        t: f64,
        mask: u8,
        config: &articara::biped::wrench_mpc::Config,
        stair_wrench_dt: f64,
        mpc_solve_ns: u128,
        accepted: bool,
        previous_age: f64,
    ) {
        if let Some(file) = &mut self.mpc_runtime_log {
            use std::io::Write;
            // Second column: the removed stair transfer's phase,
            // `Capture` being the walk's placeholder.
            writeln!(file, "{t:.9},Capture,{mask},{:?},{},{stair_wrench_dt:.9},{mpc_solve_ns},{},{},{previous_age:.9}",
                config.qp.solver, config.horizon,
                mpc_solve_ns > (stair_wrench_dt * 1e9) as u128,
                accepted).unwrap();
        }
    }

    fn keeps_mpc_frames(&self) -> bool {
        self.qp_wrench_log.is_some()
    }

    fn mpc_solved(
        &mut self,
        t: f64,
        com: &na::Vector3<f64>,
        input: &articara::biped::wrench_mpc::Input,
        solution: &articara::biped::wrench_mpc::Solution,
        net_cop: [f64; 2],
        landing: Option<&articara::biped::wrench_mpc::Landing>,
        by_side: &[articara::biped::wrench_mpc::Wrench; 2],
    ) {
        if let Some(f) = self.mpc_traj_file.as_mut() {
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
            row += &format!(",{:.5},{:.5}", net_cop[0], net_cop[1]);
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
    }
}

/// The walker's log lines to this process's stdout / stderr (the run's log
/// is taken with `2>&1`, so the two streams interleave as written).
#[cfg(feature = "mujoco")]
struct StdSink;

#[cfg(feature = "mujoco")]
impl misa_biped::sink::LogSink for StdSink {
    fn line(&mut self, s: &str) {
        println!("{s}");
    }
    fn err_line(&mut self, s: &str) {
        eprintln!("{s}");
    }
}

/// Between the solve and the plant: the degraded-solve policy and whether a
/// good command has been seen yet (harness side; see `CommandPolicy`).
#[cfg(feature = "mujoco")]
struct CommandGuard {
    policy: articara::biped::actuate::CommandPolicy,
    qp_has_good_command: bool,
}

#[cfg(feature = "mujoco")]
fn main() {
    use articara::biped::actuate::{
        CommandPolicy, DegradedTally, gravity_plus_posture, write_to_plant,
    };
    use articara::biped::gait::{GaitParams, Support};
    use articara::biped::log::{Row, TrajLog, measure_contacts};
    use articara::biped::profile;
    use articara::biped::rig::{BipedRig, CtrlMode, G, RigOptions};
    use articara::biped::wbc_stack::{StackDiag, TaskStack, TaskStackConfig};
    use misa_biped::robot::{Estimate, Observation};
    use misa_biped::wbc::WbcStack;
    use misa_wbc::{SolveConfig, Solver, tasks};
    use nalgebra as na;
    use std::io::Write;

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
        margin: wcfg.descent.leg_reach_margin,
    };
    // The sole's half-size (SOLE_HALF_L / SOLE_HALF_W over the profile's).
    let prof = config_env::profile_from_kv(prof, &|k| std::env::var(k).ok());
    let (sole_half_l, sole_half_w) = prof.cop_half;
    let mu_ground = o.mu_ground;
    let ctrl_mode = o.ctrl_mode;

    // The scene around the robot: the sparring target, the staircase or
    // slope of the planner's terrain model, and the unmodelled bumps.
    let Scene { stairs, spar_wall } = build_scene(&mut o);
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
    let na_count = robot.na;
    let mj_dt = rig.mj_dt;
    let total_mass = robot.total_mass;

    let f_max_per_foot = wcfg.wbc.f_max_per_foot(total_mass, G);
    assert!(
        f_max_per_foot >= total_mass * G,
        "f_max ({f_max_per_foot:.1} N) is below the single-support nominal ({:.1} N)",
        total_mass * G
    );

    // ── Control-law knobs ──────────────────────────────────────────────
    let solver = Solver::new();
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
    let qp_has_good_command = false;
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
    let friction_mu: f64 = wcfg.wbc.friction_mu(mu_ground, prof.friction_margin);
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
    let cop_frac = wcfg.dcm.cop_frac;
    let hold_kp = wcfg.wbc.hold_kp;
    let hold_kd = wcfg.wbc.hold_kd;
    let hold_bridge = wcfg.wbc.hold_bridge;
    let fallback_max_level = wcfg.wbc.fallback_level;
    let total_t = env_f64("T", 25.0);
    let gp = GaitParams {
        t_start: wcfg.gait.t_start,
        t_ds: wcfg.gait.t_ds,
        t_ss: wcfg.gait.t_ss,
        n_steps: wcfg.gait.n_steps,
        first_swing: wcfg.gait.first_swing,
        t_end: total_t,
    };

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
    // its initial state (walker.rs; the plan's lines are printed here).
    // The command, read here and woven into the plan at start-up.
    let command = walker::CommandSchedule {
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
        jab_pattern: jab_pattern.clone(),
        jab_pitch,
        jab_elbow,
    };
    let (vx, vy, wz) = (command.vx, command.vy, command.wz);
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
        &mut StdSink,
    );
    let omega = init.plan().dcm.omega;

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
    // P2-1 contact planner (doc/from_claude/20260923_p2_contact_planner_design_v2.md).
    // Predictive replacement for ADAPT_STEP/ADAPT_TIME (both removed, never
    // adopted): picks the landing shift AND the end of the current single
    // support so the predicted end-of-step DCM meets the plan's. Default OFF.
    // (Its knobs, CP_*, are `ContactPlanConfig`; T_max defaults to 1, never
    // extending the single support.)
    let contact_plan_on = wcfg.contact_plan.on;
    let contact_plan_cfg = wcfg.contact_plan.planner();
    // Terrain height under the support, slewed (below) so the CoM height
    // reference climbs a riser over about a step instead of jumping at a
    // slice boundary. Starts at the spawn platform's height.
    // FIX_*: the corrective-impulse probe (see `FixProbe`).
    let fix = FixProbe::from_env();
    let com_replan_forward_gain = wcfg.com_plan.replan_forward_gain;
    assert!(com_replan_forward_gain.is_finite() && (0.0..=1.0).contains(&com_replan_forward_gain));
    // (The confirmation's xy / z tolerances, LAND_CONFIRM_XY / _Z, are set
    // on the landing clock by `Config::landing_timing`; see
    // LandingTiming::update.)
    // (P2-2 LAND_BY_LOAD, confirming a landing on real load, was not
    // adopted and is removed.)
    // Admit a CONFIRMED landing to the contact set on the schedule instead
    // of waiting for the 10% BW load hysteresis (`ContactCorrection::admit`).
    // The landing gate already certified the sole on its tread (xy/z within
    // tolerance, fz above the gate); what it cannot certify is LOAD, because
    // load needs the QP to model the contact first. Default off = shipped.
    let contact_admit_confirmed = wcfg.contact.admit_confirmed;
    let contact_admit_scheduled = wcfg.contact.admit_scheduled;
    let gait_dump_t: Option<(f64, f64)> = std::env::var("GAIT_DUMP_T").ok().and_then(|v| {
        let mut it = v.split(',').map(|x| x.trim().parse::<f64>().ok());
        Some((it.next()??, it.next()??))
    });
    let contact_dump_t: Option<(f64, f64)> = std::env::var("CONTACT_DUMP_T").ok().and_then(|v| {
        let mut it = v.split(',').map(|x| x.trim().parse::<f64>().ok());
        Some((it.next()??, it.next()??))
    });
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
    let ds_feasible_continuous = wcfg.double_support.feasible_continuous;
    // Per side: the DOUBLE-SUPPORT slice that follows a certified landing.
    // The foot is a planned contact for that slice and is re-admitted
    // whenever the load hysteresis drops it there; cleared afterwards.
    // Event-driven contact (doc/from_claude/20261004_event_driven_contact_design.md):
    // touchdown = measured load with the sole at its planned height (no xy
    // gate), the plan is moved onto the foot, a descending swing ends at the
    // event, a rising one keeps COM_LAND_EXTEND_RISE as an explicit dwell,
    // and a swing that has not touched by its deadline probes down instead
    // of timing out. Default off = bit-identical.
    // (REF_MPC / REF_WBC, the MPC's and the WBC's references taken from
    // the reference library for every reference, were not adopted and are
    // removed.)
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
    let ds_feasible = wcfg.double_support.feasible;
    // (LIFT_GATE_FRAC / _MAX, holding a double support while the lateral DCM
    // is still over the lifting foot, were not adopted and are removed.)
    // Landing-deadline slack for a swing that steps DOWN. Default = the rise
    // value (bit-identical). Measured 2026-10-03 on a 60 mm descent: the
    // 1.08 s rise slack made the stepping-down foot hover on the lower
    // surface unloaded while the body walked on (2 timeouts, 2.16 s net),
    // and the next swing started with 22 N on the new stance -- a fall at
    // 3/48. With 0 the same member walked out.
    // ... and let its solution reach the WBC outside a transfer. Measured
    // 2026-09-24: every consumer of `wrench_mpc_command` (accel, coupled
    // level, forces_nominal blend, fz plan) is conditioned on a transfer, so
    // ALWAYS alone solved 1500 times per run and discarded the result; a
    // load-share change in the MPC input was bit-identical at the output.
    // With this on, the coupled regulariser level and the forces_nominal
    // blend follow the MPC whenever it has a fresh solution. Default off.
    let mpc_footstep = wcfg.mpc_footstep.on;
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
    let mpc_footstep_cross_y = wcfg.mpc_cross.y;
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
    let push_replan_trig = wcfg.push_replan.trig;
    // (slice, inward is +y) latched when the crossing gate opens: once the
    // landing has crossed, `landing_inward` would read the sign the other way.
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
    // The CoM preview QP's horizontal trajectory, kept whole with the time it
    // was solved at, so the MPC can read its own horizon out of it.

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

    let p0_body = rig.sim.body_world_position(&rig.robot.root_link).unwrap();
    let yaw0_body = rig
        .sim
        .body_world_orientation(&rig.robot.root_link)
        .unwrap()
        .euler_angles()
        .2;
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
    // What the run measured about itself (achievement, extremes, the
    // push response), for the report after the loop.
    let mut m = RunMetrics::new(rig.robot.joints.len(), p0_body, yaw0_body);
    // Optional norm bound on the change in commanded horizontal CoM
    // acceleration.  The DCM/ZMP reference is acceleration-continuous, but
    // its piecewise-linear ZMP slope changes at support boundaries, producing
    // a jerk step.  Limiting that step preserves the full DCM feedback law
    // while giving contacts and counterweight joints time to respond.
    let com_xy_jerk_max = wcfg.dcm.com_xy_jerk_max;
    assert!(
        com_xy_jerk_max.is_finite() && com_xy_jerk_max > 0.0 || com_xy_jerk_max == f64::INFINITY,
        "COM_XY_JERK_MAX must be positive"
    );
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
    // Where the MPC's own answer puts the NET pressure of both feet, in
    // world. The split alone does not determine this -- each sole's own CoP
    // moves it too -- so it has to be computed where the wrenches and their
    // contact frames are both in hand.
    let act_delay_ticks = env_f64("ACT_DELAY_TICKS", 0.0).max(0.0) as usize;
    let mut act_delay_buf: std::collections::VecDeque<Vec<f64>> = std::collections::VecDeque::new();
    // Preserve the physical origin/axes of the held MPC forecast for diagnostic shifts.
    // MPC_FOOTSTEP bookkeeping: which slice's landing is being moved and how
    // much of the MPC's offset has already been applied to it (the same
    // non-ratcheting rule as `cp_applied`).
    // (first slice, last slice) the landing variable is armed for.

    // The WBC: the stack of QP levels (the solver, its warm start and the
    // contact anchors are its own), with the QP_DIAG_DIR settings and
    // whether QP_RUNTIME_CSV wants the physical violation measured.
    let stack = TaskStack::new(
        TaskStackConfig::new(
            &wcfg,
            cfg,
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
    let mut guard = CommandGuard {
        policy,
        qp_has_good_command,
    };
    let mut walker = init.with_stack(stack);


    // The walker's log lines go straight to stdout / stderr.
    let mut out = StdSink;
    for tick in 0..n_ticks {
        let qp_control_start = std::time::Instant::now();
        let t = tick as f64 * dt;
        m.t_last = t;
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
        // What the walker is given this tick: the state the rig assembled,
        // the vertical contact forces, and the plant's joint angles (read by
        // the posture levels; `misa_biped::robot::Observation`).
        let obs = Observation {
            tick,
            estimate: Estimate::State(&st),
            contact_fz: measured_pre,
            joint_pos_plant: &rig.robot.joint_positions,
        };
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

        // ---- the walker's control (walker.rs, phases 1-12): its lines are
        // printed as they are decided, its records go to `diag`.
        let walker::StepOutput {
            command: walker::Command { sol, extracted, tau_gravity, rejected: qp_rejected, degraded },
            record: rec,
        } = walker.step(&obs, &mut out, &mut diag);
        let measured = measured_c;
        tally.observe(&sol.status, t, tick, rec.nc, &rec.level_names);
        // Run measurements from the record (they were taken inside the
        // control before; the values are the same).
        m.max_dcm_err = m.max_dcm_err.max(rec.dcm_err);
        // Post-push accumulation. The push fires late in the tick, after this
        // point, so the firing tick's error is still the undisturbed one and
        // the window starts one tick later -- which is what we want.
        if in_push_window {
            m.win_dcm_err_max = m.win_dcm_err_max.max(rec.dcm_err);
        }
        if m.push_at_t.is_some() {
            m.push_dcm_err_max = m.push_dcm_err_max.max(rec.dcm_err);
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
            if rec.dcm_err >= push_recover_m {
                m.push_exceeded = true;
                m.push_last_exceed = Some(t);
            }
        }
        m.max_zmp_clamp = m.max_zmp_clamp.max(rec.clamped);
        if let Some(file) = diag.transfer_diag.as_mut() {
            let mut nominal = [0.0; 2];
            let mut mask = 0u8;
            for (slot, &side) in rec.stance_sides.iter().enumerate() {
                mask |= 1 << side;
                // Native WBC force blocks use world axes (as f_qp_w above).
                nominal[side] = rec.forces_nominal[6 * slot + 5];
            }
            let bw = total_mass * G;
            // Load threshold is evidence of support, not a contact detector.
            let measured_mask = u8::from(measured_pre[0] > 0.05 * bw)
                | (u8::from(measured_pre[1] > 0.05 * bw) << 1);
            let rot = misarta::se3::rotation_matrix(&data.oMi[rig.trunk_mi]);
            let mpc_valid = rec.wrench_mpc_command.is_some();
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
                    rec.diag_mpc_time.map_or(f64::NAN, |ts| t - ts)
                } else {
                    f64::NAN
                },
                walker.stats().wrench_mpc_failures as f64,
                if mpc_valid {
                    rec.diag_mpc_reference[0]
                } else {
                    f64::NAN
                },
                if mpc_valid {
                    rec.diag_mpc_reference[1]
                } else {
                    f64::NAN
                },
                rec.wrench_mpc_command.as_ref().map_or(f64::NAN, |w| w[0][5]),
                rec.wrench_mpc_command.as_ref().map_or(f64::NAN, |w| w[1][5]),
                nominal[0],
                nominal[1],
                rec.f_qp_w[0][2],
                rec.f_qp_w[1][2],
                measured_pre[0],
                measured_pre[1],
                (-rot[(2, 0)]).clamp(-1.0, 1.0).asin(),
                rot[(2, 1)].atan2(rot[(2, 2)]),
                rec.cop_qp[0][0],
                rec.cop_qp[1][0],
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
        m.max_cop_use = m.max_cop_use.max(rec.cop_use);
        if m.push_at_t.is_some() {
            m.push_cop_max = m.push_cop_max.max(rec.cop_use);
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
                if let (Some(Support::Double), Support::Single { .. }) = (m.support_prev, rec.support) {
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
                    if let Support::Single { stance, .. } = rec.support {
                        let u = rec.steps.xy(stance);
                        m.xfer_dcm_off = [rec.xi.x - u.x, rec.xi.y - u.y];
                        m.xfer_stance = stance as i32;
                    }
                }
            } else if m.transfer2_t.is_none() && t > m.transfer_t.unwrap() + 1e-9 {
                if let (Some(Support::Double), Support::Single { .. }) = (m.support_prev, rec.support) {
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
                        if !rec.support.is_stance(side) {
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
                m.win_cop_max = m.win_cop_max.max(rec.cop_use);
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
        // WBC_REPAIR_ORIGINAL / WBC_PREFIX_MIN_LEVEL (repair the failed QP,
        // or command a checked prefix) were experiments and are removed;
        // QP_RUNTIME_CSV keeps their columns at the off values.
        let prefix_applied = false;
        let repair_applied = false;
        let repair_scaled_delta = 0.0_f64;
        let repair_ns = 0u128;
        {
            let rig_ref = &rig;
            let tg = &tau_gravity;
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
        if matches!(rec.support, Support::Single { .. }) {
            let stance_side = match rec.support {
                Support::Single { stance, .. } => stance,
                Support::Double => unreachable!(),
            };
            if m.step_stats.last().map(|s| s.slice) != Some(rec.slice_idx) {
                m.step_stats.push(StepStat {
                    idx: rec.step_idx,
                    slice: rec.slice_idx,
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
            s.dcm_err_max = s.dcm_err_max.max(rec.dcm_err);
            s.cop_use_max = s.cop_use_max.max(rec.cop_use);
            s.fz_peak = s.fz_peak.max(measured.f_w[0][2] + measured.f_w[1][2]);
            s.degraded += u32::from(degraded);
            if let Some(sw) = rec.support.swing() {
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
                &PushTick {
                    t,
                    step_idx: rec.step_idx,
                    support: rec.support,
                    slice_frac: rec.slice_frac,
                    slice_t1: rec.slice.t1,
                },
                total_mass,
                omega,
                jab.then_some(JabClock { start: jab_start, period: jab_period, pattern: &jab_pattern }),
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
                .saturating_sub(rec.qp_capture_ns)
                .saturating_sub(rec.mpc_capture_ns);
            writeln!(file, "{t:.9},{qp_solve_ns},{qp_check_ns},\"{:?}\",{qp_physical_violation:.12e},{qp_reset},{},{control_ns},{},{wrench_mpc_solves},{wrench_mpc_failures},{prefix_applied},{repair_applied},{repair_scaled_delta:.12e},{repair_ns}", sol.status,
                control_ns > (dt * 1e9) as u128, rec.diag_mpc_time.map_or(-1.0, |stamp| t-stamp),
                wrench_mpc_solves = walker.stats().wrench_mpc_solves, wrench_mpc_failures = walker.stats().wrench_mpc_failures,
                qp_solve_ns = rec.qp_solve_ns, qp_check_ns = rec.qp_check_ns,
                qp_physical_violation = rec.qp_physical_violation, qp_reset = rec.qp_reset).unwrap();
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
                let slot = rec.stance_sides.iter().position(|s| *s == side);
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
                            &rec.forces_nominal.as_slice()[6 * slot + 3..6 * slot + 6],
                        );
                        let moment = na::Vector3::from_column_slice(
                            &rec.forces_nominal.as_slice()[6 * slot..6 * slot + 3],
                        );
                        planned =
                            contact_wrench_in_sole(&origin, &force, &moment, &sole, &rotation);
                    }
                    if let (Some(command), Some(frames)) = (&rec.wrench_mpc_command, &rec.wrench_mpc_frames)
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
                let planned_support = match rec.support {
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
                    rec.diag_mpc_time.map_or(-1.0, |stamp| t - stamp)
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
                [rec.f_qp_w[0][2], rec.f_qp_w[1][2]],
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
                rec.f_qp_w[1][0], rec.f_qp_w[1][1], rec.f_qp_w[1][2],
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
            let support_code = match rec.support {
                Support::Double => 0.0,
                Support::Single { stance, .. } => 1.0 + stance as f64,
            };
            let actual_feet = rec.actual_feet;
            let [al, ar] = actual_feet.sole;
            let [gl, gr] = rec.steps.sole;
            let link_l = misarta::se3::translation(&data.oMi[rig.foot_mi[0]]);
            let link_r = misarta::se3::translation(&data.oMi[rig.foot_mi[1]]);
            let anchor_l_y = walker.stack().anchor(0).map_or(f64::NAN, |(p, _)| p.y);
            let anchor_r_y = walker.stack().anchor(1).map_or(f64::NAN, |(p, _)| p.y);
            let foot_v_l = rig
                .sim
                .body_world_linear_velocity(prof.foot_links[0])
                .unwrap_or([0.0; 3]);
            let foot_v_r = rig
                .sim
                .body_world_linear_velocity(prof.foot_links[1])
                .unwrap_or([0.0; 3]);
            let extra = [
                rec.xi.x,
                rec.xi.y,
                rec.r.xi.x,
                rec.r.xi.y,
                rec.r.p.x,
                rec.r.p.y,
                rec.p_cmd.x,
                rec.p_cmd.y,
                rec.clamped,
                rec.dcm_err,
                support_code,
                rec.step_idx as f64,
                *rec.shares.first().unwrap_or(&0.0),
                *rec.shares.get(1).unwrap_or(&0.0),
                rec.swing_tgt.x,
                rec.swing_tgt.y,
                rec.swing_tgt.z,
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
                if rec.replan_engaged { 1.0 } else { 0.0 },
                rec.preview_c.x,
                rec.preview_c.y,
                rec.preview_v.x,
                rec.preview_v.y,
                walker.stats().preview_error,
                walker.stats().preview_count as f64,
                walker.stats().preview_failures as f64,
                walker.stats().landing_delay,
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
                rec.trunk_cmd_rp[1],
                rec.trunk_acc_rp[1],
                rec.trunk_cmd_rp[0],
                rec.trunk_acc_rp[0],
                -1.0, // transfer_guard (removed stair transfer)
                if rec.wrench_mpc_command.is_some() {
                    1.0
                } else {
                    0.0
                },
                rec.wrench_mpc_command.as_ref().map_or(0.0, |w| w[0][5]),
                rec.wrench_mpc_command.as_ref().map_or(0.0, |w| w[1][5]),
                rec.diag_mpc_net_cop[0],
                rec.diag_mpc_net_cop[1],
                rec.mpc_fs_last.x,
                rec.mpc_fs_last.y,
                rec.diag_wbc_net_cop[0],
                rec.diag_wbc_net_cop[1],
                walker.stats().wrench_mpc_solves as f64,
                walker.stats().wrench_mpc_failures as f64,
                rec.swing_pos_err_xy,
                rec.swing_pos_err_z,
                rec.swing_task_residual_rms,
                rec.swing_res_z,
                rec.swing_res_rows,
                rec.swing_cmd_acc_z,
                rec.swing_achieved_acc_z,
                tau_use_max,
                tau_saturated_count as f64,
                rec.reach_used[0],
                rec.reach_used[1],
                rec.hip_z[0],
                rec.hip_z[1],
                rec.stride_max_now,
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
                    measured.cop[0][2],measured.cop[1][2],rec.cop_qp[0][2],rec.cop_qp[1][2],rec.nc).unwrap();
            }
            // GAIT_DUMP_T="t0,t1": one line per tick with the gait slice the
            // clock is on against the contact-driven stance set.
            if let Some((d0, d1)) = gait_dump_t
                && t >= d0
                && t <= d1
            {
                let i = walker.plan().gait.index_at(t);
                let sl = &walker.plan().gait.slices[i];
                println!(
                    "  [gait] t={t:.3} slice {i} {:?} [{:.3},{:.3}] stance={:?} fz=({:.0},{:.0}) xi.x={:.3} ref.x={:.3}",
                    sl.support, sl.t0, sl.t1, rec.stance_sides, rec.fz_meas[0], rec.fz_meas[1], rec.xi.x, rec.r.xi.x
                );
            }
            let row = Row {
                t,
                com,
                com_ref_z: rec.z_ref,
                // The renderer's `com_ref_y` panel is the lateral reference;
                // for a walk that is the DCM target, not a fixed point.
                com_ref_y: rec.r.xi.y,
                tilt: rec.roll.abs().max(rec.pitch.abs()),
                trunk_tilt,
                n_stance: rec.nc,
                foot_z,
                swing_z,
                foot_vz,
                acc_dbg: rec.acc_dbg,
                a_com: rec.a_com,
                rp_ref: rec.rp_ref,
                degraded,
                cop_box: (sole_half_l * cop_frac, sole_half_w * cop_frac),
                cop_qp: rec.cop_qp,
                f_qp_w: rec.f_qp_w,
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
        let tilt = rec.roll.abs().max(rec.pitch.abs());
        m.max_tilt = m.max_tilt.max(tilt);
        if m.push_at_t.is_some() {
            m.push_tilt_max = m.push_tilt_max.max(tilt);
        }
        if in_push_window {
            m.win_tilt_max = m.win_tilt_max.max(tilt);
        }
        if tick % 40 == 0 {
            let sup = match rec.support {
                Support::Double => "DS  ".to_string(),
                Support::Single { stance, .. } => {
                    format!("SS{}", if stance == 0 { "L" } else { "R" })
                }
            };
            println!(
                "  t={t:6.3} {sup} step={step_idx:3}  com=({:+.4},{:+.4},{:+.4})  \
                 xi=({:+.4},{:+.4}) err={:.4}  zmp=({:+.4},{:+.4})  cop_use={cop_use:.2}  {:?}",
                com.x, com.y, com.z, rec.xi.x, rec.xi.y, rec.dcm_err, rec.p_cmd.x, rec.p_cmd.y, sol.status,
                step_idx = rec.step_idx, cop_use = rec.cop_use
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
    let walker::WalkerStats { n_mom_cmp_ticks, mom_cmp_peak, n_push_z_drop, n_cp_shift, n_cp_retime, cp_retime_total, n_confirmed_admissions, ref_engagements, n_ds_capture, ds_capture_total, n_ds_feasible, n_ds_continuous, ds_feasible_total, n_push_replan, n_mpc_fs_cross, preview_count, preview_failures, landing_retimes, landing_delay, n_replan, n_corrected, .. } = walker.stats().clone();
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
        walker.plan().landing.touchdowns, walker.plan().landing.timeouts
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
    if wcfg.push_crouch.drop > 0.0 {
        println!("  push CoM drops: {n_push_z_drop}");
    }
    if ref_any {
        println!("  reference engagements: {ref_engagements}");
    }
    if walker.plan().landing.event_mode {
        println!(
            "  contact events: {} touchdowns by event, {} probes, {} timeouts",
            walker.plan().landing.events, walker.plan().landing.probes, walker.plan().landing.timeouts
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
                    (walker.initial_feet().sole[0].y - walker.initial_feet().sole[1].y).abs() / (1.0 + (omega * gp.t_ss).exp());
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
                walker.plan().gait.steps_taken(te.max(1e-9)).saturating_sub(push_at_step),
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
