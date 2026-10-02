//! Model + plant bring-up for the biped WBC, and the per-tick state sync.
//!
//! Everything here happens once, before the control loop, except
//! [`BipedRig::sync`]. Splitting it out is not cosmetic: the setup contains
//! three separate guards that each caught a bug which made a later
//! measurement meaningless (self-collision at spawn, self-collision in the
//! burn-in rig, and an unset torque-box row), and burying them inside a
//! 1900-line `main` is how the second one came to be missing for a month.

use nalgebra as na;
use std::collections::HashMap;
use std::cell::RefCell;
use std::collections::VecDeque;

use super::profile::Profile;
use crate::mjcf::{GroundPlaneCfg, MjcfExportOptions};
use crate::mujoco_sim::MujocoSim;
use crate::rbd::model::ActuatorMode;
use crate::robot::RobotModel;
use crate::wbc_pipeline::build_floating_base_model;

pub const G: f64 = 9.81;

/// A link that carries mass, paired with its misarta index and the CoM
/// offset in the link frame -- everything `J_com` needs.
pub struct MassLink {
    pub mi: usize,
    pub m: f64,
    pub com_local: na::Vector3<f64>,
}

/// How the WBC's answer reaches the plant.
///
/// The motivation for anything but `Torque` is measured, not stylistic:
/// between WBC ticks the plant runs open-loop on a torque that was computed
/// for a state it has since left, and at 5 ms G1's commanded torque reached
/// 2.06x its own box purely from that drift. Measured, though, every
/// alternative is worse -- see `doc/kyo46rs_biped_wbc.md` section 7.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum CtrlMode {
    /// Command tau directly.
    Torque,
    /// `(q, dq, kp, kd, tau)` -- the interface real hardware exposes.
    Hybrid,
    /// Integrate the QP's qddot to a joint velocity command, realised by an
    /// explicit PD in Rust. The force variables stop being commands and
    /// become predictions.
    Velocity,
    /// Velocity-resolved as well, but the loop lives inside MuJoCo
    /// (`<velocity>` actuator + `implicitfast`) instead of Rust.
    Servo,
}

impl CtrlMode {
    pub fn from_env_name(s: &str) -> Self {
        match s {
            "torque" => CtrlMode::Torque,
            "hybrid" => CtrlMode::Hybrid,
            "velocity" => CtrlMode::Velocity,
            "servo" => CtrlMode::Servo,
            other => panic!("unknown CTRL_MODE={other:?} (torque | hybrid | velocity | servo)"),
        }
    }
    pub fn name(self) -> &'static str {
        match self {
            CtrlMode::Torque => "torque",
            CtrlMode::Hybrid => "hybrid",
            CtrlMode::Velocity => "velocity",
            CtrlMode::Servo => "servo",
        }
    }
    /// Velocity-resolved, either flavour.
    pub fn is_velocity(self) -> bool {
        matches!(self, CtrlMode::Velocity | CtrlMode::Servo)
    }
}

/// Everything the bring-up reads that is not already in the [`Profile`].
/// Split out so a driver can expose whichever of them it wants as env knobs
/// without the rig having to know about the environment at all.
pub struct RigOptions {
    /// Extra MJCF spliced into the scene's `<worldbody>` -- a target to hit, a
    /// wall, an obstacle. See [`crate::mjcf::MjcfExportOptions::extra_worldbody`].
    pub extra_worldbody: Option<String>,
    /// Crouch seed. hip + knee + ankle must sum to zero for a flat sole.
    pub knee: f64,
    pub hip_pitch: f64,
    pub ankle_pitch: f64,
    pub joint_damping: f64,
    pub armature: f64,
    /// Scale every actuator limit, to separate "the foot is too small" from
    /// "the motors are too weak" as the cause of a level-0 infeasibility.
    pub torque_scale: f64,
    pub burnin_kp: f64,
    pub burnin_kv: f64,
    pub burnin_s: f64,
    pub run_kp: f64,
    pub run_kv: f64,
    pub ctrl_mode: CtrlMode,
    pub sim_dt: f64,
    /// Plant-side friction, separate from the QP's `friction_mu`. Raising it
    /// is not a fix -- it is how to measure what the stance foot's slip is
    /// costing, by removing the slip and nothing else.
    pub mu_ground: f64,
    /// Optional MuJoCo impedance curve for collision geoms on both feet.
    /// `None` preserves the model/default contact. This is a plant design
    /// input for evaluating compliant soles, independent of the WBC cone.
    pub foot_solimp: Option<[f64; 3]>,
    pub probe_z: f64,
    /// Extra `(joint, angle)` seeds applied on top of the crouch.
    ///
    /// Needed because the crouch alone does not determine whether the robot
    /// can move without hitting itself. On kyo46rs the arm has only a
    /// shoulder PITCH and an elbow -- no abduction -- so a forearm left
    /// hanging beside the hip has 2.5 mm of clearance (shoulder at y=0.115,
    /// forearm 35 mm wide, hip block reaching y=0.095), and the hip roll a
    /// step needs closes it. Swinging the arm out of the frontal plane is the
    /// only free variable that fixes it.
    pub extra_seed: Vec<(&'static str, f64)>,
    /// Seed offsets ADDED to whatever the posture already holds, applied
    /// after [`Self::extra_seed`]. `extra_seed` assigns, which is what a
    /// stance-width seed wants; a jitter on the initial condition has to add,
    /// or it silently erases the profile's own posture (measured 2026-09-24:
    /// assigning a 1e-9 "jitter" to the leg joints straightened the knees and
    /// moved the foot inner-gap 22 mm).
    pub extra_seed_delta: Vec<(&'static str, f64)>,
    /// DIAGNOSTIC: drop the collision geometry of these links entirely.
    ///
    /// Purpose is separation, not repair. When the robot fouls itself the
    /// resulting force is invisible to the QP, and every downstream number is
    /// then a measurement of the brace rather than of the controller. Removing
    /// the geometry answers one question -- "is the self-collision the only
    /// thing stopping this?" -- and answers nothing else. A run with this set
    /// is not a result; it is an experiment about which result to chase.
    pub uncollide_links: Vec<&'static str>,
    /// Per-joint viscous damping overrides, matched by substring against the
    /// joint name (e.g. `("elbow", 0.25)`), applied over the uniform
    /// `joint_damping` and carried consistently into BOTH the MJCF export
    /// (plant) and the QP's nonlinear-effects compensation. This is a DESIGN
    /// input, not a tuning trick: the robot exists only in simulation, and a
    /// torque-mode actuator's own back-EMF is a real viscous term of order
    /// stall_torque / no_load_speed (XM430: 2.3 N*m / ~10 rad/s ~ 0.23),
    /// where the uniform 0.15 was only ever a guess.
    pub joint_damping_overrides: Vec<(String, f64)>,
    /// Per-joint reflected rotor inertia, matched by joint-name substring.
    /// The same value reaches the MuJoCo joint and the WBC mass diagonal.
    pub joint_armature_overrides: Vec<(String, f64)>,
    /// Added to the measured spawn height. The probe lift puts the soles on
    /// the z=0 ground plane; a driver that starts the robot ON something --
    /// a platform at the top of a descending staircase -- owes the spawn the
    /// platform's height, because the probe knows only the plane.
    pub spawn_z_offset: f64,
}

impl RigOptions {
    /// Profile defaults, before any env override.
    pub fn from_profile(p: &Profile) -> Self {
        RigOptions {
            extra_worldbody: None,
            knee: p.knee_seed,
            hip_pitch: -p.knee_seed / 2.0,
            ankle_pitch: -p.knee_seed / 2.0,
            joint_damping: p.joint_damping,
            armature: p.armature,
            torque_scale: 1.0,
            burnin_kp: p.burnin_kp,
            burnin_kv: p.burnin_kv,
            burnin_s: p.burnin_s,
            run_kp: p.burnin_kp,
            run_kv: p.burnin_kv,
            ctrl_mode: CtrlMode::Torque,
            sim_dt: 0.001,
            mu_ground: 0.7,
            foot_solimp: None,
            probe_z: p.probe_z,
            extra_seed: Vec::new(),
            extra_seed_delta: Vec::new(),
            uncollide_links: Vec::new(),
            joint_damping_overrides: Vec::new(),
            joint_armature_overrides: Vec::new(),
            spawn_z_offset: 0.0,
        }
    }
}

/// Model, plant and the index maps between them.
pub struct BipedRig {
    pub prof: Profile,
    pub opts_ctrl_mode: CtrlMode,
    pub robot: RobotModel,
    pub sim: MujocoSim,
    pub model: misarta::model::Model<f64>,
    /// articara joint index -> misarta index.
    pub a2m: Vec<Option<usize>>,
    pub link_to_idx: HashMap<String, usize>,
    /// misarta indices of [left, right] foot links.
    pub foot_mi: [usize; 2],
    /// Body P2 holds upright, and whether that body IS the FreeFlyer.
    pub trunk_mi: usize,
    pub trunk_from_base: bool,
    pub mass_links: Vec<MassLink>,
    pub total_mass: f64,
    pub torque_max: na::DVector<f64>,
    /// URDF position/velocity limits, indexed like `torque_max` (misarta
    /// v-index - 6). For `joint_limit_cbf` (doc Sec.21.7 item 4 / Sec.25.4) --
    /// not wired into any level by default (see `JLIM` in kyo46rs_walk.rs).
    pub q_min: na::DVector<f64>,
    pub q_max: na::DVector<f64>,
    pub v_max: na::DVector<f64>,
    /// Joint angles the posture task and the degraded-solve fallback hold.
    pub q_seed: Vec<f64>,
    pub nv: usize,
    pub na: usize,
    pub mj_dt: f64,
    pub spawn_z: f64,
    pub armature: f64,
    pub joint_damping: f64,
    /// Joint-velocity magnitude the CONTROLLER is allowed to see (sync()
    /// clamps the plant readback to this). Historically hardcoded at
    /// 5 rad/s; measured on the four-step stair climb, leg joints really
    /// move at 10-31 rad/s, so the QP's velocity feedback, Coriolis terms
    /// and swing damping were computed from saturated readings exactly
    /// during the fastest swings -- and the saturation is a hard
    /// nonlinearity, which is one candidate for the knife-edge parameter
    /// sensitivity. `QD_CLAMP` overrides; default keeps the old 5.0.
    pub qd_clamp: f64,
    /// Sensor corruption for benchmarks (all default off = the plant's truth,
    /// bit-identical): `SENSE_DELAY_TICKS` holds every measurement that many
    /// control ticks; `SENSE_NOISE_Q/QD/V/W` add white noise (std, in rad,
    /// rad/s, m/s, rad/s) to joint positions, joint velocities, base linear
    /// and base angular velocity; `SENSE_SEED` seeds it.
    pub sense_delay: usize,
    pub sense_noise: [f64; 4],
    sense_rng: RefCell<u64>,
    sense_buf: RefCell<VecDeque<RawSense>>,
}

#[derive(Clone)]
struct RawSense {
    body_pos: [f64; 3],
    body_quat: na::UnitQuaternion<f64>,
    v_lin_w: [f64; 3],
    v_ang_w: [f64; 3],
    joint_q: Vec<f64>,
    joint_qd: Vec<Option<f64>>,
}

fn sense_env(k: &str, d: f64) -> f64 {
    std::env::var(k).ok().and_then(|v| v.parse().ok()).unwrap_or(d)
}

/// xorshift64* -> standard normal (Box-Muller), no dependency.
fn gauss(state: &mut u64) -> f64 {
    let mut next = || {
        let mut x = *state;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        *state = x;
        (x.wrapping_mul(0x2545F4914F6CDD1D) >> 11) as f64 / (1u64 << 53) as f64
    };
    let u1 = next().max(1e-300);
    let u2 = next();
    (-2.0 * u1.ln()).sqrt() * (2.0 * std::f64::consts::PI * u2).cos()
}

/// One tick's worth of synchronised state: the plant read into misarta's
/// coordinates, plus the dynamics quantities every task needs.
pub struct BipedState {
    pub q: Vec<f64>,
    pub v: Vec<f64>,
    pub v_dvec: na::DVector<f64>,
    pub data: misarta::data::Data<f64>,
    /// Mass matrix WITH reflected rotor inertia on the actuated diagonal.
    pub mass: na::DMatrix<f64>,
    /// Nonlinear effects WITH viscous joint damping.
    pub h: na::DVector<f64>,
    pub com: na::Vector3<f64>,
    pub com_vel: na::Vector3<f64>,
    pub j_com: na::DMatrix<f64>,
    pub djv_com: na::Vector3<f64>,
    pub body_pos: [f64; 3],
    pub body_quat: na::UnitQuaternion<f64>,
    pub v_ang_w: [f64; 3],
}

impl BipedRig {
    /// URDF -> crouch seed -> spawn probe -> welded burn-in -> free base.
    ///
    /// Prints the same diagnostics the monolithic example did, because they
    /// are how a broken run is recognised before its numbers are believed.
    pub fn build(prof: Profile, o: &RigOptions) -> Self {
        println!("robot: {}", prof.name);

        let urdf_path = std::path::Path::new(prof.urdf);
        let mut robot =
            RobotModel::from_urdf(urdf_path).unwrap_or_else(|e| panic!("load {}: {e}", prof.urdf));
        let crouch: Vec<(&str, f64)> = prof
            .sagittal
            .iter()
            .flat_map(|[h, k, a]| [(*h, o.hip_pitch), (*k, o.knee), (*a, o.ankle_pitch)])
            .collect();
        for (name, q) in crouch.iter().copied() {
            if let Some(&ji) = robot.joint_map.get(name) {
                robot.joint_positions[ji] = q;
            }
        }
        for (name, q) in o.extra_seed.iter().copied() {
            let ji = *robot.joint_map.get(name).unwrap_or_else(|| {
                panic!("extra_seed names {name}, which this robot does not have")
            });
            robot.joint_positions[ji] = q;
        }
        for (name, dq) in o.extra_seed_delta.iter().copied() {
            let ji = *robot.joint_map.get(name).unwrap_or_else(|| {
                panic!("extra_seed_delta names {name}, which this robot does not have")
            });
            robot.joint_positions[ji] += dq;
        }
        if prof.collide_primitives_only {
            use crate::rbd::model::GeomData;
            let mut dropped = 0usize;
            for link in robot.links.iter_mut() {
                let before = link.collisions.len();
                link.collisions
                    .retain(|c| !matches!(c.geometry, GeomData::Mesh { .. }));
                dropped += before - link.collisions.len();
            }
            let kept: usize = robot.links.iter().map(|l| l.collisions.len()).sum();
            println!(
                "collision: dropped {dropped} mesh geoms, kept {kept} primitives (visual meshes untouched)"
            );
            assert!(
                kept > 0,
                "no primitive colliders left -- the feet would have nothing to stand on"
            );
        }
        if let Some(solimp) = o.foot_solimp {
            let mut tuned = 0usize;
            for link in robot.links.iter_mut() {
                if prof.foot_links.contains(&link.name.as_str()) {
                    for collision in &mut link.collisions {
                        let physics = collision.physics.get_or_insert_with(Default::default);
                        physics.solimp = Some(solimp);
                        // Ensure the foot's impedance wins against the default
                        // stair/ground geom when MuJoCo combines parameters.
                        physics.priority = Some(1);
                        tuned += 1;
                    }
                }
            }
            assert!(tuned > 0, "FOOT_SOLIMP found no foot collision geoms");
            println!("foot contact solimp: {:?} on {tuned} geoms", solimp);
        }
        if !o.uncollide_links.is_empty() {
            let mut hit = 0usize;
            for link in robot.links.iter_mut() {
                if o.uncollide_links.iter().any(|n| link.name.contains(n))
                    && !link.collisions.is_empty()
                {
                    hit += link.collisions.len();
                    link.collisions.clear();
                }
            }
            println!(
                "  [DIAGNOSTIC] dropped {hit} collision geoms from links matching {:?} -- \
                 self-collision involving them can no longer happen, and neither can any real \
                 contact they would have made. This run measures a robot that does not exist.",
                o.uncollide_links
            );
        }
        robot.rebuild_misarta_model();
        let q_seed: Vec<f64> = robot.joint_positions.clone();
        assert!(
            o.armature.is_finite() && o.armature >= 0.0,
            "uniform armature must be finite and nonnegative"
        );
        for (pat, val) in &o.joint_armature_overrides {
            assert!(
                !pat.is_empty() && robot.joints.iter().any(|j| j.name.contains(pat)),
                "armature override pattern {pat:?} matches no joint"
            );
            assert!(
                val.is_finite() && *val >= 0.0,
                "armature override {pat:?} must be finite and nonnegative"
            );
        }

        for j in robot.joints.iter_mut() {
            j.actuator_mode = ActuatorMode::Position;
            j.actuator_kp = o.burnin_kp;
            j.actuator_kv = o.burnin_kv;
            j.joint_damping = o.joint_damping;
            for (pat, val) in &o.joint_damping_overrides {
                if j.name.contains(pat.as_str()) {
                    j.joint_damping = *val;
                }
            }
            j.armature = o.armature;
            for (pat, val) in &o.joint_armature_overrides {
                if j.name.contains(pat.as_str()) {
                    j.armature = *val;
                    println!("  armature {} = {:.6} kg*m^2", j.name, j.armature);
                }
            }
            // Scale the EFFORT ITSELF, so the MJCF export (plant forcerange)
            // and the QP torque box below both see the scaled value. The
            // previous form scaled only the QP box: above 1.0 the QP was
            // allowed to command torque MuJoCo then clipped, so the
            // "stronger motors" diagnostic measured nothing -- measured,
            // TORQUE_SCALE=3 changed a stair-mount failure by 5 ms.
            j.effort *= o.torque_scale;
        }

        // ── Spawn so the soles just touch: measured, not hand-derived ──
        const SOLE_CLEARANCE: f64 = 0.001;
        let native_servo = o.ctrl_mode == CtrlMode::Servo;
        let mu_ground = o.mu_ground;
        let sim_dt = o.sim_dt;
        let run_kv = o.run_kv;
        let extra_wb = o.extra_worldbody.clone();
        let make_opts = move |z: f64| MjcfExportOptions {
            base_pos: Some([0.0, 0.0, z]),
            extra_worldbody: extra_wb.clone(),
            ground_plane: Some(GroundPlaneCfg {
                z: 0.0,
                half_size: 2.0,
                roll: 0.0,
                pitch: 0.0,
            }),
            timestep: Some(sim_dt),
            default_friction: [mu_ground, 0.005, 0.0001],
            // CTRL_MODE=servo hands the velocity loop to MuJoCo itself rather
            // than computing it in Rust, which is what lifts the kv ceiling.
            native_velocity_servo: if native_servo { Some(run_kv) } else { None },
            integrator: None,
            ..MjcfExportOptions::default()
        };
        let spawn_z = {
            let probe = MujocoSim::new(&robot, make_opts(o.probe_z)).expect("probe sim");
            let f = probe.body_world_position(prof.foot_links[0]).expect("foot")[2];
            o.probe_z - ((f - prof.sole_below_origin) - SOLE_CLEARANCE) + o.spawn_z_offset
        };
        {
            // Prove the base is genuinely FREE and the ground is real, rather
            // than trusting MjcfExportOptions::default(). Several sibling
            // examples deliberately weld the torso (base_locked_axes:
            // [true; 6]) and it would be easy to confuse a suspended rig's
            // result for a standing one.
            let xml = crate::mjcf::export_mjcf_with_options(&robot, make_opts(spawn_z));
            if let Ok(path) = std::env::var("RIG_MJCF_OUT") {
                std::fs::write(path, &xml).expect("write runtime MJCF");
            }
            let base_free = xml.contains("<freejoint/>");
            let has_ground = xml.contains(r#"type="plane""#);
            println!("rig check: freejoint={base_free}  ground_plane={has_ground}");
            assert!(
                base_free,
                "base is NOT free -- this would be a suspended rig, not standing"
            );
            assert!(
                has_ground,
                "no ground plane -- the feet would have nothing to push on"
            );
        }
        let sim = MujocoSim::new(&robot, make_opts(spawn_z)).expect("MujocoSim::new");
        let mj_dt = sim.timestep();

        // The forearms used to sit geometrically INSIDE the hip blocks
        // (shoulder at y = +-0.08, 35 mm forearm, hip actuator reaching
        // y = 0.095): 16 contacts and 37.2 kN at the spawn pose, 570x body
        // weight, present in every tick of every run. It braced the robot --
        // single-leg stance "passed" only because of it. Shoulders moved to
        // +-0.115; assert it stays gone, because no number below means
        // anything while the robot is fighting itself.
        {
            let hits: Vec<String> = sim
                .contacts()
                .into_iter()
                .filter(|c| !c.body1.is_empty() && !c.body2.is_empty())
                .map(|c| format!("{} <-> {}", c.body1, c.body2))
                .collect();
            assert!(
                hits.is_empty(),
                "self-collision at spawn ({}): {:?}",
                hits.len(),
                hits
            );
        }

        let (model, a2m, link_to_idx) = build_floating_base_model(&robot);
        let nv = model.nv;
        let na_count = nv - 6;
        let left_foot_mi = *link_to_idx
            .get(prof.foot_links[0])
            .unwrap_or_else(|| panic!("no link {}", prof.foot_links[0]));
        let right_foot_mi = *link_to_idx
            .get(prof.foot_links[1])
            .unwrap_or_else(|| panic!("no link {}", prof.foot_links[1]));
        // P2's target body: the FreeFlyer itself unless the profile names another.
        let trunk_mi = match prof.trunk_link {
            None => 1usize,
            Some(n) => *link_to_idx.get(n).unwrap_or_else(|| panic!("no link {n}")),
        };
        let trunk_from_base = prof.trunk_link.is_none();

        let mass_links: Vec<MassLink> = robot
            .links
            .iter()
            .filter(|l| l.inertial.mass > 0.0)
            .filter_map(|l| {
                link_to_idx.get(&l.name).map(|&mi| {
                    let o = l.inertial.origin.translation.vector;
                    MassLink {
                        mi,
                        m: l.inertial.mass,
                        com_local: na::Vector3::new(o.x as f64, o.y as f64, o.z as f64),
                    }
                })
            })
            .collect();
        let total_mass: f64 = mass_links.iter().map(|l| l.m).sum();
        println!(
            "centroidal model: nv={nv} na={na_count} mass_links={} total_mass={total_mass:.3} kg  dt={mj_dt}",
            mass_links.len()
        );

        // NaN, not a number: 6.0 was kyo46rs's small-joint effort, so any
        // actuated row the loop below failed to reach kept a plausible-looking
        // limit from the wrong robot instead of failing loudly.
        let mut torque_max = na::DVector::from_element(na_count, f64::NAN);
        let mut q_min = na::DVector::from_element(na_count, f64::NAN);
        let mut q_max = na::DVector::from_element(na_count, f64::NAN);
        let mut v_max = na::DVector::from_element(na_count, f64::NAN);
        for ji in 0..robot.joints.len() {
            let Some(mi) = a2m[ji] else { continue };
            if model.joints[mi].joint_type.nv() != 1 {
                continue;
            }
            let vi = model.v_idx[mi];
            if vi >= 6 {
                // Effort already carries torque_scale (applied at build).
                torque_max[vi - 6] = robot.joints[ji].effort.max(1.0);
                q_min[vi - 6] = robot.joints[ji].lower;
                q_max[vi - 6] = robot.joints[ji].upper;
                v_max[vi - 6] = robot.joints[ji].velocity;
            }
        }

        // The QP's torque box and MuJoCo's actuator forcerange must be the
        // same numbers -- both come from the URDF effort, but only if every
        // actuated row was actually written.
        {
            let bad: Vec<String> = (0..na_count)
                .filter(|&i| !torque_max[i].is_finite())
                .map(|i| format!("row {i}"))
                .collect();
            assert!(
                bad.is_empty(),
                "torque box left unset on: {}",
                bad.join(", ")
            );
            let mut lims: Vec<(String, f64)> = Vec::new();
            for ji in 0..robot.joints.len() {
                let Some(mi) = a2m[ji] else { continue };
                if model.joints[mi].joint_type.nv() != 1 {
                    continue;
                }
                let vi = model.v_idx[mi];
                if vi >= 6 {
                    lims.push((robot.joints[ji].name.clone(), robot.joints[ji].effort));
                }
            }
            let mut uniq: Vec<f64> = lims.iter().map(|(_, e)| *e).collect();
            uniq.sort_by(|a, b| a.partial_cmp(b).unwrap());
            uniq.dedup();
            println!(
                "  torque limits (QP box == MuJoCo forcerange, from URDF effort): {:?} N*m over {} joints",
                uniq,
                lims.len()
            );
        }

        // ── Settle with the base WELDED, then hand a clean pose over ───
        // A free-standing position-controlled biped is laterally unstable:
        // it has no balance control, so any numerical asymmetry grows and the
        // whole robot slides sideways. Settling in place therefore lands the
        // CoM somewhere arbitrary -- measured across burn-in lengths of
        // 0.3/0.6/0.9/1.2/1.6 s it drifted to com_y between -0.024 and
        // -0.090 m, and since the lateral support only spans +-0.089 m the
        // run's survival flipped with it. That makes every downstream
        // comparison a coin toss rather than a measurement.
        //
        // The model is symmetric, so the correct settled state is symmetric.
        // Split the two problems: settle the JOINTS against the floor with
        // the base held (the well-posed half), then start the real free-base
        // run from those angles, centred. Same two-pass idea as the spawn
        // probe above.
        let mut robot = robot;
        {
            let mut settle_robot = robot.clone();
            let settle_opts = MjcfExportOptions {
                base_locked_axes: [true; 6],
                ..make_opts(spawn_z)
            };
            let mut settle = MujocoSim::new(&settle_robot, settle_opts).expect("settle sim");
            for ji in 0..settle_robot.joints.len() {
                settle.set_position_target(ji, q_seed[ji]);
            }
            {
                let fz0 = settle.body_world_position(prof.foot_links[0]).unwrap()[2];
                let nc0 = settle.contacts().len();
                println!(
                    "  burn-in t=0: foot z={:.4} (sole {:+.4} vs floor), contacts={nc0}",
                    fz0,
                    fz0 - prof.sole_below_origin
                );
                // The free-base spawn has had a self-collision assert since
                // the kyo46rs forearm/hip brace; this rig did not, and G1
                // walked straight through the gap -- 241.9 kN of pelvis-cover
                // against hip link, 722x body weight, on every burn-in tick.
                // A guard that covers one of two sims is not a guard.
                let hits: Vec<String> = settle
                    .contacts()
                    .iter()
                    .filter(|c| !c.body1.is_empty() && !c.body2.is_empty())
                    .map(|c| format!("{} <-> {} ({:.0} N)", c.body1, c.body2, c.force_mag))
                    .collect();
                assert!(
                    hits.is_empty(),
                    "self-collision in the burn-in rig -- the robot is braced against itself, \
                     and nothing measured after this means anything:\n  {}",
                    hits.join("\n  ")
                );
            }
            settle.step_n_frames(&mut settle_robot, (o.burnin_s / mj_dt) as u32, true);
            {
                for c in settle.contacts().iter().take(12) {
                    println!(
                        "      contact {:>28} <-> {:<28} |f|={:.1} N",
                        if c.body1.is_empty() {
                            "WORLD"
                        } else {
                            &c.body1
                        },
                        if c.body2.is_empty() {
                            "WORLD"
                        } else {
                            &c.body2
                        },
                        c.force_mag
                    );
                }
                let fz1 = settle.body_world_position(prof.foot_links[0]).unwrap()[2];
                let nc1 = settle.contacts().len();
                let f: f64 = settle.contacts().iter().map(|c| c.force_world[2]).sum();
                println!(
                    "  burn-in end: foot z={:.4} (sole {:+.4}), contacts={nc1}, sum fz={f:.1} N (weight {:.1} N)",
                    fz1,
                    fz1 - prof.sole_below_origin,
                    total_mass * G
                );
            }
            for ji in 0..robot.joints.len() {
                robot.joint_positions[ji] = settle_robot.joint_positions[ji];
            }
            // Name the joint, not just the number. A big number on an arm is
            // cosmetic; the same number on a stance knee means the plant
            // cannot hold the seed pose and nothing measured afterwards is
            // worth much.
            let (wi, worst) = (0..robot.joints.len())
                .map(|ji| (ji, (robot.joint_positions[ji] - q_seed[ji]).abs()))
                .fold((0, 0.0_f64), |acc, x| if x.1 > acc.1 { x } else { acc });
            println!(
                "settled (base welded) for {}s: max joint move from seed = {worst:.4} rad  ({})",
                o.burnin_s, robot.joints[wi].name
            );
            let mut moved: Vec<(f64, &str)> = (0..robot.joints.len())
                .map(|ji| {
                    (
                        (robot.joint_positions[ji] - q_seed[ji]).abs(),
                        robot.joints[ji].name.as_str(),
                    )
                })
                .filter(|(d, _)| *d > 0.01)
                .collect();
            moved.sort_by(|a, b| b.0.partial_cmp(&a.0).unwrap());
            for (d, n) in moved.iter().take(6) {
                println!("    {n:32} {:+.4} rad", d);
            }
        }
        if o.ctrl_mode != CtrlMode::Torque {
            // Run-time gains, not the burn-in ones. This applied only to
            // `hybrid` at first, which silently left velocity mode on the
            // burn-in kv and made a sweep from 5 to 1000 return byte-identical
            // results -- the third time in this session that a knob failed to
            // reach the plant. Hence the print below: the effective
            // configuration is now stated, not assumed.
            for j in robot.joints.iter_mut() {
                j.actuator_kp = o.run_kp;
                j.actuator_kv = o.run_kv;
            }
        }
        let hybrid = o.ctrl_mode == CtrlMode::Hybrid;
        println!(
            "actuation: mode={} kp={} kv={}",
            o.ctrl_mode.name(),
            if hybrid { o.run_kp } else { 0.0 },
            if hybrid || o.ctrl_mode.is_velocity() {
                o.run_kv
            } else {
                0.0
            }
        );
        // Re-spawn free-based at the settled pose, centred at the origin.
        let mut sim = MujocoSim::new(&robot, make_opts(spawn_z)).expect("MujocoSim::new (run)");
        for ji in 0..robot.joints.len() {
            sim.set_position_target(ji, robot.joint_positions[ji]);
        }
        // Brief hold so the contacts engage before torque control starts.
        sim.step_n_frames(&mut robot, (0.05 / mj_dt) as u32, true);
        for j in robot.joints.iter_mut() {
            // Hybrid keeps Position mode -- that arm is the only one that
            // reads the position/velocity targets and the torque feedforward
            // at all. Switching everything to Torque here silently made every
            // hybrid gain a no-op, which is why a sweep from kp=0 to kp=2000
            // moved survival by 0.04 s.
            j.actuator_mode = match o.ctrl_mode {
                CtrlMode::Hybrid => ActuatorMode::Position,
                CtrlMode::Velocity | CtrlMode::Servo => ActuatorMode::Velocity,
                CtrlMode::Torque => ActuatorMode::Torque,
            };
        }
        {
            let hr = |n: &str| sim.joint_q_qd(n).map(|(q, _)| q).unwrap_or(f64::NAN);
            let lp = sim.body_world_position(prof.foot_links[0]).unwrap();
            let rp = sim.body_world_position(prof.foot_links[1]).unwrap();
            let rpy = sim
                .body_world_orientation(&robot.root_link)
                .unwrap()
                .euler_angles();
            println!(
                "post-burn-in: rpy=({:+.3},{:+.3},{:+.3}) hip_roll=({:+.3},{:+.3}) foot inner-gap={:+.4}",
                rpy.0,
                rpy.1,
                rpy.2,
                hr(prof.hip_roll[0]),
                hr(prof.hip_roll[1]),
                (lp[1] - 0.019) - (rp[1] + 0.019),
            );
        }

        BipedRig {
            prof,
            opts_ctrl_mode: o.ctrl_mode,
            robot,
            sim,
            model,
            a2m,
            link_to_idx,
            foot_mi: [left_foot_mi, right_foot_mi],
            trunk_mi,
            trunk_from_base,
            mass_links,
            total_mass,
            torque_max,
            q_min,
            q_max,
            v_max,
            q_seed,
            nv,
            na: na_count,
            mj_dt,
            spawn_z,
            armature: o.armature,
            joint_damping: o.joint_damping,
            qd_clamp: std::env::var("QD_CLAMP")
                .ok()
                .and_then(|v| v.parse().ok())
                .unwrap_or(5.0),
            sense_delay: sense_env("SENSE_DELAY_TICKS", 0.0).max(0.0) as usize,
            sense_noise: [
                sense_env("SENSE_NOISE_Q", 0.0).max(0.0),
                sense_env("SENSE_NOISE_QD", 0.0).max(0.0),
                sense_env("SENSE_NOISE_V", 0.0).max(0.0),
                sense_env("SENSE_NOISE_W", 0.0).max(0.0),
            ],
            sense_rng: RefCell::new((sense_env("SENSE_SEED", 1.0) as u64).max(1) ^ 0x9E3779B97F4A7C15),
            sense_buf: RefCell::new(VecDeque::new()),
        }
    }

    /// Advance the plant `n` physics steps, keeping `robot`'s joint state in
    /// sync. Disjoint field borrows make this legal here and awkward at the
    /// call site, which is the only reason it is a method.
    pub fn step(&mut self, n: u32) {
        self.sim.step_n_frames(&mut self.robot, n, true);
    }

    pub fn left_foot_mi(&self) -> usize {
        self.foot_mi[0]
    }
    pub fn right_foot_mi(&self) -> usize {
        self.foot_mi[1]
    }
    /// 0 = left, 1 = right.
    pub fn side_of(&self, foot_mi: usize) -> usize {
        usize::from(foot_mi != self.foot_mi[0])
    }
    /// Weight in newtons.
    pub fn weight(&self) -> f64 {
        self.total_mass * G
    }

    /// World-frame CoM position from an FK snapshot.
    pub fn com_of(&self, data: &misarta::data::Data<f64>) -> na::Vector3<f64> {
        let mut c = na::Vector3::zeros();
        for l in &self.mass_links {
            let r = misarta::se3::rotation_matrix(&data.oMi[l.mi]);
            let o = misarta::se3::translation(&data.oMi[l.mi]);
            c += l.m * (o + r * l.com_local);
        }
        c / self.total_mass
    }

    /// FK at the plant's CURRENT pose, without running a control tick. Used
    /// by the pre-run footprint report and by anything that needs a
    /// world-frame landmark (sole centres for the ZMP plan) before t=0.
    pub fn fk_now(&self) -> misarta::data::Data<f64> {
        let p = self.sim.body_world_position(&self.robot.root_link).unwrap();
        let qq = self
            .sim
            .body_world_orientation(&self.robot.root_link)
            .unwrap();
        let mut q = self.model.neutral_q();
        q[0] = p[0];
        q[1] = p[1];
        q[2] = p[2];
        q[3] = qq.i;
        q[4] = qq.j;
        q[5] = qq.k;
        q[6] = qq.w;
        for ji in 0..self.robot.joints.len() {
            if let Some(mi) = self.a2m[ji] {
                if self.model.joints[mi].joint_type.nq() == 1 {
                    q[self.model.q_idx[mi]] = self.robot.joint_positions[ji];
                }
            }
        }
        misarta::fk::forward_kinematics(&self.model, &q)
    }

    /// Read the plant into misarta's coordinates and build everything the
    /// tasks need from it: `M` (with armature), `h` (with joint damping),
    /// FK, and the centroidal Jacobian with its bias.
    pub fn sync(&self) -> BipedState {
        let sim = &self.sim;
        let robot = &self.robot;
        let model = &self.model;
        let nv = self.nv;

        let mut raw = RawSense {
            body_pos: sim.body_world_position(&robot.root_link).unwrap(),
            body_quat: sim.body_world_orientation(&robot.root_link).unwrap(),
            v_lin_w: sim.body_world_linear_velocity(&robot.root_link).unwrap(),
            v_ang_w: sim.body_world_angular_velocity(&robot.root_link).unwrap(),
            joint_q: robot.joint_positions.clone(),
            joint_qd: robot
                .joints
                .iter()
                .map(|j| sim.joint_q_qd(&j.name).map(|(_, qd)| qd))
                .collect(),
        };
        if self.sense_delay > 0 {
            let mut buf = self.sense_buf.borrow_mut();
            buf.push_back(raw.clone());
            while buf.len() > self.sense_delay + 1 {
                buf.pop_front();
            }
            raw = buf.front().cloned().unwrap_or(raw);
        }
        if self.sense_noise.iter().any(|s| *s > 0.0) {
            let mut rng = self.sense_rng.borrow_mut();
            let [nq, nqd, nv, nw] = self.sense_noise;
            if nq > 0.0 {
                for q in raw.joint_q.iter_mut() {
                    *q += nq * gauss(&mut rng);
                }
            }
            if nqd > 0.0 {
                for qd in raw.joint_qd.iter_mut().flatten() {
                    *qd += nqd * gauss(&mut rng);
                }
            }
            if nv > 0.0 {
                for c in raw.v_lin_w.iter_mut() {
                    *c += nv * gauss(&mut rng);
                }
            }
            if nw > 0.0 {
                for c in raw.v_ang_w.iter_mut() {
                    *c += nw * gauss(&mut rng);
                }
            }
        }
        let body_pos = raw.body_pos;
        let body_quat = raw.body_quat;
        let v_lin_w = raw.v_lin_w;
        let v_ang_w = raw.v_ang_w;
        let r_wb = body_quat.to_rotation_matrix();
        let r_bw = r_wb.transpose();
        let v_lin_body = r_bw * na::Vector3::new(v_lin_w[0], v_lin_w[1], v_lin_w[2]);
        let v_ang_body = r_bw * na::Vector3::new(v_ang_w[0], v_ang_w[1], v_ang_w[2]);

        let mut q = model.neutral_q();
        q[0] = body_pos[0];
        q[1] = body_pos[1];
        q[2] = body_pos[2];
        q[3] = body_quat.i;
        q[4] = body_quat.j;
        q[5] = body_quat.k;
        q[6] = body_quat.w;
        let mut v = vec![0.0_f64; nv];
        // FreeFlyer motion subspace is I6 in the BODY frame with row order
        // [angular; linear] (misarta joint.rs), so v[0..3] is omega_body and
        // v[3..6] is v_body.
        v[0] = v_ang_body.x;
        v[1] = v_ang_body.y;
        v[2] = v_ang_body.z;
        v[3] = v_lin_body.x;
        v[4] = v_lin_body.y;
        v[5] = v_lin_body.z;
        for ji in 0..robot.joints.len() {
            let Some(mi) = self.a2m[ji] else { continue };
            if model.joints[mi].joint_type.nq() == 1 {
                q[model.q_idx[mi]] = raw.joint_q[ji];
            }
            if model.joints[mi].joint_type.nv() == 1 {
                if let Some(qd) = raw.joint_qd[ji] {
                    v[model.v_idx[mi]] = qd.clamp(-self.qd_clamp, self.qd_clamp);
                }
            }
        }
        let v_dvec = na::DVector::from_column_slice(&v);

        let mut mass = misarta::crba::crba(model, &q);
        let mut h = misarta::rnea::nonlinear_effects(model, &q, &v);
        // misarta's dynamics Model carries no rotor inertia and no joint
        // damping -- `armature`/`joint_damping` are MJCF export fields and
        // reach the plant only. So the WBC has been solving a model that
        // differs from the simulator on EVERY actuated joint. Reflected rotor
        // inertia is a diagonal add to M; viscous damping is a velocity term
        // in h. Add both at this boundary so the two agree.
        for ji in 0..robot.joints.len() {
            let Some(mi) = self.a2m[ji] else { continue };
            if model.joints[mi].joint_type.nv() != 1 {
                continue;
            }
            let vi = model.v_idx[mi];
            if vi < 6 {
                continue;
            }
            mass[(vi, vi)] += robot.joints[ji].armature;
            // Per-joint: the plant carries each joint's own damping (uniform
            // value plus any name-matched override), so the compensation
            // must read the same number or the two models diverge exactly
            // on the joints being designed.
            h[vi] += robot.joints[ji].joint_damping * v[vi];
        }
        let data = misarta::fk::forward_kinematics(model, &q);

        // ---- CoM position, Jacobian and bias, all world-frame ----------
        // Per link: take its parent joint's world Jacobian (rows 0..3 =
        // angular, 3..6 = linear, at the JOINT origin) and shift the linear
        // part out to the link's own CoM,
        //     v_c = v_o + omega x r      =>  J_lin_c = J_lin_o - [r]x J_ang
        // then mass-average. The bias picks up the centripetal term:
        //     dJv_lin_c = dJv_lin_o - [r]x dJv_ang + omega x (omega x r)
        let com = self.com_of(&data);
        let mut j_com = na::DMatrix::zeros(3, nv);
        let mut djv_com = na::Vector3::zeros();
        for l in &self.mass_links {
            let rot = misarta::se3::rotation_matrix(&data.oMi[l.mi]);
            let r = rot * l.com_local;
            let skew = na::Matrix3::new(0.0, -r.z, r.y, r.z, 0.0, -r.x, -r.y, r.x, 0.0);
            let j = misarta::jacobian::compute_joint_jacobian_from_data(model, &q, &data, l.mi);
            let j_ang = j.rows(0, 3).into_owned();
            let j_lin = j.rows(3, 3).into_owned();
            let j_lin_c = &j_lin - &skew * &j_ang;
            j_com += l.m * j_lin_c;

            let dj = misarta::jacobian::compute_joint_jacobian_time_derivative(model, &q, &v, l.mi);
            let djv = &dj * &v_dvec;
            let djv_ang = na::Vector3::new(djv[0], djv[1], djv[2]);
            let djv_lin = na::Vector3::new(djv[3], djv[4], djv[5]);
            let omega = &j_ang * &v_dvec;
            let omega = na::Vector3::new(omega[0], omega[1], omega[2]);
            djv_com += l.m * (djv_lin - skew * djv_ang + omega.cross(&omega.cross(&r)));
        }
        j_com /= self.total_mass;
        djv_com /= self.total_mass;

        let com_vel = &j_com * &v_dvec;
        let com_vel = na::Vector3::new(com_vel[0], com_vel[1], com_vel[2]);

        BipedState {
            q,
            v,
            v_dvec,
            data,
            mass,
            h,
            com,
            com_vel,
            j_com,
            djv_com,
            body_pos,
            body_quat,
            v_ang_w,
        }
    }

    /// Gravity-compensation torque on the actuated rows, at this state.
    pub fn gravity_torque(&self, q: &[f64]) -> na::DVector<f64> {
        let g_full = misarta::rnea::compute_gravity(&self.model, q);
        let mut tau_gravity = na::DVector::zeros(self.na);
        for ji in 0..self.robot.joints.len() {
            let Some(mi) = self.a2m[ji] else { continue };
            if self.model.joints[mi].joint_type.nv() != 1 {
                continue;
            }
            let vi = self.model.v_idx[mi];
            if vi >= 6 {
                tau_gravity[vi - 6] = g_full[vi];
            }
        }
        tau_gravity
    }

    /// Iterate `(articara joint index, actuated row index)` over the joints
    /// that map to a single-DoF actuated misarta row. Every loop in the
    /// controller that touches tau, qddot or the posture task walks exactly
    /// this set, and each open-coded copy was a chance to miss a row.
    pub fn actuated(&self) -> Vec<(usize, usize)> {
        let mut out = Vec::with_capacity(self.na);
        for ji in 0..self.robot.joints.len() {
            let Some(mi) = self.a2m[ji] else { continue };
            if self.model.joints[mi].joint_type.nv() != 1 {
                continue;
            }
            let vi = self.model.v_idx[mi];
            if vi >= 6 {
                out.push((ji, vi));
            }
        }
        out
    }

    /// Where the CoM sits inside the footprint, and how much CoP travel that
    /// leaves each way. The fore/aft split of the sole about the ankle only
    /// helps if it is matched to which way the robot actually tends to fall;
    /// copying the human 25/75 split assumes a human's stance, where the CoM
    /// sits well forward of the ankle.
    pub fn report_footprint(&self, sole_half_l: f64) {
        let d0 = self.fk_now();
        let com0 = self.com_of(&d0);
        let ankle_x = 0.5
            * (misarta::se3::translation(&d0.oMi[self.foot_mi[0]]).x
                + misarta::se3::translation(&d0.oMi[self.foot_mi[1]]).x);
        let (cx, half) = (self.prof.sole_centre_x, sole_half_l);
        let (back, front) = (ankle_x + cx - half, ankle_x + cx + half);
        println!(
            "footprint: ankle x={ankle_x:+.4}  sole x=[{back:+.4},{front:+.4}]  CoM x={:+.4}",
            com0.x
        );
        println!(
            "  CoM is {:+.1} mm relative to the ankle;  margin back {:.1} mm / front {:.1} mm",
            (com0.x - ankle_x) * 1000.0,
            (com0.x - back) * 1000.0,
            (front - com0.x) * 1000.0
        );
        let centred_cx = com0.x - ankle_x;
        println!(
            "  sole centre that would put the CoM mid-footprint: x = {centred_cx:+.4} (currently {cx:+.4})"
        );
    }
}
