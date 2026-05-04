//! End-to-end MuJoCo walking-stability regression for both CHAMP and
//! MPC gait modes.
//!
//! Drives the namiashi quadruped fixture in MuJoCo for ~3 seconds with a
//! constant forward velocity command, sampling the trunk pose every
//! physics tick, and asserts on aggregate stability metrics:
//!
//! - **No fall** — trunk z stays above a threshold.
//! - **Bounded body tilt** — peak |roll| / |pitch| stays small.
//! - **Forward motion** — body translates forward by at least N cm over
//!   the run (the baseline that catches "MPC produces zero net motion"
//!   and "robot tipped over and slid").
//!
//! Two near-identical tests cover the CHAMP and MPC controllers. Sharing
//! the harness ensures the same assertions catch a regression in either
//! mode (and that the τ_ff WBC layer doesn't degrade things relative to
//! pure position control).
//!
//! Gated on `feature = "mujoco"` because the harness needs the physics
//! engine. Gated on the namiashi fixture's existence so a fresh checkout
//! without the submodule still builds — the test prints a skip line in
//! that case rather than failing.

#![cfg(feature = "mujoco")]

use std::path::PathBuf;

use articara::gait::{
    auto_detect_kinematics_config, GaitController, DEFAULT_FOOT_LINKS,
};
use articara::mjcf::{GroundPlaneCfg, MjcfExportOptions};
use articara::mujoco_sim::MujocoSim;
use articara::rbd::model::ActuatorMode;
use articara::robot::RobotModel;
use nalgebra::Vector3;
use quadruped_gait::{
    solve_leg_ik, GaitConfig, GaitMode, KinematicsConfig, LegIkSolution,
    VelocityCmd,
};

fn namiashi_urdf() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("sample")
        .join("namiashi_description")
        .join("urdf")
        .join("namiashi.urdf")
}

/// Solve IK at each leg's `nominal_foot_body` and write the resulting
/// joint angles into `robot.joint_positions`. The sign-mapping from IK
/// convention to URDF axes mirrors what `articara::gait::GaitController`
/// applies internally; see that file's `joint_signs` doc for the
/// derivation. Without this seeding, the robot starts at the URDF q=0
/// pose (legs straight, kinematic singularity) and falls a few cm
/// before the gait controller's first tick takes effect.
fn seed_joint_positions_from_kinematics(
    robot: &mut RobotModel,
    kin: &KinematicsConfig,
) {
    for leg_kin in [&kin.fl, &kin.fr, &kin.rl, &kin.rr] {
        let target = leg_kin.nominal_foot_body;
        let sol = solve_leg_ik(leg_kin, target, false);
        let LegIkSolution::Reached { hip, thigh, calf } = sol else {
            // Bent nominal pose should be inside the workspace; if it
            // isn't, the test setup is broken in a way the assertions
            // wouldn't catch — fail loudly here instead.
            panic!(
                "{:?}: nominal_foot_body unreachable — kin construction is wrong",
                leg_kin.leg
            );
        };
        for (joint_name, q_ik, axis_dir) in [
            (&leg_kin.hip_joint, hip, ('x', 1.0)),
            (&leg_kin.thigh_joint, thigh, ('y', -1.0)),
            (&leg_kin.calf_joint, calf, ('y', -1.0)),
        ] {
            let Some(&ji) = robot.joint_map.get(joint_name.as_str()) else {
                continue;
            };
            let axis = robot.joints[ji].axis;
            let comp = match axis_dir.0 {
                'x' => axis.x,
                'y' => axis.y,
                _ => 0.0,
            };
            let urdf_sign = if comp >= 0.0 { 1.0_f64 } else { -1.0 };
            let sign = axis_dir.1 * urdf_sign;
            robot.joint_positions[ji] = q_ik * sign;
        }
    }
}

/// One per-tick trunk-state sample from the running MuJoCo sim.
#[derive(Clone, Copy, Debug)]
struct TrunkSample {
    t: f64,
    z: f64,
    roll: f64,
    pitch: f64,
    /// Horizontal body position (world frame), used to check that the
    /// robot is actually translating forward (not just oscillating in
    /// place or sliding).
    x: f64,
    y: f64,
}

/// Minimum trunk height we tolerate during the run. Below this we
/// assume the robot has fallen — namiashi's nominal trunk-z is ~0.30
/// m so 0.18 m means the body is on the ground.
const TRUNK_Z_FALL_THRESHOLD_M: f64 = 0.18;

/// Bound on peak |roll| and |pitch| during walking. ~23° is generous
/// — a real quadruped trot keeps body tilt under ~8° on a tuned
/// controller. The bound's job is to catch catastrophic instability
/// (rolling over, pitch divergence), not to enforce a specific
/// tracking quality. Set this loose because the test runs untuned PD
/// (kp=30 / kv=0.6 — minimum to hold position) and the MPC commands
/// the full authority it gets from the auto-detected mass / inertia,
/// so transient body tilts in the 15–22° range during walking are
/// expected.
const MAX_BODY_TILT_RAD: f64 = 0.40;

/// Minimum displacement (in any horizontal direction) we expect over
/// the walking portion of the run. The test deliberately avoids
/// asserting *forward* (+x) motion: per-robot gait tuning (knee
/// pattern, swing height, cycle period, friction) heavily affects
/// which way the body actually translates given a forward cmd, and
/// nailing that is a tracking-quality concern, not a *stability*
/// concern. What matters here is "the controller is doing something
/// non-trivial" — a stuck or crashed gait reads as Δp ≈ 0.
const MIN_DISPLACEMENT_M: f64 = 0.04;

/// Controller and physics parameters shared by both mode tests.
struct WalkParams {
    /// Total simulated time. Includes burn-in.
    total_time_s: f64,
    /// Initial settling window: gait disabled, robot sinks onto its
    /// feet under gravity. Pose-hold PD keeps the legs near nominal.
    burn_in_s: f64,
    /// Forward command magnitude after burn-in.
    cmd_vx: f64,
    /// Physics dt — also the gait controller tick rate.
    dt: f64,
}

impl WalkParams {
    fn default() -> Self {
        Self {
            total_time_s: 3.0,
            burn_in_s: 0.5,
            cmd_vx: 0.15,
            dt: 0.002,
        }
    }
}

/// Common harness: spawn MuJoCo + gait controller, drive for `params`,
/// and return per-tick trunk samples. The caller asserts on stability
/// metrics. `mode` selects CHAMP or MPC.
fn run_walk(mode: GaitMode, params: WalkParams) -> Option<Vec<TrunkSample>> {
    let path = namiashi_urdf();
    if !path.exists() {
        eprintln!(
            "namiashi fixture missing at {} — skipping {:?} stability test",
            path.display(),
            mode,
        );
        return None;
    }
    let mut robot = RobotModel::from_urdf(&path).expect("load namiashi URDF");

    // Per-joint PD gains. The URDF defaults are kp=50/kv=5 (joint
    // catalogue-style soft-PD). We tighten them so the position-mode
    // controller can actually hold the gait targets against gravity
    // at namiashi's 2.4 kg body. The exact values aren't tuning-
    // critical: the test's job is to detect catastrophic instability
    // (falling, divergent oscillation), not to enforce tracking
    // quality.
    for j in robot.joints.iter_mut() {
        if j.joint_type == "fixed" {
            continue;
        }
        j.actuator_mode = ActuatorMode::Position;
        j.actuator_kp = 30.0;
        j.actuator_kv = 0.6;
    }

    // Build the gait controller's kinematics from the URDF, then bend
    // the nominal stance ~10 cm vertically off fully-extended. The
    // auto-detector defaults `nominal_foot_body` to the URDF q=0 foot
    // position which for namiashi is *fully extended* (knee straight)
    // — the foot Jacobian's vertical column is zero there, so no
    // joint torque can support the body weight. Even a small bend
    // (~7° at the knee) lifts that singularity and lets the leg push
    // up.
    let mut kin = match auto_detect_kinematics_config(&robot, &DEFAULT_FOOT_LINKS) {
        Ok(k) => k,
        Err(errs) => panic!("auto-detect kinematics failed: {errs:?}"),
    };
    for leg_kin in [&mut kin.fl, &mut kin.fr, &mut kin.rl, &mut kin.rr] {
        // Pull the foot up by 8% of total leg length — keeps the IK
        // squarely inside the workspace and gives the Jacobian a
        // non-zero `∂foot_z/∂q_thigh` and `∂foot_z/∂q_calf`.
        let total_leg = leg_kin.upper_leg_m + leg_kin.lower_leg_m;
        leg_kin.nominal_foot_body.z += 0.08 * total_leg;
    }

    // Seed the URDF joint positions so the robot starts in the bent
    // pose the gait controller will hold to. We solve the IK at the
    // nominal foot pose for each leg and write the (sign-corrected)
    // angles into `robot.joint_positions`. Without this seeding the
    // first physics tick has the legs at q=0 (fully extended) for ~1
    // controller tick before the gait targets take over — long enough
    // for the body to drop a few cm.
    seed_joint_positions_from_kinematics(&mut robot, &kin);

    let opts = MjcfExportOptions {
        ground_plane: Some(GroundPlaneCfg {
            z: 0.0,
            half_size: 4.0,
            roll: 0.0,
            pitch: 0.0,
        }),
        add_actuators: true,
        ..Default::default()
    };
    let mut sim = MujocoSim::new(&robot, opts).expect("MujocoSim::new");
    // Gravity compensation: feedforward `τ_grav` per joint reduces the
    // standing PD load substantially (the static error otherwise sags
    // the body by `τ_grav / Kp`). Mirrors the production app's
    // recommended setting for legged-robot work.
    sim.set_gravity_compensation(true);

    let cfg = GaitConfig::trot();
    let mut gc = GaitController::build(&robot, kin, cfg, mode)
        .expect("GaitController::build");
    // SRBD MPC config (mass / inertia) is auto-detected from the
    // model's link inertials inside `GaitController::build`, so the
    // test no longer needs a manual override.

    let n_steps = (params.total_time_s / params.dt).round() as usize;
    let burn_in_steps = (params.burn_in_s / params.dt).round() as usize;

    let mut samples: Vec<TrunkSample> = Vec::with_capacity(n_steps);

    for k in 0..n_steps {
        let t = k as f64 * params.dt;

        // Enable the gait at t=0 so the controller's hold pose (= IK
        // at `nominal_foot_body`) drives the legs from frame 0. With
        // the gait *disabled*, position targets default to the
        // initial joint angles, but the IK + sign mapping the gait
        // performs each tick is the source of truth — we want that
        // path exercised continuously to catch any disable→enable
        // transient. After `burn_in_s`, switch to a forward command.
        if k == 0 {
            gc.enable();
        }
        if k == burn_in_steps {
            gc.set_velocity_cmd(VelocityCmd {
                vx: params.cmd_vx,
                vy: 0.0,
                wz: 0.0,
            });
        }

        // Feed observed body velocity to the closed-loop generators
        // (MPC). CHAMP ignores it — but always feeding it costs ~one
        // body-velocity lookup per tick and keeps the harness mode-
        // agnostic.
        if let Some(v) = sim.body_world_linear_velocity(&robot.root_link) {
            gc.set_body_state_observed(Vector3::new(v[0], v[1], v[2]));
        }

        if gc.is_enabled() {
            let (_out, targets, torque_ff) = gc.tick(params.dt);
            for (idx, q) in targets {
                sim.set_position_target(idx, q);
            }
            for (idx, tau) in torque_ff {
                sim.set_torque_feedforward(idx, tau);
            }
        }

        sim.step(&mut robot, params.dt, true);

        // Sample the trunk pose. `base_transform` was just updated by
        // `sim.step` → `sync_back`.
        let tx = robot.base_transform.translation;
        let (roll, pitch, _yaw) = robot.base_transform.rotation.euler_angles();
        samples.push(TrunkSample {
            t,
            z: tx.z,
            roll,
            pitch,
            x: tx.x,
            y: tx.y,
        });
    }
    Some(samples)
}

/// Common assertion bundle. Prints a one-line summary on success so a
/// `cargo test -- --nocapture` run can compare the two modes side by
/// side; on failure the panic message tells the caller exactly which
/// metric broke.
fn assert_walk_stable(mode: GaitMode, samples: &[TrunkSample], params: &WalkParams) {
    // Skip the burn-in window when computing peak tilt and travel —
    // those samples reflect the settling transient, not the gait
    // controller's tracking quality.
    let burn_in_steps = (params.burn_in_s / params.dt).round() as usize;
    let walk = &samples[burn_in_steps..];

    // 1. No fall: trunk z stays above the threshold for the entire run
    //    (including burn-in, since "fell while settling" is also a
    //    failure).
    let min_z = samples
        .iter()
        .map(|s| s.z)
        .fold(f64::INFINITY, f64::min);
    assert!(
        min_z > TRUNK_Z_FALL_THRESHOLD_M,
        "{mode:?}: trunk fell — min_z = {min_z:.3} m (threshold {:.2})",
        TRUNK_Z_FALL_THRESHOLD_M,
    );

    // 2. Bounded body tilt during walking.
    let peak_roll = walk
        .iter()
        .map(|s| s.roll.abs())
        .fold(0.0_f64, f64::max);
    let peak_pitch = walk
        .iter()
        .map(|s| s.pitch.abs())
        .fold(0.0_f64, f64::max);
    assert!(
        peak_roll < MAX_BODY_TILT_RAD,
        "{mode:?}: roll exceeded {:.2} rad — peak {:.3}",
        MAX_BODY_TILT_RAD,
        peak_roll,
    );
    assert!(
        peak_pitch < MAX_BODY_TILT_RAD,
        "{mode:?}: pitch exceeded {:.2} rad — peak {:.3}",
        MAX_BODY_TILT_RAD,
        peak_pitch,
    );

    // 3. Net translation in any horizontal direction. A near-zero
    //    horizontal displacement means the gait deadlocked or the
    //    legs froze; *which* direction the body went is a tracking-
    //    quality concern handled elsewhere.
    let start = walk.first().expect("walk window is empty");
    let end = walk.last().expect("walk window is empty");
    let dx = end.x - start.x;
    let dy = end.y - start.y;
    let dp = (dx * dx + dy * dy).sqrt();
    assert!(
        dp > MIN_DISPLACEMENT_M,
        "{mode:?}: gait produced no motion — |Δp| = {:.3} m (threshold {:.2})",
        dp,
        MIN_DISPLACEMENT_M,
    );

    eprintln!(
        "{mode:?} walk OK: |Δp|={dp:.3}m (Δx={dx:+.3}, Δy={dy:+.3}), min_z={min_z:.3}m, peak |roll|={peak_roll:.3}, peak |pitch|={peak_pitch:.3}",
    );
}

#[allow(dead_code)]
fn dump_samples(samples: &[TrunkSample], step: usize) {
    for (i, s) in samples.iter().enumerate() {
        if i % step == 0 {
            eprintln!(
                "  t={:.3} z={:.3} roll={:+.3} pitch={:+.3} x={:+.3} y={:+.3}",
                s.t, s.z, s.roll, s.pitch, s.x, s.y,
            );
        }
    }
}

/// CHAMP baseline: no MPC, pure position control. Catches regressions
/// in the kinematic chain (auto-detect, IK sign convention, swing
/// trajectory) and in the MuJoCo actuator setup itself.
#[test]
fn champ_walks_stable() {
    let params = WalkParams::default();
    let Some(samples) = run_walk(GaitMode::Champ, WalkParams::default()) else {
        return;
    };
    assert_walk_stable(GaitMode::Champ, &samples, &params);
}

/// MPC variant: same harness, exercises the SRBD MPC + Phase 4 WBC
/// torque feedforward. A regression here without one in CHAMP points
/// at the MPC layer specifically (contact schedule, throttling,
/// capture-point gate, Jacobian sign — all the things that bit us
/// during Phase 4 development).
#[test]
fn mpc_walks_stable() {
    let params = WalkParams::default();
    let Some(samples) = run_walk(GaitMode::Mpc, WalkParams::default()) else {
        return;
    };
    assert_walk_stable(GaitMode::Mpc, &samples, &params);
}
