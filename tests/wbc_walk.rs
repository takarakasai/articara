//! End-to-end MuJoCo regression for the Hierarchical WBC pipeline.
//!
//! Mirrors [`gait_walk_stability`](./gait_walk_stability.rs)'s harness
//! (URDF → MujocoSim → trot loop) but routes the gait controller's
//! output through [`articara::wbc_pipeline::WbcPipeline`] instead of
//! the per-joint Position-PD path. Asserts on two invariants:
//!
//! 1. **Gravity balance** — at hold (zero command), the sum of
//!    contact-force z-components should approximately equal `m·g`.
//!    The WBC's friction-cone + EoM tasks are responsible for this:
//!    if the QP fails or hands over a wrong-sign GRF, the body would
//!    fall through the floor or float.
//! 2. **Forward motion** — under a steady forward command, the trunk
//!    must translate by at least `MIN_DISPLACEMENT_M` over the run.
//!    Catches the "WBC bobs in place" regression where the
//!    direction-preserving floor in `compute_mpc_footstep` collapses
//!    to zero or the WBC output is dominated by gravity comp.
//!
//! ## Known limitations (documented per-test)
//!
//! - The WBC pipeline keeps the misarta floating base at neutral
//!   orientation each tick (`q[3..7] = identity`). On flat ground
//!   that approximation is fine; large body tilt would skew the
//!   gravity-comp direction. We use mild commands so the body stays
//!   roughly upright.
//! - Friction enforcement happens in the WBC's QP, but the actual
//!   MuJoCo contact solver may still allow micro-slip. Thresholds
//!   are loose enough to absorb that.

#![cfg(feature = "mujoco")]

use std::path::PathBuf;

use articara::gait::{
    auto_detect_centroidal_mpc_config, auto_detect_kinematics_config,
    GaitController, DEFAULT_FOOT_LINKS,
};
use articara::mjcf::{GroundPlaneCfg, MjcfExportOptions};
use articara::mujoco_sim::MujocoSim;
use articara::rbd::model::ActuatorMode;
use articara::robot::RobotModel;
use articara::wbc_pipeline::WbcPipeline;
use nalgebra::Vector3;
use quadruped_gait::wbc;
use quadruped_gait::{
    solve_leg_ik, ContactDrivenPhase, GaitConfig, GaitMode, GaitType,
    KinematicsConfig, LegIkSolution, VelocityCmd,
};

fn namiashi_misa() -> PathBuf {
    namiashi_misa_named("namiashi.misa")
}

/// The model every test runs on unless it says otherwise.
///
/// Both 3.3 kg variants also carry the knee's 9:14 reduction referred through
/// properly (see `rescale_mass.py`); `namiashi.misa` does not, so any
/// comparison against it mixes the mass change with the gearing change. The
/// comparison that matters -- `hip` vs `prop` -- is unaffected, since both
/// have it.
///
/// `prop` rather than `hip` on purpose. Both are 3.3 kg with 600 g legs; they
/// differ only in how far down the leg the added mass sits, and the spec does
/// not say. `prop` keeps the CAD's own proportions, so it is the variant that
/// assumes nothing, and it is the pessimistic one -- it is the model on which
/// Walk showed a failure band. Tuning against it means the tuning still holds
/// if the real robot turns out to be closer to `hip`; the reverse is not true.
const DEFAULT_MISA: &str = "namiashi_3p3_prop.misa";

/// The shipped `namiashi.misa` came from a CAD export totalling 2.400 kg with
/// 36% of that in the legs. The built robot is 3.3 kg with 600 g per leg --
/// 73% in the legs. `rescale_mass.py` in the same directory regenerates the
/// corrected models; `_hip` puts the extra mass at the hip where the motors
/// are, `_prop` spreads it along the leg as the CAD distribution implies.
fn namiashi_misa_named(file: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("tests")
        .join("fixtures")
        .join("namiashi")
        .join(file)
}

/// Same seeding logic as `gait_walk_stability`. Keeps the legs out of
/// their q=0 fully-extended kinematic singularity at sim start.
fn seed_joint_positions_from_kinematics(
    robot: &mut RobotModel,
    kin: &KinematicsConfig,
) {
    for leg_kin in [&kin.fl, &kin.fr, &kin.rl, &kin.rr] {
        let target = leg_kin.nominal_foot_body;
        let sol = solve_leg_ik(leg_kin, target, false);
        let LegIkSolution::Reached { hip, thigh, calf } = sol else {
            panic!(
                "{:?}: nominal_foot_body unreachable",
                leg_kin.leg
            );
        };
        for (joint_name, q_ik, sign) in [
            (&leg_kin.hip_joint, hip, 1.0),
            (&leg_kin.thigh_joint, thigh, -1.0),
            (&leg_kin.calf_joint, calf, -1.0),
        ] {
            let Some(&ji) = robot.joint_map.get(joint_name.as_str()) else {
                continue;
            };
            robot.joint_positions[ji] = q_ik * sign;
        }
    }
}

/// Per-tick sample. We track `total_fz_world` (Σ contact-force z over
/// all contacts) so the static-balance test can compare it with
/// `m · g` after the burn-in window.
#[derive(Debug, Clone, Copy)]
struct WbcSample {
    t: f64,
    body_x: f64,
    body_z: f64,
    roll: f64,
    pitch: f64,
    total_fz_world: f64,
    /// Per-foot normal force (N), FL/FR/RL/RR. Contact state is what
    /// distinguishes a gait that is walking from one that is shuffling, and
    /// the original four tests could not see the difference -- they checked
    /// trunk height and net displacement only.
    foot_fz: [f64; 4],
    body_y: f64,
    yaw: f64,
    /// Carried per-sample only so `report_walk_cmd` can size its averaging
    /// window in whole gait cycles without every call site threading it.
    cycle_period_s: f64,
    /// |applied torque| / that joint's `effort` limit, per joint. A joint at
    /// 1.0 is clamped: the commanded torque is not the torque being produced,
    /// and the modelled controller is not the controller running.
    tau_frac: [f64; 12],
    /// |joint velocity| / that joint's rated `velocity`. The knee's rating
    /// drops to 21.5 rad/s once its 9:14 reduction is referred properly, and
    /// `mujoco_sim` brakes overspeed with a torque added *before* the effort
    /// clamp -- so a joint that is too fast shows up as a joint that is out
    /// of torque, which is a different problem with a different fix.
    qd_frac: [f64; 12],
}

/// Threshold for "trunk has fallen". Below this z, the body has either
/// tipped over or sunk through the ground. Same value as
/// `gait_walk_stability`.
const TRUNK_Z_FALL_THRESHOLD_M: f64 = 0.18;

/// Which actuator interface the leg joints are driven through.
///
/// The target hardware for namiashi is the LKMTech MG4005, which has no MIT
/// mode: the CAN protocol offers closed-loop position, closed-loop speed, and
/// closed-loop iq. So the position-plus-torque-feedforward path this file has
/// used throughout -- which is what a Unitree-style `(q, dq, kp, kd, tau)`
/// frame gives you -- is not available as such.
///
/// Speed mode is the first choice over iq for a reason worth writing down:
/// torque mode puts the entire stabilising loop on the host, so host rate and
/// bus latency set the stability margin directly. Position and speed modes
/// close a fast inner loop inside the driver, and the host only has to be
/// fast enough to shape the trajectory.
#[derive(Debug, Clone, Copy, PartialEq)]
enum Actuation {
    /// Joint position target plus the WBC's torque feedforward -- everything
    /// in this file before the MG4005 was chosen. Not available on that part.
    PositionTorque,
    /// Joint velocity command only, as a speed-mode driver takes.
    ///
    /// `qd_des = dq*/dt + k_track * (q* - q)`. The feedforward alone would
    /// leave position free to drift, since a speed loop has no position
    /// feedback of its own; `k_track` is the outer position loop the host has
    /// to supply. `loop_kv` stands in for the driver's own speed-loop
    /// stiffness -- the model's `actuator_kv` of 1.2 is a position-mode
    /// damping value and would need 1.25 rad/s of error to reach the 1.5 N*m
    /// limit, which is not what a real speed loop does.
    Velocity { k_track: f64, loop_kv: f64, loop_ki: f64 },
    /// Speed mode with the driver modelled as an ideal velocity source,
    /// limited only by torque. The right abstraction for a loop that closes
    /// at 8-16 kHz against a host at a few hundred: from the host's side it
    /// tracks whatever it is asked for, until the motor runs out.
    VelocityIdeal { k_track: f64 },
    /// Raw joint torque, as a closed-loop-iq driver takes. The driver holds no
    /// position or velocity loop at all, so the host has to supply the whole
    /// thing: `tau = kp*(q* - q) + kd*(0 - qd) + tau_wbc`.
    ///
    /// Numerically identical to the position path if the host computes that
    /// PD at the same rate -- which is the point. What differs on hardware is
    /// *where* the loop runs. A speed-mode driver closes its inner loop
    /// internally on fresh encoder data at several kHz whatever the host is
    /// doing; in torque mode there is no inner loop, and the last torque the
    /// host sent is held until the next one arrives. `WbcParams::host_rate_hz`
    /// is what makes that difference visible.
    Torque { kp: f64, kd: f64 },
}

/// What the controller is told about its own velocity.
#[derive(Debug, Clone, Copy, PartialEq)]
enum VelObs {
    /// Simulator ground truth -- what every result in this file was measured
    /// with, and what no robot has.
    Truth,
    /// Nothing. Both the MPC's state feedback and the WBC's SRBD state see a
    /// stationary robot.
    Zero,
    /// The commanded velocity, i.e. open loop: no velocity sensing at all,
    /// the controller simply believes it is doing what it was asked.
    Command,
    /// Truth plus a constant error, in m/s. Stands in for the drift an
    /// IMU-only estimate accumulates.
    Bias(f64, f64),
    /// Truth delayed by this many seconds.
    Lag(f64),
}

/// How far below the harness's original nominal stance every run now sits.
///
/// The file spent its whole history at one height, ~0.295 m, and
/// `namiashi_trunk_height_sweep` shows that was not a neutral choice. All
/// three gaits walk at 0, 2, 4 and 6 cm of crouch -- tracking stays 98-103%
/// and trunk z lands within a millimetre or two of target -- but crouching
/// rotates the leg Jacobian, so the same foot force gets split differently
/// between thigh and knee:
///
///     Trot   thigh 26.7 -> 10.7 %    calf  6.0 -> 17.0 %
///     Walk   thigh  6.3 ->  5.3 %    calf 11.3 ->  1.9 %
///     Crawl  thigh  3.4 ->  2.1 %    calf  9.8 ->  0.5 %
///
/// 6 cm is the baseline because the thigh is the joint that actually binds --
/// it was clamped at its 1.5 N*m limit for a quarter of every Trot run, and
/// `mujoco_sim.rs:1121` does that silently -- and crouching more than halves
/// it. Walk and Crawl get it nearly free; their knee saturation all but
/// disappears and nothing else moves.
///
/// Trot pays for it. Its knee saturation nearly triples, and peak roll goes
/// from 2.8 deg to 5.3. That is a deliberate trade, not an oversight: 17% on
/// a 2.3 N*m knee is a better place to be than 27% on a 1.5 N*m thigh.
const NAMIASHI_STANCE_DROP_M: f64 = 0.06;

/// Minimum forward displacement during the walking window — the same
/// 4 cm threshold the gait stability test uses.
const MIN_DISPLACEMENT_M: f64 = 0.04;

/// How much of the tail of a static-stand run the gravity-balance average
/// covers.
const STATIC_AVG_WINDOW_S: f64 = 0.5;

struct WbcParams {
    total_time_s: f64,
    burn_in_s: f64,
    cmd_vx: f64,
    cmd_vy: f64,
    cmd_wz: f64,
    /// Model file under `tests/fixtures/namiashi/`.
    misa_file: &'static str,
    /// Early-touchdown force at which `ContactDrivenPhase` overrides the
    /// nominal phase, newtons. The harness has always used 5.0. Heavier legs
    /// hit harder, so on the 3.3 kg models this threshold fires on contacts
    /// the 2.4 kg model never produced.
    early_contact_n: f64,
    /// Diagnostic: drop the WBC's torque and the MPC's GRF reference, leaving
    /// only the joint position-PD tracking the IK targets. If the measured
    /// gait does not change, the dynamic layers are not contributing.
    kinematic_only: bool,
    /// Give the WBC the robot it is actually driving. `WbcPipeline::new`
    /// hardcodes `mass_kg: 9.0` and a 9 kg inertia diagonal; namiashi is
    /// 3.3 kg. Off by default so the effect can be measured against the
    /// state every earlier result in this file was produced under.
    wbc_real_inertia: bool,
    /// Decomposition of `wbc_real_inertia`, which bundles three changes:
    /// the mass (9.0 -> 3.3, a 2.7x amplification of `a_base_des`), the CoM
    /// offset, and the composite inertia (which also switches the pipeline
    /// onto the CoM-moment-arm accel prediction). Each can be applied alone.
    wbc_real_mass_only: bool,
    wbc_real_com_only: bool,
    wbc_real_inertia_only: bool,
    /// Minimum normal force on commanded-stance feet, as a fraction of the
    /// static per-foot share `m*g/4`. Expressed as a fraction rather than
    /// newtons so it means the same thing on the 2.4 and 3.3 kg models.
    f_min_stance_frac: f64,
    /// Override for `WbcWeights::base_accel` (default 200.0). This is the
    /// largest soft weight in the QP and the one that carries the MPC's
    /// predicted base acceleration into the torque.
    base_accel_weight: Option<f64>,
    /// Override for `WbcWeights::contact_force` (default 5.0) -- how hard the
    /// WBC pulls its own GRF solution toward the MPC's prediction.
    contact_force_weight: Option<f64>,
    /// Write the exact MJCF the sim runs plus a per-tick root-pose and joint
    /// trace here, for `render_namiashi.py` to replay. Purely a side channel;
    /// nothing in the harness reads it back.
    replay_dir: Option<String>,
    /// Lower the nominal stance by this much, metres. Applied to every leg's
    /// `nominal_foot_body.z`, which is what both the IK seed and the MPC's
    /// body-z reference are taken from, so the whole stack follows.
    /// Defaults to [`NAMIASHI_STANCE_DROP_M`].
    trunk_drop_m: f64,
    /// Constant offset added to the base position the WBC observes, and a
    /// per-second drift on top of it. Both are diagnostics for the hardware
    /// question: is a bounded absolute position actually required?
    base_pos_bias_m: [f64; 3],
    base_pos_drift_mps: [f64; 3],
    /// How the observed body velocity is corrupted before the controller sees
    /// it. The hardware question: with only an IMU there is no bounded
    /// velocity estimate to be had -- 1 deg of attitude error leaks
    /// g*sin(1 deg) = 0.17 m/s^2 of phantom horizontal acceleration, which is
    /// 0.85 m/s of velocity error in five seconds against a 0.80 m/s command.
    /// So: how much does this controller actually depend on it?
    vel_obs: VelObs,
    /// Which actuator interface the controller drives.
    actuation: Actuation,
    /// Rate at which the whole host-side controller runs -- gait generator,
    /// WBC, and the command it emits. `None` means every physics tick, which
    /// is what this file has always assumed and no robot on a shared CAN bus
    /// gets. Between updates the last command is held.
    ///
    /// Only rates that divide the physics rate are representable, because the
    /// gate is an integer tick count. At the default `dt` of 0.002 s that is
    /// 500 / 250 / 167 / 125 / 100 Hz and nothing between -- asking for 400
    /// silently gives 500, and asking for 200 silently gives 167. The run
    /// prints what it actually used; set a finer `dt` to reach other rates.
    host_rate_hz: Option<f64>,
    /// Piecewise-constant command schedule: `(t_from, vx, vy, wz)`. Overrides
    /// `cmd_vx/vy/wz` once the first entry's time is reached. For asking how
    /// the two interfaces handle a command that moves, rather than one held
    /// constant for the whole run.
    cmd_schedule: Vec<(f64, f64, f64, f64)>,
    /// World-frame push on the trunk: `(t_start, [fx, fy, fz], duration)`.
    /// A disturbance neither interface plans for, so it is the one test that
    /// asks what happens when the model is wrong.
    push: Option<(f64, [f64; 3], f64)>,
    dt: f64,
    /// `None` = the legacy `wbc::solve_warm_with_weights` path
    /// (walk-validated default). `Some` opts the pipeline into the
    /// misa-wbc `Dynamics`-backed [`WbcSolver`] with the given
    /// formulation/strategy/backend — the equivalence-study switch
    /// documented in `ref/wbc_comparison.md`.
    misa_wbc_mode: Option<(wbc::Formulation, wbc::SolveConfig)>,
    /// Gait family. `None` keeps `GaitConfig::trot()`, which is what the
    /// original four tests ran and the only thing this file had ever
    /// exercised.
    gait_type: Option<GaitType>,
    /// Overrides applied after the family's own defaults. namiashi is 2.400 kg
    /// against Go2's 15.606 with a 0.306 m leg against 0.426, so the numbers
    /// tuned on Go2 do not carry over directly -- under Froude similarity
    /// (T proportional to sqrt(L/g), stride to L) Go2's 0.18 s / 0.20 m
    /// become 0.152 s / 0.143 m here. `None` keeps the family default.
    cycle_period_s: Option<f64>,
    duty_factor: Option<f64>,
    max_step_length_m: Option<f64>,
    /// Swing-foot apex clearance. Crawl's library default is 0.005 m, which
    /// is fine over a 0.06 m step and almost certainly scuffing over a
    /// 0.145 m one.
    swing_height_m: Option<f64>,
    /// Capture-point feedback gain, seconds. The library default is 0.05 s
    /// for every robot; the LIP value it is meant to approximate is
    /// sqrt(h/g), which for namiashi's 0.30 m trunk is 0.175 s.
    k_capture_s: Option<f64>,
}

impl WbcParams {
    fn static_stand() -> Self {
        Self {
            total_time_s: 1.5,
            burn_in_s: 0.5,
            cmd_vx: 0.0,
            dt: 0.002,
            misa_wbc_mode: None,
            gait_type: None, cycle_period_s: None,
            duty_factor: None, max_step_length_m: None,
            swing_height_m: None, k_capture_s: None,
            cmd_vy: 0.0, cmd_wz: 0.0,
            misa_file: DEFAULT_MISA,
            early_contact_n: 5.0,
            kinematic_only: false,
            wbc_real_inertia: false,
            wbc_real_mass_only: false,
            wbc_real_com_only: false,
            wbc_real_inertia_only: false,
            f_min_stance_frac: 0.0,
            base_accel_weight: None,
            contact_force_weight: None,
            replay_dir: None,
            trunk_drop_m: NAMIASHI_STANCE_DROP_M,
            base_pos_bias_m: [0.0; 3],
            base_pos_drift_mps: [0.0; 3],
            vel_obs: VelObs::Truth,
            actuation: Actuation::PositionTorque,
            host_rate_hz: None,
            cmd_schedule: Vec::new(),
            push: None,
        }
    }
    fn forward_walk() -> Self {
        Self {
            total_time_s: 3.0,
            burn_in_s: 0.5,
            cmd_vx: 0.15,
            dt: 0.002,
            misa_wbc_mode: None,
            gait_type: None, cycle_period_s: None,
            duty_factor: None, max_step_length_m: None,
            swing_height_m: None, k_capture_s: None,
            cmd_vy: 0.0, cmd_wz: 0.0,
            misa_file: DEFAULT_MISA,
            early_contact_n: 5.0,
            kinematic_only: false,
            wbc_real_inertia: false,
            wbc_real_mass_only: false,
            wbc_real_com_only: false,
            wbc_real_inertia_only: false,
            f_min_stance_frac: 0.0,
            base_accel_weight: None,
            contact_force_weight: None,
            replay_dir: None,
            trunk_drop_m: NAMIASHI_STANCE_DROP_M,
            base_pos_bias_m: [0.0; 3],
            base_pos_drift_mps: [0.0; 3],
            vel_obs: VelObs::Truth,
            actuation: Actuation::PositionTorque,
            host_rate_hz: None,
            cmd_schedule: Vec::new(),
            push: None,
        }
    }

    /// Same schedule as [`Self::static_stand`], routed through
    /// `WbcSolver` with the given formulation/strategy/backend.
    fn static_stand_misa_wbc(formulation: wbc::Formulation, cfg: wbc::SolveConfig) -> Self {
        Self { misa_wbc_mode: Some((formulation, cfg)), ..Self::static_stand() }
    }

    /// Same schedule as [`Self::forward_walk`], routed through
    /// `WbcSolver` with the given formulation/strategy/backend.
    fn forward_walk_misa_wbc(formulation: wbc::Formulation, cfg: wbc::SolveConfig) -> Self {
        Self { misa_wbc_mode: Some((formulation, cfg)), ..Self::forward_walk() }
    }
}

/// Run a WBC sim, sampling per-tick. Returns `None` if the namiashi
/// fixture is missing (skip cleanly).
fn run_wbc_sim(params: WbcParams) -> Option<Vec<WbcSample>> {
    let path = namiashi_misa_named(params.misa_file);
    if !path.exists() {
        eprintln!(
            "namiashi fixture missing at {} — skipping WBC test",
            path.display()
        );
        return None;
    }
    // Load from the master .misa so PD gains (kp=100, kv=1.2) and
    // joint damping (0.1) come from the same source the GUI uses; the
    // burn-in Position-PD then settles at the real GUI body-z. See
    // commit `f53482c` for the same migration in gait_walk_stability.
    let mut robot = RobotModel::from_misa(&path).expect("load namiashi .misa");

    let mut kin = auto_detect_kinematics_config(&robot, &DEFAULT_FOOT_LINKS)
        .expect("auto-detect kinematics");
    for leg_kin in [&mut kin.fl, &mut kin.fr, &mut kin.rl, &mut kin.rr] {
        let total_leg = leg_kin.upper_leg_m + leg_kin.lower_leg_m;
        leg_kin.nominal_foot_body.z += 0.08 * total_leg;
        // Positive z moves the nominal foot toward the body, i.e. crouches.
        leg_kin.nominal_foot_body.z += params.trunk_drop_m;
    }
    seed_joint_positions_from_kinematics(&mut robot, &kin);

    // Actuator interface. Set before the sim is built so the exported MJCF
    // and the per-tick control law agree.
    if !matches!(params.actuation, Actuation::PositionTorque) {
        // Resolve the twelve leg joints by name -- the gait controller, which
        // would hand over the indices, is not built until after the sim.
        let leg_joint_names: Vec<String> = [&kin.fl, &kin.fr, &kin.rl, &kin.rr]
            .iter()
            .flat_map(|lk| {
                [
                    lk.hip_joint.clone(),
                    lk.thigh_joint.clone(),
                    lk.calf_joint.clone(),
                ]
            })
            .collect();
        for name in &leg_joint_names {
            let Some(&ji) = robot.joint_map.get(name.as_str()) else {
                panic!("velocity path: joint {name:?} not in the model");
            };
            match params.actuation {
                Actuation::Velocity { loop_kv, .. } => {
                    robot.joints[ji].actuator_mode = ActuatorMode::Velocity;
                    robot.joints[ji].actuator_kv = loop_kv;
                }
                Actuation::VelocityIdeal { .. } => {
                    robot.joints[ji].actuator_mode = ActuatorMode::Velocity;
                }
                Actuation::Torque { .. } => {
                    robot.joints[ji].actuator_mode = ActuatorMode::Torque;
                }
                Actuation::PositionTorque => unreachable!(),
            }
        }
    }

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
    // Optional replay export for video rendering. Writes the exact MJCF the
    // sim is running plus a per-tick root-pose + joint trace, so a renderer
    // can replay the run kinematically without re-deriving any of it. Purely
    // a side channel -- nothing below reads it.
    let replay_out = params.replay_dir.clone();
    if let Some(dir) = replay_out.as_deref() {
        std::fs::create_dir_all(dir).expect("create replay dir");
        std::fs::write(
            format!("{dir}/model.xml"),
            articara::mjcf::export_mjcf_with_options(&robot, opts.clone()),
        )
        .expect("write replay model.xml");
    }
    let mut sim = MujocoSim::new(&robot, opts).expect("MujocoSim::new");
    match params.actuation {
        Actuation::Velocity { loop_ki, .. } => sim.velocity_loop_ki = loop_ki,
        Actuation::VelocityIdeal { .. } => sim.velocity_loop_ideal = true,
        _ => {}
    }
    // We don't want gravity-comp to compete with the WBC during the
    // walking window — the WBC's floating-base EoM task already
    // handles it. But during burn-in (gait disabled, WBC inactive),
    // grav-comp keeps the body from sagging. Toggle is per-sim, so
    // we leave it on; the WBC bypasses the per-joint path entirely
    // when active.
    sim.set_gravity_compensation(true);

    let mut cfg = match params.gait_type {
        Some(GaitType::Trot) | None => GaitConfig::trot(),
        Some(GaitType::Walk) => GaitConfig::walk(),
        Some(GaitType::Pace) => GaitConfig::pace(),
        Some(GaitType::Bound) => GaitConfig::bound(),
        Some(GaitType::Crawl) => GaitConfig::crawl(),
    };
    if let Some(v) = params.cycle_period_s {
        cfg.cycle_period_s = v;
    }
    if let Some(v) = params.duty_factor {
        cfg.duty_factor = v;
    }
    if let Some(v) = params.swing_height_m {
        cfg.swing_height_m = v;
    }
    if let Some(v) = params.max_step_length_m {
        cfg.max_step_length_m = v;
    }
    if let Some(hz) = params.host_rate_hz {
        let decim = ((1.0 / hz) / params.dt).round().max(1.0);
        eprintln!(
            "[host] requested {hz:.0} Hz -> decim {decim:.0} -> effective \
             {:.1} Hz (physics {:.0} Hz)",
            1.0 / (params.dt * decim),
            1.0 / params.dt,
        );
    }
    eprintln!(
        "[gait] {:?}  T={:.3}s duty={:.3} max_step={:.3}m  cmd_vx={:.3}  \
         model={} m={:.3}kg",
        cfg.gait_type, cfg.cycle_period_s, cfg.duty_factor,
        cfg.max_step_length_m, params.cmd_vx,
        params.misa_file,
        robot.links.iter().map(|l| l.inertial.mass).sum::<f64>(),
    );
    let cycle_period_s = cfg.cycle_period_s;
    let mut gc = GaitController::build(&robot, kin.clone(), cfg, GaitMode::Mpc)
        .expect("GaitController::build (Mpc mode)");
    if let Some(k) = params.k_capture_s {
        gc.set_capture_point_gain(k);
    }

    // Foot link names for the WBC pipeline.
    let foot_links: [String; 4] = [
        DEFAULT_FOOT_LINKS[0].1.to_string(),
        DEFAULT_FOOT_LINKS[1].1.to_string(),
        DEFAULT_FOOT_LINKS[2].1.to_string(),
        DEFAULT_FOOT_LINKS[3].1.to_string(),
    ];
    let mut wbc_pipeline = WbcPipeline::new(&robot, foot_links);
    if let Some((formulation, cfg)) = params.misa_wbc_mode.clone() {
        wbc_pipeline = wbc_pipeline.with_wbc_solver(formulation, cfg);
    }
    if let Some(w) = params.contact_force_weight {
        wbc_pipeline.weights.contact_force = w;
        eprintln!("[wbc] contact_force weight = {w} (default 5.0)");
    }
    if let Some(w) = params.base_accel_weight {
        wbc_pipeline.weights.base_accel = w;
        eprintln!("[wbc] base_accel weight = {w} (default 200.0)");
    }
    if params.f_min_stance_frac > 0.0 {
        let mg: f64 = robot.links.iter().map(|l| l.inertial.mass).sum::<f64>() * 9.81;
        wbc_pipeline.f_min_stance_n = params.f_min_stance_frac * mg / 4.0;
        eprintln!(
            "[wbc] f_min_stance = {:.2} N ({:.0}% of m*g/4 = {:.2} N)",
            wbc_pipeline.f_min_stance_n,
            100.0 * params.f_min_stance_frac,
            mg / 4.0,
        );
    }
    if params.wbc_real_mass_only || params.wbc_real_com_only || params.wbc_real_inertia_only {
        let c = auto_detect_centroidal_mpc_config(&robot);
        if params.wbc_real_mass_only {
            wbc_pipeline.mass_kg = c.mass_kg;
        }
        if params.wbc_real_com_only {
            wbc_pipeline.com_offset_body = c.com_offset_body;
        }
        if params.wbc_real_inertia_only {
            wbc_pipeline.centroidal_inertia_body = Some(c.centroidal_inertia_body);
        }
        eprintln!(
            "[wbc] partial: mass={} com={} inertia={}",
            params.wbc_real_mass_only,
            params.wbc_real_com_only,
            params.wbc_real_inertia_only,
        );
    }
    if params.wbc_real_inertia {
        // Same source the centroidal MPC config already uses: total mass,
        // aggregate CoM relative to the body root, and the angular block of
        // the composite-rigid-body inertia. Setting `centroidal_inertia_body`
        // also switches the pipeline onto the centroidal accel prediction,
        // which takes moment arms from the CoM instead of the root.
        let c = auto_detect_centroidal_mpc_config(&robot);
        wbc_pipeline.mass_kg = c.mass_kg;
        wbc_pipeline.com_offset_body = c.com_offset_body;
        wbc_pipeline.centroidal_inertia_body = Some(c.centroidal_inertia_body);
        let i = c.centroidal_inertia_body;
        eprintln!(
            "[wbc] real inertia: m={:.3}kg (was 9.000)  com_off=({:+.4},{:+.4},{:+.4})m  \
             I_diag=({:.5},{:.5},{:.5}) (was 0.07000,0.26000,0.24200)",
            c.mass_kg,
            c.com_offset_body.x, c.com_offset_body.y, c.com_offset_body.z,
            i[(0, 0)], i[(1, 1)], i[(2, 2)],
        );
    }

    // Joint names in the order `gc.joint_indices()` reports them, so the
    // replay CSV and the model agree without either side guessing.
    let replay_joint_names: Vec<String> = gc
        .joint_indices()
        .iter()
        .flatten()
        .map(|&ji| robot.joints[ji].name.clone())
        .collect();
    let mut replay_buf = String::new();
    if replay_out.is_some() {
        replay_buf.push_str("t,root_x,root_y,root_z,root_qw,root_qx,root_qy,root_qz");
        for n in &replay_joint_names {
            replay_buf.push_str(&format!(",{n}"));
        }
        // Measured per-foot normal force. A renderer cannot recover this from
        // pose: foot height says whether a foot is touching, not whether it
        // is carrying anything, and the two disagree by up to 0.24 of the
        // cycle on Walk's front pair -- which is precisely the foot that gets
        // unloaded. Writing the force out is cheaper than a proxy that
        // misrepresents the thing being compared.
        for (_, link) in DEFAULT_FOOT_LINKS.iter() {
            replay_buf.push_str(&format!(",fz_{link}"));
        }
        // The command as applied, and whether a push is being delivered.
        // Recorded rather than restated in the renderer: a schedule written
        // out twice is a schedule that will disagree with itself.
        replay_buf.push_str(",cmd_vx,cmd_vy,cmd_wz,push_fy");
        replay_buf.push('\n');
    }

    let n_steps = (params.total_time_s / params.dt).round() as usize;
    let burn_in_steps = (params.burn_in_s / params.dt).round() as usize;
    let mut samples: Vec<WbcSample> = Vec::with_capacity(n_steps);
    let mut v_hist: Vec<[f64; 3]> = Vec::with_capacity(n_steps);
    // Previous tick's IK targets, so the velocity path can differentiate them.
    let mut prev_targets = [0.0_f64; 12];
    // Host-computed PD for the torque path, paired with its joint index.
    let mut host_pd = [(0usize, 0.0_f64); 12];

    for k in 0..n_steps {
        let t = k as f64 * params.dt;

        if k == 0 {
            gc.enable();
        }
        if k == burn_in_steps {
            gc.set_velocity_cmd(VelocityCmd {
                vx: params.cmd_vx,
                vy: params.cmd_vy,
                wz: params.cmd_wz,
            });
        }
        // Command schedule, applied on the tick its segment begins.
        if k >= burn_in_steps {
            if let Some(&(_, vx, vy, wz)) = params
                .cmd_schedule
                .iter()
                .rev()
                .find(|(t_from, ..)| t >= *t_from)
            {
                let now = gc.velocity_cmd();
                if (now.vx - vx).abs() > 1e-9
                    || (now.vy - vy).abs() > 1e-9
                    || (now.wz - wz).abs() > 1e-9
                {
                    gc.set_velocity_cmd(VelocityCmd { vx, vy, wz });
                }
            }
        }
        // Push. Applied once, at its start tick; MuJoCo counts the duration
        // down itself.
        if let Some((t_push, force, dur)) = params.push {
            if k > 0 && t >= t_push && (t - params.dt) < t_push {
                sim.apply_external_force(&robot.root_link, force, [0.0; 3], dur);
            }
        }

        // Feed observed body velocity to the closed-loop generators, after
        // corrupting it the way the chosen `vel_obs` says. Everything
        // downstream -- the gait controller's feedback, the MPC's SRBD state
        // and the WBC -- sees this and only this.
        let v_true = sim
            .body_world_linear_velocity(&robot.root_link)
            .unwrap_or([0.0, 0.0, 0.0]);
        v_hist.push(v_true);
        let v_obs: [f64; 3] = match params.vel_obs {
            VelObs::Truth => v_true,
            VelObs::Zero => [0.0; 3],
            VelObs::Command => {
                // Body-frame command rotated into world by the true yaw.
                let (_, _, yaw_now) = robot.base_transform.rotation.euler_angles();
                let cmd = if k >= burn_in_steps {
                    (params.cmd_vx, params.cmd_vy)
                } else {
                    (0.0, 0.0)
                };
                [
                    yaw_now.cos() * cmd.0 - yaw_now.sin() * cmd.1,
                    yaw_now.sin() * cmd.0 + yaw_now.cos() * cmd.1,
                    0.0,
                ]
            }
            VelObs::Bias(bx, by) => [v_true[0] + bx, v_true[1] + by, v_true[2]],
            VelObs::Lag(secs) => {
                let back = (secs / params.dt).round() as usize;
                let idx = v_hist.len().saturating_sub(1 + back);
                v_hist[idx]
            }
        };
        let w_obs = sim
            .body_world_angular_velocity(&robot.root_link)
            .unwrap_or([0.0, 0.0, 0.0]);
        gc.set_body_state_observed(
            Vector3::new(v_obs[0], v_obs[1], v_obs[2]),
            Vector3::new(w_obs[0], w_obs[1], w_obs[2]),
        );

        // Host-rate gate. On a host tick the controller runs and emits a new
        // command; between them the sim keeps whatever was last written,
        // which is what a driver does with a stale command. The gait
        // generator is advanced by the host period, not the physics one, so
        // the gait keeps real time.
        let host_decim = params
            .host_rate_hz
            .map(|hz| ((1.0 / hz) / params.dt).round().max(1.0) as usize)
            .unwrap_or(1);
        let host_tick = k % host_decim == 0;
        if gc.is_enabled() && host_tick {
            let host_dt = params.dt * host_decim as f64;
            let (out, targets, torque_ff) = gc.tick(host_dt);
            match params.actuation {
                Actuation::PositionTorque => {
                    for (idx, q) in targets {
                        sim.set_position_target(idx, q);
                    }
                }
                Actuation::Torque { kp, kd } => {
                    // The whole loop, host-side, including gravity -- a raw
                    // torque command means what it says and the driver adds
                    // nothing. Written every host tick, burn-in included: the
                    // WBC does not start until burn-in ends, and with no
                    // driver-side loop a joint whose torque was never set is
                    // a joint with zero torque.
                    let grav = sim.gravity_torques(&robot);
                    for (slot, &(idx, q_star)) in targets.iter().enumerate() {
                        let (q_meas, qd_meas) = sim
                            .joint_q_qd(&robot.joints[idx].name)
                            .unwrap_or((q_star, 0.0));
                        let pd = kp * (q_star - q_meas) - kd * qd_meas
                            + grav.get(idx).copied().unwrap_or(0.0);
                        host_pd[slot] = (idx, pd);
                        sim.set_torque_target(idx, pd);
                    }
                }
                Actuation::Velocity { k_track, .. }
                | Actuation::VelocityIdeal { k_track } => {
                    // Trajectory velocity plus an outer position loop. A speed
                    // loop has no position feedback, so without the second
                    // term the leg tracks the right *rate* while its absolute
                    // position walks away.
                    for (slot, &(idx, q_star)) in targets.iter().enumerate() {
                        let q_prev = prev_targets[slot];
                        let qd_ff = if k > burn_in_steps {
                            (q_star - q_prev) / host_dt
                        } else {
                            0.0
                        };
                        let q_meas = sim
                            .joint_q_qd(&robot.joints[idx].name)
                            .map(|(q, _)| q)
                            .unwrap_or(q_star);
                        sim.set_velocity_target(
                            idx,
                            qd_ff + k_track * (q_star - q_meas),
                        );
                        prev_targets[slot] = q_star;
                    }
                }
            }
            // After burn-in: route through WBC. Skip during burn-in
            // so the body has a chance to settle on its feet via the
            // Position-PD path (the WBC's static balance only
            // converges once the legs are loaded).
            if k >= burn_in_steps {
                let f_grf_world = gc
                    .predicted_grfs()
                    .map(|sol| sol.grfs_first_step)
                    .unwrap_or([Vector3::zeros(); 4]);
                let cmd = gc.velocity_cmd();
                // Body-frame command — the WBC pipeline rotates the
                // observation internally using the current xquat.
                let v_cmd_body = Vector3::new(cmd.vx, cmd.vy, 0.0);
                // Contact-driven phase correction (Phase C). Read the
                // per-foot ground reaction force from MuJoCo and apply
                // ContactDrivenPhase's stateless override to the
                // gait controller's nominal phases. This catches early
                // touchdown / late liftoff before the WBC's
                // no_contact_motion task assumes the wrong contact
                // pattern, which would otherwise destabilise the body
                // during trotting (stance=2/4).
                let foot_links_str: [&str; 4] = [
                    wbc_pipeline.foot_links[0].as_str(),
                    wbc_pipeline.foot_links[1].as_str(),
                    wbc_pipeline.foot_links[2].as_str(),
                    wbc_pipeline.foot_links[3].as_str(),
                ];
                let force_z = sim.contact_force_per_foot(&foot_links_str);
                let nominal_phases = [
                    out.legs[0].phase,
                    out.legs[1].phase,
                    out.legs[2].phase,
                    out.legs[3].phase,
                ];
                let corrected = ContactDrivenPhase::apply_correction(
                    &nominal_phases,
                    force_z,
                    params.early_contact_n,
                    // late_liftoff disabled (= 0 N): if every foot is
                    // momentarily unloaded during a transient body fall,
                    // a non-zero threshold would flip ALL legs to swing
                    // and there would be no support at all → unrecoverable
                    // collapse. Early touchdown is the more important
                    // direction anyway; late liftoff matters mainly for
                    // slip detection on real hardware.
                    /* late_liftoff_threshold_n = */ 0.0,
                );
                let contact_flag = [
                    corrected[0].is_stance,
                    corrected[1].is_stance,
                    corrected[2].is_stance,
                    corrected[3].is_stance,
                ];
                wbc_pipeline.base_pos_bias_world = Vector3::new(
                    params.base_pos_bias_m[0] + params.base_pos_drift_mps[0] * t,
                    params.base_pos_bias_m[1] + params.base_pos_drift_mps[1] * t,
                    params.base_pos_bias_m[2] + params.base_pos_drift_mps[2] * t,
                );
                let taus = wbc_pipeline.solve(
                    &robot,
                    &sim,
                    &out,
                    gc.kinematics(),
                    gc.joint_indices(),
                    gc.joint_signs(),
                    &v_cmd_body,
                    cmd.wz,
                    &Vector3::new(v_obs[0], v_obs[1], v_obs[2]),
                    &Vector3::new(w_obs[0], w_obs[1], w_obs[2]),
                    &f_grf_world,
                    contact_flag,
                    params.dt,
                );
                // Diagnostic dump every 100 ticks (200 ms). Detailed
                // breakdown for the G-step (Phase 1.5 / forward walk)
                // root-cause hunt: per-leg q*-vs-q tracking error,
                // swing target vs measured foot position, MPC f_GRF
                // reference vs WBC sol.f_grf, and the body-frame
                // foot offsets. Tab-aligned so a regression diff
                // shows column-by-column.
                if k % 100 == 0 {
                    let body_pos = sim
                        .body_world_position(&robot.root_link)
                        .unwrap_or([0.0, 0.0, 0.0]);
                    let tau_max = taus
                        .iter()
                        .cloned()
                        .fold(0.0_f64, |a, b| a.max(b.abs()));
                    let mpc_fz_sum: f64 = f_grf_world.iter().map(|v| v.z).sum();
                    let stance_count = contact_flag.iter().filter(|b| **b).count();
                    eprintln!(
                        "[diag k={k:5} t={:.3}s] z={:.3} m  Σmpc_f_z={:.2} N  \
                         max|τ|={:.2} N·m  stance={}/4",
                        k as f64 * params.dt,
                        body_pos[2],
                        mpc_fz_sum,
                        tau_max,
                        stance_count
                    );
                    // Per-leg tracking detail. `targets` carries
                    // (joint_idx, q_target_urdf); compare against
                    // mj_sim.joint_q_qd to see how well Position-PD
                    // is following.
                    let mut q_err_max = 0.0_f64;
                    let mut qd_err_max = 0.0_f64;
                    for (ji, q_target) in targets.iter() {
                        if let Some((q_actual, qd_actual)) =
                            sim.joint_q_qd(&robot.joints[*ji].name)
                        {
                            let q_err = (*q_target - q_actual).abs();
                            q_err_max = q_err_max.max(q_err);
                            qd_err_max = qd_err_max.max(qd_actual.abs());
                        }
                    }
                    // WBC solution breakdown (cached on the pipeline).
                    let (wbc_fz_sum, q_ddot_z, tau_norm) =
                        if let Some(sol) = wbc_pipeline.last_solution.as_ref() {
                            let fz: f64 =
                                (0..4).map(|s| sol.f_grf[3 * s + 2]).sum();
                            let q_ddot_z = sol.q_ddot[5]; // body z (Featherstone [ang;lin])
                            let tau_norm: f64 = sol.tau.iter().map(|x| x * x).sum::<f64>().sqrt();
                            (fz, q_ddot_z, tau_norm)
                        } else {
                            (0.0, 0.0, 0.0)
                        };
                    // Per-leg swing target vs measured: only meaningful
                    // when the leg is in nominal swing, but compute
                    // for all 4 so the dump is uniform width.
                    let mut swing_err_max = 0.0_f64;
                    for slot in 0..4 {
                        if !out.legs[slot].phase.is_stance {
                            // foot_body target (body frame).
                            let target_body = out.legs[slot].foot_body;
                            // Measured via FK on the actual joint q.
                            let leg_kin = gc.kinematics().legs()[slot];
                            let mut q_leg = [0.0_f64; 3];
                            let signs = gc.joint_signs()[slot];
                            for kk in 0..3 {
                                let ji = gc.joint_indices()[slot][kk];
                                if let Some((q, _)) =
                                    sim.joint_q_qd(&robot.joints[ji].name)
                                {
                                    q_leg[kk] = signs[kk] * q;
                                }
                            }
                            let measured_body =
                                quadruped_gait::forward_leg_kinematics(
                                    leg_kin, q_leg[0], q_leg[1], q_leg[2],
                                );
                            let err = (target_body - measured_body).norm();
                            swing_err_max = swing_err_max.max(err);
                        }
                    }
                    eprintln!(
                        "[diag-detail k={k:5}] q*-q max={:.4} rad   q̇ max={:.3} rad/s   \
                         WBC Σf_z={:.2} N  q̈_z={:.2}  ‖τ‖={:.2}  swing FK err={:.4} m",
                        q_err_max, qd_err_max,
                        wbc_fz_sum, q_ddot_z, tau_norm, swing_err_max,
                    );
                    // MPC vs WBC f_GRF per-leg comparison (z-component
                    // only; horizontal components omitted to keep the
                    // line readable).
                    if let Some(sol) = wbc_pipeline.last_solution.as_ref() {
                        let mpc_per_leg: [f64; 4] =
                            std::array::from_fn(|s| f_grf_world[s].z);
                        let wbc_per_leg: [f64; 4] =
                            std::array::from_fn(|s| sol.f_grf[3 * s + 2]);
                        eprintln!(
                            "[diag-grf  k={k:5}] MPC_f_z=[{:.1} {:.1} {:.1} {:.1}]   \
                             WBC_f_z=[{:.1} {:.1} {:.1} {:.1}]",
                            mpc_per_leg[0], mpc_per_leg[1], mpc_per_leg[2], mpc_per_leg[3],
                            wbc_per_leg[0], wbc_per_leg[1], wbc_per_leg[2], wbc_per_leg[3],
                        );
                    }
                }
                // Hybrid-joint command (legged_control style):
                // Position-PD already runs against the gait controller's
                // q* (set_position_target above), so WBC τ goes in as
                // feedforward — NOT as a full PD bypass.
                //
                // This is the single biggest difference from the previous
                // `set_wbc_torques` path: the Position-PD is what actually
                // drives the joints to track the gait controller's q*,
                // while WBC's τ_ff adds the dynamic / contact-force
                // contribution on top. Without this hybrid scheme, WBC's
                // long-term tracking error accumulates because the QP
                // produces accelerations not positions, and there's no
                // integrator to drive joint-position drift back to zero.
                // Hybrid joint command: Position-PD tracks q*, WBC τ
                // adds dynamic + gravity + contact-force compensation
                // on top. The WBC tries to produce τ such that the QP
                // solution's f_GRF matches the MPC's predicted GRFs
                // (= forward thrust included), but the actual tracking
                // depends on `W_CONTACT_FORCE` in `quadruped_gait::wbc`.
                let _ = torque_ff; // discard MPC ff — WBC owns the τ stream
                // A speed-mode driver takes a speed and nothing else, so the
                // WBC's torque has nowhere to go on that path. The sim's
                // Velocity law ignores `tau_ff` for the same reason; this
                // keeps the two from disagreeing about why.
                let deliver_tau = params.actuation == Actuation::PositionTorque
                    && !params.kinematic_only;
                for (ji, &tau) in taus.iter().enumerate() {
                    sim.set_torque_feedforward(ji, if deliver_tau { tau } else { 0.0 });
                }
                if let Actuation::Torque { .. } = params.actuation {
                    // One raw torque per joint: the host PD computed above,
                    // plus the WBC's. Nothing on the driver side adds gravity
                    // in this mode -- the WBC's tau carries it, since the QP
                    // solves the full equation of motion.
                    for &(idx, pd) in host_pd.iter() {
                        let tau_wbc =
                            if params.kinematic_only { 0.0 } else { taus[idx] };
                        sim.set_torque_target(idx, pd + tau_wbc);
                    }
                }
                sim.clear_wbc_torques();
            } else {
                sim.clear_wbc_torques();
                // No WBC active during burn-in — clear any feedforward
                // torque so the per-joint PD path runs cleanly.
                for ji in 0..robot.joints.len() {
                    sim.set_torque_feedforward(ji, 0.0);
                }
            }
        }

        sim.step(&mut robot, params.dt, true);

        // Sample after the step so contact forces and pose are
        // synchronised. `base_transform` was just refreshed by
        // `sim.step → sync_back`.
        let tx = robot.base_transform.translation;
        let (roll, pitch, yaw) = robot.base_transform.rotation.euler_angles();
        let total_fz_world: f64 =
            sim.contacts().iter().map(|c| c.force_world[2]).sum();
        // Applied joint torque against its own effort limit. `mujoco_sim`
        // clamps to `joint.effort` silently, so without this a saturated
        // actuator is invisible: the harness would report a gait that the
        // hardware cannot produce and nothing would say so.
        let qfrc = sim.qfrc_actuator();
        let mut tau_frac = [0.0f64; 12];
        let mut qd_frac = [0.0f64; 12];
        for (leg, tri) in gc.joint_indices().iter().enumerate() {
            for (j, &ji) in tri.iter().enumerate() {
                let joint = &robot.joints[ji];
                if joint.effort > 0.0 {
                    if let Some(adr) = sim.joint_dof_adr(&joint.name) {
                        if let Some(&t) = qfrc.get(adr) {
                            tau_frac[leg * 3 + j] = (t / joint.effort).abs();
                        }
                    }
                }
                if joint.velocity > 0.0 {
                    if let Some((_, qd)) = sim.joint_q_qd(&joint.name) {
                        qd_frac[leg * 3 + j] = (qd / joint.velocity).abs();
                    }
                }
            }
        }
        let mut foot_fz = [0.0f64; 4];
        for (fi, (_, link)) in DEFAULT_FOOT_LINKS.iter().enumerate() {
            let lname = link.to_lowercase();
            foot_fz[fi] = sim
                .contacts()
                .iter()
                .filter(|c| {
                    c.body1.to_lowercase() == lname || c.body2.to_lowercase() == lname
                })
                .map(|c| c.force_world[2].abs())
                .sum();
        }

        if replay_out.is_some() {
            let p = sim.body_world_position(&robot.root_link).unwrap_or([0.0; 3]);
            let q = robot.base_transform.rotation;
            replay_buf.push_str(&format!(
                "{:.5},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6}",
                t, p[0], p[1], p[2], q.w, q.i, q.j, q.k
            ));
            for name in &replay_joint_names {
                let qj = sim.joint_q_qd(name).map(|(q, _)| q).unwrap_or(0.0);
                replay_buf.push_str(&format!(",{qj:.6}"));
            }
            // After `foot_fz` is filled, not before -- the renderer's footfall
            // diagram is only worth drawing if it shows measured load.
            for fz in foot_fz {
                replay_buf.push_str(&format!(",{fz:.4}"));
            }
            let c = gc.velocity_cmd();
            let push_fy = params
                .push
                .filter(|(t0, _, dur)| t >= *t0 && t < *t0 + *dur)
                .map(|(_, f, _)| f[1])
                .unwrap_or(0.0);
            replay_buf.push_str(&format!(
                ",{:.4},{:.4},{:.4},{push_fy:.2}",
                c.vx, c.vy, c.wz
            ));
            replay_buf.push('\n');
        }

        samples.push(WbcSample {
            t,
            body_x: tx.x,
            body_z: tx.z,
            roll,
            pitch,
            total_fz_world,
            foot_fz,
            body_y: tx.y,
            yaw,
            cycle_period_s,
            tau_frac,
            qd_frac,
        });
    }
    if let Some(dir) = replay_out.as_deref() {
        std::fs::write(format!("{dir}/trace.csv"), &replay_buf)
            .expect("write replay trace.csv");
        eprintln!("[replay] wrote {dir}/model.xml and trace.csv");
    }
    Some(samples)
}

/// Compute the total robot mass by summing every link's `mass` field.
/// `m·g` is the reference for the static-balance assertion.
fn robot_mass(robot: &RobotModel) -> f64 {
    robot.links.iter().map(|l| l.inertial.mass).sum()
}

/// Static stand under WBC torque control. The Phase A integration
/// went through several rounds of tightening before this could pass:
///
/// 1. `q[3..7]` quaternion sync from MuJoCo (resolved at adbc9f1).
/// 2. SE(3)-correct `compute_joint_jacobian_time_derivative` so a
///    moving floating base doesn't blow up `J̇·v` (Phase 1.2).
/// 3. Per-task LSQ weights so EoM/no_contact_motion dominate
///    contact_force/τ_grav (Phase 1.4).
/// 4. `a_base_des` derived from the **MPC's predicted GRFs** via
///    SRBD physics, instead of a hand-tuned PD on body velocity
///    (Phase 1.5-A) — this is the single biggest fix; without it
///    the WBC drives the body to a different equilibrium than the
///    MPC chose and the two fight to body collapse.
///
/// The avg Σf_z tolerance is ±60% rather than the ideal ±10% because
/// MuJoCo's contact bouncing produces large per-tick swings even when
/// the body is mean-stable around the target z. The *min_z fall*
/// check catches the real failure mode (body slumping below 0.18 m).
#[test]
fn wbc_static_stand_balances_gravity() {
    let params = WbcParams::static_stand();
    let misa = params.misa_file;
    let Some(samples) = run_wbc_sim(params) else {
        return;
    };
    assert_static_stand_balances_gravity(&samples, misa);
}

/// Same invariants as [`wbc_static_stand_balances_gravity`], but
/// routed through the misa-wbc `Dynamics` path in the `ForceSpace`
/// formulation (GID's decision-variable layout, `x = [τ; f]`) with
/// the `ActiveSet` backend — the fastest combination per the
/// `misa-wbc` formulation benchmarks (see `ref/wbc_comparison.md`).
/// Confirms the equivalence holds under real MuJoCo contact dynamics,
/// not just on synthetic matrices.
#[test]
fn wbc_static_stand_balances_gravity_force_space_active_set() {
    let cfg = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
    let params = WbcParams::static_stand_misa_wbc(wbc::Formulation::ForceSpace, cfg);
    let misa = params.misa_file;
    let Some(samples) = run_wbc_sim(params) else {
        return;
    };
    assert_static_stand_balances_gravity(&samples, misa);
}

/// NOTE: takes the model path because the m*g reference has to come from the
/// same robot the samples came from. Before `WbcParams::misa_file` existed
/// there was only one model and this read a hardcoded `namiashi.misa`; once a
/// 3.3 kg variant could be passed in, that silently compared 32.4 N of
/// measured contact force against a 23.5 N reference -- a 38% error sitting
/// inside a +/-60% tolerance, so it would have passed while being wrong.
fn assert_static_stand_balances_gravity(samples: &[WbcSample], misa_file: &str) {
    // No fall.
    let min_z = samples.iter().map(|s| s.body_z).fold(f64::INFINITY, f64::min);
    assert!(
        min_z > TRUNK_Z_FALL_THRESHOLD_M,
        "static stand: trunk fell, min_z = {min_z:.3} m (threshold {:.2})",
        TRUNK_Z_FALL_THRESHOLD_M,
    );

    // Burn-in window done; sample the last 0.5 s for the f_z average.
    // Window derived from the samples, not from hardcoded run lengths: these
    // used to be literal 1.5/0.5 s constants that happened to match
    // `WbcParams::static_stand()` and would have silently sliced the wrong
    // window the moment anyone changed it.
    let t_end = samples[samples.len() - 1].t;
    let start = samples
        .iter()
        .position(|s| s.t >= t_end - STATIC_AVG_WINDOW_S)
        .unwrap_or(0);
    let avg_fz: f64 = samples[start..]
        .iter()
        .map(|s| s.total_fz_world)
        .sum::<f64>()
        / (samples.len() - start) as f64;

    // Reference m·g. Recompute by reloading the .misa (cheap).
    let path = namiashi_misa_named(misa_file);
    let robot = RobotModel::from_misa(&path).unwrap();
    let mg = robot_mass(&robot) * 9.81;

    let pct_err = ((avg_fz - mg) / mg).abs();
    eprintln!(
        "[wbc] static_stand: avg Σf_z = {avg_fz:.2} N, m·g = {mg:.2} N \
         (err = {:.1}%)",
        pct_err * 100.0,
    );
    // ±60% tolerance — generous because:
    //   - per-tick MuJoCo contact force has large bouncing components
    //     (the integrator's substeps see sharp normal-force spikes
    //     around touchdown which the 0.5 s sample window doesn't fully
    //     average out),
    //   - friction-cone clipping in the QP can re-allocate force
    //     among feet, making instantaneous totals oscillate,
    //   - the WBC's static-stand z drifts within a few cm because
    //     there's no explicit z-position regulator (the SRBD MPC's
    //     z weight handles it but its 30 ms horizon discretisation
    //     leaves residual oscillation).
    // The accompanying min_z assertion (above) is the real fall
    // detector; this avg-Σf_z test mostly guards against pathological
    // gravity-direction errors.
    assert!(
        pct_err < 0.60,
        "static stand: Σf_z = {avg_fz:.2} N deviates from m·g = {mg:.2} N \
         by {:.1}% — friction cone + EoM may be inconsistent",
        pct_err * 100.0,
    );
}

/// Forward walk under WBC + Position-PD hybrid joint command.
/// Asserts the trunk advances at least `MIN_DISPLACEMENT_M` over the
/// run without falling (`min_z > TRUNK_Z_FALL_THRESHOLD_M`).
///
/// The test sequence — MPC-driven `a_base_des` (Phase 1.5-A), the
/// joint-space swing_leg task (G2, sharing q* with Position-PD), and
/// `W_CONTACT_FORCE = 5` (H4 tuning, lets the WBC track MPC's
/// predicted GRF tightly enough that stance forward thrust flows
/// into joint torque) — together produce ~17 cm of forward
/// displacement under a 0.15 m/s command over 2.5 s.
#[test]
fn wbc_forward_command_advances_body() {
    let Some(samples) = run_wbc_sim(WbcParams::forward_walk()) else {
        return;
    };
    assert_forward_command_advances_body(&samples);
}

/// Same invariants as [`wbc_forward_command_advances_body`], routed
/// through misa-wbc's `ForceSpace` + `ActiveSet` — see
/// [`wbc_static_stand_balances_gravity_force_space_active_set`] for
/// the rationale.
#[test]
fn wbc_forward_command_advances_body_force_space_active_set() {
    let cfg = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
    let Some(samples) =
        run_wbc_sim(WbcParams::forward_walk_misa_wbc(wbc::Formulation::ForceSpace, cfg))
    else {
        return;
    };
    assert_forward_command_advances_body(&samples);
}

/// One line per run: speed, attitude, support pattern.
///
/// The original assertions here checked trunk height and net displacement.
/// Neither can tell a gait that is walking from one that is shuffling in
/// place, or a robot standing on three legs from one standing on four --
/// both of which turned out to be happening on the Go2 side and were only
/// found by measuring contact directly.
/// What a run is judged on. Body-frame throughout -- see the note in
/// `report_walk_cmd` for why the run-start frame is not trustworthy.
#[derive(Debug, Default, Clone, Copy)]
struct WalkMetrics {
    /// Forward speed in the instantaneous heading frame, m/s.
    body_vx: f64,
    /// Sideways speed in the instantaneous heading frame, m/s.
    body_vy: f64,
    /// deg/s, unwrapped.
    yaw_rate_deg_s: f64,
    z_min: f64,
}

fn report_walk_cmd(
    label: &str,
    samples: &[WbcSample],
    cmd_vx: f64,
    cmd_vy: f64,
    cmd_wz: f64,
    burn_in_s: f64,
) -> WalkMetrics {
    let t0 = samples[0].t + burn_in_s;
    let walk: Vec<&WbcSample> = samples.iter().filter(|s| s.t >= t0).collect();
    if walk.len() < 10 {
        eprintln!("=== {label}: too few samples ===");
        return WalkMetrics::default();
    }
    let n = walk.len() as f64;
    let (a, b) = (walk[0], walk[walk.len() - 1]);
    let span = b.t - a.t;

    // Displacement projected on the heading the robot had when the command
    // arrived, and its normal. Reported for the path shape only -- do NOT
    // read `lat` as sideways motion. If the robot yaws while walking
    // straight ahead, forward travel leaks into this frame's lateral axis:
    // at Trot's -15 deg of yaw drift, 0.89 m/s forward shows up here as
    // 0.89*sin(15 deg) = 0.23 m/s of "crab" that is not happening. Body-frame
    // vx/vy below is the honest measure.
    let (c0, s0) = ((-a.yaw).cos(), (-a.yaw).sin());
    let (dx, dy) = (b.body_x - a.body_x, b.body_y - a.body_y);
    let fwd = c0 * dx - s0 * dy;
    let lat = s0 * dx + c0 * dy;
    // Accumulate wrapped per-sample increments rather than differencing the
    // endpoints. Endpoint differencing wraps at +/-180 deg, so it cannot see
    // more than half a turn: a clean 0.76-gain response to wz=0.90 rad/s over
    // 10 s is 392 deg of real rotation and reads back as +32 deg, which looks
    // like the turn collapsing to 7% when nothing collapsed.
    let mut dyaw = 0.0;
    for w in walk.windows(2) {
        let mut d = w[1].yaw - w[0].yaw;
        while d > std::f64::consts::PI {
            d -= 2.0 * std::f64::consts::PI;
        }
        while d < -std::f64::consts::PI {
            d += 2.0 * std::f64::consts::PI;
        }
        dyaw += d;
    }

    let z_mean = walk.iter().map(|s| s.body_z).sum::<f64>() / n;
    let z_min = walk.iter().map(|s| s.body_z).fold(f64::INFINITY, f64::min);
    let pitch_pk = walk.iter().map(|s| s.pitch.abs()).fold(0.0_f64, f64::max);
    let roll_pk = walk.iter().map(|s| s.roll.abs()).fold(0.0_f64, f64::max);

    // Support census at a 1 N threshold -- namiashi is 2.4 kg, so its
    // per-foot loads are roughly a sixth of Go2's and the 5 N threshold used
    // there would read a loaded foot as airborne.
    let mut hist = [0usize; 5];
    let mut duty = [0usize; 4];
    for s in walk.iter() {
        let mut n_down = 0;
        for fi in 0..4 {
            if s.foot_fz[fi] > 1.0 {
                n_down += 1;
                duty[fi] += 1;
            }
        }
        hist[n_down] += 1;
    }
    let f = |k: usize| hist[k] as f64 / n;

    eprintln!(
        "=== {label} (cmd_vx={cmd_vx:.3}) ===\n\
         fwd={fwd:+.3}m lat={lat:+.3}m over {span:.1}s  speed={:+.3} m/s  \
         track={:.0}%  yaw_drift={:+.1}deg\n\
         trunk z: mean={z_mean:.3} min={z_min:.3}m   peak roll={:.1}deg pitch={:.1}deg\n\
         support@1N: n_down 0/1/2/3/4 = {:.3}/{:.3}/{:.3}/{:.3}/{:.3}   \
         duty per foot = {:.2} {:.2} {:.2} {:.2}",
        fwd / span,
        if cmd_vx.abs() > 1e-9 { 100.0 * (fwd / span) / cmd_vx } else { 0.0 },
        dyaw.to_degrees(),
        roll_pk.to_degrees(), pitch_pk.to_degrees(),
        f(0), f(1), f(2), f(3), f(4),
        duty[0] as f64 / n, duty[1] as f64 / n,
        duty[2] as f64 / n, duty[3] as f64 / n,
    );

    // Per-second velocity in each window's OWN heading frame, plus that
    // window's yaw rate. Rotating by the yaw at the start of the window
    // instead of the start of the run is the whole point: it separates "the
    // robot is sliding sideways" from "the robot is pointing somewhere else
    // by now", which the run-start frame silently mixes together.
    // Round the averaging window up to a whole number of gait cycles. A fixed
    // 1.0 s window beats against the gait: 1.0/0.400 = 2.5 and 1.0/0.800 =
    // 1.25, so every Walk window boundary lands on one of two gait phases and
    // every Crawl boundary on one of four. Averaging many such windows does
    // not average the phase out, it averages two or four clusters.
    let period = walk[0].cycle_period_s;
    let win_s = period * (1.0 / period).ceil();
    // Torque headroom. `mujoco_sim` clamps to `joint.effort` without a word,
    // so a gait can look fine here while asking the hardware for torque it
    // does not have. Reported per joint role because the roles have different
    // limits (hip and thigh 1.5 N*m, calf 2.205) and very different jobs.
    const ROLE: [&str; 3] = ["hip", "thigh", "calf"];
    let mut role_line = String::from("  tau/limit:");
    for j in 0..3 {
        let mut peak = 0.0f64;
        let mut sat = 0usize;
        for s in walk.iter() {
            for leg in 0..4 {
                let f = s.tau_frac[leg * 3 + j];
                peak = peak.max(f);
                if f > 0.99 {
                    sat += 1;
                }
            }
        }
        let mut qd_peak = 0.0f64;
        let mut qd_over = 0usize;
        for s in walk.iter() {
            for leg in 0..4 {
                let f = s.qd_frac[leg * 3 + j];
                qd_peak = qd_peak.max(f);
                if f > 1.0 {
                    qd_over += 1;
                }
            }
        }
        role_line += &format!(
            "  {}: tau pk={peak:.2} sat={:.1}% | qd pk={qd_peak:.2} over={:.1}%",
            ROLE[j],
            100.0 * sat as f64 / (4.0 * n),
            100.0 * qd_over as f64 / (4.0 * n),
        );
    }
    eprintln!("{role_line}");

    let mut line = format!("  per-{win_s:.2}s vx/vy/wz:");
    let (mut vx_sum, mut vy_sum, mut nw) = (0.0, 0.0, 0usize);
    let mut w0 = 0usize;
    while w0 < walk.len() {
        let w1 = walk
            .iter()
            .position(|s| s.t >= walk[w0].t + win_s)
            .unwrap_or(walk.len() - 1);
        if w1 <= w0 {
            break;
        }
        let (p, q) = (walk[w0], walk[w1]);
        let dt = q.t - p.t;
        let (cw, sw) = ((-p.yaw).cos(), (-p.yaw).sin());
        let (ddx, ddy) = (q.body_x - p.body_x, q.body_y - p.body_y);
        let (fx, fy) = ((cw * ddx - sw * ddy) / dt, (sw * ddx + cw * ddy) / dt);
        let mut dy = q.yaw - p.yaw;
        while dy > std::f64::consts::PI {
            dy -= 2.0 * std::f64::consts::PI;
        }
        while dy < -std::f64::consts::PI {
            dy += 2.0 * std::f64::consts::PI;
        }
        line += &format!("  {fx:+.2}/{fy:+.2}/{:+.1}", dy.to_degrees() / dt);
        vx_sum += fx;
        vy_sum += fy;
        nw += 1;
        w0 = w1;
    }
    eprintln!("{line}");
    let mut metrics = WalkMetrics {
        z_min,
        yaw_rate_deg_s: dyaw.to_degrees() / span,
        ..WalkMetrics::default()
    };
    if nw > 0 {
        let (bvx, bvy) = (vx_sum / nw as f64, vy_sum / nw as f64);
        metrics.body_vx = bvx;
        metrics.body_vy = bvy;
        eprintln!(
            "  body-frame: vx={bvx:+.3} m/s ({:.0}% of cmd)  vy={bvy:+.3} m/s  \
             yaw rate={:+.2} deg/s",
            if cmd_vx.abs() > 1e-9 { 100.0 * bvx / cmd_vx } else { 0.0 },
            dyaw.to_degrees() / span,
        );
        if cmd_vy.abs() > 1e-9 || cmd_wz.abs() > 1e-9 {
            eprintln!(
                "  cmd vy={cmd_vy:+.2} -> {bvy:+.3} m/s ({:.0}%)   \
                 cmd wz={:+.1} -> {:+.1} deg/s ({:.0}%)",
                if cmd_vy.abs() > 1e-9 { 100.0 * bvy / cmd_vy } else { 0.0 },
                cmd_wz.to_degrees(),
                dyaw.to_degrees() / span,
                if cmd_wz.abs() > 1e-9 {
                    100.0 * (dyaw / span) / cmd_wz
                } else {
                    0.0
                },
            );
        }
    }
    metrics
}

fn report_walk(
    label: &str,
    samples: &[WbcSample],
    cmd_vx: f64,
    burn_in_s: f64,
) -> WalkMetrics {
    report_walk_cmd(label, samples, cmd_vx, 0.0, 0.0, burn_in_s)
}

/// WHERE namiashi ACTUALLY IS (2026-08-02).
///
/// This file had four tests, all on Trot, all asking only "did the trunk stay
/// up and did x increase" over 3 s at 0.15 m/s. Nothing here had ever run
/// Walk, Crawl or Pace, and nothing measured speed, heading or contact.
///
/// This is the baseline sweep before any tuning: each gait at its own default
/// period and duty, over a range of commands, reporting what actually
/// happens. Failures are expected and are the point -- they say which gait
/// needs what.
///
/// SCALE. namiashi is 2.400 kg with a 0.306 m leg; Go2 is 15.606 kg with
/// 0.426 m. Under Froude similarity (T ~ sqrt(L/g), stride ~ L) Go2's tuned
/// 0.18 s / 0.20 m map to 0.152 s / 0.143 m here, and its 1.63 m/s to
/// 1.38 m/s. The commands below bracket that, but the gait defaults are left
/// alone in this first pass so the starting point is the library's own.
#[test]
#[ignore = "exploratory sweep -- run with --ignored"]
fn namiashi_gait_baseline_sweep() {
    for gait in [GaitType::Trot, GaitType::Walk, GaitType::Crawl] {
        for cmd_vx in [0.15, 0.30, 0.50] {
            let params = WbcParams {
                total_time_s: 6.0,
                burn_in_s: 1.0,
                cmd_vx,
                gait_type: Some(gait),
                ..WbcParams::forward_walk()
            };
            let Some(samples) = run_wbc_sim(params) else { return };
            report_walk(&format!("{gait:?} default"), &samples, cmd_vx, 1.0);
        }
    }
}

/// SCALING THE THREE GAITS TO namiashi'S GEOMETRY.
///
/// The baseline sweep says all three gaits are step-length starved, not
/// broken. Each one sits at (or below) its own geometric ceiling
///
///     v_max = max_step / (T * duty)
///
/// and those ceilings are 0.50 / 0.18 / 0.04 m/s for the library defaults --
/// so Walk saturating at 0.17 m/s under a 0.50 m/s command is arithmetic, not
/// a controller failure.
///
/// Normalised by leg length (0.306 m) the default steps are 33% / 26% / 20%.
/// The Go2 Bound work settled on 0.20 m over a 0.426 m leg = 47%, and that
/// robot walks. So the defaults here are conservative by roughly a factor of
/// two across the board.
///
/// This sweep raises max_step toward that ratio and shortens the period
/// (Froude: T ~ sqrt(L/g), so namiashi's equivalent of Go2's 0.18 s is
/// 0.152 s), asking each gait for a speed its geometry can actually deliver.
/// Two settings per gait: "reach" moves the ceiling up mostly via step
/// length, "quick" mostly via period. Which one holds tells us whether the
/// limit is swing reach or swing time.
#[test]
#[ignore = "exploratory sweep -- run with --ignored"]
fn namiashi_gait_scaled_sweep() {
    // (gait, label, T, duty, max_step, cmd)
    // cmd is set to ~80% of that row's ceiling so the command is inside what
    // the geometry allows -- commanding past the ceiling only measures the
    // ceiling again.
    let rows: &[(GaitType, &str, f64, f64, f64, f64)] = &[
        // Trot: default already reaches 0.5. Push on both axes.
        (GaitType::Trot, "reach", 0.400, 0.50, 0.145, 0.58),
        (GaitType::Trot, "quick", 0.260, 0.50, 0.100, 0.61),
        (GaitType::Trot, "both", 0.260, 0.50, 0.145, 0.89),
        // Walk: duty 0.75 costs a lot of ceiling, so it needs the most step.
        (GaitType::Walk, "reach", 0.600, 0.75, 0.145, 0.25),
        (GaitType::Walk, "quick", 0.400, 0.75, 0.100, 0.26),
        (GaitType::Walk, "both", 0.400, 0.75, 0.145, 0.38),
        // Crawl: duty 0.85 and a 1.67 s period leave almost nothing.
        (GaitType::Crawl, "reach", 1.667, 0.85, 0.145, 0.08),
        (GaitType::Crawl, "quick", 0.800, 0.85, 0.060, 0.07),
        (GaitType::Crawl, "both", 0.800, 0.85, 0.145, 0.17),
    ];
    for &(gait, tag, t, duty, step, cmd_vx) in rows {
        let params = WbcParams {
            total_time_s: 6.0,
            burn_in_s: 1.0,
            cmd_vx,
            gait_type: Some(gait),
            cycle_period_s: Some(t),
            duty_factor: Some(duty),
            max_step_length_m: Some(step),
            ..WbcParams::forward_walk()
        };
        let Some(samples) = run_wbc_sim(params) else { return };
        report_walk(&format!("{gait:?} {tag}"), &samples, cmd_vx, 1.0);
    }
}

/// TWO CANDIDATE CAUSES FOR WHAT THE SCALED SWEEP LEFT BROKEN.
///
/// After scaling, Trot and Walk overshoot their command by 3-16% and Trot
/// crabs sideways (+0.89 m in 5 s at one setting, -0.70 m at another -- the
/// sign flips, so it is not a fixed bias). Crawl instead *under*shoots, at
/// 66-74%.
///
/// H1 (both symptoms): `DEFAULT_CAPTURE_POINT_GAIN_S = 0.05`. That gain is
/// meant to be the LIP model's sqrt(h/g); for a 0.30 m trunk that is
/// 0.175 s, so the library ships a value 3.5x too small, for every robot.
/// The footstep feedback is pure proportional (`k_capture_pulse` and
/// `v_capture_deadband` both default to 0), and proportional feedback that
/// weak leaves exactly this: a standing error in x, and a lateral velocity
/// nothing pulls back to zero.
///
/// H2 (Crawl only): `GaitConfig::crawl()` sets `swing_height_m = 0.005`.
/// Five millimetres of clearance is defensible over the default 0.06 m step;
/// over the 0.145 m step the scaled sweep uses, the swing foot is dragging.
///
/// H1 predicts the sweep over k moves both the overshoot and the drift. H2
/// predicts swing height moves Crawl and nothing else.
#[test]
#[ignore = "diagnostic -- run with --ignored"]
fn namiashi_capture_gain_and_swing_height() {
    // H1: gain sweep on the two gaits that overshoot.
    for (gait, t, duty, step, cmd_vx) in [
        (GaitType::Trot, 0.260, 0.50, 0.145, 0.89),
        (GaitType::Walk, 0.400, 0.75, 0.145, 0.38),
    ] {
        for k in [0.05, 0.10, 0.175, 0.25] {
            let params = WbcParams {
                total_time_s: 6.0,
                burn_in_s: 1.0,
                cmd_vx,
                gait_type: Some(gait),
                cycle_period_s: Some(t),
                duty_factor: Some(duty),
                max_step_length_m: Some(step),
                k_capture_s: Some(k),
                ..WbcParams::forward_walk()
            };
            let Some(samples) = run_wbc_sim(params) else { return };
            report_walk(&format!("{gait:?} k={k:.3}"), &samples, cmd_vx, 1.0);
        }
    }

    // H2: swing clearance on Crawl, at both the old and the LIP gain, so a
    // clearance effect cannot be confused with a gain effect.
    for k in [0.05, 0.175] {
        for h in [0.005, 0.020, 0.040] {
            let params = WbcParams {
                total_time_s: 6.0,
                burn_in_s: 1.0,
                cmd_vx: 0.17,
                gait_type: Some(GaitType::Crawl),
                cycle_period_s: Some(0.800),
                duty_factor: Some(0.85),
                max_step_length_m: Some(0.145),
                swing_height_m: Some(h),
                k_capture_s: Some(k),
                ..WbcParams::forward_walk()
            };
            let Some(samples) = run_wbc_sim(params) else { return };
            report_walk(&format!("Crawl k={k:.3} h={h:.3}"), &samples, 0.17, 1.0);
        }
    }
}

/// H1 WAS BACKWARDS: THE FOOTSTEP FEEDBACK IS ALREADY AT ITS LIMIT.
///
/// The prediction was that `k_capture = 0.05` is too weak (LIP sqrt(h/g) is
/// 0.175 for this trunk) and that raising it would pull in both the 16%
/// speed overshoot and the lateral crab. Raising it did the opposite: Trot
/// at k=0.175 makes 2% of its command and yaws 43 deg; Walk at k=0.175
/// drifts 2.07 m sideways. Every increase made both symptoms worse, and the
/// drift *flipped sign* between k=0.05 and k=0.10.
///
/// A sign flip under a gain change is a closed-loop property, not a plant
/// bias -- so the lateral drift is the footstep feedback going unstable, and
/// 0.05 is already near the edge rather than 3.5x below where it belongs.
/// (The sqrt(h/g) reasoning assumes the foothold takes effect within one
/// step; here it is filtered through the MPC's own horizon, which adds lag
/// the LIP formula does not know about.)
///
/// So sweep the other way, down to and including k=0 -- pure open-loop
/// Raibert. If the drift is feedback-driven, k=0 has the least of it.
#[test]
#[ignore = "diagnostic -- run with --ignored"]
fn namiashi_capture_gain_low_side() {
    for (gait, t, duty, step, cmd_vx) in [
        (GaitType::Trot, 0.260, 0.50, 0.145, 0.89),
        (GaitType::Walk, 0.400, 0.75, 0.145, 0.38),
    ] {
        for k in [0.0, 0.015, 0.030, 0.050] {
            let params = WbcParams {
                total_time_s: 8.0,
                burn_in_s: 1.0,
                cmd_vx,
                gait_type: Some(gait),
                cycle_period_s: Some(t),
                duty_factor: Some(duty),
                max_step_length_m: Some(step),
                k_capture_s: Some(k),
                ..WbcParams::forward_walk()
            };
            let Some(samples) = run_wbc_sim(params) else { return };
            report_walk(&format!("{gait:?} k={k:.3}"), &samples, cmd_vx, 1.0);
        }
    }
}

/// DOES THE LATERAL MODE SATURATE, OR DOES IT DIVERGE?
///
/// The low-side sweep settled two things. Speed tracking is best with the
/// capture-point feedback OFF (98-100% at k=0, 120% at k=0.05) -- the
/// overshoot was the feedback, not the plan. And Trot's lateral velocity
/// grows monotonically (0.00 -> -0.13 m/s over 7 s) *even at k=0*, so the
/// crab is not a feedback artefact either; it is in the gait.
///
/// -0.02 m/s^2 over seven seconds is far too slow to call from a 6 s run.
/// A mode that saturates at some small lateral rate is a robot that walks
/// slightly sideways -- annoying, correctable later by a heading loop. A
/// mode that keeps growing is a robot that eventually falls over, and that
/// has to be fixed before any of this goes on hardware.
///
/// So: 25 s, the same duration the Go2 Bound runs were judged on, on each of
/// the three gaits at its best-so-far setting. This is also the first real
/// endurance test any of them has had -- the four original tests ran 3 s.
#[test]
#[ignore = "long run -- run with --ignored"]
fn namiashi_gait_endurance() {
    let rows: &[(GaitType, &str, f64, f64, f64, f64, f64, f64)] = &[
        // gait, tag, T, duty, step, swing_h, k, cmd
        (GaitType::Trot, "k0", 0.260, 0.50, 0.145, 0.040, 0.0, 0.89),
        (GaitType::Trot, "k.03", 0.260, 0.50, 0.145, 0.040, 0.030, 0.89),
        (GaitType::Walk, "k0", 0.400, 0.75, 0.145, 0.035, 0.0, 0.38),
        (GaitType::Crawl, "h.04", 0.800, 0.85, 0.145, 0.040, 0.050, 0.17),
    ];
    for &(gait, tag, t, duty, step, h, k, cmd_vx) in rows {
        let params = WbcParams {
            total_time_s: 26.0,
            burn_in_s: 1.0,
            cmd_vx,
            gait_type: Some(gait),
            cycle_period_s: Some(t),
            duty_factor: Some(duty),
            max_step_length_m: Some(step),
            swing_height_m: Some(h),
            k_capture_s: Some(k),
            ..WbcParams::forward_walk()
        };
        let Some(samples) = run_wbc_sim(params) else { return };
        report_walk(&format!("{gait:?} {tag} 25s"), &samples, cmd_vx, 1.0);
    }
}

/// CRAWL'S YAW DRIFT IS THE ONE REAL DEFECT LEFT.
///
/// Measured in each window's own heading frame, all three gaits track
/// forward speed to 101-103% and slide sideways by 2-19 mm/s. What looked
/// like a lateral instability was the run-start reporting frame; what looked
/// like Crawl decaying was Crawl turning.
///
/// What survives that correction is yaw: -0.60 deg/s on Trot, +0.41 on Walk,
/// and +2.34 on Crawl -- four to six times the others, 58 deg over 25 s.
/// Nothing commands a turn, so this is the wz=0 reference not being held.
///
/// Trot's real lateral drift went away at k=0 (-0.145 m/s at k=0.03 -> +0.002
/// at k=0), so the same question is worth asking of Crawl's yaw.
#[test]
#[ignore = "diagnostic -- run with --ignored"]
fn namiashi_crawl_yaw_drift() {
    for k in [0.0, 0.025, 0.050] {
        let params = WbcParams {
            total_time_s: 26.0,
            burn_in_s: 1.0,
            cmd_vx: 0.17,
            gait_type: Some(GaitType::Crawl),
            cycle_period_s: Some(0.800),
            duty_factor: Some(0.85),
            max_step_length_m: Some(0.145),
            swing_height_m: Some(0.040),
            k_capture_s: Some(k),
            ..WbcParams::forward_walk()
        };
        let Some(samples) = run_wbc_sim(params) else { return };
        report_walk(&format!("Crawl k={k:.3}"), &samples, 0.17, 1.0);
    }
}

/// CAN THESE GAITS DO ANYTHING BUT WALK FORWARD?
///
/// Every run so far has commanded (vx>0, 0, 0). That is the one case the
/// four original tests covered too, so nothing in this file has ever asked
/// namiashi to back up, strafe or turn -- and a gait that only goes forward
/// is not a controller.
///
/// Each gait is asked for all four at ~half its geometric ceiling
/// `max_step/(T*duty)`, since a reversing or strafing foothold is drawn from
/// the same step-length budget as a forward one.
#[test]
#[ignore = "coverage sweep -- run with --ignored"]
fn namiashi_command_coverage() {
    // gait, T, duty, step, swing_h, k, ceiling
    let gaits: &[(GaitType, f64, f64, f64, f64, f64, f64)] = &[
        (GaitType::Trot, 0.260, 0.50, 0.145, 0.040, 0.0, 1.115),
        (GaitType::Walk, 0.400, 0.75, 0.145, 0.035, 0.0, 0.483),
        (GaitType::Crawl, 0.800, 0.85, 0.145, 0.040, 0.0, 0.213),
    ];
    for &(gait, t, duty, step, h, k, ceil) in gaits {
        let v = 0.5 * ceil;
        let cases: [(&str, f64, f64, f64); 4] = [
            ("fwd", v, 0.0, 0.0),
            ("back", -v, 0.0, 0.0),
            ("strafe", 0.0, v, 0.0),
            ("turn", 0.0, 0.0, 0.35),
        ];
        for (tag, vx, vy, wz) in cases {
            let params = WbcParams {
                total_time_s: 11.0,
                burn_in_s: 1.0,
                cmd_vx: vx,
                cmd_vy: vy,
                cmd_wz: wz,
                gait_type: Some(gait),
                cycle_period_s: Some(t),
                duty_factor: Some(duty),
                max_step_length_m: Some(step),
                swing_height_m: Some(h),
                k_capture_s: Some(k),
                ..WbcParams::forward_walk()
            };
            let Some(samples) = run_wbc_sim(params) else { return };
            report_walk_cmd(&format!("{gait:?} {tag}"), &samples, vx, vy, wz, 1.0);
        }
    }
}

/// TURN RATE IS THE ONE THING THAT DOES NOT TRACK.
///
/// Command coverage came back clean on translation -- forward, backward and
/// sideways all land within 98-104% on all three gaits, and all twelve runs
/// keep the trunk between 0.293 and 0.296 m. Turning does not: 0.35 rad/s
/// (20.1 deg/s) commanded produces 15.2 / 16.3 / 15.7 deg/s on
/// Trot / Walk / Crawl -- 76-81%, and near enough the same deficit on three
/// gaits whose duty factors are 0.50, 0.75 and 0.85.
///
/// A shortfall that ignores duty that completely is not in the footstep
/// plan. (The rotational part of the plan is tiny anyway: at 0.35 rad/s and
/// a ~0.10 m hip offset the yaw contribution to the half-stride is ~2 mm,
/// two orders below the 0.145 m clamp, so nothing is being truncated.)
///
/// Which of two things it is decides whether it matters:
///   - a constant gain (~0.78) -- an outer heading loop absorbs it, and any
///     real robot has one;
///   - saturation -- the ratio falls as wz rises, and there is a turn rate
///     above which namiashi simply cannot comply.
/// Only a sweep over wz separates them.
#[test]
#[ignore = "diagnostic -- run with --ignored"]
fn namiashi_turn_rate_linearity() {
    for wz in [0.10, 0.20, 0.35, 0.60, 0.90] {
        let params = WbcParams {
            total_time_s: 11.0,
            burn_in_s: 1.0,
            cmd_vx: 0.0,
            cmd_wz: wz,
            gait_type: Some(GaitType::Trot),
            cycle_period_s: Some(0.260),
            duty_factor: Some(0.50),
            max_step_length_m: Some(0.145),
            swing_height_m: Some(0.040),
            k_capture_s: Some(0.0),
            ..WbcParams::forward_walk()
        };
        let Some(samples) = run_wbc_sim(params) else { return };
        report_walk_cmd(&format!("Trot wz={wz:.2}"), &samples, 0.0, 0.0, wz, 1.0);
    }
}

/// The settings each gait was tuned to, and the numbers they have to hold.
///
/// `k_capture = 0.0` is not an oversight -- see `namiashi_tuned_gaits_hold`.
const NAMIASHI_TUNED: [(GaitType, f64, f64, f64, f64, f64); 3] = [
    // gait, cycle_period_s, duty, max_step_m, swing_height_m, cmd_vx
    //
    // Retuned for the corrected 3.3 kg mass. Two rows moved:
    //   Trot 0.260 -> 0.320 s. At 0.260 the corrected model was airborne
    //     2.0-2.8% of the time with 4.5 deg of roll; 0.320 cuts that to
    //     0.5-1.3%. The 2.4 kg model was never airborne at either.
    //   Walk 0.400 -> 0.500 s, command 0.380 -> 0.330. T=0.400 has a failure
    //     band from ~0.37 to ~0.43 m/s that only exists with mass out at the
    //     thigh and calf; T=0.500 has none. Its ceiling is
    //     0.145/(0.500*0.75) = 0.387, so 0.330 keeps 15% of margin.
    // Crawl was 101-102% at every speed tried on every model and is unchanged.
    (GaitType::Trot, 0.320, 0.50, 0.145, 0.040, 0.800),
    (GaitType::Walk, 0.500, 0.75, 0.145, 0.035, 0.330),
    (GaitType::Crawl, 0.800, 0.85, 0.145, 0.040, 0.170),
];

fn namiashi_tuned_params(i: usize) -> WbcParams {
    let (gait, t, duty, step, h, cmd_vx) = NAMIASHI_TUNED[i];
    WbcParams {
        total_time_s: 26.0,
        burn_in_s: 1.0,
        cmd_vx,
        gait_type: Some(gait),
        cycle_period_s: Some(t),
        duty_factor: Some(duty),
        max_step_length_m: Some(step),
        swing_height_m: Some(h),
        // Deliberately zero. The library default is 0.05 s and it is what was
        // producing every symptom this file started with.
        k_capture_s: Some(0.0),
        ..WbcParams::forward_walk()
    }
}

/// REGRESSION: Trot, Walk and Crawl each hold for 25 s.
///
/// What the tuning came down to, in order of how much it mattered:
///
/// 1. **The step lengths were half what the geometry allows.** Every gait
///    was pinned to its own ceiling `v_max = max_step/(T*duty)` -- 0.50,
///    0.18 and 0.04 m/s for the library defaults. Walk answering a 0.50 m/s
///    command with 0.17 m/s was arithmetic, not a controller failure.
///    Normalised by the 0.306 m leg the defaults are 33/26/20%; 0.145 m
///    (47%) is the ratio the Go2 work settled on, and it holds here.
///
/// 2. **`k_capture` had to go to zero.** The library default of 0.05 s was
///    the single cause of three separate symptoms: forward speed overshooting
///    by up to 20%, Trot sliding sideways at 0.145 m/s, and Crawl yawing at
///    2.34 deg/s. All three vanish at k=0, where the open-loop Raibert plan
///    alone tracks to 101-103%. The initial guess was the opposite -- that
///    0.05 was 3.5x *below* the LIP value sqrt(h/g)=0.175 -- and raising it
///    made everything worse (Trot at k=0.175 makes 2% of its command and
///    yaws 43 deg). The sqrt(h/g) formula assumes the foothold acts within
///    one step; here it is filtered through the MPC horizon, which adds lag
///    the formula does not model.
///
/// 3. **Crawl's swing clearance was 5 mm.** Fine over its default 0.06 m
///    step, a scuff over 0.145 m: at fixed gain, 0.005 -> 0.020 m took Crawl
///    from 74% to 94% of command, and 0.040 m to 104%.
///
/// Not fixed, and deliberately so: turning tracks at a flat 76-77% of
/// command from 0.10 to 0.90 rad/s (see `namiashi_turn_rate_linearity`).
/// It is a constant gain with no saturation in range, which any outer
/// heading loop absorbs -- unlike the three items above, it does not stop
/// the gait from working.
#[test]
#[ignore = "25 s per gait -- run with --ignored"]
fn namiashi_tuned_gaits_hold() {
    for i in 0..NAMIASHI_TUNED.len() {
        let (gait, .., cmd_vx) = NAMIASHI_TUNED[i];
        let Some(samples) = run_wbc_sim(namiashi_tuned_params(i)) else {
            return;
        };
        let m = report_walk(&format!("{gait:?} tuned"), &samples, cmd_vx, 1.0);

        assert!(
            m.z_min > TRUNK_Z_FALL_THRESHOLD_M,
            "{gait:?}: trunk fell to {:.3} m",
            m.z_min
        );
        let track = m.body_vx / cmd_vx;
        assert!(
            (0.90..=1.10).contains(&track),
            "{gait:?}: tracked {:.0}% of {cmd_vx:.2} m/s (got {:.3})",
            100.0 * track,
            m.body_vx
        );
        // Body frame, so this is real sideways sliding and not the robot
        // having turned -- the run-start frame reported 0.23 m/s of "crab"
        // for Trot that was entirely -15 deg of yaw drift leaking in.
        assert!(
            m.body_vy.abs() < 0.05,
            "{gait:?}: slid sideways at {:.3} m/s",
            m.body_vy
        );
        // Nothing commands a turn. 1.5 deg/s is loose enough for the
        // 0.29-0.66 deg/s these settings produce and tight enough to catch
        // the 2.34 deg/s that k_capture=0.05 caused on Crawl.
        assert!(
            m.yaw_rate_deg_s.abs() < 1.5,
            "{gait:?}: yawed at {:.2} deg/s with wz=0",
            m.yaw_rate_deg_s
        );
    }
}

/// THE TUNING WAS DONE ON A ROBOT THAT WEIGHS 0.9 kg LESS THAN THE REAL ONE.
///
/// `namiashi.misa` came from a CAD export totalling 2.400 kg, 36% of it in
/// the legs. The built robot is 3.3 kg with 600 g per leg, so the legs are
/// 2.4 kg and the body is 0.9 kg: 73% of the machine is leg. That is not a
/// scale factor, it is an inversion of where the mass lives, and it lands on
/// the two assumptions this controller rests on.
///
/// The SRBD MPC treats the legs as massless -- all mass in one rigid trunk,
/// contact forces the only thing acting on it. At 36% leg that is a stretch;
/// at 73% the swinging legs carry more momentum than the body they are
/// supposed to be steering, and every swing reacts on the trunk directly.
/// The Raibert foothold plan has the same blind spot: `v * stance/2` says
/// where to put a massless foot, and says nothing about the impulse of
/// throwing 600 g forward and stopping it.
///
/// Three models, same tuned settings, so the comparison is only about mass:
///   - `namiashi.misa`      2.400 kg, 36% leg -- what the tuning was done on
///   - `namiashi_3p3_hip`   3.300 kg, 73% leg, added mass at the hip
///   - `namiashi_3p3_prop`  3.300 kg, 73% leg, added mass spread down the leg
///
/// hip-vs-prop is the same total and the same leg fraction, differing only in
/// how far out the mass sits. If the controller cares about mass at all, it
/// cares through leg inertia, and those two bracket it.
#[test]
#[ignore = "9 x 25 s runs -- run with --ignored"]
fn namiashi_mass_variants() {
    for model in [
        "namiashi.misa",
        "namiashi_3p3_hip.misa",
        "namiashi_3p3_prop.misa",
    ] {
        for i in 0..NAMIASHI_TUNED.len() {
            let (gait, .., cmd_vx) = NAMIASHI_TUNED[i];
            let params = WbcParams {
                misa_file: model,
                ..namiashi_tuned_params(i)
            };
            let Some(samples) = run_wbc_sim(params) else { return };
            let tag = model.trim_end_matches(".misa");
            report_walk(&format!("{gait:?} @{tag}"), &samples, cmd_vx, 1.0);
        }
    }
}

/// WHY prop BREAKS WALK, AND WHETHER SWING TIME BUYS IT BACK.
///
/// Correcting the mass to 3.3 kg costs nothing on its own: with the extra
/// mass at the hip, all three gaits track 100-101% and the yaw drift
/// actually halves. Spreading the same extra mass down the leg is what
/// hurts, and the two models differ by only 74 g per leg (thigh 20.7 -> 57.3
/// g, calf 21.3 -> 58.9 g). Walk falls to 82% of command and its three-foot
/// support collapses from 70.7% to 14.2% -- it stops being a walk and starts
/// running on two supports at an effective duty of 0.53 against a commanded
/// 0.75.
///
/// The ranking points at swing time. Required swing speed at each gait's
/// tuned command -- ground travel per stance, over the swing window -- is
/// Walk 1.14 m/s, Crawl 0.97, Trot 0.89, and they broke in exactly that
/// order. Walk has the shortest swing window of the three (0.400 s * 0.25 =
/// 0.100 s) despite being the slowest of the two fast gaits, because duty
/// 0.75 spends the cycle on the ground. A leg with three times the distal
/// mass has to be accelerated and stopped inside that window, and if it
/// arrives late the foot touches down late, which is exactly the low
/// measured duty.
///
/// If that is the mechanism, buying swing time fixes it, and the two ways to
/// buy it are a longer period and a lower duty. Both cost something: the
/// ceiling `max_step/(T*duty)` moves with each, so each row is checked
/// against its own ceiling rather than a fixed command.
#[test]
#[ignore = "re-tune sweep -- run with --ignored"]
fn namiashi_prop_walk_swing_time() {
    // T, duty -- swing window is T*(1-duty)
    let rows: &[(f64, f64)] = &[
        (0.400, 0.75), // the current tuning: 0.100 s of swing
        (0.500, 0.75), // 0.125 s, via period
        (0.600, 0.75), // 0.150 s, via period
        (0.400, 0.65), // 0.140 s, via duty
        (0.400, 0.55), // 0.180 s, via duty -- close to a trot
        (0.500, 0.65), // 0.175 s, both
    ];
    for &(t, duty) in rows {
        let step = 0.145;
        let ceil = step / (t * duty);
        let cmd_vx = 0.80 * ceil;
        let params = WbcParams {
            misa_file: "namiashi_3p3_prop.misa",
            total_time_s: 16.0,
            burn_in_s: 1.0,
            cmd_vx,
            gait_type: Some(GaitType::Walk),
            cycle_period_s: Some(t),
            duty_factor: Some(duty),
            max_step_length_m: Some(step),
            swing_height_m: Some(0.035),
            k_capture_s: Some(0.0),
            ..WbcParams::forward_walk()
        };
        let Some(samples) = run_wbc_sim(params) else { return };
        report_walk(
            &format!("Walk T={t:.3} d={duty:.2} swing={:.3}s", t * (1.0 - duty)),
            &samples,
            cmd_vx,
            1.0,
        );
    }
}

/// SWING TIME WAS THE WRONG VARIABLE, AND THE SWEEP THAT SAID SO WAS RIGGED.
///
/// `namiashi_prop_walk_swing_time` set each row's command to 80% of that
/// row's ceiling, so the command moved with the parameters. The two rows
/// that worked were the two slowest commands (0.309 and 0.258 m/s); every
/// row at 0.357 m/s or above failed, whatever its swing window. The row with
/// the most swing time of all (T=0.400, duty 0.55, 0.180 s) failed at
/// 0.527 m/s while a row with less (T=0.500, duty 0.75, 0.125 s) succeeded
/// at 0.309. That is a speed threshold wearing a parameter's clothes.
///
/// Two sweeps that do not confound:
///   A. hold T and duty at the tuning, walk the command up -- where does the
///      three-foot support actually go away?
///   B. hold the command at 0.38 m/s, vary T and duty among settings whose
///      ceiling clears it -- does any of them survive a speed that the
///      tuned setting cannot?
#[test]
#[ignore = "re-tune sweep -- run with --ignored"]
fn namiashi_prop_walk_speed_vs_shape() {
    let base = |t: f64, duty: f64, cmd_vx: f64| WbcParams {
        misa_file: "namiashi_3p3_prop.misa",
        total_time_s: 16.0,
        burn_in_s: 1.0,
        cmd_vx,
        gait_type: Some(GaitType::Walk),
        cycle_period_s: Some(t),
        duty_factor: Some(duty),
        max_step_length_m: Some(0.145),
        swing_height_m: Some(0.035),
        k_capture_s: Some(0.0),
        ..WbcParams::forward_walk()
    };

    eprintln!("---- A: speed sweep at the tuned shape (T=0.400, duty=0.75) ----");
    for cmd_vx in [0.20, 0.26, 0.31, 0.34, 0.38, 0.44] {
        let Some(samples) = run_wbc_sim(base(0.400, 0.75, cmd_vx)) else { return };
        report_walk(&format!("Walk v={cmd_vx:.2}"), &samples, cmd_vx, 1.0);
    }

    eprintln!("---- B: shape sweep at a fixed 0.38 m/s ----");
    // Every row's ceiling 0.145/(T*duty) clears 0.38.
    for &(t, duty) in &[
        (0.400, 0.75),
        (0.500, 0.75),
        (0.400, 0.65),
        (0.500, 0.65),
        (0.400, 0.55),
        (0.300, 0.75),
    ] {
        let Some(samples) = run_wbc_sim(base(t, duty, 0.38)) else { return };
        report_walk(&format!("Walk T={t:.3} d={duty:.2}"), &samples, 0.38, 1.0);
    }
}

/// IS THE BAD BAND REAL, AND IS THE PHASE OVERRIDE WHAT DIGS IT?
///
/// The speed sweep found a hole, not a limit: on the prop model at T=0.400 /
/// duty 0.75, Walk tracks 103/101/97/96% at 0.20-0.34 m/s, drops to 82% at
/// 0.38, and is back to 94% at 0.44. Every setting that failed anywhere
/// failed into the same state -- effective duty ~0.53 with 81-87% of the
/// time on two feet. Duty 0.53 on two supports is a trot. Walk is not
/// degrading, it is being captured by a different gait.
///
/// The only thing in this loop that can rewrite the gait pattern at runtime
/// is `ContactDrivenPhase`, which flips a leg to stance the moment its foot
/// reads above 5 N. That threshold was chosen for a 2.4 kg robot whose whole
/// leg weighed 217 g. A 600 g leg with 74 g more of it out at the thigh and
/// calf lands harder, so contacts that never reached 5 N now do -- and each
/// spurious flip shortens that leg's stance, which is the low measured duty.
///
/// Two questions, one sweep: is the band narrow (fine speed steps), and does
/// raising the override threshold out of reach close it (5 N vs 15 N vs off)?
#[test]
#[ignore = "diagnostic -- run with --ignored"]
fn namiashi_prop_walk_contact_override() {
    for thr in [5.0, 15.0, 1.0e9] {
        let label = if thr > 1.0e6 {
            "off".to_string()
        } else {
            format!("{thr:.0}N")
        };
        eprintln!("---- early-contact override = {label} ----");
        for cmd_vx in [0.34, 0.36, 0.38, 0.40, 0.42, 0.44] {
            let params = WbcParams {
                misa_file: "namiashi_3p3_prop.misa",
                total_time_s: 16.0,
                burn_in_s: 1.0,
                cmd_vx,
                gait_type: Some(GaitType::Walk),
                cycle_period_s: Some(0.400),
                duty_factor: Some(0.75),
                max_step_length_m: Some(0.145),
                swing_height_m: Some(0.035),
                k_capture_s: Some(0.0),
                early_contact_n: thr,
                ..WbcParams::forward_walk()
            };
            let Some(samples) = run_wbc_sim(params) else { return };
            report_walk(&format!("Walk v={cmd_vx:.2} thr={label}"), &samples, cmd_vx, 1.0);
        }
    }
}

/// NOT THE PHASE OVERRIDE EITHER -- AND IT IS A BAND, NOT A HOLE.
///
/// `ContactDrivenPhase` was the only thing in the loop that can rewrite the
/// gait pattern at runtime, so the guess was that heavier legs trip its 5 N
/// early-touchdown threshold spuriously. Disabling it entirely changes
/// nothing: at 0.38 m/s the prop model tracks 82% with the override at 5 N,
/// 83% at 15 N and 83% with it off. Every speed matches to within 1-3 points
/// across all three settings. The override is not involved.
///
/// The finer sweep also corrected the shape of the failure. It is not a hole
/// at 0.38 -- it is a band: 96% at 0.34, 95/88/89% at 0.36 (the edge),
/// 79-83% from 0.38 to 0.42, and back to 94-95% at 0.44. Roughly 0.37 to
/// 0.43 m/s, about 0.06 m/s wide, with clean walking on both sides.
///
/// Which leaves the question that actually matters for the design, since two
/// readings of the evidence so far are still alive:
///   - distal leg mass CREATES a band that the light model does not have; or
///   - a band is always there and the parameters only move it, and the 2.4 kg
///     tuning happened to sit beside it.
/// Those imply opposite fixes -- redistribute mass in hardware, versus pick a
/// period. Sweeping the same speeds on the hip model (same total mass, same
/// leg fraction, mass held proximal) and on prop at the period that escaped
/// (T=0.500) separates them.
#[test]
#[ignore = "diagnostic -- run with --ignored"]
fn namiashi_walk_band_is_mass_or_tuning() {
    let cases: &[(&str, &str, f64)] = &[
        ("namiashi_3p3_hip.misa", "hip T=0.400", 0.400),
        ("namiashi_3p3_prop.misa", "prop T=0.500", 0.500),
        ("namiashi.misa", "2.4kg T=0.400", 0.400),
    ];
    for &(model, label, t) in cases {
        eprintln!("---- {label} ----");
        for cmd_vx in [0.34, 0.36, 0.38, 0.40, 0.42, 0.44] {
            let params = WbcParams {
                misa_file: model,
                total_time_s: 16.0,
                burn_in_s: 1.0,
                cmd_vx,
                gait_type: Some(GaitType::Walk),
                cycle_period_s: Some(t),
                duty_factor: Some(0.75),
                max_step_length_m: Some(0.145),
                swing_height_m: Some(0.035),
                k_capture_s: Some(0.0),
                ..WbcParams::forward_walk()
            };
            let Some(samples) = run_wbc_sim(params) else { return };
            report_walk(&format!("{label} v={cmd_vx:.2}"), &samples, cmd_vx, 1.0);
        }
    }
}

/// RE-TUNE TROT AND CRAWL ON THE CORRECTED MASS.
///
/// The band belongs to distal leg mass: at T=0.400 the 2.4 kg model tracks
/// 101% flat from 0.34 to 0.44 m/s and the hip model 99-100%, while prop
/// drops to 79-82% across 0.38-0.42. Moving prop to T=0.500 removes it --
/// what looks like decay there (99 -> 87% as speed rises) is just the
/// ceiling: 0.145/(0.500*0.75) = 0.387 m/s, and 0.387/0.44 = 88% against 87%
/// measured, with three-foot support holding at 0.67.
///
/// So Walk moves to T=0.500 and a command safely under 0.387. Trot needs
/// checking too: on prop at the old tuning it overshot to 111% and was
/// airborne 2.6% of the time with 4.5 deg of roll, which the 2.4 kg model
/// never did. Crawl was 102% and untouched, but gets swept anyway rather
/// than assumed.
#[test]
#[ignore = "re-tune sweep -- run with --ignored"]
fn namiashi_prop_retune_trot_crawl() {
    eprintln!("---- Trot on prop: speed at T=0.260, and a longer period ----");
    for &(t, cmd_vx) in &[
        (0.260, 0.70),
        (0.260, 0.80),
        (0.260, 0.89),
        (0.320, 0.70),
        (0.320, 0.80),
        (0.320, 0.89),
    ] {
        let params = WbcParams {
            misa_file: "namiashi_3p3_prop.misa",
            total_time_s: 16.0,
            burn_in_s: 1.0,
            cmd_vx,
            gait_type: Some(GaitType::Trot),
            cycle_period_s: Some(t),
            duty_factor: Some(0.50),
            max_step_length_m: Some(0.145),
            swing_height_m: Some(0.040),
            k_capture_s: Some(0.0),
            ..WbcParams::forward_walk()
        };
        let Some(samples) = run_wbc_sim(params) else { return };
        report_walk(&format!("Trot T={t:.3} v={cmd_vx:.2}"), &samples, cmd_vx, 1.0);
    }

    eprintln!("---- Crawl on prop ----");
    for cmd_vx in [0.13, 0.17, 0.20] {
        let params = WbcParams {
            misa_file: "namiashi_3p3_prop.misa",
            total_time_s: 16.0,
            burn_in_s: 1.0,
            cmd_vx,
            gait_type: Some(GaitType::Crawl),
            cycle_period_s: Some(0.800),
            duty_factor: Some(0.85),
            max_step_length_m: Some(0.145),
            swing_height_m: Some(0.040),
            k_capture_s: Some(0.0),
            ..WbcParams::forward_walk()
        };
        let Some(samples) = run_wbc_sim(params) else { return };
        report_walk(&format!("Crawl v={cmd_vx:.2}"), &samples, cmd_vx, 1.0);
    }
}

/// IS THE DYNAMIC LAYER DOING ANYTHING?
///
/// An architecture review of this stack argued that the measured locomotion
/// is produced by the joint position-PD tracking IK targets, and that the MPC
/// and WBC contribute almost nothing. Its evidence is checkable and checks
/// out:
///
///   - `WbcPipeline::new` hardcodes `mass_kg: 9.0` and an inertia diagonal
///     for a 9 kg machine (`articara/src/wbc_pipeline.rs:250`), and this
///     harness never overrides either. namiashi is 3.3 kg. The `base_accel`
///     task carries the largest soft weight in the QP.
///   - The MPC's horizon contact schedule is `self.cfg.duty_factor > 0.5`
///     for every node past the first (`mpc_controller.rs:600`). Trot's duty
///     is exactly 0.50, so that is false, and the MPC plans all four legs
///     airborne for nine of its ten nodes. Walk and Crawl plan all four in
///     stance for all nine. Neither resembles the gait being walked.
///   - `wbc_pipeline.rs:515` is `let _ = (v_cmd_body, wz_cmd, omega_obs_world);`
///     and every attitude PD gain defaults to (0.0, 0.0).
///   - During stance `Footstep::stance_at` sweeps the foot from
///     `nominal + half` to `nominal - half` over the stance window, so under
///     no-slip `v_body = 2*half/T_st` identically. With open-loop Raibert
///     that is `v_cmd` exactly -- which is what 101-103% tracking and an
///     exactly-obeyed geometric ceiling look like.
///
/// The claim is falsifiable in one run: zero the WBC's torque feedforward and
/// leave the position-PD alone. If the gait is unchanged, every number in
/// this file describes the PD and IK layer, and the tuning conclusions are
/// statements about kinematics.
#[test]
#[ignore = "diagnostic -- run with --ignored"]
fn namiashi_is_the_dynamic_layer_load_bearing() {
    for kinematic_only in [false, true] {
        let tag = if kinematic_only { "PD only" } else { "WBC on" };
        for i in 0..NAMIASHI_TUNED.len() {
            let (gait, .., cmd_vx) = NAMIASHI_TUNED[i];
            let params = WbcParams {
                kinematic_only,
                total_time_s: 16.0,
                ..namiashi_tuned_params(i)
            };
            let Some(samples) = run_wbc_sim(params) else { return };
            report_walk(&format!("{gait:?} {tag}"), &samples, cmd_vx, 1.0);
        }
    }
}

/// THE BAND WAS AN ARTEFACT OF AN UNDER-MODELLED KNEE.
///
/// This test was written before the knee's 9:14 reduction was referred
/// through the model. The source URDF raised the calf's `effort` to 2.205
/// against the others' 1.5 -- a ratio of 1.470 where 14/9 is 1.5556 -- and
/// left `velocity` and `armature` at the same values as every other joint.
/// A reduction does all three: torque x14/9, speed x9/14, and reflected
/// rotor inertia x(14/9)^2. So the knee was carrying 2.4x too little
/// inertia.
///
/// With that corrected the speed dependence disappears. Walk on prop at
/// T=0.400 now holds three-foot support of only 0.16-0.21 at *every* speed
/// from 0.30 to 0.48 m/s, where before it held 0.61-0.64 below the band and
/// 0.65 above it. There is no band because there is no good region: T=0.400
/// simply cannot walk this leg. The tuned setting had already moved to
/// T=0.500, which holds 0.79.
///
/// The speed limit is not what binds, either -- knee velocity peaks at
/// 0.63-0.70 of its (now correct) 21.5 rad/s rating and never exceeds it.
/// The knee is torque-limited, and referring the inertia properly is what
/// made that visible: calf saturation went from 0-1% to 9-11% on every gait
/// despite the torque limit going *up* by 5.8%.
///
/// Original question, kept because the answer still stands:
///
/// DOES TORQUE SATURATION EXPLAIN THE WALK BAND?
///
/// `mujoco_sim` clamps every joint to its `effort` limit silently
/// (`src/mujoco_sim.rs:1121`), so a saturated actuator has been invisible in
/// every run in this file. Measuring it changes the picture: on the tuned
/// settings the thigh joint is clamped 22.5% of the time on Trot with the
/// corrected mass, 11.1% even on the original 2.4 kg model, and every joint
/// role reaches its limit at some point on every gait. namiashi's hip and
/// thigh are rated 1.5 N*m and its calf 2.205.
///
/// That is a candidate mechanism for the Walk band that neither swing time
/// nor the contact override could account for. If it is the cause, the
/// saturation fraction should spike inside 0.38-0.42 and fall away on both
/// sides, tracking the failure rather than rising monotonically with speed.
/// If it rises smoothly through the band, saturation is a separate (and
/// separately serious) problem and the band is still unexplained.
#[test]
#[ignore = "diagnostic -- run with --ignored"]
fn namiashi_walk_band_torque_saturation() {
    for cmd_vx in [0.30, 0.34, 0.38, 0.40, 0.42, 0.44, 0.48] {
        let params = WbcParams {
            misa_file: "namiashi_3p3_prop.misa",
            total_time_s: 16.0,
            burn_in_s: 1.0,
            cmd_vx,
            gait_type: Some(GaitType::Walk),
            cycle_period_s: Some(0.400),
            duty_factor: Some(0.75),
            max_step_length_m: Some(0.145),
            swing_height_m: Some(0.035),
            k_capture_s: Some(0.0),
            ..WbcParams::forward_walk()
        };
        let Some(samples) = run_wbc_sim(params) else { return };
        report_walk(&format!("Walk v={cmd_vx:.2}"), &samples, cmd_vx, 1.0);
    }
}

/// STEP 1 OF MAKING THE DYNAMIC LAYER REAL: GIVE THE WBC THE RIGHT ROBOT.
///
/// `namiashi_is_the_dynamic_layer_load_bearing` established that zeroing the
/// WBC's torque feedforward changes nothing -- Trot 106 -> 102%, Walk
/// 100 -> 101%, Crawl 101 -> 100%, and Walk's three-foot support actually
/// improves. That is a controller that is not controlling.
///
/// The most likely reason is that it is solving for the wrong machine.
/// `WbcPipeline::new` hardcodes `mass_kg: 9.0` and `inertia_diag_body:
/// (0.07, 0.26, 0.242)` -- a 9 kg robot -- and this harness has never
/// overridden either. namiashi is 3.3 kg. The `base_accel` task those feed
/// carries the largest soft weight in the QP, so at a static stand where the
/// MPC hands over roughly m*g, the WBC reads that as 32.4/9.0 - 9.81 =
/// -6.2 m/s^2 and spends its largest weight asking the trunk to accelerate
/// downward at 0.63 g, continuously.
///
/// One variable at a time: mass, CoM offset and composite inertia from the
/// same source the centroidal MPC config already uses, nothing else touched.
/// Then the same falsification test, to see whether the dynamic layer has
/// become load-bearing or merely better informed.
#[test]
#[ignore = "diagnostic -- run with --ignored"]
fn namiashi_wbc_real_inertia() {
    for real in [false, true] {
        for kinematic_only in [false, true] {
            let tag = match (real, kinematic_only) {
                (false, false) => "9kg WBC on",
                (false, true) => "9kg PD only",
                (true, false) => "real WBC on",
                (true, true) => "real PD only",
            };
            for i in 0..NAMIASHI_TUNED.len() {
                let (gait, .., cmd_vx) = NAMIASHI_TUNED[i];
                let params = WbcParams {
                    wbc_real_inertia: real,
                    kinematic_only,
                    total_time_s: 16.0,
                    ..namiashi_tuned_params(i)
                };
                let Some(samples) = run_wbc_sim(params) else { return };
                report_walk(&format!("{gait:?} {tag}"), &samples, cmd_vx, 1.0);
            }
        }
    }
}

/// STEP 3: STOP THE QP ANSWERING A THREE-CONTACT PLAN WITH TWO CONTACTS.
///
/// With the horizon schedule and the WBC's inertia both corrected, the
/// dynamic layer finally helps Trot -- peak roll 3.8 deg with its torque
/// zeroed against 2.0 deg with it on. Walk went the other way: three-foot
/// support 0.842 zeroed against 0.608 applied, and thigh saturation doubled.
/// Walk is duty 0.75, so three feet are down by construction; a controller
/// that costs three-foot support on that gait is doing something specific.
///
/// The friction pyramid constrains stance feet to push and not pull, and
/// nothing more. With three or four contacts the GRF allocation is redundant
/// and a two-contact solution satisfies every constraint in the QP -- the
/// only thing arguing against it is a low-weight regulariser toward the
/// MPC's plan. 0.608 is what choosing that vertex looks like from outside.
///
/// So demand a floor. Expressed as a fraction of the static per-foot share
/// `m*g/4` so it means the same on every model. It is a hard constraint at
/// priority 0: too large and the touchdown transient, where the commanded
/// contact set and the physical one disagree for a tick, becomes infeasible
/// rather than merely wrong. The sweep is there to find where that starts.
#[test]
#[ignore = "diagnostic -- run with --ignored"]
fn namiashi_min_stance_force() {
    for frac in [0.0, 0.05, 0.10, 0.20, 0.35] {
        for i in 0..NAMIASHI_TUNED.len() {
            let (gait, .., cmd_vx) = NAMIASHI_TUNED[i];
            let params = WbcParams {
                wbc_real_inertia: true,
                f_min_stance_frac: frac,
                total_time_s: 16.0,
                ..namiashi_tuned_params(i)
            };
            let Some(samples) = run_wbc_sim(params) else { return };
            report_walk(&format!("{gait:?} fmin={frac:.2}"), &samples, cmd_vx, 1.0);
        }
    }
}

/// STEP 4: IS THE BASE-ACCEL REFERENCE WHAT COSTS WALK ITS SUPPORT?
///
/// The minimum-normal-force floor did nothing. Walk's three-foot support sits
/// at 0.606-0.610 whether the floor is 0% or 35% of the static per-foot
/// share, so the QP was never picking a two-contact vertex -- it was already
/// loading every stance foot, and the constraint never binds. The support is
/// being lost in the physics, not in the solution.
///
/// Which moves the suspicion up one level, to what the torque is being asked
/// to achieve. `base_accel` carries the MPC's predicted base acceleration and
/// is the largest soft weight in the QP at 200, against 1 for swing-leg and 5
/// for contact-force and tau-gravity. If that reference is wrong for Walk,
/// the WBC spends most of its authority chasing it and unloads a foot doing
/// so.
///
/// Sweeping the weight to zero turns the WBC into gravity compensation plus
/// the low-weight terms, without disabling it the way `kinematic_only` does.
/// If Walk's support climbs back toward the 0.842 it reaches with the torque
/// zeroed, the reference is the problem. If it stays at 0.61, the reference
/// is innocent and something in the priority-0 block is responsible.
#[test]
#[ignore = "diagnostic -- run with --ignored"]
fn namiashi_base_accel_weight() {
    for w in [200.0, 50.0, 10.0, 0.0] {
        for i in 0..NAMIASHI_TUNED.len() {
            let (gait, .., cmd_vx) = NAMIASHI_TUNED[i];
            let params = WbcParams {
                wbc_real_inertia: true,
                base_accel_weight: Some(w),
                total_time_s: 16.0,
                ..namiashi_tuned_params(i)
            };
            let Some(samples) = run_wbc_sim(params) else { return };
            report_walk(&format!("{gait:?} w={w:.0}"), &samples, cmd_vx, 1.0);
        }
    }
}

/// STEP 5: THE FLAGS THE WBC IS GIVEN DISAGREE WITH EACH OTHER.
///
/// Two suspects down. A minimum normal force on stance feet moves Walk's
/// three-foot support from 0.608 to 0.610 across 0-35% of the static share,
/// so the QP was never picking a two-contact vertex. Dropping `base_accel`
/// from its default weight of 200 to zero moves it to 0.626, so the MPC's
/// predicted base acceleration is not what the torque is being spent on
/// either. Neither comes near the 0.842 the same gait reaches with the WBC's
/// torque zeroed outright.
///
/// That leaves priority 0, and there is a specific inconsistency there.
/// `ContactDrivenPhase::apply_correction` is called with a late-liftoff
/// threshold of 0, so it is monotone: it can add stance legs, never remove
/// them. When a foot touches down early, `contact_flag[i]` flips to true
/// while `gait_out.legs[i].phase.is_stance` stays false. That leg then gets,
/// simultaneously:
///
///   - `no_contact_motion`, a priority-0 hard equality gated on
///     `contact_flag`, nailing the foot in place;
///   - the swing-tracking cost, gated on the *nominal* phase, still asking
///     it to follow an advancing swing arc;
///   - the joint position-PD at kp=100, still driving q* along that arc.
///
/// Priority 0 wins. The foot is pinned while two other terms drag it, which
/// unloads it -- and an unloaded foot is exactly what a support census reads
/// as airborne.
///
/// This needs no code change to test: raising the early-touchdown threshold
/// out of reach removes the correction, and with it the disagreement.
#[test]
#[ignore = "diagnostic -- run with --ignored"]
fn namiashi_contact_flag_consistency() {
    for (thr, tag) in [(5.0, "5N"), (1.0e9, "off")] {
        for kinematic_only in [false, true] {
            let k = if kinematic_only { "PD only" } else { "WBC on" };
            for i in 0..NAMIASHI_TUNED.len() {
                let (gait, .., cmd_vx) = NAMIASHI_TUNED[i];
                let params = WbcParams {
                    wbc_real_inertia: true,
                    early_contact_n: thr,
                    kinematic_only,
                    total_time_s: 16.0,
                    ..namiashi_tuned_params(i)
                };
                let Some(samples) = run_wbc_sim(params) else { return };
                report_walk(&format!("{gait:?} {tag} {k}"), &samples, cmd_vx, 1.0);
            }
        }
    }
}

/// STEP 6: THE WBC IS UNLOADING THE FRONT FEET SPECIFICALLY.
///
/// Removing `ContactDrivenPhase` moved Walk's three-foot support from 0.608
/// to 0.585 -- the wrong direction, so the flag disagreement is not it
/// either. But the per-foot duty in that run says what is happening:
///
///     Walk, WBC torque on   0.54 0.53 | 0.79 0.78
///     Walk, WBC torque off  0.68 0.66 | 0.79 0.79
///
/// The rear pair is identical to two decimal places. Only the front pair
/// drops. The WBC is not degrading the gait, it is shifting load rearward.
///
/// Of the terms that can distribute force front-to-rear, `base_accel` is
/// already ruled out (weight 200 -> 0 moved support by 0.018). `contact_force`
/// is the other one: at weight 5 it pulls the QP's own GRF solution toward
/// the MPC's prediction, and the MPC computes its moment arms from the body
/// root while the WBC -- now that `centroidal_inertia_body` is set -- computes
/// its predicted acceleration from the CoM, 15.9 mm below the root on this
/// model. If the MPC's allocation is front-light, the WBC is faithfully
/// reproducing it.
#[test]
#[ignore = "diagnostic -- run with --ignored"]
fn namiashi_contact_force_weight() {
    for w in [5.0, 1.0, 0.0] {
        let params = WbcParams {
            wbc_real_inertia: true,
            contact_force_weight: Some(w),
            total_time_s: 16.0,
            ..namiashi_tuned_params(1)
        };
        let Some(samples) = run_wbc_sim(params) else { return };
        report_walk(&format!("Walk cf={w:.0}"), &samples, NAMIASHI_TUNED[1].5, 1.0);
    }
    // Both off: what is left is gravity comp plus the swing-leg term.
    let params = WbcParams {
        wbc_real_inertia: true,
        contact_force_weight: Some(0.0),
        base_accel_weight: Some(0.0),
        total_time_s: 16.0,
        ..namiashi_tuned_params(1)
    };
    if let Some(samples) = run_wbc_sim(params) {
        report_walk("Walk cf=0 ba=0", &samples, NAMIASHI_TUNED[1].5, 1.0);
    }
}

/// STEP 7: WHICH THIRD OF THE "REAL INERTIA" CHANGE COSTS WALK ITS FRONT FEET?
///
/// Correcting the MPC's own inertia -- it was using the heaviest link's
/// tensor, 12 to 24 times under the composite -- moved Walk's three-foot
/// support from 0.725 to 0.745 with the WBC still on its 9 kg placeholder,
/// and its front duty from 0.54/0.53 back to 0.63/0.59. But with the WBC
/// corrected too it sits at 0.592. Something in the WBC-side correction is
/// undoing it.
///
/// `wbc_real_inertia` is three changes at once, and they are not equivalent:
///   - mass 9.0 -> 3.3 kg scales `a_base_des` by 2.7 across the board;
///   - the CoM offset moves where moments are taken about;
///   - setting `centroidal_inertia_body` also switches the pipeline from
///     `predicted_base_accel_world` to its centroidal variant, which is a
///     different function, not just a different constant.
///
/// Bundling them is how the earlier one-at-a-time sweeps kept missing:
/// `base_accel` and `contact_force` turned out to be jointly responsible
/// (0.608 with both on, 0.655 and 0.626 with either alone, 0.819 with both
/// off) because they carry the same MPC prediction down two paths. Same
/// discipline applies here.
#[test]
#[ignore = "diagnostic -- run with --ignored"]
fn namiashi_wbc_inertia_decomposition() {
    let cases: &[(&str, bool, bool, bool)] = &[
        ("none", false, false, false),
        ("mass", true, false, false),
        ("com", false, true, false),
        ("inertia", false, false, true),
        ("all", true, true, true),
    ];
    for &(tag, m, c, i) in cases {
        let params = WbcParams {
            wbc_real_mass_only: m,
            wbc_real_com_only: c,
            wbc_real_inertia_only: i,
            total_time_s: 16.0,
            ..namiashi_tuned_params(1)
        };
        let Some(samples) = run_wbc_sim(params) else { return };
        report_walk(&format!("Walk {tag}"), &samples, NAMIASHI_TUNED[1].5, 1.0);
    }
}

/// VIDEO SOURCE: the three tuned gaits, and the four commands.
///
/// Writes a replay trace per run under `NAMIASHI_REPLAY_OUT/<label>/`, which
/// `render_namiashi.py` turns into an MP4. Nothing here asserts -- the
/// assertions live in `namiashi_tuned_gaits_hold`; this exists so the video
/// and the regression are demonstrably the same configuration, taken from
/// `NAMIASHI_TUNED` rather than restated.
///
/// Run as:
///   NAMIASHI_REPLAY_OUT=/tmp/namiashi_replay cargo test --release \
///     --no-default-features --features mujoco --test wbc_walk \
///     namiashi_video_source -- --ignored --nocapture
#[test]
#[ignore = "video source -- needs NAMIASHI_REPLAY_OUT"]
fn namiashi_video_source() {
    let Ok(root) = std::env::var("NAMIASHI_REPLAY_OUT") else {
        eprintln!("NAMIASHI_REPLAY_OUT unset -- nothing to record");
        return;
    };

    // Part 1: each tuned gait walking forward.
    for i in 0..NAMIASHI_TUNED.len() {
        let (gait, .., cmd_vx) = NAMIASHI_TUNED[i];
        let label = format!("{gait:?}").to_lowercase();
        let params = WbcParams {
            total_time_s: 13.0,
            replay_dir: Some(format!("{root}/{label}")),
            ..namiashi_tuned_params(i)
        };
        let Some(samples) = run_wbc_sim(params) else { return };
        report_walk(&format!("{gait:?} video"), &samples, cmd_vx, 1.0);
    }

    // Part 2: Trot answering all four commands, so the turn is visible. The
    // yaw moment arm fix took turning from 76% of command to 100%, and that
    // is the one result a still number does not convey.
    let (_, t, duty, step, h, fwd) = NAMIASHI_TUNED[0];
    for (tag, vx, vy, wz) in [
        ("cmd_fwd", fwd, 0.0, 0.0),
        ("cmd_back", -fwd, 0.0, 0.0),
        ("cmd_strafe", 0.0, 0.45, 0.0),
        ("cmd_turn", 0.0, 0.0, 0.60),
    ] {
        let params = WbcParams {
            replay_dir: Some(format!("{root}/{tag}")),
            total_time_s: 11.0,
            burn_in_s: 1.0,
            cmd_vx: vx,
            cmd_vy: vy,
            cmd_wz: wz,
            gait_type: Some(GaitType::Trot),
            cycle_period_s: Some(t),
            duty_factor: Some(duty),
            max_step_length_m: Some(step),
            swing_height_m: Some(h),
            k_capture_s: Some(0.0),
            ..WbcParams::forward_walk()
        };
        let Some(samples) = run_wbc_sim(params) else { return };
        report_walk_cmd(&format!("Trot {tag}"), &samples, vx, vy, wz, 1.0);
    }
}

/// HOW LOW CAN namiashi STAND AND STILL WALK?
///
/// Every result so far is at one stance height, the 0.295 m the harness has
/// always used. Crouching is not a free parameter: it rotates the leg
/// Jacobian, and the same foot force then needs a different split between
/// thigh and knee. On this robot that matters more than usual, because the
/// thigh is already the joint that saturates -- 24% of the time on Trot
/// against a 1.5 N*m limit.
///
/// It also eats the step-length budget from the other end. `max_step_length_m`
/// is 0.145 m against a 0.306 m leg, and a crouched leg has less horizontal
/// reach before it runs out of extension, so a drop that leaves the gait
/// geometrically fine can still leave it kinematically infeasible.
///
/// Four heights, three gaits, tuned settings otherwise unchanged. `drop` here
/// is absolute, measured from the harness's original ~0.295 m stance -- it
/// overrides `NAMIASHI_STANCE_DROP_M` rather than adding to it, so the 0 cm
/// row is the old baseline and the 6 cm row is the current one.
#[test]
#[ignore = "12 runs -- run with --ignored"]
fn namiashi_trunk_height_sweep() {
    for drop in [0.0, 0.02, 0.04, 0.06] {
        for i in 0..NAMIASHI_TUNED.len() {
            let (gait, .., cmd_vx) = NAMIASHI_TUNED[i];
            let params = WbcParams {
                trunk_drop_m: drop,
                total_time_s: 16.0,
                ..namiashi_tuned_params(i)
            };
            let Some(samples) = run_wbc_sim(params) else { return };
            report_walk(
                &format!("{gait:?} drop={:.0}cm", drop * 100.0),
                &samples,
                cmd_vx,
                1.0,
            );
        }
    }
}

/// VIDEO SOURCE: the same gait at four stance heights.
///
/// Records all three tuned gaits at 0 / 2 / 4 / 6 cm of crouch for
/// `render_namiashi_height.py` to show as a 2x2 grid. Same settings as
/// `NAMIASHI_TUNED` otherwise, so the comparison is only about height.
///
/// Run as:
///   NAMIASHI_REPLAY_OUT=/tmp/namiashi_height cargo test --release \
///     --no-default-features --features mujoco --test wbc_walk \
///     namiashi_height_video_source -- --ignored --nocapture
#[test]
#[ignore = "video source -- needs NAMIASHI_REPLAY_OUT"]
fn namiashi_height_video_source() {
    let Ok(root) = std::env::var("NAMIASHI_REPLAY_OUT") else {
        eprintln!("NAMIASHI_REPLAY_OUT unset -- nothing to record");
        return;
    };
    for i in 0..NAMIASHI_TUNED.len() {
        let (gait, .., cmd_vx) = NAMIASHI_TUNED[i];
        for drop in [0.0, 0.02, 0.04, 0.06] {
            let tag = format!(
                "{}_{:.0}cm",
                format!("{gait:?}").to_lowercase(),
                drop * 100.0
            );
            let params = WbcParams {
                trunk_drop_m: drop,
                total_time_s: 12.0,
                replay_dir: Some(format!("{root}/{tag}")),
                ..namiashi_tuned_params(i)
            };
            let Some(samples) = run_wbc_sim(params) else { return };
            report_walk(&format!("{gait:?} {tag}"), &samples, cmd_vx, 1.0);
        }
    }
}

/// DOES THE CONTROLLER NEED A BOUNDED ABSOLUTE BASE POSITION?
///
/// It reads one -- `WbcPipeline::solve` takes `body_world_position` straight
/// from the simulator and writes it into the floating base's `q[0..3]`. On
/// hardware there is nothing to read it from. IMU integration drifts without
/// bound, and a legged state estimator gives good orientation and velocity
/// but a position that walks away, so if this were a real dependency it would
/// be a blocker.
///
/// It should not be. Rigid-body dynamics is translation invariant: `M(q)`,
/// the nonlinear terms and the contact Jacobians depend on base *orientation*
/// and joint angles, not on where the base is. The two places the position
/// appears downstream are moment arms, `foot_pos_world - body_pos_w` and the
/// CoM variant, where it cancels.
///
/// Also worth noting: nothing calls `set_body_pose_observed`, so the MPC's
/// own `world_position` is never updated from observation at all, and its
/// reference trajectory is built from that -- self-referential, hence
/// invariant too.
///
/// Reasoning is not measurement. A constant 1 km offset tests invariance; a
/// 0.10 m/s drift is what an unaided estimator actually does, and over 16 s
/// that is 1.6 m of accumulated error.
#[test]
#[ignore = "diagnostic -- run with --ignored"]
fn namiashi_absolute_position_dependence() {
    let cases: &[(&str, [f64; 3], [f64; 3])] = &[
        ("none", [0.0; 3], [0.0; 3]),
        ("bias 1km", [1000.0, -1000.0, 0.0], [0.0; 3]),
        ("bias 1km + z", [1000.0, -1000.0, 5.0], [0.0; 3]),
        ("drift 0.1m/s", [0.0; 3], [0.1, -0.1, 0.0]),
        ("drift 0.5m/s", [0.0; 3], [0.5, -0.5, 0.05]),
    ];
    for &(tag, bias, drift) in cases {
        for i in 0..NAMIASHI_TUNED.len() {
            let (gait, .., cmd_vx) = NAMIASHI_TUNED[i];
            let params = WbcParams {
                base_pos_bias_m: bias,
                base_pos_drift_mps: drift,
                total_time_s: 16.0,
                ..namiashi_tuned_params(i)
            };
            let Some(samples) = run_wbc_sim(params) else { return };
            report_walk(&format!("{gait:?} {tag}"), &samples, cmd_vx, 1.0);
        }
    }
}

/// HOW MUCH VELOCITY SENSING DOES THIS CONTROLLER ACTUALLY NEED?
///
/// Every result in this file was measured with simulator ground truth for the
/// body velocity: exact, noiseless, zero lag. namiashi will have an IMU and
/// nothing else, and an IMU cannot give a bounded velocity. The horizontal
/// acceleration comes out as `R(q_hat) * a_meas - g`, so an attitude error
/// theta leaks `g*sin(theta)` of phantom acceleration -- 0.17 m/s^2 at one
/// degree, which integrates to 0.85 m/s in five seconds against a 0.80 m/s
/// command. Accelerometer bias, at 0.005-0.02 m/s^2 for a good part, is the
/// smaller problem. Better hardware does not fix this; it is the structure.
///
/// The standard answer is leg odometry -- a loaded foot is stationary in the
/// world, so `v_body = -J(q) q_dot` is a velocity measurement, and the
/// 18-state KF in `legged-estimation` fuses exactly that with the IMU. But it
/// wants a per-foot contact flag, which is the sensor namiashi is not going
/// to carry.
///
/// Before building any of that, the cheaper question: what breaks without it?
/// `k_capture` is already 0, so footstep placement does not read the velocity
/// at all. What is left is the MPC's state feedback, and zeroing both paths
/// its prediction reaches the torque by cost Walk almost nothing.
///
/// Five conditions, from ground truth down to no velocity sensing whatsoever.
#[test]
#[ignore = "diagnostic -- run with --ignored"]
fn namiashi_velocity_observation_dependence() {
    let cases: &[(&str, VelObs)] = &[
        ("truth", VelObs::Truth),
        ("lag 20ms", VelObs::Lag(0.020)),
        ("lag 50ms", VelObs::Lag(0.050)),
        ("bias 0.3m/s", VelObs::Bias(0.3, 0.15)),
        ("open loop", VelObs::Command),
        ("zero", VelObs::Zero),
    ];
    for &(tag, vo) in cases {
        for i in 0..NAMIASHI_TUNED.len() {
            let (gait, .., cmd_vx) = NAMIASHI_TUNED[i];
            let params = WbcParams {
                vel_obs: vo,
                total_time_s: 16.0,
                ..namiashi_tuned_params(i)
            };
            let Some(samples) = run_wbc_sim(params) else { return };
            report_walk(&format!("{gait:?} {tag}"), &samples, cmd_vx, 1.0);
        }
    }
}

/// THE SENSOR SET namiashi WILL ACTUALLY HAVE.
///
/// Two dependencies have already been measured away. Absolute base position
/// is not used at all -- a 1 km offset and an 8 m accumulated drift leave
/// every reported figure bit-identical, because rigid-body dynamics is
/// translation invariant and the only place the position appears downstream
/// is a moment arm where it cancels. Body velocity is barely used -- with
/// `k_capture` at 0 the footstep plan does not read it, and feeding a
/// constant zero still tracks 101-102%.
///
/// One is left. `ContactDrivenPhase` takes a per-foot normal force and flips
/// a leg to stance the moment it reads above 5 N, and that flag decides the
/// WBC's priority-0 constraints. namiashi will not carry foot force sensors.
///
/// So run the configuration the hardware can actually supply: joint encoders
/// and an IMU, nothing else. Contact state from the gait schedule's nominal
/// phase, no velocity estimate at all. Against the same settings with ground
/// truth, at the tuned stance, for 25 s.
#[test]
#[ignore = "long -- run with --ignored"]
fn namiashi_hardware_sensor_set() {
    // label, contact-force threshold, velocity observation
    let cases: &[(&str, f64, VelObs)] = &[
        ("truth + contact", 5.0, VelObs::Truth),
        ("truth, no contact", 1.0e9, VelObs::Truth),
        ("no vel, contact", 5.0, VelObs::Zero),
        ("IMU+enc only", 1.0e9, VelObs::Zero),
    ];
    for &(tag, thr, vo) in cases {
        for i in 0..NAMIASHI_TUNED.len() {
            let (gait, .., cmd_vx) = NAMIASHI_TUNED[i];
            let params = WbcParams {
                early_contact_n: thr,
                vel_obs: vo,
                total_time_s: 26.0,
                ..namiashi_tuned_params(i)
            };
            let Some(samples) = run_wbc_sim(params) else { return };
            report_walk(&format!("{gait:?} | {tag}"), &samples, cmd_vx, 1.0);
        }
    }
}

/// CAN THIS RUN THROUGH A SPEED-MODE DRIVER?
///
/// The LKMTech MG4005 has no MIT mode. Its CAN protocol offers closed-loop
/// position, closed-loop speed and closed-loop iq, so the
/// position-target-plus-torque-feedforward interface every result in this
/// file was produced with does not exist on the part. Speed mode is the first
/// choice over iq because torque mode moves the whole stabilising loop onto
/// the host, where bus latency and host rate set the stability margin
/// directly.
///
/// Two gains decide whether it works, and they are different things:
///
///   `loop_kv`  stands in for the driver's own speed loop. The model ships
///              `actuator_kv = 1.2`, which is a position-mode damping value:
///              reaching the 1.5 N*m limit would need 1.25 rad/s of velocity
///              error. A real speed loop is far stiffer, and has integral
///              action this proportional stand-in does not.
///   `k_track`  is the outer position loop the host has to add. A speed loop
///              has no position feedback, so trajectory velocity alone tracks
///              the right rate while absolute position walks away.
///
/// Swept against the position path at the same stance and settings.
#[test]
#[ignore = "sweep -- run with --ignored"]
fn namiashi_velocity_actuation_sweep() {
    eprintln!("---- reference: position target + torque feedforward ----");
    for i in 0..NAMIASHI_TUNED.len() {
        let (gait, .., cmd_vx) = NAMIASHI_TUNED[i];
        let params = WbcParams {
            total_time_s: 16.0,
            ..namiashi_tuned_params(i)
        };
        let Some(samples) = run_wbc_sim(params) else { return };
        report_walk(&format!("{gait:?} pos+tau"), &samples, cmd_vx, 1.0);
    }

    eprintln!("---- speed mode: loop stiffness x position-loop gain ----");
    for loop_kv in [1.2, 5.0, 20.0] {
        for k_track in [0.0, 20.0, 60.0] {
            for i in 0..NAMIASHI_TUNED.len() {
                let (gait, .., cmd_vx) = NAMIASHI_TUNED[i];
                let params = WbcParams {
                    actuation: Actuation::Velocity { k_track, loop_kv, loop_ki: 0.0 },
                    total_time_s: 16.0,
                    ..namiashi_tuned_params(i)
                };
                let Some(samples) = run_wbc_sim(params) else { return };
                report_walk(
                    &format!("{gait:?} kv={loop_kv:.1} kt={k_track:.0}"),
                    &samples,
                    cmd_vx,
                    1.0,
                );
            }
        }
    }
}

/// SPEED MODE, ZOOMED IN ON WHAT THE FIRST SWEEP FOUND.
///
/// Three things came out of it. Trajectory velocity alone does not stand the
/// robot up -- at `k_track = 0` all three gaits end the run at a trunk height
/// of 0.041 m, which is lying down. A speed loop has no position feedback, so
/// the outer loop is not optional.
///
/// `k_track = 60` gets Walk and Crawl to 103% of command, but Trot only to
/// 93% and with 3 deg/s of yaw drift against 0.39 on the position path.
///
/// And a *stiffer* driver loop is worse, not better: at `k_track = 60`,
/// `loop_kv` of 1.2 / 5 / 20 gives 93 / 91 / 90% on Trot and 103 / 97 / 90%
/// on Walk. Worth understanding rather than just noting -- a stiff
/// proportional speed loop reaches the 1.5 N*m clamp on a small velocity
/// error, so it spends most of its time saturated and stops being
/// proportional at all.
///
/// So: hold the loop soft and push the outer gain. `k_track` is in 1/s -- 60
/// means a 0.01 rad position error asks for 0.6 rad/s -- so this is also
/// asking how much outer-loop bandwidth the host has to supply, which on a
/// CAN bus is a real constraint and not just a number.
#[test]
#[ignore = "sweep -- run with --ignored"]
fn namiashi_velocity_actuation_zoom() {
    for loop_kv in [1.2, 2.5] {
        for k_track in [60.0, 100.0, 150.0, 220.0] {
            for i in 0..NAMIASHI_TUNED.len() {
                let (gait, .., cmd_vx) = NAMIASHI_TUNED[i];
                let params = WbcParams {
                    actuation: Actuation::Velocity { k_track, loop_kv, loop_ki: 0.0 },
                    total_time_s: 16.0,
                    ..namiashi_tuned_params(i)
                };
                let Some(samples) = run_wbc_sim(params) else { return };
                report_walk(
                    &format!("{gait:?} kv={loop_kv:.1} kt={k_track:.0}"),
                    &samples,
                    cmd_vx,
                    1.0,
                );
            }
        }
    }
}

/// SPEED MODE AGAINST TORQUE MODE, AT HOST RATES A CAN BUS CAN ACTUALLY HOLD.
///
/// Both are on the MG4005. The choice between them is not about which control
/// law is better -- at the same update rate they are close to the same law,
/// since the torque path is just the driver's PD computed host-side. The
/// choice is about *where the loop lives*.
///
/// A speed-mode driver closes its inner loop internally, on fresh encoder
/// data, at several kHz, whatever the host is doing. The host only has to
/// shape the trajectory and supply the outer position term. In torque mode
/// there is no inner loop at all: the last torque the host sent is held until
/// the next one arrives, so host rate and bus latency set the stability
/// margin directly.
///
/// Everything measured in this file so far ran the controller at 500 Hz, the
/// physics rate. A CAN bus with twelve motors on it does not. So sweep the
/// host rate and watch which path degrades first.
#[test]
#[ignore = "sweep -- run with --ignored"]
fn namiashi_speed_vs_torque_host_rate() {
    for hz in [500.0, 200.0, 100.0] {
        for i in 0..NAMIASHI_TUNED.len() {
            let (gait, .., cmd_vx) = NAMIASHI_TUNED[i];
            for (tag, act) in [
                ("speed", Actuation::Velocity { k_track: 100.0, loop_kv: 1.2, loop_ki: 0.0 }),
                // kp/kd are the model's own actuator gains, so the torque path
                // reproduces the driver's PD exactly and only the rate differs.
                ("torque", Actuation::Torque { kp: 100.0, kd: 1.2 }),
            ] {
                let params = WbcParams {
                    actuation: act,
                    host_rate_hz: Some(hz),
                    total_time_s: 16.0,
                    ..namiashi_tuned_params(i)
                };
                let Some(samples) = run_wbc_sim(params) else { return };
                report_walk(
                    &format!("{gait:?} {tag} {hz:.0}Hz"),
                    &samples,
                    cmd_vx,
                    1.0,
                );
            }
        }
    }
}

/// VIDEO SOURCE: speed mode against torque mode, per gait.
///
/// Both interfaces the MG4005 offers, at the 400 Hz the bus is designed for.
/// The speed side uses the ideal-velocity-source model, since that is what an
/// 8-16 kHz driver loop looks like from a 400 Hz host, and `k_track = 40`,
/// which is the value that suits all three gaits under that model.
#[test]
#[ignore = "video source -- needs NAMIASHI_REPLAY_OUT"]
fn namiashi_actuation_video_source() {
    let Ok(root) = std::env::var("NAMIASHI_REPLAY_OUT") else {
        eprintln!("NAMIASHI_REPLAY_OUT unset -- nothing to record");
        return;
    };
    for i in 0..NAMIASHI_TUNED.len() {
        let (gait, .., cmd_vx) = NAMIASHI_TUNED[i];
        for (tag, act) in [
            ("speed", Actuation::VelocityIdeal { k_track: 40.0 }),
            ("torque", Actuation::Torque { kp: 100.0, kd: 1.2 }),
        ] {
            let g = format!("{gait:?}").to_lowercase();
            let params = WbcParams {
                actuation: act,
                host_rate_hz: Some(400.0),
                dt: 0.0005,
                total_time_s: 12.0,
                replay_dir: Some(format!("{root}/{g}_{tag}")),
                ..namiashi_tuned_params(i)
            };
            let Some(samples) = run_wbc_sim(params) else { return };
            report_walk(&format!("{gait:?} {tag}"), &samples, cmd_vx, 1.0);
        }
    }
}

/// WHAT HOST RATE DOES TORQUE MODE NEED, PER GAIT?
///
/// The coarse sweep put the breakdown between 200 Hz, where torque mode holds
/// 97-102% on all three gaits, and 100 Hz, where it falls to 73-90%. The bus
/// is expected to reach 400 Hz, so the question is which gaits have margin
/// there and which are running near their edge.
///
/// Torque mode has no inner loop: the last torque the host sent is held until
/// the next arrives, so the host period is dead time inside the only loop
/// there is. The rate a gait needs should therefore scale with how fast that
/// gait moves -- Trot's 0.320 s period against Crawl's 0.800 s -- and that is
/// worth checking rather than assuming, because the position loop's own
/// bandwidth (kp=100, kd=1.2) is the same for all three and may be what binds
/// instead.
///
/// Speed mode is swept alongside at the same rates, since the comparison at
/// 400 Hz is the decision actually being made.
#[test]
#[ignore = "sweep -- run with --ignored"]
fn namiashi_host_rate_requirement() {
    // 2 kHz physics so every rate below is an exact integer divisor. At the
    // usual 0.002 s the gate quantises hard -- 400 Hz became 500 and 200
    // became 167 -- and the sweep silently reported duplicate columns.
    const SWEEP_DT: f64 = 0.0005;
    for hz in [500.0, 400.0, 333.0, 250.0, 200.0, 143.0, 125.0, 100.0] {
        for i in 0..NAMIASHI_TUNED.len() {
            let (gait, .., cmd_vx) = NAMIASHI_TUNED[i];
            for (tag, act) in [
                ("torque", Actuation::Torque { kp: 100.0, kd: 1.2 }),
                ("speed", Actuation::Velocity { k_track: 100.0, loop_kv: 1.2, loop_ki: 0.0 }),
            ] {
                let params = WbcParams {
                    actuation: act,
                    host_rate_hz: Some(hz),
                    dt: SWEEP_DT,
                    total_time_s: 16.0,
                    ..namiashi_tuned_params(i)
                };
                let Some(samples) = run_wbc_sim(params) else { return };
                report_walk(
                    &format!("{gait:?} {tag} {hz:.0}"),
                    &samples,
                    cmd_vx,
                    1.0,
                );
            }
        }
    }
}

/// THE SPEED-LOOP MODEL WAS PROPORTIONAL, AND A REAL ONE IS NOT.
///
/// Every speed-mode figure in this file so far came from a driver model of
/// `tau = kv * (qd* - qd)`, with `kv = 1.2`. That leaves a standing error
/// proportional to load: reaching the 1.5 N*m limit needs 1.25 rad/s of
/// velocity error, so the commanded speed is never actually achieved. Trot's
/// 89-94% ceiling on that path was the model, not the interface.
///
/// An MG4005 closes a PI loop internally at 8-16 kHz. The integral term is
/// what removes the standing error and rejects load torque, and it is the
/// bigger of the two things the model was missing -- the rate matters less
/// than it sounds, since even the 2 kHz the simulator can offer is already
/// five times the 400 Hz host.
///
/// So sweep the integral gain. `loop_ki` is in N*m per (rad/s * s); with the
/// knee's 0.0034 kg*m^2 of reflected inertia, the proportional part alone
/// puts the loop corner near 56 Hz, so an integral corner in the tens to low
/// hundreds of rad/s is the region worth looking at.
#[test]
#[ignore = "sweep -- run with --ignored"]
fn namiashi_speed_loop_integral() {
    for loop_ki in [0.0, 20.0, 80.0, 300.0, 1000.0] {
        for i in 0..NAMIASHI_TUNED.len() {
            let (gait, .., cmd_vx) = NAMIASHI_TUNED[i];
            let params = WbcParams {
                actuation: Actuation::Velocity {
                    k_track: 100.0,
                    loop_kv: 1.2,
                    loop_ki,
                },
                host_rate_hz: Some(400.0),
                dt: 0.0005,
                total_time_s: 16.0,
                ..namiashi_tuned_params(i)
            };
            let Some(samples) = run_wbc_sim(params) else { return };
            report_walk(&format!("{gait:?} ki={loop_ki:.0}"), &samples, cmd_vx, 1.0);
        }
    }
}

/// SO WHAT IS ACTUALLY LIMITING SPEED MODE ON TROT?
///
/// The proportional-only speed loop was a real modelling defect, and adding
/// the integral a real driver has does not fix Trot: 89 / 89 / 85 / 81 / 83%
/// across `loop_ki` of 0 / 20 / 80 / 300 / 1000. It makes it worse, and the
/// saturation column says why -- the thigh goes 29 -> 40% clamped as the
/// integral is turned up. The loop is not short of authority. It is already
/// spending more than it has.
///
/// Which rules out the standing-error story and leaves the other half of the
/// interface. In speed mode the host does not command a position at all; it
/// commands `qd = dq*/dt + k_track*(q* - q)`, and everything the WBC computed
/// is discarded, because a speed-mode driver has nowhere to put a torque.
/// Trot is the gait with the shortest stance and the highest foot loads, so
/// it is the one that misses that most.
///
/// Two things are still confounded in every speed-mode figure so far, and
/// they pull opposite ways: `k_track` sets how hard the outer loop chases
/// position error, and `loop_kv` sets how hard the driver resists. Sweeping
/// them together at the 400 Hz the bus will actually run, with the integral
/// at a value a real driver would have, is what separates them.
#[test]
#[ignore = "sweep -- run with --ignored"]
fn namiashi_speed_mode_trot_limit() {
    for loop_kv in [1.2, 4.0, 12.0] {
        for k_track in [40.0, 100.0, 200.0] {
            let i = 0; // Trot -- the only gait that does not already work
            let (gait, .., cmd_vx) = NAMIASHI_TUNED[i];
            let params = WbcParams {
                actuation: Actuation::Velocity {
                    k_track,
                    loop_kv,
                    loop_ki: 80.0,
                },
                host_rate_hz: Some(400.0),
                dt: 0.0005,
                total_time_s: 16.0,
                ..namiashi_tuned_params(i)
            };
            let Some(samples) = run_wbc_sim(params) else { return };
            report_walk(
                &format!("{gait:?} kv={loop_kv:.1} kt={k_track:.0}"),
                &samples,
                cmd_vx,
                1.0,
            );
        }
    }
}

/// THE DRIVER MODELLED AS WHAT IT IS: A VELOCITY SOURCE.
///
/// Every speed-mode figure before this one came from a driver model that was
/// a P or PI controller evaluated at the *physics* rate. That imports a
/// slowness the real part does not have. An MG4005 closes its speed loop at
/// 8-16 kHz; against a host at 400 Hz that is twenty to forty times faster
/// than anything being asked of it, so from the controller's side it simply
/// delivers the commanded velocity.
///
/// `VelocityIdeal` models it that way -- deadbeat, `M_ii*(qd* - qd)/dt` plus
/// the joint's bias forces, clamped to `effort`. The clamp stays because
/// torque is what actually binds on this robot; the loop bandwidth was never
/// the constraint.
///
/// Swept against the PI model at both `k_track` values the earlier sweeps
/// disagreed about -- 40 suited Trot, 100 suited Walk and Crawl -- at the
/// 400 Hz the bus is designed for.
#[test]
#[ignore = "sweep -- run with --ignored"]
fn namiashi_ideal_velocity_source() {
    for k_track in [40.0, 100.0, 200.0] {
        for i in 0..NAMIASHI_TUNED.len() {
            let (gait, .., cmd_vx) = NAMIASHI_TUNED[i];
            for (tag, act) in [
                ("PI kv1.2", Actuation::Velocity {
                    k_track,
                    loop_kv: 1.2,
                    loop_ki: 80.0,
                }),
                ("ideal", Actuation::VelocityIdeal { k_track }),
            ] {
                let params = WbcParams {
                    actuation: act,
                    host_rate_hz: Some(400.0),
                    dt: 0.0005,
                    total_time_s: 16.0,
                    ..namiashi_tuned_params(i)
                };
                let Some(samples) = run_wbc_sim(params) else { return };
                report_walk(
                    &format!("{gait:?} {tag} kt={k_track:.0}"),
                    &samples,
                    cmd_vx,
                    1.0,
                );
            }
        }
    }
}

/// What a push did, measured against the same run's own pre-push behaviour.
///
/// Reported rather than asserted, because the interesting quantity is not
/// "did it survive" -- both interfaces do -- but how far it went and how long
/// it took to come back, which is what separates them.
fn report_push(label: &str, samples: &[WbcSample], t_push: f64, cmd_vx: f64) {
    let period = samples[0].cycle_period_s;
    let win = period * (0.8 / period).ceil();

    // Body-frame rates over a whole number of gait cycles, same convention as
    // everywhere else in this file.
    let rate_at = |i: usize| -> (f64, f64, f64) {
        let t0 = samples[i].t - win;
        let j = samples.iter().position(|s| s.t >= t0).unwrap_or(0);
        if j >= i {
            return (0.0, 0.0, 0.0);
        }
        let (a, b) = (&samples[j], &samples[i]);
        let dt = b.t - a.t;
        let (c, sn) = ((-a.yaw).cos(), (-a.yaw).sin());
        let (dx, dy) = (b.body_x - a.body_x, b.body_y - a.body_y);
        let mut dyaw = b.yaw - a.yaw;
        while dyaw > std::f64::consts::PI {
            dyaw -= 2.0 * std::f64::consts::PI;
        }
        while dyaw < -std::f64::consts::PI {
            dyaw += 2.0 * std::f64::consts::PI;
        }
        (
            (c * dx - sn * dy) / dt,
            (sn * dx + c * dy) / dt,
            dyaw.to_degrees() / dt,
        )
    };

    let idx_at = |t: f64| samples.iter().position(|s| s.t >= t).unwrap_or(0);
    let i_push = idx_at(t_push);
    let after: Vec<usize> = (i_push..samples.len()).collect();

    let mut vy_peak = 0.0_f64;
    let mut roll_peak = 0.0_f64;
    let mut z_min = f64::INFINITY;
    let mut t_recover = None;
    for &i in &after {
        let (_, vy, _) = rate_at(i);
        vy_peak = vy_peak.max(vy.abs());
        roll_peak = roll_peak.max(samples[i].roll.abs());
        z_min = z_min.min(samples[i].body_z);
        // Recovered: sideways rate back under 5 cm/s and staying there.
        if t_recover.is_none() && samples[i].t > t_push + 0.3 && vy.abs() < 0.05 {
            t_recover = Some(samples[i].t - t_push);
        }
    }
    // Lateral offset left behind, in the heading frame at the push.
    let a = &samples[i_push];
    let b = &samples[samples.len() - 1];
    let (c, sn) = ((-a.yaw).cos(), (-a.yaw).sin());
    let (dx, dy) = (b.body_x - a.body_x, b.body_y - a.body_y);
    let lat_left = sn * dx + c * dy;
    let fwd_after = (c * dx - sn * dy) / (b.t - a.t);

    eprintln!(
        "=== {label} (push at {t_push:.1}s) ===\n\
         peak |vy|={vy_peak:.3} m/s  recovered after {}  \
         lateral offset left={lat_left:+.3}m\n\
         peak roll={:.1}deg  min z={z_min:.3}m  \
         forward after push={fwd_after:+.3} m/s ({:.0}% of cmd)",
        t_recover.map_or("never".into(), |t| format!("{t:.2}s")),
        roll_peak.to_degrees(),
        if cmd_vx.abs() > 1e-9 { 100.0 * fwd_after / cmd_vx } else { 0.0 },
    );
}

/// SPEED MODE AGAINST TORQUE MODE, ACROSS EVERYTHING THE ROBOT IS ASKED TO DO.
///
/// Both at 400 Hz, both on the tuned configuration, differing only in the
/// interface: speed mode as an ideal torque-limited velocity source with
/// `k_track = 40`, torque mode as the host computing the whole loop.
///
/// Forward is the case every earlier result covered. The other five are the
/// ones that decide whether an interface is usable rather than merely
/// demonstrable -- a gait that only walks forward on flat ground with a
/// constant command is not a controller.
#[test]
#[ignore = "large sweep -- run with --ignored"]
fn namiashi_interface_full_comparison() {
    const HOST_HZ: f64 = 400.0;
    const SWEEP_DT: f64 = 0.0005;
    // 12 N for 0.12 s on a 3.3 kg robot is 1.44 N*s, a 0.44 m/s kick
    // sideways -- comparable to the fastest gait's own command, so it is a
    // real disturbance and not a nudge.
    const PUSH_T: f64 = 7.0;

    let modes: [(&str, Actuation); 2] = [
        ("speed", Actuation::VelocityIdeal { k_track: 40.0 }),
        ("torque", Actuation::Torque { kp: 100.0, kd: 1.2 }),
    ];

    for i in 0..NAMIASHI_TUNED.len() {
        let (gait, .., cmd) = NAMIASHI_TUNED[i];
        let lat = 0.5 * cmd;
        for (mtag, act) in modes {
            let base = |p: WbcParams| WbcParams {
                actuation: act,
                host_rate_hz: Some(HOST_HZ),
                dt: SWEEP_DT,
                ..p
            };

            // Steady commands: forward, backward, turn, strafe both ways.
            for (tag, vx, vy, wz) in [
                ("forward", cmd, 0.0, 0.0),
                ("backward", -cmd, 0.0, 0.0),
                ("turn", 0.0, 0.0, 0.60),
                ("strafe_L", 0.0, lat, 0.0),
                ("strafe_R", 0.0, -lat, 0.0),
            ] {
                let params = base(WbcParams {
                    cmd_vx: vx,
                    cmd_vy: vy,
                    cmd_wz: wz,
                    total_time_s: 14.0,
                    ..namiashi_tuned_params(i)
                });
                let Some(samples) = run_wbc_sim(params) else { return };
                report_walk_cmd(
                    &format!("{gait:?} {mtag} {tag}"),
                    &samples,
                    vx,
                    vy,
                    wz,
                    1.0,
                );
            }

            // Speed regulation: a command that moves, in steps.
            let params = base(WbcParams {
                cmd_vx: 0.0,
                cmd_schedule: vec![
                    (1.0, 0.35 * cmd, 0.0, 0.0),
                    (5.0, cmd, 0.0, 0.0),
                    (9.0, 0.6 * cmd, 0.0, 0.0),
                    (13.0, 0.0, 0.0, 0.0),
                ],
                total_time_s: 17.0,
                ..namiashi_tuned_params(i)
            });
            if let Some(samples) = run_wbc_sim(params) {
                // Reported per segment, since a whole-run average over a
                // moving command means nothing.
                for (t0, want) in
                    [(2.0, 0.35 * cmd), (6.0, cmd), (10.0, 0.6 * cmd), (14.0, 0.0)]
                {
                    let seg: Vec<WbcSample> = samples
                        .iter()
                        .filter(|s| s.t >= t0 && s.t < t0 + 3.0)
                        .map(|s| WbcSample { ..*s })
                        .collect();
                    if seg.len() > 10 {
                        report_walk(
                            &format!("{gait:?} {mtag} ramp@{want:.2}"),
                            &seg,
                            want,
                            0.0,
                        );
                    }
                }
            } else {
                return;
            }

            // Disturbance: walk forward, get pushed sideways.
            let params = base(WbcParams {
                cmd_vx: cmd,
                push: Some((PUSH_T, [0.0, 12.0, 0.0], 0.12)),
                total_time_s: 14.0,
                ..namiashi_tuned_params(i)
            });
            let Some(samples) = run_wbc_sim(params) else { return };
            report_push(&format!("{gait:?} {mtag} push"), &samples, PUSH_T, cmd);
        }
    }
}

/// VIDEO SOURCE: the two cases that separate the interfaces, on Trot.
///
/// Steady commands and speed regulation came out close between speed and
/// torque mode, so neither makes a useful film. These two do.
///
/// The push clips record a whole stride's worth of push phases, because
/// `namiashi_push_phase_dependence` showed a single push time says nothing:
/// over eight phases of Trot's 0.320 s cycle, speed mode falls at three of
/// them and torque mode at three, and they are not the same three. A video
/// built on one push time would be showing an accident of phase as though it
/// were a property of the interface.
///
/// The schedule clip steps through forward slow, forward fast, turn, strafe,
/// backward and stop. Steps rather than ramps, because a step is what exposes
/// handover behaviour.
#[test]
#[ignore = "video source -- needs NAMIASHI_REPLAY_OUT"]
fn namiashi_robustness_video_source() {
    let Ok(root) = std::env::var("NAMIASHI_REPLAY_OUT") else {
        eprintln!("NAMIASHI_REPLAY_OUT unset -- nothing to record");
        return;
    };
    const I: usize = 0; // Trot
    let (_, period, .., cmd) = NAMIASHI_TUNED[I];
    let modes: [(&str, Actuation); 2] = [
        ("speed", Actuation::VelocityIdeal { k_track: 40.0 }),
        ("torque", Actuation::Torque { kp: 100.0, kd: 1.2 }),
    ];

    for (mtag, act) in modes {
        let base = |p: WbcParams| WbcParams {
            actuation: act,
            host_rate_hz: Some(400.0),
            dt: 0.0005,
            ..p
        };

        for step in 0..8 {
            let t_push = 6.0 + period * step as f64 / 8.0;
            let params = base(WbcParams {
                cmd_vx: cmd,
                push: Some((t_push, [0.0, 12.0, 0.0], 0.12)),
                total_time_s: 10.0,
                replay_dir: Some(format!("{root}/push{step}_{mtag}")),
                ..namiashi_tuned_params(I)
            });
            let Some(samples) = run_wbc_sim(params) else { return };
            report_push(&format!("Trot {mtag} p{step}"), &samples, t_push, cmd);
        }

        let params = base(WbcParams {
            cmd_vx: 0.0,
            cmd_schedule: vec![
                (1.0, 0.35 * cmd, 0.0, 0.0),
                (4.0, cmd, 0.0, 0.0),
                (7.0, 0.0, 0.0, 0.60),
                (10.0, 0.0, 0.5 * cmd, 0.0),
                (13.0, -cmd, 0.0, 0.0),
                (16.0, 0.0, 0.0, 0.0),
            ],
            total_time_s: 19.0,
            replay_dir: Some(format!("{root}/schedule_{mtag}")),
            ..namiashi_tuned_params(I)
        });
        let Some(samples) = run_wbc_sim(params) else { return };
        report_walk(&format!("Trot {mtag} schedule"), &samples, cmd, 1.0);
    }
}

/// THE PUSH OUTCOME DEPENDS ON WHEN IN THE STRIDE IT LANDS.
///
/// `namiashi_interface_full_comparison` pushed at 7.0 s and found Trot
/// shrugging it off in both modes -- 6.3 and 12.2 degrees of roll, 95% of
/// command afterwards. Moving the same push to 6.0 s knocks both down: 17 and
/// 99 degrees of roll, trunk to 0.124 m, 10-13% of command afterwards.
///
/// Nothing about the interfaces changed. What changed is which feet were
/// loaded when the impulse arrived, and Trot spends most of its cycle on a
/// diagonal pair, so a sideways kick lands somewhere between "into the
/// support line" and "across it" depending on phase.
///
/// So a single push time says nothing about an interface, and the videos
/// should not be built on one. Sweep a whole stride -- eight phases over
/// Trot's 0.320 s -- in both modes, and see what the spread actually is.
#[test]
#[ignore = "sweep -- run with --ignored"]
fn namiashi_push_phase_dependence() {
    const I: usize = 0; // Trot
    let (_, period, .., cmd) = NAMIASHI_TUNED[I];
    for step in 0..8 {
        let t_push = 6.0 + period * step as f64 / 8.0;
        for (mtag, act) in [
            ("speed", Actuation::VelocityIdeal { k_track: 40.0 }),
            ("torque", Actuation::Torque { kp: 100.0, kd: 1.2 }),
        ] {
            let params = WbcParams {
                actuation: act,
                host_rate_hz: Some(400.0),
                dt: 0.0005,
                cmd_vx: cmd,
                push: Some((t_push, [0.0, 12.0, 0.0], 0.12)),
                total_time_s: 12.0,
                ..namiashi_tuned_params(I)
            };
            let Some(samples) = run_wbc_sim(params) else { return };
            report_push(
                &format!("Trot {mtag} phase{step}"),
                &samples,
                t_push,
                cmd,
            );
        }
    }
}

fn assert_forward_command_advances_body(samples: &[WbcSample]) {
    // Skip burn-in for tilt + displacement metrics.
    let dt: f64 = 0.002;
    let burn_in_steps = (0.5 / dt).round() as usize;
    let walk = &samples[burn_in_steps..];

    // No fall during the entire run (including burn-in).
    let min_z = samples.iter().map(|s| s.body_z).fold(f64::INFINITY, f64::min);
    assert!(
        min_z > TRUNK_Z_FALL_THRESHOLD_M,
        "forward walk: trunk fell, min_z = {min_z:.3} m",
    );

    // Forward displacement during the walking window.
    let x_start = walk.first().map(|s| s.body_x).unwrap_or(0.0);
    let x_end = walk.last().map(|s| s.body_x).unwrap_or(0.0);
    let dx = x_end - x_start;
    eprintln!(
        "[wbc] forward_command: Δx = {dx:.3} m over {:.1} s \
         (threshold ≥ {:.2})",
        walk.last().map(|s| s.t).unwrap_or(0.0)
            - walk.first().map(|s| s.t).unwrap_or(0.0),
        MIN_DISPLACEMENT_M,
    );
    assert!(
        dx >= MIN_DISPLACEMENT_M,
        "forward walk: Δx = {dx:.3} m < {} m — WBC produced near-zero \
         net forward motion (足踏み regression?)",
        MIN_DISPLACEMENT_M,
    );

    // Bounded body tilt during walking. WBC + Position-PD with
    // gravity-comp should keep the trunk roughly level on flat ground.
    let peak_roll = walk
        .iter()
        .map(|s| s.roll.abs())
        .fold(0.0_f64, f64::max);
    let peak_pitch = walk
        .iter()
        .map(|s| s.pitch.abs())
        .fold(0.0_f64, f64::max);
    eprintln!(
        "[wbc] forward_command: peak |roll| = {peak_roll:.2} rad, \
         peak |pitch| = {peak_pitch:.2} rad",
    );
    // 0.5 rad ≈ 30° — generous because WBC q[3..7] = identity means
    // tilt feedback is approximate.
    assert!(
        peak_roll < 0.5,
        "forward walk: |roll| peak {peak_roll:.2} rad too large",
    );
    assert!(
        peak_pitch < 0.5,
        "forward walk: |pitch| peak {peak_pitch:.2} rad too large",
    );
}
