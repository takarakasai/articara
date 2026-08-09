//! Shared WBC/MPC MuJoCo simulation harness for namiashi.
//!
//! Promoted out of `tests/wbc_walk.rs` so a proper binary (whose `fn
//! main()` runs on the real process main thread, required by
//! `mujoco-rs`'s `winit`-based viewer -- `cargo test`'s harness always
//! runs test bodies on a worker thread, which `MjViewer::launch_passive`
//! cannot tolerate) can call the exact same, already-validated
//! `run_wbc_sim` pipeline that every WBC/MPC measurement in
//! `tests/wbc_walk.rs` is built on, instead of a second, drifting
//! reimplementation. `tests/wbc_walk.rs` re-imports everything here via
//! `use articara::wbc_harness::*;` -- nothing about its ~85 tests'
//! behavior changes, only where the code they call lives.

use std::path::PathBuf;

use crate::gait::{
    auto_detect_centroidal_mpc_config, auto_detect_kinematics_config,
    auto_detect_srbd_mpc_config, GaitController, DEFAULT_FOOT_LINKS,
};
use crate::mjcf::{GroundPlaneCfg, MjcfExportOptions, StaircaseCfg};
use crate::mujoco_sim::MujocoSim;
use crate::rbd::model::ActuatorMode;
use crate::robot::RobotModel;
use crate::wbc_pipeline::WbcPipeline;
use nalgebra::Vector3;
use quadruped_gait::wbc;
use quadruped_gait::{
    solve_leg_ik, ContactDrivenPhase, GaitConfig, GaitMode, GaitType,
    KinematicsConfig, LegIkSolution, VelocityCmd,
};

pub fn namiashi_misa() -> PathBuf {
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
pub const DEFAULT_MISA: &str = "namiashi_3p3_prop.misa";

/// The shipped `namiashi.misa` came from a CAD export totalling 2.400 kg with
/// 36% of that in the legs. The built robot is 3.3 kg with 600 g per leg --
/// 73% in the legs. `rescale_mass.py` in the same directory regenerates the
/// corrected models; `_hip` puts the extra mass at the hip where the motors
/// are, `_prop` spreads it along the leg as the CAD distribution implies.
pub fn namiashi_misa_named(file: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("tests")
        .join("fixtures")
        .join("namiashi")
        .join(file)
}

/// Same seeding logic as `gait_walk_stability`. Keeps the legs out of
/// their q=0 fully-extended kinematic singularity at sim start.
pub fn seed_joint_positions_from_kinematics(
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
pub struct WbcSample {
    pub t: f64,
    pub body_x: f64,
    pub body_z: f64,
    pub roll: f64,
    pub pitch: f64,
    pub total_fz_world: f64,
    /// Per-foot normal force (N), FL/FR/RL/RR. Contact state is what
    /// distinguishes a gait that is walking from one that is shuffling, and
    /// the original four tests could not see the difference -- they checked
    /// trunk height and net displacement only.
    pub foot_fz: [f64; 4],
    /// Per-foot fore-aft contact force (N), body-frame x. Where propulsion
    /// actually comes from: with a position servo on the stance legs the body
    /// is dragged forward kinematically by the foot sweep, but with them at
    /// kp=0 the only thing left is this.
    pub foot_fx: [f64; 4],
    /// Fore-aft force the MPC planned and the WBC's QP settled on, heading
    /// frame. The ratio of the two is how much of the plan is getting
    /// through, which is the question "is the MPC load-bearing" made
    /// measurable on every run instead of by hand.
    pub mpc_fx: f64,
    /// Vertical component of the same plan. A controller supporting a 3.3 kg
    /// robot must plan about +32 N; anything near -32 is a sign convention,
    /// not a control error, and the two are worth telling apart before
    /// chasing the latter.
    pub mpc_fz: f64,
    pub wbc_fx: f64,
    pub body_y: f64,
    pub yaw: f64,
    /// Carried per-sample only so `report_walk_cmd` can size its averaging
    /// window in whole gait cycles without every call site threading it.
    pub cycle_period_s: f64,
    /// |applied torque| / that joint's `effort` limit, per joint. A joint at
    /// 1.0 is clamped: the commanded torque is not the torque being produced,
    /// and the modelled controller is not the controller running.
    pub tau_frac: [f64; 12],
    /// |applied torque| in N*m. Kept alongside `tau_frac` because the peak
    /// and the continuous rating are different numbers and only the peak is
    /// in the model.
    pub tau_nm: [f64; 12],
    /// |joint velocity| / that joint's rated `velocity`. The knee's rating
    /// drops to 21.5 rad/s once its 9:14 reduction is referred properly, and
    /// `mujoco_sim` brakes overspeed with a torque added *before* the effort
    /// clamp -- so a joint that is too fast shows up as a joint that is out
    /// of torque, which is a different problem with a different fix.
    pub qd_frac: [f64; 12],
    /// Which legs the gait schedule had in stance this tick. Saturation means
    /// two different things depending on it -- a clamped stance joint is a
    /// robot that cannot hold itself up, a clamped swing joint is a leg that
    /// cannot be thrown fast enough -- and the fixes are unrelated.
    pub stance_mask: [bool; 4],
}

/// Threshold for "trunk has fallen". Below this z, the body has either
/// tipped over or sunk through the ground. Same value as
/// `gait_walk_stability`.
pub const TRUNK_Z_FALL_THRESHOLD_M: f64 = 0.18;

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
pub enum Actuation {
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
    /// `legged_control`'s actual low-level command, read off
    /// `legged_controller.cpp:142`:
    ///
    /// ```text
    /// setCommand(pos_des, vel_des, 5, 3, torque)
    ///     -> tau = kp*(pos_des - q) + kd*(vel_des - qd) + torque
    /// ```
    ///
    /// Three things this file has been getting wrong at once. The gain is 5,
    /// not 100 -- a twentieth. It is uniform, with no stance/swing split. And
    /// the damping term tracks a *velocity target* rather than damping toward
    /// zero, so it does not fight the swing it is supposed to be producing.
    ///
    /// `pos_des` there comes from the MPC's optimised state rather than an IK
    /// trajectory, which this does not reproduce -- the trajectory is still
    /// the gait controller's. So this is half the difference, and worth
    /// measuring before deciding whether the other half is needed.
    LeggedControl { kp: f64, kd: f64 },

    /// Stance legs pure torque, swing legs position-tracked.
    ///
    /// That stack sends Unitree's `(q, dq, kp, kd, tau)` with `kp = kd = 0`
    /// on stance legs, so the WBC's contact force *is* the joint torque and
    /// nothing else acts on them; swing legs get position gains because they
    /// are tracking a trajectory, not producing a force.
    ///
    /// Every result in this file has instead run kp=100 on all twelve joints
    /// with the WBC's torque added as feedforward. Under a stiff position
    /// servo tracking an IK trajectory, a force allocation is a small
    /// perturbation -- which is the obvious candidate for why four separate
    /// measurements have found the MPC not to matter here while it plainly
    /// does in legged_control.
    TorqueLeggedControl {
        swing_kp: f64,
        swing_kd: f64,
        /// Position gain on stance legs. `legged_control` uses 0 -- the
        /// WBC's contact force is the whole command. 100 is what this file
        /// has run throughout, and at that value the servo supplies the
        /// propulsion and the WBC only trims it. The interesting question is
        /// whether anything in between hands over cleanly.
        stance_kp: f64,
        stance_kd: f64,
        /// Scale on an explicit `h(q, q̇)` bias feedforward -- gravity plus
        /// Coriolis, not gravity alone. 0 leaves it to the WBC, whose `tau`
        /// solves the full equation of motion and should already carry it;
        /// 1 adds the whole thing. Measured rather than reasoned about,
        /// because at a static stand `tau_wbc` on a stance leg is
        /// (-0.369, +0.008, +0.559) against a leg-gravity of (+0.074, +0.048,
        /// -0.033) -- the support load dwarfs the bias, so a duplicate is a
        /// 10-20% error rather than a doubling, and the sign of its effect is
        /// not obvious.
        bias_ff: f64,
    },
}

/// What the controller is told about its own velocity.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum VelObs {
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

/// Capture-point footstep gain, seconds.
///
/// This was zero for most of this file's history, and that was right at the
/// time: at the library's 0.05 the 0.295 m stance overshot by 20%, drifted
/// sideways at 0.145 m/s and yawed at 2.34 deg/s, and zero fixed all three.
///
/// Every condition that was measured under has since changed -- the stance
/// dropped 6 cm, the actuation moved to a speed- or torque-mode driver, the
/// host went to 400 Hz -- and, more to the point, it was measured with
/// nothing disturbing the robot. Footstep feedback has nothing to do on flat
/// ground at a constant command; it can only add noise there. That is the one
/// condition under which its value cannot be judged.
///
/// With a push test to judge it by, a small gain is better on both counts
/// (Trot, speed mode, 400 Hz):
///
///     k        nominal   yaw drift      push survival
///     0.000       90%    +5.28 deg/s      5 of 8
///     0.015       97%    +0.85            6 of 8
///     0.030      102%    +1.31            6 of 8
///     0.050      106%    +0.11            6 of 8
///
/// 0.015 rather than 0.030 or 0.050 because the recovery is the same at all
/// three and the walking is not: on the regression's own path over 25 s,
/// 0.030 overshoots Trot to 115% where 0.015 sits at 107%.
///
/// Still far below the library's 0.05 and further below the LIP value
/// sqrt(h/g) = 0.155 for this stance. See `namiashi_capture_gain_low_side`
/// for why that formula does not apply here.
pub const NAMIASHI_CAPTURE_GAIN_S: f64 = 0.015;

/// Continuous torque rating, N*m, hip and thigh. The knee's 9:14 reduction
/// scales it the same way its peak is scaled.
///
/// `effort` in the model is the *peak* (2.5 N*m, 3.889 at the knee) and
/// MuJoCo clamps to it, so exceeding the continuous rating is invisible
/// there: a gait that sits at 2 N*m for a whole run never clamps and still
/// cooks the motor. Reported separately for that reason.
pub const NAMIASHI_RATED_TORQUE_NM: f64 = 1.0;

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
pub const NAMIASHI_STANCE_DROP_M: f64 = 0.06;

/// Minimum forward displacement during the walking window — the same
/// 4 cm threshold the gait stability test uses.
pub const MIN_DISPLACEMENT_M: f64 = 0.04;

/// How much of the tail of a static-stand run the gravity-balance average
/// covers.
pub const STATIC_AVG_WINDOW_S: f64 = 0.5;

/// Proprioceptive mid-swing collision recovery.
///
/// `namiashi_staircase_5cm_swing_clearance_sweep` ruled out swing-height
/// alone: raising it up to 3x flat-ground clearance still tips the robot
/// over on the first riser. But the diagnostic stream already carries a
/// clean signal for the moment of the collision itself -- swing FK error
/// (measured foot position vs. the open-loop swing target, in body frame)
/// spikes to ~0.09 m the instant a foot catches the riser, against a
/// flat-ground ceiling of ~0.05 m -- roughly 1.4 s before the yaw-rate
/// runaway that actually causes the fall. That is real lead time, and the
/// signal needs no new sensor: joint encoders plus the swing target the
/// gait controller already computes are enough.
///
/// This is the reflex built on that signal: when a swinging leg's FK error
/// crosses `trigger_m`, stop tracking its open-loop target and instead drive
/// straight up by `lift_m` from wherever the foot actually is -- "back off
/// and go over it" -- until the error falls back under `resume_m` (hysteresis
/// so it does not chatter at the threshold), at which point it resumes
/// tracking the normal swing trajectory.
#[derive(Clone, Copy, Debug)]
pub struct ContactReflexCfg {
    pub trigger_m: f64,
    pub resume_m: f64,
    pub lift_m: f64,
    /// While any leg is `reflex_active`, pass `dt=0` to `gc.tick()` instead of
    /// the real host period, freezing every leg's phase together rather than
    /// just overriding the disrupted leg's target.
    ///
    /// Without this, the other three legs' schedule keeps advancing on the
    /// gait's single shared phase clock while the disrupted leg is still
    /// fighting the obstacle -- so by the time it resolves, the rest of the
    /// gait already expects it to have been in stance for however long the
    /// reflex took, i.e. the robot is trying to balance on however many legs
    /// actually landed on schedule, not however many the gait *thinks* are
    /// down. That desync, not the collision itself, is the leading
    /// suspect for why the first two reflex attempts (lift from current
    /// position; lift the nominal target) both still ended in a collapse.
    pub freeze_phase_during_reflex: bool,
}

/// Perception-informed swing height (and, with `horizontal_margin_m > 0`,
/// touchdown placement): the "would knowing the terrain help at all"
/// question, tested in the cheapest honest way before building a real
/// sensor or a terrain-aware MPC cost term.
///
/// Every reflex variant in `ContactReflexCfg` reacts *after* a foot has
/// already caught the riser -- proprioception has no way to see it coming.
/// This instead queries `StaircaseCfg::height_at`/`snap_to_tread` at the
/// swing target's current world x every tick: raises the *z* component to
/// `terrain_height + clearance_m` (continuous ground-contouring rather than
/// a threshold-triggered correction), and, separately, nudges the *x*
/// component off a riser edge and onto solid tread by `horizontal_margin_m`.
///
/// `clearance_m` alone (`horizontal_margin_m = 0`) is what
/// `namiashi_staircase_5cm_terrain_footplan` measured: gets onto and
/// balances on the first tread for ~7 s, but stalls there -- touchdown x
/// oscillates right at the tread-1/riser boundary (1.44-1.56 m against a
/// riser at 1.5 m) rather than committing onto solid ground, which is
/// exactly what `horizontal_margin_m` targets.
#[derive(Clone, Copy, Debug)]
pub struct TerrainFootplanCfg {
    pub clearance_m: f64,
    pub horizontal_margin_m: f64,
}

/// See `WbcParams::hip_bias_gate`'s doc comment.
#[derive(Clone, Copy, Debug)]
pub struct HipBiasGateCfg {
    /// Swing-leg FK tracking error, metres, above which the gate opens.
    pub trigger_m: f64,
    /// `hip_lr_bias_rad` magnitude applied while the gate is open, at the
    /// moment the gate opens (`err == trigger_m`).
    pub bias_mag: f64,
    /// Extra bias per metre of FK error beyond `trigger_m`, added to
    /// `bias_mag` at trigger time -- e.g. a bigger riser collision gets a
    /// bigger correction, rather than every trigger getting the same
    /// constant nudge regardless of how far off the nominal foothold the
    /// leg actually was. 0.0 recovers the original constant-magnitude
    /// gate. The magnitude is fixed for the whole `duration_s` window
    /// once computed at trigger time, not recomputed every tick, so a
    /// later smaller error mid-window can't shrink an already-open gate.
    pub bias_gain: f64,
    /// Hard clamp on the trigger-time magnitude (`bias_mag +
    /// bias_gain * (err - trigger_m)`), radians -- a safety bound so the
    /// gain term can't run away on a large collision.
    pub max_bias_rad: f64,
    /// How long the gate stays open after the last trigger, seconds.
    pub duration_s: f64,
}

pub struct WbcParams {
    pub total_time_s: f64,
    pub burn_in_s: f64,
    pub cmd_vx: f64,
    pub cmd_vy: f64,
    pub cmd_wz: f64,
    /// Model file under `tests/fixtures/namiashi/`.
    pub misa_file: &'static str,
    /// Early-touchdown force at which `ContactDrivenPhase` overrides the
    /// nominal phase, newtons. The harness has always used 5.0. Heavier legs
    /// hit harder, so on the 3.3 kg models this threshold fires on contacts
    /// the 2.4 kg model never produced.
    pub early_contact_n: f64,
    /// Diagnostic: drop the WBC's torque and the MPC's GRF reference, leaving
    /// only the joint position-PD tracking the IK targets. If the measured
    /// gait does not change, the dynamic layers are not contributing.
    pub kinematic_only: bool,
    /// Give the WBC the robot it is actually driving. `WbcPipeline::new`
    /// hardcodes `mass_kg: 9.0` and a 9 kg inertia diagonal; namiashi is
    /// 3.3 kg. Off by default so the effect can be measured against the
    /// state every earlier result in this file was produced under.
    pub wbc_real_inertia: bool,
    /// Decomposition of `wbc_real_inertia`, which bundles three changes:
    /// the mass (9.0 -> 3.3, a 2.7x amplification of `a_base_des`), the CoM
    /// offset, and the composite inertia (which also switches the pipeline
    /// onto the CoM-moment-arm accel prediction). Each can be applied alone.
    pub wbc_real_mass_only: bool,
    pub wbc_real_com_only: bool,
    pub wbc_real_inertia_only: bool,
    /// Minimum normal force on commanded-stance feet, as a fraction of the
    /// static per-foot share `m*g/4`. Expressed as a fraction rather than
    /// newtons so it means the same thing on the 2.4 and 3.3 kg models.
    pub f_min_stance_frac: f64,
    /// Override for `WbcWeights::base_accel` (default 200.0). This is the
    /// largest soft weight in the QP and the one that carries the MPC's
    /// predicted base acceleration into the torque.
    pub base_accel_weight: Option<f64>,
    /// Override for `WbcWeights::contact_force` (default 5.0) -- how hard the
    /// WBC pulls its own GRF solution toward the MPC's prediction.
    pub contact_force_weight: Option<f64>,
    /// `WbcPipeline::yaw_pd_gain`: an explicit `(kp, kd)` heading correction,
    /// additive on top of whatever the GRF-derived feedforward already does
    /// -- `(0.0, 0.0)` (the default everywhere else in this file) is an
    /// exact no-op. Nothing commands a turn on the staircase (cmd_wz stays
    /// 0 throughout), yet yaw drifts by a full 180 deg over ~5-8 s while
    /// the robot is balanced on a 3 cm-riser tread -- this is the direct
    /// lever to test whether that drift is simply unregulated rather than
    /// something a footplan or WBC weight change would ever reach.
    pub yaw_pd_gain: Option<(f64, f64)>,
    /// Constant hip-roll offset (IK convention, radians), added to the
    /// LEFT legs (FL, RL) and SUBTRACTED from the RIGHT legs (FR, RR),
    /// every tick, stance and swing alike -- a persistent left-right
    /// stance-width asymmetry, not a footstep-plan or swing-only
    /// correction. Taken directly from a namiashi RL policy's own
    /// recorded joint trace (go2_rl/namiashi_rl), which does NOT show
    /// this on flat ground (L-R hip asymmetry ~0.01 rad, noise level) but
    /// holds a consistent +0.07 to +0.16 rad asymmetry throughout all ten
    /// riser crossings of a successful 5cm climb -- this field tests
    /// whether that alone, injected into the existing WBC/IK output
    /// rather than learned, buys any of the same yaw/roll stability.
    /// `None` (everywhere else in this file) is a no-op.
    pub hip_lr_bias_rad: Option<f64>,
    /// A GATED version of `hip_lr_bias_rad`: instead of applying constantly,
    /// only activate (for `duration_s` seconds) once any swing leg's FK
    /// tracking error exceeds `trigger_m` -- the same collision signal
    /// `ContactReflexCfg` already uses, computed independently here so
    /// this field works without `contact_reflex` also being set. The
    /// point: a constant bias (`hip_lr_bias_rad` alone) did not help
    /// (`namiashi_staircase_5cm_rl_inspired_hip_bias`); this tests whether
    /// the RL policy's benefit instead came from applying that same
    /// correction only IN RESPONSE to a detected riser disturbance --
    /// i.e. an explicit, hand-written state-based rule standing in for
    /// whatever implicit state-conditioned behavior the RL policy learned.
    /// The three fields are exactly what a small gradient-free search
    /// (see `namiashi_staircase_5cm_hip_gate_search`) optimizes.
    pub hip_bias_gate: Option<HipBiasGateCfg>,
    /// Write the exact MJCF the sim runs plus a per-tick root-pose and joint
    /// trace here, for `render_namiashi.py` to replay. Purely a side channel;
    /// nothing in the harness reads it back.
    pub replay_dir: Option<String>,
    /// Lower the nominal stance by this much, metres. Applied to every leg's
    /// `nominal_foot_body.z`, which is what both the IK seed and the MPC's
    /// body-z reference are taken from, so the whole stack follows.
    /// Defaults to [`NAMIASHI_STANCE_DROP_M`].
    pub trunk_drop_m: f64,
    /// Constant offset added to the base position the WBC observes, and a
    /// per-second drift on top of it. Both are diagnostics for the hardware
    /// question: is a bounded absolute position actually required?
    pub base_pos_bias_m: [f64; 3],
    pub base_pos_drift_mps: [f64; 3],
    /// How the observed body velocity is corrupted before the controller sees
    /// it. The hardware question: with only an IMU there is no bounded
    /// velocity estimate to be had -- 1 deg of attitude error leaks
    /// g*sin(1 deg) = 0.17 m/s^2 of phantom horizontal acceleration, which is
    /// 0.85 m/s of velocity error in five seconds against a 0.80 m/s command.
    /// So: how much does this controller actually depend on it?
    pub vel_obs: VelObs,
    /// Which gait controller to run. `Mpc` is the body-root SRBD path this
    /// file has always used; `FullCentroidal` is the one that carries the
    /// OCS2-derived predicted-footstep planner.
    pub gait_mode: GaitMode,
    /// Enable `set_use_mpc_predicted_footstep` -- the legged_control /
    /// OCS2 `SwingTrajectoryPlanner` analogue, which takes the foothold
    /// correction from the MPC's own predicted base displacement over one
    /// swing instead of extrapolating `k_capture * v_err`. Only has an
    /// effect under `GaitMode::FullCentroidal`.
    pub mpc_predicted_footstep: bool,
    /// FullCentroidal's `legged_control_parity`: builds the per-step contact
    /// schedule from a per-leg phase projection and enables the OCS2-shaped
    /// swing reference. Prerequisite for `dynamic_joint_q_reference`.
    pub legged_control_parity: bool,
    /// Sample the swing/stance foot curve at each horizon step's projected
    /// phase for the MPC's joint_q reference, instead of holding it flat.
    ///
    /// Note this changes what the MPC *plans against*, not what the joints
    /// are commanded to. `FullCentroidalMpcGaitController`'s own docs are
    /// explicit that the output joint angles remain the analytical IK's --
    /// the MPC's joint_q exists so the per-node moment arm
    /// `r = R (foot_body(q) - com_offset)` updates within the horizon. There
    /// is no equivalent of legged_control's
    /// `pos_des = getJointAngles(optimized_state)` in this implementation.
    pub dynamic_joint_q_reference: bool,
    /// Input cost on the 24-state MPC's GRF entries, `r_diag[0..12]`.
    ///
    /// The default is `[1e-3; 24]` -- one scalar across both halves of an
    /// input vector whose first twelve entries are forces in newtons and
    /// whose last twelve are joint velocities in rad/s. The field's own doc
    /// says those "two distinct scales coexist ... unlike the 12-state
    /// version we cannot share a scalar", and then the default shares one.
    /// `q_diag` is `[1.0; 24]` over a state mixing radians, metres, m/s and
    /// joint angles, with the same problem.
    ///
    /// `None` leaves the default. The 12-state SRBD does not suffer this
    /// because its twelve inputs are all forces.
    /// SRBD horizon length in steps. The default 10 at `dt_per_step = 0.030`
    /// is 0.300 s -- shorter than Trot's 0.320 s cycle, so the MPC cannot see
    /// one complete contact sequence. `legged_control` uses 1.0 s over 67
    /// nodes, roughly two cycles of its own gait.
    pub mpc_horizon_steps: Option<usize>,
    /// Fore-aft position cost, `q_diag[p_x]`. Zero in all three MPCs here;
    /// `task.info:159` gives it 1000. Without it the MPC has no objective that
    /// grows when the body falls behind, which is why it plans +0.22 N.
    pub mpc_px_cost: Option<f64>,
    /// `set_task_space_joint_vel_weight` -- `legged_control`'s
    /// `initializeInputCostWeight` maps a task-space foot-velocity weight
    /// through the leg Jacobian into joint space. Implemented here and never
    /// called, so the joint-velocity cost is isotropic in rad/s instead.
    pub fcm_taskspace_jv_weight: Option<[f64; 3]>,
    /// Warm start with one SQP iteration, as the reference does, instead of
    /// three cold ones.
    pub fcm_warm_start: bool,
    /// FullCentroidal horizon, in steps and seconds-per-step. The default is
    /// 10 x 0.030 = 0.300 s against legged_control's 1.0 s, and
    /// `set_srbd_mpc_config` cannot reach it -- that setter is a silent no-op
    /// outside `GaitMode::Mpc`, so `mpc_horizon_steps` does nothing here.
    pub fcm_horizon_steps: Option<usize>,
    pub fcm_dt_per_step: Option<f64>,
    /// SQP iterations, separately from `fcm_warm_start`. The warm-start path
    /// also drops iterations from 3 to 1, and those are two different changes.
    pub fcm_sqp_iterations: Option<usize>,
    /// Solve the FullCentroidal QP in sparse multiple-shooting form. See
    /// `FullCentroidalMpcConfig::sparse_qp` -- the condensed form forms
    /// `A_d^k` explicitly and dies at any horizon worth having.
    pub fcm_sparse_qp: bool,
    pub fcm_grf_cost: Option<f64>,
    /// `r_diag[12..24]` -- the joint-velocity cost. `fcm_grf_cost` only sets
    /// the GRF half, and the default leaves this at 1.0 against the GRF's
    /// 1e-3, so ground force is a thousand times cheaper than leg motion.
    pub fcm_jointv_cost: Option<f64>,
    /// Ablation of the `base_accel` task's feedforward. When set, the GRF fed
    /// into `WbcPipeline::solve` (both the linear/angular Newton's-law
    /// feedforward and the priority-2 contact_force reference) is replaced by
    /// a quasi-static gravity split across whichever feet are in stance --
    /// no momentum plan, no footstep-driven moment -- and the pipeline's
    /// `roll_pd_gain`/`pitch_pd_gain` are set to this `(kp, kd)`, so attitude
    /// correction comes entirely from an explicit PD term rather than from
    /// whatever GaitMode is doing upstream. Isolates what the MPC's own
    /// predicted trajectory is contributing through the channel
    /// (`base_accel`, priority 1) that already has real authority, as
    /// opposed to a bare regulator.
    pub attitude_pd_ablation: Option<(f64, f64)>,
    /// Replace the flat ground plane with a staircase. `None` (default)
    /// behaves exactly as before -- the infinite `GroundPlaneCfg` plane.
    pub staircase: Option<StaircaseCfg>,
    /// Proprioceptive "hit something mid-swing" recovery. `None` (default)
    /// leaves every swing leg tracking its open-loop target unmodified, no
    /// matter how far off it drifts -- the behaviour every other test in
    /// this file assumes.
    pub contact_reflex: Option<ContactReflexCfg>,
    /// Perception-informed swing height, using `StaircaseCfg::height_at` as
    /// an idealized stand-in for a real height-map. `None` (default) is the
    /// terrain-blind behaviour every other test in this file assumes.
    pub terrain_footplan: Option<TerrainFootplanCfg>,
    /// Per-block state cost for the 24-state MPC: `[v_com, omega, base_pos,
    /// euler, joint_q]`, applied over `q_diag`'s
    /// `[0..3, 3..6, 6..9, 9..12, 12..24]`.
    ///
    /// The default is `[1.0; 24]` over five different physical quantities --
    /// m/s, rad/s, m, rad, rad. The base position entry is the one to look at
    /// first: it grows without bound as the robot walks, so a weight on it is
    /// a weight on an error that can only increase.
    pub fcm_state_cost: Option<[f64; 5]>,
    /// Add the missing `- omega_b x v_b` term to the WBC's base-accel
    /// reference. See `WbcPipeline::base_accel_coriolis`.
    pub base_accel_coriolis: bool,
    /// Set every `WbcWeights` entry to 1.0, as `legged_control` effectively
    /// has them -- it concatenates tasks within a priority level unweighted.
    pub flat_wbc_weights: bool,
    /// Drop the `tau_gravity` task, which `legged_control` does not have.
    pub drop_tau_gravity: bool,
    /// Give the SRBD MPC namiashi's composite inertia instead of the heaviest
    /// link's.
    ///
    /// `auto_detect_srbd_mpc_config` takes the heaviest link's own tensor,
    /// which on this model is the 0.872 kg trunk: (0.00111, 0.00504, 0.00529)
    /// against a composite of (0.02722, 0.07575, 0.06584), so 12 to 24 times
    /// too small. A pitch inertia that small tells the MPC a tiny moment buys
    /// a large angular acceleration.
    ///
    /// Fixing it inside `auto_detect` was tried and reverted: it costs Go2
    /// 0.485 -> 0.194 m/s at a 1.00 m/s command, and Go2's tuning was built
    /// against the old value. The function's doc says to set it per robot
    /// instead, which is what this does.
    pub mpc_composite_inertia: bool,
    /// Which actuator interface the controller drives.
    pub actuation: Actuation,
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
    pub host_rate_hz: Option<f64>,
    /// Piecewise-constant command schedule: `(t_from, vx, vy, wz)`. Overrides
    /// `cmd_vx/vy/wz` once the first entry's time is reached. For asking how
    /// the two interfaces handle a command that moves, rather than one held
    /// constant for the whole run.
    pub cmd_schedule: Vec<(f64, f64, f64, f64)>,
    /// Live teleop state (velocity command + requested gait), updated
    /// asynchronously by the viewer's keyboard callback and read every
    /// physics tick. The command takes priority over `cmd_vx/vy/wz` and
    /// `cmd_schedule` once burn-in ends; a changed gait is applied by
    /// swapping the whole `GaitConfig` (see `namiashi_tuned_gait_config`).
    /// `None` (the default) leaves every other call site's fixed- or
    /// scheduled-command behavior completely unaffected.
    #[cfg(feature = "mujoco-viewer")]
    pub live_teleop: Option<std::sync::Arc<std::sync::Mutex<crate::teleop::LiveTeleop>>>,
    /// Open a real-time `mujoco::viewer::MjViewer` (feature `mujoco-viewer`)
    /// on the sim instead of running headless-and-tracing. Runs until the
    /// viewer window is closed rather than for a fixed `total_time_s`.
    pub live_viewer: bool,
    /// World-frame push on the trunk: `(t_start, [fx, fy, fz], duration)`.
    /// A disturbance neither interface plans for, so it is the one test that
    /// asks what happens when the model is wrong.
    pub push: Option<(f64, [f64; 3], f64)>,
    pub dt: f64,
    /// `None` = the legacy `wbc::solve_warm_with_weights` path
    /// (walk-validated default). `Some` opts the pipeline into the
    /// misa-wbc `Dynamics`-backed [`WbcSolver`] with the given
    /// formulation/strategy/backend — the equivalence-study switch
    /// documented in `ref/wbc_comparison.md`.
    pub misa_wbc_mode: Option<(wbc::Formulation, wbc::SolveConfig)>,
    /// Gait family. `None` keeps `GaitConfig::trot()`, which is what the
    /// original four tests ran and the only thing this file had ever
    /// exercised.
    pub gait_type: Option<GaitType>,
    /// Overrides applied after the family's own defaults. namiashi is 2.400 kg
    /// against Go2's 15.606 with a 0.306 m leg against 0.426, so the numbers
    /// tuned on Go2 do not carry over directly -- under Froude similarity
    /// (T proportional to sqrt(L/g), stride to L) Go2's 0.18 s / 0.20 m
    /// become 0.152 s / 0.143 m here. `None` keeps the family default.
    pub cycle_period_s: Option<f64>,
    pub duty_factor: Option<f64>,
    pub max_step_length_m: Option<f64>,
    /// Swing-foot apex clearance. Crawl's library default is 0.005 m, which
    /// is fine over a 0.06 m step and almost certainly scuffing over a
    /// 0.145 m one.
    pub swing_height_m: Option<f64>,
    /// Capture-point feedback gain, seconds. The library default is 0.05 s
    /// for every robot; the LIP value it is meant to approximate is
    /// sqrt(h/g), which for namiashi's 0.30 m trunk is 0.175 s.
    pub k_capture_s: Option<f64>,
}

impl WbcParams {
    pub fn static_stand() -> Self {
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
            yaw_pd_gain: None,
            hip_lr_bias_rad: None,
            hip_bias_gate: None,
            replay_dir: None,
            trunk_drop_m: NAMIASHI_STANCE_DROP_M,
            base_pos_bias_m: [0.0; 3],
            base_pos_drift_mps: [0.0; 3],
            vel_obs: VelObs::Truth,
            gait_mode: GaitMode::Mpc,
            mpc_predicted_footstep: false,
            legged_control_parity: false,
            dynamic_joint_q_reference: false,
            mpc_horizon_steps: None,
            mpc_px_cost: None,
            fcm_taskspace_jv_weight: None,
            fcm_warm_start: false,
            fcm_horizon_steps: None,
            fcm_dt_per_step: None,
            fcm_sqp_iterations: None,
            fcm_sparse_qp: false,
            fcm_grf_cost: None,
            fcm_jointv_cost: None,
            attitude_pd_ablation: None,
            staircase: None,
            contact_reflex: None,
            terrain_footplan: None,
            fcm_state_cost: None,
            base_accel_coriolis: false,
            flat_wbc_weights: false,
            drop_tau_gravity: false,
            mpc_composite_inertia: false,
            actuation: Actuation::PositionTorque,
            host_rate_hz: None,
            cmd_schedule: Vec::new(),
            #[cfg(feature = "mujoco-viewer")]
            live_teleop: None,
            live_viewer: false,
            push: None,
        }
    }
    pub fn forward_walk() -> Self {
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
            yaw_pd_gain: None,
            hip_lr_bias_rad: None,
            hip_bias_gate: None,
            replay_dir: None,
            trunk_drop_m: NAMIASHI_STANCE_DROP_M,
            base_pos_bias_m: [0.0; 3],
            base_pos_drift_mps: [0.0; 3],
            vel_obs: VelObs::Truth,
            gait_mode: GaitMode::Mpc,
            mpc_predicted_footstep: false,
            legged_control_parity: false,
            dynamic_joint_q_reference: false,
            mpc_horizon_steps: None,
            mpc_px_cost: None,
            fcm_taskspace_jv_weight: None,
            fcm_warm_start: false,
            fcm_horizon_steps: None,
            fcm_dt_per_step: None,
            fcm_sqp_iterations: None,
            fcm_sparse_qp: false,
            fcm_grf_cost: None,
            fcm_jointv_cost: None,
            attitude_pd_ablation: None,
            staircase: None,
            contact_reflex: None,
            terrain_footplan: None,
            fcm_state_cost: None,
            base_accel_coriolis: false,
            flat_wbc_weights: false,
            drop_tau_gravity: false,
            mpc_composite_inertia: false,
            actuation: Actuation::PositionTorque,
            host_rate_hz: None,
            cmd_schedule: Vec::new(),
            #[cfg(feature = "mujoco-viewer")]
            live_teleop: None,
            live_viewer: false,
            push: None,
        }
    }

    /// Same schedule as [`Self::static_stand`], routed through
    /// `WbcSolver` with the given formulation/strategy/backend.
    pub fn static_stand_misa_wbc(formulation: wbc::Formulation, cfg: wbc::SolveConfig) -> Self {
        Self { misa_wbc_mode: Some((formulation, cfg)), ..Self::static_stand() }
    }

    /// Same schedule as [`Self::forward_walk`], routed through
    /// `WbcSolver` with the given formulation/strategy/backend.
    pub fn forward_walk_misa_wbc(formulation: wbc::Formulation, cfg: wbc::SolveConfig) -> Self {
        Self { misa_wbc_mode: Some((formulation, cfg)), ..Self::forward_walk() }
    }
}

/// The settings each gait was tuned to, and the numbers they have to hold.
///
/// `k_capture = 0.0` is not an oversight -- see `namiashi_tuned_gaits_hold`.
pub const NAMIASHI_TUNED: [(GaitType, f64, f64, f64, f64, f64); 3] = [
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

pub fn namiashi_tuned_params(i: usize) -> WbcParams {
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
        k_capture_s: Some(NAMIASHI_CAPTURE_GAIN_S),
        ..WbcParams::forward_walk()
    }
}

/// That gait's own row in [`NAMIASHI_TUNED`], or Trot's if it has none
/// (only Trot/Walk/Crawl were ever tuned for namiashi).
fn namiashi_tuned_row(gait: GaitType) -> (GaitType, f64, f64, f64, f64, f64) {
    NAMIASHI_TUNED
        .iter()
        .copied()
        .find(|row| row.0 == gait)
        .unwrap_or(NAMIASHI_TUNED[0])
}

/// The forward speed [`NAMIASHI_TUNED`] settled on for this gait. Each is
/// bounded by its own `max_step_length_m / (cycle_period_s * duty_factor)`,
/// so they are genuinely different numbers, not one speed with different
/// leg timings -- see [`crate::teleop::SpeedEnvelope::for_gait`].
pub fn namiashi_tuned_cmd_vx(gait: GaitType) -> f64 {
    namiashi_tuned_row(gait).5
}

/// The peak swing-foot lift [`NAMIASHI_TUNED`] settled on for this gait.
pub fn namiashi_tuned_swing_height_m(gait: GaitType) -> f64 {
    namiashi_tuned_row(gait).4
}

/// A [`GaitConfig`] carrying this gait's tuned period/duty/step/swing.
/// Used to swap gaits at runtime (`GaitController::set_config`) in the
/// interactive teleop demo, so a live switch lands on exactly the same
/// settings a batch run of that gait would have used.
pub fn namiashi_tuned_gait_config(gait: GaitType) -> GaitConfig {
    let (_, t, duty, step, h, _) = namiashi_tuned_row(gait);
    let mut cfg = match gait {
        GaitType::Walk => GaitConfig::walk(),
        GaitType::Pace => GaitConfig::pace(),
        GaitType::Bound => GaitConfig::bound(),
        GaitType::Crawl => GaitConfig::crawl(),
        GaitType::Trot => GaitConfig::trot(),
    };
    cfg.cycle_period_s = t;
    cfg.duty_factor = duty;
    cfg.max_step_length_m = step;
    cfg.swing_height_m = h;
    cfg
}

/// Run a WBC sim, sampling per-tick. Returns `None` if the namiashi
/// fixture is missing (skip cleanly).
pub fn run_wbc_sim(params: WbcParams) -> Option<Vec<WbcSample>> {
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
                Actuation::Torque { .. }
                | Actuation::TorqueLeggedControl { .. }
                | Actuation::LeggedControl { .. } => {
                    robot.joints[ji].actuator_mode = ActuatorMode::Torque;
                }
                Actuation::PositionTorque => unreachable!(),
            }
        }
    }

    let opts = MjcfExportOptions {
        ground_plane: if params.staircase.is_none() {
            Some(GroundPlaneCfg { z: 0.0, half_size: 4.0, roll: 0.0, pitch: 0.0 })
        } else {
            None
        },
        extra_worldbody_xml: params.staircase.map(|s| s.worldbody_xml()),
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
            crate::mjcf::export_mjcf_with_options(&robot, opts.clone()),
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
    // `mut` only because live teleop can swap the gait mid-run; every
    // non-interactive caller leaves both of these at their built values.
    #[allow(unused_mut)]
    let mut cycle_period_s = cfg.cycle_period_s;
    #[cfg(feature = "mujoco-viewer")]
    let mut live_gait = cfg.gait_type;
    #[cfg(feature = "mujoco-viewer")]
    let mut live_swing_height_m = cfg.swing_height_m;
    let mut gc = GaitController::build(&robot, kin.clone(), cfg, params.gait_mode)
        .expect("GaitController::build");
    if params.mpc_predicted_footstep {
        gc.set_use_mpc_predicted_footstep(true);
    }
    if params.mpc_horizon_steps.is_some() || params.mpc_px_cost.is_some() {
        if let Some(c) = gc.srbd_mpc_config() {
            let mut cfg = c.clone();
            if let Some(h) = params.mpc_horizon_steps {
                cfg.horizon_steps = h;
            }
            if let Some(w) = params.mpc_px_cost {
                // q_diag layout: [theta(3); p(3); omega(3); v(3); g].
                cfg.q_diag[3] = w;
            }
            eprintln!(
                "[srbd] horizon={} ({:.3}s)  q_diag[p_x]={}",
                cfg.horizon_steps,
                cfg.horizon_steps as f64 * cfg.dt_per_step,
                cfg.q_diag[3],
            );
            gc.set_srbd_mpc_config(cfg);
        }
    }
    if let Some(w) = params.fcm_taskspace_jv_weight {
        gc.set_task_space_joint_vel_weight(Some(w));
        eprintln!("[fcm] task-space joint-vel weight = {w:?}");
    }
    if params.fcm_warm_start {
        if let Some(c) = gc.full_centroidal_mpc_config() {
            let mut cfg = c.clone();
            cfg.warm_start = true;
            cfg.sqp_iterations = 1;
            gc.set_full_centroidal_mpc_config(cfg);
            eprintln!("[fcm] warm start, 1 SQP iteration");
        }
    }
    if params.fcm_grf_cost.is_some()
        || params.fcm_jointv_cost.is_some()
        || params.fcm_state_cost.is_some()
    {
        if let Some(c) = gc.full_centroidal_mpc_config() {
            let mut cfg = c.clone();
            if let Some(w) = params.fcm_grf_cost {
                for r in cfg.r_diag.iter_mut().take(12) {
                    *r = w;
                }
            }
            if let Some(w) = params.fcm_jointv_cost {
                for r in cfg.r_diag.iter_mut().skip(12) {
                    *r = w;
                }
                eprintln!("[fcm] r_diag joint_v = {w}");
            }
            if let Some(q) = params.fcm_state_cost {
                let blocks = [(0, 3), (3, 6), (6, 9), (9, 12), (12, 24)];
                for (bi, (a, b)) in blocks.iter().enumerate() {
                    for e in cfg.q_diag[*a..*b].iter_mut() {
                        *e = q[bi];
                    }
                }
                eprintln!(
                    "[fcm] q_diag: v_com={} omega={} pos={} euler={} joint_q={}",
                    q[0], q[1], q[2], q[3], q[4]
                );
            }
            gc.set_full_centroidal_mpc_config(cfg);
        }
    }
    if params.gait_mode == GaitMode::FullCentroidal
        && (params.fcm_horizon_steps.is_some()
            || params.fcm_dt_per_step.is_some()
            || params.fcm_sqp_iterations.is_some()
            || params.fcm_sparse_qp)
    {
        if let Some(c) = gc.full_centroidal_mpc_config() {
            let mut cfg = c.clone();
            if let Some(n) = params.fcm_horizon_steps {
                cfg.horizon_steps = n;
            }
            if let Some(dt) = params.fcm_dt_per_step {
                cfg.dt_per_step = dt;
            }
            if let Some(n) = params.fcm_sqp_iterations {
                cfg.sqp_iterations = n;
            }
            if params.fcm_sparse_qp {
                cfg.sparse_qp = true;
            }
            gc.set_full_centroidal_mpc_config(cfg);
        }
    }
    if params.gait_mode == GaitMode::FullCentroidal {
        if let Some(c) = gc.full_centroidal_mpc_config() {
            let i = c.centroidal_inertia_body;
            eprintln!(
                "[fcm] m={:.3}kg  mu={:.2}  fz_max={:.1}N  I=({:.5},{:.5},{:.5})  \
                 horizon={} dt={:.3} sqp={} sparse={}",
                c.mass_kg, c.friction_mu, c.max_normal_force,
                i[(0, 0)], i[(1, 1)], i[(2, 2)],
                c.horizon_steps, c.dt_per_step, c.sqp_iterations, c.sparse_qp,
            );
        } else {
            eprintln!("[fcm] config unavailable");
        }
    }
    if params.legged_control_parity {
        gc.set_legged_control_parity(true);
    }
    if params.dynamic_joint_q_reference {
        gc.set_dynamic_joint_q_reference(true);
    }
    if params.mpc_composite_inertia {
        let c = auto_detect_centroidal_mpc_config(&robot);
        let i = c.centroidal_inertia_body;
        // Start from whatever is already configured, not from a fresh
        // auto-detect. Rebuilding from scratch here silently discarded a
        // horizon of 30 and a p_x cost of 1000 that had been set earlier in
        // this same function, and the sweep reported three rungs of a ladder
        // as having no effect.
        let mut cfg = gc
            .srbd_mpc_config()
            .cloned()
            .unwrap_or_else(|| auto_detect_srbd_mpc_config(&robot));
        cfg.mass_kg = c.mass_kg;
        cfg.inertia_diag_body =
            Vector3::new(i[(0, 0)], i[(1, 1)], i[(2, 2)]);
        eprintln!(
            "[mpc] composite inertia: m={:.3}kg  I=({:.5},{:.5},{:.5})",
            cfg.mass_kg,
            cfg.inertia_diag_body.x,
            cfg.inertia_diag_body.y,
            cfg.inertia_diag_body.z,
        );
        gc.set_srbd_mpc_config(cfg);
        // Read it straight back, because a setter that silently does nothing
        // would look exactly like a controller that ignores its inertia.
        match gc.srbd_mpc_config_inertia() {
            Some(b) => eprintln!(
                "[mpc] read back: I=({:.5},{:.5},{:.5})", b.x, b.y, b.z
            ),
            None => eprintln!("[mpc] read back: NONE -- setter did not apply"),
        }
    }
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
    let mg_total: f64 = robot.links.iter().map(|l| l.inertial.mass).sum::<f64>() * 9.81;
    let mut wbc_pipeline = WbcPipeline::new(&robot, foot_links);
    if let Some((formulation, cfg)) = params.misa_wbc_mode.clone() {
        wbc_pipeline = wbc_pipeline.with_wbc_solver(formulation, cfg);
    }
    wbc_pipeline.base_accel_coriolis = params.base_accel_coriolis;
    if params.flat_wbc_weights {
        let w = &mut wbc_pipeline.weights;
        w.floating_base_eom = 1.0;
        w.no_contact_motion = 1.0;
        w.base_accel = 1.0;
        w.swing_leg = 1.0;
        w.contact_force = 1.0;
        w.tau_gravity = 1.0;
    }
    if params.drop_tau_gravity {
        wbc_pipeline.weights.tau_gravity = 0.0;
    }
    if let Some(w) = params.contact_force_weight {
        wbc_pipeline.weights.contact_force = w;
        eprintln!("[wbc] contact_force weight = {w} (default 5.0)");
    }
    if let Some(w) = params.base_accel_weight {
        wbc_pipeline.weights.base_accel = w;
        eprintln!("[wbc] base_accel weight = {w} (default 200.0)");
    }
    if let Some((kp, kd)) = params.yaw_pd_gain {
        wbc_pipeline.yaw_pd_gain = (kp, kd);
        eprintln!("[wbc] yaw_pd_gain = ({kp}, {kd}) (default (0.0, 0.0))");
    }
    if let Some((kp, kd)) = params.attitude_pd_ablation {
        wbc_pipeline.roll_pd_gain = (kp, kd);
        wbc_pipeline.pitch_pd_gain = (kp, kd);
        eprintln!("[wbc] attitude_pd_ablation: roll/pitch PD = ({kp}, {kd}), GRF plan replaced by gravity split");
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
    let mut stance_mask = [true; 4];
    let mut reflex_active = [false; 4];
    // `hip_bias_gate`'s countdown: the sim time (seconds since t=0) at
    // which the gate closes again. Starts at -infinity so it reads as
    // "closed" before the first trigger.
    let mut hip_gate_open_until_s = f64::NEG_INFINITY;
    // Magnitude latched at the moment the gate last opened (see
    // `HipBiasGateCfg::bias_gain`'s doc comment).
    let mut hip_gate_bias_now = 0.0_f64;
    // Committed foothold plan per leg: (world_x, world_z_target), decided
    // once at the first tick of a swing and held fixed until that leg
    // returns to stance. `None` means "no plan yet for this swing" -- the
    // trigger to compute one, not an ongoing per-tick re-evaluation. See
    // `TerrainFootplanCfg`'s doc comment for why this replaced continuous
    // re-targeting.
    let mut planned_foothold: [Option<(f64, f64)>; 4] = [None; 4];
    let mut yaw_prev = 0.0_f64;
    let mut mpc_fx = 0.0_f64;
    let mut mpc_fz = 0.0_f64;
    let mut wbc_fx = 0.0_f64;
    // Host-computed PD for the torque path, paired with its joint index.
    let mut host_pd = [(0usize, 0.0_f64); 12];

    #[cfg(feature = "mujoco-viewer")]
    let mut viewer: Option<mujoco::viewer::MjViewer> = if params.live_viewer {
        let mut v = mujoco::viewer::MjViewer::launch_passive(sim.mj_model().clone(), 0)
            .expect("launch MjViewer");
        if let Some(live) = params.live_teleop.clone() {
            // Bindings live in `crate::teleop` so this and the RL demo
            // (examples/namiashi_rl_teleop.rs) cannot drift apart.
            // `_detached` (no `&mut MjData` param) since this only reads
            // egui's key state and writes `live` -- it never touches sim
            // state, so the cheaper detached path applies directly.
            use crate::teleop::{
                draw_hud, poll_cmd, poll_friction_deltas, poll_gait,
                poll_swing_height_delta, SpeedEnvelope, FRICTION_RANGE,
                SWING_HEIGHT_RANGE_M,
            };
            v.add_ui_callback_detached(move |ctx| {
                let mut st = live.lock().unwrap();
                if let Some(g) = poll_gait(ctx) {
                    st.gait = g;
                }
                let dh = poll_swing_height_delta(ctx);
                if dh != 0.0 {
                    st.swing_height_m = (st.swing_height_m + dh)
                        .clamp(SWING_HEIGHT_RANGE_M.0, SWING_HEIGHT_RANGE_M.1);
                }
                let (dg, dc) = poll_friction_deltas(ctx);
                if dg != 0.0 {
                    st.ground_mu =
                        (st.ground_mu + dg).clamp(FRICTION_RANGE.0, FRICTION_RANGE.1);
                }
                if dc != 0.0 {
                    st.controller_mu =
                        (st.controller_mu + dc).clamp(FRICTION_RANGE.0, FRICTION_RANGE.1);
                }
                // Envelope follows the *requested* gait, so switching to
                // Crawl immediately re-scales what a full-deflection key
                // means rather than leaving Trot's ceiling in place.
                let env = SpeedEnvelope::for_gait(st.gait);
                st.cmd = poll_cmd(ctx, env);
                draw_hud(ctx, &st, env, "WBC / MPC", true);
            });
        }
        Some(v)
    } else {
        None
    };
    #[cfg(not(feature = "mujoco-viewer"))]
    if params.live_viewer {
        panic!("WbcParams::live_viewer requires building with --features mujoco-viewer");
    }
    // ~60 Hz render/sync cadence, independent of the (much finer) physics dt
    // -- rendering every physics tick would be thousands of frames/sec for
    // no visual benefit.
    let render_decim = ((1.0 / 60.0) / params.dt).round().max(1.0) as usize;
    let wall_start = std::time::Instant::now();
    #[cfg(feature = "mujoco-viewer")]
    let mut fps_meter = crate::teleop::FpsMeter::new();
    // Seed the live friction knobs from what the sim and the controller
    // were actually built with, so the HUD opens showing the truth rather
    // than a guess, and the first keypress steps off the real value.
    #[cfg(feature = "mujoco-viewer")]
    let (mut live_ground_mu, mut live_controller_mu) = {
        let g = sim.slide_friction();
        let c = wbc_pipeline.friction_mu;
        if let Some(live) = &params.live_teleop {
            let mut st = live.lock().unwrap();
            st.ground_mu = g;
            st.controller_mu = c;
        }
        (g, c)
    };

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
        // Live teleop command, highest priority -- read every tick (not just
        // on change) since it can change between any two ticks.
        #[cfg(feature = "mujoco-viewer")]
        if k >= burn_in_steps {
            if let Some(live) = &params.live_teleop {
                let st = *live.lock().unwrap();
                let [vx, vy, wz] = st.cmd;
                let now = gc.velocity_cmd();
                if (now.vx - vx).abs() > 1e-9
                    || (now.vy - vy).abs() > 1e-9
                    || (now.wz - wz).abs() > 1e-9
                {
                    gc.set_velocity_cmd(VelocityCmd { vx, vy, wz });
                }
                let gait_changed = st.gait != live_gait;
                let swing_changed = (st.swing_height_m - live_swing_height_m).abs() > 1e-9;
                if gait_changed || swing_changed {
                    // One `set_config` covers both: it swaps
                    // period/duty/step/swing AND the per-leg phase offsets.
                    // Rebuilding from the tuned row each time is idempotent
                    // when only the swing height moved.
                    //
                    // The phase generator keeps its current cycle_phase
                    // across the swap, so a gait switch mid-stride moves
                    // legs to their new offsets in one tick -- fine from a
                    // standstill (phase is frozen while the command is
                    // zero, all four feet in stance), a visible transient
                    // if done at speed.
                    let mut cfg = namiashi_tuned_gait_config(st.gait);
                    cfg.swing_height_m = st.swing_height_m;
                    gc.set_config(cfg);
                    cycle_period_s = gc.config().cycle_period_s;
                    live_gait = st.gait;
                    live_swing_height_m = st.swing_height_m;
                    if gait_changed {
                        eprintln!(
                            "[teleop] gait -> {:?} (T={:.3}s duty={:.2} \
                             max_step={:.3}m, tuned cmd_vx={:.3})",
                            st.gait,
                            cycle_period_s,
                            gc.config().duty_factor,
                            gc.config().max_step_length_m,
                            namiashi_tuned_cmd_vx(st.gait),
                        );
                    }
                    if swing_changed {
                        eprintln!(
                            "[teleop] swing height -> {:.3} m (tuned {:.3} m for {:?})",
                            st.swing_height_m,
                            namiashi_tuned_swing_height_m(st.gait),
                            st.gait,
                        );
                    }
                }
                if (st.ground_mu - live_ground_mu).abs() > 1e-9 {
                    sim.set_slide_friction_all(st.ground_mu);
                    live_ground_mu = st.ground_mu;
                    eprintln!("[teleop] ground mu -> {:.2}", st.ground_mu);
                }
                if (st.controller_mu - live_controller_mu).abs() > 1e-9 {
                    // Both consumers of the belief: the WBC QP's friction
                    // cone (what actually clips the commanded tangential
                    // force) and the MPC's own force planning. Setting one
                    // and not the other would leave the plan and the
                    // solver disagreeing about the same ground.
                    wbc_pipeline.friction_mu = st.controller_mu;
                    if let Some(mut mpc) = gc.srbd_mpc_config().cloned() {
                        mpc.friction_mu = st.controller_mu;
                        gc.set_srbd_mpc_config(mpc);
                    }
                    live_controller_mu = st.controller_mu;
                    eprintln!("[teleop] controller mu -> {:.2}", st.controller_mu);
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
        // Feed the MPC's own body-pose estimate -- until now nothing called
        // this, so `body_state.world_yaw` inside the vendored MPC
        // (quadruped-gait) stayed pinned at its open-loop-integrated value
        // (identically 0.0 here, since cmd_wz==0 the whole run) regardless
        // of the real robot's yaw. The MPC uses that yaw to rotate each
        // foot's body-frame offset into world frame for its r_i x f_i
        // moment-arm balance (mpc_controller.rs's own comment: "any
        // integrated yaw makes the cross product mix frames and breaks
        // in-place rotation tracking") -- once a riser disturbs the real
        // yaw, every subsequent corrective GRF is computed against the
        // WRONG foot geometry, which can easily point the "correction"
        // the wrong way and amplify the drift instead of damping it. Using
        // ground-truth yaw/position here (matches `PoseSource::GroundTruth`
        // in gait.rs, already an established pattern in
        // tests/integration_walk.rs) isolates whether this wiring gap
        // alone explains the runaway drift, before swapping in a
        // proprioceptive estimator (Madgwick IMU fusion + leg odometry,
        // both already implemented -- see gait.rs's `PoseSource`) for a
        // real-hardware-deployable version.
        let yaw_obs = sim.body_world_yaw(&robot.root_link).unwrap_or(0.0);
        let pos_obs = sim.body_world_position(&robot.root_link).unwrap_or([0.0, 0.0, 0.0]);
        gc.set_body_pose_observed(yaw_obs, Vector3::new(pos_obs[0], pos_obs[1], pos_obs[2]));

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
            // Freeze the gait's phase clock -- not just the disrupted leg's
            // target -- while any leg is mid-reflex. `reflex_active` here is
            // still last tick's value (this tick's has not been computed
            // yet), which is exactly what is wanted: the decision has to be
            // made before `gc.tick()` runs, not after.
            let phase_frozen = params
                .contact_reflex
                .is_some_and(|cfg| cfg.freeze_phase_during_reflex)
                && reflex_active.iter().any(|&a| a);
            let gait_dt = if phase_frozen { 0.0 } else { host_dt };
            let (out, targets, torque_ff) = gc.tick(gait_dt);
            let mut targets = targets;
            // `out` is shadowed mutable so any per-leg override below can
            // rewrite `out.legs[slot].{q_hip,q_thigh,q_calf}` in place, not
            // just `targets`. `wbc_pipeline.solve()` (called later this tick)
            // takes `&out` directly and its swing_leg task reads those three
            // fields for its own q_ddot reference -- independently of
            // `targets`, which only feeds the host-side PD in
            // `Actuation::Torque`. The two paths are summed
            // (`pd + tau_wbc`), not one overriding the other, so writing only
            // `targets` left the WBC's swing task pulling every overridden
            // leg back toward the original flat-ground target the whole
            // time, fighting the correction rather than sharing it.
            let mut out = out;
            for slot in 0..4 {
                stance_mask[slot] = out.legs[slot].phase.is_stance;
            }
            // Contact reflex: only meaningful once the WBC has taken over
            // (before that every leg is held on position gains regardless of
            // gait phase, so "swing FK error" isn't measuring a collision).
            if let Some(cfg) = params.contact_reflex {
                if k >= burn_in_steps {
                    for slot in 0..4 {
                        if out.legs[slot].phase.is_stance {
                            reflex_active[slot] = false;
                            continue;
                        }
                        let leg_kin = gc.kinematics().legs()[slot];
                        let signs = gc.joint_signs()[slot];
                        let ji = gc.joint_indices()[slot];
                        let mut q_leg = [0.0_f64; 3];
                        for kk in 0..3 {
                            if let Some((q, _)) = sim.joint_q_qd(&robot.joints[ji[kk]].name) {
                                q_leg[kk] = signs[kk] * q;
                            }
                        }
                        let measured_body = quadruped_gait::forward_leg_kinematics(
                            leg_kin, q_leg[0], q_leg[1], q_leg[2],
                        );
                        let nominal_body = out.legs[slot].foot_body;
                        let lift_target = nominal_body + Vector3::new(0.0, 0.0, cfg.lift_m);
                        // Trigger reads error against the *nominal* target --
                        // that is the collision signal. Resume must read error
                        // against the *lift* target instead: while active this
                        // leg is deliberately not tracking nominal, so it sits
                        // ~lift_m away from it even once the manoeuvre has
                        // fully succeeded. Comparing against nominal here was
                        // a bug in the first two attempts -- with lift_m
                        // (0.06) above resume_m (0.025), it could never
                        // satisfy its own exit condition, and combined with
                        // `freeze_phase_during_reflex` (which stops the
                        // nominal target from moving at all) it deadlocked
                        // permanently: q_dot pinned at exactly 0.000 rad/s for
                        // the rest of a 20 s run.
                        let err_nominal = (nominal_body - measured_body).norm();
                        let err_lift = (lift_target - measured_body).norm();
                        if reflex_active[slot] {
                            if err_lift < cfg.resume_m {
                                reflex_active[slot] = false;
                            }
                        } else if err_nominal > cfg.trigger_m {
                            reflex_active[slot] = true;
                        }
                        if reflex_active[slot] {
                            // Lift the *intended* touchdown, not the stuck
                            // position: pushing straight up from wherever the
                            // foot currently was does not advance it past the
                            // obstacle horizontally -- it comes back down on
                            // the same edge and re-triggers. Adding the lift
                            // to the nominal target keeps the leg heading
                            // toward wherever the gait's own swing curve
                            // wants it, just higher.
                            let sol = solve_leg_ik(
                                leg_kin, lift_target, gc.knee_forward()[slot],
                            );
                            let (hip, thigh, calf) = sol.angles();
                            let q_ik = [hip, thigh, calf];
                            for kk in 0..3 {
                                targets[slot * 3 + kk] = (ji[kk], signs[kk] * q_ik[kk]);
                            }
                            // Keep the WBC's swing_leg reference in sync --
                            // see the comment where `out` is shadowed mutable
                            // above.
                            out.legs[slot].q_hip = hip;
                            out.legs[slot].q_thigh = thigh;
                            out.legs[slot].q_calf = calf;
                            out.legs[slot].foot_body = lift_target;
                        }
                    }
                }
            }
            if let Some(cfg) = params.terrain_footplan {
                if k >= burn_in_steps {
                    let body_pos_world = robot.base_transform.translation.vector;
                    let (_, _, yaw_now) = robot.base_transform.rotation.euler_angles();
                    let (cy, sy) = (yaw_now.cos(), yaw_now.sin());
                    for slot in 0..4 {
                        if out.legs[slot].phase.is_stance {
                            continue;
                        }
                        let nominal_body = out.legs[slot].foot_body;
                        // `footstep.touch_down` is the Raibert planner's own
                        // fixed target for this swing -- decided once by the
                        // gait controller itself and held constant until
                        // landing, unlike `foot_body`, which is *this tick's*
                        // point along the lift_off -> touch_down arc. An
                        // earlier version of this block sampled `foot_body`
                        // at the first swing tick to decide a target, which
                        // is nearly the *lift-off* position, not touchdown --
                        // it was planning to land almost where the foot
                        // started, not where the gait actually intended.
                        // Reading the planner's own target instead needs no
                        // state of this block's own to freeze anything: it is
                        // already fixed for the whole swing by construction.
                        let touch_down_body = out.legs[slot].footstep.touch_down;
                        let touch_down_world_x = body_pos_world.x
                            + cy * touch_down_body.x
                            - sy * touch_down_body.y;
                        let terrain_z = params
                            .staircase
                            .map(|s| s.height_at(touch_down_world_x))
                            .unwrap_or(0.0);
                        // Flat ground where this swing is headed -- no plan
                        // needed, leave it alone.
                        if terrain_z <= 1e-6 {
                            continue;
                        }
                        let world_x_target = if cfg.horizontal_margin_m > 0.0 {
                            params
                                .staircase
                                .map(|s| s.snap_to_tread(touch_down_world_x, cfg.horizontal_margin_m))
                                .unwrap_or(touch_down_world_x)
                        } else {
                            touch_down_world_x
                        };
                        // Apply the snap as a small delta on top of the
                        // *current* interpolated position (`nominal_body`,
                        // preserving the natural swing arc) rather than
                        // solving for an exact body-frame x via `/cy`. That
                        // division is only safe for small yaw, and this
                        // staircase routinely produces much more than that
                        // (40-115 deg excursions are the norm in the failure
                        // modes this whole investigation has been chasing) --
                        // an earlier version of this line divided by `cy` and
                        // sent the body to z=-219 m the one time yaw swung
                        // wide during a run. `delta_world_x` is usually 0
                        // (most touchdowns need no snapping at all), and the
                        // multiply-only form cannot blow up regardless of
                        // yaw.
                        let delta_world_x = world_x_target - touch_down_world_x;
                        let target_x = nominal_body.x + cy * delta_world_x;
                        let target_y_snap = nominal_body.y - sy * delta_world_x;
                        let desired_body_z =
                            terrain_z + cfg.clearance_m - body_pos_world.z;
                        // Never target *below* the open-loop plan -- this
                        // only adds clearance over what the terrain needs,
                        // it does not shortcut a foot down early.
                        let target_z = desired_body_z.max(nominal_body.z);
                        let lift_target = Vector3::new(target_x, target_y_snap, target_z);
                        let leg_kin = gc.kinematics().legs()[slot];
                        let signs = gc.joint_signs()[slot];
                        let ji = gc.joint_indices()[slot];
                        let sol = solve_leg_ik(leg_kin, lift_target, gc.knee_forward()[slot]);
                        let (hip, thigh, calf) = sol.angles();
                        let q_ik = [hip, thigh, calf];
                        for kk in 0..3 {
                            targets[slot * 3 + kk] = (ji[kk], signs[kk] * q_ik[kk]);
                        }
                        // Keep the WBC's swing_leg reference in sync -- see
                        // the comment where `out` is shadowed mutable above.
                        out.legs[slot].q_hip = hip;
                        out.legs[slot].q_thigh = thigh;
                        out.legs[slot].q_calf = calf;
                        out.legs[slot].foot_body = lift_target;
                        if std::env::var_os("NAMI_FOOTPLAN_LOG").is_some() && k % 20 == 0 {
                            eprintln!(
                                "[footplan k={k:5} slot={slot}] world_x_target={world_x_target:.3} \
                                 terrain_z={terrain_z:.3} nominal.z={:.4} target_z={target_z:.4} \
                                 reachable={:?}",
                                nominal_body.z,
                                matches!(sol, LegIkSolution::Reached { .. }),
                            );
                        }
                    }
                }
            }
            if let Some(cfg) = params.hip_bias_gate {
                if k >= burn_in_steps {
                    // Same FK-tracking-error collision signal
                    // `ContactReflexCfg` uses, computed independently here
                    // (see `hip_bias_gate`'s doc comment) -- max over
                    // currently-swinging legs only, matching the reflex's
                    // own "stance legs can't be mid-collision" reasoning.
                    let mut max_err = 0.0_f64;
                    for slot in 0..4 {
                        if out.legs[slot].phase.is_stance {
                            continue;
                        }
                        let leg_kin = gc.kinematics().legs()[slot];
                        let signs = gc.joint_signs()[slot];
                        let ji = gc.joint_indices()[slot];
                        let mut q_leg = [0.0_f64; 3];
                        for kk in 0..3 {
                            if let Some((q, _)) = sim.joint_q_qd(&robot.joints[ji[kk]].name) {
                                q_leg[kk] = signs[kk] * q;
                            }
                        }
                        let measured_body = quadruped_gait::forward_leg_kinematics(
                            leg_kin, q_leg[0], q_leg[1], q_leg[2],
                        );
                        let nominal_body = out.legs[slot].foot_body;
                        max_err = max_err.max((nominal_body - measured_body).norm());
                    }
                    if max_err > cfg.trigger_m {
                        hip_gate_open_until_s = t + cfg.duration_s;
                        hip_gate_bias_now = (cfg.bias_mag + cfg.bias_gain * (max_err - cfg.trigger_m))
                            .min(cfg.max_bias_rad);
                    }
                    if t < hip_gate_open_until_s {
                        for slot in 0..4 {
                            let signs = gc.joint_signs()[slot];
                            let leg_bias = if slot == 0 || slot == 2 { hip_gate_bias_now } else { -hip_gate_bias_now };
                            out.legs[slot].q_hip += leg_bias;
                            let (idx, q) = targets[slot * 3];
                            targets[slot * 3] = (idx, q + signs[0] * leg_bias);
                        }
                    }
                }
            }
            if let Some(bias) = params.hip_lr_bias_rad {
                // slot order is LegId::ALL = [FL, FR, RL, RR] -- left legs
                // are slots 0 and 2, right legs 1 and 3. Applied to BOTH
                // `out` (keeps the WBC's swing_leg q_ddot reference in
                // sync, same reasoning as the reflex/footplan blocks
                // above) and `targets` (the host-side PD target actually
                // sent to the sim) every tick, stance and swing alike --
                // unlike those two blocks this is not conditioned on
                // swing phase or burn-in, since the RL trace this is
                // testing showed the asymmetry held constantly, not just
                // during a swing correction.
                for slot in 0..4 {
                    let signs = gc.joint_signs()[slot];
                    let leg_bias = if slot == 0 || slot == 2 { bias } else { -bias };
                    out.legs[slot].q_hip += leg_bias;
                    let (idx, q) = targets[slot * 3];
                    targets[slot * 3] = (idx, q + signs[0] * leg_bias);
                }
            }
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
                Actuation::LeggedControl { kp, kd } => {
                    // Uniform gains, velocity feedforward from the
                    // trajectory, no gravity term -- the WBC's tau carries
                    // it, as it does in legged_control where nothing else
                    // adds one.
                    for (slot, &(idx, q_star)) in targets.iter().enumerate() {
                        let qd_star = if k > burn_in_steps {
                            (q_star - prev_targets[slot]) / host_dt
                        } else {
                            0.0
                        };
                        let (q_meas, qd_meas) = sim
                            .joint_q_qd(&robot.joints[idx].name)
                            .unwrap_or((q_star, 0.0));
                        let pd =
                            kp * (q_star - q_meas) + kd * (qd_star - qd_meas);
                        host_pd[slot] = (idx, pd);
                        sim.set_torque_target(idx, pd);
                        prev_targets[slot] = q_star;
                    }
                }
                Actuation::TorqueLeggedControl {
                    swing_kp,
                    swing_kd,
                    stance_kp,
                    stance_kd,
                    bias_ff,
                } => {
                    // Gains chosen per leg by gait phase, the way
                    // legged_control does: a stance leg is a force source, a
                    // swing leg is a position tracker.
                    //
                    let bias = if bias_ff != 0.0 {
                        sim.bias_torques(&robot)
                    } else {
                        vec![0.0; robot.joints.len()]
                    };
                    // Before burn-in the WBC is not running, so a stance leg
                    // at kp=0 has no command at all and the robot free-falls.
                    // Hold every leg on position gains until then, the way a
                    // real startup sequence would, and hand over to the
                    // stance/swing split once the WBC is up.
                    //
                    // This is the second time this bug has appeared -- the
                    // same z_min of 0.039 showed up on `Actuation::Torque`
                    // for the same reason. Any path with no driver-side loop
                    // needs a command written on every tick, burn-in
                    // included.
                    let handed_over = k >= burn_in_steps;
                    for (slot, tri) in gc.joint_indices().iter().enumerate() {
                        let stance = handed_over && out.legs[slot].phase.is_stance;
                        for (j, &ji) in tri.iter().enumerate() {
                            let k = slot * 3 + j;
                            let q_star = targets[k].1;
                            let (q_meas, qd_meas) = sim
                                .joint_q_qd(&robot.joints[ji].name)
                                .unwrap_or((q_star, 0.0));
                            let pd = if stance {
                                stance_kp * (q_star - q_meas) - stance_kd * qd_meas
                            } else {
                                swing_kp * (q_star - q_meas) - swing_kd * qd_meas
                            };
                            let tau = pd + bias_ff * bias[ji];
                            host_pd[k] = (ji, tau);
                            sim.set_torque_target(ji, tau);
                        }
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
                let f_grf_world = if params.attitude_pd_ablation.is_some() {
                    // Quasi-static gravity split across the feet currently in
                    // stance per the gait's own phase (not yet the corrected
                    // early-touchdown flags, which are computed a few lines
                    // below from measured force -- open-loop is enough here,
                    // this is a *reference*, not a contact determination).
                    let n_stance = out.legs.iter().filter(|l| l.phase.is_stance).count().max(1);
                    let f_each = mg_total / n_stance as f64;
                    let mut f = [Vector3::zeros(); 4];
                    for (slot, leg) in out.legs.iter().enumerate() {
                        if leg.phase.is_stance {
                            f[slot] = Vector3::new(0.0, 0.0, f_each);
                        }
                    }
                    f
                } else {
                    gc.predicted_grfs()
                        .map(|sol| sol.grfs_first_step)
                        .unwrap_or([Vector3::zeros(); 4])
                };
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
                {
                    let (cy, sy) = ((-yaw_prev).cos(), (-yaw_prev).sin());
                    mpc_fx = f_grf_world
                        .iter()
                        .map(|v| cy * v.x - sy * v.y)
                        .sum();
                    mpc_fz = f_grf_world.iter().map(|v| v.z).sum();
                }
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
                {
                    let (cy, sy) = ((-yaw_prev).cos(), (-yaw_prev).sin());
                    wbc_fx = wbc_pipeline
                        .last_solution
                        .as_ref()
                        .map(|sol| {
                            (0..4)
                                .map(|i| {
                                    cy * sol.f_grf[3 * i] - sy * sol.f_grf[3 * i + 1]
                                })
                                .sum()
                        })
                        .unwrap_or(0.0);
                }
                if k == burn_in_steps + 2000 || k == burn_in_steps + 2050 {
                    if let Some(sol) = wbc_pipeline.last_solution.as_ref() {
                        // Exact dimensions, from the solution vector itself:
                        // x = [q_ddot (nv); f (3nc); tau (na)].
                        let nv = sol.q_ddot.len();
                        let na = sol.tau.len();
                        let n_dec = nv + sol.f_grf.len() + na;
                        let n_st = contact_flag.iter().filter(|b| **b).count();
                        let n_sw = 4 - n_st;
                        // P0 equalities: EoM (nv) + no-contact-motion (3/stance)
                        // + friction-cone swing-zero (3/swing) = nv + 12.
                        let p0 = nv + 3 * n_st + 3 * n_sw;
                        // P1: base accel (6) + swing leg (3 actuators/swing leg).
                        let p1 = 6 + 3 * n_sw;
                        eprintln!(
                            "[null k={k}] nv={nv} na={na} n_dec={n_dec} stance={n_st}  \
                             dimZ0={} dimZ1={}  <- freedom left for priority 2",
                            n_dec as i64 - p0 as i64,
                            n_dec as i64 - p0 as i64 - p1 as i64,
                        );
                    }
                }
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
                if matches!(
                    params.actuation,
                    Actuation::Torque { .. }
                        | Actuation::TorqueLeggedControl { .. }
                        | Actuation::LeggedControl { .. }
                ) {
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

        if params.live_viewer {
            #[cfg(feature = "mujoco-viewer")]
            if let Some(v) = &mut viewer {
                if k % render_decim == 0 {
                    // Telemetry for the HUD, at render cadence rather than
                    // every physics tick -- nothing reads it faster. Body
                    // frame, so `vx meas` is directly comparable to the
                    // `vx cmd` shown beside it (a turned robot's world-x
                    // speed is not its forward speed).
                    let fps = fps_meter.tick();
                    if let Some(live) = &params.live_teleop {
                        let p = robot.base_transform.translation;
                        let v_w = sim
                            .body_world_linear_velocity(&robot.root_link)
                            .unwrap_or([0.0; 3]);
                        let v_b = robot
                            .base_transform
                            .rotation
                            .inverse_transform_vector(&Vector3::new(v_w[0], v_w[1], v_w[2]));
                        let mut st = live.lock().unwrap();
                        st.body_x_m = p.x;
                        st.body_z_m = p.z;
                        st.measured_vx_mps = v_b.x;
                        st.sim_time_s = t;
                        st.wall_time_s = wall_start.elapsed().as_secs_f64();
                        st.fps = fps;
                    }
                    v.sync_data(sim.mj_data_mut());
                    let _ = v.render();
                }
                if !v.running() {
                    break;
                }
            }
            // Real-time pacing: physics alone runs faster than real time,
            // so without this the whole run would blow past in a couple of
            // seconds instead of being watchable/steerable live.
            let target = std::time::Duration::from_secs_f64(t + params.dt);
            let elapsed = wall_start.elapsed();
            if elapsed < target {
                std::thread::sleep(target - elapsed);
            }
        }

        // Sample after the step so contact forces and pose are
        // synchronised. `base_transform` was just refreshed by
        // `sim.step → sync_back`.
        let tx = robot.base_transform.translation;
        let (roll, pitch, yaw) = robot.base_transform.rotation.euler_angles();
        yaw_prev = yaw;
        let total_fz_world: f64 =
            sim.contacts().iter().map(|c| c.force_world[2]).sum();
        // Applied joint torque against its own effort limit. `mujoco_sim`
        // clamps to `joint.effort` silently, so without this a saturated
        // actuator is invisible: the harness would report a gait that the
        // hardware cannot produce and nothing would say so.
        let qfrc = sim.qfrc_actuator();
        let mut tau_frac = [0.0f64; 12];
        let mut tau_nm = [0.0f64; 12];
        let mut qd_frac = [0.0f64; 12];
        for (leg, tri) in gc.joint_indices().iter().enumerate() {
            for (j, &ji) in tri.iter().enumerate() {
                let joint = &robot.joints[ji];
                if joint.effort > 0.0 {
                    if let Some(adr) = sim.joint_dof_adr(&joint.name) {
                        if let Some(&t) = qfrc.get(adr) {
                            tau_frac[leg * 3 + j] = (t / joint.effort).abs();
                            tau_nm[leg * 3 + j] = t.abs();
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
        let mut foot_fx = [0.0f64; 4];
        let (cy, sy) = ((-yaw).cos(), (-yaw).sin());
        for (fi, (_, link)) in DEFAULT_FOOT_LINKS.iter().enumerate() {
            let lname = link.to_lowercase();
            for c in sim.contacts().iter().filter(|c| {
                c.body1.to_lowercase() == lname || c.body2.to_lowercase() == lname
            }) {
                foot_fz[fi] += c.force_world[2].abs();
                // Rotated into the heading frame, since a force that is
                // "forward" is only forward relative to where the robot
                // points.
                foot_fx[fi] += cy * c.force_world[0] - sy * c.force_world[1];
            }
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
            foot_fx,
            mpc_fx,
            mpc_fz,
            wbc_fx,
            body_y: tx.y,
            yaw,
            cycle_period_s,
            tau_frac,
            tau_nm,
            qd_frac,
            stance_mask,
        });
    }
    if let Some(dir) = replay_out.as_deref() {
        std::fs::write(format!("{dir}/trace.csv"), &replay_buf)
            .expect("write replay trace.csv");
        eprintln!("[replay] wrote {dir}/model.xml and trace.csv");
    }
    Some(samples)
}

