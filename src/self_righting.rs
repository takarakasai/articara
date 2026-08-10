//! Getting namiashi off its back.
//!
//! An open-loop joint trajectory with three regimes, switched on one
//! measured quantity (the trunk's world +z) plus one sign (which side of the
//! trunk is underneath). Both come from the attitude estimate, so this needs
//! nothing the robot does not already have -- IMU and joint encoders, no
//! exteroception.
//!
//! The shape is not a guess. `examples/namiashi_self_righting_probe`
//! measured 18 hand-written plans across 5 sites on the ring and 3 friction
//! values and found:
//!
//!   * the arm, despite 6.865 N.m against a hip's 2.5 and a 0.29 m reach,
//!     never gets the trunk near vertical -- its axis is pitch, so it can
//!     only tip the body over its longest dimension (0.294 m hip to hip);
//!   * repeated leg splay pulses roll the body onto its side (about 45
//!     degrees) reliably, from every site and friction tried;
//!   * no hand-written plan crossed the remaining 45 degrees; and
//!   * that ceiling is NOT kinematic -- widening hip roll from 1.05 to 2.40
//!     rad left the result bit-identical.
//!
//! So the primitive works and the schedule is what is missing, which is what
//! [`RecoveryParams`] exposes to search. Front and rear legs are tied
//! together (FL=RL, FR=RR): the probe's working plans were all front/rear
//! symmetric, and halving the dimension is worth more than the asymmetry is
//! likely to buy.

/// Joint order used throughout: FL, FR, RL, RR, each `[hip, thigh, calf]`.
pub type LegTargets = [[f64; 3]; 4];

/// Hip roll limits from the .misa, per side. Left legs (FL, RL) and right
/// legs (FR, RR) are mirrored, so a "push left" and a "push right" of equal
/// magnitude are not equally available.
pub const HIP_LIMIT_L: (f64, f64) = (-0.785, 1.05);
pub const HIP_LIMIT_R: (f64, f64) = (-1.05, 0.785);
pub const THIGH_LIMIT: (f64, f64) = (-2.62, 2.62);
pub const CALF_LIMIT: (f64, f64) = (-2.62, 2.62);
pub const ARM_LIMIT: (f64, f64) = (-2.3, 0.85);

/// The pose the robot holds when it is not recovering -- the .misa's spawn
/// stance, i.e. what `respawn` restores.
pub const STANCE: [f64; 3] = [0.0, 0.9, -1.8];

/// Above this much trunk +z the recovery is over and the robot just stands.
pub const UPRIGHT_UP: f64 = 0.85;

/// An open-loop recovery trajectory.
///
/// Regime 1 (`up < handoff_up`): a rocking cycle, `push_frac` of each
/// `period_s` spent in the push pose and the rest back at the rest pose,
/// with `ramp_s` to move between them. Ramp matters more than it looks: the
/// arm's actuator kp is 5.0 against the legs' 100.0, so a ramped target
/// commands very little arm torque while a step saturates it.
///
/// Regime 2 (`handoff_up <= up < UPRIGHT_UP`): past vertical and still
/// rolling. The legs underneath extend to push the trunk across; the ones on
/// top fold so they cannot prop it and stall the roll. Which side is
/// underneath comes from the sign of gravity's y component in the trunk
/// frame.
///
/// Regime 3: [`STANCE`], and stand up.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RecoveryParams {
    pub period_s: f64,
    pub push_frac: f64,
    pub ramp_s: f64,
    /// Hip roll in the push pose, `[left, right]`.
    pub hip_push: [f64; 2],
    /// Hip roll in the rest pose, `[left, right]`.
    pub hip_rest: [f64; 2],
    pub thigh_push: f64,
    pub calf_push: f64,
    pub thigh_rest: f64,
    pub calf_rest: f64,
    pub arm_push: f64,
    pub arm_rest: f64,
    /// Trunk +z at which regime 1 hands over to regime 2.
    pub handoff_up: f64,
    /// Regime 2 hip roll magnitude for the legs underneath; the sign is set
    /// by which side that is.
    pub finish_hip: f64,
    pub finish_thigh: f64,
    pub finish_calf: f64,
    pub finish_arm: f64,
    /// Regime 3. Searched rather than fixed at [`STANCE`] because the first
    /// search round righted the robot in 10 of 15 conditions and four of the
    /// five failures reached a peak of 0.85 to 0.98 first -- they stood up
    /// and toppled back. A nominal stance has no margin against the angular
    /// momentum left over from the roll; a wider one might. Left legs take
    /// `+stand_hip` and right legs `-stand_hip`, so the sign chooses whether
    /// the stance widens or narrows.
    pub stand_hip: f64,
    pub stand_thigh: f64,
    pub stand_calf: f64,
}

/// The best hand-written plan from the probe, as a starting point for
/// search: a 0.1 s symmetric splay pulse every 2 s with the arm stepped
/// down. Rolls onto its side every time and stops there.
pub const PROBE_BEST: RecoveryParams = RecoveryParams {
    period_s: 2.0,
    push_frac: 0.5,
    ramp_s: 0.10,
    hip_push: [1.05, 0.785],
    hip_rest: [0.0, 0.0],
    thigh_push: 0.0,
    calf_push: -0.5,
    thigh_rest: 0.9,
    calf_rest: -1.8,
    arm_push: -2.3,
    arm_rest: 0.85,
    handoff_up: 0.6,
    finish_hip: 1.05,
    finish_thigh: 0.2,
    finish_calf: -0.5,
    finish_arm: 0.85,
    stand_hip: 0.0,
    stand_thigh: STANCE[1],
    stand_calf: STANCE[2],
};

/// Round 1 of the CEM search in `examples/namiashi_self_righting_search`:
/// trained on 6 conditions (2 sites x 3 friction), 6/6 on those and 10/15
/// over the full grid, against 1/15 for the best hand-written plan.
pub const SEARCHED_V1: RecoveryParams = RecoveryParams {
    period_s: 1.8126687973621372,
    push_frac: 0.6944644628777423,
    ramp_s: 0.11522486352708783,
    hip_push: [0.9620204067463585, 0.7546771946344781],
    hip_rest: [-0.3918493133503482, 0.16646240728247277],
    thigh_push: -0.774712182335545,
    calf_push: -0.9340819340746747,
    thigh_rest: -2.0890661028677333,
    calf_rest: -1.9721905662017687,
    arm_push: -2.110021721638257,
    arm_rest: 0.15883274785937398,
    handoff_up: 0.7467059294643767,
    finish_hip: 0.6681007974414791,
    finish_thigh: -0.04819512389931907,
    finish_calf: -0.16762677828496886,
    finish_arm: 0.85,
    stand_hip: 0.0,
    stand_thigh: STANCE[1],
    stand_calf: STANCE[2],
};

/// Round 2: seeded from [`SEARCHED_V1`], trained on all five ring sites at
/// friction 0.30 and 1.00 with 0.70 held out entirely, and searching the
/// standing pose as well. Rights the robot in all fifteen validation
/// conditions with a final trunk +z of 1.000, the five held-out ones
/// included. For comparison: the best hand-written plan managed 1/15.
pub const SEARCHED_V2: RecoveryParams = RecoveryParams {
    period_s: 1.8826887588548362,
    push_frac: 0.7191825882571744,
    ramp_s: 0.14282697950355674,
    hip_push: [0.8474995305882992, 0.6768977968828486],
    hip_rest: [-0.5649890212703218, -0.17080952763284502],
    thigh_push: -1.583573075859423,
    calf_push: -0.8235622205674531,
    thigh_rest: -1.9596440886388187,
    calf_rest: -2.3176836121285898,
    arm_push: -2.0280483426417875,
    arm_rest: 0.1472016387694903,
    handoff_up: 0.6857466418347575,
    finish_hip: 0.6910705694248478,
    finish_thigh: -0.11530437093683112,
    finish_calf: -0.2417548547087312,
    finish_arm: 0.85,
    stand_hip: 0.2382010722464898,
    stand_thigh: 0.767670263584147,
    stand_calf: -2.0872727063104746,
};

/// Round 3: the first searched against the teleop's own drive law, which
/// lifted that path from 8/15 to 13/15 -- but against an evaluator that let
/// it park in whatever crouch was stable (`stand_thigh` sits at its limit
/// and it settled at 0.920) and that scored at 6 s while validating at 8.
/// Under the corrected objective it scores -0.48. Kept as the record of
/// where the numbers came from.
pub const SEARCHED_V3: RecoveryParams = RecoveryParams {
    period_s: 1.6350900029811668,
    push_frac: 0.6826834924476096,
    ramp_s: 0.1708444572306406,
    hip_push: [0.8073424223849209, 0.3597124689100646],
    hip_rest: [-0.7380442201104547, -0.3669480987556565],
    thigh_push: -1.7090575629311473,
    calf_push: -1.2671453644922914,
    thigh_rest: -1.9627428232936004,
    calf_rest: -1.947691045672705,
    arm_push: -1.9269753184094436,
    arm_rest: -0.20938094408744395,
    handoff_up: 0.7377810245183191,
    finish_hip: 0.8124492456070739,
    finish_thigh: -2.1024606633420597,
    finish_calf: -0.649774683660324,
    finish_arm: 0.85,
    stand_hip: 0.7607914621396008,
    stand_thigh: 2.62,
    stand_calf: -0.11086124631030347,
};

/// Round 5, and the one in use. Searched against the teleop's drive law with
/// the search horizon raised to match validation, and scored only after the
/// robot has been handed back to a normal stance. Rights the robot in 14 of
/// the 15 validation conditions with a final trunk +z of 0.999 to 1.000 --
/// actually standing, not parked in a stable crouch.
///
/// The one failure is the red platform at friction 0.70, a condition held
/// out of training. It is a 5 cm plinth barely wider than the robot, and the
/// trajectory travels 0.4 to 0.9 m while righting, so most of what happens
/// there is a fall off the edge.
///
/// For scale: the best hand-written plan righted 1 of 15, and rounds 2 and 3
/// scored 15/15 and 13/15 against evaluators that turned out to be measuring
/// something else (see [`SEARCHED_V2`] and [`SEARCHED_V3`]).
pub const SEARCHED_V4: RecoveryParams = RecoveryParams {
    period_s: 1.190846481137671,
    push_frac: 0.7023220869469812,
    ramp_s: 0.12504386005728735,
    hip_push: [0.8918040573406579, -0.02653376421509206],
    hip_rest: [-0.551193407817909, -0.6406002214040931],
    thigh_push: -1.9066134657923735,
    calf_push: -0.8779970260362576,
    thigh_rest: -2.2664264201522055,
    calf_rest: -2.62,
    arm_push: -1.7250151106095313,
    arm_rest: -0.07762321822155842,
    handoff_up: 0.6595217954031017,
    finish_hip: 0.7013438997158615,
    finish_thigh: -2.11912022264595,
    finish_calf: -1.107535850753163,
    finish_arm: 0.85,
    stand_hip: 0.7591406978703054,
    stand_thigh: 2.593111185239448,
    stand_calf: 1.9850827369063018,
};

/// The trajectory the teleop `V` key runs. An alias so that re-running the
/// search and pointing this at a new constant is a one-line change.
pub const RECOVERY: RecoveryParams = SEARCHED_V4;

/// Number of searchable dimensions; see [`RecoveryParams::to_vec`].
pub const DIM: usize = 20;

/// Per-dimension search bounds, in the order [`RecoveryParams::to_vec`]
/// uses. Every one is a joint limit from the .misa or a duration that a
/// 3.3 kg body can plausibly act on, so no candidate is unphysical.
pub const BOUNDS: [(f64, f64); DIM] = [
    (0.20, 2.00),  // period_s
    (0.10, 0.90),  // push_frac
    (0.00, 0.40),  // ramp_s
    HIP_LIMIT_L,   // hip_push[0]
    HIP_LIMIT_R,   // hip_push[1]
    HIP_LIMIT_L,   // hip_rest[0]
    HIP_LIMIT_R,   // hip_rest[1]
    THIGH_LIMIT,   // thigh_push
    CALF_LIMIT,    // calf_push
    THIGH_LIMIT,   // thigh_rest
    CALF_LIMIT,    // calf_rest
    ARM_LIMIT,     // arm_push
    ARM_LIMIT,     // arm_rest
    (-0.20, 0.80), // handoff_up
    HIP_LIMIT_L,   // finish_hip
    THIGH_LIMIT,   // finish_thigh
    CALF_LIMIT,    // finish_calf
    HIP_LIMIT_L,   // stand_hip
    THIGH_LIMIT,   // stand_thigh
    CALF_LIMIT,    // stand_calf
];

impl RecoveryParams {
    pub fn to_vec(&self) -> [f64; DIM] {
        [
            self.period_s,
            self.push_frac,
            self.ramp_s,
            self.hip_push[0],
            self.hip_push[1],
            self.hip_rest[0],
            self.hip_rest[1],
            self.thigh_push,
            self.calf_push,
            self.thigh_rest,
            self.calf_rest,
            self.arm_push,
            self.arm_rest,
            self.handoff_up,
            self.finish_hip,
            self.finish_thigh,
            self.finish_calf,
            self.stand_hip,
            self.stand_thigh,
            self.stand_calf,
        ]
    }

    /// Inverse of [`to_vec`], clamping each element into [`BOUNDS`] so a
    /// search that samples outside them still yields a legal trajectory.
    /// `finish_arm` is not searched -- the probe showed the arm contributes
    /// nothing once the body is past vertical -- and is fixed at its high
    /// limit, out of the way.
    ///
    /// [`to_vec`]: RecoveryParams::to_vec
    pub fn from_vec(v: &[f64; DIM]) -> Self {
        let c = |i: usize| v[i].clamp(BOUNDS[i].0, BOUNDS[i].1);
        Self {
            period_s: c(0),
            push_frac: c(1),
            ramp_s: c(2),
            hip_push: [c(3), c(4)],
            hip_rest: [c(5), c(6)],
            thigh_push: c(7),
            calf_push: c(8),
            thigh_rest: c(9),
            calf_rest: c(10),
            arm_push: c(11),
            arm_rest: c(12),
            handoff_up: c(13),
            finish_hip: c(14),
            finish_thigh: c(15),
            finish_calf: c(16),
            finish_arm: ARM_LIMIT.1,
            stand_hip: c(17),
            stand_thigh: c(18),
            stand_calf: c(19),
        }
    }

    /// Joint targets at time `t` into the recovery.
    ///
    /// `up` is the trunk's own +z expressed in world coordinates (+1
    /// upright, -1 on its back) and `g_body_y` is gravity's y component in
    /// the trunk frame, whose sign says which side is underneath. Both are
    /// available from an attitude estimate alone.
    pub fn targets(&self, t: f64, up: f64, g_body_y: f64) -> (f64, LegTargets) {
        if up >= UPRIGHT_UP {
            let h = self.stand_hip;
            let l = [h.clamp(HIP_LIMIT_L.0, HIP_LIMIT_L.1), self.stand_thigh, self.stand_calf];
            let r = [(-h).clamp(HIP_LIMIT_R.0, HIP_LIMIT_R.1), self.stand_thigh, self.stand_calf];
            return (ARM_LIMIT.1, [l, r, l, r]);
        }
        if up >= self.handoff_up {
            let down_is_left = g_body_y > 0.0;
            let hip = self.finish_hip.abs();
            let push = if down_is_left {
                [hip.min(HIP_LIMIT_L.1), self.finish_thigh, self.finish_calf]
            } else {
                [(-hip).max(HIP_LIMIT_R.0), self.finish_thigh, self.finish_calf]
            };
            // Folded, so the top-side legs cannot prop the body up and stall
            // the roll short of standing.
            let fold = [0.0, 2.5, -2.6];
            let (l, r) = if down_is_left { (push, fold) } else { (fold, push) };
            return (self.finish_arm, [l, r, l, r]);
        }

        // Regime 1: rock. `s` is 0 at the rest pose, 1 at the push pose.
        let period = self.period_s.max(1e-3);
        let phase = t % period;
        let push_end = period * self.push_frac;
        let ramp = self.ramp_s.max(1e-4);
        let s = if phase < push_end {
            (phase / ramp).clamp(0.0, 1.0)
        } else {
            1.0 - ((phase - push_end) / ramp).clamp(0.0, 1.0)
        };
        let mix = |rest: f64, push: f64| rest + (push - rest) * s;
        let hip_l = mix(self.hip_rest[0], self.hip_push[0]);
        let hip_r = mix(self.hip_rest[1], self.hip_push[1]);
        let thigh = mix(self.thigh_rest, self.thigh_push);
        let calf = mix(self.calf_rest, self.calf_push);
        let arm = mix(self.arm_rest, self.arm_push);
        (
            arm,
            [
                [hip_l, thigh, calf],
                [hip_r, thigh, calf],
                [hip_l, thigh, calf],
                [hip_r, thigh, calf],
            ],
        )
    }
}

/// How the joints are driven during an evaluation.
///
/// Both exist because the two differ in ways that could each break the
/// recovery on their own, and only measuring says whether they do.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Drive {
    /// The .misa's own Position actuators -- kp 100 / kv 1.2 on the legs,
    /// kp 5 / kv 0.5 on the arm -- with attitude read from the simulator.
    /// What the search optimises against.
    Position,
    /// What the WBC teleop actually does: leg joints in torque mode with a
    /// host-side PD plus gravity compensation, and attitude from the
    /// Madgwick IMU filter instead of the simulator's pose. The arm stays on
    /// its Position actuator, as it does there.
    TorqueAhrs { kp: f64, kd: f64, imu_beta: f64 },
}

/// What one run of a recovery trajectory did.
#[derive(Clone, Copy, Debug)]
pub struct Outcome {
    /// Highest trunk +z reached. Separates "nearly" from "wrong actuator".
    pub peak_up: f64,
    /// Trunk +z at the end. This is the one that counts.
    pub final_up: f64,
    /// When it first passed [`UPRIGHT_UP`], or NaN.
    pub t_right_s: f64,
    /// How far it travelled in xy. Large values mean it slid rather than
    /// rolled, which on the ring usually means it fell off something.
    pub travel_m: f64,
}

impl Outcome {
    pub fn righted(&self) -> bool {
        self.final_up > 0.9
    }

    /// A single number to search on. Mostly the final attitude, with a
    /// smaller credit for the peak so that a trajectory which gets partway
    /// and falls back still scores above one that never moves -- without
    /// that gradient the search sees a flat landscape of failures.
    pub fn score(&self) -> f64 {
        self.final_up + 0.3 * self.peak_up + if self.righted() { 0.5 } else { 0.0 }
    }
}

#[cfg(feature = "mujoco")]
mod sim {
    use super::*;
    use crate::mjcf::{KawasakiRingCfg, MjcfExportOptions};
    use crate::mujoco_sim::MujocoSim;
    use crate::robot::RobotModel;

    /// Drop the robot on its back at `xy` on the ring and run `params`.
    ///
    /// Deterministic: MuJoCo's forward dynamics and this trajectory contain
    /// no RNG, so one evaluation per candidate is enough and a search need
    /// not average over seeds.
    pub fn evaluate(
        misa: &std::path::Path,
        ring: &KawasakiRingCfg,
        xy: (f64, f64),
        mu: f64,
        params: &RecoveryParams,
        horizon_s: f64,
    ) -> Outcome {
        evaluate_with(misa, ring, xy, mu, params, horizon_s, Drive::Position)
    }

    pub fn evaluate_with(
        misa: &std::path::Path,
        ring: &KawasakiRingCfg,
        xy: (f64, f64),
        mu: f64,
        params: &RecoveryParams,
        horizon_s: f64,
        drive: Drive,
    ) -> Outcome {
        let mut robot = RobotModel::from_misa(misa).expect("load robot");
        if let Drive::TorqueAhrs { .. } = drive {
            // Set before the sim is built, so the exported MJCF and the
            // control law agree -- the same ordering `run_wbc_sim` uses.
            for j in robot.joints.iter_mut() {
                if j.name.ends_with("_hip_joint")
                    || j.name.ends_with("_thigh_joint")
                    || j.name.ends_with("_calf_joint")
                {
                    j.actuator_mode = crate::rbd::model::ActuatorMode::Torque;
                }
            }
        }
        let opts = MjcfExportOptions {
            base_xy: Some(xy),
            extra_asset_xml: Some(ring.asset_xml("kawasaki")),
            extra_worldbody_xml: Some(ring.worldbody_xml("kawasaki")),
            add_actuators: true,
            ..MjcfExportOptions::default()
        };
        let mut sim = MujocoSim::new(&robot, opts).expect("MujocoSim::new");
        sim.set_hfield_data("kawasaki", &ring.heights()).expect("fill hfield");
        sim.set_slide_friction_all(mu);
        if let Drive::TorqueAhrs { .. } = drive {
            sim.set_gravity_compensation(true);
        }
        let dt = sim.timestep();
        let mut ahrs = match drive {
            Drive::TorqueAhrs { imu_beta, .. } => {
                Some(crate::attitude_estimator::MadgwickAhrs::new(imu_beta))
            }
            Drive::Position => None,
        };
        let root = robot.root_link.clone();

        sim.respawn_inverted(&mut robot, 0.03);
        // Settle the drop before driving anything, so the trajectory is
        // scored on its own motion and not on the landing.
        for _ in 0..(0.5 / dt) as u32 {
            sim.step(&mut robot, dt, true);
        }
        let start = sim.body_world_position(&root).unwrap();

        let arm_ji = robot.joint_map.get("arm_pitch_joint").copied();
        let leg_ji: Vec<[Option<usize>; 3]> = ["FL", "FR", "RL", "RR"]
            .iter()
            .map(|p| {
                [
                    robot.joint_map.get(&format!("{p}_hip_joint")).copied(),
                    robot.joint_map.get(&format!("{p}_thigh_joint")).copied(),
                    robot.joint_map.get(&format!("{p}_calf_joint")).copied(),
                ]
            })
            .collect();

        // Seconds of continuous upright attitude before the recovery is
        // declared over and the robot is handed back to a normal stance --
        // exactly what `run_wbc_sim` does on the `V` key. Scoring without
        // this let a trajectory win by parking in whatever crouch happened
        // to be stable: round 3 drove `stand_thigh` to its limit and settled
        // at a trunk +z of 0.920, which passes a 0.9 threshold while being a
        // robot sitting down.
        const HOLD_S: f64 = 1.0;
        let mut upright_s = 0.0_f64;
        let mut handed_back = false;

        let (mut peak_up, mut t_right_s, mut t) = (-1.0_f64, f64::NAN, 0.0);
        while t < horizon_s {
            // Attitude, from whichever source this drive is entitled to.
            let (up, g_body_y) = match &mut ahrs {
                Some(f) => {
                    if let Some(imu) = sim.imu_readings(&robot).first() {
                        f.update_imu(imu.gyro, imu.accel, dt);
                    }
                    let q = f.quaternion();
                    (
                        (q * nalgebra::Vector3::z()).z,
                        (q.inverse() * nalgebra::Vector3::new(0.0, 0.0, -1.0)).y,
                    )
                }
                None => {
                    let r = sim.body_world_orientation(&root).unwrap();
                    (
                        (r * nalgebra::Vector3::z()).z,
                        (r.inverse() * nalgebra::Vector3::new(0.0, 0.0, -1.0)).y,
                    )
                }
            };
            upright_s = if up > UPRIGHT_UP { upright_s + dt } else { 0.0 };
            handed_back |= upright_s > HOLD_S;
            let (arm, legs) = if handed_back {
                (ARM_LIMIT.1, [STANCE; 4])
            } else {
                params.targets(t, up, g_body_y)
            };
            // The arm is on its Position actuator either way.
            if let Some(ji) = arm_ji {
                sim.set_position_target(ji, arm);
            }
            match drive {
                Drive::Position => {
                    for (leg, targets) in leg_ji.iter().zip(legs.iter()) {
                        for (ji, &q) in leg.iter().zip(targets.iter()) {
                            if let Some(ji) = ji {
                                sim.set_position_target(*ji, q);
                            }
                        }
                    }
                }
                Drive::TorqueAhrs { kp, kd, .. } => {
                    let grav = sim.gravity_torques(&robot);
                    for (leg, targets) in leg_ji.iter().zip(legs.iter()) {
                        for (ji, &q_star) in leg.iter().zip(targets.iter()) {
                            let Some(ji) = ji else { continue };
                            let (q, qd) = sim
                                .joint_q_qd(&robot.joints[*ji].name)
                                .unwrap_or((q_star, 0.0));
                            let tau = kp * (q_star - q) - kd * qd
                                + grav.get(*ji).copied().unwrap_or(0.0);
                            sim.set_torque_target(*ji, tau);
                        }
                    }
                }
            }
            sim.step(&mut robot, dt, true);
            t += dt;
            let up = (sim.body_world_orientation(&root).unwrap() * nalgebra::Vector3::z()).z;
            peak_up = peak_up.max(up);
            if up > UPRIGHT_UP && t_right_s.is_nan() {
                t_right_s = t;
            }
        }
        let final_up = (sim.body_world_orientation(&root).unwrap() * nalgebra::Vector3::z()).z;
        let end = sim.body_world_position(&root).unwrap();
        Outcome {
            peak_up,
            final_up,
            t_right_s,
            travel_m: ((end[0] - start[0]).powi(2) + (end[1] - start[1]).powi(2)).sqrt(),
        }
    }
}

#[cfg(feature = "mujoco")]
pub use sim::{evaluate, evaluate_with};
