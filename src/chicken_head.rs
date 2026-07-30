//! **ChickenHead** — world-frame attitude hold for a single non-leg head /
//! arm / payload joint, à la the classic robot-arm "chicken head" demo where
//! the end platform stays put in space while the body underneath moves.
//!
//! ## What it does
//!
//! A quadruped with a body-mounted 1-DoF head joint (namiashi's
//! `arm_pitch_joint`, a `+Y`/pitch revolute) can keep that head **level in
//! the world** even while the trunk pitches — during a Bound gait, a stumble,
//! or any body attitude excursion. The head therefore behaves like a chicken
//! stabilising its head against body sway: a steady camera / payload platform.
//!
//! ## The law (1-DoF, kinematic)
//!
//! The head link's world attitude about the stabilised axis is, to first
//! order, the sum of the trunk's attitude and the joint angle:
//!
//! ```text
//! θ_head_world  ≈  θ_trunk  +  sign · q_joint
//! ```
//!
//! where `sign = +1` when the joint's URDF axis points along the positive
//! stabilised world axis and `-1` when it points the other way. Holding
//! `θ_head_world` at a fixed reference `θ_ref` therefore just needs
//!
//! ```text
//! q*_joint  =  sign · (θ_ref − θ_trunk)              (clamped to joint limits)
//! q̇*_joint  =  −sign · θ̇_trunk    ≈  −sign · ω_body[axis]   (feed-forward)
//! ```
//!
//! This is a **pure kinematic reference** — no dynamics, no model. That makes
//! it usable from either control path:
//!
//! * **Position-PD path** — the host commands `q*` (and optionally `q̇*`)
//!   straight to the head actuator via
//!   [`crate::mujoco_sim::MujocoSim::set_position_target`].
//! * **WBC torque path** — the reference becomes a joint-acceleration task
//!   `q̈* = kp·(q*−q) + kd·(q̇*−q̇)` fed through the WBC's existing
//!   per-actuator swing-task channel (`swing_q_ddot_des` /
//!   `swing_actuator_flag`), so it composes with a running WBC gait without
//!   touching the solver.
//!
//! The Euler-rate ≈ body-rate substitution in `q̇*` is exact only when the two
//! *other* Euler angles are zero; away from that the residual is mopped up by
//! the `kp` position feedback (WBC path) or the actuator PD (position path),
//! so it is a benign feed-forward approximation, not a modelling error.

use nalgebra as na;

use crate::rbd::model::RobotModel;

/// Which body attitude axis the head joint stabilises. namiashi's
/// `arm_pitch_joint` is [`StabAxis::Pitch`]; the roll / yaw variants are
/// provided for other robots whose head joint spins about a different axis.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StabAxis {
    /// Roll — body X. Stabilised against `euler.0` / `ω_body.x`.
    Roll,
    /// Pitch — body Y (namiashi's arm). Stabilised against `euler.1` / `ω_body.y`.
    Pitch,
    /// Yaw — body Z. Stabilised against `euler.2` / `ω_body.z`.
    Yaw,
}

/// ChickenHead configuration + tuning. Held by the host (GUI panel / test)
/// and pushed onto the [`crate::wbc_pipeline::WbcPipeline`] each tick, or read
/// directly to command the position-PD actuator.
///
/// Construct with [`ChickenHeadConfig::for_joint`] to auto-resolve the joint
/// index, axis sign, and limits from a [`RobotModel`]; the resulting config
/// starts **disabled** so merely building it never commands motion.
#[derive(Debug, Clone)]
pub struct ChickenHeadConfig {
    /// Master enable. `false` ⇒ every consumer is an exact no-op.
    pub enabled: bool,
    /// Name of the head joint (e.g. `"arm_pitch_joint"`). Used by consumers
    /// to resolve the actuator / misarta index each tick.
    pub joint_name: String,
    /// `RobotModel::joints` index of the head joint.
    pub joint_idx: usize,
    /// Body attitude axis this joint holds.
    pub axis: StabAxis,
    /// `+1.0` / `-1.0` — sign relating a positive joint rotation to a positive
    /// rotation about the stabilised world axis (from the URDF joint axis).
    pub axis_sign: f64,
    /// World-frame attitude the head is held at, in rad. `0.0` = level
    /// (the classic chicken-head). Positive tips the head up/nose-down per the
    /// stabilised axis' right-hand rule.
    pub target_world_angle: f64,
    /// Joint lower / upper limits (rad) the reference is clamped to.
    pub limit_lo: f64,
    pub limit_hi: f64,
    /// WBC joint-acceleration task gains (units 1/s² and 1/s), applied to the
    /// position / velocity error. Ignored by the position-PD path (which uses
    /// the actuator's own PD). Defaults chosen to match the WBC swing gains'
    /// scale.
    pub kp: f64,
    pub kd: f64,
}

impl ChickenHeadConfig {
    /// Resolve a ChickenHead config for `joint_name` on `robot`, reading the
    /// joint's axis sign (for the given [`StabAxis`]) and position limits from
    /// the model. Returns `None` if the joint name isn't in the model.
    ///
    /// The config starts **disabled** — the caller flips `enabled` when the
    /// feature is switched on, so constructing it is side-effect free.
    pub fn for_joint(robot: &RobotModel, joint_name: &str, axis: StabAxis) -> Option<Self> {
        let &idx = robot.joint_map.get(joint_name)?;
        let joint = &robot.joints[idx];
        // The relevant axis component picks how a +q rotation maps onto the
        // world attitude axis. A near-zero component means the joint doesn't
        // actually rotate about the stabilised axis; we default sign to +1 and
        // let the caller notice via a degenerate hold.
        let comp = match axis {
            StabAxis::Roll => joint.axis.x,
            StabAxis::Pitch => joint.axis.y,
            StabAxis::Yaw => joint.axis.z,
        } as f64;
        let axis_sign = if comp >= 0.0 { 1.0 } else { -1.0 };
        Some(Self {
            enabled: false,
            joint_name: joint_name.to_string(),
            joint_idx: idx,
            axis,
            axis_sign,
            target_world_angle: 0.0,
            limit_lo: joint.lower,
            limit_hi: joint.upper,
            kp: 100.0,
            kd: 10.0,
        })
    }

    /// Extract the stabilised-axis component of a body attitude, in rad.
    #[inline]
    fn body_angle(&self, trunk_quat: &na::UnitQuaternion<f64>) -> f64 {
        let (roll, pitch, yaw) = trunk_quat.euler_angles();
        match self.axis {
            StabAxis::Roll => roll,
            StabAxis::Pitch => pitch,
            StabAxis::Yaw => yaw,
        }
    }

    /// Extract the stabilised-axis component of a body-frame angular rate.
    #[inline]
    fn body_rate(&self, omega_body: &na::Vector3<f64>) -> f64 {
        match self.axis {
            StabAxis::Roll => omega_body.x,
            StabAxis::Pitch => omega_body.y,
            StabAxis::Yaw => omega_body.z,
        }
    }

    /// Joint-angle reference `q* = sign·(θ_ref − θ_trunk)`, clamped to the
    /// joint limits. `trunk_quat` is the body's world orientation.
    pub fn target_angle(&self, trunk_quat: &na::UnitQuaternion<f64>) -> f64 {
        let q = self.axis_sign * (self.target_world_angle - self.body_angle(trunk_quat));
        q.clamp(self.limit_lo, self.limit_hi)
    }

    /// Joint-velocity feed-forward `q̇* = −sign·θ̇_trunk ≈ −sign·ω_body[axis]`.
    /// `omega_body` is the body-frame angular velocity. Ignores clamping (a
    /// feed-forward term only); at a limit the position error still drives the
    /// joint back inside via `kp`.
    pub fn target_velocity(&self, omega_body: &na::Vector3<f64>) -> f64 {
        -self.axis_sign * self.body_rate(omega_body)
    }

    /// WBC joint-acceleration task value
    /// `q̈* = kp·(q*−q) + kd·(q̇*−q̇)` for the head actuator, given the current
    /// measured joint state and the body attitude / rate.
    pub fn target_accel(
        &self,
        trunk_quat: &na::UnitQuaternion<f64>,
        omega_body: &na::Vector3<f64>,
        q_meas: f64,
        qd_meas: f64,
    ) -> f64 {
        let q_ref = self.target_angle(trunk_quat);
        let qd_ref = self.target_velocity(omega_body);
        self.kp * (q_ref - q_meas) + self.kd * (qd_ref - qd_meas)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    fn cfg_pitch() -> ChickenHeadConfig {
        ChickenHeadConfig {
            enabled: true,
            joint_name: "arm_pitch_joint".into(),
            joint_idx: 0,
            axis: StabAxis::Pitch,
            axis_sign: 1.0,
            target_world_angle: 0.0,
            limit_lo: -2.3,
            limit_hi: 0.85,
            kp: 100.0,
            kd: 10.0,
        }
    }

    fn pitch_quat(pitch: f64) -> na::UnitQuaternion<f64> {
        na::UnitQuaternion::from_euler_angles(0.0, pitch, 0.0)
    }

    #[test]
    fn level_body_holds_at_target() {
        let cfg = cfg_pitch();
        // Body level, target level ⇒ joint stays at 0.
        assert!((cfg.target_angle(&pitch_quat(0.0)) - 0.0).abs() < 1e-9);
    }

    #[test]
    fn compensates_body_pitch_one_for_one() {
        let cfg = cfg_pitch();
        // Trunk pitches +0.3 rad; to keep the head level the joint must
        // rotate -0.3 (sign +1 · (0 − 0.3)).
        let q = cfg.target_angle(&pitch_quat(0.3));
        assert!((q - (-0.3)).abs() < 1e-9, "q={q}");
        // And the head's world pitch = trunk + sign·q = 0.3 + (-0.3) = 0.
        let head_world = 0.3 + cfg.axis_sign * q;
        assert!(head_world.abs() < 1e-9);
    }

    #[test]
    fn nonzero_target_offsets_the_hold() {
        let mut cfg = cfg_pitch();
        cfg.target_world_angle = 0.2;
        // Head held at +0.2 world while trunk at -0.1 ⇒ q = 0.2 − (−0.1) = 0.3.
        let q = cfg.target_angle(&pitch_quat(-0.1));
        assert!((q - 0.3).abs() < 1e-9, "q={q}");
    }

    #[test]
    fn clamps_to_joint_limits() {
        let mut cfg = cfg_pitch();
        // A far-above-limit target world angle demands q beyond the upper
        // limit; the reference must clamp. (Trunk pitch alone can't exceed
        // ±π/2 as a pure Euler pitch, so we drive the clamp via the target.)
        cfg.target_world_angle = 5.0;
        let q = cfg.target_angle(&pitch_quat(0.0));
        assert!((q - cfg.limit_hi).abs() < 1e-9, "q={q} should clamp to hi");
        // And a far-below target clamps to the lower limit.
        cfg.target_world_angle = -5.0;
        let q = cfg.target_angle(&pitch_quat(0.0));
        assert!((q - cfg.limit_lo).abs() < 1e-9, "q={q} should clamp to lo");
    }

    #[test]
    fn negative_axis_sign_flips_command() {
        let mut cfg = cfg_pitch();
        cfg.axis_sign = -1.0;
        // With a -Y joint axis, holding level under +0.3 trunk pitch needs
        // q = -1·(0 − 0.3) = +0.3.
        let q = cfg.target_angle(&pitch_quat(0.3));
        assert!((q - 0.3).abs() < 1e-9, "q={q}");
    }

    #[test]
    fn velocity_feedforward_opposes_body_rate() {
        let cfg = cfg_pitch();
        let omega = na::Vector3::new(0.0, 0.5, 0.0); // pitching up at 0.5 rad/s
        // Head must counter-rotate: q̇* = -sign·ω_y = -0.5.
        assert!((cfg.target_velocity(&omega) - (-0.5)).abs() < 1e-9);
    }

    #[test]
    fn accel_task_drives_toward_reference() {
        let cfg = cfg_pitch();
        // Trunk pitched +0.3 (ref q* = -0.3), joint currently at 0, still ⇒
        // q̈* = kp·(-0.3 - 0) + kd·(0 - 0) = -30.
        let a = cfg.target_accel(&pitch_quat(0.3), &na::Vector3::zeros(), 0.0, 0.0);
        assert!((a - (-30.0)).abs() < 1e-6, "a={a}");
    }

    #[test]
    fn yaw_axis_wraps_are_left_to_caller() {
        // Sanity: yaw stabilisation reads euler.2 and produces a finite q.
        let mut cfg = cfg_pitch();
        cfg.axis = StabAxis::Yaw;
        cfg.limit_lo = -PI;
        cfg.limit_hi = PI;
        let q = cfg.target_angle(&na::UnitQuaternion::from_euler_angles(0.0, 0.0, 0.4));
        assert!((q - (-0.4)).abs() < 1e-9, "q={q}");
    }
}
