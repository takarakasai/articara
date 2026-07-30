//! **Standing gestures** — small, cute, rhythmic idle motions the quadruped
//! performs while standing in place (all four feet in stance): a head *nod*,
//! a gentle body *sway*, and room to grow. Think of the little idle
//! animations a pet robot plays when it's just standing there being adorable.
//!
//! ## How it drives the robot
//!
//! Each gesture is a parametric **oscillator** — a sine wave with an
//! amplitude and a frequency — that emits, per tick, a small offset on one of
//! two channels:
//!
//! * **Head channel** — an offset (rad) on the head joint (namiashi's
//!   `arm_pitch_joint`), applied via
//!   [`crate::mujoco_sim::MujocoSim::set_position_target`]. On robots with no
//!   head joint the same gesture maps onto the body-pitch channel instead, so
//!   a `Nod` still reads as a nod.
//! * **Body-attitude channel** — offsets (rad) on the trunk roll / pitch /
//!   yaw references the WBC already tracks
//!   ([`crate::wbc_pipeline::WbcPipeline`]'s `roll_ref` / `pitch_ref` /
//!   `yaw_ref` + `*_pd_gain`). This channel is only effective when the
//!   Hierarchical WBC is running (MPC-family gait mode); with WBC off, a
//!   body gesture is simply inert and only head gestures show.
//!
//! Because the output is a pure kinematic offset the module is stateless
//! aside from the elapsed-time phase the host feeds in — the same design as
//! [`crate::chicken_head`]. When a head gesture runs alongside ChickenHead,
//! the host composes them: the gesture offset rides *on top of* the
//! ChickenHead world-level hold, so the head bobs around level.

use crate::rbd::model::RobotModel;

/// The kind of idle gesture. Small on purpose — extend as the palette grows.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GestureKind {
    /// A head *nod* — a pitch bob. Uses the head joint when present, else the
    /// body-pitch reference.
    Nod,
    /// A gentle body *sway* — a slow trunk roll rocking, driven through the
    /// WBC roll reference.
    Sway,
}

impl GestureKind {
    /// All kinds, for UI pickers.
    pub const ALL: [GestureKind; 2] = [GestureKind::Nod, GestureKind::Sway];

    /// Human-readable label.
    pub fn label(self) -> &'static str {
        match self {
            GestureKind::Nod => "Nod (頷き)",
            GestureKind::Sway => "Sway (ゆらゆら)",
        }
    }

    /// Whether this gesture drives the body-attitude channel (⇒ needs the WBC
    /// running to be visible).
    pub fn uses_body_attitude(self, has_head: bool) -> bool {
        match self {
            // Nod uses the head joint if there is one; otherwise it falls back
            // to the body-pitch channel.
            GestureKind::Nod => !has_head,
            GestureKind::Sway => true,
        }
    }
}

/// Per-tick output of a gesture: a head-joint offset (already in joint space,
/// `None` if this gesture doesn't move the head) plus body roll/pitch/yaw
/// offsets (rad, all zero if this gesture doesn't move the body).
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct GestureOutput {
    pub head_offset: Option<f64>,
    pub roll: f64,
    pub pitch: f64,
    pub yaw: f64,
}

/// A configured standing gesture. Build with [`StandingGestureConfig::for_robot`]
/// so the head joint (if any) is resolved from the model, then flip `enabled`
/// and tune `amplitude_rad` / `frequency_hz` from the host.
#[derive(Debug, Clone)]
pub struct StandingGestureConfig {
    /// Master enable. `false` ⇒ [`Self::sample`] returns a zero output and
    /// every consumer is a no-op.
    pub enabled: bool,
    /// Which gesture to play.
    pub kind: GestureKind,
    /// Peak offset of the oscillation (rad).
    pub amplitude_rad: f64,
    /// Oscillation frequency (Hz).
    pub frequency_hz: f64,
    /// `RobotModel::joints` index of the head joint, if the model has one.
    pub head_joint_idx: Option<usize>,
    /// Sign relating a positive world-pitch offset to a positive head-joint
    /// rotation (from the URDF axis), same convention as
    /// [`crate::chicken_head::ChickenHeadConfig`].
    pub head_axis_sign: f64,
    /// Head-joint position limits (rad) the composed target is clamped to.
    pub head_limits: (f64, f64),
    /// WBC attitude PD gains (1/s², 1/s) applied while a body-attitude gesture
    /// is active, so the WBC actually tracks the oscillating reference.
    pub attitude_pd_gain: (f64, f64),
}

impl StandingGestureConfig {
    /// Resolve a gesture config for `robot`, auto-detecting the head joint
    /// (`arm_pitch_joint`) if present. Starts **disabled** — building it is
    /// side-effect free. Missing head joint ⇒ `head_joint_idx = None`, and a
    /// [`GestureKind::Nod`] then drives the body-pitch channel instead.
    pub fn for_robot(robot: &RobotModel, kind: GestureKind) -> Self {
        let (head_joint_idx, head_axis_sign, head_limits) =
            match robot.joint_map.get("arm_pitch_joint") {
                Some(&idx) => {
                    let j = &robot.joints[idx];
                    let sign = if j.axis.y >= 0.0 { 1.0 } else { -1.0 };
                    (Some(idx), sign, (j.lower, j.upper))
                }
                None => (None, 1.0, (f64::NEG_INFINITY, f64::INFINITY)),
            };
        Self {
            enabled: false,
            kind,
            // ~9° head bob / body sway — visible but gentle.
            amplitude_rad: 0.15,
            frequency_hz: 0.5,
            head_joint_idx,
            head_axis_sign,
            head_limits,
            attitude_pd_gain: (150.0, 15.0),
        }
    }

    /// Whether this gesture, on this robot, drives the body-attitude channel.
    pub fn uses_body_attitude(&self) -> bool {
        self.kind.uses_body_attitude(self.head_joint_idx.is_some())
    }

    /// Sample the oscillator at elapsed time `t` (seconds). Returns a zero
    /// output when disabled.
    pub fn sample(&self, t: f64) -> GestureOutput {
        if !self.enabled {
            return GestureOutput::default();
        }
        let s = self.amplitude_rad * (2.0 * std::f64::consts::PI * self.frequency_hz * t).sin();
        let has_head = self.head_joint_idx.is_some();
        match self.kind {
            GestureKind::Nod => {
                if has_head {
                    // Head-joint bob. Convert the world-pitch offset to joint
                    // space via the axis sign (matches ChickenHead).
                    GestureOutput {
                        head_offset: Some(self.head_axis_sign * s),
                        ..Default::default()
                    }
                } else {
                    // No head — nod the whole body in pitch instead.
                    GestureOutput { pitch: s, ..Default::default() }
                }
            }
            GestureKind::Sway => GestureOutput { roll: s, ..Default::default() },
        }
    }

    /// Compose the head-joint target: the gesture's head offset added on top
    /// of a `base` angle (the ChickenHead world-level hold when active, else
    /// the head's neutral), clamped to the joint limits. Returns `None` when
    /// this gesture doesn't move the head.
    pub fn head_target(&self, t: f64, base: f64) -> Option<f64> {
        self.sample(t)
            .head_offset
            .map(|off| (base + off).clamp(self.head_limits.0, self.head_limits.1))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn nod_with_head() -> StandingGestureConfig {
        StandingGestureConfig {
            enabled: true,
            kind: GestureKind::Nod,
            amplitude_rad: 0.15,
            frequency_hz: 0.5,
            head_joint_idx: Some(0),
            head_axis_sign: 1.0,
            head_limits: (-2.3, 0.85),
            attitude_pd_gain: (150.0, 15.0),
        }
    }

    #[test]
    fn disabled_is_zero() {
        let mut g = nod_with_head();
        g.enabled = false;
        assert_eq!(g.sample(0.3), GestureOutput::default());
    }

    #[test]
    fn oscillation_starts_at_zero_and_peaks_at_quarter_period() {
        let g = nod_with_head();
        // sin(0) = 0.
        assert!((g.sample(0.0).head_offset.unwrap()).abs() < 1e-12);
        // Quarter period (t = 1/(4f)) ⇒ sin(π/2) = 1 ⇒ full amplitude.
        let t_peak = 1.0 / (4.0 * g.frequency_hz);
        let peak = g.sample(t_peak).head_offset.unwrap();
        assert!((peak - g.amplitude_rad).abs() < 1e-9, "peak={peak}");
    }

    #[test]
    fn nod_with_head_uses_head_channel_only() {
        let out = nod_with_head().sample(0.1);
        assert!(out.head_offset.is_some());
        assert_eq!((out.roll, out.pitch, out.yaw), (0.0, 0.0, 0.0));
    }

    #[test]
    fn nod_without_head_falls_back_to_body_pitch() {
        let mut g = nod_with_head();
        g.head_joint_idx = None;
        let t_peak = 1.0 / (4.0 * g.frequency_hz);
        let out = g.sample(t_peak);
        assert!(out.head_offset.is_none());
        assert!((out.pitch - g.amplitude_rad).abs() < 1e-9);
    }

    #[test]
    fn sway_uses_body_roll() {
        let mut g = nod_with_head();
        g.kind = GestureKind::Sway;
        let t_peak = 1.0 / (4.0 * g.frequency_hz);
        let out = g.sample(t_peak);
        assert!(out.head_offset.is_none());
        assert!((out.roll - g.amplitude_rad).abs() < 1e-9);
        assert!(g.uses_body_attitude());
    }

    #[test]
    fn axis_sign_flips_head_direction() {
        let mut g = nod_with_head();
        g.head_axis_sign = -1.0;
        let t_peak = 1.0 / (4.0 * g.frequency_hz);
        let off = g.sample(t_peak).head_offset.unwrap();
        assert!((off + g.amplitude_rad).abs() < 1e-9, "off={off}");
    }

    #[test]
    fn head_target_composes_and_clamps() {
        let g = nod_with_head();
        let t_peak = 1.0 / (4.0 * g.frequency_hz);
        // base 0.3 + amplitude 0.15 = 0.45, inside limits.
        let q = g.head_target(t_peak, 0.3).unwrap();
        assert!((q - 0.45).abs() < 1e-9, "q={q}");
        // A base beyond the upper limit clamps.
        let q = g.head_target(t_peak, 5.0).unwrap();
        assert!((q - g.head_limits.1).abs() < 1e-9);
    }
}
