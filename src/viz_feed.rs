//! Live gait **viewer**: subscribe to Zenoh streams of
//! [`quadruped_gait::viz::GaitVizFrame`] (published by `go2-gait-runner --viz`)
//! and drive the loaded model's joints + trunk pose so the gait the runner
//! generates can be watched in real time.
//!
//! Two streams are consumed, on separate keys:
//!
//! - **target** (`go2/gait/planned`) — the commanded joint angles the runner
//!   sends to the robot;
//! - **current** (`go2/gait/measured`) — the angles read back from the robot
//!   (`LowState`), published only by a hardware run.
//!
//! When both are up, the *current* pose drives the model solidly and the
//! *target* pose is superimposed as a translucent ghost (the renderer's
//! `ghost_transforms` pass), so command vs. response is visible at a glance.
//! With only the target stream (an offline `--viz` run) it drives the model
//! itself, as before.
//!
//! The transport lives in [`quadruped_gait::viz_sub`] (Zenoh, pure Rust —
//! no DDS/C toolchain); this module only applies received frames to the
//! loaded model each repaint, via the same path as kinematic playback
//! (`model.joint_positions` + `model.base_transform`). Feature-gated (`viz`).

use std::collections::HashMap;

use nalgebra as na;
use quadruped_gait::viz::GaitVizFrame;
use quadruped_gait::viz_sub::VizSubscriber;

use crate::robot::RobotModel;

/// Go2 joint names per `GaitVizFrame` slot (FL, FR, RL, RR) × (hip, thigh,
/// calf). The frame carries angles in this fixed order; we map them onto the
/// loaded model by name (skipping any the model doesn't have).
const VIZ_JOINT_NAMES: [[&str; 3]; 4] = [
    ["FL_hip_joint", "FL_thigh_joint", "FL_calf_joint"],
    ["FR_hip_joint", "FR_thigh_joint", "FR_calf_joint"],
    ["RL_hip_joint", "RL_thigh_joint", "RL_calf_joint"],
    ["RR_hip_joint", "RR_thigh_joint", "RR_calf_joint"],
];

/// Default Zenoh key for the **measured** (robot read-back) stream — the
/// counterpart to [`quadruped_gait::viz::VIZ_KEY_PLANNED`], published by
/// `go2-gait-runner` on a hardware run.
pub const VIZ_KEY_MEASURED: &str = "go2/gait/measured";

/// GUI-side state for the live gait feed. Holds the (optional) running
/// subscribers and the editable Zenoh keys.
pub struct VizFeedState {
    /// Target (commanded) stream — the ghost when a measured stream is up.
    sub: Option<VizSubscriber>,
    /// Current (measured) stream; empty [`Self::key_measured`] = not subscribed.
    sub_meas: Option<VizSubscriber>,
    /// Zenoh key of the target stream (editable while stopped).
    pub key: String,
    /// Zenoh key of the measured stream (editable while stopped); empty =
    /// target-only, and the target then drives the model solidly.
    pub key_measured: String,
    /// Optional Zenoh **connect** endpoint (e.g. `tcp/127.0.0.1:7447`) for hosts
    /// without multicast; connects to the publisher's `--viz-endpoint`. Empty =
    /// auto multicast discovery.
    pub endpoint: String,
    /// Connect endpoint for the measured stream when it comes from a *different*
    /// publisher than the target (a separate state bridge, a replay, or a
    /// sim-vs-robot comparison across two hosts). Empty = same as
    /// [`Self::endpoint`], which is the usual one-runner case. Each subscriber
    /// opens its own Zenoh session, so the two are independent.
    pub endpoint_measured: String,
    /// Sequence number of the last target frame (for a status read-out).
    pub last_seq: Option<u64>,
    /// Sequence number of the last measured frame.
    pub last_seq_measured: Option<u64>,
    /// Superimpose the target pose as a translucent ghost over the measured one.
    pub overlay_target: bool,
    /// Alpha of that ghost (0 = invisible, 1 = opaque).
    pub ghost_alpha: f32,
    /// Latest target frame, kept between repaints so the ghost persists even on
    /// repaints where no new frame arrived.
    target: Option<GaitVizFrame>,
    /// Whether a measured frame has ever arrived — decides which stream drives
    /// the solid model.
    got_measured: bool,
}

impl Default for VizFeedState {
    fn default() -> Self {
        Self {
            sub: None,
            sub_meas: None,
            key: quadruped_gait::viz::VIZ_KEY_PLANNED.to_string(),
            key_measured: VIZ_KEY_MEASURED.to_string(),
            endpoint: String::new(),
            endpoint_measured: String::new(),
            last_seq: None,
            last_seq_measured: None,
            overlay_target: true,
            ghost_alpha: 0.35,
            target: None,
            got_measured: false,
        }
    }
}

impl VizFeedState {
    /// Whether the subscribers are currently running.
    pub fn active(&self) -> bool {
        self.sub.is_some() || self.sub_meas.is_some()
    }

    /// Whether a measured stream is driving the model (so the target is drawn
    /// as a ghost rather than driving the model itself).
    pub fn has_measured(&self) -> bool {
        self.got_measured
    }

    /// Whether the measured subscriber is actually running (its key was set
    /// and not in conflict with the target's).
    pub fn measured_subscribed(&self) -> bool {
        self.sub_meas.is_some()
    }

    /// Whether both keys name the **same** stream — a misconfiguration: the
    /// two subscribers would receive the same frames, so the ghost would
    /// coincide exactly with the solid model (invisible) and the pose would
    /// flip between whichever sample landed last. The measured subscriber is
    /// skipped in that case.
    pub fn measured_key_conflicts(&self) -> bool {
        let m = self.key_measured.trim();
        !m.is_empty() && m == self.key.trim()
    }

    /// Start the subscribers (if stopped) or stop them (if running).
    pub fn toggle(&mut self) {
        if self.active() {
            self.sub = None;
            self.sub_meas = None;
            self.last_seq = None;
            self.last_seq_measured = None;
            self.target = None;
            self.got_measured = false;
        } else {
            let ep = non_empty(&self.endpoint);
            // The measured stream may come from another publisher entirely, so
            // it gets its own endpoint; unset falls back to the target's.
            let ep_meas = non_empty(&self.endpoint_measured).or(ep);
            // Either key may be left empty: measured-only (watch the robot
            // alone) and target-only (offline `--viz`, no read-back) are both
            // valid — an empty key is skipped rather than subscribed to.
            if !self.key.trim().is_empty() {
                match VizSubscriber::new(self.key.trim(), ep) {
                    Ok(s) => self.sub = Some(s),
                    Err(e) => eprintln!("viz-feed: {e}"),
                }
            }
            if self.measured_key_conflicts() {
                eprintln!(
                    "viz-feed: measured key '{}' is the same stream as the \
                     target's — measured not subscribed (the two must \
                     be distinct keys)",
                    self.key_measured.trim(),
                );
            } else if !self.key_measured.trim().is_empty() {
                match VizSubscriber::new(self.key_measured.trim(), ep_meas) {
                    Ok(s) => self.sub_meas = Some(s),
                    Err(e) => eprintln!("viz-feed (measured): {e}"),
                }
            }
        }
    }

    /// Apply the latest received frames to `model` (joint angles + trunk pose),
    /// rebuilding the render model. The measured frame drives the model once
    /// one has arrived; until then the target frame does. Returns `true` if a
    /// frame was applied.
    ///
    /// Either stream alone is fine: target-only drives the model with the
    /// target and draws no ghost, measured-only drives it with the measured
    /// and likewise has nothing to ghost.
    ///
    /// A measured dropout **latches**: "a measured frame has arrived" is
    /// sticky, so the model holds its last measured pose and simply stops
    /// updating (the ghost keeps moving, which is what makes the dropout
    /// visible) — it never snaps back to the target, which would hide it.
    /// When the stream resumes it picks up at the newest frame:
    /// [`VizSubscriber::take_latest`] keeps only the most recent sample, so
    /// there is no backlog to replay.
    pub fn apply(&mut self, model: &mut RobotModel) -> bool {
        let mut new_target = false;
        if let Some(sub) = &self.sub {
            if let Some(frame) = sub.take_latest() {
                self.last_seq = Some(frame.seq);
                self.target = Some(frame);
                new_target = true;
            }
        }
        let mut measured = None;
        if let Some(sub) = &self.sub_meas {
            if let Some(frame) = sub.take_latest() {
                self.last_seq_measured = Some(frame.seq);
                self.got_measured = true;
                measured = Some(frame);
            }
        }
        // Whoever drives the solid model: the measured frame when the robot is
        // reporting back, otherwise the target (offline `--viz`, no read-back).
        // Only a *newly arrived* frame re-poses the model — repaints in between
        // must not report a change (the caller re-uploads the model on `true`).
        let solid = match (&measured, self.got_measured, new_target) {
            (Some(f), ..) => Some(f),
            (None, false, true) => self.target.as_ref(),
            _ => None,
        };
        let Some(frame) = solid else {
            return false;
        };
        set_pose(model, frame);
        model.rebuild_misarta_model();
        true
    }

    /// Link transforms for the translucent target ghost, or `None` when the
    /// overlay is off / there is nothing to overlay (no target frame yet, or
    /// no measured stream — the target would coincide with the solid model).
    ///
    /// Computed by posing `model` at the target frame, running FK, and putting
    /// the model's own pose straight back — the model is left untouched.
    pub fn ghost_transforms(
        &self,
        model: &mut RobotModel,
    ) -> Option<HashMap<String, na::Isometry3<f32>>> {
        if !self.overlay_target || !self.got_measured {
            return None;
        }
        let frame = self.target.as_ref()?;
        let saved_q = model.joint_positions.clone();
        let saved_base = model.base_transform;
        set_pose(model, frame);
        let transforms = model.compute_transforms();
        model.joint_positions = saved_q;
        model.base_transform = saved_base;
        Some(transforms)
    }
}

/// A trimmed setting, or `None` when it is blank.
fn non_empty(s: &str) -> Option<&str> {
    Some(s.trim()).filter(|s| !s.is_empty())
}

/// Write one frame's joint angles and trunk pose into `model` (no rebuild —
/// `compute_transforms` reads `joint_positions` live).
fn set_pose(model: &mut RobotModel, frame: &GaitVizFrame) {
    for slot in 0..4 {
        for k in 0..3 {
            if let Some(&idx) = model.joint_map.get(VIZ_JOINT_NAMES[slot][k]) {
                if let Some(p) = model.joint_positions.get_mut(idx) {
                    *p = frame.joints[3 * slot + k];
                }
            }
        }
    }
    let p = frame.pose;
    model.base_transform = na::Isometry3::from_parts(
        na::Translation3::new(p[0], p[1], p[2]),
        na::UnitQuaternion::from_euler_angles(0.0, 0.0, p[3]),
    );
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::path::PathBuf;

    fn go2_model() -> RobotModel {
        let path = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("models/unitree_go2/go2.misa");
        RobotModel::from_misa(&path).expect("load go2.misa")
    }

    fn frame(seq: u64, calf: f64) -> GaitVizFrame {
        let mut joints = [0.0f64; 12];
        for slot in 0..4 {
            joints[3 * slot + 1] = 0.9;
            joints[3 * slot + 2] = calf;
        }
        GaitVizFrame {
            version: quadruped_gait::viz::VIZ_FORMAT_VERSION,
            seq,
            t_s: 0.0,
            pose: [0.0, 0.0, 0.3, 0.0],
            joints,
            stance: [true; 4],
        }
    }

    /// The ghost is the *target* pose: it must differ from the model's own
    /// (measured) pose and must leave the model exactly as it found it — the
    /// solid render still comes from `model`.
    #[test]
    fn ghost_transforms_pose_the_target_without_touching_the_model() {
        let mut model = go2_model();
        let mut viz = VizFeedState {
            target: Some(frame(1, -1.2)),
            got_measured: true,
            ..Default::default()
        };
        set_pose(&mut model, &frame(1, -1.8)); // measured pose drives the model
        model.rebuild_misarta_model();
        let solid = model.compute_transforms();
        let saved_q = model.joint_positions.clone();

        let ghost = viz.ghost_transforms(&mut model).expect("ghost");

        assert_eq!(
            model.joint_positions, saved_q,
            "model must be left untouched"
        );
        assert_eq!(model.compute_transforms()["FL_calf"], solid["FL_calf"]);
        assert_ne!(
            ghost["FL_calf"], solid["FL_calf"],
            "target calf differs from measured, so the ghost must too"
        );
        assert_eq!(ghost["FL_thigh"], solid["FL_thigh"], "thighs agree");

        // Overlay off, or no measured stream (the target then *is* the solid
        // model): nothing to ghost.
        viz.overlay_target = false;
        assert!(viz.ghost_transforms(&mut model).is_none());
        viz.overlay_target = true;
        viz.got_measured = false;
        assert!(viz.ghost_transforms(&mut model).is_none());
    }

    /// Measured-only (nothing publishing the target): the measured stream
    /// drives the model and there is simply nothing to ghost.
    #[test]
    fn measured_without_a_target_stream_ghosts_nothing() {
        let mut model = go2_model();
        let viz = VizFeedState {
            target: None,
            got_measured: true,
            ..Default::default()
        };
        assert!(viz.ghost_transforms(&mut model).is_none());
    }

    /// Both keys naming one stream is a misconfiguration, not a valid setup:
    /// the ghost would land exactly on the solid model and the pose would flip
    /// between whichever sample arrived last.
    #[test]
    fn the_same_key_twice_is_a_conflict() {
        let viz = VizFeedState {
            key: "go2/gait/planned".into(),
            key_measured: " go2/gait/planned ".into(), // whitespace must not hide it
            ..Default::default()
        };
        assert!(viz.measured_key_conflicts());

        let ok = VizFeedState::default();
        assert_ne!(ok.key, ok.key_measured);
        assert!(!ok.measured_key_conflicts());

        let target_only = VizFeedState {
            key_measured: String::new(),
            ..Default::default()
        };
        assert!(
            !target_only.measured_key_conflicts(),
            "an empty measured key is target-only, not a conflict"
        );
    }

    /// An empty key is skipped, not subscribed to — so a single-stream setup
    /// (either one) still reports itself as active.
    #[test]
    fn an_empty_key_is_not_subscribed() {
        let mut viz = VizFeedState {
            key: String::new(),
            key_measured: String::new(),
            ..Default::default()
        };
        viz.toggle();
        assert!(!viz.active(), "both keys empty: nothing to subscribe to");
    }
}
