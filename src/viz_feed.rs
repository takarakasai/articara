//! Live gait **viewer**: subscribe to a Zenoh stream of
//! [`quadruped_gait::viz::GaitVizFrame`] (published by `go2-gait-runner --viz`)
//! and drive the loaded model's joints + trunk pose so the gait the runner
//! generates can be watched in real time.
//!
//! The transport lives in [`quadruped_gait::viz_sub`] (Zenoh, pure Rust —
//! no DDS/C toolchain); this module only applies received frames to the
//! loaded model each repaint, via the same path as kinematic playback
//! (`model.joint_positions` + `model.base_transform`). Feature-gated (`viz`).

use nalgebra as na;
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

/// GUI-side state for the live gait feed. Holds the (optional) running
/// subscriber and the editable Zenoh key.
pub struct VizFeedState {
    sub: Option<VizSubscriber>,
    /// Zenoh key to subscribe to (editable while stopped).
    pub key: String,
    /// Optional Zenoh **connect** endpoint (e.g. `tcp/127.0.0.1:7447`) for hosts
    /// without multicast; connects to the publisher's `--viz-endpoint`. Empty =
    /// auto multicast discovery.
    pub endpoint: String,
    /// Sequence number of the last applied frame (for a status read-out).
    pub last_seq: Option<u64>,
}

impl Default for VizFeedState {
    fn default() -> Self {
        Self {
            sub: None,
            key: quadruped_gait::viz::VIZ_KEY_PLANNED.to_string(),
            endpoint: String::new(),
            last_seq: None,
        }
    }
}

impl VizFeedState {
    /// Whether the subscriber is currently running.
    pub fn active(&self) -> bool {
        self.sub.is_some()
    }

    /// Start the subscriber (if stopped) or stop it (if running).
    pub fn toggle(&mut self) {
        if self.sub.is_some() {
            self.sub = None;
            self.last_seq = None;
        } else {
            let ep = if self.endpoint.trim().is_empty() {
                None
            } else {
                Some(self.endpoint.trim())
            };
            match VizSubscriber::new(&self.key, ep) {
                Ok(s) => self.sub = Some(s),
                Err(e) => eprintln!("viz-feed: {e}"),
            }
        }
    }

    /// Apply the latest received frame to `model` (joint angles + trunk pose),
    /// rebuilding the render model. Returns `true` if a frame was applied.
    pub fn apply(&mut self, model: &mut RobotModel) -> bool {
        let Some(sub) = &self.sub else {
            return false;
        };
        let Some(frame) = sub.take_latest() else {
            return false;
        };
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
        model.rebuild_misarta_model();
        self.last_seq = Some(frame.seq);
        true
    }
}
