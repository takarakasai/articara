//! Live gait **viewer**: subscribe to a Zenoh stream of
//! [`quadruped_gait::viz::GaitVizFrame`] (published by `go2-gait-runner --viz`)
//! and drive the loaded model's joints + trunk pose so the gait the runner
//! generates can be watched in real time.
//!
//! The transport is Zenoh (pure Rust — no DDS/C toolchain in `articara`). A
//! background thread runs the subscriber and keeps the latest frame; the GUI
//! applies it each repaint via the same path as kinematic playback
//! (`model.joint_positions` + `model.base_transform`). Feature-gated (`viz`).

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use nalgebra as na;
use quadruped_gait::viz::GaitVizFrame;
use zenoh::Wait;

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

/// Background Zenoh subscriber holding the latest received frame.
struct VizSubscriber {
    latest: Arc<Mutex<Option<GaitVizFrame>>>,
    running: Arc<AtomicBool>,
    _handle: std::thread::JoinHandle<()>,
}

impl VizSubscriber {
    /// `endpoint = Some(ep)` connects to a Zenoh peer listening at `ep` (TCP)
    /// and disables multicast — use it when multicast discovery isn't available
    /// (same host / WSL2). `None` = auto multicast discovery.
    fn new(key: &str, endpoint: Option<&str>) -> Result<Self, String> {
        let latest: Arc<Mutex<Option<GaitVizFrame>>> = Arc::new(Mutex::new(None));
        let running = Arc::new(AtomicBool::new(true));
        let l2 = latest.clone();
        let r2 = running.clone();
        let key = key.to_string();
        let mut config = zenoh::Config::default();
        if let Some(ep) = endpoint {
            config
                .insert_json5("connect/endpoints", &format!("[\"{ep}\"]"))
                .map_err(|e| format!("zenoh connect endpoint '{ep}': {e}"))?;
            let _ = config.insert_json5("scouting/multicast/enabled", "false");
        }
        let handle = std::thread::Builder::new()
            .name("viz-sub".into())
            .spawn(move || {
                let session = match zenoh::open(config).wait() {
                    Ok(s) => s,
                    Err(e) => {
                        eprintln!("viz-feed: zenoh open failed: {e}");
                        return;
                    }
                };
                let sub = match session.declare_subscriber(&key).wait() {
                    Ok(s) => s,
                    Err(e) => {
                        eprintln!("viz-feed: subscribe '{key}' failed: {e}");
                        return;
                    }
                };
                // recv_timeout (not blocking recv) so the thread can notice the
                // stop flag and exit when the user toggles the feed off.
                while r2.load(Ordering::Relaxed) {
                    match sub.recv_timeout(Duration::from_millis(200)) {
                        Ok(Some(sample)) => {
                            let bytes = sample.payload().to_bytes();
                            if let Ok(frame) = serde_json::from_slice::<GaitVizFrame>(&bytes) {
                                if frame.is_compatible() {
                                    *l2.lock().unwrap() = Some(frame);
                                }
                            }
                        }
                        Ok(None) => {} // timeout — re-check the stop flag
                        Err(_) => break,
                    }
                }
            })
            .map_err(|e| format!("spawn viz-sub thread: {e}"))?;
        Ok(Self {
            latest,
            running,
            _handle: handle,
        })
    }

    /// Take (consume) the latest frame, if a new one has arrived.
    fn take_latest(&self) -> Option<GaitVizFrame> {
        self.latest.lock().ok().and_then(|mut g| g.take())
    }
}

impl Drop for VizSubscriber {
    fn drop(&mut self) {
        self.running.store(false, Ordering::Relaxed);
    }
}

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
