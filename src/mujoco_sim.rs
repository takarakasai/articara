//! MuJoCo physics simulation integration.

use std::collections::VecDeque;
use std::path::PathBuf;
use std::sync::Arc;

use mujoco::prelude::{MjData, MjModel, load_all_plugin_libraries};
use nalgebra as na;

use crate::mjcf::MjcfExportOptions;
use crate::robot::RobotModel;

/// One MuJoCo physics tick worth of state, captured *before* `data.step()` was
/// called. Replaying it via [`MujocoSim::step_back_frames`] restores the sim to
/// that pre-step state.
#[derive(Clone)]
struct FrameSnapshot {
    qpos: Vec<f64>,
    qvel: Vec<f64>,
    time: f64,
}

/// Active MuJoCo simulation instance.
pub struct MujocoSim {
    model: Arc<MjModel>,
    data: MjData<Arc<MjModel>>,
    time_accumulator: f64,
    /// Robot pose captured at sim start, restored on Stop.
    saved_base_transform: na::Isometry3<f64>,
    saved_joint_positions: Vec<f64>,
    /// Ring buffer of pre-step snapshots, used for backward frame stepping.
    history: VecDeque<FrameSnapshot>,
    /// Maximum number of snapshots to retain (older entries are discarded).
    history_max: usize,
}

/// Finds the `bin/mujoco_plugin` directory inside the MuJoCo installation.
/// Checks `MUJOCO_DOWNLOAD_DIR` first, then `$HOME/.mujoco`.
fn find_plugin_dir() -> Option<PathBuf> {
    let base = std::env::var("MUJOCO_DOWNLOAD_DIR")
        .map(PathBuf::from)
        .unwrap_or_else(|_| {
            std::env::var("HOME")
                .map(|h| PathBuf::from(h).join(".mujoco"))
                .unwrap_or_default()
        });

    std::fs::read_dir(&base).ok()?.find_map(|entry| {
        let entry = entry.ok()?;
        let name = entry.file_name();
        if name.to_string_lossy().starts_with("mujoco-") {
            let plugin_dir = entry.path().join("bin").join("mujoco_plugin");
            plugin_dir.exists().then_some(plugin_dir)
        } else {
            None
        }
    })
}

impl MujocoSim {
    /// Create a new MuJoCo simulation instance from the current RobotModel
    /// using the supplied MJCF export options.
    pub fn new(robot: &RobotModel, opts: MjcfExportOptions) -> Result<Self, String> {
        // Load MuJoCo plugins (STL decoder, OBJ decoder, etc.) before loading any model.
        if let Some(dir) = find_plugin_dir() {
            load_all_plugin_libraries(&dir, None)
                .map_err(|e| format!("Failed to load MuJoCo plugins from {dir:?}: {e}"))?;
        }

        let xml = crate::mjcf::export_mjcf_with_options(robot, opts);

        let model = Arc::new(
            MjModel::from_xml_string(&xml)
                .map_err(|e| format!("Failed to load MuJoCo model: {e}"))?,
        );
        let data = MjData::new(Arc::clone(&model));

        Ok(Self {
            model,
            data,
            time_accumulator: 0.0,
            saved_base_transform: robot.base_transform,
            saved_joint_positions: robot.joint_positions.clone(),
            history: VecDeque::new(),
            // ~10s of history at the default 2 ms timestep — bounded so the
            // ring buffer can't grow without bound during long sessions.
            history_max: 5000,
        })
    }

    /// Restore the robot's pre-sim pose (called when the user stops the sim).
    pub fn restore(&self, robot: &mut RobotModel) {
        robot.base_transform = self.saved_base_transform;
        robot.joint_positions = self.saved_joint_positions.clone();
    }

    /// MuJoCo's native physics timestep (s).
    pub fn timestep(&self) -> f64 {
        self.model.ffi().opt.timestep as f64
    }

    /// Step the simulation by `dt` seconds and sync the state back to `RobotModel`.
    pub fn step(&mut self, robot: &mut RobotModel, dt: f64) {
        self.time_accumulator += dt;

        let mj_dt = self.timestep();
        while self.time_accumulator >= mj_dt {
            self.snapshot();
            self.data.step();
            self.time_accumulator -= mj_dt;
        }

        self.sync_back(robot);
    }

    /// Advance the simulation by exactly `n` physics frames (each = `timestep()`
    /// seconds) and sync the state back. Each frame is pre-snapshotted so it
    /// can be reversed via [`Self::step_back_frames`].
    pub fn step_n_frames(&mut self, robot: &mut RobotModel, n: u32) {
        for _ in 0..n {
            self.snapshot();
            self.data.step();
        }
        // Drop any partial-frame accumulator so explicit frame stepping is exact.
        self.time_accumulator = 0.0;
        self.sync_back(robot);
    }

    /// Restore the simulation to its state `n` frames ago (or as far back as
    /// the history allows). Calls `mj_forward` to refresh derived quantities
    /// before syncing back to `robot`.
    pub fn step_back_frames(&mut self, robot: &mut RobotModel, n: u32) {
        let mut popped = 0;
        while popped < n {
            let Some(snap) = self.history.pop_back() else {
                break;
            };
            self.data.qpos_mut().copy_from_slice(&snap.qpos);
            self.data.qvel_mut().copy_from_slice(&snap.qvel);
            // SAFETY: Writing the scalar `time` field on the FFI struct is a
            // simple memory store — no MuJoCo invariants depend on the value
            // beyond the next call to `data.forward()` below.
            unsafe { self.data.ffi_mut().time = snap.time; }
            popped += 1;
        }
        if popped > 0 {
            // Refresh xpos/xquat/qfrc_bias etc. from the restored qpos/qvel.
            self.data.forward();
            self.time_accumulator = 0.0;
        }
        self.sync_back(robot);
    }

    /// Number of recorded frames currently available for [`Self::step_back_frames`].
    pub fn history_len(&self) -> usize {
        self.history.len()
    }

    /// Push a pre-step snapshot to the history ring, dropping the oldest if
    /// the buffer is full.
    fn snapshot(&mut self) {
        if self.history.len() >= self.history_max {
            self.history.pop_front();
        }
        self.history.push_back(FrameSnapshot {
            qpos: self.data.qpos().to_vec(),
            qvel: self.data.qvel().to_vec(),
            time: self.data.ffi().time,
        });
    }

    /// Mirror MuJoCo's body and joint state back into `robot`.
    fn sync_back(&self, robot: &mut RobotModel) {
        // Floating-base world pose from root body xpos / xquat.
        if let Some(body_info) = self.data.body(&robot.root_link) {
            let view = body_info.view(&self.data);
            let translation = na::Translation3::new(view.xpos[0], view.xpos[1], view.xpos[2]);
            // MuJoCo stores quaternions in (w, x, y, z) order, matching
            // nalgebra's `Quaternion::new(w, i, j, k)` constructor.
            let quat = na::Quaternion::new(view.xquat[0], view.xquat[1], view.xquat[2], view.xquat[3]);
            let rotation = na::UnitQuaternion::from_quaternion(quat);
            robot.base_transform = na::Isometry3::from_parts(translation, rotation);
        }

        for (ji, joint) in robot.joints.iter().enumerate() {
            if joint.joint_type == "fixed" {
                continue;
            }
            if let Some(joint_info) = self.data.joint(&joint.name) {
                let view = joint_info.view(&self.data);
                if !view.qpos.is_empty() {
                    robot.joint_positions[ji] = view.qpos[0] as f64;
                }
            }
        }
    }
}
