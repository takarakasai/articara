//! MuJoCo physics simulation integration.

use std::collections::VecDeque;
use std::path::PathBuf;
use std::sync::Arc;

use mujoco::prelude::{MjData, MjModel, load_all_plugin_libraries};
use nalgebra as na;

use crate::mjcf::MjcfExportOptions;
use crate::rbd::model::ActuatorMode;
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
    /// Per-joint position target (used by Position-mode controller). Indexed
    /// by joint index in the [`RobotModel`]. Initialised to the robot's pose
    /// at sim start so the robot holds that pose against gravity.
    position_targets: Vec<f64>,
    /// Per-joint velocity target (used by Velocity-mode controller).
    velocity_targets: Vec<f64>,
    /// Per-joint direct torque command (used by Torque-mode controller).
    torque_targets: Vec<f64>,
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
    ///
    /// `opts.add_actuators` is forced to `true` — every interactive sim needs
    /// per-joint actuators so the user-set initial pose can be held against
    /// gravity. The actuator type for each joint is selected by its
    /// `actuator_mode` field (Position / Velocity / Torque).
    pub fn new(robot: &RobotModel, mut opts: MjcfExportOptions) -> Result<Self, String> {
        opts.add_actuators = true;

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
        let mut data = MjData::new(Arc::clone(&model));

        // Seed MuJoCo's qpos with the user's current joint angles so the sim
        // starts in the same pose the editor is showing. The MJCF only carries
        // structure; per-joint qpos defaults to 0 unless we write them here.
        for (ji, joint) in robot.joints.iter().enumerate() {
            if joint.joint_type == "fixed" {
                continue;
            }
            if let Some(joint_info) = data.joint(&joint.name) {
                let mut view = joint_info.view_mut(&mut data);
                if !view.qpos.is_empty() {
                    view.qpos[0] = robot.joint_positions[ji];
                }
            }
        }

        // Refresh xpos/xquat/qfrc_bias etc. from the seeded qpos so the very
        // first sync_back can render the correct initial pose.
        data.forward();

        // Initial control targets: hold the start pose in Position mode, no
        // velocity/torque command in the other modes.
        let position_targets = robot.joint_positions.clone();
        let velocity_targets = vec![0.0; robot.joints.len()];
        let torque_targets = vec![0.0; robot.joints.len()];

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
            position_targets,
            velocity_targets,
            torque_targets,
        })
    }

    /// Set the position target (rad / m) for a joint by index.
    pub fn set_position_target(&mut self, joint_idx: usize, target: f64) {
        if let Some(slot) = self.position_targets.get_mut(joint_idx) {
            *slot = target;
        }
    }

    /// Set the velocity target (rad/s / m/s) for a joint by index.
    pub fn set_velocity_target(&mut self, joint_idx: usize, target: f64) {
        if let Some(slot) = self.velocity_targets.get_mut(joint_idx) {
            *slot = target;
        }
    }

    /// Set the direct torque command for a joint by index.
    pub fn set_torque_target(&mut self, joint_idx: usize, target: f64) {
        if let Some(slot) = self.torque_targets.get_mut(joint_idx) {
            *slot = target;
        }
    }

    /// Compute and write each motor's `ctrl` (= applied torque) for the
    /// upcoming physics tick, based on the per-joint mode + gains in `robot`
    /// and the controller targets stored on `self`.
    fn apply_controller(&mut self, robot: &RobotModel) {
        for (ji, joint) in robot.joints.iter().enumerate() {
            if joint.joint_type == "fixed" {
                continue;
            }

            // Read current MuJoCo state for this joint.
            let (q, qd) = match self.data.joint(&joint.name) {
                Some(info) => {
                    let view = info.view(&self.data);
                    if view.qpos.is_empty() || view.qvel.is_empty() {
                        continue;
                    }
                    (view.qpos[0], view.qvel[0])
                }
                None => continue,
            };

            let tau = match joint.actuator_mode {
                ActuatorMode::Position => {
                    let q_target = self.position_targets.get(ji).copied().unwrap_or(q);
                    joint.actuator_kp * (q_target - q)
                        + joint.actuator_kv * (0.0 - qd)
                }
                ActuatorMode::Velocity => {
                    let qd_target = self.velocity_targets.get(ji).copied().unwrap_or(0.0);
                    joint.actuator_kv * (qd_target - qd)
                }
                ActuatorMode::Torque => {
                    self.torque_targets.get(ji).copied().unwrap_or(0.0)
                }
            };

            // Write to the motor actuator's ctrl slot.
            let actuator_name = format!("motor_{}", joint.name);
            if let Some(act_info) = self.data.actuator(&actuator_name) {
                let mut view = act_info.view_mut(&mut self.data);
                if !view.ctrl.is_empty() {
                    view.ctrl[0] = tau;
                }
            }
        }
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
            self.apply_controller(robot);
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
            self.apply_controller(robot);
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
