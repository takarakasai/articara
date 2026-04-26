/// misartaのForwardDynamicsStateを使い、misartaのトルク計算でMuJoCoを駆動するジャンプシミュレーション
pub struct MujocoMisartaJumpSim {
    pub sim: MujocoSim,
    pub fd_state: crate::rbd::dynamics::ForwardDynamicsState,
    pub joint_names: Vec<String>,
    pub kp: f64,
    pub kd: f64,
    pub elapsed: f64,
    pub duration: f64,
    pub phase: JumpPhase,
    pub phase_time: f64,
    pub retract_targets: Option<Vec<f64>>,
    pub initial_joint_positions: Vec<f64>,
    pub landed_hold: f64,
    pub frame_history: std::collections::VecDeque<(f64, f64, JumpPhase, f64, Vec<f64>, na::Isometry3<f64>)>,
    pub frame_history_max: usize,
}

impl MujocoMisartaJumpSim {
        /// 1フレーム戻す
        pub fn step_back(&mut self, robot: &mut RobotModel) {
            if let Some((elapsed, phase_time, phase, duration, joint_positions, base_transform)) = self.frame_history.pop_back() {
                self.elapsed = elapsed;
                self.phase_time = phase_time;
                self.phase = phase;
                self.duration = duration;
                robot.joint_positions = joint_positions;
                robot.base_transform = base_transform;
            }
        }
    /// セットアップ: misartaのstart_jump_simでForwardDynamicsStateを生成し、MuJoCoに適用
    pub fn new(
        robot: &mut RobotModel,
        ground_links: &[String],
        body_link: Option<&str>,
        speed: f64,
        locked_joints: &HashSet<String>,
        extension_override: Option<f64>,
        pd_kp: f64,
        pd_kd: f64,
        base_pos: Option<[f64; 3]>,
        ground_plane: Option<GroundPlaneCfg>,
    ) -> Result<Self, String> {
        let jump = crate::dynamics::start_jump_sim(
            robot,
            ground_links,
            body_link,
            speed,
            locked_joints,
            [false, false, true],
            extension_override,
            false,
            false,
            None,
            pd_kp,
            pd_kd,
        ).ok_or_else(|| "Cannot plan jump (no leg joints with effort limits?)".to_string())?;

        let fd_state = jump.fd_state.clone().ok_or_else(|| "No forward dynamics state".to_string())?;
        let joint_names = robot.joints.iter().map(|j| j.name.clone()).collect();
        let initial_joint_positions = robot.joint_positions.clone();

        let mut sim = MujocoSim::new(
            robot,
            MjcfExportOptions {
                base_pos,
                ground_plane,
                add_actuators: true,
            },
        )?;

        // --- base_transformのz座標をMuJoCoのqposに反映 ---
        {
            let base_z = robot.base_transform.translation.vector.z;
            if let Some(qpos) = sim.data.qpos_mut().get_mut(2) {
                *qpos = base_z;
            }
        }
        Ok(Self {
            sim,
            fd_state,
            joint_names,
            kp: pd_kp,
            kd: pd_kd,
            elapsed: 0.0,
            duration: jump.extension_duration,
            phase: JumpPhase::Extension,
            phase_time: 0.0,
            retract_targets: None,
            initial_joint_positions,
            landed_hold: 0.3, // [s] 着地後に保持する時間
            frame_history: std::collections::VecDeque::new(),
            frame_history_max: 2000,
        })
    }

    /// ロボットの初期姿勢に戻す
    pub fn restore(&self, robot: &mut RobotModel) {
        self.sim.restore(robot);
    }

    /// misartaのトルク計算でMuJoCoを1ステップ進める
    pub fn step(&mut self, robot: &mut RobotModel, dt: f64) {
        // スナップショット保存
        if self.frame_history.len() >= self.frame_history_max {
            self.frame_history.pop_front();
        }
        self.frame_history.push_back((
            self.elapsed,
            self.phase_time,
            self.phase,
            self.duration,
            robot.joint_positions.clone(),
            robot.base_transform,
        ));
        self.sim.time_accumulator += dt;
        let mj_dt = self.sim.model.ffi().opt.timestep as f64;
        while self.sim.time_accumulator >= mj_dt {
            self.apply_misarta_controller(robot, mj_dt);
            self.sim.data.step();
            self.elapsed += mj_dt;
            self.phase_time += mj_dt;
            self.fd_state.trajectory_time += mj_dt;
            self.sim.time_accumulator -= mj_dt;
        }
        self.sim.sync_back(robot);
    }

    /// misartaのForwardDynamicsState (CRBA + RNEA) で計算トルク制御。
    /// τ = M(q)·a_pd + h(q,qd)  with  a_pd = qdd* + Kp(q*-q) + Kd(qd*-qd)
    ///
    /// 1. MuJoCoの最新state(qpos, qvel)を model + fd_state に同期
    /// 2. misartaの `current_torque` でアクティブ関節のτを計算
    /// 3. アクティブ関節 → misarta τ、非アクティブ可動関節 → PD保持(+qfrc_bias)
    fn apply_misarta_controller(&mut self, robot: &mut RobotModel, _dt: f64) {
        // (1) MuJoCo state を model / fd_state に同期
        for (ji, joint) in robot.joints.iter().enumerate() {
            if joint.joint_type == "fixed" {
                continue;
            }
            if let Some((q, qd, _)) = self.sim.joint_state(&joint.name) {
                robot.joint_positions[ji] = q;
                self.fd_state.joint_velocities.insert(ji, qd);
            }
        }

        // (2) misartaでトルクベクトルを計算
        let tau_misarta = self.fd_state.current_torque(robot);

        // (3) 振り分けて motor ctrl に書き込む
        let active: std::collections::HashSet<usize> =
            self.fd_state.joint_order.iter().copied().collect();
        for (ji, joint) in robot.joints.iter().enumerate() {
            if joint.joint_type == "fixed" {
                continue;
            }
            let tau_cmd = if active.contains(&ji) {
                // 計算トルク制御 — misartaのCRBA/RNEA結果を直接適用
                tau_misarta.get(ji).copied().unwrap_or(0.0)
            } else {
                // 腕 / locked / その他 → 初期姿勢を PD + qfrc_bias で保持
                let Some((q, qd, bias)) = self.sim.joint_state(&joint.name) else {
                    continue;
                };
                let q_target = self
                    .initial_joint_positions
                    .get(ji)
                    .copied()
                    .unwrap_or(q);
                self.kp * (q_target - q) + self.kd * (0.0 - qd) + bias
            };
            self.sim.set_motor_ctrl(&joint.name, tau_cmd);
        }
    }

    pub fn extension_done(&self) -> bool {
        self.elapsed >= self.duration
    }
}
/// MuJoCo physics simulation integration.

use std::collections::HashSet;
use crate::dynamics::JumpPhase;
use std::path::PathBuf;
use std::sync::Arc;

use mujoco::prelude::{MjData, MjModel, load_all_plugin_libraries};
use nalgebra as na;

use crate::mjcf::{GroundPlaneCfg, MjcfExportOptions};
use crate::robot::RobotModel;

/// Active MuJoCo simulation instance.
pub struct MujocoSim {
    model: Arc<MjModel>,
    data: MjData<Arc<MjModel>>,
    time_accumulator: f64,
    /// Robot pose captured at sim start, restored on Stop.
    saved_base_transform: na::Isometry3<f64>,
    saved_joint_positions: Vec<f64>,
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
        })
    }

    /// Restore the robot's pre-sim pose (called when the user stops the sim).
    pub fn restore(&self, robot: &mut RobotModel) {
        robot.base_transform = self.saved_base_transform;
        robot.joint_positions = self.saved_joint_positions.clone();
    }

    /// Step the simulation by `dt` seconds and sync the state back to `RobotModel`.
    pub fn step(&mut self, robot: &mut RobotModel, dt: f64) {
        self.time_accumulator += dt;

        let mj_dt = self.model.ffi().opt.timestep as f64;

        while self.time_accumulator >= mj_dt {
            self.data.step();
            self.time_accumulator -= mj_dt;
        }

        self.sync_back(robot);
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

    /// Read (qpos[0], qvel[0], qfrc_bias[0]) for a 1-DoF joint by name.
    fn joint_state(&self, joint_name: &str) -> Option<(f64, f64, f64)> {
        let info = self.data.joint(joint_name)?;
        let view = info.view(&self.data);
        if view.qpos.is_empty() || view.qvel.is_empty() {
            return None;
        }
        let bias = view.qfrc_bias.first().copied().unwrap_or(0.0);
        Some((view.qpos[0], view.qvel[0], bias))
    }

    /// Write a torque to the actuator named `motor_<joint_name>`.
    fn set_motor_ctrl(&mut self, joint_name: &str, tau: f64) {
        let actuator_name = format!("motor_{joint_name}");
        if let Some(info) = self.data.actuator(&actuator_name) {
            let mut view_mut = info.view_mut(&mut self.data);
            if !view_mut.ctrl.is_empty() {
                view_mut.ctrl[0] = tau;
            }
        }
    }
}

// =============================================================================
// Active-sim enum (plain MuJoCo vs MuJoCo jump)
// =============================================================================

/// Whichever MuJoCo simulation kind is currently running, if any.
pub enum MujocoActiveSim {
    Plain(MujocoSim),
    Jump(MujocoMisartaJumpSim),
}

impl MujocoActiveSim {
    pub fn step(&mut self, robot: &mut RobotModel, dt: f64) {
        match self {
            Self::Plain(s) => s.step(robot, dt),
            Self::Jump(s) => s.step(robot, dt),
        }
    }

    pub fn restore(&self, robot: &mut RobotModel) {
        match self {
            Self::Plain(s) => s.restore(robot),
            Self::Jump(s) => s.restore(robot),
        }
    }
}
