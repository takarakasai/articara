//! RL-policy controller (sim2sim). Loads an exported Isaac Lab locomotion policy
//! (ONNX) via the pure-Rust `tract` runtime and reproduces the exact deployment
//! contract used by `go2-gait-runner`'s `policy` mode, so a policy can be
//! validated in articara's MuJoCo sim before it ever touches hardware.
//!
//! Observation (45-d, Isaac joint order, no scaling/normalization):
//!   base_ang_vel(3, body) · projected_gravity(3, body) · velocity_commands(3)
//!   · (joint_pos − default)(12) · joint_vel(12) · last_action(12)
//! Action: q_des = default + ACTION_SCALE · action, tracked by the sim's PD.
//!
//! Only compiled with the `onnx` feature.

use tract_onnx::prelude::*;

/// Number of observation inputs the deploy policy expects.
pub const N_OBS: usize = 45;

/// Joints in **Isaac Lab order** (grouped by type: all hips, thighs, calves).
/// The policy's obs/action are in this order; the sim is read/written by name.
pub const ISAAC_JOINT_NAMES: [&str; 12] = [
    "FL_hip_joint", "FR_hip_joint", "RL_hip_joint", "RR_hip_joint",
    "FL_thigh_joint", "FR_thigh_joint", "RL_thigh_joint", "RR_thigh_joint",
    "FL_calf_joint", "FR_calf_joint", "RL_calf_joint", "RR_calf_joint",
];

/// Default joint positions in Isaac order (the policy's nominal pose). The
/// action is applied as `q_des = default + ACTION_SCALE * action`.
pub const DEFAULT_ISAAC: [f64; 12] = [
    0.1, -0.1, 0.1, -0.1, // hips: FL,FR,RL,RR
    0.8, 0.8, 1.0, 1.0, //   thighs
    -1.5, -1.5, -1.5, -1.5, // calves
];

/// Isaac Lab `JointPositionActionCfg` scale (use_default_offset = True).
pub const ACTION_SCALE: f64 = 0.5;

/// On-board PD gains the policy was trained with (Go2 actuator cfg).
pub const POLICY_KP: f64 = 25.0;
pub const POLICY_KD: f64 = 0.5;

/// Build the 45-d observation. Inputs are already in **Isaac joint order**:
///   `ang_vel_b`, `proj_grav_b` — base angular velocity / projected gravity in
///   the body frame; `cmd` = [vx, vy, wz]; `q`, `qd` = absolute joint position
///   and velocity; `last_action` = previous raw policy output.
pub fn build_obs(
    ang_vel_b: [f64; 3],
    proj_grav_b: [f64; 3],
    cmd: [f64; 3],
    q: &[f64; 12],
    qd: &[f64; 12],
    last_action: &[f32; 12],
) -> [f32; N_OBS] {
    let mut o = [0.0f32; N_OBS];
    for i in 0..3 {
        o[i] = ang_vel_b[i] as f32;
        o[3 + i] = proj_grav_b[i] as f32;
        o[6 + i] = cmd[i] as f32;
    }
    for i in 0..12 {
        o[9 + i] = (q[i] - DEFAULT_ISAAC[i]) as f32;
        o[21 + i] = qd[i] as f32;
        o[33 + i] = last_action[i];
    }
    o
}

/// A loaded ONNX policy (small MLP, 45 → 12). Output is the deterministic mean
/// action; apply it as `q_des = default + ACTION_SCALE * action`.
pub struct OnnxPolicy {
    plan: TypedRunnableModel<TypedModel>,
}

impl OnnxPolicy {
    /// Load and optimize an ONNX policy with a fixed `[1, N_OBS]` input.
    pub fn load(path: &str) -> Result<Self, String> {
        let plan = tract_onnx::onnx()
            .model_for_path(path)
            .map_err(|e| format!("load onnx {path}: {e}"))?
            .with_input_fact(0, f32::fact([1, N_OBS]).into())
            .map_err(|e| format!("input fact: {e}"))?
            .into_optimized()
            .map_err(|e| format!("optimize: {e}"))?
            .into_runnable()
            .map_err(|e| format!("runnable: {e}"))?;
        Ok(Self { plan })
    }

    /// Run inference; returns the 12 raw action values (Isaac order).
    pub fn infer(&self, obs: &[f32; N_OBS]) -> Result<[f32; 12], String> {
        let input: Tensor = tract_ndarray::Array2::<f32>::from_shape_vec((1, N_OBS), obs.to_vec())
            .map_err(|e| format!("obs shape: {e}"))?
            .into();
        let out = self
            .plan
            .run(tvec!(input.into()))
            .map_err(|e| format!("inference: {e}"))?;
        let view = out[0]
            .to_array_view::<f32>()
            .map_err(|e| format!("output view: {e}"))?;
        let mut a = [0.0f32; 12];
        for i in 0..12 {
            a[i] = view[[0, i]];
        }
        Ok(a)
    }
}
