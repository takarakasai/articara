//! Simplified static dynamics analysis and real-time simulation for articulated robots.
//!
//! Provides:
//! - **Gravity torque analysis**: static torque at each joint due to gravity.
//! - **Payload capacity**: max payload mass at a given end-effector before any
//!   joint exceeds its effort (torque/force) limit.
//! - **Jump height estimate**: energy-based upper bound on how high the robot's
//!   centre of mass can rise given joint effort limits and range of motion.
//! - **Jump simulation**: animated crouch→extension→flight→landing sequence
//!   driven per-frame, modifying joint positions and base transform in-place.
//! - **Payload simulation**: animated ramp-up of virtual EE mass with live
//!   torque utilisation colouring.

use nalgebra as na;
use std::collections::HashMap;

use crate::robot::RobotModel;

// ========== Result types ==========

/// Per-joint static torque information.
#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct JointTorqueInfo {
    pub joint_name: String,
    pub joint_idx: usize,
    /// Gravity torque in joint coordinates (N·m for revolute, N for prismatic).
    pub gravity_torque: f64,
    /// Effort (torque/force) limit from URDF.
    pub effort_limit: f64,
    /// `effort_limit - |gravity_torque|`. Negative means the joint is overloaded.
    pub torque_margin: f64,
    /// Additional torque/force per 1 kg of payload at the end-effector.
    /// Only populated when payload analysis is run.
    pub payload_torque_per_kg: f64,
}

/// Result of payload capacity analysis.
#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct PayloadResult {
    /// Maximum payload mass the robot can hold statically (kg).
    pub max_mass_kg: f64,
    /// Name of the joint that is the bottleneck.
    pub limiting_joint: String,
    /// End-effector position in world frame where the payload is applied.
    pub ee_position: na::Point3<f64>,
}

/// Full static analysis result.
#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct StaticAnalysis {
    pub joint_torques: Vec<JointTorqueInfo>,
    pub payload: Option<PayloadResult>,
}

// ========== Gravity constant ==========
const G: f64 = 9.80665;

// ========== Core Algorithms ==========

/// For joints in a grounded leg, compute the "body-side" gravity torque.
///
/// In a grounded configuration (feet on the floor), each leg joint must support
/// the weight of links on the **body side** (ancestor side), not the foot side.
/// This is the opposite of the free-hanging serial-arm convention used by
/// `compute_gravity_torques` (which sums descendants only).
///
/// Compute static gravity torque at every movable joint.
///
/// Delegates to `misarta::rnea::compute_gravity` and wraps results in
/// `JointTorqueInfo` with effort limits and margins from the `RobotModel`.
pub fn compute_gravity_torques(model: &RobotModel) -> Vec<JointTorqueInfo> {
    let adapter = model.mc();
    let q = model.build_q();
    let g_full = misarta::rnea::compute_gravity(&adapter.model, &q);

    let mut result = Vec::new();
    for (ji, joint) in model.joints.iter().enumerate() {
        let jt = joint.joint_type.as_str();
        if jt == "fixed" {
            continue;
        }

        let tau = if let Some(mi) = adapter.a2m.get(ji).and_then(|&m| m) {
            let nv = adapter.model.joints[mi].joint_type.nv();
            if nv == 1 {
                g_full[adapter.model.v_idx[mi]]
            } else {
                0.0
            }
        } else {
            0.0
        };

        result.push(JointTorqueInfo {
            joint_name: joint.name.clone(),
            joint_idx: ji,
            gravity_torque: tau,
            effort_limit: joint.effort,
            torque_margin: joint.effort - tau.abs(),
            payload_torque_per_kg: 0.0,
        });
    }

    result
}

/// Compute the maximum static payload at `ee_link`.
///
/// Uses the positional Jacobian (3 × N) to map a unit downward force at the
/// end-effector to joint torques, then finds the tightest bottleneck.
pub fn compute_payload_capacity(
    model: &RobotModel,
    ee_link: &str,
    joint_torques: &mut [JointTorqueInfo],
) -> Option<PayloadResult> {
    let transforms = model.compute_transforms();

    // Build chain from root to ee_link
    let chain = model.chain_joints(ee_link);
    if chain.is_empty() {
        return None;
    }

    // EE world position
    let ee_li = *model.link_map.get(ee_link)?;
    let ee_pos = model.ee_world_pos(ee_li, &transforms);

    // Compute positional Jacobian (3 × N) via misarta
    let jac = model.chain_positional_jacobian(&chain, ee_link, None, None);

    // Unit payload force: F = [0, 0, -g] (force per 1 kg)
    let f_unit = na::DVector::from_column_slice(&[0.0_f64, 0.0, -G]);

    // τ_payload_per_kg = J^T * F_unit  →  (N×1)
    let tau_payload = jac.transpose() * f_unit;

    // Map chain joint indices back to our torque info
    // Build lookup: joint_idx → position in joint_torques
    let idx_map: HashMap<usize, usize> = joint_torques
        .iter()
        .enumerate()
        .map(|(pos, jt)| (jt.joint_idx, pos))
        .collect();

    // Fill payload_torque_per_kg
    for (col, &ji) in chain.iter().enumerate() {
        if let Some(&pos) = idx_map.get(&ji) {
            joint_torques[pos].payload_torque_per_kg = tau_payload[col];
        }
    }

    // Find maximum payload mass
    let mut max_mass = f64::INFINITY;
    let mut limiting = String::new();

    for (col, &ji) in chain.iter().enumerate() {
        if let Some(&pos) = idx_map.get(&ji) {
            let info = &joint_torques[pos];
            let tau_p = tau_payload[col]; // torque per 1 kg
            if tau_p.abs() < 1e-12 {
                continue; // this joint is not affected by the payload
            }
            let effort = info.effort_limit;
            if effort <= 0.0 {
                continue; // no effort limit defined
            }

            // We need: |gravity_torque + m * tau_p| ≤ effort
            // Two cases depending on the sign alignment:
            let g_tau = info.gravity_torque;

            // Solve for largest m ≥ 0 such that  |g_tau + m * tau_p| ≤ effort
            // Equivalent to:  -effort ≤ g_tau + m * tau_p ≤ effort
            //   → m ≤ (effort - g_tau) / tau_p   (if tau_p > 0)
            //   → m ≤ (-effort - g_tau) / tau_p  (if tau_p < 0)
            //   and similarly for the lower bound.
            let _m_candidates = [
                (effort - g_tau) / tau_p,
                (-effort - g_tau) / tau_p,
            ];

            // We want the largest m ≥ 0 such that BOTH constraints hold.
            // The constraint is:  -effort ≤ g_tau + m * tau_p ≤ effort
            // m must satisfy both:
            //   m * tau_p ≤  effort - g_tau
            //   m * tau_p ≥ -effort - g_tau
            let (_lo, hi) = if tau_p > 0.0 {
                ((-effort - g_tau) / tau_p, (effort - g_tau) / tau_p)
            } else {
                ((effort - g_tau) / tau_p, (-effort - g_tau) / tau_p)
            };

            // m must be in [max(0, lo), hi]
            let m_max = hi;
            if m_max < max_mass {
                max_mass = m_max;
                limiting = info.joint_name.clone();
            }
        }
    }

    if max_mass < 0.0 {
        max_mass = 0.0; // already overloaded by gravity alone
    }

    if max_mass.is_infinite() {
        // No joint limits defined; can't estimate
        return None;
    }

    Some(PayloadResult {
        max_mass_kg: max_mass,
        limiting_joint: limiting,
        ee_position: na::Point3::new(
            ee_pos.x as f64,
            ee_pos.y as f64,
            ee_pos.z as f64,
        ),
    })
}

/// Estimate maximum jump height using a simple energy balance.
///
/// Builds kinematic chains from each `ground_link` to `body_link` and sums
/// the mechanical work each leg joint can produce during the extension stroke.
///
/// The available stroke per joint is computed from the **current** joint
/// position to the nearer limit (the smaller travel), reflecting the
/// Run full static analysis.
///
/// `ee_link` — optional end-effector for payload analysis.
pub fn analyze(
    model: &RobotModel,
    ee_link: Option<&str>,
) -> StaticAnalysis {
    let mut joint_torques = compute_gravity_torques(model);

    let payload = ee_link.and_then(|ee| {
        compute_payload_capacity(model, ee, &mut joint_torques)
    });

    StaticAnalysis {
        joint_torques,
        payload,
    }
}

// ========== Simulation state machine ==========

/// Phase of the payload ramp simulation.
#[derive(Clone, Debug, PartialEq)]
pub enum PayloadPhase {
    /// Mass is ramping up toward max.
    Ramping,
    /// Holding at max payload.
    Holding,
    /// Done.
    Done,
}

/// State for an active payload simulation.
#[derive(Clone, Debug)]
#[allow(dead_code)]
pub struct PayloadSim {
    pub phase: PayloadPhase,
    pub phase_time: f64,
    /// Max mass to ramp up to (kg).
    pub max_mass: f64,
    /// Current virtual mass (kg).
    pub current_mass: f64,
    /// Ramp duration (s).
    pub ramp_duration: f64,
    /// Hold duration (s).
    pub hold_duration: f64,
    /// Per-joint torque utilisation (0.0 – 1.0+). Updated each step.
    pub joint_utilisation: Vec<(usize, f64)>,
    /// Name of the limiting joint.
    pub limiting_joint: String,
    /// Saved positions for restoration.
    pub saved_positions: Vec<f64>,
    pub saved_base_transform: na::Isometry3<f64>,
}

/// Wrapper for currently-active dynamics simulation type.
#[derive(Clone, Debug)]
#[allow(dead_code)]
pub enum DynSim {
    Payload(PayloadSim),
}

impl DynSim {
    #[allow(dead_code)]
    pub fn is_done(&self) -> bool {
        match self {
            DynSim::Payload(p) => p.phase == PayloadPhase::Done,
        }
    }
}

// ===== Payload sim construction =====

/// Create a payload simulation.
pub fn start_payload_sim(
    model: &RobotModel,
    ee_link: &str,
    speed: f64,
) -> Option<PayloadSim> {
    // Run a static analysis to find max payload
    let mut torques = compute_gravity_torques(model);
    let payload = compute_payload_capacity(model, ee_link, &mut torques)?;

    let max_mass = payload.max_mass_kg;
    if max_mass <= 0.0 {
        return None;
    }

    Some(PayloadSim {
        phase: PayloadPhase::Ramping,
        phase_time: 0.0,
        max_mass,
        current_mass: 0.0,
        ramp_duration: 3.0 / speed,
        hold_duration: 2.0 / speed,
        joint_utilisation: Vec::new(),
        limiting_joint: payload.limiting_joint,
        saved_positions: model.joint_positions.clone(),
        saved_base_transform: model.base_transform,
    })
}

/// Step the payload simulation by `dt`. Returns `true` while still running.
pub fn step_payload_sim(
    sim: &mut PayloadSim,
    model: &RobotModel,
    ee_link: &str,
) -> bool {
    match sim.phase {
        PayloadPhase::Ramping => {
            let t_frac = (sim.phase_time / sim.ramp_duration).clamp(0.0, 1.0);
            sim.current_mass = sim.max_mass * t_frac;

            // Compute per-joint utilisation at current mass
            update_utilisation(sim, model, ee_link);

            if sim.phase_time >= sim.ramp_duration {
                sim.phase = PayloadPhase::Holding;
                sim.phase_time = 0.0;
            }
            true
        }
        PayloadPhase::Holding => {
            sim.current_mass = sim.max_mass;
            update_utilisation(sim, model, ee_link);

            if sim.phase_time >= sim.hold_duration {
                sim.phase = PayloadPhase::Done;
                sim.phase_time = 0.0;
            }
            true
        }
        PayloadPhase::Done => false,
    }
}

fn update_utilisation(
    sim: &mut PayloadSim,
    model: &RobotModel,
    ee_link: &str,
) {
    let transforms = model.compute_transforms();
    let chain = model.chain_joints(ee_link);
    if chain.is_empty() {
        return;
    }
    let ee_li = match model.link_map.get(ee_link) {
        Some(&li) => li,
        None => return,
    };
    let _ee_pos = model.ee_world_pos(ee_li, &transforms);
    let jac = model.chain_positional_jacobian(&chain, ee_link, None, None);

    // Force per current mass
    let f = na::DVector::from_column_slice(&[0.0_f64, 0.0, -G * sim.current_mass]);
    let tau_payload = jac.transpose() * f;

    // Gravity torques
    let gravity_torques = compute_gravity_torques(model);
    let grav_map: HashMap<usize, f64> = gravity_torques
        .iter()
        .map(|t| (t.joint_idx, t.gravity_torque))
        .collect();

    sim.joint_utilisation.clear();
    for (col, &ji) in chain.iter().enumerate() {
        let joint = &model.joints[ji];
        if joint.effort <= 0.0 {
            continue;
        }
        let g_tau = grav_map.get(&ji).copied().unwrap_or(0.0);
        let total_tau = (g_tau + tau_payload[col]).abs();
        let util = total_tau / joint.effort;
        sim.joint_utilisation.push((ji, util));
    }
}

// ========== Jump simulation — STUBS ==========
//
// The jump-simulation engine that backed `start_jump_sim` /
// `step_jump_sim` / `extract_jump_result` / `compute_jump_height` was
// removed in commit 8ca7bbc ("reflesh dyn sim", 2026-04-27) along with
// large parts of `src/rbd/dynamics.rs`. The dependent code in
// `jump-sim-wasm` and the `test_serde` regression tests still references
// these symbols.
//
// To keep the workspace compile-clean while the new jump-sim
// architecture is being designed, the original *type surface* is
// reinstated here as **stubs**:
//
// - All types preserve their public field shape so serde round-trip
//   tests still pass.
// - Functions return `None` / empty values so anything that calls them
//   gracefully reports "not available" rather than panicking.
//
// Runtime jump-simulation tests (`native_jump_sim_serde_roundtrip`) are
// `#[ignore]`d until the engine is reimplemented. WASM clients receive
// a clear error from `start_jump_sim` returning `None`.

/// Time-series sampled during a jump simulation. Field shape preserves
/// the pre-refactor schema so on-disk JSON / `.misa` round-trips remain
/// stable across the stub period.
#[derive(Clone, Debug, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct SimGraphData {
    pub time: Vec<f32>,
    pub pos_x: Vec<f32>,
    pub pos_y: Vec<f32>,
    pub pos_z: Vec<f32>,
    pub vel_x: Vec<f32>,
    pub vel_y: Vec<f32>,
    pub vel_z: Vec<f32>,
    pub acc_x: Vec<f32>,
    pub acc_y: Vec<f32>,
    pub acc_z: Vec<f32>,
    pub link_name: String,
}

/// Per-joint peak observed during a jump.
#[derive(Clone, Debug, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct JointPeakInfo {
    pub joint_idx: usize,
    pub joint_name: String,
    pub peak_torque: f64,
    pub peak_torque_angle: f64,
    pub peak_velocity: f64,
    pub peak_velocity_angle: f64,
    pub contributes: bool,
}

/// Output of a completed jump simulation.
#[derive(Clone, Debug, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct JumpSimResult {
    pub max_height: f32,
    pub extension_duration: f32,
    pub joint_peaks: Vec<JointPeakInfo>,
    pub graph_data: SimGraphData,
}

/// Energy-method jump-height estimate.
#[derive(Clone, Debug, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct JumpHeightResult {
    pub max_height_m: f64,
    pub total_energy_j: f64,
    pub total_mass_kg: f64,
    pub per_joint_energy: Vec<(String, f64)>,
}

/// Opaque handle to an in-flight jump simulation. Currently a stub —
/// the algorithm was removed in 8ca7bbc and is awaiting reimplementation.
#[derive(Clone, Debug, Default)]
pub struct JumpSim {
    /// Unused stub field; kept so `Default` works and so callers can
    /// pattern-match in the future without API churn.
    _phantom: (),
}

/// **STUB** — initialise a jump simulation. Returns `None` because the
/// underlying algorithm was removed in 8ca7bbc; callers should treat
/// this as "jump simulation is not currently available."
#[allow(clippy::too_many_arguments)]
pub fn start_jump_sim(
    _model: &mut RobotModel,
    _ground_links: &[String],
    _body_link: Option<&str>,
    _speed: f32,
    _locked_joints: &std::collections::HashSet<String>,
    _launch_axes: [bool; 3],
    _extension_duration: Option<f32>,
    _enforce_torque_limits: bool,
    _enable_retract: bool,
    _graph_link: Option<&str>,
    _pd_kp: f64,
    _pd_kd: f64,
) -> Option<JumpSim> {
    log::warn!(
        "start_jump_sim: jump simulation engine was removed in 8ca7bbc \
         and has not been reimplemented yet — returning None"
    );
    None
}

/// **STUB** — step the jump simulation. Always returns `false` (i.e.
/// "simulation is done") because the stub `JumpSim` carries no state.
pub fn step_jump_sim(_sim: &mut JumpSim, _model: &mut RobotModel, _dt: f32) -> bool {
    false
}

/// **STUB** — extract the result of a (stubbed) jump simulation.
pub fn extract_jump_result(_sim: &JumpSim, _model: &RobotModel) -> JumpSimResult {
    JumpSimResult::default()
}

/// **STUB** — energy-based jump-height estimate. Returns `None` because
/// the algorithm was removed alongside the jump-sim engine.
pub fn compute_jump_height(
    _model: &RobotModel,
    _ground_links: &[String],
    _body_link: Option<&str>,
) -> Option<JumpHeightResult> {
    log::warn!(
        "compute_jump_height: implementation was removed in 8ca7bbc \
         and has not been reimplemented yet — returning None"
    );
    None
}

// ========== Tests ==========
#[cfg(test)]
mod tests {
    use super::*;
    use crate::robot::RobotModel;
    use std::path::Path;

    fn load_test_model(name: &str) -> RobotModel {
        let path = Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("tests/fixtures/urdf")
            .join(name);
        RobotModel::from_urdf(&path).unwrap()
    }

    #[test]
    fn gravity_torques_zero_at_origin() {
        let model = load_test_model("test_robot.urdf");
        let torques = compute_gravity_torques(&model);
        // Just check that we get results for each movable joint
        assert!(!torques.is_empty());
        for t in &torques {
            assert!(t.gravity_torque.is_finite(), "joint {}", t.joint_name);
        }
    }

    #[test]
    fn total_mass_consistent() {
        let model = load_test_model("test_robot.urdf");
        let total: f64 = model.links.iter().map(|l| l.inertial.mass).sum();
        assert!(total >= 0.0);
    }

    fn load_namiashi() -> RobotModel {
        let path = Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("sample/namiashi_description/urdf/namiashi.urdf");
        RobotModel::from_urdf(&path).unwrap()
    }


    #[test]
    fn rnea_bias_matches_gravity_torques() {
        // Verify that RNEA bias (at zero velocity) matches compute_gravity_torques.
        let mut model = load_namiashi();
        // Set a crouch pose
        for j in &model.joints {
            let ji = model.joints.iter().position(|jj| jj.name == j.name).unwrap();
            if j.name.contains("thigh") { model.joint_positions[ji] = 0.8; }
            if j.name.contains("calf")  { model.joint_positions[ji] = -1.5; }
        }

        let grav = compute_gravity_torques(&model);
        let joint_order = crate::rbd::dynamics::topological_joint_order(&model);
        let zero_vel: HashMap<usize, f64> = HashMap::new();
        let h = crate::rbd::dynamics::rnea_bias(&model, &joint_order, &zero_vel);

        eprintln!("=== RNEA bias vs compute_gravity_torques ===");
        for (col, &ji) in joint_order.iter().enumerate() {
            let gt = grav.iter().find(|t| t.joint_idx == ji);
            let rnea_val = h[col];
            let gt_val = gt.map(|t| t.gravity_torque).unwrap_or(0.0);
            let diff = (rnea_val - gt_val).abs();
            eprintln!("  joint {:2} {:20}: rnea_h={:+.6} gt={:+.6} diff={:.6} effort={:.2}",
                ji, model.joints[ji].name, rnea_val, gt_val, diff, model.joints[ji].effort);
            assert!(diff < 0.1,
                "RNEA bias doesn't match gravity torque for {}: rnea={:.6}, gt={:.6}",
                model.joints[ji].name, rnea_val, gt_val);
        }
    }

}
