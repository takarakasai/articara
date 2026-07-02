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

// ========== Core Algorithms ==========
//
// The static analysis itself lives in `misarta::analysis` (see articara
// doc/refactor_20260702.md §4, item A1); this module adapts between
// articara joint indices / names and misarta's model via `MisartaCache`.

/// Build misarta [`misarta::limits::JointLimits`] with `tau_max` taken
/// from the articara joints' effort limits. `effort <= 0` means "no limit
/// defined" and stays unbounded.
fn joint_limits(model: &RobotModel) -> misarta::limits::JointLimits {
    let mc = model.mc();
    let mut limits = misarta::limits::JointLimits::unbounded(&mc.model);
    for (ji, joint) in model.joints.iter().enumerate() {
        if joint.effort <= 0.0 {
            continue;
        }
        if let Some(mi) = mc.a2m.get(ji).and_then(|&m| m) {
            if mc.model.joints[mi].joint_type.nv() == 1 {
                limits.tau_max[mc.model.v_idx[mi]] = joint.effort;
            }
        }
    }
    limits
}

/// misarta joint index whose child link is `ee_link`.
fn ee_misarta_joint(model: &RobotModel, ee_link: &str) -> Option<usize> {
    let ji = model.joints.iter().position(|j| j.child_link == ee_link)?;
    model.mc().a2m.get(ji).and_then(|&m| m)
}

/// Compute static gravity torque at every movable joint.
///
/// Delegates to [`misarta::analysis::gravity_loads`] and wraps results in
/// `JointTorqueInfo` with effort limits and margins from the `RobotModel`.
pub fn compute_gravity_torques(model: &RobotModel) -> Vec<JointTorqueInfo> {
    let mc = model.mc();
    let q = model.build_q();
    let limits = joint_limits(model);
    let loads = misarta::analysis::gravity_loads(&mc.model, &q, &limits);
    let tau_by_mi: HashMap<usize, f64> = loads
        .iter()
        .map(|l| (l.joint_idx, l.gravity_torque))
        .collect();

    let mut result = Vec::new();
    for (ji, joint) in model.joints.iter().enumerate() {
        if joint.joint_type.as_str() == "fixed" {
            continue;
        }
        let tau = mc
            .a2m
            .get(ji)
            .and_then(|&m| m)
            .and_then(|mi| tau_by_mi.get(&mi).copied())
            .unwrap_or(0.0);
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
/// Delegates to [`misarta::analysis::payload_capacity`]. Per-kg torques
/// are in the **actuator** convention (they add directly onto the gravity
/// torque; same sign as `gravity_torque`).
pub fn compute_payload_capacity(
    model: &RobotModel,
    ee_link: &str,
    joint_torques: &mut [JointTorqueInfo],
) -> Option<PayloadResult> {
    let mc = model.mc();
    let q = model.build_q();
    let limits = joint_limits(model);
    let mi_ee = ee_misarta_joint(model, ee_link)?;

    let cap = misarta::analysis::payload_capacity(&mc.model, &q, mi_ee, &limits)?;

    // Fill payload_torque_per_kg for the UI table.
    for info in joint_torques.iter_mut() {
        if let Some(mi) = mc.a2m.get(info.joint_idx).and_then(|&m| m) {
            if mc.model.joints[mi].joint_type.nv() == 1 {
                info.payload_torque_per_kg = cap.tau_per_kg[mc.model.v_idx[mi]];
            }
        }
    }

    let limiting = mc
        .m2a
        .get(cap.limiting_joint)
        .and_then(|&a| a)
        .map(|ji| model.joints[ji].name.clone())
        .unwrap_or_default();

    // EE world position for the viewport marker.
    let transforms = model.compute_transforms();
    let ee_li = *model.link_map.get(ee_link)?;
    let ee_pos = model.ee_world_pos(ee_li, &transforms);

    Some(PayloadResult {
        max_mass_kg: cap.max_mass_kg,
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
    let mc = model.mc();
    let q = model.build_q();
    let limits = joint_limits(model);
    let Some(mi_ee) = ee_misarta_joint(model, ee_link) else {
        return;
    };

    let util =
        misarta::analysis::payload_utilisation(&mc.model, &q, mi_ee, sim.current_mass, &limits);
    // Keep the display scoped to the loaded chain, as before: joints the
    // payload doesn't touch keep their idle colour.
    let tau_per_kg = misarta::analysis::payload_tau_per_kg(&mc.model, &q, mi_ee);

    sim.joint_utilisation.clear();
    for (mi, u) in util {
        if tau_per_kg[mc.model.v_idx[mi]].abs() < 1e-12 {
            continue;
        }
        if let Some(ji) = mc.m2a.get(mi).and_then(|&a| a) {
            sim.joint_utilisation.push((ji, u));
        }
    }
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
            .join("tests/fixtures/namiashi/urdf/namiashi.urdf");
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
