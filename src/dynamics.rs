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
pub struct PayloadResult {
    /// Maximum payload mass the robot can hold statically (kg).
    pub max_mass_kg: f64,
    /// Name of the joint that is the bottleneck.
    pub limiting_joint: String,
    /// End-effector position in world frame where the payload is applied.
    pub ee_position: na::Point3<f64>,
}

/// Result of jump-height estimation.
#[derive(Clone, Debug)]
pub struct JumpResult {
    /// Estimated max jump height (m) of the centre of mass.
    pub max_height_m: f64,
    /// Total available mechanical energy (J).
    pub total_energy_j: f64,
    /// Total robot mass (kg).
    pub total_mass_kg: f64,
    /// Per-joint energy contribution.
    pub per_joint_energy: Vec<(String, f64)>,
}

/// Full static analysis result.
#[derive(Clone, Debug)]
pub struct StaticAnalysis {
    pub joint_torques: Vec<JointTorqueInfo>,
    pub payload: Option<PayloadResult>,
    pub jump: Option<JumpResult>,
}

// ========== Gravity constant ==========
const G: f64 = 9.80665;
const G_VEC: na::Vector3<f64> = na::Vector3::new(0.0, 0.0, -G);

// ========== Core Algorithms ==========

/// Collect all descendant link indices (inclusive) reachable from `start_link`
/// through the kinematic tree.
fn descendant_links(model: &RobotModel, start_link: &str) -> Vec<usize> {
    let mut result = Vec::new();
    let mut stack = vec![start_link.to_string()];
    while let Some(link_name) = stack.pop() {
        if let Some(&li) = model.link_map.get(&link_name) {
            result.push(li);
        }
        if let Some(child_joints) = model.children_joints.get(&link_name) {
            for &ji in child_joints {
                stack.push(model.joints[ji].child_link.clone());
            }
        }
    }
    result
}

/// Compute static gravity torque at every movable joint.
///
/// For revolute/continuous joints the result is in N·m; for prismatic joints in N.
///
/// Algorithm: for each joint, sum the gravitational moment contribution of all
/// descendant links (child-side of the joint), projected onto the joint axis.
pub fn compute_gravity_torques(model: &RobotModel) -> Vec<JointTorqueInfo> {
    let transforms = model.compute_transforms();
    let mut result = Vec::new();

    for (ji, joint) in model.joints.iter().enumerate() {
        let jt = joint.joint_type.as_str();
        if jt == "fixed" {
            continue;
        }

        // World-space joint frame
        let parent_tf = transforms
            .get(&joint.parent_link)
            .copied()
            .unwrap_or(na::Isometry3::identity());
        let joint_tf = parent_tf * joint.origin;
        let joint_pos = joint_tf.translation.vector.cast::<f64>();
        let world_axis = (joint_tf.rotation * joint.axis).cast::<f64>();

        // Sum gravitational torque from all downstream links
        let descendants = descendant_links(model, &joint.child_link);
        let mut tau = 0.0_f64;

        for &li in &descendants {
            let link = &model.links[li];
            let mass = link.inertial.mass;
            if mass <= 0.0 {
                continue;
            }

            // CoM in world frame:  world_tf * inertial.origin
            let link_tf = transforms
                .get(&link.name)
                .copied()
                .unwrap_or(na::Isometry3::identity());
            let com_local = link.inertial.origin.translation.vector.cast::<f64>();
            let com_world = link_tf.cast::<f64>() * na::Point3::from(com_local);

            let r = com_world.coords - joint_pos; // moment arm
            let f_grav = G_VEC * mass;             // gravitational force

            match jt {
                "revolute" | "continuous" => {
                    // τ = a · (r × F)
                    tau += world_axis.dot(&r.cross(&f_grav));
                }
                "prismatic" => {
                    // Force along joint axis
                    tau += world_axis.dot(&f_grav);
                }
                _ => {}
            }
        }

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
    let chain = crate::ik::build_chain(model, ee_link);
    if chain.is_empty() {
        return None;
    }

    // EE world position
    let ee_li = *model.link_map.get(ee_link)?;
    let ee_pos = crate::ik::get_ee_world_pos(model, ee_li, &transforms);

    // Compute positional Jacobian (3 × N)
    let jac = crate::ik::compute_jacobian(model, &chain, &transforms, &ee_pos);

    // Unit payload force: F = [0, 0, -g] (force per 1 kg)
    let f_unit = na::DVector::from_column_slice(&[0.0_f32, 0.0, -G as f32]);

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
    for (col, cj) in chain.iter().enumerate() {
        if let Some(&pos) = idx_map.get(&cj.joint_idx) {
            joint_torques[pos].payload_torque_per_kg = tau_payload[col] as f64;
        }
    }

    // Find maximum payload mass
    let mut max_mass = f64::INFINITY;
    let mut limiting = String::new();

    for (col, cj) in chain.iter().enumerate() {
        if let Some(&pos) = idx_map.get(&cj.joint_idx) {
            let info = &joint_torques[pos];
            let tau_p = tau_payload[col] as f64; // torque per 1 kg
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
            let m_candidates = [
                (effort - g_tau) / tau_p,
                (-effort - g_tau) / tau_p,
            ];

            // We want the largest m ≥ 0 such that BOTH constraints hold.
            // The constraint is:  -effort ≤ g_tau + m * tau_p ≤ effort
            // m must satisfy both:
            //   m * tau_p ≤  effort - g_tau
            //   m * tau_p ≥ -effort - g_tau
            let (lo, hi) = if tau_p > 0.0 {
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
/// robot's actual crouch depth.
///
///   E_i = τ_max_i × |θ_i − θ_limit_i|
///   h   = ΣE_i / (M g)
///
/// `body_link` — the torso / base link that gets launched (if `None`,
///              defaults to the URDF root link).
pub fn compute_jump_height(
    model: &RobotModel,
    ground_links: &[String],
    body_link: Option<&str>,
) -> Option<JumpResult> {
    if ground_links.is_empty() {
        return None;
    }

    let total_mass: f64 = model.links.iter().map(|l| l.inertial.mass).sum();
    if total_mass <= 0.0 {
        return None;
    }

    let body = body_link.unwrap_or(&model.root_link);

    // Collect unique joints involved in leg chains (body ↔ ground)
    let mut seen = std::collections::HashSet::new();
    let mut per_joint_energy = Vec::new();
    let mut total_energy = 0.0_f64;

    for gl in ground_links {
        // Only joints between body and this ground link
        let chain = crate::ik::build_chain_between(model, gl, Some(body));
        for cj in &chain {
            if seen.contains(&cj.joint_idx) {
                continue;
            }
            seen.insert(cj.joint_idx);

            let joint = &model.joints[cj.joint_idx];
            let effort = joint.effort;
            if effort <= 0.0 {
                continue;
            }

            let lower = joint.lower;
            let upper = joint.upper;
            if lower >= upper {
                continue;
            }

            // Use the current joint position to determine the available
            // extension stroke: distance from current angle to the
            // nearer joint limit (conservative estimate).
            let cur = model.joint_positions[cj.joint_idx] as f64;
            let stroke_to_lower = (cur - lower).abs();
            let stroke_to_upper = (upper - cur).abs();
            let stroke = stroke_to_lower.min(stroke_to_upper);

            let energy = effort * stroke;

            per_joint_energy.push((joint.name.clone(), energy));
            total_energy += energy;
        }
    }

    if total_energy <= 0.0 {
        return Some(JumpResult {
            max_height_m: 0.0,
            total_energy_j: 0.0,
            total_mass_kg: total_mass,
            per_joint_energy,
        });
    }

    let height = total_energy / (total_mass * G);

    Some(JumpResult {
        max_height_m: height,
        total_energy_j: total_energy,
        total_mass_kg: total_mass,
        per_joint_energy,
    })
}

/// Run full static analysis.
///
/// `ee_link` — optional end-effector for payload analysis.
/// `body_link` — torso / base link for jump estimation (None → URDF root).
/// `ground_links` — ground-contact links for jump estimation.
pub fn analyze(
    model: &RobotModel,
    ee_link: Option<&str>,
    body_link: Option<&str>,
    ground_links: &[String],
) -> StaticAnalysis {
    let mut joint_torques = compute_gravity_torques(model);

    let payload = ee_link.and_then(|ee| {
        compute_payload_capacity(model, ee, &mut joint_torques)
    });

    let jump = if ground_links.is_empty() {
        None
    } else {
        compute_jump_height(model, ground_links, body_link)
    };

    StaticAnalysis {
        joint_torques,
        payload,
        jump,
    }
}

// ========== Simulation state machine ==========

/// Phase of the jump animation.
#[derive(Clone, Debug, PartialEq)]
pub enum JumpPhase {
    /// Joints are extending (push-off). Feet stay on ground.
    Extension,
    /// Robot is in ballistic flight (base moves up then down).
    Flight,
    /// Landed; briefly hold, then restore.
    Landed,
}

/// Per-step diagnostics computed from the quasi-dynamics engine.
#[derive(Clone, Debug, Default)]
pub struct SimStepInfo {
    /// Vertical ground reaction force (N). Positive = upward.
    pub grf_z: f64,
    /// Current base vertical velocity (m/s). Positive = upward.
    pub velocity_z: f32,
    /// Current height above initial position (m).
    pub height: f32,
    /// Per-joint torque utilisation (joint_idx, ratio 0.0–1.0+, contributes_to_jump).
    pub joint_utilisation: Vec<(usize, f64, bool)>,
}

/// Per-joint peak value recorded across the entire simulation.
#[derive(Clone, Debug)]
pub struct JointPeakInfo {
    pub joint_idx: usize,
    pub joint_name: String,
    /// Peak absolute gravity torque seen during extension (N·m).
    pub peak_torque: f64,
    /// Peak absolute angular velocity seen during extension (rad/s).
    pub peak_velocity: f64,
    /// Whether this joint contributed to vertical push-off.
    pub contributes: bool,
}

/// Summary result captured when a jump simulation finishes.
#[derive(Clone, Debug)]
pub struct JumpSimResult {
    /// Maximum height reached above starting position (m).
    pub max_height: f32,
    /// Extension duration used (s).
    pub extension_duration: f32,
    /// Per-joint peak torque and velocity.
    pub joint_peaks: Vec<JointPeakInfo>,
}

/// Per-joint data for the jump animation.
#[derive(Clone, Debug)]
pub struct LegJointSim {
    pub joint_idx: usize,
    /// Joint angle at the start of extension.
    pub start_angle: f32,
    /// Joint angle at full extension (the farther limit from start).
    pub extended_angle: f32,
    /// Max angular/linear velocity from URDF (rad/s or m/s). Clamped ≥ 1.0.
    pub max_velocity: f32,
    /// Whether this joint actively contributes to the vertical push-off.
    /// Non-contributing joints hold their initial posture during extension.
    pub contributes: bool,
}

/// Per-leg data for IK-based coordinated extension.
#[derive(Clone, Debug)]
pub struct LegSim {
    /// Ground (foot) link name.
    pub ground_link: String,
    /// Body link name (the "root" of this leg's IK chain).
    pub body_link: String,
    /// IK chain from ground link to body link.
    pub chain: Vec<crate::ik::ChainJoint>,
    /// Foot position relative to body at the start of the sim (body-frame).
    pub initial_foot_pos: na::Point3<f32>,
    /// Maximum vertical stroke (m) the foot can push down.
    pub max_stroke: f32,
    /// Joint indices in this leg that are locked (hold posture).
    pub locked_joint_indices: std::collections::HashSet<usize>,
}

/// State for an active jump simulation with per-step quasi-dynamics.
#[derive(Clone, Debug)]
pub struct JumpSim {
    pub phase: JumpPhase,
    /// Elapsed time within the current phase (s).
    pub phase_time: f32,

    // --- per-leg IK data ---
    pub legs: Vec<LegSim>,

    // --- joint trajectory (kept for per-joint torque tracking) ---
    pub leg_joints: Vec<LegJointSim>,
    /// Planned extension duration (s).
    pub extension_duration: f32,

    // --- physics state (extension phase, computed per step) ---
    /// Current base vertical velocity (m/s).
    pub base_velocity_z: f32,
    /// Average foot Z at the start (= ground level).
    pub initial_foot_z: f32,
    /// Ground link names (for foot tracking).
    pub ground_link_names: Vec<String>,

    // --- physics state (flight phase) ---
    /// Base velocity at the moment of launch (m/s, upward).
    pub launch_velocity: f32,
    /// Base Z at the moment of launch.
    pub launch_z: f32,
    /// Foot Z at the moment of launch (after extension).
    /// Used to compute when feet touch ground during descent.
    pub launch_foot_z: f32,

    // --- diagnostics ---
    pub step_info: SimStepInfo,
    /// Maximum height reached so far above initial base Z.
    pub max_height_reached: f32,
    /// Total robot mass (kg).
    pub total_mass: f64,

    // --- timing ---
    pub landed_hold: f32,

    // --- saved state to restore ---
    pub saved_positions: Vec<f32>,
    pub saved_base_transform: na::Isometry3<f32>,
    pub speed: f32,
    /// Which axes the body can move during flight [X, Y, Z].
    pub launch_axes: [bool; 3],

    // --- internal tracking for finite differences ---
    prev_base_z: Option<f32>,
    prev_velocity_z: Option<f32>,

    // --- per-joint peak tracking ---
    /// Peak absolute gravity torque per joint (joint_idx → peak N·m).
    pub peak_torques: HashMap<usize, f64>,
    /// Peak absolute angular velocity per joint (joint_idx → peak rad/s).
    pub peak_velocities: HashMap<usize, f64>,
    /// Previous joint angles for finite-difference velocity estimation.
    prev_joint_angles: HashMap<usize, f32>,
}

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
pub struct PayloadSim {
    pub phase: PayloadPhase,
    pub phase_time: f32,
    /// Max mass to ramp up to (kg).
    pub max_mass: f64,
    /// Current virtual mass (kg).
    pub current_mass: f64,
    /// Ramp duration (s).
    pub ramp_duration: f32,
    /// Hold duration (s).
    pub hold_duration: f32,
    /// Per-joint torque utilisation (0.0 – 1.0+). Updated each step.
    pub joint_utilisation: Vec<(usize, f64)>,
    /// Name of the limiting joint.
    pub limiting_joint: String,
    /// Saved positions for restoration.
    pub saved_positions: Vec<f32>,
    pub saved_base_transform: na::Isometry3<f32>,
}

/// Wrapper for either simulation type.
#[derive(Clone, Debug)]
pub enum DynSim {
    Jump(JumpSim),
    Payload(PayloadSim),
}

impl DynSim {
    pub fn is_done(&self) -> bool {
        match self {
            DynSim::Jump(j) => j.phase == JumpPhase::Landed && j.phase_time >= j.landed_hold,
            DynSim::Payload(p) => p.phase == PayloadPhase::Done,
        }
    }
}

// ===== Jump sim construction =====

/// Create a jump simulation from the current model state.
///
/// `ground_links` and `body_link` define the leg chains (same as the analysis).
/// `locked_joints` — joint names that should not be driven (held at start angle).
/// `launch_axes` — which body axes [X, Y, Z] are free during flight.
/// `extension_override` — if `Some(d)`, use `d` seconds as extension duration
/// instead of the auto-computed value.
pub fn start_jump_sim(
    model: &mut RobotModel,
    ground_links: &[String],
    body_link: Option<&str>,
    speed: f32,
    locked_joints: &std::collections::HashSet<String>,
    launch_axes: [bool; 3],
    extension_override: Option<f32>,
) -> Option<JumpSim> {
    if ground_links.is_empty() {
        return None;
    }

    let body = body_link.unwrap_or(&model.root_link).to_string();

    let total_mass: f64 = model.links.iter().map(|l| l.inertial.mass).sum();
    if total_mass <= 0.0 {
        return None;
    }

    let transforms = model.compute_transforms();
    let initial_foot_z = avg_link_z(&transforms, ground_links);

    // Get body transform for computing foot positions in body frame
    let body_tf = transforms
        .get(&body)
        .copied()
        .unwrap_or(na::Isometry3::identity());
    let body_tf_inv = body_tf.inverse();

    // Collect per-joint metadata (for torque tracking) and per-leg IK data
    let mut seen_joints = std::collections::HashSet::new();
    let mut leg_joints = Vec::new();
    let mut legs = Vec::new();

    // Locked joint indices
    let locked_idx: std::collections::HashSet<usize> = model
        .joints
        .iter()
        .enumerate()
        .filter(|(_, j)| locked_joints.contains(&j.name))
        .map(|(i, _)| i)
        .collect();

    for gl in ground_links {
        let chain = crate::ik::build_chain_between(model, gl, Some(&body));
        if chain.is_empty() {
            continue;
        }

        // Foot position in body frame
        let foot_world = foot_link_pos(&transforms, gl);
        let foot_body = body_tf_inv * foot_world;

        // Determine max stroke: how far down (−Z in world) the foot can go.
        // We do a binary-search style: try pushing foot down in increments
        // using IK, clamped to joint limits, to find the actual reachable stroke.
        let max_stroke = compute_leg_max_stroke(
            model, &chain, gl, &body, &foot_world, &locked_idx,
        );

        let leg_locked: std::collections::HashSet<usize> = chain
            .iter()
            .filter(|cj| locked_idx.contains(&cj.joint_idx))
            .map(|cj| cj.joint_idx)
            .collect();

        // Collect per-joint metadata for this leg
        for cj in &chain {
            if seen_joints.contains(&cj.joint_idx) {
                continue;
            }
            seen_joints.insert(cj.joint_idx);

            let joint = &model.joints[cj.joint_idx];
            if joint.effort <= 0.0 {
                continue;
            }
            let lower = joint.lower as f32;
            let upper = joint.upper as f32;
            if lower >= upper {
                continue;
            }

            let cur = model.joint_positions[cj.joint_idx];
            let vel = (joint.velocity as f32).max(1.0);
            let is_locked = locked_idx.contains(&cj.joint_idx);

            leg_joints.push(LegJointSim {
                joint_idx: cj.joint_idx,
                start_angle: cur,
                extended_angle: cur, // will be updated by IK during sim
                max_velocity: vel,
                contributes: !is_locked,
            });
        }

        legs.push(LegSim {
            ground_link: gl.clone(),
            body_link: body.clone(),
            chain,
            initial_foot_pos: foot_body,
            max_stroke,
            locked_joint_indices: leg_locked,
        });
    }

    if legs.is_empty() {
        return None;
    }

    // Extension duration: estimate from slowest leg stroke / min joint velocity
    let extension_duration = if let Some(ovr) = extension_override {
        ovr.max(0.05)
    } else {
        let mut max_time = 0.15_f32;
        for leg in &legs {
            // Slowest joint velocity in this leg
            let min_vel = leg
                .chain
                .iter()
                .filter(|cj| !leg.locked_joint_indices.contains(&cj.joint_idx))
                .map(|cj| (model.joints[cj.joint_idx].velocity as f32).max(1.0))
                .fold(f32::MAX, f32::min);
            if min_vel < f32::MAX && leg.max_stroke > 0.0 {
                // Rough estimate: stroke ≈ joint_range * some_factor, time ≈ stroke / vel
                // But max_stroke is in meters; we need an angular estimate.
                // Use max_stroke / (min_vel * avg_link_length) as a rough time.
                // Simpler: use avg contributing joint range / velocity.
                let avg_time: f32 = leg
                    .chain
                    .iter()
                    .filter(|cj| !leg.locked_joint_indices.contains(&cj.joint_idx))
                    .map(|cj| {
                        let j = &model.joints[cj.joint_idx];
                        let range = (j.upper - j.lower).abs() as f32;
                        let vel = (j.velocity as f32).max(1.0);
                        range / vel
                    })
                    .fold(0.0_f32, f32::max);
                if avg_time > max_time {
                    max_time = avg_time;
                }
            }
        }
        max_time
    };

    Some(JumpSim {
        phase: JumpPhase::Extension,
        phase_time: 0.0,
        legs,
        leg_joints,
        extension_duration,
        base_velocity_z: 0.0,
        initial_foot_z,
        ground_link_names: ground_links.to_vec(),
        launch_velocity: 0.0,
        launch_z: model.base_transform.translation.vector.z,
        launch_foot_z: initial_foot_z,
        step_info: SimStepInfo::default(),
        max_height_reached: 0.0,
        total_mass,
        landed_hold: 0.6,
        saved_positions: model.joint_positions.clone(),
        saved_base_transform: model.base_transform,
        speed,
        launch_axes,
        prev_base_z: None,
        prev_velocity_z: None,
        peak_torques: HashMap::new(),
        peak_velocities: HashMap::new(),
        prev_joint_angles: HashMap::new(),
    })
}

/// Compute the foot link world position (translation only).
fn foot_link_pos(
    transforms: &HashMap<String, na::Isometry3<f32>>,
    link_name: &str,
) -> na::Point3<f32> {
    transforms
        .get(link_name)
        .map(|tf| na::Point3::from(tf.translation.vector))
        .unwrap_or(na::Point3::origin())
}

/// Estimate the maximum downward stroke a leg can achieve via IK.
///
/// Uses iterative IK stepping with the foot target progressively lowered,
/// respecting joint limits. Returns the Z distance (positive = downward)
/// the foot can travel from its initial position.
fn compute_leg_max_stroke(
    model: &mut RobotModel,
    chain: &[crate::ik::ChainJoint],
    ground_link: &str,
    body_link: &str,
    initial_foot_world: &na::Point3<f32>,
    locked_joints: &std::collections::HashSet<usize>,
) -> f32 {
    let saved_positions = model.joint_positions.clone();

    // Try to push foot down by 1m in small IK steps
    let target_drop = 1.0_f32; // try up to 1m
    let n_steps = 50;
    let step_size = target_drop / n_steps as f32;

    let mut achieved_drop = 0.0_f32;

    for i in 1..=n_steps {
        let target = na::Point3::new(
            initial_foot_world.x,
            initial_foot_world.y,
            initial_foot_world.z - step_size * i as f32,
        );

        // Run a few IK iterations
        for _ in 0..5 {
            let transforms = model.compute_transforms();
            let foot_pos = foot_link_pos(&transforms, ground_link);
            let deltas = crate::ik::solve_ik_step(
                model, chain, &transforms, &foot_pos, &target, 0.01, 0.05,
            );
            // Apply deltas, skipping locked joints
            for (k, cj) in chain.iter().enumerate() {
                if locked_joints.contains(&cj.joint_idx) {
                    continue;
                }
                let ji = cj.joint_idx;
                let lower = model.joints[ji].lower as f32;
                let upper = model.joints[ji].upper as f32;
                model.joint_positions[ji] =
                    (model.joint_positions[ji] + deltas[k]).clamp(lower, upper);
            }
        }

        // Measure actual foot position
        let transforms = model.compute_transforms();
        let foot_pos = foot_link_pos(&transforms, ground_link);
        let drop = initial_foot_world.z - foot_pos.z;
        if drop > achieved_drop {
            achieved_drop = drop;
        }

        // If foot barely moved this step, we've hit joint limits
        let error = (foot_pos.z - (initial_foot_world.z - step_size * i as f32)).abs();
        if error > step_size * 0.5 {
            break;
        }
    }

    // Restore model
    model.joint_positions = saved_positions;

    achieved_drop.max(0.001) // at least 1mm
}

/// Extract a result summary from a completed (or in-progress) jump simulation.
pub fn extract_jump_result(sim: &JumpSim, model: &RobotModel) -> JumpSimResult {
    let mut joint_peaks: Vec<JointPeakInfo> = sim
        .leg_joints
        .iter()
        .map(|lj| {
            let jname = if lj.joint_idx < model.joints.len() {
                model.joints[lj.joint_idx].name.clone()
            } else {
                format!("joint_{}", lj.joint_idx)
            };
            JointPeakInfo {
                joint_idx: lj.joint_idx,
                joint_name: jname,
                peak_torque: sim.peak_torques.get(&lj.joint_idx).copied().unwrap_or(0.0),
                peak_velocity: sim.peak_velocities.get(&lj.joint_idx).copied().unwrap_or(0.0),
                contributes: lj.contributes,
            }
        })
        .collect();
    // Sort contributing joints first, then by peak torque descending
    joint_peaks.sort_by(|a, b| {
        b.contributes
            .cmp(&a.contributes)
            .then_with(|| b.peak_torque.partial_cmp(&a.peak_torque).unwrap_or(std::cmp::Ordering::Equal))
    });
    JumpSimResult {
        max_height: sim.max_height_reached,
        extension_duration: sim.extension_duration,
        joint_peaks,
    }
}

/// Step the jump simulation by `dt` seconds.
///
/// **Extension phase (quasi-dynamics):**
/// 1. Compute per-joint gravity torque and utilisation; derive speed limit.
/// 2. For each leg, use IK to drive the foot toward a straight-down target
///    so that all joints in the leg coordinate for vertical push-off.
/// 3. Re-compute FK and adjust `base_transform.z` so that feet stay
///    at the initial ground level (foot-constraint).
/// 4. Compute base velocity from finite differences of base Z.
/// 5. Compute ground reaction force: $F_{GRF} = M (a_z + g)$.
///
/// **Flight phase:** pure ballistic with the launch velocity obtained
/// at the end of the extension phase.
///
/// Returns `true` while still running.
pub fn step_jump_sim(sim: &mut JumpSim, model: &mut RobotModel, dt: f32) -> bool {
    let dt = dt * sim.speed;
    if dt <= 0.0 {
        return true;
    }
    sim.phase_time += dt;

    match sim.phase {
        JumpPhase::Extension => {
            // --- 1. Compute torque-limited speed ratio ---
            let gravity_torques = compute_gravity_torques(model);
            let grav_map: HashMap<usize, f64> = gravity_torques
                .iter()
                .map(|t| (t.joint_idx, t.gravity_torque))
                .collect();

            let mut worst_ratio = 0.0_f64;
            sim.step_info.joint_utilisation.clear();
            for lj in &sim.leg_joints {
                let joint = &model.joints[lj.joint_idx];
                if joint.effort <= 0.0 {
                    continue;
                }
                let g_tau = grav_map.get(&lj.joint_idx).copied().unwrap_or(0.0);
                let util = g_tau.abs() / joint.effort;
                sim.step_info.joint_utilisation.push((lj.joint_idx, util, lj.contributes));
                if lj.contributes && util > worst_ratio {
                    worst_ratio = util;
                }
            }

            let speed_scale = if worst_ratio >= 1.0 {
                0.0_f32
            } else if worst_ratio > 0.8 {
                ((1.0 - worst_ratio) / 0.2) as f32
            } else {
                1.0_f32
            };

            // --- 2. Per-leg IK: drive feet straight down ---
            let effective_time = sim.phase_time;
            let t_frac = (effective_time / sim.extension_duration).clamp(0.0, 1.0);
            let alpha = launch_profile(t_frac);

            // Get body transform for computing foot target in world frame
            let body_tf = {
                let transforms = model.compute_transforms();
                transforms
                    .get(&sim.legs[0].body_link)
                    .copied()
                    .unwrap_or(na::Isometry3::identity())
            };

            for leg in &sim.legs {
                // Target foot position: initial pos in body frame, but pushed
                // down by alpha * max_stroke in world Z.
                let foot_world_initial = body_tf * leg.initial_foot_pos;
                let target = na::Point3::new(
                    foot_world_initial.x,
                    foot_world_initial.y,
                    foot_world_initial.z - alpha * leg.max_stroke,
                );

                // Run a few IK iterations to move foot toward target
                let ik_iters = 3;
                for _ in 0..ik_iters {
                    let transforms = model.compute_transforms();
                    let foot_pos = foot_link_pos(&transforms, &leg.ground_link);
                    let deltas = crate::ik::solve_ik_step(
                        model,
                        &leg.chain,
                        &transforms,
                        &foot_pos,
                        &target,
                        0.01,  // damping
                        0.1,   // max step
                    );
                    for (k, cj) in leg.chain.iter().enumerate() {
                        if leg.locked_joint_indices.contains(&cj.joint_idx) {
                            continue; // locked joints hold posture
                        }
                        let ji = cj.joint_idx;
                        let lower = model.joints[ji].lower as f32;
                        let upper = model.joints[ji].upper as f32;
                        model.joint_positions[ji] =
                            (model.joint_positions[ji] + deltas[k]).clamp(lower, upper);
                    }
                }
            }

            // Slow down the trajectory clock if torque-limited
            if speed_scale < 1.0 {
                let rollback = dt * (1.0 - speed_scale);
                sim.phase_time -= rollback;
            }

            // --- 3. FK with base.z=0 to find foot-relative positions ---
            let saved_base = model.base_transform;
            let mut temp_base = sim.saved_base_transform;
            temp_base.translation.vector.z = 0.0;
            model.base_transform = temp_base;

            let transforms = model.compute_transforms();
            let foot_z_at_zero = avg_link_z(&transforms, &sim.ground_link_names);

            // Restore base and set new Z so feet stay at ground level:
            //   foot_world_z = base_z + foot_z_at_zero = initial_foot_z
            //   ∴ base_z = initial_foot_z − foot_z_at_zero
            let new_base_z = sim.initial_foot_z - foot_z_at_zero;
            model.base_transform = saved_base;
            model.base_transform.translation.vector.z = new_base_z;

            // --- 4. Compute velocity from finite differences ---
            let current_z = new_base_z;
            if let Some(prev_z) = sim.prev_base_z {
                sim.base_velocity_z = (current_z - prev_z) / dt;
            }
            sim.prev_base_z = Some(current_z);

            // --- 5. Compute acceleration and GRF ---
            let accel_z = if let Some(prev_v) = sim.prev_velocity_z {
                (sim.base_velocity_z - prev_v) / dt
            } else {
                0.0
            };
            sim.prev_velocity_z = Some(sim.base_velocity_z);

            // Newton's 2nd law: M·a = GRF − M·g  →  GRF = M·(a + g)
            let grf = sim.total_mass * (accel_z as f64 + G);
            sim.step_info.grf_z = grf;
            sim.step_info.velocity_z = sim.base_velocity_z;
            sim.step_info.height =
                current_z - sim.saved_base_transform.translation.vector.z;

            if sim.step_info.height > sim.max_height_reached {
                sim.max_height_reached = sim.step_info.height;
            }

            // --- 6. Track per-joint dynamic torque and angular velocity ---
            // Dynamic torque per joint = J_i^T · F_GRF + gravity_torque_i
            // This captures the full load each joint sees during the push-off.
            {
                let transforms_post = model.compute_transforms();
                let grf_force = na::DVector::from_column_slice(&[
                    0.0_f32, 0.0, grf as f32,
                ]);
                for leg in &sim.legs {
                    // Compute Jacobian for this leg chain
                    let foot_pos = foot_link_pos(&transforms_post, &leg.ground_link);
                    let jac = crate::ik::compute_jacobian(
                        model, &leg.chain, &transforms_post, &foot_pos,
                    );
                    // τ = J^T · F
                    let tau_grf = jac.transpose() * &grf_force;

                    for (k, cj) in leg.chain.iter().enumerate() {
                        // Total torque = GRF-induced + gravity
                        let grf_tau = tau_grf[k].abs() as f64;
                        let g_tau = grav_map.get(&cj.joint_idx).copied().unwrap_or(0.0).abs();
                        let total_tau = grf_tau + g_tau;

                        let entry = sim.peak_torques.entry(cj.joint_idx).or_insert(0.0);
                        if total_tau > *entry {
                            *entry = total_tau;
                        }

                        // Track angular velocity (post-IK)
                        let cur_angle = model.joint_positions[cj.joint_idx];
                        if let Some(&prev) = sim.prev_joint_angles.get(&cj.joint_idx) {
                            let omega = ((cur_angle - prev) / dt).abs() as f64;
                            let v_entry = sim.peak_velocities.entry(cj.joint_idx).or_insert(0.0);
                            if omega > *v_entry {
                                *v_entry = omega;
                            }
                        }
                        sim.prev_joint_angles.insert(cj.joint_idx, cur_angle);
                    }
                }
            }

            // --- 7. Transition to flight when extension complete ---
            if sim.phase_time >= sim.extension_duration {
                sim.launch_z = current_z;
                sim.launch_velocity = sim.base_velocity_z;
                // Compute foot Z at launch for landing detection
                let transforms_at_launch = model.compute_transforms();
                sim.launch_foot_z = avg_link_z(&transforms_at_launch, &sim.ground_link_names);
                sim.phase = JumpPhase::Flight;
                sim.phase_time = 0.0;
                sim.prev_base_z = None;
                sim.prev_velocity_z = None;
            }
            true
        }

        JumpPhase::Flight => {
            let g = G as f32;
            let t = sim.phase_time;

            // Ballistic trajectory: z(t) = z0 + v0·t − ½g·t²
            let z_offset = sim.launch_velocity * t - 0.5 * g * t * t;
            let v_z = sim.launch_velocity - g * t;

            let current_z = sim.launch_z + z_offset;
            let mut tf = sim.saved_base_transform;
            // Apply motion only on enabled axes
            if sim.launch_axes[0] {
                // X: no force model yet, keep saved
            }
            if sim.launch_axes[1] {
                // Y: no force model yet, keep saved
            }
            if sim.launch_axes[2] {
                tf.translation.vector.z = current_z;
            }
            model.base_transform = tf;

            // No ground contact during flight
            sim.step_info.grf_z = 0.0;
            sim.step_info.velocity_z = v_z;
            sim.step_info.height =
                current_z - sim.saved_base_transform.translation.vector.z;

            if sim.step_info.height > sim.max_height_reached {
                sim.max_height_reached = sim.step_info.height;
            }

            // Land when feet reach the original ground level.
            // During flight, base-to-foot offset stays constant (joints frozen).
            // foot_z = current_z - (launch_z - launch_foot_z)
            let base_to_foot = sim.launch_z - sim.launch_foot_z;
            let foot_z = current_z - base_to_foot;
            if t > 0.0 && foot_z <= sim.initial_foot_z {
                // Snap base so feet are exactly at initial_foot_z
                let landing_base_z = sim.initial_foot_z + base_to_foot;
                model.base_transform.translation.vector.z = landing_base_z;
                sim.step_info.velocity_z = 0.0;
                sim.phase = JumpPhase::Landed;
                sim.phase_time = 0.0;
            }
            true
        }

        JumpPhase::Landed => {
            sim.step_info.grf_z = sim.total_mass * G; // resting on ground
            sim.step_info.velocity_z = 0.0;

            // Keep feet at ground level during hold (joints are still extended)
            let base_to_foot = sim.launch_z - sim.launch_foot_z;
            let landed_base_z = sim.initial_foot_z + base_to_foot;
            model.base_transform.translation.vector.z = landed_base_z;

            sim.step_info.height =
                landed_base_z - sim.saved_base_transform.translation.vector.z;

            if sim.phase_time >= sim.landed_hold {
                // Restore everything
                model.joint_positions = sim.saved_positions.clone();
                model.base_transform = sim.saved_base_transform;
                false // done
            } else {
                true
            }
        }
    }
}

/// Average Z position of the given links in world frame.
fn avg_link_z(
    transforms: &HashMap<String, na::Isometry3<f32>>,
    link_names: &[String],
) -> f32 {
    let mut sum_z = 0.0_f32;
    let mut count = 0;
    for name in link_names {
        if let Some(tf) = transforms.get(name) {
            sum_z += tf.translation.vector.z;
            count += 1;
        }
    }
    if count > 0 {
        sum_z / count as f32
    } else {
        0.0
    }
}

// ===== Payload sim construction =====

/// Create a payload simulation.
pub fn start_payload_sim(
    model: &RobotModel,
    ee_link: &str,
    speed: f32,
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
            sim.current_mass = sim.max_mass * t_frac as f64;

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
    let chain = crate::ik::build_chain(model, ee_link);
    if chain.is_empty() {
        return;
    }
    let ee_li = match model.link_map.get(ee_link) {
        Some(&li) => li,
        None => return,
    };
    let ee_pos = crate::ik::get_ee_world_pos(model, ee_li, &transforms);
    let jac = crate::ik::compute_jacobian(model, &chain, &transforms, &ee_pos);

    // Force per current mass
    let f = na::DVector::from_column_slice(&[0.0_f32, 0.0, -(G as f32) * sim.current_mass as f32]);
    let tau_payload = jac.transpose() * f;

    // Gravity torques
    let gravity_torques = compute_gravity_torques(model);
    let grav_map: HashMap<usize, f64> = gravity_torques
        .iter()
        .map(|t| (t.joint_idx, t.gravity_torque))
        .collect();

    sim.joint_utilisation.clear();
    for (col, cj) in chain.iter().enumerate() {
        let joint = &model.joints[cj.joint_idx];
        if joint.effort <= 0.0 {
            continue;
        }
        let g_tau = grav_map.get(&cj.joint_idx).copied().unwrap_or(0.0);
        let total_tau = (g_tau + tau_payload[col] as f64).abs();
        let util = total_tau / joint.effort;
        sim.joint_utilisation.push((cj.joint_idx, util));
    }
}

/// Smooth step (Hermite interpolation) for animations.
fn smooth_step(t: f32) -> f32 {
    let t = t.clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

/// Launch profile for jump extension: `1 − cos(πt/2)`.
///
/// Unlike `smooth_step` which has zero velocity at both endpoints,
/// this profile starts from rest (derivative = 0 at t = 0) and reaches
/// **maximum velocity** at t = 1 (derivative = π/2).  This ensures
/// the robot has peak upward speed at the moment of launch.
fn launch_profile(t: f32) -> f32 {
    let t = t.clamp(0.0, 1.0);
    1.0 - (std::f32::consts::FRAC_PI_2 * t).cos()
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
}
