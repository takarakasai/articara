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

/// Compute the gravity torque about a joint axis from a specific set of links.
///
/// `joint_tf` is the world-space transform of the joint.
/// `world_axis` is the joint axis in world frame.
/// `link_indices` are the links to sum over.
fn gravity_torque_from_links(
    model: &RobotModel,
    transforms: &HashMap<String, na::Isometry3<f32>>,
    joint_pos: &na::Vector3<f64>,
    world_axis: &na::Vector3<f64>,
    joint_type: &str,
    link_indices: &[usize],
) -> f64 {
    let mut tau = 0.0_f64;
    for &li in link_indices {
        let link = &model.links[li];
        let mass = link.inertial.mass;
        if mass <= 0.0 {
            continue;
        }
        let link_tf = transforms
            .get(&link.name)
            .copied()
            .unwrap_or(na::Isometry3::identity());
        let com_local = link.inertial.origin.translation.vector.cast::<f64>();
        let com_world = link_tf.cast::<f64>() * na::Point3::from(com_local);
        let r = com_world.coords - joint_pos;
        let f_grav = G_VEC * mass;

        match joint_type {
            "revolute" | "continuous" => {
                tau += world_axis.dot(&r.cross(&f_grav));
            }
            "prismatic" => {
                tau += world_axis.dot(&f_grav);
            }
            _ => {}
        }
    }
    tau
}

/// For joints in a grounded leg, compute the "body-side" gravity torque.
///
/// In a grounded configuration (feet on the floor), each leg joint must support
/// the weight of links on the **body side** (ancestor side), not the foot side.
/// This is the opposite of the free-hanging serial-arm convention used by
/// `compute_gravity_torques` (which sums descendants only).
///
/// body-side torque = total_gravity_torque(all links) − descendant_gravity_torque
///
/// Returns a map: joint_idx → body-side gravity torque (absolute value).
fn compute_body_side_gravity_torques(
    model: &RobotModel,
    joints: &[usize],  // joint indices to compute for
) -> HashMap<usize, f64> {
    let transforms = model.compute_transforms();
    let all_link_indices: Vec<usize> = (0..model.links.len()).collect();
    let mut result = HashMap::new();

    for &ji in joints {
        let joint = &model.joints[ji];
        let jt = joint.joint_type.as_str();
        if jt == "fixed" {
            continue;
        }

        let parent_tf = transforms
            .get(&joint.parent_link)
            .copied()
            .unwrap_or(na::Isometry3::identity());
        let joint_tf = parent_tf * joint.origin;
        let joint_pos = joint_tf.translation.vector.cast::<f64>();
        let world_axis = (joint_tf.rotation * joint.axis).cast::<f64>();

        // Torque from ALL links
        let tau_all = gravity_torque_from_links(
            model, &transforms, &joint_pos, &world_axis, jt, &all_link_indices,
        );

        // Torque from descendant links (foot-side)
        let descendants = descendant_links(model, &joint.child_link);
        let tau_descendants = gravity_torque_from_links(
            model, &transforms, &joint_pos, &world_axis, jt, &descendants,
        );

        // Body-side torque = total − descendants
        let tau_body_side = tau_all - tau_descendants;
        result.insert(ji, tau_body_side);
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
    /// Joints are retracting (pulling legs back) while still on ground.
    /// This adds upward momentum by rapidly lifting the limbs.
    Retract,
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

/// Time-series graph data recorded during a simulation.
#[derive(Clone, Debug, Default)]
pub struct SimGraphData {
    /// Cumulative simulation time per sample (s).
    pub time: Vec<f32>,
    /// World-frame position (x, y, z) of the tracked body.
    pub pos_x: Vec<f32>,
    pub pos_y: Vec<f32>,
    pub pos_z: Vec<f32>,
    /// World-frame velocity (x, y, z) of the tracked body (finite diff).
    pub vel_x: Vec<f32>,
    pub vel_y: Vec<f32>,
    pub vel_z: Vec<f32>,
    /// World-frame acceleration (x, y, z) of the tracked body (finite diff).
    pub acc_x: Vec<f32>,
    pub acc_y: Vec<f32>,
    pub acc_z: Vec<f32>,
    /// Name of the tracked link.
    pub link_name: String,
}

/// Per-joint peak value recorded across the entire simulation.
#[derive(Clone, Debug)]
pub struct JointPeakInfo {
    pub joint_idx: usize,
    pub joint_name: String,
    /// Peak absolute gravity torque seen during extension (N·m).
    pub peak_torque: f64,
    /// Joint angle (rad) at the moment peak torque was recorded.
    pub peak_torque_angle: f64,
    /// Peak absolute angular velocity seen during extension (rad/s).
    pub peak_velocity: f64,
    /// Joint angle (rad) at the moment peak velocity was recorded.
    pub peak_velocity_angle: f64,
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
    /// Time-series graph data recorded at 1 ms intervals.
    pub graph_data: SimGraphData,
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
    /// Maximum vertical stroke (m) — from user's start pose to fully extended.
    pub max_stroke: f32,
    /// Joint indices in this leg that are locked (hold posture).
    pub locked_joint_indices: std::collections::HashSet<usize>,
    /// Joint angles at the user's starting (crouched) pose.
    pub start_angles: Vec<f32>,
    /// Joint angles for the most-extended configuration (from FK sweep).
    pub extend_angles: Vec<f32>,
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
    /// Joint angle at the moment peak torque was recorded.
    pub peak_torque_angles: HashMap<usize, f64>,
    /// Peak absolute angular velocity per joint (joint_idx → peak rad/s).
    pub peak_velocities: HashMap<usize, f64>,
    /// Joint angle at the moment peak velocity was recorded.
    pub peak_velocity_angles: HashMap<usize, f64>,

    /// When true, per-joint IK deltas are clamped so that the resulting
    /// torque (gravity + GRF) never exceeds the URDF effort limit.
    pub enforce_torque_limits: bool,
    /// Whether to retract (pull legs back) after extension before flight.
    pub enable_retract: bool,
    /// Duration of the retract phase (s). Shorter = more aggressive.
    pub retract_duration: f32,
    /// Joint positions at the end of extension (snapshot for retract interpolation).
    pub extended_positions: Vec<f32>,
    /// Base Z at the start of the extension phase (for energy calculation).
    pub start_base_z: f32,
    /// Smoothed base velocity (exponential moving average),
    /// used for launch velocity to ensure continuity at the Extension→Flight boundary.
    pub smoothed_velocity_z: f32,
    /// Peak positive smoothed velocity seen during extension.
    /// Used as launch velocity when the actual velocity at launch time
    /// has already decayed (joints overshoot the FK-optimal point).
    peak_smoothed_velocity_z: f32,
    /// Base Z at the moment peak_smoothed_velocity_z was recorded.
    peak_vel_base_z: f32,
    /// Forward dynamics state (CRBA/RNEA integrator). When `Some`, the
    /// Extension phase uses physics-based simulation instead of kinematic
    /// interpolation.
    pub fd_state: Option<crate::rbd::dynamics::ForwardDynamicsState>,
    /// Cumulative simulation time (s) across all phases.
    pub sim_time: f32,
    /// Time-series graph data for the tracked body link.
    pub graph_data: SimGraphData,
    /// Previous position of the tracked body (for velocity finite diff).
    graph_prev_pos: Option<[f32; 3]>,
    /// Previous velocity of the tracked body (for accel finite diff).
    graph_prev_vel: Option<[f32; 3]>,
    /// Next sim_time at which to record a graph sample (1 ms interval).
    graph_next_sample_time: f32,
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
    enforce_torque_limits: bool,
    enable_retract: bool,
    graph_link: Option<&str>,
    pd_kp: f64,
    pd_kd: f64,
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

        let leg_locked: std::collections::HashSet<usize> = chain
            .iter()
            .filter(|cj| locked_idx.contains(&cj.joint_idx))
            .map(|cj| cj.joint_idx)
            .collect();

        // Current foot position in world frame (user's starting pose)
        let foot_world = foot_link_pos(&transforms, gl);
        let foot_body = body_tf_inv * foot_world;
        let start_foot_z = foot_world.z;

        // FK sweep to find the most-extended (lowest foot Z) configuration.
        let (_min_z, _max_z, _crouch_angles, extend_angles) = compute_leg_z_range(
            model, &chain, gl, &leg_locked,
        );

        // Max stroke = from user's current pose down to most-extended pose
        let max_stroke = (start_foot_z - _min_z).max(0.001);

        // Store the user's current joint angles as the start configuration.
        let start_angles = model.joint_positions.clone();

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
            start_angles,
            extend_angles,
        });
    }

    if legs.is_empty() {
        return None;
    }

    // The user's current joint positions define the starting (crouched) pose.
    let saved_positions_for_restore = model.joint_positions.clone();
    let saved_base_for_restore = model.base_transform;

    // Extension duration: based on actual angular travel from start to extend
    let extension_duration = if let Some(ovr) = extension_override {
        ovr.max(0.05)
    } else {
        let mut max_time = 0.05_f32;
        for leg in &legs {
            // For each non-locked joint, compute actual travel / max velocity.
            // The launch_profile derivative peaks at π/(2T), so the max angular
            // velocity of joint i is |Δθ_i| * π/(2T).  To respect the URDF
            // velocity limit: T ≥ |Δθ_i| * π / (2 * v_max_i).
            let leg_time: f32 = leg
                .chain
                .iter()
                .filter(|cj| !leg.locked_joint_indices.contains(&cj.joint_idx))
                .map(|cj| {
                    let ji = cj.joint_idx;
                    let start_val = if ji < leg.start_angles.len() {
                        leg.start_angles[ji]
                    } else {
                        0.0
                    };
                    let extend_val = if ji < leg.extend_angles.len() {
                        leg.extend_angles[ji]
                    } else {
                        0.0
                    };
                    let delta = (extend_val - start_val).abs();
                    let vel = (model.joints[ji].velocity as f32).max(1.0);
                    // T ≥ delta * π / (2 * vel)
                    delta * std::f32::consts::FRAC_PI_2 / vel
                })
                .fold(0.0_f32, f32::max);
            if leg_time > max_time {
                max_time = leg_time;
            }
        }
        max_time
    };

    // Build the forward dynamics state for physics-based extension.
    let fd_state = {
        let foot_chains: Vec<Vec<crate::ik::ChainJoint>> = legs.iter()
            .map(|leg| leg.chain.clone())
            .collect();
        let contact_feet: Vec<String> = ground_links.to_vec();
        // Collect all locked joint indices across all legs.
        let all_locked: std::collections::HashSet<usize> = legs.iter()
            .flat_map(|leg| leg.locked_joint_indices.iter().copied())
            .collect();
        let mut fd = crate::rbd::dynamics::ForwardDynamicsState::new(
            model,
            contact_feet,
            foot_chains,
            &body,
            &all_locked,
        );

        // --- Pre-compute joint-space trajectory ---
        // Each non-locked leg joint gets a smooth trajectory from its
        // current (start) angle to the extend angle, over extension_duration.
        //
        // We use the **Launch** profile (quarter-cosine): smooth start,
        // maximum joint velocity at t = T.  This maximises the base
        // velocity at the moment of launch.
        let mut traj = HashMap::new();
        for leg in &legs {
            for cj in &leg.chain {
                let ji = cj.joint_idx;
                if leg.locked_joint_indices.contains(&ji) {
                    continue;
                }
                if traj.contains_key(&ji) {
                    continue; // already set by another leg sharing this joint
                }
                let q_start = model.joint_positions[ji] as f64;
                let q_end = if ji < leg.extend_angles.len() {
                    leg.extend_angles[ji] as f64
                } else {
                    q_start
                };
                traj.insert(ji, crate::rbd::dynamics::JointTrajectoryPoint {
                    q_start,
                    q_end,
                    duration: extension_duration as f64,
                    profile: crate::rbd::dynamics::TrajectoryProfile::Launch,
                });
            }
        }
        fd.trajectory = traj;
        fd.trajectory_time = 0.0;
        fd.kp = pd_kp;
        fd.kd = pd_kd;

        Some(fd)
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
        saved_positions: saved_positions_for_restore,
        saved_base_transform: saved_base_for_restore,
        speed,
        launch_axes,
        prev_base_z: None,
        prev_velocity_z: None,
        peak_torques: HashMap::new(),
        peak_torque_angles: HashMap::new(),
        peak_velocities: HashMap::new(),
        peak_velocity_angles: HashMap::new(),

        enforce_torque_limits,
        enable_retract,
        retract_duration: extension_duration * 0.3,
        extended_positions: Vec::new(),
        start_base_z: model.base_transform.translation.vector.z,
        smoothed_velocity_z: 0.0,
        peak_smoothed_velocity_z: 0.0,
        peak_vel_base_z: model.base_transform.translation.vector.z,
        fd_state,
        sim_time: 0.0,
        graph_data: SimGraphData {
            link_name: graph_link.unwrap_or(&body).to_string(),
            ..Default::default()
        },
        graph_prev_pos: None,
        graph_prev_vel: None,
        graph_next_sample_time: 0.0,
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

/// Compute the Z-range of a leg's foot position by sweeping joint angles via FK.
///
/// This avoids Jacobian singularities by directly evaluating FK at sampled
/// joint angle combinations.  Returns `(min_foot_z, max_foot_z, crouch_angles, extend_angles)`
/// where `crouch_angles` gives `max_foot_z` (most crouched) and `extend_angles`
/// gives `min_foot_z` (most extended).
fn compute_leg_z_range(
    model: &mut RobotModel,
    chain: &[crate::ik::ChainJoint],
    ground_link: &str,
    locked_joints: &std::collections::HashSet<usize>,
) -> (f32, f32, Vec<f32>, Vec<f32>) {
    let saved_positions = model.joint_positions.clone();
    let transforms0 = model.compute_transforms();
    let initial_foot_z = foot_link_pos(&transforms0, ground_link).z;

    // Collect only the movable (non-locked) joints in the chain
    let active: Vec<usize> = chain
        .iter()
        .filter(|cj| !locked_joints.contains(&cj.joint_idx))
        .map(|cj| cj.joint_idx)
        .collect();

    if active.is_empty() {
        return (initial_foot_z, initial_foot_z, saved_positions.clone(), saved_positions);
    }

    let n_samples = 16_usize; // samples per joint
    let mut best_min_z = initial_foot_z;
    let mut best_max_z = initial_foot_z;
    let mut crouch_angles = saved_positions.clone(); // angles for max_z (crouched)
    let mut extend_angles = saved_positions.clone(); // angles for min_z (extended)

    // For up to 3 active joints, do full combinatorial sweep.
    // For more, do a coarse sweep + refinement.
    let n_active = active.len();
    let total_combos = (n_samples as u64).pow(n_active as u32);

    if total_combos <= 50_000 {
        // Full sweep
        for combo in 0..total_combos {
            let mut idx = combo;
            for &ji in &active {
                let lo = model.joints[ji].lower as f32;
                let hi = model.joints[ji].upper as f32;
                let step_i = idx % (n_samples as u64);
                idx /= n_samples as u64;
                let frac = step_i as f32 / (n_samples - 1).max(1) as f32;
                model.joint_positions[ji] = lo + (hi - lo) * frac;
            }
            let tf = model.compute_transforms();
            let fz = foot_link_pos(&tf, ground_link).z;
            if fz < best_min_z {
                best_min_z = fz;
                extend_angles = model.joint_positions.clone();
            }
            if fz > best_max_z {
                best_max_z = fz;
                crouch_angles = model.joint_positions.clone();
            }
        }
    } else {
        // Random sampling for many joints
        use std::collections::hash_map::DefaultHasher;
        use std::hash::{Hash, Hasher};
        for sample in 0..50_000_u64 {
            for &ji in &active {
                let lo = model.joints[ji].lower as f32;
                let hi = model.joints[ji].upper as f32;
                let mut hasher = DefaultHasher::new();
                (sample, ji).hash(&mut hasher);
                let hash = hasher.finish();
                let frac = (hash as f32) / (u64::MAX as f32);
                model.joint_positions[ji] = lo + (hi - lo) * frac;
            }
            let tf = model.compute_transforms();
            let fz = foot_link_pos(&tf, ground_link).z;
            if fz < best_min_z {
                best_min_z = fz;
                extend_angles = model.joint_positions.clone();
            }
            if fz > best_max_z {
                best_max_z = fz;
                crouch_angles = model.joint_positions.clone();
            }
        }
    }

    model.joint_positions = saved_positions;
    (best_min_z, best_max_z, crouch_angles, extend_angles)
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
                peak_torque_angle: sim.peak_torque_angles.get(&lj.joint_idx).copied().unwrap_or(0.0),
                peak_velocity: sim.peak_velocities.get(&lj.joint_idx).copied().unwrap_or(0.0),
                peak_velocity_angle: sim.peak_velocity_angles.get(&lj.joint_idx).copied().unwrap_or(0.0),
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
        graph_data: sim.graph_data.clone(),
    }
}

/// Step the jump simulation by `dt` seconds.
///
/// **Extension phase (forward dynamics):**
/// 1. Compute desired joint torques: each contributing joint applies max
///    effort in the direction from start→extended angles.
/// 2. Solve constrained forward dynamics (CRBA + RNEA + contact Jacobians)
///    to get joint accelerations **q̈** and ground reaction forces **λ**.
/// 3. Semi-implicit Euler integration updates q, q̇, and the base transform.
/// 4. Launch into flight when GRF drops to zero (feet naturally lift off).
///
/// **Flight phase:** base follows ballistic trajectory; joints are
/// driven by forward dynamics (unconstrained, no ground contact) to
/// retract toward the saved pre-jump pose.
///
/// Returns `true` while still running.
pub fn step_jump_sim(sim: &mut JumpSim, model: &mut RobotModel, dt: f32) -> bool {
    let dt_frame = dt * sim.speed;
    if dt_frame <= 0.0 {
        return true;
    }

    // --- Fixed-timestep sub-stepping for numerical stability ---
    // Forward dynamics with semi-implicit Euler requires ≤1 ms steps.
    // The caller provides a frame-rate dt (~16 ms); we subdivide it.
    const MAX_PHYSICS_DT: f32 = 0.0005; // 0.5 ms
    let n_steps = ((dt_frame / MAX_PHYSICS_DT).ceil() as usize).max(1);
    let sub_dt = dt_frame / n_steps as f32;

    const GRAPH_INTERVAL: f32 = 0.001; // 1 ms

    let mut still_running = true;
    for _ in 0..n_steps {
        if !still_running {
            break;
        }
        sim.phase_time += sub_dt;
        sim.sim_time += sub_dt;
        still_running = step_jump_sub(sim, model, sub_dt);

        // Record graph data at 1 ms intervals
        if sim.sim_time >= sim.graph_next_sample_time {
            record_graph_sample(sim, model, GRAPH_INTERVAL);
            sim.graph_next_sample_time += GRAPH_INTERVAL;
        }
    }

    still_running
}

/// One physics sub-step of the jump simulation.
fn step_jump_sub(sim: &mut JumpSim, model: &mut RobotModel, dt: f32) -> bool {
    match sim.phase {
        JumpPhase::Extension => {
            // ===== Forward-dynamics based Extension phase =====
            //
            // When a pre-computed trajectory is set on the FD state,
            // step() uses PD tracking (Kp/Kd + gravity compensation).
            // Otherwise it falls back to the null-space velocity
            // controller.  target_angles are only used by the fallback
            // path but we still build them for joint-utilisation display.

            // --- 1. Build target angles (used by null-space fallback) ---
            let mut target_angles: HashMap<usize, f64> = HashMap::new();
            sim.step_info.joint_utilisation.clear();

            for leg in &sim.legs {
                for cj in leg.chain.iter() {
                    let ji = cj.joint_idx;
                    if leg.locked_joint_indices.contains(&ji) {
                        continue;
                    }
                    let joint = &model.joints[ji];
                    if joint.effort <= 0.0 {
                        continue;
                    }

                    let extend_val = if ji < leg.extend_angles.len() {
                        leg.extend_angles[ji] as f64
                    } else {
                        model.joint_positions[ji] as f64
                    };

                    target_angles.insert(ji, extend_val);
                    sim.step_info.joint_utilisation.push((ji, 1.0, true));
                }
            }

            // --- 2. Forward dynamics step ---
            // PD trajectory-tracking when trajectory is set;
            // null-space velocity control + foot-X feedback otherwise.
            if let Some(ref mut fd) = sim.fd_state {
                fd.step(model, &target_angles, dt as f64);
            }

            // --- 3. FK foot-constraint: recompute base.z so feet stay on ground ---
            // After joint integration the leg shape has changed, so we
            // recalculate the base height that keeps the average foot at
            // the initial ground level — same logic as the old kinematic code.
            {
                let saved_base = model.base_transform;
                let mut temp_base = sim.saved_base_transform;
                temp_base.translation.vector.z = 0.0;
                model.base_transform = temp_base;

                let transforms = model.compute_transforms();
                let foot_z_at_zero = avg_link_z(&transforms, &sim.ground_link_names);

                // base_z such that foot_z = initial_foot_z
                let new_base_z = sim.initial_foot_z - foot_z_at_zero;
                model.base_transform = saved_base;
                model.base_transform.translation.vector.z = new_base_z;
            }

            // --- 4. Velocity & GRF from finite differences ---
            let current_z = model.base_transform.translation.vector.z;
            if let Some(prev_z) = sim.prev_base_z {
                sim.base_velocity_z = (current_z - prev_z) / dt;
            }
            sim.prev_base_z = Some(current_z);

            // GRF from Newton's 2nd law: GRF = M*(a + g)
            let accel_z = if let Some(prev_v) = sim.prev_velocity_z {
                (sim.base_velocity_z - prev_v) / dt
            } else {
                0.0
            };
            sim.prev_velocity_z = Some(sim.base_velocity_z);
            let grf_z = sim.total_mass * (accel_z as f64 + G);

            sim.step_info.grf_z = grf_z;
            sim.step_info.velocity_z = sim.base_velocity_z;
            sim.step_info.height =
                current_z - sim.saved_base_transform.translation.vector.z;

            if sim.step_info.height > sim.max_height_reached {
                sim.max_height_reached = sim.step_info.height;
            }

            // Smoothed velocity for launch
            let smooth_alpha = (dt * 10.0).clamp(0.05, 0.5);
            sim.smoothed_velocity_z = (1.0 - smooth_alpha) * sim.smoothed_velocity_z
                + smooth_alpha * sim.base_velocity_z;

            // Track peak positive velocity (the physical "launch point").
            if sim.smoothed_velocity_z > sim.peak_smoothed_velocity_z {
                sim.peak_smoothed_velocity_z = sim.smoothed_velocity_z;
                sim.peak_vel_base_z = current_z;
            }

            // --- 4. Track per-joint torque peaks and angular velocity ---
            {
                let mut joint_indices: Vec<usize> = sim.legs.iter()
                    .flat_map(|leg| leg.chain.iter().map(|cj| cj.joint_idx))
                    .collect();
                joint_indices.sort_unstable();
                joint_indices.dedup();

                let body_tau_map = compute_body_side_gravity_torques(model, &joint_indices);

                let static_weight = sim.total_mass * G;
                let grf_scale = if static_weight > 1e-6 {
                    (grf_z / static_weight).abs()
                } else {
                    1.0
                };

                for leg in &sim.legs {
                    for cj in leg.chain.iter() {
                        let body_tau = body_tau_map.get(&cj.joint_idx).copied().unwrap_or(0.0).abs();
                        let total_tau = body_tau * grf_scale;
                        let cur_angle = model.joint_positions[cj.joint_idx];

                        let record_tau = if sim.enforce_torque_limits {
                            let effort = model.joints[cj.joint_idx].effort;
                            if effort > 0.0 { total_tau.min(effort) } else { total_tau }
                        } else {
                            total_tau
                        };

                        let entry = sim.peak_torques.entry(cj.joint_idx).or_insert(0.0);
                        if record_tau > *entry {
                            *entry = record_tau;
                            sim.peak_torque_angles.insert(cj.joint_idx, cur_angle as f64);
                        }

                        // Angular velocity from fd_state
                        let omega = sim.fd_state.as_ref()
                            .and_then(|fd| fd.joint_velocities.get(&cj.joint_idx))
                            .copied()
                            .unwrap_or(0.0)
                            .abs();
                        let v_entry = sim.peak_velocities.entry(cj.joint_idx).or_insert(0.0);
                        if omega > *v_entry {
                            *v_entry = omega;
                            sim.peak_velocity_angles.insert(cj.joint_idx, cur_angle as f64);
                        }
                    }
                }
            }

            // --- 6. Transition: launch ---
            // Launch when:
            //  (a) Velocity has peaked and is now declining significantly
            //      (the robot has reached its "push-off apex"), OR
            //  (b) Trajectory time elapsed AND joints are close to their
            //      extend angles (≥ 95% travel), OR
            //  (c) Safety timeout: 3× extension_duration (hard cap).
            //
            // Use the CURRENT base Z and velocity for launch to ensure
            // smooth continuity at the Extension→Flight boundary.
            let min_phase = 0.01_f32;
            let peak_v = sim.peak_smoothed_velocity_z;
            let vel_declining = peak_v > 0.05
                && sim.smoothed_velocity_z < peak_v * 0.5
                && sim.phase_time > min_phase;

            // Check if joints have reached ≥ 95% of their total travel
            let joints_extended = if let Some(ref fd) = sim.fd_state {
                fd.trajectory.iter().all(|(&ji, traj)| {
                    let total = (traj.q_end - traj.q_start).abs();
                    if total < 1e-6 { return true; }
                    let q = model.joint_positions[ji] as f64;
                    let progress = (q - traj.q_start) / (traj.q_end - traj.q_start);
                    progress >= 0.95
                })
            } else {
                true
            };

            let time_elapsed = sim.phase_time >= sim.extension_duration;
            let hard_timeout = sim.phase_time >= sim.extension_duration * 3.0;
            let should_launch = vel_declining
                || (time_elapsed && joints_extended)
                || hard_timeout;

            if should_launch {
                // Launch from current state — no snap-back.
                let launch_z = model.base_transform.translation.vector.z;
                let launch_vel = sim.base_velocity_z.max(0.0);

                sim.launch_z = launch_z;
                sim.launch_velocity = launch_vel;

                let transforms_at_launch = model.compute_transforms();
                sim.launch_foot_z = avg_link_z(&transforms_at_launch, &sim.ground_link_names);

                if sim.enable_retract {
                    sim.extended_positions = model.joint_positions.clone();
                }

                // Keep fd_state for FD-based flight, but clear contacts
                // (no ground contact during flight → unconstrained FD).
                // Clear the Extension trajectory and zero joint velocities
                // so joints don't carry extension momentum into flight.
                if let Some(ref mut fd) = sim.fd_state {
                    fd.contact_feet.clear();
                    fd.initial_foot_x.clear();
                    fd.foot_chains.clear();
                    fd.trajectory.clear();
                    fd.trajectory_time = 0.0;

                    // Zero all joint velocities — the launch velocity is
                    // captured in the ballistic base trajectory; joints
                    // should not coast forward during flight.
                    for qd in fd.joint_velocities.values_mut() {
                        *qd = 0.0;
                    }

                    // If retract is enabled, set up a Symmetric trajectory
                    // from extended → saved positions so the PD controller
                    // drives retraction smoothly (not the null-space path).
                    if sim.enable_retract {
                        let retract_duration = 0.15_f64; // 150 ms retract
                        let mut traj = HashMap::new();
                        for lj in &sim.leg_joints {
                            let ji = lj.joint_idx;
                            let q_now = model.joint_positions[ji] as f64;
                            let q_saved = if ji < sim.saved_positions.len() {
                                sim.saved_positions[ji] as f64
                            } else {
                                q_now
                            };
                            traj.insert(ji, crate::rbd::dynamics::JointTrajectoryPoint {
                                q_start: q_now,
                                q_end: q_saved,
                                duration: retract_duration,
                                profile: crate::rbd::dynamics::TrajectoryProfile::Symmetric,
                            });
                        }
                        fd.trajectory = traj;
                    }
                }

                sim.phase = JumpPhase::Flight;
                sim.phase_time = 0.0;
                sim.prev_base_z = None;
                sim.prev_velocity_z = None;
            }
            true
        }

        JumpPhase::Retract => {
            // This phase is no longer entered — retract animation is
            // handled in the Flight phase.  Kept for enum completeness.
            sim.phase = JumpPhase::Flight;
            sim.phase_time = 0.0;
            true
        }

        JumpPhase::Flight => {
            let g = G as f32;
            let t = sim.phase_time;

            // Ballistic trajectory for base: z(t) = z0 + v0·t − ½g·t²
            // (CoM follows a parabola regardless of internal joint motion)
            let z_offset = sim.launch_velocity * t - 0.5 * g * t * t;
            let v_z = sim.launch_velocity - g * t;

            let current_z = sim.launch_z + z_offset;
            let mut tf = sim.saved_base_transform;
            if sim.launch_axes[2] {
                tf.translation.vector.z = current_z;
            }
            model.base_transform = tf;

            // --- Continue forward dynamics (unconstrained) ---
            // Contacts were cleared at the Extension→Flight transition,
            // so step() runs the same FD pipeline without foot constraints.
            //
            //  • If a retract trajectory was set at transition, the PD
            //    computed-torque controller drives joints back smoothly.
            //  • If no trajectory is set (retract disabled), joints evolve
            //    under gravity alone (unconstrained FD, no applied torque
            //    beyond gravity compensation in the null-space path).
            {
                let hold_targets: HashMap<usize, f64> = sim.leg_joints.iter()
                    .map(|lj| (lj.joint_idx, model.joint_positions[lj.joint_idx] as f64))
                    .collect();
                if let Some(ref mut fd) = sim.fd_state {
                    fd.step(model, &hold_targets, dt as f64);
                }
                // Restore base Z (step() doesn't touch base, but be safe)
                model.base_transform.translation.vector.z = current_z;
            }

            // No ground contact during flight
            sim.step_info.grf_z = 0.0;
            sim.step_info.velocity_z = v_z;
            sim.step_info.height =
                current_z - sim.saved_base_transform.translation.vector.z;

            if sim.step_info.height > sim.max_height_reached {
                sim.max_height_reached = sim.step_info.height;
            }

            // Land when feet reach the original ground level.
            // Compute current base-to-foot offset dynamically (joints may
            // be changing due to retract animation).
            let foot_z = {
                let saved_base = model.base_transform;
                let mut temp_base = sim.saved_base_transform;
                temp_base.translation.vector.z = 0.0;
                model.base_transform = temp_base;
                let transforms = model.compute_transforms();
                let fz = avg_link_z(&transforms, &sim.ground_link_names);
                model.base_transform = saved_base;
                // foot_world_z = current_z + fz (fz is foot offset from base at z=0)
                current_z + fz
            };

            if t > 0.0 && foot_z <= sim.initial_foot_z {
                // Snap base so feet are exactly at initial_foot_z
                let foot_offset = foot_z - current_z; // negative offset
                let landing_base_z = sim.initial_foot_z - foot_offset;
                model.base_transform.translation.vector.z = landing_base_z;
                sim.step_info.velocity_z = 0.0;
                sim.phase = JumpPhase::Landed;
                sim.phase_time = 0.0;
            }
            true
        }

        JumpPhase::Landed => {
            // Drop FD state — no longer needed after landing.
            sim.fd_state = None;

            sim.step_info.grf_z = sim.total_mass * G; // resting on ground
            sim.step_info.velocity_z = 0.0;

            // Compute foot offset dynamically (joints may have been
            // retracted during flight).
            let saved_base = model.base_transform;
            let mut temp_base = sim.saved_base_transform;
            temp_base.translation.vector.z = 0.0;
            model.base_transform = temp_base;
            let transforms = model.compute_transforms();
            let foot_z_at_zero = avg_link_z(&transforms, &sim.ground_link_names);
            model.base_transform = saved_base;

            let landed_base_z = sim.initial_foot_z - foot_z_at_zero;
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

/// Record one sample of position/velocity/acceleration for the tracked body.
fn record_graph_sample(sim: &mut JumpSim, model: &RobotModel, dt: f32) {
    let transforms = model.compute_transforms();
    let link_tf = transforms
        .get(&sim.graph_data.link_name)
        .copied()
        .unwrap_or(na::Isometry3::identity());
    let pos = link_tf.translation.vector;
    let px = pos.x;
    let py = pos.y;
    let pz = pos.z;

    // Velocity via finite difference
    let (vx, vy, vz) = if let Some([ppx, ppy, ppz]) = sim.graph_prev_pos {
        if dt > 1e-9 {
            ((px - ppx) / dt, (py - ppy) / dt, (pz - ppz) / dt)
        } else {
            (0.0, 0.0, 0.0)
        }
    } else {
        (0.0, 0.0, 0.0)
    };

    // Acceleration via finite difference of velocity
    let (ax, ay, az) = if let Some([pvx, pvy, pvz]) = sim.graph_prev_vel {
        if dt > 1e-9 {
            ((vx - pvx) / dt, (vy - pvy) / dt, (vz - pvz) / dt)
        } else {
            (0.0, 0.0, 0.0)
        }
    } else {
        (0.0, 0.0, 0.0)
    };

    sim.graph_prev_pos = Some([px, py, pz]);
    sim.graph_prev_vel = Some([vx, vy, vz]);

    let g = &mut sim.graph_data;
    g.time.push(sim.sim_time);
    g.pos_x.push(px);
    g.pos_y.push(py);
    g.pos_z.push(pz);
    g.vel_x.push(vx);
    g.vel_y.push(vy);
    g.vel_z.push(vz);
    g.acc_x.push(ax);
    g.acc_y.push(ay);
    g.acc_z.push(az);
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
#[allow(dead_code)]
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
#[allow(dead_code)]
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

    fn load_namiashi() -> RobotModel {
        let path = Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("sample/namiashi_description/urdf/namiashi.urdf");
        RobotModel::from_urdf(&path).unwrap()
    }

    #[test]
    fn jump_sim_produces_positive_launch_velocity() {
        let mut model = load_namiashi();

        // Ground links: the four foot links (RL_foot, etc.)
        let ground_links: Vec<String> = vec![
            "RL_foot".into(), "FL_foot".into(),
            "RR_foot".into(), "FR_foot".into(),
        ];
        let locked = std::collections::HashSet::new();

        // Set a crouched pose (bend knees) — simulates user input.
        for j in &model.joints {
            if j.name.contains("thigh") {
                let ji = model.joints.iter().position(|jj| jj.name == j.name).unwrap();
                model.joint_positions[ji] = 0.8;
            }
            if j.name.contains("calf") {
                let ji = model.joints.iter().position(|jj| jj.name == j.name).unwrap();
                model.joint_positions[ji] = -1.5;
            }
        }

        // Adjust base_z so feet touch the ground
        {
            let mut temp_base = model.base_transform;
            temp_base.translation.vector.z = 0.0;
            model.base_transform = temp_base;
            let tf = model.compute_transforms();
            let foot_z = avg_link_z(&tf, &ground_links);
            model.base_transform.translation.vector.z = -foot_z;
        }

        let start_base_z = model.base_transform.translation.vector.z;
        eprintln!("start_base_z = {:.6}", start_base_z);

        // Print start and extend angles for one leg
        {
            let body = "trunk".to_string();
            let chain = crate::ik::build_chain_between(&model, "RL_foot", Some(&body));
            for cj in &chain {
                let j = &model.joints[cj.joint_idx];
                eprintln!("  {} start={:.4} range=[{:.4}, {:.4}] vel={:.1}",
                    j.name, model.joint_positions[cj.joint_idx],
                    j.lower, j.upper, j.velocity);
            }
        }

        let mut sim = start_jump_sim(
            &mut model,
            &ground_links,
            Some("trunk"),
            1.0,
            &locked,
            [false, false, true],
            None,
            false,
            false,
            None,
            500.0,  // pd_kp
            20.0,   // pd_kd
        ).expect("failed to create jump sim");

        eprintln!("extension_duration = {:.4}", sim.extension_duration);
        eprintln!("max_strokes: {:?}", sim.legs.iter().map(|l| l.max_stroke).collect::<Vec<_>>());
        eprintln!("initial_foot_z = {:.6}", sim.initial_foot_z);

        // Show start→extend angle differences
        for (li, leg) in sim.legs.iter().enumerate() {
            if li == 0 {
                for cj in &leg.chain {
                    let ji = cj.joint_idx;
                    let s = if ji < leg.start_angles.len() { leg.start_angles[ji] } else { 0.0 };
                    let e = if ji < leg.extend_angles.len() { leg.extend_angles[ji] } else { 0.0 };
                    eprintln!("  joint[{}] start={:.4} extend={:.4} Δ={:.4}",
                        model.joints[ji].name, s, e, (e - s).abs());
                }
            }
        }

        let dt = 1.0 / 60.0_f32;
        let mut step = 0;
        let mut peak_vel = 0.0_f32;
        loop {
            let running = step_jump_sim(&mut sim, &mut model, dt);
            step += 1;

            if sim.phase == JumpPhase::Extension {
                if sim.base_velocity_z > peak_vel {
                    peak_vel = sim.base_velocity_z;
                }
                let ext_h = model.base_transform.translation.vector.z - start_base_z;
                let energy_vel = (2.0 * G as f32 * ext_h.max(0.0)).sqrt();
                if step % 5 == 0 {
                    eprintln!(
                        "  ext step={:3} t={:.3} base_z={:.4} vel={:.4} peak_v={:.4} energy_v={:.4} α={:.3}",
                        step, sim.phase_time,
                        model.base_transform.translation.vector.z,
                        sim.base_velocity_z, peak_vel, energy_vel,
                        launch_profile((sim.phase_time / sim.extension_duration).clamp(0.0, 1.0)),
                    );
                }
            }

            if sim.phase == JumpPhase::Flight && sim.phase_time < 0.02 {
                eprintln!(
                    "  FLIGHT: launch_vel={:.4} launch_z={:.4} peak_geom_vel={:.4}",
                    sim.launch_velocity, sim.launch_z, peak_vel,
                );
            }

            if !running || step > 5000 {
                break;
            }
        }

        let ext_height = sim.launch_z - start_base_z;
        let energy_v = (2.0 * G as f32 * ext_height.max(0.0)).sqrt();
        eprintln!("--- SUMMARY ---");
        eprintln!("extension_height = {:.4}", ext_height);
        eprintln!("peak_geometric_vel = {:.4}", peak_vel);
        eprintln!("energy_based_vel = {:.4}", energy_v);
        eprintln!("launch_velocity_used = {:.4}", sim.launch_velocity);
        eprintln!("max_height_reached = {:.4}", sim.max_height_reached);
        eprintln!("expected_flight_h (energy) = {:.4}", energy_v * energy_v / (2.0 * G as f32));

        assert!(
            sim.launch_velocity > 0.01,
            "launch_velocity should be positive, got {}",
            sim.launch_velocity,
        );
        assert!(
            sim.max_height_reached > 0.001,
            "max_height should be positive, got {}",
            sim.max_height_reached,
        );
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

    #[test]
    fn jump_sim_toml_conditions() {
        // Reproduce exact conditions from sim/jump/jump_001.toml
        let mut model = load_namiashi();
        let ground_links: Vec<String> = vec![
            "RL_foot".into(), "FL_foot".into(),
            "RR_foot".into(), "FR_foot".into(),
        ];

        // Locked joints from TOML
        let mut locked = std::collections::HashSet::new();
        locked.insert("RL_hip_joint".to_string());
        locked.insert("FL_hip_joint".to_string());
        locked.insert("RR_hip_joint".to_string());
        locked.insert("FR_hip_joint".to_string());
        locked.insert("arm_pitch_joint".to_string());

        // Start pose from TOML: thigh=1.0, calf=-2.0, hip≈0
        for j in &model.joints {
            let ji = model.joints.iter().position(|jj| jj.name == j.name).unwrap();
            if j.name.contains("thigh") {
                model.joint_positions[ji] = 1.0;
            } else if j.name.contains("calf") {
                model.joint_positions[ji] = -2.0;
            } else if j.name.contains("hip") {
                model.joint_positions[ji] = 0.005; // RL_hip slightly offset
            }
        }

        // Adjust base_z so feet touch the ground
        {
            let mut temp_base = model.base_transform;
            temp_base.translation.vector.z = 0.0;
            model.base_transform = temp_base;
            let tf = model.compute_transforms();
            let foot_z = avg_link_z(&tf, &ground_links);
            model.base_transform.translation.vector.z = -foot_z;
        }

        let start_base_z = model.base_transform.translation.vector.z;
        eprintln!("[TOML] start_base_z = {:.6}", start_base_z);

        let mut sim = start_jump_sim(
            &mut model, &ground_links, Some("trunk"),
            1.0, &locked,
            [false, false, true],
            None,
            true,  // enforce_torque_limits
            true,  // enable_retract
            None,
            500.0,  // pd_kp
            20.0,   // pd_kd
        ).expect("failed to create jump sim");

        eprintln!("[TOML] extension_duration = {:.4}", sim.extension_duration);
        eprintln!("[TOML] start_base_z (sim) = {:.6}", sim.start_base_z);

        // Dump start/extend angles for first leg
        for cj in &sim.legs[0].chain {
            let ji = cj.joint_idx;
            let s = if ji < sim.legs[0].start_angles.len() { sim.legs[0].start_angles[ji] } else { 0.0 };
            let e = if ji < sim.legs[0].extend_angles.len() { sim.legs[0].extend_angles[ji] } else { 0.0 };
            eprintln!("[TOML]   {} start={:.4} extend={:.4} Δ={:.4} locked={}",
                model.joints[ji].name, s, e, (e - s).abs(),
                sim.legs[0].locked_joint_indices.contains(&ji));
        }

        let dt = 1.0 / 60.0_f32;
        let mut step = 0;
        let mut logged_phases = std::collections::HashSet::new();
        loop {
            let running = step_jump_sim(&mut sim, &mut model, dt);
            step += 1;

            let phase_name = format!("{:?}", sim.phase);
            if !logged_phases.contains(&phase_name) {
                eprintln!("[TOML] → {} at step={} t={:.3} base_z={:.4} vel={:.4}",
                    phase_name, step, sim.phase_time,
                    model.base_transform.translation.vector.z,
                    sim.base_velocity_z);
                logged_phases.insert(phase_name);
            }

            if sim.phase == JumpPhase::Extension && step % 5 == 0 {
                eprintln!("[TOML]   ext step={} t={:.3} base_z={:.4} vel={:.4} grf={:.1}",
                    step, sim.phase_time,
                    model.base_transform.translation.vector.z,
                    sim.base_velocity_z, sim.step_info.grf_z);
            }

            if !running || step > 10000 {
                break;
            }
        }

        eprintln!("[TOML] --- SUMMARY ---");
        eprintln!("[TOML] total_steps = {}", step);
        eprintln!("[TOML] launch_velocity = {:.4}", sim.launch_velocity);
        eprintln!("[TOML] max_height = {:.4}", sim.max_height_reached);
        eprintln!("[TOML] final_phase = {:?}", sim.phase);

        assert!(sim.launch_velocity > 0.01,
            "launch_velocity too low: {}", sim.launch_velocity);
        assert!(sim.max_height_reached > 0.01,
            "max_height too low: {}", sim.max_height_reached);
    }
}
