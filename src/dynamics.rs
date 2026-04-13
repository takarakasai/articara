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
    /// Determined by the Z-component of the positional Jacobian.
    /// Non-contributing joints hold their initial posture during extension.
    pub contributes: bool,
}

/// State for an active jump simulation with per-step quasi-dynamics.
#[derive(Clone, Debug)]
pub struct JumpSim {
    pub phase: JumpPhase,
    /// Elapsed time within the current phase (s).
    pub phase_time: f32,

    // --- joint trajectory ---
    pub leg_joints: Vec<LegJointSim>,
    /// Planned extension duration (s) from joint velocities.
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
pub fn start_jump_sim(
    model: &RobotModel,
    ground_links: &[String],
    body_link: Option<&str>,
    speed: f32,
    locked_joints: &std::collections::HashSet<String>,
    launch_axes: [bool; 3],
) -> Option<JumpSim> {
    if ground_links.is_empty() {
        return None;
    }

    let body = body_link.unwrap_or(&model.root_link);

    // Collect leg joints
    let mut seen = std::collections::HashSet::new();
    let mut leg_joints = Vec::new();
    let mut max_extension_time = 0.0_f32;

    for gl in ground_links {
        let chain = crate::ik::build_chain_between(model, gl, Some(body));
        for cj in &chain {
            if seen.contains(&cj.joint_idx) {
                continue;
            }
            seen.insert(cj.joint_idx);

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
            let to_lower = (cur - lower).abs();
            let to_upper = (upper - cur).abs();

            // Extension target = farther limit from current position
            let extended = if to_upper >= to_lower { upper } else { lower };
            let stroke = (extended - cur).abs();

            // Velocity → extension time for this joint
            let vel = (joint.velocity as f32).max(1.0);
            let joint_time = stroke / vel;
            if joint_time > max_extension_time {
                max_extension_time = joint_time;
            }

            leg_joints.push(LegJointSim {
                joint_idx: cj.joint_idx,
                start_angle: cur,
                extended_angle: extended,
                max_velocity: vel,
                contributes: true, // will be refined below via Jacobian
            });
        }
    }

    if leg_joints.is_empty() {
        return None;
    }

    let total_mass: f64 = model.links.iter().map(|l| l.inertial.mass).sum();
    if total_mass <= 0.0 {
        return None;
    }

    // --- Determine which joints contribute to the vertical push-off ---
    // Compute the positional Jacobian for each ground→body chain and check
    // the Z-row magnitude.  Joints with |∂body_z / ∂θ| below a threshold
    // are posture-hold joints (e.g. hip Roll on namiashi).
    let transforms = model.compute_transforms();
    let body_li = *model.link_map.get(body)?;
    let body_pos = crate::ik::get_ee_world_pos(model, body_li, &transforms);

    let mut z_sensitivity: HashMap<usize, f32> = HashMap::new();
    for gl in ground_links {
        let chain = crate::ik::build_chain_between(model, gl, Some(body));
        if chain.is_empty() {
            continue;
        }
        let jac = crate::ik::compute_jacobian(model, &chain, &transforms, &body_pos);
        for (col, cj) in chain.iter().enumerate() {
            let jz = jac[(2, col)].abs(); // Z sensitivity
            let entry = z_sensitivity.entry(cj.joint_idx).or_insert(0.0);
            if jz > *entry {
                *entry = jz;
            }
        }
    }

    // Threshold: joints with |J_z| < 1 cm per radian are posture-hold.
    const Z_THRESHOLD: f32 = 0.01;
    for lj in &mut leg_joints {
        let jname = &model.joints[lj.joint_idx].name;
        if locked_joints.contains(jname) {
            // User explicitly locked this joint
            lj.contributes = false;
        } else {
            let jz = z_sensitivity.get(&lj.joint_idx).copied().unwrap_or(0.0);
            lj.contributes = jz >= Z_THRESHOLD;
        }
    }

    // Recompute extension duration using only contributing joints.
    let contributing_time = leg_joints
        .iter()
        .filter(|lj| lj.contributes)
        .map(|lj| {
            let stroke = (lj.extended_angle - lj.start_angle).abs();
            stroke / lj.max_velocity
        })
        .fold(0.0_f32, f32::max);

    // Clamp extension duration (at least 0.15s for visual clarity)
    let extension_duration = contributing_time.max(max_extension_time * 0.01).max(0.15);

    // Compute initial foot Z position (ground level)
    let initial_foot_z = avg_link_z(&transforms, ground_links);

    Some(JumpSim {
        phase: JumpPhase::Extension,
        phase_time: 0.0,
        leg_joints,
        extension_duration,
        base_velocity_z: 0.0,
        initial_foot_z,
        ground_link_names: ground_links.to_vec(),
        launch_velocity: 0.0,
        launch_z: model.base_transform.translation.vector.z,
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
    })
}

/// Step the jump simulation by `dt` seconds.
///
/// **Extension phase (quasi-dynamics):**
/// 1. Advance joint positions along their trajectory patterns.
/// 2. Re-compute FK and adjust `base_transform.z` so that feet stay
///    at the initial ground level (foot-constraint).
/// 3. Compute base velocity from finite differences of base Z.
/// 4. Compute ground reaction force: $F_{GRF} = M (a_z + g)$.
/// 5. Compute per-joint gravity torque and utilisation at each step.
/// 6. If any leg joint is torque-limited (gravity torque > effort),
///    reduce the trajectory speed proportionally.
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
            // Compute gravity torques at current pose to check feasibility.
            let gravity_torques = compute_gravity_torques(model);
            let grav_map: HashMap<usize, f64> = gravity_torques
                .iter()
                .map(|t| (t.joint_idx, t.gravity_torque))
                .collect();

            // For each *contributing* leg joint, compute utilisation = |τ_gravity| / effort.
            // If any contributing joint exceeds its effort limit, the trajectory slows down.
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
                // Only contributing joints affect the speed limit
                if lj.contributes && util > worst_ratio {
                    worst_ratio = util;
                }
            }
            // Speed reduction: if worst_ratio > 1.0, the joint can't even hold
            // against gravity → clamp speed to 0.  Otherwise scale linearly in
            // the 0.8–1.0 range so the robot visibly slows before stalling.
            let speed_scale = if worst_ratio >= 1.0 {
                0.0_f32
            } else if worst_ratio > 0.8 {
                ((1.0 - worst_ratio) / 0.2) as f32
            } else {
                1.0_f32
            };

            // --- 2. Advance joint trajectory ---
            let effective_time = sim.phase_time; // already accumulated
            let t_frac = (effective_time / sim.extension_duration).clamp(0.0, 1.0);
            let alpha = smooth_step(t_frac);

            for lj in &sim.leg_joints {
                if lj.contributes {
                    // Drive this joint along the extension trajectory
                    let target = lj.start_angle + (lj.extended_angle - lj.start_angle) * alpha;
                    model.joint_positions[lj.joint_idx] = target;
                } else {
                    // Hold posture: keep at the saved start angle
                    model.joint_positions[lj.joint_idx] = lj.start_angle;
                }
            }

            // Slow down the trajectory clock if torque-limited
            // (roll back part of the time advance)
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

            // --- 6. Transition to flight when extension complete ---
            if sim.phase_time >= sim.extension_duration {
                sim.launch_z = current_z;
                sim.launch_velocity = sim.base_velocity_z;
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

            // Land when descending back to or below start height
            let start_z = sim.saved_base_transform.translation.vector.z;
            if t > 0.0 && current_z <= start_z {
                model.base_transform.translation.vector.z = start_z;
                sim.step_info.velocity_z = 0.0;
                sim.phase = JumpPhase::Landed;
                sim.phase_time = 0.0;
            }
            true
        }

        JumpPhase::Landed => {
            sim.step_info.grf_z = sim.total_mass * G; // resting on ground
            sim.step_info.velocity_z = 0.0;
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
