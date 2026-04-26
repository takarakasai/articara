/// Core dynamics algorithms for rigid-body robots.
//
/// Provides:
// - **Gravity torque computation**: static torque at each joint due to gravity.
// - **Descendant-link tree traversal** (used by torque computations).
// - **CRBA** (Composite Rigid Body Algorithm): joint-space inertia matrix M(q).
// - **RNEA** (Recursive Newton-Euler Algorithm): inverse dynamics / bias forces.
// - **Constrained forward dynamics**: solve q̈ with ground-contact constraints.
// - **Forward dynamics state & integrator**: semi-implicit Euler stepping.
//
/// # Conventions
//
// All 6D spatial vectors use the **body-fixed** convention with ordering
// `[angular(3); linear(3)]` (Featherstone convention).
//
// The robot is assumed to have a **fixed base** (the URDF root link is
// rigidly attached to the world).  The floating-base extension (6 virtual
// DOFs) is *not* modelled — instead, the base translation is advanced as
/// a rigid body driven by the net ground reaction force.

use nalgebra as na;
use std::collections::HashMap;

use super::model::{RobotModel, MisartaCache};

// ========== Constants ==========

/// Standard gravitational acceleration (m/s²).
pub const G: f64 = 9.80665;
/// Gravity vector (pointing downward in Z-down convention).
pub const G_VEC: na::Vector3<f64> = na::Vector3::new(0.0, 0.0, -G);

// ========== Tree helpers ==========

/// Return **movable** joint indices in parent-first (topological) order.
///
/// "Movable" means `revolute`, `continuous`, or `prismatic`.
/// The ordering guarantees that a parent joint always appears before its
/// child joints in the returned list.
pub fn topological_joint_order(model: &RobotModel) -> Vec<usize> {
    let mut order = Vec::new();
    let mut stack = vec![model.root_link.clone()];
    while let Some(link) = stack.pop() {
        if let Some(child_joints) = model.children_joints.get(&link) {
            for &ji in child_joints {
                let jt = model.joints[ji].joint_type.as_str();
                if jt == "revolute" || jt == "continuous" || jt == "prismatic" {
                    order.push(ji);
                }
                stack.push(model.joints[ji].child_link.clone());
            }
        }
    }
    order
}

// =========================================================================
//  CRBA — Composite Rigid Body Algorithm
// =========================================================================

/// Compute the joint-space inertia matrix **M(q)** using the Composite
/// Rigid Body Algorithm (CRBA).
///
/// Returns an N×N symmetric positive-definite matrix where N is the number
/// of movable joints (in topological order).
///
/// The `joint_order` parameter must be the output of [`topological_joint_order`].
/// `idx_in_M[joint_idx]` maps a global joint index to its column/row in M
/// (or `None` if the joint is fixed).
///
/// Delegates to [`misarta::crba::crba`] and extracts the submatrix for the
/// requested joint subset.  Reuses a pre-built [`ModelAdapter`].
pub fn crba(
    model: &RobotModel,
    joint_order: &[usize],
    mc: &MisartaCache,
) -> (na::DMatrix<f64>, Vec<Option<usize>>) {
    let q = mc.build_q(model);
    let m_full = misarta::crba::crba(&mc.model, &q);
    mc.extract_submatrix(&m_full, joint_order)
}

// =========================================================================
//  RNEA — Recursive Newton-Euler Algorithm (inverse dynamics)
// =========================================================================

/// Compute the **bias force** vector  h(q, q̇) = C(q, q̇)·q̇ + g(q)
/// using the Recursive Newton-Euler Algorithm.
///
/// Returns an N×1 vector (one entry per movable joint in `joint_order`).
///
/// `joint_velocities` maps global joint index → q̇.
///
/// Delegates to [`misarta::rnea::nonlinear_effects`] and extracts the
/// entries for the requested joint subset.
#[allow(dead_code)]
pub fn rnea_bias(
    model: &RobotModel,
    joint_order: &[usize],
    joint_velocities: &HashMap<usize, f64>,
) -> na::DVector<f64> {
    let mc = model.mc();
    rnea_bias_with_mc(model, joint_order, joint_velocities, mc)
}

/// Like [`rnea_bias`] but reuses a pre-built [`MisartaCache`].
pub fn rnea_bias_with_mc(
    model: &RobotModel,
    joint_order: &[usize],
    joint_velocities: &HashMap<usize, f64>,
    mc: &MisartaCache,
) -> na::DVector<f64> {
    let q = mc.build_q(model);
    let v = mc.build_v(joint_velocities);
    let h_full = misarta::rnea::nonlinear_effects(&mc.model, &q, v.as_slice());
    let (h_sub, _) = mc.extract_subvector(&h_full, joint_order);
    h_sub
}

// =========================================================================
//  Forward dynamics (unconstrained)
// =========================================================================

/// Solve **unconstrained** forward dynamics:
///
///   M(q) q̈ = τ − h(q, q̇)
///
/// Returns q̈ as a DVector (one entry per joint in `joint_order`).
pub fn forward_dynamics(
    m_mat: &na::DMatrix<f64>,
    h: &na::DVector<f64>,
    tau: &na::DVector<f64>,
) -> na::DVector<f64> {
    let rhs = tau - h;
    // Solve M q̈ = rhs  via Cholesky (M is SPD)
    match na::linalg::Cholesky::new(m_mat.clone()) {
        Some(chol) => chol.solve(&rhs),
        None => {
            // Fallback: LU decomposition (less efficient, but handles near-singular)
            m_mat.clone().lu().solve(&rhs).unwrap_or_else(|| na::DVector::zeros(h.len()))
        }
    }
}

// =========================================================================
//  Constrained forward dynamics (ground contact)
// =========================================================================

/// Compute the 3×N positional Jacobian **J_foot** for a foot link,
/// mapping joint velocities → foot Cartesian velocity.
///
/// Delegates to `ModelAdapter::foot_positional_jacobian` which uses
/// misarta's relative Jacobian internally.
///
/// Only joints in `joint_order` are included (mapped via `idx_in_m`).
pub fn foot_jacobian(
    model: &RobotModel,
    foot_link: &str,
    body_link: &str,
    joint_order: &[usize],
    idx_in_m: &[Option<usize>],
) -> na::DMatrix<f64> {
    model.foot_positional_jacobian(foot_link, body_link, joint_order, idx_in_m)
}


// ========== Gravity torque ==========

/// Compute the gravity torque about a joint axis from a specific set of links.
///
/// `joint_pos` is the joint position in world frame.
/// `world_axis` is the joint axis in world frame.
/// `link_indices` are the links to sum over.
pub fn gravity_torque_from_links(
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
/// Uses misarta's `compute_gravity` for full gravity vector, then computes
/// body-side as (total - descendant) using the per-joint gravity values.
///
/// Returns a map: joint_idx → body-side gravity torque.
pub fn compute_body_side_gravity_torques(
    model: &RobotModel,
    joints: &[usize],
) -> HashMap<usize, f64> {
    let mc = model.mc();
    let q = mc.build_q(model);
    let g_full = misarta::rnea::compute_gravity(&mc.model, &q);

    // Build a lookup: misarta joint idx → gravity torque value
    let g_for = |ji: usize| -> f64 {
        if let Some(mi) = mc.a2m.get(ji).and_then(|&m| m) {
            let nv = mc.model.joints[mi].joint_type.nv();
            if nv == 1 {
                return g_full[mc.model.v_idx[mi]];
            }
        }
        0.0
    };

    // For body-side torque we need: total_gravity - descendant_gravity.
    // descendant_gravity = g(q) for the full model, but restricted to the sub-tree.
    // However, g(q) from RNEA *already* is the descendant torque (it sums moments
    // from all descendant links). So body-side = total_from_all_links - g(q).
    // We compute total_from_all_links using the old method.
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

        let tau_all = gravity_torque_from_links(
            model, &transforms, &joint_pos, &world_axis, jt, &all_link_indices,
        );

        let tau_descendants = g_for(ji);
        let tau_body_side = tau_all - tau_descendants;
        result.insert(ji, tau_body_side);
    }

    result
}

// =========================================================================
//  ABA — Articulated Body Algorithm (O(n) forward dynamics)
// =========================================================================

/// Compute forward dynamics via ABA: q̈ = M(q)⁻¹ (τ − C(q,q̇)q̇ − g(q))
///
/// This is O(n) and avoids forming M(q) explicitly.
/// Returns q̈ (one entry per joint in `joint_order`).
#[allow(dead_code)]
pub fn aba_forward_dynamics(
    model: &RobotModel,
    joint_order: &[usize],
    joint_velocities: &HashMap<usize, f64>,
    joint_torques: &HashMap<usize, f64>,
    mc: &MisartaCache,
) -> na::DVector<f64> {
    let q = mc.build_q(model);
    let v = mc.build_v(joint_velocities);
    let mut tau = na::DVector::zeros(mc.model.nv);
    for (&ji, &t) in joint_torques {
        if let Some(mi) = mc.a2m.get(ji).and_then(|&m| m) {
            let nv = mc.model.joints[mi].joint_type.nv();
            if nv == 1 {
                tau[mc.model.v_idx[mi]] = t;
            }
        }
    }
    let qdd_full = misarta::aba::aba(&mc.model, &q, v.as_slice(), tau.as_slice());
    let (qdd_sub, _) = mc.extract_subvector(&qdd_full, joint_order);
    qdd_sub
}

/// Compute M(q)⁻¹ τ using the O(n) ABA without forming M explicitly.
#[allow(dead_code)]
pub fn minv_times_vec(
    model: &RobotModel,
    joint_order: &[usize],
    joint_torques: &HashMap<usize, f64>,
    mc: &MisartaCache,
) -> na::DVector<f64> {
    let q = mc.build_q(model);
    let mut tau = na::DVector::zeros(mc.model.nv);
    for (&ji, &t) in joint_torques {
        if let Some(mi) = mc.a2m.get(ji).and_then(|&m| m) {
            let nv = mc.model.joints[mi].joint_type.nv();
            if nv == 1 {
                tau[mc.model.v_idx[mi]] = t;
            }
        }
    }
    let result = misarta::aba::compute_minv_times_vec(&mc.model, &q, tau.as_slice());
    let (sub, _) = mc.extract_subvector(&result, joint_order);
    sub
}

// =========================================================================
//  Centroidal dynamics — CoM, momentum
// =========================================================================

/// Compute world-frame center of mass position.
#[allow(dead_code)]
pub fn compute_com(model: &RobotModel, mc: &MisartaCache) -> na::Point3<f64> {
    let q = mc.build_q(model);
    let com = misarta::centroidal::compute_com(&mc.model, &q);
    na::Point3::from(com)
}

/// Compute total robot mass via misarta.
#[allow(dead_code)]
pub fn total_mass(mc: &MisartaCache) -> f64 {
    misarta::centroidal::total_mass(&mc.model)
}

/// Compute the CoM Jacobian (3 × nv), mapping generalized velocity to CoM velocity.
#[allow(dead_code)]
pub fn compute_com_jacobian(
    model: &RobotModel,
    joint_order: &[usize],
    mc: &MisartaCache,
) -> na::DMatrix<f64> {
    let q = mc.build_q(model);
    let j_full = misarta::centroidal::compute_com_jacobian(&mc.model, &q);
    let n = joint_order.len();
    let mut j_sub = na::DMatrix::zeros(3, n);
    for (col, &ji) in joint_order.iter().enumerate() {
        if let Some(mi) = mc.a2m.get(ji).and_then(|&m| m) {
            let nv = mc.model.joints[mi].joint_type.nv();
            if nv == 1 {
                let vi = mc.model.v_idx[mi];
                for r in 0..3 {
                    j_sub[(r, col)] = j_full[(r, vi)];
                }
            }
        }
    }
    j_sub
}

/// Compute the 6D centroidal momentum matrix (6 × nv).
#[allow(dead_code)]
pub fn compute_centroidal_momentum_matrix(
    model: &RobotModel,
    joint_order: &[usize],
    mc: &MisartaCache,
) -> na::DMatrix<f64> {
    let q = mc.build_q(model);
    let ag_full = misarta::centroidal::compute_centroidal_momentum_matrix(&mc.model, &q);
    let n = joint_order.len();
    let mut ag_sub = na::DMatrix::zeros(6, n);
    for (col, &ji) in joint_order.iter().enumerate() {
        if let Some(mi) = mc.a2m.get(ji).and_then(|&m| m) {
            let nv = mc.model.joints[mi].joint_type.nv();
            if nv == 1 {
                let vi = mc.model.v_idx[mi];
                for r in 0..6 {
                    ag_sub[(r, col)] = ag_full[(r, vi)];
                }
            }
        }
    }
    ag_sub
}

// =========================================================================
//  iLQR optimal control — re-export from misarta
// =========================================================================

/// Re-export misarta's iLQR types and solver so articara callers
/// can use them through the `rbd::dynamics` namespace.
#[allow(unused_imports)]
pub use misarta::optimization::{
    IlqrConfig, IlqrResult, solve_ilqr,
    discrete_dynamics_step,
};