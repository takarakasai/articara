//! Differential Inverse Kinematics solver using Damped Least Squares (DLS).
//!
//! Given a kinematic chain of revolute joints, computes joint angle deltas
//! that move the end-effector toward a desired world-space position.

use nalgebra as na;
use std::collections::HashMap;

use crate::robot::RobotModel;

/// One entry in a kinematic chain from root to end-effector.
#[derive(Clone, Debug)]
pub struct ChainJoint {
    pub joint_idx: usize,
    pub joint_name: String,
}

/// Build the kinematic chain (list of movable joints) from root to the given link.
/// Returns joints in order from root → end-effector.
pub fn build_chain(model: &RobotModel, end_link: &str) -> Vec<ChainJoint> {
    let mut chain = Vec::new();
    let mut current = end_link.to_string();

    // Walk up from end-effector to root, collecting joints
    while let Some(ji) = model.parent_joint_of_link(&current) {
        let joint = &model.joints[ji];
        let jt = joint.joint_type.as_str();
        if jt == "revolute" || jt == "continuous" || jt == "prismatic" {
            chain.push(ChainJoint {
                joint_idx: ji,
                joint_name: joint.name.clone(),
            });
        }
        current = joint.parent_link.clone();
    }

    chain.reverse(); // root → end-effector order
    chain
}

/// Compute the Jacobian matrix for positional IK.
///
/// For each revolute joint in the chain, the Jacobian column is:
///   J_i = axis_i × (p_ee - p_joint_i)
///
/// For prismatic joints:
///   J_i = axis_i
///
/// Returns a 3×N matrix where N is the number of joints in the chain.
pub fn compute_jacobian(
    model: &RobotModel,
    chain: &[ChainJoint],
    transforms: &HashMap<String, na::Isometry3<f32>>,
    ee_pos: &na::Point3<f32>,
) -> na::DMatrix<f32> {
    let n = chain.len();
    let mut jac = na::DMatrix::zeros(3, n);

    for (col, cj) in chain.iter().enumerate() {
        let joint = &model.joints[cj.joint_idx];
        let parent_tf = transforms
            .get(&joint.parent_link)
            .copied()
            .unwrap_or(na::Isometry3::identity());
        let joint_tf = parent_tf * joint.origin;
        let world_axis = joint_tf.rotation * joint.axis;
        let joint_pos = na::Point3::from(joint_tf.translation.vector);

        match joint.joint_type.as_str() {
            "revolute" | "continuous" => {
                let r = ee_pos - joint_pos;
                let j_col = world_axis.cross(&r);
                jac[(0, col)] = j_col.x;
                jac[(1, col)] = j_col.y;
                jac[(2, col)] = j_col.z;
            }
            "prismatic" => {
                jac[(0, col)] = world_axis.x;
                jac[(1, col)] = world_axis.y;
                jac[(2, col)] = world_axis.z;
            }
            _ => {}
        }
    }

    jac
}

/// Solve one step of positional IK using Damped Least Squares.
///
/// Given:
///   - Current end-effector position
///   - Desired end-effector position (target)
///   - The kinematic chain
///
/// Computes joint angle deltas: Δq = J^T (J J^T + λ²I)^{-1} Δx
///
/// Returns the joint angle deltas for each joint in the chain.
pub fn solve_ik_step(
    model: &RobotModel,
    chain: &[ChainJoint],
    transforms: &HashMap<String, na::Isometry3<f32>>,
    ee_pos: &na::Point3<f32>,
    target_pos: &na::Point3<f32>,
    damping: f32,
    max_step: f32,
) -> Vec<f32> {
    let n = chain.len();
    if n == 0 {
        return Vec::new();
    }

    // Position error
    let dx = target_pos - ee_pos;

    // Clamp error magnitude to prevent large jumps
    let error_mag = dx.norm();
    let dx_clamped = if error_mag > max_step {
        dx * (max_step / error_mag)
    } else {
        dx
    };

    let dx_vec = na::DVector::from_column_slice(&[dx_clamped.x, dx_clamped.y, dx_clamped.z]);

    // Compute Jacobian
    let jac = compute_jacobian(model, chain, transforms, ee_pos);

    // DLS: Δq = J^T (J J^T + λ²I)^{-1} Δx
    let jjt = &jac * jac.transpose();
    let lambda_sq = damping * damping;
    let identity = na::DMatrix::identity(3, 3);
    let jjt_reg = jjt + identity * lambda_sq;

    // Solve (J J^T + λ²I) y = Δx, then Δq = J^T y
    let decomp = jjt_reg.lu();
    let y = decomp.solve(&dx_vec).unwrap_or(na::DVector::zeros(3));
    let dq = jac.transpose() * y;

    (0..n).map(|i| dq[i]).collect()
}

/// Get the world position of the end-effector (center of the link's bounding sphere).
pub fn get_ee_world_pos(
    model: &RobotModel,
    link_idx: usize,
    transforms: &HashMap<String, na::Isometry3<f32>>,
) -> na::Point3<f32> {
    let link_name = &model.links[link_idx].name;
    let world_tf = transforms
        .get(link_name)
        .copied()
        .unwrap_or(na::Isometry3::identity());
    let (local_center, _) = model.link_bounding_sphere(link_idx);
    world_tf * local_center
}

/// Apply IK deltas to joint positions, respecting limits.
pub fn apply_ik_deltas(
    model: &mut RobotModel,
    chain: &[ChainJoint],
    deltas: &[f32],
) {
    for (i, cj) in chain.iter().enumerate() {
        let ji = cj.joint_idx;
        let lower = model.joints[ji].lower as f32;
        let upper = model.joints[ji].upper as f32;
        model.joint_positions[ji] = (model.joint_positions[ji] + deltas[i]).clamp(lower, upper);
    }
}
