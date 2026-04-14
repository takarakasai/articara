//! Kinematic chain building, Jacobian computation, and IK solver.
//!
//! Supports cross-branch IK chains through the tree (via LCA) with
//! inverted joints for re-rooted kinematics.

use nalgebra as na;
use std::collections::{HashMap, HashSet};

use super::model::RobotModel;

/// One entry in a kinematic chain from root to end-effector.
#[derive(Clone, Debug)]
pub struct ChainJoint {
    pub joint_idx: usize,
    pub joint_name: String,
    /// If true, this joint is on the "upward" path from the IK root toward the
    /// LCA. Rotating it effectively moves the body in the opposite direction
    /// (the IK root is fixed, so the rest of the tree moves).
    pub inverted: bool,
}

/// Build the kinematic chain (list of movable joints) from URDF root to the given link.
/// Returns joints in order from root → end-effector. All joints have `inverted = false`.
pub fn build_chain(model: &RobotModel, end_link: &str) -> Vec<ChainJoint> {
    let mut chain = Vec::new();
    let mut current = end_link.to_string();

    while let Some(ji) = model.parent_joint_of_link(&current) {
        let joint = &model.joints[ji];
        let jt = joint.joint_type.as_str();
        if jt == "revolute" || jt == "continuous" || jt == "prismatic" {
            chain.push(ChainJoint {
                joint_idx: ji,
                joint_name: joint.name.clone(),
                inverted: false,
            });
        }
        current = joint.parent_link.clone();
    }

    chain.reverse();
    chain
}

/// Build a kinematic chain between two arbitrary links in the tree.
///
/// The chain goes from `root_link` (treated as the fixed base) through the
/// Lowest Common Ancestor (LCA) to `end_link` (the end-effector).
///
/// Joints on the path from `root_link` up to the LCA are marked `inverted = true`,
/// meaning their Jacobian contribution is negated (rotating the joint moves the
/// body opposite to the URDF convention because the IK root is fixed).
///
/// Joints on the path from the LCA down to `end_link` are `inverted = false`.
///
/// If `root_link` is `None`, behaves like `build_chain` (full chain to URDF root).
pub fn build_chain_between(
    model: &RobotModel,
    end_link: &str,
    root_link: Option<&str>,
) -> Vec<ChainJoint> {
    let root_link = match root_link {
        Some(r) => r,
        None => return build_chain(model, end_link),
    };

    if root_link == end_link {
        return Vec::new();
    }

    // Find ancestors of both links
    let ancestors_root = ancestors_list(model, root_link);
    let ancestors_end = ancestors_list(model, end_link);

    // Find lowest common ancestor (LCA)
    let ancestor_set: HashSet<&str> = ancestors_root.iter().map(|s| s.as_str()).collect();
    let lca = ancestors_end
        .iter()
        .find(|a| ancestor_set.contains(a.as_str()))
        .cloned()
        .unwrap_or_else(|| model.root_link.clone());

    // Path from root_link up to LCA (inverted joints)
    let up_joints = collect_path_up(model, root_link, &lca, true);

    // Path from LCA down to end_link (normal joints)
    let down_joints = collect_path_up(model, end_link, &lca, false);

    // Combine: up_joints are already in root→LCA order, down_joints need reversing
    let mut chain = up_joints;
    let mut down_reversed: Vec<ChainJoint> = down_joints;
    down_reversed.reverse();
    chain.extend(down_reversed);

    chain
}

/// Walk from `from_link` up to `to_ancestor`, collecting movable joints.
/// Returns joints in from→ancestor order.
/// If `inverted` is true, joints are marked as inverted.
fn collect_path_up(
    model: &RobotModel,
    from_link: &str,
    to_ancestor: &str,
    inverted: bool,
) -> Vec<ChainJoint> {
    let mut joints = Vec::new();
    let mut current = from_link.to_string();

    while current != to_ancestor {
        if let Some(ji) = model.parent_joint_of_link(&current) {
            let joint = &model.joints[ji];
            let jt = joint.joint_type.as_str();
            if jt == "revolute" || jt == "continuous" || jt == "prismatic" {
                joints.push(ChainJoint {
                    joint_idx: ji,
                    joint_name: joint.name.clone(),
                    inverted,
                });
            }
            current = joint.parent_link.clone();
        } else {
            break; // reached URDF root
        }
    }

    joints
}

/// Get the list of ancestor links (including the link itself) from a link up to the URDF root.
fn ancestors_list(model: &RobotModel, link: &str) -> Vec<String> {
    let mut ancestors = vec![link.to_string()];
    let mut current = link.to_string();
    while let Some(ji) = model.parent_joint_of_link(&current) {
        current = model.joints[ji].parent_link.clone();
        ancestors.push(current.clone());
    }
    ancestors
}

// Keep backward-compatible wrapper.
/// Build chain with a specified root that must be an ancestor of end_link.
/// Deprecated in favor of `build_chain_between`.
pub fn build_chain_with_root(
    model: &RobotModel,
    end_link: &str,
    root_link: Option<&str>,
) -> Vec<ChainJoint> {
    build_chain_between(model, end_link, root_link)
}

/// Compute the Jacobian matrix for positional IK.
///
/// For each revolute joint in the chain, the Jacobian column is:
///   J_i = sign_i * axis_i × (p_ee - p_joint_i)
///
/// For prismatic joints:
///   J_i = sign_i * axis_i
///
/// where `sign_i` is -1 for inverted joints, +1 for normal joints.
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

        let sign: f32 = if cj.inverted { -1.0 } else { 1.0 };

        match joint.joint_type.as_str() {
            "revolute" | "continuous" => {
                let r = ee_pos - joint_pos;
                let j_col = world_axis.cross(&r) * sign;
                jac[(0, col)] = j_col.x;
                jac[(1, col)] = j_col.y;
                jac[(2, col)] = j_col.z;
            }
            "prismatic" => {
                jac[(0, col)] = world_axis.x * sign;
                jac[(1, col)] = world_axis.y * sign;
                jac[(2, col)] = world_axis.z * sign;
            }
            _ => {}
        }
    }

    jac
}

/// Solve one step of positional IK using Damped Least Squares.
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

    let dx = target_pos - ee_pos;

    let error_mag = dx.norm();
    let dx_clamped = if error_mag > max_step {
        dx * (max_step / error_mag)
    } else {
        dx
    };

    let dx_vec = na::DVector::from_column_slice(&[dx_clamped.x, dx_clamped.y, dx_clamped.z]);

    let jac = compute_jacobian(model, chain, transforms, ee_pos);

    let jjt = &jac * jac.transpose();
    let lambda_sq = damping * damping;
    let identity = na::DMatrix::identity(3, 3);
    let jjt_reg = jjt + identity * lambda_sq;

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
