//! Adapter between `RobotModel` (articara) and `misarta::Model<f64>`.
//!
//! Converts the GUI-oriented `RobotModel` into the Pinocchio-style
//! `misarta::Model<f64>` so that misarta's algorithms (CRBA, RNEA, ABA,
//! FK, Jacobians, …) can be called on articara robot data.
//!
//! # Design
//!
//! - The adapter is built **once** from a `RobotModel` snapshot and can be
//!   reused across many algorithm calls (the misarta `Model` is immutable).
//! - Configuration (`q`) and velocity (`v`) vectors are assembled from the
//!   mutable `RobotModel` state on each call.
//! - Joint-index mappings allow translating results back to articara's
//!   joint-index space.

use std::collections::HashMap;

use nalgebra as na;
use na::Matrix3;

use misarta::joint::JointType;
use misarta::model::{LinkInertia, Model, ModelBuilder};

use super::model::RobotModel;

/// Cached mapping between an `articara::RobotModel` and a `misarta::Model<f64>`.
#[derive(Clone, Debug)]
pub struct ModelAdapter {
    /// The misarta model built from the `RobotModel`.
    pub model: Model<f64>,
    /// `articara_to_misarta[articara_joint_idx]` → misarta joint index (1-based).
    /// `None` if the articara joint was not included (shouldn't happen normally).
    pub articara_to_misarta: Vec<Option<usize>>,
    /// `misarta_to_articara[misarta_joint_idx]` → articara joint index.
    /// Index 0 (universe) maps to `None`.
    pub misarta_to_articara: Vec<Option<usize>>,
}

impl ModelAdapter {
    /// Build a `ModelAdapter` from a `RobotModel`.
    ///
    /// This traverses the kinematic tree (BFS from root) and creates a
    /// corresponding `misarta::Model<f64>`, including **all** joints
    /// (fixed, revolute, continuous, prismatic).
    pub fn from_robot_model(robot: &RobotModel) -> Self {
        let mut builder = ModelBuilder::<f64>::new()
            .name(robot.name.clone())
            .root_link_name(robot.root_link.clone());

        // Set root link inertia.
        if let Some(&root_li) = robot.link_map.get(&robot.root_link) {
            let root_inertia = convert_link_inertia(&robot.links[root_li]);
            // The builder starts with LinkInertia::zero() at index 0.
            // We need to set it properly. The builder doesn't expose a setter,
            // so we'll reconstruct.  Actually, we keep the default zero for
            // the universe and handle root via the first joint if needed.
            // For robots with a meaningful root-link mass, we can add it later.
            // Most URDFs have the root link mass at zero or very small.
            let _ = root_inertia; // will be addressed below
        }

        // Gravity: misarta defaults to (0, 0, -9.81). Match that.
        builder = builder.gravity(na::Vector3::new(0.0, 0.0, -9.81));

        // Maps for index translation.
        let mut articara_to_misarta: Vec<Option<usize>> = vec![None; robot.joints.len()];
        let mut misarta_to_articara: Vec<Option<usize>> = vec![None]; // index 0 = universe

        // For tree traversal we need to know which misarta joint index
        // corresponds to the articara link that is a child of a given joint.
        // link_name → misarta joint index that has this link as its child.
        let mut link_to_misarta_idx: HashMap<String, usize> = HashMap::new();
        link_to_misarta_idx.insert(robot.root_link.clone(), 0); // root → universe

        // BFS from the root link.
        let mut queue = vec![robot.root_link.clone()];
        while let Some(link_name) = queue.pop() {
            let parent_misarta_idx = link_to_misarta_idx[&link_name];

            if let Some(child_joint_indices) = robot.children_joints.get(&link_name) {
                for &ji in child_joint_indices {
                    let joint = &robot.joints[ji];
                    let joint_type = convert_joint_type(joint);
                    let placement = joint.origin.cast::<f64>();

                    // Get child link inertia.
                    let child_link_name = &joint.child_link;
                    let inertia = robot
                        .link_map
                        .get(child_link_name)
                        .map(|&li| convert_link_inertia(&robot.links[li]))
                        .unwrap_or_else(LinkInertia::zero);

                    builder = builder.add_joint_with_link(
                        joint.name.clone(),
                        parent_misarta_idx,
                        joint_type,
                        placement,
                        inertia,
                        child_link_name.clone(),
                    );

                    // The new joint gets the next index (= current length of joints vec
                    // before this push, but the builder does push inside add_joint_with_link,
                    // so the index is: prev joints count).
                    // After build(), joints[0] = universe, joints[1] = first added, etc.
                    let misarta_idx = misarta_to_articara.len(); // next index
                    articara_to_misarta[ji] = Some(misarta_idx);
                    misarta_to_articara.push(Some(ji));
                    link_to_misarta_idx.insert(child_link_name.clone(), misarta_idx);

                    queue.push(child_link_name.clone());
                }
            }
        }

        // Handle root link inertia: inject into the builder's inertias[0].
        // We'll do this after build by mutating the model (it's just a struct).
        let mut model = builder.build();

        if let Some(&root_li) = robot.link_map.get(&robot.root_link) {
            model.inertias[0] = convert_link_inertia(&robot.links[root_li]);
        }

        Self {
            model,
            articara_to_misarta,
            misarta_to_articara,
        }
    }

    /// Build the full configuration vector `q` from the current
    /// `RobotModel.joint_positions`.
    pub fn build_q(&self, robot: &RobotModel) -> Vec<f64> {
        let mut q = self.model.neutral_q();
        for (ji, &maybe_mi) in self.articara_to_misarta.iter().enumerate() {
            if let Some(mi) = maybe_mi {
                let nq = self.model.joints[mi].joint_type.nq();
                if nq == 1 {
                    let qi = self.model.q_idx[mi];
                    q[qi] = robot.joint_positions[ji] as f64;
                }
                // nq == 0 (fixed): nothing to do
                // nq == 7 (free-flyer): not used in articara's RobotModel
            }
        }
        q
    }

    /// Build the full velocity vector `v` from a sparse velocity map
    /// (keyed by articara joint index).
    pub fn build_v(
        &self,
        velocities: &HashMap<usize, f64>,
    ) -> na::DVector<f64> {
        let mut v = na::DVector::zeros(self.model.nv);
        for (&ji, &qd) in velocities {
            if let Some(mi) = self.articara_to_misarta.get(ji).and_then(|&m| m) {
                let nv = self.model.joints[mi].joint_type.nv();
                if nv == 1 {
                    let vi = self.model.v_idx[mi];
                    v[vi] = qd;
                }
            }
        }
        v
    }

    /// Map a result vector indexed by misarta v-indices to a vector indexed
    /// by positions in `joint_order` (articara joint indices).
    ///
    /// Returns both the mapped vector and an `idx_in_result` table
    /// (`idx_in_result[articara_ji]` → column in result, or `None`).
    pub fn extract_subvector(
        &self,
        full: &na::DVector<f64>,
        joint_order: &[usize],
    ) -> (na::DVector<f64>, Vec<Option<usize>>) {
        let n = joint_order.len();
        let mut sub = na::DVector::zeros(n);
        let mut idx_in_result: Vec<Option<usize>> = vec![None; self.articara_to_misarta.len()];

        for (col, &ji) in joint_order.iter().enumerate() {
            if let Some(mi) = self.articara_to_misarta.get(ji).and_then(|&m| m) {
                let nv = self.model.joints[mi].joint_type.nv();
                if nv == 1 {
                    let vi = self.model.v_idx[mi];
                    sub[col] = full[vi];
                }
            }
            idx_in_result[ji] = Some(col);
        }

        (sub, idx_in_result)
    }

    /// Extract a sub-matrix from a full nv×nv matrix for the joints in
    /// `joint_order`.
    ///
    /// Returns both the N×N sub-matrix and an `idx_in_M` table.
    pub fn extract_submatrix(
        &self,
        full: &na::DMatrix<f64>,
        joint_order: &[usize],
    ) -> (na::DMatrix<f64>, Vec<Option<usize>>) {
        let n = joint_order.len();
        let mut sub = na::DMatrix::zeros(n, n);
        let mut idx_in_m: Vec<Option<usize>> = vec![None; self.articara_to_misarta.len()];

        // Collect misarta v-indices for each joint in the order.
        let mut v_indices: Vec<Option<usize>> = Vec::with_capacity(n);
        for (col, &ji) in joint_order.iter().enumerate() {
            idx_in_m[ji] = Some(col);
            if let Some(mi) = self.articara_to_misarta.get(ji).and_then(|&m| m) {
                let nv = self.model.joints[mi].joint_type.nv();
                if nv == 1 {
                    v_indices.push(Some(self.model.v_idx[mi]));
                } else {
                    v_indices.push(None);
                }
            } else {
                v_indices.push(None);
            }
        }

        for (r, vi_r) in v_indices.iter().enumerate() {
            for (c, vi_c) in v_indices.iter().enumerate() {
                if let (Some(vr), Some(vc)) = (vi_r, vi_c) {
                    sub[(r, c)] = full[(*vr, *vc)];
                }
            }
        }

        (sub, idx_in_m)
    }
}

// ─── Conversion helpers ─────────────────────────────────────────────────────

/// Convert an articara `JointData.joint_type` string + axis to a misarta `JointType`.
fn convert_joint_type(joint: &super::model::JointData) -> JointType<f64> {
    let axis = joint.axis.cast::<f64>();
    match joint.joint_type.as_str() {
        "revolute" | "continuous" => JointType::Revolute {
            axis: na::Unit::new_normalize(axis).into_inner(),
        },
        "prismatic" => JointType::Prismatic {
            axis: na::Unit::new_normalize(axis).into_inner(),
        },
        _ => JointType::Fixed,
    }
}

/// Convert an articara `LinkData` inertial properties to a misarta `LinkInertia`.
fn convert_link_inertia(link: &super::model::LinkData) -> LinkInertia<f64> {
    let i = &link.inertial;
    let mass = i.mass;
    let com = i.origin.translation.vector.cast::<f64>();
    let rot = i.origin.rotation.to_rotation_matrix();
    let r = rot.matrix().cast::<f64>();

    // URDF inertia is given at the CoM in the inertial frame.
    // misarta expects rotational_inertia expressed in the body (link) frame.
    // Apply the inertial frame's rotation: I_body = R * I_com * Rᵀ
    let i_com = Matrix3::new(
        i.ixx, i.ixy, i.ixz,
        i.ixy, i.iyy, i.iyz,
        i.ixz, i.iyz, i.izz,
    );
    let rotational_inertia = &r * &i_com * r.transpose();

    LinkInertia {
        mass,
        center_of_mass: com,
        rotational_inertia,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::path::Path;

    #[test]
    fn adapter_round_trip_joint_count() {
        // Load a URDF with both articara and misarta, compare joint counts.
        let urdf_path = Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("sample/namiashi_description/urdf/namiashi.urdf");
        if !urdf_path.exists() {
            return; // skip if sample not present
        }
        let robot = RobotModel::from_urdf(&urdf_path).unwrap();
        let adapter = ModelAdapter::from_robot_model(&robot);

        // Misarta model should have same total joints (universe + all joints).
        let movable_articara = robot
            .joints
            .iter()
            .filter(|j| matches!(j.joint_type.as_str(), "revolute" | "continuous" | "prismatic"))
            .count();
        assert_eq!(adapter.model.nv, movable_articara);
    }
}
