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
use misarta::geometry::{GeometryModel, GeometryObject, GeometryShape};
use misarta::mesh::MeshData;

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

    /// Compute FK via misarta and return results in articara's
    /// `HashMap<String, Isometry3<f32>>` format (link-name keyed, f32).
    ///
    /// Applies `robot.base_transform` to all results.
    pub fn compute_transforms_compat(
        &self,
        robot: &RobotModel,
    ) -> HashMap<String, na::Isometry3<f32>> {
        let q = self.build_q(robot);
        let data = misarta::fk::forward_kinematics(&self.model, &q);

        let mut transforms: HashMap<String, na::Isometry3<f32>> = HashMap::new();

        // Root link → base_transform (universe joint, oMi[0] = identity)
        transforms.insert(robot.root_link.clone(), robot.base_transform);

        // For each misarta joint i (1-based), oMi[i] is the world pose
        // of the joint's child link frame.
        for i in 1..self.model.joints.len() {
            let link_name = &self.model.link_names[i];
            let world_pose_f64 = &data.oMi[i];
            // Cast f64 → f32 and pre-multiply by base_transform
            let world_pose_f32 = robot.base_transform * isometry_f64_to_f32(world_pose_f64);
            transforms.insert(link_name.clone(), world_pose_f32);
        }

        transforms
    }

    /// Look up the misarta joint index for a given articara link name.
    ///
    /// Returns `None` if the link is the root link (universe) or not found.
    pub fn link_name_to_misarta_joint(&self, link_name: &str) -> Option<usize> {
        self.model.link_names.iter().position(|n| n == link_name)
    }

    /// Build a `GeometryModel` for collision geometry from the `RobotModel`.
    pub fn build_collision_geometry(&self, robot: &RobotModel) -> GeometryModel {
        self.build_collision_geometry_with_map(robot).0
    }

    /// Build a `GeometryModel` for collision geometry from the `RobotModel`,
    /// plus a map from geometry-object index → `(link_idx, collision_idx)`.
    pub fn build_collision_geometry_with_map(
        &self,
        robot: &RobotModel,
    ) -> (GeometryModel, Vec<(usize, usize)>) {
        let mut gmodel = GeometryModel::new();
        let mut geo_map: Vec<(usize, usize)> = Vec::new();

        for (_li, link) in robot.links.iter().enumerate() {
            let parent_joint = self
                .link_name_to_misarta_joint(&link.name)
                .unwrap_or(0);
            let li = robot.link_map.get(&link.name).copied().unwrap_or(0);

            for (ci, col) in link.collisions.iter().enumerate() {
                let (shape, mesh_data) = match convert_geom_to_shape_with_mesh(&col.geometry) {
                    Some(pair) => pair,
                    None => continue,
                };
                let placement = col.origin.cast::<f64>();
                let mesh_scale = match &col.geometry {
                    super::model::GeomData::Mesh { scale, .. } => {
                        scale.map(|s| na::Vector3::new(s[0] as f64, s[1] as f64, s[2] as f64))
                    }
                    _ => None,
                };
                gmodel.add(GeometryObject {
                    name: format!("{}_collision_{}", link.name, ci),
                    parent_joint,
                    placement,
                    shape,
                    mesh_path: None,
                    mesh_scale,
                    mesh_data,
                    material: None,
                });
                geo_map.push((li, ci));
            }
        }

        (gmodel, geo_map)
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

/// Cast an `Isometry3<f64>` to `Isometry3<f32>`.
fn isometry_f64_to_f32(iso: &na::Isometry3<f64>) -> na::Isometry3<f32> {
    na::Isometry3::from_parts(
        na::Translation3::new(
            iso.translation.x as f32,
            iso.translation.y as f32,
            iso.translation.z as f32,
        ),
        na::UnitQuaternion::new_normalize(na::Quaternion::new(
            iso.rotation.w as f32,
            iso.rotation.i as f32,
            iso.rotation.j as f32,
            iso.rotation.k as f32,
        )),
    )
}

/// Convert an articara `GeomData` to a misarta `GeometryShape`,
/// optionally returning `MeshData` for mesh shapes.
fn convert_geom_to_shape_with_mesh(
    geom: &super::model::GeomData,
) -> Option<(GeometryShape, Option<MeshData>)> {
    match geom {
        super::model::GeomData::Box { hx, hy, hz } => {
            Some((GeometryShape::Box {
                x: *hx as f64 * 2.0,
                y: *hy as f64 * 2.0,
                z: *hz as f64 * 2.0,
            }, None))
        }
        super::model::GeomData::Sphere { radius } => {
            Some((GeometryShape::Sphere {
                radius: *radius as f64,
            }, None))
        }
        super::model::GeomData::Cylinder { radius, half_length } => {
            Some((GeometryShape::Cylinder {
                radius: *radius as f64,
                length: *half_length as f64 * 2.0,
            }, None))
        }
        super::model::GeomData::Capsule { radius, half_length } => {
            Some((GeometryShape::Capsule {
                radius: *radius as f64,
                length: *half_length as f64 * 2.0,
            }, None))
        }
        super::model::GeomData::Mesh { vertices, scale, .. } => {
            let s = scale.unwrap_or([1.0, 1.0, 1.0]);
            let n_verts = vertices.len() / 6;
            if n_verts < 3 {
                return None;
            }

            let mut points = Vec::with_capacity(n_verts);
            for i in 0..n_verts {
                let base = i * 6;
                points.push(na::Point3::new(
                    vertices[base] as f64 * s[0] as f64,
                    vertices[base + 1] as f64 * s[1] as f64,
                    vertices[base + 2] as f64 * s[2] as f64,
                ));
            }

            let mut indices = Vec::new();
            let mut face_normals = Vec::new();
            for i in (0..n_verts).step_by(3) {
                if i + 2 >= n_verts {
                    break;
                }
                indices.push([i as u32, (i + 1) as u32, (i + 2) as u32]);
                // Compute face normal from vertices.
                let v0 = &points[i];
                let v1 = &points[i + 1];
                let v2 = &points[i + 2];
                let e1 = v1 - v0;
                let e2 = v2 - v0;
                let n = e1.cross(&e2);
                let len = n.norm();
                if len > 1e-12 {
                    face_normals.push(n / len);
                } else {
                    face_normals.push(na::Vector3::z());
                }
            }
            if indices.is_empty() {
                return None;
            }

            let md = MeshData {
                vertices: points,
                indices,
                face_normals,
                vertex_normals: Vec::new(),
                texcoords: Vec::new(),
                materials: Vec::new(),
                submeshes: Vec::new(),
            };

            // Scale is already baked into vertices, so pass [1,1,1].
            Some((GeometryShape::Mesh {
                scale: na::Vector3::new(1.0, 1.0, 1.0),
                filename: String::new(),
            }, Some(md)))
        }
    }
}

// =========================================================================
//  Positional Jacobian / IK step via misarta
// =========================================================================

impl ModelAdapter {
    /// Compute the 3×chain_len positional Jacobian for a chain of articara
    /// joint indices, using misarta's world-frame Jacobian under the hood.
    ///
    /// When `root_link` is `Some`, a relative Jacobian (J_ee − J_base) is
    /// used so that joints on the "inverted" path get the correct sign
    /// automatically.  Otherwise the absolute Jacobian is computed.
    ///
    /// Returns an f32 matrix whose columns correspond 1-to-1 with `chain`.
    pub fn chain_positional_jacobian(
        &self,
        robot: &RobotModel,
        chain: &[usize],
        ee_link: &str,
        root_link: Option<&str>,
    ) -> na::DMatrix<f32> {
        let q = self.build_q(robot);
        let ee_mi = match self.link_name_to_misarta_joint(ee_link) {
            Some(v) if v > 0 => v,
            _ => return na::DMatrix::zeros(3, chain.len()),
        };

        let full_jac: na::DMatrix<f64> = if let Some(rl) = root_link {
            match self.link_name_to_misarta_joint(rl) {
                Some(base_mi) if base_mi > 0 => {
                    misarta::jacobian::compute_relative_jacobian(&self.model, &q, base_mi, ee_mi)
                }
                _ => {
                    // base_mi == 0 means root/universe joint — use absolute Jacobian
                    misarta::jacobian::compute_joint_jacobian(&self.model, &q, ee_mi)
                }
            }
        } else {
            misarta::jacobian::compute_joint_jacobian(&self.model, &q, ee_mi)
        };

        // Extract linear rows (3..6) and columns for chain joints.
        let mut jac = na::DMatrix::<f32>::zeros(3, chain.len());
        for (col, &ji) in chain.iter().enumerate() {
            if let Some(&Some(mi)) = self.articara_to_misarta.get(ji) {
                let vi = self.model.q_idx[mi]; // DOF index
                for row in 0..3 {
                    jac[(row, col)] = full_jac[(row + 3, vi)] as f32;
                }
            }
        }
        jac
    }

    /// Compute the 3×n_order positional Jacobian for a foot chain,
    /// remapped to a specific joint order (columns correspond to `joint_order`).
    ///
    /// This replaces the old `foot_jacobian` function.
    pub fn foot_positional_jacobian(
        &self,
        robot: &RobotModel,
        foot_link: &str,
        body_link: &str,
        joint_order: &[usize],
        idx_in_m: &[Option<usize>],
    ) -> na::DMatrix<f64> {
        let q = self.build_q(robot);
        let n = joint_order.len();

        let ee_mi = match self.link_name_to_misarta_joint(foot_link) {
            Some(v) if v > 0 => v,
            _ => return na::DMatrix::zeros(3, n),
        };
        let base_mi = match self.link_name_to_misarta_joint(body_link) {
            Some(v) => v,
            None => return na::DMatrix::zeros(3, n),
        };

        let full_jac = if base_mi > 0 {
            misarta::jacobian::compute_relative_jacobian(&self.model, &q, base_mi, ee_mi)
        } else {
            misarta::jacobian::compute_joint_jacobian(&self.model, &q, ee_mi)
        };

        let mut jac = na::DMatrix::<f64>::zeros(3, n);
        for &ji in joint_order {
            let col = match idx_in_m.get(ji).and_then(|&c| c) {
                Some(c) => c,
                None => continue,
            };
            if let Some(&Some(mi)) = self.articara_to_misarta.get(ji) {
                let vi = self.model.q_idx[mi];
                for row in 0..3 {
                    jac[(row, col)] = full_jac[(row + 3, vi)];
                }
            }
        }
        jac
    }

    /// Perform one Damped-Least-Squares IK step using misarta's Jacobian.
    ///
    /// Returns joint-angle deltas (one per element in `chain`) clamped by `max_step`.
    /// The caller should apply them via `robot.apply_joint_deltas(chain, &deltas)`.
    pub fn solve_ik_step(
        &self,
        robot: &RobotModel,
        chain: &[usize],
        ee_link: &str,
        root_link: Option<&str>,
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
        let dx_vec =
            na::DVector::from_column_slice(&[dx_clamped.x, dx_clamped.y, dx_clamped.z]);

        let jac = self.chain_positional_jacobian(robot, chain, ee_link, root_link);

        let jjt = &jac * jac.transpose();
        let lambda_sq = damping * damping;
        let identity = na::DMatrix::<f32>::identity(3, 3);
        let jjt_reg = jjt + identity * lambda_sq;

        let decomp = jjt_reg.lu();
        let y = decomp.solve(&dx_vec).unwrap_or(na::DVector::zeros(3));
        let dq = jac.transpose() * y;

        (0..n).map(|i| dq[i]).collect()
    }
}

// =========================================================================
//  Constraint IK via misarta
// =========================================================================

use misarta::constraint::{
    ConstrainedIkConfig, ConstrainedIkResult, ConstraintModel, ConstraintType,
    RigidConstraint,
};
use misarta::frames::Frame;

impl ModelAdapter {
    /// Build a misarta `Frame` for an articara link name.
    ///
    /// The frame is placed at the joint that owns the link, with identity
    /// local offset.  Returns `None` for unknown links.
    pub fn frame_for_link(&self, link_name: &str) -> Option<Frame<f64>> {
        let mi = self.link_name_to_misarta_joint(link_name)?;
        Some(Frame {
            name: link_name.to_string(),
            parent_joint: mi,
            placement: misarta::se3::identity(),
        })
    }

    /// Build a misarta `Frame` for an articara link with a local offset.
    pub fn frame_for_link_with_offset(
        &self,
        link_name: &str,
        offset: na::Isometry3<f32>,
    ) -> Option<Frame<f64>> {
        let mi = self.link_name_to_misarta_joint(link_name)?;
        Some(Frame {
            name: link_name.to_string(),
            parent_joint: mi,
            placement: offset.cast::<f64>(),
        })
    }

    /// Create a position-only (3D) constraint between two articara links.
    pub fn position_constraint(
        &self,
        link_a: &str,
        link_b: &str,
    ) -> Option<RigidConstraint<f64>> {
        let f1 = self.frame_for_link(link_a)?;
        let f2 = self.frame_for_link(link_b)?;
        Some(RigidConstraint::position(f1, f2))
    }

    /// Create a full-pose (6D) constraint between two articara links.
    pub fn pose_constraint(
        &self,
        link_a: &str,
        link_b: &str,
    ) -> Option<RigidConstraint<f64>> {
        let f1 = self.frame_for_link(link_a)?;
        let f2 = self.frame_for_link(link_b)?;
        Some(RigidConstraint::pose(f1, f2))
    }

    /// Solve constrained IK (loop-closure / cross-branch alignment).
    ///
    /// Returns the solved configuration mapped back to articara joint positions.
    pub fn solve_constrained_ik(
        &self,
        robot: &RobotModel,
        constraints: Vec<RigidConstraint<f64>>,
        config: &ConstrainedIkConfig,
    ) -> ConstrainedIkResult {
        let q0 = self.build_q(robot);
        let cm = ConstraintModel::from_constraints(constraints);
        misarta::constraint::solve_constrained_ik(&self.model, &q0, &cm, config)
    }

    /// Solve IK with a primary task (position) and rigid constraints.
    pub fn solve_task_with_constraints(
        &self,
        robot: &RobotModel,
        ee_link: &str,
        target: na::Vector3<f64>,
        constraints: Vec<RigidConstraint<f64>>,
        config: &ConstrainedIkConfig,
    ) -> ConstrainedIkResult {
        let q0 = self.build_q(robot);
        let cm = ConstraintModel::from_constraints(constraints);
        let joint_idx = match self.link_name_to_misarta_joint(ee_link) {
            Some(idx) => idx,
            None => {
                return ConstrainedIkResult {
                    q: q0,
                    iterations: 0,
                    constraint_error_norm: f64::INFINITY,
                    task_error_norm: f64::INFINITY,
                    converged: false,
                };
            }
        };
        misarta::constraint::solve_task_with_constraints(
            &self.model, &q0, joint_idx, target, &cm, config,
        )
    }

    /// Apply a solved misarta q-vector back to the articara RobotModel.
    pub fn apply_q_to_robot(&self, robot: &mut RobotModel, q: &[f64]) {
        for (ji, &maybe_mi) in self.articara_to_misarta.iter().enumerate() {
            if let Some(mi) = maybe_mi {
                let nq = self.model.joints[mi].joint_type.nq();
                if nq == 1 {
                    let qi = self.model.q_idx[mi];
                    robot.joint_positions[ji] = q[qi] as f32;
                }
            }
        }
    }

    /// Enforce mimic constraints on the current robot model.
    ///
    /// Converts q, applies `misarta::mimic::enforce_mimic`, writes back.
    pub fn enforce_mimic(&self, robot: &mut RobotModel) {
        if self.model.mimic.is_empty() {
            return;
        }
        let q = self.build_q(robot);
        let q_enforced = misarta::mimic::enforce_mimic(&self.model, &q);
        self.apply_q_to_robot(robot, &q_enforced);
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
