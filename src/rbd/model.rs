//! Robot body structure: data types, FK, tree navigation, inertia computation.
//!
//! This module contains the core data model for rigid body robots,
//! independent of any file format (URDF/SDF/MJCF) or UI framework.
//!
//! The kinematic tree, inertia, and dynamics are delegated to the embedded
//! `misarta::Model<f64>`.  GUI-specific data (visual/collision geometry,
//! joint limits, materials) lives alongside in the articara data structures.

use nalgebra as na;
use na::Matrix3;
use std::collections::HashMap;
use std::path::PathBuf;

use misarta::joint::JointType;
use misarta::model::{LinkInertia, Model, ModelBuilder};
use misarta::geometry::{GeometryModel, GeometryObject, GeometryShape};
use misarta::mesh::MeshData;

// ========== Data Structures ==========

/// Cached misarta model + index mappings.
#[derive(Clone, Debug)]
pub struct MisartaCache {
    pub model: Model<f64>,
    /// `a2m[articara_joint_idx]` → misarta joint index (1-based), or `None`.
    pub a2m: Vec<Option<usize>>,
    /// `m2a[misarta_joint_idx]` → articara joint index. Index 0 (universe) → `None`.
    pub m2a: Vec<Option<usize>>,
}

#[derive(Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct RobotModel {
    pub name: String,
    pub links: Vec<LinkData>,
    pub joints: Vec<JointData>,
    pub link_map: HashMap<String, usize>,
    pub joint_map: HashMap<String, usize>,
    pub root_link: String,
    pub children_joints: HashMap<String, Vec<usize>>,
    pub materials: HashMap<String, [f32; 4]>,
    pub joint_positions: Vec<f64>,
    /// Path of the originally loaded URDF file.
    #[cfg_attr(feature = "serde", serde(skip))]
    pub source_path: Option<PathBuf>,
    /// World-space transform of the URDF root link (identity by default).
    /// Used to re-root the display when fixing a non-root link as IK base.
    pub base_transform: na::Isometry3<f64>,
    /// Cached misarta model (kinematic tree + inertia).
    /// Built eagerly by constructors; `None` after serde deserialization
    /// until [`rebuild_misarta_model`] is called.
    #[cfg_attr(feature = "serde", serde(skip))]
    pub misarta_cache: Option<MisartaCache>,
}

#[derive(Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct LinkData {
    pub name: String,
    pub visuals: Vec<VisualData>,
    pub collisions: Vec<CollisionData>,
    pub inertial: InertialData,
}

#[derive(Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct VisualData {
    pub origin: na::Isometry3<f32>,
    pub geometry: GeomData,
    pub color: [f32; 4],
}

#[derive(Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct CollisionData {
    pub origin: na::Isometry3<f32>,
    pub geometry: GeomData,
}

#[derive(Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum GeomData {
    Box { hx: f32, hy: f32, hz: f32 },
    Cylinder { radius: f32, half_length: f32 },
    Sphere { radius: f32 },
    /// Capsule: cylinder body + hemispherical caps.
    /// `half_length` is the half-length of the **cylindrical** portion only (total = 2*half_length + 2*radius).
    Capsule { radius: f32, half_length: f32 },
    Mesh {
        vertices: Vec<f32>,          // flat [x, y, z, nx, ny, nz, ...]
        filename: Option<String>,    // original URI e.g. "package://..."
        scale: Option<[f32; 3]>,
    },
}

#[derive(Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct InertialData {
    pub origin: na::Isometry3<f32>,
    pub mass: f64,
    pub ixx: f64,
    pub ixy: f64,
    pub ixz: f64,
    pub iyy: f64,
    pub iyz: f64,
    pub izz: f64,
}

#[derive(Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct JointData {
    pub name: String,
    pub joint_type: String,
    pub parent_link: String,
    pub child_link: String,
    pub origin: na::Isometry3<f32>,
    pub axis: na::Vector3<f32>,
    pub lower: f64,
    pub upper: f64,
    pub effort: f64,
    pub velocity: f64,
}


// ========== Forward Kinematics & Tree Navigation ==========

impl RobotModel {
    pub fn compute_transforms(&self) -> HashMap<String, na::Isometry3<f32>> {
        let mc = self.mc_or_temp();
        let q = mc.build_q(self);
        let data = misarta::fk::forward_kinematics(&mc.model, &q);

        let base_f32 = isometry_f64_to_f32(&self.base_transform);
        let mut transforms: HashMap<String, na::Isometry3<f32>> = HashMap::new();
        transforms.insert(self.root_link.clone(), base_f32);
        for i in 1..mc.model.joints.len() {
            let link_name = &mc.model.link_names[i];
            let world_pose_f32 = base_f32 * isometry_f64_to_f32(&data.oMi[i]);
            transforms.insert(link_name.clone(), world_pose_f32);
        }
        transforms
    }


    /// Find the parent joint index for a given link name.
    /// Returns None for the root link.
    pub fn parent_joint_of_link(&self, link_name: &str) -> Option<usize> {
        self.joints
            .iter()
            .position(|j| j.child_link == link_name)
    }

    /// Return the list of ancestor link names from root down to (but not including)
    /// the given link.  Returns an empty vec for the root link.
    pub fn ancestor_links(&self, link_name: &str) -> Vec<String> {
        let mut ancestors = Vec::new();
        let mut current = link_name.to_string();
        loop {
            if let Some(ji) = self.parent_joint_of_link(&current) {
                let parent = self.joints[ji].parent_link.clone();
                ancestors.push(parent.clone());
                current = parent;
            } else {
                break;
            }
        }
        ancestors.reverse();
        ancestors
    }

    /// Compute a bounding sphere (center, radius) for a link's visual geometry
    /// in the link's local frame. Properly transforms all vertices by visual origin.
    pub fn link_bounding_sphere(&self, link_idx: usize) -> (na::Point3<f32>, f32) {
        let link = &self.links[link_idx];
        if link.visuals.is_empty() {
            return (na::Point3::origin(), 0.0);
        }

        let mut points: Vec<na::Point3<f32>> = Vec::new();
        for vis in &link.visuals {
            match &vis.geometry {
                GeomData::Box { hx, hy, hz } => {
                    for &sx in &[-1.0_f32, 1.0] {
                        for &sy in &[-1.0_f32, 1.0] {
                            for &sz in &[-1.0_f32, 1.0] {
                                points.push(vis.origin * na::Point3::new(*hx * sx, *hy * sy, *hz * sz));
                            }
                        }
                    }
                }
                GeomData::Cylinder { radius, half_length } => {
                    for i in 0..8 {
                        let a = (i as f32) * std::f32::consts::TAU / 8.0;
                        let (c, s) = (a.cos(), a.sin());
                        points.push(vis.origin * na::Point3::new(radius * c, radius * s, *half_length));
                        points.push(vis.origin * na::Point3::new(radius * c, radius * s, -*half_length));
                    }
                }
                GeomData::Sphere { radius } => {
                    let r = *radius;
                    for &d in &[
                        [r, 0.0, 0.0], [-r, 0.0, 0.0],
                        [0.0, r, 0.0], [0.0, -r, 0.0],
                        [0.0, 0.0, r], [0.0, 0.0, -r],
                    ] {
                        points.push(vis.origin * na::Point3::new(d[0], d[1], d[2]));
                    }
                }
                GeomData::Capsule { radius, half_length } => {
                    let total_h = *half_length + *radius;
                    for i in 0..8 {
                        let a = (i as f32) * std::f32::consts::TAU / 8.0;
                        let (c, s) = (a.cos(), a.sin());
                        points.push(vis.origin * na::Point3::new(radius * c, radius * s, total_h));
                        points.push(vis.origin * na::Point3::new(radius * c, radius * s, -total_h));
                    }
                }
                GeomData::Mesh { vertices, .. } => {
                    let step = if vertices.len() > 6000 { 6 * 10 } else { 6 };
                    for chunk in vertices.chunks(step) {
                        if chunk.len() >= 3 {
                            points.push(vis.origin * na::Point3::new(chunk[0], chunk[1], chunk[2]));
                        }
                    }
                }
            }
        }

        if points.is_empty() {
            return (na::Point3::origin(), 0.0);
        }

        let sum = points.iter().fold(na::Vector3::zeros(), |a, p| a + p.coords);
        let center = na::Point3::from(sum / points.len() as f32);

        let radius = points
            .iter()
            .map(|p| na::distance(&center, p))
            .fold(0.001_f32, f32::max);

        (center, radius * 1.05)
    }

    // =====================================================================
    //  Kinematic chain traversal
    // =====================================================================

    /// Return the list of movable joint indices from the URDF root to `end_link`.
    ///
    /// Joints appear in root→end-effector order.
    /// Only revolute, continuous, and prismatic joints are included.
    pub fn chain_joints(&self, end_link: &str) -> Vec<usize> {
        let mut chain = Vec::new();
        let mut current = end_link.to_string();
        while let Some(ji) = self.parent_joint_of_link(&current) {
            let jt = self.joints[ji].joint_type.as_str();
            if jt == "revolute" || jt == "continuous" || jt == "prismatic" {
                chain.push(ji);
            }
            current = self.joints[ji].parent_link.clone();
        }
        chain.reverse();
        chain
    }

    /// Return the list of movable joint indices between two arbitrary links.
    ///
    /// If `root_link` is `None`, equivalent to [`chain_joints`].
    /// Otherwise, finds the Lowest Common Ancestor (LCA) and returns all
    /// movable joints on both paths (root_link→LCA and LCA→end_link).
    pub fn chain_joints_between(
        &self,
        end_link: &str,
        root_link: Option<&str>,
    ) -> Vec<usize> {
        let root_link = match root_link {
            Some(r) => r,
            None => return self.chain_joints(end_link),
        };
        if root_link == end_link {
            return Vec::new();
        }

        // Ancestors of both links
        let anc_root = self.ancestors_with_self(root_link);
        let anc_end = self.ancestors_with_self(end_link);

        let anc_root_set: std::collections::HashSet<&str> =
            anc_root.iter().map(|s| s.as_str()).collect();

        let lca = anc_end
            .iter()
            .find(|a| anc_root_set.contains(a.as_str()))
            .cloned()
            .unwrap_or_else(|| self.root_link.clone());

        // Collect joints root_link → LCA
        let up = self.collect_path_joints(root_link, &lca);
        // Collect joints end_link → LCA, then reverse
        let mut down = self.collect_path_joints(end_link, &lca);
        down.reverse();

        let mut result = up;
        result.extend(down);
        result
    }

    /// Helper: all ancestor link names including `link` itself, from self → root.
    fn ancestors_with_self(&self, link: &str) -> Vec<String> {
        let mut list = vec![link.to_string()];
        let mut current = link.to_string();
        while let Some(ji) = self.parent_joint_of_link(&current) {
            current = self.joints[ji].parent_link.clone();
            list.push(current.clone());
        }
        list
    }

    /// Helper: collect movable joint indices from `from_link` up to `to_ancestor`.
    fn collect_path_joints(&self, from_link: &str, to_ancestor: &str) -> Vec<usize> {
        let mut joints = Vec::new();
        let mut current = from_link.to_string();
        while current != to_ancestor {
            if let Some(ji) = self.parent_joint_of_link(&current) {
                let jt = self.joints[ji].joint_type.as_str();
                if jt == "revolute" || jt == "continuous" || jt == "prismatic" {
                    joints.push(ji);
                }
                current = self.joints[ji].parent_link.clone();
            } else {
                break;
            }
        }
        joints
    }

    /// World position of a link's bounding-sphere center.
    pub fn ee_world_pos(
        &self,
        link_idx: usize,
        transforms: &HashMap<String, na::Isometry3<f32>>,
    ) -> na::Point3<f32> {
        let link_name = &self.links[link_idx].name;
        let world_tf = transforms
            .get(link_name)
            .copied()
            .unwrap_or(na::Isometry3::identity());
        let (local_center, _) = self.link_bounding_sphere(link_idx);
        world_tf * local_center
    }

    /// Apply joint-angle deltas (one per joint index), clamping to limits.
    pub fn apply_joint_deltas(&mut self, joint_indices: &[usize], deltas: &[f64]) {
        for (&ji, &d) in joint_indices.iter().zip(deltas.iter()) {
            let lower = self.joints[ji].lower;
            let upper = self.joints[ji].upper;
            self.joint_positions[ji] = (self.joint_positions[ji] + d).clamp(lower, upper);
        }
    }
}

// =========================================================================
//  Inertia tensor computation (uniform-density)
// =========================================================================

/// Inertia tensor components (about the geometry's own centroid).
#[derive(Debug, Clone, Copy)]
pub struct InertiaTensor {
    pub ixx: f64,
    pub ixy: f64,
    pub ixz: f64,
    pub iyy: f64,
    pub iyz: f64,
    pub izz: f64,
}

/// Compute the inertia tensor for a single geometry primitive at uniform
/// density, given the total `mass` of that piece.
///
/// All results are about the geometry's own centroid (center of mass at origin).
pub fn compute_geometry_inertia(geom: &GeomData, mass: f64) -> InertiaTensor {
    match geom {
        GeomData::Box { hx, hy, hz } => {
            // Full dimensions: a=2*hx, b=2*hy, c=2*hz
            let a2 = (2.0 * *hx as f64).powi(2);
            let b2 = (2.0 * *hy as f64).powi(2);
            let c2 = (2.0 * *hz as f64).powi(2);
            InertiaTensor {
                ixx: mass / 12.0 * (b2 + c2),
                ixy: 0.0,
                ixz: 0.0,
                iyy: mass / 12.0 * (a2 + c2),
                iyz: 0.0,
                izz: mass / 12.0 * (a2 + b2),
            }
        }
        GeomData::Cylinder { radius, half_length } => {
            // Solid cylinder, axis along Z.
            let r2 = (*radius as f64).powi(2);
            let h2 = (2.0 * *half_length as f64).powi(2);
            InertiaTensor {
                ixx: mass / 12.0 * (3.0 * r2 + h2),
                ixy: 0.0,
                ixz: 0.0,
                iyy: mass / 12.0 * (3.0 * r2 + h2),
                iyz: 0.0,
                izz: mass / 2.0 * r2,
            }
        }
        GeomData::Sphere { radius } => {
            // Solid sphere.
            let r2 = (*radius as f64).powi(2);
            let i = 2.0 / 5.0 * mass * r2;
            InertiaTensor {
                ixx: i,
                ixy: 0.0,
                ixz: 0.0,
                iyy: i,
                iyz: 0.0,
                izz: i,
            }
        }
        GeomData::Capsule { radius, half_length } => {
            // Solid capsule = cylinder + 2 hemispheres (= 1 full sphere).
            // Reference: https://www.gamedev.net/resources/_/technical/math-and-physics/capsule-inertia-tensor-r3149
            let r = *radius as f64;
            let h = 2.0 * *half_length as f64; // cylinder height
            let r2 = r * r;
            let h2 = h * h;
            let vol_cyl = std::f64::consts::PI * r2 * h;
            let vol_sph = 4.0 / 3.0 * std::f64::consts::PI * r2 * r;
            let total_vol = vol_cyl + vol_sph;
            let m_cyl = mass * vol_cyl / total_vol;
            let m_sph = mass * vol_sph / total_vol;
            // Cylinder about Z axis
            let i_cyl_xx = m_cyl / 12.0 * (3.0 * r2 + h2);
            let i_cyl_zz = m_cyl / 2.0 * r2;
            // Sphere about its own center
            let i_sph_own = 2.0 / 5.0 * m_sph * r2;
            // Parallel axis: each hemisphere center is at ±(h/2 + 3r/8) from capsule center
            let d = h / 2.0 + 3.0 * r / 8.0;
            let i_sph_xx = i_sph_own + m_sph * d * d;
            let i_sph_zz = i_sph_own;
            InertiaTensor {
                ixx: i_cyl_xx + i_sph_xx,
                ixy: 0.0,
                ixz: 0.0,
                iyy: i_cyl_xx + i_sph_xx,
                iyz: 0.0,
                izz: i_cyl_zz + i_sph_zz,
            }
        }
        GeomData::Mesh { vertices, .. } => {
            compute_mesh_inertia(vertices, mass)
        }
    }
}

/// Compute inertia tensor for a triangle mesh using the method described in
/// "Efficient Feature Extraction for 2D/3D Objects in Mesh Representation"
/// (Mirtich 1996) — canonicalised for a closed surface.  The mesh is treated
/// as a solid with uniform density.
///
/// `vertices` is the flat buffer `[x, y, z, nx, ny, nz, ...]` (stride 6).
fn compute_mesh_inertia(vertices: &[f32], mass: f64) -> InertiaTensor {
    let num_verts = vertices.len() / 6;
    let num_tris = num_verts / 3;
    if num_tris == 0 {
        return InertiaTensor {
            ixx: 0.001, ixy: 0.0, ixz: 0.0,
            iyy: 0.001, iyz: 0.0, izz: 0.001,
        };
    }

    // Accumulate volume and moments using the divergence theorem.
    let mut vol = 0.0f64;
    // Second moments of volume about origin
    let mut vxx = 0.0f64;
    let mut vyy = 0.0f64;
    let mut vzz = 0.0f64;
    let mut vxy = 0.0f64;
    let mut vxz = 0.0f64;
    let mut vyz = 0.0f64;

    for ti in 0..num_tris {
        let i0 = ti * 3;
        let p = |vi: usize| -> (f64, f64, f64) {
            let base = (i0 + vi) * 6;
            (
                vertices[base] as f64,
                vertices[base + 1] as f64,
                vertices[base + 2] as f64,
            )
        };
        let (x0, y0, z0) = p(0);
        let (x1, y1, z1) = p(1);
        let (x2, y2, z2) = p(2);

        // Cross product of edges
        let ex1 = x1 - x0;
        let ey1 = y1 - y0;
        let ez1 = z1 - z0;
        let ex2 = x2 - x0;
        let ey2 = y2 - y0;
        let ez2 = z2 - z0;
        let nx = ey1 * ez2 - ez1 * ey2;
        let ny = ez1 * ex2 - ex1 * ez2;
        let nz = ex1 * ey2 - ey1 * ex2;

        // Signed volume of tetrahedron from origin to this triangle face
        let det = x0 * nx + y0 * ny + z0 * nz; // 6 * signed volume
        vol += det;

        // For the tetrahedron (origin, v0, v1, v2) the volume integrals of
        // x², y², z², xy, xz, yz can be expressed analytically.
        // Using the factored forms for uniform-density tetrahedra:
        let xs = x0 + x1 + x2;
        let ys = y0 + y1 + y2;
        let zs = z0 + z1 + z2;

        vxx += det * (x0 * x0 + x1 * x1 + x2 * x2 + xs * xs);
        vyy += det * (y0 * y0 + y1 * y1 + y2 * y2 + ys * ys);
        vzz += det * (z0 * z0 + z1 * z1 + z2 * z2 + zs * zs);
        vxy += det * (x0 * (2.0 * y0 + y1 + y2)
                     + x1 * (y0 + 2.0 * y1 + y2)
                     + x2 * (y0 + y1 + 2.0 * y2));
        vxz += det * (x0 * (2.0 * z0 + z1 + z2)
                     + x1 * (z0 + 2.0 * z1 + z2)
                     + x2 * (z0 + z1 + 2.0 * z2));
        vyz += det * (y0 * (2.0 * z0 + z1 + z2)
                     + y1 * (z0 + 2.0 * z1 + z2)
                     + y2 * (z0 + z1 + 2.0 * z2));
    }

    // vol is 6× the signed volume
    let volume = vol / 6.0;
    if volume.abs() < 1e-20 {
        return InertiaTensor {
            ixx: 0.001, ixy: 0.0, ixz: 0.0,
            iyy: 0.001, iyz: 0.0, izz: 0.001,
        };
    }

    // Normalise volume integrals
    let density = mass / volume.abs();
    let xx = density * vxx / 120.0;
    let yy = density * vyy / 120.0;
    let zz = density * vzz / 120.0;
    let xy = density * vxy / 120.0;
    let xz = density * vxz / 120.0;
    let yz = density * vyz / 120.0;

    InertiaTensor {
        ixx: yy + zz,
        ixy: -xy,
        ixz: -xz,
        iyy: xx + zz,
        iyz: -yz,
        izz: xx + yy,
    }
}

/// Compute the volume of a geometry primitive.
pub fn compute_geometry_volume(geom: &GeomData) -> f64 {
    match geom {
        GeomData::Box { hx, hy, hz } => {
            (2.0 * *hx as f64) * (2.0 * *hy as f64) * (2.0 * *hz as f64)
        }
        GeomData::Cylinder { radius, half_length } => {
            std::f64::consts::PI * (*radius as f64).powi(2) * (2.0 * *half_length as f64)
        }
        GeomData::Sphere { radius } => {
            4.0 / 3.0 * std::f64::consts::PI * (*radius as f64).powi(3)
        }
        GeomData::Capsule { radius, half_length } => {
            let r = *radius as f64;
            let h = 2.0 * *half_length as f64;
            // cylinder + sphere
            std::f64::consts::PI * r * r * h + 4.0 / 3.0 * std::f64::consts::PI * r * r * r
        }
        GeomData::Mesh { vertices, .. } => {
            compute_mesh_volume(vertices)
        }
    }
}

/// Compute the signed volume of a triangle mesh using the divergence theorem.
fn compute_mesh_volume(vertices: &[f32]) -> f64 {
    let num_verts = vertices.len() / 6;
    let num_tris = num_verts / 3;
    let mut vol = 0.0f64;
    for ti in 0..num_tris {
        let i0 = ti * 3;
        let p = |vi: usize| -> (f64, f64, f64) {
            let base = (i0 + vi) * 6;
            (
                vertices[base] as f64,
                vertices[base + 1] as f64,
                vertices[base + 2] as f64,
            )
        };
        let (x0, y0, z0) = p(0);
        let (x1, y1, z1) = p(1);
        let (x2, y2, z2) = p(2);
        let nx = (y1 - y0) * (z2 - z0) - (z1 - z0) * (y2 - y0);
        let ny = (z1 - z0) * (x2 - x0) - (x1 - x0) * (z2 - z0);
        let nz = (x1 - x0) * (y2 - y0) - (y1 - y0) * (x2 - x0);
        vol += x0 * nx + y0 * ny + z0 * nz;
    }
    (vol / 6.0).abs()
}

/// Compute combined inertia for all visuals of a link, distributing the
/// total `mass` proportionally by volume to each visual, then using the
/// parallel axis theorem to combine them about the link centroid.
///
/// Returns `(InertialData)` with the computed inertia and the combined
/// center of mass as `origin`.
pub fn compute_link_inertia(visuals: &[VisualData], total_mass: f64) -> InertialData {
    if visuals.is_empty() || total_mass <= 0.0 {
        return InertialData {
            origin: na::Isometry3::identity(),
            mass: total_mass,
            ixx: 0.001, ixy: 0.0, ixz: 0.0,
            iyy: 0.001, iyz: 0.0, izz: 0.001,
        };
    }

    // 1) Compute volume and centroid (in visual-local frame) for each visual.
    let vols: Vec<f64> = visuals.iter().map(|v| compute_geometry_volume(&v.geometry)).collect();
    let total_vol: f64 = vols.iter().sum();
    if total_vol < 1e-20 {
        return InertialData {
            origin: na::Isometry3::identity(),
            mass: total_mass,
            ixx: 0.001, ixy: 0.0, ixz: 0.0,
            iyy: 0.001, iyz: 0.0, izz: 0.001,
        };
    }

    // Mass of each visual (proportional to volume)
    let masses: Vec<f64> = vols.iter().map(|v| total_mass * v / total_vol).collect();

    // 2) Compute combined center of mass (in link frame).
    let mut com = na::Vector3::<f64>::zeros();
    for (i, vis) in visuals.iter().enumerate() {
        let vis_center = vis.origin.translation.vector.cast::<f64>();
        com += masses[i] * vis_center;
    }
    com /= total_mass;

    // 3) For each visual, compute local inertia then apply parallel axis theorem
    //    to shift to the combined CoM.
    let mut ixx_total = 0.0f64;
    let mut ixy_total = 0.0f64;
    let mut ixz_total = 0.0f64;
    let mut iyy_total = 0.0f64;
    let mut iyz_total = 0.0f64;
    let mut izz_total = 0.0f64;

    for (i, vis) in visuals.iter().enumerate() {
        let mi = masses[i];
        if mi < 1e-20 {
            continue;
        }

        // Inertia about the visual's own centroid (in visual-local axes)
        let local_inertia = compute_geometry_inertia(&vis.geometry, mi);

        // Rotate the inertia tensor from visual-local frame to link frame.
        // I_link = R * I_local * R^T
        let rot = vis.origin.rotation.to_rotation_matrix();
        let r = rot.matrix().cast::<f64>();
        let i_local = na::Matrix3::new(
            local_inertia.ixx, local_inertia.ixy, local_inertia.ixz,
            local_inertia.ixy, local_inertia.iyy, local_inertia.iyz,
            local_inertia.ixz, local_inertia.iyz, local_inertia.izz,
        );
        let i_rotated = &r * &i_local * r.transpose();

        // Parallel axis theorem: shift from visual centroid to combined CoM
        let d = vis.origin.translation.vector.cast::<f64>() - com;
        let dx = d.x;
        let dy = d.y;
        let dz = d.z;
        let d2 = dx * dx + dy * dy + dz * dz;

        // I_shifted = I_rotated + m * (d²·E - d⊗d)
        ixx_total += i_rotated[(0, 0)] + mi * (d2 - dx * dx);
        ixy_total += i_rotated[(0, 1)] + mi * (    - dx * dy);
        ixz_total += i_rotated[(0, 2)] + mi * (    - dx * dz);
        iyy_total += i_rotated[(1, 1)] + mi * (d2 - dy * dy);
        iyz_total += i_rotated[(1, 2)] + mi * (    - dy * dz);
        izz_total += i_rotated[(2, 2)] + mi * (d2 - dz * dz);
    }

    InertialData {
        origin: na::Isometry3::from_parts(
            na::Translation3::new(com.x as f32, com.y as f32, com.z as f32),
            na::UnitQuaternion::identity(),
        ),
        mass: total_mass,
        ixx: ixx_total,
        ixy: ixy_total,
        ixz: ixz_total,
        iyy: iyy_total,
        iyz: iyz_total,
        izz: izz_total,
    }
}


// ========== Inertia Validation ==========

/// Severity of a validation issue.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ValidationSeverity {
    Error,
    Warning,
}

/// A single validation diagnostic.
#[derive(Debug, Clone)]
pub struct ValidationIssue {
    pub severity: ValidationSeverity,
    pub message: String,
}

/// Result of validating one link's inertial data.
#[derive(Debug, Clone)]
pub struct InertiaValidation {
    pub link_name: String,
    pub issues: Vec<ValidationIssue>,
}

impl InertiaValidation {
    pub fn is_ok(&self) -> bool {
        self.issues.is_empty()
    }

    pub fn has_errors(&self) -> bool {
        self.issues.iter().any(|i| i.severity == ValidationSeverity::Error)
    }

    pub fn has_warnings(&self) -> bool {
        self.issues.iter().any(|i| i.severity == ValidationSeverity::Warning)
    }
}

/// Validate mass and inertia tensor of a single link.
///
/// Checks performed:
/// 1. Mass must be positive (> 0).
/// 2. Diagonal elements (Ixx, Iyy, Izz) must be non-negative.
/// 3. Triangle inequality on principal moments (Ixx + Iyy >= Izz, etc.).
/// 4. Inertia tensor matrix must be positive semi-definite (all eigenvalues >= 0).
/// 5. Each diagonal element should not exceed mass × reasonable_radius² heuristic
///    (warning only — assumes max equivalent radius of 10 m).
pub fn validate_inertia(link: &LinkData) -> InertiaValidation {
    let mut issues = Vec::new();
    let inertial = &link.inertial;

    // --- 1. Mass check ---
    if inertial.mass < 0.0 {
        issues.push(ValidationIssue {
            severity: ValidationSeverity::Error,
            message: "Mass is negative".into(),
        });
    } else if inertial.mass == 0.0 {
        // mass=0 is sometimes intentional (dummy link), just warn
        issues.push(ValidationIssue {
            severity: ValidationSeverity::Warning,
            message: "Mass is zero".into(),
        });
    }

    // --- 2. Diagonal elements non-negative ---
    if inertial.ixx < 0.0 {
        issues.push(ValidationIssue {
            severity: ValidationSeverity::Error,
            message: format!("Ixx is negative ({:.6e})", inertial.ixx),
        });
    }
    if inertial.iyy < 0.0 {
        issues.push(ValidationIssue {
            severity: ValidationSeverity::Error,
            message: format!("Iyy is negative ({:.6e})", inertial.iyy),
        });
    }
    if inertial.izz < 0.0 {
        issues.push(ValidationIssue {
            severity: ValidationSeverity::Error,
            message: format!("Izz is negative ({:.6e})", inertial.izz),
        });
    }

    // --- 3. Triangle inequality on diagonal elements ---
    // For any physically realizable rigid body: Ia + Ib >= Ic
    let (ixx, iyy, izz) = (inertial.ixx, inertial.iyy, inertial.izz);
    if ixx >= 0.0 && iyy >= 0.0 && izz >= 0.0 {
        let eps = 1e-12;
        if ixx + iyy + eps < izz {
            issues.push(ValidationIssue {
                severity: ValidationSeverity::Error,
                message: format!("Triangle inequality violated: Ixx + Iyy < Izz ({:.6e} + {:.6e} < {:.6e})", ixx, iyy, izz),
            });
        }
        if iyy + izz + eps < ixx {
            issues.push(ValidationIssue {
                severity: ValidationSeverity::Error,
                message: format!("Triangle inequality violated: Iyy + Izz < Ixx ({:.6e} + {:.6e} < {:.6e})", iyy, izz, ixx),
            });
        }
        if ixx + izz + eps < iyy {
            issues.push(ValidationIssue {
                severity: ValidationSeverity::Error,
                message: format!("Triangle inequality violated: Ixx + Izz < Iyy ({:.6e} + {:.6e} < {:.6e})", ixx, izz, iyy),
            });
        }
    }

    // --- 4. Positive semi-definite (eigenvalue check) ---
    let mat = na::Matrix3::new(
        inertial.ixx, inertial.ixy, inertial.ixz,
        inertial.ixy, inertial.iyy, inertial.iyz,
        inertial.ixz, inertial.iyz, inertial.izz,
    );
    let eigen = na::SymmetricEigen::new(mat);
    let min_eigenvalue = eigen.eigenvalues.min();
    if min_eigenvalue < -1e-10 {
        issues.push(ValidationIssue {
            severity: ValidationSeverity::Error,
            message: format!(
                "Inertia tensor is not positive semi-definite (min eigenvalue = {:.6e})",
                min_eigenvalue
            ),
        });
    }

    // --- 5. Sanity: diagonal vs mass (heuristic) ---
    // For a solid sphere of radius R: I = 2/5 * m * R²
    // We warn if any diagonal element exceeds m * (10 m)² = 100 * m
    // which would imply an equivalent radius > 10 m — unusual for most robots.
    if inertial.mass > 0.0 {
        let max_reasonable = inertial.mass * 100.0; // m * R² with R=10m
        for (name, val) in [("Ixx", ixx), ("Iyy", iyy), ("Izz", izz)] {
            if val > max_reasonable {
                issues.push(ValidationIssue {
                    severity: ValidationSeverity::Warning,
                    message: format!(
                        "{name} ({:.4e}) seems very large for mass {:.4} kg (equivalent radius > 10 m)",
                        val, inertial.mass
                    ),
                });
            }
        }

        // Also warn if diagonal is extremely small relative to mass
        // (point-mass like). Threshold: I < mass * 1e-10
        let min_reasonable = inertial.mass * 1e-10;
        let all_tiny = ixx < min_reasonable && iyy < min_reasonable && izz < min_reasonable;
        if all_tiny && inertial.mass > 1e-6 {
            issues.push(ValidationIssue {
                severity: ValidationSeverity::Warning,
                message: format!(
                    "All diagonal elements are near zero for mass {:.4} kg (point-mass-like)",
                    inertial.mass
                ),
            });
        }
    }

    InertiaValidation {
        link_name: link.name.clone(),
        issues,
    }
}

/// Validate inertia for all links in a model.
pub fn validate_all_inertia(model: &RobotModel) -> Vec<InertiaValidation> {
    model.links.iter().map(|link| validate_inertia(link)).collect()
}

// =========================================================================
//  Misarta integration: model building, FK, Jacobians, IK
// =========================================================================

impl MisartaCache {
    /// Build the cache from a `RobotModel`.
    pub fn build(robot: &RobotModel) -> Self {
        let mut builder = ModelBuilder::<f64>::new()
            .name(robot.name.clone())
            .root_link_name(robot.root_link.clone());

        builder = builder.gravity(na::Vector3::new(0.0, 0.0, -9.81));

        let mut a2m: Vec<Option<usize>> = vec![None; robot.joints.len()];
        let mut m2a: Vec<Option<usize>> = vec![None]; // index 0 = universe

        let mut link_to_misarta_idx: HashMap<String, usize> = HashMap::new();
        link_to_misarta_idx.insert(robot.root_link.clone(), 0);

        // BFS from root
        let mut queue = vec![robot.root_link.clone()];
        while let Some(link_name) = queue.pop() {
            let parent_misarta_idx = link_to_misarta_idx[&link_name];
            if let Some(child_joint_indices) = robot.children_joints.get(&link_name) {
                for &ji in child_joint_indices {
                    let joint = &robot.joints[ji];
                    let joint_type = convert_joint_type(joint);
                    let placement = joint.origin.cast::<f64>();

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

                    let misarta_idx = m2a.len();
                    a2m[ji] = Some(misarta_idx);
                    m2a.push(Some(ji));
                    link_to_misarta_idx.insert(child_link_name.clone(), misarta_idx);
                    queue.push(child_link_name.clone());
                }
            }
        }

        let mut model = builder.build();
        if let Some(&root_li) = robot.link_map.get(&robot.root_link) {
            model.inertias[0] = convert_link_inertia(&robot.links[root_li]);
        }

        Self { model, a2m, m2a }
    }

    /// Build the full q-vector from `RobotModel.joint_positions`.
    pub fn build_q(&self, robot: &RobotModel) -> Vec<f64> {
        let mut q = self.model.neutral_q();
        for (ji, &maybe_mi) in self.a2m.iter().enumerate() {
            if let Some(mi) = maybe_mi {
                let nq = self.model.joints[mi].joint_type.nq();
                if nq == 1 {
                    let qi = self.model.q_idx[mi];
                    q[qi] = robot.joint_positions[ji];
                }
            }
        }
        q
    }

    /// Build the full velocity vector from a sparse map (keyed by articara joint index).
    pub fn build_v(&self, velocities: &HashMap<usize, f64>) -> na::DVector<f64> {
        let mut v = na::DVector::zeros(self.model.nv);
        for (&ji, &qd) in velocities {
            if let Some(mi) = self.a2m.get(ji).and_then(|&m| m) {
                let nv = self.model.joints[mi].joint_type.nv();
                if nv == 1 {
                    let vi = self.model.v_idx[mi];
                    v[vi] = qd;
                }
            }
        }
        v
    }

    /// Map a result vector indexed by misarta v-indices to a vector indexed by `joint_order`.
    pub fn extract_subvector(
        &self,
        full: &na::DVector<f64>,
        joint_order: &[usize],
    ) -> (na::DVector<f64>, Vec<Option<usize>>) {
        let n = joint_order.len();
        let mut sub = na::DVector::zeros(n);
        let mut idx_in_result: Vec<Option<usize>> = vec![None; self.a2m.len()];
        for (col, &ji) in joint_order.iter().enumerate() {
            if let Some(mi) = self.a2m.get(ji).and_then(|&m| m) {
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

    /// Extract a sub-matrix from a full nv×nv matrix for joints in `joint_order`.
    pub fn extract_submatrix(
        &self,
        full: &na::DMatrix<f64>,
        joint_order: &[usize],
    ) -> (na::DMatrix<f64>, Vec<Option<usize>>) {
        let n = joint_order.len();
        let mut sub = na::DMatrix::zeros(n, n);
        let mut idx_in_m: Vec<Option<usize>> = vec![None; self.a2m.len()];

        let mut v_indices: Vec<Option<usize>> = Vec::with_capacity(n);
        for (col, &ji) in joint_order.iter().enumerate() {
            idx_in_m[ji] = Some(col);
            if let Some(mi) = self.a2m.get(ji).and_then(|&m| m) {
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

    /// Look up the misarta joint index for a given articara link name.
    pub fn link_name_to_misarta_joint(&self, link_name: &str) -> Option<usize> {
        self.model.link_names.iter().position(|n| n == link_name)
    }

    /// Apply a solved misarta q-vector back to the `RobotModel`.
    pub fn apply_q_to_robot(&self, robot: &mut RobotModel, q: &[f64]) {
        for (ji, &maybe_mi) in self.a2m.iter().enumerate() {
            if let Some(mi) = maybe_mi {
                let nq = self.model.joints[mi].joint_type.nq();
                if nq == 1 {
                    let qi = self.model.q_idx[mi];
                    robot.joint_positions[ji] = q[qi];
                }
            }
        }
    }
}

impl RobotModel {
    /// Rebuild the cached misarta model from current links/joints.
    ///
    /// Called automatically by constructors. Must be called explicitly
    /// after structural changes (add/remove joints) or serde deserialization.
    pub fn rebuild_misarta_model(&mut self) {
        self.misarta_cache = Some(MisartaCache::build(self));
    }

    /// Get the cached misarta model, or build a temporary one.
    fn mc_or_temp(&self) -> std::borrow::Cow<'_, MisartaCache> {
        match &self.misarta_cache {
            Some(mc) => std::borrow::Cow::Borrowed(mc),
            None => std::borrow::Cow::Owned(MisartaCache::build(self)),
        }
    }

    /// Get the cached misarta model. Panics if not built.
    pub fn mc(&self) -> &MisartaCache {
        self.misarta_cache.as_ref().expect("misarta model not built; call rebuild_misarta_model()")
    }

    /// Build the q-vector from current joint positions.
    pub fn build_q(&self) -> Vec<f64> {
        self.mc().build_q(self)
    }

    /// Build a velocity vector from a sparse map.
    pub fn build_v(&self, velocities: &HashMap<usize, f64>) -> na::DVector<f64> {
        self.mc().build_v(velocities)
    }

    /// Compute 3×chain_len positional Jacobian for a chain of joint indices.
    ///
    /// When `root_link` is `Some`, uses a relative Jacobian.
    /// Returns an f64 matrix expressed in the **world frame**.
    pub fn chain_positional_jacobian(
        &self,
        chain: &[usize],
        ee_link: &str,
        root_link: Option<&str>,
    ) -> na::DMatrix<f64> {
        let mc = self.mc();
        let q = mc.build_q(self);
        let ee_mi = mc.link_name_to_misarta_joint(ee_link).unwrap_or(0);

        let full_jac: na::DMatrix<f64> = if let Some(rl) = root_link {
            let base_mi = mc.link_name_to_misarta_joint(rl).unwrap_or(0);
            if ee_mi > 0 && base_mi > 0 {
                // Both non-root: standard relative Jacobian
                misarta::jacobian::compute_relative_jacobian(&mc.model, &q, base_mi, ee_mi)
            } else if ee_mi == 0 && base_mi > 0 {
                // EE is URDF root: J_rel = J(root=0) - J(base) = -J(base)
                -misarta::jacobian::compute_joint_jacobian(&mc.model, &q, base_mi)
            } else if ee_mi > 0 {
                // base is root: standard absolute Jacobian
                misarta::jacobian::compute_joint_jacobian(&mc.model, &q, ee_mi)
            } else {
                // Both are root: zero
                return na::DMatrix::zeros(3, chain.len());
            }
        } else if ee_mi > 0 {
            misarta::jacobian::compute_joint_jacobian(&mc.model, &q, ee_mi)
        } else {
            return na::DMatrix::zeros(3, chain.len());
        };

        // misarta Jacobian is in URDF-root frame; rotate to world frame
        let r = self.base_transform.rotation.to_rotation_matrix();

        let mut jac = na::DMatrix::<f64>::zeros(3, chain.len());
        for (col, &ji) in chain.iter().enumerate() {
            if let Some(&Some(mi)) = mc.a2m.get(ji) {
                let vi = mc.model.q_idx[mi];
                let v = na::Vector3::new(
                    full_jac[(3, vi)],
                    full_jac[(4, vi)],
                    full_jac[(5, vi)],
                );
                let v_world = r * v;
                jac[(0, col)] = v_world[0];
                jac[(1, col)] = v_world[1];
                jac[(2, col)] = v_world[2];
            }
        }
        jac
    }

    /// Compute 3×n_order positional Jacobian for a foot, remapped to `joint_order`.
    pub fn foot_positional_jacobian(
        &self,
        foot_link: &str,
        body_link: &str,
        joint_order: &[usize],
        idx_in_m: &[Option<usize>],
    ) -> na::DMatrix<f64> {
        let mc = self.mc();
        let q = mc.build_q(self);
        let n = joint_order.len();

        let ee_mi = match mc.link_name_to_misarta_joint(foot_link) {
            Some(v) if v > 0 => v,
            _ => return na::DMatrix::zeros(3, n),
        };
        let base_mi = match mc.link_name_to_misarta_joint(body_link) {
            Some(v) => v,
            None => return na::DMatrix::zeros(3, n),
        };

        let full_jac = if base_mi > 0 {
            misarta::jacobian::compute_relative_jacobian(&mc.model, &q, base_mi, ee_mi)
        } else {
            misarta::jacobian::compute_joint_jacobian(&mc.model, &q, ee_mi)
        };

        // misarta Jacobian is in URDF-root frame; rotate to world frame
        let r = self.base_transform.rotation.to_rotation_matrix();

        let mut jac = na::DMatrix::<f64>::zeros(3, n);
        for &ji in joint_order {
            let col = match idx_in_m.get(ji).and_then(|&c| c) {
                Some(c) => c,
                None => continue,
            };
            if let Some(&Some(mi)) = mc.a2m.get(ji) {
                let vi = mc.model.q_idx[mi];
                let v = na::Vector3::new(
                    full_jac[(3, vi)],
                    full_jac[(4, vi)],
                    full_jac[(5, vi)],
                );
                let v_world = r * v;
                jac[(0, col)] = v_world[0];
                jac[(1, col)] = v_world[1];
                jac[(2, col)] = v_world[2];
            }
        }
        jac
    }

    /// Perform one Damped-Least-Squares IK step with optional null-space
    /// posture stabilization.
    ///
    /// When `ref_positions` is `Some`, joints are pulled toward the reference
    /// posture in the Jacobian null space so that redundant DOFs stay stable.
    ///
    /// Returns joint-angle deltas (one per element in `chain`).
    pub fn solve_ik_step(
        &self,
        chain: &[usize],
        ee_link: &str,
        root_link: Option<&str>,
        ee_pos: &na::Point3<f64>,
        target_pos: &na::Point3<f64>,
        damping: f64,
        max_step: f64,
        ref_positions: Option<&[f64]>,
    ) -> Vec<f64> {
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

        let jac = self.chain_positional_jacobian(chain, ee_link, root_link);

        // Damped pseudo-inverse: J⁺ = J^T (J J^T + λ²I)⁻¹
        let jjt = &jac * jac.transpose();
        let lambda_sq = damping * damping;
        let identity3 = na::DMatrix::<f64>::identity(3, 3);
        let jjt_reg = jjt + &identity3 * lambda_sq;

        let decomp = jjt_reg.lu();
        let y = decomp.solve(&dx_vec).unwrap_or(na::DVector::zeros(3));
        let dq_primary = jac.transpose() * &y;

        // Null-space posture stabilization:
        //   Δq = J⁺ Δx  +  (I − J⁺ J) · k_ns · (q_ref − q_cur)
        let dq = if let Some(ref_pos) = ref_positions {
            // Build J⁺ explicitly (n×3)
            let j_pinv = {
                let mut jp = na::DMatrix::<f64>::zeros(n, 3);
                for col in 0..3 {
                    let e = na::DVector::from_fn(3, |r, _| if r == col { 1.0 } else { 0.0 });
                    let solved = decomp.solve(&e).unwrap_or(na::DVector::zeros(3));
                    let jp_col = &jac.transpose() * &solved;
                    for row in 0..n {
                        jp[(row, col)] = jp_col[row];
                    }
                }
                jp
            };
            // Null-space projector: N = I − J⁺ J
            let identity_n = na::DMatrix::<f64>::identity(n, n);
            let null_proj = &identity_n - &j_pinv * &jac;

            // Posture error: pull toward reference
            let k_ns = 0.5;
            let mut dq_posture = na::DVector::<f64>::zeros(n);
            for (i, &ji) in chain.iter().enumerate() {
                if i < ref_pos.len() {
                    dq_posture[i] = k_ns * (ref_pos[i] - self.joint_positions[ji]);
                }
            }

            &dq_primary + &null_proj * &dq_posture
        } else {
            dq_primary
        };

        (0..n).map(|i| dq[i]).collect()
    }

    /// Build collision `GeometryModel` from current model data.
    pub fn build_collision_geometry(&self) -> GeometryModel {
        self.build_collision_geometry_with_map().0
    }

    /// Build collision `GeometryModel` with a map from geo-obj index → `(link_idx, collision_idx)`.
    pub fn build_collision_geometry_with_map(&self) -> (GeometryModel, Vec<(usize, usize)>) {
        let mc = self.mc();
        let mut gmodel = GeometryModel::new();
        let mut geo_map: Vec<(usize, usize)> = Vec::new();

        for link in &self.links {
            let parent_joint = mc.link_name_to_misarta_joint(&link.name).unwrap_or(0);
            let li = self.link_map.get(&link.name).copied().unwrap_or(0);

            for (ci, col) in link.collisions.iter().enumerate() {
                let (shape, mesh_data) = match convert_geom_to_shape_with_mesh(&col.geometry) {
                    Some(pair) => pair,
                    None => continue,
                };
                let placement = col.origin.cast::<f64>();
                let mesh_scale = match &col.geometry {
                    GeomData::Mesh { scale, .. } => {
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

    /// Apply a solved misarta q-vector back to joint positions.
    pub fn apply_q(&mut self, q: &[f64]) {
        // Take cache temporarily to avoid borrow conflict
        let mc = self.misarta_cache.take().expect("misarta model not built");
        mc.apply_q_to_robot(self, q);
        self.misarta_cache = Some(mc);
    }

    /// Enforce mimic constraints on current joint positions.
    pub fn enforce_mimic(&mut self) {
        let mc = self.misarta_cache.as_ref().expect("misarta model not built");
        if mc.model.mimic.is_empty() {
            return;
        }
        let q = mc.build_q(self);
        let q_enforced = misarta::mimic::enforce_mimic(&mc.model, &q);
        // Must clone a2m since we borrow self mutably below
        let a2m = mc.a2m.clone();
        let model_joints = &mc.model.joints;
        let q_idx = &mc.model.q_idx;
        for (ji, &maybe_mi) in a2m.iter().enumerate() {
            if let Some(mi) = maybe_mi {
                let nq = model_joints[mi].joint_type.nq();
                if nq == 1 {
                    self.joint_positions[ji] = q_enforced[q_idx[mi]];
                }
            }
        }
    }
}

// =========================================================================
//  Constraint IK via misarta
// =========================================================================

use misarta::constraint::{
    ConstrainedIkConfig, ConstrainedIkResult, ConstraintModel,
    RigidConstraint,
};
use misarta::frames::Frame;

impl RobotModel {
    /// Build a misarta `Frame` for an articara link name.
    pub fn frame_for_link(&self, link_name: &str) -> Option<Frame<f64>> {
        let mc = self.mc();
        let mi = mc.link_name_to_misarta_joint(link_name)?;
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
        offset: na::Isometry3<f64>,
    ) -> Option<Frame<f64>> {
        let mc = self.mc();
        let mi = mc.link_name_to_misarta_joint(link_name)?;
        Some(Frame {
            name: link_name.to_string(),
            parent_joint: mi,
            placement: offset,
        })
    }

    /// Create a position-only (3D) constraint between two links.
    pub fn position_constraint(
        &self,
        link_a: &str,
        link_b: &str,
    ) -> Option<RigidConstraint<f64>> {
        let f1 = self.frame_for_link(link_a)?;
        let f2 = self.frame_for_link(link_b)?;
        Some(RigidConstraint::position(f1, f2))
    }

    /// Create a full-pose (6D) constraint between two links.
    pub fn pose_constraint(
        &self,
        link_a: &str,
        link_b: &str,
    ) -> Option<RigidConstraint<f64>> {
        let f1 = self.frame_for_link(link_a)?;
        let f2 = self.frame_for_link(link_b)?;
        Some(RigidConstraint::pose(f1, f2))
    }

    /// Solve constrained IK.
    pub fn solve_constrained_ik(
        &self,
        constraints: Vec<RigidConstraint<f64>>,
        config: &ConstrainedIkConfig,
    ) -> ConstrainedIkResult {
        let mc = self.mc();
        let q0 = mc.build_q(self);
        let cm = ConstraintModel::from_constraints(constraints);
        misarta::constraint::solve_constrained_ik(&mc.model, &q0, &cm, config)
    }

    /// Solve IK with a primary task (position) and rigid constraints.
    pub fn solve_task_with_constraints(
        &self,
        ee_link: &str,
        target: na::Vector3<f64>,
        constraints: Vec<RigidConstraint<f64>>,
        config: &ConstrainedIkConfig,
    ) -> ConstrainedIkResult {
        let mc = self.mc();
        let q0 = mc.build_q(self);
        let cm = ConstraintModel::from_constraints(constraints);
        let joint_idx = match mc.link_name_to_misarta_joint(ee_link) {
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
            &mc.model, &q0, joint_idx, target, &cm, config,
        )
    }
}

// ─── Conversion helpers ─────────────────────────────────────────────────────

/// Convert an articara `JointData.joint_type` string + axis to a misarta `JointType`.
fn convert_joint_type(joint: &JointData) -> JointType<f64> {
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
fn convert_link_inertia(link: &LinkData) -> LinkInertia<f64> {
    let i = &link.inertial;
    let mass = i.mass;
    let com = i.origin.translation.vector.cast::<f64>();
    let rot = i.origin.rotation.to_rotation_matrix();
    let r = rot.matrix().cast::<f64>();

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
pub fn isometry_f64_to_f32(iso: &na::Isometry3<f64>) -> na::Isometry3<f32> {
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
pub fn convert_geom_to_shape_with_mesh(
    geom: &GeomData,
) -> Option<(GeometryShape, Option<MeshData>)> {
    match geom {
        GeomData::Box { hx, hy, hz } => {
            Some((GeometryShape::Box {
                x: *hx as f64 * 2.0,
                y: *hy as f64 * 2.0,
                z: *hz as f64 * 2.0,
            }, None))
        }
        GeomData::Sphere { radius } => {
            Some((GeometryShape::Sphere {
                radius: *radius as f64,
            }, None))
        }
        GeomData::Cylinder { radius, half_length } => {
            Some((GeometryShape::Cylinder {
                radius: *radius as f64,
                length: *half_length as f64 * 2.0,
            }, None))
        }
        GeomData::Capsule { radius, half_length } => {
            Some((GeometryShape::Capsule {
                radius: *radius as f64,
                length: *half_length as f64 * 2.0,
            }, None))
        }
        GeomData::Mesh { vertices, scale, .. } => {
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

            Some((GeometryShape::Mesh {
                scale: na::Vector3::new(1.0, 1.0, 1.0),
                filename: String::new(),
            }, Some(md)))
        }
    }
}
