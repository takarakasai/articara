//! Robot body structure: data types, FK, tree navigation, inertia computation.
//!
//! This module contains the core data model for rigid body robots,
//! independent of any file format (URDF/SDF/MJCF) or UI framework.

use nalgebra as na;
use std::collections::HashMap;
use std::path::PathBuf;

// ========== Data Structures ==========

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
    pub joint_positions: Vec<f32>,
    /// Path of the originally loaded URDF file.
    #[cfg_attr(feature = "serde", serde(skip))]
    pub source_path: Option<PathBuf>,
    /// World-space transform of the URDF root link (identity by default).
    /// Used to re-root the display when fixing a non-root link as IK base.
    pub base_transform: na::Isometry3<f32>,
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
        let adapter = super::adapter::ModelAdapter::from_robot_model(self);
        adapter.compute_transforms_compat(self)
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
