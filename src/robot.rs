use nalgebra as na;
use std::collections::{HashMap, HashSet};
use std::io::BufReader;
use std::path::{Path, PathBuf};

// ========== Data Structures ==========

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
}

pub struct LinkData {
    pub name: String,
    pub visuals: Vec<VisualData>,
    pub collisions: Vec<CollisionData>,
    pub inertial: InertialData,
}

pub struct VisualData {
    pub origin: na::Isometry3<f32>,
    pub geometry: GeomData,
    pub color: [f32; 4],
}

pub struct CollisionData {
    pub origin: na::Isometry3<f32>,
    pub geometry: GeomData,
}

pub enum GeomData {
    Box { hx: f32, hy: f32, hz: f32 },
    Cylinder { radius: f32, half_length: f32 },
    Sphere { radius: f32 },
    Mesh { vertices: Vec<f32> }, // flat [x, y, z, nx, ny, nz, ...]
}

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

// ========== URDF Loading ==========

impl RobotModel {
    pub fn from_urdf(path: &Path) -> Result<Self, String> {
        let robot = urdf_rs::read_file(path).map_err(|e| format!("URDF parse error: {e}"))?;

        let urdf_dir = path.parent().unwrap_or(Path::new("."));
        let package_dir = urdf_dir.parent().unwrap_or(urdf_dir);

        // Materials
        let mut materials: HashMap<String, [f32; 4]> = HashMap::new();
        for mat in &robot.materials {
            if let Some(ref color) = mat.color {
                materials.insert(
                    mat.name.clone(),
                    [
                        color.rgba.0[0] as f32,
                        color.rgba.0[1] as f32,
                        color.rgba.0[2] as f32,
                        color.rgba.0[3] as f32,
                    ],
                );
            }
        }

        // Links
        let mut links = Vec::new();
        let mut link_map = HashMap::new();
        for (i, link) in robot.links.iter().enumerate() {
            let visuals = link
                .visual
                .iter()
                .map(|vis| {
                    let origin = pose_to_isometry(&vis.origin);
                    let color = vis
                        .material
                        .as_ref()
                        .and_then(|m| {
                            m.color
                                .as_ref()
                                .map(|c| {
                                    [
                                        c.rgba.0[0] as f32,
                                        c.rgba.0[1] as f32,
                                        c.rgba.0[2] as f32,
                                        c.rgba.0[3] as f32,
                                    ]
                                })
                                .or_else(|| materials.get(&m.name).copied())
                        })
                        .unwrap_or([0.8, 0.8, 0.8, 1.0]);
                    let geometry = convert_geometry(&vis.geometry, package_dir);
                    VisualData {
                        origin,
                        geometry,
                        color,
                    }
                })
                .collect();

            let collisions = link
                .collision
                .iter()
                .map(|col| CollisionData {
                    origin: pose_to_isometry(&col.origin),
                    geometry: convert_geometry(&col.geometry, package_dir),
                })
                .collect();

            let inertial = InertialData {
                origin: pose_to_isometry(&link.inertial.origin),
                mass: link.inertial.mass.value,
                ixx: link.inertial.inertia.ixx,
                ixy: link.inertial.inertia.ixy,
                ixz: link.inertial.inertia.ixz,
                iyy: link.inertial.inertia.iyy,
                iyz: link.inertial.inertia.iyz,
                izz: link.inertial.inertia.izz,
            };

            link_map.insert(link.name.clone(), i);
            links.push(LinkData {
                name: link.name.clone(),
                visuals,
                collisions,
                inertial,
            });
        }

        // Joints
        let mut joints = Vec::new();
        let mut joint_map = HashMap::new();
        let mut children_joints: HashMap<String, Vec<usize>> = HashMap::new();
        let mut child_links: HashSet<String> = HashSet::new();

        for (i, joint) in robot.joints.iter().enumerate() {
            let jtype = format!("{:?}", joint.joint_type).to_lowercase();
            let origin = pose_to_isometry(&joint.origin);
            let axis = na::Vector3::new(
                joint.axis.xyz.0[0] as f32,
                joint.axis.xyz.0[1] as f32,
                joint.axis.xyz.0[2] as f32,
            );

            joint_map.insert(joint.name.clone(), i);
            children_joints
                .entry(joint.parent.link.clone())
                .or_default()
                .push(i);
            child_links.insert(joint.child.link.clone());

            joints.push(JointData {
                name: joint.name.clone(),
                joint_type: jtype,
                parent_link: joint.parent.link.clone(),
                child_link: joint.child.link.clone(),
                origin,
                axis,
                lower: joint.limit.lower,
                upper: joint.limit.upper,
                effort: joint.limit.effort,
                velocity: joint.limit.velocity,
            });
        }

        // Root link = not a child of any joint
        let root_link = links
            .iter()
            .find(|l| !child_links.contains(&l.name))
            .map(|l| l.name.clone())
            .unwrap_or_default();

        let joint_positions = vec![0.0f32; joints.len()];

        log::info!(
            "Loaded robot '{}': {} links, {} joints, root='{}'",
            robot.name,
            links.len(),
            joints.len(),
            root_link
        );

        Ok(Self {
            name: robot.name.clone(),
            links,
            joints,
            link_map,
            joint_map,
            root_link,
            children_joints,
            materials,
            joint_positions,
        })
    }

    /// Compute world transforms for all links based on current joint positions.
    pub fn compute_transforms(&self) -> HashMap<String, na::Isometry3<f32>> {
        let mut transforms: HashMap<String, na::Isometry3<f32>> = HashMap::new();
        transforms.insert(self.root_link.clone(), na::Isometry3::identity());

        let mut stack = vec![self.root_link.clone()];
        while let Some(link_name) = stack.pop() {
            let parent_tf = transforms[&link_name];
            if let Some(child_joints) = self.children_joints.get(&link_name) {
                for &ji in child_joints {
                    let joint = &self.joints[ji];
                    let joint_rotation = match joint.joint_type.as_str() {
                        "revolute" | "continuous" => {
                            let angle = self.joint_positions[ji];
                            na::Isometry3::from_parts(
                                na::Translation3::identity(),
                                na::UnitQuaternion::from_axis_angle(
                                    &na::Unit::new_normalize(joint.axis),
                                    angle,
                                ),
                            )
                        }
                        "prismatic" => {
                            let offset = self.joint_positions[ji];
                            na::Isometry3::from_parts(
                                na::Translation3::from(joint.axis * offset),
                                na::UnitQuaternion::identity(),
                            )
                        }
                        _ => na::Isometry3::identity(),
                    };
                    let child_tf = parent_tf * joint.origin * joint_rotation;
                    transforms.insert(joint.child_link.clone(), child_tf);
                    stack.push(joint.child_link.clone());
                }
            }
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

    /// Compute a bounding sphere (center, radius) for a link's visual geometry
    /// in the link's local frame. Properly transforms all vertices by visual origin.
    pub fn link_bounding_sphere(&self, link_idx: usize) -> (na::Point3<f32>, f32) {
        let link = &self.links[link_idx];
        if link.visuals.is_empty() {
            return (na::Point3::origin(), 0.0);
        }

        // Collect representative points in link-local frame (transformed by visual origin)
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
                GeomData::Mesh { vertices } => {
                    // Sample every Nth vertex for large meshes to keep it fast
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

        // Compute centroid
        let sum = points.iter().fold(na::Vector3::zeros(), |a, p| a + p.coords);
        let center = na::Point3::from(sum / points.len() as f32);

        // Radius = max distance from center to any point
        let radius = points
            .iter()
            .map(|p| na::distance(&center, p))
            .fold(0.001_f32, f32::max);

        (center, radius * 1.05) // 5% margin
    }

    /// Pick: find the closest link hit by a ray, given current world transforms.
    /// Uses two-pass: bounding sphere (coarse) → triangle/analytic intersection (precise).
    /// Returns (link_index, distance) or None.
    pub fn pick_link(
        &self,
        ray_origin: &na::Point3<f32>,
        ray_dir: &na::Vector3<f32>,
        transforms: &std::collections::HashMap<String, na::Isometry3<f32>>,
    ) -> Option<(usize, f32)> {
        let mut best: Option<(usize, f32)> = None;

        for (li, link) in self.links.iter().enumerate() {
            let world_tf = transforms
                .get(&link.name)
                .copied()
                .unwrap_or(na::Isometry3::identity());

            // Coarse pass: bounding sphere
            let (local_center, radius) = self.link_bounding_sphere(li);
            if radius < 1e-6 {
                continue; // Skip links with no visual geometry
            }
            let world_center = world_tf * local_center;
            if ray_sphere_intersect(ray_origin, ray_dir, &world_center, radius).is_none() {
                continue; // Ray misses bounding sphere
            }

            // Precise pass: test against actual geometry of each visual
            let mut link_best_dist: Option<f32> = None;
            for vis in &link.visuals {
                let full_tf = world_tf * vis.origin;
                let dist = precise_geometry_intersect(
                    ray_origin, ray_dir, &full_tf, &vis.geometry,
                );
                if let Some(d) = dist {
                    if d > 0.0 && (link_best_dist.is_none() || d < link_best_dist.unwrap()) {
                        link_best_dist = Some(d);
                    }
                }
            }

            if let Some(d) = link_best_dist {
                if best.is_none() || d < best.unwrap().1 {
                    best = Some((li, d));
                }
            }
        }
        best
    }
}

// ========== Ray Intersection Tests ==========

/// Precise geometry intersection: transforms ray into geometry-local space and tests.
fn precise_geometry_intersect(
    ray_origin: &na::Point3<f32>,
    ray_dir: &na::Vector3<f32>,
    geom_tf: &na::Isometry3<f32>,
    geom: &GeomData,
) -> Option<f32> {
    // Transform ray into geometry's local frame
    let inv_tf = geom_tf.inverse();
    let local_origin = inv_tf * ray_origin;
    let local_dir = inv_tf * ray_dir;

    match geom {
        GeomData::Box { hx, hy, hz } => ray_box_intersect(&local_origin, &local_dir, *hx, *hy, *hz),
        GeomData::Cylinder { radius, half_length } => {
            ray_cylinder_intersect(&local_origin, &local_dir, *radius, *half_length)
        }
        GeomData::Sphere { radius } => {
            ray_sphere_intersect(&local_origin, &local_dir, &na::Point3::origin(), *radius)
        }
        GeomData::Mesh { vertices } => ray_mesh_intersect(&local_origin, &local_dir, vertices),
    }
}

/// Ray-sphere intersection. Returns the nearest positive distance or None.
fn ray_sphere_intersect(
    origin: &na::Point3<f32>,
    dir: &na::Vector3<f32>,
    center: &na::Point3<f32>,
    radius: f32,
) -> Option<f32> {
    let oc = origin - center;
    let a = dir.dot(dir);
    let b = 2.0 * oc.dot(dir);
    let c = oc.dot(&oc) - radius * radius;
    let disc = b * b - 4.0 * a * c;
    if disc < 0.0 {
        return None;
    }
    let sqrt_disc = disc.sqrt();
    let t1 = (-b - sqrt_disc) / (2.0 * a);
    let t2 = (-b + sqrt_disc) / (2.0 * a);
    if t1 > 0.0 {
        Some(t1)
    } else if t2 > 0.0 {
        Some(t2)
    } else {
        None
    }
}

/// Ray-AABB (box) intersection using slab method.
fn ray_box_intersect(
    origin: &na::Point3<f32>,
    dir: &na::Vector3<f32>,
    hx: f32,
    hy: f32,
    hz: f32,
) -> Option<f32> {
    let mut tmin = f32::NEG_INFINITY;
    let mut tmax = f32::INFINITY;
    let halves = [hx, hy, hz];

    for i in 0..3 {
        if dir[i].abs() < 1e-10 {
            if origin[i] < -halves[i] || origin[i] > halves[i] {
                return None;
            }
        } else {
            let inv_d = 1.0 / dir[i];
            let mut t1 = (-halves[i] - origin[i]) * inv_d;
            let mut t2 = (halves[i] - origin[i]) * inv_d;
            if t1 > t2 {
                std::mem::swap(&mut t1, &mut t2);
            }
            tmin = tmin.max(t1);
            tmax = tmax.min(t2);
            if tmin > tmax {
                return None;
            }
        }
    }
    if tmax < 0.0 {
        None
    } else if tmin > 0.0 {
        Some(tmin)
    } else {
        Some(tmax)
    }
}

/// Ray-cylinder intersection (Z-axis aligned, centered at origin).
fn ray_cylinder_intersect(
    origin: &na::Point3<f32>,
    dir: &na::Vector3<f32>,
    radius: f32,
    half_length: f32,
) -> Option<f32> {
    // Infinite cylinder in XY
    let a = dir.x * dir.x + dir.y * dir.y;
    let b = 2.0 * (origin.x * dir.x + origin.y * dir.y);
    let c = origin.x * origin.x + origin.y * origin.y - radius * radius;
    let disc = b * b - 4.0 * a * c;

    let mut best: Option<f32> = None;

    if disc >= 0.0 && a > 1e-10 {
        let sqrt_disc = disc.sqrt();
        for &t in &[(-b - sqrt_disc) / (2.0 * a), (-b + sqrt_disc) / (2.0 * a)] {
            if t > 0.0 {
                let z = origin.z + t * dir.z;
                if z.abs() <= half_length {
                    if best.is_none() || t < best.unwrap() {
                        best = Some(t);
                    }
                }
            }
        }
    }

    // Cap discs (top and bottom)
    if dir.z.abs() > 1e-10 {
        for &cap_z in &[half_length, -half_length] {
            let t = (cap_z - origin.z) / dir.z;
            if t > 0.0 {
                let px = origin.x + t * dir.x;
                let py = origin.y + t * dir.y;
                if px * px + py * py <= radius * radius {
                    if best.is_none() || t < best.unwrap() {
                        best = Some(t);
                    }
                }
            }
        }
    }

    best
}

/// Ray-mesh (triangle soup) intersection using Möller–Trumbore algorithm.
/// Vertices are in flat format: [x, y, z, nx, ny, nz, x, y, z, nx, ny, nz, ...].
/// Every 3 vertices (18 floats) form one triangle.
fn ray_mesh_intersect(
    origin: &na::Point3<f32>,
    dir: &na::Vector3<f32>,
    vertices: &[f32],
) -> Option<f32> {
    let mut best: Option<f32> = None;
    let stride = 6; // x,y,z,nx,ny,nz per vertex
    let tri_stride = stride * 3; // 18 floats per triangle

    let mut i = 0;
    while i + tri_stride <= vertices.len() {
        let v0 = na::Point3::new(vertices[i], vertices[i + 1], vertices[i + 2]);
        let v1 = na::Point3::new(vertices[i + stride], vertices[i + stride + 1], vertices[i + stride + 2]);
        let v2 = na::Point3::new(vertices[i + stride * 2], vertices[i + stride * 2 + 1], vertices[i + stride * 2 + 2]);

        if let Some(t) = ray_triangle_intersect(origin, dir, &v0, &v1, &v2) {
            if t > 0.0 && (best.is_none() || t < best.unwrap()) {
                best = Some(t);
            }
        }
        i += tri_stride;
    }
    best
}

/// Möller–Trumbore ray-triangle intersection.
fn ray_triangle_intersect(
    origin: &na::Point3<f32>,
    dir: &na::Vector3<f32>,
    v0: &na::Point3<f32>,
    v1: &na::Point3<f32>,
    v2: &na::Point3<f32>,
) -> Option<f32> {
    let edge1 = v1 - v0;
    let edge2 = v2 - v0;
    let h = dir.cross(&edge2);
    let a = edge1.dot(&h);
    if a.abs() < 1e-8 {
        return None; // Ray parallel to triangle
    }
    let f = 1.0 / a;
    let s = origin - v0;
    let u = f * s.dot(&h);
    if !(0.0..=1.0).contains(&u) {
        return None;
    }
    let q = s.cross(&edge1);
    let v = f * dir.dot(&q);
    if v < 0.0 || u + v > 1.0 {
        return None;
    }
    let t = f * edge2.dot(&q);
    if t > 1e-6 {
        Some(t)
    } else {
        None
    }
}

// ========== Helper Functions ==========

fn pose_to_isometry(pose: &urdf_rs::Pose) -> na::Isometry3<f32> {
    let xyz = &pose.xyz.0;
    let rpy = &pose.rpy.0;
    let translation = na::Translation3::new(xyz[0] as f32, xyz[1] as f32, xyz[2] as f32);
    let rotation =
        na::UnitQuaternion::from_euler_angles(rpy[0] as f32, rpy[1] as f32, rpy[2] as f32);
    na::Isometry3::from_parts(translation, rotation)
}

fn convert_geometry(geom: &urdf_rs::Geometry, package_dir: &Path) -> GeomData {
    match geom {
        urdf_rs::Geometry::Box { size } => GeomData::Box {
            hx: size.0[0] as f32 / 2.0,
            hy: size.0[1] as f32 / 2.0,
            hz: size.0[2] as f32 / 2.0,
        },
        urdf_rs::Geometry::Cylinder { radius, length } => GeomData::Cylinder {
            radius: *radius as f32,
            half_length: *length as f32 / 2.0,
        },
        urdf_rs::Geometry::Sphere { radius } => GeomData::Sphere {
            radius: *radius as f32,
        },
        urdf_rs::Geometry::Mesh { filename, scale } => {
            let mesh_path = resolve_package_path(filename, package_dir);
            let vertices = load_stl_mesh(&mesh_path, scale.as_ref());
            GeomData::Mesh { vertices }
        }
        _ => GeomData::Box {
            hx: 0.01,
            hy: 0.01,
            hz: 0.01,
        },
    }
}

fn resolve_package_path(filename: &str, package_dir: &Path) -> PathBuf {
    if let Some(rest) = filename.strip_prefix("package://") {
        if let Some(slash_pos) = rest.find('/') {
            let rel_path = &rest[slash_pos + 1..];
            package_dir.join(rel_path)
        } else {
            package_dir.join(rest)
        }
    } else if filename.starts_with("file://") {
        PathBuf::from(filename.strip_prefix("file://").unwrap())
    } else {
        PathBuf::from(filename)
    }
}

fn load_stl_mesh(path: &PathBuf, scale: Option<&urdf_rs::Vec3>) -> Vec<f32> {
    let sf = scale
        .map(|s| [s.0[0] as f32, s.0[1] as f32, s.0[2] as f32])
        .unwrap_or([1.0, 1.0, 1.0]);

    let file = match std::fs::File::open(path) {
        Ok(f) => f,
        Err(e) => {
            log::warn!("Failed to open STL file {:?}: {}", path, e);
            return Vec::new();
        }
    };
    let mut reader = BufReader::new(file);
    let mesh = match stl_io::read_stl(&mut reader) {
        Ok(m) => m,
        Err(e) => {
            log::warn!("Failed to read STL {:?}: {}", path, e);
            return Vec::new();
        }
    };

    let mut vertices = Vec::with_capacity(mesh.faces.len() * 3 * 6);
    for face in &mesh.faces {
        let nx = face.normal[0];
        let ny = face.normal[1];
        let nz = face.normal[2];
        for &vi in &face.vertices {
            let vtx = &mesh.vertices[vi];
            vertices.push(vtx[0] * sf[0]);
            vertices.push(vtx[1] * sf[1]);
            vertices.push(vtx[2] * sf[2]);
            vertices.push(nx);
            vertices.push(ny);
            vertices.push(nz);
        }
    }

    log::info!(
        "Loaded STL {:?}: {} triangles",
        path.file_name().unwrap_or_default(),
        mesh.faces.len()
    );
    vertices
}
