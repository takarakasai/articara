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
