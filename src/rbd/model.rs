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

// ========== IK Solver Variants ==========

/// Differential IK solver method.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum IkSolver {
    /// Damped Least Squares with fixed λ.
    ///   δq = Jᵀ(JJᵀ + λ²I)⁻¹ · Δx
    Dls,
    /// Singularity-Robust Inverse (SR-Inverse, Nakamura & Hanafusa 1986).
    /// λ adapts based on manipulability: large near singularities, zero away.
    ///   λ² = λ_max²·(1 − (w/w₀)²)  when w < w₀,  else 0
    SrInverse,
    /// Jacobian Transpose.
    ///   δq = α·Jᵀ·Δx,  α = Δxᵀ·J·Jᵀ·Δx / ‖JJᵀΔx‖²
    JacobianTranspose,
}

impl IkSolver {
    /// Human-readable label for UI display.
    pub fn label(self) -> &'static str {
        match self {
            IkSolver::Dls => "DLS",
            IkSolver::SrInverse => "SR-Inverse",
            IkSolver::JacobianTranspose => "Jacobian Transpose",
        }
    }

    /// All variants for iteration.
    pub const ALL: [IkSolver; 3] = [
        IkSolver::Dls,
        IkSolver::SrInverse,
        IkSolver::JacobianTranspose,
    ];
}

/// IK constraint dimensionality.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum IkDof {
    /// 3-DoF: track target in full 3D (x, y, z).
    World3D,
    /// 2-DoF: track target only on the camera screen plane (right, up).
    /// Depth (camera forward) is left unconstrained.
    ScreenPlane2D,
}

impl IkDof {
    pub fn label(self) -> &'static str {
        match self {
            IkDof::World3D => "3D (World)",
            IkDof::ScreenPlane2D => "2D (Screen)",
        }
    }
    pub const ALL: [IkDof; 2] = [IkDof::World3D, IkDof::ScreenPlane2D];
}

/// Pin specification for multi-constraint IK.
#[derive(Clone, Debug)]
pub struct PinSpec {
    /// Link name to pin.
    pub link_name: String,
    /// Target world position.
    pub target_pos: na::Point3<f64>,
    /// Target world orientation (used only when `pose_6dof` is true).
    pub target_rot: na::UnitQuaternion<f64>,
    /// Whether to constrain orientation as well (6-DoF).
    pub pose_6dof: bool,
}

/// A kinematic loop-closure constraint.
///
/// Specifies that a point on `link_a` (at `offset_a` in link-local frame)
/// should coincide with a point on `link_b` (at `offset_b`).
#[derive(Clone, Debug)]
pub struct LoopClosure {
    /// Human-readable name.
    pub name: String,
    /// First link of the loop pair.
    pub link_a: String,
    /// Offset in link_a's local frame (typically a pure translation to a tip).
    pub offset_a: na::Isometry3<f64>,
    /// Second link of the loop pair.
    pub link_b: String,
    /// Offset in link_b's local frame.
    pub offset_b: na::Isometry3<f64>,
    /// Whether to enforce full pose (6-DoF) or position only (3-DoF).
    pub pose_6dof: bool,
}

impl LoopClosure {
    /// Create a position-only (3-DoF) loop closure between two link tips.
    pub fn position(
        name: impl Into<String>,
        link_a: impl Into<String>,
        offset_a: na::Vector3<f64>,
        link_b: impl Into<String>,
        offset_b: na::Vector3<f64>,
    ) -> Self {
        Self {
            name: name.into(),
            link_a: link_a.into(),
            offset_a: na::Isometry3::from_parts(
                na::Translation3::from(offset_a),
                na::UnitQuaternion::identity(),
            ),
            link_b: link_b.into(),
            offset_b: na::Isometry3::from_parts(
                na::Translation3::from(offset_b),
                na::UnitQuaternion::identity(),
            ),
            pose_6dof: false,
        }
    }

    /// Create a 6-DoF (full pose) loop closure between two link tips.
    ///
    /// `offset_a` / `offset_b` are full local-frame transforms; the
    /// constraint solver enforces `link_a · offset_a == link_b · offset_b`
    /// in both translation and rotation.
    pub fn pose(
        name: impl Into<String>,
        link_a: impl Into<String>,
        offset_a: na::Isometry3<f64>,
        link_b: impl Into<String>,
        offset_b: na::Isometry3<f64>,
    ) -> Self {
        Self {
            name: name.into(),
            link_a: link_a.into(),
            offset_a,
            link_b: link_b.into(),
            offset_b,
            pose_6dof: true,
        }
    }

    /// Convert to misarta's serialisable config representation.
    pub fn to_config(&self) -> misarta::config::LoopClosureConfig {
        let q_a = self.offset_a.rotation.quaternion();
        let q_b = self.offset_b.rotation.quaternion();
        misarta::config::LoopClosureConfig {
            name: self.name.clone(),
            link_a: self.link_a.clone(),
            offset_a: self.offset_a.translation.vector.into(),
            rot_a: [q_a.i, q_a.j, q_a.k, q_a.w],
            link_b: self.link_b.clone(),
            offset_b: self.offset_b.translation.vector.into(),
            rot_b: [q_b.i, q_b.j, q_b.k, q_b.w],
            pose_6dof: self.pose_6dof,
        }
    }

    /// Construct from misarta's serialisable config representation.
    pub fn from_config(cfg: &misarta::config::LoopClosureConfig) -> Self {
        let q_a = na::UnitQuaternion::from_quaternion(na::Quaternion::new(
            cfg.rot_a[3], cfg.rot_a[0], cfg.rot_a[1], cfg.rot_a[2],
        ));
        let q_b = na::UnitQuaternion::from_quaternion(na::Quaternion::new(
            cfg.rot_b[3], cfg.rot_b[0], cfg.rot_b[1], cfg.rot_b[2],
        ));
        Self {
            name: cfg.name.clone(),
            link_a: cfg.link_a.clone(),
            offset_a: na::Isometry3::from_parts(
                na::Translation3::from(na::Vector3::from(cfg.offset_a)),
                q_a,
            ),
            link_b: cfg.link_b.clone(),
            offset_b: na::Isometry3::from_parts(
                na::Translation3::from(na::Vector3::from(cfg.offset_b)),
                q_b,
            ),
            pose_6dof: cfg.pose_6dof,
        }
    }
}

// ========== Data Structures ==========

/// Cached misarta model + index mappings.
#[derive(Clone, Debug)]
pub struct MisartaCache {
    pub model: Model<f64>,
    /// `a2m[articara_joint_idx]` → misarta joint index (1-based), or `None`.
    pub a2m: Vec<Option<usize>>,
    /// `m2a[misarta_joint_idx]` → articara joint index. Index 0 (universe) → `None`.
    #[allow(dead_code)]
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
    #[allow(dead_code)]
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
    /// Kinematic loop-closure constraints.
    /// Populated via UI or from model file metadata.
    #[cfg_attr(feature = "serde", serde(skip))]
    pub loop_closures: Vec<LoopClosure>,
    /// Named joint-space poses for replay during simulation. Keyed by joint
    /// name so renames don't break stored poses.
    #[cfg_attr(feature = "serde", serde(skip))]
    pub poses: Vec<NamedPose>,
    /// Per-link-pair collision overrides. Pairs are stored with `link_a <
    /// link_b` (alphabetical) so the matrix UI / TOML stay symmetric and
    /// diff-friendly. Default behaviour for unlisted pairs is "collide".
    #[cfg_attr(feature = "serde", serde(skip))]
    pub collision_pairs: Vec<CollisionPair>,
    /// Named pose sequences for chained replay.
    #[cfg_attr(feature = "serde", serde(skip))]
    pub sequences: Vec<Sequence>,
    /// Coupled (mimic) joints — `joint = multiplier · source + offset`.
    #[cfg_attr(feature = "serde", serde(skip))]
    pub mimics: Vec<Mimic>,
    /// Sensors mounted on links.
    #[cfg_attr(feature = "serde", serde(skip))]
    pub sensors: Vec<Sensor>,
}

/// Linear coupling between two joints (in-memory mirror of
/// [`misarta::config::MimicConfig`]).
#[derive(Clone, Debug)]
pub struct Mimic {
    pub joint: String,
    pub source: String,
    pub multiplier: f64,
    pub offset: f64,
}

/// Sensor attached to a link (in-memory mirror of
/// [`misarta::config::SensorConfig`]).
#[derive(Clone, Debug)]
pub struct Sensor {
    pub name: String,
    pub link: String,
    pub origin: na::Isometry3<f64>,
    pub update_rate: f64,
    pub kind: SensorKind,
}

#[derive(Clone, Debug)]
pub enum SensorKind {
    Camera {
        fov: f64,
        width: u32,
        height: u32,
        near: f64,
        far: f64,
    },
    Lidar {
        range_min: f64,
        range_max: f64,
        h_fov: f64,
        h_samples: u32,
        v_fov: f64,
        v_samples: u32,
    },
    Imu {
        gyro_noise: f64,
        accel_noise: f64,
    },
    ForceTorque {
        joint: Option<String>,
    },
    Contact {
        partner: Option<String>,
    },
    Generic {
        kind: String,
        params: std::collections::BTreeMap<String, String>,
    },
}

/// A named, ordered sequence of pose-targets to replay one after another.
#[derive(Clone, Debug)]
pub struct Sequence {
    pub name: String,
    pub steps: Vec<SequenceStep>,
}

/// One step in a [`Sequence`]: which pose to head for, how long the
/// transition into it should take, and which interpolation curve to use.
#[derive(Clone, Debug)]
pub struct SequenceStep {
    pub pose_name: String,
    pub duration: f64,
    pub kind: misarta::trajectory::InterpolationKind,
}

/// Per-link-pair collision setting (in-memory mirror of
/// [`misarta::config::CollisionPairConfig`]).
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct CollisionPair {
    pub link_a: String,
    pub link_b: String,
    /// `true` = collide, `false` = excluded.
    pub enabled: bool,
}

impl CollisionPair {
    /// Build a normalised pair (link names sorted alphabetically) so equality
    /// checks treat `(A, B)` and `(B, A)` the same.
    pub fn new(a: impl Into<String>, b: impl Into<String>, enabled: bool) -> Self {
        let (link_a, link_b) = {
            let a: String = a.into();
            let b: String = b.into();
            if a <= b { (a, b) } else { (b, a) }
        };
        Self { link_a, link_b, enabled }
    }

    /// Whether this pair refers to the same two links as `(a, b)` ignoring
    /// order.
    pub fn matches(&self, a: &str, b: &str) -> bool {
        (self.link_a == a && self.link_b == b)
            || (self.link_a == b && self.link_b == a)
    }
}

/// A user-registered joint-space pose with a display name.
#[derive(Clone, Debug)]
pub struct NamedPose {
    pub name: String,
    /// Joint name → angle (or prismatic displacement). Joints not present
    /// here keep their current model value when the pose is replayed.
    pub angles: std::collections::BTreeMap<String, f64>,
    /// Default transition time (s) used when this pose is replayed. The UI
    /// shows it as a seed value the user can edit per play without modifying
    /// the saved default.
    pub duration: f64,
    /// Default interpolation curve. Same role as `duration` — saved per-pose
    /// but overridable in the UI at playback time.
    pub kind: misarta::trajectory::InterpolationKind,
}

impl NamedPose {
    /// Snapshot the model's current joint positions into a named pose,
    /// using the supplied transition defaults.
    pub fn snapshot(
        name: impl Into<String>,
        model: &RobotModel,
        duration: f64,
        kind: misarta::trajectory::InterpolationKind,
    ) -> Self {
        let mut angles = std::collections::BTreeMap::new();
        for (ji, joint) in model.joints.iter().enumerate() {
            if joint.joint_type == "fixed" {
                continue;
            }
            angles.insert(joint.name.clone(), model.joint_positions[ji]);
        }
        Self { name: name.into(), angles, duration, kind }
    }

    /// Resolve the pose into a full joint-position vector matching
    /// `model.joints` order. Joints not in the pose keep `current[ji]`.
    pub fn to_vector(&self, model: &RobotModel, current: &[f64]) -> Vec<f64> {
        let n = model.joints.len();
        let mut out = vec![0.0; n];
        for (ji, joint) in model.joints.iter().enumerate() {
            out[ji] = self
                .angles
                .get(&joint.name)
                .copied()
                .unwrap_or_else(|| current.get(ji).copied().unwrap_or(0.0));
        }
        out
    }
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

/// Per-joint actuator control mode. Selects which MuJoCo actuator type is
/// emitted in the exported MJCF.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum ActuatorMode {
    /// MuJoCo `<position>` — built-in PD: τ = kp·(q*−q) − kv·qd
    Position,
    /// MuJoCo `<velocity>` — proportional velocity tracker: τ = kv·(qd*−qd)
    Velocity,
    /// MuJoCo `<motor>` — direct torque command (no built-in feedback).
    Torque,
}

impl Default for ActuatorMode {
    fn default() -> Self {
        ActuatorMode::Position
    }
}

impl ActuatorMode {
    pub fn label(self) -> &'static str {
        match self {
            ActuatorMode::Position => "Position",
            ActuatorMode::Velocity => "Velocity",
            ActuatorMode::Torque => "Torque",
        }
    }
    pub const ALL: [ActuatorMode; 3] = [
        ActuatorMode::Position,
        ActuatorMode::Velocity,
        ActuatorMode::Torque,
    ];
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
    /// Actuator control mode for MJCF export and physics sim.
    #[cfg_attr(feature = "serde", serde(default))]
    pub actuator_mode: ActuatorMode,
    /// Position gain (used by Position mode; `kp` in MuJoCo `<position>`).
    #[cfg_attr(feature = "serde", serde(default = "default_kp"))]
    pub actuator_kp: f64,
    /// Damping / velocity gain (used by Position and Velocity modes;
    /// `kv` in MuJoCo `<position>` / `<velocity>`).
    #[cfg_attr(feature = "serde", serde(default = "default_kv"))]
    pub actuator_kv: f64,
    /// Reflected rotor inertia (kg·m² for revolute, kg for prismatic). Mapped
    /// to MuJoCo's `<joint armature="…"/>` on export, this raises the joint's
    /// effective inertia and acts as a natural high-frequency filter for the
    /// external PD controller. Real motors and gearboxes always have non-zero
    /// armature; leaving it at 0 makes the simulator more prone to numerical
    /// oscillation than the physical system would be.
    #[cfg_attr(feature = "serde", serde(default))]
    pub armature: f64,
    /// Passive linear damping coefficient at the joint (N·m·s/rad for revolute,
    /// N·s/m for prismatic). Mapped to MuJoCo's `<joint damping="…"/>`. Models
    /// bearing friction / lubricant drag and absorbs energy at impact, which
    /// dampens the torque spike when a leg lands during a jump.
    #[cfg_attr(feature = "serde", serde(default))]
    pub joint_damping: f64,
}

#[cfg(feature = "serde")]
fn default_kp() -> f64 { 50.0 }
#[cfg(feature = "serde")]
fn default_kv() -> f64 { 5.0 }


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

    /// Compute the world-space minimum Z coordinate across all visual geometry.
    ///
    /// Uses the current joint positions and base transform.  Returns `None` if
    /// the model has no visual geometry at all.
    pub fn compute_min_z(&self) -> Option<f32> {
        let transforms = self.compute_transforms();
        let mut min_z: Option<f32> = None;

        for link in &self.links {
            let world_tf = transforms
                .get(&link.name)
                .copied()
                .unwrap_or(na::Isometry3::identity());

            for vis in &link.visuals {
                let full_tf = world_tf * vis.origin;
                let sample_points = Self::geometry_sample_points(&vis.geometry);
                for p in &sample_points {
                    let wp = full_tf * p;
                    min_z = Some(min_z.map_or(wp.z, |m: f32| m.min(wp.z)));
                }
            }
        }
        min_z
    }

    /// Generate representative sample points for a geometry primitive
    /// (in the geometry's local frame).
    fn geometry_sample_points(geom: &GeomData) -> Vec<na::Point3<f32>> {
        let mut pts = Vec::new();
        match geom {
            GeomData::Box { hx, hy, hz } => {
                for &sx in &[-1.0_f32, 1.0] {
                    for &sy in &[-1.0_f32, 1.0] {
                        for &sz in &[-1.0_f32, 1.0] {
                            pts.push(na::Point3::new(*hx * sx, *hy * sy, *hz * sz));
                        }
                    }
                }
            }
            GeomData::Cylinder { radius, half_length } => {
                for i in 0..8 {
                    let a = (i as f32) * std::f32::consts::TAU / 8.0;
                    let (c, s) = (a.cos(), a.sin());
                    pts.push(na::Point3::new(radius * c, radius * s, *half_length));
                    pts.push(na::Point3::new(radius * c, radius * s, -*half_length));
                }
            }
            GeomData::Sphere { radius } => {
                let r = *radius;
                for &d in &[
                    [r, 0.0, 0.0], [-r, 0.0, 0.0],
                    [0.0, r, 0.0], [0.0, -r, 0.0],
                    [0.0, 0.0, r], [0.0, 0.0, -r],
                ] {
                    pts.push(na::Point3::new(d[0], d[1], d[2]));
                }
            }
            GeomData::Capsule { radius, half_length } => {
                let total_h = *half_length + *radius;
                for i in 0..8 {
                    let a = (i as f32) * std::f32::consts::TAU / 8.0;
                    let (c, s) = (a.cos(), a.sin());
                    pts.push(na::Point3::new(radius * c, radius * s, total_h));
                    pts.push(na::Point3::new(radius * c, radius * s, -total_h));
                }
            }
            GeomData::Mesh { vertices, .. } => {
                let step = if vertices.len() > 6000 { 6 * 10 } else { 6 };
                for chunk in vertices.chunks(step) {
                    if chunk.len() >= 3 {
                        pts.push(na::Point3::new(chunk[0], chunk[1], chunk[2]));
                    }
                }
            }
        }
        pts
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

    /// World position of an arbitrary point on a link, given a local-frame offset.
    pub fn ee_world_pos_at(
        &self,
        link_idx: usize,
        transforms: &HashMap<String, na::Isometry3<f32>>,
        local_offset: &na::Point3<f32>,
    ) -> na::Point3<f32> {
        let link_name = &self.links[link_idx].name;
        let world_tf = transforms
            .get(link_name)
            .copied()
            .unwrap_or(na::Isometry3::identity());
        world_tf * local_offset
    }

    /// World orientation of a link.
    pub fn link_world_orientation(
        &self,
        link_idx: usize,
        transforms: &HashMap<String, na::Isometry3<f32>>,
    ) -> na::UnitQuaternion<f32> {
        let link_name = &self.links[link_idx].name;
        let world_tf = transforms
            .get(link_name)
            .copied()
            .unwrap_or(na::Isometry3::identity());
        world_tf.rotation
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
    #[allow(dead_code)]
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
    #[allow(dead_code)]
    pub fn build_v(&self, velocities: &HashMap<usize, f64>) -> na::DVector<f64> {
        self.mc().build_v(velocities)
    }

    /// Compute 3×chain_len positional Jacobian for a chain of joint indices.
    ///
    /// When `root_link` is `Some`, uses a relative Jacobian.
    /// When `ee_offset_world` is `Some`, shifts the reference point from the
    /// link frame origin to an arbitrary point offset by that vector (world frame).
    /// Returns an f64 matrix expressed in the **world frame**.
    pub fn chain_positional_jacobian(
        &self,
        chain: &[usize],
        ee_link: &str,
        root_link: Option<&str>,
        ee_offset_world: Option<&na::Vector3<f64>>,
    ) -> na::DMatrix<f64> {
        // We want the Jacobian of `click_point` in world frame *with the
        // IK-root pinned to its initial pose* (the same constraint the GUI
        // re-applies via `base_transform` after each Δq). That isn't the
        // same thing as the unconstrained relative Jacobian
        // `J(ee) − J(base)`; pinning the base means we also need to undo
        // the rigid-body twist that the unconstrained motion imparted at
        // p_base. Working out the algebra for a single revolute joint θ
        // with axis `ω` at world position `p_θ`:
        //
        //   v_constrained(p_click) = v_click − v_base − ω_base × (p_click − p_base)
        //
        // Splitting by where θ sits in the URDF tree (relative to the
        // dragged EE link and the IK-root base link):
        //
        //   • θ upstream of EE only    →  v_constr = +ω × (p_click − p_θ)
        //   • θ upstream of BASE only  →  v_constr = −ω × (p_click − p_θ)
        //   • θ common ancestor        →  v_constr = 0
        //   • θ neither                →  v_constr = 0
        //
        // The earlier implementation built `J_rel = J(ee) − J(base)` from
        // misarta's relative Jacobian and tacked a uniform
        // `ω × (click − p_ee)` lever-arm correction onto every column.
        // That correction is right for the EE-upstream case but produces
        // the wrong column (and wrong sign in many geometries) for
        // BASE-upstream joints — the bug surfaced when picking RL_hip with
        // ik_root=RL_foot, where every chain joint sits in the
        // BASE-upstream branch. The dragged link then moved in the wrong
        // direction. Computing each column from the case table above
        // gives the correct constrained Jacobian and falls back to the
        // standard tip-link Jacobian when ik_root = URDF root.
        let n = chain.len();
        if n == 0 {
            return na::DMatrix::zeros(3, 0);
        }

        let mc = self.mc();
        let q = mc.build_q(self);
        let data = misarta::fk::forward_kinematics(&mc.model, &q);

        let ee_mi = mc.link_name_to_misarta_joint(ee_link).unwrap_or(0);
        let base_mi = root_link
            .and_then(|rl| mc.link_name_to_misarta_joint(rl))
            .unwrap_or(0);

        // Click and base reference points, both in URDF-root frame. The
        // result is rotated to world via `base_transform` at the end.
        let r_base = self.base_transform.rotation.to_rotation_matrix();
        let r_base_inv = r_base.transpose();
        let p_ee_root = if ee_mi > 0 {
            misarta::se3::translation(&data.oMi[ee_mi])
        } else {
            na::Vector3::zeros()
        };
        let click_root: na::Vector3<f64> = match ee_offset_world {
            Some(off_world) => p_ee_root + r_base_inv * off_world,
            None => p_ee_root,
        };
        // (We don't need `p_base_root` separately — both branches operate
        // on `click_root` only; see the case table.)

        // Ancestor sets for fast "is θ on the EE/BASE path" lookups.
        let ee_ancestors = ancestor_set(&mc.model, ee_mi);
        let base_ancestors = ancestor_set(&mc.model, base_mi);

        let mut jac = na::DMatrix::<f64>::zeros(3, n);
        for (col, &ji) in chain.iter().enumerate() {
            let mi = match mc.a2m.get(ji).and_then(|x| *x) {
                Some(m) if m > 0 => m,
                _ => continue,
            };
            let in_ee = ee_ancestors.contains(&mi);
            let in_base = base_ancestors.contains(&mi);
            if !in_ee && !in_base {
                continue;
            }

            // Joint axis (angular subspace col 0) and origin in URDF-root.
            let r_joint = misarta::se3::rotation_matrix(&data.oMi[mi]);
            let p_joint = misarta::se3::translation(&data.oMi[mi]);
            let qi = mc.model.q_idx[mi];
            let nq = mc.model.joints[mi].joint_type.nq();
            let s_local =
                mc.model.joints[mi].joint_type.motion_subspace(&q[qi..qi + nq]);
            let s_ang = na::Vector3::new(
                s_local[(0, 0)],
                s_local[(1, 0)],
                s_local[(2, 0)],
            );
            let omega_root = r_joint * s_ang;

            let mut v_root = na::Vector3::zeros();
            // Both branches use `click_root - p_joint` (NOT `p_base - p_joint`):
            // when θ is upstream of base only, pinning the base imparts a
            // rigid-body correction that, evaluated at p_click, also picks
            // up a `−ω × (p_click − p_base)` term — adding it to the raw
            // `−ω × (p_base − p_θ)` from `J(base)` collapses to the form
            // below (see the case derivation in the doc-comment above).
            if in_ee {
                v_root += omega_root.cross(&(click_root - p_joint));
            }
            if in_base {
                v_root -= omega_root.cross(&(click_root - p_joint));
            }
            let v_world = r_base * v_root;
            jac[(0, col)] = v_world.x;
            jac[(1, col)] = v_world.y;
            jac[(2, col)] = v_world.z;
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

    /// Differential IK step.
    ///
    /// Computes a small joint velocity update using the selected solver.
    /// Delegates to [`misarta::ik::differential_ik_step`] for the core solve,
    /// then optionally adds null-space posture stabilization.
    ///
    /// - `screen_axes`: if `Some((right, up))`, project to 2-DoF screen plane.
    /// - `joint_weights`: if `Some`, per-joint cost weights (one per chain element).
    pub fn solve_ik_step(
        &self,
        chain: &[usize],
        ee_link: &str,
        root_link: Option<&str>,
        ee_pos: &na::Point3<f64>,
        target_pos: &na::Point3<f64>,
        damping: f64,
        gain: f64,
        max_step: f64,
        ref_positions: Option<&[f64]>,
        solver: IkSolver,
        screen_axes: Option<(na::Vector3<f64>, na::Vector3<f64>)>,
        joint_weights: Option<&[f64]>,
        ee_offset_world: Option<&na::Vector3<f64>>,
    ) -> Vec<f64> {
        let n = chain.len();
        if n == 0 {
            return Vec::new();
        }

        // Build 3×n positional Jacobian in world frame
        let jac3 = self.chain_positional_jacobian(chain, ee_link, root_link, ee_offset_world);

        // Map articara IkSolver → misarta types
        let misarta_damping = match solver {
            IkSolver::SrInverse => misarta::ik::Damping::AdaptiveManipulability {
                lambda_min: 0.0,
                lambda_max: damping,
                manipulability_threshold: 0.05,
            },
            _ => misarta::ik::Damping::Fixed(damping),
        };
        let misarta_method = match solver {
            IkSolver::JacobianTranspose => misarta::ik::SolverMethod::JacobianTranspose,
            _ => misarta::ik::SolverMethod::DampedLeastSquares,
        };

        // Build task-space projection for 2-DoF screen plane
        let task_projection = screen_axes.map(|(cam_right, cam_up)| {
            let mut p = na::DMatrix::<f64>::zeros(2, 3);
            p[(0, 0)] = cam_right.x; p[(0, 1)] = cam_right.y; p[(0, 2)] = cam_right.z;
            p[(1, 0)] = cam_up.x;    p[(1, 1)] = cam_up.y;    p[(1, 2)] = cam_up.z;
            p
        });

        // Build misarta JointWeights
        let misarta_weights = joint_weights.map(|w| {
            misarta::ik::JointWeights {
                weights: (0..n).map(|i| if i < w.len() { w[i].max(1e-6) } else { 1.0 }).collect(),
            }
        });

        let diff_config = misarta::ik::DiffIkConfig {
            gain,
            max_joint_step: max_step,
            damping: misarta_damping,
            solver_method: misarta_method,
            joint_weights: misarta_weights.clone(),
            task_projection,
        };

        let ee_v = na::Vector3::new(ee_pos.x, ee_pos.y, ee_pos.z);
        let tgt_v = na::Vector3::new(target_pos.x, target_pos.y, target_pos.z);

        let result = misarta::ik::differential_ik_step(&jac3, &ee_v, &tgt_v, &diff_config);

        // Null-space posture stabilization (computed locally since it needs
        // chain→joint_positions mapping that misarta doesn't have).
        if let Some(ref_pos) = ref_positions {
            // Need pseudo-inverse for null-space projector
            let (jac, m) = if let Some((cam_right, cam_up)) = screen_axes {
                let mut p = na::DMatrix::<f64>::zeros(2, 3);
                p[(0, 0)] = cam_right.x; p[(0, 1)] = cam_right.y; p[(0, 2)] = cam_right.z;
                p[(1, 0)] = cam_up.x;    p[(1, 1)] = cam_up.y;    p[(1, 2)] = cam_up.z;
                (&p * &jac3, 2_usize)
            } else {
                (jac3.clone(), 3_usize)
            };

            // Build W⁻¹ diagonal
            let w_inv: Option<Vec<f64>> = joint_weights.map(|w| {
                (0..n).map(|i| 1.0 / (if i < w.len() { w[i] } else { 1.0 }).max(1e-6)).collect()
            });

            // Weighted JJᵀ
            let jjt = if let Some(ref wi) = w_inv {
                let mut jw = jac.clone();
                for col in 0..n {
                    for row in 0..m {
                        jw[(row, col)] *= wi[col];
                    }
                }
                &jw * jac.transpose()
            } else {
                &jac * jac.transpose()
            };

            let lambda_sq = damping * damping;
            let identity_m = na::DMatrix::<f64>::identity(m, m);
            let jjt_reg = &jjt + &identity_m * lambda_sq;
            if let Some(decomp_result) = jjt_reg.lu().solve(&na::DMatrix::identity(m, m)) {
                let mut j_pinv = jac.transpose() * &decomp_result;
                if let Some(ref wi) = w_inv {
                    for row in 0..n {
                        for col in 0..m {
                            j_pinv[(row, col)] *= wi[row];
                        }
                    }
                }
                let identity_n = na::DMatrix::<f64>::identity(n, n);
                let null_proj = &identity_n - &j_pinv * &jac;

                let k_ns = 0.5;
                let mut dq_posture = na::DVector::<f64>::zeros(n);
                for (i, &ji) in chain.iter().enumerate() {
                    if i < ref_pos.len() {
                        dq_posture[i] = k_ns * (ref_pos[i] - self.joint_positions[ji]);
                    }
                }

                let dq_primary = na::DVector::from_vec(result.dq);
                let dq = &dq_primary + &null_proj * &dq_posture;
                return (0..n).map(|i| dq[i]).collect();
            }
        }

        result.dq
    }

    /// Compute 3×nv positional Jacobian for a link in the **full** joint space
    /// (all model DoFs), expressed in the world frame.
    ///
    /// This is used by the multi-constraint IK solver where constraints span
    /// different kinematic branches and must share a common column space.
    ///
    /// When `ee_offset_world` is `Some`, shifts the reference point from the
    /// link frame origin by that vector (world frame): J_v += J_ω × r.
    pub fn link_positional_jacobian_full(
        &self,
        link_name: &str,
        ee_offset_world: Option<&na::Vector3<f64>>,
    ) -> na::DMatrix<f64> {
        let mc = self.mc();
        let q = mc.build_q(self);
        let nv = mc.model.nv;
        let mi = match mc.link_name_to_misarta_joint(link_name) {
            Some(v) if v > 0 => v,
            _ => return na::DMatrix::zeros(3, nv),
        };
        let full6 = misarta::jacobian::compute_joint_jacobian(&mc.model, &q, mi);
        let r = self.base_transform.rotation.to_rotation_matrix();

        let mut jac = na::DMatrix::<f64>::zeros(3, nv);
        for col in 0..nv {
            let v = na::Vector3::new(full6[(3, col)], full6[(4, col)], full6[(5, col)]);
            let mut v_world = r * v;
            // Apply offset correction: v_click = v_origin + ω × r
            if let Some(offset) = ee_offset_world {
                let omega = na::Vector3::new(full6[(0, col)], full6[(1, col)], full6[(2, col)]);
                let omega_world = r * omega;
                v_world += omega_world.cross(offset);
            }
            jac[(0, col)] = v_world[0];
            jac[(1, col)] = v_world[1];
            jac[(2, col)] = v_world[2];
        }
        jac
    }

    /// Compute 6×nv full (angular + linear) Jacobian for a link in the
    /// **full** joint space, expressed in the world frame.
    ///
    /// Row layout: [ω_x, ω_y, ω_z, v_x, v_y, v_z] (Featherstone ordering).
    pub fn link_full_jacobian_full(&self, link_name: &str) -> na::DMatrix<f64> {
        let mc = self.mc();
        let q = mc.build_q(self);
        let nv = mc.model.nv;
        let mi = match mc.link_name_to_misarta_joint(link_name) {
            Some(v) if v > 0 => v,
            _ => return na::DMatrix::zeros(6, nv),
        };
        let full6 = misarta::jacobian::compute_joint_jacobian(&mc.model, &q, mi);
        let r = self.base_transform.rotation.to_rotation_matrix();

        let mut jac = na::DMatrix::<f64>::zeros(6, nv);
        for col in 0..nv {
            // Angular part (rows 0-2)
            let w = na::Vector3::new(full6[(0, col)], full6[(1, col)], full6[(2, col)]);
            let w_world = r * w;
            jac[(0, col)] = w_world[0];
            jac[(1, col)] = w_world[1];
            jac[(2, col)] = w_world[2];
            // Linear part (rows 3-5)
            let v = na::Vector3::new(full6[(3, col)], full6[(4, col)], full6[(5, col)]);
            let v_world = r * v;
            jac[(3, col)] = v_world[0];
            jac[(4, col)] = v_world[1];
            jac[(5, col)] = v_world[2];
        }
        jac
    }

    /// Differential IK step with pinned-link constraints.
    ///
    /// Like [`solve_ik_step`], but additionally enforces equality constraints
    /// that keep pinned links at their target world positions/orientations
    /// (augmented Jacobian approach via
    /// `misarta::ik::differential_ik_step_with_constraints`).
    ///
    /// Returns deltas for **all model joints** (one per `joint_positions` entry),
    /// not just the chain.
    ///
    /// Each pin specifies link name, target position, optional target
    /// orientation, and whether to use 3-DoF (position) or 6-DoF (pose).
    pub fn solve_ik_step_with_pins(
        &self,
        ee_link: &str,
        ee_pos: &na::Point3<f64>,
        target_pos: &na::Point3<f64>,
        pins: &[PinSpec],
        damping: f64,
        gain: f64,
        max_step: f64,
        solver: IkSolver,
        screen_axes: Option<(na::Vector3<f64>, na::Vector3<f64>)>,
        joint_weights_raw: Option<&[f64]>,
        pin_weight: f64,
        extra_constraints: &[misarta::ik::DiffIkConstraint],
        ee_offset_world: Option<&na::Vector3<f64>>,
    ) -> Vec<f64> {
        let mc = self.mc();
        let nv = mc.model.nv;
        if nv == 0 {
            return Vec::new();
        }

        // Full-nv Jacobian for the primary task (EE)
        let jac_ee = self.link_positional_jacobian_full(ee_link, ee_offset_world);

        // Build constraints for each pinned link
        let transforms = self.compute_transforms();
        let mut constraints = Vec::with_capacity(pins.len());
        for pin in pins {
            let li = self.link_map.get(pin.link_name.as_str()).copied();

            if pin.pose_6dof {
                // 6-DoF constraint (position + orientation)
                let jac6 = self.link_full_jacobian_full(&pin.link_name);

                // Current world pose
                let (pin_pos, pin_rot) = li
                    .map(|idx| {
                        let pos = self.ee_world_pos(idx, &transforms).cast::<f64>();
                        let rot = self.link_world_orientation(idx, &transforms).cast::<f64>();
                        (pos, rot)
                    })
                    .unwrap_or((na::Point3::origin(), na::UnitQuaternion::identity()));

                // Position error (rows 3-5 in Featherstone order)
                let pos_err = pin_pos - pin.target_pos;
                // Orientation error: log(R_cur * R_target^{-1}) → axis-angle 3-vector
                let rot_err_q = pin_rot * pin.target_rot.inverse();
                let rot_err = rot_err_q.scaled_axis();

                // Error vector: [ω_err; v_err] (6D, Featherstone order)
                let err = na::DVector::from_column_slice(&[
                    rot_err.x, rot_err.y, rot_err.z,
                    pos_err.x, pos_err.y, pos_err.z,
                ]);
                constraints.push(misarta::ik::DiffIkConstraint {
                    jacobian: jac6,
                    error: err,
                    weight: pin_weight,
                });
            } else {
                // 3-DoF constraint (position only)
                let jac_pin = self.link_positional_jacobian_full(&pin.link_name, None);
                let pin_world = li
                    .and_then(|idx| {
                        let tf = transforms.get(&self.links[idx].name)?;
                        let (center, _) = self.link_bounding_sphere(idx);
                        Some(*tf * center)
                    })
                    .unwrap_or(na::Point3::origin())
                    .cast::<f64>();

                let err = pin_world - pin.target_pos;
                constraints.push(misarta::ik::DiffIkConstraint {
                    jacobian: jac_pin,
                    error: na::DVector::from_column_slice(&[err.x, err.y, err.z]),
                    weight: pin_weight,
                });
            }
        }

        // Map solver → misarta types
        let misarta_damping = match solver {
            IkSolver::SrInverse => misarta::ik::Damping::AdaptiveManipulability {
                lambda_min: 0.0,
                lambda_max: damping,
                manipulability_threshold: 0.05,
            },
            _ => misarta::ik::Damping::Fixed(damping),
        };
        let misarta_method = match solver {
            IkSolver::JacobianTranspose => misarta::ik::SolverMethod::JacobianTranspose,
            _ => misarta::ik::SolverMethod::DampedLeastSquares,
        };

        // Task projection for 2-DoF screen plane
        let task_projection = screen_axes.map(|(cam_right, cam_up)| {
            let mut p = na::DMatrix::<f64>::zeros(2, 3);
            p[(0, 0)] = cam_right.x; p[(0, 1)] = cam_right.y; p[(0, 2)] = cam_right.z;
            p[(1, 0)] = cam_up.x;    p[(1, 1)] = cam_up.y;    p[(1, 2)] = cam_up.z;
            p
        });

        // Full-nv joint weights
        let misarta_weights = joint_weights_raw.map(|w| {
            misarta::ik::JointWeights {
                weights: (0..nv).map(|i| if i < w.len() { w[i].max(1e-6) } else { 1.0 }).collect(),
            }
        });

        let diff_config = misarta::ik::DiffIkConfig {
            gain,
            max_joint_step: max_step,
            damping: misarta_damping,
            solver_method: misarta_method,
            joint_weights: misarta_weights,
            task_projection,
        };

        let ee_v = na::Vector3::new(ee_pos.x, ee_pos.y, ee_pos.z);
        let tgt_v = na::Vector3::new(target_pos.x, target_pos.y, target_pos.z);

        // Append any extra constraints (e.g. loop closures)
        constraints.extend_from_slice(extra_constraints);

        let result = misarta::ik::differential_ik_step_with_constraints(
            &jac_ee, &ee_v, &tgt_v, &constraints, &diff_config,
        );

        // Map full-nv deltas back to articara joint indices
        let mut deltas = vec![0.0_f64; self.joint_positions.len()];
        for (ji, maybe_mi) in mc.a2m.iter().enumerate() {
            if let Some(mi) = maybe_mi {
                let vi = mc.model.q_idx[*mi];
                if vi < result.dq.len() {
                    deltas[ji] = result.dq[vi];
                }
            }
        }
        deltas
    }

    /// Apply all-joint deltas (one per joint_positions entry), clamping to limits.
    pub fn apply_all_joint_deltas(&mut self, deltas: &[f64]) {
        for (ji, d) in deltas.iter().enumerate() {
            if ji < self.joints.len() && d.abs() > 1e-15 {
                let lower = self.joints[ji].lower;
                let upper = self.joints[ji].upper;
                self.joint_positions[ji] = (self.joint_positions[ji] + d).clamp(lower, upper);
            }
        }
    }

    /// Build collision `GeometryModel` from current model data.
    #[allow(dead_code)]
    pub fn build_collision_geometry(&self) -> GeometryModel {
        self.build_collision_geometry_with_map().0
    }

    /// Build collision `GeometryModel` with a map from geo-obj index → `(link_idx, collision_idx)`.
    #[allow(dead_code)]
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
    #[allow(dead_code)]
    pub fn apply_q(&mut self, q: &[f64]) {
        // Take cache temporarily to avoid borrow conflict
        let mc = self.misarta_cache.take().expect("misarta model not built");
        mc.apply_q_to_robot(self, q);
        self.misarta_cache = Some(mc);
    }

    /// Enforce mimic constraints on current joint positions.
    #[allow(dead_code)]
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
    #[allow(dead_code)]
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
    #[allow(dead_code)]
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
    #[allow(dead_code)]
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
    #[allow(dead_code)]
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
    #[allow(dead_code)]
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

    // ─── Loop closure helpers ─────────────────────────────────────────

    /// Build a [`ConstraintModel`] from this model's stored loop closures.
    pub fn build_loop_constraint_model(&self) -> ConstraintModel<f64> {
        let mut constraints = Vec::with_capacity(self.loop_closures.len());
        for lc in &self.loop_closures {
            let f1 = match self.frame_for_link_with_offset(&lc.link_a, lc.offset_a) {
                Some(f) => f,
                None => continue,
            };
            let f2 = match self.frame_for_link_with_offset(&lc.link_b, lc.offset_b) {
                Some(f) => f,
                None => continue,
            };
            let c = if lc.pose_6dof {
                RigidConstraint::pose(f1, f2).with_name(lc.name.clone())
            } else {
                RigidConstraint::position(f1, f2).with_name(lc.name.clone())
            };
            constraints.push(c);
        }
        ConstraintModel::from_constraints(constraints)
    }

    /// Build [`DiffIkConstraint`]s from stored loop closures at the current
    /// configuration, suitable for single-step differential IK.
    pub fn build_loop_diff_constraints(
        &self,
        weight: f64,
    ) -> Vec<misarta::ik::DiffIkConstraint> {
        let cm = self.build_loop_constraint_model();
        if cm.is_empty() {
            return Vec::new();
        }
        let mc = self.mc();
        let q = mc.build_q(self);
        misarta::constraint::build_diff_ik_constraints(&mc.model, &q, &cm, weight)
    }

    /// Compute the current loop-closure error norm.
    pub fn loop_closure_error(&self) -> f64 {
        let cm = self.build_loop_constraint_model();
        if cm.is_empty() {
            return 0.0;
        }
        let mc = self.mc();
        let q = mc.build_q(self);
        let err = misarta::constraint::compute_constraint_error(&mc.model, &q, &cm);
        err.norm()
    }

    /// Build a `KeyframeAnimation` for the named sequence, suitable for
    /// passing to [`crate::mujoco_sim::MujocoSim::start_sequence`]. The
    /// first keyframe is the *current* joint configuration at time 0;
    /// each subsequent keyframe sits at the cumulative sum of the steps'
    /// `duration` values, with the q-vector for the pose looked up via
    /// [`NamedPose::to_vector`] so renames / missing joints are handled.
    /// Returns `None` if the sequence (or any referenced pose) doesn't
    /// exist.
    pub fn build_sequence_animation(
        &self,
        sequence_name: &str,
    ) -> Option<misarta::trajectory::KeyframeAnimation<f64>> {
        let seq = self.sequences.iter().find(|s| s.name == sequence_name)?;
        let mut keyframes = Vec::with_capacity(seq.steps.len() + 1);
        // Anchor: current joint vector at t=0. Use the model's current
        // joint_positions so the first segment starts where the robot is.
        let mut q_prev = self.joint_positions.clone();
        keyframes.push(misarta::trajectory::Keyframe::new(
            0.0,
            q_prev.clone(),
            misarta::trajectory::InterpolationKind::Linear,
        ));
        let mut t_acc = 0.0;
        for step in &seq.steps {
            let pose = self.poses.iter().find(|p| p.name == step.pose_name)?;
            let q_target = pose.to_vector(self, &q_prev);
            t_acc += step.duration;
            keyframes.push(misarta::trajectory::Keyframe::new(
                t_acc,
                q_target.clone(),
                step.kind,
            ));
            q_prev = q_target;
        }
        Some(misarta::trajectory::KeyframeAnimation::new(keyframes))
    }

    /// Build a `MisartaConfig` from the current loop closures, named poses,
    /// and per-joint actuator settings (mode + Kp + Kv).
    pub fn to_misarta_config(&self) -> misarta::config::MisartaConfig {
        let mut cfg = misarta::config::MisartaConfig::new();
        for lc in &self.loop_closures {
            cfg.loop_closure.push(lc.to_config());
        }
        for p in &self.poses {
            cfg.pose.push(misarta::config::PoseConfig {
                name: p.name.clone(),
                angles: p.angles.clone(),
                duration: p.duration,
                kind: p.kind,
            });
        }
        // Persist actuator settings for every movable joint so re-loading
        // restores the exact controller behaviour. Fixed joints have no
        // actuator and are skipped.
        for j in &self.joints {
            if j.joint_type == "fixed" {
                continue;
            }
            cfg.actuator.push(misarta::config::ActuatorConfig {
                joint_name: j.name.clone(),
                mode: actuator_mode_to_config(j.actuator_mode),
                kp: j.actuator_kp,
                kv: j.actuator_kv,
                armature: j.armature,
                joint_damping: j.joint_damping,
            });
        }
        // Persist per-link-pair collision overrides. Pairs are stored
        // alphabetically so the TOML stays diff-friendly.
        for cp in &self.collision_pairs {
            cfg.collision_pair.push(misarta::config::CollisionPairConfig {
                link_a: cp.link_a.clone(),
                link_b: cp.link_b.clone(),
                enabled: cp.enabled,
            });
        }
        // Persist named sequences.
        for seq in &self.sequences {
            cfg.sequence.push(misarta::config::SequenceConfig {
                name: seq.name.clone(),
                steps: seq
                    .steps
                    .iter()
                    .map(|s| misarta::config::SequenceStepConfig {
                        pose_name: s.pose_name.clone(),
                        duration: s.duration,
                        kind: s.kind,
                    })
                    .collect(),
            });
        }
        // Persist mimics.
        for m in &self.mimics {
            cfg.mimic.push(misarta::config::MimicConfig {
                joint: m.joint.clone(),
                source: m.source.clone(),
                multiplier: m.multiplier,
                offset: m.offset,
            });
        }
        // Persist sensors.
        for s in &self.sensors {
            let q = s.origin.rotation.quaternion();
            cfg.sensor.push(misarta::config::SensorConfig {
                name: s.name.clone(),
                link: s.link.clone(),
                origin: s.origin.translation.vector.into(),
                orientation: [q.i, q.j, q.k, q.w],
                update_rate: s.update_rate,
                kind: sensor_kind_to_config(&s.kind),
            });
        }
        cfg
    }

    /// Load loop closures, poses, and actuator settings from a
    /// `MisartaConfig`, replacing any existing ones (and updating per-joint
    /// actuator fields by name match).
    pub fn load_misarta_config(&mut self, cfg: &misarta::config::MisartaConfig) {
        self.loop_closures = cfg
            .loop_closure
            .iter()
            .map(LoopClosure::from_config)
            .collect();
        self.poses = cfg
            .pose
            .iter()
            .map(|p| NamedPose {
                name: p.name.clone(),
                angles: p.angles.clone(),
                duration: p.duration,
                kind: p.kind,
            })
            .collect();
        // Restore actuator settings; joints not mentioned in the config keep
        // their current values so partial sidecars don't blow away unrelated
        // tuning.
        for ac in &cfg.actuator {
            if let Some(&ji) = self.joint_map.get(&ac.joint_name) {
                self.joints[ji].actuator_mode =
                    actuator_mode_from_config(ac.mode);
                self.joints[ji].actuator_kp = ac.kp;
                self.joints[ji].actuator_kv = ac.kv;
                self.joints[ji].armature = ac.armature;
                self.joints[ji].joint_damping = ac.joint_damping;
            }
        }
        // Restore collision pair overrides. We keep entries even when the
        // referenced links are missing (the user might be mid-rename) so
        // round-tripping doesn't silently drop them.
        self.collision_pairs = cfg
            .collision_pair
            .iter()
            .map(|cp| CollisionPair::new(cp.link_a.clone(), cp.link_b.clone(), cp.enabled))
            .collect();
        // Restore sequences.
        self.sequences = cfg
            .sequence
            .iter()
            .map(|sc| Sequence {
                name: sc.name.clone(),
                steps: sc
                    .steps
                    .iter()
                    .map(|s| SequenceStep {
                        pose_name: s.pose_name.clone(),
                        duration: s.duration,
                        kind: s.kind,
                    })
                    .collect(),
            })
            .collect();
        // Restore mimics.
        self.mimics = cfg
            .mimic
            .iter()
            .map(|m| Mimic {
                joint: m.joint.clone(),
                source: m.source.clone(),
                multiplier: m.multiplier,
                offset: m.offset,
            })
            .collect();
        // Restore sensors.
        self.sensors = cfg
            .sensor
            .iter()
            .map(|s| Sensor {
                name: s.name.clone(),
                link: s.link.clone(),
                origin: na::Isometry3::from_parts(
                    na::Translation3::new(s.origin[0], s.origin[1], s.origin[2]),
                    na::UnitQuaternion::from_quaternion(na::Quaternion::new(
                        s.orientation[3],
                        s.orientation[0],
                        s.orientation[1],
                        s.orientation[2],
                    )),
                ),
                update_rate: s.update_rate,
                kind: sensor_kind_from_config(&s.kind),
            })
            .collect();
    }

    /// Try to load the `.misarta.toml` sidecar file next to `source_path`.
    /// Returns `Some(SidecarLoadReport)` when a config was found, parsed, and
    /// applied; `None` when no sidecar exists.
    pub fn load_sidecar_config(&mut self) -> Option<SidecarLoadReport> {
        let src = self.source_path.as_ref()?.clone();
        let toml_path = misarta::config::MisartaConfig::config_path_for(&src);
        if !toml_path.exists() {
            return None;
        }
        match misarta::config::MisartaConfig::load(&toml_path) {
            Ok(cfg) => {
                // Track which actuator entries failed to match a joint name —
                // those silently dropped before, which made it look like the
                // sidecar load did nothing when actually only the lookup failed.
                let mut applied = Vec::new();
                let mut unmatched = Vec::new();
                for ac in &cfg.actuator {
                    if self.joint_map.contains_key(&ac.joint_name) {
                        applied.push(ac.joint_name.clone());
                    } else {
                        unmatched.push(ac.joint_name.clone());
                    }
                }
                self.load_misarta_config(&cfg);
                log::info!(
                    "Loaded {} loop closure(s), {} pose(s), {}/{} actuator setting(s) from {}",
                    self.loop_closures.len(),
                    self.poses.len(),
                    applied.len(),
                    cfg.actuator.len(),
                    toml_path.display()
                );
                if !unmatched.is_empty() {
                    log::warn!(
                        "{} actuator entry(ies) skipped (joint not found in model): {}",
                        unmatched.len(),
                        unmatched.join(", ")
                    );
                }
                Some(SidecarLoadReport {
                    path: toml_path,
                    n_loop_closures: self.loop_closures.len(),
                    n_poses: self.poses.len(),
                    n_actuators_applied: applied.len(),
                    n_actuators_total: cfg.actuator.len(),
                    unmatched_actuators: unmatched,
                })
            }
            Err(e) => {
                log::warn!("Failed to load {}: {}", toml_path.display(), e);
                None
            }
        }
    }

    /// Save loop closures to the `.misarta.toml` sidecar file.
    /// If there are no closures the file is NOT written (and any existing one is left).
    pub fn save_sidecar_config(&self, model_path: &std::path::Path) -> Result<(), String> {
        let cfg = self.to_misarta_config();
        if cfg.is_empty() {
            return Ok(());
        }
        let toml_path = misarta::config::MisartaConfig::config_path_for(model_path);
        cfg.save(&toml_path)
    }
}

/// Summary of what [`RobotModel::load_sidecar_config`] applied. The UI surfaces
/// this in the status bar so the user can confirm at a glance how many
/// actuator entries actually reached `JointData` (and which were silently
/// skipped because the joint name didn't match the model).
#[derive(Debug, Clone)]
pub struct SidecarLoadReport {
    pub path: std::path::PathBuf,
    pub n_loop_closures: usize,
    pub n_poses: usize,
    pub n_actuators_applied: usize,
    pub n_actuators_total: usize,
    pub unmatched_actuators: Vec<String>,
}

/// 1:1 conversion between in-memory [`SensorKind`] and the
/// serialisation-friendly [`misarta::config::SensorKind`].
fn sensor_kind_to_config(k: &SensorKind) -> misarta::config::SensorKind {
    match k {
        SensorKind::Camera { fov, width, height, near, far } => {
            misarta::config::SensorKind::Camera {
                fov: *fov, width: *width, height: *height, near: *near, far: *far,
            }
        }
        SensorKind::Lidar {
            range_min, range_max, h_fov, h_samples, v_fov, v_samples,
        } => misarta::config::SensorKind::Lidar {
            range_min: *range_min,
            range_max: *range_max,
            h_fov: *h_fov,
            h_samples: *h_samples,
            v_fov: *v_fov,
            v_samples: *v_samples,
        },
        SensorKind::Imu { gyro_noise, accel_noise } => {
            misarta::config::SensorKind::Imu {
                gyro_noise: *gyro_noise,
                accel_noise: *accel_noise,
            }
        }
        SensorKind::ForceTorque { joint } => {
            misarta::config::SensorKind::ForceTorque { joint: joint.clone() }
        }
        SensorKind::Contact { partner } => {
            misarta::config::SensorKind::Contact { partner: partner.clone() }
        }
        SensorKind::Generic { kind, params } => {
            misarta::config::SensorKind::Generic {
                kind: kind.clone(),
                params: params.clone(),
            }
        }
    }
}

fn sensor_kind_from_config(k: &misarta::config::SensorKind) -> SensorKind {
    match k {
        misarta::config::SensorKind::Camera { fov, width, height, near, far } => {
            SensorKind::Camera {
                fov: *fov, width: *width, height: *height, near: *near, far: *far,
            }
        }
        misarta::config::SensorKind::Lidar {
            range_min, range_max, h_fov, h_samples, v_fov, v_samples,
        } => SensorKind::Lidar {
            range_min: *range_min,
            range_max: *range_max,
            h_fov: *h_fov,
            h_samples: *h_samples,
            v_fov: *v_fov,
            v_samples: *v_samples,
        },
        misarta::config::SensorKind::Imu { gyro_noise, accel_noise } => {
            SensorKind::Imu {
                gyro_noise: *gyro_noise,
                accel_noise: *accel_noise,
            }
        }
        misarta::config::SensorKind::ForceTorque { joint } => {
            SensorKind::ForceTorque { joint: joint.clone() }
        }
        misarta::config::SensorKind::Contact { partner } => {
            SensorKind::Contact { partner: partner.clone() }
        }
        misarta::config::SensorKind::Generic { kind, params } => {
            SensorKind::Generic {
                kind: kind.clone(),
                params: params.clone(),
            }
        }
    }
}

/// Walk misarta's joint tree from `start` toward joint 0 (the URDF root)
/// collecting every joint encountered into a set. Returns an empty set when
/// `start` is 0. Used by [`RobotModel::chain_positional_jacobian`] to test
/// whether a chain joint lies on the EE / base path.
fn ancestor_set(
    model: &misarta::model::Model<f64>,
    start: usize,
) -> std::collections::HashSet<usize> {
    let mut set = std::collections::HashSet::new();
    if start == 0 || start >= model.joints.len() {
        return set;
    }
    let mut cur = start;
    while cur > 0 {
        set.insert(cur);
        cur = model.joints[cur].parent;
    }
    set
}

// ─── Conversion helpers ─────────────────────────────────────────────────────

/// Project the articara-side [`ActuatorMode`] onto the misarta config enum.
/// Both share the same three variants; the conversion is a 1:1 mapping kept
/// out-of-line so the misarta crate stays free of articara-specific imports.
fn actuator_mode_to_config(m: ActuatorMode) -> misarta::config::ActuatorMode {
    match m {
        ActuatorMode::Position => misarta::config::ActuatorMode::Position,
        ActuatorMode::Velocity => misarta::config::ActuatorMode::Velocity,
        ActuatorMode::Torque => misarta::config::ActuatorMode::Torque,
    }
}

/// Inverse of [`actuator_mode_to_config`].
fn actuator_mode_from_config(m: misarta::config::ActuatorMode) -> ActuatorMode {
    match m {
        misarta::config::ActuatorMode::Position => ActuatorMode::Position,
        misarta::config::ActuatorMode::Velocity => ActuatorMode::Velocity,
        misarta::config::ActuatorMode::Torque => ActuatorMode::Torque,
    }
}

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
#[allow(dead_code)]
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
