//! Robot body structure: data types, FK, tree navigation, inertia computation.
//!
//! This module contains the core data model for rigid body robots,
//! independent of any file format (URDF/SDF/MJCF) or UI framework.
//!
//! The kinematic tree, inertia, and dynamics are delegated to the embedded
//! `misarta::Model<f64>`.  GUI-specific data (visual/collision geometry,
//! joint limits, materials) lives alongside in the articara data structures.

use nalgebra as na;
use std::collections::HashMap;
use std::path::PathBuf;

use misarta::model::Model;

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
    /// Quadruped gait presets (in-memory mirror of the sidecar's
    /// `[[gait]]` entries). The host UI / Rhai bindings read/write the
    /// first entry as the active preset; multiple are supported for
    /// future "switch gait by name" workflows.
    #[cfg_attr(feature = "serde", serde(skip))]
    pub gaits: Vec<GaitDescriptor>,
}

/// Quadruped gait preset stored on the model. Mirrors
/// [`misarta::config::GaitConfigEntry`] one-to-one. Holding this on
/// `RobotModel` (rather than ArticaraApp) means the data round-trips
/// through `to_misarta_config` / `load_misarta_config` with the same
/// machinery as poses, sequences, and actuator settings.
#[derive(Clone, Debug)]
pub struct GaitDescriptor {
    pub name: String,
    pub gait_type: misarta::config::GaitTypeConfig,
    pub cycle_period_s: f64,
    pub duty_factor: f64,
    pub swing_height_m: f64,
    pub max_step_length_m: f64,
    pub fl_foot: String,
    pub fr_foot: String,
    pub rl_foot: String,
    pub rr_foot: String,
    pub knee_forward: [bool; 4],
    /// LinearCrawl-only: 4-support fraction of each per-leg sub-cycle.
    /// Ignored by every other [`quadruped_gait::GaitMode`].
    pub four_support_fraction: f64,
}

impl GaitDescriptor {
    /// Sensible defaults: Trot with the standard CHAMP foot link names.
    pub fn default_trot() -> Self {
        Self {
            name: "default".into(),
            gait_type: misarta::config::GaitTypeConfig::Trot,
            cycle_period_s: 0.4,
            duty_factor: 0.5,
            swing_height_m: 0.04,
            max_step_length_m: 0.10,
            fl_foot: "FL_foot".into(),
            fr_foot: "FR_foot".into(),
            rl_foot: "RL_foot".into(),
            rr_foot: "RR_foot".into(),
            knee_forward: [false; 4],
            four_support_fraction: 0.5,
        }
    }
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
    /// Whether this link's collision geoms participate in contact detection.
    /// `false` is the moral equivalent of MuJoCo `contype=0 conaffinity=0`:
    /// the geoms are still rendered (as collision-group visuals) but the
    /// physics engine skips every contact pair involving them. Defaults to
    /// `true` so legacy models load with their original behaviour.
    pub collision_enabled: bool,
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
    /// Optional MuJoCo-specific contact physics attributes. Carried
    /// through MJCF import / export and `.misa` round-trip so that
    /// per-geom tuning (high-friction foot sphere etc.) survives
    /// articara's pipeline. `None` ⇒ inherit MuJoCo's compiler default
    /// (typically `friction = 0.6 0.005 0.0001`, `condim = 3`,
    /// `priority = 0`, `margin = 0`). All [`MjcfPhysics`] fields are
    /// themselves optional and emit only when set.
    #[cfg_attr(feature = "serde", serde(default, skip_serializing_if = "Option::is_none"))]
    pub physics: Option<MjcfPhysics>,
}

/// Per-collision-geom MuJoCo physics tuning. Maps 1:1 onto MJCF
/// `<geom>` attributes; absent fields fall back to MuJoCo's
/// `<default>`-inherited values at sim time.
#[derive(Clone, Default, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct MjcfPhysics {
    /// `friction = "tangential torsional rolling"`. MuJoCo default
    /// `[0.6, 0.005, 0.0001]`. Pure Coulomb only needs the first.
    #[cfg_attr(feature = "serde", serde(default, skip_serializing_if = "Option::is_none"))]
    pub friction: Option<[f64; 3]>,
    /// `condim`. 1 = frictionless, 3 = sliding only (Coulomb),
    /// 4 = sliding + rolling, 6 = full (sliding + torsional + rolling).
    /// Foot spheres typically want 6 so the leg can't free-roll under
    /// load.
    #[cfg_attr(feature = "serde", serde(default, skip_serializing_if = "Option::is_none"))]
    pub condim: Option<u32>,
    /// `priority`. Higher-priority geom wins when two geoms with
    /// different contact properties are involved in the same contact
    /// pair. Default `0`.
    #[cfg_attr(feature = "serde", serde(default, skip_serializing_if = "Option::is_none"))]
    pub priority: Option<u32>,
    /// `solimp = "d0 d_width width"`. Soft-contact impedance curve;
    /// see MuJoCo docs. `Some([0.015, 1.0, 0.022])` matches Go2's
    /// stock foot tuning.
    #[cfg_attr(feature = "serde", serde(default, skip_serializing_if = "Option::is_none"))]
    pub solimp: Option<[f64; 3]>,
    /// `margin` — distance at which contacts start to be generated
    /// (m). Default `0`.
    #[cfg_attr(feature = "serde", serde(default, skip_serializing_if = "Option::is_none"))]
    pub margin: Option<f64>,
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
        /// Loaded mesh geometry. The [`misarta::mesh::MeshData`] is the
        /// source of truth (indexed triangles, face normals, materials);
        /// flat `[x, y, z, nx, ny, nz]` arrays for GL upload etc. are
        /// derived on demand via `to_flat_vertices_f32()`. `Arc` makes
        /// model clones (undo snapshots!) share the mesh instead of
        /// deep-copying every vertex. Serialized as the legacy flat
        /// `vertices` array so the wire format is unchanged.
        #[cfg_attr(
            feature = "serde",
            serde(rename = "vertices", with = "mesh_as_flat_vertices")
        )]
        mesh: std::sync::Arc<misarta::mesh::MeshData>,
        filename: Option<String>,    // original URI e.g. "package://..."
        scale: Option<[f32; 3]>,
    },
}

/// Serialize the shared [`misarta::mesh::MeshData`] as the legacy flat
/// `[x, y, z, nx, ny, nz]` vertex array (and back).
#[cfg(feature = "serde")]
mod mesh_as_flat_vertices {
    use serde::{Deserialize, Deserializer, Serialize, Serializer};
    use std::sync::Arc;

    pub fn serialize<S: Serializer>(
        mesh: &Arc<misarta::mesh::MeshData>,
        s: S,
    ) -> Result<S::Ok, S::Error> {
        mesh.to_flat_vertices_f32().serialize(s)
    }

    pub fn deserialize<'de, D: Deserializer<'de>>(
        d: D,
    ) -> Result<Arc<misarta::mesh::MeshData>, D::Error> {
        let flat = Vec::<f32>::deserialize(d)?;
        Ok(Arc::new(misarta::mesh::MeshData::from_flat_vertices_f32(
            &flat,
        )))
    }
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
    /// Joint-side PD with trajectory-velocity feedforward:
    /// `τ = Kp·(q*−q) + Kv·(q̇*−q̇)`. Cheapest mode; relies on the user's
    /// trajectory + Kp/Kv to compensate gravity / inertia coupling.
    Position,
    /// Velocity tracker: `τ = Kv·(q̇*−q̇)`. Only Kv is meaningful.
    Velocity,
    /// Direct torque command — the controller is the user's own code via
    /// `set_torque_target`. No built-in feedback.
    Torque,
    /// Computed-torque (inverse-dynamics feedforward) law:
    /// `τ = M(q)·q̈* + h(q, q̇) + Kp·(q*−q) + Kv·(q̇*−q̇)`,
    /// where M is the joint-space inertia matrix (CRBA) and h is the
    /// nonlinear bias (gravity + Coriolis + centrifugal, RNEA at q̈=0).
    /// The PD only has to correct tracking error, so Kp / Kv can be much
    /// lower than in plain Position mode while delivering tighter tracking.
    ComputedTorque,
    /// MJCF-export-only "fixed" mode: the joint is emitted as a welded
    /// (no DoF) connection in the generated MJCF, and no actuator is
    /// produced. The underlying `joint_type` in `JointData` is preserved,
    /// so URDF / .misa export and articara's internal FK keep treating the
    /// joint as movable. Use to disable wheel / passive joints for a
    /// MuJoCo sim run without rewriting the URDF.
    Fixed,
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
            ActuatorMode::ComputedTorque => "Computed-τ",
            ActuatorMode::Fixed => "🔒 Fixed",
        }
    }
    pub const ALL: [ActuatorMode; 5] = [
        ActuatorMode::Position,
        ActuatorMode::Velocity,
        ActuatorMode::Torque,
        ActuatorMode::ComputedTorque,
        ActuatorMode::Fixed,
    ];

    /// Whether this mode causes MJCF export to treat the joint as
    /// permanently welded (no `<joint>` element, no actuator).
    pub fn is_fixed(self) -> bool {
        matches!(self, ActuatorMode::Fixed)
    }
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
    ///
    /// Default is `0.0014 kg·m²` — a small but realistic motor-rotor inertia
    /// that keeps the (kp, kv) defaults (50, 5) inside the stability envelope
    /// at MuJoCo's default 2 ms timestep. Models that explicitly want
    /// `0` (ideal massless rotor) must set the field directly.
    #[cfg_attr(feature = "serde", serde(default = "default_armature"))]
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
#[cfg(feature = "serde")]
fn default_armature() -> f64 { 0.0014 }


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
                GeomData::Mesh { mesh, .. } => {
                    let step = if mesh.vertices.len() > 1000 { 10 } else { 1 };
                    for v in mesh.vertices.iter().step_by(step) {
                        points.push(vis.origin * na::Point3::new(v.x as f32, v.y as f32, v.z as f32));
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
            GeomData::Mesh { mesh, .. } => {
                let step = if mesh.vertices.len() > 1000 { 10 } else { 1 };
                for v in mesh.vertices.iter().step_by(step) {
                    pts.push(na::Point3::new(v.x as f32, v.y as f32, v.z as f32));
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
        GeomData::Mesh { mesh, .. } => {
            compute_mesh_inertia(&mesh.to_flat_vertices_f32(), mass)
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
        GeomData::Mesh { mesh, .. } => {
            compute_mesh_volume(&mesh.to_flat_vertices_f32())
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

// The misarta integration (cache building, FK / Jacobian / IK solves,
// MisartaConfig + sidecar conversion) lives in `super::misarta_bridge`;
// re-export its public items so `crate::robot::*` keeps covering them.
pub use super::misarta_bridge::*;
