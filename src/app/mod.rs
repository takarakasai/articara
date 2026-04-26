use eframe::egui;
use nalgebra as na;
use std::collections::HashMap;
use std::path::PathBuf;
use std::sync::{Arc, Mutex};

use crate::camera::OrbitCamera;
use crate::dynamics;
use crate::format::RobotFormat;
use crate::history::History;
use crate::renderer::{DisplayMode, GlRenderer, MeshKind};
use crate::robot::RobotModel;

/// How a left-mouse drag on a link is interpreted while a MuJoCo sim is running.
#[cfg(feature = "mujoco")]
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum SimDragMode {
    /// Drag delta becomes a continuous world-frame force applied via
    /// `xfrc_applied`. Magnitude is `sim_drag_force_gain` × drag distance.
    Force,
    /// Drag delta steers a posture target: an IK solve from the link's
    /// kinematic root keeps the link at the dragged world position by
    /// updating the controller's `position_targets`. The PD loop then
    /// catches up to that target while obeying gains / limits.
    Posture,
}

#[cfg(feature = "mujoco")]
impl SimDragMode {
    pub const ALL: [SimDragMode; 2] = [SimDragMode::Force, SimDragMode::Posture];

    pub fn label(self) -> &'static str {
        match self {
            SimDragMode::Force => "Force (apply wrench)",
            SimDragMode::Posture => "Posture (IK target)",
        }
    }
}

/// State for an in-flight sim-time drag interaction.
#[cfg(feature = "mujoco")]
#[derive(Clone)]
pub(super) struct SimDragState {
    pub mode: SimDragMode,
    pub link_name: String,
    /// Local-frame offset on the dragged link where the click landed.
    /// Used so the dragged point follows the cursor, not just the link origin.
    pub ee_local_offset: na::Point3<f32>,
    /// Camera-forward depth of the click point, used to keep the drag plane
    /// stable under perspective projection.
    pub drag_depth: f32,
    /// Kinematic chain from root to the dragged link, used by Posture mode
    /// for the IK solve. Empty for Force mode.
    pub chain: Vec<usize>,
    /// IK root link (None = full chain from URDF root).
    pub ik_root_link: Option<String>,
}

/// Which signal the Joint Peaks plot window is rendering.
#[cfg(feature = "mujoco")]
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum PeaksPlotMetric {
    /// Joint angle / displacement q.
    Position,
    /// Joint velocity q̇.
    Velocity,
    /// Commanded torque / force τ.
    Torque,
}

#[cfg(feature = "mujoco")]
impl PeaksPlotMetric {
    pub const ALL: [PeaksPlotMetric; 3] = [
        PeaksPlotMetric::Position,
        PeaksPlotMetric::Velocity,
        PeaksPlotMetric::Torque,
    ];

    pub fn label(self) -> &'static str {
        match self {
            PeaksPlotMetric::Position => "q (Position)",
            PeaksPlotMetric::Velocity => "q̇ (Velocity)",
            PeaksPlotMetric::Torque => "τ (Torque)",
        }
    }

    pub fn unit(self, is_prismatic: bool) -> &'static str {
        match self {
            PeaksPlotMetric::Position => {
                if is_prismatic { "m" } else { "rad" }
            }
            PeaksPlotMetric::Velocity => {
                if is_prismatic { "m/s" } else { "rad/s" }
            }
            PeaksPlotMetric::Torque => {
                if is_prismatic { "N" } else { "N·m" }
            }
        }
    }
}

/// Pin constraint dimensionality.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PinDof {
    /// Position only (3-DoF).
    Position,
    /// Position + orientation (6-DoF).
    Pose,
}

impl PinDof {
    pub fn label(self) -> &'static str {
        match self {
            PinDof::Position => "Position (3D)",
            PinDof::Pose => "Pose (6D)",
        }
    }
    pub const ALL: [PinDof; 2] = [PinDof::Position, PinDof::Pose];
}

/// A link pinned to a world-space position (and optionally orientation)
/// for multi-constraint IK.
#[derive(Clone, Debug)]
pub struct PinnedLink {
    /// Link name.
    pub link_name: String,
    /// Target world position to maintain.
    pub target_pos: na::Point3<f64>,
    /// Target world orientation (for 6-DoF mode).
    pub target_rot: na::UnitQuaternion<f64>,
    /// Constraint dimensionality.
    pub dof: PinDof,
}

/// Top-level interaction mode.
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum InteractionMode {
    /// Drive joints by dragging links (single-joint or IK).
    JointDrive,
    /// Adjust link/joint origin offsets via gizmo arrows.
    OffsetAdjust,
}

/// Gizmo operation type in Offset Adjust mode.
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum GizmoOp {
    Translate,
    Rotate,
    Scale,
}

/// Which element's origin to adjust in Offset Adjust mode.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum OffsetTarget {
    Joint,
    Visual,
    Collision,
}

impl OffsetTarget {
    pub const ALL: [OffsetTarget; 3] = [
        OffsetTarget::Joint,
        OffsetTarget::Visual,
        OffsetTarget::Collision,
    ];

    pub fn label(self) -> &'static str {
        match self {
            OffsetTarget::Joint => "Joint",
            OffsetTarget::Visual => "Visual",
            OffsetTarget::Collision => "Collision",
        }
    }

    pub fn icon(self) -> &'static str {
        match self {
            OffsetTarget::Joint => "\u{1f517}",   // 🔗
            OffsetTarget::Visual => "\u{1f441}",  // 👁
            OffsetTarget::Collision => "\u{1f6e1}", // 🛡
        }
    }
}

/// Drag manipulation mode (within JointDrive).
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum DragMode {
    /// Move only the immediate parent joint of the clicked link.
    SingleJoint,
    /// Solve IK on the entire chain from root to the clicked link.
    InverseKinematics,
}

/// Drag state for manipulating joints by dragging links.
#[derive(Clone)]
struct DragState {
    /// Index of the link being dragged.
    link_idx: usize,
    /// Mode at drag start.
    mode: DragMode,
    // --- Single-joint mode fields ---
    /// Index of the parent joint to rotate (single-joint mode only).
    joint_idx: usize,
    /// The joint axis in world space at drag start.
    world_axis: na::Vector3<f32>,
    /// The joint pivot point in world space at drag start.
    pivot_world: na::Point3<f32>,
    // --- IK mode fields ---
    /// The kinematic chain (movable joint indices) from root to end-effector.
    chain: Vec<usize>,
    /// End-effector link name (for Jacobian computation).
    ee_link: String,
    /// The IK root link name (for base correction). None = URDF root.
    ik_root_link: Option<String>,
    /// World-space transform of the IK root link at drag start.
    /// Used as the fixed anchor so the root doesn't drift.
    ik_root_initial_tf: Option<na::Isometry3<f32>>,
    /// World-space position of the IK root link at drag start (for translation-only correction).
    #[allow(dead_code)]
    ik_root_initial_pos: Option<na::Point3<f64>>,
    /// Reference joint positions at drag start for null-space posture stabilization.
    #[allow(dead_code)]
    ref_positions: Vec<f64>,
    /// Signed depth of the EE along the camera forward axis at drag start.
    /// Used to keep the IK target plane stable during drag.
    drag_depth: f32,
    /// Local-frame offset of the IK target point on the end-effector link.
    /// Set to the surface point that was clicked (in link-local coordinates).
    ee_local_offset: na::Point3<f32>,
}

/// Drag state for gizmo offset adjustment.
#[derive(Clone)]
struct OffsetDragState {
    /// Which axis is being dragged (0=X, 1=Y, 2=Z).
    axis: u8,
    /// What we are adjusting.
    target: OffsetTarget,
    /// Index of the link (for Visual/Collision targets) or joint (for Joint target).
    entity_idx: usize,
    /// Sub-index within link's visuals/collisions array.
    sub_idx: usize,
    /// World-space direction of the dragged axis (unit vector).
    axis_dir_world: na::Vector3<f32>,
    /// World-space gizmo origin at drag start.
    gizmo_origin: na::Point3<f32>,
    /// The ray-axis parameter "t" at drag start (translate mode).
    initial_t: f32,
    /// The origin translation at drag start.
    initial_translation: na::Translation3<f32>,
    /// Inverse rotation for converting world displacement to local displacement.
    inv_parent_rotation: na::UnitQuaternion<f32>,
    /// Current gizmo operation.
    op: GizmoOp,
    /// The origin rotation at drag start (for rotation mode).
    initial_rotation: na::UnitQuaternion<f32>,
    /// Angle at drag start (for rotation mode).
    initial_angle: f32,
    /// Initial geometry parameters at drag start (for scale mode).
    /// Box: [hx, hy, hz], Cylinder: [radius, half_length, 0], Sphere: [radius, 0, 0]
    initial_geom_params: [f32; 3],
}

pub struct ArticaraApp {
    model: Option<RobotModel>,
    camera: OrbitCamera,
    gl: Arc<glow::Context>,
    gl_renderer: Arc<Mutex<GlRenderer>>,
    selected_link: Option<usize>,
    selected_joint: Option<usize>,
    needs_upload: bool,
    urdf_path_input: String,
    status_message: String,
    /// Current drag interaction (link manipulation).
    drag_state: Option<DragState>,
    /// Link currently hovered by mouse (for highlighting).
    hovered_link: Option<usize>,
    /// Cached viewport rect from last frame.
    viewport_rect: egui::Rect,
    /// Current drag mode.
    drag_mode: DragMode,
    /// Current interaction mode (JointDrive vs OffsetAdjust).
    interaction_mode: InteractionMode,
    /// Gizmo drag state for offset adjustment mode.
    offset_drag_state: Option<OffsetDragState>,
    /// Currently hovered gizmo axis (0=X, 1=Y, 2=Z).
    hovered_gizmo_axis: Option<u8>,
    /// Which element type to adjust in Offset Adjust mode.
    offset_target: OffsetTarget,
    /// Gizmo operation: Translate or Rotate.
    gizmo_op: GizmoOp,
    /// Selected visual index within the currently selected link.
    selected_visual: Option<usize>,
    /// Selected collision index within the currently selected link.
    selected_collision: Option<usize>,
    /// IK damping factor (λ for DLS / λ_max for SR-Inverse).
    ik_damping: f32,
    /// IK solver method.
    ik_solver: crate::robot::IkSolver,
    /// IK constraint dimensionality (2D screen-plane or 3D world).
    ik_dof: crate::robot::IkDof,
    /// IK joint weight gradient: 0 = uniform, larger = prefer EE-proximal joints.
    ik_weight_gradient: f32,
    /// IK root link name. None = use URDF root (full chain).
    ik_root_link: Option<String>,
    /// IK target position in world space (for debug overlay). None = no active IK.
    ik_target_marker: Option<na::Point3<f32>>,
    /// Current end-effector position in world space (for debug overlay).
    ik_ee_marker: Option<na::Point3<f32>>,
    /// IK residual error (distance between EE and target) for debug overlay.
    ik_error: Option<f32>,
    /// Links pinned to their world positions for multi-constraint IK.
    pinned_links: Vec<PinnedLink>,
    /// Weight for pin constraints (higher = harder constraint).
    ik_pin_weight: f32,
    /// Weight for loop-closure constraints (higher = harder).
    loop_closure_weight: f32,
    /// Links to auto-pin at IK drag start (chicken-head stabilization).
    chicken_head_links: Vec<String>,
    /// Default DoF mode for chicken-head pins.
    chicken_head_dof: PinDof,
    /// Show center-of-mass markers and mass labels.
    show_com: bool,
    /// Show joint axis arrows in viewport.
    show_joint_axes: bool,
    /// Show a semi-transparent ground plane in the viewport.
    show_ground_plane: bool,
    /// Z height of the ground plane.
    ground_z: f32,
    /// Half-extent size of the ground plane.
    ground_size: f32,
    /// Ground plane rotation about X axis (rad).
    ground_plane_roll: f32,
    /// Ground plane rotation about Y axis (rad).
    ground_plane_pitch: f32,
    /// Whether the ground plane was auto-enabled by a running simulation.
    ground_plane_auto: bool,
    /// Show gravity/bias direction arrow in viewport.
    show_gravity_arrow: bool,
    /// Gravity (bias) direction vector (unit). Default: [0, 0, -1].
    gravity_dir: [f32; 3],
    /// Scale factor for CoM sphere size (sphere radius = mass × com_scale).
    com_scale: f32,
    /// Show robot links in wireframe mode (legacy, kept for compat).
    wireframe: bool,
    /// Global visual display mode.
    visual_mode: DisplayMode,
    /// Global collision display mode.
    collision_mode: DisplayMode,
    /// Per-link display mode overrides. Key=(link_name, MeshKind).
    link_display_modes: HashMap<(String, MeshKind), DisplayMode>,
    /// Export output directory path.
    export_dir: String,
    /// Selected format for export.
    export_format: RobotFormat,
    /// Status message from last export attempt.
    export_message: String,
    /// Whether to show the export dialog window.
    show_export_dialog: bool,
    // --- Add link/joint dialog state ---
    /// Whether the "Add Child" section is open.
    show_add_child: bool,
    /// New link name input.
    new_link_name: String,
    /// New joint name input.
    new_joint_name: String,
    /// Selected parent link name for the new child.
    new_parent_link: String,
    /// New joint type (revolute, prismatic, fixed, continuous).
    new_joint_type_idx: usize,
    /// New geometry type (box, cylinder, sphere).
    new_geom_type_idx: usize,
    /// Geometry size parameters [x, y, z] (reused for different shapes).
    new_geom_size: [f32; 3],
    /// New joint origin XYZ.
    new_joint_origin: [f32; 3],
    /// New joint axis.
    new_joint_axis: [f32; 3],
    /// New link color (RGBA).
    new_link_color: [f32; 3],
    /// Joint limits [lower, upper].
    new_joint_limits: [f32; 2],
    /// When set, these ancestor link names should be auto-expanded in the tree.
    tree_reveal_ancestors: Vec<String>,
    /// Undo/redo history.
    history: History,
    /// Model snapshot taken at the start of each frame (before any edits).
    pre_frame_snapshot: Option<RobotModel>,
    /// Whether any model edit occurred this frame.
    any_edit_this_frame: bool,
    /// Show density input dialog for mass-from-density calculation.
    show_density_input: bool,
    /// Density value (kg/m³) for mass-from-density calculation.
    density_value: f64,
    /// Show the inertia validation results window.
    show_validation_window: bool,
    /// Cached inertia validation results.
    validation_results: Vec<crate::robot::InertiaValidation>,
    // --- Dynamics analysis state ---
    /// Selected end-effector link for payload capacity analysis.
    dynamics_ee_link: Option<String>,
    /// Cached dynamics analysis result.
    dynamics_result: Option<dynamics::StaticAnalysis>,
    /// Active dynamics simulation (payload).
    dynamics_sim: Option<dynamics::DynSim>,
    /// Simulation playback speed.
    dynamics_sim_speed: f32,
    /// Whether the simulation is paused.
    dynamics_sim_paused: bool,
    /// When `Some(n)`, advance the active MuJoCo sim by exactly `n` physics
    /// frames (negative = step backward through the snapshot history) then
    /// re-pause. Ignored when the active sim is not MuJoCo.
    dynamics_step_frames: Option<i32>,
    /// Last frame instant for delta-time calculation.
    dynamics_last_instant: Option<std::time::Instant>,
    /// Active MuJoCo simulation instance.
    #[cfg(feature = "mujoco")]
    mujoco_sim: Option<crate::mujoco_sim::MujocoSim>,
    /// When true, the MuJoCo sim auto-lifts the floating base just above z=0.
    /// When false, [`Self::mujoco_base_pos`] is used as the initial world position.
    #[cfg(feature = "mujoco")]
    mujoco_auto_base: bool,
    /// Manual initial world position for the floating base (used when
    /// [`Self::mujoco_auto_base`] is false).
    #[cfg(feature = "mujoco")]
    mujoco_base_pos: [f32; 3],
    /// Per-axis lock state for the trunk before MuJoCo sim start, ordered
    /// `[TX, TY, TZ, RX, RY, RZ]`. `true` = locked. All `false` = full
    /// floating base (default), all `true` = welded to world.
    #[cfg(feature = "mujoco")]
    mujoco_base_locked: [bool; 6],
    /// Name buffer for the next "Save current pose as…" entry.
    pose_save_name: String,
    /// Default interpolation kind seeded into newly-saved poses.
    pose_transition_kind: misarta::trajectory::InterpolationKind,
    /// Default transition duration (s) seeded into newly-saved poses.
    pose_transition_duration: f32,
    /// Currently selected target link for the external-force panel.
    ext_force_link: Option<String>,
    /// Force vector (N) for the external-force panel.
    ext_force_value: [f32; 3],
    /// Torque vector (N·m) for the external-force panel.
    ext_torque_value: [f32; 3],
    /// Duration (s) of the next external-force application.
    ext_force_duration: f32,
    /// Whether contact-point markers + force vectors are drawn over the viewport.
    #[cfg(feature = "mujoco")]
    show_contacts: bool,
    /// How a sim-time link drag is interpreted (force vs posture).
    #[cfg(feature = "mujoco")]
    sim_drag_mode: SimDragMode,
    /// Active sim-drag state while the user is holding the mouse button.
    #[cfg(feature = "mujoco")]
    sim_drag_state: Option<SimDragState>,
    /// Force gain (N per metre of drag) for Force mode. Tuned so a typical
    /// 30 cm drag exerts ~150 N out of the box, enough to push a kg-scale
    /// link around without flinging lighter ones.
    #[cfg(feature = "mujoco")]
    sim_drag_force_gain: f32,
    /// Whether to enforce per-joint torque/velocity limits during MuJoCo
    /// simulation. When `true`, the controller torque is clamped to ±τmax
    /// and the velocity-mode reference / commanded torque are gated by ωmax.
    #[cfg(feature = "mujoco")]
    enforce_actuator_limits: bool,
    /// Whether the Joint Peaks time-series plot window is open.
    #[cfg(feature = "mujoco")]
    show_peaks_plot: bool,
    /// Joint selected for the Joint Peaks plot. `None` = plot all movable joints.
    #[cfg(feature = "mujoco")]
    peaks_plot_joint: Option<String>,
    /// Which signal to display on the Joint Peaks plot.
    #[cfg(feature = "mujoco")]
    peaks_plot_metric: PeaksPlotMetric,
    /// File path for sim config save/load.
    sim_config_path: String,
    // --- Posture save/load ---
    /// File path for posture save/load (.toml).
    posture_path: String,
    // --- File dialogs ---
    /// Dialog for loading a robot model file.
    dlg_open_model: file_dialog::FileDialog,
    /// Dialog for loading a posture file.
    dlg_open_posture: file_dialog::FileDialog,
    /// Dialog for saving a posture file.
    dlg_save_posture: file_dialog::FileDialog,
    /// Dialog for choosing the export directory.
    dlg_export_dir: file_dialog::FileDialog,
    /// Dialog for loading a sim config file.
    dlg_open_sim_config: file_dialog::FileDialog,
    /// Dialog for saving a sim config file.
    dlg_save_sim_config: file_dialog::FileDialog,
    /// Dialog for loading a mesh file (STL/DAE) to add as visual or collision.
    dlg_add_mesh: file_dialog::FileDialog,
    /// Target for the mesh file dialog: which link index and whether visual or collision.
    add_mesh_target: Option<AddMeshTarget>,
    // --- Script console ---
    /// Whether the script console window is visible.
    show_script_console: bool,
    /// Rhai script engine for model manipulation.
    #[cfg(feature = "scripting")]
    script_engine: Option<crate::scripting_model::ModelScriptEngine>,
    /// Current script input text.
    script_input: String,
    /// Captured script output lines (with type tags for colouring).
    script_output: Vec<ScriptLine>,
    /// Input history (up/down arrow navigation).
    script_history: Vec<String>,
    /// Current position in input history (0 = newest).
    script_history_idx: usize,
    /// Whether to auto-scroll to bottom next frame.
    script_scroll_to_bottom: bool,
    /// Pending tab-completion candidates (shown after Tab press).
    script_tab_candidates: Vec<String>,
    /// Selected mesh decimation algorithm.
    decimation_method: misarta::decimate::DecimationMethod,
    /// Selected mesh decomposition method (V-HACD / Sphere Tree).
    decomposition_method: misarta::decompose::DecompositionMethod,
    /// Background decomposition task (V-HACD is slow, so we run it off-thread).
    decompose_task: Option<DecomposeTask>,
    /// Edit buffer for renaming the currently selected link.
    rename_link_buf: String,
    /// Edit buffer for renaming the currently selected joint.
    rename_joint_buf: String,
}

/// Whether a decompose task targets a visual or collision slot.
#[derive(Clone, Copy, PartialEq, Eq)]
enum DecomposeTarget {
    Visual,
    Collision,
}

/// The result produced by a background decomposition thread.
enum DecomposeResult {
    Visuals(Vec<crate::robot::VisualData>),
    Collisions(Vec<crate::robot::CollisionData>),
}

/// A running background decomposition task.
struct DecomposeTask {
    /// Link index where the result should be applied.
    link_index: usize,
    /// Slot index (visual or collision) to replace.
    slot_index: usize,
    /// Whether this targets a visual or collision.
    target: DecomposeTarget,
    /// Decomposition method used (for status messages).
    method: misarta::decompose::DecompositionMethod,
    /// Atomic progress phase (polled by UI thread).
    progress: std::sync::Arc<std::sync::atomic::AtomicU8>,
    /// Fine-grained 0–100 sub-progress within the current phase.
    sub_progress: std::sync::Arc<std::sync::atomic::AtomicU8>,
    /// Join handle for the background thread.
    handle: Option<std::thread::JoinHandle<DecomposeResult>>,
    /// Instant the task was started (for elapsed time display).
    started: std::time::Instant,
}

/// A tagged line in the script console output.
#[derive(Clone)]
enum ScriptLine {
    /// User input (echoed with prompt).
    Input(String),
    /// Normal output from `print()`.
    Output(String),
    /// Error message.
    Error(String),
    /// System/info message (help, greeting).
    System(String),
}

/// Tracks which link and slot (visual / collision) a pending mesh-add dialog is for.
#[derive(Clone)]
struct AddMeshTarget {
    link_index: usize,
    kind: MeshAddKind,
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum MeshAddKind {
    Visual,
    Collision,
}

impl ArticaraApp {
    pub fn new(cc: &eframe::CreationContext<'_>) -> Self {
        let gl = cc
            .gl
            .as_ref()
            .expect("glow context required — use Renderer::Glow")
            .clone();
        let renderer = GlRenderer::new(&gl);

        Self {
            model: None,
            camera: OrbitCamera::new(),
            gl,
            gl_renderer: Arc::new(Mutex::new(renderer)),
            selected_link: None,
            selected_joint: None,
            needs_upload: false,
            urdf_path_input: String::new(),
            status_message: "No model loaded".into(),
            drag_state: None,
            hovered_link: None,
            viewport_rect: egui::Rect::NOTHING,
            drag_mode: DragMode::SingleJoint,
            interaction_mode: InteractionMode::JointDrive,
            offset_drag_state: None,
            hovered_gizmo_axis: None,
            offset_target: OffsetTarget::Joint,
            gizmo_op: GizmoOp::Translate,
            selected_visual: None,
            selected_collision: None,
            ik_damping: 0.05,
            ik_solver: crate::robot::IkSolver::SrInverse,
            ik_dof: crate::robot::IkDof::ScreenPlane2D,
            ik_weight_gradient: 1.5,
            ik_root_link: None,
            ik_target_marker: None,
            ik_ee_marker: None,
            ik_error: None,
            pinned_links: Vec::new(),
            ik_pin_weight: 10.0,
            loop_closure_weight: 50.0,
            chicken_head_links: Vec::new(),
            chicken_head_dof: PinDof::Position,
            show_com: false,
            show_joint_axes: false,
            show_ground_plane: false,
            ground_z: 0.0,
            ground_plane_roll: 0.0,
            ground_plane_pitch: 0.0,
            ground_plane_auto: false,
            show_gravity_arrow: true,
            gravity_dir: [0.0, 0.0, -1.0],
            ground_size: 2.0,
            com_scale: 0.01,
            wireframe: false,
            visual_mode: DisplayMode::Solid,
            collision_mode: DisplayMode::Off,
            link_display_modes: HashMap::new(),
            export_dir: String::new(),
            export_format: RobotFormat::Urdf,
            export_message: String::new(),
            show_export_dialog: false,
            // Add child dialog defaults
            show_add_child: false,
            new_link_name: String::new(),
            new_joint_name: String::new(),
            new_parent_link: String::new(),
            new_joint_type_idx: 0,
            new_geom_type_idx: 0,
            new_geom_size: [0.05, 0.05, 0.05],
            new_joint_origin: [0.0, 0.0, 0.1],
            new_joint_axis: [0.0, 0.0, 1.0],
            new_link_color: [0.5, 0.7, 1.0],
            new_joint_limits: [-1.57, 1.57],
            tree_reveal_ancestors: Vec::new(),
            history: History::new(50),
            pre_frame_snapshot: None,
            any_edit_this_frame: false,
            show_density_input: false,
            density_value: 1000.0, // default: water (1000 kg/m³)
            show_validation_window: false,
            validation_results: Vec::new(),
            dynamics_ee_link: None,
            dynamics_result: None,
            dynamics_sim: None,
            dynamics_sim_speed: 1.0,
            dynamics_sim_paused: false,
            dynamics_step_frames: None,
            dynamics_last_instant: None,
            #[cfg(feature = "mujoco")]
            mujoco_sim: None,
            #[cfg(feature = "mujoco")]
            mujoco_auto_base: true,
            #[cfg(feature = "mujoco")]
            mujoco_base_pos: [0.0, 0.0, 0.0],
            #[cfg(feature = "mujoco")]
            mujoco_base_locked: [false; 6],
            pose_save_name: String::new(),
            pose_transition_kind: misarta::trajectory::InterpolationKind::QuinticSmooth,
            pose_transition_duration: 1.0,
            ext_force_link: None,
            ext_force_value: [0.0, 0.0, 0.0],
            ext_torque_value: [0.0, 0.0, 0.0],
            ext_force_duration: 0.5,
            #[cfg(feature = "mujoco")]
            show_contacts: true,
            #[cfg(feature = "mujoco")]
            sim_drag_mode: SimDragMode::Force,
            #[cfg(feature = "mujoco")]
            sim_drag_state: None,
            #[cfg(feature = "mujoco")]
            sim_drag_force_gain: 500.0,
            #[cfg(feature = "mujoco")]
            enforce_actuator_limits: false,
            #[cfg(feature = "mujoco")]
            show_peaks_plot: false,
            #[cfg(feature = "mujoco")]
            peaks_plot_joint: None,
            #[cfg(feature = "mujoco")]
            peaks_plot_metric: PeaksPlotMetric::Torque,
            sim_config_path: String::new(),
            posture_path: String::new(),
            dlg_open_model: file_dialog::FileDialog::new("dlg_open_model"),
            dlg_open_posture: file_dialog::FileDialog::new("dlg_open_posture"),
            dlg_save_posture: file_dialog::FileDialog::new("dlg_save_posture"),
            dlg_export_dir: file_dialog::FileDialog::new("dlg_export_dir"),
            dlg_open_sim_config: file_dialog::FileDialog::new("dlg_open_sim_config"),
            dlg_save_sim_config: file_dialog::FileDialog::new("dlg_save_sim_config"),
            dlg_add_mesh: file_dialog::FileDialog::new("dlg_add_mesh"),
            add_mesh_target: None,
            show_script_console: false,
            #[cfg(feature = "scripting")]
            script_engine: None,
            script_input: String::new(),
            script_output: Vec::new(),
            script_history: Vec::new(),
            script_history_idx: 0,
            script_scroll_to_bottom: false,
            script_tab_candidates: Vec::new(),
            decimation_method: misarta::decimate::DecimationMethod::Qem,
            decomposition_method: misarta::decompose::DecompositionMethod::Vhacd,
            decompose_task: None,
            rename_link_buf: String::new(),
            rename_joint_buf: String::new(),
        }
    }

    pub fn load_model(&mut self, path: PathBuf) {
        match RobotModel::from_file(&path) {
            Ok(model) => {
                self.status_message = format!(
                    "Loaded: {} ({} links, {} joints)",
                    model.name,
                    model.links.len(),
                    model.joints.len()
                );
                self.model = Some(model);
                self.urdf_path_input = path.display().to_string();
                // Load .misarta.toml sidecar if present
                if let Some(ref mut m) = self.model {
                    m.load_sidecar_config();
                }
                // Auto-set export format to match source
                if let Some(fmt) = RobotFormat::detect(&path) {
                    self.export_format = fmt;
                }
                self.selected_link = None;
                self.selected_joint = None;
                self.needs_upload = true;
                self.ik_root_link = None; // reset IK root on new model
                self.history.clear();
                // Default posture path: <model_dir>/<robot_name>.toml
                if let Some(parent) = path.parent() {
                    if let Some(ref m) = self.model {
                        self.posture_path =
                            parent.join(format!("{}.toml", m.name)).display().to_string();
                    }
                }
                // Default export dir to the URDF's parent directory
                if self.export_dir.is_empty() {
                    if let Some(parent) = path.parent() {
                        self.export_dir = parent.display().to_string();
                    }
                }
            }
            Err(e) => {
                self.status_message = format!("Error: {e}");
                log::error!("Failed to load model: {e}");
            }
        }
    }

    /// Record that an edit is about to happen (or is happening).
    /// On the first call per continuous editing phase, the pre-frame model
    /// snapshot is committed to the undo stack.  Subsequent calls with the
    /// same description are merged (no duplicate entries).
    fn mark_edit(&mut self, desc: &str) {
        if !self.any_edit_this_frame {
            if let Some(snapshot) = self.pre_frame_snapshot.take() {
                self.history.record(desc, snapshot);
            }
        }
        self.any_edit_this_frame = true;
    }

    /// Advance the dynamics simulation by one frame, modifying model state.
    fn step_dynamics_sim(&mut self) {
        #[cfg(feature = "mujoco")]
        let has_mujoco = self.mujoco_sim.is_some();
        #[cfg(not(feature = "mujoco"))]
        let has_mujoco = false;

        let sim = match self.dynamics_sim.as_mut() {
            Some(s) => s,
            None if !has_mujoco => {
                self.dynamics_last_instant = None;
                return;
            }
            None => {
                // mujoco_sim is active but dynamics_sim is not — fall through to MuJoCo step
                // Use a dummy reference; the MuJoCo branch returns before using `sim`.
                let now = std::time::Instant::now();
                #[cfg(feature = "mujoco")]
                {
                    let frame_request = self.dynamics_step_frames.take();
                    let enforce_limits = self.enforce_actuator_limits;
                    if let Some(mj_sim) = self.mujoco_sim.as_mut() {
                        if let Some(ref mut model) = self.model {
                            if let Some(n) = frame_request {
                                self.dynamics_sim_paused = true;
                                self.dynamics_last_instant = Some(now);
                                if n > 0 {
                                    mj_sim.step_n_frames(model, n as u32, enforce_limits);
                                } else if n < 0 {
                                    mj_sim.step_back_frames(model, (-n) as u32);
                                }
                                return;
                            }
                            if self.dynamics_sim_paused {
                                self.dynamics_last_instant = Some(now);
                                return;
                            }
                            let dt = match self.dynamics_last_instant {
                                Some(prev) => now.duration_since(prev).as_secs_f32().min(0.05),
                                None => 0.016,
                            };
                            self.dynamics_last_instant = Some(now);
                            mj_sim.step(model, dt as f64, enforce_limits);
                        }
                    }
                }
                return;
            }
        };

        // Handle pause / step-once (payload sim path; frame stepping is MuJoCo-only)
        if self.dynamics_sim_paused {
            self.dynamics_last_instant = Some(std::time::Instant::now());
            return;
        }

        // Compute delta-time
        let now = std::time::Instant::now();
        let dt = {
            let d = match self.dynamics_last_instant {
                Some(prev) => now.duration_since(prev).as_secs_f32().min(0.05),
                None => 0.016,
            };
            self.dynamics_last_instant = Some(now);
            d
        };

        let still_running = match sim {
            dynamics::DynSim::Payload(ps) => {
                ps.phase_time += dt as f64;
                if let Some(ref model) = self.model {
                    let ee = self.dynamics_ee_link.as_deref().unwrap_or("");
                    dynamics::step_payload_sim(ps, model, ee)
                } else {
                    false
                }
            }
        };

        if !still_running {
            // Auto-disable ground plane if we enabled it
            if self.ground_plane_auto {
                self.show_ground_plane = false;
                self.ground_plane_auto = false;
            }
            self.dynamics_sim = None;
            self.dynamics_last_instant = None;
        }
    }

    /// Draw and process all file dialog windows, handling their results.
    fn process_file_dialogs(&mut self, ctx: &egui::Context) {
        use file_dialog::FileDialogResult;

        // --- Open Model dialog ---
        match self.dlg_open_model.show(ctx) {
            FileDialogResult::Confirmed(path) => {
                self.urdf_path_input = path.display().to_string();
                self.load_model(path);
            }
            _ => {}
        }

        // --- Open Posture dialog ---
        match self.dlg_open_posture.show(ctx) {
            FileDialogResult::Confirmed(path) => {
                self.posture_path = path.display().to_string();
                if let Some(ref mut model) = self.model {
                    match posture::load_posture(model, &path) {
                        Ok(n) => {
                            self.needs_upload = true;
                            self.status_message = format!(
                                "Loaded posture ({n} joints matched) ← {}",
                                path.display()
                            );
                        }
                        Err(e) => {
                            self.status_message = format!("Load error: {e}");
                        }
                    }
                }
            }
            _ => {}
        }

        // --- Save Posture dialog ---
        match self.dlg_save_posture.show(ctx) {
            FileDialogResult::Confirmed(path) => {
                self.posture_path = path.display().to_string();
                if let Some(ref model) = self.model {
                    match posture::save_posture(model, &path) {
                        Ok(()) => {
                            self.status_message =
                                format!("Saved posture → {}", path.display());
                        }
                        Err(e) => {
                            self.status_message = format!("Save error: {e}");
                        }
                    }
                }
            }
            _ => {}
        }

        // --- Export Directory dialog ---
        match self.dlg_export_dir.show(ctx) {
            FileDialogResult::Confirmed(path) => {
                self.export_dir = path.display().to_string();
            }
            _ => {}
        }

        // --- Export dialog window ---
        self.draw_export_dialog(ctx);

        // --- Open Sim Config dialog ---
        match self.dlg_open_sim_config.show(ctx) {
            FileDialogResult::Confirmed(path) => {
                self.sim_config_path = path.display().to_string();
                match dynamics_panel::load_sim_config(&path) {
                    Ok(cfg) => {
                        dynamics_panel::apply_sim_config(self, cfg);
                        self.status_message =
                            format!("Loaded sim config ← {}", path.display());
                    }
                    Err(e) => {
                        self.status_message = format!("Load sim config error: {e}");
                    }
                }
            }
            _ => {}
        }

        // --- Save Sim Config dialog ---
        match self.dlg_save_sim_config.show(ctx) {
            FileDialogResult::Confirmed(path) => {
                self.sim_config_path = path.display().to_string();
                match dynamics_panel::save_sim_config(self, &path) {
                    Ok(()) => {
                        self.status_message =
                            format!("Saved sim config → {}", path.display());
                    }
                    Err(e) => {
                        self.status_message = format!("Save sim config error: {e}");
                    }
                }
            }
            _ => {}
        }

        // --- Add Mesh (STL/DAE) dialog ---
        match self.dlg_add_mesh.show(ctx) {
            FileDialogResult::Confirmed(path) => {
                let vertices = crate::robot::load_mesh_file(&path);
                if vertices.is_empty() {
                    self.status_message = format!(
                        "メッシュ読み込み失敗: {}",
                        path.display()
                    );
                } else if let Some(target) = self.add_mesh_target.take() {
                    let tri_count = vertices.len() / 18;
                    let fname = path.display().to_string();
                    if let Some(ref mut model) = self.model {
                        if target.link_index < model.links.len() {
                            let link = &mut model.links[target.link_index];
                            let geom = crate::robot::GeomData::Mesh {
                                vertices,
                                filename: Some(fname.clone()),
                                scale: None,
                            };
                            match target.kind {
                                MeshAddKind::Visual => {
                                    link.visuals.push(crate::robot::VisualData {
                                        origin: nalgebra::Isometry3::identity(),
                                        geometry: geom,
                                        color: [0.7, 0.7, 0.7, 1.0],
                                    });
                                    self.status_message = format!(
                                        "Visual メッシュ追加 ({tri_count} tris) ← {fname}"
                                    );
                                }
                                MeshAddKind::Collision => {
                                    link.collisions.push(crate::robot::CollisionData {
                                        origin: nalgebra::Isometry3::identity(),
                                        geometry: geom,
                                    });
                                    self.status_message = format!(
                                        "Collision メッシュ追加 ({tri_count} tris) ← {fname}"
                                    );
                                }
                            }
                            self.needs_upload = true;
                            self.any_edit_this_frame = true;
                        }
                    }
                }
            }
            FileDialogResult::Cancelled => {
                self.add_mesh_target = None;
            }
            _ => {}
        }
    }

}

// ===== UI sub-modules =====
mod menu_bar;
mod title_bar;
mod tree_panel;
mod joint_sliders;
mod properties_panel;
mod validation;
mod history_panel;
mod viewport;
mod viewport_overlay;
mod dynamics_panel;
mod posture;
mod file_dialog;
mod status_bar;
mod script_console;
#[cfg(feature = "mujoco")]
mod peaks_plot_window;
#[cfg(feature = "mujoco")]
mod sim_drag;

// Sentinel to mark the end of module-level code.
// Everything below was moved to sub-modules.
// Only the eframe::App impl remains here.

// ========== eframe::App ==========

impl eframe::App for ArticaraApp {
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        let ctx = ui.ctx().clone();

        // Custom resize handles at window edges (since native decorations are off)
        self.draw_resize_borders(&ctx);

        // --- Undo/Redo history: snapshot model at frame start ---
        self.any_edit_this_frame = false;
        self.pre_frame_snapshot = self.model.as_ref().map(|m| m.clone());

        // --- Undo/Redo keyboard shortcuts (Ctrl+Z / Ctrl+Shift+Z / Ctrl+Y) ---
        let undo_pressed = ctx.input(|i| {
            i.key_pressed(egui::Key::Z) && i.modifiers.command && !i.modifiers.shift
        });
        let redo_pressed = ctx.input(|i| {
            (i.key_pressed(egui::Key::Z) && i.modifiers.command && i.modifiers.shift)
                || (i.key_pressed(egui::Key::Y) && i.modifiers.command)
        });
        if undo_pressed {
            if let Some(ref mut model) = self.model {
                if let Some(desc) = self.history.undo(model) {
                    self.status_message = format!("↩ Undo: {desc}");
                    self.needs_upload = true;
                }
            }
        }
        if redo_pressed {
            if let Some(ref mut model) = self.model {
                if let Some(desc) = self.history.redo(model) {
                    self.status_message = format!("↪ Redo: {desc}");
                    self.needs_upload = true;
                }
            }
        }

        // Update transforms every frame (joint positions may have changed)
        if let Some(ref model) = self.model {
            let transforms = model.compute_transforms();
            let mut r = self.gl_renderer.lock().unwrap();
            r.update_transforms(transforms);
            r.show_com = self.show_com;
            r.show_joint_axes = self.show_joint_axes;
            r.show_ground_plane = self.show_ground_plane;
            r.ground_z = self.ground_z;
            r.ground_size = self.ground_size;
            r.ground_plane_roll = self.ground_plane_roll;
            r.ground_plane_pitch = self.ground_plane_pitch;
            r.show_gravity_arrow = self.show_gravity_arrow;
            r.gravity_dir = self.gravity_dir;
            r.com_scale = self.com_scale;
            r.wireframe = self.wireframe;
            r.visual_mode = self.visual_mode;
            r.collision_mode = self.collision_mode;
            r.link_display_modes = self.link_display_modes.clone();
        }

        // --- Step dynamics simulation (if active) ---
        self.step_dynamics_sim();

        // Custom title bar (replaces OS window decorations)
        egui::Panel::top("title_bar")
            .size_range(28.0..=28.0)
            .show_inside(ui, |ui| {
                self.draw_title_bar(ui, &ctx);
            });

        // Top panel: menu / file selector
        egui::Panel::top("menu_bar").show_inside(ui, |ui| {
            self.draw_menu_bar(ui);
        });

        // Bottom status bar (before left/right panels so it spans full width)
        egui::Panel::bottom("status_bar")
            .size_range(20.0..=20.0)
            .show_inside(ui, |ui| {
                self.draw_status_bar(ui);
            });

        // --- Script console (docked bottom panel, above status bar) ---
        self.draw_script_console(ui);

        // Left panel: tree + joint sliders
        egui::Panel::left("tree_panel")
            .default_size(260.0)
            .resizable(true)
            .show_inside(ui, |ui| {
                egui::ScrollArea::vertical().show(ui, |ui| {
                    self.draw_tree_panel(ui);
                    ui.separator();
                    self.draw_joint_sliders(ui);
                    ui.separator();
                    self.draw_dynamics_panel(ui);
                    ui.separator();
                    self.draw_history_panel(ui);
                });
            });

        // Right panel: properties
        egui::Panel::right("properties_panel")
            .default_size(280.0)
            .resizable(true)
            .show_inside(ui, |ui| {
                egui::ScrollArea::vertical().show(ui, |ui| {
                    self.draw_properties_panel(ui);
                });
            });

        // Central viewport (use remaining space)
        self.draw_viewport(ui);

        // --- Validation results window ---
        self.draw_validation_window(&ctx);

        // --- Sim result dialog ---
        self.draw_sim_result_window(&ctx);

        // --- Dynamics graph window ---
        self.draw_dynamics_graph_window(&ctx);

        // --- Joint Peaks time-series plot window ---
        #[cfg(feature = "mujoco")]
        self.draw_peaks_plot_window(&ctx);

        // --- File dialogs ---
        self.process_file_dialogs(&ctx);

        // --- Poll background decomposition task ---
        self.poll_decompose_task();

        // --- Draw decomposition progress overlay ---
        self.draw_decompose_progress(&ctx);

        // Upload robot geometry to GPU when needed.
        if self.needs_upload {
            if let Some(ref model) = self.model {
                self.gl_renderer
                    .lock()
                    .unwrap()
                    .upload_robot(&self.gl, model);
                self.needs_upload = false;
            }
        }

        // Request continuous repaint for smooth camera interaction
        ctx.request_repaint();

        // --- End-of-frame: finalize history if no edits occurred ---
        if self.drag_state.is_some() || self.offset_drag_state.is_some() {
            self.any_edit_this_frame = true;
        }
        if !self.any_edit_this_frame {
            self.history.finalize();
        }
        self.pre_frame_snapshot = None;
    }

    fn on_exit(&mut self, gl: Option<&glow::Context>) {
        if let Some(gl) = gl {
            self.gl_renderer.lock().unwrap().destroy(gl);
        }
    }
}

// ── Background decomposition task ───────────────────────────────────────────

impl ArticaraApp {
    /// Poll the background decompose task; apply results when done.
    fn poll_decompose_task(&mut self) {
        let task = match self.decompose_task.as_mut() {
            Some(t) => t,
            None => return,
        };

        // Check if the thread has finished.
        let is_finished = task
            .handle
            .as_ref()
            .map_or(true, |h| h.is_finished());

        if !is_finished {
            return;
        }

        // Thread is done — join and apply results.
        let handle = task.handle.take().unwrap();
        let result = match handle.join() {
            Ok(r) => r,
            Err(_) => {
                self.decompose_task = None;
                self.status_message = "Decomposition thread panicked".into();
                return;
            }
        };
        let li = task.link_index;
        let si = task.slot_index;
        let target = task.target;
        let method_label = task.method.label().to_string();
        let elapsed = task.started.elapsed();

        // Remove the task.
        self.decompose_task = None;

        // Apply to model.
        if let Some(ref mut model) = self.model {
            if li >= model.links.len() {
                return;
            }
            let link_name = model.links[li].name.clone();
            let kind_str = match target {
                DecomposeTarget::Visual => "visual",
                DecomposeTarget::Collision => "collision",
            };

            let n = match (target, result) {
                (DecomposeTarget::Collision, DecomposeResult::Collisions(new_cols)) => {
                    let n = new_cols.len();
                    if n == 0 { self.status_message = format!("Decomposition produced 0 shapes ({method_label})"); return; }
                    if si >= model.links[li].collisions.len() { return; }
                    model.links[li].collisions.remove(si);
                    for (i, c) in new_cols.into_iter().enumerate() {
                        model.links[li].collisions.insert(si + i, c);
                    }
                    n
                }
                (DecomposeTarget::Visual, DecomposeResult::Visuals(new_vis)) => {
                    let n = new_vis.len();
                    if n == 0 { self.status_message = format!("Decomposition produced 0 shapes ({method_label})"); return; }
                    if si >= model.links[li].visuals.len() { return; }
                    model.links[li].visuals.remove(si);
                    for (i, v) in new_vis.into_iter().enumerate() {
                        model.links[li].visuals.insert(si + i, v);
                    }
                    n
                }
                _ => { return; }
            };

            self.needs_upload = true;
            self.status_message = format!(
                "Decomposed {kind_str} of '{}' into {} shapes ({method_label}, {:.1}s)",
                link_name, n, elapsed.as_secs_f64()
            );
            // Record in undo history.
            if let Some(snapshot) = self.pre_frame_snapshot.take() {
                self.history.record(
                    &format!("Decompose {kind_str} of '{}' ({method_label})", link_name),
                    snapshot,
                );
                self.any_edit_this_frame = true;
            }
        }
    }

    /// Draw a progress overlay while a decomposition task is running.
    fn draw_decompose_progress(&self, ctx: &egui::Context) {
        let task = match self.decompose_task.as_ref() {
            Some(t) => t,
            None => return,
        };

        let phase = task.progress.load(std::sync::atomic::Ordering::Relaxed);
        let sub = task.sub_progress.load(std::sync::atomic::Ordering::Relaxed);
        let phase_str = misarta::decompose::phase_label(phase);
        let elapsed = task.started.elapsed().as_secs_f64();
        let method_label = task.method.label();

        egui::Window::new("⏳ Decomposing…")
            .id(egui::Id::new("decompose_progress"))
            .collapsible(false)
            .resizable(false)
            .anchor(egui::Align2::CENTER_CENTER, [0.0, 0.0])
            .show(ctx, |ui| {
                ui.vertical_centered(|ui| {
                    ui.label(
                        egui::RichText::new(format!("{method_label} Decomposition"))
                            .strong()
                            .size(16.0),
                    );
                    ui.add_space(8.0);
                    ui.spinner();
                    ui.add_space(4.0);
                    // Show phase label with sub-progress percentage when available.
                    if sub > 0
                        && phase != misarta::decompose::PHASE_DONE
                        && phase != misarta::decompose::PHASE_NOT_STARTED
                    {
                        ui.label(format!("{phase_str} ({sub}%)"));
                    } else {
                        ui.label(phase_str);
                    }
                    ui.add_space(4.0);
                    ui.label(
                        egui::RichText::new(format!("Elapsed: {elapsed:.1}s"))
                            .weak()
                            .monospace(),
                    );
                    // Compute progress fraction from phase + sub-progress.
                    // Phase ranges: PREPARING 0.00–0.05, DECOMPOSING 0.05–0.50,
                    //               HULLS 0.50–0.90, BUILDING 0.90–1.00
                    let frac = match phase {
                        misarta::decompose::PHASE_NOT_STARTED => 0.0,
                        misarta::decompose::PHASE_PREPARING => {
                            0.00 + 0.05 * (sub as f64 / 100.0)
                        }
                        misarta::decompose::PHASE_DECOMPOSING => {
                            0.05 + 0.45 * (sub as f64 / 100.0)
                        }
                        misarta::decompose::PHASE_HULLS => {
                            0.50 + 0.40 * (sub as f64 / 100.0)
                        }
                        misarta::decompose::PHASE_BUILDING => {
                            0.90 + 0.10 * (sub as f64 / 100.0)
                        }
                        misarta::decompose::PHASE_DONE => 1.0,
                        _ => 0.5,
                    };
                    ui.add_space(4.0);
                    ui.add(
                        egui::ProgressBar::new(frac as f32)
                            .desired_width(250.0)
                            .animate(true),
                    );
                });
            });
    }
}
