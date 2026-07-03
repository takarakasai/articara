use eframe::egui;
use nalgebra as na;
use std::collections::HashMap;
use std::path::PathBuf;
use std::sync::{Arc, Mutex};

use articara::camera::OrbitCamera;
use articara::dynamics;
use articara::format::RobotFormat;
use articara::history::History;
use crate::renderer::{DisplayMode, GlRenderer, MeshKind};
use articara::robot::RobotModel;

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

/// Which write operation the user has queued behind a pre-export
/// compatibility confirmation dialog. The dialog's Continue button
/// dispatches on this; Cancel just clears it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PendingExportAction {
    /// In-place save to the model's original URDF path.
    Save,
    /// Export to the directory + format selected in the export dialog.
    Export,
}

/// X-axis behaviour for the Joint Peaks plot.
#[cfg(feature = "mujoco")]
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum PeaksXAxisMode {
    /// X-axis grows with each new sample. Older samples beyond the buffer
    /// limit (or unlimited) are dropped from the front.
    Auto,
    /// X-axis is a sliding window of fixed length, anchored at the most
    /// recent sample.
    Fixed,
}

#[cfg(feature = "mujoco")]
impl PeaksXAxisMode {
    pub const ALL: [PeaksXAxisMode; 2] = [PeaksXAxisMode::Auto, PeaksXAxisMode::Fixed];
    pub fn label(self) -> &'static str {
        match self {
            PeaksXAxisMode::Auto => "Auto-update",
            PeaksXAxisMode::Fixed => "Fixed period",
        }
    }
}

/// Hard cap on the trace ring buffer when the user picks "Unlimited" history.
/// Sized to keep worst-case memory bounded (≈200k samples × ~hundreds of
/// joints × 3 metrics × 8 B → low hundreds of MB).
#[cfg(feature = "mujoco")]
pub const PEAKS_PLOT_UNLIMITED_CAP: usize = 200_000;

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

/// Interactive-IK state: solver configuration, the transient drag markers,
/// pinned links, loop-closure solve weights and chicken-head stabilisation.
/// Grouped so [`ArticaraApp`] carries one `ik` field instead of fifteen.
pub struct IkState {
    /// IK damping factor (λ for DLS / λ_max for SR-Inverse).
    pub damping: f32,
    /// IK solver method.
    pub solver: articara::robot::IkSolver,
    /// IK constraint dimensionality (2D screen-plane or 3D world).
    pub dof: articara::robot::IkDof,
    /// IK joint weight gradient: 0 = uniform, larger = prefer EE-proximal joints.
    pub weight_gradient: f32,
    /// IK root link name. None = use URDF root (full chain).
    pub root_link: Option<String>,
    /// IK target position in world space (debug overlay). None = no active IK.
    pub target_marker: Option<na::Point3<f32>>,
    /// Current end-effector position in world space (debug overlay).
    pub ee_marker: Option<na::Point3<f32>>,
    /// IK residual error (distance between EE and target) for debug overlay.
    pub error: Option<f32>,
    /// Links pinned to their world positions for multi-constraint IK.
    pub pinned_links: Vec<PinnedLink>,
    /// Weight for pin constraints (higher = harder constraint).
    pub pin_weight: f32,
    /// Weight for loop-closure constraints (higher = harder).
    pub loop_closure_weight: f32,
    /// Links to auto-pin at IK drag start (chicken-head stabilization).
    pub chicken_head_links: Vec<String>,
    /// Default DoF mode for chicken-head pins.
    pub chicken_head_dof: PinDof,
    /// When `true`, the next viewport left-click sets
    /// [`Self::loop_closure_link_b`] instead of doing the usual JointDrive
    /// selection. Toggled by the "👆 Pick B from viewport" button and
    /// cleared when a click lands or the user cancels.
    pub loop_closure_picking_b: bool,
    /// Index into `model.links` of the selected loop-closure link B.
    /// `None` when nothing has been chosen yet.
    pub loop_closure_link_b: Option<usize>,
}

impl Default for IkState {
    fn default() -> Self {
        Self {
            damping: 0.05,
            solver: articara::robot::IkSolver::SrInverse,
            dof: articara::robot::IkDof::ScreenPlane2D,
            weight_gradient: 1.5,
            root_link: None,
            target_marker: None,
            ee_marker: None,
            error: None,
            pinned_links: Vec::new(),
            pin_weight: 10.0,
            loop_closure_weight: 50.0,
            chicken_head_links: Vec::new(),
            chicken_head_dof: PinDof::Position,
            loop_closure_picking_b: false,
            loop_closure_link_b: None,
        }
    }
}

pub struct ArticaraApp {
    model: Option<RobotModel>,
    /// Active camera used for the main viewport render and all
    /// world-to-screen / screen-to-world projections. In Free mode this
    /// is the user-controlled orbit camera; in TPS mode it's overwritten
    /// each frame from `tps_settings` + the followed link's pose.
    camera: OrbitCamera,
    /// Snapshot of the user-driven free camera, kept while TPS mode is
    /// active so toggling back to Free restores the previous view
    /// instead of leaving the camera mid-follow.
    saved_free_camera: OrbitCamera,
    /// Live TPS camera. Always tracked (regardless of which mode is
    /// "main") so the wipe / picture-in-picture overlay can show the
    /// non-active perspective without a one-frame lag.
    tps_camera: OrbitCamera,
    tps_settings: articara::camera::TpsSettings,
    camera_mode: articara::camera::CameraMode,
    /// When `true` the main viewport additionally renders the *other*
    /// camera (free or TPS, opposite of the active one) as a small
    /// picture-in-picture wipe in the upper-right corner.
    show_camera_wipe: bool,
    /// Pending Save/Export action awaiting user confirmation in the
    /// `export_compat_issues` dialog. `None` outside the dialog flow.
    pending_export_action: Option<PendingExportAction>,
    /// Compatibility issues raised by the latest pre-export analysis.
    /// Non-empty while the confirmation dialog is shown.
    export_compat_issues: Vec<articara::format::ExportIssue>,
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
    /// Interactive-IK state: solver settings, drag markers, pins,
    /// loop-closure weights and chicken-head stabilisation. See [`IkState`].
    ik: IkState,
    /// Show center-of-mass markers and mass labels.
    show_com: bool,
    /// Show the **whole-robot** centre-of-mass marker (= mass-weighted
    /// centroid of every link). Independent from [`Self::show_com`]
    /// which draws one sphere per link.
    show_total_com: bool,
    /// Show the support polygon — convex hull of the four foot world
    /// positions, projected down to the ground plane. Useful for
    /// visualising static-stability margin during LinearCrawl etc.
    show_support_polygon: bool,
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
    validation_results: Vec<articara::robot::InertiaValidation>,
    // --- Dynamics analysis state ---
    /// Selected end-effector link for payload capacity analysis.
    dynamics_ee_link: Option<String>,
    /// Cached dynamics analysis result.
    dynamics_result: Option<dynamics::StaticAnalysis>,
    /// Simulation-execution state (payload / MuJoCo sims, estimators,
    /// WBC, external forces, drag). See [`dynamics_panel::SimState`].
    sim: dynamics_panel::SimState,
    /// Name buffer for the next "Save current pose as…" entry.
    pose_save_name: String,
    /// Name buffer for creating a new (empty) sequence.
    sequence_save_name: String,
    /// Pose name buffer for the per-row "+ Add step" form (per sequence).
    /// Keyed by sequence index so multiple sequence panels can be edited
    /// independently. Held on the app rather than per-frame so it survives
    /// across frames.
    sequence_step_pose_buf: std::collections::HashMap<usize, String>,
    /// Default interpolation kind seeded into newly-saved poses.
    pose_transition_kind: misarta::trajectory::InterpolationKind,
    /// Default transition duration (s) seeded into newly-saved poses.
    pose_transition_duration: f32,
    /// Joint Peaks plot window state (open flag, selection, axis and
    /// CSV-export dialog). See [`peaks_plot_window::PeaksPlotState`].
    #[cfg(feature = "mujoco")]
    peaks_plot: peaks_plot_window::PeaksPlotState,
    /// File dialog for loading a Rhai script to run in the console.
    dlg_open_script: file_dialog::FileDialog,
    /// Most-recently-loaded script path (used to pre-fill the file dialog).
    script_path: String,
    /// One-shot: when `Some`, the next console-update tick reads + runs this
    /// path and clears the field. Set by the "📂 Run file…" button after the
    /// file dialog confirms.
    pending_script_run: Option<std::path::PathBuf>,
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
    script_engine: Option<articara::scripting_model::ModelScriptEngine>,
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
    /// Whether the collision-pair matrix dialog is open.
    show_collision_matrix: bool,
    /// Whether the actuator-settings dialog is open.
    show_actuator_dialog: bool,
    /// Persistent bulk-edit slot state for the actuator dialog. Kept here
    /// rather than as a closure-local so the user's "include in bulk write"
    /// toggles and values survive across UI ticks.
    actuator_bulk: actuator_dialog::BulkEdit,
    /// Pending `.misa` LoadReport surfaced as a dialog after load.
    /// `None` when nothing to show (or already dismissed). The dialog
    /// closes itself when the user clicks OK; we don't auto-clear so the
    /// user has time to read sanitisation / missing-mesh entries.
    pending_misa_report: Option<misarta::native::LoadReport>,
    /// Pending MuJoCo runtime version mismatch surfaced as a startup
    /// dialog. Populated in [`Self::new`] from the cached
    /// [`articara::mujoco_version::CheckResult`]; cleared when the user
    /// dismisses the dialog. Only set when the `mujoco` feature is
    /// active **and** the version check came back as `Mismatch`.
    #[cfg(feature = "mujoco")]
    pending_mujoco_warning:
        Option<articara::mujoco_version::CheckResult>,
    /// Quadruped gait controller + panel tuning state. See
    /// [`gait_panel::GaitPanelState`].
    gait: gait_panel::GaitPanelState,
    /// Kinematic playback: drive `model.joint_positions` and
    /// `model.base_transform` directly from the gait controller's
    /// `tick()` output every frame, bypassing MuJoCo. Lets the user
    /// see the planner's intended gait pattern in isolation (no
    /// physical slip, PD lag, contact dynamics, or trunk sway from
    /// inertia). When toggled ON, the current `base_transform` is
    /// snapshotted as the playback anchor and the gait is reset so
    /// `body_state` integrates from world origin.
    kinematic_playback_active: bool,
    /// Snapshot of `model.base_transform` at the moment kinematic
    /// playback was last enabled. The plotted trunk pose each frame
    /// is `kinematic_playback_base_offset · body_state_iso(out)` so
    /// the robot moves relative to wherever the user had placed it
    /// before pressing the toggle. (`f64` to match `base_transform`.)
    kinematic_playback_base_offset: na::Isometry3<f64>,
    /// Live gait viewer: subscribe to a `go2-gait-runner --viz` Zenoh stream
    /// and animate the loaded model from the received frames. See
    /// [`articara::viz_feed`].
    #[cfg(feature = "viz")]
    viz: articara::viz_feed::VizFeedState,
}

/// Whether a decompose task targets a visual or collision slot.
#[derive(Clone, Copy, PartialEq, Eq)]
enum DecomposeTarget {
    Visual,
    Collision,
}

/// The result produced by a background decomposition thread.
enum DecomposeResult {
    Visuals(Vec<articara::robot::VisualData>),
    Collisions(Vec<articara::robot::CollisionData>),
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
            saved_free_camera: OrbitCamera::new(),
            tps_camera: OrbitCamera::new(),
            tps_settings: articara::camera::TpsSettings::default(),
            camera_mode: articara::camera::CameraMode::Free,
            show_camera_wipe: false,
            pending_export_action: None,
            export_compat_issues: Vec::new(),
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
            ik: IkState::default(),
            show_com: false,
            show_total_com: false,
            show_support_polygon: false,
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
            sim: dynamics_panel::SimState::default(),
            pose_save_name: String::new(),
            sequence_save_name: String::new(),
            sequence_step_pose_buf: std::collections::HashMap::new(),
            pose_transition_kind: misarta::trajectory::InterpolationKind::QuinticSmooth,
            pose_transition_duration: 1.0,
            #[cfg(feature = "mujoco")]
            #[cfg(feature = "mujoco")]
            peaks_plot: peaks_plot_window::PeaksPlotState::default(),
            dlg_open_script: file_dialog::FileDialog::new("dlg_open_script"),
            script_path: String::new(),
            pending_script_run: None,
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
            show_collision_matrix: false,
            show_actuator_dialog: false,
            actuator_bulk: actuator_dialog::BulkEdit::default(),
            pending_misa_report: None,
            #[cfg(feature = "mujoco")]
            pending_mujoco_warning: match articara::mujoco_version::cached() {
                Some(r @ articara::mujoco_version::CheckResult::Mismatch { .. }) => {
                    Some(r.clone())
                }
                _ => None,
            },
            gait: gait_panel::GaitPanelState::default(),
            kinematic_playback_active: false,
            kinematic_playback_base_offset: na::Isometry3::identity(),
            #[cfg(feature = "viz")]
            viz: articara::viz_feed::VizFeedState::default(),
        }
    }

    /// Pull foot link names out of the model's first gait descriptor
    /// (if any) into the UI panel state. Called after a sidecar load so
    /// the user sees their saved configuration. Falls back to the default
    /// FL/FR/RL/RR_foot when the model has no gait entries.
    pub(crate) fn sync_gait_panel_from_model(&mut self) {
        let descriptor = self
            .model
            .as_ref()
            .and_then(|m| m.gaits.first().cloned());
        let names = match descriptor {
            Some(d) => [d.fl_foot, d.fr_foot, d.rl_foot, d.rr_foot],
            None => articara::gait::DEFAULT_FOOT_LINKS.map(|(_, s)| s.to_string()),
        };
        for (slot, name) in names.into_iter().enumerate() {
            self.gait.foot_links[slot].1 = name;
        }
    }

    /// Push the UI's current foot link names + (if present) the live gait
    /// controller's config into `model.gaits[0]` so the next sidecar save
    /// picks them up. Inserts a default descriptor if none exists yet.
    pub(crate) fn sync_gait_panel_to_model(&mut self) {
        let Some(model) = self.model.as_mut() else {
            return;
        };
        if model.gaits.is_empty() {
            model.gaits.push(articara::rbd::model::GaitDescriptor::default_trot());
        }
        let g = &mut model.gaits[0];
        g.fl_foot = self.gait.foot_links[0].1.clone();
        g.fr_foot = self.gait.foot_links[1].1.clone();
        g.rl_foot = self.gait.foot_links[2].1.clone();
        g.rr_foot = self.gait.foot_links[3].1.clone();
        if let Some(ctrl) = self.gait.controller.as_ref() {
            let cfg = ctrl.config();
            g.gait_type = match cfg.gait_type {
                quadruped_gait::GaitType::Trot => misarta::config::GaitTypeConfig::Trot,
                quadruped_gait::GaitType::Walk => misarta::config::GaitTypeConfig::Walk,
                quadruped_gait::GaitType::Pace => misarta::config::GaitTypeConfig::Pace,
                quadruped_gait::GaitType::Bound => misarta::config::GaitTypeConfig::Bound,
                quadruped_gait::GaitType::Crawl => misarta::config::GaitTypeConfig::Crawl,
            };
            g.cycle_period_s = cfg.cycle_period_s;
            g.duty_factor = cfg.duty_factor;
            g.swing_height_m = cfg.swing_height_m;
            g.max_step_length_m = cfg.max_step_length_m;
            g.knee_forward = ctrl.knee_forward();
            g.four_support_fraction = cfg.four_support_fraction;
        }
    }

    pub fn load_model(&mut self, path: PathBuf) {
        // Dispatch on extension up-front so `.misa` files can capture the
        // LoadReport (which includes identifier sanitisations and missing
        // mesh references — the user needs to see those, not just the
        // success summary).
        let detected = RobotFormat::detect(&path);
        let load_outcome: Result<RobotModel, String> =
            if matches!(detected, Some(RobotFormat::Misa)) {
                match RobotModel::from_misa_with_report(&path) {
                    Ok((model, report)) => {
                        if !report.is_empty() {
                            self.pending_misa_report = Some(report);
                        }
                        Ok(model)
                    }
                    Err(e) => Err(e),
                }
            } else {
                RobotModel::from_file(&path)
            };

        match load_outcome {
            Ok(model) => {
                self.status_message = format!(
                    "Loaded: {} ({} links, {} joints)",
                    model.name,
                    model.links.len(),
                    model.joints.len()
                );
                self.model = Some(model);
                self.urdf_path_input = path.display().to_string();
                // Legacy URDF + `.misarta.toml` sidecar path. `.misa` already
                // carries everything in a single file, so the sidecar load is
                // a no-op for those — but we still call it unconditionally
                // so an existing `<name>.misarta.toml` next to a `.misa`
                // continues to merge in (helps users mid-migration).
                if let Some(ref mut m) = self.model {
                    if let Some(report) = m.load_sidecar_config() {
                        self.status_message = format!(
                            "{}  ·  sidecar: {}/{} actuator(s), {} pose(s), {} loop(s){}",
                            self.status_message,
                            report.n_actuators_applied,
                            report.n_actuators_total,
                            report.n_poses,
                            report.n_loop_closures,
                            if report.unmatched_actuators.is_empty() {
                                String::new()
                            } else {
                                format!(
                                    "  ⚠ unmatched: {}",
                                    report.unmatched_actuators.join(", ")
                                )
                            },
                        );
                    }
                }
                // Auto-set export format to match source
                if let Some(fmt) = RobotFormat::detect(&path) {
                    self.export_format = fmt;
                }
                self.selected_link = None;
                self.selected_joint = None;
                self.needs_upload = true;
                self.ik.root_link = None; // reset IK root on new model
                self.history.clear();
                // Pull saved gait foot link names out of the sidecar (if
                // any) so the UI panel reflects the user's last setup.
                // Drop any stale gait controller — its kinematics belong
                // to the previous model.
                self.gait.controller = None;
                self.sync_gait_panel_from_model();
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

    /// Recompute `self.tps_camera` from the current model pose and
    /// `tps_settings`. The followed link defaults to the model's root
    /// link when no explicit name is set. With no model loaded the TPS
    /// camera resolves to "track the world origin".
    pub(crate) fn refresh_tps_camera(&mut self) {
        let link_world = if let Some(model) = self.model.as_ref() {
            // Walk forward kinematics to find the followed link's world
            // pose. We don't cache because the cost is cheap and the
            // pose changes whenever joints / base_transform move.
            let transforms = model.compute_transforms();
            let link_name = self
                .tps_settings
                .follow_link
                .as_deref()
                .unwrap_or(&model.root_link);
            transforms
                .get(link_name)
                .copied()
                .unwrap_or_else(|| {
                    // Fallback if the configured link disappeared
                    // (mid-rename, etc.): use the base transform.
                    model.base_transform.cast::<f32>()
                })
        } else {
            na::Isometry3::identity()
        };
        self.tps_camera.update_from_tps(&link_world, &self.tps_settings);
    }

    /// Switch the active camera mode, preserving the user's free-camera
    /// state across the transition so toggling Free → TPS → Free
    /// returns to the same orbit pose.
    pub(crate) fn set_camera_mode(&mut self, mode: articara::camera::CameraMode) {
        use articara::camera::CameraMode;
        if self.camera_mode == mode {
            return;
        }
        match (self.camera_mode, mode) {
            (CameraMode::Free, CameraMode::Tps) => {
                // Save the user's current free-camera so we can come back to it.
                self.saved_free_camera = self.camera.clone();
            }
            (CameraMode::Tps, CameraMode::Free) => {
                // Restore the saved free camera.
                self.camera = self.saved_free_camera.clone();
            }
            _ => {}
        }
        self.camera_mode = mode;
    }

    /// Dispatch mouse orbit/pan/zoom input depending on the active
    /// camera mode. Free mode passes through to the standard
    /// `OrbitCamera::handle_orbit_pan_zoom`; TPS mode redirects yaw /
    /// pitch / scroll into `tps_settings` (panning is no-op in TPS
    /// since the look-at follows the body).
    pub(crate) fn handle_camera_input(&mut self, response: &eframe::egui::Response) {
        use articara::camera::CameraMode;
        match self.camera_mode {
            CameraMode::Free => {
                self.camera.handle_orbit_pan_zoom(response);
            }
            CameraMode::Tps => {
                if response.dragged_by(eframe::egui::PointerButton::Primary) {
                    let delta = response.drag_delta();
                    self.tps_settings.yaw_offset -= delta.x * 0.005;
                    self.tps_settings.pitch_offset += delta.y * 0.005;
                    self.tps_settings.pitch_offset =
                        self.tps_settings.pitch_offset.clamp(-1.5, 1.5);
                }
                if response.hovered() {
                    let scroll = response.ctx.input(|i| i.smooth_scroll_delta.y);
                    if scroll != 0.0 {
                        self.tps_settings.distance *= 1.0 - scroll * 0.002;
                        self.tps_settings.distance =
                            self.tps_settings.distance.clamp(0.05, 50.0);
                    }
                }
                // Right / middle drag: nudge the look-at point's local
                // offset so the camera frames slightly above/around the
                // body (e.g. raise to chest height). Use small gain.
                if response.dragged_by(eframe::egui::PointerButton::Secondary)
                    || response.dragged_by(eframe::egui::PointerButton::Middle)
                {
                    let delta = response.drag_delta();
                    let pan_speed = self.tps_settings.distance * 0.002;
                    self.tps_settings.target_local_offset.z += delta.y * pan_speed;
                }
            }
        }
    }

    /// Build a fresh Madgwick estimator for every IMU sensor in the
    /// loaded `RobotModel`. Called when MuJoCo sim starts so a previous
    /// Play→Stop cycle's estimator state doesn't bleed into the new
    /// run.
    #[cfg(feature = "mujoco")]
    pub(super) fn rebuild_imu_estimators(&mut self) {
        self.sim.imu_estimators.clear();
        self.sim.imu_last_sim_time.clear();
        let Some(ref model) = self.model else {
            return;
        };
        for sensor in &model.sensors {
            if matches!(sensor.kind, articara::rbd::model::SensorKind::Imu { .. }) {
                self.sim.imu_estimators.insert(
                    sensor.name.clone(),
                    articara::attitude_estimator::MadgwickAhrs::default(),
                );
            }
        }
    }

    /// Advance the dynamics simulation by one frame, modifying model state.
    fn step_dynamics_sim(&mut self) {
        // Async queue integration: instantaneous ops (Print, SaveCsv,
        // SetGaitVelocity, SetPositionTarget) drain here at the top of the
        // tick so they take effect *before* the gait+WBC control loop runs.
        // A `StepFrames` op at the queue head doesn't bypass the regular
        // path — instead it caps how many physics frames this UI tick can
        // advance, so the gait controller still gets ticked and writes
        // fresh position targets / WBC τ_ff each UI frame. Without this
        // the script-driven timeline would step physics with whatever
        // position_targets were latched at script start, and the robot
        // wouldn't actually walk.
        #[cfg(feature = "mujoco")]
        self.drain_instantaneous_async_ops();
        #[cfg(feature = "mujoco")]
        let async_step_remaining: Option<u32> = self
            .sim.mujoco_sim
            .as_ref()
            .and_then(|s| match s.async_peek() {
                Some(articara::mujoco_sim::AsyncSimOp::StepFrames(n)) => Some(*n),
                _ => None,
            });

        #[cfg(feature = "mujoco")]
        let has_mujoco = self.sim.mujoco_sim.is_some();
        #[cfg(not(feature = "mujoco"))]
        let has_mujoco = false;

        let sim = match self.sim.dynamics_sim.as_mut() {
            Some(s) => s,
            None if !has_mujoco => {
                self.sim.dynamics_last_instant = None;
                return;
            }
            None => {
                // mujoco_sim is active but dynamics_sim is not — fall through to MuJoCo step
                // Use a dummy reference; the MuJoCo branch returns before using `sim`.
                #[cfg(feature = "mujoco")]
                {
                    let now = std::time::Instant::now();
                    let frame_request = self.sim.dynamics_step_frames.take();
                    let enforce_limits = self.sim.enforce_actuator_limits;
                    if let Some(mj_sim) = self.sim.mujoco_sim.as_mut() {
                        if let Some(ref mut model) = self.model {
                            if let Some(n) = frame_request {
                                self.sim.dynamics_sim_paused = true;
                                self.sim.dynamics_last_instant = Some(now);
                                if n > 0 {
                                    mj_sim.step_n_frames(model, n as u32, enforce_limits);
                                } else if n < 0 {
                                    mj_sim.step_back_frames(model, (-n) as u32);
                                }
                                return;
                            }
                            if self.sim.dynamics_sim_paused {
                                self.sim.dynamics_last_instant = Some(now);
                                return;
                            }
                            let wall_dt = match self.sim.dynamics_last_instant {
                                Some(prev) => now.duration_since(prev).as_secs_f32().min(0.05),
                                None => 0.016,
                            };
                            self.sim.dynamics_last_instant = Some(now);
                            // Apply the user's Speed multiplier so the
                            // sim can be slowed down for inspection or
                            // sped up for quick rollouts. Clamp the
                            // multiplier to a sane range to avoid
                            // pathological huge dt values when the
                            // slider is pushed up.
                            let speed = self.sim.dynamics_sim_speed.clamp(0.05, 5.0);
                            let dt = wall_dt * speed;
                            // If the gait controller is enabled, advance
                            // it by the same `dt` as the physics step and
                            // write its joint targets into the sim's
                            // position_targets BEFORE the controller runs.
                            // This way the underlying PD / computed-torque
                            // loop sees fresh setpoints every tick.
                            // ── Madgwick estimators (sim-time driven) ──
                            // Pull fresh accel + gyro from MuJoCo and integrate
                            // every IMU's quaternion. Inlined here (instead of
                            // a `self.update_imu_estimators()` call) because
                            // `mj_sim` and `model` are both held by this
                            // scope's mutable borrow of `self.*`, so going
                            // through a `&mut self` method would double-borrow.
                            for reading in mj_sim.imu_readings(model) {
                                let dt_imu = match self
                                    .sim.imu_last_sim_time
                                    .get(&reading.name)
                                {
                                    Some(prev) if reading.sim_time > *prev => {
                                        reading.sim_time - *prev
                                    }
                                    _ => {
                                        self.sim.imu_last_sim_time
                                            .insert(reading.name.clone(), reading.sim_time);
                                        continue;
                                    }
                                };
                                self.sim.imu_last_sim_time
                                    .insert(reading.name.clone(), reading.sim_time);
                                if let Some(est) =
                                    self.sim.imu_estimators.get_mut(&reading.name)
                                {
                                    est.update_imu(reading.gyro, reading.accel, dt_imu);
                                }
                            }

                            // Resolve the body pose (yaw + position) for the
                            // MPC's `body_state` based on the user's selected
                            // PoseSource.
                            let trunk = &model.root_link;
                            let yaw_observed = match self.sim.pose_source {
                                articara::gait::PoseSource::ImuFusion
                                | articara::gait::PoseSource::LegOdometry => {
                                    // Primary IMU = first one mounted on the
                                    // trunk; fallback to any IMU; fallback to
                                    // MuJoCo's xquat when no IMU is wired.
                                    let primary_imu = model
                                        .sensors
                                        .iter()
                                        .find(|s| {
                                            matches!(
                                                s.kind,
                                                articara::rbd::model::SensorKind::Imu { .. }
                                            ) && &s.link == trunk
                                        })
                                        .or_else(|| {
                                            model.sensors.iter().find(|s| {
                                                matches!(
                                                    s.kind,
                                                    articara::rbd::model::SensorKind::Imu {
                                                        ..
                                                    }
                                                )
                                            })
                                        });
                                    primary_imu
                                        .and_then(|s| self.sim.imu_estimators.get(&s.name))
                                        .map(|est| est.euler_zyx().2)
                                        .or_else(|| mj_sim.body_world_yaw(trunk))
                                        .unwrap_or(0.0)
                                }
                                articara::gait::PoseSource::GroundTruth => mj_sim
                                    .body_world_yaw(trunk)
                                    .unwrap_or(0.0),
                            };

                            // Update the leg-odometry estimator if the user
                            // wants the LegOdometry source. We need joint
                            // state from MuJoCo per leg + last-tick stance
                            // flags + body angular velocity (gyro / cvel).
                            // The estimator maintains its own integrated
                            // position; we read it back below.
                            if matches!(
                                self.sim.pose_source,
                                articara::gait::PoseSource::LegOdometry
                            ) {
                                if let Some(gc) = self.gait.controller.as_ref() {
                                    let kin = gc.kinematics().clone();
                                    let joint_indices = gc.joint_indices();
                                    let joint_signs = gc.joint_signs();
                                    let mut legs =
                                        [(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, false); 4];
                                    for slot in 0..4 {
                                        for k in 0..3 {
                                            let ji = joint_indices[slot][k];
                                            let sign = joint_signs[slot][k];
                                            // MuJoCo state is in URDF axes;
                                            // IK convention is `q_ik = sign · q_urdf`,
                                            // same factor for q̇.
                                            let (q_urdf, qd_urdf) = model
                                                .joints
                                                .get(ji)
                                                .and_then(|j| {
                                                    let info =
                                                        mj_sim.joint_q_qd(&j.name)?;
                                                    Some(info)
                                                })
                                                .unwrap_or((0.0, 0.0));
                                            let q_ik = sign * q_urdf;
                                            let qd_ik = sign * qd_urdf;
                                            match k {
                                                0 => {
                                                    legs[slot].0 = q_ik;
                                                    legs[slot].3 = qd_ik;
                                                }
                                                1 => {
                                                    legs[slot].1 = q_ik;
                                                    legs[slot].4 = qd_ik;
                                                }
                                                _ => {
                                                    legs[slot].2 = q_ik;
                                                    legs[slot].5 = qd_ik;
                                                }
                                            }
                                        }
                                        legs[slot].6 = self.sim.leg_odometry_last_stance[slot];
                                    }
                                    // Body angular velocity from MuJoCo cvel
                                    // (= gyro on hardware). Pre-feedback to
                                    // the estimator so the ω×r term is real.
                                    let omega_w = mj_sim
                                        .body_world_angular_velocity(trunk)
                                        .unwrap_or([0.0, 0.0, 0.0]);
                                    self.sim.leg_odometry.update(
                                        &kin,
                                        legs,
                                        nalgebra::Vector3::new(
                                            omega_w[0], omega_w[1], omega_w[2],
                                        ),
                                        yaw_observed,
                                        dt as f64,
                                    );
                                }
                            }

                            let pos_observed = match self.sim.pose_source {
                                articara::gait::PoseSource::LegOdometry => {
                                    let p = self.sim.leg_odometry.position_world();
                                    [p.x, p.y, p.z]
                                }
                                _ => mj_sim
                                    .body_world_position(trunk)
                                    .unwrap_or([0.0, 0.0, 0.0]),
                            };

                            if let Some(gc) = self.gait.controller.as_mut() {
                                if gc.is_enabled() {
                                    // Feed observed body linear + angular
                                    // velocity (world frame) so the MPC's
                                    // SRBD layer can compute real tracking
                                    // errors. CHAMP ignores both.
                                    let v = mj_sim
                                        .body_world_linear_velocity(&model.root_link)
                                        .unwrap_or([0.0, 0.0, 0.0]);
                                    let w = mj_sim
                                        .body_world_angular_velocity(&model.root_link)
                                        .unwrap_or([0.0, 0.0, 0.0]);
                                    gc.set_body_state_observed(
                                        nalgebra::Vector3::new(v[0], v[1], v[2]),
                                        nalgebra::Vector3::new(w[0], w[1], w[2]),
                                    );
                                    // Feed observed pose so the MPC's
                                    // `body_state` reflects the *real* yaw +
                                    // position instead of integrating cmd
                                    // (which drifts whenever the robot
                                    // can't perfectly track the command).
                                    gc.set_body_pose_observed(
                                        yaw_observed,
                                        nalgebra::Vector3::new(
                                            pos_observed[0],
                                            pos_observed[1],
                                            pos_observed[2],
                                        ),
                                    );
                                    let (out, targets, torque_ff) = gc.tick(dt as f64);
                                    for (idx, q) in targets {
                                        mj_sim.set_position_target(idx, q);
                                    }
                                    // Phase 4 WBC (single-layer feedforward):
                                    // layer the SRBD MPC's GRF-derived
                                    // `-J^T · f_GRF` torque on top of the
                                    // position-mode PD. CHAMP and the
                                    // pre-first-MPC-tick path emit zeros,
                                    // so this is a no-op for them.
                                    for (idx, tau) in torque_ff {
                                        mj_sim.set_torque_feedforward(idx, tau);
                                    }
                                    // Snapshot stance flags for the
                                    // *next* leg-odometry update — the
                                    // estimator runs *before* this tick,
                                    // so it needs the previous tick's
                                    // schedule.
                                    for slot in 0..4 {
                                        self.sim.leg_odometry_last_stance[slot] =
                                            out.legs[slot].phase.is_stance;
                                    }

                                    // ── Phase A: Hierarchical WBC ──────
                                    // When the user enables WBC, replace
                                    // the per-joint position PD path with
                                    // a full HoQp solve using the gait's
                                    // post-tick targets + the SRBD MPC's
                                    // predicted GRFs as references. Only
                                    // active in MPC mode (CHAMP doesn't
                                    // produce GRFs).
                                    let wbc_active = self.sim.wbc_enabled
                                        && matches!(
                                            gc.mode(),
                                            quadruped_gait::GaitMode::Mpc
                                                | quadruped_gait::GaitMode::CentroidalSrbd
                                                | quadruped_gait::GaitMode::FullCentroidal
                                        );
                                    if wbc_active {
                                        // Lazy-initialise the pipeline on first use.
                                        if self.sim.wbc_pipeline.is_none() {
                                            let foot_links: [String; 4] = [
                                                self.gait.foot_links[0].1.clone(),
                                                self.gait.foot_links[1].1.clone(),
                                                self.gait.foot_links[2].1.clone(),
                                                self.gait.foot_links[3].1.clone(),
                                            ];
                                            let mut new_pipe =
                                                articara::wbc_pipeline::WbcPipeline::new(
                                                    model, foot_links,
                                                );
                                            // Sync mass / inertia from the auto-detected
                                            // SrbdMpcConfig so `predicted_base_accel_world`
                                            // uses the **right** robot physics. Default
                                            // values match a 9 kg Cheetah; running namiashi
                                            // (2.4 kg) without this override produces a
                                            // ~4× force overestimate that flings the legs
                                            // into the ground and tips the robot over.
                                            if let Some(full_cfg) =
                                                gc.full_centroidal_mpc_config()
                                            {
                                                // FullCentroidal mode shares the
                                                // CoM-aware `a_base_des` path —
                                                // the 24-state MPC's GRFs satisfy
                                                // the centroidal moment-arm
                                                // relationship.
                                                new_pipe.mass_kg = full_cfg.mass_kg;
                                                new_pipe.centroidal_inertia_body = Some(
                                                    full_cfg.centroidal_inertia_body,
                                                );
                                                new_pipe.com_offset_body =
                                                    full_cfg.com_offset_body;
                                            } else if let Some(centroidal_cfg) =
                                                gc.centroidal_mpc_config()
                                            {
                                                // CentroidalSrbd mode: WBC uses
                                                // the CoM-aware `a_base_des` path.
                                                new_pipe.mass_kg = centroidal_cfg.mass_kg;
                                                new_pipe.centroidal_inertia_body = Some(
                                                    centroidal_cfg.centroidal_inertia_body,
                                                );
                                                new_pipe.com_offset_body =
                                                    centroidal_cfg.com_offset_body;
                                            } else if let Some(srbd_cfg) = gc.srbd_mpc_config() {
                                                // body-root SRBD: leave
                                                // centroidal_inertia_body = None
                                                // so the WBC stays on the SRBD
                                                // body-root reference path.
                                                new_pipe.mass_kg = srbd_cfg.mass_kg;
                                                new_pipe.inertia_diag_body =
                                                    srbd_cfg.inertia_diag_body;
                                                new_pipe.centroidal_inertia_body = None;
                                            }
                                            self.sim.wbc_pipeline = Some(new_pipe);
                                        }
                                        // Pull MPC predicted GRFs (if any)
                                        // for the contact-force regulariser.
                                        let f_grf_world: [nalgebra::Vector3<f64>; 4] = gc
                                            .predicted_grfs()
                                            .map(|sol| sol.grfs_first_step)
                                            .unwrap_or([nalgebra::Vector3::zeros(); 4]);
                                        let cmd = gc.velocity_cmd();
                                        // The gait command is naturally body-frame; the
                                        // WBC pipeline now expects body-frame inputs and
                                        // rotates the observation internally.
                                        let v_cmd_body = nalgebra::Vector3::new(
                                            cmd.vx, cmd.vy, 0.0,
                                        );
                                        let v_obs_v3 =
                                            nalgebra::Vector3::new(v[0], v[1], v[2]);
                                        let omega_obs_v3 =
                                            nalgebra::Vector3::new(w[0], w[1], w[2]);
                                        let kin = gc.kinematics().clone();
                                        let joint_indices = gc.joint_indices();
                                        let joint_signs = gc.joint_signs();
                                        let contact_flag = [
                                            out.legs[0].phase.is_stance,
                                            out.legs[1].phase.is_stance,
                                            out.legs[2].phase.is_stance,
                                            out.legs[3].phase.is_stance,
                                        ];
                                        let pipeline = self.sim.wbc_pipeline.as_mut().unwrap();
                                        // P5b: per-cmd-direction weight scheduling.
                                        // For lateral / yaw commands the joint-space
                                        // swing_leg PD reaction-torques the body in
                                        // the wrong direction; `for_cmd` linearly fades
                                        // the swing_leg weight from forward-default 1.0
                                        // down to lateral-optimum 0.1.
                                        // Mode-aware swing_leg weight (D2/H):
                                        // CentroidalSrbd halves swing_leg
                                        // (`for_cmd_centroidal`) to reduce
                                        // joint-PD reaction-torque amplification
                                        // through the MPC's CoM-aware predictions.
                                        pipeline.weights = if pipeline
                                            .centroidal_inertia_body
                                            .is_some()
                                        {
                                            quadruped_gait::wbc::WbcWeights::for_cmd_centroidal(&cmd)
                                        } else {
                                            quadruped_gait::wbc::WbcWeights::for_cmd(&cmd)
                                        };
                                        let taus = pipeline.solve(
                                            model,
                                            mj_sim,
                                            &out,
                                            &kin,
                                            joint_indices,
                                            joint_signs,
                                            &v_cmd_body,
                                            cmd.wz,
                                            &v_obs_v3,
                                            &omega_obs_v3,
                                            &f_grf_world,
                                            contact_flag,
                                            dt as f64,
                                        );
                                        // Hybrid joint command (legged_control 流):
                                        // route the WBC τ as **feedforward** on top of
                                        // Position-PD instead of replacing the whole PD
                                        // path. The plain `set_wbc_torques` route bypasses
                                        // gravity-comp + Position-PD entirely, which
                                        // causes drift / collapse over time because the
                                        // QP produces accelerations not positions. The
                                        // Hybrid scheme is what `wbc_walk` /
                                        // `integration_walk` regression tests use, so
                                        // the GUI now matches their behaviour.
                                        for (ji, &tau) in taus.iter().enumerate() {
                                            mj_sim.set_torque_feedforward(ji, tau);
                                        }
                                        mj_sim.clear_wbc_torques();
                                    } else {
                                        // WBC off: clear both override paths so a
                                        // previous WBC ON tick's τ_ff doesn't keep
                                        // pushing the joints after the user toggles WBC
                                        // back off (otherwise the body drifts).
                                        mj_sim.clear_wbc_torques();
                                        mj_sim.clear_torque_feedforward();
                                    }
                                } else {
                                    // Disabled gait: drop any stale
                                    // feedforward so the legs aren't
                                    // commanded into motion by the last
                                    // tick's GRF prediction.
                                    mj_sim.clear_torque_feedforward();
                                    mj_sim.clear_wbc_torques();
                                }
                            } else {
                                mj_sim.clear_torque_feedforward();
                                mj_sim.clear_wbc_torques();
                            }
                            // Step physics. When a script's async queue has
                            // a `StepFrames` op at the head, switch from the
                            // wall-clock-driven `step(dt)` accumulator to an
                            // exact `step_n_frames(N)` capped by both the
                            // wall-clock budget and the remaining StepFrames
                            // count. This keeps the script's timeline
                            // deterministic while still ticking gait+WBC
                            // each UI frame (so the robot actually walks).
                            if let Some(remaining) = async_step_remaining {
                                let mj_dt = mj_sim.timestep();
                                let frames_from_wall =
                                    ((dt as f64) / mj_dt).round().max(1.0) as u32;
                                let frames_to_step =
                                    frames_from_wall.min(remaining);
                                if frames_to_step > 0 {
                                    mj_sim.step_n_frames(
                                        model,
                                        frames_to_step,
                                        enforce_limits,
                                    );
                                    mj_sim.async_consume_step_frames(
                                        frames_to_step,
                                    );
                                }
                            } else {
                                mj_sim.step(model, dt as f64, enforce_limits);
                            }
                        }
                    }
                }
                return;
            }
        };

        // Handle pause / step-once (payload sim path; frame stepping is MuJoCo-only)
        if self.sim.dynamics_sim_paused {
            self.sim.dynamics_last_instant = Some(std::time::Instant::now());
            return;
        }

        // Compute delta-time
        let now = std::time::Instant::now();
        let dt = {
            let d = match self.sim.dynamics_last_instant {
                Some(prev) => now.duration_since(prev).as_secs_f32().min(0.05),
                None => 0.016,
            };
            self.sim.dynamics_last_instant = Some(now);
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
            self.sim.dynamics_sim = None;
            self.sim.dynamics_last_instant = None;
        }
    }

    /// Drain instantaneous ops (Print, SaveCsv, SetGaitVelocity,
    /// SetPositionTarget) from the head of the MuJoCo async queue. Stops
    /// at the first `StepFrames` op (or empty queue). Caller is then
    /// expected to run the regular gait+WBC+step path; the `StepFrames`
    /// remaining count caps how many physics frames that path advances.
    #[cfg(feature = "mujoco")]
    fn drain_instantaneous_async_ops(&mut self) {
        use articara::mujoco_sim::AsyncSimOp;
        loop {
            let head_kind = match self.sim.mujoco_sim.as_ref().and_then(|s| s.async_peek()) {
                Some(AsyncSimOp::StepFrames(_)) => return,
                Some(AsyncSimOp::SetPositionTarget(idx, q)) => {
                    let (idx, q) = (*idx, *q);
                    if let Some(sim) = self.sim.mujoco_sim.as_mut() {
                        sim.async_pop();
                        sim.set_position_target(idx, q);
                    }
                    continue;
                }
                Some(AsyncSimOp::SetGaitVelocity(vx, vy, wz)) => {
                    let (vx, vy, wz) = (*vx, *vy, *wz);
                    if let Some(sim) = self.sim.mujoco_sim.as_mut() {
                        sim.async_pop();
                    }
                    if let Some(gc) = self.gait.controller.as_mut() {
                        gc.set_velocity_cmd(quadruped_gait::VelocityCmd {
                            vx,
                            vy,
                            wz,
                        });
                    }
                    continue;
                }
                Some(AsyncSimOp::Print(_)) => "print",
                Some(AsyncSimOp::SaveCsv(_)) => "save",
                None => return,
            };
            if head_kind == "print" {
                let msg = if let Some(sim) = self.sim.mujoco_sim.as_mut() {
                    if let Some(AsyncSimOp::Print(s)) = sim.async_pop() {
                        s
                    } else {
                        continue;
                    }
                } else {
                    return;
                };
                self.script_output.push(ScriptLine::System(msg));
            } else if head_kind == "save" {
                let path = if let Some(sim) = self.sim.mujoco_sim.as_mut() {
                    if let Some(AsyncSimOp::SaveCsv(p)) = sim.async_pop() {
                        p
                    } else {
                        continue;
                    }
                } else {
                    return;
                };
                let result = if let (Some(model), Some(sim)) =
                    (self.model.as_ref(), self.sim.mujoco_sim.as_ref())
                {
                    articara::mujoco_sim::save_peaks_csv(model, sim, &path)
                } else {
                    Err("simulation not active".to_string())
                };
                let line = match result {
                    Ok(n) => ScriptLine::System(format!(
                        "[async] saved {n} samples → {}",
                        path.display(),
                    )),
                    Err(e) => ScriptLine::Error(format!(
                        "[async] save_csv {}: {e}",
                        path.display(),
                    )),
                };
                self.script_output.push(line);
                self.script_scroll_to_bottom = true;
            }
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

        // --- Pre-export compatibility warning dialog ---
        // Drawn after Export so it floats on top when both are visible.
        self.draw_export_compat_dialog(ctx);

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

        // --- Open Script File dialog ---
        // Confirmed → store the chosen path on `pending_script_run`. The
        // console (drawn elsewhere this frame) is what actually reads + runs
        // it, since that's the only place with access to the live engine
        // and the input/output buffers.
        match self.dlg_open_script.show(ctx) {
            FileDialogResult::Confirmed(path) => {
                self.script_path = path.display().to_string();
                self.pending_script_run = Some(path);
            }
            _ => {}
        }

        // --- Save Peaks Plot CSV dialog ---
        #[cfg(feature = "mujoco")]
        match self.peaks_plot.dlg_save_csv.show(ctx) {
            FileDialogResult::Confirmed(path) => {
                self.peaks_plot.csv_path = path.display().to_string();
                let result = if let (Some(model), Some(sim)) =
                    (self.model.as_ref(), self.sim.mujoco_sim.as_ref())
                {
                    peaks_plot_window::save_peaks_csv(model, sim, &path)
                } else {
                    Err("MuJoCo simulation is not running".to_string())
                };
                match result {
                    Ok(n) => {
                        self.status_message = format!(
                            "Saved {n} samples → {}",
                            path.display(),
                        );
                    }
                    Err(e) => {
                        self.status_message = format!("Save CSV error: {e}");
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
                let mesh = articara::robot::load_mesh(&path, None);
                if mesh.num_triangles() == 0 {
                    self.status_message = format!(
                        "メッシュ読み込み失敗: {}",
                        path.display()
                    );
                } else if let Some(target) = self.add_mesh_target.take() {
                    let tri_count = mesh.num_triangles();
                    let fname = path.display().to_string();
                    if let Some(ref mut model) = self.model {
                        if target.link_index < model.links.len() {
                            let link = &mut model.links[target.link_index];
                            let geom = articara::robot::GeomData::Mesh {
                                mesh,
                                filename: Some(fname.clone()),
                                scale: None,
                            };
                            match target.kind {
                                MeshAddKind::Visual => {
                                    link.visuals.push(articara::robot::VisualData {
                                        origin: nalgebra::Isometry3::identity(),
                                        geometry: geom,
                                        color: [0.7, 0.7, 0.7, 1.0],
                                    });
                                    self.status_message = format!(
                                        "Visual メッシュ追加 ({tri_count} tris) ← {fname}"
                                    );
                                }
                                MeshAddKind::Collision => {
                                    link.collisions.push(articara::robot::CollisionData {
                                        origin: nalgebra::Isometry3::identity(),
                                        geometry: geom,
                                    
                                        physics: None,
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
mod camera_panel;
mod dynamics_panel;
mod gait_panel;
mod posture;
mod file_dialog;
mod status_bar;
mod script_console;
#[cfg(feature = "mujoco")]
mod peaks_plot_window;
#[cfg(feature = "mujoco")]
mod sim_drag;
mod actuator_dialog;
mod collision_matrix;
mod misa_report_dialog;
#[cfg(feature = "mujoco")]
mod mujoco_warning_dialog;

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

        // --- Refresh TPS camera from the current body pose ---
        // Always done (regardless of which mode is "main") so the wipe
        // can render the opposite camera without a one-frame lag and
        // toggling between modes is instantaneous.
        self.refresh_tps_camera();
        // When TPS is the active mode, mirror the live tps_camera into
        // the main self.camera so screen-space projections, picking, and
        // overlays all use the TPS view.
        if matches!(self.camera_mode, articara::camera::CameraMode::Tps) {
            self.camera = self.tps_camera.clone();
        }

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
                    self.draw_camera_panel(ui);
                    ui.separator();
                    self.draw_gait_panel(ui);
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

        // --- Collision pair matrix dialog ---
        self.draw_collision_matrix_window(&ctx);
        self.draw_actuator_dialog(&ctx);
        self.draw_misa_report_dialog(&ctx);
        #[cfg(feature = "mujoco")]
        self.draw_mujoco_warning_dialog(&ctx);

        // --- File dialogs ---
        self.process_file_dialogs(&ctx);

        // --- Poll background decomposition task ---
        self.poll_decompose_task();

        // --- Draw decomposition progress overlay ---
        self.draw_decompose_progress(&ctx);

        // ── Kinematic playback ──────────────────────────────────
        // Drive the model's joint angles and trunk pose directly from
        // the gait planner each frame, with no MuJoCo round-trip. The
        // planner's `body_state` integrator owns the trunk's world
        // pose; we compose it with `kinematic_playback_base_offset`
        // (= the trunk's pose at the moment playback was toggled on)
        // so the robot animates from wherever the user placed it.
        if self.kinematic_playback_active {
            if let (Some(model), Some(gc)) =
                (self.model.as_mut(), self.gait.controller.as_mut())
            {
                let dt = ctx.input(|i| i.stable_dt).max(1e-4) as f64;
                let dt = dt.min(0.05); // cap so huge frame stalls don't fast-forward
                let (out, targets, _ff) = gc.tick(dt);
                for (idx, q) in targets {
                    if let Some(slot) = model.joint_positions.get_mut(idx) {
                        *slot = q;
                    }
                }
                // Compose trunk pose: `offset · body_state_iso`. The
                // body-state integrator only updates X/Y/yaw, so Z and
                // roll/pitch of the offset survive untouched.
                let body_iso = na::Isometry3::from_parts(
                    na::Translation3::new(
                        out.body_state.world_position.x,
                        out.body_state.world_position.y,
                        0.0,
                    ),
                    na::UnitQuaternion::from_euler_angles(
                        0.0,
                        0.0,
                        out.body_state.world_yaw,
                    ),
                );
                model.base_transform =
                    self.kinematic_playback_base_offset * body_iso;
                model.rebuild_misarta_model();
                self.needs_upload = true;
            }
        }

        // ── Live gait feed (Zenoh) ──────────────────────────────
        // A small window toggles a subscriber to a `go2-gait-runner --viz`
        // stream; when running, each received GaitVizFrame drives the loaded
        // model's joints + trunk pose (same path as kinematic playback).
        #[cfg(feature = "viz")]
        {
            egui::Window::new("Live feed (Zenoh)")
                .default_open(false)
                .show(&ctx, |ui| {
                    ui.horizontal(|ui| {
                        let mut on = self.viz.active();
                        if ui.checkbox(&mut on, "Subscribe").changed() {
                            self.viz.toggle();
                        }
                        ui.add_enabled(
                            !self.viz.active(),
                            egui::TextEdit::singleline(&mut self.viz.key)
                                .hint_text("key")
                                .desired_width(170.0),
                        );
                    });
                    ui.horizontal(|ui| {
                        ui.label("endpoint:");
                        ui.add_enabled(
                            !self.viz.active(),
                            egui::TextEdit::singleline(&mut self.viz.endpoint)
                                .hint_text("auto (tcp/127.0.0.1:7447 for same PC)")
                                .desired_width(220.0),
                        );
                    });
                    if self.viz.active() {
                        match self.viz.last_seq {
                            Some(s) => ui.label(format!("● receiving — frame #{s}")),
                            None => ui.label("● subscribed — waiting for frames…"),
                        };
                    } else {
                        ui.label("off — run: go2-gait-runner run eth0 --viz");
                    }
                });
            if self.viz.active() {
                if let Some(model) = self.model.as_mut() {
                    if self.viz.apply(model) {
                        self.needs_upload = true;
                    }
                }
                // Keep repainting so newly arrived frames are applied promptly.
                ctx.request_repaint();
            }
        }

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
    /// Queue a Rhai script to auto-run on the next frame, opening
    /// the Script Console panel so the run is visible. Used by the
    /// `--script <path>` CLI flag to fire a startup script with no
    /// manual GUI clicks (= reproducible benchmark / demo runs).
    ///
    /// The script file is read inside the console's normal "📂 run
    /// from file" branch, so the same echo / output / error UX
    /// applies as if the user had clicked the button.
    #[cfg(feature = "scripting")]
    pub fn queue_initial_script(&mut self, path: std::path::PathBuf) {
        self.show_script_console = true;
        self.pending_script_run = Some(path);
    }

    /// Apply pending [`articara::scripting_model::ScriptOverrides`] from
    /// the most recent script run. Called once after the engine eval
    /// finishes — each `Some(_)` field maps to the corresponding
    /// [`ArticaraApp`] field, and the override struct's contents are
    /// consumed (`drain_overrides` already moved them out).
    #[cfg(feature = "scripting")]
    fn apply_script_overrides(
        &mut self,
        ov: articara::scripting_model::ScriptOverrides,
    ) {
        if let Some(mode) = ov.gait_mode {
            self.gait.mode = mode;
            // Re-build the gait controller in the new mode if one is
            // already active (so the change isn't quietly deferred to
            // the next mj_start).
            if let Some(gc) = self.gait.controller.as_mut() {
                gc.set_mode(mode);
            }
        }
        if let Some(k) = ov.capture_point_gain {
            if let Some(gc) = self.gait.controller.as_mut() {
                gc.set_capture_point_gain(k);
            }
        }
        #[cfg(feature = "mujoco")]
        {
            if let Some(src) = ov.pose_source {
                self.sim.pose_source = src;
            }
            if let Some(on) = ov.wbc_enabled {
                self.sim.wbc_enabled = on;
            }
        }
        #[cfg(not(feature = "mujoco"))]
        {
            let _ = (ov.pose_source, ov.wbc_enabled);
        }
        if let Some(on) = ov.ground_plane_enabled {
            self.show_ground_plane = on;
        }
        if let Some(z) = ov.ground_plane_z {
            self.ground_z = z;
        }
        if let Some(p) = ov.ground_plane_pitch {
            self.ground_plane_pitch = p;
        }
        if let Some(r) = ov.ground_plane_roll {
            self.ground_plane_roll = r;
        }
    }

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
