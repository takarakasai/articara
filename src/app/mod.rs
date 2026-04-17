use eframe::egui;
use nalgebra as na;
use std::collections::HashMap;
use std::path::PathBuf;
use std::sync::{Arc, Mutex};

use crate::camera::OrbitCamera;
use crate::dynamics;
use crate::format::RobotFormat;
use crate::history::History;
use crate::ik;
use crate::renderer::{DisplayMode, GlRenderer, MeshKind};
use crate::robot::RobotModel;

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
    /// The kinematic chain from root to end-effector.
    chain: Vec<ik::ChainJoint>,
    /// The IK root link name (for base correction). None = URDF root.
    ik_root_link: Option<String>,
    /// World-space transform of the IK root link at drag start.
    /// Used as the fixed anchor so the root doesn't drift.
    ik_root_initial_tf: Option<na::Isometry3<f32>>,
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
    /// IK damping factor (λ for DLS).
    ik_damping: f32,
    /// IK root link name. None = use URDF root (full chain).
    ik_root_link: Option<String>,
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
    /// Body link (the torso/base that gets launched) for jump estimation.
    dynamics_body_link: Option<String>,
    /// Selected ground-contact links for jump height estimation.
    dynamics_ground_links: Vec<String>,
    /// Cached dynamics analysis result.
    dynamics_result: Option<dynamics::StaticAnalysis>,
    /// Active dynamics simulation (jump or payload).
    dynamics_sim: Option<dynamics::DynSim>,
    /// Simulation playback speed.
    dynamics_sim_speed: f32,
    /// Whether the simulation is paused.
    dynamics_sim_paused: bool,
    /// When `Some(dt)`, advance by exactly `dt` seconds then re-pause.
    dynamics_step_dt: Option<f32>,
    /// Last frame instant for delta-time calculation.
    dynamics_last_instant: Option<std::time::Instant>,
    /// Which axes the body link can move during flight (true = free).
    dynamics_launch_axes: [bool; 3],
    /// Joint names that are locked (not driven) during jump sim.
    dynamics_locked_joints: std::collections::HashSet<String>,
    /// User-specified extension duration override (None = auto-compute).
    dynamics_extension_duration: Option<f32>,
    /// Whether to enforce URDF effort (torque) limits during jump sim.
    dynamics_enforce_torque_limits: bool,
    /// Whether to retract (pull legs back) after extension for extra hang time.
    dynamics_enable_retract: bool,
    /// PD position gain Kp (N·m/rad) for computed-torque controller.
    dynamics_pd_kp: f64,
    /// PD derivative gain Kd (N·m·s/rad) for computed-torque controller.
    dynamics_pd_kd: f64,
    /// Last jump simulation result (displayed after sim ends).
    dynamics_sim_result: Option<dynamics::JumpSimResult>,
    /// Show the sim result dialog window.
    show_sim_result_window: bool,
    /// Link to track in the dynamics graph (position/velocity/acceleration).
    dynamics_graph_link: Option<String>,
    /// Whether to show the dynamics graph window.
    show_dynamics_graph: bool,
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
            ik_root_link: None,
            show_com: false,
            show_joint_axes: false,
            show_ground_plane: false,
            ground_z: 0.0,
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
            dynamics_body_link: None,
            dynamics_ground_links: Vec::new(),
            dynamics_result: None,
            dynamics_sim: None,
            dynamics_sim_speed: 1.0,
            dynamics_sim_paused: false,
            dynamics_step_dt: None,
            dynamics_last_instant: None,
            dynamics_launch_axes: [false, false, true], // Z-only by default
            dynamics_locked_joints: std::collections::HashSet::new(),
            dynamics_extension_duration: None,
            dynamics_enforce_torque_limits: false,
            dynamics_enable_retract: false,
            dynamics_pd_kp: 500.0,
            dynamics_pd_kd: 20.0,
            dynamics_sim_result: None,
            show_sim_result_window: false,
            dynamics_graph_link: None,
            show_dynamics_graph: false,
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
        let sim = match self.dynamics_sim.as_mut() {
            Some(s) => s,
            None => {
                self.dynamics_last_instant = None;
                return;
            }
        };

        // Handle pause / step-once
        if self.dynamics_sim_paused && self.dynamics_step_dt.is_none() {
            // Still paused — skip physics but keep last_instant fresh
            self.dynamics_last_instant = Some(std::time::Instant::now());
            return;
        }

        // Compute delta-time
        let now = std::time::Instant::now();
        let dt = if let Some(step_dt) = self.dynamics_step_dt.take() {
            // Fixed step requested — use exact dt, then re-pause
            self.dynamics_sim_paused = true;
            self.dynamics_last_instant = Some(now);
            step_dt
        } else {
            let d = match self.dynamics_last_instant {
                Some(prev) => now.duration_since(prev).as_secs_f32().min(0.05),
                None => 0.016,
            };
            self.dynamics_last_instant = Some(now);
            d
        };

        let still_running = match sim {
            dynamics::DynSim::Jump(js) => {
                // Auto-enable ground plane at foot level during jump sim
                if !self.ground_plane_auto {
                    self.ground_plane_auto = true;
                    self.show_ground_plane = true;
                    self.ground_z = js.initial_foot_z;
                }
                if let Some(ref mut model) = self.model {
                    dynamics::step_jump_sim(js, model, dt)
                } else {
                    false
                }
            }
            dynamics::DynSim::Payload(ps) => {
                ps.phase_time += dt;
                if let Some(ref model) = self.model {
                    let ee = self.dynamics_ee_link.as_deref().unwrap_or("");
                    dynamics::step_payload_sim(ps, model, ee)
                } else {
                    false
                }
            }
        };

        if !still_running {
            // Capture jump sim result before clearing
            if let Some(dynamics::DynSim::Jump(ref js)) = self.dynamics_sim {
                if let Some(ref model) = self.model {
                    self.dynamics_sim_result = Some(dynamics::extract_jump_result(js, model));
                    self.show_sim_result_window = true;
                }
            }
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
            .height_range(28.0..=28.0)
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

        // --- File dialogs ---
        self.process_file_dialogs(&ctx);

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

