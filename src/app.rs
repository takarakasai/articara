use eframe::egui;
use nalgebra as na;
use std::collections::HashMap;
use std::path::PathBuf;
use std::sync::{Arc, Mutex};

use crate::camera::OrbitCamera;
use crate::format::RobotFormat;
use crate::ik;
use crate::renderer::{DisplayMode, GlRenderer, MeshKind};
use crate::robot::{self, GeomData, RobotModel};

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
            com_scale: 0.01,
            wireframe: false,
            visual_mode: DisplayMode::Solid,
            collision_mode: DisplayMode::Off,
            link_display_modes: HashMap::new(),
            export_dir: String::new(),
            export_format: RobotFormat::Urdf,
            export_message: String::new(),
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

    // ===== UI Panels =====

    fn draw_menu_bar(&mut self, ui: &mut egui::Ui) {
        ui.horizontal(|ui| {
            if ui.button("📄 New").clicked() {
                self.model = Some(RobotModel::new_empty("new_robot"));
                self.selected_link = None;
                self.selected_joint = None;
                self.needs_upload = true;
                self.status_message = "Created new empty model".into();
            }
            ui.separator();

            // ===== Edit menu =====
            ui.menu_button("Edit", |ui| {
                ui.menu_button("Mode", |ui| {
                    let jd = self.interaction_mode == InteractionMode::JointDrive;
                    let oa = self.interaction_mode == InteractionMode::OffsetAdjust;
                    if ui.selectable_label(jd, "🔧 Joint Drive").clicked() {
                        self.interaction_mode = InteractionMode::JointDrive;
                        ui.close();
                    }
                    if ui.selectable_label(oa, "✥ Offset Adjust").clicked() {
                        self.interaction_mode = InteractionMode::OffsetAdjust;
                        ui.close();
                    }
                });
                if self.interaction_mode == InteractionMode::OffsetAdjust {
                    ui.menu_button("Offset Target", |ui| {
                        for t in OffsetTarget::ALL {
                            let sel = self.offset_target == t;
                            let label = format!("{} {}", t.icon(), t.label());
                            if ui.selectable_label(sel, label).clicked() {
                                self.offset_target = t;
                                if t == OffsetTarget::Joint && self.gizmo_op == GizmoOp::Scale {
                                    self.gizmo_op = GizmoOp::Translate;
                                }
                                ui.close();
                            }
                        }
                    });
                }
            });
            ui.separator();

            ui.label("File:");
            let response = ui.text_edit_singleline(&mut self.urdf_path_input);
            if (response.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Enter)))
                || ui.button("Load").clicked()
            {
                let path = PathBuf::from(&self.urdf_path_input);
                self.load_model(path);
            }
            ui.separator();
            ui.label(&self.status_message);
        });
    }

    fn draw_tree_panel(&mut self, ui: &mut egui::Ui) {
        if self.model.is_none() {
            ui.label("No model loaded.");
            ui.label("Enter a URDF path above and click Load,");
            ui.label("or click 📄 New to create a new model.");
            if ui.button("📄 New Model").clicked() {
                self.model = Some(RobotModel::new_empty("new_robot"));
                self.selected_link = None;
                self.selected_joint = None;
                self.needs_upload = true;
                self.status_message = "Created new empty model".into();
            }
            return;
        }

        let model_name = self.model.as_ref().unwrap().name.clone();
        let root_link = self.model.as_ref().unwrap().root_link.clone();

        ui.heading(&model_name);
        ui.separator();

        // Hierarchical link tree
        ui.collapsing("🔗 Links", |ui| {
            self.draw_link_tree(ui, &root_link);
        });
        // Clear auto-expand after drawing
        self.tree_reveal_ancestors.clear();

        ui.separator();

        // Joint list
        let joint_info: Vec<(String, String)> = self
            .model
            .as_ref()
            .unwrap()
            .joints
            .iter()
            .map(|j| (j.name.clone(), j.joint_type.clone()))
            .collect();
        let joint_count = joint_info.len();
        ui.collapsing(format!("⚙ Joints ({joint_count})"), |ui| {
            for (i, (name, jtype)) in joint_info.iter().enumerate() {
                let selected = self.selected_joint == Some(i);
                let label = format!("{name} [{jtype}]");
                if ui.selectable_label(selected, &label).clicked() {
                    self.selected_joint = Some(i);
                    self.selected_link = None;
                }
            }
        });

        ui.separator();

        // --- Add Child (Link + Joint) section ---
        self.draw_add_child_panel(ui);

        // --- Remove selected link ---
        if let Some(li) = self.selected_link {
            let link_name = self.model.as_ref().unwrap().links[li].name.clone();
            let is_root = link_name == self.model.as_ref().unwrap().root_link;
            ui.add_enabled_ui(!is_root, |ui| {
                if ui.button("🗑 Remove Selected Link").clicked() {
                    if let Some(model) = &mut self.model {
                        match model.remove_link(&link_name) {
                            Ok(removed) => {
                                self.status_message =
                                    format!("Removed {} link(s): {}", removed.len(), removed.join(", "));
                                self.selected_link = None;
                                self.selected_joint = None;
                                self.needs_upload = true;
                            }
                            Err(e) => {
                                self.status_message = format!("Remove error: {e}");
                            }
                        }
                    }
                }
            });
        }
    }

    fn draw_link_tree(&mut self, ui: &mut egui::Ui, link_name: &str) {
        let model = self.model.as_ref().unwrap();
        let link_idx = model.link_map.get(link_name).copied();
        let selected = link_idx.is_some() && self.selected_link == link_idx;

        // Collect child info before creating UI to avoid borrow conflicts
        let children: Vec<(String, String)> = model
            .children_joints
            .get(link_name)
            .map(|joints| {
                joints
                    .iter()
                    .map(|&ji| {
                        let j = &model.joints[ji];
                        (j.name.clone(), j.child_link.clone())
                    })
                    .collect()
            })
            .unwrap_or_default();

        if children.is_empty() {
            // Leaf node
            if ui.selectable_label(selected, link_name).clicked() {
                self.selected_link = link_idx;
                self.selected_joint = None;
            }
        } else {
            // Branch node
            let id = ui.make_persistent_id(link_name);
            let mut state = egui::collapsing_header::CollapsingState::load_with_default_open(
                ui.ctx(), id, true,
            );
            // Auto-expand if this link is an ancestor of the viewport-selected link
            if self.tree_reveal_ancestors.iter().any(|a| a == link_name) {
                state.set_open(true);
            }
            state
                .show_header(ui, |ui| {
                    if ui.selectable_label(selected, link_name).clicked() {
                        self.selected_link = link_idx;
                        self.selected_joint = None;
                    }
                })
                .body(|ui| {
                    for (_joint_name, child_link) in &children {
                        self.draw_link_tree(ui, child_link);
                    }
                });
        }
    }

    fn draw_add_child_panel(&mut self, ui: &mut egui::Ui) {
        const JOINT_TYPES: &[&str] = &["revolute", "prismatic", "fixed", "continuous"];
        const GEOM_TYPES: &[&str] = &["Box", "Cylinder", "Sphere"];

        // Toggle button
        let toggle_label = if self.show_add_child {
            "▼ Add Child Link+Joint"
        } else {
            "➕ Add Child Link+Joint"
        };
        if ui.button(toggle_label).clicked() {
            self.show_add_child = !self.show_add_child;
            // Auto-fill parent from selected link
            if self.show_add_child {
                if let (Some(li), Some(model)) = (self.selected_link, self.model.as_ref()) {
                    self.new_parent_link = model.links[li].name.clone();
                } else if let Some(model) = self.model.as_ref() {
                    self.new_parent_link = model.root_link.clone();
                }
                // Auto-generate names
                if let Some(model) = self.model.as_ref() {
                    self.new_link_name = model.generate_link_name("new_link");
                    self.new_joint_name = model.generate_joint_name("new_joint");
                }
            }
        }

        if !self.show_add_child {
            return;
        }

        ui.group(|ui| {
            ui.set_width(ui.available_width());

            // Parent link combo
            let link_names: Vec<String> = self
                .model
                .as_ref()
                .map(|m| m.link_names())
                .unwrap_or_default();
            ui.horizontal(|ui| {
                ui.label("Parent:");
                egui::ComboBox::from_id_salt("add_parent_combo")
                    .selected_text(&self.new_parent_link)
                    .show_ui(ui, |ui| {
                        for name in &link_names {
                            ui.selectable_value(&mut self.new_parent_link, name.clone(), name);
                        }
                    });
            });

            // Link name
            ui.horizontal(|ui| {
                ui.label("Link name:");
                ui.text_edit_singleline(&mut self.new_link_name);
            });

            // Joint name
            ui.horizontal(|ui| {
                ui.label("Joint name:");
                ui.text_edit_singleline(&mut self.new_joint_name);
            });

            // Joint type
            ui.horizontal(|ui| {
                ui.label("Joint type:");
                egui::ComboBox::from_id_salt("add_joint_type")
                    .selected_text(JOINT_TYPES[self.new_joint_type_idx])
                    .show_ui(ui, |ui| {
                        for (i, &jt) in JOINT_TYPES.iter().enumerate() {
                            ui.selectable_value(&mut self.new_joint_type_idx, i, jt);
                        }
                    });
            });

            // Geometry type
            ui.horizontal(|ui| {
                ui.label("Geometry:");
                egui::ComboBox::from_id_salt("add_geom_type")
                    .selected_text(GEOM_TYPES[self.new_geom_type_idx])
                    .show_ui(ui, |ui| {
                        for (i, &gt) in GEOM_TYPES.iter().enumerate() {
                            ui.selectable_value(&mut self.new_geom_type_idx, i, gt);
                        }
                    });
            });

            // Geometry size
            match self.new_geom_type_idx {
                0 => {
                    // Box: hx, hy, hz
                    ui.horizontal(|ui| {
                        ui.label("Size XYZ:");
                        ui.add(egui::DragValue::new(&mut self.new_geom_size[0]).speed(0.005).prefix("x:"));
                        ui.add(egui::DragValue::new(&mut self.new_geom_size[1]).speed(0.005).prefix("y:"));
                        ui.add(egui::DragValue::new(&mut self.new_geom_size[2]).speed(0.005).prefix("z:"));
                    });
                }
                1 => {
                    // Cylinder: radius, length
                    ui.horizontal(|ui| {
                        ui.label("Cyl:");
                        ui.add(egui::DragValue::new(&mut self.new_geom_size[0]).speed(0.005).prefix("r:"));
                        ui.add(egui::DragValue::new(&mut self.new_geom_size[1]).speed(0.005).prefix("l:"));
                    });
                }
                2 => {
                    // Sphere: radius
                    ui.horizontal(|ui| {
                        ui.label("Radius:");
                        ui.add(egui::DragValue::new(&mut self.new_geom_size[0]).speed(0.005));
                    });
                }
                _ => {}
            }

            // Joint origin
            ui.horizontal(|ui| {
                ui.label("Origin XYZ:");
                ui.add(egui::DragValue::new(&mut self.new_joint_origin[0]).speed(0.005).prefix("x:"));
                ui.add(egui::DragValue::new(&mut self.new_joint_origin[1]).speed(0.005).prefix("y:"));
                ui.add(egui::DragValue::new(&mut self.new_joint_origin[2]).speed(0.005).prefix("z:"));
            });

            // Joint axis
            ui.horizontal(|ui| {
                ui.label("Axis:");
                ui.add(egui::DragValue::new(&mut self.new_joint_axis[0]).speed(0.01).prefix("x:"));
                ui.add(egui::DragValue::new(&mut self.new_joint_axis[1]).speed(0.01).prefix("y:"));
                ui.add(egui::DragValue::new(&mut self.new_joint_axis[2]).speed(0.01).prefix("z:"));
            });

            // Joint limits (only for revolute / prismatic)
            if self.new_joint_type_idx < 2 {
                ui.horizontal(|ui| {
                    ui.label("Limits:");
                    ui.add(
                        egui::DragValue::new(&mut self.new_joint_limits[0])
                            .speed(0.01)
                            .prefix("lo:"),
                    );
                    ui.add(
                        egui::DragValue::new(&mut self.new_joint_limits[1])
                            .speed(0.01)
                            .prefix("hi:"),
                    );
                });
            }

            // Color
            ui.horizontal(|ui| {
                ui.label("Color:");
                ui.color_edit_button_rgb(&mut self.new_link_color);
            });

            // "Add" button
            let can_add = !self.new_link_name.is_empty()
                && !self.new_joint_name.is_empty()
                && !self.new_parent_link.is_empty();
            ui.add_enabled_ui(can_add, |ui| {
                if ui.button("✔ Add").clicked() {
                    self.execute_add_child();
                }
            });
        });
    }

    /// Actually execute adding a child link + joint.
    fn execute_add_child(&mut self) {
        const JOINT_TYPES: &[&str] = &["revolute", "prismatic", "fixed", "continuous"];

        let geom = match self.new_geom_type_idx {
            0 => GeomData::Box {
                hx: self.new_geom_size[0],
                hy: self.new_geom_size[1],
                hz: self.new_geom_size[2],
            },
            1 => GeomData::Cylinder {
                radius: self.new_geom_size[0],
                half_length: self.new_geom_size[1] * 0.5,
            },
            2 => GeomData::Sphere {
                radius: self.new_geom_size[0],
            },
            _ => GeomData::Box {
                hx: 0.05,
                hy: 0.05,
                hz: 0.05,
            },
        };

        let color = [
            self.new_link_color[0],
            self.new_link_color[1],
            self.new_link_color[2],
            1.0,
        ];

        let origin = na::Isometry3::new(
            na::Vector3::new(
                self.new_joint_origin[0],
                self.new_joint_origin[1],
                self.new_joint_origin[2],
            ),
            na::Vector3::zeros(),
        );

        let axis = na::Vector3::new(
            self.new_joint_axis[0],
            self.new_joint_axis[1],
            self.new_joint_axis[2],
        );

        let jtype = JOINT_TYPES[self.new_joint_type_idx];
        let (lower, upper) = if self.new_joint_type_idx < 2 {
            (self.new_joint_limits[0] as f64, self.new_joint_limits[1] as f64)
        } else {
            (0.0, 0.0)
        };

        if let Some(model) = &mut self.model {
            match model.add_child(
                &self.new_parent_link,
                &self.new_link_name,
                &self.new_joint_name,
                jtype,
                origin,
                axis,
                geom,
                color,
                lower,
                upper,
            ) {
                Ok((li, _ji)) => {
                    self.status_message = format!(
                        "Added link '{}' + joint '{}'",
                        self.new_link_name, self.new_joint_name
                    );
                    self.selected_link = Some(li);
                    self.selected_joint = None;
                    self.needs_upload = true;
                    self.show_add_child = false;
                    // Prepare next auto-names
                    self.new_link_name = model.generate_link_name("new_link");
                    self.new_joint_name = model.generate_joint_name("new_joint");
                }
                Err(e) => {
                    self.status_message = format!("Add error: {e}");
                }
            }
        }
    }

    fn draw_joint_sliders(&mut self, ui: &mut egui::Ui) {
        if let Some(model) = &mut self.model {
            let mut changed = false;
            ui.heading("Joint Positions");
            ui.separator();

            // --- Interaction Mode (display current) ---
            ui.horizontal(|ui| {
                let mode_label = match self.interaction_mode {
                    InteractionMode::JointDrive => "🔧 Joint Drive",
                    InteractionMode::OffsetAdjust => "✥ Offset Adjust",
                };
                ui.label(format!("Mode: {mode_label}"));
            });

            if self.interaction_mode == InteractionMode::OffsetAdjust {
                // --- Offset Target selector ---
                ui.horizontal(|ui| {
                    ui.label("Target:");
                    let prev_target = self.offset_target;
                    for t in OffsetTarget::ALL {
                        ui.radio_value(&mut self.offset_target, t, t.label());
                    }
                    if self.offset_target == OffsetTarget::Joint
                        && prev_target != OffsetTarget::Joint
                        && self.gizmo_op == GizmoOp::Scale
                    {
                        self.gizmo_op = GizmoOp::Translate;
                    }
                });
                // --- Gizmo operation selector ---
                ui.horizontal(|ui| {
                    ui.label("Gizmo:");
                    ui.radio_value(&mut self.gizmo_op, GizmoOp::Translate, "⬌ Translate");
                    ui.radio_value(&mut self.gizmo_op, GizmoOp::Rotate, "↻ Rotate");
                    // Scale only for Visual/Collision
                    if self.offset_target != OffsetTarget::Joint {
                        ui.radio_value(&mut self.gizmo_op, GizmoOp::Scale, "⬡ Scale");
                    }
                });
                // Show which element is selected
                match self.offset_target {
                    OffsetTarget::Joint => {
                        if let Some(ji) = self.selected_joint {
                            ui.label(format!("  → {}", model.joints[ji].name));
                        } else {
                            ui.label("  (click a link to select its joint)");
                        }
                    }
                    OffsetTarget::Visual => {
                        if let Some(li) = self.selected_link {
                            let n_vis = model.links[li].visuals.len();
                            if n_vis == 0 {
                                ui.label(format!("  {} has no visuals", model.links[li].name));
                            } else {
                                ui.horizontal(|ui| {
                                    ui.label("  →");
                                    for vi in 0..n_vis {
                                        let sel = self.selected_visual == Some(vi);
                                        if ui.selectable_label(sel, format!("V{vi}")).clicked() {
                                            self.selected_visual = Some(vi);
                                        }
                                    }
                                });
                            }
                        } else {
                            ui.label("  (click a link to select)");
                        }
                    }
                    OffsetTarget::Collision => {
                        if let Some(li) = self.selected_link {
                            let n_col = model.links[li].collisions.len();
                            if n_col == 0 {
                                ui.label(format!("  {} has no collisions", model.links[li].name));
                            } else {
                                ui.horizontal(|ui| {
                                    ui.label("  →");
                                    for ci in 0..n_col {
                                        let sel = self.selected_collision == Some(ci);
                                        if ui.selectable_label(sel, format!("C{ci}")).clicked() {
                                            self.selected_collision = Some(ci);
                                        }
                                    }
                                });
                            }
                        } else {
                            ui.label("  (click a link to select)");
                        }
                    }
                }
                ui.separator();
            }

            if self.interaction_mode == InteractionMode::JointDrive {
                // --- Joint Drive sub-mode selector ---
                ui.horizontal(|ui| {
                    ui.label("Drag mode:");
                });
                ui.horizontal(|ui| {
                    ui.radio_value(&mut self.drag_mode, DragMode::SingleJoint, "Single Joint");
                    ui.radio_value(
                        &mut self.drag_mode,
                        DragMode::InverseKinematics,
                        "IK Chain",
                    );
                });
                if self.drag_mode == DragMode::InverseKinematics {
                    ui.horizontal(|ui| {
                        ui.label("IK Damping (λ):");
                        ui.add(
                            egui::Slider::new(&mut self.ik_damping, 0.001..=0.5)
                                .logarithmic(true)
                                .text("λ"),
                        );
                    });
                    // IK root link selector
                    let link_names: Vec<String> = model
                        .links
                        .iter()
                        .map(|l| l.name.clone())
                        .collect();
                    let prev_ik_root = self.ik_root_link.clone();
                    ui.horizontal(|ui| {
                        ui.label("IK Root:");
                        let current_label = match &self.ik_root_link {
                            None => "Auto (URDF Root)".to_string(),
                            Some(name) => name.clone(),
                        };
                        egui::ComboBox::from_id_salt("ik_root_link")
                            .selected_text(&current_label)
                            .show_ui(ui, |ui| {
                                // "Auto" option — use full chain to URDF root
                                if ui
                                    .selectable_label(
                                        self.ik_root_link.is_none(),
                                        "Auto (URDF Root)",
                                    )
                                    .clicked()
                                {
                                    self.ik_root_link = None;
                                }
                                // List all links as possible roots
                                for name in &link_names {
                                    let selected = self.ik_root_link.as_deref()
                                        == Some(name.as_str());
                                    if ui
                                        .selectable_label(selected, name)
                                        .clicked()
                                    {
                                        self.ik_root_link =
                                            Some(name.clone());
                                    }
                                }
                            });
                    });
                    // Reset base_transform when IK root changes
                    if self.ik_root_link != prev_ik_root {
                        model.base_transform = na::Isometry3::identity();
                    }
                }
            }
            ui.separator();

            // --- Display options ---
            ui.heading("Display");
            // Global visual mode
            ui.horizontal(|ui| {
                ui.label("Visual:");
                egui::ComboBox::from_id_salt("global_visual_mode")
                    .selected_text(self.visual_mode.label())
                    .show_ui(ui, |ui| {
                        for m in DisplayMode::ALL {
                            ui.selectable_value(&mut self.visual_mode, m, m.label());
                        }
                    });
            });
            // Global collision mode
            ui.horizontal(|ui| {
                ui.label("Collision:");
                egui::ComboBox::from_id_salt("global_collision_mode")
                    .selected_text(self.collision_mode.label())
                    .show_ui(ui, |ui| {
                        for m in DisplayMode::ALL {
                            ui.selectable_value(&mut self.collision_mode, m, m.label());
                        }
                    });
            });
            ui.checkbox(&mut self.show_com, "Show CoM & Mass");
            if self.show_com {
                ui.horizontal(|ui| {
                    ui.label("CoM scale:");
                    ui.add(
                        egui::Slider::new(&mut self.com_scale, 0.001..=0.1)
                            .logarithmic(true)
                            .text("m/kg"),
                    );
                });
            }
            ui.separator();

            for i in 0..model.joints.len() {
                if model.joints[i].joint_type == "fixed" {
                    continue;
                }
                let lower = model.joints[i].lower as f32;
                let upper = model.joints[i].upper as f32;
                if lower >= upper {
                    continue;
                }
                let name = model.joints[i].name.clone();
                ui.horizontal(|ui| {
                    ui.set_min_width(200.0);
                    ui.label(&name);
                });
                if ui
                    .add(
                        egui::Slider::new(&mut model.joint_positions[i], lower..=upper)
                            .step_by(0.01)
                            .text("rad"),
                    )
                    .changed()
                {
                    changed = true;
                }
            }

            // Joint slider changes only affect transforms, not geometry.
            // Do NOT touch needs_upload here — it may be true from add/remove operations.
            let _ = changed;

            if ui.button("Reset All Joints").clicked() {
                for pos in model.joint_positions.iter_mut() {
                    *pos = 0.0;
                }
            }
        }
    }

    fn draw_properties_panel(&mut self, ui: &mut egui::Ui) {
        ui.heading("Properties");
        ui.separator();

        if let Some(model) = &mut self.model {
            if let Some(li) = self.selected_link {
                let link = &mut model.links[li];
                let link_name = link.name.clone();
                ui.label(egui::RichText::new(&link_name).strong().size(16.0));
                ui.separator();

                // --- Per-link display mode controls ---
                ui.horizontal(|ui| {
                    ui.label("Visual:");
                    let key_v = (link_name.clone(), MeshKind::Visual);
                    let mut cur_v = self
                        .link_display_modes
                        .get(&key_v)
                        .copied()
                        .unwrap_or(self.visual_mode);
                    let prev_v = cur_v;
                    egui::ComboBox::from_id_salt(format!("link_vis_{li}"))
                        .width(80.0)
                        .selected_text(cur_v.label())
                        .show_ui(ui, |ui| {
                            for m in DisplayMode::ALL {
                                ui.selectable_value(&mut cur_v, m, m.label());
                            }
                        });
                    if cur_v != prev_v {
                        if cur_v == self.visual_mode {
                            self.link_display_modes.remove(&key_v);
                        } else {
                            self.link_display_modes.insert(key_v, cur_v);
                        }
                    }
                });
                ui.horizontal(|ui| {
                    ui.label("Collision:");
                    let key_c = (link_name.clone(), MeshKind::Collision);
                    let mut cur_c = self
                        .link_display_modes
                        .get(&key_c)
                        .copied()
                        .unwrap_or(self.collision_mode);
                    let prev_c = cur_c;
                    egui::ComboBox::from_id_salt(format!("link_col_{li}"))
                        .width(80.0)
                        .selected_text(cur_c.label())
                        .show_ui(ui, |ui| {
                            for m in DisplayMode::ALL {
                                ui.selectable_value(&mut cur_c, m, m.label());
                            }
                        });
                    if cur_c != prev_c {
                        if cur_c == self.collision_mode {
                            self.link_display_modes.remove(&key_c);
                        } else {
                            self.link_display_modes.insert(key_c, cur_c);
                        }
                    }
                });
                ui.separator();

                egui::CollapsingHeader::new("📐 Inertial")
                    .default_open(true)
                    .show(ui, |ui| {
                        egui::Grid::new("inertial_grid")
                            .striped(true)
                            .num_columns(2)
                            .show(ui, |ui| {
                                ui.label("Mass (kg):");
                                ui.add(egui::DragValue::new(&mut link.inertial.mass).speed(0.001).range(0.0..=f64::MAX));
                                ui.end_row();

                                ui.label("Origin xyz:");
                                ui.horizontal(|ui| {
                                    let t = &mut link.inertial.origin.translation;
                                    ui.add(egui::DragValue::new(&mut t.x).speed(0.0001).prefix("x:"));
                                    ui.add(egui::DragValue::new(&mut t.y).speed(0.0001).prefix("y:"));
                                    ui.add(egui::DragValue::new(&mut t.z).speed(0.0001).prefix("z:"));
                                });
                                ui.end_row();

                                ui.label("Ixx:");
                                ui.add(egui::DragValue::new(&mut link.inertial.ixx).speed(0.000001));
                                ui.end_row();
                                ui.label("Ixy:");
                                ui.add(egui::DragValue::new(&mut link.inertial.ixy).speed(0.000001));
                                ui.end_row();
                                ui.label("Ixz:");
                                ui.add(egui::DragValue::new(&mut link.inertial.ixz).speed(0.000001));
                                ui.end_row();
                                ui.label("Iyy:");
                                ui.add(egui::DragValue::new(&mut link.inertial.iyy).speed(0.000001));
                                ui.end_row();
                                ui.label("Iyz:");
                                ui.add(egui::DragValue::new(&mut link.inertial.iyz).speed(0.000001));
                                ui.end_row();
                                ui.label("Izz:");
                                ui.add(egui::DragValue::new(&mut link.inertial.izz).speed(0.000001));
                                ui.end_row();
                            });
                    });

                ui.add_space(8.0);
                let vis_count = link.visuals.len();
                let vis_header = egui::CollapsingHeader::new(format!(
                    "🎨 Visuals ({vis_count})"
                ))
                .default_open(true)
                .show(ui, |ui| {
                    let mut geom_changed = false;
                    let mut vis_to_remove: Option<usize> = None;
                    let mut vis_to_duplicate: Option<usize> = None;
                    for vi in 0..link.visuals.len() {
                        let vis = &mut link.visuals[vi];
                        ui.push_id(vi, |ui| {
                            let header_resp = ui.label(egui::RichText::new(format!("Visual #{vi}")).strong());
                            // Right-click on individual visual item
                            header_resp.context_menu(|ui| {
                                if ui.button("📋 Duplicate").clicked() {
                                    vis_to_duplicate = Some(vi);
                                    ui.close();
                                }
                                if ui.button("🗑 Delete").clicked() {
                                    vis_to_remove = Some(vi);
                                    ui.close();
                                }
                            });

                            // --- Geometry editing ---
                            match &mut vis.geometry {
                                GeomData::Box { hx, hy, hz } => {
                                    ui.label("Box (half-extents):");
                                    ui.horizontal(|ui| {
                                        geom_changed |= ui.add(egui::DragValue::new(hx).speed(0.005).prefix("hx:").range(0.001..=10.0)).changed();
                                        geom_changed |= ui.add(egui::DragValue::new(hy).speed(0.005).prefix("hy:").range(0.001..=10.0)).changed();
                                        geom_changed |= ui.add(egui::DragValue::new(hz).speed(0.005).prefix("hz:").range(0.001..=10.0)).changed();
                                    });
                                }
                                GeomData::Cylinder { radius, half_length } => {
                                    ui.label("Cylinder:");
                                    ui.horizontal(|ui| {
                                        geom_changed |= ui.add(egui::DragValue::new(radius).speed(0.005).prefix("r:").range(0.001..=10.0)).changed();
                                        geom_changed |= ui.add(egui::DragValue::new(half_length).speed(0.005).prefix("hl:").range(0.001..=10.0)).changed();
                                    });
                                }
                                GeomData::Sphere { radius } => {
                                    ui.label("Sphere:");
                                    ui.horizontal(|ui| {
                                        geom_changed |= ui.add(egui::DragValue::new(radius).speed(0.005).prefix("r:").range(0.001..=10.0)).changed();
                                    });
                                }
                                GeomData::Mesh { vertices, filename, .. } => {
                                    let tri_count = vertices.len() / 18;
                                    let fname = filename.as_deref().unwrap_or("(inline)");
                                    ui.label(format!("Mesh: {tri_count} tris — {fname}"));
                                }
                            }

                            // --- Color editing ---
                            ui.horizontal(|ui| {
                                ui.label("Color:");
                                let mut col3 = [vis.color[0], vis.color[1], vis.color[2]];
                                if ui.color_edit_button_rgb(&mut col3).changed() {
                                    vis.color[0] = col3[0];
                                    vis.color[1] = col3[1];
                                    vis.color[2] = col3[2];
                                    geom_changed = true;
                                }
                                ui.add(egui::DragValue::new(&mut vis.color[3]).speed(0.01).prefix("a:").range(0.0..=1.0));
                            });

                            // --- Origin editing ---
                            ui.horizontal(|ui| {
                                ui.label("Origin:");
                                let t = &mut vis.origin.translation.vector;
                                ui.add(egui::DragValue::new(&mut t.x).speed(0.005).prefix("x:"));
                                ui.add(egui::DragValue::new(&mut t.y).speed(0.005).prefix("y:"));
                                ui.add(egui::DragValue::new(&mut t.z).speed(0.005).prefix("z:"));
                            });
                            // --- Rotation editing (RPY) ---
                            ui.horizontal(|ui| {
                                ui.label("Rot RPY:");
                                let (cur_r, cur_p, cur_y) = vis.origin.rotation.euler_angles();
                                let mut r_deg = cur_r.to_degrees();
                                let mut p_deg = cur_p.to_degrees();
                                let mut y_deg = cur_y.to_degrees();
                                let r_changed = ui.add(egui::DragValue::new(&mut r_deg).speed(0.5).prefix("R:").suffix("°")).changed();
                                let p_changed = ui.add(egui::DragValue::new(&mut p_deg).speed(0.5).prefix("P:").suffix("°")).changed();
                                let y_changed = ui.add(egui::DragValue::new(&mut y_deg).speed(0.5).prefix("Y:").suffix("°")).changed();
                                if r_changed || p_changed || y_changed {
                                    vis.origin.rotation = na::UnitQuaternion::from_euler_angles(
                                        r_deg.to_radians(),
                                        p_deg.to_radians(),
                                        y_deg.to_radians(),
                                    );
                                    geom_changed = true;
                                }
                            });

                            ui.separator();
                        });
                    }
                    // Process deferred add/remove/duplicate
                    if let Some(idx) = vis_to_duplicate {
                        let cloned = link.visuals[idx].clone();
                        link.visuals.insert(idx + 1, cloned);
                        geom_changed = true;
                    }
                    if let Some(idx) = vis_to_remove {
                        link.visuals.remove(idx);
                        geom_changed = true;
                    }
                    if geom_changed {
                        self.needs_upload = true;
                    }
                });
                // Right-click on Visuals header to add new visual
                vis_header.header_response.context_menu(|ui: &mut egui::Ui| {
                    if ui.button("➕ Add Box").clicked() {
                        link.visuals.push(crate::robot::VisualData {
                            origin: na::Isometry3::identity(),
                            geometry: GeomData::Box { hx: 0.05, hy: 0.05, hz: 0.05 },
                            color: [0.7, 0.7, 0.7, 1.0],
                        });
                        self.needs_upload = true;
                        ui.close();
                    }
                    if ui.button("➕ Add Cylinder").clicked() {
                        link.visuals.push(crate::robot::VisualData {
                            origin: na::Isometry3::identity(),
                            geometry: GeomData::Cylinder { radius: 0.02, half_length: 0.1 },
                            color: [0.7, 0.7, 0.7, 1.0],
                        });
                        self.needs_upload = true;
                        ui.close();
                    }
                    if ui.button("➕ Add Sphere").clicked() {
                        link.visuals.push(crate::robot::VisualData {
                            origin: na::Isometry3::identity(),
                            geometry: GeomData::Sphere { radius: 0.05 },
                            color: [0.7, 0.7, 0.7, 1.0],
                        });
                        self.needs_upload = true;
                        ui.close();
                    }
                });

                let col_count = link.collisions.len();
                let col_header = egui::CollapsingHeader::new(format!(
                    "💥 Collisions ({col_count})"
                ))
                .show(ui, |ui| {
                    let mut col_changed = false;
                    let mut col_to_remove: Option<usize> = None;
                    let mut col_to_duplicate: Option<usize> = None;
                    for ci in 0..link.collisions.len() {
                        let col = &mut link.collisions[ci];
                        ui.push_id(format!("col_{ci}"), |ui| {
                            let col_item_resp = ui.label(egui::RichText::new(format!("Collision #{ci}")).strong());
                            col_item_resp.context_menu(|ui| {
                                if ui.button("📋 Duplicate").clicked() {
                                    col_to_duplicate = Some(ci);
                                    ui.close();
                                }
                                if ui.button("🗑 Delete").clicked() {
                                    col_to_remove = Some(ci);
                                    ui.close();
                                }
                            });
                            match &mut col.geometry {
                                GeomData::Box { hx, hy, hz } => {
                                    ui.horizontal(|ui| {
                                        col_changed |= ui.add(egui::DragValue::new(hx).speed(0.005).prefix("hx:").range(0.001..=10.0)).changed();
                                        col_changed |= ui.add(egui::DragValue::new(hy).speed(0.005).prefix("hy:").range(0.001..=10.0)).changed();
                                        col_changed |= ui.add(egui::DragValue::new(hz).speed(0.005).prefix("hz:").range(0.001..=10.0)).changed();
                                    });
                                }
                                GeomData::Cylinder { radius, half_length } => {
                                    ui.horizontal(|ui| {
                                        col_changed |= ui.add(egui::DragValue::new(radius).speed(0.005).prefix("r:").range(0.001..=10.0)).changed();
                                        col_changed |= ui.add(egui::DragValue::new(half_length).speed(0.005).prefix("hl:").range(0.001..=10.0)).changed();
                                    });
                                }
                                GeomData::Sphere { radius } => {
                                    ui.horizontal(|ui| {
                                        col_changed |= ui.add(egui::DragValue::new(radius).speed(0.005).prefix("r:").range(0.001..=10.0)).changed();
                                    });
                                }
                                GeomData::Mesh { vertices, .. } => {
                                    ui.label(format!("Mesh ({} tris)", vertices.len() / 18));
                                }
                            }
                            // Origin
                            ui.horizontal(|ui| {
                                ui.label("Origin:");
                                let t = &mut col.origin.translation.vector;
                                ui.add(egui::DragValue::new(&mut t.x).speed(0.005).prefix("x:"));
                                ui.add(egui::DragValue::new(&mut t.y).speed(0.005).prefix("y:"));
                                ui.add(egui::DragValue::new(&mut t.z).speed(0.005).prefix("z:"));
                            });
                            // Rotation (RPY)
                            ui.horizontal(|ui| {
                                ui.label("Rot RPY:");
                                let (cur_r, cur_p, cur_y) = col.origin.rotation.euler_angles();
                                let mut r_deg = cur_r.to_degrees();
                                let mut p_deg = cur_p.to_degrees();
                                let mut y_deg = cur_y.to_degrees();
                                let r_changed = ui.add(egui::DragValue::new(&mut r_deg).speed(0.5).prefix("R:").suffix("°")).changed();
                                let p_changed = ui.add(egui::DragValue::new(&mut p_deg).speed(0.5).prefix("P:").suffix("°")).changed();
                                let y_changed = ui.add(egui::DragValue::new(&mut y_deg).speed(0.5).prefix("Y:").suffix("°")).changed();
                                if r_changed || p_changed || y_changed {
                                    col.origin.rotation = na::UnitQuaternion::from_euler_angles(
                                        r_deg.to_radians(),
                                        p_deg.to_radians(),
                                        y_deg.to_radians(),
                                    );
                                    col_changed = true;
                                }
                            });
                            ui.separator();
                        });
                    }
                    // Process deferred add/remove/duplicate for collisions
                    if let Some(idx) = col_to_duplicate {
                        let cloned = link.collisions[idx].clone();
                        link.collisions.insert(idx + 1, cloned);
                        col_changed = true;
                    }
                    if let Some(idx) = col_to_remove {
                        link.collisions.remove(idx);
                        col_changed = true;
                    }
                    if col_changed {
                        self.needs_upload = true;
                    }
                });
                // Right-click on Collisions header to add new collision
                col_header.header_response.context_menu(|ui: &mut egui::Ui| {
                    if ui.button("➕ Add Box").clicked() {
                        link.collisions.push(crate::robot::CollisionData {
                            origin: na::Isometry3::identity(),
                            geometry: GeomData::Box { hx: 0.05, hy: 0.05, hz: 0.05 },
                        });
                        self.needs_upload = true;
                        ui.close();
                    }
                    if ui.button("➕ Add Cylinder").clicked() {
                        link.collisions.push(crate::robot::CollisionData {
                            origin: na::Isometry3::identity(),
                            geometry: GeomData::Cylinder { radius: 0.02, half_length: 0.1 },
                        });
                        self.needs_upload = true;
                        ui.close();
                    }
                    if ui.button("➕ Add Sphere").clicked() {
                        link.collisions.push(crate::robot::CollisionData {
                            origin: na::Isometry3::identity(),
                            geometry: GeomData::Sphere { radius: 0.05 },
                        });
                        self.needs_upload = true;
                        ui.close();
                    }
                });
            }

            if let Some(ji) = self.selected_joint {
                let joint = &mut model.joints[ji];
                ui.label(egui::RichText::new(&joint.name).strong().size(16.0));
                ui.separator();
                egui::Grid::new("joint_props")
                    .striped(true)
                    .num_columns(2)
                    .show(ui, |ui| {
                        ui.label("Type:");
                        ui.label(&joint.joint_type);
                        ui.end_row();
                        ui.label("Parent:");
                        ui.label(&joint.parent_link);
                        ui.end_row();
                        ui.label("Child:");
                        ui.label(&joint.child_link);
                        ui.end_row();

                        ui.label("Axis:");
                        ui.horizontal(|ui| {
                            ui.add(egui::DragValue::new(&mut joint.axis.x).speed(0.01).prefix("x:"));
                            ui.add(egui::DragValue::new(&mut joint.axis.y).speed(0.01).prefix("y:"));
                            ui.add(egui::DragValue::new(&mut joint.axis.z).speed(0.01).prefix("z:"));
                        });
                        ui.end_row();

                        ui.label("Lower (rad):");
                        ui.add(egui::DragValue::new(&mut joint.lower).speed(0.01));
                        ui.end_row();
                        ui.label("Upper (rad):");
                        ui.add(egui::DragValue::new(&mut joint.upper).speed(0.01));
                        ui.end_row();
                        ui.label("Effort (Nm):");
                        ui.add(egui::DragValue::new(&mut joint.effort).speed(0.1).range(0.0..=f64::MAX));
                        ui.end_row();
                        ui.label("Velocity (rad/s):");
                        ui.add(egui::DragValue::new(&mut joint.velocity).speed(0.1).range(0.0..=f64::MAX));
                        ui.end_row();

                        ui.label("Origin xyz:");
                        ui.horizontal(|ui| {
                            let t = &mut joint.origin.translation;
                            ui.add(egui::DragValue::new(&mut t.x).speed(0.0001).prefix("x:"));
                            ui.add(egui::DragValue::new(&mut t.y).speed(0.0001).prefix("y:"));
                            ui.add(egui::DragValue::new(&mut t.z).speed(0.0001).prefix("z:"));
                        });
                        ui.end_row();
                    });
            }

            if self.selected_link.is_none() && self.selected_joint.is_none() {
                ui.label("Select a link or joint to view properties.");
            }
        }

        // --- Save / Export section ---
        ui.separator();
        ui.heading("Save / Export");

        // Save: overwrite original (always URDF)
        ui.horizontal(|ui| {
            if ui.button("💾 Save").clicked() {
                self.do_save();
            }
            if let Some(ref model) = self.model {
                if let Some(ref p) = model.source_path {
                    ui.label(
                        egui::RichText::new(p.display().to_string())
                            .small()
                            .weak(),
                    );
                }
            }
        });

        ui.add_space(4.0);

        // Export: write to a different directory in selected format
        ui.horizontal(|ui| {
            ui.label("Format:");
            egui::ComboBox::from_id_salt("export_fmt")
                .selected_text(self.export_format.label())
                .show_ui(ui, |ui| {
                    for &fmt in RobotFormat::ALL {
                        if fmt.supports_export() {
                            ui.selectable_value(&mut self.export_format, fmt, fmt.label());
                        }
                    }
                });
        });
        ui.horizontal(|ui| {
            ui.label("Dir:");
            ui.text_edit_singleline(&mut self.export_dir);
        });
        if ui.button("📦 Export").clicked() {
            self.do_export();
        }
        if !self.export_message.is_empty() {
            ui.label(&self.export_message);
        }
    }

    fn do_save(&mut self) {
        let Some(ref model) = self.model else {
            self.export_message = "⚠ No model loaded.".into();
            return;
        };
        if model.source_path.is_none() {
            self.export_message = "⚠ New model has no source file. Use Export instead.".into();
            return;
        }
        match model.save_urdf() {
            Ok(path) => {
                self.export_message = format!("✔ Saved to {}", path.display());
            }
            Err(e) => {
                self.export_message = format!("⚠ Save failed: {e}");
            }
        }
    }

    fn do_export(&mut self) {
        if self.export_dir.is_empty() {
            self.export_message = "⚠ Please specify an output directory.".into();
            return;
        }
        let Some(ref model) = self.model else {
            self.export_message = "⚠ No model loaded.".into();
            return;
        };
        let dir = PathBuf::from(&self.export_dir);
        if let Err(e) = std::fs::create_dir_all(&dir) {
            self.export_message = format!("⚠ Cannot create dir: {e}");
            return;
        }

        let fmt = self.export_format;
        let base_name = model
            .source_path
            .as_ref()
            .and_then(|p| p.file_stem().map(|s| s.to_string_lossy().into_owned()))
            .unwrap_or_else(|| model.name.clone());

        match fmt {
            RobotFormat::Urdf => {
                let filename = format!("{base_name}.urdf");
                let output_path = dir.join(&filename);
                match model.export_urdf_to_file(&output_path) {
                    Ok(()) => {
                        self.export_message =
                            format!("✔ Exported URDF to {} (with meshes)", output_path.display());
                    }
                    Err(e) => {
                        self.export_message = format!("⚠ URDF export failed: {e}");
                    }
                }
            }
            RobotFormat::Sdf => {
                let filename = format!("{base_name}.sdf");
                let output_path = dir.join(&filename);
                match crate::sdf::export_sdf_to_file(model, &output_path) {
                    Ok(()) => {
                        self.export_message =
                            format!("✔ Exported SDF to {} (with meshes)", output_path.display());
                    }
                    Err(e) => {
                        self.export_message = format!("⚠ SDF export failed: {e}");
                    }
                }
            }
            RobotFormat::Mjcf => {
                let filename = format!("{base_name}.xml");
                let output_path = dir.join(&filename);
                let xml = crate::mjcf::export_mjcf(model);
                match std::fs::write(&output_path, &xml) {
                    Ok(()) => {
                        self.export_message =
                            format!("✔ Exported MJCF to {}", output_path.display());
                    }
                    Err(e) => {
                        self.export_message = format!("⚠ MJCF export failed: {e}");
                    }
                }
            }
            RobotFormat::IsaacUsd => {
                match crate::isaac::export_isaac_to_dir(model, &dir) {
                    Ok(()) => {
                        self.export_message =
                            format!("✔ Isaac export to {} (URDF + Python script)", dir.display());
                    }
                    Err(e) => {
                        self.export_message = format!("⚠ Isaac export failed: {e}");
                    }
                }
            }
        }
    }

    fn draw_viewport(&mut self, ui: &mut egui::Ui) {
        let available = ui.available_size();
        let (rect, response) =
            ui.allocate_exact_size(available, egui::Sense::click_and_drag());

        self.viewport_rect = rect;
        let aspect = rect.width() / rect.height().max(1.0);

        // ===== Picking & Drag Logic =====
        let transforms = self
            .model
            .as_ref()
            .map(|m| m.compute_transforms())
            .unwrap_or_default();

        // Convert mouse position to normalized viewport coords [0..1]
        let mouse_ndc = response.hover_pos().map(|pos| {
            na::Point2::new(
                (pos.x - rect.left()) / rect.width(),
                (pos.y - rect.top()) / rect.height(),
            )
        });

        // Hover highlight: cast ray on hover
        if let (Some(ndc), Some(model)) = (mouse_ndc, &self.model) {
            if self.drag_state.is_none() && self.offset_drag_state.is_none() {
                let (ro, rd) = self.camera.screen_ray(ndc, aspect);
                self.hovered_link = model.pick_link(&ro, &rd, &transforms).map(|(li, _)| li);
            }
        } else {
            self.hovered_link = None;
        }

        // --- Gizmo hover detection (OffsetAdjust mode) ---
        // Compute gizmo transform based on target type.
        // In Translate mode the gizmo axes align with the parent/link frame.
        // In Rotate mode they align with the element's own world orientation
        // so the rings follow the current rotation during drag.
        let use_element_rot = self.gizmo_op == GizmoOp::Rotate || self.gizmo_op == GizmoOp::Scale;
        let gizmo_tf: Option<na::Isometry3<f32>> =
            if self.interaction_mode == InteractionMode::OffsetAdjust {
                self.model.as_ref().and_then(|m| {
                    match self.offset_target {
                        OffsetTarget::Joint => {
                            self.selected_joint.map(|ji| {
                                let joint = &m.joints[ji];
                                let parent_tf = transforms
                                    .get(&joint.parent_link)
                                    .copied()
                                    .unwrap_or(na::Isometry3::identity());
                                let joint_world = parent_tf * joint.origin;
                                let rot = if use_element_rot {
                                    joint_world.rotation
                                } else {
                                    parent_tf.rotation
                                };
                                na::Isometry3::from_parts(
                                    joint_world.translation,
                                    rot,
                                )
                            })
                        }
                        OffsetTarget::Visual => {
                            self.selected_link.and_then(|li| {
                                self.selected_visual.and_then(|vi| {
                                    m.links.get(li).and_then(|link| {
                                        link.visuals.get(vi).map(|vis| {
                                            let link_tf = transforms
                                                .get(&link.name)
                                                .copied()
                                                .unwrap_or(na::Isometry3::identity());
                                            let vis_world = link_tf * vis.origin;
                                            let rot = if use_element_rot {
                                                vis_world.rotation
                                            } else {
                                                link_tf.rotation
                                            };
                                            na::Isometry3::from_parts(
                                                vis_world.translation,
                                                rot,
                                            )
                                        })
                                    })
                                })
                            })
                        }
                        OffsetTarget::Collision => {
                            self.selected_link.and_then(|li| {
                                self.selected_collision.and_then(|ci| {
                                    m.links.get(li).and_then(|link| {
                                        link.collisions.get(ci).map(|col| {
                                            let link_tf = transforms
                                                .get(&link.name)
                                                .copied()
                                                .unwrap_or(na::Isometry3::identity());
                                            let col_world = link_tf * col.origin;
                                            let rot = if use_element_rot {
                                                col_world.rotation
                                            } else {
                                                link_tf.rotation
                                            };
                                            na::Isometry3::from_parts(
                                                col_world.translation,
                                                rot,
                                            )
                                        })
                                    })
                                })
                            })
                        }
                    }
                })
            } else {
                None
            };

        const GIZMO_ARROW_LENGTH: f32 = 0.08; // shaft + head total
        const GIZMO_PICK_RADIUS: f32 = 0.012;
        const GIZMO_RING_RADIUS: f32 = 0.05;  // must match ring_radius in renderer
        const GIZMO_RING_PICK_TOL: f32 = 0.015;

        // Hover: check which gizmo axis the mouse is over
        self.hovered_gizmo_axis = None;
        if self.interaction_mode == InteractionMode::OffsetAdjust
            && self.offset_drag_state.is_none()
        {
            if let (Some(ndc), Some(gt)) = (mouse_ndc, gizmo_tf) {
                let (ro, rd) = self.camera.screen_ray(ndc, aspect);
                let origin = na::Point3::from(gt.translation.vector);
                let axes = [
                    gt.rotation * na::Vector3::x(),
                    gt.rotation * na::Vector3::y(),
                    gt.rotation * na::Vector3::z(),
                ];

                if self.gizmo_op == GizmoOp::Translate || self.gizmo_op == GizmoOp::Scale {
                    let mut best_dist = f32::MAX;
                    for (i, axis) in axes.iter().enumerate() {
                        let (t_line, dist) = robot::ray_axis_closest(&ro, &rd, &origin, axis);
                        if t_line >= 0.0
                            && t_line <= GIZMO_ARROW_LENGTH
                            && dist < GIZMO_PICK_RADIUS
                            && dist < best_dist
                        {
                            best_dist = dist;
                            self.hovered_gizmo_axis = Some(i as u8);
                        }
                    }
                } else {
                    // Rotate mode: pick ring circles
                    let mut best_dist = f32::MAX;
                    for (i, axis) in axes.iter().enumerate() {
                        let dist = ray_ring_distance(
                            &ro, &rd, &origin, axis, GIZMO_RING_RADIUS,
                        );
                        if dist < GIZMO_RING_PICK_TOL && dist < best_dist {
                            best_dist = dist;
                            self.hovered_gizmo_axis = Some(i as u8);
                        }
                    }
                }
            }
        }

        // Left mouse button pressed: start drag
        if response.drag_started_by(egui::PointerButton::Primary) {
            match self.interaction_mode {
                InteractionMode::OffsetAdjust => {
                    // Try to pick a gizmo arrow first
                    if let (Some(axis_idx), Some(gt)) =
                        (self.hovered_gizmo_axis, gizmo_tf)
                    {
                        if let Some(ndc) = mouse_ndc {
                            let (ro, rd) = self.camera.screen_ray(ndc, aspect);
                            let origin = na::Point3::from(gt.translation.vector);
                            let axes = [
                                gt.rotation * na::Vector3::x(),
                                gt.rotation * na::Vector3::y(),
                                gt.rotation * na::Vector3::z(),
                            ];
                            let axis_dir = axes[axis_idx as usize];
                            let (t_line, _) = robot::ray_axis_closest(&ro, &rd, &origin, &axis_dir);
                            let inv_rot = gt.rotation.inverse();

                            // Compute initial angle for rotation mode
                            let initial_angle = if self.gizmo_op == GizmoOp::Rotate {
                                compute_ring_angle(&ro, &rd, &origin, &axis_dir)
                            } else {
                                0.0
                            };
                            let cur_op = self.gizmo_op;

                            if let Some(model) = &self.model {
                                let drag = match self.offset_target {
                                    OffsetTarget::Joint => {
                                        self.selected_joint.map(|ji| OffsetDragState {
                                            axis: axis_idx,
                                            target: OffsetTarget::Joint,
                                            entity_idx: ji,
                                            sub_idx: 0,
                                            axis_dir_world: axis_dir,
                                            gizmo_origin: origin,
                                            initial_t: t_line,
                                            initial_translation: model.joints[ji].origin.translation,
                                            inv_parent_rotation: inv_rot,
                                            op: cur_op,
                                            initial_rotation: model.joints[ji].origin.rotation,
                                            initial_angle,
                                            initial_geom_params: [0.0; 3],
                                        })
                                    }
                                    OffsetTarget::Visual => {
                                        self.selected_link.and_then(|li| {
                                            self.selected_visual.map(|vi| OffsetDragState {
                                                axis: axis_idx,
                                                target: OffsetTarget::Visual,
                                                entity_idx: li,
                                                sub_idx: vi,
                                                axis_dir_world: axis_dir,
                                                gizmo_origin: origin,
                                                initial_t: t_line,
                                                initial_translation: model.links[li].visuals[vi].origin.translation,
                                                inv_parent_rotation: inv_rot,
                                                op: cur_op,
                                                initial_rotation: model.links[li].visuals[vi].origin.rotation,
                                                initial_angle,
                                                initial_geom_params: geom_params(&model.links[li].visuals[vi].geometry),
                                            })
                                        })
                                    }
                                    OffsetTarget::Collision => {
                                        self.selected_link.and_then(|li| {
                                            self.selected_collision.map(|ci| OffsetDragState {
                                                axis: axis_idx,
                                                target: OffsetTarget::Collision,
                                                entity_idx: li,
                                                sub_idx: ci,
                                                axis_dir_world: axis_dir,
                                                gizmo_origin: origin,
                                                initial_t: t_line,
                                                initial_translation: model.links[li].collisions[ci].origin.translation,
                                                inv_parent_rotation: inv_rot,
                                                op: cur_op,
                                                initial_rotation: model.links[li].collisions[ci].origin.rotation,
                                                initial_angle,
                                                initial_geom_params: geom_params(&model.links[li].collisions[ci].geometry),
                                            })
                                        })
                                    }
                                };
                                self.offset_drag_state = drag;
                            }
                        }
                    } else if let (Some(ndc), Some(model)) = (mouse_ndc, &self.model) {
                        // No gizmo arrow hit: pick a link to select
                        let (ro, rd) = self.camera.screen_ray(ndc, aspect);
                        if let Some((li, _)) = model.pick_link(&ro, &rd, &transforms) {
                            let link_name = &model.links[li].name;
                            let changed_link = self.selected_link != Some(li);
                            self.selected_link = Some(li);
                            self.selected_joint = model.parent_joint_of_link(link_name);
                            // Auto-expand tree ancestors to reveal the selected link
                            self.tree_reveal_ancestors = model.ancestor_links(link_name);
                            // Auto-select first visual/collision when changing link
                            if changed_link {
                                self.selected_visual = if model.links[li].visuals.is_empty() {
                                    None
                                } else {
                                    Some(0)
                                };
                                self.selected_collision =
                                    if model.links[li].collisions.is_empty() {
                                        None
                                    } else {
                                        Some(0)
                                    };
                            }
                        }
                    }
                }
                InteractionMode::JointDrive => {
                    if let (Some(ndc), Some(model)) = (mouse_ndc, &self.model) {
                        let (ro, rd) = self.camera.screen_ray(ndc, aspect);
                        if let Some((li, _dist)) = model.pick_link(&ro, &rd, &transforms) {
                            let link_name = &model.links[li].name;
                            match self.drag_mode {
                                DragMode::SingleJoint => {
                                    if let Some(ji) = model.parent_joint_of_link(link_name) {
                                        let joint = &model.joints[ji];
                                        if joint.joint_type == "revolute"
                                            || joint.joint_type == "continuous"
                                        {
                                            let parent_tf = transforms
                                                .get(&joint.parent_link)
                                                .copied()
                                                .unwrap_or(na::Isometry3::identity());
                                            let joint_tf = parent_tf * joint.origin;
                                            let world_axis = joint_tf * joint.axis;
                                            let pivot_world =
                                                na::Point3::from(joint_tf.translation.vector);

                                            self.drag_state = Some(DragState {
                                                link_idx: li,
                                                mode: DragMode::SingleJoint,
                                                joint_idx: ji,
                                                world_axis,
                                                pivot_world,
                                                chain: Vec::new(),
                                                ik_root_link: None,
                                                ik_root_initial_tf: None,
                                            });
                                            self.selected_link = Some(li);
                                            self.selected_joint = Some(ji);
                                            self.tree_reveal_ancestors = model.ancestor_links(link_name);
                                        }
                                    }
                                }
                                DragMode::InverseKinematics => {
                                    let chain = ik::build_chain_between(
                                        model,
                                        link_name,
                                        self.ik_root_link.as_deref(),
                                    );
                                    if !chain.is_empty() {
                                        let ji =
                                            chain.last().map(|c| c.joint_idx).unwrap_or(0);
                                        // Capture the IK root's world transform at drag start
                                        // so we can anchor it exactly throughout the drag.
                                        let ik_root_tf = self.ik_root_link.as_ref().and_then(|name| {
                                            transforms.get(name).copied()
                                        });
                                        self.drag_state = Some(DragState {
                                            link_idx: li,
                                            mode: DragMode::InverseKinematics,
                                            joint_idx: ji,
                                            world_axis: na::Vector3::zeros(),
                                            pivot_world: na::Point3::origin(),
                                            chain,
                                            ik_root_link: self.ik_root_link.clone(),
                                            ik_root_initial_tf: ik_root_tf,
                                        });
                                        self.selected_link = Some(li);
                                        self.tree_reveal_ancestors = model.ancestor_links(link_name);
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }

        // While dragging: handle joint drive or offset adjustment
        if response.dragged_by(egui::PointerButton::Primary) {
            if let Some(ref odrag) = self.offset_drag_state.clone() {
                // --- Offset adjustment drag ---
                if let Some(ndc) = mouse_ndc {
                    let (ro, rd) = self.camera.screen_ray(ndc, aspect);

                    match odrag.op {
                        GizmoOp::Translate => {
                            let (t_line, _) =
                                robot::ray_axis_closest(&ro, &rd, &odrag.gizmo_origin, &odrag.axis_dir_world);
                            let delta_t = t_line - odrag.initial_t;

                            if let Some(ref mut model) = self.model {
                                let world_disp = odrag.axis_dir_world * delta_t;
                                let local_disp = odrag.inv_parent_rotation * world_disp;
                                let new_trans = na::Translation3::new(
                                    odrag.initial_translation.vector.x + local_disp.x,
                                    odrag.initial_translation.vector.y + local_disp.y,
                                    odrag.initial_translation.vector.z + local_disp.z,
                                );

                                match odrag.target {
                                    OffsetTarget::Joint => {
                                        model.joints[odrag.entity_idx].origin.translation = new_trans;
                                    }
                                    OffsetTarget::Visual => {
                                        model.links[odrag.entity_idx].visuals[odrag.sub_idx]
                                            .origin.translation = new_trans;
                                    }
                                    OffsetTarget::Collision => {
                                        model.links[odrag.entity_idx].collisions[odrag.sub_idx]
                                            .origin.translation = new_trans;
                                    }
                                }
                                self.needs_upload = true;
                            }
                        }
                        GizmoOp::Rotate => {
                            let cur_angle = compute_ring_angle(
                                &ro, &rd, &odrag.gizmo_origin, &odrag.axis_dir_world,
                            );
                            let delta_angle = cur_angle - odrag.initial_angle;

                            if let Some(ref mut model) = self.model {
                                // Build rotation around the LOCAL axis corresponding to odrag.axis
                                let local_axis = match odrag.axis {
                                    0 => na::Vector3::x_axis(),
                                    1 => na::Vector3::y_axis(),
                                    _ => na::Vector3::z_axis(),
                                };
                                let delta_rot = na::UnitQuaternion::from_axis_angle(
                                    &local_axis,
                                    delta_angle,
                                );
                                let new_rot = odrag.initial_rotation * delta_rot;

                                match odrag.target {
                                    OffsetTarget::Joint => {
                                        model.joints[odrag.entity_idx].origin.rotation = new_rot;
                                    }
                                    OffsetTarget::Visual => {
                                        model.links[odrag.entity_idx].visuals[odrag.sub_idx]
                                            .origin.rotation = new_rot;
                                    }
                                    OffsetTarget::Collision => {
                                        model.links[odrag.entity_idx].collisions[odrag.sub_idx]
                                            .origin.rotation = new_rot;
                                    }
                                }
                                self.needs_upload = true;
                            }
                        }
                        GizmoOp::Scale => {
                            let (t_line, _) =
                                robot::ray_axis_closest(&ro, &rd, &odrag.gizmo_origin, &odrag.axis_dir_world);
                            let delta_t = t_line - odrag.initial_t;

                            if let Some(ref mut model) = self.model {
                                match odrag.target {
                                    OffsetTarget::Visual => {
                                        let geom = &mut model.links[odrag.entity_idx]
                                            .visuals[odrag.sub_idx].geometry;
                                        apply_geom_scale(geom, odrag.axis, odrag.initial_geom_params, delta_t);
                                    }
                                    OffsetTarget::Collision => {
                                        let geom = &mut model.links[odrag.entity_idx]
                                            .collisions[odrag.sub_idx].geometry;
                                        apply_geom_scale(geom, odrag.axis, odrag.initial_geom_params, delta_t);
                                    }
                                    OffsetTarget::Joint => {} // no geometry to scale
                                }
                                self.needs_upload = true;
                            }
                        }
                    }
                }
            } else if let Some(ref drag) = self.drag_state.clone() {
                let delta = response.drag_delta();
                if delta.length_sq() > 0.0 {
                    match drag.mode {
                        DragMode::SingleJoint => {
                            // Intersect previous and current mouse rays with the
                            // plane perpendicular to the joint axis at the pivot.
                            // Then compute the signed angle between the two radial
                            // vectors around the axis.
                            if let Some(pos) = response.hover_pos() {
                                let axis = drag.world_axis.normalize();
                                let pivot = drag.pivot_world;

                                let prev_ndc = na::Point2::new(
                                    (pos.x - delta.x - rect.left()) / rect.width(),
                                    (pos.y - delta.y - rect.top()) / rect.height(),
                                );
                                let curr_ndc = na::Point2::new(
                                    (pos.x - rect.left()) / rect.width(),
                                    (pos.y - rect.top()) / rect.height(),
                                );

                                let (ro0, rd0) = self.camera.screen_ray(prev_ndc, aspect);
                                let (ro1, rd1) = self.camera.screen_ray(curr_ndc, aspect);

                                // Intersect ray with plane: pivot·axis = (ro + t*rd)·axis
                                let ray_plane_hit =
                                    |ro: &na::Point3<f32>,
                                     rd: &na::Vector3<f32>|
                                     -> Option<na::Point3<f32>> {
                                        let denom = rd.dot(&axis);
                                        if denom.abs() < 1e-7 {
                                            return None; // ray parallel to plane
                                        }
                                        let t = (pivot - ro).dot(&axis) / denom;
                                        Some(ro + rd * t)
                                    };

                                let angle_delta = match (
                                    ray_plane_hit(&ro0, &rd0),
                                    ray_plane_hit(&ro1, &rd1),
                                ) {
                                    (Some(p0), Some(p1)) => {
                                        let v0 = p0 - pivot;
                                        let v1 = p1 - pivot;
                                        if v0.norm() < 1e-8 || v1.norm() < 1e-8 {
                                            0.0
                                        } else {
                                            let v0n = v0.normalize();
                                            let v1n = v1.normalize();
                                            let cross = v0n.cross(&v1n);
                                            let dot = v0n.dot(&v1n).clamp(-1.0, 1.0);
                                            // signed angle: positive when rotating
                                            // in the direction of the axis (right-hand rule)
                                            cross.dot(&axis).atan2(dot)
                                        }
                                    }
                                    _ => {
                                        // Fallback: ray nearly parallel to the axis plane.
                                        // Use simple screen-space delta.
                                        let delta_ndc = na::Vector2::new(
                                            delta.x / rect.width(),
                                            delta.y / rect.height(),
                                        );
                                        delta_ndc.norm() * 3.0 * delta.x.signum()
                                    }
                                };

                                if angle_delta.abs() > 1e-8 {
                                    if let Some(ref mut model) = self.model {
                                        let ji = drag.joint_idx;
                                        let lower = model.joints[ji].lower as f32;
                                        let upper = model.joints[ji].upper as f32;
                                        model.joint_positions[ji] =
                                            (model.joint_positions[ji] + angle_delta)
                                                .clamp(lower, upper);
                                    }
                                }
                            }
                        }
                        DragMode::InverseKinematics => {
                            // IK mode: cast ray from current mouse position and intersect
                            // with a plane at the end-effector to get a 3D target.
                            if let (Some(ndc), Some(ref mut model)) =
                                (mouse_ndc, self.model.as_mut())
                            {
                                let cur_transforms = model.compute_transforms();
                                let ee_pos = ik::get_ee_world_pos(
                                    model,
                                    drag.link_idx,
                                    &cur_transforms,
                                );

                                // Record IK root transform from drag start (fixed anchor)
                                let ik_root_tf_desired = drag.ik_root_initial_tf;

                                // Camera view direction for the intersection plane
                                let (ray_o, ray_d) = self.camera.screen_ray(ndc, aspect);
                                let cam_forward = (self.camera.target
                                    - na::Point3::from(
                                        self.camera.eye().coords,
                                    ))
                                .normalize();

                                // Intersect ray with plane through ee_pos, normal = cam_forward
                                let denom = ray_d.dot(&cam_forward);
                                if denom.abs() > 1e-6 {
                                    let t = (ee_pos - ray_o).dot(&cam_forward) / denom;
                                    if t > 0.0 {
                                        let target = ray_o + ray_d * t;

                                        let damping = self.ik_damping;
                                        let deltas = ik::solve_ik_step(
                                            model,
                                            &drag.chain,
                                            &cur_transforms,
                                            &ee_pos,
                                            &target,
                                            damping,
                                            0.1, // max_step
                                        );
                                        ik::apply_ik_deltas(model, &drag.chain, &deltas);

                                        // If an IK root is set, correct base_transform
                                        // so the IK root link stays fixed at its drag-start position.
                                        if let Some(desired_tf) = ik_root_tf_desired {
                                            // Recompute with identity base to get
                                            // the root-relative transform of the IK root link.
                                            let saved_base = model.base_transform;
                                            model.base_transform = na::Isometry3::identity();
                                            let identity_transforms = model.compute_transforms();
                                            if let Some(&ik_root_tf_rel) = drag.ik_root_link.as_ref()
                                                .and_then(|name| identity_transforms.get(name))
                                            {
                                                // new_base * ik_root_tf_rel = desired_tf
                                                // new_base = desired_tf * inv(ik_root_tf_rel)
                                                model.base_transform = desired_tf * ik_root_tf_rel.inverse();
                                            } else {
                                                model.base_transform = saved_base;
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            } else {
                // No drag state: left-drag orbits camera
                self.camera.handle_orbit_pan_zoom(&response);
            }
        }

        // Right-drag = pan, middle-drag = pan, scroll = zoom (always active)
        if response.dragged_by(egui::PointerButton::Secondary)
            || response.dragged_by(egui::PointerButton::Middle)
        {
            // Pan with right/middle mouse
            let delta = response.drag_delta();
            let right = na::Vector3::new(-self.camera.yaw.sin(), self.camera.yaw.cos(), 0.0);
            let up = na::Vector3::z();
            let pan_speed = self.camera.distance * 0.002;
            self.camera.target -= right * delta.x * pan_speed;
            self.camera.target += up * delta.y * pan_speed;
        }
        // Scroll zoom always
        if response.hovered() {
            let scroll = ui.ctx().input(|i| i.smooth_scroll_delta.y);
            if scroll != 0.0 {
                self.camera.distance *= 1.0 - scroll * 0.002;
                self.camera.distance = self.camera.distance.clamp(0.01, 50.0);
            }
        }

        // Drag released
        if response.drag_stopped_by(egui::PointerButton::Primary) {
            self.drag_state = None;
            self.offset_drag_state = None;
        }

        // Update highlight and gizmo state in renderer
        {
            let mut r = self.gl_renderer.lock().unwrap();
            let highlight = if self.drag_state.is_some() {
                self.drag_state
                    .as_ref()
                    .and_then(|d| self.model.as_ref().map(|m| m.links[d.link_idx].name.clone()))
            } else {
                self.hovered_link
                    .and_then(|li| self.model.as_ref().map(|m| m.links[li].name.clone()))
            };
            r.highlight_link = highlight;

            // Gizmo state
            r.gizmo_transform = gizmo_tf;
            r.gizmo_hovered_axis = self.hovered_gizmo_axis;
            r.gizmo_dragged_axis = self.offset_drag_state.as_ref().map(|d| d.axis);
            r.gizmo_op = match self.gizmo_op {
                GizmoOp::Translate => 0,
                GizmoOp::Rotate => 1,
                GizmoOp::Scale => 2,
            };
        }

        // Change cursor when hovering a draggable link or gizmo arrow
        if self.drag_state.is_some() || self.offset_drag_state.is_some() {
            ui.ctx().set_cursor_icon(egui::CursorIcon::Grabbing);
        } else if self.hovered_gizmo_axis.is_some() {
            ui.ctx().set_cursor_icon(egui::CursorIcon::Grab);
        } else if self.hovered_link.is_some() {
            // Check if the hovered link has a movable parent joint (or IK chain)
            let is_draggable = self.hovered_link.and_then(|li| {
                self.model.as_ref().and_then(|m| {
                    let link_name = &m.links[li].name;
                    match self.interaction_mode {
                        InteractionMode::OffsetAdjust => {
                            // In offset mode, any link is "selectable"
                            Some(&m.links[li].name as &str)
                        }
                        InteractionMode::JointDrive => match self.drag_mode {
                            DragMode::SingleJoint => m
                                .parent_joint_of_link(link_name)
                                .map(|ji| &m.joints[ji].joint_type)
                                .filter(|jt| *jt == "revolute" || *jt == "continuous")
                                .map(|s| s.as_str()),
                            DragMode::InverseKinematics => {
                                let chain = ik::build_chain(m, link_name);
                                if chain.is_empty() {
                                    None
                                } else {
                                    Some(m.joints[chain[0].joint_idx].joint_type.as_str())
                                }
                            }
                        },
                    }
                })
            });
            if is_draggable.is_some() {
                ui.ctx().set_cursor_icon(egui::CursorIcon::Grab);
            }
        }

        // Clone data for the paint callback closure
        let renderer = self.gl_renderer.clone();
        let camera = self.camera.clone();

        let callback = egui::PaintCallback {
            rect,
            callback: Arc::new(eframe::egui_glow::CallbackFn::new(
                move |info, painter| {
                    let gl = painter.gl();
                    let renderer = renderer.lock().unwrap();

                    let vp = info.viewport_in_pixels();
                    let screen_h = info.screen_size_px[1] as i32;
                    let gl_viewport = [
                        vp.left_px,
                        screen_h - vp.top_px - vp.height_px,
                        vp.width_px,
                        vp.height_px,
                    ];

                    renderer.render(gl, &camera, gl_viewport);
                },
            )),
        };

        ui.painter().add(callback);

        // ===== Viewport overlay: mode toolbar =====
        {
            let painter = ui.painter();
            let icon_size = egui::vec2(32.0, 32.0);
            let margin = 8.0;
            let toolbar_x = rect.left() + margin;
            let toolbar_y = rect.top() + margin;

            // --- Row 1: Interaction Mode buttons ---
            let modes: [(InteractionMode, &str); 2] = [
                (InteractionMode::JointDrive, "Joint Drive"),
                (InteractionMode::OffsetAdjust, "Offset Adjust"),
            ];

            for (i, (mode, tooltip)) in modes.iter().enumerate() {
                let btn_pos = egui::pos2(
                    toolbar_x + i as f32 * (icon_size.x + 4.0),
                    toolbar_y,
                );
                let btn_rect = egui::Rect::from_min_size(btn_pos, icon_size);
                let is_active = self.interaction_mode == *mode;

                let btn_response = ui.interact(
                    btn_rect,
                    ui.id().with("mode_btn").with(i),
                    egui::Sense::click(),
                );

                let bg_color = if is_active {
                    egui::Color32::from_rgba_unmultiplied(60, 130, 220, 200)
                } else if btn_response.hovered() {
                    egui::Color32::from_rgba_unmultiplied(80, 80, 80, 180)
                } else {
                    egui::Color32::from_rgba_unmultiplied(40, 40, 50, 160)
                };
                painter.rect_filled(btn_rect, egui::CornerRadius::same(4), bg_color);

                if is_active {
                    painter.rect_stroke(
                        btn_rect,
                        egui::CornerRadius::same(4),
                        egui::Stroke::new(2.0, egui::Color32::from_rgb(100, 180, 255)),
                        egui::StrokeKind::Outside,
                    );
                }

                let icon_color = if is_active {
                    egui::Color32::WHITE
                } else {
                    egui::Color32::from_gray(200)
                };

                // Draw custom icon
                let c = btn_rect.center();
                match mode {
                    InteractionMode::JointDrive => {
                        // Joint drive icon: parent link, joint pivot, child link
                        // showing before (ghost) and after (solid) positions
                        let ghost_color = if is_active {
                            egui::Color32::from_rgba_unmultiplied(255, 255, 255, 70)
                        } else {
                            egui::Color32::from_rgba_unmultiplied(200, 200, 200, 50)
                        };

                        // Joint pivot position
                        let pivot = egui::pos2(c.x - 2.0, c.y + 1.0);

                        // Parent link (fixed, from top-left to pivot)
                        let arm1_start = egui::pos2(c.x - 11.0, c.y - 8.0);
                        painter.line_segment(
                            [arm1_start, pivot],
                            egui::Stroke::new(3.0, icon_color),
                        );

                        // Child link BEFORE rotation (ghost — going right/down)
                        let ghost_end = egui::pos2(pivot.x + 10.0, pivot.y + 3.0);
                        painter.line_segment(
                            [pivot, ghost_end],
                            egui::Stroke::new(2.5, ghost_color),
                        );

                        // Child link AFTER rotation (solid — rotated upward)
                        let arm2_end = egui::pos2(pivot.x + 8.0, pivot.y - 7.0);
                        painter.line_segment(
                            [pivot, arm2_end],
                            egui::Stroke::new(3.0, icon_color),
                        );

                        // Joint pivot (filled circle)
                        painter.circle_filled(pivot, 3.0, icon_color);
                        painter.circle_stroke(
                            pivot,
                            3.0,
                            egui::Stroke::new(1.2, bg_color),
                        );

                        // Rotation arc arrow from ghost to solid position
                        let arc_r = 7.0;
                        // Arc from ~10° (ghost direction) to ~-40° (solid direction)
                        let a_start = -0.29_f32; // atan2(3, 10) ≈ 0.29 rad
                        let a_end = 0.72_f32;    // atan2(7, 8) ≈ 0.72 rad
                        let n_seg = 8;
                        let arc_pts: Vec<egui::Pos2> = (0..=n_seg)
                            .map(|k| {
                                let t = k as f32 / n_seg as f32;
                                let a = a_start + (a_end - a_start) * t;
                                egui::pos2(
                                    pivot.x + arc_r * a.cos(),
                                    pivot.y - arc_r * a.sin(),
                                )
                            })
                            .collect();
                        for w in arc_pts.windows(2) {
                            painter.line_segment(
                                [w[0], w[1]],
                                egui::Stroke::new(1.2, icon_color),
                            );
                        }
                        // Arrowhead at end of arc
                        if let Some(&tip) = arc_pts.last() {
                            let prev = arc_pts[arc_pts.len() - 2];
                            let dir = egui::vec2(tip.x - prev.x, tip.y - prev.y);
                            let len = (dir.x * dir.x + dir.y * dir.y).sqrt().max(0.01);
                            let nd = egui::vec2(dir.x / len, dir.y / len);
                            let perp = egui::vec2(-nd.y, nd.x);
                            let sz = 3.5;
                            painter.line_segment(
                                [tip, tip - nd * sz + perp * sz * 0.6],
                                egui::Stroke::new(1.2, icon_color),
                            );
                            painter.line_segment(
                                [tip, tip - nd * sz - perp * sz * 0.6],
                                egui::Stroke::new(1.2, icon_color),
                            );
                        }
                    }
                    InteractionMode::OffsetAdjust => {
                        // Offset adjust icon: a 3D box moving from old to new position
                        let ghost_color = if is_active {
                            egui::Color32::from_rgba_unmultiplied(255, 255, 255, 60)
                        } else {
                            egui::Color32::from_rgba_unmultiplied(200, 200, 200, 45)
                        };
                        let solid_color = icon_color;
                        let arrow_color = if is_active {
                            egui::Color32::from_rgb(100, 200, 255)
                        } else {
                            egui::Color32::from_rgb(130, 170, 200)
                        };

                        // 3D box helper: draw an isometric-ish box outline
                        // at a given center with half-sizes, using perspective offsets
                        let draw_box = |cx: f32, cy: f32, hw: f32, hh: f32, depth: f32,
                                        stroke: egui::Stroke| {
                            // Front face corners
                            let fl = egui::pos2(cx - hw, cy - hh);
                            let fr = egui::pos2(cx + hw, cy - hh);
                            let br = egui::pos2(cx + hw, cy + hh);
                            let bl = egui::pos2(cx - hw, cy + hh);

                            // Back face offset (isometric depth)
                            let dx = depth * 0.6;
                            let dy = -depth * 0.5;
                            let fl2 = egui::pos2(fl.x + dx, fl.y + dy);
                            let fr2 = egui::pos2(fr.x + dx, fr.y + dy);
                            let br2 = egui::pos2(br.x + dx, br.y + dy);
                            let bl2 = egui::pos2(bl.x + dx, bl.y + dy);

                            // Front face
                            painter.line_segment([fl, fr], stroke);
                            painter.line_segment([fr, br], stroke);
                            painter.line_segment([br, bl], stroke);
                            painter.line_segment([bl, fl], stroke);

                            // Back face (top + right visible edges)
                            painter.line_segment([fl2, fr2], stroke);
                            painter.line_segment([fr2, br2], stroke);

                            // Depth edges (visible 3 corners)
                            painter.line_segment([fl, fl2], stroke);
                            painter.line_segment([fr, fr2], stroke);
                            painter.line_segment([br, br2], stroke);
                        };

                        // Ghost box (original position, lower-left)
                        let g_cx = c.x - 5.0;
                        let g_cy = c.y + 3.0;
                        draw_box(
                            g_cx, g_cy, 4.5, 3.5, 4.0,
                            egui::Stroke::new(1.2, ghost_color),
                        );

                        // Solid box (moved position, upper-right)
                        let s_cx = c.x + 4.5;
                        let s_cy = c.y - 3.5;
                        draw_box(
                            s_cx, s_cy, 4.5, 3.5, 4.0,
                            egui::Stroke::new(1.6, solid_color),
                        );

                        // Dashed move arrow from ghost center to solid center
                        let arrow_start = egui::pos2(g_cx + 2.0, g_cy - 1.5);
                        let arrow_end = egui::pos2(s_cx - 2.0, s_cy + 1.5);
                        // Draw dashed line (3 segments)
                        let n_dash = 3;
                        for k in 0..n_dash {
                            let t0 = (k as f32 * 2.0) / (n_dash as f32 * 2.0 - 1.0);
                            let t1 = (k as f32 * 2.0 + 1.0) / (n_dash as f32 * 2.0 - 1.0);
                            let t1 = t1.min(1.0);
                            let p0 = egui::pos2(
                                arrow_start.x + (arrow_end.x - arrow_start.x) * t0,
                                arrow_start.y + (arrow_end.y - arrow_start.y) * t0,
                            );
                            let p1 = egui::pos2(
                                arrow_start.x + (arrow_end.x - arrow_start.x) * t1,
                                arrow_start.y + (arrow_end.y - arrow_start.y) * t1,
                            );
                            painter.line_segment(
                                [p0, p1],
                                egui::Stroke::new(1.4, arrow_color),
                            );
                        }
                        // Arrowhead
                        let ad = egui::vec2(
                            arrow_end.x - arrow_start.x,
                            arrow_end.y - arrow_start.y,
                        );
                        let al = (ad.x * ad.x + ad.y * ad.y).sqrt().max(0.01);
                        let and = egui::vec2(ad.x / al, ad.y / al);
                        let aperp = egui::vec2(-and.y, and.x);
                        let ah = 3.5_f32;
                        painter.line_segment(
                            [arrow_end, arrow_end - and * ah + aperp * ah * 0.5],
                            egui::Stroke::new(1.4, arrow_color),
                        );
                        painter.line_segment(
                            [arrow_end, arrow_end - and * ah - aperp * ah * 0.5],
                            egui::Stroke::new(1.4, arrow_color),
                        );
                    }
                }

                if btn_response.clicked() {
                    self.interaction_mode = *mode;
                }
                if btn_response.hovered() {
                    let tip_pos = egui::pos2(btn_rect.left(), btn_rect.bottom() + 4.0);
                    painter.text(
                        tip_pos,
                        egui::Align2::LEFT_TOP,
                        *tooltip,
                        egui::FontId::proportional(12.0),
                        egui::Color32::from_gray(220),
                    );
                }
            }

            // --- Row 2: Target buttons (only in OffsetAdjust mode) ---
            if self.interaction_mode == InteractionMode::OffsetAdjust {
                let row2_y = toolbar_y + icon_size.y + 4.0;
                let small_size = egui::vec2(28.0, 28.0);

                for (i, target) in OffsetTarget::ALL.iter().enumerate() {
                    let btn_pos = egui::pos2(
                        toolbar_x + i as f32 * (small_size.x + 3.0),
                        row2_y,
                    );
                    let btn_rect = egui::Rect::from_min_size(btn_pos, small_size);
                    let is_active = self.offset_target == *target;

                    let btn_response = ui.interact(
                        btn_rect,
                        ui.id().with("target_btn").with(i),
                        egui::Sense::click(),
                    );

                    let bg_color = if is_active {
                        egui::Color32::from_rgba_unmultiplied(60, 180, 100, 200)
                    } else if btn_response.hovered() {
                        egui::Color32::from_rgba_unmultiplied(80, 80, 80, 180)
                    } else {
                        egui::Color32::from_rgba_unmultiplied(40, 40, 50, 160)
                    };
                    painter.rect_filled(btn_rect, egui::CornerRadius::same(3), bg_color);

                    if is_active {
                        painter.rect_stroke(
                            btn_rect,
                            egui::CornerRadius::same(3),
                            egui::Stroke::new(2.0, egui::Color32::from_rgb(80, 220, 130)),
                            egui::StrokeKind::Outside,
                        );
                    }

                    let text_color = if is_active {
                        egui::Color32::WHITE
                    } else {
                        egui::Color32::from_gray(200)
                    };
                    painter.text(
                        btn_rect.center(),
                        egui::Align2::CENTER_CENTER,
                        target.icon(),
                        egui::FontId::proportional(15.0),
                        text_color,
                    );

                    if btn_response.clicked() {
                        self.offset_target = *target;
                        if *target == OffsetTarget::Joint && self.gizmo_op == GizmoOp::Scale {
                            self.gizmo_op = GizmoOp::Translate;
                        }
                    }
                    if btn_response.hovered() {
                        let tip_pos = egui::pos2(btn_rect.left(), btn_rect.bottom() + 4.0);
                        painter.text(
                            tip_pos,
                            egui::Align2::LEFT_TOP,
                            target.label(),
                            egui::FontId::proportional(12.0),
                            egui::Color32::from_gray(220),
                        );
                    }
                }

                // --- Row 3: Gizmo op buttons (Translate / Rotate / Scale) ---
                let row3_y = row2_y + small_size.y + 4.0;
                let gizmo_ops: &[(GizmoOp, &str, &str)] = if self.offset_target != OffsetTarget::Joint {
                    &[
                        (GizmoOp::Translate, "⬌", "Translate"),
                        (GizmoOp::Rotate, "↻", "Rotate"),
                        (GizmoOp::Scale, "⬡", "Scale"),
                    ]
                } else {
                    &[
                        (GizmoOp::Translate, "⬌", "Translate"),
                        (GizmoOp::Rotate, "↻", "Rotate"),
                    ]
                };
                for (i, (op, icon, label)) in gizmo_ops.iter().enumerate() {
                    let btn_pos = egui::pos2(
                        toolbar_x + i as f32 * (small_size.x + 3.0),
                        row3_y,
                    );
                    let btn_rect = egui::Rect::from_min_size(btn_pos, small_size);
                    let is_active = self.gizmo_op == *op;

                    let btn_response = ui.interact(
                        btn_rect,
                        ui.id().with("gizmo_op_btn").with(i),
                        egui::Sense::click(),
                    );

                    let bg_color = if is_active {
                        egui::Color32::from_rgba_unmultiplied(60, 120, 200, 200)
                    } else if btn_response.hovered() {
                        egui::Color32::from_rgba_unmultiplied(80, 80, 80, 180)
                    } else {
                        egui::Color32::from_rgba_unmultiplied(40, 40, 50, 160)
                    };
                    painter.rect_filled(btn_rect, egui::CornerRadius::same(3), bg_color);

                    if is_active {
                        painter.rect_stroke(
                            btn_rect,
                            egui::CornerRadius::same(3),
                            egui::Stroke::new(2.0, egui::Color32::from_rgb(100, 160, 255)),
                            egui::StrokeKind::Outside,
                        );
                    }

                    let text_color = if is_active {
                        egui::Color32::WHITE
                    } else {
                        egui::Color32::from_gray(200)
                    };
                    painter.text(
                        btn_rect.center(),
                        egui::Align2::CENTER_CENTER,
                        *icon,
                        egui::FontId::proportional(15.0),
                        text_color,
                    );

                    if btn_response.clicked() {
                        self.gizmo_op = *op;
                    }
                    if btn_response.hovered() {
                        let tip_pos = egui::pos2(btn_rect.left(), btn_rect.bottom() + 4.0);
                        painter.text(
                            tip_pos,
                            egui::Align2::LEFT_TOP,
                            label,
                            egui::FontId::proportional(12.0),
                            egui::Color32::from_gray(220),
                        );
                    }
                }
            }
        }

        // Draw CoM mass labels as egui text on top of the 3D viewport
        if self.show_com {
            let com_positions = self.gl_renderer.lock().unwrap().com_world_positions();
            let painter = ui.painter();
            for (world_pos, mass) in &com_positions {
                if let Some(ndc) = self.camera.project(world_pos, aspect) {
                    let screen_pos = egui::pos2(
                        rect.left() + ndc.x * rect.width(),
                        rect.top() + ndc.y * rect.height(),
                    );
                    // Only draw if within the viewport
                    if rect.contains(screen_pos) {
                        let label = format!("{:.3} kg", mass);
                        painter.text(
                            screen_pos + egui::vec2(6.0, -6.0),
                            egui::Align2::LEFT_BOTTOM,
                            &label,
                            egui::FontId::proportional(11.0),
                            egui::Color32::from_rgb(255, 128, 255),
                        );
                    }
                }
            }
        }

        // ===== IK Root anchor icon =====
        // When in JointDrive + IK Chain mode with a custom root link, draw an
        // anchor icon at the IK root link's world position projected to screen.
        if self.interaction_mode == InteractionMode::JointDrive
            && self.drag_mode == DragMode::InverseKinematics
        {
            if let Some(ref root_name) = self.ik_root_link {
                if let Some(ref model) = self.model {
                    let transforms = model.compute_transforms();
                    if let Some(root_tf) = transforms.get(root_name) {
                        let world_pos = na::Point3::from(root_tf.translation.vector);
                        if let Some(ndc) = self.camera.project(&world_pos, aspect) {
                            let screen_pos = egui::pos2(
                                rect.left() + ndc.x * rect.width(),
                                rect.top() + ndc.y * rect.height(),
                            );
                            if rect.contains(screen_pos) {
                                let painter = ui.painter();
                                let c = screen_pos;
                                let anchor_color = egui::Color32::from_rgb(255, 180, 50);
                                let anchor_bg = egui::Color32::from_rgba_unmultiplied(0, 0, 0, 140);

                                // Background circle for readability
                                painter.circle_filled(c, 13.0, anchor_bg);

                                // Anchor icon: ring at top + vertical shank + curved arms
                                // Ring (top)
                                let ring_cy = c.y - 5.5;
                                painter.circle_stroke(
                                    egui::pos2(c.x, ring_cy),
                                    3.0,
                                    egui::Stroke::new(1.6, anchor_color),
                                );

                                // Vertical shank (from ring bottom to base)
                                let shank_top = ring_cy + 3.0;
                                let shank_bot = c.y + 7.0;
                                painter.line_segment(
                                    [egui::pos2(c.x, shank_top), egui::pos2(c.x, shank_bot)],
                                    egui::Stroke::new(1.8, anchor_color),
                                );

                                // Horizontal crossbar
                                let bar_y = c.y + 1.0;
                                painter.line_segment(
                                    [egui::pos2(c.x - 5.0, bar_y), egui::pos2(c.x + 5.0, bar_y)],
                                    egui::Stroke::new(1.6, anchor_color),
                                );

                                // Curved arms (left and right)
                                let n_seg = 6;
                                for &sign in &[-1.0_f32, 1.0] {
                                    let pts: Vec<egui::Pos2> = (0..=n_seg)
                                        .map(|k| {
                                            let t = k as f32 / n_seg as f32;
                                            let angle = t * std::f32::consts::FRAC_PI_2;
                                            egui::pos2(
                                                c.x + sign * 5.0 * angle.sin(),
                                                bar_y + 6.0 * angle.cos(),
                                            )
                                        })
                                        .collect();
                                    for w in pts.windows(2) {
                                        painter.line_segment(
                                            [w[0], w[1]],
                                            egui::Stroke::new(1.6, anchor_color),
                                        );
                                    }
                                }

                                // Label below
                                painter.text(
                                    egui::pos2(c.x, c.y + 15.0),
                                    egui::Align2::CENTER_TOP,
                                    root_name,
                                    egui::FontId::proportional(10.0),
                                    anchor_color,
                                );
                            }
                        }
                    }
                }
            }
        }

        // ===== Camera orientation axes (bottom-right) =====
        {
            let painter = ui.painter();
            let axes_size = 50.0_f32;
            let margin = 10.0;
            let center = egui::pos2(
                rect.right() - margin - axes_size,
                rect.bottom() - margin - axes_size,
            );

            // Background circle
            painter.circle_filled(
                center,
                axes_size,
                egui::Color32::from_rgba_unmultiplied(20, 20, 30, 150),
            );

            // Get camera rotation matrix (view matrix upper-left 3x3 = rotation)
            let view = self.camera.view_matrix();
            let view3 = view.fixed_view::<3, 3>(0, 0);

            // World axes projected into screen space via view rotation
            let axis_len = axes_size * 0.7;
            let world_axes: [(na::Vector3<f32>, egui::Color32, &str); 3] = [
                (na::Vector3::x(), egui::Color32::from_rgb(230, 60, 60), "X"),
                (na::Vector3::y(), egui::Color32::from_rgb(60, 200, 60), "Y"),
                (na::Vector3::z(), egui::Color32::from_rgb(60, 100, 230), "Z"),
            ];

            // Sort by depth (draw farthest first)
            let mut draw_order: Vec<(usize, f32)> = world_axes
                .iter()
                .enumerate()
                .map(|(i, (ax, _, _))| {
                    let cam = view3 * ax;
                    (i, cam.z) // negative z = towards camera = closer
                })
                .collect();
            draw_order.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap());

            for (i, _depth) in draw_order {
                let (ax, color, label) = &world_axes[i];
                let cam = view3 * ax;
                // Screen X = cam.x (right), Screen Y = -cam.y (egui Y is down)
                let tip = egui::pos2(
                    center.x + cam.x * axis_len,
                    center.y - cam.y * axis_len,
                );
                // Arrow line
                painter.line_segment(
                    [center, tip],
                    egui::Stroke::new(2.5, *color),
                );
                // Arrowhead (small circle)
                painter.circle_filled(tip, 4.0, *color);
                // Label
                painter.text(
                    tip + egui::vec2(6.0, -6.0),
                    egui::Align2::LEFT_BOTTOM,
                    *label,
                    egui::FontId::proportional(11.0),
                    *color,
                );
            }
        }

        // ===== Camera reset button (bottom-right, above axes) =====
        {
            let painter = ui.painter();
            let btn_size = egui::vec2(28.0, 28.0);
            let margin = 10.0;
            let btn_pos = egui::pos2(
                rect.right() - margin - 100.0 - btn_size.x,
                rect.bottom() - margin - btn_size.y,
            );
            let btn_rect = egui::Rect::from_min_size(btn_pos, btn_size);

            let btn_response = ui.interact(
                btn_rect,
                ui.id().with("camera_reset_btn"),
                egui::Sense::click(),
            );

            let bg_color = if btn_response.hovered() {
                egui::Color32::from_rgba_unmultiplied(100, 100, 100, 200)
            } else {
                egui::Color32::from_rgba_unmultiplied(40, 40, 50, 160)
            };
            painter.rect_filled(btn_rect, egui::CornerRadius::same(4), bg_color);

            painter.text(
                btn_rect.center(),
                egui::Align2::CENTER_CENTER,
                "⟲",
                egui::FontId::proportional(18.0),
                egui::Color32::from_gray(220),
            );

            if btn_response.clicked() {
                self.camera.reset();
            }

            if btn_response.hovered() {
                let tip_pos = egui::pos2(btn_rect.left(), btn_rect.top() - 4.0);
                painter.text(
                    tip_pos,
                    egui::Align2::LEFT_BOTTOM,
                    "Reset Camera",
                    egui::FontId::proportional(12.0),
                    egui::Color32::from_gray(220),
                );
            }
        }
    }
}

/// Compute the closest distance from a ray to a circle (ring) in 3D.
///
/// The ring has center `center`, normal `normal` (unit vector), and radius `radius`.
/// Returns the minimum distance from the ray to the ring circumference.
fn ray_ring_distance(
    ray_o: &na::Point3<f32>,
    ray_d: &na::Vector3<f32>,
    center: &na::Point3<f32>,
    normal: &na::Vector3<f32>,
    radius: f32,
) -> f32 {
    // Intersect ray with the plane of the ring
    let denom = ray_d.dot(normal);
    if denom.abs() < 1e-8 {
        // Ray is parallel to the ring plane — use distance to closest point on ring
        // Project ray origin onto ring plane
        let to_origin = ray_o - center;
        let plane_dist = to_origin.dot(normal);
        let proj = ray_o - normal * plane_dist;
        let from_center = proj - center;
        let r_proj = from_center.norm();
        if r_proj < 1e-8 {
            return (plane_dist * plane_dist + radius * radius).sqrt();
        }
        let closest_on_ring = center + from_center * (radius / r_proj);
        // Find closest point on ray to this ring point
        let to_ring = closest_on_ring - ray_o;
        let t = to_ring.dot(ray_d) / ray_d.dot(ray_d);
        let closest_on_ray = ray_o + ray_d * t.max(0.0);
        na::distance(&closest_on_ray, &closest_on_ring)
    } else {
        let t = (center - ray_o).dot(normal) / denom;
        let hit = ray_o + ray_d * t;
        let from_center = hit - center;
        let r_hit = from_center.norm();
        if r_hit < 1e-8 {
            // Ray hits exactly the center of the ring
            return radius;
        }
        // Distance from the intersection point to the ring circumference
        (r_hit - radius).abs()
    }
}

/// Extract geometry parameters as [f32; 3] for scale dragging.
/// Box: [hx, hy, hz], Cylinder: [radius, half_length, 0], Sphere: [radius, 0, 0], Mesh: [0,0,0]
fn geom_params(geom: &robot::GeomData) -> [f32; 3] {
    match geom {
        robot::GeomData::Box { hx, hy, hz } => [*hx, *hy, *hz],
        robot::GeomData::Cylinder { radius, half_length } => [*radius, *half_length, 0.0],
        robot::GeomData::Sphere { radius } => [*radius, 0.0, 0.0],
        robot::GeomData::Mesh { .. } => [0.0, 0.0, 0.0],
    }
}

/// Apply a per-axis scale delta to geometry.
/// `axis` is 0=X, 1=Y, 2=Z; `delta_t` is the drag displacement along
/// the element's local axis.
fn apply_geom_scale(geom: &mut robot::GeomData, axis: u8, initial: [f32; 3], delta: f32) {
    const MIN_DIM: f32 = 0.001;
    match geom {
        robot::GeomData::Box { hx, hy, hz } => {
            // Each axis maps to (hx, hy, hz) respectively
            match axis {
                0 => *hx = (initial[0] + delta).max(MIN_DIM),
                1 => *hy = (initial[1] + delta).max(MIN_DIM),
                _ => *hz = (initial[2] + delta).max(MIN_DIM),
            }
        }
        robot::GeomData::Cylinder { radius, half_length } => {
            // X/Y → radius, Z → half_length
            match axis {
                0 | 1 => *radius = (initial[0] + delta).max(MIN_DIM),
                _ => *half_length = (initial[1] + delta).max(MIN_DIM),
            }
        }
        robot::GeomData::Sphere { radius } => {
            // Any axis → radius
            *radius = (initial[0] + delta).max(MIN_DIM);
        }
        robot::GeomData::Mesh { .. } => {} // Cannot scale mesh
    }
}

/// Compute the angle of a ray's intersection with a ring's plane,
/// measured from an arbitrary but consistent reference direction.
fn compute_ring_angle(
    ray_o: &na::Point3<f32>,
    ray_d: &na::Vector3<f32>,
    center: &na::Point3<f32>,
    normal: &na::Vector3<f32>,
) -> f32 {
    let denom = ray_d.dot(normal);
    if denom.abs() < 1e-8 {
        return 0.0;
    }
    let t = (center - ray_o).dot(normal) / denom;
    let hit = ray_o + ray_d * t;
    let from_center = hit - center;

    // Build a consistent right-handed reference frame on the plane.
    // ref_x × ref_y should point along `normal` so that the
    // angle increases in the positive (right-hand rule) direction.
    let ref_x = if normal.x.abs() < 0.9 {
        na::Vector3::x().cross(normal).normalize()
    } else {
        na::Vector3::y().cross(normal).normalize()
    };
    let ref_y = normal.cross(&ref_x);

    from_center.dot(&ref_y).atan2(from_center.dot(&ref_x))
}

// ========== eframe::App ==========

impl eframe::App for ArticaraApp {
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        let ctx = ui.ctx().clone();

        // Update transforms every frame (joint positions may have changed)
        if let Some(ref model) = self.model {
            let transforms = model.compute_transforms();
            let mut r = self.gl_renderer.lock().unwrap();
            r.update_transforms(transforms);
            r.show_com = self.show_com;
            r.com_scale = self.com_scale;
            r.wireframe = self.wireframe;
            r.visual_mode = self.visual_mode;
            r.collision_mode = self.collision_mode;
            r.link_display_modes = self.link_display_modes.clone();
        }

        // Top panel: menu / file selector
        egui::Panel::top("menu_bar").show_inside(ui, |ui| {
            self.draw_menu_bar(ui);
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

        // Upload robot geometry to GPU when needed.
        // Placed AFTER all UI panels so that add/remove operations in this frame
        // set needs_upload=true and are captured here in the same frame.
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
    }

    fn on_exit(&mut self, gl: Option<&glow::Context>) {
        if let Some(gl) = gl {
            self.gl_renderer.lock().unwrap().destroy(gl);
        }
    }
}
