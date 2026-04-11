use eframe::egui;
use nalgebra as na;
use std::path::PathBuf;
use std::sync::{Arc, Mutex};

use crate::camera::OrbitCamera;
use crate::ik;
use crate::renderer::GlRenderer;
use crate::robot::RobotModel;

/// Drag manipulation mode.
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
}

pub struct RoboViewApp {
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
    /// IK damping factor (λ for DLS).
    ik_damping: f32,
}

impl RoboViewApp {
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
            ik_damping: 0.05,
        }
    }

    pub fn load_urdf(&mut self, path: PathBuf) {
        match RobotModel::from_urdf(&path) {
            Ok(model) => {
                self.status_message = format!(
                    "Loaded: {} ({} links, {} joints)",
                    model.name,
                    model.links.len(),
                    model.joints.len()
                );
                self.model = Some(model);
                self.urdf_path_input = path.display().to_string();
                self.selected_link = None;
                self.selected_joint = None;
                self.needs_upload = true;
            }
            Err(e) => {
                self.status_message = format!("Error: {e}");
                log::error!("Failed to load URDF: {e}");
            }
        }
    }

    // ===== UI Panels =====

    fn draw_menu_bar(&mut self, ui: &mut egui::Ui) {
        ui.horizontal(|ui| {
            ui.label("URDF:");
            let response = ui.text_edit_singleline(&mut self.urdf_path_input);
            if (response.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Enter)))
                || ui.button("Load").clicked()
            {
                let path = PathBuf::from(&self.urdf_path_input);
                self.load_urdf(path);
            }
            ui.separator();
            ui.label(&self.status_message);
        });
    }

    fn draw_tree_panel(&mut self, ui: &mut egui::Ui) {
        if self.model.is_none() {
            ui.label("No model loaded.");
            ui.label("Enter a URDF path above and click Load.");
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
            egui::collapsing_header::CollapsingState::load_with_default_open(ui.ctx(), id, true)
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

    fn draw_joint_sliders(&mut self, ui: &mut egui::Ui) {
        if let Some(model) = &mut self.model {
            let mut changed = false;
            ui.heading("Joint Positions");
            ui.separator();

            // --- Drag Mode selector ---
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

            if changed {
                self.needs_upload = false; // Don't need to re-upload geometry, just update transforms
            }

            if ui.button("Reset All Joints").clicked() {
                for pos in model.joint_positions.iter_mut() {
                    *pos = 0.0;
                }
            }
        }
    }

    fn draw_properties_panel(&self, ui: &mut egui::Ui) {
        ui.heading("Properties");
        ui.separator();

        if let Some(model) = &self.model {
            if let Some(li) = self.selected_link {
                let link = &model.links[li];
                ui.label(egui::RichText::new(&link.name).strong().size(16.0));
                ui.separator();

                egui::CollapsingHeader::new("📐 Inertial")
                    .default_open(true)
                    .show(ui, |ui| {
                        egui::Grid::new("inertial_grid")
                            .striped(true)
                            .show(ui, |ui| {
                                ui.label("Mass:");
                                ui.label(format!("{:.6} kg", link.inertial.mass));
                                ui.end_row();
                                ui.label("Origin:");
                                ui.label(format!(
                                    "[{:.4}, {:.4}, {:.4}]",
                                    link.inertial.origin.translation.x,
                                    link.inertial.origin.translation.y,
                                    link.inertial.origin.translation.z
                                ));
                                ui.end_row();
                                ui.label("Ixx:");
                                ui.label(format!("{:.6}", link.inertial.ixx));
                                ui.end_row();
                                ui.label("Ixy:");
                                ui.label(format!("{:.6}", link.inertial.ixy));
                                ui.end_row();
                                ui.label("Ixz:");
                                ui.label(format!("{:.6}", link.inertial.ixz));
                                ui.end_row();
                                ui.label("Iyy:");
                                ui.label(format!("{:.6}", link.inertial.iyy));
                                ui.end_row();
                                ui.label("Iyz:");
                                ui.label(format!("{:.6}", link.inertial.iyz));
                                ui.end_row();
                                ui.label("Izz:");
                                ui.label(format!("{:.6}", link.inertial.izz));
                                ui.end_row();
                            });
                    });

                ui.add_space(8.0);
                egui::CollapsingHeader::new(format!(
                    "🎨 Visuals ({})",
                    link.visuals.len()
                ))
                .show(ui, |ui| {
                    for (vi, visual) in link.visuals.iter().enumerate() {
                        let geom_str = match &visual.geometry {
                            crate::robot::GeomData::Box { hx, hy, hz } => {
                                format!("Box [{:.3}×{:.3}×{:.3}]", hx * 2.0, hy * 2.0, hz * 2.0)
                            }
                            crate::robot::GeomData::Cylinder {
                                radius,
                                half_length,
                            } => {
                                format!("Cylinder r={radius:.3} l={:.3}", half_length * 2.0)
                            }
                            crate::robot::GeomData::Sphere { radius } => {
                                format!("Sphere r={radius:.3}")
                            }
                            crate::robot::GeomData::Mesh { vertices } => {
                                format!("Mesh ({} tris)", vertices.len() / 18)
                            }
                        };
                        ui.label(format!(
                            "#{vi}: {geom_str} color=[{:.2},{:.2},{:.2}]",
                            visual.color[0], visual.color[1], visual.color[2]
                        ));
                    }
                });

                egui::CollapsingHeader::new(format!(
                    "💥 Collisions ({})",
                    link.collisions.len()
                ))
                .show(ui, |ui| {
                    for (ci, col) in link.collisions.iter().enumerate() {
                        let geom_str = match &col.geometry {
                            crate::robot::GeomData::Box { hx, hy, hz } => {
                                format!("Box [{:.3}×{:.3}×{:.3}]", hx * 2.0, hy * 2.0, hz * 2.0)
                            }
                            crate::robot::GeomData::Cylinder {
                                radius,
                                half_length,
                            } => {
                                format!("Cylinder r={radius:.3} l={:.3}", half_length * 2.0)
                            }
                            crate::robot::GeomData::Sphere { radius } => {
                                format!("Sphere r={radius:.3}")
                            }
                            crate::robot::GeomData::Mesh { vertices } => {
                                format!("Mesh ({} tris)", vertices.len() / 18)
                            }
                        };
                        ui.label(format!("#{ci}: {geom_str}"));
                    }
                });
            }

            if let Some(ji) = self.selected_joint {
                let joint = &model.joints[ji];
                ui.label(egui::RichText::new(&joint.name).strong().size(16.0));
                ui.separator();
                egui::Grid::new("joint_props")
                    .striped(true)
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
                        ui.label(format!(
                            "[{:.3}, {:.3}, {:.3}]",
                            joint.axis.x, joint.axis.y, joint.axis.z
                        ));
                        ui.end_row();
                        ui.label("Lower:");
                        ui.label(format!("{:.4} rad", joint.lower));
                        ui.end_row();
                        ui.label("Upper:");
                        ui.label(format!("{:.4} rad", joint.upper));
                        ui.end_row();
                        ui.label("Effort:");
                        ui.label(format!("{:.4} Nm", joint.effort));
                        ui.end_row();
                        ui.label("Velocity:");
                        ui.label(format!("{:.4} rad/s", joint.velocity));
                        ui.end_row();
                        ui.label("Origin:");
                        ui.label(format!(
                            "[{:.4}, {:.4}, {:.4}]",
                            joint.origin.translation.x,
                            joint.origin.translation.y,
                            joint.origin.translation.z
                        ));
                        ui.end_row();
                    });
            }

            if self.selected_link.is_none() && self.selected_joint.is_none() {
                ui.label("Select a link or joint to view properties.");
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
            if self.drag_state.is_none() {
                let (ro, rd) = self.camera.screen_ray(ndc, aspect);
                self.hovered_link = model.pick_link(&ro, &rd, &transforms).map(|(li, _)| li);
            }
        } else {
            self.hovered_link = None;
        }

        // Left mouse button pressed: start drag if a link is hit
        if response.drag_started_by(egui::PointerButton::Primary) {
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
                                    });
                                    self.selected_link = Some(li);
                                    self.selected_joint = Some(ji);
                                }
                            }
                        }
                        DragMode::InverseKinematics => {
                            let chain = ik::build_chain(model, link_name);
                            if !chain.is_empty() {
                                // Use first joint for display selection
                                let ji = chain.last().map(|c| c.joint_idx).unwrap_or(0);
                                self.drag_state = Some(DragState {
                                    link_idx: li,
                                    mode: DragMode::InverseKinematics,
                                    joint_idx: ji,
                                    world_axis: na::Vector3::zeros(),
                                    pivot_world: na::Point3::origin(),
                                    chain,
                                });
                                self.selected_link = Some(li);
                            }
                        }
                    }
                }
            }
        }

        // While dragging: map mouse delta to joint angle change
        if response.dragged_by(egui::PointerButton::Primary) {
            if let Some(ref drag) = self.drag_state.clone() {
                let delta = response.drag_delta();
                if delta.length_sq() > 0.0 {
                    match drag.mode {
                        DragMode::SingleJoint => {
                            // Project joint axis to screen space to determine rotation direction
                            let pivot_screen = self
                                .camera
                                .project(&drag.pivot_world, aspect)
                                .unwrap_or(na::Point2::new(0.5, 0.5));
                            let axis_tip = drag.pivot_world + drag.world_axis * 0.05;
                            let tip_screen = self
                                .camera
                                .project(&axis_tip, aspect)
                                .unwrap_or(pivot_screen);

                            // Screen-space axis direction
                            let screen_axis = na::Vector2::new(
                                tip_screen.x - pivot_screen.x,
                                tip_screen.y - pivot_screen.y,
                            );
                            let screen_axis_len = screen_axis.norm();

                            if screen_axis_len > 1e-6 {
                                let screen_axis_norm = screen_axis / screen_axis_len;
                                let perp =
                                    na::Vector2::new(-screen_axis_norm.y, screen_axis_norm.x);
                                let delta_ndc = na::Vector2::new(
                                    delta.x / rect.width(),
                                    delta.y / rect.height(),
                                );
                                let angle_delta = delta_ndc.dot(&perp) * 5.0;

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
                                    }
                                }
                            }
                        }
                    }
                }
            } else {
                // No drag state: orbit camera as fallback
                self.camera.handle_orbit_pan_zoom(&response);
            }
        } else {
            // Not primary drag: camera controls (middle / right / scroll)
            self.camera.handle_orbit_pan_zoom(&response);
        }

        // Drag released
        if response.drag_stopped_by(egui::PointerButton::Primary) {
            self.drag_state = None;
        }

        // Update highlight in renderer
        {
            let highlight = if self.drag_state.is_some() {
                self.drag_state
                    .as_ref()
                    .and_then(|d| self.model.as_ref().map(|m| m.links[d.link_idx].name.clone()))
            } else {
                self.hovered_link
                    .and_then(|li| self.model.as_ref().map(|m| m.links[li].name.clone()))
            };
            self.gl_renderer.lock().unwrap().highlight_link = highlight;
        }

        // Change cursor when hovering a draggable link
        if self.drag_state.is_some() {
            ui.ctx().set_cursor_icon(egui::CursorIcon::Grabbing);
        } else if self.hovered_link.is_some() {
            // Check if the hovered link has a movable parent joint (or IK chain)
            let is_draggable = self.hovered_link.and_then(|li| {
                self.model.as_ref().and_then(|m| {
                    let link_name = &m.links[li].name;
                    match self.drag_mode {
                        DragMode::SingleJoint => m
                            .parent_joint_of_link(link_name)
                            .map(|ji| &m.joints[ji].joint_type)
                            .filter(|jt| *jt == "revolute" || *jt == "continuous"),
                        DragMode::InverseKinematics => {
                            let chain = ik::build_chain(m, link_name);
                            if chain.is_empty() {
                                None
                            } else {
                                Some(&m.joints[chain[0].joint_idx].joint_type)
                            }
                        }
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
    }
}

// ========== eframe::App ==========

impl eframe::App for RoboViewApp {
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        let ctx = ui.ctx().clone();

        // Upload robot to GPU if model just loaded
        if self.needs_upload {
            if let Some(ref model) = self.model {
                self.gl_renderer
                    .lock()
                    .unwrap()
                    .upload_robot(&self.gl, model);
                self.needs_upload = false;
            }
        }

        // Update transforms every frame (joint positions may have changed)
        if let Some(ref model) = self.model {
            let transforms = model.compute_transforms();
            self.gl_renderer
                .lock()
                .unwrap()
                .update_transforms(transforms);
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

        // Request continuous repaint for smooth camera interaction
        ctx.request_repaint();
    }

    fn on_exit(&mut self, gl: Option<&glow::Context>) {
        if let Some(gl) = gl {
            self.gl_renderer.lock().unwrap().destroy(gl);
        }
    }
}
