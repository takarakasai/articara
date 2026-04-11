use eframe::egui;
use std::path::PathBuf;
use std::sync::{Arc, Mutex};

use crate::camera::OrbitCamera;
use crate::renderer::GlRenderer;
use crate::robot::RobotModel;

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

        // Handle camera interaction
        self.camera.handle_response(&response);

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
                    // Convert from egui coords (top-left origin) to GL coords (bottom-left)
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
