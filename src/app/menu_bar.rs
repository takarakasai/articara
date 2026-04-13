use eframe::egui;
use std::path::PathBuf;

use super::{ArticaraApp, DragMode, GizmoOp, InteractionMode, OffsetTarget};
use crate::renderer::DisplayMode;
use crate::robot::RobotModel;

impl ArticaraApp {
    pub(super) fn draw_menu_bar(&mut self, ui: &mut egui::Ui) {
        ui.horizontal(|ui| {
            if ui.button("📄 New").clicked() {
                self.model = Some(RobotModel::new_empty("new_robot"));
                self.selected_link = None;
                self.selected_joint = None;
                self.needs_upload = true;
                self.status_message = "Created new empty model".into();
                self.history.clear();
            }
            ui.separator();

            // ===== Edit menu =====
            ui.menu_button("Edit", |ui| {
                // --- Undo / Redo ---
                let undo_label = if let Some(desc) = self.history.undo_description() {
                    format!("↩ Undo: {}  (Ctrl+Z)", desc)
                } else {
                    "↩ Undo  (Ctrl+Z)".to_string()
                };
                if ui.add_enabled(self.history.can_undo(), egui::Button::new(&undo_label)).clicked() {
                    if let Some(ref mut model) = self.model {
                        if let Some(desc) = self.history.undo(model) {
                            self.status_message = format!("↩ Undo: {desc}");
                            self.needs_upload = true;
                        }
                    }
                    ui.close();
                }
                let redo_label = if let Some(desc) = self.history.redo_description() {
                    format!("↪ Redo: {}  (Ctrl+Shift+Z)", desc)
                } else {
                    "↪ Redo  (Ctrl+Shift+Z)".to_string()
                };
                if ui.add_enabled(self.history.can_redo(), egui::Button::new(&redo_label)).clicked() {
                    if let Some(ref mut model) = self.model {
                        if let Some(desc) = self.history.redo(model) {
                            self.status_message = format!("↪ Redo: {desc}");
                            self.needs_upload = true;
                        }
                    }
                    ui.close();
                }
                ui.separator();

                // --- Validate all inertia ---
                let has_model = self.model.is_some();
                if ui.add_enabled(has_model, egui::Button::new("🔍 Validate All Inertia"))
                    .on_hover_text("Check mass and inertia tensor consistency for every link")
                    .clicked()
                {
                    if let Some(ref model) = self.model {
                        self.validation_results = crate::robot::validate_all_inertia(model);
                        self.show_validation_window = true;
                    }
                    ui.close();
                }

                // --- Auto-compute inertia for ALL links ---
                if ui.add_enabled(has_model, egui::Button::new("⚡ Auto-compute All Inertia"))
                    .on_hover_text("Re-compute inertia tensors for every link from visual geometries (uniform density)")
                    .clicked()
                {
                    self.mark_edit("Auto-compute all inertia");
                    if let Some(ref mut model) = self.model {
                        let mut count = 0usize;
                        for link in &mut model.links {
                            if link.visuals.is_empty() || link.inertial.mass <= 0.0 {
                                continue;
                            }
                            let computed = crate::robot::compute_link_inertia(
                                &link.visuals,
                                link.inertial.mass,
                            );
                            link.inertial.origin = computed.origin;
                            link.inertial.ixx = computed.ixx;
                            link.inertial.ixy = computed.ixy;
                            link.inertial.ixz = computed.ixz;
                            link.inertial.iyy = computed.iyy;
                            link.inertial.iyz = computed.iyz;
                            link.inertial.izz = computed.izz;
                            count += 1;
                        }
                        self.needs_upload = true;
                        self.status_message = format!("⚡ Auto-computed inertia for {count} links");
                    }
                    ui.close();
                }
            });

            // ===== Posture menu =====
            ui.menu_button("Posture", |ui| {
                self.draw_posture_menu(ui);
            });

            // ===== View menu =====
            ui.menu_button("View", |ui| {
                ui.horizontal(|ui| {
                    ui.label("Visual:");
                    egui::ComboBox::from_id_salt("menu_visual_mode")
                        .selected_text(self.visual_mode.label())
                        .show_ui(ui, |ui| {
                            for m in DisplayMode::ALL {
                                ui.selectable_value(&mut self.visual_mode, m, m.label());
                            }
                        });
                });
                ui.horizontal(|ui| {
                    ui.label("Collision:");
                    egui::ComboBox::from_id_salt("menu_collision_mode")
                        .selected_text(self.collision_mode.label())
                        .show_ui(ui, |ui| {
                            for m in DisplayMode::ALL {
                                ui.selectable_value(&mut self.collision_mode, m, m.label());
                            }
                        });
                });
                ui.separator();
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
                ui.checkbox(&mut self.show_joint_axes, "Show Joint Axes");
                ui.separator();
                ui.checkbox(&mut self.show_ground_plane, "Show Ground Plane");
                if self.show_ground_plane {
                    ui.horizontal(|ui| {
                        ui.label("Ground Z:");
                        ui.add(
                            egui::DragValue::new(&mut self.ground_z)
                                .speed(0.01)
                                .suffix(" m"),
                        );
                    });
                    ui.horizontal(|ui| {
                        ui.label("Ground size:");
                        ui.add(
                            egui::DragValue::new(&mut self.ground_size)
                                .speed(0.1)
                                .range(0.1..=50.0)
                                .suffix(" m"),
                        );
                    });
                }
            });

            ui.separator();

            // ===== Mode toggle buttons (inline in toolbar) =====
            self.draw_mode_toolbar(ui);

            ui.separator();

            ui.label("File:");
            let response = ui.text_edit_singleline(&mut self.urdf_path_input);
            if (response.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Enter)))
                || ui.button("Load").clicked()
            {
                let path = PathBuf::from(&self.urdf_path_input);
                self.load_model(path);
            }
            if ui.button("📂").on_hover_text("Browse for model file…").clicked() {
                let start = if self.urdf_path_input.is_empty() {
                    None
                } else {
                    Some(std::path::Path::new(&self.urdf_path_input).to_path_buf())
                };
                self.dlg_open_model.open(
                    "Open Robot Model",
                    super::file_dialog::FileDialogMode::Open,
                    start.as_deref(),
                    &["urdf", "sdf", "xml", "mjcf", "usd", "usda"],
                );
            }
            ui.separator();
            ui.label(&self.status_message);
        });
    }

    /// Draw the inline mode toolbar: interaction mode toggles + context-sensitive sub-mode buttons.
    fn draw_mode_toolbar(&mut self, ui: &mut egui::Ui) {
        // --- Interaction mode toggle ---
        let jd = self.interaction_mode == InteractionMode::JointDrive;
        let oa = self.interaction_mode == InteractionMode::OffsetAdjust;
        if ui.selectable_label(jd, "🔧 Joint Drive").clicked() {
            self.interaction_mode = InteractionMode::JointDrive;
        }
        if ui.selectable_label(oa, "✥ Offset Adjust").clicked() {
            self.interaction_mode = InteractionMode::OffsetAdjust;
        }

        ui.separator();

        match self.interaction_mode {
            InteractionMode::JointDrive => {
                // Drag mode toggle
                let sj = self.drag_mode == DragMode::SingleJoint;
                let ik = self.drag_mode == DragMode::InverseKinematics;
                if ui.selectable_label(sj, "Single").on_hover_text("Drag rotates one joint").clicked() {
                    self.drag_mode = DragMode::SingleJoint;
                }
                if ui.selectable_label(ik, "IK").on_hover_text("Drag solves IK chain").clicked() {
                    self.drag_mode = DragMode::InverseKinematics;
                }
            }
            InteractionMode::OffsetAdjust => {
                // Offset target toggles
                for t in OffsetTarget::ALL {
                    let sel = self.offset_target == t;
                    if ui.selectable_label(sel, format!("{} {}", t.icon(), t.label())).clicked() {
                        self.offset_target = t;
                        if t == OffsetTarget::Joint && self.gizmo_op == GizmoOp::Scale {
                            self.gizmo_op = GizmoOp::Translate;
                        }
                    }
                }
                ui.separator();
                // Gizmo op toggles
                if ui.selectable_label(self.gizmo_op == GizmoOp::Translate, "⬌").on_hover_text("Translate").clicked() {
                    self.gizmo_op = GizmoOp::Translate;
                }
                if ui.selectable_label(self.gizmo_op == GizmoOp::Rotate, "↻").on_hover_text("Rotate").clicked() {
                    self.gizmo_op = GizmoOp::Rotate;
                }
                if self.offset_target != OffsetTarget::Joint {
                    if ui.selectable_label(self.gizmo_op == GizmoOp::Scale, "⬡").on_hover_text("Scale").clicked() {
                        self.gizmo_op = GizmoOp::Scale;
                    }
                }
            }
        }
    }
}
