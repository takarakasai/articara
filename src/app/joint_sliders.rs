use eframe::egui;
use nalgebra as na;

use super::{ArticaraApp, DragMode, InteractionMode};

impl ArticaraApp {
    pub(super) fn draw_joint_sliders(&mut self, ui: &mut egui::Ui) {
        if let Some(model) = &mut self.model {
            let mut changed = false;

            // --- IK parameters (only shown in JointDrive + IK mode) ---
            if self.interaction_mode == InteractionMode::JointDrive
                && self.drag_mode == DragMode::InverseKinematics
            {
                egui::CollapsingHeader::new("IK Parameters")
                    .default_open(true)
                    .show(ui, |ui| {
                        ui.horizontal(|ui| {
                            ui.label("Damping (λ):");
                            ui.add(
                                egui::Slider::new(&mut self.ik_damping, 0.001..=0.5)
                                    .logarithmic(true)
                                    .text("λ"),
                            );
                        });
                        // IK root link selector
                        let link_names: Vec<String> =
                            model.links.iter().map(|l| l.name.clone()).collect();
                        let prev_ik_root = self.ik_root_link.clone();
                        ui.horizontal(|ui| {
                            ui.label("Root:");
                            let current_label = match &self.ik_root_link {
                                None => "Auto (URDF Root)".to_string(),
                                Some(name) => name.clone(),
                            };
                            egui::ComboBox::from_id_salt("ik_root_link")
                                .selected_text(&current_label)
                                .show_ui(ui, |ui| {
                                    if ui
                                        .selectable_label(
                                            self.ik_root_link.is_none(),
                                            "Auto (URDF Root)",
                                        )
                                        .clicked()
                                    {
                                        self.ik_root_link = None;
                                    }
                                    for name in &link_names {
                                        let selected =
                                            self.ik_root_link.as_deref() == Some(name.as_str());
                                        if ui.selectable_label(selected, name).clicked() {
                                            self.ik_root_link = Some(name.clone());
                                        }
                                    }
                                });
                        });
                        if self.ik_root_link != prev_ik_root {
                            model.base_transform = na::Isometry3::identity();
                        }
                    });
                ui.separator();
            }

            // --- Joint sliders (collapsible) ---
            let header = egui::CollapsingHeader::new("Joint Positions")
                .default_open(false)
                .show(ui, |ui| {
                    for i in 0..model.joints.len() {
                        if model.joints[i].joint_type == "fixed" {
                            continue;
                        }
                        let lower = model.joints[i].lower;
                        let upper = model.joints[i].upper;
                        if lower >= upper {
                            continue;
                        }
                        let name = model.joints[i].name.clone();
                        ui.horizontal(|ui| {
                            ui.set_min_width(200.0);
                            ui.label(&name);
                        });
                        // Slider operates on f64 (joint_positions is now Vec<f64>);
                        // egui natively supports f64 sliders.
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
                });
            let _ = header;

            // Joint slider changes only affect transforms, not geometry.
            // Do NOT touch needs_upload here — it may be true from add/remove operations.
            let _ = changed;

            let reset_clicked = ui.button("Reset All Joints").clicked();
            if reset_clicked {
                for pos in model.joint_positions.iter_mut() {
                    *pos = 0.0;
                }
                model.base_transform = na::Isometry3::identity();
            }
            drop(model);
            // Record undo after releasing the model borrow
            if changed {
                self.mark_edit("Set joint position");
            }
            if reset_clicked {
                self.mark_edit("Reset all joints");
            }
        }
    }
}
