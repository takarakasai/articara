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
                        // Solver selector
                        ui.horizontal(|ui| {
                            ui.label("Solver:");
                            egui::ComboBox::from_id_salt("ik_solver")
                                .selected_text(self.ik_solver.label())
                                .show_ui(ui, |ui| {
                                    for &s in &crate::robot::IkSolver::ALL {
                                        ui.selectable_value(&mut self.ik_solver, s, s.label());
                                    }
                                });
                        });
                        // DoF selector (2D screen vs 3D world)
                        ui.horizontal(|ui| {
                            ui.label("DoF:");
                            egui::ComboBox::from_id_salt("ik_dof")
                                .selected_text(self.ik_dof.label())
                                .show_ui(ui, |ui| {
                                    for &d in &crate::robot::IkDof::ALL {
                                        ui.selectable_value(&mut self.ik_dof, d, d.label());
                                    }
                                });
                        });
                        // Damping slider (not shown for JT which doesn't use it)
                        if self.ik_solver != crate::robot::IkSolver::JacobianTranspose {
                            ui.horizontal(|ui| {
                                let label = match self.ik_solver {
                                    crate::robot::IkSolver::SrInverse => "λ_max:",
                                    _ => "Damping (λ):",
                                };
                                ui.label(label);
                                ui.add(
                                    egui::Slider::new(&mut self.ik_damping, 0.001..=0.5)
                                        .logarithmic(true)
                                        .text("λ"),
                                );
                            });
                        }
                        // Joint weight gradient slider
                        ui.horizontal(|ui| {
                            ui.label("Weight:");
                            ui.add(
                                egui::Slider::new(&mut self.ik_weight_gradient, 0.0..=5.0)
                                    .text("EE-proximal")
                            );
                        })
                        .response
                        .on_hover_text("0 = uniform weights, larger = prefer joints near EE");
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

                // --- Pinned Links (multi-constraint IK) ---
                egui::CollapsingHeader::new("Pinned Links")
                    .default_open(true)
                    .show(ui, |ui| {
                        // Pin weight slider
                        ui.horizontal(|ui| {
                            ui.label("Pin weight:");
                            ui.add(
                                egui::Slider::new(&mut self.ik_pin_weight, 1.0..=100.0)
                                    .logarithmic(true)
                                    .text("w"),
                            );
                        })
                        .response
                        .on_hover_text("Constraint strength. Higher = pinned links move less.");

                        // Pin current selected link button
                        if let Some(li) = self.selected_link {
                            let link_name = model.links[li].name.clone();
                            let already_pinned = self.pinned_links.iter().any(|p| p.link_name == link_name);
                            if !already_pinned {
                                if ui.button(format!("📌 Pin \"{}\"", &link_name)).clicked() {
                                    let transforms = model.compute_transforms();
                                    let world_pos = model.ee_world_pos(li, &transforms);
                                    let world_rot = model.link_world_orientation(li, &transforms);
                                    self.pinned_links.push(super::PinnedLink {
                                        link_name: link_name.clone(),
                                        target_pos: world_pos.cast::<f64>(),
                                        target_rot: world_rot.cast::<f64>(),
                                        dof: super::PinDof::Position,
                                    });
                                }
                            }
                        }

                        // List pinned links with remove buttons
                        let mut remove_idx = None;
                        for (i, pin) in self.pinned_links.iter().enumerate() {
                            ui.horizontal(|ui| {
                                ui.label(format!("📌 {}", &pin.link_name));
                                ui.label(format!(
                                    "({:.3}, {:.3}, {:.3})",
                                    pin.target_pos.x, pin.target_pos.y, pin.target_pos.z
                                ));
                                if ui.small_button("✕").clicked() {
                                    remove_idx = Some(i);
                                }
                            });
                        }
                        if let Some(idx) = remove_idx {
                            self.pinned_links.remove(idx);
                        }

                        if !self.pinned_links.is_empty() {
                            // Update pin targets to current positions button
                            if ui.button("🔄 Re-pin all to current").on_hover_text(
                                "Update all pinned link targets to their current world positions"
                            ).clicked() {
                                let transforms = model.compute_transforms();
                                for pin in &mut self.pinned_links {
                                    if let Some(&li) = model.link_map.get(&pin.link_name) {
                                        let pos = model.ee_world_pos(li, &transforms);
                                        pin.target_pos = pos.cast::<f64>();
                                        pin.target_rot = model.link_world_orientation(li, &transforms).cast::<f64>();
                                    }
                                }
                            }
                            if ui.button("🗑 Clear all pins").clicked() {
                                self.pinned_links.clear();
                            }
                        }

                        if self.pinned_links.is_empty() {
                            ui.label("No links pinned. Select a link and click Pin.");
                        }
                    });

                // --- Chicken Head (auto-pin at drag start) ---
                egui::CollapsingHeader::new("Chicken Head")
                    .default_open(false)
                    .show(ui, |ui| {
                        ui.label("Links auto-pinned during IK drag:");

                        // DoF selector
                        ui.horizontal(|ui| {
                            ui.label("DoF:");
                            for dof in super::PinDof::ALL {
                                ui.selectable_value(
                                    &mut self.chicken_head_dof,
                                    dof,
                                    dof.label(),
                                );
                            }
                        });

                        // Add selected link button
                        if let Some(li) = self.selected_link {
                            let link_name = &model.links[li].name;
                            let already = self.chicken_head_links.iter().any(|n| n == link_name);
                            if !already {
                                if ui.button(format!("🐔 Add \"{}\"", link_name)).clicked() {
                                    self.chicken_head_links.push(link_name.clone());
                                }
                            }
                        }

                        // List with remove buttons
                        let mut ch_remove = None;
                        for (i, name) in self.chicken_head_links.iter().enumerate() {
                            ui.horizontal(|ui| {
                                ui.label(format!("🐔 {}", name));
                                if ui.small_button("✕").clicked() {
                                    ch_remove = Some(i);
                                }
                            });
                        }
                        if let Some(idx) = ch_remove {
                            self.chicken_head_links.remove(idx);
                        }

                        if !self.chicken_head_links.is_empty() {
                            if ui.button("🗑 Clear all").clicked() {
                                self.chicken_head_links.clear();
                            }
                        }

                        if self.chicken_head_links.is_empty() {
                            ui.label("Select a link and click Add.");
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
