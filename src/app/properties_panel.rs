use eframe::egui;
use nalgebra as na;
use std::path::PathBuf;

use super::{ArticaraApp, InteractionMode, OffsetTarget};
use crate::format::RobotFormat;
use crate::renderer::{DisplayMode, MeshKind};
use crate::robot::GeomData;

impl ArticaraApp {
    pub(super) fn draw_properties_panel(&mut self, ui: &mut egui::Ui) {
        ui.heading("Properties");
        ui.separator();

        // Track whether any property was edited this frame
        let mut props_edit_desc: Option<String> = None;

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
                        let mut inertial_changed = false;
                        egui::Grid::new("inertial_grid")
                            .striped(true)
                            .num_columns(2)
                            .show(ui, |ui| {
                                ui.label("Mass (kg):");
                                inertial_changed |= ui.add(egui::DragValue::new(&mut link.inertial.mass).speed(0.001).range(0.0..=f64::MAX)).changed();
                                ui.end_row();

                                ui.label("Origin xyz:");
                                ui.horizontal(|ui| {
                                    let t = &mut link.inertial.origin.translation;
                                    inertial_changed |= ui.add(egui::DragValue::new(&mut t.x).speed(0.0001).prefix("x:")).changed();
                                    inertial_changed |= ui.add(egui::DragValue::new(&mut t.y).speed(0.0001).prefix("y:")).changed();
                                    inertial_changed |= ui.add(egui::DragValue::new(&mut t.z).speed(0.0001).prefix("z:")).changed();
                                });
                                ui.end_row();

                                ui.label("Ixx:");
                                inertial_changed |= ui.add(egui::DragValue::new(&mut link.inertial.ixx).speed(0.000001)).changed();
                                ui.end_row();
                                ui.label("Ixy:");
                                inertial_changed |= ui.add(egui::DragValue::new(&mut link.inertial.ixy).speed(0.000001)).changed();
                                ui.end_row();
                                ui.label("Ixz:");
                                inertial_changed |= ui.add(egui::DragValue::new(&mut link.inertial.ixz).speed(0.000001)).changed();
                                ui.end_row();
                                ui.label("Iyy:");
                                inertial_changed |= ui.add(egui::DragValue::new(&mut link.inertial.iyy).speed(0.000001)).changed();
                                ui.end_row();
                                ui.label("Iyz:");
                                inertial_changed |= ui.add(egui::DragValue::new(&mut link.inertial.iyz).speed(0.000001)).changed();
                                ui.end_row();
                                ui.label("Izz:");
                                inertial_changed |= ui.add(egui::DragValue::new(&mut link.inertial.izz).speed(0.000001)).changed();
                                ui.end_row();
                            });

                        // --- Auto-compute inertia from geometry ---
                        ui.add_space(4.0);
                        ui.horizontal(|ui| {
                            if ui.button("⚡ Auto-compute from geometry")
                                .on_hover_text("Compute inertia tensor assuming uniform density, based on visual geometries and current mass")
                                .clicked()
                            {
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
                                inertial_changed = true;
                            }
                        });

                        // --- Compute mass from density + volume ---
                        ui.horizontal(|ui| {
                            if ui.button("📏 Mass from density")
                                .on_hover_text("Set mass = density × total volume, then recompute inertia tensor")
                                .clicked()
                            {
                                self.show_density_input = !self.show_density_input;
                            }
                        });
                        if self.show_density_input {
                            ui.horizontal(|ui| {
                                ui.label("Density (kg/m³):");
                                ui.add(egui::DragValue::new(&mut self.density_value).speed(1.0).range(0.1..=50000.0));
                            });
                            // Show computed volume and resulting mass
                            let total_vol: f64 = link.visuals.iter()
                                .map(|v| crate::robot::compute_geometry_volume(&v.geometry))
                                .sum();
                            let new_mass = self.density_value * total_vol;
                            ui.label(egui::RichText::new(
                                format!("Volume: {:.6} m³ → Mass: {:.4} kg", total_vol, new_mass)
                            ).small());
                            ui.horizontal(|ui| {
                                if ui.button("✔ Apply").clicked() {
                                    link.inertial.mass = new_mass;
                                    let computed = crate::robot::compute_link_inertia(
                                        &link.visuals,
                                        new_mass,
                                    );
                                    link.inertial.origin = computed.origin;
                                    link.inertial.ixx = computed.ixx;
                                    link.inertial.ixy = computed.ixy;
                                    link.inertial.ixz = computed.ixz;
                                    link.inertial.iyy = computed.iyy;
                                    link.inertial.iyz = computed.iyz;
                                    link.inertial.izz = computed.izz;
                                    inertial_changed = true;
                                    self.show_density_input = false;
                                }
                                if ui.button("✖ Cancel").clicked() {
                                    self.show_density_input = false;
                                }
                            });
                            // Show common material reference
                            ui.label(egui::RichText::new(
                                "ABS: 1050 / Al: 2700 / Steel: 7800 / PLA: 1240"
                            ).small().weak());
                        }

                        // --- Inertia validation ---
                        ui.add_space(4.0);
                        let validation = crate::robot::validate_inertia(link);
                        if validation.is_ok() {
                            ui.label(egui::RichText::new("✅ Inertia OK")
                                .small().color(egui::Color32::from_rgb(80, 200, 80)));
                        } else {
                            for issue in &validation.issues {
                                let (icon, color) = match issue.severity {
                                    crate::robot::ValidationSeverity::Error =>
                                        ("❌", egui::Color32::from_rgb(220, 60, 60)),
                                    crate::robot::ValidationSeverity::Warning =>
                                        ("⚠", egui::Color32::from_rgb(220, 180, 40)),
                                };
                                ui.label(egui::RichText::new(format!("{icon} {}", issue.message))
                                    .small().color(color));
                            }
                        }

                        if inertial_changed {
                            props_edit_desc = Some(format!("Edit inertial of '{}'", link_name));
                        }
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
                            let is_selected = self.interaction_mode == InteractionMode::OffsetAdjust
                                && self.offset_target == OffsetTarget::Visual
                                && self.selected_visual == Some(vi);
                            let header_resp = ui.selectable_label(
                                is_selected,
                                egui::RichText::new(format!("Visual #{vi}")).strong(),
                            );
                            if header_resp.clicked() {
                                self.interaction_mode = InteractionMode::OffsetAdjust;
                                self.offset_target = OffsetTarget::Visual;
                                self.selected_visual = Some(vi);
                            }
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
                                geom_changed |= ui.add(egui::DragValue::new(&mut t.x).speed(0.005).prefix("x:")).changed();
                                geom_changed |= ui.add(egui::DragValue::new(&mut t.y).speed(0.005).prefix("y:")).changed();
                                geom_changed |= ui.add(egui::DragValue::new(&mut t.z).speed(0.005).prefix("z:")).changed();
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
                        props_edit_desc = Some(format!("Duplicate visual of '{}'", link_name));
                    }
                    if let Some(idx) = vis_to_remove {
                        link.visuals.remove(idx);
                        geom_changed = true;
                        props_edit_desc = Some(format!("Remove visual from '{}'", link_name));
                    }
                    if geom_changed {
                        self.needs_upload = true;
                        if props_edit_desc.is_none() {
                            props_edit_desc = Some(format!("Edit visual of '{}'", link_name));
                        }
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
                        props_edit_desc = Some(format!("Add visual to '{}'", link_name));
                        ui.close();
                    }
                    if ui.button("➕ Add Cylinder").clicked() {
                        link.visuals.push(crate::robot::VisualData {
                            origin: na::Isometry3::identity(),
                            geometry: GeomData::Cylinder { radius: 0.02, half_length: 0.1 },
                            color: [0.7, 0.7, 0.7, 1.0],
                        });
                        self.needs_upload = true;
                        props_edit_desc = Some(format!("Add visual to '{}'", link_name));
                        ui.close();
                    }
                    if ui.button("➕ Add Sphere").clicked() {
                        link.visuals.push(crate::robot::VisualData {
                            origin: na::Isometry3::identity(),
                            geometry: GeomData::Sphere { radius: 0.05 },
                            color: [0.7, 0.7, 0.7, 1.0],
                        });
                        self.needs_upload = true;
                        props_edit_desc = Some(format!("Add visual to '{}'", link_name));
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
                            let is_selected = self.interaction_mode == InteractionMode::OffsetAdjust
                                && self.offset_target == OffsetTarget::Collision
                                && self.selected_collision == Some(ci);
                            let col_item_resp = ui.selectable_label(
                                is_selected,
                                egui::RichText::new(format!("Collision #{ci}")).strong(),
                            );
                            if col_item_resp.clicked() {
                                self.interaction_mode = InteractionMode::OffsetAdjust;
                                self.offset_target = OffsetTarget::Collision;
                                self.selected_collision = Some(ci);
                            }
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
                                col_changed |= ui.add(egui::DragValue::new(&mut t.x).speed(0.005).prefix("x:")).changed();
                                col_changed |= ui.add(egui::DragValue::new(&mut t.y).speed(0.005).prefix("y:")).changed();
                                col_changed |= ui.add(egui::DragValue::new(&mut t.z).speed(0.005).prefix("z:")).changed();
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
                        props_edit_desc = Some(format!("Duplicate collision of '{}'", link_name));
                    }
                    if let Some(idx) = col_to_remove {
                        link.collisions.remove(idx);
                        col_changed = true;
                        props_edit_desc = Some(format!("Remove collision from '{}'", link_name));
                    }
                    if col_changed {
                        self.needs_upload = true;
                        if props_edit_desc.is_none() {
                            props_edit_desc = Some(format!("Edit collision of '{}'", link_name));
                        }
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
                        props_edit_desc = Some(format!("Add collision to '{}'", link_name));
                        ui.close();
                    }
                    if ui.button("➕ Add Cylinder").clicked() {
                        link.collisions.push(crate::robot::CollisionData {
                            origin: na::Isometry3::identity(),
                            geometry: GeomData::Cylinder { radius: 0.02, half_length: 0.1 },
                        });
                        self.needs_upload = true;
                        props_edit_desc = Some(format!("Add collision to '{}'", link_name));
                        ui.close();
                    }
                    if ui.button("➕ Add Sphere").clicked() {
                        link.collisions.push(crate::robot::CollisionData {
                            origin: na::Isometry3::identity(),
                            geometry: GeomData::Sphere { radius: 0.05 },
                        });
                        self.needs_upload = true;
                        props_edit_desc = Some(format!("Add collision to '{}'", link_name));
                        ui.close();
                    }
                });
            }

            if let Some(ji) = self.selected_joint {
                let joint = &mut model.joints[ji];
                let joint_name = joint.name.clone();
                ui.label(egui::RichText::new(&joint_name).strong().size(16.0));
                ui.separator();
                let mut joint_changed = false;
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
                            joint_changed |= ui.add(egui::DragValue::new(&mut joint.axis.x).speed(0.01).prefix("x:")).changed();
                            joint_changed |= ui.add(egui::DragValue::new(&mut joint.axis.y).speed(0.01).prefix("y:")).changed();
                            joint_changed |= ui.add(egui::DragValue::new(&mut joint.axis.z).speed(0.01).prefix("z:")).changed();
                        });
                        ui.end_row();

                        ui.label("Lower (rad):");
                        joint_changed |= ui.add(egui::DragValue::new(&mut joint.lower).speed(0.01)).changed();
                        ui.end_row();
                        ui.label("Upper (rad):");
                        joint_changed |= ui.add(egui::DragValue::new(&mut joint.upper).speed(0.01)).changed();
                        ui.end_row();
                        ui.label("Effort (Nm):");
                        joint_changed |= ui.add(egui::DragValue::new(&mut joint.effort).speed(0.1).range(0.0..=f64::MAX)).changed();
                        ui.end_row();
                        ui.label("Velocity (rad/s):");
                        joint_changed |= ui.add(egui::DragValue::new(&mut joint.velocity).speed(0.1).range(0.0..=f64::MAX)).changed();
                        ui.end_row();

                        ui.label("Origin xyz:");
                        ui.horizontal(|ui| {
                            let t = &mut joint.origin.translation;
                            joint_changed |= ui.add(egui::DragValue::new(&mut t.x).speed(0.0001).prefix("x:")).changed();
                            joint_changed |= ui.add(egui::DragValue::new(&mut t.y).speed(0.0001).prefix("y:")).changed();
                            joint_changed |= ui.add(egui::DragValue::new(&mut t.z).speed(0.0001).prefix("z:")).changed();
                        });
                        ui.end_row();
                    });
                if joint_changed {
                    self.needs_upload = true;
                    props_edit_desc = Some(format!("Edit joint '{}'", joint_name));
                }
            }

            if self.selected_link.is_none() && self.selected_joint.is_none() {
                ui.label("Select a link or joint to view properties.");
            }
        }

        // Commit any property edit to undo history
        if let Some(desc) = props_edit_desc {
            self.mark_edit(&desc);
        }


    }

    /// Draw the Export dialog window (format + directory + export button).
    pub(super) fn draw_export_dialog(&mut self, ctx: &egui::Context) {
        if !self.show_export_dialog {
            return;
        }

        let mut open = self.show_export_dialog;
        egui::Window::new("Export")
            .open(&mut open)
            .resizable(false)
            .default_width(360.0)
            .anchor(egui::Align2::CENTER_CENTER, egui::Vec2::ZERO)
            .show(ctx, |ui| {
                ui.horizontal(|ui| {
                    ui.label("Format:");
                    egui::ComboBox::from_id_salt("export_dlg_fmt")
                        .selected_text(self.export_format.label())
                        .show_ui(ui, |ui| {
                            for &fmt in RobotFormat::ALL {
                                if fmt.supports_export() {
                                    ui.selectable_value(&mut self.export_format, fmt, fmt.label());
                                }
                            }
                        });
                });
                ui.add_space(4.0);
                ui.horizontal(|ui| {
                    ui.label("Directory:");
                    ui.text_edit_singleline(&mut self.export_dir);
                    if ui.button("📂").on_hover_text("Browse…").clicked() {
                        let start = if self.export_dir.is_empty() {
                            None
                        } else {
                            Some(std::path::Path::new(&self.export_dir).to_path_buf())
                        };
                        self.dlg_export_dir.open(
                            "Select Export Directory",
                            super::file_dialog::FileDialogMode::ChooseDir,
                            start.as_deref(),
                            &[],
                        );
                    }
                });
                ui.add_space(8.0);
                ui.horizontal(|ui| {
                    if ui.button("📦 Export").clicked() {
                        self.do_export();
                        if self.export_message.starts_with("✔") {
                            self.show_export_dialog = false;
                        }
                    }
                    if ui.button("Cancel").clicked() {
                        self.show_export_dialog = false;
                    }
                });
                if !self.export_message.is_empty() {
                    ui.add_space(4.0);
                    let color = if self.export_message.starts_with("✔") {
                        egui::Color32::from_rgb(80, 200, 80)
                    } else {
                        egui::Color32::from_rgb(220, 180, 40)
                    };
                    ui.label(egui::RichText::new(&self.export_message).color(color));
                }
            });
        self.show_export_dialog = open;
    }

    pub(super) fn do_save(&mut self) {
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

    pub(super) fn do_export(&mut self) {
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
                match crate::usd::export_usda_to_dir(model, &dir) {
                    Ok(path) => {
                        self.export_message =
                            format!("✔ Exported USD ASCII to {}", path.display());
                    }
                    Err(e) => {
                        self.export_message = format!("⚠ USD export failed: {e}");
                    }
                }
            }
        }
    }
}
