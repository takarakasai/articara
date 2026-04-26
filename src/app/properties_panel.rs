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
        // Deferred link rename (for app-level ref update after model borrow is released)
        let mut link_renamed: Option<(String, String)> = None;

        if let Some(model) = &mut self.model {
            if let Some(li) = self.selected_link {
                let link_name = model.links[li].name.clone();

                // Sync rename buffer when selection changes
                if self.rename_link_buf.is_empty() || !model.link_map.contains_key(&self.rename_link_buf) {
                    self.rename_link_buf = link_name.clone();
                }
                // Editable link name (before taking &mut link)
                let mut rename_result: Option<(String, String)> = None;
                ui.horizontal(|ui| {
                    ui.label("Link:");
                    let resp = ui.add(
                        egui::TextEdit::singleline(&mut self.rename_link_buf)
                            .desired_width(160.0)
                            .font(egui::TextStyle::Heading),
                    );
                    if resp.lost_focus() && self.rename_link_buf != link_name {
                        rename_result = Some((link_name.clone(), self.rename_link_buf.clone()));
                    }
                });
                if let Some((ref old_name, ref new_name)) = rename_result {
                    if model.rename_link(old_name, new_name) {
                        link_renamed = Some((old_name.clone(), new_name.clone()));
                        props_edit_desc = Some(format!("Rename link '{}' → '{}'", old_name, new_name));
                    } else {
                        self.rename_link_buf = model.links[li].name.clone();
                    }
                }
                // Re-read link_name after potential rename
                let link_name = model.links[li].name.clone();
                let link = &mut model.links[li];
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
                                GeomData::Capsule { radius, half_length } => {
                                    ui.label("Capsule:");
                                    ui.horizontal(|ui| {
                                        geom_changed |= ui.add(egui::DragValue::new(radius).speed(0.005).prefix("r:").range(0.001..=10.0)).changed();
                                        geom_changed |= ui.add(egui::DragValue::new(half_length).speed(0.005).prefix("hl:").range(0.001..=10.0)).changed();
                                    });
                                }
                                GeomData::Mesh { vertices, filename, .. } => {
                                    let tri_count = vertices.len() / 18;
                                    let fname = filename.as_deref().unwrap_or("(inline)");
                                    ui.label(format!("Mesh: {tri_count} tris — {fname}"));
                                    if tri_count > 4 {
                                        ui.horizontal(|ui| {
                                            ui.label("Algo:");
                                            egui::ComboBox::from_id_salt(format!("dec_vis_{vi}"))
                                                .width(100.0)
                                                .selected_text(self.decimation_method.label())
                                                .show_ui(ui, |ui| {
                                                    for m in misarta::decimate::DecimationMethod::ALL {
                                                        ui.selectable_value(
                                                            &mut self.decimation_method,
                                                            m,
                                                            m.label(),
                                                        ).on_hover_text(m.description());
                                                    }
                                                });
                                        });
                                        ui.horizontal(|ui| {
                                            for (label, ratio) in [("75%", 0.75), ("50%", 0.5), ("25%", 0.25), ("10%", 0.1)] {
                                                if ui.small_button(label).on_hover_text(
                                                    format!("Reduce to ~{} tris ({})", (tri_count as f64 * ratio).ceil() as usize, self.decimation_method.label())
                                                ).clicked() {
                                                    let mesh_data = misarta::mesh::MeshData::from_flat_vertices_f32(vertices);
                                                    let reduced = mesh_data.decimate_with(ratio, self.decimation_method);
                                                    *vertices = reduced.to_flat_vertices_f32();
                                                    geom_changed = true;
                                                    props_edit_desc = Some(format!("Reduce visual mesh of '{}' to {} ({})", link_name, label, self.decimation_method.label()));
                                                }
                                            }
                                        });
                                        // ── Decomposition (V-HACD / Sphere Tree) ──
                                        ui.horizontal(|ui| {
                                            ui.label("Decompose:");
                                            egui::ComboBox::from_id_salt(format!("decomp_vis_{vi}"))
                                                .width(100.0)
                                                .selected_text(self.decomposition_method.label())
                                                .show_ui(ui, |ui| {
                                                    for dm in misarta::decompose::DecompositionMethod::ALL {
                                                        ui.selectable_value(
                                                            &mut self.decomposition_method,
                                                            dm,
                                                            dm.label(),
                                                        ).on_hover_text(dm.description());
                                                    }
                                                });
                                        });
                                        ui.horizontal(|ui| {
                                            let busy = self.decompose_task.is_some();
                                            let btn = ui.add_enabled(
                                                !busy,
                                                egui::Button::new("▶ Decompose"),
                                            ).on_hover_text(
                                                if busy {
                                                    "Decomposition in progress…".to_string()
                                                } else {
                                                    format!("Replace this mesh with multiple visual shapes ({})", self.decomposition_method.label())
                                                }
                                            );
                                            if btn.clicked() {
                                                let mesh_data = misarta::mesh::MeshData::from_flat_vertices_f32(vertices);
                                                let origin = vis.origin;
                                                let color = vis.color;
                                                let method = self.decomposition_method;
                                                let progress = std::sync::Arc::new(std::sync::atomic::AtomicU8::new(
                                                    misarta::decompose::PHASE_NOT_STARTED,
                                                ));
                                                let sub_progress = std::sync::Arc::new(std::sync::atomic::AtomicU8::new(0));
                                                let prog_clone = std::sync::Arc::clone(&progress);
                                                let sub_clone = std::sync::Arc::clone(&sub_progress);
                                                let handle = std::thread::spawn(move || {
                                                    super::DecomposeResult::Visuals(match method {
                                                        misarta::decompose::DecompositionMethod::Vhacd => {
                                                            let hulls = misarta::decompose::vhacd_with_progress(
                                                                &mesh_data,
                                                                &misarta::decompose::VhacdParams::default(),
                                                                Some(&prog_clone),
                                                                Some(&sub_clone),
                                                            );
                                                            hulls.iter().map(|h| {
                                                                crate::robot::VisualData {
                                                                    origin,
                                                                    geometry: GeomData::Mesh {
                                                                        vertices: h.to_flat_vertices_f32(),
                                                                        filename: None,
                                                                        scale: None,
                                                                    },
                                                                    color,
                                                                }
                                                            }).collect::<Vec<_>>()
                                                        }
                                                        misarta::decompose::DecompositionMethod::SphereTree => {
                                                            let spheres = misarta::decompose::sphere_tree_with_progress(
                                                                &mesh_data,
                                                                &misarta::decompose::SphereTreeParams::default(),
                                                                Some(&prog_clone),
                                                                Some(&sub_clone),
                                                            );
                                                            spheres.iter().map(|s| {
                                                                let t = na::Translation3::new(s.center.x as f32, s.center.y as f32, s.center.z as f32);
                                                                let sphere_origin = origin * na::Isometry3::from_parts(t, na::UnitQuaternion::identity());
                                                                crate::robot::VisualData {
                                                                    origin: sphere_origin,
                                                                    geometry: GeomData::Sphere { radius: s.radius as f32 },
                                                                    color,
                                                                }
                                                            }).collect::<Vec<_>>()
                                                        }
                                                        misarta::decompose::DecompositionMethod::PrimitiveFit => {
                                                            let prims = misarta::decompose::primitive_fit_with_progress(
                                                                &mesh_data,
                                                                &misarta::decompose::VhacdParams::default(),
                                                                Some(&prog_clone),
                                                                Some(&sub_clone),
                                                            );
                                                            prims.iter().map(|p| {
                                                                let t = na::Translation3::new(
                                                                    p.center.x as f32,
                                                                    p.center.y as f32,
                                                                    p.center.z as f32,
                                                                );
                                                                let r = na::UnitQuaternion::new_normalize(na::Quaternion::new(
                                                                    p.rotation.w as f32,
                                                                    p.rotation.i as f32,
                                                                    p.rotation.j as f32,
                                                                    p.rotation.k as f32,
                                                                ));
                                                                let prim_origin = origin * na::Isometry3::from_parts(t, r);
                                                                let geometry = match p.kind {
                                                                    misarta::decompose::PrimitiveKind::Box { hx, hy, hz } => {
                                                                        GeomData::Box {
                                                                            hx: hx as f32,
                                                                            hy: hy as f32,
                                                                            hz: hz as f32,
                                                                        }
                                                                    }
                                                                    misarta::decompose::PrimitiveKind::Cylinder { radius, half_length } => {
                                                                        GeomData::Cylinder {
                                                                            radius: radius as f32,
                                                                            half_length: half_length as f32,
                                                                        }
                                                                    }
                                                                    misarta::decompose::PrimitiveKind::Sphere { radius } => {
                                                                        GeomData::Sphere { radius: radius as f32 }
                                                                    }
                                                                };
                                                                crate::robot::VisualData {
                                                                    origin: prim_origin,
                                                                    geometry,
                                                                    color,
                                                                }
                                                            }).collect::<Vec<_>>()
                                                        }
                                                        misarta::decompose::DecompositionMethod::PrimitiveFitDirect => {
                                                            let p = misarta::decompose::primitive_fit_direct_with_progress(
                                                                &mesh_data,
                                                                Some(&prog_clone),
                                                                Some(&sub_clone),
                                                            );
                                                            let t = na::Translation3::new(
                                                                p.center.x as f32,
                                                                p.center.y as f32,
                                                                p.center.z as f32,
                                                            );
                                                            let r = na::UnitQuaternion::new_normalize(na::Quaternion::new(
                                                                p.rotation.w as f32,
                                                                p.rotation.i as f32,
                                                                p.rotation.j as f32,
                                                                p.rotation.k as f32,
                                                            ));
                                                            let prim_origin = origin * na::Isometry3::from_parts(t, r);
                                                            let geometry = match p.kind {
                                                                misarta::decompose::PrimitiveKind::Box { hx, hy, hz } => {
                                                                    GeomData::Box {
                                                                        hx: hx as f32,
                                                                        hy: hy as f32,
                                                                        hz: hz as f32,
                                                                    }
                                                                }
                                                                misarta::decompose::PrimitiveKind::Cylinder { radius, half_length } => {
                                                                    GeomData::Cylinder {
                                                                        radius: radius as f32,
                                                                        half_length: half_length as f32,
                                                                    }
                                                                }
                                                                misarta::decompose::PrimitiveKind::Sphere { radius } => {
                                                                    GeomData::Sphere { radius: radius as f32 }
                                                                }
                                                            };
                                                            vec![crate::robot::VisualData {
                                                                origin: prim_origin,
                                                                geometry,
                                                                color,
                                                            }]
                                                        }
                                                    })
                                                });
                                                self.decompose_task = Some(super::DecomposeTask {
                                                    link_index: li,
                                                    slot_index: vi,
                                                    target: super::DecomposeTarget::Visual,
                                                    method,
                                                    progress,
                                                    sub_progress,
                                                    handle: Some(handle),
                                                    started: std::time::Instant::now(),
                                                });
                                            }
                                        });
                                    }
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
                    if ui.button("➕ Add Capsule").clicked() {
                        link.visuals.push(crate::robot::VisualData {
                            origin: na::Isometry3::identity(),
                            geometry: GeomData::Capsule { radius: 0.02, half_length: 0.1 },
                            color: [0.7, 0.7, 0.7, 1.0],
                        });
                        self.needs_upload = true;
                        props_edit_desc = Some(format!("Add visual to '{}'", link_name));
                        ui.close();
                    }
                    if ui.button("📦 Add Mesh (STL/DAE)…").clicked() {
                        self.add_mesh_target = Some(super::AddMeshTarget {
                            link_index: li,
                            kind: super::MeshAddKind::Visual,
                        });
                        self.dlg_add_mesh.open(
                            "メッシュファイルを開く (Visual)",
                            super::file_dialog::FileDialogMode::Open,
                            None,
                            &["stl", "dae"],
                        );
                        ui.close();
                    }
                });

                // Button to copy all visuals as collisions
                if !link.visuals.is_empty() {
                    if ui.button("📋 Copy Visuals → Collisions").on_hover_text(
                        "Replace all collision shapes with copies of the visual shapes"
                    ).clicked() {
                        link.collisions = link.visuals.iter().map(|v| {
                            crate::robot::CollisionData {
                                origin: v.origin,
                                geometry: v.geometry.clone(),
                            }
                        }).collect();
                        self.needs_upload = true;
                        props_edit_desc = Some(format!("Copy visuals → collisions for '{}'", link_name));
                    }
                }

                let col_count = link.collisions.len();
                let col_header = egui::CollapsingHeader::new(format!(
                    "💥 Collisions ({col_count})"
                ))
                .show(ui, |ui| {
                    let mut col_changed = false;
                    let mut col_to_remove: Option<usize> = None;
                    let mut col_to_duplicate: Option<usize> = None;
                    // Deferred decomposition: (index, replacement CollisionData list)
                    let col_decompose: Option<(usize, Vec<crate::robot::CollisionData>)> = None;
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
                            GeomData::Capsule { radius, half_length } => {
                                    ui.horizontal(|ui| {
                                        col_changed |= ui.add(egui::DragValue::new(radius).speed(0.005).prefix("r:").range(0.001..=10.0)).changed();
                                        col_changed |= ui.add(egui::DragValue::new(half_length).speed(0.005).prefix("hl:").range(0.001..=10.0)).changed();
                                    });
                                }
                                GeomData::Mesh { vertices, .. } => {
                                    let tri_count = vertices.len() / 18;
                                    ui.label(format!("Mesh ({tri_count} tris)"));
                                    if tri_count > 4 {
                                        ui.horizontal(|ui| {
                                            ui.label("Algo:");
                                            egui::ComboBox::from_id_salt(format!("dec_col_{ci}"))
                                                .width(100.0)
                                                .selected_text(self.decimation_method.label())
                                                .show_ui(ui, |ui| {
                                                    for m in misarta::decimate::DecimationMethod::ALL {
                                                        ui.selectable_value(
                                                            &mut self.decimation_method,
                                                            m,
                                                            m.label(),
                                                        ).on_hover_text(m.description());
                                                    }
                                                });
                                        });
                                        ui.horizontal(|ui| {
                                            for (label, ratio) in [("75%", 0.75), ("50%", 0.5), ("25%", 0.25), ("10%", 0.1)] {
                                                if ui.small_button(label).on_hover_text(
                                                    format!("Reduce to ~{} tris ({})", (tri_count as f64 * ratio).ceil() as usize, self.decimation_method.label())
                                                ).clicked() {
                                                    let mesh_data = misarta::mesh::MeshData::from_flat_vertices_f32(vertices);
                                                    let reduced = mesh_data.decimate_with(ratio, self.decimation_method);
                                                    *vertices = reduced.to_flat_vertices_f32();
                                                    col_changed = true;
                                                    props_edit_desc = Some(format!("Reduce collision mesh of '{}' to {} ({})", link_name, label, self.decimation_method.label()));
                                                }
                                            }
                                        });
                                        // ── Decomposition (V-HACD / Sphere Tree) ──
                                        ui.horizontal(|ui| {
                                            ui.label("Decompose:");
                                            egui::ComboBox::from_id_salt(format!("decomp_col_{ci}"))
                                                .width(100.0)
                                                .selected_text(self.decomposition_method.label())
                                                .show_ui(ui, |ui| {
                                                    for dm in misarta::decompose::DecompositionMethod::ALL {
                                                        ui.selectable_value(
                                                            &mut self.decomposition_method,
                                                            dm,
                                                            dm.label(),
                                                        ).on_hover_text(dm.description());
                                                    }
                                                });
                                        });
                                        ui.horizontal(|ui| {
                                            let busy = self.decompose_task.is_some();
                                            let btn = ui.add_enabled(
                                                !busy,
                                                egui::Button::new("▶ Decompose"),
                                            ).on_hover_text(
                                                if busy {
                                                    "Decomposition in progress…".to_string()
                                                } else {
                                                    format!("Replace this mesh with multiple collision shapes ({})", self.decomposition_method.label())
                                                }
                                            );
                                            if btn.clicked() {
                                                let mesh_data = misarta::mesh::MeshData::from_flat_vertices_f32(vertices);
                                                let origin = col.origin;
                                                let method = self.decomposition_method;
                                                let progress = std::sync::Arc::new(std::sync::atomic::AtomicU8::new(
                                                    misarta::decompose::PHASE_NOT_STARTED,
                                                ));
                                                let sub_progress = std::sync::Arc::new(std::sync::atomic::AtomicU8::new(0));
                                                let prog_clone = std::sync::Arc::clone(&progress);
                                                let sub_clone = std::sync::Arc::clone(&sub_progress);
                                                let handle = std::thread::spawn(move || {
                                                    super::DecomposeResult::Collisions(match method {
                                                        misarta::decompose::DecompositionMethod::Vhacd => {
                                                            let hulls = misarta::decompose::vhacd_with_progress(
                                                                &mesh_data,
                                                                &misarta::decompose::VhacdParams::default(),
                                                                Some(&prog_clone),
                                                                Some(&sub_clone),
                                                            );
                                                            hulls.iter().map(|h| {
                                                                crate::robot::CollisionData {
                                                                    origin,
                                                                    geometry: GeomData::Mesh {
                                                                        vertices: h.to_flat_vertices_f32(),
                                                                        filename: None,
                                                                        scale: None,
                                                                    },
                                                                }
                                                            }).collect::<Vec<_>>()
                                                        }
                                                        misarta::decompose::DecompositionMethod::SphereTree => {
                                                            let spheres = misarta::decompose::sphere_tree_with_progress(
                                                                &mesh_data,
                                                                &misarta::decompose::SphereTreeParams::default(),
                                                                Some(&prog_clone),
                                                                Some(&sub_clone),
                                                            );
                                                            spheres.iter().map(|s| {
                                                                let t = na::Translation3::new(s.center.x as f32, s.center.y as f32, s.center.z as f32);
                                                                let sphere_origin = origin * na::Isometry3::from_parts(t, na::UnitQuaternion::identity());
                                                                crate::robot::CollisionData {
                                                                    origin: sphere_origin,
                                                                    geometry: GeomData::Sphere { radius: s.radius as f32 },
                                                                }
                                                            }).collect::<Vec<_>>()
                                                        }
                                                        misarta::decompose::DecompositionMethod::PrimitiveFit => {
                                                            let prims = misarta::decompose::primitive_fit_with_progress(
                                                                &mesh_data,
                                                                &misarta::decompose::VhacdParams::default(),
                                                                Some(&prog_clone),
                                                                Some(&sub_clone),
                                                            );
                                                            prims.iter().map(|p| {
                                                                let t = na::Translation3::new(
                                                                    p.center.x as f32,
                                                                    p.center.y as f32,
                                                                    p.center.z as f32,
                                                                );
                                                                let r = na::UnitQuaternion::new_normalize(na::Quaternion::new(
                                                                    p.rotation.w as f32,
                                                                    p.rotation.i as f32,
                                                                    p.rotation.j as f32,
                                                                    p.rotation.k as f32,
                                                                ));
                                                                let prim_origin = origin * na::Isometry3::from_parts(t, r);
                                                                let geometry = match p.kind {
                                                                    misarta::decompose::PrimitiveKind::Box { hx, hy, hz } => {
                                                                        GeomData::Box {
                                                                            hx: hx as f32,
                                                                            hy: hy as f32,
                                                                            hz: hz as f32,
                                                                        }
                                                                    }
                                                                    misarta::decompose::PrimitiveKind::Cylinder { radius, half_length } => {
                                                                        GeomData::Cylinder {
                                                                            radius: radius as f32,
                                                                            half_length: half_length as f32,
                                                                        }
                                                                    }
                                                                    misarta::decompose::PrimitiveKind::Sphere { radius } => {
                                                                        GeomData::Sphere { radius: radius as f32 }
                                                                    }
                                                                };
                                                                crate::robot::CollisionData {
                                                                    origin: prim_origin,
                                                                    geometry,
                                                                }
                                                            }).collect::<Vec<_>>()
                                                        }
                                                        misarta::decompose::DecompositionMethod::PrimitiveFitDirect => {
                                                            let p = misarta::decompose::primitive_fit_direct_with_progress(
                                                                &mesh_data,
                                                                Some(&prog_clone),
                                                                Some(&sub_clone),
                                                            );
                                                            let t = na::Translation3::new(
                                                                p.center.x as f32,
                                                                p.center.y as f32,
                                                                p.center.z as f32,
                                                            );
                                                            let r = na::UnitQuaternion::new_normalize(na::Quaternion::new(
                                                                p.rotation.w as f32,
                                                                p.rotation.i as f32,
                                                                p.rotation.j as f32,
                                                                p.rotation.k as f32,
                                                            ));
                                                            let prim_origin = origin * na::Isometry3::from_parts(t, r);
                                                            let geometry = match p.kind {
                                                                misarta::decompose::PrimitiveKind::Box { hx, hy, hz } => {
                                                                    GeomData::Box {
                                                                        hx: hx as f32,
                                                                        hy: hy as f32,
                                                                        hz: hz as f32,
                                                                    }
                                                                }
                                                                misarta::decompose::PrimitiveKind::Cylinder { radius, half_length } => {
                                                                    GeomData::Cylinder {
                                                                        radius: radius as f32,
                                                                        half_length: half_length as f32,
                                                                    }
                                                                }
                                                                misarta::decompose::PrimitiveKind::Sphere { radius } => {
                                                                    GeomData::Sphere { radius: radius as f32 }
                                                                }
                                                            };
                                                            vec![crate::robot::CollisionData {
                                                                origin: prim_origin,
                                                                geometry,
                                                            }]
                                                        }
                                                    })
                                                });
                                                self.decompose_task = Some(super::DecomposeTask {
                                                    link_index: li,
                                                    slot_index: ci,
                                                    target: super::DecomposeTarget::Collision,
                                                    method,
                                                    progress,
                                                    sub_progress,
                                                    handle: Some(handle),
                                                    started: std::time::Instant::now(),
                                                });
                                            }
                                        });
                                    }
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
                    // Process deferred decomposition
                    if let Some((idx, new_cols)) = col_decompose {
                        let n = new_cols.len();
                        link.collisions.remove(idx);
                        for (i, c) in new_cols.into_iter().enumerate() {
                            link.collisions.insert(idx + i, c);
                        }
                        col_changed = true;
                        props_edit_desc = Some(format!("Decompose collision of '{}' into {} shapes", link_name, n));
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
                    if ui.button("➕ Add Capsule").clicked() {
                        link.collisions.push(crate::robot::CollisionData {
                            origin: na::Isometry3::identity(),
                            geometry: GeomData::Capsule { radius: 0.02, half_length: 0.1 },
                        });
                        self.needs_upload = true;
                        props_edit_desc = Some(format!("Add collision to '{}'", link_name));
                        ui.close();
                    }
                    if ui.button("📦 Add Mesh (STL/DAE)…").clicked() {
                        self.add_mesh_target = Some(super::AddMeshTarget {
                            link_index: li,
                            kind: super::MeshAddKind::Collision,
                        });
                        self.dlg_add_mesh.open(
                            "メッシュファイルを開く (Collision)",
                            super::file_dialog::FileDialogMode::Open,
                            None,
                            &["stl", "dae"],
                        );
                        ui.close();
                    }
                });
            }

            if let Some(ji) = self.selected_joint {
                let joint_name = model.joints[ji].name.clone();
                // Sync rename buffer when selection changes
                if self.rename_joint_buf.is_empty() || !model.joint_map.contains_key(&self.rename_joint_buf) {
                    self.rename_joint_buf = joint_name.clone();
                }
                // Editable joint name (before taking &mut joint)
                let mut jrename_result: Option<(String, String)> = None;
                ui.horizontal(|ui| {
                    ui.label("Joint:");
                    let resp = ui.add(
                        egui::TextEdit::singleline(&mut self.rename_joint_buf)
                            .desired_width(160.0)
                            .font(egui::TextStyle::Heading),
                    );
                    if resp.lost_focus() && self.rename_joint_buf != joint_name {
                        jrename_result = Some((joint_name.clone(), self.rename_joint_buf.clone()));
                    }
                });
                if let Some((old_name, new_name)) = jrename_result {
                    if model.rename_joint(&old_name, &new_name) {
                        props_edit_desc = Some(format!("Rename joint '{}' → '{}'", old_name, new_name));
                    } else {
                        self.rename_joint_buf = model.joints[ji].name.clone();
                    }
                }
                let joint = &mut model.joints[ji];
                let joint_name = joint.name.clone();
                ui.separator();
                let mut joint_changed = false;
                egui::Grid::new("joint_props")
                    .striped(true)
                    .num_columns(2)
                    .show(ui, |ui| {
                        ui.label("Type:");
                        {
                            const JOINT_TYPES: &[&str] = &["revolute", "prismatic", "fixed", "continuous"];
                            let mut cur_type = joint.joint_type.clone();
                            egui::ComboBox::from_id_salt(format!("jtype_{ji}"))
                                .selected_text(&cur_type)
                                .show_ui(ui, |ui| {
                                    for &jt in JOINT_TYPES {
                                        ui.selectable_value(&mut cur_type, jt.to_string(), jt);
                                    }
                                });
                            if cur_type != joint.joint_type {
                                joint.joint_type = cur_type;
                                joint_changed = true;
                            }
                        }
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

                // --- Actuator (control mode + gains) ---
                // Skip for fixed joints; they have no actuator.
                if joint.joint_type != "fixed" {
                    ui.separator();
                    egui::CollapsingHeader::new("Actuator")
                        .default_open(true)
                        .show(ui, |ui| {
                            egui::Grid::new(format!("actuator_props_{ji}"))
                                .striped(true)
                                .num_columns(2)
                                .show(ui, |ui| {
                                    ui.label("Mode:");
                                    let mut mode = joint.actuator_mode;
                                    egui::ComboBox::from_id_salt(format!("actmode_{ji}"))
                                        .selected_text(mode.label())
                                        .show_ui(ui, |ui| {
                                            for m in crate::rbd::model::ActuatorMode::ALL {
                                                ui.selectable_value(&mut mode, m, m.label());
                                            }
                                        });
                                    if mode != joint.actuator_mode {
                                        joint.actuator_mode = mode;
                                        joint_changed = true;
                                    }
                                    ui.end_row();

                                    let kp_enabled = matches!(
                                        joint.actuator_mode,
                                        crate::rbd::model::ActuatorMode::Position
                                    );
                                    let kv_enabled = matches!(
                                        joint.actuator_mode,
                                        crate::rbd::model::ActuatorMode::Position
                                            | crate::rbd::model::ActuatorMode::Velocity
                                    );

                                    ui.label("Kp (N·m/rad):");
                                    ui.add_enabled_ui(kp_enabled, |ui| {
                                        joint_changed |= ui
                                            .add(
                                                egui::DragValue::new(&mut joint.actuator_kp)
                                                    .speed(1.0)
                                                    .range(0.0..=f64::MAX)
                                                    .fixed_decimals(1),
                                            )
                                            .changed();
                                    });
                                    ui.end_row();

                                    ui.label("Kv (N·m·s/rad):");
                                    ui.add_enabled_ui(kv_enabled, |ui| {
                                        joint_changed |= ui
                                            .add(
                                                egui::DragValue::new(&mut joint.actuator_kv)
                                                    .speed(0.1)
                                                    .range(0.0..=f64::MAX)
                                                    .fixed_decimals(2),
                                            )
                                            .changed();
                                    });
                                    ui.end_row();
                                });
                            ui.label(
                                egui::RichText::new(
                                    "Gains apply on the next physics tick. \
                                     Position holds the user-set pose; Velocity \
                                     and Torque take their target via the API.",
                                )
                                .small()
                                .weak(),
                            );
                        });
                }

                if joint_changed {
                    self.needs_upload = true;
                    model.rebuild_misarta_model();
                    props_edit_desc = Some(format!("Edit joint '{}'", joint_name));
                }
            }

            if self.selected_link.is_none() && self.selected_joint.is_none() {
                ui.label("Select a link or joint to view properties.");
            }

            // ===== Global Actuator panel — all movable joints in one grid =====
            // Always visible (regardless of selection) so users can tune gains
            // for the running MuJoCo sim without hunting through the tree.
            ui.separator();
            let mut actuators_changed = false;
            egui::CollapsingHeader::new("⚙ Actuators (all joints)")
                .default_open(false)
                .show(ui, |ui| {
                    let n_movable = model
                        .joints
                        .iter()
                        .filter(|j| j.joint_type != "fixed")
                        .count();
                    if n_movable == 0 {
                        ui.label("(no movable joints)");
                        return;
                    }
                    ui.label(
                        egui::RichText::new(
                            "Edits apply on the next physics tick.",
                        )
                        .small()
                        .weak(),
                    );
                    egui::ScrollArea::vertical()
                        .max_height(280.0)
                        .show(ui, |ui| {
                            egui::Grid::new("global_actuators_grid")
                                .striped(true)
                                .num_columns(4)
                                .min_col_width(40.0)
                                .show(ui, |ui| {
                                    ui.strong("Joint");
                                    ui.strong("Mode");
                                    ui.strong("Kp");
                                    ui.strong("Kv");
                                    ui.end_row();
                                    for joint in model
                                        .joints
                                        .iter_mut()
                                        .filter(|j| j.joint_type != "fixed")
                                    {
                                        ui.label(
                                            egui::RichText::new(&joint.name).monospace().small(),
                                        );

                                        let mut mode = joint.actuator_mode;
                                        egui::ComboBox::from_id_salt(format!(
                                            "all_actmode_{}", joint.name
                                        ))
                                        .selected_text(mode.label())
                                        .width(72.0)
                                        .show_ui(ui, |ui| {
                                            for m in
                                                crate::rbd::model::ActuatorMode::ALL
                                            {
                                                ui.selectable_value(
                                                    &mut mode, m, m.label(),
                                                );
                                            }
                                        });
                                        if mode != joint.actuator_mode {
                                            joint.actuator_mode = mode;
                                            actuators_changed = true;
                                        }

                                        let kp_enabled = matches!(
                                            joint.actuator_mode,
                                            crate::rbd::model::ActuatorMode::Position
                                        );
                                        let kv_enabled = matches!(
                                            joint.actuator_mode,
                                            crate::rbd::model::ActuatorMode::Position
                                                | crate::rbd::model::ActuatorMode::Velocity
                                        );
                                        ui.add_enabled_ui(kp_enabled, |ui| {
                                            actuators_changed |= ui
                                                .add(
                                                    egui::DragValue::new(
                                                        &mut joint.actuator_kp,
                                                    )
                                                    .speed(1.0)
                                                    .range(0.0..=f64::MAX)
                                                    .fixed_decimals(1),
                                                )
                                                .changed();
                                        });
                                        ui.add_enabled_ui(kv_enabled, |ui| {
                                            actuators_changed |= ui
                                                .add(
                                                    egui::DragValue::new(
                                                        &mut joint.actuator_kv,
                                                    )
                                                    .speed(0.1)
                                                    .range(0.0..=f64::MAX)
                                                    .fixed_decimals(2),
                                                )
                                                .changed();
                                        });
                                        ui.end_row();
                                    }
                                });
                        });
                });
            if actuators_changed {
                props_edit_desc = Some("Edit actuator gains".into());
            }
        }

        // ===== Named-pose registry — register / replay / delete =====
        // Done in its own scope so it can borrow `self` mutably (for the
        // MuJoCo sim handle) and `self.model` separately.
        if self.model.is_some() {
            ui.separator();
            if let Some(desc) = self.draw_poses_panel(ui) {
                props_edit_desc = Some(desc);
            }
        }

        // Apply deferred link rename to app-level references
        if let Some((old_name, new_name)) = link_renamed {
            self.update_link_name_refs(&old_name, &new_name);
        }

        // Commit any property edit to undo history
        if let Some(desc) = props_edit_desc {
            self.mark_edit(&desc);
        }


    }

    /// Draw the named-pose registry section. Returns an undo description if
    /// the model was edited (pose added / renamed / removed).
    fn draw_poses_panel(&mut self, ui: &mut egui::Ui) -> Option<String> {
        // Take the model out for the duration of this call so we can borrow
        // `self` mutably for the MuJoCo handle without overlapping borrows.
        let mut model = match self.model.take() {
            Some(m) => m,
            None => return None,
        };
        let result = self.draw_poses_panel_with(ui, &mut model);
        self.model = Some(model);
        result
    }

    fn draw_poses_panel_with(
        &mut self,
        ui: &mut egui::Ui,
        model: &mut crate::robot::RobotModel,
    ) -> Option<String> {
        let mut edit_desc: Option<String> = None;

        egui::CollapsingHeader::new("🧍 Poses")
            .default_open(true)
            .show(ui, |ui| {
                // --- Save current pose ---
                ui.horizontal(|ui| {
                    ui.label("Name:");
                    ui.add(
                        egui::TextEdit::singleline(&mut self.pose_save_name)
                            .desired_width(120.0),
                    );
                    if ui
                        .add_enabled(
                            !self.pose_save_name.trim().is_empty(),
                            egui::Button::new("📌 Save"),
                        )
                        .on_hover_text(
                            "Snapshot the model's current joint angles into a \
                             named pose (saved to .misarta.toml).",
                        )
                        .clicked()
                    {
                        let name = self.pose_save_name.trim().to_string();
                        // Replace by name if exists, else append.
                        let snap = crate::rbd::model::NamedPose::snapshot(&name, model);
                        if let Some(existing) =
                            model.poses.iter_mut().find(|p| p.name == name)
                        {
                            existing.angles = snap.angles;
                        } else {
                            model.poses.push(snap);
                        }
                        edit_desc = Some(format!("Save pose '{name}'"));
                    }
                });

                // --- Transition settings ---
                ui.horizontal(|ui| {
                    ui.label("Duration:");
                    ui.add(
                        egui::DragValue::new(&mut self.pose_transition_duration)
                            .speed(0.05)
                            .range(0.05..=30.0)
                            .fixed_decimals(2)
                            .suffix(" s"),
                    );
                });
                ui.horizontal(|ui| {
                    ui.label("Curve:");
                    egui::ComboBox::from_id_salt("pose_curve_kind")
                        .selected_text(self.pose_transition_kind.label())
                        .show_ui(ui, |ui| {
                            for k in misarta::trajectory::InterpolationKind::ALL {
                                ui.selectable_value(
                                    &mut self.pose_transition_kind,
                                    k,
                                    k.label(),
                                );
                            }
                        });
                });

                if model.poses.is_empty() {
                    ui.label(
                        egui::RichText::new(
                            "(no poses yet — set the joint angles, type a name, and press Save)",
                        )
                        .small()
                        .weak(),
                    );
                    return;
                }

                ui.separator();
                #[cfg(feature = "mujoco")]
                let mj_active = self.mujoco_sim.is_some();
                #[cfg(not(feature = "mujoco"))]
                let mj_active = false;

                // --- Pose list ---
                let mut to_remove: Option<usize> = None;
                let mut to_play: Option<usize> = None;
                let mut to_apply_static: Option<usize> = None;
                egui::ScrollArea::vertical()
                    .max_height(220.0)
                    .show(ui, |ui| {
                        for (i, pose) in model.poses.iter().enumerate() {
                            ui.horizontal(|ui| {
                                ui.label(
                                    egui::RichText::new(&pose.name).monospace(),
                                );
                                if ui
                                    .small_button("📐")
                                    .on_hover_text(
                                        "Apply pose directly to the editor \
                                         (no MuJoCo transition).",
                                    )
                                    .clicked()
                                {
                                    to_apply_static = Some(i);
                                }
                                if ui
                                    .add_enabled(
                                        mj_active,
                                        egui::Button::new("▶ Play"),
                                    )
                                    .on_hover_text(if mj_active {
                                        "Smoothly transition the running MuJoCo \
                                         sim to this pose."
                                    } else {
                                        "Start MuJoCo first to enable playback."
                                    })
                                    .clicked()
                                {
                                    to_play = Some(i);
                                }
                                if ui
                                    .small_button("🗑")
                                    .on_hover_text("Delete this pose.")
                                    .clicked()
                                {
                                    to_remove = Some(i);
                                }
                            });
                        }
                    });

                if let Some(i) = to_remove {
                    let name = model.poses[i].name.clone();
                    model.poses.remove(i);
                    edit_desc = Some(format!("Delete pose '{name}'"));
                }
                if let Some(i) = to_apply_static {
                    let pose = &model.poses[i];
                    let cur = model.joint_positions.clone();
                    let q = pose.to_vector(model, &cur);
                    model.joint_positions = q;
                    self.needs_upload = true;
                    edit_desc = Some(format!("Apply pose '{}'", pose.name));
                }
                #[cfg(feature = "mujoco")]
                if let Some(i) = to_play {
                    let pose = &model.poses[i];
                    let cur = model.joint_positions.clone();
                    let q = pose.to_vector(model, &cur);
                    if let Some(ref mut sim) = self.mujoco_sim {
                        sim.start_transition(
                            q,
                            self.pose_transition_duration as f64,
                            self.pose_transition_kind,
                        );
                        // Auto-resume playback so the user sees motion.
                        self.dynamics_sim_paused = false;
                        self.status_message = format!(
                            "Playing pose '{}' over {:.2}s ({})",
                            pose.name,
                            self.pose_transition_duration,
                            self.pose_transition_kind.label(),
                        );
                    }
                }

                // --- Live transition status (visible during playback) ---
                #[cfg(feature = "mujoco")]
                if let Some(ref sim) = self.mujoco_sim {
                    if let Some(p) = sim.transition_progress() {
                        ui.add(
                            egui::ProgressBar::new(p)
                                .text(format!("Transitioning {:.0}%", p * 100.0)),
                        );
                    }
                }
            });

        edit_desc
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
                // Save .misarta.toml sidecar alongside the model
                if let Err(e) = model.save_sidecar_config(&path) {
                    self.export_message += &format!(" (⚠ config: {e})");
                }
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

        // Save .misarta.toml sidecar alongside the exported model
        let model_file = dir.join(&base_name);
        if let Err(e) = model.save_sidecar_config(&model_file) {
            self.export_message += &format!(" (⚠ config: {e})");
        }
    }

    /// Update all app-level references when a link is renamed.
    fn update_link_name_refs(&mut self, old: &str, new: &str) {
        // Pinned links
        for pin in &mut self.pinned_links {
            if pin.link_name == old {
                pin.link_name = new.to_string();
            }
        }
        // Display mode overrides
        let keys_to_update: Vec<_> = self
            .link_display_modes
            .keys()
            .filter(|(name, _)| name == old)
            .cloned()
            .collect();
        for key in keys_to_update {
            if let Some(mode) = self.link_display_modes.remove(&key) {
                self.link_display_modes.insert((new.to_string(), key.1), mode);
            }
        }
        // IK root link
        if self.ik_root_link.as_deref() == Some(old) {
            self.ik_root_link = Some(new.to_string());
        }
        // Dynamics EE link
        if self.dynamics_ee_link.as_deref() == Some(old) {
            self.dynamics_ee_link = Some(new.to_string());
        }
    }
}
