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
                // Wrap the link-properties block in a CollapsingHeader so it
                // shares the visual chrome (▼/▶ icon, indent-line, body
                // padding) of the Actuators / Peaks / Poses sections below
                // and the user can clearly see where the section starts and
                // ends. State is persisted by egui via `id_salt`.
                let li_name_for_hdr = model.links[li].name.clone();
                egui::CollapsingHeader::new(
                    egui::RichText::new(format!("🔗 Link: {li_name_for_hdr}"))
                        .heading(),
                )
                .id_salt("link_props_collapsing")
                .default_open(true)
                .show(ui, |ui| {
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
                            
                                physics: None,
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
                                                                
                                                                    physics: None,
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
                                                                
                                                                    physics: None,
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
                                                                
                                                                    physics: None,
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
                                                            
                                                                physics: None,
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
                        
                            physics: None,
                        });
                        self.needs_upload = true;
                        props_edit_desc = Some(format!("Add collision to '{}'", link_name));
                        ui.close();
                    }
                    if ui.button("➕ Add Cylinder").clicked() {
                        link.collisions.push(crate::robot::CollisionData {
                            origin: na::Isometry3::identity(),
                            geometry: GeomData::Cylinder { radius: 0.02, half_length: 0.1 },
                        
                            physics: None,
                        });
                        self.needs_upload = true;
                        props_edit_desc = Some(format!("Add collision to '{}'", link_name));
                        ui.close();
                    }
                    if ui.button("➕ Add Sphere").clicked() {
                        link.collisions.push(crate::robot::CollisionData {
                            origin: na::Isometry3::identity(),
                            geometry: GeomData::Sphere { radius: 0.05 },
                        
                            physics: None,
                        });
                        self.needs_upload = true;
                        props_edit_desc = Some(format!("Add collision to '{}'", link_name));
                        ui.close();
                    }
                    if ui.button("➕ Add Capsule").clicked() {
                        link.collisions.push(crate::robot::CollisionData {
                            origin: na::Isometry3::identity(),
                            geometry: GeomData::Capsule { radius: 0.02, half_length: 0.1 },
                        
                            physics: None,
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
                }); // close link-properties CollapsingHeader::show closure
            }

            if let Some(ji) = self.selected_joint {
                let joint_name = model.joints[ji].name.clone();
                // Same CollapsingHeader treatment as the link section so the
                // two top-level property blocks present consistently.
                let ji_name_for_hdr = joint_name.clone();
                egui::CollapsingHeader::new(
                    egui::RichText::new(format!("⚙ Joint: {ji_name_for_hdr}"))
                        .heading(),
                )
                .id_salt("joint_props_collapsing")
                .default_open(true)
                .show(ui, |ui| {
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
                                            | crate::rbd::model::ActuatorMode::ComputedTorque
                                    );
                                    let kv_enabled = matches!(
                                        joint.actuator_mode,
                                        crate::rbd::model::ActuatorMode::Position
                                            | crate::rbd::model::ActuatorMode::Velocity
                                            | crate::rbd::model::ActuatorMode::ComputedTorque
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

                                    ui.label("Armature (kg·m²):")
                                        .on_hover_text(
                                            "Reflected rotor inertia. \
                                             Mapped to MuJoCo `<joint armature>`. \
                                             Acts as a low-pass filter for the PD \
                                             controller and absorbs landing impacts. \
                                             Real motors with gearboxes typically \
                                             have 1e-4 .. 1e-2 kg·m².",
                                        );
                                    joint_changed |= ui
                                        .add(
                                            egui::DragValue::new(&mut joint.armature)
                                                .speed(0.0001)
                                                .range(0.0..=f64::MAX)
                                                .fixed_decimals(5),
                                        )
                                        .changed();
                                    ui.end_row();

                                    ui.label("Damping (N·m·s/rad):")
                                        .on_hover_text(
                                            "Passive joint damping. Mapped to \
                                             MuJoCo `<joint damping>`. Models bearing \
                                             friction and dissipates impact energy.",
                                        );
                                    joint_changed |= ui
                                        .add(
                                            egui::DragValue::new(&mut joint.joint_damping)
                                                .speed(0.01)
                                                .range(0.0..=f64::MAX)
                                                .fixed_decimals(3),
                                        )
                                        .changed();
                                    ui.end_row();
                                });
                            ui.label(
                                egui::RichText::new(
                                    "Kp / Kv apply on the next physics tick. \
                                     Armature and damping are baked into the \
                                     MJCF at sim start, so changing them \
                                     requires Stop → Play. Position holds the \
                                     user-set pose; Velocity / Torque modes \
                                     take their target via the API.",
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
                }); // close joint-properties CollapsingHeader::show closure
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
                                .num_columns(6)
                                .min_col_width(40.0)
                                .show(ui, |ui| {
                                    ui.strong("Joint");
                                    ui.strong("Mode");
                                    ui.strong("Kp");
                                    ui.strong("Kv");
                                    ui.strong("Arm")
                                        .on_hover_text(
                                            "Armature (rotor inertia, kg·m²)",
                                        );
                                    ui.strong("Damp")
                                        .on_hover_text(
                                            "Joint damping (N·m·s/rad)",
                                        );
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
                                        actuators_changed |= ui
                                            .add(
                                                egui::DragValue::new(
                                                    &mut joint.armature,
                                                )
                                                .speed(0.0001)
                                                .range(0.0..=f64::MAX)
                                                .fixed_decimals(5),
                                            )
                                            .changed();
                                        actuators_changed |= ui
                                            .add(
                                                egui::DragValue::new(
                                                    &mut joint.joint_damping,
                                                )
                                                .speed(0.01)
                                                .range(0.0..=f64::MAX)
                                                .fixed_decimals(3),
                                            )
                                            .changed();
                                        ui.end_row();
                                    }
                                });
                        });
                });
            if actuators_changed {
                props_edit_desc = Some("Edit actuator gains".into());
            }
        }

        // ===== Joint Peaks — running max |τ|, |q̇| since last reset =====
        // Sits directly under the Actuators panel because the two are read
        // together: the user dials gains in Actuators and watches the peak
        // τ / q̇ here to size the real motors / detect saturation. Peaks reset
        // automatically when ▶ Play is hit on a pose or an external force
        // pulse is applied so each command produces its own measurement window.
        #[cfg(feature = "mujoco")]
        if self.model.is_some() {
            self.draw_joint_peaks_panel(ui);
        }

        // ===== Contacts list — every active MuJoCo contact this tick =====
        // Surfaces self-collision pairs (and their force magnitudes) in a
        // table so the user can pinpoint geom interpenetrations that
        // wouldn't be obvious from the viewport arrows alone.
        #[cfg(feature = "mujoco")]
        if self.model.is_some() {
            self.draw_contacts_panel(ui);
        }

        // ===== Named-pose registry — register / replay / delete =====
        // Done in its own scope so it can borrow `self` mutably (for the
        // MuJoCo sim handle) and `self.model` separately.
        if self.model.is_some() {
            ui.separator();
            if let Some(desc) = self.draw_poses_panel(ui) {
                props_edit_desc = Some(desc);
            }
            ui.separator();
            if let Some(desc) = self.draw_sequences_panel(ui) {
                props_edit_desc = Some(desc);
            }
            // External force panel — lives on the right next to Poses since
            // both are sim-time disturbance/replay primitives.
            #[cfg(feature = "mujoco")]
            {
                ui.separator();
                self.draw_external_force_panel(ui);
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
                // The "default duration / curve" rows below seed both the new
                // pose's stored defaults *and* the values used immediately by
                // ▶ Play; per-pose rows further down let the user override
                // them per pose without re-saving.
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
                             named pose (saved with the model — to .misa or, \
                             for legacy URDF workflows, to .misarta.toml).",
                        )
                        .clicked()
                    {
                        let name = self.pose_save_name.trim().to_string();
                        let snap = crate::rbd::model::NamedPose::snapshot(
                            &name,
                            model,
                            self.pose_transition_duration as f64,
                            self.pose_transition_kind,
                        );
                        if let Some(existing) =
                            model.poses.iter_mut().find(|p| p.name == name)
                        {
                            existing.angles = snap.angles;
                            // Refresh the saved defaults too so re-save acts
                            // as "update everything about this pose".
                            existing.duration = snap.duration;
                            existing.kind = snap.kind;
                        } else {
                            model.poses.push(snap);
                        }
                        edit_desc = Some(format!("Save pose '{name}'"));
                    }
                });

                // --- Default transition settings (used by Save) ---
                ui.horizontal(|ui| {
                    ui.label("Default duration:");
                    ui.add(
                        egui::DragValue::new(&mut self.pose_transition_duration)
                            .speed(0.05)
                            .range(0.05..=30.0)
                            .fixed_decimals(2)
                            .suffix(" s"),
                    );
                });
                ui.horizontal(|ui| {
                    ui.label("Default curve:");
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
                // Each row owns its own duration / curve editor; edits flow
                // straight back into `model.poses[i]` so they're persisted to
                // the sidecar TOML on the next save and used by ▶ Play.
                let mut to_remove: Option<usize> = None;
                let mut to_play: Option<usize> = None;
                let mut to_apply_static: Option<usize> = None;
                egui::ScrollArea::vertical()
                    .max_height(260.0)
                    .show(ui, |ui| {
                        for (i, pose) in model.poses.iter_mut().enumerate() {
                            // Row 1: name + action buttons
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
                                         sim to this pose using the row's duration / curve."
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
                            // Row 2: per-pose duration + curve overrides.
                            // Edits modify the saved defaults so the value the
                            // user dialled in is what gets used next time.
                            let mut row_changed = false;
                            ui.horizontal(|ui| {
                                ui.label("    Dur:");
                                if ui
                                    .add(
                                        egui::DragValue::new(&mut pose.duration)
                                            .speed(0.05)
                                            .range(0.05..=30.0)
                                            .fixed_decimals(2)
                                            .suffix(" s"),
                                    )
                                    .changed()
                                {
                                    row_changed = true;
                                }
                                let mut k = pose.kind;
                                egui::ComboBox::from_id_salt(format!("pose_kind_{i}"))
                                    .selected_text(k.label())
                                    .show_ui(ui, |ui| {
                                        for kk in
                                            misarta::trajectory::InterpolationKind::ALL
                                        {
                                            ui.selectable_value(
                                                &mut k, kk, kk.label(),
                                            );
                                        }
                                    });
                                if k != pose.kind {
                                    pose.kind = k;
                                    row_changed = true;
                                }
                            });
                            if row_changed {
                                edit_desc =
                                    Some(format!("Edit pose defaults '{}'", pose.name));
                            }
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
                    let dur = pose.duration;
                    let kind = pose.kind;
                    let pose_name = pose.name.clone();
                    if let Some(ref mut sim) = self.mujoco_sim {
                        sim.start_transition(q, dur, kind);
                        // Auto-resume playback so the user sees motion.
                        self.dynamics_sim_paused = false;
                        self.status_message = format!(
                            "Playing pose '{}' over {:.2}s ({})",
                            pose_name,
                            dur,
                            kind.label(),
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

    /// Draw the named-sequence registry section. A sequence is an ordered
    /// list of pose targets with per-step durations / interpolation kinds;
    /// playing one chains the transitions back-to-back via
    /// [`crate::mujoco_sim::MujocoSim::start_sequence`].
    ///
    /// Returns an undo description if the model was edited.
    fn draw_sequences_panel(&mut self, ui: &mut egui::Ui) -> Option<String> {
        // take/restore pattern, mirroring draw_poses_panel.
        let mut model = self.model.take()?;
        let result = self.draw_sequences_panel_with(ui, &mut model);
        self.model = Some(model);
        result
    }

    fn draw_sequences_panel_with(
        &mut self,
        ui: &mut egui::Ui,
        model: &mut crate::robot::RobotModel,
    ) -> Option<String> {
        let mut edit_desc: Option<String> = None;

        egui::CollapsingHeader::new("🎬 Sequences")
            .default_open(false)
            .show(ui, |ui| {
                // ── Create a new (empty) sequence ──
                ui.horizontal(|ui| {
                    ui.label("Name:");
                    ui.add(
                        egui::TextEdit::singleline(&mut self.sequence_save_name)
                            .desired_width(120.0),
                    );
                    let name = self.sequence_save_name.trim().to_string();
                    if ui
                        .add_enabled(
                            !name.is_empty()
                                && !model.sequences.iter().any(|s| s.name == name),
                            egui::Button::new("📌 New"),
                        )
                        .on_hover_text(
                            "Create an empty sequence; add pose steps below.",
                        )
                        .clicked()
                    {
                        model.sequences.push(crate::rbd::model::Sequence {
                            name: name.clone(),
                            steps: Vec::new(),
                        });
                        edit_desc = Some(format!("Create sequence '{name}'"));
                        self.sequence_save_name.clear();
                    }
                });

                if model.sequences.is_empty() {
                    ui.label(
                        egui::RichText::new(
                            "(no sequences yet — create one above and add pose steps)",
                        )
                        .small()
                        .weak(),
                    );
                    return;
                }

                #[cfg(feature = "mujoco")]
                let mj_active = self.mujoco_sim.is_some();
                #[cfg(not(feature = "mujoco"))]
                let mj_active = false;

                // Pose dropdown candidates — needed by every step's edit row.
                let pose_names: Vec<String> =
                    model.poses.iter().map(|p| p.name.clone()).collect();

                let mut to_remove_seq: Option<usize> = None;
                let mut to_play_seq: Option<usize> = None;
                let mut to_remove_step: Option<(usize, usize)> = None;
                let mut to_add_step: Option<(usize, String)> = None;

                for (si, seq) in model.sequences.iter_mut().enumerate() {
                    egui::CollapsingHeader::new(format!("▶ {}  ({} steps)", seq.name, seq.steps.len()))
                        .id_salt(format!("seq_{si}"))
                        .default_open(true)
                        .show(ui, |ui| {
                            ui.horizontal(|ui| {
                                if ui
                                    .add_enabled(
                                        mj_active && !seq.steps.is_empty(),
                                        egui::Button::new("▶ Play"),
                                    )
                                    .on_hover_text(if mj_active {
                                        "Replay the chained transitions in this sequence."
                                    } else {
                                        "Start MuJoCo first to enable playback."
                                    })
                                    .clicked()
                                {
                                    to_play_seq = Some(si);
                                }
                                if ui
                                    .small_button("🗑")
                                    .on_hover_text("Delete this sequence.")
                                    .clicked()
                                {
                                    to_remove_seq = Some(si);
                                }
                            });

                            // Step list
                            for (stepi, step) in seq.steps.iter_mut().enumerate() {
                                let mut row_changed = false;
                                ui.horizontal(|ui| {
                                    ui.label(
                                        egui::RichText::new(format!("{stepi}."))
                                            .small().weak(),
                                    );
                                    // Pose name dropdown
                                    egui::ComboBox::from_id_salt(format!(
                                        "seqstep_pose_{si}_{stepi}"
                                    ))
                                    .selected_text(&step.pose_name)
                                    .width(120.0)
                                    .show_ui(ui, |ui| {
                                        for n in &pose_names {
                                            if ui
                                                .selectable_label(
                                                    &step.pose_name == n,
                                                    n,
                                                )
                                                .clicked()
                                            {
                                                step.pose_name = n.clone();
                                                row_changed = true;
                                            }
                                        }
                                    });
                                    // Duration
                                    if ui
                                        .add(
                                            egui::DragValue::new(&mut step.duration)
                                                .speed(0.05)
                                                .range(0.05..=30.0)
                                                .fixed_decimals(2)
                                                .suffix(" s"),
                                        )
                                        .changed()
                                    {
                                        row_changed = true;
                                    }
                                    // Curve
                                    let mut k = step.kind;
                                    egui::ComboBox::from_id_salt(format!(
                                        "seqstep_kind_{si}_{stepi}"
                                    ))
                                    .selected_text(k.label())
                                    .show_ui(ui, |ui| {
                                        for kk in
                                            misarta::trajectory::InterpolationKind::ALL
                                        {
                                            ui.selectable_value(
                                                &mut k, kk, kk.label(),
                                            );
                                        }
                                    });
                                    if k != step.kind {
                                        step.kind = k;
                                        row_changed = true;
                                    }
                                    // Delete step
                                    if ui
                                        .small_button("🗑")
                                        .on_hover_text("Remove this step.")
                                        .clicked()
                                    {
                                        to_remove_step = Some((si, stepi));
                                    }
                                });
                                if row_changed {
                                    edit_desc = Some(format!(
                                        "Edit sequence '{}' step {}",
                                        seq.name, stepi
                                    ));
                                }
                            }

                            // Add-step row.
                            ui.horizontal(|ui| {
                                ui.label("    + Add:");
                                let buf = self
                                    .sequence_step_pose_buf
                                    .entry(si)
                                    .or_default();
                                let label = if buf.is_empty() {
                                    "(pick pose)".to_string()
                                } else {
                                    buf.clone()
                                };
                                egui::ComboBox::from_id_salt(format!("seqadd_pose_{si}"))
                                    .selected_text(label)
                                    .width(140.0)
                                    .show_ui(ui, |ui| {
                                        for n in &pose_names {
                                            if ui.selectable_label(buf == n, n).clicked() {
                                                *buf = n.clone();
                                            }
                                        }
                                    });
                                if ui
                                    .add_enabled(
                                        !buf.is_empty(),
                                        egui::Button::new("➕"),
                                    )
                                    .on_hover_text("Append this pose as a new step.")
                                    .clicked()
                                {
                                    to_add_step = Some((si, buf.clone()));
                                    *buf = String::new();
                                }
                            });
                        });
                }

                if let Some(i) = to_remove_seq {
                    let name = model.sequences[i].name.clone();
                    model.sequences.remove(i);
                    edit_desc = Some(format!("Delete sequence '{name}'"));
                }
                if let Some((si, stepi)) = to_remove_step {
                    if let Some(seq) = model.sequences.get_mut(si) {
                        if stepi < seq.steps.len() {
                            seq.steps.remove(stepi);
                            edit_desc = Some(format!(
                                "Remove step {} from sequence '{}'",
                                stepi, seq.name
                            ));
                        }
                    }
                }
                if let Some((si, pose_name)) = to_add_step {
                    if let Some(seq) = model.sequences.get_mut(si) {
                        seq.steps.push(crate::rbd::model::SequenceStep {
                            pose_name: pose_name.clone(),
                            duration: 1.0,
                            kind: misarta::trajectory::InterpolationKind::QuinticSmooth,
                        });
                        edit_desc = Some(format!(
                            "Add step '{}' to sequence '{}'",
                            pose_name, seq.name
                        ));
                    }
                }

                #[cfg(feature = "mujoco")]
                if let Some(si) = to_play_seq {
                    let seq_name = model.sequences[si].name.clone();
                    if let Some(anim) = model.build_sequence_animation(&seq_name) {
                        if let Some(ref mut sim) = self.mujoco_sim {
                            sim.start_sequence(anim, seq_name.clone());
                            self.dynamics_sim_paused = false;
                            self.status_message =
                                format!("Playing sequence '{}'", seq_name);
                        }
                    } else {
                        self.status_message = format!(
                            "Sequence '{}' references missing pose(s)",
                            seq_name
                        );
                    }
                }

                // Live progress (only one sequence at a time).
                #[cfg(feature = "mujoco")]
                if let Some(ref sim) = self.mujoco_sim {
                    if let Some(p) = sim.sequence_progress() {
                        let name = sim.current_sequence_name().unwrap_or("").to_string();
                        ui.add(egui::ProgressBar::new(p).text(format!(
                            "▶ {}  {:.0}%",
                            name,
                            p * 100.0
                        )));
                    }
                }
            });

        edit_desc
    }

    /// Draw the "apply external force/torque to a link for N seconds" panel.
    ///
    /// The pulse goes through [`crate::mujoco_sim::MujocoSim::apply_external_force`],
    /// which writes `xfrc_applied` each tick until the timer expires. Force /
    /// torque are interpreted in the world frame.
    #[cfg(feature = "mujoco")]
    fn draw_external_force_panel(&mut self, ui: &mut egui::Ui) {
        // Build the link list outside of any closure that borrows self.
        let link_names: Vec<String> = match self.model.as_ref() {
            Some(m) => m.links.iter().map(|l| l.name.clone()).collect(),
            None => return,
        };
        let mj_active = self.mujoco_sim.is_some();
        // Snapshot the active pulses for the status display so we don't
        // borrow `self.mujoco_sim` while UI closures hold `&mut self`.
        let active_pulses: Vec<(String, f64, f64)> = self
            .mujoco_sim
            .as_ref()
            .map(|s| {
                s.external_force_pulses()
                    .iter()
                    .map(|p| (p.link_name.clone(), p.elapsed, p.duration))
                    .collect()
            })
            .unwrap_or_default();

        egui::CollapsingHeader::new("💥 External Force")
            .default_open(false)
            .show(ui, |ui| {
                ui.horizontal(|ui| {
                    ui.label("Link:");
                    let label = self
                        .ext_force_link
                        .as_deref()
                        .unwrap_or("(select)")
                        .to_string();
                    egui::ComboBox::from_id_salt("ext_force_link")
                        .selected_text(label)
                        .show_ui(ui, |ui| {
                            for name in &link_names {
                                let sel =
                                    self.ext_force_link.as_deref() == Some(name.as_str());
                                if ui.selectable_label(sel, name).clicked() {
                                    self.ext_force_link = Some(name.clone());
                                }
                            }
                        });
                });
                ui.horizontal(|ui| {
                    ui.label("Force (N):");
                    ui.add(
                        egui::DragValue::new(&mut self.ext_force_value[0])
                            .speed(0.1).fixed_decimals(2).prefix("x:"),
                    );
                    ui.add(
                        egui::DragValue::new(&mut self.ext_force_value[1])
                            .speed(0.1).fixed_decimals(2).prefix("y:"),
                    );
                    ui.add(
                        egui::DragValue::new(&mut self.ext_force_value[2])
                            .speed(0.1).fixed_decimals(2).prefix("z:"),
                    );
                });
                ui.horizontal(|ui| {
                    ui.label("Torque (N·m):");
                    ui.add(
                        egui::DragValue::new(&mut self.ext_torque_value[0])
                            .speed(0.05).fixed_decimals(2).prefix("x:"),
                    );
                    ui.add(
                        egui::DragValue::new(&mut self.ext_torque_value[1])
                            .speed(0.05).fixed_decimals(2).prefix("y:"),
                    );
                    ui.add(
                        egui::DragValue::new(&mut self.ext_torque_value[2])
                            .speed(0.05).fixed_decimals(2).prefix("z:"),
                    );
                });
                ui.horizontal(|ui| {
                    ui.label("Duration:");
                    ui.add(
                        egui::DragValue::new(&mut self.ext_force_duration)
                            .speed(0.05)
                            .range(0.01..=30.0)
                            .fixed_decimals(2)
                            .suffix(" s"),
                    );
                });
                ui.horizontal(|ui| {
                    let can_apply = mj_active
                        && self.ext_force_link.is_some()
                        && (self.ext_force_value.iter().any(|&v| v != 0.0)
                            || self.ext_torque_value.iter().any(|&v| v != 0.0));
                    if ui
                        .add_enabled(
                            can_apply,
                            egui::Button::new("⚡ Apply pulse"),
                        )
                        .on_hover_text(if mj_active {
                            "Apply the wrench (world frame) to the selected link \
                             for the specified duration."
                        } else {
                            "Start MuJoCo first to apply forces."
                        })
                        .clicked()
                    {
                        if let (Some(link), Some(ref mut sim)) =
                            (self.ext_force_link.clone(), self.mujoco_sim.as_mut())
                        {
                            let f = [
                                self.ext_force_value[0] as f64,
                                self.ext_force_value[1] as f64,
                                self.ext_force_value[2] as f64,
                            ];
                            let t = [
                                self.ext_torque_value[0] as f64,
                                self.ext_torque_value[1] as f64,
                                self.ext_torque_value[2] as f64,
                            ];
                            sim.apply_external_force(
                                &link, f, t, self.ext_force_duration as f64,
                            );
                            self.dynamics_sim_paused = false;
                            self.status_message = format!(
                                "Applying [{:.1},{:.1},{:.1}]N to '{}' for {:.2}s",
                                f[0], f[1], f[2], link, self.ext_force_duration,
                            );
                        }
                    }
                    if ui
                        .add_enabled(
                            mj_active && self.ext_force_link.is_some(),
                            egui::Button::new("⏹ Cancel"),
                        )
                        .on_hover_text("Stop any pulse currently on this link.")
                        .clicked()
                    {
                        if let (Some(link), Some(ref mut sim)) =
                            (self.ext_force_link.clone(), self.mujoco_sim.as_mut())
                        {
                            sim.cancel_external_force(&link);
                        }
                    }
                });
                if !active_pulses.is_empty() {
                    ui.separator();
                    ui.label(
                        egui::RichText::new("Active pulses").small().strong(),
                    );
                    for (name, elapsed, duration) in &active_pulses {
                        let frac =
                            ((duration - elapsed).max(0.0) / duration.max(1e-9))
                                as f32;
                        ui.add(
                            egui::ProgressBar::new(frac).text(format!(
                                "{} — {:.2}s left",
                                name,
                                (duration - elapsed).max(0.0),
                            )),
                        );
                    }
                }
            });
    }

    /// Draw the running max-|τ| / max-|q̇| table for the active MuJoCo sim.
    ///
    /// Displayed under the Actuators panel since it's the natural feedback
    /// loop for tuning gains and sizing motors. Each row shows one movable
    /// joint with the unit suffix selected by joint type (N·m / N for τ,
    /// rad/s / m/s for q̇). The "↺ Reset" button manually clears the window;
    /// the sim itself also resets the window on each ▶ Play / pulse so peak
    /// readings only reflect the most recent command.
    #[cfg(feature = "mujoco")]
    fn draw_joint_peaks_panel(&mut self, ui: &mut egui::Ui) {
        ui.separator();
        let mut reset_clicked = false;
        // Snapshot peaks + joint metadata up-front so we don't borrow
        // self.mujoco_sim while building the UI.
        let peaks_snapshot: Vec<(String, String, f64, f64, f64, f64)> = match (
            self.model.as_ref(),
            self.mujoco_sim.as_ref(),
        ) {
            (Some(model), Some(sim)) => {
                let peaks = sim.peaks();
                model
                    .joints
                    .iter()
                    .enumerate()
                    .filter(|(_, j)| j.joint_type != "fixed")
                    .map(|(i, j)| {
                        let p = peaks.get(i).cloned().unwrap_or_default();
                        (
                            j.name.clone(),
                            j.joint_type.clone(),
                            p.tau_abs,
                            p.tau_signed,
                            p.qvel_abs,
                            p.qvel_signed,
                        )
                    })
                    .collect()
            }
            _ => Vec::new(),
        };

        egui::CollapsingHeader::new("📊 Joint Peaks")
            .default_open(false)
            .show(ui, |ui| {
                if self.mujoco_sim.is_none() {
                    ui.label(
                        egui::RichText::new(
                            "(start MuJoCo to record τ / q̇ peaks)",
                        )
                        .small()
                        .weak(),
                    );
                    return;
                }
                if peaks_snapshot.is_empty() {
                    ui.label("(no movable joints)");
                    return;
                }
                ui.label(
                    egui::RichText::new(
                        "Auto-resets on each ▶ Play / pulse. Suffix is N·m / rad/s for revolute, N / m/s for prismatic.",
                    )
                    .small()
                    .weak(),
                );
                let mut plot_clicked = false;
                ui.horizontal(|ui| {
                    if ui
                        .small_button("↺ Reset peaks")
                        .on_hover_text("Clear the running max values.")
                        .clicked()
                    {
                        reset_clicked = true;
                    }
                    if ui
                        .small_button("📈 Plot")
                        .on_hover_text(
                            "Open the time-series plot window for q / q̇ / τ.",
                        )
                        .clicked()
                    {
                        plot_clicked = true;
                    }
                });
                if plot_clicked {
                    self.show_peaks_plot = true;
                }
                egui::ScrollArea::vertical()
                    .max_height(280.0)
                    .show(ui, |ui| {
                        egui::Grid::new("joint_peaks_grid")
                            .striped(true)
                            .num_columns(3)
                            .min_col_width(60.0)
                            .show(ui, |ui| {
                                ui.strong("Joint");
                                ui.strong("|τ|max");
                                ui.strong("|q̇|max");
                                ui.end_row();
                                for (name, jt, tau_abs, tau_s, qd_abs, qd_s) in
                                    &peaks_snapshot
                                {
                                    let is_prismatic = jt == "prismatic";
                                    let tau_unit =
                                        if is_prismatic { "N" } else { "N·m" };
                                    let qd_unit =
                                        if is_prismatic { "m/s" } else { "rad/s" };
                                    ui.label(
                                        egui::RichText::new(name).monospace().small(),
                                    );
                                    ui.label(
                                        egui::RichText::new(format!(
                                            "{:>7.3} {} ({:+.3})",
                                            tau_abs, tau_unit, tau_s,
                                        ))
                                        .monospace()
                                        .small(),
                                    );
                                    ui.label(
                                        egui::RichText::new(format!(
                                            "{:>7.3} {} ({:+.3})",
                                            qd_abs, qd_unit, qd_s,
                                        ))
                                        .monospace()
                                        .small(),
                                    );
                                    ui.end_row();
                                }
                            });
                    });
            });

        if reset_clicked {
            if let Some(ref mut sim) = self.mujoco_sim {
                sim.reset_peaks();
            }
        }
    }

    /// Per-tick contact table for the active MuJoCo sim.
    ///
    /// Lists every contact MuJoCo reported, sorted by force magnitude (so
    /// the most concerning interpenetrations float to the top). Self-
    /// collisions (both bodies are robot links) are tagged with `🟣` and
    /// shown alongside ground contacts (`🟡`) in a single grid. A
    /// per-row "🛡 Exclude" button drops that pair into the user's
    /// `collision_pairs` so subsequent MJCF / USD exports emit the
    /// appropriate `<exclude>` / `filteredPairs` entry.
    #[cfg(feature = "mujoco")]
    fn draw_contacts_panel(&mut self, ui: &mut egui::Ui) {
        ui.separator();
        // Snapshot rows up-front so we don't keep `self.mujoco_sim` borrowed
        // while egui's grid runs.
        struct Row {
            mag: f64,
            body1: String,
            body2: String,
            is_self: bool,
        }
        let rows: Vec<Row> = match self.mujoco_sim.as_ref() {
            Some(sim) => {
                let mut v: Vec<Row> = sim
                    .contacts()
                    .into_iter()
                    .map(|c| Row {
                        mag: c.force_mag,
                        is_self: c.is_self_collision(),
                        body1: c.body1,
                        body2: c.body2,
                    })
                    .collect();
                v.sort_by(|a, b| {
                    b.mag.partial_cmp(&a.mag).unwrap_or(std::cmp::Ordering::Equal)
                });
                v
            }
            None => Vec::new(),
        };
        let n_self = rows.iter().filter(|r| r.is_self).count();
        let n_external = rows.len() - n_self;

        // Pending pair to add to collision_pairs after the closure releases self.
        let mut pending_exclude: Option<(String, String)> = None;

        egui::CollapsingHeader::new(format!(
            "💥 Contacts  ({} self, {} ground)",
            n_self, n_external,
        ))
        .default_open(false)
        .show(ui, |ui| {
            if self.mujoco_sim.is_none() {
                ui.label(
                    egui::RichText::new("(start MuJoCo to see contacts)")
                        .small()
                        .weak(),
                );
                return;
            }
            if rows.is_empty() {
                ui.label(
                    egui::RichText::new("(no contacts this tick)").small().weak(),
                );
                return;
            }
            ui.label(
                egui::RichText::new(
                    "🟣 self-collision · 🟡 ground/world contact. Sorted by |F|. \
                     Click 🛡 Exclude to add a `[[collision_pair]]` entry that \
                     persists with the model (.misa or .misarta.toml sidecar) \
                     and shows up in MJCF/USD export.",
                )
                .small()
                .weak(),
            );
            egui::ScrollArea::vertical()
                .max_height(220.0)
                .show(ui, |ui| {
                    egui::Grid::new("contacts_grid")
                        .striped(true)
                        .num_columns(4)
                        .min_col_width(40.0)
                        .show(ui, |ui| {
                            ui.strong("");
                            ui.strong("|F|");
                            ui.strong("Pair");
                            ui.strong("");
                            ui.end_row();
                            for r in &rows {
                                let icon = if r.is_self { "🟣" } else { "🟡" };
                                let pair = match (r.body1.as_str(), r.body2.as_str()) {
                                    ("", "") => "(world↔world)".to_string(),
                                    ("", b) => format!("world ↔ {b}"),
                                    (a, "") => format!("{a} ↔ world"),
                                    (a, b) => format!("{a} ↔ {b}"),
                                };
                                ui.label(icon);
                                ui.label(
                                    egui::RichText::new(format!("{:.2} N", r.mag))
                                        .monospace()
                                        .small(),
                                );
                                ui.label(
                                    egui::RichText::new(pair).monospace().small(),
                                );
                                if r.is_self
                                    && ui
                                        .small_button("🛡 Exclude")
                                        .on_hover_text(
                                            "Add this self-collision pair to \
                                             collision_pairs (excluded). \
                                             Persists with the model (.misa \
                                             or legacy .misarta.toml sidecar).",
                                        )
                                        .clicked()
                                {
                                    pending_exclude = Some((
                                        r.body1.clone(),
                                        r.body2.clone(),
                                    ));
                                }
                                ui.end_row();
                            }
                        });
                });
        });

        if let Some((a, b)) = pending_exclude {
            if let Some(model) = self.model.as_mut() {
                // Remove any existing entry for this pair (so we toggle
                // explicit-enabled → excluded cleanly), then push disabled.
                model.collision_pairs.retain(|p| !p.matches(&a, &b));
                model
                    .collision_pairs
                    .push(crate::rbd::model::CollisionPair::new(a, b, false));
                self.status_message = "Excluded self-collision pair (saved to model)".into();
            }
        }
    }

    /// Draw the Export dialog window (format + directory + export button).
    /// Compatibility-warning dialog shown before Save / Export when the
    /// selected format can't natively express something currently in
    /// the model. The user picks Continue (proceed with the loss /
    /// approximation, sidecar still preserves everything) or Cancel
    /// (abort the export). When `pending_export_action` is `None` the
    /// method early-returns, so it's cheap to call every frame.
    pub(super) fn draw_export_compat_dialog(&mut self, ctx: &egui::Context) {
        let Some(action) = self.pending_export_action else {
            return;
        };
        let mut open = true;
        let mut decision: Option<bool> = None; // Some(true)=continue, Some(false)=cancel
        egui::Window::new(match action {
            super::PendingExportAction::Save => "Save — compatibility warning",
            super::PendingExportAction::Export => "Export — compatibility warning",
        })
        .open(&mut open)
        .resizable(false)
        .collapsible(false)
        .show(ctx, |ui| {
            ui.label(
                egui::RichText::new(
                    "The target format can't natively express some items \
                     in your model. The .misa master file (or the legacy \
                     .misarta.toml sidecar) still preserves them, but the \
                     exported file alone will be incomplete. Save as Misa \
                     to keep everything in a single file.",
                )
                .small()
                .weak(),
            );
            ui.separator();
            for issue in &self.export_compat_issues {
                ui.horizontal(|ui| {
                    let color = match issue.severity {
                        crate::format::ExportSeverity::Drop => {
                            egui::Color32::from_rgb(230, 120, 80)
                        }
                        crate::format::ExportSeverity::Approximate => {
                            egui::Color32::from_rgb(220, 200, 80)
                        }
                    };
                    ui.colored_label(
                        color,
                        format!(
                            "[{}] {} ({})",
                            issue.severity.label(),
                            issue.feature,
                            issue.count,
                        ),
                    );
                });
                ui.label(
                    egui::RichText::new(&issue.message)
                        .small()
                        .weak(),
                );
                ui.add_space(4.0);
            }
            ui.separator();
            ui.horizontal(|ui| {
                if ui.button("Continue (preserve in sidecar)").clicked() {
                    decision = Some(true);
                }
                if ui.button("Cancel").clicked() {
                    decision = Some(false);
                }
            });
        });
        // Window-close (X) acts like Cancel.
        if !open {
            decision = Some(false);
        }
        if let Some(go) = decision {
            self.pending_export_action = None;
            self.export_compat_issues.clear();
            if go {
                match action {
                    super::PendingExportAction::Save => self.save_now(),
                    super::PendingExportAction::Export => self.export_now(),
                }
            }
        }
    }

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

    /// Public entry point for the menu's Save button. Runs the
    /// pre-export compatibility check and either proceeds (no issues)
    /// or queues a confirmation dialog with the list of features that
    /// would be lost / approximated.
    pub(super) fn do_save(&mut self) {
        self.sync_gait_panel_to_model();
        let Some(model) = self.model.as_ref() else {
            self.export_message = "⚠ No model loaded.".into();
            return;
        };
        if model.source_path.is_none() {
            self.export_message = "⚠ New model has no source file. Use Export instead.".into();
            return;
        }
        // Save always targets URDF (the model's source). Run the
        // compatibility analysis against the URDF handler.
        let registry = crate::format::FormatRegistry::default_registry();
        let urdf_handler = registry
            .handlers()
            .iter()
            .find(|h| h.name() == "URDF")
            .map(|h| h.as_ref());
        let issues = if let Some(h) = urdf_handler {
            crate::format::analyze_export_compatibility(model, h)
        } else {
            Vec::new()
        };
        if issues.is_empty() {
            self.save_now();
        } else {
            self.export_compat_issues = issues;
            self.pending_export_action = Some(super::PendingExportAction::Save);
        }
    }

    /// Bypass the confirmation dialog and write the file immediately.
    /// Called from the dialog's Continue button and from the no-issues
    /// fast path in [`Self::do_save`].
    pub(super) fn save_now(&mut self) {
        let Some(ref model) = self.model else {
            self.export_message = "⚠ No model loaded.".into();
            return;
        };
        let Some(source) = model.source_path.clone() else {
            self.export_message = "⚠ New model has no source file. Use Export instead.".into();
            return;
        };

        // Dispatch on source format. `.misa` is the master format and
        // round-trips losslessly via save_as_misa; URDF (and other legacy
        // sources) keep the URDF + `.misarta.toml` sidecar pair so users
        // who haven't migrated yet don't silently lose state.
        let fmt = crate::format::RobotFormat::detect(&source);
        match fmt {
            Some(crate::format::RobotFormat::Misa) => match model.save_as_misa(&source) {
                Ok(()) => {
                    self.export_message = format!("✔ Saved Misa to {}", source.display());
                }
                Err(e) => {
                    self.export_message = format!("⚠ Save failed: {e}");
                }
            },
            _ => match model.save_urdf() {
                Ok(path) => {
                    self.export_message = format!("✔ Saved to {}", path.display());
                    // Legacy: keep the .misarta.toml sidecar in sync so URDF
                    // workflows that pre-date .misa don't lose actuator /
                    // pose / sequence state on save → reload.
                    if let Err(e) = model.save_sidecar_config(&path) {
                        self.export_message += &format!(" (⚠ config: {e})");
                    }
                }
                Err(e) => {
                    self.export_message = format!("⚠ Save failed: {e}");
                }
            },
        }
    }

    /// Public entry for the Export dialog's `Export` button. Same
    /// pattern as [`Self::do_save`] — pre-flight check then either
    /// proceed or queue a confirmation dialog.
    pub(super) fn do_export(&mut self) {
        if self.export_dir.is_empty() {
            self.export_message = "⚠ Please specify an output directory.".into();
            return;
        }
        self.sync_gait_panel_to_model();
        let Some(model) = self.model.as_ref() else {
            self.export_message = "⚠ No model loaded.".into();
            return;
        };
        // Match the user's selected target format against the registry.
        let registry = crate::format::FormatRegistry::default_registry();
        let target_name = self.export_format.label().split_whitespace().next().unwrap_or("");
        let handler = registry
            .handlers()
            .iter()
            .find(|h| h.name().eq_ignore_ascii_case(target_name)
                || h.name().contains(target_name))
            .map(|h| h.as_ref());
        let issues = if let Some(h) = handler {
            crate::format::analyze_export_compatibility(model, h)
        } else {
            Vec::new()
        };
        if issues.is_empty() {
            self.export_now();
        } else {
            self.export_compat_issues = issues;
            self.pending_export_action = Some(super::PendingExportAction::Export);
        }
    }

    /// Bypass the confirmation dialog and write the export immediately.
    pub(super) fn export_now(&mut self) {
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
            RobotFormat::Misa => {
                let filename = format!("{base_name}.misa");
                let output_path = dir.join(&filename);
                match model.save_as_misa(&output_path) {
                    Ok(()) => {
                        self.export_message = format!(
                            "✔ Exported Misa to {} (full state preserved — no sidecar needed)",
                            output_path.display(),
                        );
                    }
                    Err(e) => {
                        self.export_message = format!("⚠ Misa export failed: {e}");
                    }
                }
            }
        }

        // For non-Misa exports, also drop a `.misarta.toml` sidecar so
        // round-trips through that target format don't silently lose
        // articara-specific data (poses, sequences, actuator gains, etc).
        // For Misa exports the sidecar is redundant — `.misa` already
        // carries everything.
        if fmt != RobotFormat::Misa {
            let model_file = dir.join(&base_name);
            if let Err(e) = model.save_sidecar_config(&model_file) {
                self.export_message += &format!(" (⚠ config: {e})");
            }
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
