use eframe::egui;
use nalgebra as na;

use super::ArticaraApp;
use articara::robot::{GeomData, RobotModel};

impl ArticaraApp {
    pub(super) fn draw_tree_panel(&mut self, ui: &mut egui::Ui) {
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
                let resp = ui.selectable_label(selected, &label);
                if resp.clicked() {
                    self.selected_joint = Some(i);
                    self.selected_link = None;
                }
                self.joint_context_menu(&resp, i, name);
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
                    self.mark_edit(&format!("Remove link '{}'", link_name));
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

    /// Build a rich-text label for a link, appending colored IK markers.
    ///
    /// Uses BMP-safe symbols so they render in egui's default font:
    /// - `⚓` (U+2693) for IK root link  (blue)
    /// - `◆` (U+25C6) for IK-pinned link (orange)
    fn link_tree_label(&self, link_name: &str, visuals: &egui::Visuals) -> egui::text::LayoutJob {
        let is_ik_root = self.ik.root_link.as_deref() == Some(link_name);
        let is_pinned = self.ik.pinned_links.iter().any(|p| p.link_name == link_name);

        let base_color = visuals.text_color();
        let font_id = egui::FontId::proportional(13.0);
        let mut job = egui::text::LayoutJob::default();
        job.append(link_name, 0.0, egui::TextFormat {
            font_id: font_id.clone(),
            color: base_color,
            ..Default::default()
        });
        if is_ik_root {
            job.append(" ⚓", 0.0, egui::TextFormat {
                font_id: font_id.clone(),
                color: egui::Color32::from_rgb(70, 140, 255),
                ..Default::default()
            });
        }
        if is_pinned {
            job.append(" ◆", 0.0, egui::TextFormat {
                font_id: font_id.clone(),
                color: egui::Color32::from_rgb(255, 160, 0),
                ..Default::default()
            });
        }
        job
    }

    pub(super) fn draw_link_tree(&mut self, ui: &mut egui::Ui, link_name: &str) {
        let model = self.model.as_ref().unwrap();
        let link_idx = model.link_map.get(link_name).copied();
        let selected = link_idx.is_some() && self.selected_link == link_idx;
        let label_text = self.link_tree_label(link_name, ui.visuals());

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
            let resp = ui.selectable_label(selected, label_text);
            if resp.clicked() {
                self.selected_link = link_idx;
                self.selected_joint = None;
            }
            self.link_context_menu(&resp, link_name);
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
                    let resp = ui.selectable_label(selected, label_text.clone());
                    if resp.clicked() {
                        self.selected_link = link_idx;
                        self.selected_joint = None;
                    }
                    self.link_context_menu(&resp, link_name);
                })
                .body(|ui| {
                    for (_joint_name, child_link) in &children {
                        self.draw_link_tree(ui, child_link);
                    }
                });
        }
    }

    pub(super) fn draw_add_child_panel(&mut self, ui: &mut egui::Ui) {
        const JOINT_TYPES: &[&str] = &["revolute", "prismatic", "fixed", "continuous"];
        const GEOM_TYPES: &[&str] = &["Box", "Cylinder", "Sphere", "Capsule"];

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
                3 => {
                    // Capsule: radius, half_length
                    ui.horizontal(|ui| {
                        ui.label("Cap:");
                        ui.add(egui::DragValue::new(&mut self.new_geom_size[0]).speed(0.005).prefix("r:"));
                        ui.add(egui::DragValue::new(&mut self.new_geom_size[1]).speed(0.005).prefix("l:"));
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
    pub(super) fn execute_add_child(&mut self) {
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
            3 => GeomData::Capsule {
                radius: self.new_geom_size[0],
                half_length: self.new_geom_size[1] * 0.5,
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

        self.mark_edit(&format!("Add child '{}'", self.new_link_name));
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

    /// Right-click context menu for a link in the tree panel.
    fn link_context_menu(&mut self, resp: &egui::Response, link_name: &str) {
        resp.context_menu(|ui| {
            // Select this link
            if let Some(model) = &self.model {
                if let Some(&li) = model.link_map.get(link_name) {
                    if self.selected_link != Some(li) {
                        self.selected_link = Some(li);
                        self.selected_joint = None;
                    }
                }
            }

            if ui.button("✏ Rename…").clicked() {
                // Select + set rename buffer so properties panel shows the text edit
                if let Some(model) = &self.model {
                    if let Some(&li) = model.link_map.get(link_name) {
                        self.selected_link = Some(li);
                        self.selected_joint = None;
                        self.rename_link_buf = link_name.to_string();
                    }
                }
                ui.close();
            }

            if ui.button("➕ Add Child…").clicked() {
                self.show_add_child = true;
                self.new_parent_link = link_name.to_string();
                if let Some(model) = &self.model {
                    self.new_link_name = model.generate_link_name("new_link");
                    self.new_joint_name = model.generate_joint_name("new_joint");
                }
                ui.close();
            }

            if ui.button("📋 Copy Visuals → Collisions").clicked() {
                if let Some(model) = &mut self.model {
                    if let Some(&li) = model.link_map.get(link_name) {
                        let new_cols: Vec<_> = model.links[li].visuals.iter().map(|v| {
                            articara::robot::CollisionData {
                                origin: v.origin,
                                geometry: v.geometry.clone(),
                            
                                physics: None,
                            }
                        }).collect();
                        if !new_cols.is_empty() {
                            model.links[li].collisions = new_cols;
                            self.needs_upload = true;
                            self.mark_edit(&format!("Copy visuals → collisions for '{}'", link_name));
                        }
                    }
                }
                ui.close();
            }

            ui.separator();

            let is_root = self.model.as_ref().map_or(false, |m| m.root_link == link_name);
            let del_btn = ui.add_enabled(!is_root, egui::Button::new("🗑 Delete"))
                .on_disabled_hover_text("Cannot delete root link");
            if del_btn.clicked() {
                if let Some(model) = &mut self.model {
                    match model.remove_link(link_name) {
                        Ok(_removed) => {
                            self.selected_link = None;
                            self.selected_joint = None;
                            self.needs_upload = true;
                            self.mark_edit(&format!("Delete link '{}'", link_name));
                        }
                        Err(e) => {
                            self.status_message = format!("Remove error: {e}");
                        }
                    }
                }
                ui.close();
            }
        });
    }

    /// Right-click context menu for a joint in the tree panel.
    fn joint_context_menu(&mut self, resp: &egui::Response, ji: usize, joint_name: &str) {
        resp.context_menu(|ui| {
            // Select this joint
            if self.selected_joint != Some(ji) {
                self.selected_joint = Some(ji);
                self.selected_link = None;
            }

            if ui.button("✏ Rename…").clicked() {
                self.selected_joint = Some(ji);
                self.selected_link = None;
                self.rename_joint_buf = joint_name.to_string();
                ui.close();
            }

            let mut type_change_msg: Option<String> = None;
            ui.menu_button("🔄 Change Type", |ui| {
                const JOINT_TYPES: &[&str] = &["revolute", "prismatic", "fixed", "continuous"];
                if let Some(model) = &mut self.model {
                    let current = model.joints[ji].joint_type.clone();
                    for &jt in JOINT_TYPES {
                        let is_current = current == jt;
                        if ui.selectable_label(is_current, jt).clicked() && !is_current {
                            model.joints[ji].joint_type = jt.to_string();
                            model.rebuild_misarta_model();
                            type_change_msg = Some(format!("Change joint '{}' type to {}", joint_name, jt));
                            ui.close();
                            break;
                        }
                    }
                }
            });
            if let Some(msg) = type_change_msg {
                self.needs_upload = true;
                self.mark_edit(&msg);
            }
        });
    }
}
