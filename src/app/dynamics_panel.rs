use eframe::egui;

use super::ArticaraApp;
use crate::dynamics::{self, StaticAnalysis, DynSim, JumpPhase, PayloadPhase};

impl ArticaraApp {
    pub(super) fn draw_dynamics_panel(&mut self, ui: &mut egui::Ui) {
        egui::CollapsingHeader::new("⚡ Dynamics Analysis")
            .default_open(false)
            .show(ui, |ui| {
                if self.model.is_none() {
                    ui.label("(no model loaded)");
                    return;
                }

                // --- End-effector link selector for payload ---
                let link_names: Vec<String> = self
                    .model
                    .as_ref()
                    .unwrap()
                    .links
                    .iter()
                    .map(|l| l.name.clone())
                    .collect();

                ui.horizontal(|ui| {
                    ui.label("EE link:");
                    let current_label = self
                        .dynamics_ee_link
                        .as_deref()
                        .unwrap_or("(select)")
                        .to_string();
                    egui::ComboBox::from_id_salt("dynamics_ee_link")
                        .selected_text(&current_label)
                        .show_ui(ui, |ui| {
                            if ui
                                .selectable_label(self.dynamics_ee_link.is_none(), "(none)")
                                .clicked()
                            {
                                self.dynamics_ee_link = None;
                            }
                            for name in &link_names {
                                let sel =
                                    self.dynamics_ee_link.as_deref() == Some(name.as_str());
                                if ui.selectable_label(sel, name).clicked() {
                                    self.dynamics_ee_link = Some(name.clone());
                                }
                            }
                        });
                });

                ui.separator();
                ui.label(egui::RichText::new("Jump Estimation").strong());

                // --- Body link selector ---
                ui.horizontal(|ui| {
                    ui.label("Body:");
                    let body_label = self
                        .dynamics_body_link
                        .as_deref()
                        .unwrap_or("Auto (URDF root)")
                        .to_string();
                    egui::ComboBox::from_id_salt("dynamics_body_link")
                        .selected_text(&body_label)
                        .show_ui(ui, |ui| {
                            if ui
                                .selectable_label(
                                    self.dynamics_body_link.is_none(),
                                    "Auto (URDF root)",
                                )
                                .clicked()
                            {
                                self.dynamics_body_link = None;
                            }
                            for name in &link_names {
                                let sel =
                                    self.dynamics_body_link.as_deref() == Some(name.as_str());
                                if ui.selectable_label(sel, name).clicked() {
                                    self.dynamics_body_link = Some(name.clone());
                                }
                            }
                        });
                })
                .response
                .on_hover_text("The torso/base link that gets launched upward");

                // --- Ground link selector for jump ---
                ui.horizontal(|ui| {
                    ui.label("Ground:");
                    let current_label = if self.dynamics_ground_links.is_empty() {
                        "(select)".to_string()
                    } else {
                        self.dynamics_ground_links.join(", ")
                    };
                    egui::ComboBox::from_id_salt("dynamics_ground_links")
                        .selected_text(&current_label)
                        .show_ui(ui, |ui| {
                            for name in &link_names {
                                let mut checked = self.dynamics_ground_links.contains(name);
                                if ui.checkbox(&mut checked, name).changed() {
                                    if checked {
                                        self.dynamics_ground_links.push(name.clone());
                                    } else {
                                        self.dynamics_ground_links
                                            .retain(|n| n != name);
                                    }
                                }
                            }
                        });
                })
                .response
                .on_hover_text("Foot/end-effector links in contact with the ground");

                // --- Speed slider ---
                ui.horizontal(|ui| {
                    ui.label("Speed:");
                    ui.add(
                        egui::Slider::new(&mut self.dynamics_sim_speed, 0.1..=5.0)
                            .logarithmic(true)
                            .text("×"),
                    );
                });

                ui.separator();

                // --- Simulation controls ---
                let sim_active = self.dynamics_sim.is_some();

                ui.horizontal(|ui| {
                    // Static analysis
                    if ui
                        .add_enabled(!sim_active, egui::Button::new("📊 Analyze"))
                        .clicked()
                    {
                        if let Some(ref model) = self.model {
                            let result = dynamics::analyze(
                                model,
                                self.dynamics_ee_link.as_deref(),
                                self.dynamics_body_link.as_deref(),
                                &self.dynamics_ground_links,
                            );
                            self.dynamics_result = Some(result);
                        }
                    }
                });

                ui.horizontal(|ui| {
                    // Jump simulation
                    let can_jump = !sim_active && !self.dynamics_ground_links.is_empty();
                    if ui
                        .add_enabled(can_jump, egui::Button::new("🦘 Play Jump"))
                        .on_hover_text(
                            "Animate the robot jumping: extend legs → ballistic flight → land",
                        )
                        .clicked()
                    {
                        if let Some(ref model) = self.model {
                            if let Some(sim) = dynamics::start_jump_sim(
                                model,
                                &self.dynamics_ground_links,
                                self.dynamics_body_link.as_deref(),
                                self.dynamics_sim_speed,
                            ) {
                                self.dynamics_sim = Some(DynSim::Jump(sim));
                            } else {
                                self.status_message =
                                    "Cannot start jump sim (no leg joints with effort limits?)"
                                        .into();
                            }
                        }
                    }

                    // Payload simulation
                    let can_payload = !sim_active && self.dynamics_ee_link.is_some();
                    if ui
                        .add_enabled(can_payload, egui::Button::new("🏋 Play Payload"))
                        .on_hover_text(
                            "Gradually load the end-effector and visualise joint torque utilisation",
                        )
                        .clicked()
                    {
                        if let Some(ref model) = self.model {
                            let ee = self.dynamics_ee_link.as_deref().unwrap_or("");
                            if let Some(sim) = dynamics::start_payload_sim(
                                model,
                                ee,
                                self.dynamics_sim_speed,
                            ) {
                                self.dynamics_sim = Some(DynSim::Payload(sim));
                            } else {
                                self.status_message =
                                    "Cannot start payload sim (no effort limits or 0 capacity?)"
                                        .into();
                            }
                        }
                    }
                });

                // Stop button
                if sim_active {
                    ui.horizontal(|ui| {
                        if ui.button("⏹ Stop").clicked() {
                            // Restore model state
                            self.stop_dynamics_sim();
                        }
                    });
                }

                // --- Live simulation status ---
                self.draw_sim_status(ui);

                ui.separator();

                // --- Display static analysis results ---
                if let Some(ref result) = self.dynamics_result {
                    self.draw_dynamics_results(ui, result);
                }
            });
    }

    /// Draw live simulation status readout.
    fn draw_sim_status(&self, ui: &mut egui::Ui) {
        match &self.dynamics_sim {
            Some(DynSim::Jump(sim)) => {
                ui.separator();
                let phase_str = match sim.phase {
                    JumpPhase::Extension => "🦵 Extension",
                    JumpPhase::Flight => "🚀 Flight",
                    JumpPhase::Landed => "🛬 Landed",
                };
                ui.colored_label(
                    egui::Color32::from_rgb(100, 200, 255),
                    format!("▶ Jump: {}", phase_str),
                );

                // --- Per-step physics readout ---
                ui.label(format!(
                    "Height: {:.3} m  (max {:.3} m)",
                    sim.step_info.height.max(0.0),
                    sim.max_height_reached,
                ));
                ui.label(format!(
                    "Velocity: {:.3} m/s",
                    sim.step_info.velocity_z,
                ));

                if sim.phase == JumpPhase::Extension {
                    // GRF readout
                    let grf_color = if sim.step_info.grf_z >= 0.0 {
                        egui::Color32::from_rgb(100, 200, 100)
                    } else {
                        egui::Color32::from_rgb(255, 80, 80)
                    };
                    ui.horizontal(|ui| {
                        ui.label("GRF:");
                        ui.colored_label(grf_color, format!("{:.1} N", sim.step_info.grf_z));
                    });

                    let pct = (sim.phase_time / sim.extension_duration * 100.0).min(100.0);
                    ui.label(format!("Extension: {:.0}%", pct));
                }

                // Progress bar
                let est_flight = if sim.launch_velocity > 0.0 {
                    2.0 * sim.launch_velocity / 9.80665_f32
                } else if sim.phase == JumpPhase::Extension {
                    // Estimate from current velocity
                    let v = sim.base_velocity_z.max(0.0);
                    2.0 * v / 9.80665_f32
                } else {
                    0.5
                };
                let total_dur = sim.extension_duration + est_flight + sim.landed_hold;
                let elapsed = match sim.phase {
                    JumpPhase::Extension => sim.phase_time,
                    JumpPhase::Flight => sim.extension_duration + sim.phase_time,
                    JumpPhase::Landed => {
                        sim.extension_duration + est_flight + sim.phase_time
                    }
                };
                ui.add(
                    egui::ProgressBar::new((elapsed / total_dur).clamp(0.0, 1.0))
                        .text(format!("{:.1}s / {:.1}s", elapsed, total_dur)),
                );

                // Per-joint torque utilisation bars (same renderer as payload)
                if !sim.step_info.joint_utilisation.is_empty() {
                    ui.separator();
                    ui.label(egui::RichText::new("Joint Torque Load").small().strong());
                    self.draw_jump_utilisation_bars(ui, &sim.step_info.joint_utilisation);
                }
            }
            Some(DynSim::Payload(sim)) => {
                ui.separator();
                let phase_str = match sim.phase {
                    PayloadPhase::Ramping => "📈 Loading",
                    PayloadPhase::Holding => "⚖️ Holding max",
                    PayloadPhase::Done => "✅ Done",
                };
                ui.colored_label(
                    egui::Color32::from_rgb(255, 200, 100),
                    format!("▶ Payload: {}", phase_str),
                );
                ui.label(format!(
                    "Current: {:.2} / {:.2} kg",
                    sim.current_mass, sim.max_mass
                ));

                // Progress bar
                let pct = (sim.current_mass / sim.max_mass).clamp(0.0, 1.0) as f32;
                ui.add(
                    egui::ProgressBar::new(pct)
                        .text(format!("{:.1}%", pct * 100.0)),
                );

                // Per-joint utilisation bars
                if !sim.joint_utilisation.is_empty() {
                    ui.separator();
                    ui.label(egui::RichText::new("Joint Load").small().strong());
                    self.draw_utilisation_bars(ui, &sim.joint_utilisation);
                }
            }
            None => {}
        }
    }

    /// Draw torque utilisation bars for jump sim: (joint_idx, ratio, contributes).
    ///
    /// Contributing joints show a coloured bar (green/yellow/red).
    /// Hold joints are dimmed and labelled "hold".
    fn draw_jump_utilisation_bars(&self, ui: &mut egui::Ui, utils: &[(usize, f64, bool)]) {
        let available_width = ui.available_width().min(220.0);
        let bar_height = 12.0;

        for &(ji, util, contributes) in utils {
            if let Some(ref model) = self.model {
                if ji < model.joints.len() {
                    let jname = &model.joints[ji].name;
                    let name_short = if jname.len() > 10 {
                        format!("{}\u{2026}", &jname[..9])
                    } else {
                        jname.clone()
                    };

                    ui.horizontal(|ui| {
                        let label_color = if contributes {
                            egui::Color32::from_gray(200)
                        } else {
                            egui::Color32::from_gray(100)
                        };
                        ui.label(
                            egui::RichText::new(&name_short)
                                .small()
                                .monospace()
                                .color(label_color),
                        );
                        let (rect, _) = ui.allocate_exact_size(
                            egui::vec2(available_width, bar_height),
                            egui::Sense::hover(),
                        );
                        let painter = ui.painter_at(rect);
                        painter.rect_filled(
                            rect,
                            2.0,
                            egui::Color32::from_gray(50),
                        );
                        let frac = (util as f32).clamp(0.0, 1.5);
                        let color = if !contributes {
                            // Hold joint: dim blue-grey
                            egui::Color32::from_rgb(80, 100, 130)
                        } else if util <= 0.7 {
                            egui::Color32::from_rgb(80, 200, 80)
                        } else if util <= 1.0 {
                            egui::Color32::from_rgb(255, 200, 50)
                        } else {
                            egui::Color32::from_rgb(255, 60, 60)
                        };
                        let bar_w = (rect.width() * frac / 1.5).min(rect.width());
                        let bar = egui::Rect::from_min_max(
                            rect.min,
                            egui::pos2(rect.min.x + bar_w, rect.max.y),
                        );
                        painter.rect_filled(bar, 2.0, color);
                        let suffix = if contributes { "" } else { " hold" };
                        ui.label(
                            egui::RichText::new(format!("{:.0}%{}", util * 100.0, suffix))
                                .small()
                                .monospace()
                                .color(label_color),
                        );
                    });
                }
            }
        }
    }

    /// Draw torque utilisation bars for a list of (joint_idx, ratio) pairs.
    fn draw_utilisation_bars(&self, ui: &mut egui::Ui, utils: &[(usize, f64)]) {
        let available_width = ui.available_width().min(220.0);
        let bar_height = 12.0;

        for &(ji, util) in utils {
            if let Some(ref model) = self.model {
                if ji < model.joints.len() {
                    let jname = &model.joints[ji].name;
                    let name_short = if jname.len() > 10 {
                        format!("{}…", &jname[..9])
                    } else {
                        jname.clone()
                    };

                    ui.horizontal(|ui| {
                        ui.label(
                            egui::RichText::new(&name_short)
                                .small()
                                .monospace(),
                        );
                        let (rect, _) = ui.allocate_exact_size(
                            egui::vec2(available_width, bar_height),
                            egui::Sense::hover(),
                        );
                        let painter = ui.painter_at(rect);
                        painter.rect_filled(
                            rect,
                            2.0,
                            egui::Color32::from_gray(50),
                        );
                        let frac = (util as f32).clamp(0.0, 1.5);
                        let color = if util <= 0.7 {
                            egui::Color32::from_rgb(80, 200, 80)
                        } else if util <= 1.0 {
                            egui::Color32::from_rgb(255, 200, 50)
                        } else {
                            egui::Color32::from_rgb(255, 60, 60)
                        };
                        let bar_w = (rect.width() * frac / 1.5).min(rect.width());
                        let bar = egui::Rect::from_min_max(
                            rect.min,
                            egui::pos2(rect.min.x + bar_w, rect.max.y),
                        );
                        painter.rect_filled(bar, 2.0, color);
                        ui.label(
                            egui::RichText::new(format!("{:.0}%", util * 100.0))
                                .small()
                                .monospace(),
                        );
                    });
                }
            }
        }
    }

    /// Stop any running dynamics simulation and restore the model.
    pub(super) fn stop_dynamics_sim(&mut self) {
        if let Some(sim) = self.dynamics_sim.take() {
            if let Some(ref mut model) = self.model {
                match sim {
                    DynSim::Jump(js) => {
                        model.joint_positions = js.saved_positions;
                        model.base_transform = js.saved_base_transform;
                    }
                    DynSim::Payload(ps) => {
                        model.joint_positions = ps.saved_positions;
                        model.base_transform = ps.saved_base_transform;
                    }
                }
            }
        }
        self.dynamics_last_instant = None;
    }

    fn draw_dynamics_results(&self, ui: &mut egui::Ui, result: &StaticAnalysis) {
        // ===== Joint Torque Table =====
        egui::CollapsingHeader::new("Gravity Torques")
            .default_open(true)
            .show(ui, |ui| {
                egui::Grid::new("torque_grid")
                    .num_columns(4)
                    .striped(true)
                    .show(ui, |ui| {
                        ui.strong("Joint");
                        ui.strong("τ_grav");
                        ui.strong("Limit");
                        ui.strong("Margin");
                        ui.end_row();

                        for info in &result.joint_torques {
                            ui.label(&info.joint_name);

                            // Gravity torque
                            ui.label(format!("{:.3}", info.gravity_torque));

                            // Effort limit
                            if info.effort_limit > 0.0 {
                                ui.label(format!("{:.1}", info.effort_limit));
                            } else {
                                ui.label("—");
                            }

                            // Margin with color
                            if info.effort_limit > 0.0 {
                                let color = if info.torque_margin >= 0.0 {
                                    egui::Color32::from_rgb(100, 200, 100) // green
                                } else {
                                    egui::Color32::from_rgb(255, 80, 80)   // red
                                };
                                ui.colored_label(color, format!("{:.3}", info.torque_margin));
                            } else {
                                ui.label("—");
                            }

                            ui.end_row();
                        }
                    });

                // Torque bar chart
                self.draw_torque_bars(ui, &result.joint_torques);
            });

        // ===== Payload Capacity =====
        if let Some(ref payload) = result.payload {
            ui.separator();
            egui::CollapsingHeader::new("🏋 Payload Capacity")
                .default_open(true)
                .show(ui, |ui| {
                    let color = if payload.max_mass_kg > 0.0 {
                        egui::Color32::from_rgb(100, 200, 100)
                    } else {
                        egui::Color32::from_rgb(255, 80, 80)
                    };
                    ui.horizontal(|ui| {
                        ui.label("Max payload:");
                        ui.colored_label(color, format!("{:.2} kg", payload.max_mass_kg));
                    });
                    ui.label(format!("Limiting joint: {}", payload.limiting_joint));
                    ui.label(format!(
                        "EE pos: ({:.3}, {:.3}, {:.3})",
                        payload.ee_position.x,
                        payload.ee_position.y,
                        payload.ee_position.z,
                    ));

                    // Per-joint payload contribution
                    egui::CollapsingHeader::new("Per-joint payload torque")
                        .default_open(false)
                        .show(ui, |ui| {
                            egui::Grid::new("payload_grid")
                                .num_columns(2)
                                .striped(true)
                                .show(ui, |ui| {
                                    ui.strong("Joint");
                                    ui.strong("τ/kg");
                                    ui.end_row();
                                    for info in &result.joint_torques {
                                        if info.payload_torque_per_kg.abs() > 1e-6 {
                                            ui.label(&info.joint_name);
                                            ui.label(format!(
                                                "{:.3} N·m/kg",
                                                info.payload_torque_per_kg
                                            ));
                                            ui.end_row();
                                        }
                                    }
                                });
                        });
                });
        }

        // ===== Jump Height =====
        if let Some(ref jump) = result.jump {
            ui.separator();
            egui::CollapsingHeader::new("🦘 Jump Estimate")
                .default_open(true)
                .show(ui, |ui| {
                    ui.horizontal(|ui| {
                        ui.label("Max height:");
                        ui.colored_label(
                            egui::Color32::from_rgb(100, 180, 255),
                            format!("{:.3} m", jump.max_height_m),
                        );
                    });
                    ui.label(format!("Total energy: {:.2} J", jump.total_energy_j));
                    ui.label(format!("Total mass: {:.2} kg", jump.total_mass_kg));

                    egui::CollapsingHeader::new("Per-joint energy")
                        .default_open(false)
                        .show(ui, |ui| {
                            egui::Grid::new("jump_grid")
                                .num_columns(2)
                                .striped(true)
                                .show(ui, |ui| {
                                    ui.strong("Joint");
                                    ui.strong("Energy");
                                    ui.end_row();
                                    for (name, energy) in &jump.per_joint_energy {
                                        ui.label(name);
                                        ui.label(format!("{:.3} J", energy));
                                        ui.end_row();
                                    }
                                });
                        });

                    ui.separator();
                    ui.label(
                        egui::RichText::new(
                            "⚠ Upper bound estimate. Assumes ideal energy transfer \
                             and half joint range as extension stroke.",
                        )
                        .small()
                        .weak(),
                    );
                });
        }
    }

    fn draw_torque_bars(&self, ui: &mut egui::Ui, torques: &[dynamics::JointTorqueInfo]) {
        // Find the max scale for normalization
        let max_val = torques
            .iter()
            .map(|t| {
                t.gravity_torque
                    .abs()
                    .max(t.effort_limit.abs())
            })
            .fold(0.0_f64, f64::max);

        if max_val < 1e-12 {
            return;
        }

        let available_width = ui.available_width().min(250.0);
        let bar_height = 14.0;

        for info in torques {
            if info.effort_limit <= 0.0 {
                continue; // skip joints without effort limits
            }

            ui.horizontal(|ui| {
                ui.set_min_width(available_width + 80.0);

                // Joint name (truncated)
                let name_short = if info.joint_name.len() > 12 {
                    format!("{}…", &info.joint_name[..11])
                } else {
                    info.joint_name.clone()
                };
                ui.label(
                    egui::RichText::new(name_short)
                        .small()
                        .monospace(),
                );

                // Draw bar
                let (rect, _) = ui.allocate_exact_size(
                    egui::vec2(available_width, bar_height),
                    egui::Sense::hover(),
                );

                let painter = ui.painter_at(rect);

                // Background (effort limit = full width)
                painter.rect_filled(rect, 2.0, egui::Color32::from_gray(50));

                // Effort limit marker
                let limit_frac = (info.effort_limit / max_val).min(1.0) as f32;
                let limit_rect = egui::Rect::from_min_max(
                    rect.min,
                    egui::pos2(
                        rect.min.x + rect.width() * limit_frac,
                        rect.max.y,
                    ),
                );
                painter.rect_filled(limit_rect, 2.0, egui::Color32::from_gray(80));

                // Gravity torque bar
                let tau_frac = (info.gravity_torque.abs() / max_val).min(1.0) as f32;
                let bar_color = if info.torque_margin >= 0.0 {
                    egui::Color32::from_rgb(80, 180, 80) // green
                } else {
                    egui::Color32::from_rgb(230, 60, 60)  // red
                };
                let tau_rect = egui::Rect::from_min_max(
                    rect.min,
                    egui::pos2(
                        rect.min.x + rect.width() * tau_frac,
                        rect.max.y,
                    ),
                );
                painter.rect_filled(tau_rect, 2.0, bar_color);

                // Utilization percentage
                let pct = if info.effort_limit > 0.0 {
                    (info.gravity_torque.abs() / info.effort_limit * 100.0) as i32
                } else {
                    0
                };
                ui.label(
                    egui::RichText::new(format!("{}%", pct))
                        .small()
                        .monospace(),
                );
            });
        }
    }
}
