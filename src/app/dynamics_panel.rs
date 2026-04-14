use eframe::egui;

use super::ArticaraApp;
use crate::dynamics::{self, StaticAnalysis, DynSim, JumpPhase, JumpSimResult, PayloadPhase, SimGraphData};

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
                    // Auto-detect tip (leaf) links
                    if ui
                        .small_button("Auto")
                        .on_hover_text(
                            "Auto-select leaf links (links with no child joints) as ground contacts",
                        )
                        .clicked()
                    {
                        if let Some(ref model) = self.model {
                            self.dynamics_ground_links.clear();
                            for link in &model.links {
                                let has_children = model
                                    .children_joints
                                    .get(&link.name)
                                    .is_some_and(|v| !v.is_empty());
                                if !has_children && link.name != model.root_link {
                                    self.dynamics_ground_links.push(link.name.clone());
                                }
                            }
                        }
                    }
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

                // --- Extension duration slider ---
                {
                    let mut use_auto = self.dynamics_extension_duration.is_none();
                    ui.horizontal(|ui| {
                        ui.label("Ext dur:");
                        if ui.checkbox(&mut use_auto, "Auto").changed() {
                            if use_auto {
                                self.dynamics_extension_duration = None;
                            } else {
                                self.dynamics_extension_duration = Some(0.3);
                            }
                        }
                        if let Some(ref mut dur) = self.dynamics_extension_duration {
                            ui.add(
                                egui::Slider::new(dur, 0.05..=3.0)
                                    .suffix(" s")
                                    .fixed_decimals(2),
                            );
                        }
                    })
                    .response
                    .on_hover_text("Extension phase duration. Auto = computed from joint velocities.");
                }

                // --- Torque limit enforcement ---
                ui.checkbox(
                    &mut self.dynamics_enforce_torque_limits,
                    "Enforce torque limits",
                )
                .on_hover_text(
                    "When checked, joints whose gravity torque approaches the URDF \
                     effort limit will have their IK motion scaled back during extension.",
                );

                // --- Retract after extension ---
                ui.checkbox(
                    &mut self.dynamics_enable_retract,
                    "Retract after extend",
                )
                .on_hover_text(
                    "After full extension, rapidly pull legs back to the initial pose \
                     while still on the ground. This adds upward momentum for more hang time.",
                );

                // --- Launch axes ---
                ui.horizontal(|ui| {
                    ui.label("Launch:");
                    ui.checkbox(&mut self.dynamics_launch_axes[0], "X");
                    ui.checkbox(&mut self.dynamics_launch_axes[1], "Y");
                    ui.checkbox(&mut self.dynamics_launch_axes[2], "Z");
                })
                .response
                .on_hover_text("Which axes the body link can move during flight");

                // --- Locked joints selector ---
                {
                    let joint_names: Vec<String> = self
                        .model
                        .as_ref()
                        .unwrap()
                        .joints
                        .iter()
                        .filter(|j| j.joint_type != "fixed")
                        .map(|j| j.name.clone())
                        .collect();
                    let locked_label = if self.dynamics_locked_joints.is_empty() {
                        "(none)".to_string()
                    } else {
                        format!("{} locked", self.dynamics_locked_joints.len())
                    };
                    ui.horizontal(|ui| {
                        ui.label("Lock:");
                        egui::ComboBox::from_id_salt("dynamics_locked_joints")
                            .selected_text(&locked_label)
                            .show_ui(ui, |ui| {
                                for name in &joint_names {
                                    let mut checked =
                                        self.dynamics_locked_joints.contains(name);
                                    if ui.checkbox(&mut checked, name).changed() {
                                        if checked {
                                            self.dynamics_locked_joints
                                                .insert(name.clone());
                                        } else {
                                            self.dynamics_locked_joints.remove(name);
                                        }
                                    }
                                }
                            });
                        if ui.small_button("Clear").clicked() {
                            self.dynamics_locked_joints.clear();
                        }
                    })
                    .response
                    .on_hover_text(
                        "Joints to lock (hold at initial angle) during jump simulation",
                    );
                }

                // --- Sim config save/load ---
                ui.horizontal(|ui| {
                    ui.label("Config:");
                    ui.add_sized(
                        egui::vec2(100.0, 18.0),
                        egui::TextEdit::singleline(&mut self.sim_config_path),
                    );
                    if ui.small_button("💾").on_hover_text("Save sim config").clicked() {
                        if !self.sim_config_path.is_empty() {
                            let path = std::path::PathBuf::from(&self.sim_config_path);
                            match save_sim_config(self, &path) {
                                Ok(()) => {
                                    self.status_message =
                                        format!("Saved sim config → {}", path.display());
                                }
                                Err(e) => {
                                    self.status_message =
                                        format!("Save sim config error: {e}");
                                }
                            }
                        }
                    }
                    if ui.small_button("📂").on_hover_text("Load sim config").clicked() {
                        if !self.sim_config_path.is_empty() {
                            let path = std::path::PathBuf::from(&self.sim_config_path);
                            match load_sim_config(&path) {
                                Ok(cfg) => {
                                    apply_sim_config(self, cfg);
                                    self.status_message =
                                        format!("Loaded sim config ← {}", path.display());
                                }
                                Err(e) => {
                                    self.status_message =
                                        format!("Load sim config error: {e}");
                                }
                            }
                        }
                    }
                    if ui
                        .small_button("…")
                        .on_hover_text("Browse for sim config file")
                        .clicked()
                    {
                        let start = if self.sim_config_path.is_empty() {
                            None
                        } else {
                            Some(
                                std::path::Path::new(&self.sim_config_path)
                                    .to_path_buf(),
                            )
                        };
                        self.dlg_open_sim_config.open(
                            "Load Sim Config",
                            super::file_dialog::FileDialogMode::Open,
                            start.as_deref(),
                            &["toml"],
                        );
                    }
                });

                ui.separator();

                // --- Graph link selector ---
                ui.horizontal(|ui| {
                    ui.label("Graph:");
                    let graph_label = self
                        .dynamics_graph_link
                        .as_deref()
                        .unwrap_or("(Body link)")
                        .to_string();
                    egui::ComboBox::from_id_salt("dynamics_graph_link")
                        .selected_text(&graph_label)
                        .show_ui(ui, |ui| {
                            if ui
                                .selectable_label(
                                    self.dynamics_graph_link.is_none(),
                                    "(Body link)",
                                )
                                .clicked()
                            {
                                self.dynamics_graph_link = None;
                            }
                            for name in &link_names {
                                let sel =
                                    self.dynamics_graph_link.as_deref() == Some(name.as_str());
                                if ui.selectable_label(sel, name).clicked() {
                                    self.dynamics_graph_link = Some(name.clone());
                                }
                            }
                        });
                })
                .response
                .on_hover_text("Link whose position/velocity/acceleration to plot");

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
                        .add_enabled(can_jump, egui::Button::new("🦘 Jump"))
                        .on_hover_text(
                            "Prepare jump simulation (use ▶ Play or ⏭ Step to start)",
                        )
                        .clicked()
                    {
                        if let Some(ref mut model) = self.model {
                            if let Some(sim) = dynamics::start_jump_sim(
                                model,
                                &self.dynamics_ground_links,
                                self.dynamics_body_link.as_deref(),
                                self.dynamics_sim_speed,
                                &self.dynamics_locked_joints,
                                self.dynamics_launch_axes,
                                self.dynamics_extension_duration,
                                self.dynamics_enforce_torque_limits,
                                self.dynamics_enable_retract,
                                self.dynamics_graph_link.as_deref(),
                            ) {
                                self.dynamics_sim_result = None; // clear previous result
                                self.dynamics_sim = Some(DynSim::Jump(sim));
                                self.dynamics_sim_paused = true; // start paused
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

                // Playback controls
                if sim_active {
                    ui.horizontal(|ui| {
                        if ui.button("⏹ Stop").clicked() {
                            self.dynamics_sim_paused = false;
                            self.stop_dynamics_sim();
                        }
                        if self.dynamics_sim_paused {
                            if ui.button("▶ Play").clicked() {
                                self.dynamics_sim_paused = false;
                            }
                        } else {
                            if ui.button("⏸ Pause").clicked() {
                                self.dynamics_sim_paused = true;
                            }
                        }
                    });
                    // Step buttons (only when paused)
                    if self.dynamics_sim_paused {
                        ui.horizontal(|ui| {
                            ui.label("Step:");
                            for (label, dt) in [
                                ("1ms",  0.001_f32),
                                ("10ms", 0.01_f32),
                                ("100ms", 0.1_f32),
                            ] {
                                if ui.button(format!("⏭ {}", label))
                                    .on_hover_text(format!("Advance {} then pause", label))
                                    .clicked()
                                {
                                    self.dynamics_step_dt = Some(dt);
                                }
                            }
                        });
                    }
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
                    JumpPhase::Retract => "🔄 Retract",
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

                if sim.phase == JumpPhase::Extension || sim.phase == JumpPhase::Retract {
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

                    if sim.phase == JumpPhase::Extension {
                        let pct = (sim.phase_time / sim.extension_duration * 100.0).min(100.0);
                        ui.label(format!("Extension: {:.0}%", pct));
                    } else {
                        let pct = (sim.phase_time / sim.retract_duration * 100.0).min(100.0);
                        ui.label(format!("Retract: {:.0}%", pct));
                    }
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
                let retract_dur = if sim.enable_retract { sim.retract_duration } else { 0.0 };
                let total_dur = sim.extension_duration + retract_dur + est_flight + sim.landed_hold;
                let elapsed = match sim.phase {
                    JumpPhase::Extension => sim.phase_time,
                    JumpPhase::Retract => sim.extension_duration + sim.phase_time,
                    JumpPhase::Flight => sim.extension_duration + retract_dur + sim.phase_time,
                    JumpPhase::Landed => {
                        sim.extension_duration + retract_dur + est_flight + sim.phase_time
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

    /// Draw jump simulation result as a standalone egui::Window dialog.
    pub(super) fn draw_sim_result_window(&mut self, ctx: &egui::Context) {
        if !self.show_sim_result_window {
            return;
        }
        let result = match &self.dynamics_sim_result {
            Some(r) => r.clone(),
            None => {
                self.show_sim_result_window = false;
                return;
            }
        };

        let mut open = true;
        let mut close_clicked = false;

        egui::Window::new("📋 Jump Simulation Result")
            .open(&mut open)
            .resizable(true)
            .default_width(520.0)
            .default_height(700.0)
            .show(ctx, |ui| {
                egui::ScrollArea::vertical().show(ui, |ui| {
                // --- Summary ---
                ui.horizontal(|ui| {
                    ui.label(
                        egui::RichText::new("Reached Height")
                            .strong()
                            .size(14.0),
                    );
                    ui.colored_label(
                        egui::Color32::from_rgb(80, 200, 255),
                        egui::RichText::new(format!("{:.4} m", result.max_height))
                            .size(18.0)
                            .strong(),
                    );
                });
                ui.add_space(4.0);
                ui.horizontal(|ui| {
                    ui.label("Extension duration:");
                    ui.monospace(format!("{:.3} s", result.extension_duration));
                });

                if result.joint_peaks.is_empty() {
                    ui.separator();
                    ui.label("No joint data recorded.");
                } else {
                    ui.separator();
                    ui.label(
                        egui::RichText::new("Per-Joint Peaks")
                            .strong()
                            .size(13.0),
                    );
                    ui.add_space(2.0);

                    egui::ScrollArea::vertical()
                        .max_height(300.0)
                        .show(ui, |ui| {
                            egui::Grid::new("sim_result_dialog_grid")
                                .num_columns(6)
                                .striped(true)
                                .min_col_width(55.0)
                                .show(ui, |ui| {
                                    ui.strong("Joint");
                                    ui.strong("Peak τ (N·m)");
                                    ui.strong("θ@τ (deg)");
                                    ui.strong("Peak ω (rad/s)");
                                    ui.strong("θ@ω (deg)");
                                    ui.strong("Role");
                                    ui.end_row();

                                    for jp in &result.joint_peaks {
                                        ui.label(
                                            egui::RichText::new(&jp.joint_name)
                                                .monospace(),
                                        );
                                        ui.label(
                                            egui::RichText::new(format!(
                                                "{:.3}",
                                                jp.peak_torque
                                            ))
                                            .monospace(),
                                        );
                                        ui.label(
                                            egui::RichText::new(format!(
                                                "{:.1}",
                                                jp.peak_torque_angle.to_degrees()
                                            ))
                                            .monospace(),
                                        );
                                        ui.label(
                                            egui::RichText::new(format!(
                                                "{:.3}",
                                                jp.peak_velocity
                                            ))
                                            .monospace(),
                                        );
                                        ui.label(
                                            egui::RichText::new(format!(
                                                "{:.1}",
                                                jp.peak_velocity_angle.to_degrees()
                                            ))
                                            .monospace(),
                                        );
                                        let (role, color) = if jp.contributes {
                                            (
                                                "drive",
                                                egui::Color32::from_rgb(80, 200, 80),
                                            )
                                        } else {
                                            (
                                                "hold",
                                                egui::Color32::from_gray(130),
                                            )
                                        };
                                        ui.colored_label(color, role);
                                        ui.end_row();
                                    }
                                });
                        });
                }

                ui.separator();
                if ui.button("Close").clicked() {
                    close_clicked = true;
                }

                // --- Graph plots (position / velocity / acceleration) ---
                if !result.graph_data.time.is_empty() {
                    ui.separator();
                    ui.label(egui::RichText::new(
                        format!("📈 {} — 1 ms resolution", result.graph_data.link_name)
                    ).strong().size(13.0));
                    ui.add_space(2.0);

                    let gd = &result.graph_data;
                    let n = gd.time.len();

                    let to_points = |vals: &[f32]| -> egui_plot::PlotPoints {
                        egui_plot::PlotPoints::new(
                            (0..n)
                                .map(|i| [gd.time[i] as f64 * 1000.0, vals[i] as f64])
                                .collect::<Vec<_>>(),
                        )
                    };

                    // Position
                    ui.label(egui::RichText::new("Position (m)").strong());
                    egui_plot::Plot::new("result_pos_plot")
                        .height(150.0)
                        .x_axis_label("Time (ms)")
                        .legend(egui_plot::Legend::default())
                        .show(ui, |plot_ui| {
                            plot_ui.line(
                                egui_plot::Line::new("X", to_points(&gd.pos_x))
                                    .color(egui::Color32::from_rgb(255, 100, 100)),
                            );
                            plot_ui.line(
                                egui_plot::Line::new("Y", to_points(&gd.pos_y))
                                    .color(egui::Color32::from_rgb(100, 255, 100)),
                            );
                            plot_ui.line(
                                egui_plot::Line::new("Z", to_points(&gd.pos_z))
                                    .color(egui::Color32::from_rgb(100, 100, 255)),
                            );
                        });

                    // Velocity
                    ui.label(egui::RichText::new("Velocity (m/s)").strong());
                    egui_plot::Plot::new("result_vel_plot")
                        .height(150.0)
                        .x_axis_label("Time (ms)")
                        .legend(egui_plot::Legend::default())
                        .show(ui, |plot_ui| {
                            plot_ui.line(
                                egui_plot::Line::new("X", to_points(&gd.vel_x))
                                    .color(egui::Color32::from_rgb(255, 100, 100)),
                            );
                            plot_ui.line(
                                egui_plot::Line::new("Y", to_points(&gd.vel_y))
                                    .color(egui::Color32::from_rgb(100, 255, 100)),
                            );
                            plot_ui.line(
                                egui_plot::Line::new("Z", to_points(&gd.vel_z))
                                    .color(egui::Color32::from_rgb(100, 100, 255)),
                            );
                        });

                    // Acceleration
                    ui.label(egui::RichText::new("Acceleration (m/s²)").strong());
                    egui_plot::Plot::new("result_acc_plot")
                        .height(150.0)
                        .x_axis_label("Time (ms)")
                        .legend(egui_plot::Legend::default())
                        .show(ui, |plot_ui| {
                            plot_ui.line(
                                egui_plot::Line::new("X", to_points(&gd.acc_x))
                                    .color(egui::Color32::from_rgb(255, 100, 100)),
                            );
                            plot_ui.line(
                                egui_plot::Line::new("Y", to_points(&gd.acc_y))
                                    .color(egui::Color32::from_rgb(100, 255, 100)),
                            );
                            plot_ui.line(
                                egui_plot::Line::new("Z", to_points(&gd.acc_z))
                                    .color(egui::Color32::from_rgb(100, 100, 255)),
                            );
                        });
                }
                }); // ScrollArea
            });

        if !open || close_clicked {
            self.show_sim_result_window = false;
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
        // Auto-disable ground plane if we enabled it
        if self.ground_plane_auto {
            self.show_ground_plane = false;
            self.ground_plane_auto = false;
        }
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

// ===== Sim config TOML save/load =====
//
// Format:
// ```toml
// # Articara Sim Config
// [jump]
// body_link = "trunk"
// ground_links = ["RL_foot", "FL_foot", "RR_foot", "FR_foot"]
// launch_axes = [false, false, true]
// speed = 1.0
//
// [jump.locked_joints]
// RL_hip_joint = true
// FL_hip_joint = true
//
// [payload]
// ee_link = "arm"
// ```

use std::io::{BufRead, Write};
use std::path::Path;

/// Intermediate struct holding all sim config values.
pub(super) struct SimConfig {
    pub body_link: Option<String>,
    pub ground_links: Vec<String>,
    pub launch_axes: [bool; 3],
    pub speed: f32,
    pub locked_joints: std::collections::HashSet<String>,
    pub ee_link: Option<String>,
    pub extension_duration: Option<f32>,
    pub enforce_torque_limits: bool,
    pub enable_retract: bool,
    /// Joint positions that define the starting (crouched) pose.
    pub start_pose: Vec<(String, f32)>,
}

/// Save the current simulation configuration to a TOML file.
pub(super) fn save_sim_config(app: &ArticaraApp, path: &Path) -> Result<(), String> {
    if let Some(parent) = path.parent() {
        if !parent.exists() {
            std::fs::create_dir_all(parent).map_err(|e| format!("{e}"))?;
        }
    }
    let mut f = std::fs::File::create(path).map_err(|e| format!("{e}"))?;

    writeln!(f, "# Articara Sim Config").map_err(|e| format!("{e}"))?;
    writeln!(f).map_err(|e| format!("{e}"))?;

    writeln!(f, "[jump]").map_err(|e| format!("{e}"))?;
    if let Some(ref bl) = app.dynamics_body_link {
        writeln!(f, "body_link = \"{}\"", bl).map_err(|e| format!("{e}"))?;
    }
    // ground_links as TOML array
    let gl_str: Vec<String> = app
        .dynamics_ground_links
        .iter()
        .map(|s| format!("\"{}\"", s))
        .collect();
    writeln!(f, "ground_links = [{}]", gl_str.join(", ")).map_err(|e| format!("{e}"))?;

    let ax = app.dynamics_launch_axes;
    writeln!(f, "launch_axes = [{}, {}, {}]", ax[0], ax[1], ax[2])
        .map_err(|e| format!("{e}"))?;
    writeln!(f, "speed = {}", app.dynamics_sim_speed).map_err(|e| format!("{e}"))?;

    if let Some(dur) = app.dynamics_extension_duration {
        writeln!(f, "extension_duration = {}", dur).map_err(|e| format!("{e}"))?;
    }

    if app.dynamics_enforce_torque_limits {
        writeln!(f, "enforce_torque_limits = true").map_err(|e| format!("{e}"))?;
    }

    if app.dynamics_enable_retract {
        writeln!(f, "enable_retract = true").map_err(|e| format!("{e}"))?;
    }

    if !app.dynamics_locked_joints.is_empty() {
        writeln!(f).map_err(|e| format!("{e}"))?;
        writeln!(f, "[jump.locked_joints]").map_err(|e| format!("{e}"))?;
        let mut sorted: Vec<&String> = app.dynamics_locked_joints.iter().collect();
        sorted.sort();
        for name in sorted {
            let key = toml_key(name);
            writeln!(f, "{} = true", key).map_err(|e| format!("{e}"))?;
        }
    }

    if app.dynamics_ee_link.is_some() {
        writeln!(f).map_err(|e| format!("{e}"))?;
        writeln!(f, "[payload]").map_err(|e| format!("{e}"))?;
        if let Some(ref ee) = app.dynamics_ee_link {
            writeln!(f, "ee_link = \"{}\"", ee).map_err(|e| format!("{e}"))?;
        }
    }

    // Save current joint positions as the starting pose
    if let Some(ref model) = app.model {
        writeln!(f).map_err(|e| format!("{e}"))?;
        writeln!(f, "[jump.start_pose]").map_err(|e| format!("{e}"))?;
        for (ji, joint) in model.joints.iter().enumerate() {
            if joint.joint_type == "fixed" {
                continue;
            }
            let angle = model.joint_positions[ji];
            let key = toml_key(&joint.name);
            writeln!(f, "{} = {}", key, angle).map_err(|e| format!("{e}"))?;
        }
    }

    Ok(())
}

/// Load simulation configuration from a TOML file.
pub(super) fn load_sim_config(path: &Path) -> Result<SimConfig, String> {
    let file = std::fs::File::open(path).map_err(|e| format!("{e}"))?;
    let reader = std::io::BufReader::new(file);

    let mut cfg = SimConfig {
        body_link: None,
        ground_links: Vec::new(),
        launch_axes: [false, false, true],
        speed: 1.0,
        locked_joints: std::collections::HashSet::new(),
        ee_link: None,
        extension_duration: None,
        enforce_torque_limits: false,
        enable_retract: false,
        start_pose: Vec::new(),
    };

    let mut section = SimSection::None;

    for line in reader.lines() {
        let line = line.map_err(|e| format!("{e}"))?;
        let line = line.trim();
        if line.is_empty() || line.starts_with('#') {
            continue;
        }

        if line == "[jump]" {
            section = SimSection::Jump;
            continue;
        }
        if line == "[jump.locked_joints]" {
            section = SimSection::LockedJoints;
            continue;
        }
        if line == "[jump.start_pose]" {
            section = SimSection::StartPose;
            continue;
        }
        if line == "[payload]" {
            section = SimSection::Payload;
            continue;
        }
        if line.starts_with('[') {
            section = SimSection::Unknown;
            continue;
        }

        if let Some((key, value)) = parse_kv(line) {
            match section {
                SimSection::Jump => match key {
                    "body_link" => {
                        cfg.body_link = Some(strip_quotes(value).to_string());
                    }
                    "ground_links" => {
                        cfg.ground_links = parse_string_array(value);
                    }
                    "launch_axes" => {
                        if let Some(bools) = parse_bool_array(value) {
                            if bools.len() == 3 {
                                cfg.launch_axes = [bools[0], bools[1], bools[2]];
                            }
                        }
                    }
                    "speed" => {
                        if let Ok(v) = value.parse::<f32>() {
                            cfg.speed = v;
                        }
                    }
                    "extension_duration" => {
                        if let Ok(v) = value.parse::<f32>() {
                            cfg.extension_duration = Some(v);
                        }
                    }
                    "enforce_torque_limits" => {
                        cfg.enforce_torque_limits =
                            value == "true" || value == "1";
                    }
                    "enable_retract" => {
                        cfg.enable_retract =
                            value == "true" || value == "1";
                    }
                    _ => {}
                },
                SimSection::LockedJoints => {
                    if strip_quotes(value) == "true" {
                        cfg.locked_joints.insert(key.to_string());
                    }
                }
                SimSection::StartPose => {
                    if let Ok(v) = value.parse::<f32>() {
                        cfg.start_pose.push((key.to_string(), v));
                    }
                }
                SimSection::Payload => {
                    if key == "ee_link" {
                        cfg.ee_link = Some(strip_quotes(value).to_string());
                    }
                }
                _ => {}
            }
        }
    }

    Ok(cfg)
}

/// Apply a loaded sim config to the app state.
pub(super) fn apply_sim_config(app: &mut ArticaraApp, cfg: SimConfig) {
    app.dynamics_body_link = cfg.body_link;
    app.dynamics_ground_links = cfg.ground_links;
    app.dynamics_launch_axes = cfg.launch_axes;
    app.dynamics_sim_speed = cfg.speed;
    app.dynamics_locked_joints = cfg.locked_joints;
    app.dynamics_ee_link = cfg.ee_link;
    app.dynamics_extension_duration = cfg.extension_duration;
    app.dynamics_enforce_torque_limits = cfg.enforce_torque_limits;
    app.dynamics_enable_retract = cfg.enable_retract;

    // Apply saved joint positions (start pose) to the model
    if !cfg.start_pose.is_empty() {
        if let Some(ref mut model) = app.model {
            for (name, angle) in &cfg.start_pose {
                if let Some(ji) = model.joints.iter().position(|j| j.name == *name) {
                    model.joint_positions[ji] = *angle;
                }
            }
        }
    }
}

impl ArticaraApp {
    /// Dynamics graph window — now integrated into the result window.
    /// Kept as a no-op for API compatibility.
    pub(super) fn draw_dynamics_graph_window(&mut self, _ctx: &egui::Context) {}
}

// ───────── TOML helpers ─────────

#[derive(Clone, Copy, PartialEq)]
enum SimSection {
    None,
    Jump,
    LockedJoints,
    StartPose,
    Payload,
    Unknown,
}

fn toml_key(name: &str) -> String {
    let bare_ok = !name.is_empty()
        && name
            .chars()
            .all(|c| c.is_ascii_alphanumeric() || c == '_' || c == '-');
    if bare_ok {
        name.to_string()
    } else {
        format!("\"{}\"", name.replace('\\', "\\\\").replace('"', "\\\""))
    }
}

fn parse_kv(line: &str) -> Option<(&str, &str)> {
    let eq = line.find('=')?;
    let key = line[..eq].trim();
    let value = line[eq + 1..].trim();
    let key = key
        .strip_prefix('"')
        .and_then(|k| k.strip_suffix('"'))
        .unwrap_or(key);
    Some((key, value))
}

fn strip_quotes(s: &str) -> &str {
    s.trim()
        .strip_prefix('"')
        .and_then(|s| s.strip_suffix('"'))
        .unwrap_or(s.trim())
}

/// Parse `["a", "b", "c"]` into Vec<String>.
fn parse_string_array(s: &str) -> Vec<String> {
    let s = s.trim();
    let inner = match s.strip_prefix('[').and_then(|s| s.strip_suffix(']')) {
        Some(i) => i,
        None => return Vec::new(),
    };
    inner
        .split(',')
        .map(|p| strip_quotes(p).to_string())
        .filter(|s| !s.is_empty())
        .collect()
}

/// Parse `[true, false, true]` into Vec<bool>.
fn parse_bool_array(s: &str) -> Option<Vec<bool>> {
    let s = s.trim();
    let inner = s.strip_prefix('[')?.strip_suffix(']')?;
    let vals: Result<Vec<bool>, _> = inner.split(',').map(|p| p.trim().parse::<bool>()).collect();
    vals.ok()
}
