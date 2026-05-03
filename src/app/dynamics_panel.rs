#[cfg(feature = "mujoco")]
impl ArticaraApp {
    /// MuJoCo用のbase_posとground_plane設定をUI状態から取得
    pub fn collect_mujoco_setup(&self) -> (Option<[f64; 3]>, Option<crate::mjcf::GroundPlaneCfg>) {
        let base_pos = if self.mujoco_base_pos.iter().any(|&v| v != 0.0) {
            Some([
                self.mujoco_base_pos[0] as f64,
                self.mujoco_base_pos[1] as f64,
                self.mujoco_base_pos[2] as f64,
            ])
        } else {
            None
        };
        let ground = if self.show_ground_plane {
            Some(crate::mjcf::GroundPlaneCfg {
                z: self.ground_z as f64,
                half_size: self.ground_size as f64,
                roll: self.ground_plane_roll as f64,
                pitch: self.ground_plane_pitch as f64,
            })
        } else {
            None
        };
        (base_pos, ground)
    }
}
use eframe::egui;

use super::ArticaraApp;
use crate::dynamics::{self, StaticAnalysis, DynSim, PayloadPhase};

impl ArticaraApp {
    pub fn draw_dynamics_panel(&mut self, ui: &mut egui::Ui) {
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

                // --- Speed slider ---
                ui.horizontal(|ui| {
                    ui.label("Speed:");
                    ui.add(
                        egui::Slider::new(&mut self.dynamics_sim_speed, 0.1..=5.0)
                            .logarithmic(true)
                            .text("×"),
                    );
                });

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

                // --- Simulation controls ---
                let mut sim_active = self.dynamics_sim.is_some();
                #[cfg(feature = "mujoco")]
                {
                    sim_active = sim_active || self.mujoco_sim.is_some();
                }

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
                            );
                            self.dynamics_result = Some(result);
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
                                self.dynamics_sim_speed as f64,
                            ) {
                                self.dynamics_sim = Some(DynSim::Payload(sim));
                            } else {
                                self.status_message =
                                    "Cannot start payload sim (no effort limits or 0 capacity?)"
                                        .into();
                            }
                        }
                    }

                    // MuJoCo シミュレーション
                    #[cfg(feature = "mujoco")]
                    if ui
                        .add_enabled(!sim_active, egui::Button::new("🦾 Play MuJoCo"))
                        .on_hover_text("Start real-time MuJoCo physics simulation")
                        .clicked()
                    {
                        let (base_pos, ground) = self.collect_mujoco_setup();
                        if let Some(ref model) = self.model {
                            // The "⛔ Limits" checkbox now controls BOTH the
                            // runtime controller clamp AND the MuJoCo-level
                            // `forcelimited` / joint `range` attributes —
                            // otherwise the UI would mislead: the host could
                            // command 100 N·m but MuJoCo would silently clip
                            // to ±τmax baked into the MJCF, and the user
                            // would see "no change" when toggling limits.
                            let bake = self.enforce_actuator_limits;
                            let opts = crate::mjcf::MjcfExportOptions {
                                base_pos,
                                ground_plane: ground,
                                add_actuators: false,
                                base_locked_axes: self.mujoco_base_locked,
                                bake_actuator_limits: bake,
                                bake_joint_position_limits: bake,
                                // Live MuJoCo sim consumes the XML via
                                // from_xml_string which has no on-disk
                                // anchor → mesh paths must be absolute.
                                mesh_path_style:
                                    crate::mesh_paths::MeshPathStyle::Absolute,
                            };
                            match crate::mujoco_sim::MujocoSim::new(model, opts) {
                                Ok(mut sim) => {
                                    // Carry the user's grav-comp toggle into
                                    // the freshly-built sim so Stop → Play
                                    // doesn't silently reset it to off.
                                    sim.set_gravity_compensation(
                                        self.enforce_gravity_compensation,
                                    );
                                    self.mujoco_sim = Some(sim);
                                    // Initialise one Madgwick estimator
                                    // per IMU sensor in the model. Old
                                    // state (from a previous Play→Stop
                                    // cycle) is discarded.
                                    self.rebuild_imu_estimators();
                                    // Start paused so the user can choose between
                                    // frame stepping or ▶ Play before any time
                                    // advances.
                                    self.dynamics_sim_paused = true;
                                    self.status_message =
                                        "MuJoCo paused at t=0 — press ▶ Play or ⏩ +N to advance".into();
                                }
                                Err(e) => self.status_message = format!("MuJoCo init error: {e}"),
                            }
                        }
                    }
                });

                // --- MuJoCo floating-base initial position + lock ---
                // Wrapped in a collapsing header so users who only need the
                // playback controls (Play / Pause / step) aren't forced to
                // scroll past 6 axis checkboxes on every glance.
                #[cfg(feature = "mujoco")]
                egui::CollapsingHeader::new("🦿 Base setup")
                    .default_open(false)
                    .id_salt("dyn_base_setup")
                    .show(ui, |ui| {
                    ui.horizontal(|ui| {
                        ui.label("Base pos:");
                        ui.checkbox(&mut self.mujoco_auto_base, "Auto")
                            .on_hover_text(
                                "When checked, the root link is auto-lifted just \
                                 above the ground plane. When unchecked, the values \
                                 below are used as the floating-base initial \
                                 world-frame position.",
                            );
                    });
                    ui.add_enabled_ui(!self.mujoco_auto_base, |ui| {
                        ui.horizontal(|ui| {
                            ui.label("X:");
                            ui.add(
                                egui::DragValue::new(&mut self.mujoco_base_pos[0])
                                    .speed(0.01)
                                    .fixed_decimals(3)
                                    .suffix(" m"),
                            );
                            ui.label("Y:");
                            ui.add(
                                egui::DragValue::new(&mut self.mujoco_base_pos[1])
                                    .speed(0.01)
                                    .fixed_decimals(3)
                                    .suffix(" m"),
                            );
                            ui.label("Z:");
                            ui.add(
                                egui::DragValue::new(&mut self.mujoco_base_pos[2])
                                    .speed(0.01)
                                    .fixed_decimals(3)
                                    .suffix(" m"),
                            );
                        })
                        .response
                        .on_hover_text(
                            "Initial world-frame position of the floating base \
                             link at MuJoCo sim start.",
                        );
                    });

                    // --- Base 6-DoF lock ---
                    // All unchecked = full <freejoint/>; all checked = welded;
                    // mixed = only the unlocked axes get individual joints.
                    ui.label("Base lock:");
                    ui.horizontal(|ui| {
                        ui.checkbox(&mut self.mujoco_base_locked[0], "TX");
                        ui.checkbox(&mut self.mujoco_base_locked[1], "TY");
                        ui.checkbox(&mut self.mujoco_base_locked[2], "TZ");
                    })
                    .response
                    .on_hover_text("Lock translation along world X / Y / Z");
                    ui.horizontal(|ui| {
                        ui.checkbox(&mut self.mujoco_base_locked[3], "RX");
                        ui.checkbox(&mut self.mujoco_base_locked[4], "RY");
                        ui.checkbox(&mut self.mujoco_base_locked[5], "RZ");
                    })
                    .response
                    .on_hover_text("Lock rotation about world X / Y / Z");
                });

                // ── Sim-time visualisation / safety toggles (MuJoCo only) ──
                // Same rationale as base setup — kept tucked away so the
                // primary playback controls dominate the panel by default.
                #[cfg(feature = "mujoco")]
                egui::CollapsingHeader::new("🎛 Sim toggles")
                    .default_open(false)
                    .id_salt("dyn_sim_toggles")
                    .show(ui, |ui| {
                    ui.horizontal(|ui| {
                        ui.checkbox(&mut self.show_contacts, "👣 Contacts")
                            .on_hover_text(
                                "Draw contact points and contact-force \
                                 vectors over the viewport.",
                            );
                        ui.checkbox(
                            &mut self.enforce_actuator_limits,
                            "⛔ Limits",
                        )
                        .on_hover_text(
                            "Enforce per-joint hardware limits at three places:\n\
                             • runtime PD: clamp τ to ±τmax, q̇* to ±ωmax\n\
                             • MJCF <motor>: forcelimited=\"true\"\n\
                             • MJCF <joint>: range=\"lower upper\"\n\
                             The two MJCF flags are baked at sim construction \
                             time, so changing this checkbox while a sim is \
                             active requires Stop → Play to take effect on \
                             MuJoCo's side. Off = unrestricted at all three.",
                        );
                        if ui
                            .checkbox(
                                &mut self.enforce_gravity_compensation,
                                "🌍 Grav comp",
                            )
                            .on_hover_text(
                                "Add a feedforward gravity-compensation \
                                 torque (RNEA) to every Position/Velocity-\
                                 mode joint. With this on, the PD only \
                                 corrects tracking error, not the static \
                                 load — Kp and the resulting deflection \
                                 drop dramatically.",
                            )
                            .changed()
                        {
                            if let Some(sim) = self.mujoco_sim.as_mut() {
                                sim.set_gravity_compensation(
                                    self.enforce_gravity_compensation,
                                );
                            }
                        }
                    });
                    // Mouse-drag interaction during sim:
                    // pick Force (apply wrench) vs Posture (IK target) and
                    // tune the force gain for Force mode.
                    ui.horizontal(|ui| {
                        ui.label("🖱 Drag:");
                        let mut mode = self.sim_drag_mode;
                        egui::ComboBox::from_id_salt("sim_drag_mode")
                            .selected_text(mode.label())
                            .show_ui(ui, |ui| {
                                for m in super::SimDragMode::ALL {
                                    ui.selectable_value(&mut mode, m, m.label());
                                }
                            });
                        if mode != self.sim_drag_mode {
                            self.sim_drag_mode = mode;
                        }
                    });
                    if matches!(self.sim_drag_mode, super::SimDragMode::Force) {
                        ui.horizontal(|ui| {
                            ui.label("    Force gain:");
                            ui.add(
                                egui::DragValue::new(&mut self.sim_drag_force_gain)
                                    .speed(10.0)
                                    .range(1.0..=10000.0)
                                    .fixed_decimals(0)
                                    .suffix(" N/m"),
                            );
                        });
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
                    // Frame stepper — MuJoCo only (payload sim has no frame
                    // history). Visible whenever MuJoCo is running; clicking a
                    // step button while playing also pauses the sim.
                    #[cfg(feature = "mujoco")]
                    if self.mujoco_sim.is_some() {
                        let history_len = self
                            .mujoco_sim
                            .as_ref()
                            .map(|s| s.history_len())
                            .unwrap_or(0);
                        let mj_dt_ms = self
                            .mujoco_sim
                            .as_ref()
                            .map(|s| s.timestep() * 1000.0)
                            .unwrap_or(0.0);

                        ui.label(format!(
                            "Frame step  ({:.1} ms each, {history_len} buffered)",
                            mj_dt_ms,
                        ));
                        // Backward row (disabled when there's no history yet).
                        ui.horizontal(|ui| {
                            let can_back = history_len > 0;
                            for n in [100u32, 10, 1] {
                                if ui
                                    .add_enabled(
                                        can_back,
                                        egui::Button::new(format!("⏪ -{n}")),
                                    )
                                    .on_hover_text(format!(
                                        "Restore state from {n} frame(s) ago \
                                         (auto-pauses if running)",
                                    ))
                                    .clicked()
                                {
                                    self.dynamics_sim_paused = true;
                                    self.dynamics_step_frames = Some(-(n as i32));
                                }
                            }
                        });
                        // Forward row.
                        ui.horizontal(|ui| {
                            for n in [1u32, 10, 100] {
                                if ui
                                    .button(format!("⏩ +{n}"))
                                    .on_hover_text(format!(
                                        "Advance {n} frame(s) then pause",
                                    ))
                                    .clicked()
                                {
                                    self.dynamics_sim_paused = true;
                                    self.dynamics_step_frames = Some(n as i32);
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

    /// Placeholder retained for API compatibility — jump result UI was removed.
    pub fn draw_sim_result_window(&mut self, _ctx: &egui::Context) {}

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
                    DynSim::Payload(ps) => {
                        model.joint_positions = ps.saved_positions;
                        model.base_transform = ps.saved_base_transform;
                    }
                }
            }
        }
        #[cfg(feature = "mujoco")]
        if let Some(mj_sim) = self.mujoco_sim.take() {
            if let Some(ref mut model) = self.model {
                mj_sim.restore(model);
            }
        }
        #[cfg(not(feature = "mujoco"))]
        let _ = ();
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
// [payload]
// ee_link = "arm"
// speed = 1.0
// ```

use std::io::{BufRead, Write};
use std::path::Path;

/// Intermediate struct holding all sim config values.
pub(super) struct SimConfig {
    pub speed: f32,
    pub ee_link: Option<String>,
    /// Joint positions that define the starting pose.
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

    writeln!(f, "[payload]").map_err(|e| format!("{e}"))?;
    writeln!(f, "speed = {}", app.dynamics_sim_speed).map_err(|e| format!("{e}"))?;
    if let Some(ref ee) = app.dynamics_ee_link {
        writeln!(f, "ee_link = \"{}\"", ee).map_err(|e| format!("{e}"))?;
    }

    // Save current joint positions as the starting pose
    if let Some(ref model) = app.model {
        writeln!(f).map_err(|e| format!("{e}"))?;
        writeln!(f, "[start_pose]").map_err(|e| format!("{e}"))?;
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
        speed: 1.0,
        ee_link: None,
        start_pose: Vec::new(),
    };

    let mut section = SimSection::None;

    for line in reader.lines() {
        let line = line.map_err(|e| format!("{e}"))?;
        let line = line.trim();
        if line.is_empty() || line.starts_with('#') {
            continue;
        }

        if line == "[payload]" {
            section = SimSection::Payload;
            continue;
        }
        if line == "[start_pose]" {
            section = SimSection::StartPose;
            continue;
        }
        if line.starts_with('[') {
            section = SimSection::Unknown;
            continue;
        }

        if let Some((key, value)) = parse_kv(line) {
            match section {
                SimSection::Payload => match key {
                    "ee_link" => {
                        cfg.ee_link = Some(strip_quotes(value).to_string());
                    }
                    "speed" => {
                        if let Ok(v) = value.parse::<f32>() {
                            cfg.speed = v;
                        }
                    }
                    _ => {}
                },
                SimSection::StartPose => {
                    if let Ok(v) = value.parse::<f32>() {
                        cfg.start_pose.push((key.to_string(), v));
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
    app.dynamics_sim_speed = cfg.speed;
    app.dynamics_ee_link = cfg.ee_link;

    // Apply saved joint positions (start pose) to the model
    if !cfg.start_pose.is_empty() {
        if let Some(ref mut model) = app.model {
            for (name, angle) in &cfg.start_pose {
                if let Some(ji) = model.joints.iter().position(|j| j.name == *name) {
                    model.joint_positions[ji] = *angle as f64;
                }
            }
        }
    }
}

impl ArticaraApp {
    /// Stub kept for API compatibility — graph window was removed with the
    /// jump simulation.
    pub fn draw_dynamics_graph_window(&mut self, _ctx: &egui::Context) {}
}

// ───────── TOML helpers ─────────

#[derive(Clone, Copy, PartialEq)]
enum SimSection {
    None,
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

