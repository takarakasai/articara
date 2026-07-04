#[cfg(feature = "mujoco")]
impl ArticaraApp {
    /// MuJoCo用のbase_posとground_plane設定をUI状態から取得
    pub fn collect_mujoco_setup(&self) -> (Option<[f64; 3]>, Option<articara::mjcf::GroundPlaneCfg>) {
        let base_pos = if self.sim.mujoco_base_pos.iter().any(|&v| v != 0.0) {
            Some([
                self.sim.mujoco_base_pos[0] as f64,
                self.sim.mujoco_base_pos[1] as f64,
                self.sim.mujoco_base_pos[2] as f64,
            ])
        } else {
            None
        };
        let ground = if self.view.show_ground_plane {
            Some(articara::mjcf::GroundPlaneCfg {
                z: self.view.ground_z as f64,
                half_size: self.view.ground_size as f64,
                roll: self.view.ground_plane_roll as f64,
                pitch: self.view.ground_plane_pitch as f64,
            })
        } else {
            None
        };
        (base_pos, ground)
    }
}
use eframe::egui;

use super::ArticaraApp;
use articara::dynamics::{self, StaticAnalysis, DynSim, PayloadPhase};

/// Simulation-execution state owned by the dynamics panel: the active
/// payload / MuJoCo sims, sim-time estimators, WBC pipeline, external
/// force + drag interaction and the per-run toggles. Grouped so
/// [`ArticaraApp`] carries one `sim` field instead of ~26 loose ones,
/// and the `mujoco` feature gates stay contained here.
pub(super) struct SimState {
    /// Active dynamics simulation (payload).
    pub dynamics_sim: Option<DynSim>,
    /// Simulation playback speed.
    pub dynamics_sim_speed: f32,
    /// Whether the simulation is paused.
    pub dynamics_sim_paused: bool,
    /// When `Some(n)`, advance the active MuJoCo sim by exactly `n` physics
    /// frames (negative = step backward through the snapshot history) then
    /// re-pause. Ignored when the active sim is not MuJoCo.
    #[cfg(feature = "mujoco")]
    pub dynamics_step_frames: Option<i32>,
    /// Last frame instant for delta-time calculation.
    pub dynamics_last_instant: Option<std::time::Instant>,
    /// Active MuJoCo simulation instance.
    #[cfg(feature = "mujoco")]
    pub mujoco_sim: Option<articara::mujoco_sim::MujocoSim>,
    /// Madgwick attitude estimators keyed by IMU sensor name. Built on
    /// MuJoCo sim start (one per `[[sensor]]` of kind `Imu` in the
    /// loaded `RobotModel`); updated every physics tick from
    /// `MujocoSim::imu_readings()`. The gait controller's pose-source
    /// selector reads the primary IMU's quaternion to drive the MPC's
    /// `body_state.world_yaw` in `PoseSource::ImuFusion` mode.
    #[cfg(feature = "mujoco")]
    pub imu_estimators:
        std::collections::HashMap<String, articara::attitude_estimator::MadgwickAhrs>,
    /// Last sim time we fed an IMU sample, per sensor name. Used to
    /// derive `dt` for the estimator without re-querying MuJoCo's clock
    /// state (the estimator is sim-time-driven, not wall-clock).
    #[cfg(feature = "mujoco")]
    pub imu_last_sim_time: std::collections::HashMap<String, f64>,
    /// Source for the body pose (yaw + position) that's fed to the gait
    /// controller's MPC each tick. Switchable from the gait panel so
    /// the user can A/B compare the IMU-fusion path against MuJoCo's
    /// oracle while debugging the controller.
    #[cfg(feature = "mujoco")]
    pub pose_source: articara::gait::PoseSource,
    /// Kinematics-based leg-odometry estimator. Maintains an integrated
    /// world-frame body position from stance-foot kinematics, used
    /// when [`Self::pose_source`] is [`articara::gait::PoseSource::LegOdometry`].
    /// Reset on MuJoCo sim start so a previous run's drift doesn't
    /// bleed in.
    #[cfg(feature = "mujoco")]
    pub leg_odometry: articara::leg_odometry::LegOdometry,
    /// Stance flags `[FL, FR, RL, RR]` from the gait controller's
    /// previous tick output. The leg-odometry estimator runs *before*
    /// `gc.tick`, so it relies on this last-tick snapshot to know
    /// which legs are pinned to the ground. One-tick lag is harmless
    /// at typical 2 ms physics ticks.
    #[cfg(feature = "mujoco")]
    pub leg_odometry_last_stance: [bool; 4],
    /// When true, the gait integration loop runs the Hierarchical WBC
    /// (`wbc_pipeline::WbcPipeline`) and writes its solved torques
    /// directly to `MujocoSim` via `set_wbc_torques`, bypassing
    /// per-joint Position-PD. Off by default; toggled from the gait
    /// panel. Active only in MPC gait mode (CHAMP doesn't produce
    /// the GRF / contact references the WBC needs).
    #[cfg(feature = "mujoco")]
    pub wbc_enabled: bool,
    /// Lazy-initialised WBC pipeline. Built on the first tick the
    /// gait controller is enabled in MPC mode so we don't pay the
    /// kinematic-cache lookup cost on robots that aren't quadrupeds.
    #[cfg(feature = "mujoco")]
    pub wbc_pipeline: Option<articara::wbc_pipeline::WbcPipeline>,
    /// When true, the MuJoCo sim auto-lifts the floating base just above z=0.
    /// When false, [`Self::mujoco_base_pos`] is used as the initial world position.
    #[cfg(feature = "mujoco")]
    pub mujoco_auto_base: bool,
    /// Manual initial world position for the floating base (used when
    /// [`Self::mujoco_auto_base`] is false).
    #[cfg(feature = "mujoco")]
    pub mujoco_base_pos: [f32; 3],
    /// Per-axis lock state for the trunk before MuJoCo sim start, ordered
    /// `[TX, TY, TZ, RX, RY, RZ]`. `true` = locked. All `false` = full
    /// floating base (default), all `true` = welded to world.
    #[cfg(feature = "mujoco")]
    pub mujoco_base_locked: [bool; 6],
    /// Currently selected target link for the external-force panel.
    #[cfg(feature = "mujoco")]
    pub ext_force_link: Option<String>,
    /// Force vector (N) for the external-force panel.
    #[cfg(feature = "mujoco")]
    pub ext_force_value: [f32; 3],
    /// Torque vector (N·m) for the external-force panel.
    #[cfg(feature = "mujoco")]
    pub ext_torque_value: [f32; 3],
    /// Duration (s) of the next external-force application.
    #[cfg(feature = "mujoco")]
    pub ext_force_duration: f32,
    /// Whether contact-point markers + force vectors are drawn over the viewport.
    #[cfg(feature = "mujoco")]
    pub show_contacts: bool,
    /// How a sim-time link drag is interpreted (force vs posture).
    #[cfg(feature = "mujoco")]
    pub sim_drag_mode: super::SimDragMode,
    /// Active sim-drag state while the user is holding the mouse button.
    #[cfg(feature = "mujoco")]
    pub sim_drag_state: Option<super::SimDragState>,
    /// Force gain (N per metre of drag) for Force mode. Tuned so a typical
    /// 30 cm drag exerts ~150 N out of the box, enough to push a kg-scale
    /// link around without flinging lighter ones.
    #[cfg(feature = "mujoco")]
    pub sim_drag_force_gain: f32,
    /// Whether to enforce per-joint torque/velocity limits during MuJoCo
    /// simulation. When `true`, the controller torque is clamped to ±τmax
    /// and the velocity-mode reference / commanded torque are gated by ωmax.
    #[cfg(feature = "mujoco")]
    pub enforce_actuator_limits: bool,
    /// Default sliding-friction coefficient (μ_slide) applied to every
    /// emitted MJCF geom — see [`articara::mjcf::MjcfExportOptions::default_friction`].
    /// Surfaced as a Sim-toggles slider so the user can sweep
    /// foot-on-ground μ without editing the misa. Baked into MJCF at
    /// MuJoCo init, so changes require Stop → Play to take effect.
    #[cfg(feature = "mujoco")]
    pub sim_default_friction: f64,
    /// Whether to enable gravity-compensation feedforward in the MuJoCo
    /// controller. The flag mirrors `MujocoSim::set_gravity_compensation`
    /// so the toggle survives Stop → Play (we re-apply it on `mj_start`).
    #[cfg(feature = "mujoco")]
    pub enforce_gravity_compensation: bool,
}

impl Default for SimState {
    fn default() -> Self {
        Self {
            dynamics_sim: None,
            dynamics_sim_speed: 1.0,
            dynamics_sim_paused: false,
            #[cfg(feature = "mujoco")]
            dynamics_step_frames: None,
            dynamics_last_instant: None,
            #[cfg(feature = "mujoco")]
            mujoco_sim: None,
            #[cfg(feature = "mujoco")]
            imu_estimators: std::collections::HashMap::new(),
            #[cfg(feature = "mujoco")]
            imu_last_sim_time: std::collections::HashMap::new(),
            #[cfg(feature = "mujoco")]
            pose_source: articara::gait::PoseSource::default(),
            #[cfg(feature = "mujoco")]
            leg_odometry: articara::leg_odometry::LegOdometry::new(),
            #[cfg(feature = "mujoco")]
            leg_odometry_last_stance: [true; 4], // start in all-stance pose
            #[cfg(feature = "mujoco")]
            wbc_enabled: false,
            #[cfg(feature = "mujoco")]
            wbc_pipeline: None,
            #[cfg(feature = "mujoco")]
            mujoco_auto_base: true,
            #[cfg(feature = "mujoco")]
            mujoco_base_pos: [0.0, 0.0, 0.0],
            #[cfg(feature = "mujoco")]
            mujoco_base_locked: [false; 6],
            #[cfg(feature = "mujoco")]
            ext_force_link: None,
            #[cfg(feature = "mujoco")]
            ext_force_value: [0.0, 0.0, 0.0],
            #[cfg(feature = "mujoco")]
            ext_torque_value: [0.0, 0.0, 0.0],
            #[cfg(feature = "mujoco")]
            ext_force_duration: 0.5,
            #[cfg(feature = "mujoco")]
            show_contacts: true,
            #[cfg(feature = "mujoco")]
            sim_drag_mode: super::SimDragMode::Force,
            #[cfg(feature = "mujoco")]
            sim_drag_state: None,
            #[cfg(feature = "mujoco")]
            sim_drag_force_gain: 500.0,
            #[cfg(feature = "mujoco")]
            enforce_actuator_limits: false,
            // Matches `MjcfExportOptions::default()` so the value the user
            // sees on the slider equals what gets baked into MJCF if they
            // never touch it.
            #[cfg(feature = "mujoco")]
            sim_default_friction: 0.7,
            #[cfg(feature = "mujoco")]
            enforce_gravity_compensation: false,
        }
    }
}


impl ArticaraApp {
    pub fn draw_dynamics_panel(&mut self, ui: &mut egui::Ui) {
        egui::CollapsingHeader::new("⚡ Dynamics Analysis")
            .default_open(false)
            .show(ui, |ui| {
                if self.model.is_none() {
                    ui.label("(no model loaded)");
                    return;
                }

                // EE link selector lives in the "Payload simulation" group
                // below — it only applies to Play Payload (and is an
                // optional hint to Analyze). Play MuJoCo doesn't need it.

                // --- Speed slider ---
                ui.horizontal(|ui| {
                    ui.label("Speed:");
                    ui.add(
                        egui::Slider::new(&mut self.sim.dynamics_sim_speed, 0.1..=5.0)
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
                        self.dialogs.open_sim_config.open(
                            "Load Sim Config",
                            super::file_dialog::FileDialogMode::Open,
                            start.as_deref(),
                            &["toml"],
                        );
                    }
                });

                ui.separator();

                // --- Simulation controls ---
                let sim_active = self.sim.dynamics_sim.is_some();
                #[cfg(feature = "mujoco")]
                let sim_active = sim_active || self.sim.mujoco_sim.is_some();

                ui.horizontal(|ui| {
                    // Static analysis — uses EE link if one is set, otherwise
                    // reports whole-model torques.
                    if ui
                        .add_enabled(!sim_active, egui::Button::new("📊 Analyze"))
                        .on_hover_text(
                            "Static torque analysis at current pose. EE link is \
                             optional — open the Payload simulation section below \
                             to set one and include payload-side results.",
                        )
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
                            let bake = self.sim.enforce_actuator_limits;
                            let opts = articara::mjcf::MjcfExportOptions {
                                base_pos,
                                ground_plane: ground,
                                add_actuators: false,
                                base_locked_axes: self.sim.mujoco_base_locked,
                                bake_actuator_limits: bake,
                                bake_joint_position_limits: bake,
                                mesh_path_style:
                                    articara::mesh_paths::MeshPathStyle::default(),
                                default_friction: [
                                    self.sim.sim_default_friction,
                                    0.005,
                                    0.0001,
                                ],
                            };
                            match articara::mujoco_sim::MujocoSim::new(model, opts) {
                                Ok(mut sim) => {
                                    // Carry the user's grav-comp toggle into
                                    // the freshly-built sim so Stop → Play
                                    // doesn't silently reset it to off.
                                    sim.set_gravity_compensation(
                                        self.sim.enforce_gravity_compensation,
                                    );
                                    self.sim.mujoco_sim = Some(sim);
                                    // Build a fresh Madgwick estimator per IMU
                                    // for this run so a previous Play→Stop
                                    // cycle's quaternion doesn't bleed in.
                                    self.rebuild_imu_estimators();
                                    // Reset the leg-odometry estimator and
                                    // seed its position with the body's
                                    // current world position so the integrated
                                    // estimate doesn't start from origin.
                                    self.sim.leg_odometry.reset();
                                    if let (Some(ref mj_sim), Some(ref m)) =
                                        (self.sim.mujoco_sim.as_ref(), self.model.as_ref())
                                    {
                                        if let Some(p) =
                                            mj_sim.body_world_position(&m.root_link)
                                        {
                                            self.sim.leg_odometry.set_position(
                                                nalgebra::Vector3::new(p[0], p[1], p[2]),
                                            );
                                        }
                                    }
                                    self.sim.leg_odometry_last_stance = [true; 4];
                                    // Start paused so the user can choose between
                                    // frame stepping or ▶ Play before any time
                                    // advances.
                                    self.sim.dynamics_sim_paused = true;
                                    self.status_message =
                                        "MuJoCo paused at t=0 — press ▶ Play or ⏩ +N to advance".into();
                                }
                                Err(e) => self.status_message = format!("MuJoCo init error: {e}"),
                            }
                        }
                    }
                });

                // --- Payload simulation (manipulator-style EE loading) ---
                // Niche feature: gradually loads a chosen end-effector link to
                // visualise joint-torque saturation. Useless for legged
                // robots (gait sim), so wrap it in a collapsing header that
                // stays closed by default — keeps it discoverable without
                // cluttering the main controls or surfacing "0 capacity"
                // errors during normal walk workflows.
                ui.add_space(4.0);
                egui::CollapsingHeader::new("🏋 Payload simulation (manipulator)")
                    .default_open(false)
                    .id_salt("dyn_payload_sim")
                    .show(ui, |ui| {
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
                                        .selectable_label(
                                            self.dynamics_ee_link.is_none(),
                                            "(none)",
                                        )
                                        .clicked()
                                    {
                                        self.dynamics_ee_link = None;
                                    }
                                    for name in &link_names {
                                        let sel = self.dynamics_ee_link.as_deref()
                                            == Some(name.as_str());
                                        if ui.selectable_label(sel, name).clicked() {
                                            self.dynamics_ee_link = Some(name.clone());
                                        }
                                    }
                                });
                        });

                        let can_payload =
                            !sim_active && self.dynamics_ee_link.is_some();
                        let payload_hover: &str = if sim_active {
                            "⚠ Another simulation is already running — stop it first."
                        } else if self.dynamics_ee_link.is_none() {
                            "⚠ Pick an EE link above first (the payload is applied to that link)."
                        } else {
                            "Gradually load the end-effector and visualise joint torque utilisation"
                        };
                        if ui
                            .add_enabled(
                                can_payload,
                                egui::Button::new("🏋 Play Payload"),
                            )
                            .on_hover_text(payload_hover)
                            .clicked()
                        {
                            if let Some(ref model) = self.model {
                                let ee = self
                                    .dynamics_ee_link
                                    .as_deref()
                                    .unwrap_or("");
                                if let Some(sim) = dynamics::start_payload_sim(
                                    model,
                                    ee,
                                    self.sim.dynamics_sim_speed as f64,
                                ) {
                                    self.sim.dynamics_sim =
                                        Some(DynSim::Payload(sim));
                                } else {
                                    self.status_message =
                                        "Cannot start payload sim (no effort limits or 0 capacity?)"
                                            .into();
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
                        ui.checkbox(&mut self.sim.mujoco_auto_base, "Auto")
                            .on_hover_text(
                                "When checked, the root link is auto-lifted just \
                                 above the ground plane. When unchecked, the values \
                                 below are used as the floating-base initial \
                                 world-frame position.",
                            );
                    });
                    ui.add_enabled_ui(!self.sim.mujoco_auto_base, |ui| {
                        ui.horizontal(|ui| {
                            ui.label("X:");
                            ui.add(
                                egui::DragValue::new(&mut self.sim.mujoco_base_pos[0])
                                    .speed(0.01)
                                    .fixed_decimals(3)
                                    .suffix(" m"),
                            );
                            ui.label("Y:");
                            ui.add(
                                egui::DragValue::new(&mut self.sim.mujoco_base_pos[1])
                                    .speed(0.01)
                                    .fixed_decimals(3)
                                    .suffix(" m"),
                            );
                            ui.label("Z:");
                            ui.add(
                                egui::DragValue::new(&mut self.sim.mujoco_base_pos[2])
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
                        ui.checkbox(&mut self.sim.mujoco_base_locked[0], "TX");
                        ui.checkbox(&mut self.sim.mujoco_base_locked[1], "TY");
                        ui.checkbox(&mut self.sim.mujoco_base_locked[2], "TZ");
                    })
                    .response
                    .on_hover_text("Lock translation along world X / Y / Z");
                    ui.horizontal(|ui| {
                        ui.checkbox(&mut self.sim.mujoco_base_locked[3], "RX");
                        ui.checkbox(&mut self.sim.mujoco_base_locked[4], "RY");
                        ui.checkbox(&mut self.sim.mujoco_base_locked[5], "RZ");
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
                        ui.checkbox(&mut self.sim.show_contacts, "👣 Contacts")
                            .on_hover_text(
                                "Draw contact points and contact-force \
                                 vectors over the viewport.",
                            );
                        ui.checkbox(
                            &mut self.sim.enforce_actuator_limits,
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
                                &mut self.sim.enforce_gravity_compensation,
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
                            if let Some(sim) = self.sim.mujoco_sim.as_mut() {
                                sim.set_gravity_compensation(
                                    self.sim.enforce_gravity_compensation,
                                );
                            }
                        }
                    });
                    // Default sliding-friction coefficient baked into MJCF
                    // at MuJoCo init. Changes only take effect at the next
                    // Stop → Play (MuJoCo compiles the value in once).
                    ui.horizontal(|ui| {
                        ui.label("🪨 Friction μ:");
                        let resp = ui.add(
                            egui::Slider::new(
                                &mut self.sim.sim_default_friction,
                                0.0..=2.0,
                            )
                            .fixed_decimals(2),
                        )
                        .on_hover_text(
                            "Default sliding-friction coefficient applied \
                             to every emitted MJCF geom (ground plane, foot \
                             collisions, link colliders). MuJoCo combines \
                             contact pairs via per-axis max, so foot-on-ground \
                             at this μ from both sides gives a contact μ \
                             equal to the slider. Bake-time setting — changes \
                             require Stop → Play to take effect on a \
                             running sim.",
                        );
                        if ui.small_button("0.7").on_hover_text("Reset to default (0.7)").clicked() {
                            self.sim.sim_default_friction = 0.7;
                        }
                        if sim_active && resp.changed() {
                            self.status_message = format!(
                                "🪨 Friction μ = {:.2} — Stop → Play to apply",
                                self.sim.sim_default_friction,
                            );
                        }
                    });
                    // Mouse-drag interaction during sim:
                    // pick Force (apply wrench) vs Posture (IK target) and
                    // tune the force gain for Force mode.
                    ui.horizontal(|ui| {
                        ui.label("🖱 Drag:");
                        let mut mode = self.sim.sim_drag_mode;
                        egui::ComboBox::from_id_salt("sim_drag_mode")
                            .selected_text(mode.label())
                            .show_ui(ui, |ui| {
                                for m in super::SimDragMode::ALL {
                                    ui.selectable_value(&mut mode, m, m.label());
                                }
                            });
                        if mode != self.sim.sim_drag_mode {
                            self.sim.sim_drag_mode = mode;
                        }
                    });
                    if matches!(self.sim.sim_drag_mode, super::SimDragMode::Force) {
                        ui.horizontal(|ui| {
                            ui.label("    Force gain:");
                            ui.add(
                                egui::DragValue::new(&mut self.sim.sim_drag_force_gain)
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
                            self.sim.dynamics_sim_paused = false;
                            self.stop_dynamics_sim();
                        }
                        if self.sim.dynamics_sim_paused {
                            if ui.button("▶ Play").clicked() {
                                self.sim.dynamics_sim_paused = false;
                            }
                        } else {
                            if ui.button("⏸ Pause").clicked() {
                                self.sim.dynamics_sim_paused = true;
                            }
                        }
                    });
                    // Frame stepper — MuJoCo only (payload sim has no frame
                    // history). Visible whenever MuJoCo is running; clicking a
                    // step button while playing also pauses the sim.
                    #[cfg(feature = "mujoco")]
                    if self.sim.mujoco_sim.is_some() {
                        let history_len = self
                            .sim.mujoco_sim
                            .as_ref()
                            .map(|s| s.history_len())
                            .unwrap_or(0);
                        let mj_dt_ms = self
                            .sim.mujoco_sim
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
                                    self.sim.dynamics_sim_paused = true;
                                    self.sim.dynamics_step_frames = Some(-(n as i32));
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
                                    self.sim.dynamics_sim_paused = true;
                                    self.sim.dynamics_step_frames = Some(n as i32);
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
        match &self.sim.dynamics_sim {
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
        if let Some(sim) = self.sim.dynamics_sim.take() {
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
        if let Some(mj_sim) = self.sim.mujoco_sim.take() {
            if let Some(ref mut model) = self.model {
                mj_sim.restore(model);
            }
        }
        #[cfg(not(feature = "mujoco"))]
        let _ = ();
        self.sim.dynamics_last_instant = None;
        // Auto-disable ground plane if we enabled it
        if self.view.ground_plane_auto {
            self.view.show_ground_plane = false;
            self.view.ground_plane_auto = false;
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
    writeln!(f, "speed = {}", app.sim.dynamics_sim_speed).map_err(|e| format!("{e}"))?;
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
    app.sim.dynamics_sim_speed = cfg.speed;
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

