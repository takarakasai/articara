//! UI panel for the quadruped gait controller.
//!
//! Lives under "🐕 Quadruped gait" in the right-side Dynamics column.
//! Drives [`crate::gait::GaitController`] via [`super::ArticaraApp`].

use eframe::egui;

use super::ArticaraApp;
use quadruped_gait::{GaitConfig, GaitType, LegId, VelocityCmd};

impl ArticaraApp {
    pub(super) fn draw_gait_panel(&mut self, ui: &mut egui::Ui) {
        egui::CollapsingHeader::new("🐕 Quadruped gait")
            .default_open(false)
            .show(ui, |ui| {
                if self.model.is_none() {
                    ui.label("(load a robot model first)");
                    return;
                }

                self.draw_gait_setup_section(ui);
                ui.separator();
                self.draw_gait_dpad(ui);
                ui.separator();
                self.draw_gait_runtime_section(ui);
            });
    }

    /// Hold-to-drive D-pad with yaw rotation buttons. Each direction
    /// button accumulates a velocity component while held; on release
    /// every button-driven component returns to zero so the robot stops
    /// the moment the user lets go. The conventional numeric sliders
    /// below remain live for "set a static value" use cases.
    fn draw_gait_dpad(&mut self, ui: &mut egui::Ui) {
        if self.gait_controller.is_none() {
            return;
        }

        ui.label(
            egui::RichText::new("D-pad (hold to drive)")
                .strong()
                .small(),
        );
        ui.label(
            egui::RichText::new(
                "Hold a direction button to drive; release to stop. Diagonal \
                 motion = two adjacent buttons. Yaw uses the right pair.",
            )
            .small()
            .weak(),
        );

        // egui's default `Sense::click()` clears the "down on" flag the
        // moment the cursor drifts even slightly off the button rect,
        // which manifested as the gait stopping after ~1 s of holding
        // when the user moved their hand a hair. `Sense::click_and_drag`
        // adds drag-tracking so the response stays "active" as long as
        // the primary button is down on the widget — ergonomically what
        // the user expects of a hold-to-drive control.
        let mk = |label: &str| {
            egui::Button::new(label).sense(egui::Sense::click_and_drag())
        };
        // A button is "held" if either (a) the press originated on it
        // and the primary mouse button is still down, OR (b) the user
        // is dragging from it (mouse moved while held). Using both
        // tolerates small cursor wobbles during a long press.
        let held = |r: &egui::Response| -> bool {
            r.is_pointer_button_down_on() || r.dragged()
        };

        let speed = self.gait_dpad_speed as f64;
        let yaw_speed = self.gait_dpad_yaw_speed as f64;
        let btn_size = egui::vec2(40.0, 40.0);
        let mut vx = 0.0_f64;
        let mut vy = 0.0_f64;
        let mut wz = 0.0_f64;
        let mut any_held = false;
        let mut march_held = false;
        let mut march_clicked = false;

        // Side-by-side layout: 3×3 translation cross on the left, yaw
        // pair on the right. Wrapping in `ui.horizontal` keeps both
        // groups together visually.
        ui.horizontal(|ui| {
            // ── Translation D-pad ──
            egui::Grid::new("gait_dpad_translation")
                .num_columns(3)
                .min_col_width(0.0)
                .spacing(egui::vec2(2.0, 2.0))
                .show(ui, |ui| {
                    ui.label("");
                    let r_up = ui.add_sized(btn_size, mk("⬆"))
                        .on_hover_text("Forward (+vx)");
                    ui.label("");
                    ui.end_row();

                    let r_left = ui.add_sized(btn_size, mk("⬅"))
                        .on_hover_text("Left (+vy)");
                    // Center cell: march in place. Holding sends a
                    // tiny-but-nonzero cmd so the phase generator keeps
                    // cycling (it freezes at cmd=0) while the Raibert
                    // stride amplitude collapses to ~0 — feet lift in
                    // swing but the body doesn't translate.
                    let r_march = ui.add_sized(btn_size, mk("👣"))
                        .on_hover_text(
                            "Press AND HOLD: march in place (gait active, \
                             no translation). Feet lift through the swing \
                             curve while the body stays put. A quick click \
                             will only flash a tiny cmd for one frame — \
                             the foot lift takes ~200 ms per stride, so \
                             hold the button for at least one full gait \
                             cycle (≈ 0.4 s) to see motion. Direction \
                             buttons override this when both are held.",
                        );
                    let r_right = ui.add_sized(btn_size, mk("➡"))
                        .on_hover_text("Right (−vy)");
                    ui.end_row();

                    ui.label("");
                    let r_down = ui.add_sized(btn_size, mk("⬇"))
                        .on_hover_text("Backward (−vx)");
                    ui.label("");
                    ui.end_row();

                    if held(&r_up)    { vx += speed; any_held = true; }
                    if held(&r_down)  { vx -= speed; any_held = true; }
                    if held(&r_left)  { vy += speed; any_held = true; }
                    if held(&r_right) { vy -= speed; any_held = true; }
                    if held(&r_march) { march_held = true; }
                    if r_march.clicked() { march_clicked = true; }
                });

            ui.add_space(12.0);

            // ── Yaw rotation pair ──
            // ↺ = counter-clockwise viewed from above = +wz.
            // ↻ = clockwise = −wz.
            ui.vertical(|ui| {
                ui.label(egui::RichText::new("Yaw").small().weak());
                let r_yaw_l = ui.add_sized(btn_size, mk("↺"))
                    .on_hover_text("Turn left (+wz)");
                let r_yaw_r = ui.add_sized(btn_size, mk("↻"))
                    .on_hover_text("Turn right (−wz)");
                if held(&r_yaw_l) { wz += yaw_speed; any_held = true; }
                if held(&r_yaw_r) { wz -= yaw_speed; any_held = true; }
            });
        });

        // Speed knobs.
        ui.horizontal(|ui| {
            ui.label("Linear (m/s):");
            ui.add(
                egui::Slider::new(&mut self.gait_dpad_speed, 0.0..=1.0)
                    .fixed_decimals(2),
            );
        });
        ui.horizontal(|ui| {
            ui.label("Yaw (rad/s):");
            ui.add(
                egui::Slider::new(&mut self.gait_dpad_yaw_speed, 0.0..=2.0)
                    .fixed_decimals(2),
            );
        });
        // LinearCrawl-only: choose whether the D-pad operates in
        // vx-primary (default) or stride-primary mode. See the field
        // docstring on `gait_dpad_stride_primary`.
        ui.checkbox(
            &mut self.gait_dpad_stride_primary,
            "Stride-primary D-pad (LinearCrawl)",
        )
        .on_hover_text(
            "When ON: pressing the D-pad rescales cycle_period (= 3/4-leg \
             support times) to keep the foot stride per swing constant at the \
             value of the `Stride length` slider. The displayed t3 / t4 will \
             move as vx changes — that's the math of fixed-stride / fixed-T \
             constraints being mutually exclusive. \n\n\
             When OFF (default): the D-pad only sets vx; t3, t4 and cycle_period \
             stay where you put them, and stride varies with vx. \n\n\
             Affects LinearCrawl mode only.",
        );

        // Kinematic playback toggle — drive the model directly from
        // the planner each frame, bypassing MuJoCo.
        let was_kinematic = self.kinematic_playback_active;
        ui.add_enabled_ui(self.gait_controller.is_some(), |ui| {
            ui.checkbox(
                &mut self.kinematic_playback_active,
                "▶ Kinematic playback (planner only, no physics)",
            )
            .on_hover_text(
                "Drive the model's joint angles and trunk pose directly from the \
                 gait controller's plan each frame, ignoring MuJoCo. Lets you see \
                 the planner's intended motion in isolation — no slip, no PD lag, \
                 no contact dynamics, no trunk sway from inertia. \n\n\
                 When toggled ON, the current trunk pose is snapshotted as the \
                 playback anchor; the gait is reset so the body integrates from \
                 there. Disable to freeze the robot at its current pose; the next \
                 MuJoCo step (if running) will take over from that pose.",
            );
        });
        if self.kinematic_playback_active && !was_kinematic {
            // Just turned ON — snapshot the current base + reset gait
            // so body_state integrates from world origin (= we compose
            // `offset · body_state` to place the robot back where it
            // was before the toggle).
            if let Some(model) = self.model.as_ref() {
                self.kinematic_playback_base_offset = model.base_transform;
            }
            if let Some(gc) = self.gait_controller.as_mut() {
                // `disable()` calls `inner.reset()` which clears phase,
                // body_state and velocity_cmd. We save & restore the
                // cmd so the user's vx survives the toggle.
                let saved_cmd = gc.velocity_cmd();
                gc.disable();
                gc.enable();
                gc.set_velocity_cmd(saved_cmd);
            }
        }

        // Drive the controller. While at least one button is held,
        // command the accumulated (vx, vy, wz). On the first frame
        // after all buttons are released, send a single zero so the
        // robot stops immediately. The slider section below can still
        // be used afterward to set a sticky non-zero command.
        //
        // March-in-place: when 👣 is the only button held, we emit a
        // tiny ε on `vx` (1e-6 m/s) — enough to keep `phase_gen` ticking
        // (it freezes on `cmd.is_zero()`) while the Raibert step amplitude
        // (= vx · T_stance) collapses to ~1e-7 m so the body stays put.
        // Direction buttons + 👣 together: direction wins (any_held is
        // already true, ε path skipped).
        // Track whether status_message changes this frame so we can
        // force a follow-up repaint. egui draws `status_bar` BEFORE
        // `gait_panel` in the frame, so an update here won't show until
        // the next frame — and in reactive mode there is no next frame
        // unless something requests it. Without `request_repaint` the
        // user clicks 👣 but the status bar stays stale until they
        // hover somewhere else.
        let mut status_changed = false;
        let design_stride = self.gait_design_stride_m;
        let stride_primary = self.gait_dpad_stride_primary;
        if let Some(gc) = self.gait_controller.as_mut() {
            // **Stride-primary D-pad mode** (only when the user has
            // explicitly opted in via the checkbox above). The D-pad
            // press sets vx as usual, but additionally rescales
            // `cycle_period_s` so the foot stride per swing stays at
            // `gait_design_stride_m`. The visible side-effect is that
            // `t3` / `t4` sliders move whenever vx changes — that's
            // the mathematical price of holding stride constant while
            // varying vx with a fixed T-based timing.
            //
            // In **vx-primary** mode (default), the D-pad just sets vx
            // and leaves cycle_period / t3 / t4 untouched; stride
            // simply varies with vx.
            let is_linear_crawl =
                gc.mode() == quadruped_gait::GaitMode::LinearCrawl;
            let apply_stride_primary =
                stride_primary && is_linear_crawl && vx.abs() > 1e-6 && design_stride > 1e-6;
            if any_held && apply_stride_primary {
                let new_t = (design_stride / vx.abs()).clamp(0.05, 20.0);
                if (gc.config().cycle_period_s - new_t).abs() > 1e-6 {
                    let mut cfg2 = gc.config().clone();
                    cfg2.cycle_period_s = new_t;
                    gc.set_config(cfg2);
                }
            }
            if any_held {
                gc.set_velocity_cmd(quadruped_gait::VelocityCmd { vx, vy, wz });
                if !self.gait_dpad_was_active {
                    self.status_message = if apply_stride_primary {
                        let new_t = (design_stride / vx.abs()).clamp(0.05, 20.0);
                        format!(
                            "D-pad cmd vx={vx:+.2} vy={vy:+.2} wz={wz:+.2}  \
                             (stride-primary: T → {new_t:.3} s for stride = {design_stride:.3} m)"
                        )
                    } else {
                        format!("D-pad cmd vx={vx:+.2} vy={vy:+.2} wz={wz:+.2}")
                    };
                    status_changed = true;
                }
            } else if march_held {
                gc.set_velocity_cmd(quadruped_gait::VelocityCmd {
                    vx: 1e-6,
                    vy: 0.0,
                    wz: 0.0,
                });
                if !self.gait_dpad_was_active {
                    self.status_message =
                        "March in place (hold 👣) — feet cycle every 0.4 s, body stays put".into();
                    status_changed = true;
                }
            } else if self.gait_dpad_was_active {
                gc.set_velocity_cmd(quadruped_gait::VelocityCmd::zero());
                self.status_message = "D-pad released → cmd=0".into();
                status_changed = true;
            }
        }
        // Even a quick click on 👣 (press+release within one frame) should
        // give the user feedback that the button is wired up.
        if march_clicked {
            self.status_message =
                "👣 clicked — for march-in-place, press AND HOLD for ≥ 0.4 s".into();
            status_changed = true;
        }
        if status_changed {
            ui.ctx().request_repaint();
        }
        self.gait_dpad_was_active = any_held || march_held;
    }

    /// Foot-link configuration + auto-detect button. Always visible so the
    /// user can re-run detection after editing the link names.
    fn draw_gait_setup_section(&mut self, ui: &mut egui::Ui) {
        ui.label(
            egui::RichText::new("Setup")
                .strong()
                .small(),
        );
        ui.label(
            egui::RichText::new(
                "Foot link names per leg. Defaults assume the standard \
                 FL/FR/RL/RR_foot convention. Click '🔍 Auto-detect' to \
                 walk the URDF chain and build the gait kinematics.",
            )
            .small()
            .weak(),
        );

        // 4 text inputs in a 2x2 grid.
        egui::Grid::new("gait_foot_links_grid")
            .num_columns(2)
            .show(ui, |ui| {
                for (idx, (leg, name)) in self.gait_foot_links.iter_mut().enumerate() {
                    ui.label(format!("{}: ", leg.label()));
                    ui.text_edit_singleline(name);
                    if idx % 2 == 1 {
                        ui.end_row();
                    }
                }
            });

        // Generator mode picker. Lives next to Setup so the user
        // chooses CHAMP / MPC before building the controller; a live
        // switch is also offered (apply to existing gc).
        ui.horizontal(|ui| {
            ui.label("Generator:");
            let mut new_mode = self.gait_mode;
            egui::ComboBox::from_id_salt("gait_mode_combo")
                .selected_text(self.gait_mode.label())
                .show_ui(ui, |ui| {
                    for m in quadruped_gait::GaitMode::ALL {
                        ui.selectable_value(&mut new_mode, m, m.label());
                    }
                });
            if new_mode != self.gait_mode {
                self.gait_mode = new_mode;
                if let Some(gc) = self.gait_controller.as_mut() {
                    gc.set_mode(new_mode);
                    // `set_mode` rebuilds the inner controller from
                    // scratch, which resets the async-solve flag — re-arm
                    // it so MPC modes keep solving off the UI thread (else
                    // the freeze returns on a live switch into a heavy MPC).
                    gc.set_async_mpc(true);
                    self.status_message =
                        format!("Gait mode → {}", new_mode.label());
                }
            }
        });

        // Pose-source picker. Switches the (yaw, position) feedback
        // source for the MPC's body_state between IMU+Madgwick and
        // MuJoCo ground truth — useful for A/B-debugging the
        // estimator path.
        #[cfg(feature = "mujoco")]
        ui.horizontal(|ui| {
            ui.label("Pose source:");
            let mut new_src = self.pose_source;
            egui::ComboBox::from_id_salt("pose_source_combo")
                .selected_text(self.pose_source.label())
                .show_ui(ui, |ui| {
                    for s in crate::gait::PoseSource::ALL {
                        ui.selectable_value(&mut new_src, s, s.label());
                    }
                });
            if new_src != self.pose_source {
                self.pose_source = new_src;
                self.status_message = format!("Pose source → {}", new_src.label());
            }
        })
        .response
        .on_hover_text(
            "Source for the body yaw + position the MPC's body_state \
             tracks each tick. ImuFusion runs the Madgwick estimator \
             on the trunk IMU's accel+gyro; GroundTruth reads MuJoCo's \
             xquat / xpos directly (sim oracle).",
        );

        // Hierarchical WBC toggle (MPC mode only — CHAMP doesn't
        // produce the GRF / contact references the WBC needs).
        #[cfg(feature = "mujoco")]
        ui.horizontal(|ui| {
            let enabled =
                self.gait_mode == quadruped_gait::GaitMode::Mpc;
            let resp = ui.add_enabled(
                enabled,
                egui::Checkbox::new(
                    &mut self.wbc_enabled,
                    "Hierarchical WBC",
                ),
            );
            if !enabled {
                self.wbc_enabled = false;
            }
            if resp.changed() {
                if self.wbc_enabled {
                    self.status_message =
                        "WBC enabled — torques solved by 3-priority HoQp".into();
                } else {
                    self.status_message =
                        "WBC disabled — back to per-joint Position-PD + τ_ff".into();
                }
            }
        })
        .response
        .on_hover_text(
            "When ON, the gait controller's joint targets are routed \
             through a 3-priority Hierarchical QP (floating-base EoM + \
             friction cone + torque limits enforced as hard constraints) \
             before being commanded to MuJoCo. Available only in MPC \
             gait mode.",
        );
        // Capture-point feedback gain (MPC-only — CHAMP has no
        // closed-loop foot placement correction). Lowering this to 0
        // disables the positive-feedback loop documented in commit
        // `eafbfc6` / `memory/project_mpc_frame_bug.md`. Required for
        // namiashi (stiff PD via .misa) to get full forward tracking.
        ui.horizontal(|ui| {
            let is_mpc = matches!(
                self.gait_mode,
                quadruped_gait::GaitMode::Mpc
                    | quadruped_gait::GaitMode::CentroidalSrbd
                    | quadruped_gait::GaitMode::FullCentroidal
            );
            ui.add_enabled_ui(is_mpc, |ui| {
                ui.label("Capture-point gain:");
                let resp = ui.add(
                    egui::Slider::new(&mut self.gait_capture_point_gain, 0.0..=0.5)
                        .fixed_decimals(3),
                );
                if resp.changed() {
                    if let Some(gc) = self.gait_controller.as_mut() {
                        gc.set_capture_point_gain(self.gait_capture_point_gain as f64);
                    }
                    if self.gait_capture_point_gain == 0.0 {
                        self.status_message =
                            "Capture-point disabled (k=0) — stiff-PD safe mode".into();
                    } else {
                        self.status_message = format!(
                            "Capture-point gain → {:.3}", self.gait_capture_point_gain,
                        );
                    }
                }
                if ui.small_button("0").on_hover_text(
                    "Set capture-point gain to 0 (= current controller default, \
                     legged_control-style open-loop Raibert).",
                ).clicked() {
                    self.gait_capture_point_gain = 0.0;
                    if let Some(gc) = self.gait_controller.as_mut() {
                        gc.set_capture_point_gain(0.0);
                    }
                    self.status_message =
                        "Capture-point gain → 0 (default, open-loop Raibert)".into();
                }
                if ui.small_button("legacy").on_hover_text(
                    "Restore the pre-D3.3.7 legacy heuristic gain (k=0.175). \
                     Use for A/B comparison against the LIP capture-point \
                     behaviour. Under stiff PD this turns into a positive \
                     feedback loop and degrades tracking — keep at 0 for \
                     production.",
                ).clicked() {
                    self.gait_capture_point_gain = 0.175;
                    if let Some(gc) = self.gait_controller.as_mut() {
                        gc.set_capture_point_gain(0.175);
                    }
                    self.status_message =
                        "Capture-point gain → 0.175 (legacy heuristic, for A/B)".into();
                }
            });
        });

        ui.label(
            egui::RichText::new(
                "CHAMP: open-loop Raibert footstep + Bezier swing. \
                 MPC: + capture-point feedback (closed-loop) + LIP \
                 horizon look-ahead — needs body velocity from the \
                 sim, fed automatically when MuJoCo is running.",
            )
            .small()
            .weak(),
        );

        // ── Goal-pose mode ────────────────────────────────────────────────
        // FullCentroidal can track an **absolute (x, y, yaw) goal** in
        // the world frame. After a lateral push, the recomputed cmd
        // pulls the body back to the goal instead of leaving it offset
        // — the legged_control `goalToTargetTrajectories` equivalent.
        ui.separator();
        let is_fullc = matches!(self.gait_mode, quadruped_gait::GaitMode::FullCentroidal);
        ui.add_enabled_ui(is_fullc, |ui| {
            ui.label(
                egui::RichText::new("Goal-pose mode (FullCentroidal only)")
                    .strong()
                    .small(),
            );
            ui.label(
                egui::RichText::new(
                    "Absolute world-frame target. After a disturbance, \
                     the body actively recovers to (x, y) — equivalent \
                     to legged_control's /move_base_simple/goal path.",
                )
                .small()
                .weak(),
            );
            ui.horizontal(|ui| {
                ui.label("x:");
                ui.add(egui::DragValue::new(&mut self.gait_goal_x_m).speed(0.05).suffix(" m"));
                ui.label("y:");
                ui.add(egui::DragValue::new(&mut self.gait_goal_y_m).speed(0.05).suffix(" m"));
                ui.label("yaw:");
                ui.add(
                    egui::DragValue::new(&mut self.gait_goal_yaw_rad)
                        .speed(0.05)
                        .suffix(" rad"),
                );
            });
            ui.horizontal(|ui| {
                ui.label("max v:");
                ui.add(
                    egui::DragValue::new(&mut self.gait_goal_max_v_m_s)
                        .speed(0.01)
                        .range(0.01..=1.0)
                        .suffix(" m/s"),
                );
                ui.label("max wz:");
                ui.add(
                    egui::DragValue::new(&mut self.gait_goal_max_wz_rad_s)
                        .speed(0.05)
                        .range(0.05..=3.0)
                        .suffix(" rad/s"),
                );
            });
            ui.horizontal(|ui| {
                let activate = ui
                    .button("🎯 Set goal")
                    .on_hover_text(
                        "Activate goal-pose mode with the values above. \
                         While active, the controller drives the body \
                         toward (x, y, yaw) and actively recovers from \
                         lateral disturbances. Calling D-pad or the \
                         velocity sliders implicitly clears the goal.",
                    )
                    .clicked();
                let clear = ui
                    .add_enabled(self.gait_goal_pose_active, egui::Button::new("✕ Clear"))
                    .on_hover_text(
                        "Stop tracking the goal and return to velocity \
                         (cmd_vel) mode. The last velocity command is \
                         kept; press a D-pad button or move the \
                         velocity sliders to issue a new one.",
                    )
                    .clicked();
                if activate {
                    if let Some(gc) = self.gait_controller.as_mut() {
                        gc.set_goal_pose_world(quadruped_gait::GoalPoseWorld {
                            x_m: self.gait_goal_x_m as f64,
                            y_m: self.gait_goal_y_m as f64,
                            yaw_rad: self.gait_goal_yaw_rad as f64,
                            max_v_m_s: self.gait_goal_max_v_m_s as f64,
                            max_wz_rad_s: self.gait_goal_max_wz_rad_s as f64,
                            position_tolerance_m: 0.02,
                            yaw_tolerance_rad: 0.05,
                        });
                        self.gait_goal_pose_active = true;
                        self.status_message = format!(
                            "Goal-pose mode → ({:+.2}, {:+.2}, {:+.2}) max_v={:.2}",
                            self.gait_goal_x_m,
                            self.gait_goal_y_m,
                            self.gait_goal_yaw_rad,
                            self.gait_goal_max_v_m_s,
                        );
                    }
                }
                if clear {
                    if let Some(gc) = self.gait_controller.as_mut() {
                        gc.clear_goal_pose();
                        self.gait_goal_pose_active = false;
                        self.status_message =
                            "Goal-pose mode cleared → cmd_vel mode".into();
                    }
                }
                // Active-state indicator. We also poll the controller
                // each frame so a script clearing the goal via
                // `gait_clear_goal_pose` reflects here.
                let live_active = self
                    .gait_controller
                    .as_ref()
                    .and_then(|gc| gc.goal_pose_world())
                    .is_some();
                self.gait_goal_pose_active = live_active;
                if live_active {
                    ui.colored_label(
                        egui::Color32::from_rgb(120, 200, 120),
                        "● goal mode active",
                    );
                } else {
                    ui.colored_label(egui::Color32::GRAY, "○ cmd_vel mode");
                }
            });
        });

        // ── Experimental flags ────────────────────────────────────────────
        // Research toggles that landed during the 2026-05 session.
        // Default values are tuned so the panel matches the
        // controller's behaviour at construction time; toggling here
        // pushes the change into the live controller. Hover each row
        // for the "what / why" summary.
        ui.separator();
        ui.add_enabled_ui(is_fullc, |ui| {
            ui.label(
                egui::RichText::new("Experimental flags (FullCentroidal only)")
                    .strong()
                    .small(),
            );
            ui.label(
                egui::RichText::new(
                    "Research toggles from the external-force robustness \
                     experiments. Most are off by default; hover for the \
                     bench summary of each.",
                )
                .small()
                .weak(),
            );

            // Poll live state so the panel reflects whatever the
            // controller currently holds (script / test code may have
            // toggled them out-of-band).
            if let Some(gc) = self.gait_controller.as_ref() {
                if let Some(p) = gc.legged_control_parity() {
                    self.gait_exp_legged_control_parity = p;
                }
                if let Some(p) = gc.use_mpc_predicted_footstep() {
                    self.gait_exp_use_mpc_predicted_footstep = p;
                }
                let cfg = gc.config();
                self.gait_exp_transition_fraction = cfg.transition_fraction as f32;
                self.gait_exp_transition_enforce_constraint = cfg.transition_enforce_constraint;
                self.gait_exp_friction_cone_soft = cfg.friction_cone_soft;
                self.gait_exp_friction_cone_slack_penalty =
                    cfg.friction_cone_slack_penalty as f32;
                self.gait_exp_warm_start = cfg.warm_start;
                self.gait_exp_mpc_optimized_footstep = cfg.mpc_optimized_footstep;
                self.gait_exp_q_foot_xy_world = cfg.q_foot_xy_world as f32;
            }

            // legged_control parity (per-step phase + swing v_z constraint).
            let parity_resp = ui
                .checkbox(
                    &mut self.gait_exp_legged_control_parity,
                    "legged_control parity",
                )
                .on_hover_text(
                    "Per-step phase projection + NormalVelocityConstraintCppAd \
                     analogue (swing leg vertical foot velocity equality). \
                     Bench note: didn't fix lateral 4N+ fall on its own \
                     (cap-pt 0.05 did). Kept for A/B and as the prerequisite \
                     for transition_fraction below.",
                );
            if parity_resp.changed() {
                if let Some(gc) = self.gait_controller.as_mut() {
                    gc.set_legged_control_parity(self.gait_exp_legged_control_parity);
                }
            }

            // transition_fraction slider (C1, cost-side GRF ramp).
            ui.horizontal(|ui| {
                ui.label("transition_fraction:");
                let resp = ui.add(
                    egui::Slider::new(&mut self.gait_exp_transition_fraction, 0.0..=0.30)
                        .fixed_decimals(2),
                );
                if resp.changed() {
                    if let Some(gc) = self.gait_controller.as_mut() {
                        let mut new_cfg = gc.config().clone();
                        new_cfg.transition_fraction = self.gait_exp_transition_fraction as f64;
                        gc.set_config(new_cfg);
                    }
                }
            }).response.on_hover_text(
                "C1 experiment: ramps the per-leg GRF reference at touchdown / \
                 lift-off. By itself (cost-side) bench was bit-exact identical \
                 to off — `r_diag[GRF]` is too small to make the MPC track the \
                 ramp. Pair with the next checkbox for the real effect.",
            );

            // transition_enforce_constraint (C1-2, constraint-side hard f_max).
            let enforce_resp = ui
                .checkbox(
                    &mut self.gait_exp_transition_enforce_constraint,
                    "transition: enforce as hard constraint (C1-2)",
                )
                .on_hover_text(
                    "C1-2: ramps the per-leg `max_normal_force` upper bound at \
                     touchdown / lift-off as a HARD QP inequality. Bench: \
                     lateral 6N peak roll −30 %, forward 6N peak |dy| −42 % \
                     at trans_fraction = 0.05. Off when transition_fraction = 0.",
                );
            if enforce_resp.changed() {
                if let Some(gc) = self.gait_controller.as_mut() {
                    let mut new_cfg = gc.config().clone();
                    new_cfg.transition_enforce_constraint =
                        self.gait_exp_transition_enforce_constraint;
                    gc.set_config(new_cfg);
                }
            }

            // friction_cone_soft (A3): replace pyramid hard constraint
            // with slack-relaxed form + quadratic penalty.
            let soft_resp = ui
                .checkbox(
                    &mut self.gait_exp_friction_cone_soft,
                    "friction cone soft + slack (A3)",
                )
                .on_hover_text(
                    "A3: relaxes the friction pyramid via per-(leg, step) \
                     slack variables `s_x, s_y ≥ 0` with quadratic penalty \
                     `λ · s²`. Useful at the pyramid corner (μ=0.5 lateral \
                     4-6N regime) where the hard form returns AlmostSolved \
                     or falls back to the reference. f_z bounds stay hard. \
                     legged_control analogue: FrictionConeConstraint + \
                     RelaxedBarrierPenalty.",
                );
            if soft_resp.changed() {
                if let Some(gc) = self.gait_controller.as_mut() {
                    let mut new_cfg = gc.config().clone();
                    new_cfg.friction_cone_soft = self.gait_exp_friction_cone_soft;
                    gc.set_config(new_cfg);
                }
            }
            // A3 slack penalty slider.
            ui.horizontal(|ui| {
                ui.label("slack penalty:");
                let resp = ui.add(
                    egui::Slider::new(
                        &mut self.gait_exp_friction_cone_slack_penalty,
                        10.0..=10000.0,
                    )
                    .logarithmic(true)
                    .fixed_decimals(0),
                );
                if resp.changed() {
                    if let Some(gc) = self.gait_controller.as_mut() {
                        let mut new_cfg = gc.config().clone();
                        new_cfg.friction_cone_slack_penalty =
                            self.gait_exp_friction_cone_slack_penalty as f64;
                        gc.set_config(new_cfg);
                    }
                }
            })
            .response
            .on_hover_text(
                "Quadratic cost on each `s_i`. Larger → cone stays closer to \
                 hard. Smaller → more slack budget under disturbance. Only \
                 effective when A3 is on.",
            );

            // warm-start (B3).
            let warm_resp = ui
                .checkbox(&mut self.gait_exp_warm_start, "MPC warm-start (B3)")
                .on_hover_text(
                    "B3: seed each MPC tick's SQP iter 0 from the previous \
                     tick's solved trajectory (shifted by one step) instead \
                     of the gravity-balanced cmd reference. Same convergence \
                     point at steady state, but fewer iterations to get \
                     there — typical 2× speed-up on cmd-held workloads. \
                     legged_control analogue: OCS2's solverObservation \
                     warm-start.",
                );
            if warm_resp.changed() {
                if let Some(gc) = self.gait_controller.as_mut() {
                    let mut new_cfg = gc.config().clone();
                    new_cfg.warm_start = self.gait_exp_warm_start;
                    gc.set_config(new_cfg);
                }
            }

            // mpc_optimized_footstep (A1).
            let a1_resp = ui
                .checkbox(
                    &mut self.gait_exp_mpc_optimized_footstep,
                    "MPC-optimised footstep XY (A1)",
                )
                .on_hover_text(
                    "A1: adds a soft cost penalising the predicted foot-XY \
                     vs the planner-supplied touchdown target. The MPC \
                     deviates the swing-leg joint trajectory to land at \
                     the target, self-consistently with its predicted base \
                     motion. Closes the loop that P2 (above) couldn't.",
                );
            if a1_resp.changed() {
                if let Some(gc) = self.gait_controller.as_mut() {
                    let mut new_cfg = gc.config().clone();
                    new_cfg.mpc_optimized_footstep = self.gait_exp_mpc_optimized_footstep;
                    gc.set_config(new_cfg);
                }
            }
            // A1 weight slider.
            ui.horizontal(|ui| {
                ui.label("q_foot_xy_world:");
                let resp = ui.add(
                    egui::Slider::new(&mut self.gait_exp_q_foot_xy_world, 10.0..=5000.0)
                        .logarithmic(true)
                        .fixed_decimals(0),
                );
                if resp.changed() {
                    if let Some(gc) = self.gait_controller.as_mut() {
                        let mut new_cfg = gc.config().clone();
                        new_cfg.q_foot_xy_world = self.gait_exp_q_foot_xy_world as f64;
                        gc.set_config(new_cfg);
                    }
                }
            })
            .response
            .on_hover_text(
                "Weight on the foot-XY tracking residual. Only active when \
                 A1 is on. Higher → more aggressive footstep tracking, \
                 may overshoot on jumpy planner targets.",
            );

            // use_mpc_predicted_footstep (P2).
            let pred_resp = ui
                .checkbox(
                    &mut self.gait_exp_use_mpc_predicted_footstep,
                    "MPC-predicted footstep (P2)",
                )
                .on_hover_text(
                    "Replaces cap-pt feedback with a footstep correction \
                     derived from the MPC's predicted base trajectory \
                     (legged_control SwingTrajectoryPlanner analogue). Bench: \
                     **made lateral push worse** because articara's MPC \
                     doesn't optimise foot XY — the predicted base reflects \
                     sliding, not restoring. Kept as a documented negative \
                     result; the real fix is A1 (MPC state expansion).",
                );
            if pred_resp.changed() {
                if let Some(gc) = self.gait_controller.as_mut() {
                    gc.set_use_mpc_predicted_footstep(
                        self.gait_exp_use_mpc_predicted_footstep,
                    );
                }
            }
        });

        ui.horizontal(|ui| {
            if ui
                .button("🔍 Auto-detect")
                .on_hover_text(
                    "Walk the kinematic chain from each foot link upward, \
                     identifying the calf / thigh / hip joints and \
                     extracting link lengths + hip offsets. \
                     Replaces any existing gait controller.",
                )
                .clicked()
            {
                self.gait_run_autodetect();
            }
            if self.gait_controller.is_some() {
                if ui
                    .button("✕ Clear")
                    .on_hover_text("Discard the current gait controller.")
                    .clicked()
                {
                    self.gait_controller = None;
                }
            }
        });

        // Status / error display.
        match self.gait_controller.as_ref() {
            Some(_) => {
                ui.colored_label(
                    egui::Color32::from_rgb(120, 200, 120),
                    "✓ gait controller ready",
                );
            }
            None => {
                ui.colored_label(
                    egui::Color32::from_gray(150),
                    "(no gait controller built yet)",
                );
            }
        }
    }

    /// Velocity command, gait params, and start/stop. Only enabled while
    /// a controller has been built and the MuJoCo sim is alive (gait
    /// otherwise has nowhere to write its joint targets).
    fn draw_gait_runtime_section(&mut self, ui: &mut egui::Ui) {
        let Some(gc) = self.gait_controller.as_mut() else {
            ui.label("(build a controller first to enable gait playback)");
            return;
        };

        // Velocity command sliders.
        //
        // **Subtle**: egui's `Slider` with `fixed_decimals(2)` rounds the
        // underlying `&mut f64` to 2-decimal precision **on every draw**,
        // even without user interaction, and reports `changed() = true`
        // when the rounded value differs from the original. For the
        // d-pad's march-in-place command (`vx = 1e-6`) this rounds to
        // `0.00`, the slider reports changed=true, and the write-back
        // below clobbers the d-pad's cmd back to zero — freezing the
        // phase generator and killing the foot-lift animation.
        //
        // Fix: only write the cmd back when the user **actively
        // interacted** with the slider — `dragged()` / `has_focus()` /
        // `clicked()` — not when the slider auto-rounded the displayed
        // value. The local `cmd` may still hold the rounded value, but
        // the controller's stored cmd remains untouched, so the d-pad's
        // tiny-ε hack continues to drive the phase generator.
        ui.label(egui::RichText::new("Velocity command").strong().small());
        let mut cmd = gc.velocity_cmd();
        let r_vx = ui.horizontal(|ui| {
            ui.label("vx (m/s):");
            ui.add(egui::Slider::new(&mut cmd.vx, -1.0..=1.0).fixed_decimals(2))
        }).inner;
        let r_vy = ui.horizontal(|ui| {
            ui.label("vy (m/s):");
            ui.add(egui::Slider::new(&mut cmd.vy, -1.0..=1.0).fixed_decimals(2))
        }).inner;
        let r_wz = ui.horizontal(|ui| {
            ui.label("wz (rad/s):");
            ui.add(egui::Slider::new(&mut cmd.wz, -2.0..=2.0).fixed_decimals(2))
        }).inner;
        let zero_clicked = ui.button("🛑 Zero velocity").clicked();
        let user_interacted = r_vx.dragged() || r_vx.has_focus() || r_vx.clicked()
            || r_vy.dragged() || r_vy.has_focus() || r_vy.clicked()
            || r_wz.dragged() || r_wz.has_focus() || r_wz.clicked();
        if user_interacted || zero_clicked {
            if zero_clicked { cmd = VelocityCmd::zero(); }
            gc.set_velocity_cmd(cmd);
        }

        ui.separator();

        // Gait config (replaceable on the fly; the phase generator
        // preserves the current cycle position when config changes).
        ui.label(egui::RichText::new("Gait params").strong().small());
        let mut cfg = gc.config().clone();
        let mut cfg_changed = false;
        ui.horizontal(|ui| {
            ui.label("Type:");
            let mut g = cfg.gait_type;
            egui::ComboBox::from_id_salt("gait_type")
                .selected_text(g.label())
                .show_ui(ui, |ui| {
                    for t in GaitType::ALL {
                        let label = t.label();
                        ui.selectable_value(&mut g, t, label);
                    }
                });
            if g != cfg.gait_type {
                // Pull the family preset's timing & sizing defaults so the
                // GUI sliders reflect the new family at once. MPC-side
                // knobs (transition_fraction / friction_cone_* / warm_start
                // / mpc_optimized_footstep / q_foot_xy_world) are tuned
                // independently of gait family and are intentionally NOT
                // overwritten here.
                let preset = GaitConfig::for_type(g);
                cfg.gait_type = g;
                cfg.cycle_period_s = preset.cycle_period_s;
                cfg.duty_factor = preset.duty_factor;
                cfg.swing_height_m = preset.swing_height_m;
                cfg.max_step_length_m = preset.max_step_length_m;
                cfg_changed = true;
            }
        });
        ui.horizontal(|ui| {
            ui.label("Cycle period (s):");
            cfg_changed |= ui
                .add(
                    egui::DragValue::new(&mut cfg.cycle_period_s)
                        .speed(0.01)
                        .range(0.05..=20.0)
                        .fixed_decimals(3),
                )
                .changed();
        });
        ui.horizontal(|ui| {
            ui.label("Duty factor:");
            cfg_changed |= ui
                .add(
                    egui::DragValue::new(&mut cfg.duty_factor)
                        .speed(0.01)
                        .range(0.05..=0.95)
                        .fixed_decimals(3),
                )
                .changed();
        });
        ui.horizontal(|ui| {
            ui.label("Swing height (m):");
            cfg_changed |= ui
                .add(
                    egui::DragValue::new(&mut cfg.swing_height_m)
                        .speed(0.005)
                        .range(0.0..=0.2)
                        .fixed_decimals(3),
                )
                .changed();
        });
        // `Max step length` is the Raibert footstep planner's stride
        // clamp (CHAMP / MPC / Centroidal / FullCentroidal). LinearCrawl
        // computes its own stride from `cycle_period × vx` and never
        // consults this value — grey out so the user doesn't waste time
        // tuning a no-op when LinearCrawl is the active mode.
        let raibert_active =
            gc.mode() != quadruped_gait::GaitMode::LinearCrawl;
        ui.horizontal(|ui| {
            ui.add_enabled_ui(raibert_active, |ui| {
                ui.label("Max step length (m):").on_hover_text(
                    "Raibert footstep planner stride clamp. Caps the half-stride at \
                     max_step_length_m / 2 so the swing leg stays inside the leg's \
                     workspace even under aggressive vx. Used by CHAMP / MPC / \
                     Centroidal / FullCentroidal modes. Has NO effect in LinearCrawl \
                     mode (which derives stride from cycle_period × vx directly).",
                );
                cfg_changed |= ui
                    .add(
                        egui::DragValue::new(&mut cfg.max_step_length_m)
                            .speed(0.005)
                            .range(0.005..=0.5)
                            .fixed_decimals(3),
                    )
                    .changed();
            });
        });
        // ── LinearCrawl primary timing inputs ─────────────────────
        //
        // The user-facing knobs for LinearCrawl are:
        //   t3 = 3-leg support time per leg (= swing duration)
        //   t4 = 4-leg support time per sub-cycle (= 4-support window)
        //   Stride length (= |vx| · T)
        //
        // Internal storage stays as `cfg.cycle_period_s` (T) and
        // `cfg.four_support_fraction` (α). We derive t3, t4 each frame
        // for display and write back to T / α when the user edits.
        //   T = 4 · (t3 + t4)
        //   α = t4 / (t3 + t4)
        // Stride is bidirectionally bound to VelocityCmd.vx via T:
        //   stride = |vx| · T   (display)
        //   vx = sign · stride / T   (writeback on stride edit)
        let linear_crawl_active =
            gc.mode() == quadruped_gait::GaitMode::LinearCrawl;
        let t3_cur = cfg.cycle_period_s * (1.0 - cfg.four_support_fraction) / 4.0;
        let t4_cur = cfg.cycle_period_s * cfg.four_support_fraction / 4.0;
        ui.horizontal(|ui| {
            ui.add_enabled_ui(linear_crawl_active, |ui| {
                ui.label("3-leg support time (s):").on_hover_text(
                    "LinearCrawl only. Time each leg spends in the swing (= 3-leg \
                     support) phase. Together with the 4-leg support time below, sets \
                     the cycle period: T = 4 × (t3 + t4). Has no effect in CHAMP / \
                     MPC modes.",
                );
                let mut t3 = t3_cur;
                if ui
                    .add(
                        egui::DragValue::new(&mut t3)
                            .speed(0.005)
                            .range(0.001..=5.0)
                            .fixed_decimals(4),
                    )
                    .changed()
                {
                    let denom = (t3 + t4_cur).max(1e-6);
                    cfg.cycle_period_s = (4.0 * denom).clamp(0.05, 20.0);
                    cfg.four_support_fraction = (t4_cur / denom).clamp(0.05, 0.95);
                    cfg_changed = true;
                }
            });
        });
        ui.horizontal(|ui| {
            ui.add_enabled_ui(linear_crawl_active, |ui| {
                ui.label("4-leg support time (s):").on_hover_text(
                    "LinearCrawl only. Time spent in 4-leg support between each \
                     swing. Together with the 3-leg support time above, sets the \
                     cycle period: T = 4 × (t3 + t4). Has no effect in CHAMP / MPC \
                     modes.",
                );
                let mut t4 = t4_cur;
                if ui
                    .add(
                        egui::DragValue::new(&mut t4)
                            .speed(0.005)
                            .range(0.001..=5.0)
                            .fixed_decimals(4),
                    )
                    .changed()
                {
                    let denom = (t3_cur + t4).max(1e-6);
                    cfg.cycle_period_s = (4.0 * denom).clamp(0.05, 20.0);
                    cfg.four_support_fraction = (t4 / denom).clamp(0.05, 0.95);
                    cfg_changed = true;
                }
            });
        });
        // Stride length — represents the user's **design stride** (=
        // intended foot stride per swing), stored persistently in
        // `gait_design_stride_m`. The slider value is independent of
        // the live velocity command, so releasing the D-pad (vx → 0)
        // doesn't blank out the display.
        //
        // Editing the slider:
        //   - always updates `gait_design_stride_m`
        //   - if the robot is currently moving (|vx| > 1e-6), also
        //     writes back `vx = sign(vx) · stride / cycle_period` so
        //     the change takes immediate effect
        //   - if stopped (vx ≈ 0), only the design value is stored —
        //     the next D-pad press picks it up
        let cur_vx = gc.velocity_cmd().vx;
        let big_t = cfg.cycle_period_s.max(1e-6);
        ui.horizontal(|ui| {
            ui.add_enabled_ui(linear_crawl_active, |ui| {
                ui.label("Stride length (m):").on_hover_text(
                    "LinearCrawl only. **Design stride** — the foot stride per swing \
                     the gait targets. Persists across D-pad press / release (the value \
                     stays put even when vx = 0). Editing this slider always updates \
                     the design value; if the robot is currently walking, it also \
                     writes vx = stride / cycle_period back to the command so the \
                     change takes effect immediately.",
                );
                let mut stride_m = self.gait_design_stride_m;
                let max_s = 1.0 * big_t;
                let stride_changed = ui
                    .add(
                        egui::DragValue::new(&mut stride_m)
                            .speed(0.001)
                            .range(0.0..=max_s)
                            .fixed_decimals(4),
                    )
                    .changed();
                if stride_changed && big_t > 0.0 {
                    if stride_m > 1e-6 {
                        self.gait_design_stride_m = stride_m;
                    }
                    // Only push vx back when the robot is actively
                    // walking — editing the design stride while stopped
                    // shouldn't accidentally start motion.
                    if cur_vx.abs() > 1e-6 {
                        let sign = cur_vx.signum();
                        let new_vx = (sign * stride_m / big_t).clamp(-1.0, 1.0);
                        let mut new_cmd = gc.velocity_cmd();
                        new_cmd.vx = new_vx;
                        gc.set_velocity_cmd(new_cmd);
                    }
                }
            });
        });
        // Derived / read-only display so the user can still see the
        // legacy quantities that the new sliders compute behind the
        // scenes.
        ui.horizontal(|ui| {
            ui.add_enabled_ui(linear_crawl_active, |ui| {
                ui.label(format!(
                    "  ┄ derived: cycle_period = {:.3} s,  4-support fraction = {:.3}",
                    cfg.cycle_period_s, cfg.four_support_fraction,
                ))
                .on_hover_text(
                    "Live computed from 3-leg / 4-leg support times above. The legacy \
                     `cycle_period_s` and `four_support_fraction` storage fields are \
                     populated from these.",
                );
            });
        });
        // ── Swing-foot feasibility cap ────────────────────────────
        // Raising the 4-leg support time shrinks the swing window, so the
        // foot must cover the same stride faster; past the actuator limit
        // the body shakes during swing. This cap auto-reduces forward speed
        // to keep the peak swing-foot speed feasible (α is preserved). 0 =
        // disabled (legacy unbounded behaviour).
        ui.horizontal(|ui| {
            ui.add_enabled_ui(linear_crawl_active, |ui| {
                ui.label("Max swing-foot speed (m/s):").on_hover_text(
                    "LinearCrawl only. Caps the peak swing-foot speed by auto-reducing \
                     forward speed (the chosen 4-support fraction is kept). A high 4-leg \
                     support time makes the swing window tiny, so the foot speed explodes \
                     (≈ 8·v/(1−α)) and the body shakes during swing — this guard prevents \
                     that. Note: slowing the cycle does NOT help (stride scales with it \
                     too); only a lower forward speed or a lower 4-support fraction does. \
                     0 = disabled (unbounded, legacy spec).",
                );
                if ui
                    .add(
                        egui::DragValue::new(&mut cfg.max_swing_foot_speed_mps)
                            .speed(0.1)
                            .range(0.0..=20.0)
                            .fixed_decimals(1),
                    )
                    .changed()
                {
                    cfg_changed = true;
                }
            });
        });
        // Live feasibility readout: peak swing-foot speed at the current
        // design stride, and whether/where the cap is limiting it.
        ui.horizontal(|ui| {
            ui.add_enabled_ui(linear_crawl_active, |ui| {
                let alpha = cfg.four_support_fraction.clamp(0.05, 0.95);
                let s = (1.0 - alpha) * 0.25;
                let gain = (2.0 - s) / s;
                let v_design = (self.gait_design_stride_m / big_t).abs();
                let peak_uncapped = v_design * gain;
                let cap = cfg.max_swing_foot_speed_mps;
                let default_color = ui.visuals().text_color();
                let (txt, color) = if cap > 0.0 && peak_uncapped > cap {
                    let v_eff = cap / gain;
                    (
                        format!(
                            "  ┄ swing-foot peak {:.1} m/s > cap {:.1} → forward speed limited to {:.3} m/s",
                            peak_uncapped, cap, v_eff
                        ),
                        egui::Color32::from_rgb(220, 160, 60),
                    )
                } else {
                    (
                        format!(
                            "  ┄ swing-foot peak {:.1} m/s (cap {})",
                            peak_uncapped,
                            if cap > 0.0 {
                                format!("{cap:.1}")
                            } else {
                                "off".to_string()
                            }
                        ),
                        default_color,
                    )
                };
                ui.colored_label(color, txt).on_hover_text(
                    "Peak body-frame swing-foot speed at the current design stride. When \
                     it exceeds the cap the controller walks slower (shown) so the swing \
                     stays smooth. Go2-class legs track roughly up to ~4–6 m/s.",
                );
            });
        });
        if cfg_changed {
            gc.set_config(cfg);
        }

        // Knee pattern (front/rear bend direction). Four-option radio so
        // the user can flip to the typical mammalian `<>` (or whatever
        // their robot demands) in one click; per-leg overrides are still
        // possible via the Rhai `set_knee_forward` API.
        let current_pattern = gc.knee_pattern();
        let mut new_pattern = current_pattern;
        ui.horizontal(|ui| {
            ui.label("Knee pattern:")
                .on_hover_text(
                    "First char = front legs, second = rear legs.\n\
                     '<' bends backward, '>' bends forward.\n\
                     << = all back   <> = mammalian (dog/horse)\n\
                     >< = reverse    >> = all forward",
                );
            for p in quadruped_gait::KneePattern::ALL {
                if ui
                    .selectable_label(p == current_pattern, p.label())
                    .on_hover_text(match p {
                        quadruped_gait::KneePattern::BothBack => "all knees backward",
                        quadruped_gait::KneePattern::MammalianForward => {
                            "front backward, rear forward (mammalian)"
                        }
                        quadruped_gait::KneePattern::MammalianReverse => {
                            "front forward, rear backward"
                        }
                        quadruped_gait::KneePattern::BothForward => "all knees forward",
                    })
                    .clicked()
                {
                    new_pattern = p;
                }
            }
        });
        if new_pattern != current_pattern {
            gc.set_knee_pattern(new_pattern);
        }

        ui.separator();

        // Start / stop. Disabled when no MuJoCo sim — the controller
        // writes into MujocoSim::position_targets, no sim no point.
        #[cfg(feature = "mujoco")]
        let sim_alive = self.mujoco_sim.is_some();
        #[cfg(not(feature = "mujoco"))]
        let sim_alive = false;

        ui.horizontal(|ui| {
            let enabled = gc.is_enabled();
            if enabled {
                if ui.button("⏹ Stop gait").clicked() {
                    gc.disable();
                }
            } else if ui
                .add_enabled(sim_alive, egui::Button::new("▶ Start gait"))
                .on_hover_text(
                    "Begin driving joint targets every physics tick. \
                     Requires a running MuJoCo sim.",
                )
                .clicked()
            {
                gc.enable();
            }
            if !sim_alive {
                ui.colored_label(
                    egui::Color32::from_gray(150),
                    "(start MuJoCo first)",
                );
            }
        });

        // Per-leg phase status: one bar per leg, green = stance, red = swing.
        if gc.is_enabled() {
            ui.label(egui::RichText::new("Leg phases (live)").small().weak());
            // Run a no-op tick read isn't possible (the controller
            // doesn't expose its phase generator without advancing it),
            // so just show the leg labels — the real per-leg phases are
            // driven by the sim loop and the user sees them via foot
            // motion in the viewport.
            for leg in [LegId::FL, LegId::FR, LegId::RL, LegId::RR] {
                ui.label(format!("  {}: tracking", leg.label()));
            }
        }
    }

    /// Run auto-detection and replace [`Self::gait_controller`]. Errors
    /// are emitted to the status bar so the user knows which leg failed
    /// without opening the script console.
    fn gait_run_autodetect(&mut self) {
        let Some(model) = self.model.as_ref() else {
            self.status_message = "Gait setup: no model loaded".into();
            return;
        };
        let foot_links: [(LegId, &str); 4] = [
            (self.gait_foot_links[0].0, self.gait_foot_links[0].1.as_str()),
            (self.gait_foot_links[1].0, self.gait_foot_links[1].1.as_str()),
            (self.gait_foot_links[2].0, self.gait_foot_links[2].1.as_str()),
            (self.gait_foot_links[3].0, self.gait_foot_links[3].1.as_str()),
        ];
        match crate::gait::auto_detect_kinematics_config(model, &foot_links) {
            Ok(kin) => {
                // Seed the controller from the saved gait descriptor (if
                // any) so re-running auto-detect doesn't reset the user's
                // knee pattern / cycle period etc.
                let (cfg, knee_forward) = match self
                    .model
                    .as_ref()
                    .and_then(|m| m.gaits.first())
                {
                    Some(d) => {
                        let mut cfg = GaitConfig::trot()
                            .with_cycle_period(d.cycle_period_s)
                            .with_duty_factor(d.duty_factor)
                            .with_swing_height(d.swing_height_m)
                            .with_max_step_length(d.max_step_length_m)
                            .with_four_support_fraction(d.four_support_fraction);
                        cfg.gait_type = match d.gait_type {
                            misarta::config::GaitTypeConfig::Trot => GaitType::Trot,
                            misarta::config::GaitTypeConfig::Walk => GaitType::Walk,
                            misarta::config::GaitTypeConfig::Pace => GaitType::Pace,
                            misarta::config::GaitTypeConfig::Bound => GaitType::Bound,
                            misarta::config::GaitTypeConfig::Crawl => GaitType::Crawl,
                        };
                        (cfg, d.knee_forward)
                    }
                    None => (GaitConfig::trot(), [false; 4]),
                };
                match crate::gait::GaitController::build(model, kin, cfg, self.gait_mode) {
                    Ok(mut gc) => {
                        for (slot, leg) in [LegId::FL, LegId::FR, LegId::RL, LegId::RR]
                            .iter()
                            .enumerate()
                        {
                            gc.set_knee_forward(*leg, knee_forward[slot]);
                        }
                        // Carry the slider's current capture-point gain
                        // onto the freshly built controller (it has its
                        // own internal default otherwise).
                        gc.set_capture_point_gain(self.gait_capture_point_gain as f64);
                        // Solve the MPC QP on a background thread in the
                        // GUI. The solve runs synchronously inside `tick()`
                        // by default, but here it would block the eframe
                        // update loop — a full-centroidal solve (≈0.4 s)
                        // then froze the whole window once the solve time
                        // exceeded the re-solve window. Off-thread solving
                        // keeps the UI responsive (ZOH on the last result).
                        gc.set_async_mpc(true);
                        self.gait_controller = Some(gc);
                        self.status_message =
                            "Gait controller built (saved params restored)".into();
                    }
                    Err(e) => {
                        self.status_message = format!("Gait build failed: {e}");
                    }
                }
            }
            Err(errs) => {
                let summary: Vec<String> = errs
                    .into_iter()
                    .map(|(leg, msg)| format!("{}: {msg}", leg.label()))
                    .collect();
                self.status_message = format!(
                    "Gait auto-detect failed: {}",
                    summary.join("; "),
                );
            }
        }
    }
}
