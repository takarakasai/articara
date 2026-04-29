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
                self.draw_gait_runtime_section(ui);
            });
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

        // Velocity command.
        ui.label(egui::RichText::new("Velocity command").strong().small());
        let mut cmd = gc.velocity_cmd();
        let mut changed = false;
        ui.horizontal(|ui| {
            ui.label("vx (m/s):");
            changed |= ui
                .add(egui::Slider::new(&mut cmd.vx, -1.0..=1.0).fixed_decimals(2))
                .changed();
        });
        ui.horizontal(|ui| {
            ui.label("vy (m/s):");
            changed |= ui
                .add(egui::Slider::new(&mut cmd.vy, -1.0..=1.0).fixed_decimals(2))
                .changed();
        });
        ui.horizontal(|ui| {
            ui.label("wz (rad/s):");
            changed |= ui
                .add(egui::Slider::new(&mut cmd.wz, -2.0..=2.0).fixed_decimals(2))
                .changed();
        });
        if ui.button("🛑 Zero velocity").clicked() {
            cmd = VelocityCmd::zero();
            changed = true;
        }
        if changed {
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
                cfg.gait_type = g;
                cfg.duty_factor = g.default_duty_factor();
                cfg_changed = true;
            }
        });
        ui.horizontal(|ui| {
            ui.label("Cycle period (s):");
            cfg_changed |= ui
                .add(
                    egui::DragValue::new(&mut cfg.cycle_period_s)
                        .speed(0.01)
                        .range(0.05..=2.0)
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
        ui.horizontal(|ui| {
            ui.label("Max step length (m):");
            cfg_changed |= ui
                .add(
                    egui::DragValue::new(&mut cfg.max_step_length_m)
                        .speed(0.005)
                        .range(0.005..=0.5)
                        .fixed_decimals(3),
                )
                .changed();
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
                        let cfg = GaitConfig::trot()
                            .with_cycle_period(d.cycle_period_s)
                            .with_duty_factor(d.duty_factor)
                            .with_swing_height(d.swing_height_m)
                            .with_max_step_length(d.max_step_length_m);
                        (cfg, d.knee_forward)
                    }
                    None => (GaitConfig::trot(), [false; 4]),
                };
                match crate::gait::GaitController::build(model, kin, cfg) {
                    Ok(mut gc) => {
                        for (slot, leg) in [LegId::FL, LegId::FR, LegId::RL, LegId::RR]
                            .iter()
                            .enumerate()
                        {
                            gc.set_knee_forward(*leg, knee_forward[slot]);
                        }
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
