//! Camera control panel — mode selector (Free / TPS), wipe toggle,
//! and TPS follow tunables. Lives as a CollapsingHeader in the
//! left-side scroll alongside Dynamics / Gait / History.

use eframe::egui;

use super::ArticaraApp;
use crate::camera::CameraMode;

impl ArticaraApp {
    pub(super) fn draw_camera_panel(&mut self, ui: &mut egui::Ui) {
        egui::CollapsingHeader::new("📷 Camera")
            .default_open(false)
            .show(ui, |ui| {
                // Mode selector. Use a horizontal radio so toggling is
                // a single click — most users hop between Free and TPS
                // frequently while tuning gait.
                ui.horizontal(|ui| {
                    ui.label("Mode:");
                    let mut mode = self.camera_mode;
                    for m in CameraMode::ALL {
                        if ui.selectable_label(mode == m, m.label()).clicked() {
                            mode = m;
                        }
                    }
                    if mode != self.camera_mode {
                        self.set_camera_mode(mode);
                    }
                });

                // Picture-in-picture wipe toggle — shows the *other*
                // camera in a small rect at the top-right of the
                // viewport so the user can monitor both perspectives
                // simultaneously (handy while debugging gait).
                ui.horizontal(|ui| {
                    ui.checkbox(&mut self.show_camera_wipe, "📺 Wipe")
                        .on_hover_text(
                            "Show the non-active camera as a small \
                             picture-in-picture overlay at the top-right \
                             of the viewport.",
                        );
                });

                ui.separator();

                // ── TPS settings ──
                ui.label(
                    egui::RichText::new("TPS settings")
                        .strong()
                        .small(),
                );
                let s = &mut self.tps_settings;

                ui.horizontal(|ui| {
                    ui.label("Follow link:");
                    let mut name = s.follow_link.clone().unwrap_or_default();
                    let placeholder =
                        self.model.as_ref().map(|m| m.root_link.as_str()).unwrap_or("");
                    let resp = ui.add(
                        egui::TextEdit::singleline(&mut name)
                            .hint_text(format!("(default: {placeholder})"))
                            .desired_width(140.0),
                    );
                    if resp.changed() {
                        s.follow_link = if name.trim().is_empty() {
                            None
                        } else {
                            Some(name.trim().to_string())
                        };
                    }
                });

                ui.horizontal(|ui| {
                    ui.label("Distance (m):");
                    ui.add(
                        egui::Slider::new(&mut s.distance, 0.1..=5.0)
                            .fixed_decimals(2),
                    );
                });
                ui.horizontal(|ui| {
                    ui.label("Yaw offset (rad):");
                    ui.add(
                        egui::Slider::new(&mut s.yaw_offset, -3.14..=3.14)
                            .fixed_decimals(2),
                    );
                });
                ui.horizontal(|ui| {
                    ui.label("Pitch (rad):");
                    ui.add(
                        egui::Slider::new(&mut s.pitch_offset, -1.5..=1.5)
                            .fixed_decimals(2),
                    );
                });
                ui.horizontal(|ui| {
                    ui.label("Look-at lift (m):");
                    ui.add(
                        egui::Slider::new(&mut s.target_local_offset.z, -0.5..=1.0)
                            .fixed_decimals(2),
                    )
                    .on_hover_text(
                        "Raise / lower the look-at point relative to the \
                         followed link's origin (z-axis in the link's \
                         local frame). Useful for framing a tall body.",
                    );
                });

                if ui.button("⟲ Reset TPS").clicked() {
                    self.tps_settings = crate::camera::TpsSettings::default();
                }
            });
    }
}
