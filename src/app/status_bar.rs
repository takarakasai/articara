use eframe::egui;

use super::{ArticaraApp, InteractionMode};

impl ArticaraApp {
    /// Draw the bottom status bar: current mode (left) + status message (right).
    pub(super) fn draw_status_bar(&self, ui: &mut egui::Ui) {
        ui.horizontal(|ui| {
            // ── Current mode (left) ──
            let mode_label = match self.interaction_mode {
                InteractionMode::JointDrive => "🔧 Joint Drive",
                InteractionMode::OffsetAdjust => "✥ Offset Adjust",
            };
            ui.label(
                egui::RichText::new(mode_label)
                    .size(11.0)
                    .color(egui::Color32::from_rgb(160, 180, 220)),
            );

            // ── Joint limits (only when something is wrong) ──
            // Cheap to recompute and worth surfacing globally: the viewport
            // tint is easy to miss on a link that happens to be facing away.
            if let Some(model) = self.model.as_ref() {
                let violations = articara::joint_limits::check(model);
                if !violations.is_empty() {
                    let beyond = violations
                        .iter()
                        .filter(|v| {
                            v.state == articara::joint_limits::LimitState::Beyond
                        })
                        .count();
                    let (text, [r, g, b, _]) = if beyond > 0 {
                        (
                            format!("⚠ {beyond} joint(s) past limit"),
                            articara::joint_limits::BEYOND_COLOR,
                        )
                    } else {
                        (
                            format!("{} joint(s) at limit", violations.len()),
                            articara::joint_limits::AT_LIMIT_COLOR,
                        )
                    };
                    ui.separator();
                    ui.label(
                        egui::RichText::new(text)
                            .size(11.0)
                            .color(egui::Color32::from_rgb(
                                (r * 255.0) as u8,
                                (g * 255.0) as u8,
                                (b * 255.0) as u8,
                            )),
                    )
                    .on_hover_text(
                        violations
                            .iter()
                            .map(|v| v.describe())
                            .collect::<Vec<_>>()
                            .join("\n"),
                    );
                }
            }

            ui.separator();

            // ── Status message (remaining space) ──
            ui.label(
                egui::RichText::new(&self.status_message)
                    .size(11.0)
                    .color(egui::Color32::from_rgb(180, 180, 190)),
            );
        });
    }
}
