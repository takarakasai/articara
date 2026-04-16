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
