use eframe::egui;

use super::ArticaraApp;

impl ArticaraApp {
    pub(super) fn draw_validation_window(&mut self, ctx: &egui::Context) {
        if !self.show_validation_window {
            return;
        }

        let results = &self.validation_results;
        let error_count = results.iter().filter(|v| v.has_errors()).count();
        let warn_count = results.iter().filter(|v| v.has_warnings() && !v.has_errors()).count();
        let ok_count = results.iter().filter(|v| v.is_ok()).count();
        let total = results.len();

        let title = format!("🔍 Inertia Validation ({total} links)");
        let mut open = true;
        let mut close_clicked = false;

        egui::Window::new(title)
            .open(&mut open)
            .resizable(true)
            .default_width(380.0)
            .default_height(300.0)
            .show(ctx, |ui| {
                // Summary bar
                ui.horizontal(|ui| {
                    ui.label(egui::RichText::new(format!("✅ {ok_count}"))
                        .color(egui::Color32::from_rgb(80, 200, 80)));
                    ui.label(egui::RichText::new(format!("❌ {error_count}"))
                        .color(egui::Color32::from_rgb(220, 60, 60)));
                    ui.label(egui::RichText::new(format!("⚠ {warn_count}"))
                        .color(egui::Color32::from_rgb(220, 180, 40)));
                });
                ui.separator();

                // Scrollable list
                egui::ScrollArea::vertical().max_height(400.0).show(ui, |ui| {
                    for v in results {
                        if v.is_ok() {
                            // Compact: show OK links on one line
                            ui.horizontal(|ui| {
                                ui.label(egui::RichText::new("✅").small());
                                ui.label(egui::RichText::new(&v.link_name).small());
                            });
                        } else {
                            // Show link name + each issue
                            let icon = if v.has_errors() { "❌" } else { "⚠" };
                            egui::CollapsingHeader::new(
                                egui::RichText::new(format!("{icon} {}", v.link_name))
                            )
                            .default_open(true)
                            .show(ui, |ui| {
                                for issue in &v.issues {
                                    let (icon, color) = match issue.severity {
                                        articara::robot::ValidationSeverity::Error =>
                                            ("❌", egui::Color32::from_rgb(220, 60, 60)),
                                        articara::robot::ValidationSeverity::Warning =>
                                            ("⚠", egui::Color32::from_rgb(220, 180, 40)),
                                    };
                                    ui.label(egui::RichText::new(format!("  {icon} {}", issue.message))
                                        .small().color(color));
                                }
                            });
                        }
                    }
                });

                ui.separator();
                if ui.button("Close").clicked() {
                    close_clicked = true;
                }
            });

        if !open || close_clicked {
            self.show_validation_window = false;
        }
    }
}
