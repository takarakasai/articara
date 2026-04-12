use eframe::egui;

use super::ArticaraApp;

impl ArticaraApp {
    pub(super) fn draw_history_panel(&mut self, ui: &mut egui::Ui) {
        let mut goto_pos: Option<usize> = None;

        egui::CollapsingHeader::new("📋 History")
            .default_open(false)
            .show(ui, |ui| {
                let undo_n = self.history.undo_count();
                let redo_n = self.history.redo_count();

                if undo_n == 0 && redo_n == 0 {
                    ui.label(egui::RichText::new("(no operations)").weak());
                    return;
                }

                // --- Undo entries (oldest at top, newest towards current) ---
                for i in 0..undo_n {
                    let desc = self.history.undo_entries()[i].description.clone();
                    let is_current = i == undo_n - 1 && redo_n == 0;
                    let text = if i == undo_n - 1 {
                        egui::RichText::new(format!("▸ {desc}")).strong()
                    } else {
                        egui::RichText::new(format!("  {desc}"))
                    };
                    if ui.selectable_label(is_current, text).clicked() {
                        goto_pos = Some(i + 1);
                    }
                }

                // --- Redo entries (chronological = reverse of redo_stack) ---
                if redo_n > 0 {
                    ui.separator();
                    for j in 0..redo_n {
                        let idx = redo_n - 1 - j;
                        let desc = self.history.redo_entries()[idx].description.clone();
                        let text = egui::RichText::new(format!("  {desc}")).weak().italics();
                        if ui.selectable_label(false, text).clicked() {
                            goto_pos = Some(undo_n + j + 1);
                        }
                    }
                }

                ui.add_space(4.0);
                ui.label(egui::RichText::new(
                    format!("Undo: {} / Redo: {}", undo_n, redo_n)
                ).small().weak());
            });

        // Execute history jump (outside borrow scope)
        if let Some(pos) = goto_pos {
            if let Some(ref mut model) = self.model {
                if let Some(desc) = self.history.goto(pos, model) {
                    self.status_message = format!("📋 {desc}");
                }
                self.needs_upload = true;
            }
        }
    }
}
