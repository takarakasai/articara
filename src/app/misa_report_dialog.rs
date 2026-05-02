//! Post-load dialog for `.misa` [`LoadReport`].
//!
//! When `RobotModel::from_misa_with_report` returns a non-empty
//! [`misarta::native::LoadReport`], the editor stashes it in
//! `pending_misa_report` and this module renders it in an egui window
//! the next frame. The dialog stays open until the user clicks OK so
//! sanitisation entries and missing-mesh warnings aren't dismissed by a
//! stray click — they're load-time decisions worth confirming.

use eframe::egui;

use super::ArticaraApp;

impl ArticaraApp {
    pub(super) fn draw_misa_report_dialog(&mut self, ctx: &egui::Context) {
        let Some(report) = self.pending_misa_report.as_ref() else {
            return;
        };
        // Snapshot so the borrow on `self` doesn't extend through the closure.
        let report = report.clone();
        let mut close = false;

        egui::Window::new("📋 .misa load report")
            .collapsible(false)
            .resizable(true)
            .default_size([640.0, 420.0])
            .anchor(egui::Align2::CENTER_CENTER, [0.0, 0.0])
            .show(ctx, |ui| {
                ui.label(
                    egui::RichText::new(
                        "The model loaded successfully but the following \
                         changes / warnings were applied. Review them and \
                         click OK when done.",
                    )
                    .small()
                    .weak(),
                );
                ui.separator();

                // ── Identifier sanitisations ────────────────────────────
                if !report.sanitized_names.is_empty() {
                    egui::CollapsingHeader::new(format!(
                        "🔧 Sanitised identifiers ({})",
                        report.sanitized_names.len()
                    ))
                    .default_open(true)
                    .show(ui, |ui| {
                        ui.label(
                            egui::RichText::new(
                                "Names that violated the identifier regex \
                                 ^[A-Za-z_][A-Za-z0-9_]*$ were rewritten. \
                                 Cross-references (joint.parent / .child, \
                                 actuator.joints[*].name, etc.) were updated \
                                 automatically.",
                            )
                            .small()
                            .weak(),
                        );
                        egui::ScrollArea::vertical()
                            .max_height(180.0)
                            .show(ui, |ui| {
                                egui::Grid::new("misa_sanitisation_grid")
                                    .striped(true)
                                    .min_col_width(80.0)
                                    .show(ui, |ui| {
                                        ui.label(egui::RichText::new("category").strong());
                                        ui.label(egui::RichText::new("original").strong());
                                        ui.label(egui::RichText::new("→").strong());
                                        ui.label(egui::RichText::new("sanitised").strong());
                                        ui.label(egui::RichText::new("reason").strong());
                                        ui.end_row();
                                        for s in &report.sanitized_names {
                                            ui.label(&s.category);
                                            ui.label(
                                                egui::RichText::new(&s.original).monospace(),
                                            );
                                            ui.label("→");
                                            ui.label(
                                                egui::RichText::new(&s.sanitized).monospace(),
                                            );
                                            ui.label(
                                                egui::RichText::new(&s.reason)
                                                    .small()
                                                    .weak(),
                                            );
                                            ui.end_row();
                                        }
                                    });
                            });
                    });
                }

                // ── Material collisions ─────────────────────────────────
                if !report.material_collisions.is_empty() {
                    egui::CollapsingHeader::new(format!(
                        "🎨 Material name collisions ({})",
                        report.material_collisions.len()
                    ))
                    .default_open(true)
                    .show(ui, |ui| {
                        ui.label(
                            egui::RichText::new(
                                "Materials with names that collided on import \
                                 were renamed; the original keeps its name.",
                            )
                            .small()
                            .weak(),
                        );
                        egui::Grid::new("misa_material_collision_grid")
                            .striped(true)
                            .show(ui, |ui| {
                                ui.label(egui::RichText::new("original").strong());
                                ui.label(egui::RichText::new("→").strong());
                                ui.label(egui::RichText::new("renamed to").strong());
                                ui.end_row();
                                for c in &report.material_collisions {
                                    ui.label(egui::RichText::new(&c.original).monospace());
                                    ui.label("→");
                                    ui.label(
                                        egui::RichText::new(&c.renamed_to).monospace(),
                                    );
                                    ui.end_row();
                                }
                            });
                    });
                }

                // ── Missing meshes ──────────────────────────────────────
                if !report.missing_meshes.is_empty() {
                    egui::CollapsingHeader::new(format!(
                        "📐 Missing mesh files ({})",
                        report.missing_meshes.len()
                    ))
                    .default_open(true)
                    .show(ui, |ui| {
                        ui.label(
                            egui::RichText::new(
                                "Mesh references that the AssetSource could \
                                 not resolve. Affected visuals are loaded \
                                 without their mesh data — non-fatal but the \
                                 viewport will show empty geometry.",
                            )
                            .small()
                            .weak(),
                        );
                        egui::ScrollArea::vertical()
                            .max_height(140.0)
                            .show(ui, |ui| {
                                for path in &report.missing_meshes {
                                    ui.label(egui::RichText::new(path).monospace());
                                }
                            });
                    });
                }

                // ── Warnings ────────────────────────────────────────────
                if !report.warnings.is_empty() {
                    egui::CollapsingHeader::new(format!(
                        "⚠ Other warnings ({})",
                        report.warnings.len()
                    ))
                    .default_open(true)
                    .show(ui, |ui| {
                        for w in &report.warnings {
                            ui.label(w);
                        }
                    });
                }

                ui.separator();
                ui.horizontal(|ui| {
                    if ui.button("OK").clicked() {
                        close = true;
                    }
                    ui.label(
                        egui::RichText::new(format!(
                            "Total: {} item(s)",
                            report.total()
                        ))
                        .small()
                        .weak(),
                    );
                });
            });

        if close {
            self.pending_misa_report = None;
        }
    }
}
