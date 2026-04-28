//! Collision-pair matrix dialog.
//!
//! A modal-ish window that shows every link × link pair as a checkable cell.
//! Default behaviour is "all pairs collide" (matching MuJoCo's collide-by-default
//! and physics engines' usual default). Unchecking a cell stores a
//! [`CollisionPair { enabled: false }`](crate::rbd::model::CollisionPair) in
//! the model so it gets persisted to `.misarta.toml` and emitted as MJCF
//! `<contact><exclude>` / USD `physics:filteredPairs` on export.
//!
//! The matrix is symmetric, so we only render the upper triangle (j > i).
//! Selecting either `(A, B)` or `(B, A)` flips the same stored pair —
//! [`crate::rbd::model::CollisionPair::new`] normalises the order.

use eframe::egui;

use super::ArticaraApp;
use crate::rbd::model::CollisionPair;

impl ArticaraApp {
    pub(super) fn draw_collision_matrix_window(&mut self, ctx: &egui::Context) {
        if !self.show_collision_matrix {
            return;
        }
        let mut open = true;
        // Snapshot link names + current disabled set up-front so the closure
        // doesn't keep `self` borrowed while egui's grid runs.
        let link_names: Vec<String> = match self.model.as_ref() {
            Some(m) => m.links.iter().map(|l| l.name.clone()).collect(),
            None => {
                self.show_collision_matrix = false;
                return;
            }
        };
        // Map (a,b) → enabled? (default true if absent)
        let mut enabled_map = std::collections::HashMap::<(String, String), bool>::new();
        if let Some(model) = self.model.as_ref() {
            for cp in &model.collision_pairs {
                enabled_map.insert(
                    (cp.link_a.clone(), cp.link_b.clone()),
                    cp.enabled,
                );
            }
        }

        // Pending toggles applied after the closure to avoid borrowing self
        // mutably from within the egui callbacks.
        let mut toggles: Vec<(String, String)> = Vec::new();
        let mut clear_all = false;
        let mut disable_all = false;

        egui::Window::new("🛡 Collision Pair Matrix")
            .open(&mut open)
            .default_size([720.0, 520.0])
            .resizable(true)
            .show(ctx, |ui| {
                ui.label(
                    egui::RichText::new(
                        "Default: all link pairs collide. Uncheck a cell to \
                         exclude that pair (saved to .misarta.toml; emitted \
                         as MJCF <contact><exclude> and USD filteredPairs).",
                    )
                    .small()
                    .weak(),
                );
                ui.horizontal(|ui| {
                    if ui
                        .button("Reset (all collide)")
                        .on_hover_text("Remove every explicit pair so the model falls back to default-collide behaviour.")
                        .clicked()
                    {
                        clear_all = true;
                    }
                    if ui
                        .button("Disable all pairs")
                        .on_hover_text(
                            "Mark every link pair as excluded. Useful as a \
                             starting point when you only want a small set of \
                             pairs to collide.",
                        )
                        .clicked()
                    {
                        disable_all = true;
                    }
                    let n_disabled = enabled_map.values().filter(|v| !**v).count();
                    let n_enabled_explicit =
                        enabled_map.values().filter(|v| **v).count();
                    ui.label(
                        egui::RichText::new(format!(
                            "  {} excluded · {} explicit-enabled",
                            n_disabled, n_enabled_explicit,
                        ))
                        .small()
                        .weak(),
                    );
                });
                ui.separator();

                if link_names.is_empty() {
                    ui.label("(no links)");
                    return;
                }

                egui::ScrollArea::both()
                    .auto_shrink([false, false])
                    .show(ui, |ui| {
                        egui::Grid::new("collision_matrix")
                            .striped(true)
                            .spacing([4.0, 2.0])
                            .show(ui, |ui| {
                                // Header row: blank corner cell, then column headers.
                                ui.label("");
                                for col_name in &link_names {
                                    // Vertical-ish label so wide tables stay manageable.
                                    ui.label(
                                        egui::RichText::new(col_name).monospace().small(),
                                    );
                                }
                                ui.end_row();

                                // Body rows
                                for (i, row_name) in link_names.iter().enumerate() {
                                    ui.label(
                                        egui::RichText::new(row_name).monospace().small().strong(),
                                    );
                                    for (j, col_name) in link_names.iter().enumerate() {
                                        if j <= i {
                                            // Lower triangle + diagonal: skip (matrix is symmetric;
                                            // self-pair never collides).
                                            ui.label("");
                                            continue;
                                        }
                                        // Normalise to alphabetical key.
                                        let (a, b) = if row_name <= col_name {
                                            (row_name.clone(), col_name.clone())
                                        } else {
                                            (col_name.clone(), row_name.clone())
                                        };
                                        let key = (a.clone(), b.clone());
                                        let mut enabled =
                                            enabled_map.get(&key).copied().unwrap_or(true);
                                        let was_enabled = enabled;
                                        ui.checkbox(&mut enabled, "")
                                            .on_hover_text(format!(
                                                "{} ↔ {}",
                                                a, b
                                            ));
                                        if enabled != was_enabled {
                                            toggles.push((a, b));
                                        }
                                    }
                                    ui.end_row();
                                }
                            });
                    });
            });

        // Apply toggles AFTER the closure so we can take &mut self.model.
        if let Some(model) = self.model.as_mut() {
            for (a, b) in toggles {
                // Determine current state and flip.
                let cur = model
                    .collision_pairs
                    .iter()
                    .find(|p| p.matches(&a, &b))
                    .map(|p| p.enabled)
                    .unwrap_or(true);
                let new_enabled = !cur;
                // Remove any existing entry, then insert the toggled one
                // unless the new state matches the default ("collide").
                model.collision_pairs.retain(|p| !p.matches(&a, &b));
                if !new_enabled {
                    // Default is "collide", so we only need to record the
                    // explicit-disable case to keep TOML small. The user's
                    // explicit-enable case (if they want to override an
                    // ancestor exclude later) can be re-introduced if we
                    // ever add scene-level groups; for now disable-only.
                    model.collision_pairs.push(CollisionPair::new(a, b, false));
                }
            }
            if clear_all {
                model.collision_pairs.clear();
            }
            if disable_all {
                let names: Vec<String> = model.links.iter().map(|l| l.name.clone()).collect();
                let mut new_pairs = Vec::new();
                for i in 0..names.len() {
                    for j in (i + 1)..names.len() {
                        new_pairs.push(CollisionPair::new(
                            names[i].clone(),
                            names[j].clone(),
                            false,
                        ));
                    }
                }
                model.collision_pairs = new_pairs;
            }
        }

        if !open {
            self.show_collision_matrix = false;
        }
    }
}
