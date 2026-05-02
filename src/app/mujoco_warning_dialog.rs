//! Startup dialog that surfaces a MuJoCo runtime version mismatch.
//!
//! When [`crate::mujoco_version::init`] (called from `main`) detects
//! that the linked `libmujoco` doesn't match the version `mujoco-rs`'s
//! FFI bindings were generated against, the editor stashes the result
//! in `pending_mujoco_warning` and this module renders it as a modal-ish
//! egui window the next frame. The user clicks OK to dismiss; the
//! dialog stays put until then so the (multi-line) recovery
//! instructions don't disappear behind a stray click.
//!
//! Dialog is **only compiled** when the `mujoco` feature is on — the
//! whole `pending_mujoco_warning` field doesn't exist otherwise, so the
//! call site in `mod.rs` is also feature-gated.

#![cfg(feature = "mujoco")]

use eframe::egui;

use super::ArticaraApp;
use crate::mujoco_version::CheckResult;

impl ArticaraApp {
    pub(super) fn draw_mujoco_warning_dialog(&mut self, ctx: &egui::Context) {
        let Some(result) = self.pending_mujoco_warning.as_ref() else {
            return;
        };
        // We only ever set the field when the result is `Mismatch`, but
        // pattern-match defensively rather than `unwrap`-ing so a future
        // refactor that adds another error variant doesn't silently
        // panic here.
        let CheckResult::Mismatch { linked, expected } = result else {
            self.pending_mujoco_warning = None;
            return;
        };
        let linked = *linked;
        let expected = *expected;

        let mut close = false;

        egui::Window::new("⚠ MuJoCo version mismatch")
            .collapsible(false)
            .resizable(false)
            .default_size([560.0, 280.0])
            .anchor(egui::Align2::CENTER_CENTER, [0.0, 0.0])
            .show(ctx, |ui| {
                ui.label(
                    egui::RichText::new(format!(
                        "Linked libmujoco is version {linked}, but mujoco-rs \
                         was built against {expected}. Calling any MuJoCo \
                         function would panic, so MuJoCo-backed features \
                         are disabled in this session."
                    ))
                    .strong(),
                );
                ui.add_space(8.0);
                ui.separator();
                ui.add_space(4.0);

                ui.label("To fix:");
                ui.add_space(4.0);

                egui::Grid::new("mujoco_version_fix_steps")
                    .num_columns(2)
                    .spacing([12.0, 6.0])
                    .show(ui, |ui| {
                        ui.label(egui::RichText::new("1.").strong());
                        ui.label(format!(
                            "Install MuJoCo {expected} \
                             (https://github.com/google-deepmind/mujoco/releases)"
                        ));
                        ui.end_row();

                        ui.label(egui::RichText::new("2.").strong());
                        ui.label(
                            "Set MUJOCO_DYNAMIC_LINK_DIR to the new install's \
                             lib/ directory and re-launch articara, or run via \
                             `cargo xtask` for auto-detection.",
                        );
                        ui.end_row();

                        ui.label(egui::RichText::new("📖").strong());
                        ui.label("See MUJOCO_SETUP.md for full instructions.");
                        ui.end_row();
                    });

                ui.add_space(8.0);
                ui.separator();
                ui.add_space(4.0);

                ui.label(
                    egui::RichText::new(
                        "Non-MuJoCo features (URDF / .misa view, IK, gait, \
                         Featherstone dynamics via misarta) remain available.",
                    )
                    .small()
                    .weak(),
                );

                ui.add_space(8.0);
                ui.horizontal(|ui| {
                    if ui.button("OK").clicked() {
                        close = true;
                    }
                    ui.label(
                        egui::RichText::new(
                            "Dismissing this dialog won't enable MuJoCo — \
                             relaunch with the correct version.",
                        )
                        .small()
                        .weak(),
                    );
                });
            });

        if close {
            self.pending_mujoco_warning = None;
        }
    }
}
