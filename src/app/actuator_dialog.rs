//! Per-joint actuator settings dialog.
//!
//! Opens from `Edit → ⚙ Actuator Settings…`. Lets the user edit `actuator_mode`,
//! `actuator_kp`, `actuator_kv`, `armature`, and `joint_damping` for every
//! non-fixed joint, with bulk-apply and one-click "namiashi preset" actions so
//! getting a freshly imported URDF into a numerically-stable PD regime is a
//! couple of clicks rather than dozens of slider drags.
//!
//! "Namiashi preset" = the canonical values from `tests/fixtures/namiashi/namiashi.misa`
//! (mode=Position, kp=100, kv=1.2, armature=0.0014, joint_damping=0.1). These
//! sit comfortably inside MuJoCo's stability envelope at the default 2 ms
//! timestep for most small quadruped scales.

use eframe::egui;

use super::ArticaraApp;
use crate::rbd::model::ActuatorMode;

/// Default values from `tests/fixtures/namiashi/namiashi.misa`. Copied here as
/// constants so the preset survives even if the fixture file moves.
mod namiashi_preset {
    use super::ActuatorMode;
    pub const MODE: ActuatorMode = ActuatorMode::Position;
    pub const KP: f64 = 100.0;
    pub const KV: f64 = 1.2;
    pub const ARMATURE: f64 = 0.0014;
    pub const JOINT_DAMPING: f64 = 0.1;
}

#[derive(Clone, Copy, Default)]
struct BulkEdit {
    mode: Option<ActuatorMode>,
    kp: Option<f64>,
    kv: Option<f64>,
    armature: Option<f64>,
    joint_damping: Option<f64>,
}

impl ArticaraApp {
    pub(super) fn draw_actuator_dialog(&mut self, ctx: &egui::Context) {
        if !self.show_actuator_dialog {
            return;
        }
        let has_model = self.model.is_some();
        let mut open = true;

        // Operations queued by the closure and applied afterward to avoid
        // borrowing self mutably from within egui callbacks.
        let mut per_joint_edits: Vec<(usize, JointFieldEdit)> = Vec::new();
        let mut apply_namiashi_all = false;
        let mut bulk_apply: Option<BulkEdit> = None;
        // Bulk edit field state (kept in `self`-style stack vars because the
        // dialog doesn't have a persistent UI struct; values reset every
        // frame the dialog is open).
        let mut bulk = BulkEdit::default();

        egui::Window::new("⚙ Actuator Settings")
            .open(&mut open)
            .default_size([880.0, 560.0])
            .resizable(true)
            .show(ctx, |ui| {
                ui.label(
                    egui::RichText::new(
                        "Per-joint actuator parameters used by MJCF export and \
                         the in-process MuJoCo sim. Position-mode joints use \
                         Kp / Kv for the PD law; Velocity uses Kv only; \
                         Torque expects user-supplied τ. Armature is the \
                         reflected rotor inertia, joint_damping is passive.",
                    )
                    .small()
                    .weak(),
                );
                ui.separator();

                // ── Preset row ─────────────────────────────────────────────
                ui.horizontal(|ui| {
                    ui.label(
                        egui::RichText::new("Preset:").strong(),
                    );
                    if ui
                        .add_enabled(
                            has_model,
                            egui::Button::new("🦵 Apply namiashi preset to all"),
                        )
                        .on_hover_text(format!(
                            "Set every non-fixed joint to: mode={:?}, Kp={}, Kv={}, \
                             armature={}, joint_damping={}. Matches the namiashi.misa \
                             fixture defaults that simulate cleanly at MuJoCo's 2 ms dt.",
                            namiashi_preset::MODE,
                            namiashi_preset::KP,
                            namiashi_preset::KV,
                            namiashi_preset::ARMATURE,
                            namiashi_preset::JOINT_DAMPING,
                        ))
                        .clicked()
                    {
                        apply_namiashi_all = true;
                    }
                });
                ui.separator();

                // ── Bulk edit row ──────────────────────────────────────────
                ui.label(
                    egui::RichText::new("Bulk apply (leave a field empty to keep current per-joint value):")
                        .strong(),
                );
                ui.horizontal_wrapped(|ui| {
                    // Mode dropdown — "(keep)" is the no-op default.
                    let mut mode_choice: Option<ActuatorMode> = bulk.mode;
                    let mode_label = match mode_choice {
                        Some(m) => m.label().to_string(),
                        None => "(keep)".into(),
                    };
                    ui.label("Mode:");
                    egui::ComboBox::from_id_salt("actuator_bulk_mode")
                        .selected_text(mode_label)
                        .show_ui(ui, |ui| {
                            if ui.selectable_label(mode_choice.is_none(), "(keep)").clicked() {
                                mode_choice = None;
                            }
                            for m in ActuatorMode::ALL {
                                if ui
                                    .selectable_label(mode_choice == Some(m), m.label())
                                    .clicked()
                                {
                                    mode_choice = Some(m);
                                }
                            }
                        });
                    bulk.mode = mode_choice;

                    bulk_field(ui, "Kp:", &mut bulk.kp, 0.0..=1000.0);
                    bulk_field(ui, "Kv:", &mut bulk.kv, 0.0..=100.0);
                    bulk_field(ui, "Armature:", &mut bulk.armature, 0.0..=1.0);
                    bulk_field(ui, "Damping:", &mut bulk.joint_damping, 0.0..=100.0);

                    if ui
                        .add_enabled(has_model, egui::Button::new("Apply bulk"))
                        .on_hover_text("Write every non-empty bulk-field above to every non-fixed joint.")
                        .clicked()
                    {
                        bulk_apply = Some(bulk);
                    }
                });
                ui.separator();

                // ── Per-joint grid ─────────────────────────────────────────
                let Some(model) = self.model.as_ref() else {
                    ui.label("(no model loaded)");
                    return;
                };
                egui::ScrollArea::vertical()
                    .auto_shrink([false, false])
                    .show(ui, |ui| {
                        egui::Grid::new("actuator_grid")
                            .striped(true)
                            .num_columns(7)
                            .spacing([10.0, 2.0])
                            .show(ui, |ui| {
                                // Header row.
                                ui.label(egui::RichText::new("Joint").strong());
                                ui.label(egui::RichText::new("Mode").strong());
                                ui.label(egui::RichText::new("Kp").strong());
                                ui.label(egui::RichText::new("Kv").strong());
                                ui.label(egui::RichText::new("Armature").strong());
                                ui.label(egui::RichText::new("Damping").strong());
                                ui.label(egui::RichText::new("Effort").strong())
                                    .on_hover_text("Joint effort limit (read-only here; edit in Properties panel)");
                                ui.end_row();

                                for (ji, joint) in model.joints.iter().enumerate() {
                                    if joint.joint_type == "fixed" {
                                        continue;
                                    }
                                    // Joint name (read-only).
                                    ui.label(egui::RichText::new(&joint.name).monospace().small());

                                    // Mode dropdown
                                    let mut new_mode = joint.actuator_mode;
                                    egui::ComboBox::from_id_salt(format!("actmode_{ji}"))
                                        .width(110.0)
                                        .selected_text(new_mode.label())
                                        .show_ui(ui, |ui| {
                                            for m in ActuatorMode::ALL {
                                                if ui.selectable_label(new_mode == m, m.label()).clicked() {
                                                    new_mode = m;
                                                }
                                            }
                                        });
                                    if new_mode != joint.actuator_mode {
                                        per_joint_edits.push((ji, JointFieldEdit::Mode(new_mode)));
                                    }

                                    // Numeric fields. Use DragValue so the user can tweak quickly.
                                    let mut kp = joint.actuator_kp;
                                    if ui
                                        .add(egui::DragValue::new(&mut kp).speed(1.0).range(0.0..=10000.0).max_decimals(3))
                                        .changed()
                                    {
                                        per_joint_edits.push((ji, JointFieldEdit::Kp(kp)));
                                    }
                                    let mut kv = joint.actuator_kv;
                                    if ui
                                        .add(egui::DragValue::new(&mut kv).speed(0.1).range(0.0..=1000.0).max_decimals(3))
                                        .changed()
                                    {
                                        per_joint_edits.push((ji, JointFieldEdit::Kv(kv)));
                                    }
                                    let mut arm = joint.armature;
                                    if ui
                                        .add(egui::DragValue::new(&mut arm).speed(0.0001).range(0.0..=10.0).max_decimals(5))
                                        .changed()
                                    {
                                        per_joint_edits.push((ji, JointFieldEdit::Armature(arm)));
                                    }
                                    let mut d = joint.joint_damping;
                                    if ui
                                        .add(egui::DragValue::new(&mut d).speed(0.01).range(0.0..=1000.0).max_decimals(4))
                                        .changed()
                                    {
                                        per_joint_edits.push((ji, JointFieldEdit::Damping(d)));
                                    }
                                    ui.label(
                                        egui::RichText::new(format!("{:.2}", joint.effort))
                                            .monospace()
                                            .small()
                                            .weak(),
                                    );
                                    ui.end_row();
                                }
                            });
                    });
            });

        // Apply queued edits with &mut self.
        if let Some(model) = self.model.as_mut() {
            let mut applied_any = false;
            for (ji, edit) in per_joint_edits {
                if ji >= model.joints.len() {
                    continue;
                }
                let j = &mut model.joints[ji];
                match edit {
                    JointFieldEdit::Mode(m) => j.actuator_mode = m,
                    JointFieldEdit::Kp(v) => j.actuator_kp = v,
                    JointFieldEdit::Kv(v) => j.actuator_kv = v,
                    JointFieldEdit::Armature(v) => j.armature = v,
                    JointFieldEdit::Damping(v) => j.joint_damping = v,
                }
                applied_any = true;
            }
            if apply_namiashi_all {
                let mut n = 0usize;
                for j in &mut model.joints {
                    if j.joint_type == "fixed" {
                        continue;
                    }
                    j.actuator_mode = namiashi_preset::MODE;
                    j.actuator_kp = namiashi_preset::KP;
                    j.actuator_kv = namiashi_preset::KV;
                    j.armature = namiashi_preset::ARMATURE;
                    j.joint_damping = namiashi_preset::JOINT_DAMPING;
                    n += 1;
                }
                self.status_message =
                    format!("🦵 namiashi preset applied to {n} joint(s)");
                applied_any = true;
            }
            if let Some(b) = bulk_apply {
                let mut n = 0usize;
                for j in &mut model.joints {
                    if j.joint_type == "fixed" {
                        continue;
                    }
                    if let Some(m) = b.mode {
                        j.actuator_mode = m;
                    }
                    if let Some(v) = b.kp {
                        j.actuator_kp = v;
                    }
                    if let Some(v) = b.kv {
                        j.actuator_kv = v;
                    }
                    if let Some(v) = b.armature {
                        j.armature = v;
                    }
                    if let Some(v) = b.joint_damping {
                        j.joint_damping = v;
                    }
                    n += 1;
                }
                self.status_message = format!("✓ Bulk-applied to {n} joint(s)");
                applied_any = true;
            }
            if applied_any {
                // Invalidate the cached misarta model so the next MuJoCo
                // build picks up the new gains / armature.
                model.misarta_cache = None;
            }
        }

        if !open {
            self.show_actuator_dialog = false;
        }
    }
}

#[derive(Clone, Copy)]
enum JointFieldEdit {
    Mode(ActuatorMode),
    Kp(f64),
    Kv(f64),
    Armature(f64),
    Damping(f64),
}

/// Render one `Optional<f64>` slot for the bulk-edit row: a label, a
/// "(keep)" toggle, and a DragValue. When the toggle is OFF the field is
/// considered "(keep current per-joint value)" and the slot stays `None`.
fn bulk_field(
    ui: &mut egui::Ui,
    label: &str,
    slot: &mut Option<f64>,
    range: std::ops::RangeInclusive<f64>,
) {
    ui.label(label);
    let mut on = slot.is_some();
    ui.checkbox(&mut on, "")
        .on_hover_text("On = include this field in the bulk write");
    if on && slot.is_none() {
        // Default to mid-range when the user enables the slot.
        *slot = Some(*range.start());
    } else if !on && slot.is_some() {
        *slot = None;
    }
    if let Some(v) = slot.as_mut() {
        ui.add(
            egui::DragValue::new(v)
                .speed(0.1)
                .range(range)
                .max_decimals(4),
        );
    } else {
        ui.add_enabled(false, egui::DragValue::new(&mut 0.0_f64).speed(0.1));
    }
}
