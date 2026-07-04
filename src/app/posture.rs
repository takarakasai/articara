//! Posture save/load: persist joint positions and base transform to a TOML file.
//!
//! File format (`.toml`):
//! ```toml
//! # Articara Posture
//! [base]
//! translation = [0.0, 0.0, 0.0]
//! rotation = [0.0, 0.0, 0.0, 1.0]   # quaternion (x, y, z, w)
//!
//! [joints]
//! joint_name_1 = 0.123
//! joint_name_2 = -0.456
//! ```
//!
//! When loading, joints are matched by **name** so the posture file is
//! portable across URDF edits that reorder (but don't rename) joints.

use eframe::egui;
use nalgebra as na;
use std::io::{BufRead, Write};
use std::path::Path;

use super::ArticaraApp;
use articara::robot::RobotModel;

// ───────── Save ─────────

/// Save the current posture (joint positions + base transform) to a TOML file.
pub fn save_posture(model: &RobotModel, path: &Path) -> Result<(), String> {
    // Create parent directories if needed
    if let Some(parent) = path.parent() {
        if !parent.exists() {
            std::fs::create_dir_all(parent).map_err(|e| format!("{e}"))?;
        }
    }

    let mut file = std::fs::File::create(path).map_err(|e| format!("{e}"))?;

    writeln!(file, "# Articara Posture").map_err(|e| format!("{e}"))?;
    writeln!(file).map_err(|e| format!("{e}"))?;

    // Base transform
    let t = model.base_transform.translation.vector;
    let q = model.base_transform.rotation.quaternion();
    writeln!(file, "[base]").map_err(|e| format!("{e}"))?;
    writeln!(file, "translation = [{}, {}, {}]", t.x, t.y, t.z).map_err(|e| format!("{e}"))?;
    writeln!(file, "rotation = [{}, {}, {}, {}]", q.i, q.j, q.k, q.w)
        .map_err(|e| format!("{e}"))?;
    writeln!(file).map_err(|e| format!("{e}"))?;

    // Joint positions (by name)
    writeln!(file, "[joints]").map_err(|e| format!("{e}"))?;
    for (ji, joint) in model.joints.iter().enumerate() {
        let jt = joint.joint_type.as_str();
        if jt == "fixed" {
            continue;
        }
        let pos = model.joint_positions[ji];
        // TOML bare keys: only allow A-Za-z0-9, -, _
        // If the name contains other chars, quote it.
        let key = toml_key(&joint.name);
        writeln!(file, "{} = {}", key, pos).map_err(|e| format!("{e}"))?;
    }

    Ok(())
}

// ───────── Load ─────────

/// Load posture from a TOML file and apply it to the model.
///
/// Joints are matched by name.  Missing joints in the file are left untouched;
/// extra joints in the file (not present in the model) are silently ignored.
///
/// Returns the number of joints that were successfully matched.
pub fn load_posture(model: &mut RobotModel, path: &Path) -> Result<usize, String> {
    let file = std::fs::File::open(path).map_err(|e| format!("{e}"))?;
    let reader = std::io::BufReader::new(file);

    let mut matched = 0usize;
    let mut section = Section::None;

    for line in reader.lines() {
        let line = line.map_err(|e| format!("{e}"))?;
        let line = line.trim();
        if line.is_empty() || line.starts_with('#') {
            continue;
        }

        // Section headers
        if line == "[base]" {
            section = Section::Base;
            continue;
        }
        if line == "[joints]" {
            section = Section::Joints;
            continue;
        }
        // Skip unknown sections
        if line.starts_with('[') {
            section = Section::Unknown;
            continue;
        }

        // Key = value
        if let Some((key, value)) = parse_kv(line) {
            match section {
                Section::Base => {
                    if key == "translation" {
                        if let Some(v) = parse_f64_array(value) {
                            if v.len() == 3 {
                                model.base_transform.translation =
                                    na::Translation3::new(v[0], v[1], v[2]);
                            }
                        }
                    } else if key == "rotation" {
                        if let Some(v) = parse_f64_array(value) {
                            if v.len() == 4 {
                                let quat = na::UnitQuaternion::from_quaternion(
                                    na::Quaternion::new(v[3], v[0], v[1], v[2]),
                                );
                                model.base_transform.rotation = quat;
                            }
                        }
                    }
                }
                Section::Joints => {
                    if let Ok(val) = value.trim().parse::<f64>() {
                        if let Some(&ji) = model.joint_map.get(key) {
                            model.joint_positions[ji] = val;
                            matched += 1;
                        }
                    }
                }
                _ => {}
            }
        }
    }

    Ok(matched)
}

// ───────── TOML helpers (minimal, no crate dependency) ─────────

#[derive(Clone, Copy, PartialEq)]
enum Section {
    None,
    Base,
    Joints,
    Unknown,
}

/// Format a joint name as a TOML key.
/// Uses a bare key if it only contains `[A-Za-z0-9_-]`, otherwise quotes it.
fn toml_key(name: &str) -> String {
    let bare_ok = !name.is_empty()
        && name
            .chars()
            .all(|c| c.is_ascii_alphanumeric() || c == '_' || c == '-');
    if bare_ok {
        name.to_string()
    } else {
        format!("\"{}\"", name.replace('\\', "\\\\").replace('"', "\\\""))
    }
}

/// Parse a `key = value` line.  Returns (key, value) with quotes stripped from the key.
fn parse_kv(line: &str) -> Option<(&str, &str)> {
    let eq = line.find('=')?;
    let key = line[..eq].trim();
    let value = line[eq + 1..].trim();
    // Strip surrounding quotes from key if present
    let key = key
        .strip_prefix('"')
        .and_then(|k| k.strip_suffix('"'))
        .unwrap_or(key);
    Some((key, value))
}

/// Parse a TOML inline array of floats: `[1.0, 2.0, 3.0]`.
#[allow(dead_code)]
fn parse_f32_array(s: &str) -> Option<Vec<f32>> {
    let s = s.trim();
    let inner = s.strip_prefix('[')?.strip_suffix(']')?;
    let vals: Result<Vec<f32>, _> = inner.split(',').map(|p| p.trim().parse::<f32>()).collect();
    vals.ok()
}

/// Parse a TOML inline array of f64 floats: `[1.0, 2.0, 3.0]`.
fn parse_f64_array(s: &str) -> Option<Vec<f64>> {
    let s = s.trim();
    let inner = s.strip_prefix('[')?.strip_suffix(']')?;
    let vals: Result<Vec<f64>, _> = inner.split(',').map(|p| p.trim().parse::<f64>()).collect();
    vals.ok()
}

// ───────── UI ─────────

impl ArticaraApp {
    /// Draw the Posture menu contents (called from inside a `menu_button` block).
    pub(super) fn draw_posture_menu(&mut self, ui: &mut egui::Ui) {
        let has_model = self.model.is_some();

        // --- Path input + browse buttons ---
        ui.horizontal(|ui| {
            ui.label("File:");
            ui.add_sized(
                egui::vec2(180.0, 18.0),
                egui::TextEdit::singleline(&mut self.posture_path),
            );
        });

        ui.separator();

        // --- Save ---
        ui.horizontal(|ui| {
            if ui
                .add_enabled(
                    has_model && !self.posture_path.is_empty(),
                    egui::Button::new("💾 Save"),
                )
                .on_hover_text("Save joint positions and base transform to .toml")
                .clicked()
            {
                if let Some(ref model) = self.model {
                    let path = std::path::PathBuf::from(&self.posture_path);
                    match save_posture(model, &path) {
                        Ok(()) => {
                            self.status_message = format!("Saved posture → {}", path.display());
                        }
                        Err(e) => {
                            self.status_message = format!("Save error: {e}");
                        }
                    }
                }
                ui.close();
            }
            if ui
                .add_enabled(has_model, egui::Button::new("📂 Save As…"))
                .on_hover_text("Choose file to save posture to")
                .clicked()
            {
                let start = if self.posture_path.is_empty() {
                    None
                } else {
                    Some(std::path::Path::new(&self.posture_path).to_path_buf())
                };
                self.dialogs.save_posture.open(
                    "Save Posture",
                    super::file_dialog::FileDialogMode::Save,
                    start.as_deref(),
                    &["toml"],
                );
                ui.close();
            }
        });

        // --- Load ---
        ui.horizontal(|ui| {
            if ui
                .add_enabled(
                    has_model && !self.posture_path.is_empty(),
                    egui::Button::new("📂 Load"),
                )
                .on_hover_text("Load joint positions from the .toml path above")
                .clicked()
            {
                if let Some(ref mut model) = self.model {
                    let path = std::path::PathBuf::from(&self.posture_path);
                    match load_posture(model, &path) {
                        Ok(n) => {
                            self.needs_upload = true;
                            self.status_message = format!(
                                "Loaded posture ({n} joints matched) ← {}",
                                path.display()
                            );
                        }
                        Err(e) => {
                            self.status_message = format!("Load error: {e}");
                        }
                    }
                }
                ui.close();
            }
            if ui
                .add_enabled(has_model, egui::Button::new("📂 Load…"))
                .on_hover_text("Browse for a posture file to load")
                .clicked()
            {
                let start = if self.posture_path.is_empty() {
                    None
                } else {
                    Some(std::path::Path::new(&self.posture_path).to_path_buf())
                };
                self.dialogs.open_posture.open(
                    "Load Posture",
                    super::file_dialog::FileDialogMode::Open,
                    start.as_deref(),
                    &["toml"],
                );
                ui.close();
            }
        });

        ui.separator();

        // --- Reset ---
        if ui
            .add_enabled(has_model, egui::Button::new("🔄 Reset to Zero"))
            .on_hover_text("Set all joints to 0 and reset base transform")
            .clicked()
        {
            if let Some(ref mut model) = self.model {
                for p in model.joint_positions.iter_mut() {
                    *p = 0.0;
                }
                model.base_transform = na::Isometry3::identity();
                self.needs_upload = true;
                self.status_message = "Posture reset to zero".into();
            }
            ui.close();
        }
    }
}
