//! Viewport overlay: IMU-derived vertical vibration strip chart.
//!
//! Mirrors the bottom-right gravity / camera-axes cluster: this widget
//! sits in the **bottom-left** corner and shows the trunk's recent
//! vertical (world-Z) **linear** acceleration history, computed by
//! rotating the IMU's body-frame proper acceleration into world frame
//! via the live Madgwick estimate and subtracting the gravity
//! reaction. With that subtraction a stationary trunk reads ~0 m/s²;
//! gait bounce / impact events show as oscillations above and below
//! the centre line.
//!
//! Drawn one panel per IMU sensor, stacked vertically. namiashi only
//! has the trunk IMU so the user sees a single ~120×60 px strip in
//! the corner, but the layout naturally extends if more are added.

#![cfg(feature = "mujoco")]

use eframe::egui;

use super::ArticaraApp;

/// Panel size and chrome.
const PANEL_W: f32 = 160.0;
const PANEL_H: f32 = 70.0;
const MARGIN: f32 = 10.0;
const PANEL_GAP: f32 = 6.0;

/// Default vertical scale (m/s² per half-height). Auto-grows when the
/// signal spikes past it; never shrinks below the default so light
/// motion stays visible.
const DEFAULT_SCALE_MS2: f32 = 5.0;

impl ArticaraApp {
    pub(super) fn draw_imu_vibration_overlay(
        &self,
        ui: &mut egui::Ui,
        rect: egui::Rect,
    ) {
        if self.mujoco_sim.is_none() {
            return;
        }
        let Some(ref model) = self.model else {
            return;
        };
        // Collect IMU sensors that have any history. Skip the rest so
        // an empty chart doesn't clutter the corner before sim starts.
        let mut imus: Vec<&str> = model
            .sensors
            .iter()
            .filter(|s| matches!(s.kind, crate::rbd::model::SensorKind::Imu { .. }))
            .map(|s| s.name.as_str())
            .filter(|n| {
                self.imu_vibration_history
                    .get(*n)
                    .map(|h| !h.is_empty())
                    .unwrap_or(false)
            })
            .collect();
        if imus.is_empty() {
            return;
        }
        // Stable order so panels don't jump around frame-to-frame when
        // the sensor map's HashMap ordering changes.
        imus.sort();

        let painter = ui.painter_at(rect);

        // Anchor at bottom-left, mirroring the bottom-right cluster.
        // First panel at the bottom; subsequent panels stack upward.
        for (idx, name) in imus.iter().enumerate() {
            let bottom = rect.bottom()
                - MARGIN
                - (idx as f32) * (PANEL_H + PANEL_GAP);
            let panel = egui::Rect::from_min_size(
                egui::pos2(rect.left() + MARGIN, bottom - PANEL_H),
                egui::vec2(PANEL_W, PANEL_H),
            );
            self.draw_one_vibration_panel(&painter, panel, name);
        }
    }

    fn draw_one_vibration_panel(
        &self,
        painter: &egui::Painter,
        panel: egui::Rect,
        sensor_name: &str,
    ) {
        let Some(history) = self.imu_vibration_history.get(sensor_name) else {
            return;
        };
        if history.is_empty() {
            return;
        }

        // ── Background panel ─────────────────────────────────────────
        painter.rect_filled(
            panel,
            4.0,
            egui::Color32::from_rgba_unmultiplied(20, 12, 25, 160),
        );
        painter.rect_stroke(
            panel,
            4.0,
            egui::Stroke::new(1.0, egui::Color32::from_rgba_unmultiplied(160, 70, 160, 100)),
            egui::epaint::StrokeKind::Outside,
        );

        // ── Chart layout ─────────────────────────────────────────────
        // Reserve the top ~14px for the header label so the chart area
        // doesn't collide with text.
        let header_h = 14.0;
        let pad_x = 6.0;
        let pad_y = 4.0;
        let chart = egui::Rect::from_min_max(
            egui::pos2(panel.left() + pad_x, panel.top() + header_h + pad_y),
            egui::pos2(panel.right() - pad_x, panel.bottom() - pad_y),
        );

        // Auto-scale: pick the larger of the default scale and the
        // history's peak magnitude. Never shrinks below the default so
        // the y-axis tick stays meaningful at low excitation.
        let peak_mag = history
            .iter()
            .map(|(_, a)| a.abs() as f32)
            .fold(0.0_f32, f32::max);
        let scale = peak_mag.max(DEFAULT_SCALE_MS2);

        // Centre horizontal line at 0 m/s²
        let mid_y = chart.center().y;
        painter.line_segment(
            [
                egui::pos2(chart.left(), mid_y),
                egui::pos2(chart.right(), mid_y),
            ],
            egui::Stroke::new(
                1.0,
                egui::Color32::from_rgba_unmultiplied(180, 180, 180, 80),
            ),
        );

        // ── Plot polyline ────────────────────────────────────────────
        // Map the most recent N=chart.width() samples to pixels so the
        // line uses every horizontal pixel without aliasing artefacts.
        let n_pixels = chart.width().ceil() as usize;
        let take = history.len().min(n_pixels.max(2));
        let start = history.len() - take;
        let half_h = chart.height() * 0.5;

        let pts: Vec<egui::Pos2> = history
            .iter()
            .skip(start)
            .enumerate()
            .map(|(i, (_t, a))| {
                let x = chart.left()
                    + (i as f32 / (take.saturating_sub(1).max(1) as f32))
                        * chart.width();
                // y axis: positive accel up the chart (visually
                // "trunk bouncing up" → line goes up).
                let y = mid_y - (*a as f32 / scale) * half_h;
                egui::pos2(x, y.clamp(chart.top(), chart.bottom()))
            })
            .collect();

        if pts.len() >= 2 {
            // Fill underneath the line lightly so the signal envelope
            // is readable even when the line is thin.
            painter.add(egui::Shape::line(
                pts.clone(),
                egui::Stroke::new(1.5, egui::Color32::from_rgb(120, 220, 255)),
            ));
        }

        // ── Header: sensor name + current value ──────────────────────
        let current_val = history.back().map(|(_, a)| *a).unwrap_or(0.0);
        let header = format!("{sensor_name}  {:>+5.2} m/s²", current_val);
        painter.text(
            egui::pos2(panel.left() + 6.0, panel.top() + 2.0),
            egui::Align2::LEFT_TOP,
            header,
            egui::FontId::proportional(11.0),
            egui::Color32::from_rgb(220, 220, 220),
        );
        // Right-side scale tick label so the y-axis is interpretable.
        painter.text(
            egui::pos2(chart.right() - 2.0, chart.top()),
            egui::Align2::RIGHT_TOP,
            format!("±{:.1}", scale),
            egui::FontId::monospace(9.0),
            egui::Color32::from_rgba_unmultiplied(180, 180, 180, 200),
        );
    }
}
