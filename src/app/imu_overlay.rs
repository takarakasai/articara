//! Viewport overlay: IMU attitude estimate vs ground truth.
//!
//! For every IMU sensor declared in the loaded `RobotModel`, this draws
//! two coordinate triads at the sensor's mount point:
//!
//! - **Ground truth** (solid) — the IMU mount frame's actual world
//!   orientation, taken from MuJoCo's forward-kinematics output.
//! - **Estimate** (dashed) — the Madgwick filter's reconstruction,
//!   computed in `ArticaraApp::update_imu_estimators`.
//!
//! Without a magnetometer, the Madgwick filter has no absolute heading
//! reference: roll & pitch lock to gravity, but yaw integrates from
//! gyro only and drifts. The dashed-vs-solid comparison makes that
//! visible at a glance.

#![cfg(feature = "mujoco")]

use eframe::egui;
use nalgebra as na;

use super::ArticaraApp;

/// Length of the drawn axes in world units (m). 8 cm reads well at
/// the namiashi viewport scale; tweak if other models look off.
const AXIS_LENGTH_M: f32 = 0.08;

/// X / Y / Z colours, matching the camera-axes overlay.
const X_COLOR: egui::Color32 = egui::Color32::from_rgb(255, 80, 80);
const Y_COLOR: egui::Color32 = egui::Color32::from_rgb(80, 220, 80);
const Z_COLOR: egui::Color32 = egui::Color32::from_rgb(100, 140, 255);

impl ArticaraApp {
    pub(super) fn draw_imu_attitude_overlay(
        &self,
        ui: &mut egui::Ui,
        rect: egui::Rect,
        aspect: f32,
    ) {
        if self.sim.mujoco_sim.is_none() {
            return;
        }
        let Some(ref model) = self.model else {
            return;
        };
        if model.sensors.is_empty() {
            return;
        }

        let transforms = model.compute_transforms();
        let painter = ui.painter_at(rect);

        for sensor in &model.sensors {
            if !matches!(sensor.kind, articara::rbd::model::SensorKind::Imu { .. }) {
                continue;
            }

            // Mount point in world frame (link transform composed with
            // the sensor's local origin). transforms are f32 but
            // sensor.origin is f64 — promote f64→f32 explicitly.
            let Some(link_tf) = transforms.get(&sensor.link) else {
                continue;
            };
            let s_o = &sensor.origin;
            let s_t = s_o.translation.vector;
            let s_q = s_o.rotation.quaternion();
            let sensor_origin_f32 = na::Isometry3::from_parts(
                na::Translation3::new(s_t.x as f32, s_t.y as f32, s_t.z as f32),
                na::UnitQuaternion::from_quaternion(na::Quaternion::new(
                    s_q.w as f32,
                    s_q.i as f32,
                    s_q.j as f32,
                    s_q.k as f32,
                )),
            );
            let mount_world = link_tf * sensor_origin_f32;
            let origin_world = na::Point3::from(mount_world.translation.vector);

            // ── Ground truth triad (solid) ──────────────────────────
            self.draw_triad(
                &painter,
                rect,
                aspect,
                &origin_world,
                &mount_world.rotation,
                3.0,
                false,
            );

            // ── Estimated triad (dashed, slightly transparent) ──────
            if let Some(est) = self.sim.imu_estimators.get(&sensor.name) {
                let q = est.quaternion();
                // The estimator output is sensor-frame → world rotation
                // (gravity-aligned). Convert nalgebra UnitQuaternion<f64>
                // → UnitQuaternion<f32> for the f32 rotation pipeline.
                let q_f32 = na::UnitQuaternion::from_quaternion(na::Quaternion::new(
                    q.w as f32,
                    q.i as f32,
                    q.j as f32,
                    q.k as f32,
                ));
                self.draw_triad(
                    &painter,
                    rect,
                    aspect,
                    &origin_world,
                    &q_f32,
                    2.0,
                    true,
                );

                // Numeric Roll / Pitch / Yaw label next to the triad.
                let (roll, pitch, yaw) = est.euler_zyx();
                let label_pos = self.project_world(origin_world, rect, aspect);
                if let Some(p) = label_pos {
                    painter.text(
                        p + egui::vec2(12.0, 12.0),
                        egui::Align2::LEFT_TOP,
                        format!(
                            "{}\nR {:>+6.1}°\nP {:>+6.1}°\nY {:>+6.1}°",
                            sensor.name,
                            roll.to_degrees(),
                            pitch.to_degrees(),
                            yaw.to_degrees(),
                        ),
                        egui::FontId::monospace(10.0),
                        egui::Color32::from_rgba_premultiplied(220, 220, 220, 220),
                    );
                }
            }
        }
    }

    /// Draw a coordinate triad (X red, Y green, Z blue) at `origin`
    /// rotated by `rot`. `dashed = true` renders short segmented lines
    /// to distinguish the estimate from the ground truth.
    #[allow(clippy::too_many_arguments)]
    fn draw_triad(
        &self,
        painter: &egui::Painter,
        rect: egui::Rect,
        aspect: f32,
        origin: &na::Point3<f32>,
        rot: &na::UnitQuaternion<f32>,
        thickness: f32,
        dashed: bool,
    ) {
        let axes = [
            (na::Vector3::x(), X_COLOR),
            (na::Vector3::y(), Y_COLOR),
            (na::Vector3::z(), Z_COLOR),
        ];
        let Some(o2) = self.project_world(*origin, rect, aspect) else {
            return;
        };
        for (axis, color) in axes {
            let tip_world = origin + rot * axis * AXIS_LENGTH_M;
            let Some(t2) = self.project_world(tip_world, rect, aspect) else {
                continue;
            };
            if dashed {
                draw_dashed_line(painter, o2, t2, color, thickness);
            } else {
                painter.line_segment([o2, t2], egui::Stroke::new(thickness, color));
            }
            // Small dot at the tip so the axis direction is unambiguous
            // even when foreshortened.
            painter.circle_filled(t2, thickness * 0.9, color);
        }
        // Common origin marker so the two triads are visibly anchored
        // to the same point even when their tips diverge.
        painter.circle_stroke(o2, 3.0, egui::Stroke::new(1.0, egui::Color32::WHITE));
    }
}

/// Draw a screen-space dashed line. egui's painter has no built-in
/// dashed-line primitive, so we segment the line manually.
fn draw_dashed_line(
    painter: &egui::Painter,
    from: egui::Pos2,
    to: egui::Pos2,
    color: egui::Color32,
    thickness: f32,
) {
    const DASH: f32 = 4.0;
    const GAP: f32 = 3.0;
    let v = to - from;
    let len = v.length();
    if len < 1e-3 {
        return;
    }
    let dir = v / len;
    let mut t = 0.0;
    while t < len {
        let a = from + dir * t;
        let b = from + dir * (t + DASH).min(len);
        painter.line_segment([a, b], egui::Stroke::new(thickness, color));
        t += DASH + GAP;
    }
}
