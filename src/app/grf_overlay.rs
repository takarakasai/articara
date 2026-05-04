//! Viewport overlay: predicted ground reaction forces from the SRBD MPC.
//!
//! Draws one arrow per stance foot at the foot's world position,
//! pointing along the predicted GRF direction with length scaled to
//! the magnitude. Active only when the gait controller is in
//! `GaitMode::Mpc` (CHAMP doesn't compute GRFs).
//!
//! The arrows are *predicted* — the MPC says "this is the force the
//! body would need to apply to track the velocity command over the
//! next ~300 ms". Currently articara does not actually drive MuJoCo
//! with these forces (position-control chain), so they're a
//! diagnostic that becomes drive signal once the Phase 4 torque-
//! actuation rework lands.
//!
//! Colour scheme:
//! - **Green arrow** = predicted normal force (pushing up against gravity)
//! - The same arrow's component perpendicular to gravity is the
//!   tangential (friction) force — magnitude bounded by μ·f_z by the
//!   QP constraint, so visible-but-shorter than f_z when the body is
//!   tracking smoothly.

use eframe::egui;
use nalgebra as na;

use super::ArticaraApp;

/// World-units per Newton for arrow scaling. 1/200 means a 50 N force
/// draws 0.25 m long, which is roughly trunk-height — visible without
/// dwarfing the robot.
const NEWTONS_PER_METRE: f32 = 1.0 / 200.0;

/// Minimum force magnitude to draw. Below this the arrow is too small
/// to read and just clutters the corner.
const MIN_FORCE_N: f32 = 5.0;

const GRF_COLOR: egui::Color32 = egui::Color32::from_rgb(80, 230, 130);

impl ArticaraApp {
    pub(super) fn draw_grf_overlay(
        &self,
        ui: &mut egui::Ui,
        rect: egui::Rect,
        aspect: f32,
    ) {
        let Some(gc) = self.gait_controller.as_ref() else {
            return;
        };
        let Some(model) = self.model.as_ref() else {
            return;
        };
        let Some(sol) = gc.predicted_grfs() else {
            return;
        };
        if !sol.solved {
            return;
        }

        let painter = ui.painter_at(rect);
        let kin = gc.kinematics();
        let transforms = model.compute_transforms();

        // Look up the foot world position for each leg via the
        // kinematics' foot link name. If the link isn't in the model
        // (e.g. user mid-rebuild) we silently skip that arrow.
        let foot_links: [&str; 4] = [
            kin.fl.foot_link.as_str(),
            kin.fr.foot_link.as_str(),
            kin.rl.foot_link.as_str(),
            kin.rr.foot_link.as_str(),
        ];

        for (slot, foot_link) in foot_links.iter().enumerate() {
            let f_world = sol.grfs_first_step[slot];
            let mag_n = f_world.norm() as f32;
            if mag_n < MIN_FORCE_N {
                continue;
            }
            let Some(tf) = transforms.get(*foot_link) else {
                continue;
            };
            let foot_world = na::Point3::from(tf.translation.vector);
            // Scale the force vector to world-frame metres for drawing.
            let f_dir = na::Vector3::new(
                f_world.x as f32,
                f_world.y as f32,
                f_world.z as f32,
            );
            let tip_world = foot_world + f_dir * NEWTONS_PER_METRE;

            let Some(base_screen) = self.project_world(foot_world, rect, aspect) else {
                continue;
            };
            let Some(tip_screen) = self.project_world(tip_world, rect, aspect) else {
                continue;
            };
            Self::draw_screen_arrow(&painter, base_screen, tip_screen, GRF_COLOR, 2.0);
            // Numeric label next to the arrowhead.
            painter.text(
                tip_screen + egui::vec2(4.0, -4.0),
                egui::Align2::LEFT_BOTTOM,
                format!("{mag_n:.0}N"),
                egui::FontId::monospace(10.0),
                GRF_COLOR,
            );
        }
    }
}
