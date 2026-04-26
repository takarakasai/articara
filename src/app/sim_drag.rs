//! Sim-time mouse-drag interaction.
//!
//! While a MuJoCo simulation is running, a left-click drag on a link can be
//! interpreted in one of two ways (selected via [`super::SimDragMode`]):
//!
//! - **Force**: the drag delta becomes a continuous world-frame wrench
//!   applied via MuJoCo's `xfrc_applied`. The force is refreshed every frame
//!   while the mouse is held so the user effectively "pushes" the link
//!   around. Magnitude scales with both the drag distance and a user-tunable
//!   gain so a fixed mouse movement can apply different orders of magnitude.
//! - **Posture**: the drag delta steers a posture target — an IK solve from
//!   the kinematic root keeps the dragged link's surface point under the
//!   cursor by updating the per-joint `position_targets` consumed by the
//!   PD controller. Unlike the standalone IK drag in [`super::viewport`]
//!   (which writes directly into `joint_positions`), this path leaves the
//!   actual physical motion to the controller, so gains, motor limits, and
//!   the active enforcement toggle all stay in effect.
//!
//! The two modes share the same pick-and-drag plumbing; only the per-frame
//! update differs. State is held on [`super::ArticaraApp`] in `sim_drag_state`
//! so the existing left-click drag handlers can early-out when sim drag
//! takes precedence.

#![cfg(feature = "mujoco")]

use eframe::egui;
use nalgebra as na;
use std::collections::HashMap;

use super::{ArticaraApp, SimDragMode, SimDragState};

impl ArticaraApp {
    /// Whether a sim-drag should take over the next mouse-press, given the
    /// current sim state and interaction mode.
    pub(super) fn sim_drag_active_target(&self) -> bool {
        // Sim drag only takes precedence when MuJoCo is running.
        // Offset Adjust mode keeps its gizmo-driven flow.
        self.mujoco_sim.is_some()
            && matches!(self.interaction_mode, super::InteractionMode::JointDrive)
    }

    /// Try to start a sim-drag in response to a left-mouse press. Returns
    /// `true` if a sim-drag was started so the caller can skip the normal
    /// JointDrive drag-start path.
    pub(super) fn handle_sim_drag_start(
        &mut self,
        mouse_ndc: Option<na::Point2<f32>>,
        aspect: f32,
        transforms: &HashMap<String, na::Isometry3<f32>>,
    ) -> bool {
        if !self.sim_drag_active_target() {
            return false;
        }
        let Some(ndc) = mouse_ndc else {
            return false;
        };
        let Some(model) = self.model.as_ref() else {
            return false;
        };
        let (ro, rd) = self.camera.screen_ray(ndc, aspect);
        let pick_result = model.pick_link(&ro, &rd, transforms);
        let (li, hit_dist) = if let Some(hit) = pick_result {
            hit
        } else if let Some(hov_li) = self.hovered_link {
            let link_tf = transforms
                .get(&model.links[hov_li].name)
                .copied()
                .unwrap_or(na::Isometry3::identity());
            let (center, _) = model.link_bounding_sphere(hov_li);
            let world_center = link_tf * center;
            let approx_dist = (world_center - ro).dot(&rd).max(0.01);
            (hov_li, approx_dist)
        } else {
            return false;
        };

        let link_name = model.links[li].name.clone();
        let hit_world = ro + rd * hit_dist;
        let link_world_tf = transforms
            .get(&link_name)
            .copied()
            .unwrap_or(na::Isometry3::identity());
        let ee_local_offset = link_world_tf.inverse() * hit_world;

        let cam_fwd =
            (self.camera.target - na::Point3::from(self.camera.eye().coords)).normalize();
        let drag_depth = (hit_world - self.camera.eye()).dot(&cam_fwd);

        let chain = match self.sim_drag_mode {
            SimDragMode::Posture => {
                model.chain_joints_between(&link_name, self.ik_root_link.as_deref())
            }
            SimDragMode::Force => Vec::new(),
        };

        // Posture mode without a chain has nothing to update — bail and let
        // the user know via status.
        if matches!(self.sim_drag_mode, SimDragMode::Posture) && chain.is_empty() {
            self.status_message =
                format!("(no IK chain to '{}', cannot run posture drag)", link_name);
            return false;
        }

        self.sim_drag_state = Some(SimDragState {
            mode: self.sim_drag_mode,
            link_name: link_name.clone(),
            ee_local_offset,
            drag_depth,
            chain,
            ik_root_link: self.ik_root_link.clone(),
        });
        self.selected_link = Some(li);
        self.selected_joint = model.parent_joint_of_link(&link_name);
        true
    }

    /// Per-frame update during a sim drag. Reads the current pointer NDC
    /// position and, depending on mode, either re-applies an external force
    /// or pushes a fresh IK posture target into the sim controller.
    pub(super) fn handle_sim_drag_update(
        &mut self,
        mouse_ndc: Option<na::Point2<f32>>,
        aspect: f32,
        transforms: &HashMap<String, na::Isometry3<f32>>,
    ) {
        let Some(state) = self.sim_drag_state.clone() else {
            return;
        };
        let Some(ndc) = mouse_ndc else { return };
        if self.model.is_none() {
            return;
        }

        // Project cursor onto the drag plane (perpendicular to the camera
        // forward at the click depth) so the drag stays in 3D world space.
        let (ro, rd) = self.camera.screen_ray(ndc, aspect);
        let cam_fwd =
            (self.camera.target - na::Point3::from(self.camera.eye().coords)).normalize();
        let denom = rd.dot(&cam_fwd);
        if denom.abs() < 1e-6 {
            return;
        }
        let plane_t = state.drag_depth / denom;
        let target_world = ro + rd * plane_t;

        match state.mode {
            SimDragMode::Force => {
                let link_world_tf = transforms
                    .get(&state.link_name)
                    .copied()
                    .unwrap_or(na::Isometry3::identity());
                let cur_world = link_world_tf * state.ee_local_offset;
                let delta = target_world - cur_world;
                let f = delta * self.sim_drag_force_gain;
                if let Some(ref mut sim) = self.mujoco_sim {
                    // Continuously refresh — short duration so the pulse
                    // dies the instant we stop calling apply, even if the
                    // user releases the mouse outside the viewport.
                    sim.apply_external_force(
                        &state.link_name,
                        [f.x as f64, f.y as f64, f.z as f64],
                        [0.0, 0.0, 0.0],
                        0.1,
                    );
                }
            }
            SimDragMode::Posture => {
                // Run a single IK step toward the cursor target, then push
                // the resulting joint angles into position_targets so the
                // PD controller chases the new posture.
                if state.chain.is_empty() {
                    return;
                }
                let target = target_world.cast::<f64>();
                // Take the model out so we can solve IK and write back; the
                // sim borrow is independent.
                let mut model_owned = match self.model.take() {
                    Some(m) => m,
                    None => return,
                };
                let link_world_tf = transforms
                    .get(&state.link_name)
                    .copied()
                    .unwrap_or(na::Isometry3::identity());
                let ee_world_f32 = link_world_tf * state.ee_local_offset;
                let ee_world = ee_world_f32.cast::<f64>();
                let ref_positions: Vec<f64> = state
                    .chain
                    .iter()
                    .map(|&ji| model_owned.joint_positions[ji])
                    .collect();
                let deltas = model_owned.solve_ik_step(
                    &state.chain,
                    &state.link_name,
                    state.ik_root_link.as_deref(),
                    &ee_world,
                    &target,
                    self.ik_damping as f64,
                    0.5,
                    0.1,
                    Some(&ref_positions),
                    self.ik_solver,
                    None,
                    None,
                    None,
                );
                model_owned.apply_joint_deltas(&state.chain, &deltas);
                // Snapshot the resulting full joint vector for the controller
                // before putting the model back. `joint_positions` may get
                // overwritten by `sync_back` later this frame, but the
                // position targets are sticky on the sim side.
                let target_q = model_owned.joint_positions.clone();
                self.model = Some(model_owned);
                if let Some(ref mut sim) = self.mujoco_sim {
                    for (ji, q) in target_q.iter().enumerate() {
                        sim.set_position_target(ji, *q);
                    }
                }
            }
        }
        // Make sure the sim is actually running so the drag has an effect.
        self.dynamics_sim_paused = false;
    }

    /// Clean up at the end of a sim drag. Cancels any active force pulse
    /// from Force mode so the link doesn't keep moving on its own.
    pub(super) fn handle_sim_drag_end(&mut self) {
        let Some(state) = self.sim_drag_state.take() else {
            return;
        };
        if matches!(state.mode, SimDragMode::Force) {
            if let Some(ref mut sim) = self.mujoco_sim {
                sim.cancel_external_force(&state.link_name);
            }
        }
        // Posture mode: leave the position_targets alone so the controller
        // holds the last posture the user dragged to.
    }

    /// Overlay a thin guide line from the click anchor to the current cursor
    /// position so the user can see what they're dragging.
    pub(super) fn draw_sim_drag_overlay(
        &self,
        ui: &mut egui::Ui,
        rect: egui::Rect,
        aspect: f32,
    ) {
        let Some(state) = self.sim_drag_state.as_ref() else {
            return;
        };
        let Some(model) = self.model.as_ref() else {
            return;
        };
        let transforms = model.compute_transforms();
        let link_world_tf = transforms
            .get(&state.link_name)
            .copied()
            .unwrap_or(na::Isometry3::identity());
        let cur_world = link_world_tf * state.ee_local_offset;
        let Some(p0) = self.project_world(cur_world, rect, aspect) else {
            return;
        };
        let painter = ui.painter();
        let color = match state.mode {
            SimDragMode::Force => egui::Color32::from_rgb(255, 200, 80),
            SimDragMode::Posture => egui::Color32::from_rgb(120, 220, 120),
        };
        painter.circle_stroke(p0, 6.0, egui::Stroke::new(2.0, color));
        let label = match state.mode {
            SimDragMode::Force => "Force drag",
            SimDragMode::Posture => "Posture drag",
        };
        painter.text(
            p0 + egui::vec2(8.0, -8.0),
            egui::Align2::LEFT_BOTTOM,
            label,
            egui::FontId::monospace(11.0),
            color,
        );
    }
}
