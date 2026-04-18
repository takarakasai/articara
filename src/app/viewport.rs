use eframe::egui;
use nalgebra as na;
use std::sync::Arc;

use super::{
    ArticaraApp, DragMode, DragState, GizmoOp, InteractionMode, OffsetDragState, OffsetTarget,
};
use crate::robot;

impl ArticaraApp {
    pub(super) fn draw_viewport(&mut self, ui: &mut egui::Ui) {
        let available = ui.available_size();
        let (rect, response) =
            ui.allocate_exact_size(available, egui::Sense::click_and_drag());

        self.viewport_rect = rect;
        let aspect = rect.width() / rect.height().max(1.0);

        // ===== Picking & Drag Logic =====
        let transforms = self
            .model
            .as_ref()
            .map(|m| m.compute_transforms())
            .unwrap_or_default();

        // Convert mouse position to normalized viewport coords [0..1]
        let mouse_ndc = response.hover_pos().map(|pos| {
            na::Point2::new(
                (pos.x - rect.left()) / rect.width(),
                (pos.y - rect.top()) / rect.height(),
            )
        });

        // Hover highlight: cast ray on hover
        if let (Some(ndc), Some(model)) = (mouse_ndc, &self.model) {
            if self.drag_state.is_none() && self.offset_drag_state.is_none() {
                let (ro, rd) = self.camera.screen_ray(ndc, aspect);
                self.hovered_link = model.pick_link(&ro, &rd, &transforms).map(|(li, _)| li);
            }
        } else {
            self.hovered_link = None;
        }

        // --- Gizmo hover detection (OffsetAdjust mode) ---
        let use_element_rot = self.gizmo_op == GizmoOp::Rotate || self.gizmo_op == GizmoOp::Scale;
        let gizmo_tf = self.compute_gizmo_transform(&transforms, use_element_rot);

        const GIZMO_ARROW_LENGTH: f32 = 0.08;
        const GIZMO_PICK_RADIUS: f32 = 0.012;
        const GIZMO_RING_RADIUS: f32 = 0.05;
        const GIZMO_RING_PICK_TOL: f32 = 0.015;

        // Hover: check which gizmo axis the mouse is over
        self.hovered_gizmo_axis = None;
        if self.interaction_mode == InteractionMode::OffsetAdjust
            && self.offset_drag_state.is_none()
        {
            if let (Some(ndc), Some(gt)) = (mouse_ndc, gizmo_tf) {
                let (ro, rd) = self.camera.screen_ray(ndc, aspect);
                let origin = na::Point3::from(gt.translation.vector);
                let axes = [
                    gt.rotation * na::Vector3::x(),
                    gt.rotation * na::Vector3::y(),
                    gt.rotation * na::Vector3::z(),
                ];

                if self.gizmo_op == GizmoOp::Translate || self.gizmo_op == GizmoOp::Scale {
                    let mut best_dist = f32::MAX;
                    for (i, axis) in axes.iter().enumerate() {
                        let (t_line, dist) = robot::ray_axis_closest(&ro, &rd, &origin, axis);
                        if t_line >= 0.0
                            && t_line <= GIZMO_ARROW_LENGTH
                            && dist < GIZMO_PICK_RADIUS
                            && dist < best_dist
                        {
                            best_dist = dist;
                            self.hovered_gizmo_axis = Some(i as u8);
                        }
                    }
                } else {
                    // Rotate mode: pick ring circles
                    let mut best_dist = f32::MAX;
                    for (i, axis) in axes.iter().enumerate() {
                        let dist = ray_ring_distance(
                            &ro, &rd, &origin, axis, GIZMO_RING_RADIUS,
                        );
                        if dist < GIZMO_RING_PICK_TOL && dist < best_dist {
                            best_dist = dist;
                            self.hovered_gizmo_axis = Some(i as u8);
                        }
                    }
                }
            }
        }

        // Left mouse button pressed: start drag
        let had_drag_before = self.drag_state.is_some() || self.offset_drag_state.is_some();
        if response.drag_started_by(egui::PointerButton::Primary) {
            self.handle_drag_start(&response, mouse_ndc, aspect, &transforms, gizmo_tf);
        }
        // Record undo snapshot if a new drag just started
        if !had_drag_before && (self.drag_state.is_some() || self.offset_drag_state.is_some()) {
            let desc = if let Some(ref odrag) = self.offset_drag_state {
                let target_str = match odrag.target {
                    OffsetTarget::Joint => "joint origin",
                    OffsetTarget::Visual => "visual origin",
                    OffsetTarget::Collision => "collision origin",
                };
                let op_str = match odrag.op {
                    GizmoOp::Translate => "Translate",
                    GizmoOp::Rotate => "Rotate",
                    GizmoOp::Scale => "Scale",
                };
                format!("{op_str} {target_str}")
            } else if let Some(ref ds) = self.drag_state {
                match ds.mode {
                    DragMode::SingleJoint => "Drive joint".to_string(),
                    DragMode::InverseKinematics => "IK drag".to_string(),
                }
            } else {
                "Drag".to_string()
            };
            self.mark_edit(&desc);
        }

        // Plain click (no drag): select the clicked link in tree/properties
        if response.clicked() {
            self.handle_click(mouse_ndc, aspect, &transforms);
        }

        // While dragging: handle joint drive or offset adjustment
        if response.dragged_by(egui::PointerButton::Primary) {
            self.handle_drag_update(&response, mouse_ndc, rect, aspect);
        }

        // Right-drag = pan, middle-drag = pan, scroll = zoom (always active)
        if response.dragged_by(egui::PointerButton::Secondary)
            || response.dragged_by(egui::PointerButton::Middle)
        {
            let delta = response.drag_delta();
            let right = na::Vector3::new(-self.camera.yaw.sin(), self.camera.yaw.cos(), 0.0);
            let up = na::Vector3::z();
            let pan_speed = self.camera.distance * 0.002;
            self.camera.target -= right * delta.x * pan_speed;
            self.camera.target += up * delta.y * pan_speed;
        }
        if response.hovered() {
            let scroll = ui.ctx().input(|i| i.smooth_scroll_delta.y);
            if scroll != 0.0 {
                self.camera.distance *= 1.0 - scroll * 0.002;
                self.camera.distance = self.camera.distance.clamp(0.01, 50.0);
            }
        }

        // Drag released
        if response.drag_stopped_by(egui::PointerButton::Primary) {
            self.drag_state = None;
            self.offset_drag_state = None;
            self.history.finalize();
        }

        // Update highlight and gizmo state in renderer
        {
            let mut r = self.gl_renderer.lock().unwrap();
            let highlight = if self.drag_state.is_some() {
                self.drag_state
                    .as_ref()
                    .and_then(|d| self.model.as_ref().map(|m| m.links[d.link_idx].name.clone()))
            } else {
                self.hovered_link
                    .and_then(|li| self.model.as_ref().map(|m| m.links[li].name.clone()))
            };
            r.highlight_link = highlight;

            r.gizmo_transform = gizmo_tf;
            r.gizmo_hovered_axis = self.hovered_gizmo_axis;
            r.gizmo_dragged_axis = self.offset_drag_state.as_ref().map(|d| d.axis);
            r.gizmo_op = match self.gizmo_op {
                GizmoOp::Translate => 0,
                GizmoOp::Rotate => 1,
                GizmoOp::Scale => 2,
            };
        }

        // Change cursor
        self.update_cursor(ui);

        // Paint callback
        let renderer = self.gl_renderer.clone();
        let camera = self.camera.clone();

        let callback = egui::PaintCallback {
            rect,
            callback: Arc::new(eframe::egui_glow::CallbackFn::new(
                move |info, painter| {
                    let gl = painter.gl();
                    let renderer = renderer.lock().unwrap();

                    let vp = info.viewport_in_pixels();
                    let screen_h = info.screen_size_px[1] as i32;
                    let gl_viewport = [
                        vp.left_px,
                        screen_h - vp.top_px - vp.height_px,
                        vp.width_px,
                        vp.height_px,
                    ];

                    renderer.render(gl, &camera, gl_viewport);
                },
            )),
        };

        ui.painter().add(callback);

        // ===== Viewport overlays =====
        self.draw_viewport_overlay(ui, rect);
        self.draw_com_labels(ui, rect, aspect);
        self.draw_ik_root_anchor(ui, rect, aspect);
        self.draw_camera_axes(ui, rect);
        self.draw_gravity_indicator(ui, rect);
        self.draw_camera_reset_button(ui, rect);
        self.draw_display_toggles(ui, rect);
    }

    /// Compute gizmo transform based on target type and mode.
    fn compute_gizmo_transform(
        &self,
        transforms: &std::collections::HashMap<String, na::Isometry3<f32>>,
        use_element_rot: bool,
    ) -> Option<na::Isometry3<f32>> {
        if self.interaction_mode != InteractionMode::OffsetAdjust {
            return None;
        }
        let m = self.model.as_ref()?;
        match self.offset_target {
            OffsetTarget::Joint => {
                self.selected_joint.map(|ji| {
                    let joint = &m.joints[ji];
                    let parent_tf = transforms
                        .get(&joint.parent_link)
                        .copied()
                        .unwrap_or(na::Isometry3::identity());
                    let joint_world = parent_tf * joint.origin;
                    let rot = if use_element_rot {
                        joint_world.rotation
                    } else {
                        parent_tf.rotation
                    };
                    na::Isometry3::from_parts(joint_world.translation, rot)
                })
            }
            OffsetTarget::Visual => {
                self.selected_link.and_then(|li| {
                    self.selected_visual.and_then(|vi| {
                        m.links.get(li).and_then(|link| {
                            link.visuals.get(vi).map(|vis| {
                                let link_tf = transforms
                                    .get(&link.name)
                                    .copied()
                                    .unwrap_or(na::Isometry3::identity());
                                let vis_world = link_tf * vis.origin;
                                let rot = if use_element_rot {
                                    vis_world.rotation
                                } else {
                                    link_tf.rotation
                                };
                                na::Isometry3::from_parts(vis_world.translation, rot)
                            })
                        })
                    })
                })
            }
            OffsetTarget::Collision => {
                self.selected_link.and_then(|li| {
                    self.selected_collision.and_then(|ci| {
                        m.links.get(li).and_then(|link| {
                            link.collisions.get(ci).map(|col| {
                                let link_tf = transforms
                                    .get(&link.name)
                                    .copied()
                                    .unwrap_or(na::Isometry3::identity());
                                let col_world = link_tf * col.origin;
                                let rot = if use_element_rot {
                                    col_world.rotation
                                } else {
                                    link_tf.rotation
                                };
                                na::Isometry3::from_parts(col_world.translation, rot)
                            })
                        })
                    })
                })
            }
        }
    }

    /// Handle the start of a drag interaction.
    fn handle_drag_start(
        &mut self,
        _response: &egui::Response,
        mouse_ndc: Option<na::Point2<f32>>,
        aspect: f32,
        transforms: &std::collections::HashMap<String, na::Isometry3<f32>>,
        gizmo_tf: Option<na::Isometry3<f32>>,
    ) {
        match self.interaction_mode {
            InteractionMode::OffsetAdjust => {
                // Try to pick a gizmo arrow first
                if let (Some(axis_idx), Some(gt)) =
                    (self.hovered_gizmo_axis, gizmo_tf)
                {
                    if let Some(ndc) = mouse_ndc {
                        let (ro, rd) = self.camera.screen_ray(ndc, aspect);
                        let origin = na::Point3::from(gt.translation.vector);
                        let axes = [
                            gt.rotation * na::Vector3::x(),
                            gt.rotation * na::Vector3::y(),
                            gt.rotation * na::Vector3::z(),
                        ];
                        let axis_dir = axes[axis_idx as usize];
                        let (t_line, _) = robot::ray_axis_closest(&ro, &rd, &origin, &axis_dir);
                        let inv_rot = gt.rotation.inverse();

                        let initial_angle = if self.gizmo_op == GizmoOp::Rotate {
                            compute_ring_angle(&ro, &rd, &origin, &axis_dir)
                        } else {
                            0.0
                        };
                        let cur_op = self.gizmo_op;

                        if let Some(model) = &self.model {
                            let drag = match self.offset_target {
                                OffsetTarget::Joint => {
                                    self.selected_joint.map(|ji| OffsetDragState {
                                        axis: axis_idx,
                                        target: OffsetTarget::Joint,
                                        entity_idx: ji,
                                        sub_idx: 0,
                                        axis_dir_world: axis_dir,
                                        gizmo_origin: origin,
                                        initial_t: t_line,
                                        initial_translation: model.joints[ji].origin.translation,
                                        inv_parent_rotation: inv_rot,
                                        op: cur_op,
                                        initial_rotation: model.joints[ji].origin.rotation,
                                        initial_angle,
                                        initial_geom_params: [0.0; 3],
                                    })
                                }
                                OffsetTarget::Visual => {
                                    self.selected_link.and_then(|li| {
                                        self.selected_visual.map(|vi| OffsetDragState {
                                            axis: axis_idx,
                                            target: OffsetTarget::Visual,
                                            entity_idx: li,
                                            sub_idx: vi,
                                            axis_dir_world: axis_dir,
                                            gizmo_origin: origin,
                                            initial_t: t_line,
                                            initial_translation: model.links[li].visuals[vi].origin.translation,
                                            inv_parent_rotation: inv_rot,
                                            op: cur_op,
                                            initial_rotation: model.links[li].visuals[vi].origin.rotation,
                                            initial_angle,
                                            initial_geom_params: geom_params(&model.links[li].visuals[vi].geometry),
                                        })
                                    })
                                }
                                OffsetTarget::Collision => {
                                    self.selected_link.and_then(|li| {
                                        self.selected_collision.map(|ci| OffsetDragState {
                                            axis: axis_idx,
                                            target: OffsetTarget::Collision,
                                            entity_idx: li,
                                            sub_idx: ci,
                                            axis_dir_world: axis_dir,
                                            gizmo_origin: origin,
                                            initial_t: t_line,
                                            initial_translation: model.links[li].collisions[ci].origin.translation,
                                            inv_parent_rotation: inv_rot,
                                            op: cur_op,
                                            initial_rotation: model.links[li].collisions[ci].origin.rotation,
                                            initial_angle,
                                            initial_geom_params: geom_params(&model.links[li].collisions[ci].geometry),
                                        })
                                    })
                                }
                            };
                            self.offset_drag_state = drag;
                        }
                    }
                } else if let (Some(ndc), Some(model)) = (mouse_ndc, &self.model) {
                    // No gizmo arrow hit: pick a link to select
                    let (ro, rd) = self.camera.screen_ray(ndc, aspect);
                    if let Some((li, _)) = model.pick_link(&ro, &rd, transforms) {
                        let link_name = &model.links[li].name;
                        let changed_link = self.selected_link != Some(li);
                        self.selected_link = Some(li);
                        self.tree_reveal_ancestors = model.ancestor_links(link_name);
                        if changed_link {
                            self.selected_visual = if model.links[li].visuals.is_empty() {
                                None
                            } else {
                                Some(0)
                            };
                            self.selected_collision =
                                if model.links[li].collisions.is_empty() {
                                    None
                                } else {
                                    Some(0)
                                };
                        }
                        match self.offset_target {
                            OffsetTarget::Joint => {
                                self.selected_joint = model.parent_joint_of_link(link_name);
                            }
                            OffsetTarget::Visual | OffsetTarget::Collision => {
                                self.selected_joint = None;
                            }
                        }
                    }
                }
            }
            InteractionMode::JointDrive => {
                if let (Some(ndc), Some(model)) = (mouse_ndc, &self.model) {
                    let (ro, rd) = self.camera.screen_ray(ndc, aspect);
                    if let Some((li, _dist)) = model.pick_link(&ro, &rd, transforms) {
                        let link_name = &model.links[li].name;
                        self.selected_link = Some(li);
                        self.selected_joint = model.parent_joint_of_link(link_name);
                        self.tree_reveal_ancestors = model.ancestor_links(link_name);
                        match self.drag_mode {
                            DragMode::SingleJoint => {
                                if let Some(ji) = model.parent_joint_of_link(link_name) {
                                    let joint = &model.joints[ji];
                                    if joint.joint_type == "revolute"
                                        || joint.joint_type == "continuous"
                                    {
                                        let parent_tf = transforms
                                            .get(&joint.parent_link)
                                            .copied()
                                            .unwrap_or(na::Isometry3::identity());
                                        let joint_tf = parent_tf * joint.origin;
                                        let world_axis = joint_tf * joint.axis;
                                        let pivot_world =
                                            na::Point3::from(joint_tf.translation.vector);

                                        self.drag_state = Some(DragState {
                                            link_idx: li,
                                            mode: DragMode::SingleJoint,
                                            joint_idx: ji,
                                            world_axis,
                                            pivot_world,
                                            chain: Vec::new(),
                                            ee_link: String::new(),
                                            ik_root_link: None,
                                            ik_root_initial_tf: None,
                                        });
                                    }
                                }
                            }
                            DragMode::InverseKinematics => {
                                let chain = model.chain_joints_between(
                                    link_name,
                                    self.ik_root_link.as_deref(),
                                );
                                if !chain.is_empty() {
                                    let ji =
                                        *chain.last().unwrap_or(&0);
                                    let ik_root_tf = self.ik_root_link.as_ref().and_then(|name| {
                                        transforms.get(name).copied()
                                    });
                                    self.drag_state = Some(DragState {
                                        link_idx: li,
                                        mode: DragMode::InverseKinematics,
                                        joint_idx: ji,
                                        world_axis: na::Vector3::zeros(),
                                        pivot_world: na::Point3::origin(),
                                        chain,
                                        ee_link: link_name.to_string(),
                                        ik_root_link: self.ik_root_link.clone(),
                                        ik_root_initial_tf: ik_root_tf,
                                    });
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    /// Handle a plain click to select a link.
    fn handle_click(
        &mut self,
        mouse_ndc: Option<na::Point2<f32>>,
        aspect: f32,
        transforms: &std::collections::HashMap<String, na::Isometry3<f32>>,
    ) {
        if let (Some(ndc), Some(model)) = (mouse_ndc, &self.model) {
            let (ro, rd) = self.camera.screen_ray(ndc, aspect);
            if let Some((li, _)) = model.pick_link(&ro, &rd, transforms) {
                let link_name = &model.links[li].name;
                let changed_link = self.selected_link != Some(li);
                self.selected_link = Some(li);
                self.tree_reveal_ancestors = model.ancestor_links(link_name);
                if changed_link {
                    self.selected_visual = if model.links[li].visuals.is_empty() {
                        None
                    } else {
                        Some(0)
                    };
                    self.selected_collision =
                        if model.links[li].collisions.is_empty() {
                            None
                        } else {
                            Some(0)
                        };
                }
                match self.interaction_mode {
                    InteractionMode::JointDrive => {
                        self.selected_joint = model.parent_joint_of_link(link_name);
                    }
                    InteractionMode::OffsetAdjust => {
                        match self.offset_target {
                            OffsetTarget::Joint => {
                                self.selected_joint = model.parent_joint_of_link(link_name);
                            }
                            OffsetTarget::Visual | OffsetTarget::Collision => {
                                self.selected_joint = None;
                            }
                        }
                    }
                }
            }
        }
    }

    /// Handle continuous drag updates (offset adjustment, joint drive, IK, or camera orbit).
    fn handle_drag_update(
        &mut self,
        response: &egui::Response,
        mouse_ndc: Option<na::Point2<f32>>,
        rect: egui::Rect,
        aspect: f32,
    ) {
        if let Some(ref odrag) = self.offset_drag_state.clone() {
            // --- Offset adjustment drag ---
            if let Some(ndc) = mouse_ndc {
                let (ro, rd) = self.camera.screen_ray(ndc, aspect);

                match odrag.op {
                    GizmoOp::Translate => {
                        let (t_line, _) =
                            robot::ray_axis_closest(&ro, &rd, &odrag.gizmo_origin, &odrag.axis_dir_world);
                        let delta_t = t_line - odrag.initial_t;

                        if let Some(ref mut model) = self.model {
                            let world_disp = odrag.axis_dir_world * delta_t;
                            let local_disp = odrag.inv_parent_rotation * world_disp;
                            let new_trans = na::Translation3::new(
                                odrag.initial_translation.vector.x + local_disp.x,
                                odrag.initial_translation.vector.y + local_disp.y,
                                odrag.initial_translation.vector.z + local_disp.z,
                            );

                            match odrag.target {
                                OffsetTarget::Joint => {
                                    model.joints[odrag.entity_idx].origin.translation = new_trans;
                                }
                                OffsetTarget::Visual => {
                                    model.links[odrag.entity_idx].visuals[odrag.sub_idx]
                                        .origin.translation = new_trans;
                                }
                                OffsetTarget::Collision => {
                                    model.links[odrag.entity_idx].collisions[odrag.sub_idx]
                                        .origin.translation = new_trans;
                                }
                            }
                            self.needs_upload = true;
                        }
                    }
                    GizmoOp::Rotate => {
                        let cur_angle = compute_ring_angle(
                            &ro, &rd, &odrag.gizmo_origin, &odrag.axis_dir_world,
                        );
                        let delta_angle = cur_angle - odrag.initial_angle;

                        if let Some(ref mut model) = self.model {
                            let local_axis = match odrag.axis {
                                0 => na::Vector3::x_axis(),
                                1 => na::Vector3::y_axis(),
                                _ => na::Vector3::z_axis(),
                            };
                            let delta_rot = na::UnitQuaternion::from_axis_angle(
                                &local_axis,
                                delta_angle,
                            );
                            let new_rot = odrag.initial_rotation * delta_rot;

                            match odrag.target {
                                OffsetTarget::Joint => {
                                    model.joints[odrag.entity_idx].origin.rotation = new_rot;
                                }
                                OffsetTarget::Visual => {
                                    model.links[odrag.entity_idx].visuals[odrag.sub_idx]
                                        .origin.rotation = new_rot;
                                }
                                OffsetTarget::Collision => {
                                    model.links[odrag.entity_idx].collisions[odrag.sub_idx]
                                        .origin.rotation = new_rot;
                                }
                            }
                            self.needs_upload = true;
                        }
                    }
                    GizmoOp::Scale => {
                        let (t_line, _) =
                            robot::ray_axis_closest(&ro, &rd, &odrag.gizmo_origin, &odrag.axis_dir_world);
                        let delta_t = t_line - odrag.initial_t;

                        if let Some(ref mut model) = self.model {
                            match odrag.target {
                                OffsetTarget::Visual => {
                                    let geom = &mut model.links[odrag.entity_idx]
                                        .visuals[odrag.sub_idx].geometry;
                                    apply_geom_scale(geom, odrag.axis, odrag.initial_geom_params, delta_t);
                                }
                                OffsetTarget::Collision => {
                                    let geom = &mut model.links[odrag.entity_idx]
                                        .collisions[odrag.sub_idx].geometry;
                                    apply_geom_scale(geom, odrag.axis, odrag.initial_geom_params, delta_t);
                                }
                                OffsetTarget::Joint => {}
                            }
                            self.needs_upload = true;
                        }
                    }
                }
            }
        } else if let Some(ref drag) = self.drag_state.clone() {
            let delta = response.drag_delta();
            if delta.length_sq() > 0.0 {
                match drag.mode {
                    DragMode::SingleJoint => {
                        if let Some(pos) = response.hover_pos() {
                            let axis = drag.world_axis.normalize();
                            let pivot = drag.pivot_world;

                            let prev_ndc = na::Point2::new(
                                (pos.x - delta.x - rect.left()) / rect.width(),
                                (pos.y - delta.y - rect.top()) / rect.height(),
                            );
                            let curr_ndc = na::Point2::new(
                                (pos.x - rect.left()) / rect.width(),
                                (pos.y - rect.top()) / rect.height(),
                            );

                            let (ro0, rd0) = self.camera.screen_ray(prev_ndc, aspect);
                            let (ro1, rd1) = self.camera.screen_ray(curr_ndc, aspect);

                            let ray_plane_hit =
                                |ro: &na::Point3<f32>,
                                 rd: &na::Vector3<f32>|
                                 -> Option<na::Point3<f32>> {
                                    let denom = rd.dot(&axis);
                                    if denom.abs() < 1e-7 {
                                        return None;
                                    }
                                    let t = (pivot - ro).dot(&axis) / denom;
                                    Some(ro + rd * t)
                                };

                            let angle_delta = match (
                                ray_plane_hit(&ro0, &rd0),
                                ray_plane_hit(&ro1, &rd1),
                            ) {
                                (Some(p0), Some(p1)) => {
                                    let v0 = p0 - pivot;
                                    let v1 = p1 - pivot;
                                    if v0.norm() < 1e-8 || v1.norm() < 1e-8 {
                                        0.0
                                    } else {
                                        let v0n = v0.normalize();
                                        let v1n = v1.normalize();
                                        let cross = v0n.cross(&v1n);
                                        let dot = v0n.dot(&v1n).clamp(-1.0, 1.0);
                                        cross.dot(&axis).atan2(dot)
                                    }
                                }
                                _ => {
                                    let delta_ndc = na::Vector2::new(
                                        delta.x / rect.width(),
                                        delta.y / rect.height(),
                                    );
                                    delta_ndc.norm() * 3.0 * delta.x.signum()
                                }
                            };

                            if angle_delta.abs() > 1e-8 {
                                if let Some(ref mut model) = self.model {
                                    let ji = drag.joint_idx;
                                    let lower = model.joints[ji].lower;
                                    let upper = model.joints[ji].upper;
                                    model.joint_positions[ji] =
                                        (model.joint_positions[ji] + angle_delta as f64)
                                            .clamp(lower, upper);
                                }
                            }
                        }
                    }
                    DragMode::InverseKinematics => {
                        if let (Some(ndc), Some(ref mut model)) =
                            (mouse_ndc, self.model.as_mut())
                        {
                            let cur_transforms = model.compute_transforms();
                            let ee_pos = model.ee_world_pos(
                                drag.link_idx,
                                &cur_transforms,
                            );

                            let ik_root_tf_desired = drag.ik_root_initial_tf;

                            let (ray_o, ray_d) = self.camera.screen_ray(ndc, aspect);
                            let cam_forward = (self.camera.target
                                - na::Point3::from(
                                    self.camera.eye().coords,
                                ))
                            .normalize();

                            let denom = ray_d.dot(&cam_forward);
                            if denom.abs() > 1e-6 {
                                let t = (ee_pos - ray_o).dot(&cam_forward) / denom;
                                if t > 0.0 {
                                    let target = ray_o + ray_d * t;

                                    let damping = self.ik_damping;
                                    let deltas = model.solve_ik_step(
                                        &drag.chain,
                                        &drag.ee_link,
                                        drag.ik_root_link.as_deref(),
                                        &ee_pos.cast::<f64>(),
                                        &target.cast::<f64>(),
                                        damping as f64,
                                        0.1,
                                    );
                                    model.apply_joint_deltas(&drag.chain, &deltas);

                                    if let Some(desired_tf) = ik_root_tf_desired {
                                        let saved_base = model.base_transform;
                                        model.base_transform = na::Isometry3::identity();
                                        let identity_transforms = model.compute_transforms();
                                        if let Some(ik_root_tf_rel) = drag.ik_root_link.as_ref()
                                            .and_then(|name| identity_transforms.get(name))
                                        {
                                            model.base_transform = ik_root_tf_rel.inverse().cast::<f64>();
                                            model.base_transform = desired_tf.cast::<f64>() * model.base_transform;
                                        } else {
                                            model.base_transform = saved_base;
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        } else {
            // No drag state: left-drag orbits camera
            self.camera.handle_orbit_pan_zoom(response);
        }
    }

    /// Update cursor icon based on hover/drag state.
    fn update_cursor(&self, ui: &mut egui::Ui) {
        if self.drag_state.is_some() || self.offset_drag_state.is_some() {
            ui.ctx().set_cursor_icon(egui::CursorIcon::Grabbing);
        } else if self.hovered_gizmo_axis.is_some() {
            ui.ctx().set_cursor_icon(egui::CursorIcon::Grab);
        } else if self.hovered_link.is_some() {
            let is_draggable = self.hovered_link.and_then(|li| {
                self.model.as_ref().and_then(|m| {
                    let link_name = &m.links[li].name;
                    match self.interaction_mode {
                        InteractionMode::OffsetAdjust => {
                            Some(&m.links[li].name as &str)
                        }
                        InteractionMode::JointDrive => match self.drag_mode {
                            DragMode::SingleJoint => m
                                .parent_joint_of_link(link_name)
                                .map(|ji| &m.joints[ji].joint_type)
                                .filter(|jt| *jt == "revolute" || *jt == "continuous")
                                .map(|s| s.as_str()),
                            DragMode::InverseKinematics => {
                                let chain = m.chain_joints(link_name);
                                if chain.is_empty() {
                                    None
                                } else {
                                    Some(m.joints[chain[0]].joint_type.as_str())
                                }
                            }
                        },
                    }
                })
            });
            if is_draggable.is_some() {
                ui.ctx().set_cursor_icon(egui::CursorIcon::Grab);
            }
        }
    }
}

// ===== Free helper functions =====

/// Compute the closest distance from a ray to a circle (ring) in 3D.
fn ray_ring_distance(
    ray_o: &na::Point3<f32>,
    ray_d: &na::Vector3<f32>,
    center: &na::Point3<f32>,
    normal: &na::Vector3<f32>,
    radius: f32,
) -> f32 {
    let denom = ray_d.dot(normal);
    if denom.abs() < 1e-8 {
        let to_origin = ray_o - center;
        let plane_dist = to_origin.dot(normal);
        let proj = ray_o - normal * plane_dist;
        let from_center = proj - center;
        let r_proj = from_center.norm();
        if r_proj < 1e-8 {
            return (plane_dist * plane_dist + radius * radius).sqrt();
        }
        let closest_on_ring = center + from_center * (radius / r_proj);
        let to_ring = closest_on_ring - ray_o;
        let t = to_ring.dot(ray_d) / ray_d.dot(ray_d);
        let closest_on_ray = ray_o + ray_d * t.max(0.0);
        na::distance(&closest_on_ray, &closest_on_ring)
    } else {
        let t = (center - ray_o).dot(normal) / denom;
        let hit = ray_o + ray_d * t;
        let from_center = hit - center;
        let r_hit = from_center.norm();
        if r_hit < 1e-8 {
            return radius;
        }
        (r_hit - radius).abs()
    }
}

/// Extract geometry parameters as [f32; 3] for scale dragging.
fn geom_params(geom: &robot::GeomData) -> [f32; 3] {
    match geom {
        robot::GeomData::Box { hx, hy, hz } => [*hx, *hy, *hz],
        robot::GeomData::Cylinder { radius, half_length } => [*radius, *half_length, 0.0],
        robot::GeomData::Sphere { radius } => [*radius, 0.0, 0.0],
        robot::GeomData::Capsule { radius, half_length } => [*radius, *half_length, 0.0],
        robot::GeomData::Mesh { .. } => [0.0, 0.0, 0.0],
    }
}

/// Apply a per-axis scale delta to geometry.
fn apply_geom_scale(geom: &mut robot::GeomData, axis: u8, initial: [f32; 3], delta: f32) {
    const MIN_DIM: f32 = 0.001;
    match geom {
        robot::GeomData::Box { hx, hy, hz } => {
            match axis {
                0 => *hx = (initial[0] + delta).max(MIN_DIM),
                1 => *hy = (initial[1] + delta).max(MIN_DIM),
                _ => *hz = (initial[2] + delta).max(MIN_DIM),
            }
        }
        robot::GeomData::Cylinder { radius, half_length } => {
            match axis {
                0 | 1 => *radius = (initial[0] + delta).max(MIN_DIM),
                _ => *half_length = (initial[1] + delta).max(MIN_DIM),
            }
        }
        robot::GeomData::Sphere { radius } => {
            *radius = (initial[0] + delta).max(MIN_DIM);
        }
        robot::GeomData::Capsule { radius, half_length } => {
            match axis {
                0 | 1 => *radius = (initial[0] + delta).max(MIN_DIM),
                _ => *half_length = (initial[1] + delta).max(MIN_DIM),
            }
        }
        robot::GeomData::Mesh { .. } => {}
    }
}

/// Compute the angle of a ray's intersection with a ring's plane.
fn compute_ring_angle(
    ray_o: &na::Point3<f32>,
    ray_d: &na::Vector3<f32>,
    center: &na::Point3<f32>,
    normal: &na::Vector3<f32>,
) -> f32 {
    let denom = ray_d.dot(normal);
    if denom.abs() < 1e-8 {
        return 0.0;
    }
    let t = (center - ray_o).dot(normal) / denom;
    let hit = ray_o + ray_d * t;
    let from_center = hit - center;

    let ref_x = if normal.x.abs() < 0.9 {
        na::Vector3::x().cross(normal).normalize()
    } else {
        na::Vector3::y().cross(normal).normalize()
    };
    let ref_y = normal.cross(&ref_x);

    from_center.dot(&ref_y).atan2(from_center.dot(&ref_x))
}
