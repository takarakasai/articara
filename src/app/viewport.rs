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

        // Loop-closure "Pick B" mode: the next left-click consumes the
        // cursor as the link-B selector instead of the usual drag/drive
        // flow. We have to intercept on BOTH `drag_started_by` (when the
        // user begins a drag) AND `clicked()` (a clean press-release
        // without crossing the drag threshold) — egui only fires one of
        // them per interaction, so checking just `drag_started_by` would
        // miss clean clicks and let the regular handler overwrite link A.
        //
        // `pick_b_active_this_frame` short-circuits the regular click/drag
        // handlers below for the entire frame, so the press half of a
        // press-release pair doesn't accidentally start a JointDrive drag
        // before the release lands and resolves the pick.
        let pick_b_active_this_frame = self.loop_closure_picking_b;
        if self.loop_closure_picking_b {
            let pressed = response.drag_started_by(egui::PointerButton::Primary);
            let clicked = response.clicked();
            if pressed || clicked {
                self.handle_pick_b_click(mouse_ndc, aspect, &transforms);
            }
        }

        // Left mouse button pressed: start drag
        let had_drag_before = self.drag_state.is_some() || self.offset_drag_state.is_some();
        if !pick_b_active_this_frame
            && response.drag_started_by(egui::PointerButton::Primary)
        {
            // Sim drag takes precedence when MuJoCo is running so the user
            // can poke the live robot without the editor's joint-drive logic
            // mutating angles directly.
            #[cfg(feature = "mujoco")]
            let sim_drag_started =
                self.handle_sim_drag_start(mouse_ndc, aspect, &transforms);
            #[cfg(not(feature = "mujoco"))]
            let sim_drag_started = false;
            if !sim_drag_started {
                self.handle_drag_start(&response, mouse_ndc, aspect, &transforms, gizmo_tf);
            }
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
        if !pick_b_active_this_frame && response.clicked() {
            self.handle_click(mouse_ndc, aspect, &transforms);
        }

        // While dragging: handle joint drive or offset adjustment.
        // When a sim-drag is active, route there exclusively so the
        // existing JointDrive update doesn't compete for the same button.
        if response.dragged_by(egui::PointerButton::Primary) {
            #[cfg(feature = "mujoco")]
            let sim_drag_active = self.sim_drag_state.is_some();
            #[cfg(not(feature = "mujoco"))]
            let sim_drag_active = false;
            if sim_drag_active {
                #[cfg(feature = "mujoco")]
                self.handle_sim_drag_update(mouse_ndc, aspect, &transforms);
            } else {
                self.handle_drag_update(&response, mouse_ndc, rect, aspect);
            }
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
            // Remove chicken-head auto-pins (those whose link name
            // appears in chicken_head_links but not manually pinned).
            if !self.chicken_head_links.is_empty() {
                let ch_set: std::collections::HashSet<&str> = self
                    .chicken_head_links
                    .iter()
                    .map(|s| s.as_str())
                    .collect();
                self.pinned_links
                    .retain(|p| !ch_set.contains(p.link_name.as_str()));
            }
            self.drag_state = None;
            self.offset_drag_state = None;
            #[cfg(feature = "mujoco")]
            self.handle_sim_drag_end();
            self.ik_target_marker = None;
            self.ik_ee_marker = None;
            self.ik_error = None;
            self.history.finalize();
        }

        // Update highlight and gizmo state in renderer.
        // Priority: drag > hover > selected. Falling through to `selected_link`
        // means the user keeps a visual on the link they last picked even
        // when the cursor is somewhere else (e.g. dragging a Properties
        // panel slider) — without it the highlight disappeared the instant
        // the mouse left the viewport.
        {
            let mut r = self.gl_renderer.lock().unwrap();
            let highlight = if self.drag_state.is_some() {
                self.drag_state
                    .as_ref()
                    .and_then(|d| self.model.as_ref().map(|m| m.links[d.link_idx].name.clone()))
            } else if let Some(li) = self.hovered_link {
                self.model.as_ref().map(|m| m.links[li].name.clone())
            } else {
                self.selected_link
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

        // ===== Picture-in-picture wipe (alternate camera) =====
        // Render the *other* camera (free / tps, opposite of the active
        // mode) into a small rect at the top-right of the viewport when
        // `show_camera_wipe` is enabled. The renderer's own scissor +
        // viewport calls keep its draws inside the sub-rect, so the
        // main viewport's content underneath is preserved.
        if self.show_camera_wipe {
            use crate::camera::CameraMode;
            let alt_camera = match self.camera_mode {
                CameraMode::Free => self.tps_camera.clone(),
                CameraMode::Tps => self.saved_free_camera.clone(),
            };
            let alt_label = match self.camera_mode {
                CameraMode::Free => "TPS",
                CameraMode::Tps => "Free",
            };
            // Sub-rect: 25% of width × ~28% of height (4:3-ish), pinned
            // to the top-right corner with an 8 px margin.
            let wipe_w = (rect.width() * 0.28).max(180.0).min(360.0);
            let wipe_h = wipe_w * 0.72;
            let margin = 8.0;
            let wipe_rect = egui::Rect::from_min_size(
                egui::pos2(
                    rect.right() - wipe_w - margin,
                    rect.top() + margin,
                ),
                egui::vec2(wipe_w, wipe_h),
            );
            // Frame outline so the wipe stands out from the background.
            ui.painter().rect(
                wipe_rect,
                4.0,
                egui::Color32::from_rgba_unmultiplied(20, 20, 30, 220),
                egui::Stroke::new(1.0, egui::Color32::from_gray(180)),
                egui::StrokeKind::Inside,
            );
            // Label inside the wipe.
            ui.painter().text(
                wipe_rect.left_top() + egui::vec2(4.0, 2.0),
                egui::Align2::LEFT_TOP,
                alt_label,
                egui::FontId::monospace(10.0),
                egui::Color32::from_gray(220),
            );
            // GL paint callback for the wipe — same pattern as the main
            // viewport, but with the alt camera and sub-rect.
            let renderer = self.gl_renderer.clone();
            let wipe_callback = egui::PaintCallback {
                rect: wipe_rect,
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
                        renderer.render(gl, &alt_camera, gl_viewport);
                    },
                )),
            };
            ui.painter().add(wipe_callback);
        }

        // ===== Viewport overlays =====
        self.draw_viewport_overlay(ui, rect);
        self.draw_com_labels(ui, rect, aspect);
        self.draw_ik_root_anchor(ui, rect, aspect);
        self.draw_ik_pin_markers(ui, rect, aspect);
        self.draw_ik_target_marker(ui, rect, aspect);
        self.draw_ik_ee_marker(ui, rect, aspect);
        self.draw_camera_axes(ui, rect);
        self.draw_gravity_indicator(ui, rect);
        self.draw_selection_markers(ui, rect, aspect);
        #[cfg(feature = "mujoco")]
        {
            self.draw_contact_markers(ui, rect, aspect);
            self.draw_force_pulse_markers(ui, rect, aspect);
            self.draw_sim_drag_overlay(ui, rect, aspect);
            self.draw_imu_attitude_overlay(ui, rect, aspect);
            self.draw_imu_vibration_overlay(ui, rect);
        }
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
                    // Try precise ray-pick first; fall back to hovered link
                    // so that a near-miss drag doesn't orbit the camera.
                    // Only fall back when the cursor is visually over a link
                    // (hovered_link), NOT when clicking empty space.
                    let pick_result = model.pick_link(&ro, &rd, transforms);
                    let (li, hit_dist) = if let Some(hit) = pick_result {
                        hit
                    } else if let Some(hov_li) = self.hovered_link {
                        // Use the bounding-sphere center distance as an
                        // approximate hit distance for the fallback.
                        let link_tf = transforms
                            .get(&model.links[hov_li].name)
                            .copied()
                            .unwrap_or(na::Isometry3::identity());
                        let (center, _) = model.link_bounding_sphere(hov_li);
                        let world_center = link_tf * center;
                        let approx_dist = (world_center - ro).dot(&rd).max(0.01);
                        (hov_li, approx_dist)
                    } else {
                        // Nothing selected and no hit — orbit camera
                        return;
                    };
                    {
                        let link_name = &model.links[li].name;
                        self.selected_link = Some(li);
                        self.selected_joint = model.parent_joint_of_link(link_name);
                        self.tree_reveal_ancestors = model.ancestor_links(link_name);
                        match self.drag_mode {
                            DragMode::SingleJoint => {
                                // Walk up the kinematic tree from the clicked
                                // link to find the nearest revolute/continuous
                                // ancestor joint.
                                let mut cur_link = link_name.to_string();
                                let mut found_ji = None;
                                loop {
                                    if let Some(ji) = model.parent_joint_of_link(&cur_link) {
                                        let joint = &model.joints[ji];
                                        if joint.joint_type == "revolute"
                                            || joint.joint_type == "continuous"
                                        {
                                            found_ji = Some(ji);
                                            break;
                                        }
                                        // Not movable — continue up to parent link
                                        cur_link = joint.parent_link.clone();
                                    } else {
                                        break; // reached root
                                    }
                                }
                                if let Some(ji) = found_ji {
                                    let joint = &model.joints[ji];
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
                                        ik_root_initial_pos: None,
                                        ref_positions: Vec::new(),
                                        drag_depth: 0.0,
                                        ee_local_offset: na::Point3::origin(),
                                    });
                                }
                            }
                            DragMode::InverseKinematics => {
                                let chain = model.chain_joints_between(
                                    link_name,
                                    self.ik_root_link.as_deref(),
                                );
                                // Allow drag even with empty chain when pins,
                                // chicken-head links, or loop closures exist,
                                // because the constrained solver works in full
                                // joint space.
                                let has_constraints = !self.pinned_links.is_empty()
                                    || !self.chicken_head_links.is_empty()
                                    || !model.loop_closures.is_empty();
                                if !chain.is_empty() || has_constraints {
                                    let ji =
                                        *chain.last().unwrap_or(&0);
                                    let ik_root_tf = self.ik_root_link.as_ref().and_then(|name| {
                                        transforms.get(name).copied()
                                    });
                                    // Capture reference posture for null-space stabilization
                                    let ref_positions: Vec<f64> = chain.iter()
                                        .map(|&ji| model.joint_positions[ji])
                                        .collect();
                                    // Capture IK root world position for translation-only correction
                                    let ik_root_initial_pos = ik_root_tf.map(|tf| {
                                        na::Point3::from(tf.translation.vector).cast::<f64>()
                                    });
                                    // Compute EE local offset from the clicked surface point
                                    let hit_world = ro + rd * hit_dist;
                                    let link_world_tf = transforms
                                        .get(link_name)
                                        .copied()
                                        .unwrap_or(na::Isometry3::identity());
                                    let ee_local_offset = link_world_tf.inverse() * hit_world;
                                    // Use clicked surface point for drag depth
                                    let cam_fwd = (self.camera.target
                                        - na::Point3::from(self.camera.eye().coords))
                                        .normalize();
                                    let drag_depth = (hit_world - self.camera.eye()).dot(&cam_fwd);

                                    // Chicken-head: auto-pin designated links at their current poses
                                    for ch_link in &self.chicken_head_links {
                                        // Skip if already manually pinned
                                        if self.pinned_links.iter().any(|p| p.link_name == *ch_link) {
                                            continue;
                                        }
                                        // Skip if this is the link being dragged
                                        if ch_link == link_name {
                                            continue;
                                        }
                                        if let Some(&ch_li) = model.link_map.get(ch_link.as_str()) {
                                            let pos = model.ee_world_pos(ch_li, transforms).cast::<f64>();
                                            let rot = model.link_world_orientation(ch_li, transforms).cast::<f64>();
                                            self.pinned_links.push(super::PinnedLink {
                                                link_name: ch_link.clone(),
                                                target_pos: pos,
                                                target_rot: rot,
                                                dof: self.chicken_head_dof,
                                            });
                                        }
                                    }

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
                                        ik_root_initial_pos,
                                        ref_positions,
                                        drag_depth,
                                        ee_local_offset,
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
    /// Click handler used while [`Self::loop_closure_picking_b`] is on.
    /// Picks a link by ray-cast and stores it in
    /// [`Self::loop_closure_link_b`] without disturbing the regular
    /// `selected_link` (which is the loop closure's link A).
    fn handle_pick_b_click(
        &mut self,
        mouse_ndc: Option<na::Point2<f32>>,
        aspect: f32,
        transforms: &std::collections::HashMap<String, na::Isometry3<f32>>,
    ) {
        let Some(ndc) = mouse_ndc else { return };
        let Some(model) = self.model.as_ref() else {
            return;
        };
        let (ro, rd) = self.camera.screen_ray(ndc, aspect);
        if let Some((li, _)) = model.pick_link(&ro, &rd, transforms) {
            self.loop_closure_link_b = Some(li);
            self.status_message = format!(
                "Loop-closure link B = '{}'",
                model.links[li].name,
            );
        } else if let Some(hov_li) = self.hovered_link {
            // Fallback when the precise mesh test misses but the hover
            // pick succeeded.
            self.loop_closure_link_b = Some(hov_li);
            self.status_message = format!(
                "Loop-closure link B = '{}'",
                model.links[hov_li].name,
            );
        } else {
            self.status_message =
                "Loop-closure pick: no link under cursor".into();
        }
        // One-shot: turn off picking mode after a successful or failed click.
        self.loop_closure_picking_b = false;
    }

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
                            let _ee_pos = model.ee_world_pos_at(
                                drag.link_idx,
                                &cur_transforms,
                                &drag.ee_local_offset,
                            );

                            let _ik_root_tf_desired = drag.ik_root_initial_tf;

                            let (ray_o, ray_d) = self.camera.screen_ray(ndc, aspect);
                            let cam_forward = (self.camera.target
                                - na::Point3::from(
                                    self.camera.eye().coords,
                                ))
                            .normalize();

                            let denom = ray_d.dot(&cam_forward);
                            if denom.abs() > 1e-6 {
                                // Use fixed depth from drag start instead of
                                // current ee_pos to prevent target jumps.
                                let t = drag.drag_depth / denom;
                                if t > 0.0 {
                                    let target = ray_o + ray_d * t;
                                    let target_f64 = target.cast::<f64>();

                                    // Store target for debug overlay
                                    self.ik_target_marker = Some(target);
                                    // Will compute IK error after solve loop

                                    let damping = self.ik_damping as f64;

                                    // Differential IK: apply one small velocity-level
                                    // update per frame with proportional gain.
                                    // gain < 1 ensures smooth convergence over frames.
                                    let ik_gain = 0.3_f64;
                                    let max_joint_step = 0.15_f64; // rad per frame safety clamp

                                    let cur_tf = model.compute_transforms();
                                    let ee_now = model.ee_world_pos_at(
                                        drag.link_idx, &cur_tf,
                                        &drag.ee_local_offset,
                                    );
                                    // Compute world-frame offset vector for Jacobian correction.
                                    // r_world = R_link * r_local (vector from link origin to click point)
                                    let ee_offset_world = {
                                        let link_rot = model.link_world_orientation(
                                            drag.link_idx, &cur_tf,
                                        );
                                        let r_local = drag.ee_local_offset.coords.cast::<f64>();
                                        let r_world = link_rot.cast::<f64>() * r_local;
                                        r_world
                                    };
                                    // Compute camera right/up for 2-DoF screen-plane mode.
                                    // Pull the basis directly from the view
                                    // matrix so the projection axes match
                                    // whatever the user actually sees — the
                                    // previous `cam_fwd × world_up` form
                                    // could fall out of sync once the camera
                                    // was pitched / rolled, leading to IK
                                    // updates that pushed the link slightly
                                    // off-axis from the cursor (which read
                                    // as "wrong direction" for non-tip
                                    // links because their click points sit
                                    // off the joint origin).
                                    let screen_axes = if self.ik_dof == crate::robot::IkDof::ScreenPlane2D {
                                        let cam_right = self.camera.world_right().cast::<f64>();
                                        let cam_up = self.camera.world_up_screen().cast::<f64>();
                                        Some((cam_right, cam_up))
                                    } else {
                                        None
                                    };

                                    // Compute per-joint cost weights: joints far from EE
                                    // are more expensive to move.
                                    // w_i = α^(n-1-i), i=n-1 (EE joint) → 1, i=0 (root) → α^(n-1)
                                    let weights = if self.ik_weight_gradient > 0.01 {
                                        let n = drag.chain.len();
                                        let alpha = (1.0 + self.ik_weight_gradient as f64).max(1.0);
                                        let w: Vec<f64> = (0..n)
                                            .map(|i| alpha.powi((n - 1 - i) as i32))
                                            .collect();
                                        Some(w)
                                    } else {
                                        None
                                    };

                                    // Build loop-closure constraints (empty if none defined)
                                    let loop_constraints = model.build_loop_diff_constraints(
                                        self.loop_closure_weight as f64,
                                    );
                                    let has_loops = !loop_constraints.is_empty();

                                    // Use constrained IK when pins or loop closures exist
                                    if !self.pinned_links.is_empty() || has_loops {
                                        let pins: Vec<crate::robot::PinSpec> = self.pinned_links
                                            .iter()
                                            .map(|p| crate::robot::PinSpec {
                                                link_name: p.link_name.clone(),
                                                target_pos: p.target_pos,
                                                target_rot: p.target_rot,
                                                pose_6dof: p.dof == super::PinDof::Pose,
                                            })
                                            .collect();

                                        let is_root_drag = drag.chain.is_empty();
                                        if is_root_drag && !self.pinned_links.is_empty() {
                                            // Dragged link is at or near the root: move
                                            // base_transform directly, then solve pin
                                            // + loop constraints via joints.
                                            let dx = (target_f64 - ee_now.cast::<f64>()) * ik_gain;
                                            let dx_clamped = {
                                                let len = dx.norm();
                                                if len > max_joint_step {
                                                    dx * (max_joint_step / len)
                                                } else {
                                                    dx
                                                }
                                            };
                                            model.base_transform.translation.vector +=
                                                dx_clamped;

                                            // Solve pin + loop constraints only (no primary
                                            // task, just keep constraints satisfied).
                                            let mc = model.mc();
                                            let nv = mc.model.nv;
                                            if nv > 0 && (!pins.is_empty() || has_loops) {
                                                let dummy_jac =
                                                    na::DMatrix::<f64>::zeros(3, nv);
                                                let zero = na::Vector3::zeros();

                                                let misarta_damping = match self.ik_solver {
                                                    crate::robot::IkSolver::SrInverse => {
                                                        misarta::ik::Damping::AdaptiveManipulability {
                                                            lambda_min: 0.0,
                                                            lambda_max: damping,
                                                            manipulability_threshold: 0.05,
                                                        }
                                                    }
                                                    _ => misarta::ik::Damping::Fixed(damping),
                                                };
                                                let misarta_method = match self.ik_solver {
                                                    crate::robot::IkSolver::JacobianTranspose => {
                                                        misarta::ik::SolverMethod::JacobianTranspose
                                                    }
                                                    _ => {
                                                        misarta::ik::SolverMethod::DampedLeastSquares
                                                    }
                                                };

                                                // Build pin constraints from post-move positions
                                                let post_tf = model.compute_transforms();
                                                let mut constraints: Vec<misarta::ik::DiffIkConstraint> = Vec::new();
                                                for pin in &pins {
                                                    if let Some(&li) =
                                                        model.link_map.get(pin.link_name.as_str())
                                                    {
                                                        if pin.pose_6dof {
                                                            let jac = model.link_full_jacobian_full(
                                                                &pin.link_name,
                                                            );
                                                            let pin_world = model
                                                                .ee_world_pos(li, &post_tf)
                                                                .cast::<f64>();
                                                            let pin_rot = model
                                                                .link_world_orientation(li, &post_tf)
                                                                .cast::<f64>();
                                                            let pos_err = pin_world - pin.target_pos;
                                                            let rot_err = (pin_rot
                                                                * pin.target_rot.inverse())
                                                                .scaled_axis();
                                                            constraints.push(
                                                                misarta::ik::DiffIkConstraint {
                                                                    jacobian: jac,
                                                                    error: na::DVector::from_column_slice(
                                                                        &[
                                                                            rot_err.x, rot_err.y,
                                                                            rot_err.z, pos_err.x,
                                                                            pos_err.y, pos_err.z,
                                                                        ],
                                                                    ),
                                                                    weight: self.ik_pin_weight as f64,
                                                                },
                                                            );
                                                        } else {
                                                            let jac_pin = model
                                                                .link_positional_jacobian_full(
                                                                    &pin.link_name,
                                                                    None,
                                                                );
                                                            let pin_world = model
                                                                .ee_world_pos(li, &post_tf)
                                                                .cast::<f64>();
                                                            let err = pin_world - pin.target_pos;
                                                            constraints.push(
                                                                misarta::ik::DiffIkConstraint {
                                                                    jacobian: jac_pin,
                                                                    error: na::DVector::from_column_slice(
                                                                        &[err.x, err.y, err.z],
                                                                    ),
                                                                    weight: self.ik_pin_weight as f64,
                                                                },
                                                            );
                                                        }
                                                    }
                                                }

                                                // Add loop-closure constraints (recomputed after base move)
                                                let loop_cs_post = model.build_loop_diff_constraints(
                                                    self.loop_closure_weight as f64,
                                                );
                                                constraints.extend(loop_cs_post);

                                                let diff_config = misarta::ik::DiffIkConfig {
                                                    gain: 1.0, // full correction
                                                    max_joint_step,
                                                    damping: misarta_damping,
                                                    solver_method: misarta_method,
                                                    joint_weights: None,
                                                    task_projection: None,
                                                };

                                                let result =
                                                    misarta::ik::differential_ik_step_with_constraints(
                                                        &dummy_jac, &zero, &zero,
                                                        &constraints, &diff_config,
                                                    );

                                                // Map full-nv deltas to articara joints
                                                let mc = model.mc();
                                                let mut deltas =
                                                    vec![0.0_f64; model.joint_positions.len()];
                                                for (ji, maybe_mi) in mc.a2m.iter().enumerate() {
                                                    if let Some(mi) = maybe_mi {
                                                        let vi = mc.model.q_idx[*mi];
                                                        if vi < result.dq.len() {
                                                            deltas[ji] = result.dq[vi];
                                                        }
                                                    }
                                                }
                                                model.apply_all_joint_deltas(&deltas);
                                            }
                                        } else {
                                            // Non-root drag with pins/loops: augmented Jacobian
                                            let mc = model.mc();
                                            let nv = mc.model.nv;
                                            let full_weights = if self.ik_weight_gradient > 0.01 {
                                                let alpha = (1.0 + self.ik_weight_gradient as f64).max(1.0);
                                                let mut w = vec![alpha.powi(drag.chain.len().max(1) as i32 - 1); nv];
                                                for (i, &ji) in drag.chain.iter().enumerate() {
                                                    if let Some(&Some(mi)) = mc.a2m.get(ji) {
                                                        let vi = mc.model.q_idx[mi];
                                                        if vi < nv {
                                                            w[vi] = alpha.powi((drag.chain.len() - 1 - i) as i32);
                                                        }
                                                    }
                                                }
                                                Some(w)
                                            } else {
                                                None
                                            };

                                            let deltas = model.solve_ik_step_with_pins(
                                                &drag.ee_link,
                                                &ee_now.cast::<f64>(),
                                                &target_f64,
                                                &pins,
                                                damping,
                                                ik_gain,
                                                max_joint_step,
                                                self.ik_solver,
                                                screen_axes,
                                                full_weights.as_deref(),
                                                self.ik_pin_weight as f64,
                                                &loop_constraints,
                                                Some(&ee_offset_world),
                                            );
                                            model.apply_all_joint_deltas(&deltas);
                                        }
                                    } else {
                                        let deltas = model.solve_ik_step(
                                            &drag.chain,
                                            &drag.ee_link,
                                            drag.ik_root_link.as_deref(),
                                            &ee_now.cast::<f64>(),
                                            &target_f64,
                                            damping,
                                            ik_gain,
                                            max_joint_step,
                                            None,
                                            self.ik_solver,
                                            screen_axes,
                                            weights.as_deref(),
                                            Some(&ee_offset_world),
                                        );
                                        model.apply_joint_deltas(&drag.chain, &deltas);
                                    }

                                    // Base correction: pin the IK root to its initial pose.
                                    // Skip when using pinned-link mode with root drag
                                    // (base_transform was moved intentionally).
                                    let skip_base_correction = !self.pinned_links.is_empty()
                                        && drag.chain.is_empty();
                                    if !skip_base_correction {
                                        if let Some(desired_tf) = drag.ik_root_initial_tf {
                                            if let Some(ik_root_name) = drag.ik_root_link.as_ref() {
                                                model.base_transform = na::Isometry3::identity();
                                                let id_tf = model.compute_transforms();
                                                if let Some(root_rel) = id_tf.get(ik_root_name) {
                                                    model.base_transform =
                                                        desired_tf.cast::<f64>()
                                                        * root_rel.inverse().cast::<f64>();
                                                }
                                            }
                                        }
                                    }

                                    // Compute IK error for debug overlay
                                    let final_tf = model.compute_transforms();
                                    let ee_final = model.ee_world_pos_at(
                                        drag.link_idx, &final_tf,
                                        &drag.ee_local_offset,
                                    );
                                    self.ik_ee_marker = Some(ee_final);
                                    self.ik_error = Some(
                                        (ee_final.cast::<f32>() - target).norm()
                                    );
                                }
                            }
                        }
                    }
                }
            }
        } else {
            // No drag state: left-drag orbits camera. Dispatch by mode
            // so TPS mode redirects yaw/pitch/scroll to the follow
            // settings instead of the (now-derived) main `self.camera`.
            self.handle_camera_input(response);
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
                                if chain.is_empty() && self.pinned_links.is_empty() {
                                    None
                                } else if chain.is_empty() {
                                    // Pinned mode: allow drag even on root
                                    Some("pinned_ik")
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
