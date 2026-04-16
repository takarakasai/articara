use eframe::egui;
use nalgebra as na;

use crate::renderer::DisplayMode;

use super::{ArticaraApp, DragMode, GizmoOp, InteractionMode, OffsetTarget};

impl ArticaraApp {
    /// Draw the mode toolbar overlay on the viewport.
    pub(super) fn draw_viewport_overlay(&mut self, ui: &mut egui::Ui, rect: egui::Rect) {
        let painter = ui.painter();
        let icon_size = egui::vec2(32.0, 32.0);
        let margin = 8.0;
        let toolbar_x = rect.left() + margin;
        let toolbar_y = rect.top() + margin;

        // --- Row 1: Interaction Mode buttons ---
        let modes: [(InteractionMode, &str); 2] = [
            (InteractionMode::JointDrive, "Joint Drive"),
            (InteractionMode::OffsetAdjust, "Offset Adjust"),
        ];

        for (i, (mode, tooltip)) in modes.iter().enumerate() {
            let btn_pos = egui::pos2(
                toolbar_x + i as f32 * (icon_size.x + 4.0),
                toolbar_y,
            );
            let btn_rect = egui::Rect::from_min_size(btn_pos, icon_size);
            let is_active = self.interaction_mode == *mode;

            let btn_response = ui.interact(
                btn_rect,
                ui.id().with("mode_btn").with(i),
                egui::Sense::click(),
            );

            let bg_color = if is_active {
                egui::Color32::from_rgba_unmultiplied(60, 130, 220, 200)
            } else if btn_response.hovered() {
                egui::Color32::from_rgba_unmultiplied(80, 80, 80, 180)
            } else {
                egui::Color32::from_rgba_unmultiplied(40, 40, 50, 160)
            };
            painter.rect_filled(btn_rect, egui::CornerRadius::same(4), bg_color);

            if is_active {
                painter.rect_stroke(
                    btn_rect,
                    egui::CornerRadius::same(4),
                    egui::Stroke::new(2.0, egui::Color32::from_rgb(100, 180, 255)),
                    egui::StrokeKind::Outside,
                );
            }

            let icon_color = if is_active {
                egui::Color32::WHITE
            } else {
                egui::Color32::from_gray(200)
            };

            // Draw custom icon
            let c = btn_rect.center();
            match mode {
                InteractionMode::JointDrive => {
                    self.draw_joint_drive_icon(painter, c, bg_color, icon_color, is_active);
                }
                InteractionMode::OffsetAdjust => {
                    self.draw_offset_adjust_icon(painter, c, bg_color, icon_color, is_active);
                }
            }

            if btn_response.clicked() {
                self.interaction_mode = *mode;
            }
            if btn_response.hovered() {
                let tip_pos = egui::pos2(btn_rect.left(), btn_rect.bottom() + 4.0);
                painter.text(
                    tip_pos,
                    egui::Align2::LEFT_TOP,
                    *tooltip,
                    egui::FontId::proportional(12.0),
                    egui::Color32::from_gray(220),
                );
            }
        }

        // --- Row 2: Target buttons (only in OffsetAdjust mode) ---
        if self.interaction_mode == InteractionMode::OffsetAdjust {
            let row2_y = toolbar_y + icon_size.y + 4.0;
            let small_size = egui::vec2(28.0, 28.0);

            for (i, target) in OffsetTarget::ALL.iter().enumerate() {
                let btn_pos = egui::pos2(
                    toolbar_x + i as f32 * (small_size.x + 3.0),
                    row2_y,
                );
                let btn_rect = egui::Rect::from_min_size(btn_pos, small_size);
                let is_active = self.offset_target == *target;

                let btn_response = ui.interact(
                    btn_rect,
                    ui.id().with("target_btn").with(i),
                    egui::Sense::click(),
                );

                let bg_color = if is_active {
                    egui::Color32::from_rgba_unmultiplied(60, 180, 100, 200)
                } else if btn_response.hovered() {
                    egui::Color32::from_rgba_unmultiplied(80, 80, 80, 180)
                } else {
                    egui::Color32::from_rgba_unmultiplied(40, 40, 50, 160)
                };
                painter.rect_filled(btn_rect, egui::CornerRadius::same(3), bg_color);

                if is_active {
                    painter.rect_stroke(
                        btn_rect,
                        egui::CornerRadius::same(3),
                        egui::Stroke::new(2.0, egui::Color32::from_rgb(80, 220, 130)),
                        egui::StrokeKind::Outside,
                    );
                }

                let text_color = if is_active {
                    egui::Color32::WHITE
                } else {
                    egui::Color32::from_gray(200)
                };
                painter.text(
                    btn_rect.center(),
                    egui::Align2::CENTER_CENTER,
                    target.icon(),
                    egui::FontId::proportional(15.0),
                    text_color,
                );

                if btn_response.clicked() {
                    self.offset_target = *target;
                    if *target == OffsetTarget::Joint && self.gizmo_op == GizmoOp::Scale {
                        self.gizmo_op = GizmoOp::Translate;
                    }
                }
                if btn_response.hovered() {
                    let tip_pos = egui::pos2(btn_rect.left(), btn_rect.bottom() + 4.0);
                    painter.text(
                        tip_pos,
                        egui::Align2::LEFT_TOP,
                        target.label(),
                        egui::FontId::proportional(12.0),
                        egui::Color32::from_gray(220),
                    );
                }
            }

            // --- Row 3: Gizmo op buttons (Translate / Rotate / Scale) ---
            let row3_y = row2_y + small_size.y + 4.0;
            let gizmo_ops: &[(GizmoOp, &str, &str)] = if self.offset_target != OffsetTarget::Joint {
                &[
                    (GizmoOp::Translate, "⬌", "Translate"),
                    (GizmoOp::Rotate, "↻", "Rotate"),
                    (GizmoOp::Scale, "⬡", "Scale"),
                ]
            } else {
                &[
                    (GizmoOp::Translate, "⬌", "Translate"),
                    (GizmoOp::Rotate, "↻", "Rotate"),
                ]
            };
            for (i, (op, icon, label)) in gizmo_ops.iter().enumerate() {
                let btn_pos = egui::pos2(
                    toolbar_x + i as f32 * (small_size.x + 3.0),
                    row3_y,
                );
                let btn_rect = egui::Rect::from_min_size(btn_pos, small_size);
                let is_active = self.gizmo_op == *op;

                let btn_response = ui.interact(
                    btn_rect,
                    ui.id().with("gizmo_op_btn").with(i),
                    egui::Sense::click(),
                );

                let bg_color = if is_active {
                    egui::Color32::from_rgba_unmultiplied(60, 120, 200, 200)
                } else if btn_response.hovered() {
                    egui::Color32::from_rgba_unmultiplied(80, 80, 80, 180)
                } else {
                    egui::Color32::from_rgba_unmultiplied(40, 40, 50, 160)
                };
                painter.rect_filled(btn_rect, egui::CornerRadius::same(3), bg_color);

                if is_active {
                    painter.rect_stroke(
                        btn_rect,
                        egui::CornerRadius::same(3),
                        egui::Stroke::new(2.0, egui::Color32::from_rgb(100, 160, 255)),
                        egui::StrokeKind::Outside,
                    );
                }

                let text_color = if is_active {
                    egui::Color32::WHITE
                } else {
                    egui::Color32::from_gray(200)
                };
                painter.text(
                    btn_rect.center(),
                    egui::Align2::CENTER_CENTER,
                    *icon,
                    egui::FontId::proportional(15.0),
                    text_color,
                );

                if btn_response.clicked() {
                    self.gizmo_op = *op;
                }
                if btn_response.hovered() {
                    let tip_pos = egui::pos2(btn_rect.left(), btn_rect.bottom() + 4.0);
                    painter.text(
                        tip_pos,
                        egui::Align2::LEFT_TOP,
                        label,
                        egui::FontId::proportional(12.0),
                        egui::Color32::from_gray(220),
                    );
                }
            }
        }
    }

    /// Draw Joint Drive mode icon.
    fn draw_joint_drive_icon(
        &self,
        painter: &egui::Painter,
        c: egui::Pos2,
        bg_color: egui::Color32,
        icon_color: egui::Color32,
        is_active: bool,
    ) {
        let ghost_color = if is_active {
            egui::Color32::from_rgba_unmultiplied(255, 255, 255, 70)
        } else {
            egui::Color32::from_rgba_unmultiplied(200, 200, 200, 50)
        };

        let pivot = egui::pos2(c.x - 2.0, c.y + 1.0);
        let arm1_start = egui::pos2(c.x - 11.0, c.y - 8.0);
        painter.line_segment(
            [arm1_start, pivot],
            egui::Stroke::new(3.0, icon_color),
        );

        let ghost_end = egui::pos2(pivot.x + 10.0, pivot.y + 3.0);
        painter.line_segment(
            [pivot, ghost_end],
            egui::Stroke::new(2.5, ghost_color),
        );

        let arm2_end = egui::pos2(pivot.x + 8.0, pivot.y - 7.0);
        painter.line_segment(
            [pivot, arm2_end],
            egui::Stroke::new(3.0, icon_color),
        );

        painter.circle_filled(pivot, 3.0, icon_color);
        painter.circle_stroke(
            pivot,
            3.0,
            egui::Stroke::new(1.2, bg_color),
        );

        let arc_r = 7.0;
        let a_start = -0.29_f32;
        let a_end = 0.72_f32;
        let n_seg = 8;
        let arc_pts: Vec<egui::Pos2> = (0..=n_seg)
            .map(|k| {
                let t = k as f32 / n_seg as f32;
                let a = a_start + (a_end - a_start) * t;
                egui::pos2(
                    pivot.x + arc_r * a.cos(),
                    pivot.y - arc_r * a.sin(),
                )
            })
            .collect();
        for w in arc_pts.windows(2) {
            painter.line_segment(
                [w[0], w[1]],
                egui::Stroke::new(1.2, icon_color),
            );
        }
        if let Some(&tip) = arc_pts.last() {
            let prev = arc_pts[arc_pts.len() - 2];
            let dir = egui::vec2(tip.x - prev.x, tip.y - prev.y);
            let len = (dir.x * dir.x + dir.y * dir.y).sqrt().max(0.01);
            let nd = egui::vec2(dir.x / len, dir.y / len);
            let perp = egui::vec2(-nd.y, nd.x);
            let sz = 3.5;
            painter.line_segment(
                [tip, tip - nd * sz + perp * sz * 0.6],
                egui::Stroke::new(1.2, icon_color),
            );
            painter.line_segment(
                [tip, tip - nd * sz - perp * sz * 0.6],
                egui::Stroke::new(1.2, icon_color),
            );
        }
    }

    /// Draw Offset Adjust mode icon.
    fn draw_offset_adjust_icon(
        &self,
        painter: &egui::Painter,
        c: egui::Pos2,
        _bg_color: egui::Color32,
        icon_color: egui::Color32,
        is_active: bool,
    ) {
        let ghost_color = if is_active {
            egui::Color32::from_rgba_unmultiplied(255, 255, 255, 60)
        } else {
            egui::Color32::from_rgba_unmultiplied(200, 200, 200, 45)
        };
        let solid_color = icon_color;
        let arrow_color = if is_active {
            egui::Color32::from_rgb(100, 200, 255)
        } else {
            egui::Color32::from_rgb(130, 170, 200)
        };

        let draw_box = |cx: f32, cy: f32, hw: f32, hh: f32, depth: f32,
                        stroke: egui::Stroke| {
            let fl = egui::pos2(cx - hw, cy - hh);
            let fr = egui::pos2(cx + hw, cy - hh);
            let br = egui::pos2(cx + hw, cy + hh);
            let bl = egui::pos2(cx - hw, cy + hh);

            let dx = depth * 0.6;
            let dy = -depth * 0.5;
            let fl2 = egui::pos2(fl.x + dx, fl.y + dy);
            let fr2 = egui::pos2(fr.x + dx, fr.y + dy);
            let br2 = egui::pos2(br.x + dx, br.y + dy);
            let _bl2 = egui::pos2(bl.x + dx, bl.y + dy);

            painter.line_segment([fl, fr], stroke);
            painter.line_segment([fr, br], stroke);
            painter.line_segment([br, bl], stroke);
            painter.line_segment([bl, fl], stroke);

            painter.line_segment([fl2, fr2], stroke);
            painter.line_segment([fr2, br2], stroke);

            painter.line_segment([fl, fl2], stroke);
            painter.line_segment([fr, fr2], stroke);
            painter.line_segment([br, br2], stroke);
        };

        let g_cx = c.x - 5.0;
        let g_cy = c.y + 3.0;
        draw_box(
            g_cx, g_cy, 4.5, 3.5, 4.0,
            egui::Stroke::new(1.2, ghost_color),
        );

        let s_cx = c.x + 4.5;
        let s_cy = c.y - 3.5;
        draw_box(
            s_cx, s_cy, 4.5, 3.5, 4.0,
            egui::Stroke::new(1.6, solid_color),
        );

        let arrow_start = egui::pos2(g_cx + 2.0, g_cy - 1.5);
        let arrow_end = egui::pos2(s_cx - 2.0, s_cy + 1.5);
        let n_dash = 3;
        for k in 0..n_dash {
            let t0 = (k as f32 * 2.0) / (n_dash as f32 * 2.0 - 1.0);
            let t1 = (k as f32 * 2.0 + 1.0) / (n_dash as f32 * 2.0 - 1.0);
            let t1 = t1.min(1.0);
            let p0 = egui::pos2(
                arrow_start.x + (arrow_end.x - arrow_start.x) * t0,
                arrow_start.y + (arrow_end.y - arrow_start.y) * t0,
            );
            let p1 = egui::pos2(
                arrow_start.x + (arrow_end.x - arrow_start.x) * t1,
                arrow_start.y + (arrow_end.y - arrow_start.y) * t1,
            );
            painter.line_segment(
                [p0, p1],
                egui::Stroke::new(1.4, arrow_color),
            );
        }
        let ad = egui::vec2(
            arrow_end.x - arrow_start.x,
            arrow_end.y - arrow_start.y,
        );
        let al = (ad.x * ad.x + ad.y * ad.y).sqrt().max(0.01);
        let and = egui::vec2(ad.x / al, ad.y / al);
        let aperp = egui::vec2(-and.y, and.x);
        let ah = 3.5_f32;
        painter.line_segment(
            [arrow_end, arrow_end - and * ah + aperp * ah * 0.5],
            egui::Stroke::new(1.4, arrow_color),
        );
        painter.line_segment(
            [arrow_end, arrow_end - and * ah - aperp * ah * 0.5],
            egui::Stroke::new(1.4, arrow_color),
        );
    }

    /// Draw CoM mass labels on the viewport.
    pub(super) fn draw_com_labels(
        &self,
        ui: &mut egui::Ui,
        rect: egui::Rect,
        aspect: f32,
    ) {
        if !self.show_com {
            return;
        }
        let com_positions = self.gl_renderer.lock().unwrap().com_world_positions();
        let painter = ui.painter();
        for (world_pos, mass) in &com_positions {
            if let Some(ndc) = self.camera.project(world_pos, aspect) {
                let screen_pos = egui::pos2(
                    rect.left() + ndc.x * rect.width(),
                    rect.top() + ndc.y * rect.height(),
                );
                if rect.contains(screen_pos) {
                    let label = format!("{:.3} kg", mass);
                    painter.text(
                        screen_pos + egui::vec2(6.0, -6.0),
                        egui::Align2::LEFT_BOTTOM,
                        &label,
                        egui::FontId::proportional(11.0),
                        egui::Color32::from_rgb(255, 128, 255),
                    );
                }
            }
        }
    }

    /// Draw the IK root anchor icon on the viewport.
    pub(super) fn draw_ik_root_anchor(
        &self,
        ui: &mut egui::Ui,
        rect: egui::Rect,
        aspect: f32,
    ) {
        if self.interaction_mode != InteractionMode::JointDrive
            || self.drag_mode != DragMode::InverseKinematics
        {
            return;
        }
        let Some(ref root_name) = self.ik_root_link else { return };
        let Some(ref model) = self.model else { return };

        let transforms = model.compute_transforms();
        let Some(root_tf) = transforms.get(root_name) else { return };

        let world_pos = na::Point3::from(root_tf.translation.vector);
        let Some(ndc) = self.camera.project(&world_pos, aspect) else { return };

        let screen_pos = egui::pos2(
            rect.left() + ndc.x * rect.width(),
            rect.top() + ndc.y * rect.height(),
        );
        if !rect.contains(screen_pos) {
            return;
        }

        let painter = ui.painter();
        let c = screen_pos;
        let anchor_color = egui::Color32::from_rgb(255, 180, 50);
        let anchor_bg = egui::Color32::from_rgba_unmultiplied(0, 0, 0, 140);

        painter.circle_filled(c, 13.0, anchor_bg);

        let ring_cy = c.y - 5.5;
        painter.circle_stroke(
            egui::pos2(c.x, ring_cy),
            3.0,
            egui::Stroke::new(1.6, anchor_color),
        );

        let shank_top = ring_cy + 3.0;
        let shank_bot = c.y + 7.0;
        painter.line_segment(
            [egui::pos2(c.x, shank_top), egui::pos2(c.x, shank_bot)],
            egui::Stroke::new(1.8, anchor_color),
        );

        let bar_y = c.y + 1.0;
        painter.line_segment(
            [egui::pos2(c.x - 5.0, bar_y), egui::pos2(c.x + 5.0, bar_y)],
            egui::Stroke::new(1.6, anchor_color),
        );

        let n_seg = 6;
        for &sign in &[-1.0_f32, 1.0] {
            let pts: Vec<egui::Pos2> = (0..=n_seg)
                .map(|k| {
                    let t = k as f32 / n_seg as f32;
                    let angle = t * std::f32::consts::FRAC_PI_2;
                    egui::pos2(
                        c.x + sign * 5.0 * angle.sin(),
                        bar_y + 6.0 * angle.cos(),
                    )
                })
                .collect();
            for w in pts.windows(2) {
                painter.line_segment(
                    [w[0], w[1]],
                    egui::Stroke::new(1.6, anchor_color),
                );
            }
        }

        painter.text(
            egui::pos2(c.x, c.y + 15.0),
            egui::Align2::CENTER_TOP,
            root_name,
            egui::FontId::proportional(10.0),
            anchor_color,
        );
    }

    /// Draw camera orientation axes (bottom-right corner).
    pub(super) fn draw_camera_axes(&self, ui: &mut egui::Ui, rect: egui::Rect) {
        let painter = ui.painter();
        let axes_size = 50.0_f32;
        let margin = 10.0;
        let center = egui::pos2(
            rect.right() - margin - axes_size,
            rect.bottom() - margin - axes_size,
        );

        painter.circle_filled(
            center,
            axes_size,
            egui::Color32::from_rgba_unmultiplied(20, 20, 30, 150),
        );

        let view = self.camera.view_matrix();
        let view3 = view.fixed_view::<3, 3>(0, 0);

        let axis_len = axes_size * 0.7;
        let world_axes: [(na::Vector3<f32>, egui::Color32, &str); 3] = [
            (na::Vector3::x(), egui::Color32::from_rgb(230, 60, 60), "X"),
            (na::Vector3::y(), egui::Color32::from_rgb(60, 200, 60), "Y"),
            (na::Vector3::z(), egui::Color32::from_rgb(60, 100, 230), "Z"),
        ];

        let mut draw_order: Vec<(usize, f32)> = world_axes
            .iter()
            .enumerate()
            .map(|(i, (ax, _, _))| {
                let cam = view3 * ax;
                (i, cam.z)
            })
            .collect();
        draw_order.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap());

        for (i, _depth) in draw_order {
            let (ax, color, label) = &world_axes[i];
            let cam = view3 * ax;
            let tip = egui::pos2(
                center.x + cam.x * axis_len,
                center.y - cam.y * axis_len,
            );
            painter.line_segment(
                [center, tip],
                egui::Stroke::new(2.5, *color),
            );
            painter.circle_filled(tip, 4.0, *color);
            painter.text(
                tip + egui::vec2(6.0, -6.0),
                egui::Align2::LEFT_BOTTOM,
                *label,
                egui::FontId::proportional(11.0),
                *color,
            );
        }
    }

    /// Draw gravity direction indicator (bottom-right corner, left of camera axes).
    ///
    /// Shows:
    ///  - Faint X/Y/Z reference axes (same projection as camera-axes widget)
    ///  - Bold gravity arrow
    ///  - Dashed horizontal-plane projection of gravity
    ///  - Tilt angle from −Z displayed as text
    pub(super) fn draw_gravity_indicator(&self, ui: &mut egui::Ui, rect: egui::Rect) {
        if !self.show_gravity_arrow {
            return;
        }

        let painter = ui.painter();
        let axes_size = 50.0_f32;
        let margin = 10.0;
        let gap = 8.0;

        // Camera axes center is at (right - margin - axes_size, bottom - margin - axes_size).
        // Place gravity indicator to its left.
        let center = egui::pos2(
            rect.right() - margin - axes_size - (axes_size * 2.0 + gap),
            rect.bottom() - margin - axes_size,
        );

        // Background circle
        painter.circle_filled(
            center,
            axes_size,
            egui::Color32::from_rgba_unmultiplied(20, 12, 25, 160),
        );
        painter.circle_stroke(
            center,
            axes_size,
            egui::Stroke::new(1.0, egui::Color32::from_rgba_unmultiplied(160, 70, 160, 100)),
        );

        let view = self.camera.view_matrix();
        let view3 = view.fixed_view::<3, 3>(0, 0);
        let axis_len = axes_size * 0.7;

        // ── 1. Faint reference axes (X / Y / Z) ──
        let ref_axes: [(na::Vector3<f32>, egui::Color32, &str); 3] = [
            (na::Vector3::x(), egui::Color32::from_rgba_unmultiplied(230, 60, 60, 55), "X"),
            (na::Vector3::y(), egui::Color32::from_rgba_unmultiplied(60, 200, 60, 55), "Y"),
            (na::Vector3::z(), egui::Color32::from_rgba_unmultiplied(60, 100, 230, 55), "Z"),
        ];
        // depth-sort
        let mut ref_order: Vec<(usize, f32)> = ref_axes
            .iter()
            .enumerate()
            .map(|(i, (ax, _, _))| {
                let c = view3 * ax;
                (i, c.z)
            })
            .collect();
        ref_order.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap());

        for (i, _depth) in &ref_order {
            let (ax, color, label) = &ref_axes[*i];
            let c = view3 * ax;
            let tip = egui::pos2(center.x + c.x * axis_len, center.y - c.y * axis_len);
            painter.line_segment([center, tip], egui::Stroke::new(1.5, *color));
            painter.circle_filled(tip, 6.0, *color);
            painter.text(
                tip + egui::vec2(5.0, -5.0),
                egui::Align2::LEFT_BOTTOM,
                *label,
                egui::FontId::proportional(9.0),
                *color,
            );
        }

        // ── 2. Gravity vector ──
        let gd = na::Vector3::new(
            self.gravity_dir[0],
            self.gravity_dir[1],
            self.gravity_dir[2],
        );
        let len = gd.norm();
        if len < 1e-6 {
            return;
        }
        let dir = gd / len;
        let cam_g = view3 * dir;

        let arrow_len = axes_size * 0.8;
        let tip = egui::pos2(
            center.x + cam_g.x * arrow_len,
            center.y - cam_g.y * arrow_len,
        );

        // ── 3. Horizontal-plane projection (dashed) ──
        // Project gravity onto the horizontal (XY) plane and draw as a dashed line.
        let horiz = na::Vector3::new(dir.x, dir.y, 0.0);
        let horiz_len = horiz.norm();
        if horiz_len > 1e-4 {
            let horiz_dir = horiz / horiz_len;
            let cam_h = view3 * horiz_dir;
            let h_screen_len = arrow_len * horiz_len; // scale by horizontal component magnitude
            let h_tip = egui::pos2(
                center.x + cam_h.x * h_screen_len,
                center.y - cam_h.y * h_screen_len,
            );
            let dash_color = egui::Color32::from_rgba_unmultiplied(210, 100, 210, 70);
            // Draw dashed line from center to h_tip
            let n_dashes = 8;
            for d in 0..n_dashes {
                if d % 2 == 0 {
                    let t0 = d as f32 / n_dashes as f32;
                    let t1 = (d + 1) as f32 / n_dashes as f32;
                    let p0 = egui::pos2(
                        center.x + (h_tip.x - center.x) * t0,
                        center.y + (h_tip.y - center.y) * t0,
                    );
                    let p1 = egui::pos2(
                        center.x + (h_tip.x - center.x) * t1,
                        center.y + (h_tip.y - center.y) * t1,
                    );
                    painter.line_segment([p0, p1], egui::Stroke::new(1.2, dash_color));
                }
            }
            // Dashed vertical drop line from h_tip to arrow tip
            let v_dashes = 6;
            for d in 0..v_dashes {
                if d % 2 == 0 {
                    let t0 = d as f32 / v_dashes as f32;
                    let t1 = (d + 1) as f32 / v_dashes as f32;
                    let p0 = egui::pos2(
                        h_tip.x + (tip.x - h_tip.x) * t0,
                        h_tip.y + (tip.y - h_tip.y) * t0,
                    );
                    let p1 = egui::pos2(
                        h_tip.x + (tip.x - h_tip.x) * t1,
                        h_tip.y + (tip.y - h_tip.y) * t1,
                    );
                    painter.line_segment([p0, p1], egui::Stroke::new(1.0, dash_color));
                }
            }
        }

        // ── 4. Main gravity arrow (bold purple) ──
        let arrow_color = egui::Color32::from_rgb(220, 110, 220);
        painter.line_segment(
            [center, tip],
            egui::Stroke::new(3.0, arrow_color),
        );

        // Arrowhead
        let dx = tip.x - center.x;
        let dy = tip.y - center.y;
        let shaft_len = (dx * dx + dy * dy).sqrt().max(1e-6);
        let ux = dx / shaft_len;
        let uy = dy / shaft_len;
        let head_len = 10.0_f32;
        let head_width = 5.0_f32;
        let fin1 = egui::pos2(
            tip.x - ux * head_len + uy * head_width,
            tip.y - uy * head_len - ux * head_width,
        );
        let fin2 = egui::pos2(
            tip.x - ux * head_len - uy * head_width,
            tip.y - uy * head_len + ux * head_width,
        );
        painter.add(egui::Shape::convex_polygon(
            vec![tip, fin1, fin2],
            arrow_color,
            egui::Stroke::NONE,
        ));

        // Origin dot
        painter.circle_filled(center, 3.0, arrow_color);

        // "g" label at tip
        painter.text(
            tip + egui::vec2(7.0, -7.0),
            egui::Align2::LEFT_BOTTOM,
            "g",
            egui::FontId::proportional(13.0),
            arrow_color,
        );

        // ── 5. Tilt angle from −Z ──
        let neg_z = na::Vector3::new(0.0, 0.0, -1.0);
        let cos_angle = dir.dot(&neg_z).clamp(-1.0, 1.0);
        let angle_deg = cos_angle.acos().to_degrees();
        let angle_text = if angle_deg < 0.5 {
            "0°".to_string()
        } else {
            format!("{:.1}°", angle_deg)
        };

        // Show below the circle
        painter.text(
            egui::pos2(center.x, center.y + axes_size + 3.0),
            egui::Align2::CENTER_TOP,
            format!("tilt {}", angle_text),
            egui::FontId::monospace(10.0),
            egui::Color32::from_rgba_unmultiplied(210, 140, 210, 200),
        );

        // Title label at top of circle
        painter.text(
            egui::pos2(center.x, center.y - axes_size - 2.0),
            egui::Align2::CENTER_BOTTOM,
            "Gravity",
            egui::FontId::proportional(10.0),
            egui::Color32::from_rgba_unmultiplied(210, 100, 210, 180),
        );
    }

    /// Draw camera reset button (above the camera axes widget, bottom-right).
    pub(super) fn draw_camera_reset_button(&mut self, ui: &mut egui::Ui, rect: egui::Rect) {
        let painter = ui.painter();
        let btn_size = egui::vec2(28.0, 28.0);
        let margin = 10.0;
        let axes_size = 50.0_f32; // must match draw_camera_axes

        // Place above the camera axes circle (center at rect.right()-margin-axes_size)
        let axes_top = rect.bottom() - margin - axes_size * 2.0;
        let btn_pos = egui::pos2(
            rect.right() - margin - axes_size - btn_size.x * 0.5,
            axes_top - 8.0 - btn_size.y,
        );
        let btn_rect = egui::Rect::from_min_size(btn_pos, btn_size);

        let btn_response = ui.interact(
            btn_rect,
            ui.id().with("camera_reset_btn"),
            egui::Sense::click(),
        );

        let bg_color = if btn_response.hovered() {
            egui::Color32::from_rgba_unmultiplied(100, 100, 100, 200)
        } else {
            egui::Color32::from_rgba_unmultiplied(40, 40, 50, 160)
        };
        painter.rect_filled(btn_rect, egui::CornerRadius::same(4), bg_color);

        painter.text(
            btn_rect.center(),
            egui::Align2::CENTER_CENTER,
            "⟲",
            egui::FontId::proportional(18.0),
            egui::Color32::from_gray(220),
        );

        if btn_response.clicked() {
            self.camera.reset();
        }

        if btn_response.hovered() {
            let tip_pos = egui::pos2(btn_rect.left(), btn_rect.top() - 4.0);
            painter.text(
                tip_pos,
                egui::Align2::LEFT_BOTTOM,
                "Reset Camera",
                egui::FontId::proportional(12.0),
                egui::Color32::from_gray(220),
            );
        }

        // --- Gravity direction label (bottom-left corner) ---
        // (gravity indicator now drawn alongside camera axes in draw_gravity_indicator)
    }

    /// Draw Visual / Collision display toggle buttons (top-right of viewport).
    pub(super) fn draw_display_toggles(&mut self, ui: &mut egui::Ui, rect: egui::Rect) {
        let painter = ui.painter();
        let btn_size = egui::vec2(28.0, 28.0);
        let margin = 8.0;
        let gap = 4.0;

        // Position: top-right corner of the viewport
        let start_x = rect.right() - margin - btn_size.x * 2.0 - gap;
        let start_y = rect.top() + margin;

        // --- Visual toggle ---
        let vis_on = self.visual_mode != DisplayMode::Off;
        let vis_rect = egui::Rect::from_min_size(
            egui::pos2(start_x, start_y),
            btn_size,
        );
        let vis_resp = ui.interact(
            vis_rect,
            ui.id().with("toggle_visual"),
            egui::Sense::click(),
        );

        let vis_bg = if vis_on {
            egui::Color32::from_rgba_unmultiplied(60, 130, 220, 200)
        } else if vis_resp.hovered() {
            egui::Color32::from_rgba_unmultiplied(80, 80, 80, 180)
        } else {
            egui::Color32::from_rgba_unmultiplied(40, 40, 50, 160)
        };
        painter.rect_filled(vis_rect, egui::CornerRadius::same(4), vis_bg);

        // Eye icon for visual
        let vis_icon = match self.visual_mode {
            DisplayMode::Off => "ⓥ",
            DisplayMode::Solid => "👁",
            DisplayMode::Wireframe => "◫",
            DisplayMode::Transparent => "◑",
            DisplayMode::FlatShading => "▧",
            DisplayMode::Points => "⁙",
        };
        painter.text(
            vis_rect.center(),
            egui::Align2::CENTER_CENTER,
            vis_icon,
            egui::FontId::proportional(14.0),
            if vis_on {
                egui::Color32::WHITE
            } else {
                egui::Color32::from_gray(140)
            },
        );

        if vis_resp.clicked() {
            self.visual_mode = self.visual_mode.next();
        }
        if vis_resp.hovered() {
            let label = format!("Visual: {}", self.visual_mode.label());
            painter.text(
                egui::pos2(vis_rect.left(), vis_rect.bottom() + 4.0),
                egui::Align2::LEFT_TOP,
                label,
                egui::FontId::proportional(12.0),
                egui::Color32::from_gray(220),
            );
        }

        // --- Collision toggle ---
        let col_on = self.collision_mode != DisplayMode::Off;
        let col_rect = egui::Rect::from_min_size(
            egui::pos2(start_x + btn_size.x + gap, start_y),
            btn_size,
        );
        let col_resp = ui.interact(
            col_rect,
            ui.id().with("toggle_collision"),
            egui::Sense::click(),
        );

        let col_bg = if col_on {
            egui::Color32::from_rgba_unmultiplied(220, 140, 40, 200)
        } else if col_resp.hovered() {
            egui::Color32::from_rgba_unmultiplied(80, 80, 80, 180)
        } else {
            egui::Color32::from_rgba_unmultiplied(40, 40, 50, 160)
        };
        painter.rect_filled(col_rect, egui::CornerRadius::same(4), col_bg);

        // Shield icon for collision
        let col_icon = match self.collision_mode {
            DisplayMode::Off => "ⓒ",
            DisplayMode::Solid => "🛡",
            DisplayMode::Wireframe => "◫",
            DisplayMode::Transparent => "◑",
            DisplayMode::FlatShading => "▧",
            DisplayMode::Points => "⁙",
        };
        painter.text(
            col_rect.center(),
            egui::Align2::CENTER_CENTER,
            col_icon,
            egui::FontId::proportional(14.0),
            if col_on {
                egui::Color32::WHITE
            } else {
                egui::Color32::from_gray(140)
            },
        );

        if col_resp.clicked() {
            self.collision_mode = self.collision_mode.next_collision();
        }
        if col_resp.hovered() {
            let label = format!("Collision: {}", self.collision_mode.label());
            painter.text(
                egui::pos2(col_rect.left(), col_rect.bottom() + 4.0),
                egui::Align2::LEFT_TOP,
                label,
                egui::FontId::proportional(12.0),
                egui::Color32::from_gray(220),
            );
        }
    }
}
