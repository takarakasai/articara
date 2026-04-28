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

    /// Draw a crosshair at the IK target position during IK drag.
    pub(super) fn draw_ik_target_marker(
        &self,
        ui: &mut egui::Ui,
        rect: egui::Rect,
        aspect: f32,
    ) {
        let Some(target_world) = self.ik_target_marker else { return };
        let Some(ndc) = self.camera.project(&target_world, aspect) else { return };

        let screen_pos = egui::pos2(
            rect.left() + ndc.x * rect.width(),
            rect.top() + ndc.y * rect.height(),
        );
        if !rect.contains(screen_pos) {
            return;
        }

        let painter = ui.painter();
        let c = screen_pos;
        let color = egui::Color32::from_rgb(50, 255, 100);
        let size = 10.0_f32;

        // Crosshair: two lines
        painter.line_segment(
            [egui::pos2(c.x - size, c.y), egui::pos2(c.x + size, c.y)],
            egui::Stroke::new(1.5, color),
        );
        painter.line_segment(
            [egui::pos2(c.x, c.y - size), egui::pos2(c.x, c.y + size)],
            egui::Stroke::new(1.5, color),
        );
        // Circle
        painter.circle_stroke(
            c,
            size * 0.7,
            egui::Stroke::new(1.5, color),
        );

        // Numeric position label + IK error
        let error_str = match self.ik_error {
            Some(e) => format!("  err={:.4}", e),
            None => String::new(),
        };
        let label = format!(
            "({:.3}, {:.3}, {:.3}){}",
            target_world.x, target_world.y, target_world.z, error_str,
        );
        let font = egui::FontId::monospace(11.0);
        let bg = egui::Color32::from_rgba_unmultiplied(0, 0, 0, 180);
        let text_pos = egui::pos2(c.x + size + 4.0, c.y - 7.0);
        let galley = painter.layout_no_wrap(label, font, color);
        let text_rect = egui::Rect::from_min_size(
            egui::pos2(text_pos.x - 2.0, text_pos.y - 1.0),
            galley.size() + egui::vec2(4.0, 2.0),
        );
        painter.rect_filled(text_rect, 2.0, bg);
        painter.galley(text_pos, galley, color);
    }

    /// Draw a diamond marker at the current end-effector (bounding-sphere center)
    /// position during IK drag so the user can see where the EE actually is.
    pub(super) fn draw_ik_ee_marker(
        &self,
        ui: &mut egui::Ui,
        rect: egui::Rect,
        aspect: f32,
    ) {
        let Some(ee_world) = self.ik_ee_marker else { return };
        let Some(ndc) = self.camera.project(&ee_world, aspect) else { return };

        let screen_pos = egui::pos2(
            rect.left() + ndc.x * rect.width(),
            rect.top() + ndc.y * rect.height(),
        );
        if !rect.contains(screen_pos) {
            return;
        }

        let painter = ui.painter();
        let c = screen_pos;
        let color = egui::Color32::from_rgb(255, 180, 50); // orange
        let size = 8.0_f32;

        // Diamond shape (rotated square)
        let points = vec![
            egui::pos2(c.x, c.y - size),
            egui::pos2(c.x + size, c.y),
            egui::pos2(c.x, c.y + size),
            egui::pos2(c.x - size, c.y),
        ];
        let shape = egui::Shape::convex_polygon(
            points,
            egui::Color32::from_rgba_unmultiplied(255, 180, 50, 80),
            egui::Stroke::new(2.0, color),
        );
        painter.add(shape);

        // Label: "EE" + coordinates
        let label = format!(
            "EE ({:.3}, {:.3}, {:.3})",
            ee_world.x, ee_world.y, ee_world.z,
        );
        let font = egui::FontId::monospace(10.0);
        let bg = egui::Color32::from_rgba_unmultiplied(0, 0, 0, 160);
        let text_pos = egui::pos2(c.x + size + 4.0, c.y - 6.0);
        let galley = painter.layout_no_wrap(label, font, color);
        let text_rect = egui::Rect::from_min_size(
            egui::pos2(text_pos.x - 2.0, text_pos.y - 1.0),
            galley.size() + egui::vec2(4.0, 2.0),
        );
        painter.rect_filled(text_rect, 2.0, bg);
        painter.galley(text_pos, galley, color);
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

    /// Draw IK pin markers (◆) on the viewport for each pinned link.
    pub(super) fn draw_ik_pin_markers(
        &self,
        ui: &mut egui::Ui,
        rect: egui::Rect,
        aspect: f32,
    ) {
        if self.pinned_links.is_empty() {
            return;
        }
        let Some(ref model) = self.model else { return };
        let transforms = model.compute_transforms();
        let painter = ui.painter();
        let pin_color = egui::Color32::from_rgb(255, 160, 0);
        let pin_bg = egui::Color32::from_rgba_unmultiplied(0, 0, 0, 140);

        for pin in &self.pinned_links {
            let Some(&li) = model.link_map.get(&pin.link_name) else { continue };
            let world_pos = model.ee_world_pos(li, &transforms);
            let Some(ndc) = self.camera.project(&world_pos, aspect) else { continue };

            let screen_pos = egui::pos2(
                rect.left() + ndc.x * rect.width(),
                rect.top() + ndc.y * rect.height(),
            );
            if !rect.contains(screen_pos) {
                continue;
            }

            // Background circle
            painter.circle_filled(screen_pos, 10.0, pin_bg);

            // Diamond (◆) shape
            let s = 6.0_f32;
            let diamond = vec![
                egui::pos2(screen_pos.x, screen_pos.y - s),
                egui::pos2(screen_pos.x + s * 0.7, screen_pos.y),
                egui::pos2(screen_pos.x, screen_pos.y + s),
                egui::pos2(screen_pos.x - s * 0.7, screen_pos.y),
            ];
            painter.add(egui::Shape::convex_polygon(
                diamond,
                pin_color,
                egui::Stroke::NONE,
            ));

            // Label below
            painter.text(
                egui::pos2(screen_pos.x, screen_pos.y + 12.0),
                egui::Align2::CENTER_TOP,
                &pin.link_name,
                egui::FontId::proportional(10.0),
                pin_color,
            );
        }
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

    /// Draw a persistent marker on the currently-selected link and joint so
    /// the user sees which entity they're editing in the Properties panel
    /// even when the mouse is far away. The link receives a thin dashed
    /// circle around its bounding-sphere centre; the joint gets a crosshair
    /// at its origin, with the joint name tagged below.
    pub(super) fn draw_selection_markers(
        &self,
        ui: &mut egui::Ui,
        rect: egui::Rect,
        aspect: f32,
    ) {
        let Some(model) = self.model.as_ref() else {
            return;
        };
        let painter = ui.painter();
        let transforms = model.compute_transforms();

        // ── Selected link: dashed circle at bounding-sphere centre ──
        if let Some(li) = self.selected_link {
            // Don't double-draw when the link is also being dragged or
            // hovered — the renderer already colours it in those cases.
            let already_highlit = self
                .drag_state
                .as_ref()
                .map(|d| d.link_idx == li)
                .unwrap_or(false)
                || self.hovered_link == Some(li);
            if !already_highlit {
                let link_name = &model.links[li].name;
                if let Some(tf) = transforms.get(link_name) {
                    let (center, radius) = model.link_bounding_sphere(li);
                    let world_center = *tf * center;
                    if let Some(p_screen) =
                        self.project_world(world_center, rect, aspect)
                    {
                        // Approximate screen radius by projecting a point
                        // offset by `radius` along the camera right axis.
                        let cam_right = na::Vector3::new(
                            -self.camera.yaw.sin(),
                            self.camera.yaw.cos(),
                            0.0,
                        );
                        let edge_world = world_center + cam_right * radius;
                        let edge_screen = self
                            .project_world(edge_world, rect, aspect)
                            .unwrap_or(p_screen + egui::vec2(20.0, 0.0));
                        let r_screen =
                            (edge_screen - p_screen).length().max(8.0);
                        let color = egui::Color32::from_rgb(120, 220, 255);
                        // Dashed circle approximation via short arcs.
                        let segs = 24;
                        let mut a = 0.0_f32;
                        let step = std::f32::consts::TAU / segs as f32;
                        for k in 0..segs {
                            if k % 2 == 0 {
                                let p0 = egui::pos2(
                                    p_screen.x + r_screen * a.cos(),
                                    p_screen.y + r_screen * a.sin(),
                                );
                                let p1 = egui::pos2(
                                    p_screen.x + r_screen * (a + step).cos(),
                                    p_screen.y + r_screen * (a + step).sin(),
                                );
                                painter.line_segment(
                                    [p0, p1],
                                    egui::Stroke::new(2.0, color),
                                );
                            }
                            a += step;
                        }
                        // Name tag near the circle.
                        painter.text(
                            p_screen + egui::vec2(r_screen + 4.0, -r_screen),
                            egui::Align2::LEFT_BOTTOM,
                            format!("◆ {}", link_name),
                            egui::FontId::monospace(11.0),
                            color,
                        );
                    }
                }
            }
        }

        // ── Selected joint: crosshair at joint origin ──
        if let Some(ji) = self.selected_joint {
            if ji < model.joints.len() {
                let joint = &model.joints[ji];
                let parent_tf = transforms
                    .get(&joint.parent_link)
                    .copied()
                    .unwrap_or(na::Isometry3::identity());
                let joint_world = parent_tf * joint.origin;
                let pivot = na::Point3::from(joint_world.translation.vector);
                if let Some(p_screen) = self.project_world(pivot, rect, aspect) {
                    let color = egui::Color32::from_rgb(255, 220, 100);
                    let s = 10.0_f32;
                    painter.line_segment(
                        [
                            egui::pos2(p_screen.x - s, p_screen.y),
                            egui::pos2(p_screen.x + s, p_screen.y),
                        ],
                        egui::Stroke::new(2.0, color),
                    );
                    painter.line_segment(
                        [
                            egui::pos2(p_screen.x, p_screen.y - s),
                            egui::pos2(p_screen.x, p_screen.y + s),
                        ],
                        egui::Stroke::new(2.0, color),
                    );
                    painter.circle_stroke(
                        p_screen,
                        s * 0.6,
                        egui::Stroke::new(2.0, color),
                    );
                    painter.text(
                        p_screen + egui::vec2(s + 4.0, -s),
                        egui::Align2::LEFT_BOTTOM,
                        format!("⚙ {}", joint.name),
                        egui::FontId::monospace(11.0),
                        color,
                    );
                }
            }
        }
    }

    /// Project a world-space point to screen coordinates within `rect`.
    /// Returns `None` if the point is behind the camera or off-screen by a
    /// margin. Used by the contact-force / external-force overlays so they
    /// share a single conversion routine instead of inlining the math.
    pub(super) fn project_world(
        &self,
        world: na::Point3<f32>,
        rect: egui::Rect,
        aspect: f32,
    ) -> Option<egui::Pos2> {
        let ndc = self.camera.project(&world, aspect)?;
        Some(egui::pos2(
            rect.left() + ndc.x * rect.width(),
            rect.top() + ndc.y * rect.height(),
        ))
    }

    /// Draw a 2D arrow from `from` to `to` in screen space, with an
    /// arrowhead at the tip. Used by the contact-force / external-force
    /// overlays which both end up projecting a 3D vector onto the screen
    /// before drawing.
    pub(super) fn draw_screen_arrow(
        painter: &egui::Painter,
        from: egui::Pos2,
        to: egui::Pos2,
        color: egui::Color32,
        thickness: f32,
    ) {
        let v = to - from;
        let len = v.length();
        if len < 1e-3 {
            return;
        }
        painter.line_segment([from, to], egui::Stroke::new(thickness, color));
        let dir = v / len;
        let perp = egui::vec2(-dir.y, dir.x);
        let head = (8.0_f32).min(len * 0.4);
        let h_back = to - dir * head;
        painter.line_segment(
            [to, h_back + perp * (head * 0.5)],
            egui::Stroke::new(thickness, color),
        );
        painter.line_segment(
            [to, h_back - perp * (head * 0.5)],
            egui::Stroke::new(thickness, color),
        );
    }

    /// Draw markers + force vectors for every contact reported by the active
    /// MuJoCo sim. Suppressed when [`Self::show_contacts`] is false. Force
    /// arrow length is logarithmically scaled so small grazing contacts
    /// remain visible alongside heavy load-bearing ones.
    #[cfg(feature = "mujoco")]
    pub(super) fn draw_contact_markers(
        &self,
        ui: &mut egui::Ui,
        rect: egui::Rect,
        aspect: f32,
    ) {
        if !self.show_contacts {
            return;
        }
        let Some(sim) = self.mujoco_sim.as_ref() else {
            return;
        };
        let contacts = sim.contacts();
        if contacts.is_empty() {
            return;
        }

        let painter = ui.painter();
        // Calibrate arrow length so a 100 N force renders at ~60 px regardless
        // of zoom. Logarithmic scale keeps both 1 N and 1 kN on the same plot.
        let base_px = 60.0_f32;
        let calib_n = 100.0_f32;
        for c in &contacts {
            let p = na::Point3::new(c.pos[0] as f32, c.pos[1] as f32, c.pos[2] as f32);
            let Some(p_screen) = self.project_world(p, rect, aspect) else {
                continue;
            };
            // Marker dot
            let dot_color = egui::Color32::from_rgb(255, 200, 80);
            painter.circle_filled(p_screen, 4.0, dot_color);

            if c.force_mag < 1e-3 {
                continue;
            }
            // Tip in world space — scale linear force vector logarithmically.
            let log_scale =
                (1.0 + (c.force_mag as f32 / calib_n)).ln() / (1.0_f32 + 1.0).ln();
            let scale = log_scale * base_px;
            // Use the world force direction; convert to a world-space delta
            // small enough that the projected arrow respects perspective.
            let dir = na::Vector3::new(
                c.force_world[0] as f32,
                c.force_world[1] as f32,
                c.force_world[2] as f32,
            );
            let dnorm = dir.norm().max(1e-6);
            let world_step = 0.05_f32; // 5 cm step in world space
            let tip_world = p + (dir / dnorm) * world_step;
            let Some(tip_screen) = self.project_world(tip_world, rect, aspect) else {
                continue;
            };
            // Stretch screen-space tip vector by `scale` so heavy contacts
            // get longer arrows than light ones while keeping the orientation.
            let v = tip_screen - p_screen;
            let v_len = v.length().max(1e-3);
            let scaled_tip = p_screen + v * (scale / v_len);
            let color = egui::Color32::from_rgb(255, 90, 90);
            Self::draw_screen_arrow(painter, p_screen, scaled_tip, color, 2.0);
            // Magnitude label near the tip.
            let label = format!("{:.1} N", c.force_mag);
            painter.text(
                scaled_tip + egui::vec2(4.0, -10.0),
                egui::Align2::LEFT_BOTTOM,
                label,
                egui::FontId::monospace(10.0),
                color,
            );
        }
    }

    /// Draw active external-force pulses as world-space arrows anchored at
    /// each pulse's link origin. Pulses with a non-zero torque are drawn as
    /// a separate dashed arrow alongside the linear force arrow.
    #[cfg(feature = "mujoco")]
    pub(super) fn draw_force_pulse_markers(
        &self,
        ui: &mut egui::Ui,
        rect: egui::Rect,
        aspect: f32,
    ) {
        let Some(sim) = self.mujoco_sim.as_ref() else {
            return;
        };
        let pulses = sim.external_force_pulses();
        if pulses.is_empty() {
            return;
        }
        let Some(model) = self.model.as_ref() else {
            return;
        };
        let transforms = model.compute_transforms();
        let painter = ui.painter();

        for pulse in pulses {
            let Some(tf) = transforms.get(&pulse.link_name) else {
                continue;
            };
            let origin = na::Point3::new(
                tf.translation.x,
                tf.translation.y,
                tf.translation.z,
            );
            let Some(p_screen) = self.project_world(origin, rect, aspect) else {
                continue;
            };

            let f = na::Vector3::new(
                pulse.force[0] as f32,
                pulse.force[1] as f32,
                pulse.force[2] as f32,
            );
            let f_norm = f.norm();
            // Force arrow (cyan) — fixed-length screen render so direction
            // is always visible regardless of scale.
            if f_norm > 1e-4 {
                let world_step = 0.1_f32;
                let tip_world = origin + (f / f_norm) * world_step;
                if let Some(tip_screen) = self.project_world(tip_world, rect, aspect) {
                    let v = tip_screen - p_screen;
                    let v_len = v.length().max(1e-3);
                    let len_px = 80.0_f32; // fixed visible length
                    let scaled = p_screen + v * (len_px / v_len);
                    let color = egui::Color32::from_rgb(80, 200, 255);
                    Self::draw_screen_arrow(painter, p_screen, scaled, color, 2.5);
                    let remaining = (pulse.duration - pulse.elapsed).max(0.0);
                    painter.text(
                        scaled + egui::vec2(4.0, -10.0),
                        egui::Align2::LEFT_BOTTOM,
                        format!("{:.1} N · {:.2}s", f_norm, remaining),
                        egui::FontId::monospace(10.0),
                        color,
                    );
                }
            }

            // Torque arrow (magenta) drawn slightly offset so it doesn't
            // overlap with the linear force arrow.
            let m = na::Vector3::new(
                pulse.torque[0] as f32,
                pulse.torque[1] as f32,
                pulse.torque[2] as f32,
            );
            let m_norm = m.norm();
            if m_norm > 1e-4 {
                let world_step = 0.1_f32;
                let tip_world = origin + (m / m_norm) * world_step;
                if let Some(tip_screen) = self.project_world(tip_world, rect, aspect) {
                    let v = tip_screen - p_screen;
                    let v_len = v.length().max(1e-3);
                    let len_px = 60.0_f32;
                    let scaled = p_screen + v * (len_px / v_len);
                    let color = egui::Color32::from_rgb(220, 100, 255);
                    Self::draw_screen_arrow(painter, p_screen, scaled, color, 2.0);
                    painter.text(
                        scaled + egui::vec2(4.0, 4.0),
                        egui::Align2::LEFT_TOP,
                        format!("{:.2} N·m", m_norm),
                        egui::FontId::monospace(10.0),
                        color,
                    );
                }
            }
        }
    }
}
