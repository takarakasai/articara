use eframe::egui;

use super::ArticaraApp;

impl ArticaraApp {
    /// Draw invisible resize handles at all edges and corners of the window.
    /// Must be called once per frame (e.g. at the top of `ui()`).
    pub(super) fn draw_resize_borders(&self, ctx: &egui::Context) {
        let is_maximized = ctx.input(|i| i.viewport().maximized).unwrap_or(false);
        if is_maximized {
            return; // no resize when maximized
        }

        let screen = ctx.input(|i| i.viewport_rect());
        let edge = 5.0_f32; // width of the resize zone in logical pixels
        let corner = 12.0_f32; // size of corner resize zone

        // Helper: add an invisible resize zone at the given rect.
        let resize_zone =
            |id_salt: &str,
             rect: egui::Rect,
             cursor: egui::CursorIcon,
             dir: egui::ResizeDirection| {
                // Use an Area so the zone is always on top of all panels.
                egui::Area::new(egui::Id::new(id_salt))
                    .fixed_pos(rect.min)
                    .order(egui::Order::Foreground)
                    .interactable(true)
                    .show(ctx, |ui| {
                        let resp = ui.allocate_response(
                            rect.size(),
                            egui::Sense::click_and_drag(),
                        );
                        if resp.hovered() {
                            ui.ctx().set_cursor_icon(cursor);
                        }
                        if resp.drag_started() {
                            ui.ctx()
                                .send_viewport_cmd(egui::ViewportCommand::BeginResize(dir));
                        }
                    });
            };

        // ── Corners (checked first so they take priority) ──
        // Top-left
        resize_zone(
            "resize_nw",
            egui::Rect::from_min_size(screen.left_top(), egui::vec2(corner, corner)),
            egui::CursorIcon::ResizeNwSe,
            egui::ResizeDirection::NorthWest,
        );
        // Top-right
        resize_zone(
            "resize_ne",
            egui::Rect::from_min_size(
                egui::pos2(screen.right() - corner, screen.top()),
                egui::vec2(corner, corner),
            ),
            egui::CursorIcon::ResizeNeSw,
            egui::ResizeDirection::NorthEast,
        );
        // Bottom-left
        resize_zone(
            "resize_sw",
            egui::Rect::from_min_size(
                egui::pos2(screen.left(), screen.bottom() - corner),
                egui::vec2(corner, corner),
            ),
            egui::CursorIcon::ResizeNeSw,
            egui::ResizeDirection::SouthWest,
        );
        // Bottom-right
        resize_zone(
            "resize_se",
            egui::Rect::from_min_size(
                egui::pos2(screen.right() - corner, screen.bottom() - corner),
                egui::vec2(corner, corner),
            ),
            egui::CursorIcon::ResizeNwSe,
            egui::ResizeDirection::SouthEast,
        );

        // ── Edges (exclude corner regions) ──
        // Top
        resize_zone(
            "resize_n",
            egui::Rect::from_min_max(
                egui::pos2(screen.left() + corner, screen.top()),
                egui::pos2(screen.right() - corner, screen.top() + edge),
            ),
            egui::CursorIcon::ResizeVertical,
            egui::ResizeDirection::North,
        );
        // Bottom
        resize_zone(
            "resize_s",
            egui::Rect::from_min_max(
                egui::pos2(screen.left() + corner, screen.bottom() - edge),
                egui::pos2(screen.right() - corner, screen.bottom()),
            ),
            egui::CursorIcon::ResizeVertical,
            egui::ResizeDirection::South,
        );
        // Left
        resize_zone(
            "resize_w",
            egui::Rect::from_min_max(
                egui::pos2(screen.left(), screen.top() + corner),
                egui::pos2(screen.left() + edge, screen.bottom() - corner),
            ),
            egui::CursorIcon::ResizeHorizontal,
            egui::ResizeDirection::West,
        );
        // Right
        resize_zone(
            "resize_e",
            egui::Rect::from_min_max(
                egui::pos2(screen.right() - edge, screen.top() + corner),
                egui::pos2(screen.right(), screen.bottom() - corner),
            ),
            egui::CursorIcon::ResizeHorizontal,
            egui::ResizeDirection::East,
        );
    }

    /// Render a custom title bar with drag-to-move, title text, and window control buttons.
    pub(super) fn draw_title_bar(&mut self, ui: &mut egui::Ui, ctx: &egui::Context) {
        let title_bar_rect = ui.max_rect();

        // Paint background – slightly lighter than the default dark panel
        let bg = egui::Color32::from_rgb(35, 35, 42);
        ui.painter()
            .rect_filled(title_bar_rect, egui::CornerRadius::ZERO, bg);

        // Thin separator line along the bottom edge
        let sep_color = egui::Color32::from_rgb(60, 60, 72);
        ui.painter().line_segment(
            [
                title_bar_rect.left_bottom(),
                title_bar_rect.right_bottom(),
            ],
            egui::Stroke::new(1.0, sep_color),
        );

        // ── Button dimensions ──
        let btn_w = 40.0_f32;
        let btn_h = title_bar_rect.height();
        let btn_count = 3.0_f32;
        let buttons_total_w = btn_w * btn_count;

        // ── 1. Drag-to-move zone (exclude button area on the right) ──
        let drag_rect = egui::Rect::from_min_max(
            title_bar_rect.left_top(),
            egui::pos2(
                title_bar_rect.right() - buttons_total_w,
                title_bar_rect.bottom(),
            ),
        );
        let drag_response =
            ui.interact(drag_rect, ui.id().with("title_bar_drag"), egui::Sense::click());
        if drag_response.is_pointer_button_down_on() {
            ctx.send_viewport_cmd(egui::ViewportCommand::StartDrag);
        }
        if drag_response.double_clicked() {
            let is_maximized = ctx.input(|i| i.viewport().maximized).unwrap_or(false);
            ctx.send_viewport_cmd(egui::ViewportCommand::Maximized(!is_maximized));
        }

        // ── 2. Title text (painted over the drag zone) ──
        ui.scope_builder(egui::UiBuilder::new().max_rect(drag_rect), |ui| {
            ui.horizontal_centered(|ui| {
                ui.add_space(8.0);
                ui.label(
                    egui::RichText::new("⚙")
                        .size(14.0)
                        .color(egui::Color32::from_rgb(130, 160, 220)),
                );
                let title = if let Some(ref model) = self.model {
                    format!("Articara — {}", model.name)
                } else {
                    "Articara - Robot Dynamics Editor".to_string()
                };
                ui.label(
                    egui::RichText::new(title)
                        .size(12.0)
                        .color(egui::Color32::from_rgb(200, 200, 210)),
                );
            });
        });

        // ── 3. Window-control buttons (right-aligned, each with hover highlight) ──
        let buttons_left = title_bar_rect.right() - buttons_total_w;

        // Helper: paint a title-bar button with hover/press highlight
        let draw_btn = |index: usize,
                            label: &str,
                            text_color: egui::Color32,
                            hover_bg: egui::Color32,
                            id_salt: &str|
         -> egui::Response {
            let btn_rect = egui::Rect::from_min_size(
                egui::pos2(buttons_left + index as f32 * btn_w, title_bar_rect.top()),
                egui::vec2(btn_w, btn_h),
            );
            let resp = ui.interact(btn_rect, ui.id().with(id_salt), egui::Sense::click());

            // Background highlight on hover / press
            if resp.is_pointer_button_down_on() {
                ui.painter()
                    .rect_filled(btn_rect, egui::CornerRadius::ZERO, hover_bg);
            } else if resp.hovered() {
                let mut c = hover_bg;
                c = egui::Color32::from_rgba_unmultiplied(c.r(), c.g(), c.b(), c.a() / 2);
                ui.painter()
                    .rect_filled(btn_rect, egui::CornerRadius::ZERO, c);
            }

            // Centered text
            ui.painter().text(
                btn_rect.center(),
                egui::Align2::CENTER_CENTER,
                label,
                egui::FontId::proportional(14.0),
                if resp.hovered() {
                    egui::Color32::WHITE
                } else {
                    text_color
                },
            );

            resp
        };

        // Minimize  ─
        let min_resp = draw_btn(
            0,
            "─",
            egui::Color32::from_rgb(180, 180, 190),
            egui::Color32::from_rgb(55, 55, 65),
            "btn_min",
        );
        if min_resp.clicked() {
            ctx.send_viewport_cmd(egui::ViewportCommand::Minimized(true));
        }

        // Maximize  ☐
        let max_resp = draw_btn(
            1,
            "☐",
            egui::Color32::from_rgb(180, 180, 190),
            egui::Color32::from_rgb(55, 55, 65),
            "btn_max",
        );
        if max_resp.clicked() {
            let is_maximized = ctx.input(|i| i.viewport().maximized).unwrap_or(false);
            ctx.send_viewport_cmd(egui::ViewportCommand::Maximized(!is_maximized));
        }

        // Close  ✕  — red hover background for visibility
        let close_resp = draw_btn(
            2,
            "✕",
            egui::Color32::from_rgb(230, 120, 120),
            egui::Color32::from_rgb(200, 50, 50),
            "btn_close",
        );
        if close_resp.clicked() {
            ctx.send_viewport_cmd(egui::ViewportCommand::Close);
        }
    }
}
