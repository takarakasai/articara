//! Script console panel — VSCode-terminal-style embedded Rhai REPL.

use eframe::egui;

#[cfg(feature = "scripting")]
use crate::scripting_model::ModelScriptEngine;

use super::{ArticaraApp, ScriptLine};

// ── Terminal colour palette (VSCode Dark+ inspired) ─────────────────────
const BG: egui::Color32 = egui::Color32::from_rgb(30, 30, 30);       // #1e1e1e
const FG: egui::Color32 = egui::Color32::from_rgb(204, 204, 204);    // #cccccc
const FG_INPUT: egui::Color32 = egui::Color32::from_rgb(86, 182, 194);  // #56b6c2 (cyan)
const FG_ERROR: egui::Color32 = egui::Color32::from_rgb(244, 71, 71);   // #f44747
const FG_SYSTEM: egui::Color32 = egui::Color32::from_rgb(106, 153, 85); // #6a9955 (green)
const FG_PROMPT: egui::Color32 = egui::Color32::from_rgb(86, 182, 194); // cyan
const FONT_SIZE: f32 = 13.0;
const INPUT_BG: egui::Color32 = egui::Color32::from_rgb(37, 37, 38);   // slightly lighter

impl ArticaraApp {
    /// Draw the script console as a VSCode-terminal-style window.
    #[cfg(feature = "scripting")]
    pub(super) fn draw_script_console(&mut self, ctx: &egui::Context) {
        if !self.show_script_console {
            return;
        }

        let mut open = self.show_script_console;
        let frame = egui::Frame::new()
            .fill(BG)
            .inner_margin(egui::Margin::same(4))
            .corner_radius(4.0);

        egui::Window::new("Terminal")
            .open(&mut open)
            .frame(frame)
            .default_size([600.0, 340.0])
            .min_size([300.0, 150.0])
            .resizable(true)
            .collapsible(true)
            .title_bar(true)
            .show(ctx, |ui| {
                // Lazy-init engine
                #[cfg(feature = "scripting")]
                if self.script_engine.is_none() {
                    self.script_engine = Some(ModelScriptEngine::new());
                    // Welcome message
                    self.script_output.push(ScriptLine::System(
                        "Articara Script Console (Rhai)  — type help() for commands".into(),
                    ));
                    self.script_scroll_to_bottom = true;
                }

                // Sync model into engine
                #[cfg(feature = "scripting")]
                if let Some(eng) = &mut self.script_engine {
                    if let Some(model) = &self.model {
                        eng.set_model(model.clone());
                    }
                }

                let mono = egui::FontId::monospace(FONT_SIZE);

                // ── Tab bar (mimics VSCode terminal tabs) ──
                ui.horizontal(|ui| {
                    ui.visuals_mut().override_text_color = Some(FG);
                    let tab_frame = egui::Frame::new()
                        .fill(egui::Color32::from_rgb(37, 37, 38))
                        .inner_margin(egui::Margin::symmetric(8, 2))
                        .corner_radius(egui::CornerRadius {
                            nw: 4,
                            ne: 4,
                            sw: 0,
                            se: 0,
                        });
                    tab_frame.show(ui, |ui| {
                        ui.label(
                            egui::RichText::new("⌥ rhai")
                                .font(egui::FontId::monospace(11.0))
                                .color(FG),
                        );
                    });

                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        if ui
                            .add(egui::Button::new(
                                egui::RichText::new("🗑").color(FG).font(egui::FontId::proportional(12.0)),
                            ).frame(false))
                            .on_hover_text("Clear")
                            .clicked()
                        {
                            self.script_output.clear();
                        }
                    });
                });

                ui.add_space(2.0);

                // ── Output area (scrollable) ──
                let available = ui.available_height() - 28.0; // reserve space for input row
                let scroll_area = egui::ScrollArea::vertical()
                    .max_height(available.max(60.0))
                    .stick_to_bottom(true)
                    .auto_shrink([false; 2]);

                scroll_area.show(ui, |ui| {
                    ui.set_min_width(ui.available_width());

                    // Paint dark background behind all lines
                    let rect = ui.available_rect_before_wrap();
                    ui.painter().rect_filled(rect, 0.0, BG);

                    for line in &self.script_output {
                        match line {
                            ScriptLine::Input(s) => {
                                ui.horizontal_wrapped(|ui| {
                                    ui.label(
                                        egui::RichText::new("❯ ")
                                            .font(mono.clone())
                                            .color(FG_PROMPT),
                                    );
                                    ui.label(
                                        egui::RichText::new(s)
                                            .font(mono.clone())
                                            .color(FG_INPUT),
                                    );
                                });
                            }
                            ScriptLine::Output(s) => {
                                ui.label(
                                    egui::RichText::new(s)
                                        .font(mono.clone())
                                        .color(FG),
                                );
                            }
                            ScriptLine::Error(s) => {
                                ui.label(
                                    egui::RichText::new(s)
                                        .font(mono.clone())
                                        .color(FG_ERROR),
                                );
                            }
                            ScriptLine::System(s) => {
                                ui.label(
                                    egui::RichText::new(s)
                                        .font(mono.clone())
                                        .color(FG_SYSTEM),
                                );
                            }
                        }
                    }

                    // Scroll anchor
                    if self.script_scroll_to_bottom {
                        ui.scroll_to_cursor(Some(egui::Align::BOTTOM));
                        self.script_scroll_to_bottom = false;
                    }
                });

                // ── Input row ──
                let input_frame = egui::Frame::new()
                    .fill(INPUT_BG)
                    .inner_margin(egui::Margin::symmetric(4, 2))
                    .corner_radius(2.0);

                input_frame.show(ui, |ui| {
                    let mut submit = false;

                    ui.horizontal(|ui| {
                        ui.label(
                            egui::RichText::new("❯")
                                .font(mono.clone())
                                .color(FG_PROMPT),
                        );

                        let input_id = ui.id().with("term_input");
                        let re = ui.add(
                            egui::TextEdit::singleline(&mut self.script_input)
                                .desired_width(ui.available_width())
                                .font(mono.clone())
                                .text_color(FG_INPUT)
                                .frame(egui::Frame::NONE)
                                .return_key(None::<egui::KeyboardShortcut>) // Don't auto-surrender focus
                                .hint_text(
                                    egui::RichText::new("Type Rhai code…")
                                        .color(egui::Color32::from_rgb(90, 90, 90)),
                                )
                                .id(input_id),
                        );

                        // Auto-focus when console is first shown
                        if self.show_script_console && !re.has_focus() {
                            ui.memory_mut(|m| m.request_focus(input_id));
                        }

                        // Enter → submit (detect key press while widget has focus)
                        if re.has_focus()
                            && ui.input(|i| i.key_pressed(egui::Key::Enter))
                        {
                            submit = true;
                        }

                        // Up/Down arrow for history navigation
                        if re.has_focus() {
                            let up = ui.input(|i| i.key_pressed(egui::Key::ArrowUp));
                            let down = ui.input(|i| i.key_pressed(egui::Key::ArrowDown));
                            if up && !self.script_history.is_empty() {
                                if self.script_history_idx < self.script_history.len() {
                                    self.script_history_idx += 1;
                                }
                                let i = self.script_history.len() - self.script_history_idx;
                                self.script_input = self.script_history[i].clone();
                            }
                            if down {
                                if self.script_history_idx > 1 {
                                    self.script_history_idx -= 1;
                                    let i = self.script_history.len() - self.script_history_idx;
                                    self.script_input = self.script_history[i].clone();
                                } else {
                                    self.script_history_idx = 0;
                                    self.script_input.clear();
                                }
                            }
                        }
                    });

                    if submit && !self.script_input.is_empty() {
                        let src = self.script_input.clone();
                        self.script_input.clear();
                        self.script_history_idx = 0;

                        // Add to history (avoid duplicating consecutive)
                        if self.script_history.last().map_or(true, |last| last != &src) {
                            self.script_history.push(src.clone());
                        }

                        // Handle built-in commands
                        let trimmed = src.trim();
                        if trimmed == "clear" || trimmed == "clear()" {
                            self.script_output.clear();
                            self.script_scroll_to_bottom = true;
                            return;
                        }
                        if trimmed == "help" || trimmed == "help()" {
                            self.emit_help();
                            self.script_scroll_to_bottom = true;
                            return;
                        }

                        // Echo input
                        self.script_output.push(ScriptLine::Input(src.clone()));

                        #[cfg(feature = "scripting")]
                        if let Some(eng) = &mut self.script_engine {
                            match eng.eval(&src) {
                                Ok(lines) => {
                                    for line in lines {
                                        self.script_output.push(ScriptLine::Output(line));
                                    }
                                    // Sync model back from engine → app
                                    if let Some(model) = eng.model() {
                                        self.model = Some(model.clone());
                                        self.needs_upload = true;
                                    }
                                }
                                Err(e) => {
                                    self.script_output.push(ScriptLine::Error(e));
                                }
                            }
                        }

                        // Bound log size
                        while self.script_output.len() > 1000 {
                            self.script_output.remove(0);
                        }

                        self.script_scroll_to_bottom = true;
                    }
                });
            });
        self.show_script_console = open;
    }

    /// Emit help text as system lines.
    #[cfg(feature = "scripting")]
    fn emit_help(&mut self) {
        let lines = [
            "──────────── Articara Script Console ────────────",
            "",
            "  Model:",
            "    load(path)              Load URDF/SDF/MJCF/USD",
            "    model_name()            Model name",
            "    has_model()             Whether a model is loaded",
            "",
            "  Links:",
            "    link_names()            All link names",
            "    num_links()             Link count",
            "    link_pos(name)          Link world position [x,y,z]",
            "    link_rpy(name)          Link orientation [r,p,y]",
            "",
            "  Joints:",
            "    joint_names()           All joint names",
            "    num_joints()            Joint count",
            "    joint_pos(name)         Get position by name",
            "    joint_pos_idx(i)        Get position by index",
            "    set_joint(name, val)    Set position by name",
            "    set_joint_idx(i, val)   Set position by index",
            "    joint_positions()       All positions as array",
            "    set_joints(array)       Set all from array",
            "    joint_limits(name)      [lower, upper]",
            "    joint_type(name)        Type string",
            "",
            "  Kinematics:",
            "    fk()                    FK map {name: [x,y,z]}",
            "    ik(link, x, y, z)       10 IK steps",
            "    ik_steps(link, x,y,z,n) n IK steps → error",
            "",
            "  Constraints:",
            "    add_loop_closure(name, linkA, ox,oy,oz, linkB, ox,oy,oz)",
            "    loop_closure_error()    Current constraint error",
            "    num_loop_closures()     Constraint count",
            "",
            "  Export:",
            "    export_urdf(path)       Export URDF",
            "    export_sdf(path)        Export SDF",
            "    export_mjcf(path)       Export MJCF",
            "",
            "  Math:  sin cos sqrt abs atan2 clamp to_deg to_rad PI()",
            "         min_f max_f dist(ax,ay,az,bx,by,bz)",
            "",
            "  Console:  clear  help  ↑/↓ history",
            "──────────────────────────────────────────────────",
        ];
        for l in lines {
            self.script_output.push(ScriptLine::System(l.into()));
        }
    }

    /// No-op when scripting feature is disabled.
    #[cfg(not(feature = "scripting"))]
    pub(super) fn draw_script_console(&mut self, _ctx: &egui::Context) {}
}
