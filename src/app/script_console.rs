//! Script console panel — VSCode-terminal-style embedded Rhai REPL.

use eframe::egui;

#[cfg(feature = "scripting")]
use articara::scripting_model::ModelScriptEngine;

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
    /// Draw the script console as a docked bottom panel (VSCode-terminal style).
    #[cfg(feature = "scripting")]
    pub(super) fn draw_script_console(&mut self, ui: &mut egui::Ui) {
        if !self.show_script_console {
            return;
        }

        let frame = egui::Frame::new()
            .fill(BG)
            .inner_margin(egui::Margin::same(4))
            .corner_radius(0.0);

        egui::Panel::bottom("script_console_panel")
            .default_size(220.0)
            .size_range(80.0..=600.0)
            .resizable(true)
            .frame(frame)
            .show_inside(ui, |ui| {
                // egui Panel persists state by storing `inner_response.response.rect`
                // (the *content* rect) rather than the panel rect itself, so a
                // user-resized panel snaps back to whatever height the contents
                // ended up using last frame. Forcing the inner ui's min height
                // to its max fixes that — the response rect now extends to the
                // panel's full allocated height, and PanelState faithfully
                // round-trips the resized value.
                ui.set_min_height(ui.max_rect().height());
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

                // ── Header bar (mimics VSCode terminal header) ──
                ui.horizontal(|ui| {
                    ui.visuals_mut().override_text_color = Some(FG);

                    // "TERMINAL" label
                    ui.label(
                        egui::RichText::new("TERMINAL")
                            .font(egui::FontId::proportional(11.0))
                            .color(egui::Color32::from_rgb(150, 150, 150)),
                    );

                    ui.add_space(8.0);

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
                        // Close button (×)
                        if ui
                            .add(egui::Button::new(
                                egui::RichText::new("✕").color(FG).font(egui::FontId::proportional(12.0)),
                            ).frame(false))
                            .on_hover_text("Close")
                            .clicked()
                        {
                            self.show_script_console = false;
                        }

                        // Clear button (🗑)
                        if ui
                            .add(egui::Button::new(
                                egui::RichText::new("🗑").color(FG).font(egui::FontId::proportional(12.0)),
                            ).frame(false))
                            .on_hover_text("Clear")
                            .clicked()
                        {
                            self.script_output.clear();
                        }

                        // Run script file (📂) — opens a file dialog. The
                        // dialog itself is shown by the main update() loop
                        // via the borrow on `dlg_open_script`; once the user
                        // confirms, `pending_script_run` is populated and
                        // we read+eval it below in this same panel draw.
                        if ui
                            .add(egui::Button::new(
                                egui::RichText::new("📂").color(FG).font(egui::FontId::proportional(12.0)),
                            ).frame(false))
                            .on_hover_text("Run script from file (.rhai)")
                            .clicked()
                        {
                            let start = if self.script_path.is_empty() {
                                Some(std::path::PathBuf::from("scripts"))
                            } else {
                                Some(std::path::PathBuf::from(&self.script_path))
                            };
                            self.dlg_open_script.open(
                                "Run Rhai Script",
                                super::file_dialog::FileDialogMode::Open,
                                start.as_deref(),
                                &["rhai"],
                            );
                        }
                    });
                });

                ui.add_space(2.0);

                // ── Run pending script file (set by the 📂 button) ──
                // Read the file off disk and eval it as a single block. We
                // echo the path and the source under it so users can scroll
                // back to see exactly what was run.
                if let Some(path) = self.pending_script_run.take() {
                    self.script_output.push(ScriptLine::System(format!(
                        "── Running script: {} ──",
                        path.display(),
                    )));
                    match std::fs::read_to_string(&path) {
                        Ok(source) => {
                            // Echo the source so the run is reproducible from
                            // history alone — users often re-tune by copying
                            // a previous block out of the console.
                            for line in source.lines() {
                                self.script_output.push(ScriptLine::Input(line.to_string()));
                            }
                            #[cfg(feature = "scripting")]
                            #[allow(unused_assignments)]
                            let mut script_overrides_pending: Option<
                                articara::scripting_model::ScriptOverrides,
                            > = None;
                            #[cfg(feature = "scripting")]
                            if let Some(eng) = &mut self.script_engine {
                                #[cfg(feature = "mujoco")]
                                eng.set_mujoco_sim(self.mujoco_sim.take());
                                eng.set_gait_controller(self.gait.controller.take());
                                let result = eng.eval(&source);
                                #[cfg(feature = "mujoco")]
                                {
                                    self.mujoco_sim = eng.take_mujoco_sim();
                                }
                                self.gait.controller = eng.take_gait_controller();
                                script_overrides_pending = Some(eng.drain_overrides());
                                match result {
                                    Ok(lines) => {
                                        for line in lines {
                                            self.script_output.push(ScriptLine::Output(line));
                                        }
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
                            #[cfg(feature = "scripting")]
                            if let Some(ov) = script_overrides_pending.take() {
                                self.apply_script_overrides(ov);
                            }
                        }
                        Err(e) => {
                            self.script_output.push(ScriptLine::Error(format!(
                                "Failed to read {}: {e}",
                                path.display(),
                            )));
                        }
                    }
                    self.script_scroll_to_bottom = true;
                }

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

                        // Tab → completion
                        if re.has_focus()
                            && ui.input(|i| i.key_pressed(egui::Key::Tab))
                        {
                            #[cfg(feature = "scripting")]
                            if let Some(eng) = &self.script_engine {
                                let input = &self.script_input;
                                // Extract the token being typed (last word-like fragment)
                                let token_start = input.rfind(|c: char| !c.is_alphanumeric() && c != '_')
                                    .map(|i| i + 1)
                                    .unwrap_or(0);
                                let prefix = &input[token_start..];

                                if !prefix.is_empty() {
                                    let candidates = eng.completion_candidates();
                                    let matches: Vec<String> = candidates
                                        .iter()
                                        .filter(|c| c.starts_with(prefix) && c.len() > prefix.len())
                                        .cloned()
                                        .collect();

                                    if matches.len() == 1 {
                                        // Single match → auto-complete
                                        let completed = &matches[0];
                                        let suffix = &completed[prefix.len()..];
                                        self.script_input.push_str(suffix);
                                        self.script_tab_candidates.clear();
                                    } else if matches.len() > 1 {
                                        // Multiple matches → complete common prefix, show candidates
                                        let common = Self::longest_common_prefix(&matches);
                                        if common.len() > prefix.len() {
                                            let suffix = &common[prefix.len()..];
                                            self.script_input.push_str(suffix);
                                        }
                                        self.script_tab_candidates = matches;
                                        // Show candidates in output
                                        let display = self.script_tab_candidates
                                            .chunks(6)
                                            .map(|row| row.join("  "))
                                            .collect::<Vec<_>>()
                                            .join("\n");
                                        self.script_output.push(ScriptLine::System(display));
                                        self.script_scroll_to_bottom = true;
                                    }
                                }
                            }
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
                            // Hand the live MuJoCo sim to the engine for the
                            // duration of the eval so scripts can drive it
                            // directly. Cleared on the way out, regardless
                            // of script success.
                            #[cfg(feature = "mujoco")]
                            eng.set_mujoco_sim(self.mujoco_sim.take());
                            eng.set_gait_controller(self.gait.controller.take());

                            let eval_result = eng.eval(&src);

                            #[cfg(feature = "mujoco")]
                            {
                                self.mujoco_sim = eng.take_mujoco_sim();
                            }
                            self.gait.controller = eng.take_gait_controller();
                            let pending_ov = eng.drain_overrides();

                            match eval_result {
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
                            // End the `eng` borrow before mutating other
                            // ArticaraApp fields via apply_script_overrides.
                            drop(eng);
                            self.apply_script_overrides(pending_ov);
                        }

                        // Bound log size
                        while self.script_output.len() > 1000 {
                            self.script_output.remove(0);
                        }

                        self.script_scroll_to_bottom = true;
                    }
                });
            });
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
            "  Mesh Reduction (method: \"qem\"|\"edge\"|\"cluster\"):",
            "    reduce_mesh(link, vi, ratio)        QEM (default)",
            "    reduce_mesh(link, vi, ratio, method) With algorithm",
            "    reduce_collision_mesh(link,ci,r,[m]) Collision mesh",
            "    reduce_all_meshes(ratio, [method])   All meshes",
            "",
            "  Mesh Decomposition:",
            "    decompose_vhacd(link, ci)            V-HACD (default)",
            "    decompose_vhacd(link, ci, max_hulls) V-HACD with limit",
            "    decompose_spheres(link, ci)          Sphere tree",
            "    decompose_spheres(link, ci, max_n)   Sphere tree with limit",
            "",
            "  MuJoCo Sim:",
            "    mj_start()                  Construct sim (default options)",
            "    mj_stop()                   Destroy sim, restore pose",
            "    mj_active()                 Whether a sim is running",
            "    mj_step(n)                  Advance n physics frames",
            "    mj_step_back(n)             Replay n frames backwards",
            "    mj_timestep()               Native physics dt (s)",
            "    mj_history_len()            Frames available for step_back",
            "    mj_trace_len()              Samples in (q,q̇,τ) trace ring",
            "    mj_set_trace_max(n)         Resize trace ring (returns n)",
            "    mj_gravity_compensation(on) Toggle τ_grav feedforward",
            "    save_peaks_csv(path)        Write trace to CSV; rows or -1",
            "",
            "  Quadruped gait (CHAMP-equivalent):",
            "    gait_setup()                 Auto-detect from default foot links (FL/FR/RL/RR_foot)",
            "    gait_setup_with_feet(fl,fr,rl,rr)  Same but custom names",
            "    gait_set_velocity(vx,vy,wz)  Body-frame velocity command",
            "    gait_start() / gait_stop()   Drive joint targets while sim runs",
            "    gait_running() / gait_active()  Bool — currently driving / built",
            "    gait_set_cycle_period(s) / gait_set_swing_height(m)",
            "    gait_set_duty(0..1) / gait_set_max_step(m)",
            "    gait_set_knee_pattern(\"<<|<>|><|>>\")  Front/rear knee bend",
            "    gait_knee_pattern()          Read back as shorthand string",
            "",
            "  Async timeline (UI animates the queue, viewport reflects it):",
            "    mj_async_step_seconds(s)    Queue advancing the sim by s seconds",
            "    mj_async_step_frames(n)     Queue advancing by n physics frames",
            "    mj_async_set_position_target(joint, q)   Queue setting target",
            "    mj_async_print(msg)         Queue a console message",
            "    mj_async_save_csv(path)     Queue a CSV save",
            "    mj_async_pending()          Ops still queued",
            "    mj_async_clear()            Drop all queued ops; returns count",
            "",
            "  Pose / Force:",
            "    play_pose(name)             Smooth transition (saved dur)",
            "    play_pose(name, dur)        Transition with explicit duration",
            "    transition_in_progress()    Bool",
            "    transition_progress()       0..1 (-1 if idle)",
            "    play_sequence(name)         Chained pose sequence playback",
            "    sequence_in_progress()      Bool",
            "    sequence_progress()         0..1 (-1 if idle)",
            "    apply_force(link,fx,fy,fz,tx,ty,tz,dur)  World-frame pulse",
            "    cancel_force(link)          Stop active pulse on link",
            "",
            "  Joint peaks (max |τ| / |q̇| since last reset):",
            "    reset_peaks()               Clear all peaks",
            "    peak_torque(joint)          N·m or N",
            "    peak_velocity(joint)        rad/s or m/s",
            "    peaks()                     Map: name → [tau_abs, qvel_abs]",
            "",
            "  Actuator / motor properties (per joint):",
            "    set_actuator_mode(joint, \"Position|Velocity|Torque|ComputedTorque\")",
            "    set_actuator_mode_all(mode)    Bulk variant; returns n joints",
            "    set_kp(joint, kp)           Position/CT-mode P gain",
            "    set_kv(joint, kv)           Position/Velocity/CT-mode D gain",
            "    set_armature(joint, I)      Reflected rotor inertia (kg·m²)",
            "    set_joint_damping(joint, b) Passive damping (N·m·s/rad)",
            "    set_kp_all(kp) … set_joint_damping_all(b)  → returns n joints",
            "    set_position_target(joint, q)",
            "    set_velocity_target(joint, qd)",
            "    set_torque_target(joint, tau)",
            "",
            "  Math:  sin cos sqrt abs atan2 clamp to_deg to_rad PI()",
            "         min_f max_f dist(ax,ay,az,bx,by,bz)",
            "",
            "  Console:  clear  help  ↑/↓ history  Tab completion",
            "  Run a file: 📂 button at top-right (or set up your own .rhai)",
            "──────────────────────────────────────────────────",
        ];
        for l in lines {
            self.script_output.push(ScriptLine::System(l.into()));
        }
    }

    /// No-op when scripting feature is disabled.
    #[cfg(not(feature = "scripting"))]
    pub(super) fn draw_script_console(&mut self, _ui: &mut egui::Ui) {}

    /// Find the longest common prefix among a set of strings.
    fn longest_common_prefix(strings: &[String]) -> String {
        if strings.is_empty() {
            return String::new();
        }
        let first = &strings[0];
        let mut len = first.len();
        for s in &strings[1..] {
            len = len.min(s.len());
            for (i, (a, b)) in first.chars().zip(s.chars()).enumerate() {
                if a != b {
                    len = len.min(i);
                    break;
                }
            }
        }
        first[..len].to_string()
    }
}
