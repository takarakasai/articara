//! Motion editor: a docked bottom panel that puts a sequence on a timeline.
//!
//! Keyframes are the sequence's steps at their absolute times. The
//! playhead previews the motion kinematically (no simulation needed) by
//! driving the model's joint positions; keys are added from the current
//! posture (pose it with the joint sliders or IK first), dragged in time,
//! re-captured or deleted, and each segment's curve can be changed. The
//! plot shows position / velocity / required torque per joint against the
//! joint's limits, and [`articara::motion::check`] lists every interval a
//! real robot couldn't follow. The trajectory exports as CSV.
//!
//! With MuJoCo, "Run in MuJoCo" rebuilds the sim at the start pose, holds
//! it for a settle time, plays the sequence and records the result: the
//! recorded q / q̇ / applied τ overlay the plan, and the same checks run on
//! them — the physical torques include contact, which the plan's
//! fixed-base estimate can't — plus per-joint tracking error and base
//! height / tilt. Edit, run again, compare: the loop before real hardware.

use std::hash::{Hash, Hasher};

use eframe::egui;
use egui_plot::{HLine, Line, LineStyle, Plot, PlotPoints, VLine};

use articara::motion::{self, Issue, IssueKind, Series, Trajectory};
use misarta::trajectory::InterpolationKind;

use super::ArticaraApp;
use super::file_dialog::{FileDialog, FileDialogMode, FileDialogResult};

/// Which signal the plot shows.
#[derive(Clone, Copy, PartialEq, Eq)]
pub(super) enum PlotSignal {
    Position,
    Velocity,
    Torque,
}

impl PlotSignal {
    fn label(self) -> &'static str {
        match self {
            PlotSignal::Position => "Position",
            PlotSignal::Velocity => "Velocity",
            PlotSignal::Torque => "Torque",
        }
    }
}

/// How the robot is held in a MuJoCo run.
#[derive(Clone, Copy, PartialEq, Eq)]
#[cfg_attr(not(feature = "mujoco"), allow(dead_code))]
enum RunBase {
    /// Free base standing on a ground plane (enabled for the run).
    Ground,
    /// Base welded in place — a hanging test for limb motions.
    Fixed,
}

/// Which result the issue list shows.
#[derive(Clone, Copy, PartialEq, Eq)]
enum ResultTab {
    Plan,
    Sim,
}

/// One MuJoCo run of a sequence and what was recorded. Times are plan time
/// (sim time minus the settle period), so they line up with the timeline.
#[cfg_attr(not(feature = "mujoco"), allow(dead_code))]
struct SimRun {
    seq: usize,
    /// Analysis key of the plan that was run; differs once it's edited.
    plan_key: u64,
    settle: f64,
    /// Sim time at which recording stops.
    end: f64,
    last_sim_time: f64,
    times: Vec<f64>,
    q: Vec<Vec<f64>>,
    qd: Vec<Vec<f64>>,
    tau: Vec<Vec<f64>>,
    base_z: Vec<f64>,
    /// Angle between the root link's z axis and world up (rad).
    tilt: Vec<f64>,
    done: bool,
    issues: Vec<Issue>,
    /// `(joint, max |q_sim − q_plan|, time)`, worst first.
    tracking: Vec<(usize, f64, f64)>,
}

/// Recording continues this long after the sequence ends (s), to catch
/// overshoot and falls.
#[cfg(feature = "mujoco")]
const RUN_TAIL: f64 = 0.5;

/// Analysis of the edited sequence, recomputed only when its inputs change.
struct Analysis {
    key: u64,
    trajectory: Result<Trajectory, String>,
    series: Option<Series>,
    issues: Vec<Issue>,
}

pub(super) struct MotionEditorState {
    pub open: bool,
    /// Index into `model.sequences`.
    pub sequence: Option<usize>,
    /// Configuration the sequence starts from (full joint vector).
    start_q: Option<Vec<f64>>,
    time: f64,
    playing: bool,
    looping: bool,
    speed: f32,
    selected_key: Option<usize>,
    /// Keyframe being dragged on the timeline.
    dragging_key: Option<usize>,
    signal: PlotSignal,
    /// Joint indices shown in the plot.
    shown_joints: std::collections::BTreeSet<usize>,
    new_key_kind: InterpolationKind,
    csv_rate_hz: f64,
    dlg_save_csv: FileDialog,
    analysis: Option<Analysis>,
    /// MuJoCo owns the joint positions this frame (preview must not write).
    sim_active: bool,
    /// Hold time before the sequence starts in a MuJoCo run (s).
    #[cfg_attr(not(feature = "mujoco"), allow(dead_code))]
    settle: f64,
    #[cfg_attr(not(feature = "mujoco"), allow(dead_code))]
    run_requested: bool,
    #[cfg_attr(not(feature = "mujoco"), allow(dead_code))]
    run_base: RunBase,
    run: Option<SimRun>,
    show_sim: bool,
    tab: ResultTab,
}

impl Default for MotionEditorState {
    fn default() -> Self {
        Self {
            open: false,
            sequence: None,
            start_q: None,
            time: 0.0,
            playing: false,
            looping: false,
            speed: 1.0,
            selected_key: None,
            dragging_key: None,
            signal: PlotSignal::Position,
            shown_joints: Default::default(),
            new_key_kind: InterpolationKind::QuinticSmooth,
            csv_rate_hz: 100.0,
            dlg_save_csv: FileDialog::new("dlg_save_motion_csv"),
            analysis: None,
            sim_active: false,
            settle: 1.0,
            run_requested: false,
            run_base: RunBase::Ground,
            run: None,
            show_sim: true,
            tab: ResultTab::Plan,
        }
    }
}

/// Sampling step of the analysis (s).
const ANALYSIS_DT: f64 = 0.01;
/// Keyframe times snap to this grid (s).
const TIME_SNAP: f64 = 0.01;

fn snap(t: f64) -> f64 {
    (t / TIME_SNAP).round() * TIME_SNAP
}

/// Fingerprint of everything the analysis depends on.
fn analysis_key(model: &articara::robot::RobotModel, seq: usize, start_q: &[f64]) -> u64 {
    let mut h = std::collections::hash_map::DefaultHasher::new();
    let s = &model.sequences[seq];
    format!("{s:?}").hash(&mut h);
    for step in &s.steps {
        if let Some(p) = model.poses.iter().find(|p| p.name == step.pose_name) {
            format!("{:?}", p.angles).hash(&mut h);
        }
    }
    start_q.iter().for_each(|q| q.to_bits().hash(&mut h));
    for j in &model.joints {
        (j.lower.to_bits(), j.upper.to_bits(), j.velocity.to_bits(), j.effort.to_bits()).hash(&mut h);
    }
    h.finish()
}

fn issue_color(kind: IssueKind) -> egui::Color32 {
    match kind {
        IssueKind::Position => egui::Color32::from_rgb(230, 70, 70),
        IssueKind::Velocity => egui::Color32::from_rgb(240, 160, 40),
        IssueKind::Torque => egui::Color32::from_rgb(200, 90, 230),
    }
}

fn series_color(i: usize) -> egui::Color32 {
    const PALETTE: [egui::Color32; 6] = [
        egui::Color32::from_rgb(90, 160, 255),
        egui::Color32::from_rgb(110, 210, 120),
        egui::Color32::from_rgb(255, 200, 80),
        egui::Color32::from_rgb(240, 120, 200),
        egui::Color32::from_rgb(120, 220, 220),
        egui::Color32::from_rgb(200, 160, 255),
    ];
    PALETTE[i % PALETTE.len()]
}

impl ArticaraApp {
    /// Open the motion editor on sequence `seq`.
    pub(super) fn open_motion_editor(&mut self, seq: usize) {
        let m = &mut self.motion;
        m.open = true;
        if m.sequence != Some(seq) {
            m.sequence = Some(seq);
            m.start_q = None;
            m.time = 0.0;
            m.selected_key = None;
            m.analysis = None;
        }
    }

    /// Docked bottom panel; call between the status bar and the side panels.
    pub(super) fn draw_motion_editor(&mut self, ui: &mut egui::Ui) {
        if !self.motion.open {
            return;
        }
        let Some(mut model) = self.model.take() else {
            self.motion.open = false;
            return;
        };
        #[cfg(feature = "mujoco")]
        {
            self.motion.sim_active = self.sim.mujoco_sim.is_some();
        }
        let mut edit: Option<String> = None;
        egui::Panel::bottom("motion_editor_panel")
            .default_size(300.0)
            .size_range(160.0..=700.0)
            .resizable(true)
            .show_inside(ui, |ui| {
                edit = self.motion_editor_contents(ui, &mut model);
            });
        self.motion_csv_dialog(ui.ctx(), &model);
        self.model = Some(model);
        if let Some(desc) = edit {
            self.mark_edit(&desc);
        }
        #[cfg(feature = "mujoco")]
        {
            if std::mem::take(&mut self.motion.run_requested) {
                self.motion_run_in_mujoco();
            }
            if self.motion.run.as_ref().is_some_and(|r| !r.done) {
                self.motion_record_run();
                ui.ctx().request_repaint();
            }
        }
    }

    /// Restart MuJoCo at the editor's start pose and play the sequence
    /// after the settle time, recording from scratch.
    #[cfg(feature = "mujoco")]
    fn motion_run_in_mujoco(&mut self) {
        let Some(seq_idx) = self.motion.sequence else { return };
        self.stop_dynamics_sim();
        let Some(model) = self.model.as_mut() else { return };
        let start_q = self.motion.start_q.clone().unwrap_or_else(|| model.joint_positions.clone());
        model.joint_positions = start_q.clone();
        let seq = model.sequences[seq_idx].clone();
        let (anim, duration) = match (
            motion::keyframe_animation(model, &seq, &start_q, self.motion.settle),
            Trajectory::from_sequence(model, &seq, &start_q),
        ) {
            (Ok(a), Ok(t)) => (a, t.duration()),
            (Err(e), _) | (_, Err(e)) => {
                self.status_message = format!("Run in MuJoCo: {e}");
                return;
            }
        };
        let plan_key = analysis_key(model, seq_idx, &start_q);
        // The run's conditions come from the editor, not from whatever the
        // viewport happens to show; the user's base lock is restored once
        // the sim is built.
        let saved_lock = self.sim.mujoco_base_locked;
        match self.motion.run_base {
            RunBase::Ground => {
                if !self.view.show_ground_plane {
                    self.view.show_ground_plane = true;
                    // Turned back off when the sim stops.
                    self.view.ground_plane_auto = true;
                }
            }
            RunBase::Fixed => self.sim.mujoco_base_locked = [true; 6],
        }
        self.start_mujoco_sim();
        self.sim.mujoco_base_locked = saved_lock;
        let Some(sim) = self.sim.mujoco_sim.as_mut() else { return };
        let end = self.motion.settle + duration + RUN_TAIL;
        sim.set_trace_max((end / sim.timestep().max(1e-5)) as usize + 1000);
        sim.start_sequence(anim, seq.name.clone());
        self.sim.dynamics_sim_paused = false;
        self.motion.playing = false;
        self.motion.tab = ResultTab::Sim;
        self.motion.run = Some(SimRun {
            seq: seq_idx,
            plan_key,
            settle: self.motion.settle,
            end,
            last_sim_time: f64::NEG_INFINITY,
            times: Vec::new(),
            q: Vec::new(),
            qd: Vec::new(),
            tau: Vec::new(),
            base_z: Vec::new(),
            tilt: Vec::new(),
            done: false,
            issues: Vec::new(),
            tracking: Vec::new(),
        });
        self.status_message = format!("MuJoCo: running '{}' after {:.1} s settle", seq.name, self.motion.settle);
    }

    /// Pull new trace frames of the active run; finish it once the sequence
    /// (plus a short tail) has played or the sim was stopped.
    #[cfg(feature = "mujoco")]
    fn motion_record_run(&mut self) {
        let Some(run) = self.motion.run.as_mut() else { return };
        let Some(sim) = self.sim.mujoco_sim.as_ref() else {
            run.done = true;
            return self.motion_finish_run();
        };
        let since = run.last_sim_time;
        for f in sim.trace().filter(|f| f.time > since) {
            run.last_sim_time = f.time;
            let t = f.time - run.settle;
            // Before the sequence starts the robot is only settling.
            if t < 0.0 {
                continue;
            }
            run.times.push(t);
            run.q.push(f.q.clone());
            run.qd.push(f.qvel.clone());
            run.tau.push(f.tau.clone());
            run.base_z.push(f.base_pos.map_or(f64::NAN, |p| p[2]));
            run.tilt.push(f.base_quat.map_or(f64::NAN, |[_, x, y, _]| (1.0 - 2.0 * (x * x + y * y)).clamp(-1.0, 1.0).acos()));
        }
        // Follow the sim on the timeline.
        self.motion.time = (run.last_sim_time - run.settle).max(0.0);
        if run.last_sim_time >= run.end {
            run.done = true;
            self.sim.dynamics_sim_paused = true;
            self.motion_finish_run();
        }
    }

    #[cfg(feature = "mujoco")]
    fn motion_finish_run(&mut self) {
        let (Some(run), Some(model)) = (self.motion.run.as_mut(), self.model.as_ref()) else { return };
        if run.times.is_empty() || run.seq >= model.sequences.len() {
            self.status_message = "MuJoCo run stopped before the sequence started".into();
            return;
        }
        let series = motion::Series::from_samples(model, run.times.clone(), &run.q, &run.qd, &run.tau);
        run.issues = motion::check(model, &series);
        let start_q = self.motion.start_q.clone().unwrap_or_default();
        if let Ok(plan) = Trajectory::from_sequence(model, &model.sequences[run.seq], &start_q) {
            run.tracking = motion::tracking_error(&plan, &series);
        }
        let worst_tilt = run.tilt.iter().copied().filter(|v| v.is_finite()).fold(0.0, f64::max);
        self.status_message = format!(
            "MuJoCo run done: {} issue(s), max tracking error {:.1}°, max tilt {:.0}°",
            run.issues.len(),
            run.tracking.first().map_or(0.0, |t| t.1.to_degrees()),
            worst_tilt.to_degrees()
        );
    }

    fn motion_editor_contents(
        &mut self,
        ui: &mut egui::Ui,
        model: &mut articara::robot::RobotModel,
    ) -> Option<String> {
        let mut edit = None;
        let st = &mut self.motion;

        // ── Sequence picker ──
        if st.sequence.is_some_and(|i| i >= model.sequences.len()) {
            st.sequence = None;
        }
        ui.horizontal(|ui| {
            ui.strong("🎞 Motion Editor");
            let current = st.sequence.map(|i| model.sequences[i].name.clone()).unwrap_or_else(|| "(choose)".into());
            egui::ComboBox::from_id_salt("motion_seq").selected_text(current).show_ui(ui, |ui| {
                for (i, s) in model.sequences.iter().enumerate() {
                    if ui.selectable_label(st.sequence == Some(i), &s.name).clicked() && st.sequence != Some(i) {
                        st.sequence = Some(i);
                        st.start_q = None;
                        st.time = 0.0;
                        st.selected_key = None;
                    }
                }
            });
            if ui.button("➕ New").on_hover_text("New empty sequence").clicked() {
                let mut n = model.sequences.len();
                let name = loop {
                    let name = format!("motion_{n}");
                    if !model.sequences.iter().any(|s| s.name == name) {
                        break name;
                    }
                    n += 1;
                };
                model.sequences.push(articara::robot::Sequence { name: name.clone(), steps: Vec::new() });
                st.sequence = Some(model.sequences.len() - 1);
                st.start_q = None;
                st.time = 0.0;
                edit = Some(format!("Create sequence '{name}'"));
            }
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                if ui.button("✖").on_hover_text("Close the motion editor").clicked() {
                    st.open = false;
                    st.playing = false;
                }
            });
        });
        let Some(seq_idx) = st.sequence else {
            ui.label("Choose or create a sequence to edit it on a timeline.");
            return edit;
        };

        // The start configuration is captured once per sequence so the
        // timeline is deterministic while the preview moves the model.
        if st.start_q.as_ref().is_none_or(|q| q.len() != model.joint_positions.len()) {
            st.start_q = Some(model.joint_positions.clone());
        }
        let start_q = st.start_q.clone().unwrap_or_default();

        // ── Analysis (cached) ──
        let key = analysis_key(model, seq_idx, &start_q);
        if st.analysis.as_ref().is_none_or(|a| a.key != key) {
            let trajectory = Trajectory::from_sequence(model, &model.sequences[seq_idx], &start_q);
            let series = trajectory.as_ref().ok().map(|t| motion::sample_series(model, t, ANALYSIS_DT));
            let issues = series.as_ref().map(|s| motion::check(model, s)).unwrap_or_default();
            if st.shown_joints.is_empty() || st.shown_joints.iter().any(|&j| j >= model.joints.len()) {
                // Default view: joints with problems, else the moving ones.
                st.shown_joints = issues.iter().map(|i| i.joint).collect();
                if st.shown_joints.is_empty() {
                    if let Some(s) = &series {
                        st.shown_joints = s
                            .joints
                            .iter()
                            .zip(&s.q)
                            .filter(|(_, q)| q.iter().any(|v| (v - q[0]).abs() > 1e-6))
                            .map(|(j, _)| *j)
                            .take(4)
                            .collect();
                    }
                }
            }
            st.analysis = Some(Analysis { key, trajectory, series, issues });
        }
        let analysis = st.analysis.as_ref().expect("analysis computed above");
        let (duration, key_times) = match &analysis.trajectory {
            Ok(t) => (t.duration(), t.keyframe_times()),
            Err(e) => {
                ui.colored_label(egui::Color32::LIGHT_RED, format!("⚠ {e}"));
                return edit;
            }
        };

        // ── Transport + key operations ──
        let mut seek: Option<f64> = None;
        ui.horizontal(|ui| {
            if ui.button(if st.playing { "⏸" } else { "▶" }).on_hover_text("Play / pause (kinematic preview)").clicked() {
                if !st.playing && st.time >= duration {
                    st.time = 0.0;
                }
                st.playing = !st.playing;
            }
            if ui.button("⏮").on_hover_text("Back to start").clicked() {
                st.playing = false;
                seek = Some(0.0);
            }
            ui.checkbox(&mut st.looping, "Loop");
            ui.add(egui::DragValue::new(&mut st.speed).range(0.1..=4.0).speed(0.05).suffix("×"));
            ui.label(format!("{:.2} / {:.2} s", st.time, duration));
            ui.separator();

            if ui
                .button("◆ Key at playhead")
                .on_hover_text("Add a keyframe at the playhead holding the current posture (pose it with the sliders or IK first)")
                .clicked()
            {
                st.playing = false;
                let angles = motion::movable_joints(model)
                    .into_iter()
                    .map(|ji| (model.joints[ji].name.clone(), model.joint_positions[ji]))
                    .collect();
                let t = snap(st.time);
                let k = motion::insert_keyframe(model, seq_idx, t, angles, st.new_key_kind);
                st.selected_key = Some(k);
                edit = Some("Add keyframe".into());
            }
            egui::ComboBox::from_id_salt("motion_new_kind")
                .selected_text(st.new_key_kind.label())
                .show_ui(ui, |ui| {
                    for k in InterpolationKind::ALL {
                        ui.selectable_value(&mut st.new_key_kind, k, k.label());
                    }
                })
                .response
                .on_hover_text("Curve for new keys");

            if let Some(k) = st.selected_key.filter(|&k| k < key_times.len()) {
                ui.separator();
                ui.label(format!("Key {k} @ {:.2} s", key_times[k]));
                if ui
                    .button("⟲ Capture")
                    .on_hover_text("Replace this key's pose with the current posture (affects every sequence using that pose)")
                    .clicked()
                {
                    let pose_name = model.sequences[seq_idx].steps[k].pose_name.clone();
                    let angles = motion::movable_joints(model)
                        .into_iter()
                        .map(|ji| (model.joints[ji].name.clone(), model.joint_positions[ji]))
                        .collect();
                    if let Some(p) = model.poses.iter_mut().find(|p| p.name == pose_name) {
                        p.angles = angles;
                        edit = Some(format!("Capture pose '{pose_name}'"));
                    }
                }
                let step = &mut model.sequences[seq_idx].steps[k];
                let mut kind = step.kind;
                egui::ComboBox::from_id_salt("motion_key_kind")
                    .selected_text(kind.label())
                    .show_ui(ui, |ui| {
                        for kk in InterpolationKind::ALL {
                            ui.selectable_value(&mut kind, kk, kk.label());
                        }
                    })
                    .response
                    .on_hover_text("Curve into this key");
                if kind != step.kind {
                    step.kind = kind;
                    edit = Some("Change keyframe curve".into());
                }
                if ui.button("🗑").on_hover_text("Delete this key (its pose is kept)").clicked() {
                    motion::delete_keyframe(&mut model.sequences[seq_idx], k);
                    st.selected_key = None;
                    edit = Some("Delete keyframe".into());
                }
            }

            #[cfg(feature = "mujoco")]
            {
                ui.separator();
                let recording = st.run.as_ref().is_some_and(|r| !r.done);
                if ui
                    .add_enabled(!recording && !key_times.is_empty(), egui::Button::new("🦾 Run in MuJoCo"))
                    .on_hover_text("Restart MuJoCo at the start pose, hold it for the settle time, play this sequence and record q / q̇ / τ to compare with the plan")
                    .clicked()
                {
                    st.run_requested = true;
                }
                egui::ComboBox::from_id_salt("motion_run_base")
                    .selected_text(match st.run_base {
                        RunBase::Ground => "On ground",
                        RunBase::Fixed => "Fixed base",
                    })
                    .show_ui(ui, |ui| {
                        ui.selectable_value(&mut st.run_base, RunBase::Ground, "On ground")
                            .on_hover_text("Free base standing on a ground plane");
                        ui.selectable_value(&mut st.run_base, RunBase::Fixed, "Fixed base")
                            .on_hover_text("Base held in place: a hanging test for limb motions");
                    });
                ui.add(egui::DragValue::new(&mut st.settle).range(0.0..=5.0).speed(0.05).prefix("settle ").suffix(" s"))
                    .on_hover_text("Hold the start pose this long before the sequence starts (lets a legged robot land)");
                if recording {
                    ui.spinner();
                }
            }
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                if ui.button("💾 CSV…").on_hover_text("Export the joint trajectory (time + one column per joint, rad / m)").clicked() {
                    st.dlg_save_csv.open("Export trajectory CSV", FileDialogMode::Save, None, &["csv"]);
                }
                ui.add(egui::DragValue::new(&mut st.csv_rate_hz).range(1.0..=2000.0).speed(10.0).suffix(" Hz"));
                if ui.button("Start = current").on_hover_text("Use the current posture as the sequence's starting point").clicked() {
                    st.playing = false;
                    st.start_q = Some(model.joint_positions.clone());
                }
            });
        });

        // ── Timeline strip ──
        let (rect, response) =
            ui.allocate_exact_size(egui::vec2(ui.available_width(), 34.0), egui::Sense::click_and_drag());
        let span = duration.max(1.0) * 1.05;
        let x_of = |t: f64| rect.left() + 8.0 + (t / span) as f32 * (rect.width() - 16.0);
        let t_of = |x: f32| (((x - rect.left() - 8.0) / (rect.width() - 16.0)) as f64 * span).max(0.0);
        let painter = ui.painter_at(rect);
        painter.rect_filled(rect, 3.0, ui.visuals().extreme_bg_color);
        let mid = rect.center().y;
        painter.line_segment(
            [egui::pos2(x_of(0.0), mid), egui::pos2(x_of(duration), mid)],
            egui::Stroke::new(2.0_f32, ui.visuals().weak_text_color()),
        );
        for issue in &analysis.issues {
            let (x0, x1) = (x_of(issue.t_start), x_of(issue.t_end).max(x_of(issue.t_start) + 2.0));
            painter.rect_filled(
                egui::Rect::from_min_max(egui::pos2(x0, rect.bottom() - 6.0), egui::pos2(x1, rect.bottom() - 2.0)),
                1.0,
                issue_color(issue.kind),
            );
        }
        if let Some(run) = st.run.as_ref().filter(|r| r.seq == seq_idx && r.done) {
            for issue in &run.issues {
                let (x0, x1) = (x_of(issue.t_start), x_of(issue.t_end).max(x_of(issue.t_start) + 2.0));
                painter.rect_filled(
                    egui::Rect::from_min_max(egui::pos2(x0, rect.bottom() - 12.0), egui::pos2(x1, rect.bottom() - 8.0)),
                    1.0,
                    issue_color(issue.kind).gamma_multiply(0.6),
                );
            }
        }
        let mut tick = 0.0;
        while tick <= span {
            let x = x_of(tick);
            painter.line_segment([egui::pos2(x, rect.top() + 2.0), egui::pos2(x, rect.top() + 7.0)], egui::Stroke::new(1.0_f32, ui.visuals().weak_text_color()));
            painter.text(egui::pos2(x + 2.0, rect.top() + 1.0), egui::Align2::LEFT_TOP, format!("{tick:.0}"), egui::FontId::monospace(9.0), ui.visuals().weak_text_color());
            tick += if span > 20.0 { 5.0 } else { 1.0 };
        }
        for (k, &t) in key_times.iter().enumerate() {
            let c = egui::pos2(x_of(t), mid);
            let r = if st.selected_key == Some(k) { 7.0 } else { 5.0 };
            let fill = if st.selected_key == Some(k) { egui::Color32::from_rgb(255, 210, 80) } else { egui::Color32::from_rgb(200, 200, 210) };
            painter.add(egui::Shape::convex_polygon(
                vec![c + egui::vec2(0.0, -r), c + egui::vec2(r, 0.0), c + egui::vec2(0.0, r), c + egui::vec2(-r, 0.0)],
                fill,
                egui::Stroke::new(1.0_f32, egui::Color32::BLACK),
            ));
        }
        let px = x_of(st.time);
        painter.line_segment([egui::pos2(px, rect.top()), egui::pos2(px, rect.bottom())], egui::Stroke::new(2.0_f32, egui::Color32::from_rgb(255, 90, 90)));

        let near_key = |pos: egui::Pos2| {
            key_times
                .iter()
                .enumerate()
                .map(|(k, &t)| (k, (x_of(t) - pos.x).abs()))
                .filter(|(_, d)| *d < 8.0)
                .min_by(|a, b| a.1.total_cmp(&b.1))
                .map(|(k, _)| k)
        };
        if response.drag_started() {
            // A drag only registers after the pointer has moved, so hit-test
            // where the press began, not where the pointer is now.
            let origin = ui.input(|i| i.pointer.press_origin()).or(response.interact_pointer_pos());
            st.dragging_key = origin.and_then(near_key);
            if let Some(k) = st.dragging_key {
                st.selected_key = Some(k);
                st.playing = false;
            }
        }
        if let Some(pos) = response.interact_pointer_pos() {
            if response.dragged() {
                match st.dragging_key {
                    Some(k) => {
                        let t = motion::move_keyframe(&mut model.sequences[seq_idx], k, snap(t_of(pos.x)));
                        seek = Some(t);
                        edit = Some("Move keyframe".into());
                    }
                    None => {
                        st.playing = false;
                        seek = Some(t_of(pos.x).min(duration));
                    }
                }
            } else if response.clicked() {
                st.playing = false;
                st.selected_key = near_key(pos);
                seek = Some(match st.selected_key {
                    Some(k) => key_times[k],
                    None => t_of(pos.x).min(duration),
                });
            }
        }
        if response.drag_stopped() {
            st.dragging_key = None;
        }

        // ── Plot + side lists ──
        let series = analysis.series.as_ref();
        ui.horizontal_top(|ui| {
            // Joint list.
            ui.vertical(|ui| {
                ui.set_width(170.0);
                ui.horizontal(|ui| {
                    for s in [PlotSignal::Position, PlotSignal::Velocity, PlotSignal::Torque] {
                        ui.selectable_value(&mut st.signal, s, s.label());
                    }
                });
                egui::ScrollArea::vertical().id_salt("motion_joints").max_height(ui.available_height()).show(ui, |ui| {
                    for ji in motion::movable_joints(model) {
                        let n_issues = analysis.issues.iter().filter(|i| i.joint == ji).count();
                        let mut on = st.shown_joints.contains(&ji);
                        let label = if n_issues > 0 { format!("⚠ {}", model.joints[ji].name) } else { model.joints[ji].name.clone() };
                        if ui.checkbox(&mut on, label).changed() {
                            if on {
                                st.shown_joints.insert(ji);
                            } else {
                                st.shown_joints.remove(&ji);
                            }
                        }
                    }
                });
            });
            ui.separator();

            // Issue list on the right; the plot takes what's left.
            let plot_width = (ui.available_width() - 270.0).max(200.0);
            ui.vertical(|ui| {
                ui.set_width(plot_width);
                if let Some(series) = series {
                    let plot = Plot::new("motion_plot")
                        .height(ui.available_height().max(80.0))
                        .allow_scroll(false)
                        .x_axis_label("t [s]")
                        .legend(egui_plot::Legend::default());
                    let resp = plot.show(ui, |plot_ui| {
                        for (n, &ji) in st.shown_joints.iter().enumerate() {
                            let Some(c) = series.joints.iter().position(|&j| j == ji) else { continue };
                            let joint = &model.joints[ji];
                            let revolute = joint.joint_type != "prismatic";
                            // Show angles in degrees; slides stay in m.
                            let scale = if revolute && st.signal != PlotSignal::Torque { 180.0 / std::f64::consts::PI } else { 1.0 };
                            let data = match st.signal {
                                PlotSignal::Position => &series.q[c],
                                PlotSignal::Velocity => &series.qd[c],
                                PlotSignal::Torque => &series.tau[c],
                            };
                            let color = series_color(n);
                            let pts: PlotPoints = series.times.iter().zip(data).map(|(t, v)| [*t, v * scale]).collect();
                            plot_ui.line(Line::new(joint.name.clone(), pts).color(color).width(1.5_f32));
                            let bounds: Vec<f64> = match st.signal {
                                PlotSignal::Position if joint.joint_type != "continuous" && joint.upper > joint.lower => vec![joint.lower, joint.upper],
                                PlotSignal::Velocity if joint.velocity > 0.0 => vec![-joint.velocity, joint.velocity],
                                PlotSignal::Torque if joint.effort > 0.0 => vec![-joint.effort, joint.effort],
                                _ => vec![],
                            };
                            for b in bounds {
                                plot_ui.hline(HLine::new("", b * scale).color(color.gamma_multiply(0.6)).style(LineStyle::dashed_loose()));
                            }
                            if let Some(run) = st.run.as_ref().filter(|r| st.show_sim && r.seq == seq_idx) {
                                let rows = match st.signal {
                                    PlotSignal::Position => &run.q,
                                    PlotSignal::Velocity => &run.qd,
                                    PlotSignal::Torque => &run.tau,
                                };
                                // Thin out dense sim traces for drawing.
                                let stride = (rows.len() / 2000).max(1);
                                let pts: PlotPoints = run
                                    .times
                                    .iter()
                                    .zip(rows)
                                    .step_by(stride)
                                    .map(|(t, r)| [*t, r.get(ji).copied().unwrap_or(f64::NAN) * scale])
                                    .collect();
                                plot_ui.line(
                                    Line::new(format!("{} (MuJoCo)", joint.name), pts)
                                        .color(color)
                                        .width(1.0_f32)
                                        .style(LineStyle::dashed_dense()),
                                );
                            }
                        }
                        plot_ui.vline(VLine::new("", st.time).color(egui::Color32::from_rgb(255, 90, 90)));
                        for &t in &key_times {
                            plot_ui.vline(VLine::new("", t).color(egui::Color32::from_gray(110)).style(LineStyle::dotted_dense()));
                        }
                        plot_ui.pointer_coordinate()
                    });
                    if resp.response.clicked() {
                        if let Some(p) = resp.inner {
                            st.playing = false;
                            seek = Some(p.x.clamp(0.0, duration));
                        }
                    }
                }
            });
            ui.separator();

            ui.vertical(|ui| {
                let run = st.run.as_ref().filter(|r| r.seq == seq_idx);
                if run.is_some() {
                    ui.horizontal(|ui| {
                        ui.selectable_value(&mut st.tab, ResultTab::Plan, "Plan");
                        ui.selectable_value(&mut st.tab, ResultTab::Sim, "MuJoCo");
                        ui.checkbox(&mut st.show_sim, "overlay");
                    });
                }
                let mut clicked: Option<&Issue> = None;
                match (st.tab, run) {
                    (ResultTab::Sim, Some(run)) if !run.done => {
                        ui.label(format!("Recording… {:.2} s", run.times.last().copied().unwrap_or(0.0)));
                    }
                    (ResultTab::Sim, Some(run)) => {
                        if run.plan_key != analysis.key {
                            ui.colored_label(egui::Color32::YELLOW, "⟳ edited since this run — run again")
                                .on_hover_text("The plan changed after this MuJoCo run; the dashed curves show the old run.");
                        }
                        let tilt = run.tilt.iter().copied().filter(|v| v.is_finite()).fold(0.0, f64::max).to_degrees();
                        let z_min = run.base_z.iter().copied().filter(|v| v.is_finite()).fold(f64::INFINITY, f64::min);
                        let tilt_color = if tilt > 45.0 { egui::Color32::LIGHT_RED } else { ui.visuals().text_color() };
                        ui.colored_label(tilt_color, format!("base: max tilt {tilt:.0}°{}", if z_min.is_finite() { format!(", min z {z_min:.3} m") } else { String::new() }))
                            .on_hover_text("Root link orientation and height over the run; a tilt past ~45° usually means it fell.");
                        if let Some(&(ji, err, t)) = run.tracking.first() {
                            ui.label(format!("max tracking error: {} {:.1}° @ {t:.2}s", model.joints[ji].name, err.to_degrees()))
                                .on_hover_text(
                                    run.tracking
                                        .iter()
                                        .take(8)
                                        .map(|&(j, e, t)| format!("{}: {:.2}° @ {t:.2}s", model.joints[j].name, e.to_degrees()))
                                        .collect::<Vec<_>>()
                                        .join("\n"),
                                );
                        }
                        clicked = issue_list(ui, model, &run.issues, "Applied torques from MuJoCo, including contact.");
                    }
                    _ => {
                        clicked = issue_list(
                            ui,
                            model,
                            &analysis.issues,
                            "Torques are fixed-base inverse dynamics: exact for arms and legs in the air, not for legs carrying the body.",
                        );
                    }
                }
                if let Some(issue) = clicked {
                    st.playing = false;
                    seek = Some(issue.t_start.min(duration));
                    st.shown_joints.insert(issue.joint);
                }
            });
        });

        // ── Playback / preview ──
        if st.playing {
            let dt = ui.input(|i| i.stable_dt).min(0.1) as f64 * st.speed as f64;
            let mut t = st.time + dt;
            if t >= duration {
                if st.looping && duration > 0.0 {
                    t %= duration;
                } else {
                    t = duration;
                    st.playing = false;
                }
            }
            seek = Some(t);
            ui.ctx().request_repaint();
        }
        if let (Some(t), Ok(traj)) = (seek, &analysis.trajectory) {
            st.time = t.clamp(0.0, duration);
            // While MuJoCo runs, it owns the joints; only the playhead moves.
            if !st.sim_active {
                model.joint_positions = traj.sample(st.time).0;
                model.enforce_mimic();
            }
        }
        edit
    }

    fn motion_csv_dialog(&mut self, ctx: &egui::Context, model: &articara::robot::RobotModel) {
        if let FileDialogResult::Confirmed(path) = self.motion.dlg_save_csv.show(ctx) {
            let result = match (self.motion.analysis.as_ref().map(|a| &a.trajectory), self.motion.start_q.as_ref()) {
                (Some(Ok(traj)), Some(_)) => {
                    let csv = motion::trajectory_csv(model, traj, self.motion.csv_rate_hz);
                    let rows = csv.lines().count().saturating_sub(1);
                    std::fs::write(&path, csv).map(|_| rows).map_err(|e| e.to_string())
                }
                _ => Err("no valid trajectory".to_string()),
            };
            self.status_message = match result {
                Ok(rows) => format!("Saved trajectory ({rows} rows) → {}", path.display()),
                Err(e) => format!("Trajectory CSV error: {e}"),
            };
        }
    }
}

/// Issue list with a summary line; returns the issue the user clicked.
fn issue_list<'a>(ui: &mut egui::Ui, model: &articara::robot::RobotModel, issues: &'a [Issue], note: &str) -> Option<&'a Issue> {
    if issues.is_empty() {
        ui.colored_label(egui::Color32::from_rgb(110, 210, 120), "✔ Within limits")
            .on_hover_text(format!("Joint range, declared velocity and declared effort are respected everywhere. {note}"));
    } else {
        ui.colored_label(egui::Color32::LIGHT_RED, format!("⚠ {} issue(s)", issues.len())).on_hover_text(note);
    }
    let mut clicked = None;
    egui::ScrollArea::vertical().id_salt("motion_issues").show(ui, |ui| {
        for issue in issues {
            let joint = &model.joints[issue.joint];
            let deg = joint.joint_type != "prismatic" && issue.kind != IssueKind::Torque;
            let (unit, f) = match (issue.kind, deg) {
                (IssueKind::Torque, _) => (if joint.joint_type == "prismatic" { "N" } else { "N·m" }, 1.0),
                (IssueKind::Velocity, true) => ("°/s", 180.0 / std::f64::consts::PI),
                (IssueKind::Position, true) => ("°", 180.0 / std::f64::consts::PI),
                (IssueKind::Velocity, false) => ("m/s", 1.0),
                (IssueKind::Position, false) => ("m", 1.0),
            };
            let text = format!(
                "{:.2}–{:.2}s {} {}: {:.1} (limit {:.1}) {unit}",
                issue.t_start, issue.t_end, joint.name, issue.kind.label(), issue.peak * f, issue.limit * f
            );
            if ui.add(egui::Label::new(egui::RichText::new(text).small().color(issue_color(issue.kind))).sense(egui::Sense::click())).clicked() {
                clicked = Some(issue);
            }
        }
    });
    clicked
}
