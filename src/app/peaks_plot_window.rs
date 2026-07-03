//! Joint Peaks time-series plot window.
//!
//! Opens from the right-panel "📊 Joint Peaks" section's "📈 Plot" button.
//! Renders the (q, q̇, τ) ring buffer recorded by [`articara::mujoco_sim::MujocoSim`]
//! using `egui_plot`. The buffer is reset on every ▶ Play / pulse so the plot
//! shows just the response to the latest command — useful for tuning gains
//! or reading peak load timing without scrubbing through long histories.

#![cfg(feature = "mujoco")]

use eframe::egui;
use egui::PointerButton;
use egui_plot::{Legend, Line, Plot, PlotPoints};

use super::file_dialog::FileDialogMode;
use super::{
    ArticaraApp, PeaksPlotMetric, PeaksXAxisMode, PEAKS_PLOT_UNLIMITED_CAP,
};

/// UI state of the Joint Peaks plot window, grouped so [`ArticaraApp`]
/// carries one field instead of eleven loose `peaks_plot_*` ones.
pub(super) struct PeaksPlotState {
    /// Whether the window is open.
    pub open: bool,
    /// Joint selected for the plot. `None` = plot all movable joints.
    pub joint: Option<String>,
    /// Which signal to display.
    pub metric: PeaksPlotMetric,
    /// One-shot flag: when set, the next plot draw resets zoom + pan to
    /// the data's auto-bounds, then clears itself.
    pub reset_view: bool,
    /// Auto-fit x-axis vs fixed-length sliding window.
    pub xaxis_mode: PeaksXAxisMode,
    /// In Auto mode, seconds of history to retain (sizes the MuJoCo trace
    /// ring buffer).
    pub auto_seconds: f32,
    /// In Auto mode, uncapped buffer (subject to
    /// [`PEAKS_PLOT_UNLIMITED_CAP`]).
    pub auto_unlimited: bool,
    /// In Fixed mode, length of the sliding x-axis window in seconds.
    pub fixed_seconds: f32,
    /// Joint names hidden in "all" mode; single-joint focus bypasses this.
    pub hidden_joints: std::collections::HashSet<String>,
    /// File path used for the most recent CSV export.
    pub csv_path: String,
    /// File dialog for saving the trace as CSV.
    pub dlg_save_csv: super::file_dialog::FileDialog,
}

impl Default for PeaksPlotState {
    fn default() -> Self {
        Self {
            open: false,
            joint: None,
            metric: PeaksPlotMetric::Torque,
            reset_view: false,
            xaxis_mode: PeaksXAxisMode::Auto,
            auto_seconds: 10.0,
            auto_unlimited: false,
            fixed_seconds: 5.0,
            hidden_joints: std::collections::HashSet::new(),
            csv_path: String::new(),
            dlg_save_csv: super::file_dialog::FileDialog::new("dlg_save_peaks_csv"),
        }
    }
}

impl ArticaraApp {
    pub(super) fn draw_peaks_plot_window(&mut self, ctx: &egui::Context) {
        if !self.peaks_plot.open {
            return;
        }

        let mut open = true;
        let mut metric = self.peaks_plot.metric;
        let mut selected_joint = self.peaks_plot.joint.clone();
        // Capture the reset flag locally so the inner UI closure can clear
        // it after triggering Plot::reset(); we flush back to self at the
        // end of the function.
        let mut reset_view = self.peaks_plot.reset_view;
        let mut reset_clicked = false;

        // Local copies of x-axis state so the closure can mutate without
        // overlapping borrow on `self`.
        let mut xaxis_mode = self.peaks_plot.xaxis_mode;
        let mut auto_seconds = self.peaks_plot.auto_seconds;
        let mut auto_unlimited = self.peaks_plot.auto_unlimited;
        let mut fixed_seconds = self.peaks_plot.fixed_seconds;
        let mut hidden_joints = self.peaks_plot.hidden_joints.clone();
        let mut save_csv_clicked = false;

        // Snapshot trace + joint metadata up-front so the closure doesn't
        // borrow `self` while the plot widget runs.
        struct JointTrace {
            name: String,
            is_prismatic: bool,
            samples: Vec<(f64, f64)>, // (time_s, value)
        }

        let (joint_names, traces, t_latest, sim_dt) = {
            let model = match self.model.as_ref() {
                Some(m) => m,
                None => {
                    self.peaks_plot.open = false;
                    return;
                }
            };
            let sim = match self.sim.mujoco_sim.as_ref() {
                Some(s) => s,
                None => {
                    // Render a placeholder window and bail.
                    egui::Window::new("📈 Joint Peaks Plot")
                        .open(&mut open)
                        .default_size([520.0, 320.0])
                        .show(ctx, |ui| {
                            ui.label("Start MuJoCo to record samples.");
                        });
                    if !open {
                        self.peaks_plot.open = false;
                    }
                    return;
                }
            };

            let movable_joints: Vec<(usize, &str, bool)> = model
                .joints
                .iter()
                .enumerate()
                .filter(|(_, j)| j.joint_type != "fixed")
                .map(|(i, j)| (i, j.name.as_str(), j.joint_type == "prismatic"))
                .collect();

            let joint_names: Vec<String> = movable_joints
                .iter()
                .map(|(_, n, _)| n.to_string())
                .collect();

            // Determine which joints to render.
            let render_set: Vec<(usize, String, bool)> = match selected_joint.as_deref() {
                Some(name) => movable_joints
                    .iter()
                    .filter(|(_, n, _)| *n == name)
                    .map(|(i, n, p)| (*i, n.to_string(), *p))
                    .collect(),
                None => movable_joints
                    .iter()
                    .filter(|(_, n, _)| !hidden_joints.contains(*n))
                    .map(|(i, n, p)| (*i, n.to_string(), *p))
                    .collect(),
            };

            // Anchor t at 0 so the plot's x-axis reads "seconds since the
            // last reset" rather than the absolute MuJoCo clock — keeps
            // labels short and comparable across runs.
            let t0 = sim.trace().next().map(|f| f.time).unwrap_or(0.0);
            let t_latest = sim
                .trace()
                .last()
                .map(|f| f.time - t0)
                .unwrap_or(0.0);

            let mut traces: Vec<JointTrace> = render_set
                .iter()
                .map(|(_, n, p)| JointTrace {
                    name: n.clone(),
                    is_prismatic: *p,
                    samples: Vec::with_capacity(sim.trace_len()),
                })
                .collect();

            // In Fixed mode, drop samples older than the window so the plot
            // auto-bounds settle on exactly the last `fixed_seconds` of data.
            let t_min = if matches!(xaxis_mode, PeaksXAxisMode::Fixed) {
                t_latest - fixed_seconds as f64
            } else {
                f64::NEG_INFINITY
            };

            for frame in sim.trace() {
                let dt_anchor = frame.time - t0;
                if dt_anchor < t_min {
                    continue;
                }
                for (slot, (idx, _, _)) in traces.iter_mut().zip(render_set.iter()) {
                    let v = match metric {
                        PeaksPlotMetric::Position => frame.q.get(*idx).copied().unwrap_or(0.0),
                        PeaksPlotMetric::Velocity => frame.qvel.get(*idx).copied().unwrap_or(0.0),
                        PeaksPlotMetric::Torque => frame.tau.get(*idx).copied().unwrap_or(0.0),
                    };
                    slot.samples.push((dt_anchor, v));
                }
            }

            (joint_names, traces, t_latest, sim.timestep())
        };

        egui::Window::new("📈 Joint Peaks Plot")
            .open(&mut open)
            .default_size([640.0, 420.0])
            .resizable(true)
            .show(ctx, |ui| {
                ui.horizontal(|ui| {
                    ui.label("Signal:");
                    egui::ComboBox::from_id_salt("peaks_plot_metric")
                        .selected_text(metric.label())
                        .show_ui(ui, |ui| {
                            for m in PeaksPlotMetric::ALL {
                                ui.selectable_value(&mut metric, m, m.label());
                            }
                        });

                    ui.add_space(12.0);
                    ui.label("Joint:");
                    let label = selected_joint
                        .as_deref()
                        .map(|s| s.to_string())
                        .unwrap_or_else(|| "(all visible)".to_string());
                    egui::ComboBox::from_id_salt("peaks_plot_joint")
                        .selected_text(label)
                        .width(180.0)
                        .show_ui(ui, |ui| {
                            if ui
                                .selectable_label(selected_joint.is_none(), "(all)")
                                .clicked()
                            {
                                selected_joint = None;
                            }
                            for n in &joint_names {
                                let sel = selected_joint.as_deref() == Some(n.as_str());
                                if ui.selectable_label(sel, n).clicked() {
                                    selected_joint = Some(n.clone());
                                }
                            }
                        });

                    ui.with_layout(
                        egui::Layout::right_to_left(egui::Align::Center),
                        |ui| {
                            if ui
                                .button("⟲ Reset view")
                                .on_hover_text(
                                    "Reset zoom and pan to fit all samples.",
                                )
                                .clicked()
                            {
                                reset_clicked = true;
                            }
                            if ui
                                .button("💾 Save CSV")
                                .on_hover_text(
                                    "Export the recorded trace (time × all joints × q/q̇/τ) as CSV.",
                                )
                                .clicked()
                            {
                                save_csv_clicked = true;
                            }
                        },
                    );
                });

                // ── X-axis controls ──
                ui.horizontal(|ui| {
                    ui.label("X-axis:");
                    egui::ComboBox::from_id_salt("peaks_plot_xaxis_mode")
                        .selected_text(xaxis_mode.label())
                        .show_ui(ui, |ui| {
                            for m in PeaksXAxisMode::ALL {
                                ui.selectable_value(&mut xaxis_mode, m, m.label());
                            }
                        });
                    match xaxis_mode {
                        PeaksXAxisMode::Auto => {
                            ui.checkbox(&mut auto_unlimited, "Unlimited")
                                .on_hover_text(
                                    "Keep all samples (capped at \
                                     PEAKS_PLOT_UNLIMITED_CAP to bound memory).",
                                );
                            ui.add_enabled_ui(!auto_unlimited, |ui| {
                                ui.label("Max length:");
                                ui.add(
                                    egui::DragValue::new(&mut auto_seconds)
                                        .speed(0.5)
                                        .range(0.5..=600.0)
                                        .suffix(" s"),
                                );
                            });
                        }
                        PeaksXAxisMode::Fixed => {
                            ui.label("Window:");
                            ui.add(
                                egui::DragValue::new(&mut fixed_seconds)
                                    .speed(0.1)
                                    .range(0.1..=600.0)
                                    .suffix(" s"),
                            );
                        }
                    }
                });

                // ── Per-joint visibility (only meaningful in "all" mode) ──
                if selected_joint.is_none() && !joint_names.is_empty() {
                    egui::CollapsingHeader::new(format!(
                        "Visible joints ({}/{})",
                        joint_names.len() - hidden_joints.len(),
                        joint_names.len(),
                    ))
                    .id_salt("peaks_plot_visible")
                    .default_open(false)
                    .show(ui, |ui| {
                        ui.horizontal(|ui| {
                            if ui.small_button("Show all").clicked() {
                                hidden_joints.clear();
                            }
                            if ui.small_button("Hide all").clicked() {
                                for n in &joint_names {
                                    hidden_joints.insert(n.clone());
                                }
                            }
                        });
                        egui::ScrollArea::vertical()
                            .max_height(120.0)
                            .id_salt("peaks_plot_visible_scroll")
                            .show(ui, |ui| {
                                for n in &joint_names {
                                    let mut visible = !hidden_joints.contains(n);
                                    if ui.checkbox(&mut visible, n).changed() {
                                        if visible {
                                            hidden_joints.remove(n);
                                        } else {
                                            hidden_joints.insert(n.clone());
                                        }
                                    }
                                }
                            });
                    });
                }

                if traces.is_empty()
                    || traces.iter().all(|t| t.samples.is_empty())
                {
                    ui.label(
                        egui::RichText::new(
                            "(no samples yet — run the sim or hit ▶ Play)",
                        )
                        .small()
                        .weak(),
                    );
                    return;
                }

                let unit = traces
                    .first()
                    .map(|t| metric.unit(t.is_prismatic))
                    .unwrap_or("");
                ui.label(
                    egui::RichText::new(format!(
                        "x = sim time since last reset (s),  y in {unit}  ·  scroll = zoom, L-drag = box zoom, R-drag = pan",
                    ))
                    .small()
                    .weak(),
                );

                let mut plot = Plot::new("peaks_plot")
                    .legend(Legend::default())
                    .x_axis_label("t [s]")
                    .y_axis_label(unit)
                    .allow_zoom(true)
                    .allow_scroll(true)
                    .allow_boxed_zoom(true)
                    .boxed_zoom_pointer_button(PointerButton::Primary)
                    .allow_drag(true)
                    .pan_pointer_button(PointerButton::Secondary);
                // Fixed mode: re-fit every frame so the window scrolls with
                // the latest sample rather than freezing at a stale view.
                let force_reset = matches!(xaxis_mode, PeaksXAxisMode::Fixed);
                if reset_clicked || reset_view || force_reset {
                    plot = plot.reset();
                    reset_view = false;
                }
                plot.show(ui, |plot_ui| {
                    for tr in &traces {
                        let pts: PlotPoints = tr
                            .samples
                            .iter()
                            .map(|(t, v)| [*t, *v])
                            .collect();
                        plot_ui.line(Line::new(tr.name.clone(), pts));
                    }
                    // In Fixed mode, force the x-axis to span exactly the
                    // configured window even when fewer samples are present
                    // (e.g. right after a reset).
                    if matches!(xaxis_mode, PeaksXAxisMode::Fixed) {
                        let t_end = t_latest;
                        let t_start = t_end - fixed_seconds as f64;
                        let bounds = plot_ui.plot_bounds();
                        let y_min = bounds.min()[1];
                        let y_max = bounds.max()[1];
                        plot_ui.set_plot_bounds(egui_plot::PlotBounds::from_min_max(
                            [t_start, y_min],
                            [t_end, y_max],
                        ));
                    }
                });
            });

        // ── Apply trace_max according to current x-axis settings ──
        if let Some(sim) = self.sim.mujoco_sim.as_mut() {
            let dt = sim_dt.max(1e-6);
            let target = match xaxis_mode {
                PeaksXAxisMode::Auto if auto_unlimited => PEAKS_PLOT_UNLIMITED_CAP,
                PeaksXAxisMode::Auto => {
                    ((auto_seconds as f64 / dt).ceil() as usize).max(1)
                }
                PeaksXAxisMode::Fixed => {
                    ((fixed_seconds as f64 / dt).ceil() as usize).max(1)
                }
            };
            sim.set_trace_max(target.min(PEAKS_PLOT_UNLIMITED_CAP));
        }

        // ── CSV save dialog trigger ──
        if save_csv_clicked {
            let start = if self.peaks_plot.csv_path.is_empty() {
                None
            } else {
                Some(std::path::PathBuf::from(&self.peaks_plot.csv_path))
            };
            self.peaks_plot.dlg_save_csv.open(
                "Save Peaks Trace CSV",
                FileDialogMode::Save,
                start.as_deref(),
                &["csv"],
            );
        }

        self.peaks_plot.metric = metric;
        self.peaks_plot.joint = selected_joint;
        self.peaks_plot.reset_view = reset_view;
        self.peaks_plot.xaxis_mode = xaxis_mode;
        self.peaks_plot.auto_seconds = auto_seconds;
        self.peaks_plot.auto_unlimited = auto_unlimited;
        self.peaks_plot.fixed_seconds = fixed_seconds;
        self.peaks_plot.hidden_joints = hidden_joints;
        if !open {
            self.peaks_plot.open = false;
        }
    }
}

/// Re-export so callers that already used `peaks_plot_window::save_peaks_csv`
/// keep working. The implementation now lives on `MujocoSim` so scripts in
/// the (lib-only) scripting layer can call it without depending on the GUI
/// `app` module.
pub use articara::mujoco_sim::save_peaks_csv;
