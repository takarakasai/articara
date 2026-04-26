//! Joint Peaks time-series plot window.
//!
//! Opens from the right-panel "📊 Joint Peaks" section's "📈 Plot" button.
//! Renders the (q, q̇, τ) ring buffer recorded by [`crate::mujoco_sim::MujocoSim`]
//! using `egui_plot`. The buffer is reset on every ▶ Play / pulse so the plot
//! shows just the response to the latest command — useful for tuning gains
//! or reading peak load timing without scrubbing through long histories.

#![cfg(feature = "mujoco")]

use eframe::egui;
use egui_plot::{Legend, Line, Plot, PlotPoints};

use super::{ArticaraApp, PeaksPlotMetric};

/// Maximum number of joint traces to render simultaneously to keep the plot
/// readable on a smaller side window. When more joints are movable, the user
/// has to pick one explicitly via the joint dropdown.
const MAX_TRACES_ALL_MODE: usize = 8;

impl ArticaraApp {
    pub(super) fn draw_peaks_plot_window(&mut self, ctx: &egui::Context) {
        if !self.show_peaks_plot {
            return;
        }

        let mut open = true;
        let mut metric = self.peaks_plot_metric;
        let mut selected_joint = self.peaks_plot_joint.clone();

        // Snapshot trace + joint metadata up-front so the closure doesn't
        // borrow `self` while the plot widget runs.
        struct JointTrace {
            name: String,
            is_prismatic: bool,
            samples: Vec<(f64, f64)>, // (time_s, value)
        }

        let (joint_names, traces, t0) = {
            let model = match self.model.as_ref() {
                Some(m) => m,
                None => {
                    self.show_peaks_plot = false;
                    return;
                }
            };
            let sim = match self.mujoco_sim.as_ref() {
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
                        self.show_peaks_plot = false;
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
                    .take(MAX_TRACES_ALL_MODE)
                    .map(|(i, n, p)| (*i, n.to_string(), *p))
                    .collect(),
            };

            // Anchor t at 0 so the plot's x-axis reads "seconds since the
            // last reset" rather than the absolute MuJoCo clock — keeps
            // labels short and comparable across runs.
            let t0 = sim.trace().next().map(|f| f.time).unwrap_or(0.0);

            let mut traces: Vec<JointTrace> = render_set
                .iter()
                .map(|(_, n, p)| JointTrace {
                    name: n.clone(),
                    is_prismatic: *p,
                    samples: Vec::with_capacity(sim.trace_len()),
                })
                .collect();

            for frame in sim.trace() {
                let dt_anchor = frame.time - t0;
                for (slot, (idx, _, _)) in traces.iter_mut().zip(render_set.iter()) {
                    let v = match metric {
                        PeaksPlotMetric::Position => frame.q.get(*idx).copied().unwrap_or(0.0),
                        PeaksPlotMetric::Velocity => frame.qvel.get(*idx).copied().unwrap_or(0.0),
                        PeaksPlotMetric::Torque => frame.tau.get(*idx).copied().unwrap_or(0.0),
                    };
                    slot.samples.push((dt_anchor, v));
                }
            }

            (joint_names, traces, t0)
        };

        let _ = t0; // anchor only needed during snapshot; kept for symmetry

        egui::Window::new("📈 Joint Peaks Plot")
            .open(&mut open)
            .default_size([640.0, 360.0])
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
                        .unwrap_or_else(|| {
                            format!("(all, ≤{} shown)", MAX_TRACES_ALL_MODE)
                        });
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
                });

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
                        "x = sim time since last reset (s),  y in {unit}",
                    ))
                    .small()
                    .weak(),
                );

                Plot::new("peaks_plot")
                    .legend(Legend::default())
                    .x_axis_label("t [s]")
                    .y_axis_label(unit)
                    .show(ui, |plot_ui| {
                        for tr in &traces {
                            let pts: PlotPoints = tr
                                .samples
                                .iter()
                                .map(|(t, v)| [*t, *v])
                                .collect();
                            plot_ui.line(Line::new(tr.name.clone(), pts));
                        }
                    });
            });

        self.peaks_plot_metric = metric;
        self.peaks_plot_joint = selected_joint;
        if !open {
            self.show_peaks_plot = false;
        }
    }
}
