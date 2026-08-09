//! Shared keyboard bindings for the interactive namiashi demos.
//!
//! One place, so `examples/namiashi_wbc_teleop.rs` (model-based WBC/MPC,
//! driven through [`crate::wbc_harness::run_wbc_sim`]) and
//! `examples/namiashi_rl_teleop.rs` (the trained RL policy) cannot drift
//! apart -- the whole point of having both demos is that the only thing
//! differing between them is the controller, so the same keys must mean
//! the same thing on each.
//!
//! | keys                      | axis                        |
//! |---------------------------|-----------------------------|
//! | `W`/`S` or `Up`/`Down`    | forward/back speed (vx)     |
//! | `A`/`D` or `Left`/`Right` | turn rate (wz)              |
//! | `Q`/`E` or `PgUp`/`PgDn`  | strafe (vy)                 |
//! | `Shift` (held)            | full speed instead of half  |
//! | `1`/`2`/`3`               | Crawl / Walk / Trot         |
//! | `R`/`F`                   | swing height +/- 5 mm       |
//!
//! Hold-to-move, release-to-stop: a released key means zero on that axis
//! that same frame, not a latched command that keeps running. The two
//! discrete controls (gait, swing height) are edge-triggered instead, so
//! holding the key doesn't repeat.
//!
//! Two of these overlap with `MjViewer`'s own built-in hotkeys, both
//! harmless but worth knowing: `E` also toggles constraint visualisation,
//! and `Q` quits **only** with Ctrl held (plain `Q` is ours). `PgUp`/`PgDn`
//! are the conflict-free way to strafe. The viewer's `X` (hide the side
//! panel) does NOT stop these bindings -- mujoco-rs runs detached UI
//! callbacks outside its `ViewerStatusBit::UI` gate.

use mujoco::viewer::egui;
use quadruped_gait::GaitType;

/// What the viewer's key callback writes and the physics loop reads, once
/// per tick, behind a mutex.
#[derive(Clone, Copy, Debug)]
pub struct LiveTeleop {
    /// Body-frame velocity command `[vx, vy, wz]`.
    pub cmd: [f64; 3],
    /// Requested gait. The WBC/MPC loop applies a change by swapping the
    /// whole `GaitConfig`; the RL demo ignores this (a learned policy has
    /// no gait to switch).
    pub gait: GaitType,
    /// Requested peak swing-foot lift, metres -- `GaitConfig::swing_height_m`.
    /// The same knob `namiashi_staircase_5cm_swing_clearance_sweep` swept
    /// from 0.040 to 0.200 m trying to clear a 5 cm riser, exposed live so
    /// the effect can be felt against the staircase directly.
    ///
    /// Carried across a gait switch rather than snapping back to the new
    /// gait's tuned value: a raised foot is usually raised deliberately
    /// (to clear something), and silently undoing that on a gait change
    /// would be the more surprising behaviour. The tuned values are within
    /// 5 mm of each other anyway (0.035-0.040 m).
    pub swing_height_m: f64,

    // ── Telemetry: written by the physics loop, read by the HUD. These
    // flow the OPPOSITE way from the fields above -- the key callback
    // must never write them, the loop must never read them as input.
    /// Trunk world x, metres. On the staircase this is the progress
    /// readout: risers start at 1.5 m, treads are 0.20 m apart.
    pub body_x_m: f64,
    /// Trunk world z, metres.
    pub body_z_m: f64,
    /// Measured world-frame forward speed, m/s -- what the robot is
    /// actually doing, against the `cmd[0]` it was asked for.
    pub measured_vx_mps: f64,
}

impl LiveTeleop {
    /// Stopped, in the given gait, at that gait's tuned swing height.
    pub fn new(gait: GaitType) -> Self {
        Self {
            cmd: [0.0; 3],
            gait,
            swing_height_m: crate::wbc_harness::namiashi_tuned_swing_height_m(gait),
            body_x_m: 0.0,
            body_z_m: 0.0,
            measured_vx_mps: 0.0,
        }
    }
}

/// Per-axis ceiling for a full-deflection (Shift-held) command.
#[derive(Clone, Copy, Debug)]
pub struct SpeedEnvelope {
    pub vx: f64,
    pub vy: f64,
    pub wz: f64,
}

impl SpeedEnvelope {
    /// The tuned envelope for one gait.
    ///
    /// `vx` is that gait's own tuned command from
    /// [`crate::wbc_harness::NAMIASHI_TUNED`] -- NOT a single number shared
    /// across gaits. Each gait's reachable speed is
    /// `max_step_length_m / (cycle_period_s * duty_factor)`, so commanding
    /// Trot's 0.80 m/s in Crawl would just saturate against a ceiling under
    /// a quarter of that and read as a broken controller rather than a slow
    /// gait.
    ///
    /// `vy`/`wz` have no equivalent tuned reference anywhere in this repo,
    /// so they are scaled off the Trot values by that gait's own vx ratio
    /// -- a heuristic that keeps a crawl from being asked to spin at a
    /// trot's yaw rate, not a measured limit.
    pub fn for_gait(gait: GaitType) -> Self {
        const TROT_VX: f64 = 0.800;
        const TROT_VY: f64 = 0.30;
        const TROT_WZ: f64 = 1.00;
        let vx = crate::wbc_harness::namiashi_tuned_cmd_vx(gait);
        let r = vx / TROT_VX;
        Self { vx, vy: TROT_VY * r, wz: TROT_WZ * r }
    }
}

/// Fraction of [`SpeedEnvelope`] used when Shift is *not* held.
const SLOW_FRACTION: f64 = 0.5;

/// Read the current velocity command straight off which keys are down
/// right now. Nothing is integrated or latched: releasing a key zeroes
/// that axis on the next poll, which is what makes this stop-on-release
/// rather than a cruise control.
pub fn poll_cmd(ctx: &egui::Context, env: SpeedEnvelope) -> [f64; 3] {
    ctx.input(|r| {
        let scale = if r.modifiers.shift { 1.0 } else { SLOW_FRACTION };
        let axis = |pos: bool, neg: bool, max: f64| {
            ((pos as i32 - neg as i32) as f64) * max * scale
        };
        [
            axis(
                r.key_down(egui::Key::W) || r.key_down(egui::Key::ArrowUp),
                r.key_down(egui::Key::S) || r.key_down(egui::Key::ArrowDown),
                env.vx,
            ),
            axis(
                r.key_down(egui::Key::Q) || r.key_down(egui::Key::PageUp),
                r.key_down(egui::Key::E) || r.key_down(egui::Key::PageDown),
                env.vy,
            ),
            axis(
                r.key_down(egui::Key::A) || r.key_down(egui::Key::ArrowLeft),
                r.key_down(egui::Key::D) || r.key_down(egui::Key::ArrowRight),
                env.wz,
            ),
        ]
    })
}

/// One `R`/`F` press worth of swing-height change.
pub const SWING_HEIGHT_STEP_M: f64 = 0.005;

/// Clamp on the live swing height, metres. The upper bound is the top of
/// the range `namiashi_staircase_5cm_swing_clearance_sweep` already
/// established the leg can actually reach; the lower one keeps the foot
/// clearing its own stance compression.
pub const SWING_HEIGHT_RANGE_M: (f64, f64) = (0.010, 0.200);

/// Swing-height change requested this frame, metres (0.0 if neither key
/// was pressed). Edge-triggered, so holding `R` steps once rather than
/// ramping at frame rate.
pub fn poll_swing_height_delta(ctx: &egui::Context) -> f64 {
    ctx.input(|r| {
        let up = r.key_pressed(egui::Key::R);
        let down = r.key_pressed(egui::Key::F);
        ((up as i32 - down as i32) as f64) * SWING_HEIGHT_STEP_M
    })
}

/// The gait requested this frame, if any. Edge-triggered (`key_pressed`,
/// not `key_down`) so holding `1` doesn't re-apply Crawl every frame.
pub fn poll_gait(ctx: &egui::Context) -> Option<GaitType> {
    ctx.input(|r| {
        if r.key_pressed(egui::Key::Num1) {
            Some(GaitType::Crawl)
        } else if r.key_pressed(egui::Key::Num2) {
            Some(GaitType::Walk)
        } else if r.key_pressed(egui::Key::Num3) {
            Some(GaitType::Trot)
        } else {
            None
        }
    })
}

/// Always-on overlay of everything the keys control, plus what the robot
/// is actually doing about it. Anchored top-right so it clears
/// `MjViewer`'s own left side panel.
///
/// `gaited` is false for a learned policy: it has no gait schedule and no
/// swing-height parameter, so showing either would be showing a knob that
/// does nothing. Drawn from the shared [`LiveTeleop`] alone -- no MjData
/// needed, so this works from the cheap detached UI callback.
pub fn draw_hud(
    ctx: &egui::Context,
    st: &LiveTeleop,
    env: SpeedEnvelope,
    controller: &str,
    gaited: bool,
) {
    let fast = ctx.input(|r| r.modifiers.shift);
    egui::Window::new("teleop_hud")
        .title_bar(false)
        .resizable(false)
        .movable(false)
        .anchor(egui::Align2::RIGHT_TOP, [-8.0, 8.0])
        .show(ctx, |ui| {
            ui.label(egui::RichText::new(controller).strong());
            ui.separator();

            let row = |ui: &mut egui::Ui, k: &str, v: String| {
                ui.horizontal(|ui| {
                    ui.label(egui::RichText::new(format!("{k:<10}")).monospace().weak());
                    ui.label(egui::RichText::new(v).monospace());
                });
            };

            if gaited {
                row(ui, "gait", format!("{:?}   [1/2/3]", st.gait));
                row(
                    ui,
                    "swing h",
                    format!(
                        "{:.3} m (tuned {:.3})   [R/F]",
                        st.swing_height_m,
                        crate::wbc_harness::namiashi_tuned_swing_height_m(st.gait),
                    ),
                );
                ui.separator();
            }

            row(
                ui,
                "speed",
                format!(
                    "{}  (Shift for full)",
                    if fast { "FULL" } else { "half" },
                ),
            );
            row(ui, "vx cmd", format!("{:+.3} / {:.3} m/s   [W/S]", st.cmd[0], env.vx));
            row(ui, "vy cmd", format!("{:+.3} / {:.3} m/s   [Q/E]", st.cmd[1], env.vy));
            row(ui, "wz cmd", format!("{:+.3} / {:.3} rad/s [A/D]", st.cmd[2], env.wz));

            ui.separator();
            row(ui, "vx meas", format!("{:+.3} m/s", st.measured_vx_mps));
            row(ui, "trunk x", format!("{:+.3} m", st.body_x_m));
            row(ui, "trunk z", format!("{:+.3} m", st.body_z_m));
        });
}
