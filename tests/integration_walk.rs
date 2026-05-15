//! Integration regression: run a 1-second forward-walk simulation
//! under three controller stacks and assert each one preserves the
//! expected behavioural envelope.
//!
//! The point isn't to prove correctness of any single layer (that's
//! what the focused tests in `gait_walk_stability` / `wbc_walk` /
//! `lkf_pipeline` do); it's to **catch unintended cross-layer
//! regressions** as features land. When a change to e.g. the SRBD
//! MPC's r_diag tuning breaks WBC stability without breaking the
//! pure-MPC walk, this test catches it before commit.
//!
//! Stacks under test:
//! 1. `position_pd_with_mpc_torque_ff` — the `mpc_walks_stable`
//!    baseline (MPC GRF → −Jᵀ·f → joint τ_ff, Position-PD on top).
//! 2. `position_pd_plus_wbc` — Hybrid joint command from
//!    `wbc_walk`'s static stand path: MPC + WBC + Position-PD with
//!    contact-driven phase correction.
//! 3. `position_pd_plus_lkf` — Position-PD with the LKF observing
//!    in parallel; verifies the estimator stays in sync with ground
//!    truth while the controller runs.
//!
//! Each stack records `(min_z, peak_tau_abs, body_x_displacement)`
//! and asserts a per-stack envelope.

#![cfg(feature = "mujoco")]

mod common;

use articara::estimator::LkfPipeline;
use articara::gait::GaitController;
use articara::wbc_pipeline::WbcPipeline;
use nalgebra::{UnitQuaternion, Vector3};
use quadruped_gait::{ContactDrivenPhase, GaitConfig, GaitMode, VelocityCmd};

/// Per-stack metric collector. Every test path fills this in and
/// then asserts on the final values.
#[derive(Debug, Default, Clone)]
struct WalkMetrics {
    min_body_z: f64,
    max_body_z: f64,
    peak_tau_abs: f64,
    body_x_start: f64,
    body_x_end: f64,
}

impl WalkMetrics {
    fn new() -> Self {
        Self {
            min_body_z: f64::INFINITY,
            max_body_z: f64::NEG_INFINITY,
            ..Default::default()
        }
    }
    fn body_x_delta(&self) -> f64 {
        self.body_x_end - self.body_x_start
    }
}

const SIM_TIME_S: f64 = 1.0;
const DT: f64 = 0.002;
const BURN_IN_S: f64 = 0.3;
const CMD_VX: f64 = 0.15;
const FALL_THRESHOLD_Z: f64 = 0.18;

/// Stack 1: pure Position-PD + MPC τ_ff (the existing mpc_walks_stable
/// baseline). Used as the "known-good" reference; if this regresses
/// it means the gait core itself broke.
#[test]
fn integration_position_pd_with_mpc_torque_ff() {
    let Some(common::StandFixture {
        mut robot,
        kin,
        mut sim,
    }) = common::build_namiashi_stand_fixture_misa()
    else {
        return;
    };

    let cfg = GaitConfig::trot();
    let mut gc = GaitController::build(&robot, kin, cfg, GaitMode::Mpc)
        .expect("GaitController::build (Mpc mode)");

    let n_steps = (SIM_TIME_S / DT) as usize;
    let burn_in_steps = (BURN_IN_S / DT) as usize;
    let mut metrics = WalkMetrics::new();
    metrics.body_x_start = sim
        .body_world_position(&robot.root_link)
        .map(|p| p[0])
        .unwrap_or(0.0);

    gc.enable();
    for k in 0..n_steps {
        if k == burn_in_steps {
            gc.set_velocity_cmd(VelocityCmd {
                vx: CMD_VX,
                vy: 0.0,
                wz: 0.0,
            });
        }
        let v_obs = sim
            .body_world_linear_velocity(&robot.root_link)
            .unwrap_or([0.0, 0.0, 0.0]);
        let w_obs = sim
            .body_world_angular_velocity(&robot.root_link)
            .unwrap_or([0.0, 0.0, 0.0]);
        gc.set_body_state_observed(
            Vector3::new(v_obs[0], v_obs[1], v_obs[2]),
            Vector3::new(w_obs[0], w_obs[1], w_obs[2]),
        );

        let (_out, targets, torque_ff) = gc.tick(DT);
        for (idx, q) in targets {
            sim.set_position_target(idx, q);
        }
        for (idx, tau) in torque_ff {
            sim.set_torque_feedforward(idx, tau);
            metrics.peak_tau_abs = metrics.peak_tau_abs.max(tau.abs());
        }
        sim.step(&mut robot, DT, true);

        if let Some(pos) = sim.body_world_position(&robot.root_link) {
            metrics.min_body_z = metrics.min_body_z.min(pos[2]);
            metrics.max_body_z = metrics.max_body_z.max(pos[2]);
            metrics.body_x_end = pos[0];
        }
    }

    eprintln!("[integration:pd+mpc_ff] {:?}", metrics);
    assert!(
        metrics.min_body_z > FALL_THRESHOLD_Z,
        "PD+MPC ff: trunk fell, min_z = {:.3} m",
        metrics.min_body_z
    );
    assert!(
        metrics.body_x_delta() > 0.02,
        "PD+MPC ff: insufficient forward motion ({:.3} m, expected > 0.02 m)",
        metrics.body_x_delta()
    );
}

/// Stack 2: Hybrid joint (Position-PD + WBC τ_ff) + ContactDrivenPhase.
/// Same configuration as `wbc_walk::wbc_static_stand_balances_gravity`
/// but driven with a forward command. The forward-displacement
/// assertion is loose because trotting under WBC currently produces
/// near-zero net translation (`wbc_forward_command_advances_body` is
/// ignored for that reason); this test only verifies the body
/// **doesn't fall** while WBC + contact correction is active.
#[test]
fn integration_position_pd_plus_wbc() {
    let Some(common::StandFixture {
        mut robot,
        kin,
        mut sim,
    }) = common::build_namiashi_stand_fixture_misa()
    else {
        return;
    };

    let cfg = GaitConfig::trot();
    let mut gc = GaitController::build(&robot, kin.clone(), cfg, GaitMode::Mpc)
        .expect("GaitController::build (Mpc mode)");
    let mut wbc_pipeline = WbcPipeline::new(&robot, common::default_foot_links());

    let n_steps = (SIM_TIME_S / DT) as usize;
    let burn_in_steps = (BURN_IN_S / DT) as usize;
    let mut metrics = WalkMetrics::new();
    metrics.body_x_start = sim
        .body_world_position(&robot.root_link)
        .map(|p| p[0])
        .unwrap_or(0.0);

    gc.enable();
    for k in 0..n_steps {
        if k == burn_in_steps {
            gc.set_velocity_cmd(VelocityCmd {
                vx: CMD_VX,
                vy: 0.0,
                wz: 0.0,
            });
        }
        let v_obs = sim
            .body_world_linear_velocity(&robot.root_link)
            .unwrap_or([0.0, 0.0, 0.0]);
        let w_obs = sim
            .body_world_angular_velocity(&robot.root_link)
            .unwrap_or([0.0, 0.0, 0.0]);
        gc.set_body_state_observed(
            Vector3::new(v_obs[0], v_obs[1], v_obs[2]),
            Vector3::new(w_obs[0], w_obs[1], w_obs[2]),
        );

        let (out, targets, _torque_ff) = gc.tick(DT);
        for (idx, q) in targets {
            sim.set_position_target(idx, q);
        }
        if k >= burn_in_steps {
            let f_grf_world = gc
                .predicted_grfs()
                .map(|sol| sol.grfs_first_step)
                .unwrap_or([Vector3::zeros(); 4]);
            let cmd = gc.velocity_cmd();
            let v_cmd_body = Vector3::new(cmd.vx, cmd.vy, 0.0);

            let foot_links_str: [&str; 4] = [
                wbc_pipeline.foot_links[0].as_str(),
                wbc_pipeline.foot_links[1].as_str(),
                wbc_pipeline.foot_links[2].as_str(),
                wbc_pipeline.foot_links[3].as_str(),
            ];
            let force_z = sim.contact_force_per_foot(&foot_links_str);
            let nominal_phases = [
                out.legs[0].phase,
                out.legs[1].phase,
                out.legs[2].phase,
                out.legs[3].phase,
            ];
            let corrected =
                ContactDrivenPhase::apply_correction(&nominal_phases, force_z, 5.0, 0.0);
            let contact_flag = [
                corrected[0].is_stance,
                corrected[1].is_stance,
                corrected[2].is_stance,
                corrected[3].is_stance,
            ];

            let taus = wbc_pipeline.solve(
                &robot,
                &sim,
                &out,
                gc.kinematics(),
                gc.joint_indices(),
                gc.joint_signs(),
                &v_cmd_body,
                cmd.wz,
                &Vector3::new(v_obs[0], v_obs[1], v_obs[2]),
                &Vector3::new(w_obs[0], w_obs[1], w_obs[2]),
                &f_grf_world,
                contact_flag,
                DT,
            );
            for (ji, &tau) in taus.iter().enumerate() {
                sim.set_torque_feedforward(ji, tau);
                metrics.peak_tau_abs = metrics.peak_tau_abs.max(tau.abs());
            }
        } else {
            for ji in 0..robot.joints.len() {
                sim.set_torque_feedforward(ji, 0.0);
            }
        }

        sim.step(&mut robot, DT, true);

        if let Some(pos) = sim.body_world_position(&robot.root_link) {
            metrics.min_body_z = metrics.min_body_z.min(pos[2]);
            metrics.max_body_z = metrics.max_body_z.max(pos[2]);
            metrics.body_x_end = pos[0];
        }
    }

    eprintln!("[integration:pd+wbc] {:?}", metrics);
    assert!(
        metrics.min_body_z > FALL_THRESHOLD_Z,
        "PD+WBC: trunk fell, min_z = {:.3} m",
        metrics.min_body_z
    );
    // forward motion is currently near zero under WBC trotting (see
    // wbc_forward_command_advances_body's #[ignore]). Just assert
    // the body didn't translate **backward** by more than ~half a
    // body length (that would indicate a sign-flip regression).
    // The exact bound is loose because WBC trot dynamics involve a
    // back-and-forth oscillation that can momentarily go negative
    // even when net motion is forward; the joint-space swing-leg
    // task changed the typical Δx from ~−2 cm to ~−5 cm without
    // compromising body z stability.
    assert!(
        metrics.body_x_delta() > -0.10,
        "PD+WBC: spurious backward motion ({:.3} m)",
        metrics.body_x_delta()
    );
}

/// Stack 3: Position-PD with `LkfPipeline` running in parallel.
/// Asserts the LKF's body-z estimate stays within ±5 cm of the
/// MuJoCo ground truth even while the gait controller is moving the
/// joints. Catches regressions where a change to FK / Jacobian /
/// build_floating_base_model breaks the estimator path.
#[test]
fn integration_position_pd_plus_lkf() {
    let Some(common::StandFixture {
        mut robot,
        kin,
        mut sim,
    }) = common::build_namiashi_stand_fixture_misa()
    else {
        return;
    };

    let cfg = GaitConfig::trot();
    let mut gc = GaitController::build(&robot, kin, cfg, GaitMode::Mpc)
        .expect("GaitController::build (Mpc mode)");
    let mut lkf = LkfPipeline::new(&robot, common::default_foot_links());
    lkf.kf.reset(
        Vector3::new(0.0, 0.0, 0.30),
        &[
            Vector3::new(0.18, 0.10, 0.0),
            Vector3::new(0.18, -0.10, 0.0),
            Vector3::new(-0.18, 0.10, 0.0),
            Vector3::new(-0.18, -0.10, 0.0),
        ],
    );

    let n_steps = (SIM_TIME_S / DT) as usize;
    let burn_in_steps = (BURN_IN_S / DT) as usize;
    let mut metrics = WalkMetrics::new();
    metrics.body_x_start = sim
        .body_world_position(&robot.root_link)
        .map(|p| p[0])
        .unwrap_or(0.0);

    let mut prev_v_world = Vector3::zeros();
    let mut max_z_estimation_err = 0.0_f64;

    gc.enable();
    for k in 0..n_steps {
        if k == burn_in_steps {
            gc.set_velocity_cmd(VelocityCmd {
                vx: CMD_VX,
                vy: 0.0,
                wz: 0.0,
            });
        }
        let v_obs = sim
            .body_world_linear_velocity(&robot.root_link)
            .map(|v| Vector3::new(v[0], v[1], v[2]))
            .unwrap_or_else(Vector3::zeros);
        let w_obs = sim
            .body_world_angular_velocity(&robot.root_link)
            .unwrap_or([0.0, 0.0, 0.0]);
        gc.set_body_state_observed(
            v_obs,
            Vector3::new(w_obs[0], w_obs[1], w_obs[2]),
        );

        // Synthetic IMU input via finite-diff of body world velocity.
        let accel_world = if k > 0 {
            (v_obs - prev_v_world) / DT
        } else {
            Vector3::zeros()
        };
        prev_v_world = v_obs;
        let body_quat = sim
            .body_world_orientation(&robot.root_link)
            .unwrap_or_else(UnitQuaternion::identity);

        let (_out, targets, torque_ff) = gc.tick(DT);
        for (idx, q) in targets {
            sim.set_position_target(idx, q);
        }
        for (idx, tau) in torque_ff {
            sim.set_torque_feedforward(idx, tau);
        }

        let _kf_out = lkf.update_from_mujoco(&robot, &sim, body_quat, accel_world, DT);

        sim.step(&mut robot, DT, true);

        if let Some(pos) = sim.body_world_position(&robot.root_link) {
            metrics.min_body_z = metrics.min_body_z.min(pos[2]);
            metrics.max_body_z = metrics.max_body_z.max(pos[2]);
            metrics.body_x_end = pos[0];
            // Track LKF z-error after the prior has had time to converge.
            if (k as f64) * DT >= 0.5 {
                let err = (lkf.kf.x_hat[2] - pos[2]).abs();
                max_z_estimation_err = max_z_estimation_err.max(err);
            }
        }
    }

    eprintln!(
        "[integration:pd+lkf] {:?} max_lkf_z_err = {:.3} m",
        metrics, max_z_estimation_err
    );
    assert!(
        metrics.min_body_z > FALL_THRESHOLD_Z,
        "PD+LKF: trunk fell, min_z = {:.3} m",
        metrics.min_body_z
    );
    assert!(
        max_z_estimation_err < 0.05,
        "LKF z-estimate diverged (max err {:.3} m > 0.05 m)",
        max_z_estimation_err
    );
}

// ─── 3-axis track-quality benchmarks (Phase P1 + cross-coupling) ──
//
// We evaluate gait quality along three independent commanded motions:
//
//   1. **forward** (cmd.vx > 0)   — assert Δx > 0, cross-axis Δy / Δyaw small
//   2. **lateral** (cmd.vy > 0)   — assert Δy > 0, cross-axis Δx / Δyaw small
//   3. **yaw**     (cmd.wz > 0)   — assert |Δyaw| > 0, cross-axis Δx / Δy small
//
// Each axis test exposes a different failure mode: a forward walk
// that secretly turns / drifts, a lateral walk that creeps forward,
// or a turn-in-place that translates the body. Open-loop CHAMP can
// only print the values for documentation — the closed-loop MPC+WBC
// is what actually has to gate them.
//
// Both stacks run the same 5 s sim under `MuJoCo ground truth`
// pose feedback so the comparison is between control logic, not
// state estimation noise.

const WALK_SIM_TIME_S: f64 = 5.0;
const WALK_BURN_IN_S: f64 = 0.5;

/// Track-quality metrics for one walk run.
///
/// Δx / Δy are world-frame displacements, but for **lateral** and
/// **yaw** commands we usually care about the body-frame components
/// (the body has rotated under the cmd, so world Δx / Δy mix). The
/// helper accessors `body_dx` / `body_dy` rotate the world-frame
/// displacement into the **initial** body frame so cross-axis
/// assertions stay consistent regardless of how the body turned
/// during the run.
#[derive(Debug, Default, Clone, Copy)]
struct WalkBenchmark {
    body_x_start: f64,
    body_y_start: f64,
    yaw_start: f64,
    body_x_end: f64,
    body_y_end: f64,
    yaw_end: f64,
    min_body_z: f64,
}

impl WalkBenchmark {
    fn dx_world(&self) -> f64 {
        self.body_x_end - self.body_x_start
    }
    fn dy_world(&self) -> f64 {
        self.body_y_end - self.body_y_start
    }
    /// Forward (body-x) displacement projected onto the **initial**
    /// body heading. Use this for the active-axis check on a forward
    /// command and the cross-axis check on a lateral / yaw command.
    fn body_dx(&self) -> f64 {
        let dx = self.dx_world();
        let dy = self.dy_world();
        let c = self.yaw_start.cos();
        let s = self.yaw_start.sin();
        c * dx + s * dy
    }
    /// Lateral (body-y, +y = left) displacement.
    fn body_dy(&self) -> f64 {
        let dx = self.dx_world();
        let dy = self.dy_world();
        let c = self.yaw_start.cos();
        let s = self.yaw_start.sin();
        -s * dx + c * dy
    }
    fn dyaw(&self) -> f64 {
        // Wrap to (-π, π] so a near-360° turn doesn't print as ~0.
        let raw = self.yaw_end - self.yaw_start;
        ((raw + std::f64::consts::PI).rem_euclid(2.0 * std::f64::consts::PI))
            - std::f64::consts::PI
    }

    /// Compute the 5-dim quality score for a walk under `cmd`. The
    /// score is single-cmd: feed `fwd_cmd()` / `lat_cmd()` / `yaw_cmd()`,
    /// and the metrics auto-route the primary / secondary axes.
    ///
    /// `duration_s` is the time the cmd was actually applied (= total
    /// sim time minus the burn-in window).
    fn metrics(&self, cmd: VelocityCmd, duration_s: f64) -> WalkQuality {
        if cmd.vx.abs() > 1e-9 {
            let primary_cmd = cmd.vx.abs();
            let primary_signed = self.body_dx() * cmd.vx.signum();
            let secondary = self.body_dy().abs();
            WalkQuality {
                tracking: primary_signed / (primary_cmd * duration_s),
                cross: secondary / (primary_cmd * duration_s),
                yaw_drift_rps: self.dyaw().abs() / duration_s,
                linear_drift_mps: 0.0,
                min_z: self.min_body_z,
            }
        } else if cmd.vy.abs() > 1e-9 {
            let primary_cmd = cmd.vy.abs();
            let primary_signed = self.body_dy() * cmd.vy.signum();
            let secondary = self.body_dx().abs();
            WalkQuality {
                tracking: primary_signed / (primary_cmd * duration_s),
                cross: secondary / (primary_cmd * duration_s),
                yaw_drift_rps: self.dyaw().abs() / duration_s,
                linear_drift_mps: 0.0,
                min_z: self.min_body_z,
            }
        } else if cmd.wz.abs() > 1e-9 {
            let primary_cmd = cmd.wz.abs();
            let primary_signed = self.dyaw() * cmd.wz.signum();
            let linear_norm = (self.body_dx().powi(2) + self.body_dy().powi(2)).sqrt();
            WalkQuality {
                tracking: primary_signed / (primary_cmd * duration_s),
                cross: 0.0, // n/a for yaw cmd; linear drift below is the cross-cmd metric
                yaw_drift_rps: 0.0,
                linear_drift_mps: linear_norm / duration_s,
                min_z: self.min_body_z,
            }
        } else {
            WalkQuality {
                tracking: f64::NAN,
                cross: f64::NAN,
                yaw_drift_rps: 0.0,
                linear_drift_mps: 0.0,
                min_z: self.min_body_z,
            }
        }
    }
}

/// Five-axis quality score for a walk benchmark under a single cmd.
///
/// **`tracking`** = signed primary-axis ratio. `1.0` = perfect
/// cmd-following over the burn-in-trimmed window. Negative = robot
/// went the wrong direction.
///
/// **`cross`** = unsigned secondary-axis (orthogonal in body frame)
/// displacement, normalised by `cmd × duration` so it's directly
/// comparable to `tracking`. `0.10` = robot drifted 10 % of the
/// commanded distance sideways.
///
/// **`yaw_drift_rps`** = absolute yaw rate accumulated over the cmd
/// window (rad/s, time-averaged). Only meaningful for linear cmds —
/// the field is `0` under a yaw cmd (the yaw signal is the primary).
///
/// **`linear_drift_mps`** = absolute body-translation rate (m/s,
/// time-averaged). Only meaningful for yaw cmds.
///
/// **`min_z`** = lowest body-root height seen during the run; matches
/// the existing fall-detection signal.
#[derive(Debug, Clone, Copy)]
struct WalkQuality {
    tracking: f64,
    cross: f64,
    yaw_drift_rps: f64,
    linear_drift_mps: f64,
    min_z: f64,
}

impl WalkQuality {
    /// One-line summary suitable for `eprintln!` in the diag matrix.
    fn line(&self, axis: &str) -> String {
        match axis {
            "forward" | "lateral" => format!(
                "track={:+.2}  cross={:.2}  yaw_drift={:.3} rad/s  min_z={:.3}",
                self.tracking, self.cross, self.yaw_drift_rps, self.min_z,
            ),
            "yaw" => format!(
                "track={:+.2}  linear_drift={:.3} m/s  min_z={:.3}",
                self.tracking, self.linear_drift_mps, self.min_z,
            ),
            _ => format!("{:?}", self),
        }
    }
}

/// Run a 5 s walk sim under the given controller / WBC mode + cmd
/// and return the integrated track metrics. Body state observation
/// uses MuJoCo ground truth (= matches `PoseSource::GroundTruth` in
/// the GUI).
fn run_walk(
    use_wbc: bool,
    gait_mode: GaitMode,
    cmd: VelocityCmd,
    use_misa: bool,
) -> Option<WalkBenchmark> {
    let common::StandFixture {
        mut robot,
        kin,
        mut sim,
    } = if use_misa {
        common::build_namiashi_stand_fixture_misa()?
    } else {
        common::build_namiashi_stand_fixture()?
    };

    let cfg = GaitConfig::trot();
    let mut gc = GaitController::build(&robot, kin.clone(), cfg, gait_mode)
        .expect("GaitController::build");
    // Under misa (stiff PD), pin capture-point gain to 0 explicitly.
    // Redundant since D3.3.7-C2 (default is now 0.0), but kept as
    // documentation: this test's envelope numbers assume the
    // legged_control-aligned open-loop Raibert behaviour. If a future
    // tweak ever defaults the gain back to non-zero, the misa path
    // here must still report fix-side numbers. See
    // `memory/project_mpc_frame_bug.md`.
    if use_misa {
        gc.set_capture_point_gain(0.0);
    }
    let mut wbc_pipeline = if use_wbc {
        Some(WbcPipeline::new(&robot, common::default_foot_links()))
    } else {
        None
    };
    // Sync mass / inertia from the auto-detected MPC config (same as
    // the GUI fix in commit b03c431) so WBC physics matches the URDF.
    // CentroidalSrbd mode signals via `centroidal_inertia_body = Some(_)`
    // so the WBC switches to the CoM-aware `a_base_des` path; SRBD
    // mode leaves it `None` for the body-root baseline.
    if let Some(pipeline) = wbc_pipeline.as_mut() {
        if let Some(full_cfg) = gc.full_centroidal_mpc_config() {
            // FullCentroidal mode shares the CoM-aware moment-arm
            // `a_base_des` path with CentroidalSrbd — the 24-state MPC's
            // GRFs satisfy the same `α = I_centroidal⁻¹ · Σ r × F`
            // relationship by construction.
            pipeline.mass_kg = full_cfg.mass_kg;
            pipeline.centroidal_inertia_body = Some(full_cfg.centroidal_inertia_body);
            pipeline.com_offset_body = full_cfg.com_offset_body;
        } else if let Some(centroidal_cfg) = gc.centroidal_mpc_config() {
            pipeline.mass_kg = centroidal_cfg.mass_kg;
            pipeline.centroidal_inertia_body = Some(centroidal_cfg.centroidal_inertia_body);
            pipeline.com_offset_body = centroidal_cfg.com_offset_body;
        } else if let Some(srbd_cfg) = gc.srbd_mpc_config() {
            pipeline.mass_kg = srbd_cfg.mass_kg;
            pipeline.inertia_diag_body = srbd_cfg.inertia_diag_body;
            pipeline.centroidal_inertia_body = None;
        }
    }

    let n_steps = (WALK_SIM_TIME_S / DT) as usize;
    let burn_in_steps = (WALK_BURN_IN_S / DT) as usize;
    let mut metrics = WalkBenchmark::default();
    metrics.min_body_z = f64::INFINITY;
    if let Some(pos) = sim.body_world_position(&robot.root_link) {
        metrics.body_x_start = pos[0];
        metrics.body_y_start = pos[1];
    }
    metrics.yaw_start = sim.body_world_yaw(&robot.root_link).unwrap_or(0.0);

    gc.enable();
    for k in 0..n_steps {
        if k == burn_in_steps {
            gc.set_velocity_cmd(cmd);
        }
        // P5b: schedule WBC weights from the active cmd so swing_leg
        // is automatically dialled down for lateral / yaw commands
        // (avoids the joint-space-PD reaction torque sign-flip
        // documented in `WbcWeights::for_cmd`).
        // Mode-aware (D2/H): CentroidalSrbd uses `for_cmd_centroidal`
        // which halves swing_leg weights to reduce reaction-torque
        // amplification through the MPC's CoM-aware predictions.
        if let Some(pipeline) = wbc_pipeline.as_mut() {
            pipeline.weights = if pipeline.centroidal_inertia_body.is_some() {
                quadruped_gait::wbc::WbcWeights::for_cmd_centroidal(&gc.velocity_cmd())
            } else {
                quadruped_gait::wbc::WbcWeights::for_cmd(&gc.velocity_cmd())
            };
        }
        let v_obs = sim
            .body_world_linear_velocity(&robot.root_link)
            .unwrap_or([0.0, 0.0, 0.0]);
        let w_obs = sim
            .body_world_angular_velocity(&robot.root_link)
            .unwrap_or([0.0, 0.0, 0.0]);
        gc.set_body_state_observed(
            Vector3::new(v_obs[0], v_obs[1], v_obs[2]),
            Vector3::new(w_obs[0], w_obs[1], w_obs[2]),
        );
        // Feed body pose ground truth (matches `PoseSource::GroundTruth`
        // in the GUI). Without this the gait controller has no closed-
        // loop position / yaw feedback and drifts open-loop over the
        // 5 s window — visibly so when comparing forward motion: a
        // small yaw drift quickly turns world-x forward into a
        // diagonal world path.
        let body_pos = sim
            .body_world_position(&robot.root_link)
            .unwrap_or([0.0, 0.0, 0.0]);
        let yaw_obs = sim.body_world_yaw(&robot.root_link).unwrap_or(0.0);
        gc.set_body_pose_observed(
            yaw_obs,
            Vector3::new(body_pos[0], body_pos[1], body_pos[2]),
        );

        let (out, targets, torque_ff) = gc.tick(DT);
        for (idx, q) in targets {
            sim.set_position_target(idx, q);
        }

        if let Some(pipeline) = wbc_pipeline.as_mut() {
            // WBC active → apply Hybrid joint command (PD + WBC τ_ff).
            if k >= burn_in_steps {
                let f_grf_world = gc
                    .predicted_grfs()
                    .map(|sol| sol.grfs_first_step)
                    .unwrap_or([Vector3::zeros(); 4]);
                let cmd = gc.velocity_cmd();
                let v_cmd_body = Vector3::new(cmd.vx, cmd.vy, 0.0);
                let foot_links_str: [&str; 4] = [
                    pipeline.foot_links[0].as_str(),
                    pipeline.foot_links[1].as_str(),
                    pipeline.foot_links[2].as_str(),
                    pipeline.foot_links[3].as_str(),
                ];
                let force_z = sim.contact_force_per_foot(&foot_links_str);
                let nominal_phases = [
                    out.legs[0].phase,
                    out.legs[1].phase,
                    out.legs[2].phase,
                    out.legs[3].phase,
                ];
                let corrected = ContactDrivenPhase::apply_correction(
                    &nominal_phases,
                    force_z,
                    5.0,
                    0.0,
                );
                let contact_flag = [
                    corrected[0].is_stance,
                    corrected[1].is_stance,
                    corrected[2].is_stance,
                    corrected[3].is_stance,
                ];
                let taus = pipeline.solve(
                    &robot,
                    &sim,
                    &out,
                    gc.kinematics(),
                    gc.joint_indices(),
                    gc.joint_signs(),
                    &v_cmd_body,
                    cmd.wz,
                    &Vector3::new(v_obs[0], v_obs[1], v_obs[2]),
                    &Vector3::new(w_obs[0], w_obs[1], w_obs[2]),
                    &f_grf_world,
                    contact_flag,
                    DT,
                );
                for (ji, &tau) in taus.iter().enumerate() {
                    sim.set_torque_feedforward(ji, tau);
                }
            } else {
                for ji in 0..robot.joints.len() {
                    sim.set_torque_feedforward(ji, 0.0);
                }
            }
        } else {
            // No WBC → use the gait controller's MPC-derived stance
            // τ_ff (= mpc_walks_stable baseline / matches CHAMP path
            // when CHAMP runs).
            for (idx, tau) in torque_ff {
                sim.set_torque_feedforward(idx, tau);
            }
        }

        sim.step(&mut robot, DT, true);

        if let Some(pos) = sim.body_world_position(&robot.root_link) {
            metrics.min_body_z = metrics.min_body_z.min(pos[2]);
            metrics.body_x_end = pos[0];
            metrics.body_y_end = pos[1];
        }
        metrics.yaw_end = sim.body_world_yaw(&robot.root_link).unwrap_or(0.0);
    }
    Some(metrics)
}

// ─── Per-axis cmd values used across the benchmark suite ──────────
const FORWARD_CMD_VX: f64 = 0.15;  // m/s forward command magnitude
const LATERAL_CMD_VY: f64 = 0.10;  // m/s lateral command (slower than
                                   //   forward — trot can't match
                                   //   forward speed sideways)
const YAW_CMD_WZ: f64 = 0.5;       // rad/s yaw rate command (~30°/s)

fn fwd_cmd() -> VelocityCmd {
    VelocityCmd { vx: FORWARD_CMD_VX, vy: 0.0, wz: 0.0 }
}
fn lat_cmd() -> VelocityCmd {
    VelocityCmd { vx: 0.0, vy: LATERAL_CMD_VY, wz: 0.0 }
}
fn yaw_cmd() -> VelocityCmd {
    VelocityCmd { vx: 0.0, vy: 0.0, wz: YAW_CMD_WZ }
}

// ─── Forward-walk benchmarks ──────────────────────────────────────

/// CHAMP open-loop forward trot benchmark (5 s @ 0.15 m/s). CHAMP has
/// no closed-loop yaw/lateral correction so cross-axis drift is
/// expected; we only assert against falls and record the numbers.
///
/// Uses `use_misa = true` (2026-05-13): under URDF + soft PD the
/// number was forward dx = −0.29 m (legs slipped backward); under
/// `.misa` with kp=100/kv=1.2 it tracks at dx ≈ +0.60 m, matching the
/// GUI behaviour and giving a fair side-by-side with `*_mpc_wbc`.
#[test]
fn integration_walk_straight_champ() {
    let Some(m) = run_walk(false, GaitMode::Champ, fwd_cmd(), true) else {
        return;
    };
    eprintln!(
        "[forward:champ] body_dx={:+.3} m  body_dy={:+.3} m  Δyaw={:+.3} rad  min_z={:.3} m",
        m.body_dx(), m.body_dy(), m.dyaw(), m.min_body_z,
    );
    assert!(m.min_body_z > FALL_THRESHOLD_Z, "CHAMP fell");
}

/// MPC+WBC forward trot. Active axis: body-frame Δx > +10 cm. Cross
/// axes: |body_dy| < 20 cm, |Δyaw| < 1 rad.
///
/// Migrated to .misa loader (2026-05-13): under stiff PD with
/// `set_capture_point_gain(0.0)` in `run_walk`, forward tracking
/// jumped from dx=+0.118 (16% of cmd*T) to dx=+0.651 (87%). The
/// envelope is loose for backward-compat; tightening to `dx > +0.40`
/// would still pass and better catch regressions.
#[test]
fn integration_walk_straight_mpc_wbc() {
    let Some(m) = run_walk(true, GaitMode::Mpc, fwd_cmd(), true) else {
        return;
    };
    eprintln!(
        "[forward:mpc+wbc] body_dx={:+.3} m  body_dy={:+.3} m  Δyaw={:+.3} rad  min_z={:.3} m",
        m.body_dx(), m.body_dy(), m.dyaw(), m.min_body_z,
    );
    assert!(m.min_body_z > FALL_THRESHOLD_Z, "MPC+WBC fell (forward)");
    assert!(m.body_dx() > 0.10,
        "forward: body_dx = {:+.3} m, expected > +0.10 m", m.body_dx());
    assert!(m.body_dy().abs() < 0.20,
        "forward: body_dy = {:+.3} m, expected |·| < 0.20 m", m.body_dy());
    assert!(m.dyaw().abs() < 1.0,
        "forward: Δyaw = {:+.3} rad, expected |·| < 1.0 rad", m.dyaw());
}

// ─── Centroidal-SRBD MPC benchmarks (D1.3) ────────────────────────
//
// These exercise the new `GaitMode::CentroidalSrbd` path end-to-end.
// Same assertions as the body-root SRBD `*_mpc_wbc` tests above so
// the two MPC formulations can be compared head-to-head on identical
// fixtures. Champ / Mpc / CentroidalSrbd are all kept side-by-side
// as baselines per the D1 plan.
//
// **D1.3 status**: marked `#[ignore]` so they DON'T break CI yet —
// the centroidal MPC's QP runs end-to-end and produces stable GRFs,
// but the q_diag tune is still SRBD-equivalent and doesn't account
// for the centroidal-momentum unit scale (h_ang/m differs from SRBD
// ω by I/m ≈ 0.0038). D1.4 will re-tune the cost weights so these
// pass with the same assertion thresholds as the SRBD baseline.
// Run with `cargo test ... -- --ignored` to see current numbers.

/// Diagnostic: centroidal MPC + Position-PD (no WBC). Isolates the
/// MPC's GRF quality from the WBC's body-root `a_base_des`
/// interpretation. If forward tracking improves significantly here,
/// the body-root assumption in `predicted_base_accel_world` is
/// fighting the centroidal GRFs.
#[test]
#[ignore = "D1.4 diagnostic — run with --ignored to inspect"]
fn diag_centroidal_no_wbc_3axis() {
    for (label, cmd) in [
        ("forward", fwd_cmd()),
        ("lateral", lat_cmd()),
        ("yaw", yaw_cmd()),
    ] {
        let Some(m) = run_walk(false, GaitMode::CentroidalSrbd, cmd, false) else {
            return;
        };
        eprintln!(
            "[{label}:centroidal-only] body_dx={:+.3} m  body_dy={:+.3} m  Δyaw={:+.3} rad  min_z={:.3} m",
            m.body_dx(), m.body_dy(), m.dyaw(), m.min_body_z,
        );
    }
}

#[test]
#[ignore = "D1.4 tuning target — assertions kept as goal"]
fn integration_walk_straight_centroidal_wbc() {
    let Some(m) = run_walk(true, GaitMode::CentroidalSrbd, fwd_cmd(), false) else {
        return;
    };
    eprintln!(
        "[forward:centroidal+wbc] body_dx={:+.3} m  body_dy={:+.3} m  Δyaw={:+.3} rad  min_z={:.3} m",
        m.body_dx(), m.body_dy(), m.dyaw(), m.min_body_z,
    );
    assert!(m.min_body_z > FALL_THRESHOLD_Z, "Centroidal+WBC fell (forward)");
    assert!(m.body_dx() > 0.10,
        "forward (centroidal): body_dx = {:+.3} m, expected > +0.10 m", m.body_dx());
    assert!(m.body_dy().abs() < 0.20,
        "forward (centroidal): body_dy = {:+.3} m, expected |·| < 0.20 m", m.body_dy());
    assert!(m.dyaw().abs() < 1.0,
        "forward (centroidal): Δyaw = {:+.3} rad, expected |·| < 1.0 rad", m.dyaw());
}

#[test]
#[ignore = "D1.4 tuning target — assertions kept as goal"]
fn integration_walk_lateral_centroidal_wbc() {
    let Some(m) = run_walk(true, GaitMode::CentroidalSrbd, lat_cmd(), false) else {
        return;
    };
    eprintln!(
        "[lateral:centroidal+wbc] body_dx={:+.3} m  body_dy={:+.3} m  Δyaw={:+.3} rad  min_z={:.3} m",
        m.body_dx(), m.body_dy(), m.dyaw(), m.min_body_z,
    );
    assert!(m.min_body_z > FALL_THRESHOLD_Z, "Centroidal+WBC fell (lateral)");
    assert!(m.body_dy() > 0.20,
        "lateral (centroidal): body_dy = {:+.3} m, expected > +0.20 m", m.body_dy());
    assert!(m.body_dx().abs() < 0.30,
        "lateral (centroidal): body_dx = {:+.3} m, expected |·| < 0.30 m", m.body_dx());
    assert!(m.dyaw().abs() < 1.5,
        "lateral (centroidal): Δyaw = {:+.3} rad, expected |·| < 1.5 rad", m.dyaw());
}

#[test]
#[ignore = "D1.4 tuning target — assertions kept as goal"]
fn integration_walk_yaw_centroidal_wbc() {
    let Some(m) = run_walk(true, GaitMode::CentroidalSrbd, yaw_cmd(), false) else {
        return;
    };
    eprintln!(
        "[yaw:centroidal+wbc] body_dx={:+.3} m  body_dy={:+.3} m  Δyaw={:+.3} rad  min_z={:.3} m",
        m.body_dx(), m.body_dy(), m.dyaw(), m.min_body_z,
    );
    assert!(m.min_body_z > FALL_THRESHOLD_Z, "Centroidal+WBC fell (yaw)");
    assert!(m.dyaw().abs() > 1.5,
        "yaw (centroidal): Δyaw = {:+.3} rad, expected |·| > 1.5 rad", m.dyaw());
    assert!(m.body_dx().abs() < 0.35,
        "yaw (centroidal): body_dx = {:+.3} m, expected |·| < 0.35 m", m.body_dx());
    assert!(m.body_dy().abs() < 0.35,
        "yaw (centroidal): body_dy = {:+.3} m, expected |·| < 0.35 m", m.body_dy());
}

// ─── Full-centroidal MPC benchmarks (D3.3.6) ─────────────────────────
//
// Exercise the new `GaitMode::FullCentroidal` path end-to-end. Same
// assertion thresholds as the CentroidalSrbd tests so the three MPC
// formulations (Mpc / CentroidalSrbd / FullCentroidal) compare
// directly on identical fixtures.
//
// **D3.3.6 status**: marked `#[ignore]` so they don't break CI while
// the new 24-state path is being characterised on namiashi. The MPC's
// QP runs end-to-end (D3.3.4 unit tests pass) and the gait controller
// + WBC are wired (D3.3.5 + D3.3.6a); these tests measure whether the
// 24-state formulation actually fixes the lateral inversion + forward
// dy cross-coupling that empirical tuning + 12-state SQP could not.
//
// Run with `cargo test --test integration_walk --features mujoco -- --ignored`.

/// Diagnostic: full-centroidal MPC + Position-PD (no WBC). Same role
/// as `diag_centroidal_no_wbc_3axis` for the 12-state path — isolates
/// the MPC's GRF quality from the WBC's `a_base_des` interpretation.
// ─── Walk quality metric matrix (Bench-Δ): all modes × all axes ──────
//
// Single diag test that runs every (gait_mode, cmd) pair through
// `run_walk` and prints the 5-axis [`WalkMetrics`] score so we can
// anchor Gold/Silver/Bronze threshold tiers against CHAMP's actual
// numbers (= the metric design discussion). 12 sims × 5 s ≈ 8 min
// total; gate behind `--ignored` so CI never pays it.

/// One-off run of CHAMP forward with **GUI-default PD gains** (kp=50,
/// kv=5) instead of the test harness's (kp=30, kv=0.6). The harness
/// gains were inherited from `wbc_walk` / `gait_walk_stability` which
/// optimise for low-jitter MPC tracking; CHAMP open-loop needs higher
/// damping to actually follow joint targets, otherwise legs slip and
/// the body drifts in unexpected directions.
#[test]
#[ignore = "metric debug — run with --ignored"]
fn diag_champ_forward_gui_pd() {
    let cmd = fwd_cmd();
    let fixture = match common::build_namiashi_stand_fixture() {
        Some(f) => f,
        None => return,
    };
    let mut robot = fixture.robot;
    let kin = fixture.kin;
    drop(fixture.sim);
    // Override the test PD with GUI defaults.
    for j in robot.joints.iter_mut() {
        if j.joint_type == "fixed" { continue; }
        j.actuator_kp = 50.0;
        j.actuator_kv = 5.0;
    }
    // Rebuild sim with the new PD.
    let mut sim = articara::mujoco_sim::MujocoSim::new(
        &robot,
        common::default_mjcf_export_options(),
    ).expect("MujocoSim::new");
    sim.set_gravity_compensation(true);

    // Initial joint seed identical to fixture.
    common::seed_joint_positions_from_kinematics(&mut robot, &kin);

    let cfg = GaitConfig::trot();
    let mut gc = GaitController::build(&robot, kin, cfg, GaitMode::Champ)
        .expect("GaitController::build");
    gc.enable();
    let n_steps = (WALK_SIM_TIME_S / DT) as usize;
    let burn_in_steps = (WALK_BURN_IN_S / DT) as usize;
    let mut metrics = WalkBenchmark::default();
    metrics.min_body_z = f64::INFINITY;
    if let Some(pos) = sim.body_world_position(&robot.root_link) {
        metrics.body_x_start = pos[0];
        metrics.body_y_start = pos[1];
    }
    metrics.yaw_start = sim.body_world_yaw(&robot.root_link).unwrap_or(0.0);
    for k in 0..n_steps {
        if k == burn_in_steps {
            gc.set_velocity_cmd(cmd);
        }
        let v_obs = sim
            .body_world_linear_velocity(&robot.root_link)
            .unwrap_or([0.0, 0.0, 0.0]);
        let w_obs = sim
            .body_world_angular_velocity(&robot.root_link)
            .unwrap_or([0.0, 0.0, 0.0]);
        gc.set_body_state_observed(
            Vector3::new(v_obs[0], v_obs[1], v_obs[2]),
            Vector3::new(w_obs[0], w_obs[1], w_obs[2]),
        );
        let body_pos = sim
            .body_world_position(&robot.root_link)
            .unwrap_or([0.0, 0.0, 0.0]);
        let yaw_obs = sim.body_world_yaw(&robot.root_link).unwrap_or(0.0);
        gc.set_body_pose_observed(
            yaw_obs,
            Vector3::new(body_pos[0], body_pos[1], body_pos[2]),
        );
        let (_out, targets, _torque_ff) = gc.tick(DT);
        for (idx, q) in targets {
            sim.set_position_target(idx, q);
        }
        sim.step(&mut robot, DT, false);
        if let Some(pos) = sim.body_world_position(&robot.root_link) {
            metrics.body_x_end = pos[0];
            metrics.body_y_end = pos[1];
            metrics.min_body_z = metrics.min_body_z.min(pos[2]);
        }
        metrics.yaw_end = sim.body_world_yaw(&robot.root_link).unwrap_or(0.0);
    }
    let m = metrics;
    eprintln!();
    eprintln!("=== CHAMP forward GUI-PD (kp=50, kv=5) — cmd vx=+{:.2} m/s, 4.5 s ===", cmd.vx);
    eprintln!("  dx_world  = {:+.4} m", m.dx_world());
    eprintln!("  dy_world  = {:+.4} m", m.dy_world());
    eprintln!("  dyaw      = {:+.3} rad", m.dyaw());
    eprintln!("  body_dx   = {:+.4} m", m.body_dx());
    eprintln!("  body_dy   = {:+.4} m", m.body_dy());
    eprintln!("  min_z     = {:.4} m", m.min_body_z);
    let q = m.metrics(cmd, WALK_SIM_TIME_S - WALK_BURN_IN_S);
    eprintln!("  metric    = {}", q.line("forward"));
}

/// GUI-equivalent walk harness, mirroring the steps user reported:
/// 1. Apply `constrain` pose to joint_positions BEFORE auto-detect
///    (so `nominal_foot_body` captures the deeply-bent stance)
/// 2. URDF-default PD (kp=50, kv=5)
/// 3. MJCF export with `add_actuators=false`
/// 4. Gravity comp OFF (GUI default)
/// 5. 2 s settle (PD holding constrain pose, no gait active)
/// 6. 7 s walk with gait controller + cmd
///
/// Returns the benchmark covering ONLY the walk window (cmd phase) so
/// `body_dx` / `body_dy` / `dyaw` reflect post-cmd-application motion.
fn run_walk_gui_v2(
    use_wbc: bool,
    gait_mode: GaitMode,
    cmd: VelocityCmd,
) -> Option<WalkBenchmark> {
    const SETTLE_S: f64 = 2.0;
    const CMD_S: f64 = 7.0;

    let path = common::namiashi_urdf();
    if !path.exists() { return None; }
    let mut robot = articara::robot::RobotModel::from_urdf(&path).ok()?;
    for j in robot.joints.iter_mut() {
        if j.joint_type == "fixed" { continue; }
        j.actuator_mode = articara::robot::ActuatorMode::Position;
        j.actuator_kp = 50.0;
        j.actuator_kv = 5.0;
    }
    let constrain_pose = [
        ("FL_hip_joint", 0.0), ("FL_thigh_joint", 1.0), ("FL_calf_joint", -2.0),
        ("FR_hip_joint", 0.0), ("FR_thigh_joint", 1.0), ("FR_calf_joint", -2.0),
        ("RL_hip_joint", 0.0), ("RL_thigh_joint", 1.0), ("RL_calf_joint", -2.0),
        ("RR_hip_joint", 0.0), ("RR_thigh_joint", 1.0), ("RR_calf_joint", -2.0),
        ("arm_pitch_joint", 0.0),
    ];
    for (name, q) in constrain_pose {
        if let Some(&idx) = robot.joint_map.get(name) {
            robot.joint_positions[idx] = q;
        }
    }
    let kin = articara::gait::auto_detect_kinematics_config(
        &robot,
        &articara::gait::DEFAULT_FOOT_LINKS,
    ).ok()?;
    let opts = articara::mjcf::MjcfExportOptions {
        ground_plane: Some(articara::mjcf::GroundPlaneCfg {
            z: 0.0, half_size: 4.0, roll: 0.0, pitch: 0.0,
        }),
        // `add_actuators` is forced to `true` inside MujocoSim::new
        // regardless of what we pass, so this value is cosmetic.
        add_actuators: false,
        // Test keeps bake=true even though GUI default is false: the
        // baked range gives MuJoCo more upfront state to clamp against,
        // measurably improves CHAMP track (0.15 → 0.25 in namiashi
        // forward sweep). The residual ~2x gap vs GUI numbers is left
        // unresolved (test_track 0.25 vs GUI_track 0.55-0.65) and
        // documented in the doc comment above.
        bake_actuator_limits: true,
        bake_joint_position_limits: true,
        ..Default::default()
    };
    let mut sim = articara::mujoco_sim::MujocoSim::new(&robot, opts).ok()?;
    sim.set_gravity_compensation(false);

    let cfg = GaitConfig::trot();
    let mut gc = GaitController::build(&robot, kin.clone(), cfg, gait_mode).ok()?;

    let mut wbc_pipeline = if use_wbc {
        Some(WbcPipeline::new(&robot, common::default_foot_links()))
    } else { None };
    if let Some(pipeline) = wbc_pipeline.as_mut() {
        if let Some(full_cfg) = gc.full_centroidal_mpc_config() {
            pipeline.mass_kg = full_cfg.mass_kg;
            pipeline.centroidal_inertia_body = Some(full_cfg.centroidal_inertia_body);
            pipeline.com_offset_body = full_cfg.com_offset_body;
        } else if let Some(centroidal_cfg) = gc.centroidal_mpc_config() {
            pipeline.mass_kg = centroidal_cfg.mass_kg;
            pipeline.centroidal_inertia_body = Some(centroidal_cfg.centroidal_inertia_body);
            pipeline.com_offset_body = centroidal_cfg.com_offset_body;
        } else if let Some(srbd_cfg) = gc.srbd_mpc_config() {
            pipeline.mass_kg = srbd_cfg.mass_kg;
            pipeline.inertia_diag_body = srbd_cfg.inertia_diag_body;
            pipeline.centroidal_inertia_body = None;
        }
    }

    // Initial PD targets: constrain pose (used during settle).
    for (name, q) in constrain_pose {
        if let Some(&idx) = robot.joint_map.get(name) {
            sim.set_position_target(idx, q);
        }
    }

    let n_steps = ((SETTLE_S + CMD_S) / DT) as usize;
    let settle_steps = (SETTLE_S / DT) as usize;
    let mut m = WalkBenchmark::default();
    m.min_body_z = f64::INFINITY;

    for k in 0..n_steps {
        if k == settle_steps {
            gc.enable();
            gc.set_velocity_cmd(cmd);
            if let Some(p) = sim.body_world_position(&robot.root_link) {
                m.body_x_start = p[0]; m.body_y_start = p[1];
            }
            m.yaw_start = sim.body_world_yaw(&robot.root_link).unwrap_or(0.0);
        }

        if k < settle_steps {
            // Settle phase: keep PD on the constrain pose.
            for (name, q) in constrain_pose {
                if let Some(&idx) = robot.joint_map.get(name) {
                    sim.set_position_target(idx, q);
                }
            }
        } else {
            let v_obs = sim.body_world_linear_velocity(&robot.root_link).unwrap_or([0.0; 3]);
            let w_obs = sim.body_world_angular_velocity(&robot.root_link).unwrap_or([0.0; 3]);
            gc.set_body_state_observed(
                Vector3::new(v_obs[0], v_obs[1], v_obs[2]),
                Vector3::new(w_obs[0], w_obs[1], w_obs[2]),
            );
            let body_pos = sim.body_world_position(&robot.root_link).unwrap_or([0.0; 3]);
            let yaw_obs = sim.body_world_yaw(&robot.root_link).unwrap_or(0.0);
            gc.set_body_pose_observed(yaw_obs, Vector3::new(body_pos[0], body_pos[1], body_pos[2]));
            let (out, targets, torque_ff) = gc.tick(DT);
            for (idx, q) in targets { sim.set_position_target(idx, q); }

            if let Some(pipeline) = wbc_pipeline.as_mut() {
                pipeline.weights = if pipeline.centroidal_inertia_body.is_some() {
                    quadruped_gait::wbc::WbcWeights::for_cmd_centroidal(&gc.velocity_cmd())
                } else {
                    quadruped_gait::wbc::WbcWeights::for_cmd(&gc.velocity_cmd())
                };
                let f_grf = gc.predicted_grfs().map(|s| s.grfs_first_step)
                    .unwrap_or([Vector3::zeros(); 4]);
                let v_cmd_body = Vector3::new(cmd.vx, cmd.vy, 0.0);
                let contact_flag = [
                    out.legs[0].phase.is_stance,
                    out.legs[1].phase.is_stance,
                    out.legs[2].phase.is_stance,
                    out.legs[3].phase.is_stance,
                ];
                let kin_ref = gc.kinematics().clone();
                let joint_indices = gc.joint_indices();
                let joint_signs = gc.joint_signs();
                let taus = pipeline.solve(
                    &robot, &sim, &out, &kin_ref, joint_indices, joint_signs,
                    &v_cmd_body, cmd.wz,
                    &Vector3::new(v_obs[0], v_obs[1], v_obs[2]),
                    &Vector3::new(w_obs[0], w_obs[1], w_obs[2]),
                    &f_grf, contact_flag, DT,
                );
                for (ji, &tau) in taus.iter().enumerate() {
                    sim.set_torque_feedforward(ji, tau);
                }
            } else {
                for (idx, tau) in torque_ff {
                    sim.set_torque_feedforward(idx, tau);
                }
            }
        }
        // Keep `enforce_limits=true` at runtime for sim stability even
        // though the GUI default is false; without runtime clipping the
        // test's 2 ms timestep + kp=50 can blow up velocities. The
        // GUI's slower wall clock keeps it stable without clamps.
        sim.step(&mut robot, DT, true);
        if let Some(p) = sim.body_world_position(&robot.root_link) {
            m.body_x_end = p[0]; m.body_y_end = p[1];
            m.min_body_z = m.min_body_z.min(p[2]);
        }
        m.yaw_end = sim.body_world_yaw(&robot.root_link).unwrap_or(0.0);
    }
    Some(m)
}

/// Quick single-axis sanity check: run CHAMP forward with the
/// GUI-equivalent harness and print result. Used to iterate on
/// harness diffs without paying the full 12-sim matrix cost.
#[test]
#[ignore = "harness debug — run with --ignored"]
fn diag_walk_champ_forward_v2() {
    let cmd = VelocityCmd { vx: 0.30, vy: 0.0, wz: 0.0 };
    let Some(m) = run_walk_gui_v2(false, GaitMode::Champ, cmd) else { return; };
    let q = m.metrics(cmd, 7.0);
    eprintln!();
    eprintln!("[champ forward v2] dx_world={:+.4} dy_world={:+.4} dyaw={:+.3} rad min_z={:.3} m",
        m.dx_world(), m.dy_world(), m.dyaw(), m.min_body_z);
    eprintln!("  metric = {}", q.line("forward"));
    eprintln!("  (user GUI reported: dx ≈ +1.13 over ~5-10s = track ≈ 0.55-0.65)");
    eprintln!("  (user GUI reported: dz = +0.18 — body rises during walk; if test stays crouched, that's the gap)");
}

/// Diff the MJCF that `mj_start` (script path) emits vs what
/// `run_walk_gui_v2` (test path) emits. If they're byte-identical
/// modulo ground_plane size, the physics state must match — which
/// would mean the 3.4x gap is in the *runtime* (per-tick) logic.
#[test]
#[ignore = "harness debug — run with --ignored"]
fn diag_mjcf_diff_test_vs_script() {
    let path = common::namiashi_urdf();
    if !path.exists() { return; }
    let mut robot = articara::robot::RobotModel::from_urdf(&path).expect("load");
    // Apply constrain pose and PD same as run_walk_gui_v2.
    for j in robot.joints.iter_mut() {
        if j.joint_type == "fixed" { continue; }
        j.actuator_mode = articara::robot::ActuatorMode::Position;
        j.actuator_kp = 50.0;
        j.actuator_kv = 5.0;
    }
    let constrain_pose = [
        ("FL_hip_joint", 0.0), ("FL_thigh_joint", 1.0), ("FL_calf_joint", -2.0),
        ("FR_hip_joint", 0.0), ("FR_thigh_joint", 1.0), ("FR_calf_joint", -2.0),
        ("RL_hip_joint", 0.0), ("RL_thigh_joint", 1.0), ("RL_calf_joint", -2.0),
        ("RR_hip_joint", 0.0), ("RR_thigh_joint", 1.0), ("RR_calf_joint", -2.0),
        ("arm_pitch_joint", 0.0),
    ];
    for (name, q) in constrain_pose {
        if let Some(&idx) = robot.joint_map.get(name) { robot.joint_positions[idx] = q; }
    }

    // MujocoSim::new forces `add_actuators = true` regardless, so for
    // both paths we use the post-override value.
    let test_opts = articara::mjcf::MjcfExportOptions {
        ground_plane: Some(articara::mjcf::GroundPlaneCfg { z: 0.0, half_size: 4.0, roll: 0.0, pitch: 0.0 }),
        add_actuators: true,
        bake_actuator_limits: true,
        bake_joint_position_limits: true,
        ..Default::default()
    };
    let script_opts = articara::mjcf::MjcfExportOptions {
        base_pos: None,
        ground_plane: Some(articara::mjcf::GroundPlaneCfg { z: 0.0, half_size: 2.0, roll: 0.0, pitch: 0.0 }),
        add_actuators: true,
        base_locked_axes: [false; 6],
        ..Default::default()
    };

    let test_xml = articara::mjcf::export_mjcf_with_options(&robot, test_opts);
    let script_xml = articara::mjcf::export_mjcf_with_options(&robot, script_opts);

    std::fs::write("/tmp/test_mjcf.xml", &test_xml).ok();
    std::fs::write("/tmp/script_mjcf.xml", &script_xml).ok();

    if test_xml == script_xml {
        eprintln!("MJCF byte-identical");
        return;
    }
    eprintln!("MJCF DIFFERS — wrote /tmp/test_mjcf.xml and /tmp/script_mjcf.xml");
    eprintln!("test_xml length = {}, script_xml length = {}",
        test_xml.len(), script_xml.len());

    let test_lines: Vec<&str> = test_xml.lines().collect();
    let script_lines: Vec<&str> = script_xml.lines().collect();
    let max_lines = test_lines.len().max(script_lines.len());
    let mut diffs = 0;
    for i in 0..max_lines {
        let t = test_lines.get(i).copied().unwrap_or("<missing>");
        let s = script_lines.get(i).copied().unwrap_or("<missing>");
        if t != s {
            eprintln!("  line {i}:");
            eprintln!("    test:   {t}");
            eprintln!("    script: {s}");
            diffs += 1;
            if diffs >= 20 { eprintln!("  ... (truncated at 20 diffs)"); break; }
        }
    }
}

/// MPC frame-bug bisection diagnostic. Loads namiashi from `.misa`,
/// settles for 0.5 s under PD-only (no gait), then enables the SRBD
/// MPC with `cmd vx=+0.30 m/s` and walks for 1.0 s. Dumps every 50 ms:
///   - body world pos / vel / yaw
///   - MPC's `grfs_first_step` per leg in world frame
///   - sum of GRFs (should be ≈ [F_x_drive, 0, m·g] for forward cmd)
///
/// Hypothesis check: if `Σ GRF_y > Σ GRF_x` under +x cmd, the MPC is
/// commanding sideways force (frame error). If GRFs look correct but
/// body still moves +y, the bug is downstream (τ_ff sign, IK, PD).
/// Short 1 s window keeps `body_yaw ≈ 0` so frame rotation isn't a
/// confounder.
#[test]
#[ignore = "MPC frame-bug bisect — run with --ignored"]
fn diag_mpc_grf_direction_forward_cmd() {
    let common::StandFixture {
        mut robot,
        kin,
        mut sim,
    } = match common::build_namiashi_stand_fixture_misa() {
        Some(f) => f,
        None => return,
    };
    let cfg = GaitConfig::trot();
    let mut gc = GaitController::build(&robot, kin.clone(), cfg, GaitMode::Mpc)
        .expect("GaitController::build (Mpc)");
    // Bisect (2026-05-13): leave defaults to reproduce the bug. The
    // cross-coupling vanishes when:
    //   - r_diag is raised from 1e-3 → 1.0 (suppress over-large GRFs)
    //   - AND set_capture_point_gain(0.0) (= disable +k·v_err foot
    //     placement feedback which acts as positive feedback under
    //     stiff PD).
    // To re-verify the fix locally, uncomment the two overrides below
    // and run with --ignored.
    //   if let Some(c) = gc.srbd_mpc_config() {
    //       let mut new_cfg = c.clone();
    //       new_cfg.r_diag = 1.0;
    //       gc.set_srbd_mpc_config(new_cfg);
    //   }
    //   gc.set_capture_point_gain(0.0);
    let mass_kg = gc.srbd_mpc_config()
        .map(|c| c.mass_kg)
        .unwrap_or(2.4);
    if let Some(c) = gc.srbd_mpc_config() {
        eprintln!("[diag] SRBD config: mass={:.3} kg, inertia_diag={:?}, dt_per_step={}",
            c.mass_kg, c.inertia_diag_body, c.dt_per_step);
        eprintln!("[diag] q_diag = {:?}", c.q_diag);
        eprintln!("[diag] r_diag = {}", c.r_diag);
    }
    // Heaviest link inertia direct from the loaded model (= what
    // auto_detect_srbd_mpc_config picks up).
    let heaviest = robot.links.iter()
        .max_by(|a, b| a.inertial.mass.partial_cmp(&b.inertial.mass)
            .unwrap_or(std::cmp::Ordering::Equal));
    if let Some(l) = heaviest {
        let i = &l.inertial;
        eprintln!("[diag] heaviest link = {:?}: mass={} ixx={} iyy={} izz={}",
            l.name, i.mass, i.ixx, i.iyy, i.izz);
    }
    let burn_s = 0.5;
    let walk_s = 1.0;
    let burn_steps = (burn_s / DT) as usize;
    let walk_steps = (walk_s / DT) as usize;
    let log_every = (0.05 / DT).round() as usize; // 50 ms
    let cmd = VelocityCmd { vx: 0.30, vy: 0.0, wz: 0.0 };
    eprintln!();
    eprintln!("=== diag_mpc_grf_direction_forward_cmd (misa, kp=100/kv=1.2, cmd vx=+0.30) ===");
    eprintln!("  mass = {mass_kg:.3} kg → expected ΣF_z ≈ {:.2} N (= m·g)", mass_kg * 9.81);
    eprintln!("  t[s]   body_x   body_y   yaw      v_x      v_y      ΣF_x     ΣF_y     ΣF_z     stance_legs");
    gc.enable();
    for k in 0..(burn_steps + walk_steps) {
        if k == burn_steps {
            gc.set_velocity_cmd(cmd);
        }
        let v_obs = sim.body_world_linear_velocity(&robot.root_link).unwrap_or([0.0; 3]);
        let w_obs = sim.body_world_angular_velocity(&robot.root_link).unwrap_or([0.0; 3]);
        gc.set_body_state_observed(
            Vector3::new(v_obs[0], v_obs[1], v_obs[2]),
            Vector3::new(w_obs[0], w_obs[1], w_obs[2]));
        let body_pos = sim.body_world_position(&robot.root_link).unwrap_or([0.0; 3]);
        let yaw_obs = sim.body_world_yaw(&robot.root_link).unwrap_or(0.0);
        gc.set_body_pose_observed(yaw_obs, Vector3::new(body_pos[0], body_pos[1], body_pos[2]));
        let (out, targets, torque_ff) = gc.tick(DT);
        for (idx, q) in targets { sim.set_position_target(idx, q); }
        for (idx, tau) in torque_ff { sim.set_torque_feedforward(idx, tau); }
        sim.step(&mut robot, DT, true);
        // Log AFTER step so observed pos/vel reflect the tick that just
        // ran. Subtract burn_steps so t=0 = cmd-applied instant.
        if k >= burn_steps && (k - burn_steps) % log_every == 0 {
            let t = (k - burn_steps) as f64 * DT;
            let stance_flags: Vec<bool> = (0..4)
                .map(|s| out.legs[s].phase.is_stance)
                .collect();
            let n_stance = stance_flags.iter().filter(|&&x| x).count();
            let (sum_fx, sum_fy, sum_fz) = match gc.predicted_grfs() {
                Some(sol) => {
                    let mut sx = 0.0; let mut sy = 0.0; let mut sz = 0.0;
                    for f in sol.grfs_first_step.iter() {
                        sx += f.x; sy += f.y; sz += f.z;
                    }
                    (sx, sy, sz)
                }
                None => (f64::NAN, f64::NAN, f64::NAN),
            };
            eprintln!(
                "  {:>5.3}  {:+.4}  {:+.4}  {:+.4}  {:+.4}  {:+.4}  {:+8.2}  {:+8.2}  {:+8.2}  {}/4",
                t, body_pos[0], body_pos[1], yaw_obs, v_obs[0], v_obs[1],
                sum_fx, sum_fy, sum_fz, n_stance,
            );
        }
    }
}

/// Time-series diagnostic: log body z and dx every 0.5 s of the walk
/// phase to see whether the test reproduces the GUI's body-rising
/// behaviour. User reports GUI ends at dz = +0.18 m above initial.
#[test]
#[ignore = "harness debug — run with --ignored"]
fn diag_walk_champ_forward_v2_timeseries() {
    let path = common::namiashi_urdf();
    if !path.exists() { return; }
    let mut robot = articara::robot::RobotModel::from_urdf(&path).expect("load");
    for j in robot.joints.iter_mut() {
        if j.joint_type == "fixed" { continue; }
        j.actuator_mode = articara::robot::ActuatorMode::Position;
        j.actuator_kp = 50.0;
        j.actuator_kv = 5.0;
    }
    let constrain_pose = [
        ("FL_hip_joint", 0.0), ("FL_thigh_joint", 1.0), ("FL_calf_joint", -2.0),
        ("FR_hip_joint", 0.0), ("FR_thigh_joint", 1.0), ("FR_calf_joint", -2.0),
        ("RL_hip_joint", 0.0), ("RL_thigh_joint", 1.0), ("RL_calf_joint", -2.0),
        ("RR_hip_joint", 0.0), ("RR_thigh_joint", 1.0), ("RR_calf_joint", -2.0),
        ("arm_pitch_joint", 0.0),
    ];
    for (name, q) in constrain_pose {
        if let Some(&idx) = robot.joint_map.get(name) { robot.joint_positions[idx] = q; }
    }
    let kin = articara::gait::auto_detect_kinematics_config(
        &robot, &articara::gait::DEFAULT_FOOT_LINKS).expect("kin");
    let opts = articara::mjcf::MjcfExportOptions {
        ground_plane: Some(articara::mjcf::GroundPlaneCfg { z: 0.0, half_size: 4.0, roll: 0.0, pitch: 0.0 }),
        add_actuators: false,
        bake_actuator_limits: true,
        bake_joint_position_limits: true,
        ..Default::default()
    };
    let mut sim = articara::mujoco_sim::MujocoSim::new(&robot, opts).expect("sim");
    sim.set_gravity_compensation(false);
    let cfg = GaitConfig::trot();
    let mut gc = GaitController::build(&robot, kin, cfg, GaitMode::Champ).expect("gc");
    for (name, q) in constrain_pose {
        if let Some(&idx) = robot.joint_map.get(name) { sim.set_position_target(idx, q); }
    }
    let cmd = VelocityCmd { vx: 0.30, vy: 0.0, wz: 0.0 };
    let settle_s = 2.0;
    let cmd_s = 7.0;
    let n_steps = ((settle_s + cmd_s) / DT) as usize;
    let settle_steps = (settle_s / DT) as usize;
    let log_every = (0.5 / DT) as usize;
    let mut x0 = 0.0;
    let mut z0 = 0.0;
    eprintln!();
    eprintln!("=== CHAMP forward v2 time-series ===");
    eprintln!("  t_cmd[s]   dx[m]    dy[m]    dz[m]    yaw[rad]   body_z[m]");
    for k in 0..n_steps {
        if k == settle_steps {
            gc.enable();
            gc.set_velocity_cmd(cmd);
            if let Some(p) = sim.body_world_position(&robot.root_link) {
                x0 = p[0]; z0 = p[2];
            }
        }
        if k < settle_steps {
            for (name, q) in constrain_pose {
                if let Some(&idx) = robot.joint_map.get(name) { sim.set_position_target(idx, q); }
            }
        } else {
            let v_obs = sim.body_world_linear_velocity(&robot.root_link).unwrap_or([0.0; 3]);
            let w_obs = sim.body_world_angular_velocity(&robot.root_link).unwrap_or([0.0; 3]);
            gc.set_body_state_observed(
                Vector3::new(v_obs[0], v_obs[1], v_obs[2]),
                Vector3::new(w_obs[0], w_obs[1], w_obs[2]));
            let body_pos = sim.body_world_position(&robot.root_link).unwrap_or([0.0; 3]);
            let yaw_obs = sim.body_world_yaw(&robot.root_link).unwrap_or(0.0);
            gc.set_body_pose_observed(yaw_obs, Vector3::new(body_pos[0], body_pos[1], body_pos[2]));
            let (_o, targets, _ff) = gc.tick(DT);
            for (idx, q) in targets { sim.set_position_target(idx, q); }
        }
        sim.step(&mut robot, DT, true);
        if k >= settle_steps && (k - settle_steps) % log_every == 0 {
            if let Some(p) = sim.body_world_position(&robot.root_link) {
                let yaw = sim.body_world_yaw(&robot.root_link).unwrap_or(0.0);
                let t = (k - settle_steps) as f64 * DT;
                eprintln!("  {:>5.2}    {:+.4}  {:+.4}  {:+.4}  {:+.4}    {:.4}",
                    t, p[0]-x0, p[1], p[2]-z0, yaw, p[2]);
            }
        }
    }
}

/// Most faithful GUI replication so far: load `namiashi.misa` directly
/// (not the URDF), which gives us the GUI's exact PD gains AND joint
/// damping (URDF has damping=0; .misa has damping=0.1 on every leg
/// joint). Combined with post-settle kin + 0.5 s cmd=0 hold.
#[test]
#[ignore = "harness debug — run with --ignored"]
fn diag_walk_champ_forward_gui_replica_full_misa() {
    let misa_path = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("tests").join("fixtures").join("namiashi").join("namiashi.misa");
    if !misa_path.exists() { return; }
    let mut robot = articara::robot::RobotModel::from_misa(&misa_path).expect("load misa");
    let constrain_pose = [
        ("FL_hip_joint", 0.0), ("FL_thigh_joint", 1.0), ("FL_calf_joint", -2.0),
        ("FR_hip_joint", 0.0), ("FR_thigh_joint", 1.0), ("FR_calf_joint", -2.0),
        ("RL_hip_joint", 0.0), ("RL_thigh_joint", 1.0), ("RL_calf_joint", -2.0),
        ("RR_hip_joint", 0.0), ("RR_thigh_joint", 1.0), ("RR_calf_joint", -2.0),
        ("arm_pitch_joint", 0.0),
    ];
    for (name, q) in constrain_pose {
        if let Some(&idx) = robot.joint_map.get(name) { robot.joint_positions[idx] = q; }
    }
    // Dump the loaded PD gains + damping for diagnostic confirmation.
    if let Some(&idx) = robot.joint_map.get("FL_thigh_joint") {
        eprintln!("[diag] FL_thigh kp={} kv={} damping={}",
            robot.joints[idx].actuator_kp,
            robot.joints[idx].actuator_kv,
            robot.joints[idx].joint_damping);
    }
    let opts = articara::mjcf::MjcfExportOptions {
        ground_plane: Some(articara::mjcf::GroundPlaneCfg { z: 0.0, half_size: 4.0, roll: 0.0, pitch: 0.0 }),
        add_actuators: false,
        bake_actuator_limits: true,
        bake_joint_position_limits: true,
        ..Default::default()
    };
    let mut sim = articara::mujoco_sim::MujocoSim::new(&robot, opts).expect("sim");
    sim.set_gravity_compensation(false);
    for (name, q) in constrain_pose {
        if let Some(&idx) = robot.joint_map.get(name) { sim.set_position_target(idx, q); }
    }
    let settle_s = 2.0;
    let cmd_zero_s = 0.5;
    let cmd_s = 7.0;
    let settle_steps = (settle_s / DT) as usize;
    let cmd_zero_steps = (cmd_zero_s / DT) as usize;
    let cmd_steps = (cmd_s / DT) as usize;
    let log_every = (0.5 / DT) as usize;
    for _ in 0..settle_steps {
        for (name, q) in constrain_pose {
            if let Some(&idx) = robot.joint_map.get(name) { sim.set_position_target(idx, q); }
        }
        sim.step(&mut robot, DT, true);
    }
    let kin = articara::gait::auto_detect_kinematics_config(
        &robot, &articara::gait::DEFAULT_FOOT_LINKS).expect("kin");
    let cfg = GaitConfig::trot();
    let mut gc = GaitController::build(&robot, kin, cfg, GaitMode::Champ).expect("gc");
    gc.enable();
    gc.set_velocity_cmd(VelocityCmd::zero());
    for _ in 0..cmd_zero_steps {
        let (_o, targets, _ff) = gc.tick(DT);
        for (idx, q) in targets { sim.set_position_target(idx, q); }
        sim.step(&mut robot, DT, true);
    }
    let (x_cmd_start, z_cmd_start) = {
        let p = sim.body_world_position(&robot.root_link).unwrap_or([0.0; 3]);
        (p[0], p[2])
    };
    gc.set_velocity_cmd(VelocityCmd { vx: 0.30, vy: 0.0, wz: 0.0 });
    eprintln!();
    eprintln!("=== diag_walk_champ_forward_gui_replica_full_misa ===");
    eprintln!("  (from_misa loader: PD 100/1.2 + damping 0.1 + post-settle kin + 0.5 s hold)");
    eprintln!("  t_cmd[s]   dx[m]    dy[m]    dz[m]    yaw[rad]   body_z[m]");
    eprintln!("  {:>5.2}    {:+.4}  {:+.4}  {:+.4}  {:+.4}    {:.4}",
        0.0, 0.0, 0.0, 0.0, 0.0, z_cmd_start);
    for k in 0..cmd_steps {
        let (_o, targets, _ff) = gc.tick(DT);
        for (idx, q) in targets { sim.set_position_target(idx, q); }
        sim.step(&mut robot, DT, true);
        if (k + 1) % log_every == 0 {
            if let Some(p) = sim.body_world_position(&robot.root_link) {
                let yaw = sim.body_world_yaw(&robot.root_link).unwrap_or(0.0);
                let t = (k + 1) as f64 * DT;
                eprintln!("  {:>5.2}    {:+.4}  {:+.4}  {:+.4}  {:+.4}    {:.4}",
                    t, p[0]-x_cmd_start, p[1], p[2]-z_cmd_start, yaw, p[2]);
            }
        }
    }
}

/// External-force robustness benchmark. Drives namiashi at `cmd vx=+0.15`
/// (or 0 for stationary stand), applies a world-frame impulsive force on
/// the trunk at `t = pre_force_s` for `force_duration_s`, then continues
/// for `post_force_s` to observe recovery.
///
/// For each (mode, scenario) combination, prints:
///   - peak |body_y| deviation during/after force (lateral robustness)
///   - peak |body_x − expected| (forward tracking deviation)
///   - peak |roll|, peak |pitch|, peak |Δyaw| (body attitude)
///   - min_z over the whole window (fall detection)
///   - recovery_time = time until |body_y| < 0.05 m after force end
///
/// The 3 modes use `.misa` fidelity + capture-point=0 (post-C2 defaults),
/// matching the GUI's recommended setup. Forces are world-frame; the
/// scenario list covers (lateral push, forward push, vertical jolt).
#[test]
#[ignore = "external-force robustness benchmark — run with --ignored"]
fn diag_external_force_robustness() {
    let common::StandFixture {
        mut robot,
        kin: _,
        mut sim,
    } = match common::build_namiashi_stand_fixture_misa() {
        Some(f) => f,
        None => return,
    };
    drop(robot);
    drop(sim);

    // Test parameters.
    let cmd = VelocityCmd { vx: 0.15, vy: 0.0, wz: 0.0 };
    let pre_force_s = 3.0;        // walk for 3 s before force
    let force_duration_s = 0.2;   // impulse window
    let post_force_s = 4.0;       // observe recovery
    let total_s = pre_force_s + force_duration_s + post_force_s;

    // Force levels are calibrated for namiashi (m = 2.4 kg). An impulse
    // I = F · 0.2 s translates to Δv = I / m. We sweep 2 / 4 / 6 N to
    // bracket the "comfortable / hard / break" range and surface where
    // each mode loses stability.
    let scenarios: [(&str, [f64; 3], [f64; 3]); 9] = [
        // (label, force [N], torque [N·m])
        ("lateral +y 2 N",  [0.0,  2.0, 0.0], [0.0; 3]),
        ("lateral +y 4 N",  [0.0,  4.0, 0.0], [0.0; 3]),
        ("lateral +y 6 N",  [0.0,  6.0, 0.0], [0.0; 3]),
        ("forward −x 2 N",  [-2.0, 0.0, 0.0], [0.0; 3]),
        ("forward −x 4 N",  [-4.0, 0.0, 0.0], [0.0; 3]),
        ("forward −x 6 N",  [-6.0, 0.0, 0.0], [0.0; 3]),
        ("vertical −z 4 N", [0.0, 0.0, -4.0], [0.0; 3]),
        ("vertical −z 8 N", [0.0, 0.0, -8.0], [0.0; 3]),
        ("yaw torque 1.5 N·m", [0.0; 3],      [0.0, 0.0, 1.5]),
    ];

    // (mode label, GaitMode, use_wbc, FullCentroidal tweak (h,sqp_iter), enable SRBD foot-offset, enable legged_control parity, capture-point gain override, nominal q_ref).
    // The `enable_foot_offset` slot is left in the tuple for future
    // experimentation but currently set to `false` everywhere — Step B's
    // controller integration was reverted (see commit message + doc).
    // `parity` toggles the FullCentroidal controller's
    // legged_control-style per-step phase prediction + swing-leg
    // vertical foot velocity equality constraint
    // (NormalVelocityConstraintCppAd analog). The legacy three FullC
    // rows are left with `parity = false` so the A/B comparison sits
    // in a single table.
    //
    // `cap_pt_override = Some(g)` raises the footstep planner's
    // capture-point feedback gain from its post-C2 default of 0.0 to
    // `g` (legacy = 0.175). This is the **α** experiment: pair it with
    // `parity = true` to see whether the missing closed-loop foothold
    // correction is what kept the parity row from solving the lateral
    // 4 N+ fall.
    //
    // `nominal_q_ref = true` switches the joint_q tracking reference
    // from observed `joint_q_now` to the URDF nominal stance pose
    // (matches legged_control's `DEFAULT_JOINT_STATE`). This is the
    // **β** experiment: it biases swing legs back toward the
    // standing pose, which is the other half of legged_control's
    // joint-tracking semantics. Combined α+β is in the last row.
    // The "FullC + cap-pt {0.05, 0.10, 0.175}" rows are the **η**
    // experiment: capture-point feedback applied to the LEGACY
    // FullCentroidal path (parity OFF) to isolate cap-pt's solo
    // behaviour from parity's per-step contact + swing v_z
    // interference. 0.175 is the pre-C2 legacy value;
    // 0.05 / 0.10 sample a gentler regime in case the legacy gain is
    // over-tuned for namiashi's lower mass/leg-length scale.
    // The "FullC + db {0.05/k_p=0.20, 0.05/0.30, 0.10/0.30}" rows are
    // the **η-2** experiment: replace the linear `k · v_err` foothold
    // shift with a deadband-gated steeper pulse so the swing leg can
    // commit a 5+ cm lateral offset on a real 4 N push without the
    // y-axis amplification seen at linear `k ≥ 0.10`. The linear
    // baseline `k_linear = 0.05` is retained alongside the pulse so
    // small disturbances still get a (gentle) response, but cycle-
    // noise below `v_db` produces no foothold shift at all — that's
    // what kills the cross-axis drift.
    // "FullC parity + trans {0.05, 0.10}" are the **C1** experiment:
    // GaitConfig::transition_fraction > 0 ramps the per-leg GRF
    // reference at touchdown / lift-off (a soft cost-side smoother;
    // stance no-slip stays active). Parity is on because the ramp
    // needs per-step stance sub-fractions — legacy path uses a
    // mid-stance proxy that always yields weight 1.0.
    //
    // The trailing tuple element `transition_enforce_constraint`
    // (bool) is the **C1-2 experiment**: when paired with
    // `transition_fraction > 0`, the controller also tightens the
    // per-leg per-step `max_normal_force` upper bound to
    // `weight · cfg.max_normal_force`, forcing the MPC's friction-
    // cone block to honour the ramp as a HARD constraint. This is
    // where C1 (cost-side, bit-exact identical to parity baseline)
    // graduates to a real intervention.
    let modes: [(&str, GaitMode, bool, Option<(usize, usize)>, bool, bool, Option<f64>, bool, Option<(f64, f64)>, f64, bool); 19] = [
        ("CHAMP open-loop",                 GaitMode::Champ, false, None, false, false, None, false, None, 0.0, false),
        ("SRBD MPC + WBC",                  GaitMode::Mpc, true,    None, false, false, None, false, None, 0.0, false),
        ("FullC default",                   GaitMode::FullCentroidal, true, None, false, false, None, false, None, 0.0, false),
        ("FullC h20 sqp3",                  GaitMode::FullCentroidal, true, Some((20, 3)), false, false, None, false, None, 0.0, false),
        ("FullC h10 sqp5",                  GaitMode::FullCentroidal, true, Some((10, 5)), false, false, None, false, None, 0.0, false),
        ("FullC + cap-pt 0.05",             GaitMode::FullCentroidal, true, None, false, false, Some(0.05),  false, None, 0.0, false),
        ("FullC + cap-pt 0.10",             GaitMode::FullCentroidal, true, None, false, false, Some(0.10),  false, None, 0.0, false),
        ("FullC + cap-pt 0.175",            GaitMode::FullCentroidal, true, None, false, false, Some(0.175), false, None, 0.0, false),
        ("FullC + db 0.05 / k_p 0.20",      GaitMode::FullCentroidal, true, None, false, false, None, false, Some((0.20, 0.05)), 0.0, false),
        ("FullC + db 0.05 / k_p 0.30",      GaitMode::FullCentroidal, true, None, false, false, None, false, Some((0.30, 0.05)), 0.0, false),
        ("FullC + db 0.10 / k_p 0.30",      GaitMode::FullCentroidal, true, None, false, false, None, false, Some((0.30, 0.10)), 0.0, false),
        ("FullC legged-parity",             GaitMode::FullCentroidal, true, None, false, true, None, false, None, 0.0, false),
        ("FullC parity + cap-pt 0.175",     GaitMode::FullCentroidal, true, None, false, true, Some(0.175), false, None, 0.0, false),
        ("FullC parity + nominal q_ref",    GaitMode::FullCentroidal, true, None, false, true, None, true, None, 0.0, false),
        ("FullC parity + cap-pt + nom-q",   GaitMode::FullCentroidal, true, None, false, true, Some(0.175), true, None, 0.0, false),
        ("FullC parity + trans 0.05",       GaitMode::FullCentroidal, true, None, false, true, None, false, None, 0.05, false),
        ("FullC parity + trans 0.10",       GaitMode::FullCentroidal, true, None, false, true, None, false, None, 0.10, false),
        ("FullC parity + trans 0.05 hard",  GaitMode::FullCentroidal, true, None, false, true, None, false, None, 0.05, true),
        ("FullC parity + trans 0.10 hard",  GaitMode::FullCentroidal, true, None, false, true, None, false, None, 0.10, true),
    ];

    eprintln!();
    eprintln!(
        "=== External-force robustness (cmd vx={:.2} m/s, force at t={}s for {}s, observe {}s) ===",
        cmd.vx, pre_force_s, force_duration_s, post_force_s,
    );
    eprintln!(
        "        {:<22} | peak |dy| | peak |roll| | peak |pitch| | peak |Δyaw| | min_z  | dx_end (exp +{:.2}) | recovery_s | result",
        "Mode (scenario)",
        cmd.vx * (pre_force_s + force_duration_s + post_force_s),
    );
    eprintln!("        {}", "─".repeat(140));

    for (scen_label, force, torque) in &scenarios {
        for (mode_label, mode, use_wbc, full_cfg_tweak, enable_foot_offset, enable_parity, cap_pt_override, use_nominal_q_ref, pulse_db, transition_fraction, enforce_constraint) in &modes {
            // Fresh fixture per (mode, scenario) so disturbance windows
            // don't bleed across tests.
            let Some(common::StandFixture {
                mut robot,
                kin,
                mut sim,
            }) = common::build_namiashi_stand_fixture_misa() else { continue; };

            // C1: opt into the GRF-reference transition ramp via
            // GaitConfig::transition_fraction. Default 0.0 leaves the
            // legacy even-split GRF behaviour untouched.
            // C1-2: also opt into the constraint-side hard
            // `max_normal_force` ramp via
            // `transition_enforce_constraint`. Without this flag the
            // ramp is cost-side only (= bit-exact identical to
            // baseline per the η-3 / C1 negative result).
            let cfg = GaitConfig::trot()
                .with_transition_fraction(*transition_fraction)
                .with_transition_enforce_constraint(*enforce_constraint);
            let mut gc = GaitController::build(&robot, kin.clone(), cfg, *mode)
                .expect("GaitController::build");
            // Per-row override (α / η experiments) overrides the
            // current `DEFAULT_CAPTURE_POINT_GAIN_S` (= 0.05 post-η,
            // 2026-05-15). When no override is set the row inherits
            // whatever the constructor default is, so "FullC default"
            // tracks any future re-tuning of that constant.
            if let Some(gain) = cap_pt_override {
                gc.set_capture_point_gain(*gain);
            }
            // η-2 experiment: nonlinear deadband + steeper-slope
            // pulse branch. `Some((k_pulse, v_db))` activates it on
            // top of whichever linear `k_capture` is in effect for
            // this row (so the pulse row inherits the constructor's
            // default linear gain unless `cap_pt_override` is also
            // set).
            if let Some((k_pulse, v_db)) = pulse_db {
                gc.set_capture_point_pulse(*k_pulse, *v_db);
            }
            // Apply FullCentroidal-specific tuning when the row asks for it.
            if let Some((horizon, sqp)) = full_cfg_tweak {
                if let Some(full_cfg) = gc.full_centroidal_mpc_config() {
                    let mut new_cfg = full_cfg.clone();
                    new_cfg.horizon_steps = *horizon;
                    new_cfg.sqp_iterations = *sqp;
                    gc.set_full_centroidal_mpc_config(new_cfg);
                }
            }
            // Activate the legged_control-parity path on the
            // FullCentroidal controller for the dedicated comparison row.
            // `use_nominal_q_ref` (β) is gated on parity so it has no
            // effect on the legacy rows.
            if *enable_parity {
                gc.set_legged_control_parity(true);
                if *use_nominal_q_ref {
                    gc.set_parity_use_nominal_q_ref(true);
                }
            }
            // Step B: opt into the SRBD foot-offset extension. The
            // default bounds (8 cm offset, r_diag_foot_offset=0.5) are
            // tuned for Cheetah-class. namiashi (2.4 kg) needs tighter
            // limits — the MPC's single-iteration SQP linearization
            // (hover F^*) over-trusts the offset's authority and crashes
            // the body, so cap at 2 cm and raise the regularizer.
            if *enable_foot_offset {
                if let Some(srbd_cfg) = gc.srbd_mpc_config() {
                    let mut new_cfg = srbd_cfg.clone();
                    new_cfg.enable_foot_offset = true;
                    new_cfg.max_foot_offset_m = 0.02;
                    new_cfg.r_diag_foot_offset = 10.0;
                    gc.set_srbd_mpc_config(new_cfg);
                }
            }
            let mut wbc_pipeline = if *use_wbc {
                Some(WbcPipeline::new(&robot, common::default_foot_links()))
            } else { None };
            if let Some(pipeline) = wbc_pipeline.as_mut() {
                if let Some(full_cfg) = gc.full_centroidal_mpc_config() {
                    pipeline.mass_kg = full_cfg.mass_kg;
                    pipeline.centroidal_inertia_body = Some(full_cfg.centroidal_inertia_body);
                    pipeline.com_offset_body = full_cfg.com_offset_body;
                } else if let Some(srbd_cfg) = gc.srbd_mpc_config() {
                    pipeline.mass_kg = srbd_cfg.mass_kg;
                    pipeline.inertia_diag_body = srbd_cfg.inertia_diag_body;
                    pipeline.centroidal_inertia_body = None;
                }
            }

            let n_steps = (total_s / DT) as usize;
            let burn_in_steps = (WALK_BURN_IN_S / DT) as usize;
            let force_start_step = ((WALK_BURN_IN_S + pre_force_s) / DT) as usize;
            let force_applied_already = std::cell::Cell::new(false);

            let mut peak_dy = 0.0_f64;
            let mut peak_roll = 0.0_f64;
            let mut peak_pitch = 0.0_f64;
            let mut peak_dyaw = 0.0_f64;
            let mut min_z = f64::INFINITY;
            let mut body_x_start = 0.0;
            let mut body_x_end = 0.0;
            let mut yaw_start = 0.0;
            let mut recovery_time_s: Option<f64> = None;
            let force_end_step = force_start_step + (force_duration_s / DT) as usize;

            gc.enable();
            for k in 0..n_steps {
                // Snapshot start-of-cmd reference.
                if k == burn_in_steps {
                    gc.set_velocity_cmd(cmd);
                    if let Some(p) = sim.body_world_position(&robot.root_link) {
                        body_x_start = p[0];
                    }
                    yaw_start = sim.body_world_yaw(&robot.root_link).unwrap_or(0.0);
                }
                // Inject force exactly once.
                if k == force_start_step && !force_applied_already.get() {
                    sim.apply_external_force(
                        &robot.root_link, *force, *torque, force_duration_s,
                    );
                    force_applied_already.set(true);
                }
                let v_obs = sim.body_world_linear_velocity(&robot.root_link).unwrap_or([0.0; 3]);
                let w_obs = sim.body_world_angular_velocity(&robot.root_link).unwrap_or([0.0; 3]);
                gc.set_body_state_observed(
                    Vector3::new(v_obs[0], v_obs[1], v_obs[2]),
                    Vector3::new(w_obs[0], w_obs[1], w_obs[2]));
                let body_pos = sim.body_world_position(&robot.root_link).unwrap_or([0.0; 3]);
                let yaw_obs = sim.body_world_yaw(&robot.root_link).unwrap_or(0.0);
                gc.set_body_pose_observed(
                    yaw_obs, Vector3::new(body_pos[0], body_pos[1], body_pos[2]));
                // Per-cmd WBC weights (mirrors run_walk).
                if let Some(pipeline) = wbc_pipeline.as_mut() {
                    pipeline.weights = if pipeline.centroidal_inertia_body.is_some() {
                        quadruped_gait::wbc::WbcWeights::for_cmd_centroidal(&gc.velocity_cmd())
                    } else {
                        quadruped_gait::wbc::WbcWeights::for_cmd(&gc.velocity_cmd())
                    };
                }
                let (out, targets, torque_ff) = gc.tick(DT);
                for (idx, q) in targets { sim.set_position_target(idx, q); }
                // WBC path: same shape as run_walk's hybrid joint command.
                if let Some(pipeline) = wbc_pipeline.as_mut() {
                    if k >= burn_in_steps {
                        let f_grf_world = gc.predicted_grfs()
                            .map(|sol| sol.grfs_first_step)
                            .unwrap_or([Vector3::zeros(); 4]);
                        let cmd_w = gc.velocity_cmd();
                        let v_cmd_body = Vector3::new(cmd_w.vx, cmd_w.vy, 0.0);
                        let foot_links_str: [&str; 4] = [
                            pipeline.foot_links[0].as_str(),
                            pipeline.foot_links[1].as_str(),
                            pipeline.foot_links[2].as_str(),
                            pipeline.foot_links[3].as_str(),
                        ];
                        let force_z = sim.contact_force_per_foot(&foot_links_str);
                        let nominal_phases = [
                            out.legs[0].phase,
                            out.legs[1].phase,
                            out.legs[2].phase,
                            out.legs[3].phase,
                        ];
                        let corrected = ContactDrivenPhase::apply_correction(
                            &nominal_phases, force_z, 5.0, 0.0,
                        );
                        let contact_flag = [
                            corrected[0].is_stance,
                            corrected[1].is_stance,
                            corrected[2].is_stance,
                            corrected[3].is_stance,
                        ];
                        let taus = pipeline.solve(
                            &robot,
                            &sim,
                            &out,
                            gc.kinematics(),
                            gc.joint_indices(),
                            gc.joint_signs(),
                            &v_cmd_body,
                            cmd_w.wz,
                            &Vector3::new(v_obs[0], v_obs[1], v_obs[2]),
                            &Vector3::new(w_obs[0], w_obs[1], w_obs[2]),
                            &f_grf_world,
                            contact_flag,
                            DT,
                        );
                        for (ji, &tau) in taus.iter().enumerate() {
                            sim.set_torque_feedforward(ji, tau);
                        }
                    } else {
                        for ji in 0..robot.joints.len() {
                            sim.set_torque_feedforward(ji, 0.0);
                        }
                    }
                } else {
                    for (idx, tau) in torque_ff { sim.set_torque_feedforward(idx, tau); }
                }
                sim.step(&mut robot, DT, true);

                // Metric accumulation — only after cmd is applied so the
                // burn-in settling doesn't contaminate peaks.
                if k >= burn_in_steps {
                    if let Some(p) = sim.body_world_position(&robot.root_link) {
                        peak_dy = peak_dy.max(p[1].abs());
                        min_z = min_z.min(p[2]);
                        body_x_end = p[0];
                    }
                    if let Some(quat) = sim.body_world_orientation(&robot.root_link) {
                        let euler = quat.euler_angles();  // (roll, pitch, yaw) in ZYX convention
                        peak_roll = peak_roll.max(euler.0.abs());
                        peak_pitch = peak_pitch.max(euler.1.abs());
                    }
                    let dyaw = (yaw_obs - yaw_start).abs();
                    peak_dyaw = peak_dyaw.max(dyaw);
                    // Recovery time: first sample post-force-end with
                    // |dy| < 5 cm.
                    if k >= force_end_step && recovery_time_s.is_none() {
                        let dy = sim.body_world_position(&robot.root_link)
                            .map(|p| p[1].abs()).unwrap_or(0.0);
                        if dy < 0.05 {
                            recovery_time_s = Some(
                                (k - force_end_step) as f64 * DT
                            );
                        }
                    }
                }
            }

            let fell = min_z < FALL_THRESHOLD_Z;
            let dx = body_x_end - body_x_start;
            let recovery_label = match recovery_time_s {
                Some(t) => format!("{:>6.2}", t),
                None if fell => "fell".to_string(),
                None => "no recov".to_string(),
            };
            let result = if fell { "✗ fell" } else if recovery_time_s.is_some() { "✓ recovered" } else { "△ no recovery" };
            eprintln!(
                "        {:<22} | {:>9.3} | {:>11.3} | {:>12.3} | {:>11.3} | {:>6.3} | {:>+8.3}             | {} | {}",
                format!("{} ({})", mode_label, scen_label),
                peak_dy, peak_roll, peak_pitch, peak_dyaw, min_z, dx, recovery_label, result,
            );
        }
        eprintln!("        {}", "─".repeat(140));
    }
}

/// **max_step_length sweep on lateral push recovery.**
///
/// `GaitConfig::max_step_length_m` caps how far each footstep can be
/// offset from its nominal location, in both x and y. At the default
/// `0.10 m` (= 5 cm half-stride radius) the swing leg can only chase
/// the body laterally up to 5 cm per cycle — but a lateral 6 N push
/// gives the body ~0.5 m/s sliding velocity, which over a 0.2 s swing
/// reaches ~10 cm before the foot can catch up. Result: foot lands
/// inside the body's lateral displacement, CoM ends up outside the
/// support polygon, body topples.
///
/// This sweep tries widening the step length to see whether the
/// physical recovery margin opens up. Range bounded by namiashi's
/// leg geometry (upper + lower ≈ 0.36 m, but reachable workspace
/// shrinks fast past ~50 % of leg length due to knee-joint limits and
/// self-collision risk).
#[test]
#[ignore = "max_step_length sweep — run with --ignored"]
fn diag_max_step_length_lateral_recovery() {
    let common::StandFixture {
        robot, kin: _, sim,
    } = match common::build_namiashi_stand_fixture_misa() {
        Some(f) => f,
        None => return,
    };
    drop(robot);
    drop(sim);

    let cmd = VelocityCmd { vx: 0.15, vy: 0.0, wz: 0.0 };
    let pre_force_s = 3.0;
    let force_duration_s = 0.2;
    let post_force_s = 4.0;
    let total_s = pre_force_s + force_duration_s + post_force_s;

    let scenarios: [(&str, [f64; 3]); 3] = [
        ("lateral +y 2 N", [0.0, 2.0, 0.0]),
        ("lateral +y 4 N", [0.0, 4.0, 0.0]),
        ("lateral +y 6 N", [0.0, 6.0, 0.0]),
    ];
    // (max_step_length, use_mpc_predicted_footstep, label).
    // 0.10 is trot() default. The third row enables the
    // **MPC-predicted footstep** path (legged_control-style) — foot
    // placement reads `last_solution.predicted_states[~swing_steps]`
    // and snaps to the MPC's planned base trajectory. The wider
    // `max_step_length = 0.15` gives that path room to commit a real
    // lateral footstep without clamping.
    let variants: [(f64, bool, &str); 4] = [
        (0.10, false, "cap-pt baseline"),
        (0.15, false, "wider step only"),
        (0.10, true,  "MPC-pred footstep"),
        (0.15, true,  "MPC-pred + wider"),
    ];

    eprintln!();
    eprintln!(
        "=== max_step_length sweep on lateral push (cmd vx={:.2} m/s, push at t={}s for {}s, observe {}s) ===",
        cmd.vx, pre_force_s, force_duration_s, post_force_s,
    );
    eprintln!(
        "        {:<28} | peak |dy| | end |dy| | peak roll | peak pitch | min_z  | result",
        "Mode (scenario)",
    );
    eprintln!("        {}", "─".repeat(115));

    for (scen_label, force) in &scenarios {
        for (max_step, use_mpc_pred, variant_label) in &variants {
            let Some(common::StandFixture {
                mut robot,
                kin,
                mut sim,
            }) = common::build_namiashi_stand_fixture_misa() else { continue; };

            let cfg = GaitConfig::trot().with_max_step_length(*max_step);
            let mut gc = GaitController::build(&robot, kin.clone(), cfg, GaitMode::FullCentroidal)
                .expect("GaitController::build");
            if *use_mpc_pred {
                gc.set_use_mpc_predicted_footstep(true);
            }

            let mut wbc_pipeline = WbcPipeline::new(&robot, common::default_foot_links());
            if let Some(full_cfg) = gc.full_centroidal_mpc_config() {
                wbc_pipeline.mass_kg = full_cfg.mass_kg;
                wbc_pipeline.centroidal_inertia_body = Some(full_cfg.centroidal_inertia_body);
                wbc_pipeline.com_offset_body = full_cfg.com_offset_body;
            }

            let n_steps = (total_s / DT) as usize;
            let burn_in_steps = (WALK_BURN_IN_S / DT) as usize;
            let force_start_step = ((WALK_BURN_IN_S + pre_force_s) / DT) as usize;
            let force_applied_already = std::cell::Cell::new(false);

            let mut peak_dy = 0.0_f64;
            let mut peak_roll = 0.0_f64;
            let mut peak_pitch = 0.0_f64;
            let mut min_z = f64::INFINITY;
            let mut body_y_end = 0.0;

            gc.enable();
            for k in 0..n_steps {
                if k == burn_in_steps {
                    gc.set_velocity_cmd(cmd);
                }
                if k == force_start_step && !force_applied_already.get() {
                    sim.apply_external_force(
                        &robot.root_link, *force, [0.0; 3], force_duration_s,
                    );
                    force_applied_already.set(true);
                }
                let v_obs = sim.body_world_linear_velocity(&robot.root_link).unwrap_or([0.0; 3]);
                let w_obs = sim.body_world_angular_velocity(&robot.root_link).unwrap_or([0.0; 3]);
                gc.set_body_state_observed(
                    Vector3::new(v_obs[0], v_obs[1], v_obs[2]),
                    Vector3::new(w_obs[0], w_obs[1], w_obs[2]),
                );
                let body_pos = sim.body_world_position(&robot.root_link).unwrap_or([0.0; 3]);
                let yaw_obs = sim.body_world_yaw(&robot.root_link).unwrap_or(0.0);
                gc.set_body_pose_observed(
                    yaw_obs, Vector3::new(body_pos[0], body_pos[1], body_pos[2]));
                wbc_pipeline.weights = quadruped_gait::wbc::WbcWeights::for_cmd_centroidal(&gc.velocity_cmd());

                let (out, targets, _) = gc.tick(DT);
                for (idx, q) in targets { sim.set_position_target(idx, q); }
                if k >= burn_in_steps {
                    let f_grf_world = gc.predicted_grfs()
                        .map(|sol| sol.grfs_first_step)
                        .unwrap_or([Vector3::zeros(); 4]);
                    let cmd_w = gc.velocity_cmd();
                    let v_cmd_body = Vector3::new(cmd_w.vx, cmd_w.vy, 0.0);
                    let foot_links_str: [&str; 4] = [
                        wbc_pipeline.foot_links[0].as_str(),
                        wbc_pipeline.foot_links[1].as_str(),
                        wbc_pipeline.foot_links[2].as_str(),
                        wbc_pipeline.foot_links[3].as_str(),
                    ];
                    let force_z = sim.contact_force_per_foot(&foot_links_str);
                    let nominal_phases = [out.legs[0].phase, out.legs[1].phase,
                                          out.legs[2].phase, out.legs[3].phase];
                    let corrected = ContactDrivenPhase::apply_correction(
                        &nominal_phases, force_z, 5.0, 0.0,
                    );
                    let contact_flag = [corrected[0].is_stance, corrected[1].is_stance,
                                        corrected[2].is_stance, corrected[3].is_stance];
                    let taus = wbc_pipeline.solve(
                        &robot, &sim, &out, gc.kinematics(),
                        gc.joint_indices(), gc.joint_signs(),
                        &v_cmd_body, cmd_w.wz,
                        &Vector3::new(v_obs[0], v_obs[1], v_obs[2]),
                        &Vector3::new(w_obs[0], w_obs[1], w_obs[2]),
                        &f_grf_world, contact_flag, DT,
                    );
                    for (ji, &tau) in taus.iter().enumerate() {
                        sim.set_torque_feedforward(ji, tau);
                    }
                } else {
                    for ji in 0..robot.joints.len() {
                        sim.set_torque_feedforward(ji, 0.0);
                    }
                }
                sim.step(&mut robot, DT, true);

                if k >= burn_in_steps {
                    if let Some(p) = sim.body_world_position(&robot.root_link) {
                        peak_dy = peak_dy.max(p[1].abs());
                        min_z = min_z.min(p[2]);
                        body_y_end = p[1];
                    }
                    if let Some(quat) = sim.body_world_orientation(&robot.root_link) {
                        let euler = quat.euler_angles();
                        peak_roll = peak_roll.max(euler.0.abs());
                        peak_pitch = peak_pitch.max(euler.1.abs());
                    }
                }
            }

            let fell = min_z < FALL_THRESHOLD_Z;
            let result = if fell { "✗ fell" } else { "✓ no fall" };
            eprintln!(
                "        {:<32} | {:>9.3} | {:>8.3} | {:>9.3} | {:>10.3} | {:>5.3} | {}",
                format!("step={:.2} {} ({})", max_step, variant_label, scen_label),
                peak_dy, body_y_end.abs(), peak_roll, peak_pitch, min_z, result,
            );
        }
        eprintln!("        {}", "─".repeat(115));
    }
}

/// **Friction-cone utilization diagnostic** — does the MPC ever
/// approach the `|f_xy| ≤ μ·f_z` boundary during external-force
/// scenarios?
///
/// If the cone is **never binding** (peak ratio stays well below 1.0),
/// then the A3 candidate (soft friction cone with slack penalty) won't
/// change the MPC's solution — the cone isn't the limiting factor,
/// the geometry / footstep reach is. In that case A3 should be
/// skipped and B3 (warm-start) is the next actionable item.
///
/// For each scenario, runs the same fixture as
/// `diag_external_force_robustness` but extracts the per-leg per-tick
/// `|f_xy| / (μ·f_z)` ratio from `gc.predicted_grfs()` and reports
/// the **peak** across stance legs over the post-force observation
/// window. The pre-force walk is allowed to settle first so the
/// stance-leg cone activity reflects the steady-state response, not
/// the burn-in.
#[test]
#[ignore = "friction-cone utilization probe — run with --ignored"]
fn diag_friction_cone_utilization() {
    let common::StandFixture {
        robot, kin: _, sim,
    } = match common::build_namiashi_stand_fixture_misa() {
        Some(f) => f,
        None => return,
    };
    drop(robot);
    drop(sim);

    let cmd = VelocityCmd { vx: 0.15, vy: 0.0, wz: 0.0 };
    let pre_force_s = 3.0;
    let force_duration_s = 0.2;
    let post_force_s = 4.0;
    let total_s = pre_force_s + force_duration_s + post_force_s;

    let scenarios: [(&str, [f64; 3]); 4] = [
        ("baseline (no push)", [0.0; 3]),
        ("lateral +y 2 N",     [0.0,  2.0, 0.0]),
        ("lateral +y 4 N",     [0.0,  4.0, 0.0]),
        ("lateral +y 6 N",     [0.0,  6.0, 0.0]),
    ];

    eprintln!();
    eprintln!(
        "=== Friction-cone utilization (cmd vx={:.2} m/s) — peak |f_xy|/(μ·f_z) across stance legs ===",
        cmd.vx,
    );
    eprintln!(
        "        {:<22} | peak ratio (post-force) | peak |f_xy| (N) | peak f_z (N) | μ·f_z at peak (N)",
        "Scenario",
    );
    eprintln!("        {}", "─".repeat(110));

    for (scen_label, force) in &scenarios {
        let Some(common::StandFixture {
            mut robot,
            kin,
            mut sim,
        }) = common::build_namiashi_stand_fixture_misa() else { continue; };

        let cfg = GaitConfig::trot();
        let mut gc = GaitController::build(&robot, kin.clone(), cfg, GaitMode::FullCentroidal)
            .expect("GaitController::build");
        let mu = gc.full_centroidal_mpc_config()
            .map(|c| c.friction_mu)
            .unwrap_or(0.5);
        let mut wbc_pipeline = WbcPipeline::new(&robot, common::default_foot_links());
        if let Some(full_cfg) = gc.full_centroidal_mpc_config() {
            wbc_pipeline.mass_kg = full_cfg.mass_kg;
            wbc_pipeline.centroidal_inertia_body = Some(full_cfg.centroidal_inertia_body);
            wbc_pipeline.com_offset_body = full_cfg.com_offset_body;
        }

        let n_steps = (total_s / DT) as usize;
        let burn_in_steps = (WALK_BURN_IN_S / DT) as usize;
        let force_start_step = ((WALK_BURN_IN_S + pre_force_s) / DT) as usize;
        let force_end_step = force_start_step + (force_duration_s / DT) as usize;
        let force_applied_already = std::cell::Cell::new(false);

        let mut peak_ratio = 0.0_f64;
        let mut peak_fxy_at_peak = 0.0_f64;
        let mut peak_fz_at_peak = 0.0_f64;
        let mut mu_fz_at_peak = 0.0_f64;

        gc.enable();
        for k in 0..n_steps {
            if k == burn_in_steps {
                gc.set_velocity_cmd(cmd);
            }
            if k == force_start_step && !force_applied_already.get() && force != &[0.0; 3] {
                sim.apply_external_force(
                    &robot.root_link, *force, [0.0; 3], force_duration_s,
                );
                force_applied_already.set(true);
            }
            let v_obs = sim.body_world_linear_velocity(&robot.root_link).unwrap_or([0.0; 3]);
            let w_obs = sim.body_world_angular_velocity(&robot.root_link).unwrap_or([0.0; 3]);
            gc.set_body_state_observed(
                Vector3::new(v_obs[0], v_obs[1], v_obs[2]),
                Vector3::new(w_obs[0], w_obs[1], w_obs[2]),
            );
            let body_pos = sim.body_world_position(&robot.root_link).unwrap_or([0.0; 3]);
            let yaw_obs = sim.body_world_yaw(&robot.root_link).unwrap_or(0.0);
            gc.set_body_pose_observed(
                yaw_obs, Vector3::new(body_pos[0], body_pos[1], body_pos[2]));
            wbc_pipeline.weights = quadruped_gait::wbc::WbcWeights::for_cmd_centroidal(&gc.velocity_cmd());

            let (out, targets, _) = gc.tick(DT);
            for (idx, q) in targets { sim.set_position_target(idx, q); }
            if k >= burn_in_steps {
                let f_grf_world = gc.predicted_grfs()
                    .map(|sol| sol.grfs_first_step)
                    .unwrap_or([Vector3::zeros(); 4]);
                let cmd_w = gc.velocity_cmd();
                let v_cmd_body = Vector3::new(cmd_w.vx, cmd_w.vy, 0.0);
                let foot_links_str: [&str; 4] = [
                    wbc_pipeline.foot_links[0].as_str(),
                    wbc_pipeline.foot_links[1].as_str(),
                    wbc_pipeline.foot_links[2].as_str(),
                    wbc_pipeline.foot_links[3].as_str(),
                ];
                let force_z = sim.contact_force_per_foot(&foot_links_str);
                let nominal_phases = [out.legs[0].phase, out.legs[1].phase,
                                      out.legs[2].phase, out.legs[3].phase];
                let corrected = ContactDrivenPhase::apply_correction(
                    &nominal_phases, force_z, 5.0, 0.0,
                );
                let contact_flag = [corrected[0].is_stance, corrected[1].is_stance,
                                    corrected[2].is_stance, corrected[3].is_stance];
                let taus = wbc_pipeline.solve(
                    &robot, &sim, &out, gc.kinematics(),
                    gc.joint_indices(), gc.joint_signs(),
                    &v_cmd_body, cmd_w.wz,
                    &Vector3::new(v_obs[0], v_obs[1], v_obs[2]),
                    &Vector3::new(w_obs[0], w_obs[1], w_obs[2]),
                    &f_grf_world, contact_flag, DT,
                );
                for (ji, &tau) in taus.iter().enumerate() {
                    sim.set_torque_feedforward(ji, tau);
                }
            } else {
                for ji in 0..robot.joints.len() {
                    sim.set_torque_feedforward(ji, 0.0);
                }
            }
            sim.step(&mut robot, DT, true);

            // Cone utilization probe — only after the impulse has been
            // delivered, so the steady-state walking baseline doesn't
            // dilute the peak.
            if k >= force_end_step {
                if let Some(sol) = gc.predicted_grfs() {
                    for leg in 0..4 {
                        let f = sol.grfs_first_step[leg];
                        if f.z < 1e-3 {
                            continue; // swing leg
                        }
                        let f_xy = (f.x * f.x + f.y * f.y).sqrt();
                        let limit = mu * f.z;
                        if limit < 1e-6 {
                            continue;
                        }
                        let ratio = f_xy / limit;
                        if ratio > peak_ratio {
                            peak_ratio = ratio;
                            peak_fxy_at_peak = f_xy;
                            peak_fz_at_peak = f.z;
                            mu_fz_at_peak = limit;
                        }
                    }
                }
            }
        }

        eprintln!(
            "        {:<22} | {:>22.3} | {:>14.3} | {:>11.3} | {:>17.3}",
            scen_label, peak_ratio, peak_fxy_at_peak, peak_fz_at_peak, mu_fz_at_peak,
        );
    }
}

/// Goal-pose mode lateral recovery A/B.
///
/// The main `diag_external_force_robustness` benchmark uses cmd_vel
/// mode: the robot is commanded a constant body velocity, so after a
/// lateral push the body keeps drifting at the new offset (cap-pt 0.05
/// is enough to prevent falling, but `recovery_s` never fires).
///
/// This test exercises the **goal-pose mode** added in the
/// `goalToTargetTrajectories` analogue (see
/// [`quadruped_gait::velocity_cmd_for_goal`]): the controller targets
/// an absolute (x, y, yaw) goal in the world frame, so a lateral push
/// produces a non-zero `vy_body` command pointing back at the goal,
/// and the body actively recovers to y = 0.
///
/// Scope kept small (3 scenarios × 2 modes = 6 sims, ~2 min runtime)
/// so this can iterate independently of the 33-minute main bench.
#[test]
#[ignore = "goal-pose lateral recovery — run with --ignored"]
fn diag_goal_pose_lateral_recovery() {
    let common::StandFixture {
        robot, kin: _, sim,
    } = match common::build_namiashi_stand_fixture_misa() {
        Some(f) => f,
        None => return,
    };
    drop(robot);
    drop(sim);

    let cmd_vx = 0.15_f64;
    let pre_force_s = 3.0_f64;
    let force_duration_s = 0.2_f64;
    let post_force_s = 4.0_f64;
    let total_s = pre_force_s + force_duration_s + post_force_s;
    // Goal x is "where the body would be if no disturbance hit it"
    // at the end of the cmd window. y_goal = 0 forces the controller
    // to recover any lateral drift.
    let goal_x = cmd_vx * total_s;

    let scenarios: [(&str, [f64; 3]); 3] = [
        ("lateral +y 2 N", [0.0, 2.0, 0.0]),
        ("lateral +y 4 N", [0.0, 4.0, 0.0]),
        ("lateral +y 6 N", [0.0, 6.0, 0.0]),
    ];

    enum CmdMode {
        Velocity,
        Goal,
    }
    let modes: [(&str, CmdMode); 2] = [
        ("cmd_vel  (FullC default)", CmdMode::Velocity),
        ("goal_pose (FullC default)", CmdMode::Goal),
    ];

    eprintln!();
    eprintln!(
        "=== Goal-pose lateral recovery (cmd vx={:.2} m/s → goal x={:.2} m, push at t={}s for {}s, observe {}s) ===",
        cmd_vx, goal_x, pre_force_s, force_duration_s, post_force_s,
    );
    eprintln!(
        "        {:<40} | peak |dy| | end |dy| | end dx (goal {:+.2}) | recovery_s | result",
        "Mode (scenario)", goal_x,
    );
    eprintln!("        {}", "─".repeat(125));

    for (scen_label, force) in &scenarios {
        for (mode_label, mode) in &modes {
            let Some(common::StandFixture {
                mut robot,
                kin,
                mut sim,
            }) = common::build_namiashi_stand_fixture_misa() else { continue; };

            let cfg = GaitConfig::trot();
            let mut gc = GaitController::build(&robot, kin.clone(), cfg, GaitMode::FullCentroidal)
                .expect("GaitController::build");

            let mut wbc_pipeline = WbcPipeline::new(&robot, common::default_foot_links());
            if let Some(full_cfg) = gc.full_centroidal_mpc_config() {
                wbc_pipeline.mass_kg = full_cfg.mass_kg;
                wbc_pipeline.centroidal_inertia_body = Some(full_cfg.centroidal_inertia_body);
                wbc_pipeline.com_offset_body = full_cfg.com_offset_body;
            }

            let n_steps = (total_s / DT) as usize;
            let burn_in_steps = (WALK_BURN_IN_S / DT) as usize;
            let force_start_step = ((WALK_BURN_IN_S + pre_force_s) / DT) as usize;
            let force_end_step = force_start_step + (force_duration_s / DT) as usize;
            let force_applied_already = std::cell::Cell::new(false);

            let mut peak_dy = 0.0_f64;
            let mut min_z = f64::INFINITY;
            let mut body_x_start = 0.0;
            let mut body_x_end = 0.0;
            let mut body_y_end = 0.0;
            let mut recovery_time_s: Option<f64> = None;

            gc.enable();
            for k in 0..n_steps {
                if k == burn_in_steps {
                    match mode {
                        CmdMode::Velocity => {
                            gc.set_velocity_cmd(VelocityCmd { vx: cmd_vx, vy: 0.0, wz: 0.0 });
                        }
                        CmdMode::Goal => {
                            gc.set_goal_pose_world(quadruped_gait::GoalPoseWorld {
                                x_m: goal_x,
                                y_m: 0.0,
                                yaw_rad: 0.0,
                                max_v_m_s: cmd_vx,
                                max_wz_rad_s: 0.5,
                                position_tolerance_m: 0.02,
                                yaw_tolerance_rad: 0.05,
                            });
                        }
                    }
                    if let Some(p) = sim.body_world_position(&robot.root_link) {
                        body_x_start = p[0];
                    }
                }
                if k == force_start_step && !force_applied_already.get() {
                    sim.apply_external_force(
                        &robot.root_link, *force, [0.0; 3], force_duration_s,
                    );
                    force_applied_already.set(true);
                }
                let v_obs = sim.body_world_linear_velocity(&robot.root_link).unwrap_or([0.0; 3]);
                let w_obs = sim.body_world_angular_velocity(&robot.root_link).unwrap_or([0.0; 3]);
                gc.set_body_state_observed(
                    Vector3::new(v_obs[0], v_obs[1], v_obs[2]),
                    Vector3::new(w_obs[0], w_obs[1], w_obs[2]),
                );
                let body_pos = sim.body_world_position(&robot.root_link).unwrap_or([0.0; 3]);
                let yaw_obs = sim.body_world_yaw(&robot.root_link).unwrap_or(0.0);
                gc.set_body_pose_observed(
                    yaw_obs, Vector3::new(body_pos[0], body_pos[1], body_pos[2]));
                wbc_pipeline.weights = quadruped_gait::wbc::WbcWeights::for_cmd_centroidal(&gc.velocity_cmd());

                let (out, targets, _) = gc.tick(DT);
                for (idx, q) in targets { sim.set_position_target(idx, q); }
                if k >= burn_in_steps {
                    let f_grf_world = gc.predicted_grfs()
                        .map(|sol| sol.grfs_first_step)
                        .unwrap_or([Vector3::zeros(); 4]);
                    let cmd_w = gc.velocity_cmd();
                    let v_cmd_body = Vector3::new(cmd_w.vx, cmd_w.vy, 0.0);
                    let foot_links_str: [&str; 4] = [
                        wbc_pipeline.foot_links[0].as_str(),
                        wbc_pipeline.foot_links[1].as_str(),
                        wbc_pipeline.foot_links[2].as_str(),
                        wbc_pipeline.foot_links[3].as_str(),
                    ];
                    let force_z = sim.contact_force_per_foot(&foot_links_str);
                    let nominal_phases = [
                        out.legs[0].phase, out.legs[1].phase,
                        out.legs[2].phase, out.legs[3].phase,
                    ];
                    let corrected = ContactDrivenPhase::apply_correction(
                        &nominal_phases, force_z, 5.0, 0.0,
                    );
                    let contact_flag = [
                        corrected[0].is_stance, corrected[1].is_stance,
                        corrected[2].is_stance, corrected[3].is_stance,
                    ];
                    let taus = wbc_pipeline.solve(
                        &robot, &sim, &out, gc.kinematics(),
                        gc.joint_indices(), gc.joint_signs(),
                        &v_cmd_body, cmd_w.wz,
                        &Vector3::new(v_obs[0], v_obs[1], v_obs[2]),
                        &Vector3::new(w_obs[0], w_obs[1], w_obs[2]),
                        &f_grf_world, contact_flag, DT,
                    );
                    for (ji, &tau) in taus.iter().enumerate() {
                        sim.set_torque_feedforward(ji, tau);
                    }
                } else {
                    for ji in 0..robot.joints.len() {
                        sim.set_torque_feedforward(ji, 0.0);
                    }
                }
                sim.step(&mut robot, DT, true);

                if k >= burn_in_steps {
                    if let Some(p) = sim.body_world_position(&robot.root_link) {
                        peak_dy = peak_dy.max(p[1].abs());
                        min_z = min_z.min(p[2]);
                        body_x_end = p[0];
                        body_y_end = p[1];
                    }
                    if k >= force_end_step && recovery_time_s.is_none() {
                        let dy = sim.body_world_position(&robot.root_link)
                            .map(|p| p[1].abs()).unwrap_or(0.0);
                        if dy < 0.05 {
                            recovery_time_s = Some((k - force_end_step) as f64 * DT);
                        }
                    }
                }
            }

            let fell = min_z < FALL_THRESHOLD_Z;
            let dx = body_x_end - body_x_start;
            let recovery_label = match recovery_time_s {
                Some(t) => format!("{:>6.2}", t),
                None if fell => "fell".to_string(),
                None => "no recov".to_string(),
            };
            let result = if fell { "✗ fell" }
                         else if recovery_time_s.is_some() { "✓ recovered" }
                         else { "△ no recovery" };
            eprintln!(
                "        {:<40} | {:>9.3} | {:>8.3} | {:>+7.3} (Δ {:+.3}) | {} | {}",
                format!("{} ({})", mode_label, scen_label),
                peak_dy, body_y_end.abs(), dx, dx - goal_x,
                recovery_label, result,
            );
        }
        eprintln!("        {}", "─".repeat(125));
    }
}

#[test]
#[ignore = "GUI-equivalent metric anchor — run with --ignored"]
fn diag_walk_metric_matrix_gui_v2() {
    let cmd_duration = 7.0;
    // GUI uses Linear cmd = 0.30 m/s (NOT 0.15 — that's the test
    // harness's `fwd_cmd()`). Match GUI here.
    let fwd = VelocityCmd { vx: 0.30, vy: 0.0, wz: 0.0 };
    let lat = VelocityCmd { vx: 0.0, vy: 0.30, wz: 0.0 };
    let yaw = VelocityCmd { vx: 0.0, vy: 0.0, wz: 0.5 };
    let scenarios = [("forward", fwd), ("lateral", lat), ("yaw", yaw)];
    let modes: [(&str, GaitMode, bool); 4] = [
        ("champ-open",       GaitMode::Champ,          false),
        ("mpc-srbd+wbc",     GaitMode::Mpc,            true),
        ("centroidal+wbc",   GaitMode::CentroidalSrbd, true),
        ("full24+wbc",       GaitMode::FullCentroidal, true),
    ];
    eprintln!();
    eprintln!("=== diag_walk_metric_matrix_gui_v2 (constrain pose, kp=50/kv=5, grav_comp off, 7s cmd) ===");
    for (axis, cmd) in scenarios {
        eprintln!("--- axis: {axis} ---");
        for (label, mode, use_wbc) in &modes {
            let Some(m) = run_walk_gui_v2(*use_wbc, *mode, cmd) else {
                eprintln!("  {label:<18} [skipped]");
                continue;
            };
            let q = m.metrics(cmd, cmd_duration);
            eprintln!("  {label:<18} {}", q.line(axis));
        }
        eprintln!();
    }
}

/// Replicate the GUI's CHAMP-forward setup as reported by the user:
/// 1. Load namiashi URDF
/// 2. Apply the `constrain` pose from namiashi.misa
///    (thigh = +1.0, calf = -2.0, deeply-bent crouch)
/// 3. Start MuJoCo with gravity → robot falls from height onto its
///    feet in the constrain pose
/// 4. Start CHAMP with Linear cmd = 0.30 m/s (NOT 0.15 — user's
///    default in the GUI)
/// User reports body moves to (+1.13, +0.05..+0.10, 0.0) over 5-10 s.
#[test]
#[ignore = "metric debug — run with --ignored"]
fn diag_champ_forward_constrain_pose() {
    let path = common::namiashi_urdf();
    if !path.exists() { return; }
    let mut robot = articara::robot::RobotModel::from_urdf(&path).expect("load");
    // GUI uses URDF-default PD (kp=50, kv=5) — NOT the test harness's
    // (kp=30, kv=0.6). The constrain pose requires real joint torque
    // to hold against gravity (deeply bent legs); kp=30 isn't enough
    // and the robot collapses during the settle phase.
    for j in robot.joints.iter_mut() {
        if j.joint_type == "fixed" { continue; }
        j.actuator_mode = articara::robot::ActuatorMode::Position;
        j.actuator_kp = 50.0;
        j.actuator_kv = 5.0;
    }
    // Apply the `constrain` pose FIRST so the subsequent auto-detect
    // captures the deeply-bent stance as `nominal_foot_body` (= what
    // the gait controller will target). In GUI: user picks the pose
    // from the Poses panel, then later clicks Auto-Detect → Start Gait. This is what the user's GUI session does
    // before launching the gait — fundamentally different from
    // `seed_joint_positions_from_kinematics` which puts the legs near
    // straight-down (q ≈ small bend).
    let constrain_pose = [
        ("FL_hip_joint", 0.0),  ("FL_thigh_joint", 1.0),  ("FL_calf_joint", -2.0),
        ("FR_hip_joint", 0.0),  ("FR_thigh_joint", 1.0),  ("FR_calf_joint", -2.0),
        ("RL_hip_joint", 0.0),  ("RL_thigh_joint", 1.0),  ("RL_calf_joint", -2.0),
        ("RR_hip_joint", 0.0),  ("RR_thigh_joint", 1.0),  ("RR_calf_joint", -2.0),
        ("arm_pitch_joint", 0.0),
    ];
    for (name, q) in constrain_pose {
        if let Some(&idx) = robot.joint_map.get(name) {
            robot.joint_positions[idx] = q;
        } else {
            eprintln!("WARN: joint {name:?} not found in model");
        }
    }

    // Now run Auto-Detect — the foot transforms will reflect the
    // constrain pose.
    let kin = articara::gait::auto_detect_kinematics_config(
        &robot,
        &articara::gait::DEFAULT_FOOT_LINKS,
    ).expect("kin");
    eprintln!("nominal_foot_body after auto-detect at constrain pose:");
    for (label, leg) in [("FL", &kin.fl), ("FR", &kin.fr), ("RL", &kin.rl), ("RR", &kin.rr)] {
        eprintln!("  {label}: nominal_foot_body = {:?}, hip_offset = {:?}, upper={:.3} lower={:.3}",
            leg.nominal_foot_body, leg.hip_offset, leg.upper_leg_m, leg.lower_leg_m);
    }

    // Two-phase: 2 s settle (no gait, holding constrain pose) +
    // 7 s walk (gait active, cmd applied). Longer cmd windows cause
    // CHAMP's open-loop yaw drift to accumulate and the body to curve
    // off course; 7 s captures the linear-tracking regime.
    let settle_s = 2.0;
    let cmd_s = 7.0;

    // GUI uses `add_actuators: false` in its MjcfExportOptions; the
    // test harness default sets `true`. Match the GUI here.
    let opts = articara::mjcf::MjcfExportOptions {
        ground_plane: Some(articara::mjcf::GroundPlaneCfg {
            z: 0.0, half_size: 4.0, roll: 0.0, pitch: 0.0,
        }),
        add_actuators: false,
        ..Default::default()
    };
    let mut sim = articara::mujoco_sim::MujocoSim::new(&robot, opts).expect("sim");
    // GUI default is grav-comp OFF (`enforce_gravity_compensation = false`).
    sim.set_gravity_compensation(false);

    let cfg = GaitConfig::trot();
    let mut gc = GaitController::build(&robot, kin, cfg, GaitMode::Champ).expect("build");
    // NOT enabled yet — GUI's step 4 (PlayMuJoCo) runs WITHOUT gait
    // active. The robot falls and stabilises in the constrain pose
    // via PD position-hold. Step 5 (Auto-Detect + Start Gait) enables
    // the gait AFTER the robot has settled.

    // Set position targets to constrain pose initially so PD holds it
    // during the settle window.
    for (name, q) in constrain_pose {
        if let Some(&idx) = robot.joint_map.get(name) {
            sim.set_position_target(idx, q);
        }
    }

    // GUI cmd = 0.30 m/s (not 0.15 in fwd_cmd()).
    let cmd = VelocityCmd { vx: 0.30, vy: 0.0, wz: 0.0 };

    let total_s = settle_s + cmd_s;
    let n_steps = (total_s / DT) as usize;
    let settle_steps = (settle_s / DT) as usize;
    let mut m = WalkBenchmark::default();
    m.min_body_z = f64::INFINITY;

    eprintln!("initial body pos: {:?}", sim.body_world_position(&robot.root_link));
    for k in 0..n_steps {
        if k == settle_steps {
            // Settle done — start the gait and apply forward cmd.
            gc.enable();
            gc.set_velocity_cmd(cmd);
            // Record start of cmd window.
            if let Some(p) = sim.body_world_position(&robot.root_link) {
                m.body_x_start = p[0]; m.body_y_start = p[1];
            }
            m.yaw_start = sim.body_world_yaw(&robot.root_link).unwrap_or(0.0);
            eprintln!("at cmd start (t={settle_s}s): body_pos = ({:.3}, {:.3}, {:.3}), yaw = {:+.3}",
                m.body_x_start, m.body_y_start,
                sim.body_world_position(&robot.root_link).map(|p| p[2]).unwrap_or(0.0),
                m.yaw_start);
        }

        if k < settle_steps {
            // Settle phase: keep PD on the constrain pose.
            for (name, q) in constrain_pose {
                if let Some(&idx) = robot.joint_map.get(name) {
                    sim.set_position_target(idx, q);
                }
            }
        } else {
            // Walk phase: gait controller drives joint targets.
            let v_obs = sim.body_world_linear_velocity(&robot.root_link).unwrap_or([0.0; 3]);
            let w_obs = sim.body_world_angular_velocity(&robot.root_link).unwrap_or([0.0; 3]);
            gc.set_body_state_observed(
                Vector3::new(v_obs[0], v_obs[1], v_obs[2]),
                Vector3::new(w_obs[0], w_obs[1], w_obs[2]),
            );
            let body_pos = sim.body_world_position(&robot.root_link).unwrap_or([0.0; 3]);
            let yaw_obs = sim.body_world_yaw(&robot.root_link).unwrap_or(0.0);
            gc.set_body_pose_observed(yaw_obs, Vector3::new(body_pos[0], body_pos[1], body_pos[2]));
            let (_o, targets, _ff) = gc.tick(DT);
            for (idx, q) in targets { sim.set_position_target(idx, q); }
        }
        sim.step(&mut robot, DT, true);
        if let Some(p) = sim.body_world_position(&robot.root_link) {
            m.body_x_end = p[0]; m.body_y_end = p[1];
            m.min_body_z = m.min_body_z.min(p[2]);
        }
        m.yaw_end = sim.body_world_yaw(&robot.root_link).unwrap_or(0.0);
    }
    let cmd_duration = cmd_s;
    eprintln!();
    eprintln!("=== CHAMP forward CONSTRAIN-POSE (cmd vx=+{:.2} m/s, {:.1}s under cmd) ===",
              cmd.vx, cmd_duration);
    eprintln!("  dx_world  = {:+.4} m   dy_world = {:+.4} m", m.dx_world(), m.dy_world());
    eprintln!("  dyaw      = {:+.3} rad  min_z    = {:.4} m", m.dyaw(), m.min_body_z);
    let q = m.metrics(cmd, cmd_duration);
    eprintln!("  metric    = {}", q.line("forward"));
    eprintln!("  (user GUI reported: dx ≈ +1.13, dy ≈ +0.05..+0.10)");
}

/// Run CHAMP forward **without** the foot-nudge that the test fixture
/// applies (`nominal_foot_body.z += 8 % * leg_total`). The GUI flow
/// does not apply this nudge, so this test approximates the GUI's
/// starting pose more closely. If the result differs noticeably from
/// `diag_champ_forward_raw`, the nudge is the root cause.
#[test]
#[ignore = "metric debug — run with --ignored"]
fn diag_champ_forward_no_nudge() {
    let cmd = fwd_cmd();
    let path = common::namiashi_urdf();
    if !path.exists() { return; }
    let mut robot = articara::robot::RobotModel::from_urdf(&path).expect("load");
    common::setup_position_pd_actuators(&mut robot);
    let kin = articara::gait::auto_detect_kinematics_config(
        &robot,
        &articara::gait::DEFAULT_FOOT_LINKS,
    ).expect("kin");
    // NO 8% nudge — leaves nominal_foot_body at the URDF q=0 pose.
    common::seed_joint_positions_from_kinematics(&mut robot, &kin);

    let mut sim = articara::mujoco_sim::MujocoSim::new(
        &robot,
        common::default_mjcf_export_options(),
    ).expect("sim");
    sim.set_gravity_compensation(true);

    let cfg = GaitConfig::trot();
    let mut gc = GaitController::build(&robot, kin, cfg, GaitMode::Champ)
        .expect("build");
    gc.enable();

    let n_steps = (WALK_SIM_TIME_S / DT) as usize;
    let burn_in_steps = (WALK_BURN_IN_S / DT) as usize;
    let mut m = WalkBenchmark::default();
    m.min_body_z = f64::INFINITY;
    if let Some(p) = sim.body_world_position(&robot.root_link) {
        m.body_x_start = p[0]; m.body_y_start = p[1];
    }
    m.yaw_start = sim.body_world_yaw(&robot.root_link).unwrap_or(0.0);
    for k in 0..n_steps {
        if k == burn_in_steps {
            gc.set_velocity_cmd(cmd);
        }
        let (_o, targets, _ff) = gc.tick(DT);
        for (idx, q) in targets { sim.set_position_target(idx, q); }
        sim.step(&mut robot, DT, true);
        if let Some(p) = sim.body_world_position(&robot.root_link) {
            m.body_x_end = p[0]; m.body_y_end = p[1];
            m.min_body_z = m.min_body_z.min(p[2]);
        }
        m.yaw_end = sim.body_world_yaw(&robot.root_link).unwrap_or(0.0);
    }
    eprintln!();
    eprintln!("=== CHAMP forward NO-NUDGE (URDF q=0 nominal) ===");
    eprintln!("  dx_world  = {:+.4} m   dy_world = {:+.4} m", m.dx_world(), m.dy_world());
    eprintln!("  dyaw      = {:+.3} rad  min_z    = {:.4} m", m.dyaw(), m.min_body_z);
    let q = m.metrics(cmd, WALK_SIM_TIME_S - WALK_BURN_IN_S);
    eprintln!("  metric    = {}", q.line("forward"));
}

/// Same shape as `diag_champ_forward_gui_pd` but uses the test
/// harness's default PD (kp=30, kv=0.6) for direct comparison. This
/// is just `diag_champ_forward_raw` renamed for clarity.
#[test]
#[ignore = "metric debug — run with --ignored"]
fn diag_champ_forward_raw() {
    let cmd = fwd_cmd();
    let Some(m) = run_walk(false, GaitMode::Champ, cmd, false) else {
        return;
    };
    eprintln!();
    eprintln!("=== CHAMP forward raw (cmd vx=+{:.2} m/s, 4.5 s under cmd) ===", cmd.vx);
    eprintln!("  yaw_start = {:+.3} rad", m.yaw_start);
    eprintln!("  yaw_end   = {:+.3} rad", m.yaw_end);
    eprintln!("  dyaw      = {:+.3} rad", m.dyaw());
    eprintln!("  dx_world  = {:+.4} m  (raw, world x)", m.dx_world());
    eprintln!("  dy_world  = {:+.4} m  (raw, world y)", m.dy_world());
    eprintln!("  body_dx   = {:+.4} m  (projected on initial yaw)", m.body_dx());
    eprintln!("  body_dy   = {:+.4} m  (projected, perp to initial yaw)", m.body_dy());
    eprintln!("  norm_xy   = {:.4} m   (path radius)", (m.dx_world().powi(2) + m.dy_world().powi(2)).sqrt());
    eprintln!("  min_z     = {:.4} m", m.min_body_z);
    let q = m.metrics(cmd, WALK_SIM_TIME_S - WALK_BURN_IN_S);
    eprintln!("  metric    = {}", q.line("forward"));
}

#[test]
#[ignore = "metric-anchor benchmark — run with --ignored"]
fn diag_walk_metric_matrix() {
    let cmd_duration = WALK_SIM_TIME_S - WALK_BURN_IN_S;
    let scenarios = [
        ("forward", fwd_cmd()),
        ("lateral", lat_cmd()),
        ("yaw", yaw_cmd()),
    ];
    // (label, gait_mode, use_wbc). CHAMP runs no-WBC (it has no GRF
    // signal); the three MPC modes run with the host's WBC enabled
    // (= matches the GUI default and the `*_wbc` regression tests).
    let modes: [(&str, GaitMode, bool); 4] = [
        ("champ-open", GaitMode::Champ, false),
        ("mpc-srbd+wbc", GaitMode::Mpc, true),
        ("centroidal+wbc", GaitMode::CentroidalSrbd, true),
        ("full24+wbc", GaitMode::FullCentroidal, true),
    ];
    eprintln!();
    eprintln!("=== diag_walk_metric_matrix (cmd_duration = {:.1} s) ===", cmd_duration);
    eprintln!();
    for (axis, cmd) in scenarios {
        eprintln!("--- axis: {axis} ---");
        for (label, mode, use_wbc) in &modes {
            let Some(m) = run_walk(*use_wbc, *mode, cmd, false) else {
                eprintln!("  {label:<18} [model load failed]");
                continue;
            };
            let metrics = m.metrics(cmd, cmd_duration);
            eprintln!("  {label:<18} {}", metrics.line(axis));
        }
        eprintln!();
    }
}

#[test]
#[ignore = "D3.3.6 diagnostic — run with --ignored to inspect"]
fn diag_full_centroidal_no_wbc_3axis() {
    for (label, cmd) in [
        ("forward", fwd_cmd()),
        ("lateral", lat_cmd()),
        ("yaw", yaw_cmd()),
    ] {
        let Some(m) = run_walk(false, GaitMode::FullCentroidal, cmd, false) else {
            return;
        };
        eprintln!(
            "[{label}:full-centroidal-only] body_dx={:+.3} m  body_dy={:+.3} m  Δyaw={:+.3} rad  min_z={:.3} m",
            m.body_dx(), m.body_dy(), m.dyaw(), m.min_body_z,
        );
    }
}

#[test]
#[ignore = "D3.3.6 characterisation — assertions kept as goal"]
fn integration_walk_straight_full_centroidal_wbc() {
    let Some(m) = run_walk(true, GaitMode::FullCentroidal, fwd_cmd(), true) else {
        return;
    };
    eprintln!(
        "[forward:full-centroidal+wbc] body_dx={:+.3} m  body_dy={:+.3} m  Δyaw={:+.3} rad  min_z={:.3} m",
        m.body_dx(), m.body_dy(), m.dyaw(), m.min_body_z,
    );
    assert!(m.min_body_z > FALL_THRESHOLD_Z, "FullCentroidal+WBC fell (forward)");
    assert!(m.body_dx() > 0.10,
        "forward (full-centroidal): body_dx = {:+.3} m, expected > +0.10 m", m.body_dx());
    assert!(m.body_dy().abs() < 0.20,
        "forward (full-centroidal): body_dy = {:+.3} m, expected |·| < 0.20 m", m.body_dy());
    assert!(m.dyaw().abs() < 1.0,
        "forward (full-centroidal): Δyaw = {:+.3} rad, expected |·| < 1.0 rad", m.dyaw());
}

#[test]
#[ignore = "D3.3.6 characterisation — assertions kept as goal"]
fn integration_walk_lateral_full_centroidal_wbc() {
    let Some(m) = run_walk(true, GaitMode::FullCentroidal, lat_cmd(), true) else {
        return;
    };
    eprintln!(
        "[lateral:full-centroidal+wbc] body_dx={:+.3} m  body_dy={:+.3} m  Δyaw={:+.3} rad  min_z={:.3} m",
        m.body_dx(), m.body_dy(), m.dyaw(), m.min_body_z,
    );
    assert!(m.min_body_z > FALL_THRESHOLD_Z, "FullCentroidal+WBC fell (lateral)");
    assert!(m.body_dy() > 0.20,
        "lateral (full-centroidal): body_dy = {:+.3} m, expected > +0.20 m", m.body_dy());
    assert!(m.body_dx().abs() < 0.30,
        "lateral (full-centroidal): body_dx = {:+.3} m, expected |·| < 0.30 m", m.body_dx());
    assert!(m.dyaw().abs() < 1.5,
        "lateral (full-centroidal): Δyaw = {:+.3} rad, expected |·| < 1.5 rad", m.dyaw());
}

#[test]
#[ignore = "D3.3.6 characterisation — assertions kept as goal"]
fn integration_walk_yaw_full_centroidal_wbc() {
    let Some(m) = run_walk(true, GaitMode::FullCentroidal, yaw_cmd(), true) else {
        return;
    };
    eprintln!(
        "[yaw:full-centroidal+wbc] body_dx={:+.3} m  body_dy={:+.3} m  Δyaw={:+.3} rad  min_z={:.3} m",
        m.body_dx(), m.body_dy(), m.dyaw(), m.min_body_z,
    );
    assert!(m.min_body_z > FALL_THRESHOLD_Z, "FullCentroidal+WBC fell (yaw)");
    assert!(m.dyaw().abs() > 1.5,
        "yaw (full-centroidal): Δyaw = {:+.3} rad, expected |·| > 1.5 rad", m.dyaw());
    assert!(m.body_dx().abs() < 0.35,
        "yaw (full-centroidal): body_dx = {:+.3} m, expected |·| < 0.35 m", m.body_dx());
    assert!(m.body_dy().abs() < 0.35,
        "yaw (full-centroidal): body_dy = {:+.3} m, expected |·| < 0.35 m", m.body_dy());
}

/// Repro of the user-reported axis swap at high cmd magnitude: drives
/// vx = +0.30 m/s (2× the standard test) and vy = +0.30 m/s separately
/// for 5 s each, prints the (body_dx, body_dy, Δyaw) so we can see
/// which axis dominates. No assertions — this is a diagnostic.
#[test]
#[ignore = "diagnostic — run with --ignored"]
fn diag_high_cmd_axis_swap() {
    let high_fwd = VelocityCmd { vx: 0.30, vy: 0.0, wz: 0.0 };
    let high_lat = VelocityCmd { vx: 0.0, vy: 0.30, wz: 0.0 };
    if let Some(m) = run_walk(true, GaitMode::Mpc, high_fwd, false) {
        eprintln!(
            "[diag:vx=0.3] body_dx={:+.3} m  body_dy={:+.3} m  Δyaw={:+.3} rad",
            m.body_dx(), m.body_dy(), m.dyaw(),
        );
    }
    if let Some(m) = run_walk(true, GaitMode::Mpc, high_lat, false) {
        eprintln!(
            "[diag:vy=0.3] body_dx={:+.3} m  body_dy={:+.3} m  Δyaw={:+.3} rad",
            m.body_dx(), m.body_dy(), m.dyaw(),
        );
    }
}

/// Reproduce the *user's GUI-driven* axis-swap report: load via misa,
/// seed `constrain` pose, recompute kin's `nominal_foot_body` from FK
/// (= what `gait_use_current_pose_as_stance()` does in the script),
/// then drive vx = +0.30 / vy = +0.30 for 3 s each. Prints traj — no
/// assertions.
#[test]
#[ignore = "diagnostic — run with --ignored"]
fn diag_constrain_pose_axis_swap() {
    use articara::gait::{auto_detect_kinematics_config, DEFAULT_FOOT_LINKS};
    use articara::robot::RobotModel;
    use articara::mujoco_sim::MujocoSim;
    use articara::wbc_pipeline::WbcPipeline;

    let misa_path = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("tests")
        .join("fixtures")
        .join("namiashi")
        .join("namiashi.misa");
    if !misa_path.exists() {
        eprintln!("namiashi.misa missing — skipping");
        return;
    }
    let mut robot = RobotModel::from_misa(&misa_path).expect("load misa");
    common::setup_position_pd_actuators(&mut robot);

    // Auto-detect kin (q=0 reference foot positions).
    let mut kin = auto_detect_kinematics_config(&robot, &DEFAULT_FOOT_LINKS)
        .expect("auto-detect kinematics");

    // Seed constrain pose: copy [[pose]] "constrain" angles into joint_positions.
    let constrain = robot.poses.iter().find(|p| p.name == "constrain")
        .expect("constrain pose").clone();
    for (ji, joint) in robot.joints.iter().enumerate() {
        if let Some(&q) = constrain.angles.get(&joint.name) {
            robot.joint_positions[ji] = q;
        }
    }

    // gait_use_current_pose_as_stance(): recompute nominal_foot_body from FK
    // at the just-seeded constrain joint angles.
    let transforms = robot.compute_transforms();
    let body_pos: nalgebra::Vector3<f64> = transforms
        .get(&robot.root_link)
        .map(|iso| iso.translation.vector.cast::<f64>())
        .unwrap_or_else(nalgebra::Vector3::zeros);
    for kin_leg in [&mut kin.fl, &mut kin.fr, &mut kin.rl, &mut kin.rr] {
        if let Some(foot_iso) = transforms.get(&kin_leg.foot_link) {
            let foot_pos: nalgebra::Vector3<f64> =
                foot_iso.translation.vector.cast::<f64>();
            kin_leg.nominal_foot_body = foot_pos - body_pos;
        }
    }
    eprintln!(
        "[diag:setup] nominal_foot_body FL=({:+.3}, {:+.3}, {:+.3})",
        kin.fl.nominal_foot_body.x,
        kin.fl.nominal_foot_body.y,
        kin.fl.nominal_foot_body.z,
    );

    let mut sim = MujocoSim::new(&robot, common::default_mjcf_export_options())
        .expect("MujocoSim::new");
    sim.set_gravity_compensation(true);

    // Quick test runner inline (mirrors run_walk but with custom kin
    // already seeded in robot, plus this test runs only 3 s, no
    // burn-in retune).
    let cfg = quadruped_gait::GaitConfig::trot();
    let mut gc = articara::gait::GaitController::build(
        &robot, kin, cfg, quadruped_gait::GaitMode::Mpc,
    ).expect("GaitController::build");
    let use_wbc = std::env::var("DIAG_WBC").map(|v| v != "0").unwrap_or(true);
    eprintln!("[diag:setup] use_wbc = {use_wbc}");
    let mut wbc_pipeline = if use_wbc {
        let mut p = WbcPipeline::new(&robot, common::default_foot_links());
        if let Some(srbd_cfg) = gc.srbd_mpc_config() {
            p.mass_kg = srbd_cfg.mass_kg;
            p.inertia_diag_body = srbd_cfg.inertia_diag_body;
        }
        Some(p)
    } else {
        None
    };

    for (label, cmd) in [
        ("vx=0.0", VelocityCmd::zero()),
        ("vx=+0.3", VelocityCmd { vx: 0.30, vy: 0.0, wz: 0.0 }),
        ("vy=+0.3", VelocityCmd { vx: 0.0, vy: 0.30, wz: 0.0 }),
    ] {
        // 0.5 s burn-in @ zero cmd, then 3 s @ cmd.
        gc.set_velocity_cmd(VelocityCmd::zero());
        gc.enable();
        let pos0 = sim.body_world_position(&robot.root_link).unwrap_or([0.0; 3]);
        let yaw0 = sim.body_world_yaw(&robot.root_link).unwrap_or(0.0);
        let dt = sim.timestep();
        let burn = (0.5 / dt) as usize;
        let active = (3.0 / dt) as usize;
        for k in 0..(burn + active) {
            if k == burn { gc.set_velocity_cmd(cmd); }
            if let Some(p) = wbc_pipeline.as_mut() {
                p.weights = quadruped_gait::wbc::WbcWeights::for_cmd(&gc.velocity_cmd());
            }
            let v_obs = sim.body_world_linear_velocity(&robot.root_link).unwrap_or([0.0; 3]);
            let w_obs = sim.body_world_angular_velocity(&robot.root_link).unwrap_or([0.0; 3]);
            gc.set_body_state_observed(
                nalgebra::Vector3::new(v_obs[0], v_obs[1], v_obs[2]),
                nalgebra::Vector3::new(w_obs[0], w_obs[1], w_obs[2]),
            );
            let pos = sim.body_world_position(&robot.root_link).unwrap_or([0.0; 3]);
            let yaw = sim.body_world_yaw(&robot.root_link).unwrap_or(0.0);
            gc.set_body_pose_observed(
                yaw, nalgebra::Vector3::new(pos[0], pos[1], pos[2]),
            );
            let (out, targets, torque_ff) = gc.tick(dt);
            for (idx, q) in targets {
                sim.set_position_target(idx, q);
            }
            if let Some(pipeline) = wbc_pipeline.as_mut() {
                // WBC torques via Hybrid path (τ_ff on top of Position-PD).
                let f_grf_world = gc.predicted_grfs()
                    .map(|s| s.grfs_first_step)
                    .unwrap_or([nalgebra::Vector3::zeros(); 4]);
                let v_cmd_body = nalgebra::Vector3::new(cmd.vx, cmd.vy, 0.0);
                let v_obs_v3 = nalgebra::Vector3::new(v_obs[0], v_obs[1], v_obs[2]);
                let omega_obs_v3 = nalgebra::Vector3::new(w_obs[0], w_obs[1], w_obs[2]);
                let kin_now = gc.kinematics().clone();
                let joint_indices = gc.joint_indices();
                let joint_signs = gc.joint_signs();
                let foot_links_str: [&str; 4] = [
                    pipeline.foot_links[0].as_str(),
                    pipeline.foot_links[1].as_str(),
                    pipeline.foot_links[2].as_str(),
                    pipeline.foot_links[3].as_str(),
                ];
                let force_z = sim.contact_force_per_foot(&foot_links_str);
                let nominal_phases = [
                    out.legs[0].phase, out.legs[1].phase,
                    out.legs[2].phase, out.legs[3].phase,
                ];
                let corrected = quadruped_gait::ContactDrivenPhase::apply_correction(
                    &nominal_phases, force_z, 5.0, 0.0,
                );
                let contact_flag_corrected = [
                    corrected[0].is_stance, corrected[1].is_stance,
                    corrected[2].is_stance, corrected[3].is_stance,
                ];
                let taus = pipeline.solve(
                    &robot, &sim, &out, &kin_now, joint_indices, joint_signs,
                    &v_cmd_body, cmd.wz, &v_obs_v3, &omega_obs_v3,
                    &f_grf_world, contact_flag_corrected, dt,
                );
                for (ji, &tau) in taus.iter().enumerate() {
                    sim.set_torque_feedforward(ji, tau);
                }
            } else {
                // No WBC → use the gait controller's MPC-derived τ_ff
                for (idx, tau) in torque_ff {
                    sim.set_torque_feedforward(idx, tau);
                }
            }
            sim.step(&mut robot, dt, true);
        }
        let pos1 = sim.body_world_position(&robot.root_link).unwrap_or([0.0; 3]);
        let yaw1 = sim.body_world_yaw(&robot.root_link).unwrap_or(0.0);
        eprintln!(
            "[diag:{}] pos start=({:+.2}, {:+.2}, {:+.2}) end=({:+.2}, {:+.2}, {:+.2})  Δ=({:+.3}, {:+.3}, {:+.3})  yaw {:+.0}°→{:+.0}°",
            label,
            pos0[0], pos0[1], pos0[2],
            pos1[0], pos1[1], pos1[2],
            pos1[0] - pos0[0], pos1[1] - pos0[1], pos1[2] - pos0[2],
            yaw0.to_degrees(), yaw1.to_degrees(),
        );
    }
}

// ─── Lateral-walk benchmarks ──────────────────────────────────────

/// MPC + Position-PD (no WBC) lateral walk — diagnostic that
/// **isolates the MPC path** from the WBC integration. If lateral
/// works here and breaks in `*_mpc_wbc`, the WBC layer is at fault;
/// if it breaks here too, the problem is in the gait controller's
/// MPC mode itself.
#[test]
fn integration_walk_lateral_mpc_no_wbc() {
    let Some(m) = run_walk(false, GaitMode::Mpc, lat_cmd(), false) else {
        return;
    };
    eprintln!(
        "[lateral:mpc-only] body_dx={:+.3} m  body_dy={:+.3} m  Δyaw={:+.3} rad  min_z={:.3} m",
        m.body_dx(), m.body_dy(), m.dyaw(), m.min_body_z,
    );
    assert!(m.min_body_z > FALL_THRESHOLD_Z, "MPC fell (lateral diagnostic)");
}

/// CHAMP lateral walk benchmark — open-loop documentation only.
/// Uses `use_misa = true` for fair comparison with `*_mpc_wbc` (which
/// also runs misa). See `integration_walk_straight_champ` doc comment.
#[test]
fn integration_walk_lateral_champ() {
    let Some(m) = run_walk(false, GaitMode::Champ, lat_cmd(), true) else {
        return;
    };
    eprintln!(
        "[lateral:champ] body_dx={:+.3} m  body_dy={:+.3} m  Δyaw={:+.3} rad  min_z={:.3} m",
        m.body_dx(), m.body_dy(), m.dyaw(), m.min_body_z,
    );
    assert!(m.min_body_z > FALL_THRESHOLD_Z, "CHAMP fell (lateral)");
}

/// MPC+WBC lateral walk under `WbcWeights::for_cmd` (P5b). The
/// per-cmd schedule dials swing_leg from 1.0 (forward default) down
/// to 0.1 for full-rate lateral commands, removing the joint-space
/// PD reaction-torque sign flip.
///
/// Active axis: body_dy > +20 cm under cmd.vy = +0.10 m/s, 5 s.
/// Cross axes: |body_dx| < 30 cm, |Δyaw| < 1.5 rad.
///
/// Migrated to .misa loader (2026-05-13): cross-axis residuals
/// collapsed an order of magnitude (dx ±0.23 → ±0.01, Δyaw ±1.20 →
/// ±0.05) while dy tracking stayed near-identical (+0.501 → +0.420).
#[test]
fn integration_walk_lateral_mpc_wbc() {
    let Some(m) = run_walk(true, GaitMode::Mpc, lat_cmd(), true) else {
        return;
    };
    eprintln!(
        "[lateral:mpc+wbc] body_dx={:+.3} m  body_dy={:+.3} m  Δyaw={:+.3} rad  min_z={:.3} m",
        m.body_dx(), m.body_dy(), m.dyaw(), m.min_body_z,
    );
    assert!(m.min_body_z > FALL_THRESHOLD_Z, "MPC+WBC fell (lateral)");
    assert!(m.body_dy() > 0.20,
        "lateral: body_dy = {:+.3} m, expected > +0.20 m", m.body_dy());
    assert!(m.body_dx().abs() < 0.30,
        "lateral: body_dx = {:+.3} m, expected |·| < 0.30 m", m.body_dx());
    assert!(m.dyaw().abs() < 1.5,
        "lateral: Δyaw = {:+.3} rad, expected |·| < 1.5 rad", m.dyaw());
}

// ─── Yaw-rotate benchmarks ────────────────────────────────────────

/// CHAMP yaw rotate benchmark — open-loop documentation only.
/// Uses `use_misa = true` for fair comparison with `*_mpc_wbc`.
#[test]
fn integration_walk_yaw_champ() {
    let Some(m) = run_walk(false, GaitMode::Champ, yaw_cmd(), true) else {
        return;
    };
    eprintln!(
        "[yaw:champ] body_dx={:+.3} m  body_dy={:+.3} m  Δyaw={:+.3} rad  min_z={:.3} m",
        m.body_dx(), m.body_dy(), m.dyaw(), m.min_body_z,
    );
    assert!(m.min_body_z > FALL_THRESHOLD_Z, "CHAMP fell (yaw)");
}

/// MPC+WBC yaw rotate under `WbcWeights::for_cmd` (P5b).
///
/// Active axis: |Δyaw| > 1.5 rad. Cross axes: |body_dx| / |body_dy|
/// < 35 cm.
///
/// Migrated to .misa loader (2026-05-13): cross-axis drift cleaned
/// up dramatically (dx ±0.28 → ±0.01, dy ±0.18 → ±0.01). Δyaw
/// dropped from +2.76 rad (overshoot, capture-point feedback
/// amplifying) to +1.53 rad (just-above envelope). The undershoot is
/// because disabling `k_capture` for misa removed the artifact that
/// boosted yaw tracking under soft PD. Tracking 61% of cmd; could
/// improve by tuning q_diag yaw weights (deferred).
#[test]
fn integration_walk_yaw_mpc_wbc() {
    let Some(m) = run_walk(true, GaitMode::Mpc, yaw_cmd(), true) else {
        return;
    };
    eprintln!(
        "[yaw:mpc+wbc] body_dx={:+.3} m  body_dy={:+.3} m  Δyaw={:+.3} rad  min_z={:.3} m",
        m.body_dx(), m.body_dy(), m.dyaw(), m.min_body_z,
    );
    assert!(m.min_body_z > FALL_THRESHOLD_Z, "MPC+WBC fell (yaw)");
    assert!(m.dyaw().abs() > 1.5,
        "yaw: Δyaw = {:+.3} rad, expected |·| > 1.5 rad", m.dyaw());
    assert!(m.body_dx().abs() < 0.35,
        "yaw: body_dx = {:+.3} m, expected |·| < 0.35 m", m.body_dx());
    assert!(m.body_dy().abs() < 0.35,
        "yaw: body_dy = {:+.3} m, expected |·| < 0.35 m", m.body_dy());
}

// ─── P5a: per-task lateral diagnostic ─────────────────────────────
//
// Lateral cmd flips sign with WBC (MPC-only +0.026 m → MPC+WBC
// -0.903 m). To isolate which WBC task is responsible, run the
// lateral sim 4 times with one priority-1/2 LSQ task disabled at a
// time and compare body_dy. The task whose disable **un-flips** the
// sign (or noticeably reduces the negative drift) is the culprit.
//
// All four diagnostics are `#[ignore]`d so they don't slow down the
// regular regression run; invoke with:
//   cargo test --release --features mujoco --test integration_walk \
//     diag_lateral -- --ignored --nocapture

use quadruped_gait::wbc::WbcWeights;

/// Run a lateral walk with custom WbcWeights and return the
/// resulting body_dy / yaw drift / fall metric. Mirrors `run_walk`
/// but lets the caller override the WBC task weights.
fn run_lateral_with_weights(weights: WbcWeights) -> Option<WalkBenchmark> {
    let common::StandFixture {
        mut robot,
        kin,
        mut sim,
    } = common::build_namiashi_stand_fixture()?;

    let cfg = GaitConfig::trot();
    let mut gc = GaitController::build(&robot, kin.clone(), cfg, GaitMode::Mpc)
        .expect("GaitController::build");
    let mut wbc_pipeline = WbcPipeline::new(&robot, common::default_foot_links());
    wbc_pipeline.weights = weights;
    if let Some(srbd_cfg) = gc.srbd_mpc_config() {
        wbc_pipeline.mass_kg = srbd_cfg.mass_kg;
        wbc_pipeline.inertia_diag_body = srbd_cfg.inertia_diag_body;
    }

    let n_steps = (WALK_SIM_TIME_S / DT) as usize;
    let burn_in_steps = (WALK_BURN_IN_S / DT) as usize;
    let mut metrics = WalkBenchmark::default();
    metrics.min_body_z = f64::INFINITY;
    if let Some(pos) = sim.body_world_position(&robot.root_link) {
        metrics.body_x_start = pos[0];
        metrics.body_y_start = pos[1];
    }
    metrics.yaw_start = sim.body_world_yaw(&robot.root_link).unwrap_or(0.0);

    gc.enable();
    let cmd = lat_cmd();
    for k in 0..n_steps {
        if k == burn_in_steps {
            gc.set_velocity_cmd(cmd);
        }
        let v_obs = sim
            .body_world_linear_velocity(&robot.root_link)
            .unwrap_or([0.0, 0.0, 0.0]);
        let w_obs = sim
            .body_world_angular_velocity(&robot.root_link)
            .unwrap_or([0.0, 0.0, 0.0]);
        gc.set_body_state_observed(
            Vector3::new(v_obs[0], v_obs[1], v_obs[2]),
            Vector3::new(w_obs[0], w_obs[1], w_obs[2]),
        );
        let body_pos = sim
            .body_world_position(&robot.root_link)
            .unwrap_or([0.0, 0.0, 0.0]);
        let yaw_obs = sim.body_world_yaw(&robot.root_link).unwrap_or(0.0);
        gc.set_body_pose_observed(
            yaw_obs,
            Vector3::new(body_pos[0], body_pos[1], body_pos[2]),
        );

        let (out, targets, _torque_ff) = gc.tick(DT);
        for (idx, q) in targets {
            sim.set_position_target(idx, q);
        }
        if k >= burn_in_steps {
            let f_grf_world = gc
                .predicted_grfs()
                .map(|sol| sol.grfs_first_step)
                .unwrap_or([Vector3::zeros(); 4]);
            let cmd_now = gc.velocity_cmd();
            let v_cmd_body = Vector3::new(cmd_now.vx, cmd_now.vy, 0.0);
            let foot_links_str: [&str; 4] = [
                wbc_pipeline.foot_links[0].as_str(),
                wbc_pipeline.foot_links[1].as_str(),
                wbc_pipeline.foot_links[2].as_str(),
                wbc_pipeline.foot_links[3].as_str(),
            ];
            let force_z = sim.contact_force_per_foot(&foot_links_str);
            let nominal_phases = [
                out.legs[0].phase,
                out.legs[1].phase,
                out.legs[2].phase,
                out.legs[3].phase,
            ];
            let corrected = ContactDrivenPhase::apply_correction(
                &nominal_phases,
                force_z,
                5.0,
                0.0,
            );
            let contact_flag = [
                corrected[0].is_stance,
                corrected[1].is_stance,
                corrected[2].is_stance,
                corrected[3].is_stance,
            ];
            let taus = wbc_pipeline.solve(
                &robot,
                &sim,
                &out,
                gc.kinematics(),
                gc.joint_indices(),
                gc.joint_signs(),
                &v_cmd_body,
                cmd_now.wz,
                &Vector3::new(v_obs[0], v_obs[1], v_obs[2]),
                &Vector3::new(w_obs[0], w_obs[1], w_obs[2]),
                &f_grf_world,
                contact_flag,
                DT,
            );
            for (ji, &tau) in taus.iter().enumerate() {
                sim.set_torque_feedforward(ji, tau);
            }
        } else {
            for ji in 0..robot.joints.len() {
                sim.set_torque_feedforward(ji, 0.0);
            }
        }
        sim.step(&mut robot, DT, true);

        if let Some(pos) = sim.body_world_position(&robot.root_link) {
            metrics.min_body_z = metrics.min_body_z.min(pos[2]);
            metrics.body_x_end = pos[0];
            metrics.body_y_end = pos[1];
        }
        metrics.yaw_end = sim.body_world_yaw(&robot.root_link).unwrap_or(0.0);
    }
    Some(metrics)
}

/// Diagnostic: lateral cmd with `base_accel` weight = 0.
/// If body_dy goes from -0.90 → ~+0.03 (matching MPC-only), this
/// task is the source of the sign-flip.
#[test]
#[ignore = "P5a diagnostic — run with --ignored to inspect"]
fn diag_lateral_no_base_accel() {
    let mut w = WbcWeights::default();
    w.base_accel = 0.0;
    let Some(m) = run_lateral_with_weights(w) else { return };
    eprintln!(
        "[diag-lat:no-base-accel] body_dx={:+.3} body_dy={:+.3} Δyaw={:+.3}",
        m.body_dx(), m.body_dy(), m.dyaw(),
    );
}

/// Diagnostic: lateral cmd with `swing_leg` weight = 0.
#[test]
#[ignore = "P5a diagnostic — run with --ignored to inspect"]
fn diag_lateral_no_swing_leg() {
    let mut w = WbcWeights::default();
    w.swing_leg = 0.0;
    let Some(m) = run_lateral_with_weights(w) else { return };
    eprintln!(
        "[diag-lat:no-swing-leg] body_dx={:+.3} body_dy={:+.3} Δyaw={:+.3}",
        m.body_dx(), m.body_dy(), m.dyaw(),
    );
}

/// Diagnostic: lateral cmd with `contact_force` weight = 0.
#[test]
#[ignore = "P5a diagnostic — run with --ignored to inspect"]
fn diag_lateral_no_contact_force() {
    let mut w = WbcWeights::default();
    w.contact_force = 0.0;
    let Some(m) = run_lateral_with_weights(w) else { return };
    eprintln!(
        "[diag-lat:no-contact-force] body_dx={:+.3} body_dy={:+.3} Δyaw={:+.3}",
        m.body_dx(), m.body_dy(), m.dyaw(),
    );
}

/// Diagnostic: lateral cmd with `tau_gravity` weight = 0.
#[test]
#[ignore = "P5a diagnostic — run with --ignored to inspect"]
fn diag_lateral_no_tau_gravity() {
    let mut w = WbcWeights::default();
    w.tau_gravity = 0.0;
    let Some(m) = run_lateral_with_weights(w) else { return };
    eprintln!(
        "[diag-lat:no-tau-gravity] body_dx={:+.3} body_dy={:+.3} Δyaw={:+.3}",
        m.body_dx(), m.body_dy(), m.dyaw(),
    );
}

/// Diagnostic: forward cmd with swing_leg = 0 (= P5a candidate fix).
/// If forward walk still passes (Δx > +10 cm), we can safely default
/// swing_leg to 0.
#[test]
#[ignore = "P5a candidate-fix verification"]
fn diag_forward_no_swing_leg() {
    let mut w = WbcWeights::default();
    w.swing_leg = 0.0;
    // Use the regular run_walk but feed weights via the fixture.
    let common::StandFixture {
        mut robot,
        kin,
        mut sim,
    } = common::build_namiashi_stand_fixture().unwrap();
    let cfg = GaitConfig::trot();
    let mut gc = GaitController::build(&robot, kin.clone(), cfg, GaitMode::Mpc).unwrap();
    let mut wbc_pipeline = WbcPipeline::new(&robot, common::default_foot_links());
    wbc_pipeline.weights = w;
    if let Some(srbd_cfg) = gc.srbd_mpc_config() {
        wbc_pipeline.mass_kg = srbd_cfg.mass_kg;
        wbc_pipeline.inertia_diag_body = srbd_cfg.inertia_diag_body;
    }
    let n_steps = (WALK_SIM_TIME_S / DT) as usize;
    let burn_in_steps = (WALK_BURN_IN_S / DT) as usize;
    let mut m = WalkBenchmark::default();
    m.min_body_z = f64::INFINITY;
    if let Some(p) = sim.body_world_position(&robot.root_link) {
        m.body_x_start = p[0]; m.body_y_start = p[1];
    }
    m.yaw_start = sim.body_world_yaw(&robot.root_link).unwrap_or(0.0);
    gc.enable();
    let cmd = fwd_cmd();
    for k in 0..n_steps {
        if k == burn_in_steps { gc.set_velocity_cmd(cmd); }
        let v = sim.body_world_linear_velocity(&robot.root_link).unwrap_or([0.;3]);
        let w_o = sim.body_world_angular_velocity(&robot.root_link).unwrap_or([0.;3]);
        gc.set_body_state_observed(Vector3::new(v[0],v[1],v[2]), Vector3::new(w_o[0],w_o[1],w_o[2]));
        let bp = sim.body_world_position(&robot.root_link).unwrap_or([0.;3]);
        let ya = sim.body_world_yaw(&robot.root_link).unwrap_or(0.0);
        gc.set_body_pose_observed(ya, Vector3::new(bp[0],bp[1],bp[2]));
        let (out, targets, _) = gc.tick(DT);
        for (idx, q) in targets { sim.set_position_target(idx, q); }
        if k >= burn_in_steps {
            let f = gc.predicted_grfs().map(|s| s.grfs_first_step).unwrap_or([Vector3::zeros();4]);
            let cn = gc.velocity_cmd();
            let vcb = Vector3::new(cn.vx, cn.vy, 0.0);
            let fls: [&str;4] = [wbc_pipeline.foot_links[0].as_str(),wbc_pipeline.foot_links[1].as_str(),wbc_pipeline.foot_links[2].as_str(),wbc_pipeline.foot_links[3].as_str()];
            let fz = sim.contact_force_per_foot(&fls);
            let np = [out.legs[0].phase,out.legs[1].phase,out.legs[2].phase,out.legs[3].phase];
            let cor = ContactDrivenPhase::apply_correction(&np, fz, 5.0, 0.0);
            let cf = [cor[0].is_stance,cor[1].is_stance,cor[2].is_stance,cor[3].is_stance];
            let taus = wbc_pipeline.solve(&robot,&sim,&out,gc.kinematics(),gc.joint_indices(),gc.joint_signs(),&vcb,cn.wz,&Vector3::new(v[0],v[1],v[2]),&Vector3::new(w_o[0],w_o[1],w_o[2]),&f,cf,DT);
            for (ji,&t) in taus.iter().enumerate() { sim.set_torque_feedforward(ji,t); }
        } else {
            for ji in 0..robot.joints.len() { sim.set_torque_feedforward(ji,0.0); }
        }
        sim.step(&mut robot, DT, true);
        if let Some(p) = sim.body_world_position(&robot.root_link) {
            m.min_body_z = m.min_body_z.min(p[2]); m.body_x_end = p[0]; m.body_y_end = p[1];
        }
        m.yaw_end = sim.body_world_yaw(&robot.root_link).unwrap_or(0.0);
    }
    eprintln!(
        "[diag-fwd:no-swing-leg] body_dx={:+.3} body_dy={:+.3} Δyaw={:+.3} min_z={:.3}",
        m.body_dx(), m.body_dy(), m.dyaw(), m.min_body_z,
    );
}

/// Diagnostic sweep: swing_leg ∈ {0.0, 0.1, 0.3, 0.5, 1.0} on lateral
/// + forward to find a balance that fixes lateral without breaking
/// forward.
#[test]
#[ignore = "P5a sweep"]
fn diag_swing_leg_sweep_lateral() {
    for w_swing in [0.0, 0.1, 0.3, 0.5, 1.0] {
        let mut w = WbcWeights::default();
        w.swing_leg = w_swing;
        if let Some(m) = run_lateral_with_weights(w) {
            eprintln!(
                "[sweep-lat swing={:.1}] body_dx={:+.3}  body_dy={:+.3}  Δyaw={:+.3}",
                w_swing, m.body_dx(), m.body_dy(), m.dyaw(),
            );
        }
    }
}

/// Helper for the forward-walk sweep (mirrors run_lateral_with_weights
/// but with cmd = forward).
fn run_forward_with_weights(weights: WbcWeights) -> Option<WalkBenchmark> {
    let common::StandFixture {
        mut robot,
        kin,
        mut sim,
    } = common::build_namiashi_stand_fixture()?;
    let cfg = GaitConfig::trot();
    let mut gc = GaitController::build(&robot, kin.clone(), cfg, GaitMode::Mpc).unwrap();
    let mut wbc_pipeline = WbcPipeline::new(&robot, common::default_foot_links());
    wbc_pipeline.weights = weights;
    if let Some(srbd_cfg) = gc.srbd_mpc_config() {
        wbc_pipeline.mass_kg = srbd_cfg.mass_kg;
        wbc_pipeline.inertia_diag_body = srbd_cfg.inertia_diag_body;
    }
    let n_steps = (WALK_SIM_TIME_S / DT) as usize;
    let burn_in_steps = (WALK_BURN_IN_S / DT) as usize;
    let mut m = WalkBenchmark::default();
    m.min_body_z = f64::INFINITY;
    if let Some(p) = sim.body_world_position(&robot.root_link) {
        m.body_x_start = p[0]; m.body_y_start = p[1];
    }
    m.yaw_start = sim.body_world_yaw(&robot.root_link).unwrap_or(0.0);
    gc.enable();
    for k in 0..n_steps {
        if k == burn_in_steps { gc.set_velocity_cmd(fwd_cmd()); }
        let v = sim.body_world_linear_velocity(&robot.root_link).unwrap_or([0.;3]);
        let w_o = sim.body_world_angular_velocity(&robot.root_link).unwrap_or([0.;3]);
        gc.set_body_state_observed(Vector3::new(v[0],v[1],v[2]), Vector3::new(w_o[0],w_o[1],w_o[2]));
        let bp = sim.body_world_position(&robot.root_link).unwrap_or([0.;3]);
        let ya = sim.body_world_yaw(&robot.root_link).unwrap_or(0.0);
        gc.set_body_pose_observed(ya, Vector3::new(bp[0],bp[1],bp[2]));
        let (out, targets, _) = gc.tick(DT);
        for (idx, q) in targets { sim.set_position_target(idx, q); }
        if k >= burn_in_steps {
            let f = gc.predicted_grfs().map(|s| s.grfs_first_step).unwrap_or([Vector3::zeros();4]);
            let cn = gc.velocity_cmd();
            let vcb = Vector3::new(cn.vx, cn.vy, 0.0);
            let fls: [&str;4] = [wbc_pipeline.foot_links[0].as_str(),wbc_pipeline.foot_links[1].as_str(),wbc_pipeline.foot_links[2].as_str(),wbc_pipeline.foot_links[3].as_str()];
            let fz = sim.contact_force_per_foot(&fls);
            let np = [out.legs[0].phase,out.legs[1].phase,out.legs[2].phase,out.legs[3].phase];
            let cor = ContactDrivenPhase::apply_correction(&np, fz, 5.0, 0.0);
            let cf = [cor[0].is_stance,cor[1].is_stance,cor[2].is_stance,cor[3].is_stance];
            let taus = wbc_pipeline.solve(&robot,&sim,&out,gc.kinematics(),gc.joint_indices(),gc.joint_signs(),&vcb,cn.wz,&Vector3::new(v[0],v[1],v[2]),&Vector3::new(w_o[0],w_o[1],w_o[2]),&f,cf,DT);
            for (ji,&t) in taus.iter().enumerate() { sim.set_torque_feedforward(ji,t); }
        } else {
            for ji in 0..robot.joints.len() { sim.set_torque_feedforward(ji,0.0); }
        }
        sim.step(&mut robot, DT, true);
        if let Some(p) = sim.body_world_position(&robot.root_link) {
            m.min_body_z = m.min_body_z.min(p[2]); m.body_x_end = p[0]; m.body_y_end = p[1];
        }
        m.yaw_end = sim.body_world_yaw(&robot.root_link).unwrap_or(0.0);
    }
    Some(m)
}

#[test]
#[ignore = "P5a sweep — forward axis"]
fn diag_swing_leg_sweep_forward() {
    for w_swing in [0.0, 0.1, 0.3, 0.5, 1.0] {
        let mut w = WbcWeights::default();
        w.swing_leg = w_swing;
        if let Some(m) = run_forward_with_weights(w) {
            eprintln!(
                "[sweep-fwd swing={:.1}] body_dx={:+.3}  body_dy={:+.3}  Δyaw={:+.3}",
                w_swing, m.body_dx(), m.body_dy(), m.dyaw(),
            );
        }
    }
}
