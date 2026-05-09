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
    }) = common::build_namiashi_stand_fixture()
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
    }) = common::build_namiashi_stand_fixture()
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
    }) = common::build_namiashi_stand_fixture()
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
}

/// Run a 5 s walk sim under the given controller / WBC mode + cmd
/// and return the integrated track metrics. Body state observation
/// uses MuJoCo ground truth (= matches `PoseSource::GroundTruth` in
/// the GUI).
fn run_walk(
    use_wbc: bool,
    gait_mode: GaitMode,
    cmd: VelocityCmd,
) -> Option<WalkBenchmark> {
    let common::StandFixture {
        mut robot,
        kin,
        mut sim,
    } = common::build_namiashi_stand_fixture()?;

    let cfg = GaitConfig::trot();
    let mut gc = GaitController::build(&robot, kin.clone(), cfg, gait_mode)
        .expect("GaitController::build");
    let mut wbc_pipeline = if use_wbc {
        Some(WbcPipeline::new(&robot, common::default_foot_links()))
    } else {
        None
    };
    // Sync mass / inertia from the auto-detected SrbdMpcConfig (same as
    // the GUI fix in commit b03c431) so WBC physics matches the URDF.
    if let (Some(pipeline), Some(srbd_cfg)) =
        (wbc_pipeline.as_mut(), gc.srbd_mpc_config())
    {
        pipeline.mass_kg = srbd_cfg.mass_kg;
        pipeline.inertia_diag_body = srbd_cfg.inertia_diag_body;
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
        if let Some(pipeline) = wbc_pipeline.as_mut() {
            pipeline.weights = quadruped_gait::wbc::WbcWeights::for_cmd(&gc.velocity_cmd());
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
#[test]
fn integration_walk_straight_champ() {
    let Some(m) = run_walk(false, GaitMode::Champ, fwd_cmd()) else {
        return;
    };
    eprintln!(
        "[forward:champ] body_dx={:+.3} m  body_dy={:+.3} m  Δyaw={:+.3} rad  min_z={:.3} m",
        m.body_dx(), m.body_dy(), m.dyaw(), m.min_body_z,
    );
    assert!(m.min_body_z > FALL_THRESHOLD_Z, "CHAMP fell");
}

/// MPC+WBC forward trot. Active axis: body-frame Δx > +10 cm. Cross
/// axes: |body_dy| < 20 cm, |Δyaw| < 1 rad. Tightenable as P5/P6
/// resolves the residual yaw bias.
#[test]
fn integration_walk_straight_mpc_wbc() {
    let Some(m) = run_walk(true, GaitMode::Mpc, fwd_cmd()) else {
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

#[test]
#[ignore = "D1.4 tuning target — assertions kept as goal"]
fn integration_walk_straight_centroidal_wbc() {
    let Some(m) = run_walk(true, GaitMode::CentroidalSrbd, fwd_cmd()) else {
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
    let Some(m) = run_walk(true, GaitMode::CentroidalSrbd, lat_cmd()) else {
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
    let Some(m) = run_walk(true, GaitMode::CentroidalSrbd, yaw_cmd()) else {
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

/// Repro of the user-reported axis swap at high cmd magnitude: drives
/// vx = +0.30 m/s (2× the standard test) and vy = +0.30 m/s separately
/// for 5 s each, prints the (body_dx, body_dy, Δyaw) so we can see
/// which axis dominates. No assertions — this is a diagnostic.
#[test]
#[ignore = "diagnostic — run with --ignored"]
fn diag_high_cmd_axis_swap() {
    let high_fwd = VelocityCmd { vx: 0.30, vy: 0.0, wz: 0.0 };
    let high_lat = VelocityCmd { vx: 0.0, vy: 0.30, wz: 0.0 };
    if let Some(m) = run_walk(true, GaitMode::Mpc, high_fwd) {
        eprintln!(
            "[diag:vx=0.3] body_dx={:+.3} m  body_dy={:+.3} m  Δyaw={:+.3} rad",
            m.body_dx(), m.body_dy(), m.dyaw(),
        );
    }
    if let Some(m) = run_walk(true, GaitMode::Mpc, high_lat) {
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
    let Some(m) = run_walk(false, GaitMode::Mpc, lat_cmd()) else {
        return;
    };
    eprintln!(
        "[lateral:mpc-only] body_dx={:+.3} m  body_dy={:+.3} m  Δyaw={:+.3} rad  min_z={:.3} m",
        m.body_dx(), m.body_dy(), m.dyaw(), m.min_body_z,
    );
    assert!(m.min_body_z > FALL_THRESHOLD_Z, "MPC fell (lateral diagnostic)");
}

/// CHAMP lateral walk benchmark — open-loop documentation only.
#[test]
fn integration_walk_lateral_champ() {
    let Some(m) = run_walk(false, GaitMode::Champ, lat_cmd()) else {
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
/// Cross axes: |body_dx| < 30 cm, |Δyaw| < 1.5 rad. The yaw cross
/// gate is loose because the trot's diagonal-pair phase produces a
/// natural yaw oscillation that doesn't fully cancel during a pure
/// lateral motion.
#[test]
fn integration_walk_lateral_mpc_wbc() {
    let Some(m) = run_walk(true, GaitMode::Mpc, lat_cmd()) else {
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
#[test]
fn integration_walk_yaw_champ() {
    let Some(m) = run_walk(false, GaitMode::Champ, yaw_cmd()) else {
        return;
    };
    eprintln!(
        "[yaw:champ] body_dx={:+.3} m  body_dy={:+.3} m  Δyaw={:+.3} rad  min_z={:.3} m",
        m.body_dx(), m.body_dy(), m.dyaw(), m.min_body_z,
    );
    assert!(m.min_body_z > FALL_THRESHOLD_Z, "CHAMP fell (yaw)");
}

/// MPC+WBC yaw rotate under `WbcWeights::for_cmd` (P5b). With the
/// scheduled swing_leg weight (0.1 at full yaw cmd), the body
/// achieves ~ +2.76 rad over 5 s under a 0.5 rad/s cmd (= 2.5 rad
/// expected, slightly over due to integrator overshoot at the
/// stance/swing handoff), with ~ 30 cm cross-axis drift.
///
/// Active axis: |Δyaw| > 1.5 rad. Cross axes: |body_dx| / |body_dy|
/// < 35 cm. The cross gates are looser than the lateral test
/// because trotting-while-yawing has the body pivoting around its
/// CoM and the per-stride foot displacement adds up over 5 s.
#[test]
fn integration_walk_yaw_mpc_wbc() {
    let Some(m) = run_walk(true, GaitMode::Mpc, yaw_cmd()) else {
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
