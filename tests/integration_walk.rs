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
    // the body didn't translate **backward** by more than a foot
    // length (that would indicate a sign-flip regression).
    assert!(
        metrics.body_x_delta() > -0.05,
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
