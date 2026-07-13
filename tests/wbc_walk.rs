//! End-to-end MuJoCo regression for the Hierarchical WBC pipeline.
//!
//! Mirrors [`gait_walk_stability`](./gait_walk_stability.rs)'s harness
//! (URDF → MujocoSim → trot loop) but routes the gait controller's
//! output through [`articara::wbc_pipeline::WbcPipeline`] instead of
//! the per-joint Position-PD path. Asserts on two invariants:
//!
//! 1. **Gravity balance** — at hold (zero command), the sum of
//!    contact-force z-components should approximately equal `m·g`.
//!    The WBC's friction-cone + EoM tasks are responsible for this:
//!    if the QP fails or hands over a wrong-sign GRF, the body would
//!    fall through the floor or float.
//! 2. **Forward motion** — under a steady forward command, the trunk
//!    must translate by at least `MIN_DISPLACEMENT_M` over the run.
//!    Catches the "WBC bobs in place" regression where the
//!    direction-preserving floor in `compute_mpc_footstep` collapses
//!    to zero or the WBC output is dominated by gravity comp.
//!
//! ## Known limitations (documented per-test)
//!
//! - The WBC pipeline keeps the misarta floating base at neutral
//!   orientation each tick (`q[3..7] = identity`). On flat ground
//!   that approximation is fine; large body tilt would skew the
//!   gravity-comp direction. We use mild commands so the body stays
//!   roughly upright.
//! - Friction enforcement happens in the WBC's QP, but the actual
//!   MuJoCo contact solver may still allow micro-slip. Thresholds
//!   are loose enough to absorb that.

#![cfg(feature = "mujoco")]

use std::path::PathBuf;

use articara::gait::{
    auto_detect_kinematics_config, GaitController, DEFAULT_FOOT_LINKS,
};
use articara::mjcf::{GroundPlaneCfg, MjcfExportOptions};
use articara::mujoco_sim::MujocoSim;
use articara::robot::RobotModel;
use articara::wbc_pipeline::WbcPipeline;
use nalgebra::Vector3;
use quadruped_gait::wbc;
use quadruped_gait::{
    solve_leg_ik, ContactDrivenPhase, GaitConfig, GaitMode, KinematicsConfig,
    LegIkSolution, VelocityCmd,
};

fn namiashi_misa() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("tests")
        .join("fixtures")
        .join("namiashi")
        .join("namiashi.misa")
}

/// Same seeding logic as `gait_walk_stability`. Keeps the legs out of
/// their q=0 fully-extended kinematic singularity at sim start.
fn seed_joint_positions_from_kinematics(
    robot: &mut RobotModel,
    kin: &KinematicsConfig,
) {
    for leg_kin in [&kin.fl, &kin.fr, &kin.rl, &kin.rr] {
        let target = leg_kin.nominal_foot_body;
        let sol = solve_leg_ik(leg_kin, target, false);
        let LegIkSolution::Reached { hip, thigh, calf } = sol else {
            panic!(
                "{:?}: nominal_foot_body unreachable",
                leg_kin.leg
            );
        };
        for (joint_name, q_ik, sign) in [
            (&leg_kin.hip_joint, hip, 1.0),
            (&leg_kin.thigh_joint, thigh, -1.0),
            (&leg_kin.calf_joint, calf, -1.0),
        ] {
            let Some(&ji) = robot.joint_map.get(joint_name.as_str()) else {
                continue;
            };
            robot.joint_positions[ji] = q_ik * sign;
        }
    }
}

/// Per-tick sample. We track `total_fz_world` (Σ contact-force z over
/// all contacts) so the static-balance test can compare it with
/// `m · g` after the burn-in window.
#[derive(Debug)]
struct WbcSample {
    t: f64,
    body_x: f64,
    body_z: f64,
    roll: f64,
    pitch: f64,
    total_fz_world: f64,
}

/// Threshold for "trunk has fallen". Below this z, the body has either
/// tipped over or sunk through the ground. Same value as
/// `gait_walk_stability`.
const TRUNK_Z_FALL_THRESHOLD_M: f64 = 0.18;

/// Minimum forward displacement during the walking window — the same
/// 4 cm threshold the gait stability test uses.
const MIN_DISPLACEMENT_M: f64 = 0.04;

struct WbcParams {
    total_time_s: f64,
    burn_in_s: f64,
    cmd_vx: f64,
    dt: f64,
    /// `None` = the legacy `wbc::solve_warm_with_weights` path
    /// (walk-validated default). `Some` opts the pipeline into the
    /// misa-wbc `Dynamics`-backed [`WbcSolver`] with the given
    /// formulation/strategy/backend — the equivalence-study switch
    /// documented in `ref/wbc_comparison.md`.
    misa_wbc_mode: Option<(wbc::Formulation, wbc::SolveConfig)>,
}

impl WbcParams {
    fn static_stand() -> Self {
        Self {
            total_time_s: 1.5,
            burn_in_s: 0.5,
            cmd_vx: 0.0,
            dt: 0.002,
            misa_wbc_mode: None,
        }
    }
    fn forward_walk() -> Self {
        Self {
            total_time_s: 3.0,
            burn_in_s: 0.5,
            cmd_vx: 0.15,
            dt: 0.002,
            misa_wbc_mode: None,
        }
    }

    /// Same schedule as [`Self::static_stand`], routed through
    /// `WbcSolver` with the given formulation/strategy/backend.
    fn static_stand_misa_wbc(formulation: wbc::Formulation, cfg: wbc::SolveConfig) -> Self {
        Self { misa_wbc_mode: Some((formulation, cfg)), ..Self::static_stand() }
    }

    /// Same schedule as [`Self::forward_walk`], routed through
    /// `WbcSolver` with the given formulation/strategy/backend.
    fn forward_walk_misa_wbc(formulation: wbc::Formulation, cfg: wbc::SolveConfig) -> Self {
        Self { misa_wbc_mode: Some((formulation, cfg)), ..Self::forward_walk() }
    }
}

/// Run a WBC sim, sampling per-tick. Returns `None` if the namiashi
/// fixture is missing (skip cleanly).
fn run_wbc_sim(params: WbcParams) -> Option<Vec<WbcSample>> {
    let path = namiashi_misa();
    if !path.exists() {
        eprintln!(
            "namiashi fixture missing at {} — skipping WBC test",
            path.display()
        );
        return None;
    }
    // Load from the master .misa so PD gains (kp=100, kv=1.2) and
    // joint damping (0.1) come from the same source the GUI uses; the
    // burn-in Position-PD then settles at the real GUI body-z. See
    // commit `f53482c` for the same migration in gait_walk_stability.
    let mut robot = RobotModel::from_misa(&path).expect("load namiashi .misa");

    let mut kin = auto_detect_kinematics_config(&robot, &DEFAULT_FOOT_LINKS)
        .expect("auto-detect kinematics");
    for leg_kin in [&mut kin.fl, &mut kin.fr, &mut kin.rl, &mut kin.rr] {
        let total_leg = leg_kin.upper_leg_m + leg_kin.lower_leg_m;
        leg_kin.nominal_foot_body.z += 0.08 * total_leg;
    }
    seed_joint_positions_from_kinematics(&mut robot, &kin);

    let opts = MjcfExportOptions {
        ground_plane: Some(GroundPlaneCfg {
            z: 0.0,
            half_size: 4.0,
            roll: 0.0,
            pitch: 0.0,
        }),
        add_actuators: true,
        ..Default::default()
    };
    let mut sim = MujocoSim::new(&robot, opts).expect("MujocoSim::new");
    // We don't want gravity-comp to compete with the WBC during the
    // walking window — the WBC's floating-base EoM task already
    // handles it. But during burn-in (gait disabled, WBC inactive),
    // grav-comp keeps the body from sagging. Toggle is per-sim, so
    // we leave it on; the WBC bypasses the per-joint path entirely
    // when active.
    sim.set_gravity_compensation(true);

    let cfg = GaitConfig::trot();
    let mut gc = GaitController::build(&robot, kin.clone(), cfg, GaitMode::Mpc)
        .expect("GaitController::build (Mpc mode)");

    // Foot link names for the WBC pipeline.
    let foot_links: [String; 4] = [
        DEFAULT_FOOT_LINKS[0].1.to_string(),
        DEFAULT_FOOT_LINKS[1].1.to_string(),
        DEFAULT_FOOT_LINKS[2].1.to_string(),
        DEFAULT_FOOT_LINKS[3].1.to_string(),
    ];
    let mut wbc_pipeline = WbcPipeline::new(&robot, foot_links);
    if let Some((formulation, cfg)) = params.misa_wbc_mode.clone() {
        wbc_pipeline = wbc_pipeline.with_wbc_solver(formulation, cfg);
    }

    let n_steps = (params.total_time_s / params.dt).round() as usize;
    let burn_in_steps = (params.burn_in_s / params.dt).round() as usize;
    let mut samples: Vec<WbcSample> = Vec::with_capacity(n_steps);

    for k in 0..n_steps {
        let t = k as f64 * params.dt;

        if k == 0 {
            gc.enable();
        }
        if k == burn_in_steps {
            gc.set_velocity_cmd(VelocityCmd {
                vx: params.cmd_vx,
                vy: 0.0,
                wz: 0.0,
            });
        }

        // Feed observed body velocity to the closed-loop generators.
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

        if gc.is_enabled() {
            let (out, targets, torque_ff) = gc.tick(params.dt);
            for (idx, q) in targets {
                sim.set_position_target(idx, q);
            }
            // After burn-in: route through WBC. Skip during burn-in
            // so the body has a chance to settle on its feet via the
            // Position-PD path (the WBC's static balance only
            // converges once the legs are loaded).
            if k >= burn_in_steps {
                let f_grf_world = gc
                    .predicted_grfs()
                    .map(|sol| sol.grfs_first_step)
                    .unwrap_or([Vector3::zeros(); 4]);
                let cmd = gc.velocity_cmd();
                // Body-frame command — the WBC pipeline rotates the
                // observation internally using the current xquat.
                let v_cmd_body = Vector3::new(cmd.vx, cmd.vy, 0.0);
                // Contact-driven phase correction (Phase C). Read the
                // per-foot ground reaction force from MuJoCo and apply
                // ContactDrivenPhase's stateless override to the
                // gait controller's nominal phases. This catches early
                // touchdown / late liftoff before the WBC's
                // no_contact_motion task assumes the wrong contact
                // pattern, which would otherwise destabilise the body
                // during trotting (stance=2/4).
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
                    /* early_contact_threshold_n = */ 5.0,
                    // late_liftoff disabled (= 0 N): if every foot is
                    // momentarily unloaded during a transient body fall,
                    // a non-zero threshold would flip ALL legs to swing
                    // and there would be no support at all → unrecoverable
                    // collapse. Early touchdown is the more important
                    // direction anyway; late liftoff matters mainly for
                    // slip detection on real hardware.
                    /* late_liftoff_threshold_n = */ 0.0,
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
                    cmd.wz,
                    &Vector3::new(v_obs[0], v_obs[1], v_obs[2]),
                    &Vector3::new(w_obs[0], w_obs[1], w_obs[2]),
                    &f_grf_world,
                    contact_flag,
                    params.dt,
                );
                // Diagnostic dump every 100 ticks (200 ms). Detailed
                // breakdown for the G-step (Phase 1.5 / forward walk)
                // root-cause hunt: per-leg q*-vs-q tracking error,
                // swing target vs measured foot position, MPC f_GRF
                // reference vs WBC sol.f_grf, and the body-frame
                // foot offsets. Tab-aligned so a regression diff
                // shows column-by-column.
                if k % 100 == 0 {
                    let body_pos = sim
                        .body_world_position(&robot.root_link)
                        .unwrap_or([0.0, 0.0, 0.0]);
                    let tau_max = taus
                        .iter()
                        .cloned()
                        .fold(0.0_f64, |a, b| a.max(b.abs()));
                    let mpc_fz_sum: f64 = f_grf_world.iter().map(|v| v.z).sum();
                    let stance_count = contact_flag.iter().filter(|b| **b).count();
                    eprintln!(
                        "[diag k={k:5} t={:.3}s] z={:.3} m  Σmpc_f_z={:.2} N  \
                         max|τ|={:.2} N·m  stance={}/4",
                        k as f64 * params.dt,
                        body_pos[2],
                        mpc_fz_sum,
                        tau_max,
                        stance_count
                    );
                    // Per-leg tracking detail. `targets` carries
                    // (joint_idx, q_target_urdf); compare against
                    // mj_sim.joint_q_qd to see how well Position-PD
                    // is following.
                    let mut q_err_max = 0.0_f64;
                    let mut qd_err_max = 0.0_f64;
                    for (ji, q_target) in targets.iter() {
                        if let Some((q_actual, qd_actual)) =
                            sim.joint_q_qd(&robot.joints[*ji].name)
                        {
                            let q_err = (*q_target - q_actual).abs();
                            q_err_max = q_err_max.max(q_err);
                            qd_err_max = qd_err_max.max(qd_actual.abs());
                        }
                    }
                    // WBC solution breakdown (cached on the pipeline).
                    let (wbc_fz_sum, q_ddot_z, tau_norm) =
                        if let Some(sol) = wbc_pipeline.last_solution.as_ref() {
                            let fz: f64 =
                                (0..4).map(|s| sol.f_grf[3 * s + 2]).sum();
                            let q_ddot_z = sol.q_ddot[5]; // body z (Featherstone [ang;lin])
                            let tau_norm: f64 = sol.tau.iter().map(|x| x * x).sum::<f64>().sqrt();
                            (fz, q_ddot_z, tau_norm)
                        } else {
                            (0.0, 0.0, 0.0)
                        };
                    // Per-leg swing target vs measured: only meaningful
                    // when the leg is in nominal swing, but compute
                    // for all 4 so the dump is uniform width.
                    let mut swing_err_max = 0.0_f64;
                    for slot in 0..4 {
                        if !out.legs[slot].phase.is_stance {
                            // foot_body target (body frame).
                            let target_body = out.legs[slot].foot_body;
                            // Measured via FK on the actual joint q.
                            let leg_kin = gc.kinematics().legs()[slot];
                            let mut q_leg = [0.0_f64; 3];
                            let signs = gc.joint_signs()[slot];
                            for kk in 0..3 {
                                let ji = gc.joint_indices()[slot][kk];
                                if let Some((q, _)) =
                                    sim.joint_q_qd(&robot.joints[ji].name)
                                {
                                    q_leg[kk] = signs[kk] * q;
                                }
                            }
                            let measured_body =
                                quadruped_gait::forward_leg_kinematics(
                                    leg_kin, q_leg[0], q_leg[1], q_leg[2],
                                );
                            let err = (target_body - measured_body).norm();
                            swing_err_max = swing_err_max.max(err);
                        }
                    }
                    eprintln!(
                        "[diag-detail k={k:5}] q*-q max={:.4} rad   q̇ max={:.3} rad/s   \
                         WBC Σf_z={:.2} N  q̈_z={:.2}  ‖τ‖={:.2}  swing FK err={:.4} m",
                        q_err_max, qd_err_max,
                        wbc_fz_sum, q_ddot_z, tau_norm, swing_err_max,
                    );
                    // MPC vs WBC f_GRF per-leg comparison (z-component
                    // only; horizontal components omitted to keep the
                    // line readable).
                    if let Some(sol) = wbc_pipeline.last_solution.as_ref() {
                        let mpc_per_leg: [f64; 4] =
                            std::array::from_fn(|s| f_grf_world[s].z);
                        let wbc_per_leg: [f64; 4] =
                            std::array::from_fn(|s| sol.f_grf[3 * s + 2]);
                        eprintln!(
                            "[diag-grf  k={k:5}] MPC_f_z=[{:.1} {:.1} {:.1} {:.1}]   \
                             WBC_f_z=[{:.1} {:.1} {:.1} {:.1}]",
                            mpc_per_leg[0], mpc_per_leg[1], mpc_per_leg[2], mpc_per_leg[3],
                            wbc_per_leg[0], wbc_per_leg[1], wbc_per_leg[2], wbc_per_leg[3],
                        );
                    }
                }
                // Hybrid-joint command (legged_control style):
                // Position-PD already runs against the gait controller's
                // q* (set_position_target above), so WBC τ goes in as
                // feedforward — NOT as a full PD bypass.
                //
                // This is the single biggest difference from the previous
                // `set_wbc_torques` path: the Position-PD is what actually
                // drives the joints to track the gait controller's q*,
                // while WBC's τ_ff adds the dynamic / contact-force
                // contribution on top. Without this hybrid scheme, WBC's
                // long-term tracking error accumulates because the QP
                // produces accelerations not positions, and there's no
                // integrator to drive joint-position drift back to zero.
                // Hybrid joint command: Position-PD tracks q*, WBC τ
                // adds dynamic + gravity + contact-force compensation
                // on top. The WBC tries to produce τ such that the QP
                // solution's f_GRF matches the MPC's predicted GRFs
                // (= forward thrust included), but the actual tracking
                // depends on `W_CONTACT_FORCE` in `quadruped_gait::wbc`.
                let _ = torque_ff; // discard MPC ff — WBC owns the τ stream
                for (ji, &tau) in taus.iter().enumerate() {
                    sim.set_torque_feedforward(ji, tau);
                }
                sim.clear_wbc_torques();
            } else {
                sim.clear_wbc_torques();
                // No WBC active during burn-in — clear any feedforward
                // torque so the per-joint PD path runs cleanly.
                for ji in 0..robot.joints.len() {
                    sim.set_torque_feedforward(ji, 0.0);
                }
            }
        }

        sim.step(&mut robot, params.dt, true);

        // Sample after the step so contact forces and pose are
        // synchronised. `base_transform` was just refreshed by
        // `sim.step → sync_back`.
        let tx = robot.base_transform.translation;
        let (roll, pitch, _yaw) = robot.base_transform.rotation.euler_angles();
        let total_fz_world: f64 =
            sim.contacts().iter().map(|c| c.force_world[2]).sum();
        samples.push(WbcSample {
            t,
            body_x: tx.x,
            body_z: tx.z,
            roll,
            pitch,
            total_fz_world,
        });
    }
    Some(samples)
}

/// Compute the total robot mass by summing every link's `mass` field.
/// `m·g` is the reference for the static-balance assertion.
fn robot_mass(robot: &RobotModel) -> f64 {
    robot.links.iter().map(|l| l.inertial.mass).sum()
}

/// Static stand under WBC torque control. The Phase A integration
/// went through several rounds of tightening before this could pass:
///
/// 1. `q[3..7]` quaternion sync from MuJoCo (resolved at adbc9f1).
/// 2. SE(3)-correct `compute_joint_jacobian_time_derivative` so a
///    moving floating base doesn't blow up `J̇·v` (Phase 1.2).
/// 3. Per-task LSQ weights so EoM/no_contact_motion dominate
///    contact_force/τ_grav (Phase 1.4).
/// 4. `a_base_des` derived from the **MPC's predicted GRFs** via
///    SRBD physics, instead of a hand-tuned PD on body velocity
///    (Phase 1.5-A) — this is the single biggest fix; without it
///    the WBC drives the body to a different equilibrium than the
///    MPC chose and the two fight to body collapse.
///
/// The avg Σf_z tolerance is ±60% rather than the ideal ±10% because
/// MuJoCo's contact bouncing produces large per-tick swings even when
/// the body is mean-stable around the target z. The *min_z fall*
/// check catches the real failure mode (body slumping below 0.18 m).
#[test]
fn wbc_static_stand_balances_gravity() {
    let Some(samples) = run_wbc_sim(WbcParams::static_stand()) else {
        return;
    };
    assert_static_stand_balances_gravity(&samples);
}

/// Same invariants as [`wbc_static_stand_balances_gravity`], but
/// routed through the misa-wbc `Dynamics` path in the `ForceSpace`
/// formulation (GID's decision-variable layout, `x = [τ; f]`) with
/// the `ActiveSet` backend — the fastest combination per the
/// `misa-wbc` formulation benchmarks (see `ref/wbc_comparison.md`).
/// Confirms the equivalence holds under real MuJoCo contact dynamics,
/// not just on synthetic matrices.
#[test]
fn wbc_static_stand_balances_gravity_force_space_active_set() {
    let cfg = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
    let Some(samples) =
        run_wbc_sim(WbcParams::static_stand_misa_wbc(wbc::Formulation::ForceSpace, cfg))
    else {
        return;
    };
    assert_static_stand_balances_gravity(&samples);
}

fn assert_static_stand_balances_gravity(samples: &[WbcSample]) {
    // No fall.
    let min_z = samples.iter().map(|s| s.body_z).fold(f64::INFINITY, f64::min);
    assert!(
        min_z > TRUNK_Z_FALL_THRESHOLD_M,
        "static stand: trunk fell, min_z = {min_z:.3} m (threshold {:.2})",
        TRUNK_Z_FALL_THRESHOLD_M,
    );

    // Burn-in window done; sample the last 0.5 s for the f_z average.
    let dt: f64 = 0.002;
    let total_time = 1.5;
    let burn_in = 0.5;
    let total_n = (total_time / dt).round() as usize;
    let window_n = (0.5 / dt).round() as usize;
    let start = total_n.saturating_sub(window_n);
    let avg_fz: f64 = samples[start..]
        .iter()
        .map(|s| s.total_fz_world)
        .sum::<f64>()
        / (samples.len() - start) as f64;

    // Reference m·g. Recompute by reloading the .misa (cheap).
    let path = namiashi_misa();
    let robot = RobotModel::from_misa(&path).unwrap();
    let mg = robot_mass(&robot) * 9.81;

    let pct_err = ((avg_fz - mg) / mg).abs();
    eprintln!(
        "[wbc] static_stand: avg Σf_z = {avg_fz:.2} N, m·g = {mg:.2} N \
         (err = {:.1}%)",
        pct_err * 100.0,
    );
    // ±60% tolerance — generous because:
    //   - per-tick MuJoCo contact force has large bouncing components
    //     (the integrator's substeps see sharp normal-force spikes
    //     around touchdown which the 0.5 s sample window doesn't fully
    //     average out),
    //   - friction-cone clipping in the QP can re-allocate force
    //     among feet, making instantaneous totals oscillate,
    //   - the WBC's static-stand z drifts within a few cm because
    //     there's no explicit z-position regulator (the SRBD MPC's
    //     z weight handles it but its 30 ms horizon discretisation
    //     leaves residual oscillation).
    // The accompanying min_z assertion (above) is the real fall
    // detector; this avg-Σf_z test mostly guards against pathological
    // gravity-direction errors.
    assert!(
        pct_err < 0.60,
        "static stand: Σf_z = {avg_fz:.2} N deviates from m·g = {mg:.2} N \
         by {:.1}% — friction cone + EoM may be inconsistent",
        pct_err * 100.0,
    );
    let _ = burn_in;
}

/// Forward walk under WBC + Position-PD hybrid joint command.
/// Asserts the trunk advances at least `MIN_DISPLACEMENT_M` over the
/// run without falling (`min_z > TRUNK_Z_FALL_THRESHOLD_M`).
///
/// The test sequence — MPC-driven `a_base_des` (Phase 1.5-A), the
/// joint-space swing_leg task (G2, sharing q* with Position-PD), and
/// `W_CONTACT_FORCE = 5` (H4 tuning, lets the WBC track MPC's
/// predicted GRF tightly enough that stance forward thrust flows
/// into joint torque) — together produce ~17 cm of forward
/// displacement under a 0.15 m/s command over 2.5 s.
#[test]
fn wbc_forward_command_advances_body() {
    let Some(samples) = run_wbc_sim(WbcParams::forward_walk()) else {
        return;
    };
    assert_forward_command_advances_body(&samples);
}

/// Same invariants as [`wbc_forward_command_advances_body`], routed
/// through misa-wbc's `ForceSpace` + `ActiveSet` — see
/// [`wbc_static_stand_balances_gravity_force_space_active_set`] for
/// the rationale.
#[test]
fn wbc_forward_command_advances_body_force_space_active_set() {
    let cfg = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
    let Some(samples) =
        run_wbc_sim(WbcParams::forward_walk_misa_wbc(wbc::Formulation::ForceSpace, cfg))
    else {
        return;
    };
    assert_forward_command_advances_body(&samples);
}

fn assert_forward_command_advances_body(samples: &[WbcSample]) {
    // Skip burn-in for tilt + displacement metrics.
    let dt: f64 = 0.002;
    let burn_in_steps = (0.5 / dt).round() as usize;
    let walk = &samples[burn_in_steps..];

    // No fall during the entire run (including burn-in).
    let min_z = samples.iter().map(|s| s.body_z).fold(f64::INFINITY, f64::min);
    assert!(
        min_z > TRUNK_Z_FALL_THRESHOLD_M,
        "forward walk: trunk fell, min_z = {min_z:.3} m",
    );

    // Forward displacement during the walking window.
    let x_start = walk.first().map(|s| s.body_x).unwrap_or(0.0);
    let x_end = walk.last().map(|s| s.body_x).unwrap_or(0.0);
    let dx = x_end - x_start;
    eprintln!(
        "[wbc] forward_command: Δx = {dx:.3} m over {:.1} s \
         (threshold ≥ {:.2})",
        walk.last().map(|s| s.t).unwrap_or(0.0)
            - walk.first().map(|s| s.t).unwrap_or(0.0),
        MIN_DISPLACEMENT_M,
    );
    assert!(
        dx >= MIN_DISPLACEMENT_M,
        "forward walk: Δx = {dx:.3} m < {} m — WBC produced near-zero \
         net forward motion (足踏み regression?)",
        MIN_DISPLACEMENT_M,
    );

    // Bounded body tilt during walking. WBC + Position-PD with
    // gravity-comp should keep the trunk roughly level on flat ground.
    let peak_roll = walk
        .iter()
        .map(|s| s.roll.abs())
        .fold(0.0_f64, f64::max);
    let peak_pitch = walk
        .iter()
        .map(|s| s.pitch.abs())
        .fold(0.0_f64, f64::max);
    eprintln!(
        "[wbc] forward_command: peak |roll| = {peak_roll:.2} rad, \
         peak |pitch| = {peak_pitch:.2} rad",
    );
    // 0.5 rad ≈ 30° — generous because WBC q[3..7] = identity means
    // tilt feedback is approximate.
    assert!(
        peak_roll < 0.5,
        "forward walk: |roll| peak {peak_roll:.2} rad too large",
    );
    assert!(
        peak_pitch < 0.5,
        "forward walk: |pitch| peak {peak_pitch:.2} rad too large",
    );
}
