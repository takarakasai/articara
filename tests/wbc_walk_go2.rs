//! End-to-end MuJoCo regression for the Hierarchical WBC pipeline on
//! the **real Go2 model** (`models/unitree_go2/go2.misa`), not the
//! lightweight `namiashi` fixture [`wbc_walk`](./wbc_walk.rs) uses.
//!
//! Same harness and invariants as `wbc_walk.rs` (static gravity
//! balance, forward displacement under a velocity command) — this
//! file exists to answer a question `wbc_walk.rs` alone can't:
//! **does misa-wbc's WBC solve path actually walk the genuine Go2
//! model (real ~15.6 kg mass, real joint limits/actuator gains) in
//! MuJoCo, not just a small synthetic quadruped?** `go2.misa` already
//! carries real Kp=500/Kv=5 actuator gains and per-joint torque
//! limits (hip/thigh 23.7 N·m, calf 45.43 N·m) — no manual retuning
//! duplicated here; `GaitController::build` auto-detects the SRBD
//! MPC's `mass_kg` from the model too (`articara::gait::
//! auto_detect_srbd_mpc_config`), so this is the *same* code path as
//! `wbc_walk.rs`, just pointed at a different, real model.
//!
//! ## Known limitations (see `wbc_walk.rs` for the shared ones)
//!
//! - Home pose (hip=0, thigh=0.9, calf=-1.8) and initial base height
//!   (0.30 m) match the existing `examples/go2_crawl.rs` /
//!   `examples/go2_sim.rs` conventions for this model.
//! - Go2 is ~6x heavier than `namiashi` — thresholds below are Go2-
//!   specific, not copy-pasted from `wbc_walk.rs`.

#![cfg(feature = "mujoco")]

use std::path::PathBuf;

use articara::gait::{auto_detect_kinematics_config, GaitController, DEFAULT_FOOT_LINKS};
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

fn go2_misa() -> PathBuf {
    // Sibling to articara/tests/ -- the model lives at the repo root,
    // not under tests/fixtures/ (it's a submodule shared with
    // go2-gait-runner, not a test-only asset).
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("models")
        .join("unitree_go2")
        .join("go2.misa")
}

/// Same idea as `wbc_walk.rs`'s namiashi seeding: start each leg at
/// its nominal IK solution rather than q=0 (hip=0, thigh=0, calf=0 is
/// Go2's near-fully-extended kinematic singularity -- see this
/// session's `go2_leg_singularity_demo` D6 CBF work).
fn seed_joint_positions_from_kinematics(robot: &mut RobotModel, kin: &KinematicsConfig) {
    for leg_kin in [&kin.fl, &kin.fr, &kin.rl, &kin.rr] {
        let target = leg_kin.nominal_foot_body;
        let sol = solve_leg_ik(leg_kin, target, false);
        let LegIkSolution::Reached { hip, thigh, calf } = sol else {
            panic!("{:?}: nominal_foot_body unreachable", leg_kin.leg);
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

#[derive(Debug)]
struct WbcSample {
    t: f64,
    body_x: f64,
    body_z: f64,
    roll: f64,
    pitch: f64,
    total_fz_world: f64,
}

/// Go2's real standing height is ~0.30 m; a collapse/faceplant drops
/// well below this. Looser than namiashi's 0.18 m floor only because
/// Go2 starts taller, not because the bar is lower in relative terms
/// (0.15 m ≈ 50% of nominal height, same ratio `wbc_walk.rs` uses).
const TRUNK_Z_FALL_THRESHOLD_M: f64 = 0.15;

/// Go2 is heavier and its default trot cadence/step length differ
/// from namiashi's tuning, so this threshold is deliberately loose —
/// the point of this test is "does it walk at all", not a precise
/// speed-tracking check.
const MIN_DISPLACEMENT_M: f64 = 0.03;

struct WbcParams {
    total_time_s: f64,
    burn_in_s: f64,
    cmd_vx: f64,
    dt: f64,
    misa_wbc_mode: Option<(wbc::Formulation, wbc::SolveConfig)>,
    /// When set, `cmd_vx` is ignored and the commanded forward
    /// velocity instead steps through `0.0, 0.5, 1.0, … 5.0 m/s`
    /// (`STAIRCASE_STEP_MPS`), holding each level for this many
    /// seconds — a stress test to find where Trot+WBC+MPC stops
    /// tracking cleanly, not a pass/fail regression check.
    staircase_step_s: Option<f64>,
}

/// Velocity increment per staircase level (m/s) — see `staircase_step_s`.
const STAIRCASE_STEP_MPS: f64 = 0.5;
/// Top commanded speed (m/s); `STAIRCASE_STEP_MPS` apart gives 11 levels
/// (0.0..=5.0).
const STAIRCASE_MAX_MPS: f64 = 5.0;

impl WbcParams {
    fn static_stand() -> Self {
        Self {
            total_time_s: 1.5, burn_in_s: 0.5, cmd_vx: 0.0, dt: 0.002,
            misa_wbc_mode: None, staircase_step_s: None,
        }
    }
    fn forward_walk() -> Self {
        Self {
            total_time_s: 3.0, burn_in_s: 0.5, cmd_vx: 0.15, dt: 0.002,
            misa_wbc_mode: None, staircase_step_s: None,
        }
    }
    fn static_stand_misa_wbc(formulation: wbc::Formulation, cfg: wbc::SolveConfig) -> Self {
        Self { misa_wbc_mode: Some((formulation, cfg)), ..Self::static_stand() }
    }
    fn forward_walk_misa_wbc(formulation: wbc::Formulation, cfg: wbc::SolveConfig) -> Self {
        Self { misa_wbc_mode: Some((formulation, cfg)), ..Self::forward_walk() }
    }

    /// 30s staircase, 0 to 5 m/s in 0.5 m/s steps (11 levels, ~2.73s
    /// each), routed through misa-wbc's ForceSpace+ActiveSet.
    fn velocity_staircase_misa_wbc(formulation: wbc::Formulation, cfg: wbc::SolveConfig) -> Self {
        let total_time_s = 30.0;
        let n_levels = (STAIRCASE_MAX_MPS / STAIRCASE_STEP_MPS).round() as usize + 1;
        Self {
            total_time_s,
            burn_in_s: 0.0, // level 0 (vx=0) already acts as the settle window
            cmd_vx: 0.0,    // unused; staircase_step_s drives the command instead
            dt: 0.002,
            misa_wbc_mode: Some((formulation, cfg)),
            staircase_step_s: Some(total_time_s / n_levels as f64),
        }
    }
}

fn run_wbc_sim(params: WbcParams) -> Option<Vec<WbcSample>> {
    let path = go2_misa();
    if !path.exists() {
        eprintln!("go2.misa missing at {} — skipping Go2 WBC test", path.display());
        return None;
    }
    let mut robot = RobotModel::from_misa(&path).expect("load go2.misa");

    let mut kin = auto_detect_kinematics_config(&robot, &DEFAULT_FOOT_LINKS)
        .expect("auto-detect kinematics");
    for leg_kin in [&mut kin.fl, &mut kin.fr, &mut kin.rl, &mut kin.rr] {
        let total_leg = leg_kin.upper_leg_m + leg_kin.lower_leg_m;
        leg_kin.nominal_foot_body.z += 0.08 * total_leg;
    }
    seed_joint_positions_from_kinematics(&mut robot, &kin);

    let opts = MjcfExportOptions {
        base_pos: Some([0.0, 0.0, 0.30]),
        ground_plane: Some(GroundPlaneCfg { z: 0.0, half_size: 4.0, roll: 0.0, pitch: 0.0 }),
        add_actuators: true,
        ..Default::default()
    };
    let mut sim = MujocoSim::new(&robot, opts).expect("MujocoSim::new");
    sim.set_gravity_compensation(true);

    let cfg = GaitConfig::trot();
    let mut gc = GaitController::build(&robot, kin.clone(), cfg, GaitMode::Mpc)
        .expect("GaitController::build (Mpc mode)");

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

    // Optional per-tick link-pose CSV export for external video
    // rendering (visualization side-channel only, no effect on the
    // pass/fail assertions below). Every body MuJoCo actually
    // simulates -- base + all 4 legs' hip/thigh/calf/foot -- queried
    // by name directly from MuJoCo (sidesteps needing to know
    // misarta's floating-base joint indexing at all).
    const LINK_NAMES: [&str; 17] = [
        "base",
        "FL_hip", "FL_thigh", "FL_calf", "FL_foot",
        "FR_hip", "FR_thigh", "FR_calf", "FR_foot",
        "RL_hip", "RL_thigh", "RL_calf", "RL_foot",
        "RR_hip", "RR_thigh", "RR_calf", "RR_foot",
    ];
    let csv_out = std::env::var("WBC_WALK_CSV_OUT").ok();
    let mut csv_buf = String::new();
    if csv_out.is_some() {
        csv_buf.push_str("tick,t");
        for name in LINK_NAMES {
            csv_buf.push_str(&format!(
                ",{name}_tx,{name}_ty,{name}_tz,\
                 {name}_r00,{name}_r01,{name}_r02,\
                 {name}_r10,{name}_r11,{name}_r12,\
                 {name}_r20,{name}_r21,{name}_r22"
            ));
        }
        csv_buf.push('\n');
    }

    let n_steps = (params.total_time_s / params.dt).round() as usize;
    let burn_in_steps = (params.burn_in_s / params.dt).round() as usize;
    let mut samples: Vec<WbcSample> = Vec::with_capacity(n_steps);

    let mut last_staircase_level: Option<usize> = None;
    for k in 0..n_steps {
        let t = k as f64 * params.dt;

        if k == 0 {
            gc.enable();
        }
        if let Some(step_s) = params.staircase_step_s {
            let n_levels = (STAIRCASE_MAX_MPS / STAIRCASE_STEP_MPS).round() as usize + 1;
            let level = ((t / step_s) as usize).min(n_levels - 1);
            if last_staircase_level != Some(level) {
                let vx = level as f64 * STAIRCASE_STEP_MPS;
                gc.set_velocity_cmd(VelocityCmd { vx, vy: 0.0, wz: 0.0 });
                eprintln!("[staircase] t={t:6.2}s level={level:2} cmd_vx={vx:.1} m/s");
                last_staircase_level = Some(level);
            }
        } else if k == burn_in_steps {
            gc.set_velocity_cmd(VelocityCmd { vx: params.cmd_vx, vy: 0.0, wz: 0.0 });
        }

        let v_obs = sim.body_world_linear_velocity(&robot.root_link).unwrap_or([0.0, 0.0, 0.0]);
        let w_obs = sim.body_world_angular_velocity(&robot.root_link).unwrap_or([0.0, 0.0, 0.0]);
        gc.set_body_state_observed(
            Vector3::new(v_obs[0], v_obs[1], v_obs[2]),
            Vector3::new(w_obs[0], w_obs[1], w_obs[2]),
        );

        if gc.is_enabled() {
            let (out, targets, torque_ff) = gc.tick(params.dt);
            for (idx, q) in targets {
                sim.set_position_target(idx, q);
            }
            if k >= burn_in_steps {
                let f_grf_world =
                    gc.predicted_grfs().map(|sol| sol.grfs_first_step).unwrap_or([Vector3::zeros(); 4]);
                let cmd = gc.velocity_cmd();
                let v_cmd_body = Vector3::new(cmd.vx, cmd.vy, 0.0);
                let foot_links_str: [&str; 4] = [
                    wbc_pipeline.foot_links[0].as_str(),
                    wbc_pipeline.foot_links[1].as_str(),
                    wbc_pipeline.foot_links[2].as_str(),
                    wbc_pipeline.foot_links[3].as_str(),
                ];
                let force_z = sim.contact_force_per_foot(&foot_links_str);
                let nominal_phases =
                    [out.legs[0].phase, out.legs[1].phase, out.legs[2].phase, out.legs[3].phase];
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
                    cmd.wz,
                    &Vector3::new(v_obs[0], v_obs[1], v_obs[2]),
                    &Vector3::new(w_obs[0], w_obs[1], w_obs[2]),
                    &f_grf_world,
                    contact_flag,
                    params.dt,
                );
                if k % 250 == 0 {
                    let body_pos = sim.body_world_position(&robot.root_link).unwrap_or([0.0, 0.0, 0.0]);
                    let tau_max = taus.iter().cloned().fold(0.0_f64, |a, b| a.max(b.abs()));
                    let mpc_fz_sum: f64 = f_grf_world.iter().map(|v| v.z).sum();
                    let stance_count = contact_flag.iter().filter(|b| **b).count();
                    eprintln!(
                        "[diag k={k:5} t={:.3}s] z={:.3} m  Σmpc_f_z={:.2} N  max|τ|={:.2} N·m  stance={}/4",
                        k as f64 * params.dt, body_pos[2], mpc_fz_sum, tau_max, stance_count,
                    );
                }
                let _ = torque_ff;
                for (ji, &tau) in taus.iter().enumerate() {
                    sim.set_torque_feedforward(ji, tau);
                }
                sim.clear_wbc_torques();
            } else {
                sim.clear_wbc_torques();
                for ji in 0..robot.joints.len() {
                    sim.set_torque_feedforward(ji, 0.0);
                }
            }
        }

        sim.step(&mut robot, params.dt, true);

        let tx = robot.base_transform.translation;
        let (roll, pitch, _yaw) = robot.base_transform.rotation.euler_angles();
        let total_fz_world: f64 = sim.contacts().iter().map(|c| c.force_world[2]).sum();
        samples.push(WbcSample { t, body_x: tx.x, body_z: tx.z, roll, pitch, total_fz_world });

        if csv_out.is_some() {
            csv_buf.push_str(&format!("{k},{t:.4}"));
            for name in LINK_NAMES {
                let p = sim.body_world_position(name).unwrap_or([0.0, 0.0, 0.0]);
                let r = sim
                    .body_world_orientation(name)
                    .map(|q| *q.to_rotation_matrix().matrix())
                    .unwrap_or_else(nalgebra::Matrix3::identity);
                csv_buf.push_str(&format!(",{:.5},{:.5},{:.5}", p[0], p[1], p[2]));
                for row in 0..3 {
                    for col in 0..3 {
                        csv_buf.push_str(&format!(",{:.6}", r[(row, col)]));
                    }
                }
            }
            csv_buf.push('\n');
        }
    }
    if let Some(path) = csv_out {
        std::fs::write(&path, csv_buf).expect("write WBC_WALK_CSV_OUT");
        eprintln!("wrote {path}");
    }
    Some(samples)
}

fn robot_mass(robot: &RobotModel) -> f64 {
    robot.links.iter().map(|l| l.inertial.mass).sum()
}

#[test]
fn go2_wbc_static_stand_balances_gravity() {
    let Some(samples) = run_wbc_sim(WbcParams::static_stand()) else { return };
    assert_static_stand_balances_gravity(&samples);
}

#[test]
fn go2_wbc_static_stand_balances_gravity_force_space_active_set() {
    let cfg = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
    let Some(samples) =
        run_wbc_sim(WbcParams::static_stand_misa_wbc(wbc::Formulation::ForceSpace, cfg))
    else {
        return;
    };
    assert_static_stand_balances_gravity(&samples);
}

fn assert_static_stand_balances_gravity(samples: &[WbcSample]) {
    let min_z = samples.iter().map(|s| s.body_z).fold(f64::INFINITY, f64::min);
    assert!(
        min_z > TRUNK_Z_FALL_THRESHOLD_M,
        "static stand: trunk fell, min_z = {min_z:.3} m (threshold {:.2})",
        TRUNK_Z_FALL_THRESHOLD_M,
    );

    let dt: f64 = 0.002;
    let total_time = 1.5;
    let total_n = (total_time / dt).round() as usize;
    let window_n = (0.5 / dt).round() as usize;
    let start = total_n.saturating_sub(window_n);
    let avg_fz: f64 =
        samples[start..].iter().map(|s| s.total_fz_world).sum::<f64>() / (samples.len() - start) as f64;

    let path = go2_misa();
    let robot = RobotModel::from_misa(&path).unwrap();
    let mg = robot_mass(&robot) * 9.81;

    let pct_err = ((avg_fz - mg) / mg).abs();
    eprintln!("[wbc:go2] static_stand: avg Σf_z = {avg_fz:.2} N, m·g = {mg:.2} N (err = {:.1}%)", pct_err * 100.0);
    assert!(
        pct_err < 0.60,
        "static stand: Σf_z = {avg_fz:.2} N deviates from m·g = {mg:.2} N by {:.1}%",
        pct_err * 100.0,
    );
}

#[test]
fn go2_wbc_forward_command_advances_body() {
    let Some(samples) = run_wbc_sim(WbcParams::forward_walk()) else { return };
    assert_forward_command_advances_body(&samples);
}

#[test]
fn go2_wbc_forward_command_advances_body_force_space_active_set() {
    let cfg = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
    let Some(samples) =
        run_wbc_sim(WbcParams::forward_walk_misa_wbc(wbc::Formulation::ForceSpace, cfg))
    else {
        return;
    };
    assert_forward_command_advances_body(&samples);
}

/// Stress test, not a regression check: command a 0 -> 5 m/s staircase
/// (0.5 m/s per level, ~2.73 s each) over 30 s and report per-level
/// tracking quality -- where does Trot+WBC+SRBD-MPC stop keeping up?
/// No hard pass/fail assertion (a fall at high commanded speed is an
/// expected, informative outcome, not a bug); `#[ignore]`d like the
/// other exploratory benchmarks in this session, run manually with
/// `WBC_WALK_CSV_OUT=<path> cargo test --release --features mujoco \
///  --test wbc_walk_go2 -- --ignored --nocapture go2_wbc_velocity_staircase`.
#[test]
#[ignore = "exploratory stress test — run with --ignored"]
fn go2_wbc_velocity_staircase() {
    let cfg = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
    let Some(samples) =
        run_wbc_sim(WbcParams::velocity_staircase_misa_wbc(wbc::Formulation::ForceSpace, cfg))
    else {
        return;
    };

    let n_levels = (STAIRCASE_MAX_MPS / STAIRCASE_STEP_MPS).round() as usize + 1;
    let total_time_s = 30.0_f64;
    let step_s = total_time_s / n_levels as f64;

    eprintln!(
        "\n{:>6} {:>8} {:>9} {:>9} {:>9} {:>9}",
        "level", "cmd_vx", "meas_vx", "min_z", "peak_roll", "peak_pitch"
    );
    for level in 0..n_levels {
        let t0 = level as f64 * step_s;
        let t1 = (level as f64 + 1.0) * step_s;
        let window: Vec<&WbcSample> =
            samples.iter().filter(|s| s.t >= t0 && s.t < t1).collect();
        if window.is_empty() {
            continue;
        }
        let cmd_vx = level as f64 * STAIRCASE_STEP_MPS;
        let x0 = window.first().unwrap().body_x;
        let x1 = window.last().unwrap().body_x;
        let meas_vx = (x1 - x0) / (t1 - t0).min(window.last().unwrap().t - t0);
        let min_z = window.iter().map(|s| s.body_z).fold(f64::INFINITY, f64::min);
        let peak_roll = window.iter().map(|s| s.roll.abs()).fold(0.0_f64, f64::max);
        let peak_pitch = window.iter().map(|s| s.pitch.abs()).fold(0.0_f64, f64::max);
        eprintln!(
            "{level:6} {cmd_vx:8.1} {meas_vx:9.2} {min_z:9.3} {peak_roll:9.2} {peak_pitch:9.2}",
        );
    }

    let min_z_overall = samples.iter().map(|s| s.body_z).fold(f64::INFINITY, f64::min);
    eprintln!(
        "\n[wbc:go2] velocity_staircase: min_z over full run = {min_z_overall:.3} m \
         (fall threshold {TRUNK_Z_FALL_THRESHOLD_M:.2} m)"
    );
}

fn assert_forward_command_advances_body(samples: &[WbcSample]) {
    let dt: f64 = 0.002;
    let burn_in_steps = (0.5 / dt).round() as usize;
    let walk = &samples[burn_in_steps..];

    let min_z = samples.iter().map(|s| s.body_z).fold(f64::INFINITY, f64::min);
    assert!(min_z > TRUNK_Z_FALL_THRESHOLD_M, "forward walk: trunk fell, min_z = {min_z:.3} m");

    let x_start = walk.first().map(|s| s.body_x).unwrap_or(0.0);
    let x_end = walk.last().map(|s| s.body_x).unwrap_or(0.0);
    let dx = x_end - x_start;
    eprintln!(
        "[wbc:go2] forward_command: Δx = {dx:.3} m over {:.1} s (threshold ≥ {:.2})",
        walk.last().map(|s| s.t).unwrap_or(0.0) - walk.first().map(|s| s.t).unwrap_or(0.0),
        MIN_DISPLACEMENT_M,
    );
    assert!(
        dx >= MIN_DISPLACEMENT_M,
        "forward walk: Δx = {dx:.3} m < {} m — WBC produced near-zero net forward motion",
        MIN_DISPLACEMENT_M,
    );

    let peak_roll = walk.iter().map(|s| s.roll.abs()).fold(0.0_f64, f64::max);
    let peak_pitch = walk.iter().map(|s| s.pitch.abs()).fold(0.0_f64, f64::max);
    eprintln!("[wbc:go2] forward_command: peak |roll| = {peak_roll:.2} rad, peak |pitch| = {peak_pitch:.2} rad");
    assert!(peak_roll < 0.5, "forward walk: |roll| peak {peak_roll:.2} rad too large");
    assert!(peak_pitch < 0.5, "forward walk: |pitch| peak {peak_pitch:.2} rad too large");
}
