//! Headless bench sweeper for the FullCentroidal MPC's experimental
//! knobs.
//!
//! Runs a MuJoCo walk for every combination in a grid of
//! `quadruped_gait::exp` knob values (optionally seeded from a saved
//! preset), measures the trunk stability metrics the walk regression
//! suite asserts on, and prints an aligned table (plus optional CSV).
//! Turns "toggle a knob in the GUI and eyeball it" into "sweep
//! overnight, read the table in the morning".
//!
//! ```text
//! cargo xtask run --features mujoco --example exp_sweep -- \
//!     --model tests/fixtures/namiashi/namiashi.misa \
//!     --set transition_fraction=0,0.05,0.10 \
//!     --set transition_enforce_constraint=true,false \
//!     --push-n 6 --push-t 1.5 --push-axis y \
//!     --csv /tmp/c1_sweep.csv
//! ```
//!
//! Defaults reproduce the `integration_walk` disturbance-bench harness:
//! namiashi `.misa`, 0.15 m/s forward command after a 0.5 s burn-in,
//! 3 s of steady walking before the (optional) pulse, ~4 s of recovery
//! observation, 2 ms physics/controller dt.

#[cfg(feature = "mujoco")]
fn main() {
    if let Err(e) = sweep::run() {
        eprintln!("exp_sweep: {e}");
        std::process::exit(1);
    }
}

#[cfg(feature = "mujoco")]
mod sweep {
    use articara::gait::{
        auto_detect_kinematics_config, GaitController, DEFAULT_FOOT_LINKS,
    };
    use articara::mjcf::{GroundPlaneCfg, MjcfExportOptions};
    use articara::mujoco_sim::MujocoSim;
    use articara::robot::RobotModel;
    use articara::wbc_pipeline::WbcPipeline;
    use nalgebra::Vector3;
    use quadruped_gait::{
        load_presets, solve_leg_ik, wbc::WbcWeights, ContactDrivenPhase, ExpValue, GaitConfig,
        GaitMode, KinematicsConfig, LegIkSolution, VelocityCmd,
    };
    use std::path::PathBuf;

    struct Args {
        model: PathBuf,
        /// Grid axes: (key, candidate values). Cartesian product is swept.
        axes: Vec<(String, Vec<ExpValue>)>,
        /// Base preset applied before each combo: (file, name).
        preset: Option<(PathBuf, String)>,
        cmd: VelocityCmd,
        total_s: f64,
        burn_in_s: f64,
        dt: f64,
        /// Lateral/longitudinal disturbance pulse (N); 0 = off.
        push_n: f64,
        push_t: f64,
        push_dur: f64,
        push_axis: usize,
        /// Trunk-z fall threshold; mirrors gait_walk_stability.
        fall_z: f64,
        /// Vertical bend of the nominal stance as a fraction of leg
        /// length (lifts the q=0 straight-knee Jacobian singularity).
        bend: f64,
        csv: Option<PathBuf>,
    }

    fn parse_exp_value(s: &str) -> Result<ExpValue, String> {
        match s {
            "true" => Ok(ExpValue::Bool(true)),
            "false" => Ok(ExpValue::Bool(false)),
            v => v
                .parse::<f64>()
                .map(ExpValue::F64)
                .map_err(|e| format!("bad knob value '{v}': {e}")),
        }
    }

    fn parse_args() -> Result<Args, String> {
        let mut args = Args {
            model: PathBuf::from("tests/fixtures/namiashi/namiashi.misa"),
            axes: Vec::new(),
            preset: None,
            cmd: VelocityCmd { vx: 0.15, vy: 0.0, wz: 0.0 },
            total_s: 7.7,
            burn_in_s: 0.5,
            dt: 0.002,
            push_n: 0.0,
            push_t: 3.5,
            push_dur: 0.2,
            push_axis: 1,
            fall_z: 0.18,
            bend: 0.08,
            csv: None,
        };
        let mut it = std::env::args().skip(1);
        while let Some(flag) = it.next() {
            let mut val = |name: &str| {
                it.next().ok_or_else(|| format!("{name} needs a value"))
            };
            match flag.as_str() {
                "--model" => args.model = PathBuf::from(val("--model")?),
                "--set" => {
                    let spec = val("--set")?;
                    let (key, values) = spec
                        .split_once('=')
                        .ok_or_else(|| format!("--set expects key=v1,v2,...: {spec}"))?;
                    let values = values
                        .split(',')
                        .map(parse_exp_value)
                        .collect::<Result<Vec<_>, _>>()?;
                    if values.is_empty() {
                        return Err(format!("--set {key}: no values"));
                    }
                    args.axes.push((key.to_string(), values));
                }
                "--preset" => {
                    let spec = val("--preset")?;
                    let (file, name) = spec
                        .split_once(':')
                        .ok_or_else(|| format!("--preset expects file:name: {spec}"))?;
                    args.preset = Some((PathBuf::from(file), name.to_string()));
                }
                "--vx" => args.cmd.vx = val("--vx")?.parse().map_err(|e| format!("--vx: {e}"))?,
                "--vy" => args.cmd.vy = val("--vy")?.parse().map_err(|e| format!("--vy: {e}"))?,
                "--wz" => args.cmd.wz = val("--wz")?.parse().map_err(|e| format!("--wz: {e}"))?,
                "--duration" => {
                    args.total_s = val("--duration")?.parse().map_err(|e| format!("--duration: {e}"))?
                }
                "--burn-in" => {
                    args.burn_in_s = val("--burn-in")?.parse().map_err(|e| format!("--burn-in: {e}"))?
                }
                "--dt" => args.dt = val("--dt")?.parse().map_err(|e| format!("--dt: {e}"))?,
                "--push-n" => {
                    args.push_n = val("--push-n")?.parse().map_err(|e| format!("--push-n: {e}"))?
                }
                "--push-t" => {
                    args.push_t = val("--push-t")?.parse().map_err(|e| format!("--push-t: {e}"))?
                }
                "--push-dur" => {
                    args.push_dur = val("--push-dur")?.parse().map_err(|e| format!("--push-dur: {e}"))?
                }
                "--push-axis" => {
                    args.push_axis = match val("--push-axis")?.as_str() {
                        "x" => 0,
                        "y" => 1,
                        other => return Err(format!("--push-axis expects x|y, got {other}")),
                    }
                }
                "--fall-z" => {
                    args.fall_z = val("--fall-z")?.parse().map_err(|e| format!("--fall-z: {e}"))?
                }
                "--bend" => args.bend = val("--bend")?.parse().map_err(|e| format!("--bend: {e}"))?,
                "--csv" => args.csv = Some(PathBuf::from(val("--csv")?)),
                "--help" | "-h" => {
                    println!("{}", HELP);
                    std::process::exit(0);
                }
                other => return Err(format!("unknown flag {other} (see --help)")),
            }
        }
        Ok(args)
    }

    const HELP: &str = "\
exp_sweep — headless FullCentroidal exp-knob bench sweeper

  --model <path>        robot model (.misa / .urdf; default namiashi fixture)
  --set key=v1,v2,...   grid axis over an experimental knob (repeatable;
                        bools as true/false). Cartesian product is swept.
  --preset file:name    apply a saved exp preset before each combo
  --vx/--vy/--wz <f>    velocity command after burn-in (default 0.15/0/0)
  --duration <s>        total sim time incl. burn-in (default 7.7)
  --burn-in <s>         settle window before the command (default 0.5)
  --dt <s>              physics + controller dt (default 0.002)
  --push-n <N>          disturbance pulse magnitude (default 0 = off)
  --push-t <s>          pulse start time (default 3.5)
  --push-dur <s>        pulse duration (default 0.2)
  --push-axis x|y       pulse direction in world frame (default y)
  --fall-z <m>          trunk-z fall threshold (default 0.18)
  --bend <frac>         nominal-stance vertical bend as a fraction of
                        leg length (default 0.08)
  --csv <path>          also write the rows as CSV";

    /// Metrics from one run. Mirrors what gait_walk_stability asserts on,
    /// plus solve-time accounting.
    struct RunMetrics {
        dx: f64,
        dy_max: f64,
        roll_max: f64,
        pitch_max: f64,
        min_z: f64,
        fell: bool,
        tick_ms_avg: f64,
        tick_ms_max: f64,
    }

    /// Solve per-leg IK at `nominal_foot_body` and seed the joint
    /// positions (same seeding as the walk regression harness — without
    /// it the first ticks run with straight knees at the Jacobian
    /// singularity and the body drops).
    fn seed_joint_positions(robot: &mut RobotModel, kin: &KinematicsConfig) -> Result<(), String> {
        for leg_kin in [&kin.fl, &kin.fr, &kin.rl, &kin.rr] {
            let sol = solve_leg_ik(leg_kin, leg_kin.nominal_foot_body, false);
            let LegIkSolution::Reached { hip, thigh, calf } = sol else {
                return Err(format!("{:?}: nominal_foot_body unreachable", leg_kin.leg));
            };
            for (joint_name, q_ik, sign) in [
                (&leg_kin.hip_joint, hip, 1.0),
                (&leg_kin.thigh_joint, thigh, -1.0),
                (&leg_kin.calf_joint, calf, -1.0),
            ] {
                if let Some(&ji) = robot.joint_map.get(joint_name.as_str()) {
                    robot.joint_positions[ji] = q_ik * sign;
                }
            }
        }
        Ok(())
    }

    fn run_one(
        args: &Args,
        combo: &[(String, ExpValue)],
        base: Option<&quadruped_gait::ExpPreset>,
    ) -> Result<RunMetrics, String> {
        // Fresh everything per combo so runs can't taint each other.
        let mut robot = RobotModel::from_file(&args.model)
            .map_err(|e| format!("load {}: {e}", args.model.display()))?;

        let mut kin = auto_detect_kinematics_config(&robot, &DEFAULT_FOOT_LINKS)
            .map_err(|e| format!("kinematics auto-detect: {e:?}"))?;
        for leg_kin in [&mut kin.fl, &mut kin.fr, &mut kin.rl, &mut kin.rr] {
            let total_leg = leg_kin.upper_leg_m + leg_kin.lower_leg_m;
            leg_kin.nominal_foot_body.z += args.bend * total_leg;
        }
        seed_joint_positions(&mut robot, &kin)?;

        let opts = MjcfExportOptions {
            ground_plane: Some(GroundPlaneCfg { z: 0.0, half_size: 4.0, roll: 0.0, pitch: 0.0 }),
            add_actuators: true,
            ..Default::default()
        };
        let mut sim = MujocoSim::new(&robot, opts).map_err(|e| format!("MujocoSim: {e}"))?;
        sim.set_gravity_compensation(true);

        let mut gc = GaitController::build(&robot, kin, GaitConfig::trot(), GaitMode::FullCentroidal)
            .map_err(|e| format!("GaitController::build: {e}"))?;

        if let Some(preset) = base {
            let (_, skipped) = gc.apply_experimental(preset);
            for key in skipped {
                eprintln!("preset '{}': knob '{key}' skipped", preset.name);
            }
        }
        for (key, value) in combo {
            gc.set_experimental(key, *value)
                .map_err(|e| format!("--set {key}: {e}"))?;
        }

        // FullCentroidal runs with the hierarchical WBC feeding hybrid
        // torque feedforward — the same drive shape as the GUI and the
        // integration_walk bench matrix (position-PD alone can't hold
        // the FullCentroidal solution and falls).
        let foot_links: [String; 4] =
            DEFAULT_FOOT_LINKS.map(|(_, name)| name.to_string());
        let mut wbc = WbcPipeline::new(&robot, foot_links);
        if let Some(full_cfg) = gc.full_centroidal_mpc_config() {
            wbc.mass_kg = full_cfg.mass_kg;
            wbc.centroidal_inertia_body = Some(full_cfg.centroidal_inertia_body);
            wbc.com_offset_body = full_cfg.com_offset_body;
        }

        let n_steps = (args.total_s / args.dt).round() as usize;
        let burn_in_steps = (args.burn_in_s / args.dt).round() as usize;
        let root = robot.root_link.clone();

        let mut m = RunMetrics {
            dx: 0.0,
            dy_max: 0.0,
            roll_max: 0.0,
            pitch_max: 0.0,
            min_z: f64::INFINITY,
            fell: false,
            tick_ms_avg: 0.0,
            tick_ms_max: 0.0,
        };
        let mut tick_ms_sum = 0.0;
        let mut walk_start_xy = (0.0, 0.0);
        let mut pushed = false;

        gc.enable();
        for k in 0..n_steps {
            let t = k as f64 * args.dt;
            if k == burn_in_steps {
                gc.set_velocity_cmd(args.cmd);
                let tx = robot.base_transform.translation;
                walk_start_xy = (tx.x as f64, tx.y as f64);
            }
            if args.push_n != 0.0 && !pushed && t >= args.push_t {
                let mut f = [0.0; 3];
                f[args.push_axis] = args.push_n;
                sim.apply_external_force(&root, f, [0.0; 3], args.push_dur);
                pushed = true;
            }

            let v = sim.body_world_linear_velocity(&root).unwrap_or([0.0; 3]);
            let w = sim.body_world_angular_velocity(&root).unwrap_or([0.0; 3]);
            gc.set_body_state_observed(
                Vector3::new(v[0], v[1], v[2]),
                Vector3::new(w[0], w[1], w[2]),
            );
            let body_pos = sim.body_world_position(&root).unwrap_or([0.0; 3]);
            let yaw_obs = sim.body_world_yaw(&root).unwrap_or(0.0);
            gc.set_body_pose_observed(
                yaw_obs,
                Vector3::new(body_pos[0], body_pos[1], body_pos[2]),
            );
            wbc.weights = WbcWeights::for_cmd_centroidal(&gc.velocity_cmd());

            let t0 = std::time::Instant::now();
            let (out, targets, _torque_ff) = gc.tick(args.dt);
            let tick_ms = t0.elapsed().as_secs_f64() * 1e3;
            tick_ms_sum += tick_ms;
            m.tick_ms_max = m.tick_ms_max.max(tick_ms);

            for (idx, q) in targets {
                sim.set_position_target(idx, q);
            }
            // Hybrid joint command: WBC torques ride as feedforward on
            // top of Position-PD (integration_walk / GUI shape).
            if k >= burn_in_steps {
                let f_grf_world = gc
                    .predicted_grfs()
                    .map(|sol| sol.grfs_first_step)
                    .unwrap_or([Vector3::zeros(); 4]);
                let cmd_w = gc.velocity_cmd();
                let v_cmd_body = Vector3::new(cmd_w.vx, cmd_w.vy, 0.0);
                let foot_links_str: [&str; 4] = [
                    wbc.foot_links[0].as_str(),
                    wbc.foot_links[1].as_str(),
                    wbc.foot_links[2].as_str(),
                    wbc.foot_links[3].as_str(),
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
                let taus = wbc.solve(
                    &robot,
                    &sim,
                    &out,
                    gc.kinematics(),
                    gc.joint_indices(),
                    gc.joint_signs(),
                    &v_cmd_body,
                    cmd_w.wz,
                    &Vector3::new(v[0], v[1], v[2]),
                    &Vector3::new(w[0], w[1], w[2]),
                    &f_grf_world,
                    contact_flag,
                    args.dt,
                );
                for (ji, &tau) in taus.iter().enumerate() {
                    sim.set_torque_feedforward(ji, tau);
                }
            } else {
                for ji in 0..robot.joints.len() {
                    sim.set_torque_feedforward(ji, 0.0);
                }
            }
            sim.step(&mut robot, args.dt, true);

            let tx = robot.base_transform.translation;
            let (roll, pitch, _yaw) = robot.base_transform.rotation.euler_angles();
            m.min_z = m.min_z.min(tx.z as f64);
            if k >= burn_in_steps {
                m.roll_max = m.roll_max.max((roll as f64).abs());
                m.pitch_max = m.pitch_max.max((pitch as f64).abs());
                m.dy_max = m.dy_max.max((tx.y as f64 - walk_start_xy.1).abs());
                m.dx = tx.x as f64 - walk_start_xy.0;
            }
            if (tx.z as f64) < args.fall_z {
                // Record and keep going — the interesting differences
                // (e.g. C1-2's recovery behaviour) live *after* the
                // disturbance, and a truncated run would freeze every
                // combo at the same pre-divergence prefix.
                m.fell = true;
            }
        }
        m.tick_ms_avg = tick_ms_sum / n_steps as f64;
        Ok(m)
    }

    pub fn run() -> Result<(), String> {
        let args = parse_args()?;

        let base = match &args.preset {
            Some((file, name)) => {
                let presets = load_presets(file)?;
                Some(
                    presets
                        .into_iter()
                        .find(|p| p.name == *name)
                        .ok_or_else(|| format!("preset '{name}' not in {}", file.display()))?,
                )
            }
            None => None,
        };

        // Cartesian product over the grid axes (odometer).
        let combos: Vec<Vec<(String, ExpValue)>> = if args.axes.is_empty() {
            vec![Vec::new()]
        } else {
            let mut combos = Vec::new();
            let mut idx = vec![0usize; args.axes.len()];
            loop {
                combos.push(
                    args.axes
                        .iter()
                        .zip(&idx)
                        .map(|((k, vs), &i)| (k.clone(), vs[i]))
                        .collect(),
                );
                let mut d = args.axes.len();
                loop {
                    if d == 0 {
                        break;
                    }
                    d -= 1;
                    idx[d] += 1;
                    if idx[d] < args.axes[d].1.len() {
                        break;
                    }
                    idx[d] = 0;
                    if d == 0 {
                        d = usize::MAX;
                        break;
                    }
                }
                if d == usize::MAX {
                    break;
                }
            }
            combos
        };

        let fmt_v = |v: &ExpValue| match v {
            ExpValue::Bool(b) => b.to_string(),
            ExpValue::F64(x) => format!("{x}"),
        };

        println!(
            "model={} combos={} cmd=({},{},{}) t={}s push={}N@{}s axis={}\n",
            args.model.display(),
            combos.len(),
            args.cmd.vx,
            args.cmd.vy,
            args.cmd.wz,
            args.total_s,
            args.push_n,
            args.push_t,
            if args.push_axis == 0 { "x" } else { "y" },
        );

        let combo_label = |combo: &[(String, ExpValue)]| -> String {
            if combo.is_empty() {
                "(baseline)".to_string()
            } else {
                combo
                    .iter()
                    .map(|(k, v)| format!("{k}={}", fmt_v(v)))
                    .collect::<Vec<_>>()
                    .join(" ")
            }
        };
        let width = combos
            .iter()
            .map(|c| combo_label(c).len())
            .max()
            .unwrap_or(10)
            .max(10);

        println!(
            "{:<width$} |   Δx [m] | |dy|max |  roll_max | pitch_max |  min_z | fell | tick avg/max [ms]",
            "combo",
        );
        println!("{}", "-".repeat(width + 78));

        let mut csv = String::from(
            "combo,dx_m,dy_max_m,roll_max_rad,pitch_max_rad,min_z_m,fell,tick_ms_avg,tick_ms_max\n",
        );
        for combo in &combos {
            let label = combo_label(combo);
            let m = run_one(&args, combo, base.as_ref())?;
            println!(
                "{label:<width$} | {dx:+8.3} | {dy:7.4} | {roll:9.4} | {pitch:9.4} | {z:6.3} | {fell:>4} | {ta:5.2} / {tm:6.2}",
                dx = m.dx,
                dy = m.dy_max,
                roll = m.roll_max,
                pitch = m.pitch_max,
                z = m.min_z,
                fell = if m.fell { "YES" } else { "no" },
                ta = m.tick_ms_avg,
                tm = m.tick_ms_max,
            );
            csv.push_str(&format!(
                "\"{label}\",{},{},{},{},{},{},{},{}\n",
                m.dx, m.dy_max, m.roll_max, m.pitch_max, m.min_z, m.fell, m.tick_ms_avg, m.tick_ms_max,
            ));
        }

        if let Some(path) = &args.csv {
            std::fs::write(path, csv).map_err(|e| format!("write {}: {e}", path.display()))?;
            println!("\nCSV → {}", path.display());
        }
        Ok(())
    }
}

#[cfg(not(feature = "mujoco"))]
fn main() {
    eprintln!("exp_sweep requires the `mujoco` feature: cargo xtask run --features mujoco --example exp_sweep");
}
