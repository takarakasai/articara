//! Interactive REPL for trying out the Rhai scripting engine.
//!
//! ## Usage
//!
//! ```sh
//! cargo run --example script_repl --no-default-features --features scripting
//! ```
//!
//! This launches a REPL that simulates a robot control loop, letting you
//! type Rhai scripts interactively and see the outputs each cycle.
//!
//! ## Commands
//!
//! - Type Rhai code and press Enter to compile & run it.
//! - Multi-line: end a line with `\` to continue on the next line.
//! - `.run [N]`  — run the current script for N cycles (default: 5).
//! - `.sensors`  — show current simulated sensor values.
//! - `.clear`    — clear the loaded script.
//! - `.help`     — show this help.
//! - `.quit`     — exit.

#[cfg(feature = "scripting")]
fn main() {
    use articara::scripting::{ScriptEngine, ScriptInputs};
    use std::io::{self, BufRead, Write};

    println!("╔═══════════════════════════════════════════╗");
    println!("║  Articara Script REPL  (Rhai engine)      ║");
    println!("║  Type .help for commands                  ║");
    println!("╚═══════════════════════════════════════════╝");
    println!();

    let mut engine = ScriptEngine::new();

    // Simulated sensor values (pretend robot state)
    let mut sim_time = 0.0_f64;
    let dt = 0.001;  // 1kHz control loop

    // Simulated joint state: 6 joints
    let joint_positions: Vec<(usize, f64)> = vec![
        (0, 0.0), (1, -0.5), (2, 1.0), (3, -0.3), (4, 0.8), (5, -0.2),
    ];
    let joint_velocities: Vec<(usize, f64)> = vec![
        (0, 0.0), (1, 0.1), (2, -0.05), (3, 0.0), (4, 0.02), (5, 0.0),
    ];

    let stdin = io::stdin();
    let mut stdout = io::stdout();

    loop {
        print!("rhai> ");
        stdout.flush().unwrap();

        let mut line = String::new();
        if stdin.lock().read_line(&mut line).unwrap() == 0 {
            break; // EOF
        }
        let line = line.trim_end();

        // Multi-line input: if line ends with \, keep reading
        let input = if line.ends_with('\\') {
            let mut buf = line.trim_end_matches('\\').to_string();
            buf.push('\n');
            loop {
                print!("....  ");
                stdout.flush().unwrap();
                let mut cont = String::new();
                if stdin.lock().read_line(&mut cont).unwrap() == 0 {
                    break;
                }
                let cont = cont.trim_end();
                if cont.ends_with('\\') {
                    buf.push_str(cont.trim_end_matches('\\'));
                    buf.push('\n');
                } else {
                    buf.push_str(cont);
                    break;
                }
            }
            buf
        } else {
            line.to_string()
        };

        if input.is_empty() {
            continue;
        }

        match input.as_str() {
            ".quit" | ".exit" | ".q" => {
                println!("Bye!");
                break;
            }
            ".help" | ".h" => {
                println!("Commands:");
                println!("  .run [N]   Run current script for N cycles (default: 5)");
                println!("  .sensors   Show simulated sensor values");
                println!("  .clear     Clear loaded script & scope");
                println!("  .help      Show this help");
                println!("  .quit      Exit");
                println!();
                println!("Available script variables:");
                println!("  t          Simulation time (s)");
                println!("  dt         Time step (s)");
                println!("  q[\"N\"]     Joint position for joint N");
                println!("  qd[\"N\"]    Joint velocity for joint N");
                println!("  sensor[\"name\"]  Sensor value");
                println!();
                println!("Available script functions:");
                println!("  set_torque(joint_idx, value)   Set torque override");
                println!("  debug_val(name, value)         Log a debug value");
                println!("  sin, cos, abs, sqrt, atan2, min, max, clamp, sign");
                println!();
                println!("Multi-line: end a line with \\ to continue");
            }
            ".clear" => {
                engine.clear();
                sim_time = 0.0;
                println!("Script and scope cleared.");
            }
            ".sensors" => {
                println!("Simulated state (t = {:.3}s, dt = {}s):", sim_time, dt);
                println!("  Joint positions:");
                for &(i, v) in &joint_positions {
                    println!("    q[{i}] = {v:.4}");
                }
                println!("  Joint velocities:");
                for &(i, v) in &joint_velocities {
                    println!("    qd[{i}] = {v:.4}");
                }
                println!("  Sensors:");
                println!("    foot_force = 45.0");
                println!("    imu_pitch  = 0.02");
            }
            cmd if cmd.starts_with(".run") => {
                let n: usize = cmd
                    .split_whitespace()
                    .nth(1)
                    .and_then(|s| s.parse().ok())
                    .unwrap_or(5);

                if !engine.has_script() {
                    println!("No script loaded. Type a script first.");
                    continue;
                }

                println!("Running {} cycles...", n);
                for cycle in 0..n {
                    let mut inputs = ScriptInputs::default();
                    inputs.time = sim_time;
                    inputs.dt = dt;
                    for &(i, v) in &joint_positions {
                        inputs.joint_positions.insert(i, v);
                    }
                    for &(i, v) in &joint_velocities {
                        inputs.joint_velocities.insert(i, v);
                    }
                    inputs.sensors.insert("foot_force".to_string(), 45.0);
                    inputs.sensors.insert("imu_pitch".to_string(), 0.02);

                    let outputs = engine.eval(&inputs);
                    sim_time += dt;

                    // Print outputs for this cycle
                    let has_output = !outputs.torque_overrides.is_empty()
                        || !outputs.debug_values.is_empty();
                    if has_output {
                        print!("  [cycle {cycle}] ");
                        for (ji, tau) in &outputs.torque_overrides {
                            print!("torque[{ji}]={tau:.4}  ");
                        }
                        for (name, val) in &outputs.debug_values {
                            print!("{name}={val:.4}  ");
                        }
                        println!();
                    }
                }

                if let Some(err) = engine.last_error() {
                    println!("  Error: {err}");
                } else {
                    println!("Done. t = {:.3}s", sim_time);
                }
            }
            _ => {
                // Treat as Rhai source code — compile and run once
                match engine.compile(&input) {
                    Ok(()) => {
                        // Run one cycle
                        let mut inputs = ScriptInputs::default();
                        inputs.time = sim_time;
                        inputs.dt = dt;
                        for &(i, v) in &joint_positions {
                            inputs.joint_positions.insert(i, v);
                        }
                        for &(i, v) in &joint_velocities {
                            inputs.joint_velocities.insert(i, v);
                        }
                        inputs.sensors.insert("foot_force".to_string(), 45.0);
                        inputs.sensors.insert("imu_pitch".to_string(), 0.02);

                        let outputs = engine.eval(&inputs);
                        sim_time += dt;

                        // Print results
                        if !outputs.torque_overrides.is_empty() {
                            println!("  Torque overrides:");
                            for (ji, tau) in &outputs.torque_overrides {
                                println!("    joint[{ji}] = {tau:.6} N·m");
                            }
                        }
                        if !outputs.debug_values.is_empty() {
                            println!("  Debug values:");
                            for (name, val) in &outputs.debug_values {
                                println!("    {name} = {val:.6}");
                            }
                        }
                        if let Some(err) = engine.last_error() {
                            println!("  Runtime error: {err}");
                        }
                        if outputs.torque_overrides.is_empty()
                            && outputs.debug_values.is_empty()
                            && engine.last_error().is_none()
                        {
                            println!("  OK (no outputs)");
                        }
                    }
                    Err(e) => {
                        println!("  {e}");
                    }
                }
            }
        }
    }
}

#[cfg(not(feature = "scripting"))]
fn main() {
    eprintln!("This example requires the 'scripting' feature:");
    eprintln!("  cargo run --example script_repl --no-default-features --features scripting");
}
