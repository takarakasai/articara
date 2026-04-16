//! Generic host-side runner for articara WASM plugins.
//!
//! Loads the compiled WASM module, sends a command via the `execute`
//! entry point, and renders the response `View` list to the terminal.
//!
//! Usage:
//!   cargo run -p jump-sim-runner -- <command> <urdf_path> [options]
//!
//! Examples:
//!   jump-sim-runner list_commands <urdf>
//!   jump-sim-runner jump_sim <urdf>
//!   jump-sim-runner gravity_torques <urdf>
//!   jump-sim-runner static_analysis <urdf>
//!   jump-sim-runner payload_capacity <urdf> --ee-link FL_foot
//!   jump-sim-runner jump_height <urdf>

use std::collections::HashSet;
use std::path::PathBuf;

use serde::Serialize;
use wasmtime::*;

use articara::robot::RobotModel;
use articara_plugin_api as api;

// ── CLI parsing ────────────────────────────────────────────────────

struct Args {
    command: String,
    urdf_path: PathBuf,
    wasm_path: PathBuf,
    ee_link: Option<String>,
}

fn default_wasm_path() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .unwrap()
        .join("target/wasm32-unknown-unknown/release/jump_sim_wasm.wasm")
}

fn parse_args() -> Args {
    let mut raw = std::env::args().skip(1);
    let mut command: Option<String> = None;
    let mut urdf_path: Option<PathBuf> = None;
    let mut wasm_path: Option<PathBuf> = None;
    let mut ee_link: Option<String> = None;

    while let Some(arg) = raw.next() {
        match arg.as_str() {
            "--wasm" => wasm_path = raw.next().map(PathBuf::from),
            "--ee-link" => ee_link = raw.next(),
            _ => {
                if command.is_none() {
                    command = Some(arg);
                } else if urdf_path.is_none() {
                    urdf_path = Some(PathBuf::from(arg));
                }
            }
        }
    }

    let command = command.unwrap_or_else(|| {
        eprintln!(
            "Usage: jump-sim-runner <command> <urdf_path> [--wasm <path>] [--ee-link <link>]\n\
             Commands: list_commands, jump_sim, static_analysis, gravity_torques,\n\
                       payload_capacity, jump_height, payload_sim"
        );
        std::process::exit(1);
    });
    let urdf_path = urdf_path.unwrap_or_else(|| {
        eprintln!("Error: URDF path required");
        std::process::exit(1);
    });

    Args {
        command,
        urdf_path,
        wasm_path: wasm_path.unwrap_or_else(default_wasm_path),
        ee_link,
    }
}

// ── Build params JSON for each command ─────────────────────────────

fn build_params(args: &Args, model: &RobotModel) -> serde_json::Value {
    match args.command.as_str() {
        "list_commands" => serde_json::json!({}),
        "jump_sim" => {
            #[derive(Serialize)]
            struct P {
                model: RobotModel,
                ground_links: Vec<String>,
                body_link: Option<String>,
                speed: f32,
                locked_joints: HashSet<String>,
                launch_axes: [bool; 3],
                extension_duration: Option<f32>,
                enforce_torque_limits: bool,
                enable_retract: bool,
                graph_link: Option<String>,
                pd_kp: f64,
                pd_kd: f64,
            }
            serde_json::to_value(&P {
                model: model.clone(),
                ground_links: vec![
                    "RL_foot".into(), "FL_foot".into(),
                    "RR_foot".into(), "FR_foot".into(),
                ],
                body_link: Some("trunk".into()),
                speed: 1.0,
                locked_joints: HashSet::new(),
                launch_axes: [false, false, true],
                extension_duration: None,
                enforce_torque_limits: false,
                enable_retract: true,
                graph_link: Some("trunk".into()),
                pd_kp: 500.0,
                pd_kd: 20.0,
            }).unwrap()
        }
        "gravity_torques" => {
            serde_json::json!({ "model": model })
        }
        "payload_capacity" => {
            let ee = args.ee_link.as_deref().unwrap_or("FL_foot");
            serde_json::json!({ "model": model, "ee_link": ee })
        }
        "jump_height" => {
            serde_json::json!({
                "model": model,
                "ground_links": ["RL_foot", "FL_foot", "RR_foot", "FR_foot"],
                "body_link": "trunk",
            })
        }
        "static_analysis" => {
            let ee = args.ee_link.as_deref();
            serde_json::json!({
                "model": model,
                "ee_link": ee,
                "body_link": "trunk",
                "ground_links": ["RL_foot", "FL_foot", "RR_foot", "FR_foot"],
            })
        }
        "payload_sim" => {
            let ee = args.ee_link.as_deref().unwrap_or("FL_foot");
            serde_json::json!({ "model": model, "ee_link": ee, "speed": 1.0 })
        }
        _ => serde_json::json!({})
    }
}

// ── Generic View renderer (terminal) ───────────────────────────────

fn render_views(views: &[api::View]) {
    for view in views {
        match view {
            api::View::Heading { text, level } => {
                let prefix = "#".repeat(*level as usize);
                println!("\n{prefix} {text}");
            }
            api::View::Scalars { title, items } => {
                if let Some(t) = title {
                    println!("\n--- {t} ---");
                }
                for item in items {
                    let emphasis = match item.emphasis.as_deref() {
                        Some("primary") => " ★",
                        Some("warning") => " ⚠",
                        _ => "",
                    };
                    println!("  {:<25} {}{}", item.label, item.value, emphasis);
                }
            }
            api::View::Table { title, columns, rows } => {
                if let Some(t) = title {
                    println!("\n--- {t} ---");
                }
                // Simple fixed-width table
                let widths: Vec<usize> = columns
                    .iter()
                    .enumerate()
                    .map(|(ci, col)| {
                        let header_w = col.name.len();
                        let max_cell = rows.iter().map(|row| {
                            row.get(ci).map(|c| cell_text(c).len()).unwrap_or(0)
                        }).max().unwrap_or(0);
                        header_w.max(max_cell).max(6)
                    })
                    .collect();

                // Header
                let header: String = columns.iter().enumerate()
                    .map(|(i, c)| format!("{:>width$}", c.name, width = widths[i]))
                    .collect::<Vec<_>>().join("  ");
                println!("  {header}");
                println!("  {}", "-".repeat(header.len()));

                // Rows
                for row in rows {
                    let line: String = row.iter().enumerate()
                        .map(|(i, c)| format!("{:>width$}", cell_text(c), width = widths[i]))
                        .collect::<Vec<_>>().join("  ");
                    println!("  {line}");
                }
            }
            api::View::LinePlot { title, series, .. } => {
                println!("\n--- {title} ---");
                for s in series {
                    let n = s.y.len();
                    if n == 0 { continue; }
                    let min = s.y.iter().cloned().fold(f64::INFINITY, f64::min);
                    let max = s.y.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
                    println!("  {:<6} {} pts  min={:.4}  max={:.4}", s.name, n, min, max);
                }
            }
            api::View::BarChart { title, bars, .. } => {
                println!("\n--- {title} ---");
                let max_label = bars.iter().map(|b| b.label.len()).max().unwrap_or(0);
                for bar in bars {
                    let bar_len = (bar.value * 40.0).min(60.0).max(0.0) as usize;
                    let tag = bar.tag.as_deref().unwrap_or("");
                    println!(
                        "  {:<width$} [{:<bar_len$}] {:.1}% {tag}",
                        bar.label,
                        "█".repeat(bar_len),
                        bar.value * 100.0,
                        width = max_label,
                        bar_len = 40,
                    );
                }
            }
            api::View::Progress { label, value, text } => {
                let pct = (value * 100.0) as u32;
                let txt = text.as_deref().unwrap_or("");
                println!("  {label}: [{pct}%] {txt}");
            }
            api::View::Log { messages } => {
                for msg in messages {
                    let prefix = match msg.level.as_str() {
                        "warning" => "⚠ ",
                        "error" => "✗ ",
                        _ => "  ",
                    };
                    println!("{prefix}{}", msg.text);
                }
            }
        }
    }
}

fn cell_text(c: &api::Cell) -> String {
    match c {
        api::Cell::Text { value } => value.clone(),
        api::Cell::Number { value, format } => {
            match format.as_deref() {
                Some(".1f") => format!("{:.1}", value),
                Some(".2f") => format!("{:.2}", value),
                Some(".3f") => format!("{:.3}", value),
                Some(".4f") => format!("{:.4}", value),
                _ => format!("{:.4}", value),
            }
        }
        api::Cell::Tag { value, .. } => value.clone(),
    }
}

// ── WASM invocation ────────────────────────────────────────────────

fn call_plugin(
    store: &mut Store<()>,
    instance: &Instance,
    request: &api::Request,
) -> anyhow::Result<api::Response> {
    let alloc_fn = instance.get_typed_func::<u32, u32>(&mut *store, "alloc")?;
    let dealloc_fn = instance.get_typed_func::<(u32, u32), ()>(&mut *store, "dealloc")?;
    let execute_fn = instance.get_typed_func::<(u32, u32), u32>(&mut *store, "execute")?;
    let out_ptr_fn = instance.get_typed_func::<(), u32>(&mut *store, "last_output_ptr")?;
    let out_len_fn = instance.get_typed_func::<(), u32>(&mut *store, "last_output_len")?;
    let memory = instance
        .get_memory(&mut *store, "memory")
        .expect("WASM module must export 'memory'");

    let json = serde_json::to_vec(request)?;
    let len = json.len() as u32;

    let ptr = alloc_fn.call(&mut *store, len)?;
    memory.write(&mut *store, ptr as usize, &json)?;
    let _rc = execute_fn.call(&mut *store, (ptr, len))?;

    let out_ptr = out_ptr_fn.call(&mut *store, ())? as usize;
    let out_len = out_len_fn.call(&mut *store, ())? as usize;
    let mut buf = vec![0u8; out_len];
    memory.read(&*store, out_ptr, &mut buf)?;

    dealloc_fn.call(&mut *store, (ptr, len))?;

    let response: api::Response = serde_json::from_slice(&buf)?;
    Ok(response)
}

// ── Main ───────────────────────────────────────────────────────────

fn main() -> anyhow::Result<()> {
    let args = parse_args();

    println!("Loading URDF: {}", args.urdf_path.display());
    let model = RobotModel::from_urdf(&args.urdf_path)
        .unwrap_or_else(|_| panic!("Failed to load URDF: {}", args.urdf_path.display()));

    let params = build_params(&args, &model);

    println!("Loading WASM: {}", args.wasm_path.display());
    if !args.wasm_path.exists() {
        eprintln!(
            "WASM file not found. Build it first:\n  \
             cargo build -p jump-sim-wasm --target wasm32-unknown-unknown --release"
        );
        std::process::exit(1);
    }

    let engine = Engine::default();
    let module = Module::from_file(&engine, &args.wasm_path)?;
    let mut store = Store::new(&engine, ());
    let instance = Instance::new(&mut store, &module, &[])?;

    let request = api::Request {
        version: 1,
        command: args.command.clone(),
        params,
    };

    let req_size = serde_json::to_vec(&request)?.len();
    println!("Command: {}  (request: {} bytes)", args.command, req_size);

    let t0 = std::time::Instant::now();
    let response = call_plugin(&mut store, &instance, &request)?;
    let elapsed = t0.elapsed();

    println!(
        "Completed in {:.3}s — ok={}",
        elapsed.as_secs_f64(),
        response.ok
    );

    if !response.ok {
        eprintln!("Error: {}", response.error.as_deref().unwrap_or("(unknown)"));
        std::process::exit(1);
    }

    // Render views
    if let Some(ref views) = response.views {
        render_views(views);
    }

    Ok(())
}
