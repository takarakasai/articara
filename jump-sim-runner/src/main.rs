//! Host-side runner for the jump-sim WASM plugin.
//!
//! Loads the compiled `jump_sim_wasm.wasm` module, serialises a
//! `JumpSimInput` as JSON, invokes the simulation inside the WASM
//! sandbox, and deserialises the `JumpSimOutput` result.
//!
//! Usage:
//!   cargo run -p jump-sim-runner -- <urdf_path> [--wasm <path_to_wasm>]
//!
//! The WASM module is expected at
//!   target/wasm32-unknown-unknown/release/jump_sim_wasm.wasm
//! unless overridden with `--wasm`.

use std::collections::HashSet;
use std::path::PathBuf;

use serde::{Deserialize, Serialize};
use wasmtime::*;

use articara::dynamics;
use articara::robot::RobotModel;

// ── Mirror types from jump-sim-wasm (host side) ────────────────────

#[derive(Serialize)]
struct JumpSimInput {
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

#[derive(Deserialize)]
struct JumpSimOutput {
    ok: bool,
    error: Option<String>,
    result: Option<dynamics::JumpSimResult>,
}

// ── Helpers ────────────────────────────────────────────────────────

fn default_wasm_path() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .unwrap()
        .join("target/wasm32-unknown-unknown/release/jump_sim_wasm.wasm")
}

fn parse_args() -> (PathBuf, PathBuf) {
    let mut args = std::env::args().skip(1);
    let mut urdf_path: Option<PathBuf> = None;
    let mut wasm_path: Option<PathBuf> = None;

    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--wasm" => {
                wasm_path = args.next().map(PathBuf::from);
            }
            _ => {
                if urdf_path.is_none() {
                    urdf_path = Some(PathBuf::from(arg));
                }
            }
        }
    }

    let urdf = urdf_path.unwrap_or_else(|| {
        eprintln!("Usage: jump-sim-runner <urdf_path> [--wasm <wasm_path>]");
        std::process::exit(1);
    });
    let wasm = wasm_path.unwrap_or_else(default_wasm_path);
    (urdf, wasm)
}

// ── Main ───────────────────────────────────────────────────────────

fn main() -> anyhow::Result<()> {
    let (urdf_path, wasm_path) = parse_args();

    // 1. Load the robot model from URDF
    println!("Loading URDF: {}", urdf_path.display());
    let model = RobotModel::from_urdf(&urdf_path)
        .unwrap_or_else(|_| panic!("Failed to load URDF: {}", urdf_path.display()));

    // 2. Build the input JSON
    let input = JumpSimInput {
        model,
        ground_links: vec![
            "RL_foot".into(),
            "FL_foot".into(),
            "RR_foot".into(),
            "FR_foot".into(),
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
    };
    let input_json = serde_json::to_vec(&input)?;
    println!("Input JSON size: {} bytes", input_json.len());

    // 3. Load the WASM module
    println!("Loading WASM: {}", wasm_path.display());
    if !wasm_path.exists() {
        eprintln!(
            "WASM file not found. Build it first:\n  \
             cargo build -p jump-sim-wasm --target wasm32-unknown-unknown --release"
        );
        std::process::exit(1);
    }

    let engine = Engine::default();
    let module = Module::from_file(&engine, &wasm_path)?;
    let mut store = Store::new(&engine, ());
    let instance = Instance::new(&mut store, &module, &[])?;

    // 4. Get exported functions
    let alloc_fn = instance
        .get_typed_func::<u32, u32>(&mut store, "alloc")?;
    let dealloc_fn = instance
        .get_typed_func::<(u32, u32), ()>(&mut store, "dealloc")?;
    let run_fn = instance
        .get_typed_func::<(u32, u32), u32>(&mut store, "run_jump_sim")?;
    let out_ptr_fn = instance
        .get_typed_func::<(), u32>(&mut store, "last_output_ptr")?;
    let out_len_fn = instance
        .get_typed_func::<(), u32>(&mut store, "last_output_len")?;
    let memory = instance
        .get_memory(&mut store, "memory")
        .expect("WASM module must export 'memory'");

    // 5. Allocate input buffer in WASM memory and write JSON
    let input_len = input_json.len() as u32;
    let input_ptr = alloc_fn.call(&mut store, input_len)?;
    memory.write(&mut store, input_ptr as usize, &input_json)?;

    // 6. Run the simulation
    println!("Running jump simulation in WASM sandbox...");
    let t0 = std::time::Instant::now();
    let rc = run_fn.call(&mut store, (input_ptr, input_len))?;
    let elapsed = t0.elapsed();
    println!("Simulation completed in {:.3}s (return code: {})", elapsed.as_secs_f64(), rc);

    // 7. Read the output
    let out_ptr = out_ptr_fn.call(&mut store, ())? as usize;
    let out_len = out_len_fn.call(&mut store, ())? as usize;
    let mut output_buf = vec![0u8; out_len];
    memory.read(&store, out_ptr, &mut output_buf)?;

    // 8. Free input buffer
    dealloc_fn.call(&mut store, (input_ptr, input_len))?;

    // 9. Parse and display the result
    let output: JumpSimOutput = serde_json::from_slice(&output_buf)?;
    if output.ok {
        let r = output.result.as_ref().unwrap();
        println!("\n=== Jump Simulation Result ===");
        println!("  Max height:          {:.4} m", r.max_height);
        println!("  Extension duration:  {:.4} s", r.extension_duration);
        println!("  Graph samples:       {}", r.graph_data.time.len());
        println!("  Joint peaks:");
        for jp in &r.joint_peaks {
            println!(
                "    [{:2}] {:<20} torque={:.2} N·m  vel={:.2} rad/s  contributes={}",
                jp.joint_idx, jp.joint_name, jp.peak_torque, jp.peak_velocity, jp.contributes,
            );
        }
    } else {
        eprintln!("Simulation failed: {}", output.error.unwrap_or_default());
        std::process::exit(1);
    }

    Ok(())
}
