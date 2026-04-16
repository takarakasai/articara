//! WASM plugin for batch jump simulation.
//!
//! Exports a single function `run_jump_sim` that:
//! 1. Receives a JSON-encoded `JumpSimInput` (RobotModel + config)
//! 2. Runs the full Extension→Flight→Landed simulation to completion
//! 3. Returns a JSON-encoded `JumpSimResult`
//!
//! The host calls this via a WASM runtime (wasmtime / wasmer) and
//! deserialises the result for display.

use serde::{Deserialize, Serialize};
use std::collections::HashSet;

use articara::dynamics;
use articara::rbd::model::RobotModel;

// ======================================================================
//  Input / Output types (JSON-serialisable)
// ======================================================================

/// Configuration sent from the host to the WASM plugin.
#[derive(Serialize, Deserialize)]
pub struct JumpSimInput {
    /// The robot model (already parsed from URDF on the host side).
    pub model: RobotModel,
    /// Ground-contact link names (e.g. `["RL_foot", "FL_foot", ...]`).
    pub ground_links: Vec<String>,
    /// Body (trunk) link name.  `None` → use root link.
    pub body_link: Option<String>,
    /// Simulation speed multiplier (usually 1.0).
    pub speed: f32,
    /// Joint names that are locked (not driven) during the jump.
    pub locked_joints: HashSet<String>,
    /// Which axes the body link can move during flight `[x, y, z]`.
    pub launch_axes: [bool; 3],
    /// Override for extension duration (seconds).  `None` → auto.
    pub extension_duration: Option<f32>,
    /// Whether to enforce URDF effort (torque) limits.
    pub enforce_torque_limits: bool,
    /// Whether to retract legs after extension.
    pub enable_retract: bool,
    /// Link name to track in the graph (position/velocity/acceleration).
    pub graph_link: Option<String>,
    /// PD position gain Kp (N·m/rad).
    pub pd_kp: f64,
    /// PD derivative gain Kd (N·m·s/rad).
    pub pd_kd: f64,
}

/// Output returned from the WASM plugin to the host.
#[derive(Serialize, Deserialize)]
pub struct JumpSimOutput {
    /// Whether the simulation completed successfully.
    pub ok: bool,
    /// Error message if `ok` is false.
    pub error: Option<String>,
    /// Simulation result (present when `ok` is true).
    pub result: Option<dynamics::JumpSimResult>,
}

// ======================================================================
//  WASM-exported functions
// ======================================================================

// We use a simple protocol:
//   1. Host writes JSON bytes into WASM linear memory via `alloc()`
//   2. Host calls `run_jump_sim(ptr, len)` → returns ptr to output
//   3. Host reads output length from `last_output_len()`, then reads bytes
//   4. Host calls `dealloc(ptr, len)` to free

static mut LAST_OUTPUT: Vec<u8> = Vec::new();

/// Allocate `size` bytes in WASM memory and return the pointer.
#[unsafe(no_mangle)]
pub extern "C" fn alloc(size: usize) -> *mut u8 {
    let mut buf = Vec::with_capacity(size);
    let ptr = buf.as_mut_ptr();
    std::mem::forget(buf);
    ptr
}

/// Deallocate a previously allocated buffer.
/// # Safety
/// `ptr` must have been returned by `alloc` with at least `size` capacity.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn dealloc(ptr: *mut u8, size: usize) {
    let _ = unsafe { Vec::from_raw_parts(ptr, 0, size) };
}

/// Return the length of the last output produced by `run_jump_sim`.
#[unsafe(no_mangle)]
pub extern "C" fn last_output_len() -> usize {
    unsafe { (*std::ptr::addr_of!(LAST_OUTPUT)).len() }
}

/// Return a pointer to the last output bytes.
#[unsafe(no_mangle)]
pub extern "C" fn last_output_ptr() -> *const u8 {
    unsafe { (*std::ptr::addr_of!(LAST_OUTPUT)).as_ptr() }
}

/// Run the full jump simulation and store the JSON result.
///
/// # Safety
/// `input_ptr` must point to `input_len` valid bytes (allocated via `alloc`).
///
/// # Returns
/// 0 on success, 1 on error.  Read the result via `last_output_ptr` / `last_output_len`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn run_jump_sim(input_ptr: *const u8, input_len: usize) -> u32 {
    let input_bytes = unsafe { std::slice::from_raw_parts(input_ptr, input_len) };

    let output = match run_jump_sim_inner(input_bytes) {
        Ok(result) => JumpSimOutput {
            ok: true,
            error: None,
            result: Some(result),
        },
        Err(e) => JumpSimOutput {
            ok: false,
            error: Some(e),
            result: None,
        },
    };

    let json = serde_json::to_vec(&output).unwrap_or_else(|e| {
        let fallback = JumpSimOutput {
            ok: false,
            error: Some(format!("JSON serialization failed: {e}")),
            result: None,
        };
        serde_json::to_vec(&fallback).unwrap()
    });

    unsafe { *std::ptr::addr_of_mut!(LAST_OUTPUT) = json; }
    if output.ok { 0 } else { 1 }
}

// ======================================================================
//  Core simulation logic
// ======================================================================

fn run_jump_sim_inner(input_bytes: &[u8]) -> Result<dynamics::JumpSimResult, String> {
    let input: JumpSimInput =
        serde_json::from_slice(input_bytes).map_err(|e| format!("JSON parse error: {e}"))?;

    let mut model = input.model;

    let mut sim = dynamics::start_jump_sim(
        &mut model,
        &input.ground_links,
        input.body_link.as_deref(),
        input.speed,
        &input.locked_joints,
        input.launch_axes,
        input.extension_duration,
        input.enforce_torque_limits,
        input.enable_retract,
        input.graph_link.as_deref(),
        input.pd_kp,
        input.pd_kd,
    )
    .ok_or_else(|| "Failed to initialise jump simulation".to_string())?;

    // Run the simulation to completion.
    // Use a 60 Hz frame rate (same as the GUI); each call subdivides
    // into 0.5 ms physics sub-steps internally.
    let frame_dt = 1.0 / 60.0_f32;
    let max_frames = 10_000; // safety cap (~167 s of sim time)

    for _ in 0..max_frames {
        let running = dynamics::step_jump_sim(&mut sim, &mut model, frame_dt);
        if !running {
            break;
        }
    }

    let result = dynamics::extract_jump_result(&sim, &model);
    Ok(result)
}
