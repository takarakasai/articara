//! Articara WASM plugin — command-dispatch architecture.
//!
//! Exports a single entry point `execute(ptr, len)` that accepts a JSON
//! [`Request`](articara_plugin_api::Request) envelope, dispatches to the
//! appropriate command handler, and stores the JSON
//! [`Response`](articara_plugin_api::Response) for the host to read back.
//!
//! ## Available commands
//!
//! | Command            | Description                                             |
//! |--------------------|---------------------------------------------------------|
//! | `list_commands`    | Enumerate available commands and metadata                |
//! | `jump_sim`         | Full jump simulation (Extension→Flight→Landed)           |
//! | `static_analysis`  | Gravity torques + payload capacity + jump height         |
//! | `gravity_torques`  | Per-joint static gravity torque                          |
//! | `payload_capacity` | Max payload mass at an end-effector                      |
//! | `jump_height`      | Energy-based jump height estimate                        |
//! | `payload_sim`      | Animated payload ramp-up simulation                      |
//!
//! ## Memory protocol
//!
//! 1. Host calls `alloc(len)` → ptr
//! 2. Host writes JSON request into `memory[ptr..ptr+len]`
//! 3. Host calls `execute(ptr, len)` → 0 (ok) / 1 (error)
//! 4. Host reads `last_output_len()` and `last_output_ptr()`
//! 5. Host reads response bytes from linear memory
//! 6. Host calls `dealloc(ptr, len)` to free the input buffer

use serde::{Deserialize, Serialize};
use std::collections::HashSet;

use articara::dynamics;
use articara::robot::RobotModel;
use articara_plugin_api as api;

// ======================================================================
//  WASM-exported functions (ABI layer)
// ======================================================================

static mut LAST_OUTPUT: Vec<u8> = Vec::new();

#[unsafe(no_mangle)]
pub extern "C" fn alloc(size: usize) -> *mut u8 {
    let mut buf = Vec::with_capacity(size);
    let ptr = buf.as_mut_ptr();
    std::mem::forget(buf);
    ptr
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn dealloc(ptr: *mut u8, size: usize) {
    let _ = unsafe { Vec::from_raw_parts(ptr, 0, size) };
}

#[unsafe(no_mangle)]
pub extern "C" fn last_output_len() -> usize {
    unsafe { (*std::ptr::addr_of!(LAST_OUTPUT)).len() }
}

#[unsafe(no_mangle)]
pub extern "C" fn last_output_ptr() -> *const u8 {
    unsafe { (*std::ptr::addr_of!(LAST_OUTPUT)).as_ptr() }
}

/// Single entry point: dispatch a JSON [`Request`] to the appropriate handler.
///
/// # Safety
/// `input_ptr` must point to `input_len` valid bytes (allocated via `alloc`).
///
/// # Returns
/// 0 on success, 1 on error.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn execute(input_ptr: *const u8, input_len: usize) -> u32 {
    let input_bytes = unsafe { std::slice::from_raw_parts(input_ptr, input_len) };

    let response = match serde_json::from_slice::<api::Request>(input_bytes) {
        Ok(req) => dispatch(req),
        Err(e) => api::Response::err("(parse)", format!("Invalid request JSON: {e}")),
    };

    let json = serde_json::to_vec(&response).unwrap_or_else(|e| {
        serde_json::to_vec(&api::Response::err(
            "(serialize)",
            format!("JSON serialization failed: {e}"),
        ))
        .unwrap()
    });

    let ok = response.ok;
    unsafe {
        *std::ptr::addr_of_mut!(LAST_OUTPUT) = json;
    }
    if ok { 0 } else { 1 }
}

// ── Backward-compatible export ──────────────────────────────────────

/// Legacy export: runs `jump_sim` command directly.
/// Kept for backward compatibility with existing hosts.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn run_jump_sim(input_ptr: *const u8, input_len: usize) -> u32 {
    let input_bytes = unsafe { std::slice::from_raw_parts(input_ptr, input_len) };

    let req = api::Request {
        version: 1,
        command: "jump_sim".into(),
        params: match serde_json::from_slice::<serde_json::Value>(input_bytes) {
            Ok(v) => v,
            Err(e) => {
                let resp = api::Response::err("jump_sim", format!("JSON parse error: {e}"));
                let json = serde_json::to_vec(&resp).unwrap();
                unsafe {
                    *std::ptr::addr_of_mut!(LAST_OUTPUT) = json;
                }
                return 1;
            }
        },
    };
    let response = dispatch(req);
    let json = serde_json::to_vec(&response).unwrap();
    let ok = response.ok;
    unsafe {
        *std::ptr::addr_of_mut!(LAST_OUTPUT) = json;
    }
    if ok { 0 } else { 1 }
}

// ======================================================================
//  Command dispatcher
// ======================================================================

type Handler = fn(serde_json::Value) -> Result<(Vec<api::View>, serde_json::Value), String>;

fn dispatch(req: api::Request) -> api::Response {
    let handler: Handler = match req.command.as_str() {
        "list_commands" => cmd_list_commands,
        "jump_sim" => cmd_jump_sim,
        "static_analysis" => cmd_static_analysis,
        "gravity_torques" => cmd_gravity_torques,
        "payload_capacity" => cmd_payload_capacity,
        "jump_height" => cmd_jump_height,
        "payload_sim" => cmd_payload_sim,
        _ => {
            return api::Response::err(
                req.command,
                "Unknown command. Use `list_commands` to see available commands.",
            );
        }
    };

    match handler(req.params) {
        Ok((views, data)) => api::Response::ok(req.command, views, data),
        Err(e) => api::Response::err(req.command, e),
    }
}

// ======================================================================
//  Command: list_commands
// ======================================================================

fn cmd_list_commands(
    _params: serde_json::Value,
) -> Result<(Vec<api::View>, serde_json::Value), String> {
    let commands = vec![
        api::CommandInfo {
            name: "list_commands".into(),
            description: "Enumerate available commands and metadata".into(),
            category: Some("meta".into()),
        },
        api::CommandInfo {
            name: "jump_sim".into(),
            description: "Run full jump simulation (Extension→Flight→Landed)".into(),
            category: Some("simulation".into()),
        },
        api::CommandInfo {
            name: "static_analysis".into(),
            description: "Gravity torques + payload capacity + jump height estimate".into(),
            category: Some("analysis".into()),
        },
        api::CommandInfo {
            name: "gravity_torques".into(),
            description: "Per-joint static gravity torque".into(),
            category: Some("analysis".into()),
        },
        api::CommandInfo {
            name: "payload_capacity".into(),
            description: "Max payload mass at an end-effector".into(),
            category: Some("analysis".into()),
        },
        api::CommandInfo {
            name: "jump_height".into(),
            description: "Energy-based jump height estimate".into(),
            category: Some("analysis".into()),
        },
        api::CommandInfo {
            name: "payload_sim".into(),
            description: "Animated payload ramp-up simulation".into(),
            category: Some("simulation".into()),
        },
    ];

    let mut views = vec![
        api::View::Heading {
            text: "articara-dynamics plugin".into(),
            level: 1,
        },
        api::View::Scalars {
            title: Some("Plugin Info".into()),
            items: vec![
                api::ScalarItem {
                    label: "Version".into(),
                    value: env!("CARGO_PKG_VERSION").into(),
                    numeric: None,
                    emphasis: None,
                },
                api::ScalarItem {
                    label: "Commands".into(),
                    value: format!("{}", commands.len()),
                    numeric: Some(commands.len() as f64),
                    emphasis: None,
                },
            ],
        },
    ];

    let columns = vec![
        api::Column { name: "Command".into(), align: Some("left".into()) },
        api::Column { name: "Category".into(), align: Some("left".into()) },
        api::Column { name: "Description".into(), align: Some("left".into()) },
    ];
    let rows: Vec<Vec<api::Cell>> = commands
        .iter()
        .map(|c| {
            vec![
                api::Cell::Text { value: c.name.clone() },
                api::Cell::Tag {
                    value: c.category.clone().unwrap_or_default(),
                    color: Some(
                        match c.category.as_deref() {
                            Some("simulation") => "green",
                            Some("analysis") => "yellow",
                            _ => "gray",
                        }
                        .into(),
                    ),
                },
                api::Cell::Text { value: c.description.clone() },
            ]
        })
        .collect();

    views.push(api::View::Table {
        title: Some("Available Commands".into()),
        columns,
        rows,
    });

    let data = serde_json::to_value(&commands).unwrap();
    Ok((views, data))
}

// ======================================================================
//  Command: jump_sim
// ======================================================================

#[derive(Deserialize)]
struct JumpSimParams {
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

fn cmd_jump_sim(
    params: serde_json::Value,
) -> Result<(Vec<api::View>, serde_json::Value), String> {
    let input: JumpSimParams =
        serde_json::from_value(params).map_err(|e| format!("Invalid params: {e}"))?;

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

    let frame_dt = 1.0 / 60.0_f32;
    for _ in 0..10_000 {
        if !dynamics::step_jump_sim(&mut sim, &mut model, frame_dt) {
            break;
        }
    }

    let result = dynamics::extract_jump_result(&sim, &model);

    // ── Build views ──
    let mut views = Vec::new();

    views.push(api::View::Scalars {
        title: Some("Summary".into()),
        items: vec![
            api::ScalarItem {
                label: "Reached Height".into(),
                value: format!("{:.4} m", result.max_height),
                numeric: Some(result.max_height as f64),
                emphasis: Some("primary".into()),
            },
            api::ScalarItem {
                label: "Extension Duration".into(),
                value: format!("{:.3} s", result.extension_duration),
                numeric: Some(result.extension_duration as f64),
                emphasis: None,
            },
        ],
    });

    if !result.joint_peaks.is_empty() {
        let columns = vec![
            api::Column { name: "Joint".into(), align: Some("left".into()) },
            api::Column { name: "Peak τ (N·m)".into(), align: Some("right".into()) },
            api::Column { name: "θ@τ (deg)".into(), align: Some("right".into()) },
            api::Column { name: "Peak ω (rad/s)".into(), align: Some("right".into()) },
            api::Column { name: "θ@ω (deg)".into(), align: Some("right".into()) },
            api::Column { name: "Role".into(), align: Some("center".into()) },
        ];
        let rows: Vec<Vec<api::Cell>> = result
            .joint_peaks
            .iter()
            .map(|jp| {
                vec![
                    api::Cell::Text { value: jp.joint_name.clone() },
                    api::Cell::Number { value: jp.peak_torque, format: Some(".3f".into()) },
                    api::Cell::Number {
                        value: jp.peak_torque_angle.to_degrees(),
                        format: Some(".1f".into()),
                    },
                    api::Cell::Number { value: jp.peak_velocity, format: Some(".3f".into()) },
                    api::Cell::Number {
                        value: jp.peak_velocity_angle.to_degrees(),
                        format: Some(".1f".into()),
                    },
                    api::Cell::Tag {
                        value: if jp.contributes { "drive" } else { "hold" }.into(),
                        color: Some(if jp.contributes { "green" } else { "gray" }.into()),
                    },
                ]
            })
            .collect();

        views.push(api::View::Table { title: Some("Per-Joint Peaks".into()), columns, rows });
    }

    // Graph plots
    let gd = &result.graph_data;
    if !gd.time.is_empty() {
        let time_ms: Vec<f64> = gd.time.iter().map(|t| *t as f64 * 1000.0).collect();

        let make_xyz_plot =
            |title: &str, y_label: &str, xs: &[f32], ys: &[f32], zs: &[f32]| -> api::View {
                api::View::LinePlot {
                    title: title.into(),
                    x_label: "Time (ms)".into(),
                    y_label: y_label.into(),
                    series: vec![
                        api::Series {
                            name: "X".into(),
                            x: time_ms.clone(),
                            y: xs.iter().map(|v| *v as f64).collect(),
                            color: Some("#FF6464".into()),
                        },
                        api::Series {
                            name: "Y".into(),
                            x: time_ms.clone(),
                            y: ys.iter().map(|v| *v as f64).collect(),
                            color: Some("#64FF64".into()),
                        },
                        api::Series {
                            name: "Z".into(),
                            x: time_ms.clone(),
                            y: zs.iter().map(|v| *v as f64).collect(),
                            color: Some("#6464FF".into()),
                        },
                    ],
                }
            };

        views.push(make_xyz_plot("Position (m)", "m", &gd.pos_x, &gd.pos_y, &gd.pos_z));
        views.push(make_xyz_plot("Velocity (m/s)", "m/s", &gd.vel_x, &gd.vel_y, &gd.vel_z));
        views.push(make_xyz_plot(
            "Acceleration (m/s²)",
            "m/s²",
            &gd.acc_x,
            &gd.acc_y,
            &gd.acc_z,
        ));
    }

    let data = serde_json::to_value(&result).map_err(|e| format!("Serialization error: {e}"))?;
    Ok((views, data))
}

// ======================================================================
//  Command: gravity_torques
// ======================================================================

#[derive(Deserialize)]
struct ModelOnlyParams {
    model: RobotModel,
}

fn cmd_gravity_torques(
    params: serde_json::Value,
) -> Result<(Vec<api::View>, serde_json::Value), String> {
    let input: ModelOnlyParams =
        serde_json::from_value(params).map_err(|e| format!("Invalid params: {e}"))?;

    let torques = dynamics::compute_gravity_torques(&input.model);

    let mut views = Vec::new();

    let columns = vec![
        api::Column { name: "Joint".into(), align: Some("left".into()) },
        api::Column { name: "Gravity τ (N·m)".into(), align: Some("right".into()) },
        api::Column { name: "Effort Limit".into(), align: Some("right".into()) },
        api::Column { name: "Margin".into(), align: Some("right".into()) },
    ];
    let rows: Vec<Vec<api::Cell>> = torques
        .iter()
        .map(|t| {
            vec![
                api::Cell::Text { value: t.joint_name.clone() },
                api::Cell::Number { value: t.gravity_torque, format: Some(".4f".into()) },
                api::Cell::Number { value: t.effort_limit, format: Some(".2f".into()) },
                api::Cell::Number { value: t.torque_margin, format: Some(".4f".into()) },
            ]
        })
        .collect();

    views.push(api::View::Table { title: Some("Joint Gravity Torques".into()), columns, rows });

    let bars: Vec<api::Bar> = torques
        .iter()
        .filter(|t| t.effort_limit > 0.0)
        .map(|t| {
            let util = t.gravity_torque.abs() / t.effort_limit;
            api::Bar {
                label: t.joint_name.clone(),
                value: util,
                color: Some(util_color(util).into()),
                tag: None,
            }
        })
        .collect();

    if !bars.is_empty() {
        views.push(api::View::BarChart {
            title: "Torque Utilisation".into(),
            bars,
            max_value: Some(1.5),
        });
    }

    let data = serde_json::to_value(&torques).map_err(|e| format!("Serialization error: {e}"))?;
    Ok((views, data))
}

// ======================================================================
//  Command: payload_capacity
// ======================================================================

#[derive(Deserialize)]
struct PayloadCapParams {
    model: RobotModel,
    ee_link: String,
}

fn cmd_payload_capacity(
    params: serde_json::Value,
) -> Result<(Vec<api::View>, serde_json::Value), String> {
    let input: PayloadCapParams =
        serde_json::from_value(params).map_err(|e| format!("Invalid params: {e}"))?;

    let mut torques = dynamics::compute_gravity_torques(&input.model);
    let result =
        dynamics::compute_payload_capacity(&input.model, &input.ee_link, &mut torques)
            .ok_or_else(|| "Payload capacity computation failed".to_string())?;

    let mut views = Vec::new();

    views.push(api::View::Scalars {
        title: Some("Payload Capacity".into()),
        items: vec![
            api::ScalarItem {
                label: "Max Payload".into(),
                value: format!("{:.3} kg", result.max_mass_kg),
                numeric: Some(result.max_mass_kg),
                emphasis: Some("primary".into()),
            },
            api::ScalarItem {
                label: "Limiting Joint".into(),
                value: result.limiting_joint.clone(),
                numeric: None,
                emphasis: Some("warning".into()),
            },
            api::ScalarItem {
                label: "EE Position".into(),
                value: format!(
                    "({:.3}, {:.3}, {:.3})",
                    result.ee_position.x, result.ee_position.y, result.ee_position.z
                ),
                numeric: None,
                emphasis: None,
            },
        ],
    });

    let columns = vec![
        api::Column { name: "Joint".into(), align: Some("left".into()) },
        api::Column { name: "Gravity τ".into(), align: Some("right".into()) },
        api::Column { name: "τ/kg Payload".into(), align: Some("right".into()) },
        api::Column { name: "Effort Limit".into(), align: Some("right".into()) },
        api::Column { name: "Margin".into(), align: Some("right".into()) },
    ];
    let rows: Vec<Vec<api::Cell>> = torques
        .iter()
        .map(|t| {
            vec![
                api::Cell::Text { value: t.joint_name.clone() },
                api::Cell::Number { value: t.gravity_torque, format: Some(".4f".into()) },
                api::Cell::Number { value: t.payload_torque_per_kg, format: Some(".4f".into()) },
                api::Cell::Number { value: t.effort_limit, format: Some(".2f".into()) },
                api::Cell::Number { value: t.torque_margin, format: Some(".4f".into()) },
            ]
        })
        .collect();

    views.push(api::View::Table {
        title: Some("Joint Torques with Payload".into()),
        columns,
        rows,
    });

    #[derive(Serialize)]
    struct PayloadCapData {
        payload: dynamics::PayloadResult,
        joint_torques: Vec<dynamics::JointTorqueInfo>,
    }
    let data = serde_json::to_value(&PayloadCapData {
        payload: result,
        joint_torques: torques,
    })
    .map_err(|e| format!("Serialization error: {e}"))?;
    Ok((views, data))
}

// ======================================================================
//  Command: jump_height
// ======================================================================

#[derive(Deserialize)]
struct JumpHeightParams {
    model: RobotModel,
    ground_links: Vec<String>,
    body_link: Option<String>,
}

fn cmd_jump_height(
    params: serde_json::Value,
) -> Result<(Vec<api::View>, serde_json::Value), String> {
    let input: JumpHeightParams =
        serde_json::from_value(params).map_err(|e| format!("Invalid params: {e}"))?;

    let result = dynamics::compute_jump_height(
        &input.model,
        &input.ground_links,
        input.body_link.as_deref(),
    )
    .ok_or_else(|| "Jump height computation failed".to_string())?;

    let mut views = Vec::new();

    views.push(api::View::Scalars {
        title: Some("Jump Height Estimate".into()),
        items: vec![
            api::ScalarItem {
                label: "Max Height".into(),
                value: format!("{:.4} m", result.max_height_m),
                numeric: Some(result.max_height_m),
                emphasis: Some("primary".into()),
            },
            api::ScalarItem {
                label: "Total Energy".into(),
                value: format!("{:.3} J", result.total_energy_j),
                numeric: Some(result.total_energy_j),
                emphasis: None,
            },
            api::ScalarItem {
                label: "Total Mass".into(),
                value: format!("{:.3} kg", result.total_mass_kg),
                numeric: Some(result.total_mass_kg),
                emphasis: None,
            },
        ],
    });

    if !result.per_joint_energy.is_empty() {
        let max_e = result
            .per_joint_energy
            .iter()
            .map(|(_, e)| *e)
            .fold(0.0_f64, f64::max);
        let bars: Vec<api::Bar> = result
            .per_joint_energy
            .iter()
            .map(|(name, energy)| api::Bar {
                label: name.clone(),
                value: *energy,
                color: Some("green".into()),
                tag: Some(format!("{:.3} J", energy)),
            })
            .collect();

        views.push(api::View::BarChart {
            title: "Per-Joint Energy Contribution".into(),
            bars,
            max_value: if max_e > 0.0 { Some(max_e * 1.1) } else { None },
        });
    }

    let data = serde_json::to_value(&result).map_err(|e| format!("Serialization error: {e}"))?;
    Ok((views, data))
}

// ======================================================================
//  Command: static_analysis
// ======================================================================

#[derive(Deserialize)]
struct StaticAnalysisParams {
    model: RobotModel,
    ee_link: Option<String>,
    body_link: Option<String>,
    ground_links: Vec<String>,
}

fn cmd_static_analysis(
    params: serde_json::Value,
) -> Result<(Vec<api::View>, serde_json::Value), String> {
    let input: StaticAnalysisParams =
        serde_json::from_value(params).map_err(|e| format!("Invalid params: {e}"))?;

    let result = dynamics::analyze(
        &input.model,
        input.ee_link.as_deref(),
        input.body_link.as_deref(),
        &input.ground_links,
    );

    let mut views = Vec::new();

    // Gravity torques table
    let columns = vec![
        api::Column { name: "Joint".into(), align: Some("left".into()) },
        api::Column { name: "Gravity τ (N·m)".into(), align: Some("right".into()) },
        api::Column { name: "Effort Limit".into(), align: Some("right".into()) },
        api::Column { name: "Margin".into(), align: Some("right".into()) },
    ];
    let rows: Vec<Vec<api::Cell>> = result
        .joint_torques
        .iter()
        .map(|t| {
            vec![
                api::Cell::Text { value: t.joint_name.clone() },
                api::Cell::Number { value: t.gravity_torque, format: Some(".4f".into()) },
                api::Cell::Number { value: t.effort_limit, format: Some(".2f".into()) },
                api::Cell::Number { value: t.torque_margin, format: Some(".4f".into()) },
            ]
        })
        .collect();

    views.push(api::View::Table { title: Some("Joint Gravity Torques".into()), columns, rows });

    // Utilisation bars
    let bars: Vec<api::Bar> = result
        .joint_torques
        .iter()
        .filter(|t| t.effort_limit > 0.0)
        .map(|t| {
            let util = t.gravity_torque.abs() / t.effort_limit;
            api::Bar {
                label: t.joint_name.clone(),
                value: util,
                color: Some(util_color(util).into()),
                tag: None,
            }
        })
        .collect();
    if !bars.is_empty() {
        views.push(api::View::BarChart {
            title: "Torque Utilisation".into(),
            bars,
            max_value: Some(1.5),
        });
    }

    // Payload
    if let Some(ref p) = result.payload {
        views.push(api::View::Scalars {
            title: Some("Payload Capacity".into()),
            items: vec![
                api::ScalarItem {
                    label: "Max Payload".into(),
                    value: format!("{:.3} kg", p.max_mass_kg),
                    numeric: Some(p.max_mass_kg),
                    emphasis: Some("primary".into()),
                },
                api::ScalarItem {
                    label: "Limiting Joint".into(),
                    value: p.limiting_joint.clone(),
                    numeric: None,
                    emphasis: Some("warning".into()),
                },
            ],
        });
    }

    // Jump
    if let Some(ref j) = result.jump {
        views.push(api::View::Scalars {
            title: Some("Jump Height Estimate".into()),
            items: vec![
                api::ScalarItem {
                    label: "Max Height".into(),
                    value: format!("{:.4} m", j.max_height_m),
                    numeric: Some(j.max_height_m),
                    emphasis: Some("primary".into()),
                },
                api::ScalarItem {
                    label: "Total Energy".into(),
                    value: format!("{:.3} J", j.total_energy_j),
                    numeric: Some(j.total_energy_j),
                    emphasis: None,
                },
            ],
        });
    }

    let data = serde_json::to_value(&result).map_err(|e| format!("Serialization error: {e}"))?;
    Ok((views, data))
}

// ======================================================================
//  Command: payload_sim
// ======================================================================

#[derive(Deserialize)]
struct PayloadSimParams {
    model: RobotModel,
    ee_link: String,
    speed: f32,
}

#[derive(Serialize)]
struct PayloadSimResult {
    max_mass_kg: f64,
    limiting_joint: String,
    phases_completed: bool,
}

fn cmd_payload_sim(
    params: serde_json::Value,
) -> Result<(Vec<api::View>, serde_json::Value), String> {
    let input: PayloadSimParams =
        serde_json::from_value(params).map_err(|e| format!("Invalid params: {e}"))?;

    let mut sim = dynamics::start_payload_sim(&input.model, &input.ee_link, input.speed)
        .ok_or_else(|| "Failed to initialise payload simulation".to_string())?;

    // Run to completion
    for _ in 0..10_000 {
        if !dynamics::step_payload_sim(&mut sim, &input.model, &input.ee_link) {
            break;
        }
        sim.phase_time += 1.0 / 60.0;
    }

    let mut views = Vec::new();

    views.push(api::View::Scalars {
        title: Some("Payload Simulation Result".into()),
        items: vec![
            api::ScalarItem {
                label: "Max Payload".into(),
                value: format!("{:.3} kg", sim.max_mass),
                numeric: Some(sim.max_mass),
                emphasis: Some("primary".into()),
            },
            api::ScalarItem {
                label: "Limiting Joint".into(),
                value: sim.limiting_joint.clone(),
                numeric: None,
                emphasis: Some("warning".into()),
            },
        ],
    });

    if !sim.joint_utilisation.is_empty() {
        let bars: Vec<api::Bar> = sim
            .joint_utilisation
            .iter()
            .map(|&(ji, util)| {
                let jname = if ji < input.model.joints.len() {
                    input.model.joints[ji].name.clone()
                } else {
                    format!("joint_{ji}")
                };
                api::Bar {
                    label: jname,
                    value: util,
                    color: Some(util_color(util).into()),
                    tag: Some(format!("{:.0}%", util * 100.0)),
                }
            })
            .collect();

        views.push(api::View::BarChart {
            title: "Joint Utilisation at Max Payload".into(),
            bars,
            max_value: Some(1.5),
        });
    }

    let out = PayloadSimResult {
        max_mass_kg: sim.max_mass,
        limiting_joint: sim.limiting_joint.clone(),
        phases_completed: matches!(sim.phase, dynamics::PayloadPhase::Done),
    };
    let data = serde_json::to_value(&out).map_err(|e| format!("Serialization error: {e}"))?;
    Ok((views, data))
}

// ======================================================================
//  Helpers
// ======================================================================

/// Map a utilisation ratio (0.0 – 1.0+) to a colour name.
fn util_color(util: f64) -> &'static str {
    if util <= 0.7 {
        "green"
    } else if util <= 1.0 {
        "yellow"
    } else {
        "red"
    }
}
