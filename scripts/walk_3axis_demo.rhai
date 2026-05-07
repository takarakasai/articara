// ==========================================================================
// walk_3axis_demo.rhai — Reproduce the integration_walk benchmark
// (forward / lateral / yaw) inside the GUI so the user can SEE the trot.
//
// Pipeline:
//   1. Configure joints for Position-PD (matches the regression test stack).
//   2. (Re)build a fresh MuJoCo sim.
//   3. Auto-detect quadruped kinematics from the loaded URDF.
//   4. Schedule 3 successive 5 s walks: forward → lateral → yaw, with
//      brief 1 s zero-cmd settle phases between, then a final stop.
//   5. Save CSV trace of the body trajectory.
//
// Prerequisites — set these in the Quadruped gait panel BEFORE running:
//   - Generator   = MPC (capture-point)
//   - Pose source = MuJoCo ground truth
//   - Hierarchical WBC = ON
//
// Without WBC, lateral / yaw motion will look very different from the
// integration_walk benchmark numbers. With WBC + ground truth + MPC,
// the script reproduces P5b's per-cmd swing_leg scheduling that lets
// all three axes follow the cmd cleanly.
//
// Run from articara: Console (📂) → select this file. Watch the
// viewport. CSV at log/walk_3axis_demo.csv when done.
//
// Reference (same constants as `tests/integration_walk.rs`):
//   FORWARD_CMD_VX = 0.15 m/s
//   LATERAL_CMD_VY = 0.10 m/s
//   YAW_CMD_WZ     = 0.5 rad/s
//   SIM time/axis  = 5.0 s    (matches WALK_SIM_TIME_S)
//   burn-in        = 0.5 s    (matches WALK_BURN_IN_S)
// ==========================================================================

// ── Axis cmd magnitudes (must match tests/integration_walk.rs) ─────────────

let FWD_VX     = 0.15;
let LAT_VY     = 0.10;
let YAW_WZ     = 0.5;

let BURN_IN_S  = 0.5;     // settle before each cmd
let WALK_S     = 5.0;     // active cmd duration per axis
let SETTLE_S   = 1.0;     // zero-cmd handoff between axes

// Trot params identical to integration_walk benchmark.
let CYCLE_PERIOD_S = 0.4;
let SWING_HEIGHT_M = 0.04;
let DUTY           = 0.5;

// Position-PD gains (matches tests/common/mod.rs).
let KP             = 30.0;
let KV             = 0.6;

let CSV_PATH       = "log/walk_3axis_demo.csv";

// ── Sanity ─────────────────────────────────────────────────────────────────

if !has_model() {
    print("ERROR: load a quadruped URDF first (e.g. namiashi.urdf)");
    return;
}
print(`Model: ${model_name()}, ${num_joints()} joints`);
print("");
print("=== walk_3axis_demo: forward → lateral → yaw ===");
print("Prereqs (set via GUI before pressing Run):");
print("  Generator   = MPC (capture-point)");
print("  Pose source = MuJoCo ground truth");
print("  Hierarchical WBC = ON");
print("");

// ── Configure joints for Position-PD ──────────────────────────────────────

set_actuator_mode_all("position");
set_kp_all(KP);
set_kv_all(KV);
print(`Set Position-PD: Kp=${KP}, Kv=${KV}`);

// ── Build / rebuild sim ────────────────────────────────────────────────────

if mj_active() { mj_stop(); }
if !mj_start() {
    print("ERROR: mj_start() failed — check the model has a base link");
    return;
}
mj_set_trace_max(40000);
reset_peaks();

// ── Auto-detect quadruped kinematics ──────────────────────────────────────

if !gait_setup() {
    print("ERROR: gait_setup() failed.");
    print("Likely your foot links don't match FL/FR/RL/RR_foot.");
    print("Try: gait_setup_with_feet(\"YOUR_FL\", \"YOUR_FR\", \"YOUR_RL\", \"YOUR_RR\")");
    mj_stop();
    return;
}
gait_set_cycle_period(CYCLE_PERIOD_S);
gait_set_swing_height(SWING_HEIGHT_M);
gait_set_duty(DUTY);
print("Gait controller built (Trot)");

// ── Async timeline: 3 axes back to back ────────────────────────────────────
// Each axis: zero-cmd burn-in → active cmd → zero-cmd settle.
// gait_start runs synchronously; the async queue paces the stepping.

mj_async_clear();

gait_start();

// 1) Burn-in (let body settle at zero cmd)
gait_set_velocity(0.0, 0.0, 0.0);
mj_async_print(`[walk] burn-in  ${BURN_IN_S} s @ vx=0 vy=0 wz=0`);
mj_async_step_seconds(BURN_IN_S);

// 2) FORWARD axis
mj_async_print(`[walk] forward  ${WALK_S} s @ vx=${FWD_VX}`);
mj_async_set_velocity(FWD_VX, 0.0, 0.0);
mj_async_step_seconds(WALK_S);

// 3) Settle between axes
mj_async_print(`[walk] settle   ${SETTLE_S} s`);
mj_async_set_velocity(0.0, 0.0, 0.0);
mj_async_step_seconds(SETTLE_S);

// 4) LATERAL axis
mj_async_print(`[walk] lateral  ${WALK_S} s @ vy=${LAT_VY}`);
mj_async_set_velocity(0.0, LAT_VY, 0.0);
mj_async_step_seconds(WALK_S);

// 5) Settle
mj_async_print(`[walk] settle   ${SETTLE_S} s`);
mj_async_set_velocity(0.0, 0.0, 0.0);
mj_async_step_seconds(SETTLE_S);

// 6) YAW axis
mj_async_print(`[walk] yaw      ${WALK_S} s @ wz=${YAW_WZ}`);
mj_async_set_velocity(0.0, 0.0, YAW_WZ);
mj_async_step_seconds(WALK_S);

// 7) Final stop
mj_async_print(`[walk] stop     ${SETTLE_S} s`);
mj_async_set_velocity(0.0, 0.0, 0.0);
mj_async_step_seconds(SETTLE_S);

mj_async_save_csv(CSV_PATH);
mj_async_print(`[walk] done — CSV at ${CSV_PATH}`);

print(`Queued ${mj_async_pending()} ops. Watch the viewport!`);
print("Total run time: " +
    `${BURN_IN_S + 3.0 * (WALK_S + SETTLE_S)} s`);
print("Each axis transition makes the trot direction change in real time.");
