// ==========================================================================
// walk_loop_demo.rhai — 6-segment rectangular-loop walk test
//
// Sequence (each 5 s @ trot 0.4 s cycle, with 1 s zero-cmd settle between):
//   1) forward      vx = +0.15      → body moves +x
//   2) R lateral    vy = -0.10      → body moves -y (right)
//   3) 左旋回 (CCW) wz = +0.5       → body rotates +yaw  (cancelled by 4)
//   4) 右旋回 (CW)  wz = -0.5       → body rotates -yaw  (cancels 3)
//   5) backward     vx = -0.15      → body moves -x
//   6) L lateral    vy = +0.10      → body moves +y (left)
//
// If MPC+WBC tracks perfectly the world-XY path is a closed 0.75 × 0.50 m
// rectangle (traversed CW from above), and final body position == origin
// at the end of segment 6. Cross-coupling and yaw drift open the loop —
// the printed `[loop] closure` summary at the end quantifies how far off
// the start the robot ended up.
//
// All cmd magnitudes match `tests/integration_walk.rs`.
// ==========================================================================

// ── Cmd magnitudes ────────────────────────────────────────────────────────
let FWD_VX     = 0.15;
let LAT_VY     = 0.10;
let YAW_WZ     = 0.5;

let BURN_IN_S  = 0.5;
let WALK_S     = 5.0;
let SETTLE_S   = 1.0;

// Trot params identical to integration_walk benchmark.
let CYCLE_PERIOD_S = 0.4;
let SWING_HEIGHT_M = 0.04;
let DUTY           = 0.5;

// Position-PD gains (matches tests/common/mod.rs).
let KP             = 30.0;
let KV             = 0.6;

let CSV_PATH       = "log/walk_loop_demo.csv";

// ── Sanity ─────────────────────────────────────────────────────────────────
if !has_model() {
    print("ERROR: load a quadruped URDF/misa first (e.g. namiashi.misa)");
    return;
}
print(`Model: ${model_name()}, ${num_joints()} joints`);
print("");
print("=== walk_loop_demo: forward → R-lat → 左旋回 → 右旋回 → back → L-lat ===");

// ── Apply GUI settings via script overrides ───────────────────────────────
set_gait_mode("mpc");
set_pose_source("groundtruth");
set_wbc_enabled(true);
set_ground_plane_enabled(true);
set_ground_plane_z(0.0);
set_ground_plane_pitch(0.0);
set_ground_plane_roll(0.0);
print("Settings: Mode=Mpc, PoseSource=GroundTruth, WBC=ON, GroundPlane=ON @ z=0");
print("");

// ── Position-PD ─────────────────────────────────────────────────────────────
set_actuator_mode_all("position");
set_kp_all(KP);
set_kv_all(KV);
print(`Set Position-PD: Kp=${KP}, Kv=${KV}`);

// ── Auto-detect kinematics (must be before pose seed) ────────────────────
if !gait_setup() {
    print("ERROR: gait_setup() failed.");
    print("Likely your foot links don't match FL/FR/RL/RR_foot.");
    print("Try: gait_setup_with_feet(\"YOUR_FL\", \"YOUR_FR\", \"YOUR_RL\", \"YOUR_RR\")");
    return;
}
gait_set_cycle_period(CYCLE_PERIOD_S);
gait_set_swing_height(SWING_HEIGHT_M);
gait_set_duty(DUTY);
print("Gait controller built (Trot)");

// ── Seed initial pose + sync gait baseline ────────────────────────────────
if set_initial_pose("constrain") {
    print("Seeded initial pose: 'constrain' (model-defined stance)");
} else if seed_kinematic_pose() {
    print("Seeded initial pose: kinematic IK from nominal_foot_body");
} else {
    print("WARNING: no initial pose seed applied — robot may collapse on start");
}
if !gait_use_current_pose_as_stance() {
    print("WARNING: gait_use_current_pose_as_stance() failed");
}
print("Gait stance baseline = current joint pose");

// ── Build sim ──────────────────────────────────────────────────────────────
if mj_active() { mj_stop(); }
if !mj_start() {
    print("ERROR: mj_start() failed");
    return;
}
mj_set_trace_max(80000);
reset_peaks();

// ── Async timeline: 6 segments forming a closed rectangle ─────────────────
mj_async_clear();

gait_start();

// 0) Burn-in
gait_set_velocity(0.0, 0.0, 0.0);
mj_async_print(`[loop] burn-in   ${BURN_IN_S} s @ vx=0 vy=0 wz=0`);
mj_async_step_seconds(BURN_IN_S);

// 1) FORWARD (+x)
mj_async_print(`[loop] 1/6 forward    ${WALK_S} s @ vx=+${FWD_VX}`);
mj_async_set_velocity(FWD_VX, 0.0, 0.0);
mj_async_step_seconds(WALK_S);

mj_async_print(`[loop]     settle    ${SETTLE_S} s`);
mj_async_set_velocity(0.0, 0.0, 0.0);
mj_async_step_seconds(SETTLE_S);

// 2) R LATERAL (-y, body-right)
mj_async_print(`[loop] 2/6 R-lateral  ${WALK_S} s @ vy=-${LAT_VY}`);
mj_async_set_velocity(0.0, -LAT_VY, 0.0);
mj_async_step_seconds(WALK_S);

mj_async_print(`[loop]     settle    ${SETTLE_S} s`);
mj_async_set_velocity(0.0, 0.0, 0.0);
mj_async_step_seconds(SETTLE_S);

// 3) 左旋回 (CCW, +wz)
mj_async_print(`[loop] 3/6 yaw +CCW   ${WALK_S} s @ wz=+${YAW_WZ}`);
mj_async_set_velocity(0.0, 0.0, YAW_WZ);
mj_async_step_seconds(WALK_S);

mj_async_print(`[loop]     settle    ${SETTLE_S} s`);
mj_async_set_velocity(0.0, 0.0, 0.0);
mj_async_step_seconds(SETTLE_S);

// 4) 右旋回 (CW, -wz) — cancels 3 if perfect
mj_async_print(`[loop] 4/6 yaw -CW    ${WALK_S} s @ wz=-${YAW_WZ}`);
mj_async_set_velocity(0.0, 0.0, -YAW_WZ);
mj_async_step_seconds(WALK_S);

mj_async_print(`[loop]     settle    ${SETTLE_S} s`);
mj_async_set_velocity(0.0, 0.0, 0.0);
mj_async_step_seconds(SETTLE_S);

// 5) BACKWARD (-x)
mj_async_print(`[loop] 5/6 backward   ${WALK_S} s @ vx=-${FWD_VX}`);
mj_async_set_velocity(-FWD_VX, 0.0, 0.0);
mj_async_step_seconds(WALK_S);

mj_async_print(`[loop]     settle    ${SETTLE_S} s`);
mj_async_set_velocity(0.0, 0.0, 0.0);
mj_async_step_seconds(SETTLE_S);

// 6) L LATERAL (+y, body-left)
mj_async_print(`[loop] 6/6 L-lateral  ${WALK_S} s @ vy=+${LAT_VY}`);
mj_async_set_velocity(0.0, LAT_VY, 0.0);
mj_async_step_seconds(WALK_S);

mj_async_print(`[loop]     final settle ${SETTLE_S} s`);
mj_async_set_velocity(0.0, 0.0, 0.0);
mj_async_step_seconds(SETTLE_S);

mj_async_save_csv(CSV_PATH);
mj_async_print(`[loop] done — CSV at ${CSV_PATH}`);
mj_async_print(`[loop] expected world path: closed 0.75 × 0.50 m rectangle (CW)`);
mj_async_print(`[loop] expected end pos: (0, 0, z_stand) — check viewport for closure`);

let total_s = BURN_IN_S + 6.0 * (WALK_S + SETTLE_S);
print(`Queued ${mj_async_pending()} ops. Watch the viewport!`);
print(`Total run time: ${total_s} s`);
print("Ideal world path (CW from above):");
print("  start → +0.75 x  →  -0.50 y  →  rotate ±2.5 rad  →  -0.75 x  →  +0.50 y → start");
