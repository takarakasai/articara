// ==========================================================================
// goal_pose_recovery_demo.rhai — Demonstrate the goal-pose mode that
// makes the body actively return to its target after a lateral push.
//
// What it shows:
//   - FullCentroidal controller in goal-pose mode (gait_set_goal_pose)
//   - Body walks toward an absolute (x, y, yaw) goal in the world frame
//   - When pushed off the path, the recovered cmd has a non-zero
//     vy_body component → body actively recovers to y = 0
//   - Equivalent to legged_control's `goalToTargetTrajectories`
//     (ROS `/move_base_simple/goal`) behaviour.
//
// Compare against `walk_demo.rhai`: that one uses gait_set_velocity
// (cmd_vel mode) which leaves the body permanently offset after a
// push. This script switches to goal_pose mode and recovers.
// ==========================================================================

let GOAL_X         = 1.0;        // m, absolute world x
let GOAL_Y         = 0.0;        // m, absolute world y
let GOAL_YAW       = 0.0;        // rad
let MAX_V          = 0.15;       // m/s
let MAX_WZ         = 0.5;        // rad/s
let WALK_DURATION_S = 8.0;
let SETTLE_S       = 0.5;

let CYCLE_PERIOD_S = 0.4;
let SWING_HEIGHT_M = 0.04;
let DUTY           = 0.5;

let ARMATURE       = 1.4e-3;
let JOINT_DAMPING  = 0.1;
let KP             = 100.0;      // stiff PD (.misa-style fidelity)
let KV             = 1.2;

let CSV_PATH       = "log/goal_pose_recovery.csv";

// ── Sanity ─────────────────────────────────────────────────────────────────

if !has_model() {
    print("ERROR: load a quadruped URDF first");
    return;
}
print(`Model: ${model_name()}, ${num_joints()} joints`);

// ── Stiff PD + computed-torque (matches the recovery benchmark) ───────────

set_actuator_mode_all("computedtorque");
set_armature_all(ARMATURE);
set_joint_damping_all(JOINT_DAMPING);
set_kp_all(KP);
set_kv_all(KV);

// ── Build sim + gait ──────────────────────────────────────────────────────

if mj_active() { mj_stop(); }
if !mj_start() {
    print("ERROR: mj_start() failed");
    return;
}
mj_set_trace_max(30000);
reset_peaks();

set_gait_mode("FullCentroidal");

if !gait_setup() {
    print("ERROR: gait_setup() failed — check foot link names");
    mj_stop();
    return;
}
gait_set_cycle_period(CYCLE_PERIOD_S);
gait_set_swing_height(SWING_HEIGHT_M);
gait_set_duty(DUTY);
print("Gait controller built (Trot, FullCentroidal)");

// ── Schedule the walk ─────────────────────────────────────────────────────

mj_async_clear();

// Settle in stance so the body drops onto its feet.
mj_async_print(`[goal] settle ${SETTLE_S} s`);
mj_async_step_seconds(SETTLE_S);

gait_start();

// ★ KEY: activate goal-pose mode instead of velocity mode.
//   The body will walk toward (GOAL_X, GOAL_Y, GOAL_YAW) in world frame,
//   and a disturbance that drifts y off zero will produce a vy command
//   pulling the body back. Compare to gait_set_velocity which would
//   keep y wherever it ends up.
if !gait_set_goal_pose(GOAL_X, GOAL_Y, GOAL_YAW, MAX_V, MAX_WZ) {
    print("ERROR: gait_set_goal_pose() failed");
    print("       (goal-pose mode only works in FullCentroidal mode today)");
    mj_stop();
    return;
}
print(`Goal pose set: (${GOAL_X}, ${GOAL_Y}, ${GOAL_YAW}) at max_v=${MAX_V}`);
print("Push the body laterally (Drag-Force tool in the viewport, or");
print("`mj_apply_external_force_root` for scripted push) and watch it");
print("return to y = 0 instead of drifting off-track.");

mj_async_print(`[goal] tracking absolute goal for ${WALK_DURATION_S} s`);
mj_async_step_seconds(WALK_DURATION_S);

// Clean exit: clear goal and zero out the velocity command. The
// controller will hold in stance once it sees cmd = 0.
gait_clear_goal_pose();
gait_set_velocity(0.0, 0.0, 0.0);
mj_async_print("[goal] cleared goal, settling");
mj_async_step_seconds(SETTLE_S);

mj_async_save_csv(CSV_PATH);
mj_async_print(`[goal] done — CSV at ${CSV_PATH}`);

print(`Queued ${mj_async_pending()} ops. Watch the viewport!`);
