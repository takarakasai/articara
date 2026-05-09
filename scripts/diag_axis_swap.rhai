// ==========================================================================
// diag_axis_swap.rhai — Setup phase only, no walking yet
//
// Purpose: prepare the robot in the same state as walk_loop_demo (MPC + WBC
// + constrain pose + gait_use_current_pose_as_stance) but DON'T start any
// motion. The user can then drive cmd via the Quadruped Gait panel's arrow
// buttons and observe whether vx → world+x or world+y, vy → world+y or
// world+x. body yaw is guaranteed 0 because no time has been integrated.
// ==========================================================================

if !has_model() {
    print("ERROR: load a model first");
    return;
}

set_gait_mode("mpc");
set_pose_source("groundtruth");
set_wbc_enabled(true);
set_ground_plane_enabled(true);
set_ground_plane_z(0.0);

set_actuator_mode_all("position");
set_kp_all(30.0);
set_kv_all(0.6);

if !gait_setup() {
    print("ERROR: gait_setup() failed");
    return;
}
gait_set_cycle_period(0.4);
gait_set_swing_height(0.04);
gait_set_duty(0.5);

if set_initial_pose("constrain") {
    print("Seeded: 'constrain' pose");
} else if seed_kinematic_pose() {
    print("Seeded: kinematic IK");
}
gait_use_current_pose_as_stance();

if mj_active() { mj_stop(); }
if !mj_start() {
    print("ERROR: mj_start() failed");
    return;
}

mj_async_clear();
gait_start();
gait_set_velocity(0.0, 0.0, 0.0);

print("=== diag setup ready ===");
print("Body should be at yaw=0, ready for arrow-button cmd test.");
print("Watch viewport overlay for `Body: yaw=...°  pos=(X, Y, Z)`.");
print("Press ⬆ for ~3s and report which of pos.x or pos.y changes.");
