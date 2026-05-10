// ============================================================================
// champ_forward_bench.rhai — GUI replication of test's CHAMP forward benchmark.
//
// Drives the GUI through the exact same sequence the user reported:
//   1. Apply `constrain` pose
//   2. Show ground plane
//   3. mj_start (= PlayMuJoCo + Play)
//   4. 2 s settle (gait not yet active)
//   5. gait_setup + set_gait_mode("champ") + gait_start
//   6. Brief 0.5 s in stance (cmd=0)
//   7. set_velocity(0.30, 0, 0) (= Linear[m/s]=0.30, press up arrow)
//   8. 7 s walk
//   9. Save CSV trace
//
// Run from GUI: Script Console → 📂 → select this file. Or from CLI:
//   cargo xtask run --features mujoco --bin articara --release \
//     -- tests/fixtures/namiashi/namiashi.misa \
//     --script scripts/champ_forward_bench.rhai
//
// Output: /tmp/gui_champ_forward_bench.csv (trace with floating-base qpos at
// q[0..7] = [px, py, pz, qx, qy, qz, qw]; analyse dx = q[0](t) − q[0](settle_end))
// ============================================================================

let SETTLE_S = 2.0;          // pure stance hold (gait built but cmd=0)
let WALK_VX = 0.30;          // m/s — GUI default Linear setting
let WALK_DURATION_S = 7.0;
let CSV_PATH = "/tmp/gui_champ_forward_bench.csv";

if !has_model() {
    print("ERROR: load namiashi.misa first (CLI: --script with namiashi.misa as positional arg)");
    return;
}
print(`Model: ${model_name()}`);

// 1. Apply constrain pose synchronously (= GUI Pose panel "constrain" pick).
//    play_pose is async, so use set_joint per-joint for an immediate apply
//    BEFORE mj_start so the initial sim state has the bent pose.
set_joint("FL_hip_joint",    0.0);
set_joint("FL_thigh_joint",  1.0);
set_joint("FL_calf_joint",  -2.0);
set_joint("FR_hip_joint",    0.0);
set_joint("FR_thigh_joint",  1.0);
set_joint("FR_calf_joint",  -2.0);
set_joint("RL_hip_joint",    0.0);
set_joint("RL_thigh_joint",  1.0);
set_joint("RL_calf_joint",  -2.0);
set_joint("RR_hip_joint",    0.0);
set_joint("RR_thigh_joint",  1.0);
set_joint("RR_calf_joint",  -2.0);
set_joint("arm_pitch_joint", 0.0);
print("Applied constrain pose to joint_positions");

// 2. Show ground plane (Step 3 in user's recipe).
set_ground_plane_enabled(true);

// 3. mj_start (= PlayMuJoCo + Play).
if mj_active() { mj_stop(); }
if !mj_start() {
    print("ERROR: mj_start failed");
    return;
}
mj_set_trace_max(40000);
print(`mj_start OK, timestep = ${mj_timestep()} s`);

// 4. Settle (no gait active yet).
mj_async_clear();
mj_async_print(`[bench] settle ${SETTLE_S} s (no gait, PD holding constrain pose)`);
mj_async_step_seconds(SETTLE_S);

// 5. Build gait controller and switch to CHAMP. `gait_setup` runs the
//    Auto-Detect equivalent — captures current foot transform (=
//    constrain stance) into `nominal_foot_body`.
if !gait_setup() {
    print("ERROR: gait_setup failed");
    mj_stop();
    return;
}
set_gait_mode("champ");

// 6. Start gait + brief stance hold at cmd=0.
gait_start();
mj_async_print("[bench] gait started, cmd=0, 0.5 s stance hold");
mj_async_step_seconds(0.5);

// 7. Set cmd at the deferred timeline point so the 7 s walk window
//    starts exactly when cmd applies (= user pressing up arrow).
mj_async_set_velocity(WALK_VX, 0.0, 0.0);
mj_async_print(`[bench] cmd vx=${WALK_VX} applied, walking ${WALK_DURATION_S} s`);
mj_async_step_seconds(WALK_DURATION_S);

// 8. Stop the cmd (decelerate), brief settle, save.
mj_async_set_velocity(0.0, 0.0, 0.0);
mj_async_step_seconds(0.5);
mj_async_save_csv(CSV_PATH);
mj_async_print(`[bench] DONE — CSV saved: ${CSV_PATH}`);
mj_async_print(`[bench] cmd starts at sim_time = ${SETTLE_S + 0.5} s; record dx_world = qpos_x at that time vs end`);

print(`Queued ${mj_async_pending()} ops, watch the viewport`);
print(`Expected: forward walking, dx ≈ +1.13 m over ${WALK_DURATION_S} s`);
print(`After completion, share ${CSV_PATH} (or relevant columns)`);
