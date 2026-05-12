// ============================================================================
// wbc_improvement_demo.rhai — verify the D3.3.7 WBC tracking improvement
// in the GUI.
//
// Before this fix, MPC+WBC forward walk under namiashi achieved only ~16% of
// commanded distance (dx ≈ +0.12 m vs cmd·T = +0.75 m). Root cause: the
// `compute_mpc_footstep` capture-point feedback `+k·(v_obs - v_cmd)` was a
// positive loop under stiff PD (= the GUI's actuator config loaded from
// `namiashi.misa`). Disabling `k_capture` lifts tracking to ~87%.
//
// This script drives the GUI through the same forward-walk benchmark as
// `champ_forward_bench.rhai` but with `GaitMode::Mpc` + WBC enabled, twice:
//   pass A: capture_point_gain = 0.175 (default, "bug visible")
//   pass B: capture_point_gain = 0.000 (fix applied)
// Each pass saves a CSV with the floating-base qpos so the user can plot
// dx(t) side-by-side and see the difference visually in the viewport.
//
// Run from GUI: Script Console → 📂 → wbc_improvement_demo.rhai. Or:
//   cargo run --features mujoco --bin articara -- \
//     tests/fixtures/namiashi/namiashi.misa \
//     --script scripts/wbc_improvement_demo.rhai
//
// Outputs:
//   /tmp/wbc_demo_bug.csv  (pass A — default capture-point, weak tracking)
//   /tmp/wbc_demo_fix.csv  (pass B — capture-point disabled, real tracking)
// ============================================================================

let SETTLE_S = 2.0;
let WALK_VX = 0.30;
let WALK_DURATION_S = 5.0;

if !has_model() {
    print("ERROR: load namiashi.misa first");
    return;
}
print(`Model: ${model_name()}`);

fn apply_constrain_pose() {
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
}

fn run_pass(label, k_capture, csv_path) {
    apply_constrain_pose();
    set_ground_plane_enabled(true);
    if mj_active() { mj_stop(); }
    if !mj_start() { print(`ERROR: mj_start failed (${label})`); return; }
    mj_set_trace_max(40000);
    mj_async_clear();
    mj_async_print(`[${label}] settle ${SETTLE_S} s (PD only)`);
    mj_async_step_seconds(SETTLE_S);
    if !gait_setup() { print(`ERROR: gait_setup failed (${label})`); mj_stop(); return; }
    set_gait_mode("mpc");
    set_pose_source("groundtruth");
    set_wbc_enabled(true);
    set_capture_point_gain(k_capture);
    gait_start();
    mj_async_print(`[${label}] gait+wbc started, k_capture=${k_capture}, 0.5 s cmd=0 hold`);
    mj_async_step_seconds(0.5);
    mj_async_set_velocity(WALK_VX, 0.0, 0.0);
    mj_async_print(`[${label}] cmd vx=${WALK_VX}, walking ${WALK_DURATION_S} s`);
    mj_async_step_seconds(WALK_DURATION_S);
    mj_async_set_velocity(0.0, 0.0, 0.0);
    mj_async_step_seconds(0.5);
    mj_async_save_csv(csv_path);
    mj_async_print(`[${label}] DONE — CSV: ${csv_path}`);
}

run_pass("pass A: bug (k_capture=0.175)", 0.175, "/tmp/wbc_demo_bug.csv");
run_pass("pass B: fix (k_capture=0.000)", 0.000, "/tmp/wbc_demo_fix.csv");

print(`Queued ${mj_async_pending()} ops total.`);
print("Watch the viewport: pass A drifts ~0.12 m forward, pass B reaches ~0.65 m.");
print("Compare CSVs: each ends with `base_px` showing the actual forward travel.");
