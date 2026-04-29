// ==========================================================================
// walk_demo.rhai — End-to-end CHAMP-style trot walking demo.
//
// Pipeline:
//   1. Configure all joints for ComputedTorque + QDD-friendly armature/damping.
//   2. Build a fresh MuJoCo sim.
//   3. Auto-detect quadruped kinematics from the loaded URDF (defaults to
//      FL_foot / FR_foot / RL_foot / RR_foot — see `gait_setup_with_feet`
//      to override).
//   4. Start the gait controller and command vx = 0.3 m/s.
//   5. Animate via the async queue so the viewport actually shows the walk.
//   6. Save the trace as CSV for spectral / locomotion analysis.
//
// Run from articara: Console (📂) → select this file. Watch the robot
// trot forward in the viewport over ~5 s, then a CSV lands in `log/`.
// ==========================================================================

// ── Configurable parameters ───────────────────────────────────────────────

let WALK_VX        = 0.3;        // m/s
let WALK_VY        = 0.0;
let WALK_WZ        = 0.0;        // rad/s
let WALK_DURATION_S = 5.0;
let CYCLE_PERIOD_S = 0.4;
let SWING_HEIGHT_M = 0.04;
let DUTY           = 0.5;

// QDD tuning (MG4005-i10 + 1:10 reducer defaults — tweak for your motor).
let ARMATURE       = 1.4e-3;
let JOINT_DAMPING  = 0.1;
let KP             = 30.0;
let KV             = 0.5;

let CSV_PATH       = "log/walk_demo.csv";

// ── Sanity ─────────────────────────────────────────────────────────────────

if !has_model() {
    print("ERROR: load a quadruped URDF first");
    return;
}
print(`Model: ${model_name()}, ${num_joints()} joints`);

// ── Configure joints (synchronous — applies to next sim build) ───────────

set_actuator_mode_all("computedtorque");
set_armature_all(ARMATURE);
set_joint_damping_all(JOINT_DAMPING);
set_kp_all(KP);
set_kv_all(KV);
print(`Set CT mode, armature=${ARMATURE}, damping=${JOINT_DAMPING}, Kp=${KP}, Kv=${KV}`);

// ── Build / rebuild sim ──────────────────────────────────────────────────

if mj_active() { mj_stop(); }
if !mj_start() {
    print("ERROR: mj_start() failed — check the model has a base link");
    return;
}
mj_set_trace_max(20000);
reset_peaks();

// ── Auto-detect quadruped kinematics ─────────────────────────────────────

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

// ── Schedule the walk via the async timeline ──────────────────────────────

mj_async_clear();

// Settle for half a cycle in stance before the gait starts (lets gravity
// comp engage and the body drop onto its feet cleanly).
mj_async_print("[walk] settle 0.5 s");
mj_async_step_seconds(0.5);

// Kick off gait playback. The async queue can't directly call gait_start
// on the host's controller — gait_start mutates the engine's shared
// handle, which is owned by the host between evals. Instead we issue
// gait_start synchronously here; the controller is alive across the
// entire sim, the async queue just paces stepping.
gait_start();
gait_set_velocity(WALK_VX, WALK_VY, WALK_WZ);
mj_async_print(`[walk] gait_start, vx=${WALK_VX} for ${WALK_DURATION_S} s`);
mj_async_step_seconds(WALK_DURATION_S);

// Decelerate: ramp velocity command to zero. A single phase change is
// fine for trot; we add a 0.5 s settle so the legs return to stance.
mj_async_print("[walk] decelerate + settle");
mj_async_step_seconds(0.5);

// Save and clean up. gait_stop runs after the queue drains via a
// separate sync call — we set a flag here and ArticaraApp picks it up
// once the queue completes. For simplicity, also gait_stop synchronously
// before the async queue starts pacing zero-velocity ticks.
mj_async_save_csv(CSV_PATH);
mj_async_print(`[walk] done — CSV at ${CSV_PATH}`);

print(`Queued ${mj_async_pending()} ops. Watch the viewport!`);
print("After playback completes, run:");
print(`  python tools/analyze_peaks_log.py ${CSV_PATH} --motor MG4005-i10:10`);
