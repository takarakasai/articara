// ==========================================================================
// example_jump.rhai — Self-contained jump simulation demo using ComputedTorque
//
// Run from articara: Console (📂) → select this file.
//
// What this script does:
//   1. Configures all leg joints for QDD computed-torque control
//      (armature/damping/Kp/Kv tuned for MG4005-i10 + 1:10 reducer).
//   2. Identifies thigh + calf joints by name and drives them through a
//      4-phase jump: settle → crouch → extend → land. No pre-saved poses
//      needed; everything is procedural.
//   3. Saves the trace as `log/example_jump.csv` for spectral analysis.
//
// Edit the JOINT NAMES and POSE ANGLES at the top to match your robot.
// The defaults assume a generic quadruped with `*_thigh_*` and `*_calf_*`
// naming.
// ==========================================================================

// ── Configurable: joint name substrings and target angles (rad) ───────────

let THIGH_PATTERN = "thigh";
let CALF_PATTERN  = "calf";

// One target per phase, applied to every matched joint. Sign convention
// depends on your URDF axis direction — flip the signs below if your robot
// curls the wrong way.
let POSE_STAND   = #{ thigh:  0.4, calf: -0.8 };  // legs slightly bent
let POSE_CROUCH  = #{ thigh:  1.0, calf: -1.8 };  // wind up: knees deep
let POSE_EXTEND  = #{ thigh: -0.1, calf: -0.3 };  // legs straight: launch

// Phase durations in milliseconds.
let DUR_STAND_MS   = 500;
let DUR_CROUCH_MS  = 300;
let DUR_EXTEND_MS  = 100;
let DUR_FLIGHT_MS  = 1000;

// QDD tuning for MG4005-i10 + 1:10 (override with your motor's values).
let ARMATURE       = 1.4e-3;
let JOINT_DAMPING  = 0.1;
let KP             = 30.0;
let KV             = 0.5;

let CSV_PATH       = "log/example_jump.csv";

// ── Sanity ─────────────────────────────────────────────────────────────────

if !has_model() {
    print("ERROR: load a robot first (File → Open or `load(path)`)");
    return;
}
print(`Model: ${model_name()}, ${num_joints()} joints`);

// ── Identify leg joints by name pattern ────────────────────────────────────

let thigh = [];
let calf  = [];
for jn in joint_names() {
    if jn.contains(THIGH_PATTERN) { thigh.push(jn); }
    if jn.contains(CALF_PATTERN)  { calf.push(jn);  }
}
if thigh.len() == 0 || calf.len() == 0 {
    print(`No joints matched "${THIGH_PATTERN}"/"${CALF_PATTERN}" — edit the patterns at the top of this script.`);
    return;
}
print(`Found ${thigh.len()} thigh and ${calf.len()} calf joints`);

// ── Configure all joints for ComputedTorque ────────────────────────────────

set_actuator_mode_all("computedtorque");
set_armature_all(ARMATURE);
set_joint_damping_all(JOINT_DAMPING);
set_kp_all(KP);
set_kv_all(KV);
print(`Configured CT mode: armature=${ARMATURE}, damping=${JOINT_DAMPING}, Kp=${KP}, Kv=${KV}`);

// ── Start a fresh sim with these params baked in ──────────────────────────

if mj_active() { mj_stop(); }
if !mj_start() {
    print("ERROR: mj_start() failed (check that the model has a valid base link).");
    return;
}
mj_set_trace_max(10000);    // ~20 s of history at the default 2 ms timestep
reset_peaks();
let dt = mj_timestep();
print(`Sim started, dt = ${(dt*1000)} ms (Nyquist ${1.0/(2.0*dt)} Hz)`);

// ── Phase helpers ──────────────────────────────────────────────────────────

// Apply (thigh_target, calf_target) to every matched joint.
fn apply_pose(thigh_joints, calf_joints, pose) {
    for jn in thigh_joints { set_position_target(jn, pose.thigh); }
    for jn in calf_joints  { set_position_target(jn, pose.calf);  }
}

// Step the sim by `ms` milliseconds, given dt in seconds. Rhai promotes
// int → float in mixed expressions, so `ms / 1000.0` works directly.
fn step_ms(ms, dt) {
    let n = (ms / 1000.0 / dt).to_int();
    if n < 1 { n = 1; }
    mj_step(n);
}

// ── 4-phase jump ───────────────────────────────────────────────────────────

print("");
print("[phase 1] STAND — settle in nominal stance");
apply_pose(thigh, calf, POSE_STAND);
step_ms(DUR_STAND_MS, dt);

print("[phase 2] CROUCH — wind up");
apply_pose(thigh, calf, POSE_CROUCH);
step_ms(DUR_CROUCH_MS, dt);

print("[phase 3] EXTEND — JUMP!");
apply_pose(thigh, calf, POSE_EXTEND);
step_ms(DUR_EXTEND_MS, dt);

print("[phase 4] FLIGHT — release controller, observe peaks");
// Keep targets at extended pose so legs stay out during flight; controller
// will hold them with the PD residual (no q̇*/q̈* feedforward since no
// transition is active, which is desired during flight).
step_ms(DUR_FLIGHT_MS, dt);

// ── Report ─────────────────────────────────────────────────────────────────

print("");
print("─── Peak observations ───");
let probe = calf[0];
print(`  ${probe}:  |τ|max = ${peak_torque(probe)} N·m`);
print(`  ${probe}:  |q̇|max = ${peak_velocity(probe)} rad/s`);
let probe2 = thigh[0];
print(`  ${probe2}:  |τ|max = ${peak_torque(probe2)} N·m`);
print(`  ${probe2}:  |q̇|max = ${peak_velocity(probe2)} rad/s`);

let n = save_peaks_csv(CSV_PATH);
if n < 0 {
    print(`ERROR: failed to write ${CSV_PATH}`);
} else {
    print("");
    print(`Wrote ${n} rows → ${CSV_PATH}`);
    print("");
    print("Analyse with:");
    print(`  python tools/analyze_peaks_log.py ${CSV_PATH} --motor MG4005-i10:10`);
}

mj_stop();
