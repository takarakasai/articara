// ==========================================================================
// example_jump_async.rhai — Animated jump scheduled via the async timeline.
//
// Same 4-phase jump as `example_jump.rhai`, but every step / target change
// is queued through `mj_async_*` so the host UI loop can play it out at
// real time. The script itself runs in a few milliseconds and returns; the
// viewport then animates the queued timeline.
//
// Use this when you want to *watch* the simulation. Use the synchronous
// `example_jump.rhai` when you want a fast batch run (it's faster but you
// only see the final pose).
//
// Run from articara: Console (📂) → select this file. The `Speed` slider
// in Dynamics Analysis applies during async playback.
// ==========================================================================

// ── Configurable: same surface as example_jump.rhai ───────────────────────

let THIGH_PATTERN = "thigh";
let CALF_PATTERN  = "calf";

let POSE_STAND   = #{ thigh:  0.4, calf: -0.8 };
let POSE_CROUCH  = #{ thigh:  1.0, calf: -1.8 };
let POSE_EXTEND  = #{ thigh: -0.1, calf: -0.3 };

let DUR_STAND_S    = 0.5;
let DUR_CROUCH_S   = 0.3;
let DUR_EXTEND_S   = 0.1;
let DUR_FLIGHT_S   = 1.0;

let ARMATURE       = 1.4e-3;
let JOINT_DAMPING  = 0.1;
let KP             = 30.0;
let KV             = 0.5;

let CSV_PATH       = "log/example_jump_async.csv";

// ── Sanity ─────────────────────────────────────────────────────────────────

if !has_model() {
    print("ERROR: load a robot first");
    return;
}

// ── Identify leg joints ────────────────────────────────────────────────────

let thigh = [];
let calf  = [];
for jn in joint_names() {
    if jn.contains(THIGH_PATTERN) { thigh.push(jn); }
    if jn.contains(CALF_PATTERN)  { calf.push(jn);  }
}
if thigh.len() == 0 || calf.len() == 0 {
    print(`No joints matched "${THIGH_PATTERN}"/"${CALF_PATTERN}" — edit patterns at top.`);
    return;
}
print(`Found ${thigh.len()} thigh and ${calf.len()} calf joints`);

// ── Configure synchronously: Kp / Kv / mode / armature affect the next sim
//    creation, so set them before mj_start() ─────────────────────────────────

set_actuator_mode_all("computedtorque");
set_armature_all(ARMATURE);
set_joint_damping_all(JOINT_DAMPING);
set_kp_all(KP);
set_kv_all(KV);

// ── (Re)start sim synchronously ───────────────────────────────────────────

if mj_active() { mj_stop(); }
if !mj_start() {
    print("ERROR: mj_start() failed");
    return;
}
mj_set_trace_max(10000);
reset_peaks();

// ── Build the async timeline ──────────────────────────────────────────────
// Helper: queue a "set_position_target" for every joint in `list` at `value`.
// Each call enqueues an op; the UI loop pops them in order.
fn queue_pose(list, value) {
    for jn in list {
        mj_async_set_position_target(jn, value);
    }
}

mj_async_clear();

mj_async_print("[async] phase 1: STAND");
queue_pose(thigh, POSE_STAND.thigh);
queue_pose(calf,  POSE_STAND.calf);
mj_async_step_seconds(DUR_STAND_S);

mj_async_print("[async] phase 2: CROUCH");
queue_pose(thigh, POSE_CROUCH.thigh);
queue_pose(calf,  POSE_CROUCH.calf);
mj_async_step_seconds(DUR_CROUCH_S);

mj_async_print("[async] phase 3: EXTEND — JUMP!");
queue_pose(thigh, POSE_EXTEND.thigh);
queue_pose(calf,  POSE_EXTEND.calf);
mj_async_step_seconds(DUR_EXTEND_S);

mj_async_print("[async] phase 4: FLIGHT + LANDING");
mj_async_step_seconds(DUR_FLIGHT_S);

mj_async_save_csv(CSV_PATH);
mj_async_print(`[async] done — see ${CSV_PATH}`);

print(`Queued ${mj_async_pending()} ops. Watch the viewport — `
      + `Speed slider in Dynamics Analysis applies during playback.`);
