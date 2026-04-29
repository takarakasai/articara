// ===========================================================================
// verify_jump_tuning.rhai — automated A/B comparison for jump-tuning sweeps.
//
// Workflow:
//   1. Loads tuning configurations (preset list at the top).
//   2. For each config: stop sim → set armature/damping/Kp/Kv on every joint
//      → start sim → play the named jump sequence → wait until done →
//      save the trace as `log/jump_<tag>.csv`.
//   3. Prints a summary table of |τ| / |q̇| peaks per config.
//
// After this finishes, run the analyzer outside articara to plot:
//   for f in log/jump_*.csv; do
//     python tools/analyze_peaks_log.py "$f" --motor MG4005-i10:10
//   done
//
// Edit `JUMP_SEQUENCE`, `WATCHED_JOINTS`, and `CONFIGS` for your robot.
// ===========================================================================

// ── Configuration ──────────────────────────────────────────────────────────

// Name of the saved sequence in the model that performs the jump motion.
// Must already exist in the loaded model (created via the Sequences UI panel
// or the Rhai `add_sequence` API).
let JUMP_SEQUENCE = "jump";

// Joints whose peaks we report in the summary. Empty = report all movable
// joints (more verbose but useful for unfamiliar models).
let WATCHED_JOINTS = [
    "FR_calf_joint", "FL_calf_joint", "RR_calf_joint", "RL_calf_joint",
];

// Tuning sweeps. Each row is `[tag, armature, damping, Kp, Kv]`. Apply
// uniformly to every movable joint via the `*_all` setters. For per-joint
// asymmetry (e.g. heavier hip than calf), call set_kp(name,…) etc. instead.
let CONFIGS = [
    // tag,                     armature, damping,  Kp,   Kv
    // tag,                     armature, damping,  Kp,    Kv,    grav_comp
    #{ tag: "baseline",         armature: 1.4e-3,   damping: 1.0,  kp: 50.0,  kv: 0.5, grav_comp: false },
    #{ tag: "qdd_recommended",  armature: 1.4e-3,   damping: 0.1,  kp: 80.0,  kv: 1.2, grav_comp: false },
    #{ tag: "qdd_grav_comp",    armature: 1.4e-3,   damping: 0.1,  kp: 80.0,  kv: 1.2, grav_comp: true  },
    #{ tag: "qdd_aggressive",   armature: 1.4e-3,   damping: 0.05, kp: 120.0, kv: 1.5, grav_comp: true  },
];

// Trace ring buffer cap. The trace is captured in real time by MuJoCo, so
// we make it large enough to hold the entire jump (~5 s @ 500 Hz = 2500
// samples; allow some headroom for longer sequences).
let TRACE_MAX = 8000;

// Hard timeout for waiting on the sequence (frames). Prevents infinite loops
// if `play_sequence` silently failed (e.g. wrong sequence name).
let TIMEOUT_FRAMES = 10000;   // ≈ 20 s at 2 ms timestep

// Output directory for CSVs. Must already exist (we don't create it here).
let LOG_DIR = "log";

// ── Sanity checks ──────────────────────────────────────────────────────────

if !has_model() {
    print("ERROR: no model loaded. Open a robot first, then re-run this script.");
    return;
}
print(`Model: ${model_name()}, ${num_joints()} joints`);

let joints = joint_names();
let watched = if WATCHED_JOINTS.is_empty() { joints } else { WATCHED_JOINTS };

// ── Sweep loop ─────────────────────────────────────────────────────────────

let summary = [];

for cfg in CONFIGS {
    print(`──── ${cfg.tag} ────`);
    print(`  armature=${cfg.armature}, damping=${cfg.damping}, Kp=${cfg.kp}, Kv=${cfg.kv}, grav_comp=${cfg.grav_comp}`);

    // Make sure no sim is alive before we re-bake the MJCF — armature and
    // damping only flow into MuJoCo at sim construction time.
    if mj_active() {
        mj_stop();
    }

    let n1 = set_armature_all(cfg.armature);
    let n2 = set_joint_damping_all(cfg.damping);
    let n3 = set_kp_all(cfg.kp);
    let n4 = set_kv_all(cfg.kv);
    print(`  applied to ${n1} joints (armature/damping/Kp/Kv all reached)`);

    if !mj_start() {
        print("  ERROR: mj_start() failed; skipping this config");
        continue;
    }
    // Gravity comp must be set AFTER mj_start since it lives on the sim,
    // not the model — a fresh sim defaults to off.
    mj_gravity_compensation(cfg.grav_comp);
    mj_set_trace_max(TRACE_MAX);
    reset_peaks();

    // Settle for one physics tick so the controller observes the initial
    // pose under gravity before we kick off the jump.
    mj_step(1);

    if !play_sequence(JUMP_SEQUENCE) {
        print(`  ERROR: play_sequence("${JUMP_SEQUENCE}") failed; skipping`);
        mj_stop();
        continue;
    }

    // Run the sim forward in 100-frame chunks until the sequence completes
    // or we time out.
    let total = 0;
    while sequence_in_progress() && total < TIMEOUT_FRAMES {
        mj_step(100);
        total += 100;
    }
    // Tail: some sims keep settling for a few hundred ms after the sequence
    // ends (landing impact, body settling on the ground) — sample those too.
    mj_step(500);
    total += 500;

    let n_samples = mj_trace_len();
    let csv_path = `${LOG_DIR}/jump_${cfg.tag}.csv`;
    let n_written = save_peaks_csv(csv_path);
    if n_written < 0 {
        print(`  ERROR: failed to write ${csv_path}`);
    } else {
        print(`  wrote ${n_written} rows → ${csv_path}  (sim stepped ${total} frames)`);
    }

    // Snapshot peaks before stopping (mj_stop() clears the running sim).
    let row = #{ tag: cfg.tag, samples: n_samples, peaks: #{} };
    for jn in watched {
        row.peaks[jn] = #{
            tau: peak_torque(jn),
            qvel: peak_velocity(jn),
        };
    }
    summary.push(row);

    mj_stop();
}

// ── Print summary ──────────────────────────────────────────────────────────
// Plain `tag → joint → |τ|max, |q̇|max` listing; the `analyze_peaks_log.py`
// run prints the cleaner table with FFT info, so don't duplicate that work
// here — just give a quick sanity check that the trace was captured.

print("");
print("═══════════════ Peak summary (per config × watched joint) ═══════════════");
for row in summary {
    print(`[${row.tag}]  samples=${row.samples}`);
    for jn in watched {
        let p = row.peaks[jn];
        print(`    ${jn}:  |τ|max = ${p.tau}  |q̇|max = ${p.qvel}`);
    }
}

print("");
print("Run spectral analysis on each CSV:");
for cfg in CONFIGS {
    print(`  python tools/analyze_peaks_log.py log/jump_${cfg.tag}.csv --motor MG4005-i10:10`);
}
