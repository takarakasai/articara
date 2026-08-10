"""Push rejection while jabbing, binned by the jab phase the shove landed in.

The jab runs on its own clock, so a plain "jab on / jab off" comparison averages
over an unrecorded phase -- and being shoved at full extension is not the same
experiment as being shoved back in guard. JAB_START is swept across one period
to cover the phase space, and each run is binned by the extension the walk
PRINTS at the push instant, not by the offset that was asked for.
"""
import math, os, re, subprocess, sys, concurrent.futures as cf

BIN = "/home/takara/work/dp/articara/target/release/examples/kyo46rs_walk"
URDF = "/tmp/kyo46rs_vy_regression/urdf/kyo46rs_v6_rec_box.urdf"
GUARD = {"SOLE_HALF_W": "0.030", "ARM_HOLD": "1", "ARM_HOLD_JOINTS": "shoulder",
         "KP_ARM": "200", "KD_ARM": "20",
         "ARM_PITCH": f"{math.radians(-25):.4f}", "ELBOW": f"{math.radians(-130):.4f}"}
BASE = {"T_SS": "0.35", "T_DS": "0.20", "STRIDE_RAMP": "6", "LIFT_H": "0.02",
        "PHASE_BY_CONTACT": "0", "N_STEPS": "20", "T": "15.0", "VX": "0.055",
        "PUSH_DT": "0.10", "PUSH_AT": "0.5", "PUSH_RECOVER_MM": "20",
        "PUSH_RECOVER_S": "0.5", "ADAPT_STEP": "0", "K_DCM": "2.0"}
CELLS = {"right ss": {"PUSH_DEG": "270", "PUSH_SUPPORT": "ss", "PUSH_STEP": "6"},
         "left ss":  {"PUSH_DEG": "90",  "PUSH_SUPPORT": "ss", "PUSH_STEP": "6"}}
IMPULSES = [round(0.10 * k, 2) for k in range(1, 17)]     # 0.10 .. 1.60
JAB_STARTS = [round(2.5 + 0.70 * k / 8, 4) for k in range(8)]

def run(cell, imp, jab_start):
    env = dict(os.environ)
    env["LD_LIBRARY_PATH"] = "/home/takara/.mujoco/mujoco-3.8.0/lib:" + env.get("LD_LIBRARY_PATH", "")
    env.update(BASE); env.update(GUARD); env.update(CELLS[cell])
    env["URDF"] = URDF; env["PUSH_IMPULSE"] = f"{imp:.4f}"
    if jab_start is not None:
        env.update({"JAB": "1", "JAB_PERIOD": "0.70", "JAB_START": f"{jab_start}"})
    txt = subprocess.run([BIN], env=env, capture_output=True, text=True, timeout=900)
    out = txt.stdout + txt.stderr
    fired = "PUSH at t=" in out
    m = re.search(r"PUSH JAB PHASE: side=(\w) phase=([\d.]+) extension=([\d.]+)", out)
    return {"cell": cell, "imp": imp, "fired": fired,
            "surv": "verdict: SURVIVED" in out,
            "ext": float(m.group(3)) if m else None,
            "side": m.group(1) if m else None}

jobs = [(c, i, s) for c in CELLS for i in IMPULSES for s in [None] + JAB_STARTS]
rows = []
with cf.ThreadPoolExecutor(max_workers=8) as ex:
    futs = [ex.submit(run, *j) for j in jobs]
    for k, f in enumerate(cf.as_completed(futs), 1):
        rows.append(f.result())
        print(f"\r{k}/{len(jobs)}", end="", file=sys.stderr, flush=True)
print(file=sys.stderr)
assert all(r["fired"] for r in rows), "some cells never fired"

def safe(rr):
    s = 0.0
    for i in sorted({r["imp"] for r in rr}):
        if all(r["surv"] for r in rr if r["imp"] <= i + 1e-9):
            s = i
    return s

for cell in CELLS:
    cr = [r for r in rows if r["cell"] == cell]
    off = [r for r in cr if r["ext"] is None]
    on  = [r for r in cr if r["ext"] is not None]
    print(f"\n=== {cell} ===")
    print(f"  jab off            safe {safe(off):.2f} N*s   survived {sum(r['surv'] for r in off)}/{len(off)}")
    print(f"  jab on (all phases) safe {safe(on):.2f} N*s   survived {sum(r['surv'] for r in on)}/{len(on)}")
    print(f"  {'extension bin':<18}{'runs':>6}{'survived':>10}   safe")
    for lo, hi, lab in ((0.0, 0.25, "guard  0.00-0.25"), (0.25, 0.6, "rising 0.25-0.60"),
                        (0.6, 0.9, "out    0.60-0.90"), (0.9, 1.01, "extended 0.90-1.0")):
        b = [r for r in on if lo <= r["ext"] < hi]
        if b:
            print(f"  {lab:<18}{len(b):>6}{sum(r['surv'] for r in b):>10}   {safe(b):.2f}")
import json
json.dump(rows, open("/home/takara/work/dp/articara/csv/push_jab_phase.json", "w"))
print("\nwrote csv/push_jab_phase.json")
