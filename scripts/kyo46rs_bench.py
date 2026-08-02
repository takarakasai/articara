#!/usr/bin/env python3
"""Locomotion benchmark for the kyo46rs biped WBC.

Sweeps the command space -- forward, backward, sideways, turning, and squatting
while walking -- and reports what survived, how far it actually got, and what
was saturating when it did not.

The point is the ACHIEVED column, not the survived column. A gait that never
falls while covering 55% of its commanded distance is a thing that happens on
this robot, and it looks exactly like walking until you check.

    python3 scripts/kyo46rs_bench.py                  # the whole matrix
    python3 scripts/kyo46rs_bench.py --group turn     # one group
    python3 scripts/kyo46rs_bench.py --csv out.csv    # machine-readable
    python3 scripts/kyo46rs_bench.py --record fwd_0.09  # keep that run's CSV

Every case runs the same number of steps so the rows are comparable; duration
follows from the step time.
"""
import argparse
import csv as csvmod
import os
import re
import subprocess
import sys

BIN = "./target/release/examples/kyo46rs_walk"
MUJOCO_LIB = "/home/takara/.mujoco/mujoco-3.8.0/lib"

# Fixed for every case, so the only thing varying is the command.
COMMON = {
    "T_SS": "0.35",
    "T_DS": "0.20",
    "STRIDE_RAMP": "6",
    "ARM_PITCH": "0",
    "LIFT_H": "0.02",
    "PHASE_BY_CONTACT": "0",
}
T_STEP = 0.55
N_STEPS = 40
# Two step-times of settle after the plan ends: enough to see whether it stands.
T_END = 2.0 + N_STEPS * T_STEP + 2.0


def cases(group=None):
    out = []
    for v in (0.02, 0.036, 0.055, 0.073, 0.091, 0.109, 0.145):
        out.append(("forward", f"vx={v:+.3f}", {"VX": f"{v}"}))
    for v in (0.036, 0.073, 0.091, 0.109):
        out.append(("backward", f"vx={-v:+.3f}", {"VX": f"{-v}"}))
    for v in (0.018, 0.036, 0.055, 0.073):
        out.append(("lateral", f"vy={v:+.3f}", {"VY": f"{v}"}))
        out.append(("lateral", f"vy={-v:+.3f}", {"VY": f"{-v}"}))
    for w in (0.05, 0.10, 0.20, 0.40):
        out.append(("turn", f"wz={w:+.2f}", {"WZ": f"{w}"}))
        out.append(("turn", f"wz={-w:+.2f}", {"WZ": f"{-w}"}))
    for a in (0.01, 0.02, 0.04):
        out.append(("squat", f"amp={a*1e3:.0f}mm still", {"SQUAT_AMP": f"{a}", "N_STEPS": "0"}))
        out.append(("squat", f"amp={a*1e3:.0f}mm walk", {"SQUAT_AMP": f"{a}", "VX": "0.055"}))
    out.append(("combined", "vx+wz", {"VX": "0.055", "WZ": "0.10"}))
    out.append(("combined", "vx+vy", {"VX": "0.055", "VY": "0.036"}))
    out.append(("combined", "vx+squat", {"VX": "0.055", "SQUAT_AMP": "0.02"}))
    return [c for c in out if group is None or c[0] == group]


PATTERNS = {
    "steps": r"steps taken:\s*(\d+)",
    "achieved": r"achieved over the walk \([\d.]+ s\): (.*)",
    "dcm": r"max \|xi - xi_ref\| =\s*([\d.]+) mm",
    "cop": r"max CoP box use =\s*([\d.]+)",
    "ankle": r"max ankle_roll use =\s*(\d+)%",
    "degraded": r"degraded solves:\s*(\d+)",
    "selfcol": r"self-collision ticks:\s*(\d+)",
    "openloop": r"open-loop ticks:\s*(\d+)",
    "fell_t": r"FELL at t=([\d.]+)",
}


def run(env_extra, traj=None):
    env = dict(os.environ)
    env["LD_LIBRARY_PATH"] = MUJOCO_LIB + ":" + env.get("LD_LIBRARY_PATH", "")
    env.update(COMMON)
    env["N_STEPS"] = str(N_STEPS)
    env["T"] = f"{T_END}"
    env.update({k: str(v) for k, v in env_extra.items()})
    if traj:
        env["TRAJ_CSV"] = traj
    p = subprocess.run([BIN], env=env, capture_output=True, text=True, timeout=1800)
    txt = p.stdout + p.stderr
    r = {}
    for k, pat in PATTERNS.items():
        m = re.search(pat, txt)
        r[k] = m.group(1) if m else ""
    r["survived"] = "SURVIVED" if "verdict: SURVIVED" in txt else "FELL"
    return r, txt


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--group")
    ap.add_argument("--csv")
    ap.add_argument("--record", help="label of one case whose trajectory CSV to keep")
    a = ap.parse_args()

    if not os.path.exists(BIN):
        sys.exit(f"{BIN} not found -- cargo build --release --features mujoco --examples")

    rows = []
    hdr = f"{'group':>9} {'command':>18} {'':>4} {'steps':>5} {'achieved / commanded':>52} " \
          f"{'dcm':>6} {'ankl':>5} {'degr':>6} {'self':>5}"
    print(hdr)
    print("-" * len(hdr))
    for group, label, envx in cases(a.group):
        traj = f"csv/bench_{a.record}.csv" if a.record and label.startswith(a.record) else None
        r, _ = run(envx, traj)
        ok = "ok" if r["survived"] == "SURVIVED" else "FELL"
        ach = (r["achieved"] or "-")[:52]
        print(f"{group:>9} {label:>18} {ok:>4} {r['steps'] or '-':>5} {ach:>52} "
              f"{r['dcm'] or '-':>6} {r['ankle'] or '-':>5} {r['degraded'] or '-':>6} "
              f"{r['selfcol'] or '-':>5}")
        rows.append({"group": group, "command": label, **r})

    if a.csv:
        with open(a.csv, "w", newline="") as f:
            w = csvmod.DictWriter(f, fieldnames=list(rows[0].keys()))
            w.writeheader()
            w.writerows(rows)
        print(f"\nwrote {a.csv}")

    n_ok = sum(1 for r in rows if r["survived"] == "SURVIVED")
    # A case only counts as passing if it also went where it was told, on
    # every axis that was commanded.
    def tracked(r):
        pcts = [int(m) for m in re.findall(r"\((-?\d+)%\)", r["achieved"] or "")]
        return bool(pcts) and all(p >= 85 for p in pcts)
    n_real = sum(1 for r in rows if r["survived"] == "SURVIVED" and tracked(r))
    print(f"\n{n_ok}/{len(rows)} survived; {n_real}/{len(rows)} also tracked "
          f"the command to >=85% on every commanded axis")


if __name__ == "__main__":
    main()
