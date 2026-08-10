#!/usr/bin/env python3
"""What torque does ankle_roll actually need? -- the measurement that decides
whether the most distal leg actuator can be a lighter one.

Motivation. ankle_roll sits 0.265 m from the hip_pitch axis and carries an
Edulite05 (0.242 kg), which makes it 56% of the swing leg's inertia about that
axis -- by far the largest single contributor. Swapping it for an 82 g class
actuator would take 0.320 kg off the machine AND 28% off the swing inertia,
about three to six times what re-making the leg structure in bent aluminium can
buy. The only thing in the way is torque: ankle_roll is the joint that keeps
the sole flat while the CoM crosses over one foot, and nothing had measured
what that costs.

`max ankle_roll use` in kyo46rs_bench.py is NOT this number -- it is the
joint's ANGLE against its position limit (kyo46rs_walk.rs, "ankle_roll travel,
against the joint's own limit"). Torque never had a metric, so this script
reads it out of the trajectory CSV's `tau_<joint>` columns instead, by column
NAME (the header is parsed, never assumed positionally -- Sec.8's unit-mixing
lesson applies to column order too).

Harness validation. hip_roll is measured in the same pass, because Sec.10.11
already put a number on it (4.56 N*m in single support). If this script cannot
reproduce that, its ankle_roll number means nothing either -- so the hip_roll
column is a CONTROL, not a bonus.

    python3 scripts/kyo46rs_ankle_roll_torque.py                  # v6 base
    python3 scripts/kyo46rs_ankle_roll_torque.py --urdf <path>     # a variant
    python3 scripts/kyo46rs_ankle_roll_torque.py --group lateral   # one group
"""
import argparse
import concurrent.futures as cf
import csv
import os
import re
import subprocess
import sys

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BIN = f"{REPO}/target/release/examples/kyo46rs_walk"
MUJOCO_LIB = "/home/takara/.mujoco/mujoco-3.8.0/lib"
BASE_URDF = "/home/takara/work/dp/humanoid/kyo46rs_description/urdf/kyo46rs.urdf"

# Pinned to kyo46rs_bench.py's COMMON so the numbers sit on the same ruler as
# every other measurement in doc/kyo46rs_biped_wbc.md.
COMMON = {
    "T_SS": "0.35", "T_DS": "0.20", "STRIDE_RAMP": "6",
    "ARM_PITCH": "0", "LIFT_H": "0.02", "PHASE_BY_CONTACT": "0",
}
N_STEPS = 40

# The joints whose torque is read out. ankle_roll is the subject; hip_roll is
# the control (Sec.10.11 = 4.56 N*m); knee and ankle_pitch are there because a
# lighter foot unloads them too, and a claim about one distal joint is easier
# to believe next to its neighbours.
WANT = ["ankle_roll", "ankle_pitch", "knee", "hip_roll", "hip_pitch"]

# Effort limits as modelled, for the "% of what" column. Continuous ratings are
# NOT in the URDF (its `effort` is the PEAK figure, per README) so they are
# named here explicitly.
RATING = {           # (continuous, peak) N*m
    "ankle_roll": (1.8, 6.0),      # Edulite05
    "ankle_pitch": (1.8, 6.0),     # Edulite05
    "knee": (5.0, 14.0),           # RS00
    "hip_roll": (5.0, 14.0),       # RS00
    "hip_pitch": (1.8, 6.0),       # Edulite05
}


def cases(group=None):
    """(group, label, env). Chosen for where ankle_roll is loaded, not for
    coverage of the command space: single support with the CoM over one sole,
    lateral commands, and lateral pushes."""
    out = [("still", "stand still", {"N_STEPS": "0"})]
    for v in (0.055, 0.109, 0.145):
        out.append(("forward", f"vx={v:+.3f}", {"VX": f"{v}"}))
    # Lateral is the ankle_roll-critical direction. The safe band on v6 is
    # |VY| <= 0.018 (Sec.28), so 0.036 is deliberately outside it -- a command
    # that falls still shows what ankle_roll was asked for before it did.
    for v in (0.018, 0.036):
        out.append(("lateral", f"vy={v:+.3f}", {"VY": f"{v}"}))
        out.append(("lateral", f"vy={-v:+.3f}", {"VY": f"{-v}"}))
    for w in (0.40, -0.40):
        out.append(("turn", f"wz={w:+.2f}", {"WZ": f"{w}"}))
    out.append(("squat", "amp=40mm still", {"SQUAT_AMP": "0.04", "N_STEPS": "0"}))
    out.append(("squat", "amp=40mm walk", {"SQUAT_AMP": "0.04", "VX": "0.055"}))
    out.append(("combined", "vx+vy", {"VX": "0.055", "VY": "0.036"}))
    # Pushes at each cell's baseline safe impulse (kyo46rs_bench.py
    # GRID_CENTRE, 2026-08-07). Lateral pushes in single support are where
    # ankle_roll has the least help from the other foot.
    for name, deg, phase, imp in (
        ("left", 90.0, "ss", 0.60), ("right", 270.0, "ss", 0.60),
        ("left", 90.0, "ds", 0.65), ("right", 270.0, "ds", 0.95),
    ):
        env = {"PUSH_DEG": f"{deg}", "PUSH_SUPPORT": phase, "PUSH_STEP": "6",
               "PUSH_IMPULSE": f"{imp}"}
        if phase == "ds":
            env["PUSH_AT"] = "0.25"
        out.append(("push", f"push {name} {phase} {imp} N*s", env))
    return [c for c in out if group is None or c[0] == group]


def run_one(urdf, label, env_extra, csv_path):
    env = dict(os.environ)
    env["LD_LIBRARY_PATH"] = MUJOCO_LIB + ":" + env.get("LD_LIBRARY_PATH", "")
    env.update(COMMON)
    env["N_STEPS"] = str(N_STEPS)
    env["URDF"] = urdf
    env.update({k: str(v) for k, v in env_extra.items()})
    t_step = float(env["T_SS"]) + float(env["T_DS"])
    env["T"] = f"{2.0 + int(env['N_STEPS']) * t_step + 2.0}"
    env["TRAJ_CSV"] = csv_path
    p = subprocess.run([BIN], cwd=REPO, env=env, capture_output=True,
                       text=True, timeout=1800)
    txt = p.stdout + p.stderr
    fell = "verdict: SURVIVED" not in txt
    m = re.search(r"steps taken:\s*(\d+)", txt)
    steps = int(m.group(1)) if m else 0
    # Time of the topple, so the saturated tail can be clipped off. A 0.30 s
    # guard band ahead of it: the torque box is already saturating before the
    # trunk angle crosses whatever threshold declares the fall.
    m = re.search(r"FELL at t=([\d.]+)", txt)
    t_fell = float(m.group(1)) if m else None
    t_cut = (t_fell - 0.30) if t_fell is not None else None
    return fell, steps, t_cut, txt


def peaks(csv_path, t_cut=None):
    """Per-joint-family torque statistics, read by column NAME. Returns
    {family: {"peak","p99","rms","lim"}} over both sides.

    `t_cut` drops every sample at or after that time. Two reasons it matters:

      * A run that FELL saturates the torque box on the way down, so its peak
        is the LIMIT (6.000, 14.002 - the effort value, to three decimals) and
        not a demand. Reading that as "the joint needs 6 N*m" is reading the
        constraint back out of the solver.
      * A run that completes its 40 steps and then topples during the settle
        window is a legitimate measurement of walking right up to the point it
        stopped walking. Clipping keeps that, where discarding the whole run
        would throw it away.

    peak alone cannot size an actuator: one degraded tick moves it. p99 and rms
    are reported next to it so a one-tick spike is distinguishable from
    sustained demand - the same reason Sec.21.4 stopped judging on one window.
    """
    with open(csv_path, newline="") as f:
        rd = csv.reader(f)
        head = next(rd)
        idx = {n: i for i, n in enumerate(head)}
        if "t" not in idx:
            raise SystemExit(f"{csv_path}: no `t` column; header is not what "
                             f"this parser was written against")
        ti_t = idx["t"]
        cols = {}
        for fam in WANT:
            for side in ("left", "right"):
                tn = f"tau_{side}_{fam}_joint"
                ln = f"lim_{side}_{fam}_joint"
                if tn in idx:
                    cols.setdefault(fam, []).append((idx[tn], idx.get(ln)))
        missing = [f for f in WANT if f not in cols]
        if missing:
            raise SystemExit(
                f"{csv_path}: no tau_ column for {missing}. The walk example's "
                f"profile log_joints does not carry them -- add them there "
                f"rather than guessing a column index.")
        vals = {f: [] for f in cols}
        lim = {f: float("nan") for f in cols}
        for row in rd:
            if not row:
                continue
            if t_cut is not None and float(row[ti_t]) >= t_cut:
                continue
            for fam, pairs in cols.items():
                for tix, li in pairs:
                    vals[fam].append(abs(float(row[tix])))
                    if li is not None:
                        lim[fam] = float(row[li])
    out = {}
    for fam, v in vals.items():
        if not v:
            out[fam] = {"peak": 0.0, "p99": 0.0, "rms": 0.0, "lim": lim[fam],
                        "n": 0}
            continue
        v.sort()
        n = len(v)
        rms = (sum(x * x for x in v) / n) ** 0.5
        out[fam] = {"peak": v[-1], "p99": v[min(n - 1, int(0.99 * n))],
                    "rms": rms, "lim": lim[fam], "n": n}
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--urdf", default=BASE_URDF)
    ap.add_argument("--group", default=None)
    ap.add_argument("--tag", default="base")
    ap.add_argument("--jobs", type=int, default=6)
    ap.add_argument("--work", default="/tmp/kyo46rs_ankle_roll")
    a = ap.parse_args()
    os.makedirs(a.work, exist_ok=True)
    cs = cases(a.group)
    print(f"URDF {a.urdf}")
    print(f"{len(cs)} cases, {a.jobs} at a time\n")

    def job(i_c):
        i, (grp, label, env) = i_c
        cp = f"{a.work}/{a.tag}_{i:02d}.csv"
        fell, steps, t_cut, txt = run_one(a.urdf, label, env, cp)
        return grp, label, fell, steps, t_cut, peaks(cp, t_cut)

    rows = []
    with cf.ThreadPoolExecutor(max_workers=a.jobs) as ex:
        for r in ex.map(job, list(enumerate(cs))):
            rows.append(r)
            grp, label, fell, steps, t_cut, pk = r
            ar, hr = pk["ankle_roll"], pk["hip_roll"]
            cut = f" cut@{t_cut:.1f}s" if t_cut is not None else ""
            print(f"  {label:24s} {'FELL' if fell else 'ok  '} steps={steps:3d} "
                  f"ankle_roll peak={ar['peak']:6.3f} p99={ar['p99']:6.3f} "
                  f"rms={ar['rms']:6.3f} | hip_roll peak={hr['peak']:6.3f}{cut}")

    def table(sel, title):
        if not sel:
            return
        print(f"\n--- {title} ({len(sel)} cases) ---")
        print(f"{'joint':13s} {'peak':>7s} {'p99':>7s} {'rms':>7s} "
              f"{'peak% cont':>11s} {'peak% peak':>11s}  worst case")
        for fam in WANT:
            best = max(sel, key=lambda r: r[5][fam]["peak"])
            s = best[5][fam]
            cont, pk_r = RATING[fam]
            p99 = max(r[5][fam]["p99"] for r in sel)
            rms = max(r[5][fam]["rms"] for r in sel)
            print(f"{fam:13s} {s['peak']:7.3f} {p99:7.3f} {rms:7.3f} "
                  f"{s['peak']/cont*100:10.0f}% {s['peak']/pk_r*100:10.0f}%  "
                  f"{best[1]}")

    survived = [r for r in rows if not r[2]]
    fellrows = [r for r in rows if r[2]]
    print(f"\n{len(survived)}/{len(rows)} cases survived.")
    table(survived, "SURVIVORS - this is the sizing number")
    table(fellrows, "FALLERS, clipped 0.30 s before the topple")
    print("\nNOTE. An unclipped faller reports the effort LIMIT (6.000 / "
          "14.002), not a demand: the torque box saturates on the way down. "
          "Size against the survivor table.")


if __name__ == "__main__":
    main()
