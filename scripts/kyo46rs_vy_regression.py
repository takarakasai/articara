#!/usr/bin/env python3
"""Where did the kyo46rs lateral margin go? -- a bisection over the v2..v6 redesign.

The 2026-08-09 handover reports ONE run in which VY=+0.036 fell on the v6
model, where Sec.16.6 had that command passing.  One run is not a result, and
it does not name which of the four mech changes did it.  This script answers
both questions the way Sec.25/Sec.26 answer them: the same ladder, every
condition, counted in cells.

    python3 scripts/kyo46rs_vy_regression.py            # the whole thing
    python3 scripts/kyo46rs_vy_regression.py --stage rev  # just the git bisect

Conditions are URDFs, not code paths.  The four redesign commits are checked
out of `kyo46rs_description` into a scratch dir; two of them are further SPLIT
by hand because 839d69c bundles v5 (hip_pitch booster retired) with v6
(shoulder roll added) and neither can be blamed while they ride together:

    v2base  1d2e446  pre-redesign, EL05 knee with booster
    v2knee  1d32b17  knee identity fix (RS00 x2, mass + limits)
    v3      79f5afd  knee -> single RS00, booster gone
    v4      a9298c9  hip_roll -> single RS00
    v5only  (cut)    v6 with the shoulder roll removed  = v4 + v5 alone
    v6only  (cut)    v6 with v5's hip_pitch change undone = v4 + v6 alone
    v6      839d69c  both, as shipped
    v6fixed (cut)    v6 with the roll joints made `fixed` -- SAME mass, SAME
                     geometry, SAME inertia, no free axis.  The control that
                     decides between "the arm is heavy" and "the arm is loose".

Each condition runs a |VY| ladder in BOTH directions with BOTH feet taking the
first swing, and reports `safe` = the largest magnitude at or below which every
probe survived.  Survival near the boundary is speckled here exactly as it is
under the push bench (Sec.18.2), so a single cell decides nothing and the cell
count is reported next to `safe`.
"""
import argparse
import concurrent.futures as cf
import csv
import os
import re
import subprocess
import sys

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DESC = "/home/takara/work/dp/humanoid/kyo46rs_description"
BIN = f"{REPO}/target/release/examples/kyo46rs_walk"
MUJOCO_LIB = "/home/takara/.mujoco/mujoco-3.8.0/lib"
WORK = os.environ.get("VY_WORK", "/tmp/kyo46rs_vy_regression")

# Pinned to the locomotion bench's COMMON so the numbers sit on its ruler.
COMMON = {
    "T_SS": "0.35", "T_DS": "0.20", "STRIDE_RAMP": "6",
    "ARM_PITCH": "0", "LIFT_H": "0.02", "PHASE_BY_CONTACT": "0",
    "N_STEPS": "40", "T": "26.0",
}
MAGS = [0.018, 0.022, 0.026, 0.030, 0.036, 0.045, 0.055]
SIGNS = [+1, -1]
FIRST_SWING = [0, 1]

GIT_REVS = [("v2base", "1d2e446"), ("v2knee", "1d32b17"),
            ("v3", "79f5afd"), ("v4", "a9298c9"), ("v6", "839d69c")]
CUT_REVS = ["v5only", "v6only", "v6fixed", "v6_rec", "v6_rec_free"]
ORDER = ["v2base", "v2knee", "v3", "v4", "v5only", "v6only", "v6", "v6fixed"]

PATTERNS = {
    "steps": r"steps taken:\s*(\d+)",
    "achieved_pct": r"achieved after the ramp \([\d.]+ s\):.*?\((-?\d+)%\)",
    "trunk_z": r"min trunk z =\s*([\d.]+)",
    "tilt": r"max tilt =\s*([\d.]+) rad",
    "dcm": r"max \|xi - xi_ref\| =\s*([\d.]+) mm",
    "cop": r"max CoP box use =\s*([\d.]+)",
    "ankle": r"max ankle_roll use =\s*(\d+)%",
    "degraded": r"degraded solves:\s*(\d+)",
    "selfcol_ticks": r"self-collision ticks:\s*(\d+)",
    "selfcol_peak": r"self-collision ticks:\s*\d+\s*\(peak ([\d.]+) N\)",
    "fell_t": r"FELL at t=([\d.]+)",
    "mass": r"total_mass=([\d.]+) kg",
}


# ---- conditions -----------------------------------------------------------

def build_urdfs():
    """Check the four commits out and cut the three split variants."""
    os.makedirs(f"{WORK}/urdf", exist_ok=True)
    # meshes are referenced as ../meshes/, so the scratch urdf dir needs a
    # sibling that resolves -- a symlink, not a copy.
    link = f"{WORK}/meshes"
    if not os.path.islink(link):
        os.symlink(f"{DESC}/meshes", link)
    for tag, sha in GIT_REVS:
        out = subprocess.run(["git", "-C", DESC, "show", f"{sha}:urdf/kyo46rs.urdf"],
                             capture_output=True, text=True, check=True).stdout
        open(f"{WORK}/urdf/kyo46rs_{tag}.urdf", "w").write(out)
    src = open(f"{WORK}/urdf/kyo46rs_v6.urdf").read()

    # v5only -- delete both shoulder_roll blocks, re-parent pitch to the torso
    # at the offset v4 used.  The comment body contains "->", so the comment
    # has to be matched over any character, not over "not a >".
    v5 = src
    for side, y in (("left", "0.115"), ("right", "-0.115")):
        v5 = re.sub(r"\n  <!-- v6.*?-->\n  <link name=\"%s_shoulder_roll_link\">"
                    r".*?</joint>\n" % side, "\n", v5, flags=re.S)
        v5 = v5.replace(
            f'<parent link="{side}_shoulder_roll_link"/>\n'
            f'    <child link="{side}_upper_arm_link"/>\n'
            f'    <origin xyz="0 {"0.03" if side == "left" else "-0.03"} 0" rpy="0 0 0"/>',
            f'<parent link="torso"/>\n'
            f'    <child link="{side}_upper_arm_link"/>\n'
            f'    <origin xyz="0 {y} 0.09" rpy="0 0 0"/>')
    assert "shoulder_roll" not in v5

    # v6only -- v5's hip_pitch retirement undone (torso mass/inertia + effort)
    v6o = src.replace(
        '<mass value="0.7500"/><inertia ixx="0.0034584" ixy="0" ixz="0" '
        'iyy="0.0032453" iyz="0" izz="0.0018078"/>',
        '<mass value="1.2740"/><inertia ixx="0.0058747" ixy="0" ixz="0" '
        'iyy="0.0055126" iyz="0" izz="0.0030709"/>')
    v6o = v6o.replace('<limit lower="-1.7453" upper="0.5236" effort="6.0" velocity="10.0"/>',
                      '<limit lower="-1.7453" upper="0.5236" effort="12.0" velocity="10.0"/>')
    assert v6o.count('effort="12.0"') == 2 and 'mass value="1.2740"' in v6o

    # v6fixed -- the axis, and only the axis, taken away
    v6f = src
    for side in ("left", "right"):
        v6f = re.sub(r'(<joint name="%s_shoulder_roll_joint" type=")revolute(">)' % side,
                     r'\1fixed\2', v6f)

    # The recommendation of Sec.34, as one file: v5's torque halving kept, its
    # 0.524 kg put back in the torso, the roll axis locked, the sole widened to
    # 60 mm with the stance left at 100 mm. Emitted here so the push bench and
    # the demo script can point at a path that regenerates, rather than at
    # whatever happened to be in a scratch directory.
    M_LIGHT = ('<mass value="0.7500"/><inertia ixx="0.0034584" ixy="0" ixz="0" '
               'iyy="0.0032453" iyz="0" izz="0.0018078"/>')
    M_HEAVY = ('<mass value="1.2740"/><inertia ixx="0.0058747" ixy="0" ixz="0" '
               'iyy="0.0055126" iyz="0" izz="0.0030709"/>')
    SOLE38 = '<geometry><box size="0.098 0.038 0.012"/></geometry>'
    SOLE60 = '<geometry><box size="0.098 0.06 0.012"/></geometry>'
    rec = v6f.replace(M_LIGHT, M_HEAVY).replace(SOLE38, SOLE60)
    assert 'mass value="1.2740"' in rec and rec.count(SOLE60) == 4
    # Same stack with the roll axis left free -- for the guard, where ARM_HOLD
    # commands the arm instead of the joint being dead (Sec.35.2).
    rec_free = src.replace(M_LIGHT, M_HEAVY).replace(SOLE38, SOLE60)

    for name, txt, want in (("v5only", v5, 16), ("v6only", v6o, 18), ("v6fixed", v6f, 16),
                            ("v6_rec", rec, 16), ("v6_rec_free", rec_free, 18)):
        n = txt.count('type="revolute"')
        assert n == want, f"{name}: {n} revolute joints, expected {want}"
        open(f"{WORK}/urdf/kyo46rs_{name}.urdf", "w").write(txt)


# ---- runner ---------------------------------------------------------------

def one(rev, vy, fs, extra=None):
    env = dict(os.environ)
    env["LD_LIBRARY_PATH"] = MUJOCO_LIB + ":" + env.get("LD_LIBRARY_PATH", "")
    env.update(COMMON)
    env.update(extra or {})
    env["URDF"] = f"{WORK}/urdf/kyo46rs_{rev}.urdf"
    env["VY"] = f"{vy}"
    env["FIRST_SWING"] = str(fs)
    p = subprocess.run([BIN], env=env, capture_output=True, text=True, timeout=600)
    txt = p.stdout + p.stderr
    r = {"rev": rev, "vy": f"{vy:+.3f}", "first_swing": fs}
    r.update({k: v for k, v in (extra or {}).items()})
    for k, pat in PATTERNS.items():
        m = re.search(pat, txt, re.S)
        r[k] = m.group(1) if m else ""
    r["survived"] = "SURVIVED" if "verdict: SURVIVED" in txt else "FELL"
    return r


def ladder(revs, extra_axis=None, jobs_workers=8):
    """extra_axis: (env_name, [values]) run as an extra condition dimension."""
    name, vals = extra_axis or (None, [None])
    jobs = [(rev, s * m, fs, ({name: str(v)} if name else None))
            for rev in revs for m in MAGS for s in SIGNS
            for fs in FIRST_SWING for v in vals]
    rows = []
    with cf.ThreadPoolExecutor(max_workers=jobs_workers) as ex:
        futs = [ex.submit(one, *j) for j in jobs]
        for i, f in enumerate(cf.as_completed(futs), 1):
            rows.append(f.result())
            print(f"\r  {i}/{len(jobs)}", end="", file=sys.stderr, flush=True)
    print(file=sys.stderr)
    return rows


# ---- reporting ------------------------------------------------------------

def safe_of(rr):
    """Largest |VY| at or below which EVERY probe survived. `safe` is the only
    well-defined number here: above it survival is speckled, same as Sec.18.2."""
    safe = 0.0
    for m in sorted({abs(float(r["vy"])) for r in rr}):
        if all(r["survived"] == "SURVIVED" for r in rr if abs(float(r["vy"])) <= m + 1e-9):
            safe = m
    return safe


def report(rows, key="rev", order=None):
    keys = order or sorted({r[key] for r in rows})
    keys = [k for k in keys if any(r[key] == k for r in rows)]
    d = {(r[key], r["vy"], str(r["first_swing"])): r for r in rows}
    vys = sorted({r["vy"] for r in rows}, key=float)
    print(f"{'VY':>7} | " + " | ".join(f"{k:^9}" for k in keys))
    print("-" * (9 + 12 * len(keys)))
    for vy in vys:
        cells = ["/".join("o" if (d.get((k, vy, fs)) or {}).get("survived") == "SURVIVED"
                          else "X" for fs in ("0", "1")) for k in keys]
        print(f"{vy:>7} | " + " | ".join(f"{c:^9}" for c in cells))
    print("\no = SURVIVED, X = FELL;  cell = FIRST_SWING 0 / 1\n")
    print(f"{key:<9} {'mass':>6} {'safe':>6} {'cells':>8} {'selfcol med':>12} {'peak N':>9}")
    for k in keys:
        rr = [r for r in rows if r[key] == k]
        tk = sorted(float(r["selfcol_ticks"] or 0) for r in rr)
        print(f"{k:<9} {rr[0]['mass']:>6} {safe_of(rr):>6.3f} "
              f"{sum(1 for r in rr if r['survived'] == 'SURVIVED'):>5}/{len(rr):<2} "
              f"{tk[len(tk) // 2]:>12.0f} "
              f"{max(float(r['selfcol_peak'] or 0) for r in rr):>9.1f}")


def dump(rows, path):
    cols = list(dict.fromkeys(k for r in rows for k in r))
    with open(path, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=cols)
        w.writeheader()
        w.writerows(rows)
    print(f"-> {path}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--stage", choices=["rev", "seed", "all"], default="all")
    ap.add_argument("--csv-dir", default=f"{REPO}/csv")
    a = ap.parse_args()
    build_urdfs()
    if a.stage in ("rev", "all"):
        print("== which mech change narrowed the lateral margin ==")
        rows = ladder(ORDER)
        report(rows, order=ORDER)
        dump(rows, f"{a.csv_dir}/vy_regression_revs.csv")
    if a.stage in ("seed", "all"):
        print("\n== does seeding the shoulder roll outward fix v6 ==")
        rows = ladder(["v6"], extra_axis=("SHOULDER_ROLL", [0.0, 0.15, 0.30, 0.60]))
        report(rows, key="SHOULDER_ROLL", order=["0.0", "0.15", "0.3", "0.6"])
        dump(rows, f"{a.csv_dir}/vy_regression_seed.csv")


if __name__ == "__main__":
    main()
