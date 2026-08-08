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

--- Disturbance mode -------------------------------------------------------

    python3 scripts/kyo46rs_bench.py --push           # the push matrix
    python3 scripts/kyo46rs_bench.py --push --push-group left

Each cell reports the largest impulse [N*s] the machine takes without falling,
next to the ankle-strategy budget that impulse is being measured against.

SURVIVAL IS NOT MONOTONIC IN THE IMPULSE, so `--push-sweep` uses a ladder and
reports a BAND rather than bisecting for a threshold. Measured on
`left ss (mirror)`: 0.45 survives, 0.50 falls, 0.55 and 0.60 survive again,
everything from 0.65 up falls -- and two probes 0.001 N*s apart landed on
opposite sides. The plain `--push` mode still bisects, which is fine for a
quick look at one cell but must not be used to compare conditions: it returns
an arbitrary point inside that band.

Two things are pinned in every cell, because leaving either free means
measuring it instead of the impulse:

* the SUPPORT PHASE (double or single), since what the machine can absorb
  differs several-fold between them, and
* WHICH FOOT is in stance, since a shove toward the stance foot and one away
  from it are not the same disturbance on a machine with a 99 mm stance width
  and a +-19 mm lateral CoP box.
"""
import argparse
import csv as csvmod
import os
import re
import subprocess
import sys

BIN = "./target/release/examples/kyo46rs_walk"
URDF_DIR = "/home/takara/work/dp/humanoid/kyo46rs_description/urdf"
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
    for w in (0.05, 0.10, 0.20, 0.40, 0.60, 0.80):
        out.append(("turn", f"wz={w:+.2f}", {"WZ": f"{w}"}))
        out.append(("turn", f"wz={-w:+.2f}", {"WZ": f"{-w}"}))
    # 0.80 rad/s needs the shorter step; the turn ceiling is ~19 deg PER STEP,
    # so the rate follows the cadence. Below t_step=0.40 it stops holding.
    for w in (0.80, 1.00):
        out.append(("turn fast", f"wz={w:+.2f} t=0.40",
                    {"WZ": f"{w}", "T_SS": "0.25", "T_DS": "0.15"}))
    for a in (0.01, 0.02, 0.04):
        out.append(("squat", f"amp={a*1e3:.0f}mm still", {"SQUAT_AMP": f"{a}", "N_STEPS": "0"}))
        out.append(("squat", f"amp={a*1e3:.0f}mm walk", {"SQUAT_AMP": f"{a}", "VX": "0.055"}))
    out.append(("combined", "vx+wz", {"VX": "0.055", "WZ": "0.10"}))
    out.append(("combined", "vx+vy", {"VX": "0.055", "VY": "0.036"}))
    out.append(("combined", "vx+squat", {"VX": "0.055", "SQUAT_AMP": "0.02"}))
    return [c for c in out if group is None or c[0] == group]


PUSH_PATTERNS = {
    "fired": r"PUSH at t=([\d.]+)",
    "stance": r"PUSH at t=[\d.]+ step=\d+ (DS|SS/[LR])",
    "budget": r"budget in that direction:\s*([\d.]+) N\*s",
    "peak_dcm": r"peak \|xi - xi_ref\| after the push =\s*([\d.]+) mm",
    "peak_tilt": r"peak tilt after the push =\s*([\d.]+) rad",
    "push_degraded": r"degraded solves after the push:\s*(\d+)",
    "recover_s": r"RECOVERED at t=[\d.]+ \(([\d.]+) s",
    "push_cop": r"peak CoP box use after the push =\s*([\d.]+)",
    "push_ankle": r"peak ankle_roll use after the push =\s*(\d+)%",
    "spill": r"PUSH WARNING: the [\d.]+ s pulse outlasts this slice by ([\d.]+) s",
    "edge_pct": r"CoP on the sole edge \([^)]*\) for \d+/\d+ ticks \((\d+)%\)",
    "edge_leave": r"left the edge (\d+) ms after the transfer",
    "xfer_t": r"weight transfer at t=[\d.]+ \(([\d.]+) s after the push\)",
    "impact": r"peak [\d.]+ N \(([\d.]+)x weight\)",
    "fz_min": r"min [\d.]+ N \(([\d.]+)x weight\)",
    "unloaded": r"unloaded ticks after the push: \d+ \((\d+) ms",
    "slid": r"sliding ticks after the push: \d+ \((\d+) ms",
}

# Landing impact, as a multiple of body weight, above which the machine has
# never survived. Established on `left ss` FIRST_SWING=0 over the whole 0.05-
# 0.80 grid: every survivor peaked at or below 1.84x, every faller at or above
# 2.84x, one exception (0.70 survived at 3.62x). The bench carries it as a
# CRITERION so that a change can be judged by whether it lowers the impact,
# not only by whether that particular run happened to stay upright -- survival
# near the limit is speckled and impact is not.
IMPACT_LIMIT_X = 2.0

# The competing criterion: how long the two feet together carried less than
# half the robot's weight. Impact is a PEAK, so one stray tick moves it, and
# measured to the ceiling it produces in-cell inversions -- `fwd ds` survives
# 2.22x and falls at 1.79x. Airborne time is an INTEGRAL of the interval where
# there was no contact to plan against, and over every grid point taken so far
# survivors sat at 0-30 ms against 65-250 ms for fallers. Both are reported so
# the claim can be checked rather than asserted.
AIRBORNE_LIMIT_MS = 50.0

# The step to push on, per support phase. Step 6 is deep enough into the walk
# that the stride ramp is over, and its single-support half stands on the RIGHT
# foot -- so `_MIRROR` gives the same phase on the other stance leg. Anything
# that reports a left/right asymmetry has to be checked against both.
PUSH_STEP = 6
PUSH_STEP_MIRROR = 7

# Per-cell centre for `--grid-around`: the baseline `safe` impulse measured on
# 2026-08-07 with `timing off` over the full 0.05-2.00 grid.
#
# Most of a full grid is deep-survive or deep-fail territory that no
# intervention moves; all the information (and all the noise) lives within a
# couple of grid steps of the boundary. Narrowing to +-4 steps costs ~3x fewer
# runs for the same discrimination.
#
# The window is per CELL, never per CONDITION -- the conditions being compared
# share one grid, which is the property that the adaptive ladder broke (Sec.
# 18.2). Cells are never compared with each other, so their windows may differ.
GRID_CENTRE = {
    "fwd ds": 1.35, "back ds": 0.75, "left ds": 0.65, "right ds": 0.95,
    "diag ds": 0.55, "fwd ss": 1.40, "back ss": 0.90, "left ss": 0.60,
    "right ss": 0.60, "diag ss": 0.70,
    "left ds (mirror)": 0.95, "left ss (mirror)": 0.45,
    "right ds (mirror)": 0.45, "right ss (mirror)": 0.55,
}


def push_cases(group=None):
    """(group, label, env-overrides) for the disturbance matrix."""
    out = []
    dirs = [
        ("fwd", 0.0), ("back", 180.0),
        ("left", 90.0), ("right", 270.0),
        ("diag", 45.0),
    ]
    for phase in ("ds", "ss"):
        for name, deg in dirs:
            cell = {"PUSH_DEG": f"{deg}", "PUSH_SUPPORT": phase,
                    "PUSH_STEP": PUSH_STEP}
            if phase == "ds":
                # Double support is only 0.20 s against a 0.10 s pulse, so the
                # midpoint puts the tail of the pulse on liftoff. Fire earlier
                # so the whole disturbance lands inside the phase being named.
                cell["PUSH_AT"] = "0.25"
            out.append((f"{name}", f"{name} {phase}", cell))
    # Same lateral pushes one step later, which swaps the two feet's roles.
    #
    # In single support that means the other leg is the stance leg. In double
    # support there is no stance leg to mirror -- but DS is not symmetric in
    # TIME either: the CoP is walking from the foot that is unloading to the
    # foot that is loading, so a shove toward the departing foot and one toward
    # the arriving foot are different disturbances. The (b) sweep found DS
    # left 0.35-0.65 against DS right 0.65-1.00 in all six conditions, and
    # these cells are what decides whether that 1.5x is the machine's left/
    # right or the feet's roles in that particular double support.
    for name, deg in (("left", 90.0), ("right", 270.0)):
        for phase in ("ds", "ss"):
            cell = {"PUSH_DEG": f"{deg}", "PUSH_SUPPORT": phase,
                    "PUSH_STEP": PUSH_STEP_MIRROR}
            if phase == "ds":
                cell["PUSH_AT"] = "0.25"
            out.append((f"{name}", f"{name} {phase} (mirror)", cell))
    # Yaw is deliberately NOT in this matrix. It is an angular impulse
    # (N*m*s) with a different budget, and putting it in a column of N*s
    # invites reading the two as comparable. The driver takes
    # PUSH_YAW_IMPULSE for it; give it its own table when it is wanted.
    if group is None:
        return out
    want = {g.strip() for g in group.split(",")}
    return [c for c in out if c[0] in want]


PATTERNS = {
    "steps": r"steps taken:\s*(\d+)",
    "achieved": r"achieved after the ramp \([\d.]+ s\): (.*)",
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
    env.update({k: str(v) for k, v in env_extra.items()})
    # Duration follows the step time, so a faster-cadence case still gets its
    # full N_STEPS plus the same settle window.
    t_step = float(env["T_SS"]) + float(env["T_DS"])
    env["T"] = f"{2.0 + int(env['N_STEPS']) * t_step + 2.0}"
    env.setdefault("STRIDE_RAMP", COMMON["STRIDE_RAMP"])
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


# ---- disturbance mode ----------------------------------------------------

def run_push(envx, impulse, cmd_env):
    """One push run. Returns (verdict, parsed) where verdict is one of
    'absorbed' | 'recovered' | 'unsettled' | 'fell' | 'nofire'."""
    env = dict(cmd_env)
    env.update({k: str(v) for k, v in envx.items()})
    env["PUSH_IMPULSE"] = f"{impulse:.4f}"
    r, txt = run(env)
    for k, pat in PUSH_PATTERNS.items():
        m = re.search(pat, txt)
        r[k] = m.group(1) if m else ""
    if not r["fired"]:
        # A cell whose phase never matched is not a survival -- it is a
        # missing measurement, and reporting it as "took the impulse fine"
        # is how a bench comes to claim a limit it never tested.
        v = "nofire"
    elif r["survived"] == "FELL":
        v = "fell"
    elif "ABSORBED" in txt:
        v = "absorbed"
    elif "RECOVERED at" in txt:
        v = "recovered"
    else:
        v = "unsettled"
    r["verdict_push"] = v
    r["impulse"] = f"{impulse:.4f}"
    return v, r


def grid_limit(envx, cmd_env, step=0.05, hi=1.20, log=None, centre=None,
               half_pts=4):
    """Survival over a FIXED impulse grid, identical for every cell and
    condition. Returns (safe, fail, n_surv_in_band, n_in_band, best_row).

    The grid is fixed on purpose. `ladder_limit` below brackets geometrically
    and then refines, which makes the probe set depend on where that cell's
    first fall happened -- so two conditions get different probe sets and their
    `safe` values are not measured on the same ruler. It also silently
    overstates safety: on `left ss (mirror)` it reported safe=0.516 while a
    hand ladder at 0.05 resolution found 0.50 falling, purely because 0.50 was
    not on its grid. `safe` only ever means "no probe at or below this fell",
    so the grid resolution IS the error bar and it has to be the same
    everywhere for the numbers to be comparable.
    """
    if centre is None:
        n = int(round(hi / step))
        pts = [step * (k + 1) for k in range(n)]
    else:
        lo = max(step, centre - half_pts * step)
        pts = [round(lo + k * step, 4) for k in range(2 * half_pts + 1)]
    seen, best, detail = [], None, []
    for p in pts:
        v, r = run_push(envx, p, cmd_env)
        if log:
            log(p, v, r)
        if v == "nofire":
            return None, None, 0, 0, None
        surv = v != "fell"
        seen.append((p, surv))
        detail.append((p, surv, r))  # r carries edge_pct / edge_leave
        if surv:
            best = r
    safe = 0.0
    for q, s in seen:
        if not s:
            break
        safe = q
    fail = None
    for q, s in reversed(seen):
        if s:
            break
        fail = q
    band = [(q, s) for q, s in seen if q > safe and (fail is None or q < fail)]
    if best is not None:
        best = dict(best)
        best["_detail"] = detail
    return safe, fail, sum(1 for _, s in band if s), len(band), best


def ladder_limit(envx, cmd_env, hi=8.0, fine_n=8, log=None):
    """Survival limit by LADDER, not bisection -- survival is not monotonic.

    Measured on `left ss (mirror)`: 0.45 survives, 0.50 falls, 0.55 and 0.60
    survive again, and everything from 0.65 up falls. Two probes 0.001 N*s
    apart land on opposite sides (0.550 survived, 0.551 fell). So there is no
    threshold to bisect for; near the boundary the outcome is speckled, and a
    bisection returns an arbitrary point inside the speckle while looking like
    a limit. Reporting that number and comparing it across conditions is
    comparing noise.

    What is well-defined, and what this returns:

      safe  -- the largest impulse such that EVERY probe at or below it
               survived. The conservative limit; this is the number to compare.
      fail  -- the smallest impulse such that EVERY probe at or above it fell.
      speck -- how many probes strictly between the two survived, out of how
               many were run there. `fail / safe` is how wide the speckle is.

    Returns (safe, fail, n_surv_in_band, n_in_band, best_row) or
    (None, ...) when the cell never fired.
    """
    seen = []  # (impulse, survived?)
    best = None

    def probe_at(p):
        nonlocal best
        v, r = run_push(envx, p, cmd_env)
        if log:
            log(p, v, r)
        if v == "nofire":
            return None
        surv = v != "fell"
        seen.append((p, surv))
        if surv:
            best = r
        return surv

    # Stage 1: geometric bracket. 1.5x rather than 2x so the fine ladder that
    # follows does not have to span a factor of two.
    p, first_fall = 0.18, None
    while p <= hi:
        s = probe_at(p)
        if s is None:
            return None, None, 0, 0, None
        if not s:
            first_fall = p
            break
        p *= 1.5
    if first_fall is None:
        return hi, None, 0, 0, best

    # Stage 2: fine ladder over the region where the outcome can flip. Up to
    # 1.4x past the first fall, because a survival above the first fall is
    # exactly the thing being looked for.
    lo = max((q for q, s in seen if s), default=0.0)
    top = first_fall * 1.4
    for k in range(1, fine_n + 1):
        probe_at(lo + (top - lo) * k / fine_n)

    seen.sort()
    # safe: walk up while everything stays alive.
    safe = 0.0
    for q, s in seen:
        if not s:
            break
        safe = q
    # fail: walk down while everything stays dead.
    fail = None
    for q, s in reversed(seen):
        if s:
            break
        fail = q
    band = [(q, s) for q, s in seen if q > safe and (fail is None or q < fail)]
    return safe, fail, sum(1 for _, s in band if s), len(band), best


def bisect_limit(envx, cmd_env, lo=0.0, hi=8.0, tol=0.02, log=None):
    """Largest impulse the cell survives, by doubling then bisecting.

    `lo` is known-survivable and `hi` is the ceiling to search under. Returns
    (limit, best_row, worst_row). A cell that survives `hi` reports `hi` with a
    '>=' marker rather than silently claiming a limit it never bracketed.
    """
    # Bracket upward first: start small and double until something falls.
    probe = 0.18
    survived, best = lo, None
    fell_at = None
    while probe <= hi:
        v, r = run_push(envx, probe, cmd_env)
        if log:
            log(probe, v, r)
        if v == "nofire":
            return None, r, r
        if v == "fell":
            fell_at = probe
            worst = r
            break
        survived, best = probe, r
        probe *= 2.0
    else:
        return survived, best, best  # never fell under `hi`
    # Bisect the bracket [survived, fell_at].
    while fell_at - survived > tol:
        mid = 0.5 * (survived + fell_at)
        v, r = run_push(envx, mid, cmd_env)
        if log:
            log(mid, v, r)
        if v == "fell":
            fell_at, worst = mid, r
        else:
            survived, best = mid, r
    return survived, best, worst


# The (b) sweep: footstep adaptation crossed with the CoP-loop gain.
#
# These two are not independent knobs. Sec.14.3 measured that step adjustment
# does nothing at k_dcm=2.0 and only starts working once the CoP loop is
# demoted -- two loops fighting over the same error. So the sweep has to be a
# CROSS, not two one-at-a-time ladders, or the conclusion "capture point does
# not help" comes back for the third time.
PUSH_CONDITIONS = [
    (f"adapt{ad} k{k}", {"ADAPT_STEP": str(ad), "K_DCM": str(k)})
    for ad in (0, 1)
    for k in (2.0, 1.0, 0.5)
] + [
    # Step TIMING adaptation (dcm::step_timing), against the same baseline.
    # Kept separate from the adapt/k cross because it is a different layer:
    # placement moves WHERE the next foot goes, timing moves WHEN.
    ("timing off", {"ADAPT_STEP": "0", "K_DCM": "2.0", "ADAPT_TIME": "0"}),
    ("timing on", {"ADAPT_STEP": "0", "K_DCM": "2.0", "ADAPT_TIME": "1"}),
    # Same, with a deadband wide enough to stop the every-tick retiming seen
    # on the first runs (54-60 ticks / ~500 ms even with no disturbance at
    # all). If the benefit survives this, it is a response to the push; if it
    # does not, the first result was the gait quietly changing shape.
    ("timing dead20", {"ADAPT_STEP": "0", "K_DCM": "2.0", "ADAPT_TIME": "1",
                       "ADAPT_TIME_DEAD": "0.020"}),
    # The reduced-step reflex on top of timing: when the solve reports
    # Unreachable (the DCM is outboard of the stance ZMP, measured on 75 of 83
    # ticks in a run that falls), end the step instead of doing nothing.
    ("timing cut", {"ADAPT_STEP": "0", "K_DCM": "2.0", "ADAPT_TIME": "1",
                    "ADAPT_TIME_CUT": "1"}),
    # Sole width. The ankle-roll actuator is a cylinder of radius 23 mm with
    # its axis fore-aft, so it is ALREADY 46 mm across in y while the sole
    # plate is 38 mm -- widening to 46 adds no swept volume at all. 60 mm is
    # the ceiling this sweep measures; the inner gap between the two feet at a
    # 99.4 mm stance goes 61.4 -> 39.4 mm there, which the swing foot has to
    # clear (doc Sec.10.6 records it hitting the stance foot once already).
    # Both the URDF box and the WBC's assumed CoP box have to move together or
    # the controller is planning against a foot the robot does not have.
    # Cadence. `t_ss` has been 0.35 s in every result in this doc, fixed in
    # COMMON and never swept. It is the strongest untried lever on lateral
    # balance because the periodic-walk DCM offset depends on it
    # EXPONENTIALLY: b_y = l_p / (1 + exp(omega * t_ss)) goes 12.3 -> 21.5 mm
    # between 0.35 and 0.25 s, and a faster cadence also puts the foot down
    # more often, which is the one thing a small-footed machine can do about a
    # sideways push. Sec.16.5 already ran this robot at t_step 0.40 for fast
    # turns, so the shorter end is known to be feasible.
    ("cad 0.35", {}),
    ("cad 0.30", {"T_SS": "0.30"}),
    ("cad 0.25", {"T_SS": "0.25"}),
    ("cad 0.20", {"T_SS": "0.20"}),
    ("sole 38", {}),
    ("sole 46", {"URDF": f"{URDF_DIR}/kyo46rs_sole46.urdf", "SOLE_HALF_W": "0.0230"}),
    ("sole 60", {"URDF": f"{URDF_DIR}/kyo46rs_sole60.urdf", "SOLE_HALF_W": "0.0300"}),
]


def push_conditions(only=None):
    if not only:
        return PUSH_CONDITIONS
    want = {w.strip() for w in only.split(",")}
    return [c for c in PUSH_CONDITIONS if c[0] in want]


# Physically meaningless perturbations used to measure the bench's own noise.
#
# A 1e-7 RELATIVE change in the ground friction -- one ten-millionth of 0.7 --
# moves the 24-point survival count of a single cell by +-2, and individual
# grid points flip. That is the error bar on every single-cell comparison this
# script can make, and it is bigger than several effects that were reported as
# real before it was measured (doc Sec.20). Nothing derived from one cell's
# count is meaningful below about +-8%.
def perturbations(n):
    return [("", {})] if n <= 1 else [
        (f"+{k}e-7", {"MU_GROUND": f"{0.7 + k * 1e-7:.9f}"}) for k in range(n)
    ]


def push_sweep_main(a):
    """One limit per (cell, condition). Rows are cells, columns conditions."""
    base = {"VX": a.vx, "PUSH_DT": a.push_dt, "PUSH_AT": a.push_at,
            "PUSH_RECOVER_MM": a.recover_mm, "PUSH_RECOVER_S": a.recover_s,
            "N_STEPS": a.push_steps}
    if a.first_swing is not None:
        base["FIRST_SWING"] = a.first_swing
    cells = push_cases(a.push_group)
    grid = {}
    extra = {}
    edge = {}
    surv_map = {}
    repeat_counts = {}
    reps = perturbations(a.repeat)
    for cond_label, cond_env in push_conditions(a.push_conditions):
      for rep_label, rep_env in reps:
        cmd_env = dict(base)
        cmd_env.update(cond_env)
        cmd_env.update(rep_env)
        for _, label, envx in cells:
            safe, fail, n_s, n_b, best = grid_limit(
                envx, cmd_env, step=a.grid_step, hi=a.grid_max,
                centre=GRID_CENTRE.get(label) if a.grid_around else None,
                half_pts=a.grid_half)
            key = (label, cond_label)
            grid[key] = (safe, fail, n_s, n_b)
            b = best or {}
            extra.setdefault(label, {"budget": b.get("budget", ""),
                                     "stance": b.get("stance", "")})
            # Does the impact criterion separate this cell's survivors from
            # its fallers? Reported per cell, because a threshold that only
            # holds on the cell it was fitted to is not a criterion.
            det = (best or {}).get("_detail", [])
            repeat_counts.setdefault(key, []).append(
                sum(1 for _, sv, _r in det if sv))
            # `run_push` stores `m.group(1)`, i.e. a STRING -- indexing it
            # takes the first character, which turned 2.84 into 2.0 and made
            # every cell look like it separated at a suspiciously round number.
            surv_x = [float(r["impact"]) for _, sv, r in det
                      if sv and r.get("impact")]
            fell_x = [float(r["impact"]) for _, sv, r in det
                      if not sv and r.get("impact")]
            extra[label]["max_surv_x"] = max(surv_x) if surv_x else None
            extra[label]["min_fell_x"] = min(fell_x) if fell_x else None
            surv_a = [float(r["unloaded"]) for _, sv, r in det
                      if sv and r.get("unloaded")]
            fell_a = [float(r["unloaded"]) for _, sv, r in det
                      if not sv and r.get("unloaded")]
            extra[label]["max_surv_air"] = max(surv_a) if surv_a else None
            extra[label]["min_fell_air"] = min(fell_a) if fell_a else None
            # CoP-edge dwell, per grid point, for the paired comparison. A
            # point with no transfer (run ended first) is dropped rather than
            # scored 0 -- absence of a transfer is not a clean transfer.
            edge[(label, cond_label)] = {
                p: float(r["edge_pct"])
                for p, _sv, r in det if r.get("edge_pct")
            }
            surv_map[(label, cond_label)] = {p: sv for p, sv, _r in det}
            print(f"  {label:>17} {cond_label:>12} -> safe "
                  f"{'nofire' if safe is None else format(safe, '.3f')}"
                  f"  fail {'-' if fail is None else format(fail, '.3f')}"
                  f"  speckle {n_s}/{n_b}"
                  f"  impact surv<={extra[label]['max_surv_x'] or float('nan'):.2f}x "
                  f"fell>={extra[label]['min_fell_x'] or float('nan'):.2f}x", flush=True)

    labels = [c[1] for c in cells]
    conds = [c[0] for c in push_conditions(a.push_conditions)]
    print(f"\n=== Safe impulse [N*s] -- everything at or below survives. "
          f"vx={a.vx}, pulse={a.push_dt} s, grid {a.grid_step} to {a.grid_max} ===")
    hdr = f"{'cell':>17} {'budget':>7} " + " ".join(f"{c:>11}" for c in conds)
    print(hdr)
    print("-" * len(hdr))
    for lb in labels:
        budget = extra[lb]["budget"] or "-"
        vals = []
        for c in conds:
            safe = grid[(lb, c)][0]
            vals.append("nofire" if safe is None else f"{safe:.3f}")
        # Mark the best condition for this cell so the table can be read a row
        # at a time; the interesting claim is per-cell, not per-column.
        nums = [float(v) for v in vals if v != "nofire"]
        best_v = max(nums) if nums else None
        cells_txt = " ".join(
            f"{v:>11}" if best_v is None or v == "nofire" or float(v) < best_v
            else f"{v + '*':>11}"
            for v in vals
        )
        print(f"{lb:>17} {budget:>7} {cells_txt}")
    print("\n* = best condition for that cell.  budget = ankle-strategy "
          "impulse in that direction.")
    print("\n=== Speckle: fail/safe, and survivals inside the band ===")
    shdr = f"{'cell':>17} " + " ".join(f"{c:>13}" for c in conds)
    print(shdr)
    print("-" * len(shdr))
    for lb in labels:
        parts = []
        for c in conds:
            safe, fail, n_s, n_b = grid[(lb, c)]
            if safe is None:
                parts.append(f"{'nofire':>13}")
            elif fail is None:
                parts.append(f"{'no fail':>13}")
            else:
                parts.append(f"{fail / max(safe, 1e-9):>6.2f}x {n_s}/{n_b:<4}")
        print(f"{lb:>17} " + " ".join(parts))
    print("\nfail/safe = 1.00 would be a clean threshold. Anything above it is "
          "a band where the\noutcome flips, and a single run inside that band "
          "is not a measurement of anything.")
    print("\n=== Two candidate criteria, per cell: worst survivor vs best faller ===")
    ihdr = (f"{'cell':>17} | {'impact surv':>11} {'impact fell':>11} {'sep':>4} "
            f"| {'air surv ms':>11} {'air fell ms':>11} {'sep':>4}")
    print(ihdr)
    print("-" * len(ihdr))
    tally = {"impact": [0, 0], "air": [0, 0]}
    for lb in labels:
        cols = []
        for key, name in ((("max_surv_x", "min_fell_x"), "impact"),
                          (("max_surv_air", "min_fell_air"), "air")):
            ws, bf = extra[lb].get(key[0]), extra[lb].get(key[1])
            if ws is None or bf is None:
                cols.append(f"{'-':>11} {'-':>11} {'n/a':>4}")
                continue
            tally[name][1] += 1
            ok = ws < bf
            tally[name][0] += 1 if ok else 0
            cols.append(f"{ws:>11.2f} {bf:>11.2f} {('yes' if ok else 'NO'):>4}")
        print(f"{lb:>17} | {cols[0]} | {cols[1]}")
    print(f"\nimpact:   {tally['impact'][0]}/{tally['impact'][1]} cells separated")
    print(f"airborne: {tally['air'][0]}/{tally['air'][1]} cells separated")
    print("A cell marked NO has a survivor scoring worse than one of its own "
          "fallers, which\nrules that quantity out as a criterion for that cell.")
    # Column summary: does any condition win across the board?
    print(f"\n{'condition':>12} {'weakest cell':>13} {'median':>8} {'sum':>8}")
    for c in conds:
        nums = sorted(v for lb in labels if (v := grid[(lb, c)][0]) is not None)
        if not nums:
            continue
        med = nums[len(nums) // 2]
        print(f"{c:>12} {nums[0]:>13.3f} {med:>8.3f} {sum(nums):>8.2f}")
    if a.repeat > 1:
        print(f"\n=== Noise floor: the same grid under {a.repeat} physically "
              f"meaningless perturbations ===")
        print("Survivals out of the grid, per repeat. The spread is the error "
              "bar on this cell.")
        nhdr = f"{'cell':>17} {'condition':>14} {'counts':>28} {'spread':>7}"
        print(nhdr)
        print("-" * len(nhdr))
        worst = 0
        for (lb, cd), counts in repeat_counts.items():
            if len(counts) < 2:
                continue
            sp = max(counts) - min(counts)
            worst = max(worst, sp)
            print(f"{lb:>17} {cd:>14} {str(counts):>28} {sp:>7}")
        print(f"\nWorst spread {worst} grid points. Any difference between two "
              f"conditions smaller\nthan this, on one cell, is not a "
              f"measurement.")

    # Pairwise against the FIRST condition, so a three-way sweep (e.g. a
    # baseline and two variants) still gets the paired read rather than only
    # the per-condition totals.
    for c1 in conds[1:]:
        c0 = conds[0]
        print(f"\n=== CoP-edge dwell after the transfer, {c0} -> {c1} ===")
        print("Percent of the first 100 ms in single support with the stance "
              "CoP on the sole\nedge. Lower is better: a foot on the edge cannot "
              "modulate its own pressure.")
        thdr = (f"{'cell':>17} {'mean %':>13} {'better':>7} {'worse':>6} "
                f"{'survived':>12}")
        print(thdr)
        print("-" * len(thdr))
        tot_b = tot_w = 0
        for lb in labels:
            e0, e1 = edge.get((lb, c0), {}), edge.get((lb, c1), {})
            common = sorted(set(e0) & set(e1))
            if not common:
                print(f"{lb:>17} {'-':>13} {'-':>7} {'-':>6} {'-':>12}")
                continue
            m0 = sum(e0[p] for p in common) / len(common)
            m1 = sum(e1[p] for p in common) / len(common)
            b = sum(1 for p in common if e1[p] < e0[p] - 1e-9)
            w_ = sum(1 for p in common if e1[p] > e0[p] + 1e-9)
            tot_b += b
            tot_w += w_
            s0 = sum(1 for v in surv_map.get((lb, c0), {}).values() if v)
            s1 = sum(1 for v in surv_map.get((lb, c1), {}).values() if v)
            n = len(surv_map.get((lb, c0), {}))
            print(f"{lb:>17} {m0:>5.0f} -> {m1:>4.0f} {b:>7} {w_:>6} "
                  f"{s0:>4}/{n} -> {s1:>2}/{n}")
        print(f"\n{tot_b} grid points improved, {tot_w} worsened, over all cells.")

    if a.csv:
        with open(a.csv, "w", newline="") as f:
            w = csvmod.writer(f)
            w.writerow(["cell", "stance", "budget_ns"]
                       + [f"{c} {k}" for c in conds
                          for k in ("safe", "fail", "speckle")])
            for lb in labels:
                row = [lb, extra[lb]["stance"], extra[lb]["budget"]]
                for c in conds:
                    safe, fail, n_s, n_b = grid[(lb, c)]
                    row += ["" if safe is None else f"{safe:.3f}",
                            "" if fail is None else f"{fail:.3f}",
                            f"{n_s}/{n_b}"]
                    _ = (n_s, n_b)
                w.writerow(row)
        print(f"\nwrote {a.csv}")


def push_main(a):
    # Fewer steps than the command matrix: the push lands at step 6 and the
    # rest of the run only has to be long enough to decide the recovery dwell.
    cmd_env = {"VX": a.vx, "PUSH_DT": a.push_dt, "PUSH_AT": a.push_at,
               "PUSH_RECOVER_MM": a.recover_mm, "PUSH_RECOVER_S": a.recover_s,
               "N_STEPS": a.push_steps}
    rows = []
    hdr = (f"{'dir':>16} {'stance':>7} {'limit N*s':>10} {'budget':>7} {'x':>5} "
           f"{'outcome':>10} {'dcm_mm':>7} {'tilt':>6} {'cop':>5} {'ankl':>5} "
           f"{'degr':>5} {'rec_s':>6}")
    print(f"vx={a.vx} m/s   pulse={a.push_dt} s   recover: <{a.recover_mm} mm "
          f"for {a.recover_s} s")
    print(hdr)
    print("-" * len(hdr))
    for group, label, envx in push_cases(a.push_group):
        verbose = (lambda i, v, r: print(f"      probe {i:6.3f} -> {v}")) if a.verbose else None
        limit, best, worst = bisect_limit(
            envx, cmd_env, hi=a.push_max, tol=a.push_tol, log=verbose)
        if limit is None:
            print(f"{label:>16} {'-':>7} {'NO FIRE':>10}   -- the requested phase "
                  f"never matched; check PUSH_STEP / PUSH_SUPPORT")
            rows.append({"cell": label, "limit": "", "note": "nofire"})
            continue
        b = best or {}
        budget = float(b.get("budget") or 0.0)
        ratio = (limit / budget) if budget > 0 else float("nan")
        spill = "  <- PULSE SPILLS PHASE" if b.get("spill") else ""
        print(f"{label:>16} {b.get('stance', '-'):>7} {limit:>10.3f} "
              f"{budget:>7.2f} {ratio:>5.2f} {b.get('verdict_push', '-'):>10} "
              f"{b.get('peak_dcm', '-'):>7} {b.get('peak_tilt', '-'):>6} "
              f"{b.get('push_cop', '-'):>5} {b.get('push_ankle', '-'):>5} "
              f"{b.get('push_degraded', '-'):>5} {b.get('recover_s', '-'):>6}{spill}")
        rows.append({"cell": label, "stance": b.get("stance", ""),
                     "limit_ns": f"{limit:.3f}", "budget_ns": b.get("budget", ""),
                     "ratio": f"{ratio:.2f}", "outcome": b.get("verdict_push", ""),
                     "peak_dcm_mm": b.get("peak_dcm", ""),
                     "peak_tilt_rad": b.get("peak_tilt", ""),
                     "cop_use": b.get("push_cop", ""),
                     "ankle_use_pct": b.get("push_ankle", ""),
                     "degraded": b.get("push_degraded", ""),
                     "recover_s": b.get("recover_s", ""),
                     "pulse_spill_s": b.get("spill", "")})
    if a.csv:
        with open(a.csv, "w", newline="") as f:
            w = csvmod.DictWriter(f, fieldnames=list(rows[0].keys()))
            w.writeheader()
            w.writerows(rows)
        print(f"\nwrote {a.csv}")
    lims = [float(r["limit_ns"]) for r in rows if r.get("limit_ns")]
    if lims:
        print(f"\nweakest cell {min(lims):.3f} N*s, strongest {max(lims):.3f} N*s "
              f"-- a {max(lims) / max(min(lims), 1e-9):.1f}x spread across "
              f"direction and phase")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--group")
    ap.add_argument("--csv")
    ap.add_argument("--record", help="label of one case whose trajectory CSV to keep")
    ap.add_argument("--robot", help="profile name (kyo46rs | kyo46rs2 | g1)")
    ap.add_argument("--push", action="store_true",
                    help="disturbance mode: bisect the surviving impulse per cell")
    ap.add_argument("--push-group",
                    help="direction groups, comma-separated (fwd|back|left|right|diag)")
    ap.add_argument("--push-max", type=float, default=8.0,
                    help="ceiling for the impulse search, N*s")
    ap.add_argument("--push-tol", type=float, default=0.02,
                    help="bisection tolerance, N*s")
    ap.add_argument("--push-dt", default="0.10", help="pulse duration, s")
    ap.add_argument("--push-at", default="0.5",
                    help="fraction through the matched slice at which to fire")
    ap.add_argument("--recover-mm", default="20", help="recovery band, mm of DCM error")
    ap.add_argument("--recover-s", default="0.5", help="dwell inside the band, s")
    ap.add_argument("--vx", default="0.055", help="forward command during the push")
    ap.add_argument("--push-steps", default="20", help="steps per push run")
    ap.add_argument("--repeat", type=int, default=1,
                    help="repeat each grid under N negligible perturbations, "
                         "to measure this bench's own noise floor")
    ap.add_argument("--push-conditions",
                    help="comma-separated condition labels, e.g. 'adapt0 k2.0'")
    ap.add_argument("--grid-step", type=float, default=0.05,
                    help="impulse grid resolution, N*s (this IS the error bar)")
    ap.add_argument("--grid-around", action="store_true",
                    help="grid only +-grid-half steps around each cell's known "
                         "boundary (GRID_CENTRE) instead of the full range")
    ap.add_argument("--grid-half", type=int, default=4,
                    help="half-width of the --grid-around window, in steps")
    ap.add_argument("--grid-max", type=float, default=1.20,
                    help="top of the impulse grid, N*s")
    ap.add_argument("--push-sweep", action="store_true",
                    help="cross the push matrix with ADAPT_STEP x K_DCM")
    ap.add_argument("--first-swing",
                    help="override FIRST_SWING (which leg swings first)")
    ap.add_argument("--verbose", action="store_true", help="print every probe")
    a = ap.parse_args()

    if a.robot:
        COMMON["ROBOT"] = a.robot
        print(f"robot: {a.robot}")
    if not os.path.exists(BIN):
        sys.exit(f"{BIN} not found -- cargo build --release --features mujoco --examples")

    if a.push_sweep:
        push_sweep_main(a)
        return
    if a.push:
        push_main(a)
        return

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
        # A case with no locomotion command (standing squat) has nothing to
        # track, so surviving IS the criterion -- counting it as a tracking
        # failure understated the pass rate by exactly those three rows.
        pcts = [int(m) for m in re.findall(r"\((-?\d+)%\)", r["achieved"] or "")]
        return all(p >= 85 for p in pcts)
    n_real = sum(1 for r in rows if r["survived"] == "SURVIVED" and tracked(r))
    print(f"\n{n_ok}/{len(rows)} survived; {n_real}/{len(rows)} also tracked "
          f"the command to >=85% on every commanded axis")


if __name__ == "__main__":
    main()
