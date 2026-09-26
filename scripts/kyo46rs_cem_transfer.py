#!/usr/bin/env python3
"""CEM over the stair-transfer recipe, with a progress-dominant objective.

Why this file exists
--------------------
The earlier CEM run (log/claude_cem2_20260919) was driven by an ad-hoc script
that was never committed, and its objective had a defect recorded in
doc/from_claude/20260919_cem2_legal_ascent.md: standing still scored higher
than making progress and falling, and the search duly converged on a recipe
that survives the whole test without ever completing the transfer.

`score()` below fixes that and is the only place the trade-off is expressed:
the progress terms (steps entered, treads loaded, transfer completed) outweigh
the survival terms, so "advanced two more steps and fell at 16 s" beats
"stood on one leg for 18 s". Joint-limit violations are a penalty rather than a
veto, so the search can cross an illegal region on the way to a legal optimum
instead of seeing a flat landscape.

The search space now includes the phase-scoped gains (`*_XFER` / `*_POST`, see
doc/from_claude/20260919_phase_scoped_gains.md) and the parallel thread's
`STAIR_RESUME_SWING_COUPLE`, which are the two axes measured to move the
transfer/resume boundary at all.

Usage
-----
    python3 scripts/kyo46rs_cem_transfer.py \
        --base log/claude_cem2_20260919/BEST/controls.json \
        --binary target/release/examples/kyo46rs_walk \
        --out log/claude_cem3_<date> --gens 6 --pop 16 --elite 5 --jobs 8

Re-running with the same --out resumes from cem_state.json.
"""
import argparse
import json
import math
import os
import random
import re
import shutil
import statistics
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from kyo46rs_stairs_bench import REPO, sha, summarize  # noqa: E402

# name -> (low, high, initial mean, initial sigma)
# Transfer timings/loads seed from the generation-4 mean of the previous run
# (log/claude_cem2_20260919/cem_state.json). The gains seed at their neutral
# value with a wide sigma, because neutral is the only point measured for them.
SPACE = {
    "STAIR_CAPTURE_TIME":     (0.30, 0.80, 0.474901, 0.030),
    "STAIR_CAPTURE_TOL":      (0.001, 0.008, 0.003091, 0.0006),
    "STAIR_CAPTURE_MARGIN":   (0.40, 0.95, 0.705619, 0.030),
    "STAIR_CAPTURE_INBOARD":  (0.000, 0.020, 0.006252, 0.0020),
    "STAIR_RISE_TIME":        (0.80, 3.00, 1.600493, 0.300),
    "STAIR_RISE_LOAD":        (0.20, 0.70, 0.434434, 0.030),
    "STAIR_COMMIT_TIME":      (0.30, 1.50, 0.639975, 0.120),
    "STAIR_COMMIT_LOAD":      (0.50, 0.95, 0.715981, 0.030),
    "STAIR_RECONTACT_LOAD":   (0.010, 0.150, 0.050531, 0.012),
    "STAIR_RECONTACT_Z":      (0.001, 0.012, 0.005299, 0.0010),
    "STAIR_RECONTACT_ANGLE":  (0.02, 0.25, 0.092158, 0.015),
    "STAIR_RECONTACT_ALPHA":  (0.50, 0.99, 0.741996, 0.040),
    "STAIR_TRANSFER_DCM":     (0.010, 0.060, 0.026874, 0.0030),
    "STAIR_TRANSFER_Z":       (0.005, 0.030, 0.013223, 0.0020),
    "STAIR_TRANSFER_TILT":    (0.10, 0.35, 0.188329, 0.015),
    "STAIR_TRUNK_COUPLE":     (0.10, 10.0, 2.145651, 1.100),
    "STAIR_FFDC":             (0.0, 1.5e-3, 1.1195e-4, 1.0e-4),
    # Phase-scoped gains: the two axes that moved the transfer/resume boundary.
    "KP_SWING_XFER":          (200.0, 2400.0, 400.0, 500.0),
    "KD_SWING_XFER":          (10.0, 160.0, 40.0, 35.0),
    "KP_TRUNK_POST":          (100.0, 1200.0, 300.0, 250.0),
    "STAIR_RESUME_SWING_COUPLE": (0.0, 0.40, 0.10, 0.08),
}

TRANSFER_LINE = re.compile(
    r"STAIR TRANSFER:\s*(\d+) started / (\d+) completed / (\d+) timeouts")


def score_flat(stairs, _transfer_completed, report=None):
    """Flat walking: distance and steps dominate, survival is a small term.

    Used for bring-up on a robot that cannot yet walk at all (the G1 probe of
    2026-09-19), where there is no stair and no transfer to score. Forward
    distance is weighted directly because "took steps in place" and "walked"
    are otherwise indistinguishable -- the G1 failures reached step 4 having
    moved 15 mm.
    """
    violations = len(stairs.get("joint_limits", {}).get("violations", []))
    return (
        5000.0 * bool(stairs.get("completed"))
        + 100.0 * stairs.get("steps_entered", 0)
        + 500.0 * stairs.get("last_x", 0.0)
        + 10.0 * stairs.get("last_t", 0.0)
        - 10.0 * violations
        + _substep_term(report)
    )


def _substep_term(report):
    """Reward passing the 0.5 ms physics-rate limit audit, penalise its excess.

    The CSV-sample violation count above is a 5 ms view of a 0.5 ms plant, so a
    recipe can score clean there and still drive a joint past its URDF stop
    between samples. Measured 2026-09-21 on one binary: `JLIM=0` survives 16/16
    with no sampled violation but passes the physics-rate audit only 6/16, and
    `HIP_ROLL_SEED=0.06` scored 14/16 sampled against 2/16 audited. Weighting
    the audit at the same order as `completed` keeps the search from buying
    sampled cleanliness with real excursions.
    """
    if not report:
        return 0.0
    if report.get("substep_passed") is None:
        return 0.0
    worst = report.get("substep_worst_excess", 0.0) or 0.0
    return 2000.0 * bool(report.get("substep_passed")) - 500.0 * min(worst, 1.0)


def score(stairs, transfer_completed, report=None):
    """Progress dominates survival; illegality is a penalty, not a veto.

    There is no survival bonus on purpose. Survival enters only through
    `last_t` at 10/s, so a 25 s test is worth 250 -- less than three steps.
    Give survival a bonus of its own and the original defect comes straight
    back: standing on one leg for the whole test outscores climbing the step.

    The violation penalty is deliberately smaller than one step. In a run that
    falls, the audit is dominated by the impact rather than by the control:
    fallen runs on 2026-09-19 showed velocity excesses of 20-41 rad/s (the
    slam), while the same recipe truncated before its fall showed only two
    position excesses of 1.2 and 0.4 deg. A large penalty therefore scores how
    violently a run ended, not how well it was controlled, and at -30 it really
    did rank `fell at 16.4 s with 4 violations` above `fell at 18.4 s with 12`.

    The weights order the regimes measured on 2026-09-19 the way the task
    values them:

        stood 25 s, 13 steps, transfer never completed, 0 violations -> 1950
        reached 15 steps, transfer completed, fell at 16.4 s,  4 viol -> 2624
        reached 16 steps, transfer completed, fell at 18.4 s, 12 viol -> 2664
    """
    violations = len(stairs.get("joint_limits", {}).get("violations", []))
    return (
        5000.0 * bool(stairs.get("completed"))
        + 600.0 * bool(transfer_completed)
        + 400.0 * len(stairs.get("loaded_treads", []))
        + 100.0 * stairs.get("steps_entered", 0)
        + 200.0 * stairs.get("landing_hold_s", 0.0)
        + 10.0 * stairs.get("last_t", 0.0)
        - 10.0 * violations
        + DISTANCE_WEIGHT * stairs.get("last_x", 0.0)
        + SURVIVE_BONUS * bool(stairs.get("survived"))
        * bool(stairs.get("last_x", 0.0) >= SURVIVE_MIN_X)
    )


# Flat bonus for reaching the end of the test upright. DEFAULT 0, which is the
# objective every result before 2026-09-22 was searched against.
#
# The docstring above explains why there was none: on 2026-09-19, when the
# machine could not climb, a survival bonus made "stand on one leg for the
# whole test" outscore climbing the step. That guard is now obsolete -- the
# machine climbs 44-47 of 48 and takes 16-18 steps, so the progress terms
# dominate any standing degenerate by a wide margin:
#
#     stand 30 s:   0 treads + ~0 steps + 300 (last_t)            = ~300
#     climb+stand:  400 + 1700 + 300 + 600 (transfer)             = ~3000
#
# and at 2000 the stander reaches ~2300 against ~5000. Leaving it at zero has
# a measured cost: CEM17 (2026-09-22, 24 axes, 32-member ensembles) selected
#     17 steps, transfer completed, FELL at 27.86 s  ->  2938.6
# as its best candidate, and that recipe verified at 5/48 survival on its own
# training riser while the recipe it started from scored 31/48. Raising the
# ensemble made the search optimise this objective MORE reliably, and this
# objective does not ask the machine to stay up.
SURVIVE_BONUS = float(os.environ.get("CEM_SURVIVE_BONUS", "0"))
# The survival bonus is paid only PAST this x. DEFAULT 0 = unconditional.
#
# Measured 2026-09-22: with the bonus unconditional and `--quantile 0.25`,
# CEM19 selected a recipe that survives 42-47 of 48 and climbs 0 of 48. It
# walks its 16 approach steps, puts ONE foot on tread 1 (`loaded_treads: [1]`)
# and stands at x = 2.046 for the remaining 25 s:
#
#     2000 (survive) + 400 (tread) + 1600 (16 steps) + 300 (last_t) = 4300
#
# against a full climb that survives only ~60% of the time, whose 25th
# percentile member is therefore a faller worth ~2400. The search was right;
# the objective paid for timidity. Gating the bonus on getting PAST the stair
# removes that option, and `CEM_DISTANCE_WEIGHT` (score_flat already uses
# 500/m) keeps standing still from paying at all.
SURVIVE_MIN_X = float(os.environ.get("CEM_SURVIVE_MIN_X", "0"))
DISTANCE_WEIGHT = float(os.environ.get("CEM_DISTANCE_WEIGHT", "0"))
# Risers the ensemble cycles over, e.g. "0.020,0.040,0.060,0.080". Empty
# (the default) keeps every member on the base recipe's own STAIRS string.
RISERS = [r.strip() for r in os.environ.get("CEM_RISERS", "").split(",") if r.strip()]
# Make an ensemble member a SITUATION rather than a rounding perturbation.
#
# The KD_COM*(1+k*1e-9) member below samples the chaotic spread around ONE
# scenario: one initial posture, one friction, one stair position. A search
# scored on it buys a machine for that scenario. `log/claude_g1rand_20260924`
# draws what the verification actually varies, and 2026-09-24 the two gave
# OPPOSITE verdicts on the same candidate. Setting CEM_SITUATIONS=1 draws the
# member the same way the verification does, keyed on (CEM_SIT_SEED, member),
# so training and testing sample the same population.
#
# Off by default: every recipe before 2026-09-25 was selected under the old
# ensemble and its scores are only comparable to itself.
SITUATIONS = os.environ.get("CEM_SITUATIONS", "") not in ("", "0")
SIT_SEED = os.environ.get("CEM_SIT_SEED", "20260925")
SIT_JITTER = os.environ.get("CEM_SIT_JITTER", "0.01")
SIT_MU = (float(os.environ.get("CEM_SIT_MU_LO", "0.60")),
          float(os.environ.get("CEM_SIT_MU_HI", "0.80")))
SIT_DX = float(os.environ.get("CEM_SIT_DX", "0.02"))


# Spend part of the ensemble on FLAT PUSHES instead of stairs.
#
# A search scored only on stairs buys only stairs. Measured 2026-09-25: the
# free-arm recipe this driver found scored 145 -> 168 on the randomised stairs
# and 143 -> 174 on the holdout seed, while flat pushes went 214 -> 168
# (z=-5.58), the loss concentrated in the LATERAL conditions. It had cut every
# lateral reserve -- KD_COM 60->41, K_DCM 2.79->2.26, COP_FRAC 0.95->0.92 --
# because nothing in the objective asked for one. That is the objective's
# fault, not the search's.
#
# `CEM_PUSH_CONDS` ("impulse@deg,...") plus `CEM_PUSH_MEMBERS` (how many of
# the ensemble are push members) makes the last N members flat runs with a
# push, cycling the conditions. Both empty/0 by default.
PUSH_CONDS = [c.strip() for c in os.environ.get("CEM_PUSH_CONDS", "").split(",") if c.strip()]
PUSH_MEMBERS = int(os.environ.get("CEM_PUSH_MEMBERS", "0"))


def situation(member):
    """The same draw as log/claude_g1rand_20260924/rand.py, same key form."""
    r = random.Random(f"{SIT_SEED}:{member}")
    return {"mu": round(r.uniform(*SIT_MU), 4),
            "dx": round(r.uniform(-SIT_DX, SIT_DX), 4),
            "step": r.choice([7, 8, 9]),
            "at": round(r.uniform(0.25, 0.75), 3)}


def push_member(member, ensemble):
    """Which push condition member `member` runs, or None for a stair member."""
    if not PUSH_CONDS or PUSH_MEMBERS <= 0 or member is None:
        return None
    first = max(ensemble - PUSH_MEMBERS, 0)
    if member < first:
        return None
    return PUSH_CONDS[(member - first) % len(PUSH_CONDS)]


def sample(mean, sigma, rng):
    out = {}
    for name, (lo, hi, _, _) in SPACE.items():
        value = rng.gauss(mean[name], sigma[name])
        out[name] = min(hi, max(lo, value))
    return out


def evaluate(binary, base, params, out, timeout, kind='up-one', scorer=None,
             perturb=0, ensemble=1):
    """Run one candidate and return (score, summary). Never raises."""
    out.mkdir(parents=True, exist_ok=True)
    controls = dict(base)
    controls.update({k: f"{v:.6g}" for k, v in params.items()})
    # Ensemble member index, applied AFTER the candidate's own parameters.
    #
    # It used to be applied to `base` in the caller, which silently did nothing
    # whenever KD_COM was a search axis -- the line above overwrote it with the
    # candidate's value, identically for every member. Measured 2026-09-22:
    # CEM16 (ensemble 8), CEM17 and CEM18 (ensemble 32) each produced members
    # whose scores were IDENTICAL to the last digit, i.e. every "ensemble" was
    # N copies of one run. 1280 / 2304 / 2304 runs bought 160 / 72 / 72
    # distinct evaluations, and the search selected single draws throughout.
    if SITUATIONS and perturb is not None:
        sit = situation(perturb)
        controls["INIT_JITTER"] = SIT_JITTER
        controls["INIT_SEED"] = str(perturb + 1)
        controls["MU_GROUND"] = str(sit["mu"])
    elif perturb:
        nominal = float(controls.get("KD_COM", 80.0))
        controls["KD_COM"] = repr(nominal * (1.0 + perturb * 1e-9))
    # Spread the ensemble over RISERS as well as over perturbations.
    #
    # Measured 2026-09-22: CEM20, whose 16 members all ran the 60 mm riser,
    # reached 35/48 THERE (the best 60 mm figure of the day) and 0/48, 3/48,
    # 4/48 at 20, 40 and 80 mm. A single training condition buys a machine for
    # that condition. `CEM_RISERS` gives member mi the riser
    # RISERS[mi % len(RISERS)], so a candidate's quantile score is taken over
    # heights as well as draws and a height-specific recipe cannot win.
    cond = push_member(perturb, ensemble)
    if cond is not None:
        # A flat run with a push: the stair recipe with a zero riser, exactly
        # as log/claude_g1rand_20260924/rand.py builds its push members, so a
        # candidate is scored on the same population the verification uses.
        impulse, deg = cond.split("@")
        sit = situation(perturb)
        spec = str(controls.get("STAIRS", "1,0.020,0.211,2.00")).split(",")
        while len(spec) < 4:
            spec.append("2.00")
        spec[1] = "0.000"
        spec[3] = f"{2.00 + sit['dx']:.4f}"
        controls["STAIRS"] = ",".join(spec)
        controls["PUSH_IMPULSE"] = impulse
        controls["PUSH_DEG"] = deg
        controls["PUSH_STEP"] = str(sit["step"])
        controls["PUSH_AT"] = str(sit["at"])
        controls["PUSH_SUPPORT"] = os.environ.get("CEM_PUSH_SUPPORT", "ss")
    elif RISERS and perturb is not None:
        spec = controls.get("STAIRS")
        if spec:
            parts = str(spec).split(",")
            parts[1] = RISERS[perturb % len(RISERS)]
            if SITUATIONS and len(parts) > 3:
                parts[3] = f"{2.00 + situation(perturb)['dx']:.4f}"
            controls["STAIRS"] = ",".join(parts)
    controls["TRAJ_CSV"] = str(out / "trajectory.csv")
    # Physics-rate limit audit, on by default.
    #
    # The trajectory CSV is written at the WBC tick (5 ms); the plant runs at
    # 0.5 ms. Judging joint limits from the CSV therefore misses every excursion
    # that opens and closes between samples. Measured 2026-09-21 on a common
    # binary: a condition that survives 16/16 with no CSV-sample violation
    # passes the 0.5 ms audit only 6/16, and a stance-width setting scored
    # 14/16 on CSV samples collapsed to 2/16 under it. A search scored on the
    # loose criterion converges on recipes that violate the strict one, so the
    # audit is part of the objective rather than a later check.
    controls.setdefault("SUBSTEP_LIMIT_AUDIT_JSON", str(out / "substep.json"))
    controls.pop("TRANSFER_DIAG_CSV", None)
    (out / "controls.json").write_text(json.dumps(controls, indent=1) + "\n")
    env = {k: os.environ[k] for k in ("PATH", "HOME", "LD_LIBRARY_PATH", "LANG") if k in os.environ}
    env.update(controls)
    started = time.monotonic()
    try:
        with (out / "run.log").open("w") as stream:
            run = subprocess.run([str(binary)], cwd=REPO, env=env, stdout=stream,
                                 stderr=subprocess.STDOUT, timeout=timeout)
        report = {"returncode": run.returncode}
    except subprocess.TimeoutExpired:
        report = {"error": "wall_timeout"}
    report["wall_s"] = round(time.monotonic() - started, 1)
    log = (out / "run.log").read_text()
    match = TRANSFER_LINE.search(log)
    report["transfer_started"] = int(match.group(1)) if match else 0
    report["transfer_completed"] = int(match.group(2)) if match else 0
    report["transfer_timeouts"] = int(match.group(3)) if match else 0
    audit = out / "substep.json"
    if audit.exists():
        try:
            a = json.loads(audit.read_text())
            worst = max((j.get("excess", {}).get(k, 0.0)
                         for j in a.get("joints", [])
                         for k in ("position_rad", "velocity_rad_s", "effort_nm")),
                        default=0.0)
            report["substep_passed"] = bool(a.get("passed"))
            report["substep_worst_excess"] = worst
        except (ValueError, KeyError, AttributeError):
            report["substep_passed"] = None
    traj = out / "trajectory.csv"
    if traj.exists():
        try:
            report["stairs"] = summarize(traj, log, controls, kind)
        except (ValueError, KeyError, AttributeError, IndexError) as error:
            report["summary_error"] = str(error)
    fn = scorer or score
    report["score"] = (fn(report.get("stairs", {}), report["transfer_completed"], report)
                       if "stairs" in report else -1e6)
    report["params"] = params
    (out / "summary.json").write_text(json.dumps(report, indent=1, allow_nan=False) + "\n")
    # The trajectory CSV is the bulk of the disk cost and is reproducible from
    # controls.json + the recorded binary hash; keep only the elite ones.
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--base", type=Path, required=True, help="controls.json to start from")
    parser.add_argument("--binary", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--gens", type=int, default=6)
    parser.add_argument("--pop", type=int, default=16)
    parser.add_argument("--elite", type=int, default=5)
    parser.add_argument("--jobs", type=int, default=8)
    parser.add_argument("--timeout", type=float, default=600)
    parser.add_argument("--seed", type=int, default=20260919)
    parser.add_argument("--set", action="append", default=[], metavar="KEY=VALUE",
                        help="fixed control applied to every candidate")
    parser.add_argument("--keep-trajectories", action="store_true")
    parser.add_argument("--quantile", type=float, default=0.5, metavar="Q",
                        help="which quantile of the ensemble members to score (default 0.5 = "
                             "median). Taking the best of many candidates on a noisy median "
                             "selects the lucky medians, so a LOWER quantile (0.25, or 0.0 for "
                             "the worst member) counteracts that selection bias at the cost of "
                             "being more conservative.")
    parser.add_argument("--aggregate", default="quantile", choices=["quantile", "mean"],
                        help="how a candidate's member scores become one score. 'quantile' "
                             "(default) takes --quantile of the sorted members. 'mean' averages "
                             "them: with a 0/1 survival bonus a quantile of a ~45%% population is "
                             "almost always a faller, so every candidate scores the same and the "
                             "search has no gradient (measured 2026-09-22, CEM19-21); the mean "
                             "turns the bonus into survival RATE x bonus.")
    parser.add_argument("--ensemble", type=int, default=1, metavar="N",
                        help="evaluate each candidate N times with an inert 1e-9 perturbation "
                             "and score the MEDIAN. 1 (the default) scores a single draw, which "
                             "for this system means scoring luck -- see "
                             "doc/from_claude/20260919_single_run_ab_is_invalid.md. "
                             "N=4 is still optimistic: taking the best of 24 candidates on a "
                             "4-sample median selects the lucky medians too. Measured on the G1 "
                             "run of 2026-09-20, a winner whose N=4 median completed the test "
                             "completed only 3/16 when re-measured. Prefer N>=8, or score a "
                             "lower quantile than the median.")
    parser.add_argument("--space-json", type=Path,
                        help='{"NAME": [lo, hi, mean, sigma]} replacing the built-in SPACE')
    parser.add_argument("--kind", default="up-one",
                        choices=["up-one", "down-one", "flat", "platform"])
    args = parser.parse_args()

    if args.space_json:
        SPACE.clear()
        SPACE.update({k: tuple(v) for k, v in json.loads(args.space_json.read_text()).items()})
    scorer = score_flat if args.kind == "flat" else score
    out = args.out.resolve()
    out.mkdir(parents=True, exist_ok=True)
    base = {k: str(v) for k, v in json.loads(args.base.read_text()).items()}
    for pair in args.set:
        key, value = pair.split("=", 1)
        base[key] = value
    binary = out / "kyo46rs_walk"
    if not binary.exists():
        shutil.copy2(args.binary.resolve(), binary)
    (out / "manifest.json").write_text(json.dumps({
        "schema": 1,
        "binary_sha256": sha(binary),
        "base_controls_sha256": sha(args.base),
        "git_head": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=REPO, text=True).strip(),
        "walk_example_sha256": sha(REPO / "examples/kyo46rs_walk.rs"),
        "space": {k: list(v) for k, v in SPACE.items()},
        "objective": scorer.__doc__,
        "kind": args.kind,
        "fixed_overrides": args.set,
        "ensemble": args.ensemble,
        "quantile": args.quantile,
        "aggregate": args.aggregate,
    }, indent=1) + "\n")

    state_path = out / "cem_state.json"
    if state_path.exists():
        state = json.loads(state_path.read_text())
    else:
        state = {"gen": 0,
                 "mean": {k: v[2] for k, v in SPACE.items()},
                 "sigma": {k: v[3] for k, v in SPACE.items()},
                 "best": None,
                 "history": []}
    rng = random.Random(args.seed + state["gen"])

    for _ in range(args.gens):
        gen = state["gen"]
        candidates = [sample(state["mean"], state["sigma"], rng) for _ in range(args.pop)]
        # One job per (candidate, ensemble member). A candidate's score is the
        # MEDIAN of its members: with --ensemble 1 the search picks the luckiest
        # draw rather than the best recipe, which is how both CEM runs of
        # 2026-09-19 came to optimise noise.
        jobs = [(ci, mi) for ci in range(args.pop) for mi in range(args.ensemble)]

        def run_one(job):
            ci, mi = job
            controls = dict(base)
            suffix = f"g{gen}_c{ci}" + (f"_m{mi}" if args.ensemble > 1 else "")
            return ci, evaluate(binary, controls, candidates[ci], out / suffix,
                                args.timeout, args.kind, scorer,
                                perturb=mi if args.ensemble > 1 else 0,
                                ensemble=args.ensemble)

        with ThreadPoolExecutor(max_workers=args.jobs) as pool:
            done = list(pool.map(run_one, jobs))
        by_candidate = {ci: [] for ci in range(args.pop)}
        for ci, report in done:
            by_candidate[ci].append(report)
        reports = []
        for ci in range(args.pop):
            members = by_candidate[ci]
            representative = dict(members[0])
            scores = sorted(m["score"] for m in members)
            idx = min(len(scores) - 1, max(0, int(round(args.quantile * (len(scores) - 1)))))
            representative["score"] = (
                sum(scores) / len(scores) if args.aggregate == "mean" else scores[idx]
            )
            representative["member_scores"] = sorted(round(m["score"], 1) for m in members)
            reports.append(representative)
        ranked = sorted(reports, key=lambda r: r["score"], reverse=True)
        elite = ranked[:args.elite]
        for name in SPACE:
            values = [r["params"][name] for r in elite]
            mean = sum(values) / len(values)
            var = sum((v - mean) ** 2 for v in values) / max(1, len(values) - 1)
            lo, hi, _, sigma0 = SPACE[name]
            # Floor the spread at 5% of the initial sigma so a dimension that
            # collapses early can still move in later generations.
            state["mean"][name] = mean
            state["sigma"][name] = max(math.sqrt(var), 0.05 * sigma0)
        if state["best"] is None or elite[0]["score"] > state["best"]["score"]:
            state["best"] = {"score": elite[0]["score"], "gen": gen,
                             "params": elite[0]["params"],
                             "stairs": elite[0].get("stairs"),
                             "transfer_completed": elite[0]["transfer_completed"]}
            best_dir = out / "BEST"
            shutil.rmtree(best_dir, ignore_errors=True)
            best_ci = reports.index(elite[0])
            src = out / (f"g{gen}_c{best_ci}" + ("_m0" if args.ensemble > 1 else ""))
            shutil.copytree(src, best_dir)
        state["history"].append({
            "gen": gen,
            "scores": [round(r["score"], 1) for r in ranked],
            "best_stairs": {k: elite[0].get("stairs", {}).get(k) for k in
                            ("survived", "completed", "steps_entered", "loaded_treads", "last_t")},
            "best_transfer_completed": elite[0]["transfer_completed"],
        })
        state["gen"] = gen + 1
        state_path.write_text(json.dumps(state, indent=1) + "\n")
        if not args.keep_trajectories:
            keep = {id(r) for r in elite}
            for r, cand in zip(reports, range(args.pop)):
                if id(r) in keep:
                    continue
                for mi in range(args.ensemble):
                    suffix = f"g{gen}_c{cand}" + (f"_m{mi}" if args.ensemble > 1 else "")
                    (out / suffix / "trajectory.csv").unlink(missing_ok=True)
        print(json.dumps(state["history"][-1], ensure_ascii=False), flush=True)

    print(json.dumps({"best": state["best"]}, ensure_ascii=False, indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
