#!/usr/bin/env python3
"""Re-shoot the forward/backward/left/right/turn demo clips, reproducibly.

The 2026-08-09 clips were made by hand and the commands are gone. Worse, they
were shot before `log_joints` carried the v6 shoulder roll, so the renderer --
which reads each URDF joint by NAME out of the CSV and defaults a missing one
to 0.0 (`q.get(jn, 0.0)`) -- drew both arms with roll pinned at zero for the
whole clip. The arm motion Sec.28 is about was not merely hard to see in those
videos; it was not in them.

    python3 scripts/kyo46rs_demo_video.py                 # all five clips
    python3 scripts/kyo46rs_demo_video.py --only left     # one
    python3 scripts/kyo46rs_demo_video.py --urdf <path> --tag noadd

`--urdf` shoots the same five clips against a different model, so a candidate
fix can be put next to the shipped one frame for frame.
"""
import argparse
import os
import subprocess
import sys

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BIN = f"{REPO}/target/release/examples/kyo46rs_walk"
RENDER = f"{REPO}/examples/render_com_squat_video.py"
MUJOCO_LIB = "/home/takara/.mujoco/mujoco-3.8.0/lib"

# The bench's COMMON, at the demo's shorter length (16 steps, ~12.8 s).
COMMON = {
    "T_SS": "0.35", "T_DS": "0.20", "STRIDE_RAMP": "6",
    "ARM_PITCH": "0", "LIFT_H": "0.02", "PHASE_BY_CONTACT": "0",
    "N_STEPS": "16", "T": "12.8",
}
# Lateral is shot at 0.018, not 0.036: Sec.28 measured 0.036 falling on v6, and
# a demo clip is not the place to argue about it.
CLIPS = [
    ("forward",  {"VX": "0.055"}),
    ("backward", {"VX": "-0.055"}),
    ("left",     {"VY": "0.018"}),
    ("right",    {"VY": "-0.018"}),
    ("turn",     {"WZ": "0.20"}),
]


def shoot(name, cmd_env, urdf, tag, keep_going):
    suffix = f"_{tag}" if tag else ""
    csv = f"{REPO}/csv/demo_{name}{suffix}.csv"
    mp4 = f"{REPO}/videos/demo_{name}{suffix}.mp4"
    env = dict(os.environ)
    env["LD_LIBRARY_PATH"] = MUJOCO_LIB + ":" + env.get("LD_LIBRARY_PATH", "")
    env.update(COMMON)
    env.update(cmd_env)
    env["TRAJ_CSV"] = csv
    if urdf:
        env["URDF"] = urdf
    p = subprocess.run([BIN], env=env, capture_output=True, text=True, timeout=1800)
    txt = p.stdout + p.stderr
    verdict = "SURVIVED" if "verdict: SURVIVED" in txt else "FELL"
    selfcol = next((l.strip() for l in txt.splitlines()
                    if "self-collision ticks:" in l), "")
    selfcol = selfcol.split("--")[0].strip()
    print(f"  {name:<9} {verdict:<9} {selfcol}")
    if verdict == "FELL" and not keep_going:
        print(f"  {name}: FELL -- not rendering. Pass --render-falls to keep it.")
        return None
    r = subprocess.run([sys.executable, RENDER, csv, mp4],
                       env=env, capture_output=True, text=True, timeout=3600)
    if r.returncode != 0:
        print(r.stdout[-2000:] + r.stderr[-2000:])
        raise SystemExit(f"render failed for {name}")
    return mp4


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--only", help="one clip name")
    ap.add_argument("--urdf", help="model to shoot against (default: the profile's)")
    ap.add_argument("--tag", default="", help="suffix for the output names")
    ap.add_argument("--render-falls", action="store_true",
                    help="render a clip even if the run fell")
    ap.add_argument("--no-join", action="store_true")
    a = ap.parse_args()
    clips = [c for c in CLIPS if a.only is None or c[0] == a.only]
    os.makedirs(f"{REPO}/videos", exist_ok=True)
    made = [m for m in (shoot(n, e, a.urdf, a.tag, a.render_falls) for n, e in clips) if m]
    if not made:
        raise SystemExit("nothing rendered")
    for m in made:
        print(m)
    if a.no_join or len(made) < 2:
        return
    suffix = f"_{a.tag}" if a.tag else ""
    joined = f"{REPO}/videos/demo_fwd_back_left_right_turn{suffix}.mp4"
    lst = f"{REPO}/videos/.concat{suffix}.txt"
    with open(lst, "w") as fh:
        for m in made:
            fh.write(f"file '{m}'\n")
    subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-f", "concat",
                    "-safe", "0", "-i", lst, "-c", "copy", joined], check=True)
    os.remove(lst)
    print(joined)


if __name__ == "__main__":
    main()
