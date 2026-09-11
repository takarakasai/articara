#!/usr/bin/env python3
"""Render the ChickenHead demo video from the generator's CSV.

Draws two side-by-side "chickens" driven by the SAME trunk-pitch disturbance
(from `examples/chicken_head_demo.rs`): the left one runs ChickenHead ON (head
held level in the world), the right one OFF (head rides the trunk). A trace
below plots each head's world pitch over time.

Usage:
    python3 scripts/chicken_head_demo.py \
        --csv /tmp/chicken_head_demo.csv \
        --out tests/media/chicken_head_demo.mp4
"""
import argparse
import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, FFMpegWriter
from matplotlib.patches import Ellipse, Polygon, Circle

GREEN = "#1f9d55"
GREY = "#9aa0a6"
RED = "#d93838"
INK = "#222222"


def rot(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s], [s, c]])


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--csv", default="/tmp/chicken_head_demo.csv")
    p.add_argument("--out", default="tests/media/chicken_head_demo.mp4")
    p.add_argument("--fps", type=int, default=30)
    return p.parse_args()


def load(csv):
    d = np.genfromtxt(csv, delimiter=",", names=True)
    return d


def chicken_artists(ax, accent):
    """Create the patch/line artists for one chicken; return an updater."""
    ground = ax.axhline(0.0, color="#c9a06a", lw=3, zorder=0)
    ax.fill_between([-1, 1], -0.4, 0.0, color="#efe0c8", zorder=-1)

    body = Ellipse((0, 0), 0.52, 0.34, angle=0, fc=accent, ec=INK, lw=2, zorder=3)
    ax.add_patch(body)
    tail = Polygon([[0, 0]], closed=True, fc=accent, ec=INK, lw=1.5, zorder=2)
    ax.add_patch(tail)
    (leg1,) = ax.plot([], [], color="#e0912f", lw=4, solid_capstyle="round", zorder=2)
    (leg2,) = ax.plot([], [], color="#e0912f", lw=4, solid_capstyle="round", zorder=2)
    (neck,) = ax.plot([], [], color=accent, lw=9, solid_capstyle="round", zorder=4)
    head = Circle((0, 0), 0.11, fc=accent, ec=INK, lw=2, zorder=6)
    ax.add_patch(head)
    comb = Polygon([[0, 0]], closed=True, fc=RED, ec=INK, lw=1, zorder=5)
    ax.add_patch(comb)
    beak = Polygon([[0, 0]], closed=True, fc="#f2a900", ec=INK, lw=1, zorder=6)
    ax.add_patch(beak)
    wattle = Polygon([[0, 0]], closed=True, fc=RED, ec=INK, lw=1, zorder=5)
    ax.add_patch(wattle)
    eye = Circle((0, 0), 0.018, fc=INK, zorder=7)
    ax.add_patch(eye)

    H = 0.52  # body-centre height

    def update(trunk_pitch, head_world):
        C = np.array([0.0, H])
        Rb = rot(trunk_pitch)
        body.center = (C[0], C[1])
        body.angle = np.degrees(trunk_pitch)

        # Tail at the rear-top of the body.
        tbase = C + Rb @ np.array([-0.26, 0.02])
        t1 = C + Rb @ np.array([-0.46, 0.18])
        t2 = C + Rb @ np.array([-0.44, -0.05])
        tail.set_xy([tbase, t1, t2])

        # Legs drop from the two hip points straight to the ground.
        for leg, dx in ((leg1, -0.13), (leg2, 0.12)):
            hip = C + Rb @ np.array([dx, -0.14])
            foot_x = hip[0]
            leg.set_data([hip[0], foot_x], [hip[1], 0.0])

        # Neck base at the front-top of the body; head oriented by head_world.
        nbase = C + Rb @ np.array([0.24, 0.10])
        Rh = rot(head_world)
        hcen = nbase + Rh @ np.array([0.14, 0.14])
        neck.set_data([nbase[0], hcen[0]], [nbase[1], hcen[1]])
        head.center = (hcen[0], hcen[1])

        # Comb (crest) on top, beak forward, wattle below, eye — all in the
        # head's own frame so ON stays upright while OFF tilts with the body.
        comb.set_xy([
            hcen + Rh @ np.array([-0.04, 0.10]),
            hcen + Rh @ np.array([0.00, 0.17]),
            hcen + Rh @ np.array([0.04, 0.10]),
            hcen + Rh @ np.array([0.08, 0.15]),
            hcen + Rh @ np.array([0.09, 0.09]),
        ])
        beak.set_xy([
            hcen + Rh @ np.array([0.10, 0.02]),
            hcen + Rh @ np.array([0.20, -0.01]),
            hcen + Rh @ np.array([0.10, -0.04]),
        ])
        wattle.set_xy([
            hcen + Rh @ np.array([0.10, -0.04]),
            hcen + Rh @ np.array([0.11, -0.12]),
            hcen + Rh @ np.array([0.06, -0.06]),
        ])
        eye.center = tuple(hcen + Rh @ np.array([0.03, 0.04]))

    return update


def main():
    args = parse_args()
    d = load(args.csv)
    t = d["t"]
    step = max(1, int(round((1.0 / args.fps) / (t[1] - t[0]))))
    idx = np.arange(0, len(t), step)

    fig = plt.figure(figsize=(10, 7.2), dpi=120)
    fig.patch.set_facecolor("white")
    gs = fig.add_gridspec(2, 2, height_ratios=[2.4, 1.0], hspace=0.28, wspace=0.12)
    ax_on = fig.add_subplot(gs[0, 0])
    ax_off = fig.add_subplot(gs[0, 1])
    ax_tr = fig.add_subplot(gs[1, :])

    for ax in (ax_on, ax_off):
        ax.set_xlim(-0.85, 0.95)
        ax.set_ylim(-0.15, 1.15)
        ax.set_aspect("equal")
        ax.axis("off")
    ax_on.set_title("ChickenHead  ON", color=GREEN, fontsize=16, fontweight="bold")
    ax_off.set_title("ChickenHead  OFF", color=RED, fontsize=16, fontweight="bold")

    # A faint world-horizontal guide line through each head region.
    for ax in (ax_on, ax_off):
        ax.axhline(0.78, color="#4a90d9", lw=1, ls=(0, (4, 4)), alpha=0.5, zorder=1)

    up_on = chicken_artists(ax_on, GREEN)
    up_off = chicken_artists(ax_off, GREY)

    # Trace panel.
    on_deg = np.degrees(d["head_world_on"])
    off_deg = np.degrees(d["head_world_off"])
    trunk_deg = np.degrees(d["trunk_pitch"])
    ax_tr.plot(t, trunk_deg, color="#4a90d9", lw=1.2, alpha=0.55, label="trunk pitch")
    ax_tr.plot(t, off_deg, color=GREY, lw=2.0, label="head world pitch — OFF")
    ax_tr.plot(t, on_deg, color=GREEN, lw=2.4, label="head world pitch — ON")
    ax_tr.axhline(0.0, color=INK, lw=0.8, alpha=0.4)
    ax_tr.set_xlim(t[0], t[-1])
    ymax = max(20.0, np.max(np.abs(trunk_deg)) * 1.15)
    ax_tr.set_ylim(-ymax, ymax)
    ax_tr.set_xlabel("time (s)")
    ax_tr.set_ylabel("pitch (deg)")
    ax_tr.legend(loc="upper right", ncol=3, fontsize=9, framealpha=0.9)
    ax_tr.grid(True, alpha=0.25)
    cursor = ax_tr.axvline(t[0], color=RED, lw=1.5)

    caption = fig.text(
        0.5, 0.015,
        "namiashi arm_pitch_joint held level in the world by the real "
        "articara ChickenHead controller  ·  same trunk disturbance both sides",
        ha="center", fontsize=9, color="#555555",
    )

    def frame(i):
        k = idx[i]
        up_on(d["trunk_pitch"][k], d["head_world_on"][k])
        up_off(d["trunk_pitch"][k], d["head_world_off"][k])
        cursor.set_xdata([t[k], t[k]])
        return ()

    anim = FuncAnimation(fig, frame, frames=len(idx), interval=1000 / args.fps, blit=False)
    writer = FFMpegWriter(fps=args.fps, bitrate=2400, metadata={"title": "ChickenHead demo"})
    import os
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    anim.save(args.out, writer=writer)
    print(f"wrote {args.out}  ({len(idx)} frames @ {args.fps} fps)")


if __name__ == "__main__":
    main()
