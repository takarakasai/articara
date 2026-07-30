#!/usr/bin/env python3
"""Render the kyo46rs centroidal squat run to an mp4.

Self-contained software renderer: parses the URDF, walks its kinematic
tree per logged frame, and rasterises the link primitives with numpy +
PIL. Deliberately does NOT use the `mujoco` Python bindings — they are
not installed here and fetching them would mean reaching the network.
Everything below runs against what is already on the machine, and the
physics has already happened anyway: this is a kinematic replay of the
trajectory CSV the Rust sim wrote.

Usage:
    python3 examples/render_com_squat_video.py <traj.csv> <out.mp4>
"""
import csv
import math
import os
import subprocess
import sys
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image, ImageDraw, ImageFont

URDF = "/home/takara/work/dp/humanoid/kyo46rs_description/urdf/kyo46rs.urdf"
W, H = int(os.environ.get("VID_W", 960)), 720
FPS = 50

# ── URDF ───────────────────────────────────────────────────────────────


def rpy_to_mat(r, p, y):
    cr, sr, cp, sp, cy, sy = (
        math.cos(r), math.sin(r), math.cos(p),
        math.sin(p), math.cos(y), math.sin(y),
    )
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def parse_urdf(path):
    root = ET.parse(path).getroot()
    links, joints = {}, {}
    for lk in root.findall("link"):
        vis = []
        for v in lk.findall("visual"):
            o = v.find("origin")
            xyz = [float(x) for x in (o.get("xyz", "0 0 0").split() if o is not None else [0, 0, 0])]
            rpy = [float(x) for x in (o.get("rpy", "0 0 0").split() if o is not None else [0, 0, 0])]
            g = v.find("geometry")
            box, sph, cyl = g.find("box"), g.find("sphere"), g.find("cylinder")
            mat = v.find("material")
            name = mat.get("name") if mat is not None else ""
            if box is not None:
                vis.append(("box", np.array(xyz), rpy_to_mat(*rpy),
                            np.array([float(x) for x in box.get("size").split()]), name))
            elif sph is not None:
                vis.append(("sphere", np.array(xyz), rpy_to_mat(*rpy),
                            float(sph.get("radius")), name))
            elif cyl is not None:
                vis.append(("cyl", np.array(xyz), rpy_to_mat(*rpy),
                            (float(cyl.get("radius")), float(cyl.get("length"))), name))
        links[lk.get("name")] = vis
    for j in root.findall("joint"):
        o = j.find("origin")
        xyz = [float(x) for x in (o.get("xyz", "0 0 0").split() if o is not None else [0, 0, 0])]
        rpy = [float(x) for x in (o.get("rpy", "0 0 0").split() if o is not None else [0, 0, 0])]
        ax = j.find("axis")
        axis = [float(x) for x in (ax.get("xyz").split() if ax is not None else [1, 0, 0])]
        joints[j.get("name")] = dict(
            parent=j.find("parent").get("link"),
            child=j.find("child").get("link"),
            xyz=np.array(xyz),
            rot=rpy_to_mat(*rpy),
            axis=np.array(axis, dtype=float),
            type=j.get("type"),
        )
    return links, joints


def axis_angle(axis, ang):
    a = axis / (np.linalg.norm(axis) or 1.0)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(ang) * K + (1 - math.cos(ang)) * (K @ K)


def forward_kinematics(links, joints, root_link, base_p, base_R, q):
    """link name -> (world position, world rotation)."""
    pose = {root_link: (base_p, base_R)}
    children = {}
    for name, j in joints.items():
        children.setdefault(j["parent"], []).append(name)
    stack = [root_link]
    while stack:
        parent = stack.pop()
        pp, pR = pose[parent]
        for jn in children.get(parent, []):
            j = joints[jn]
            R = pR @ j["rot"]
            p = pp + pR @ j["xyz"]
            if j["type"] in ("revolute", "continuous"):
                R = R @ axis_angle(j["axis"], q.get(jn, 0.0))
            pose[j["child"]] = (p, R)
            stack.append(j["child"])
    return pose


# ── renderer ───────────────────────────────────────────────────────────

TAU_COLS = [(232, 168, 92), (120, 190, 235), (150, 205, 140)]  # hip / knee / ankle
SOLE = (78, 168, 178)     # ground-contact plate
OUTPUT = (235, 115, 33)   # driven face of a joint
PALETTE = {
    "dark": (58, 62, 70),
    "grey": (118, 124, 136),      # actuator housings
    "output": OUTPUT,             # driven face of each joint
    "": (190, 195, 205),
}
BG_TOP = (24, 27, 34)
BG_BOT = (44, 48, 58)

_FD = "/usr/share/fonts/truetype/dejavu/"
F_TITLE = ImageFont.truetype(_FD + "DejaVuSans-Bold.ttf", 26)
F_BODY = ImageFont.truetype(_FD + "DejaVuSansMono.ttf", 19)
F_SMALL = ImageFont.truetype(_FD + "DejaVuSansMono.ttf", 15)


class Camera:
    def __init__(self, eye, target, up=(0, 0, 1), fov=42.0, y_shift=0.0):
        self.y_shift = y_shift
        self.eye = np.array(eye, float)
        f = np.array(target, float) - self.eye
        f /= np.linalg.norm(f)
        up = np.array(up, float)
        s = np.cross(f, up)
        s /= np.linalg.norm(s)
        u = np.cross(s, f)
        self.R = np.stack([s, u, -f])          # world -> camera
        self.fl = (H / 2) / math.tan(math.radians(fov) / 2)

    def project(self, pts):
        cam = (pts - self.eye) @ self.R.T
        z = -cam[:, 2]
        z = np.maximum(z, 1e-4)
        x = W / 2 + self.fl * cam[:, 0] / z
        y = H / 2 - self.y_shift - self.fl * cam[:, 1] / z
        return np.stack([x, y], 1), z


BOX_FACES = [
    (0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1),
    (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3),
]
BOX_CORNERS = np.array([[i, j, k] for i in (-1, 1) for j in (-1, 1) for k in (-1, 1)], float)


def shade(base, normal, light=np.array([0.4, 0.7, 0.6])):
    light = light / np.linalg.norm(light)
    d = 0.42 + 0.58 * max(0.0, float(normal @ light))
    return tuple(int(np.clip(c * d, 0, 255)) for c in base)


def gradient_bg():
    col = np.linspace(0, 1, H)[:, None]
    img = (np.array(BG_TOP)[None, None, :] * (1 - col[:, :, None])
           + np.array(BG_BOT)[None, None, :] * col[:, :, None])
    return Image.fromarray(np.repeat(img.astype(np.uint8), W, axis=1))


def draw_ground(draw, cam):
    """Faint grid so the squat depth is legible."""
    n, step = 12, 0.10
    segs = []
    for i in range(-n, n + 1):
        segs.append((np.array([[i * step, -n * step, 0], [i * step, n * step, 0]]), i == 0))
        segs.append((np.array([[-n * step, i * step, 0], [n * step, i * step, 0]]), i == 0))
    for pts, axis in segs:
        p, z = cam.project(pts)
        if (z < 0.05).any():
            continue
        draw.line([tuple(p[0]), tuple(p[1])],
                  fill=(86, 92, 104) if axis else (60, 65, 76), width=2 if axis else 1)


def sole_roll_deg(pose):
    """Roll of the LEFT (stance) sole. Reads zero when the foot is flat."""
    _, Rf = pose["left_foot_link"]
    return math.degrees(math.atan2(Rf[2][1], Rf[2][2]))


def render_frame(links, joints, pose, cam, hud):
    img = gradient_bg()
    draw = ImageDraw.Draw(img, "RGBA")
    draw_ground(draw, cam)

    faces = []
    for lname, vis in links.items():
        if lname not in pose:
            continue
        lp, lR = pose[lname]
        for prim in vis:
            kind, off, orot, size, mat = prim
            wR = lR @ orot
            wp = lp + lR @ off
            # Accent only the part that actually touches the floor: the
            # sole plate (material "dark"). The ankle-roll actuator block
            # stacked above it is ordinary structure.
            is_sole = lname.endswith("foot_link") and mat == "dark"
            base = SOLE if is_sole else PALETTE.get(mat, PALETTE[""])
            if kind == "box":
                corners = wp + (BOX_CORNERS * (size / 2)) @ wR.T
            elif kind == "sphere":
                corners = wp + (BOX_CORNERS * size * 0.78) @ wR.T
            elif kind == "cyl":
                r, l = size
                # ring of prisms around the axis reads as a cylinder and keeps
                # the painter's-algorithm face sort working unchanged
                n = 12
                th = np.linspace(0, 2 * np.pi, n, endpoint=False)
                ring = np.stack([r * np.cos(th), r * np.sin(th)], 1)
                pts = np.concatenate([
                    np.column_stack([ring, np.full(n, -l / 2)]),
                    np.column_stack([ring, np.full(n, +l / 2)]),
                ])
                corners = wp + pts @ wR.T
                p2, z = cam.project(corners)
                if (z < 0.05).any():
                    continue
                side = [(i, (i + 1) % n, n + (i + 1) % n, n + i) for i in range(n)]
                caps = [tuple(range(n)), tuple(range(n, 2 * n))]
                for f in side + caps:
                    quad = corners[list(f)]
                    nv_ = np.cross(quad[1] - quad[0], quad[2] - quad[0])
                    nn = np.linalg.norm(nv_)
                    if nn < 1e-12:
                        continue
                    nv_ /= nn
                    if nv_ @ (cam.eye - quad.mean(0)) < 0:
                        continue
                    faces.append((z[list(f)].mean(), [tuple(p2[i]) for i in f],
                                  shade(base, nv_)))
                continue
            else:
                r, l = size
                corners = wp + (BOX_CORNERS * np.array([r, r, l / 2])) @ wR.T
            p2, z = cam.project(corners)
            if (z < 0.05).any():
                continue
            for f in BOX_FACES:
                quad = corners[list(f)]
                n = np.cross(quad[1] - quad[0], quad[2] - quad[0])
                nn = np.linalg.norm(n)
                if nn < 1e-12:
                    continue
                n /= nn
                if n @ (cam.eye - quad.mean(0)) < 0:
                    continue  # backface
                faces.append((z[list(f)].mean(), [tuple(p2[i]) for i in f], shade(base, n)))

    for _, poly, col in sorted(faces, key=lambda t: -t[0]):
        draw.polygon(poly, fill=col, outline=(28, 30, 36))

    # ── HUD ────────────────────────────────────────────────────────────
    (t, com_z, ref_z, tilt, hist, taus, tau_names, tau_lims, tau_total,
     n_stance, degraded, sole_roll) = hud
    compact = os.environ.get("COMPACT")
    if compact:
        draw.rectangle([0, 0, W, 76], fill=(16, 18, 23, 214))
        draw.text((16, 8), os.environ.get("TITLE", ""), fill=(238, 242, 250), font=F_BODY)
        draw.text((16, 32), f"t {t:5.2f}s   tilt {math.degrees(tilt):5.1f} deg",
                  fill=(176, 184, 198), font=F_SMALL)
        col = (214, 97, 90) if degraded else (126, 200, 140)
        draw.text((16, 52), "QP infeasible" if degraded else "QP solving",
                  fill=col, font=F_SMALL)
        draw.text((W - 150, 52), "1 foot" if n_stance == 1 else "2 feet",
                  fill=(226, 140, 92) if n_stance == 1 else (126, 200, 140), font=F_SMALL)
        sr = hud[-1]
        draw.text((W - 150, 32), f"sole {sr:+5.1f} deg",
                  fill=(214, 97, 90) if abs(sr) > 3.0 else (150, 158, 172), font=F_SMALL)
        return img
    draw.rectangle([0, 0, W, 84], fill=(16, 18, 23, 210))
    draw.text((22, 10), os.environ.get("TITLE", "kyo46rs  /  centroidal WBC squat"),
              fill=(238, 242, 250), font=F_TITLE)
    draw.text((22, 48), f"t {t:5.2f}s", fill=(176, 184, 198), font=F_BODY)
    draw.text((150, 48), f"CoM z {com_z:.3f} m  (ref {ref_z:.3f})",
              fill=(232, 168, 92), font=F_BODY)
    draw.text((468, 48), f"tilt {math.degrees(tilt):4.1f} deg",
              fill=(176, 184, 198), font=F_BODY)
    if degraded:
        draw.text((W - 262, 14), "QP: INFEASIBLE", fill=(214, 97, 90), font=F_BODY)
    else:
        draw.text((W - 262, 14), "QP: solving", fill=(126, 200, 140), font=F_BODY)
    if n_stance == 1:
        draw.text((W - 262, 48), "contact: LEFT foot only", fill=(226, 140, 92), font=F_BODY)
    else:
        draw.text((W - 262, 48), "contact: both feet", fill=(126, 200, 140), font=F_BODY)


    # colour key, upper right
    kx, ky = W - 244, 96
    for i, (col, lab) in enumerate([
        (PALETTE["grey"], "actuator housing"),
        (OUTPUT, "output side"),
        (SOLE, "sole / contact"),
    ]):
        y = ky + i * 20
        draw.rectangle([kx, y, kx + 12, y + 11], fill=col)
        draw.text((kx + 20, y - 3), lab, fill=(150, 158, 172), font=F_SMALL)
    draw.text((kx, ky + 64), "EL05 46x44 mm, true size",
              fill=(108, 116, 130), font=F_SMALL)
    draw.text((kx, ky + 82), "knee / hip_pitch: dual",
              fill=(108, 116, 130), font=F_SMALL)

    # ── live pitch-joint torque strip, full width along the bottom ─────
    px, py, pw, ph = 22, H - 140, W - 62, 92
    draw.rectangle([px - 10, py - 30, px + pw + 10, py + ph + 26], fill=(16, 18, 23, 214))
    draw.text((px, py - 26), "sagittal joint torque, left leg",
              fill=(198, 206, 220), font=F_SMALL)

    span = float(os.environ.get("TAU_SPAN", 2.5))
    ymid = py + ph / 2
    def ty(v):
        return ymid - (ph / 2) * max(-1.0, min(1.0, v / span))

    # zero line + gridlines every 1 N*m
    for g in (-2.0, -1.0, 1.0, 2.0):
        draw.line([(px, ty(g)), (px + pw, ty(g))], fill=(46, 51, 62), width=1)
        draw.text((px + pw + 4, ty(g) - 8), f"{g:+.0f}", fill=(96, 104, 118), font=F_SMALL)
    draw.line([(px, ymid), (px + pw, ymid)], fill=(78, 86, 100), width=1)
    draw.text((px + pw + 4, ymid - 8), " 0", fill=(120, 128, 142), font=F_SMALL)

    n_pts = len(taus[0]) if taus and taus[0] else 0
    if n_pts > 1:
        # x maps the WHOLE run with a playhead at "now", so the trace grows
        # left-to-right in step with the video rather than scrolling.
        for series, col in zip(taus, TAU_COLS):
            pts = [(px + pw * i / (tau_total - 1), ty(v)) for i, v in enumerate(series)]
            draw.line(pts, fill=col, width=2)
        head = px + pw * (n_pts - 1) / (tau_total - 1)
        draw.line([(head, py), (head, py + ph)], fill=(226, 232, 244), width=1)

    # per-joint readout
    for i, (nm, lim) in enumerate(zip(tau_names, tau_lims)):
        cur = taus[i][-1] if taus[i] else 0.0
        x = px + i * (pw // 3)
        draw.rectangle([x, py + ph + 8, x + 12, py + ph + 19], fill=TAU_COLS[i])
        draw.text((x + 20, py + ph + 5),
                  f"{nm}  {cur:+5.2f} N*m   lim {lim:.0f}",
                  fill=(168, 176, 190), font=F_SMALL)

    # CoM-height trace, small, upper left
    if len(hist) > 2:
        x0, y0, ww, hh = 22, 122, 250, 62
        draw.rectangle([x0 - 10, y0 - 26, x0 + ww + 10, y0 + hh + 10], fill=(16, 18, 23, 200))
        lo, hi = 0.24, 0.33
        def sc(i, v):
            return (x0 + ww * i / max(1, len(hist) - 1),
                    y0 + hh * (1 - (v - lo) / (hi - lo)))
        draw.line([sc(i, r) for i, (_, r) in enumerate(hist)], fill=(110, 120, 140), width=1)
        draw.line([sc(i, c) for i, (c, _) in enumerate(hist)], fill=(232, 168, 92), width=2)
        draw.text((x0, y0 - 22), "CoM height vs commanded", fill=(150, 158, 172), font=F_SMALL)
    return img


def main():
    csv_path = sys.argv[1]
    out_mp4 = sys.argv[2]
    frame_dir = os.path.join(os.path.dirname(out_mp4), "com_squat_frames")
    os.makedirs(frame_dir, exist_ok=True)
    for f in os.listdir(frame_dir):
        os.remove(os.path.join(frame_dir, f))

    links, joints = parse_urdf(URDF)
    with open(csv_path) as f:
        rows = list(csv.DictReader(f))
    print(f"{len(rows)} logged ticks")

    jnames = [k for k in rows[0] if k.endswith("_joint")]
    # log is 200 Hz; decimate to FPS
    stride = max(1, int(round((1.0 / 200) / (1.0 / FPS) ** -1 * 1)) )
    dt = float(rows[1]["t"]) - float(rows[0]["t"])
    stride = max(1, int(round((1.0 / FPS) / dt)))
    sel = rows[::stride]
    print(f"dt={dt:.4f}s -> stride {stride} -> {len(sel)} frames @ {FPS}fps")

    TAU_JOINTS = [("left_hip_pitch_joint", "hip_pitch"),
                  ("left_knee_joint", "knee"),
                  ("left_ankle_pitch_joint", "ankle_pitch")]
    tau_lims = [float(sel[0]["lim_" + c]) for c, _ in TAU_JOINTS]
    tau_names = [lbl for _, lbl in TAU_JOINTS]
    tau_hist = [[] for _ in TAU_JOINTS]

    if os.environ.get("ANKLE_CLOSEUP"):
        cam = Camera(eye=(0.30, -0.42, 0.16), target=(0.01, 0.0, 0.06), fov=34, y_shift=60)
    else:
        cam = (Camera(eye=(1.30, -1.55, 0.62), target=(0.02, 0.0, 0.28), fov=30, y_shift=-10)
               if os.environ.get("COMPACT")
               else Camera(eye=(1.15, -1.38, 0.62), target=(0.02, 0.0, 0.30), fov=33, y_shift=24))
    hist = []
    for i, r in enumerate(sel):
        q = {n: float(r[n]) for n in jnames}
        bp = np.array([float(r["x"]), float(r["y"]), float(r["z"])])
        qw, qx, qy, qz = (float(r["qw"]), float(r["qx"]), float(r["qy"]), float(r["qz"]))
        n = math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz) or 1.0
        qw, qx, qy, qz = qw / n, qx / n, qy / n, qz / n
        bR = np.array([
            [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
            [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
            [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
        ])
        pose = forward_kinematics(links, joints, "torso", bp, bR, q)
        hist.append((float(r["com_z"]), float(r["com_ref_z"])))
        for k, (col, _) in enumerate(TAU_JOINTS):
            tau_hist[k].append(float(r["tau_" + col]))
        img = render_frame(links, joints, pose, cam,
                           (float(r["t"]), float(r["com_z"]), float(r["com_ref_z"]),
                            float(r["tilt"]), hist[-260:],
                            tau_hist, tau_names, tau_lims, len(sel),
                            int(r.get("n_stance", 2)),
                            int(r.get("n_stance", 2)) == 1,
                            sole_roll_deg(pose)))
        img.save(os.path.join(frame_dir, f"f{i:05d}.png"))
        if i % 50 == 0:
            print(f"  frame {i}/{len(sel)}")

    subprocess.run([
        "ffmpeg", "-y", "-loglevel", "error", "-framerate", str(FPS),
        "-i", os.path.join(frame_dir, "f%05d.png"),
        "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "20", out_mp4,
    ], check=True)
    print("wrote", out_mp4)


if __name__ == "__main__":
    main()
