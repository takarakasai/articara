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

# Which machine. The replay is kinematic -- it re-walks the URDF per logged
# frame -- so it needs the same description the sim ran, not a copy of the
# controller's assumptions.
ROBOT = os.environ.get("ROBOT", "kyo46rs")
# Half the lateral foot spacing, for the top-down CoP panel's layout only.
STANCE_Y = float(os.environ.get("STANCE_Y", 0.0996 if os.environ.get("ROBOT", "").startswith("g1") else 0.0706))
# Camera distance / target height multiplier, roughly the height ratio.
CAM_SCALE = float(os.environ.get("CAM_SCALE", 2.2 if os.environ.get("ROBOT", "").startswith("g1") else 1.0))
LEGEND_NOTE = (
    ["visual: full STL geometry", "collision: primitives only"]
    if os.environ.get("ROBOT", "").startswith("g1")
    # v6: hip_roll/knee are RS00 (57x51mm), everything else Edulite05
    # (46x44mm), single motor each -- no more dual-motor joints (v3/v5
    # retired the knee's and hip_pitch's boosters).
    else ["Edulite05 46x44mm, RS00 57x51mm", "hip_roll/knee: RS00, no boosters"]
)
URDFS = {
    "kyo46rs": "/home/takara/work/dp/humanoid/kyo46rs_description/urdf/kyo46rs.urdf",
    "kyo46rs2": "/home/takara/work/dp/humanoid/kyo46rs2_description/urdf/kyo46rs2.urdf",
    "g1": "/home/takara/work/dp/articara/models/unitree_g1_src/robots/g1_description/g1_23dof.urdf",
    "g1_23dof": "/home/takara/work/dp/articara/models/unitree_g1_src/robots/g1_description/g1_23dof.urdf",
}
# No silent fallback. This defaulted to kyo46rs for ANY unknown ROBOT, so a
# kyo46rs2 run rendered as v1 and looked plausible -- the two machines differ
# by 8 mm of leg. A renderer that draws the wrong robot without saying so is
# worse than one that refuses.
if "URDF" in os.environ:
    URDF = os.environ["URDF"]
elif ROBOT in URDFS:
    URDF = URDFS[ROBOT]
else:
    raise SystemExit(
        f"ROBOT={ROBOT!r} has no URDF here. Known: {', '.join(sorted(URDFS))}. "
        f"Pass URDF=<path> to override."
    )
FOOT_L = os.environ.get("FOOT_L", "left_ankle_roll_link" if ROBOT.startswith("g1") else "left_foot_link")
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


_MESH_CACHE = {}


def load_stl(path):
    """Binary STL as a triangle soup, shape (ntri, 3, 3). No decimation.

    Two earlier attempts simplified the geometry -- a convex hull, then
    vertex clustering -- and both were the wrong trade. G1's 29 instanced
    visuals are 401k triangles, but at this camera nearly all of them are
    smaller than a pixel: the model is dense because it is a CAD export, not
    because the detail is visible. So keep every triangle and drop, per
    frame, the ones that cannot be seen. What survives is the real surface
    rather than an approximation of it.
    """
    if path in _MESH_CACHE:
        return _MESH_CACHE[path]
    import struct
    with open(path, "rb") as fh:
        ntri = struct.unpack("<I", fh.read(84)[80:84])[0]
        buf = fh.read(50 * ntri)
    if len(buf) < 50 * ntri:                       # ASCII STL, or truncated
        _MESH_CACHE[path] = None
        return None
    raw = np.frombuffer(buf, dtype=np.uint8).reshape(ntri, 50)
    tri = raw[:, 12:48].copy().view(np.float32).reshape(ntri, 3, 3).astype(np.float64)
    _MESH_CACHE[path] = tri
    return tri


# Screen-space area, in px^2, below which a triangle is not worth drawing.
MIN_TRI_PX = float(os.environ.get("MIN_TRI_PX", 1.0))
# Restore one-sided rendering. Faster, and how every frame before 2026-08-03
# was drawn -- kept so those can be reproduced, not because it is correct.
CULL = os.environ.get("CULL", "0") != "0"
_LIGHT = np.array([0.4, 0.7, 0.6])
_LIGHT = _LIGHT / np.linalg.norm(_LIGHT)


def emit_mesh(faces, tri_world, cam, base):
    """Project, cull and shade a triangle soup straight into `faces`."""
    n = len(tri_world)
    p2, z = cam.project(tri_world.reshape(-1, 3))
    p2 = p2.reshape(n, 3, 2)
    z = z.reshape(n, 3)

    keep = (z > 0.05).all(1)
    a = p2[:, 1] - p2[:, 0]
    b = p2[:, 2] - p2[:, 0]
    keep &= 0.5 * np.abs(a[:, 0] * b[:, 1] - a[:, 1] * b[:, 0]) >= MIN_TRI_PX

    e0 = tri_world[:, 1] - tri_world[:, 0]
    e1 = tri_world[:, 2] - tri_world[:, 0]
    nrm = np.cross(e0, e1)
    ln = np.linalg.norm(nrm, axis=1)
    keep &= ln > 1e-12
    nrm = nrm / np.maximum(ln, 1e-12)[:, None]
    # Two-sided. Back-face culling is only valid on a closed mesh with
    # consistent winding, and these STLs are neither -- a motor barrel with no
    # end caps loses its far wall and reads as see-through from behind, which
    # is exactly how the ankle_roll cylinder looked. Instead of dropping the
    # away-facing triangles, flip their normal toward the eye and shade them:
    # with the painter's sort running far-to-near, the near surface paints
    # over the far one and a closed shape still comes out solid.
    facing = np.einsum("ij,ij->i", nrm, cam.eye - tri_world.mean(1))
    if CULL:
        keep &= facing > 0
    else:
        nrm = np.where(facing[:, None] < 0.0, -nrm, nrm)

    idx = np.nonzero(keep)[0]
    if not len(idx):
        return
    d = 0.42 + 0.58 * np.clip(nrm[idx] @ _LIGHT, 0.0, None)
    cols = np.clip(np.asarray(base, float)[None, :] * d[:, None], 0, 255).astype(int)
    zc = z[idx].mean(1)
    pp = p2[idx]
    for k in range(len(idx)):
        faces.append((zc[k],
                      [(pp[k, 0, 0], pp[k, 0, 1]),
                       (pp[k, 1, 0], pp[k, 1, 1]),
                       (pp[k, 2, 0], pp[k, 2, 1])],
                      (cols[k, 0], cols[k, 1], cols[k, 2]),
                      None))


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
            else:
                msh = g.find("mesh")
                if msh is None:
                    continue
                fn = msh.get("filename", "")
                fn = fn.replace("package://", "")
                cand = fn if os.path.isabs(fn) else os.path.join(os.path.dirname(path), fn)
                if not os.path.exists(cand):
                    continue
                sc = msh.get("scale")
                sc = float(sc.split()[0]) if sc else 1.0
                vis.append(("mesh", np.array(xyz), rpy_to_mat(*rpy), (cand, sc), name))
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


def root_link_of(joints):
    """The one link that is nobody's child."""
    kids = {j["child"] for j in joints.values()}
    parents = {j["parent"] for j in joints.values()}
    roots = sorted(parents - kids)
    return roots[0] if roots else "torso"


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
    # w/h default to the main frame; the side panels pass their own so the
    # same projection code serves both without a second implementation.
    def __init__(self, eye, target, up=(0, 0, 1), fov=42.0, y_shift=0.0,
                 w=None, h=None):
        self.y_shift = y_shift
        self.w = W if w is None else w
        self.h = H if h is None else h
        self.eye = np.array(eye, float)
        f = np.array(target, float) - self.eye
        f /= np.linalg.norm(f)
        up = np.array(up, float)
        s = np.cross(f, up)
        s /= np.linalg.norm(s)
        u = np.cross(s, f)
        self.R = np.stack([s, u, -f])          # world -> camera
        self.fl = (self.h / 2) / math.tan(math.radians(fov) / 2)

    def project(self, pts):
        cam = (pts - self.eye) @ self.R.T
        z = -cam[:, 2]
        z = np.maximum(z, 1e-4)
        x = self.w / 2 + self.fl * cam[:, 0] / z
        y = self.h / 2 - self.y_shift - self.fl * cam[:, 1] / z
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


def gradient_bg(w=None, h=None):
    w, h = (W if w is None else w), (H if h is None else h)
    col = np.linspace(0, 1, h)[:, None]
    img = (np.array(BG_TOP)[None, None, :] * (1 - col[:, :, None])
           + np.array(BG_BOT)[None, None, :] * col[:, :, None])
    return Image.fromarray(np.repeat(img.astype(np.uint8), w, axis=1))


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
    if FOOT_L not in pose:
        return 0.0
    _, Rf = pose[FOOT_L]
    return math.degrees(math.atan2(Rf[2][1], Rf[2][2]))


def draw_body(draw, links, pose, cam):
    """Rasterise the visual primitives, painter's algorithm, back to front."""
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
                # Both rings run counter-clockwise, so both cap normals come
                # out as +z -- correct for the top, INWARD for the bottom, so
                # the bottom cap was culled whenever it faced you and drawn
                # when it did not. Reverse it.
                caps = [tuple(reversed(range(n))), tuple(range(n, 2 * n))]
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
                                  shade(base, nv_), (28, 30, 36)))
                continue
            elif kind == "mesh":
                fn, sc = size
                tri = load_stl(fn)
                if tri is None:
                    continue
                emit_mesh(faces, wp + (tri * sc) @ wR.T, cam, base)
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
                faces.append((z[list(f)].mean(), [tuple(p2[i]) for i in f], shade(base, n), (28, 30, 36)))

    # Mesh faces carry outline=None. A decimated mesh has hundreds of small
    # facets and stroking each one turns the whole link into a dark scribble;
    # the primitives are few and large and read better with an edge.
    for _, poly, col, edge in sorted(faces, key=lambda t: -t[0]):
        draw.polygon(poly, fill=col, outline=edge)


COP_MJ = (240, 196, 72)    # what MuJoCo's contacts actually produce
COP_QP = (128, 196, 246)   # what the QP's solved wrench assumes


def render_side_view(links, pose, w, h, label):
    """Frontal-plane view. The lateral weight shift is what decides this
    experiment, and it is edge-on -- invisible -- in the main camera."""
    k = CAM_SCALE
    cam = Camera(eye=(1.85 * k, 0.0, 0.33 * k), target=(0.0, 0.0, 0.33 * k),
                 fov=26, y_shift=0, w=w, h=h)
    img = gradient_bg(w, h)
    draw = ImageDraw.Draw(img, "RGBA")
    draw_ground(draw, cam)
    draw_body(draw, links, pose, cam)
    draw.rectangle([0, 0, w - 1, h - 1], outline=(64, 70, 82))
    draw.text((10, 8), label, fill=(198, 206, 220), font=F_SMALL)
    return img


def draw_cop_panel(img, ox, oy, w, h, feet, box, trails):
    """Both soles seen from above, with the centre of pressure on each.

    `feet` is [(cop_qp, cop_mj, airborne), ...] for left then right, each
    cop a (x, y, fz) triple in sole-frame metres. The rectangle drawn is
    the CoP box the QP is constrained to -- which is the sole itself, so
    a dot on the edge means the foot is about to roll over that edge.
    """
    draw = ImageDraw.Draw(img, "RGBA")
    draw.rectangle([ox, oy, ox + w - 1, oy + h - 1], fill=(16, 18, 23, 230),
                   outline=(64, 70, 82))
    draw.text((ox + 10, oy + 8), "soles from above  /  centre of pressure",
              fill=(198, 206, 220), font=F_SMALL)

    lx, ly = box                      # CoP half-extents, metres
    stance_y = STANCE_Y               # foot centres at +-STANCE_Y
    # Fit both soles plus their separation into the panel width.
    span_y = 2 * (stance_y + ly) * 1e3
    scale = (w - 56) / span_y         # px per mm
    cx, cy = ox + w / 2, oy + h / 2 + 12

    def to_px(side_y, fx, fy):
        # top-down: +x (forward) is up the screen, +y (left) is screen-left
        return (cx - (side_y + fy) * 1e3 * scale, cy - fx * 1e3 * scale)

    for i, ((qp, mj, airborne), side_y) in enumerate(zip(feet, (stance_y, -stance_y))):
        a, b = to_px(side_y, +lx, +ly), to_px(side_y, -lx, -ly)
        rect = [min(a[0], b[0]), min(a[1], b[1]), max(a[0], b[0]), max(a[1], b[1])]
        edge = (70, 78, 92) if airborne else SOLE
        draw.rectangle(rect, fill=(30, 34, 42, 210), outline=edge, width=2)
        # centre cross-hairs, so an off-centre CoP is readable at a glance
        draw.line([(rect[0], (rect[1] + rect[3]) / 2), (rect[2], (rect[1] + rect[3]) / 2)],
                  fill=(58, 64, 76))
        draw.line([((rect[0] + rect[2]) / 2, rect[1]), ((rect[0] + rect[2]) / 2, rect[3])],
                  fill=(58, 64, 76))
        draw.text((rect[0], rect[3] + 6), "LEFT" if i == 0 else "RIGHT",
                  fill=(120, 128, 142) if airborne else (168, 176, 190), font=F_SMALL)
        def readout(s, col):
            # keep the label inside the panel; the outer foot would clip
            tw = draw.textlength(s, font=F_SMALL)
            x = min(max(rect[0], ox + 8), ox + w - 8 - tw)
            draw.text((x, rect[1] - 20), s, fill=col, font=F_SMALL)

        if airborne:
            readout("airborne", (120, 128, 142))
            continue

        for pt in trails[i]:
            p = to_px(side_y, pt[0], pt[1])
            draw.ellipse([p[0] - 1.5, p[1] - 1.5, p[0] + 1.5, p[1] + 1.5],
                         fill=(96, 82, 40))
        if qp[2] > 1e-6:
            p = to_px(side_y, qp[0], qp[1])
            draw.ellipse([p[0] - 7, p[1] - 7, p[0] + 7, p[1] + 7], outline=COP_QP, width=2)
        if mj[2] > 1e-6:
            p = to_px(side_y, mj[0], mj[1])
            draw.ellipse([p[0] - 5, p[1] - 5, p[0] + 5, p[1] + 5], fill=COP_MJ)
            use = abs(mj[1]) / ly if ly > 0 else 0.0
            col = (214, 97, 90) if use > 0.97 else (168, 176, 190)
            readout(f"{mj[2]:4.0f}N y{mj[1]*1e3:+5.1f} {use:4.2f}", col)

    ky = oy + h - 24
    draw.ellipse([ox + 12, ky, ox + 22, ky + 10], fill=COP_MJ)
    draw.text((ox + 28, ky - 3), "MuJoCo", fill=(150, 158, 172), font=F_SMALL)
    draw.ellipse([ox + 118, ky - 1, ox + 130, ky + 11], outline=COP_QP, width=2)
    draw.text((ox + 138, ky - 3), "QP assumed", fill=(150, 158, 172), font=F_SMALL)


def draw_push(draw, cam, origin, fxy, phase=1.0, dt_rel=None):
    """The disturbance, drawn where it is applied and pointing where it pushes.

    The force is read from the trajectory log (`push_fx` / `push_fy`, written
    by the driver from the simulator's own live pulse list), so the arrow
    cannot disagree with what the plant actually received -- a caption saying
    "1.20 N*s to the left" can, and a viewer has no way to check it.

    The pulse itself lasts 0.10 s, which at 50 fps is five frames and is over
    before a viewer has found it. `phase` fades the same arrow in beforehand
    and leaves it fading afterwards, so the eye has somewhere to be: a hollow
    ghost while the push is coming, solid while it is applied, then a fading
    trace of what was done. `dt_rel` (seconds relative to the pulse start)
    drives the label.
    """
    f = np.array([fxy[0], fxy[1], 0.0])
    mag = float(np.linalg.norm(f))
    if mag < 1e-6 or phase <= 0.0:
        return
    scale = float(os.environ.get("PUSH_ARROW_SCALE", 0.020))
    u = f / mag
    length = 0.055 + mag * scale
    # Start the shaft back from the body so the arrow reads as pushing INTO
    # the robot rather than emerging from inside it, and lift it to chest
    # height where nothing else is drawn.
    tail = np.asarray(origin, float) + np.array([0.0, 0.0, 0.06]) - u * length
    tip = tail + u * length
    # Project the shaft, then build the head in SCREEN space. Doing it in 3D
    # spreads the head along `cross(u, up)`, which for a sideways push is the
    # fore-aft axis -- and this camera looks nearly along it, so the triangle
    # collapsed to a line and the arrow had no readable direction at all.
    (p2, _z) = cam.project(np.array([tail, tip]))
    t0 = np.array([float(p2[0][0]), float(p2[0][1])])
    t1 = np.array([float(p2[1][0]), float(p2[1][1])])
    d = t1 - t0
    n = float(np.linalg.norm(d))
    if n < 1e-6:
        return
    d = d / n
    perp = np.array([-d[1], d[0]])
    head_px = max(18.0, min(46.0, 0.30 * n))
    apex = t1
    base = t1 - d * head_px
    left = base + perp * head_px * 0.58
    right = base - perp * head_px * 0.58
    a = int(round(255 * max(0.0, min(1.0, phase))))
    solid = phase >= 0.999
    col = (255, 110, 80, a)
    shaft = max(3, int(round(11 * phase)))
    tri = [tuple(apex), tuple(left), tuple(right)]
    if solid:
        # A dark outline first: the arrow lands on the torso, and a bare
        # orange triangle on grey plastic loses its edges.
        draw.line([tuple(t0), tuple(base)], fill=(12, 14, 18, a), width=shaft + 6)
        draw.polygon([tuple(apex + d * 5), tuple(left + (perp * 4 - d * 3)),
                      tuple(right + (-perp * 4 - d * 3))], fill=(12, 14, 18, a))
        draw.line([tuple(t0), tuple(base)], fill=col, width=shaft)
        draw.polygon(tri, fill=col)
    else:
        draw.line([tuple(t0), tuple(base)], fill=col, width=max(2, shaft // 2))
        draw.polygon(tri, outline=col)
    lbl = f"PUSH  {mag:.0f} N"
    imp = os.environ.get("PUSH_IMPULSE_LABEL")
    if imp:
        lbl += f"  ({imp} N*s)"
    if dt_rel is not None:
        if dt_rel < -1e-6:
            lbl = f"PUSH in {-dt_rel:4.2f} s"
        elif dt_rel > 0.105:
            lbl = f"pushed {dt_rel - 0.10:4.2f} s ago"
    # Above the shaft's midpoint: the tail sits over the CoM-height panel and
    # the tip sits on the robot, so both ends are already busy.
    mx = 0.5 * (t0[0] + t1[0])
    my = 0.5 * (t0[1] + t1[1]) - 46
    w = draw.textlength(lbl, font=F_BODY)
    draw.rectangle([mx - w / 2 - 10, my - 6, mx + w / 2 + 10, my + 26],
                   fill=(16, 18, 23, 232))
    draw.text((mx - w / 2, my), lbl, fill=col, font=F_BODY)


def render_frame(links, joints, pose, cam, hud, push=None, push_at=None,
                 push_phase=1.0, push_dt=None):
    img = gradient_bg()
    draw = ImageDraw.Draw(img, "RGBA")
    draw_ground(draw, cam)
    draw_body(draw, links, pose, cam)
    if push is not None and push_at is not None:
        draw_push(draw, cam, push_at, push, push_phase, push_dt)

    # ── HUD ────────────────────────────────────────────────────────────
    (t, com_z, ref_z, tilt, hist, taus, tau_names, tau_lims, tau_total,
     n_stance, degraded, sole_roll, stance_side) = hud
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
        # Which foot, read from the MEASURED contact forces rather than
        # assumed. The squat example always lifted the right foot, so this
        # label was hardcoded to "LEFT"; a walk alternates, and a label that
        # names the wrong foot is how the last overlay bug (doc trap 8) turned
        # into a wrong conclusion.
        draw.text((W - 262, 48), f"contact: {stance_side} foot only",
                  fill=(226, 140, 92), font=F_BODY)
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
    for i, line in enumerate(LEGEND_NOTE):
        draw.text((kx, ky + 64 + 18 * i), line, fill=(108, 116, 130), font=F_SMALL)

    # ── live pitch-joint torque strip, full width along the bottom ─────
    px, py, pw, ph = 22, H - 140, W - 62, 92
    draw.rectangle([px - 10, py - 30, px + pw + 10, py + ph + 26], fill=(16, 18, 23, 214))
    draw.text((px, py - 26), os.environ.get("TAU_LABEL", "sagittal joint torque, left leg"),
              fill=(198, 206, 220), font=F_SMALL)

    span = float(os.environ.get("TAU_SPAN", max(2.5, 0.25 * max(tau_lims))))
    ymid = py + ph / 2
    def ty(v):
        return ymid - (ph / 2) * max(-1.0, min(1.0, v / span))

    # zero line + gridlines every 1 N*m
    for frac in (-0.8, -0.4, 0.4, 0.8):
        g = frac * span
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
    root = root_link_of(joints)
    print(f"robot={ROBOT}  root={root}  links={len(links)}  joints={len(joints)}")
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

    # Which three joints the bottom strip plots. Default is the sagittal set,
    # which is what a squat is about; a walk is about the FRONTAL plane, and on
    # kyo46rs the joint that actually runs out of torque in single support is
    # hip_roll (doc section 10.11), so that has to be selectable rather than
    # baked in.
    #   TAU_JOINTS="left_hip_roll_joint:L_hip_roll,right_hip_roll_joint:R_hip_roll,left_knee_joint:knee"
    _tj = os.environ.get("TAU_JOINTS")
    if _tj:
        TAU_JOINTS = [tuple(x.split(":", 1)) if ":" in x else (x, x) for x in _tj.split(",")]
    else:
        TAU_JOINTS = [("left_hip_pitch_joint", "hip_pitch"),
                      ("left_knee_joint", "knee"),
                      ("left_ankle_pitch_joint", "ankle_pitch")]
    tau_lims = [float(sel[0]["lim_" + c]) for c, _ in TAU_JOINTS]
    tau_names = [lbl for _, lbl in TAU_JOINTS]
    tau_hist = [[] for _ in TAU_JOINTS]
    def _xyz(name):
        v = os.environ.get(name)
        return tuple(float(x) for x in v.split(",")) if v else None

    FOLLOW = os.environ.get("CAM_FOLLOW", "0") != "0"

    if os.environ.get("ANKLE_CLOSEUP"):
        cam = Camera(eye=(0.30, -0.42, 0.16), target=(0.01, 0.0, 0.06), fov=34, y_shift=60)
    else:
        # Frame by the machine, not by kyo46rs. G1 is twice the height and
        # the old camera showed it from the knees down.
        k = CAM_SCALE
        cam = (Camera(eye=(1.30 * k, -1.55 * k, 0.62 * k), target=(0.02, 0.0, 0.28 * k), fov=30, y_shift=-10)
               if os.environ.get("COMPACT")
               else Camera(eye=(1.15 * k, -1.38 * k, 0.62 * k), target=(0.02, 0.0, 0.30 * k), fov=33, y_shift=24))
    # CAM_EYE / CAM_TARGET override the built-in framing, which is what makes
    # a view from behind checkable at all -- the bug above only shows there.
    if _xyz("CAM_EYE") or _xyz("CAM_TARGET"):
        cam = Camera(eye=_xyz("CAM_EYE") or cam.eye,
                     target=_xyz("CAM_TARGET") or (0.02, 0.0, 0.28),
                     fov=float(os.environ.get("CAM_FOV", 33)))
    base_eye = np.array(cam.eye, dtype=float)
    # Side column: frontal view on top, top-down CoP panel underneath.
    SIDE_W = 420
    FRONT_H = 350
    COP_H = H - FRONT_H
    box = (float(sel[0].get("cop_lx", 0.049)), float(sel[0].get("cop_ly", 0.019)))
    has_cop = "cop_mj_l_x" in sel[0]
    trails = [[], []]

    hist = []
    # Pre-scan the WHOLE log (not the decimated frames) for the pulse, so the
    # anticipation and decay windows are anchored on when it really happened
    # rather than on whichever frame happened to catch it.
    push_rows = [(float(x["t"]), float(x.get("push_fx", 0) or 0),
                  float(x.get("push_fy", 0) or 0)) for x in rows
                 if abs(float(x.get("push_fy", 0) or 0)) > 1e-9
                 or abs(float(x.get("push_fx", 0) or 0)) > 1e-9]
    push_t0 = push_rows[0][0] if push_rows else None
    push_t1 = push_rows[-1][0] if push_rows else None
    push_vec = (push_rows[0][1], push_rows[0][2]) if push_rows else (0.0, 0.0)
    PUSH_PRE = float(os.environ.get("PUSH_PRE", 0.8))
    PUSH_POST = float(os.environ.get("PUSH_POST", 2.0))
    if push_t0 is not None:
        print(f"push: {push_vec[1]:+.1f} N in y, t={push_t0:.3f}..{push_t1:.3f}, "
              f"shown from -{PUSH_PRE}s to +{PUSH_POST}s")

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
        pose = forward_kinematics(links, joints, root, bp, bR, q)
        # Follow the robot along x. A forward walk leaves a fixed frame within
        # a couple of seconds, and a robot that has walked out of shot cannot
        # be judged. CAM_FOLLOW=0 keeps the old fixed camera, which is what a
        # squat or a step in place wants -- there, motion relative to the
        # ground is the thing being looked at.
        if FOLLOW:
            # Translating the eye along x is enough: the camera keeps only an
            # orientation and a focal length, and both are invariant under a
            # common translation of eye and target.
            cam.eye = base_eye + np.array([bp[0], 0.0, 0.0])
        hist.append((float(r["com_z"]), float(r["com_ref_z"])))
        for k, (col, _) in enumerate(TAU_JOINTS):
            tau_hist[k].append(float(r["tau_" + col]))
        # Show the arrow across a window around the pulse, not only while it
        # is live: 0.10 s is five frames at 50 fps and is gone before the eye
        # finds it.
        push_xy, push_phase, push_dt = (0.0, 0.0), 0.0, None
        if push_t0 is not None:
            tt = float(r["t"])
            push_dt = tt - push_t0
            if -PUSH_PRE <= push_dt <= (push_t1 - push_t0) + PUSH_POST:
                push_xy = push_vec
                if push_dt < 0.0:
                    push_phase = 0.15 + 0.55 * (1.0 + push_dt / PUSH_PRE)
                elif tt <= push_t1 + 1e-9:
                    push_phase = 1.0
                else:
                    decay = (tt - push_t1) / PUSH_POST
                    push_phase = 0.75 * (1.0 - decay)
        img = render_frame(links, joints, pose, cam,
                           (float(r["t"]), float(r["com_z"]), float(r["com_ref_z"]),
                            float(r["tilt"]), hist[-260:],
                            tau_hist, tau_names, tau_lims, len(sel),
                            int(r.get("n_stance", 2)),
                            # the solver's ACTUAL status when the log carries
                            # it; older CSVs have no such column, and stance
                            # count is not a substitute, so say nothing then
                            bool(int(r.get("degraded", 0))),
                            sole_roll_deg(pose),
                            ("LEFT" if float(r.get("fz_mj_l", 0)) >= float(r.get("fz_mj_r", 0))
                             else "RIGHT")),
                           push=push_xy, push_at=bp,
                           push_phase=push_phase, push_dt=push_dt)
        if has_cop:
            feet = []
            for side in ("l", "r"):
                qp = tuple(float(r[f"cop_qp_{side}_{k}"]) for k in ("x", "y")) \
                     + (float(r[f"fz_qp_{side}"]),)
                mj = tuple(float(r[f"cop_mj_{side}_{k}"]) for k in ("x", "y")) \
                     + (float(r[f"fz_mj_{side}"]),)
                feet.append((qp, mj, mj[2] <= 1e-6))
            for k in range(2):
                if not feet[k][2]:
                    trails[k].append(feet[k][1][:2])
                else:
                    trails[k].clear()
                del trails[k][:-90]

            canvas = Image.new("RGB", (W + SIDE_W, H), BG_TOP)
            canvas.paste(img, (0, 0))
            canvas.paste(render_side_view(links, pose, SIDE_W, FRONT_H,
                                          "front view  (frontal plane)"),
                         (W, 0))
            draw_cop_panel(canvas, W, FRONT_H, SIDE_W, COP_H, feet, box, trails)
            img = canvas

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
