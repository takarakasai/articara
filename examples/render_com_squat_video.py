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
def _legend():
    """Two lines naming what the actuators ARE.

    This has now gone stale twice -- once for the v3/v5 booster retirements
    (fixed in 786d0e8) and again for v7/v8, where the arms became Dynamixels
    and the hardcoded "Edulite05" line started captioning a robot that has
    none in its arms. A caption nobody re-derives is a caption that lies, so
    `LEGEND` overrides it and the default at least says which file was drawn.
    """
    if "LEGEND" in os.environ:
        return (os.environ["LEGEND"].split("|") + ["", ""])[:2]
    if os.environ.get("ROBOT", "").startswith("g1"):
        return ["visual: full STL geometry", "collision: primitives only"]
    urdf = os.environ.get("URDF", "")
    if "_v8" in urdf:
        return ["legs Edulite05 / RS00 (hip_roll, knee)", "arms XM430-W210 x3 per side"]
    if "_v7" in urdf:
        return ["legs Edulite05 / RS00 (hip_roll, knee)", "arms XM430 x2 + XM335 elbow"]
    # v6 and its cuts: hip_roll/knee RS00 (57x51), everything else Edulite05
    # (46x44), single motor each -- v3/v5 retired the knee's and hip_pitch's.
    return ["Edulite05 46x44mm, RS00 57x51mm", "hip_roll/knee: RS00, no boosters"]


LEGEND_NOTE = _legend()
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
    # the depth buffer selects the near surface and a closed shape stays solid.
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
    depths = z[idx]
    pp = p2[idx]
    for k in range(len(idx)):
        faces.append((depths[k],
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
    # the SPAR_WALL target, so a punch has something visible to land on
    "target": (188, 78, 56),
    # staircase treads -- mid grey, darker than the robot's housings so the
    # silhouette still reads against them
    "stair": (96, 102, 114),
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
    """Rasterise visual primitives with per-pixel perspective-correct depth."""
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
                # Polygonal sides and caps use the same depth buffer as boxes.
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
                    faces.append((z[list(f)], [tuple(p2[i]) for i in f],
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
                faces.append((z[list(f)], [tuple(p2[i]) for i in f], shade(base, n), (28, 30, 36)))

    rasterize_faces(draw._image, faces)


def rasterize_faces(image, faces):
    """Depth-test convex polygons, including large surfaces crossing small ones.

    Screen-space barycentric weights interpolate reciprocal camera depth;
    sorting by mean depth cannot resolve partially overlapping polygons.
    Polygon edges participate in the same depth test as their surface.
    """
    pixels = np.array(image)
    height, width = pixels.shape[:2]
    inverse_depth = np.zeros((height, width), dtype=float)
    for depths, polygon, color, edge in faces:
        points = np.asarray(polygon, dtype=float)
        depths = np.asarray(depths, dtype=float)
        for i in range(1, len(points) - 1):
            indices = [0, i, i + 1]
            a, b, c = points[indices]
            x0, y0 = np.maximum(np.floor(np.min([a, b, c], axis=0)), [0, 0]).astype(int)
            x1, y1 = np.minimum(np.ceil(np.max([a, b, c], axis=0)), [width - 1, height - 1]).astype(int)
            if x0 > x1 or y0 > y1:
                continue
            denominator = (b[1] - c[1]) * (a[0] - c[0]) + (c[0] - b[0]) * (a[1] - c[1])
            if abs(denominator) < 1e-10:
                continue
            yy, xx = np.mgrid[y0:y1 + 1, x0:x1 + 1]
            xx, yy = xx + 0.5, yy + 0.5
            wa = ((b[1] - c[1]) * (xx - c[0]) + (c[0] - b[0]) * (yy - c[1])) / denominator
            wb = ((c[1] - a[1]) * (xx - c[0]) + (a[0] - c[0]) * (yy - c[1])) / denominator
            wc = 1 - wa - wb
            iz = wa / depths[0] + wb / depths[i] + wc / depths[i + 1]
            region = inverse_depth[y0:y1 + 1, x0:x1 + 1]
            visible = (wa >= -1e-9) & (wb >= -1e-9) & (wc >= -1e-9) & (iz > region)
            if not visible.any():
                continue
            rgb = pixels[y0:y1 + 1, x0:x1 + 1]
            rgb[visible] = color
            region[visible] = iz[visible]
            if edge is not None:
                # Stroke only the polygon boundary, never the fan diagonals.
                boundary = np.zeros_like(visible)
                for start, end in zip(points, np.roll(points, -1, axis=0)):
                    delta = end - start
                    length2 = float(delta @ delta)
                    if length2 < 1e-12:
                        continue
                    along = np.clip(((xx - start[0]) * delta[0] + (yy - start[1]) * delta[1]) / length2, 0, 1)
                    distance2 = (xx - start[0] - along * delta[0]) ** 2 + (yy - start[1] - along * delta[1]) ** 2
                    boundary |= distance2 < 0.36
                rgb[visible & boundary] = edge
    image.paste(Image.fromarray(pixels))


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
                 push_phase=1.0, push_dt=None, draw_set=None, hit=None):
    """`draw_set` overrides WHAT is rasterised without changing what the HUD
    reads. Two robots are drawn from one merged dict so the depth buffer
    orders them against each other, but every HUD panel below is about the
    logged robot and would read a prefixed key as a missing joint."""
    img = gradient_bg()
    draw = ImageDraw.Draw(img, "RGBA")
    draw_ground(draw, cam)
    dl, dp = draw_set if draw_set else (links, pose)
    draw_body(draw, dl, dp, cam)
    if hit is not None:
        # What the punch is actually delivering, on the frame that delivers it.
        # Peak is carried across frames by the caller through `hit_peak`.
        f_now, imp_punch, imp_total = hit
        live = f_now > 0.0
        # Sits ABOVE the sagittal-torque strip (label from H-166, plot from
        # H-140): the box below spans [y0-8, y0+118], so y0 = H-300 keeps an
        # 8 px gap. At H-152 it landed inside the strip and neither was legible.
        x0, y0 = 16, H - 300
        draw.rectangle([x0 - 8, y0 - 8, x0 + 348, y0 + 118],
                       fill=(16, 18, 23, 224))
        draw.text((x0, y0), "force on target",
                  fill=(226, 140, 92) if live else (120, 126, 138), font=F_SMALL)
        draw.text((x0 + 170, y0), f"{max(f_now, 0.0):6.1f} N",
                  fill=(255, 196, 96) if live else (120, 126, 138), font=F_BODY)
        draw.text((x0, y0 + 30), "impulse, this punch", fill=(170, 178, 190), font=F_SMALL)
        draw.text((x0 + 200, y0 + 30), f"{imp_punch:5.3f} N*s",
                  fill=(255, 196, 96) if live else (150, 156, 168), font=F_SMALL)
        draw.text((x0, y0 + 52), "impulse, run total", fill=(170, 178, 190), font=F_SMALL)
        draw.text((x0 + 200, y0 + 52), f"{imp_total:6.3f} N*s",
                  fill=(190, 196, 206), font=F_SMALL)
        draw.text((x0, y0 + 76), "vs frontal topple threshold 0.6 N*s",
                  fill=(150, 156, 168), font=F_SMALL)
        # This punch's impulse against that threshold. Full width = 2x it, so a
        # bar past the halfway tick is a hit that would put someone down.
        bx, by, bw = x0, y0 + 98, 336
        draw.rectangle([bx, by, bx + bw, by + 12], outline=(90, 96, 108))
        w = int(min(imp_punch / 1.2, 1.0) * bw)
        if w > 0:
            draw.rectangle([bx, by, bx + w, by + 12],
                           fill=(226, 96, 72) if imp_punch > 0.6 else (110, 190, 130))
        draw.line([bx + bw // 2, by - 3, bx + bw // 2, by + 15], fill=(210, 214, 222))
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
        # hud[-1] is `stance_side`, a string; the sole roll is hud[-2]. This
        # only ever ran under COMPACT, so the crash sat here unseen.
        sr = hud[-2]
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
    sub = os.environ.get("SUBTITLE")
    if sub:
        draw.rectangle([0, 84, W, 112], fill=(16, 18, 23, 180))
        draw.text((22, 88), sub, fill=(200, 206, 216), font=F_SMALL)
    if os.environ.get("NO_PANELS"):
        # Summary videos: header only; the plan view carries the rest.
        return img

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
        # The band this trace is drawn against. The defaults are kyo46rs's
        # squat range; G1 stands at 0.685 m, which is off the box entirely and
        # renders as an empty panel, so the band is overridable per robot.
        lo = float(os.environ.get("COM_PLOT_LO", 0.24))
        hi = float(os.environ.get("COM_PLOT_HI", 0.33))
        def sc(i, v):
            return (x0 + ww * i / max(1, len(hist) - 1),
                    y0 + hh * (1 - (v - lo) / (hi - lo)))
        draw.line([sc(i, r) for i, (_, r) in enumerate(hist)], fill=(110, 120, 140), width=1)
        draw.line([sc(i, c) for i, (c, _) in enumerate(hist)], fill=(232, 168, 92), width=2)
        draw.text((x0, y0 - 22), "CoM height vs commanded", fill=(150, 158, 172), font=F_SMALL)
    return img


# ── planner overlay (TARGETS=1) ─────────────────────────────────────────
# What the planners were aiming at, drawn in the 3D view on top of the body:
#   PREVIEW_CSV  the LIPM bounded-ZMP CoM preview (COM_REPLAN): planned ZMP
#                sequence and predicted CoM path of the latest solve
#   MPC_CSV      the SRBD wrench MPC: predicted CoM at each node end, node-0
#                net CoP, landing shift (G1 recipes; kyo46rs runs without it)
# plus, from the trajectory CSV itself: DCM and its reference, the ZMP
# reference, the CoM, the planned landing of the swing foot and the swing
# target. Written by kyo46rs_walk.rs (PREVIEW_TRAJ_CSV / MPC_TRAJ_CSV).
C_ZMP = (96, 210, 240)
C_PCOM = (230, 120, 230)
C_DCM = (255, 170, 60)
C_COM = (240, 244, 250)
C_LAND = (120, 220, 130)
C_MPC = (250, 230, 90)


def load_planner_csv(path, long_form):
    """{t: rows} for the long-form preview file, or [(t, row)] for the MPC."""
    if not path or not os.path.exists(path):
        return None
    with open(path) as f:
        rows = list(csv.DictReader(f))
    if long_form:
        out = {}
        for r in rows:
            out.setdefault(float(r["t"]), []).append(r)
        ts = sorted(out)
        return ts, out
    ts = [float(r["t"]) for r in rows]
    return ts, rows


def latest_at(store, t, max_age):
    if store is None:
        return None
    ts, data = store
    import bisect
    i = bisect.bisect_right(ts, t + 1e-9) - 1
    if i < 0 or t - ts[i] > max_age:
        return None
    return data[ts[i]] if isinstance(data, dict) else data[i]


def _f(r, k):
    try:
        v = float(r[k])
    except (KeyError, ValueError, TypeError):
        return float("nan")
    return v


def draw_targets(img, cam, r, prev, mpc, foot_half, legend=True):
    draw = ImageDraw.Draw(img, "RGBA")
    fl, fr = _f(r, "fz_mj_l"), _f(r, "fz_mj_r")
    zg = _f(r, "sole_l_z") if fl >= fr else _f(r, "sole_r_z")
    if not math.isfinite(zg):
        zg = 0.0
    zg += 0.002

    def px(pts):
        p, z = cam.project(np.asarray(pts, float).reshape(-1, 3))
        return [tuple(map(float, q)) for q in p], z

    def dot(x, y, z, col, rad=6, ring=False, alpha=235):
        if not all(map(math.isfinite, (x, y, z))):
            return
        (q,), _ = px([[x, y, z]])
        bb = [q[0] - rad, q[1] - rad, q[0] + rad, q[1] + rad]
        if ring:
            draw.ellipse(bb, outline=col + (alpha,), width=3)
        else:
            draw.ellipse(bb, fill=col + (alpha,), outline=(20, 20, 24, 200))

    def poly(pts, col, width=3, alpha=220):
        pts = [p for p in pts if all(map(math.isfinite, p))]
        if len(pts) < 2:
            return
        q, _ = px(pts)
        draw.line(q, fill=col + (alpha,), width=width)

    # planned landing of the swing foot (sole outline on its target surface)
    sup = int(_f(r, "support")) if math.isfinite(_f(r, "support")) else 0
    swing = {1: "r", 2: "l"}.get(sup)
    if swing:
        gx, gy, gz = _f(r, f"goal_{swing}_x"), _f(r, f"goal_{swing}_y"), _f(r, f"goal_{swing}_z")
        if all(map(math.isfinite, (gx, gy, gz))) and (gx or gy):
            hx, hy = foot_half
            c = [[gx - hx, gy - hy, gz + 0.002], [gx + hx, gy - hy, gz + 0.002],
                 [gx + hx, gy + hy, gz + 0.002], [gx - hx, gy + hy, gz + 0.002],
                 [gx - hx, gy - hy, gz + 0.002]]
            poly(c, C_LAND, width=3)
        tx, ty, tz = _f(r, "swing_tgt_x"), _f(r, "swing_tgt_y"), _f(r, "swing_tgt_z")
        if (tx or ty) and all(map(math.isfinite, (tx, ty, tz))):
            (q,), _ = px([[tx, ty, tz]])
            draw.line([q[0] - 7, q[1], q[0] + 7, q[1]], fill=C_LAND + (240,), width=3)
            draw.line([q[0], q[1] - 7, q[0], q[1] + 7], fill=C_LAND + (240,), width=3)

    # LIPM CoM preview: planned ZMP sequence on the ground, predicted CoM path
    cz = _f(r, "com_z")
    if prev:
        zs = [[_f(p, "zmp_x"), _f(p, "zmp_y"), zg] for p in prev]
        cs = [[_f(r, "com_x"), _f(r, "com_y"), cz]] + [[_f(p, "c_x"), _f(p, "c_y"), cz] for p in prev]
        poly(zs, C_ZMP, width=2, alpha=200)
        for k, z in enumerate(zs):
            if k % 2 == 0:
                dot(*z, C_ZMP, rad=3, alpha=210)
        poly(cs, C_PCOM, width=3)
    # SRBD wrench MPC: predicted CoM at node ends, node-0 net CoP
    if mpc:
        n = int(_f(mpc, "n_nodes")) if math.isfinite(_f(mpc, "n_nodes")) else 0
        cs = [[_f(mpc, "com_x"), _f(mpc, "com_y"), _f(mpc, "com_z")]] + \
             [[_f(mpc, f"c{k}_x"), _f(mpc, f"c{k}_y"), _f(mpc, f"c{k}_z")] for k in range(min(n, 16))]
        poly(cs, C_MPC, width=3)
        for c in cs[1:]:
            dot(*c, C_MPC, rad=3)
        dot(_f(mpc, "cop0_x"), _f(mpc, "cop0_y"), zg, C_MPC, rad=7, ring=True)
    # references and state
    dot(_f(r, "zmp_ref_x"), _f(r, "zmp_ref_y"), zg, C_ZMP, rad=7)
    dot(_f(r, "xi_ref_x"), _f(r, "xi_ref_y"), zg, C_DCM, rad=8, ring=True)
    dot(_f(r, "xi_x"), _f(r, "xi_y"), zg, C_DCM, rad=6)
    dot(_f(r, "com_x"), _f(r, "com_y"), cz, C_COM, rad=6)
    dot(_f(r, "com_x"), _f(r, "com_y"), zg, C_COM, rad=3, alpha=160)

    if not legend:
        return
    # legend
    items = [("CoM (and its ground point)", C_COM, "dot"),
             ("DCM  (ring = reference)", C_DCM, "dot"),
             ("ZMP reference", C_ZMP, "dot")]
    if prev is not None:
        items += [("preview: planned ZMP", C_ZMP, "line"), ("preview: predicted CoM", C_PCOM, "line")]
    if mpc is not None:
        items += [("wrench MPC: CoM nodes / CoP", C_MPC, "line")]
    items += [("landing target / swing target", C_LAND, "line")]
    x0, y0 = 14, img.size[1] - 18 - 22 * len(items)
    draw.rectangle([x0 - 8, y0 - 10, x0 + 330, img.size[1] - 10], fill=(16, 18, 23, 200))
    for k, (lbl, col, kind) in enumerate(items):
        y = y0 + 22 * k
        if kind == "dot":
            draw.ellipse([x0, y + 3, x0 + 12, y + 15], fill=col + (240,))
        else:
            draw.line([x0, y + 9, x0 + 16, y + 9], fill=col + (240,), width=3)
        draw.text((x0 + 24, y), lbl, fill=(214, 220, 230), font=F_SMALL)


def draw_plan_panel(img, ox, oy, w, h, r, prev, mpc, foot_half, edges):
    """Top-down world-frame view around the robot: soles, stair edges, the
    planned landing, ZMP/DCM/CoM and the planners' predicted paths."""
    draw = ImageDraw.Draw(img, "RGBA")
    draw.rectangle([ox, oy, ox + w, oy + h], fill=(18, 20, 26))
    draw.text((ox + 10, oy + 8), "plan view  (world, from above)", fill=(214, 220, 230), font=F_SMALL)
    top = oy + 32
    legend_h = 150
    ph = h - 32 - legend_h
    scale = float(os.environ.get("PLAN_SCALE", 900.0))  # px per metre
    cx = _f(r, "com_x") + 0.06
    cy = 0.0

    def P(x, y):
        return (ox + w / 2 + (x - cx) * scale, top + ph / 2 - (y - cy) * scale)

    def inside(q):
        return ox <= q[0] <= ox + w and top <= q[1] <= top + ph

    # 5 cm grid
    g0 = math.floor((cx - w / 2 / scale) / 0.05) * 0.05
    x = g0
    while x < cx + w / 2 / scale:
        q = P(x, 0)
        draw.line([q[0], top, q[0], top + ph], fill=(34, 38, 46), width=1)
        x += 0.05
    y = -0.2
    while y <= 0.2 + 1e-9:
        q = P(0, y)
        if top <= q[1] <= top + ph:
            draw.line([ox, q[1], ox + w, q[1]], fill=(34, 38, 46), width=1)
        y += 0.05
    for ex, lbl in edges:
        q = P(ex, 0)
        if ox <= q[0] <= ox + w:
            draw.line([q[0], top, q[0], top + ph], fill=(200, 200, 210, 160), width=2)
            draw.text((q[0] + 4, top + 4), lbl, fill=(170, 176, 188), font=F_SMALL)
    hx, hy = foot_half
    for side, fz in (("l", _f(r, "fz_mj_l")), ("r", _f(r, "fz_mj_r"))):
        sx, sy = _f(r, f"sole_{side}_x"), _f(r, f"sole_{side}_y")
        a, b = P(sx - hx, sy + hy), P(sx + hx, sy - hy)
        if fz > 3.0:
            draw.rectangle([a, b], fill=(60, 150, 160, 120), outline=(90, 190, 200), width=2)
        else:
            draw.rectangle([a, b], outline=(110, 116, 128), width=2)
    sup = int(_f(r, "support")) if math.isfinite(_f(r, "support")) else 0
    swing = {1: "r", 2: "l"}.get(sup)
    if swing:
        gx, gy = _f(r, f"goal_{swing}_x"), _f(r, f"goal_{swing}_y")
        if math.isfinite(gx) and (gx or gy):
            draw.rectangle([P(gx - hx, gy + hy), P(gx + hx, gy - hy)], outline=C_LAND + (255,), width=3)
        tx, ty = _f(r, "swing_tgt_x"), _f(r, "swing_tgt_y")
        if (tx or ty) and math.isfinite(tx):
            q = P(tx, ty)
            draw.line([q[0] - 7, q[1], q[0] + 7, q[1]], fill=C_LAND + (255,), width=3)
            draw.line([q[0], q[1] - 7, q[0], q[1] + 7], fill=C_LAND + (255,), width=3)

    def dot(x, y, col, rad=5, ring=False):
        if not (math.isfinite(x) and math.isfinite(y)):
            return
        q = P(x, y)
        bb = [q[0] - rad, q[1] - rad, q[0] + rad, q[1] + rad]
        if ring:
            draw.ellipse(bb, outline=col + (255,), width=3)
        else:
            draw.ellipse(bb, fill=col + (255,), outline=(16, 16, 20))

    def poly(pts, col, width=3):
        pts = [P(x, y) for x, y in pts if math.isfinite(x) and math.isfinite(y)]
        if len(pts) >= 2:
            draw.line(pts, fill=col + (230,), width=width)

    if prev:
        poly([(_f(p, "zmp_x"), _f(p, "zmp_y")) for p in prev], C_ZMP, width=2)
        for p in prev[::2]:
            dot(_f(p, "zmp_x"), _f(p, "zmp_y"), C_ZMP, rad=3)
        poly([(_f(r, "com_x"), _f(r, "com_y"))] + [(_f(p, "c_x"), _f(p, "c_y")) for p in prev], C_PCOM, width=3)
    if mpc:
        n = int(_f(mpc, "n_nodes")) if math.isfinite(_f(mpc, "n_nodes")) else 0
        pts = [(_f(mpc, "com_x"), _f(mpc, "com_y"))] + [(_f(mpc, f"c{k}_x"), _f(mpc, f"c{k}_y")) for k in range(min(n, 16))]
        poly(pts, C_MPC, width=3)
        for x, y in pts[1:]:
            dot(x, y, C_MPC, rad=3)
        dot(_f(mpc, "cop0_x"), _f(mpc, "cop0_y"), C_MPC, rad=8, ring=True)
    dot(_f(r, "zmp_ref_x"), _f(r, "zmp_ref_y"), C_ZMP, rad=6)
    dot(_f(r, "xi_ref_x"), _f(r, "xi_ref_y"), C_DCM, rad=8, ring=True)
    dot(_f(r, "xi_x"), _f(r, "xi_y"), C_DCM, rad=5)
    dot(_f(r, "com_x"), _f(r, "com_y"), C_COM, rad=6)
    # legend
    items = [("CoM", C_COM, "dot"), ("DCM (ring = ref)", C_DCM, "dot"),
             ("ZMP ref", C_ZMP, "dot"),
             ("plan: ZMP", C_ZMP, "line"), ("plan: CoM", C_PCOM, "line")]
    if mpc is not None or os.environ.get("MPC_CSV"):
        items.append(("SRBD MPC: CoM", C_MPC, "line"))
    items.append(("landing / swing tgt", C_LAND, "line"))
    ly = oy + h - legend_h + 4
    col_w = w // 2
    for k, (lbl, col, kind) in enumerate(items):
        x0 = ox + 10 + (k % 2) * col_w
        y0 = ly + (k // 2) * 22
        if kind == "dot":
            draw.ellipse([x0, y0 + 3, x0 + 12, y0 + 15], fill=col + (255,))
        else:
            draw.line([x0, y0 + 9, x0 + 16, y0 + 9], fill=col + (255,), width=3)
        draw.text((x0 + 20, y0), lbl, fill=(200, 206, 216), font=F_SMALL)
    note = os.environ.get("PLAN_NOTE")
    if note:
        draw.text((ox + 10, oy + h - 24), note, fill=(150, 156, 168), font=F_SMALL)


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

    # A second robot, drawn from its own trajectory, turned to face this one.
    # Kinematic only: nothing in this file simulates contact, so this shows
    # reach and timing and NOT what a hit would do -- articara's MujocoSim
    # takes a single RobotModel, so two bodies in one scene is a rig change,
    # not a rendering one (doc Sec.42).
    OPP = os.environ.get("OPPONENT_CSV")
    OPP_GAP = float(os.environ.get("OPPONENT_GAP", 0.473))
    OPP_YAW = math.radians(float(os.environ.get("OPPONENT_YAW", 180.0)))
    opp_rows = list(csv.DictReader(open(OPP))) if OPP else None

    # The thing the run was punching. `MujocoSim` has it via
    # MjcfExportOptions::extra_worldbody, but this renderer builds the scene
    # from the URDF alone and would draw the fist stopping in mid-air against
    # nothing. SPAR_BOX="x,y,z,sx,sy,sz" (full extents, metres).
    if os.environ.get("SPAR_BOX"):
        v = [float(x) for x in os.environ["SPAR_BOX"].split(",")]
        if len(v) != 6:
            raise SystemExit("SPAR_BOX wants x,y,z,sx,sy,sz")
        spar = (np.array(v[:3]), np.array(v[3:]))
    else:
        spar = None

    # The staircase the run walked on. Same STAIRS string as the sim
    # (`n,riser,tread[,start_x]`), and the geometry below has to mirror
    # kyo46rs_walk.rs's worldbody builder box for box, or the feet visibly
    # land in (or above) the drawn steps.
    stair_geoms = []   # (centre xyz, FULL extents xyz)
    if os.environ.get("STAIRS"):
        v = [float(x) for x in os.environ["STAIRS"].split(",")]
        n, riser, tread = int(v[0]), v[1], v[2]
        start = v[3] if len(v) > 3 else 0.25
        half_w = float(os.environ.get("STAIR_HALF_W", 0.30))
        base = n * -riser if riser < 0 else 0.0

        def _add(x0, x1, top):
            if top > 1e-9:
                stair_geoms.append((np.array([0.5 * (x0 + x1), 0.0, 0.5 * top]),
                                    np.array([x1 - x0, 2 * half_w, top])))
        if riser < 0:
            _add(start - 0.55, start, base)
        for k in range(1, n + 1):
            _add(start + (k - 1) * tread, start + k * tread, base + k * riser)
        if riser > 0:
            # STAIR_LANDING_LEN / STAIRS_DOWN mirror the walker's knobs (the
            # up-and-over course); unset = the historical 0.60 m landing.
            land_len = float(os.environ.get("STAIR_LANDING_LEN", 0.60))
            le = start + n * tread + land_len
            _add(start + n * tread, le, n * riser)
            down_n = min(int(float(os.environ.get("STAIRS_DOWN", 0))), n)
            for k in range(1, down_n + 1):
                _add(le + (k - 1) * tread, le + k * tread, (n - k) * riser)
            if 0 < down_n < n:
                _add(le + down_n * tread, le + down_n * tread + 4.0, (n - down_n) * riser)

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

    PLAN_EDGES = []
    if os.environ.get("STAIRS"):
        v = [float(x) for x in os.environ["STAIRS"].split(",")]
        if abs(v[1]) > 1e-9:
            st0 = v[3] if len(v) > 3 else 0.25
            for k in range(int(v[0]) + (0 if v[1] < 0 else 1)):
                PLAN_EDGES.append((st0 + k * v[2], "edge" if k == 0 else ""))
    PREV = load_planner_csv(os.environ.get("PREVIEW_CSV"), True)
    MPCS = load_planner_csv(os.environ.get("MPC_CSV"), False)
    FOOT_HALF = (float(os.environ.get("FOOT_HALF_X", sel[0].get("cop_lx", 0.049))),
                 float(os.environ.get("FOOT_HALF_Y", 0.030)))
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
        links_draw, pose_draw = links, pose
        hit = None
        if spar is not None and "hit_f" in r:
            hit = (float(r["hit_f"]), float(r["hit_imp_punch"]), float(r["hit_imp_total"]))
        if spar is not None or stair_geoms:
            links_draw = dict(links_draw)
            pose_draw = dict(pose_draw)
        if spar is not None:
            links_draw["spar_target"] = [("box", np.zeros(3), np.eye(3), spar[1], "target")]
            pose_draw["spar_target"] = (spar[0], np.eye(3))
        for gi, (sc, ss) in enumerate(stair_geoms):
            links_draw[f"stair_{gi}"] = [("box", np.zeros(3), np.eye(3), ss, "stair")]
            pose_draw[f"stair_{gi}"] = (sc, np.eye(3))
        if opp_rows:
            # Same clock, clamped: the two runs need not be the same length.
            ro = opp_rows[min(int(i * len(opp_rows) / max(1, len(sel))), len(opp_rows) - 1)]
            qo = {n: float(ro[n]) for n in jnames}
            bpo = np.array([float(ro["x"]), float(ro["y"]), float(ro["z"])])
            w, xq, yq, zq = (float(ro["qw"]), float(ro["qx"]), float(ro["qy"]), float(ro["qz"]))
            nn = math.sqrt(w * w + xq * xq + yq * yq + zq * zq) or 1.0
            w, xq, yq, zq = w / nn, xq / nn, yq / nn, zq / nn
            bRo = np.array([
                [1 - 2 * (yq * yq + zq * zq), 2 * (xq * yq - zq * w), 2 * (xq * zq + yq * w)],
                [2 * (xq * yq + zq * w), 1 - 2 * (xq * xq + zq * zq), 2 * (yq * zq - xq * w)],
                [2 * (xq * zq - yq * w), 2 * (yq * zq + xq * w), 1 - 2 * (xq * xq + yq * yq)],
            ])
            cy, sy = math.cos(OPP_YAW), math.sin(OPP_YAW)
            Rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
            off = np.array([OPP_GAP, 0.0, 0.0])
            po = forward_kinematics(links, joints, root, Rz @ bpo + off, Rz @ bRo, qo)
            links_draw = {f"A/{k}": v for k, v in links.items()}
            links_draw.update({f"B/{k}": v for k, v in links.items()})
            pose_draw = {f"A/{k}": v for k, v in pose.items()}
            pose_draw.update({f"B/{k}": v for k, v in po.items()})
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
                           push_phase=push_phase, push_dt=push_dt,
                           draw_set=(links_draw, pose_draw)
                           if (opp_rows or spar or stair_geoms) else None,
                           hit=hit)
        if os.environ.get("TARGETS"):
            tt = float(r["t"])
            draw_targets(img, cam, r,
                         latest_at(PREV, tt, 0.10) if PREV is not None else None,
                         latest_at(MPCS, tt, 0.15) if MPCS is not None else None,
                         FOOT_HALF, legend=not has_cop)
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
            if os.environ.get("TARGETS"):
                tt = float(r["t"])
                draw_plan_panel(canvas, W, FRONT_H, SIDE_W, COP_H, r,
                                latest_at(PREV, tt, 0.10) if PREV is not None else None,
                                latest_at(MPCS, tt, 0.15) if MPCS is not None else None,
                                FOOT_HALF, PLAN_EDGES)
            else:
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
