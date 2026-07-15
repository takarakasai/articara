#!/usr/bin/env python3
"""Render tests/wbc_walk_go2.rs's WBC_WALK_CSV_OUT trace (every real
MuJoCo body's world pose per tick: base + all 4 legs' hip/thigh/calf/
foot) as an MP4, using Go2's real meshes (via a name-keyed join of
go2_mesh_manifest.csv + go2_topology.csv, both from misa-wbc's
go2_leg_singularity_demo) -- the full articulated robot actually
walking under WBC + Trot in MuJoCo, not the SRBD trunk-only abstraction
render_trot_mpc_horizon.py uses.
"""
import argparse
import csv
from pathlib import Path
import subprocess
import sys

import numpy as np
import vtk

ap = argparse.ArgumentParser(description=__doc__)
ap.add_argument("--trace", required=True)
ap.add_argument("--mesh-manifest", required=True, help="go2_mesh_manifest.csv (parent_joint-keyed)")
ap.add_argument("--topology", required=True, help="go2_topology.csv (idx,name,parent)")
ap.add_argument("--out", required=True)
ap.add_argument("--frames-dir", required=True)
ap.add_argument("--title", default="misa-wbc + quadruped-gait -- Go2 Trot walk (MuJoCo)")
ap.add_argument("--stride", type=int, default=4, help="sample every Nth tick")
ap.add_argument("--fps", type=int, default=50)
ap.add_argument("--staircase-step-s", type=float, default=None,
                 help="if set, show the commanded-speed readout in the title")
ap.add_argument("--staircase-step-mps", type=float, default=0.5,
                 help="velocity increment per staircase level (m/s)")
ap.add_argument("--staircase-max-mps", type=float, default=5.0,
                 help="top commanded speed (m/s); with --staircase-step-mps sets the level count")
args = ap.parse_args()

LINK_NAMES = (
    ["base"]
    + [f"{p}_{part}" for p in ("FL", "FR", "RL", "RR") for part in ("hip", "thigh", "calf", "foot")]
)

# ---- idx -> link name (mirrors the Rust suffix-stripping convention) ----
idx_to_link = {0: "base"}
with open(args.topology) as f:
    for row in csv.DictReader(f):
        idx = int(row["idx"])
        name = row["name"]
        if name.endswith("_joint"):
            link = name[: -len("_joint")]
        elif name.endswith("_fixed"):
            link = name[: -len("_fixed")]
        else:
            link = name
        idx_to_link[idx] = link

# ---- mesh manifest, keyed by link name ----
meshes_by_link = {}
with open(args.mesh_manifest) as f:
    for row in csv.DictReader(f):
        idx = int(row["parent_joint"])
        link = idx_to_link.get(idx)
        if link is None:
            continue
        t = np.array([float(row["tx"]), float(row["ty"]), float(row["tz"])])
        r = np.array([[float(row[f"r{i}{j}"]) for j in range(3)] for i in range(3)])
        meshes_by_link.setdefault(link, []).append((row["mesh_path"], t, r))

# ---- trace CSV: tick,t, then per-link tx,ty,tz,r00..r22 (12 cols each) ----
IDX_T = 1
IDX_LINK0 = 2
STRIDE_COLS = 12

rows = []
with open(args.trace) as f:
    header = f.readline()
    for line in f:
        rows.append([float(v) for v in line.strip().split(",")])
rows = np.array(rows)
n_frames_total = len(rows)


def link_pose(row, link_idx):
    base = IDX_LINK0 + link_idx * STRIDE_COLS
    t = row[base : base + 3]
    r = np.array(row[base + 3 : base + 12]).reshape(3, 3)
    return t, r


def compose(t1, r1, t2, r2):
    return r1 @ t2 + t1, r1 @ r2


def vtk_matrix(t, r):
    m = vtk.vtkMatrix4x4()
    m.Identity()
    for i in range(3):
        for j in range(3):
            m.SetElement(i, j, r[i, j])
        m.SetElement(i, 3, t[i])
    return m


renderer = vtk.vtkRenderer()
renderer.SetBackground(0x0e / 255, 0x14 / 255, 0x20 / 255)
render_window = vtk.vtkRenderWindow()
render_window.SetOffScreenRendering(1)
render_window.AddRenderer(renderer)
render_window.SetSize(1100, 850)
render_window.SetMultiSamples(8)

# One actor per mesh piece; remember which link index (into LINK_NAMES)
# drives its transform + its static placement.
mesh_actors = []
for link_idx, link_name in enumerate(LINK_NAMES):
    for mesh_path, place_t, place_r in meshes_by_link.get(link_name, []):
        reader = vtk.vtkOBJReader()
        reader.SetFileName(mesh_path)
        normals = vtk.vtkPolyDataNormals()
        normals.SetInputConnection(reader.GetOutputPort())
        normals.ConsistencyOn()
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(normals.GetOutputPort())
        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetColor(0.62, 0.63, 0.66)
        actor.GetProperty().SetAmbient(0.25)
        actor.GetProperty().SetDiffuse(0.75)
        actor.GetProperty().SetSpecular(0.2)
        actor.GetProperty().SetSpecularPower(12)
        renderer.AddActor(actor)
        mesh_actors.append((link_idx, place_t, place_r, actor))
print(f"loaded {len(mesh_actors)} mesh pieces across {len(LINK_NAMES)} links", file=sys.stderr)

# ---- foot trail + ground grid ----
FOOT_LINK_IDX = {leg: LINK_NAMES.index(f"{leg}_foot") for leg in ("FL", "FR", "RL", "RR")}
FOOT_COLOR = {"FL": (0.22, 0.53, 0.9), "FR": (0.9, 0.62, 0.0), "RL": (0.0, 0.62, 0.45), "RR": (0.84, 0.33, 0.0)}
TRAIL_TICKS = 120

trail_actors = {}
for leg in FOOT_LINK_IDX:
    pts = vtk.vtkPoints()
    lines = vtk.vtkCellArray()
    poly = vtk.vtkPolyData()
    poly.SetPoints(pts)
    poly.SetLines(lines)
    tube = vtk.vtkTubeFilter()
    tube.SetInputData(poly)
    tube.SetRadius(0.004)
    tube.SetNumberOfSides(6)
    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputConnection(tube.GetOutputPort())
    actor = vtk.vtkActor()
    actor.SetMapper(mapper)
    actor.GetProperty().SetColor(*FOOT_COLOR[leg])
    actor.GetProperty().SetOpacity(0.7)
    renderer.AddActor(actor)
    trail_actors[leg] = (pts, lines, poly, actor)


def update_trail(tick, leg):
    pts, lines, poly, _ = trail_actors[leg]
    start = max(0, tick - TRAIL_TICKS)
    sel_ticks = range(start, tick + 1, max(args.stride, 1))
    pts.Reset()
    lines.Reset()
    n = 0
    for tk in sel_ticks:
        t, _ = link_pose(rows[tk], FOOT_LINK_IDX[leg])
        pts.InsertNextPoint(*t)
        n += 1
    if n > 1:
        lines.InsertNextCell(n)
        for i in range(n):
            lines.InsertCellPoint(i)
    pts.Modified()
    poly.Modified()

grid = vtk.vtkPlaneSource()
grid.SetOrigin(-0.3, -0.5, 0.0)
grid.SetPoint1(1.2, -0.5, 0.0)
grid.SetPoint2(-0.3, 0.5, 0.0)
grid.SetXResolution(15)
grid.SetYResolution(10)
grid_mapper = vtk.vtkPolyDataMapper()
grid_mapper.SetInputConnection(grid.GetOutputPort())
grid_actor = vtk.vtkActor()
grid_actor.SetMapper(grid_mapper)
grid_actor.GetProperty().SetRepresentationToWireframe()
grid_actor.GetProperty().SetColor(0.25, 0.3, 0.38)
grid_actor.GetProperty().SetOpacity(0.35)
renderer.AddActor(grid_actor)

key_light = vtk.vtkLight()
key_light.SetPosition(1.0, -1.4, 1.6)
key_light.SetFocalPoint(0.3, 0, 0.15)
key_light.SetIntensity(0.9)
key_light.SetColor(1.0, 1.0, 0.98)
renderer.AddLight(key_light)
fill_light = vtk.vtkLight()
fill_light.SetPosition(-0.8, 1.3, 0.9)
fill_light.SetFocalPoint(0.3, 0, 0.15)
fill_light.SetIntensity(0.35)
fill_light.SetColor(0.75, 0.82, 1.0)
renderer.AddLight(fill_light)

text_actor = vtk.vtkTextActor()
text_actor.SetPosition(20, 800)
text_actor.GetTextProperty().SetFontSize(19)
text_actor.GetTextProperty().SetColor(0.9, 0.92, 0.94)
text_actor.GetTextProperty().SetFontFamilyToCourier()
renderer.AddActor2D(text_actor)

# ---- camera: follow the body's x position, fixed offset ----
base_path = np.array([link_pose(rows[i], 0)[0] for i in range(0, n_frames_total, 20)])
z0 = base_path[:, 2].mean()
elev = np.radians(22)
azim = np.radians(-125)
distance = 1.9
direction = np.array([np.cos(elev) * np.cos(azim), np.cos(elev) * np.sin(azim), np.sin(elev)])
camera = renderer.GetActiveCamera()
camera.SetViewUp(0, 0, 1)
camera.SetViewAngle(38)

w2i = vtk.vtkWindowToImageFilter()
w2i.SetInput(render_window)
w2i.SetInputBufferTypeToRGB()
writer = vtk.vtkPNGWriter()

FRAMES_DIR = Path(args.frames_dir)
FRAMES_DIR.mkdir(exist_ok=True, parents=True)
for f in FRAMES_DIR.glob("*.png"):
    f.unlink()

frame_indices = list(range(0, n_frames_total, args.stride))
print(f"Rendering {len(frame_indices)} frames...", file=sys.stderr)
for fi, tick in enumerate(frame_indices):
    row = rows[tick]
    base_t, _ = link_pose(row, 0)
    focal = np.array([base_t[0] + 0.15, base_t[1], z0 + 0.05])
    camera.SetFocalPoint(*focal)
    camera.SetPosition(*(focal + distance * direction))

    for link_idx, place_t, place_r, actor in mesh_actors:
        link_t, link_r = link_pose(row, link_idx)
        wt, wr = compose(link_t, link_r, place_t, place_r)
        actor.SetUserMatrix(vtk_matrix(wt, wr))

    for leg in FOOT_LINK_IDX:
        update_trail(tick, leg)

    if args.staircase_step_s:
        n_levels = round(args.staircase_max_mps / args.staircase_step_mps) + 1
        level = min(int(row[IDX_T] / args.staircase_step_s), n_levels - 1)
        cmd_vx = level * args.staircase_step_mps
        text_actor.SetInput(f"{args.title}   t = {row[IDX_T]:5.2f}s   cmd_vx = {cmd_vx:.2f} m/s")
    else:
        text_actor.SetInput(f"{args.title}   t = {row[IDX_T]:5.2f}s")

    render_window.Render()
    w2i.Modified()
    w2i.Update()
    writer.SetFileName(str(FRAMES_DIR / f"f{fi:05d}.png"))
    writer.SetInputConnection(w2i.GetOutputPort())
    writer.Write()
    if fi % 50 == 0:
        print(f"  frame {fi}/{len(frame_indices)}", file=sys.stderr)

print("Encoding mp4...", file=sys.stderr)
subprocess.run([
    "ffmpeg", "-y", "-framerate", str(args.fps),
    "-i", str(FRAMES_DIR / "f%05d.png"),
    "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "18",
    str(args.out),
], check=True)
print(f"Done: {args.out}", file=sys.stderr)
