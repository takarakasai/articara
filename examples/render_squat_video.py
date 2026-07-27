#!/usr/bin/env python3
"""Replay the kyo46rs squat-WBC trajectory log (from the Rust/articara sim)
by directly setting MuJoCo qpos each frame (kinematic replay -- the physics
already happened in the Rust simulation; this just renders it), and encode
to an mp4 via ffmpeg.
"""
import csv
import subprocess
import numpy as np
import mujoco
import PIL.Image

URDF = "/home/takara/work/dp/humanoid/kyo46rs_description/urdf/kyo46rs.urdf"
CSV_PATH = "/tmp/claude-1000/-home-takara-work/3288d9bb-da13-4665-bfb2-9595dd62f7ab/scratchpad/kyo46rs_squat_traj.csv"
FRAME_DIR = "/tmp/claude-1000/-home-takara-work/3288d9bb-da13-4665-bfb2-9595dd62f7ab/scratchpad/squat_frames"
OUT_MP4 = "/tmp/claude-1000/-home-takara-work/3288d9bb-da13-4665-bfb2-9595dd62f7ab/scratchpad/kyo46rs_squat.mp4"

import os
os.makedirs(FRAME_DIR, exist_ok=True)

with open(CSV_PATH) as f:
    reader = csv.DictReader(f)
    rows = list(reader)
print(f"{len(rows)} frames loaded")

joint_names = [
    "left_hip_yaw_joint", "left_hip_roll_joint", "left_hip_pitch_joint",
    "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_yaw_joint", "right_hip_roll_joint", "right_hip_pitch_joint",
    "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
    "left_shoulder_pitch_joint", "left_elbow_joint",
    "right_shoulder_pitch_joint", "right_elbow_joint",
]

# kyo46rs.urdf compiles with the torso FIXED to the world (MuJoCo's URDF
# importer default) -- fine for a static screenshot, but this replay needs
# the torso to actually move/fall, so add a freejoint via the MjSpec API
# before compiling.
spec = mujoco.MjSpec.from_file(URDF)
spec.body("torso").add_freejoint()
model = spec.compile()
data = mujoco.MjData(model)

free_joint_id = None
for j in range(model.njnt):
    if model.jnt_type[j] == mujoco.mjtJoint.mjJNT_FREE:
        free_joint_id = j
        break
assert free_joint_id is not None, "no free joint found in compiled model"
free_qpos_adr = model.jnt_qposadr[free_joint_id]

joint_qpos_adr = {}
for name in joint_names:
    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
    assert jid >= 0, f"joint {name} not found"
    joint_qpos_adr[name] = model.jnt_qposadr[jid]

renderer = mujoco.Renderer(model, height=480, width=640)
cam = mujoco.MjvCamera()
cam.lookat[:] = [0, 0, 0.35]
cam.distance = 1.3
cam.azimuth = 55
cam.elevation = -12

for i, row in enumerate(rows):
    data.qpos[free_qpos_adr + 0] = float(row["x"])
    data.qpos[free_qpos_adr + 1] = float(row["y"])
    data.qpos[free_qpos_adr + 2] = float(row["z"])
    data.qpos[free_qpos_adr + 3] = float(row["qw"])
    data.qpos[free_qpos_adr + 4] = float(row["qx"])
    data.qpos[free_qpos_adr + 5] = float(row["qy"])
    data.qpos[free_qpos_adr + 6] = float(row["qz"])
    for name in joint_names:
        data.qpos[joint_qpos_adr[name]] = float(row[name])
    mujoco.mj_forward(model, data)

    renderer.update_scene(data, camera=cam)
    scene = renderer.scene
    gi = scene.ngeom
    mujoco.mjv_initGeom(
        scene.geoms[gi], type=mujoco.mjtGeom.mjGEOM_PLANE,
        size=[1.0, 1.0, 0.02], pos=[0, 0, 0.0], mat=np.eye(3).flatten(),
        rgba=[0.55, 0.57, 0.62, 1.0],
    )
    scene.ngeom = gi + 1

    img = renderer.render()
    PIL.Image.fromarray(img).save(f"{FRAME_DIR}/frame_{i:04d}.png")

print(f"rendered {len(rows)} frames to {FRAME_DIR}")

# Assemble to mp4 (input frame rate matches the log's own dt so playback
# speed matches real elapsed sim time).
dt = float(rows[1]["t"]) - float(rows[0]["t"])
fps = 1.0 / dt
subprocess.run([
    "ffmpeg", "-y", "-framerate", f"{fps:.3f}",
    "-i", f"{FRAME_DIR}/frame_%04d.png",
    "-vf", "scale=640:480",
    "-pix_fmt", "yuv420p",
    OUT_MP4,
], check=True)
print(f"saved {OUT_MP4}")
