#!/usr/bin/env python3
"""Generate kyo46rs URDF variants for the walking-interference sweep.

The forearm fouls the hip block during the lateral lean that stepping needs
(doc/kyo46rs_biped_wbc.md section 10.4). Measured on the shipped model:

    forearm inner face   y = 0.0975   (shoulder 0.115 - forearm half-width 0.0175)
    hip block outer face y = 0.095    (hip at 0.07 + half-width 0.025)
    clearance                 2.5 mm

and the hip block is a 50 mm SQUARE, so rotating it about hip_yaw or hip_roll
swings its corner out to half-diagonal 0.0354, i.e. y = 0.1054 -- 8 mm PAST
the forearm. The clearance is negative for any hip rotation worth having.

Variants are written to a directory and pointed at with URDF=<path>.
"""
import argparse, os, re, shutil, sys
import xml.etree.ElementTree as ET

SRC = "/home/takara/work/dp/humanoid/kyo46rs_description/urdf/kyo46rs.urdf"


def joint(root, name):
    for j in root.findall("joint"):
        if j.get("name") == name:
            return j
    raise KeyError(name)


def set_origin_y(root, jname, y):
    j = joint(root, jname)
    o = j.find("origin")
    xyz = o.get("xyz").split()
    xyz[1] = f"{y:.5f}" if float(xyz[1]) >= 0 else f"{-y:.5f}"
    o.set("xyz", " ".join(xyz))


def shoulder_out(root, y):
    """Move both shoulders outboard. Pure origin change: no new DoF, no mass
    change, and the arm's own geometry is untouched."""
    for side, sign in (("left", 1.0), ("right", -1.0)):
        j = joint(root, f"{side}_shoulder_pitch_joint")
        o = j.find("origin")
        xyz = o.get("xyz").split()
        xyz[1] = f"{sign * y:.5f}"
        o.set("xyz", " ".join(xyz))


def shoulder_up(root, z):
    for side in ("left", "right"):
        j = joint(root, f"{side}_shoulder_pitch_joint")
        o = j.find("origin")
        xyz = o.get("xyz").split()
        xyz[2] = f"{z:.5f}"
        o.set("xyz", " ".join(xyz))


def roll_rom(root, ankle_deg=None, hip_deg=None):
    """Widen the frontal-plane travel.

    The lean that puts the CoM over one sole is atan(half_stance / L_leg) and
    ankle_roll must supply it with the sole flat, while hip_roll supplies the
    matching counter-rotation. Measured on the shipped model: a 69.8 mm lean
    costs ankle_roll 0.265 rad (76% of +-20 deg) AND hip_roll -0.252 rad (72%
    of its -20 deg side), so widening either one alone just moves the wall.
    """
    import math
    for side in ("left", "right"):
        if ankle_deg is not None:
            j = joint(root, f"{side}_ankle_roll_joint")
            r = math.radians(ankle_deg)
            j.find("limit").set("lower", f"{-r:.4f}")
            j.find("limit").set("upper", f"{r:.4f}")
        if hip_deg is not None:
            j = joint(root, f"{side}_hip_roll_joint")
            r = math.radians(hip_deg)
            lim = j.find("limit")
            # Keep the generous abduction side; only the adduction side binds.
            lim.set("lower", f"{-r:.4f}")
            lim.set("upper", f"{max(float(lim.get('upper')), r):.4f}")


def stance_width(root, hip_y):
    """Move the whole leg inboard by relocating the hip_yaw origin.

    This is the moment arm of the single-support hip_roll torque, and it is
    also what pushes the hip block out toward the forearm -- so one change
    addresses both walls at once. Measured on the shipped 0.07 m:
    hip_roll demand 4.56 N*m = 76% of the EL05 peak and 253% of its 1.8 N*m
    continuous rating, reached before the robot has taken a single step.
    """
    for side, sign in (("left", 1.0), ("right", -1.0)):
        j = joint(root, f"{side}_hip_yaw_joint")
        o = j.find("origin")
        xyz = o.get("xyz").split()
        xyz[1] = f"{sign * hip_y:.5f}"
        o.set("xyz", " ".join(xyz))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--shoulder-y", type=float, default=None)
    ap.add_argument("--shoulder-z", type=float, default=None)
    ap.add_argument("--name", required=True)
    ap.add_argument("--hip-y", type=float, default=None)
    ap.add_argument("--ankle-roll-deg", type=float, default=None)
    ap.add_argument("--hip-roll-deg", type=float, default=None)
    a = ap.parse_args()

    tree = ET.parse(SRC)
    root = tree.getroot()
    if a.shoulder_y is not None:
        shoulder_out(root, a.shoulder_y)
    if a.shoulder_z is not None:
        shoulder_up(root, a.shoulder_z)
    if a.hip_y is not None:
        stance_width(root, a.hip_y)
    if a.ankle_roll_deg is not None or a.hip_roll_deg is not None:
        roll_rom(root, a.ankle_roll_deg, a.hip_roll_deg)

    os.makedirs(a.out, exist_ok=True)
    path = os.path.join(a.out, f"{a.name}.urdf")
    tree.write(path, encoding="utf-8", xml_declaration=True)
    # Report the clearance the variant actually buys.
    if a.shoulder_y is not None:
        inner = a.shoulder_y - 0.0175
        print(f"{path}: forearm inner face y={inner:.4f}, "
              f"hip block worst-case (rotated) y=0.1054, "
              f"clearance {(inner - 0.1054) * 1e3:+.1f} mm")
    else:
        print(path)


if __name__ == "__main__":
    main()
