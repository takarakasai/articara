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


WAIST_AXES = {"yaw": "0 0 1", "roll": "1 0 0", "pitch": "0 1 0"}


def add_waist(root, dofs):
    """Split `torso` into a pelvis (legs) and an upper body (arms, head), with
    `dofs` revolute joints between them.

    The pelvis keeps the ORIGINAL torso frame, so every hip joint origin stays
    valid untouched; the upper body gets a frame 50 mm up and its children are
    shifted to match. Split point z = -0.05: below the shoulders (+0.09),
    above the hips (-0.11).

    Mass is divided by height and each half given a uniform-box inertia. That
    is an approximation -- the real torso's CoM sits 21 mm below centre, so it
    is bottom-heavy -- and it does NOT add the actuator mass a real waist would
    carry (0.242 kg per EL05). Both errors flatter the waist, so read a
    positive result as an upper bound.
    """
    Z_SPLIT, H, W, D = -0.05, 0.22, 0.14, 0.10
    h_pel, h_top = Z_SPLIT - (-0.11), 0.11 - Z_SPLIT

    torso = next(l for l in root.findall("link") if l.get("name") == "torso")
    m_tot = float(torso.find("inertial").find("mass").get("value"))
    m_pel, m_top = m_tot * h_pel / H, m_tot * h_top / H

    def box_inertia(m, dx, dy, dz):
        return (m * (dy * dy + dz * dz) / 12.0,
                m * (dx * dx + dz * dz) / 12.0,
                m * (dx * dx + dy * dy) / 12.0)

    # 2 mm off each collision box at the split plane. With a single waist DoF
    # pelvis and torso are parent-child and MuJoCo excludes the pair; with two
    # massless intermediates they are not, and two boxes that share a plane
    # collide. Inertia keeps the full height -- this is a collision-only trim.
    GAP = 0.002

    def inertial(parent, mass, com_z, dz):
        ixx, iyy, izz = box_inertia(mass, W, D, dz)
        i = ET.SubElement(parent, "inertial")
        ET.SubElement(i, "origin", xyz=f"0 0 {com_z:.5f}")
        ET.SubElement(i, "mass", value=f"{mass:.4f}")
        ET.SubElement(i, "inertia", ixx=f"{ixx:.7f}", ixy="0", ixz="0",
                      iyy=f"{iyy:.7f}", iyz="0", izz=f"{izz:.7f}")

    # --- pelvis: new root, keeps the original torso frame ---------------
    pelvis = ET.Element("link", name="pelvis")
    inertial(pelvis, m_pel, (-0.11 + Z_SPLIT) / 2.0, h_pel)
    for tag in ("visual", "collision"):
        e = ET.SubElement(pelvis, tag)
        ET.SubElement(e, "origin", xyz=f"0 0 {(-0.11 + Z_SPLIT) / 2.0:.5f}")
        g = ET.SubElement(e, "geometry")
        ET.SubElement(g, "box",
                      size=f"{W} {D} {h_pel - (GAP if tag == 'collision' else 0.0):.4f}")
        if tag == "visual":
            ET.SubElement(e, "material", name="grey")
    root.insert(list(root).index(torso), pelvis)

    # --- upper body: reuse the `torso` link, reframed 50 mm up ----------
    for tag in ("inertial", "visual", "collision"):
        for e in torso.findall(tag):
            torso.remove(e)
    inertial(torso, m_top, h_top / 2.0, h_top)
    for tag in ("visual", "collision"):
        e = ET.SubElement(torso, tag)
        ET.SubElement(e, "origin", xyz=f"0 0 {h_top / 2.0:.5f}")
        g = ET.SubElement(e, "geometry")
        ET.SubElement(g, "box",
                      size=f"{W} {D} {h_top - (GAP if tag == 'collision' else 0.0):.4f}")
        if tag == "visual":
            ET.SubElement(e, "material", name="grey")

    # --- re-parent: legs stay on the pelvis, arms and head go up --------
    for j in root.findall("joint"):
        if j.find("parent").get("link") != "torso":
            continue
        child = j.find("child").get("link")
        if "hip" in child:
            j.find("parent").set("link", "pelvis")
        else:
            o = j.find("origin")
            xyz = o.get("xyz").split()
            xyz[2] = f"{float(xyz[2]) - Z_SPLIT:.5f}"
            o.set("xyz", " ".join(xyz))

    # --- the waist chain, massless intermediates for multi-DoF ----------
    prev, z = "pelvis", Z_SPLIT
    for k, name in enumerate(dofs):
        last = k == len(dofs) - 1
        child = "torso" if last else f"waist_{name}_link"
        if not last:
            link = ET.Element("link", name=child)
            inertial(link, 1e-4, 0.0, 0.01)
            root.insert(list(root).index(torso), link)
        j = ET.Element("joint", name=f"waist_{name}_joint", type="revolute")
        ET.SubElement(j, "parent", link=prev)
        ET.SubElement(j, "child", link=child)
        ET.SubElement(j, "origin", xyz=f"0 0 {z:.5f}" if k == 0 else "0 0 0",
                      rpy="0 0 0")
        ET.SubElement(j, "axis", xyz=WAIST_AXES[name])
        lim = "0.5236" if name == "yaw" else "0.3491"
        ET.SubElement(j, "limit", lower=f"-{lim}", upper=lim,
                      effort="6.0", velocity="10.0")
        root.append(j)
        prev, z = child, 0.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--shoulder-y", type=float, default=None)
    ap.add_argument("--shoulder-z", type=float, default=None)
    ap.add_argument("--name", required=True)
    ap.add_argument("--hip-y", type=float, default=None)
    ap.add_argument("--waist", default=None,
                    help="comma-separated waist DoF from yaw,roll,pitch (proximal first)")
    ap.add_argument("--ankle-roll-deg", type=float, default=None)
    ap.add_argument("--hip-roll-deg", type=float, default=None)
    a = ap.parse_args()

    tree = ET.parse(SRC)
    root = tree.getroot()
    if a.shoulder_y is not None:
        shoulder_out(root, a.shoulder_y)
    if a.shoulder_z is not None:
        shoulder_up(root, a.shoulder_z)
    if a.waist:
        add_waist(root, [d.strip() for d in a.waist.split(",")])
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
