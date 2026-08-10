#!/usr/bin/env python3
"""v8 = v7 (arm actuators) + the Sec.34/36/38 findings from the guard/jab thread.

v7 swapped the arm actuators and nothing else; it does not carry any of the
control-side thread's conclusions, and its elbow cannot do the jab. Five
changes, each traceable:

  1  shoulder roll -> `fixed`                     Sec.31.4 A
  2  torso 0.750 -> 1.274 kg                      Sec.31.1 B  (v5 removed mass,
                                                   not torque; must land WITH 1)
  3  sole 38 -> 60 mm wide                        Sec.33.5 F'
  4  elbow range mirrored to flex FORWARD         Sec.36      (without it there
                                                   is no boxing guard and the
                                                   jab seed is out of range)
  5  elbow XM335-T323-T -> XM430-W210-T           handover Sec.5-2, now measured

(5) is the one the handover left open. Measured with a jab that actually fires
(the handover's reproduction used kyo46rs_v6_rec, whose elbow range excludes
the jab's own seed, so the elbow moved 0.5 deg and the torque was the arm
hanging): peak elbow torque is 1.047 N*m at the shipped 0.70 s period, against
XM335-T323-T's 1.03 N*m STALL. 102% of stall, and stall is not a continuous
rating. XM430-W210-T's 2.3 N*m puts the same demand at 46%.

(5) forces a packaging change v7 did not have to make. XM430-W210-T is 34 mm
across and the forearm's channel is 35 mm outside / 33 mm inside at 1 mm wall,
so it does not fit; the channel goes to 40 mm, the same section the upper arm
already uses for this motor. **That breaks v7's "collision byte-identical to
v6" property on two links.** Stated rather than hidden: self-collision and
clearance numbers measured against v6/v7 do not carry over to the forearm.

Inertia follows v7's own method, which this script re-derives from v7's
numbers before using it: motor as a solid box at the joint origin, structure as
a solid box of the channel's outside dimensions centred at -0.0605, summed
through the parallel-axis theorem about the combined CoM.
"""
import re
import sys

SRC = "/home/takara/work/dp/humanoid/kyo46rs_description/urdf/kyo46rs_v7.urdf"
OUT = sys.argv[1] if len(sys.argv) > 1 else "/tmp/kyo46rs_vy_regression/urdf/kyo46rs_v8.urdf"


def box_I(m, x, y, z):
    return (m * (y * y + z * z) / 12, m * (x * x + z * z) / 12, m * (x * x + y * y) / 12)


def two_body(m_mot, mot_dims, m_str, str_dims, z_str):
    """Motor at z=0, structure centred at z_str. Returns (com_z, ixx, iyy, izz)."""
    com = (m_mot * 0.0 + m_str * z_str) / (m_mot + m_str)
    im, is_ = box_I(m_mot, *mot_dims), box_I(m_str, *str_dims)
    d_m, d_s = abs(0.0 - com), abs(z_str - com)
    ixx = im[0] + m_mot * d_m ** 2 + is_[0] + m_str * d_s ** 2
    iyy = im[1] + m_mot * d_m ** 2 + is_[1] + m_str * d_s ** 2
    izz = im[2] + is_[2]                      # displacement is along z
    return com, ixx, iyy, izz


# --- self-check: reproduce v7's forearm from its own stated parts ------------
c, ixx, iyy, izz = two_body(0.027, (0.019, 0.022, 0.035),
                            0.080, (0.035, 0.035, 0.121), -0.0605)
assert abs(c - (-0.04523)) < 5e-5, c
assert abs(ixx - 0.0001835) < 2e-6 and abs(izz - 0.0000182) < 2e-6, (ixx, izz)
print(f"self-check on v7 forearm: com {c:+.5f} ixx {ixx:.7f} izz {izz:.7f}  -- matches v7")

# --- v8 forearm: XM430-W210-T in a 40 mm channel -----------------------------
M_STRUCT = round(0.080 * 40 / 35, 4)          # channel mass scales with width
c8, ixx8, iyy8, izz8 = two_body(0.082, (0.0285, 0.034, 0.0465),
                                M_STRUCT, (0.040, 0.040, 0.121), -0.0605)
m8 = round(0.082 + M_STRUCT, 4)
print(f"v8 forearm: {m8:.4f} kg (was 0.1070), com {c8:+.5f}, "
      f"ixx {ixx8:.7f} iyy {iyy8:.7f} izz {izz8:.7f}")

s = open(SRC).read()
n = dict.fromkeys("1 2 3 4 5a 5b 5c".split(), 0)

# 1 -- shoulder roll fixed
for side in ("left", "right"):
    s, k = re.subn(f'(<joint name="{side}_shoulder_roll_joint" type=")revolute(">)',
                   r"\1fixed\2", s)
    n["1"] += k

# 2 -- torso mass back (the v6 two-booster figure)
s, n["2"] = re.subn(
    r'<mass value="0\.7500"/><inertia ixx="[\d.]+" ixy="0" ixz="0" '
    r'iyy="[\d.]+" iyz="0" izz="[\d.]+"/>',
    '<mass value="1.2740"/><inertia ixx="0.0058747" ixy="0" ixz="0" '
    'iyy="0.0055126" iyz="0" izz="0.0030709"/>', s)

# 3 -- 60 mm sole. v7 draws the foot as a 2 mm bent pan with two side walls, so
# the collider, the pan and the walls' y offsets all have to move together --
# the bench's own note (`kyo46rs_bench.py`) is that the URDF box and the WBC's
# assumed CoP box must never move apart, and the same goes for the drawing.
for old, new, cnt in (
    ('<box size="0.098 0.038 0.012"/>', '<box size="0.098 0.06 0.012"/>', 2),   # collider
    ('<box size="0.098 0.038 0.002"/>', '<box size="0.098 0.06 0.002"/>', 2),   # pan
    ('<origin xyz="0 0.018 -0.027"/>',  '<origin xyz="0 0.029 -0.027"/>',  2),  # +y wall
    ('<origin xyz="0 -0.018 -0.027"/>', '<origin xyz="0 -0.029 -0.027"/>', 2),  # -y wall
):
    k = s.count(old)
    assert k == cnt, f"sole: found {k} of {old}, expected {cnt}"
    s = s.replace(old, new)
    n["3"] += k

# 4 + 5 -- elbow flexes forward, and is an XM430
s, n["4"] = re.subn(r'<limit lower="0" upper="2\.2689" effort="1\.03" velocity="5\.55"/>',
                    '<limit lower="-2.2689" upper="0" effort="2.3" velocity="7.33"/>', s)
# 5a motor visual box, 5b channel walls 35 -> 40, 5c inertial
s, n["5a"] = re.subn(r'<geometry><box size="0\.019 0\.022 0\.035"/></geometry>',
                     '<geometry><box size="0.0285 0.034 0.0465"/></geometry>', s)
# The two forearms are MIRRORED -- v7 turned each channel's opening outward so
# a 1 mm wall is distinguishable from solid stock (its sec.4-5) -- so the wall
# offsets differ in sign and a single literal replace only catches one arm.
# Scope the edit to each forearm block and carry the sign through.
for side, sgn in (("left", "-"), ("right", "")):
    blk = re.search(r'<link name="%s_forearm_link">.*?</link>' % side, s, re.S).group(0)
    new = blk
    for old_t, new_t in (
        (f'<origin xyz="0 {sgn}0.017 -0.0605"/><geometry><box size="0.035 0.001 0.121"/>',
         f'<origin xyz="0 {sgn}0.0195 -0.0605"/><geometry><box size="0.04 0.001 0.121"/>'),
        (f'<origin xyz="0.017 {sgn}0.0105 -0.0605"/><geometry><box size="0.001 0.014 0.121"/>',
         f'<origin xyz="0.0195 {sgn}0.012 -0.0605"/><geometry><box size="0.001 0.016 0.121"/>'),
        (f'<origin xyz="-0.017 {sgn}0.0105 -0.0605"/><geometry><box size="0.001 0.014 0.121"/>',
         f'<origin xyz="-0.0195 {sgn}0.012 -0.0605"/><geometry><box size="0.001 0.016 0.121"/>'),
        ('<box size="0.035 0.035 0.121"/>', '<box size="0.04 0.04 0.121"/>'),
    ):
        assert new.count(old_t) == 1, f"{side} forearm: {old_t}"
        new = new.replace(old_t, new_t)
        n["5b"] += 1
    s = s.replace(blk, new)
s, n["5c"] = re.subn(
    r'<inertial><origin xyz="0.00000 0.00000 -0\.04523"/><mass value="0\.1070"/>'
    r'<inertia ixx="[\d.]+" ixy="0" ixz="0" iyy="[\d.]+" iyz="0" izz="[\d.]+"/></inertial>',
    f'<inertial><origin xyz="0.00000 0.00000 {c8:.5f}"/><mass value="{m8:.4f}"/>'
    f'<inertia ixx="{ixx8:.7f}" ixy="0" ixz="0" iyy="{iyy8:.7f}" iyz="0" '
    f'izz="{izz8:.7f}"/></inertial>', s)

want = {"1": 2, "2": 1, "3": 8, "4": 2, "5a": 2, "5b": 8, "5c": 2}
for k, v in want.items():
    assert n[k] == v, f"change {k}: patched {n[k]}, expected {v}"
open(OUT, "w").write(s)
tot = sum(float(x) for x in re.findall(r'<mass value="([\d.]+)"', s))
print(f"\n{OUT}\n  substitutions {n}\n  total mass {tot:.3f} kg (v7 5.418, v6 6.338)")
print(f"  revolute joints {s.count('type=\"revolute\"')} (v7 18 -> v8 16 with the roll axes fixed)")
