#!/usr/bin/env python3
"""Rewrite namiashi's link masses to a measured total, keeping geometry fixed.

The shipped `namiashi.misa` was converted from a CAD export that totals
2.400 kg, with 0.869 kg of that in the legs (36%). The built robot weighs
3.3 kg, and each leg is 600 g including its motors -- so the legs are 2.4 kg
(73%) and everything else is 0.9 kg. That is not a small correction: it
roughly inverts where the mass lives.

Geometry is unchanged, so for a link whose mass scales by `f` the inertia
tensor scales by `f` too (I = integral r^2 dm, and r is untouched). Scaling
mass without scaling inertia would leave a link that weighs three times more
but resists rotation the same, which is not any physical object.

Within a leg the 600 g has to go somewhere, and the spec does not say where.
The two variants below bracket the plausible range:

  hip   -- all the added mass at the hip link, where a quadruped's abduction
           and hip motors normally sit. Leg inertia about the hip stays low,
           so the swing leg is cheap to move and the trunk sees most of it.
  prop  -- every leg link scaled by the same factor, i.e. the added mass
           follows the CAD distribution. Puts mass out at the thigh and calf,
           which is the pessimistic case for swing cost and for the massless
           -leg assumption in the SRBD MPC.

Which one is right is a fact about the hardware. Which one *matters* is a
question the simulator can answer, so both get built and measured.
"""

import re
import sys
from pathlib import Path

HERE = Path(__file__).parent
SRC = HERE / "namiashi.misa"

TOTAL_KG = 3.3
PER_LEG_KG = 0.600
LEG_PREFIXES = ("FL_", "FR_", "RL_", "RR_")
INERTIA_KEYS = ("ixx", "iyy", "izz", "ixy", "ixz", "iyz")


def link_blocks(text):
    """Yield (name, start, end) for each [[link]] block."""
    starts = [m.start() for m in re.finditer(r"^\[\[link\]\]", text, re.M)]
    for i, st in enumerate(starts):
        en = starts[i + 1] if i + 1 < len(starts) else len(text)
        name = re.search(r'^name = "([^"]+)"', text[st:en], re.M)
        yield (name.group(1) if name else "", st, en)


def mass_of(block):
    m = re.search(r"^mass = ([0-9.eE+-]+)", block, re.M)
    return float(m.group(1)) if m else None


def scale_block(block, f):
    """Scale a link's mass and inertia by `f`, leaving origin/geometry alone."""
    if f == 1.0:
        return block

    def sub_mass(m):
        return f"mass = {float(m.group(1)) * f:.6g}"

    out = re.sub(r"^mass = ([0-9.eE+-]+)", sub_mass, block, count=1, flags=re.M)
    for key in INERTIA_KEYS:
        def sub_i(m, k=key):
            return f"{k} = {float(m.group(1)) * f:.6g}"

        out = re.sub(
            rf"^{key} = ([0-9.eE+-]+)", sub_i, out, count=1, flags=re.M
        )
    return out


def build(variant):
    text = SRC.read_text()
    blocks = list(link_blocks(text))

    leg = {n: mass_of(text[s:e]) for n, s, e in blocks if n.startswith(LEG_PREFIXES)}
    leg = {n: m for n, m in leg.items() if m is not None}
    body = {
        n: mass_of(text[s:e])
        for n, s, e in blocks
        if not n.startswith(LEG_PREFIXES) and mass_of(text[s:e])
    }

    # One leg's worth, read off the FL_ links.
    fl = {n: m for n, m in leg.items() if n.startswith("FL_")}
    leg_now = sum(fl.values())
    body_now = sum(body.values())
    body_target = TOTAL_KG - 4 * PER_LEG_KG
    if body_target <= 0:
        sys.exit(f"4 x {PER_LEG_KG} kg of legs leaves nothing for a {TOTAL_KG} kg robot")

    f_body = body_target / body_now
    if variant == "prop":
        factors = {suffix: PER_LEG_KG / leg_now for suffix in ("hip", "thigh", "calf", "foot")}
    elif variant == "hip":
        non_hip = sum(m for n, m in fl.items() if not n.endswith("_hip"))
        hip_now = leg_now - non_hip
        factors = {"hip": (PER_LEG_KG - non_hip) / hip_now, "thigh": 1.0, "calf": 1.0, "foot": 1.0}
    else:
        sys.exit(f"unknown variant {variant!r}")

    # Rebuild back-to-front so the offsets stay valid.
    out = text
    for name, st, en in reversed(blocks):
        block = text[st:en]
        if name.startswith(LEG_PREFIXES):
            suffix = name.split("_", 1)[1]
            f = factors.get(suffix)
            if f is None:
                sys.exit(f"leg link {name!r} has no factor -- unexpected link name")
        elif mass_of(block):
            f = f_body
        else:
            continue
        out = out[:st] + scale_block(block, f) + out[en:]

    dst = HERE / f"namiashi_3p3_{variant}.misa"
    dst.write_text(out)

    check = {n: mass_of(out[s:e]) for n, s, e in link_blocks(out) if mass_of(out[s:e])}
    total = sum(check.values())
    legs = sum(m for n, m in check.items() if n.startswith(LEG_PREFIXES))
    print(f"{dst.name}: total={total:.4f} kg  legs={legs:.4f} ({100*legs/total:.0f}%)  "
          f"body={total-legs:.4f}")
    print("   per-leg (FL): " + "  ".join(
        f"{n.split('_',1)[1]}={check[n]:.4f}" for n in sorted(check) if n.startswith("FL_")))
    return dst


if __name__ == "__main__":
    for v in ("hip", "prop"):
        build(v)
