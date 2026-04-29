#!/usr/bin/env python3
"""Analyze a Joint Peaks plot CSV exported from articara.

Reports for each joint:
  - peak / RMS / std of τ
  - peak |q̇|
  - dominant FFT frequency of τ (with attention to Nyquist artefacts)
  - whether τ is sign-alternating every tick (a tell-tale sign of
    discrete-time PD instability or contact-coupled limit-cycle)

Usage:
  python tools/analyze_peaks_log.py log/log.csv
  python tools/analyze_peaks_log.py log/log.csv --joints RL_calf,FL_calf
  python tools/analyze_peaks_log.py log/log.csv --motor MG4005-i10:10

The optional --motor flag prints suggested armature / joint_damping values
derived from a motor data sheet (see motor_specs/ below).
"""

import argparse
import cmath
import csv
import math
import statistics
import sys
from pathlib import Path


# ---------------------------------------------------------------------------
# Motor presets
# ---------------------------------------------------------------------------
# Each entry holds the catalogue values needed to back out armature and
# joint damping. Reflect the rotor inertia by n² and the electrical damping
# Kt²/R by n² for the joint-side numbers.
MOTOR_SPECS = {
    "MG4005-i10": {
        "rotor_inertia_kgm2": 140e-7,   # 140 g·cm² → SI
        "torque_const_Nm_per_A": 0.06,
        "line_resistance_ohm": 1.4,
        "rated_torque_Nm": 1.0,
        "max_torque_Nm": 2.5,
        "max_speed_rpm": 320,
        "default_gear_ratio": 10.0,
    },
}


def load_csv(path):
    with open(path, newline="") as f:
        rdr = csv.reader(f)
        header = next(rdr)
        rows = [list(map(float, row)) for row in rdr]
    return header, rows


def joint_columns(header):
    """Return list of joint names appearing as `tau[NAME]` columns."""
    return [h[4:-1] for h in header if h.startswith("tau[")]


def col_index(header):
    return {h: i for i, h in enumerate(header)}


def fft(x):
    """Naive radix-2 Cooley-Tukey FFT — input length must be a power of 2."""
    n = len(x)
    if n == 1:
        return list(x)
    even = fft(x[0::2])
    odd = fft(x[1::2])
    twid = [cmath.exp(-2j * math.pi * k / n) for k in range(n // 2)]
    return [even[k] + twid[k] * odd[k] for k in range(n // 2)] + [
        even[k] - twid[k] * odd[k] for k in range(n // 2)
    ]


def next_pow2_floor(n):
    p = 1
    while p * 2 <= n:
        p *= 2
    return p


def spectral_peaks(samples, dt, top_k=3, min_separation_hz=10.0):
    """Return up to top_k strongest non-DC peaks as (freq_hz, magnitude)."""
    n = next_pow2_floor(len(samples))
    if n < 32:
        return []
    x = samples[:n]
    mu = sum(x) / n
    x_dc_removed = [v - mu for v in x]
    X = fft(x_dc_removed)
    half = n // 2
    mag = [abs(z) / n for z in X[:half]]
    freqs = [k / (n * dt) for k in range(half)]
    order = sorted(range(1, half), key=lambda i: -mag[i])
    picks = []
    for i in order:
        if all(abs(freqs[i] - freqs[j]) > min_separation_hz for j in picks):
            picks.append(i)
        if len(picks) >= top_k:
            break
    return [(freqs[i], mag[i]) for i in picks]


def sign_flip_rate(values):
    """Fraction of consecutive samples whose signs differ (Nyquist signature)."""
    n = 0
    flips = 0
    for i in range(1, len(values)):
        if values[i - 1] == 0 or values[i] == 0:
            continue
        n += 1
        if (values[i - 1] > 0) != (values[i] > 0):
            flips += 1
    return flips / max(n, 1)


def per_joint_stats(header, rows, joint_names, dt):
    cols = col_index(header)
    out = []
    for jn in joint_names:
        tau_col = cols[f"tau[{jn}]"]
        qv_col = cols[f"qvel[{jn}]"]
        tau = [r[tau_col] for r in rows]
        qv = [r[qv_col] for r in rows]
        tau_max = max(tau)
        tau_min = min(tau)
        tau_pp = tau_max - tau_min
        tau_rms = math.sqrt(sum(t * t for t in tau) / len(tau))
        tau_std = statistics.pstdev(tau)
        qv_max = max(abs(v) for v in qv)
        peaks = spectral_peaks(tau, dt, top_k=3)
        tau_flip = sign_flip_rate(tau)
        qv_flip = sign_flip_rate(qv)
        out.append(
            {
                "joint": jn,
                "tau_max": tau_max,
                "tau_min": tau_min,
                "tau_pp": tau_pp,
                "tau_rms": tau_rms,
                "tau_std": tau_std,
                "qv_max": qv_max,
                "peaks": peaks,
                "tau_flip_rate": tau_flip,
                "qv_flip_rate": qv_flip,
            }
        )
    return out


def print_joint_table(stats, fs):
    nyquist = fs / 2.0
    print(
        f"{'joint':<22} {'τmax':>8} {'τmin':>8} {'τpp':>8} {'τrms':>8} "
        f"{'|q̇|max':>8} {'q̇flip%':>8}  {'top spectral peaks (Hz, magnitude)'}"
    )
    print("-" * 130)
    for s in stats:
        peaks_str = ", ".join(
            f"{f:6.1f}({m:.3f})"
            f"{'!' if abs(f - nyquist) < 1.0 else ''}"
            for f, m in s["peaks"]
        )
        print(
            f"{s['joint']:<22} "
            f"{s['tau_max']:>8.2f} {s['tau_min']:>8.2f} {s['tau_pp']:>8.2f} "
            f"{s['tau_rms']:>8.3f} {s['qv_max']:>8.2f} "
            f"{s['qv_flip_rate']*100:>7.1f}%  {peaks_str}"
        )
    print()
    print("  (! = peak at the Nyquist frequency = numerical artefact, not physical")
    print("   resonance. q̇flip% close to 50% with a Nyquist spectral peak confirms")
    print("   a 2-tick limit cycle. τflip% can stay low when τ is saturated and")
    print("   pinned to one rail for many ticks, so q̇flip% is the more reliable cue.)")


def estimate_armature_damping(motor, gear_ratio):
    """Reflect catalogue rotor inertia and Kt²/R to the joint side."""
    n2 = gear_ratio ** 2
    armature = motor["rotor_inertia_kgm2"] * n2
    kt = motor["torque_const_Nm_per_A"]
    r = motor["line_resistance_ohm"]
    damp_motor = (kt * kt) / r       # N·m·s/rad at the motor shaft
    damp_joint = damp_motor * n2
    # Mechanical friction is harder to estimate; report a typical 1–5% of
    # stall torque per rad/s as a rough additional viscous term.
    stall = motor["max_torque_Nm"]
    mech_low = 0.01 * stall * gear_ratio   # very rough
    mech_high = 0.05 * stall * gear_ratio
    return {
        "armature_kgm2": armature,
        "damping_electrical_Nms_per_rad": damp_joint,
        "damping_mechanical_low_Nms_per_rad": mech_low,
        "damping_mechanical_high_Nms_per_rad": mech_high,
        "damping_total_low_Nms_per_rad": damp_joint + mech_low,
        "damping_total_high_Nms_per_rad": damp_joint + mech_high,
    }


def parse_motor_arg(spec):
    """Parse `MG4005-i10:10` → (motor_dict, gear_ratio)."""
    if ":" in spec:
        name, gr_str = spec.split(":", 1)
        gear = float(gr_str)
    else:
        name = spec
        gear = MOTOR_SPECS[name]["default_gear_ratio"]
    if name not in MOTOR_SPECS:
        raise SystemExit(
            f"Unknown motor '{name}'. Known: {list(MOTOR_SPECS.keys())}"
        )
    return MOTOR_SPECS[name], gear, name


def print_motor_recommendation(motor, gear, name):
    est = estimate_armature_damping(motor, gear)
    print(f"=== Motor recommendation: {name} (gear ratio 1:{gear:g}) ===")
    print(f"  Catalogue rotor inertia : {motor['rotor_inertia_kgm2']*1e7:.1f} g·cm²")
    print(f"  Catalogue Kt / R        : {motor['torque_const_Nm_per_A']} N·m/A,"
          f" {motor['line_resistance_ohm']} Ω (line-to-line)")
    print(f"  Catalogue max torque    : {motor['max_torque_Nm']} N·m (motor)")
    print()
    print(f"  → armature              ≈ {est['armature_kgm2']:.3e} kg·m²"
          f"   (= {motor['rotor_inertia_kgm2']*1e7:.0f} g·cm² × {gear:g}²)")
    print(f"  → joint_damping (elec.) ≈ {est['damping_electrical_Nms_per_rad']:.3f} N·m·s/rad"
          f"   (= Kt²/R × {gear:g}²)")
    print(f"  → mechanical friction    ≈ {est['damping_mechanical_low_Nms_per_rad']:.3f} … "
          f"{est['damping_mechanical_high_Nms_per_rad']:.3f} N·m·s/rad (rough; 1–5% τ_max)")
    print(f"  → total joint_damping    ≈ {est['damping_total_low_Nms_per_rad']:.3f} … "
          f"{est['damping_total_high_Nms_per_rad']:.3f} N·m·s/rad")
    print()


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv_path", nargs="?",
                    help="Peaks plot CSV exported from articara")
    ap.add_argument("--joints", help="Comma-separated joint name prefixes to filter "
                                     "(e.g. RL_calf,FL_calf). Default: all.")
    ap.add_argument("--motor",
                    help="Print suggested armature/joint_damping for a motor preset, "
                         "with optional `:gear_ratio` (e.g. MG4005-i10:10).")
    ap.add_argument("--list-motors", action="store_true",
                    help="List known motor presets and exit.")
    args = ap.parse_args()

    if args.list_motors:
        for name, spec in MOTOR_SPECS.items():
            print(f"  {name}: rotor={spec['rotor_inertia_kgm2']*1e7:.0f} g·cm², "
                  f"τmax={spec['max_torque_Nm']} N·m, "
                  f"default gear 1:{spec['default_gear_ratio']:g}")
        return 0

    if not args.csv_path:
        ap.error("csv_path is required (or use --list-motors)")
    csv_path = Path(args.csv_path)
    if not csv_path.exists():
        raise SystemExit(f"CSV not found: {csv_path}")
    header, rows = load_csv(csv_path)
    if len(rows) < 64:
        raise SystemExit("Need at least 64 samples to compute spectra")

    dt = (rows[-1][0] - rows[0][0]) / (len(rows) - 1)
    fs = 1.0 / dt
    print(f"=== {csv_path}  ({len(rows)} samples,"
          f" dt={dt*1000:.3f} ms, fs={fs:.1f} Hz, Nyquist={fs/2:.1f} Hz) ===\n")

    joints = joint_columns(header)
    if args.joints:
        wanted = args.joints.split(",")
        joints = [j for j in joints if any(w in j for w in wanted)]
        if not joints:
            raise SystemExit(f"No joints match filter {args.joints!r}")

    stats = per_joint_stats(header, rows, joints, dt)
    print_joint_table(stats, fs)

    if args.motor:
        motor, gear, name = parse_motor_arg(args.motor)
        print()
        print_motor_recommendation(motor, gear, name)

    # Quick health verdict.
    nyquist = fs / 2.0
    bad = [
        s for s in stats
        if any(abs(f - nyquist) < 1.0 and m > 0.05 for f, m in s["peaks"])
    ]
    if bad:
        print("⚠ Joints with strong Nyquist-frequency content (likely numerical):")
        for s in bad:
            print(f"    {s['joint']:<24} q̇-flip rate={s['qv_flip_rate']*100:.1f}%, "
                  f"|τ| peak-peak={s['tau_pp']:.2f} N·m, "
                  f"|q̇|max={s['qv_max']:.1f} rad/s")
        print()
        print("  → Add per-joint `armature` (rotor inertia reflected by gear²) and")
        print("    `joint_damping` (Kt²/R reflected by gear² + mechanical friction)")
        print("    in MJCF and re-run. The `--motor` flag prints catalog-derived values.")
    else:
        print("No Nyquist artefacts detected — spectral content looks physical.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
