#!/usr/bin/env python3
"""pair.py A B: per-condition paired churn (+up / -down) of two kr05.sh output dirs, and the total z."""
import sys, pathlib, math
A, B = map(pathlib.Path, sys.argv[1:3])
U = Dn = 0
for c in sorted(p.name for p in A.iterdir() if p.is_dir()):
    u = d = 0
    for f in (A / c).glob('st*_s*.log'):
        g = B / c / f.name
        if not g.exists(): continue
        a = 'verdict: SURVIVED' in f.read_text(errors='replace'); b = 'verdict: SURVIVED' in g.read_text(errors='replace')
        u += (not a) and b; d += a and not b
    U += u; Dn += d
    if u or d: print(f"{c}: +{u}/-{d}")
print(f"total +{U}/-{Dn} z {(U-Dn)/math.sqrt(U+Dn) if U+Dn else 0:+.2f}")
