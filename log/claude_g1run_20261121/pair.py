#!/usr/bin/env python3
"""pair.py BASE NEW [conds]: per condition, survivors and paired churn (up/down/z) NEW vs BASE (same file names)."""
import math, sys, pathlib
B, N = pathlib.Path(sys.argv[1]), pathlib.Path(sys.argv[2])
conds = sys.argv[3:] or sorted(p.name for p in N.iterdir() if p.is_dir())
def res(d):
    return {f.name: ('verdict: SURVIVED' in f.read_text()) for f in d.glob('st*_s*.log')}
U = D = 0
for c in conds:
    if not (B / c).is_dir() or not (N / c).is_dir(): continue
    b, n = res(B / c), res(N / c)
    k = sorted(set(b) & set(n))
    up = sum(1 for x in k if n[x] and not b[x]); dn = sum(1 for x in k if b[x] and not n[x])
    U += up; D += dn
    z = (up - dn) / math.sqrt(up + dn) if up + dn else 0.0
    print(f"{c:8s} {sum(b[x] for x in k):2d} -> {sum(n[x] for x in k):2d}/{len(k)}  +{up}/-{dn}  z {z:+.2f}")
z = (U - D) / math.sqrt(U + D) if U + D else 0.0
print(f"total +{U}/-{D} z {z:+.2f}")
