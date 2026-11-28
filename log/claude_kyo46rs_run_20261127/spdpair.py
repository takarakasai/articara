#!/usr/bin/env python3
"""spdpair.py A B: per condition of two kr05.sh output dirs, the runs whose achieved speed (the run log's '(N%)') rose / fell
by more than 10 points from A to B (fallen runs count as -100 %), and the median change."""
import re, sys, pathlib, statistics as st
def sp(f):
    t = f.read_text(errors='replace')
    if 'verdict: SURVIVED' not in t: return -100
    m = re.search(r'achieved after the ramp.*?\((-?\d+)%\)', t); return int(m.group(1)) if m else -100
A, B = map(pathlib.Path, sys.argv[1:3]); up = dn = 0
for c in sorted(p.name for p in A.iterdir() if p.is_dir()):
    d = [sp(B / c / f.name) - sp(f) for f in (A / c).glob('st*_s*.log') if (B / c / f.name).exists()]
    u, w = sum(x > 10 for x in d), sum(x < -10 for x in d); up += u; dn += w
    if u or w: print(f"{c:<8} +{u}/-{w}  median change {st.median(d):+.0f}")
print(f"total +{up}/-{dn}")
