#!/usr/bin/env python3
"""rbcmp.py DIR: for each condition of a wr05.sh run, survivors, paired churn against nom
(up / down / z) and the fall-time split (< 10 s = while running, >= 10 s = at / after the stop)."""
import math, re, sys, pathlib
D = pathlib.Path(sys.argv[1])
def res(c):
    out = {}
    for f in sorted((D / c).glob('st*_s*.log')):
        t = f.read_text(errors='replace')
        if 'verdict: SURVIVED' in t: out[f.name] = None
        else:
            m = re.search(r'(?:FELL|fell)[^\n]*?t\s*=\s*([0-9.]+)', t) or re.search(r'fall[^\n]*?([0-9]+\.[0-9]+)\s*s', t)
            out[f.name] = float(m.group(1)) if m else -1.0
    return out
base = res('nom')
for c in sorted(p.name for p in D.iterdir() if p.is_dir()):
    r = res(c); n = len(r); s = sum(v is None for v in r.values())
    up = sum(1 for k, v in r.items() if v is None and base.get(k, 0) is not None)
    dn = sum(1 for k, v in r.items() if v is not None and base.get(k, 1) is None)
    z = (up - dn) / math.sqrt(up + dn) if up + dn else 0.0
    run = sum(1 for v in r.values() if v is not None and 0 <= v < 10); stop = sum(1 for v in r.values() if v is not None and v >= 10)
    unk = sum(1 for v in r.values() if v is not None and v < 0)
    print(f"{c:8s} {s:2d}/{n}  +{up}/-{dn} z {z:+.2f}  falls run {run} stop {stop}" + (f" ?{unk}" if unk else ""))
