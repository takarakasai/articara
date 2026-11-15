#!/usr/bin/env python3
"""One-leg hop battery summary (§51): per TAG and JUMP_V, how many stood,
and where the fallers fell (before the launch / no flight / after landing),
with the lift-off vz and pitch and the first flight time.
Usage: python3 hopsumm.py TAG [TAG ...]   (glob patterns allowed)"""
import glob, re, sys, os
os.chdir(os.path.dirname(os.path.abspath(__file__)))
def one(m):
    if not os.path.exists(m + '/run.log'): return None
    t = open(m + '/run.log', errors='replace').read()
    v = re.search(r'verdict: (\S+)', t)
    if not v: return None
    tl = float(re.search(r'"T_LAUNCH": "(\S+)"', open(m + '/env.json').read()).group(1))
    fl = [(float(a), float(b)) for a, b in re.findall(r'\[touchdown\] t=(\S+) after (\S+) s', t) if float(b) > 0.03]
    lo = re.search(r'\[lift-off\] t=\S+ com \([^)]*\) v \([^,]*, ([^)]*)\).*pitch (\S+)', t)
    ft = re.search(r'FELL at t=(\S+)', t) or re.search(r'fell at t=(\S+)', t)
    S = [l for l in t.splitlines() if l.startswith('  t= ')]
    fall_t = None
    if v.group(1) == 'FELL':
        for l in S:
            z = float(re.search(r'com=\([^,]+,[^,]+,([^)]+)\)', l).group(1))
            if z < 0.45: fall_t = float(l.split()[1]); break
    ph = '-'
    if v.group(1) == 'FELL':
        if fall_t is not None and fall_t < tl: ph = 'pre'
        elif not fl: ph = 'noflight'
        else: ph = 'land'
    return dict(v=v.group(1), ph=ph, fl=fl[0][1] if fl else 0.0, lovz=float(lo.group(1)) if lo and fl else float('nan'), lop=float(lo.group(2)) if lo and fl else float('nan'), nfl=len(fl))
for pat in sys.argv[1:]:
    for tag in sorted(glob.glob('runs/' + pat)):
        out = []
        for vd in sorted(glob.glob(tag + '/v*')):
            R = [r for r in (one(m) for m in sorted(glob.glob(vd + '/m*'))) if r]
            ok = [r for r in R if r['v'] != 'FELL']
            nf = sum(r['v'] == 'NO' or r['v'].startswith('NO') for r in R)
            from collections import Counter
            c = Counter(r['ph'] for r in R if r['v'] == 'FELL')
            fl = [r['fl'] for r in R if r['nfl']]
            out.append(f"{vd.split('/')[-1]} {len(ok)}/{len(R)} (fell {dict(c)}) flight {min(fl, default=0):.2f}-{max(fl, default=0):.2f}")
        print(f"{tag[5:]:10s} " + ' | '.join(out))
