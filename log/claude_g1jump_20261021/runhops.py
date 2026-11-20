#!/usr/bin/env python3
"""Hops landed in a row per member of a HOP_RUN battery (§56).

A hop counts when the robot re-arms after it ([run] re-arm following a
[run] lift-off), the last one when the run ends LANDED.
Usage: python3 runhops.py TAG [TAG...]
"""
import glob, re, sys
D = '/home/takara/work/dp/articara/log/claude_g1jump_20261021/runs'
for tag in sys.argv[1:]:
    for vd in sorted(glob.glob(f'{D}/{tag}/v*')):
        hops = []
        for m in sorted(glob.glob(vd + '/m*')):
            t = open(m + '/run.log').read()
            n, armed = 0, False
            for l in t.splitlines():
                if re.search(r'\[run\] t=\S+ lift-off', l):
                    armed = True
                elif armed and re.search(r'\[run\] t=\S+ re-arm', l):
                    n += 1; armed = False
            if armed and 'verdict: LANDED' in t:
                n += 1
            hops.append(n)
        hist = {k: hops.count(k) for k in sorted(set(hops))}
        print(f'{tag:12s} {vd.split("/")[-1]} hops {sum(hops):4d} / {10 * len(hops)}  all 10: {hops.count(10):2d}  >=5: {sum(h >= 5 for h in hops):2d}  {hist}')
