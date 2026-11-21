#!/usr/bin/env python3
"""Forward hop-running summary (HOP_RUN with HOP_RUN_VX), per TAG:
hops landed in a row (as runhops.py), runs that did all 10 and stood,
lift-off forward speed from the 3rd hop on (before any fall), and the
distance from the first lift-off to the last landing of the runs that stood.
Usage: python3 runspeed.py TAG [TAG...]"""
import glob, re, sys
import numpy as np

for tag in sys.argv[1:]:
    for cond in sorted(glob.glob(f'runs/{tag}/v*')):
        hops, full, vx, dist, speed = [], 0, [], [], []
        for f in sorted(glob.glob(f'{cond}/m*/run.log')):
            t = open(f).read()
            n, armed = 0, False
            for l in t.splitlines():
                if re.search(r'\[run\] t=\S+ lift-off', l):
                    armed = True
                elif armed and re.search(r'\[run\] t=\S+ re-arm', l):
                    n, armed = n + 1, False
            stood = 'verdict: LANDED' in t
            if armed and stood:
                n += 1
            hops.append(n)
            lo = re.findall(r'\[lift-off\] t=(\S+) com \((\S+), z \S+ v \((\S+),', t)
            vx += [float(x[2]) for x in lo[2:min(len(lo), n + 1)]]
            td = re.findall(r'\[touchdown\] t=(\S+) after \S+ s, com \((\S+),', t)
            if stood and n >= 10 and lo and len(td) >= 10:
                full += 1
                d = float(td[9][1]) - float(lo[0][1])
                dist.append(d)
                speed.append(d / (float(td[9][0]) - float(lo[0][0])))
        q = lambda a, p: np.percentile(a, p) if a else float('nan')
        print(f'{tag:10s} {cond.split("/")[-1]} hops {sum(hops):4d} / {10 * len(hops)}  all 10: {full:2d}'
              f' | lift-off vx median {q(vx, 50):+.3f} (q25 {q(vx, 25):+.3f} q75 {q(vx, 75):+.3f})'
              f' | 10 hops: {q(dist, 50):+.2f} m, {q(speed, 50):+.3f} m/s')
