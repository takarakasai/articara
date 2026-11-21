#!/usr/bin/env python3
"""flights.py TRAJ.csv ...: real flights (sum of the MuJoCo sole fz < 5 N) after t=2.5 s up to the
end of the run or the fall: count, median / p90 duration [ms], and the CoM height range."""
import csv, sys, statistics as st
for f in sys.argv[1:]:
    r = list(csv.DictReader(open(f)))
    segs, t0, zs = [], None, []
    for row in r:
        t = float(row['t'])
        if t < 2.5 or float(row['tilt']) > 0.3:
            continue
        zs.append(float(row['com_z']))
        air = float(row['fz_mj_l']) + float(row['fz_mj_r']) < 5.0
        if air and t0 is None:
            t0 = t
        elif not air and t0 is not None:
            segs.append(t - t0); t0 = None
    d = sorted(1000 * s for s in segs if s > 0.004)
    if d:
        print(f"{f.split('/')[-1]}: flights {len(d)}  median {st.median(d):.0f} ms  p90 {d[int(0.9*(len(d)-1))]:.0f} ms  com_z {min(zs):.3f}..{max(zs):.3f}")
    else:
        print(f"{f.split('/')[-1]}: no flights")
