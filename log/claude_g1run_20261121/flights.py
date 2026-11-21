#!/usr/bin/env python3
"""flights.py TRAJ.csv ...: real flights (sum of the MuJoCo sole fz < 5 N) after t=2.5 s up to the
end of the run or the fall: count, median / p90 duration [ms], the CoM height range, and the share
of the PLANNED flight time (n_stance 0) that is really airborne. With -s, one summary line over all."""
import csv, sys, statistics as st
summ = sys.argv[1] == '-s'
files = sys.argv[2:] if summ else sys.argv[1:]
alld, pl_all, pa_all = [], 0, 0
for f in files:
    r = list(csv.DictReader(open(f)))
    segs, t0, zs, pl, pa = [], None, [], 0, 0
    for row in r:
        t = float(row['t'])
        if t < 2.5 or float(row['tilt']) > 0.3:
            continue
        zs.append(float(row['com_z']))
        air = float(row['fz_mj_l']) + float(row['fz_mj_r']) < 5.0
        if int(float(row['n_stance'])) == 0 and t < float(r[-1]['t']) - 4.0:
            pl += 1; pa += air
        if air and t0 is None:
            t0 = t
        elif not air and t0 is not None:
            segs.append(t - t0); t0 = None
    d = sorted(1000 * s for s in segs if s > 0.004)
    alld += d; pl_all += pl; pa_all += pa
    if summ:
        continue
    if d:
        print(f"{f.split('/')[-1]}: flights {len(d)}  median {st.median(d):.0f} ms  p90 {d[int(0.9*(len(d)-1))]:.0f} ms  com_z {min(zs):.3f}..{max(zs):.3f}  airborne {pa}/{pl} of planned")
    else:
        print(f"{f.split('/')[-1]}: no flights")
if summ and alld:
    alld.sort()
    print(f"{len(files)} runs: flights {len(alld)}  median {st.median(alld):.0f} ms  p90 {alld[int(0.9*(len(alld)-1))]:.0f} ms  airborne {100*pa_all/max(pl_all,1):.0f}% of planned flight ticks")
