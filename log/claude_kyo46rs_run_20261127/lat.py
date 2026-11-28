#!/usr/bin/env python3
"""lat.py CSV...: per-stance lateral table for walker runs (support 1 = left stance, 2 = right stance).
Columns (mm, world y, sign = toward the swing side is +):
  land   = stance sole y at stance start minus the foot's planned goal y (out = away from the body centre)
  trk    = stance sole y at stance start minus the swing target y at the end of the previous stance (tracking error, out +)
  goal   = goal y minus the nominal (initial) y of that foot (out +)
  xi0/xi1 = (xi_y - xi_ref_y) at stance start / end, toward the swing side +
  cop    = mean CoP (MuJoCo, stance foot, foot-local) y, toward the swing side + (inner edge = +30)
  drag   = mean other-foot vertical force during the first 40 ms of the stance (N)
"""
import csv, sys
for fn in sys.argv[1:]:
    r = list(csv.DictReader(open(fn)))
    f = lambda i, k: float(r[i][k])
    nom = {'l': f(0, 'sole_l_y'), 'r': f(0, 'sole_r_y')}
    segs = []; i = 0
    while i < len(r):
        s = r[i]['support']
        j = i
        while j < len(r) and r[j]['support'] == s: j += 1
        if s in ('1.000000', '2.000000') and float(r[i]['t']) >= 2.0: segs.append((s, i, j - 1))
        i = j
    print(f"# {fn}  ({len(segs)} stances, t_end {r[-1]['t']})")
    print(" k   t     st  land   trk  goal   xi0   xi1   cop  drag  vy_end")
    for k, (s, a, b) in enumerate(segs):
        st = 'l' if s == '1.000000' else 'r'; ot = 'r' if st == 'l' else 'l'
        out = 1 if st == 'l' else -1          # world +y is outward for the left foot
        inw = -out                            # toward the swing side
        y0 = f(a, f'sole_{st}_y')
        land = (y0 - f(a, f'goal_{st}_y')) * out * 1e3
        pb = segs[k - 1][2] if k else a          # last tick of the previous stance = this foot's swing target
        tgt = f(pb, 'swing_tgt_y'); trk = (y0 - tgt) * out * 1e3 if k else float('nan')
        goal = (f(a, f'goal_{st}_y') - nom[st]) * out * 1e3
        xi0 = (f(a, 'xi_y') - f(a, 'xi_ref_y')) * inw * 1e3
        xi1 = (f(b, 'xi_y') - f(b, 'xi_ref_y')) * inw * 1e3
        cs = [f(i, f'cop_mj_{st}_y') * inw * 1e3 for i in range(a, b + 1) if f(i, f'fz_mj_{st}') > 3]
        cop = sum(cs) / len(cs) if cs else float('nan')
        dr = [f(i, f'fz_mj_{ot}') for i in range(a, min(a + 8, b + 1))]
        vy = (f(b, 'com_y') - f(max(b - 2, 0), 'com_y')) / (f(b, 't') - f(max(b - 2, 0), 't'))
        print(f"{k:2d} {f(a,'t'):5.3f}  {st}  {land:5.1f} {trk:5.1f} {goal:5.1f} {xi0:5.1f} {xi1:5.1f} {cop:5.1f} {sum(dr)/len(dr):5.1f} {vy:+.2f}")
