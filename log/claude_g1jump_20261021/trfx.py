"""Per push of a traced JUMP_REPEAT run (TRAJ_CSV next to the log): the contact
set over the push (flips, ticks on one foot, whether a dropped foot came back),
pelvis roll / pitch change, peak tilt, leg joint-ticks at >= 98% of the torque limit, and ok / FAIL
(ok = a later [jump-again], or the last push of a run that did not fall).
  python3 trfx.py DIR          (DIR holds mNN.log and mNN.csv)"""
import re, glob, sys, csv, math, statistics as st
d = sys.argv[1]; rows = []
LEGS = [f'lim_{s}_{j}_joint' for s in ('left', 'right') for j in ('hip_pitch', 'hip_roll', 'hip_yaw', 'knee', 'ankle_pitch', 'ankle_roll')]
def rp(r):
    w, x, y, z = (float(r[k]) for k in ('qw', 'qx', 'qy', 'qz'))
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1, min(1, 2 * (w * y - z * x))))
    return roll, pitch
for lf in sorted(glob.glob(f'{d}/m*.log')):
    t = open(lf).read(); fell = 'verdict: FELL' in t
    ps = []
    for m in re.finditer(r'\[push-start\] t=(\S+)', t):
        p = float(m.group(1))
        if not ps or p - ps[-1][0] > 0.05: ps.append((p, m.start()))
    tr = list(csv.DictReader(open(lf[:-4] + '.csv')))
    for i, (p, pos) in enumerate(ps):
        seg = t[pos:ps[i + 1][1] if i + 1 < len(ps) else len(t)]
        ok = '[jump-again]' in seg or (i == len(ps) - 1 and not fell)
        lo = re.search(r'\[lift-off\] t=(\S+)', seg)
        te = float(lo.group(1)) if lo else p + 0.31
        w = [r for r in tr if p <= float(r['t']) <= te]
        if len(w) < 10: continue
        n = [int(float(r['n_stance'])) for r in w]
        flips = sum(a != b for a, b in zip(n, n[1:]))
        one = sum(x == 1 for x in n) / len(n)
        first_drop = next((k for k, x in enumerate(n) if x < 2), None)
        back = first_drop is not None and any(x == 2 for x in n[first_drop:])
        r0, p0 = rp(w[0]); r1, p1 = rp(w[-1])
        lim = sum(sum(abs(float(r['tau_' + c[4:]])) >= 0.98 * float(r[c]) for c in LEGS) for r in w)
        fzl = [float(r['fz_mj_l']) for r in w]; fzr = [float(r['fz_mj_r']) for r in w]
        rows.append(dict(m=lf.split('/')[-1][:-4], j=i + 1, ok=ok, flips=flips, one=one, drop=(first_drop or len(n)) * 0.002,
                         back=back, droll=r1 - r0, dpitch=p1 - p0, tilt=max(float(r['tilt']) for r in w), lim=lim,
                         imb=abs(sum(fzl) - sum(fzr)) / max(1, sum(fzl) + sum(fzr))))
med = lambda a: st.median(a) if a else float('nan')
for name, sel in (('jump 1', lambda r: r['j'] == 1), ('jumps >= 2', lambda r: r['j'] >= 2)):
    R = [r for r in rows if sel(r)]
    if not R: continue
    print(f'{name}: {len(R)} pushes, {sum(not r["ok"] for r in R)} FAIL')
    for k in ['flips', 'one', 'drop', 'back', 'droll', 'dpitch', 'tilt', 'lim', 'imb']:
        v = sorted(float(r[k]) for r in R); q1, q2 = v[len(v) // 3], v[2 * len(v) // 3]
        bins = [[r for r in R if r[k] < q1], [r for r in R if q1 <= r[k] < q2], [r for r in R if r[k] >= q2]]
        fr = ' '.join(f'{sum(not r["ok"] for r in b) / max(1, len(b)):.2f}({len(b)})' for b in bins)
        print(f'  {k:6s} ok med {med([float(r[k]) for r in R if r["ok"]]):+.3f} FAIL med {med([float(r[k]) for r in R if not r["ok"]]):+.3f} | FAIL by tercile (cuts {q1:+.3f} {q2:+.3f}): {fr}')
    for b in (False, True):
        S = [r for r in R if r['back'] == b]
        print(f'  dropped foot came back = {b}: {len(S)} pushes, FAIL {sum(not r["ok"] for r in S)}')
if '-v' in sys.argv:
    for r in rows:
        print(r)
