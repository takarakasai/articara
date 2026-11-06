"""Push / lift-off / landing quantities vs whether that jump was completed
(JUMP_REPEAT runs). One row per push (split at [push-start]); ok = a later
[jump-again] exists, or it is the last push and the run did not fall.
Prints medians per group and the FAIL rate by tercile of each feature, for
jump 1 and for jumps >= 2 separately, and where the failing jumps fell.
  python3 pushfx.py COND TAG [TAG...]"""
import re, glob, sys, statistics as st
D = '/home/takara/work/dp/articara/log/claude_g1jump_20261021/runs'
F = r'([-+.\dNa]+)'
LO = re.compile(r'\[lift-off\] t=(\S+) com \(\S+, z \S+\) v \(' + F + r', ' + F + r'\), foot x \["' + F + r'", "' + F + r'"\] z \["' + F + r'", "' + F + r'"\], h_ang \(' + F + ',' + F + ',' + F + r'\), roll ' + F + ' pitch ' + F)
PU = re.compile(r'\[hx\] push +dt (\S+) dh_x \S+ ground ' + F + ' = fzdiff ' + F + ' cop ' + F + ' com ' + F + r' fy\*h ' + F + r' \| mean fzL-fzR ' + F)
TD = re.compile(r'\[touchdown\] t=(\S+) after (\S+) s, com \(' + F + r', z \S+\) v \(' + F + ', ' + F + r'\), foot x \["' + F + r'", "' + F + r'"\]')
TDF = re.compile(r'\[td-foot\] (\w) first load t=(\S+)')
cond = sys.argv[1]; rows = []; falls = []
for tag in sys.argv[2:]:
    for f in glob.glob(f'{D}/{tag}/{cond}/m*/run.log'):
        t = open(f).read(); fell = 'verdict: FELL' in t
        starts = [m.start() for m in re.finditer(r'\[push-start\]', t)]
        for i, s0 in enumerate(starts):
            seg = t[s0:starts[i + 1] if i + 1 < len(starts) else len(t)]
            last = i == len(starts) - 1
            ok = '[jump-again]' in seg or (last and not fell)
            lo = LO.search(seg); pu = PU.search(seg)
            if not lo or not pu: continue
            tlo = float(lo.group(1))
            vx, vz, fxl, fxr, zl, zr, hx, hy, hz, roll, pitch = map(float, lo.group(*range(2, 13)))
            dt, gr, fzd, cop, com, fyh, dfz = map(float, pu.group(*range(1, 8)))
            r = dict(j=i + 1, ok=ok, hx=abs(hx), hy=hy, hz=abs(hz), roll=abs(roll), pitch=pitch, vx=vx, vz=vz,
                     dfz=abs(dfz), dzlo=abs(zl - zr), push=dt, fzd=abs(fzd), cop=cop, com=com, fyh=abs(fyh))
            td = TD.search(seg, lo.end())
            if td:
                ttd, fl, cx, tvx, tvz, ftl, ftr = map(float, td.group(*range(1, 8)))
                r['flight'] = fl; r['td_cx'] = cx - 0.5 * (ftl + ftr)
                loads = {}
                for m in TDF.finditer(seg, td.start()):
                    loads.setdefault(m.group(1), float(m.group(2)))
                    if len(loads) == 2: break
                r['td_gap'] = abs(loads['L'] - loads['R']) if len(loads) == 2 else 0.3
                if not ok:
                    fm = re.search(r'verdict: FELL', seg)
                    tl = re.findall(r'\n\s*t=\s*([\d.]+)', seg)
                    falls.append(float(tl[-1]) - ttd if tl else float('nan'))
            rows.append(r)
med = lambda a: st.median(a) if a else float('nan')
for name, sel in (('jump 1', lambda r: r['j'] == 1), ('jumps >= 2', lambda r: r['j'] >= 2)):
    R = [r for r in rows if sel(r)]
    if not R: continue
    print(f'{cond} {name}: {len(R)} pushes, {sum(not r["ok"] for r in R)} FAIL')
    for k in ['hx', 'hy', 'hz', 'roll', 'pitch', 'vx', 'vz', 'push', 'dfz', 'fzd', 'fyh', 'cop', 'com', 'dzlo', 'flight', 'td_gap', 'td_cx']:
        S = [r for r in R if k in r]
        v = sorted(r[k] for r in S); q1, q2 = v[len(v) // 3], v[2 * len(v) // 3]
        bins = [[r for r in S if r[k] < q1], [r for r in S if q1 <= r[k] < q2], [r for r in S if r[k] >= q2]]
        fr = ' '.join(f'{sum(not r["ok"] for r in b) / max(1, len(b)):.2f}' for b in bins)
        print(f'  {k:6s} ok med {med([r[k] for r in S if r["ok"]]):+.3f} FAIL med {med([r[k] for r in S if not r["ok"]]):+.3f} | FAIL rate by tercile (cuts {q1:+.3f} {q2:+.3f}): {fr}')
if falls:
    fs = sorted(falls)
    print(f'  last log time after touchdown in FAIL jumps: med {med(fs):.2f} s, q10 {fs[len(fs) // 10]:.2f}, q90 {fs[9 * len(fs) // 10]:.2f}')
