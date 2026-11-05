"""Drift across repeated jumps (JUMP_REPEAT): per tag, JUMP_V and jump index k
(the k-th re-arm), the members still up, and medians at re-arm of the stance
width (sole separation across the mean feet yaw), |com - feet mid| (mm),
|feet yaw| (largest of the two), the feet yaw spread, and the CoM's lateral
position y. Accumulating drift shows as a trend in k.
  python3 rdrift.py TAG [TAG...]"""
import re, glob, sys, math, statistics as st
D = '/home/takara/work/dp/articara/log/claude_g1jump_20261021/runs'
P = re.compile(r'\[jump-again\] t=(\S+) jump (\d+) of (\d+): com \(([-+.\d]+),([-+.\d]+), z \S+\) .*?soles L \(([-+.\d]+),([-+.\d]+)\) R \(([-+.\d]+),([-+.\d]+)\) yaw ([-+.\d]+) ([-+.\d]+) \| com-mid \(([-+.\d]+),([-+.\d]+)\)')
med = lambda v: st.median(v) if v else float('nan')
for tag in sys.argv[1:]:
    rows = {}
    for f in glob.glob(f'{D}/{tag}/v*/m*/run.log'):
        c = f.split('/')[-3]
        for m in P.finditer(open(f).read()):
            k = int(m.group(2)); cy = float(m.group(5))
            lx, ly, rx, ry, yl, yr, dx, dy = map(float, m.group(6, 7, 8, 9, 10, 11, 12, 13))
            h = math.atan2(math.sin(yl) + math.sin(yr), math.cos(yl) + math.cos(yr))
            w = -(lx - rx) * math.sin(h) + (ly - ry) * math.cos(h)
            rows.setdefault((c, k), []).append((w, 1000 * math.hypot(dx, dy), max(abs(yl), abs(yr)), abs(yl - yr), cy))
    for c in sorted({k[0] for k in rows}, key=lambda s: float(s[1:])):
        print(f'{tag} {c}')
        for k in sorted(kk for cc, kk in rows if cc == c):
            r = rows[(c, k)]
            print(f'  re-arm for jump {k:2d}: n {len(r):2d} | width {med([x[0] for x in r]):.3f} | |com-mid| {med([x[1] for x in r]):4.0f} mm'
                  f' | max|yaw| {med([x[2] for x in r]):.2f} | yaw spread {med([x[3] for x in r]):.2f} | com y {med([x[4] for x in r]):+.2f}')
