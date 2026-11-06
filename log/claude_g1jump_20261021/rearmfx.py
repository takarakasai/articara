"""Re-arm geometry vs whether that jump was completed (JUMP_REPEAT runs):
for each re-arm (jump k >= 2), the feet's fore-aft stagger and width in the
feet's mean-yaw frame, the yaw spread, com-mid in that frame, and ok / FAIL.
Prints medians per group and the FAIL rate by tercile of each feature.
  python3 rearmfx.py COND TAG [TAG...]"""
import re, glob, sys, math, statistics as st
D = '/home/takara/work/dp/articara/log/claude_g1jump_20261021/runs'
P = re.compile(r'\[jump-again\] t=\S+ jump (\d+) of \d+: .*?soles L \(([-+.\d]+),([-+.\d]+)\) R \(([-+.\d]+),([-+.\d]+)\) yaw ([-+.\d]+) ([-+.\d]+) \| com-mid \(([-+.\d]+),([-+.\d]+)\)')
cond = sys.argv[1]; rows = []
for tag in sys.argv[2:]:
    for f in glob.glob(f'{D}/{tag}/{cond}/m*/run.log'):
        t = open(f).read(); fell = 'verdict: FELL' in t
        ms = list(P.finditer(t))
        for i, m in enumerate(ms):
            lx, ly, rx, ry, yl, yr, dx, dy = map(float, m.group(2, 3, 4, 5, 6, 7, 8, 9))
            h = math.atan2(math.sin(yl) + math.sin(yr), math.cos(yl) + math.cos(yr))
            c, s = math.cos(h), math.sin(h)
            stag = (lx - rx) * c + (ly - ry) * s          # +: left ahead
            wid = -(lx - rx) * s + (ly - ry) * c
            cx = dx * c + dy * s; cy = -dx * s + dy * c
            ok = i < len(ms) - 1 or not fell
            rows.append(dict(ok=ok, stag=abs(stag), wid=wid, yspr=abs(yl - yr), cx=cx, cy=abs(cy), yaw=abs(h)))
print(f'{cond}: {len(rows)} re-arms, {sum(not r["ok"] for r in rows)} FAIL')
for k in ['stag', 'wid', 'yspr', 'yaw', 'cx', 'cy']:
    v = sorted(r[k] for r in rows); q1, q2 = v[len(v) // 3], v[2 * len(v) // 3]
    bins = [[r for r in rows if r[k] < q1], [r for r in rows if q1 <= r[k] < q2], [r for r in rows if r[k] >= q2]]
    fr = ' '.join(f'{sum(not r["ok"] for r in b) / max(1, len(b)):.2f}' for b in bins)
    print(f'  {k:5s} ok med {st.median(r[k] for r in rows if r["ok"]):+.3f} FAIL med {st.median(r[k] for r in rows if not r["ok"]):+.3f} | FAIL rate by tercile (cuts {q1:+.3f} {q2:+.3f}): {fr}')
