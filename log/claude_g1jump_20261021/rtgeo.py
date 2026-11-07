"""Stance geometry at each re-arm of repeated (turning) jumps, in the heading
frame: width across, stagger along (L minus R), toe angle (L yaw minus R
yaw), each foot's yaw minus the heading; split by whether that jump then
failed. Tertiles of each measure with the failure rate.
  python3 rtgeo.py TAG [V]"""
import re, glob, sys, math, statistics as st
D = '/home/takara/work/dp/articara/log/claude_g1jump_20261021/runs'
tag = sys.argv[1]; V = sys.argv[2] if len(sys.argv) > 2 else '*'
rows = []
for f in sorted(glob.glob(f'{D}/{tag}/v{V}/m*/run.log')):
    t = open(f).read(); fell = 'verdict: FELL' in t
    segs = re.split(r'(?=\[jump-again\])', t)
    for j, sg in enumerate(segs[1:], start=2):
        m = re.search(r'soles L \(([-+.\d]+),([-+.\d]+)\) R \(([-+.\d]+),([-+.\d]+)\) yaw (\S+) (\S+) \| com-mid \(([-+.\d]+),([-+.\d]+)\)', sg)
        if not m: continue
        lx, ly, rx, ry, yl, yr, cx, cy = map(float, m.groups())
        psi = math.atan2(math.sin(yl) + math.sin(yr), math.cos(yl) + math.cos(yr))
        dx, dy = lx - rx, ly - ry
        along = dx * math.cos(psi) + dy * math.sin(psi); across = -dx * math.sin(psi) + dy * math.cos(psi)
        cxa = cx * math.cos(psi) + cy * math.sin(psi); cya = -cx * math.sin(psi) + cy * math.cos(psi)
        failed = fell and j == len(segs)
        rows.append(dict(j=j, fail=failed, width=across, stagger=along, toe=yl - yr, com_along=cxa, com_across=cya))
print(f'{tag} v{V}: {len(rows)} re-arms, {sum(r["fail"] for r in rows)} then failed')
for k in ['width', 'stagger', 'toe', 'com_along', 'com_across']:
    xs = sorted(rows, key=lambda r: r[k]); n = len(xs)
    parts = [xs[:n // 3], xs[n // 3:2 * n // 3], xs[2 * n // 3:]]
    s = ' | '.join(f'{p[0][k]:+.3f}..{p[-1][k]:+.3f}: {sum(r["fail"] for r in p)}/{len(p)}' for p in parts)
    print(f'  {k:10s} {s}')
