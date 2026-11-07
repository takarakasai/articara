"""Turning jumps (JUMP_TURN_HZ): per TAG and JUMP_V, landed count and medians of
lift-off h_z and |h_x|, pelvis yaw turned by lift-off / touchdown / final
(from the ramp start), the feet's mean yaw at touchdown and the final feet
yaw from the last [yaw] line (both from the ramp start's pelvis yaw).
  python3 tsumm.py TAG [TAG...]"""
import re, glob, sys, math, statistics as st
D = '/home/takara/work/dp/articara/log/claude_g1jump_20261021/runs'
med = lambda a: st.median(a) if a else float('nan')
for tag in sys.argv[1:]:
    for c in sorted(glob.glob(f'{D}/{tag}/v*')):
        R = []
        for f in sorted(glob.glob(f'{c}/m*/run.log')):
            t = open(f).read()
            ok = 'verdict: LANDED' in t
            lo = re.search(r'\[lift-off\] .*?h_ang \(([-+.\d]+),([-+.\d]+),([-+.\d]+)\)', t)
            fin = re.search(r'turn: pelvis yaw push start (\S+), lift-off (\S+), touchdown (\S+), final (\S+)', t)
            td = re.search(r'\[turn\] touchdown .*?feet yaw \["(\S+)", "(\S+)"\]', t)
            yl = re.findall(r'\[yaw\] t=\S+ nc \d pelvis \S+ feet (\S+) (\S+)', t)
            if not lo or not fin: continue
            y0, ylo, ytd, yfi = (float(x.rstrip(',')) for x in fin.groups())
            r = dict(ok=ok, hz=float(lo.group(3)), hx=abs(float(lo.group(1))), lo=ylo - y0, td=ytd - y0, fi=yfi - y0)
            if td:
                a, b = float(td.group(1)), float(td.group(2)); r['ftd'] = math.atan2(math.sin(a) + math.sin(b), math.cos(a) + math.cos(b)) - y0
            if yl:
                a, b = map(float, yl[-1]); r['ffi'] = math.atan2(math.sin(a) + math.sin(b), math.cos(a) + math.cos(b)) - y0
            R.append(r)
        if not R: continue
        L = [r for r in R if r['ok']]
        g = lambda k, S: med([r[k] for r in S if k in r])
        print(f'{tag:10s} {c.split("/")[-1]}: landed {len(L)}/{len(R)} | lift-off h_z {g("hz", R):+.2f} |h_x| {g("hx", R):.2f} | '
              f'pelvis turned: lift-off {g("lo", R):+.2f} touchdown {g("td", R):+.2f} final {g("fi", R):+.2f} | feet: touchdown {g("ftd", R):+.2f} final {g("ffi", R):+.2f} '
              f'| landed only: final pelvis {g("fi", L):+.2f} feet {g("ffi", L):+.2f}')
