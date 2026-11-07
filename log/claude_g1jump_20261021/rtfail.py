"""Where repeated turning jumps fail: per TAG/JUMP_V, the failed members by
the jump that failed and its phase -- 'push' (a foot off > 2 cm above the
other at lift-off, or no lift-off), 'land' (fell after a normal lift-off).
Also per jump (landed members' first flights): lift-off body-frame h_x
(roll) and h_y, pelvis-feet twist at re-arm.
  python3 rtfail.py TAG..."""
import re, glob, sys, math, statistics as st
D = '/home/takara/work/dp/articara/log/claude_g1jump_20261021/runs'
med = lambda a: st.median(a) if a else float('nan')
for tag in sys.argv[1:]:
    for c in sorted(glob.glob(f'{D}/{tag}/v*')):
        fails = {}; hx = {}; tw = {}
        for f in sorted(glob.glob(f'{c}/m*/run.log')):
            t = open(f).read(); fell = 'verdict: FELL' in t
            segs = re.split(r'\[jump-again\]', t)
            for j, sg in enumerate(segs):
                hd = re.search(r'\[heading\] (\S+)', sg); psi = float(hd.group(1)) if hd else 0.0
                lo = re.search(r'\[lift-off\] .*?z \["(\S+)", "(\S+)"\], h_ang \(([-+.\d]+),([-+.\d]+),([-+.\d]+)\)', sg)
                if lo:
                    za, zb, a, b, _ = map(float, lo.groups())
                    bx = a * math.cos(psi) + b * math.sin(psi); by = -a * math.sin(psi) + b * math.cos(psi)
                    hx.setdefault(j, []).append((bx, by, abs(za - zb) > 0.02))
                y = re.search(r'\[yaw\] t=\S+ nc 2 pelvis (\S+) feet (\S+) (\S+)', sg)
                if j > 0 and y:
                    p, a, b = map(float, y.groups()); tw.setdefault(j, []).append(abs(p - math.atan2(math.sin(a) + math.sin(b), math.cos(a) + math.cos(b))))
            if fell:
                j = len(segs) - 1; sg = segs[-1]
                lo = re.search(r'\[lift-off\] .*?z \["(\S+)", "(\S+)"\]', sg)
                ph = 'nolift' if not lo else ('push' if abs(float(lo.group(1)) - float(lo.group(2))) > 0.02 else 'land')
                fails[(j + 1, ph)] = fails.get((j + 1, ph), 0) + 1
        fs = ' '.join(f'j{k[0]}:{k[1]} {v}' for k, v in sorted(fails.items()))
        hs = ' | '.join(f'j{j+1} roll {med([abs(x[0]) for x in v]):.2f} pitch {med([x[1] for x in v]):+.2f} uneven {sum(x[2] for x in v)}/{len(v)}' + (f' twist {med(tw.get(j, [])):.2f}' if j in tw else '') for j, v in sorted(hx.items()))
        print(f'{tag:6s} {c.split("/")[-1]}: fails {fs} || {hs}')
