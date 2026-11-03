"""Per jump of a JUMP_REPEAT run: lift-off h_x, its push-phase split, mean
fzL-fzR in the push, the re-arm geometry, and whether that jump was completed.
  python3 hxrep.py TAG"""
import re, glob, sys, statistics as st
D = '/home/takara/work/dp/articara/log/claude_g1jump_20261021/runs'
tag = sys.argv[1]
rows = []
for f in sorted(glob.glob(f'{D}/{tag}/v*/m*/run.log')):
    t = open(f).read(); c = f.split('/')[-3]
    cuts = [0] + [m.start() for m in re.finditer(r'\[jump-again\]', t)] + [len(t)]
    fell = 'verdict: FELL' in t
    for j in range(len(cuts) - 1):
        seg = t[cuts[j]:cuts[j + 1]]
        last = j == len(cuts) - 2
        ok = (not last) or (not fell)
        lo = re.search(r'\[hx\] lift-off h_x (\S+)', seg)
        pu = re.search(r'\[hx\] push +dt \S+ dh_x \S+ ground (\S+) = fzdiff (\S+) cop (\S+) com (\S+) fy\*h (\S+) \| mean fzL-fzR (\S+)', seg)
        cr = re.search(r'\[hx\] crouch +dt \S+ dh_x (\S+)', seg)
        ho = re.search(r'\[hx\] hold +dt \S+ dh_x (\S+)', seg)
        ra = re.search(r'com-mid \(([-+0-9.]+),([-+0-9.]+)\) \| roll ([-+0-9.]+)', seg) if j > 0 else None
        yw = re.search(r'yaw ([-+0-9.]+) ([-+0-9.]+) \|', seg) if j > 0 else None
        if not lo or not pu: continue
        rows.append(dict(c=c, j=j + 1, ok=ok, hx=float(lo.group(1)), push=float(pu.group(1)), fzd=float(pu.group(2)),
                         cop=float(pu.group(3)), com=float(pu.group(4)), fyh=float(pu.group(5)), dfz=float(pu.group(6)),
                         cr=float(cr.group(1)) if cr else 0, ho=float(ho.group(1)) if ho else 0,
                         my=float(ra.group(2)) if ra else 0.0, roll=float(ra.group(3)) if ra else 0.0,
                         ydiff=abs(float(yw.group(1)) - float(yw.group(2))) if yw else 0.0))
med = lambda a: st.median(a) if a else float('nan')
for c in sorted({r['c'] for r in rows}):
    for j in (1, 2, 3):
        for ok in (True, False):
            R = [r for r in rows if r['c'] == c and r['j'] == j and r['ok'] == ok]
            if not R: continue
            a = lambda k: med([abs(r[k]) for r in R])
            print(f"{c} jump {j} {'ok  ' if ok else 'FAIL'} n={len(R):2d} | |h_x| {a('hx'):.2f} (crouch {a('cr'):.2f} hold {a('ho'):.2f} push {a('push'):.2f}) | push: |fzL-fzR| {a('dfz'):5.0f} N, |fzdiff| {a('fzd'):.2f} |cop| {a('cop'):.2f} |com| {a('com'):.2f} |fy*h| {a('fyh'):.2f} | re-arm |com-mid y| {a('my'):.3f} |roll| {a('roll'):.3f} |yaw L-R| {a('ydiff'):.2f}")
