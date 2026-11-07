"""Repeated turning jumps (JUMP_REPEAT + JUMP_TURN_HZ): per TAG and JUMP_V,
jumps completed (as rsumm.py), and the soles' mean yaw unwrapped over the
[yaw] lines: total turned at the end (standing members only) and per jump
(at each re-arm). With two tags, paired churn on the completed count.
  python3 rtsumm.py TAG [TAG...]"""
import re, glob, sys, json, math, statistics as st
D = '/home/takara/work/dp/articara/log/claude_g1jump_20261021/runs'
med = lambda a: st.median(a) if a else float('nan')
def load(tag):
    out = {}
    for f in glob.glob(f'{D}/{tag}/v*/m*/run.log'):
        t = open(f).read(); c = f.split('/')[-3]; m = f.split('/')[-2]
        again = len(re.findall(r'\[jump-again\]', t)); fell = 'verdict: FELL' in t
        nj = 1 + int(float(json.load(open(f[:-7] + 'env.json')).get('JUMP_REPEAT', '0')))
        # feet mean yaw, unwrapped, from [yaw] lines; re-arm marks in between
        ys, marks, prev, acc = [], [], None, 0.0
        for ln in t.splitlines():
            if '[jump-again]' in ln: marks.append(len(ys))
            mm = re.search(r'\[yaw\] t=\S+ nc \d pelvis \S+ feet (\S+) (\S+)', ln)
            if mm:
                a, b = map(float, mm.groups()); y = math.atan2(math.sin(a) + math.sin(b), math.cos(a) + math.cos(b))
                if prev is not None: acc += (y - prev + math.pi) % (2 * math.pi) - math.pi
                prev = y; ys.append(acc)
        rearm = [ys[i - 1] if i > 0 else 0.0 for i in marks]
        out[(c, m)] = dict(done=again + (0 if fell else 1), fell=fell, nj=nj, fin=ys[-1] if ys else None, rearm=rearm)
    return out
tags = sys.argv[1:]; R = [load(t) for t in tags]
for c in sorted({k[0] for k in R[0]}, key=lambda s: float(s[1:])):
    for ti, (tag, A) in enumerate(zip(tags, R)):
        ks = sorted(k for k in A if k[0] == c)
        if not ks: continue
        dn = [A[k]['done'] for k in ks]; nj = sum(A[k]['nj'] for k in ks)
        allj = sum(A[k]['done'] == A[k]['nj'] for k in ks)
        fin = [A[k]['fin'] for k in ks if not A[k]['fell'] and A[k]['fin'] is not None]
        nre = max((len(A[k]['rearm']) for k in ks), default=0)
        per = [med([A[k]['rearm'][i] for k in ks if len(A[k]['rearm']) > i]) for i in range(nre)]
        s = (f"{tag:10s} {c}: jumps done {sum(dn)} of {nj}, all {allj}/{len(ks)} | feet turned at re-arm "
             + ' '.join(f'{math.degrees(p):+.0f}' for p in per)
             + f" | final (standing) med {math.degrees(med(fin)):+.0f} deg min {math.degrees(min(fin, key=abs)) if fin else float('nan'):+.0f}")
        if ti > 0:
            B = R[0]; kk = [k for k in ks if k in B]
            up = sum(A[k]['done'] > B[k]['done'] for k in kk); down = sum(A[k]['done'] < B[k]['done'] for k in kk)
            z = (up - down) / math.sqrt(up + down) if up + down else 0
            s += f" | vs {tags[0]} +{up}/-{down} z {z:+.2f}"
        print(s)
