"""Repeated-jump summary (JUMP_REPEAT): per tag and JUMP_V, how many of the
jumps each member completed (a jump is complete when the robot is still up at
the next re-arm, or at the end of the run for the last one), flights, and the
feet's yaw at each re-arm. With two tags, paired churn on the completed count.
  python3 rsumm.py TAG [TAG2]"""
import re, glob, sys, json, statistics as st, math
D = '/home/takara/work/dp/articara/log/claude_g1jump_20261021/runs'
def load(tag):
    out = {}
    for f in glob.glob(f'{D}/{tag}/v*/m*/run.log'):
        t = open(f).read(); c = f.split('/')[-3]; m = f.split('/')[-2]
        again = len(re.findall(r'\[jump-again\]', t))
        fell = 'verdict: FELL' in t
        rep = re.search(r'repeat: (\d+) jumps started, (\d+) flights', t)
        yaws = [abs(float(a)) for y in re.findall(r'yaw ([-+0-9.]+) ([-+0-9.]+) \|', t) for a in y]
        dx = re.search(r'final CoM dx (\S+)', t)
        nj = 1 + int(float(json.load(open(f[:-7] + 'env.json')).get('JUMP_REPEAT', '0')))
        out[(c, m)] = dict(done=again + (0 if fell else 1), fell=fell, nj=nj, yaw=max(yaws) if yaws else None,
                           dx=float(dx.group(1)) if dx else None)
    return out
tags = sys.argv[1:]
R = [load(t) for t in tags]
for c in sorted({k[0] for k in R[0]}, key=lambda s: float(s[1:])):
    for ti, (tag, A) in enumerate(zip(tags, R)):
        ks = sorted(k for k in A if k[0] == c)
        if not ks: continue
        dn = [A[k]['done'] for k in ks]
        hist = [dn.count(i) for i in range(0, max(dn) + 1)]
        yw = [A[k]['yaw'] for k in ks if A[k]['yaw'] is not None]
        dx = [A[k]['dx'] for k in ks if not A[k]['fell'] and A[k]['dx'] is not None]
        s = f"{tag:12s} {c}: jumps done {sum(dn)} of {sum(A[k]['nj'] for k in ks)} | members by jumps done {hist} | max|feet yaw| at re-arm med {st.median(yw) if yw else float('nan'):.2f} | final dx (standing) med {st.median(dx) if dx else float('nan'):+.2f}"
        if ti > 0:
            B = R[0]; kk = [k for k in ks if k in B]
            up = sum(A[k]['done'] > B[k]['done'] for k in kk); down = sum(A[k]['done'] < B[k]['done'] for k in kk)
            z = (up - down) / math.sqrt(up + down) if up + down else 0
            s += f" | vs {tags[0]} +{up}/-{down} z {z:+.2f}"
        print(s)
