#!/usr/bin/env python3
# 位置を狙う跳躍の集計: fsumm.py TAG... — 各 TAG・各 JUMP_V で、立って終えた本、足裏の中点の前進（中央値・sd・範囲）、狙いがあれば ±5／±10 cm 以内の本、離地の vx（1 回目）、跳んだ回数
import re, sys, os, glob, statistics as st
R = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'runs')
for tag in sys.argv[1:]:
    for vd in sorted(glob.glob(f'{R}/{tag}/v*')):
        ok, d, e, vx, fell, nj = [], [], [], [], 0, {}
        for m in sorted(glob.glob(f'{vd}/m*')):
            try:
                t = open(f'{m}/run.log').read()
            except OSError:
                continue
            if 'FELL at' in t:
                fell += 1
                continue
            x = re.search(r'soles moved (\S+) m at the end(?: \(error (\S+)\))?', t)
            if not x:
                continue
            ok.append(m); d.append(float(x.group(1)))
            if x.group(2): e.append(float(x.group(2).rstrip(')')))
            j = re.search(r', (\d+) jumps, reached', t)
            if j: nj[int(j.group(1))] = nj.get(int(j.group(1)), 0) + 1
            y = re.search(r'forward: lift-off vx (\S+),', t)
            if y: vx.append(float(y.group(1).rstrip(',')))
        n = len(ok) + fell
        if not d:
            print(f'{tag:9s} {os.path.basename(vd)}: standing 0/{n}'); continue
        sd = st.pstdev(d) if len(d) > 1 else 0.0
        s = f'{tag:9s} {os.path.basename(vd)}: standing {len(d)}/{n} | soles dx med {st.median(d):+.3f} sd {sd:.3f} [{min(d):+.3f},{max(d):+.3f}] | lo vx med {st.median(vx) if vx else float("nan"):+.3f}'
        if e:
            s += f' | err med {st.median(e):+.3f} |err|<=0.05 {sum(abs(a)<=0.05 for a in e)} <=0.10 {sum(abs(a)<=0.10 for a in e)} max {max(abs(a) for a in e):.3f}'
        if nj:
            s += f' | jumps {dict(sorted(nj.items()))}'
        print(s)
