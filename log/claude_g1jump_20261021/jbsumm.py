#!/usr/bin/env python3
# 間合いを揃えて台へ跳ぶ（JUMP_BOX_GAP）の集計: jbsumm.py TAG... — 各 TAG・各 JUMP_V で、立って終えた本、台に載って立った本、跳んだ回数、台へ跳んだときの間合い（中央値・範囲）、倒れた本が何回目の跳躍（平地の跳躍か台への跳躍か）
import re, sys, os, glob, json, statistics as st
R = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'runs')
for tag in sys.argv[1:]:
    for vd in sorted(glob.glob(f'{R}/{tag}/v*')):
        n = stand = onbox = 0; nj = {}; gaps = []; falls = []
        for m in sorted(glob.glob(f'{vd}/m*')):
            try:
                t = open(f'{m}/run.log').read()
            except OSError:
                continue
            n += 1
            bs = float(json.load(open(f'{m}/env.json')).get('JUMP_BOX_S', '1'))
            g = [(float(a), float(b)) for a, b in re.findall(r'\[box-gap\] t=\S+ toe to box (\S+) m: scale (\S+)', t)]
            last = g[-1] if g else None
            box_jump = last is not None and abs(last[1] - bs) < 1e-4
            if box_jump:
                gaps.append(last[0])
            fell = 'FELL at' in t
            if fell:
                kind = 'box' if box_jump else 'hop'
                falls.append(f'{os.path.basename(m)}:{len(g)}{kind}')
                continue
            stand += 1
            if 'on the box true' in t:
                onbox += 1
            j = re.search(r'box gap: (\d+) jumps', t)
            if j: nj[int(j.group(1))] = nj.get(int(j.group(1)), 0) + 1
        gs = f'gap at box jump med {st.median(gaps):+.3f} [{min(gaps):+.3f},{max(gaps):+.3f}]' if gaps else 'gap -'
        print(f'{tag:6s} {os.path.basename(vd)}: standing {stand}/{n} on box {onbox} | jumps {dict(sorted(nj.items()))} | {gs} | falls {" ".join(falls)}')
