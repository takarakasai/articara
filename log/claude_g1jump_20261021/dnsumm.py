#!/usr/bin/env python3
# 台から跳び降りるの集計: dnsumm.py TAG... — 各 TAG・各 JUMP_V で、立って終えた本、最初の着地の高さ（台の上＝縁に引っかかる／地面）、
# 最初の着地の踵（後ろの足首の原点 − 0.05）の縁からの距離、着地の vz、倒れた本の着地からの時間、狙いとの差、跳んだ回数
import re, sys, os, glob, json, statistics as st
R = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'runs')
med = lambda a: f'{st.median(a):+.3f}' if a else '-'
for tag in sys.argv[1:]:
    for vd in sorted(glob.glob(f'{R}/{tag}/v*')):
        n = stand = 0; heel = []; vz = []; onbox = 0; falls = []; err = []; nj = {}
        for m in sorted(glob.glob(f'{vd}/m*')):
            try:
                t = open(f'{m}/run.log', errors='replace').read()
            except OSError:
                continue
            n += 1
            env = json.load(open(f'{m}/env.json'))
            edge = float(env['JUMP_BOX'].split(',')[0]) + 2.0 if 'JUMP_BOX' in env else None
            td = re.search(r'\[touchdown\] t=(\S+) after \S+ s, com \([^)]*\) v \(\S+, (\S+)\), foot x \["(\S+)", "(\S+)"\]', t)
            gr = re.search(r'\[ground\] t=\S+ landed (\S+) m', t)
            tdt = None
            if td:
                tdt = float(td.group(1)); vz.append(float(td.group(2).rstrip(')')))
                if edge is not None:
                    heel.append(min(float(td.group(3)), float(td.group(4))) - 0.05 - edge)
                if gr and edge is not None and float(gr.group(1)) > -0.02:
                    onbox += 1
            f = re.search(r'FELL at t=(\S+)', t)
            if f:
                ft = float(f.group(1))
                falls.append(f'{os.path.basename(m)}:' + ('pre' if tdt is None or ft < tdt else f'{ft - tdt:.1f}'))
                continue
            stand += 1
            x = re.search(r'soles moved \S+ m at the end \(error (\S+)\)', t)
            if x: err.append(float(x.group(1).rstrip(')')))
            j = re.search(r', (\d+) jumps, reached', t)
            if j: nj[int(j.group(1))] = nj.get(int(j.group(1)), 0) + 1
        hs = f'heel-edge med {med(heel)} [{min(heel):+.3f},{max(heel):+.3f}] <0: {sum(h < 0 for h in heel)}' if heel else 'heel -'
        print(f'{tag:8s} {os.path.basename(vd)}: standing {stand:2d}/{n} | 1st td on box {onbox} | {hs} | td vz {med(vz)} | err med {med(err)} | jumps {dict(sorted(nj.items()))} | falls {" ".join(falls)}')
