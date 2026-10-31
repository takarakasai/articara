#!/usr/bin/env python3
"""PUSH_TRIG_LPF_HZ の掃引の比較（2026-11-01）。雑音（大）と凹凸 5 mm を、3 つの種で v3（k3）と個体ごとに対応させる。
使い方: python3 lpf_cmp.py NAME...（例: kpc4s lpf5 lpf10 lpf3）。"""
import glob, os, re, statistics, sys
D = 'log/claude_g1rand_20260924'
CONDS = [('kbump05', 'T28'), ('ksense', 'SENSE_NOISE_QD0p1_SENSE_NOISE_V0p03_SENSE_NOISE_W0p03')]
def runs(p, b, c):
    out = {}
    for f in glob.glob(f'{D}/KFAM_{p}_{b}/{c}/m*/run.log'):
        s = open(f).read()
        m = re.search(r'(\d+) live replans', s)
        out[os.path.basename(os.path.dirname(f))] = ('verdict: SURVIVED' in s, int(m.group(1)) if m else 0)
    return out
for name in sys.argv[1:]:
    for b, c in CONDS:
        tb = tw = 0; cells = []
        for s in 'not':
            base, cand = runs(f'k3{s}', b, c), runs(f'{name}{s}', b, c)
            if not cand: cells.append(f'{s}: なし'); continue
            bt = sum(1 for k in cand if cand[k][0] and not base[k][0]); wt = sum(1 for k in cand if base[k][0] and not cand[k][0])
            tb += bt; tw += wt
            rp = statistics.median(v[1] for v in cand.values())
            cells.append(f'{s}: {sum(v[0] for v in base.values())}→{sum(v[0] for v in cand.values())}/{len(cand)}（{bt}／{wt}、作動の中央値 {rp:g}）')
        z = (tb - tw) / (tb + tw) ** 0.5 if tb + tw else 0.0
        print(f'{name:8s} {b:8s} 合算 {tb}／{tw}（z {z:+.2f}） ' + '  '.join(cells))
