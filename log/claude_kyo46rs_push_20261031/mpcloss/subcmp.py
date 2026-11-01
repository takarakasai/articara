"""subchk.sh の結果の比較（2026-11-01）。使い方（リポジトリの根から）: python3 .../subcmp.py CAND...
種 n の kp 0.9 N·s 4 方向・kdesc r-0.020・kbump05・ksense 雑音（大）で、k3（基準）・kpc4L・候補の生存数と、
候補の k3 比・kpc4L 比の改善／悪化を出す。"""
import re, sys, glob, os
R = 'log/claude_g1rand_20260924'
def ok(p):
    try: return 'verdict: SURVIVED' in open(p + '/run.log').read()
    except OSError: return None
cands = sys.argv[1:]
benches = [('kp', c) for c in ('i0.9_d0', 'i0.9_d90', 'i0.9_d180', 'i0.9_d270')] + [('kdesc', 'r-0.020'), ('kbump05', 'T28'),
          ('ksense', 'SENSE_NOISE_QD0p1_SENSE_NOISE_V0p03_SENSE_NOISE_W0p03')]
print(f"{'ベンチ':<22} {'k3':>4} {'kpc4L':>6} " + ' '.join(f'{c:>22}' for c in cands))
tot = {c: [0, 0, 0, 0] for c in cands}
for b, c in benches:
    ms = sorted(os.path.basename(p) for p in glob.glob(f'{R}/KFAM_k3n_{b}/{c}/m*'))
    k3 = [ok(f'{R}/KFAM_k3n_{b}/{c}/{m}') for m in ms]; L = [ok(f'{R}/KFAM_kpc4Ln_{b}/{c}/{m}') for m in ms]
    row = f"{b+'/'+c[:12]:<22} {sum(map(bool,k3)):>4} {sum(map(bool,L)):>6} "
    for cd in cands:
        x = [ok(f'{R}/KFAM_{cd}n_{b}/{c}/{m}') for m in ms]
        if any(v is None for v in x): row += f"{'(未)':>22} "; continue
        u3 = sum(1 for a, y in zip(k3, x) if not a and y); d3 = sum(1 for a, y in zip(k3, x) if a and not y)
        uL = sum(1 for a, y in zip(L, x) if not a and y); dL = sum(1 for a, y in zip(L, x) if a and not y)
        t = tot[cd]; t[0] += u3; t[1] += d3; t[2] += uL; t[3] += dL
        row += f"{sum(map(bool,x)):>4}（k3比 {u3}/{d3}、L比 {uL}/{dL}）"
    print(row)
for cd in cands:
    u3, d3, uL, dL = tot[cd]; print(f"{cd}: 合計 k3 比 {u3}／{d3}、kpc4L 比 {uL}／{dL}")
