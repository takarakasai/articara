"""mixchk.sh の比較（2026-11-01）。使い方（リポジトリの根から）: python3 .../mixcmp.py BASE CAND
凹凸・雑音を家族の種で振った版（*m）と、元の版（地形 k+1・雑音の種 既定）を並べ、種ごとの生存数と
対の改善／悪化、3 つの種の合算の z（符号検定の正規近似）を出す。"""
import re, sys, glob, os, math
R = 'log/claude_g1rand_20260924'
def ok(p):
    try: return 'verdict: SURVIVED' in open(p + '/run.log').read()
    except OSError: return None
A, B = sys.argv[1:3]
cases = [('kbump05', 'T28'), ('kbump10', 'T28'),
         ('ksense', 'SENSE_NOISE_QD0p05_SENSE_NOISE_V0p015_SENSE_NOISE_W0p015'),
         ('ksense', 'SENSE_NOISE_QD0p1_SENSE_NOISE_V0p03_SENSE_NOISE_W0p03'), ('ksense', 'SENSE_NOISE_Q0p002')]
for b, c in cases:
    for mix in ('', 'm'):
        row = f"{b + mix:<10} {c[:24]:<24}"; U = D = 0; fin = True
        for s in 'not':
            ms = sorted(os.path.basename(p) for p in glob.glob(f'{R}/KFAM_{A}{s}_{b}{mix}/{c}/m*'))
            x = [ok(f'{R}/KFAM_{A}{s}_{b}{mix}/{c}/{m}') for m in ms]; y = [ok(f'{R}/KFAM_{B}{s}_{b}{mix}/{c}/{m}') for m in ms]
            if not ms or any(v is None for v in x + y): row += f"  {s}: (未)          "; fin = False; continue
            u = sum(1 for a, z in zip(x, y) if not a and z); d = sum(1 for a, z in zip(x, y) if a and not z); U += u; D += d
            row += f"  {s}: {sum(x):>2}→{sum(y):>2} (+{u}/−{d})"
        z = (U - D) / math.sqrt(U + D) if U + D else 0.0
        print(row + (f"   合算 +{U}/−{D} z={z:+.2f}" if fin else ''))
