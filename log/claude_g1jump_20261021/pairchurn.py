# 同じ本どうしの入れ替わり: pairchurn.py 基準の接頭辞 候補の接頭辞（各 _20 _30 _40 _60 の v1.5 v2.0 を突き合わせる）
import sys, os, glob, math
def stood(p):
    t = open(p).read()
    return 'FELL at' not in t and 'on the box true' in t
a, b = sys.argv[1], sys.argv[2]
for c in ('v1.5', 'v2.0'):
    up = dn = n = sa = sb = 0; dl = []
    for bx in ('20', '30', '40', '60'):
        for r in sorted(glob.glob(f'runs/{a}_{bx}/{c}/m*/run.log')):
            m = r.split('/')[-2]; q = f'runs/{b}_{bx}/{c}/{m}/run.log'
            if not os.path.exists(q): continue
            x, y = stood(r), stood(q); n += 1; sa += x; sb += y
            if y and not x: up += 1
            if x and not y: dn += 1; dl.append(f'{bx}:{m}')
    z = (up - dn) / math.sqrt(up + dn) if up + dn else 0.0
    print(f'{a} -> {b} {c}: {sa}/{n} -> {sb}/{n}  改善 {up} 悪化 {dn} z {z:+.2f}  悪化 {" ".join(dl)}')
