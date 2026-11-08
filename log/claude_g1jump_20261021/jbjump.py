#!/usr/bin/env python3
# 台への跳躍 1 回分の特徴: jbjump.py TAG... — 台への跳躍（倍率 JUMP_BOX_S の [box-gap]）の後の最初の離地と着地から、間合い、離地の vx・vz・h_y・ピッチ、着地の足首の原点の x − 台の位置（踵は −0.05）、着地の高さ、倒れたか
import re, sys, glob, json, os
R = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'runs')
print('run                 gap    vx    vz    h_y  pitch | td x-bx  dz    | fell')
for tag in sys.argv[1:]:
    for f in sorted(glob.glob(f'{R}/{tag}/v*/m*/run.log')):
        t = open(f).read()
        env = json.load(open(os.path.dirname(f) + '/env.json'))
        bx, bs = float(env['JUMP_BOX'].split(',')[0]), float(env.get('JUMP_BOX_S', '1'))
        i = [m.start() for m in re.finditer(r'\[box-gap\] t=\S+ toe to box \S+ m: scale (\S+)', t) if abs(float(m.group(1)) - bs) < 1e-4]
        if not i: continue
        s = t[i[-1]:]
        g = float(re.search(r'toe to box (\S+)', s).group(1))
        lo = re.search(r'\[lift-off\] t=\S+ com \(\S+, z \S+\) v \((\S+), (\S+)\).*h_ang \(\S+,(\S+),\S+\), roll \S+ pitch (\S+)', s)
        td = re.search(r'\[touchdown\] t=\S+ after \S+ s, com .*foot x \["(\S+)", "(\S+)"\]', s)
        gd = re.search(r'\[ground\] t=\S+ landed (\S+) m', s)
        fell = 'FELL' in s
        if not (lo and td): print(f'{f[len(R)+1:-8]:18s} {g:+.3f} no flight', 'FELL' if fell else ''); continue
        vx, vz, hy, p = (float(lo.group(k).rstrip(',')) for k in range(1, 5))
        tx = min(float(td.group(1)), float(td.group(2))) - bx
        print(f'{f[len(R)+1:-8]:18s} {g:+.3f} {vx:+.2f} {vz:+.2f} {hy:+.2f} {p:+.2f} | {tx:+.3f} {float(gd.group(1)) if gd else float("nan"):+.3f} | {"FELL" if fell else ""}')
