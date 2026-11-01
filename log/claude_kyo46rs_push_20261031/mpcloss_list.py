"""MPC の適用そのものの損の個体の一覧（2026-11-01）。使い方（リポジトリの根から）:
  python3 log/claude_kyo46rs_push_20261031/mpcloss_list.py [CAND]（既定 dm0）
基準 k3 と CAND の凹凸 5 mm（kbump05/T28）・降段 20 mm（kdesc/r-0.020）で、3 つの種の個体ごとに
生存と転倒の時刻を並べ、悪化（基準は生存・候補は転倒）と改善を数える。"""
import re, sys
R = 'log/claude_g1rand_20260924'
C = sys.argv[1] if len(sys.argv) > 1 else 'dm0'
def info(p):
    try: t = open(p + '/run.log').read()
    except OSError: return None
    v = re.search(r'verdict:\s*(\w+)', t)
    f = re.search(r'FELL at t=([\d.]+) \(z=([\d.]+), floor=([-\d.]+), tilt=([\d.]+)\)', t)
    s = re.search(r'steps taken: (\d+)', t)
    return dict(ok=v and v.group(1) == 'SURVIVED', t=float(f.group(1)) if f else None, tilt=float(f.group(4)) if f else None,
                steps=int(s.group(1)) if s else -1)
for bench in ('kbump05/T28', 'kdesc/r-0.020'):
    b, c = bench.split('/')
    for sd in 'not':
        up = dn = 0; rows = []
        for i in range(48):
            m = f'm{i:02d}'
            a = info(f'{R}/KFAM_k3{sd}_{b}/{c}/{m}'); x = info(f'{R}/KFAM_{C}{sd}_{b}/{c}/{m}')
            if not a or not x: continue
            if a['ok'] and not x['ok']: dn += 1; rows.append(f"  悪化 {m} t={x['t']} 歩={x['steps']} tilt={x['tilt']}")
            if not a['ok'] and x['ok']: up += 1; rows.append(f"  改善 {m} 基準 t={a['t']}")
            if not a['ok'] and not x['ok']: rows.append(f"  両方 {m} 基準 t={a['t']} 候補 t={x['t']}")
        print(f'{bench} 種 {sd}: 改善 {up} 悪化 {dn}'); print('\n'.join(rows))
