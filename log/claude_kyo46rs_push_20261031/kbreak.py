"""押しの小ベンチの内訳（2026-10-31）。使い方（リポジトリの根から）:
  python3 log/claude_kyo46rs_push_20261031/kbreak.py BASE_DIR CAND_DIR [COND ...]
  BASE_DIR／CAND_DIR は log/claude_g1rand_20260924/ の下のベンチの出力（例 KFAM_k3n_kp）。
  条件ごと・押しの側（支持足側／遊脚側、PUSH_STEP の奇偶と PUSH_DEG）・位相（PUSH_AT を 3 等分:
  早い <0.417、中 <0.583、遅い）で、生存の前後と改善／悪化を出す。前後の 0.9 N·s の横押しの
  「支持足側」は、奇数の歩（左足が支持足）で 90°、偶数の歩で 270°。live replans の中央値も出す。
"""
import glob, json, math, os, re, sys, statistics as st
from collections import defaultdict
R = 'log/claude_g1rand_20260924'
def z(u, d): return (u - d) / math.sqrt(u + d) if u + d else 0.0
def info(md):
    try: t = open(md + '/run.log').read()
    except OSError: return None
    v = re.search(r'verdict:\s*(\w+)', t)
    if not v: return None
    c = json.load(open(md + '/controls.json'))
    lr = re.search(r'(\d+) live replans', t)
    return dict(ok=v.group(1) == 'SURVIVED', step=int(c.get('PUSH_STEP', 0)), at=float(c.get('PUSH_AT', 0)),
                deg=c.get('PUSH_DEG', '0'), lr=int(lr.group(1)) if lr else -1)
def side(i):
    d = i['deg']
    if d in ('90', '270'):
        stance_left = i['step'] % 2 == 1
        return '支持足側' if (d == '90') == stance_left else '遊脚側'
    return {'0': '前', '180': '後ろ'}.get(d, d)
def ph(i): return '早' if i['at'] < 0.4167 else '中' if i['at'] < 0.5833 else '遅'
A, B = sys.argv[1], sys.argv[2]; conds = sys.argv[3:]
if not conds:
    conds = sorted(os.path.basename(p) for p in glob.glob(f'{R}/{B}/*') if os.path.isdir(p))
U = D = 0
grp = defaultdict(lambda: [0, 0, 0, 0, 0])
for c in conds:
    rows = []
    for md in sorted(glob.glob(f'{R}/{B}/{c}/m*')):
        m = os.path.basename(md)
        a = info(f'{R}/{A}/{c}/{m}'); b = info(md)
        if a is None or b is None: continue
        rows.append((m, a, b))
    if not rows: continue
    na = sum(r[1]['ok'] for r in rows); nb = sum(r[2]['ok'] for r in rows)
    u = sum(1 for r in rows if r[2]['ok'] and not r[1]['ok']); d = sum(1 for r in rows if r[1]['ok'] and not r[2]['ok'])
    U += u; D += d
    lr = st.median([r[2]['lr'] for r in rows])
    print(f'{c}: {na}→{nb}/{len(rows)}（{u}／{d}、z {z(u, d):+.2f}）live replans 中央値 {lr}')
    for m, a, b in rows:
        g = grp[(c, side(b), ph(b))]
        g[0] += 1; g[1] += a['ok']; g[2] += b['ok']; g[3] += b['ok'] and not a['ok']; g[4] += a['ok'] and not b['ok']
print(f'全体: 改善 {U}・悪化 {D}、z {z(U, D):+.2f}')
print('内訳（条件・側・位相: 前→後/N（改善／悪化））')
for k in sorted(grp):
    n, a, b, u, d = grp[k]
    print(f'  {k[0]} {k[1]} {k[2]}: {a}→{b}/{n}（{u}／{d}）')
