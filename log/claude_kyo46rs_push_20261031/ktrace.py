"""押しの個体を軌跡付きで回し直し、押しの後の最初の着地を要約する（2026-10-31）。
使い方（リポジトリの根から）: python3 log/claude_kyo46rs_push_20261031/ktrace.py BIN OUTDIR MEMBER_DIR... [-- KEY=VAL ...]
  MEMBER_DIR は rand.py の個体（controls.json を読む）。KEY=VAL は上書き。T は押しの歩 + 3 歩で打ち切る。
出力（個体ごと 1 行）: 判定の再現、押しの位相、遊脚の目標からの横のずれの最大（押しの後の片脚支持）、
  着地の時刻・位置の目標からのずれ（x, y）、着地時の DCM の新しい支持足からの位置、支持足の滑りの最大。
"""
import csv, json, os, re, subprocess, sys
from concurrent.futures import ThreadPoolExecutor
BIN, OUT = sys.argv[1], sys.argv[2]; rest = sys.argv[3:]
if '--' in rest:
    i = rest.index('--'); mds, kv = rest[:i], rest[i + 1:]
else:
    mds, kv = rest, []
os.makedirs(OUT, exist_ok=True)
def one(md):
    c = json.load(open(md + '/controls.json'))
    for x in kv:
        k, v = x.split('=', 1); c[k] = v
    name = md.rstrip('/').split('/')[-2] + '_' + md.rstrip('/').split('/')[-1]
    csvp = f'{OUT}/{name}.csv'; c['TRAJ_CSV'] = csvp
    e = dict(os.environ); e.update({k: str(v) for k, v in c.items()})
    r = subprocess.run([BIN], env=e, capture_output=True, text=True)
    open(f'{OUT}/{name}.log', 'w').write(r.stdout + r.stderr)
    t = r.stdout + r.stderr
    v = re.search(r'verdict:\s*(\w+)', t).group(1)
    old = re.search(r'verdict:\s*(\w+)', open(md + '/run.log').read()).group(1)
    rows = list(csv.DictReader(open(csvp)))
    f = lambda x, k: float(x[k])
    tp = next(f(x, 't') for x in rows if abs(f(x, 'push_fx')) + abs(f(x, 'push_fy')) > 0)
    i0 = next(i for i, x in enumerate(rows) if f(x, 't') >= tp)
    sup0 = rows[i0]['support']  # 0/1 = 片脚支持の側？ 下で決める
    # 押しの時の遊脚: fz の小さい足
    sw = 'r' if f(rows[i0], 'fz_mj_l') > f(rows[i0], 'fz_mj_r') else 'l'
    st_ = 'l' if sw == 'r' else 'r'
    dev = 0.0; td = None
    for x in rows[i0:]:
        dev = max(dev, abs(f(x, f'sole_{sw}_y') - f(x, 'swing_tgt_y')))
        if f(x, f'fz_mj_{sw}') > 5.0 and f(x, 't') > tp + 0.02:
            td = x; break
    slip = max(f(x, f'slip_{st_}') for x in rows[i0:i0 + 200])
    out = f'{name} {old}->{v} at={c["PUSH_AT"]} sw={sw} tp={tp:.3f}'
    if td:
        out += (f' td={f(td, "t") - tp:.3f}s dev_max={dev*1000:.0f}mm td_err=({(f(td, f"sole_{sw}_x") - f(td, "swing_tgt_x"))*1000:+.0f},{(f(td, f"sole_{sw}_y") - f(td, "swing_tgt_y"))*1000:+.0f})mm'
                f' xi-foot=({(f(td, "xi_x") - f(td, f"sole_{sw}_x"))*1000:+.0f},{(f(td, "xi_y") - f(td, f"sole_{sw}_y"))*1000:+.0f})mm'
                f' xi-stance_y={(f(td, "xi_y") - f(td, f"sole_{st_}_y"))*1000:+.0f}mm')
    out += f' slip_st_max={slip:.2f}'
    return out
with ThreadPoolExecutor(int(os.environ.get('W', '16'))) as p:
    for s in p.map(one, mds):
        print(s, flush=True)
