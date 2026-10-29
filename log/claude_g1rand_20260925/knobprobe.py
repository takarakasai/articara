#!/usr/bin/env python3
"""ノブを 1 つずつ振って、チャンピオンの出力が変わるかを見る（台帳の分類の実行による確認）。

使い方（リポジトリの根から）:
  BIN=... REF=dir_of_ref_csvs OUT=dir python3 log/claude_g1rand_20260925/knobprobe.py

- 見本 SAMPLES は eq.sh の見本（log/claude_g1rand_20260924/<id>/controls.json）から選んだ 15 本。
  REF に、振る前のバイナリで回した <id>.csv が要る。
- 振る値: flag は実効値の反対。env_f64 は実効値（チャンピオンの値、無ければ既定の数値）の 1.5 倍、
  実効値が 0 なら 1。既定が式（prof.* など）でチャンピオンにも無いものは 0 と 1.37 の 2 通り。
  直読みの数値ノブは 0 と 1 の 2 通り。
- 見本を順に回し、TRAJ_CSV が REF と違った時点で「効く」として打ち切る。全部同じなら「効かない」。
- 台帳（doc/from_claude/20261029_knob_inventory.md）の分類 4（シナリオ）と 5（計測）は振らない。
- 結果は OUT/results.tsv に 1 行ずつ追記（再開できる）。
- SAMPLES_FILE（id を 1 行ずつ）で見本を、TASKS_FILE（[[名前, 値, 分類], ...] の JSON）で振る組を差し替えられる。
"""
import json, os, re, subprocess, sys, tempfile, filecmp
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

ROOT = Path('log/claude_g1rand_20260924')
BIN = str(Path(os.environ['BIN']).resolve())
REF = Path(os.environ['REF'])
OUT = Path(os.environ['OUT']); OUT.mkdir(parents=True, exist_ok=True)
W = int(os.environ.get('W', '30'))
CHAMP = json.load(open('log/claude_g1champion_20261029v619/controls.json'))

SAMPLES = [
    'FAM_nh10_hold__i11_d180__m00', 'FAM_nh10_s__r0.040__m00', 'FAM_nh10_desc__r-0.040__m00',
    'KV3_flat_v3c__r0.000__m00', 'FAM_nh10_lat__i3_d90__m00', 'FAM_nh10_spd_50__i11_d180__m00',
    'FAM_nh10_slope5__T40__m00', 'FAM_nh10_desc_deep__r-0.120__m00', 'FAM_nh10_c3__r0.040__m00',
    'FAM_nh10_rough30__T40__m00', 'FAM_nh10_sense__SENSE_DELAY_TICKS8__m00', 'FAM_nh10_p__i11_d0__m00',
    'KV3_desc_v2__r-0.020__m00', 'KV3_push_v3c__i0.3_d180__m00', 'FAM_nh10_comb_slope3__i11_d180__m00',
]
if os.environ.get('SAMPLES_FILE'):
    SAMPLES = Path(os.environ['SAMPLES_FILE']).read_text().split()
# 直読みのうち数値のもの（文字列・リスト・経路・デバッグは振らない）
DIRECT_NUM = ['COM_HEIGHT_DS_FRAC', 'LAND_CONFIRM_FZ_DESC_FRAC', 'LAND_CONFIRM_XY_DESC', 'LAND_CONFIRM_XY_RISE',
              'LAND_CONFIRM_Z_DESC', 'MJ_NOSLIP', 'MOM_LEVEL', 'PUSH_REPLAN_ANKLE', 'SWING_CAUTIOUS_ERR',
              'SWING_TOE_UP_W', 'ROUGH_MODE', 'ROUGH_MODE_LIFT_H']


def sample_controls(sid):
    return json.load(open(ROOT / sid.replace('__', '/') / 'controls.json'))


def inventory_class():
    cls = {}
    for line in open('doc/from_claude/20261029_knob_inventory.md'):
        m = re.match(r'\|\s*`([A-Z0-9_]+)`\s*\|(.*)', line)
        if m:
            cells = [c.strip() for c in m.group(2).split('|')]
            if len(cells) >= 4:
                cls[m.group(1)] = cells[3]
    return cls


def tasks():
    s = open('examples/kyo46rs_walk.rs').read()
    seen = {}
    for kind, pat in (('f64', r'env_f64\(\s*"([A-Z0-9_]+)"\s*,\s*([^)]*?)\)'),
                      ('flag', r'flag\(\s*"([A-Z0-9_]+)"\s*,\s*(true|false)\s*\)')):
        for n, d in re.findall(pat, s):
            seen.setdefault(n, (kind, d.strip()))
    for n in DIRECT_NUM:
        seen.setdefault(n, ('direct', ''))
    cls = inventory_class()
    out = []
    for n, (kind, d) in sorted(seen.items()):
        c = cls.get(n, '?')
        if c in ('4', '5'):
            continue
        champ = CHAMP.get(n)
        if kind == 'flag':
            eff = (champ != '0') if champ is not None else (d == 'true')
            out.append((n, '0' if eff else '1', c))
        elif kind == 'direct':
            out += [(n, '0', c), (n, '1', c)]
        else:
            lit = re.fullmatch(r'-?[0-9.]+(e[+-]?[0-9]+)?', d)
            eff = None
            if champ is not None:
                try: eff = float(champ)
                except ValueError: pass
            elif lit:
                eff = float(d)
            if eff is None:
                out += [(n, '0', c), (n, '1.37', c)]
            else:
                out.append((n, repr(eff * 1.5) if eff != 0 else '1', c))
    return out


def run(sid, n, v):
    c = sample_controls(sid); c.pop('TRAJ_CSV', None); c[n] = v
    e = dict(os.environ); e.update({k: str(x) for k, x in c.items()})
    fd, csv = tempfile.mkstemp(suffix='.csv', dir=OUT); os.close(fd)
    e['TRAJ_CSV'] = csv
    rc = 0
    try:
        rc = subprocess.run([BIN], env=e, capture_output=True, timeout=1500).returncode
        same = filecmp.cmp(csv, REF / f'{sid}.csv', shallow=False)
    except subprocess.TimeoutExpired:
        same = False
    os.unlink(csv)
    return same, rc


def probe(t):
    n, v, c = t
    for i, sid in enumerate(SAMPLES):
        same, rc = run(sid, n, v)
        if not same:
            # 異常終了（assert など）は「効く」と区別する
            return f'{n}\t{v}\t{c}\t{"効く" if rc == 0 else f"異常終了{rc}"}\t{sid}\t{i + 1}'
    return f'{n}\t{v}\t{c}\t効かない\t-\t{len(SAMPLES)}'


def main():
    res = OUT / 'results.tsv'
    done = set()
    if res.exists():
        done = {tuple(l.split('\t')[:2]) for l in res.read_text().splitlines()}
    ts = ([tuple(t) for t in json.load(open(os.environ['TASKS_FILE']))] if os.environ.get('TASKS_FILE')
          else tasks())
    todo = [t for t in ts if (t[0], t[1]) not in done]
    print(f'tasks {len(todo)} (done {len(done)})', flush=True)
    with ThreadPoolExecutor(W) as ex, open(res, 'a') as f:
        for k, line in enumerate(ex.map(probe, todo)):
            f.write(line + '\n'); f.flush()
            if k % 20 == 0:
                print(f'{k + 1}/{len(todo)} {line}', flush=True)
    print('#### KNOBPROBE_DONE', flush=True)


if __name__ == '__main__':
    if len(sys.argv) > 1 and sys.argv[1] == 'list':
        for t in tasks(): print(*t)
    else:
        main()
