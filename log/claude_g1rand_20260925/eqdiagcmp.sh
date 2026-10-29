#!/bin/bash
# eqdiagcmp.sh REF NEW : eqdiag.sh の出力を見本ごとに比べる。ファイルの集合が同じで、
#   CSV は時間の列（見出しが *_ns か deadline_miss*）を除いて各欄の文字列が一致、
#   QP のダンプ（qp_diag/*.json）は solve_ns／check_ns を除いて一致、他のファイルはバイト一致、
#   run.log は出力先のパスを OUT に置き換えてから一致、を見る。
python3 -I - "$1" "$2" <<'EOF'
import csv, json, os, sys
R, N = sys.argv[1], sys.argv[2]
def files(d):
    return sorted(os.path.relpath(os.path.join(r, f), d) for r, _, fs in os.walk(d) for f in fs)
def csv_rows(p):
    # csv で読む: QP の status 列は "Degraded { level: .., .. }" のように引用符の中にカンマを含む。
    with open(p, newline='') as f:
        rows = list(csv.reader(f))
    if not rows:
        return rows
    drop = {i for i, h in enumerate(rows[0]) if h.endswith('_ns') or h.startswith('deadline_miss')}
    return [[c for i, c in enumerate(r) if i not in drop] for r in rows]
def log_lines(p, root):
    with open(p, 'rb') as f:
        return [l.replace(root.encode(), b'OUT') for l in f.read().split(b'\n') if b'logging trajectory' not in l]
def qp_json(p):
    d = json.load(open(p))
    d.pop('solve_ns', None); d.pop('check_ns', None)
    return d
total = bad = 0
for s in sorted(os.listdir(R)):
    a, b = os.path.join(R, s), os.path.join(N, s)
    fa, fb = files(a), files(b) if os.path.isdir(b) else []
    if fa != fb:
        bad += 1; print(f'ファイルの集合が違う {s}: 片方だけ {sorted(set(fa) ^ set(fb))[:5]}')
    nd = 0
    for f in fa:
        if f not in fb: continue
        pa, pb = os.path.join(a, f), os.path.join(b, f)
        if f.endswith('.csv'): same = csv_rows(pa) == csv_rows(pb)
        elif f == 'run.log': same = log_lines(pa, a) == log_lines(pb, b)
        elif f.startswith('qp_diag/'): same = qp_json(pa) == qp_json(pb)
        else: same = open(pa, 'rb').read() == open(pb, 'rb').read()
        if not same:
            nd += 1
            if nd <= 5: print(f'差 {s}/{f}')
    total += 1
    if nd: bad += 1
    print(f'{s}: {len(fa)} ファイル、差 {nd}')
print(f'計 {total} 本: 差のある見本 {bad}')
EOF
