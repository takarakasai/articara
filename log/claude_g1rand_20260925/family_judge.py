"""家族の昇格判定（2026-10-31）。使い方（リポジトリの根から）:
  python3 log/claude_g1rand_20260925/family_judge.py BASE1:CAND1 [BASE2:CAND2 ...]
  例: family_judge.py v620n:fA58n v620o:fA58o v620t:fA58t

family_cmp.sh との違い:
- 全部の条件を出す（family_cmp.sh は z が負の条件しか出さないので、条件ごとに足すと損の側に偏る）。
- 個体を controls.json の中身（REF_LIBRARY・TRAJ_CSV を除く）で見分ける。FAM_SEED と FAM_SEED_HOLD を
  入れ替えた 2 つの種では、p・lat（種 S1）と hold（種 S2）が共通の条件で同じ個体を回すので、条件ごとの
  合計と「別の種での再現」はその重複を除いて数える。
規則（memory family-include-speed-push）:
  1. 入れ替わり 4 本以上で z ≤ −1.5 の条件（組ごと）。
  2. 別の組でも z ≤ −1 に再現した条件。落ちた個体が組の間で重なる場合は再現と数えない。
"""
import glob, hashlib, json, math, os, re, sys
from collections import defaultdict

R = 'log/claude_g1rand_20260924'
BENCHES = ['p', 'lat', 'w', 'hold', 's', 'desc', 'desc_n3', 'desc_deep', 'asc_n4', 'c3', 'spd_30', 'spd_50',
           'bump10', 'rough30', 'slope5', 'slope-5', 'sense', 'mis_p5', 'mis_ad2', 'comb_slope3', 'comb_bump10']
SKIP = {'REF_LIBRARY', 'TRAJ_CSV'}


def z(u, d):
    return (u - d) / math.sqrt(u + d) if u + d else 0.0


def member_key(md):
    c = json.load(open(md + '/controls.json'))
    s = json.dumps({k: v for k, v in c.items() if k not in SKIP}, sort_keys=True)
    return hashlib.md5(s.encode()).hexdigest()[:12]


def surv(md):
    try:
        v = re.search(r'verdict:\s*(\w+)', open(md + '/run.log').read())
    except OSError:
        return None
    return v.group(1) == 'SURVIVED' if v else None


def pair_rows(base, cand):
    """(bench, cond, key, base_ok, cand_ok) の列。"""
    out = []
    for b in BENCHES:
        for md in sorted(glob.glob(f'{R}/FAM_{base}_{b}/*/m*')):
            cond, m = md.split('/')[-2:]
            a = surv(md); c = surv(f'{R}/FAM_{cand}_{b}/{cond}/{m}')
            if a is None or c is None:
                continue
            out.append((b, cond, member_key(md), a, c))
    return out


def main():
    pairs = [p.split(':') for p in sys.argv[1:]]
    rows = {tuple(p): pair_rows(*p) for p in pairs}
    # 1. 組ごとのベンチの表
    print('## ベンチごと（組ごと、全部の条件）')
    print('| ベンチ | ' + ' | '.join(f'{b}→{c}' for b, c in pairs) + ' |')
    print('|---|' + '---|' * len(pairs))
    tot = {p: [0, 0, 0, 0] for p in rows}
    for b in BENCHES:
        cells = []
        for p, rs in rows.items():
            r = [x for x in rs if x[0] == b]
            A = sum(x[3] for x in r); C = sum(x[4] for x in r)
            u = sum(1 for x in r if x[4] and not x[3]); d = sum(1 for x in r if x[3] and not x[4])
            t = tot[p]; t[0] += A; t[1] += C; t[2] += u; t[3] += d
            cells.append(f'{A}→{C}（{u}／{d}{"、%+.2f" % z(u, d) if u + d else ""}）')
        print(f'| {b} | ' + ' | '.join(cells) + ' |')
    print('| 家族全体 | ' + ' | '.join(f'{t[0]}→{t[1]}（{t[2]}／{t[3]}、{z(t[2], t[3]):+.2f}）' for t in tot.values()) + ' |')
    # 2. 規則 1
    print('\n## 規則 1（入れ替わり 4 本以上で z ≤ −1.5、組ごと）')
    cell = defaultdict(lambda: defaultdict(list))   # (bench, cond) -> pair -> rows
    for p, rs in rows.items():
        for x in rs:
            cell[(x[0], x[1])][p].append(x)
    hit1 = []
    for (b, c), per in sorted(cell.items()):
        for p, r in per.items():
            u = sum(1 for x in r if x[4] and not x[3]); d = sum(1 for x in r if x[3] and not x[4])
            if u + d >= 4 and z(u, d) <= -1.5:
                hit1.append((b, c, p, u, d))
                print(f'- {b} {c} {p[0]}→{p[1]}: {u}／{d}、z {z(u, d):+.2f}')
    if not hit1:
        print('- なし')
    # 3. 規則 2（別の組で再現。落ちた個体が重なれば再現と数えない）
    print('\n## 規則 2（2 つ以上の組で z ≤ −1、落ちた個体が組の間で別）')
    hits2 = []
    for (b, c), per in sorted(cell.items()):
        neg = []
        for p, r in per.items():
            u = sum(1 for x in r if x[4] and not x[3]); d = sum(1 for x in r if x[3] and not x[4])
            if z(u, d) <= -1:
                neg.append((p, u, d, {x[2] for x in r if x[3] and not x[4]}))
        if len(neg) < 2:
            continue
        lost_sets = [s for _, _, _, s in neg]
        distinct = len({k for s in lost_sets for k in s}) == sum(len(s) for s in lost_sets)
        hits2.append((b, c, neg, distinct, set().union(*lost_sets)))
    for i, (b, c, neg, distinct, lost) in enumerate(hits2):
        # 別のベンチの記載と落ちた個体が同じ（p と hold の種の入れ替わり）なら、その記載の重複
        same = [f'{b2} {c2}' for j, (b2, c2, _, _, l2) in enumerate(hits2) if j < i and lost <= l2]
        tag = ('再現' if distinct else '個体が組の間で重なる（再現と数えない）') + (f'。{same[0]} と同じ個体' if same else '')
        print(f'- {b} {c}: ' + '、'.join(f'{p[0]}→{p[1]} {u}／{d}' for p, u, d, _ in neg) + f' ── {tag}')
    if not hits2:
        print('- なし')
    # 4. 条件ごと（全ベンチ・全組、個体の重複を除く）
    print('\n## 条件ごと（全ベンチ・全組、個体の重複を除く）')
    seen = {}
    for p, rs in rows.items():
        for x in rs:
            seen.setdefault((x[1], x[2]), x)    # (cond, member) で一意
    agg = defaultdict(lambda: [0, 0, 0])
    for (cond, _), x in seen.items():
        a = agg[cond]; a[0] += 1; a[1] += x[4] and not x[3]; a[2] += x[3] and not x[4]
    U = D = N = 0
    for cond in sorted(agg):
        n, u, d = agg[cond]; N += n; U += u; D += d
        if u + d:
            print(f'- {cond}: 個体 {n}、改善 {u}・悪化 {d}、z {z(u, d):+.2f}')
    dup = sum(len(rs) for rs in rows.values()) - N
    print(f'- 全体: 個体 {N}（重複 {dup} を除く）、改善 {U}・悪化 {D}、z {z(U, D):+.2f}')


if __name__ == '__main__':
    main()
