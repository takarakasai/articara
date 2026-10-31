"""家族の昇格判定（2026-10-31）。使い方（リポジトリの根から）:
  python3 log/claude_g1rand_20260925/family_judge.py [--kyo | --prefix P --benches B1,B2,...] BASE1:CAND1 [BASE2:CAND2 ...]
  例: family_judge.py v620n:fA58n v620o:fA58o v620t:fA58t
      family_judge.py --kyo v2n:v3n v2o:v3o v2t:v3t        （kyo46rs の家族 kfamily.sh、KFAM_*）
  家族の出力は <R>/<P>_<NAME>_<bench>/。既定は G1 の family.sh（P=FAM、ベンチは BENCHES）。
  --kyo は kfamily.sh の家族（P=KFAM、ベンチは KYO_BENCHES）。--prefix／--benches（または環境変数
  FAM_PREFIX／FAM_BENCHES、コンマ区切り）で別の家族を渡せる。--benches を省いて --prefix だけを渡すと、
  最初の組の BASE の出力にあるベンチを全部使う（2026-10-31）。

family_cmp.sh との違い:
- 全部の条件を出す（family_cmp.sh は z が負の条件しか出さないので、条件ごとに足すと損の側に偏る）。
- 個体を controls.json の中身（REF_LIBRARY・TRAJ_CSV を除く）で見分ける。FAM_SEED と FAM_SEED_HOLD を
  入れ替えた 2 つの種では、p・lat（種 S1）と hold（種 S2）が共通の条件で同じ個体を回すので、条件ごとの
  合計と「別の種での再現」はその重複を除いて数える。
旧い規則（〜10-30、memory family-include-speed-push）: 1 = 組ごとに入れ替わり 4 本以上で z ≤ −1.5、
  2 = 別の組でも z ≤ −1 に再現。2 は 0／1（1 本の損）で z = −1 になるので、約 100 条件 × 3 組では偶然でも当たる。
  参考として旧い節（規則 1・2）も出す。
改定した規則（10-31、ユーザ判断）。組は個体の重なりが無いもの（独立な種）だけを数える:
  A. 強い悪化の再現: ある組で入れ替わり 4 本以上・z ≤ −1.5 の条件が、別の組でも z ≤ −1 かつ損 2 本以上。
     別の組の損の個体は、最初の組の同じ条件の個体と重ならないこと（ベンチの条件ごとに見る）。
  B. 合算の悪化: 組を合わせ（個体の重複を除く）、ベンチの条件ごとに損の片側の正確な二項検定
     （入れ替わりのうち損が d 本以上の確率、p=1/2）。入れ替わりのある条件の数で Holm 補正し、0.05 未満。
     z ≤ −1.5 のような固定の閾値は約 100 条件で偶然でも当たる（片側約 7%）ので使わない。
  A か B に当たれば悪化。A の前半だけ（再現なし）は「要確認」として記録する。家族全体の z も出す。
"""
import glob, hashlib, json, math, os, re, sys
from collections import defaultdict

R = 'log/claude_g1rand_20260924'
BENCHES = ['p', 'lat', 'w', 'hold', 's', 'desc', 'desc_n3', 'desc_deep', 'asc_n4', 'c3', 'spd_30', 'spd_50',
           'bump10', 'rough30', 'slope5', 'slope-5', 'sense', 'mis_p5', 'mis_ad2', 'comb_slope3', 'comb_bump10']
# kyo46rs の家族（kfamily.sh、2026-10-31）
KYO_BENCHES = ['kp', 'khold', 'kw', 'kdesc', 'kflat', 'kbump05', 'kbump10', 'ksense']
PREFIX = 'FAM'
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
        for md in sorted(glob.glob(f'{R}/{PREFIX}_{base}_{b}/*/m*')):
            cond, m = md.split('/')[-2:]
            a = surv(md); c = surv(f'{R}/{PREFIX}_{cand}_{b}/{cond}/{m}')
            if a is None or c is None:
                continue
            out.append((b, cond, member_key(md), a, c))
    return out


def parse_args(argv):
    """--kyo／--prefix／--benches（と FAM_PREFIX／FAM_BENCHES）を読み、組の列を返す。"""
    global PREFIX, BENCHES
    prefix = os.environ.get('FAM_PREFIX'); benches = os.environ.get('FAM_BENCHES'); rest = []
    it = iter(argv)
    for a in it:
        if a == '--kyo':
            prefix = prefix or 'KFAM'; benches = benches or ','.join(KYO_BENCHES)
        elif a == '--prefix':
            prefix = next(it)
        elif a == '--benches':
            benches = next(it)
        else:
            rest.append(a)
    pairs = [p.split(':') for p in rest]
    if prefix:
        PREFIX = prefix
    if benches:
        BENCHES = [b for b in benches.split(',') if b]
    elif prefix and pairs:
        head = f'{R}/{PREFIX}_{pairs[0][0]}_'
        BENCHES = sorted(d[len(head):] for d in glob.glob(head + '*') if os.path.isdir(d))
    return pairs


def main():
    g1 = list(BENCHES)
    pairs = parse_args(sys.argv[1:])
    if PREFIX != 'FAM' or BENCHES != g1:   # G1 の既定の出力は 10-31 のまま
        print(f'家族 {PREFIX}_*、ベンチ {len(BENCHES)}: {",".join(BENCHES)}')
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
    verdict(rows, cell, hit1)


def binom_tail(n, d):
    """入れ替わり n 本のうち損が d 本以上になる確率（p=1/2）。"""
    return sum(math.comb(n, k) for k in range(d, n + 1)) / 2 ** n


def verdict(rows, cell, hit1):
    """改定した規則（10-31）による判定。"""
    print('\n## 判定（改定した規則 A・B、10-31）')
    bad = []
    # A: 強い悪化が、別の組で z ≤ −1 かつ損 2 本以上（その条件の個体が重ならないこと）
    for b, c, p, u, d in hit1:
        mine = {x[2] for x in cell[(b, c)][p]}
        rep = []
        for q, r in cell[(b, c)].items():
            if q == p:
                continue
            lost = {x[2] for x in r if x[3] and not x[4]}
            u2 = sum(1 for x in r if x[4] and not x[3]); d2 = len(lost)
            if d2 >= 2 and z(u2, d2) <= -1 and not (lost & mine):
                rep.append(f'{q[0]}→{q[1]} {u2}／{d2}')
        if rep:
            bad.append(f'A {b} {c}: {p[0]}→{p[1]} {u}／{d} が ' + '、'.join(rep) + ' で再現')
        else:
            print(f'- 要確認（再現なし）: {b} {c} {p[0]}→{p[1]} {u}／{d}')
    # B: 合算（個体の重複を除く）、片側の二項検定を Holm 補正
    tests = []
    for (b, c), per in sorted(cell.items()):
        seen = {}
        for r in per.values():
            for x in r:
                seen.setdefault(x[2], x)
        u = sum(1 for x in seen.values() if x[4] and not x[3]); d = sum(1 for x in seen.values() if x[3] and not x[4])
        if u + d:
            tests.append((binom_tail(u + d, d), b, c, u, d))
    tests.sort()
    m = len(tests)
    for i, (pv, b, c, u, d) in enumerate(tests):
        if pv * (m - i) >= 0.05:
            break
        bad.append(f'B {b} {c}: 合算 {u}／{d}、片側 p={pv:.4f}（Holm {pv * (m - i):.3f}、条件 {m}）')
    if tests:
        pv, b, c, u, d = tests[0]
        print(f'- 合算で最も損に寄った条件: {b} {c} {u}／{d}、片側 p={pv:.4f}（Holm {pv * m:.3f}、条件 {m}）')
    for s_ in bad:
        print(f'- 悪化 {s_}')
    print(f'- 結論: {"悪化あり" if bad else "規則上の悪化なし"}')

if __name__ == '__main__':
    main()
