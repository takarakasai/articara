"""転倒の手前の印（2026-11-01）: トルク飽和の始まり、その 60 ms 前までの支持足の CoP（QP）、足裏の浮き、
遊脚の早い接地（片脚支持の予定のまま遊脚に 5 N 超）、合力の最大。使い方: python3 sig.py NAME..."""
import csv, sys
for nm in sys.argv[1:]:
    r = list(csv.DictReader(open(nm + '.csv'))); g = lambda row, k: float(row[k])
    fall = next((i for i, row in enumerate(r) if g(row, 'tilt') > 0.5), len(r) - 1)
    sat = next((i for i in range(len(r)) if g(r[i], 'tau_use_max') >= 0.99 and i > fall - 200), None)
    if sat is None: print(f'{nm}: 飽和なし（転倒 t={g(r[fall],"t"):.3f}）'); continue
    w = r[max(0, sat - 12):sat]
    cx = [max(abs(g(x, 'cop_qp_l_x')) if g(x, 'fz_qp_l') > 5 else 0, abs(g(x, 'cop_qp_r_x')) if g(x, 'fz_qp_r') > 5 else 0) for x in w]
    lift = max(max(g(x, 'sole_l_z'), g(x, 'sole_r_z')) for x in w if g(x, 'support') == 0) if any(g(x, 'support') == 0 for x in w) else float('nan')
    early = None
    for x in r[max(0, sat - 60):sat + 1]:
        s = int(g(x, 'support'))
        if s in (1, 2):
            sw = 'l' if s == 2 else 'r'  # support 2 = 左が遊脚、1 = 右が遊脚（b28 の 9.10〜9.54 で確認）
            if g(x, f'fz_mj_{sw}') > 5: early = g(x, 't'); break
    fmax = max(g(x, 'fz_mj_l') + g(x, 'fz_mj_r') for x in r[sat:sat + 10])
    print(f"{nm}: 飽和 t={g(r[sat],'t'):.3f} 転倒 t={g(r[fall],'t'):.3f} | 直前 60 ms の支持足 |CoP_x| 最大 {1000*max(cx):.0f} mm、"
          f"両脚支持の足裏の最高 {1000*lift:.1f} mm、早い接地 {early}、飽和後の合力最大 {fmax:.0f} N、x={g(r[sat],'com_x'):.3f}")
