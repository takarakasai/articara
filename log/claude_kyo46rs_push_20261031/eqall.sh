#!/bin/bash
# 未設定の一致（2026-10-31）: tu63（参照、同じ日に作る）と tu64（LAND_REANCHOR を足したもの）で
# eq.sh（96 本）・押し 14 本（eqlist_push.txt）・eqdiag.sh（4 本）・eqpreset.sh を回して比べる。出力は /data 側。
cd /home/takara/work/dp/articara
L=log/claude_g1rand_20260925; O=/data/articara/log/claude_kyo46rs_push_20261031_eq; mkdir -p $O
for v in 63 64; do
  B=$L/kyo46rs_walk_tu$v
  bash $L/eq.sh $B $O/eq_$v
  LIST=$L/eqlist_push.txt bash $L/eq.sh $B $O/push_$v
  bash $L/eqdiag.sh $B $O/diag_$v
done
bash $L/eqpreset.sh $L/kyo46rs_walk_tu64 $O/preset_64
{ echo "== eq 96"; bash $L/eqcmp.sh $O/eq_63 $O/eq_64
  echo "== push 14"; bash $L/eqcmp.sh $O/push_63 $O/push_64
  echo "== eqdiag"; bash $L/eqdiagcmp.sh $O/diag_63 $O/diag_64
  echo "== eqpreset (tu64 preset vs tu63 full controls)"; bash $L/eqpresetcmp.sh $O/preset_64 $O/eq_63 $O/push_63; } > log/claude_kyo46rs_push_20261031/eqall_result.out 2>&1
echo "#### EQALL_DONE"
