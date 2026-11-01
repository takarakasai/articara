#!/bin/bash
# 候補 kl8 = kpc4L ＋ COP_LEASH=0.005・COP_LEASH_MPC=1（misa-locomotion の作業中、バイナリ ml8）を、家族 3 つの種と
# 振り直しの凹凸・雑音（mixchk.sh）で。並べて ml8 の未設定の一致（tu63 の参照に対して）。2026-11-01。
cd /home/takara/work/dp/articara
L=log/claude_g1rand_20260925; P=log/claude_kyo46rs_push_20261031; B=$L/kyo46rs_walk_ml8
O=/data/articara/log/claude_kyo46rs_push_20261031_eq; R=/data/articara/log/claude_kyo46rs_push_20261031_eq
( bash $L/eq.sh $B $O/eq_ml8; LIST=$L/eqlist_push.txt bash $L/eq.sh $B $O/push_ml8; bash $L/eqdiag.sh $B $O/diag_ml8
  { echo "== eq 96"; bash $L/eqcmp.sh $R/eq_63 $O/eq_ml8
    echo "== push 14"; bash $L/eqcmp.sh $R/push_63 $O/push_ml8
    echo "== eqdiag"; bash $L/eqdiagcmp.sh $R/diag_63 $O/diag_ml8; } > $P/eq_ml8_result.out 2>&1
  echo "#### EQ_ML8_DONE" ) > $P/eq_ml8.out 2>&1 &
BIN=$B bash $P/kfam_push2.sh kl8 "$(cat $P/cand_kpc4Lr.extra)"
python3 $L/family_judge.py --kyo kpc4Ln:kl8n kpc4Lo:kl8o kpc4Lt:kl8t > $P/kfam_judge_kl8_vs_kpc4L.out 2>&1
BIN=$B W=24 bash $P/mixchk.sh kl8 $P/cand_kpc4Lr.extra > $P/mixchk_kl8.out 2>&1
python3 $P/mixcmp.py k3 kl8 > $P/mixcmp_k3_kl8.out; python3 $P/mixcmp.py kpc4L kl8 > $P/mixcmp_kpc4L_kl8.out
wait
echo "#### KL8_CHAIN_DONE"
