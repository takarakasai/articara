#!/bin/bash
# 候補 kl9 = kpc4L ＋ COP_LEASH=0.005・COP_LEASH_TILT=0.02・COP_LEASH_MPC=1（misa-locomotion a672047、バイナリ ml8）。
# kl8（傾きの門なし）は遅れ（ACT_DELAY_TICKS2 0/48、SENSE_DELAY_TICKS4 0/44）で全滅したので打ち切り、門を足した。
# 家族 3 つの種 → 振り直しの凹凸・雑音（mixchk.sh）→ k3・kpc4L との判定。2026-11-01。
cd /home/takara/work/dp/articara
L=log/claude_g1rand_20260925; P=log/claude_kyo46rs_push_20261031; B=$L/kyo46rs_walk_ml8
BIN=$B bash $P/kfam_push2.sh kl9 "$(cat $P/cand_kl9.extra)"
python3 $L/family_judge.py --kyo kpc4Ln:kl9n kpc4Lo:kl9o kpc4Lt:kl9t > $P/kfam_judge_kl9_vs_kpc4L.out 2>&1
BIN=$B W=24 bash $P/mixchk.sh kl9 $P/cand_kl9.extra > $P/mixchk_kl9.out 2>&1
python3 $P/mixcmp.py k3 kl9 > $P/mixcmp_k3_kl9.out; python3 $P/mixcmp.py kpc4L kl9 > $P/mixcmp_kpc4L_kl9.out
echo "#### KL9_CHAIN_DONE"
