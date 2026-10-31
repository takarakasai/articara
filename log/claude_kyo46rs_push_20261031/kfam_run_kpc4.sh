#!/bin/bash
# 候補 kpc4（= 小ベンチの c4: レンチ MPC の適用 ＋ MPC の着地（内側なし）＋ PUSH_REPLAN（G1 の閾値、その場足踏みなし））を家族 3 つの種で。chain4 の後。
cd /home/takara/work/dp/articara
until grep -q CHAIN4_DONE log/claude_kyo46rs_push_20261031/kpb_chain4.out 2>/dev/null; do sleep 10; done
bash log/claude_kyo46rs_push_20261031/kfam_push.sh kpc4 "$(cat log/claude_kyo46rs_push_20261031/cand_kpc4.extra)"
