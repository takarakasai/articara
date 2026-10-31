#!/bin/bash
# 押しの小ベンチの 6 便目（2026-10-31、tu64）: LAND_REANCHOR（着いた足へ計画を付け直す、新設）を単独と c4 の上で。
cd /home/takara/work/dp/articara
r() { W=8 bash log/claude_kyo46rs_push_20261031/kpb.sh $1 "$2" log/claude_g1rand_20260925/kyo46rs_walk_tu64 > log/claude_kyo46rs_push_20261031/kpb_$1.out 2>&1; }
r c6 "$(cat log/claude_kyo46rs_push_20261031/cand_kpc4.extra),\"LAND_REANCHOR\":\"0.015\"" & r ra1 '"LAND_REANCHOR":"0.015"' & wait
echo "#### CHAIN5_DONE"
