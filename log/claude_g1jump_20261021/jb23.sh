#!/bin/bash
# (b) 着地の参照を足に付いていかせる（2026-11-02）: TV3F に LAND_REF_FOLLOW=1。j22。
#   真の転倒（2.0）は足が着地後 1.0-1.4 s に前へ 7 cm ずれ、世界に残った参照が踵の縁になって後ろへ倒れていた。
#   未設定の j22 は TV3F（j21）とビット同一のはず → F0（2.0 だけ）で確かめる。
cd /home/takara/work/dp/articara
source ./setup-mujoco.sh >/dev/null
D=log/claude_g1jump_20261021; B=$D/g1_flight_probe_j22
X='"JUMP_ABORT":"1","JUMP_PUSH_R":"0.3","QD_CLAMP":"40","DIAG_TD":"1","FLIGHT_FEET_FLAT":"1","DIAG_HX":"1","RELEASE_FIRST_OFF":"1","WBCW_CONTACT_REST":"0.1","WBCW_TRUNK_LAND_VZ":"0.7","WBCW_TRUNK_LAND":"0.03","FALL_TILT":"1.5"'
run() { TAG=$1 SEED=$2 BIN=$B CONDS=$4 N=48 W=$5 EXTRA="{$X$3}" python3 $D/jrand.py > /dev/null; }
run F0 31415 '' 2.0 4 &
run RF1 31415 ',"LAND_REF_FOLLOW":"1"' 1.0,1.5,2.0 6 &
run RF1s2 27182 ',"LAND_REF_FOLLOW":"1"' 1.0,1.5,2.0 6 &
wait
echo "=== FT15 -> F0 (31415, 2.0 only, should be identical)"; python3 $D/jrand.py cmp FT15 F0
for m in $D/runs/F0/v2.0/m*; do cmp -s $m/run.log $D/runs/FT15/v2.0/$(basename $m)/run.log || echo "DIFF $m"; done
echo "=== FT15 -> RF1 (31415)"; python3 $D/jrand.py cmp FT15 RF1
echo "=== FT15s2 -> RF1s2 (27182)"; python3 $D/jrand.py cmp FT15s2 RF1s2
echo "#### JB23_DONE"
