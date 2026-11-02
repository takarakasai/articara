#!/bin/bash
# (b') 接地している足だけに付いていく LAND_REF_FOLLOW=2 と、床の上の飛行の打ち切り AIR_ABORT_T=0.08（2026-11-02）。j24。
#   =1 は 2.0 を 7 本救ったが、1.0 の小刻みな跳ねで浮いた足に参照が引かれて 6 本落とした。
cd /home/takara/work/dp/articara
source ./setup-mujoco.sh >/dev/null
D=log/claude_g1jump_20261021; B=$D/g1_flight_probe_j24
X='"JUMP_ABORT":"1","JUMP_PUSH_R":"0.3","QD_CLAMP":"40","DIAG_TD":"1","FLIGHT_FEET_FLAT":"1","DIAG_HX":"1","RELEASE_FIRST_OFF":"1","WBCW_CONTACT_REST":"0.1","WBCW_TRUNK_LAND_VZ":"0.7","WBCW_TRUNK_LAND":"0.03","FALL_TILT":"1.5"'
run() { TAG=$1 SEED=$2 BIN=$B CONDS=$4 N=48 W=$5 EXTRA="{$X$3}" python3 $D/jrand.py > /dev/null; }
run G0 31415 '' 2.0 2 &
run RF2 31415 ',"LAND_REF_FOLLOW":"2"' 1.0,1.5,2.0 4 &
run RF2s2 27182 ',"LAND_REF_FOLLOW":"2"' 1.0,1.5,2.0 4 &
run RF2A 31415 ',"LAND_REF_FOLLOW":"2","AIR_ABORT_T":"0.08"' 1.0,1.5,2.0 3 &
run RF2As2 27182 ',"LAND_REF_FOLLOW":"2","AIR_ABORT_T":"0.08"' 1.0,1.5,2.0 3 &
wait
echo "=== FT15 -> G0 (2.0, identical?)"; for m in $D/runs/G0/v2.0/m*; do cmp -s $m/run.log $D/runs/FT15/v2.0/$(basename $m)/run.log || echo "DIFF $m"; done; python3 $D/jrand.py cmp FT15 G0 | tail -1
for p in "FT15 RF2" "FT15s2 RF2s2" "FT15 RF2A" "FT15s2 RF2As2" "RF1 RF2" "RF1s2 RF2s2"; do echo "=== $p"; python3 $D/jrand.py cmp $p; done
for t in RF2A RF2As2; do echo "--- $t 飛ばず/転倒"; grep -l "verdict: NO FLIGHT" $D/runs/$t/v*/m*/run.log | sed "s|$D/runs/||"; grep -l "verdict: FELL" $D/runs/$t/v*/m*/run.log | sed "s|$D/runs/||;s|$| FELL|"; done
echo "#### JB24_DONE"
