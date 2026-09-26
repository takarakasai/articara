set -u
cd /home/takara/work/dp/articara
# 1 cm 候補の評価（ZR/TM）が終わるまで待ってから開始（CPU の取り合いを避ける）
until grep -q CAND_DONE_ZR log/claude_g1rand_20260925/cand_ZR.out 2>/dev/null && grep -q CAND_DONE_TM log/claude_g1rand_20260925/cand_TM.out 2>/dev/null; do sleep 60; done
V4=log/claude_g1champion_20260926v4/controls.json
W=13 scripts/g1_battery.sh S4a   $V4 '{"MPC_ARM_ABOUT_PRIOR":"1"}' std &
W=13 scripts/g1_battery.sh S4ab  $V4 '{"MPC_ARM_ABOUT_PRIOR":"1","MPC_W_LDOT":"100"}' std &
wait
W=13 scripts/g1_battery.sh S4ab0 $V4 '{"MPC_ARM_ABOUT_PRIOR":"1","MPC_W_LDOT":"100","MPC_W_MOM":"0"}' std &
W=13 scripts/g1_battery.sh S4b   $V4 '{"MPC_W_LDOT":"100"}' std &
wait
echo "#### STEP4AB_DONE"
