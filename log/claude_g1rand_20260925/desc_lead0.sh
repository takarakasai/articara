set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
L=log/claude_g1rand_20260925; C=log/claude_g1champion_20261003v6/controls.json; BIN=$L/kyo46rs_walk_desc3
[ "$(grep -ac COM_HEIGHT_DESC_LEAD $BIN)" -ge 1 ] || { echo "binary lacks knob"; exit 1; }
echo "##### 降段 EXTEND_DESC=0(RISE 1.08) + DESC_LEAD=0.5"; TAG=DL0.5x0_desc BASE=$C BIN=$BIN MODE=stairs N=48 W=10 SEED=20260924 START_X=0.40 EXTRA='{"SPAWN_Z_EXTRA":"0.005","COM_HEIGHT_DESC_LEAD":"0.5"}' RISERS=-0.020,-0.040,-0.060,-0.080 python3 log/claude_g1rand_20260924/rand.py
{ date +%H:%M; echo "=== LEAD=0.5 のみ（延長は既定 1.08）対 DL0.5_desc（延長 0.25＋LEAD 0.5: 48/48/48/48）==="; python3 $L/mcpair.py DL0.5_desc DL0.5x0_desc; echo "#### DESC_LEAD0_DONE"; } > $L/desc_lead0_summary.out 2>&1
