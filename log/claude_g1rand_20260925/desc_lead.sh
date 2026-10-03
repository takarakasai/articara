set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
L=log/claude_g1rand_20260925; C=log/claude_g1champion_20261003v6/controls.json; BIN=$L/kyo46rs_walk_desc3
[ "$(grep -ac COM_HEIGHT_DESC_LEAD $BIN)" -ge 1 ] || { echo "binary lacks knob"; exit 1; }
for v in 0.5 0.75; do echo "##### 降段 EXTEND_DESC=0.25 + DESC_LEAD=$v"; TAG=DL${v}_desc BASE=$C BIN=$BIN MODE=stairs N=48 W=10 SEED=20260924 START_X=0.40 EXTRA="{\"SPAWN_Z_EXTRA\":\"0.005\",\"COM_LAND_EXTEND_DESC\":\"0.25\",\"COM_HEIGHT_DESC_LEAD\":\"$v\"}" RISERS=-0.020,-0.040,-0.060,-0.080 python3 log/claude_g1rand_20260924/rand.py; done
{ date +%H:%M; for v in 0.5 0.75; do echo "=== DESC_LEAD=$v 対 v6.1 候補（DX0.25_desc: 48/47/42/3）==="; python3 $L/mcpair.py DX0.25_desc DL${v}_desc; echo "=== DESC_LEAD=$v 対 v6（B6_desc: 48/45/3/0）==="; python3 $L/mcpair.py B6_desc DL${v}_desc; done; echo "#### DESC_LEAD_DONE"; } > $L/desc_lead_summary.out 2>&1
