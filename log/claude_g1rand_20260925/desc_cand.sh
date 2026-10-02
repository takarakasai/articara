set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
L=log/claude_g1rand_20260925; C=log/claude_g1champion_20261003v6/controls.json; BIN=$L/kyo46rs_walk_desc
[ "$(grep -ac COM_LAND_EXTEND_DESC $BIN)" -ge 1 ] || { echo "binary lacks knob"; exit 1; }
for v in 0 0.25; do echo "##### 降段 EXTEND_DESC=$v"; TAG=DX${v}_desc BASE=$C BIN=$BIN MODE=stairs N=48 W=10 SEED=20260924 START_X=0.40 EXTRA="{\"SPAWN_Z_EXTRA\":\"0.005\",\"COM_LAND_EXTEND_DESC\":\"$v\"}" RISERS=-0.020,-0.040,-0.060,-0.080 python3 log/claude_g1rand_20260924/rand.py; done
{ date +%H:%M; for v in 0 0.25; do echo "=== EXTEND_DESC=$v 対 v6 降段基準（48／45／3／0）==="; python3 $L/mcpair.py B6_desc DX${v}_desc; done; echo "#### DESC_CAND_DONE"; } > $L/desc_cand_summary.out 2>&1
