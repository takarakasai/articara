set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
L=log/claude_g1rand_20260925; C=log/claude_g1champion_20261003v62/controls.json; BIN=$L/kyo46rs_walk_desc5
[ "$(grep -ac STAIR_TD_BIAS_DESC $BIN)" -ge 1 ] || { echo "binary lacks knob"; exit 1; }
run() { TAG=$1 STAIR_N=$2 BASE=$C BIN=$BIN MODE=stairs N=48 W=12 SEED=20260924 START_X=0.40 EXTRA="{\"SPAWN_Z_EXTRA\":\"0.005\",\"STAIR_TD_BIAS_DESC\":\"$4\"}" RISERS=$3 python3 log/claude_g1rand_20260924/rand.py; }
for b in 0.02 0.03; do for n in 2 3; do echo "##### bias(nosing only) $b n=$n"; run DN${b}_desc_n$n $n -0.040,-0.060,-0.080 $b; done; done
{ for b in 0.02 0.03; do echo "=== 段鼻のある踏面のみ後退 $b 対 v6.2"; for s in n2 n3; do python3 $L/mcpair.py B62_desc_$s DN${b}_desc_$s; done; done; echo "#### DESC_BIAS2_DONE"; } > $L/desc_bias2_summary.out 2>&1
