set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
L=log/claude_g1rand_20260925; C=log/claude_g1champion_20261003v62/controls.json; BIN=$L/kyo46rs_walk_desc4
[ "$(grep -ac STAIR_TD_BIAS_DESC $BIN)" -ge 1 ] || { echo "binary lacks knob"; exit 1; }
run() { TAG=$1 STAIR_N=$2 BASE=$C BIN=$BIN MODE=stairs N=48 W=12 SEED=20260924 START_X=0.40 EXTRA="{\"SPAWN_Z_EXTRA\":\"0.005\",\"STAIR_TD_BIAS_DESC\":\"$4\"}" RISERS=$3 python3 log/claude_g1rand_20260924/rand.py; }
for b in 0.02 0.03; do
 echo "##### bias $b n=1"; run DB${b}_desc 1 -0.020,-0.040,-0.060,-0.080 $b
 echo "##### bias $b deep"; run DB${b}_desc_deep 1 -0.100,-0.120,-0.150 $b
 echo "##### bias $b n=2"; run DB${b}_desc_n2 2 -0.040,-0.060,-0.080 $b
 echo "##### bias $b n=3"; run DB${b}_desc_n3 3 -0.040,-0.060,-0.080 $b
done
{ for b in 0.02 0.03; do echo "=== bias $b 対 v6.2"; python3 $L/mcpair.py DL0.5_desc DB${b}_desc | tail -1; for s in deep n2 n3; do python3 $L/mcpair.py B62_desc_$s DB${b}_desc_$s; done; done; echo "#### DESC_BIAS_DONE"; } > $L/desc_bias_summary.out 2>&1
