set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
L=log/claude_g1rand_20260925; BIN=$L/kyo46rs_walk_ev5; C=log/claude_g1champion_20261003v62/controls.json
for b in 0.01 0.02; do P=O$b
F="\"CONTACT_EVENT\":\"1\",\"STAIR_ABSOLUTE_TD\":\"1\",\"TRACKING_CUBIC_FF\":\"1\",\"STAIR_TD_BIAS_DESC\":\"$b\""
desc() { TAG=$1 STAIR_N=$2 BASE=$C BIN=$BIN MODE=stairs N=48 W=12 SEED=20260924 START_X=0.40 EXTRA="{\"SPAWN_Z_EXTRA\":\"0.005\",$F}" RISERS=$3 python3 log/claude_g1rand_20260924/rand.py; }
course() { TAG=$1 BASE=$C STAIR_N=$2 T=$4 BIN=$BIN MODE=stairs N=48 W=12 SEED=20260924 EXTRA="{\"STAIRS_DOWN\":\"$2\",\"STAIR_LANDING_LEN\":\"1.5\",\"N_STEPS\":\"$3\",$F}" RISERS=0.020,0.040,0.060,0.080 python3 log/claude_g1rand_20260924/rand.py; }
course ${P}_c2 2 46 75; desc ${P}_desc_n2 2 -0.040,-0.060,-0.080; desc ${P}_desc_n3 3 -0.040,-0.060,-0.080; course ${P}_c3 3 48 80
{ echo "=== bias $b"; for p in "AB0.02_c2 _c2" "X3Y_desc_n2 _desc_n2" "X3Y_desc_n3 _desc_n3" "AB0.02_c3 _c3"; do set -- $p; python3 $L/mcpair.py $1 ${P}$2 | grep -E '対|総合'; done; } >> $L/oo_summary.out 2>&1
done
echo "#### OO_DONE" >> $L/oo_summary.out
