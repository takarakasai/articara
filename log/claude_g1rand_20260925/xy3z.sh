set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
L=log/claude_g1rand_20260925; BIN=$L/kyo46rs_walk_course6; V63=log/claude_g1bdbaseline_20260926/controls_v63_candidate_descbias.json
[ "$(grep -ac LAND_CONFIRM_Z_DESC $BIN)" -ge 1 ] || { echo "binary lacks knob"; exit 1; }
K='"LAND_CONFIRM_XY_DESC":"0.03","LAND_CONFIRM_Z_DESC":"0.003"'
course() { TAG=$1 BASE=$V63 STAIR_N=$2 T=$4 BIN=$BIN MODE=stairs N=48 W=12 SEED=20260924 EXTRA="{\"STAIRS_DOWN\":\"$2\",\"STAIR_LANDING_LEN\":\"1.5\",\"N_STEPS\":\"$3\",$K}" RISERS=0.020,0.040,0.060,0.080 python3 log/claude_g1rand_20260924/rand.py; }
desc() { TAG=$1 STAIR_N=$2 BASE=$V63 BIN=$BIN MODE=stairs N=48 W=12 SEED=20260924 START_X=0.40 EXTRA="{\"SPAWN_Z_EXTRA\":\"0.005\",$K}" RISERS=$3 python3 log/claude_g1rand_20260924/rand.py; }
echo "##### 往復 1 段"; course X3Z_c1 1 44 70
echo "##### 往復 2 段"; course X3Z_c2 2 46 75
echo "##### 降段 1 段"; desc X3Z_desc 1 -0.020,-0.040,-0.060,-0.080
echo "##### 深い 1 段"; desc X3Z_desc_deep 1 -0.100,-0.120,-0.150
echo "##### 連続 2 段"; desc X3Z_desc_n2 2 -0.040,-0.060,-0.080
echo "##### 連続 3 段"; desc X3Z_desc_n3 3 -0.040,-0.060,-0.080
{ for p in "CS1_v62 X3Z_c1" "CS2_v63 X3Z_c2" "DL0.5_desc X3Z_desc" "B62_desc_deep X3Z_desc_deep" "DN0.02_desc_n2 X3Z_desc_n2" "DN0.02_desc_n3 X3Z_desc_n3"; do python3 $L/mcpair.py $p; done; echo "#### XY3_DONE"; } > $L/xy3z_summary.out 2>&1
