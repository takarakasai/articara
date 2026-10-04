set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
L=log/claude_g1rand_20260925; BIN=$L/kyo46rs_walk_ff4; C=log/claude_g1champion_20261003v62/controls.json; P=J0.10
[ "$(grep -ac COM_LAND_EXTEND_DESC_CP $BIN)" -ge 1 ] || { echo "binary lacks knob"; exit 1; }
F='"TRACKING_CUBIC_FF":"1","LAND_CONFIRM_XY_RISE":"0","COM_LAND_EXTEND_DESC_CP":"0.10"'
desc() { TAG=$1 STAIR_N=$2 BASE=$C BIN=$BIN MODE=stairs N=48 W=12 SEED=20260924 START_X=0.40 EXTRA="{\"SPAWN_Z_EXTRA\":\"0.005\",$F}" RISERS=$3 python3 log/claude_g1rand_20260924/rand.py; }
course() { TAG=$1 BASE=$C STAIR_N=$2 T=$4 BIN=$BIN MODE=stairs N=48 W=12 SEED=20260924 EXTRA="{\"STAIRS_DOWN\":\"$2\",\"STAIR_LANDING_LEN\":\"1.5\",\"N_STEPS\":\"$3\",$F}" RISERS=0.020,0.040,0.060,0.080 python3 log/claude_g1rand_20260924/rand.py; }
course ${P}_c1 1 44 70; course ${P}_c2 2 46 75; course ${P}_c3 3 48 80
desc ${P}_desc 1 -0.020,-0.040,-0.060,-0.080; desc ${P}_desc_deep 1 -0.100,-0.120,-0.150; desc ${P}_desc_n2 2 -0.040,-0.060,-0.080; desc ${P}_desc_n3 3 -0.040,-0.060,-0.080
{ for p in "X3Y_c1 _c1" "AB0.02_c2 _c2" "AB0.02_c3 _c3" "X3Y_desc _desc" "X3Y_desc_deep _desc_deep" "X3Y_desc_n2 _desc_n2" "X3Y_desc_n3 _desc_n3"; do set -- $p; python3 $L/mcpair.py $1 ${P}$2; done; echo "#### JJ_DONE"; } > $L/jj0.10_summary.out 2>&1
