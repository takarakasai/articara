set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
L=log/claude_g1rand_20260925; BIN=$L/kyo46rs_walk_zo1; C=log/claude_g1champion_20261004v64/controls.json; P=ZO
[ "$(grep -ac Z_COM_OFFSET $BIN)" -ge 1 ] || { echo "binary lacks knob"; exit 1; }
until grep -q ZC_DONE $L/zc_summary.out 2>/dev/null; do sleep 30; done
F='"Z_COM_OFFSET":"-0.024"'
desc() { TAG=$1 STAIR_N=$2 BASE=$C BIN=$BIN MODE=stairs N=48 W=12 SEED=20260924 START_X=0.40 EXTRA="{\"SPAWN_Z_EXTRA\":\"0.005\",$F}" RISERS=$3 python3 log/claude_g1rand_20260924/rand.py; }
desc ${P}_desc 1 -0.020,-0.040,-0.060,-0.080; desc ${P}_desc_deep 1 -0.100,-0.120,-0.150; desc ${P}_desc_n2 2 -0.040,-0.060,-0.080; desc ${P}_desc_n3 3 -0.040,-0.060,-0.080
{ for p in "X3Y_desc _desc" "X3Y_desc_deep _desc_deep" "X3Y_desc_n2 _desc_n2" "X3Y_desc_n3 _desc_n3"; do set -- $p; python3 $L/mcpair.py $1 ${P}$2; done; echo "#### ZO_DONE"; } > $L/zo_summary.out 2>&1
