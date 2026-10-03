set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
L=log/claude_g1rand_20260925; BIN=$L/kyo46rs_walk_course2
V6=log/claude_g1champion_20261003v6/controls.json; V62=log/claude_g1champion_20261003v62/controls.json; V63=log/claude_g1bdbaseline_20260926/controls_v63_candidate_descbias.json
[ "$(grep -ac STAIR_DESC_STAND_MARGIN $BIN)" -ge 1 ] || { echo "binary lacks knob"; exit 1; }
run() { TAG=$1 BASE=$2 STAIR_N=$3 T=$5 BIN=$BIN MODE=stairs N=48 W=12 SEED=20260924 EXTRA="{\"STAIRS_DOWN\":\"$3\",\"STAIR_LANDING_LEN\":\"1.5\",\"N_STEPS\":\"$4\"}" RISERS=0.020,0.040,0.060,0.080 python3 log/claude_g1rand_20260924/rand.py; }
echo "##### 往復 1 段 v6";   run CS1_v6  $V6  1 44 70
echo "##### 往復 1 段 v6.2"; run CS1_v62 $V62 1 44 70
echo "##### 往復 2 段 v6.2"; run CS2_v62 $V62 2 46 75
echo "##### 往復 2 段 v6.3 候補"; run CS2_v63 $V63 2 46 75
{ python3 $L/mcpair.py CS1_v6 CS1_v62; python3 $L/mcpair.py CS2_v62 CS2_v63; echo "#### COURSE_DONE"; } > $L/course2_summary.out 2>&1
