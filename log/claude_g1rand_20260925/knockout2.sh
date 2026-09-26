set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
BIN=target/release/examples/kyo46rs_walk; V1=log/claude_g1bdbaseline_20260926/controls.json
run () { echo "##### $1"; TAG=$2 BASE=$V1 BIN=$BIN MODE=stairs N=24 W=13 SEED=20260924 EXTRA="$3" RISERS=0.010 python3 log/claude_g1rand_20260924/rand.py; }
run "T_SS=1.0036(v3.1)"             KO_tss     '{"T_SS":"1.00364"}'
run "STAIR_TOE_MARGIN=0.0235(v3.1)" KO_toe     '{"STAIR_TOE_MARGIN":"0.0234543"}'
run "STRIDE=0.1426(v3.1)"           KO_stride  '{"STRIDE":"0.142585"}'
run "段ノブ全部 v3.1 値"              KO_allstair '{"STAIR_LANDING_FRAC":"0.85","STAIR_TOE_MARGIN":"0.0234543","STAIR_TRUNK_COUPLE":"1.74244","LIFT_H":"0.0665855"}'
echo "#### KNOCKOUT2_DONE"
