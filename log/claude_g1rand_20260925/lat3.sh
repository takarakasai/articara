set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
C=log/claude_g1champion_20260927v52/controls.json; BIN=log/claude_g1champion_20260927v52/kyo46rs_walk
run () { ( echo "##### $1"; TAG=$2 BASE=$C BIN=$BIN MODE=push N=48 W=6 SEED=20260924 EXTRA="$3" CONDS=7@90,5@270 python3 log/claude_g1rand_20260924/rand.py ) & }
run "CP_T_FROM_X=0"              LAT3_fx0     '{"CP_T_FROM_X":"0"}'
run "CP_T_FROM_X=0 + CP_T_MIN 0.3" LAT3_fx0t3  '{"CP_T_FROM_X":"0","CP_T_MIN":"0.3"}'
wait
echo "#### LAT3_DONE"
