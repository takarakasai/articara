set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
C=log/claude_g1champion_20260927v52/controls.json; BIN=log/claude_g1rand_20260925/kyo46rs_walk_latemg
run () { ( echo "##### $1"; TAG=$2 BASE=$C BIN=$BIN MODE=push N=48 W=6 SEED=20260924 EXTRA="$3" CONDS=7@90,5@270 python3 log/claude_g1rand_20260924/rand.py ) & }
run "緊急着地 0.03" LE03_p '{"CP_LAT_EMERGENCY":"0.03"}'
run "緊急着地 0.02" LE02_p '{"CP_LAT_EMERGENCY":"0.02"}'
run "緊急着地 0.03 TMIN 0.5" LE03t5_p '{"CP_LAT_EMERGENCY":"0.03","CP_LAT_EMERGENCY_TMIN":"0.5"}'
( echo "##### 緊急着地 0.03: std 段 N=24"; TAG=LE03_s BASE=$C BIN=$BIN MODE=stairs N=24 W=4 SEED=20260924 EXTRA='{"CP_LAT_EMERGENCY":"0.03"}' RISERS=0.020,0.040,0.060,0.080 python3 log/claude_g1rand_20260924/rand.py ) &
wait
echo "#### LATEMG_DONE"
