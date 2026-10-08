set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib
L=log/claude_g1rand_20260925; C=log/claude_g1champion_20261004v64/controls.json; BIN=$L/kyo46rs_walk_ref10
LIB="/data/articara/trajopt/dwk_03.json,/data/articara/trajopt/dwk_05.json,/data/articara/trajopt/dwk7_03.json"
TAG=RFE_lat BASE=$C BIN=$BIN MODE=push N=24 W=12 SEED=20260924 EXTRA="{\"REF_LIBRARY\":\"$LIB\",\"REF_TRIG_DCM\":\"0.036\",\"REF_DELAY_COMP\":\"0\",\"REF_TRIG_MAX_FRAC\":\"0.65\"}" CONDS=3@270,4@270,5@270,6@270,3@90,5@90,7@90 python3 log/claude_g1rand_20260924/rand.py
{ echo "=== detector 0.036 + delay-solved refs vs v6.4"; python3 $L/mcpair.py WQF_lat RFE_lat push; echo "#### REFLAT3_DONE"; } > $L/reflat3_summary.out 2>&1
