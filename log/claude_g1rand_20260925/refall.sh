set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib
L=log/claude_g1rand_20260925; C=log/claude_g1champion_20261004v64/controls.json; BIN=$L/kyo46rs_walk_ref10
X='{"REF_LIBRARY":"/data/articara/trajopt/dwk_03.json,/data/articara/trajopt/dwk_05.json,/data/articara/trajopt/dwk7_03.json","REF_TRIG_DCM":"0.036","REF_DELAY_COMP":"0","REF_TRIG_MAX_FRAC":"0.65"}'
C8=0@0,8@0,11@0,3@270,5@270,7@90,11@180,14@180; R=0.020,0.040,0.060,0.080
TAG=RFE_p BASE=$C BIN=$BIN MODE=push N=48 W=12 SEED=20260924 EXTRA="$X" CONDS=$C8 python3 log/claude_g1rand_20260924/rand.py
TAG=RFE_s BASE=$C BIN=$BIN MODE=stairs N=48 W=12 SEED=20260924 EXTRA="$X" RISERS=$R python3 log/claude_g1rand_20260924/rand.py
( export MU_LO=0.40 MU_HI=0.90 JITTER=0.03 DX=0.05; TAG=RFEW_p BASE=$C BIN=$BIN MODE=push N=48 W=12 SEED=20260924 EXTRA="$X" CONDS=$C8 python3 log/claude_g1rand_20260924/rand.py; TAG=RFEW_s BASE=$C BIN=$BIN MODE=stairs N=48 W=12 SEED=20260924 EXTRA="$X" RISERS=$R python3 log/claude_g1rand_20260924/rand.py )
{ echo "=== std push"; python3 $L/mcpair.py WQF_p RFE_p push; echo "=== std stairs"; python3 $L/mcpair.py WQF_s RFE_s; echo "=== wide push"; python3 $L/mcpair.py WQFW_p RFEW_p push; echo "=== wide stairs"; python3 $L/mcpair.py WQFW_s RFEW_s; echo "#### REFALL_DONE"; } > $L/refall_summary.out 2>&1
