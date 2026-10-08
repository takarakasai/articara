set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib
L=log/claude_g1rand_20260925; C=log/claude_g1champion_20261008v65/controls.json; BIN=$L/kyo46rs_walk_zd1
until grep -q ZDROP_DONE $L/zdrop_summary.out 2>/dev/null; do sleep 30; done
X='{"PUSH_Z_DROP":"0.024"}'; R=0.020,0.040,0.060,0.080
TAG=ZD_s BASE=$C BIN=$BIN MODE=stairs N=48 W=12 SEED=20260924 EXTRA="$X" RISERS=$R python3 log/claude_g1rand_20260924/rand.py
TAG=ZD_desc BASE=$C BIN=$BIN MODE=stairs N=48 W=12 SEED=20260924 START_X=0.40 EXTRA='{"PUSH_Z_DROP":"0.024","SPAWN_Z_EXTRA":"0.005"}' RISERS=-0.020,-0.040,-0.060,-0.080 python3 log/claude_g1rand_20260924/rand.py
TAG=ZD_desc_n3 STAIR_N=3 BASE=$C BIN=$BIN MODE=stairs N=48 W=12 SEED=20260924 START_X=0.40 EXTRA='{"PUSH_Z_DROP":"0.024","SPAWN_Z_EXTRA":"0.005"}' RISERS=-0.040,-0.060,-0.080 python3 log/claude_g1rand_20260924/rand.py
TAG=ZD_c2 STAIR_N=2 T=75 BASE=$C BIN=$BIN MODE=stairs N=48 W=12 SEED=20260924 EXTRA='{"PUSH_Z_DROP":"0.024","STAIRS_DOWN":"2","STAIR_LANDING_LEN":"1.5","N_STEPS":"46"}' RISERS=$R python3 log/claude_g1rand_20260924/rand.py
( export MU_LO=0.40 MU_HI=0.90 JITTER=0.03 DX=0.05; TAG=ZDW_p BASE=$C BIN=$BIN MODE=push N=48 W=12 SEED=20260924 EXTRA="$X" CONDS=0@0,8@0,11@0,3@270,5@270,7@90,11@180,14@180 python3 log/claude_g1rand_20260924/rand.py )
TAG=ZD_lat BASE=$C BIN=$BIN MODE=push N=24 W=12 SEED=20260924 EXTRA="$X" CONDS=3@270,4@270,5@270,6@270,3@90,5@90,7@90 python3 log/claude_g1rand_20260924/rand.py
{ echo "=== stairs"; python3 $L/mcpair.py RFE_s ZD_s; echo "=== desc"; python3 $L/mcpair.py RFE_desc ZD_desc; echo "=== desc n3"; python3 $L/mcpair.py RFE_desc_n3 ZD_desc_n3; echo "=== course 2"; python3 $L/mcpair.py RFE_c2 ZD_c2; echo "=== wide push"; python3 $L/mcpair.py RFEW_p ZDW_p push; echo "=== lat"; python3 $L/mcpair.py RFE_lat ZD_lat push; echo "#### ZDROP2_DONE"; } > $L/zdrop2_summary.out 2>&1
