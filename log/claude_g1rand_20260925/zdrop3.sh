set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib
L=log/claude_g1rand_20260925; C=log/claude_g1champion_20261008v65/controls.json; BIN=$L/kyo46rs_walk_zd2
[ "$(grep -ac PUSH_Z_DROP_LAT_MAX $BIN)" -ge 1 ] || { echo "binary lacks knob"; exit 1; }
C8=0@0,8@0,11@0,3@270,5@270,7@90,11@180,14@180; LAT=3@270,4@270,5@270,6@270,3@90,5@90,7@90
for z in 0.024 0.035; do X="{\"PUSH_Z_DROP\":\"$z\"}"
 TAG=ZE${z}_p BASE=$C BIN=$BIN MODE=push N=48 W=12 SEED=20260924 EXTRA="$X" CONDS=$C8 python3 log/claude_g1rand_20260924/rand.py
 TAG=ZE${z}_lat BASE=$C BIN=$BIN MODE=push N=24 W=12 SEED=20260924 EXTRA="$X" CONDS=$LAT python3 log/claude_g1rand_20260924/rand.py
 ( export MU_LO=0.40 MU_HI=0.90 JITTER=0.03 DX=0.05; TAG=ZE${z}W_p BASE=$C BIN=$BIN MODE=push N=48 W=12 SEED=20260924 EXTRA="$X" CONDS=$C8 python3 log/claude_g1rand_20260924/rand.py )
done
{ for z in 0.024 0.035; do echo "=== drop $z (gated) std push"; python3 $L/mcpair.py RFE_p ZE${z}_p push; echo "=== lat"; python3 $L/mcpair.py RFE_lat ZE${z}_lat push; echo "=== wide push"; python3 $L/mcpair.py RFEW_p ZE${z}W_p push; done; echo "#### ZDROP3_DONE"; } > $L/zdrop3_summary.out 2>&1
