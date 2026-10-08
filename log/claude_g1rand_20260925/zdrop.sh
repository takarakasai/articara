set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib
L=log/claude_g1rand_20260925; C=log/claude_g1champion_20261008v65/controls.json; BIN=$L/kyo46rs_walk_zd1
[ "$(grep -ac PUSH_Z_DROP $BIN)" -ge 1 ] || { echo "binary lacks knob"; exit 1; }
C8=0@0,8@0,11@0,3@270,5@270,7@90,11@180,14@180
for z in 0.024 0.035; do TAG=ZD${z}_p BASE=$C BIN=$BIN MODE=push N=48 W=12 SEED=20260924 EXTRA="{\"PUSH_Z_DROP\":\"$z\"}" CONDS=$C8 python3 log/claude_g1rand_20260924/rand.py; done
{ for z in 0.024 0.035; do echo "=== PUSH_Z_DROP $z vs v6.5"; python3 $L/mcpair.py RFE_p ZD${z}_p push; done; echo "#### ZDROP_DONE"; } > $L/zdrop_summary.out 2>&1
