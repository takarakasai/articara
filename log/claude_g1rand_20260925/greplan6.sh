set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib
L=log/claude_g1rand_20260925; C=log/claude_g1champion_20261020v612/controls.json; BIN=$L/kyo46rs_walk_tu36
C8=0@0,8@0,11@0,3@270,5@270,7@90,11@180,14@180
X='{"MPC_FOOTSTEP_SHIFT_REST":"1","MPC_FOOTSTEP_HOLD_SHIFT":"0.06","MPC_FOOTSTEP_CAPTURE_FLOOR":"0.5"}'
TAG=GREP_p_h06f BASE=$C BIN=$BIN MODE=push N=48 W=12 SEED=20260924 EXTRA="$X" CONDS=$C8 python3 log/claude_g1rand_20260924/rand.py | sed "s/^/p /"
TAG=GREP_lat_h06f BASE=$C BIN=$BIN MODE=push N=24 W=12 SEED=20260924 EXTRA="$X" CONDS=3@270,4@270,5@270,6@270,3@90,5@90,7@90 python3 log/claude_g1rand_20260924/rand.py | sed "s/^/lat /"
python3 $L/mcpair.py GREP_p_base GREP_p_h06f push; python3 $L/mcpair.py GREP_lat_base GREP_lat_h06f push
echo "#### GREP6_DONE"
