set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib
L=log/claude_g1rand_20260925; C=$L/kyo_desc1_v2.json; BIN=$L/kyo46rs_walk_tu29
export MODE=stairs START_X=0.40 STAIR_TREAD=0.110 T=28 N=12 W=12 SEED=20260924 DX=0.01 RISERS=0.000
run() { TAG=KDESC2_$1 BASE=$C BIN=$BIN EXTRA="$2" python3 log/claude_g1rand_20260924/rand.py | sed "s/^/$1 /"; }
run rot3      '{"SWING_ROT":"3"}'
run off02     '{"Z_COM_OFFSET":"-0.02"}'
run off02r3   '{"Z_COM_OFFSET":"-0.02","SWING_ROT":"3"}'
run off03r3   '{"Z_COM_OFFSET":"-0.03","SWING_ROT":"3"}'
run off04r3   '{"Z_COM_OFFSET":"-0.04","SWING_ROT":"3"}'
run off04r3abs '{"Z_COM_OFFSET":"-0.04","SWING_ROT":"3","STAIR_ABSOLUTE_TD":"1"}'
run abs       '{"STAIR_ABSOLUTE_TD":"1"}'
run rsp       '{"STAIR_RISE_SOLE_POSE":"1"}'
echo "#### KDESC2_DONE"
