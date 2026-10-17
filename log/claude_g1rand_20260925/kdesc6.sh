set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib
L=log/claude_g1rand_20260925; C=$L/kyo_desc1_v2.json; BIN=$L/kyo46rs_walk_tu29
export MODE=stairs START_X=0.40 STAIR_TREAD=0.110 T=28 N=12 W=12 SEED=20260924 DX=0.01 RISERS=0.000,-0.010,-0.020,-0.030
run() { TAG=KDESC6_$1 BASE=$C BIN=$BIN EXTRA="{\"STAIR_SWING_FF\":\"0\",\"STAIR_ABSOLUTE_TD\":\"1\",\"SWING_ROT\":\"3\"$2}" python3 log/claude_g1rand_20260924/rand.py | sed "s/^/$1 /"; }
run o2b15   ',"Z_COM_OFFSET":"-0.02","STAIR_FULL_SOLE":"0","STAIR_TD_BIAS":"-0.015"'
run o2b25   ',"Z_COM_OFFSET":"-0.02","STAIR_FULL_SOLE":"0","STAIR_TD_BIAS":"-0.025"'
run o25     ',"Z_COM_OFFSET":"-0.025"'
run o25b15  ',"Z_COM_OFFSET":"-0.025","STAIR_FULL_SOLE":"0","STAIR_TD_BIAS":"-0.015"'
run o2sink  ',"Z_COM_OFFSET":"-0.02","STAIR_LAND_SINK":"0.015"'
run o2b15sink ',"Z_COM_OFFSET":"-0.02","STAIR_FULL_SOLE":"0","STAIR_TD_BIAS":"-0.015","STAIR_LAND_SINK":"0.015"'
echo "#### KDESC6_DONE"
