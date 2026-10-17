set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib
L=log/claude_g1rand_20260925; C=$L/kyo_desc1_v2.json; BIN=$L/kyo46rs_walk_tu29
export MODE=stairs START_X=0.40 STAIR_TREAD=0.110 T=28 N=12 W=12 SEED=20260924 DX=0.01 RISERS=-0.010
B='"Z_COM_OFFSET":"-0.02","SWING_ROT":"3"'
run() { TAG=KDESC4_$1 BASE=$C BIN=$BIN EXTRA="{$B$2}" python3 log/claude_g1rand_20260924/rand.py | sed "s/^/$1 /"; }
run ff0     ',"STAIR_SWING_FF":"0"'
run win     ',"STAIR_SWING_WINDOWS":"0.30,0.25,0.70,0.60"'
run sink0   ',"STAIR_TD_SINK":"0","STAIR_LAND_SINK":"0"'
run abs     ',"STAIR_ABSOLUTE_TD":"1"'
run lead    ',"COM_HEIGHT_DESC_LEAD":"1.0"'
run absff0  ',"STAIR_ABSOLUTE_TD":"1","STAIR_SWING_FF":"0"'
run plan    ',"CONTACT_PLAN":"1","CONTACT_ADMIT_CONFIRMED":"1"'
run desc_conf ',"LAND_CONFIRM_XY_DESC":"0.03","LAND_CONFIRM_Z_DESC":"0.003"'
echo "#### KDESC4_DONE"
