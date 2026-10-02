set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
C=log/claude_g1champion_20260927v52/controls.json; BIN=log/claude_g1rand_20260925/kyo46rs_walk_mom
lat () { echo "##### $1"; TAG=$2 BASE=$C BIN=$BIN MODE=push N=48 W=4 SEED=20260924 EXTRA="$3" CONDS=7@90,5@270 python3 log/claude_g1rand_20260924/rand.py; }
sag () { echo "##### $1"; TAG=$2 BASE=$C BIN=$BIN MODE=push N=48 W=4 SEED=20260924 EXTRA="$3" CONDS=11@180,11@0 python3 log/claude_g1rand_20260924/rand.py; }
lat "ロール上限 60"            MM_r60   '{"MOM_ARM_DCM_MAX":"60"}'
lat "ロール上限 120 利得 60"    MM_r120  '{"MOM_ARM_DCM_MAX":"120","MOM_ARM_DCM":"60"}'
sag "ピッチ +40 上限 60"        MM_pp40  '{"MOM_AXES":"xy-","MOM_ARM_DCM_X":"40","MOM_ARM_DCM_X_MAX":"60"}'
sag "ピッチ −40 上限 60"        MM_pm40  '{"MOM_AXES":"xy-","MOM_ARM_DCM_X":"-40","MOM_ARM_DCM_X_MAX":"60"}'
sag "MOM_AXES=xy- のみ"         MM_axes  '{"MOM_AXES":"xy-"}'
echo "#### MOM2_DONE"
