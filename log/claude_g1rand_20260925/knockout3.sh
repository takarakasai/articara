set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
BIN=target/release/examples/kyo46rs_walk; V4=log/claude_g1champion_20260926v4/controls.json
run () { echo "##### $1"; TAG=$2 BASE=$V4 BIN=$BIN MODE=stairs N=24 W=26 SEED=20260924 EXTRA="$3" RISERS=0.010 python3 log/claude_g1rand_20260924/rand.py; }
run "v4 そのまま（1 cm）"          K3_base     '{}'
run "STAIR_ZONE_CROUCH=0"         K3_crouch   '{"STAIR_ZONE_CROUCH":"0"}'
run "STAIR_TD_SOLE_FRAME=0"       K3_tdframe  '{"STAIR_TD_SOLE_FRAME":"0"}'
run "STAIR_TD_BIAS=0"             K3_tdbias   '{"STAIR_TD_BIAS":"0"}'
run "STAIR_TRUNK_COUPLE=0"        K3_trunk    '{"STAIR_TRUNK_COUPLE":"0"}'
run "STAIR_LANDING_FRAC=0.85"     K3_landfrac '{"STAIR_LANDING_FRAC":"0.85"}'
run "STAIR_TOE_MARGIN=0"          K3_toe      '{"STAIR_TOE_MARGIN":"0"}'
run "STAIR_Y_OUT=0"               K3_yout     '{"STAIR_Y_OUT":"0"}'
run "STAIR_TRANSFER_DCM=0"        K3_trdcm    '{"STAIR_TRANSFER_DCM":"0"}'
run "STAIR_TRANSFER_KEEP_DCM=0"   K3_trkeep   '{"STAIR_TRANSFER_KEEP_DCM":"0"}'
run "STAIR_TRANSFER_HOLD=0"       K3_trhold   '{"STAIR_TRANSFER_HOLD":"0"}'
run "STAIR_ALIGN_STAND_LINE=0"    K3_align    '{"STAIR_ALIGN_STAND_LINE":"0"}'
run "LIFT_H=0.0666"               K3_lifth    '{"LIFT_H":"0.0665855"}'
run "ZMP_ROLL=0"                  K3_roll     '{"ZMP_ROLL":"0"}'
echo "#### KNOCKOUT3_DONE"
