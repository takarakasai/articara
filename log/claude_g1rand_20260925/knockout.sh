set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
BIN=target/release/examples/kyo46rs_walk; V1=log/claude_g1bdbaseline_20260926/controls.json
run () { echo "##### $1"; TAG=$2 BASE=$V1 BIN=$BIN MODE=stairs N=24 W=13 SEED=20260924 EXTRA="$3" RISERS=0.010 python3 log/claude_g1rand_20260924/rand.py; }
run "v1 そのまま（1 cm）"            KO_base    '{}'
run "STAIR_ZONE_CROUCH=0"           KO_crouch  '{"STAIR_ZONE_CROUCH":"0"}'
run "STAIR_TRUNK_COUPLE=0"          KO_trunk   '{"STAIR_TRUNK_COUPLE":"0"}'
run "STAIR_TD_BIAS=0"               KO_tdbias  '{"STAIR_TD_BIAS":"0"}'
run "STAIR_RISE_SOLE_POSE=0"        KO_risepose '{"STAIR_RISE_SOLE_POSE":"0"}'
run "STAIR_TD_SOLE_FRAME=0"         KO_tdframe '{"STAIR_TD_SOLE_FRAME":"0"}'
run "STAIR_ALIGN_STAND_LINE=0"      KO_align   '{"STAIR_ALIGN_STAND_LINE":"0"}'
run "STAIR_LANDING_FRAC=0.85(v3.1)" KO_landfrac '{"STAIR_LANDING_FRAC":"0.85"}'
run "LIFT_H=0.0666(v3.1)"           KO_lifth   '{"LIFT_H":"0.0665855"}'
echo "#### KNOCKOUT_DONE"
