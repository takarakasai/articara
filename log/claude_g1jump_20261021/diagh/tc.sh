#!/bin/bash
# 接触の重みを上げると、角運動量のタスクが実際の地面のモーメントに届くか（2026-11-02、j25、1 本ずつ）。
cd /home/takara/work/dp/articara
source ./setup-mujoco.sh >/dev/null
D=log/claude_g1jump_20261021; B=$D/g1_flight_probe_j25; O=$D/diagh
common="ROBOT=g1 WBC_STRATEGY=weighted T=7 FLIGHT_FEET=1 LAND_ABSORB=1 LAND_T=0.20 JUMP_RELEASE=1 MOM_TASK=1 CROUCH=0.18 JUMP_Z_TO=0.0 INIT_SEED=1 INIT_JITTER=0.01 MU_GROUND=0.7 T_LAUNCH=1.2 PLANT_MASS_SCALE=1.0 JUMP_ABORT=1 JUMP_PUSH_R=0.3 QD_CLAMP=40 DIAG_TD=1 FLIGHT_FEET_FLAT=1 DIAG_HX=1 RELEASE_FIRST_OFF=1 WBCW_CONTACT_REST=0.1 WBCW_TRUNK_LAND_VZ=0.7 WBCW_TRUNK_LAND=0.03 FALL_TILT=1.5 LAND_REF_FOLLOW=2 AIR_ABORT_T=0.08 DIAG_H=1 JUMP_V=1.5"
for tp in 0.3 0.003; do for mw in 1 10; do for wc in 10 100; do
  env $common WBCW_TRUNK_PUSH=$tp WBCW_MOMENTUM=$mw WBCW_CONTACT=$wc $B > $O/tc_tp${tp}_mw${mw}_wc${wc}.log 2>&1 &
done; done; done
wait
echo "#### TC_DONE"
