#!/bin/bash
# 離地の vx はどこで失われるか（2026-11-02、DIAG_PUSHX=1、j26）。H8L（接触 100）と接触 1 の比較、VX 0／0.8、V 1.5／2.0。未設定の j26 は j25 とバイト一致を先に確かめる。
cd /home/takara/work/dp/articara
source ./setup-mujoco.sh >/dev/null
D=log/claude_g1jump_20261021; O=$D/diagh
eq() { env $(python3 -c "import json,sys; print(' '.join(f'{k}={v}' for k,v in json.load(open(sys.argv[1])).items()))" $D/runs/$1/env.json) $D/g1_flight_probe_j26 > $O/eq_$2.log 2>&1; diff -q <(grep -v "^\s*$" $D/runs/$1/run.log) <(grep -v "^\s*$" $O/eq_$2.log) >/dev/null && echo "eq $1 same" || echo "eq $1 DIFF"; }
eq H8L/v1.5/m00 a & eq H8/v1.5/m01 b & eq RF2A/v2.0/m05 c & wait
common="ROBOT=g1 WBC_STRATEGY=weighted T=7 FLIGHT_FEET=1 LAND_ABSORB=1 LAND_T=0.20 JUMP_RELEASE=1 MOM_TASK=1 CROUCH=0.18 JUMP_Z_TO=0.0 INIT_SEED=1 INIT_JITTER=0.01 MU_GROUND=0.7 T_LAUNCH=1.2 PLANT_MASS_SCALE=1.0 JUMP_ABORT=1 JUMP_PUSH_R=0.3 QD_CLAMP=40 DIAG_TD=1 FLIGHT_FEET_FLAT=1 DIAG_HX=1 RELEASE_FIRST_OFF=1 WBCW_CONTACT_REST=0.1 WBCW_TRUNK_LAND=0.03 FALL_TILT=1.5 LAND_REF_FOLLOW=2 AIR_ABORT_T=0.08 FLIGHT_FEET_CAPTURE=1 FLIGHT_FEET_CAPTURE_T=0.06 KP_FLIGHT_FEET=1000 KD_FLIGHT_FEET=63 DIAG_PUSHX=1"
H="WBCW_TRUNK_PUSH=0.003 WBCW_MOMENTUM=10 WBCW_CONTACT=100 WBCW_TRUNK_LAND_VZ=0"
S="WBCW_MOMENTUM=1 WBCW_CONTACT=1 WBCW_TRUNK_LAND_VZ=0.7"
for v in 1.5 2.0; do for vx in 0 0.8; do
  env $common $H JUMP_V=$v JUMP_VX=$vx $D/g1_flight_probe_j26 > $O/px_H_V${v}_vx${vx}.log 2>&1 &
  env $common $S JUMP_V=$v JUMP_VX=$vx $D/g1_flight_probe_j26 > $O/px_S_V${v}_vx${vx}.log 2>&1 &
done; done
wait
echo "#### PUSHX_DONE"
