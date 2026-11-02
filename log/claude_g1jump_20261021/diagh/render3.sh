#!/bin/bash
# HV12k7 の前への跳躍（1.5 m09、前進 +0.27 m）の動画（2026-11-02）。
set -u
cd /home/takara/work/dp/articara
source ./setup-mujoco.sh >/dev/null
D=log/claude_g1jump_20261021; O=$D/diagh; V=$O/vid; n=HV12k7_v1.5_m09
env $(python3 -c "import json,sys; print(' '.join(f'{k}={v}' for k,v in json.load(open(sys.argv[1])).items()))" $D/runs/HV12k7/v1.5/m09/env.json) TRAJ_CSV=$O/$n.csv $D/g1_flight_probe_j27 > $O/$n.log 2>&1
grep -E "verdict|forward" $O/$n.log
t0=$(grep -m1 "T_LAUNCH" $D/runs/HV12k7/v1.5/m09/env.json | grep -oE "[0-9.]+")
awk -F, -v a=$(echo "$t0 + 0.6" | bc) -v b=$(echo "$t0 + 3.4" | bc) 'NR==1 || ($1>=a && $1<=b)' $O/$n.csv > $V/$n.csv
mkdir -p $V/$n
env ROBOT=g1 SLOWMO=0.25 COM_PLOT_LO=0.40 COM_PLOT_HI=0.90 TAU_LABEL=sagittal_joint_torque,_left_leg TITLE="G1 forward jump: stiff contact + forward 1.2 + capture feet x0.7 (1.5 m/s)" SUBTITLE="lift-off vx ~+0.45, pitch momentum ~0 -> LANDED, +0.27 m forward (HV12k7, 92/96)   (0.25x)" python3 examples/render_com_squat_video.py $V/$n.csv $V/$n/$n.mp4 > $V/$n.log 2>&1
cp $V/$n/$n.mp4 $D/g1_forward_jump_hv12k7.mp4 && echo "#### RENDER3_DONE"
