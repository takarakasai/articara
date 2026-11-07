#!/bin/bash
# 向きを変える 5 回跳び（RF5Y: MU_TORSION 0.02、TRUNK_RP_YAWED、JUMP_TURN_FREE_YAW、h_z ±2）: 2.0 の右回り m00 と 1.5 の左回り m02。種 31415、j51。
set -u
cd /home/takara/work/dp/articara
source ./setup-mujoco.sh >/dev/null
D=log/claude_g1jump_20261021; O=$D/diagh; V=$O/vid; B=$D/g1_flight_probe_j51
mkdir -p $V
one() { # 名前 runs のディレクトリ タイトル
  local n=$1 src=$D/runs/$2 title=$3
  env $(python3 -c "import json,sys; print(' '.join(f'{k}={v}' for k,v in json.load(open(sys.argv[1])).items()))" $src/env.json) TRAJ_CSV=$O/$n.csv $B > $O/$n.log 2>&1
  local sub=$(python3 $D/rtsumm1.py $O/$n.log)
  echo "$n: $sub"
  local t0=$(python3 -c "import json,sys; print(json.load(open(sys.argv[1]))['T_LAUNCH'])" $src/env.json)
  awk -F, -v a=$(echo "$t0 + 0.3" | bc) -v b=19.6 'NR==1 || ($1>=a && $1<=b)' $O/$n.csv > $V/$n.csv
  mkdir -p $V/$n
  env ROBOT=g1 SLOWMO=0.5 COM_PLOT_LO=0.40 COM_PLOT_HI=0.90 TAU_LABEL=sagittal_joint_torque,_left_leg TITLE="$title" SUBTITLE="$sub   (0.5x)" python3 examples/render_com_squat_video.py $V/$n.csv $V/$n/$n.mp4 > $V/$n.log 2>&1
}
one RF5Y_v2.0_m00 RF5Y/v2.0/m00 "Turning jumps (2.0 m/s), 5 jumps, h_z +2" &
one RF5Yn_v1.5_m02 RF5Yn/v1.5/m02 "Turning jumps (1.5 m/s), 5 jumps, h_z -2" &
wait
printf "file 'RF5Y_v2.0_m00/RF5Y_v2.0_m00.mp4'\nfile 'RF5Yn_v1.5_m02/RF5Yn_v1.5_m02.mp4'\n" > $V/concat_rf5.txt
ffmpeg -y -loglevel error -f concat -safe 0 -i $V/concat_rf5.txt -c copy $D/g1_turn_jump_5x.mp4 && echo "#### RENDER8_DONE"
