#!/bin/bash
# 角度を狙う回す跳躍（TX: JUMP_TURN_TARGET＋JUMP_TURN_TAB）: 2.0 で 90° の m00（2 回）と 1.5 で 180° の m00（5 回）。種 31415、j53。
set -u
cd /home/takara/work/dp/articara
source ./setup-mujoco.sh >/dev/null
D=log/claude_g1jump_20261021; O=$D/diagh; V=$O/vid; B=$D/g1_flight_probe_j53
mkdir -p $V
one() { # 名前 runs のディレクトリ タイトル 終わりの時刻
  local n=$1 src=$D/runs/$2 title=$3 tend=$4
  env $(python3 -c "import json,sys; print(' '.join(f'{k}={v}' for k,v in json.load(open(sys.argv[1])).items()))" $src/env.json) TRAJ_CSV=$O/$n.csv $B > $O/$n.log 2>&1
  local sub=$(grep -o "turn target .* deg at the end (error [^)]*), [0-9]* turning jumps" $O/$n.log)
  echo "$n: $sub"
  local t0=$(python3 -c "import json,sys; print(json.load(open(sys.argv[1]))['T_LAUNCH'])" $src/env.json)
  awk -F, -v a=$(echo "$t0 + 0.3" | bc) -v b=$tend 'NR==1 || ($1>=a && $1<=b)' $O/$n.csv > $V/$n.csv
  mkdir -p $V/$n
  env ROBOT=g1 SLOWMO=0.5 COM_PLOT_LO=0.40 COM_PLOT_HI=0.90 TAU_LABEL=sagittal_joint_torque,_left_leg TITLE="$title" SUBTITLE="$sub   (0.5x)" python3 examples/render_com_squat_video.py $V/$n.csv $V/$n/$n.mp4 > $V/$n.log 2>&1
}
one TX90_v2.0_m00 TX90/v2.0/m00 "Turning jumps aimed at 90 deg (2.0 m/s)" 8.8 &
one TX180_v1.5_m00 TX180/v1.5/m00 "Turning jumps aimed at 180 deg (1.5 m/s)" 19.3 &
wait
printf "file 'TX90_v2.0_m00/TX90_v2.0_m00.mp4'\nfile 'TX180_v1.5_m00/TX180_v1.5_m00.mp4'\n" > $V/concat_tx.txt
ffmpeg -y -loglevel error -f concat -safe 0 -i $V/concat_tx.txt -c copy $D/g1_turn_target.mp4 && echo "#### RENDER9_DONE"
