#!/bin/bash
# 位置を狙う連続の跳躍（JUMP_FWD_TARGET＋JUMP_FWD_TAB＋FLIGHT_FEET_YAW）: 2.0 で +1 m の m00（3 回）と 1.5 で −0.3 m の m00（3 回）。種 27182、j55。
set -u
cd /home/takara/work/dp/articara
source ./setup-mujoco.sh >/dev/null
D=log/claude_g1jump_20261021; O=$D/diagh; V=$O/vid; B=$D/g1_flight_probe_j55
mkdir -p $V
one() { # 名前 runs のディレクトリ タイトル 終わりの時刻
  local n=$1 src=$D/runs/$2 title=$3 tend=$4
  env $(python3 -c "import json,sys; print(' '.join(f'{k}={v}' for k,v in json.load(open(sys.argv[1])).items()))" $src/env.json) TRAJ_CSV=$O/$n.csv $B > $O/$n.log 2>&1
  local sub=$(grep -o "soles moved .* m at the end (error [^)]*), [0-9]* jumps" $O/$n.log)
  echo "$n: $sub"
  local t0=$(python3 -c "import json,sys; print(json.load(open(sys.argv[1]))['T_LAUNCH'])" $src/env.json)
  awk -F, -v a=$(echo "$t0 + 0.3" | bc) -v b=$tend 'NR==1 || ($1>=a && $1<=b)' $O/$n.csv > $V/$n.csv
  mkdir -p $V/$n
  env ROBOT=g1 SLOWMO=0.5 COM_PLOT_LO=0.40 COM_PLOT_HI=0.90 TAU_LABEL=sagittal_joint_torque,_left_leg TITLE="$title" SUBTITLE="$sub   (0.5x)" python3 examples/render_com_squat_video.py $V/$n.csv $V/$n/$n.mp4 > $V/$n.log 2>&1
}
one FY100_v2.0_m00 FY100/v2.0/m00 "Jumps aimed at 1 m forward (2.0 m/s)" 11.4 &
one FYm30_v1.5_m00 FYm30/v1.5/m00 "Jumps aimed at 0.3 m backward (1.5 m/s)" 11.1 &
wait
printf "file 'FY100_v2.0_m00/FY100_v2.0_m00.mp4'\nfile 'FYm30_v1.5_m00/FYm30_v1.5_m00.mp4'\n" > $V/concat_fy.txt
ffmpeg -y -loglevel error -f concat -safe 0 -i $V/concat_fy.txt -c copy $D/g1_fwd_target.mp4 && echo "#### RENDER10_DONE"
