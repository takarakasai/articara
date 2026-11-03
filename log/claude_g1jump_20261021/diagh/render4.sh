#!/bin/bash
# TB3（上）と TB4（下）の前への跳躍を同じ状況（member）で並べた動画（2026-11-03、ユーザ依頼）。
# 1.5 m07（TB3 +0.27 m／TB4 +0.33 m）と 2.0 m08（両方 +0.28 m）、種 31415。
set -u
cd /home/takara/work/dp/articara
source ./setup-mujoco.sh >/dev/null
D=log/claude_g1jump_20261021; O=$D/diagh; V=$O/vid; B=$D/g1_flight_probe_j33
one() { # 名前 runs のディレクトリ タイトル 副題
  local n=$1 src=$D/runs/$2
  env $(python3 -c "import json,sys; print(' '.join(f'{k}={v}' for k,v in json.load(open(sys.argv[1])).items()))" $src/env.json) TRAJ_CSV=$O/$n.csv $B > $O/$n.log 2>&1
  grep -E "verdict|forward" $O/$n.log
  local t0=$(python3 -c "import json,sys; print(json.load(open(sys.argv[1]))['T_LAUNCH'])" $src/env.json)
  awk -F, -v a=$(echo "$t0 + 0.6" | bc) -v b=$(echo "$t0 + 3.4" | bc) 'NR==1 || ($1>=a && $1<=b)' $O/$n.csv > $V/$n.csv
  mkdir -p $V/$n
  env ROBOT=g1 SLOWMO=0.25 COM_PLOT_LO=0.40 COM_PLOT_HI=0.90 TAU_LABEL=sagittal_joint_torque,_left_leg TITLE="$3" SUBTITLE="$4" python3 examples/render_com_squat_video.py $V/$n.csv $V/$n/$n.mp4 > $V/$n.log 2>&1
}
one TB3_v1.5_m07 FW30T0/v1.5/m07 "TB3 (1.5 m/s): flight trunk task off, flight feet weight 30, lean 3 cm" "LANDED, +0.27 m forward   (0.25x)" &
one TB4_v1.5_m07 L45FW10T0/v1.5/m07 "TB4 (1.5 m/s): flight feet weight 10, lean 4.5 cm" "LANDED, +0.33 m forward   (0.25x)" &
one TB3_v2.0_m08 FW30T0/v2.0/m08 "TB3 (2.0 m/s): flight feet weight 30, lean 4.5 cm" "LANDED, +0.28 m forward   (0.25x)" &
one TB4_v2.0_m08 FW10T0/v2.0/m08 "TB4 (2.0 m/s): flight feet weight 10, lean 4.5 cm" "LANDED, +0.29 m forward   (0.25x)" &
wait
for v in 1.5_m07 2.0_m08; do
  ffmpeg -y -loglevel error -i $V/TB3_v$v/TB3_v$v.mp4 -i $V/TB4_v$v/TB4_v$v.mp4 -filter_complex "[0:v][1:v]vstack=inputs=2[v]" -map "[v]" -c:v libx264 -pix_fmt yuv420p -crf 23 $V/TB34_v$v.mp4
done
printf "file 'TB34_v1.5_m07.mp4'\nfile 'TB34_v2.0_m08.mp4'\n" > $V/concat_tb34.txt
ffmpeg -y -loglevel error -f concat -safe 0 -i $V/concat_tb34.txt -c copy $D/g1_forward_jump_tb3_vs_tb4.mp4 && echo "#### RENDER4_DONE"
