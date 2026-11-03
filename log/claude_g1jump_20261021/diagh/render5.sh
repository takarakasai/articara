#!/bin/bash
# TB4（上）と TB5（下）の前への跳躍を同じ状況（member）で並べた動画（2026-11-03、ユーザ依頼）。
# 1.75 m03（TB4 +0.30 m／TB5 +0.35 m）と 2.0 m08（TB4 +0.29 m／TB5 +0.36 m、render4 と同じ状況）、種 31415。
# TB4 の 2.0 m08 は render4.sh の描画を使い回す（j33 と j34 は未設定でビット同一）。
set -u
cd /home/takara/work/dp/articara
source ./setup-mujoco.sh >/dev/null
D=log/claude_g1jump_20261021; O=$D/diagh; V=$O/vid; B=$D/g1_flight_probe_j34
one() { # 名前 runs のディレクトリ タイトル 副題
  local n=$1 src=$D/runs/$2
  env $(python3 -c "import json,sys; print(' '.join(f'{k}={v}' for k,v in json.load(open(sys.argv[1])).items()))" $src/env.json) TRAJ_CSV=$O/$n.csv $B > $O/$n.log 2>&1
  grep -E "verdict|forward" $O/$n.log
  local t0=$(python3 -c "import json,sys; print(json.load(open(sys.argv[1]))['T_LAUNCH'])" $src/env.json)
  awk -F, -v a=$(echo "$t0 + 0.6" | bc) -v b=$(echo "$t0 + 3.4" | bc) 'NR==1 || ($1>=a && $1<=b)' $O/$n.csv > $V/$n.csv
  mkdir -p $V/$n
  env ROBOT=g1 SLOWMO=0.25 COM_PLOT_LO=0.40 COM_PLOT_HI=0.90 TAU_LABEL=sagittal_joint_torque,_left_leg TITLE="$3" SUBTITLE="$4" python3 examples/render_com_squat_video.py $V/$n.csv $V/$n/$n.mp4 > $V/$n.log 2>&1
}
one TB4_v1.75_m03 TB4mid/v1.75/m03 "TB4 (1.75 m/s): lean 4.5 cm, push momentum weight 10" "LANDED, +0.30 m forward   (0.25x)" &
one TB5_v1.75_m03 TB5mid/v1.75/m03 "TB5 (1.75 m/s): lean 5.25 cm, push momentum weight 1" "LANDED, +0.35 m forward   (0.25x)" &
one TB5_v2.0_m08 L6MP1/v2.0/m08 "TB5 (2.0 m/s): lean 6 cm, push momentum weight 1" "LANDED, +0.36 m forward   (0.25x)" &
wait
for v in 1.75_m03 2.0_m08; do
  ffmpeg -y -loglevel error -i $V/TB4_v$v/TB4_v$v.mp4 -i $V/TB5_v$v/TB5_v$v.mp4 -filter_complex "[0:v][1:v]vstack=inputs=2[v]" -map "[v]" -c:v libx264 -pix_fmt yuv420p -crf 23 $V/TB45_v$v.mp4
done
printf "file 'TB45_v1.75_m03.mp4'\nfile 'TB45_v2.0_m08.mp4'\n" > $V/concat_tb45.txt
ffmpeg -y -loglevel error -f concat -safe 0 -i $V/concat_tb45.txt -c copy $D/g1_forward_jump_tb4_vs_tb5.mp4 && echo "#### RENDER5_DONE"
