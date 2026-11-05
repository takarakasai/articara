#!/bin/bash
# 連続の跳躍（3 回跳び）: R8TC（上）と R10W（下、空中で足の幅を戻す）を同じ状況で並べた動画（2026-11-05、R10W を基準にする確認）。
# 1.5 m01 と 2.0 m08 は R8TC が 2 回目で倒れ R10W が 3 回とも跳ぶ本、1.0 m02 は R10W でも 3 回目で崩れる本（残りの課題）。種 31415、j40（R8TC の設定はビット同一）。
set -u
cd /home/takara/work/dp/articara
source ./setup-mujoco.sh >/dev/null
D=log/claude_g1jump_20261021; O=$D/diagh; V=$O/vid; B=$D/g1_flight_probe_j40
one() { # 名前 runs のディレクトリ タイトル 副題
  local n=$1 src=$D/runs/$2
  env $(python3 -c "import json,sys; print(' '.join(f'{k}={v}' for k,v in json.load(open(sys.argv[1])).items()))" $src/env.json) TRAJ_CSV=$O/$n.csv $B > $O/$n.log 2>&1
  echo "$n: $(grep -c jump-again $O/$n.log) again, $(grep verdict $O/$n.log) | stored: $(grep -c jump-again $src/run.log) again, $(grep verdict $src/run.log)"
  local t0=$(python3 -c "import json,sys; print(json.load(open(sys.argv[1]))['T_LAUNCH'])" $src/env.json)
  awk -F, -v a=$(echo "$t0 + 0.3" | bc) -v b=12.6 'NR==1 || ($1>=a && $1<=b)' $O/$n.csv > $V/$n.csv
  mkdir -p $V/$n
  env ROBOT=g1 SLOWMO=0.5 COM_PLOT_LO=0.40 COM_PLOT_HI=0.90 TAU_LABEL=sagittal_joint_torque,_left_leg TITLE="$3" SUBTITLE="$4" python3 examples/render_com_squat_video.py $V/$n.csv $V/$n/$n.mp4 > $V/$n.log 2>&1
}
one R8TC_v1.5_m01 R8TC/v1.5/m01 "R8TC (1.5 m/s), 3 jumps: flight feet keep lift-off offsets" "falls on jump 2   (0.5x)" &
one R10W_v1.5_m01 R10W/v1.5/m01 "R10W (1.5 m/s), 3 jumps: flight feet back to 0.236 m width" "3 jumps, +0.93 m forward   (0.5x)" &
one R8TC_v2.0_m08 R8TC/v2.0/m08 "R8TC (2.0 m/s), 3 jumps: flight feet keep lift-off offsets" "falls on jump 2   (0.5x)" &
one R10W_v2.0_m08 R10W/v2.0/m08 "R10W (2.0 m/s), 3 jumps: flight feet back to 0.236 m width" "3 jumps, +0.97 m forward   (0.5x)" &
one R8TC_v1.0_m02 R8TC/v1.0/m02 "R8TC (1.0 m/s), 3 jumps: flight feet keep lift-off offsets" "falls on jump 2   (0.5x)" &
one R10W_v1.0_m02 R10W/v1.0/m02 "R10W (1.0 m/s), 3 jumps: flight feet back to 0.236 m width" "falls on jump 3 (remaining 1.0 failure)   (0.5x)" &
wait
for v in 1.5_m01 2.0_m08 1.0_m02; do
  ffmpeg -y -loglevel error -i $V/R8TC_v$v/R8TC_v$v.mp4 -i $V/R10W_v$v/R10W_v$v.mp4 -filter_complex "[0:v][1:v]vstack=inputs=2[v]" -map "[v]" -c:v libx264 -pix_fmt yuv420p -crf 23 $V/R8W_v$v.mp4
done
printf "file 'R8W_v1.5_m01.mp4'\nfile 'R8W_v2.0_m08.mp4'\nfile 'R8W_v1.0_m02.mp4'\n" > $V/concat_r8w.txt
ffmpeg -y -loglevel error -f concat -safe 0 -i $V/concat_r8w.txt -c copy $D/g1_repeat_jump_r8tc_vs_r10w.mp4 && echo "#### RENDER6_DONE"
