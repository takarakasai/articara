#!/bin/bash
# 前への跳躍の動画の 5・6 本目（H8: 接触 100 で h_y ≈ 0 の踏み切り＋前 0.8）と、6 本をつないだもの（2026-11-02）。
set -u
cd /home/takara/work/dp/articara
D=log/claude_g1jump_20261021; O=$D/diagh; V=$O/vid
for n in H8_v2.0_m00 H8_v1.5_m01; do
  awk -F, 'NR==1 || ($1>=1.6 && $1<=4.5)' $O/$n.csv > $V/$n.csv
done
common="ROBOT=g1 SLOWMO=0.25 COM_PLOT_LO=0.40 COM_PLOT_HI=0.90 TAU_LABEL=sagittal_joint_torque,_left_leg"
r() { mkdir -p $V/$1; env $common TITLE="$2" SUBTITLE="$3" python3 examples/render_com_squat_video.py $V/$1.csv $V/$1/$1.mp4 > $V/$1.log 2>&1; }
r H8_v2.0_m00 "G1 jump 5/6: stiff contact (h_y ~ 0 at lift-off) + forward 0.8 + capture feet (2.0 m/s)" "lift-off vx +0.13, pitch momentum ~0, trunk leans back -> LANDED, +0.05 m   (0.25x)" &
r H8_v1.5_m01 "G1 jump 6/6: same at 1.5 m/s" "lift-off vx +0.17, lands 4 cm ahead, then drifts back ~1 s later -> FELL backward   (0.25x)" &
wait
until grep -q RENDER_DONE $O/render.out 2>/dev/null; do sleep 10; done
printf "file 'V2.0_tv3r/V2.0_tv3r.mp4'\nfile 'V2.0_vx0/V2.0_vx0.mp4'\nfile 'V1.5_vx0.2/V1.5_vx0.2.mp4'\nfile 'V1.5_vx0.4/V1.5_vx0.4.mp4'\nfile 'H8_v2.0_m00/H8_v2.0_m00.mp4'\nfile 'H8_v1.5_m01/H8_v1.5_m01.mp4'\n" > $V/concat6.txt
ffmpeg -y -loglevel error -f concat -safe 0 -i $V/concat6.txt -c copy $D/g1_forward_jump_probe6.mp4 && echo "#### RENDER2_DONE"
