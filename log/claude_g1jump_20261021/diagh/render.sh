#!/bin/bash
# 前への跳躍の動画（2026-11-02、コマ置き場が出力の場所ごとなので 1 本ずつ別の場所へ）: 1.9〜4.8 s を切り出して 0.25 倍速で 4 本、つなげて 1 本に。
set -u
cd /home/takara/work/dp/articara
D=log/claude_g1jump_20261021; O=$D/diagh; V=$O/vid
for n in V2.0_tv3r V2.0_vx0 V1.5_vx0.2 V1.5_vx0.4; do
  awk -F, 'NR==1 || ($1>=1.9 && $1<=4.8)' $O/$n.csv > $V/$n.csv
done
common="ROBOT=g1 SLOWMO=0.25 COM_PLOT_LO=0.40 COM_PLOT_HI=0.90 TAU_LABEL=sagittal_joint_torque,_left_leg"
r() { mkdir -p $V/$1; env $common TITLE="$2" SUBTITLE="$3" python3 examples/render_com_squat_video.py $V/$1.csv $V/$1/$1.mp4 > $V/$1.log 2>&1; }
r V2.0_tv3r "G1 jump 1/4: vertical 2.0 m/s (baseline TV3R)" "lift-off vx -0.29 m/s, pitch momentum +3.4: they cancel -> LANDED   (0.25x)" &
r V2.0_vx0 "G1 jump 2/4: same + feet to capture point (no angular momentum)" "feet pulled 8 cm behind for vx -0.29, the +3.4 rotation remains -> FELL forward   (0.25x)" &
r V1.5_vx0.2 "G1 jump 3/4: forward command 0.2 m/s + capture feet (1.5 m/s)" "lift-off vx -0.05, pitch momentum +3.3 -> LANDED, +0.17 m forward   (0.25x)" &
r V1.5_vx0.4 "G1 jump 4/4: forward command 0.4 m/s + capture feet (1.5 m/s)" "lift-off vx +0.10 only, pitch momentum +3.7 -> FELL   (0.25x)" &
wait
printf "file 'V2.0_tv3r/V2.0_tv3r.mp4'\nfile 'V2.0_vx0/V2.0_vx0.mp4'\nfile 'V1.5_vx0.2/V1.5_vx0.2.mp4'\nfile 'V1.5_vx0.4/V1.5_vx0.4.mp4'\n" > $V/concat.txt
ffmpeg -y -loglevel error -f concat -safe 0 -i $V/concat.txt -c copy $D/g1_forward_jump_probe.mp4 && echo "#### RENDER_DONE"
