#!/bin/bash
# 連続の跳躍（3 回跳び）: R10W（上）と R10C（下、R10W＋CROUCH_TAB 1.0:0.12,1.5:0.18 でしゃがみを浅く）を同じ状況で並べた動画（2026-11-07、R10C を基準にする確認）。
# 1.0 m06・m09 と 1.25 m28 は R10W が 2・3 回目で倒れる本。種 31415、j43（R10W の設定はビット同一）。副題は実行の記録から作る。
set -u
cd /home/takara/work/dp/articara
source ./setup-mujoco.sh >/dev/null
D=log/claude_g1jump_20261021; O=$D/diagh; V=$O/vid; B=$D/g1_flight_probe_j43
one() { # 名前 runs のディレクトリ タイトル [追加の VAR=値 ...]
  local n=$1 src=$D/runs/$2 title=$3; shift 3
  env $(python3 -c "import json,sys; print(' '.join(f'{k}={v}' for k,v in json.load(open(sys.argv[1])).items()))" $src/env.json) "$@" TRAJ_CSV=$O/$n.csv $B > $O/$n.log 2>&1
  local again=$(grep -c jump-again $O/$n.log) sub
  if grep -q "verdict: FELL" $O/$n.log; then sub="falls on jump $((again + 1))   (0.5x)"; else
    sub="3 jumps, $(python3 -c "import re,sys; t=open(sys.argv[1]).read(); print('%+.2f m forward' % float(re.findall(r'final CoM dx ([-+.\d]+)', t)[-1]))" $O/$n.log)   (0.5x)"; fi
  echo "$n: $sub"
  local t0=$(python3 -c "import json,sys; print(json.load(open(sys.argv[1]))['T_LAUNCH'])" $src/env.json)
  awk -F, -v a=$(echo "$t0 + 0.3" | bc) -v b=12.6 'NR==1 || ($1>=a && $1<=b)' $O/$n.csv > $V/$n.csv
  mkdir -p $V/$n
  env ROBOT=g1 SLOWMO=0.5 COM_PLOT_LO=0.40 COM_PLOT_HI=0.90 TAU_LABEL=sagittal_joint_torque,_left_leg TITLE="$title" SUBTITLE="$sub" python3 examples/render_com_squat_video.py $V/$n.csv $V/$n/$n.mp4 > $V/$n.log 2>&1
}
CT=CROUCH_TAB=1.0:0.12,1.5:0.18
one R10W_v1.0_m06 R10W/v1.0/m06 "R10W (1.0 m/s), 3 jumps: crouch 0.18 m" &
one R10C_v1.0_m06 R10W/v1.0/m06 "R10C (1.0 m/s), 3 jumps: crouch 0.12 m" $CT &
one R10W_v1.0_m09 R10W/v1.0/m09 "R10W (1.0 m/s), 3 jumps: crouch 0.18 m" &
one R10C_v1.0_m09 R10W/v1.0/m09 "R10C (1.0 m/s), 3 jumps: crouch 0.12 m" $CT &
one R10W_v1.25_m28 R10Wm/v1.25/m28 "R10W (1.25 m/s), 3 jumps: crouch 0.18 m" &
one R10C_v1.25_m28 R10Wm/v1.25/m28 "R10C (1.25 m/s), 3 jumps: crouch 0.15 m" $CT &
wait
for v in 1.0_m06 1.0_m09 1.25_m28; do
  ffmpeg -y -loglevel error -i $V/R10W_v$v/R10W_v$v.mp4 -i $V/R10C_v$v/R10C_v$v.mp4 -filter_complex "[0:v][1:v]vstack=inputs=2[v]" -map "[v]" -c:v libx264 -pix_fmt yuv420p -crf 23 $V/RWC_v$v.mp4
done
printf "file 'RWC_v1.0_m06.mp4'\nfile 'RWC_v1.0_m09.mp4'\nfile 'RWC_v1.25_m28.mp4'\n" > $V/concat_rwc.txt
ffmpeg -y -loglevel error -f concat -safe 0 -i $V/concat_rwc.txt -c copy $D/g1_repeat_jump_r10w_vs_r10c.mp4 && echo "#### RENDER7_DONE"
