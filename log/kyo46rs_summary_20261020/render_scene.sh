#!/bin/bash
# render_scene.sh NAME T0 T1 TITLE SUBTITLE
cd /home/takara/work/dp/articara
S=/tmp/claude-1000/-home-takara-work-dp-articara/c7814efe-5799-450c-ad10-d7119cecb8a9/scratchpad/vid
n=$1; t0=$2; t1=$3
python3 - "$S/$n" $t0 $t1 <<'PY'
import csv,sys
d,t0,t1=sys.argv[1],float(sys.argv[2]),float(sys.argv[3])
rows=list(csv.DictReader(open(f'{d}/traj.csv')))
w=csv.DictWriter(open(f'{d}/seg.csv','w'),fieldnames=rows[0].keys()); w.writeheader()
w.writerows([r for r in rows if t0<=float(r['t'])<=t1])
PY
STAIRS=$(python3 -c "import json;print(json.load(open('$S/$n/controls.json'))['STAIRS'])")
mkdir -p $S/$n/r
env URDF=log/5_step_c026s009_jerk100_repeat_astra_20260912/model.urdf STAIRS=$STAIRS TARGETS=1 NO_PANELS=1 \
  PREVIEW_CSV=$S/$n/preview.csv CAM_FOLLOW=1 CAM_EYE=0.95,-1.15,0.55 CAM_TARGET=0.06,0,0.30 CAM_FOV=34 \
  TITLE="$4" SUBTITLE="$5" PUSH_IMPULSE_LABEL="${PUSH_LABEL:-}" \
  python3 -c "import sys;sys.path.insert(0,'examples');import render_com_squat_video as r;r.FPS=25;r.main()" $S/$n/seg.csv $S/$n/r/clip.mp4 > $S/$n/render.log 2>&1
echo "$n $(tail -1 $S/$n/render.log)"
