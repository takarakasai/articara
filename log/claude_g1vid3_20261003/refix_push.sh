set -u
cd /home/takara/work/dp/articara
V=log/claude_g1vid3_20261003
export ROBOT=g1_23dof VID_W=640 CAM_FOLLOW=1 LEGEND="|" URDF=log/claude_g1arms_20260925/g1_23dof_arms_free.urdf
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
python3 - <<'PY'
import csv,re
V='log/claude_g1vid3_20261003'
for scene in ('pushfwd','pushback'):
    tp=float(re.search(r'PUSH at t=([0-9.]+)',open(f'{V}/{scene}_v6g.log').read()).group(1))
    rows=list(csv.DictReader(open(f'{V}/{scene}_v6g.csv'))); sel=[r for r in rows if tp-2.0<=float(r['t'])<=tp+5.0]
    with open(f'{V}/{scene}_v6g_clip.csv','w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=rows[0].keys()); w.writeheader(); w.writerows(sel)
    print(scene,'v6g push',tp,len(sel),'rows')
PY
for scene in pushfwd pushback; do rm -rf $V/${scene}_v6g; mkdir -p $V/${scene}_v6g; ( TITLE="x" python3 examples/render_com_squat_video.py $V/${scene}_v6g_clip.csv $V/${scene}_v6g/clip.mp4 > $V/${scene}_v6g_render.log 2>&1 ) & done; wait
F=/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf
declare -A SC=( [stairs8w]="80 mm riser, wide envelope" [pushfwd]="flat, 11 N*s forward push" [pushback]="flat, 11 N*s backward push" )
declare -A RC=( [v52]="v5.2 (previous champion)" [v6]="v6 = weighted QP on stairs" [v6g]="gait CEM best (T_ss 0.63 s)" )
for scene in pushfwd pushback; do fc=""; i=0; for rec in v52 v6 v6g; do fc="$fc[$i:v]drawbox=x=0:y=0:w=700:h=34:color=black@1:t=fill,drawtext=fontfile=$F:text='${SC[$scene]}  |  ${RC[$rec]}':x=12:y=8:fontsize=22:fontcolor=white[p$i];"; i=$((i+1)); done
  ffmpeg -y -loglevel error -i $V/${scene}_v52/clip.mp4 -i $V/${scene}_v6/clip.mp4 -i $V/${scene}_v6g/clip.mp4 -filter_complex "${fc}[p0][p1][p2]hstack=inputs=3" -c:v libx264 -pix_fmt yuv420p $V/${scene}_side.mp4; done
ffmpeg -y -loglevel error -f concat -safe 0 -i $V/concat.txt -c copy $V/g1_v52_vs_v6_vs_gaitcem.mp4
echo "#### REFIX_DONE"
