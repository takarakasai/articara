set -u
cd /home/takara/work/dp/articara
V=log/claude_g1vid3_20261003
export ROBOT=g1_23dof VID_W=640 CAM_FOLLOW=1 LEGEND="|" URDF=log/claude_g1arms_20260925/g1_23dof_arms_free.urdf
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
declare -A TITLE=( [v52]="v5.2 (previous champion)" [v6]="v6 = v5.2 + weighted QP on stairs" [v6g]="gait CEM best (T_ss 0.63 s, stride 0.17 m)" )
declare -A SCENE=( [stairs8w]="G1 / wide envelope, 80 mm riser" [pushfwd]="G1 / flat, 11 N*s forward push" [pushback]="G1 / flat, 11 N*s backward push" )
for scene in stairs8w pushfwd pushback; do for rec in v52 v6 v6g; do
  mkdir -p $V/${scene}_${rec}; ( TITLE="${SCENE[$scene]} -- ${TITLE[$rec]}" python3 examples/render_com_squat_video.py $V/${scene}_${rec}_clip.csv $V/${scene}_${rec}/clip.mp4 > $V/${scene}_${rec}_render.log 2>&1 ) &
done; done; wait
for scene in stairs8w pushfwd pushback; do
  ffmpeg -y -loglevel error -i $V/${scene}_v52/clip.mp4 -i $V/${scene}_v6/clip.mp4 -i $V/${scene}_v6g/clip.mp4 -filter_complex "[0:v][1:v][2:v]hstack=inputs=3" -c:v libx264 -pix_fmt yuv420p $V/${scene}_side.mp4
done
printf "file '%s'\n" $PWD/$V/stairs8w_side.mp4 $PWD/$V/pushfwd_side.mp4 $PWD/$V/pushback_side.mp4 > $V/concat.txt
ffmpeg -y -loglevel error -f concat -safe 0 -i $V/concat.txt -c copy $V/g1_v52_vs_v6_vs_gaitcem.mp4
echo "#### RENDER3_DONE"
