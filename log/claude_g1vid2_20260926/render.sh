set -u
cd /home/takara/work/dp/articara
D=log/claude_g1vid2_20260926
export ROBOT=g1_23dof VID_W=720 CAM_FOLLOW=1 LEGEND="|"
W=log/claude_g1cem4_20260920/urdf/g1_23dof_arms_fixed.urdf
F=log/claude_g1arms_20260925/g1_23dof_arms_free.urdf
case "$1" in
  stairs_v31) URDF=$W TITLE="G1 / 80 mm riser -- v3.1 champion (welded arms)"            python3 examples/render_com_squat_video.py $D/stairs_v31_clip.csv $D/stairs_v31/stairs_v31.mp4 ;;
  stairs_v1b) URDF=$F TITLE="G1 / 80 mm riser -- BD-style MPC+WBC v1b (free arms)"        python3 examples/render_com_squat_video.py $D/stairs_v1b_clip.csv $D/stairs_v1b/stairs_v1b.mp4 ;;
  flat_v31)   URDF=$W TITLE="G1 / flat, 11 N*s forward push -- v3.1 champion (welded arms)" python3 examples/render_com_squat_video.py $D/flat_v31_clip.csv $D/flat_v31/flat_v31.mp4 ;;
  flat_v1b)   URDF=$F TITLE="G1 / flat, 11 N*s forward push -- BD-style MPC+WBC v1b (free arms)" python3 examples/render_com_squat_video.py $D/flat_v1b_clip.csv $D/flat_v1b/flat_v1b.mp4 ;;
esac
echo "#### RENDER_DONE_$1"
