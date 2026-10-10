set -u
cd /home/takara/work/dp/articara
V=log/claude_g1vid5_20261010
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib
export ROBOT=g1_23dof VID_W=800 CAM_FOLLOW=1 LEGEND="|" URDF=log/claude_g1arms_20260925/g1_23dof_arms_free.urdf
r() { n=$1; shift; rm -rf $V/$n; mkdir -p $V/$n; ( export TITLE="$1"; shift; for kv in "$@"; do export "$kv"; done; python3 examples/render_com_squat_video.py $V/${n}_clip.csv $V/$n/clip.mp4 > $V/${n}_render.log 2>&1 ) & }
r speed45  "v6.7 / flat 0.45 m/s" STAIRS=1,0.000,0.211,30.0114
r asc4     "v6.7 / 4 x 80 mm up" STAIRS=4,0.080,0.211,2.0114 STAIR_LANDING_LEN=3.5
r course3  "v6.7 / 3 up, 1.5 m, 3 down" STAIRS=3,0.080,0.211,2.0114 STAIR_LANDING_LEN=1.5 STAIRS_DOWN=3
r desc3    "v6.7 / 3 x 80 mm down" STAIRS=3,-0.080,0.211,0.4114
r descdeep "v6.7 / 120 mm down" STAIRS=1,-0.120,0.211,0.4114
r pushfwd  "v6.7 / 11 N*s forward" STAIRS=1,0.000,0.211,1.9923 PUSH_IMPULSE_LABEL="11 N*s"
r pushback_v65 "v6.5 / 11 N*s backward" STAIRS=1,0.000,0.211,1.9923 PUSH_IMPULSE_LABEL="11 N*s"
r pushback_v67 "v6.7 / 11 N*s backward" STAIRS=1,0.000,0.211,1.9923 PUSH_IMPULSE_LABEL="11 N*s"
r push7_v66 "v6.6 / 7 N*s to stance" STAIRS=1,0.000,0.211,1.9923 PUSH_IMPULSE_LABEL="7 N*s"
r push7_v67 "v6.7 / 7 N*s to stance" STAIRS=1,0.000,0.211,1.9923 PUSH_IMPULSE_LABEL="7 N*s"
r push5sw  "v6.7 / 5 N*s to swing" STAIRS=1,0.000,0.211,1.9923 PUSH_IMPULSE_LABEL="5 N*s"
wait
echo "#### RENDER_DONE"
for n in speed45 asc4 course3 desc3 descdeep pushfwd push5sw; do ffmpeg -y -loglevel error -i $V/$n/clip.mp4 -vf "pad=iw*2:ih:(ow-iw)/2:0:black" -c:v libx264 -pix_fmt yuv420p $V/${n}_wide.mp4; done
for s in pushback push7; do a=v65; [ $s = push7 ] && a=v66; ffmpeg -y -loglevel error -i $V/${s}_$a/clip.mp4 -i $V/${s}_v67/clip.mp4 -filter_complex "[0:v][1:v]hstack=inputs=2" -c:v libx264 -pix_fmt yuv420p $V/${s}_side.mp4; done
ffmpeg -y -loglevel error -f concat -safe 0 -i $V/concat.txt -c copy $V/g1_v67_evaluation_tour.mp4
for f in $V/g1_v67_evaluation_tour.mp4; do echo "$f $(du -h $f | cut -f1) $(ffprobe -v error -show_entries format=duration -of csv=p=0 $f)s"; done
ffmpeg -y -loglevel error -ss 4.6 -i $V/push7_side.mp4 -frames:v 1 $V/f_push7.png; ffmpeg -y -loglevel error -ss 20 -i $V/course3_wide.mp4 -frames:v 1 $V/f_course3.png; ffmpeg -y -loglevel error -ss 2.3 -i $V/pushback_side.mp4 -frames:v 1 $V/f_pushback.png; ffmpeg -y -loglevel error -ss 5 -i $V/speed45_wide.mp4 -frames:v 1 $V/f_speed45.png
echo "#### VID5B_DONE"
