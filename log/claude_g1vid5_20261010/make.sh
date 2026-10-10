set -u
cd /home/takara/work/dp/articara
V=log/claude_g1vid5_20261010; R=log/claude_g1rand_20260924; CH=log/claude_g1champion_20261010v67
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib
BIN=$CH/kyo46rs_walk
# scene controls = champion v6.7 + the member's scenario keys (+ variant overrides)
mk() { python3 - "$@" <<'PY'
import json,sys
out,member=sys.argv[1],sys.argv[2]; over=dict(kv.split('=',1) for kv in sys.argv[3:])
base=json.load(open('log/claude_g1champion_20261010v67/controls.json')); c=json.load(open(member))
sc={k:v for k,v in c.items() if k not in base}
for k in ('T','N_STEPS','STAIR_LANDING_LEN'):
    if k in c: sc[k]=c[k]
base.update(sc); base.update(over); base.pop('TRAJ_CSV',None)
json.dump(base,open(out,'w'),indent=1)
PY
}
sim() { # name member [K=V...]
  n=$1; m=$2; shift 2; mk $V/${n}_controls.json $m "$@"
  ( eval "$(python3 -c "import json; c=json.load(open('$V/${n}_controls.json')); print(' '.join(f'export {k}={v!r}' for k,v in c.items()))")"; export TRAJ_CSV=$V/$n.csv; $BIN > $V/$n.log 2>&1 ) &
}
sim speed45  $R/VZ0.024_speed/VX20p45_VX2_AT8/m00/controls.json
sim asc4     $R/AB0.02_asc_n4/r0.080/m00/controls.json
sim course3  $R/AB0.02_c3/r0.080/m00/controls.json
sim desc3    $R/RFE_desc_n3/r-0.080/m00/controls.json
sim descdeep $R/ZE_desc_deep/r-0.120/m00/controls.json
sim pushfwd  $R/ZE0.035_p/i11_d0/m00/controls.json
sim pushback_v65 $R/RFE_p/i11_d180/m00/controls.json PUSH_Z_DROP=0
sim pushback_v67 $R/RFE_p/i11_d180/m00/controls.json
sim push7_v66 $R/ZE0.035_p/i7_d90/m02/controls.json REF_RESET_INCL_CURRENT=0
sim push7_v67 $R/ZE0.035_p/i7_d90/m02/controls.json
sim push5sw  $R/ZE0.035_p/i5_d270/m00/controls.json
wait
grep -H -E 'FELL at|verdict|PUSH at' $V/*.log | sed "s|$V/||"
echo "#### SIMS_DONE"
# clips
python3 - <<'PY'
import csv,re
V='log/claude_g1vid5_20261010'
def clip(n,sel):
    rows=list(csv.DictReader(open(f'{V}/{n}.csv'))); keep=[r for r in rows if sel(r,open(f'{V}/{n}.log').read())]
    with open(f'{V}/{n}_clip.csv','w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=rows[0].keys()); w.writeheader(); w.writerows(keep)
    print(n,len(keep),'rows',keep[0]['t'],'..',keep[-1]['t'])
def push(r,log):
    tp=float(re.search(r'PUSH at t=([0-9.]+)',log).group(1)); t=float(r['t']); return tp-2.0<=t<=tp+5.0
for n in ('pushfwd','pushback_v65','pushback_v67','push7_v66','push7_v67','push5sw'): clip(n,push)
clip('speed45',lambda r,l: 5.0<=float(r['t'])<=15.0)
clip('asc4',lambda r,l: 1.4<=float(r['com_x'])<=3.7)
clip('course3',lambda r,l: 1.4<=float(r['com_x'])<=5.6)
clip('desc3',lambda r,l: 1.0<=float(r['t']) and float(r['com_x'])<=1.9)
clip('descdeep',lambda r,l: 1.0<=float(r['t']) and float(r['com_x'])<=1.5)
PY
echo "#### CLIPS_DONE"
# render
export ROBOT=g1_23dof VID_W=800 CAM_FOLLOW=1 LEGEND="|" URDF=$CH/g1_23dof_arms_free.urdf
r() { n=$1; shift; mkdir -p $V/$n; ( export TITLE="$1"; shift; for kv in "$@"; do export "$kv"; done; python3 examples/render_com_squat_video.py $V/${n}_clip.csv $V/$n/clip.mp4 > $V/${n}_render.log 2>&1 ) & }
r speed45  "G1 v6.7 / flat, 0.45 m/s (VX2_Z_DROP bench)" STAIRS=1,0.000,0.211,30.0114
r asc4     "G1 v6.7 / 4 steps up, 80 mm risers" STAIRS=4,0.080,0.211,2.0114 STAIR_LANDING_LEN=3.5
r course3  "G1 v6.7 / up 3 x 80 mm, landing 1.5 m, down 3 x 80 mm" STAIRS=3,0.080,0.211,2.0114 STAIR_LANDING_LEN=1.5 STAIRS_DOWN=3
r desc3    "G1 v6.7 / 3 steps down, 80 mm risers" STAIRS=3,-0.080,0.211,0.4114
r descdeep "G1 v6.7 / 1 step down, 120 mm riser" STAIRS=1,-0.120,0.211,0.4114
r pushfwd  "G1 v6.7 / 11 N*s forward push" STAIRS=1,0.000,0.211,1.9923 PUSH_IMPULSE_LABEL="11 N*s"
r pushback_v65 "11 N*s backward push -- v6.5 (no CoM drop)" STAIRS=1,0.000,0.211,1.9923 PUSH_IMPULSE_LABEL="11 N*s"
r pushback_v67 "11 N*s backward push -- v6.7 (PUSH_Z_DROP)" STAIRS=1,0.000,0.211,1.9923 PUSH_IMPULSE_LABEL="11 N*s"
r push7_v66 "7 N*s push toward stance foot -- v6.6" STAIRS=1,0.000,0.211,1.9923 PUSH_IMPULSE_LABEL="7 N*s"
r push7_v67 "7 N*s push toward stance foot -- v6.7 (re-centre)" STAIRS=1,0.000,0.211,1.9923 PUSH_IMPULSE_LABEL="7 N*s"
r push5sw  "G1 v6.7 / 5 N*s push toward swing foot" STAIRS=1,0.000,0.211,1.9923 PUSH_IMPULSE_LABEL="5 N*s"
wait
echo "#### RENDER_DONE"
for n in speed45 asc4 course3 desc3 descdeep pushfwd push5sw; do ffmpeg -y -loglevel error -i $V/$n/clip.mp4 -vf "pad=iw*2:ih:(ow-iw)/2:0:black" -c:v libx264 -pix_fmt yuv420p $V/${n}_wide.mp4; done
for s in pushback push7; do a=v65; [ $s = push7 ] && a=v66; ffmpeg -y -loglevel error -i $V/${s}_$a/clip.mp4 -i $V/${s}_v67/clip.mp4 -filter_complex "[0:v][1:v]hstack=inputs=2" -c:v libx264 -pix_fmt yuv420p $V/${s}_side.mp4; done
printf "file '%s'\n" $PWD/$V/speed45_wide.mp4 $PWD/$V/asc4_wide.mp4 $PWD/$V/course3_wide.mp4 $PWD/$V/desc3_wide.mp4 $PWD/$V/descdeep_wide.mp4 $PWD/$V/pushfwd_wide.mp4 $PWD/$V/pushback_side.mp4 $PWD/$V/push7_side.mp4 $PWD/$V/push5sw_wide.mp4 > $V/concat.txt
ffmpeg -y -loglevel error -f concat -safe 0 -i $V/concat.txt -c copy $V/g1_v67_evaluation_tour.mp4
for f in $V/*_wide.mp4 $V/*_side.mp4 $V/g1_v67_evaluation_tour.mp4; do echo "$f $(du -h $f | cut -f1) $(ffprobe -v error -show_entries format=duration -of csv=p=0 $f)s"; done
echo "#### VID5_DONE"
