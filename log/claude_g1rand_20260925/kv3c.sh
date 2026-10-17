set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib
L=log/claude_g1rand_20260925; BIN=$L/kyo46rs_walk_tu29
export SEED=20260924 W=12
for v in v2 v3c; do
  MODE=stairs START_X=30 STAIR_TREAD=0.110 T=20 N=24 DX=0.01 RISERS=0.000 TAG=KV3_flat_$v BASE=$L/kyo_base_$v.json BIN=$BIN EXTRA='{}' python3 log/claude_g1rand_20260924/rand.py | sed "s/^/flat $v /"
  MODE=stairs START_X=0.40 STAIR_TREAD=0.110 T=28 N=24 DX=0.01 RISERS=0.000,-0.010 TAG=KV3_desc_$v BASE=$L/kyo_desc1_$v.json BIN=$BIN EXTRA='{}' python3 log/claude_g1rand_20260924/rand.py | sed "s/^/desc $v /"
  PUSH_STAIR_X=30 MODE=push N=24 CONDS=0.3@0,0.3@180,0.3@90,0.3@270,0.6@0,0.6@180,0.6@90,0.6@270,0.9@0,0.9@180,0.9@90,0.9@270 TAG=KV3_push_$v BASE=$L/kyo_base_$v.json BIN=$BIN EXTRA='{}' python3 log/claude_g1rand_20260924/rand.py | sed "s/^/push $v /"
done
for b in flat desc; do python3 $L/mcpair.py KV3_${b}_v2 KV3_${b}_v3c stairs; done
python3 $L/mcpair.py KV3_push_v2 KV3_push_v3c push
echo "#### KV3_DONE"
