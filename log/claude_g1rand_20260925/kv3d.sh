set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib
L=log/claude_g1rand_20260925; BIN=$L/kyo46rs_walk_tu29
export SEED=20260924 W=12
for v in o15:'{"Z_COM_OFFSET":"-0.015"}' o0:'{"Z_COM_OFFSET":"0"}' r1:'{"SWING_ROT":"1"}'; do
  n=${v%%:*}; x=${v#*:}
  MODE=stairs START_X=0.40 STAIR_TREAD=0.110 T=28 N=24 DX=0.01 RISERS=0.000,-0.010 TAG=KV3_desc_$n BASE=$L/kyo_desc1_v3c.json BIN=$BIN EXTRA="$x" python3 log/claude_g1rand_20260924/rand.py | sed "s/^/desc $n /"
  PUSH_STAIR_X=30 MODE=push N=24 CONDS=0.3@0,0.9@0,0.9@90,0.9@270 TAG=KV3_push_$n BASE=$L/kyo_base_v3c.json BIN=$BIN EXTRA="$x" python3 log/claude_g1rand_20260924/rand.py | sed "s/^/push $n /"
done
for n in o15 o0 r1; do python3 $L/mcpair.py KV3_desc_v2 KV3_desc_$n stairs; python3 $L/mcpair.py KV3_push_v2 KV3_push_$n push; done
echo "#### KV3D_DONE"
