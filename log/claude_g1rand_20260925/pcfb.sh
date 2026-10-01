set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
C=log/claude_g1champion_20260927v52/controls.json; BIN=log/claude_g1rand_20260925/kyo46rs_walk_pcfb
for w in 10000000 100000000; do
  E="{\"MPC_PCMD\":\"1\",\"MPC_CAPTURE_COP_W\":\"$w\",\"MPC_PCMD_FB\":\"1\"}"
  echo "##### PCMD=1+FB 捕捉行 $w: 押し 8 条件"; TAG=PCFB${w}_p BASE=$C BIN=$BIN MODE=push N=24 W=16 SEED=20260924 EXTRA="$E" CONDS=0@0,8@0,11@0,3@270,5@270,7@90,11@180,14@180 python3 log/claude_g1rand_20260924/rand.py
  echo "##### PCMD=1+FB 捕捉行 $w: std 段"; TAG=PCFB${w}_s BASE=$C BIN=$BIN MODE=stairs N=24 W=16 SEED=20260924 EXTRA="$E" RISERS=0.020,0.040,0.060,0.080 python3 log/claude_g1rand_20260924/rand.py
  echo "##### PCMD=1+FB 捕捉行 $w: wide ホールドアウト"; ( export MU_LO=0.40 MU_HI=0.90 JITTER=0.03 DX=0.05; TAG=PCFB${w}W_h BASE=$C BIN=$BIN MODE=stairs N=24 W=16 SEED=777 EXTRA="$E" RISERS=0.020,0.040,0.060,0.080 python3 log/claude_g1rand_20260924/rand.py )
done
echo "#### PCFB_DONE"
