set -u
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib
L=log/claude_g1rand_20260925; C=log/claude_g1champion_20261013v69/controls.json; BIN=$L/kyo46rs_walk_tu11
declare -A V=( [A]="0.50,1.0" [B]="0.40,0.90" [C]="0.45,0.95" )
for k in A B C; do
TAG=BKdrop${k}_p8 BASE=$C BIN=$BIN MODE=push N=8 W=6 SEED=20260924 EXTRA="{\"SWING_Z_HOLD\":\"1\",\"SWING_Z_DROP\":\"${V[$k]}\",\"SWING_XY_DELAY\":\"0.15\",\"CONTACT_ADMIT_LOADED\":\"0.04,0.02\"}" CONDS=14@180,11@180,7@90 python3 log/claude_g1rand_20260924/rand.py &
done
wait
{ for k in A B C; do echo "=== drop ${V[$k]} + d15 + al"; python3 $L/mcpair.py HSX_p BKdrop${k}_p8 push | tail -4; done; echo "#### BK3_DONE"; } > $L/bk3_summary.out 2>&1
