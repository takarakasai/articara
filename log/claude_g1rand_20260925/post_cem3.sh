set -u
cd /home/takara/work/dp/articara
until [ "$(grep -c GAITCEM_DONE log/claude_g1gaitcem_20261002.out)" -ge 2 ]; do sleep 120; done
W=6 bash log/claude_g1rand_20260925/bench_ext2.sh B52 log/claude_g1champion_20260927v52/controls.json > log/claude_g1rand_20260925/bench_b52b.out 2>&1
