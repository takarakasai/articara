set -u
cd /home/takara/work/dp/articara
C=log/claude_g1bdbaseline_20260926/controls_v5_candidate_C.json; BIN=log/claude_g1rand_20260925/kyo46rs_walk_ftprobe
W=13 BIN=$BIN scripts/g1_battery.sh V5C $C '{}' std
echo "#### V5C_DONE"
