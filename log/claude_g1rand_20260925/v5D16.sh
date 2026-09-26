set -u
cd /home/takara/work/dp/articara
C=log/claude_g1bdbaseline_20260926/controls_v5_candidate_D16.json; BIN=log/claude_g1rand_20260925/kyo46rs_walk_ftprobe
W=13 BIN=$BIN scripts/g1_battery.sh V5D16 $C '{}' std
echo "#### V5D16_DONE"
