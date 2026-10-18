cd /home/takara/work/dp/articara
L=log/claude_g1rand_20260925
until grep -q GTER1_DONE $L/gterr1_summary.out; do sleep 60; done
bash $L/gcomb1.sh > $L/gcomb1_summary.out 2>&1
bash $L/gmis1.sh > $L/gmis1_summary.out 2>&1
echo "#### CHAIN_ROB_DONE" >> $L/gmis1_summary.out
