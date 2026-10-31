#!/bin/bash
# run1 の後に run2
L=/home/takara/work/dp/articara/log/claude_g1rand_20260925
until grep -q KFAM_RUN1_DONE $L/kfam_run1.out; do sleep 30; done
bash $L/kfam_run2.sh > $L/kfam_run2.out 2>&1
