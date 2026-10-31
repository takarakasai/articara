#!/bin/bash
# 押しの小ベンチの 3 便目（2026-10-31）: PUSH_REPLAN の寸法と横の狭め（MIN_W）の切り分け。chain1 の後に回す。
cd /home/takara/work/dp/articara
L=log/claude_kyo46rs_push_20261031
until grep -q CHAIN1_DONE $L/kpb_chain1.out 2>/dev/null; do sleep 10; done
PR='"PUSH_REPLAN":"0.015","PUSH_REPLAN_EMIN":"0.0075","PUSH_REPLAN_MAX_BACK":"0.15","PUSH_REPLAN_MAX_W":"0.20","PUSH_REPLAN_HOLD":"10"'
r() { bash $L/kpb.sh $1 "$2" > $L/kpb_$1.out 2>&1; }
r pr4 "$PR"',"PUSH_REPLAN_MAX_FWD":"0.15","PUSH_REPLAN_MIN_W":"0.12"' &
r pr3 "$PR"',"PUSH_REPLAN_MAX_FWD":"0.08","PUSH_REPLAN_MIN_W":"0.06"' & wait
r pr5 '"PUSH_REPLAN":"0.03","PUSH_REPLAN_HOLD":"10","PUSH_REPLAN_MIN_W":"0.06","PUSH_REPLAN_MAX_FWD":"0.15","PUSH_REPLAN_MAX_BACK":"0.15","PUSH_REPLAN_MAX_W":"0.20"' &
r pr6 "$PR"',"PUSH_REPLAN_MAX_FWD":"0.15","PUSH_REPLAN_MIN_W":"0.06","PUSH_REPLAN_STAND":"0","PUSH_REPLAN_RAMP":"0"' & wait
echo "#### CHAIN2_DONE"
