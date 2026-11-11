#!/bin/bash
# jb75scr.sh: 17.5・20 cm で縁より手前に着いた本（jb74 の K150、jb75scr.txt の 58 本）を base_ht で単発に回し直す。jb75scr.sh 名前 VAR=値 ...
D=/home/takara/work/dp/articara/log/claude_g1jump_20261021; S=$D/scr75
list=$D/jb75scr.txt; name=$1; shift; vars="$*"
mkdir -p $S/scr/$name
cat $list | xargs -P 16 -I{} bash -c "cd $D && BIN=$D/g1_flight_probe_base_ht ./one.sh $S/scr/$name/\$(echo {} | tr / _).log {} $vars"
n=0; ok=0; for f in $S/scr/$name/*.log; do n=$((n+1)); if ! grep -q FELL $f && grep -q 'on the box' $f; then ok=$((ok+1)); fi; done
echo "$name: stood $ok/$n"
