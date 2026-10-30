#!/bin/bash
# eqpreset.sh BIN OUT : the S17 check of WALKER_PRESET (doc/from_claude/20261029_walker_extraction_plan.md §5.1 S17).
# 1. eqpreset_list.py: per eq sample (96 + 14), its preset and its control keys (inventory classes 1 / 2 / 1k
#    that the walker's Config reads).
# 2. The Config as read (WALKER_CONFIG_DUMP, T=0) from the full controls and from the controls without those
#    keys plus WALKER_PRESET. A sample whose two dumps differ restates a control value the preset does not
#    hold (a per-bench value, or a candidate's); it is listed in OUT/excluded.txt with the differing lines
#    and not run.
# 3. The rest run without the keys and with WALKER_PRESET, TRAJ_CSV and the log into OUT/<id>.{csv,log}, for
#    eqpresetcmp.sh against the full-controls references (eq.sh's eq_ref / eq_push).
# eq.sh is not used (do not edit it while it runs elsewhere).
cd /home/takara/work/dp/articara
export LD_LIBRARY_PATH=/home/takara/.mujoco/mujoco-3.8.0/lib
L=/home/takara/work/dp/articara/log/claude_g1rand_20260925
BIN=$(readlink -f $1); OUT=$2; mkdir -p $OUT/dump
python3 $L/eqpreset_list.py $OUT/list.txt
env_of() {  # controls drop -> export lines
  python3 -c "import json,shlex,sys;D=set(sys.argv[2].split(','));[print('export %s=%s'%(k,shlex.quote(str(v)))) for k,v in json.load(open(sys.argv[1])).items() if k not in D]" "$1" "$2"
}
idof() { echo $1 | sed 's#log/claude_g1rand_20260924/##; s#/controls.json##; s#/#__#g'; }
dump() {
  set -- $1; c=$1; preset=$2; drop=$3; id=$(idof $c)
  ( eval "$(env_of $c '')"; export T=0 WALKER_CONFIG_DUMP=$OUT/dump/$id.full.txt; $BIN > /dev/null 2>&1 )
  ( eval "$(env_of $c $drop)"; export WALKER_PRESET=$preset T=0 WALKER_CONFIG_DUMP=$OUT/dump/$id.preset.txt; $BIN > /dev/null 2>&1 )
}
run() {
  set -- $1; c=$1; preset=$2; drop=$3; id=$(idof $c)
  ( eval "$(env_of $c $drop)"; export WALKER_PRESET=$preset TRAJ_CSV=$OUT/$id.csv; $BIN > $OUT/$id.log 2>&1 )
}
export -f env_of idof dump run; export BIN OUT
xargs -P 24 -I{} bash -c 'dump "{}"' < $OUT/list.txt
: > $OUT/targets.txt; : > $OUT/excluded.txt
while read c preset drop; do id=$(idof $c)
  if cmp -s $OUT/dump/$id.full.txt $OUT/dump/$id.preset.txt; then echo "$c $preset $drop" >> $OUT/targets.txt
  else echo "$id $preset: $(diff $OUT/dump/$id.full.txt $OUT/dump/$id.preset.txt | grep '^[<>]' | tr -s ' ' | tr '\n' ' ')" >> $OUT/excluded.txt; fi
done < $OUT/list.txt
echo "targets $(wc -l < $OUT/targets.txt), excluded $(wc -l < $OUT/excluded.txt)"
xargs -P 24 -I{} bash -c 'run "{}"' < $OUT/targets.txt
echo "#### EQPRESET_DONE $OUT"
