#!/usr/bin/env python3
"""eqpreset_list.py OUT : the S17 preset check's list (walker extraction, doc/from_claude/20261029_walker_extraction_plan.md).

One line per eq sample (eqlist.txt, then eqlist_push.txt): `controls preset drop_keys`.
- preset: g1_v619 for ROBOT=g1, kyo46rs_v2 for ROBOT unset (the kyo46rs samples).
- drop_keys: the sample's keys of the knob inventory's classes 1 / 2 / 1k
  (doc/from_claude/20261029_knob_inventory.md §3, appendix A) that the walker's
  Config reads (examples/kyo46rs_walk/config_env.rs, `config_from_kv`). Class-1
  keys the Config does not own (REF_LIBRARY, BASE_VEL_FUSE_*, ...) stay.
"""
import json, re, sys
R = '/home/takara/work/dp/articara'
L = R + '/log/claude_g1rand_20260925'
cls = {}
for l in open(R + '/doc/from_claude/20261029_knob_inventory.md').read().split('\n'):
    if not l.startswith('| '):
        continue
    cols = [x.strip() for x in l.strip().strip('|').split('|')]
    if len(cols) >= 5 and re.fullmatch(r'`?[A-Za-z][A-Za-z0-9_*]*`?', cols[0]):
        cls[cols[0].strip('`')] = cols[4]  # the class column (main table and appendix A)
src = open(R + '/examples/kyo46rs_walk/config_env.rs').read()
fn = src[src.index('pub fn config_from_kv'):]
cfg_keys = (set(re.findall(r'"([A-Z][A-Z0-9_]+)"', fn)) | {'friction_mu'}) - {'WALKER_PRESET'}
def kclass(k):
    return cls.get('WBCW_*') if k.startswith('WBCW_') else cls.get(k)
with open(sys.argv[1], 'w') as out:
    for lst in ['eqlist.txt', 'eqlist_push.txt']:
        for p in open(L + '/' + lst).read().split():
            d = json.load(open(R + '/' + p))
            preset = 'g1_v619' if d.get('ROBOT') == 'g1' else 'kyo46rs_v2'
            drop = sorted(k for k in d if kclass(k) in {'1', '2', '1k'} and (k in cfg_keys or k.startswith('WBCW_')))
            out.write(f'{p} {preset} {",".join(drop)}\n')
