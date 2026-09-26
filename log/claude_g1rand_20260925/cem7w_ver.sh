cd /home/takara/work/dp/articara
python3 -c "
import json; b=json.load(open('log/claude_g1armcem7w_verify.json')); b['STAIR_RISE_SOLE_POSE']='0'; json.dump(b,open('log/claude_g1armcem7w_verify_rp0.json','w'),indent=1)"
W=13 scripts/g1_battery.sh C7W    log/claude_g1armcem7w_verify.json '{}' std
W=13 scripts/g1_battery.sh C7Wrp0 log/claude_g1armcem7w_verify_rp0.json '{}' std
echo "#### CEM7W_VER_DONE"
