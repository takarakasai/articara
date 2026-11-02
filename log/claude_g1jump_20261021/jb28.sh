#!/bin/bash
# H8L の離地の vx を伸ばす（2026-11-02、j26、種 31415）: 押しの前半は CoP が踵の縁に張り付いて前の加速が頭打ち（DIAG_PUSHX）。JUMP_VX を上げる／押しの前に重心を前へ（JUMP_LEAN）。
cd /home/takara/work/dp/articara
source ./setup-mujoco.sh >/dev/null
D=log/claude_g1jump_20261021; B=$D/g1_flight_probe_j26
X='"JUMP_ABORT":"1","JUMP_PUSH_R":"0.3","QD_CLAMP":"40","DIAG_TD":"1","FLIGHT_FEET_FLAT":"1","DIAG_HX":"1","RELEASE_FIRST_OFF":"1","WBCW_CONTACT_REST":"0.1","WBCW_TRUNK_LAND":"0.03","FALL_TILT":"1.5","LAND_REF_FOLLOW":"2","AIR_ABORT_T":"0.08"'
C=',"FLIGHT_FEET_CAPTURE":"1","FLIGHT_FEET_CAPTURE_T":"0.06","KP_FLIGHT_FEET":"1000","KD_FLIGHT_FEET":"63","WBCW_TRUNK_PUSH":"0.003","WBCW_MOMENTUM":"10","WBCW_CONTACT":"100","WBCW_TRUNK_LAND_VZ":"0"'
run() { TAG=$1 SEED=31415 BIN=$B CONDS=1.5,2.0 N=48 W=4 EXTRA="{$X$2}" python3 $D/jrand.py; }
run HV12 ',"JUMP_VX":"1.2"'"$C" &
run HV12L3 ',"JUMP_VX":"1.2","JUMP_LEAN":"0.03"'"$C" &
run HV8L6 ',"JUMP_VX":"0.8","JUMP_LEAN":"0.06"'"$C" &
wait
python3 - <<'PY'
import re,glob,statistics as st
D='log/claude_g1jump_20261021/runs'
for tag in ['H8L','HV12','HV12L3','HV8L6']:
  for c in ['1.5','2.0']:
    L=[];vx=[];dx=[];hy=[];fb=[]
    for f in sorted(glob.glob(f'{D}/{tag}/v{c}/m*/run.log')):
      t=open(f).read(); v=re.search(r'verdict: (.*)',t).group(1); L.append(v)
      m=re.search(r'forward: lift-off vx (\S+), touchdown CoM dx (\S+), final CoM dx (\S+)',t)
      lo=re.search(r'\[lift-off\][^\n]*h_ang \(([^,]+),([^,]+),',t)
      if m and m.group(1)!='NaN': vx.append(float(m.group(1)))
      if m and v=='LANDED': dx.append(float(m.group(3)))
      if m and v=='FELL': fb.append('前' if float(m.group(3))>0 else '後')
      if lo: hy.append(float(lo.group(2)))
    med=lambda a: st.median(a) if a else float('nan')
    print(f"{tag:6s} V={c}: 着地 {L.count('LANDED')}/48 転倒 {L.count('FELL')}（前 {fb.count('前')} 後 {fb.count('後')}） | 離地vx {med(vx):+.3f} | h_y {med(hy):+.2f} | 着地の最終dx 中央 {med(dx):+.3f} 最大 {max(dx) if dx else float('nan'):+.3f}")
PY
echo "#### JB28_DONE"
