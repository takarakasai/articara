# kyo46rs 基準 v3（2026-10-31）── v2 ＋ 段の区間の 5 ノブ（ユーザ判断で昇格）

`controls.json` = `log/claude_g1rand_20260925/kyo_base_v3.json`（複写）。プリセットは `WALKER_PRESET=kyo46rs_v3`（`Config::kyo46rs_v3()`、commit 9062350）。バイナリは `log/claude_g1rand_20260925/kyo46rs_walk_tu63`。URDF は 9 月 12 日の再現用（`log/5_step_c026s009_jerk100_repeat_astra_20260912/model.urdf`）。

中身: v2（9 月の基準 ＋ `CONTACT_ADMIT_SCHEDULED=1 CONTACT_ON_BW=0.05`）に
`STAIR_SWING_FF=0 STAIR_ABSOLUTE_TD_DESC=1 STAIR_ALIGN_STAND_LINE=1 STAIR_ZONE_CROUCH=0.02 STAIR_ZONE_SWING_ROT=3`。
段の区間の外では v2 とビット同一。

## 成績（家族 `kfamily.sh` 8 ベンチ 1,552 本、3 つの種、対 v2）
- 改善 253・悪化 0（z +15.9）。種ごとに 86／0、88／0、79／0。
- 降段（N=144）: 0 mm 81→144、10 mm 15→142、20 mm 0→63。
- 段の外の 7 ベンチは run.log 4,224 本が v2 と一致（`kfam_bit.sh`）。
- 判定 `log/claude_g1rand_20260925/kfam_judge_v2v3.out`（改定した規則 A・B で悪化なし）。

基準は `KFAM_k3n_*`（31415／27182）・`KFAM_k3o_*`（27182／31415）・`KFAM_k3t_*`（16180／14142）。
判定は `python3 log/claude_g1rand_20260925/family_judge.py --kyo k3n:CANDn k3o:CANDo k3t:CANDt`。

植物の漏れ: `JOINT_Q_EST=1` は中立（入れなかった）、`MU_ASSUMED=0.58` は前押し 0.9 N·s が悪化（採らない）。
弱点と次の一手: `doc/from_claude/20261031_kyo46rs_family.md`。
