# G1 跳躍の基準 TV3R（2026-11-02、TV3F を置き換え、ユーザ確定）

- 判定: TV3F と同じ（転倒は重心の高さ 0.30 m を割ったときだけ、`FALL_TILT=1.5`）。
- バイナリ: `g1_flight_probe_base_tv3r`（= j24、articara b426a74、misa-locomotion def96db）。新しいノブが未設定なら TV3F（j21）と run.log がバイト一致。
- 設定: TV3F に `LAND_REF_FOLLOW=2 AIR_ABORT_T=0.08` を足す。
  - `EXTRA='{"JUMP_ABORT":"1","JUMP_PUSH_R":"0.3","QD_CLAMP":"40","DIAG_TD":"1","FLIGHT_FEET_FLAT":"1","DIAG_HX":"1","RELEASE_FIRST_OFF":"1","WBCW_CONTACT_REST":"0.1","WBCW_TRUNK_LAND_VZ":"0.7","WBCW_TRUNK_LAND":"0.03","FALL_TILT":"1.5","LAND_REF_FOLLOW":"2","AIR_ABORT_T":"0.08"}'`

| 種 | 結果の TAG | JUMP_V 1.0 | 1.5 | 2.0 | 合計 |
|---|---|---|---|---|---|
| 31415 | `runs/RF2A` | 46 | 48 | 48 | **142/144**（飛ばず 1、転倒 1） |
| 27182 | `runs/RF2As2` | 47 | 48 | 48 | **143/144**（飛ばず 1） |

TV3F からの差:
- `LAND_REF_FOLLOW=2`: 着地の参照の水平成分を、接地している足の移動ぶんずらす（2.0 の着地後の足のずれによる後ろへの転倒 7 本）。
- `AIR_ABORT_T=0.08`: 両足が同時に抜けない離しを飛行とせず、立ちの吸収に戻す（1.0 の床の上の飛行 2 本、判定は NO FLIGHT）。

経緯は `doc/from_claude/20261021_g1_jump.md` §7。以後の比べ先はこの 2 つの TAG。
