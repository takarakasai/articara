# G1 跳躍の基準 TV3F（2026-11-02、TV3 を転倒の判定だけ読み直したもの）

- 判定: 転倒は **重心の高さ 0.30 m を割ったときだけ**（`FALL_TILT=1.5` で胴の傾きの判定を事実上外す）。0.52 rad の傾きの判定は、立ち直る 8 本を転倒と数えていた（`doc/from_claude/20261021_g1_jump.md` §6、ユーザ確定 11-02）。
- バイナリ: `g1_flight_probe_base_tv3f`（= j21、articara 9e643bf、misa-locomotion def96db）。`FALL_TILT` 未設定なら TV3（j19）と run.log がバイト一致。
- 設定: TV3 と同じ（`BASELINE_TV3.md`）に `FALL_TILT=1.5` を足す。
  - `EXTRA='{"JUMP_ABORT":"1","JUMP_PUSH_R":"0.3","QD_CLAMP":"40","DIAG_TD":"1","FLIGHT_FEET_FLAT":"1","DIAG_HX":"1","RELEASE_FIRST_OFF":"1","WBCW_CONTACT_REST":"0.1","WBCW_TRUNK_LAND_VZ":"0.7","WBCW_TRUNK_LAND":"0.03","FALL_TILT":"1.5"}'`

| 種 | 結果の TAG | JUMP_V 1.0 | 1.5 | 2.0 | 合計 |
|---|---|---|---|---|---|
| 31415 | `runs/FT15` | 47 | 48 | 42 | **137/144** |
| 27182 | `runs/FT15s2` | 47 | 48 | 47 | **142/144** |

残る転倒は 288 本中 9 本（2.0 で 6＋1、1.0 で 1＋1）。以後の比べ先はこの 2 つの TAG。
