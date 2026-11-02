# G1 前への跳躍の比べ先 HV12k7（2026-11-03）

垂直跳びの基準 TV3R とは別に、前への跳躍の候補を比べる相手。MPC の踏み切りの計画（§11）はこれを上回ることが判定の基準。

- バイナリ: `g1_flight_probe_base_hv12k7`（= j27、articara 1330b66、misa-locomotion def96db）。
- 設定: TV3R の設定から `WBCW_TRUNK_LAND_VZ=0.7` を抜き、次を足す。
  - `"FLIGHT_FEET_CAPTURE":"0.7","FLIGHT_FEET_CAPTURE_T":"0.06","KP_FLIGHT_FEET":"1000","KD_FLIGHT_FEET":"63","WBCW_TRUNK_PUSH":"0.003","WBCW_MOMENTUM":"10","WBCW_CONTACT":"100","WBCW_TRUNK_LAND_VZ":"0","JUMP_VX":"1.2"`
  - 実行例は `jb30.sh`。

| 種 | 結果の TAG | JUMP_V 1.0 | 1.5 | 2.0 | 合計 |
|---|---|---|---|---|---|
| 31415 | `runs/HV12k7v10`（1.0）、`runs/HV12k7`（1.5／2.0） | 37 | 45 | 47 | **129/144** |
| 27182 | `runs/HV12k7s2` | 35 | 40 | 46 | **121/144** |

- 離地の vx の中央値: 1.0 で +0.55、1.5 で +0.46、2.0 で +0.35 m/s（両方の種で同じ）。
- 着地したものの最終の前進の中央値: +0.24／+0.22／+0.17 m。
- 転倒 38 本のうち 35 本が後ろへの転倒。とくに低い跳躍（1.0）で多く、飛行が短いのに足を止まり点（倍率 0.7）まで運ぶ。

経緯は `doc/from_claude/20261021_g1_jump.md` §10。
