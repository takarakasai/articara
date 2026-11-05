# G1 連続の跳躍の基準 R10W（2026-11-05、ユーザ確定）

連続の跳躍（JUMP_REPEAT、着地して立っていれば跳び直す）の候補はこれと比べる。前への跳躍の基準 TB5（`BASELINE_FWD_TB5.md`）の上に積む。

- バイナリ: `g1_flight_probe_base_r10w`（= j40、articara e674b17、misa-locomotion def96db）。
- 設定: TB5 の設定に次を足す（3 回跳び）。
  - `"T":"13","JUMP_REPEAT":"2","FLIGHT_FEET_YAW":"1","JUMP_REPEAT_RECENTRE":"1","WBCW_TRUNK_LAND_T":"1.0","JUMP_REPEAT_SOFT_CROUCH":"1","FLIGHT_FEET_WIDTH":"0.236"`
  - 10 回跳びは `"T":"37","JUMP_REPEAT":"9"`。
  - 実行例は `jb59.sh`・`jb60.sh`（3 回）、`jb61.sh`（10 回）。集計は `rsumm.py`（跳び終えた数・対の比較）、`rdrift.py`（跳躍の番号ごとのずれ）、`hxrep.py`（跳躍ごとの離地 h_x の内訳）。

| 種 | 結果の TAG | 1.0 | 1.5 | 2.0 |
|---|---|---|---|---|
| 31415 | `runs/R10W`（3 回、of 144） | 99 | 144 | 141 |
| 27182 | `runs/R10Ws2`（3 回、of 144） | 102 | 141 | 137 |
| 31415 | `runs/R10W10`（10 回、of 480） | 164 | 478 | 457 |
| 27182 | `runs/R10W10s2`（10 回、of 480） | 164 | 470 | 451 |

- 10 回とも跳んだ本 of 48: 1.0 で 4〜5、1.5 で 47、2.0 で 44〜45。1.5・2.0 はずれが溜まらない。立っている本の 10 回の前進は +2.9〜3.1 m。
- 既知の限界: 1.0 の 2 回目以降は 1 回ごとに約 25% 落ちる（離地 |h_x| ~1）。
- 動画: `g1_repeat_jump_r8tc_vs_r10w.mp4`（`diagh/render6.sh`、git には入れない）。

経緯は `doc/from_claude/20261021_g1_jump.md` §21〜§23。
