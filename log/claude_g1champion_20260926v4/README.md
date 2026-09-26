# G1 チャンピオン v4（2026-09-26）── BD 型: 自由腕・MPC 単独権限

`controls.json` = `log/claude_g1bdbaseline_20260926/controls_v1b.json`（v1b）。URDF は同梱の
`g1_23dof_arms_free.urdf`（溶接版と質量一致 34.134 kg、腕 10 関節を可動化）を参照。
バイナリ `kyo46rs_walk` はコミット 1f515ed 時点（`MPC_SEED_PREVIOUS`, `STAIR_RISE_SOLE_POSE_MIN_DZ` を含む）。

要素: 自由腕 cem5 レシピ + 保持 `ARM_MERGE=0 ARM_BEFORE_SWING=1 KP_ARM=400`
+ MPC 単独権限 `WRENCH_MPC_APPLY=1 STAIR_WRENCH_COUPLED=0 MPC_SOLE_AUTHORITY=1 MPC_NET_COP_W=10 MPC_W_SMOOTH=10`
+ **`MPC_SEED_PREVIOUS=2`**（平滑項を実測力に係留）+ **`STAIR_RISE_SOLE_POSE=0`**。

## 対 v3.1（`log/claude_g1champion_20260924v31`）、対応あり McNemar、N=48

| | v3.1 | v4 | z |
|---|---|---|---|
| 段 SEED=20260924（std） | 145 | **174** | +3.71（各段 −0.82/+1.13/+3.46/+4.24） |
| 段 ホールドアウト 777（std） | 143 | **184** | +5.53 |
| 段 SEED=20260924（wide） | 136 | **159** | +3.22（+0.30/−0.33/+2.50/+3.30） |
| 段 ホールドアウト 777（wide） | 133 | **159** | +3.47 |
| 押し 8 条件（std / wide） | 214 / 203 | **243 / 231** | +4.90 / +4.43 |
| 押し ホールドアウト 777 | 217 | **243** | +4.91（悪化 最大 1 本） |
| 横段階化（std / wide） | 118 / 116 | 122 / 122 | +2.00 / +2.45 |

既知の残差: 1 cm の段で 34 対 41（`doc/from_claude/20260925_mpc_in_the_loop.md` §53）。
検証: `scripts/g1_battery.sh <PREFIX> <controls.json> '{}' [std|wide]` と `scripts/g1_paired.py`。
根拠: §29–56、計画: `doc/road_to_bd_baseline.md`。
