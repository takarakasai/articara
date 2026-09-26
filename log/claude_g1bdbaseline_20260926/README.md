# BD 型ベースライン v1（2026-09-26）── MPC が CoP・接触力の唯一の権限、自由腕 G1

`controls.json` = 自由腕 cem5 の解 + 保持（`ARM_MERGE=0 ARM_BEFORE_SWING=1 KP_ARM=400 KD_ARM=40`）
+ 筋 A（`WRENCH_MPC_APPLY=1 STAIR_WRENCH_COUPLED=0 MPC_SOLE_AUTHORITY=1 MPC_NET_COP_W=10 MPC_W_SMOOTH=10`）
+ **`MPC_SEED_PREVIOUS=2`**（平滑項の係留を実測力に）。3a（`MPC_COP_REF_CMD`）は入れない（効かない）。
URDF: `log/claude_g1arms_20260925/g1_23dof_arms_free.urdf`（質量 34.134 kg、溶接版と一致）。
バイナリ: コミット後の `target/release/examples/kyo46rs_walk`（`MPC_SEED_PREVIOUS` を含むもの）。

## 対 v3.1（溶接チャンピオン、対応あり McNemar、N=48）

| | v3.1 | ベースライン | z |
|---|---|---|---|
| 段 SEED=20260924 | 145 | **157** | +1.43 |
| 段 ホールドアウト 777 | 143 | 139 | −0.49 |
| 押し 8 条件 | 214 | **243** | **+4.90**（悪化条件 0） |
| 横段階化 3〜7 N·s | 118 | 122 | +2.00（悪化 0） |

段別（ホールドアウト）: **20mm 42→21（z=−4.38）**、40mm 44→39（−1.89）、60mm 31→42（+2.84）、80mm 26→37（+2.29）。

## 判定
- ロードマップ §1 の採用基準（自由腕・MPC 単独権限・段と押しで v3.1 と同等）は**総合では満たす**。
- **既知の欠点: 20 mm 蹴上げで有意に劣る。** これは自由腕 cem5 レシピの輪郭（筋 A なしでも 20mm 25/48）で
  MPC 権限の影響ではない（筋 A あり／なし z=+0.14）。修正は 20mm を重くした CEM の再調整で、構造の問題ではない。
- **チャンピオン（v3.1）の置き換えではない。** 各段 z>−1 の規則を 20mm で満たさない。
  「BD 型アーキテクチャがパリティで成立した基準構成」として固定する。

## 検証データ
`log/claude_g1rand_20260924/{U_f_no3a, V_h_no3a, V_p_no3a, V_lat_no3a}`（3a なし）、
`{T_f_s2, V_h_3a, U_f_p, U_f_lat}`（3a あり）。根拠は `doc/from_claude/20260925_mpc_in_the_loop.md` §44–46。
