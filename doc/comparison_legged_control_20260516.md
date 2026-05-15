# articara vs legged_control: 外力ロバスト性 session 棚卸し (2026-05-15 〜 16)

## 1. 目的とコンテキスト

2026-05-15 session 開始時点で、 `quadruped-gait` の FullCentroidal MPC は
外力 (例: lateral 4N+ push) に対して **転倒回避できない** 状態でした。
ユーザの要求は「namiashi (2.4 kg 4 脚) で legged_control 同等の外力
ロバスト性を獲得する」 こと。

本ドキュメントは 2 日に渡る session で実施した 10 commit の総括であり、
**legged_control との残存差分** を確定させるものです。

---

## 2. session の commit 履歴 (10 commits)

| # | hash | 内容 | 結果 |
|---|---|---|---|
| 1 | `e26e0fd` | `DEFAULT_CAPTURE_POINT_GAIN_S` を `0.0 → 0.05` | ✅ lateral 4N+ 転倒回避 |
| 2 | `755ada3` | capture-point に **deadband + pulse** API (η-2) | opt-in、 **negative** |
| 3 | `0120a14` | **goal-pose mode** (絶対 (x,y,yaw) 追従) | ✅ lateral 2-4N **active recovery** |
| 4 | `0807e83` | Rhai bindings for goal-pose | ✅ |
| 5 | `05687bf` | GUI panel for goal-pose | ✅ |
| 6 | `d298cf1` | `GaitConfig::transition_fraction` (C1 cost-side) | opt-in、 **negative** |
| 7 | `c1bdfd4` | **C1-2 constraint-side hard f_max ramp** | ✅ lateral 6N で **roll −30 %** |
| 8 | `bcbd2ee` | MPC + MJCF ground friction を **μ=0.5 で統一** | ✅ 真の baseline 確定 |
| 9 | `7f08c17` | `use_mpc_predicted_footstep` (P2) | opt-in、 **negative** (A1 が本道と確認) |
| 10 | `ddd09c8` | GUI **Experimental flags** セクション (4 toggles) | ✅ |

## 3. 大局: legged_control とのアーキテクチャ対比

### 3.1 アーキテクチャ概念図

```text
legged_control:
   cmd_vel / goal
        ↓
   TargetTrajectoriesPublisher
        ↓ (target_pose = current + cmd · T or absolute goal)
   ┌─────────────────────────────┐
   │  OCS2 SLQ/DDP MPC           │
   │  state: [momentum (6),      │
   │          base pose (6),     │
   │          joint_q (12)]      │
   │  input: [GRF (12), q̇ (12)]  │  ← footstep XY が暗黙に含まれる
   │  cost: x_track + u_reg      │       (foot pos = FK(joint_q))
   │  constraints:               │
   │    - swing GRF = 0           │
   │    - stance v_foot = 0       │
   │    - swing v_foot.z = planner │
   │    - friction cone (soft)    │
   └─────────────────────────────┘
        ↓ joint trajectory
   SwingTrajectoryPlanner (NormalVelocityConstraint 用)
        ↓
   Whole-Body Controller (Pinocchio)
        ↓ τ
   real / sim robot

articara (現状):
   cmd_vel / goal
        ↓
   set_velocity_cmd / set_goal_pose_world
        ↓
   ┌─────────────────────────────┐
   │  Footstep planner (Raibert + │  ← 別 planner、 MPC 出力を見ない
   │  cap-pt heuristic + 任意 P2) │     v_hip · 0.5·T + k·v_err
   │  → lift_off, touch_down       │
   └─────────────────────────────┘
        ↓
   IK → joint_q targets
        ↓
   ┌─────────────────────────────┐
   │  FullCentroidalMpc           │
   │  state: [v_com (3), ω (3),   │
   │          base pos (3),       │
   │          euler (3), q (12)]  │
   │  input: [GRF (12), q̇ (12)]   │  ← footstep は固定入力扱い
   │  cost: q_diag・(x − ref)²     │
   │  constraints:                │
   │    - swing GRF = 0            │
   │    - stance v_foot = 0        │
   │    - swing v_foot.z (opt-in) │
   │    - friction cone (hard, pyramid) │
   │    - f_z bound (per-step opt-in C1-2) │
   └─────────────────────────────┘
        ↓ GRF + q̇
   WBC pipeline (misarta)
        ↓ τ
   sim robot
```

### 3.2 制御 philosophy の差

| 観点 | legged_control | articara |
|---|---|---|
| **footstep XY の決定者** | **MPC** (foot pos = FK(joint_q) を state に持つ) | **別 planner** (Raibert + cap-pt heuristic) |
| **外乱応答** | MPC が body 軌道に補正項を planning → 自然に foot がそれに追従 | cap-pt feedback で footstep を直接 shift |
| **friction cone** | soft (slack 付き) | hard (pyramid 近似) |
| **swing 軌道** | cubic/quintic spline + 非ゼロ境界速度 | sin² bump (ゼロ境界速度、 trunk bobbing 対策) |
| **solver** | OCS2 SLQ/DDP + line search + Riccati feedback | condensed QP (clarabel) + 3-iter SQP (no line search) |
| **friction coefficient** | 0.3 (conservative) | 0.5 (sim と一致、 実機 rubber-on-floor 上限) |
| **morphology** | Pinocchio + CppAd で任意 kinematic chain | 解析 3R Jacobian (RPP-quadruped 専用) |

---

## 4. session で **closed** した gap

### 4.1 ✅ Closed: 機能パリティ達成

| 観点 | legged_control | articara (post-session) | コミット |
|---|---|---|---|
| velocity tracking | `cmdVelToTargetTrajectories` | `set_velocity_cmd` | (pre-existing) |
| **絶対 goal tracking** | `goalToTargetTrajectories` | **`set_goal_pose_world`** (Rust + Rhai + GUI) | `0120a14` `0807e83` `05687bf` |
| swing leg z-velocity 制約 | `NormalVelocityConstraintCppAd` | `enable_swing_normal_velocity_constraint` flag (parity 経由 opt-in) | (pre-existing parity infra) |
| stance no-slip 制約 | `ZeroVelocityConstraintCppAd` | 既存 stance no-slip equality | (pre-existing) |
| swing GRF=0 | `ZeroForceConstraint` | 既存 swing GRF=0 equality | (pre-existing) |
| per-step contact schedule | mode schedule (SwitchedModelReferenceManager) | `legged_control_parity` flag (opt-in) | (pre-existing parity infra) |
| **transition phase smoothing** | mode schedule に transition 明示 | **C1-2: constraint-side hard `f_max` ramp** | `c1bdfd4` |
| nominal q_ref | `DEFAULT_JOINT_STATE` | `parity_use_nominal_q_ref` flag (opt-in) | (pre-existing parity infra) |
| closed-loop foothold correction | (MPC 内包) | **cap-pt 0.05 default** | `e26e0fd` |
| sim / control friction 一致 | (両方 0.3) | **両方 μ=0.5 で統一** | `bcbd2ee` |

### 4.2 ⚠️ Negative result (opt-in API として残置)

学習として残した 3 件:

| 試行 | 仮説 | 実装 | bench 結果 | 教訓 |
|---|---|---|---|---|
| η-2 deadband + pulse cap-pt | 「ノイズで小さく、 実外力で大きく補正」 | `set_capture_point_pulse(k, db)` | db=0.05 では発散、 db=0.10 では fall regression | cap-pt の **線形不安定性** は deadband では救えない |
| C1 cost-side transition ramp | GRF reference を ramp すれば impact mitigation できる | `transition_fraction` (cost-side only) | parity baseline と **bit-exact identical** | `r_diag[GRF]=1e-3` の cost weight が小さすぎて MPC が ref を無視 |
| P2 MPC-predicted footstep | legged_control 流に MPC の predicted base に foot を置けば良い | `use_mpc_predicted_footstep` flag | lateral 2-4N で **悪化** (peak dy 1.4-2.7×) | articara MPC は footstep を最適化しないので predicted base は「sliding する」 軌道。 真の解決には A1 必須 |

---

## 5. 依然 legged_control が優位な点

### 5.1 構造的差異 (重要度高)

| # | legged_control | articara | impact | 工数 |
|---|---|---|---|---|
| **A1** | **footstep XY を MPC state に組込み**、 GRF と joint optimize | footstep は別 planner | **lateral 6N の真の解決はここに依存** (P2 で本道と確認) | 大 |
| **A3** | friction cone **soft + slack penalty** | hard pyramid 近似 | `diag_friction_cone_utilization` で ratio 1.4 (= √2 pyramid corner) 観測 → cone は binding ぎりぎり、 soft 化で graceful degradation 余地あり | 中 |
| **A4** | OCS2 SLQ/DDP + line search + Riccati feedback | condensed QP + 3-iter SQP (no line search) | overshoot 状況での収束、 bad init 復帰 | 中〜大 |
| **A2** | whole-body Pinocchio + CppAd で任意 kinematic chain | 解析 3R Jacobian (RPP-quadruped 専用) | namiashi では問題なし、 他 morphology 拡張不可 | 大 |

### 5.2 拡張性 (重要度中)

| # | 内容 | 工数 |
|---|---|---|
| **B3** | warm-start MPC (~50Hz、 articara は ~33Hz) | 小〜中 |
| B1 | 3DOF + 6DOF contact 両対応 (namiashi は点接触で該当なし) | 中 |
| B2 | gait library 豊富 (trot/walk/pace/bound/flying-trot/gallop + 自由定義) vs 4 種 | 中 |

### 5.3 動的応答性 (設計選択)

| # | 内容 |
|---|---|
| C2 | swing 軌道 cubic/quintic + 非ゼロ境界速度。 articara は trunk bobbing 対策で **sin² 意図的** — 設計選択、 trade-off あり |
| C3 | state estimator が controller と tight integration |

### 5.4 実証ギャップ

| # | 内容 |
|---|---|
| D1 | 実機 4-8N push recovery 実証 (Anymal, Unitree) — articara は sim μ=0.5/0.5 で lateral 4N OK、 6N fell |
| D2 | terrain adaptation (height field) |
| D3 | stair walking |

---

## 6. 確定 benchmark (μ=0.5 / μ=0.5、 全 commit 適用後)

`tests/integration_walk.rs::diag_external_force_robustness` 抜粋:

### 6.1 cmd_vel mode

| シナリオ | FullC default (cap-pt 0.05) | parity + C1-2 (trans 0.05 hard) | 評価 |
|---|---|---|---|
| lateral 2N | dy 0.198, ✓ no-fall | dy 0.127 ✓ no-fall | 改善 |
| lateral 4N | dy 0.272 ✓ no-fall | dy 0.129 ✓ no-fall | 改善 |
| **lateral 6N** | dy 0.579 **fell** | dy 0.654 fell | μ=0.5 物理限界 |
| forward 2-6N | dy 0.06-0.10 ✓ recovered | 同 | 全 ✓ |
| vertical 4/8N | dy ~0.09 ✓ recovered | 同 | 全 ✓ |
| yaw 1.5 N·m | dy 0.14 ✓ recovered | 同 | 全 ✓ |

### 6.2 goal-pose mode (`diag_goal_pose_lateral_recovery`)

| シナリオ | cmd_vel mode | **goal_pose mode** |
|---|---|---|
| lateral 2N | dy 0.123 → 0.123 △ | dy 0.026 → **0.009 ✓ recovered** |
| lateral 4N | dy 0.122 → 0.088 △ | dy 0.100 → **0.001 ✓ recovered** (88× 改善) |
| lateral 6N | dy 0.280 △ (no-fall) | dy 0.557 → fell (μ=1.0 sim 時代のデータ、 μ=0.5 で再計測予定) |

### 6.3 friction cone utilization (`diag_friction_cone_utilization`、 μ=1.0 sim 時代)

| シナリオ | peak \|f_xy\|/(μ·f_z) | 意味 |
|---|---|---|
| baseline (no push) | 0.33 | cone 余裕あり |
| lateral 2N | 0.98 | cone ほぼ binding |
| lateral 4N | 1.39 | pyramid corner |
| lateral 6N | **1.41** (≈ √2) | **完全な pyramid corner** = 真の SOC cone の √2 倍まで通している |

---

## 7. namiashi の実用性評価

```text
                cmd_vel mode (recovery_s 5cm)    goal-pose mode (active recovery)
                ─────────────────────────────    ─────────────────────────────
forward 2-6N    ✓ recovered (0s)                 ✓ recovered (0s)
lateral 2N      △ no-recov (dy 0.20m sliding)   ✓ recovered (0.009m 戻る)
lateral 4N      △ no-recov (dy 0.27m sliding)   ✓ recovered (0.001m 戻る)
lateral 6N      ✗ fell (μ=0.5 物理限界)          ✗ fell (同)
vertical 4-8N   ✓ recovered                      ✓ recovered
yaw 1.5 N·m     ✓ recovered                      ✓ recovered
```

→ **実用域 (lateral 4N まで) は session で達成済み**。 lateral 6N は
μ=0.5 環境では物理的に超過 (foot reach 5cm vs body sliding 10cm)。

---

## 8. 残された筋の良い next steps (優先順)

| 優先 | 案 | 期待効果 | 工数 |
|---|---|---|---|
| ★★ | **A3 friction cone soft + slack** | pyramid corner で物理超過の risk を回避 + graceful degradation | 中 |
| ★★ | **B3 MPC warm-start** | reactive 性 33→50Hz | 小〜中 |
| ★ | **A1 footstep XY を MPC state に拡張** | lateral 6N の真の解決 (P2 で本道と確認) | **大** |
| ★ | A4 line-search SQP | overshoot 状況での収束改善 | 中〜大 |
| — | A2 whole-body dynamics | namiashi では不要、 他 morphology 拡張時に検討 | 大 |

---

## 9. 切替経路の整備状況 (3 layers)

session で goal-pose mode + Experimental flags を完備:

| 機能 | Rust API | Rhai | GUI |
|---|---|---|---|
| `set_velocity_cmd` | ✓ | ✓ (`gait_set_velocity`) | ✓ (D-pad / slider) |
| `set_goal_pose_world` | ✓ | ✓ (`gait_set_goal_pose`) | ✓ (🎯 Set goal button) |
| `set_capture_point_gain` | ✓ | ✓ (`set_capture_point_gain`) | ✓ (slider) |
| `set_capture_point_pulse` | ✓ | (未登録) | (未表示) |
| `set_legged_control_parity` | ✓ | (未登録) | **✓** (Experimental flags) |
| `set_parity_use_nominal_q_ref` | ✓ | (未登録) | (未表示) |
| `set_use_mpc_predicted_footstep` | ✓ | (未登録) | **✓** (Experimental flags) |
| `GaitConfig::transition_fraction` | ✓ | (未登録) | **✓** (Experimental flags slider) |
| `GaitConfig::transition_enforce_constraint` | ✓ | (未登録) | **✓** (Experimental flags) |

---

## 10. 関連ファイル / コード参照

### controller
- [quadruped-gait/src/full_centroidal_controller.rs](../quadruped-gait/src/full_centroidal_controller.rs) — goal-pose, P2, cap-pt
- [quadruped-gait/src/full_centroidal_mpc.rs](../quadruped-gait/src/full_centroidal_mpc.rs) — MPC、 C1-2 constraint
- [quadruped-gait/src/mpc_controller.rs](../quadruped-gait/src/mpc_controller.rs) — `DEFAULT_CAPTURE_POINT_GAIN_S`, `capture_point_step`
- [quadruped-gait/src/config.rs](../quadruped-gait/src/config.rs) — `GaitConfig` 拡張、 `stance_weight_at`

### sim
- [src/mjcf.rs](../src/mjcf.rs) — MJCF `<default><geom friction>` 追加

### UI / scripting
- [src/app/gait_panel.rs](../src/app/gait_panel.rs) — goal-pose UI + Experimental flags
- [src/scripting_model.rs](../src/scripting_model.rs) — Rhai bindings
- [scripts/goal_pose_recovery_demo.rhai](../scripts/goal_pose_recovery_demo.rhai) — デモ

### tests
- [tests/integration_walk.rs](../tests/integration_walk.rs) — 4 diag tests:
  - `diag_external_force_robustness` (full bench, 19 rows × 9 scenarios)
  - `diag_goal_pose_lateral_recovery` (goal-pose A/B)
  - `diag_friction_cone_utilization` (cone binding probe)
  - `diag_max_step_length_lateral_recovery` (max_step + P2 sweep)

### legged_control 参照
- [ref/legged_control/legged_interface/src/legged_interface.cpp](../ref/legged_control/legged_interface/src/legged_interface.cpp) — MPC + constraint 構築
- [ref/legged_control/legged_controllers/src/target_trajectories_publisher.cpp](../ref/legged_control/legged_controllers/src/target_trajectories_publisher.cpp) — cmd_vel / goal trajectory 生成
- [ref/legged_control/legged_controllers/config/task.info](../ref/legged_control/legged_controllers/config/task.info) — friction_mu = 0.3、 swing config

### memory
- `memory/project_mpc_frame_bug.md` — session の経緯記録 (C2 → η → C1-2、 goal-pose、 P2 negative)

---

## 11. 結論

namiashi の **実用的な外力ロバスト性は session で達成**:
- lateral 4N まで 倒れず + goal-pose mode で active 元位置 recovery
- forward / vertical / yaw 全 ✓ recovered
- lateral 6N は μ=0.5 環境で物理的超過 — 解決には A1 (footstep XY MPC 化) または μ を上げる (実機の足材質変更) のいずれかが必要

legged_control との残る **本質的な architectural gap** は:
- **A1** (footstep を MPC state に組込み) — 真の lateral 6N 解決の本道
- **A3** (friction cone soft) — pyramid corner 問題の回避
- **B3** (warm-start) — reactive 性向上

これらは別 task として、 必要に応じて取り組み可能。
