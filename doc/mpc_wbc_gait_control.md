# MPC + WBC 歩行制御スタック設計

## 目的

[`ref/legged_control`](https://github.com/qiayuanliao/legged_control) (Liao 2022) の
NMPC + WBC + Kalman 構成を articara 上に misarta を使って再実装し、
SRBD MPC のみの現状から**摩擦コーン・トルク限界を制御層で厳格に守る**
階層 QP 制御へアップグレードする。

## 現状 vs 目標

| 層 | legged_control | articara (現状) | 目標 |
|---|---|---|---|
| MPC | OCS2 SQP NMPC (centroidal) | SRBD MPC (Di Carlo 2018) | **SRBD MPC + 強化 WBC で代替** (NMPC 移植は Phase D) |
| WBC | **3-priority Hierarchical QP** | `τ_ff = -J^T·f_GRF` 単層 | 階層 QP に置換 |
| 状態推定 | 18-state Linear Kalman | Madgwick + 脚オドメトリ (非融合) | EKF で融合 |
| 接触検出 | 物理接触センサ | 位相生成器の固定スケジュール | 接触駆動 phase |
| HW I/F | ROS Control + Hybrid Joint | MuJoCo 直接 | 据え置き |

## 前提 — misarta が pinocchio 相当を完備

| pinocchio API | misarta 対応 |
|---|---|
| `crba` (mass matrix) | [`crba::crba`](../misarta/src/crba.rs) |
| `rnea`, `nonLinearEffects`, `compute_gravity` | [`rnea`](../misarta/src/rnea.rs) |
| `aba` (forward dyn.) | [`aba::aba`](../misarta/src/aba.rs) |
| `getFrameJacobian`, `getFrameJacobianTimeVariation` | [`jacobian`](../misarta/src/jacobian.rs) |
| Centroidal model | [`centroidal`](../misarta/src/centroidal.rs) |
| `forwardKinematics` (位置/速度/加速度) | [`fk`](../misarta/src/fk.rs) |
| Constraint Jacobian | [`constraint::jacobian`](../misarta/src/constraint/jacobian.rs) |
| Constrained / impact dynamics | [`constrained`](../misarta/src/constrained.rs) |
| QP solver | [`qp::solve_qp`](../misarta/src/qp.rs) (clarabel + active-set) |

→ ピノキオ依存をほぼ素のまま misarta に置換できる。OCS2 NMPC は別物
なので置換せず、SRBD MPC + 強化 WBC で代替する。

## アーキテクチャ

```
┌─────────────────────────────────────────────────────┐
│            (User cmd via GUI / Rhai)               │
│            vx, vy, wz                              │
└─────────────────────────────────────────────────────┘
                       │
                       ▼
┌─────────────────────────────────────────────────────┐
│  Phase generator (open-loop)                       │
│  + Contact-driven correction (Phase C)             │
│  → stance flags [4], sub_fraction [4]              │
└─────────────────────────────────────────────────────┘
                       │
                       ▼
┌─────────────────────────────────────────────────────┐
│  Footstep planner                                  │
│  Raibert + capture-point + horizon-bias            │
│  → foot targets in body frame                      │
└─────────────────────────────────────────────────────┘
                       │
                       ▼
┌─────────────────────────────────────────────────────┐
│  IK (per leg)                                      │
│  → q*[12], q̇*[12]                                  │
└─────────────────────────────────────────────────────┘
                       │
                       ▼
┌─────────────────────────────────────────────────────┐
│  SRBD MPC (Di Carlo 2018) — Phase D で full NMPC ?  │
│  current state + contact schedule + reference      │
│  → f_GRF[4] over horizon                           │
└─────────────────────────────────────────────────────┘
                       │
                       ▼
┌─────────────────────────────────────────────────────┐
│  Hierarchical WBC (Phase A) ← NEW                  │
│  decision: q̈[18], f_GRF[12], τ[12]                  │
│                                                     │
│  Task 0 (hard):                                    │
│    - Floating-base EoM:  M·q̈ + h = S^T·τ + J^T·f   │
│    - Torque limits:      |τ| ≤ τ_max               │
│    - Friction cone:      |f_xy| ≤ μ·f_z, f_z ≥ 0   │
│    - No contact motion:  J_c·q̈ + J̇_c·v = 0          │
│                                                     │
│  Task 1 (soft):                                    │
│    - Base accel:         J_b·q̈ + J̇_b·v = a_b_des   │
│    - Swing leg:          J_sw·q̈ + J̇_sw·v = a_sw_des │
│                                                     │
│  Task 2:                                           │
│    - Contact force reg:  f_GRF ≈ f_MPC             │
└─────────────────────────────────────────────────────┘
                       │
                       ▼
┌─────────────────────────────────────────────────────┐
│  τ → MuJoCo motor.ctrl                             │
└─────────────────────────────────────────────────────┘

         ▲                            ▲
         │                            │
┌────────────────────┐    ┌──────────────────────────┐
│ State Estimator    │    │  IMU + Joint encoders    │
│ (Phase B: EKF)     │◀───│  + Contact sensors       │
│ → q, q̇, body pose  │    └──────────────────────────┘
└────────────────────┘
```

## Phase 計画

### Phase A — Hierarchical WBC (推奨第一着手)

**動機:** 現状 `τ_ff` は SRBD MPC の GRF を `-J^T` で逆射影しただけで
位置 PD に上乗せしている。摩擦コーン違反・トルク限界突破は事後 clip
のみで「制御層では何も保証していない」。階層 QP で**制約を厳格に守
りつつ可能な限り追従する**仕組みに置換する。

**実装するもの (`quadruped-gait/src/wbc/`):**

```
wbc/
├── mod.rs            -- Wbc トップレベル
├── ho_qp.rs          -- HoQp 階層 QP ソルバ (3 priority)
├── task.rs           -- Task { A_eq, b_eq, A_iq, b_iq } 構造体
└── tasks/
    ├── floating_base_eom.rs   -- M·q̈ + h - S^T·τ - J^T·f = 0
    ├── torque_limits.rs       -- |τ| ≤ τ_max
    ├── friction_cone.rs       -- |f_xy| ≤ μ·f_z, f_z ≥ 0
    ├── no_contact_motion.rs   -- J_c·q̈ + J̇_c·v = 0
    ├── base_accel.rs          -- J_b·q̈ + J̇_b·v = a_b_des
    ├── swing_leg.rs           -- J_sw·q̈ + J̇_sw·v = a_sw_des
    └── contact_force.rs       -- f_GRF ≈ f_MPC
```

**Decision variable layout (`x ∈ R^(nv + 3·nc + na)`):**
- `q̈ ∈ R^nv` (一般化加速度、nv = 6 + 12 = 18 for namiashi)
- `f ∈ R^(3·nc)` (4 接触 × 3軸 = 12)
- `τ ∈ R^na` (12 actuator)

**HoQp 数式 (Kim 2014 / Bouyarmane 2018):**

各 priority k で:
```
min_x ||A_k x − b_k||²   (soft equality)
s.t.  D_k x ≤ f_k        (hard inequality)
      A_j x = A_j x_{j−1}* (∀ j < k)   ← 高 priority の解空間は変えない
      D_j x ≤ f_j         (∀ j < k)
```

実装は再帰: `HoQp::new(task_2, Some(HoQp::new(task_1, Some(HoQp::new(task_0, None)))))`

各レベルの内部 QP は misarta `solve_qp` (clarabel) を使用。

**misarta API 必要箇所:**
- `crba` → M(q)
- `nonlinear_effects` → h(q, q̇) = C(q,q̇)q̇ + g(q)
- `compute_joint_jacobian` (LOCAL_WORLD_ALIGNED) → 各足 J
- `compute_joint_jacobian_time_derivative` → J̇·v
- `solve_qp` → 内部 QP

**工数感:** 1〜2 週間。HoQp 本体 ~200 行、各 Task ~50〜100 行。

**期待効果:**
- 摩擦コーン違反による foot slip が制御層で防げる
- トルク限界突破不能 (位置 PD が突き抜けることがない)
- swing 脚の高速移動・加速で発生する突発トルクの飽和回避
- 不整地・スリップ時の robust 性が legged_control 並みに

### Phase B — Linear Kalman Filter 状態推定器

**動機:** 現状 `Madgwick` (姿勢) と `LegOdometry` (位置) は独立で動い
ており、互いを補正しない。実機では IMU・接触センサ・関節エンコーダ
を **18-state Kalman で融合**するのが定石。

**18-state model:**
```
x = [body_pos(3); body_vel(3); foot_pos_world(12)]
```

**観測 z:**
- 各足の body 相対位置 (FK から)
- 各足の body 相対速度 (J·q̇ から)
- 接触センサの z=0 制約 (接地中の足)

**実装するもの (`src/estimator/`):**
```
estimator/
├── mod.rs                  -- StateEstimateBase trait
├── linear_kalman.rs        -- KalmanFilterEstimate
└── from_topic.rs (将来)    -- 外部 odom 融合
```

**misarta API:**
- `forward_kinematics` (浮動ベース込み)
- `compute_joint_jacobian` (観測 Jacobian)

**現状からの差分:**
- 既存 `MadgwickAhrs` は **観測の前処理** として残る (gyro/accel → 姿勢)
- 既存 `LegOdometry` は **比較用 baseline** として保持
- `PoseSource::ExtendedKalman` を追加し、UI で切替

**工数感:** 1 週間程度。

### Phase C — 接触駆動 stance schedule

**動機:** 固定周期 phase は外乱に弱い。接触センサで「想定より早く接
地した／離れない」を検出して `stance` フラグを補正する。

**実装するもの:**

```rust
// quadruped-gait/src/phase.rs に追加
pub struct ContactDrivenPhase {
    nominal: PhaseGenerator,
    early_contact_threshold_n: f64,
    late_liftoff_threshold_n: f64,
}

impl ContactDrivenPhase {
    pub fn step(
        &mut self,
        dt: f64,
        cmd: &VelocityCmd,
        contact_force: [f64; 4],   // f_z per foot from sensor
    ) -> [PhaseState; 4]
}
```

**MuJoCo 側:**
- `MujocoSim::contact_force_per_foot(robot) -> [f64; 4]` を新設

**工数感:** 数日。

### Phase D — Full-body NMPC (long-term, 任意)

OCS2 NMPC は外部依存重く Rust 移植非現実的。代替:

- **選択肢 1:** SRBD MPC + Phase A〜C で「実機系で十分」とする (現実的)
- **選択肢 2:** 自前 SQP/iLQR NMPC (数ヶ月工事)

**推奨:** 選択肢 1。Phase A〜C で legged_control の主要 robustness 機能はカバー。

## 着手順序

```
1. Phase A (Hierarchical WBC)         ⭐⭐⭐ 最大効果
2. Phase C (接触駆動)                 ⭐⭐  実装が軽い
3. Phase B (EKF 状態推定)             ⭐   ground truth で動くなら後回し可
4. Phase D は要相談                    
```

## クレート配置の方針

| 機能 | 置き場所 | 理由 |
|---|---|---|
| `HoQp` 階層 QP ソルバ | `quadruped-gait::wbc::ho_qp` | gait と一体運用、misarta は **基礎数学**のみに専念 |
| WBC タスク + 統合 | `quadruped-gait::wbc::tasks::*` | gait 制御の一部、`KinematicsConfig` 等の依存あり |
| EKF 状態推定 | `articara::estimator` | host-specific (MuJoCo / IMU / joint state を 1 段で受ける) |
| 接触駆動 phase | `quadruped-gait::phase::ContactDrivenPhase` | 既存 `PhaseGenerator` の拡張 |

legged_control の `legged_wbc` package と同じ粒度を採用。`HoQp` 本体は
純粋数学なので misarta 行きも検討したが、現状は WBC タスクと一体で
使うため `quadruped-gait` に置く。後で需要が出れば misarta 側に切り
出してもよい。

## 現状で legged_control に勝っている点

- **Rust** 化 (memory-safe、segfault フリー)
- MuJoCo **直結** (ROS 中間レイヤなし)
- **Rhai スクリプティング** で実験リプロ可
- **GUI viewer + viewport overlay** が初めから統合
- `PoseSource` picker で sim oracle vs estimator の **A/B 比較**容易

これらは保持したままアルゴリズム部分だけ置き換える。
