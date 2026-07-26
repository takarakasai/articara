# OpenSoT 調査メモ — 動力学 WBC 機能追加に向けて

`ref/OpenSoT/`(ADVRHumanoids/OpenSoT、commit `3a92addd`)の調査結果。
articara に**動力学(inverse-dynamics / 力制御)機能**を足すための設計参照。

- **OpenSoT とは**: 線形制約下のロボット**階層的全身制御(hierarchical
  whole-body control)**に特化した C++ ライブラリ。IEEE RAM 2025 で解説論文。
  中身は「タスクと制約を**優先度付き QP のスタック**にコンパイルして解く」
  フレームワーク。速度(IK)/加速度(ID)/力の3レベルを同じ抽象で扱う。
- **なぜ参照するか**: articara は既に quadruped-gait に3優先度の HoQp
  WBC(§7)を持つが、それは Go2 歩行に特化した固定構成。OpenSoT は
  **タスク/制約を部品化して任意に組み替える**設計で、汎用の動力学機能
  (任意リンクの力制御、CoP/接触レンチ錐、CBF 関節制限、複数接触の
  力分配 QP など)を足すときの「語彙」と「数式」の宝庫になる。

この文書の構成:
1. コア抽象(Task / Constraint / Solver / Affine)
2. QP ソルバ(iHQP 階層、バックエンド)
3. 速度レベル(IK)タスク
4. 加速度レベル(ID)タスク ← **動力学の本丸**
5. 力レベルタスク
6. 制約(特に摩擦錐・トルク限界・関節限界 CBF)
7. articara 既存資産との対応と差分
8. misarta が提供すべき量(チェックリスト)
9. 移植の設計提案

---

## 1. コア抽象

全体が `template<Matrix_type, Vector_type>` だが実質 `<Eigen::MatrixXd,
VectorXd>` の密行列。3つの直交概念に分かれる:

### 1.1 Task — 重み付き最小二乗目的 `‖A·x − b‖²_W`

`include/OpenSoT/Task.h`。**タスク自身は優先度を持たない**。主要メンバ:

```cpp
Matrix_type _A;      // タスク Jacobian (rows = タスク次元, cols = x_size)
Vector_type _b;      // タスク参照 / 誤差
Matrix_type _W;      // 重み (タスク次元 × タスク次元, PD)。x でなく誤差空間の重み
double      _lambda; // 誤差スケールゲイン (>= 0)
std::list<ConstraintPtr> _constraints;  // このタスクにローカルな制約
```

- **唯一の純粋仮想 `_update()`** が `A, b`(必要なら `W, c`)を現在の
  ロボット状態から再計算する。公開 `update()` が `_update()` を包んで
  制約更新・非アクティブ化・関節マスクを適用。ソルバは `update()` を呼ぶ。
- ソルバ用に `getWA()`(= `W*A`)、`getWb()`、`getATranspose()` を提供。
  これで Hessian `AᵀWA` を安く組める。

### 1.2 Constraint — 境界 / 等式 / 不等式

`include/OpenSoT/Constraint.h`。1つのクラスで3種を表現:

```cpp
Vector_type _lowerBound, _upperBound;                // 箱: l <= x <= u
Matrix_type _Aeq;   Vector_type _beq;                // 等式: Aeq x = beq
Matrix_type _Aineq; Vector_type _bLowerBound, _bUpperBound;  // 不等式: bL <= Aineq x <= bU
```

- **bound(`A=I` の箱)と constraint(一般線形)の区別が肝**: bound は QP の
  `l/u` スロットへ、constraint は `A/lA/uA` スロットへ流れる。
- 片側不等式は `bLowerBound = -1e20`(≈ −∞)の番兵で表す。汎用形は
  **`Aineq · x <= bUpperBound`**。
- タスクへの付き方は2スコープ: (a) タスクローカル(`Task::_constraints`、
  `task << constraint`)、(b) グローバル境界(AutoStack が全レベルに注入)。
- `TaskToConstraint` でタスクを等式制約に降格できる(`‖Ax−b‖` → `Ax=b`)。

### 1.3 AffineHelper — 変数レイアウト抽象(**移植で最重要**)

`include/OpenSoT/utils/Affine.h`。**大域最適化ベクトル `x` と局所変数 `y`
のアフィン写像** `y = M·x + q` をモデル化。

```cpp
class AffineHelper {
    MatrixXd _M; VectorXd _q;
    int getInputSize()  const;  // = M.cols() = 大域 x のサイズ
    int getOutputSize() const;  // = M.rows() = 局所 y のサイズ
    void getValue(x, y) { y = M*x + q; }
};
```

演算子オーバーロードで `matrix*affine`、`affine±affine`、`affine±vector`、
`affine/affine`(縦積み=変数直列化)、`.segment/.head/.tail`(切り出し)、
`Identity(n)` が合成可能。

- **`OptvarHelper`**: `(name, size)` の並びから大域ベクトル `x` を
  レイアウトし、各変数の選択 `AffineHelper` を返す。例:
  `x = [qddot(nv); F_c1(6); F_c2(6)]` を宣言し、各ブロックのアフィンを得る。
- **これが再利用の心臓**。タスクを「`qddot` について」「`F` について」
  書けば、大域ベクトルが何であれ QP 行列が自動で組み上がる。同じ摩擦錐
  コードが `x=[qddot,F]` でも `x=[F]` でも動く。接触の enable/disable も
  「ブロックを落とす」だけ。
- **`variables::Torque`** が模範例: トルクを `(qddot, F)` のアフィンとして
  表現(§4.0 参照)。

**タスク組み立ての普遍パターン**: どのタスクも 0 に駆動したい残差
`e(x) = M·x + q` を作り、`_A = e.getM(); _b = -e.getq();` とする。ソルバは
`‖Ax−b‖²_W = ‖Mx+q‖²_W` を最小化。等式制約として使うときは同じ `M,q` で
`Mx = -q`。この規約が全ファイルで一貫。

### 1.4 AutoStack — 「タスクの数式(Math of Tasks)」DSL

`include/OpenSoT/utils/AutoStack.h`。演算子で階層を組む:

```
AutoStack = (T1 + T2) / (T3 << ConstraintT3 + T4) << Bounds
```

| 演算子 | 意味 |
|---|---|
| `+` | **同一優先度でのタスク和**。`[A1;A2]`, `[b1;b2]`, 重みは block-diag `diag(W1,W2)` の `Aggregated` を作る |
| `/` | **異なる優先度でスタック**。左=高優先度。カスケードの各レベルになる |
| `<<` | 制約/境界の付与(タスクまたはスタックへ) |
| `%` | 行選択 → `SubTask`(例: `cartesian % {0,1,2}` で位置のみ) |
| `*` | 重み付け(`W * task`) |

---

## 2. QP ソルバ — iHQP 階層

`include/OpenSoT/solvers/iHQP.h`。**incremental Hierarchical QP**。古典的な
Stack-of-Tasks を、明示的な零空間射影を計算せず**優先度レベルごとに1つの
QP** を解いて実現する。上位レベルの最適性を**下位 QP の等式制約**として課す。

### 2.1 レベルごとのコスト

タスク `‖Ax−b‖²_W` を QP の Hessian/勾配に変換:

```
H = AᵀWA          (task->getATranspose() * task->getWA())
g = −AᵀWb + c
```

### 2.2 優先度制約(階層の核心)

レベル `j` を解いて解 `x_j*` を得たら、下位レベルには

```
A_j · x = A_j · x_j*     (等式)
```

を課す。「上位タスク `j` が達成したものを変えるな」= 上位の零空間で最適化、
を等式制約として実装。これを全 `j < i` について積む。

### 2.3 solve ループ

各レベル `i` で: (1) `H,g` 計算、(2) レベル `i` の制約を `Aggregated` で
集約(グローバル境界を注入)、(3) 上位各 `j<i` の最適性制約を積む、
(4) バックエンドに `initProblem(H,g,A,lA,uA,l,u)` または更新+`solve()`、
(5) 解が答え兼、次レベルの最適性制約の入力。最下位レベルの解が最終 `x`。

- `_epsRegularisation`(既定 `2E2`)で階数落ちの `AᵀWA` を減衰(damped LS)。

### 2.4 バックエンド抽象

`BackEnd` は密 QP ソルバ1個の薄いラッパ。正準形:

```
min_x  ½ xᵀH x + gᵀx   s.t.  lA <= A x <= uA,   l <= x <= u
```

`initProblem / solve / updateTask / updateConstraints / updateBounds` を持ち、
実体は各ソルバの `.so` を実行時ロード。レベルごとに別バックエンド可。
実装: qpOASES(active-set、既定)、OSQP(ADMM 疎)、proxQP(ProxSuite 密)、
eiQuadProg(Goldfarb-Idnani、依存軽)、qpSWIFT(内点疎)、GLPK(LP)。

- **iHQP の代替フロントエンド**(階層を別の数式で): `eHQP`(擬似逆+零空間
  射影、等式のみ)、`nHQP`(零空間基底を明示構築)、`HCOD`(soth ライブラリ、
  1回の分解で不等式も)、`l1HQP`(スラック+ℓ₁ ペナルティの単一 QP)。
  **移植の第一候補は iHQP**: 最も汎用(各レベルに不等式)、黒箱密 QP だけで
  済み、「上位出力を等式で凍結」トリックは零空間射影より実装が簡単。

---

## 3. 速度レベル(IK)タスク

`x = dq ∈ ℝ^Nv` を解く。規約: **`A` は Jacobian、`b = フィードフォワード
速度 + λ·誤差`**。

| タスク | `A` | `b` | 使うモデル量 |
|---|---|---|---|
| **Cartesian** | `J`(6×Nv、`getJacobian` / relative) | `_desiredTwist + λ·pose_error`(位置差+姿勢誤差ベクトル) | Jacobian, getPose |
| **CoM** | 重心 Jacobian(3×Nv) | `_desiredVel + λ·(com_ref − com)` | getCOMJacobian, getCOM |
| **Postural** | 単位行列(またはマスク) | `_v_desired + λ·difference(q_des, q)`(多様体差分) | getJointPosition, difference |
| **AngularMomentum** | 重心運動量行列(CMM)の下3行 | `_desiredAngularMomentum` | computeCentroidalMomentumMatrix |
| **LinearMomentum** | CMM 上3行 | `_desiredLinearMomentum` | 同上 |
| **Gaze** | カメラ frame の Cartesian 派生 | pose feedback | Jacobian, getPose |
| **Manipulability** | 単位行列 | `λ·∇(可操作性 √det(JWJᵀ))`(数値微分) | getJointPosition, sum |
| **MinimumEffort** | 単位行列 | `−λ·∇(重力トルク努力)` | computeGravityCompensation |

その他: `CartesianAdmittance`(力→速度参照)、`Contact`(接触リンク速度0)、
`PureRolling`(車輪)、`JointAdmittance`。

**注意**: `_update()` は毎 tick フィードフォワード速度を 0 にリセットする
(安全のため)。軌道追従は毎 tick `setReference` が必須。

---

## 4. 加速度レベル(ID)タスク ← **動力学の本丸**

変数は `qddot`(と接触レンチ `F`)。逆動力学 WBC の層。

### 4.0 変数スタックと Torque(`utils/InverseDynamics.h`, `variables/Torque.h`)

`InverseDynamics` が大域変数 `x` を定義:

```
x = [ qddot(6+n) ; F_c0 ; F_c1 ; ... ]
```

接触は `POINT_CONTACT`(3自由度、力のみ、`[I₃;0]` で6次レンチに埋込)か
`SURFACE_CONTACT`(6自由度、力+モーメント)。

`variables::Torque` は**トルクを `(qddot, F)` のアフィン**として表現:

```
τ(x) = S ( B(q) q̈ + h(q,q̇) − Σᵢ Jcᵢᵀ Fcᵢ )
```

`S = [0_{n×6} | I_n]` は駆動関節選択、`B` は全慣性行列、`h` は非線形項。
これで**トルク限界やトルク正則化を、τ を明示変数にせず `(qddot, F)` 空間で
直接書ける**。解の後は `computedTorque(x)` で τ を復元し、浮遊ベースの
最初の6行が ≈ 0 であることを検証(未駆動性チェック)。

### 4.1 DynamicFeasibility — Newton–Euler / 運動方程式(**中核**)

`tasks/acceleration/DynamicFeasibility.h`。浮遊ベース未駆動性
`M q̈ + h = Jcᵀ Wc` の**上6行**を等式として課す。全運動方程式

```
M(q) q̈ + h(q,q̇) = Sᵀ τ + Σᵢ Jcᵢ(q)ᵀ Fcᵢ
```

の上6行(浮遊ベース)では `Sᵀτ` が消えるので:

```
B_u q̈ + h_u − Σᵢ (Jcᵢ)_base ᵀ Fcᵢ = 0     (6式)
```

`(Jcᵢ)_base ᵀ = Jc.block<6,6>(0,0).transpose()`。行列形:

```
A = [ B_u | −(Jc0)_baseᵀ | −(Jc1)_baseᵀ | ... ]   (6 × dim(x))
b = −h_u
```

- **変数は qddot と全接触 F の両方**。これが両者を結ぶ結合制約。
- `enableContact/disableContact` で接触の列ブロックを落とす(遊脚)。
- 通常は**ハード等式制約**として積む。

### 4.2 その他の加速度タスク

| タスク | A | b | 意味 |
|---|---|---|---|
| **Cartesian** | `J` | `ẍ_ref + λ₂Kd(ẋ_ref−ẋ) + λKp(x_ref−x) − J̇q̇` | 作用空間 PD+FF 加速度指令 |
| **CoM** | `J_com` | `r̈_ref + λ₂Kd(ṙ_ref−ṙ) + λKp(r_ref−r) − J̇_com q̇` | 重心 PD |
| **Postural** | `I`(qddot ブロック) | `q̈_ref + λ₂Kd(q̇_ref−q̇) + λKp Δq` | 関節空間 PD(駆動関節のみ) |
| **Contact** | `K·Ad(wRcl)·J` | `−K·Ad·J̇q̇` | 剛体接触(接触点加速度 0、K で自由度選択) |
| **AngularMomentum** | CMM 下3行 | `L̇_d + λK(L_d−L) − (Ȧq̇)_ω` | 重心角運動量レート |
| **MinJointVel** | `I` | `q̈ + q̇/dT`(λ=0, λ₂=1/dT の Postural) | 次ステップ関節速度最小化(正則化) |

- **GainType**(`GainType.h`): `Acceleration` は Kp,Kd を作用空間誤差への
  行列 λ₁,λ₂ として使う。`Force` は Cartesian 剛性/減衰として使い、逆
  Cartesian 慣性 `M_x = J B⁻¹ Jᵀ` を通す(仮想力→加速度 `ẍ = M_x⁻¹ F`)。
- 既定は臨界減衰 `λ₂ = 2√λ`。Cartesian/CoM/Postural は毎 tick FF をリセット。

---

## 5. 力レベルタスク

変数は**接触レンチ `F` のみ**(qddot なし)。力分配 / 重心 QP。

- **force::CoM**: 重心動力学 `m r̈ = Σf + mg`, `L̇ = Σ(pᵢ×fᵢ + τᵢ)` を力に
  逆算。`A = G`(接触ごとに `[I,0; P_i,I]` を並べる grasp map、`P_i=skew(p_i)`)、
  `b = [m(r̈_ref − g); L̇_ref]`。古典的な重心力分配。
- **force::FloatingBase**: DynamicFeasibility の力版。
  `Σ (Jcᵢ)_baseᵀ Fcᵢ = τ_fb`、`τ_fb = B_u q̈ + h_u` を外部供給。qddot 非変数。
- **force::Cartesian**: インピーダンスで求めた仮想レンチに追従。
  `A = I₆`, `b = Kp(x_d−x) + Kd(ẋ_d−ẋ) + F_d`。力制御。
- **force::Force**(Wrench/Wrenches): レンチ正則化 / 参照追従。QP を良条件化。

---

## 6. 制約(動力学で重要な順)

`GenericConstraint` が `lb <= y <= ub`(`y = Mx+q`)を QP 行に変換:
`Aineq = M, bUpper = ub−q, bLower = lb−q`。

### 6.1 FrictionCone — 摩擦錐(4面ピラミッド線形化)★

`constraints/force/FrictionCone.h`。接触力が摩擦錐内(滑らない+押すのみ)。
**内接4面ピラミッド**(`mu/√2` でスケール)。接触 frame での力 `f` に対し
**5×3** 行列:

```
        [  1   0  -mu/√2 ]   fx  <= (mu/√2) fz
        [ -1   0  -mu/√2 ]  -fx  <= (mu/√2) fz
_Ci  =  [  0   1  -mu/√2 ]   fy  <= (mu/√2) fz
        [  0  -1  -mu/√2 ]  -fy  <= (mu/√2) fz
        [  0   0   -1    ]  -fz  <= 0   (単方向性 fz >= 0)
```

大域は world frame なので `_Ci = _Ci * wRl.transpose()` で回転。最終
`_Ci · (wRlᵀ f) <= 0`、上界 0・下界 −∞。**四足では足/地形法線が変わるので
毎 tick `setContactRotationMatrix` が必要**。Go2 の足に直接使える点接触形。

### 6.2 TorqueLimits(加速度レベル)— アクチュエータトルク限界 ★

`constraints/acceleration/TorqueLimits.h`。**τ を変数にせず**運動方程式
`τ = M q̈ + h − Σ Jcᵢᵀ Fcᵢ` を代入して τ を束縛:

```
−τ_lim − h  <=  [ M | −Jc0ᵀ | −Jc1ᵀ | ... ]·[q̈; F0; F1; ...]  <=  τ_lim − h
```

**浮遊ベースの6行は τ_lim=0** を与える → これで (a) アクチュエータ飽和、
(b) 浮遊ベース未駆動性等式、(c) qddot と F の全 EoM 結合、を1ブロックで実現。

### 6.3 関節位置限界(加速度レベル、3方式)

いずれも `lb(q,q̇) <= q̈ <= ub(q,q̇)` に帰着。多様体 `difference()` で
四元数ベースを正しく扱う。

- **JointLimits**(Wolff-Buss): 定加速停止モデルの2次式。位置のみの2次障壁。
- **JointLimitsViability**(Del Prete): 位置+速度+加速の3区間を交差。
  **離散時間の生存可能性(viability)保証が最強・パラメタなし**。高レート ID の推奨。
- **JointLimitsECBF**(指数制御障壁関数): ゲイン α1,α2,α3 で調整可能。
  `q̈ >= −(α1+α2)q̇ + α1α2(q_min − Δq)` 等。滑らかで**チューナブル**。

### 6.4 その他の力/接触制約

- **CoP**: 平足接触の圧力中心を足矩形内に(転倒防止)。点足四足では通常不要。
- **NormalTorque**: 法線(ヨー)トルク限界。{FrictionCone, CoP, NormalTorque}
  で矩形足の**接触レンチ錐(CWC)**完成。点足では省略。
- **WrenchLimits**: レンチの箱制約 +**接触解放**(`releaseContact` で両境界を
  0 に固定=遊脚のレンチを 0 に固定)。四足の足力キャップ兼スイング門。
- **StaticConstraint**: 準静的 `τ + Jcᵀ F = g`(等式)。力分配 QP 用。

### 6.5 速度レベル制約(簡潔)

`JointLimits`(1次位置境界)、`VelocityLimits`(箱)、`CartesianVelocity`
(`−v_max·dT <= J q̇ <= v_max·dT`)、**ConvexHull**(重心を支持多角形内=
静的安定、`A_CH·J_com·q̇ <= b_CH`)、`CollisionAvoidance`(証人点距離 Jacobian)。

---

## 7. articara 既存資産との対応と差分

articara は既に **quadruped-gait::wbc**(`~/work/dp/quadruped-gait/
quadruped-gait/src/wbc/`)に3優先度 HoQp WBC を持つ。Kim 2014 / legged_control
準拠。

**決定変数**: `x = [q̈ (nv); f_GRF (3·nc); τ (na)]`
— **τ を明示変数に含む**(OpenSoT は τ を代入消去する点が違う)。

既存タスク(`wbc/tasks/`)と OpenSoT の対応:

| articara(既存) | 優先度 | OpenSoT 対応 |
|---|---|---|
| `floating_base_eom` | 0(ハード等式) | acceleration::DynamicFeasibility |
| `torque_limits` | 0(ハード不等式) | constraints::acceleration::TorqueLimits |
| `friction_cone` | 0(等式+不等式) | constraints::force::FrictionCone + WrenchLimits(遊脚) |
| `no_contact_motion` | — | acceleration::Contact(接触点加速度0) |
| `base_accel` | 1(ソフト) | acceleration::CoM / Cartesian |
| `swing_leg` | 1(ソフト) | acceleration::Postural(脚関節) |
| `contact_force` | 2(ソフト) | force::Force(レンチ正則化) |
| `tau_gravity` | 3(ソフト) | ≈ MinimumEffort / τ 正則化 |
| HoQp(`ho_qp.rs`) | — | solvers::iHQP(零空間カスケード) |

**articara に無く、OpenSoT にあるもの(=機能追加の候補)**:

1. **汎用 Cartesian 加速度/力タスク**(任意リンクの位置・姿勢・力制御)。
   現状は base_accel と swing_leg に固定。任意 EE のインピーダンス/力制御へ。
2. **CoP / NormalTorque / 接触レンチ錐(CWC)**: 平足・面接触ロボット対応
   (lkmotor/robstride のヒューマノイド系や腕への拡張時)。
3. **CBF/viability 関節限界**(加速度レベル): 現状の torque_limits に加え、
   位置・速度限界を加速度 QP で厳密に。
4. **力分配専用 QP**(force::CoM、qddot なし): 立位・静的バランスの軽量版。
5. **AffineHelper 的な変数抽象**: 現状 `WbcDims` で手動オフセット
   (`q_offset/f_offset/tau_offset`)。接触数や変数構成を動的に変えにくい。
   汎用動力学機能には「変数を宣言してタスクを記号的に書く」層が効く。
6. **AutoStack 的タスク DSL**: タスクの組み替えを実行時/設定で。

**articara が既に持つ利点**: HoQp は legged_control 準拠で歩行実績あり
(gait_walk_stability で検証済み)。misarta の `solve_qp`(ActiveSet/Clarabel、
proximal warm-start)がバックエンド。**ゼロから作る必要はなく、既存 WBC を
「タスク部品化 + 変数抽象」方向へ一般化するのが筋**。

---

## 8. misarta が提供すべき量(チェックリスト)

OpenSoT は全量を `XBot::ModelInterface` から取る。articara の
misarta(`~/work/dp/misarta`)の対応状況:

| OpenSoT が要求 | misarta の対応 | 状態 |
|---|---|---|
| `getJacobian(link)` 6×Nv | `jacobian::compute_joint_jacobian` | ✅ |
| `getRelativeJacobian` | `compute_relative_jacobian` | ✅ |
| **`getJdotTimesV(link)` = J̇·v** | `compute_joint_jacobian_time_derivative`(J̇) | 🔶 **J̇ はあるが J̇·v の直接 API は要確認/追加** |
| `getPose` / `getVelocityTwist` | `fk`(FK) / frames | ✅ |
| `getCOM` / `getCOMJacobian` / `getCOMVelocity` | `centroidal::compute_com{,_jacobian,_velocity}` | ✅ |
| **`getCOMJdotTimesV()`** | — | 🔶 **要追加(CoM 加速度タスク用)** |
| `computeInertiaMatrix()` = M | `crba::crba` | ✅ |
| `computeInertiaInverse` = M⁻¹ | (M から) | ✅(逆行列) |
| `computeNonlinearTerm()` = h=Cv+g | `rnea::nonlinear_effects` | ✅ |
| `computeGravityCompensation()` = g | `rnea::compute_gravity` | ✅ |
| `getCentroidalMomentumMatrix()` 6×Nv | `centroidal::compute_centroidal_momentum_matrix` | ✅ |
| 重心運動量レートバイアス Ȧq̇ | `compute_centroidal_momentum_matrix_time_derivative`, `compute_momentum_rate` | ✅ |
| `sum(q,dv)` / `difference(q1,q2)`(多様体) | `manifold`(要確認) | 🔶 **浮遊ベース retraction を要確認** |
| `getNv()` vs `getNq()` | model が保持 | ✅ |
| QP バックエンド | `qp::solve_qp`(ActiveSet/Clarabel、eq/ineq、prox warm-start) | ✅ |

**結論**: 主要プリミティブ(M, h, g, Jacobian, 重心運動量行列+時間微分、
CoM Jacobian、QP ソルバ)は**ほぼ全て揃っている**。埋めるべき隙間は主に
**`J̇·v`(link と CoM の両方)の直接 API** と **浮遊ベースの多様体
`sum`/`difference` retraction** の確認/整備。constrained.rs は既に
`[M Jcᵀ; Jc 0]` の KKT を解く接触動力学を持つので、動力学の素地は厚い。

---

## 9. 移植の設計提案

OpenSoT を丸ごと移すのではなく、**既存 quadruped-gait::wbc を「汎用動力学
タスクライブラリ」へ一般化する**方針を推奨。

### 9.1 採るべきアイデア(優先順)

1. **AffineHelper 相当の変数層**(Rust では小さな `struct Affine { m:
   DMatrix, q: DVector }` + `Mul`/`Add`)。`OptvarHelper` 相当で
   `x = [qddot; F...; (τ)]` を宣言し、各タスクを局所変数で記述。接触の
   enable/disable と変数構成の動的変更が自然になる。**最初にやる価値が最大**。
2. **Task 残差規約の統一**: 全タスクが `e = M x + q` を作り、コスト
   `‖Mx+q‖²_W`、制約 `Mx=−q`。既存タスクをこの形に揃える。
3. **タスク部品の追加**: 汎用 `acceleration::Cartesian`(任意リンク PD+FF、
   GainType で力/加速度切替)、`constraints::force::FrictionCone`(点接触
   5行ピラミッド、毎 tick 回転更新)、CBF/viability 関節限界。
4. **iHQP の「上位出力を等式で凍結」**方式は既存 HoQp と同思想。misarta
   `solve_qp` をバックエンドに、レベルごとの等式積み上げで階層化。

### 9.2 やらないこと / 注意

- **1優先度の重み付き単一 QP から始めるのが簡単**(OpenSoT も両対応)。
  厳密階層は後付け可能。既存 HoQp があるので階層は再利用できる。
- **多様体 retraction を徹底**: 浮遊ベース/四元数では `q += dq` でなく
  `q = sum(q, dq)`、誤差は `difference`。misarta 側の整備が前提。
- **点足四足なら CoP/NormalTorque は不要**、FrictionCone のみで足りる。
  面接触(ヒューマノイド足・把持)を足すとき CWC 一式を導入。
- **参照は毎 tick セット**(OpenSoT は FF を毎回リセットする安全設計)。

### 9.3 最小の第一歩(提案)

「任意リンクの**加速度レベル Cartesian 力制御タスク**」を1つ追加する
のが、動力学機能拡張の足がかりとして具体的で小さい:
- misarta に `jacobian_dot_times_v(link)`(と CoM 版)を追加。
- quadruped-gait::wbc に `tasks::cartesian_accel`(GainType 切替つき)を追加。
- 既存 HoQp の priority-1 にソフトで挿せることを exp_sweep 系ハーネスで検証。

これで「WBC = Go2 歩行専用」から「WBC = 汎用全身制御」への第一歩になり、
lkmotor-rs / robstride-rs のアーム系にも展開できる。

---

## 付録: 主要ファイル索引(`ref/OpenSoT/` 相対)

- コア: `include/OpenSoT/{Task,Constraint,Solver,SubTask,SubConstraint}.h`
- Affine: `include/OpenSoT/utils/{Affine,AffineUtils,AutoStack}.h`,
  `variables/Torque.h`, `utils/InverseDynamics.h`
- ソルバ: `include/OpenSoT/solvers/{iHQP,eHQP,nHQP,HCOD,l1HQP,BackEnd,BackEndFactory}.h`
- 速度タスク: `include/OpenSoT/tasks/velocity/*.h`
- 加速度タスク: `include/OpenSoT/tasks/acceleration/{DynamicFeasibility,Cartesian,CoM,Postural,Contact,AngularMomentum,MinJointVel,GainType}.h`
- 力タスク: `include/OpenSoT/tasks/force/{CoM,FloatingBase,Cartesian,Force}.h`
- 制約: `include/OpenSoT/constraints/force/{FrictionCone,CoP,NormalTorque,StaticConstraint,WrenchLimits}.h`,
  `constraints/acceleration/{TorqueLimits,JointLimits,JointLimitsECBF,JointLimitsViability,VelocityLimits}.h`,
  `constraints/velocity/{JointLimits,VelocityLimits,CartesianVelocity,ConvexHull,CollisionAvoidance}.h`
- 例: `examples/cpp/{panda_ik,static_walk}.cpp`, `tests/tasks/{acceleration,force}/`
