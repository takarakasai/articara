# WBC 3者比較: OpenSoT / GID / Articara スタック (misa-wbc + misarta)

作成: 2026-07-10。ソース: `ref/OpenSoT/`(まとめ `ref/opensot.md`)、
`ref/GID/`(コード精読、README/test は空)、misa-wbc@3e6247c +
misarta@df46df9 + quadruped-gait@15b8be6。

---

## 0. TL;DR

3者は同じ問題(多タスク優先度付き全身制御)を解くが、**定式化の哲学が3様**:

| | OpenSoT | GID | misa-wbc (+misarta) |
|---|---|---|---|
| 一言で | 明示行列 + null-space iHQP | **行列フリー** 単位力伝播 + 力予算カスケード | 明示行列 + null-space HoQP (Kim 2014) |
| 決定変数 | `[q̈; f]`(τ は EoM で消去) | **`[τ; f]`**(力ドメイン、q̈ は暗黙) | `[q̈; f; τ]`(全部明示) |
| 動力学の入り方 | モデル IF から M/h/J/J̇v を取得し制約行列化 | **M も J も組まない** — ABI 単位力伝播で作用空間逆慣性 `I = J M⁻¹ [Sᵀ Jcᵀ]` を列ごとに直接測る | 消費者(articara)が misarta で M/h/J/J̇v を計算して渡す |
| 階層の実現 | 上位の最適値を等式制約に凍結して QP カスケード(厳密) | **貪欲な力予算カスケード**: 上位レベルの τ を確定・消費し、下位は残り `τmax − τ_used` で解く(準厳密) | 上位等式の null space 内で下位を解く(厳密、テストで保証) |
| QP backend | qpOASES/OSQP/proxQP/eiQuadProg... 差替可 | qpOASES 固定(v0.5 同梱) | Clarabel / ActiveSet 切替(`SolveConfig`) |
| 不等式 | 完備(関節限界 CBF/viability、摩擦錐、CoP/CWC、τ限界) | 摩擦ピラミッド + **ZMP/CoP 箱 + ねじり摩擦** + 単側接触 + τ限界(関節位置/速度限界は無し) | 摩擦ピラミッド + box_bound(τ)。CoP/関節限界は未実装 |
| タスク語彙 | 最大(Cartesian/CoM/Postural/Force/Momentum/Gaze...) | Absolute/Relative/Joint/**AssocJoint**/**Momentum** × 参照ラダー(Accel/Veloc/Value/**Impedance**) | EoM/cartesian_accel/contact/friction/box/track(最小) |
| RBD エンジン | XBotInterface(Pinocchio) | Proprietary(Featherstone 空間代数、Eigen 非依存) | misarta(Rust、CRBA/RNEA/ABA/centroidal) |
| テスト | 充実(randomized pinv-vs-QP 等) | **無し**(test/ 空) | randomized property + MuJoCo 歩行回帰(検証面は3者最強) |
| 言語/状態 | C++、IIT 公開 OSS | C++, qpOASES 0.5 時代 | Rust、自作、公開済み |

**要点**: misa-wbc は「OpenSoT 型の明示行列アプローチ」の系譜で、GID は
まったく別解(行列フリー・力ドメイン)。GID から学べるのは(1)接触の
豊かさ(CoP/ZMP 箱・ねじり摩擦・Multiplier 結合)、(2)Momentum
(セントロイダル)タスク、(3)参照ラダー(特に Impedance)、
(4)Relative/AssocJoint タスク、(5)行列フリー効率の発想。

---

## 1. GID とは何か(調査結果の要約)

**GID = Generalized Inverse Dynamics**。
優先度付き全身逆動力学: 各 tick で **未知の力 x = [作動関節トルク τ;
支持(接触)力 f]** を、作用空間加速度が目標に追従するよう QP で解く。

### 1.1 行列フリーの核心

M も J も一切組まない。各 OperationSpace(タスクの 1 スカラー行)が

- `ExertUnitForceDynamically()` — 空間力方向 s に単位力を注入し ABI 伝播
- `GetAccelWithVxTxGxFo()` = sᵀ·a — 生じた加速度を読む → **逆慣性 I の1要素**
- `GetAccelWithVoToGoFx()` = sᵀ·α — バイアス加速度(J̇v + 重力 + 既知力)

を提供し、Resolver が未知力ごとに単位力→全タスク行の加速度を測って
`I`(motionDim × unknownDim)を列ごとに構築する。数学的には
`I = J M⁻¹ [Sᵀ Jcᵀ]` を Featherstone ABI で暗黙計算している。
EoM は制約行として存在しない — **力学は写像 I そのもの**に入っている。

### 1.2 各レベルの QP

```
min_x  ½ xᵀ(Iᵀ W I + R)x + (W b)ᵀ I x
s.t.   l ≤ x ≤ u,   l_c ≤ C x ≤ u_c
```

- `b = −Target + biasAccel` → `‖I·x − (Target − bias)‖²_W` の加速度追従
- `R` = 微小トルク正則化(最小努力解の選択)
- 境界 `u = τmax − τ_used`(上位レベルの消費分を差引き = 階層予算)
- `C` = Multiplier 結合の摩擦/ZMP 行

qpOASES で解き、失敗時は前 tick のトルクへフォールバック
(`UseLastForce`)。

### 1.3 階層 = 力予算カスケード

`BeginMotion → (タスク登録 → NextMotion)× → EndMotion`。各
`NextMotion()` が現レベルの QP を解き、トルクをモデルに**確定加算**。
次レベルは (a) バイアス加速度に上位の力の効果が入り、(b) トルク境界が
残り予算に縮む。null-space 射影は無い — 上位タスクの最適性が下位で
厳密保存される保証はない(上位もレベル内では重み付きソフト)。

### 1.4 タスク語彙(OperationUnit)

| Unit | 内容 | misarta/misa-wbc 対応 |
|---|---|---|
| Absolute (6D/3D/1D) | 世界座標系 Cartesian | cartesian_acceleration ≈ 対応済 |
| Relative | 2リンク間相対運動 | misarta compute_relative_jacobian 有り、タスク未実装 |
| Joint (Full/1D) | 関節空間(posture) | track で表現可 |
| **AssocJoint** | 2関節の線形結合座標(ギア/差動/mimic) | 無し(Affine で表現可能) |
| **Momentum** | セントロイダル運動量(CMM 行が基底) | misarta に CMM 有り、タスク未実装 |

全 Unit 共通の**参照ラダー**: `SetAccel`(FF加速度)/ `SetVeloc`(P)/
`SetValue`(カスケード PD、姿勢は quaternion 軸角)/ **`SetImpedance`**
(仮想バネダンパ `k·Δx − d·ẋ`)。

### 1.5 接触モデル(GID の強み)

`Set*ContactSupport` 1呼びで 6D パッチ接触一式:

```cpp
fx, fy:  |f| ≤ μ·fz      (Multiplier 結合 → 摩擦ピラミッド)
fz:      0.2 ≤ fz ≤ fmax  (単側 + 上限)
mx, my:  |m| ≤ ZMP箱·fz   (CoP を支持多角形内に = 転倒防止)
mz:      |mz| ≤ μ_ang·fz  (ねじり摩擦)
```

`m_Multiplier`(他 DOF の力への線形結合ポインタ)という行列を見せない
API で摩擦錐/CoP を表現しているのが設計的に面白い。

### 1.6 弱点

- 関節位置/速度限界の制約が無い(τ 限界のみ)
- 階層の厳密性保証なし(貪欲カスケード)
- テスト皆無、qpOASES 0.5 固定、SDRDtk 密結合(可搬性低)

---

## 2. 3者の設計軸比較(詳細)

### 2.1 決定変数と動力学整合

- **OpenSoT**: `x=[q̈; f]`。τ = EoM の作動行から事後代入。τ 限界は
  代入式で不等式化(TorqueLimits)。変数最少、ただし τ を直接コスト/
  制約にしたい時に間接的。
- **GID**: `x=[τ; f]`。q̈ は I·x + bias として暗黙。真の「逆動力学」の
  出力空間で解くため、トルク正則化・トルク予算階層が自然。ただし
  q̈ 側の制約(関節加速度限界等)は表現しにくい。
- **misa-wbc**: `x=[q̈; f; τ]` 全明示 + EoM を等式タスクで結ぶ。変数は
  最大だが、**どの量にも直接タスク/制約を書ける**。しかも VarLayout/
  Affine 層があるので、実は OpenSoT 型(τ 消去)も GID 型(q̈ 消去 =
  `q̈ = M⁻¹(Sᵀτ + Jcᵀf − h)` を Affine で構成)も**同じコアで表現可能**。
  misarta には `compute_minv_times_vec`(ABA ベース M⁻¹·v)が既にあり、
  M⁻¹ 明示化も列単位で可能。

### 2.2 階層セマンティクス

- OpenSoT iHQP / misa-wbc HoQp: **辞書式最適**(上位の最適値を厳密保存)。
  misa-wbc はこれをランダム化テストで保証済(upper preserved < 1e-5)。
- GID: 貪欲カスケード。上位が使ったトルクを下位が使えないだけで、
  「下位が上位の達成度を壊さない」数学的保証は無い(バイアス持ち越しで
  実用上は近い挙動)。実装は単純・高速で、レベル毎の QP が小さい。
- → misa-wbc の `HqpStrategy` に将来 `ForceBudgetCascade` を足せば
  GID セマンティクスの A/B が可能(3つ目の戦略候補)。

### 2.3 モデル境界

- OpenSoT: XBotInterface 抽象(実質 Pinocchio)。
- GID: SDRDtk 密結合(行列フリーは Featherstone 実装への深い依存の裏返し)。
- misa-wbc: **行列を渡すだけ**の最疎結合。RBD エンジン非依存はテスト容易性
  (合成行列でテスト可)と多消費者(アーム/脚)対応の源泉。
  トレードオフ: GID 的な O(n) 行列フリー効率は取れない(M/J を毎 tick
  明示構築)。nv≈18 の四足では問題にならないが、ヒューマノイド nv≈40+
  で効いてくる可能性。

### 2.4 検証

- misa-wbc: ランダム化 property test(pinv 照合・階層契約)+ MuJoCo
  歩行回帰(articara)。null-space tol の実バグをテストが捕捉した実績。
- OpenSoT: testPinvVSQP 等、テスト文化あり。
- GID: テスト無し。**逆に言えば GID の資産(接触モデル、Momentum 基底の
  数式)を misa-wbc に移植する際は、うちのテスト網で初めて検証される**。

---

## 3. 発展方向の候補(GID/OpenSoT から misa-wbc へ)

優先度順の提案。効果 = 「Go2 歩行専用 → 汎用全身制御」への寄与。

### D1. セントロイダル Momentum タスク ★推奨・即効
- GID の Momentum Unit / OpenSoT の acceleration::CoM+AngularMomentum 相当。
- **misarta に CMM(compute_centroidal_momentum_matrix)が既にある**。
  必要なのは (a) misarta: `compute_cmm_dot_times_v`(J̇v と同じ中心差分、
  実装 30 分クラス)、(b) misa-wbc: `centroidal_momentum` タスク
  (A=CMM, ref=ḣ_des)。
- 価値: 四足の耐外乱(MPC の C1-2 と相補)、ヒューマノイドバランスの
  中核プリミティブ。GID が「バランス一次手段」に据えていた機能。

### D2. 6D パッチ接触(CoP 箱 + ねじり摩擦) ★推奨
- GID の SetContactSupport 一式を misa-wbc `tasks::patch_contact` に:
  摩擦ピラミッド(既存)+ `|mx|≤Ly·fz, |my|≤Lx·fz, |mz|≤μ_ang·fz`。
- 全て力変数の線形不等式なので Task::le で 10 行級。単体テストも
  合成レンチで完結。
- 価値: ヒューマノイド足(面接触)への必須装備。OpenSoT の CoP/CWC の
  簡易版に相当し、GID の実装が「答えの数式」をくれる。

### D3. 参照ラダー(refgen モジュール)
- GID の Accel/Veloc/Value/Impedance を misa-wbc の薄いヘルパに:
  `refgen::pd(x_ref, x, ẋ, kp, kd)`, `refgen::impedance(k, d, ...)` →
  DVector を返し cartesian_acceleration に渡す。コアは gain-agnostic の
  まま、使い勝手だけ GID 流に。
- 価値: articara/アーム系での書き味。小規模(半日)。

### D4. Relative / AssocJoint タスク
- misarta compute_relative_jacobian 既存 → `relative_acceleration` タスクは
  ほぼ無料。AssocJoint(ギア結合)は Affine の線形結合で表現できるので
  ヘルパ1個。
- 価値: 双腕相対作業・閉リンク/差動機構。lkmotor/robstride アーム系で効く。

### D5. ForceBudgetCascade strategy(GID セマンティクス)
- `HqpStrategy` 3つ目。レベル毎に x=[τ;f] の小 QP + トルク予算縮小。
- 価値: 高速近似の A/B 候補(WeightedQp と並ぶ)。研究的興味 > 実用。

### D6. 関節限界 CBF(OpenSoT 側の宿題)✅ 実装完了(22367e1、2026-07-12)
- GID にも無い機能で、OpenSoT だけが持つ(viability/CBF)。加速度レベル
  の位置/速度限界障壁。実機安全に直結。
- `tasks::joint_limit_cbf` + `JointLimitCbf`。OpenSoT の
  `JointLimitsECBF`(Khazoom et al. の指数制御障壁関数)を
  `ref/OpenSoT/src/constraints/acceleration/JointLimitsECBF.cpp` から
  数式移植: 2次障壁(位置、`ḧ+(α1+α2)ḣ+α1α2h≥0`)2本 × 1次障壁
  (速度)× ハード a_max 箱、交差時は swap で HoQp スラックに委ねる。
- 単体テスト4本(遠方で緩い/上限近傍で強制減速/速度限界単独作用/
  既に限界超過時の非パニック)+ **閉ループ統合テスト**
  (`tests/joint_limit_cbf_stack.rs`): 1DOF 二重積分器に「常に限界へ
  全開加速を要求する敵対的タスク」を優先度1、CBF を優先度0に置いて
  2000 tick シミュレート — 位置・速度とも実際に限界を超えないことを
  実証(位置は限界近傍に張り付き、過保守でないことも確認)。

### D7. 行列フリー化(長期・保留)
- misarta の ABA 資産(compute_minv_times_vec)で作用空間逆慣性を列毎に
  組む GID 型経路は原理的に可能。nv が大きくなるまで保留が妥当。

**推奨パス: D1 → D2(+D3 を随時)**。D1 は misarta の CMM 資産で最短、
D2 は GID の数式移植で確実、両方とも misa-wbc のテスト様式(合成行列
property test + articara 回帰)にそのまま乗る。

---

## 4. API 視点の比較(公開クラス/Crate・関数)

3者は API 設計の様式がそのまま3流派になっている:
**OpenSoT = オブジェクト指向 + 演算子 DSL**、**GID = 単一ファサード +
イミディエイトモード**、**misa-wbc = 純関数 + 値型**。

### 4.1 ユーザが触る語彙(概念数)

| | OpenSoT | GID | misa-wbc |
|---|---|---|---|
| エントリポイント | Task/Constraint 派生クラス群(数十)+ AutoStack + iHQP + AffineHelper/OptvarHelper | **GIDAPI 1クラス**(Set* 約79メソッド) | 公開型 ~8 + tasks::* 自由関数 7 |
| 数学型 | Eigen + KDL(Frame/Twist) | SDRVector3/SDRMatrix3/SDRReal(内製) | nalgebra のみ |
| モデル型 | XBot::ModelInterface(必須引数) | SDRObject/SDRLink(必須引数) | **無し**(行列を渡す) |
| メモリ管理 | shared_ptr だらけ(TaskPtr/Ptr typedef 群) | 参照 + 内部プール | 値型 + 借用 |

### 4.2 タスクの作り方と状態管理

**OpenSoT — ステートフルなタスクオブジェクト**。タスクはモデルを掴み、
`update()` で自分の A/b を robot 状態から再計算する。参照・ゲインは
setter で注入:

```cpp
auto cart = std::make_shared<acceleration::Cartesian>("ee", model, "hand", "world", qddot);
cart->setReference(pose_ref);  cart->setLambda(l1, l2);  cart->setKp(Kp);
// 毎tick: model.update() → task->update() → solver.solve(x)
```

**GID — イミディエイトモード(毎 tick 全宣言)**。タスクオブジェクトは
ユーザに見えない。ファサードの Set* を Begin/End 括弧内で呼ぶだけ。
優先度 = `NextMotion()` の呼び順、ゲイン = 引数:

```cpp
gid.Reset();
gid.BeginSupport();
  gid.Set3DAbsoluteContactSupport(foot, R, p, mu, fmax);   // 接触一式1呼び
gid.EndSupport();
gid.BeginMotion();
  gid.Set3DMomentumValue(com_tgt, T, fmax);                // level 1
gid.NextMotion();
  gid.Set6DAbsoluteImpedance(hand, p, x_tgt, R_tgt, imp, ang_imp, fmax, mmax); // level 2
gid.EndMotion();
gid.SetResult(&out);   // τ を書き出し
```

メソッド行列: **タスク族(Absolute/Relative/Joint/AssocJoint/Momentum ×
6D/3D/1D)× 参照種(Accel/Veloc/Value/Impedance/+Ang)で約79個**。
型安全だが組合せ爆発型 API。

**misa-wbc — 純関数 + 値**。タスクは自由関数が返す値。状態なし、
毎 tick 組み直し(GID と同じイミディエイト哲学を関数型で):

```rust
let p0 = tasks::equation_of_motion(&q, &f, &tau, &mass, &h, &jc)
    + tasks::zero_contact_acceleration(&q, &jc, &dj_v)
    + tasks::friction_pyramid(&f, 0.7)
    + tasks::box_bound(&tau, &tau_max);
let p1 = tasks::cartesian_acceleration(&q, &j_ee, &dj_v_ee, &a_ref)
    + tasks::regularize(&f, &f_nominal).weight(1e-3);
let sol = solve(&[p0, p1], &cfg)?;   // 優先度 = スライス順
```

### 4.3 優先度・重みの表現

| | OpenSoT | GID | misa-wbc |
|---|---|---|---|
| 階層 | **DSL 演算子** `t1 / t2`(AutoStack) | `NextMotion()` 呼び順 | `&[level0, level1]` スライス順 |
| 同レベル合成 | `t1 + t2` | 同レベル内で複数 Set* | `t1 + t2`(同じ) |
| 重み | `W * task`, `w * task` | 末尾引数 `i_weight` | `.weight(w)` |
| 制約付加 | `task << constraint` | Support は全レベル共有 | 同レベルに `+`(Task が eq/iq 両持ち) |
| 部分タスク | `task % {rows}` | 1D 版メソッド | 無し(Affine で行選択は可能) |

### 4.4 ソルバの寿命と誤り処理

| | OpenSoT | GID | misa-wbc |
|---|---|---|---|
| ソルバ | iHQP を1回構築、毎 tick `solve(x)`(backend warm 維持) | Resolver 内蔵、毎レベル `qp.init`(コールドスタート) | 毎 tick `solve()` 構築(`solve_warm` で prox anchor のみ) |
| backend 切替 | enum 7種(qpOASES/OSQP/proxQP...)+ boost::any option | 不可(qpOASES 0.5 固定) | enum 2種(Clarabel/ActiveSet)+ SolveConfig |
| 失敗時 | `bool solve()`(false のみ) | bool + **前 tick トルクへ自動フォールバック** | **`Result<Solution, WbcError>` + `SolveStatus::Degraded{level}`**(型付き、3者唯一) |
| 内省 | task_id / getTask / MatLogger2 | 無し | Solution フィールドのみ |

### 4.5 拡張の仕方(新タスクを足すには)

- OpenSoT: `Task` を継承し `_update()` 実装(クラス1個 + ヘッダ登録)。
- GID: OperationUnit/Space のクラス対を書き、GIDAPI にメソッド群を追加
  (ファサード肥大、それが約79メソッドの由来)。
- misa-wbc: **自由関数1個**(`fn my_task(...) -> Task`)。クレート外
  からでも書ける(Task/Affine が公開なので、ユーザクレートに閉じた
  独自タスクが定義可能)。拡張コストは3者最小。

### 4.6 API 観点で misa-wbc に取り込む価値があるもの

1. **GID の「1呼びで接触一式」**(Set3DAbsoluteContactSupport =
   摩擦+単側+CoP箱+ねじりを1関数で)→ D2 `patch_contact()` の API 形。
   misa-wbc は現状 friction_pyramid + box_bound を個別に呼ぶ。
2. **GID の参照ラダー引数**(Value=目標値+時定数、Impedance=k/d を
   引数で)→ D3 refgen。misa-wbc は accel_ref を呼び手が計算する裸 API。
3. **OpenSoT の task_id / ログ**: Degraded { level } を「どのタスクか」
   まで特定できると運用が楽(レベルに名前を付ける Option)。
4. **OpenSoT のソルバ永続化**: iHQP は1回構築で backend warm を保つ。
   misa-wbc の solve() は毎 tick 構築 — `Solver` 構造体(levels 形状を
   固定して再利用、backend warm-start 保持)は将来の実時間最適化候補。
5. **OpenSoT の `/` DSL**: Rust でも `Stack = t1 / t2` は Div 実装で
   可能だが、スライス順で十分明示的なので優先度低。

### 4.7 API 総評

- **OpenSoT**: 機能最大・語彙最大。モデル密結合の setter 文化で、タスクの
  再利用にはモデル IF ごと持ち込む必要。DSL は美しいが shared_ptr 汚染。
- **GID**: 呼び手のコードが最も短い(ファサード+デフォルト引数)。
  引き換えに 79 メソッドの組合せ爆発と、タスクの合成・内省・拡張の不能。
- **misa-wbc**: 概念数最小・拡張コスト最小・型付きエラーで、素朴だが
  一貫。弱点は (a) 参照生成が裸(D3)、(b) 接触セットアップが低レベル
  (D2)、(c) ソルバ毎 tick 構築(実時間で効いたら Solver 永続化)。

### 4.8 Layer 2 設計の確定事項(2026-07-10 議論)

方針: **素の API(Layer 0/1)はシンプルなまま、GID 流の書き味は
ヘルパ層(Layer 2)として上に載せる**。ただし GID の設計をそのまま
持ち込まず、2つの古さを明確に改善する:

1. **Begin/NextMotion/End(暗黙の状態機械)は採らない**。
   Stack はただのデータ(`Vec<Level>`、`Level = Vec<(名前, Task)>`)。
   レベル境界は呼び時刻でなくコード構造で決まり、不正な呼び順は
   型的に表現不能(solve が Stack を消費するだけ)。
2. **運動目的を陽に確認可能にする**。GID は Set* した瞬間にタスクが
   Resolver に消えるが、Stack がデータなら (a) 全タスク命名 +
   `println!("{stack}")` で積んだ目的の一覧、(b) solve 後の
   `sol.report()` でタスク毎の残差・制約マージン(達成度)を陽に確認。
   `Degraded { level }` もタスク名まで特定可能になり、OpenSoT の
   task_id + ログ(§4.6-3)を同時に回収。Stack がデータであることは
   テスト(構成そのものへの assert)や preset 保存にも波及する。

書き味スケッチ:

```rust
let stack = Stack::new(&vars)
    .level("physics", |l| l
        .eom(&q, &f, &tau, &mass, &h, &jc)
        .contact_patch("foot_fl", &f_fl, &patch)
        .torque_limit(&tau, &tau_max))
    .level("tracking", |l| l
        .cartesian("ee", &q, &j_ee, &dj_v, refgen::pd(...))
        .regularize("f_reg", &f, &f_nominal, 1e-3));
println!("{stack}");           // 目的一覧
let sol = stack.solve(&cfg)?;
println!("{}", sol.report());  // 達成度(残差・マージン)
```

trait は合成口(`Into<Task>` 系)のみ。タスク族×参照種を trait 階層で
再現しない(GID 79 メソッド問題の回避)。構成図 artifact:
https://claude.ai/code/artifact/3656cc0d-1021-4b76-9040-46537b48d7a0

### 4.9 等価モード: Formulation × Strategy の直交2軸(D8、2026-07-10 議論)

比較検討のため、GID / OpenSoT と等価な解を misa-wbc で選択可能にする。
変数空間と階層方式は独立な2軸:

| モード | Formulation | Strategy | 等価性 |
|---|---|---|---|
| 現行 | `Explicit` x=[q̈;f;τ] | `NullSpace` | 歩行回帰検証済みの基準 |
| OpenSoT 等価 | `AccelSpace` x=[q̈;f]、τ=M_a·q̈+h_a−(Jcᵀf)_a を Affine 式で | `NullSpace` | 辞書式最適・観測量一致(x 厳密一致は正則化整合で) |
| GID 等価 | `ForceSpace` x=[τ;f]、q̈=M⁻¹(Sᵀτ+Jcᵀf−h) を Affine 式で | `ForceBudgetCascade` (D5) | レベル毎 QP 同一 → 解ベクトル一致が狙える |

**数学的成立性**: どちらの消去も x の Affine 式なので現行 Affine 層で
表現可能。ForceSpace は GID の行列フリー `I = J·M⁻¹·[Sᵀ Jcᵀ]` を
misarta の行列から明示的に組む(同じ QP を別経路で構成、strictly
convex なので解は一意=ソルバ許容差内で一致)。M⁻¹ 適用は misarta の
`compute_minv_times_vec`(ABA)既存。

**実装の核**: `Dynamics` コンテキスト1個 — `(M, h, Jc, formulation)` を
受け「q̈ / τ とは何か」を Affine 式で答える。Layer 1/2 のタスクビルダー
がこれ経由で式を引くため、**同じ Stack 宣言を formulation / strategy
だけ差し替えて再構築 → 解・計算時間を直接比較**できる。

**検証**: 同一物理問題の3定式化クロス一致テスト(q̈/f/τ が tol 内で
一致)+ GID セマンティクステスト(予算縮小・確定加算)。

**ロードマップ改訂**: D8(Formulation 切替)を新設し、**D8+D5 で
比較検討インフラを先行**(Dynamics 軸は Stack 設計の土台)→ D2+D3 →
D1 の順に変更。

---

## 5. Go2 規模ベンチ結果(D8 実測、2026-07-11)

`misa-wbc examples/formulation_bench.rs`(nv=18, nc=4, na=12、現実的
質量・トルク限界・足配置の合成問題、50回中央値、--release)。

| form | strategy | backend | med [ms] | Δq̈ vs 基準 | eom 残差 | contact 残差 |
|---|---|---|---|---|---|---|
| Explicit (42var) | NullSpace | Clarabel | **2.95** | 基準 | 1.6e-8 | 4.3e-2 |
| AccelSpace (30var) | NullSpace | Clarabel | **2.30** | 2.5e-5 | 1.7e-8 | 4.3e-2 |
| ForceSpace (24var) | NullSpace | Clarabel | **1.67** | 3.9e-5 | **2.9e-14** | 4.3e-2 |
| Explicit | ForceBudget | Clarabel | 1.55 | 2.2e1 | **2.3e1 ✗** | 1.2e-1 |
| AccelSpace | ForceBudget | Clarabel | 3.05 | 2.2e1 | **1.4e2 ✗** | 1.2e-1 |
| ForceSpace | ForceBudget | Clarabel | 1.99 | 2.2e1 | 1.8e-14 | **6.0e0 ✗** |
| (全 form) | (全 strat) | ActiveSet | 100–340 | — | — | MaxIterations で全滅 |

**主要な発見**:

1. **定式化軸はロボット規模で成立**: NullSpace+Clarabel なら3定式化が
   ~1e-5 で一致、物理残差も同一。D8 の等価性が実寸で確認された。
2. **ForceSpace(GID 変数空間)が最速**: 2.95→1.67 ms(−43%)。
   変数 42→24 の縮小がそのまま効き、しかも EoM は構造的に厳密
   (1e-14 vs 1e-8)。**GID の変数空間の選択は速度面で正しかった**。
3. **GID の戦略は GID の定式化と不可分**(直交2軸にした事で発見):
   ForceBudget は Explicit/AccelSpace では EoM 等式自体を下位レベルが
   踏み潰し物理破綻(残差 23〜140)。ForceSpace でのみ物理的に整合
   (EoM が構造なので壊しようがない)— そこでは代わりに文書化済みの
   貪欲性(上位の接触等式が姿勢レベルに踏まれ残差 6.0)が現れる。
   → **GID 忠実な歩行モードには、接触を「等式タスク」でなく永続制約
   (±ε 不等式対など)として編成する必要がある**(Step B の設計指針)。
4. **ActiveSet backend はこの規模で全滅**(MaxIterations、100-340 ms)。
   ロボット規模は Clarabel 一択。
5. NullSpace 行の contact 残差 4.3e-2 は3定式化で完全一致 → 解法でなく
   問題(Kim 流 slack が等式残差と iq slack を同時最小化する設計)由来。

**Step B への示唆**: 実歩行 A/B は「ForceSpace+NullSpace(最速の厳密解)」
を第一候補に。GID 完全等価(ForceBudget)は接触の永続制約化を入れてから。

### 5b. Franka Panda ベンチ(実 URDF 動力学、2026-07-11)

`examples/panda_bench.rs` — bullet3 の inertial 付き panda.urdf を
misarta-formats → build_model で読み、CRBA/RNEA/Jacobian/**J̇·v(新API)**
の実行列でベンチ。nv=9(7 arm + 2 finger)、固定ベース(nc=0, n_base=0)。
変数: Explicit 18 / AccelSpace 9 / ForceSpace 9。

| form | strategy | backend | med [ms] | Δq̈ | eom | ee-task |
|---|---|---|---|---|---|---|
| Explicit | NullSpace | Clarabel | 0.596 | 基準 | 1.4e-11 | 8.1e-10 |
| AccelSpace | NullSpace | Clarabel | 0.533 | 4.0e-8 | **0(構造的)** | 4.7e-8 |
| **ForceSpace** | **NullSpace** | Clarabel | **0.387** | 2.2e-9 | 2.8e-14 | 3.3e-10 |
| ForceSpace | ForceBudget | Clarabel | 0.208 | 3.9e0 | 9.7e-15 | **1.6 ✗** |
| ForceSpace | ForceBudget | ActiveSet | **0.005** | 3.9e0 | 7.7e-15 | 1.6 ✗ |

**発見(アーム編)**:
1. 実モデル経路(URDF→misarta→misa-wbc)が端から端まで成立。
   3定式化は ~1e-8 一致、EE タスク達成 1e-10。ForceSpace 最速(−35%)。
2. 固定ベースでは AccelSpace/ForceSpace とも EoM が構造的に厳密。
   接触等式が無いので budget cascade も物理整合 — ただし**下位の姿勢
   レベルが EE タスクを踏む**(残差1.6)のは Go2 と同型。GID 忠実運用は
   「姿勢は同レベル内の小重み」が正しく、レベル分離は不向き。
3. **ActiveSet はアーム規模なら実用**(ForceSpace+Budget で 5µs!)。
   backend の適否は問題規模依存(Go2 規模では全滅)。

**総括**: 「ForceSpace + NullSpace + Clarabel」が脚・アーム両方で
最速の厳密解。GID 等価(Budget)は物理整合の条件(ForceSpace 必須、
等式タスクをレベル跨ぎで守らない)ごと実測で特徴付けられた。

### 5c. ActiveSet の qpOASES 化(fbc9a30、2026-07-12)

自前 ActiveSet に qpOASES の2大アイデアを移植:
(1) **増分因子更新** — Schur 補行列の Cholesky を作業集合の追加/削除で
O(m²) 更新(毎反復の再構築+LU O(n²m+m³) を廃止)、
(2) **作業集合の warm start**(QpWorkspace + solve_qp_warm)— 前回の
解と作業集合を持ち越し、摂動時は等式射影+不等式修復で再出発。
+ Bland 巡回対策。

Go2 規模の重み付き QP(n=42、iq 44行)×100 tick(参照が正弦ドリフト):

| solver | med [ms] | total iters |
|---|---|---|
| ActiveSet cold | 0.136 | 2500 (25/tick) |
| **ActiveSet warm** | **0.019** | **124 (1.2/tick)** |
| Clarabel | 1.037 | — |

→ **warm active set は Clarabel の54倍速**(解は 2e-6 一致)。
「WBC の毎 tick ほぼ同一 QP では warm active-set が IPM に勝つ」という
GID/OpenSoT の backend 選択の定説を自前実装で再現。
formulation_bench の HoQp 内部(cold、workspace 未配線)も
100-340ms → 2-12ms(10-30倍)。残 DNF は反復数由来で、
**HoQp/戦略層への QpWorkspace 配線(Solver セッション化)が次の一手**。

### 5d. Solver セッション + 条件付きリッジ(2941ae2、2026-07-12)

qpOASES 化の完成形。(1) **Solver セッション**: レベル毎の QpWorkspace を
HoQp / budget cascade に配線し、階層全体で tick 跨ぎ warm start。
(2) **条件付きリッジ + KKT polish**(欠けていた qpOASES 要素): 全ベンチの
DEGRADED の真因は HoQp 内部 QP の悪条件(H=AᵀA+1e-12、κ≈1e8+)による
這い歩きだった。Cholesky ピボット比で検出し κ を 1e6 に制限して反復、
Optimal 出口で元の H の KKT を最終能動集合上で解き直して無正則化解を回復。

formulation_bench(Go2 規模、セッション使用)— **DEGRADED 全滅、
ActiveSet が全12組合せで Clarabel を逆転**:

| 組合せ | ActiveSet | Clarabel |
|---|---|---|
| ForceSpace+NullSpace | **0.64 ms** | 2.70 ms |
| Explicit+NullSpace | 1.79 ms | 4.42 ms |
| ForceSpace+ForceBudget | **0.095 ms** | 2.50 ms |

解は基準と 5e-5 一致、EoM 残差は polish 効果で 9e-11(Clarabel 1.6e-8 より良)。
qp_warm_bench: warm 0.030 ms vs Clarabel 1.83 ms。

**総括の更新**: 「毎 tick ほぼ同一 QP では warm active-set が IPM に勝つ」が
solve() 全体で成立。**推奨構成は ForceSpace + NullSpace + ActiveSet
(セッション)= 0.64 ms** — 当初の qpOASES 比「桁2-3個劣る」評価から、
同じ設計原理の移植で同等クラスまで到達。

### 5e. 自前内点法 IPM(535e01e、2026-07-13)

Clarabel(成熟した汎用 conic ソルバ、既に IPM)とは別に、**教科書的な
primal-dual path-following 法**(Nocedal & Wright Alg.16.4、Mehrotra
predictor-corrector)を `qp.rs` に自前実装。ActiveSet の qpOASES 化と
同じ精神 — 「アプローチ自体を内部構造まで見える形で比較する」。

各反復: affine-scaling(μ=0)予測ステップで到達可能な centering を
見積り Mehrotra の σ=(μ_aff/μ)³ を導出 → corrector ステップ(centering
目標 + 予測ステップの2次項 Δs∘Δz)で実際に前進。dx,dy は reduced KKT
(Cholesky→Schur)、ds,dz は閉形式で消去 — ActiveSet の1反復と同じ2段
構成だが、H が毎反復バリア項で変わるため増分更新は使わない。

**実装中に発見したバグ**: complementarity 式から (ds,dz) を消去する際
`+A_iqᵀz` 項(dz 側は `−z` 項)を1箇所落としていた。ランク不足の
検証問題で ActiveSet との解の差が 2.43e-3 と大きく出て発覚 → 目的関数
値・stationarity は一致していたため「null space の自由度」の可能性を
疑い、フルランクの検証問題で切り分け(バグがあれば再現するはず) →
実際にバグと判明、修正後はフルランクで 4.79e-9 一致に改善。

Go2 規模ベンチ(3 backend目): **IPM は全12組合せで Optimal**(DEGRADED
無し)、物理残差は ActiveSet 同等以上(ForceSpace+NullSpace の EoM
6.3e-15 vs ActiveSet 5.7e-14)、速度は Clarabel より速く warm-started
ActiveSet には及ばない(予想通り — 今回の実装は tick 間状態を持たない)。

**総括**: misa-wbc は今や4 backend(Clarabel/ActiveSet/Ipm + 将来の
拡張)を同一 API で比較できる状態。IPM 自体の理解・検証には有用だが、
実用推奨は変わらず **ForceSpace + NullSpace + ActiveSet(セッション)**。

### 5f. IPM vs ActiveSet 専用ベンチ(40a8a64、2026-07-14)

Clarabel を排し、この2つの自前実装(IPM 006cb57 / ActiveSet fbc9a30)を
直接比較。

**Go2 tick 列**(qp_warm_bench、n=42・iq44行・100 tick ドリフト):

| solver | med [ms] | 反復数合計 |
|---|---|---|
| ActiveSet cold | 0.084 | 2500 |
| ActiveSet warm | **0.011** | 124 |
| **Ipm** | 0.349 | 1543 |
| Clarabel | 0.687 | — |

→ 自前 IPM は **Clarabel より2倍速い**(0.349ms vs 0.687ms)が、warm
ActiveSet には及ばない(tick 間状態を持たないため)。解は 1e-12 一致。

**サイズスケーリング**(ipm_vs_active_set_scaling、cold start、n=5〜160
を32倍、m_iq≈1.1n、30回中央値、**反復数**に着目):

| n | m_iq | IPM 反復 | AS 反復 |
|---|---|---|---|
| 5 | 5 | 7 | 2 |
| 160 | 176 | **11**(+57%) | **40**(+1900%、≈線形) |

→ **教科書どおりの定性的差を定量確認**: IPM の反復数はほぼ一定
(毎反復が全制約を Newton 1発で考慮)、ActiveSet の反復数は制約数に
ほぼ線形(cold start から最適解までに状態変化する制約数に依存)。
ただし絶対時間は全サイズで ActiveSet が依然速い — この IPM 実装は
毎反復 H を丸ごと再因数分解(O(n³))するのに対し、ActiveSet の増分
更新は O(m²)。反復数優位が壁時計時間で逆転するには、n がさらに
大きいか IPM の1反復コストを下げる必要がある(未検証の閾値)。

**総括**: 反復数の観点では IPM の理論的優位性を自前実装で再現。
壁時計時間では、この規模(WBC tick 相当)では ActiveSet(特に warm)
が一貫して有利 — 推奨構成(ForceSpace+NullSpace+ActiveSet)は変わらず。

### 5g. 自前 ADMM(OSQP アルゴリズム、89d2a9e、2026-07-14)

Clarabel(IPM)・ActiveSet・自前 IPM に続く**第4の、かつ根本的に異なる
パラダイム**として OSQP の ADMM(operator splitting)を自前実装。

**核心**: 問題を「等式制約 QP(線形解1回)」+「箱射影(閉形式)」に分割
し交互反復。ρ,σ を固定する限り **線形システムの行列 M=H+σI+ρAᵀA は
反復を通じて不変** — IPM(毎反復 O(n³) 再分解)・ActiveSet(作業集合
変化ごとに O(m²) 増分更新)のどちらとも違い、**最初に1回 Cholesky
すれば以降は後退代入だけ**。作業集合もバリアも一切管理不要。

**検証**: KKT 証明書・フルランク一致(ActiveSet と一意解比較)・
等式のみ/無制約の高速パス・`MaxIterations` でも発散しないことの
確認、76 tests green。

**正直な弱点(実測)**:

1. **線形収束の代償**: フルランク10変数のランダム QP で
   optimality_tol=1e-8 に達するのに **約770反復**(IPM/ActiveSet は
   一桁)必要。デフォルト max_iters=500 を超える(ただし解自体は
   3e-8 まで正確 — 発散はしない、品よく劣化するだけ)
2. **固定 ρ の WBC 規模での破綻**: Go2 tick 列(n=42、条件数
   2.3e5、重力スケール項と O(1) タスク項が混在)で ρ=10 固定では
   **1 tick あたり約2000反復**必要 — ベンチの全100 tick が
   デフォルト設定で MaxIterations。同程度の条件数の合成問題単体
   では約35反復で収束したので、**悪条件そのものでなく WBC 特有の
   タスク重み・スケール混在が効いている**ことを確認済み
3. formulation_bench(HoQp 内部、12組合せ)でも全て DEGRADED —
   ただし物理残差(EoM 等)はしばしば優秀(ForceSpace+ForceBudget で
   2.8e-16)で、誤答ではなく同じ「品よい劣化」パターン

**結論**: ADMM の謳い文句(1回だけ因数分解・反復が安い・warm-start
がほぼ無償)は KKT/クロスチェックテストで実証された事実だが、
**固定 ρ の WBC 規模での弱さは理論でなく実測で確認**された —
まさに OSQP 自身が適応的 ρ 再調整を持つ理由そのもの。この backend
を比較用でなく実用にするには、適応的 ρ 再調整(ρ 変更時に M の
再分解が必要になり「1回だけ分解」の性質を部分的に失う)の実装が
自然な次の一手。

### 5h. ADMM 適応的 ρ 再調整(3e891f9、2026-07-14)

§5g で見つかった「固定 ρ の WBC 規模での破綻」に対し、OSQP 本来の
適応的 ρ 再調整(§5.2)を実装。`ρ ← ρ·√[(r_prim/scale_p)/(r_dual/scale_d)]`
を3反復ごとにチェックし、[0.2, 5]× の帯から外れた時だけ再分解
(factorise-once の精神を保つため、毎反復ではなく間引いてチェック)。

**単一 QP(qp_warm_bench、Go2 tick 列)では劇的改善**:

| | 固定ρ | 適応的ρ |
|---|---|---|
| DEGRADED | 100/100 | **0/100** |
| med [ms] | 2.450 | **0.193**(≈13倍) |
| vs Clarabel(0.9ms)・Ipm(0.37ms) | 両方より遅い | **両方より速い** |

**HoQp 階層構造内(formulation_bench)では限定的**: ForceSpace+NullSpace
は完全 Optimal 化(基準と 1e-4 一致、EoM 残差 2.9e-14)。しかし
Explicit(42変数)・AccelSpace(30変数)は特に NullSpace 戦略下で大半が
依然 DEGRADED。適応的 ρ は「スケールの不整合」を補正する設計なので、
**HoQp の null-space cascade 特有の構造(レベル毎の等式重み+スラック
変数)が生む問題形状には万能ではない**ことが分かった。

**総括**: 「1回だけ因数分解 + warm-start がほぼ無償」という ADMM の
謳い文句は、**単一 QP・WBC tick 相当のシナリオ(まさに ADMM を検討した
動機)では完全に実証**された。一方 HoQp 内部の複雑な QP 形状には
適応的 ρ だけでは対処しきれない場合がある — これは文献が ρ 再調整を
「保証でなくヒューリスティック」と位置づけている通りの結果。

**ProxQP 試作の要否判断**: HoQp 内部で残る DEGRADED は、ADMM 自体の
限界というより「その特定の QP 形状への対処」の問題であり、これを
解くのに ProxQP(augmented Lagrangian + 内側 semismooth Newton)が
必ず優位とは限らない — ProxQP も基本は同じ ADMM 系譜の発展形で、
実装コストは IPM 並みかそれ以上(内側 Newton + 外側乗数更新の二重
ループ)。**当面は ProxQP 試作を保留**し、必要になった時点(HoQp
内部 QP の悪条件対策そのものを掘り下げる時)に着手するのが妥当と判断。

### 5i. Gondzio Multiple Centrality Correctors(9325972、2026-07-14、文献調査あり)

IPM のさらなる高速化として、Colombo & Gondzio 2008(COAP、Gondzio 1996
の再定式化)を文献取得(Optimization Online のプレプリント PDF を直接
解析し式(7)の右辺構築式を確認)して実装。`QpSolver::IpmMcc`。

**手法**: predictor + Mehrotra corrector の後、**同じ因数分解**を使い
回して追加の corrector を最大3回計算。各 corrector は「試行点で
(γμ, μ/γ) 帯から外れる outlier な相補性積のみ」を補正対象にし、
線形探索で重み ω を選んで合成(ステップ長の積 α_P·α_D を最大化)。
新規因数分解は一切発生させず、安価な後退代入だけを追加する設計。

**正直な実測結果(良し悪しの両方)**:

| 規模 | 反復数 | 時間 |
|---|---|---|
| n=160(合成問題) | 11→10(ほぼ不変) | **約2倍に悪化** |
| Go2 tick 列(qp_warm_bench) | 1543→1301(−16%) | 0.33ms→**0.65ms(+97%)** |

反復数(=因数分解回数)はわずかに減るが、各反復の追加コスト(最大3回の
solve + 8点線形探索)がその節約を上回り、**misa-wbc の規模(密行列・
n<200)では正味で遅くなる**。解自体は Ipm と完全一致(KKT 同一点)で
実装は正しい — これは「手法が想定する適用域(疎行列・大規模 LP、
因数分解が後退代入よりずっと高価な場面)と、misa-wbc の対象規模
(密・小〜中規模、因数分解自体が既に安い)のミスマッチ」という結論。

**設計判断**: `Ipm` のデフォルト挙動には混ぜず、比較専用の別 backend
`IpmMcc` として分離(退化させない)。「洗練された手法 = 常に速い」で
はないことの具体的な反例として記録する価値ありと判断し、削除せず
実装・テスト・記録を残した。

**総括(IPM 高速化施策のまとめ)**: Mehrotra predictor-corrector(基本)
→ 条件付きリッジ+KKT polish(悪条件対策、既存)→ MCC(この規模では
逆効果)。misa-wbc 規模での実用的な IPM 高速化は、warm-start(ADMM
同様の tick 間状態保持)の方が筋が良い可能性が高い — 次の一手の候補。

### 5j. IPM warm-start(337b406、2026-07-14)

ActiveSet の qpOASES 化で使った `QpWorkspace` を IPM(`Ipm`・`IpmMcc`)にも
拡張。前回 tick の `(x, s, z, y)` を次の解の出発点にする(コールド
`x=0, s=z=1` の代わりに)。前回解は境界近傍にあるため、再利用前に
`s, z` を `1e-3` で床上げして修復(ActiveSet の「実行可能域への射影」に
相当するが、IPM には射影すべき「集合」がなく正値域があるだけなので、
床上げが修復の全て)。形状が変わった tick は自動的にコールドスタートに
フォールバック。

**効果(予告どおり ActiveSet ほど劇的でないが、実測は大きい)**:

| | Ipm(cold) | Ipm warm |
|---|---|---|
| 反復数(qp_warm_bench) | 1543 | **411(−73%)** |
| 中央値 | 0.36ms | **0.072ms(≈5倍)** |
| vs Clarabel(0.77ms)・IpmMcc(0.49ms) | 両方より遅い | **両方より速い** |
| vs ActiveSet warm(0.012ms) | — | まだ及ばない(組合せ的な作業集合再利用と、パス上の位置の再利用の違い) |

**formulation_bench は変更ゼロで自動的に高速化**(既に 2941ae2 の
Solver セッション配線が HoQp/戦略層に通っていたため)。例:
ForceSpace+NullSpace が 2.5ms→0.46ms。

**総括(IPM 高速化の全体像)**: Mehrotra 基本 → 条件付きリッジ+polish
(悪条件対策)→ MCC(この規模では逆効果、比較用に温存)→ **warm-start
(この規模で最も効果的な IPM 高速化)**。misa-wbc の4 backend
(Clarabel/ActiveSet/Ipm/IpmMcc)+ ADMM の中で、**warm 状態を持つ
ActiveSet が依然最速**だが、warm IPM はコールド勢(Clarabel・IpmMcc・
Admm)を全て逆転する実用的な選択肢になった。

### 5k. 全 backend 総合ベンチマーク(2026-07-14 改めて実施)

これまで個別に見てきた6 backend(Clarabel/ActiveSet cold・warm/
Ipm cold・warm/IpmMcc cold・warm/Admm)を横断して再計測。

**① Go2 tick 列(qp_warm_bench、n=42・iq44行・100 tick ドリフト、中央値)**

| solver | med [ms] | 反復数 | 対 Clarabel |
|---|---|---|---|
| **ActiveSet warm** | **0.013** | 124 | 54倍速 |
| Ipm warm | 0.070 | 411 | 10倍速 |
| **IpmMcc warm** | 0.093 | 409 | 8倍速 |
| ActiveSet cold | 0.098 | 2500 | 7倍速 |
| Admm(適応的ρ) | 0.260 | 3700 | 2.7倍速 |
| Ipm(cold) | 0.358 | 1543 | 2倍速 |
| IpmMcc(cold) | 0.450 | 1301 | 1.6倍速 |
| Clarabel | 0.702 | — | 基準 |

新発見: **IpmMcc warm(0.093ms)は Ipm warm(0.070ms)にほぼ並ぶ** —
warm start で解が最適解近傍から出発すると、centrality corrector が
補正すべき外れ値がほぼ無くなり、MCC のオーバーヘッドがその効果と
一緒に縮む(コールドでは MCC が最も重かったのと対照的)。

**② HoQp 階層内(formulation_bench、Solver セッション経由で自動 warm)**

全18組合せ(3 formulation × NullSpace/ForceBudget × Clarabel/ActiveSet/
Ipm/IpmMcc、+Admm)中、**ActiveSet と Ipm(warm)が僅差でトップを分け合う**:

| 組合せ | ActiveSet | Ipm | 差 |
|---|---|---|---|
| ForceSpace+ForceBudget | **0.032ms** | 0.095ms | AS が3倍速 |
| AccelSpace+ForceBudget | **0.051ms** | 0.140ms | AS が2.7倍速 |
| ForceSpace+NullSpace | 0.378ms | **0.399ms** | ほぼ互角 |
| Explicit+NullSpace | 0.619ms | **0.530ms** | Ipm がわずかに速い |

IpmMcc は Ipm よりわずかに遅いが全組合せで Optimal(cold 版の
DEGRADED が warm で解消)。Admm は依然 Explicit/AccelSpace+NullSpace
で DEGRADED(§5h/§5g の既知の限界、HoQp 内部構造の悪条件)。

**③ サイズスケーリング(ipm_vs_active_set_scaling、cold・n=5→160)**

| n | IPM 反復 | AS 反復 | IPM/AS 時間比 |
|---|---|---|---|
| 5 | 7 | 2 | 6.3倍 |
| 160 | 11(+57%) | 40(+1900%、≈線形) | 4.8倍 |

反復数の伸びは IPM が圧倒的に緩やか(§5c の教科書どおりの結果を
再確認)だが、**この規模ではまだ絶対時間で ActiveSet が優位** — IPM
の1反復コスト(O(n³) 再分解)が ActiveSet の増分更新(O(m²))より
重いため。反復数優位が時間で逆転するには n がさらに大きい必要がある。

**総合順位(Go2 規模・tick 列、実用構成)**:

```
ActiveSet warm(0.013ms)
  > Ipm warm(0.070ms)
    ≈ IpmMcc warm(0.093ms)
      > ActiveSet cold(0.098ms)
        > Admm 適応的ρ(0.260ms)
          > Ipm cold(0.358ms)
            > IpmMcc cold(0.450ms)
              > Clarabel(0.702ms)
```

**変わらぬ推奨**: **ForceSpace + NullSpace + ActiveSet(Solver
セッション)** が misa-wbc の実用構成。IPM 系(特に warm-started Ipm)
は「ActiveSet が使えない・warm start の前提が崩れる場面」の堅牢な
代替、ADMM は「1回だけ因数分解」という別の設計思想の実証として、
Clarabel は商用グレード IPM の基準点として、それぞれ比較の価値を
保持している。

### 5l. IPM 反復数優位の逆転点調査(59ec0be、2026-07-12、追加調査)

§5f/§5k で「反復数優位が時間で逆転するには n がさらに大きい必要がある
(未検証)」としていた点を実測。n=5・…・160(既存)→ **640・1280・2560・
5120 まで拡張**(cold start、密行列)。

**副産物として発見したバグ**: `max_iters` を全サイズで固定500のまま
にしていたため、n=2560 で ActiveSet が(收束の途中で)上限に到達し
`MaxIterations` で panic。実際には508回で真に収束することが判明 —
反復数が genuinely n に比例して増えるので `max_iters` も n に比例
させる必要があった(`3n` に変更して解決)。

**実測(AS/IPM 実時間比、1.0 超で IPM 逆転)**:

| n | 160 | 320 | 640 | 1280 | 2560 | 5120 |
|---|---|---|---|---|---|---|
| AS/IPM | 0.22(底) | 0.25 | 0.35 | 0.54 | 0.61 | 0.66 |
| Δ(前段階比) | — | +0.03 | +0.10 | **+0.19(ピーク)** | +0.07 | +0.05 |

n=160 の底から着実に回復するが、**640→1280 でピークを打ってから
伸びが減速**(+0.19→+0.07→+0.05)。1.0 に向けた加速ではなく、
**1.0 未満のどこかへの漸近**という形状。

**理論的な説明**: 両 backend ともこの密行列実装では漸近的に O(n³) —
IPM は反復数ほぼ一定(7→14、1024倍で+2倍)× 毎回 O(n³) 再分解、
ActiveSet は反復数ほぼ線形(2→959)× 毎回 O(n²) 増分更新(合計
O(n³))。**両者ともオーダーは同じで、係数の差だけが順位を決めており、
ActiveSet の係数がこの実装では一貫して小さい**。

**結論**: n=5120(WBC 実務規模 n<200 の25倍以上)まで調べても逆転せず。
misa-wbc の推奨(ActiveSet、可能なら warm-start)はサイズを理由に
揺らがない。真の逆転を探すなら、生の密行列スケールではなく
**WBC タスクのブロック構造を使った疎行列化**(D7、long-term保留)の
方向が筋 — オーダー自体を変える改善でないと、この密行列同士の勝負
では ActiveSet が勝ち続ける。

### 5m. 特異点近傍の安定性比較(Formulation × Backend、panda_singularity_demo.rs、2026-07-13)

これまでのベンチマークはいずれも「特異点から離れた」通常姿勢での
速度・反復数比較だった。今回は逆に、**わざと特異点に近づけたときに
各構成がどう壊れるか**を比較する。対象は引き続き Franka Panda(実
URDF)。

**軌道設計**: 手首特異点(特定の関節値を厳密に狙う必要があり脆い)
ではなく、**到達境界特異点(リーチ限界)**を使う — シリアルリンク機
構なら方向を問わず一般に成立する、狙いやすい特異点。肩関節原点
(0,0,0.333)を通る固定方向(静止姿勢の EE 方向)に沿って、
reach = 0.68 ± 0.13 m(0.55〜0.81m)を周期2.5s で往復、5s(2往復)
実行。タスク構成は `panda_circle_demo.rs` と同じ(優先度0:
`dynamics_task + box_bound(τ,τ_max)`、優先度1: `cartesian_acceleration`、
優先度2: 正則化)。`HqpStrategy` は `NullSpace` に固定 — GID 型の
`ForceBudgetCascade` は優先度解決そのものが違う軸で、特異点での
数値挙動比較としては性質が異なるため、今回は対象外(将来課題)。

**比較軸**: `Formulation`(Explicit / AccelSpace / ForceSpace) ×
`QpSolver`(ActiveSet / Ipm / Admm / Clarabel、IpmMcc はこの規模で
Ipm に劣後することが既知(§5k)のため除外)= 12 通り。

**指標**: `sigma_min(J_lin)`(EE 位置ヤコビアンの最小特異値、
0 に近いほど特異点に近い)、EE 追従誤差、`|τ|` 最大、`|q̈|` 最大、
`SolveStatus::Degraded` になった tick 数(1250 tick 中)。

**結果(全12通り)**:

| formulation | backend | σ_min(最小) | 追従誤差(最大) | 追従誤差(平均) | τ(最大abs) | q̈ ノルム(最大) | degraded |
|---|---|---|---|---|---|---|---|
| Explicit | ActiveSet | 0.0191 | 0.229 | 0.0109 | 87.00 | 1,501 | 9/1250 |
| Explicit | Ipm | 0.0160 | 1.515 | 0.0951 | 87.00 | 5,291,865 | 680/1250 |
| Explicit | Admm | 0.0639 | 0.942 | 0.0532 | 87.00 | 19,525 | 717/1250 |
| Explicit | Clarabel | 0.0027 | 0.229 | 0.0072 | 87.03 | 1,286 | 2/1250 |
| AccelSpace | ActiveSet | 0.0485 | 1.565 | 0.4902 | **18,251** | 2,776 | 810/1250 |
| AccelSpace | Ipm | 0.0106 | 0.944 | 0.0460 | 87.00 | 1,275 | 507/1250 |
| AccelSpace | Admm | 0.0107 | 1.211 | 0.2804 | **319,350** | 1,899 | 835/1250 |
| AccelSpace | Clarabel | 0.0046 | 0.229 | 0.0087 | 87.00 | 979 | 25/1250 |
| ForceSpace | ActiveSet | 0.0077 | 0.264 | 0.0298 | 87.00 | 2,657 | 211/1250 |
| ForceSpace | Ipm | 0.0279 | 0.229 | 0.0176 | 87.00 | 1,567 | 706/1250 |
| ForceSpace | Admm | 0.0305 | 0.229 | 0.0157 | 87.00 | 2,250 | 658/1250 |
| ForceSpace | Clarabel | 0.0026 | 0.229 | 0.0273 | 87.00 | 1,464 | 49/1250 |

グラフ: `singularity_explicit_backends.png`(Explicit 固定・4 backend の
σ_min/追従誤差/q̈ノルム時系列 + 全12通りの degraded tick 数棒グラフ)、
`singularity_grid_heatmap.png`(3×4 グリッドの degraded tick 割合ヒート
マップ)。いずれも `misa-wbc/examples/media/` に保存済み(git 未
コミット、下記参照)。

**主要な発見**:

1. **Clarabel が全 formulation で圧倒的に頑健**(degraded 0〜4%、
   σ_min も 0.003〜0.005 まで — 他 backend が発散し始める領域まで
   平然と食い込む)。商用グレード IPM の余裕がそのまま出た形。
2. **ActiveSet は Explicit/ForceSpace では優秀(1%/17%)だが、
   AccelSpace で激しく悪化(65%)し、しかも `box_bound(τ,τ_max)` の
   上限(87 Nm)を大幅に超えた τ=18,251 Nm を返す**。これは制約式
   自体のバグではなく、QP が非最適(`MaxIterations`/`Infeasible`)の
   まま返した `x` を `extract()` がそのまま物理量に変換した結果と
   考えられる — box_bound は AccelSpace では τ を qddot/f の
   アフィン式として拘束するため、ソルバーが収束しない tick では
   その拘束自体が事実上意味を失う。AccelSpace + Admm でも同様
   (τ=319,350 Nm)に破綻しており、**τ を qddot から M 経由で導出する
   AccelSpace は、J が特異になる場面で ActiveSet/Admm と特に相性が
   悪い**ことが分かる。ForceSpace(τ が主変数で qddot を M⁻¹ 経由
   導出)や Explicit(τ が独立変数)ではこの暴走は起きていない —
   M 自体は特異点で悪条件化しないため、τ の導出経路が J の特異性を
   増幅するかどうかが分かれ目と見られる。
3. **Ipm と Admm は formulation を問わず中〜悪(41〜67%)**で一貫して
   劣る。特に Ipm は Explicit で q̈ ノルムが 529 万 rad/s² まで暴走
   しており、内点法の barrier が特異点近傍の悪条件化にうまく対応
   できていない。
4. **追従誤差に共通の"床"(≈0.229m)が多くの良好な構成で現れる** —
   これは特異点直上での物理的に不可避な瞬間誤差(その方向への
   即応加速度が原理的に出せない)であり、ソルバーの良し悪しとは
   別の下限と解釈できる。Ipm/Admm の 0.9〜1.6m はこの床を大きく
   超えており、真の不安定化と言える。

**動画(5本)**: `panda_singularity_demo` の実 Panda メッシュレンダリング
(`render_panda_vtk.py` を汎用化して再利用)。

- Explicit × ActiveSet / Ipm / Admm / Clarabel(4本、同一軌道・同一
  formulation での backend 比較)
- AccelSpace × Admm(τ=319,350 Nm の破綻ケースを可視化する目的の
  ワーストケース1本)

formulation 間(Explicit/AccelSpace/ForceSpace)は数学的に等価な
定式化なので、収束さえすれば軌道自体は視覚的にほぼ同じになる —
違いは数値的頑健性(上表)に出るのであって見た目には出にくい。その
ため動画比較は主に **backend 間**の可視化に充てた。

**推奨**: 特異点近傍でも安定性を最優先するなら **Clarabel** が最も
安全。リアルタイム性能を優先して ActiveSet を使う場合、
**AccelSpace は避け、Explicit か ForceSpace に留めるべき**(§5k の
既存推奨「ForceSpace + NullSpace + ActiveSet」は本結果とも整合する
— ForceSpace は AccelSpace ほど悪化しない)。

**限界 / 今後の課題**: `HqpStrategy::ForceBudgetCascade` との比較は
未実施(優先度解決という別軸のため、今回は NullSpace に固定)。
AccelSpace + ActiveSet/Admm の τ 暴走の根本原因(非最適解からの
extract が生む物理的に無意味な値、という以上の解析)は未着手。D6
の関節限界 CBF と組み合わせた場合に degraded 率がどう変わるかも
未検証。

### 5n. QP側での特異点低感度化 — Damped Least Squares タスク(2026-07-13)

§5m を踏まえ、「使用者側への負担を増やさずに特異点近傍の挙動を
改善できないか」を検討。検討した4案(タスク側の動的減衰/操作性
CBF/`prox_weight`の動的増加/零空間での操作性最大化)のうち、
**追加のライブラリ機能なしで組める古典的DLS(damped least squares)**
を採用 — `cartesian_acceleration` と同じ優先度レベルに
`regularize(qddot, 0).weight(λ²)` を足すだけで
`min ‖J·q̈+J̇v−a_ref‖² + λ²‖q̈‖²` が組める(`HqpStrategy::NullSpace`
では下位優先度の正則化はこのレベル自身の解には効かないため、
**同じレベルに足す**ことが必須)。操作性CBFや零空間勾配法は
`∂J/∂q`(時間微分`J̇v`とは別物)という misarta に無い微分計算が
要るため見送り、将来課題とした。

**API 設計(使用者負担の最小化)**: `tasks::cartesian_acceleration`
の**ドロップイン代替**として `tasks::cartesian_acceleration_damped`
を追加(同じ`qddot,j,dj_v,accel_ref`+`&SingularityDamping`一つだけ
追加)。`SingularityDamping`は`sigma_lo`(完全減衰域)/`sigma_hi`
(減衰オフ域)/`lambda_max_sq`(最大減衰重み)の3値、`Default`実装
込み(Panda級アームの経験的デフォルト: 0.01 / 0.08 / 5e-3)。
`sigma_min(J)`の計算(SVD)は関数内部で行うため、呼び出し側は
`σ_min`もランプ関数も一切意識しない。単体テスト3件
(`src/tasks.rs`): 健全域で無効化されること、特異点で全開すること、
合成2x2ヤコビアン`diag(1,θ)`で θ→0 でも解析的下界内に収まること
(素朴な最小二乗は`q̈∝1/θ`で発散するのに対し、DLSは有界)。

**実機ベンチマークでの検証(§5m と同一12通り、`--damped`フラグ追加)**:

| formulation | backend | undamped degraded% | damped degraded% |
|---|---|---|---|
| Explicit | ActiveSet | 0.7% | **27.7%(悪化)** |
| Explicit | Ipm | 54.4% | 3.9% |
| Explicit | Admm | 57.4% | 2.1% |
| Explicit | Clarabel | 0.2% | **発散(数値破綻、q が1e13超まで発散)** |
| AccelSpace | ActiveSet | 64.8% | 54.1%(改善するがτ超過は悪化) |
| AccelSpace | Ipm | 40.6% | 0.2% |
| AccelSpace | Admm | 66.8% | 39.9% |
| AccelSpace | Clarabel | 2.0% | 1.0% |
| ForceSpace | ActiveSet | 16.9% | 0.1% |
| ForceSpace | Ipm | 56.5% | 0.3% |
| ForceSpace | Admm | 52.6% | 3.1% |
| ForceSpace | Clarabel | 3.9% | 2.3% |

グラフ: `damping_before_after.png`(12通り undamped/damped 棒グラフ、
発散ケースは赤で明示)。動画2本追加: `sing_explicit_ipm_damped.mp4`
(680→49劣化tickの劇的改善を可視化)、
`sing_explicit_clarabel_damped_blowup.mp4`(発散の様子、1195/1250
tick で打ち切り — それ以降は数値的に無意味)。

**重要な発見 — バックエンド依存の非対称性**:

1. **Ipm と Admm には例外なく効く**(6通り全てで劣化率が大幅減、
   最大で340倍改善)。両者とも「自前の悪条件対策を持たない」
   backend であり、外付けのDLS減衰がそのまま効く。
2. **ActiveSet と Clarabel は予測不能** — 良くなる場合
   (ForceSpace+ActiveSet: 16.9%→0.1%)もあれば、悪化する場合
   (Explicit+ActiveSet: 0.7%→27.7%)、**破局的に壊れる場合**
   (Explicit+Clarabel: 発散)まである。理由は既存の内部安全機構との
   衝突と考えられる — ActiveSet には qpOASES 由来の**条件付き
   リッジ+KKT polish**(§5d)が、Clarabel には商用IPMグレードの
   スケーリング/正則化が既に入っており、そこに外付けの(スケールの
   異なる)正則化項を追加すると、内部機構が前提とする問題構造が
   崩れて逆効果になり得る。
3. **degraded tick 数という指標だけでは発散を検知できない**
   (Explicit+Clarabel の発散ケースは `SolveStatus::Degraded` の
   tick がむしろ少ない=0.5% — QP自体は「最適」と報告し続けながら
   状態量`q`が数値的に暴走した)。安全性の検証には収束率だけでなく
   状態量の有界性チェックが要る、という教訓。
4. **λ_max_sq の再チューニングでは解決しない** — 1e-3/2e-3 も試したが
   改善する組合せと悪化する組合せが入れ替わるだけで(例:
   AccelSpace+Admm は 1e-3 で τ=1600万 Nm まで悪化)、単一の
   グローバル既定値で全12通りを同時に安全にする値は見つからず、
   これ以上のチューニング探索は打ち切った。

**推奨(使用者向けの単純なルール)**: **backend が Ipm か Admm なら
`cartesian_acceleration_damped`を既定値のまま使ってよい(常に改善)。
backend が ActiveSet か Clarabel なら、検証なしに使わないこと**
(既存の内部機構で足りていることが多く、下手に足すと逆効果になり
得る)。misa-wbc の推奨構成(ForceSpace + NullSpace + ActiveSet、
§5k)は damping なしのままで問題ない。この非対称な安全域は
`tasks::cartesian_acceleration_damped`のdocコメントにも明記した。

**限界 / 今後の課題**: Explicit+Clarabel がなぜ特異的に発散するのか
(スケール不整合の具体的メカニズム)は未解明。操作性CBF・零空間
勾配法(∂J/∂q が必要)は misarta 側への機能追加を伴う別タスクとして
持ち越し。

さらに2点、副産物として着手・完了:

- **状態の有界性チェックを両デモに追加**(未コミット→
  `cb4335f`): §5n の発散ケースは `SolveStatus::Degraded` を素通り
  して発見が遅れた教訓を受け、`panda_circle_demo.rs`/
  `panda_singularity_demo.rs` の積分ループに「`q`/`v` が有限かつ
  |x|<1e3(実際の関節限界の300倍以上、誤検知の心配がない閾値)」を
  毎tick確認し、破ったら即座にログを出して打ち切るガードを追加。
  Explicit+Clarabel発散ケースで検証: 5分タイムアウト→0.8秒で
  同じ発散点を検知して終了。健全な構成には影響なし(1250tick完走)。
- **Go2脚での同一検証(下記 §5o)**。

### 5o. Go2脚での特異点近傍安定性比較(2026-07-13)

misa-wbc の本来のターゲットは quadruped-gait(Go2)であり、Panda は
あくまでベンチマーク用の腕。§5m/§5n と同じ検証を実機の脚
(`go2_leg_singularity_demo.rs`、`articara/models/unitree_go2/go2.misa`
を固定ベースのまま——floating jointなしで"架台に固定されたGo2"相当
——ロード)で行い、Panda の知見が一般化するか確認した。

**軌道設計**: FR脚のみタスク化(他3脚は優先度2の正則化で静止)。
股関節(FR_hip_joint)原点からの固定方向に沿ってreach=0.395±0.06m
を往復(股関節からの直線最大リーチは0.426m=大腿0.213m+下腿0.213m)。
Panda と異なり**脚は3自由度しかなく6Dポーズタスクは過拘束になる**
ため、追従タスクは位置のみ(ヤコビアンの並進3行のみ)——特異点で
真に正方(3x3)なヤコビアンになる、Panda(6x9、冗長)より教科書的に
素直なケース。

**重要な手戻り**: 当初、関節位置限界を課さずに実行したところ、
下腿(calf)関節が実際の可動域(-2.7227〜-0.83776 rad)を大きく超えて
-0.4755 rad まで動いてしまい、σ_minが0.02〜0.06程度にしか下がらな
かった——**これは特異点回避に成功したのではなく、シミュレータが
物理的にありえない配置(下腿関節を実際の限界を超えて曲げる)を
使って同じ目標点に到達し、真の特異点を迂回していただけ**だった。
D6の`tasks::joint_limit_cbf`(今回のセッション最初に実装したもの)
を優先度0に追加して修正——脚は自身の実限界(calf=-0.83776)で
きれいに制動し、それ以上は曲がらなくなった。副産物として、D6
CBFを実際のシナリオで再利用・再検証できた。

**結果**: CBFが効いた状態では、**σ_minは0.074で底打ちし、それ以上
下がらない**(Panda が到達した0.003〜0.06よりずっと浅い)——
**Go2脚の関節限界そのものが、ハードウェアレベルで真の特異点への
接近を防ぐ安全マージンとして機能している**ことを意味する。この
結果、**物理軌道(σ_min/追従誤差/τ/q̈)は12通り全てでほぼ完全に
同一**(σ_min=0.0741、err_max=0.172 など、backend/formulationに
よらず一致)——全員が同じ良い解に収束する。

ところが **degraded tick 数は backend によって劇的に異なる**:

| formulation | backend | undamped degraded% | damped degraded% |
|---|---|---|---|
| Explicit | ActiveSet | **0.0%** | 0.3% |
| Explicit | Ipm | **99.8%** | 99.9% |
| Explicit | Admm | 7.6% | 13.4% |
| Explicit | Clarabel | 8.8% | 6.0% |
| AccelSpace | ActiveSet | **0.0%** | 0.0% |
| AccelSpace | Ipm | **99.9%** | 18.2% |
| AccelSpace | Admm | 7.5% | 7.6% |
| AccelSpace | Clarabel | 6.7% | 4.3% |
| ForceSpace | ActiveSet | **0.0%** | 0.0% |
| ForceSpace | Ipm | **99.9%** | 46.2% |
| ForceSpace | Admm | 0.0% | 24.3% |
| ForceSpace | Clarabel | 90.9% | 90.8% |

グラフ: `go2_singularity_degraded.png`。

**動画**(Explicit + ActiveSet、実 Go2 メッシュ——`go2.misa` は
Panda と違い実メッシュを自前で持つため、`render_go2_vtk.py` が
`build_model` の `GeometryObject`(親ジョイント + 実メッシュ経路 +
配置)を直接読み、外部メッシュ探し不要で描画):
[go2_leg_explicit_activeset.mp4](media/go2_leg_explicit_activeset.mp4)
(source of truth: `misa-wbc/examples/media/go2_leg_explicit_activeset.mp4` —
this is a local preview copy under `ref/media/`, since VS Code's
Markdown preview only loads local resources from inside the current
workspace and misa-wbc isn't part of it)

<video src="media/go2_leg_explicit_activeset.mp4" controls width="480"></video>

**発見**:

1. **ActiveSet が全formulationで完全にOptimal**(0.0〜0.3%)——
   misa-wbc の既存推奨(ForceSpace+ActiveSet、§5k)を裏付ける、
   一切のノイズがないクリーンな結果。
2. **misa-wbc自前のIpm実装が、全formulationで一貫して劣化率
   ほぼ100%** — それでいて実際に積分された物理軌道はActiveSetと
   完全に同じ数値(σ_min/err/τ/q̈が一致)。つまり**実質的には正しい
   解に到達していながら、Ipm自身の収束判定(optimality_tol)だけが
   満たせていない** — CBFの`Task::in_range`(上下2本の不等式)が
   作る狭い実行可能領域が、この自前IPM実装の収束基準と相性が悪い
   可能性がある。Panda(§5m)ではIpmはむしろ最も安定した部類
   だったこととの対比が興味深く、**「良いbackend」はタスク構造
   (CBFの有無・密行列の疎密)に依存する**ことを示す一例。
3. **damping(§5nの機能)はここでは決定的な効果なし** —
   Ipmの劣化率を多少下げる場合(AccelSpace: 99.9%→18.2%)もあれば
   ほぼ変わらない場合(Explicit: 99.8%→99.9%)、無関係な構成を
   悪化させる場合(ForceSpace+Admm: 0%→24.3%)もあり、Panda同様
   「単一の既定値で全構成を安全にはできない」という結論を再確認。
   物理軌道自体が既にCBFで守られているため、damping単体の効果が
   Pandaほど劇的に出ない(そもそも改善の余地が小さい)とも言える。

**結論**: Panda(冗長6自由度腕)と Go2脚(非冗長3自由度、CBF併用)は
異なる性質の特異点接近を示した——Pandaは「ソルバーが特異点近傍で
どう壊れるか」、Go2脚は「ハードウェア限界が特異点接近そのものを
防ぎ、残るのはbackendの収束品質の差だけ」という対比。misa-wbcの
既存推奨(ForceSpace+ActiveSet)はどちらのロボットでも最も安定
した選択のままだった。

**限界 / 今後の課題**: 自前Ipm実装がCBF構造下で収束判定だけ
失敗する件は、実害(軌道は正しい)は無いものの単体の未解明現象
として残っており、`optimality_tol`まわりの調査価値がある。

(追記 2026-07-13: 動画化は完了。`build_model` の `GeometryObject`
一覧から実メッシュ経路+配置を読む `render_go2_vtk.py` を新規作成し、
外部メッシュ探し不要で Explicit+ActiveSet を実 Go2 メッシュで
レンダリング——上記の動画リンク参照。)

### 5p. Trot歩容 MPC(SRBD convex MPC)のQP backend比較(2026-07-13)

§5m-5o は misa-wbc の瞬時HoQP(WBC、n<30)の話。今回は逆に、
**quadruped-gait 側に既存の trot 歩容用 convex MPC**(Di Carlo et al.
2018 流、SRBD、`srbd_mpc.rs`、水平線N=10ステップ×12GRF成分=n=120の
凝縮QP)が組み立てる実際の問題を、clarabel 固定ではなく misa-wbc の
4 backend 全部で解いて比較した——WBCよりずっと大きい・より構造化
された(等式60行+不等式120行)QPで、§5l「反復数優位が実時間優位に
転じるにはnがどれだけ必要か」という問いに、合成ランダム行列では
なく実問題で答える形。

**下準備(quadruped-gait 側の最小リファクタ)**: `SrbdMpc::solve` は
QP行列(P, q, 制約)を組み立てた直後に clarabel 専用のCSC/コーン
変換へ直行しており、外から行列を取り出せなかった。そこで
`SrbdMpc::build_qp(...) -> SrbdQpSnapshot`(P, q, 等式/不等式を
積み上げた密行列, n_eq, n_ineq, 状態復元用の a_x/b_u/x_now)を新設
——`solve` 自身もこれを呼んでからclarabel変換するよう書き換え、
数値が一切ズレない(既存175テスト全通過、含むtrot 1周期分の
`cycle_dump` 統合テスト)ことを確認済み。

**代表trotスナップショット**: 対角ペア(FL+RR ⇄ FR+RL)が水平線内で
1回切り替わる、前進0.3 m/s の巡航状態(`SrbdMpcConfig::default()`
そのまま、mass=9kg級)。等式60行(遊脚ゼロ力)+不等式120行
(摩擦錐+垂直力上下限)。

**結果(200回反復、中央値/p99、目的関数は全backend一致 = 正しさ
のクロスチェック済み)**:

| backend | median [µs] | p99 [µs] | 反復数 | 目的関数 |
|---|---|---|---|---|
| Clarabel | **1,438** | 2,124 | 9 | -32.7910 |
| Admm | 2,326 | 3,559 | 36 | -32.7910 |
| ActiveSet | 4,403 | 5,452 | 120 | -32.7910 |
| Ipm(自前) | 10,721 | 14,349 | 13 | -32.7910 |

**§5k/§5m-oまでの WBC 規模(n<30)と完全に順位が逆転**:

```
WBC 規模(n<30、§5k):    ActiveSet > Ipm ≈ IpmMcc > Admm > Clarabel
MPC 規模(n=120、§5p):   Clarabel > Admm > ActiveSet > 自前Ipm
```

**発見**:

1. **Clarabel(商用グレードIPM)がこの規模で最速** — わずか9反復。
   §5l で「n=5120まで調べてもActiveSetを逆転する規模に届かなかった」
   という結論は**合成密行列ランダムQP**の話であり、**実際の構造化
   MPC問題(疎な等式ブロック+規則的な摩擦錐)ではもっと小さいnで
   すでに成熟IPMが優位**になり得ることを示す一例。
2. **ActiveSetはこの規模で反復数=120(≈n)** — WBC規模での強み
   (warm-start、条件付きリッジ)が、毎tick構造が変わる(遊脚/接地の
   切り替え)MPCの冷スタートでは活きにくい。理論的には
   warm-start(前ホライズンの解をシフトして初期値に使う、MPCの
   定石)を実装すれば改善余地があるが、今回は未実装(cold solve
   のみ比較)。
3. **misa-wbc自前のIpm実装は反復数が最少(13)なのに実時間は最も
   遅い(10.7ms)** — §5l の理論(「IPMは反復ごとにO(n³)再分解、
   ActiveSetは増分更新」)がそのまま裏付けられた形: 反復あたりの
   コストが密行列前提の再分解のままで、Clarabelのような疎行列/
   構造活用がない。MPCという「まさにIPMが本領を発揮するはずの
   規模」でも、実装の疎性対応が伴わなければ反復数の優位は実時間に
   結びつかない、という具体例。
4. **Admmは中間的に良好**(2.3ms、36反復)——1回だけ因数分解する
   という設計思想がこの規模でも効いている。

**推奨**: quadruped-gait の trot MPC が既に clarabel 固定なのは、
上記の実測を踏まえても**この規模では正しい選択**。misa-wbcを
このMPCに使う積極的な理由は今のところない(むしろClarabelを直接
使う既存実装の方が速い)——ただし将来 misa-wbc 側にwarm-start MPC
サポート(§5jのIPM warm-startをホライズンシフトに拡張)を追加すれば
ActiveSetが再逆転する可能性はある。

**限界 / 今後の課題**: cold solve のみの比較(quadruped-gaitは実際
30ms周期で再solveするため、tick間warm-startの効果は本来のMPC運用に
とって重要——今回は比較対象外)。1シナリオのみ(全脚立位/遊脚固定
比率が変わると等式/不等式の行数比が変わり、結果が変わる可能性)。
`misa-wbc` はこの比較のため一時的に `[patch]` でローカルsibling
参照にした(Ipm/Admm/D6等、quadruped-gaitが現在ピン止めしている
git rev(`006cb57`)より新しいコミットが必要だったため)——恒久化には
misa-wbcをpushしてquadruped-gaitのCargo.lockのrevを更新するか、
このpatchを維持するかの判断が要る(pushは未実施)。

**動画**: 全backendが同じ解(目的関数一致)に収束するため、backend間で
見た目が変わる比較映像にはならない——その代わり、代表スナップショット
(Clarabelの解)が実際に何を予測しているかを可視化した。実 Go2 の
胴体メッシュ(`base_0..4.obj`、`go2_leg_singularity_demo`が書き出す
mesh manifest 経由)をSRBD予測位置・姿勢に配置(脚は不描画——SRBD
モデル自体が脚運動学を一切持たず剛体+点質量GRFのみのため、胴体のみ
描画するのがこのモデル自身の忠実度そのもの)。初版は汎用の箱で描画
していたが、「trotなのにGo2に見えない」との指摘を受けて実メッシュに
差し替えた:
[trot_mpc_horizon.mp4](media/trot_mpc_horizon.mp4)
(source of truth: `quadruped-gait/quadruped-gait/media/trot_mpc_horizon.mp4` —
local preview copy under `ref/media/`, same reason as the Go2 video above)

<video src="media/trot_mpc_horizon.mp4" controls width="480"></video>

対角ペア(FL+RR ⇄ FR+RL)の切り替えと、水平線終盤(2脚支持のみ)での
予測姿勢の沈み込みが確認できる。

### 5q. Go2実モデル・Trot・WBC・MuJoCo歩行の初検証(2026-07-16)

これまでの WBC+Trot 統合テスト(`tests/wbc_walk.rs`)は軽量な合成
フィクスチャ `namiashi`(体幹1.48kg級)でのみ検証されており、
**実際の `go2.misa`(実機スケール、~15.6kg)で misa-wbc の WBC が
Trot歩容を歩かせられるか**は未検証だった(実機Go2の歩行実績は
`LinearCrawl`+`go2-gait-runner`経由で、WBC/misa-wbc経由ではない)。
`tests/wbc_walk_go2.rs` を新規作成し、`wbc_walk.rs`と同一のharness
(`RobotModel::from_misa`→`GaitController::build(Mpc)`→`WbcPipeline`→
実MuJoCo物理)を `go2.misa` に向けて実行した。

**結果: 一発で成功**(ゲイン等の再チューニング一切不要)。

| test | 結果 |
|---|---|
| 静止立位(重力バランス) | avg Σf_z = 153.10 N = m·g(15.6kg×9.81、**誤差0.0%**) |
| 静止立位(ForceSpace+ActiveSet) | 同上、0.0% |
| 前進歩行(legacy WBC) | Δx = 0.295 m / 2.5 s(指令0.15m/s、期待0.375mの79%)、peak\|roll\|=peak\|pitch\|=0.02 rad |
| 前進歩行(ForceSpace+ActiveSet) | Δx = 0.324 m / 2.5 s(86%)、同程度の水平姿勢 |

体高は終始 z≈0.24m 付近で安定(転倒閾値0.15mを大きく上回る)、
最大傾き角はロール・ピッチとも約1.1°のみ——**ほぼ水平を保ったまま
前進**できている。

**なぜ再調整なしで動いたか**: `articara::gait` 側の既存の自動検出
基盤(`auto_detect_srbd_mpc_config`でロボット質量をリンク総和から
自動算出、`auto_detect_kinematics_config`で脚形状を自動検出、
`DEFAULT_FOOT_LINKS`の`FL_foot`等の命名がgo2.misaの実リンク名と
完全一致)が、そもそもnamiashi専用ではなくモデル非依存に設計されて
いたため。`WbcPipeline::new`内の`build_floating_base_model`も
namiashi/Go2どちらも同じくfixed-base .misaにFreeFlyerを後付けする
汎用処理。実質的な差分は初期姿勢(`examples/go2_crawl.rs`と同じ
hip=0/thigh=0.9/calf=-1.8、初期高さ0.30m)と閾値(Go2の実寸に
合わせて調整)のみ。

**結論**: **misa-wbcのWBC(legacy・ForceSpace+ActiveSetとも)は、
実機スケールのGo2モデルでTrot歩容を問題なく歩かせられる**ことを
初めて確認した。これでこのセッション全体(§5m〜5q)が扱ってきた
「misa-wbcは実際にGo2で使えるのか」という問いに、singularity
(§5o)・MPC性能(§5p)に続き、歩行そのものについても肯定的な答えが
出た。

**限界 / 今後の課題**: 3秒程度の短時間・直進のみの検証(旋回・
より高速な歩行・不整地は未検証)。`wbc_pipeline.rs`のdocに明記
されている既知の近似(floating baseの姿勢を毎tickニュートラルに
保つ)は今回も残ったまま——今回のテストが通ったのは「平地・緩やかな
傾き」の範囲に収まっていたためで、この近似自体を解消したわけではない。

**動画**(実MuJoCo歩行、ForceSpace+ActiveSet): `wbc_walk_go2.rs`に
`WBC_WALK_CSV_OUT`(環境変数、opt-in)を追加し、毎tick MuJoCo が
実際に持つ全ボディ(base+4脚×{hip,thigh,calf,foot})の世界姿勢を
名前で直接クエリして書き出し、`render_go2_walk.py`で実メッシュ
描画(`go2_mesh_manifest.csv`+`go2_topology.csv`をリンク名で
突き合わせ——このtraceはMuJoCoボディ名キーなのでmisarta floating-base
のジョイント添字を意識する必要がない):
[go2_wbc_trot_walk.mp4](media/go2_wbc_trot_walk.mp4)
(source of truth: `articara/tests/media/go2_wbc_trot_walk.mp4`)

<video src="media/go2_wbc_trot_walk.mp4" controls width="560"></video>

体幹が前進しながら対角ペアの脚が交互に持ち上がる(FL_foot/FR_footの
z座標が接地時≈0.01m・遊脚時≈0.04-0.05mで交互に切り替わることを
生CSVで確認済み)——静止画比較では持ち上げ量が小さく分かりにくいが、
動画再生では時間的な変化として確認できる。

**制御ブロック図**: コマンド→歩容位相生成(Trot)+SRBD
MPC(quadruped-gait)→階層QP・WBC(misa-wbc、優先度0=動力学+摩擦錐+
トルク上限、優先度1=MPC由来のa_base_des+遊脚追従、優先度2=接触力
追従+正則化)→ハイブリッド関節指令(Position-PD が q* を追従、WBC
のτはfeedforward)→MuJoCo実物理、というtickごとのデータフローを
mermaidで図示: <https://claude.ai/code/artifact/655df4ff-90d8-426d-8951-c08a2fda65c9>
(フィードバック経路: 観測した体幹速度→SRBD MPCの現在状態推定、
接触力→ContactDrivenPhaseの位相補正)。

### 5r. 速度ステアケース(0→5m/s、0.5m/s刻み、30秒)ストレステスト(2026-07-16)

§5qは0.15m/s一定コマンドのみの検証だった。**指令速度を0.5m/s刻みで
0→5m/sまで段階的に上げていくとどこで破綻するか**を見るため、
`wbc_walk_go2.rs`に`go2_wbc_velocity_staircase`(`#[ignore]`、
回帰テストではなくストレス試験)を追加。30秒を11段階(各≈2.73秒)に
分割し、`WBC_WALK_CSV_OUT`で全ボディの世界姿勢をtickごとに書き出す
既存の仕組みをそのまま再利用。ForceSpace+ActiveSet(misa-wbc)固定。

**結果**: 転倒はしない(体高は全区間0.229〜0.249mで安定、閾値0.15mを
大きく上回る)が、**追従速度が指令1.0〜1.5m/s付近の約0.46m/sで頭打ちに
なり、それ以上指令を上げるほど実測速度が下がっていき、5.0m/s指令では
実測-0.17m/s(後退!)** という「なだらかな飽和→逆行」という破綻の
仕方をした。

| level | cmd_vx | meas_vx | min_z | peak\|roll\| | peak\|pitch\| |
|---|---|---|---|---|---|
| 0 | 0.0 | -0.00 | 0.229 | 0.00 | 0.01 |
| 1 | 0.5 | 0.34 | 0.234 | 0.06 | 0.04 |
| 2 | 1.0 | 0.46 | 0.237 | 0.04 | 0.03 |
| 3 | 1.5 | **0.46(実測ピーク)** | 0.238 | 0.04 | 0.03 |
| 4 | 2.0 | 0.44 | 0.237 | 0.04 | 0.03 |
| 5 | 2.5 | 0.40 | 0.238 | 0.05 | 0.03 |
| 6 | 3.0 | 0.33 | 0.237 | 0.06 | 0.03 |
| 7 | 3.5 | 0.23 | 0.238 | 0.07 | 0.04 |
| 8 | 4.0 | 0.15 | 0.239 | 0.07 | 0.04 |
| 9 | 4.5 | 0.02 | 0.235 | 0.07 | 0.03 |
| 10 | 5.0 | **-0.17(後退)** | 0.233 | 0.06 | 0.03 |

グラフ: `staircase_tracking.png`(追従曲線+体高の2枚組)。

**動画**: [go2_velocity_staircase.mp4](media/go2_velocity_staircase.mp4)
(source of truth: `articara/tests/media/go2_velocity_staircase.mp4`、
画面右上に現在のt・指令速度を表示)。低速域では足跡トレイルが素直に
前へ伸びるが、高速指令域では脚の遊脚軌道が乱れ、トレイルが小さく・
不規則になり後半は後退方向に描かれる様子が確認できる。

<video src="media/go2_velocity_staircase.mp4" controls width="560"></video>

**発見**:

1. **転倒せず、なだらかに飽和・逆行する**——WBCの優先度0(動力学+
   摩擦錐+トルク上限)が常に守られているため、指令を追従しきれない
   状況でも「安全に立ち続ける」方向に落ち着く。これは階層QPが
   まさに設計通りに機能している証拠(優先度0は絶対厳守、優先度1の
   意図達成は「できる範囲で」)。
2. **実測ピーク速度(≈0.46m/s)はGo2の実機能力(トロットで1.5m/s超も
   可能)よりずっと低い**——これはWBC自体の限界ではなく、**この
   テストが使っているTrotの footstep planner・SRBD MPCの水平線
   (30ms×10ステップ)・遊脚高さ等のパラメータが、§5qで検証した
   0.15m/s級の低速にしかチューニングされていない**ためと考えられる
   (§5qのconst値はnamiashi/低速用の閾値をそのまま流用しただけで、
   高速トロット用の再チューニングは今回していない)。
3. **5.0m/s指令での「後退」**は、遊脚のRaibert式着地点計画が過大な
   速度誤差に対して破綻的な着地点を出し続け、正のフィードバックで
   むしろ後方への重心シフトを誘発している可能性が高い(本セッションの
   範囲では根本原因の特定までは未実施)。

**結論**: misa-wbcのWBCは高速指令下でも**安全側(転倒回避)には
頑健**だが、**追従性能そのものはこのTrot設定の低速チューニングに
支配されている**。「Go2でWBCが使えるか」への答えは§5qの「歩ける」
から一歩進み、「安全マージンは広いが、高速化には歩容生成側
(footstep planner/MPC horizon)の再チューニングが必要」という、
より具体的な次の課題が見えた。

**限界 / 今後の課題**: 後退の根本原因未解明。高速トロット用の
パラメータ再チューニングは未着手(スコープ外、要望があれば次の
セッションで)。旋回・不整地との複合はまだ検証していない。

### 5s. 速度ステアケース 再走査(0→1.0m/s、0.05m/s刻み、60秒)+ 全域ズレの原因分析(2026-07-16)

§5rの0→5m/sは目標として過大だった(footstep clampがcmd_vx=0.5m/s
付近で完全に効いてしまう領域まで振っていた)との指摘を受け、
**0→1.0m/sを0.05m/s刻み・60秒**で再走査(`go2_wbc_velocity_staircase_fine`、
21段階・各≈2.86秒、`WbcParams::velocity_staircase_custom_misa_wbc`で
step/max/durationを汎用化)。

**結果**: 実測ピーク**0.341m/s**が、footstep clampの理論しきい値
(計算上ちょうどcmd_vx=0.5m/sで`half`が`0.5×max_step_length_m`に到達)
と**ほぼ完全に一致**。それ以降はなだらかに減衰するのみで(§5rのような
後退は起きない——今回はcmd_vxの上限が穏やかなためcapture-point
フィードバック項が§5rほど暴れない)、体高も全区間0.229〜0.240mで安定。

| level | cmd_vx | meas_vx | min_z |
|---|---|---|---|
| 0 | 0.00 | -0.002 | 0.229 |
| 4 | 0.20 | 0.183 | 0.238 |
| 8 | 0.40 | 0.324 | 0.234 |
| 10 | 0.50 | **0.341(ピーク)** | 0.238 |
| 14 | 0.70 | 0.254 | 0.239 |
| 20 | 1.00 | 0.040 | 0.235 |

グラフ: `staircase_fine_tracking.png`。動画:
[go2_velocity_staircase_fine.mp4](media/go2_velocity_staircase_fine.mp4)

<video src="media/go2_velocity_staircase_fine.mp4" controls width="560"></video>

**追加の疑問と分析: なぜ全区間で目標と実測がズレるのか**(footstep
clampが効き始める0.5m/s以前の低中速域でも、meas_vx/cmd_vxの比率は
80〜91.5%程度に留まり100%にならない)。まず「速度ステップ直後の
加速過渡期を平均に含めているだけでは」という測定手法上の疑いを検証
——各レベルの前半/後半の実測速度を比較したところほぼ同一
(例: level8前半0.321・後半0.327)で、**数百ms以内に定常状態へ収束
しており、過渡応答の影響ではない**ことを確認。したがって本当に定常的な
追従誤差。

比率は cmd_vx≈0.20m/s 付近で最大(≈91.5%)、低速側(0.05m/sで80%)・
高速側(0.5m/sで68%)の両方で悪化する山型パターン。原因として:

1. **制御ループ全体に積分(I)成分が一切ない** —
   `compute_mpc_footstep`のcapture-point補正(`k_capture・v_err`)も
   SRBD MPCの参照生成(`s_now.yaw + wz・t`と同型の「現在値+レート」
   方式)も比例(P)フィードバックのみ(`mpc_controller.rs`/
   `srbd_mpc.rs`に誤差積算項なし、確認済み)。比例のみのループは
   外乱(摩擦・Position-PDの追従遅れ・WBC優先度2の接触力追従が
   ソフト制約であること等)を相殺する分だけ定常偏差を残すのが数学的に
   自然な帰結。0.5m/s付近の悪化加速は、footstep clampが閉ループ項の
   影響で0.5より手前から徐々に効き始めるため。
2. **低速側の相対誤差増大** — 摩擦・PD追従遅れ・WBC力追従誤差による
   絶対的な速度ロスは速度によらずほぼ一定と考えられ、目標速度自体が
   小さい低速域ではこれが相対的に大きな割合を占める。

**結論**: 目標と実測のズレは単一のボトルネック(footstep clamp)だけ
でなく、**フィードバック系全体が積分成分を持たない設計であること**が
全域での定常偏差の根本原因。改善するならcapture-point補正または
MPC速度追従部に積分項を追加する、またはPD/WBCゲインを上げて
外乱に対する感度自体を下げる、のいずれかが素直な対策候補。

**副産物のバグ**: 動画の画面表示(`render_go2_walk.py`)が旧・粗い
ステアケース(0.5m/s刻み)の値をハードコードしたままで、今回の細かい
刻み(0.05m/s)では表示速度が10倍ズレていた(例: 実際0.30m/sを
「3.0」と表示)。ロボットの動き自体(トレースデータ)は影響を
受けておらず、表示テキストのみの不具合——`--staircase-step-mps`/
`--staircase-max-mps`を汎用パラメータ化して修正、再レンダリング済み。

**限界 / 今後の課題**: 積分項追加による改善効果は未検証(提案のみ)。
比率が低速側で悪化する具体的な内訳(摩擦 vs PD遅れ vs WBC力追従誤差の
寄与度分解)は未実施。

### 5t. legged_control比較 + MPCホライズン延長実験(2026-07-16)

§5sの結論(全域で積分成分が一切ない)を受け、`ref/legged_control`
(OCS2ベース)に同じ問題への対策があるか調査。`legged_controllers/`
`legged_interface/` `legged_wbc/`全体を`grep -rln "integral"`した
ところ**ヒット0件**——legged_controlにも積分項は無い。
`target_trajectories_publisher.cpp`の`cmdVelToTargetTrajectories`も
`target(0) = current_pose(0) + cmd_vel_rot(0)*time_to_target`と、
我々の`ReferenceTrajectory::from_constant_velocity`(`s_now.position +
v_world*t`)と同型の「現在値+レート」自己参照方式。**「legged_controlは
積分項を持つ」という前提自体が誤り**だったため、その対策案は不採用。

一方で構造的に明確に異なる点として、`legged_controllers/config/task.info`
の`mpc.timeHorizon 1.0`(1.0秒)が、我々のSRBD MPCデフォルト
(`horizon_steps=10 × dt_per_step=0.03s = 0.3秒`)より3倍以上長いことが
判明。ユーザーの選択(3択中「MPCホライズンを伸ばす(推奨・軽い)」)により、
まずは保守的に**horizon_steps=10 × dt_per_step=0.06s = 0.6秒**
(QPサイズn=120は不変、離散化誤差の増大のみ)を試験。

`go2_wbc_velocity_staircase_fine_long_horizon`(§5sと同一の0→1.0m/s・
0.05m/s刻み・60秒ステアケース、`WbcParams::
velocity_staircase_fine_with_horizon_misa_wbc`で`GaitController::build`
後に`SrbdMpcConfig::{horizon_steps,dt_per_step}`のみを上書き、質量/
慣性の自動検出は保持)で再走査した結果:

| level | cmd_vx | meas_vx (0.3s) | meas_vx (0.6s) | ratio (0.3s) | ratio (0.6s) |
|---|---|---|---|---|---|
| 0  | 0.00 | -0.002 | -0.002 | - | - |
| 4  | 0.20 | 0.183  | 0.165  | 91.5% | 82.5% |
| 6  | 0.30 | 0.266  | 0.319  | 88.7% | 106.3% |
| 8  | 0.40 | 0.324  | 0.435  | 81.0% | 108.8% |
| 10 | 0.50 | **0.341(旧peak)** | 0.498 | 68.2% | 99.6% |
| 14 | 0.70 | 0.254  | 0.506  | 36.3% | 72.3% |
| 20 | 1.00 | 0.040  | 0.468  | 4.0%  | 46.8% |

グラフ: `horizon_comparison_tracking.png`。

**結果**: 明確な改善。
1. **cmd_vx 0.25〜0.5m/s帯でほぼ100%追従**(0.3sでは68〜92%止まり
   だった帯域で、0.6sでは97〜110%——一部オーバーシュートするほど)。
2. **cmd_vx 0.5m/s以降、§5r/5sで見られた「反転して0へ収束」する
   破綻モードが消滅**。実測速度は0.47〜0.51m/s付近で頭打ちになる
   だけで、高cmd域でも崩れずプラトーを維持(footstep clampは
   引き続き効いているが、反転はしない)。
3. **低速域(cmd_vx≦0.2m/s)はむしろ僅かに悪化**(level1で実測が
   負に振れる、level4で82.5% vs 91.5%)——ホライズン延長は低速の
   相対誤差問題には効かず、むしろ離散化誤差(dt_per_step倍増)の
   影響がノイズとして僅かに乗ったと考えられる。低速側の問題は
   §5sで挙げたもう一つの原因(摩擦・PD遅れ等の絶対誤差が相対的に
   大きく見える)のままで、ホライズンでは解決しない。
4. 体高は全区間0.229〜0.253mで安定、転倒なし。

**結論(0.6s時点)**: MPCホライズン延長(0.3s→0.6s)は、中速域の定常
追従誤差と高速域の反転破綻という§5r/5sで見つかった2つの問題の**片方
(反転破綻)を完全に解消し、もう片方(定常誤差)を中速域で大幅改善**
した。低速域の相対誤差は未解決で、これは(§5sの分析どおり)積分項
追加や外乱源そのものの低減が必要。QPサイズは変えていないため
計算コストの増加はほぼ無い(離散化ステップが粗くなっただけ)。

**続き: 1.0秒(legged_control相当)まで伸ばすと悪化する**——
0.6sの結果を受け、ユーザーの選択により
`go2_wbc_velocity_staircase_fine_full_horizon`
(`horizon_steps=10 × dt_per_step=0.10s = 1.0秒`、legged_controlの
`mpc.timeHorizon 1.0`と厳密一致)を追加試験したところ、**単調に
改善するどころか明確に悪化**した:

| level | cmd_vx | meas_vx (0.3s) | meas_vx (0.6s) | meas_vx (1.0s) |
|---|---|---|---|---|
| 4  | 0.20 | 0.183  | 0.165  | 0.150 |
| 6  | 0.30 | 0.266  | 0.319  | 0.140 |
| 8  | 0.40 | 0.324  | 0.435  | **-0.063** |
| 10 | 0.50 | 0.341  | 0.498  | **-0.196** |
| 14 | 0.70 | 0.254  | 0.506  | **-0.320** |
| 20 | 1.00 | 0.040  | 0.468  | **-0.384** |

グラフ(3本比較): `horizon_comparison_tracking.png`。

cmd_vx≈0.3m/s以降、実測速度が単に頭打ちになるのではなく**符号が
反転し、指令が大きいほど後退が速くなる**(cmd=1.0m/sで実測
-0.384m/s——目標と真逆に、しかもそれなりの速さで後退歩行する)。
体高は全区間0.229〜0.250mを維持し転倒はしない(姿勢角のみ
roll≈0.12rad・pitch≈0.08radまで緩やかに増加)ため、これは
「不安定化して崩れる」破綻ではなく、**閉ループとして間違った
方向に収束する**という質的に別の失敗モード。

QPの`did not reach optimal`警告回数を3条件で比較したところ
0.3s=68239件 → 0.6s=27092件 → 1.0s=21721件で、**ホライズンが
長いほど警告は単調に減っている**——つまりQP自体は1.0sの方が
"素直に解けている"のに track性能は最悪という逆転が起きており、
QPソルバの収束性の問題ではないと確認できた。原因として最有力
なのは、SRBD MPCの前進オイラー離散化誤差(1ステップ=100ms)が
高速時ほど大きくなり、10ステップ分(計1.0秒)蓄積した予測状態が
実際の姿勢/速度から大きく乖離すること。参照生成自体が
「現在値+レート×t」の自己参照方式(§5s)であるため、この乖離した
予測に対してMPCが導出するGRF計画とRaibert足配置计画が現実と
逆向きの補正を繰り返し、指令が大きいほど誤差フィードバックが
強く効いて後退方向に発散したと考えられる(要検証)。legged_control
(OCS2完全非線形MPC、SQPで毎ステップ再線形化)がこの問題を
起こさないのは、離散化・線形化の扱いが根本的に異なるためで、
**「ホライズンだけ真似ても同じ恩恵は得られない」**ことを示す。

**結論(更新)**: ホライズン長は単調パラメータではなく、
0.3s(短すぎ、反転はしないが飽和・定常誤差が残る)と1.0s
(長すぎ、離散化誤差が蓄積し逆方向に発散)の間に**0.6s前後の
スイートスポット**が存在する。legged_controlの1.0sをそのまま
移植するのは(このSRBD+前進オイラー定式化では)悪手——真似るべき
なのはホライズン長という数値ではなく、OCS2側の非線形・逐次
再線形化という解法の質だった可能性が高い。

**続き: 0.6s〜1.0s間を細かく走査 → 0.6sは"広い最適域"ではなく
"狭いスパイク"だった**——ユーザーの指示により
`go2_wbc_velocity_staircase_fine_horizon_sweep`(dt_per_step=0.07/
0.08/0.09、horizon=0.70/0.80/0.90秒)を追加試験。cmd_vx=0.5m/s時点
の実測値(level10)を横軸=ホライズン長でプロットすると
(`horizon_sweet_spot.png`):

| horizon | 0.3s | 0.6s | 0.7s | 0.8s | 0.9s | 1.0s |
|---|---|---|---|---|---|---|
| meas_vx@cmd0.5 | 0.341 | **0.498** | -0.103 | -0.086 | -0.037 | -0.196 |

**0.6sから0.7sへのわずか0.1秒の変化で崖のように反転**(+0.498→
-0.103)し、0.8s/0.9s/1.0sは軒並み負(ただし0.9sが-0.037とやや
浅いなど、単調ではなく多少ギザギザ)。つまり0.6sは前後に緩やかな
"最適の谷"を持つ安定したパラメータ域ではなく、**孤立したスパイク**
である可能性が高い——0.6s±0.05s程度の再現性・幅を未確認のまま
デフォルト採用するのはリスクがある。

可能性のある説明(未検証・推測): Trotの`cycle_period_s=0.4s`に
対し0.6s=1.5周期という特定の位相関係でのみ、MPC予測軌道と実際の
接触スケジュールがたまたま整合し、0.7s(1.75周期)以降は整合が
崩れる、という歩容周期とMPCホライズンの共鳴/エイリアシングが
候補だが、0.8s(ちょうど2.0周期)も崩壊側であるため単純な整数/
半整数周期則では説明しきれず、確度は低い。

**結論(再更新)**: MPCホライズンは「長くするほど良い」でも
「0.3sと1.0sの間に緩やかな最適点がある」でもなく、**この
SRBD+前進オイラー定式化では0.6s付近にのみ narrow な好条件が
存在し、その両側(0.7s以降はもちろん、0.6s未満の中間値も未確認)
で容易に崩れる**。0.6sを新デフォルトとして採用するのは、この
スパイクの再現性(±0.02〜0.05s刻みでの追加走査、乱数シード相当
の初期条件依存性)を確認してからにすべきで、現時点でのコミットは
時期尚早と判断。

**続き: 0.6s近傍を細かく走査 → 孤立点ではなく幅≈0.09秒の狭い帯**
——`go2_wbc_velocity_staircase_fine_horizon_zoom`
(dt_per_step=0.055/0.058/0.062/0.065、horizon=0.55/0.58/0.62/0.65秒)
を追加試験。cmd_vx=0.5m/s時点の実測値(level10)を全10点まとめて
プロットすると(`horizon_sweet_spot.png`):

| horizon(s) | 0.30 | 0.55 | 0.58 | 0.60 | 0.62 | 0.65 | 0.70 | 0.80 | 0.90 | 1.00 |
|---|---|---|---|---|---|---|---|---|---|---|
| meas_vx@cmd0.5 | 0.341 | 0.003 | -0.053 | 0.498 | **0.521** | 0.473 | -0.103 | -0.086 | -0.037 | -0.196 |

**0.60s単独のスパイクではなく、0.60〜0.65秒に幅を持つ好条件帯**
(0.62sはさらに0.521とわずかに0.60sを上回る)。ただしこの帯は
非常に狭く、下側は0.58s(-0.053、既に崩壊側)との間、上側は
0.70s(-0.103)との間で、それぞれ0.02〜0.05秒の範囲で急峻に
切り替わる崖になっている——「0.6秒から0.65秒の間ならどこでも
安全」と言えるほど広くはなく、依然として際どいチューニングが
要求される。

**結論(再々更新)**: MPCホライズン長と追従性能の関係は
0.3s→1.0sの間で単調でも、単一の孤立点でもなく、**horizon≈0.60〜
0.65秒(10step×dt=0.060〜0.065s)という幅0.05〜0.09秒程度の狭い
帯でのみ良好**、帯の外側(0.55s以下、0.70s以上)は速やかに崩壊
(反転・逆走)する。この帯の物理的意味(歩容周期0.4sとの位相関係、
footstep clampとの相互作用など)は未解明。デフォルト採用するなら
帯の中央寄り(0.62s付近)を選ぶのがやや安全だが、この狭さ自体が
「ホライズン延長」という対策の頑健性に疑問を投げかける——本番
運用前に、初期条件・地形外乱に対するこの帯の再現性を別途確認
すべき。

**続き: 共鳴仮説を検証 → 単純な比例関係では説明できない**——
`go2_wbc_velocity_staircase_fine_cycle_resonance`で
`cycle_period_s`(デフォルト0.4s)を0.3s/0.5sに変え、各々で
「絶対値0.6s」と「比例値(1.5×cycle_period_s)」の2つのホライズンを
試験:

| cycle_period_s | horizon=絶対0.6s | horizon=比例1.5× |
|---|---|---|
| 0.3s | まずまず(cmd0.5でpeak 0.333、反転なし) | **悪化**(cmd0.25以降ほぼ負、-0.2付近) |
| 0.5s | 良好(cmd0.4以降0.37〜0.42で安定プラトー、反転なし) | 良好(同程度のプラトー、反転なし) |

**予測がどちらも外れた**: 「1.5周期という比率が常に良い」なら
0.3s周期での比例値(0.45s)が最良のはずが、実際は絶対値0.6sより
明確に悪化(反転)。「絶対時間0.6sが常に良い」なら0.3s周期でも
0.5s周期でも同程度に良いはずが、0.3s周期では両方とも§5s基準の
ような明確な好走行(近100%追従)には届いていない(0.3s周期は
horizonによらずやや不安定、0.5s周期はhorizonによらず両方とも
反転なしの穏やかなプラトーで、§5t序盤の0.4s周期×0.6sほど鋭くは
ないが安定)。

**結論**: 「horizon/cycle_period_s の比」という単純な共鳴仮説は
**否定**された。観測された傾向としては、**cycle_period_sを
長くする方向(0.4s→0.5s)がhorizon長によらず反転を防ぎ安定化に
寄与している**ように見え、逆に短くする方向(0.4s→0.3s)は不安定化
方向に働く——つまり原因は「horizonとcycle_period_sの比」ではなく、
「cycle_period_s(あるいはそれが決めるstance_duration=
cycle_period_s×duty_factor)自体がfootstep clamp・capture-point
フィードバックの安定性を左右している」可能性の方が高い。ただし
これも2点×2条件の小規模な探索であり、確証には至っていない。

**今後の課題**: (a) `cycle_period_s`単体(horizon固定)での走査を
別途行い、安定化の主要因が本当にcycle_period_s(またはstance_
duration)なのかを直接確認、(b) 0.4s周期×0.60〜0.65sという最初に
見つかった狭い好条件帯の物理的原因は依然未解明のまま、(c) 低速域
(≦0.2m/s)の相対誤差はホライズン・周期のどちらの実験でも未解決で、
明示的な積分項追加は本実験系列と独立に別途検討が必要、(d) デフォルト
値を変更するなら、少なくとも本セクションで見た「狭い帯」「周期依存の
逆転」という2つの脆さを踏まえ、単純な0.6s採用ではなくより広い
探索(cycle_period_s×horizonの2次元グリッド)を経てからにすべき。

### 5u. スタンス高さ(standing height)スイープ(2026-07-16)

ホライズン系列とは独立に、ユーザーの提案で「ボディ高さを振ってその
影響を見る」実験を実施。`mpc_controller.rs::build_srbd_inputs`は
`-kin.legs()[0].nominal_foot_body.z`をMPCの目標体高のproxyとして
使っているため(コード確認済み)、`run_wbc_sim`が全4脚に一様適用
している`nominal_foot_body.z += bias_frac * (upper_leg_m +
lower_leg_m)`の`bias_frac`(既存コードはハードコード`0.08`)を
`WbcParams::velocity_staircase_fine_with_height_misa_wbc`経由で
上書き可能にし、horizon・cycle_period_sはデフォルト(0.3s / 0.4s)
のまま、同一の0→1.0m/s・0.05m/s刻み・60秒ステアケースで
`bias_frac ∈ {-0.08, -0.04, 0.00, 0.08(既存デフォルト),
0.16, 0.24, 0.32}`を走査(`go2_wbc_velocity_staircase_fine_body_height_sweep`)。
Go2の脚全長(upper_leg_m+lower_leg_m)は0.426m、対応する実際の
スタンス高さ(`-nominal_foot_body.z`)は以下:

| bias_frac | -0.08 | 0.00 | 0.08(既存) | 0.16 | 0.24 | 0.32 |
|---|---|---|---|---|---|---|
| 高さ(m) | 0.300 | 0.266 | 0.232 | 0.198 | 0.164 | 0.130 |

cmd_vx=0.5m/sとcmd_vx=1.0m/s時点の実測値、およびその比("retention"
=高速域でどれだけ減衰せず残るか)を抜粋:

| 高さ(m) | meas@0.5 | meas@1.0 | retention |
|---|---|---|---|
| 0.300 | 0.404 | 0.158 | 39.1% |
| 0.266 | 0.347 | 0.073 | 21.0% |
| 0.232(既存) | 0.341 | 0.040 | 11.7% |
| **0.198** | 0.347 | **0.325** | **93.7%** |
| 0.164 | 0.417 | 0.104 | 24.9% |
| 0.130 | 0.450 | **-0.070** | -15.6%(反転) |

グラフ: `body_height_sweep.png`。

**結果**: 低くする(crouchする)ほどcmd_vx=0.5付近までの追従性は
単調に改善する傾向(0.232m→0.164m→0.130mでmeas@0.5が0.341→
0.417→0.450と上昇)が、**h=0.198m(bias_frac=0.16)だけが質的に
異なる挙動**を示した——他の全ての高さはcmd_vx=0.5付近でピークを
迎えた後なだらかに(あるいはh=0.130mでは反転して)減衰するのに対し、
h=0.198mだけはcmd_vx=0.5〜1.0の全域でmeas_vxが0.32〜0.35のまま
ほぼ一定のプラトーを保つ(retention 93.7%——他は12〜39%、
h=0.130mは反転してマイナス)。ホライズン延長実験(§5t)で0.6〜0.65秒
のときに見られた「反転せずプラトーする」挙動と**質的に同じ現象が、
MPCホライズンはデフォルト0.3sのまま、スタンス高さだけを変える
ことでも再現された**——2つの独立したパラメータ変更が同じ質的な
安定化効果を持つことは、これが偶然ではなく、footstep clamp・
capture-pointフィードバックループの安定性に関わる何らかの共通の
力学的要因(有効慣性・脚のヤコビアン感度・重心高さと転倒モーメントの
関係など)が存在することを示唆する。

`min_z`は各高さの目標スタンス高さに対して常に妥当な範囲(バタつき
±0.01〜0.03m程度、既存の§5s/5tと同オーダー)に収まっており、
`TRUNK_Z_FALL_THRESHOLD_M=0.15`という固定しきい値は今回目標高さ
自体を0.130〜0.164mまで下げているため転倒判定としてそのまま
使えない(例: h=0.130mでmin_z=0.123mは「正しく低いスタンスを
保っている」ことを意味し、転倒ではない)——高さごとの目標値との
相対比較が必要。roll/pitchのピーク値は低いスタンスほど僅かに増加
(h=0.300mで≈0.04、h=0.130mで≈0.08)しており、脚のリーチが
浅くなるほど姿勢補正の余裕が減る傾向はうかがえるが、いずれも
転倒に至るレベルではない。

**結論**: (a) スタンス高さは速度追従性に対しホライズン長と同等かそれ
以上に強い影響を持つ独立したレバーである、(b) h≈0.20m付近に
(§5tのホライズンと同様の)"反転せず高速域までプラトーする"狭い
好条件点が存在し、これも単調パラメータではない可能性が高い、
(c) 最も興味深い未検証の仮説は、**高さとホライズンを両方
"好条件"側に合わせた場合(例: h=0.20m×horizon=0.6s)に効果が
相加/相乗するか、それとも同じ根本要因への異なる現れ方に過ぎず
重複するだけか**。

**今後の課題**: (a) h=0.198m前後をさらに細かく走査し(§5tのホライズン
zoomと同様)、これも狭いスパイクか幅を持つ帯かを確認、(b)
h=0.20m×horizon=0.6sの組み合わせ試験、(c) より高いスタンス
(bias_frac<-0.08、h>0.30m、脚の伸びきりに近い側)は未走査——
§5oで確認済みの特異点近傍不安定性がどの高さから出現するかは別途
確認が必要、(d) 低速域(≦0.2m/s)の相対誤差はこの実験でも解決
しておらず引き続き別課題。

### 5v. 高さ×ホライズン組み合わせ実験(2026-07-16)

§5uの「今後の課題(b)」を受け、h=0.20m(bias_frac=0.16、horizon
デフォルト0.3s)とhorizon=0.6s(h デフォルト0.23m)という独立に
見つかった2つの"反転せずプラトーする"好条件を**組み合わせた場合に
効果が相加されるか**を検証(`go2_wbc_velocity_staircase_fine_horizon_and_height_combo`、
同一の0→1.0m/s・0.05m/s刻み・60秒ステアケースを3条件で比較)。

| cmd_vx | height only(h=0.20m) | horizon only(0.6s) | 組み合わせ |
|---|---|---|---|
| 0.40 | 0.326 | 0.435 | 0.276 |
| 0.50 | 0.347 | 0.498 | **0.146** |
| 0.60 | 0.350 | 0.498 | **0.036** |
| 0.65 | 0.353 | 0.511 | **0.035** |
| 0.80 | 0.349 | 0.493 | 0.084 |
| 1.00 | 0.325 | 0.468 | 0.283 |

グラフ: `height_horizon_combo.png`。

**結果**: **相加されるどころか、組み合わせは単体のどちらよりも
明確に悪化**した。cmd_vx=0.40付近までは単体2条件とほぼ同じ経路を
辿るが、そこから単体なら伸び続ける/プラトーする領域で**急落**
(cmd=0.60〜0.65でmeas_vxが0.035〜0.036まで落ち込み、単体の
10分の1程度)、その後cmd=1.0に向けてなだらかに回復(0.283まで)
という、単体のどちらとも異なる**谷型**の応答を示した。

**結論**: h≈0.20mとhorizon≈0.6sは「同じ根本メカニズムの異なる
現れ方」として単純に加算・重複するものではなく、**組み合わせると
悪い方向に相互作用する**——それぞれ単体では効いていた安定化効果が
組み合わせることで打ち消し合う、あるいは新しい不安定モードを
誘発するとみられる。これは、両パラメータが同じ1つの変数
(例えば実効ホライズン距離や脚の有効慣性)を別々の経路で動かして
いるのではなく、**複数の異なる力学的経路が絡み合っている**ことを
示唆し、§5uで立てた「共通要因」仮説を弱める材料でもある——少なくとも
単純な線形重ね合わせでは説明できない。

**結論(実務上)**: 現時点でどちらか一方を採用するなら単体で使うべきで、
「両方良かったから両方採用」という判断は明確に誤り。horizon=0.6sの
方が単体としての追従性(meas@1.0=0.468、100%近い追従帯を含む)は
height=0.20mの単体(meas@1.0=0.325)より優れているが、§5tで指摘した
通りhorizon=0.6sは狭い帯で脆弱——一方height=0.20m単体がどの程度
頑健か(§5uの「今後の課題(a)」、まだ未検証)は依然不明。

**今後の課題**: (a) height=0.20m単体の頑健性(狭いスパイクか帯か)を
§5tのホライズンzoomと同様に検証、(b) 組み合わせ実験で見られた
「谷型」応答の原因特定(cmd=0.4〜0.65の間で何が急激に変化するのかを
per-tickトレースで確認)、(c) この2パラメータ以外の第3の独立変数
(例: capture-pointゲイン、footstep clampの`max_step_length_m`)も
同様の"反転せずプラトーする"効果を持つか、あるいは同様に組み合わせ
ると悪化するかを確認し、一般則を探る。

### 5w. FullCentroidal(接触力・体幹・脚の同時最適化)初評価(2026-07-16)

§5t〜5vのホライズン・高さ実験を受け、ユーザーから「legged_control同様に
接触力・体幹軌道・遊脚軌道をMPCで同時最適化すればパラメータ数が減り
改善するのでは」という提案。実装前にコードベースを調査したところ、
**この方向性は既に`quadruped_gait::GaitMode::FullCentroidal`として
実装済み**であることが判明(調査で発見、新規実装は不要だった):

- `FullCentroidalMpc`は24状態(SRBDの13状態+脚関節角`joint_q`12個)
  を持ち、脚のモーメントアーム`r = R·(foot_body(q) − com_offset)`が
  ホライズン内で`joint_q`の変化とともに更新される——SRBDでは固定
  パラメータだった`r`がここでは状態の一部。
- スタンス脚のstance no-slip拘束(`v_foot_world = 0`)を等式制約として
  持つ(SRBDには無い)。
- `FullCentroidalMpc::solve`は単発凸QPではなく**複数回再線形化する
  SQPループ**(`full_centroidal_mpc.rs:849-898`、warm-start付き)。
- オプトインの`legged_control_parity`モードがOCS2の
  `centroidalModelType=0`設定に合わせて(a)各脚位相ベースの接触
  スケジュール、(b)遊脚の鉛直速度追従制約、を追加。
- さらに`use_mpc_predicted_footstep`で、capture-pointフィードバックの
  代わりに**MPCが予測した体幹の回復軌道から着地補正量を導出**する
  (脚がswing中でも、MPCの体幹予測とfootstep目標を結合する仕組み——
  ユーザーの提案に最も近い既存機能)。

**なお`Δr`(footstep位置をSRBD側の決定変数にする案)は過去に実装・
ベンチ・撤回済み**(コミット`45345cd`、`doc/recent_features.md`§10)
——理由は、SRBDの`r×F`項がswing脚(F=0)では勾配を持たず「swing中の
脚の着地点を動かす」ことをそもそも表現できない構造的な問題で、単純な
線形化改善では解決しない。この過去の失敗と、その回避策として
`FullCentroidalMpc`(joint_qを状態に含めることでこの問題を構造的に
回避)が既に存在することの両方が、今回のコード調査で判明した。

**実験**: 3構成(FullCentroidal既定=parity off、+parity、+parity+
predicted_footstep)を、既存のSRBD基準(§5sデフォルト、および§5tの
0.6sホライズン)と同一の0→1.0m/s・0.05m/s刻み・60秒ステアケースで
比較。FullCentroidal側は高さ・ホライズンとも**自動検出デフォルトの
まま、一切チューニングしていない**。

| cmd_vx | SRBD既定 | SRBD 0.6s horizon | FC legacy | **FC+parity** | FC+parity+predicted |
|---|---|---|---|---|---|
| 0.40 | 0.324 | 0.435 | 0.160 | 0.285 | 0.133 |
| 0.50 | 0.341 | 0.498 | 0.129 | **0.401** | -0.059 |
| 0.65 | 0.293 | 0.511 | 0.201 | **0.466** | -0.064 |
| 0.80 | 0.165 | 0.493 | 0.351 | **0.450** | -0.190 |
| 1.00 | 0.040 | 0.468 | 0.298 | **0.471** | -0.096 |

グラフ: `full_centroidal_comparison.png`。

**結果**:
1. **FullCentroidal legacy(parity off)**: cmd=0.2〜0.6付近で
   0.13〜0.17に留まりノイズっぽく変動、cmd=0.65以降不可解に
   0.20→0.36まで上昇してから再び0.30付近に落ちるという非単調な
   応答——SRBD基準ともFC+parityとも異なる、あまり有用でない挙動。
2. **FullCentroidal + legged_control_parity**: **チューニング一切
   無しで、§5tが0.6sホライズンという細い好条件帯を苦労して見つけて
   得た結果とほぼ同等のプラトー**(cmd=0.5以降0.44〜0.49で安定、
   反転なし)を達成。roll/pitchも0.04〜0.06程度に収まり安定。
3. **+ use_mpc_predicted_footstep**: 逆に**明確に悪化・不安定化**
   ——cmd=0.45以降ほぼ全域で符号が反転し後退、roll最大0.26rad
   (≈15°)まで悪化する区間もある。閉ループ化(footstep目標をMPC予測
   に委ねる)は少なくとも現在のデフォルトゲインでは裏目に出た。

**結論**: ユーザーの仮説は**部分的に的中**——`legged_control_parity`
(接触スケジュールを各脚位相ベースにし、遊脚鉛直速度をMPCの制約として
明示的に扱う)だけで、SRBD側が§5t/5uで高さ・ホライズンを個別に
探索してようやく得たのと同等の"反転なしプラトー"が、**追加パラメータ
探索なしに**得られた。一方「MPC予測とfootstepを結合してさらに閉ループ
化する」(`use_mpc_predicted_footstep`)は今回悪化を招いており、
「同時最適化を徹底するほど良くなる」という単純な話でもない——SRBDの
高さ×ホライズン組み合わせ(§5v)同様、ここでも「拡張のどこまでを
有効にするか」自体が新しいチューニング対象になっている。

**今後の課題**: (a) `use_mpc_predicted_footstep`単体が悪化する原因の
特定(cmd=0.45以降で何が壊れるかper-tickトレースで確認)、(b)
FullCentroidal+parity自体の頑健性(§5tのホライズンzoom相当——高さや
ホライズンを振っても同様に狭い/広い好条件帯を持つか)は未検証、
(c) FullCentroidal自体の計算コストはSRBDよりかなり重い(同一の
60秒ステアケースの実行に要した壁時計時間で体感、複数回再線形化する
SQPループのため)——実機/リアルタイム制約下での実用性は要検討、
(d) 低速域(≦0.2m/s)の相対誤差はFullCentroidalでも未解決(むしろ
+parityでcmd=0.05〜0.15はSRBD既定よりやや悪化気味)で、依然別課題。

### 5x. joint_q参照の動的化(D3.3.5aの解除)実装 + 初評価(2026-07-16)

ユーザーから「legged_control同様に接触力・体幹軌道・遊脚軌道をMPCで
同時最適化まで拡張できるか」と再度の要望。実装前にコード調査したところ、
`FullCentroidalMpcGaitController`は既にjoint_qを状態に含み(§5w)、MPC側の
コスト(`q_diag[12..24]`、per-node `x_ref`スタッキング)は**既に時変
joint_q参照を汎用的にサポート済み**(コード変更不要)だが、
`build_full_centroidal_inputs`が実際に渡す参照は**完全な定数保持**
(全ホライズンステップで同一の`joint_q`、legacy/parity両方とも)である
ことが判明——`doc/recent_features.md`§10が既に「D3.3.5aの簡略化解除」
として今後の課題に挙げていた項目そのもの。

**実装**(quadruped-gaitへの変更、コミット対象):
- `FullCentroidalMpcGaitController`にオプトインの`dynamic_joint_q_reference`
  フラグを追加(`legged_control_parity`必須——legacyパスはstep0以降の
  per-leg位相情報を持たないため無意味)。
- `build_full_centroidal_inputs`の各ホライズンステップ`k`で、各脚の
  `contact.is_stance[leg][k]`と(新規追加した)`phase_sub_fractions[leg][k]`
  を使い、`tick()`が現在ステップに対して既に使っているのと**全く同じ**
  `Footstep::stance_at`/`swing_position` + `solve_leg_ik`のパターンを
  ホライズン全体に適用。footstepそのものはopen-loop(Raibert+cap-pt、
  MPCの現在進行中の解には依存しない、A1と同じ循環回避パターン)。
- MPC本体・コスト重みは無変更——変更は`build_full_centroidal_inputs`
  内のみ。quadruped-gait全175ユニットテスト・articara側の4回帰テスト
  とも無変化(グリーン)。

**実験**: §5wの`FullCentroidal + legged_control_parity`基準に対して
`dynamic_joint_q_reference`のON/OFFを比較(同一の0→1.0m/s・0.05m/s刻み・
60秒ステアケース)。

| cmd_vx | parity基準(§5w) | + dynamic_joint_q_reference |
|---|---|---|
| 0.40 | 0.285 | 0.253 |
| 0.50 | 0.401 | 0.380 |
| 0.65 | 0.466 | 0.457 |
| 0.80 | 0.450 | 0.444 |
| 1.00 | 0.471 | 0.466 |

グラフ: `dynamic_joint_q_comparison.png`——2本の曲線はほぼ完全に重なる。

**結果**: **有意な変化なし**(差は全域で±0.01〜0.03程度、乱数的
ノイズの範囲内——§5wの`use_mpc_predicted_footstep`のような明確な悪化も、
期待していたような改善も、どちらも観測されなかった)。原因は調査時点で
既に指摘されていた通り: `FullCentroidalMpcConfig::q_diag[12..24]`
(joint_q追跡コストの重み)がデフォルト`0.1`と意図的に軽く設定されて
おり(stance no-slip拘束と競合しないため)、参照を定数から動的に
変えても、コストがその差にほとんど反応しない。**「配線は正しく繋がった
が、重みが効かせるほど強くない」**状態。

**結論**: ユーザーの要望した「同時最適化の完成形」に向けた配線自体は
実装・動作確認済みだが、デフォルト設定では体感できる効果に至らず。
これは§5vの「組み合わせ悪化」や§5wの「閉ループ化で悪化」とは異なる
第三のパターン——**拡張が正しく実装されていても、既存の重みバランスが
それを無効化してしまう**——であり、「同時最適化を徹底すればするほど
良くなる」という単純化はここでも成立しない。効果を確認するには
`q_diag[12..24]`自体を引き上げる実験が必要だが、そうすると
stance no-slip拘束との競合(強すぎる joint_q追従が拘束を破る)という
新たな不安定化リスクを招く可能性が高く、次の実験対象として独立に
評価する必要がある。

**今後の課題**: (a) `q_diag[12..24]`を段階的に引き上げてeffectが
現れる閾値と、拘束競合による不安定化が始まる閾値の両方を特定する、
(b) stance/swingで異なる重み(現状は12エントリ一律)に分離し、swing脚
だけ追従を強めてstance脚の拘束は妨げない設計にする、(c) この実験系列
(§5t〜5x)全体を通じて低速域(≦0.2m/s)の相対誤差は一度も解決して
おらず、依然として別課題として残る。

### 5y. OCS2本体調査(①③) + SQP反復回数チューニング実験(2026-07-16)

ユーザーの許可を得て`leggedrobotics/ocs2`本体を`ref/ocs2`にクローンし、
①(遠心運動量結合)③(SQP反復回数×頻度)を実ソースで確認。

**①確定(重要な補足あり)**: 運動量**変化率**の方程式自体はOCS2でも
純粋なNewton-Euler(重力+接触力のみ、`joint_v`は登場しない)で我々と
同型。真の結合は一段下、「運動量→一般化速度」への変換
(`CentroidalModelPinocchioMapping::getPinocchioJointVelocity`、
`momentum -= A.rightCols(nj)*jointVelocity`)にあり、これは
**`centroidalModelType=FullCentroidalDynamics`のときのみ有効**。
legged_control(chvmpフォーク)はこちらを使うが、**OCS2本体自身の
公式`ocs2_legged_robot`サンプルはデフォルトで`SingleRigidBodyDynamics`
を使っており、この場合は結合項が構造的にゼロ**——つまり我々のSRBDと
同じ。「OCS2は結合している」と一括りにはできなかった。

**③確定**: OCS2は状態を各ショットノードで決定変数に持つ真のmultiple
shootingをHPIPM(疎なRiccati再帰内点法)で解いており、我々の
「毎回denseに凝縮した単一QP」とは構造が異なる。`sqpIteration=1`@
100Hz(legged_control)/50Hz(OCS2公式サンプル)という値は
`ocs2_legged_robot`固有のRTI的チューニングで、OCS2全体の推奨値では
ない(`ocs2_ballbot`は`sqpIteration=5`)。

**実験(③)**: `sqp_iterations`を振って比較。**重大な発見**——当初
「§5w/5xの基準」だと思っていた`horizon_steps=20, dt_per_step=0.030,
sqp_iterations=3`(`FullCentroidalMpcConfig::default_with_kin`の
リテラルデフォルト)は誤りだった。実際は`auto_detect_full_centroidal_
mpc_config`(`gait.rs`)がこの3フィールドを12状態版`auto_detect_
centroidal_mpc_config`(実体`CentroidalMpcConfig::default()`)の値
——**`horizon_steps=10, dt_per_step=0.030, sqp_iterations=1`
(ホライズン0.3秒、反復1回)**——で上書きしており、§5w/5xで実際に
走っていたのはこちらだった。この誤りは実験を再実行して発見・訂正
(以下は訂正後の正しい比較)。

| cmd_vx | h=0.3s sqp=1(真の既定) | h=0.3s sqp=3 | h=0.3s sqp=1・20×0.015s(legged_control dt) |
|---|---|---|---|
| 0.50 | 0.331 | **0.401** | 0.358 |
| 0.65 | 0.393 | **0.466** | 0.345 |
| 1.00 | 0.407 | **0.471** | 0.313 |

グラフ(2パネル): `sqp_tuning_comparison.png`。

**結果**: **ホライズン0.3秒では反復を増やす(1→3)方が明確に良い**
(cmd=0.5でmeas 0.331→0.401、比率66%→80%)。ところが、当初の
誤ったホライズン0.6秒(20ステップ)での比較では**逆に反復を増やす
(1→3)と持続的な反転(後退歩行、cmd=1.0でmeas=-0.356)を引き起こし、
減らす(3→1)方が安定したプラトーになる**——§5vの「高さ×ホライズン
組み合わせ」と同様、**同じ操作(SQP反復を増やす)の効果がホライズン
長によって符号すら逆転する**。legged_controlのdt=0.015に合わせた
RTI風設定(反復1回・ノード数2倍)は、0.3秒ホライズンでは中速域まで
は既定と近いが高速域で既定を下回り(cmd=1.0で0.313 vs 既定0.407)、
0.6秒ホライズンでは既定(sqp=1)とほぼ同等——legged_controlの数値を
そのまま輸入しても単純に良くなるわけではない。

**結論**: SQP反復回数は単調パラメータではなく、ホライズン長との
組み合わせで最適方向が変わる——これは§5t(ホライズン単体)・§5u
(高さ単体)・§5v(組み合わせ)で繰り返し見てきた「このMPCの
ハイパーパラメータは互いに非単調・非汎化的に相互作用する」という
パターンの、また別の実例。legged_controlの設定値(sqpIteration、
dt、centroidalModelType等)を個別に輸入しても、我々の定式化・数値的
特性の違いにより同じ恩恵は得られない可能性が高く、パラメータごとに
実測で確認する以外に近道はなさそうだという、本セッション全体の
結論を補強する結果となった。

**副産物**: 実験の過程で「間違ったベースラインと比較していた」ことに
気づき訂正した——`auto_detect_*`系の関数が`default_with_kin`の
リテラルデフォルトを暗黙に上書きすることがある、という設計は今後も
同様の取り違えを招きやすいため、注意喚起として記録しておく。

**今後の課題**: (a) sqp_iterations×horizon_stepsの2次元グリッドを
より広く走査し、符号が反転する境界を特定する、(b) ②(タスク空間
→関節空間のヤコビアン重み写像)はOCS2本体でも確認済みで実装コストは
軽いため、次に試す価値がある、(c) ①(true full centroidal結合)は
本質的だが実装コストが最も大きく、CRBA由来の遠心運動量行列(の
結合ブロック)とその`q`微分の追加が必要——独立した実装タスクとして
別途計画すべき。

**訂正(2026-07-17、§5abの調査中に発覚)**: 上表の「真の既定」表記は
**再び誤りだった**——`auto_detect_centroidal_mpc_config`(`gait.rs:424`)
は`CentroidalMpcConfig::default()`が返す`sqp_iterations=1`を、関数の
最後で`cfg.sqp_iterations = 3;`(コメント: "3 = legged_control-style
sweet spot")と**明示的に上書き**しており、実際の既定値は
**`sqp_iterations=3`**(表の「h=0.3s sqp=3」列、meas 0.401/0.466/0.471
——§5w/5x/5z/5aa/5abの基準値と完全一致)である。「h=0.3s sqp=1」列
(0.331/0.393/0.407)の方が、実は既定より反復を1回に**減らした**非既定
構成だった。データ自体・結論の方向性(0.3秒では反復を増やす方が良く、
0.6秒では逆)は影響を受けないが、「どちらが既定か」の取り違えは
今回で二度目——`auto_detect_*`系関数は`Default`実装の値をその場で
上書きすることがあるため、既定値の主張は必ず`auto_detect_*`関数の
**最後まで**読んでから行うべき、という教訓を重ねて記録する。

### 5z. タスク空間→関節空間の重み写像(②)実装 + 初評価(2026-07-16)

推奨順②に従い、quadruped-gaitに`FullCentroidalMpcConfig::
joint_vel_nominal_jacobian`/`r_taskspace_joint_vel`を追加し、
`R_jointspace = J_nom^T · diag(r_taskspace) · J_nom`
(`J_nom`は各脚の固定名目姿勢での`foot_jacobian_body`、OCS2の
`ocs2_legged_robot::LeggedRobotInterface::initializeInputCostWeight`
と同一の技法、`ref/ocs2`で確認済み)を、既存の平坦な12関節一律
`r_diag[12..24]`の代替として実装。コントローラ側に
`FullCentroidalMpcGaitController::set_task_space_joint_vel_weight`
(オプトイン、`None`で従来の平坦対角に戻る)を追加し、QPのコスト
行列`r_block`(既にdenseな`DMatrix`で構築されていたため、対角のみ
だった部分を脚ごとの3×3ブロックに差し替えるだけで済んだ)を変更。
quadruped-gait全175ユニットテスト・articara回帰テストとも無変化。

**実験**: §5w/5y確定済みの真の既定(`legged_control_parity`、
horizon=10×0.030s、sqp=1)に対し、`r_taskspace=[1.0,1.0,1.0]`
(既存の平坦`r_diag[12..24]=1.0`と同じ全体スケール——Jacobian写像
の"形"の効果だけを重みの大きさの変化と分離する意図)を追加した
場合を比較。

| cmd_vx | 平坦r_diag=1.0(既定) | Jacobian写像 r_taskspace=[1,1,1] |
|---|---|---|
| 0.40 | 0.285 | 0.245 |
| 0.50 | 0.331 | **0.203** |
| 0.65 | 0.393 | **0.046** |
| 0.80 | 0.384 | **-0.024**(反転) |
| 1.00 | 0.407 | **0.058** |

グラフ: `taskspace_weight_comparison.png`。

**結果**: 同じ全体スケールにもかかわらず、**Jacobian写像を入れる
だけで中高速域が明確に悪化**——cmd=0.65付近から急落し、cmd=0.75〜
0.80では一時的に反転(後退)、cmd=1.0でも0.058までしか回復しない。
低速域(cmd≦0.45)はほぼ差がない。

**考察(未検証の推測)**: `J_nom^T·diag(r)·J_nom`は対角行列
ではなく、各脚のhip・thigh・calf間に**非対角の結合項**を持つ
(3関節が共通の脚先速度に寄与するため)。この結合はstance脚では
no-slip拘束(`joint_v`をヤコビアンで完全に拘束)によりほぼ無効化
されるが、swing脚では`joint_v`が自由変数のままこの結合コストを
直接受ける。しかも`J_nom`は"固定名目姿勢(直立時)"で評価された
ものであり、swing中に脚が名目姿勢から大きく離れるほど、そのコスト
計量は実際の運動学的感度とずれていく——legged_control/OCS2でも
同じ固定Jacobianを使っているため、この理論的な弱点自体は共通の
はずだが、我々のswing振幅・脚形状・他の重み(`q_diag`等)との
兼ね合いで、我々の系ではより顕著に悪影響が出ている可能性がある。
確証には至っておらず、per-tickトレースでswing脚のjoint_v配分を
直接確認する必要がある。

**結論**: ②はOCS2本体でも確認済みの正当な技法だが、**同じ技法を
移植しても我々の系では悪化する**——③(SQP反復回数)・§5v(高さ×
ホライズン)と同様、legged_controlの個別の設計判断を単体で輸入
しても恩恵が保証されないという、本セッションを通じた結論をここでも
再確認する形になった。デフォルトでは無効(オプトイン)のままとし、
採用は見送る。

**今後の課題**: (a) per-tickトレースでswing脚のjoint_v配分を確認し、
上記の"固定Jacobianのずれ"仮説を検証する、(b) `r_taskspace`の値
自体を振ってみて(現状[1,1,1]のみ試験)、悪化が写像の"形"由来か
"大きさ"由来かを切り分ける、(c) 推奨順①(true full centroidal結合)
に進むかどうかは、②③がいずれも単純な移植では恩恵が出なかった
ことを踏まえ、コストに見合うかどうか再検討が必要。

### 5aa. true full centroidal結合(①)実装 + 初評価、①②③総括(2026-07-17)

推奨順①に進むにあたり設計を再検討。当初「状態変数ωを正規化角運動量に
定義し直す」完全なOCS2式アーキテクチャを検討したが、これは両ファイル
全体に影響する大改修になると判明。ユーザーの要望(「既存実装に影響を
与えないように」)を受け、**状態表現は変えず、既存の`v̇_com`/`α`方程式に
加算的な補正項を足す**という代替設計に変更:

```text
ḣ = Σwrench                              (既存のv̇_com/α方程式そのもの)
h = Ab·v_base + A_joints(q)·q̇_j          (Abは体幹のみの一定慣性)
⇒ v̇_base = Ab⁻¹·Σwrench − Ab⁻¹·(Ȧ_joints·q̇_j + A_joints·q̈_j)
                            \_________________________________/
                                    新規の補正項
```

`Ab⁻¹`は既存コードが既に計算している`(1/mass_kg, i_world_inv)`そのもの。
`A_joints`/`Ȧ_joints`はmisartaの`compute_centroidal_momentum_matrix`/
`..._time_derivative`(実際に存在する本物のCRBA実装、ゼロから実装する
必要はなかった)。`q̈_j`(関節加速度、QPの決定変数には無い量)はSQPが
既に持つ参照joint_v軌道をホライズンステップ間で有限差分するだけで
近似——新しい決定変数は増やしていない。

**実装**:
- `quadruped-gait::autodetect`に`auto_detect_true_centroidal_coupling`
  を追加。ロボットのmisartaモデル(fixed-base、脚12関節のみ)から
  各脚の`[hip,thigh,calf]`がmisartaのv/q-vectorのどのインデックスに
  対応するかを検出(既存の`joint_signs`と同じname-lookupパターンを
  再利用)。
- `FullCentroidalMpcConfig`に`true_centroidal_coupling_data`
  (misartaモデル+インデックス写像、auto-detect時に自動投入)と
  `enable_true_centroidal_coupling`(デフォルトfalse)を追加。
- `continuous_dynamics_full`に上記補正項を追加。加算先は既存の
  "augmented gravity column"の仕組み(状態の25番目の拡張成分)を
  再利用——ただしこれまで「-9.81固定」だった拡張成分の値を
  「1.0固定の汎用バイアス運搬役」に意味を変える小さなリファクタが
  必要だった(`A[2,24]=1.0`→`A[2,24]=-9.81`、数学的に完全に等価、
  既存回帰テスト全175件で無変化を確認)。
- コントローラ側に`set_true_centroidal_coupling`をこれまでと同じ
  パターンで追加。
- quadruped-gait側に新規ユニットテスト3件追加(合成的な4脚12関節
  misartaモデルで検証): (a) 関節速度・加速度ゼロなら補正項が厳密に
  ゼロになる、(b) フラグOFFなら実データがあっても既存出力と完全一致、
  (c) フラグONで実際の動きがあれば有限かつ非ゼロの補正が入る。
  全178件(既存175+新規3)グリーン。

**実験**: §5w/5yの真の既定(`legged_control_parity`、horizon=10×0.030s、
sqp=1)に対し`true_centroidal_coupling`のON/OFFを比較。

| cmd_vx | 既定(結合なし) | + true_centroidal_coupling |
|---|---|---|
| 0.40 | 0.285 | 0.245 |
| 0.50 | 0.401 | **0.260** |
| 0.65 | 0.466 | **0.038** |
| 0.80 | 0.450 | **-0.155**(反転) |
| 1.00 | 0.471 | **-0.315**(反転) |

グラフ: `true_coupling_comparison.png`。

**結果**: cmd_vx≦0.45付近まではほぼ既定と同じ軌跡を辿るが、そこから
**急激に悪化し、cmd_vx=0.70以降は完全に反転**(後退歩行、cmd=1.0で
-0.315)。②③と全く同じ「低中速は無害、高速で崩壊」というパターンが
①でも再現した。

**①②③ 総括**: 3つとも「legged_control/OCS2が実際にやっていること」を
正確に確認した上で(すべて`ref/ocs2`本体コードで検証済み)、それぞれ
理論的に妥当な形で実装し(②はOCS2と同一のヤコビアン写像技法、③は
実際に使われている反復回数設定、①は本物のCRBAに基づく結合項)、
3つとも**同じ質的失敗モード(中高速域での反転)**に行き着いた。
これは実装ミスの繰り返しというより、**我々のシステムの高速域不安定性
の根本原因が、これら個別の力学定式化の精度不足ではなく、より上位の
構造(footstep plannerのcapture-pointフィードバック、または
`ReferenceTrajectory::from_constant_velocity`の自己参照的な参照生成
——§5sで既に指摘した「積分項の不在」)にある可能性を強く示唆する**。
個別の物理モデルをどれだけ精緻化しても、その上に乗っている
フィードバック構造自体に問題があれば、精緻化はその問題を悪化させる
方向にすら働きうる、というのが本セッション全体を通じた結論。

**今後の課題**: (a) ①②③はいずれもデフォルト無効のまま(オプトイン)
保持し、単体では採用しない、(b) 根本原因(§5sで指摘した積分項の不在、
または参照生成の自己参照性)への直接対策を、①②③を経由せず独立に
検討する方が今後の投資対効果が高いと考えられる、(c) ①の補正項自体
(CRBA・有限差分によるq̈近似)の正しさそのものは疑わしくない
(ユニットテストで数学的性質を確認済み)——悪化の原因は「補正項が
間違っている」ことではなく「正しい補正を、既に脆弱な既存システムに
足すと、既存の脆弱性がより強く表面化する」ことだと考えられる。

### 5ab. §5aaの訂正: k_capture交絡要因の発見と検証(2026-07-17)

ユーザーから重要な指摘: 「OCS2/legged_controlはGo2実機で実績のある
アルゴリズムであり、①②③が軒並み悪化するのはこちら側の実験条件に
何か違いがあるのではないか」。これを受けて5点(トルク出力経路・
歩容タイミングの速度依存性・テストプロトコルの階段状指令・
capture-pointゲインの来歴・`legged_control_parity`単体の健全性)を
再調査した結果、**`k_capture`(capture-pointゲイン)の来歴に強い
交絡要因を発見**:

- デフォルト値`0.05`は2026-05-15の「η実験」——**横方向4N/6N押し込みからの
  復帰**という、①②③の実験対象とは全く別のシナリオ・別のプラント
  (FullCentroidal導入前・parity無しのSRBD)向けにチューニングされた値
  だった(`DEFAULT_CAPTURE_POINT_GAIN_S`のdocコメントに明記)。
- **そのdocコメント自体が「legged_controlは0を使う——参照追従の
  クローズドループの仕方が違うため」と明記**しており、legged_control
  実機の実際の値は0。
- ①②③のどの実験も`k_capture`を一度も触っておらず、無関係な実験で
  チューニングされた値のまま、物理モデルが変わった新しいプラントに
  乗せて比較していた。

**検証実験**: ①(true_centroidal_coupling)について、`k_capture`を
legged_control実機の値である0.0に変えて再走査。

| cmd_vx | 既定(k=0.05)基準 | ①+k=0.05(§5aa、反転) | ①+k=0.0(交絡除去) | 基準+k=0.0のみ |
|---|---|---|---|---|
| 0.50 | 0.401 | 0.260 | **0.455** | 0.451 |
| 0.65 | 0.466 | 0.038 | **0.475** | 0.459 |
| 0.80 | 0.450 | -0.155(反転) | **0.460** | 0.463 |
| 1.00 | 0.471 | -0.315(反転) | **0.471** | 0.460 |

グラフ: `kcap_confound_comparison.png`。

**結果**: **`k_capture`を0にするだけで①の反転は完全に消え**、全区間で
既定と同等かそれ以上の追従性能になった。しかも**結合なしの基準系でも
`k_capture=0`にするだけで同様に健全**(既定k=0.05の基準よりむしろ
やや改善)——つまりこの古いゲイン自体が、①②③導入以前から高速域の
追従をわずかに悪化させる方向に働いていた可能性が高い。

**§5aaの結論を訂正する**: 「①②③は個別に正しく実装しても同じ質的
失敗モードに収束する」という結論は誤りだった。実際には**①②③すべてが
無関係な実験でチューニングされた古いcapture-pointゲインという同一の
交絡要因を共有しており**、その交絡要因が反転の主因だった可能性が高い。
OCS2/legged_controlの実機実績のあるアルゴリズムを疑う前に、こちら側の
実験条件(特に「異なるシナリオでチューニングされたゲインを使い回して
いないか」)を確認すべきだった。ユーザーの指摘が的確だった。

**②③への含意**: ②(タスク空間→関節空間の重み写像)・③(SQP反復回数×
ホライズン)の反転も同じ`k_capture=0.05`交絡要因を共有していた可能性が
高く、再検証が必要。まだ実施していない。

**今後の課題**: (a) ②③についても`k_capture=0`での再実験を行い、同様に
反転が解消するか確認する、(b) `k_capture=0`が既定系全体でも改善方向に
働くように見える(基準+k=0.0がむしろ既定より良い)ことから、
`DEFAULT_CAPTURE_POINT_GAIN_S`自体を0に近づけることが、①②③とは独立に
価値のある変更かもしれない——ただし0.05は元々「横方向外乱からの復帰」
のために導入された値なので、k=0でその耐外乱性能が悪化しないかは
別途確認が必要、(c) ①②③の結合効果(例えば①+②を同時に有効化)は
`k_capture=0`環境下ではまだ未検証。

### 5ac. ②③のk_capture交絡再確認 — 3つとも同一原因と確定(2026-07-17)

§5abで①の反転が`k_capture=0`で解消することを確認した後、同じ再確認を
②③にも拡張。

| 実験 | k_capture=0.05(既存) | k_capture=0.0(再確認) |
|---|---|---|
| ②(taskspace_weight、cmd=0.80) | -0.024(反転) | **0.405** |
| ②(taskspace_weight、cmd=1.00) | 0.058 | **0.348** |
| ③(20×0.030s sqp=3、cmd=0.80) | -0.305(反転) | **0.458** |
| ③(20×0.030s sqp=3、cmd=1.00) | -0.356(反転) | **0.479**(本セッション最良の追従結果) |

グラフ: `kcap_recheck_23_comparison.png`。

**結果**: **②③とも反転が完全に解消**。③(0.6秒ホライズン・sqp=3)に
至っては、`k_capture=0`にしただけで本セッション全体を通じて最良の
追従結果(cmd=0.5〜1.0の全域でmeas_vx 0.46〜0.48にほぼ完全なプラトー)
になった。

**確定した結論**: ①②③すべてで反転が同一の交絡要因(無関係な実験で
チューニングされた`k_capture=0.05`)によって説明できることが、3つ
独立に検証された。legged_control/OCS2由来の物理的定式化(①遠心運動量
結合、②タスク空間重み写像、③SQP反復回数設定)は**いずれも我々の系で
悪さをしていなかった**——ユーザーの最初の指摘(「OCS2/legged_control
は実機実績のあるアルゴリズムであり、こちら側の実験条件を疑うべき」)
が完全に正しかった。§5aaの「①②③すべてが同じ質的失敗に収束する
(=物理定式化そのものが我々の系に合わない)」という結論は誤りであり、
正しくは「①②③すべてが同一の未調整ゲインという交絡要因を共有して
いた」。

**副産物・今後の課題**: `k_capture=0`は①②③の有無にかかわらず単体でも
既定(k=0.05)より良好(§5ab)であり、③のように劇的に改善するケースも
ある。ただし`k_capture=0.05`は元来「横方向外乱(4N/6N)からの復帰」
のために導入された値であり、**k=0にすることでその耐外乱性能が
悪化していないかは未検証**——`diag_external_force_robustness`等の
既存の外乱ベンチマークで確認してから、既定値自体を見直すかどうか
判断すべき。この確認ができれば、①②③の採用判断(オプトインのまま
残すか、既定に組み込むか)も改めて行う価値がある。

### 5ad. k_capture=0の耐外乱性能を確認 — 劣化なし、むしろ改善(2026-07-17)

§5acの残課題に対応。既存の外乱耐性ベンチマーク
(`tests/integration_walk.rs::diag_external_force_robustness`、
namiashi・9シナリオ×既存19モード)に`FullC legged-parity + cap-pt 0.0`
行を追加し(20モードに拡張)、既存の`FullC legged-parity`
(既定k=0.05を継承)と直接比較。`k_capture=0.05`は元々2026-05-15の
η実験で「横方向4N/6N押し込みからの復帰」のために導入された値であり、
①②③のGo2速度追従実験すべてがこの値を無変更のまま引き継いでいた
ことは§5ab/5acで既に確認済み——今回はその**逆方向**、「k_captureを
0に落として速度追従は直ったが、本来の目的だった耐外乱性能を犠牲に
していないか」を確認する。

| シナリオ | k=0.05: peak\|dy\| / 結果 | k=0.0: peak\|dy\| / 結果 |
|---|---|---|
| 横+y 2N | 0.355 / 未復帰 | **0.164** / 未復帰(偏差半分以下) |
| 横+y 4N | 0.470 / 未復帰 | **0.195** / 未復帰(偏差半分以下) |
| 横+y 6N | 0.788 / **転倒**(roll 3.135, min_z 0.082) | **0.507** / 転倒(roll **1.789**, min_z **0.116**——被害は軽微) |
| 前−x 2N | 0.264 / 未復帰 | **0.100** / **✓復帰**(0.00s) |
| 前−x 4N | 0.229 / 未復帰 | **0.069** / **✓復帰** |
| 前−x 6N | 0.200 / 未復帰 | **0.043** / **✓復帰** |
| 垂直−z 4N | 0.319 / 未復帰 | **0.123** / **✓復帰** |
| 垂直−z 8N | 0.298 / 未復帰 | **0.128** / **✓復帰** |
| ヨートルク1.5N·m | 0.293 / 未復帰 | **0.175** / **✓復帰** |

**結果**: **全9シナリオで`k_capture=0`が既定(0.05)と同等かそれ以上**
——横方向押し込み(2N/4N、本来のチューニング対象)では最大偏差が
半分以下に縮小し、最も過酷な6N押し込みでは両方とも転倒するが
`k_capture=0`の方が被害が軽微(roll角が3.135→1.789 rad、およそ半分)。
前方・垂直・ヨー外乱に至っては、既定(0.05)が軒並み「未復帰」の
ところ`k_capture=0`は**すべて即座に復帰**という質的な差が出た。

劣化は一切見られず、むしろ`legged_control_parity`との組み合わせでは
`k_capture=0`が全面的に優れているという結果になった。考えられる説明:
2026-05-15のη実験は`legged_control_parity`導入**前**の(レガシーな
FullCentroidalパスの)チューニングであり、parityが持つ独自の機構
(per-step位相投影によるcontact schedule + swing脚のvertical velocity
拘束)が、古いcapture-pointフィードバックと同等かそれ以上の外乱吸収
能力を既に備えているため、古いゲインは有害無益な干渉になっている
可能性が高い(未検証の推測)。

**結論**: `k_capture=0`への変更に、少なくとも`legged_control_parity`
使用時は耐外乱性能上のトレードオフが見当たらない。§5aa〜5acで確認
した速度追従の改善(①②③反転の解消)と合わせ、`legged_control_parity`
使用時の`k_capture`既定値を0に変更する(またはparity使用時のみ0を
既定にする)ことは、デメリットなしに採用できる可能性が高い。

**今後の課題**: (a) レガシー(parity無し)パスでの`k_capture=0` vs
`0.05`の直接比較はまだ未実施(今回追加した行はparity有りのみ)——
レガシーパスでもk=0が同様に安全か確認する、(b) `k_capture`既定値を
実際に0へ変更する場合、`legged_control_parity`を使わない既存の呼び
出し元(SRBD MPCパス等)への影響を洗い出す必要がある、(c) 6N横押し
はk=0でも転倒するため、この領域の耐性向上は`k_capture`以外の対策
(footstep clampの見直し等)が必要——§5r/5uで既に指摘した既知の限界。

### 5ae. ①②③をk_capture=0の健全な基準の上で再評価(2026-07-18)

§5ab〜5adまでは「①②③がk=0.05だと悪化する」ことの原因が交絡要因
だと確認しただけで、「k=0の健全な基準に対して①②③が本当に改善効果を
持つか」はまだ見ていなかった。新規実験は不要——§5ab/5acで取得済みの
k=0時点の21点フルデータ(基準・①・②・③それぞれ)を並べて比較した。

高速域(cmd_vx=0.5〜1.0、11点)の平均・範囲:

| 系列 | 平均meas_vx | 範囲(最小〜最大) |
|---|---|---|
| 基準(parity + k_capture=0、①②③なし) | 0.4648 | 0.451〜0.482 |
| + ①(true_centroidal_coupling) | 0.4679 | 0.455〜0.483 |
| + ②(task_space_joint_vel_weight) | **0.4089** | **0.348〜0.439** |
| + ③(0.6秒ホライズン・sqp=3) | **0.4691** | **0.458〜0.479** |

グラフ: `123_reeval_k0.png`。

**結果**: 3つとも異なる結論になった。
- **①(true centroidal結合)**: 基準とほぼ完全に重なり、統計的に有意な
  差はない(平均差0.003、本セッション全体で見られてきたソルバーの
  試行ばらつきの範囲内)。**中立**——助けにも害にもなっていない。
- **②(タスク空間→関節空間の重み写像)**: `k_capture=0`にしても
  **単独で緩やかに悪化する傾向が残る**(cmd=1.0で0.348、基準の0.460
  より約24%低い、高速側でなだらかに右肩下がり)。§5zで見た反転ほど
  劇的ではないが、これは交絡要因とは独立に②自体が持つ negative な
  効果だと考えられる。
- **③(ホライズン0.6秒・sqp=3)**: 基準よりわずかに、しかし一貫して
  良い(平均+0.004、範囲が0.021と全系列中最も狭い=最も安定した
  プラトー)。本セッション全体を通じて最も滑らかで高い高速域追従を
  達成している。

**結論**: 「legged_control/OCS2の技法を輸入すれば改善する」という
単純な図式ではなく、**要素ごとに真に異なる効果を持つ**ことが、交絡
要因を除去した上で初めて明確になった。③(長いホライズン+反復回数)は
小さいが本物の改善、①(遠心運動量結合)は中立、②(ヤコビアン重み
写像)は単独でも軽微な悪化——③のみ採用候補として有望、①は当面
オプトインのまま保留、②は現状の実装(固定名目姿勢での写像)のままでは
採用を推奨しない。

**今後の課題**: (a) ③を新しい既定候補として、より広いホライズン×
反復回数のグリッドで頑健性を確認する、(b) ②がk=0でもなお悪化する
具体的なメカニズム(§5zで推測した「固定名目姿勢のヤコビアンがswing中
の実際の姿勢とずれる」仮説)をper-tickトレースで検証する、(c) ③(0.6秒
ホライズン)と①(結合項)を同時に有効化した場合の相乗効果はまだ未検証。

### 5af. 広域俯瞰調査 + ④体幹位置追従重みの実験(2026-07-18)

ユーザーの依頼で、①②③という個別の差異だけでなく、legged_control/OCS2
全体を俯瞰する調査を実施(`gait.info`全12種の歩容、`task.info`の
コスト重み実値、`legged_wbc`のゲイン、状態推定器等)。要点:

- **歩容切り替えは完全に手動**(ROSトピック経由のキーボード操作)。
  速度に応じた自動切り替えは無く、「高速では別の歩容を使っている」
  という仮説は否定された。
- **MPCホライズン**: `1.0秒・dt=0.015・sqpIteration=1`と確定(③で
  既に検証済みの方向性を再確認)。
- **状態推定**: 18状態カルマンフィルタ(接触ゲート信頼度、IMUバイアス
  推定なし)——シム内テストには無関係。
- **コスト重み・ハード制約の実値**(★新規発見、①②③とは独立):
  - 体幹**位置**追従重み: legged_control 1000/1000/1500(x/y/z、
    v_com重みの50〜67倍)、我々は**0/5/50**——同じ速度ランプ参照を
    使っているのにx方向の重みが文字通りゼロだった。
  - 摩擦係数: legged_control 0.3、我々0.5(legged_controlの方が保守的)。
  - 最大接地反力: legged_controlは上限なし、我々は200N上限。
  - WBC遊脚PDゲイン: legged_control 350/37、我々80/8(比率は同じ、
    絶対値は4.4倍弱い)。

**実験(④)**: 体幹位置追従重み`q_diag[6]`(x)/`q_diag[7]`(y)を、
健全な基準(`parity + k_capture=0`)の上で0→25→50(z既定値に合わせる)
とスイープ。

| cmd_vx | (0,5)既定 | (25,25) | (50,50)zに合わせる |
|---|---|---|---|
| 0.50 | 0.451 | 0.444 | 0.460 |
| 0.60 | 0.482 | **0.412** | 0.458 |
| 0.80 | 0.463 | **0.226** | 0.455 |
| 1.00 | 0.460 | **0.189** | 0.469 |

グラフ: `base_pos_weight_sweep.png`。

**結果**: **非単調**。(25,25)は高速域でなだらかに劣化し(cmd=1.0で
0.189、既定の半分以下)、しかし(50,50)まで上げると既定とほぼ同じ
プラトーに戻る——legged_controlの相対比(v_com重みの50〜67倍)に
素直に近づけたつもりの中間値(25)が最も悪く、遠くの端点(50、z重みと
一致)ではむしろ既定と大差ない、という直感に反する結果になった。

**結論**: ④も③と同様「legged_controlの重み配分を輸入すれば素直に
改善する」わけではなく、本セッションを通じて繰り返し見てきた
「MPCのハイパーパラメータは互いに非単調・非汎化的に相互作用する」
というパターンがここでも再現した。ただし①②③のときのような明確な
反転(後退歩行)ではなく、「なだらかな谷」型の劣化である点は異なる。
少なくとも今回試した2点(25, 50)の範囲では、既定(0,5)を上回る明確な
改善は見つからなかった。

**今後の課題**: (a) 0〜50の間をより細かく走査し、谷の正確な位置と
深さを特定する、(b) z軸の重み(50)がなぜ非零で健全なのか(x/yと
何が違うのか)を確認する、(c) ⑤(WBC遊脚PDゲイン引き上げ)・⑥
(最大接地反力上限の撤廃)は今回の調査でまだ未着手。

### 5ag. ⑤WBC遊脚PDゲイン・⑥最大接地反力上限の実験(2026-07-19)

§5afで見つかった残り2つの候補(⑤WBC遊脚PDゲイン、⑥接地反力上限)を、
健全な基準(`parity + k_capture=0`)の上で検証。

**⑤ WBC遊脚PDゲイン**: `swing_kp/kd`を既定(80/8)→中間(175/18.5)→
legged_control実値(350/37)とスイープ。

| cmd_vx | 80/8既定 | 175/18.5 | 350/37(legged_control値) |
|---|---|---|---|
| 0.60 | 0.482 | 0.458 | 0.468 |
| 0.80 | 0.463 | 0.452 | **0.356** |
| 1.00 | 0.460 | 0.459 | **0.275** |

**結果**: 中間値(175/18.5)は既定とほぼ同等だが、legged_control実値
(350/37)まで上げると高速域が明確に悪化(cmd=1.0で既定の6割)。
④・②のような非単調な谷ではなく、**ゲインを上げるほど素直に悪化**
する単調な傾向——高速になるほどswing軌道の変化が速くなり、過度に
硬いPD追従がWBCのトルクフィードフォワードと競合しやすくなっている
と考えられる(未検証の推測)。legged_controlの遊脚タスクは
Cartesian空間(足先位置)でのPDである一方、我々は関節空間PD——
表現方式が異なる中で数値だけ輸入しても素直に恩恵は出なかった。

**⑥ 最大接地反力上限**: `max_normal_force`を200N→無限大(上限撤廃)。

| cmd_vx | 200N既定 | 上限なし |
|---|---|---|
| 0.60 | 0.482 | 0.451 |
| 0.80 | 0.463 | 0.448 |
| 1.00 | 0.460 | 0.469 |

**結果**: **ほぼ無風**——両曲線はノイズの範囲内で完全に重なる。
200N×4脚=800N≫ロボット体重(≈153N)であり、通常歩行でこの上限に
達することはそもそも無かったと考えられる。上限の撤廃は無害だが、
無効化してもしなくても違いが出ない。

グラフ: `swing_pd_maxforce.png`。

**①〜⑥ 総まとめ**: legged_control/OCS2から個別に輸入した6つの
差異のうち、明確に採用価値があったのは**③(MPCホライズン延長+SQP
反復回数)のみ**。①(結合)は中立、②(タスク空間重み写像)・④(体幹
位置重み中間値)・⑤(遊脚PDゲイン)はいずれも「legged_controlの実値に
近づけるほど悪化する」という非単調・単調いずれかの悪化パターンを
示し、⑥(反力上限撤廃)は無風だった。「legged_controlの個別パラメータを
輸入すれば素直に良くなる」という単純化はどの項目についても成立せず、
本セッション全体を通じて唯一③だけが真に価値のある変更として残った。

**今後の課題**: (a) ③を新しい既定として正式に採用するかどうかの
判断、(b) ②④⑤で見られた「legged_controlの数値に近づけるほど悪化」
という現象に共通するメカニズムがあるか(表現方式の違い・我々の
プラントの数値的特性の違い等)、まとめて考察する価値がある、
(c) ①〜⑥はすべてオプトインのまま保持。

### 5ah. 比較対象そのものの再検証 — task.info/gait.infoはA1用、Go2実機実績なし(2026-07-19)

ユーザーから改めて「legged_controlはGo2実機で実績のあるアルゴリズム
なのだから、シミュレーション前提・体高・歩容パラメータ等の差分を
再確認すべき」との指摘。これを受けて、①〜⑥で比較対象にしてきた
`ref/legged_control`(chvmpフォーク)自体を再検証したところ、**比較の
前提そのものに重大な問題**が見つかった。

**ローカル調査**: `ref/legged_control`の`legged_examples/legged_unitree`
には**a1とaliengoのURDF/設定しか存在せず、go2は一切含まれていない**
(ファイル名・コミット履歴どちらにも"go2"の文字列が一つも出てこない)。
A1の実仕様を確認したところ、legged_controlの`task.info`にあった
「一律33.5Nm」というトルク上限は**A1の実際のモータ仕様そのもの**
(`unitree_description/urdf/a1/const.xacro`の`hip/thigh/calf_torque_max
= 33.5`)であり、ANYmal由来の汎用プレースホルダーではなかった
(§5afでの推測を訂正)。A1の総質量は12.776kg(trunk 6.0kg + 4×
(hip 0.595+thigh 0.888+calf 0.151+foot 0.06)kg)——我々のGo2
(約15.6kg)より約22%軽い。

**外部調査(ユーザー許可を得てWeb検索)**:
1. **「go2対応」を謳うlegged_controlフォークは実在する**
   (`Feng1909/legged_control_go2`、`WeixianLin-cc/leggedcontrol_go2`)。
   しかし**そのtask.infoはA1版と一字一句同一**——`p_base_z=0.3`、
   swing height 0.08、遊脚PDゲイン350/37、Q/R重み全て、摩擦係数0.3、
   ホライズン1.0秒、**そしてトルク上限も依然33.5Nm**(Go2の実際の
   最大トルク約45Nmではなく)。URDF(ロボット記述)だけ差し替えて、
   **数値は一切再チューニングされていない**ことが直接の差分確認で
   判明した。
2. **実機Go2でのOCS2/legged_control系NMPC+WBCの実績を示す一次資料は
   見つからなかった**。A1についてはREADMEに実機実績(XPeng、Unitree、
   Hybrid Robotics)が明記されているが、Go2については実機実績の記載
   自体が無い。最も近い"Go2 whole-body controller"の公開事例
   (UMIonLegs、arXiv 2407.10353)は**RL学習方策(MLP、PDで関節位置を
   追従)であり、OCS2的なモデルベースNMPC+階層QP-WBCとは全く異なる
   制御パラダイム**だった。
3. **A1とGo2の物理仕様比較**(Unitree公式仕様等より):

| 項目 | A1 | Go2 | 差 |
|---|---|---|---|
| 質量 | 12kg | 約15kg | +25% |
| 全長×全高 | 50×40cm | 70×40cm | +40%(全長) |
| 最大関節トルク(モータ仕様) | 33.5Nm | 約45Nm | +34% |
| 大腿リンク長 | 約0.2m | 0.213m | +6.5% |

脚のジオメトリは比較的近い(+6.5%)が、質量・トルク容量は大きく
異なる(+25%、+34%)。

**結論**: **①〜⑥で比較対象にしてきたlegged_controlの数値(コスト
重み・PDゲイン・摩擦係数・トルク上限等)はすべてA1向けであり、
Go2向けに一度も再チューニングされたことのない値だった**可能性が
極めて高い。しかも「legged_controlがGo2実機で実績がある」という
前提自体、公開情報からは裏付けが取れなかった——実在するのは
「URDFだけ差し替えたGo2設定」であり、実機検証の記載は無い。

これは§5aa〜5agとは**独立した、より上流の交絡要因**であり、
①〜⑥の結果を整合的に説明し直す:
- **③(MPCホライズン長・SQP反復回数)はアーキテクチャ的・アルゴリズム的
  な選択**であり、ロボットの質量・トルク容量にあまり依存しない
  ため、A1向けにチューニングされた値の"方向性"(長いホライズン、
  少ない反復)がGo2でも有効だった、という説明が成り立つ。
- **②④⑤(タスク空間重み・体幹位置重み・WBC遊脚PDゲイン)は
  具体的な数値そのもの**であり、A1(12kg・33.5Nm一律)向けに
  チューニングされた値を、質量25%増・トルク容量34%増で非対称な
  Go2にそのまま持ち込んでも効かなかった、というのは、むしろ
  **当然の結果**だったと言える。

**今後の課題**: (a) 本セッションの①〜⑥の実験自体は無駄ではなく、
「A1向けの絶対値を輸入しても効かない」ことを実測で確認できた点に
価値がある——今後Go2向けに②④⑤を再チューニングする場合は、A1の
数値をそのまま参考にせず、Go2自身の質量・トルク容量に対する
相対比(例: ④の体幹位置重み/v_com重みの比率など)から出発すべき、
(b) 「OCS2/legged_control系のGo2実機実績」という前提自体は、この
セッションの範囲では検証できなかった——もし何らかの一次資料
(社内資料・論文・特定のリポジトリ)を把握していれば、それを直接
確認する方が有効。

### 5ai. ①〜⑥の再分類 + ⑤の単位変換による解決(2026-07-19)

§5ahでの「Go2実機実績」の前提を巡るユーザーとのやり取りを受け、
①〜⑥を「A1の生数値を輸入したか、自前のスケールで振ったか」で
再分類したところ、**実際にA1の生数値をそのまま使ったのは⑤だけ**
だったと判明(§5ah時点の「②④⑤全部がA1数値のせい」という記述は
一部誤り、訂正):

| # | 実際に使った値 | A1/Go2ミスマッチの該当性 |
|---|---|---|
| ① 結合 | **Go2自身の**質量・慣性(misartaのCRBA) | 無関係 |
| ② 重み写像 | `[1,1,1]`(我々自身のスケール) | 無関係 |
| ③ ホライズン+SQP | アーキテクチャ的選択 | 無関係 |
| ④ 体幹位置重み | 0/25/50(我々のq_diagスケール) | 無関係 |
| ⑤ 遊脚PDゲイン | **350/37(A1の生値そのまま)** | **該当** |
| ⑥ 反力上限撤廃 | 構造的な選択(数値ではない) | 無関係 |

**⑤の深掘り**: legged_controlの遊脚WBCタスクは**Cartesian空間
(足先の位置・速度、誤差の単位はメートル)**のPD、我々は**関節空間
(誤差の単位はラジアン)**のPD——`350`と`80`は次元・単位からして
直接比較できない数値だった。

`go2_diag_swing_pd_gain_jacobian_conversion`でGo2のFL脚の実際の
ヤコビアン(`foot_jacobian_body`、名目立脚姿勢)を計算したところ:

```
特異値: 0.317, 0.280, 0.133 [m/rad]
Frobeniusノルム: 0.443
```

`Δp_cartesian ≈ J·Δq_joint`(微小角近似)なので、`kp_cart·Δp ≈
(kp_cart·‖J‖)·Δq`——legged_controlのCartesianゲインに、このヤコビアン
の大きさ(1ラジアンあたり何メートル足先が動くか)を掛けることで、
関節空間の等価値に変換できる。`sigma_max=0.317`使用でkp≈111・kd≈12、
`frobenius=0.443`使用でkp≈155・kd≈16——生の"350/37"とは全く違う、
我々の既定(80/8)に近い(やや上回る)値になった。

**実験**: 変換後の値(111/12、155/16)を、健全な基準
(`parity + k_capture=0`)の上でテスト。

| cmd_vx | 80/8既定 | 350/37(生値、§5ag) | 111/12(σ_max変換) | 155/16(Frobenius変換) |
|---|---|---|---|---|
| 0.50 | 0.451 | 0.465 | 0.471 | 0.478 |
| 0.80 | 0.463 | **0.356** | 0.464 | 0.458 |
| 1.00 | 0.460 | **0.275** | 0.458 | 0.472 |

グラフ: `swing_pd_jacobian_converted.png`——変換後の2本は既定とほぼ
完全に重なり、生値だけが高速域で明確に乖離する。

**結果**: **単位変換するだけで、⑤の悪化は完全に解消**した。変換後の
2つの値はいずれも既定(80/8)とほぼ同等(高速域平均: 既定0.4648、
σ_max変換0.4636、Frobenius変換0.4658——すべて誤差範囲内)——生の
"350/37"が示した明確な悪化(cmd=1.0で既定の6割)は再現しない。

**結論**: ⑤の失敗は、A1とGo2の質量・トルク容量の違い(§5ah)以前に、
**Cartesian空間の数値を関節空間の数値としてそのまま比較していた
という、次元の取り違えが主因**だった。正しく単位変換すれば、
legged_controlの遊脚ゲインの"設計意図"(我々よりやや高めのゲイン)
自体は我々の系でも無害——③に次ぐ、**もう1つの「輸入して悪くない」
項目**として位置付けられる(③のような明確な改善ではなく、中立)。

これにより、①〜⑥のうち明確に悪化したのは**②(タスク空間重み写像
の形状効果)・④(体幹位置重みの非単調な谷)のみ**に絞り込まれた——
どちらも自前のスケールで振った実験であり、A1/Go2ミスマッチでは
説明できない、我々自身のシステム固有の性質である。

**今後の課題**: (a) ②④についても、⑤と同様「単位・表現方式の
ミスマッチ」がないか再検討する価値がある(②は既に関節空間で
テストしているため該当しなさそうだが、④の体幹位置重みは
legged_controlの参照生成方式との整合性を再確認する余地がある)、
(b) ⑤(Jacobian変換後)を新しい既定候補として③と合わせて採用する
かどうかの判断。

### 5aj. 速度追従プラトーのベンチマーク較正(外部調査、2026-07-19)

我々の系は0.5m/s付近から高速域でmeas_vx≈0.46〜0.48m/sのプラトーに
収束する(§5w以降一貫した結果)。このプラトーが「妥当な範囲」なのか
「ボトルネックの兆候」なのかを較正するため、ユーザー許可のもと
外部調査を実施。

| 対象 | 数値 | 文脈 | 出典 |
|---|---|---|---|
| Go2 Air/Pro/Edu公式最高速度 | 2.5/3.5/3.7(実験室最大5)m/s | 歩容不明(おそらくbound/run) | unitree.com/go2/ |
| **A1公式最高速度** | **3.3 m/s** | 連続走行 | unitree.com/products/a1/ |
| A1級・外部記録 | 3.7 m/s | トレッドミル記録(MIT) | Unitree A1公式ページに記載 |
| ANYmal実機(OCS2系NMPC+WBC) | **1.5 m/s** | 20cm障害物越えを含む不整地 | Bjelonic et al. 2022, IJRR |
| legged_control(A1向け設定値) | 0.5 m/s | 自律ナビゲーション巡航速度(**設定値、達成値ではない**) | `reference.info` |
| 我々のシステム | 0.46〜0.48 m/s | Go2シム、Trotプラトー | 本セッション |

**重要な発見**: legged_controlの`reference.info`にある
`targetDisplacementVelocity=0.5`は、**A1というロボット自体の性能上限
(公式値3.3m/s)とは無関係**——単に自律ナビゲーション用に控えめに
設定された巡航速度の設定値に過ぎない。legged_controlが実機で
「達成した」速度を示す一次資料はどこにも見つからなかった(設定ファイル
の目標値のみ)。OCS2系NMPC+WBC(モデルベース、RLではない)が実機で
実際に達成した検証可能な数値としては、ANYmalでの1.5m/s(不整地・
障害物越え、Bjelonic et al. 2022)が唯一確認できたものだった
(Grandia et al. 2023の平地trot速度は、論文本文が取得できず未確認)。

**結論**: 我々の0.46〜0.48m/sは、legged_controlの**設定上の**巡航速度
(0.5m/s)とほぼ同水準だが、ロボット自体(A1: 3.3m/s、Go2: 2.5〜
3.7m/s)の機械的な性能上限にはまだ遠く及ばず、唯一確認できたNMPC+WBC
実機達成値(ANYmal 1.5m/s、ただし別ロボット・不整地条件)よりも低い。
「0.5m/s弱で頭打ち」はハードウェアの限界ではなく、footstep clamp・
capture-pointフィードバック等、制御アルゴリズム側のボトルネックで
ある可能性が高いことが、外部ベンチマークとの比較からも裏付けられた。

**今後の課題**: (a) Grandia et al. 2023の平地trot速度を別途確認できれば、
より直接比較可能な基準点が得られる、(b) footstep clampの上限
(`max_step_length_m`)自体を引き上げる実験は§5rで示唆されたのみで
未実施——プラトーの直接的な原因として次に検証する価値がある。

### 5ak. max_step_length_m引き上げで理論通りプラトーが移動(2026-07-19)

ユーザーから「これは"標準的なTrot"としての限界の可能性は?」との
質問。`GaitConfig::trot()`の実値(`cycle_period_s=0.4, duty_factor=0.5,
max_step_length_m=0.10`)から理論式:

```
stance_duration = cycle_period_s × duty_factor = 0.2秒
v_理論上限 = max_step_length_m / stance_duration = 0.10 / 0.2 = 0.5 m/s
```

を計算したところ、観測しているプラトーの立ち上がり(cmd_vx=0.5付近)
と**完全に一致**。ただしこれは「Trotという歩容そのものの限界」では
なく「この特定の設定値(周期0.4秒・duty比50%・最大ストライド0.10m)
の限界」——Go2の脚全長(約0.426m)に対し0.10mはわずか23%であり、
実際の四足ロボットのTrotでは脚長の30〜50%程度のストライドも珍しくない
ため、`max_step_length_m`を引き上げれば理論上限も比例して上がる
はずだという仮説を、健全な基準(`parity + k_capture=0`)の上で
直接検証した。

| max_step_length_m | 理論上限 | 実測ピーク(cmd=1.0でのmeas_vx) |
|---|---|---|
| 0.10(既定) | 0.5 m/s | 0.460 |
| 0.15 | 0.75 m/s | **0.636** |
| 0.20 | 1.0 m/s | **0.852(cmd=0.95では0.881)** |

グラフ: `max_step_length_sweep.png`——cmd_vx=0.5未満では3本の曲線が
完全に重なり(どの設定でもクランプがまだ効かない領域)、各設定の
理論しきい値(0.5/0.75/1.0、縦点線)を境に、まさにその設定だけが
そこから上に離脱していく、教科書的にきれいな分岐を示した。

**結果**: **理論通り、`max_step_length_m`を上げるとプラトーが比例して
上昇することを直接確認**。0.20mでは1.0m/sの指令に対しmeas_vx=0.852
(85%)まで追従し、cmd=0.95では0.881(93%)——0.10m既定時の46%から
大幅に改善した。理論値(0.75/1.0)と実測ピーク(0.64/0.85〜0.88)には
若干のギャップがあるが(遊脚速度上限・全身動特性など、純粋な
キネマティクス以外の副次的な制約によるものと考えられる)、
本セッション全体で最も明確でクリーンな「輸入ではなく我々自身の
設定値の見直しによる改善」だった。

**結論**: これまで「制御アルゴリズムのボトルネック」として①〜⑥
(legged_controlからの輸入)を検証してきたが、実際にはより単純な
話——**footstep plannerの安全マージン(`max_step_length_m=0.10`)
自体が保守的すぎた**、というのが§5s以来一貫して観測してきた
プラトーの直接的な原因だった。この設定値はGo2向けに検証・調整
されたことのない、おそらく初期の他ロボット向け設定を引き継いだ
値である可能性が高い(③同様、legged_controlとの比較ではなく、
自分たち自身の設定の妥当性を見直すことで見つかった改善)。

**今後の課題**: (a) 0.20mを超える領域(0.25m、脚長の59%)でさらに
理論上限が上がるか、あるいは遊脚速度やWBCの安定性が先に制約に
なるかを確認する、(b) `max_step_length_m`引き上げが低速域
(≦0.2m/s、§5s以来未解決の相対誤差)には影響しないことを確認する、
(c) ③(ホライズン延長)・⑤(単位変換後の遊脚PDゲイン)・本項目
(max_step_length引き上げ)を**組み合わせた**場合の相乗効果は
まだ未検証——これが次の自然な実験候補。

### 5al. §5akの理論-実測ギャップ分析 + ①の再評価(2026-07-19)

ユーザーから「理論上限との差が広がったのはなぜか」との指摘。ピーク値
(cmd=1.0時点ではなく各設定の実測ピーク)で再計算すると:

| max_step_length_m | 理論上限 | 実測ピーク | 比率 |
|---|---|---|---|
| 0.10 | 0.5 | 0.482(cmd=0.6) | 96.4% |
| 0.15 | 0.75 | 0.653(cmd=0.75) | 87.1% |
| 0.20 | 1.0 | 0.881(cmd=0.95) | 88.1% |

**0.10→0.15で比率が急落した後、0.15→0.20はほぼ横ばい**——際限なく
悪化し続けているわけではない。考えられる要因: (a) §5sで指摘した
「制御ループ全体に積分成分がない」という構造的性質により、目標
(クランプされたストライド長)自体が大きくなるほど、同じ相対誤差でも
絶対ギャップが拡大する、(b) 実測データのpeak_roll列を見ると、
0.10m時は最大0.06〜0.07だったのが0.20m時は最大0.10前後まで増加——
ストライドを伸ばすほど脚振り動作が体幹をより強く揺らし、その余力が
姿勢の立て直しに取られている可能性。

**①(真の遠心運動量結合)の再評価**: §5aeで①を「中立」と結論したが、
その検証は`max_step_length_m=0.10`時代(実測ピークが最大でも0.48
m/s程度)のもの。`max_step_length_m=0.20`の新しい基準(実測ピーク
0.85m/s近く)の上で①を再テストしたところ:

| cmd_vx | 基準(結合なし、§5ak) | + ①(結合あり) |
|---|---|---|
| 0.65 | 0.586 | 0.624(ピーク) |
| 0.80 | 0.683 | **0.497** |
| 0.90 | 0.767 | **0.362** |
| 1.00 | **0.852** | **0.276** |

グラフ: `true_coupling_at_new_speed.png`——cmd_vx=0.6〜0.65までは
2本の曲線がほぼ完全に重なるが、そこから明確に分岐し、①ありの方は
なだらかに悪化していく(peak_rollも0.10〜0.13と基準より高い)。

**結果**: **仮説通り、①は低〜中速では中立だが、真に高速域(0.6m/s超)
に到達すると明確に悪化する**。§5aaで見た「悪化」は交絡要因
(k_capture=0.05)のせいだったが、今回はその交絡を除去した上で、
かつ実際に高速域まで届くようになった新しい環境で再現された、
純粋な①自体の効果——脚を振る運動量が体幹に与える反作用が、
実際に速い脚振りが起きる速度域でようやく物理的に無視できない大きさに
なる、という直感的にも筋が通る結果。

**結論**: パラメータの効果は絶対的なものではなく、**その効果が
「意味を持つ速度域に実際に到達しているか」に強く依存する**——
§5aeでの①の「中立」判定は、当時のテスト環境(0.10m時代、実測上限
0.48m/s)では正しかったが、ボトルネック(footstep clamp)を解消して
初めて①の真の(悪)影響が見えてきた、という多段的な発見になった。

**今後の課題**: (a) ①をオプトインのまま無効に保つ(この高速域では
明確に悪化するため)、(b) ③・⑤も同様に、`max_step_length_m=0.20`
の新環境で再評価する価値がある(§5aeの中立・改善判定も旧い低速環境
でのものだったため)、(c) peak_rollの増加(0.10m→0.20mで倍増近く)
自体を安定化させる対策(q_diagの角度項強化等)を検討する。

### 5am. 姿勢安定化重み引き上げ実験 — 逆効果と判明(2026-07-19)

§5alの提案(c)を検証。`max_step_length_m=0.20`基準の上で
`q_diag[9]`/`q_diag[10]`(roll/pitch姿勢追従重み、既定25/25)を
50/50・100/100に引き上げ。

| cmd_vx | 25/25(既定) | 50/50 | 100/100 |
|---|---|---|---|
| 0.70 | 0.603 | 0.540 | 0.572 |
| 0.85 | 0.685 | 0.420 | 0.383 |
| 0.95 | **0.881** | 0.142 | 0.182 |
| 1.00 | **0.852** | **-0.183(反転)** | **-0.062(反転)** |

グラフ: `roll_pitch_weight_sweep.png`。

**結果**: **予想に反し、重みを上げるほど追従性・安定性の両方が悪化**
した。50/50・100/100とも、cmd=0.6付近までは既定とほぼ同じだが、
そこから明確に悪化し、高速域(cmd≧0.95)では**反転(後退歩行)**
まで起きた。しかもpeak_rollは重みを上げても改善せず、むしろ
0.10〜0.11(既定)から0.13〜0.15(50/50・100/100)へと**悪化**した——
姿勢を強く追従させようとした結果、かえって姿勢が乱れるという、
直感に反する結果になった。

**考えられるメカニズム(未検証の推測)**: 参照生成
(`ReferenceTrajectory::from_constant_velocity`)はroll/pitchの参照を
一定(ゼロ近傍)に保つが、実際のTrotは周期的な体幹の揺れを自然に伴う。
roll/pitchの追従重みを過度に強くすると、MPCがこの自然な周期的揺れを
「参照からの逸脱」として強く抑え込もうとし、その結果生じる過大な
補正力(GRFの急峻な変化)が、かえって振動・不安定性を誘発している
可能性がある——単純な「重みを上げれば安定する」という直感は、
この結合されたシステムでは成立しなかった。

**結論**: これは②④⑤(輸入した生数値)や①(旧環境では中立)と同様、
「単純にパラメータを一方向に動かせば改善する」という予測が外れた、
本セッションでもう一つの事例。姿勢安定化重みの引き上げは**採用
見送り**——既定値(25/25)のまま維持する。

**今後の課題**: (a) 重みを下げる方向(25未満)も試す価値がある——
もしかすると現状の25自体が既に「やや強すぎる」側にあり、下げる方が
改善する可能性、(b) roll/pitchの参照自体をTrotの自然な周期的揺れに
追従させる(定数参照ではなく)設計に変えることが、根本的な解決に
なるかもしれない、(c) §5alの他の候補((a)積分項追加、(c)③⑤の
新基準再評価、(d)max_step_length_mのさらなる引き上げ)は未検証の
まま残る。

### 5an. 現在の設定の真の上限探索 + 階段刻み幅による不一致の発見(2026-07-19)

ユーザーから「今のパラメータでのcmd_vxの最大値は?」との質問。§5akの
細かい階段(0.05刻み、0〜1.0m/s)ではcmd=1.0時点でもまだ理論値
(1.0m/s)に向けて上昇中だったため、真の上限は未確認だった。§5rの
手法を踏襲し、同じ設定(`legged_control_parity=true, k_capture=0,
max_step_length_m=0.20`)で0〜2.0m/s・0.10刻みの粗い階段を実行。

| cmd_vx | 粗い階段(0.10刻み) | 細かい階段(0.05刻み、§5ak) |
|---|---|---|
| 0.60 | 0.567 | 0.567 |
| 0.70 | **0.605(ピーク)** | 0.603 |
| 1.00 | **0.567** | **0.852** |
| 1.40 | 0.052 | (未測定) |
| 1.50 | -0.351(反転) | (未測定) |
| 2.00 | -0.860 | (未測定) |

グラフ: `ceiling_coarse_vs_fine.png`。

**重大な発見**: cmd_vx≈0.6までは両者はほぼ完全に一致するが、
**cmd_vx=1.0の同一地点で、粗い階段は0.567・細かい階段は0.852と
大きく食い違う**。粗い階段では、cmd=0.7(0.605、比率86%)を
ピークに以降なだらかに下降し、cmd=1.4付近から明確に反転、
cmd=1.8〜2.0でmeas_vx≈-0.85にほぼ飽和する——つまり§5r・§5sで
`max_step_length_m=0.10`の旧設定について見たのと**質的に同じ
"ピーク→なだらかな下降→反転→負のプラトー"というパターンが、
0.20mでも(ピーク速度・反転開始点がそれぞれ高い側にシフトした形で)
再現された**。

**不一致の原因(未検証の推測)**: 0.05刻みの階段は各レベルの変化が
小さく、システムがほぼ準静的に(前のレベルからの小さな摂動として)
高い速度域まで"忍び込む"ことができ、0.10刻みのより急な変化が引き
起こす不安定化を回避できている可能性がある。あるいは、各レベルの
滞留時間(約2.86秒)が、`max_step_length_m=0.20`という新しい(より
速い脚振りを伴う)設定条件下では、真の定常状態に達するのに十分では
なくなっている可能性もある——§5sで確認した「定常状態への収束は
数百ms以内」という前提は`max_step_length_m=0.10`時代のものであり、
0.20mでも同様に成り立つかは未検証。

**結論**: 「今のパラメータでのcmd_vxの最大値」への回答は、**測定
方法(階段の刻み幅)に依存する**という、当初の想定より込み入った
ものになった。広い範囲を粗く走査した今回の結果を真の定常特性と
みなすなら、**実用上の上限はcmd_vx≈0.7m/s(実測ピーク0.605、
比率86%)、cmd_vx≈1.4付近から反転が始まる**。§5akの「0.20mへの
引き上げでcmd=1.0まで0.85まで追従した」という報告は、細かい階段
特有の(まだ完全には理解できていない)効果によって、本来の反転を
一時的に"すり抜けた"結果だった可能性が高い。ただし`max_step_length_m
=0.10`時代のピーク(0.34〜0.46)と比べれば、0.20m時のピーク(0.60)
は依然として明確な改善であり、§5akの結論(引き上げが効く)自体は
覆らない——ただし改善幅は§5akで報告したほど劇的ではなかった。

**今後の課題**: (a) 階段の刻み幅・滞留時間自体が測定結果に与える
影響を体系的に調査する(例: 同じ0-2.0m/s範囲を0.05刻み・120秒等、
より細かく長く走査して、粗い階段の結果と一致するか確認)、(b) 各
レベルの前半/後半の実測値を比較し、`max_step_length_m=0.20`でも
§5sと同様に数百ms以内で定常状態に収束しているか再確認する、(c) 本
セクションの発見により、これまでの①②③④⑤の评価(特に§5ak以降の
`max_step_length_m=0.20`基準での結果)も、階段刻み幅依存性の影響を
受けている可能性があり、再確認が必要かもしれない。

### 5ao. Canter/Gallop足掛かり調査 — 飛行相(0接地)の生存確認(2026-07-19)

Trotの上限特定(§5an)を受け、ユーザーからCanter/Gallopへの挑戦
希望。着手前にquadruped-gaitのgait基盤をExploreエージェントで調査:

- `GaitConfig`のduty_factorは**全4脚共通の単一スカラー**。
  `GaitType::phase_offsets()`も変種ごとの固定4要素配列で、前後脚
  非対称なduty_factorは現状**未対応**——Canter/Gallopの本質的な
  必要条件であり、中心的な設計課題。
- 一方、`ContactDrivenPhase`はforce駆動補正のみでduty_factorを
  直接読まないため非対称化にも耐性あり。SRBD MPCの`continuous_
  dynamics`もB行列が全脚swingで完全ゼロになった場合、正しく自由
  落下(v̇_z=g)に退化する形で書かれており、WBCの`friction_cone`/
  `no_contact_motion`タスクも0接地でゼロ行に縮退する——**つまり
  「飛行相(0接地)」を支える下地はコード上すでにあるが、一度も
  実際にテストされたことがない**、という有望な発見。
- 既存`Bound`プリセット(`FL=FR=0.0, RL=RR=0.5`, duty=0.5)は
  ちょうど前後2脚ずつが隙間なくタイルする構成——`with_duty_factor()`
  で下げれば、コード変更ゼロで飛行相を持つ挙動を試せる。

ユーザーはこの「安価な事前検証」から始めることを選択。2段階で検証:

**A. スケジュール検証(MuJoCo不要)**: `go2_diag_bound_duty_factor_
flight_phase_sweep` — `GaitConfig::bound().with_duty_factor(d)`を
d=0.50〜0.25で振り、`ContactDrivenPhase::nominal_legs()`を1周期
2000サンプルで走査し、0接地サンプルの割合を計測。

| duty | min_stance | flight_phase_fraction |
|---|---|---|
| 0.50 | 2 | 0.000 (0%) |
| 0.45 | 0 | 0.100 (10%) |
| 0.40 | 0 | 0.200 (20%) |
| 0.35 | 0 | 0.300 (30%) |
| 0.30 | 0 | 0.400 (40%) |
| 0.25 | 0 | 0.500 (50%) |

理論式`flight_frac = 1 - 2*duty`と完全一致(例: duty=0.35→
1-0.70=0.30 ✓)。スケジュール自体は期待通り正確に動作。

**B. 実力学検証(MuJoCo)**: `go2_wbc_bound_flight_phase_duty_sweep`
— `GaitMode::FullCentroidal` + `legged_control_parity` +
`k_capture=0`(既存の健全な基準)を`GaitType::Bound`に適用し、
duty=0.50(飛行相なし、基準)とduty=0.35(30%飛行相)をcmd_vx=0.3・
4.5秒で比較。

| 設定 | min_z | peak_roll | peak_pitch | Δx (4.5s) | meas_vx |
|---|---|---|---|---|---|
| duty=0.50(基準、飛行相なし) | 0.216m | 0.000rad | 0.282rad | **-0.745m** | **-0.166** |
| duty=0.35(30%飛行相) | 0.219m | 0.021rad | 0.307rad | -0.093m | -0.021 |

両ケースとも全ステップでfinite(NaN・発散なし)、`min_z`も転倒
閾値(0.15m)を大きく上回って安定。加えて、両ケースを通してWBCの
HoQpソルバから大量の`Infeasible`/`MaxIterations`警告が出続けた
(数百〜千行規模)。

**良い知らせ**: 飛行相(0接地)の力学自体は、SRBD/FullCentroidal MPC
・WBCとも**破綻しない**。机上でしか確認されていなかった`n_stance=0`
コードパスは、実際にMuJoCo上でも安定に動作した——Canter/Gallopの
基盤としては生存確認が取れた。

**悪い知らせ、より根本的**: **duty=0.50(飛行相なし)の素のBound
ですら、cmd_vx=+0.3指令に対してmeas_vx=-0.166と大きく逆走する**。
つまり飛行相の有無以前に、Trot専用にチューニングされてきた現在の
最良設定一式(`legged_control_parity`, `k_capture=0`,
`max_step_length_m`等——このセッションで積み重ねてきたすべて)は、
**Bound自体に一切転移しない**。むしろ興味深いことに、飛行相を
入れたduty=0.35の方が逆走が小さい(-0.021 vs -0.166)——飛行相
そのものが症状を軽減する可能性があるが未検証。加えてHoQpの頻繁な
Infeasibleは、Boundの短い周期(0.3s、Trotの0.4sより速い)・
異なる接地スケジュールに対して、WBCタスクの定式化(あるいはQP
の重み/制約)がそもそも噛み合っていないことを示唆する。

**結論と提言**: Canter/Gallopは非対称duty_factorの実装に加え、
Boundより長く・複雑な飛行相を持つ、Boundの厳密な上位互換の課題。
**その土台となるBound自体が(飛行相の有無に関わらず)現状全く
歩けていない**以上、Canter/Gallopへ進む前に、まずBoundを
Trotと同程度に歩かせられるチューニング(k_capture、MPCホライズン、
swing PD、footstep plannerのBound向け再検討——これまでTrotに
対して行ってきたのと同種の作業)が必要、というのが現実的な次の
一歩と考えられる。あるいは、Bound自体の優先度を下げ、Canter/
Gallopに必要な非対称duty_factorのアーキテクチャ設計を先に進める
という手もある——いずれもユーザーの判断が必要。

ユーザーは「まずBound自体を歩けるようにする」を選択。

### 5ap. Bound逆走の原因切り分け調査 — Trotチューニングは無罪、原因は別にある(2026-07-19)

§5aoで見つかった「素のBound(飛行相なし)ですら逆走する」という
問題の原因を切り分ける。仮説: このセッションでTrot向けに積み
重ねてきた`legged_control_parity`・`k_capture=0`等のFullCentroidal
固有の調整が、Boundの全く異なる接地パターン(前後ペアの反対位相)
と噛み合っていないのではないか。

`go2_wbc_bound_baseline_survey`: `GaitConfig::bound()`の素の
デフォルト(duty=0.5, cycle=0.3s, max_step_length_m=0.12——Trot由来の
上書き一切なし)の上に、4段階の構成を低速(cmd_vx=0.15、
`WbcParams::forward_walk`のデフォルト)で比較。

| # | 構成 | meas_vx | peak\|pitch\| |
|---|---|---|---|
| 1 | `GaitMode::Mpc`(最古の素のSRBD、Trot以外でチューニングされたことは一度もない) | -0.137 | 0.266rad |
| 2 | `GaitMode::FullCentroidal`, `legged_control_parity=false`(D3.3.5aレガシー) | -0.114 | 0.340rad |
| 3 | FullCentroidal + parity, `k_capture=0.05`(デフォルト、まだ§5abの修正前) | -0.140 | 0.283rad |
| 4 | FullCentroidal + parity + `k_capture=0`(§5aoで使った"健全な基準"そのもの) | -0.124 | 0.291rad |

グラフ: `bound_baseline_survey.png`。

**結論**: **4構成すべてが同程度に逆走する**(-0.114〜-0.140、
指令+0.15に対して)。つまりこのセッションでTrot向けに行ってきた
一連のチューニング(`legged_control_parity`、`k_capture=0`等)は
**無罪**——それらを一切使わない最古の`GaitMode::Mpc`ですら同じ
逆走が起きている。原因はTrot固有の調整とBoundの相性問題ではなく、
**Bound自体(あるいはBoundのデフォルトgait parameter、または
footstep plannerがBoundの前後反対位相パターンをどう扱うか)に
根ざした、より基本的な問題**と考えられる。

もう一つの手がかり: **全構成で一貫して非常に大きなピッチ振動
(0.266〜0.340rad ≈ 15〜19.5°)** が観測された。Boundは元々
「bunny hop」的な前後非対称荷重で自然にピッチが大きくなる歩容
だが、この振幅は疑わしいレベルであり、これが逆走の原因(あるいは
症状)である可能性が高い——ピッチが大きく振れることで、実効的な
重心の前後移動が指令方向と逆になっている(斜面を転がるボールの
ような効果)のではないか、という仮説。ただし未検証。

**今後の課題**: (a) `swing_height_m`(Bound既定0.05m、Trotの
0.04mよりわずかに高い)や`cycle_period_s`(0.3s、Trotの0.4sより
速い)を落とした、より穏やかなBound設定でピッチ振動が収まるかを
確認する、(b) cmd_vx=0(静止)でBoundが単に姿勢を保持できるかを
先に確認し、動的な問題か静的な問題かを切り分ける、(c) footstep
plannerのRaibert項/capture point計算が、Trotの対角ペア前提とは
異なるBoundの前後ペア反対位相構造に対して符号やタイミングを
正しく扱えているか、コードレベルで確認する。

ユーザーから、動画による目視確認の要望あり。§5aoの逆走ケース
(config #4、cmd_vx=0.15)をCSVトレース経由でMuJoCo実メッシュ動画
化(`go2_bound_reversal.mp4`)。

### 5aq. swing_height_m引き下げで逆走解消 — ピッチ振動が主因と確定(2026-07-19)

§5apの「今後の課題」(a)を実施。`GaitConfig::bound()`の素の
timing/sizing(`cycle_period_s=0.30s`、Trotの0.4sより速い;
`swing_height_m=0.05`、Trotの0.04mよりやや高い)を穏やかにすると
ピッチ振動・逆走が改善するか、`go2_wbc_bound_gentler_parameters_
sweep`で検証。健全な基準(`legged_control_parity=true, k_capture=0`)
とcmd_vx=0.15は固定し、`cycle_period_s`/`swing_height_m`のみ4通り。

| # | 構成 | meas_vx | peak\|pitch\| |
|---|---|---|---|
| A | Bound既定(cycle=0.30, swing=0.05) | -0.124 | 0.291rad |
| B | 周期を遅く(cycle=0.40, swing=0.05) | **-0.157(悪化)** | **0.448rad(悪化)** |
| C | 遊脚高さを下げる(cycle=0.30, swing=0.02) | **+0.007(逆走解消)** | **0.067rad(4割弱に低減)** |
| D | 両方(cycle=0.40, swing=0.02) | -0.083 | 0.134rad |

グラフ: `bound_gentler_sweep.png`。動画:
`go2_bound_low_swing.mp4`(config Cのトレース、`go2_bound_reversal.mp4`
と同じcmd_vx=0.15・同じ尺で直接比較可能)。

**結論**: `swing_height_m`(遊脚高さ)が支配的な要因と確定した。
0.05→0.02への引き下げ単独(構成C)で、ピーク|pitch|が0.291→0.067radへ
約4分の1に激減し、**逆走も完全に解消**(meas_vx: -0.124→+0.007——
ただし前進もほぼゼロ、「止まってはいないが実質的に静止」という
段階)。一方、周期を遅くする(構成B)方向はむしろ**悪化**させた
(pitch 0.291→0.448rad、meas_vx -0.124→-0.157)——遊脚が同じ高さの
まま滞空時間が延びることで、着地の衝撃・モーメントがむしろ増した
と考えられる。両方を組み合わせた構成Dは、Cより明確に悪い
(pitch 0.134、meas_vx -0.083)——構成Cで得られた改善に周期変更を
重ねると、その改善の一部を打ち消している。

§5apで観測された「全構成で一貫した大きなピッチ振動」という手がかりは
的中していた。素のBoundプリセットの`swing_height_m=0.05`は、
Go2の~15.6kgの実機質量・FullCentroidalの現在のチューニングに対して
明らかに攻撃的すぎる値であり、Trot向けの調整(§5ab以降積み重ねてきた
一連の作業)とは無関係に、**Bound自体のgait parameter選定の問題
だった**と言える。

**今後の課題**: (a) 構成C(swing=0.02)はもはや逆走しないが、
前進もほぼしていない(meas_vx≈0.007)——`max_step_length_m`
(Bound既定0.12m)やfootstep plannerのゲインを、この低swing設定の
上でさらに調整すれば、実際に前進する健全なBoundが得られる見込みが
高い。(b) swing_height_mをさらに細かく振って(0.02〜0.05の間、
0.01刻みなど)、逆走がゼロ交差する正確な閾値と、前進速度が最大化
される点を特定する。(c) この知見はCanter/Gallopにも直接応用できる
——両方ともBoundよりさらに極端な前後非対称・高い遊脚軌道を持つ
可能性があるため、遊脚高さを保守的に(Go2の脚長~0.426mに対して
十分小さく)設定することが、それらの歩容でも同様に重要になると
予想される。

### 5ar. swing_height_m=0.02は「解決」ではなく「並進の消失」だった(2026-07-19)

§5aqの今後の課題(a)(b)を実施。まず`max_step_length_m`
(Bound既定0.12m)を0.08/0.12/0.16/0.20で振ったが、`swing_height_m
=0.02`固定・cmd_vx=0.15固定の下で**4通りすべてが完全に同一の結果**
(dx=0.018m, meas_vx=0.007, peak_pitch=0.067——小数点以下まで一致)
だった。footstep plannerのストライド長クランプは、この速度域では
そもそも律速要因になっていない(クランプに達していない)ことが
確定した。

次に`cmd_vx`自体を0.05〜0.30 m/sで振った(swing_height_m=0.02は
固定)。

| cmd_vx | meas_vx | peak\|pitch\| |
|---|---|---|
| 0.05 | 0.028 | 0.072rad |
| 0.10 | -0.008 | 0.069rad |
| 0.15 | 0.007 | 0.067rad |
| 0.20 | -0.033 | 0.072rad |
| 0.30 | -0.066 | 0.070rad |

グラフ: `bound_low_swing_cmdvx_flat.png`。

**結論(訂正)**: これは前回「逆走は解消したが前進もほぼゼロ、
さらに調整すれば前進する見込み」と報告した内容の、より厳しい
再評価になる。**meas_vxはcmd_vxに対して全く比例していない**——
0.05から0.30まで5倍以上指令を上げても、実測値はノイズレベル
(-0.07〜+0.03)の範囲に留まり、傾向としてはむしろ高速指令ほど
わずかに負に振れている。peak|pitch|もcmd_vxに対して完全にフラット
(0.067〜0.072rad、ほぼ一定)——これは、このピッチ振動がもはや
「前進速度に起因する動的な問題」ではなく、Boundの接地スケジュール
自体が生む**開ループの(指令とは無関係な)固有振動**であることを
示している。

つまり`swing_height_m=0.02`は、Boundを「健全に歩ける歩容」に
修正したのではなく、**遊脚の動きをほぼ無効化することで並進運動
そのものを消し、結果として逆走という症状だけを隠していた**、と
解釈するのがより正確である。ロボットは指令方向に関わらず、ほぼ
その場で足踏みしているだけの状態になっている可能性が高い。

**現状の到達点(正直な評価)**: このセッションでのBound調査は、
(1)飛行相自体は力学的に安全に扱える(§5ao)、(2)Trot向けの
チューニングは無関係で、Bound自体・そのデフォルトgait parameterに
問題がある(§5ap)、(3)`swing_height_m`が支配的な影響を持つ
(§5aq)、という3点までは確実に立証できた。しかし**「実際に前進
するBound」はまだ達成できていない**——`max_step_length_m`は無関係、
`swing_height_m`を下げると振動は収まるが並進も止まる、という
トレードオフの外に出られていない。

**今後の課題(再整理)**: (a) footstep plannerが実際にBoundの各
ステップで前後方向に足を踏み出しているか(生の足位置トレースを
直接確認する)——もし踏み出し自体がほぼゼロなら、`swing_height_m`
以前に、Boundのcapture-point/footstep補正が前後ペア反対位相構造
に対して正しく計算されていない可能性がある。(b) `swing_height_m`
を0.02〜0.05の間でもっと細かく振り、振動を抑えつつ並進も残る
「甘い点」があるか探す(現状は0.05=大振動、0.02=並進消失の2点
しか見ていない)。(c) 一旦Bound自体の深掘りを止め、Canter/Gallopに
必要な非対称duty_factorのアーキテクチャ設計に進む、という選択肢も
再浮上する——Bound自体の完全な健全化には、当初想定より多くの
掘り下げが必要と判明したため。

### 5as. GRF波形の直接観測 + max_normal_force引き上げは無効(2026-07-19)

ユーザーの「Boundとは何か」という確認質問を挟んだのち、「まずBound
の実現を目指しましょう」との方針を受け、2つのExploreエージェントで
コード監査を実施。結論:
- footstep plannerのRaibert/capture-point計算式(`compute_mpc_
  footstep`、4種類の実装すべて)は**脚・歩容に完全に非依存**——
  純粋な前進指令に対しては4脚とも同一の`half`ベクトルが計算される。
  Bound固有のバグはここにはない。
- `legged_control_parity`のk≥1接地スケジュール予測(`full_
  centroidal_controller.rs:795-838`)も、各脚の`phase_offsets()`+
  `duty_factor`から独立に計算されており、対角ペア決め打ちのような
  ハードコードは一切見つからなかった。BoundのオフセットもTrotと
  全く同じ汎用ロジックで正しく消費されている。

つまり計画・スケジューリングのロジック自体には問題がないことが
確定した。次に、実際にMPCが出力するGRF(`Σmpc_f_z`/新設の
`Σmpc_f_x`)を高頻度サンプリング(10tick=0.02秒間隔)で直接観測。

**Bound(swing_height既定0.05、逆走ケース)対Trot比較**(cmd_vx=0.15、
burn-in後):

| | Σmpc_f_x 平均 | Σmpc_f_x 範囲 | Σmpc_f_z 範囲 |
|---|---|---|---|
| Trot | -0.40N | -3.91〜+4.66N | 穏やか |
| Bound | +7.62N | **-173.16〜+200.00N** | **0〜400N**(=`max_normal_force`200N×2脚の上限に度々張り付く) |

BoundのGRFは前後方向・上下方向とも極端に暴れており、`max_normal_force`
の上限(2脚合計400N)に繰り返し貼り付いている瞬間が観測された——
体重(mg≈153N)の2.6倍に相当する垂直反力が要求される場面がある
ということ。これは§5agでTrotに対して「このキャップは一度も
効かない」と確認したのと対照的。

そこで`max_normal_force`を200(既定)→400→800→∞(無制限)へ
引き上げるスイープを実施。

| max_normal_force | meas_vx | peak\|pitch\| |
|---|---|---|
| 200(既定) | -0.124 | 0.291rad |
| 400 | -0.165 | 0.251rad |
| 800 | -0.150 | 0.283rad |
| ∞(無制限) | -0.142 | 0.257rad |

**結論**: 上限を完全に撤廃しても逆走は解消しない(-0.124〜-0.165の
範囲で変わらず)。つまり`max_normal_force`のキャップに時折貼り付く
現象は観測されたものの、それ自体が逆走の**律速要因ではなかった**
——キャップに触れる瞬間は一時的な過渡現象であり、平均的な前進力
不足の直接原因ではないと考えられる。この仮説も棄却。

**現状の到達点の総括**: footstep planner(棄却)、接地スケジュール
ロジック(棄却)、`max_normal_force`(棄却)——コード上明確に切り分け
可能な仮説は軒並み否定された。残る手がかりは2つ:
(1) BoundのGRF波形自体が(上限に関わらず)本質的にカオス的・
不安定であること自体、(2) §5ao以降繰り返し観測されている、
misa-wbcのHoQPソルバが「Infeasible」「MaxIterations」を非常に
高頻度で返している事実(こちらはまだ直接調査していない、MPCとは
別の下流のWBC層)。特に(2)は未着手の有力な残り筋であり、次に
掘るべき対象と考えられる。

### 5at. misa-wbc HoQPソルバの直接調査 + friction_mu引き上げ — 部分的改善に留まる(2026-07-21)

ユーザーから「Boundとは何か」との確認、続いて「まずBoundの実現を
目指しましょう」との方針を受け、misa-wbc本体(`ho_qp.rs`、専用
Exploreエージェントで調査)を直接監査した。

**HoQPの数学的構造**: Kim 2014方式の階層QP(タスク優先度ごとに
零空間へ逐次射影)。各レベルの内部QPは、構成上**必ず`v=0`が実行
可能**(前レベルの結果`x_{k-1}`を維持するだけの解が常に存在)——
つまり理論上は「真の実行不可能性」はほぼ起こり得ない。よって観測
される`Infeasible`/`MaxIterations`は、ソルバの**数値的な条件の
悪さ**(ほぼ平行な制約行、悪条件なSchur補行列)の兆候であり、
論理バグではないと判断される。

**Boundに特有の条件悪化の機序(エージェントの導出)**: Boundの前
ペア(または後ペア)は、体幹フレームでの前後方向モーメントアーム
`r_x`が2脚でほぼ同一。浮遊base方程式のピッチトルク項は
`Σ(r_z·f_x − r_x·f_z)`の形になるため、`r_x`が2脚で共通だと
`f_z`の脚間配分ではピッチトルクを作れず(`r_x·Σf_z`の項に潰れる)、
**ピッチ制御はほぼ`Σf_x`だけに頼らざるを得ない**。ところが
`f_x`は摩擦円錐で`|f_x|≤μ·f_z`に制限される——Trotの対角ペアが
`Δf_z·Δr_x`で"ほぼタダで"ピッチトルクを得られるのと対照的。これは
§5asで実測した「Σf_xが激しく振動し、Σf_zが上限に張り付く」現象の
物理的な"なぜ"を説明する。

**検証**: `friction_mu`(WBC側タスクとFullCentroidal MPC側の両方、
同期して変更)を0.5(既定)から引き上げるスイープを実施
(swing_height_m=既定0.05、逆走ケースそのもの、cmd_vx=0.15)。

| friction_mu | meas_vx | peak\|pitch\| |
|---|---|---|
| 0.5(既定) | -0.124 | 0.291rad |
| 0.7(実際の地面摩擦と一致) | -0.106 | 0.310rad |
| 1.0 | -0.082 | 0.294rad |
| **1.5** | **-0.040(最良)** | 0.291rad |
| 2.0 | -0.111(悪化) | 0.288rad |
| 3.0 | -0.096 | 0.257rad |
| 5.0 | -0.150(既定より悪い) | 0.284rad |

グラフ: `bound_friction_mu_sweep.png`。

**結論**: 0.5→1.5では**単調に改善**(-0.124→-0.040)——このBound
調査全体で初めて見つかった、きれいな単調傾向のあるパラメータ。
仮説を裏付ける有意な証拠と言える。しかし1.5を超えると**再び
非単調に悪化**し(2.0で-0.111へ逆戻り)、**どのμでも符号が反転
(=前進)することはなかった**。peak|pitch|は全域でほぼ一定
(0.257〜0.310rad)——friction_muはピッチ振動そのものを抑える
わけではなく、振動があっても前進力を確保できるかどうかにのみ
効いている、という仮説とも整合する。

つまり、「前後ペア支持ではピッチ制御に摩擦限界のΣf_xを要する」
という機序は実在し、部分的に効くが、**それ単独ではBoundを健全に
歩かせるには至らない**。これまでのセッション全体で繰り返し見た
パターン(パラメータを上げると"甘い点"まで改善し、その先で再び
悪化する)がここでも再現された。

**総括(このセッションでのBound調査全体)**: footstep planner・
接地スケジュール・`max_normal_force`は棄却、`friction_mu`は部分的
改善(だが反転せず)、`swing_height_m`は振動を抑えるが並進も消す
(§5aq/5ar)——複数の実在する物理的・数値的困難が見つかったが、
単純なパラメータ調整だけでは「実際に前進する健全なBound」には
到達できなかった。これは、Boundという歩容が(少なくとも現在の
position-control+WBC+FullCentroidal MPCという組み合わせにおいて)
Trotよりも本質的に難しい制御問題であることを示しており、単発の
パラメータ発見ではなく、より根本的な定式化の見直し(例:前後ペア
支持に特化したコスト関数・タスク優先度の再設計)が必要な可能性が
高い。

### 5au. 実世界のBound実現例(Raibert/MIT Cheetah)から着想した①再テスト — 効果は限定的(2026-07-21)

ユーザーから「世の中にモデルベースでBoundを実現している例は
あるか」との質問。外部検索は許可されていないため、内部知識に
基づき回答(要:一次資料の確認が必要な場合は改めて許可を得る)。

- **Raibertのホッピングマシン/バウンディング四足**(1980〜90年代、
  CMU/MIT Leg Lab): 高さ・速度・**姿勢**を3つの独立ループに分解。
  姿勢制御は**立脚中のhip関節トルクを直接使う**専用チャンネル。
- **MIT Cheetah 2/3**(Sangbae Kim研): MPCベースの高速バウンディング
  だが、姿勢制御は脚の質量・慣性を積極的に使ったインパルス整形/
  直接hipトルクに依存。
- **Hybrid Zero Dynamics**(Poulakakis, Grizzleら): 仮想拘束による
  バウンディング、姿勢制御を接地力配分とは別チャンネルとして扱う。

共通点: 姿勢(ピッチ)制御を、摩擦で制限されるΣf_x(接地力配分)
とは**独立したチャンネル**として持たせている。これは、このセッション
序盤で実装した①`true_centroidal_coupling`(misartaのCRBAベースの
重心運動量結合、脚の関節加速度が体幹運動に及ぼす反作用をMPCの
ダイナミクスに組み込む機能)と概念的に対応する——§5aeでTrotに対して
「ほぼ中立」と評価されていたが、対角ペアはΔf_z·Δr_xで元々ピッチ
トルクを安く得られるため、この追加チャンネルの恩恵が測定しづらかった
だけの可能性がある。Boundはまさにこのチャンネルが必要になる条件
(Σf_xが摩擦で干上がっている)なので、再テストした。

`go2_wbc_bound_true_coupling_sweep`(Bound既定のswing_height_m=0.05・
逆走ケースそのもの、cmd_vx=0.15):

| # | 構成 | meas_vx |
|---|---|---|
| A | 基準(①オフ、mu=0.5) | -0.124 |
| B | ①オン、mu=0.5 | -0.116(わずかな改善のみ) |
| C | ①オン、mu=1.5(§5at最良値と併用) | **-0.061(mu=1.5単独の-0.040より悪化)** |

グラフ: `bound_true_coupling_sweep.png`。

**結論**: ①単独の効果はごくわずか(-0.124→-0.116、6%程度の改善)
——§5atのfriction_mu(-0.124→-0.040、68%の改善)と比べると桁違いに
小さい。さらにfriction_mu=1.5と併用すると、①なしのmu=1.5単独より
**悪化**した(-0.040→-0.061)。文献のヒントから期待した「独立した
ピッチトルクチャンネル」としての効果は、ほぼ確認できなかった。

**なぜ効かなかったと考えられるか**: 実世界の制御(Raibert/MIT
Cheetah)における「hip関節トルクによる姿勢制御」は、姿勢誤差から
目標hipトルクを直接計算する**明示的なフィードバック制御則**
(既存の`base_accel`タスクに類する、しかし関節トルク権限を直接
使うもの)である。一方、我々の①`true_centroidal_coupling`は、
MPCが内部で使う**線形化ダイナミクスモデルの精度を上げるための
補正項**(joint_qの加速度参照からの受動的なバイアス項)に過ぎず、
「ピッチを能動的に補正する制御則」ではない。つまり文献が実際に
使っている仕組みと、我々の①が実装している仕組みは、名前は似て
いても本質的に異なるものだった、というのがより正確な総括。

**今後の選択肢**: (a) 真に文献同様の「hipトルク直接姿勢制御タスク」
を新規のWBCタスクとして設計・実装する(quadruped-gaitのwbc/tasks/
に新規モジュールを追加する規模の、本格的なエンジニアリング作業)、
(b) friction_muの微調整域(1.2〜1.8付近)をさらに細かく探る、
(c) Bound自体の深掘りをここで一旦区切り、Canter/Gallopに必要な
非対称duty_factorのアーキテクチャ設計に進む。(a)は工数が大きく、
(b)はこれまでのパターン(甘い点はあるが反転しない)から大きな
突破口は期待しにくい。

ユーザーから「(a)で既存文献を再現しよう」との方針。

### 5av. 明示的なピッチPDフィードバックを実装 — それでも効果なし(2026-07-21)

§5auの分析(Raibert/MIT Cheetahは姿勢制御を明示的なクローズド
ループ・フィードバックとして持つ)を、より忠実に再現する実装を
行った。`WbcPipeline::solve`の`a_base_des`角加速度成分は、これ
まで**完全にMPCの最適化GRFからのフィードフォワードのみ**
(`α = I⁻¹·(Σr×f − ω×Iω)`)で、測定したピッチ誤差への直接
フィードバックが一切なかった(コード内コメントにも明記: 過去に
存在した手動チューニングPDは、MPCとの整合性のため意図的に除去
されていた)。

`WbcPipeline`に新規`pitch_pd_gain: (f64, f64)`フィールドを追加し
(既定`(0.0, 0.0)`=完全な無効化、全既存歩容の挙動は不変)、
`a_ang_body.y += kp·(0 − pitch_meas) − kd·pitch_rate`を、MPC由来の
フィードフォワードの**上に直接加算**する形で実装(`src/wbc_pipeline.rs`)。
Bound逆走ケース(swing_height_m既定0.05、cmd_vx=0.15)でゲインを
スイープ。

| # | pitch_pd_gain | meas_vx | peak\|pitch\| |
|---|---|---|---|
| A | オフ(基準) | -0.124 | 0.291rad |
| B | (50, 5) | -0.128 | 0.280rad |
| C | (100, 10) | -0.113 | 0.307rad |
| D | (200, 20) | -0.135 | 0.247rad |
| E | (400, 40) | -0.121 | 0.295rad |

グラフ: `bound_pitch_pd_sweep.png`。

**結論**: どのゲインでも基準(-0.124)とほぼ同じ範囲(-0.113〜-0.135)
に留まり、meas_vx・peak_pitchとも**有意な傾向が全く見られなかった**
——friction_muの時のような単調な改善すら見られない、実質的に
無効という結果。文献を最も忠実に再現したはずの実装が、期待した
効果を全く生まなかった。

**なぜ効かなかったか(物理的な解釈)**: `a_base_des`はWBCの
優先度1(ソフト)タスクであり、優先度0(ハード制約)の
`floating_base_eom`・`friction_cone`・`no_contact_motion`が許す
実行可能領域の**内側でしか**実現できない。つまり「もっと強くピッチ
補正してほしい」というソフトな目標をいくら強めても、ハード制約
(摩擦円錐によるΣf_xの物理的上限)がそもそも許容していない加速度は
実現不可能——ソフトタスクの重み付けを変えても、実在しない力の
"予算"を生み出すことはできない。§5atのfriction_muが(部分的にでも)
効いたのは、それが**ハード制約そのものを緩和した**(実在する物理的
上限を引き上げた)からであり、§5avのpitch_pd_gainは単に**すでに
制約された実行可能領域内で目標を変えただけ**だったから、という
説明が最も筋が通る。

つまり実世界のRaibert/MIT Cheetahが持つ「姿勢の独立チャンネル」は、
単なる制御則の書き方の違いではなく、**実機が実際にその制御を実現
できるだけの物理的な力/トルク予算(モーター出力・摩擦係数など)を
持っている**ことが前提だった可能性が高い。ソフトウェア側の工夫
(制御則の再現)だけでは、シミュレーション上のGo2・現在のアクチュ
エータ/摩擦設定というハードウェア制約の中では、この限界を超えられ
なかった。

**このセッションでのBound調査の最終総括(§5av時点)**: footstep
planner・接地スケジュール・`max_normal_force`・①`true_centroidal_
coupling`・明示的なピッチPD——調べた仮説はすべて、原因ではないか、
部分的(かつ反転しない)効果に留まるか、のいずれかだった。唯一の
実質的な手がかりは`friction_mu`(ハード制約自体の緩和)であり、
これも単独では不十分。

### 5aw. 「律速要因は摩擦かトルクか」— 既存データの再分析でトルク飽和を発見(2026-07-21)

ユーザーから「一番の律速要因は摩擦か、関節の出力か」との質問。
新規シミュレーションを走らせる前に、§5asですでに収集済みだった
高頻度サンプリングログ(`Σmpc_f_x`診断)を再分析したところ、
これまで提示していなかった`max|τ|`(WBCが実際にMuJoCoへ送った
最大関節トルク)のデータが見つかった。

Go2の実機トルク上限(`robot.joints[ji].effort`由来、WBCの
`torque_limits`ハード制約に正しく反映): ヒップ/大腿=23.7N·m、
下腿=45.43N·m。

| | 最大トルク実測値 | 23.7N·m超過回数 |
|---|---|---|
| Trot | 17.29 N·m | 0/248 (0%) |
| **Bound** | **44.71 N·m** | **25/199 (12.5%)** |

Boundは実機のヒップ/大腿トルク上限を12.5%の記録で超過(最大で
上限の約1.9倍)。Trotは一度も超過しない。`bake_actuator_limits`が
既定でtrueのため、MuJoCoはこの超過分を物理的にクリップする——
つまりWBCが「この解を出力した」と思っている状態と、実際にロボ
ットに加わる力が食い違っている。これは§5at/5auで見た「HoQPが
Infeasible/MaxIterationsを頻発する」現象と直結する: ソルバが
収束に失敗して返す解が、自らのハード制約(トルク上限)を破って
いる、ということ。

**回答**: 摩擦かトルクかの二択ではなく、**両方とも「症状」で
根本原因は同じ**(Boundの前後ペア支持がQPを数値的に悪条件化させ、
ソルバが破綻した解を返す)。摩擦(friction_mu)は部分的に本物の
効果があった一方、関節トルクは今回新たに判明した、より直接的で
大きな超過(1.9倍)が見つかった。

ユーザーからの提案を受け、実際のシミュレーションモデル上で両方を
検証することに。

### 5ax. アクチュエータ出力の実引き上げ + 実地面摩擦の一致検証(2026-07-21)

**アクチュエータ出力**: `robot.joints[*].effort`を1.0/2.0/5.0倍に
スケール(MJCF出力・WBCの`torque_max`の両方に反映、実際にモーターを
強くする実験)。Bound逆走ケース(swing_height_m既定0.05、cmd_vx=0.15)。

| effort_scale | meas_vx | peak\|pitch\| | peak\|roll\| |
|---|---|---|---|
| 1.0(実機相当) | -0.124 | 0.291rad | 0.000rad |
| **2.0** | **-0.056(改善)** | 0.386rad(悪化) | 0.001rad |
| 5.0 | -0.090(2.0より悪化) | 0.298rad | **0.018rad(新たなロール不安定)** |

**実地面摩擦の一致検証**: §5atは`friction_mu`(WBCの想定)を最大5.0
まで上げたが、実際の地面摩擦(MJCFの`default_friction`)は0.7の
ままだった——WBCが実際より多い grip を想定していた可能性がある。
そこで実地面摩擦も同じ値に引き上げて一致させた場合を比較。

| 構成 | meas_vx |
|---|---|
| A. mu=1.5(想定)、ground=0.7(実際、§5atと同じ不一致) | -0.040 |
| B. mu=1.5(想定)、ground=1.5(一致) | **-0.040(全く同じ)** |
| C. mu=3.0(想定)、ground=3.0(一致) | -0.096(§5atのmu=3.0単体と同一) |

グラフ: `bound_effort_ground_friction.png`。

**結論**:
- **アクチュエータ出力は本物の(部分的な)レバー**。2倍で明確な
  改善(-0.124→-0.056)——ただしfriction_muと同じパターンで、
  5倍ではむしろ悪化し、しかも新たにロール不安定という副作用が
  出現した。反転(前進)には至らない。
- **実地面摩擦は完全に無関係だった**。WBCの想定(friction_mu)を
  実地面摩擦と一致させても、mu=0.7のまま(§5atの不一致設定)と
  **1ミリも変わらない**結果になった。つまりBoundの各ステップでは
  そもそも実地面の摩擦限界(すべり)には一度も達していない——
  friction_muの部分的効果は、地面のグリップを実際に多く使える
  ようになったからではなく、**QP内部の摩擦円錐の形状が変わり、
  ソルバが選ぶ解自体が変わる、という純粋にソルバ内部の数値効果**
  だったことが、この一致実験によって確定した。

これは§5at/5aw/5axを通じて一貫している結論——**Boundの根本問題は
物理的な力・摩擦の予算不足ではなく、前後ペア支持がQPを数値的に
悪条件化させることそのもの**——をさらに補強する。アクチュエータ
出力(本物のハード制約)は緩和すると部分的に効くが、摩擦(地面の
物理的な実体)は最初から律速要因ではなかった。

**このセッションでのBound調査の最終総括(更新)**: footstep
planner・接地スケジュール・`max_normal_force`・①`true_centroidal_
coupling`・明示的なピッチPD・実地面摩擦——調べた仮説はすべて、
原因ではないか、部分的(かつ反転しない)効果に留まるか、のいずれか
だった。実質的な効果があった手がかりは`friction_mu`(ソルバ内部の
数値効果)と`actuator_effort_scale`(本物のハード制約緩和)の2つ
だが、どちらも単独では不十分。残る現実的な選択肢は、(1)前後ペア
支持に特化したコスト関数・タスク優先度・footstep timingの根本的な
再設計(QPの数値的な悪条件そのものに手を入れる、大工数)、
(2)Bound自体をここで一旦保留し、Canter/Gallopに必要な非対称
duty_factorのアーキテクチャ設計に進む、の2つ。

### 5ay. cmd_vxの瞬時ステップ vs なだらかなランプ — 遷移タイミングは無関係と判明(2026-07-21)

ユーザーから、動物の歩容開始に関する洞察: 動物はいきなり定常的な
Bound歩容で動き出すわけではなく、屈み込み(タメ)や予備的姿勢調整
(anticipatory postural adjustments)を伴う過渡的な移行を経る。
これまでの全テストは、静止姿勢から定常Boundの接地スケジュールへ、
`cmd_vx`を**1tickで瞬時にステップ**させていた——まさに動物が避ける
「コールドスタート」そのもの。この遷移の瞬間こそが、HoQPの数値的
悪条件(§5at/5aw/5ax)を最も誘発しやすいのではないか、という仮説。

`WbcParams::cmd_vx_ramp_s`を新設し、`cmd_vx`を瞬時ステップの代わりに
指定秒数かけて線形に立ち上げる機能を実装。`go2_wbc_bound_cmd_vx_
ramp_sweep`で、ランプ時間0.0(瞬時ステップ、基準)/0.5/1.0/2.0秒を
比較(Bound既定のswing_height_m=0.05、legged_control_parity=true、
k_capture=0)。**公平な比較のため、測定窓はランプ完了後の定常状態
のみ**(`[burn_in_s + ramp_s, total_time_s]`)に限定。

| ramp_s | 定常状態meas_vx | 定常状態peak\|pitch\| |
|---|---|---|
| 0.0(瞬時ステップ、基準) | -0.116 | 0.291rad |
| 0.5 | -0.147(悪化) | 0.288rad |
| 1.0 | -0.164(さらに悪化) | 0.265rad |
| 2.0 | -0.116(基準と同程度) | 0.283rad |

グラフ: `bound_cmd_vx_ramp_sweep.png`。

**結論**: なだらかな立ち上げは**逆走を改善しない**——中程度の
ランプ(0.5〜1.0秒)はむしろ基準より悪化し、最も長いランプ(2.0秒)
でようやく基準と同程度に戻る、という結果だった。しかも
peak|pitch|は全ランプ時間でほぼ一定(0.265〜0.291rad)——遷移の
タイミングや速さに関わらず、ピッチ振動の大きさは変わらない。

これは、動物由来の洞察(過渡的な遷移が問題の一因ではないか)を
明確に**否定する**結果である。ピッチ振動・逆走は、cmd変化の瞬間に
トリガーされる一過性の現象ではなく、**Bound歩容そのものが持つ、
定常状態でも継続する固有の(開ループ的な)性質**であることが、
この実験によってはっきりした。つまり問題は「遷移の設計」ではなく、
「Bound自体の定常運転」に内在する——§5at以降積み上げてきた
「前後ペア支持がQPを恒常的に悪条件化させる」という結論と完全に
整合する。

これで、遷移タイミング(本セクション)・footstep planner・接地
スケジュール・`max_normal_force`・①・明示的ピッチPD・実地面摩擦——
調べた仮説はすべて棄却されたか部分的効果に留まった。残る現実的な
選択肢は変わらず、(1)QPの数値的悪条件そのものに手を入れる根本
的な再設計、(2)Bound自体をここで一旦保留しCanter/Gallopへ進む、
の2つ。

ユーザーは(1)を選択。

### 5az. GRF平滑化+ウォームスタート抑制 — ソルバ収束は劇的改善するが追従は変わらず(2026-07-21)

本格的なソルバ内部(`ho_qp.rs`)の改修に入る前に、§5atの外部監査が
「まず試すべき」と明示していた、**既存の・コード変更不要な2つの
ノブ**を検証: `WbcPipeline::grf_smoothing_alpha`(既定1.0=平滑化
なし、MPCの生のGRFをそのまま`contact_force`タスクの目標にする)と
`qp_prox_weight`(既定1e-4、前tickの解へのウォームスタート・アン
カー、0.0で完全なコールドソルブ)。BoundのGRFはtick間で激しく
振動する(§5as)ため、この生のGRF目標+前tickの(全く違う)解への
アンカーが、HoQPに「動く目標」と「食い違ったウォームスタート
シード」を同時に押し付け、数値的悪条件を助長しているのではないか
という仮説。

`go2_wbc_bound_grf_smoothing_and_prox_sweep`(Bound既定の
swing_height_m=0.05・逆走ケース、cmd_vx=0.15)。HoQPの
`Infeasible`/`MaxIterations`警告回数も同時に集計。

| # | 構成 | 警告回数(2.5秒間) | meas_vx |
|---|---|---|---|
| A | 基準(alpha=1.0, prox=1e-4) | 642 | -0.124 |
| B | GRF平滑化(alpha=0.3, prox=1e-4) | 637(ほぼ不変) | -0.104 |
| **C** | **コールドソルブ(alpha=1.0, prox=0.0)** | **74(約88%減)** | -0.120(ほぼ不変) |
| D | 併用(alpha=0.3, prox=0.0) | 99(約85%減) | -0.097 |

グラフ: `bound_smoothing_prox_sweep.png`。

**結論**: ウォームスタートを無効化(コールドソルブ)すると、ソルバの
収束失敗警告が**劇的に(85〜88%)減少**した——外部監査の仮説
(前tickの食い違ったシードが悪条件を助長している)は正しかった。
しかし**meas_vxはほとんど変わらなかった**(-0.124→-0.120、
ほぼ誤差範囲)。GRF平滑化単独(構成B)は警告数をほとんど減らさ
なかったが、追従はわずかに改善した(-0.104)——2つの効果は独立
していて、互いに説明し合わない。

これは非常に重要な切り分けになった。**ソルバの数値的収束を劇的に
改善しても、Boundの逆走はほとんど解消しない**——つまり、これまで
「QPの悪条件そのものが逆走の原因」と考えてきたが、より正確には
**ソルバの悪条件は実在する問題ではあるが、逆走の主因ではない**、
ということが判明した。ソルバが完璧に収束していても、Boundの
前後ペア支持という**幾何学的な制約**(§5at/5auで導出した、ピッチ
トルクがΣf_xという摩擦限界のあるチャンネルにほぼ全面的に依存する
という物理的事実)自体は変わらないため、追従は改善しない。

**このセッションでの最終的な理解**: Boundが歩けない根本原因は、
ソルバの数値的な問題ではなく、**前後ペア支持という幾何学的配置が、
安価なピッチトルク生成経路(対角ペアのΔf_z·Δr_xのような)を
構造的に持たない**という、より深いレベルの物理的制約だった。
この制約を回避する現実的な方法は2つ考えられる: (a) WBCの
コスト・タスク定式化そのものを、前後ペア支持向けに新設計する
(摩擦限界を超えない範囲でピッチトルクを生む、全く新しい仕組みを
考案する必要があり、大規模なエンジニアリングになる)、(b) **Canter
のように、前後ペア内の左右の脚位相を意図的にわずかにずらし、
純粋な前後ペア支持という退化した幾何配置そのものを避ける**——
これは奇しくも、当初のCanter/Gallopへの分岐点に戻ってくる結論
であり、Boundを無理にWBCで押し通すより、Canterの自然な脚位相
構造の方が、この根本問題を最初から回避できる可能性が高い。

### 5ba. 「必要な運動量は把握しているか」— WBCの質量・慣性プレースホルダー誤りを発見、しかし解決には至らず(2026-07-21)

ユーザーから「Boundの跳躍に必要な運動量などは把握しているか」との
質問。これまでの調査は完全にempirical(パラメータを振って結果を
観測する)であり、第一原理的な運動量・トルク予算の検証は一度も
行っていなかった。これを厳密にやろうとした過程で、`WbcPipeline`が
`a_base_des`(最優先度のソフトタスク、重み200)をNewton-Euler
(`a_lin=Σf/m+g`、`a_ang=I⁻¹·(Σr×f−ω×Iω)`)で算出する際に使う
`mass_kg`/`inertia_diag_body`が、**一度もGo2の実機値に同期されて
いない**ことが判明した。

`go2_diag_wbc_mass_inertia_mismatch`(MuJoCo不要): `WbcPipeline::
new()`の既定値(`mass_kg=9.0`、`inertia_diag_body=(0.070,0.260,0.242)`
——コード内コメントに「Cheetah-class」のプレースホルダーと明記)と、
`articara::gait::auto_detect_srbd_mpc_config`(URDFから実際に検出
する、MPC側では既に使われている値)を比較。

| | プレースホルダー | 実際のGo2 | 比率 |
|---|---|---|---|
| 質量 | 9.00 kg | **15.606 kg** | 1.734倍 |
| ピッチ慣性(I_yy) | 0.260 kg&middot;m² | **0.0981 kg&middot;m²** | **0.377倍**(プレースホルダーが2.65倍も過大) |

質量が42%軽く、ピッチ慣性が165%も過大という二重の誤りがあり、
これは本セッションを通じてFullCentroidal+WBCを使った**全ての**
テスト(Trot・Bound問わず)に影響していた可能性がある。

`go2_wbc_mass_inertia_fix_sweep`で、この値を実機値に同期する修正
(`sync_real_mass_inertia`)を実装し検証。

| | placeholder(既定) | 実機値に修正 |
|---|---|---|
| Bound(cmd_vx=0.15) meas_vx | -0.124 | -0.129(ほぼ不変) |
| Trot(cmd_vx=0.15) meas_vx | 0.114 | 0.121(わずかに改善) |
| Trot peak\|roll\| | 0.026rad | **0.061rad(2.3倍に悪化)** |

グラフ: `bound_mass_inertia_fix.png`。

**結論**: 発見自体は本物のバグ(実機と全く違うパラメータを使って
いる)だが、**Boundの逆走解決には全く寄与しなかった**。Trotでは
追従がわずかに改善する一方、ロール方向の不安定が2倍以上に悪化する
という新たな副作用が出た。つまりこの質量・慣性の誤りは、実在する
コード品質の問題として修正する価値はあるが、**Boundが歩けない
根本原因ではない**——これは§5azで確立した「幾何学的な制約
(前後ペア支持がピッチトルクの安価な経路を持たない)」という結論を
覆すものではなく、むしろ独立した、別の問題として切り分けられた
形になる。

`sync_real_mass_inertia`はテストハーネスにオプトインの切り替えと
して残したが、Trotのロール副作用が未解決のため、既定を変更する
(常時有効化する)のは時期尚早と判断し見送った。

### 5bb. 第一原理モデル(SRBD周期トリム解)による再設計 — Phase 0: 実行可能性確認(2026-07-21)

ユーザーから、これ以上のempiricalな探索ではなく第一原理での
アプローチの提案: (1)質点/少数質点モデルでBoundの理想挙動を
モデル化、(2)必要な運動量等の時系列目標を導出、(3)それを実現
する関節力・速度・摩擦の制約を整理してWBCの目標・制約に与える。
Explore+Planエージェントによる調査を経て、`/home/kasai/.claude/
plans/splendid-chasing-lollipop.md`として計画を承認済み。

**モデル選択**: 質点2つのダンベル型ではなく単一剛体(SRBD、質量+
慣性)。理由: 実慣性`I_yy=0.0981kg·m²`から逆算した等価質点間隔
`ℓ=√(I_yy/m)≈0.079m`は、実際のヒップ間隔(前後方向オフセット
r_x≈0.19m)よりずっと小さい——Go2の質量は重心近くに集中しており、
「ヒップに質点」というダンベルモデルは慣性を約5倍過大評価する。
既存のMPC/WBCが使うSRBD表現をそのまま使うのが正しい。

**前後対称性による半周期BVP**: Boundの前ペア区間`[0,T/2)`と後ペア
区間`[T/2,T)`で、`f_x^B(s)=-f_x^A(s)`, `f_z^B(s)=f_z^A(s)`という
対称アンザッツを置くと周期条件が自動的に閉じ、前半周期だけを解けば
よい。区分的定数GRFを仮定した閉形式解:
- 高さ: `F_z≡m·g`(一定)で`ż`の周期性が閉じる ⇒ **理想軌道の
  体幹高さは完全に平坦**(duty=0.5のBoundには滞空期がなく、
  上下動は構造的に不要かつ不可能)。
- ピッチ: `θ(0)=0`、`θ_peak=|α_p|·T_st²/8`、
  `α_p=(-h0·F_x-r_x·m·g)/I_yy`。
- トリム解(ピッチトルクをゼロにする): `F_x*=-r_x·m·g/h0`。
- 摩擦の必要条件: `μ_needed=|F_x*|/(m·g)=r_x/h0`。

`go2_diag_bound_trim_model_feasibility`で、実機の自動検出済み
質量・慣性・脚形状(手打ちなし、`auto_detect_srbd_mpc_config`/
`auto_detect_kinematics_config`より)を使って評価:

```
実パラメータ: m=15.606kg, I_yy=0.0981kg·m², r_x_front=0.1922m,
              r_x_rear=0.1946m, h0=0.2664m, T_st=0.150s
F_z=153.10N, F_x*=-110.44N, μ_needed=0.721
```

| F_x | theta_peak |
|---|---|
| 0(スラストなし) | 0.844 rad (48.3°) |
| F_x*(完全トリム) | 0.000 rad |
| clip(μ=0.5) | 0.259 rad (14.8°) |
| **clip(μ=0.7、実地面)** | **0.025 rad (1.43°)** |
| clip(μ=1.5) | 0.000 rad |

必要関節トルク(前ペア1脚あたり、`foot_jacobian_body`で算出):
μ=0.7クリップで`max|τ|=14.37 N·m`(実機上限23.7N·mに対し約40%の
余裕)。

**結論(Go/No-Go)**: **GO**。μ_needed=0.721は実地面摩擦0.7を
わずかに(約3%)上回り完全なトリムには届かないが、摩擦でクリップ
された現実的な解でもtheta_peak=1.43°——**現状のカオス的な実測値
(約16.6°)の1/11以下**。トルクにも十分な余裕がある(必要14.4N·mに
対し上限23.7N·m)。パイプライン統合(Phase b)に進む。

### 5bc. Phase b/c: パイプライン統合と実機検証 — 符号バグは見つかるも、核心的な改善には至らず(2026-07-21)

**Phase b(実装)**: `quadruped-gait/src/bound_reference.rs`に
`BoundTrimConfig`/`BoundTrimSample`を新設(§5bbの閉形式解の純粋
関数実装、7つの単体テストで自己整合性を検証——境界条件
`θ(0)=0`、前後対称性`θ_B(s)=-θ_A(s)`、ピーク振幅の一致など)。

これを2箇所に配線:
1. `full_centroidal_controller.rs::build_full_centroidal_inputs`
   (L995-1074付近)——MPCの各ホライズンステップ参照に、既存の
   `stance_sub_fractions`/`contact.is_stance`から復元した周期位相を
   与え、`sk.base_euler_zyx.y`(ピッチ参照)と`grfs[leg].x`
   (これまで一度も設定されたことがなかった水平GRF参照)を設定。
   `GaitType::Bound`かつ新設フラグ`enable_bound_trim_reference`が
   trueの場合のみ有効(他歩容は無変更)。
2. `wbc_pipeline.rs`——`pitch_pd_gain`/`roll_pd_gain`の目標を、
   これまでの定数ゼロから、新設`pitch_ref`/`roll_ref`フィールド
   (毎tick、ホストが設定)に変更。既存の全呼び出し元は0.0を渡す
   ため後方互換。

テストハーネス側で`sync_real_mass_inertia`(§5ba)と連動する
`bound_trim_reference: Option<(f64,f64)>`(WBC側ピッチPDゲイン)を
新設。全ての回帰テスト(quadruped-gait 185+2+1、articara 7)が
変更後も成功。

**Phase c(初回検証、修正前)**: `go2_wbc_bound_template_reference_
forward_walk`で、基準・参照ありMPCのみ・参照+WBC PD(2段階)を
比較。

| 構成 | meas_vx | peak_pitch | peak_roll |
|---|---|---|---|
| A. 基準(参照なし) | -0.124 | 0.291rad | 0.000rad |
| B. 参照あり(MPCのみ) | -0.078 | **0.336(悪化)** | **0.075(新出!)** |
| C. 参照+WBC PD(100,10) | -0.130 | 0.316 | 0.028 |
| D. 参照+WBC PD(200,20) | -0.112 | 0.312 | **0.173(大幅悪化)** |

理論値(theta_peak=0.025rad)には全く近づかず、むしろ悪化。しかも
これまで一度も出たことのなかった**ロール方向の不安定が新たに出現**
——実装に問題がある強い兆候。

**位相チェック診断**: `pitch_ref`(目標)と実測ピッチを密にサンプリング
して直接比較したところ、**参照がピークに達する瞬間、実測ピッチは
ほぼ底**(例: t=0.58sでref=+0.336、t=0.62sでmeas=-0.205)——単なる
遅れではなく、**符号が反転している**明確なパターンが見つかった。

**原因と修正**: `bound_reference.rs`内で独自に決めた「前ペア支持=
正のピッチ」という符号規約が、実際のシステム(`euler_angles()`の
ピッチ符号)と整合していなかった。`BoundTrimConfig`に`sign: f64`
フィールドを新設し、両配線箇所で`sign: -1.0`(経験的に確認した
正しい値)を設定。

**Phase c(符号修正後の再検証)**:

| 構成 | meas_vx | peak_pitch | peak_roll |
|---|---|---|---|
| A. 基準 | -0.124 | 0.291rad | 0.000rad |
| B'. 参照あり(MPCのみ、符号修正後) | -0.148 | 0.250 | 0.035(改善) |
| C'. 参照+WBC PD(100,10)、符号修正後 | -0.124(基準と同一) | 0.285 | **0.006(ほぼ解消)** |
| D'. 参照+WBC PD(200,20)、符号修正後 | -0.132 | 0.290 | **0.009(ほぼ解消)** |

グラフ: `bound_template_reference_validation.png`。

**結論**: 符号修正により、**新たに出現したロール不安定はほぼ
解消した**(0.173→0.009)——符号バグの診断・修正自体は正しかった
ことの裏付け。しかし**核心的な指標(meas_vx、peak_pitch)は、
基準とほとんど変わらないまま**——理論値(theta_peak=0.025rad)には
全く届かず、逆走も解消しなかった。

つまり、第一原理で導出した参照そのものは(符号を直せば)システムを
悪化させはしないが、**実際にその参照を追従させる効果がほとんど
出ていない**。考えられる理由: (a) Boundの周期(0.3秒)に対して、
MPCの再計算間隔・WBCの解の質・アクチュエータ応答を含む実際の
閉ループが、位相を保ったまま追従できるだけの帯域を持っていない
(遅れが周期に対して無視できない割合を占める)、(b) MPCの状態コスト
`q_diag`のピッチ項の重みが、GRFのカオス的な振る舞いに対して参照を
"効かせる"には弱すぎる、(c) まだ見つかっていない別のバグ、のいずれか。

**このセッションでのBound調査の最終総括**: footstep planner・
接地スケジュール・`max_normal_force`・①・明示的ピッチPD(定数
ゼロ目標)・実地面摩擦・cmd_vxランプ・ウォームスタート抑制・
質量慣性の誤り・そして今回の第一原理モデルに基づく時変参照——
これだけ徹底的に調べても、Boundを健全に前進させる決定打には
至らなかった。得られた確実な知見は、(1)前後ペア支持は幾何学的に
安価なピッチトルク経路を持たない(§5at)、(2)ソルバの数値的
悪条件は実在するが逆走の主因ではない(§5az)、(3)第一原理で
導出した理論上の理想解自体は妥当(Phase 0のGo判定)だが、実際の
閉ループでそれを追従させることは別の、まだ解決していない問題で
ある、ということ。

### 5bd. 制御ループ抜きの質点シミュレーションで核心的な発見 — トリム戦略自体に内在する巨大な速度振動(2026-07-19/20)

ユーザーから「動画でジャンプ時に足が後ろに行っている」という視覚的
指摘。続けて「まず制御の実態はおいて、X-Z平面上の単一剛体(質点+
ピッチ)モデルでBody挙動を再現してから考えたい」との提案。WBC/
MuJoCoを一切介さず、§5bbの力のスケジュール(前ペア支持中
`F_x=F_x_trim`一定、後ペア支持中`F_x=-F_x_trim`、`F_z=mg`常に一定)
だけをNewton-Euler方程式で純粋数値積分(`ref/scripts/simulate_
point_mass_bound.py`、実機値: m=15.606kg, I_yy=0.0981kg·m²,
r_x=0.1922m, h0=0.2664m, T=0.30s, duty=0.5)。

**確認できたこと**:
- 高さ: 理論通り完全に平坦(z̈≡0)
- ピッチ: 完全トリム(F_x=F_x_trim)で厳密にゼロのまま一定
  (θ̈=0が全区間で成立)。μ=0.7クリップでも±1.3〜1.9°程度の
  小さな周期振動に収まる(§5bbの理論値と整合)

**新たに判明した、決定的な事実**:
**前進速度が、片道あたり(|F_x|/m)×T_stの振れ幅で振動する**——
数値では**ピークtoピークで約1.03〜1.06 m/s**。これは目標の
cmd_vx=0.15 m/sの**約7倍**。この振動の平均がちょうど0.15 m/sに
なるよう初期条件を正しく設定しても、速度は**+0.69 m/s〜-0.37 m/s**
の範囲を往復し、**1周期の半分近くの時間、体は実際に後方へ動く**。
前進位置のグラフは、一定速度の基準線に対して「進んでは戻り、また
進む」という階段状の波打つ軌跡になり、動画で観察された「ジャンプ
時に足が後ろに行く」現象と定性的に完全に一致する。

**結論**: これは**制御ループの追従性の問題ではない**——遅延も
ソルバーの数値誤差もゼロの、完璧な理想物理シミュレーションでも
避けられない現象。「ピッチトルクを前後交互のF_xで打ち消す」という
トリム戦略**そのものに内在する**、Go2の実ジオメトリ(r_x/h0比
≈0.72)とこの歩容タイミング(周期0.3秒、duty=0.5)から生じる
必然的なトレードオフである。§5bcで「符号を直しても核心的な指標
(meas_vx、peak_pitch)が改善しなかった」理由は、実装のバグでは
なく、**トリム戦略自体が目標速度(0.15 m/s)に対してあまりに大きな
固有振動を持ちすぎている**ことだったと考えられる。

**今後の論点(未着手)**: (a) 目標速度をこの固有振動の規模
(±0.5m/s程度)に合わせて引き上げる、(b) `cycle_period_s`を伸ばして
振動の周波数・振幅自体を変える、(c) ピッチの許容量を増やして
必要な|F_x|自体を減らす(トリムを緩めるトレードオフ、§5az付近の
議論と関連)、(d) 3次元モデルへの拡張、のいずれか。ユーザーへ
引き継ぎ済み、次の一手は未決定。

### 5be. 安価なパラメータ実験(cmd_vx / cycle_period_s)—「cmd_vxを
上げれば直る」という理論予測が実機閉ループでは裏切られ、
`cycle_period_s`短縮が実際に効く唯一の手だと判明(2026-07-20、
plan `frolicking-munching-scone.md`)

§5bdの発見(`delta_v = |F_x_clipped|/m・T_st`がcmd_vxに依存せず
一定)を受け、ユーザーは「大掛かりなモデル拡張やCanter着手の前に、
安価なパラメータ実験(cmd_vxを上げる/cycle_period_sを縮める)で
逆走が解消する運転点があるか安く判定する」方針を選択。

**Phase A(閉形式のスクリーニング、コード変更ゼロ)**:
`ref/scripts/simulate_point_mass_bound_sweep.py`(新規)で
`(cycle_period_s, cmd_vx)`グリッドを評価。`mu_needed=0.721 >
friction_mu=0.7`のため`F_x`は`cycle_period_s`によらず常に
`-mu・F_z`にクリップされ、`delta_v(T) = mu・g・duty・T`は**Tに
線形**、`theta_peak(T)`は**Tの2乗**で効く——両方ともTを短くする
方向にしか効かない(トレードオフなし)ことを確認。理論上の
「逆走なしの境界」(`cmd_vx >= delta_v(T)/2`)は、現行T=0.30では
`cmd_vx>=0.515`(テスト値0.15の3.4倍)、T=0.09では`cmd_vx>=0.1545`
(テスト値0.15とほぼ同値)。

**Phase B(MuJoCo確認、`go2_wbc_bound_period_cmd_vx_screening`、
新規`#[ignore]`テスト)**: 同一の符号修正済みトリム参照設定
(`bound_trim_reference=(100,10)`)のまま、以下5点を実測:

| 構成 | T(s) | cmd_vx | meas_vx | peak_pitch | peak_roll |
|---|---|---|---|---|---|
| A. 基準 | 0.30 | 0.15 | **-0.124**(逆走) | 0.285 | 0.006 |
| B. cmd_vxのみ引き上げ | 0.30 | 0.60 | **-0.183**(悪化) | 0.255 | 0.025 |
| C. 周期のみ短縮(極端) | 0.09 | 0.15 | **+0.092**(正転) | 0.122 | 0.028 |
| D. 併用(穏当) | 0.18 | 0.40 | **+0.270**(正転) | 0.093 | 0.049 |
| E. 併用(穏当) | 0.16 | 0.30 | **+0.163**(正転) | 0.101 | 0.032 |

**予想と食い違った、重要な発見**: Phase Aの理論(質点シミュレー
ション)は「T=0.30のままcmd_vxを0.60まで上げれば`delta_v/2=0.515`
を超えるので逆走は解消するはず」と予測したが、**実際の閉ループ
(MPC+WBC)ではむしろ悪化した**(-0.124→-0.183、peak_rollも
0.006→0.025)。一方、**周期short化は理論の方向性通り効き**、
C/D/Eいずれもmeas_vxの符号が正に反転した(peak_pitchも0.285→
0.09〜0.12まで縮小)。つまり:

- **cmd_vxを上げるだけの延命策は実機閉ループでは機能しない**——
  質点モデルが捉えていない何か(MPC再計算間隔やソルバ収束の
  速度依存の悪化、より速い指令に対する別のカップリングなど)が
  cmd_vx単独の引き上げを裏切る。§5bcで観測された「参照を追従
  させる効果がほとんど出ない」問題は、cmd_vxを上げても解消しない
  ことがここで実証された。
- **`cycle_period_s`短縮は理論通り有効な、実際に機能する唯一の
  レバー**。極端な短縮(T=0.09、`duty=0.5`ゆえ`T_st=0.045s`)単体
  でも符号が反転し、より穏当な短縮(T=0.16〜0.18)とcmd_vxの
  控えめな引き上げ(0.30〜0.40)を組み合わせても正転する。
- ただし**どの構成も指令値の55〜67%程度までしか追従できておらず**
  (D: 0.270/0.40=67.5%、E: 0.163/0.30=54%、C: 0.092/0.15=61%)、
  逆走は解消したが完全な速度追従にはまだ届いていない——引き続き
  `misa-wbc: HoQp inner QP did not reach optimal`警告が全構成で
  多数観測されており、ソルバの収束品質も未解消のまま。

**結論・次の論点**: 「cmd_vxを上げれば直る」という質点モデルの
素朴な予測は実機では否定されたが、「`cycle_period_s`を縮めれば
逆走の符号自体は直せる」ことは実証された(§6の未決定5択のうち
「目標速度引き上げ」単独は棄却、「`cycle_period_s`変更」は
有効と確認)。残る課題は、(1)なぜcmd_vx単独の引き上げが逆効果
だったかの原因究明(質点モデルにない何か)、(2)短縮した周期での
追従率55〜67%をどう100%近くまで詰めるか(pitch_pd_gainの
再チューニング、または§5bd (b)(c)の残る論点)、(3)T=0.09のような
極端な短縮が実機のアクチュエータ帯域・遊脚キネマティクスとして
現実的か(このテストはMuJoCo上の理想アクチュエータでの結果であり、
未検証)。ユーザーへ引き継ぎ済み、次の一手は未決定。

### 5bf. §5beの3つの残課題を一気に解消 — 原因はハード摩擦錐の飽和、
`cycle_period_s=0.18`が遊脚速度的にも唯一現実的(2026-07-20)

ユーザーの指示で§5beの3つの残課題(cmd_vx単独が逆効果だった原因、
55〜67%の追従率をどう詰めるか、極端な周期短縮の実現可能性)を
まとめて調査。

**(1) cmd_vx単独が逆効果だった原因 — ハード摩擦錐がトリム項単体で
飽和済み、速度追従に使える余力がゼロ**:
`go2_wbc_bound_cmd_vx_alone_curve`(新規テスト)でT=0.30固定のまま
cmd_vxを0.15→0.80まで細かくスイープ。結果、meas_vxは**cmd_vxに
ほぼ無関係に-0.093〜-0.183の狭い帯域に張り付いた**(単調悪化では
なく、そもそもcmd_vxに反応しない)。原因は`BoundTrimConfig::
f_x_clipped()`の式そのもの——`cmd_vx`を一切引数に取らず、ピッチ
トルクを打ち消すためだけに`r_x, m, g, h0, friction_mu`だけで
決まる。かつ実機値では`mu_needed(0.721) > friction_mu(0.7)`なので、
**トリム項単体がすでにハード摩擦錐の境界(`friction_cone_soft=
false`、Bound専用設定、`config.rs` L233/L351)を使い切っている**。
つまりMPCの速度追従項やWBCの明示的ピッチPD項が同じ脚の同じ摩擦
予算に上乗せしようとしても、**もう追加できる余地が残っていない**
——cmd_vxを上げても、埋まらない誤差が広がるだけ。`cycle_period_s`
短縮が効くのは、この飽和自体を解消するからではなく、同じ飽和した
`F_x`を掛ける**時間`T_st`を短くする**ことで`delta_v=|F_x|/m・T_st`
自体を減らすから、という理解で一貫する。

**(2) 追従率55〜67%を詰められるか — 詰められない、飽和仮説の
追加的な裏付け**: `go2_wbc_bound_pitch_pd_gain_at_shortened_period_
sweep`(新規テスト)で、最も有望だった構成(T=0.18, cmd_vx=0.40、
trial D)を固定し、`pitch_pd_gain`を(0,0)〜(200,20)まで5段階
スイープ:

| pitch_pd_gain | meas_vx | 対cmd_vx比 | peak_pitch |
|---|---|---|---|
| (0,0)(MPC参照のみ) | 0.255 | 63.8% | 0.090 |
| (50,5) | 0.216 | 54.0% | 0.118 |
| (100,10)(既存既定) | **0.270** | **67.5%** | 0.093 |
| (150,15) | 0.217 | 54.3% | 0.089 |
| (200,20) | 0.254 | 63.5% | 0.104 |

単調な改善傾向はなく、0.22〜0.27の狭い帯域内で非単調に揺れる
だけ——ゲインを上げても下げても追従率は詰まらない。これは(1)の
「摩擦錐がすでに飽和していて、明示的PD項が上乗せできる余地が
そもそもない」という仮説と整合する。**この残課題を本当に詰める
には、`pitch_pd_gain`のようなゲインチューニングではなく、摩擦錐
自体を緩める(`friction_cone_soft=true`+スラックペナルティ、
または`friction_mu`の想定を引き上げてトリムのクリップ自体を
緩める)必要がある** — が、`friction_cone_soft`は現状`WbcParams`/
`FullCentroidalOpts`のテストオーバーライドとして未配線であり、
これは診断テストの域を超えたプロダクションコード変更になるため、
このセッションでは着手せず次の判断待ちとする。

**(3) 極端な周期短縮の遊脚速度としての現実性 — T=0.18が唯一
「安全」、T=0.16は境界線上、T=0.09は非現実的**: `swing_traj.rs`
の`swing_position`(smoothstep xy + sin²高さバンプ)から解析的に
遊脚ピーク速度を導出——水平方向のピーク速度は`1.5・stride/
swing_duration`(smoothstepの導関数のピークが1.5)、鉛直方向は
`π・swing_height/swing_duration`(sin²の導関数のピーク)。
`max_step_length_m=0.12`, `swing_height_m=0.05`, `swing_duration=
cycle_period_s・(1-duty)=T・0.5`を使って計算:

| cycle_period_s | 遊脚ピーク速度(合成) | 判定(対 config.rs既定 3.0 m/s) |
|---|---|---|
| 0.30(現行) | 1.59 m/s | 余裕あり |
| 0.20 | 2.39 m/s | 余裕あり |
| **0.18(trial D)** | **2.65 m/s** | **現実的** |
| 0.16(trial E) | 2.99 m/s | 境界線上 |
| 0.14 | 3.41 m/s | 超過 |
| 0.12 | 3.98 m/s | 超過 |
| **0.09(trial C)** | **5.31 m/s** | **非現実的(1.8倍超過)** |

比較基準は`config.rs`の`default_max_swing_foot_speed()=3.0 m/s`
(LinearCrawl用のGo2脚の実機ガード値だが、"suits a Go2-class leg
under Position-PD"というコード注釈があり、脚のハードウェア的な
妥当性の目安として転用可能)。**trial C(T=0.09)はMuJoCo上の
理想アクチュエータでは逆走を解消できたが、実機の遊脚速度としては
1.8倍以上の超過であり、そのまま実機に持ち込める結果ではない**。
一方**trial D(T=0.18, cmd_vx=0.40)は遊脚速度2.65 m/sで安全域に
収まっており、§5beで見つかった中で唯一「実機で試す価値がある」
構成**。

**総括**: §6の5択のうち「cmd_vx単独引き上げ」は理論・実機の両方で
棄却(むしろ悪化)、「`cycle_period_s`短縮」は有効だが実機で使える
のは`T=0.18`程度の穏当な短縮までで、そこでの追従率67.5%が
このアプローチの実質的な上限(ゲインチューニングでは詰められない、
摩擦錐の飽和が本質的な制約)。100%近い追従を狙うなら、
`friction_cone_soft`を配線してスラックを許す実験、または
§5bd (c)の「ピッチ許容量を増やしてトリムの|F_x|自体を減らす」
方向への着手が次の一手候補。ユーザーへ引き継ぎ済み、次の一手は
未決定。

### 5bg. 部分トリム(`thrust_scale`)を実装 — 摩擦予算は多少解放できたが、
追従率67.5%は本質的には詰まらなかった(2026-07-20)

§5bfの結論(「`friction_cone_soft`のスラックは実機で存在しない摩擦を
使うフェイクになりかねない」)を踏まえ、ユーザーは物理的に正直な
代替案——`BoundTrimConfig`に部分トリム係数`thrust_scale∈[0,1]`を
追加し、`F_x_used = thrust_scale・F_x_clipped`として意図的にフル
トリムより小さい力を使い、ピッチ振動を多少犠牲にして速度追従に
回せる摩擦予算を作る——の実装を選択。

**実装**: `quadruped-gait/src/bound_reference.rs`に`thrust_scale`
フィールドと`f_x_used()`を追加(`sample()`内部の`f_x_clipped()`
呼び出しを置き換え)、新規単体テスト4つ(全11件パス)。
`full_centroidal_controller.rs`に`bound_trim_thrust_scale`フィールド
+ゲッタ/セッタを追加し`BoundTrimConfig`構築箇所に配線。
`generator.rs`(`AnyGaitController`)と`articara/src/gait.rs`
(`GaitController`)にもパススルーを追加(既存の`set_bound_trim_
reference`と同じパターン)。テスト側は`WbcParams::bound_trim_
thrust_scale_override`で両方の`BoundTrimConfig`構築箇所
(MPC参照用・WBCピッチPD参照用)に同じ値を渡す。

**閉形式での事前予測**(`ref/scripts/simulate_point_mass_bound_
sweep.py`、T=0.18固定): `thrust_scale`を1.0→0.0まで下げると
`theta_peak`は0.009→0.304 radまで単調に増加、解放される摩擦予算は
0N→107.17Nまで線形に増加。

**MuJoCo実測**(`go2_wbc_bound_thrust_scale_sweep`、T=0.18,
cmd_vx=0.40, pitch_pd_gain=(100,10)固定):

| thrust_scale | meas_vx | 対cmd_vx比 | peak_pitch |
|---|---|---|---|
| 1.0(trial Dの再現、回帰確認) | 0.270 | 67.5% | 0.093 |
| 0.9 | 0.249 | 62.3% | 0.108 |
| 0.8 | 0.273 | 68.3% | 0.096 |
| 0.7 | 0.285 | 71.3% | 0.092 |
| 0.6 | 0.250 | 62.5% | 0.098 |
| 0.5 | 0.201 | 50.3%(最悪) | 0.097 |
| **0.4** | **0.296** | **74.0%(最良)** | 0.113 |
| 0.3 | 0.232 | 58.0% | 0.125 |

`thrust_scale=1.0`はtrial Dの`meas_vx=0.270`を完全に再現——配線が
正しいことの回帰確認。しかし全体としては`pitch_pd_gain`スイープ
(§5bf)と同じく**非単調でノイズの範囲(50〜74%)に留まり、系統的な
改善trendは見られない**。最良点(thrust_scale=0.4、74.0%)は基準
(67.5%)よりわずかに良いが、閉形式理論が予測した通りには
`peak_pitch`も増えておらず(理論0.186 radに対し実測0.113 rad)、
理論と実測の乖離は§5bd以降で一貫して観測されている通り。

**結論**: 部分トリムは摩擦予算を物理的に正直な形で解放できるが、
**残る追従ギャップ(約30%)を摩擦錐飽和だけで説明するのは正しくない
と判明**——`pitch_pd_gain`と`thrust_scale`という2つの独立したノブを
振っても、どちらも50〜75%の同じような帯域で頭打ちになる。つまり
真の律速要因は「摩擦予算の欠如」よりもっと一般的な、Bound特有の
閉ループ追従性の限界(ソルバの収束品質、MPC再計算間隔、または
まだ特定できていない別の要因)である可能性が高い。§5bcの表現を
借りれば、「参照を追従させる効果がほとんど出ない」問題の核心は
まだ解決していない。

**現状の到達点**: 逆走(符号の反転)は`cycle_period_s`短縮
(T=0.18、遊脚速度2.65 m/sで実機的に安全)により確実に解消できる。
追従率は67〜74%程度が現状の実質的な上限で、`pitch_pd_gain`・
`thrust_scale`のどちらのチューニングでも大きくは動かせない。ここから
先は、(a) この67〜74%という残差を許容してBoundを「完全な速度追従
ではなく、正しい向きに進む歩容」として受け入れる、(b) ソルバの
収束品質(warm-start抑制は既出、他の切り口は未検討)を疑う、
(c) §6に残る3D拡張・Canterへの着手、のいずれかが次の判断点。
ユーザーへ引き継ぎ済み、次の一手は未決定。

### 5bh. 「もっと速度を出せないか」— cmd_vxをさらに上げると比例して
伸びる(T=0.30の時とは対照的)、footstep planner上限付近で非単調に
崩れる(2026-07-20)

ユーザーの質問「もっとスピードは出せないものですか」を受け、
§5bgの最良構成(T=0.18, thrust_scale=0.4, pitch_pd_gain=(100,10))を
固定したまま`cmd_vx`をさらに引き上げてスイープ
(`go2_wbc_bound_faster_cmd_vx_at_best_config_sweep`)。footstep
plannerの速度上限は`v_max=max_step_length_m/(cycle_period_s・
duty_factor)=0.12/0.09=1.33 m/s`(`wbc_walk_go2.rs` L1113-1114と
同じ式)で、遊脚ピーク速度は`max_step_length_m`(cmd_vxではなく)で
決まるため(§5bf点3)、cmd_vxを上げること自体は新たな遊脚速度
リスクを生まない。

**上限内(cmd_vx ≤ 1.20)では比例して伸びる、全て安定**:

| cmd_vx | meas_vx | 対cmd_vx比 | peak_pitch | min_z |
|---|---|---|---|---|
| 0.40 | 0.296 | 74.0% | 0.113 | 0.228 |
| 0.50 | 0.345 | 69.0% | 0.109 | 0.229 |
| 0.60 | 0.434 | 72.3% | 0.106 | 0.228 |
| 0.70 | 0.499 | 71.3% | 0.101 | 0.229 |
| 0.80 | 0.568 | 71.0% | 0.117 | 0.229 |
| 1.00 | 0.750 | 75.0% | 0.107 | 0.229 |
| 1.20 | 0.952 | 79.3% | 0.102 | 0.229 |

§5bfで見た「cmd_vxを上げても効かない」現象(T=0.30, thrust_scale=1.0
固定)とは対照的に、T=0.18・部分トリムのこの構成では**cmd_vxが
ほぼ線形にmeas_vxへ反映され、追従率も69〜79%の狭い帯域で安定**——
`min_z`・`peak_roll`・`peak_pitch`もこの範囲全体で健全なまま。
「摩擦予算が空いていれば、速度指令はちゃんと効く」という§5bgの
仮説と整合する結果。

**footstep planner上限(v_max=1.33)付近以降は非単調に崩れる**:

| cmd_vx | meas_vx | 対cmd_vx比 | peak_pitch | min_z |
|---|---|---|---|---|
| 1.33(=v_max) | 0.452 | 34.0%(悪化) | **0.261(急悪化)** | **0.215** |
| 1.50 | 1.035 | 69.0%(回復) | 0.112 | 0.229 |
| 1.80 | 0.704 | 39.1%(悪化) | **0.290(急悪化)** | **0.215** |
| 2.20 | 1.023 | 46.5%(部分回復) | 0.097 | 0.229 |

`cmd_vx=1.33`(ちょうどfootstep plannerの上限)と`1.80`で明確な
悪化(`peak_pitch`が0.26〜0.29 radまで跳ね上がり、§5beの「混沌とした」
逆走時の値0.285に近い)、一方`1.50`と`2.20`では健全な値に回復する
——単調な劣化ではなく、footstep plannerがクリップし始める領域で
非単調(おそらく歩幅の量子化や、Boundの周期と速度指令のある種の
共振によるアトラクタの切り替わり)。

**結論**: **もっと速度は出せる**——cmd_vx≤1.20(footstep planner上限
1.33の90%)の範囲では、実測速度0.75〜0.95 m/s程度まで安定して
比例的に加速でき、§5bgで見つけた67〜74%という上限は「これ以上
コマンドを上げても意味がない」という限界ではなく、単に**それより
速いcmd_vxをまだ試していなかっただけ**だったと判明。ただし
footstep plannerの上限(v_max=1.33)以降は挙動が不安定・非単調に
なるため、実用上の安全な運転域は**cmd_vx ≤ 1.2 m/s程度まで
(v_maxの管理下)**に留めるのが妥当。ユーザーへ引き継ぎ済み、次の
一手は未決定(v_max付近の非単調性の原因調査、またはより高速な
cmd_vxでの動画撮影のいずれか)。

### 5bi. cmd_vxはどこまで上げられ、どこまで追従できるか — v_max直近の
狭い不安定帯を精密特定、v_max超では実測速度が約1.0〜1.05 m/sで
頭打ち(cmd_vx=8.0まで確認、転倒なし)(2026-07-20)

ユーザーの依頼「どこまでcmd_vxを大きくできるのか、追従できるのかを
検証」を受け、§5bhの粗いスイープ(0.3〜0.5刻み)では分解能不足だった
`v_max=1.33`付近を密にスイープ(`go2_wbc_bound_cmd_vx_boundary_
fine_sweep`、1.00〜2.20を0.05刻み中心)、さらに`v_max`を大きく
超えた領域(`go2_wbc_bound_cmd_vx_extreme_ceiling_sweep`、2.5〜8.0)
で「本当に転倒する上限」を探索。

**v_max=1.33直近に幅0.15 m/s程度の不安定帯**: `cmd_vx=1.25`
(meas_vx=0.470, peak_pitch=0.223)、`1.33`(0.452, 0.261)、`1.36`
(0.812, 0.262)、`1.40`(0.367, 0.234)が明確に悪化する一方、すぐ
隣の`1.20`(0.952, 0.102)、`1.30`(1.020, 0.097)、`1.45`(1.070,
0.098)は健全——**単調な壁ではなく、v_max付近だけがピンポイントで
崩れる狭い帯**。§5bhの粗いスイープが`1.20`(良)と`1.33`(悪)を
たまたま両方拾っていたのは偶然に近い。

**v_max超(cmd_vx≥1.45)では実測速度がほぼ一定の天井に張り付く**:
`1.45`〜`8.00`まで(cmd_vx=8.00は実に指令の23倍以上速い)、健全な
トライアルはすべて**meas_vx≈1.02〜1.08 m/s**に収束——footstep
plannerが`max_step_length_m`で歩幅を頭打ちさせるため、それ以上
cmd_vxを積んでも実際の歩行速度は変わらない。つまり**このBound
構成(T=0.18, thrust_scale=0.4)の実質的な最高速度は約1.0〜1.05 m/s
で、これはcmd_vxの選び方によらない物理的な天井**。

**ただし天井領域にも散発的な悪化点がある**: `1.80`(0.704, 0.290)、
`1.90`(0.867, 0.268)、`3.00`(0.679, 0.287)、`6.00`(0.263, 0.333)、
`8.00`(0.769, 0.289)——`v_max`直近の帯のような連続領域ではなく、
天井に達した後の広い範囲に散発的に現れる。傾向や周期性は未特定
(ソルバの数値的な相性かエイリアシングの可能性、未調査)。

**転倒は一度も起きない**: `min_z`はどの試行でも0.213〜0.229m
(転倒判定0.15mを大きく上回る)、全試行が`finite=true`——
cmd_vx=8.0という非現実的な指令でも、速度は追従しなくなるだけで
姿勢自体は崩壊しない。

**結論・推奨運転域**:
- **cmd_vx ≤ 1.20**: 比例して追従(70〜80%)、meas_vx最大約0.95m/s、
  完全に健全。
- **cmd_vx 1.22〜1.42(v_max=1.33近傍)は避けるべき**——散発的に
  peak_pitchが0.22〜0.26 radまで悪化する狭い不安定帯。
- **cmd_vx ≥ 1.45**: meas_vxは約1.0〜1.05 m/sで頭打ち——これ以上
  指令を上げる意味はない(このコンフィグの実質的な最高速度)。
  ただし散発的な悪化点がまだ残るため、この領域を常用するなら
  そのリスクは残る。
- **実用上の推奨は cmd_vx=1.20 m/s**(meas_vx≈0.95 m/s)——不安定帯
  の直下で、追従率も高く、これ以上速度を上げても実際の歩行速度は
  ほぼ変わらない。

ユーザーへ引き継ぎ済み。次の一手は未定(散発的な悪化点の原因調査、
またはcmd_vx=1.20での動画撮影のいずれか)。

### 5bj. 文献調査を踏まえた再定式化 — `velocity_ripple_fraction`
(MIT Cheetah型「力積スケーリング」)を実装。原理的には正しいが、
残る追従ギャップ・散発的な不安定点は解消しなかった(2026-07-21)

ユーザーへの文献調査(Poulakakis/Papadopoulos/Buehler「Scout II」、
Park/Wensing/Kim「MIT Cheetah 2 vertical impulse scaling」、
Cheng/Alqaham/Gan 2024「Harnessing Natural Oscillations」)で、
「ピッチトルクを毎瞬間ゼロにキャンセルする」という`f_x_clipped()`
の目的関数自体が、これらの先行研究が明示的に否定している設計
思想だと判明。ユーザーの承認を得て、`F_x`をcmd_vxから直接
サイジングする代替パスを実装した。

**実装**(`quadruped-gait/src/bound_reference.rs`): `BoundTrimConfig`
に`cmd_vx_mps: f64`と`velocity_ripple_fraction: Option<f64>`を追加。
`f_x_used()`は`Some(fraction)`の場合、目標ピークtoピーク速度
リップル`fraction·|cmd_vx_mps|`から`delta_v=|F_x|/m·T_st`の逆算で
`F_x`をサイジング(摩擦錐でクリップ)——ピッチはもう設計目標では
なく、事後的に読み取るだけの副産物になる。`None`(既定)は既存の
`thrust_scale·f_x_clipped()`経路を寸分違わず再現、完全後方互換。
`full_centroidal_controller.rs`・`generator.rs`・`articara/src/
gait.rs`に`thrust_scale`と全く同じパターンでパススルーを配線。
単体テスト6本追加(全17件パス、後方互換・線形性・摩擦錐クリップ・
符号規約・cmd_vx依存性・§5bgの`thrust_scale=0.4`実測点との
較正チェック)。

**Phase 0閉形式較正**: §5bgの最良点(`thrust_scale=0.4`, T=0.18,
cmd_vx=0.40, `F_x_used=-42.87N`)を`ripple_fraction`に逆変換すると
`fraction≈0.62`——閉形式モデルが経験則を説明できることの確認
(符号バグを1つ発見・修正: `alpha_p`は`f_x`の符号に対して非対称
なため、`theta_peak`計算には符号付きの`F_x`を渡す必要がある)。

**MuJoCo実測(1) `ripple_fraction`スイープ**(T=0.18, cmd_vx=0.40
固定、`pitch_pd_gain=(100,10)`固定):

| fraction | meas_vx | 対cmd_vx比 | peak_pitch |
|---|---|---|---|
| 0.30 | 0.297 | 74.3% | 0.122 |
| 0.40 | 0.296 | 74.0% | 0.112 |
| 0.50 | 0.287 | 71.8% | 0.095 |
| 0.60 | 0.238 | 59.5% | 0.093 |
| 0.62(較正値) | 0.233 | 58.3% | 0.129 |
| 0.70 | 0.283 | 70.8% | 0.097 |
| 0.80 | 0.242 | 60.5% | 0.101 |
| 1.00 | 0.262 | 65.5% | 0.094 |

`thrust_scale`スイープ(§5bg、50〜75%)と同じ帯域・同じ非単調性——
`fraction=0.4`が最良(74.0%)。

**MuJoCo実測(2) `ripple_fraction=0.4`固定でcmd_vxスイープ**、
§5bh/5biの`thrust_scale=0.4`固定版との比較:

| cmd_vx | thrust_scale=0.4(旧) | ripple_fraction=0.4(新) |
|---|---|---|
| 0.40 | 0.296 (74.0%) | 0.296 (74.0%) |
| 0.60 | 0.434 (72.3%) | 0.388 (64.7%) |
| 1.00 | 0.750 (75.0%) | 0.636 (63.6%) |
| **1.20** | **0.952 (79.3%, 健全)** | **0.788 (65.7%, peak_pitch 0.269へ悪化)** |
| **1.33(v_max)** | **0.452 (34.0%, 悪化)** | **1.062 (79.8%, 健全)** |
| 1.50 | 1.035 (69.0%) | 1.018 (67.9%) |
| 1.80 | 0.704 (39.1%, 悪化) | 0.823 (45.7%, peak_pitch 0.265へ悪化) |
| 2.20 | 1.023 (46.5%) | 1.033 (47.0%) |

**結論**: `velocity_ripple_fraction`は`cmd_vx`をサイジングに直接
組み込むという点で`thrust_scale`より原理的に正しく(文献の設計
思想と整合し、`thrust_scale`という決め打ち定数を物理パラメータ
1つに置き換えられた)、v_max直近の不安定点も`1.33`→`1.20`
付近へと**移動させた**(消してはいない)。しかし全体としては
——追従率の帯域(50〜80%)も、散発的な悪化点の存在自体も
——`thrust_scale`方式と本質的に変わらなかった。つまり、**この
セッションで繰り返し観測してきた「非単調な悪化点」と「67〜80%
止まりの追従率」は、`F_x`のサイジング方法(ピッチキャンセルか
速度リップルか)に起因するものではなく、もっと別の要因(ソルバの
数値的な相性、MPC再計算間隔とBound周期のエイリアシングなど、
§5biで示唆されたまま未特定)によるもの**だと、独立した2つの
サイジング手法が同じ帯域に収束したことで、より強く裏付けられた。

**現状の到達点**: 逆走の解消(`cycle_period_s`短縮)・約70〜80%の
速度追従(`thrust_scale`または`velocity_ripple_fraction`どちらでも
同等)までは再現性高く到達できるが、100%近い追従・v_max近傍の
完全な安定化には、`F_x`のサイジング方式を変える方向の追求では
届かない。ユーザーへ引き継ぎ済み。次の一手は未定——(a) 67〜80%を
Boundの実用的な運転域として受け入れる、(b) ソルバ収束品質側の
未調査の切り口(§5bi)を掘る、(c) §6に残る3D拡張・Canterへの
着手、のいずれか。

### 5bk. 適応周波数CPGの検討 — 診断計測で「散発的な悪化点は接地
タイミングのずれと強く相関する」ことを確認(2026-07-21)

ユーザーへの追加文献調査で、Iida & Pfeifer「"Cheap" Rapid Locomotion
of a Quadruped Robot: Self-Stabilization of Bounding Gait」/
「Self-organized Adaptive Legged Locomotion in a Compliant Quadruped
Robot」(適応周波数振動子がロボット自身の共振周波数を自動追跡する)
と、Zaytsev/Cnops/Remy「A Detailed Look at the SLIP Model Dynamics:
Bifurcations, Chaotic Behavior, and Fractal Basins of Attraction」
(速度を連続的に変えるとサドルノード分岐・ピリオド倍加分岐が現れ、
リターンマップ上で"良い"領域と"悪い"領域が非連続に入れ替わる)
という2つの方向を得た。§5bh/5biで見た「cmd_vx=1.20は健全、1.25は
悪化、1.30は健全、1.33は悪化」という非単調パターンは、後者の分岐
構造そのものに酷似している。

**アーキテクチャ調査**: `quadruped-gait/src/phase.rs`を精査した
ところ、`ContactDrivenPhase`(既存)が、実測GRFに基づく「早期接地/
遅延離地」の検出(`apply_correction`)を**既に実装・配線済み**
——`wbc_walk_go2.rs`の`run_wbc_sim`ループで、WBCの接地判定
(`contact_flag`)には毎ティック使われている。しかし、Boundの
トリム参照(`BoundTrimConfig::sample(cycle_phase)`)は**補正前の
nominal `cycle_phase`をそのまま使っており**、この既存の補正
信号とは接続されていない——つまり、実際の接地が固定周波数の
クロック(`cycle_period_s`)より早い/遅いときも、トリム参照の
ピッチ/`F_x`目標はズレたまま。これは適応周波数オシレータ
(Righetti/Buchli/Ijspeertの"Adaptive Frequency Oscillator"、
あるいはPLL)がまさに必要とするエラー信号——**既存のコードに
本来必要な材料が揃っていながら、フィードバックループとして
閉じられていない**状態だと判明した。

**安価な診断計測**(新規コードなし、`WbcSample`に`contact_phase_
mismatch: bool`を追加しただけ——`nominal`と`corrected`の
`is_stance`が食い違うティックの比率を記録): §5biの`go2_wbc_bound_
cmd_vx_boundary_fine_sweep`を再実行し、既知の"良い"/"悪い"cmd_vx
点でこの比率を比較。

| cmd_vx | 状態(§5bi判定) | contact_phase_mismatch |
|---|---|---|
| 1.00〜1.20, 1.30, 1.45〜1.75, 1.85, 1.95〜2.20 | 健全(16点) | **7.0〜10.4%** |
| 1.25 | 悪化 | **13.2%** |
| 1.33 | 悪化(v_max) | **18.7%(最大)** |
| 1.36 | 悪化 | **14.5%** |
| 1.40 | 悪化 | **14.3%** |
| 1.80 | 悪化 | **16.2%** |
| 1.90 | 悪化 | **11.1%** |

**健全な16点は例外なく7.0〜10.4%の帯域に収まり、悪化した6点は
例外なく11.1%以上——完全に分離している**。散発的な悪化点は、
接地タイミングが固定周波数クロックからずれる頻度と明確に相関する
ことが、新規コードほぼゼロの計測で確認できた。

**結論**: これは「ソルバの気まぐれ」ではなく、**固定周波数の位相
クロックが実際の接地タイミングと同期していない**という、具体的で
対処可能な機構である可能性が高いという強い状況証拠。次の一手
(未着手): `ContactDrivenPhase`が既に検出している早期接地/遅延
離地のエラー信号を使い、`cycle_period_s`自体をゆっくり適応させる
PLL(phase-locked loop)を`PhaseGenerator`に追加する——
`BoundTrimConfig`は`cycle_period_s`を毎ティック`self.cfg.
cycle_period_s`から読み直す設計(`full_centroidal_controller.rs`
L1087)なので、この値が動的になっても**トリム参照側の配線変更は
不要**という見込み。ただしMPCの予測ホライズン側(`build_full_
centroidal_inputs`)が将来の接地スケジュールを現在の周期で予測する
前提を置いている点は要検討——適応が十分ゆっくりであれば局所的に
定数とみなせるはずだが、未検証。全歩容が共有する`PhaseGenerator`
への変更になるため、Bound専用プリセットへの変更より波及範囲が
広く、着手前に専用の実装計画が必要。ユーザーへ引き継ぎ済み。

### 5bl. PLL実装 — 誤差検出の対称化、非リセットのセッター、
ゲイン調整の失敗と成功。全cmd_vx点で追従率が改善(2026-07-22)

§5bkの発見を受け、実際にPLL(phase-locked loop)を実装した。

**Phase A(誤差検出の対称化)**: `quadruped-gait/src/phase.rs`に
新規`PhaseErrorTracker`を追加。既存の`ContactDrivenPhase::apply_
correction`は「実際の歩容が想定より速い」方向(早期接地・早期離地)
のみを真偽値で検出していたが、新規トラッカーは逆方向(遅延接地・
スタンスのオーバーラン)も含めて**符号付き秒数**で定量化する
(単体テスト5本、既存の`apply_correction`は無変更)。§5biの
既知の良い/悪いcmd_vx点で再計測したところ、健全な点は`mean_signed
_error`が-7〜+12ms程度に収まる一方、悪化した点は**+18〜+69ms**と
明確に大きい(全て正=「想定より遅い」方向)——単なる頻度(§5bk)
だけでなく、誤差の大きさそのものが良し悪しを分離することを確認。

**Phase B(非リセットのセッター)**: `FullCentroidalMpcGaitController
::set_cycle_period_s`を新規追加——`PhaseGenerator::set_config`
(cycle_phaseを保持したままcfgだけ差し替える、既存の非リセット
メソッド)を経由するため、既存の`set_config`(phase_genを丸ごと
再生成し`cycle_phase`をゼロリセットしてしまう)と違い、周期を毎
サイクル微調整しても位相が飛ばない。`generator.rs`・`articara/
src/gait.rs`に`thrust_scale`と同じパターンでパススルーを追加。

**Phase C(PLL本体、ゲイン調整で二転三転)**: `wbc_walk_go2.rs`の
テストハーネスに、`PhaseErrorTracker`の誤差を一定間隔で平均し
`set_cycle_period_s`を呼ぶPLLを実装。**最初の試み(`gain=1.0`,
`update_interval_s=1.0`)は全cmd_vx点を悪化させた**——健全だった
点(cmd_vx=1.20, 1.30, 2.20)まで大きく崩れ(例: 1.30は追従率78.5%
→19.8%、2.20は46.5%→ほぼ停止)。AFO/PLL理論が要求する「適応は
振動そのものよりずっと遅くなければならない」という条件に反していた
と判断し、`gain=0.15`, `update_interval_s=2.0`まで大幅に緩めた
ところ、**全cmd_vx点で改善**:

| cmd_vx | 固定T=0.18(§5bi) | PLL(gain=0.15) | 収束周期 |
|---|---|---|---|
| 1.00 | 75.0% | **84.6%** | 0.182 |
| 1.20(健全) | 79.3% | **87.0%** | 0.180 |
| 1.25(悪化) | 37.6% | **61.5%** | 0.188 |
| 1.30(健全) | 78.5% | **88.1%** | 0.180 |
| 1.33(v_max、最悪) | 34.0% | **54.6%** | 0.187 |
| 1.36(悪化) | 59.7% | **76.8%** | 0.182 |
| 1.40(悪化) | 26.2% | **41.6%** | 0.188 |
| 1.45(健全) | 73.8% | **77.9%** | 0.182 |
| 1.80(悪化) | 39.1% | **53.2%** | 0.183 |
| 1.90(悪化) | 45.6% | **55.6%** | 0.183 |
| 2.20 | 46.5% | **52.3%** | 0.180 |

**「cmd_vx毎の最適周期」への回答**: 収束周期はcmd_vxに応じて大きく
変わるわけではなく、**0.180〜0.188 sという狭い帯域に収束**した
——つまりT=0.18という手動選択自体はほぼ正しかったが、実測の接地
タイミングに合わせて0.2〜4%程度の微調整を続けることで、追従率が
全点で有意に改善する。特に§5biで見つかった最悪点(cmd_vx=1.33、
v_max直近)が34.0%→54.6%まで回復したのが最大の成果。

**残る課題**: `peak_pitch`は一部の点(1.33, 1.36, 1.80, 1.90で
0.25〜0.29 rad)で改善が乏しく、固定周期時とほぼ同水準のまま——
PLLは主に速度追従の一貫性を改善しており、ピッチの散発的な悪化
自体はまだ完全には解消していない。`gain`/`update_interval_s`の
更なる調整余地も残る(今回試したのは2点のみ)。

**結論**: 適応周波数の考え方(§6bkで文献調査した Iida & Pfeifer
のアプローチ)は、Boundの残る追従ギャップに対して**実際に効く**
ことが実証された——ただしAFO/PLL理論が警告する通り、ゲインの
選び方が極めて重要で、積極的すぎるゲインはむしろ全ての運転点を
悪化させる。全回帰テスト(articara 7件、quadruped-gait 200件)
パス。ユーザーへ引き継ぎ済み。次の一手候補: (a) この結果を採用し
Boundの新しい基準構成とする、(b) gain/update_interval_sをさらに
細かくスイープして最適化する、(c) peak_pitchの残る悪化を別途
調査する。

### 5bm. peak_pitch残存問題の調査(手順(c))とPLLパラメータの
本格スイープ(手順(b))(2026-07-22)

ユーザー指示により§5blの残課題2点に着手。

**(c) peak_pitch残存問題**: PLL(gain=0.15)はcmd_vx=1.33で
meas_vxを34.0%→54.6%まで大きく改善したが、peak_pitchはほぼ
変わらなかった(0.261→0.251)。これは周期調整(PLL)とは独立な
軸である`thrust_scale`(§5bg、Fxサイジング)を併用すれば
解決するのではという仮説を立て、cmd_vx=1.33でPLL固定・
`thrust_scale`をスイープ:

| thrust_scale | meas_vx比 | peak_pitch |
|---|---|---|
| 0.2 | 80.7% | 0.105 |
| 0.3 | 80.5% | 0.272(悪) |
| 0.4(元の値) | 54.6% | 0.251 |
| **0.5** | **87.8%** | **0.097** |
| 0.6 | 87.4% | 0.092 |
| 0.7 | 80.2% | 0.114 |
| 0.8 | 76.1% | 0.244(悪) |
| 1.0 | 87.2% | 0.094 |

`thrust_scale=0.5`(または0.6, 1.0)で**meas_vx比87.8%・
peak_pitch=0.097 rad**——ほぼ問題を解消。ただし他の悪化点
(1.25, 1.36, 1.80, 1.90)で`thrust_scale=0.5`固定のまま
汎化するか確認したところ:

| cmd_vx | meas_vx比 | peak_pitch |
|---|---|---|
| 1.25 | 80.9% | 0.242(悪いまま) |
| 1.33 | 87.8% | 0.097(解消) |
| 1.36 | 74.3% | 0.264(悪いまま) |
| 1.80 | 64.5% | 0.094(解消) |
| 1.90 | 53.7% | 0.279(悪いまま) |

速度追従は全点で大幅に改善した(旧34〜46%→54〜88%)が、
**peak_pitchは`thrust_scale=0.5`が効く点(1.33, 1.80)と効かない
点(1.25, 1.36, 1.90)が混在**——thrust_scale・PLLどちらの軸でも、
cmd_vxごとに個別最適値が違うという、このセッション全体で繰り返し
観測してきた非単調パターンがここでも再現した。全cmd_vx点で
peak_pitchまで含めて解消するには、cmd_vxごとの同時最適化
(thrust_scale × gain × update_interval_sの3次元グリッド)が
必要そうだが、それは今回のスコープを超える。

**(b) gain/update_interval_sの本格スイープ**: cmd_vx=1.33、
`thrust_scale=0.4`(§5blと同一条件、Fxサイジングの効果と混同
しないよう固定)で`gain∈{0.05,0.10,0.15,0.20,0.30}`×
`update_interval_s∈{1.0,2.0,4.0}`の15点グリッド:

| gain\\interval | 1.0s | 2.0s | 4.0s |
|---|---|---|---|
| 0.05 | 83.7% / 0.221 | 59.0% / 0.411(最悪) | 72.7% / 0.261 |
| 0.10 | **84.1% / 0.222** | 63.9% / 0.370 | 70.2% / 0.261 |
| 0.15(§5bl採用値) | 80.1% / **0.218** | 46.6% / 0.251 | 68.5% / 0.261 |
| 0.20 | 72.9% / 0.234 | 62.5% / 0.290 | 69.3% / 0.261 |
| 0.30 | 32.5%(崩壊) / 0.383 | 50.8% / 0.277 | 67.9% / 0.261 |

(値はmeas_vx比 / peak_pitch)。**`update_interval_s=1.0`が
gainの値によらず一貫して最良**——§5blで採用した`update_
interval_s=2.0`はこの中で最も悪い部類だった(gain=0.15での
46.6%は今回のグリッド15点中最下位に近い)。`gain`は0.05〜0.20の
範囲ではinterval=1.0の下では大差なく(73〜84%)、0.30まで上げると
崩壊する(32.5%)——つまり「更新間隔を短く、ゲインは0.3未満」が
安定運用の条件。**新推奨: `gain=0.10, update_interval_s=1.0`**
(84.1%/0.222、このグリッドの最良点)。

**結論**: `update_interval_s`は`gain`よりずっと支配的なパラメータ
だった——最初にgain=1.0で崩壊した際「適応が速すぎる」と結論した
のは半分正しかったが、実際に効いていたのはgain自体よりも更新
「頻度」の低さ(interval=2.0)の方だったと判明。今後PLLを使う際は
`update_interval_s=1.0`を既定にすべき。ユーザーへ引き継ぎ済み。
次の一手候補: (a) `gain=0.10, update_interval_s=1.0`を新しい
既定として採用、(b) thrust_scale×gain×intervalの3次元同時最適化
(cmd_vxごとに)、(c) ここで一区切りとしてコミット。

**その後の対応**: (a)採用・(c)コミット実施済み(quadruped-gait
f1d5707、articara 0004dcb、両リポの全変更を1コミットずつ)。
`go2_wbc_bound_adaptive_cycle_period_video_source`等4箇所の
`AdaptivePeriodConfig`既定値を`gain=0.10, update_interval_s=1.0`
に更新し、動画も撮り直し(cmd_vx=1.33で追従率46.6%→84.1%まで
改善した新版)。

### 5bn. 文献再検討 — `pitch_pd_gain`をゼロに寄せる実験。単一点では
劇的成功も、全点への汎化には失敗——「万能ノブは存在しない」という
このセッション全体の結論をさらに補強(2026-07-22)

文献(Cheng/Alqaham/Gan 2024「Harnessing Natural Oscillations」、
Poulakakis & BuehlerのScout II)の核心——「ピッチを目標値に追い
込むのではなく、受動的な回転として許容する」——を踏まえ、
`wbc_pipeline.pitch_pd_gain`(トリム参照のピッチ目標を能動的に
追従させるPD項)を弱める実験を実施。

**cmd_vx=1.33固定での`pitch_pd_gain`スイープ**(PLL gain=0.10/
interval=1.0、thrust_scale=0.4固定):

| pitch_pd_gain | meas_vx比 | peak_pitch |
|---|---|---|
| (100,10)基準 | 84.1% | 0.222 |
| (50,5) | 88.7% | 0.105 |
| (20,2) | 85.1% | 0.102 |
| (10,1) | 86.6% | 0.105 |
| (5,0.5) | 88.9% | 0.102 |
| **(0,0)——ピッチ補正を完全撤廃** | **88.7%** | **0.103** |

劇的な結果——ピッチのPD補正を弱める(あるいは完全に撤廃する)
ことで、**追従率が悪化するどころか改善し(84.1%→88.7%)、
peak_pitchはほぼ半減した(0.222→0.103)**。文献の主張——「ピッチを
追わない方が良い」——を単一点では鮮やかに裏付ける結果。

**全cmd_vx点への汎化テスト**(`pitch_pd_gain=(0,0)`固定、PLL・
thrust_scale=0.4も固定):

| cmd_vx | meas_vx比 | peak_pitch | 参考: PLL単体(pitch_pd=100,10)時 |
|---|---|---|---|
| 1.00 | 66.0% | 0.130 | 84.6% / 0.094 |
| 1.20 | 80.8% | **0.227(悪化)** | 87.0% / 0.102 |
| 1.25 | 82.4% | **0.256(悪化)** | 61.5% / 0.236 |
| 1.30 | 77.8% | **0.244(悪化)** | 88.1% / 0.100 |
| 1.33 | 88.7% | 0.103(改善) | 84.1% / 0.222 |
| 1.36 | 85.7% | 0.115(改善) | 76.8% / 0.262 |
| 1.40 | 81.5% | 0.278(同程度) | 41.6% / 0.234 |
| 1.45 | 53.5%(悪化) | 0.309(悪化) | 77.9% / 0.139 |
| 1.80 | 59.4%(悪化) | **0.278(悪化)** | 53.2% / 0.290 |
| 1.90 | 41.8%(悪化) | 0.114(改善) | 55.6% / 0.268 |
| 2.20 | 42.9%(悪化) | 0.185 | 52.3% / 0.097 |

**汎化には失敗した**——`pitch_pd_gain=(0,0)`はcmd_vx=1.33/1.36/
1.90では確かにpeak_pitchを改善するが、**それまで健全だった
1.20/1.25/1.30では逆にpeak_pitchを悪化させ**、1.00/1.45/1.80/
1.90/2.20では速度追従自体を悪化させた。単一点(1.33)での鮮やかな
成功は、その点特有の偶然の一致であり、全点に効く「万能設定」では
なかった。

**結論**: 文献の主張(ピッチを追わない方が自然)自体は否定されて
いない——実際、いくつかの点では明確に効いている。しかし、
`thrust_scale`・`velocity_ripple_fraction`・PLLの`gain`/`update_
interval_s`、そして今回の`pitch_pd_gain`と、**このセッションで
振ったほぼ全てのパラメータが同じパターンを示した**: どの軸で
調整しても、「効く点」と「効かない点」がcmd_vxに応じて入れ替わり、
全域に効く単一の設定は見つからない。これは個々のパラメータの
選択ミスではなく、**Bound自体がハイブリッド力学系として持つ
分岐構造(前回文献調査で触れたZaytsev/Cnops/Remyの分岐解析)を、
形を変えて何度も観測している**という解釈をより強く支持する。

**現実的な帰結**: cmd_vxごとに全パラメータ(thrust_scale×
pitch_pd_gain×PLL gain/interval)を個別最適化しない限り、
「全域で完璧」は達成できなさそうだと分かった。次の一手候補:
(a) cmd_vxに応じてパラメータを切り替えるスケジューリング
(ゲインスケジューリング、簡易だが場当たり的)、(b) 文献にある
分岐解析の枠組みで、なぜこのパターンが起きるかを理論的に理解する
(道具作りが必要、時間がかかる)、(c) 現状(PLL+thrust_scale=0.4、
pitch_pd_gain=(100,10)、67〜90%程度の追従率)を許容範囲として
受け入れ、ここで区切りとする。ユーザーへ引き継ぎ済み。

### 5bo. cmd_vx_ramp_s(徐々に加速するスタート)とPLLの組み合わせ
——未解決の新たな相互作用を発見(2026-07-23)

ユーザーから「走り始めたら徐々にストライドを大きくして速度を
上げることはできるか」との質問。既存の`cmd_vx_ramp_s`機構
(0からcmd_vxまで線形にランプ、footstep plannerのRaibert
ヒューリスティックがストライドを自動的に速度追従させる、新規
コードは不要)が使えるはずと考え、現在の既定構成(T=0.18,
thrust_scale=0.4, pitch_pd_gain=(100,10), PLL gain=0.10/interval
=1.0)と組み合わせてcmd_vx=1.30まで3秒でランプする動画取得を試みた。

**発見: 3秒ランプ後、ロボットが後退し続ける**。body_xが t=4s
付近から単調減少し、t=10sまでに-4.0mまで後退。定常状態(t=0から
cmd_vx=1.30固定)では同じ10秒間で問題なく+11.2m前進することを
別途確認しており、後退は明確にランプ由来。

**一次診断**: PLLの誤差蓄積ウィンドウが、ランプ完了の瞬間
(t=3.5s)をまたいでいた(t=3.0〜4.0sの更新ウィンドウ)ため、
加速中の過渡的な接地誤差が「クロックが遅れている」という
誤った信号として混入し、巡航直後の周期補正を誤らせていた可能性を
疑い、**ランプ中はPLLの誤差蓄積を止め、ランプ終了の瞬間に
蓄積をリセットする**修正を実施(`wbc_walk_go2.rs`の`run_wbc_sim`、
`ramp_in_progress`ガード追加)。

**しかし修正後も後退は解消しなかった**(後退開始がt≈4s→t≈5〜7sに
遅れただけ)。定常状態を10秒維持しても後退が一切起きないことを
再確認したため、「長時間実行すると自然に劣化する」という仮説は
棄却——**問題はランプそのもの(またはランプ→巡航への遷移)に
起因**すると判明。

**ランプを6秒まで緩めると、後退は避けられたが「失速」に置き換わる**:
ランプ完了直後(t≈6.5s)にx≈2.15mまで前進した後、t=6.5〜10sの
巡航区間でほぼ完全に停止(x≈2.14〜2.17mで横ばい)——後退はしないが、
定常状態なら本来出せるはずの1.1〜1.2 m/sの巡航速度には至らない。

**結論**: `cmd_vx_ramp_s`自体は正しく機能する(ストライドはcmd_vxに
応じて自動的に拡大する、要望通りの挙動)が、**PLL/Boundの現在の
チューニングとの組み合わせでは、「同じcmd_vx・同じcycle_period_s」
という最終状態に、定常開始とランプ経由とで到達した場合に、
挙動が全く異なる**——後者は明確に悪化する。これは§5bnで確立した
「ほぼ全てのパラメータがcmd_vxごとに効く/効かないが入れ替わる」
という知見の、新しい側面(パラメータだけでなく**到達経路**も
結果を左右する)であり、文献の分岐解析の枠組み(Zaytsev/Cnops/
Remy)——同じパラメータに対して複数の解(アトラクタ盆地)が
共存しうる——とも整合的である。

**現状**: 3秒ランプ版・6秒ランプ版とも動画取得済み
(`tests/media/go2_bound_ramp_up.mp4`は6秒版、後退は避けられて
いるが失速する様子がそのまま映っている)。この問題は未解決の
まま引き継ぎ。次の一手候補: (a) さらに緩いランプ(8〜10秒)を試す、
(b) ランプ終了後もしばらくPLLを無効化してから再開する、
(c) この問題を保留し、定常速度での運用(現状の67〜90%追従率)に
留める。ユーザーへ引き継ぎ済み。

**訂正(2026-07-23)**: 上記の「後退」「失速」という診断は誤り
だった。ユーザーから「後退というより方向転換しているのでは」との
指摘を受け、`WbcSample`にこれまで捨てていた`yaw`(と`body_y`)を
追加して確認したところ、**ロボットは後退も失速もしておらず、
単に大きくヨー回転(方向転換)して、新しい向きのまま前進を
続けていた**:

- 3秒ランプ版(body_xが-4mまで「後退」に見えた回): yawが
  0°→ほぼ180°(176°)まで回転していた——つまり180°方向転換して
  向きを変えたまま前進を続けていただけで、本当に後退していた
  わけではない。
- 6秒ランプ版(body_xが2.15m前後で「失速」に見えた回): yawが
  0°→約90°まで回転。「失速」もこの間ロボットが向きを変えていた
  だけで、水平面内の実速度(planar_speed、方向によらない)は
  むしろ1.1 m/s以上出ていた(旋回中の一時的な鈍化を除く)。
- 対照として定常状態(cmd_vx=1.30を最初から10秒維持)を確認すると
  yawはほぼ0°付近で振動するのみ(±14°程度)——単調な方向転換は
  一切起きない。

**新しい結論**: 問題は速度追従の破綻ではなく、**cmd_vxランプ中に
何らかの左右非対称な外乱がヨー方向の回転を誘発し、それが
(cmd_wz=0のまま、姿勢を積極的に元へ戻すヨー方向のフィードバック
が無いため)そのまま蓄積して大きな方向転換につながる**という、
全く別種の問題だった。定常状態ではこの外乱が起きない(または
十分小さい)ため、これはランプ特有の現象。`report_walk_summary`
に`planar_speed`(方向によらない水平速度)と`yaw_drift`(観測窓
全体でのヨー変化量)を追加し、以後この種の誤診断(x軸だけ見て
「後退/失速」と誤認する)が起きないようにした。

**次の一手候補(更新)**: (a) ランプ中に何が左右非対称なヨー
トルクを生んでいるのかを特定する(footstep plannerの左右非対称な
着地タイミング、あるいはWBCの左右GRF配分の過渡的な偏りが疑わしい)、
(b) 明示的なヨー保持フィードバック(現状cmd_wz=0のレート指令のみで、
姿勢そのものを戻す項が無い可能性)を追加する、(c) この問題を
保留し、定常速度での運用に留める。ユーザーへ引き継ぎ済み。

### 5bp. WBCに明示的なヨー保持フィードバックを追加(手順(b))——
3秒ランプのyaw_drift 156°→0.4°まで解消(2026-07-23)

§5boで発見した「ランプ後の方向転換」問題への対処として、既存の
`roll_pd_gain`/`pitch_pd_gain`(§5au/5av/5bc)と全く同じパターンで
`WbcPipeline`に`yaw_pd_gain`/`yaw_ref`を追加した。

**実装**: `articara/src/wbc_pipeline.rs`の`solve()`に、`a_ang_body.z
+= yaw_kp・wrap_to_pi(yaw_ref - yaw_meas) - yaw_kd・omega_obs_body.z`
を追加(`_yaw_meas`として捨てていた実測ヨーを使うよう変更)。
ヨーは実際に±180°境界をまたぎうるため(§5boの3秒ランプは176°まで
ドリフトした)、ロール/ピッチと違い`wrap_to_pi`でラップする必要が
あり、`full_centroidal_controller.rs`の`velocity_cmd_for_goal`に
既にあった式`(x+π).rem_euclid(2π)-π`を再利用した。

`yaw_ref`の供給元は、新規追加した`AnyGaitController::world_yaw()`
(`quadruped-gait/src/generator.rs`)——`BodyState::integrate`が
`cmd.wz`を時間積分して保持している値をそのまま使う。新規の積分器は
不要で、`cmd_wz=0`なら自動的に0(直進維持)、実際に旋回コマンドを
出した場合も自然に整合する。全て`(0.0,0.0)`既定で完全後方互換
——Trot等の既存回帰テスト(articara 7件、quadruped-gait lib
200件)は無変更で全てパス。

**§5boの3秒ランプ再現ケース(cmd_vx=1.30, cmd_vx_ramp_s=3.0)で
`yaw_pd_gain`をスイープ**:

| yaw_pd_gain | yaw_drift | meas_vx(x軸) | planar_speed |
|---|---|---|---|
| (0,0)基準(§5bo相当) | **156.2°(暴走)** | -0.218(後退に見える) | 0.575 |
| (5,0.5) | 0.1° | 0.873 | 0.938 |
| **(10,1.0)** | **0.4°** | **0.991** | **0.991** |
| (20,2.0) | -4.8° | 0.976 | 0.979 |
| (50,5.0) | -9.6° | 0.979 | 0.986 |

`yaw_pd_gain=(10,1.0)`が最良——yaw_drift 156.2°→0.4°まで解消、
x軸速度・planar_speedがほぼ一致(=ほぼ完全に直進)、追従率も76.2%
まで回復した。動画を撮り直し(`tests/media/go2_bound_ramp_up.mp4`、
3秒ランプ+yaw保持あり)。

**結論**: §6bで検討した(b)案(既存roll/pitch PDと同じパターンで
ヨーを追加)は正しい選択だった——ランプ由来のヨー暴走はきれいに
解消した。ただし根本原因(なぜランプ中に左右非対称なヨートルクが
発生するのか、§5boの(a)案)はまだ未調査のまま——今回のヨー保持
フィードバックは対症療法であり、外乱そのものを消したわけではない
点に注意。また、これまでの全てのcmd_vx定常スイープ(§5be〜5bn)は
`yaw_pd_gain=(0,0)`のまま実施されており、それらの結果自体への
影響はない(定常状態ではyaw_driftがそもそも小さかったため)。

**次の一手候補**: (a) ランプ中の左右非対称外乱の根本原因を特定する
(任意、対症療法で十分なら不要)、(b) `yaw_pd_gain=(10,1.0)`を
Trot等他歩容も含めた既定値として採用するか検討する、(c) ここで
一区切りとしてコミットする。ユーザーへ引き継ぎ済み。

### 5bq. `duty_factor<0.5`(真の滞空期)への拡張 — 実装成功も、
速度上限は**むしろ悪化**という否定的結果(2026-07-23)

ユーザーから「より高速なBound歩容をやりたい、4脚全てが遊脚となる
瞬間はあるか」との要望。§5bp時点のBound(`duty_factor=0.5`既定)は
前ペア・後ペアが隙間なくタイル状に切り替わる設計で、滞空期(全4脚
遊脚)が一切ないことを`go2_diag_bound_duty_factor_flight_phase_
sweep`で確認済み。Cheng/Alqaham/Gan 2024の「高さ固定LIPは最大
3.0 m/s、真の滞空期を持つSLIPは4 m/s以上」という報告を根拠に、
`duty_factor<0.5`拡張が速度上限を突破する本筋だと想定して着手。
「既存機能(duty=0.5)を維持しつつ追加機能として対応」との指示で
実施。

**Phase 0(数理検証、コード変更ゼロ)**: `ref/scripts/simulate_
point_mass_bound_flight_phase.py`を新規作成。支持期`[0,T_st)`→
滞空期`[T_st,T/2)`の2区間半周期BVPを閉形式で解いた
(`F_z_total=m·g/(2·duty)`、境界条件`θ(0)=-α_p·T_st·T_flight/4`、
`θ̇(0)=-α_p·T_st/2`)。純粋Newton-Euler数値積分(滞空期は`F=0`)と
突き合わせ、`duty=0.5`(`T_flight=0`)で§5bdの結果(`θ(0)=0`)を
厳密に再現、`duty<0.5`でも数値積分との残差がEuler打ち切り誤差の
範囲内(dtを1/10にすると残差も1/10)であることを確認——閉形式は
正しい。

**Phase B(`bound_reference.rs`の一般化)**: `f_z_total()`を
`m·g/(2·duty_factor)`に変更、`theta_boundary()`/`theta_peak()`
(3候補の最大値: `|θ(0)|`・`|θ(T_st)|`・支持期内極値)・`sample()`
(支持期は2次式、滞空期は`θ̇`一定の1次式、`f_x_per_leg=f_z_per_leg
=0`)を新規実装。**実装中に見つけたバグ**: `f_x_trim()`が
`f_z_total()`経由ではなくハードコードした`m·g`を使っていたため、
`duty<0.5`での値がPython参照値と2倍近くずれた——`f_z_total()`を
呼ぶよう修正して解消。既存11本(実際は17本)の単体テストは
`duty=0.5`で無変更のまま全て通過(回帰確認済み)。Python参照値と
一致する新規単体テスト5本を追加(`f_z_total_grows_as_duty_factor_
shrinks_below_half`等)、全22本パス。`quadruped-gait --lib`
205件、`articara --test wbc_walk_go2`(非ignore)7件も無傷で確認。

**Phase C(MuJoCo検証)**: 最大のリスクだった`n_stance=0`
(4脚とも支持なし)経路は**構造的には生存**——`go2_wbc_bound_
flight_phase_duty_sweep`(素のbaseline設定)・`go2_wbc_bound_
flight_phase_at_best_config_sweep`(§5bp確立済みの最良構成:
T=0.18, `bound_trim=(100,10)`, thrust_scale=0.4, yaw_pd_gain=
(10,1), PLL gain=0.10/interval=1.0)のいずれも、`duty_factor`を
0.5→0.25まで下げても転倒せず(`min_z`は常に0.21〜0.23m、
`finite=true`)。QPソルバーは`HoQp inner QP did not reach optimal`
警告を頻発するが、致命的破綻はしない。

しかし**肝心の速度上限は突破できず、むしろ悪化した**。
`go2_wbc_bound_flight_phase_at_best_config_sweep`(cmd_vx=1.30
固定、duty別)では、dutyを下げるほど単調に追従が悪化:

| duty_factor | meas_vx | planar_speed | contact_phase_mismatch | yaw_drift |
|---|---|---|---|---|
| 0.50(基準) | 1.029 | 1.029 | 5.8% | -1.7° |
| 0.45 | 0.911 | 0.927 | 11.4% | 18.7° |
| 0.40 | 0.848 | 0.848 | 9.9% | 4.7° |
| 0.35 | 0.794 | 0.794 | 15.9% | -4.6° |
| 0.30 | 0.719 | 0.719 | 21.0% | -3.3° |
| 0.25 | 0.506 | 0.506 | 27.2% | -15.0° |

固定cmd_vxでの劣化だけでは「上限自体が動いたか」を判別できない
ため、`go2_wbc_bound_flight_phase_cmd_vx_ceiling_sweep`で
`duty=0.50`と`duty=0.35`それぞれについて、他の設定を1バイトも
変えずに同じcmd_vxグリッド(0.40〜2.20)を掃引し、実際に追従できた
最高速度(=上限)を比較した:

| duty_factor | 追従速度の頭打ち(meas_vx) | 到達したcmd_vx |
|---|---|---|
| 0.50(基準) | **≈0.99 m/s**(0.984〜0.988で飽和) | 1.33以上 |
| 0.35 | **≈0.82 m/s**(0.805〜0.821で飽和) | 1.20〜1.33 |

`duty=0.35`の上限は`duty=0.50`よりおよそ17%低い。全域で
`contact_phase_mismatch`も`duty=0.35`の方が一貫して高い
(13〜32% vs 6〜18%)。

**なぜ滞空期が速度上限を上げないのか(物理的な理由)**: このモデルの
`F_x`は摩擦円で頭打ちになる区分定数(`F_x_max=μ·F_z_total=
μ·m·g/(2·duty)`)——`duty`が小さいほど`F_z_total`(と`F_x_max`)は
大きくなるが、支持期の時間割合も同じだけ小さくなるため、**1周期
あたりの平均駆動力`F_x_max·duty=μ·m·g/2`は`duty`に依存せず一定**
という関係がこのモデルには元々埋め込まれている(理論上の速度上限は
duty不変のはず)。実測でむしろ悪化したのは、この理論的な中立性を
上回る形で、`n_stance=0`区間そのものがWBC/MPCの追従・接触タイミング
制御を難しくしている(`contact_phase_mismatch`の単調な悪化が示す
通り)ため——Cheng et al. 2024のSLIP優位性は、真のバネ弾性(接地
中に運動エネルギーを蓄積・解放する力学)を前提にしており、この
モデルの「区分定数力」設計にはそもそもそのメカニズムが存在しない。
`duty<0.5`拡張は**数理的には正しく実装できたが、この設計のFxの
サイジング方式(摩擦円頭打ちの区分定数)のままでは速度上限を
上げる効果がない**、というのが本セクションの結論。

**結論**: 「既存機能を維持しつつ追加機能として対応」という要件は
達成(`duty_factor=0.5`は数式レベルで恒等的に不変、既存の全回帰
テストが無傷、`duty<0.5`は`with_duty_factor()`を明示的に呼んだ
ときだけ新経路に入る完全オプトイン)。しかし当初の目的だった
「より高速なBound」は`duty_factor`を下げるだけでは達成できない
ことが判明した。動画撮影はスキップ(§5bqの数値結果で十分、かつ
劣化ケースの動画に実益は薄いと判断)。

**次の一手候補**: (a) このセッションの`duty<0.5`拡張自体は現状維持
し、速度向上は別の軸(§5bi以来何度も浮上している`max_step_length_
m`拡大、あるいはcmd_vxそのものより`F_x`サイジング方式の再設計)で
追求する、(b) `F_x`を区分定数でなく実際にバネ的に(接地区間内で
時間変化させる)サイジングし直し、Cheng et al.のSLIP的な効果を
物理的に再現できるか検討する(大掛かりな変更)、(c) `duty<0.5`
自体は「4脚とも遊脚になる瞬間がある」という見た目上の要望は満たす
ため、速度でなく見た目(跳躍感のあるBound)を目的にするなら
現状のまま採用してよい。ユーザーへ確認予定。

### 5br. §5bq候補(a)を検証 — `thrust_scale`を`duty=0.35`のまま
引き上げたところ、`duty=0.50`の速度上限にほぼ並んだ(2026-07-24)

ユーザーから「跳躍しながらより速く移動できるか」との追加要望。
§5bqの結論(「区分定数`F_x`サイジングでは1周期あたり平均駆動力
`F_x_max・duty`がduty不変」)を踏まえ、次の2つを検証した:

**(1) `thrust_scale`を`duty=0.35`のまま引き上げるスイープ**
(`go2_wbc_bound_flight_phase_duty035_thrust_scale_sweep`、
cmd_vx=1.33固定、T=0.18・yaw_pd=(10,1)・PLL既存最良設定込み):

| thrust_scale | meas_vx | peak_pitch | contact_phase_mismatch |
|---|---|---|---|
| 0.4(§5bq基準) | 0.805 | 0.119rad | 18.3% |
| 0.5 | 0.829 | 0.121rad | 16.6% |
| 0.6 | 0.346(不安定域) | 0.303rad | 26.1% |
| 0.7 | 0.358(不安定域) | 0.232rad | 27.5% |
| 0.8 | 0.493(不安定域) | 0.449rad | 21.3% |
| 0.9 | 0.860 | 0.112rad | 13.2% |
| **1.0** | **0.875** | 0.118rad | 17.1% |

0.6〜0.8に共振的な不安定域(peak_pitchが0.3〜0.45radまで跳ね上がる)
があるものの、そこを抜けた`thrust_scale=1.0`は0.4基準比+8.7%
(0.805→0.875)。`thrust_scale=0.4`が`duty=0.5`用に選ばれたのは
摩擦円飽和への保守マージンだったが(§5bf/5bg)、`duty=0.35`では
`F_z_total`(と摩擦予算)自体が絶対値として大きく、かつ滞空期が
ピッチ外乱の一部を吸収するため(Phase 0: theta_peak 0.00904→
0.01175rad、duty 0.50→0.35で緩やかにしか増えない)、そのマージンが
不要になっていた可能性が高い。

**(2) `cycle_period_s`の再チューニング**
(`go2_wbc_bound_flight_phase_duty035_cycle_period_sweep`、
thrust_scale=0.4固定、cmd_vx=1.33固定): 0.14〜0.26sを掃引したが
`T=0.18`(§5bg以来`duty=0.5`用に最適化された値)が`duty=0.35`でも
そのまま最良点のままだった(0.805 m/s、他は0.047〜0.764 m/sまで
悪化)——再チューニングの余地はなし。

**(3) `duty=0.35`+`thrust_scale=1.0`での`cmd_vx`上限スイープ**
(`go2_wbc_bound_flight_phase_duty035_thrust_scale_1_ceiling_sweep`、
§5bqの`go2_wbc_bound_flight_phase_cmd_vx_ceiling_sweep`と同じ
cmd_vxグリッド):

| 構成 | 追従速度の頭打ち(meas_vx) | 到達cmd_vx |
|---|---|---|
| duty=0.50, thrust_scale=0.4(§5bq基準) | ≈0.99 m/s(0.984〜0.988) | 1.33以上 |
| duty=0.35, thrust_scale=0.4(§5bq) | ≈0.82 m/s(0.805〜0.821) | 1.20〜1.33 |
| **duty=0.35, thrust_scale=1.0** | **≈0.985 m/s**(cmd_vx=1.50でmeas_vx=0.985) | 1.50 |

`thrust_scale`を引き上げるだけで、`duty=0.35`の速度上限が
`duty=0.50`基準(≈0.99 m/s)にほぼ完全に並んだ(差0.3%未満)。
`contact_phase_mismatch`はcmd_vx=1.50時点で14.5%(duty=0.50
基準の8.3%よりまだ高いが、`thrust_scale=0.4`版の18.4%からは
大幅改善)。

**結論**: 「跳躍しながら速く動けるか」への答えは**Yes、ただし
条件付き**——`duty_factor=0.35`(30%滞空/周期)のまま
`thrust_scale=1.0`に上げれば、`duty=0.5`基準とほぼ同じ速度上限
(≈0.99 m/s)を跳躍ありで達成できる。§5bqで見つけた「dutyを下げると
速度上限が下がる」という劣化は、物理限界ではなく`thrust_scale=0.4`
という保守的な選択がduty=0.35では過剰に効いていたことが主因だった。

**注意(限界安定の疑い)**: 動画撮影用に`total_time_s`を4.5sへ延長
して同一構成を再実行したところ、`meas_vx=0.787`・`peak_pitch=
0.349rad`まで悪化する挙動が一度観測された(3.0s版では再現的に
`meas_vx=0.985`)——この構成(`duty=0.35`+`thrust_scale=1.0`+
`cmd_vx=1.50`)はスイープが検証した2.5秒の窓では上限に並ぶが、
それより長時間の安定性は未確認、むしろ一度は不安定化する側の証拠が
出ている。動画(`tests/media/go2_bound_flight_phase_duty035_
thrust1_ceiling_match.mp4`、3.0s版・再現するmeas_vx=0.985の方を
採用)は跳躍と速度の両立を示すが、この短時間窓に限った実証である
点に注意。

**次の一手候補**: (a) `thrust_scale=1.0`+`duty=0.35`を長時間
(`total_time_s`を8〜10s程度に延ばして)再実行し、限界安定か
本当に発散するのかを確認する(優先度高)、(b) 0.6〜0.8および
今回見つかった長時間側の共振的不安定の原因を調査する、(c) さらに
duty=0.30/0.25でも同様に`thrust_scale`を引き上げて上限が回復
するか確認する(任意)。

### 5bs. §5br(a)を検証 — 10秒の時間窓別分析で判明: 「上限に
並んだ」結果は3秒ほどで崩壊する一過性の現象だった(2026-07-24)

§5brの`duty=0.35`+`thrust_scale=1.0`+`cmd_vx=1.50`が「限界安定
(marginal)」か「本当に発散するか」を確認するため、`total_time_s=
10.0`で実行し、`report_time_windowed_summary`(新規、1秒バケツ
ごとにvx/peak_pitch/peak_roll/min_z/contact_phase_mismatchを
集計)で時系列を追った。結果は「限界安定」ではなく**明確な崩壊**
だった:

| 時間窓 | vx | peak_pitch | contact_phase_mismatch | min_z |
|---|---|---|---|---|
| 0-1s(起動直後) | 0.148 | 0.102rad | 8.8% | 0.229m |
| 1-2s | **1.089** | 0.100rad | 13.4% | 0.236m |
| 2-3s | **1.218** | 0.112rad | 14.0% | 0.231m |
| 3-4s | **0.530**(崩壊開始) | **0.349rad** | 22.4% | 0.213m |
| 4-5s | 0.157 | 0.232rad | 40.0% | 0.213m |
| 5-6s | 0.091 | 0.179rad | 34.8% | 0.212m |
| 6-7s | 0.140 | 0.365rad | 32.8% | 0.210m |
| 7-8s | 0.070 | 0.311rad | 42.2% | 0.207m |
| 8-9s | 0.340 | 0.225rad | 26.2% | 0.222m |
| 9-10s | 0.306 | 0.195rad | 25.4% | 0.217m |

最初の3秒間は実際にvx≈1.1〜1.2 m/sまで到達し(§5brがスイープで
見た0.985より高いピークすらある)、`contact_phase_mismatch`も13〜
14%程度と(この構成としては)悪くない。しかし**t=3〜4s**の窓で
突然`peak_pitch`が0.35rad近くまで跳ね上がり、以降t=10sまで一度も
回復しない——vxは0.07〜0.34 m/sの間で不規則に上下し続け、
`contact_phase_mismatch`は25〜42%まで悪化したまま。転倒はしない
(`min_z`は常に0.207〜0.236m、`finite=true`)ものの、実用的な
定常歩行としては崩壊している。生ログでも`stance=1/4`・`stance=
0/4`(設計上のduty=0.35の滞空期より長い、想定外の無接地)が崩壊後
に頻出しており、PLLの`mean_error`も崩壊前(0.004〜0.018s)から
崩壊後(-0.037〜+0.026s)で振れ幅が拡大——接触タイミング追従が
実際に発散している。

**結論**: §5brの「`duty=0.35`+`thrust_scale=1.0`で`duty=0.5`の
速度上限(≈0.99 m/s)に並んだ」という結果は、**限界安定ではなく
一過性の遷移現象**だった——3秒以内に測定を打ち切っていたスイープ
(`go2_wbc_bound_flight_phase_duty035_thrust_scale_1_ceiling_sweep`
等、`total_time_s=3.0`固定)がたまたま崩壊前の"良い"窓だけを
捉えていた。§5bq/5brのスイープ結果(cmd_vxやthrust_scaleとの
関係)自体は同一条件下での相対比較としては依然有効だが、
「`duty=0.35`+`thrust_scale=1.0`が実用的な高速構成である」という
§5brの結論は**撤回**する。動画(`go2_bound_flight_phase_duty035_
thrust1_ceiling_match.mp4`)も、崩壊前の一時的な良好区間を切り
取ったものであり、定常状態の実演ではない点に注意。

**次の一手候補**: (a) `duty=0.5`基準側も同様に長時間(10s)で
崩壊しないか確認する(短時間スイープへの依存を疑う一般的な教訓
として優先度高)、(b) 崩壊のトリガー(t=3〜4s付近で何が起きて
いるか、PLLの`mean_error`符号反転のタイミングと相関するか)を
詳しく調べる、(c) 現時点で「跳躍しながら速く動く」という当初の
目標は未達成のまま──`duty<0.5`は見た目の要望(滞空期がある)は
満たすが、速度面でduty=0.5を上回る/並ぶ持続可能な構成はまだ
見つかっていない、と正直に記録する。

**(a)の検証結果(同日追記)**: `go2_wbc_bound_duty050_baseline_
long_duration_stability`で`duty=0.50`・`thrust_scale=0.4`
(§5bp確立済みの基準)を同じcmd_vx=1.50・10秒で実行したところ、
**全区間で安定**だった:

| 時間窓 | vx | peak_pitch | contact_phase_mismatch |
|---|---|---|---|
| 0-1s | 0.171 | 0.097rad | 8.4% |
| 1-2s | 1.050 | 0.092rad | 5.0% |
| 2-3s | 1.199 | 0.095rad | 7.4% |
| 3-4s | 1.234 | 0.119rad | 9.2% |
| 4-5s | 1.201 | 0.087rad | 5.4% |
| 5-6s | 1.256 | 0.086rad | 6.4% |
| 6-7s | 1.313 | 0.086rad | 7.2% |
| 7-8s | 1.301 | 0.078rad | 7.6% |
| 8-9s | 1.325 | 0.080rad | 7.6% |
| 9-10s | 1.301 | 0.085rad | 8.8% |

vxは1.0〜1.33 m/sの範囲で終始安定、peak_pitchも0.08〜0.12rad
のまま変動なし、mismatchも5〜9%で推移——`duty=0.35`版で見られた
t=3〜4s付近の崩壊は一切見られない。これにより**短時間スイープ
という手法自体は問題ではなく**(`duty=0.5`はこの手法で正しく
「安定」と判定できている)、崩壊は`duty=0.35`+`thrust_scale=1.0`
という組み合わせ固有の現象であることが確定した。この対比により
`duty=0.35`側の崩壊がいかに際立つか(1-3sは`duty=0.5`と遜色ない
vx=1.09〜1.22を記録していた)も裏付けられた。

**最終結論(このセッションの`duty<0.5`拡張について)**: 数式
一般化(Phase B)は正しく実装でき、`n_stance=0`経路も短時間なら
生存する。しかし**速度面で`duty=0.5`を上回る、あるいは持続的に
並ぶ安定した構成は最終的に見つからなかった**——`thrust_scale=1.0`
は3秒までは有望に見えたが、それ以降崩壊する不安定解だった。
「跳躍しながら速く動く」という当初の目標には未到達。`duty<0.5`
自体は「4脚とも遊脚になる瞬間がある」という見た目の要望には
引き続き応えられる(オプトイン機能として実装済み、`duty=0.5`は
無傷)ため、速度よりも見た目(跳躍感)を優先するならそのまま
使える。速度を維持したまま跳躍させるには、崩壊トリガー(t=3〜4s
付近の詳細調査)の原因究明、またはより根本的な力生成方式の見直し
(§5bq(b)で触れた、区分定数でなく時間変化するF_xサイジング)が
必要になる。

### 5bt. 崩壊への文献的仮説 — capture point再有効化を試すも
むしろ悪化、仮説は棄却(2026-07-24)

§5bsの崩壊(t=3〜4sで発生)について、先行文献(Raibert 1986の
跳躍ロボット3分解制御、Di Carlo et al. 2018のMIT Cheetah 3
convex-MPC Bound、Park/Wensing/Kim 2017のインパルス・スケーリング)
はいずれも、速度誤差の主要な訂正手段として**着地位置(footstep
placement)のリアルタイム補正**(`x_foot ~ v̄・T_st/2 + k・
(v-v_des)`)に依存している——滞空中は力を出せないので、次の着地
位置をずらすことだけが速度誤差を消す手段になるという考え方。
このセッションで確立してきたBound最良構成は全て`capture_point_
gain_override: Some(0.0)`(§5ab、Trot由来の交絡因子を排除する
ため)——まさにこの着地位置フィードバックを完全に無効化していた
ため、これが崩壊の原因ではないかという仮説を立てた。

**検証**: `go2_wbc_bound_flight_phase_duty035_capture_point_
reenabled_stability`で、崩壊した構成(duty=0.35, thrust_scale=
1.0, cmd_vx=1.50)にWBCの既定capture point gain(`DEFAULT_
CAPTURE_POINT_GAIN_S=0.05`)を再度有効化して同じ10秒プローブを
実行:

| 時間窓 | vx | planar_speed | peak_pitch | peak_roll | mismatch |
|---|---|---|---|---|---|
| 0-1s | 0.107 | 0.107 | 0.099rad | 0.000rad | 6.8% |
| 1-2s | 0.528 | 0.528 | 0.082rad | 0.016rad | 11.4% |
| 2-3s | 0.451 | 0.485 | 0.102rad | 0.035rad | 20.2% |
| 3-4s | 0.094 | 0.792 | 0.211rad | 0.066rad | 30.2% |
| 4-5s | 0.177 | 0.226 | 0.222rad | 0.089rad | 23.4% |
| 5-6s | -0.083 | 0.125 | 0.188rad | **0.143rad** | 41.4% |
| 6-7s | -0.142 | 0.194 | 0.145rad | 0.134rad | 33.4% |
| 7-8s | -0.313 | 0.811 | 0.117rad | 0.043rad | 22.2% |
| 8-9s | **-1.091** | 1.408 | 0.084rad | 0.039rad | 17.4% |
| 9-10s | -0.866 | 0.999 | 0.173rad | 0.047rad | 24.2% |

**仮説は棄却——むしろ悪化した**。§5bsの`k_capture=0`版が最初の
2秒で到達したvx=1.05〜1.09に対し、この版は1〜2sの時点で早くも
vx=0.528止まり(そもそも良好区間自体が短く・弱い)。さらに`peak_
roll`が§5bs版では終始0.04rad未満だったのに対し、この版では
5〜6s時点で0.143radまで悪化——**新たなロール不安定**が導入されて
いる。後半(t=8〜9s)は`vx`が-1.09まで負に振れる一方
`planar_speed`は1.4付近を保つ——§5boで確立した「world-x単独では
後退と方向転換を区別できない」パターンと一致し、機体が旋回して
しまっている(yaw_pd_gainは有効なままだが、それでも抑えきれて
いない)。

**解釈**: capture point(k_capture)は元々Trot用にチューニングされた
フィードバック則で、Bound用の閉形式トリム(周期的なF_x/ピッチ
スケジュール)と同時に動かすと、両者が同じ自由度(前後方向の力
配分)を取り合って干渉する——§5abで`k_capture=0`が選ばれたのは
まさにこの交絡を避けるためだった。`duty<0.5`にしても、この
干渉構造そのものは変わらないどころか、滞空期という新しい自由度が
追加された分、より複雑に絡み合って悪化したと考えられる。

**結論**: 「capture pointを再有効化するだけで直る」という単純な
仮説は誤りだった。崩壊の真因はまだ特定できていない——次に
調べるべきは、`k_capture`のような既存フィードバックの有効/無効
ではなく、崩壊が始まるt=3〜4s前後で実際に何が起きているか
(PLLの`mean_error`符号、`stance=N/4`の異常値、WBCのQP警告頻度
など)を直接観察することだと考えられる。

### 5bu. t=3〜4s崩壊の直接観察 — 診断出力を一時的に25Hz化して
特定: PLLの補正周期(1秒に1回)が追いつかない速さで発散していた
(2026-07-24)

`wbc_walk_go2.rs`の`[diag k=...]`出力間隔を一時的に(`k % 250`→
`k % 20`、0.5s→0.04s)細かくして、崩壊した構成(`k_capture=0`版、
duty=0.35・thrust_scale=1.0・cmd_vx=1.50)のt=2.4〜4.6s区間を
直接観察した(調査後に`k % 250`へ復元済み、他のテストへの影響
なし)。

**観察された時系列**:
- t=2.4〜3.12s: `pitch_meas`は±0.09rad程度で緩やかに振動、`pitch_
  ref`(±0.16rad付近)と大きくは乖離していない。ただし`stance=
  0/4`(t=2.56, 2.92)や`stance=1/4`(なし、まだ)など、想定される
  `2/4`(支持)/`0/4`(滞空)以外の接触カウントが散発し始める。
  `max|τ|=45.43 N·m`(トルク上限に張り付いた値と見られる)も
  t=2.68, 2.72あたりから頻出し始める。
- **t=3.04〜3.68s(崩壊の起点)**: `stance=1/4`(t=3.08, 3.24〜
  実際は3.28, 3.64)が繰り返し出現——支持ペアの片脚だけが接地する
  という、duty=0.35のスケジュール上ありえない状態。この間
  `pitch_meas`が+0.087(t=3.12)→-0.178(t=3.32)→**-0.239
  (t=3.68)**と、およそ0.3秒で符号反転を伴いながら急激に発散
  し始める。
- **t=3.72〜4.00s(ピーク)**: `pitch_meas`が**-0.348rad
  (t=3.80)**まで達し(`theta_peak`理論値0.159radの2倍以上)、
  同時に胴体高さ`z`が0.295〜0.298m(定常時0.24〜0.26mから明確に
  逸脱、通常より高く浮いている)まで上昇——実際に大きく跳ね
  上がってしまっている。
- **t=4.00s以降**: 一度も回復せず、`stance=3/4`・`stance=4/4`
  (t=4.08、duty=0.35なら本来滞空期のはずの瞬間に全脚接地)まで
  出現するようになり、接触タイミングが完全に破綻したまま推移する。

**解釈**: `duty=0.5`(隙間なくタイル状)と違い、`duty=0.35`は
実際の接地/離地タイミングがスケジュールから多少ずれても、その
ズレを吸収する「隙間」(滞空期)がある——これは自由度である
と同時に、ズレがそのまま蓄積できる余地でもある。この構成が
使うPLL(`update_interval_s=1.0`、§5bg以来`duty=0.5`用に
チューニング)は1秒に1回、平均化した位相誤差でしか`cycle_
period_s`を補正しない。しかし今回観察された発散は**1秒未満
(t=3.04→3.68の約0.6秒)で符号反転を伴う急激な暴走**に至って
おり、1秒に1回の補正では原理的に間に合わない——`duty=0.5`用の
補正周期が、滞空期という新しい自由度が生む速い位相誤差の蓄積に
対して遅すぎる、というのが崩壊の直接原因だと考えられる
(Iida & Pfeifer流のCPG適応周期発振器の文脈で言えば、
entrainmentの時定数が外乱の時定数より遅い状態)。

**次の一手候補(優先度高)**: `duty<0.5`の構成に限り、PLLの
`update_interval_s`を1.0sより大幅に短く(0.2〜0.5s程度)して
同じ10秒安定性プローブを再実行し、崩壊が抑えられるか確認する。
§5bg〜5biの`update_interval_s`スイープが「短い方が一貫して
良い」という傾向を既に示していたことも、この方向性を支持する。

### 5bv. §5buの仮説を検証 — PLL補正周期を短縮したところ崩壊が
実際に抑えられ、10秒平均でduty=0.5基準を上回った(2026-07-24)

崩壊した構成(`duty=0.35`, `thrust_scale=1.0`, cmd_vx=1.50)で
`update_interval_s`を1.0/0.5/0.3/0.2sと段階的に短縮し、同じ10秒
安定性プローブ(`go2_wbc_bound_flight_phase_duty035_pll_interval_
stability_sweep`)を実行した。1秒ごとのvx(m/s、t=1〜10sの9窓、
起動直後の0-1s窓は除く)の平均・最小・最大:

| update_interval_s | 平均vx | 最小vx | 最大vx |
|---|---|---|---|
| 1.0(§5bs基準) | 0.438 | 0.070 | 1.218 |
| 0.5 | 0.782 | 0.079 | 1.233 |
| 0.3 | 0.888 | -0.021 | 1.374 |
| **0.2** | **1.092** | **0.587** | 1.348 |

**明確な傾向**: 短縮するほど平均速度が上がり、崩壊からの回復力も
強くなる。`1.0s`は崩壊後(t=3〜4s)一度も回復しない。`0.5s`は
崩壊がt=7〜8sまで先送りされるだけで、やはり回復しない。`0.3s`は
t=5〜7sに一時的な落ち込み(vxが-0.021まで)があるが、t=7s以降
**自力で回復**し始めた最初のケース。`0.2s`では最小vxが0.587
(常に前進を維持、後退・停止なし)、9窓平均1.092 m/sは
`duty=0.5`基準(§5bs実測: 1.0〜1.33 m/sで安定)にほぼ並ぶか
上回る水準——t=6〜7s・t=8〜9sに一時的な乱れ(peak_pitchが
0.19〜0.28radまで上昇)はあるものの、いずれも1秒以内に回復して
いる。

**結論**: §5buの診断(PLLの補正周期がduty<0.5の速い位相誤差蓄積
に対して遅すぎる)は正しく、`update_interval_s`を短縮するという
直接的な対策で実際に改善することを確認した。`update_interval_s=
0.2`は崩壊を完全にではないが実用上十分に抑え、10秒間持続的に
前進を維持しながら`duty=0.5`基準と同等以上の平均速度を達成した
——「跳躍しながら速く動く」という当初の目標に、このセッションで
初めて実質的に到達したと言える。ただし一時的な乱れ(peak_pitch
の局所的スパイク)は残っており、完全な定常安定ではない点、また
10秒より長い時間での検証はまだ行っていない点に注意。

**次の一手候補**: (a) `update_interval_s=0.1`などさらに短い値や、
`gain`(現状0.10のまま)も併せて調整する余地を探る、(b) この
構成(`duty=0.35, thrust_scale=1.0, update_interval_s=0.2`)で
より長時間(20〜30秒)の安定性を確認する、(c) 動画撮影して
実際の跳躍+速度の両立を確認する、(d) 他のcmd_vx点でも同様に
`update_interval_s`短縮が効くか確認する。

### 5bw. (a)を実施 — さらに短縮したところ非単調、0.2sが
局所最適点だった(2026-07-24)

§5bvの傾向(短いほど良い)を追試するため、`0.20/0.15/0.10/0.05`
で同じ10秒プローブを実行(`go2_wbc_bound_flight_phase_duty035_
pll_interval_fine_sweep`)。9窓平均vx:

| update_interval_s | 平均vx | 備考 |
|---|---|---|
| **0.20** | **1.092** | §5bv、最良 |
| 0.15 | 0.300 | t=6〜10sで**負に転落**(-0.22〜-0.72) |
| 0.10 | 0.865 | 部分回復 |
| 0.05 | 0.899 | 部分回復、ただしt=9-10sで0.077まで落ち込み |

**単調ではなかった**——`0.15s`は`0.20s`よりむしろ大幅に悪化し、
`0.10`/`0.05`は`0.15`より良いが`0.20`には届かない。`0.15s`は
`cycle_period_s≈0.18s`(この構成のT)にかなり近く、PLLの補正
タイミングと歩容周期そのものが特定の比率で共振・エイリアシング
を起こしている可能性が高い(このセッション全体で繰り返し観測
されてきた「単一の万能パラメータがなく、非単調な共振構造がある」
というパターン、Zaytsev/Remyの分岐解析の文脈と整合)。

**結論**: `update_interval_s`は単純に短くすればするほど良い
わけではなく、**0.2sがこのスイープで見つかった局所最適点**
——これを「跳躍しながら速く動く」構成の最良パターンとして採用し、
動画撮影に進む。

動画撮影(`go2_wbc_bound_flight_phase_duty035_best_pattern_video_
source`、8.5秒、`tests/media/go2_bound_flight_phase_duty035_
best_pattern.mp4`)——duty=0.35(滞空期あり)・thrust_scale=1.0・
PLL update_interval_s=0.2・cmd_vx=1.50。CSV再生時の時間窓別実測は
10秒スイープの該当区間と厳密に一致(t=1-6sはvx=1.03〜1.35で安定、
t=6-7sに一時的な落ち込み0.587、t=7-8sで1.092まで回復)——崩壊せず
自力回復する様子も含め、都合の良い区間だけを切り取らずそのまま
採用した。

### 5bx. ユーザー指摘「ストライドは上げていないのか」— 検証した
ところ実際に効果あり、平均速度+21%(2026-07-24)

このセッションで動かしてきた`cmd_vx`・`duty_factor`・`thrust_
scale`・`cycle_period_s`・PLL`update_interval_s`は、いずれも
`max_step_length_m`(ストライド長、Boundの既定0.12m)には手を
付けていなかった。footstep plannerの理論上限`v_max=max_step_
length_m/(cycle_period_s・duty_factor)`は§5bwの最良構成で
0.12/(0.18・0.35)≈1.9 m/sとなり、実測到達済みの~1.3 m/sより
十分上だったため優先度を下げていたが、ユーザー指摘を受けて実際に
確認した。

**検証**: §5bwの最良構成(duty=0.35, thrust_scale=1.0, PLL
update_interval_s=0.2)のまま、`cmd_vx=2.20`(これまでで最も
攻めた指令値)で`max_step_length_m`を0.12/0.15/0.18/0.22/0.26m
とスイープ、10秒プローブ:

| max_step_length_m | 理論v_max | 9窓平均vx | 最大vx |
|---|---|---|---|
| 0.12(既定) | 1.905 m/s | 1.005 | 1.775(t=6-7s) |
| 0.15 | 2.381 m/s | 1.035 | 1.729(t=3-4s) |
| **0.18** | 2.857 m/s | **1.220** | **1.799**(t=3-4s) |
| 0.22 | 3.492 m/s | 1.220(0.18と完全一致) | 1.799 |
| 0.26 | 4.127 m/s | 1.220(0.18と完全一致) | 1.799 |

**0.18mで頭打ち**——0.22m・0.26mは0.18mと全時間窓で数値まで
完全一致しており、この構成・このcmd_vxではfootstep plannerが
そもそも0.18m以上のストライド予算を使っていない(理論上限だけ
上げても、実際に要求されるストライドがそこまで届いていない)ことが
分かる。`0.18m`は`0.12m`比で9窓平均+21%(1.005→1.220)、最大速度も
1.775→1.799とわずかに向上、かつt=3〜7sの4秒間連続でvx=1.7〜1.8
m/sを維持——これまでのどのcmd_vx点よりも高い持続速度域に到達した。
崩壊パターン自体(t=7s以降の劣化)は変わらず残っている。

**結論**: ユーザーの指摘は正しかった——ストライド長は実際に
効果があり、`max_step_length_m=0.12m→0.18m`だけで持続速度域が
明確に底上げされた(0.18mが実質的な天井、それ以上は使われない)。
`ref/wbc_comparison.md`の記録としては、§5bwの最良構成に
`max_step_length_override=0.18`を追加した構成を新しい「最良
パターン」として更新する。崩壊(t=7s以降)への対策は依然未解決
のまま。

### 5by. t=7s+崩壊の再調査(新・最良パターン) — PLLの
`cycle_period_s`が無制限に単調ドリフトしていた(積分ワインドアップ)
(2026-07-24)

`max_step_length_m=0.18`を組み込んだ新・最良パターン
(`go2_wbc_bound_flight_phase_duty035_best_pattern_video_source`、
cmd_vx=2.20)で、§5buと同じ手法(診断出力を一時的に25Hz化、
調査後`k % 250`へ復元済み)でt=6.0〜8.5s区間を直接観察した。

**観察された`cycle_period_s`の推移**(`[pll]`ログより):

| t | mean_error | cycle_period_s |
|---|---|---|
| 6.02s | +0.0108s | 0.1982 → 0.1993 |
| 6.42s | +0.0065s | 0.1997 → 0.2004 |
| 6.83s | +0.0077s | 0.2009 → 0.2016 |
| 7.23s | +0.0150s | 0.2026 → 0.2041 |
| 7.63s | **+0.0394s** | 0.2060 → 0.2099 |
| 8.03s | +0.0197s | 0.2111 → 0.2130 |
| 8.27s | +0.0216s | 0.2130 → 0.2152 |
| 8.47s | **-0.0172s**(符号反転) | 0.2152 → 0.2135 |

`mean_error`がt=6.0〜8.27sの間**一貫して正符号**(「遅すぎる」判定)
のまま、`cycle_period_s`が0.198→0.215まで**単調に増加し続けて
いる**——`update_interval_s=0.2`ごとの小さな正の補正が、8秒強の
間ほぼ休みなく積み重なった結果。クランプ(`min_period_s=0.14`,
`max_period_s=0.26`)の範囲内には収まっているが、このセッション
全体で繰り返し確認してきた「`cycle_period_s`は0.18付近の狭い
範囲でしか良い挙動を示さない、非単調な共振構造がある」という
知見(§5bg以降)に照らせば、0.215は既にその"良い範囲"を大きく
外れている可能性が高い。t=8.28sで`stance=3/4`・t=8.32sで
`stance=4/4`(想定外の接触)が現れ、t=8.48sで`pitch_meas=
-0.2504`まで発散が始まる——§5buで見た崩壊と同じパターンだが、
今回は**引き金が「速い外乱」ではなく「PLL自身の緩慢な片方向
ドリフトが8秒以上かけて許容範囲の外まで運んだこと」**だった点が
異なる。

**解釈**: このPLLは`mean_error`をそのまま`cycle_period_s`に
積分し続ける、リーク(漏れ)のない積分器として実装されている
——一貫して同符号の小さな`mean_error`(実際のタイミング誤差か、
モデル不一致由来の系統的バイアスかは未特定)が続く限り、
`cycle_period_s`はクランプの許す限りどこまでも(古典的な
「積分ワインドアップ」)動いてしまう。0.14〜0.26という広い
クランプ自体が、この構成の実際の良好範囲(0.18近辺)より大幅に
広すぎることが、今回のドリフトを許してしまった直接の原因と
考えられる。

**次の一手候補(優先度高)**: `min_period_s`/`max_period_s`を
0.18近辺(例: 0.16〜0.20)まで絞り込み、同じ8.5秒(またはより
長い)プローブでドリフトが実際に抑えられ崩壊を防げるか確認する。

### 5bz. §5byの対策を検証 — PLLクランプを0.16〜0.20に絞ったところ
15秒間崩壊が完全に消えた(2026-07-24)

同じ最良パターン(duty=0.35, thrust_scale=1.0, max_step_length_m=
0.18, cmd_vx=2.20)で、PLLのクランプだけを`0.14〜0.26`→`0.16〜
0.20`に絞り込み、より長い15秒プローブを実行
(`go2_wbc_bound_flight_phase_duty035_best_pattern_tight_pll_clamp_
stability`)。結果は**15秒間、崩壊が一度も起きなかった**:

| 時間窓 | vx | peak_pitch | mismatch |
|---|---|---|---|
| 1-2s(起動中) | 0.455 | 0.216rad | 20.0% |
| 2-3s | 1.278 | 0.108rad | 11.2% |
| 3-4s | 1.774 | 0.095rad | 9.6% |
| 4-8s | 1.775〜1.827 | 0.084〜0.114rad | 9.0〜15.0% |
| 8-11s | 1.647〜1.881 | 0.097〜0.113rad | 11.8〜16.2% |
| 11-15s | 1.374〜1.638 | 0.111〜0.136rad | 14.2〜18.4% |

t=3s以降(定常化後)の12窓平均vx=**1.697 m/s**、最小でも1.374
m/s——§5byで見られた「PLLの`cycle_period_s`が0.198から0.215まで
単調ドリフトし、8秒強で崩壊」というパターンは一切現れなかった。
peak_pitchも終始0.08〜0.14rad程度で安定、mismatchも9〜18%の
レンジで推移し、著しい悪化はない。転倒もなし(`min_z`は常に
0.23〜0.24m)。

**結論**: §5byの診断(PLLの積分ワインドアップ、クランプが広
すぎて既知の良好範囲=0.18近辺を大きく外れるまでドリフトできて
しまう)は正しく、対策(クランプを既知の良好範囲近辺に絞る)も
的確に効いた。「跳躍しながら速く動く」構成が、初めて**15秒間
持続的に安定**した——これを新しい最良パターンとして正式に確立
する:`duty_factor=0.35`, `thrust_scale=1.0`, `max_step_length_m=
0.18`, PLL(`gain=0.10`, `update_interval_s=0.2`,
`min_period_s=0.16`, `max_period_s=0.20`), `cmd_vx=2.20`
——平均速度1.7 m/s級、`duty=0.5`基準(≈1.0〜1.3 m/s)を明確に
上回る。動画を撮り直し、この構成を`go2_wbc_bound_flight_phase_
duty035_best_pattern_video_source`の正式な既定として反映する。

### 5c0. 起動直後の遷移を滑らかにする試み — 2案とも逆効果、
一方は実際に転倒した(2026-07-24)

動画(§5bzの最良パターン)のt=0〜2s区間(vx=0.026/0.455、
peak_pitch=0.207/0.216rad、定常状態の~1.7 m/s/~0.1radより明確に
粗い)について、ユーザーから「ストライドを伸ばしたりサイクルを
変更したりで滑らかに、位置・速度が単調増加するように遷移できるか」
との指摘。

**実装**: 既存の`cmd_vx_ramp_s`(§5bo、cmd_vxを0→targetへ線形ランプ)
と同じパターンで、`max_step_length_ramp_start_m`/`cycle_period_
ramp_start_s`を新規追加(quadruped-gait側に`set_max_step_length_m`
も新規実装、`set_cycle_period_s`と同じ位置付け)。`cmd_vx_ramp_s`
と同じ期間で、ストライドは`start_m→target`、サイクル周期は
`start_s→target`へ線形ランプする。

**検証1(3つとも同時にランプ)**: cmd_vx 0→2.20、ストライド
0.06→0.18m、周期0.24→0.18sを2.0秒かけて同時にランプ
(`..._smooth_startup`)。**結果は全区間で悪化**——12.5秒間
一度も§5bzの定常速度(~1.7 m/s)に到達せず、peak_pitchが0.1〜
0.44radの間で終始高止まりしたまま推移した。転倒はしない
(`min_z`は常に0.21〜0.23m)ものの、「粗い立ち上がり」どころか
「ずっと粗いまま」になってしまった。

**検証2(cmd_vxのみランプ、ストライド/周期は据え置き)**: 原因の
切り分けとして、ストライド/周期のランプを外しcmd_vxだけを2.0秒で
ランプ(`..._cmd_vx_ramp_only`)。**さらに悪化し、t=9.0〜9.5sで
実際に転倒した**(`peak_roll=3.044rad`≈π、`min_z`がt=9.5s以降
**負の値**(-0.21〜-0.25m、物理的に地面にめり込んでいる)のまま
最後まで回復せず)。

**解釈**: `thrust_scale`ベースのF_xサイジング(`F_x_used=
thrust_scale・f_x_clipped()`)は**cmd_vxを一切参照しない**——
`velocity_ripple_fraction`モード(§5bj)と違い、cmd_vxがいくら
小さくても、trimの周期的ピッチ/F_xスケジュールは物理パラメータ
(質量・摩擦・duty)だけで決まる「フル強度」のまま最初から発動
している。つまり素の(ランプなし)最良パターンが起動直後の粗さを
自然に乗り越えられていたのは、cmd_vx・ストライド・周期・trim
強度の**4つ全てが最初から目標値で揃っていた**からで、この一貫性
が実は起動の安定性を支えていた。cmd_vxだけ、あるいはcmd_vx+
ストライド+周期だけを遅らせると、trim強度(フル)と実際の指令
速度・接地パターンとの間に新しい不整合が生まれ、素の粗さより
かえって悪化する——検証2の転倒はその極端な例。

**結論**: 「なめらかに立ち上げる」という直感は自然だが、この
アーキテクチャ(thrust_scaleがcmd_vx非依存)では**部分的な
ランプがかえって新しい不整合を生む**ことが分かった。素の
(ランプなし)起動の粗さは2秒程度で自己収束し転倒もしないため、
実害としては小さい。次に試すとすれば`bound_trim_thrust_scale`
自体もcmd_vxと同期してランプする(quadruped-gait側に
`set_bound_trim_thrust_scale`が既存)案があるが、2連続で逆効果
だった実績を踏まえ、ユーザーに次の方針を確認してから進める。

**追試(ユーザー質問)**: 「cmd_vxを10秒かけて0→2.2にランプすると
どうなるか」を確認(`..._cmd_vx_ramp_only`の2.0秒版と同じ、
ストライド/周期は据え置き、ランプ時間だけ10.0秒に延長)。

**結果**: ランプを緩やかにしても問題は解消せず、**むしろ長く
苦しんだ末に転倒した**。ランプ中(t=0〜10s)は終始vxが低く
不規則(0.005〜0.55 m/s、指令されているはずの線形ランプに全く
追従できていない)、peak_pitchも0.10〜0.39radの間で断続的に
悪化を繰り返す。ランプ完了(t=10.5s、cmd_vx=2.20に到達)から
約5秒後、**t=15〜16sで実際に転倒**(peak_roll=3.142rad≈π、
min_zが-0.22m前後の負値のまま最後まで回復せず)——2.0秒版
(t=9〜9.5sで転倒)より長く持ちこたえただけで、結末は同じ
だった。

**結論**: ランプを長くしても、trim強度(フル、cmd_vx非依存)と
実際の指令速度・接地状態との不整合という根本原因は解消しない
——長いランプは「苦しむ時間」を延ばすだけで、最終的な転倒は
避けられなかった。cmd_vxだけを遅らせる方向のランプは(短くても
長くても)このアーキテクチャでは逆効果と結論づけられる。

### 5c1. `thrust_scale`自体もcmd_vxと同期してランプ — 起動直後は
明確に改善したが、t=2.5〜6sに新しい荒れとヨードリフトが残った
(2026-07-25)

§5c0の診断(trimの周期的F_x/ピッチスケジュールが`thrust_scale`
だけで決まり、cmd_vxを一切参照しない)を踏まえ、`bound_trim_
thrust_scale`自体もcmd_vxと同じ2.0秒ランプに同期させた
(quadruped-gaitに既存の`set_bound_trim_thrust_scale`を使用、
新規`thrust_scale_ramp_start`ノブ経由: 0.2→1.0)。ストライド/周期は
据え置き(§5c0で悪化要因と分かったため)。

**結果(混在)**——**起動直後(t=0〜2s)は明確に改善**:

| 時間窓 | vx | peak_pitch |
|---|---|---|
| 0-0.5s | -0.014 | 0.010rad |
| 0.5-1.0s | 0.008 | 0.117rad |
| 1.0-1.5s | 0.300 | 0.088rad |
| 1.5-2.0s | 0.664 | 0.096rad |
| 2.0-2.5s | 1.117 | 0.129rad |

vxが単調に近い形で滑らかに立ち上がり、peak_pitchも終始0.01〜
0.13rad程度——素の動画(t=0-1s: vx=0.026・pitch=0.207rad、
t=1-2s: vx=0.455・pitch=0.216rad)と比べて明確に穏やか。`F_x`の
「蹴り」自体を弱めるという仮説(`thrust_scale`を下げると
theta_peak=参照ピッチは増えるが、それはPDで追従される参照であって
生の外乱ではない)は的中した。

**しかし、t=2.5s以降(ランプ完了直後)に新しい荒れが発生**:
peak_pitchが0.279〜0.429radまで再び悪化(t=2.5〜6s)、その後
t=6.5〜11sでは`vx`と`planar_speed`が大きく乖離(例: t=10.0-10.5s
vx=0.665 vs planar=1.704)——機体が大きく旋回していることを示す
(§5boと同じパターン)。t=11s以降は`vx`が-0.167〜-0.350まで負に
転落。転倒はしない(`min_z`は常に0.209〜0.241m、`peak_roll`も
0.10rad未満)ものの、§5bzで確認した定常状態(~1.7 m/s、崩壊なし)
には最終的に戻れなかった。

**結論**: `thrust_scale`ランプは狙い通り**起動直後の荒さは
明確に解消**した——これは前2回の失敗(cmd_vxだけ・cmd_vx+
ストライド+周期)と違い、転倒もしていない。ただしランプ完了
直後(t=2.5s、ちょうどPLLの蓄積再開タイミングと一致)に新しい
不安定とヨードリフトが生じており、まだ完全な解決ではない。次に
試すべきは、(a) ランプ完了とPLL再開のタイミングをずらす・
オーバーラップさせる、(b) ランプ時間を2.0sより延ばして遷移を
さらに緩やかにする、(c) ヨードリフトの原因を個別に調査する、
のいずれか。

### 5c2. (a)(b)(c)を1つずつ検証(2026-07-25)

**(a) PLL再開を1秒遅らせる(`post_ramp_settle_s=1.0`、新規追加)**:
`ramp_in_progress`の判定窓を`burn_in_s+cmd_vx_ramp_s`から
`burn_in_s+cmd_vx_ramp_s+post_ramp_settle_s`まで延長し、PLLの
蓄積再開そのものを1秒遅らせた。**結果はさらに悪化**——t=0〜2sの
改善部分は§5c1と同じだが、今回は**t=8.0〜8.5sで実際に転倒**
(`peak_roll=3.135rad`≈π、`min_z`が-0.14m以降ずっと負値のまま
回復せず)。§5c1(遅延なし)は転倒こそしなかったが、遅延を
入れたことでかえって転倒に至った——PLLによる補正そのものを
長く止めることが、蓄積するズレを是正する機会を単に先送り
しているだけで、改善にはならないことを示す。**(a)は棄却**。

**(b) ランプ時間を2.0s→4.0sに延長**(`cmd_vx`+`thrust_scale`を
同じ4.0秒で同期ランプ、PLL遅延なし): **これまでで最良の結果**。
起動区間(t=0〜5.5s)のvxが`-0.014→-0.006→0.119→0.328→0.520→
0.784→1.037→1.076→1.384→1.618→1.816`と**ほぼ単調に増加**——
ユーザーが要望していた「位置・速度が単調増加するように滑らかに
遷移」をほぼそのまま実現できている。peak_pitchもこの区間終始
0.09〜0.13rad程度で安定。転倒もなし(`min_z`は常に0.209〜
0.242m)。さらに、§5c1で顕著だった`vx`と`planar_speed`の乖離
(旋回)もこの版ではほぼ解消している(例: t=10.0-10.5s
vx=0.926 vs planar=0.927、ほぼ一致)——ランプを長くしたことが
ヨードリフトの副作用も同時に和らげたとみられる。

ただし**t=6s以降、定常状態(~1.7-1.8 m/s)には留まれず再び
乱れる**——t=5.0-5.5sのピーク(vx=1.816)の後、t=6.5-7.0sで
peak_pitchが0.447radまで悪化しvxが0.494まで低下、以降は
0.13〜1.46の間で緩やかに上下動を続ける(転倒はしない)。起動
遷移そのものは解決したが、その後の巡航状態の安定化は依然
未解決。

**(c) ヨードリフトの原因調査**: §5c1(2.0秒ランプ版)に対し、
診断出力を一時的に25Hz化してyaw/omega_zも追加出力し(調査後
`k % 250`へ復元済み)、t=0〜11.5s全体を追った。

**判明した時系列**: t=0.5〜1.6sはyaw_meas≈0で完全に保持されて
いる(`yaw_pd_gain`が正常に機能)。**t=1.64〜1.8s付近
(2.0秒ランプの終盤)で`omega_z`が突然-0.25〜-0.28rad/sまで
跳ね上がり**、yaw_measが-0.01→-0.02→-0.03→-0.037radへ
負方向にドリフトし始める。直後t=1.8sで`omega_z`は符号反転し
+0.27〜+0.47rad/sまで振れ、yawは正方向へ転じてt=3.0sには
0.09rad程度まで蓄積。以降も`omega_z`は±0.1〜0.5rad/s(後には
±1.5rad/s超)で振動を続けながら、**正味では止まらずyawが
一貫して蓄積**し続け、t=5.8sで1.08rad、t=11.3sで2.19radに
達する——`omega_z`自体は振動的(平均すればゼロに近そうに見える)
なのに、yaw角度は着実に片方向へ蓄積し続けている。

**推定原因**: 同じログで`max|τ|`が頻繁に45.43(トルク上限の
クランプ値とみられる)に張り付いており、Bound+滞空期+cmd_vx=2.20
という高負荷な組み合わせでは、ピッチ・接地力の追従だけで
アクチュエータの余力をほぼ使い切っている可能性が高い。yaw_pd_gain
のフィードバック自体は正しく動作している(t<1.6sでyawを完全に
0へ保持できている)ため、ヨードリフトの引き金は「ゲインが弱い」
ことではなく、**ランプ終盤(t=1.6〜1.8s、ちょうどthrust_scale
とcmd_vxが目標値に近づく時間帯)に発生する非対称なヨートルク外乱
に対し、他の追従要求(ピッチ・接地力)がすでにトルク予算を占有
していて、yaw補正に回せる余力が不足している**ことだと考えられる
——§5bo(3秒ランプ版、duty=0.5)で見た「ランプ由来の左右非対称
外乱」と同種の現象が、この滞空期構成でも(トルク予算がより
逼迫した状態で)再発している可能性が高い。

**結論**: (c)は根本原因の「引き金の時刻」(ランプ終盤)までは
特定できたが、その非対称外乱自体の発生源(§5boから持ち越しの
未解決問題)は今回も未特定のまま。ただし(b)(ランプを4.0秒に
延長)がこの同じ引き金の負荷を薄めることで、結果的にヨードリフト
も大幅に緩和していた(§5c2既出)ことから、**(b)は(c)の対症療法
にもなっている**という関係が見えた。

### 5c3. t=6s以降の乱れへの対処(2026-07-25)

(b)(4.0秒ランプ)にもt=6.5〜7.0s付近でpeak_pitch=0.447radまで
悪化する崩壊が残っていた。診断出力を一時的に25Hz化して追跡した
ところ(調査後`k % 250`へ復元済み)、**§5byと同じ機構**——
`cycle_period_s`がランプ終了(t=4.5s、PLLの蓄積再開タイミング)
以降0.179→0.198まで単調にドリフトし、そのままpitch_measが
-0.446radまで暴走する——が、単にランプによってPLL開始時刻が
遅れた分だけタイミングがずれて再発していることが分かった。

**まず`min_period_s`/`max_period_s`を0.17〜0.19へさらに絞り込み**
(`..._longer_ramp_tighter_clamp`)——**変化なし**。t=6.0sまで
§5bwと完全に同一の数値、t=6.5-7.0sのpeak_pitchも0.447radと
全く同じ値——クランプ幅を絞ることが今回の崩壊には効いていない
ことが判明(§5by/5bzの「クランプを絞れば直る」ケースとは異なる
機構であることが確定)。

**次に`pll_accumulate_during_ramp`(新規)を試行**——ランプ中も
PLLの位相誤差蓄積をリセットせず継続させ、巡航突入時にはすでに
「慣らし運転済み」の`cycle_period_s`で迎える案
(`..._longer_ramp_pll_warm`)。**結果は明確に改善**:

| 時間窓 | vx |
|---|---|
| 0-4.0s(起動) | ほぼ単調に0→1.392まで増加 |
| 4.0-6.5s(小さな乱れ) | 1.149→0.653→0.330→0.232→**-0.023** |
| 7.0-7.5s(回復開始) | 0.963 |
| 7.5-14.5s(巡航) | 1.28〜1.76の範囲で持続(平均約1.50 m/s) |

以前(ランプなしのPLL、`..._longer_ramp`/`..._tighter_clamp`)は
一度崩れると最後まで(t=14.5sまで)0.13〜1.46の間で不規則に
上下し続け定常状態に戻れなかったが、今回はt=4.5〜6.5sの**より
浅く短い**乱れ(最小-0.023、転倒なし、`min_z`は常に0.21〜0.24m)
の後、**t=7.5s以降は完全に持続的な巡航状態(平均1.50 m/s)に
回復**した。

**結論**: `pll_accumulate_during_ramp=true`が明確に有効——ランプ
終了時点でPLLが既にある程度の`cycle_period_s`推定値を持っている
ことで、ランプ直後のゼロからの再蓄積(§5byと同じ暴走パターン)を
避けられる。完全に無傷ではない(t=4.5〜6.5sに小さな乱れは残る)
が、崩壊が「そのまま戻らない」から「一時的な乱れの後、持続的な
巡航へ回復する」へと質的に変わった。これを新しい最良パターンとして
採用し、動画を撮影する。

### 5c4. 残るt=4.5〜6.5s乱れの原因調査 — PLLがランプ中に
`cycle_period_s`を過剰に下げすぎていた(2026-07-25)

診断出力を一時的に25Hz化しT/roll/yaw/wz/vxも追加して(調査後
復元済み)、§5c3の残る乱れ(t=4.5〜6.5s)を追った。**判明した
機構**:

- t=3.6〜4.3s(ランプ後半、まだ高速): PLLがランプ中に
  `cycle_period_s`(T)を**0.167付近まで押し下げていた**——
  クランプ下限0.16のすぐ上。巡航の調律値0.18より大幅に短い。
- t=4.32s: `stance=4/4`(本来は滞空のはずの瞬間に全脚接地)が
  出現、vxが1.23→0.78へ急落。T=0.167では巡航速度に対して
  周期が短すぎ、遊脚が振り切る前に接地する「間に合わない着地」
  が起きている。
- t=4.4〜6.0s: vxが0.05〜0.8で低迷、pitch_measが-0.32radまで
  振れる。この間PLLは誤りに気づいてTを0.167→0.18→0.20
  (クランプ上限)まで**フルレンジで引き上げ直している**。
- t=6.5〜7s: Tが0.20に落ち着いた頃、vxが0.7〜0.95へ回復し、
  以降巡航へ。

**原因**: `pll_accumulate_during_ramp`はコールドスタートの暴走
(§5by)は防いだが、**ランプ中の過渡的な力学条件(加速中・
thrust_scale増加中)に対してPLLが最適化してしまい、巡航突入時に
T=0.167という巡航には短すぎる値に収束していた**。ランプ後、PLLが
これを0.20まで(クランプ全幅を横断して)引き直す2秒間が、まさに
残っていた乱れの正体だった。要するに「PLLがランプ中に間違った
方向に慣らし運転してしまった」。

**次の一手(検証中)**: `pll_accumulate_during_ramp`は維持しつつ、
クランプを調律値0.18の周辺に狭く中心化(例: 0.175〜0.185)して、
PLLが0.167まで下げられないようにする——巡航突入時のTを0.18近傍に
留め、ランプ後の引き直し距離をゼロに近づける狙い。

**検証結果(warm-start + 中心化クランプ0.175〜0.185)**: t=4.5〜
6.5sの乱れは**完全に消えた**——それどころか起動後t=4〜8sで
**vx=1.7〜1.94 m/s**という、このセッション最速の持続巡航を4秒間
達成(dipなし)。しかし**今度はt=8s以降に新しい崩壊が出現**
(vxが0.1〜0.96で低迷、peak_pitchが0.44radまで)。つまり乱れの
「窓」がt=4.5〜6.5sからt=8s以降へ移動しただけで、消えてはいない。

**重要な含意**: クランプを±0.005まで絞るとPLLはほぼ0.18で凍結
されるが、それでも(Tが理想値でほぼ固定でも)t=8sで崩壊する
——**この崩壊はもはやPLLの問題ではない**。周期が理想でも起きる
以上、原因は§5bo/5c1(c)で持ち越してきた「接地タイミングの
非対称外乱」そのもの、あるいはPLLで補正しきれない別の位相ドリフト
の蓄積である。パラメータ調整では「乱れの窓を時間軸上で動かす」
ことしかできず、消せない——このセッション全体で繰り返し観測して
きた「単一の万能パラメータがなく、非単調な分岐構造」(Zaytsev/
Remy)のパターンそのもの。ここで個別パラメータのwhack-a-mole
(モグラ叩き)は打ち止めとし、俯瞰的な方針転換の検討へ移る
(§5c5)。

### 5c5. 俯瞰的レビュー — ここ数日のBound安定化施策の総括と、
先行文献に照らした本質的な改善方針(2026-07-25)

**これまで積み上げた安定化施策(すべて対症療法的)**:
1. 閉形式SRBDトリム参照(周期的ピッチ/Fxスケジュール)§5bb/5bc
2. sign補正(sign=-1.0)§5bc
3. thrust_scale(Fx減衰で摩擦余裕確保)§5bg
4. velocity_ripple_fraction(インパルススケーリング代替)§5bj
5. pitch_pd_gain=(100,10)§5bn
6. PLL適応周期(接地タイミング同調)§5bl/5bz
7. yaw_pd_gain=(10,1)明示的ヨー保持 §5bp
8. duty_factor<0.5 真の滞空期 §5bq-5bx
9. PLLクランプ絞り込み(ワインドアップ対策)§5bz
10. 起動ランプ(cmd_vx/ストライド/周期/thrust_scale)§5c0-5c4
11. pll_accumulate_during_ramp §5c3

**共通する構造的限界**: 上記はすべて「**フィードフォワードの
トリム参照(開ループの周期的ピッチ/Fxスケジュール)+ 軸ごとの
反応的PD(pitch/roll/yaw)+ 遅いPLL(時間クロック)**」という
アーキテクチャへのパッチである。**Bound限界周期そのものを状態
フィードバックで能動的に安定化する仕組みが一つもない**。だから
どのパラメータも非単調で、外乱は消えず時間軸上を移動するだけ
——開ループで不安定な限界周期を反応的に追いかけている証拠。

**先行文献が指す本質的な改善方向(優先度順)**:

- **(A) 足配置制御(Raibert 1986 の3分解則、最優先)**: 滞空中は
  地面反力が使えず、**次の着地位置だけが速度・バランスの唯一の
  制御権限**。Raibertの `x_foot = v̄·T_st/2 + k·(v−v_des)` が
  bounding/hopping安定化の文献標準。§5btで一度capture_pointを
  再有効化したがTrot用チューニングでトリムと競合し悪化——
  **滞空期用にサイズしたBound専用の足配置則**はまだ真に実装して
  いない。今のアーキテクチャに欠けている最大のピース。
- **(B) 滞空期を明示的に計画するMPC(Di Carlo et al. 2018,
  MIT Cheetah 3 のconvex MPC bounding)**: 固定FFトリム+反応PD
  ではなく、弾道飛行区間と接触スケジュールをMPCが予測計画する。
  articaraにFullCentroidal MPCはあるが`n_stance=0`対応は後付けで、
  滞空期を織り込んだ予測計画にはなっていない。能動的(予測的)
  安定化への王道。
- **(C) Poincaré/HZD による離散安定化**: 繰り返し当たっている
  分岐パターン(Zaytsev/Cnops/Remy 2019)は「開ループ限界周期が
  不安定」を意味する。1ストライド1回の離散制御(deadbeat/hybrid
  zero dynamics)で Poincaré 復帰写像を安定化するのが原理的な解。
  連続PDで追いかけるより筋が良い。
- **(D) イベントベース/位相ベースCPG(Iida & Pfeifer, Ijspeert)**:
  現PLLは「時間クロック」を窓平均で適応させるためワインドアップ
  を起こす(§5by/5c4)。実際の接地イベントに位相結合する適応周波数
  発振器なら、機械共振に自然同調しワインドアップ自体が生じない。
- **(E) 学習方策(memory の go2-onnx デプロイライン)**: RL方策は
  まさにこのハイブリッド力学の安定化を頑健に扱う。本プロジェクトは
  既に policy-runtime 共有クレートで sim/実機同一コードのデプロイ
  ラインを持つ。bounding はRL歩容の代表的成功例——手調整の
  whack-a-mole より遥かに少ない労力で安定する可能性が高い。

**推奨**: 現状の手調整最良パターン(§5c3、起動滑らか+巡航回復)は
「デモとしては十分見られる」水準に達しており、ここで一区切り。
本質的な次の一歩は**(A)Bound専用足配置則**——今のアーキテクチャに
最小の追加で最大の効果が見込め、(B)〜(E)の前提にもなる。ユーザーへ
方針を確認する。

### 5c6. (A)Bound専用足配置則を実装・検証 — 負の結果、ただし
重要な副次的発見あり(2026-07-25)

**実装**: quadruped-gait に `bound_fore_aft_placement_gain`(fore-aft
=body-x 軸のみのRaibert速度誤差足配置ゲイン、既定0.0で完全後方互換)
を新規追加。`compute_mpc_footstep`で `half.x += k·v_err_body.x` を
cmdベースRaibert `half` の上に加算(§5btの汎用x+y capture_pointが
横方向ノイズでロール転倒したのを避け、**fore-aft軸のみ**に限定)。
generator/gait.rs にパススルー、テストに `bound_fore_aft_placement_
gain_override` を追加。

**検証**(定数cmd_vx=2.20・ランプなし・15秒・ゲイン0/0.02/0.05/
0.10/0.15):

| gain | vx挙動(t=2〜15s) |
|---|---|
| **0.00** | **1.37〜1.88で全区間安定**(崩壊なし) |
| 0.02 | t=8sまで1.2〜1.4、以降単調に減衰しt=11sで負に転落 |
| 0.05/0.10/0.15 | **3つとも完全に同一の数値**、早期に0.1〜0.7へ低迷 |

**発見1(足配置は悪化させた)**: ゲインを上げるほど追従が悪化。
原因は明確——cmd_vx=2.20は**この歩容が実現できる速度(~1.7 m/s)を
超えている**ため、`v_err.x = v_obs − v_cmd` が慢性的に負(-0.5程度)。
Raibert項 `k·v_err.x < 0` が足を慢性的に後方へ引き、歩容を乱す。
Raibert足配置は「動作点まわりの小さな摂動」を補正する設計であって、
0.5 m/sの定常オフセットには不適。

**発見2(クランプ飽和)**: gain=0.05/0.10/0.15が1バイト違わず同一
——`v_err`が大きいためどのゲインでも足配置項が `max_step_length/2=
0.09` クランプに張り付き、同じ飽和footholdを生む。ゲインを上げても
無意味。

**発見3(最重要・副次的)**: **gain=0のベースライン(定数cmd=2.20
+絞り込みクランプ0.16-0.20・ランプなし)が15秒間崩壊せず安定**
(vx=1.37〜1.88)。これは重要な再フレーム——§5c1〜5c4で追いかけて
きた「t=4.5〜6.5sの乱れ」は**巡航安定性の問題ではなく、起動ランプ
そのものが持ち込むアーティファクト**だった。定数cmdなら巡航は
最初から安定していて、乱れはランプが原因。つまり「起動の滑らかさ
(ランプ)」と「乱れのなさ(定数cmd)」がトレードオフになっている。

**結論**: (A)Raibert fore-aft足配置は**この状況では有効でない**
——(i)cmdが実現速度を超えると慢性v_errで逆効果、(ii)cmd実現域
(~1.5)では巡航が既に安定していて補正すべき不安定がそもそもない、
(iii)外乱注入のない定常シミュでは足配置の本来の価値(摂動除去)を
発揮する場面がない。文献での価値は「実現可能な平均速度まわりの
摂動除去」であり、そのためには neutral点を cmd でなく実現速度で
サイズする必要がある(今の `half` は cmd ベース)。機能自体は
既定0.0で無害なオプトインとして残置。

**再フレームに基づく次の一手候補**: 乱れの正体が「ランプ vs 定数cmd
のトレードオフ」と判明した以上、(a)デモは定数cmd版(巡航安定・
起動だけ粗い)を採り、起動の粗さは許容する、(b)ランプ版を採り
残る小さなdipを許容する(§5c3、既に撮影済み)、(c)足配置の
neutral点を実現速度ベースにする改修で(A)を本来の摂動除去器として
活かす道を探る、のいずれか。ユーザーへ確認。

### 5c7. (c)足配置のneutral点を実現速度ベースに改修 — さらに悪化
(後退・転倒)、だが本質的なアーキテクチャ上の知見が得られた
(2026-07-25)

**実装**: 古典Raibert走行則 `x_foot = ẋ·T_st/2 + k·(ẋ−ẋ_des)` の
**neutral項 `ẋ·T_st/2` を測定速度(EMAフィルタ、tau=0.15s)ベース**に
修正——§5c6ではneutralをcmdベースのまま`k·v_err`だけ足していたため
cmd超過時に慢性バイアスが出た。今回はneutralを実現速度でサイズし直し、
fore-aft(x)のみ上書き。フィルタ状態`v_fore_aft_filtered`をtickで更新。

**検証**(定数cmd=2.20・15秒・ゲイン掃引):

| gain | 挙動 |
|---|---|
| 0.00 | 1.4〜1.9で安定(足配置OFF、§5c6と同一) |
| 0.02 | **全区間vxが負(-0.2〜-0.96)**——後退 |
| 0.05 | 後退、t=14sで**転倒**(roll=π, min_z負) |
| 0.10 | 全区間後退(-0.2〜-0.83) |
| 0.15 | 後退、t=12sで**転倒** |

**§5c6より悪化した**——ゲインを入れた途端ロボットが後退する。

**本質的な知見(なぜ足配置が全く効かないのか)**: このアーキテクチャ
では**前後推進はトリムの`F_x`(thrust_scale駆動)がWBCに直接GRFとして
指令**しており、足配置(足のXY着地点)とは別入力になっている。一方
Raibert/MIT Cheetahの足配置則が成立するのは、**脚力が脚に沿った
radial力で、水平力=足配置(脚角度)そのものが唯一の水平力源**である
場合。つまり「足をどこに置くか」=「水平力をどう出すか」が同一。

ところが本系では`F_x`が独立に指令済みなので、そこへ運動学的な
Raibert足配置則を後付けすると、**速度制御を二重に持つことになり
競合する**——`F_x`トリムが「前へこれだけ押せ」と言う一方、足配置則が
「足を後方に置け」と言い、幾何学的に矛盾。WBCがこれを調停しきれず、
後退・転倒に至る。§5c6/5c7の負の結果は、パラメータの問題ではなく
**「直接GRF制御 + 反応的足配置ヒューリスティックの二重計上・競合」**
というアーキテクチャ上の非互換が根本原因だった。

**結論**: 反応的な足配置ヒューリスティック(A)(c)は、GRFを直接指令
する本WBCアーキテクチャには**原理的に適合しない**。文献の足配置則を
活かすには、足配置を独立入力にするのではなく、**MPCがfoothold XYと
GRFを同時に(整合を取りながら)計画する経路**——既存の
`use_mpc_predicted_footstep`(§A1、footstepをMPC予測ベースにする、
現在Bound最良パターンではOFF)を滞空期対応で成立させる=§5c5の(B)
——が筋。あるいは(E)学習方策。反応的ボルトオンではない。実装した
`bound_fore_aft_placement_gain`は既定0.0の無害なオプトインとして残置
(この否定的知見を記録として保持)。

**総括(足配置の探索を終えて)**: (A)(c)の2回の否定的結果により、
「今のアーキテクチャに最小の追加で足配置を足す」道は行き止まりと
判明。安定化の次の一歩は本質的に(B)MPCによるfoothold+GRF同時計画か
(E)学習方策のいずれかで、どちらも小さなパッチではなく設計変更を
要する。手調整による安定化はここが実質的な到達点——§5c3のランプ版
(起動滑らか+巡航回復)または定数cmd版(§5c6で判明した巡航安定・
起動粗)を「現アーキテクチャでの最良デモ」として確定する。

### 5c8. (B)MPCによるfoothold+GRF同時計画 — 既存機構の有効化だけ
では即転倒、真の(B)はMPCの滞空期スイング計画の作り込みが必要
(2026-07-25)

§5c5/5c7で「反応的足配置は直接GRF制御と競合するので、MPCが
footholdとGRFを同時計画する経路(=既存の`mpc_optimized_footstep`
+`use_mpc_predicted_footstep`)が筋」と結論づけた。この2フラグを
Bound最良パターン(定数cmd=2.20)で有効化して検証:

- `mpc_optimized_footstep=true`: MPCがfoot-XY着地コスト
  (`q_foot_xy_world=500`)を追加し、スイング脚計画を能動的に変えて
  footholdを選ぶ。
- `use_mpc_predicted_footstep=true`: コントローラがMPCの選んだ
  footholdを(開ループRaibertの代わりに)使う。
- テストに`mpc_optimized_footstep_override`(WbcParams、build前に
  cfgへ適用)を追加、`use_mpc_predicted_footstep`は既存フラグ。

**結果: 即座に転倒**。baseline(両フラグOFF)が15秒間vx=1.4〜1.9で
安定なのに対し、両フラグONは**t=1〜2sで既にpeak_roll=3.141
(≈π、完全に横転)、min_z=-0.24m(胴体が地面を突き抜け)**、以降
15秒間ずっと転倒したまま(mismatch 50〜64%)。

**原因**: `mpc_predicted_swing_target_body`はMPCの予測スイング脚
関節角`predicted_states[k_td].leg_joint_q`をFKして footholdを得る。
しかしこの機構は**Trot用に検証されたもの**(config docの「P2
negative result識別」の文脈)で、**滞空期を持つBound**では、MPCの
予測スイング関節軌道がまともなfootholdを生まない——`q_foot_xy_world`
コストや`n_stance=0`区間のスイング計画がBoundに転移せず、足がでたらめ
な場所を狙って即転倒する。§5bq以来警告してきた「`n_stance=0`経路は
紙の上では対応だが実地未検証」のリスクが、foothold計画の側面で
顕在化した形。

**結論**: (B)は**既存機構のフラグ有効化だけでは実現しない**——
本物の(B)は、FullCentroidal MPCの**滞空期を含むスイング脚/foothold
計画そのものを作り込む**大掛かりな変更(MPCがアーリアル軌道と着地点
を整合的に最適化するようスイングコスト/制約を設計し直す)を要する。
これはパラメータ調整でも小パッチでもなく、MPC定式化レベルの開発
案件。追加した`mpc_optimized_footstep_override`は既定Noneで無害な
オプトインとして残置(この否定的知見の記録)。

**現時点の到達点と方針**: このセッションの手調整+既存機構の範囲では、
安定したBound滞空歩容のデモは§5c3(ランプ版)/§5c6(定数cmd版)が
最良で確定。ここから先の本質的安定化は (B)MPCスイング計画の作り込み
(MPC定式化の大改修)か (E)学習方策(別PC学習→policy-runtimeで
sim/実機同一デプロイ、memory参照)——いずれも新規の大きな開発
フェーズになる。次にどちらへ投資するかはユーザーの判断を仰ぐ。

### 5c9. (B)の2フラグを分離検証 — 転倒の原因はfoot-XY「コスト」側、
「予測foothold→スイング目標」側は安定(2026-07-25)

ユーザー指示で(B)を本PCで継続。§5c8の全転倒がどちらのフラグ由来か
切り分けた(定数cmd=2.20、3秒、0.5秒窓):

- **cost-only**(`mpc_optimized_footstep=true`、スイング目標は開ループ
  Raibertのまま): **転倒**。t=1.5〜2sでpitch=1.41rad、t=2sで
  roll=π・min_z負。foot-XYコストをMPCに足すこと自体が不安定化。
- **predict-only**(`use_mpc_predicted_footstep=true`、foot-XYコスト
  なし=MPCはfootstep最適化していない): **3秒安定**。vx=0.3〜0.83で
  ばらつくが、peak_pitchは0.09〜0.23radに収まり転倒なし(min_z
  0.215〜0.232)。反応的足配置(§5c6/5c7、即転倒)より明確に良い。

**判明**: 不安定化の主犯は**foot-XYコスト**(`q_foot_xy_world=500`)で
あって、予測footholdをスイング目標に使う側ではない(予想と逆)。
最初に暴走するのが**pitch**(roll でなく)である点が核心——pitchは
トリム/F_xが握る最も繊細な自由度で、foot-XYコストが同じQP内でGRF/
pitch追従コストと競合し、Boundの微妙なpitchバランスを崩している。
セッション全体の主題(F_x/pitchバランスは脆い)と一致。

**(B)への具体的な道**: predict-only経路は安定なので「MPC予測foothold
をスイング目標に使う」機構自体は成立している。問題はコストの強さ
(500)がGRF/pitch解と張り合っている点。→ `q_foot_xy_world`を大幅に
下げ、MPCがpitch解を壊さずに緩くfootstepを整形できる領域を探す
(§5d0で掃引予定)。これが本PCでの(B)実装の当面の主戦略。

### 5d0. foot-XYコスト重みの掃引 — どの重みでも破綻、これは
チューニングでなく構造的問題(2026-07-25)

`q_foot_xy_world`を500/100/20/5/1と2.5桁掃引(両フラグON、定数
cmd=2.20、8秒)。**すべて破綻**——500/20/5は転倒(roll=π・min_z負)、
100/1は転倒こそ免れるがvxがほぼ0〜負でpeak_pitch 0.3〜0.5rad、
mismatch 25〜44%と、まともに歩けない。**重みを下げても救えない**。

**判明(§5c9の主戦略は失敗)**: foot-XYコストは**どの重みでも**
flight-phase Boundを壊す——チューニングの問題ではなく構造的。ごく
小さいコスト(q=1)でも、foot-XY配置をpitch臨界のGRF QPに結合させる
こと自体が、Boundの繊細なpitchバランスを崩すのに十分。

**(B)全体の総括(既存インフラでの探索を尽くして)**:
- 両フラグON(§5c8): 即転倒
- 分離(§5c9): コスト側が主犯、predict-only側は3秒安定だが遅い
  (0.5〜0.8 m/s、footholdが保守的すぎ)
- コスト重み掃引(§5d0): 全域破綻、構造的

つまり**既存のMPC-footstep機構(Trot用に設計・検証)では、
flight-phase Boundの(B)は実現できない**。安定して速い構成は依然
§5c6ベースライン(開ループRaibertスイング目標 + トリムF_x GRF、
1.7 m/s・15秒安定)であり、MPC-foothold最適化はどの変種もこれを
下回るか破綻する。本物の(B)——foot-XY最適化をpitch臨界のGRF QPに
破壊的に結合させない新規のflight-phase対応MPC定式化(コスト構造の
再設計、footholdとforceのQP分離、あるいはpitch保存制約の追加)——は、
既存フラグの有効化やチューニングではなく、**MPC定式化レベルの
大規模な研究開発案件**であることが確定した。

**方針判断**: 本PCでの(B)は「既存インフラの活用」の範囲を尽くし、
これ以上はMPCソルバ/定式化の本格改修(数日〜規模の研究開発、
成果は不確実)に入る。ユーザーが承認済みのもう一方の(E)学習方策
(別PC)の方が、bounding特有のハイブリッド力学安定化には費用対効果
が高い可能性が大きい——(E)を主軸に進め、(B)は定式化改修の独立
プロジェクトとして別途スコープするのが妥当。追加した
`mpc_optimized_footstep_override`/`q_foot_xy_world_override`は
既定Noneで無害なオプトインとして残置(この探索の記録)。

### 5d1. (B)定式化の本格改修 — 破綻機構を特定し、body-frame
foot-XYコストという限定的な修正を設計(2026-07-25)

ユーザー指示で(B)定式化の本格改修を検討。まず**なぜfoot-XYコストが
pitchを壊すのか**をMPCの式レベルで特定した。

**破綻機構(誤仮説を1つ棄却して確定)**:
- 当初「滞空期に遊脚を振るとその反動角運動量で機体が回る」と仮説。
  → `full_centroidal_dynamics`を精読して**棄却**: 角加速度は
  `α = I⁻¹·(Σrᵢ×Fᵢ − ω×Iω)`で**GRFモーメントのみ**が駆動、関節速度
  `joint_v`は角運動に一切入らない(`q̇=v`は運動学的簿記のみ、遠心力
  モデルだが遊脚反動は含まれない)。
- **真の機構**: `add_foot_xy_soft_cost`のセレクタ`e_xy[6+ax]=1`が
  **base_pos**をfoot-XY式に含めている。foot_world = base_pos +
  R·FK(q)。base_posはダイナミクス経由でGRF駆動——よって
  foot-XY誤差を減らす際、MPCは**水平GRFを調整してbaseを動かす**
  経路も使い、水平GRFは`r×F`でpitchモーメントを生む。F_xが既に
  pitchキャンセルのトリム値に張り付いているBoundでは、この余分な
  水平GRFがpitchバランスを即座に崩す(§5c9でpitchが最初に暴走した
  のはこれ)。§5d0で重みを下げても救えなかったのも、結合そのものが
  問題(重み非依存)だから。

**設計(限定的・機構由来)**: foot-XYコストを**body-frame(base相対)**に
する。`e_xy[6+ax]=1`(base_pos項)を落とし、ターゲットからも
base_pos_refを落とすと、誤差 = `R·FK(q) − R·offset`(base相対の
配置誤差)となり、**joint_v のみに依存しGRFと完全非結合**。MPCは
遊脚を動かすだけでfootを配置し、pitch臨界のGRF解を一切乱さない。
world-frame版(外乱時にfootの地面上絶対位置を保つTrot用の機能)が
まさにpitchトリムと競合していたので、Boundにはbody-frame(Raibert
そのもの)が正しい。`FullCentroidalMpcConfig`に`foot_xy_cost_body_
frame: bool`(既定false=現行world-frame)を追加し、trueでbase_pos項を
ゼロにする限定的変更。これは大規模定式化改修ではなく、破綻機構に
ピンポイントで効く小さな修正——実装して検証する(§5d2)。

### 5d2. body-frame foot-XYコストを実装・検証 — 仮説は外れ、
むしろ悪化。真因は「予測joint_qを遊脚目標に使う」経路側と判明
(2026-07-25)

`FullCentroidalMpcConfig::foot_xy_cost_body_frame`(+GaitConfig
経由、既定false)を実装:trueで`add_foot_xy_soft_cost`のセレクタ
`e_xy[6+ax]`とターゲットからbase_pos項を落とし、誤差を
`R·(FK(q)−offset)`(base相対)にする。全205 lib testパス、既定
world-frameは恒等。

**3構成比較(定数cmd=2.20、15秒)**:
| 構成 | 結果 |
|---|---|
| 1. baseline(MPC footstepなし) | 1.4〜1.9で安定(§5c6) |
| 2. world-frame cost q=100 | 転倒せず横ばい(vx≈0、roll 0.1〜0.6) |
| 3. **body-frame cost q=100(今回の修正)** | **t=2-3sでroll=π、以降ずっと転倒**(min_z負) |

**仮説は外れた**——body-frameは改善どころか**case 2より悪化**して
即転倒。§5d1の機構分析(base_pos項がGRF/pitchを結合)は、少なくとも
主因ではなかった。

**判明した真因**: `full_centroidal_dynamics`で確認した通りjoint_vは
MPCモデルの機体回転に入らない。にもかかわらずroll=πで転倒するのは、
**`use_mpc_predicted_footstep`がMPCの予測joint_qをFKして遊脚IK目標に
使う**経路のため。foot-XYコスト(world/body問わず)がMPCの予測
swing joint_qを極端・非対称な値へ駆動し、それを遊脚目標にすると
実機(sim)がL/R非対称に足を接地→実際のロールモーメント→横転。
body-frameはbase_posという「逃げ道」を塞いだ分、補正が全て関節運動に
集中し、joint_q目標がさらに極端化してむしろ悪化した。MPCモデル上は
joint_v自由なので「問題ない」解に見えるが、その予測joint_qを実機の
遊脚目標に流用する時点で破綻する。

**結論**: (B)の定式化修正は「コスト系の座標フレーム変更」より深い。
foot-XYコストが生む予測swing joint_qは、そのままでは実機の遊脚目標
として使えない(非対称・非現実的)。真の(B)には、遊脚軌道/footholdを
物理的に妥当・左右対称な範囲へ制約する仕組み(スイング軌道の陽な
生成、foothold対称性制約、あるいは予測joint_qでなくfoot-XY点だけを
取り出して左右対称化して使う等)が要る——これは§5c5で見積もった
通りの本格的な研究開発。今回のピンポイント修正(body-frame)では
届かないことが実証された。追加した`foot_xy_cost_body_frame`は
既定false(world-frame)の無害なオプトインとして残置。

**(B)全体の到達点**: 既存インフラ+機構由来のピンポイント修正まで
試して、flight-phase Boundの安定なMPC-foothold計画には至らなかった。
安定・高速な実用構成は依然§5c6(開ループRaibert+トリムF_x、1.7 m/s
15秒安定)/§5c3(起動ランプ版)。(B)properは「MPCの滞空期スイング
計画を左右対称・実機整合に作り込む」研究開発フェーズとして、(E)
学習方策と並行して別途スコープするのが妥当。本PCでの(B)探索は
ここで一区切り。

### 5d3. (B)本格研究 — 左右対称foothold + 予測foothold の直接計測。
真因を ground truth で特定(2026-07-25)

ユーザー指示で(B)本格研究を継続。§5d2で「MPCの左右非対称な予測
footholdが滞空期にロールさせる」と診断し、対策として左右ペア
(front FL/FR・rear RL/RR)のfootholdを対称化する
`bound_symmetric_foothold`(GaitConfig、既定false)を実装
(latchを事前パスに移し、ペアごとにx/z平均・yミラー化してから
本ループで消費)。全205 lib testパス、既定offで恒等。

**検証(定数cmd=2.20、15秒、対称化ON)**: body-frame/world-frame
どちらも**依然roll=πで転倒**(case2はt=2-3s、case3はt=3-4s)。
**§5d2の仮説も外れ**——左右対称化しても転倒は防げなかった。
これで3仮説(base_pos結合§5d1、コスト重み§5d0、左右非対称§5d2)が
すべて棄却された。

**そこで予測footholdを直接計測**(`FOOTHOLD_DIAG`環境変数で
latch値をログ、計測後revert):対称化は効いている(L/R一致)が、
**予測footholdそのものが2点で不良**と判明:
1. **前後方向が後ろすぎ**: 前脚のx≈0.16(nominal 0.192より3cm後方)。
   cmd=2.2 m/sなら前方~0.26が必要なのに、逆に後方。foot-XYコストが
   関節を nominal姿勢へ引くコストに負けている。
2. **Zが乱高下**: 足の深さが-0.214〜**-0.346**mまで振れる。スイング
   脚サブ問題にfoot-Z目標がないため高さが脚伸びきり近くまで漂う。

**真因(ground truth)**: MPC予測swing joint_qをFKしたfootholdは、
(1)前後が後方すぎ→前脚が後ろに着地→機体が前へpitch暴走(観測した
peak_pitch 1.5rad)、(2)Zが乱高下→脚過伸展/硬着地。これを遊脚IK目標に
使えば当然転倒する。**開ループRaibert目標(§5c6ベースライン)の方が
正しいfoothold**——前方適切・Z一定。MPCのswing脚サブ問題は
under-constrained(foot-Z目標なし、foot-XYは関節追従コストに負ける)で、
foothold源として信頼できない。

**(B)本格研究の結論(本PC)**: 「MPCにfootholdを選ばせる」(B)は、
flight-phase Boundでは開ループRaibertを上回らない——MPCのswing脚
サブ問題が構造的にunder-constrained(3D足目標のうちZ無拘束、XY弱い)
だから。真に(B)を機能させるには、MPC定式化にfoot-Z目標/拘束を追加し、
foot-XYの権限を関節追従より強くし、かつ予測joint_qでなく整形した
3D足目標を使う——という**複数の定式化変更を積む本格的な再設計**が要る。
そこまでして開ループRaibert(既に1.7 m/s安定)を上回る保証はなく、
費用対効果では(E)学習方策が優る、という当初の見立てが本格研究でも
裏付けられた。実装した機構(body-frame cost §5d1、symmetric foothold
§5d3、pre-pass latch)はすべて既定offの無害なオプトインとして残置し、
将来の(B)再設計の土台とする。

**確定した最良デモ構成**: §5c3(起動ランプ+PLL warm-start)/§5c6
(定数cmd、巡航1.7 m/s・15秒安定)。(B)は本PCでは既存インフラ+
機構由来の修正+本格研究の入口まで尽くし、開ループRaibert超えは
定式化の大規模再設計待ち。(E)を主軸に進めるのが妥当。

### 5d4. MITを参考にする——文献の外部確認 + 上下バウンス参照の
検証。**私の提案方向は誤り**、MIT整合はフラット参照だった
(2026-07-25)

ユーザーから「MIT biomimetic labのCheetah 2/3/Miniは過渡状態も
安定にモデルベースBoundを実現している、参考にできないか」との指摘。
私は「我々のMPC参照が滞空期に高さ一定・上下速度ゼロを指令していて
実現不可能。§5bqで導出済みの弾道バウンスを参照に入れるべき」と提案し、
実装(`bound_trim_vertical_reference`フラグ、既定off):stance
vertical-GRF参照を`m·g/(2duty)`に、CoM上下速度参照を弾道バウンスに。

**ユーザー指示で並行してMIT文献を外部リソースで確認**(サブエージェント
がWeb検索、一次論文PDFから逐語引用で検証、途中1件のhallucination引用も
自己検出・訂正):

- **foothold**: Cheetah 3(Di Carlo 2018)/Mini Cheetah(Kim 2019)は
  **footholdをMPC内で最適化しない**。Raibert式ヒューリスティック
  (`p_des = p_ref + v·Δt/2`)で外部計算し、MPCは固定接触スケジュールに
  対し**GRFのみ**最適化。→ 我々の§5c6ベースライン(開ループRaibert+
  GRF)がMITそのもの、失敗した(B)はMITに逆行、を裏付け。Cheetah 2は
  MPCでなくインパルス計画の簡易モデル力制御。Bledt&KimのRPC後継のみ
  footholdを最適化に含めるが、Raibert nominalへ正則化。
- **上下参照(核心)**: Cheetah 3は逐語で**「roll, pitch, roll rate,
  pitch rate, z-velocity は常に0に設定」**。つまり**高さ一定・上下速度
  ゼロ・pitchゼロのフラット参照**。**弾道バウンスとpitch振動は、剛体
  ダイナミクス+滞空中force=0から創発する挙動であって、追従参照では
  ない**。Mini Cheetah逐語「body postureへの指令はconstant…measured
  heightとpitchはconstant指令のまわりを上下する(ロボットがMPC計画通り
  跳んで着地するから)」。z追従weightはむしろ高い(50)、orientation
  weightは低い(1)。
- 滞空期はダイナミクス側で処理(離地脚のforce=0拘束+重力で自由落下)、
  参照はフラットのまま「制御不能な区間は最小二乗で追従誤差を許容」。

**結論(私の提案は誤りと判明)**: **弾道バウンス参照を足すのはMIT設計
から遠ざかる方向**だった。MIT整合は逆——フラット参照を保ち、滞空の
劣駆動区間で参照から逸脱するのを許す(z/上下速度/pitchを過度に
ペナルティしない)こと。

**実測A/Bで裏付け**(`go2_wbc_bound_flight_phase_duty035_vertical_
reference_ab`、定数cmd=2.20、15秒):
| 構成 | 結果 |
|---|---|
| **OFF(フラット参照=MIT整合)** | **1.7 m/s・15秒安定**(§5c6) |
| ON(弾道バウンス参照) | **t=2-3sで転倒**(roll=π)、回復せず |

文献・実機ともフラット参照が正しいと一致。`bound_trim_vertical_
reference`は既定off(=フラット=安定)の無害なオプトインとして残置。

**本当のMIT整合な次の一手(未検証・有望)**: MITは(a)フラット参照、
(b)**pitch/orientation weightを低く**(pitch参照0のまま振動を創発
させる)、(c)Raibert foothold。我々の現最良構成は**独自トリムが
pitch参照を注入**(非フラット)しており、ここがMITと最も違う。真の
MIT実験は「独自トリムを外し、フラットpitch参照+低pitch weightで
Boundのpitch振動を創発させ、F_xもMPCのxy速度追従に任せる」——
既存の`roll_pitch_weight_override`で weight を下げられる。歴史的に
素のMPC-Boundは失敗したが、それは低pitch weight+現行の各修正
(yaw PD/PLL/接触スケジュール)以前。次はこれを試す価値がある。

### 5d5. MIT忠実実験 — 成功。トリムなし・低pitch weightで安定な
Bound(ユーザーのMIT指摘が的中)(2026-07-25)

§5d4の結論を受け、MIT忠実な構成をテスト(`go2_wbc_bound_flight_
phase_duty035_mit_emergent_pitch`): **独自トリムを完全に外し**
(`bound_trim_reference: None` → MPCへのpitch/F_x/F_z注入なし、WBC
pitch PDも既定offなので無効)、**フラットpitch参照のまま MPCの
roll/pitch attitude weight を25→0.5まで掃引**して、Boundの
pitch振動を創発させる。固定周期(PLLなし)、開ループRaibert footstep、
実mass/inertia同期あり、yaw保持PDのみ。duty=0.35、cmd_vx=1.5、10秒。

**結果——明確なスイートスポットで安定**:

| roll/pitch weight | 挙動 |
|---|---|
| 25(既定) | t=4s以降劣化(vx 0.2〜0.6、pitchを抑えつけて振動と競合) |
| 10 | さらに悪化 |
| **5** | **全10秒 vx=1.0〜1.28で安定、peak_pitch 0.08〜0.12・転倒なし** ✓ |
| 1 | 概ね良好(vx 0.9〜1.3)、t=9sで小さな乱れ |
| 0.5 | t=6s以降ヨードリフト(方向制御を失う、低すぎ) |

**weight=5で、素のMPCがトリムもPLLもthrust_scaleも無しに安定Bound
を実現**。低pitch weightがpitch振動の創発を許し、MITの論文記述
そのままの挙動になった。**ユーザーの「MITを参考にすれば改善できる
のでは」という指摘が的中**——我々が積み上げた独自スタック(閉形式
トリム+PLL+thrust_scale+各種ランプ)は、MIT流の「フラット参照+
低pitch weight+Raibert footstep」というシンプルな構成で置き換え
可能で、しかも同等以上に素直に安定する。

**位置づけ**: 現最良の§5c6トリム版は~1.7 m/s(cmd=2.2)とより速いが、
このMIT版は~1.2 m/s(cmd=1.5、80%追従)で**遥かにシンプル・トリム
チューニング不要**。滞空期(duty=0.35)を持ちながら10秒安定。歴史的な
「素MPC-Bound失敗」は低pitch weightを試していなかったことが原因だった
と判明。

**次の一手候補**: (a) weight=5固定でcmd_vxを上げ(1.5→2.0→2.5)MIT版の
速度上限を確認、(b) weightを3〜7で微調整し最適点を絞る、(c) MIT版に
起動ランプ/PLLを足して過渡・持続をさらに改善、(d) 動画撮影。この
MIT忠実ラインは、これまでのトリムベース最良構成と並ぶ(あるいは
上回りうる)第2の実用ラインとして本格的に育てる価値がある。

### 5d6. MIT忠実ラインの育成 (b)(c)(a) — 最適weight・ランプ/PLL不要・
速度上限を確定(2026-07-25)

§5d5の次の一手 (b)(c)(a) を順に実施(cmd_vx=1.5基準、duty=0.35、
トリムなし)。

**(b) pitch weight微調整(3〜7、12秒)**: weight=**3〜4がスイート
スポット**。3/4は全12秒 vx=1.0〜1.32で安定・pitch低(0.08〜0.12);
5は起動に小さな乱れ;6/7は後半劣化(pitch 0.28〜0.34、min_z 0.21)。
**weight=4を採用**(安定かつ最速1.32到達)。

**(c) 起動ランプ/PLL追加(weight=4)**: plain vs (ramp 2s + PLL) を
比較。**plainが最良**——MIT版の起動はもともと滑らか(t=2sで1.34到達、
pitch<0.13)でランプ不要。**PLLはむしろ有害**(t=6-10sでvx 0.1〜0.8・
pitch 0.30の乱れを誘発、トリム版と同じ症状)。**MITの固定周期設計が
正しく、我々が苦労して調整したPLLはMIT版には不要どころか逆効果**。
これはMITアプローチのシンプルさをさらに裏付ける。

**(a) cmd_vx速度上限(plain weight=4、12秒)**:

| cmd_vx | 実測vx | 挙動 |
|---|---|---|
| **1.5** | ~1.1〜1.32 | 安定(追従75〜88%) |
| 2.0 | 0.02〜0.65 | 破綻(pitch 0.49) |
| 2.5 | 0.2〜0.5 | 破綻(pitch 0.4) |
| 3.0 | 0.3〜0.5 | 破綻 |

**MIT版の実効上限は ~1.3 m/s**(cmd=1.5)。cmd≥2.0は劣化する
——トリムがF_x前進推力を明示注入しないため、MPCの速度追従だけでは
高cmdの前進力が摩擦限界で足りず、pitchが暴れて崩れる。

**まとめ(2ライン比較)**:
| | トリム版(§5c6) | MIT版(§5d5/5d6) |
|---|---|---|
| 巡航速度 | **~1.7 m/s**(cmd=2.2) | ~1.3 m/s(cmd=1.5) |
| 構成 | トリム+PLL+thrust_scale+clamp+ランプ | **低pitch weightのみ** |
| チューニング | 多数のノブを長時間調整 | weight 1点 |
| 起動 | ランプ+PLL warm-startが必要 | もともと滑らか |

トリム版が速く(1.7 vs 1.3)、MIT版が圧倒的にシンプルで安定。両者は
速度 vs 単純さのトレードオフ。MIT版の1.3 m/s頭打ちは「F_x推力を
明示しない」ことに由来するので、速度を上げたければMIT構造に
最小限のF_x前進バイアス(トリムの一部だけ)を足す折衷が次の探索軸。
`bound_trim_vertical_reference`他のフラグは既定off維持。

**確定した2つの実用デモ**: (1)トリム版 §5c3(起動滑らか+~1.7 m/s、
`go2_bound_flight_phase_duty035_best_pattern.mp4`)、(2)MIT版 §5d5
(超シンプル+~1.2-1.3 m/s、`go2_bound_mit_faithful_duty035.mp4`)。

### 5d7. MIT版の速度上限を破る折衷案 — 高速weight・F_xバイアス
どちらも不発。上限は「構造化された推力」の欠如が本質(2026-07-25)

§5d6(a)でMIT版がcmd≥2.0で崩れたので、上限を上げる2案を検証。

**案1: 高速時にpitch weightを上げる**(cmd=2.0でweight 4/8/12/16/25):
**どのweightも安定させられなかった**。weight=25はt=7-9sで一時的に
vx=1.4-1.5に達したが持続せず崩壊。上限は単純なweight不足ではない。

**案2: 前進F_x推力バイアス**(新規`bound_fx_thrust_bias`、stance脚に
一定の前進GRFを加算。cmd=2.0・weight=4でbias 0/20/40/60N):
**これも安定させられなかった**。bias=40/60は瞬間的にvx=1.0-1.09に
達するが、pitchが0.3-0.49まで暴れ、**bias=60はt=11-12sで転倒**
(pitch 1.56、roll=π)。原因は明確——**一定の前進F_xはpitchモーメントを
生む**が、トリムの交番(front/rear符号反転)F_xと違い**pitch非
キャンセル**なので、低weightの姿勢制御では抑えきれない。

**本質的な結論**: MIT版の~1.3 m/s上限を破るには「推力の大きさ」では
なく「**構造化された(pitchキャンセル・位相整合の)推力**」が要る
——それこそがトリム版のF_xプロファイル(閉形式・front/rear交番)が
やっていることで、素朴なバイアスでは代替できない。つまり折衷案で
1.3 m/sを超えようとすると、結局トリムのF_x構造を再発明することになる。
**2ラインは速度 vs 単純さの真に異なる動作点**であり、単純なパッチでは
橋渡しできないことが確定した。

**最終確定(Bound滞空歩容、本PC)**:
- **速度重視 → トリム版**(§5c6/5c3、~1.7 m/s、構造化F_x+prescribed
  pitch、複雑だが速い)。
- **単純さ重視 → MIT版**(§5d5/5d6、~1.3 m/s、低pitch weightのみ、
  トリム/PLL/thrust_scale不要、MIT Cheetah 3/Mini Cheetahの設計そのもの)。
- 折衷(高速weight・F_xバイアス)は不発——両者は別動作点。

追加した`bound_fx_thrust_bias`・`bound_trim_vertical_reference`他は
既定off/0の無害なオプトインとして残置。これで「MITを参考に改善」
という軸は、(a)既存§5c6がMIT構造そのものと判明、(b)よりMIT忠実な
トリムレス版を新規確立(§5d5)、(c)両者の速度差は構造化推力の有無と
特定、まで到達し、一区切り。

### 5d8. Bound立ち上がりの先行研究調査 + MIT Cheetah 2 レシピの
実装検証(推奨1)。MIT版はabruptで既に良好、ランプは有害(2026-07-26)

**先行研究の外部調査(ユーザー承認、一次資料で確認)**で判明した
立ち上がり手法の要点:
- **最も確立=エネルギーポンプ/SLIP起動**(Raibert 1986、Ahmadi &
  Buehler CPDR 2006、Poulakakis & Grizzle 2009):各stanceでthrust注入
  し静止から目標軌道へ登る。
- **最も原理的=遷移軌道の一括最適化**(TOWR Winkler 2018、Posa
  contact-implicit 2014、standing-jump TO):静止→動的歩容を境界値
  問題として一括計画。ただし「周期Boundの立ち上げTO」は文献の空白。
- **歩容連続変形(前回の私の推奨)は名前のある確立手法ではなかった**
  ——連続変形は解析ツールとして実在(Remy "Breaking Symmetries")だが
  「duty連続変形の起動則」は私の造語、と正直に判明。
- **唯一手順を明記=MIT Cheetah 2**(Park/Wensing/Kim IJRR 2017、逐語):
  起立→**スケジュール開ループ初期ステップ**→v_d=0から**「ステップ
  ごと」に速度increment**(impulse/stride スケーリング γ)→MPCへ。

**推奨1(MIT Cheetah 2 のステップ同期ランプ)を実装**
(`cmd_vx_step_increment`、cmd_vxを半周期=ペア交番周期ごとに階段状に
increment)。MIT忠実版(トリムなし、weight=4)でabrupt/時間ランプ/
ステップランプを比較(cmd=1.5、10秒、0.5秒窓):

| 立ち上がり | 結果 |
|---|---|
| **abrupt(即指令)** | **最良**: 0→1.34を2秒で単調上昇、pitch 0.01-0.12、全安定 |
| 時間ランプ2.0s | 悪化(t=3-4.5sで乱れ、pitch 0.32) |
| ステップランプ+0.15/step | **最悪**(vx 0.1-0.5、pitch 0.25-0.37、不安定) |

**abruptが既に最良で、両ランプは有害**という予想外の結果。しかし
先行研究で説明がつく:**MIT Cheetah 2 のステップランプは、その
開ループimpulse-scalingフィードフォワード力への回避策**(全速で
即適用すると過大)。我々のMIT版は**Cheetah 3流のMPCベース**で、
MPCが閉ループで力を自然に漸増させるためランプ不要——調査で確認した
「Cheetah 3は起立→スケジュール切替でMPCに任せる」と一致。

**結論**: **MIT版の立ち上がりは既に解決済み**(MPCが自然に単調加速)。
当初の「粗い立ち上がり」問題は**トリム版**固有の話。MIT Cheetah 2の
ステップランプ・レシピは、フィードフォワード力を持つ**トリム版**に
こそ適合する(そこで我々の時間ランプが失敗した§5c0-5c2)。実装した
ステップランプは公称半周期での離散化で真の接地イベント同期ではない
点に注意——だが本命のトリム版で試す価値はある(次の一手)。
`cmd_vx_step_increment`は既定Noneの無害なオプトインとして残置。

### 5d9. ステップランプをトリム版で検証 — こちらでも不発。ヨー
ドリフトを誘発、abruptが最良。「推奨1」総括(2026-07-26)

MIT Cheetah 2 のステップランプが**フィードフォワード力を持つトリム版**
にこそ適合するはず、という§5d8の見立てを検証(トリム最良構成:
duty=0.35, thrust_scale=1.0, PLL, clamp 0.16-0.20、cmd=1.5、12秒)。

| 立ち上がり | 結果 |
|---|---|
| **abrupt** | 最良: 0→1.37を4秒で滑らか、~1.2-1.37安定(t=11s以降に軽い乱れ) |
| 時間ランプ2.0s | **ヨードリフト**: t=4.5s以降 world-x が0〜負、planar 1.2維持=旋回 |
| ステップランプ+0.2/step | 同様に**ヨードリフト**(t=6.5s以降) |

**トリム版でもステップランプは不発**——時間ランプ同様にヨードリフト
(旋回)を誘発し、abruptが最良。MIT Cheetah 2レシピはトリム版にも
転移しなかった。

**「推奨1」総括**: MIT Cheetah 2のステップランプは**開ループ
フィードフォワード力**のためのレシピで、我々の2ライン(MIT版/トリム版)
は**共にMPCベースで力を閉ループ生成する**ため、ランプ自体が不要で、
むしろ位相/ヨードリフト外乱を持ち込むだけだった。調査で確認した
「Cheetah 3(MPC)は起立→スケジュール切替でabruptに始める」と一致。
**両ラインとも abrupt が最良の立ち上がり**——立ち上がりはMPCラインの
根本問題ではなかった(トリム版の粗さは§5c0のcmd非依存F_xの症状で、
§5c3のパッケージ=thrust_scaleランプ+PLL warm-start で既に対処済み。
cmd単独ランプは時間/ステップ問わず§5c0の失敗を再現する)。

**先行研究を経た立ち上がり問題の最終結論**: モデルベースの
立ち上がり改善は、我々のMPCアーキテクチャでは「明示的な速度ランプ」
ではなく「MPCの閉ループにabruptに任せる」のが正解。さらに改善するなら、
文献で最も確立された**エネルギーポンプ起動(Raibert/CPDR)**か
**遷移軌道の一括最適化(TOWR/contact-implicit)**という、より大きな
定式化変更が必要(§5d8で調査済み)。`cmd_vx_step_increment`他の
起動ノブは既定offの無害なオプトインとして残置。

### 5f. (1b) 本物の対空を持つエネルギッシュBound — 死因はrollと判明(2026-07-26)

「低空Bound(duty=0.35、apex 数mm、実質平坦)」を脱し、**本物の滞空
(z_range 5〜25cm)を持つエネルギッシュBound**を目指した一連の実験。
duty=0.30(f_z_total=m·g/0.6=1.67 m·g で強い跳躍力)を軸に、跳躍は
出るが安定化できるかを検証した。

**§5f0 パラメータ探索**: T=0.30 duty=0.30 swing=0.10 が「最もマシ」
——weight=4 で **約8秒間 転倒せず z_range 最大0.23mの本物の跳躍**を
維持(min_z 正、roll<0.07)。ただし pitch 0.4〜0.7rad と過大で
前進しない「その場跳ね」。真の対空は出せることを確認。

**§5f1 pitch weight スイープ(MIT創発pitchライン)**:
roll_pitch_weight = 8/15/25/40 を T=0.30 duty=0.30 で試験。
**全weightが t=2-3s で roll=π に転倒**(z_rangeスパイク0.54-0.58m=
横転、以降 min_z 負でカオス跳ね、mismatch 40-55%)。weight=4 より
悪化——**高weightほど早く転倒**(§全体の非単調パターン再現)。
pitch weight を上げても跳躍は安定化しない。

**§5f2 トリム版(構造化pitch)の試み**:
T=0.30 duty=0.30、bound_trim=(100,10)、thrust_scale=1.0、
vertical_reference on/off、PLL clamp 0.28-0.32。
- vref=true: t=0〜3sは本物の跳躍(z_range 0.16→**0.246m**、roll≈0、
  pitchはトリムが 0.4〜0.7 に制御)、しかし **t=3-4sで roll=π 転倒**。
- vref=false: t=2-3sで転倒(1秒早い)。トリムのpitch構造化は転倒を
  ~1秒遅らせz_rangeを僅かに伸ばすが、rollは救えない。

**結論(1bの死因)**: MIT版・トリム版ともに **失敗モードは pitch でなく
roll**。真の対空Boundは長い滞空(全脚遊脚=**GRFゼロ=roll制御権限
ゼロ**)を必然的に持ち、その間に微小な roll rate が積分され、非対称
着地→横転(roll=π)に至る。トリムの構造化pitchは**pitchは抑える
(転倒までroll≈0)がrollには無力**。これは我々のWBC/MPCが**滞空中の
姿勢(特にroll)を能動制御する機構を持たない**というアーキテクチャ
上の限界そのもの。§5d8の先行研究サーベイが「エネルギッシュbounding
の安定化は滞空中の姿勢発散が難所で、Poincaré/HZDデッドビートか
エネルギーポンプ+姿勢制御が要る」と指摘した通りの壁に到達した。

**真の対空Boundを安定化するのに必要な、より大きな定式化変更**:
1. **足配置/着地制御**(Raibertの横方向足配置でrollを補正)——ただし
   MPC予測フットステップ経路は我々の構成に不適合と§5c6-5d3で確認済み。
2. **明示的な滞空中姿勢制御**(脚慣性/股関節反力によるreaction、
   flywheel的なroll補正)——新規定式化。
3. **リフレックス着地**(各接地でrollをリセットするdeadbeat/Poincaré)。

いずれもパラメータ調整でなく機構追加。**低空の安定Bound(§5d5-5d6の
MIT-faithful weight=4、~1.3 m/s、abrupt起動)が現アーキテクチャの
実用解**であり、本物の対空は上記1-3のいずれかを実装しない限り
安定化できない、というのが実証的な最終所見。

### 5f3-5f5. 突破口: roll/pitch-rate デッドビート + duty安定窓(2026-07-26)

§5fの死因分析を精緻化し、対策を実装。ユーザー指示「滞空中の姿勢
制御を実装」に沿い、**新機構でなく既存WBCの状態重みで滞空姿勢を
制御**するアプローチを取った。

**§5f3 死因の再同定 — 主犯はpitch(前方宙返り)**: 転倒時に
peak_pitch がちょうど **π/2(90°)** に達していることを発見。roll=π は
その帰結(tumble + ZYXジンバルロック特異点)。つまり真の死因は
「rollの横転」でなく **pitch角運動量が1周期ごとに増幅し前方宙返り
(somersault)する** こと。トリム版(§5f2、pitch impulseをミラー
対称で相殺)が1秒長持ちしたのもこれで整合。

**核心の観測**: MPCの状態重み `q_diag[3..6]`(角速度)は
**roll_rate=0.5 / pitch_rate=0.5** とごく小さい。stance中は
front/rear が Left-Right 足対なので差動GRFで roll/pitch モーメントを
作れる(制御権限は存在する)のに、MPCが角速度をほぼ罰しないため、
滞空で溜まった角速度を短いstanceで打ち消さない。

**対策 = レート・デッドビート**: `q_diag[3]`(roll rate)・
`q_diag[4]`(pitch rate)の重みを上げ、既存のL/R差動GRF権限を
「各stanceで蓄積角速度をゼロに戻すデッドビート・リフレックス」に
変える(テストハーネス `roll_rate_weight_override: Option<(f64,f64)>`、
既定0.5/0.5、完全オプトイン)。

**§5f3結果(duty=0.30)**: roll_rate 0.5→100 で転倒が t=2-3s→t=3-4s に
**1周期後退**(roll が t=3sまで≈0を維持)。pitch_rate も足すと
20→t=5-6s、100→t=4-5s と多少前後するが、**duty=0.30では単一レバー
は各≈1周期しか買えず完全安定化には至らない**(敏感な共振)。

**§5f4 duty=0.34 が最長寿命(10秒生存だが恒久安定ではない)**:
攻めすぎの duty=0.30 を 0.34/0.38/0.42 に戻し rate-deadbeat(100,100)
を効かせると、**duty=0.34 が10秒間直立を維持**(10sテスト範囲内):
- peak_roll 0.000-0.083、peak_pitch 0.27-0.61(90°まで行かず)、
  min_z 常に正(0.170-0.224m=地面貫通なし)、**z_range 0.09-0.20m の
  本物の対空を10秒間持続**。
- duty=0.38 は t=3-4s、0.42 は t=4-5s で転倒(**非単調**)。

**§5f5 15秒スイープ — 訂正: 恒久安定ではない**: §5f4の「10秒安定」を
15秒に延長して検証したところ、**どのdutyも15秒は持たない**:

| duty | 転倒時刻(pitch宙返り) |
|---|---|
| 0.32 | t=6-7s |
| 0.33 | t=4-5s |
| **0.34** | **t=10-11s(最長寿命)** |
| 0.35 | t=7-8s |
| 0.36 | t=4-5s |

**§5f4の「完全安定」は早計だった**——10秒テストがちょうど発散時刻の
直前だっただけ。duty=0.34 も t=10-11s で結局 pitch が π/2 に達し宙返り。
**rate-deadbeat は転倒を 2-3s→10s に約4倍遅らせる**(生存中 roll≈0/
pitch<0.6 を維持)が、**pitchの緩慢な発散を減速するだけで零化しない**
——エネルギッシュBoundは緩やかに発散する系で、状態重み(線形2次
コスト)では真の極限周期に収束させられない。

**§5f 総括(実証的結論)**: 本物の対空(z_range 0.1-0.20m)を持つ
Boundは、rate-deadbeat という原理的レバーで**転倒までの時間を約4倍に
延ばせる**が、**恒久安定化は状態重みチューニングの範囲外**。§5d8
サーベイ通り、真の極限周期安定化には Poincaré/HZD のデッドビート
足配置(踏み出し位置を1周期先の姿勢誤差から解く非線形写像)が必要で、
これは現MPCアーキテクチャへの**より大きな定式化変更**。実用解は依然
**低空の安定Bound(§5d5-5d6、~1.3 m/s、abrupt起動)**。rate-deadbeat
override は既定0.5のオプトインとして残置(転倒遅延の有効な部品)。

### 5f6. 突破口(本物): Poincaré/デッドビート pitch 足配置で恒久安定達成(2026-07-26)

§5f5で「rate-deadbeat状態重みは pitch宙返りを4倍遅らせるが零化しない
(15秒で全duty転倒)」と判明。ユーザー指示に従い、**真の極限周期
安定化 = Poincaré/デッドビート足配置**を実装した。

**原理**: エネルギッシュBoundの死因は pitch角運動量が1周期ごとに
蓄積し前方宙返りすること(§5f3)。接地時の足の前後位置が次stanceの
pitchモーメントを決める(上向きGRFが CoM前方 +x → nose-down モーメント
∝ −x)。よって**踏み出し位置を接地時の pitch レートでずらせば、次
stanceのGRFモーメントが蓄積運動量を「減衰」でなく「零化」する**
(Poincaré断面でのデッドビート):
```
half.x += k_angle·pitch + k_rate·pitch_rate
```
両ペアに同一適用(同じ body pitch 状態)、feedback_enabled非依存
(姿勢は零速度でも安定化要)。

**実装**(全て既定0のオプトイン、非破壊):
- `full_centroidal_controller.rs`: 観測pitch/roll受取
  (`set_body_attitude_observed`)、デッドビートゲイン
  (`set_bound_pitch_placement_gain(k_angle, k_rate)`)、
  `compute_mpc_footstep` に上式を追加。
- `generator.rs` / articara `gait.rs`: パススルー配線。
- テストハーネス: 毎tick `robot.base_transform.rotation.euler_angles()`
  の pitch を供給、`bound_pitch_placement_gain_override` 追加。

**結果(duty=0.34, rate-deadbeat(100,100)併用, 15秒)—符号が決定的**:

| k_rate | 結果 |
|---|---|
| −0.06 | t=2-3s転倒(逆符号=宙返り加速) |
| −0.03 | t=1-2s転倒(逆符号=最悪) |
| **+0.03** | **★15秒完全安定**: roll 0.00-0.11(無転倒)、pitch 0.45-0.60(90°未満)、min_z 常に正(0.176-0.232m)、z_range 0.09-0.14mの本物の対空 |
| **+0.06** | **★15秒完全安定 かつ 歩行**: vx −0.4〜−0.6を一貫維持(後退だがその場跳ねでなく移動)、roll<0.12、min_z正、z_range 0.11-0.17m |

**正の k_rate が pitch運動量を零化 → §5f0以来初の恒久安定
エネルギッシュBound**を達成。rate-deadbeat(4倍遅延止まり)と対照的に、
足配置は**15秒間ずっと直立**。文献(Raibert/HZDのデッドビート写像)が
「エネルギッシュboundingの安定化に必要」とした手法が、現MPCに
足配置1項を足すだけで機能することを実証。

**残課題**: +0.06 は後退移動(cmd=+0.5前進なのに後退)——足を前に
出すと反作用で後退するRaibertカップリング。cmd方向と一致させる
(前進エネルギッシュBound)には fore-aft速度regulator併用か
neutral項の見直しが要る。25秒延長で真の恒久性を確認中。

**§5f6追記 — 25秒スイープで安定帯を精緻化**: §5f6の15秒スナップ
ショット(0.03/0.06安定)を25秒(83周期)に延長し、真の安定帯を確定:

| k_rate | 25秒結果 |
|---|---|
| +0.03 | t=24sまで持続後 転倒(やや弱く、緩慢発散が残る) |
| **+0.045** | **★25秒完全安定**: roll 0.00-0.10、pitch 0.40-0.66(90°未満)、min_z 常に正(0.152-0.222m)、z_range 0.09-0.22m(最大22cmの対空) |
| **+0.06** | **★25秒完全安定**: roll<0.12、min_z正、z_range 0.10-0.17、後退 ~0.5 m/s |
| +0.08 | t=7s転倒(過補正でroll不安定→反転し -1.5m/sで暴走) |

**確定した安定帯 k_rate ∈ [0.045, 0.06]**(0.03弱すぎ/0.08強すぎ)。
**k_rate=0.045 / duty=0.34 が25秒=83周期 完全安定・最大対空22cm**——
Poincaréデッドビート足配置による**真の恒久安定エネルギッシュBound**。
min_z 常に正(クリーンなホップ)、roll/pitch有界=安定極限周期の実証。
移動方向は後退(cmd +0.5 に対し実 −0.5、Raibertカップリング)で
cmd整合は残課題だが、**「安定 AND 本物の対空」という§5f0以来の
主目標は達成**。ゲイン過大(0.08)は roll を巻き込む逆効果、という
上限も判明。

### 5f7. 前進化の試みと本質的トレードオフの発見(2026-07-26)

§5f6で恒久安定を達成したが、**cmd +0.5前進に対し実 −0.5後退**という
方向問題が残った(ユーザー指摘)。3つの独立手法で前進化を試みたが、
いずれも失敗し、**本質的トレードオフ**が判明した。

**§5f7a 死因(後退)の分析**: pitch-deadbeat のシフト量 =
k_rate·pitch_rate。エネルギッシュBoundの pitch_rate は数 rad/s なので
0.045×4 ≈ **0.18m = max_step 飽和**。pitchを安定化するに足るゲインが、
足配置を毎歩フル前方に飽和させ、強い後退駆動を生む。

**試みた3手法(全て duty=0.34, k_rate=0.045, 15s)**:

| 手法 | 結果 |
|---|---|
| fore-aft速度regulator (0.02-0.2) | 後退のまま。高ゲインで跳ね潰れ(z_range 0.03)、0.2で転倒 |
| touch_down-only 分離(half非対称化) | 後退のまま(歩幅中心バイアスが原因ではないと判明) |
| **F_x前進推進 bias (20-90N)** | **ドリフト不変**(20N −0.45 / 90N −0.45)= 運動学的ロック |

**本質的トレードオフ(実証)**: F_x推進を90Nまで上げても後退速度が
不変ということは、**後退は力バランスでなく飽和した足配置による
運動学的拘束で決まっている**。pitch安定化(足を前へ飽和)と前進
(足を後ろへ)が**同一の限られた足配置エンベロープを奪い合い**、
ボルトオンの前進駆動(配置ベースも力ベースも)では覆せない。
さらに、滞空を増やす(duty↓)ほど pitch外乱↑→必要配置↑→後退↑
という**トレードオフ・フロンティア**が存在する。

**§5f 最終総括**:
- **達成**: 恒久安定(25s=83周期)エネルギッシュBound、本物の対空
  10-22cm、min_z常に正。Poincaréデッドビート足配置により、状態重み
  では不可能だった真の極限周期安定化を実現(§5f0以来の主目標)。
- **限界**: その場〜後退(~0.5 m/s)。**前進 と pitch安定化が同じ
  足配置を奪い合う**アーキテクチャ上のトレードオフで、ボルトオン
  駆動では前進化できない。
- **真の前進エネルギッシュBoundへの道**: HZD/gait-library——**前進
  する周期軌道を設計し、その周りの線形化としてデッドビートを導出**
  する(配置補正が前進ノミナルの小摂動になり、大DC配置が前進と競合
  しない)。これは軌道最適化+軌道安定化のパイプラインで、パラメータ
  調整でなく新規定式化。

### 5f8. HZD前進軌道の実装 — 安定化するが前進せず、軌道最適化が必要と確定(2026-07-26)

ユーザー指示「HZD前進軌道を実装(本格)」に従い、前進する周期軌道の
偏差フィードバックとしてデッドビートを再定式化。2つのHZD的機構を実装
したが、いずれも**安定化はするが前進は達成できず**、真の前進には
軌道最適化が必要と確定した。

**実装(全て既定offのオプトイン、非破壊)**:
1. **軌道相対デッドビート**: `bound_trim_config()` を共有ヘルパ化し、
   デッドビートを `pitch − trim_nominal(phase)` / `pitch_rate −
   trim_pitch_rate(phase)` の**偏差**フィードバックに変更(トリム軌道
   が前進 nominal を与え、偏差補正が定常で0になれば後退DC配置が消える
   狙い)。`BoundTrimSample` は既に `pitch_rate` を持つので追加不要。
2. **DC-blocker**: 適用シフトの遅いEMAを差し引き、残留する前進バイアス
   (=後退駆動)だけを除去、安定化するAC偏差成分を残す
   (`bound_pitch_placement_dc_tau`、tick内で1回計算、全脚共有)。
   足配置を touch_down のみに適用する形にも整理(§5f7)。

**結果(duty=0.34, cmd_vx=0.5, 15s)**:

| 構成 | 安定 | 前進 |
|---|---|---|
| トリム単独 (k_rate=0) | t=7sで転倒 | **vx≈0**(cmd+0.5未達) |
| 軌道相対 k_rate=0.03-0.06 | 15s安定 | 後退 −0.3〜−0.7 |
| DC-block tau=0.4-1.5 | 15s安定 | 後退(一部窓で瞬間0、非持続) |

**決定的発見**: **k_rate=0のトリム単独ですら vx≈0** で cmd+0.5 の前進を
達成していない——**前進速度追従そのものがこのエネルギッシュ構成
(duty=0.34)で破綻**しており(§5biのbound速度天井と同根)、pitch
安定化はその上に後退を上乗せしているだけ。デッドビート(絶対/軌道
相対/DC-block)は姿勢を安定化するが、破綻した前進追従を回復できない。

**§5f 前進化の最終結論(7手法の帰結)**: rate-deadbeat / 絶対placement /
速度regulator / touch_down分離 / F_x推進90N / HZD軌道相対 / DC-blocker
——**すべてで後退が覆らない**。エネルギッシュBoundの前進は、既存
MPCへのボルトオン制御(状態重み・足配置・力バイアス・偏差
フィードバック)の範囲外。**真の前進エネルギッシュBoundには、前進する
全身周期軌道を contact-implicit trajopt / TOWR で設計し追従する**という、
§5d8サーベイが指摘した軌道最適化パイプライン(新規ソルバ/多日規模)が
必要——これが実証的に確定した最終所見。

**達成の確定**: Poincaréデッドビート足配置により **恒久安定(83周期)・
本物の対空(10-22cm)のエネルギッシュBound**(その場〜後退)を実現。
状態重みでは不可能だった真の極限周期安定化を、既存WBCへの足配置1項
+状態重み2個で達成した(§5f6)。前進は次段(trajopt)の課題。

### 5f9. trajopt P0: 前進周期Bound軌道の存在を確認(2026-07-26)

ユーザー指示「trajoptの設計・調査に進む」を受け、設計文書
`ref/bound_trajopt_design.md`(2段構成: 段1=周期軌道trajopt生成、
段2=既存MPC/WBC参照差し替え+デッドビート)を作成し、その前提=
**前進エネルギッシュBoundの実行可能周期軌道が存在するか**を P0 で確認。

`ref/scripts/bound_trajopt_p0_shooting.py`(単一シューティング、平面
SRBD、周期性硬制約+摩擦/fz≥0/到達性/pitch有界ペナルティ)で:
**前進周期Bound軌道が存在(feasible)** — 残差1.2e-7、vx=1.000、
pitch 0.161rad(有界)、reachability 0.128m、摩擦マージン≈0、
z_range 4cm。**§5f の前進失敗の真因=この軌道が参照として無かった**
ことが裏付けられ、trajopt計画の前提が確定。詳細は設計文書付録A。

### 5f10. trajopt P2: 前進参照は有効だが既存追従器では不十分(2026-07-26)

P0前進軌道を48行CSV化し、コントローラの表参照注入機構
(`set_bound_tabulated_reference`、位相補間で z/pitch/vx/vz/ω を MPC
参照に投入)経由で段2を実装。結果(詳細は設計文書付録B):
**前進参照は前進を引く**(§5f8平坦参照 vx≈0 に対し初期 vx +0.1〜0.2)
が、**MPCが vx=1.0 を追従できず定常で後退/≈0**(§5bi速度天井と同根)。
**安定化と前進が両立しない**——placement=恒久安定だが後退ドラッグ、
placement無=後退しないが転倒。DC-block/base_pos重み/orbit-relativeも
破れず。**段2だけでは不十分、P1(全身周期軌道+専用追従器で安定化を
軌道の一部として同時最適化)が必要**と確定。表参照/deadbeat機構は
既定offで残置、P1部品として再利用可能。
