# kyo46rs_walk.rs の環境変数ノブの棚卸し（2026-10-29、チャンピオン v6.19 時点）

対象: `examples/kyo46rs_walk.rs`（12,747 行、`main()` は 178〜12729 行）。目的は、歩行制御の本体を `misa-biped` の `Walker` + `Config` へ切り出すとき、**採用済みの仕組みだけを持っていき、不採用のノブを落とす**ための台帳。コードは読んだだけで、実行による確認はしていない（限界は §1.3）。

## 0. 要点

- 環境変数の名前は **704 個**（`env_f64` 474 呼び出し・`flag` 168・`std::env::var` 直読み 84・`phase_gain` 16 から重複を除いた数。`WBCW_<段名>` の族は 1 個と数えた）。これに `phase_gain` が作る `<名前>_XFER`／`<名前>_POST` が 16 組 32 個ある。依頼文の「約 472」は `env_f64` の呼び出し数に近いが、実際の名前の数はその 1.5 倍。
- 制御の本体として持っていくもの（分類 1・1k・2）は **240 個**。落とす候補（分類 3・3d と `_XFER/_POST`）は **362 個**で、半分を超える。うち最大の塊は段の三相移行（`STAIR_TRANSFER` 系、88 個＋`_XFER/_POST` 32 個）で、チャンピオンは `STAIR_TRANSFER=0`。
- **チャンピオンの controls に値があるのに効いていないノブが 10 個ある**（分類 3d）。`STAIR_TRANSFER=0` の下の 7 個（`STAIR_CAPTURE_TIME`、`STAIR_TRANSFER_DCM`、`STAIR_RISE_LOAD`、`STAIR_COMMIT_LOAD`、`STAIR_TRANSFER_HOLD`、`STAIR_TRANSFER_KEEP_DCM`、`STAIR_TRUNK_COUPLE`）、`ADAPT_STEP=0` の下の `ADAPT_MAX_X/Y`、それに `CP_MAX_X=CP_MAX_Y=CP_MAX_Y_IN=0` でずらしが常に 0 になる contact_plan の `CP_W_SHIFT=40`。CEM で調整した名残で、切り出しでは持っていかない。
- 名前と中身がずれているものがある。`STAIR_WRENCH_MPC`／`STAIR_WRENCH_N`／`STAIR_WRENCH_DT`／`STAIR_WRENCH_LOW_MIN`／`STAIR_HYBRID_CONTACT` は段に限らず常時の重心 MPC の設定で、`WRENCH_MPC_APPLY` は加速度・力の名目・fz 計画を一度に開く複合ゲート。`PUSH_REPLAN_HOLD=10` は「10 m 動いたら保持」で、実質は保持を切るスイッチ。Config では意味の名前に付け替える。
- G1 のベンチは平地も含めて全部 `STAIRS=…` で段差 0 の「階段」を置いている（rand.py）。そのため `stairs.is_some()` で門をした既定（`ZMP_ROLL=0.030`、`VH_DCM`、段ゾーンの判定、段の足跡計画）は**全ベンチで生きている**。ライブラリでは「地形の模型が渡されたか」と「段があるか」を分けないと、平地の挙動が変わる。
- **既定値は機体ごとに違う形にする必要がある**: kyo46rs 基準（`kyo_base_v2.json`）は重心 MPC・contact_plan・DS_FEASIBLE・MOM・参照ライブラリのどれも入れておらず、既定 off の古典的な経路（DCM 則＋予見＋段の足跡）で歩いている。逆に `TRACKING_CUBIC_FF=0` を明示し、`STAIR_GAIT=pair`／`STAIR_FULL_SOLE=1`／`COM_XY_JERK_MAX=100` など G1 で使わないノブを 12 個（分類 1k）使う。`Config::default()` を G1 チャンピオンにすると kyo46rs の基準が変わるので、機体ごとのプリセット（`Config::g1_v619()`／`Config::kyo46rs_v2()`）で持つのがよい。
- ライブラリ側にも環境変数の直読みが残っている: `crates/misa-biped/src/preview.rs:587-588`（`LAND_CONFIRM_XY`／`LAND_CONFIRM_Z`、着地確認の既定の許容）と各種 `*_DEBUG`。チャンピオンで効いているノブ 5 個（`QD_CLAMP`、`BASE_VEL_FUSE_HZ`、`BASE_VEL_FUSE_BIAS_T`、`SENSE_NOISE_A`、`SENSE_ACC_BIAS`）は walk.rs ではなく `src/biped/rig.rs` が読む（付録 A）。

## 1. 個数と方法

### 1.1 分類ごとの個数

| 分類 | 意味 | 個数 | うちチャンピオン controls にあるもの |
|---|---|---:|---:|
| 1 採用 | 採用: G1 チャンピオン v6.19 の controls で既定と違う値（既定 on を切るものを含む） | 111 | 111 |
| 1k 採用（kyo46rs 基準） | 採用（kyo46rs 基準だけ）: `kyo_base_v2.json` で既定と違う値、G1 チャンピオンには無い | 12 | 0 |
| 2 既定で有効 | 既定で有効: 既定値のまま、チャンピオン（または採用した親の仕組み）の中で働く | 117 | 7 |
| 3 不採用・既定 off | 不採用・既定 off: 既定で off、または親の仕組みが off で到達しない | 320 | 6 |
| 3d 死に値 | 死に値: チャンピオンに値があるが、門が閉じていて効かない | 10 | 10 |
| 4 シナリオ | シナリオ・ベンチ: 押し、地形、個体振り、速度指令、時間、課題 | 42 | 4 |
| 5 計測 | 計測・ログ・デバッグ | 58 | 0 |
| 6 機体 | 機体の定数・リグ: Profile と模型・植物の設定 | 27 | 6 |
| 要確認 | 要確認 | 7 | 0 |
| 計 | | 704 | 144 |
| （別枠）`<名前>_XFER`／`_POST` | `phase_gain` 16 個の段移行中／移行後の値。段移行が off なので全部到達しない（分類 3 扱い） | 32 | 0 |

チャンピオンの controls.json は 150 キー。walk.rs が名前で読むのが 143、`WBCW_COM`／`WBCW_POSTURE` は `format!("WBCW_{key}")` 経由（表では `WBCW_*` の 1 行）、残り 5 キーは `src/biped/rig.rs` が読む（付録 A）。controls にあって分類 4・6 にしたもの（`ROBOT`、`URDF`、`T`、`N_STEPS`、`SIM_DT`、`STAIR_LANDING_LEN`、`ARMATURE`、`DAMPING_OVERRIDES`、`SOLE_HALF_L`、`ELBOW`）は、値が勝ち残った意味はあるが制御の Config には入らない（ハーネスか Profile へ）。controls に既定と同じ値で明示されているもの（`KP_ARM=400`、`MPC_FOOTSTEP_W=100` など）は分類 2、既定と同じ off を明示しているもの（`STAIR_TRANSFER=0`、`ARM_MERGE=0`、`ADAPT_STEP=0`、`WBC_WEIGHTED_OFF_STAIRS=0`、`MPC_COP_REF_CMD=0`、`STAIR_RISE_SOLE_POSE=0`）は分類 3。

### 1.2 判定のしかた

1. `env_f64(` `flag(` `phase_gain(` `std::env::var(` の呼び出しを括弧の対応で切り出し、名前・既定値の式・束縛先の変数・宣言行を取った（同じ名前を複数箇所で読むものは最初の行を宣言行とした）。
2. 束縛先の変数が `main()` のどこで使われるかを全行で引き、門（`if 親 && …`）を読んで、親の仕組みが off なら子は分類 3 にした。たとえば `STAIR_TRANSFER=0` の下の `STAIR_*`、`RESTEP` の下の `RESTEP_*`、`MPC_LANDING_TIME`／`MPC_CAPTURE_LEX` の下のループ内の読み、`CONTACT_EVENT` の下の `CONTACT_EVENT_SHIFT`（既定 true でも到達しない）。
3. 既定値の比較: G1 の Profile（`crates/misa-biped/src/profile.rs` の `g1_23dof`）を引いて、`prof.*` 由来の既定とも比べた（例: `CTRL_DT` の既定は 2 ms、チャンピオンは 5 ms → 分類 1。`SOLE_HALF_L=0.085` は prof と同値）。
4. 採用の経緯は `doc/from_claude/STATUS.md` と記憶ファイル bd-baseline-v1 で確かめ、不採用の記述は `doc/from_claude/*.md` を名前で grep して照合した（例: `LIFT_GATE_FRAC`、`LOAD_RAMP_CONTACT_T`、`PUSH_REPLAN_LAT`、`DS_CAPTURE_LAT`）。v6.13 の部品（`MPC_FOOTSTEP_SHIFT_REST`／`HOLD_SHIFT`／`CAPTURE_FLOOR`／`BACK_ONLY`／`BACK_PERSIST`／`BACK_VMAX`）は v6.14 で `PUSH_REPLAN` に置き換わって controls から外れたので分類 3。
5. ベンチ側は `log/claude_g1rand_20260925/family.sh` と `log/claude_g1rand_20260924/rand.py` が入れるノブを分類 4 にした。ただし、ベンチが入れるのに中身は制御の判断であるもの（`TERRAIN_PITCH`、`ROUGH_MODE`、`VX2_SCHED`）は「要確認」にした。
6. kyo46rs は `log/claude_g1rand_20260925/kyo_base_v2.json`（基準 v2）と `kyo_base_v3.json`（v3 候補、判断待ち）を見た。v2 で既定と違う値のノブで G1 チャンピオンに無いものを分類 1k、v3 候補だけで変わるものを「要確認」にした。

### 1.3 判定の限界

- 静的に読んだだけ。「到達しない」は門の条件を読んだ結果で、実行して確かめてはいない。確かめるなら、チャンピオンのバイナリで分類 2・3 の各ノブを 1 つずつ振って短い平地と押し 1 本を回し、軌跡のビット一致で「効く／効かない」を機械的に判定するのが確実（700 本強、W=24 で数分〜十数分）。特に 3d の 10 個と、分類 2 のうち使用箇所が 1 箇所だけのもの。
- 分類 2 は「チャンピオンのどこかのベンチで値が使われる」の意味で、平地の公称歩行で効くとは限らない（例: `STAIR_DESC_STAND_MARGIN` は降段ベンチだけ、`TURN_IN_DS` は旋回指令があるときだけ、`VXS_*` は速度指令があるときだけ）。
- 段ゾーン・速度指令・押しの検出などの門は実行時の状態で決まるので、「親が on なら子も生きる」として扱った。親が on でも子の値が実際の解に影響しない場合（例: `CP_W_SHIFT`）は、計算式まで読んだものだけを 3d にした。同じ型の見落としは他にもありうる（例: `MPC_LANDING_CAPTURE_W=1e6` は `MPC_LANDING_LEX=1` の下では「0 より大きい」ことしか意味を持たない）。
- 宣言行は最初の読みの行。`COM_REPLAN`（1959／2610）、`COM_LAND_VXY/VZ`（1960／3065）、`LOAD_RAMP_FRAC`（3777／6312）などは 2 箇所で読んでいる。
- 依頼の「`main()` が 178〜12730 行」は `#[cfg(feature = "mujoco")]` 版の `main()`。12730 行の `main()` は mujoco 無しのときの案内だけ。

## 2. 仕組みごとのまとまり（採用・既定で有効）

各まとまりが将来の `Config` のサブ構造体の案。「採用」はチャンピオンの値、「既定」は既定値のまま働くもの、「落とす」は同じまとまりの分類 3／3d。行番号は `examples/kyo46rs_walk.rs`。モジュールは `articara::biped::*` の再輸出先（`crates/misa-biped` と `crates/misa-mpc`、作業中の移設に合わせた）。

| まとまり | Config 案 | 採用 | 1k | 既定 | 要確認 | 落とす |
|---|---|---:|---:|---:|---:|---:|
| WBC の解き方・重み・指令方策 | `WbcConfig` | 5 | 0 | 7 | 0 | 9 |
| 関節制限の CBF | `JointLimitConfig` | 4 | 0 | 2 | 0 | 4 |
| 重心・胴・姿勢の基本タスク（利得） | `BalanceGains` | 4 | 0 | 8 | 0 | 3 |
| DCM 追従と ZMP | `DcmConfig` | 3 | 2 | 3 | 0 | 4 |
| 歩容の時間割と公称の足跡 | `GaitConfig` | 6 | 0 | 4 | 0 | 0 |
| 段の足跡計画（踏面への並べ直し） | `StairPlanConfig` | 6 | 2 | 1 | 0 | 0 |
| 降段の前置き（重心の先行下げ・しゃがみ・確認の許容） | `DescentConfig` | 6 | 0 | 4 | 0 | 12 |
| 既知の斜面 | `SlopeConfig` | 3 | 0 | 1 | 1 | 0 |
| 遊脚の軌道と追従 | `SwingConfig` | 6 | 3 | 6 | 3 | 7 |
| 腕の保持と角運動量タスク | `ArmMomentumConfig` | 7 | 1 | 5 | 0 | 6 |
| 圧力の不足を角運動量率で補う（MOM_CMP） | `MomCmpConfig` | 4 | 0 | 3 | 0 | 2 |
| 接触集合と荷重の受け渡し | `ContactConfig` | 3 | 1 | 7 | 0 | 20 |
| 着地の確認と待ち（昇り・平地） | `LandingConfig` | 1 | 1 | 3 | 0 | 18 |
| CoM 予見と高さの計画 | `ComPlanConfig` | 3 | 2 | 10 | 0 | 7 |
| contact_plan（片脚支持の終わりの予測的な決定） | `ContactPlanConfig` | 7 | 0 | 7 | 0 | 8 |
| 両脚支持の作り直し（DS_FEASIBLE・DS_CAPTURE） | `DoubleSupportConfig` | 6 | 0 | 7 | 0 | 4 |
| 重心のレンチ MPC 本体 | `WrenchMpcConfig` | 9 | 0 | 13 | 0 | 38 |
| MPC の着地変数 | `MpcFootstepConfig` | 9 | 0 | 3 | 0 | 33 |
| MPC の交差の一歩 | `MpcCrossConfig` | 4 | 0 | 1 | 0 | 1 |
| 押しの後の再計画 | `PushReplanConfig` | 5 | 0 | 9 | 0 | 6 |
| 後ろ押しの検出に連動した重心下げ | `PushCrouchConfig` | 1 | 0 | 3 | 0 | 0 |
| 速度の表と押しの譲歩 | `SpeedScheduleConfig` | 4 | 0 | 7 | 1 | 6 |
| 押し回復の参照ライブラリ | `RecoveryRefConfig` | 5 | 0 | 3 | 0 | 11 |
| 不整地モード（要確認） | `RoughModeConfig` | 0 | 0 | 0 | 2 | 6 |

### 2.1 WBC の解き方・重み・指令方策 → `WbcConfig`

- 採用: `WBC_FEAS_TOL=1e-8`、`WBC_STRATEGY=weighted`、`FALLBACK_LEVEL=3`、`CTRL_DT=0.005`、`WBCW_*`
- 既定で有効: `F_MAX_SCALE`、`F_MAX`、`WBC_QP_BACKEND`、`WBC_WEIGHT_RATIO`、`friction_mu`、`EOM_HARD`、`HOLD_BRIDGE`
- 箇所: 宣言 711–850、1173–1183、10888–10931（`WBCW_*` はループ内で読む）。実装: 10636–10931（段の組み立てと重み付け）、10932–11124（解法・修復・受け入れの門）、2240–2246 と 11592–11635（`CommandPolicy`、劣化時の保持、指令の書き込み）
- モジュール: `misa_wbc`（`HqpStrategy::WeightedQp`、`Solver`、`SolveConfig`）、actuate（`CommandPolicy`、`write_to_plant`）、tasks（`bt::regulariser`）
- 落とす（9）: `WBC_PREFIX_MIN_LEVEL`、`WBC_REPAIR_ORIGINAL`、`PROX_WEIGHT`、`WBC_QP_RESET_EVERY_TICK`、`WBC_HARD_PHYSICAL`、`WBC_WEIGHTED_OFF_STAIRS`、`HOLD_LAST`、`BLEND_TICKS`、`WBC_WEIGHTED_ZONE_TAIL`
- 所見: `WBCW_*` の段名（"com+momentum" など）は段の組み立ての文字列から作っている。Config では段の列挙型に重みを持たせる。`CTRL_DT` は制御周期で、Profile（G1 は 2 ms）と Config（チャンピオン 5 ms）のどちらに置くかを決める必要がある。

### 2.2 関節制限の CBF → `JointLimitConfig`

- 採用: `JLIM=1`、`JLIM_ALPHA1=25.6107`、`JLIM_ALPHA3=20.6833`、`JLIM_AMAX=351.019`
- 既定で有効: `JLIM_ALPHA2`、`JLIM_V_FRAC`
- 箇所: 宣言 859–975。実装: 6399（基底タスクに CBF を足す）、4460–4464（段階別の上書き）、10645、11150
- モジュール: tasks（`bt::joint_limits`、中身は `misa_wbc::tasks::joint_limit_cbf`）、rig（関節の範囲）
- 落とす（4）: `ARM_PITCH_BAND`、`ARM_PITCH_VMAX`、`ARM_PITCH_AMAX`、`JLIM_Q_MARGIN`

### 2.3 重心・胴・姿勢の基本タスク（利得） → `BalanceGains`

- 採用: `KP_COM=419.064`、`KD_COM=48.1952`、`KP_TRUNK=73.0103`、`KD_TRUNK=28.2069`
- 既定で有効: `KP_POST`、`KD_POST`、`POST`、`TRUNK`、`TRUNK_SIGN`、`TRUNK_YAW`、`KP_YAW`、`KD_YAW`
- 箇所: 宣言 977–983、1169–1173、1423–1425。実装: 8240–8260（高さの PD）、9549–9568（CoM タスク）、9569–9672（胴の姿勢とヨー）、9932–9944（姿勢）
- モジュール: tasks（`bt::com`、`bt::trunk_ori_ref`／`bt::trunk_rpy`／`bt::TrunkGains`、`bt::posture`）
- 落とす（3）: `TRUNK_DEAD`、`TRUNK_LATE`、`GAIN_OMEGA_REF`
- 所見: CEM で G1 に合わせた利得（`KP_COM` など）は機体ごとの値。Config に置いて機体ごとの既定を Profile から与える形が自然。

### 2.4 DCM 追従と ZMP → `DcmConfig`

- 採用: `COP_FRAC=0.912835`、`K_DCM=3.09069`、`VH_DCM=1`
- 採用（kyo46rs 基準）: `ZMP_ROLL=0.02107`、`COM_XY_JERK_MAX=100`
- 既定で有効: `ZMP_MARGIN`、`CLAMP_ZMP`、`ZMP_LAT_SCALE`
- 箇所: 宣言 1174、1237–1242、1491、2036、4263。実装: 6647–6700（DCM 則の入り）、7337–7365（ZMP の切り詰めと拒まれた圧力）、7989–8058（`p_cmd`、`VH_DCM`、CoM 加速度とジャーク制限）
- モジュール: dcm（`DcmPlan::from_footsteps_rolled`、`commanded_zmp`、`com_accel_xy`、`dcm_of`）
- 落とす（4）: `DCM_PREVIEW_HORIZON`、`DCM_REF_BLEND_T`、`DCM_REF_BLEND_MIN`、`DCM_LEAD_S`
- 所見: `VH_DCM` はループ内（8028）で毎 tick `flag()` を読んでいる。`ZMP_ROLL` の既定は `stairs.is_some()` で 0.030／0 に分かれる。

### 2.5 歩容の時間割と公称の足跡 → `GaitConfig`

- 採用: `HIP_ROLL_SEED=0.122728`、`T_DS=0.365543`、`T_SS=0.843964`、`DS_SETTLE=0.0840506`、`STRIDE=0.148842`、`STRIDE_RAMP=3.15826`
- 既定で有効: `FIRST_SWING`、`DS_SETTLE_AFTER_STEP`、`ALTERNATE`、`TURN_IN_DS`
- 箇所: 宣言 360、1244–1250、1426–1441、1498–1507、1690、9617。実装: 1426–1441（`GaitPlan` と `DS_SETTLE`）、1690–1700（`FootstepPlan::velocity_with_change`）、2438（DCM 計画の初期化）、9605–9640（旋回の時期）
- モジュール: gait（`GaitParams`、`GaitPlan`、`FootstepPlan`）、dcm
- 所見: `HIP_ROLL_SEED`（立ち幅を決める初期姿勢）は CEM で調整された値なので採用扱いにしたが、置き場所は Profile の姿勢の種。

### 2.6 段の足跡計画（踏面への並べ直し） → `StairPlanConfig`

- 採用: `STAIR_TOE_MARGIN=0`、`STAIR_Y_OUT=0.00442314`、`STAIR_TD_BIAS=0.000780737`、`STAIR_TD_BIAS_ASC_MID=0.02`、`STAIR_LANDING_FRAC=0.942684`、`STAIR_ALIGN_STAND_LINE=1`
- 採用（kyo46rs 基準）: `STAIR_GAIT=pair`、`STAIR_FULL_SOLE=1`
- 既定で有効: `STAIR_SOLE_MARGIN`
- 箇所: 宣言 1719–1856。実装: 1697–1955（段の区間を決定的に作り直す足跡計画。斜面の `SLOPE_KEEP_STRIDE` もここ）
- モジュール: gait（`FootstepPlan`）、terrain（`Stairs`）
- 所見: 全部ループの前の計画作りで、ループ中の状態を持たない。切り出しは最も易しい。G1 のベンチは段差 0 の段を必ず置くので、平地のベンチでもここを通る（立ち線 `STAIR_ALIGN_STAND_LINE` など）。

### 2.7 降段の前置き（重心の先行下げ・しゃがみ・確認の許容） → `DescentConfig`

- 採用: `STAIR_TD_BIAS_DESC=0.02`、`COM_HEIGHT_DESC_LEAD=0.5`、`STAIR_ZONE_CROUCH=0.04`、`COM_LAND_EXTEND_DESC=0.25`、`LAND_CONFIRM_Z_DESC=0.003`、`LAND_CONFIRM_XY_DESC=0.03`
- 既定で有効: `LEG_REACH_MARGIN`、`STAIR_DESC_STAND_MARGIN`、`STAIR_ZONE_CROUCH_LEAD`、`STAIR_ZONE_CROUCH_TAIL`
- 箇所: 宣言 470、1755–1757、1856、2750–2763、3026、3062–3063。実装: 1745–1800 と 1856–1880（降りの足跡と後退）、4499–4525（降りの確認の許容と延長）、8059–8222（高さ参照: 8143–8160 で `height`、8187–8205 で届く範囲の上限）、9778–9781（段ゾーンの判定を MOM_CMP と共有）
- モジュール: height（`hr::`）、reach（`leg_reach`）、preview（`LandingTiming` の上書き）、gait
- 落とす（12）: `COM_DESC_DS`、`COM_DESC_XFER`、`STAIR_DESC_MIN`、`STAIR_DESC_TOE`、`STAIR_ANCHOR_FLAT`、`REACH_HEIGHT`、`REACH_HEIGHT_FLOOR`、`COM_HEIGHT_DESC_REACH`、`LAND_CONFIRM_XY_DESC_CP`、`COM_LAND_EXTEND_DESC_CP`、`LAND_CONFIRM_FZ_DESC_FRAC`、`STAIR_DESC_TOE_START`
- 所見: 段ゾーン（`STAIR_ZONE_CROUCH_LEAD/TAIL`）は MOM_CMP の無効化と weighted の範囲判定にも使われる共有の概念。Config では「段ゾーン」を独立した小構造体にする。

### 2.8 既知の斜面 → `SlopeConfig`

- 採用: `SLOPE_KEEP_STRIDE=1`、`SLOPE_HEEL_CUT=0.010`、`TERRAIN_PITCH_YAW=1`
- 既定で有効: `SLOPE_HEEL_CUT_MAX`
- 要確認: `TERRAIN_PITCH`
- 箇所: 宣言 1781、2396–2397、3017–3018。実装: 1912（歩幅を保つ）、6565–6593（踵側の CoP 箱を削る）、10495–10530（遊脚の追従軸 `SwingAxes::XyzYawPitch`）
- モジュール: terrain（`pitch_at`）、tasks（`SwingAxes`）、contact（CoP の箱）
- 所見: `TERRAIN_PITCH_YAW=1`（採用）は `TERRAIN_PITCH=1` のときだけ効く。`TERRAIN_PITCH` はチャンピオンに無く、斜面ベンチが入れている → 要確認。

### 2.9 遊脚の軌道と追従 → `SwingConfig`

- 採用: `KP_SWING=477.044`、`KD_SWING=31.493`、`LIFT_H=0.0512879`、`TRACKING_CUBIC_FF=1`、`TRACKING_FF_MIN_RISER=0.10`、`STAIR_TD_SOLE_FRAME=1`
- 採用（kyo46rs 基準）: `KP_SWING_XY=400`、`STAIR_TD_SINK=0.009`、`STAIR_SWING_WINDOWS=0.50,0.38,0.80,0.70`
- 既定で有効: `KP_SWING_ROT`、`KD_SWING_ROT`、`TRACKING_FF_SCALE`、`SWING_ROT`、`STAIR_LAND_SINK`、`STEP_DZ_SWING_MIN`
- 要確認: `STAIR_ABSOLUTE_TD_DESC`、`STAIR_SWING_FF`、`STAIR_ZONE_SWING_ROT`
- 箇所: 宣言 1213–1225、1380–1412、2336–2440、3009–3010、3259。実装: 10071–10635（遊脚タスク全体: 10120–10180 で着地目標、10216–10310 で段の遊脚の形と前置き、10325–10375 で保持と遅れ、10495–10560 で姿勢の軸）、5853–5857（段ゾーンの回転モード）
- モジュール: gait（`swing_position`／`swing_velocity`、`stair_swing_windows`）、tasks（`bt::swing_with_pose`／`bt::swing_with_pose_acceleration`、`bt::SwingAxes`）
- 落とす（7）: `LAND_KD_ROT_BOOST`、`STAIR_ABSOLUTE_TD`、`STAIR_SWING_PROFILE`、`SWING_TOE_UP`、`SWING_TOE_UP_W`、`STAIR_RISE_SOLE_POSE`、`STAIR_RISE_SOLE_POSE_MIN_DZ`
- 所見: 遊脚の目標は `footsteps`（計画）、`landing_timing`（延長）、押しの再計画（10404 の保持）、MPC の交差（10407–10420）の 4 者が書く。Config より先に「遊脚目標の権限」の整理が要る。

### 2.10 腕の保持と角運動量タスク → `ArmMomentumConfig`

- 採用: `ARM_PITCH=0`、`ARM_HOLD=1`、`ARM_BEFORE_SWING=1`、`MOM=1`、`KP_MOM=77.0372`、`MOM_REF=arms_neutral`、`MOM_ARM_DCM=27.4033`
- 採用（kyo46rs 基準）: `ARM_HOLD_JOINTS=shoulder`
- 既定で有効: `KP_ARM`、`KD_ARM`、`MOM_AXES`、`MOM_LEVEL`、`MOM_ARM_DCM_MAX`
- 箇所: 宣言 361、1000–1135。実装: 9673–9773（角運動量タスク、`MOM_ARM_DCM`）、9822–9931（腕の保持、肘、突き）、10779–10830（段の順序 `ARM_BEFORE_SWING`、`MOM_LEVEL`）
- モジュール: tasks（`bt::angular_momentum`、`bt::posture` の腕の行）、`misarta::centroidal`
- 落とす（6）: `MOM_MERGE`、`ARM_MERGE`、`ELBOW_KD`、`MOM_REF_MPC`、`MOM_ARM_DCM_X`、`MOM_ARM_DCM_X_MAX`
- 所見: 突き（`JAB*`）は課題なのでハーネス側。`ARM_PITCH`（腕の種の姿勢）は採用値 0 だが置き場所は Profile。

### 2.11 圧力の不足を角運動量率で補う（MOM_CMP） → `MomCmpConfig`

- 採用: `MOM_CMP=1`、`MOM_CMP_SIGN=-1`、`MOM_CMP_HMAX=1`、`MOM_CMP_KH=2`
- 既定で有効: `MOM_CMP_MAX`、`MOM_CMP_DEAD`、`MOM_CMP_W`
- 箇所: 宣言 1147–1166。実装: 7363–7365（拒まれた圧力）、9774–9821（タスク）、10736–10738（CoM 段への合成）
- モジュール: tasks、dcm
- 落とす（2）: `MOM_CMP_REF_ONLY`、`MOM_CMP_STAIRS`
- 所見: 段ゾーンでは無効（`MOM_CMP_STAIRS` が既定 false = 採用形）。

### 2.12 接触集合と荷重の受け渡し → `ContactConfig`

- 採用: `CONTACT_ADMIT_CONFIRMED=1`、`CONTACT_ADMIT_SCHEDULED=1`、`CONTACT_ADMIT_LOADED=0.04,0.02`
- 採用（kyo46rs 基準）: `CONTACT_ON_BW=0.05`
- 既定で有効: `ANCHOR_LEAK`、`KP_CONTACT`、`KD_CONTACT`、`CONTACT_DRIVEN`、`CONTACT_TICKS`、`CONTACT_ADMIT_Z`、`LOAD_RAMP_FRAC`
- 箇所: 宣言 1182–1203、2251–2262、2646–2668、3777。実装: 4655–4700（確認から admit へ）、5835–6080（予定から接触集合、5962–6040 の admit）、6283–6400（両脚支持の荷重傾斜）、6478–6610（接触段、CoP パッチ）
- モジュール: gait（`ContactCorrection`）、contact（`Anchors`、`contact_jacobians`、`cop_from_sole_wrench`）、tasks（`bt::contact_level`、`bt::ContactCfg`、`bt::force_nominal`）
- 落とす（20）: `ANCHOR_LEAK_ROT`、`NO_PATCH`、`CONTACT_SCHEDULED_REENTRY`、`STAIR_ROLL_PATCH`、`CONTACT_FZ_RATE`、`CONTACT_FZ_PLAN_SLEW_BW`、`CONTACT_FZ_PLAN_WEIGHT`、`DS_FZ_FLOOR_WEIGHT`、`DS_FZ_FLOOR_BW`、`CONTACT_ADMIT_SCHEDULED_OFF_STAIRS`、`STAIR_CONTINUOUS_CONTACT`、`TOUCHDOWN_RAMP`、`SWING_CONTACT_SOFT`、`SWING_CONTACT_SOFT_AFTER`、`SWING_CONTACT_ON`、`SWING_CONTACT_CAP`、`STAIR_CONTACT_ACTIVATION_TIME`、`STAIR_CONTACT_MIN`、`LOAD_RAMP_TD`、`LOAD_RAMP_CONTACT_T`
- 所見: 接触集合 `stance_sides` は段移行・管理接触・柔らかい遊脚接触の各規則が順に上書きする（5835–6270）。採用分だけ残すと規則は 3 つ（実測、確認済み admit、荷重 admit）に減る。

### 2.13 着地の確認と待ち（昇り・平地） → `LandingConfig`

- 採用: `COM_LAND_EXTEND_RISE=1.08`
- 採用（kyo46rs 基準）: `COM_LAND_PREDICT_FRAC=1`
- 既定で有効: `LAND_CONFIRM_FZ_FRAC`、`COM_LAND_EXTEND`、`STEP_DZ_MIN`
- 箇所: 宣言 2633–2640、2978–3010。実装: 4482–4525（毎 tick `landing_timing` の許容・延長を設定し直す）、4640–4700（確認と admit）、5136、8772、8848（段の遊脚の判定 `STEP_DZ_MIN`）
- モジュール: preview（`LandingTiming`）、gait
- 落とす（18）: `LAND_BY_LOAD`、`LAND_WAIT_HOLD`、`CONTACT_EVENT`、`CONTACT_EVENT_Z`、`CONTACT_PROBE_VZ`、`CONTACT_PROBE_MAX`、`CONTACT_EVENT_SHIFT`、`CONTACT_EVENT_END_ON_TOUCH`、`LAND_BY_LOAD_FRAC`、`LAND_BY_LOAD_TICKS`、`LAND_CONFIRM_XY_RISE`、`LAND_CONFIRM_LEVEL`、`LAND_CONFIRM_LEVEL_ALWAYS`、`PHASE_BY_CONTACT`、`PHASE_EXTEND`、`PHASE_MIN_FRAC`、`PHASE_LOAD`、`PHASE_TICKS`
- 所見: `LandingTiming` の既定の許容（xy 12 mm・z 10 mm）はライブラリ `preview.rs:587-588` が環境変数 `LAND_CONFIRM_XY/Z` から読んでいる。切り出しでは Config の値にする。

### 2.14 CoM 予見と高さの計画 → `ComPlanConfig`

- 採用: `COM_REPLAN=1`、`COM_LAND_VXY=0.0776444`、`COM_LAND_VZ=0.05`
- 採用（kyo46rs 基準）: `COM_STAIR_SS_MAX=0.7254`、`COM_PREPLAN_XY_MIN=0.075`
- 既定で有効: `Z_COM`、`COM_PREPLAN`、`STAIR_CLIMB_RATE`、`COM_REPLAN_FORWARD_GAIN`、`COM_REPLAN_DT`、`COM_PREVIEW_DT`、`COM_PREVIEW_N`、`COM_PREVIEW_SMOOTH`、`COM_HEIGHT_REPLAN`、`COM_HEIGHT_TIME`
- 箇所: 宣言 1449、1956–1970、2315、2610–2625。実装: 1956–2025（遊脚時間の事前確保 `COM_PREPLAN`）、4625–4645 と 5092–5128（着地での再計画）、7599–7800（有界 ZMP の LIPM 予見の再計画）、8059–8222（高さ参照）
- モジュール: preview（`HeightPlan`、CoM 予見）、height、dcm
- 落とす（7）: `Z_COM_OFFSET`、`STAIR_CROUCH`、`COM_REPLAN_LATERAL`、`COM_REPLAN_LATERAL_GAIN`、`COM_REPLAN_ALWAYS`、`COM_PREVIEW_PHASE_LOCK`、`COM_HEIGHT_DS_FRAC`
- 所見: `COM_PREPLAN` の既定は `flag("COM_REPLAN")`。G1 は `COM_REPLAN=1` なので on。

### 2.15 contact_plan（片脚支持の終わりの予測的な決定） → `ContactPlanConfig`

- 採用: `CONTACT_PLAN=1`、`CP_MAX_X=0`、`CP_MAX_Y=0`、`CP_MAX_Y_IN=0`、`CP_T_FROM_X=1`、`CP_T_LAT_OUT=1`、`CP_TRACK=1`
- 既定で有効: `CP_T_MIN`、`CP_T_MAX`、`CP_DT`、`CP_W_T`、`CP_W_RES`、`CP_DEAD_T`、`CP_TRIG`
- 箇所: 宣言 2278–2305。実装: 7366–7566（作動・解・計画への反映）、4505–4520（時間調整した歩の確認と延長）
- モジュール: contact_plan、dcm、gait
- 落とす（8）: `CP_J`、`CP_W_SHIFT`†、`CP_W_U`、`CP_DEAD_U`、`CP_J_AUTO`、`CP_TOUCHDOWN`、`CP_SHIFT_NEXT`、`CP_TREAD_BOX`
- 所見: チャンピオンは `CP_MAX_X=CP_MAX_Y=CP_MAX_Y_IN=0` で**時間調整だけ**。ずらしの側（`CP_W_SHIFT`、`CP_W_U`、`CP_J`、`CP_DEAD_U`）は効かないので、Config は時間の側だけでよい。

### 2.16 両脚支持の作り直し（DS_FEASIBLE・DS_CAPTURE） → `DoubleSupportConfig`

- 採用: `DS_FEASIBLE_ON_TIMEOUT=1`、`DS_FEASIBLE_CONTINUOUS=1`、`DS_FEASIBLE_RAMP=1`、`DS_FEASIBLE=1`、`DS_CAPTURE=1`、`DS_CAPTURE_MARGIN=0.005`
- 既定で有効: `DS_CAPTURE_HOPELESS`、`DS_CAPTURE_MIN_GAIN`、`DS_CAPTURE_FWD_ONLY`、`DS_CAPTURE_T_MIN`、`DS_CAPTURE_LOAD`、`DS_FEASIBLE_MAX`、`DS_FEASIBLE_BOX`
- 箇所: 宣言 2683–2704、2866–2969。実装: 4700–4860（DS_FEASIBLE: 両脚支持の ZMP を解き直し、実測 DCM から始める）、4960–5045（DS_CAPTURE: 早期終了）
- モジュール: dcm（`DcmPlan` の区間の書き換え）、gait（区間の時間）
- 落とす（4）: `DS_CAPTURE_LAT`、`LIFT_GATE_FRAC`、`LIFT_GATE_MAX`、`DS_FEASIBLE_PLAN`
- 所見: 同じ場所に不採用の `RESTEP`（4856–4960）と `LIFT_GATE`（5046–5085）が挟まっている。

### 2.17 重心のレンチ MPC 本体 → `WrenchMpcConfig`

- 採用: `STAIR_WRENCH_MPC=1`、`WRENCH_MPC_ALWAYS=1`、`WRENCH_MPC_APPLY=1`、`MPC_NODE_CENTRE=1`、`MPC_NET_COP_W=10`、`MPC_SEED_PREVIOUS=2`、`MPC_W_SMOOTH=10`、`MPC_SOLE_AUTHORITY=1`、`STAIR_WRENCH_COUPLED=0`
- 既定で有効: `MPC_W_POS`、`MPC_W_VEL`、`MPC_W_MOM`、`MPC_W_ACC`、`MPC_W_WRENCH`、`STAIR_WRENCH_DT`、`STAIR_WRENCH_N`、`STAIR_WRENCH_PREVIEW_DT`、`STAIR_WRENCH_LOW_MIN`、`STAIR_WRENCH_BLEND`、`STAIR_HYBRID_CONTACT`、`MPC_QP_BACKEND`、`MPC_KKT_COMP_TOL`
- 箇所: 宣言 3263–3333、3725–3790、3849–3880、4323–4337、9246（ループ内）。実装: 8262–8640（荷重の予定と接触の箱）、8960–9300（入力・設定・解）、9967–10000（足ごとのレンチを力の名目へ）、9673–9682（連成段の門）
- モジュール: wrench_mpc（misa-mpc の `Input`、`Config`、`solve`）、gait
- 落とす（38）: `MPC_F_MAX_SCALE`、`LOAD_SHARE_RAMP`、`MPC_TARGET_PREVIEW`、`MPC_LOAD_SCHEDULE`、`MPC_COP_W`、`MPC_COP_REF_CMD`、`MPC_COP_REF_CMD_MAX`、`MPC_ARM_ABOUT_PRIOR`、`MPC_W_LDOT`、`MPC_PCMD`、`MPC_PCMD_FB`、`MPC_PCMD_FROM_ACC`、`MPC_FZ_SPLIT_EQ`、`MPC_DCM_W`、`MPC_NET_COP_EQ`、`MPC_REACH`、`MPC_REACH_W`、`MPC_REACH_MAX_DROP`、`MPC_REACH_MIN_LOAD`、`MPC_REACH_AHEAD`、`MPC_REACH_SAGITTAL`、`MPC_REACH_TRACK`、`MPC_REACH_SLEW`、`MPC_AUTHORITY_ALL`、`STAIR_WRENCH_MAX_AGE`、`STAIR_WRENCH_ACCEL`、`STAIR_WRENCH_ACCEL_XY`、`MPC_XI_REF`、`MPC_XI_REF_LEAD`、`MPC_XI_REF_TRANSFER_PLAN`、`STAIR_ANKLE_REG`、`STAIR_WRENCH_LEVEL_WEIGHT`、`STAIR_WRENCH_COM_WEIGHT`、`STAIR_WRENCH_MOM_WEIGHT`、`STAIR_WRENCH_TRUNK_WEIGHT`、`STAIR_WRENCH_TRUNK_COUPLED`、`STAIR_WRENCH_EVENT_REPLAN`、`MPC_QP_WARM`
- 所見: 主スイッチが 3 つ（`STAIR_WRENCH_MPC`、`WRENCH_MPC_ALWAYS`、`WRENCH_MPC_APPLY`）に割れているのは段移行の中だけで使っていた時代の名残。採用形は「常に解いて常に渡す」なので 1 つにまとめられる。

### 2.18 MPC の着地変数 → `MpcFootstepConfig`

- 採用: `MPC_LANDING_CAPTURE_W=1000000`、`MPC_LANDING_LEX=1`、`MPC_FOOTSTEP=1`、`MPC_LANDING_CAPTURE_J=1`、`MPC_FOOTSTEP_MAX_Y_IN=0`、`MPC_FOOTSTEP_Y_IN_FWD=0.035`、`MPC_FOOTSTEP_TRIG=0.015`、`MPC_FOOTSTEP_MAX_X=0.15`、`MPC_FOOTSTEP_MAX_Y=0.08`
- 既定で有効: `MPC_FOOTSTEP_MOMENT`、`MPC_FOOTSTEP_W`、`MPC_FOOTSTEP_Y_IN_FWD_TRIG`
- 箇所: 宣言 3336–3717。実装: 8640–8960（着地の箱、武装、内側の制限）、9128–9230（捕捉行の設定）、9370–9500（解の適用: `footsteps`／`gait`／`dcm_plan` を書き換え）
- モジュール: wrench_mpc（`Landing`）、gait、dcm
- 落とす（33）: `MPC_LANDING_CAPTURE_N`、`MPC_CAPTURE_COP_W`、`MPC_CAPTURE_LEX`、`MPC_LANDING_TIME`、`CP_LAT_EMERGENCY`、`CP_LAT_EMERGENCY_TMIN`、`MPC_LANDING_TIME_W`、`MPC_LANDING_TMIN_FRAC`、`MPC_CAPTURE_LEX_WP`、`MPC_CAPTURE_LEX_WS`、`MPC_CAPTURE_LEX_MARGIN`、`MPC_FOOTSTEP_FREEZE_FRAC`、`MPC_FOOTSTEP_INCREMENT`、`MPC_FOOTSTEP_STAIR_BACK`、`MPC_FOOTSTEP_TERRAIN_W`、`MPC_FOOTSTEP_CAPTURE_FLOOR`、`MPC_FOOTSTEP_TERRAIN_MARGIN`、`MPC_FOOTSTEP_MIN_WIDTH`、`MPC_FOOTSTEP_Y_IN_RETIMED`、`MPC_FOOTSTEP_SHIFT_REST`、`MPC_FOOTSTEP_HOLD_SHIFT`、`MPC_FOOTSTEP_BACK_ONLY`、`MPC_FOOTSTEP_BACK_PERSIST`、`MPC_FOOTSTEP_FLOOR_BACK_ERR`、`MPC_FOOTSTEP_BACK_VMAX`、`MPC_FOOTSTEP_FREEZE_FRAC_RISE`、`MPC_FOOTSTEP_TRIG_HOLD`、`MPC_LANDING_TIME_FROM_X`、`MPC_LANDING_TIME_GAIN`、`MPC_CAPTURE_LEX_NET_EQ`、`MPC_CAPTURE_LEX_TOTAL_EQ`、`MPC_CAPTURE_LEX_IGNORE_MIN`、`MPC_CAPTURE_LEX_FMARGIN`
- 所見: 33 個が不採用。採用は 9 個と既定 3 個。

### 2.19 MPC の交差の一歩 → `MpcCrossConfig`

- 採用: `MPC_FOOTSTEP_CROSS_Y=0.20`、`MPC_FOOTSTEP_CROSS_EX=0.03`、`MPC_FOOTSTEP_CROSS_FLOOR=0.5`、`MPC_FOOTSTEP_CROSS_HOLD_Z=0.05`
- 既定で有効: `MPC_FOOTSTEP_CROSS_HOLD_XY`
- 箇所: 宣言 3536–3565。実装: 8673–8700（交差の門とラッチ）、8733、8862–8900（LIPM の下限）、10404–10422（遊脚の高さの保持）
- モジュール: wrench_mpc、gait
- 落とす（1）: `MPC_FOOTSTEP_CROSS_AHEAD`
- 所見: `MPC_FOOTSTEP_CROSS_HOLD_XY` は `PUSH_REPLAN` の保持と共有。

### 2.20 押しの後の再計画 → `PushReplanConfig`

- 採用: `PUSH_REPLAN=0.03`、`PUSH_REPLAN_HOLD=10`、`PUSH_REPLAN_BACK_ONLY=1`、`PUSH_REPLAN_VMAX=0.15`、`PUSH_REPLAN_SLOPE_LEAD=0.6`
- 既定で有効: `PUSH_REPLAN_STAND`、`PUSH_REPLAN_RAMP`、`PUSH_REPLAN_FREEZE`、`PUSH_REPLAN_MAX_FWD`、`PUSH_REPLAN_MAX_BACK`、`PUSH_REPLAN_MIN_W`、`PUSH_REPLAN_EMIN`、`PUSH_REPLAN_MAX_W`、`PUSH_REPLAN_ANKLE`
- 箇所: 宣言 3611–3667、4062。実装: 6882–7010（誤差を着地へ運び、足跡と DCM 参照を引き直す）、6912–6917（斜面の手前）、10404–10407（遊脚の保持、HOLD=10 で実質無効）
- モジュール: gait（`FootstepPlan` の書き換え）、dcm、terrain
- 落とす（6）: `PUSH_REPLAN_CROSS`、`PUSH_REPLAN_AHEAD`、`PUSH_REPLAN_LAT`、`PUSH_REPLAN_LAT_VMAX`、`PUSH_REPLAN_LAT_VMIN`、`PUSH_REPLAN_HOLD_Z`
- 所見: `PUSH_REPLAN_HOLD=10` は「保持なし」と同じなので、Config では保持そのものを持たない形にできる（`PUSH_REPLAN_HOLD_Z` も落ちる）。

### 2.21 後ろ押しの検出に連動した重心下げ → `PushCrouchConfig`

- 採用: `PUSH_Z_DROP=0.035`
- 既定で有効: `PUSH_Z_DROP_TRIG`、`PUSH_Z_DROP_T`、`PUSH_Z_DROP_LAT_MAX`
- 箇所: 宣言 1674–1677。実装: 7013–7032（検出）、8224–8240（高さ参照へ）
- モジュール: preview（高さ参照）

### 2.22 速度の表と押しの譲歩 → `SpeedScheduleConfig`

- 採用: `VXS_Z_MIN=0.06`、`PUSH_YIELD_V=0.20`、`PUSH_YIELD_NO_FWD=1`、`PUSH_YIELD_VMIN=0.35`
- 既定で有効: `VX2_TSCALE_MIN`、`VXS_STRIDE_MAX`、`VXS_CAP_SLOPE`、`VXS_V_LONG`、`VXS_Z_SLOPE`、`VXS_Z_MAX`、`PUSH_YIELD_TRIG`
- 要確認: `VX2_SCHED`
- 箇所: 宣言 1512–1690。実装: 1512–1690（速度表: 歩幅上限・時間・重心下げ）、6829–6880（PUSH_YIELD: 押しを検出したら残りを遅い速度で引き直す）
- モジュール: gait（`FootstepPlan::velocity_with_change`）、dcm
- 落とす（6）: `VX2_STRIDE_MAX`、`VX2_REANCHOR`、`VXS_Z_EXTRA`、`VX2_DS_SCALE`、`VX2_Z_DROP`、`VXS_BOX_GAIN`
- 所見: 速度指令 `VX2`／`VX2_AT`／`VX2_RAMP` はシナリオ、表 `VXS_*` は制御。`VX2_SCHED=1` はベンチが入れているが、ライブラリでは速度指令の既定の経路にすべき（要確認）。

### 2.23 押し回復の参照ライブラリ → `RecoveryRefConfig`

- 採用: `REF_LIBRARY=log/claude_g1champion_20261029v619/ref_walk5_phase03.json,log/claude_g1champion_20261029v619/ref_walk5_phase05.json,log/claude_g1champion_20261029v619/ref_walk7_phase03.json`、`REF_RESET_INCL_CURRENT=1`、`REF_TRIG_DCM=0.036`、`REF_DELAY_COMP=0`、`REF_TRIG_MAX_FRAC=0.65`
- 既定で有効: `REF_SWING`、`REF_RESET_STANCE`、`REF_COOLDOWN`
- 箇所: 宣言 2791–2841、構造体 `RefLib` は 32–176（`main` の外）。実装: 7033–7180（検出と作動）、10425–10440（参照の遊脚）、11766–11830（終了・再センタリング）
- モジュール: gait、dcm（参照の DCM）
- 落とす（11）: `REF_TRIG_AT_PUSH`、`REF_RESET_LEAD`、`REF_TRIG_DCM_X`、`REF_TRIG_MIN_IMPULSE`、`REF_MPC`、`REF_WBC`、`REF_WBC_LANDED`、`REF_TRIG_PRED`、`REF_POST_HOLD`、`REF_SLOPE_GATE`、`REF_SLOPE_LEAD`
- 所見: 終了と再センタリング（11766–11830）が押しを発火させるシナリオの区画（11691–）の中にある。切り出しで分ける必要がある。

### 2.24 不整地モード（要確認） → `RoughModeConfig`

- 要確認: `ROUGH_MODE`、`ROUGH_MODE_LIFT_H`
- 箇所: 宣言 208–218（環境変数を書き換える）、2352–2385。実装: 10325–10375
- モジュール: gait（遊脚の形）
- 落とす（6）: `SWING_XY_DELAY`、`SWING_Z_HOLD`、`SWING_Z_HOLD_MIN_DUR`、`SWING_XY_END`、`SWING_Z_DROP`、`SWING_CAUTIOUS_ERR`
- 所見: `ROUGH_MODE=1` は他のノブを `set_var` で上書きする前置き。採用ではないが、未知の段差 30 mm を 48/48 にした運用モードとしてベンチで使っている。Config ではモード（列挙）にする。

† = 死に値（チャンピオンに値があるが効かない）。

### 2.25 まとまりに入らない落とす束

- 段の三相移行（`STAIR_TRANSFER`、Capture→Rise→Commit とその後始末 `STAIR_RESUME_*`／`STAIR_POST_WALK_*`、管理接触 `STAIR_MANAGED_*`、再接触 `STAIR_RECONTACT_*`）: 88 個（宣言 1229–3946）。
- その他の実験（`ADAPT_STEP`／`ADAPT_TIME`／`ADAPT_DS`、`RESTEP`、`PLAN_REANCHOR`、`DS_FFDC`、`LAND_BIAS_ADAPT`）: 37 個（宣言 1544–2911）。
- 段の三相移行の実装は 5129–5683（起動と状態機械）、5684–5834（再開）、6083–6200（接触の保持）、6410–6475（FFDC）、7820–7990（移行中の DCM 参照）、8360–8530（移行中の荷重予定）にあり、`stair_transfer_cmd.is_some()` の分岐として MPC・接触・遊脚・段の組み立ての各所に散っている。`_XFER`／`_POST` の 32 個もこの状態（`RunPhase`、4422–4445）に依存する。移行を落とすと `RunPhase` と `phase_gain` の仕組みごと消せる。

## 3. 全ノブの表

宣言行の順。「既定」はコードの既定の式（`prof.*` は機体の Profile、G1 では §1.2 の値）。「チャンピオン」は v6.19 の controls.json、「kyo」は kyo46rs 基準 v2（`kyo_base_v2.json`、v3 候補で変わるものは v3 の値を併記）。分類: 1 採用／1k kyo46rs 基準で採用／2 既定で有効／3 不採用・既定 off／3d 死に値／4 シナリオ／5 計測／6 機体／? 要確認。

| 名前 | 既定 | チャンピオン | kyo | 分類 | 行 | 説明 |
|---|---|---|---|---|---:|---|
| `ROUGH_MODE` | off |  |  | ? | 208 | 不整地の前置きモード（SWING_Z_HOLD/SWING_Z_DROP/SWING_XY_DELAY/LIFT_H を上書き）。要確認: 既知の不整地で操作者・ベンチ（rough30）が入れる。ライブラリでは「地形の知覚／モード指令」の入口に |
| `ROUGH_MODE_LIFT_H` | 0.08（斜面 0.06） |  |  | ? | 212 | ROUGH_MODE の頂点の高さ（既定 0.08、既知の斜面では 0.06）。要確認: ROUGH_MODE と同じ扱い |
| `TERRAIN_SLOPE` | 未設定 |  |  | 4 | 213（+1） | 既知の斜面の地形生成（角度,x0,x1）。ROUGH_MODE の分岐でも読む |
| `ROBOT` | kyo46rs | g1 |  | 6 | 274 | 機体プロファイルの選択（g1 / kyo46rs…） |
| `KNEE` | prof.knee_seed |  |  | 6 | 277 | 初期姿勢の膝角（prof.knee_seed） |
| `HIP_PITCH` | -o.knee / 2.0 |  |  | 6 | 278 | 初期姿勢の股ピッチ（既定 -KNEE/2） |
| `ANKLE_PITCH` | -o.knee / 2.0 |  |  | 6 | 279 | 初期姿勢の足首ピッチ（既定 -KNEE/2） |
| `JOINT_DAMPING` | prof.joint_damping |  |  | 6 | 280 | 関節粘性（植物と QP 模型の両方） |
| `DAMPING_OVERRIDES` | 未設定 | hip=2.75,knee=6.95,ankle=1.167 |  | 6 | 284 | 関節名ごとの粘性の上書き（設計入力。チャンピオンは hip/knee/ankle を指定） |
| `ARMATURE` | prof.armature | 0.00863034 |  | 6 | 295 | 関節のアーマチュア（チャンピオンは CEM 値） |
| `ARMATURE_OVERRIDES` | 未設定 |  |  | 6 | 298 | 関節名ごとのアーマチュアの上書き |
| `TORQUE_SCALE` | 1.0 |  |  | 6 | 312 | 関節トルク上限の倍率（URDF effort に掛ける） |
| `BURNIN_KP` | prof.burnin_kp |  |  | 6 | 313 | 開始前の静置 PD の kp |
| `BURNIN_KV` | prof.burnin_kv |  |  | 6 | 314 | 開始前の静置 PD の kv |
| `BURNIN_S` | prof.burnin_s |  |  | 6 | 315 | 静置の時間 |
| `RUN_KP` | prof.burnin_kp |  |  | 6 | 316 | 実行中の関節 PD の kp（位置制御モード用） |
| `RUN_KV` | prof.burnin_kv |  |  | 6 | 317 | 実行中の関節 PD の kv |
| `CTRL_MODE` | torque |  |  | 6 | 319 | リグの作動モード（torque など） |
| `SIM_DT` | 0.001 | 0.0005 | 0.0005 | 4 | 320 | MuJoCo の刻み（チャンピオン 0.5 ms） |
| `MU_GROUND` | 0.7 |  |  | 4 | 321 | 床の摩擦（rand.py が個体ごとに振る）。friction_mu の既定の元 |
| `MJ_NOSLIP` | 未設定 |  |  | 4 | 326 | MuJoCo の noslip 反復（ソルバの人工物対策） |
| `FOOT_SOLIMP` | 未設定 |  |  | 4 | 327 | 足裏接触の solimp |
| `PROBE_Z` | prof.probe_z |  |  | 6 | 347 | 初期化時の探索高さ |
| `NO_ARM_COLLIDE` | false |  |  | 6 | 353 | 腕を前額面から外す初期姿勢（前腕と股の干渉回避） |
| `HIP_ROLL_SEED` | 0.0 | 0.122728 |  | 1 | 360 | 股ロールの初期値で立ち幅を狭める（チャンピオンは CEM 値）。Profile の姿勢の種に寄せる候補 |
| `ARM_PITCH` | -1.40 | 0 | -0.4363 | 1 | 361 | 腕の肩ピッチの種（既定 -1.40、チャンピオン 0 = 下ろした腕） |
| `ELBOW` | 0.0 | 0 | -2.24 | 6 | 362 | 肘の種（チャンピオンも既定 0 を明示） |
| `SHOULDER_ROLL` | 0.0 |  |  | 6 | 379 | 肩ロールの種（外転） |
| `INIT_JITTER` | 0.0 |  |  | 4 | 407 | 初期脚姿勢の振り幅（rand.py が個体ごとに入れる） |
| `INIT_SEED` | 1.0 |  |  | 4 | 415 | 初期ジッタの種 |
| `URDF` | 未設定 | log/claude_g1arms_20260925/g1_23dof_a… | log/5_step_c026s009_jerk100_repeat_as… | 6 | 449 | 模型ファイル |
| `ROOT_LINK` | 未設定 |  |  | 6 | 458 | 根のリンク名 |
| `TRUNK_LINK` | 未設定 |  |  | 6 | 461 | 姿勢を保つ胴のリンク名 |
| `LEG_REACH` | prof.leg_reach |  |  | 6 | 469 | 脚の届く長さ（prof.leg_reach） |
| `LEG_REACH_MARGIN` | 0.95 |  |  | 2 | 470 | 届く長さの余裕 0.95。重心の高さ上限（reach）と降段の高さで使う |
| `SOLE_HALF_L` | prof.cop_half.0 | 0.085 |  | 6 | 472 | 足裏の半長（チャンピオン 0.085 = G1 の prof と同値） |
| `SOLE_HALF_W` | prof.cop_half.1 |  | 0.030 | 6 | 473 | 足裏の半幅（kyo46rs 基準は 0.030 を指定） |
| `SPAR_WALL` | 0.0 |  |  | 4 | 482 | 打撃用の壁の位置 |
| `SPAR_WALL_Z` | 0.52 |  |  | 4 | 490 | 壁の高さ |
| `STAIR_LANDING_LEN` | 0.60 | 3.0 |  | 4 | 508 | 最後の踏面の先の踊り場の長さ（チャンピオン 3.0、打ち切り対策） |
| `STAIRS` | 未設定 |  |  | 4 | 515 | 階段の生成（段数,蹴上げ,踏面,開始 x）。G1 の全ベンチは段差 0 の「階段」を含むので段の計画系は常に生きる |
| `STAIRS_DOWN` | 0.0 |  |  | 4 | 523 | 往復コースの降り段数 |
| `STAIR_HALF_W` | 0.30 |  |  | 4 | 528（+1） | 段の半幅 |
| `TERRAIN_DEBUG` | 未設定 |  |  | 5 | 562 | 地形のデバッグ出力 |
| `SPAWN_Z_EXTRA` | 0.0 |  |  | 4 | 565（+1） | 出現高さの追加（降段・斜面ベンチで 0.005） |
| `TERRAIN_BUMPS` | 未設定 |  |  | 4 | 665 | 計画に無い段差の生成（未知の不整地ベンチ） |
| `TERRAIN_SEED` | 1.0 |  |  | 4 | 675 | 段差の乱数の種 |
| `F_MAX_SCALE` | 2.3 |  |  | 2 | 711 | 足 1 枚の法線力上限 = 倍率 × mg（2.3） |
| `F_MAX` | f_max_scale * total_mass * G |  |  | 2 | 712 | 足 1 枚の法線力上限 [N]（既定は F_MAX_SCALE から） |
| `MPC_F_MAX_SCALE` | NaN |  |  | 3 | 722 | MPC 側だけ別の力上限（NaN = WBC と同じ） |
| `QP_DIAG_DIR` | 未設定 |  |  | 5 | 738 | WBC の QP を保存する置き場 |
| `QP_CAPTURE_ALL` | false |  |  | 5 | 741 | QP の連続保存 |
| `QP_CAPTURE_START` | 0.0 |  |  | 5 | 742 | QP 保存の開始時刻 |
| `QP_CAPTURE_END` | ∞ |  |  | 5 | 743 | QP 保存の終了時刻 |
| `QP_CAPTURE_STRIDE` | 1.0 |  |  | 5 | 744 | QP 保存の間引き |
| `QP_CAPTURE_MAX` | 20000.0 |  |  | 5 | 745 | QP 保存の上限数 |
| `QP_BENCH_GATE` | false |  |  | 5 | 748 | 別解法の独立な受け入れ検査（ベンチ用） |
| `WBC_QP_BACKEND` | （既定の解法） |  |  | 2 | 750（+1） | WBC の QP 解法の選択（既定の解法が働く。他は実験） |
| `MPC_BENCH_GATE` | false |  |  | 5 | 757 | MPC 解の独立な受け入れ検査（ベンチ用） |
| `WBC_PREFIX_MIN_LEVEL` | 999.0 |  |  | 3 | 765 | 下位の精緻化だけ失敗したときに検査済みの前段解を使う（999 = 無効） |
| `WBC_REPAIR_ORIGINAL` | false |  |  | 3 | 766 | 失敗した元の解の修復（実験） |
| `QP_RUNTIME_CSV` | 未設定 |  |  | 5 | 773 | WBC の解法時間の CSV |
| `MPC_RUNTIME_CSV` | 未設定 |  |  | 5 | 780 | MPC の解法時間の CSV |
| `MOM_DIAG_CSV` | 未設定 |  |  | 5 | 788 | 角運動量の内訳の CSV |
| `QP_WRENCH_CSV` | 未設定 |  |  | 5 | 795 | 6 軸レンチの CSV |
| `PROX_WEIGHT` | 0.0 |  |  | 3 | 809 | 前回解への近接項（0 = 無効） |
| `WBC_QP_RESET_EVERY_TICK` | false |  |  | 3 | 810 | 毎 tick の解法状態の初期化 |
| `WBC_FEAS_TOL` | SolveConfig::default().feasibility_tol | 1e-8 | 1e-8 | 1 | 811 | WBC の可行性許容（1e-8） |
| `WBC_HARD_PHYSICAL` | false |  |  | 3 | 820 | 物理制約を全段で硬くする（実験） |
| `WBC_STRATEGY` | null_space | weighted |  | 1 | 826（+1） | 優先度の解き方 = weighted（level 0 硬、以下を重み付き 1 本の QP） |
| `WBC_WEIGHTED_OFF_STAIRS` | false | 0 |  | 3 | 827 | 段ゾーンだけ厳密カスケードに戻す（チャンピオン v6 で 0 を明示 = 不採用） |
| `WBC_WEIGHT_RATIO` | 1.0 |  |  | 2 | 834 | weighted の段ごとの重み比（既定 1 = WBCW_* だけで決める） |
| `JLIM` | false | 1 | 1 | 1 | 859 | 関節の位置・速度の安全制限（CBF）を入れる |
| `JLIM_ALPHA1` | 10.0 | 25.6107 |  | 1 | 860 | CBF の係数 α1 |
| `JLIM_ALPHA2` | 10.0 |  |  | 2 | 861 | CBF の係数 α2（既定 10） |
| `JLIM_ALPHA3` | 10.0 | 20.6833 |  | 1 | 862 | CBF の係数 α3 |
| `JLIM_AMAX` | 50.0 | 351.019 | 200 | 1 | 863 | CBF の加速度上限（段階別 phase_gain） |
| `ARM_PITCH_BAND` | 0.0 |  |  | 3 | 871 | 肩ピッチの診断用の動作範囲（0 = 無効） |
| `ARM_PITCH_VMAX` | 0.0 |  |  | 3 | 872 | 同、速度 |
| `ARM_PITCH_AMAX` | 0.0 |  |  | 3 | 873 | 同、加速度 |
| `JLIM_Q_MARGIN` | 0.0 |  |  | 3 | 898 | 機構限界の内側で CBF を効かせる余裕（0 = 無効、文書で不採用） |
| `JLIM_V_FRAC` | 1.0 |  |  | 2 | 903 | 速度制限を URDF 値の何割にするか（1.0） |
| `friction_mu` | mu_ground * prof.friction_margin |  |  | 2 | 976 | 制御側の摩擦係数（既定 MU_GROUND × prof.friction_margin）。小文字の名前のまま。要確認ではないが改名候補 |
| `KP_COM` | 300.0 | 419.064 |  | 1 | 977 | 重心タスクの kp（CEM 値） |
| `KD_COM` | 80.0 | 48.1952 |  | 1 | 978 | 重心タスクの kd（CEM 値） |
| `KP_TRUNK` | 200.0 | 73.0103 |  | 1 | 979 | 胴の姿勢の kp（phase_gain） |
| `KD_TRUNK` | 40.0 | 28.2069 |  | 1 | 980 | 胴の姿勢の kd（phase_gain） |
| `KP_POST` | 100.0 |  |  | 2 | 981 | 姿勢タスクの kp |
| `KD_POST` | 20.0 |  |  | 2 | 982 | 姿勢タスクの kd |
| `POST` | true |  |  | 2 | 983 | 姿勢タスクを入れる（既定 true） |
| `ARM_HOLD` | false | 1 | 1 | 1 | 1000 | 腕の関節を姿勢に保持する段を入れる |
| `ARM_BEFORE_SWING` | false | 1 |  | 1 | 1003 | 腕の保持を遊脚より上の優先度に |
| `KP_ARM` | 400.0 | 400 | 200 | 2 | 1004 | 腕保持の kp（チャンピオンも 400 を明示、kyo46rs 基準は 200） |
| `MOM_MERGE` | 0.0 |  |  | 3 | 1020 | 角運動量タスクを上位段に重み付きで併合（0 = 無効） |
| `ARM_MERGE` | 0.0 | 0 |  | 3 | 1021 | 腕保持を上位段に併合（チャンピオンが 0 を明示 = 不採用、MPC 権限と組むと即死） |
| `KD_ARM` | 40.0 | 40 | 20 | 2 | 1022 | 腕保持の kd（チャンピオンも 40 を明示） |
| `ELBOW_KD` | 0.0 |  |  | 3 | 1024 | 肘だけの減衰（0 = 無効） |
| `ARM_HOLD_JOINTS` | all |  | shoulder | 1k | 1035 | 保持する腕の関節（既定 all が G1 で働く。kyo46rs 基準は shoulder） |
| `JAB` | false |  |  | 4 | 1041 | 腕を突き出す動作の課題（スパー） |
| `JAB_PERIOD` | 0.55 |  |  | 4 | 1042 | 突きの周期 |
| `JAB_START` | 2.5 |  |  | 4 | 1043 | 突きの開始時刻 |
| `JAB_PITCH` | -1.3963 |  |  | 4 | 1044 | 突きの肩ピッチ |
| `JAB_ELBOW` | -0.1745 |  |  | 4 | 1045 | 突きの肘 |
| `JAB_PATTERN` | LLRR |  |  | 4 | 1046 | 突きの左右の順 |
| `MOM` | false | 1 |  | 1 | 1051 | 重心の角運動量タスクを入れる |
| `KP_MOM` | 10.0 | 77.0372 |  | 1 | 1052 | 角運動量タスクの kp |
| `MOM_AXES` | "x--" |  |  | 2 | 1054 | 角運動量タスクの軸（既定 "x--" = ロールのみ） |
| `MOM_REF_MPC` | false |  |  | 3 | 1084 | 角運動量の参照を MPC の予測から取る |
| `MOM_LEVEL` | swing_after |  |  | 2 | 1095 | 角運動量タスクの段の位置（既定 swing_after） |
| `MOM_REF` | zero | arms_neutral |  | 1 | 1101 | 角運動量の参照の作り方 = arms_neutral（腕の寄与だけをゼロに） |
| `MOM_ARM_DCM` | 0.0 | 27.4033 |  | 1 | 1125 | 横の DCM 誤差から腕のロール角運動量率を要求する利得 |
| `MOM_ARM_DCM_MAX` | 20.0 |  |  | 2 | 1129 | その要求の上限（20） |
| `MOM_ARM_DCM_X` | 0.0 |  |  | 3 | 1134 | 前後版（ピッチ角運動量、0 = 無効） |
| `MOM_ARM_DCM_X_MAX` | 20.0 |  |  | 3 | 1135 | 前後版の上限（親が無効なので到達しない） |
| `MOM_CMP` | 0.0 | 1 |  | 1 | 1147 | ZMP の飽和分を全身の角運動量率で補う（ヒップ戦略の CMP 版） |
| `MOM_CMP_MAX` | 20.0 | 20 |  | 2 | 1148 | 要求の上限（チャンピオンも 20 を明示） |
| `MOM_CMP_DEAD` | 0.003 |  |  | 2 | 1149 | 不足分の不感帯 3 mm |
| `MOM_CMP_W` | 1.0 |  |  | 2 | 1150 | CoM 段での重み |
| `MOM_CMP_REF_ONLY` | false |  |  | 3 | 1152 | 参照の作動中だけに限る |
| `MOM_CMP_SIGN` | 1.0 | -1 |  | 1 | 1157 | 符号 -1（反対へ傾ける質量移動が効く） |
| `MOM_CMP_HMAX` | 0.0 | 1 |  | 1 | 1158 | 溜める角運動量の上限 [N m s] |
| `MOM_CMP_KH` | 0.0 | 2 |  | 1 | 1159 | 上限を超えた分を戻す P 利得 |
| `MOM_CMP_STAIRS` | false |  |  | 3 | 1166 | 段ゾーンでも有効にする（既定は段ゾーンで無効 = 採用形） |
| `TRUNK` | true |  |  | 2 | 1169 | 胴の姿勢タスクを入れる（既定 true） |
| `TRUNK_SIGN` | 1.0 |  |  | 2 | 1170 | 胴タスクの符号（1） |
| `TRUNK_DEAD` | 0.0 |  |  | 3 | 1171 | 胴の不感帯（0 = 無効） |
| `TRUNK_LATE` | false |  |  | 3 | 1172 | 胴タスクを後段に置く |
| `EOM_HARD` | true |  |  | 2 | 1173 | 運動方程式を硬い制約に（既定 true） |
| `COP_FRAC` | 1.0 | 0.912835 |  | 1 | 1174 | CoP 箱を足裏の何割にするか（phase_gain） |
| `HOLD_KP` | prof.hold_kp |  |  | 6 | 1176 | 劣化時の保持指令の kp（prof） |
| `HOLD_KD` | prof.hold_kd |  |  | 6 | 1177 | 劣化時の保持指令の kd（prof） |
| `HOLD_BRIDGE` | 8.0 |  |  | 2 | 1178 | 劣化解の橋渡し tick 数（CommandPolicy） |
| `FALLBACK_LEVEL` | 999.0 | 3 | 3 | 1 | 1179 | この段までの劣化解を適用する（3） |
| `HOLD_LAST` | false |  |  | 3 | 1180 | 劣化時に最後の指令を保持 |
| `BLEND_TICKS` | 0.0 |  |  | 3 | 1181 | 指令の混合 tick 数（0 = 無効） |
| `ANCHOR_LEAK` | 0.2 |  |  | 2 | 1182 | 接触アンカーの漏れ（0.2） |
| `ANCHOR_LEAK_ROT` | 0.0 |  |  | 3 | 1183 | 回転のアンカー漏れ（0 = 無効） |
| `KP_CONTACT` | 150.0 |  |  | 2 | 1198 | 接触タスクの kp（150） |
| `KD_CONTACT` | 2.0 * kp_c.sqrt() |  |  | 2 | 1202 | 接触タスクの kd（既定 2√kp） |
| `NO_PATCH` | false |  |  | 3 | 1203 | 接触パッチ（CoP 箱）を外す |
| `KP_SWING` | prof.kp_swing | 477.044 |  | 1 | 1213 | 遊脚 z の kp（phase_gain、既定 prof） |
| `KP_SWING_XY` | 0.25 * kp_sw_z_g.base |  | 400 | 1k | 1214 | 遊脚 xy の kp（phase_gain、既定 0.25×z。kyo46rs 基準は 400） |
| `KD_SWING` | prof.kd_swing | 31.493 |  | 1 | 1215 | 遊脚の kd（phase_gain、既定 prof） |
| `LIFT_H` | 0.5 * prof.lift_h | 0.0512879 | 0.03 | 1 | 1225 | 遊脚の頂点の高さ（phase_gain、CEM 値 0.051） |
| `STAIR_FIRST_POST_SWING_FRACTION` | 1.0 |  |  | 3 | 1229 | 段移行後の最初の平地遊脚を短くする（移行 off で到達しない） |
| `K_DCM` | 2.0 | 3.09069 | 2 | 1 | 1237 | DCM 追従の利得（無次元） |
| `ZMP_MARGIN` | 0.8 |  |  | 2 | 1241 | 指令 ZMP を支持多角形の何割に収めるか（0.8） |
| `CLAMP_ZMP` | true |  |  | 2 | 1242 | 指令 ZMP の切り詰め（既定 true） |
| `T` | 25.0 | 40 | 28 | 4 | 1244 | 実行時間（チャンピオン 40） |
| `T_START` | 2.0 |  |  | 4 | 1246 | 歩き出しまでの静置時間 |
| `T_DS` | 0.20 | 0.365543 | 0.3384 | 1 | 1247 | 両脚支持の時間（CEM 値） |
| `T_SS` | 0.45 | 0.843964 | 0.45 | 1 | 1248 | 片脚支持の時間（CEM 値） |
| `N_STEPS` | 20.0 | 28 | 40 | 4 | 1249 | 計画する歩数（チャンピオン 28） |
| `FIRST_SWING` | 1.0 |  |  | 2 | 1250 | 最初に振る足 |
| `NO_LIFT` | false |  |  | 5 | 1256 | 立ち上げ段階 W1（足を上げない） |
| `PUSH_IMPULSE` | 0.0 |  |  | 4 | 1266 | 押しの力積 |
| `PUSH_DEG` | 90.0 |  |  | 4 | 1270 | 押しの向き |
| `PUSH_YAW_IMPULSE` | 0.0 |  |  | 4 | 1273 | ヨーの力積 |
| `PUSH_DT` | 0.10 |  |  | 4 | 1274 | 押しの継続時間 |
| `PUSH_STEP` | 5.0 |  |  | 4 | 1280 | 押す歩 |
| `PUSH_AT` | 0.5 |  |  | 4 | 1281 | 押す位相 |
| `PUSH_SUPPORT` | ss |  |  | 4 | 1283 | 押す支持の種類（ss/ds/any） |
| `PUSH_RECOVER_MM` | 20.0 |  |  | 5 | 1287 | 回復判定の DCM 誤差の閾値（集計用） |
| `PUSH_RECOVER_S` | 0.5 |  |  | 5 | 1291 | 回復判定の持続時間 |
| `PUSH_WINDOW_S` | 0.20 |  |  | 5 | 1305 | 押し後の計測窓 |
| `PUSH_EDGE_FRAC` | 0.9 |  |  | 5 | 1321 | CoP が縁に張り付いたとみなす割合（計測） |
| `PUSH_TRANSFER_S` | 0.10 |  |  | 5 | 1322 | 計測の移行窓 |
| `PUSH_LINK` | root |  |  | 4 | 1323 | 押すリンク |
| `CTRL_DT` | prof.ctrl_dt | 0.005 | 0.005 | 1 | 1325 | 制御周期（チャンピオン 5 ms、G1 の prof は 2 ms）。Profile/Config のどちらに置くか要検討 |
| `DIAG_VZ_BIAS_MPC` | 0.0 |  |  | 5 | 1329 | MPC にだけ見せる鉛直速度の偏り（診断） |
| `DIAG_VZ_BIAS_HEIGHT` | 0.0 |  |  | 5 | 1330 | 高さ PD にだけ見せる偏り（診断） |
| `SUBSTEP_LIMIT_AUDIT_JSON` | 未設定 |  |  | 5 | 1337 | 物理刻みでの制限の監査 |
| `KP_SWING_ROT` | 400.0 |  |  | 2 | 1380 | 遊脚の回転の kp（phase_gain） |
| `KD_SWING_ROT` | 40.0 |  |  | 2 | 1381 | 遊脚の回転の kd（phase_gain） |
| `LAND_KD_ROT_BOOST` | 1.0 |  |  | 3 | 1383 | 段の遊脚の下降中の回転 kd の倍率（1 = 無効、文書で不採用） |
| `STAIR_ABSOLUTE_TD` | false |  | 0 | 3 | 1385 | 測った踏面の高さで着地目標を作る（kyo46rs 基準も 0） |
| `STAIR_ABSOLUTE_TD_DESC` | false |  | → v3 1 | ? | 1394 | 同、降段だけ。要確認: kyo46rs v3 候補（判断待ち）で 1 |
| `STAIR_SWING_FF` | false |  | 1 → v3 0 | ? | 1395 | 段の遊脚の前置き（旧）。要確認: kyo46rs 基準 v2 は 1、v3 候補で 0 に外す |
| `TRACKING_CUBIC_FF` | false | 1 | 0 | 1 | 1397 | 三次の遊脚軌道に解析的な加速度の前置きを足す |
| `TRACKING_FF_MIN_RISER` | 0.0 | 0.10 |  | 1 | 1406 | 前置きを蹴上げ 10 cm 以上の階段だけに |
| `TRACKING_FF_SCALE` | 1.0 |  | 1 | 2 | 1408 | 前置きの倍率（1） |
| `STAIR_SWING_PROFILE` | false |  |  | 3 | 1411 | 平地でも段の遊脚の時間割を使う（分離実験） |
| `SWING_ROT` | 1.0 |  | 1 | 2 | 1412 | 遊脚の回転の追従軸のモード（phase_gain、既定 1） |
| `STAIR_ZONE_SWING_ROT` | 0.0 |  | → v3 3 | ? | 1421 | 段ゾーンだけ SWING_ROT のモードを変える。要確認: kyo46rs v3 候補で 3 |
| `TRUNK_YAW` | true |  |  | 2 | 1423 | 胴のヨーも制御（既定 true） |
| `KP_YAW` | 200.0 |  |  | 2 | 1424 | 胴ヨーの kp |
| `KD_YAW` | 40.0 |  |  | 2 | 1425 | 胴ヨーの kd |
| `DS_SETTLE` | 0.0 | 0.0840506 |  | 1 | 1432 | 両脚支持の後に足す保持時間（CEM 値） |
| `DS_SETTLE_AFTER_STEP` | 1.0 |  |  | 2 | 1433 | 保持を入れ始める歩 |
| `Z_COM` | com0.z + env_f64("Z_COM_OFFSET", 0.0) |  |  | 2 | 1449 | 歩行の重心高さ（絶対値、既定は初期の実測 + Z_COM_OFFSET） |
| `Z_COM_OFFSET` | 0.0 |  |  | 3 | 1449 | 歩行の重心高さを初期の実測からずらす（0 = 無効、kyo46rs の実験で使用） |
| `GAIN_OMEGA_REF` | 0.0 |  |  | 3 | 1473 | 利得を ω で正規化する基準（0 = 無効） |
| `ZMP_LAT_SCALE` | 1.0 |  |  | 2 | 1491 | ZMP の横移動の割合（phase_gain、1 = 真の片脚支持） |
| `STRIDE` | 0.0 | 0.148842 |  | 1 | 1498 | 一歩の長さ（CEM 値）。公称速度の指令 |
| `VX` | stride / t_step |  | 0.03 | 4 | 1499 | 前進速度の指令（既定 STRIDE/t_step、kyo46rs 基準は 0.03） |
| `VY` | 0.0 |  |  | 4 | 1500 | 横速度の指令 |
| `WZ` | 0.0 |  |  | 4 | 1501 | 旋回の指令 |
| `SQUAT_AMP` | 0.0 |  |  | 4 | 1504 | 歩きながら屈伸する課題の振幅 |
| `SQUAT_PERIOD` | 3.0 |  |  | 4 | 1505 | 屈伸の周期 |
| `STRIDE_RAMP` | 6.0 | 3.15826 | 6 | 1 | 1507 | 歩幅を立ち上げる歩数（CEM 値） |
| `VX2` | 未設定（一定速度） |  |  | 4 | 1512 | 速度変更の目標（ベンチ） |
| `VX2_AT` | 8.0 |  |  | 4 | 1513 | 速度変更の開始歩 |
| `VX2_RAMP` | 3.0 |  |  | 4 | 1513 | 速度変更の立ち上げ歩数 |
| `VX2_STRIDE_MAX` | 0.0 |  |  | 3 | 1523 | 手で決める歩幅上限（VX2_SCHED に置き換え済み） |
| `VX2_TSCALE_MIN` | 0.4 |  |  | 2 | 1524 | 速度の時間スケールの下限（速度表と PUSH_YIELD が使う） |
| `VX2_SCHED` | false |  |  | ? | 1535 | 速度表（歩幅上限・時間・重心下げを一つの表で）。要確認: ベンチが速い歩行で入れる。ライブラリでは速度指令の既定経路にすべき |
| `VX2_REANCHOR` | false |  |  | 3 | 1543 | 片脚支持の入りで計画を実足に付け直す |
| `PLAN_REANCHOR` | false |  |  | 3 | 1544 | 計画の付け直し（文書で不採用） |
| `DS_FFDC` | 0.0 |  |  | 3 | 1545 | 上りの両脚支持の荷重ループ（不採用） |
| `DS_FFDC_CLAMP` | 0.02 |  |  | 3 | 1547 | 同、切り詰め |
| `PLAN_REANCHOR_MAX` | 0.05 |  |  | 3 | 1548 | 付け直しの上限 |
| `PLAN_REANCHOR_TD` | 0.0 |  |  | 3 | 1559 | 着地で計画を実着地へ移す |
| `LAND_BIAS_ADAPT` | 0.0 |  |  | 3 | 1569 | 着地誤差の EMA で前補償（段ゾーン） |
| `LAND_BIAS_ADAPT_TAU` | 3.0 |  |  | 3 | 1570 | 同、時定数 |
| `LAND_BIAS_ADAPT_MAX` | 0.04 |  |  | 3 | 1571 | 同、上限 |
| `VXS_STRIDE_MAX` | 0.45 |  |  | 2 | 1583（+1） | 速度表: 歩幅上限 |
| `VXS_CAP_SLOPE` | 2.0 |  |  | 2 | 1584（+1） | 速度表: 上限の速度に対する傾き |
| `VXS_V_LONG` | 0.45 |  |  | 2 | 1584（+1） | 速度表: 長い歩幅に切り替える速度 |
| `VXS_Z_SLOPE` | 0.36 |  |  | 2 | 1587 | 速度表: 重心下げの傾き |
| `VXS_Z_MAX` | 0.08 |  |  | 2 | 1591 | 速度表: 重心下げの上限 |
| `VXS_Z_MIN` | 0.0 | 0.06 |  | 1 | 1598 | 速度指令中の重心下げの下限 0.06（押し×速度 0.30 で 77→133） |
| `VXS_Z_EXTRA` | 0.0 |  |  | 3 | 1602 | 表に上乗せする重心下げ（0 = 無効） |
| `VX2_DS_SCALE` | 1.0 |  |  | 3 | 1615 | 速めた歩の両脚支持の倍率（1 = 無効） |
| `PUSH_YIELD_V` | 0.0 | 0.20 |  | 1 | 1622 | 速度指令中に押しを検出したら残りをこの速度で引き直す |
| `PUSH_YIELD_TRIG` | 0.03 |  |  | 2 | 1623 | 譲歩の検出閾値（DCM 誤差 3 cm） |
| `PUSH_YIELD_NO_FWD` | false | 1 |  | 1 | 1624 | 前向きの誤差では譲歩しない |
| `PUSH_YIELD_VMIN` | 0.0 | 0.35 |  | 1 | 1625 | 譲歩はこの速度以上の指令のときだけ |
| `VX2_Z_DROP` | 0.0 |  |  | 3 | 1672 | 速度変更の前から重心を下げる（VX2_SCHED に置き換え済み） |
| `PUSH_Z_DROP` | 0.0 | 0.035 |  | 1 | 1674 | 後ろ押しの検出に連動して重心を下げる量 |
| `PUSH_Z_DROP_TRIG` | 0.03 |  |  | 2 | 1675 | 検出の閾値 |
| `PUSH_Z_DROP_T` | 1.5 |  |  | 2 | 1676 | 下げを保つ時間 |
| `PUSH_Z_DROP_LAT_MAX` | 0.02 |  |  | 2 | 1677 | 横誤差がこれ以上なら下げない |
| `ALTERNATE` | true |  |  | 2 | 1690 | 交互の歩容（0 = 寄せ足、A/B 用） |
| `STAIR_TOE_MARGIN` | 0.020 | 0 |  | 1 | 1719 | 蹴込みとつま先の余裕（チャンピオン 0、v4.1） |
| `STAIR_Y_OUT` | 0.010 | 0.00442314 |  | 1 | 1725 | 段の上の足跡を外へ広げる量 |
| `STAIR_GAIT` | （交互） |  | pair | 1k | 1726 | 段の歩き方（pair = 一段ずつ揃える）。kyo46rs 基準で pair |
| `STAIR_TD_BIAS` | 0.0 | 0.000780737 | 0 | 1 | 1737 | 踏面の着地を後ろへずらす量（CEM 値、ほぼ 0） |
| `STAIR_TD_BIAS_ASC_MID` | td_bias | 0.02 |  | 1 | 1755 | 段鼻のある昇りの踏面だけ後退 2 cm（v6.4） |
| `STAIR_TD_BIAS_DESC` | td_bias | 0.02 |  | 1 | 1757 | 降りの着地の後退 2 cm（v6.3） |
| `STAIR_FULL_SOLE` | false |  | 1 | 1k | 1761 | 段の上で足裏全体を踏面に収める。kyo46rs 基準で 1 |
| `STAIR_SOLE_MARGIN` | 0.002 |  | 0.002 | 2 | 1762 | 足裏を踏面に収める余裕 2 mm（STAIR_FULL_SOLE=1 の kyo46rs 経路で働く） |
| `STAIR_LANDING_FRAC` | 0.5 | 0.942684 |  | 1 | 1780 | 踊り場へ出る一歩の踏面に対する割合（CEM 値） |
| `SLOPE_KEEP_STRIDE` | false | 1 |  | 1 | 1781 | 既知の斜面で足跡を段の格子に並べ直さず歩幅を保つ（v6.16） |
| `STAIR_ALIGN_STAND_LINE` | false | 1 | → v3 1 | 1 | 1842 | 段の手前で立ち線に揃える |
| `STAIR_DESC_STAND_MARGIN` | 0.03 |  |  | 2 | 1856 | 降りの手前の立ち線の余裕 3 cm |
| `COM_PREPLAN` | flag("COM_REPLAN", false) |  | 1 | 2 | 1959 | 遊脚の時間を DCM 生成の前に確保（既定は COM_REPLAN と同じ = on。kyo46rs 基準も明示） |
| `COM_REPLAN` | false | 1 | 1 | 1 | 1959（+1） | 有界 ZMP の LIPM 予見で CoM 参照を引き直す |
| `COM_LAND_VXY` | 0.35 | 0.0776444 | 0.05 | 1 | 1960（+1） | 予見の着地時の水平速度上限 |
| `COM_LAND_VZ` | 0.20 | 0.05 | 0.05 | 1 | 1961（+1） | 予見の着地時の鉛直速度上限 |
| `COM_STAIR_SS_MAX` | 0.80 |  | 0.7254 | 1k | 1962 | 段の片脚支持の上限時間（kyo46rs 基準 0.7254） |
| `COM_DESC_DS` | 0.0 |  |  | 3 | 1965 | 降りの前の両脚支持を延ばす（0 = 無効） |
| `COM_DESC_XFER` | 0.0 |  |  | 3 | 1969 | 降りの後の両脚支持を延ばす（0 = 無効） |
| `COM_PREPLAN_XY_MIN` | ∞ |  | 0.075 | 1k | 1970 | 長い同高の遊脚にも時間を確保する閾値（kyo46rs 基準 0.075） |
| `STAIR_CROUCH` | 0.0 |  | 0 | 3 | 2026（+1） | LIPM 高さに段のしゃがみを含める量（0 = 無効、kyo46rs 基準も 0 を明示） |
| `ZMP_ROLL` | if stairs.is_some() { 0.030 } else { 0.0 } |  | 0.02107 | 1k | 2036 | 片脚支持中の圧力の踵→つま先移動（階段がある場面の既定 0.030 が G1 で働く。G1 ベンチは常に段がある）。kyo46rs 基準は 0.021 |
| `DCM_PREVIEW_HORIZON` | 0.0 |  |  | 3 | 2039 | DCM 計画を有限の先読みにする（0 = 全計画） |
| `DCM_REF_BLEND_T` | 0.0 |  |  | 3 | 2061 | DCM 参照の跳びを時間で混ぜる（0 = 無効） |
| `DCM_REF_BLEND_MIN` | 0.02 |  |  | 3 | 2062 | 同、混ぜる最小の跳び |
| `TRAJ_CSV` | 未設定 |  |  | 5 | 2233 | 軌跡の CSV |
| `TICK_AUDIT_CSV` | 未設定 |  |  | 5 | 2237 | tick 監査の CSV |
| `CONTACT_DRIVEN` | true |  |  | 2 | 2251 | 足が報告する接触集合で解く（既定 true） |
| `CONTACT_TICKS` | 2.0 |  |  | 2 | 2253 | 接触の履歴の tick 数 |
| `CONTACT_ON_BW` | 0.10 |  | 0.05 | 1k | 2254 | 接触入りの荷重閾値（既定 0.10 BW、kyo46rs 基準 0.05） |
| `CONTACT_SCHEDULED_REENTRY` | false |  |  | 3 | 2260 | 予定どおりの再入り |
| `ADAPT_STEP` | false | 0 |  | 3 | 2268 | ZMP 飽和からの捕捉点の足跡修正（チャンピオンが 0 を明示、contact_plan に置き換え） |
| `ADAPT_MAX_X` | 0.06 | 0.12614 |  | 3d | 2271 | ADAPT_STEP の x 上限。チャンピオンに 0.126 があるが ADAPT_STEP=0 かつ CP_MAX_X を明示しているので到達しない（死に値） |
| `ADAPT_MAX_Y` | 0.03 | 0.0319943 |  | 3d | 2272 | ADAPT_STEP の y 上限。同上（死に値） |
| `CONTACT_PLAN` | false | 1 |  | 1 | 2278 | contact_plan（着地のずらしと片脚支持の終わりの予測的な決定） |
| `CP_T_MIN` | 0.5 |  |  | 2 | 2280 | 片脚支持の終わりの下限（割合） |
| `CP_T_MAX` | 1.0 |  |  | 2 | 2285 | 同、上限（1.0 = 延ばさない） |
| `CP_DT` | 0.01 |  |  | 2 | 2286 | 予測の刻み |
| `CP_MAX_X` | adapt_max_x | 0 |  | 1 | 2287 | 着地ずらしの x 上限（チャンピオン 0 = 時間調整のみ） |
| `CP_MAX_Y` | adapt_max_y | 0 |  | 1 | 2288（+1） | 同、y（0） |
| `CP_MAX_Y_IN` | 未設定 | 0 |  | 1 | 2289 | 同、内側（0） |
| `CP_T_FROM_X` | false | 1 |  | 1 | 2290 | 時間を前後の誤差から決める |
| `CP_T_LAT_OUT` | false | 1 |  | 1 | 2291 | 横は外向きの誤差だけで時間を決める |
| `CP_J` | 0.5 |  |  | 3 | 2292 | 着地ずらしに対する計画の感度（ずらしが 0 なので効かない） |
| `CP_W_SHIFT` | 0.0 | 40 |  | 3d | 2293 | ずらしの重み。チャンピオンに 40 があるが CP_MAX_X=CP_MAX_Y=CP_MAX_Y_IN=0 でずらしが常に 0 なので効かない（死に値） |
| `CP_W_U` | 1.0 |  |  | 3 | 2294 | ずらしの費用の重み（ずらしが 0 に固定されているので効かない） |
| `CP_W_T` | 1.0 |  |  | 2 | 2295 | 時間の重み |
| `CP_W_RES` | 100.0 |  |  | 2 | 2296 | 残差の重み |
| `CP_DEAD_U` | 0.002 |  |  | 3 | 2297 | ずらしの不感帯（同上） |
| `CP_DEAD_T` | 0.005 |  |  | 2 | 2298 | 時間の不感帯 |
| `CP_TRIG` | 0.015 |  |  | 2 | 2302 | DCM 誤差の作動閾値 1.5 cm |
| `CP_TRACK` | false | 1 |  | 1 | 2303 | 決めた時間を追跡する |
| `CP_J_AUTO` | false |  |  | 3 | 2304 | 感度を計画の差分から自動で |
| `CP_TOUCHDOWN` | false |  |  | 3 | 2305 | 着地時にも計画 |
| `STAIR_CLIMB_RATE` | 1.5 |  |  | 2 | 2315 | 高さ参照が一歩あたり動ける蹴上げ数 |
| `STAIR_DESC_MIN` | false |  |  | 3 | 2318 | 降りで 2 つの足裏の低い方に乗る |
| `STAIR_TD_SINK` | 0.004 |  | 0.009 | 1k | 2336 | 段の遊脚の着地の押し込み（phase_gain、既定 4 mm、kyo46rs 基準 9 mm） |
| `STAIR_LAND_SINK` | 0.004 |  |  | 2 | 2337 | 段の着地の押し込み 4 mm |
| `STAIR_DESC_TOE` | 0.0 |  |  | 3 | 2343 | 降りでつま先から着く（0 = 無効） |
| `SWING_XY_DELAY` | 0.0 |  |  | 3 | 2352 | 遊脚の水平移動の開始を遅らせる（ROUGH_MODE が 0.15 にする） |
| `SWING_Z_HOLD` | false |  |  | 3 | 2353 | 上げ切るまで水平に進まない（ROUGH_MODE が 1 にする） |
| `SWING_Z_HOLD_MIN_DUR` | 0.0 |  |  | 3 | 2354 | 同、最短時間 |
| `SWING_XY_END` | 1.0 |  |  | 3 | 2355 | 遊脚の水平移動の終わり（1 = 既定の形） |
| `SWING_Z_DROP` | 0.60,0.95 |  |  | 3 | 2357 | 下降の開始・終了（ROUGH_MODE が 0.50,1.0）。SWING_Z_HOLD のときだけ効く |
| `SWING_CAUTIOUS_ERR` | 0,0 |  |  | 3 | 2364 | 慎重な遊脚（文書で不採用） |
| `SWING_TOE_UP` | 0.0 |  |  | 3 | 2370 | 遊脚中つま先を上げる（0 = 無効、不採用） |
| `SWING_TOE_UP_W` | 0.15,0.75 |  |  | 3 | 2372 | 同、窓 |
| `STAIR_ANCHOR_FLAT` | false |  |  | 3 | 2392 | 着地アンカーを計画の回転で（つま先着地用） |
| `STAIR_ROLL_PATCH` | false |  |  | 3 | 2395 | 足裏が傾いている間 CoP 箱を縁へ縮める |
| `SLOPE_HEEL_CUT` | 0.0 | 0.010 |  | 1 | 2396 | 斜面で踵側の CoP 箱を削る（+5° 0→46） |
| `SLOPE_HEEL_CUT_MAX` | 0.05 |  |  | 2 | 2397 | 削りの上限 |
| `CONTACT_FZ_RATE` | 0.0 |  |  | 3 | 2418 | 法線力の変化率の減衰（反証済みと明記） |
| `CONTACT_FZ_PLAN_SLEW_BW` | 0.0 |  |  | 3 | 2423 | WBC の fz 目標を MPC の値へ滑らかに |
| `CONTACT_FZ_PLAN_WEIGHT` | 0.0 |  |  | 3 | 2424 | 同、重み |
| `DS_FZ_FLOOR_WEIGHT` | 0.0 |  |  | 3 | 2427 | 両脚支持の荷重の柔らかい下限 |
| `DS_FZ_FLOOR_BW` | 0.12 |  |  | 3 | 2428 | 同、下限の割合 |
| `STAIR_SWING_WINDOWS` | 0.30,0.25,0.70,0.60 |  | 0.50,0.38,0.80,0.70 | 1k | 2430 | 段の遊脚の時間窓（上げ終わり・移動の始終・下げ始め）。kyo46rs 基準 0.50,0.38,0.80,0.70 |
| `CP_SHIFT_NEXT` | false |  |  | 3 | 2453 | 次の足跡もずらす |
| `CP_TREAD_BOX` | false |  |  | 3 | 2454（+1） | 踏面の箱で制限 |
| `ADAPT_LIVE` | true |  |  | 3 | 2462 | ADAPT_STEP の連続更新（既定 true だが CONTACT_PLAN=1 で到達しない） |
| `ADAPT_TIME` | false |  |  | 3 | 2471 | DCM 閉形式で片脚支持の終わりを決める（contact_plan に置き換え） |
| `ADAPT_TIME_AXIS` | 1.0 |  |  | 3 | 2476 | 同、優先する軸 |
| `ADAPT_TIME_MIN` | 0.6 |  |  | 3 | 2482 | 同、下限 |
| `ADAPT_TIME_MAX` | 1.4 |  |  | 3 | 2483 | 同、上限 |
| `ADAPT_TIME_DEAD` | 0.005 |  |  | 3 | 2485 | 同、不感帯 |
| `ADAPT_TIME_TRIG` | 0.015 |  |  | 3 | 2494 | 同、作動閾値 |
| `ADAPT_TIME_CUT` | false |  |  | 3 | 2502 | 到達不能で歩を切る反射 |
| `ADAPT_TIME_WIDE` | false |  |  | 3 | 2518 | 着地を外へ広げる |
| `ADAPT_TIME_WIDEN` | 0.02 |  |  | 3 | 2519 | 同、量 |
| `ADAPT_DS` | false |  |  | 3 | 2541 | 両脚支持の長さの調整 |
| `ADAPT_DS_TRIG` | 0.015 |  |  | 3 | 2545 | 同、閾値 |
| `ADAPT_DS_MIN` | 0.25 |  |  | 3 | 2549 | 同、下限 |
| `ADAPT_DS_MAX` | 2.0 |  |  | 3 | 2550 | 同、上限 |
| `FIX_DV` | 0.0 |  |  | 5 | 2566 | 修正の力積を入れる探査（原因の切り分け用） |
| `FIX_DEG` | 90.0 |  |  | 5 | 2567 | 同、向き |
| `FIX_DT` | 0.02 |  |  | 5 | 2568 | 同、時間 |
| `FIX_TAU` | 0.0 |  |  | 5 | 2575 | 角の修正の探査 |
| `FIX_TAU_AXIS` | 0.0 |  |  | 5 | 2576 | 同、軸 |
| `FIX_AT_S` | -1.0 |  |  | 5 | 2584 | 同、時刻 |
| `COM_REPLAN_LATERAL` | false |  |  | 3 | 2611 | 予見の再計画を横にも |
| `COM_REPLAN_FORWARD_GAIN` | 1.0 |  |  | 2 | 2612 | 再計画の前後の利得（1） |
| `COM_REPLAN_LATERAL_GAIN` | 1.0 |  |  | 3 | 2614 | 横の利得（LATERAL off で到達しない） |
| `COM_REPLAN_ALWAYS` | false |  |  | 3 | 2616 | 毎回再計画 |
| `COM_REPLAN_DT` | 0.02 |  |  | 2 | 2617 | 再計画の周期 20 ms |
| `COM_PREVIEW_PHASE_LOCK` | false |  |  | 3 | 2620 | 離散時計で更新率を保つ |
| `COM_PREVIEW_DT` | 0.05 |  |  | 2 | 2621 | 予見の刻み 50 ms |
| `COM_PREVIEW_N` | 20.0 |  |  | 2 | 2622 | 予見の点数 20 |
| `COM_PREVIEW_SMOOTH` | 0.1 |  |  | 2 | 2623 | 予見の平滑の重み |
| `COM_HEIGHT_REPLAN` | true |  |  | 2 | 2624 | 高さの計画の再計画（既定 true） |
| `COM_HEIGHT_TIME` | 0.60 |  |  | 2 | 2625 | 高さの移行時間 0.60 s |
| `LAND_CONFIRM_FZ_FRAC` | 0.05 |  |  | 2 | 2633 | 着地確認の力の閾値 5% BW |
| `LAND_BY_LOAD` | false |  |  | 3 | 2640 | 実荷重で着地を確認（P2-2） |
| `CONTACT_ADMIT_CONFIRMED` | false | 1 |  | 1 | 2646 | 確認済みの着地を荷重履歴を待たずに接触集合へ |
| `CONTACT_ADMIT_SCHEDULED` | false | 1 | 1 | 1 | 2647 | 予定どおりの着地を接触集合へ（kyo46rs 基準でも 1） |
| `CONTACT_ADMIT_SCHEDULED_OFF_STAIRS` | false |  |  | 3 | 2648 | 段の外だけに限る |
| `COM_HEIGHT_DS_FRAC` | 未設定 |  |  | 3 | 2650 | 両脚支持の中で高さを移す割合（未設定 = 無効） |
| `CONTACT_ADMIT_Z` | 0.010 |  |  | 2 | 2658 | 時間切れの着地も踏面から 1 cm 以内なら admit |
| `CONTACT_ADMIT_LOADED` | 未設定 | 0.04,0.02 |  | 1 | 2668 | 高さがずれていても荷重があれば admit（0.04,0.02、v6.10） |
| `GAIT_DUMP_T` | 未設定 |  |  | 5 | 2673 | 歩容の書き出し時刻 |
| `CONTACT_DUMP_T` | 未設定 |  |  | 5 | 2677 | 接触の書き出し時刻 |
| `DS_FEASIBLE_ON_TIMEOUT` | false | 1 |  | 1 | 2683 | 時間切れの着地でも両脚支持を作り直す |
| `DS_FEASIBLE_CONTINUOUS` | false | 1 |  | 1 | 2695 | 両脚支持の DCM 参照を実測の DCM から始める |
| `DS_FEASIBLE_RAMP` | false | 1 |  | 1 | 2704 | 両脚支持の ZMP を解いた点から次の足へ傾斜 |
| `LAND_WAIT_HOLD` | false |  |  | 3 | 2718 | 着地待ちの間 DCM を支持足に保つ |
| `REACH_HEIGHT` | 0.0 |  |  | 3 | 2734 | 届く範囲で重心高さに上限（0 = 無効） |
| `REACH_HEIGHT_FLOOR` | 0.45 |  |  | 3 | 2735 | 同、下限 |
| `COM_HEIGHT_DESC_LEAD` | 0.0 | 0.5 |  | 1 | 2750 | 降りの遊脚の間に重心を先に下げる（v6.2） |
| `COM_HEIGHT_DESC_REACH` | 0.0 |  |  | 3 | 2760 | 降りの着地が届く高さへの上限（不採用） |
| `STAIR_ZONE_CROUCH` | 0.0 | 0.04 | → v3 0.02 | 1 | 2761 | 段ゾーンでしゃがむ量 4 cm（kyo46rs v3 候補は 2 cm） |
| `STAIR_ZONE_CROUCH_LEAD` | 0.30 |  |  | 2 | 2762 | 段ゾーンの手前の長さ（MOM_CMP の段ゾーン判定も共有） |
| `STAIR_ZONE_CROUCH_TAIL` | 0.60 |  |  | 2 | 2763 | 段ゾーンの後ろの長さ |
| `WBC_WEIGHTED_ZONE_TAIL` | stair_crouch_tail |  |  | 3 | 2767 | WBC_WEIGHTED_OFF_STAIRS の範囲（親が無効） |
| `REF_LIBRARY` | 未設定 | log/claude_g1champion_20261029v619/re… |  | 1 | 2791 | 押し回復の参照（trajopt の JSON 3 本） |
| `REF_TRIG_AT_PUSH` | false |  |  | 3 | 2797 | ベンチの押しの瞬間に参照を入れる（神託、実機では使えない） |
| `REF_SWING` | true |  |  | 2 | 2798 | 参照の遊脚を使う（既定 true） |
| `REF_RESET_STANCE` | true |  |  | 2 | 2799 | 参照の終わりに支持足を再センタリング（既定 true） |
| `REF_RESET_INCL_CURRENT` | false | 1 |  | 1 | 2804 | 参照終了時に始まっている遊脚も再センタリング（v6.7） |
| `REF_RESET_LEAD` | 0.0 |  |  | 3 | 2805 | 再センタリングの先読み（不採用） |
| `REF_TRIG_DCM` | 0.0 | 0.036 |  | 1 | 2808 | 横の DCM 誤差の検出閾値 3.6 cm |
| `REF_TRIG_DCM_X` | 0.0 |  |  | 3 | 2811 | 前押しの検出 |
| `REF_TRIG_MIN_IMPULSE` | 4.5 |  |  | 3 | 2815 | ベンチ発火の最小力積（REF_TRIG_AT_PUSH 用） |
| `REF_COOLDOWN` | 1.0 |  |  | 2 | 2816 | 再発火までの待ち 1 s |
| `REF_DELAY_COMP` | true | 0 |  | 1 | 2819 | 検出遅れの補償（既定 true をチャンピオンが 0 に） |
| `REF_MPC` | false |  |  | 3 | 2823 | 参照から MPC の DCM・CoP 参照を作る（不採用） |
| `REF_WBC` | false |  |  | 3 | 2824 | 参照の DCM/CoP を WBC の DCM 則へ（不採用） |
| `REF_WBC_LANDED` | true |  |  | 3 | 2830 | 着地状態の参照（re-step）にだけ REF_WBC。既定 true だが RESTEP off で到達しない |
| `REF_TRIG_PRED` | 0.0 |  |  | 3 | 2836 | 予測による先読み検出（不採用） |
| `REF_TRIG_MAX_FRAC` | 1.0 | 0.65 |  | 1 | 2840 | 遊脚の位相 0.65 以降は参照を使わない |
| `REF_POST_HOLD` | 0.0 |  |  | 3 | 2841 | 参照の後の保持（不採用） |
| `CONTACT_EVENT` | false |  |  | 3 | 2844（+1） | 接触イベントで着地を決める（保留） |
| `CONTACT_EVENT_Z` | 0.003 |  |  | 3 | 2846 | 同、高さ許容 |
| `CONTACT_PROBE_VZ` | 0.05 |  |  | 3 | 2847 | 同、下へ探る速度 |
| `CONTACT_PROBE_MAX` | 0.03 |  |  | 3 | 2848 | 同、探る深さ |
| `CONTACT_EVENT_SHIFT` | true |  |  | 3 | 2849 | 同、計画を実足へ（既定 true だが CONTACT_EVENT off で到達しない） |
| `CONTACT_EVENT_END_ON_TOUCH` | true |  |  | 3 | 2852 | 同、触れたら遊脚を終える（既定 true、同上） |
| `LAND_BY_LOAD_FRAC` | 0.20 |  |  | 3 | 2853 | LAND_BY_LOAD の荷重閾値 |
| `LAND_BY_LOAD_TICKS` | 3.0 |  |  | 3 | 2854 | 同、tick 数 |
| `DS_FEASIBLE` | false | 1 |  | 1 | 2866 | 着地確認で片脚支持が切れたら両脚支持を DCM から解き直す（P2-3'） |
| `DS_CAPTURE` | false | 1 |  | 1 | 2879 | 両脚支持の圧力で DCM を戻せないとき両脚支持を早く終える（v6.8） |
| `RESTEP` | false |  |  | 3 | 2886 | 荷重のない着地足をもう一度振る（不採用） |
| `RESTEP_MARGIN` | 0.0 |  |  | 3 | 2887 | 同 |
| `RESTEP_LOAD_MAX` | 0.15 |  |  | 3 | 2888 | 同 |
| `RESTEP_T_SS` | 0.30 |  |  | 3 | 2889 | 同 |
| `RESTEP_T_DS` | 0.15 |  |  | 3 | 2890 | 同 |
| `RESTEP_X` | 0.05 |  |  | 3 | 2891 | 同 |
| `RESTEP_CROSS` | 0.03 |  |  | 3 | 2892 | 同 |
| `RESTEP_DEBUG` | 未設定 |  |  | 5 | 2893 | RESTEP のデバッグ出力 |
| `RESTEP_Y_IN` | 0.06 |  |  | 3 | 2898 | 同、内側の上限 |
| `RESTEP_T_SS2` | 0.35 |  |  | 3 | 2904 | 同 |
| `RESTEP_FOLLOW` | 1.0 |  |  | 3 | 2906 | 同 |
| `RESTEP_REF` | true |  |  | 3 | 2910 | 同、着地状態の参照で駆動（既定 true だが親が無効） |
| `RESTEP_Y_IN_STEPS` | 2.0 |  |  | 3 | 2911 | 同 |
| `DS_CAPTURE_HOPELESS` | 1.5 |  |  | 2 | 2916 | 「望みなし」の判定倍率 1.5 |
| `DS_CAPTURE_MIN_GAIN` | 0.05 |  |  | 2 | 2918 | 短縮の最小効果 50 ms |
| `DS_CAPTURE_FWD_ONLY` | true |  |  | 2 | 2924 | 前向きの逸脱だけ（既定 true） |
| `DS_CAPTURE_LAT` | false |  |  | 3 | 2934 | 横にも（不採用） |
| `DS_CAPTURE_MARGIN` | 0.01 | 0.005 |  | 1 | 2936 | 逸脱の余裕 5 mm |
| `DS_CAPTURE_T_MIN` | 0.15 |  |  | 2 | 2937 | 両脚支持の最短 0.15 s |
| `DS_CAPTURE_LOAD` | 0.4 |  |  | 2 | 2938 | 着地足の荷重 40% 以上で |
| `DS_CAPTURE_DEBUG` | 未設定 |  |  | 5 | 2939 | DS_CAPTURE のデバッグ出力 |
| `LIFT_GATE_FRAC` | 0.0 |  |  | 3 | 2951 | 上げる足の上に DCM がある間は両脚支持を延ばす（不採用） |
| `LIFT_GATE_MAX` | 0.4 |  |  | 3 | 2952 | 同、上限 |
| `LIFT_GATE_DEBUG` | 未設定 |  |  | 5 | 2953 | LIFT_GATE のデバッグ出力 |
| `DS_FEASIBLE_MAX` | 1.5 |  |  | 2 | 2960 | 作り直す両脚支持の上限 1.5 s |
| `DS_FEASIBLE_PLAN` | false |  |  | 3 | 2962 | 解いた支点を計画にも書く |
| `DS_FEASIBLE_BOX` | both |  |  | 2 | 2969 | 支点の箱の選択（既定 both = 両脚支持の箱。stance は実験） |
| `LOAD_SHARE_RAMP` | false |  |  | 3 | 2974 | MPC に両脚支持の荷重配分を時間関数で渡す（不採用） |
| `COM_LAND_PREDICT_FRAC` | 0.0 |  | 1 | 1k | 2978（+1） | 着地予測の割合（phase_gain、既定 0、kyo46rs 基準 1） |
| `COM_LAND_EXTEND` | 0.25 | 0.25 | 0.10 | 2 | 2983 | 着地の締め切りの延長 0.25 s（チャンピオンも既定値を明示。kyo46rs 基準は 0.10） |
| `COM_LAND_EXTEND_RISE` | 0.0 | 1.08 |  | 1 | 3001 | 昇りの着地の延長 1.08 s |
| `STEP_DZ_MIN` | 0.005 |  |  | 2 | 3009 | 段の遊脚とみなす高さ変化 5 mm（既知の斜面ベンチは 0.08/0.03 を入れる） |
| `STEP_DZ_SWING_MIN` | 1e-9 |  |  | 2 | 3010 | 段の遊脚の形を使う高さ変化（既定 1e-9、斜面ベンチは 0.08） |
| `TERRAIN_PITCH` | false |  |  | ? | 3017 | 既知の斜面で遊脚の向きを着地面へ傾け、ピッチを追従。要確認: チャンピオンに無く、斜面ベンチが入れる（地形の知覚の代替） |
| `TERRAIN_PITCH_YAW` | false | 1 |  | 1 | 3018 | その追従軸をヨー＋ピッチに（ロール自由、v6.18）。TERRAIN_PITCH=1 のときだけ効く |
| `COM_LAND_EXTEND_DESC` | land_extend_rise | 0.25 |  | 1 | 3026 | 降りの着地の延長 0.25 s（v6.2） |
| `LAND_CONFIRM_XY_DESC_CP` | false |  |  | 3 | 3033 | 降りの xy 許容を contact_plan が時間調整した歩だけに |
| `COM_LAND_EXTEND_DESC_CP` | 0.0 |  |  | 3 | 3042 | contact_plan が時間調整した降りの延長 |
| `LAND_CONFIRM_FZ_DESC_FRAC` | 未設定 |  |  | 3 | 3048 | 降りの確認の力閾値（未設定 = 無効） |
| `LAND_CONFIRM_XY_RISE` | 未設定 |  |  | 3 | 3057 | 昇りの確認の前後許容（不採用。TRACKING_CUBIC_FF とは組まない） |
| `LAND_CONFIRM_Z_DESC` | 未設定 | 0.003 |  | 1 | 3062 | 降りの確認の高さ許容 3 mm（v6.3） |
| `LAND_CONFIRM_XY_DESC` | 未設定 | 0.03 |  | 1 | 3063 | 降りの確認の xy 許容 3 cm（v6.3） |
| `LAND_CONFIRM_LEVEL` | false |  |  | 3 | 3076 | 段移行の後だけ平地の着地も確認 |
| `LAND_CONFIRM_LEVEL_ALWAYS` | false |  |  | 3 | 3077 | 平地の着地を常に確認（不採用） |
| `STAIR_TRANSFER` | false | 0 |  | 3 | 3081 | 接触で起動する段の三相移行（Capture→Rise→Commit）。チャンピオンが 0 を明示 = 不採用 |
| `STAIR_CAPTURE_TIME` | 0.80 | 0.703416 |  | 3d | 3082 | Capture の時間。チャンピオンに 0.703 があるが STAIR_TRANSFER=0 で到達しない（死に値、CEM の名残） |
| `STAIR_CAPTURE_FROM_XI` | false |  |  | 3 | 3083 | Capture を歩行の DCM 参照から始める |
| `STAIR_CAPTURE_LOAD_TRACK` | false |  |  | 3 | 3095 | Capture を実測荷重に追従 |
| `STAIR_CAPTURE_SHARE_TAU` | 0.08 |  |  | 3 | 3096 | 同、平滑の時定数 |
| `STAIR_CAPTURE_LEAD` | 0.0 |  |  | 3 | 3103 | 同、先行の上限 |
| `STAIR_CLOCK_GUARD_SLOW` | 0.0 |  |  | 3 | 3111 | 門が閉じている間の時計の進み |
| `STAIR_TRANSFER_MODE` | capture |  |  | 3 | 3127 | 移行の型（capture / rise_first） |
| `STAIR_RELEASE_SHARE` | 0.80 |  |  | 3 | 3128 | rise_first の低い足を離す荷重割合 |
| `STAIR_RELEASE_TAU` | 0.10 |  |  | 3 | 3133 | 同、平滑 |
| `STAIR_RELEASE_MIN_T` | 0.30 |  |  | 3 | 3134 | 同、最短時間 |
| `STAIR_RISE_TIME` | 1.50 |  |  | 3 | 3135 | Rise の時間 |
| `STAIR_CAPTURE_RELEASE_TIME` | 0.0 |  |  | 3 | 3137 | Rise 前の抜重時間 |
| `STAIR_CAPTURE_RELEASE_GUARD` | false |  |  | 3 | 3138 | 同、門 |
| `STAIR_COMMIT_TIME` | 0.40 |  |  | 3 | 3139 | Commit の時間 |
| `STAIR_POST_COMMIT_HOLD` | 0.0 |  |  | 3 | 3141 | Commit 後の保持（診断） |
| `STAIR_POST_WALK_SETTLE` | false |  |  | 3 | 3142 | 移行後の歩行の落ち着き |
| `STAIR_POST_WALK_TERMINAL_STEP` | false |  |  | 3 | 3143 | 同、終端の一歩 |
| `STAIR_POST_WALK_TERMINAL_GAP` | 0.065 |  |  | 3 | 3144 | 同 |
| `STAIR_POST_WALK_SETTLE_RAMP` | 1.00 |  |  | 3 | 3145 | 同 |
| `STAIR_POST_WALK_SETTLE_CAPTURE_HOLD` | 0.50 |  |  | 3 | 3147 | 同 |
| `STAIR_POST_WALK_SETTLE_STEPS` | 2.0 |  |  | 3 | 3149 | 同 |
| `STAIR_POST_WALK_SETTLE_LOAD` | 0.10 |  |  | 3 | 3150 | 同 |
| `STAIR_POST_WALK_SETTLE_DWELL` | 0.05 |  |  | 3 | 3151 | 同 |
| `STAIR_POST_WALK_SETTLE_FZ_WEIGHT` | 1.0 |  |  | 3 | 3153 | 同 |
| `STAIR_TRUNK_POST_TIME` | 0.0 |  |  | 3 | 3154 | 移行後の胴の連成の継続時間 |
| `STAIR_HEIGHT_HANDOVER` | false |  |  | 3 | 3155 | 移行から歩行への高さの受け渡し |
| `STAIR_CONSUME_RISE_STEP` | false |  |  | 3 | 3156 | Rise の歩を計画から消費 |
| `STAIR_POST_Z` | 0.0 |  |  | 3 | 3169 | 移行後のしゃがみ（不採用） |
| `STAIR_RESUME_CONTACT_GATE` | false |  |  | 3 | 3171 | 移行後の両脚支持の解放を荷重で門 |
| `STAIR_RESUME_SS_DCM_HOLD` | false |  |  | 3 | 3174 | 移行後の片脚支持で DCM を保持 |
| `STAIR_RESUME_SS_HOLD_MAX` | 1.0 |  |  | 3 | 3175 | 同 |
| `STAIR_RESUME_SS_HOLD_FRAC` | 0.65 |  |  | 3 | 3176 | 同 |
| `STAIR_RESUME_SS_SINK` | 0.002 |  |  | 3 | 3177 | 同 |
| `STAIR_RESUME_SS_LOAD` | 0.10 |  |  | 3 | 3178 | 同 |
| `STAIR_RESUME_SS_ADMIT_LOAD` | 0.015 |  |  | 3 | 3180 | 同 |
| `STAIR_RESUME_DS_CAPTURE` | false |  |  | 3 | 3182 | 移行後の DS を実測から橋渡し |
| `STAIR_RESUME_DS_CAPTURE_TIME` | 0.50 |  |  | 3 | 3183 | 同 |
| `STAIR_RESUME_DS_CAPTURE_AFTER` | 2.0 |  |  | 3 | 3185 | 同 |
| `STAIR_RESUME_GATE_LOAD` | 0.65 |  |  | 3 | 3186 | 再開の門（荷重） |
| `STAIR_RESUME_GATE_TILT` | 0.08 |  |  | 3 | 3187 | 再開の門（傾き） |
| `STAIR_RESUME_GATE_DWELL` | 0.05 |  |  | 3 | 3188 | 再開の門（滞在） |
| `STAIR_RESUME_SOLE_POSE` | false |  |  | 3 | 3189 | 再開時の足裏姿勢 |
| `STAIR_RESUME_SWING_PRIORITY` | false |  |  | 3 | 3190 | 再開時の遊脚の優先度 |
| `STAIR_RESUME_SWING_COUPLE` | 0.0 |  |  | 3 | 3191 | 再開時の遊脚の連成 |
| `STAIR_CAPTURE_INBOARD` | 0.012 |  |  | 3 | 3204 | Capture 目標の内寄せ |
| `STAIR_CAPTURE_MARGIN` | 0.70 |  |  | 3 | 3205 | Capture の箱の割合 |
| `STAIR_CAPTURE_TOL` | 0.003 |  |  | 3 | 3206 | Capture の許容 |
| `STAIR_CAPTURE_LOW_CAP` | 1.0 |  |  | 3 | 3216 | Capture の終わりの低い足の容量 |
| `STAIR_FFDC` | 0.0 |  |  | 3 | 3232 | 移行中の荷重のフィードバック（不採用） |
| `STAIR_FFDC_CLAMP` | 0.010 |  |  | 3 | 3233 | 同 |
| `STAIR_FFDC_LEAK` | 4.0 |  |  | 3 | 3236 | 同 |
| `STAIR_CAPTURE_K` | 1.0 |  |  | 3 | 3237 | Capture の DCM 利得 |
| `STAIR_TRANSFER_LATCH` | false |  |  | 3 | 3238 | 移行中は両足を接触に固定 |
| `STAIR_TRANSFER_TIMEOUT` | 4.0 |  |  | 3 | 3239 | 移行の時間切れ |
| `STAIR_TRANSFER_DCM` | 0.025 | 0.0270955 |  | 3d | 3241 | 移行の DCM 許容。チャンピオンに 0.0271 があるが到達しない（死に値） |
| `STAIR_TRANSFER_Z` | 0.015 |  |  | 3 | 3242 | 移行の高さ許容 |
| `STAIR_RISE_LOAD` | 0.45 | 0.363328 |  | 3d | 3243 | Rise の荷重閾値。チャンピオンに 0.363 があるが到達しない（死に値） |
| `STAIR_COMMIT_LOAD` | 0.80 | 0.850319 |  | 3d | 3244 | Commit の荷重閾値。チャンピオンに 0.850 があるが到達しない（死に値） |
| `STAIR_TRANSFER_DRIFT` | 0.0 |  |  | 3 | 3247 | 移行で運ぶ DCM の漂い |
| `STAIR_TRANSFER_HOLD` | 0.0 | 1.16593 |  | 3d | 3249 | 移行の起動時の両脚支持の延長。チャンピオンに 1.166 があるが到達しない（死に値） |
| `STAIR_TRANSFER_KEEP_DCM` | false | 1 |  | 3d | 3251 | 移行中も歩行の DCM 計画を保つ。チャンピオンに 1 があるが到達しない（死に値） |
| `STAIR_TRANSFER_TILT` | 0.12 |  |  | 3 | 3252 | 移行の傾きの門 |
| `STAIR_TRANSFER_FZ_GUARD` | 1.60 |  |  | 3 | 3253 | 移行の力の門 |
| `STAIR_TRANSFER_TICKS` | 4.0 |  |  | 3 | 3254 | 移行の門の tick 数 |
| `STAIR_TD_SOLE_FRAME` | false | 1 |  | 1 | 3259 | 段の遊脚を足首でなく足裏中心の姿勢で指令 |
| `STAIR_RISE_SOLE_POSE` | false | 0 |  | 3 | 3261 | 高さの変わる遊脚で足裏を水平に保つ（チャンピオンが 0 を明示） |
| `STAIR_RISE_SOLE_POSE_MIN_DZ` | 0.0 |  |  | 3 | 3262 | 同、最小の高さ変化 |
| `STAIR_WRENCH_MPC` | false | 1 |  | 1 | 3263 | 足ごとのレンチ MPC を入れる（名前は段用の名残、実際は常時の重心 MPC の主スイッチ） |
| `WRENCH_MPC_ALWAYS` | false | 1 |  | 1 | 3277 | 移行の外でも MPC を解く |
| `WRENCH_MPC_APPLY` | false | 1 |  | 1 | 3285 | MPC の解を移行の外でも WBC に渡す（複合ゲート: 加速度・力の名目・fz 計画） |
| `MPC_TARGET_PREVIEW` | false |  |  | 3 | 3286 | MPC の目標を予見から |
| `MPC_LOAD_SCHEDULE` | false |  |  | 3 | 3288 | MPC に WBC と同じ荷重の傾斜を見せる |
| `MPC_NODE_CENTRE` | false | 1 |  | 1 | 3290 | fz の予定を節点の中央で揃える |
| `MPC_COP_W` | 0.0 |  |  | 3 | 3295 | DCM 計画どおりの CoP の重み（0 = 無効） |
| `MPC_NET_COP_W` | 0.0 | 10 |  | 1 | 3296 | 正味 CoP 参照の重み 10 |
| `MPC_COP_REF_CMD` | false | 0 |  | 3 | 3310 | 正味 CoP 参照を WBC の指令 ZMP から（チャンピオンが 0 を明示 = 不採用） |
| `MPC_COP_REF_CMD_MAX` | ∞ |  |  | 3 | 3318 | 同、ずれの上限 |
| `MPC_SEED_PREVIOUS` | 0.0 | 2 |  | 1 | 3324 | 毎回の解を実測荷重に係留（2） |
| `MPC_ARM_ABOUT_PRIOR` | false |  |  | 3 | 3333 | 腕の角運動量を事前の周りで |
| `MPC_W_LDOT` | 0.0 |  |  | 3 | 3334 | 角運動量率の重み（0 = 無効） |
| `MPC_LANDING_CAPTURE_W` | 0.0 | 1000000 |  | 1 | 3336 | 着地による DCM 捕捉の重み 1e6（LEX では > 0 であることだけが意味を持つ） |
| `MPC_LANDING_CAPTURE_N` | 0.0 |  |  | 3 | 3339 | 捕捉行だけ長い地平（0 = MPC の地平） |
| `MPC_LANDING_LEX` | false | 1 |  | 1 | 3344 | 着地を辞書式（捕捉が先、箱の中で厳密）に決める（v5.2） |
| `MPC_CAPTURE_COP_W` | 0.0 |  |  | 3 | 3348 | MPC 自身の正味 CoP の捕捉行 |
| `MPC_CAPTURE_LEX` | false |  |  | 3 | 3354 | 辞書式の CoP |
| `MPC_LANDING_TIME` | false |  |  | 3 | 3361 | 着地時刻も MPC の決定変数に |
| `CP_LAT_EMERGENCY` | 0.0 |  |  | 3 | 3371 | 支持足側の押しで遊脚を早く下ろす（0 = 無効） |
| `CP_LAT_EMERGENCY_TMIN` | 0.3 |  |  | 3 | 3372 | 同、残り時間の割合 |
| `MPC_LANDING_TIME_W` | 1e-3 |  |  | 3 | 3375 | MPC_LANDING_TIME の重み |
| `MPC_LANDING_TMIN_FRAC` | 0.5 |  |  | 3 | 3376 | 同、最早の割合 |
| `MPC_CAPTURE_LEX_WP` | 0.1 |  |  | 3 | 3384 | MPC_CAPTURE_LEX の重み |
| `MPC_CAPTURE_LEX_WS` | 0.1 |  |  | 3 | 3385 | 同 |
| `MPC_CAPTURE_LEX_MARGIN` | 0.005 |  |  | 3 | 3386 | 同 |
| `MPC_PCMD` | 0.0 |  |  | 3 | 3390 | WBC の指令圧力点を MPC の節点 0 から（不採用） |
| `MPC_PCMD_FB` | false |  |  | 3 | 3402 | 同、計画＋速い帰還 |
| `DCM_LEAD_S` | 0.0 |  |  | 3 | 3404 | センサ遅れの DCM 先読み（0 = 無効） |
| `MPC_PCMD_FROM_ACC` | false |  |  | 3 | 3415 | MPC の加速度から圧力点 |
| `MPC_FZ_SPLIT_EQ` | false |  |  | 3 | 3418 | 両脚支持の鉛直配分を予定に固定 |
| `MPC_FOOTSTEP_MOMENT` | true |  |  | 2 | 3423 | 着地のずれをモーメント・正味 CoP の行にも入れる（既定 true） |
| `MPC_FOOTSTEP` | false | 1 |  | 1 | 3424 | 遊脚の着地を MPC の決定変数に（v5） |
| `MPC_FOOTSTEP_W` | 100.0 | 100 |  | 2 | 3442 | 着地のずれの重み 100（チャンピオンも明示） |
| `MPC_FOOTSTEP_FREEZE_FRAC` | ∞ |  |  | 3 | 3450 | この位相を過ぎたら着地を凍結（∞ = 無効） |
| `MPC_LANDING_CAPTURE_J` | false | 1 |  | 1 | 3456 | 捕捉行に計画の感度（2 cm ずらして作り直した差分）を与える |
| `MPC_FOOTSTEP_INCREMENT` | false |  |  | 3 | 3463 | MPC の答えを増分として扱う |
| `MPC_FOOTSTEP_STAIR_BACK` | ∞ |  |  | 3 | 3471 | 段で後ろへずらす上限（∞ = 無効） |
| `MPC_FOOTSTEP_TERRAIN_W` | 0.0 |  |  | 3 | 3476 | 地形をコストに |
| `MPC_FOOTSTEP_CAPTURE_FLOOR` | 0.0 |  |  | 3 | 3485 | 捕捉の下限（v6.13 の部品、v6.14 で PUSH_REPLAN に置き換え） |
| `MPC_FOOTSTEP_TERRAIN_MARGIN` | 0.01 |  |  | 3 | 3486 | 同、余裕 |
| `MPC_FOOTSTEP_MAX_Y_IN` | ∞ | 0 |  | 1 | 3493 | 支持足側への内側のずらしを禁止（0） |
| `MPC_FOOTSTEP_MIN_WIDTH` | 0.0 |  |  | 3 | 3500 | 内側ずらしを幅で制限 |
| `MPC_FOOTSTEP_Y_IN_RETIMED` | 0.0 |  |  | 3 | 3511 | 時間調整された歩の内側の上限 |
| `MPC_FOOTSTEP_Y_IN_FWD` | 0.0 | 0.035 |  | 1 | 3519 | 前向き誤差の間だけ内側 3.5 cm まで（v5.1） |
| `MPC_FOOTSTEP_CROSS_Y` | 0.0 | 0.20 |  | 1 | 3536 | DCM が支持足の外へ出たら内側を開いて交差の一歩（v6.12） |
| `MPC_FOOTSTEP_CROSS_EX` | 0.02 | 0.03 |  | 1 | 3537 | 交差を開く DCM のはみ出し 3 cm |
| `MPC_FOOTSTEP_CROSS_AHEAD` | 0.0 |  |  | 3 | 3538 | 交差中の前方の制限 |
| `MPC_FOOTSTEP_CROSS_FLOOR` | 0.0 | 0.5 |  | 1 | 3548 | 交差の LIPM 捕捉の下限（足裏半幅の 0.5） |
| `MPC_FOOTSTEP_SHIFT_REST` | false |  |  | 3 | 3556 | 着地を動かしたら残りの計画も（v6.13 の部品、PUSH_REPLAN に置き換え） |
| `MPC_FOOTSTEP_CROSS_HOLD_Z` | 0.0 | 0.05 |  | 1 | 3564 | 交差中の遊脚の高さの保持 5 cm |
| `MPC_FOOTSTEP_CROSS_HOLD_XY` | 0.02 |  |  | 2 | 3565 | 保持を解く水平距離 2 cm（PUSH_REPLAN の保持も共有） |
| `MPC_FOOTSTEP_HOLD_SHIFT` | 0.0 |  |  | 3 | 3571 | 大きく動かした着地にも高さの保持（v6.13 の部品） |
| `MPC_FOOTSTEP_BACK_ONLY` | false |  |  | 3 | 3577 | 後ろ支配の誤差だけ（v6.13 の部品） |
| `MPC_FOOTSTEP_BACK_PERSIST` | 0.04 |  |  | 3 | 3585 | 同、持続 |
| `MPC_FOOTSTEP_FLOOR_BACK_ERR` | 0.02 |  |  | 3 | 3591 | 同、下限の閾値 |
| `MPC_FOOTSTEP_BACK_VMAX` | ∞ |  |  | 3 | 3597 | 同、速度指令の上限（v6.13.1、v6.14 で親ごと外れた） |
| `PUSH_REPLAN` | 0.0 | 0.03 |  | 1 | 3611 | 押しの後の再計画: 支持開始時の計画に対する DCM 誤差を着地へ運び、足跡と DCM 参照を引き直す（閾値 3 cm、v6.14） |
| `PUSH_REPLAN_STAND` | 2.0 |  |  | 2 | 3612 | 捕捉の後のその場の歩数 2 |
| `PUSH_REPLAN_RAMP` | 2.0 |  |  | 2 | 3613 | 元の歩幅へ戻す歩数 2 |
| `PUSH_REPLAN_FREEZE` | 0.85 |  |  | 2 | 3614 | この位相を過ぎたら再計画しない 0.85 |
| `PUSH_REPLAN_MAX_FWD` | 0.35 |  |  | 2 | 3615 | 前へのずらしの上限 |
| `PUSH_REPLAN_MAX_BACK` | 0.30 |  |  | 2 | 3616 | 後ろへのずらしの上限 |
| `PUSH_REPLAN_MIN_W` | 0.12 |  |  | 2 | 3617 | 最小の横幅 |
| `PUSH_REPLAN_CROSS` | 0.0 |  |  | 3 | 3618 | 交差の許容（0 = 無効） |
| `PUSH_REPLAN_AHEAD` | 0.10 |  |  | 3 | 3619 | 交差のとき支持足より前に置く量（CROSS=0 で到達しない） |
| `PUSH_REPLAN_HOLD` | 0.06 | 10 |  | 1 | 3620 | 遊脚の高さを保持する移動量の閾値。10 m = 実質的に保持を切る（v6.19）。ライブラリでは「保持なし」として落とせる |
| `PUSH_REPLAN_BACK_ONLY` | false | 1 |  | 1 | 3627 | 後ろ支配の誤差だけで起動 |
| `PUSH_REPLAN_VMAX` | ∞ | 0.15 |  | 1 | 3628 | 速度指令 0.15 m/s 超では起動しない（PUSH_YIELD に任せる） |
| `PUSH_REPLAN_LAT` | false |  |  | 3 | 3637 | 横支配で外向きの一歩にも（未採用） |
| `PUSH_REPLAN_LAT_VMAX` | ∞ |  |  | 3 | 3638 | 同 |
| `PUSH_REPLAN_LAT_VMIN` | 0.0 |  |  | 3 | 3645 | 同 |
| `PUSH_REPLAN_EMIN` | 0.015 |  |  | 2 | 3648 | 現在の DCM 誤差の下限 1.5 cm |
| `PUSH_REPLAN_HOLD_Z` | 0.05 |  |  | 3 | 3651 | 保持の高さ 5 cm（HOLD=10 で到達しない） |
| `PUSH_REPLAN_MAX_W` | 0.40 |  |  | 2 | 3653 | 外への最大幅 0.40 |
| `PUSH_REPLAN_ANKLE` | 0,1 |  |  | 2 | 3661 | 横の誤差から足首の分を差し引く係数（既定 0,1） |
| `PUSH_REPLAN_DEBUG` | 未設定 |  |  | 5 | 3667 | PUSH_REPLAN のデバッグ出力 |
| `MPC_FS_CROSS_DEBUG` | 未設定 |  |  | 5 | 3677 | 交差のデバッグ出力 |
| `MPC_FOOTSTEP_Y_IN_FWD_TRIG` | 0.005 | 0.005 |  | 2 | 3682 | 内側を許す前向き誤差の閾値 5 mm（チャンピオンも明示） |
| `MPC_FOOTSTEP_FREEZE_FRAC_RISE` | mpc_footstep_freeze_frac |  |  | 3 | 3688 | 昇りの凍結位相（親が ∞） |
| `MPC_FS_DEBUG` | 未設定 |  |  | 5 | 3689 | MPC 着地のデバッグ出力 |
| `MPC_FOOTSTEP_TRIG` | 0.0 | 0.015 |  | 1 | 3698 | DCM 誤差 1.5 cm で着地変数を武装（その歩はラッチ） |
| `MPC_FOOTSTEP_TRIG_HOLD` | 0.0 |  |  | 3 | 3706 | 武装を後続の歩にも保つ |
| `VXS_BOX_GAIN` | 0.0 |  |  | 3 | 3712 | 速度に応じて着地の箱を広げる（0 = 無効） |
| `MPC_FOOTSTEP_MAX_X` | 0.06 | 0.15 |  | 1 | 3716 | 着地のずらしの x 上限 0.15 |
| `MPC_FOOTSTEP_MAX_Y` | 0.03 | 0.08 |  | 1 | 3717 | 着地のずらしの y 上限 0.08 |
| `MPC_DCM_W` | 0.0 |  |  | 3 | 3725 | MPC の中で計画の DCM を追う重み（0 = 無効） |
| `MPC_NET_COP_EQ` | false |  |  | 3 | 3726 | 正味 CoP を等式で |
| `MPC_W_POS` | 20.0 |  |  | 2 | 3727 | MPC の位置の重み |
| `MPC_W_VEL` | 4.0 |  |  | 2 | 3728 | 速度の重み |
| `MPC_W_MOM` | 2.0 |  |  | 2 | 3729 | 角運動量の重み |
| `MPC_W_ACC` | 6.0 |  |  | 2 | 3730 | 加速度の重み |
| `MPC_W_WRENCH` | 2e-4 |  |  | 2 | 3731 | レンチの重み |
| `MPC_W_SMOOTH` | 1e-3 | 10 |  | 1 | 3732 | 平滑の重み 10（実測荷重への係留と組） |
| `MPC_SOLE_AUTHORITY` | false | 1 |  | 1 | 3733 | MPC の足ごとのレンチを WBC の力の名目に全面採用（両脚支持） |
| `MPC_REACH` | 0.0 |  |  | 3 | 3743 | 重心高さを MPC が脚の届く範囲で決める（不採用） |
| `MPC_REACH_W` | 1e6 |  |  | 3 | 3744 | 同 |
| `MPC_REACH_MAX_DROP` | 0.06 |  |  | 3 | 3745 | 同 |
| `MPC_REACH_MIN_LOAD` | 0.3 |  |  | 3 | 3750 | 同 |
| `MPC_REACH_AHEAD` | -1.0 |  |  | 3 | 3758 | 同 |
| `MPC_REACH_SAGITTAL` | false |  |  | 3 | 3763 | 同 |
| `MPC_REACH_TRACK` | 2.0 |  |  | 3 | 3770 | 同、WBC の追い方 |
| `MPC_REACH_SLEW` | 0.05 |  |  | 3 | 3771 | 同 |
| `MPC_REACH_DEBUG` | 未設定 |  |  | 5 | 3775 | MPC_REACH のデバッグ出力 |
| `MPC_AUTHORITY_ALL` | false |  |  | 3 | 3776 | 権限を片脚支持にも（既定は両脚支持だけ） |
| `LOAD_RAMP_FRAC` | 0.6 |  |  | 2 | 3777（+1） | 両脚支持で荷重を移す割合 0.6（WBC の容量傾斜と MPC の両方で読む） |
| `STAIR_WRENCH_DT` | 0.02 |  |  | 2 | 3778 | MPC の更新周期 20 ms |
| `STAIR_WRENCH_MAX_AGE` | 0.0 |  |  | 3 | 3781 | 失敗後に保持する解の寿命（0 = 無期限） |
| `STAIR_WRENCH_N` | 8.0 | 8 |  | 2 | 3782 | MPC の節点数 8（チャンピオンも明示） |
| `STAIR_WRENCH_PREVIEW_DT` | 0.05 |  |  | 2 | 3783 | MPC の節点の刻み 50 ms |
| `STAIR_WRENCH_LOW_MIN` | 0.12 |  |  | 2 | 3784 | 歩行の荷重配分の下限 12%（歩行の枝でも使う） |
| `STAIR_WRENCH_HIGH_MIN` | 0.05 |  |  | 3 | 3785 | 移行中の高い足の下限 |
| `STAIR_TOUCHDOWN_LOAD_TIME` | 0.50 |  |  | 3 | 3786 | 移行の着地の荷重傾斜の時間 |
| `STAIR_TOUCHDOWN_HIGH_SHARE` | 0.25 |  |  | 3 | 3787 | 同、高い足の割合 |
| `STAIR_WRENCH_ACCEL` | false |  |  | 3 | 3788 | MPC の加速度を CoM タスクへ |
| `STAIR_WRENCH_ACCEL_XY` | false |  |  | 3 | 3789 | 同、水平 |
| `MPC_XI_REF` | false |  |  | 3 | 3795 | WBC の DCM 参照を MPC の予測から |
| `MPC_XI_REF_LEAD` | 0.0 |  |  | 3 | 3802 | 同、先読み |
| `MPC_XI_REF_TRANSFER_PLAN` | false |  |  | 3 | 3803 | 同、移行計画 |
| `STAIR_WRENCH_FLOOR` | false |  |  | 3 | 3804 | 移行中の力の下限 |
| `STAIR_CAPTURE_FLOOR` | false |  |  | 3 | 3819 | Capture 中の力の下限 |
| `STAIR_ANKLE_REG` | 0.0 |  |  | 3 | 3830 | 足首モーメントの正則化（0 = 無効） |
| `STAIR_TRUNK_COUPLE` | 0.0 | 2.43422 |  | 3d | 3849 | 移行中の胴の連成の重み。チャンピオンに 2.43 があるが移行 off で到達しない（死に値） |
| `STAIR_CAPTURE_LOW_MIN` | -1.0 |  |  | 3 | 3853 | Capture 中の低い足の下限 |
| `STAIR_WRENCH_BLEND` | 0.25 |  |  | 2 | 3854 | 権限が無いとき（片脚支持）の MPC レンチの混合率 0.25 |
| `STAIR_WRENCH_COUPLED` | true | 0 |  | 1 | 3855 | 連成段（MPC レンチを WBC の段に）を切る（既定 true をチャンピオンが 0 に） |
| `STAIR_WRENCH_LEVEL_WEIGHT` | 1.0 |  |  | 3 | 3856 | 連成段の重み（連成 off で到達しない） |
| `STAIR_WRENCH_COM_WEIGHT` | 10.0 |  |  | 3 | 3857 | 同 |
| `STAIR_WRENCH_MOM_WEIGHT` | 0.2 |  |  | 3 | 3858 | 同 |
| `STAIR_WRENCH_TRUNK_WEIGHT` | 2.0 |  |  | 3 | 3859 | 同 |
| `STAIR_WRENCH_TRUNK_COUPLED` | false |  |  | 3 | 3860 | 同 |
| `STAIR_MANAGED_CONTACT` | true |  |  | 3 | 3861 | 移行中の接触の管理（既定 true だが移行 off で到達しない） |
| `STAIR_MANAGED_RISE` | false |  |  | 3 | 3862 | 同 |
| `STAIR_MANAGED_FZ` | 0.20 |  |  | 3 | 3863 | 同 |
| `STAIR_MANAGED_TANGENT` | 0.03 |  |  | 3 | 3864 | 同 |
| `STAIR_MANAGED_ROTATION` | 0.05 |  |  | 3 | 3865 | 同 |
| `STAIR_MANAGED_AZ_MAX` | 15.0 |  |  | 3 | 3866 | 同 |
| `STAIR_MANAGED_RATE` | 0.0 |  |  | 3 | 3873 | 同 |
| `STAIR_MANAGED_LATCH` | 0.10 |  |  | 3 | 3876 | 同 |
| `STAIR_HYBRID_CONTACT` | true |  |  | 2 | 3880 | 実測の接触集合を MPC と WBC に結合し、傾いた足裏（転がり）の CoP 箱を縮める（既定 true、斜面・段で働く） |
| `STAIR_CONTINUOUS_CONTACT` | false |  |  | 3 | 3883 | 着いた足の権限を滑らかに（不採用） |
| `STAIR_COMMIT_ON_HIGH` | false |  |  | 3 | 3899 | Commit を高い足に |
| `TOUCHDOWN_RAMP` | 0.0 |  |  | 3 | 3913 | 着地の荷重容量の傾斜（phase_gain、0 = 無効） |
| `SWING_CONTACT_SOFT` | 0.0 |  |  | 3 | 3930 | 遊脚の接触を柔らかい容量で受ける（phase_gain、0 = 無効） |
| `SWING_CONTACT_SOFT_AFTER` | 0.0 |  |  | 3 | 3931 | 同 |
| `SWING_CONTACT_ON` | 0.10 |  |  | 3 | 3932 | 同、閾値 |
| `SWING_CONTACT_CAP` | 0.30 |  |  | 3 | 3933 | 同、容量 |
| `STAIR_CONTACT_ACTIVATION_TIME` | 0.40 |  |  | 3 | 3937 | 連続接触の立ち上げ時間（親が無効） |
| `STAIR_CONTACT_MIN` | 0.03 |  |  | 3 | 3938 | 同、下限 |
| `STAIR_LOAD_CHATTER_MAX` | 0.0 |  |  | 3 | 3941 | Rise の荷重の暴れの門（0 = 無効） |
| `STAIR_LOAD_CHATTER_TAU` | 0.20 |  |  | 3 | 3942 | 同、時定数 |
| `STAIR_RECONTACT_LOAD` | 0.05 |  |  | 3 | 3943 | 移行中の再接触の荷重 |
| `STAIR_RECONTACT_ALPHA` | 0.80 |  |  | 3 | 3944 | 同 |
| `STAIR_RECONTACT_Z` | 0.004 |  |  | 3 | 3945 | 同 |
| `STAIR_RECONTACT_ANGLE` | 0.10 |  |  | 3 | 3946 | 同 |
| `COM_PREVIEW_RUNTIME_CSV` | 未設定 |  |  | 5 | 4025 | 予見の解法時間の CSV |
| `PHASE_BY_CONTACT` | false |  | 0 | 3 | 4034 | 接触で位相を進める（kyo46rs 基準も 0 を明示） |
| `PHASE_EXTEND` | 0.5 |  |  | 3 | 4038 | 同、延長の上限 |
| `PHASE_MIN_FRAC` | 0.75 |  |  | 3 | 4044 | 同 |
| `PHASE_LOAD` | 0.30 |  |  | 3 | 4046 | 同 |
| `PHASE_TICKS` | 3.0 |  |  | 3 | 4047 | 同 |
| `DIAG_REPLAN` | false |  |  | 5 | 4061 | 再計画のたびに出力 |
| `PUSH_REPLAN_SLOPE_LEAD` | 0.0 | 0.6 |  | 1 | 4062 | 斜面の手前この距離では再計画しない（段の高さ 0 の斜面で門を補う、v6.17） |
| `REF_SLOPE_GATE` | false |  |  | 3 | 4063 | 斜面で参照を止める |
| `REF_SLOPE_LEAD` | 0.0 |  |  | 3 | 4064 | 同、距離 |
| `COM_XY_JERK_MAX` | ∞ |  | 100 | 1k | 4263 | 指令する水平加速度の変化の上限（∞ = 無効、kyo46rs 基準 100） |
| `TRANSFER_DIAG_CSV` | 未設定 |  |  | 5 | 4284 | 移行の診断 CSV |
| `ANKLE_CBF_DIAG_CSV` | 未設定 |  |  | 5 | 4292 | 足首 CBF の監査 CSV |
| `STAIR_WRENCH_EVENT_REPLAN` | false |  |  | 3 | 4323 | 位相・接触の切り替わりで MPC を引き直す |
| `MPC_QP_BACKEND` | （既定の解法） |  |  | 2 | 4327 | MPC の QP 解法の選択（既定の解法が働く） |
| `MPC_QP_WARM` | false |  |  | 3 | 4337 | MPC の温め始動 |
| `MPC_DIAG_DIR` | 未設定 |  |  | 5 | 4338 | MPC の QP の保存先 |
| `ACT_DELAY_TICKS` | 0.0 |  |  | 4 | 4339 | 作動の遅れ（模型のずれのベンチ） |
| `PREVIEW_TRAJ_CSV` | 未設定 |  |  | 5 | 4346 | 予見の解の CSV |
| `MPC_TRAJ_CSV` | 未設定 |  |  | 5 | 4359 | MPC の計画の CSV |
| `CONTACT_ADMIT_DEBUG` | 未設定 |  |  | 5 | 4669（+2） | admit のデバッグ出力 |
| `LAND_BY_LOAD_DEBUG` | 未設定 |  |  | 5 | 4691 | LAND_BY_LOAD のデバッグ出力 |
| `DS_FEASIBLE_DEBUG` | 未設定 |  |  | 5 | 4823（+1） | DS_FEASIBLE のデバッグ出力 |
| `DEBUG_ANCHOR` | false |  |  | 5 | 5932 | アンカーのデバッグ出力 |
| `LOAD_RAMP_TD` | false |  |  | 3 | 6323 | 着地足が既に担っている荷重を残す（不採用） |
| `LOAD_RAMP_CONTACT_T` | 0.0 |  |  | 3 | 6332 | 着地足の容量を接触入りの tick からも傾斜（未採用） |
| `CP_DEBUG` | 未設定 |  |  | 5 | 7454 | contact_plan のデバッグ出力 |
| `DCM_REF_BLEND_DEBUG` | 未設定 |  |  | 5 | 7722 | DCM 参照の混合のデバッグ出力 |
| `STAIR_CAPTURE_DEBUG` | 未設定 |  |  | 5 | 7892 | Capture のデバッグ出力 |
| `VH_DCM` | false | 1 |  | 1 | 8028 | 階段がある場面の可変高さの DCM（ω を支持足の上の高さから）。ループ内で読む |
| `MPC_FS_DEBUG_LANDING` | 未設定 |  |  | 5 | 8927（+3） | MPC 着地のデバッグ出力 |
| `MPC_LANDING_TIME_FROM_X` | true |  |  | 3 | 9218 | MPC_LANDING_TIME の設定（ループ内で読む、親が無効） |
| `MPC_LANDING_TIME_GAIN` | 1e-3 |  |  | 3 | 9219 | 同 |
| `MPC_CAPTURE_LEX_NET_EQ` | true |  |  | 3 | 9226 | MPC_CAPTURE_LEX の設定（ループ内で読む、親が無効） |
| `MPC_CAPTURE_LEX_TOTAL_EQ` | true |  |  | 3 | 9227 | 同 |
| `MPC_CAPTURE_LEX_IGNORE_MIN` | false |  |  | 3 | 9228 | 同 |
| `MPC_CAPTURE_LEX_FMARGIN` | 0.1 |  |  | 3 | 9229 | 同 |
| `MPC_KKT_COMP_TOL` | if mpc_capture_lex { 1e-2 } else { 1e-6 } |  |  | 2 | 9246 | MPC 解の KKT 相補性の許容（既定 1e-6、CAPTURE_LEX 時 1e-2）。ループ内で読む |
| `MPC_LEX_DEBUG_FB` | 未設定 |  |  | 5 | 9423 | 辞書式のデバッグ出力 |
| `MPC_LEX_DEBUG_T` | 未設定 |  |  | 5 | 9440 | 同、時刻 |
| `MPC_TRAJ_DEBUG` | 未設定 |  |  | 5 | 9497 | MPC 軌跡のデバッグ出力 |
| `TURN_IN_DS` | true |  |  | 2 | 9617 | 旋回を両脚支持の間に行う（既定 true、WZ≠0 のとき） |
| `STAIR_DESC_TOE_START` | stair_swing_windows[3] |  |  | 3 | 10548 | つま先着地の開始（STAIR_DESC_TOE が無効） |
| `WBCW_*` | 段ごとの表 | COM=100, POSTURE=0.001 |  | 1 | 10926 | weighted の段ごとの重み WBCW_<段名>（チャンピオン COM=100、POSTURE=0.001、他は既定表）。ループ内で format! で読む |
| `WBC_LEVEL_DEBUG_T` | 未設定 |  |  | 5 | 10963 | 段ごとの残差を出す時刻 |

`phase_gain` の段階別の値（全部分類 3、`STAIR_TRANSFER=0` で `RunPhase::Transfer/Post` に入らない）: `JLIM_AMAX_XFER/_POST`、`JLIM_Q_MARGIN_XFER/_POST`、`KP_TRUNK_XFER/_POST`、`KD_TRUNK_XFER/_POST`、`COP_FRAC_XFER/_POST`、`KP_SWING_XFER/_POST`、`KP_SWING_XY_XFER/_POST`、`KD_SWING_XFER/_POST`、`LIFT_H_XFER/_POST`、`KP_SWING_ROT_XFER/_POST`、`KD_SWING_ROT_XFER/_POST`、`SWING_ROT_XFER/_POST`、`ZMP_LAT_SCALE_XFER/_POST`、`STAIR_TD_SINK_XFER/_POST`、`TOUCHDOWN_RAMP_XFER/_POST`、`SWING_CONTACT_SOFT_XFER/_POST`（宣言は各ノブの行、仕組みは 228–272）。

### 付録 A. walk.rs の外で読まれるノブ

| 名前 | 場所 | 既定 | チャンピオン | 分類 | 説明 |
|---|---|---|---|---|---|
| QD_CLAMP | src/biped/rig.rs:838 | 5.0 | 40 | 6 | rig.sync が関節速度を切る上限（既定 5 rad/s の罠、記憶 qd-clamp-default-5） |
| BASE_VEL_FUSE_HZ | rig.rs:860 | 0 | 3 | 1 | 胴の並進速度の推定: 加速度計との融合の帯域（v6.15）。ライブラリでは状態推定器の Config |
| BASE_VEL_FUSE_BIAS_T | rig.rs:875 | 0 | 2 | 1 | 同、加速度計の偏りの推定の時定数（v6.15） |
| BASE_VEL_KIN | rig.rs:858 | off |  | 3 | 運動学だけの速度推定（不採用） |
| BASE_VEL_KIN_FMIN | rig.rs:859 | 30 |  | 3 | 同 |
| SENSE_NOISE_A | rig.rs:866 | 0 | 0.02 | 4 | 加速度計の雑音（センサ模型。v6.15 から比較の標準として controls に入れている） |
| SENSE_ACC_BIAS | rig.rs:867 | 0 | 0.05,-0.05,0.05 | 4 | 加速度計の偏り（同上） |
| SENSE_DELAY_TICKS / SENSE_DELAY_SCOPE | rig.rs:842/852 | 0 |  | 4 | センサの遅れ（sense ベンチ） |
| SENSE_NOISE_Q / _QD / _V / _W | rig.rs:844–847 | 0 |  | 4 | センサの雑音（sense ベンチ） |
| SENSE_SEED | rig.rs:849 | 1 |  | 4 | 雑音の種 |
| SENSE_FILTER_HZ / _DT / _V_HZ | rig.rs:851–856 | 0 |  | 3 | センサの低域通過（実験） |
| SENSE_VEL_BIAS_X / _Y / _Z | rig.rs:862–864 | 0 |  | 4 | 速度の偏り（診断） |
| PLANT_MASS_SCALE / PLANT_TORSO_MASS_SCALE | rig.rs:1318/1323 | 1 |  | 4 | 植物の質量のずれ |
| PLANT_PAYLOAD（_KG/_X/_Y/_Z） | rig.rs:1333–1339 | なし |  | 4 | 植物の積載物（mis_p5 ベンチ） |
| RIG_MJCF_OUT | rig.rs:512 | なし |  | 5 | 生成した MJCF の書き出し |
| LAND_CONFIRM_XY / LAND_CONFIRM_Z | crates/misa-biped/src/preview.rs:587–588 | 0.012 / 0.010 |  | 2 | 着地確認の既定の許容。**ライブラリが環境変数を読んでいる**ので Config に移す |
| LAND_TIMING_DEBUG | preview.rs:551 ほか |  |  | 5 | デバッグ出力（ライブラリ内） |
| MPC_LANDING_TIME_DEBUG / MPC_FS_DEBUG | crates/misa-mpc/src/wrench_mpc.rs:891 ほか |  |  | 5 | デバッグ出力（ライブラリ内。`MPC_FS_DEBUG` は walk.rs でも読む） |

## 4. 切り出しの所見

### 4.1 `main()` の中の区画（ループ 4417–12729）

| 行 | 中身 | 性質 |
|---|---|---|
| 4417–4537 | 段階別利得の上書き、着地確認の許容の毎 tick 設定、押しの計測窓 | 制御＋計測 |
| 4536–4541 | `rig.sync()` と `measure_contacts`（MuJoCo の接触力をそのまま「センサ」として読む） | リグ |
| 4538–5128 | 着地の確認・admit、両脚支持の作り直し（DS_FEASIBLE／RESTEP／DS_CAPTURE／LIFT_GATE）、着地での CoM 再計画 | 制御（採用と不採用が交互） |
| 5129–5834 | 段の三相移行の起動・状態機械・再開 | 不採用 |
| 5835–6646 | 予定から接触集合、admit、管理接触、柔らかい遊脚接触、荷重傾斜、接触段と CoP パッチ、力の下限 | 制御（不採用の規則が多い） |
| 6647–7205 | DCM 則の入り、PUSH_YIELD、PUSH_REPLAN、PUSH_Z_DROP、参照の作動 | 制御 |
| 7206–7598 | ADAPT_DS／ADAPT_TIME／ADAPT_STEP（不採用）と contact_plan（採用） | 制御 |
| 7599–8058 | CoM 予見の再計画、移行中の DCM、MPC_PCMD、`p_cmd` と加速度 | 制御 |
| 8059–8261 | 高さ参照（段、降段の先行、届く範囲、速度・押しの重心下げ） | 制御 |
| 8262–9568 | レンチ MPC（荷重予定、着地変数、交差、解、ログ、解の適用） | 制御＋ログ |
| 9569–10070 | 胴、角運動量、MOM_CMP、腕、姿勢、力の名目 | 制御 |
| 10071–10635 | 遊脚タスク | 制御 |
| 10636–11124 | 段の組み立て・重み・解法・受け入れの門 | 制御 |
| 11125–11690 | QP 保存、診断量、自己衝突の監視、指令の書き込み、FIX_* の探査 | 計測・デバッグ＋リグ |
| 11691–11829 | 押しの発火、ベンチの参照発火、参照の終了と再センタリング | シナリオ＋制御が同居 |
| 11830–12529 | 行ログ・CSV・監査、`rig.step` | 計測＋リグ |
| 12530–12729 | 押しの窓・回復・判定の報告 | 計測 |

### 4.2 制御とシナリオ・ログが絡んでいる箇所

- **参照の終了処理が押しの発火の区画にある**: 11766 で `ref_trig_at_push`（ベンチの押しの瞬間に参照を入れる神託）と同じ `if` の並びに、11780–11830 で参照の冷却・終了・支持足と遊脚の再センタリング（採用の `REF_RESET_INCL_CURRENT`）がある。押しの発火（11696–11762）は制御の後・`rig.step` の前に植物へ力を加えるハーネスの仕事なので、ここを境に Walker とハーネスを分けると再センタリングがハーネス側に取り残される。
- **地形の模型 = ベンチの地形生成**: 制御は `stairs`（`STAIRS`／`TERRAIN_SLOPE` から作った `terrain::Stairs`）を完全な知識として 23 箇所で `stairs.is_some()`／`stairs.as_ref()` を見る。一方 `TERRAIN_BUMPS` は計画に入らない（正しく分かれている）。Walker には「既知の地形の模型」を入力として渡し、ベンチの段差 0 の段を「段がある」と扱うか（`ZMP_ROLL`、`VH_DCM` の既定が変わる）を明示する必要がある。
- **接触の測定がシミュレータの接触力そのもの**: 4540 `measure_contacts(&rig, data, friction_mu)` は MuJoCo の接触力を読み、`measured_pre`／`fz_meas` として admit・荷重傾斜・MPC の係留（`MPC_SEED_PREVIOUS=2`）に使う。状態推定（`rig.sync()`、付録 A の融合推定器）と同じく、Walker の入力（`Observation`）として渡す形にする。
- **ログとデバッグが制御の分岐の中にある**: `println!` が 186 箇所、ループ内で `std::env::var` を毎 tick 読む箇所が 17（ほとんど `*_DEBUG`）（4669、6002、7454、8927、9392、9423、9440、9497、10963 など）。MPC の区画（9286–9330）は解の直後に CSV を書く。Walker には観測用のフック（解ごとの記録の構造体）を用意し、出力はハーネスでする。
- **ループ内で読むノブ**: `VH_DCM`（8028）、`WBCW_*`（10926）、`LOAD_RAMP_FRAC`／`LOAD_RAMP_TD`／`LOAD_RAMP_CONTACT_T`（6312–6332）、`MPC_KKT_COMP_TOL` ほか MPC の設定 7 個（9218–9246）、`TURN_IN_DS`（9617）、`STAIR_DESC_TOE_START`（10548）。Config に移すときは起動時に一度読む形にそろえる。

### 4.3 切り出しを難しくしている共有状態

- **`footsteps`（足跡計画）を書く箇所が 21**: 1690、2306、4605–4641（計画の付け直し `PLAN_REANCHOR*`／`VX2_REANCHOR`）、4678（`CONTACT_EVENT`）／4700（`LAND_BY_LOAD`）、4926（RESTEP）、5638（段移行の後始末）、5884（ADAPT_STEP）、6993（PUSH_REPLAN）、7161（参照）、7304（ADAPT_TIME_WIDE）、7533／7535（contact_plan）、7585（ADAPT_LIVE）、9481／9483（MPC の着地）、11801（参照の再センタリング）。採用分に絞っても contact_plan・PUSH_REPLAN・MPC の着地・参照の 4 者が同じ計画を tick の中で順に書き換える。
- **`dcm_plan` の作り直しが 31 箇所、`gait`（時間割）の書き換えが 28 箇所**（例: 4715／4798／4831 の DS_FEASIBLE、4956／5041 の DS_CAPTURE、6877／6994 の PUSH_YIELD／PUSH_REPLAN、7556 の contact_plan、9378／9413／9486 の MPC の着地、11804／11812 の参照）。「計画（`gait`＋`footsteps`＋`dcm_plan`＋`height_plan`）」を 1 つの構造体にまとめ、書き換えの経路を「再計画の要求」に一本化しないと、部品ごとに分けた Config が互いの不変条件を壊す。
- **`landing_timing` を毎 tick 外から設定し直している**（4482–4525: 確認の許容 `xy_tol_override`／`z_tol_override`、力の閾値、延長 `max_extension` を、昇り・降り・contact_plan の時間調整の有無で上書き）。LandingTiming に「歩の種類」を渡して内部で決める形にすれば、LAND と DESC と CP の Config が 1 か所で済む。
- **`wrench_mpc_command` が 20 箇所で読まれる**: 力の名目（9967）、連成段（9677–9682、採用形では閉じている）、加速度（9525）、FFDC、`MPC_XI_REF`。`WRENCH_MPC_APPLY` が複合ゲートなのはこのため（記憶 apply-flag-confound）。採用形では「力の名目への権限（両脚支持）と混合率（片脚支持）」だけが生きている。
- **段ゾーン**（`STAIR_ZONE_CROUCH_LEAD/TAIL`）を 3 つの仕組みが共有: 降段のしゃがみ（8156）、MOM_CMP の無効化（9778–9781）、weighted の範囲（10904、不採用側）。
- **`stair_transfer`／`stair_transfer_cmd`／`RunPhase`**: 不採用だが、MPC（8267、8360–8530）、接触集合（6083–6200）、遊脚（10080）、段の組み立て（10685–10746）、利得（4422–4445）の分岐に入り込んでいる。先に移行を消してから切り出すと、各区画の条件が 1 段ずつ浅くなる。

### 4.4 進め方の案

1. 死に値 10 個をチャンピオンの controls から外して、バイナリ同一のまま軌跡がビット一致することを確かめる（静的判定の検証を兼ねる）。
2. 分類 3 のうち段の三相移行（88＋32）と `ADAPT_*`／`RESTEP` を先に削る。挙動は変わらないはずなので、家族 21 ベンチのビット一致で確認できる。
3. §2 のまとまりを Config のサブ構造体にし、名前を意味に付け替える（`STAIR_WRENCH_MPC`→`mpc.enabled`、`PUSH_REPLAN_HOLD=10`→保持なし、など）。環境変数→Config の対応表はハーネスに残す。
4. 要確認 7 個（`ROUGH_MODE`、`ROUGH_MODE_LIFT_H`、`TERRAIN_PITCH`、`VX2_SCHED`、`STAIR_ABSOLUTE_TD_DESC`、`STAIR_SWING_FF`、`STAIR_ZONE_SWING_ROT`）は、地形の知覚・速度指令・kyo46rs v3 の判断と合わせて決める。


## 5. 実行による確認（2026-10-29、tu60 = 束 A・B 削除後のバイナリ）

方法: `log/claude_g1rand_20260925/knobprobe.py`。ノブを 1 つだけ既定と違う値にして見本を順に回し、TRAJ_CSV が振る前と 1 バイトでも違えば「効く」。分類 4・5 は振らない。結果は `log/claude_g1rand_20260924/KNOBPROBE_v619/results.tsv`（1 回目、見本 15 本、494 組）と `KNOBPROBE_v619b/results.tsv`（2 回目、押しの難しい見本 14 本＋1 回目の 15 本、値も振り直し、163 組）。

### 5.1 1 回目の見かけの食い違い
- 分類 1・2 の「効かない」55 個の大半は振り方の産物だった: 1 を on として使うスイッチを 1.5 にした（on のまま）、`PUSH_REPLAN_HOLD=10`→15（保持なしのまま）、押しの仕組み（`DS_CAPTURE`、`PUSH_REPLAN`、`REF_*`、MPC の交差）が m00 の押しでは発火しない。
- 分類 3 の「効く」88 個のうち 84 個は既定 off のスイッチを on にしただけ（台帳どおり）。残り 4 個（`COM_REPLAN_LATERAL_GAIN`、`LAND_KD_ROT_BOOST`、`MPC_FOOTSTEP_FREEZE_FRAC`、`MPC_F_MAX_SCALE`）は既定が中立の値（倍率 1、∞、NaN）で、既定のままなら効かない。削るときは中立の定数に置き換える。`COM_REPLAN_LATERAL_GAIN=1.5` は範囲の assert で起動時に落ちていた（1 回目は異常終了を「効く」に数えた。2 回目から区別）。

### 5.2 2 回目（押しの見本と値の振り直し）
- 1 回目に効かなかった分類 1・2・6・1k・? の 64 個のうち **45 個が効く**（多くは `p/i11_d180/m06`、`hold/i5_d90/m15`、`p/i11_d0/m28` の押しで初めて発火）。**3 個は範囲外の値で異常終了**（`DS_FEASIBLE_MAX=0`、`STAIR_SOLE_MARGIN=0.006`、`VXS_STRIDE_MAX≤0.225`）で、到達はしている。
- **なお効かない 16 個:**
  - 説明のつくもの: `TERRAIN_PITCH`、`VX2_SCHED`、`STAIR_ZONE_SWING_ROT`（ベンチ・kyo46rs の設定が入れる）、`STAIR_FULL_SOLE`（kyo46rs だけ）、`HOLD_KP/KD`、`RUN_KP/KV`、`NO_ARM_COLLIDE`（機体・リグ側、G1 の歩行で効く場面が無い）、`MPC_KKT_COMP_TOL`（診断の許容）。
  - **`CP_TRACK`（採用）は死に値**: contact_plan のずらしが `CP_MAX_*=0` で常に 0 なので、どちらの分岐も 0 を返す（5352–5362）。3d に移す。
  - **`CONTACT_ADMIT_CONFIRMED`（採用）は 29 本のどれでも効かない**: 採用済みの `CONTACT_ADMIT_LOADED` が先に admit するので、確認の経路まで来ないと見ている（3902 の門）。外して家族で確かめる候補。
  - **`COM_LAND_VZ`（採用 0.05）は G1 の段の高さでは効かない**: 着地前の CoM 再計画の時間が `max(xy/vx, z/vz)` で、80 mm の段でも xy の項が勝つ（1875）。構造としては生きている。
  - `DS_CAPTURE_MARGIN`、`STEP_DZ_SWING_MIN`、`VXS_Z_MAX`（分類 1・2）: 門の中の閾値で、見本の範囲では結果を変えない。構造としては生きているので残す。
- 結論: 台帳の分類 1・2 は、`CP_TRACK` を除いて実行でも生きている（または生きている構造の中の閾値）。分類 3 は中立の既定 4 個の扱いを加えれば台帳どおりに削れる。
