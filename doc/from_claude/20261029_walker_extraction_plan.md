# kyo46rs_walk.rs の制御本体を misa-biped の `Walker` へ移す計画（2026-10-29）

対象: `examples/kyo46rs_walk.rs`（7,583 行、コミット 2409d96 時点）。ノブの台帳 `doc/from_claude/20261029_knob_inventory.md`（以下「台帳」）の §2・§4・§6 を下敷きにした。**行番号はすべて 2409d96 の今のソース**で引き直したもの（台帳の行番号は削除前なので使わない）。段階を進めると行はずれるので、各区画には目印（コメントの見出しや変数名）も添えた。

この文書はコードを変えていない。読んだのは walk.rs 全体の区画、`src/biped/{rig,actuate,log,contact,tasks,mod}.rs`、`crates/misa-biped`、`crates/misa-mpc` の公開面、確認の道具 `log/claude_g1rand_20260925/{eq.sh,eqcmp.sh,eqlist.txt,eqlist_push.txt}`。

## 0. 要点

- **Walker の 1 tick は「観測 → WBC の解」**。ハーネスは `rig.sync()`＋`measure_contacts` で観測を作り、`walker.step(&obs)` で解（関節トルク・qddot・状態）を受け取り、劣化時の方策（`CommandPolicy`）・計測・押しの発火を済ませてから `walker.end_tick()`（押し回復の参照の終了処理）を呼び、植物へ書いて進める。この順にすると**今のログの行の順がそのまま保てる**（§2.5）。フックや出力の緩衝は要らない。
- 制御が観測から読むのは**接触力の鉛直成分 2 個（`f_w[side][2]`）だけ**。`MeasuredContact` の残り（CoP、すべり、レンチ）はハーネスの計測でしか使っていない。だから `log.rs`（他の人の差分あり）に触らずに切り出せる。
- **計画の書き換え手は tick の中に 13 か所**ある（§3.1 の表）。`DcmPlan::from_footsteps_rolled` の呼び出しは 14 か所（ループ内 11、感度の試し 1、起動時 2）。まず `Plan` 構造体に束ねて `rebuild_dcm()` に置き換え（機械的）、Walker が misa-biped に入った後で `PlanEditor`（書き手の名札つきの編集）と `SliceClaims`（その区間の終わりの時刻と着地点を誰が持つか）に一本化する。今の「誰が先に持ったか」の判定 7 種類（`cp_retimed_slice`、`landing_closed`、`push_replan_slice`、`ref_mode.is_some()` など）をそこへ集める。
- WBC の段の組み立て（tasks.rs・contact.rs）は **trait `WbcStack` で注入する（案 A、推奨）**。Walker は参照（CoM 加速度、胴、遊脚の目標、名目の力、接触集合、係留点の出来事）を計算して `WbcRequest` にまとめ、articara 側の `TaskStack` が段を組んで解く。misarta の運動学・動力学（M、h、FK、CoM ヤコビアン）は misa-biped の `RobotModel`／`RobotState` に置く。今それをやっている `rig.sync()` の後半（`src/biped/rig.rs` 1082–1228 行）はもともと MuJoCo に依存していない。
- ループの前の `let mut` は **170 個**。内訳は、制御の状態 42、計画 5、制御のカウンタ 25＋診断値 7、ハーネスの計測・判定 67、診断の出力先 18、起動時だけのもの 6（§1.4）。
- 段階は **S0〜S17 の 18 段**（S9・S11 は小分けにする）。S1〜S8 は example の中だけで済む機械的な束ね直しで、misa-biped に入れるのは S6 の `Plan` と S7 の参照ライブラリだけ。
- 各段の確認は、作業用の worktree で作ったバイナリを `eq.sh` にかけ、96 本と 14 本の両方で **CSV 差 0・ログ差 0** になることを見る。example のディレクトリ化と診断出力に触る段では、診断出力の見本（§5.0 の `eqdiag`）も比べる。
- 作業ツリーには他の人の未コミットの差分が 115 ファイルある（`src/biped/{actuate,contact,log}.rs` を含む）。基準のバイナリと新しいバイナリが同じ土台から作られるよう、**HEAD から切った worktree で作業する**（§5.0）。
- 挙動を変えずには直せない「漏れ」が 4 つ見つかった。いずれも §7 の未決に回す。
  - posture・劣化時の代替・Hybrid の書き込みが、遅延や雑音の入らない植物の真の関節角を読んでいる。
  - `friction_mu` の既定が、植物の摩擦 `MU_GROUND`（rand.py が個体ごとに振る値）から作られている。
  - 「段がある」の判定が 4 通りある。
  - 再構築が DS_FEASIBLE の区間 ZMP の書き換えを黙って消す。

## 1. 今の main の構造

### 1.1 全体

| 行 | 中身 |
|---|---|
| 27–178 | `RefLib`（押し回復の参照、JSON の読み込みと補間）、`RefMode`（作動中の参照の座標系）。`main` の外 |
| 179–7563 | `#[cfg(feature = "mujoco")] fn main()` |
| 7565–7569 | mujoco 無しの `main`（案内だけ） |
| 7574–7583 | `landing_inward()`（MPC の着地の内側の向き） |

### 1.2 起動時（179–2885）

| 行 | 中身 | 行き先 |
|---|---|---|
| 179–198 | `use` | — |
| 199–220 | `ROUGH_MODE=1` が `set_var` で `SWING_Z_HOLD`／`SWING_Z_DROP`／`SWING_XY_DELAY`／`LIFT_H` を書き換え、`rough mode:` を印字 | Config の上書き（§4.4）、印字はハーネス |
| 221–228 | `env_f64`／`flag` の閉包 | ハーネス |
| 229–419 | Profile、`RigOptions`（初期姿勢の種、粘性・アーマチュア、トルク倍率、焼き込み、作動モード、SIM_DT、MU_GROUND、noslip、solimp）、URDF・リンク名の上書き | ハーネス（リグ） |
| 420–431 | 脚の届く範囲 `LegReach`、`SOLE_HALF_L/W`、`mu_ground`、`ctrl_mode` | Config（reach は Walker の診断列）、残りはリグ |
| 432–451 | 突きの的 `SPAR_WALL`（MJCF） | シナリオ |
| 452–619 | 段・斜面（`STAIRS`／`TERRAIN_SLOPE`／`STAIRS_DOWN`／`STAIR_LANDING_LEN`）→ `Option<Stairs>` と MJCF の箱、出現の高さ | シナリオ（同じ `Stairs` を Walker に渡す） |
| 620–651 | 模型に入らない段差 `TERRAIN_BUMPS`（MJCF だけ） | シナリオ |
| 652–673 | `BipedRig::build`、`hip_mi`、`nv`／`na`／`mj_dt`／`total_mass` | ハーネス → `RobotModel` |
| 675–739 | WBC の `Solver`、QP 診断（`QP_DIAG_DIR`、`QP_CAPTURE_*`）、`QP_BENCH_GATE`／`MPC_BENCH_GATE`、ランタイム CSV 4 本 | Stack（解法）＋診断の出力先 |
| 740–809 | `SolveConfig`、`WBC_STRATEGY`、`WBC_QP_BACKEND`、関節制限の CBF（`JLIM*`） | Config → Stack |
| 810–1007 | 制御のノブ: `friction_mu`、利得、腕・突き（`JAB*`）、角運動量、MOM_CMP、胴、`EOM_HARD`、`COP_FRAC`、劣化時の保持、係留、遊脚の利得、`K_DCM`、ZMP | Config（`JAB*` は指令の表） |
| 1008–1016 | `GaitParams`（`T_START`／`T_DS`／`T_SS`／`N_STEPS`／`FIRST_SWING`、`t_end = T`）、`NO_LIFT` | Config＋Walker の設定（`t_end`） |
| 1017–1068 | 押しのシナリオ（`PUSH_*`、窓、計測の閾値） | シナリオ・計測 |
| 1069–1093 | 荷重移行の計測の閾値、`CTRL_DT`、`DIAG_VZ_BIAS_*` | 計測／Config |
| 1094–1113 | `mj_substeps`、`dt`、`n_ticks`、サブステップの監査 | ハーネス（`dt` は Walker に渡す） |
| 1114–1183 | 足裏の報告、初期の FK（`com0`、`steps`）、`foot_rot_at`、追従の FF の門 | Walker::new |
| 1184–1216 | `GaitPlan`、`DS_SETTLE`、`Z_COM`（既定は `com0.z`）、`ZMP_LAT_SCALE`、`t_step` | Walker::new（計画） |
| 1217–1363 | 速度指令（`STRIDE`、`VX`／`VY`／`WZ`、`SQUAT_*`、`STRIDE_RAMP`、`VX2*`）と速度表（`VXS_*`、`VX2_*`）、押しの譲歩のノブ、重心下げの予定 | 指令の表＋Config → Walker::new |
| 1364–1632 | 足跡計画（`velocity_with_change`）、段の区間の決定的な作り直し、`stairs footfalls:` の印字 | Walker::new |
| 1633–1662 | `COM_PREPLAN`（遊脚時間の事前確保、`preplan:` の印字） | Walker::new |
| 1663–1723 | `z_lip`、`ZMP_ROLL`（既定は `stairs.is_some()` で 0.030／0）、`DcmPlan`、`omega`、`gait:`／`lipm:`／`footsteps:`／`command:`／`stride:` の印字 | Walker::new |
| 1724–1737 | `ContactCfg` | Config → Stack |
| 1738–1880 | CSV の列名、`TrajLog`、`TICK_AUDIT_CSV`、`CommandPolicy`、`DegradedTally`、`Anchors`、`ContactCorrection` | ログ・方策はハーネス、係留点は Stack、接触の補正は Walker |
| 1881–1962 | contact_plan の設定、遊脚・斜面の踵のノブ、`terrain_track`、contact_plan の状態 | Config＋Walker の状態 |
| 1963–1997 | `FIX_*`（押しの後の補正の撃力、探査用） | シナリオ |
| 1998–2244 | `LandingTiming`、admit、ダンプ、参照ライブラリの読み込み、接触イベント、DS_CAPTURE／DS_FEASIBLE | Config＋Walker::new |
| 2245–2310 | 着地の確認と延長のノブ | Config |
| 2311–2554 | 重心の MPC、MPC の着地・交差、PUSH_REPLAN、管理接触（`STAIR_MANAGED_*`） | Config |
| 2555–2600 | 予見の状態、カウンタ、`DIAG_REPLAN` | Walker の状態・統計 |
| 2601–2701 | 足首ロールの限界、`StepStat`、走行の計測、的の当たり、荷重変化の EMA、管理接触の期限 | 計測（最後の 3 つは Walker） |
| 2702–2800 | 押しへの応答の計測、移行の計測、`TRANSFER_DIAG_CSV`、`ANKLE_CBF_DIAG_CSV` | 計測・診断 |
| 2801–2885 | MPC の診断、`ACT_DELAY_TICKS`、軌跡 CSV、MPC の状態 | 診断＋Walker の状態 |

起動時の印字の順（ログのバイト一致に効く）は次のとおり。

1. `rough mode:`
2. `stairs:`／`terrain mjcf:`
3. `bumps (unmodelled):`
4. リグの構築
5. サブステップの監査（1108）
6. 足裏の報告（1114）
7. `inter-step settle:`（1198）
8. `speed schedule:`／`speed change:`（1283、1333）
9. `full-sole plan:`（1444）
10. `stairs footfalls:`（1627）
11. `preplan:`（1650）
12. `gait:` 〜 `NO_LIFT:`（1682–1721）
13. `logging trajectory`（TrajLog）
14. `ankle_roll limit:`（2603）

7〜12 はまとめて `Walker::new` の中に入り、その前後はハーネスに残る。だから Walker::new の印字を LogSink 経由で即時に出せば、順はそのまま保てる。

### 1.3 ループ（2886–7158、`for tick in 0..n_ticks`）

| 行 | 中身 | 性質 → 行き先 |
|---|---|---|
| 2887–2890 | 時刻、計時の開始 | ハーネス（`t = tick as f64 * dt` は Walker も同じ式で作る） |
| 2891–2917 | 着地の確認の許容・延長を毎 tick 設定（昇り・降り・contact_plan が時間を決めた区間） | 制御 → Walker（相 1） |
| 2918–2927 | 押しの計測窓、劣化回数の窓 | 計測 → ハーネス |
| 2928–2933 | `rig.sync()`、`measure_contacts`、`measured_pre` | リグ → 観測 |
| 2934–2949 | `CONTACT_DUMP_T`（MuJoCo の接触を印字） | デバッグ → ハーネス |
| 2953–3193 | 着地の時計の更新、確認済み admit、接触イベントのずらし、DS_FEASIBLE（2992–3104）、DS_CAPTURE（3105–3177）、再計画の開始（3180–3192） | 制御 → Walker（相 2） |
| 3195–3220 | 時間割 → `support`、`steps`、`swing_rot_mode` | 制御（相 3） |
| 3222–3287 | 支持の切り替わりでの係留点の解放・設置 | 制御 → 係留点の出来事（Stack へ） |
| 3289–3300 | 荷重変化の EMA（管理接触の引き金） | 制御 |
| 3301–3412 | 接触集合（`ContactCorrection`、予定の admit、確認の admit、新しい接触の係留） | 制御（相 4） |
| 3414–3452 | 両脚支持の荷重の傾斜（`LOAD_RAMP_FRAC` を毎 tick 読む） | 制御 |
| 3454–3555 | 動力学の文脈、トルク箱、関節制限の CBF、接触段（`bt::contact_level`、管理接触、斜面の踵の CoP 箱） | WBC → Stack |
| 3556–3580 | DCM 参照（参照ライブラリの着地版の上書き） | 制御（相 5） |
| 3581–3633 | PUSH_YIELD（速度を落として引き直す） | 制御（計画の書き換え） |
| 3634–3751 | PUSH_REPLAN（後ろ押しで着地を引き直す） | 制御（計画の書き換え） |
| 3752–3767 | PUSH_Z_DROP の検出 | 制御 |
| 3768–3884 | 押し回復の参照の作動（時間割・足跡を参照に合わせる） | 制御（計画の書き換え） |
| 3885–3910 | DCM 誤差の最大・押し後の回復の判定 | 計測 → ハーネス（記録から） |
| 3912–3929 | 指令 ZMP、支持の箱での切り詰め、拒まれた圧力 | 制御 |
| 3931–4027 | contact_plan（終わりの時刻の決定、ずらし） | 制御（計画の書き換え） |
| 4028–4157 | CoM 予見の再計画（`PREVIEW_TRAJ_CSV`、`COM_PREVIEW_RUNTIME_CSV` をここで書く） | 制御＋診断 |
| 4158–4189 | 水平の CoM 加速度（`VH_DCM` を毎 tick 読む、ジャーク制限） | 制御 |
| 4190–4309 | 高さ参照（段の追従、降段の先行、段ゾーンのしゃがみ、スクワット、速度・押しの重心下げ）、`a_com` | 制御 |
| 4311–4958 | 重心のレンチ MPC（荷重予定、接触、交差の門、着地変数、入力、設定、`MPC_DIAG_DIR`、解、`MPC_RUNTIME_CSV`、`MPC_TRAJ_CSV`、着地の適用） | 制御＋診断 |
| 4959–5063 | CoM 段、胴（ヨーの計画、`TURN_IN_DS` を毎 tick 読む） | 参照 → Walker、段 → Stack |
| 5065–5135 | 角運動量（`arms_neutral`、`MOM_ARM_DCM`） | 同上 |
| 5137–5182 | MOM_CMP（段ゾーンでは off） | 同上 |
| 5184–5261 | 腕の保持（突き） | 同上（突きは指令の表から） |
| 5262–5273 | 姿勢 | Stack |
| 5275–5348 | 重力補償、荷重の分け方、名目の力（MPC の権限）、正則化 | 分け方は Walker、組み立ては Stack |
| 5350–5755 | 遊脚（着地目標、段の遊脚の形、保持、参照の遊脚、探りの深さ、姿勢の軸） | 目標は Walker、段は Stack |
| 5757–5789 | 物理行の控え（QP 診断・門用） | Stack |
| 5790–5916 | 段の並べ方、weighted の重み（`WBCW_*` を毎 tick 読む） | Stack |
| 5918–5938 | 構造が変わったら解法をリセット、解く | Stack |
| 5940–5961 | `WBC_LEVEL_DEBUG_T` | Stack（デバッグ） |
| 5962–6010 | 物理違反の門（`QP_BENCH_GATE`） | Stack |
| 6011–6024 | 胴の達成した加速度（診断列） | Stack → 記録 |
| 6025–6070 | `QP_DIAG_DIR` のダンプ | Stack（診断） |
| 6071–6119 | 遊脚の残差、脚の届く範囲（診断列） | Walker → 記録 |
| 6120 | `tally.observe`（`[degraded]` を印字） | ハーネス |
| 6121–6188 | 解の取り出し、足ごとの CoP、正味の CoP | Stack → 記録 |
| 6189–6254 | `TRANSFER_DIAG_CSV` | 診断 → ハーネス |
| 6255–6428 | 押しの後の計測（CoP、すべり、移行、`FIX_*` の撃力 6320–6343、角運動量） | 計測・シナリオ → ハーネス |
| 6429–6486 | 関節の振れ幅、自己衝突・的の当たりの監視（`[self-collision]` を印字） | ハーネス |
| 6488–6525 | トルクの取り出し、`CommandPolicy`（`[OPEN LOOP]`／`[recovered]` を印字）、トルク上限、使用率 | 方策 → ハーネス（当面） |
| 6526–6578 | 足首ロールの使用率、歩ごとの集計（植物を読む） | 計測 → ハーネス |
| 6580–6586 | 胴の傾き（診断列） | 記録から |
| 6588–6662 | **押しの発火**（`apply_external_force`、`PUSH at` を印字） | シナリオ → ハーネス |
| 6664–6695 | **押し回復の参照の終了と再センタリング**（計画の書き換え、`[ref]` を印字） | 制御 → `Walker::end_tick` |
| 6697–6802 | 押しの力（再生用）、`QP_RUNTIME_CSV`、`QP_WRENCH_CSV` | 診断 → ハーネス |
| 6803–6814 | `ACT_DELAY_TICKS`、`write_to_plant` | ハーネス |
| 6815–6878 | `rig.step`、サブステップ・tick の監査、`ANKLE_CBF_DIAG_CSV` | ハーネス |
| 6880–7077 | TRAJ_CSV の行（`MOM_DIAG_CSV`、`GAIT_DUMP_T` を含む） | ハーネス（記録から） |
| 7079–7158 | 移動量、最低高さ・傾き、40 tick ごとの印字、転倒の判定（`break`） | ハーネス（傾きは Walker の推定値） |
| 7160–7563 | 歩ごとの表、結果、カウンタ、押し・移行・判定の報告 | ハーネス（カウンタは `walker.stats()`） |

### 1.4 ループの状態（`let mut` で tick をまたぐもの）

ループの前の最上位の `let mut` は 170 個（`footsteps` の重複を含む）。ループの中の `let mut` は tick ごとの一時変数で、tick をまたがない。

| 分類 | 個数 | 中身 | 行き先 |
|---|---:|---|---|
| 計画 | 5 | `gait`、`footsteps`、`dcm_plan`、`height_plan`、`landing_timing` | `Plan`（S6） |
| 制御の状態 | 42 | `push_yield_done`、`vx2_z_track`、`push_z_drop_until`、`anchors`、`correction`、`terrain_track`、`cp_applied`／`cp_slice`／`cp_retimed_slice`、`last_scheduled_admit`／`admit_measured_z`／`confirmed_admit`、`ref_lib`／`ref_cooldown_until`／`ref_lat_quiet_t`／`ref_mode`、`dsc_retimed_last`、`push_replan_state`／`_slice`／`_shift`、`mpc_fs_cross_latched`、`replan_engaged`、`preview_p`／`_c`／`_v`／`_next`、`prev_support`、`swing_lift_off`、`fz_prev`／`fz_rate_ema`／`managed_until`、`previous_a_xy`、`wrench_mpc_next`／`wrench_mpc_command`、`mpc_fs_slice`／`_applied`／`_armed`／`_last`、`solver`、`qp_previous_structure`、`qp_has_good_command`、`policy` | `CtlState`（S5）。`solver`／`qp_previous_structure`／`anchors` は Stack、`policy`／`qp_has_good_command` はハーネス |
| 制御のカウンタ | 25 | `n_mom_cmp_ticks`、`mom_cmp_peak`、`n_push_z_drop`、`n_cp_shift`／`n_cp_retime`／`cp_retime_total`、`n_confirmed_admissions`、`ref_engagements`、`n_ds_capture`／`ds_capture_total`、`n_ds_feasible`／`n_ds_continuous`／`ds_feasible_total`、`n_push_replan`、`n_mpc_fs_cross`、`preview_count`／`_failures`／`_error`、`landing_retimes`、`landing_delay`、`n_replan`、`n_corrected`、`wrench_mpc_solves`／`_failures`、`tally` | `WalkerStats`（S5）。`tally` はハーネス |
| 制御の診断値 | 7 | `diag_mpc_reference`、`diag_mpc_time`、`diag_mpc_net_cop`、`diag_wbc_net_cop`、`trunk_cmd_rp`、`trunk_acc_rp`、`wrench_mpc_frames` | `TickRecord`（S8） |
| 計測・判定 | 67 | `step_stats`、走行の 16 個（`walk_end_state` 〜 `q_hi`）、的の当たり 9、自己衝突 2、`push_*` 12、`win_*` 10、移行 13（`transfer_t` 〜 `xfer_stance`）、`push_ankle_max`／`push_exceeded`／`push_last_exceed`／`t_last`、`fix_fired` | `RunMetrics`（S3、ハーネス） |
| 診断の出力先 | 18 | `qp_diag_count`、`qp_runtime_log`、`mpc_runtime_log`、`mom_diag_log`、`qp_wrench_log`、`substep_*` 4、`log`、`tick_audit`、`com_preview_runtime`、`transfer_diag`、`ankle_cbf_diag`、`preview_traj_file`、`mpc_traj_file`、`mpc_diag_count`、`act_delay_buf` | `DiagSinks`（S4、ハーネス）。制御の中で書くもの 5 本は §2.4 |
| 起動時だけ | 6 | `o`、`prof`、`rig`、`cfg`（`SolveConfig`）、`jlim_v_max`、`footsteps`（再束縛） | — |

制御の状態 42 とカウンタ 25 と計画 5、合わせて 72 個が Walker の中身になる。計測の 67 個はハーネスに残る。

### 1.5 環境変数の読み

walk.rs の中の読みの呼び出し数は、`env_f64(` 263、`flag(` 69、`std::env::var(` 68、`set_var` 1。このうち**ループ内**のものは次のとおり。

- 制御のノブ 5 個: `LOAD_RAMP_FRAC`（3431）、`VH_DCM`（4162）、`MPC_KKT_COMP_TOL`（4787）、`TURN_IN_DS`（5009）、`WBCW_<段名>`（5912）
- デバッグ 10 か所: `CONTACT_ADMIT_DEBUG`（2974、3337、3371）、`DS_FEASIBLE_DEBUG`（3083、3099）、`DEBUG_ANCHOR`（3273）、`CP_DEBUG`（3957）、`MPC_FS_DEBUG_LANDING`（4914）、`MPC_TRAJ_DEBUG`（4935）、`WBC_LEVEL_DEBUG_T`（5943）

ライブラリ側でも読んでいる。`crates/misa-biped/src/preview.rs` の `LAND_TIMING_DEBUG`（558、613、637、677）と、`crates/misa-mpc/src/wrench_mpc.rs` の `MPC_LANDING_TIME_DEBUG`（891）・`MPC_FS_DEBUG`（1707、1805、1821）で、どれもデバッグ用。台帳 §0 にある `preview.rs` の `LAND_CONFIRM_XY/Z` は、今は walk.rs の 2029–2030 で読んでいて、ライブラリからは消えている（台帳の記述が古い）。

## 2. Walker の API 案

### 2.1 形

```rust
// crates/misa-biped/src/walker.rs
pub struct Walker<S: WbcStack> {
    cfg: Config,            // 起動後は不変（§4）
    robot: RobotModel,      // misarta の模型＋関節の対応（§2.6）
    world: World,           // 既知の地形の模型と指令の表
    plan: Plan,             // §3
    ctl: CtlState,          // 1.4 の「制御の状態」から Stack 行きを除いたもの
    stats: WalkerStats,     // 1.4 の「制御のカウンタ」
    rec: TickRecord,        // 直近の tick の記録（§2.4）
    stack: S,               // WBC の段（§2.6）
    dt: f64,                // 実現した制御周期（= substeps × sim dt。Config の CTRL_DT とは別）
}

pub struct WalkerSetup {
    pub dt: f64,
    pub t_end: f64,                 // 時間割の終わり。今は GaitParams.t_end = T（シナリオの長さ）
    pub terrain: Option<Stairs>,    // 既知の地形。段差 0 の「段」も Some のまま渡す（§6.2）
    pub command: CommandSchedule,   // VX/VY/WZ、VX2/VX2_AT/VX2_RAMP、SQUAT_*、JAB_*
    pub recovery_refs: Vec<RefLib>, // REF_LIBRARY を読んだもの（読み込みはハーネス）
}

impl<S: WbcStack> Walker<S> {
    /// 今の 1114–1723 行（初期の足跡、時間割、速度表、段の足跡、COM_PREPLAN、DCM 計画）と、
    /// 1724–2885 行の制御の状態の初期化。印字は `out` へ即時に出す（順を保つ）。
    pub fn new(cfg: Config, robot: RobotModel, setup: WalkerSetup,
               init: &Observation, stack: S, out: &mut dyn LogSink) -> Self;

    /// 1 tick の制御。2891–2917、2950–6119、6121–6188 行に当たる。
    pub fn step(&mut self, obs: &Observation, out: &mut dyn LogSink) -> Command;

    /// tick の終わりの制御。今の 6664–6695 行（押し回復の参照の終了と再センタリング）。
    /// ハーネスが押しを発火させた後に呼ぶ。
    pub fn end_tick(&mut self, out: &mut dyn LogSink);

    pub fn record(&self) -> &TickRecord;
    pub fn state(&self) -> &RobotState;    // 今の tick の運動学・動力学（S14 以降は Walker が計算したもの）
    pub fn stats(&self) -> &WalkerStats;   // 最終報告のカウンタ（7177–7290 の読み）
    pub fn plan(&self) -> &Plan;           // GAIT_DUMP、報告の landing_timing.touchdowns など
    pub fn config(&self) -> &Config;
}

/// 印字の出口。ハーネスは println! で実装する。即時に出すので行の順は今と同じになる。
pub trait LogSink { fn line(&mut self, args: std::fmt::Arguments<'_>); }
```

### 2.2 Observation（観測）

```rust
pub struct Observation<'a> {
    pub tick: usize,                      // Walker は t = tick as f64 * dt で時刻を作る（今と同じ式）
    pub estimate: Estimate<'a>,
    pub contact_fz: [f64; 2],             // 足ごとの接触力の鉛直成分（今の measured_pre）
    pub joint_pos_plant: &'a [f64],       // 過渡: 植物の関節角（§6.6）
}

pub enum Estimate<'a> {
    /// S10〜S13: rig.sync() が組み立てた状態をそのまま渡す（misa-biped へ移した RobotState）。
    State(&'a RobotState),
    /// S14 以降: 状態推定の結果だけを渡し、Walker が運動学・動力学を計算する。
    Raw(&'a StateEstimate),
}

pub struct StateEstimate {
    pub base_pos: [f64; 3],
    pub base_quat: na::UnitQuaternion<f64>,
    pub base_lin_vel_w: [f64; 3],
    pub base_ang_vel_w: [f64; 3],
    pub joint_q: Vec<f64>,                // articara の関節順（rig.robot.joints の順）
    pub joint_qd: Vec<Option<f64>>,       // QD_CLAMP はリグ（推定器）側で済ませておく
}
```

- 接触力について。制御が `MeasuredContact` から読むのは `f_w[side][2]` だけで、次の箇所に使っている。
  - 着地の時計（2962）
  - 予定・荷重 admit（3332）
  - 管理接触の EMA（3289–3300）
  - DS_CAPTURE の着地荷重（3151 の `fz_prev`）
  - MPC の平滑化の種（4423）
- `measured`（3301 で写したもの）は、歩ごとの集計・ログ・押しの計測にしか使っていない。`sole_wrench` は `QP_WRENCH_CSV` 専用で、ハーネスの診断にあたる。
- `joint_pos_plant` は今の `rig.robot.joint_positions`。センサの遅れや雑音の入らない真の値で、`bt::posture`（5250、5266）、`gravity_plus_posture`、Hybrid の書き込み、初期の FK（`rig.fk_now()`）が読んでいる。
  - 推定の `q` とは別物なので、一致を保つには両方渡す必要がある。
  - 見本の `FAM_nh10_sense` は `SENSE_DELAY_TICKS=8` なので、ここで実際に差が出る（§6.6、§7）。

### 2.3 Command（WBC の解）

```rust
pub struct Command {
    pub tau: Vec<f64>,                  // 関節順の QP トルク（劣化時の方策・上限の切り詰めの前）。今の 6488–6491 の robot_taus
    pub qddot: na::DVector<f64>,        // Velocity/Servo/Hybrid の書き込み用（extracted.qddot）
    pub status: misa_wbc::SolveStatus,
    pub rejected: bool,                 // QP_BENCH_GATE の物理違反で棄却（今の qp_rejected）
    pub tau_gravity: na::DVector<f64>,  // 劣化時の代替（gravity_plus_posture）の材料（5276）
    pub n_contacts: usize,              // nc（方策・集計が使う）
}
```

- 劣化時の方策（`CommandPolicy`）と `DegradedTally` は**当面ハーネスに残す**。理由は 2 つある。
  - `src/biped/actuate.rs` に他の人の未コミットの差分がある。
  - ハーネス側に置けば、`[degraded]`（6120）→ `FIX at`（6339）→ `[self-collision]`（6470）→ `[OPEN LOOP]`（方策）という今の印字の順が、何もしなくても保てる。
- 方策を Walker（または misa-biped の独立部品）へ移す時期は §7 の未決にした。

### 2.4 TickRecord（ハーネスが覗く内部量）

`step()` が毎 tick 上書きする構造体。ハーネスは TRAJ_CSV の `extra` 列（6912–7040）、`Row`（7053–7076）、40 tick ごとの印字（7137–7148）、歩ごとの集計、押しの計測、転倒の判定、診断 CSV を、すべてここから読む。

| まとまり | 項目（今の変数名） |
|---|---|
| 時刻と時間割 | `t`、`slice_idx`、`slice_frac`、`slice`（`t1` を押しの警告が読む）、`support`、`step_idx`、`stance_sides`、`nc`、`steps`（その区間の計画の足裏）、`actual_feet`（FK の足裏）、`free_side` |
| 釣り合い | `xi`、`r`（DCM 参照: `xi`、`xi_dot`、`p`）、`dcm_err`、`p_raw`、`p_cmd`、`clamped`、`zmp_deficit`、`a_com`、`z_ref`、`shares` |
| 胴 | `roll`、`pitch`（推定値。転倒の判定 7129 もこれを使う）、`rp_ref`、`trunk_cmd_rp`、`trunk_acc_rp`、`trunk_rot`（`data.oMi[trunk_mi]` の回転。6580–6586 の傾きの列用） |
| 遊脚 | `swing_tgt`、`swing_cmd_acc_z`、`swing_res_z`、`swing_res_rows`、`swing_task_residual_rms`、`swing_pos_err_xy/z` |
| 予見 | `replan_engaged`、`preview_c`、`preview_v`、`preview_error` |
| MPC | `wrench_mpc_command`、`diag_mpc_reference`、`diag_mpc_time`、`diag_mpc_net_cop`、`mpc_fs_last`、`wrench_mpc_frames`、`mpc_solve_ns`、`mpc_capture_ns` |
| WBC | `cop_qp`、`f_qp_w`、`cop_use`、`diag_wbc_net_cop`、`acc_dbg`、`level_names`、`qp_solve_ns`、`qp_check_ns`、`qp_capture_ns`、`qp_physical_violation`、`qp_reset`、`forces_nominal`（TRANSFER_DIAG 用） |
| 接触 | `anchor_pos[side]`（係留点、Stack から）、`fz_meas` |
| 届く範囲 | `reach_used`、`hip_z`、`stride_max_now` |
| 状態 | 記録には写さず、`walker.state() -> &RobotState` で今の tick の状態を貸す。押しの計測が CMM（6306、6345）を、TRAJ_CSV が FK（6892–6902）を、`ANKLE_CBF_DIAG` が `q`／`v` を読む |

- 制御の中で書いている診断ファイル 5 本は、記録に載せると重い。
  - `PREVIEW_TRAJ_CSV`（4122）、`COM_PREVIEW_RUNTIME_CSV`（4141）、`MPC_DIAG_DIR` のダンプ（4791）、`MPC_RUNTIME_CSV`（4825）、`MPC_TRAJ_CSV`（4865）が該当する。
  - これらは任意のフック `Option<Box<dyn WalkerDiag>>` で出す。メソッドは `on_preview(t, &samples, &plan)`、`on_mpc(t, &input, &config, &solution, landing)` など。
  - フックが None のときは何もしない。
- `QP_DIAG_DIR` のダンプは段の中身が要るので、Stack の実装（articara）で出す。
- `QP_RUNTIME_CSV` の `control_ns` は、今は tick の頭（`rig.sync` を含む）から測っている。分けた後はハーネスが `step()` を挟んで測ることになるので、意味が少し変わる。時間の列は比較の対象外なので、一致の確認には影響しない。

### 2.5 ハーネスの 1 tick（最終形）

```text
1. t = tick*dt。押しの窓の計測（今 2918–2927）
2. st = rig.sync(); fz = measure_contacts(..); CONTACT_DUMP_T の印字
3. cmd = walker.step(&obs, &mut sink)        // 制御の印字はここで即時に出る
4. tally.observe(..)                         // [degraded]
   計測（CoP、押し、移行、FIX の撃力）、TRANSFER_DIAG、自己衝突の監視
   policy.apply(..)、上限の切り詰め、使用率  // [OPEN LOOP] / [recovered]
   足首ロール、歩ごとの集計、押しの発火     // PUSH at
5. walker.end_tick(&mut sink)                // [ref] reference over
6. 押しの力、ランタイム CSV、QP_WRENCH_CSV、ACT_DELAY、write_to_plant、rig.step、
   監査、TRAJ_CSV、移動量、40 tick の印字、転倒の判定
```

`step()` の中身（2891–2917、2950–6119、6121–6188）は、今のコードでも計測・シナリオの印字より前にある。例外は 2 つだけで、`tally.observe` は Walker に入れない（ハーネスでも今と同じ位置で呼べる）、参照の終了は `end_tick()` に分ける。この 2 つを外せば、**印字の順は今と完全に同じ**になる。2891–2917 の着地の設定は今 `rig.sync()` より前にあるが、印字も植物の読みもしないので、step() の頭へ動かしても結果は同じ。

### 2.6 WBC の段（tasks.rs・contact.rs）と misarta の動力学の置き場

#### 動力学・運動学 → misa-biped の `RobotModel`／`RobotState`

`rig.sync()`（`src/biped/rig.rs` 942–1228）は 2 つに分かれる。

- 942–1072: 植物を読み、センサの模型（遅れ、雑音、偏り、低域通過、加速度計との融合）を通す。MuJoCo に依存する。
- 1073–1228: `q`／`v` を組み（`QD_CLAMP`）、`crba`／`rnea`、アーマチュア・粘性の補正、FK、CoM とそのヤコビアン・バイアスを計算する。純粋に misarta だけを使う。

後半を `misa_biped::robot::RobotModel::assemble(&self, est) -> RobotState` に移し、`BipedState` は `RobotState` の別名にする。不採用の `BASE_VEL_KIN`（1112–1150）は MuJoCo の接触を読むので、リグ側に残す。ただし後半の途中で `v` を直すので、S14 で推定と運動学をきれいに分けるには先に削る必要がある（§7）。

```rust
pub struct RobotModel {
    pub model: misarta::model::Model<f64>,
    pub prof: Profile,                          // sole_centre_x, sole_below_origin, foot_links, …
    pub joint_names: Vec<String>, pub joint_map: HashMap<String, usize>,
    pub a2m: Vec<Option<usize>>, pub actuated: Vec<(usize, usize)>,   // rig.actuated()
    pub foot_mi: [usize; 2], pub trunk_mi: usize, pub trunk_from_base: bool,
    pub hip_mi: Option<[usize; 2]>,
    pub mass_links: Vec<MassLink>, pub total_mass: f64,
    pub armature: Vec<f64>, pub joint_damping: Vec<f64>,              // 関節順。植物と同じ値
    pub torque_max: na::DVector<f64>, pub q_min: na::DVector<f64>,
    pub q_max: na::DVector<f64>, pub v_max: na::DVector<f64>,
    pub q_seed: Vec<f64>, pub nv: usize, pub na: usize,
}
impl RobotModel {
    pub fn assemble(&self, est: &StateEstimate, qd_clamp: f64) -> RobotState; // rig.rs 1082–1228 をそのまま
    pub fn gravity_torque(&self, q: &[f64]) -> na::DVector<f64>;           // rig.rs 1230
    pub fn side_of(&self, foot_mi: usize) -> usize;
}
```

ループの制御の区画が `rig` から読んでいるのは、`model`、`a2m`、`q_idx`、`foot_mi`、`trunk_mi`、`trunk_from_base`、`actuated()`、`q_seed`、`robot.joints[].name`、`robot.joint_map`、`robot.joint_positions`、`gravity_torque`、`torque_max`、`side_of` だけ（`rig.sim` は計測とシナリオの区画でしか読まない）。だから上の構造体で足りる。ハーネスは `BipedRig::walker_model(&self) -> RobotModel` で作る。rig.rs は今、未コミットの差分が無い。

CMM（`compute_centroidal_momentum_matrix`）は今 1 tick に 3 回計算している（MPC 4475、角運動量 5075、MOM_CMP 5152）。純関数なので 1 回にまとめても値は変わらない。`Footsteps::from_fk(data, …)` も 5 回（2954、3789、4029、4397、6892）計算していて、同じく 1 回にできる。どちらも S11 でやる。

#### WBC の段 → trait で注入（4 案）

| 案 | 形 | 長所 | 短所 |
|---|---|---|---|
| **A（推奨）粗い trait** | `trait WbcStack { fn solve(&mut self, req: &WbcRequest, out: &mut dyn LogSink) -> WbcSolution; fn anchor_pos(&self) -> [Option<Vector3>; 2]; }`。articara に `TaskStack`（tasks.rs・contact.rs・`misa_wbc::Solver` を使う）を置き、`Walker<TaskStack>` | Walker は tasks.rs の関数の形を知らない。状態（`Solver` の温め、QP の構造、`Anchors`）を Stack が持てる。tasks.rs・contact.rs を移すときは `TaskStack` ごと misa-biped へ移し、既定の型引数にすれば済む | `WbcRequest` が大きい（下）。段の並べ方（`MOM_LEVEL`、`ARM_BEFORE_SWING`）という制御の判断が、当面 Stack 側に入る |
| B 細かい trait | tasks.rs の関数ごと（`com`、`trunk_rpy`、`angular_momentum`、`posture`、`swing_with_pose*`、`contact_level`、`regulariser`、`force_nominal`）にメソッドを持つ trait。段の並べ方は Walker | 段の並べ方が Walker に入る | メソッドが 10 を超え、tasks.rs の引数の変更がそのまま trait に響く。`Anchors` を関連型にする必要がある |
| C 今すぐ移す | tasks.rs・contact.rs を misa-biped へ移し、trait を設けない | 一番単純 | contact.rs に他の人の差分（`Anchors::bias_z`）がある。依頼の前提に反する |
| D 二段の呼び出し | `walker.plan(&obs) -> WbcRequest` → ハーネスが articara の関数で解く → `walker.finish(&sol) -> Command` | 型引数が要らない | どのランナーも配線を書くことになる。tick の内部の段がハーネスに漏れる |

**推奨は A**。tasks.rs・contact.rs は MuJoCo に依存していない（`misa_wbc`、`misarta`、`nalgebra` だけ）ので、移動を妨げているのは他の人の差分だけ。trait は、その移動までの継ぎ目であり、移動後も「別の QP を使う実機ランナー」の差し替え口として残る。関数の注入（`Box<dyn FnMut(&WbcRequest) -> WbcSolution>`）は A と同じ契約だが、`Solver` の温めと `Anchors` の状態を閉包の捕捉で持つことになるので、trait の方が素直。

```rust
pub struct WbcRequest<'a> {
    pub t: f64, pub tick: usize, pub dt: f64,
    pub robot: &'a RobotModel, pub state: &'a RobotState,
    pub support: Support, pub stance_sides: &'a [usize],
    pub anchor_events: &'a [AnchorEvent],          // Release(side) / Touchdown(side, p, R)。今の 3229・3281・3401 の順
    pub load_share: [f64; 2],                      // f_share(side).max(0.02)（3435–3452）
    pub contact_mode: [ContactMode; 2],            // 管理接触 Managed{..} / Rigid（3501–3517）
    pub cop_patch: [Option<([f64; 3], (f64, f64))>; 2], // 斜面の踵の切り取り（3518–3553）
    pub a_com: na::Vector3<f64>,
    pub trunk: TrunkRef,                           // ori（rp と yaw の誤差）、yaw_ref の有無（5041–5063）
    pub momentum: Option<MomentumRef>,             // arms_neutral、dh_ref、kp、axes、coupled（5074–5135）
    pub mom_cmp: Option<[f64; 3]>,                 // dh（5148–5182）
    pub arm_hold: Option<ArmHoldRef>,              // 行の集合と q_ref、kp/kd（5215–5261）
    pub nominal: NominalForces,                    // shares、MPC のレンチ、権限、混合率（5282–5333）
    pub coupled: bool,                             // 連成段（採用形では閉じている）
    pub swing: Option<SwingRef>,                   // Planned{tgt,tgt_v,tgt_a,axes,rot_tgt} / Recover{tgt,axes,rot_tgt}
}
pub struct WbcSolution {
    pub status: misa_wbc::SolveStatus, pub rejected: bool,
    pub tau: na::DVector<f64>, pub qddot: na::DVector<f64>, pub forces: na::DVector<f64>,
    pub tau_gravity: na::DVector<f64>,
    pub level_names: Vec<&'static str>,
    pub sole_wrench: [Option<na::DVector<f64>>; 2],   // p0.sole_sel × forces（CoP の列用）
    pub acc_dbg: [f64; 2], pub trunk_acc_rp: [f64; 2],
    pub swing_residual: SwingResidual, pub physical_violation: f64, pub qp_reset: bool,
    pub timings: QpTimings,
}
```

Stack の中身は、今の 3454–3555、4959 の CoM 段、5056–5063、5074–5182 の段の構築、5215–5348 の段の構築、5361–5754 の段の構築、5757–6070 と、6121–6188 の取り出しになる。参照（数値）の計算は Walker に残す。今の遊脚の区画は、目標の計算（5376–5658）と段の構築（5659–5709）が 1 つの `match` の中で混ざっている。S11 で目標を先に計算する形にほどいてから、S12 で段の構築を Stack へ移す。

## 3. 計画の状態の束ね方

### 3.1 今の書き換え手（tick の順）

| # | 書き手 | 行 | `gait` | `footsteps` | `dcm_plan` | `landing_timing`・`height_plan` | 他の書き手との取り決め（今の門） |
|---|---|---|---|---|---|---|---|
| 1 | 着地の延長の設定 | 2891–2917 | | | | `max_extension`、`xy_tol_override`、`event_end_on_touch` | `cp_retimed_slice == Some(i)` なら延長 0（#10 が終わりを持つ） |
| 2 | 着地の時計 | 2962、2992–3004 | `update(&mut gait, …)` が時刻を延ばす・確定する | | 再構築 2997（shift≠0 か timeout） | | |
| 3 | 接触イベントのずらし | 2978–2991 | | `shift_next` 2983 | 2984 | | 参照の作動中だけ届く（`event_mode`） |
| 4 | DS_FEASIBLE | 3005–3103 | `retime` 3055、3088 | | 3058、3091。区間 ZMP の上書き 3075、3079 | `preview_next = t` | |
| 5 | DS_CAPTURE | 3105–3177 | `retime` 3167 | | 3171 | | `ref_mode` 中は off。`dsc_retimed_last` で 1 区間 1 回 |
| 6 | 再計画の開始 | 3180–3192 | | | | `height_plan` 作り直し 3188 | 1 回限り |
| 7 | PUSH_YIELD | 3581–3633 | `retime` 3627 | `at[j].sole[side].x -=` 3619 | 3629 | | 1 回限り（`push_yield_done`） |
| 8 | PUSH_REPLAN | 3634–3751 | | 置き換え 3732 | 3733 | | `ref_mode` 中・段ゾーン・斜面の手前は off。`push_replan_slice` |
| 9 | 参照の作動 | 3771–3884 | `retime` 3836 | `shift_next` 3866 | 3873 | `event_mode = true` 3874 | `ref_mode` が立つと #5・#8・#10・#12 と PUSH_Z_DROP を止める |
| 10 | contact_plan | 3932–4027 | `retime` 4006 | `shift_from` 3997（チャンピオンでは常に 0） | 4017 | `max_extension = 0` 4013 | `landing_closed(i)` ならずらさない、`ref_mode` 中は off、`cp_retimed_slice` |
| 11 | 高さの計画 | 4270–4272 | | | | `height_plan.retarget` | |
| 12 | MPC の着地 | 4903–4934 | | `shift_next` 4922 | 4924 | | `push_replan_slice == Some(i)` なら off、`landing_closed`、`ref_mode` 中は off |
| 13 | 参照の終了 | 6664–6695 | | `shift_from` 6689 | 6692 | `event_mode` を戻す 6670 | |

起動時の書き換えもある。`insert_ds_settles` 1196、速度の変化の `retime` 1325、足跡 1364、段の作り直し 1561・1609・1618・1621、`COM_PREPLAN` の `retime` 1658、DCM 1680 の 8 か所。DCM の感度の試し（4728）は `dcm_plan` を書かずに別の計画を作る。

台帳 §4.3 の「`footsteps` を書く箇所 21、`dcm_plan` 31、`gait` 28」は削除前の数え。今はループの中で、`footsteps` 6、`dcm_plan` の再構築 11、`gait` 7 にまで減っている。それでも、同じ tick の中で 13 の手が順に書き、互いの門で守り合う構造は変わっていない。

### 3.2 束ねる形

```rust
// crates/misa-biped/src/plan.rs
pub struct Plan {
    pub gait: GaitPlan,
    pub footsteps: FootstepPlan,
    pub dcm: DcmPlan,
    pub height: HeightPlan,
    pub landing: LandingTiming,
    dcm_params: DcmParams,      // z_lip, zmp_lat_scale, zmp_roll
    claims: SliceClaims,        // S16
}
impl Plan {
    pub fn rebuild_dcm(&mut self);                         // = DcmPlan::from_footsteps_rolled(&gait,&footsteps,z_lip,lat,roll)
    pub fn dcm_for(&self, fs: &FootstepPlan) -> DcmPlan;   // 感度の試し（4728）
    pub fn edit(&mut self, by: Writer) -> PlanEditor<'_>;  // S16
}
```

**不変条件**: `dcm` は `(gait, footsteps, dcm_params)` の純関数で作り、そこに DS_FEASIBLE の区間 ZMP の上書き（`set_segment_zmp*`）が乗る。上書きは次の再構築で消える（#4 の後に #7〜#13 のどれかが再構築すると消える。今の挙動で、意図かどうかは §7）。

#### 1 段目（S6、機械的）

5 つの変数を `Plan` のフィールドに移し、ループ内の 11 か所と起動時の 1 か所の再構築を `plan.rebuild_dcm()` に置き換える。再構築を遅らせる（汚れの印を付けて後でまとめて作る）ことはしない。今のコードは、時間割を書き換えてから再構築するまでの間に古い `dcm_plan` を読む箇所を持っている（例: 3006–3010 は 2997 の再構築の後、3124 は DS_CAPTURE の再構築の前）。再構築の位置を 1 つでも動かすと一致が崩れる。

#### 2 段目（S16、Walker が misa-biped に入った後）

```rust
pub enum Writer { LandingClock, ContactEvent, DsFeasible, DsCapture, PushYield,
                  PushReplan, RecoveryRef, ContactPlan, MpcLanding, RecoveryEnd }

pub struct PlanEditor<'a> { plan: &'a mut Plan, by: Writer, touched: bool }
impl PlanEditor<'_> {
    pub fn retime(&mut self, slice: usize, t1: f64) -> f64;
    pub fn shift_next(&mut self, slice: usize, side: usize, d: Vector2<f64>);
    pub fn shift_from(&mut self, slice: usize, side: usize, d: Vector2<f64>);
    pub fn replace_footsteps(&mut self, fs: FootstepPlan);
    pub fn commit(self) -> bool;            // touched なら rebuild_dcm、Writer ごとのカウンタを進める
}
// 区間ごとの持ち主（今の 7 種類の門を集める）
pub struct SliceClaims {
    end_time: Option<(usize, Writer)>,      // cp_retimed_slice（#10）
    landing: Option<(usize, Writer)>,       // push_replan_slice（#8）、landing_closed（イベントの探り）
    whole: Option<Writer>,                  // ref_mode（#9 の作動中は他の再計画を全部止める）
    once: WriterSet,                        // push_yield_done、dsc_retimed_last
}
```

- 書き手ごとの再構築の回数（`n_replan`、`n_cp_shift`、`n_push_replan` など）も、`commit()` で数えれば散らばったカウンタが 1 か所に集まる。
- 取り決めの判定は、まず**写し**として持つ。古い判定と新しい判定を両方計算し、`assert_eq!` で一致を確かめる。この assert は確認用の cargo feature の下に置き、release で走らせる。96＋14 本で一致を見てから、古い判定を消す。
- DS_FEASIBLE の区間 ZMP（`set_segment_zmp*`）は、`commit()` の後に `plan.dcm` を直接書く経路として残す。これを計画の状態（再構築でも消えない上書き）にするのは挙動の変更なので、§7 で判断を待つ。

これが台帳 §4.3 の「書き換えの経路が多すぎる」への答え。書き方は `PlanEditor` の 5 種類だけになり、誰が書いたかが名札で残り、区間の持ち主は `SliceClaims` 1 か所で決まる。

## 4. Config の構造

### 4.1 サブ構造体（台帳 §2 の 24 束＋α）

| サブ構造体 | 主な項目（今の環境変数） | 実行時に決まる既定 |
|---|---|---|
| `WbcConfig` | `feas_tol`、`strategy`（weighted）、`fallback_level`、`hold_kp/kd`、`hold_bridge`、`backend`、`weight_ratio`、`f_max(_scale)`、`friction_mu`、`eom_hard`、`level_weights`（`WBCW_*`、段の列挙型ごと） | `friction_mu` = `MU_GROUND × prof.friction_margin`（§6.7） |
| `JointLimitConfig` | `JLIM*` | `v_max` は `RobotModel` から |
| `BalanceGains` | `KP/KD_COM`、`KP/KD_TRUNK`、`KP/KD_POST`、`POST`、`TRUNK(_SIGN/_YAW)`、`KP/KD_YAW`、`DIAG_VZ_BIAS_*`（診断の偏り） | |
| `DcmConfig` | `COP_FRAC`、`K_DCM`、`VH_DCM`、`ZMP_ROLL`、`ZMP_MARGIN`、`CLAMP_ZMP`、`ZMP_LAT_SCALE`、`COM_XY_JERK_MAX` | `zmp_roll`: 地形の模型があれば 0.030、無ければ 0 |
| `GaitConfig` | `T_START`、`T_DS`、`T_SS`、`N_STEPS`、`FIRST_SWING`、`DS_SETTLE(_AFTER_STEP)`、`STRIDE`、`STRIDE_RAMP`、`ALTERNATE`、`TURN_IN_DS`、`NO_LIFT` | |
| `StairPlanConfig` | `STAIR_TOE_MARGIN`、`STAIR_Y_OUT`、`STAIR_TD_BIAS*`、`STAIR_LANDING_FRAC`、`STAIR_ALIGN_STAND_LINE`、`STAIR_GAIT`、`STAIR_FULL_SOLE`、`STAIR_SOLE_MARGIN` | |
| `StairZoneConfig`（新） | `STAIR_ZONE_CROUCH_LEAD/TAIL`（降段のしゃがみ・MOM_CMP・PUSH_REPLAN・遊脚の回転モードが共有） | |
| `DescentConfig` | `STAIR_TD_BIAS_DESC`、`COM_HEIGHT_DESC_LEAD`、`STAIR_ZONE_CROUCH`、`COM_LAND_EXTEND_DESC`、`LAND_CONFIRM_Z/XY_DESC`、`LEG_REACH_MARGIN`、`STAIR_DESC_STAND_MARGIN` | |
| `SlopeConfig` | `SLOPE_KEEP_STRIDE`、`SLOPE_HEEL_CUT(_MAX)`、`TERRAIN_PITCH(_YAW)` | |
| `SwingConfig` | `KP/KD_SWING(_XY/_ROT)`、`LIFT_H`、`TRACKING_CUBIC_FF`、`TRACKING_FF_MIN_RISER/SCALE`、`STAIR_TD_SOLE_FRAME`、`STAIR_TD_SINK`、`STAIR_SWING_WINDOWS/FF`、`SWING_ROT`、`STAIR_ZONE_SWING_ROT`、`STAIR_LAND_SINK`、`STEP_DZ_SWING_MIN`、`STAIR_ABSOLUTE_TD(_DESC)`、`SWING_XY_DELAY`、`SWING_Z_HOLD(_MIN_DUR)`、`SWING_Z_DROP` | `lift_h` = `0.5 × prof.lift_h`。`ROUGH_MODE` の上書き（§4.4） |
| `ArmMomentumConfig` | `ARM_HOLD(_JOINTS)`、`ARM_BEFORE_SWING`、`KP/KD_ARM`、`MOM`、`KP_MOM`、`MOM_REF`、`MOM_AXES`、`MOM_LEVEL`、`MOM_ARM_DCM(_MAX)` | |
| `MomCmpConfig` | `MOM_CMP*` | |
| `ContactConfig` | `CONTACT_ADMIT_{CONFIRMED,SCHEDULED,LOADED,Z}`、`CONTACT_ON_BW`、`ANCHOR_LEAK`、`KP/KD_CONTACT`、`CONTACT_DRIVEN`、`CONTACT_TICKS`、`LOAD_RAMP_FRAC` | |
| `ManagedContactConfig`（残した 6） | `STAIR_MANAGED_{CONTACT,TANGENT,ROTATION,AZ_MAX,RATE,LATCH}` | |
| `LandingConfig` | `COM_LAND_EXTEND(_RISE)`、`COM_LAND_PREDICT_FRAC`、`LAND_CONFIRM_FZ_FRAC`、`LAND_CONFIRM_XY/Z`、`STEP_DZ_MIN`、`COM_LAND_VXY/VZ`、`event: ContactEventConfig`（残した 6: `CONTACT_EVENT(_Z/_SHIFT/_END_ON_TOUCH)`、`CONTACT_PROBE_VZ/_MAX`） | |
| `ComPlanConfig` | `Z_COM`、`Z_COM_OFFSET`、`COM_REPLAN(_FORWARD_GAIN/_DT)`、`COM_PREPLAN`、`COM_PREPLAN_XY_MIN`、`COM_STAIR_SS_MAX`、`STAIR_CLIMB_RATE`、`COM_PREVIEW_{DT,N,SMOOTH}`、`COM_HEIGHT_{REPLAN,TIME}` | `z_com` = 初期の CoM の高さ＋`Z_COM_OFFSET`。`com_preplan` = `COM_REPLAN` |
| `ContactPlanConfig` | `CONTACT_PLAN`、`CP_T_MIN/MAX`、`CP_DT`、`CP_T_FROM_X`、`CP_T_LAT_OUT`、`CP_W_T`、`CP_W_RES`、`CP_DEAD_T`、`CP_TRIG`、ずらし側の `CP_MAX_X/Y/Y_IN`、`CP_J`、`CP_W_SHIFT`、`CP_W_U`、`CP_DEAD_U`（§7） | `CP_MAX_Y_IN` = `CP_MAX_Y` |
| `DoubleSupportConfig` | `DS_FEASIBLE*`、`DS_CAPTURE*` | |
| `WrenchMpcConfig` | `enabled`（`STAIR_WRENCH_MPC`）、`always`、`apply`（複合ゲート、§7）、`MPC_NODE_CENTRE`、`MPC_NET_COP_W`、`MPC_SEED_PREVIOUS`、`MPC_W_*`、`MPC_SOLE_AUTHORITY`、`coupled`、`STAIR_WRENCH_{DT,N,PREVIEW_DT,LOW_MIN,BLEND}`、`STAIR_HYBRID_CONTACT`、`MPC_QP_BACKEND`、`MPC_KKT_COMP_TOL`、`MPC_BENCH_GATE` | |
| `MpcFootstepConfig` | `MPC_FOOTSTEP*`、`MPC_LANDING_CAPTURE_{W,J}`、`MPC_LANDING_LEX` | |
| `MpcCrossConfig` | `MPC_FOOTSTEP_CROSS_*` | |
| `PushReplanConfig` | `PUSH_REPLAN*`（`hold` は `Option<f64>`、§7） | |
| `PushCrouchConfig` | `PUSH_Z_DROP*` | |
| `SpeedScheduleConfig` | `VXS_*`、`VX2_TSCALE_MIN`、`VX2_SCHED`、`PUSH_YIELD_*` | |
| `RecoveryRefConfig` | `REF_TRIG_DCM`、`REF_TRIG_MAX_FRAC`、`REF_DELAY_COMP`、`REF_RESET_STANCE`、`REF_RESET_INCL_CURRENT`、`REF_SWING`、`REF_COOLDOWN`（ファイルの中身は `WalkerSetup.recovery_refs`） | |
| `DebugFlags` | ループ内の `*_DEBUG` 10 か所、`DS_CAPTURE_DEBUG`、`PUSH_REPLAN_DEBUG`、`MPC_FS_CROSS_DEBUG`、`DIAG_REPLAN`、`GAIT_DUMP_T` | |

Walker の Config に**入れない**もの。

- リグ: `RigOptions`、`SOLE_HALF_*`、`LEG_REACH`、`CTRL_MODE`、`SIM_DT`、`MU_GROUND`、`HIP_ROLL_SEED`、`ARM_PITCH`、`KNEE` など
- 推定器: rig.rs の `QD_CLAMP`、`BASE_VEL_FUSE_*`、`SENSE_*`
- シナリオ: `PUSH_*`、`FIX_*`、`STAIRS`、`TERRAIN_*`、`SPAR_*`、`T`、`ACT_DELAY_TICKS`、`PLANT_*`
- 指令の表: `VX`／`VY`／`WZ`、`VX2*`、`SQUAT_*`、`JAB*`
- 診断の出力先: `*_CSV`、`*_DIR`、`QP_CAPTURE_*`

ハーネス側に `HarnessConfig { rig, scenario, command, diag }` を置く。`CTRL_DT` は名目の値を `WbcConfig` に入れ、実現した `dt` は `WalkerSetup.dt` で渡す（台帳 §2.1 の「Profile か Config か」は、名目は Config、実現値はハーネス、と答える）。

### 4.2 実行時に決まる既定

既定値の中には起動時の文脈で決まるものがある。`Option<T>`（None は「文脈から決める」）で持ち、`Walker::new` の中の `Config::resolve(&self, &ResolveCtx)` で確定させる。

| 項目 | 文脈 |
|---|---|
| `ResolveCtx.prof` | `prof.*` 由来の既定（`CTRL_DT`、`LIFT_H`、`SOLE_HALF_*` など） |
| `ResolveCtx.terrain_modelled` | `stairs.is_some()`（`ZMP_ROLL`。今は `VH_DCM` の門も兼ねる） |
| `ResolveCtx.mu_ground` | `friction_mu` |
| `ResolveCtx.com0_z` | `Z_COM` |
| `ResolveCtx.slope_env` | `TERRAIN_SLOPE` が環境に**あるか**（`ROUGH_MODE_LIFT_H` の既定。パースした `Stairs` ではなく文字列の有無で決まっている） |

### 4.3 プリセット（G1 v6.19 と kyo46rs v2）

```rust
impl Config {
    pub fn code_defaults(prof: &Profile) -> Self;  // 環境変数が 1 つも無いときの今の値
    pub fn g1_v619() -> Self;     // code_defaults(g1) に log/claude_g1champion_20261029v619/controls.json の制御キーを載せたもの
    pub fn kyo46rs_v2() -> Self;  // code_defaults(kyo46rs) に log/claude_g1rand_20260925/kyo_base_v2.json の制御キーを載せたもの
}
// ハーネス: 環境変数でも JSON でも同じ写像を通す
pub fn config_from_kv(prof: &Profile, get: &dyn Fn(&str) -> Option<String>) -> Config;
```

- `Config::default()` は作らない。台帳 §0 のとおり、G1 チャンピオンを既定にすると kyo46rs の基準が変わる。
- プリセットは Rust の関数で持つ。確認のため、articara に単体試験を足す（`tests/` の既存ファイルは他の人の差分があるので新しいファイルにする）。試験は、`config_from_kv(g1, controls.json の読み)` と `Config::g1_v619()` が等しいことを見る。
- 運用（rand.py、family.sh、CEM）は今までどおり環境変数で振る。プリセットを使う入口として `WALKER_PRESET=g1_v619` を設ける。プリセットを土台にし、環境変数で上書きする（S17）。

### 4.4 環境変数の読み方の方針

1. Walker と misa-biped は環境変数を読まない。例外は `preview.rs`／`wrench_mpc.rs` の `*_DEBUG` で、後で `DebugFlags` を渡す形にする（§7）。
2. ハーネスは起動時に `config_from_kv` を 1 回だけ呼ぶ。毎 tick 読んでいる 5 個とデバッグの 10 か所（§1.5）は S2 で起動時へ移す。
   - 実行中に環境変数は変わらない（`set_var` は 209–220 の 1 か所で、ループより前）ので、移しても一致する。
   - `VH_DCM` は今 `stairs.is_some()` の短絡の後でだけ読んでいる。無条件に先読みしても値は同じ。
   - `WBCW_<段名>` は、段の名前ごとに既定が違う（5903–5911）。名前は次の 13 個で全部なので、起動時に表にする（表に無い名前が来たら今と同じく実行時に読む）。

     `eom`、`contact+cones`、`dynamics+contact+cones`、`com`、`com+momentum+wrench`、`trunk`、`momentum`、`arm hold(before swing)`、`swing`、`arm hold`、`posture`、`posture+momentum`、`regularise`

     表は `HashMap` ではなく `match` か配列で持つ（反復の順で結果を変えないため）。
3. **`ROUGH_MODE`**: `set_var` をやめ、`config_from_kv` の中の上書きにする。
   - 4 個（`SWING_Z_HOLD=1`、`SWING_Z_DROP=0.50,1.0`、`SWING_XY_DELAY=0.15`、`LIFT_H`）を、**今と同じ文字列から同じパースと clamp で**作る。
   - `SWING_Z_DROP` は `[0.50.clamp(0.3,0.95), 1.0.clamp(0.5,1.0)]`、`SWING_XY_DELAY` は `0.15.clamp(0,0.6)`。
   - `LIFT_H` は文字列 `ROUGH_MODE_LIFT_H`（無ければ、`TERRAIN_SLOPE` が環境にあれば "0.06"、無ければ "0.08"）をパースする。パースに失敗したら `0.5 × prof.lift_h` に落ちる（今の `env_f64` と同じ）。
   - `rough mode: …` の印字は、ハーネスの最初の行のまま残す。
   - 4 個のうち 3 個（`SWING_*`）は分類 3 で、`ROUGH_MODE` からしか届かない。上書きを `RoughMode` の列挙型にするかどうかは §7。
4. 読まれなかった環境変数の検出（打ち間違いの警告）は入れるなら opt-in にする（`WALKER_CONFIG_CHECK=1`）。eq.sh はログを `2>&1` で比べるので、既定で印字すると一致が崩れる。

## 5. 段階的な移行の手順

### 5.0 共通の確認のしかた

**作業場所**: 作業ツリーには他の人の未コミットの差分が 115 ファイルある（`src/biped/{actuate,contact,log}.rs`、`src/lib.rs`、`Cargo.lock` などを含む）。基準と新しいバイナリが同じ土台から作られるよう、HEAD から worktree を切る。URDF のパスは絶対パスで本体の作業ツリーを指している（profile.rs 192 行）ので、worktree でもそのまま動く。

```sh
git -C /home/takara/work/dp/articara worktree add -b walker-extract /home/takara/work/dp/articara-walker 2409d96
cd /home/takara/work/dp/articara-walker
export MUJOCO_DYNAMIC_LINK_DIR=~/.mujoco/mujoco-3.8.0/lib LD_LIBRARY_PATH=~/.mujoco/mujoco-3.8.0/lib
cargo build --release --no-default-features --features mujoco --example kyo46rs_walk
ls -la target/release/examples/kyo46rs_walk                    # ~3.4 MB
grep -ac MOM_AXES target/release/examples/kyo46rs_walk         # 1（スタブでないこと）
L=/home/takara/work/dp/articara/log/claude_g1rand_20260925
cp target/release/examples/kyo46rs_walk $L/kyo46rs_walk_wxNN   # 走らせるのは常に写し
```

**一致の確認**（各段で必須）。走らせるのは前景の Bash から `nohup` で行う（記憶 background-bash-kills-children）。出力は大きいので `/data` 側に置く。

```sh
O=/data/articara/log/walker_eq
nohup bash $L/eq.sh $L/kyo46rs_walk_wxNN $O/WXNN > $O/WXNN.out 2>&1 &
LIST=$L/eqlist_push.txt nohup bash $L/eq.sh $L/kyo46rs_walk_wxNN $O/WXNNp > $O/WXNNp.out 2>&1 &
# 終わったら（EQ_DONE を待つ）
bash $L/eqcmp.sh $O/WX00 $O/WXNN       # 計 96 本: CSV 差 0、ログ差 0
bash $L/eqcmp.sh $O/WX00p $O/WXNNp     # 計 14 本: CSV 差 0、ログ差 0
```

基準は S0 の `WX00`/`WX00p` を使い続ける。各段は前の段と比べるのではなく、常に S0 と比べる。差が出たら、`cmp` で最初に違うバイトの行（時刻）を出し、その tick の区画を疑う。

**診断出力の見本 `eqdiag`**（S0 で作り、印字や診断ファイルに触る段 S2・S4・S8・S9・S11〜S15 で比べる）。eq.sh は TRAJ_CSV とログしか比べないので、デバッグの印字と診断ファイルは別に確かめる。見本は 3 本にする。

- 平地 `FAM_nh10_s/…/m00`
- 押し `FAM_nh10_p/i11_d180/m06`
- 昇段 `FAM_nh10_asc_n4/r0.060/m00`

各見本に次の環境変数を足して走らせる。

- デバッグ: `CONTACT_ADMIT_DEBUG=1 DS_FEASIBLE_DEBUG=1 DS_CAPTURE_DEBUG=1 CP_DEBUG=1 PUSH_REPLAN_DEBUG=1 MPC_FS_CROSS_DEBUG=1 MPC_FS_DEBUG_LANDING=1 DEBUG_ANCHOR=1 DIAG_REPLAN=1 MPC_TRAJ_DEBUG=1`
- ダンプ: `GAIT_DUMP_T=3,3.2 CONTACT_DUMP_T=3,3.05 WBC_LEVEL_DEBUG_T=3.0`
- 診断ファイル: `PREVIEW_TRAJ_CSV MPC_TRAJ_CSV MOM_DIAG_CSV TRANSFER_DIAG_CSV QP_WRENCH_CSV ANKLE_CBF_DIAG_CSV TICK_AUDIT_CSV`
- 診断ディレクトリ: `MPC_DIAG_DIR`、`QP_DIAG_DIR`（`QP_CAPTURE_ALL=1 QP_CAPTURE_START=3 QP_CAPTURE_END=3.05`）

比べるときは、`*_RUNTIME_CSV` と、ダンプの JSON の `solve_ns`／`check_ns` を除く（時間なので毎回違う）。この 3 本の小さなスクリプトは S0 で `log/claude_g1rand_20260925/eqdiag.sh` として作る。

**規模の目安**: 1 段の確認は、ビルド数分＋96 本と 14 本を W=24 で回す時間。回す時間は S0 で実測して記録し、各段の見積もりに使う。

### 5.1 段の一覧

各段の「規模」は変更する行のおおよその数。「触る行」は 2409d96 の行で、目印を括弧に添えた。

---

#### S0 準備: 基準の取得（コード変更なし）

- **何を**:
  - worktree を切る。
  - 2409d96 のバイナリ `kyo46rs_walk_wx00` を作る。
  - `WX00`（96 本）、`WX00p`（14 本）、`eqdiag` の基準を取る。
  - 2 回走らせて、自分自身と一致する（決定的である）ことを確かめる。
  - 1 回の確認にかかる時間を測る。
- **危険**: 他の人の差分が入ったバイナリで基準を取ること。本体の作業ツリーで `cargo build` しないこと。
- **規模**: スクリプト 1 本（eqdiag.sh）。

#### S1 シナリオを関数に出す（押しの発火・FIX・地形の生成）

- **何を**:
  - 押しの発火（6588–6662、目印 `// ---- disturbance: fire once`）を `fn fire_push(rig, &PushScenario, &TickView, sink) -> Option<f64>` に出す。
  - FIX の撃力（6281–6312 の移行時の撃力と 6320–6343 の時刻の撃力）を `fn fire_fix(...)` に出す。
  - 地形の MJCF の生成（432–651、目印 `// ---- stairs`）を `fn build_scene(&mut RigOptions, &ScenarioConfig) -> Option<Stairs>` に出す。
  - 押しのノブ（1017–1068）と FIX のノブ（1963–1997）は構造体 `PushScenario`、`FixProbe` にまとめる。
  - すべて example の中に置く。misa-biped には触らない。
- **危険**:
  - 押しの区画の中で突きの位相を印字している（6623–6640）。`jab*` も引数に要る。
  - 印字の文字列とその順は 1 文字も変えない。
  - 返り値で `push_at_t`／`push_at_step`／`push_degraded_at` を戻す。
- **確認**: 96＋14（押しの 14 本が要）。
- **規模**: 約 250 行を移す。新しいコードは関数の枠と引数だけ。

#### S2 毎 tick の環境変数の読みを起動時へ

- **何を**:
  - `LOAD_RAMP_FRAC`（3431）、`VH_DCM`（4162）、`MPC_KKT_COMP_TOL`（4787）、`TURN_IN_DS`（5009）を `let` で起動時に読む。
  - `WBCW_*`（5912）は §4.4 の表にする。
  - デバッグ 10 か所は `struct DebugFlags` に入れる。
- **危険**:
  - `WBCW` の既定は名前ごとに違う。表の作り方を間違えると weighted の重みが変わる（チャンピオンは `WBCW_COM` と `WBCW_POSTURE` を controls に持つ）。
  - `WBC_LEVEL_DEBUG_T` は値（時刻）なので `Option<f64>` にする。パースに失敗したら None（今と同じく黙る）。
- **確認**: 96＋14＋eqdiag。
- **規模**: 約 60 行。

#### S3 計測・判定の状態を `RunMetrics` に束ねる

- **何を**:
  - §1.4 の「計測・判定」67 個を `struct RunMetrics` のフィールドにし、使う箇所を `m.x` に書き換える。
  - 宣言は 2601–2780 と 1997 の `fix_fired`。使う箇所は次の区画にある: 2918–2927、3885–3910、6255–6428、6429–6486、6526–6578、S1 の関数、7079–7149、7160–7563。
  - `StepStat`（2612）も同じ構造体の脇に置く。
- **危険**:
  - 閉包の捕捉。Rust 2021 はフィールド単位で捕捉するので、多くは通る。
  - `fell` と `break` の位置は動かさない。
  - `max_dcm_err` は制御の値 `dcm_err` から作っているので、計算の位置（3885）は S8 まで動かさない。
- **確認**: 96＋14。
- **規模**: 宣言 70 行、使う箇所 約 300 か所（sed で済む置換が大半）。

#### S4 診断の出力先を `DiagSinks` に束ねる

- **何を**: §1.4 の「診断の出力先」18 個と、その元の設定（`QP_DIAG_DIR`、`QP_CAPTURE_*`、`MPC_DIAG_DIR`、`*_CSV` の開き方 703–733、2566–2572、2785–2811、2841–2868）を `struct DiagSinks` にまとめる。
- **危険**:
  - CSV の見出しの行の書き出しの順。
  - `qp_physical` を作る条件（5761–5762）が `qp_runtime_log.is_some()` を読んでいる。診断が制御の計算の有無を変えている箇所なので、条件は今のまま `diag.qp_runtime.is_some()` で写す。
- **確認**: 96＋14＋eqdiag。
- **規模**: 約 120 行。

#### S5 制御の状態を `CtlState`、カウンタを `WalkerStats` に束ねる

- **何を**:
  - §1.4 の「制御の状態」42 個を `struct CtlState` に入れる。`solver`、`qp_previous_structure`、`anchors` は別の `struct WbcState` にする。`policy`、`qp_has_good_command` はハーネスの `struct CommandGuard` にする。
  - 「制御のカウンタ」25 個は `struct WalkerStats` に入れる。`tally` はハーネスに残す。
  - 起動時の定数（`omega`、`z_lip`、`dt`、`t_step`、`total_mass`、`nv`、`na_count`、`foot_rot0`、`terrain_base`、`steps`（初期））は `struct WalkerConsts` にする。
- **危険**:
  - ループ内の閉包 `f_share`／`load_share`（3435–3452）、`inward_bound`（4514）、`push_mom`（5827）、`fallback`（6497）が借用でぶつかりうる。必要なら閉包を使う前に値を局所へ写す（値の計算は変えない）。
  - `preview_p` の更新の順（3182、4117、4137）を変えない。
- **確認**: 96＋14。
- **規模**: 宣言 80 行、使う箇所 約 400 か所。S5a（CtlState）と S5b（WalkerStats・WalkerConsts）に分けてよい。

#### S6 計画を `Plan` に束ねる（`crates/misa-biped/src/plan.rs` を新設）

- **何を**:
  - §3.2 の 1 段目。`gait`、`footsteps`、`dcm_plan`、`height_plan`、`landing_timing` と `z_lip`、`zmp_lat_scale`、`zmp_roll` を `Plan` に入れる。
  - 再構築は 2984、2997、3058、3091、3171、3629、3733、3873、4017、4924、6692 の 11 か所を `plan.rebuild_dcm()` にし、1680 の 1 か所は `Plan::new` にする。
  - 感度の試し（4728）は `plan.dcm_for(&probe)` にする。
  - 起動時の計画作り（1184–1680）は、`Plan` を作る直前まで今の場所に置いたままでよい。
- **危険**:
  - `landing_timing.update(&mut gait, &footsteps, …)`（2962）が同じ構造体の 2 つのフィールドを同時に借りる。`plan.landing.update(&mut plan.gait, &plan.footsteps, …)` はフィールドが pub なら通る。
  - `push_replan_state` が `FootstepPlan` と `DcmPlan` の複製を持っている（3675）。これはそのままでよい。
  - 再構築の位置を 1 つも動かさない（§3.2）。
- **確認**: 96＋14。
- **規模**: misa-biped に約 60 行、walk.rs で約 150 か所の置換。

#### S7 押し回復の参照（`RefLib`、`RefMode`）を misa-biped へ

- **何を**: 27–178 行の構造体と補間を `crates/misa-biped/src/recovery.rs` へ移す。JSON の読み込み（`RefLib::load`）は次のどちらかにする（§7）。
  - (a) misa-biped に `serde_json` を足す。
  - (b) 読み込みはハーネスに残し、pub なフィールドで組み立てる。
- **危険**: 補間の算術（`dcm_at`、`cop_at`、`sole_at`）を書き換えないこと。
- **確認**: 96＋14（参照の作動は押しの見本で起きる）。
- **規模**: 約 150 行の移動。

#### S8 `TickRecord` の導入

- **何を**:
  - §2.4 の構造体を example の中に作り、制御が値を出した直後に記録へ写す。
  - ハーネスの読み（TRAJ_CSV 6880–7077、40 tick の印字 7137–7148、歩ごとの集計 6544–6578、押しの計測、`TRANSFER_DIAG` 6189–6254、`QP_WRENCH` 6719–6802、`QP_RUNTIME` 6709–6718、転倒 7129）を、すべて記録から読む形にする。
  - S8a（TRAJ_CSV と印字）と S8b（計測と診断 CSV）に分ける。
- **危険**:
  - 記録へ写すのは「値の確定後」。例えば `z_ref` は 4231 と 4300 で 2 度束縛し直しているので、2 度目の値を写す。`p_cmd` は 4156 の予見の補正の後の値を写す。
  - `roll`／`pitch` は推定値で、植物の真値ではない。転倒の判定もこれを使っている（§6.8）。
- **確認**: 96＋14＋eqdiag。
- **規模**: 構造体 約 80 行、置換 約 200 か所。

#### S9 `Config` の導入と example のディレクトリ化

- **何を**:
  - `git mv examples/kyo46rs_walk.rs examples/kyo46rs_walk/main.rs`。Cargo は自動で見つけ、バイナリ名は変わらない。
  - `crates/misa-biped/src/config.rs` に §4.1 のサブ構造体を置く。
  - `examples/kyo46rs_walk/config_env.rs` に `config_from_kv` を置く。
  - 起動時の `let x = env_f64(...)` を `cfg.sub.x` に置き換える。
  - 3 回に分ける。
    - S9a: WBC、関節制限、利得、DCM、歩容、段の足跡、降段、斜面
    - S9b: 遊脚（ここで ROUGH_MODE の上書きに替える）、腕、MOM_CMP、接触、管理接触、着地、CoM の計画
    - S9c: contact_plan、両脚支持、MPC 3 束、押し、速度表、参照、DebugFlags
- **危険**:
  - 文脈で決まる既定（§4.2）を取り違えること。
  - 同じ名前を 2 度読んでいるもの（`COM_REPLAN`、`COM_LAND_VXY/VZ`、`LOAD_RAMP_FRAC`、`CP_MAX_Y`）は、同じフィールドから読む。
  - `.max(...)`／`.clamp(...)` の後処理は、読みの式と一緒に移す。
  - 起動時の `assert!`（`DS_SETTLE`、`COM_REPLAN_FORWARD_GAIN` など）を印字との相対順を崩さずに移すこと。違うのは不正な値のときだけ。
  - S9b の ROUGH_MODE は `rough30` の見本 2 本が要。
- **確認**: 各回 96＋14＋eqdiag。S9c の後で `WALKER_CONFIG_DUMP=path`（Config の `Debug` を書き出す、opt-in）を足し、チャンピオンの controls で書き出した内容を S17 の照合に使う。
- **規模**: config.rs 約 600 行、config_env.rs 約 500 行、main.rs 約 700 行の削除と置換。
- **実施（2026-10-29、37ccb32・4529b7e・ffb029c と S9d）**: 4 回に分けた（S9d はプリセットと試験）。各回 96＋14＋eqdiag＋FIX・JAB の単発で差 0。設計と違えた点と、決めた細部は次のとおり。
  - **既定は misa-biped に 1 か所**。`Config::code_defaults(&Profile)` が持ち、`config_from_kv` は `num("KEY", d.field)` の形でそこから上書きする。値は読みの後処理（`.max`／`.clamp`／`as usize`）の後で持つ。実現した周期や他のサブ構造体のノブとの比較（`COM_REPLAN_DT.max(dt)`、`COM_STAIR_SS_MAX.max(T_SS)`、`STAIR_WRENCH_DT`／`_PREVIEW_DT`、`COM_PREVIEW_DT`）は使う場所に残した。
  - **§4.2 の `resolve(&ResolveCtx)` は、Option のフィールドと小さなメソッドにした**。他のノブ由来の既定（`KD_CONTACT`、`KP_SWING_XY`、`CP_MAX_Y_IN`、`STAIR_TD_BIAS_ASC_MID`／`_DESC`、`COM_PREPLAN`、`COM_LAND_EXTEND_DESC`）はサブ構造体のメソッド、文脈由来の既定は引数を取るメソッド（`WbcConfig::friction_mu(mu_ground, margin)`、`f_max_per_foot(m, g)`、`ComPlanConfig::z_com(com0_z)`、`DcmConfig::params(z_com, terrain_base) -> DcmParams`）。起動時のどこで値が決まるかが今と同じ位置に残るので、assert と印字の順が動かない。
  - **着地の時計**: `Config::landing_timing(m, g)` が設定済みの `LandingTiming` を作って `Plan::new` に渡す。起動時に `plan.landing.x = …` で散っていた 11 か所が消えた。
  - **2 度読み**: 一覧の 4 つに加え、`CONTACT_EVENT`（`event_mode_cfg` と `plan.landing.event_mode`）と `VXS_STRIDE_MAX`／`VXS_CAP_SLOPE`／`VXS_V_LONG`（速度表と `vxs_cap_of`、`SpeedScheduleConfig::stride_cap`）も 1 つにした。
  - **ROUGH_MODE**: `config_env::rough_mode(get)` が今までの 4 つの文字列を返し、`config_from_kv` はその 4 キーの読みを差し替える。`set_var` は無くなった。
  - **`LevelWeights`** は misa-biped へ。表に無い段の名前は `get` が None を返し、ハーネスがその場で環境変数を読む（今のスタックは表に無い名前を作らない）。
  - **Config に入れなかったもの**: §4.1 の除外に加えて `GAIT_DUMP_T`／`CONTACT_DUMP_T`（ハーネスの印字なので DebugFlags ではなく main に残した）、`REF_LIBRARY`。`STRIDE` は §4.1 どおり `GaitConfig` に入れた（`VX` の既定はハーネスがそこから作る）。
  - 文字列のノブ（`ARM_HOLD_JOINTS`、`MOM_LEVEL`、`MOM_REF`）は今の比較を保つため `String` のまま。列挙型にするのは Walker が入った後。
  - 不正な `WBC_STRATEGY`／`WBC_QP_BACKEND`／`MPC_QP_BACKEND` の panic は読みの時点（起動の最初）に移った。違うのは不正な値のときだけ。
  - **プリセットは S17 から S9d へ前倒し**した（§4.3）。`Config::g1_v619()` と `Config::kyo46rs_v2()`。試験は `crates/misa-biped/tests/presets.rs` で、ハーネスの表（`examples/kyo46rs_walk/config_env.rs`）を `#[path]` で取り込み、controls.json をその表に通したものとプリセットが一致すること、キーが無ければ `code_defaults` になること、ROUGH_MODE がノブに勝つことを見る。JSON は平らな文字列の辞書なので、試験の中の 20 行の読みで済ませ、serde_json は足していない（Cargo.lock を変えないため）。`WALKER_CONFIG_DUMP=path` も S9d で足した（標準出力には何も出さない）。
  - mujoco 抜きのビルド（既定の機能）: `fn main` に `#[cfg(feature = "mujoco")]` が無く、先頭の属性が 2 重で `use nalgebra` に付いていた。直した後は「MuJoCo が要る」と案内するだけの 434 KB のバイナリになる。

#### S10 `RobotModel`／`RobotState` を misa-biped へ（`rig.sync` の分割）

- **何を**:
  - `src/biped/rig.rs` の 1082–1228 を `misa_biped::robot::RobotModel::assemble` に移す。rig.sync は前半（植物の読み、センサの模型、融合）の後でこれを呼ぶ。`BASE_VEL_KIN` の補正は、q/v を組んだ後・`crba` の前に、rig 側から差し込める形にする。
  - `BipedState` は `RobotState` の別名にする。
  - `BipedRig::walker_model()` を足す。
  - walk.rs の制御の区画の `rig.x` を `robot.x` に置き換える（§2.6 の一覧）。
  - 観測は `Observation { tick, estimate: Estimate::State(&st), contact_fz, joint_pos_plant }` の形にする。
- **危険**:
  - 算術の順。`mass[(vi,vi)] += armature`、`h[vi] += damping * v[vi]` の順と、関節の反復の順を保つ。
  - rig.rs は今は他の人の差分が無い。作業中に差分が入ったら、ここで止めて調整する。
- **確認**: 96＋14（`FAM_nh10_sense` と `mis_*` が要）。
- **規模**: rig.rs から約 150 行を移し、walk.rs で約 60 か所を置換。
- **実施（2026-10-29）**: 96＋14＋eqdiag＋FIX・JAB の単発で差 0（rig.rs だけを替えた途中の段でも差 0 を見てから walk.rs を替えた）。`cargo check --no-default-features --features mujoco --examples --tests` は前からの 2 つ（`wrench_mpc_bench`、`format_mjcf`）を除いて通り、警告も増えていない。
  - `crates/misa-biped/src/robot.rs` を新設: `RobotModel`、`RobotState`、`StateEstimate`、`MassLink`（rig.rs から移し、rig は `pub use` で同じ名前を残す）、`Observation`／`Estimate`。
  - `RobotModel::assemble(est, qd_clamp)` は `configuration(est, qd_clamp) -> (q, v)` と `state(q, v, est) -> RobotState` の 2 つに分けた。`BASE_VEL_KIN`（MuJoCo の接触を読む）は rig.sync がこの 2 つの間で `v` を直す。算術と反復の順は rig.rs のまま。
  - `BipedRig` の公開フィールドは変えていない（他の example が使う。`kyo46rs_com_squat.rs` には他の人の差分がある）。代わりに build の最後で同じ値を写した `RobotModel` を内側に持ち、`sync`・`com_of`・`gravity_torque`・`actuated`・`side_of` はそこへ委ねる。`BipedRig::walker_model()` はその写しを返す。build の後に `robot.joints[].armature／joint_damping` を書き換える所は無いことを確かめた（植物と同じ値を読む前提）。
  - `BipedState` は `RobotState` の別名。
  - walk.rs: build の直後に `let robot = rig.walker_model();`。起動時の `nv`／`na`／`total_mass`／関節制限／`com_of`／`foot_mi` と、ループの `rig.sync()` から `tally.observe` までの制御の区画の `rig.x`（53 か所）を `robot.x` に置き換えた。その区画に残る `rig` は `sync`、`Observation` の植物の関節角、`CONTACT_DUMP_T` の `rig.sim.contacts()` だけ。
  - 観測: `measured_pre` の直後に `Observation { tick, estimate: Estimate::State(&st), contact_fz: measured_pre, joint_pos_plant: &rig.robot.joint_positions }` を作り、制御の区画の `measured_pre` と植物の関節角はここから読む。`tally.observe` より後（ハーネスの計測・ログ）は `measured_pre` のまま。

#### S11 tick の制御を相の関数に分ける（example の中）

- **何を**: 次の相を `fn phase_x(ctl: &mut CtlState, plan: &mut Plan, cfg: &Config, k: &WalkerConsts, obs: &Observation, out: &mut dyn LogSink) -> PhaseXOut` の形の関数にする。
  - **（ユーザー判断 2026-10-29）相の関数は自分の出力を返し、`TickRecord` は step の最後に 1 回、今のリテラルの形のまま組み立てる。** 相が `&mut TickRecord` に書き込む形にはしない（S8 の「値の確定後に写す」が、相の返り値を集めた所で 1 回だけ起きる）。

  | 回 | 相 |
  |---|---|
  | S11a | 1 着地の設定（2891–2917）／2 着地の時計・DS（2953–3193）／3 支持と係留点（3195–3287）／4 接触集合と荷重の傾斜（3289–3452） |
  | S11b | 5 DCM 参照・PUSH_YIELD・PUSH_REPLAN・PUSH_Z_DROP・参照の作動（3556–3884）／6 指令 ZMP と contact_plan（3912–4027）／7 予見（4028–4157）／8 加速度と高さ（4158–4309） |
  | S11c | 9 レンチ MPC（4311–4958） |
  | S11d | 10 段の参照（4959–5755 の参照の計算の部分。遊脚は目標の計算を段の構築より前へほどく）／11 段の組み立てと解（3454–3555 と 5757–6070）／12 解の後の診断（6071–6119、6121–6188）／13 参照の終了（6664–6695） |

  あわせて、CMM と `Footsteps::from_fk` の重複計算を 1 回にまとめる（§2.6）。
- **危険**:
  - 変数の束縛し直し（`z_ref`、`base`、`p`）と影の付け方。
  - 相をまたいで使う局所変数（`xi`、`r`、`dcm_err`、`support`、`slice`、`steps`、`stance_sides`、`p_cmd`、`a_com`、`tau_gravity`）は、相の返り値か記録に載せる。
  - 遊脚の区画の目標と段のほどき（5361–5755）が一番大きい。段の構築に渡す値（`tgt`、`tgt_v`、`tgt_a × tracking_ff_scale`、軸、回転の目標）を、今と同じ式で先に作る。
- **確認**: 各回 96＋14＋eqdiag。
- **規模**: 各回 300〜800 行の移動。
- **実施 S11a（2026-10-30）**: 96＋14＋eqdiag＋s1x（FIX・JAB）で差 0。main.rs 7,946 → 7,344 行、新設 `examples/kyo46rs_walk/walker.rs` 806 行。
  - 置き場は example の中の `walker.rs`（`mod walker;`）。相の関数は main.rs の私的な型（`CtlState`、`WalkerStats`）を `super::` で読む。
  - 形: 相の関数の本体は**ループの文をそのまま**移し、読むノブは関数の頭で `let x = cfg.sub.x;` と束縛し直した（S9 の別名と同じ右辺）。main.rs で使われなくなった別名は、その真上の説明の注釈ごと、最初にそれを読む相の頭へ移した（測定の経緯の注釈を失わないため）。直参照 `cfg.sub.x` への書き換えはしていない（影の付け直しで取り違える危険を避けた）。
  - 起動時の定数は `WalkerConsts<'a>`（`robot: &RobotModel`、`dt`、`omega`、`total_mass`、`z_com`、`terrain_base`、`sole_half_l/w`、`stairs: &Option<Stairs>`、`land_extend_desc`）。`prof` は `robot.prof`（`Profile` は Copy で、rig が持つものと main の `prof` は同値）を読む。
  - 状態は `obs_state(&obs) -> &RobotState`（`Estimate::State` を開く）から読む。
  - 相 1 `phase_landing_setup(&ctl, &mut plan, cfg, k, t)`（rig.sync の前、今と同じ位置）、相 2 `phase_landing_clock(ctl, plan, stats, cfg, k, obs, actual, t)`、相 3 `phase_support(ctl, &plan, anchors, cfg, k, obs, t) -> SupportOut { slice, slice_frac, support, step_idx, slice_idx, steps, swing_rot_mode }`、相 4 `phase_contacts(ctl, &plan, stats, anchors, cfg, k, obs, &SupportOut, t) -> ContactsOut { fz_meas, stance_sides, stance, nc, ramp: LoadRamp }`。
  - 荷重の傾斜の閉包 `f_share`／`load_share` は `LoadRamp::load_share(mi)`（同じ式）にした。ループは `move |mi| ramp.load_share(mi)` を `contact_level` と荷重の分け方に渡す。`lifting`（斜面の踵の CoP 箱）は `ramp.lifting`。
  - 係留点（`WbcState.anchors`）は相 3・4 に `&mut Anchors` で渡す。S12 で出来事の列にする（設置・解放の順は 相 3 → 相 4 の今の順）。
  - `Footsteps::from_fk` は tick に 1 回、`com_vel` の直後で `actual_feet` として作り、相 2 に渡す（今は `if com_replan` の中だったが、純関数で data は tick の中で不変なので、作る位置を前へ出しても値は同じ）。残りの 4 か所は S11b・S11c で置き換える。
- **実施 S11b（2026-10-30）**: 96＋14＋eqdiag＋s1x で差 0。main.rs 7,344 → 6,542 行、walker.rs 806 → 1,786 行（移した別名の注釈を含む）。
  - 相 5 `phase_dcm(ctl, plan, stats, cfg, k, obs, actual, &SupportOut, t) -> DcmOut { xi, r, dcm_err }`（DCM 参照と参照ライブラリの着地版の上書き、PUSH_YIELD、PUSH_REPLAN、PUSH_Z_DROP の検出、参照の作動）。
  - 相 6 `phase_cmd_zmp(ctl, plan, stats, cfg, k, &SupportOut, &DcmOut, t) -> CmdZmpOut { p_cmd, clamped, zmp_deficit }`（指令 ZMP と支持の箱、contact_plan）。
  - 相 7 `phase_preview(ctl, &plan, stats, diag, cfg, k, obs, actual, stance_sides, p_cmd, t) -> p_cmd`（予見の解き直しと x の補正）。`PREVIEW_TRAJ_CSV`／`COM_PREVIEW_RUNTIME_CSV` を書くので、当面 `&mut DiagSinks` を受ける（S13 で `WalkerDiag` のフックにする）。
  - 相 8 `phase_height(ctl, plan, cfg, k, obs, &SupportOut, stance_sides, &p_cmd, t) -> HeightOut { z_ref, a_com }`（水平の加速度とジャーク制限、高さ参照、重心下げ、鉛直の PD）。`z_ref` は 2 度目の束縛（重心下げの後）を返す。
  - `WalkerConsts` に指令とハーネス由来の値を足した: `vx`、`t_step`、`gp: &GaitParams`、`squat_amp`／`squat_period`、`vx2_change`（歩幅の上限で直した後のもの）、`vx2_cmd_speed`（直す前の VX2）、`vx2_z_from`、`vx2_z_drop`、`ref_libs: &[RefLib]`、`contact_plan_cfg`、`com_preview_dt`、`com_replan_dt`。`vxs_cap_of` の閉包は相 5 の中で `cfg.speed.stride_cap(v)` から作り直した（同じ式）。
  - 参照の作動（相 5）と予見（相 7）の `Footsteps::from_fk` を `actual_feet` に置き換えた。
- **実施 S11c（2026-10-30）**: 96＋14＋eqdiag＋s1x で差 0。main.rs 6,542 → 5,801 行、walker.rs 1,786 → 2,620 行。
  - 相 9 `phase_wrench_mpc(ctl, plan, stats, diag, cfg, k, obs, actual, &Centroidal, &SupportOut, stance_sides, &DcmOut, &a_com, t) -> u128`（MPC_DIAG_DIR の捕獲にかかった時間 `mpc_capture_ns`）。主スイッチの判定（`STAIR_WRENCH_MPC && WRENCH_MPC_ALWAYS`、off なら保持を捨てる else 枝）ごと移した。`MPC_DIAG_DIR`／`MPC_RUNTIME_CSV`／`MPC_TRAJ_CSV` を書き、`QP_WRENCH_CSV` の有無で `wrench_mpc_frames` を残すので `&mut DiagSinks` を受ける。
  - `landing_inward()`（main.rs の末尾の関数）は制御なので walker.rs へ移した。
  - `WalkerConsts` に足したもの: `f_max_per_foot`、`friction_mu`、`mpc_gate`、`wrench_mpc_apply`、`stair_wrench_dt`、`stair_wrench_preview_dt`、`contact_cfg: &ContactCfg`（MPC の CoP 箱と捩り摩擦は WBC の接触設定から読む。S12 で Stack 側の値をどう Walker に見せるか決める）。
  - **CMM の重複を 1 回にした**: `walker::Centroidal`（`A_G(q)` と `Ȧ_G v` を `OnceCell` で最初に要る所で 1 回だけ計算）を tick ごとに作り、MPC の `h`、角運動量の段、MOM_CMP がそれを読む。3 か所の呼び出しの間に `q`／`v`（`st` の不変の借用）は変わらず、misarta の `compute_centroidal_momentum_matrix(model, q)`／`compute_cmm_dot_times_v(model, q, v)` は純関数なので、値は同じ。見本はすべて MPC・`MOM=1 MOM_REF=arms_neutral`・`MOM_CMP=1` なので、共有の経路は 96＋14 本で通っている。ハーネスの計測（押しの後の `h_roll`、2 か所と TRAJ_CSV の `MOM_DIAG`）は `rig.model` で別に計算していて、そのまま残した（計測はハーネスの責任。S13 で `walker.state()` に載せるかは未定）。
  - **`Footsteps::from_fk` の重複も 1 回にした**: 制御の 4 か所（着地の時計、参照の作動、予見、MPC）はすべて `actual_feet` を読む。TRAJ_CSV の 1 か所（ハーネス）は S11d で記録から読む。
- **実施 S11d（2026-10-30）**: 96＋14＋eqdiag＋s1x で差 0。main.rs 5,801 → 4,247 行、walker.rs 2,620 → 4,554 行（S11 全体で main.rs は 7,946 → 4,247 行）。
  - 相 10 `phase_task_refs(&ctl, &plan, stats, cfg, k, obs, &Centroidal, &SupportOut, stance_sides, stance, nc, &LoadRamp, &DcmOut, &p_cmd, &zmp_deficit, t) -> TaskRefs`。段を 1 つも作らずに、段の参照だけを計算する: 胴（推定の roll／pitch、計画のヨー、`trunk_ori_ref` の指令 `ori`、胴のヤコビアンとバイアス）、角運動量（`MomRef`: arms_neutral の CMM・バイアス・h・参照・利得・軸）、MOM_CMP（`CmpRef`。カウンタ `n_mom_cmp_ticks`／`mom_cmp_peak` もここ）、腕の保持（`ArmRef`: 行と突きを掃いた参照）、重力トルク、荷重の分け方 `shares`、遊脚（`SwingRef`）。
  - **遊脚のほどき**: 段の構築の呼び出し `bt::swing_with_pose_acceleration(..)`／`bt::swing_with_pose(..)` の引数を、呼び出しが評価していた順に先に作る形にした。`axes`（TERRAIN_PITCH の分岐と `swing_rot_mode` の対応）、`rot`（測った回転、目標の回転＝`foot_rot_at` に斜面のピッチ、角速度、`kp_sw_rot`、`kd_sw_rot`）、`acc = tgt_a * tracking_ff_scale`（足に戻すだけの枝は `None` で、`swing_with_pose` を使う）。`tgt`・`tgt_v`・保持・参照の足裏の軌跡・探りの深さの式は今のまま。
  - 相 11 `phase_wbc(wbc_state, diag, &ctl, wcfg, cfg: &SolveConfig, k, obs, &Centroidal, &SupportOut, stance_sides, stance, nc, &LoadRamp, &a_com, &TaskRefs, t) -> WbcOut`。今の 2 つの区画（動力学の文脈・トルク箱・関節制限・`contact_level`、と CoM・胴・角運動量・MOM_CMP・腕・姿勢・名目の力・正則化・遊脚・段の並べ方・解・門・QP_DIAG_DIR）を 1 つにした。
    - `contact_jacobians`／`Dynamics::new`／`bt::contact_level` は、相 5〜9 の前から相 10 の後へ動いた。`contact_level` は係留点を漏らす（`&mut Anchors`）が、相 5〜10 は係留点も `contact_level` の出力も読まない（`p0.sole_sel` を読むのは名目の力の MPC の写しだけで、相 11 の中）ので値は変わらない。純粋な段の構築（`bt::com`、`trunk_rpy` など）も同じ。
    - 返り値 `WbcOut { sol, extracted, level_names, forces_nominal, sole_sel, acc_dbg, p_swing_diag, trunk_cmd_rp, trunk_acc_rp, qp_solve_ns, qp_check_ns, qp_capture_ns, qp_physical_violation, qp_reset, qp_rejected, degraded }`。`dyn_ctx.extract(&sol.x)` は相 11 の終わりで行う。
    - 名前の衝突: ループでは `cfg` が WBC の `SolveConfig`、`wcfg` が歩行の `Config`。相 11 だけ引数名をこの 2 つにそろえ、本体の文をそのまま使った。
  - 相 12 `phase_solve_diag(cfg, k, obs, stance, &TaskRefs, &WbcOut) -> SolveDiag`（遊脚の残差、脚の届く範囲、遊脚の達成加速度と位置の誤差、足ごとの CoP・力、CoP の使用率、正味の CoP）。
  - 相 13 `phase_end_tick(ctl, plan, cfg, t)`。押しの発火の後、今と同じ位置で呼ぶ。
  - `TickRecord` はループの中で 1 回、今のリテラルの形で組み立てる（相の返り値を同名の局所へ分解してから）。足したのは `actual_feet`（tick に 1 回の FK の足裏）だけで、TRAJ_CSV はそれを読む。これで `Footsteps::from_fk` は tick に 1 回になった。
  - `WalkerConsts` に足したもの: `wz`、`jab`／`jab_start`／`jab_period`／`jab_pattern`／`jab_pitch`／`jab_elbow`、`kp_sw_xy`、`tracking_ff_riser_ok`、`foot_rot0`（`foot_rot_at` の閉包は相 10 の中で同じ式で作る）、`jlim_cfg`、`qp_gate`、`weighted_levels`、`hip_mi`、`leg_reach`。
  - 新しく見つけた漏れ（ハーネスの物を制御が読んでいた）: 角運動量の `arm_cols` と腕の保持の `arms` が `rig.actuated()` を読んでいた（S10 の置き換えの漏れ）。`robot.actuated()`（rig は同じ `RobotModel` の写しに委ねている）に替えた。値は同じ。
- **S11 の後のループ（main.rs、5d06f20 の次のコミットの行）**: 2713 `WalkerConsts`、2761 `for tick`、2765 相 1、2817–2851 相 2〜12、2887 `TickRecord`、2945 `tally.observe`、3350 押しの発火、3371 相 13、3494 `rig.step`、3571 TRAJ_CSV の足裏。
- **S12（`WbcStack`）への申し送り**
  - 渡すものはそろった: 相 11 の引数のうち Stack の外から来るのは `&TaskRefs`（参照）、`a_com`、`stance_sides`／`stance`／`nc`、`LoadRamp`、`support`／`steps`（斜面の踵の CoP 箱）、`ctl.managed_until`（管理接触）、`ctl.wrench_mpc_command`（名目の力）、`obs`（`tick`、`contact_fz`＝QP_DIAG、`joint_pos_plant`＝姿勢と腕）。これが `WbcRequest` の中身。
  - 係留点の出来事は相 3（解放・設置）→ 相 4（新しい接触の設置）→ 相 11 の `contact_level`（漏れ）の順。S12 では相 3・4 が出来事を記録し、相 11 の頭でその順に適用してから `contact_level` を呼ぶ。
  - Stack が読んでいる起動時の値: `contact_cfg`、`jlim_cfg`、`SolveConfig`、`qp_gate`、`weighted_levels`、`total_mass`・`sole_half_l`（物理の行の尺度）。`contact_cfg.cop_half`・`mu_torsion` は相 9（MPC）も読む（`WalkerConsts.contact_cfg`）。Walker と Stack の両方が要るので、Walker の Config 側に置いて Stack へ渡す形がよい。
  - 段の重み `WBCW_<名前>` は、Config に無い名前だと相 11 の中で毎 tick 環境変数を読む（`level_weight_from_env`）。Config の `level_weights` に全部入れてから Stack へ渡すこと。
  - `qp_physical`（物理の行の控え）は `diag.qp_diag_dir`／`diag.qp_runtime_log` の有無でも計算される（§7 の 19）。`QP_DIAG_DIR` の捕獲と `WBC_LEVEL_DEBUG_T` は相 11 の中にある。
- **S13（Walker を misa-biped へ）への申し送り**: 制御がまだハーネスから受けている値は `WalkerConsts` に全部集まった。
  - シナリオ・指令: `stairs`（既知の地形）、`vx`・`wz`・`squat_*`・`vx2_change`／`vx2_cmd_speed`／`vx2_z_from`／`vx2_z_drop`（速度表）、`jab*`（突き）、`ref_libs`（REF_LIBRARY）、`gp`（`t_end = T` を含む）。→ `WalkerSetup`（`CommandSchedule`、`terrain`、`recovery_refs`）。
  - 植物・ハーネスの真値: `friction_mu`（`MU_GROUND` 由来、§6.7）、`obs.joint_pos_plant`（姿勢・腕の保持、§6.6）、`foot_rot0`（起動時の `rig.fk_now()`＝植物の関節角からの FK）、`sole_half_l/w`（環境変数。S13 で Profile へ）、`leg_reach`（`LEG_REACH` を読む）と `hip_mi`（`rig.link_to_idx`）。
  - 診断の出力先: 相 7（`PREVIEW_TRAJ_CSV`、`COM_PREVIEW_RUNTIME_CSV`）、相 9（`MPC_DIAG_DIR`、`MPC_RUNTIME_CSV`、`MPC_TRAJ_CSV`、`QP_WRENCH_CSV` の有無で `wrench_mpc_frames` を残す）、相 11（`QP_DIAG_DIR`、`QP_RUNTIME_CSV` の有無）が `&mut DiagSinks` を受ける。→ `WalkerDiag` のフック。
  - 相の関数は main.rs の私的な型（`CtlState`、`WalkerStats`、`WbcState`、`DiagSinks`）と `level_weight_from_env` を `super::` で読む。misa-biped へ移すときはこれらを一緒に移すか、引数の trait にする。`articara::biped::{tasks, contact}` を使うのは相 11 と相 3・4（係留点）だけ。
  - 相の関数のノブの別名（`let x = cfg.sub.x;`）は、関数の頭にまとめて残してある。測定の経緯の注釈は、main.rs から最初にそのノブを読む相の頭へ移した。

#### S12 `WbcStack` と `TaskStack`

- **何を**:
  - `crates/misa-biped/src/wbc.rs` に trait と `WbcRequest`／`WbcSolution`（§2.6）を置く。
  - articara に `src/biped/wbc_stack.rs`（新しいファイル。差分のある contact.rs・tasks.rs・actuate.rs には触らない）を置き、`TaskStack` が S11 の相 11 と段の構築を実装する。
  - `Anchors` は `TaskStack` が持ち、係留点の出来事は `WbcRequest.anchor_events` で渡す。
  - `QP_DIAG_DIR` のダンプと `WBC_LEVEL_DEBUG_T` は `TaskStack` の中で出す。
- **危険**:
  - 係留点の設置・解放の順（3229 → 3281 → 3401）と、`contact_level` の中の漏れ（leak）の順。出来事は記録した順に適用してから `contact_level` を呼ぶ。
  - MPC のレンチを WBC の力の名目へ写すとき、`p0.sole_sel` の逆を使う（5302–5330）。これは Stack の中に置く。
- **確認**: 96＋14＋eqdiag。
- **規模**: trait 約 150 行、wbc_stack.rs 約 900 行（移動が大半）。
- **実施 S12a（2026-10-30）**: 96＋14＋eqdiag＋s1x で差 0。walker.rs 4,554 → 3,988 行、main.rs 4,247 → 4,237 行、新設 `crates/misa-biped/src/wbc.rs` 258 行、`src/biped/wbc_stack.rs` 754 行（相 11 の本体の移動が大半）。1 回で済んだので S12b 以降は無い。
  - **trait**（§2.6 の案 A）: `trait WbcStack { fn solve(&mut self, req: &WbcRequest) -> WbcSolution; fn anchor(&self, side) -> Option<(Vector3, Matrix3)>; }`。`anchor` はハーネスの TRAJ_CSV（係留点の y の列）が読む。`LogSink` はまだ無いので `solve` は sink を取らない（`WBC_LEVEL_DEBUG_T` の印字は Stack の中の `println!` のまま。S13 で sink を足すなら trait に足す）。
  - **`WbcRequest<'a>`**（参照と接触の要求だけ。すべて misa-biped・misa-wbc・nalgebra の型）:
    - 時刻と状態: `t`、`tick`、`robot: &RobotModel`、`state: &RobotState`、`support`（QP_DIAG とデバッグ印字が報告する）。
    - 接触の要求: `stance_sides`／`stance`、`anchor_events: &[AnchorEvent]`（`Release(side)`／`Touchdown { side, pos, rot }`、相 3 → 相 4 の順）、側ごとの `load_share: [f64; 2]`・`contact_mode: [ContactMode; 2]`（Rigid／Managed）・`cop_patch: [Option<CopPatch>; 2]`（SLOPE_HEEL_CUT の箱）。`contact_level` に渡していた 3 つの閉包を、両側について先に評価したもの（どれも純粋で、`load_share` は `ramp.load_share(foot_mi[side])`＝今の `side = (mi != foot_mi[0])` の逆写像）。
    - 参照: `a_com`、`trunk: TrunkRef { j, djv, rp_ref, yaw }`（`yaw = yaw_ref.map(|_| ori[2])`）、`coupled`、`momentum: Option<&MomRef>`、`mom_cmp: Option<MomCmp { cmm, r: &CmpRef }>`（`cmm` は tick の `Centroidal`）、`arm_hold: Option<&ArmRef>`、`joint_pos`（姿勢・腕の保持が読む植物の関節角、§6.6）、`tau_gravity`、`shares`、`mpc: Option<MpcNominal { wrench, authority, blend }>`（`wrench_mpc_command` を `WRENCH_MPC_APPLY && nc > 0` のときだけ。`authority = MPC_SOLE_AUTHORITY && nc == 2`）、`swing: Option<&SwingRef>`、`contact_fz`（QP_DIAG のみ）。
    - `MomRef`／`CmpRef`／`ArmRef`／`SwingRef` は walker.rs から wbc.rs へ移した（中身は同じ）。`SwingRef.axes` は misa-biped の `SwingAxes`（tasks.rs の `SwingAxes` の写し）。`ContactMode` も写しで、`wbc_stack.rs` が tasks.rs の型へ写す。
  - **`WbcSolution`**: `sol`（門で落としたときは `Degraded`）、`extracted`、`level_names`、`forces_nominal`、`sole_wrench`（スロットごとの `p0.sole_sel × forces`。相 12 の CoP はこれを読む）、`acc_dbg`、`swing_residual`（重みを付ける前の遊脚の段の `A x − b`。相 12 の RMS と z 行はこれを読む）、`trunk_acc_rp`、`timings: QpTimings`、`physical_violation`、`reset`、`rejected`、`degraded`。`trunk_cmd_rp` は `refs.rp_ref` そのものなので解からは外し、ループで `rp_ref` を使う。`sole_sel` と遊脚の `Task` は外へ出さない。
  - **`TaskStack`**（`src/biped/wbc_stack.rs`、mujoco に依存しない）: `cfg: TaskStackConfig`、`Solver`、前の tick の QP の構造、`Anchors` を持つ。`solve` の頭で `anchor_events` を順に適用し、あとは相 11 の文をそのまま（動力学、トルク箱、CBF、`contact_level`、各段、名目の力の MPC の写し（`p0.sole_sel` の逆）、段の並べ方と重み、解、門、`WBC_LEVEL_DEBUG_T`、QP_DIAG_DIR、取り出し）。contact.rs・tasks.rs・actuate.rs は変えていない。
    - 係留点の順: 相 3・4 は出来事を記録するだけになり、適用は相 11 の頭へ動いた。相 5〜10 は係留点を読まない（S11d で確かめた）ので値は同じ。`[anchor]` のデバッグ印字は相 3 のその場に残した。
    - `TaskStackConfig::new(&wcfg, SolveConfig, &ContactParams, &jlim_cfg, total_mass, sole_half_l, dt, StackDiag)` が Config から段のノブを読む: `eom_hard`、JLIM（`joint_limits: Option<JointLimitCbf>`）、`validate`（`wcfg.wbc.qp_gate()`）、`weighted`、`level_weights`、姿勢・胴の有無と利得、腕の利得と `ARM_BEFORE_SWING`、`MOM_LEVEL`、`MOM_CMP_W`、`WBC_LEVEL_DEBUG_T`。
    - **段の重み**: `level_weight_from_env`（tick の中の環境変数の読み）を消した。`LevelWeights` は Stack が作る 13 の名前をすべて持っている（`LevelWeights::NAMES`）。段の名前は `levels` と必ず対で積まれるので、表に無い名前の枝には来ない。その枝は `LevelWeights::default_for` にした。
    - **接触の設定**: `bt::ContactCfg` の代わりに misa-biped の `ContactParams`（同じ 11 個の数値）を main.rs の同じ位置で作る。Walker（`WalkerConsts.contact_cfg`、相 9 の MPC の CoP 箱と捩り、相 11 の踵の切り取りの箱の幅）と Stack（`wbc_stack::contact_cfg(&ContactParams)` で `bt::ContactCfg` に写す）が同じものを読む。
    - `WalkerConsts` から `jlim_cfg`・`qp_gate`・`weighted_levels` を外した（Stack の設定に移った）。main.rs の `weighted_levels` の束縛と `WbcState` も消えた。`jlim_cfg` はハーネスの ANKLE_CBF_DIAG が読むので main.rs に残る（Stack は写しを持つ）。
    - QP_DIAG_DIR の設定（`DiagSinks` の 7 個の欄）は `wbc_stack::QpDump` 1 個にまとめた。環境変数は `DiagSinks::open` の同じ位置・同じ順で読み、Stack を作るときに `std::mem::take` で渡す。写しの番号 `count` は Stack の状態。
    - `serde_json` を articara の `[dependencies]` に足した（dev-dependencies だけだった。QP_DIAG のダンプを lib の Stack が書くため）。Cargo.lock の中身は変わらない。
  - **§7 の 19 は解いた**: 物理の行の控え（`qp_physical`）を開く条件を `validate || diag.measure_violation || diag.dump.dir.is_some()` にした。`validate`（`TaskStackConfig.validate`＝QP_BENCH_GATE か native の backend）だけが解の採否（`rejected`）を変え、`StackDiag`（`measure_violation`＝QP_RUNTIME_CSV の有無、`dump`＝QP_DIAG_DIR）は計測を足すだけ。真理値は今の `qp_diag_dir || qp_gate || qp_runtime_log` と同じ。ハーネスの `DiagSinks` は制御の計算経路から外れた（`qp_runtime_log.is_some()` は起動時に 1 回、`StackDiag` へ写すだけ）。
- **S13（Walker を misa-biped へ）への申し送り**
  - 相 11 は `phase_wbc(stack: &mut impl WbcStack, ..) -> WbcSolution` で、請求の組み立てだけになった。`Walker<S: WbcStack>` は `step` の中でこれを呼べばよい。ハーネスは `TaskStack::new(TaskStackConfig::new(..), Solver::new())` を作って渡す。係留点を TRAJ_CSV に書くので、ハーネスは `walker.stack().anchor(side)` のような読み口が要る。
  - **walker.rs に残る articara／misa-mpc への依存**（misa-biped へ移す前に片付けるもの）:
    - `bt::trunk_ori_ref`／`bt::TrunkGains`（相 10 の胴の向きの参照。段ではなく参照の計算なので、misa-biped へ移すのが筋。tasks.rs は今は差分が無い）。
    - `contact::cop_from_sole_wrench`（相 12 の CoP。`sole_wrench` から計算する 6 行の純関数。misa-biped に写すか、`WbcSolution` に CoP を載せる）。
    - `articara::biped::wrench_mpc`（＝misa-mpc、相 9）。misa-biped の依存は misarta・misa-wbc・nalgebra だけという決めごとと衝突する。(a) misa-biped が misa-mpc に依存する（同じリポジトリの crate）、(b) MPC も `WbcStack` と同じく trait で注入する、のどちらかの判断が要る（ユーザー）。
    - `rig::G`（9.81）は `misa_biped::dcm::G`（同じ 9.81）に替えられる。`serde_json`（相 9 の MPC_DIAG_DIR）は `WalkerDiag` のフックへ。
  - `ContactMode`・`SwingAxes` は misa-biped と tasks.rs に 2 つある（`wbc_stack.rs` の `contact_mode`／`swing_axes` が写す）。tasks.rs・contact.rs を `TaskStack` ごと misa-biped へ移すとき（§7 の 3）に 1 つにする。
  - 段の並べ方（MOM_LEVEL、ARM_BEFORE_SWING、姿勢・胴の有無）と重みは、案 A のとおり当面 `TaskStackConfig` にある。`ContactParams` は main.rs が作っている（`kp_c`、`kd_c`、`anchor_leak`、`friction_mu`、`f_max_per_foot`、`cop_frac`、`dt` から）。S13 で `Walker::new` が Config とプロファイルから作り、`walker.contact_params()` をハーネスが Stack に渡す形にするとよい。

#### S13 Walker を misa-biped へ

- **何を**:
  - `crates/misa-biped/src/walker.rs` に `Walker<S>` を置く。
    - `new`: 1114–1723 の計画作り（段の足跡 1373–1632、COM_PREPLAN、DCM）と、制御の状態の初期化。
    - `step`: S11 の相 1〜12。
    - `end_tick`: 相 13。
  - `LogSink` で印字する。
  - example は `walker.step()`／`walker.end_tick()` を呼び、`CommandGuard`（方策）・`DegradedTally`・計測・シナリオ・ログを担う。
  - 制御の中で書く診断ファイル 5 本（§2.4）は `WalkerDiag` のフックにする。
- **危険**:
  - §2.5 の呼び出しの順。`tally.observe` は `step()` の直後に呼ぶ。
  - `end_tick()` は押しの発火の後に呼ぶ。
  - 計画作りの印字（1198、1283、1333、1444、1627、1650、1682–1721）は sink 経由で同じ順に出す。
- **確認**: 96＋14＋eqdiag。misa-biped は `cargo build -p misa-biped`（mujoco 無し）で通ること。
- **規模**: walker.rs 約 3,500 行（移動）、main.rs は約 2,000 行まで縮む見込み。
- **実施 S13a〜S13c（2026-10-30）**: 3 回とも 96＋14＋eqdiag＋s1x で差 0。デバッグ印字を全部点けた単発 4 本（押し・MPC 診断ファイル付き・昇段・降段。`[lt]`／`[anchor]`（stderr）／`[traj]`／`[wbc-levels]`／`[adm]`／`[dsc]`／`[dsf]`、MPC_DIAG_DIR の JSON 400 個、PREVIEW／MPC_TRAJ／QP_WRENCH の CSV）も S12a と差 0（時間の列とパスの行を除く）。main.rs 4,237 → 3,351 行、`crates/misa-biped/src/walker.rs` 1,280 行（API・型・計画作り・step）＋ `walker/phases.rs` 3,916 行（相の関数）。example の walker.rs は消えた。
  - **S13a**（印字と依存の片付け、example の中のまま）:
    - `misa_biped::sink` を新設: `trait LogSink { fn line(&mut self, &str); fn err_line(&mut self, &str); }`（stdout と stderr を分けたまま。`2>&1` の 1 ファイルで順が保たれる）、`outln!`／`errln!`、`NullSink`。ハーネスは `StdSink`（`println!("{s}")`／`eprintln!("{s}")`）。`print!` を重ねて 1 行にしていた `[traj]` は文字列を組んでから 1 回で出す（バイトは同じ）。
    - `trait WalkerDiag`（既定は全部空、`NoDiag`）: `preview_solved`（PREVIEW_TRAJ_CSV）、`preview_runtime`（COM_PREVIEW_RUNTIME_CSV）、`wants_mpc_input`＋`mpc_input`（MPC_DIAG_DIR。Walker は `wants` が真のときだけ計時する＝今の `mpc_capture_ns` と同じ意味）、`mpc_runtime`（MPC_RUNTIME_CSV）、`keeps_mpc_frames`（QP_WRENCH_CSV の有無。`wrench_mpc_frames` を残すか）、`mpc_solved`（MPC_TRAJ_CSV）。書式と `serde_json` はハーネスの `impl WalkerDiag for DiagSinks` に移した（式はそのまま）。
    - `trunk_ori_ref`／`TrunkGains`／`cop_from_sole_wrench` を `misa_biped::wbc` へ移し、tasks.rs・contact.rs は旧パスで再輸出（tests/biped_tasks.rs はそのまま通る）。`rig::G` → `misa_biped::dcm::G`。
    - `WbcStack::solve(&mut self, req, out: &mut dyn LogSink)`。`[wbc-levels]` も sink 経由。
    - `LAND_TIMING_DEBUG`（preview.rs が毎回 `env::var`）を `DebugFlags.land_timing` → `LandingTiming.debug`（`Config::landing_timing` が写す）にし、`LandingTiming::update` に `out` を足した。misa-biped の中の環境変数の読みはこれで 0。
    - `SOLE_HALF_L／W`: `config_env::profile_from_kv(prof, get)` が `Profile::cop_half` を上書き（読む位置は今の行、値も同じ）。ハーネスは `prof.cop_half` を読み、Walker は `robot.prof.cop_half` を読む。
  - **S13b**（example の中に Walker）: `WalkerInit::new(cfg, robot, setup, out)` が計画作り（旧 1785–2365 行の文をそのまま。環境変数の読みだけ `WalkerSetup` へ）と `CtlState` の初期化（旧 2494–2842 行に散っていた `let` の初期値）を行う。`with_stack(stack)` で `Walker<S>`。`WalkerConsts` は借用をやめて所有にした（相の中の `let robot = k.robot;` 6 か所などを `&k.robot` に）。
    - **段の構築を 2 段にした理由**: 計画の印字（`gait:`〜`NO_LIFT`）→ `TrajLog::create` の印字 → `ankle_roll limit` の印字 → TaskStack の構築（`StackDiag` は `DiagSinks::open` の後）、の順なので、`Walker::new(.., stack)` の 1 段では印字の順が変わる。`WalkerInit` の間にハーネスが `init.contact_params()` から Stack を作る。
    - 相 1（着地の設定）は `step()` の頭に入った（今まで `rig.sync()` の前。印字も観測の読みも無いので値は同じ、§2.5）。
    - `REF_LIBRARY` の読み込み（印字なし）を計画作りの前へ動かした（`WalkerSetup.recovery_refs` に入れるため）。
    - 消えたハーネスの別名（`kp_c`、`kd_c`、`anchor_leak`、`k_dcm`、`no_lift`、`kp_sw_xy`、`f_max_per_foot` とその assert、`land_extend_desc`、`wrench_mpc_apply`、`stair_wrench_*`、`com_*_dt`、`ContactCorrection` と `CONTACT_ON_BW` の assert）は、測定の注釈ごと `WalkerInit::new` へ移した。ハーネスに残した別名は最終報告が読むもの（`contact_admit_*`、`ds_feasible*`、`contact_plan_on`、`push_replan_trig`、`mpc_footstep_cross_y`）と、起動時の assert（`com_replan_forward_gain`、`com_xy_jerk_max`、MPC_FOOTSTEP と contact_plan の排他、`mpc_gate` の作動モード）だけ。
  - **S13c**（misa-biped へ）: `crates/misa-biped/src/walker.rs` と `walker/phases.rs`。パスは `articara::biped::X` → `crate::X`、`articara::biped::wrench_mpc` → `misa_mpc::wrench_mpc`。example は `use misa_biped::walker;`。
  - **公開 API**（`misa_biped::walker`）:
    - `WalkerInit::new(cfg: Config, robot: RobotModel, setup: WalkerSetup, out: &mut dyn LogSink) -> WalkerInit`、`init.contact_params() -> &ContactParams`、`init.plan() -> &Plan`、`init.with_stack(stack: S) -> Walker<S>`。
    - `walker.step(&Observation, &mut dyn LogSink, &mut dyn WalkerDiag) -> StepOutput { command: Command { sol, extracted, tau_gravity, rejected, degraded }, record: TickRecord }`。`TickRecord` は今までのリテラルのまま（`actual_feet` ほか 56 欄）。
    - `walker.end_tick(&mut dyn LogSink)`（相 13。時刻は直前の `step` のもの）。
    - 読み口: `config()`、`robot()`、`plan()`、`stats() -> &WalkerStats`（`Clone`）、`stack() -> &S`（係留点 `walker.stack().anchor(side)`）、`omega()`、`gait_params()`、`initial_feet()`（起動時の足裏、最終報告の歩幅の式が読む）。
    - `WalkerSetup { dt, t_end, terrain: Option<Stairs>, command: CommandSchedule, recovery_refs: Vec<RefLib>, friction_mu, init_fk, leg_reach, hip_mi }`。最後の 4 つが**漏れ**（doc に LEAK と書いた）: `friction_mu`（MU_GROUND 由来、§6.7）、`init_fk`（植物の関節角からの FK＝`rig.fk_now()`。初期の足跡・CoM・`foot_rot0` がここから、§6.6）、`leg_reach`（LEG_REACH）、`hip_mi`（rig の `link_to_idx`）。
    - `CommandSchedule { vx, vy, wz, squat_amp, squat_period, vx2: Option<(usize, f64, usize)>, jab, jab_start, jab_period, jab_pattern, jab_pitch, jab_elbow }`。`vx` の既定（STRIDE ÷ 歩の周期）はハーネスが同じ式で作る。
    - `Observation { tick, estimate, contact_fz, joint_pos_plant }`。`joint_pos_plant` が漏れ（§6.6、doc に LEAK）。
  - misa-biped の依存: misarta、misa-wbc、misa-mpc（path）、nalgebra。MuJoCo・serde・articara は入っていない。misa-biped の中の環境変数の読み・印字・ファイル書き込みは 0。
- **S14 以降への申し送り**
  - 残る環境変数の読み（ライブラリ）: misa-mpc の `MPC_LANDING_TIME_DEBUG`（wrench_mpc.rs 891）、`MPC_FS_DEBUG`（1707、1805、1821）。どちらも `eprintln!`。misa-mpc の公開 API（`solve_with_workspace` の引数か `Config`）を変えるので、§7 の 15 のとおり他の利用者と調整してから。
  - Walker に残る articara 由来の物は無い。ハーネスに残る制御寄りの物: `CommandGuard`（方策）・`DegradedTally`（§7 の 4）、`jlim_cfg` と `SolveConfig`（Stack の設定。ANKLE_CBF_DIAG も読む）、TaskStack の構築。
  - S14（観測を推定の結果に）: `step()` の頭の `obs_state(obs)` が `Estimate::State` を開いている。`init_fk` も推定から作る形にするなら `WalkerSetup` の欄を替える。
  - S15（ハーネスを薄く）: main.rs は 3,351 行。`impl WalkerDiag for DiagSinks` と `StdSink` は diag.rs に、最終報告は metrics.rs に出せる。ハーネスは `walker::CommandSchedule` を作るために `jab*` を別に読み、押しの `JabClock` にも同じ値を渡している（`CommandSchedule::jab_phase` に寄せる余地）。
  - S16（`PlanEditor`）: 計画の書き手はすべて `walker/phases.rs` の中に入った（ハーネスは `walker.plan()` を読むだけ）。
  - `WalkerInit` → `with_stack` の 2 段は、印字の順の固定（§7 の 16）をやめれば `Walker::new(.., stack)` の 1 段にできる。

#### S14 観測を状態推定の結果にする（Walker が運動学を計算）

- **何を**:
  - `Estimate::Raw(&StateEstimate)` を使う形に替え、`RobotModel::assemble` は Walker の中で呼ぶ。
  - rig は `fn sense(&self) -> StateEstimate` を持ち、`QD_CLAMP` はここで済ませる。
  - `Estimate::State` の経路は消す。
- **危険**:
  - `BASE_VEL_KIN`（不採用）の補正は、本体座標で `v` を直す。推定の側に出すと、丸めが変わる（§2.6）。だから先に `BASE_VEL_KIN*` を消すか、この段では `Estimate::State` を残す（§7）。
  - 初期の FK（`rig.fk_now()` は植物の関節角を読む）を Walker::new の `init` でどう作るか。今と同じく `joint_pos_plant` から作る。
- **確認**: 96＋14。
- **規模**: 約 200 行。

#### S15 ハーネスを薄くする

- **何を**: `examples/kyo46rs_walk/` を次のモジュールに分ける。

  | ファイル | 担当 |
  |---|---|
  | `main.rs` | ループと §2.5 の順 |
  | `config_env.rs` | 環境変数 → Config |
  | `scene.rs` | MJCF の地形、押し、FIX、的、突きの位相の印字 |
  | `metrics.rs` | `RunMetrics`、最終報告 7160–7563 |
  | `diag.rs` | `DiagSinks`、`WalkerDiag` の実装 |

  使われなくなった配線を消す。
- **危険**: 最終報告の印字の順。
- **確認**: 96＋14＋eqdiag。
- **規模**: 移動が大半。main.rs は 300〜500 行が目安。

#### S16 計画の書き換えを `PlanEditor`／`SliceClaims` に一本化

- **何を**: §3.2 の 2 段目。
  - 13 の書き手を `plan.edit(Writer::X)` 経由にし、`commit()` で再構築とカウンタを行う。
  - 取り決めの判定 7 種類を `SliceClaims` に写す。
  - 確認用の feature（例: `walker-claims-check`）の下で、古い判定と新しい判定を両方計算して `assert_eq!` する。一致を見てから古い判定を消す。
- **危険**:
  - `commit()` の位置を今の再構築の位置にそろえること。
  - DS_FEASIBLE の区間 ZMP の上書きは `commit()` の後に置く。
- **確認**: 96＋14（feature 付きと無しの両方）。misa-biped の単体試験（書き手の順と区間の持ち主）。
- **規模**: 約 400 行。

#### S17 プリセットと照合

- **何を**:
  - `Config::code_defaults`、`g1_v619`、`kyo46rs_v2` を作る（§4.3）。
  - `WALKER_PRESET` の入口を足す。
  - 単体試験で、controls.json を `config_from_kv` に通したものとプリセットが等しいことを見る。
- **確認**:
  - eq.sh の見本から**制御のキーを外し**（`DROP=` に分類 1・2・1k のキー）、`WALKER_PRESET=g1_v619`（kyo の 20 本は `kyo46rs_v2`）を付けて走らせ、WX00 と CSV・ログが一致すること。
  - eq.sh の `DROP` はそのまま使える。プリセットは環境変数を 1 つ足すだけなので、eq.sh を少し直す（`EXTRA_ENV`）。
- **規模**: 約 300 行＋試験。

#### （S18 任意）劣化時の方策と `DegradedTally` を misa-biped へ

- `src/biped/actuate.rs` の差分が片付いた後に行う。
- 方策の印字（`[OPEN LOOP]`／`[recovered]`）は、今は自己衝突の監視（`[self-collision]`）と FIX の印字の後に出ている。Walker の中へ入れると同じ tick では順が入れ替わる。
- 順を保つなら、方策を Walker とは別の部品（`CommandGuard`）としてハーネスが呼ぶ形を続ける。順の変更を受け入れるなら、ログの差を明示した 1 回限りの非一致コミットにする（§7）。

---

### 5.2 段の順の理由

- S1〜S5 は example の中だけの束ね直しで、どれも小さく戻しやすい。S1・S2 は数十〜数百行で、最初の 1 日で済む大きさ。
- S6・S7 は misa-biped に入れても依存が増えない部品（計画と参照）。ここで初めて misa-biped に触る。
- S8（記録）を S11（相の関数）より前に置く。こうすると、相に分けるときに「ハーネスがどの局所変数を読んでいるか」がすでに記録の項目として固まっている。
- S9（Config）を S11 より前に置く。こうすると、相の関数の引数が局所の 400 個の `let` ではなく `&Config` になる。
- S12（Stack）は S11d で段の構築が 1 か所に集まった後に置く。S13 の移動は、それまでの段でほぼ引っ越しの準備が済んだ状態で行う。

## 6. シナリオと制御が絡んでいる箇所のほどき方

### 6.1 参照の終了処理が押しの発火の区画にある

- 今の形: 押しの発火（6588–6662）の直後、同じ「植物の書き込みの前」の区画に、参照の終了・冷却・再センタリング（6664–6695）がある。後者は制御で、計画（`footsteps`、`dcm_plan`、`landing_timing.event_mode`）を書き換える。
- ほどき方: `Walker::end_tick()` に出し、ハーネスは押しの発火の後に呼ぶ。2 つは互いの値を読まない。発火は `support`／`step_idx`／`slice_frac` を読み、終了は `gait`／`footsteps`／`ref_mode` を書く。だから順を入れ替えても数値は同じになる。ただし同じ tick で両方が印字すると行の順が変わる。今の順（発火 → 終了）を保つために `end_tick()` を分ける。
- 台帳 §4.2 の「`ref_trig_at_push`（神託）」はすでに削除されていて、今はこの区画に無い。

### 6.2 `stairs` の段差 0 の扱い

- G1 のベンチは平地も含めて `STAIRS=1,0.000,…` で段差 0 の段を置く（x≈2 m、記憶 flat-harness-stair-at-2m）。
- 今のコードには「段がある」の判定が 4 通りあり、どれも平地のベンチで生きている。

| 判定 | 行 | 段差 0 の段での結果 |
|---|---|---|
| `stairs.is_some()` | 1673（`ZMP_ROLL` の既定 0.030）、4162（`VH_DCM`）、5393（絶対の着地高さ） | 真 |
| `riser.abs() > 1e-6 \|\| down_n > 0` | 3647（PUSH_REPLAN の段ゾーン除外） | 偽 |
| `riser.abs() > 0.005` | 5139（MOM_CMP を止める段ゾーン） | 偽 |
| `riser.abs() >= TRACKING_FF_MIN_RISER` | 1168（遊脚の加速度 FF） | 偽 |
| `stair_zone_crouch(...) > 0` | 3214（遊脚の回転モード）、4258（しゃがみ） | 段差 0 でも区間内なら正になりうる |
| `st.n == 0 && st.slope != 0` | 1586、3532、3666、4219（斜面） | 偽 |

- ほどき方（一致を保つ）:
  - Walker には `Option<Stairs>` をそのまま「既知の地形の模型」として渡す（`WalkerSetup.terrain`）。
  - `Config::resolve` は `terrain_modelled = terrain.is_some()` で `zmp_roll` を決める。
  - 4 つの判定は `Terrain` の名前付きの問い合わせにする。`is_modelled()`、`has_flight(eps)`、`in_stair_zone(x, lead, tail)`、`is_slope()` の形で、閾値は今の値を引数で渡す。こうすると判定が 1 か所に並び、違いが目に見えるようになる。
- 統一（例: 段差 0 は「段なし」にそろえる）は平地の挙動を変える（`ZMP_ROLL` 0.030 → 0、`VH_DCM` off）。だから §7 の判断に回す。

### 6.3 `measure_contacts` が MuJoCo の接触力をそのまま読む

- 今の形: 2932 で `measure_contacts(&rig, data, friction_mu)` が MuJoCo の接触の一覧から足ごとの力・CoP・すべり・レンチを作り、制御はその鉛直成分を「センサ」として使っている。
- ほどき方: 観測には `contact_fz: [f64; 2]` だけを入れる（§2.2）。`measure_contacts` と `MeasuredContact` はハーネス（log.rs、他の人の差分あり）に残す。実機では足裏の力センサか、関節トルクからの推定がこの 2 数を作る。
- 注意: `measure_contacts` の引数の `friction_mu` は制御の仮定の摩擦で、すべりの比の分母にしか使っていない（計測）。

### 6.4 押しの計測が制御の途中にある

- 今の形:
  - DCM 誤差の最大と押し後の回復（3885–3910）が、制御の区画の真ん中で `dcm_err` を読んでいる。
  - 押しの窓（2918–2927）は `rig.sync` より前にある。
  - 移行の計測（6255–6428）は解の後で、`support_prev`（6697）も計測の状態。
- ほどき方: S3 で `RunMetrics` に入れ、S8 で記録の `dcm_err`、`support`、`cop_use` から計算する。計算の結果は計測の変数にしか入らないので、場所を動かしても制御は変わらない。

### 6.5 FIX の撃力が制御の出来事を合図にしている

- 今の形: 6281–6312 は、押しの後の「最初の両脚支持 → 片脚支持」（`support_prev` と `support`）で撃力を加える。6320–6343 は、押しから決まった時間の後に加える。どちらも `rig.sim.apply_external_force` で、植物への書き込み。
- ほどき方: S1 で `fire_fix()` に出す。合図は記録の `support` から取る。

### 6.6 植物の真の関節角が制御に漏れている

- 今の形: `rig.robot.joint_positions` は植物の真の関節角（センサの遅れや雑音が入らない）。次の箇所が読んでいる。
  - 姿勢の段・腕の保持（5250、5266）
  - 劣化時の代替 `gravity_plus_posture`
  - Hybrid の書き込み
  - 初期の FK（`rig.fk_now()`）
  - 関節の振れ幅（6431）
  - 足首ロール（6531）

  一方、`q` は推定値（`SENSE_DELAY_TICKS`／`SENSE_NOISE_Q` が効く）。
- ほどき方（一致を保つ）: `Observation.joint_pos_plant` で明示して渡す（§2.2）。名前で「植物の値」と分かるようにする。
- 推定の `joint_q` にそろえると、sense ベンチ（`FAM_nh10_sense`、遅れ 8 tick）の結果が変わる。だから §7。

### 6.7 `friction_mu` の既定が植物の摩擦から作られる

- 今の形: `friction_mu = env_f64("friction_mu", MU_GROUND × prof.friction_margin)`（810）。rand.py は `MU_GROUND` を個体ごとに振る。だから制御は、振られた床の摩擦を知っていることになる。
- ほどき方（一致を保つ）: `ResolveCtx.mu_ground` として明示的に渡す。
- 固定の仮定（例: 名目の 0.7×余裕）にするのは挙動の変更なので、§7。

### 6.8 転倒の判定が推定の傾きを使う

- 今の形: 7129 の `tilt = roll.abs().max(pitch.abs())` の `roll`／`pitch` は、`trunk_from_base` の場合、推定の `body_quat` から作っている（4971–4973）。転倒の判定（7153）と最大傾きの計測がこれを使っている。一方、高さは植物の真値（7127）。
- ほどき方: 記録の `roll`／`pitch` を使う（一致を保つ）。真値の傾きに替えるかどうかは、判定の定義の変更なので触らない。

### 6.9 指令（速度・スクワット・突き）は起動時に計画に織り込まれる

- 今の形:
  - `VX`／`VY`／`WZ`／`VX2*` は、起動時に `FootstepPlan::velocity_with_change` と速度表（1217–1363）で足跡と時間割に織り込まれる。
  - `SQUAT_*` は毎 tick 高さ参照（4274–4281）に、`JAB*` は毎 tick 腕の参照（5199–5245）に入る。
  - 押しの譲歩（PUSH_YIELD）は `vx2_cmd_speed` を読む。
- ほどき方: 時刻の関数として `WalkerSetup.command: CommandSchedule` で渡す。ハーネスの押しの印字（突きの位相）は、同じ式を `CommandSchedule::jab_phase(t)` として共有する。
- 実行中に指令を変える API（`walker.set_velocity(...)`）は、今の計画の作り方（起動時の一括）と合わない。だから作らない（§7）。

### 6.10 ログの印字が制御の分岐の中にある

- 今の形: 制御の区画の印字は 26 か所。常に出るのは `[yield]` 3632、`[zdrop]` 3766、`[ref]` 3788・3812・3877 の 5 か所で、残りはデバッグの門の下。起動時の計画作りにも 14 か所ある。
- ほどき方: `LogSink` で即時に出す（§2.1）。緩衝しないので順は変わらない。

### 6.11 run の長さ `T` が時間割を決める

- 今の形: `GaitParams.t_end = total_t`（1013）。時間割の最後の区間は run の長さで切られる。
- ほどき方: `WalkerSetup.t_end` で明示する。

## 7. 未決の判断

| # | 判断 | 今の状態と選択肢 | 推奨 | 誰が・いつ |
|---|---|---|---|---|
| 1 | contact_plan のずらしの仕組み（`CP_MAX_*`、`CP_J`、`CP_W_SHIFT`、`CP_W_U`、`CP_DEAD_U`） | チャンピオンは `CP_MAX_*=0` でずらしが常に 0、時間調整だけ。(a) Config に残す、(b) 時間の側だけ Config に出し、ずらしは `contact_plan::Config` の内部定数（0）にする、(c) 仕組みごと消す | (b)。消すのは S16 の後。`shift_from`（3997）の経路が `PlanEditor` で 1 行になってから判断すると軽い | ユーザー、S9c の前 |
| 2 | `CONTACT_ADMIT_CONFIRMED` | 採用だが、29 本のどれでも効かない（台帳 §5.2）。荷重 admit が先に入るため | eq.sh を `DROP=CONTACT_ADMIT_CONFIRMED` で回し、96＋14 本で一致すれば外す。一致しなければ残す | ユーザー（または S0 のついでに実測） |
| 3 | tasks.rs・contact.rs を misa-biped へ移す時期 | contact.rs に他の人の差分（`Anchors::bias_z`、FFDC 用）。tasks.rs は今は差分が無いが、依頼文では差分ありとされている | S12 の trait で待つ。差分が入った後に、`TaskStack` ごと misa-biped へ移す（1 段で済む） | contact.rs の作業の持ち主 |
| 4 | 劣化時の方策（`CommandPolicy`、`DegradedTally`、`gravity_plus_posture`）の置き場 | actuate.rs に他の人の差分。印字の順（§5.1 S18） | 当面ハーネス（`CommandGuard`）。actuate.rs が片付いたら misa-biped の独立部品にし、Walker の外に置く（安全層として） | ユーザー |
| 5 | 植物の真の関節角の漏れ（§6.6） | posture・代替・Hybrid が真値を読む | 一致の段がすべて済んだ後、推定値にそろえる 1 回限りの挙動変更として、家族 21 ベンチで判定する | ユーザー |
| 6 | `friction_mu` の既定が `MU_GROUND` 由来（§6.7） | 制御が振られた床の摩擦を知っている | 同上。固定の仮定にして、`MU_GROUND` の振りへの頑健さを測る | ユーザー |
| 7 | 段差 0 の「段」と 4 通りの判定（§6.2） | 平地のベンチで `ZMP_ROLL=0.030`、`VH_DCM` が生きている | 判定を `Terrain` の問い合わせに並べるところまでは一致の段で行う。意味の統一は挙動変更として別に判断する | ユーザー |
| 8 | 再構築が DS_FEASIBLE の区間 ZMP を消す（§3.2） | 後の再構築があると上書きが消える | 意図を確かめる。区間の上書きを Plan の状態にするなら挙動変更 | ユーザー（DS_FEASIBLE の作者） |
| 9 | MPC の 3 つの主スイッチ（`STAIR_WRENCH_MPC`、`WRENCH_MPC_ALWAYS`、`WRENCH_MPC_APPLY`）と `STAIR_WRENCH_COUPLED` | 台帳 §2.17。`APPLY` は複合ゲート（記憶 apply-flag-confound） | Config では当面 3 つの bool のまま、意味の名前（`enabled`、`always`、`apply`）に付け替えるだけ。1 つにまとめるのは、kyo46rs（全部 off）と G1（全部 on）以外の組み合わせを捨てる判断 | ユーザー |
| 10 | `PUSH_REPLAN_HOLD=10`（実質「保持なし」） | 台帳 §2.20 | `hold: Option<f64>`。プリセットは None。`10` を読んだら None として扱うのは挙動同一（`push_replan_shift > 10` は起きない）だが、環境変数の意味を変えるので明示して決める | ユーザー |
| 11 | `ROUGH_MODE` の形 | 上書き（S9b） | 当面は上書き。モード（列挙型）にして Walker が実行中に切り替えられるようにするかは、地形の知覚の設計と合わせて決める | ユーザー |
| 12 | 要確認 7 個（`ROUGH_MODE`、`ROUGH_MODE_LIFT_H`、`TERRAIN_PITCH`、`VX2_SCHED`、`STAIR_ABSOLUTE_TD_DESC`、`STAIR_SWING_FF`、`STAIR_ZONE_SWING_ROT`）と kyo の v3 候補が使う `STAIR_ABSOLUTE_TD`・`Z_COM_OFFSET` | 台帳 §4.4 の 4 | Config には全部入れる（一致のため）。既定やプリセットの値は kyo46rs v3 の採否と合わせて決める | ユーザー |
| 13 | `RefLib` の JSON の読み込み | misa-biped は serde_json に依存していない | **確定（ユーザー、2026-10-29）: (b)**。JSON の読み込みは misa-biped に入れない。依存は misarta・misa-wbc・nalgebra だけ。参照の読み込みはハーネスの `load_ref_lib`（S7）、プリセットの試験の JSON は試験の中で読む（S9d） | 済 |
| 14 | `BASE_VEL_KIN*`（不採用、rig.rs） | S14 で推定と運動学を分ける妨げ | S14 の前に消す（挙動同一。分類 3 で見本に入っていない） | ユーザー |
| 15 | ライブラリ内の `*_DEBUG` の環境変数の読み（preview.rs、wrench_mpc.rs） | 毎回 `env::var` を読む。**preview.rs の `LAND_TIMING_DEBUG` は S13a で `DebugFlags.land_timing` に入れた（misa-biped の読みは 0）。残りは misa-mpc の 2 つ** | `DebugFlags` を引数で渡す形にする。misa-mpc の公開 API が変わるので、他の利用者（namiashi 系）と調整する | ユーザー |
| 16 | 印字の順の固定をいつやめるか | §2.5 で今の順を保つ | Walker が落ち着いたら、印字を `TickRecord` の出来事の列にして、ハーネスが tick の終わりにまとめて出す形にしてよい。その 1 回はログの差（CSV は同一）を明示したコミットにする | ユーザー |
| 17 | 実行中の指令の変更 API（§6.9） | 今は起動時に計画へ織り込む | 作らない。必要になったら、`CommandSchedule` を「先の区間だけ引き直す」再計画の要求（`PlanEditor` の書き手を 1 つ足す）として設計する | ユーザー（実機の段で） |
| 18 | `CTRL_DT` の置き場 | 名目は Config、実現値はハーネス（§4.1） | この形で確定してよいか | ユーザー |
| 19 | `QP_BENCH_GATE`／`MPC_BENCH_GATE` と `qp_physical`（診断の有無が `qp_physical` の計算を開く） | 制御の結果は変えないが、診断の設定が制御の計算経路に入っている | **済（S12a）**: `TaskStackConfig.validate` と `StackDiag { measure_violation, dump }` に分けた（§5.1 S12）。`MPC_BENCH_GATE`（`mpc_gate`）は Walker 側のまま | 済 |
| 20 | misa-biped が misa-mpc に依存するか（S12 の申し送り） | (a) path 依存、(b) MPC も trait で注入 | **済（S13、ユーザー判断）: (a)**。misa-mpc の依存は misa-wbc と nalgebra だけなので、MuJoCo・serde・articara を入れない趣旨は保てる | 済 |
| 21 | 足裏の寸法 `SOLE_HALF_*` の置き場 | 今はハーネスが環境変数を読み `WalkerConsts` に渡す | **済（S13、ユーザー判断）**: robot の `Profile::cop_half`。環境変数の上書きは `config_env::profile_from_kv` が Profile に書く（読む位置と値は同じ） | 済 |
