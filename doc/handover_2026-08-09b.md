# 引き継ぎ ── 横移動マージンの原因究明から、天井と構え姿勢まで(2026-08-09、2回目)

同日 `doc/handover_2026-08-09.md` の続き。前回の引き継ぎは「優先度リスト5項目を
消化し、メカ再設計が v3〜v6 まで進み、未検証の気づきが2件残った」で終わっていた。
**その未検証の1件を潰しにいったら、機体の天井そのものまで届いた。**

詳細は `doc/kyo46rs_biped_wbc.md` **§28〜§35**、計画は
`doc/kyo46rs_next_improvements_plan.md` §1.2/§3.1/§5.1。ここは
「次に座った人が最初の10分で知るべきこと」だけ。

---

## 0. 一行で

**横移動マージンの低下はメカのせいではなく「WBC が何も指令していない自由軸」が
原因**(§28)。そこから **v5 が撤去したのはトルクではなく胴体質量だった**ことが
判明し(§31)、**天井そのものは足裏の幅**で、**前進の壁も同じ壁だった**(§32)。
足裏は人体比で正しく、**外れているのは立脚幅**(§33)。推奨3手を積むと
**28/28・safe 0.073(出荷比 4.1 倍)**(§34)。**構えたまま歩ける**、ただし
腕を保持したときだけ(§35)。

---

## 1. リポジトリの状態

### articara ── **24コミット未push**(前回同様、pushしないよう指示されている)

今日の9コミット:

```
5a7857d doc(biped): a guard stance walks, but only with the arms held
738e231 doc(biped): stack the recommendations -- 28/28, and none works alone
136c240 doc(biped): the sole is human-proportioned; the stance is 2.5x too wide
c66ba3a doc(biped): the forward wall is the lateral wall -- sole WIDTH moves both
229c6fc feat(biped): fill in the whole 24 -> 12 accounting; v5 removed the wrong thing
9abeaad feat(biped): hold the arms at their own level -- the flailing was never measured
f3b4378 doc(biped): reshoot the demo clips on both models
e29ae94 feat(biped): the momentum task has no place to stand in a strict hierarchy
f910301 feat(biped): the lateral margin was lost to an uncommanded axis, not to the mech
```

push するかは**ユーザに確認すること**。

### kyo46rs_description ── **無変更**。remote は依然無し

**今日の推奨(§34)はすべて URDF 変更だが、まだ一切適用していない。**
実験用の変種は `/tmp/kyo46rs_vy_regression/urdf/` に生成されるだけで、
`scripts/kyo46rs_vy_regression.py` を実行すれば git から再生成される
(スクラッチが消えても失われない)。**stale variant 問題(前回引き継ぎ §1)は
未対処のまま。**

### quadruped-gait ── 前回通り**触らないこと**。無変更。

---

## 2. 新しく入った道具

| 何 | どこ | 何のため |
|---|---|---|
| `scripts/kyo46rs_vy_regression.py` | 新設 | 横移動の梯子。URDF 変種を git から自動生成(`v2base`〜`v6`、`v5only`/`v6only`/`v6fixed`/`v6_rec`/`v6_rec_free`) |
| `scripts/kyo46rs_demo_video.py` | 新設 | デモ 5 本の撮影。前回の撮影コマンドが残っていなかったので |
| `ARM_HOLD` / `KP_ARM` / `KD_ARM` | `kyo46rs_walk.rs` | 肩・肘だけの posture を posture より 1 段上に(§30) |
| `MOM` / `KP_MOM` / `MOM_AXES` / `MOM_LEVEL` | 同上 | 重心角運動量タスク(§29) |
| `SHOULDER_ROLL` | 同上 | 肩ロールの姿勢シード(§28.5、単体では効かない) |
| `joint travel peak-to-peak` 行 | 同上の出力 | **腕の振り回しが自己干渉カウントに映らない**問題への対処(§30.1) |
| `arms down` / `guard` 条件 | `kyo46rs_bench.py` | 押しベンチで構え姿勢を測る |

---

## 3. 結論(数字)

梯子は `|VY| = 0.018〜0.073`、両方向 × 先行遊脚両方 = 28 セル。

| 構成 | safe [m/s] | 生存 |
|---|---|---|
| `v6` 出荷 | 0.018 | 6/28 |
| `v2base` 再設計前 | 0.050 | 16/28 |
| **A+B+F′(推奨)** | **0.073** | **28/28** |
| **A+B+F′ + 構え + `ARM_HOLD`** | **0.055** | **23/28** |

**推奨3手(すべて URDF 変更、制御コードの変更は不要)**:

- **A. 肩ロール軸を `fixed`**(§31.4)
- **B. 胴体質量を 0.75 → 1.274 kg に戻す**(§31.1、バッテリ・基板の配置で埋める)
- **F′. 足裏幅 38 → 60 mm、立脚幅は 100 mm 据え置き**(§33.5)

**順序が重要**: B を単独で先に入れると 12 → 6/28 に**悪化**し、F′ を単独で入れても
6 → 5/28 で**無効**。A → B → F′ の順に、まとめて。

**構えを使うなら A は不要** ── `ARM_HOLD` を入れれば軸を固定してもしなくても
22 対 23 セルで差が無く、軸を残した方が外転を含む姿勢を選べる(§35.2)。

---

## 4. 罠(今日踏んだもの、5件)

1. **自己干渉ティック数を症状の代理指標にしていた**(§30.1)。44 tick の
   クリップで腕が 60° 振れる。→ `joint travel peak-to-peak` を出力に追加済み。
2. **「加法的でない」を「意味が無い」と読んだ**(§28.2 → §31.3)。非加法性
   そのものが機序の情報だった。**2 条件が独立でないときは 2×2 を埋めるまで
   片方だけの結論を書かない。**
3. **1 条件だけ見て「効く」と書きかけた**(§32.5)。μ=2.0 が 8→16/24 だったので
   「摩擦が効く」と書き始めたが、0.8〜1.5 を埋めたら 6/7/10/10 で単調ですらない。
   **両端ではなく間を埋める。**
4. **コメントを信じてバグを捏造しかけた**(§29.7)。`src/biped/tasks.rs` が
   「`joint_positions` は burn-in で固定」と書いていたが実際は毎 tick 更新される。
   デバッグ出力 1 本で 5 分で否定できた。コメントは修正済み。
5. **ロガーの関節リストに新関節を足し忘れると CSV に列が無い**(§29.7)。
   前回の引き継ぎの罠リストにあった話をそのまま踏んだ。`profile.rs` の
   `log_joints` に追加済み。

---

## 5. 次にやること(優先順)

1. **推奨 A/B/F′ を `kyo46rs_description` に適用するか決める。** ユーザ判断待ち。
   適用するなら v7 として 1 コミット、順序は文書どおり。適用後に押しベンチと
   歩容ベンチ(42 ケース)を回し直す ── **今日の測定は全部横移動の梯子だけで、
   前進・旋回・スクワットの回帰は見ていない。**
2. **前進の壁 `VX=0.127`**(§34.2)。推奨構成でも残る。plan §5.1 の宿題は
   ここから先で、CoP 箱使用率は飽和して使えないので **ZMP クランプ**で見ること。
3. **不等式(CBF)版タスク**(§29.4 D)。角運動量でも `ARM_HOLD` でも同じ壁に
   当たった ── 厳密階層ではレベルが「全部持っていく」か「何も取れない」かの
   二値で、ゲインが強さを決めない。閾値まで無コストな形にできれば両方が解ける。
   §26.2 の `joint_limit_cbf` ゲイン設計と同じ土俵。
4. ~~構え姿勢での押しベンチ~~ ── **測った(§35.5)。構えは外乱耐性を
   損なわない。** 512 run で合計 175 → 183/256、8 セル中 5 改善・2 悪化、
   **最弱セルの safe 力積が 0.500 → 0.700 N·s(+40%)**。CoP 縁滞在は動かない。
   **構えの代償は横移動 safe の -25% だけで、押されたときの粘りには
   払っていない。** `csv/push_guard.csv`。
5. **`kyo46rs_description` の stale variant 群**(前回引き継ぎ §1、未対処)。
   `sole46/52/60`・`stance90〜130` は v3 以前の base 由来。ただし **F′ を
   適用するなら `sole60` は base に取り込まれるので、その分は不要になる。**

---

## 6. 動画

| ファイル | 構成 |
|---|---|
| `videos/demo_*.mp4` | 出荷 v6(対策なし)── **肩ロールが映像に出た最初の版** |
| `videos/demo_*_hold.mp4` | v6 + `ARM_HOLD`(§30) |
| `videos/demo_*_noadd.mp4` | v6 + 内転 ROM 除去(§29.5、`left` は転倒) |
| `videos/demo_*_rec.mp4` | **推奨構成 A+B+F′**(§34) |
| `videos/demo_*_guard.mp4` | **推奨構成 + 構え + `ARM_HOLD`**(§35) |
| `videos/cmp_vy055_v6.mp4` / `_v6_rec.mp4` | 出荷が転倒する `VY=0.055` の撮り分け |

いずれも `scripts/kyo46rs_demo_video.py` で再現できる。
