# `.misa` スキーマリファレンス

`.misa` は articara/misarta のネイティブマスタフォーマット。中身は TOML、
拡張子は `.misa`、最初の行は必ず `schema = "misarta/1"`。

このドキュメントは on-disk スキーマの完全リファレンス。設計議論の経緯は
[`refactor_20260502.md`](refactor_20260502.md)、他形式との機能比較は
[`comparison.md`](comparison.md) を参照。

## 1. ヘッダ

```toml
schema = "misarta/1"
```

- `<vendor>/<major>` 形式。`vendor = "misarta"`、`major = 1` 以外は
  ローダが拒否する(将来の互換しない変更時に major を bump)。
- 拡張子 `.misa` 単独では中身が判別できないため、このヘッダが
  自己説明識別子としての役割を担う。

## 2. ロボットメタ

```toml
[robot]
name = "namiashi_description"
root = "trunk"
```

| フィールド | 型 | 必須 | 説明 |
|---|---|---|---|
| `name` | string | ✓ | ロボット名(ログ・タイトル等で使用) |
| `root` | string | ✓ | ルートリンク名。`[[link]]` のいずれかと一致しなければエラー |

## 3. 共通: `Origin`

6-DoF 配置を表すサブテーブル。`xyz` は常に翻訳、回転は **`rpy` か
`quat` のどちらか一方** を使う(両方指定するとローダが拒否)。

```toml
# rpy 既定(人間編集向け)
origin = { xyz = [0.05, 0.10, 0], rpy = [0, 0, 1.5708] }

# quat 代替(USD 互換 / gimbal lock 回避用)
origin = { xyz = [0, 0, -0.10], quat = [0, 0, 0, 1] }   # x, y, z, w
```

| フィールド | 型 | 既定 | 説明 |
|---|---|---|---|
| `xyz` | `[f64; 3]` | `[0,0,0]` | 翻訳(m) |
| `rpy` | `[f64; 3]` (任意) | なし | Roll/Pitch/Yaw (rad), ZYX intrinsic |
| `quat` | `[f64; 4]` (任意) | なし | クォータニオン `[x, y, z, w]` |

すべて省略 = identity 回転。Identity の場合は serializer がフィールドを
完全省略して TOML を簡潔に保つ(`origin = {}` ではなくキー自体が消える)。

## 4. 単位と座標規約(暗黙)

- **長さ**: m
- **質量**: kg
- **角度**: rad
- **時間**: s
- **軸**: Z-up

Y-up 形式 (USD 等) への export 時は exporter が変換する。

## 5. `[[material]]`(任意)

複数の visual で共有される色定義。ユーザ命名のみ、識別子規約に従う
(`^[A-Za-z_][A-Za-z0-9_]*$`)。

```toml
[[material]]
name = "red_plastic"
color = "#cc4422"           # hex 文字列(#RRGGBB or #RRGGBBAA)

[[material]]
name = "aluminum"
color = [0.7, 0.7, 0.75, 1.0]   # RGBA 0..1
```

`color` は hex 文字列または `[r, g, b, a]` 配列のどちらか。
ローダはどちらも受理して内部表現に正規化する。

## 6. `[[link]]`

```toml
[[link]]
name = "trunk"
description = "胴体。バッテリーとIMUを内蔵"   # 任意

inertial = { mass = 5.0, ixx = 0.10, iyy = 0.10, izz = 0.10,
             ixy = 0.0, ixz = 0.0, iyz = 0.0,
             origin = { xyz = [0, 0, 0.05] } }

[[link.visual]]
origin = { xyz = [0, 0, 0] }       # identity → 省略可
geom   = { box = { size = [0.30, 0.20, 0.10] } }
color  = "#cc6644"

[[link.visual]]
origin = { xyz = [0.10, 0, 0.05], rpy = [0, 1.5708, 0] }
geom   = { mesh = { file = "meshes/trunk_decoration.stl" } }
material = "red_plastic"           # color と相互排他

[[link.collision]]
origin = { xyz = [0, 0, 0] }
geom   = { capsule = { radius = 0.04, length = 0.20 } }
```

| フィールド | 型 | 必須 | 説明 |
|---|---|---|---|
| `name` | string | ✓ | リンク名(識別子規約) |
| `description` | string | | UI tooltip 用の自由記述 |
| `inertial` | `Inertial` | | 質量・慣性テンソル・COM frame |
| `link.visual` | `Visual[]` | | 視覚ジオメトリ |
| `link.collision` | `Collision[]` | | 衝突判定ジオメトリ |

### `Inertial`

| フィールド | 型 | 既定 | 説明 |
|---|---|---|---|
| `mass` | f64 | 0 | kg |
| `ixx`, `iyy`, `izz` | f64 | 0 | 主慣性 (kg·m²) |
| `ixy`, `ixz`, `iyz` | f64 | 0 | 慣性積 |
| `origin` | `Origin` | identity | COM + 主軸 frame |

### `Visual`

| フィールド | 型 | 必須 | 説明 |
|---|---|---|---|
| `origin` | `Origin` | identity | リンク frame からの配置 |
| `geom` | `Geom` | ✓ | 形状(下記) |
| `color` | `ColorSpec` | | インライン色(`material` と相互排他) |
| `material` | string | | `[[material]]` への名前参照 |

### `Collision`

`Visual` から `color` / `material` を除いたもの。

### `Geom` タグ付き union

`geom = { <tag> = { <params> } }` の形(serde 既定 external tagging)。

| タグ | パラメータ | 備考 |
|---|---|---|
| `box` | `size = [w, h, d]` | URDF流の **全長**(half-extent ではない) |
| `cylinder` | `radius`, `length` | Z 軸円筒、`length` は全長 |
| `sphere` | `radius` | |
| `capsule` | `radius`, `length` | Z 軸、`length` は **円筒部分のみ**(総延長 = `length + 2*radius`) |
| `mesh` | `file`, `scale = [sx, sy, sz]` | `file` はマスタ相対パス |

`mesh` の `scale` は省略時 `[1, 1, 1]`。`file` はマスタファイルからの
相対パス(`meshes/trunk.stl` 等)で、絶対パスや `..` を含むと
`AssetSource` のサンドボックスチェックで拒否される。

## 7. `[[joint]]`

```toml
[[joint]]
name   = "hip_pitch"
type   = "revolute"          # revolute / continuous / prismatic / fixed / floating / planar
parent = "trunk"
child  = "thigh"
axis   = [0, 1, 0]
origin = { xyz = [0.05, 0.10, 0] }
limit  = { lower = -1.5708, upper = 1.5708, effort = 30.0, velocity = 10.0 }

# 受動物理特性: ベアリング摩擦・ロータ慣性。アクチュエータ有無に独立
dynamics = { armature = 0.0014, damping = 0.10 }
```

| フィールド | 型 | 必須 | 説明 |
|---|---|---|---|
| `name` | string | ✓ | 関節名(識別子規約) |
| `type` | enum | ✓ | `revolute` / `continuous` / `prismatic` / `fixed` / `floating` / `planar` |
| `parent` | string | ✓ | 親リンク名 |
| `child` | string | ✓ | 子リンク名(複数の joint で共有不可 — その場合は `[[loop_closure]]`) |
| `axis` | `[f64; 3]` | `[0,0,1]` | 軸ベクトル(自動 normalize) |
| `origin` | `Origin` | identity | 親 frame からの配置 |
| `limit` | `JointLimit` | 全 0 | リミット |
| `dynamics` | `JointDynamics` | 全 0 | 受動物理特性 |

### `JointLimit`

| フィールド | 型 | 説明 |
|---|---|---|
| `lower`, `upper` | f64 | 位置リミット (rad / m) |
| `effort` | f64 | 最大トルク / 最大力(0 = 制限なし) |
| `velocity` | f64 | 最大速度(0 = 制限なし) |

### `JointDynamics`

| フィールド | 型 | 説明 |
|---|---|---|
| `armature` | f64 | ロータ反映慣性(kg·m² for revolute, kg for prismatic) |
| `damping` | f64 | 受動粘性ダンピング |
| `friction` | f64 | クーロン摩擦(任意、未対応エンジンでは無視) |

## 8. `[[actuator]]` (N:M 対応)

```toml
# 1:1 (普通のサーボ)
[[actuator]]
name   = "hip_pitch_motor"
mode   = "Position"
joints = [{ name = "hip_pitch", gear = 1.0 }]
kp = 100.0
kv = 1.2

# N:1 (差動)
[[actuator]]
name   = "diff_drive_a"
mode   = "Torque"
joints = [
  { name = "wheel_left",  gear =  1.0 },
  { name = "wheel_right", gear = -1.0 },
]
kp = 0.0
kv = 5.0

# 1:N (テンドン/ケーブル駆動)
[[actuator]]
name   = "finger_tendon"
mode   = "Position"
joints = [
  { name = "finger_mcp", gear = 1.0 },
  { name = "finger_pip", gear = 0.7 },
  { name = "finger_dip", gear = 0.5 },
]
kp = 50.0
kv = 0.5
```

| フィールド | 型 | 必須 | 説明 |
|---|---|---|---|
| `name` | string | ✓ | アクチュエータ名 |
| `mode` | enum | `Position` | `Position` / `Velocity` / `Torque` / `ComputedTorque` |
| `joints` | `[ActuatorJointRef]` | ✓ (非空) | 駆動対象の関節と gear 比 |
| `kp` | f64 | 50.0 | 位置ゲイン (Position / ComputedTorque で使用) |
| `kv` | f64 | 5.0 | 速度ゲイン (Torque を除く全モード) |

### `ActuatorJointRef`

| フィールド | 型 | 既定 | 説明 |
|---|---|---|---|
| `name` | string | (必須) | 駆動対象の関節名 |
| `gear` | f64 | 1.0 | 線形カップリング係数(MJCF `gear`、tendon `coef` 相当) |

**Mimic との違い**: `[[mimic]]` は **運動学拘束**(同一自由度として扱う)、
`[[actuator]] joints=[...]` は **制御入力の分配**(独立自由度のまま 1
コマンドで動かす)。

## 9. `[[mimic]]`

```toml
[[mimic]]
joint = "FL_thigh_joint"
source = "FL_hip_joint"
multiplier = -1.0
offset = 0.0
```

`q_joint = multiplier * q_source + offset` の線形連動。両関節とも
1-DoF (`revolute` / `continuous` / `prismatic`) でなければエラー。

## 10. `[[loop_closure]]`

```toml
[[loop_closure]]
name = "five_bar_left"
link_a   = "shin_left"
offset_a = [0, 0, -0.10]                # 翻訳のみ(quat 既定 identity)
link_b   = "linkage_left"
offset_b = [0, 0,  0.05]
rot_b    = [0, 0, 0, 1]                 # x, y, z, w 任意
pose_6dof = false                       # false = 3-DoF (位置のみ), true = 6-DoF
```

`link_a` と `link_b` を 3-DoF または 6-DoF で拘束する閉ループ。木構造を
壊さずに parallel mechanism / 5-bar / 4-bar 等を表現可能。

## 11. `[[collision_pair]]`

```toml
[[collision_pair]]
link_a = "FL_hip"
link_b = "RL_hip"
enabled = false
```

ペア単位の衝突有効/無効。リスト外のペアは "collide" がデフォルト
(MuJoCo 互換)。`enabled = false` のペアは MJCF export 時
`<contact><exclude>`、USD export 時 `physics:filteredPairs` に変換。

ペアは alphabetical order に正規化されるので、`(A, B)` と `(B, A)` は
同一エントリとして扱われる。

## 12. `[[sensor]]`

```toml
[[sensor]]
name = "front_lidar"
link = "trunk"
origin = { xyz = [0.15, 0, 0.05] }
update_rate = 30.0
kind = { lidar = { range_min = 0.1, range_max = 30.0,
                   h_fov = 6.28, h_samples = 360,
                   v_fov = 0.52, v_samples = 16 } }
```

`kind` は default external tagging の union:

| タグ | パラメータ |
|---|---|
| `camera` | `fov`, `width`, `height`, `near`, `far` |
| `lidar` | `range_min`, `range_max`, `h_fov`, `h_samples`, `v_fov`, `v_samples` |
| `imu` | `gyro_noise`, `accel_noise` |
| `force_torque` | `joint` (任意 — 省略時は parent joint) |
| `contact` | `partner` (任意 — 省略時は any contact) |
| `generic` | `kind`, `params` (BTreeMap<String, String>) — 未モデル化センサ用エスケープハッチ |

## 13. `[[pose]]`

```toml
[[pose]]
name = "stand"
duration = 0.5
kind = "QuinticSmooth"   # Linear / CubicSmooth / QuinticSmooth
angles = { hip_pitch = 0.0, knee = 0.7, ankle = -0.7 }
```

ユーザ定義の名前付き関節姿勢。シーケンス再生やスナップショットで使用。
`angles` に含まれない関節は再生時の現在値を保持。

## 14. `[[sequence]]`

```toml
[[sequence]]
name = "walk"
steps = [
  { pose_name = "stand", duration = 0.5, kind = "QuinticSmooth" },
  { pose_name = "step",  duration = 0.3, kind = "QuinticSmooth" },
]
```

`[[pose]]` を順に実行する遷移シーケンス。各 step の `duration` は
**前の step (or 現在状態) からの遷移時間**。

## 15. `[[gait]]`

四脚歩容のプリセット。

```toml
[[gait]]
name = "trot"
gait_type = "Trot"            # Trot / Walk / Pace / Bound
cycle_period_s = 0.4
duty_factor = 0.5
swing_height_m = 0.05
max_step_length_m = 0.10
fl_foot = "FL_foot"
fr_foot = "FR_foot"
rl_foot = "RL_foot"
rr_foot = "RR_foot"
knee_forward = [true, true, false, false]   # FL, FR, RL, RR
```

## 16. `[home]`

ロード直後に適用される初期姿勢。

```toml
[home]
joint_positions = { hip_pitch = 0.0, knee = 0.5 }
base_position = [0, 0, 0]
base_orientation = [0, 0, 0, 1]    # x, y, z, w
```

`joint_positions` に含まれない関節はモデルの中立位置を保持。
`base_position` / `base_orientation` は floating-base ルート姿勢。

## 17. 識別子規約

すべてのエンティティ名 (link / joint / material / sensor / pose /
sequence / actuator / mimic / loop_closure / gait) は以下に従う:

```
^[A-Za-z_][A-Za-z0-9_]*$
```

ローダはこの規約に違反する名前を自動修正:

- `-` → `_` に置換
- 空白 → `_` に置換
- 識別子外文字 → 削除
- 先頭が数字 → `_` プレフィックス
- 結果が空 → `_`

すべての修正は `LoadReport.sanitized_names` に記録される(GUI
ダイアログで一覧表示される予定)。

## 18. メッシュ参照のサンドボックス

`Geom::Mesh.file` はマスタファイル相対のみ受理。以下は **拒否**:

- 絶対パス (`/path/to/foo.stl`, `\path`, `C:\path`)
- `..` を含むパス (`../escape/foo.stl`)
- 空文字列

URDF の `package://name/sub/foo.stl` は import 時に `sub/foo.stl` に
正規化される。

## 19. ファイル配置

```
robots/<name>/
├── <name>.misa            # マスタ
└── meshes/
    ├── trunk.stl
    ├── thigh.stl
    └── ...
```

メッシュは `meshes/` サブディレクトリ前提。`AssetSource` 実装は
このディレクトリ階層の外への参照を拒否する。

## 20. ローダ・ライタ

| API | 説明 |
|---|---|
| `misarta::native::load(path) -> ParseOutput` | ファイルから `MisaFile + LoadReport` をロード |
| `misarta::native::save(path, &MisaFile)` | `MisaFile` をファイルに書き出し |
| `misarta::native::parse_str(text, &dyn AssetSource)` | 文字列+ assets からパース(fs 非依存) |
| `misarta::native::write_str(&MisaFile)` | `MisaFile` を TOML 文字列に |
| `misarta::native::build_model(&MisaFile)` | `MisaFile` → `Model + visual GeometryModel + collision GeometryModel` |
| `RobotModel::from_misa(path)` | articara 側の `RobotModel` ロード(報告破棄) |
| `RobotModel::from_misa_with_report(path)` | `LoadReport` 付きでロード(GUI ダイアログ用) |
| `RobotModel::to_misa()` | `RobotModel` から `MisaFile` を構築 |
| `RobotModel::save_as_misa(path)` | `to_misa` + ファイル書き出しの便利関数 |

設計詳細は [`refactor_20260502.md`](refactor_20260502.md) §3 (アーキテクチャ),
実装は [`misarta/src/native/`](../misarta/src/native/) 配下を参照。
