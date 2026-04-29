# ロボットモデルフォーマット比較

articara が入出力としてサポート (予定含む) するロボットモデルフォーマットの
機能対応比較表。

## マスタフォーマット (`.misarta.toml`)

articara は **`.misarta.toml` を真のマスタフォーマット**として位置付けています。
URDF / SDF / MJCF / USD はそれぞれ「派生 export 先 / import 元」となり、
編集はマスタ (= articara のメモリ上のモデル + sidecar TOML) に対して行います。

```
              ┌─ modify ─┐
              ▼          │
         XXX.misarta.toml ← マスタ
        ╱        │        ╲
   import      export    (双方向)
       ╱        │        ╲
   ▼      ▼      ▼      ▼
  URDF   SDF   MJCF    USD
```

各フォーマットの import/export は [`articara::format::FormatHandler`] trait
経由でプラグイン化されており (`FormatRegistry::default_registry()`)、新しい
フォーマットは impl を一つ書けば追加できます。

### マスタに含まれるエンティティ

| カテゴリ | 主たる保管先 | 備考 |
|---|---|---|
| Links / Joints / Inertia / Geometry | URDF/SDF/MJCF/USDのいずれか (構造ファイル) | TOML 側は重複保管しない |
| Loop closures | `.misarta.toml` `[[loop_closure]]` | 6-DoF rotation 含む |
| Pose registry | `.misarta.toml` `[[pose]]` | duration / kind 含む |
| Sequences | `.misarta.toml` `[[sequence]]` | チェーン Pose 再生 |
| Actuators (Kp/Kv/mode) | `.misarta.toml` `[[actuator]]` | per-joint |
| Collision pairs | `.misarta.toml` `[[collision_pair]]` | 有効/除外 |
| **Mimic (連動関節)** | `.misarta.toml` `[[mimic]]` | linear coupling |
| **Sensors** | `.misarta.toml` `[[sensor]]` | Camera/Lidar/IMU/F-T/Contact/Generic |

双方向往復は **best-effort**: 構造ファイル (URDF など) で表現できない要素は
`.misarta.toml` 側に記録され、export 時に対象フォーマットがサポートする形で
出力されます (例: mimic は URDF/SDF/MJCF それぞれの記法に変換、USD では警告)。


対象フォーマット:

1. **URDF** — ROS1/2 用の Unified Robot Description Format
2. **SDF** — Gazebo の Simulation Description Format
3. **MJCF** — MuJoCo の XML フォーマット
4. **USD** — Isaac Sim / Omniverse の Universal Scene Description

## 5項目の比較表

| 項目 | URDF | SDF | MJCF | USD (Isaac) |
|---|---|---|---|---|
| **Collision pair (有効/除外)** | ✗ ネイティブ無し<br>SRDF サイドカーで `<disable_collisions>` | ✓ `<collide_bitmask>` (bitmask)<br>`<self_collide>` flag<br>`<collision_filter>` | ✓ `<contact><pair>` (enable)<br>`<contact><exclude>` (disable)<br>`contype/conaffinity` bitmask | ✓ `PhysicsFilteredPairsAPI`<br>`PhysicsCollisionGroup` |
| **連動関節 (mimic)** | ✓ `<mimic joint=… multiplier=… offset=…>`<br>**線形のみ・単一source** | ✓ `<axis><mimic>`<br>(SDF 1.7+, **線形のみ**) | ✓ `<equality><joint>` 多項式係数<br>`<tendon>` (複数関節を腱で結ぶ複雑な連動も可) | ✗ ネイティブ無し<br>(Isaac 側で driver / Python script で実現) |
| **形状モデル形式** | プリミティブ: box / cylinder / sphere<br>メッシュ: STL, DAE | プリミティブ: box / sphere / cylinder / **capsule** (1.10+) / ellipsoid / plane / heightmap / polyline<br>メッシュ: STL, DAE, OBJ | プリミティブ: box / sphere / cylinder / capsule / ellipsoid / plane / hfield<br>メッシュ: STL, OBJ, MSH (プラグイン経由) | UsdGeom 全種 (Cube/Sphere/Cylinder/Cone/Capsule/Mesh/…)<br>外部参照: USD/OBJ/FBX (plugin) |
| **閉リンク機構** | ✗ **木構造のみ**<br>(workaround: ROS の transmission や外部 constraint で分解) | ✓ ネイティブ対応<br>(joint で任意リンク同士を接続可) | ✓ `<equality><connect>` 点拘束<br>`<equality><weld>` 6-DoF 拘束<br>`<equality><joint>` 関節間拘束 | ✓ Joint prim で任意 RigidBody 接続可<br>(ループでも宣言可、solver 安定性は要注意) |
| **センサ** | ✗ ネイティブ無し<br>(`<gazebo>` extension で SDF を埋め込む慣例) | ✓ camera / lidar (ray) / imu / force_torque / contact / gps / magnetometer / altimeter / sonar | ✓ accelerometer / gyro / framepos / framequat / framelinvel / jointpos / jointvel / jointactuatorfrc / force / torque / touch / tendonpos など多数 | 拡張 prim (Isaac 特有): RTX LiDAR / Camera / IMU / ContactSensor / ProximitySensor<br>(USD core 定義は無し) |

## 補足比較 (articara から見た重要項目)

| 項目 | URDF | SDF | MJCF | USD |
|---|---|---|---|---|
| **関節型** | revolute / continuous / prismatic / fixed / planar / floating | revolute / prismatic / ball / universal / fixed / screw / revolute2 | hinge / slide / ball / free | Revolute / Prismatic / Spherical / Distance / Fixed / D6 |
| **慣性パラメータ** | mass + 3×3 慣性テンソル (origin 指定可) | 同上 | mass + diaginertia or fullinertia | physics:mass + UsdPhysics:Inertia |
| **アクチュエータ** | ✗ (transmission で部分対応) | `<actuator>` (SDF 1.7+) | ✓ position / velocity / motor / muscle / general | Drive API (PhysicsDrive) |
| **マテリアル/色** | `<material>` (RGBA + texture) | 同上 | rgba on geom + `<texture>` asset | UsdShade Material network |
| **物理エンジン互換** | (description only) | Gazebo (ODE/Bullet/DART/Simbody) | MuJoCo native | PhysX (Isaac) |
| **再ロードしやすさ (テキスト構造)** | XML, シンプル | XML, やや冗長 | XML, 階層 body | テキスト USDA / バイナリ USDC, 大規模 |

## 要点と articara の現状 (実装済み)

### Collision pair (実装済み)

URDF は SRDF サイドカー必須なので、articara は `.misarta.toml` を独自サイドカー
として使い、MJCF export 時 `<contact><exclude>` / USD export 時
`physics:filteredPairs` に変換する形で正規化済み。

### 連動関節 (mimic) (実装済み: master format Phase 4-5 / `4147011`)

`misarta::config::MimicConfig` + `RobotModel.mimics` で保持。
- URDF: `<mimic joint=… multiplier=… offset=…>` 入出力
- SDF: `<axis><mimic>` 入出力
- MJCF: `<equality><joint polycoef="off mult 0 0 0">` 入出力
- USD: 表現方法がないため export 時に `log::warn!`

### 形状 (capsule fallback 済み: Sprint X / `df1f5d0`)

articara の `GeomData::Capsule` は URDF export 時に **cylinder + 2 spheres**
に decompose される (`urdf_export_decomposes_capsule_into_cylinder_and_spheres`
回帰テスト済み)。

汎用的な「format が表現できない要素」の事前警告は **本回で追加** (下記
"Pre-export 互換警告" 参照)。

### 閉リンク (実装済み: Sprint X / `df1f5d0`)

`misarta::LoopClosure` で保持し、`.misarta.toml` `[[loop_closure]]` で永続化。
- MJCF: 3-DoF は `<equality><connect>`、6-DoF は `<equality><weld relpose>` で出力
- URDF: 木構造のみなので export では失われる (sidecar で保持)
- 回帰テスト `mjcf_export_emits_loop_closure_connect_and_weld` 済み

### センサ (実装済み: master format Phase 4-5 / `56aed10`, `4147011`)

`misarta::config::SensorConfig` の master 表現 (`Camera`/`Lidar`/`Imu`/
`ForceTorque`/`Contact`/`Generic`) を `RobotModel.sensors` に保持。
- SDF: `<sensor>` 各種に変換
- MJCF: 各 type を `<sensor><accelerometer>` / `<gyro>` / `<touch>` などに変換
- URDF: ネイティブ非対応のため drop (sidecar で保持)
- USD: 現時点では log::warn! (Isaac 拡張への変換は未実装)

### Pre-export 互換警告 (実装済み: 本回追加)

`format::FormatCapabilities` を使い、export 前にモデル中の要素と
ターゲットフォーマットの能力を突き合わせて、差分があればダイアログで
ユーザに通知する仕組み。

例: URDF にエクスポートしようとしたが model に sensor が 3 件ある場合、
「Sensors (3) — URDF はネイティブ非対応。.misarta.toml には保持されます。
続行しますか? [Continue] [Cancel]」を表示。

実装: `analyze_export_compatibility(model, handler) -> Vec<ExportIssue>` +
properties_panel の Save / Export ボタンが pending 状態経由でダイアログを
出す。何の問題もなければダイアログをスキップして直接 export。

### Home pose 永続化 (未実装 — 候補)

現状 `joint_positions` (現在の関節角) と `base_transform` は sidecar に
保存されない。`[[pose]]` で明示的に保存しないとセッション間で消える。
`.misarta.toml` の `[home]` セクションを追加して自動往復するのが次の
高優先候補。

### USD articulation drive (未実装)

USD export の `Drive API` (PhysicsDrive) は未対応。Isaac との連携を
重視する場合の高優先項目だが、API 階層がやや複雑なので別作業。

## 推奨ロードマップ (更新版)

| 優先度 | 項目 | 状態 |
|---|---|---|
| ✅ DONE | **連動関節 (mimic) 対応** | master format Phase 4-5 で完了 |
| ✅ DONE | **MJCF → 閉リンク export** | Sprint X で完了 |
| ✅ DONE | **センサの最小サポート** | master format Phase 4-5 で完了 |
| ✅ DONE | **形状の format-specific 制限を明示** | 本回で完了 (汎用 pre-export 警告) |
| ★★ | **Home pose の sidecar 往復** | `[home]` セクション追加で `joint_positions` / `base_transform` を自動保存 |
| ★ | **USD articulation drive 対応** | Isaac 連携用 PhysicsDrive 出力 |
| ★ | **Round-trip integrity test 拡充** | 各 format で save→load→save が安定することを CI で検証 |
