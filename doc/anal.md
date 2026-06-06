# Go2 MotionProcessor (RK3588S) ハードウェア解析メモ

Unitree Go2 の MotionProcessor（メインボード）を実機から解析した記録。
`lsusb` で見つかった `2a5b:3c6d unitree.CN USB Quad_Serial` の正体・ドライバ・通信内容を、
`usbmon` トレースとファーム解析から特定した。

- 対象: `root@Unitree`（RK3588S, Linux `5.10.176-rt86+` **PREEMPT_RT** カーネル, aarch64）
- 解析日: 2026-06-05

---

## 1. 結論（要約）

- **`USB Quad_Serial` = Unitree 独自の 8ch USB→UART ブリッジ**。
  - VID/PID `2a5b:3c6d`（`unitree.CN`）。WCH **CH9344/CH348** 互換プロトコルを実装。
  - ドライバ `gbunich` は **WCH 製 CH9344 ドライバ (`ch9344ser.ko`) をリネームしたもの**。
    決め手は `modinfo` の第2エイリアス `usb:v1A86pE018`（`1A86`=WCH/QinHeng, `E018`=CH9344）。
  - デバイスノードは `/dev/gbunich0`〜`/dev/gbunich7`（char major 168）。標準 `ttyACM/ttyUSB` ではない。
- **8 ポートの割り当て**（下表）。`4Mbps × 4` が **4 脚のモータ制御バス** で、これが "Quad" の由来。
- **歩行制御の本体は RK3588S 自身**（RT-Linux 上の `basic_service`）。
  生のモータ指令フレームが USB に流れている＝間に制御 MCU は挟まっていない。
- メインボード上の **補助 MCU（写真の "MCU"／辅MCU）** は **Cortex-M 級のコプロセッサ**。
  IMU 融合・USB ブリッジ・電源/安全を担う。Unitree 独自ファーム（暗号化）。
- 上位の AI/知覚コンピュータとは **Ethernet(`eth0`)/DDS** で接続。`go2-gait-runner` はこの経路。

---

## 2. ポート割り当て表（確定）

| ポート | baud | 占有プロセス | 用途 | 確度 |
|---|---|---|---|---|
| `gbunich0` | 4,000,000 | `basic_service` | **FR（右前）モータバス** | ✅ 実測 |
| `gbunich1` | 4,000,000 | `basic_service` | **FL（左前）モータバス** | ✅ 実測 |
| `gbunich2` | 4,000,000 | `basic_service` | **RR（右後）モータバス** | ✅ 実測 |
| `gbunich3` | 4,000,000 | `basic_service` | **RL（左後）モータバス** | ✅ 実測 |
| `gbunich4` | 921,600 | `basic_service` | **デバッグログ出力（ASCIIテキスト, 送信のみ）** | ✅ 実測 |
| `gbunich5` | 230,400 | `uwb_runner` (utrack) | **UWB 測位** | ✅ |
| `gbunich6` | 2,000,000 | `unitree_lidar_dds_node` | **LiDAR** | ✅ |
| `gbunich7` | 3,000,000 | `basic_service` | **補助 MCU 直結チャネル（IMU 融合出力 ＆ ファーム更新口）** | ✅ |

> 脚の物理対応は SDK 論理順（`LowState.motor_state[]` = FR→FL→RR→RL）と完全一致。
> 各脚バス上の `0x10/0x11/0x12` が股/腿/膝の 3 関節。

---

## 3. アーキテクチャ

```
[上位 AI/知覚コンピュータ]
        │  eth0 / DDS   ← go2-gait-runner はここを叩く
        ▼
[MAIN CPU: RK3588S]  歩容・全身制御ループ（PREEMPT_RT Linux, ~1kHz, basic_service）
        │  内部USB  "unitree.CN USB Quad_Serial"  (Bus001 Dev002)
        ▼
[補助MCU = USB 8ch シリアルブリッジ + IMU融合 + 電源/安全]   （Cortex-M級, Unitree独自FW）
        ├ port0-3 → 4脚モータバス（FE EE 指令を中継, 4Mbps）
        ├ port4   → デバッグログ（テキスト, 921600）
        ├ port5   → UWB測位（230400）
        ├ port6   → LiDAR（2Mbps）
        └ port7   → 補助MCU自身（IMUクォータニオン出力 ／ McuBootによるFW更新, 3Mbps）
        ▼
[各モータ内蔵MCU ×12]  FOC電流ループ・局所保護（過電流/過熱）
```

### 制御レートの階層（分業）

- **RK3588S** … "どう動きたいか"（インピーダンス指令: q, dq, τ, kp, kd）を ~1kHz で配信。
- **各モータ内蔵 MCU** … 数十kHz の FOC 電流ループ・自律保護を実行。
  通信が切れても自分で止まれるよう局所保護を持つ。
- **補助 MCU** … 制御ブレインではなく、I/O ハブ＋センサ融合＋安全のコプロセッサ。

---

## 4. 通信プロトコル（モータバス）

`usbmon`（`/sys/kernel/debug/usb/usbmon/1u`）で観測した、4Mbps 脚バス上のフレーム。

### CH9344/CH348 の USB 多重化

| エンドポイント | 方向 | 内容 |
|---|---|---|
| `Bo:002:2` (EP2 OUT) | host→dev | モータ指令。**先頭1バイト = ポート番号(00-03)= 脚** |
| `Bi:002:2` (EP2 IN) | dev→host | モータ応答。`[ポート][長さ][ペイロード…]` を連結 |
| `Bi:002:1` (EP1 IN) | dev→host | 各ポートのライン/モデム状態通知（`80/81/82/83`=port0-3）。管理用 |

### フレームヘッダ（Unitree モータプロトコル）

- **`FE EE` = 下り（host→モータ指令）**
- **`FD EE` = 上り（モータ→host フィードバック）**
- フィードバックは `FD EE <id>` の後に **int16(LE)** で 位置・速度・トルク・温度 が並ぶ。
- 末尾2バイト = **CRC16-CCITT**（MCU 更新ツールと同じ CRC 系統）。

### ラウンドロビン制御ループ

OUT 指令は port 00→03（4脚）へ、関節ID `10→11→12`（股/腿/膝）を巡回。
1 サイクルで 12 モータを更新する高速ループ。

```
port0: 00 | .. | fe ee 12 | ...   脚0 膝
port1: 01 | .. | fe ee 12 | ...   脚1 膝
port2: 02 | .. | fe ee 12 | ...   脚2 膝
port3: 03 | .. | fe ee 12 | ...   脚3 膝
 … 次ティックで fe ee 10（股）, fe ee 11（腿）…
```

---

## 5. IMU（port7）

- port7 のフィードバックは **float32(LE)** が並ぶ＝姿勢推定(AHRS)出力。
- 繰り返し出現する `[0.806, 0.027, -0.037, 0.59]` は **ノルム≈1.000 の単位クォータニオン**。
  → port7 = IMU で確定。後続に gyro(rad/s) / accel(m/s², 静止で1軸≈9.8) が続く。
- 並びは Unitree `IMU_State`（`quaternion[4] / gyroscope[3] / accelerometer[3] / rpy[3] / temperature`）に対応。
- IMU は補助 MCU が生センサを読んで融合し port7 に出力している（後述の FW 更新口と同一ポート）。

---

## 6. デバッグログ（port4）

- port4 は **送信(OUT)のみ・応答(IN)なし** → センサではなく **基板上のデバッグ UART への
  プレーンテキスト出力**（`basic_service` の全身ステータス・ダンプ）。
- フレーミング: `[04][len16(LE)][payload…]`（先頭3バイトがヘッダ、以降が ASCII）。
- 区切り: 行は `\n`、フィールドは `;`、ヘッダ部と脚ブロックの間は空行（`0a 0a`）。
- ※ 本モデルは足裏センサ非搭載。port4 は足裏ではなく診断ログ用途だった。

### レコード全体（約1秒ごと＝制御ループ約1000回ごとに1レコード出力）

```
Rev:   1829697;Cun:   1829695;                          ← ループカウンタ
Roll    -0.71;Pitch   -5.77;Yaw    -41.13;Tem 79.7      ← IMU姿勢(deg) + 温度
IPC    0                                                 ← プロセス間通信ステータス
Bms:3;T:  30;  32;  30;  28;   0;   0;Ste:0             ← バッテリ(BMS)
Ste:0;  9410                                             ← 状態 + 値

FR;    0    0    0;        0        0        0;Err 0 0 0;Tem 34 32 33;Pos -0.083 1.250 -2.789;Hz 606 606 606;
FL;  …                                                   ← 4脚 ×（各3関節）
RR;  …
RL;  …
BL;    0    0    0;    0    0    0;Err 0 0 0;   ×4        ← 予約スロット(本モデル未使用)
```

### ヘッダ部フィールド

| トークン | 意味 |
|---|---|
| `Rev / Cun` | ループ回数カウンタ（毎レコード約 +1000 ＝ **約 1kHz 制御ループ**。Rev=指令側 / Cun=完了側、差≈2） |
| `Roll / Pitch / Yaw` | **IMU 姿勢角(度)**（port7 の MCU 由来） |
| `Tem`（ヘッダ） | 温度(℃)。値が高め(≈79.8)で **SoC/基板温度** と推定 |
| `IPC` | プロセス間通信の状態（0=正常） |
| `Bms:N;T: …;Ste:N` | **バッテリ BMS**：状態 + セル/センサ温度 ×4(+予備2) + 状態。※BMSはCH348ポート外、別経路取得値をログ出力 |
| `Ste:N;  V` | 状態 + 値（電圧/電流系の生値と推定、未確定） |

### 脚ブロックフィールド（`;`区切り）

| # | フィールド | 例 | 意味 |
|---|---|---|---|
| 1 | leg | `FR` | 脚名（port0-3 対応） |
| 2 | 3値(狭) | `0 0 0` | **速度 dq** と推定（静止で0、要実測確認） |
| 3 | 3値(広) | `0 0 0` | **トルク tau** と推定（静止で0、要実測確認） |
| 4 | `Err` | `0 0 0` | 3関節エラーフラグ |
| 5 | `Tem` | `34 32 33` | **3モータ温度(℃)** |
| 6 | `Pos` | `-0.083 1.250 -2.789` | **3関節角(rad)** |
| 7 | `Hz` | `606 606 606` | **各モータの実通信レート(Hz)** ＝実測 約530〜606Hz |

> 重要: `Rev` は ~1kHz で回るが、`Hz`（モータバス実効レート）は **~540Hz**。
> 4Mbps バス上のモータ往復が実測 540Hz 程度で回っている、という制御の実態が読める。
> `BL;` ×4 は全ゼロの予約スロット（追加アクチュエータ用の枠、本モデルでは未使用）。

### 未確定の残タスク

- 脚ブロック field2/3（速度 dq / トルク tau）の確定: 1脚を手で動かし、非ゼロになる項目で判別。
- ヘッダ `Tem 79.8`（SoC か IMU/MCU か）と `Ste 9410` の出所・単位の確定。

---

## 7. 補助 MCU のファームウェア

`/unitree/dev/go2_firmware_tools/firmware/mcu/` に MCU 専用ファーム＆書き込みツールが存在。

### `McuBoot`（25.8KB, ELF aarch64, host 側ツール）

- RK3588S 上で動く **シリアルブートローダ書き込みツール**。
- オプション: `-D device` / `-S speed` / `-v verbose` / `-f hardflow`。
- 対象デバイス **`/dev/gbunich7`** をハードコード（＝補助MCUの管理チャネル）。
- **カスタムボーレート**を `TCGETS2/TCSETS2` + `libtty_setcustombaudrate` で設定
  （＝`stty` で "0 baud" に見えた **BOTHER/termios2** の正体）。
- ページ単位プログラミング（`page/single/size`, `SendData`, `McuRevDataCheck`）。
- CRC: `crc16_ccitt`, `auchCRCHi/Lo`(Modbus系), `crc32_core`。
- 手順: `/unitree/robot/tool/boot/AppPublic.bin` を読み、コマンド `start_Mcu_Upgrade` で焼く。
  戻り値 0=OK / 2=Fail / 3=Timeout / 4=state error / 5=write error / 255=Error。
- チップ純正 ROM ブートではなく **Unitree 独自ブートローダ**。

### `AppPublic.bin`（61KB, 補助MCUアプリ本体）

- 先頭 `ba 33 98 d9 0e 2d 2d 6e …` と高エントロピー。
  Cortex-M のベクタテーブル（初期SP `0x2000_xxxx` / リセットベクタ `0x0800_xxxx`）の痕跡なし。
  → **暗号化(または圧縮)されている**。MCU 内ブートローダが復号して書き込む方式。
- そのため **ソフトからチップ型番は特定不可**。サイズ ~60KB は Cortex-M アプリ相当。

---

## 8. 補助 MCU の位置づけ（残る曖昧さ）

ソフト証拠だけでは下記 2 解釈が残る。物理シルク印字で確定可能。

- **(A) 補助MCU = USB Quad_Serial ブリッジ本体**（CH348互換を自前エミュ）。全8ポートが MCU 経由。
  safety/e-stop でモータ指令に介入できるのはこの場合。
- **(B) ブリッジは実物の WCH CH348**（VIDのみ Unitree に書換）、
  **補助MCU は port7 の先にぶら下がる別 Cortex-M**（IMU融合＋現場更新対応）。

確実なのは「**補助MCU は port7 上に居り、IMU融合と自ファーム更新を担う Cortex-M 級チップ
（Unitree独自FW・暗号化・独自シリアルブートローダ）**」という点。

### 確定のための残タスク

- 写真の "MCU" チップの **表面シルク（型番）** を読む（`STM32F4/G4`, `GD32` 等）。
  周囲に別の WCH チップが無ければ (A)、小さな QFN の WCH がいれば (B)。
- `ls -la /unitree/dev/go2_firmware_tools/` と更新スクリプト/設定の確認（型番直書きの可能性）。

---

## 9. メインボード構成（teardown 写真より）

| ラベル | 内容 |
|---|---|
| MAIN CPU | RK3588S（ヒートシンク下） |
| MCU（辅MCU） | 補助マイコン（ETHER 隣。本解析の主役） |
| ETHER | 有線 LAN（`eth0`） |
| WIFI / GPS | 無線モジュール（左上） |
| 右上コネクタ群 | 脚ハーネス＋センサコネクタ |
| MTRS | モータ配線（下部） |
| FAN ×2 / HEAT SINK | 冷却 |
| 右下モジュール(QR) | 4G/セルラー or 無線モジュール |

---

## 10. 解析に使ったコマンド（再現用）

```bash
# デバイス特定
lsusb
lsusb -v -d 2a5b:3c6d | grep -iE "bInterfaceClass|bNumInterfaces|iProduct|iManufacturer|bcdDevice"
ls -l /sys/bus/usb/devices/1-1*/driver
modinfo gbunich           # 第2エイリアス usb:v1A86pE018 = WCH CH9344

# ポートを握るプロセス（lsof が無いので /proc 経由）
fuser -v /dev/gbunich* 2>/dev/null
for pid in <PID...>; do cat /proc/$pid/comm; tr '\0' ' ' < /proc/$pid/cmdline; done

# ボーレート（標準）
for n in 0 1 2 3 4 5 6 7; do stty -F /dev/gbunich$n speed; done
# カスタムボーレート（termios2, "0 baud" の実値を読む）
python3 -c 'import fcntl,struct;b=bytearray(44);fcntl.ioctl(open("/dev/gbunich0","rb"),0x802C542A,b);print(struct.unpack("II",b[36:44]))'

# 通信内容トレース
modprobe usbmon
cat /sys/kernel/debug/usb/usbmon/1u | head -50

# ファーム
find /unitree -iname '*mcu*' 2>/dev/null
file      /unitree/dev/go2_firmware_tools/firmware/mcu/*
strings -n4 /unitree/dev/go2_firmware_tools/firmware/mcu/McuBoot
od -A x -t x1z /unitree/dev/go2_firmware_tools/firmware/mcu/AppPublic.bin | head
```

---

## 付録: 脚番号の実測手順

ロボットを **伏せ＋ダンピング（脱力）** にし、`usbmon` を流しながら 1 脚ずつ手で膝(0x12)を
振り、位置フィールドが変化する port を見て対応付け。結果:
`port0=FR(右前) / port1=FL(左前) / port2=RR(右後) / port3=RL(左後)`（SDK 論理順と一致）。
