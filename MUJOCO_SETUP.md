# MuJoCo Setup Guide for articara

このプロジェクトを MuJoCo 対応でビルドするには、以下の手順に従ってください。

## 📋 前提条件

- Rust (cargo) がインストール済み
- MuJoCo 3.8.0 がインストール済み

---

## 🍎 macOS

### 1. MuJoCo をインストール（Homebrew）

```bash
brew install --cask mujoco
```

### 2. 環境をセットアップ

```bash
cd /path/to/articara
source ./setup-mujoco.sh
```

### 3. ビルド・実行

```bash
cargo run --features mujoco
```

---

## 🐧 Linux

### 1. MuJoCo をインストール

```bash
# ホームディレクトリにダウンロード
mkdir -p ~/.mujoco
cd ~/.mujoco
wget https://github.com/google-deepmind/mujoco/releases/download/3.8.0/mujoco-3.8.0-linux-x86_64.tar.gz
tar -xzf mujoco-3.8.0-linux-x86_64.tar.gz
```

または `/opt/mujoco` に配置します。

### 2. 環境をセットアップ

```bash
cd /path/to/articara
source ./setup-mujoco.sh
```

### 3. ビルド・実行

```bash
cargo run --features mujoco
```

---

## 🪟 Windows

### 1. MuJoCo をインストール

以下のいずれかの方法で MuJoCo をインストール：

**方法 A: Scoop（推奨）**
```powershell
scoop install mujoco
```

**方法 B: 手動ダウンロード**
- [MuJoCo GitHub Releases](https://github.com/google-deepmind/mujoco/releases) から Windows 版をダウンロード
- `C:\mujoco` または `C:\Program Files\mujoco` に展開

**方法 C: ホームディレクトリに配置**
```powershell
mkdir $env:USERPROFILE\.mujoco
# ダウンロード・展開
```

### 2. PowerShell で環境をセットアップ

```powershell
cd \path\to\articara
. .\setup-mujoco.ps1
```

**重要**: Windows では `.ps1` ファイルの実行ポリシー制限があります。  
以下のコマンドで許可を与えてください（初回のみ）：

```powershell
Set-ExecutionPolicy -ExecutionPolicy RemoteSigned -Scope CurrentUser
```

### 3. ビルド・実行

```powershell
cargo run --features mujoco
```

### 4. 永続的に環境変数を設定（オプション）

```powershell
[Environment]::SetEnvironmentVariable('MUJOCO_DYNAMIC_LINK_DIR', 'C:\path\to\mujoco\lib', 'User')
[Environment]::SetEnvironmentVariable('MUJOCO_HOME', 'C:\path\to\mujoco', 'User')
```

その後、PowerShell を再起動。

---

## 🔧 トラブルシューティング

### `MUJOCO_DYNAMIC_LINK_DIR must be path to the 'lib/' subdirectory`

- **macOS/Linux**: スクリプト内で自動検出されます。スクリプトを再実行してください。
- **Windows**: `lib` フォルダが正しい場所にあるか確認してください。

### `libmujoco.dylib/dll not found`

- MuJoCo がインストールされているか確認
- インストール位置を確認して、スクリプトを再実行

### シェル環境変数が古いままの場合

新しいターミナルウィンドウを開くか、以下を実行：

```bash
# macOS/Linux
unset MUJOCO_DYNAMIC_LINK_DIR DYLD_LIBRARY_PATH DYLD_FRAMEWORK_PATH
source ./setup-mujoco.sh
```

```powershell
# Windows: 新しい PowerShell ウィンドウを開く
```

---

## 📝 スクリプト動作確認

各プラットフォームで正常に動作するか確認：

```bash
# macOS/Linux で動作確認
bash -x ./setup-mujoco.sh

# Windows で動作確認
. .\setup-mujoco.ps1 -Verbose
```

---

## ✅ 確認事項

- ✓ スクリプト実行後、`MUJOCO_DYNAMIC_LINK_DIR` が設定されていることを確認
- ✓ `cargo run --features mujoco` が成功すること
- ✓ UI が起動すること（GL_INVALID_VALUE エラーは UI 関連で MuJoCo とは無関係）
