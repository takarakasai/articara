#!/usr/bin/env bash
# Release helpers for .github/workflows/release.yml. Each step is a
# subcommand so it can be run (and debugged) locally on Linux too.
#
#   release.sh fetch-mujoco <dir>
#       Download and unpack the MuJoCo runtime for this OS into <dir>;
#       prints MUJOCO_ROOT=… and MUJOCO_DYNAMIC_LINK_DIR=… (for $GITHUB_ENV).
#   release.sh build <lite|mujoco> <target>
#       cargo build --release of the `articara` binary. The mujoco variant
#       links against $MUJOCO_DYNAMIC_LINK_DIR with an rpath pointing next
#       to the executable, where `package` puts the runtime.
#   release.sh package <lite|mujoco> <target> <name> <out-dir>
#       Lay out <out-dir>/<name>/ (binary, docs, MuJoCo runtime) and
#       archive it (.tar.gz, or .zip on Windows); prints the archive path.
#   release.sh smoke <lite|mujoco> <target> <package-dir>
#       Run the packaged binary headless — for mujoco, start and step a
#       simulation — with no MuJoCo on any search path but the bundled one.
set -euo pipefail

MUJOCO_VERSION=3.8.0
MUJOCO_URL=https://github.com/google-deepmind/mujoco/releases/download/$MUJOCO_VERSION
REPO=$(cd "$(dirname "$0")/.." && pwd)

host_os() {
    case "$(uname -s)" in
        Linux*) echo linux ;;
        Darwin*) echo macos ;;
        MINGW* | MSYS* | CYGWIN*) echo windows ;;
        *) echo "unsupported host $(uname -s)" >&2; exit 1 ;;
    esac
}

# Absolute native path (Windows tools want C:\…, not /c/…).
native_path() {
    if [ "$(host_os)" = windows ]; then cygpath -w "$1"; else echo "$1"; fi
}

exe() {
    case "$1" in *windows*) echo articara.exe ;; *) echo articara ;; esac
}

fetch_mujoco() {
    local dir=$1
    mkdir -p "$dir"
    dir=$(cd "$dir" && pwd)
    case "$(host_os)" in
        linux)
            curl -fsSL "$MUJOCO_URL/mujoco-$MUJOCO_VERSION-linux-x86_64.tar.gz" | tar -xz -C "$dir"
            local root=$dir/mujoco-$MUJOCO_VERSION
            echo "MUJOCO_ROOT=$root"
            echo "MUJOCO_DYNAMIC_LINK_DIR=$root/lib"
            ;;
        windows)
            curl -fsSL -o "$dir/mujoco.zip" "$MUJOCO_URL/mujoco-$MUJOCO_VERSION-windows-x86_64.zip"
            local root=$dir/mujoco-$MUJOCO_VERSION
            mkdir -p "$root"
            # The archive uses `\` separators, which unzip only warns
            # about (exit 1); 7-Zip reads it cleanly.
            7z x -y -bso0 -o"$(native_path "$root")" "$(native_path "$dir/mujoco.zip")" >&2
            # Some archives wrap everything in a top-level folder.
            if [ ! -f "$root/lib/mujoco.lib" ]; then
                root=$(dirname "$(dirname "$(find "$root" -name mujoco.lib | head -1)")")
            fi
            echo "MUJOCO_ROOT=$(native_path "$root")"
            echo "MUJOCO_DYNAMIC_LINK_DIR=$(native_path "$root/lib")"
            ;;
        macos)
            curl -fsSL -o "$dir/mujoco.dmg" "$MUJOCO_URL/mujoco-$MUJOCO_VERSION-macos-universal2.dmg"
            hdiutil attach -quiet -nobrowse -readonly -mountpoint "$dir/mnt" "$dir/mujoco.dmg"
            local fw
            fw=$(find "$dir/mnt" -maxdepth 4 -name mujoco.framework -type d | head -1)
            [ -n "$fw" ] || { echo "mujoco.framework not found in the dmg" >&2; ls -R "$dir/mnt" >&2; exit 1; }
            # ditto keeps the framework's symlinks intact.
            ditto "$fw" "$dir/mujoco.framework"
            hdiutil detach -quiet "$dir/mnt"
            # mujoco-rs links against <dir>/libmujoco.dylib.
            local dylib
            dylib=$(find "$dir/mujoco.framework" -name 'libmujoco*.dylib' -type f | head -1)
            mkdir -p "$dir/lib"
            ln -sf "$dylib" "$dir/lib/libmujoco.dylib"
            echo "MUJOCO_ROOT=$dir"
            echo "MUJOCO_DYNAMIC_LINK_DIR=$dir/lib"
            ;;
    esac
}

build() {
    local variant=$1 target=$2
    local args=(build --release --locked --target "$target" --bin articara)
    if [ "$variant" = mujoco ]; then
        : "${MUJOCO_DYNAMIC_LINK_DIR:?run fetch-mujoco first}"
        args+=(--features mujoco)
        # Find the bundled runtime next to the executable.
        case "$target" in
            *linux*) export RUSTFLAGS="${RUSTFLAGS:-} -C link-arg=-Wl,-rpath,\$ORIGIN/lib" ;;
            *apple*) export RUSTFLAGS="${RUSTFLAGS:-} -C link-arg=-Wl,-rpath,@executable_path" ;;
            # Windows loads DLLs from the executable's directory already.
        esac
    fi
    (cd "$REPO" && cargo "${args[@]}")
}

package() {
    local variant=$1 target=$2 name=$3 out=$4
    local bin=$REPO/target/$target/release/$(exe "$target")
    local dir=$out/$name
    rm -rf "$dir"
    mkdir -p "$dir"
    cp "$bin" "$dir/"
    cp "$REPO/README.md" "$REPO/LICENSE" "$dir/"
    if [ "$variant" = mujoco ]; then
        : "${MUJOCO_ROOT:?run fetch-mujoco first}"
        local root
        root=$(cd "$MUJOCO_ROOT" 2>/dev/null && pwd || cygpath -u "$MUJOCO_ROOT")
        case "$target" in
            *linux*)
                mkdir -p "$dir/lib"
                # The binary needs the soname (libmujoco.so.X.Y.Z).
                cp -L "$root/lib/libmujoco.so.$MUJOCO_VERSION" "$dir/lib/"
                ;;
            *windows*) cp "$root/bin/mujoco.dll" "$dir/" ;;
            *apple*) ditto "$root/mujoco.framework" "$dir/mujoco.framework" ;;
        esac
        local lic
        lic=$(find "$root" -maxdepth 3 -iname 'LICENSE*' -type f 2>/dev/null | head -1 || true)
        {
            echo "This build bundles the MuJoCo $MUJOCO_VERSION runtime"
            echo "(https://github.com/google-deepmind/mujoco), licensed under Apache-2.0."
            if [ -n "$lic" ]; then
                echo
                cat "$lic"
            fi
        } > "$dir/MUJOCO_LICENSE.txt"
    fi
    (
        cd "$out"
        case "$target" in
            *windows*)
                rm -f "$name.zip"
                7z a -tzip -bso0 "$name.zip" "$name" >&2
                echo "$out/$name.zip"
                ;;
            *)
                tar -czf "$name.tar.gz" "$name"
                echo "$out/$name.tar.gz"
                ;;
        esac
    )
}

smoke() {
    local variant=$1 target=$2 dir=$3
    local script
    script=$(mktemp)
    if [ "$variant" = mujoco ]; then
        cat > "$script" <<'EOF'
if !mj_start() { throw "mj_start failed"; }
let n = mj_step(200);
print(`mujoco ok: ${n} steps of ${mj_timestep()} s`);
EOF
    else
        echo 'print("articara ok");' > "$script"
    fi
    local run=("$dir/$(exe "$target")")
    # An Intel build on an Apple Silicon runner goes through Rosetta.
    if [[ "$target" == x86_64-apple-darwin && "$(uname -m)" == arm64 ]]; then
        run=(arch -x86_64 "${run[@]}")
    fi
    # Nothing but the bundled runtime may satisfy the MuJoCo dependency.
    env -u LD_LIBRARY_PATH -u DYLD_LIBRARY_PATH -u DYLD_FRAMEWORK_PATH -u MUJOCO_DYNAMIC_LINK_DIR \
        "${run[@]}" --script-headless "$(native_path "$script")" "$(native_path "$REPO/tests/fixtures/urdf/test_robot.urdf")"
}

cmd=${1:-}
shift || true
case "$cmd" in
    fetch-mujoco) fetch_mujoco "$@" ;;
    build) build "$@" ;;
    package) package "$@" ;;
    smoke) smoke "$@" ;;
    *) sed -n '2,20p' "$0" >&2; exit 2 ;;
esac
