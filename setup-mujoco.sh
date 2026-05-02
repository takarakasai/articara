#!/bin/bash
# MuJoCo Environment Setup Script for macOS and Linux
# This script detects MuJoCo installation and sets up the environment
# Supports: macOS (Homebrew), Linux
#
# Windows users: Use setup-mujoco.ps1 instead
#   . .\setup-mujoco.ps1

set -e

setup_macos() {
    echo "🔍 Searching for MuJoCo on macOS..."

    # Check Homebrew Cask installation
    if [ -d "/opt/homebrew/Caskroom/mujoco" ]; then
        MUJOCO_DIR=$(ls -d /opt/homebrew/Caskroom/mujoco/* | sort -V | tail -1)
        echo "✓ Found Homebrew Cask MuJoCo: $MUJOCO_DIR"
    elif [ -d "$HOME/mujoco" ]; then
        MUJOCO_DIR="$HOME/mujoco"
        echo "✓ Found MuJoCo in home directory: $MUJOCO_DIR"
    elif [ -d "/Applications/MuJoCo.app/Contents/Frameworks" ]; then
        MUJOCO_DIR="/Applications/MuJoCo.app/Contents/Frameworks"
        echo "✓ Found MuJoCo.app: $MUJOCO_DIR"
    else
        echo "❌ MuJoCo not found on macOS. Please install via:"
        echo "   brew install --cask mujoco"
        exit 1
    fi

    # Detect library path
    if [ -f "$MUJOCO_DIR/mujoco.framework/Versions/A/libmujoco.dylib" ]; then
        LIBMUJOCO_PATH="$MUJOCO_DIR/mujoco.framework/Versions/A"
        echo "✓ Found macOS framework: $LIBMUJOCO_PATH"
    elif [ -f "$MUJOCO_DIR/lib/libmujoco.dylib" ]; then
        LIBMUJOCO_PATH="$MUJOCO_DIR/lib"
        echo "✓ Found library: $LIBMUJOCO_PATH"
    else
        echo "❌ Could not find libmujoco.dylib"
        exit 1
    fi

    # Set macOS environment variables
    export MUJOCO_DYNAMIC_LINK_DIR="$LIBMUJOCO_PATH"
    export DYLD_FRAMEWORK_PATH="$MUJOCO_DIR"
    export DYLD_LIBRARY_PATH="$LIBMUJOCO_PATH"

    echo ""
    echo "✓ macOS environment variables set:"
    echo "  MUJOCO_DYNAMIC_LINK_DIR=$MUJOCO_DYNAMIC_LINK_DIR"
    echo "  DYLD_FRAMEWORK_PATH=$DYLD_FRAMEWORK_PATH"
    echo "  DYLD_LIBRARY_PATH=$DYLD_LIBRARY_PATH"
}

setup_linux() {
    echo "🔍 Searching for MuJoCo on Linux..."

    # Common Linux installation paths
    if [ -d "$HOME/.mujoco/mujoco-3.8.0" ]; then
        MUJOCO_DIR="$HOME/.mujoco/mujoco-3.8.0"
        echo "✓ Found MuJoCo: $MUJOCO_DIR"
    elif [ -d "/opt/mujoco" ]; then
        MUJOCO_DIR="/opt/mujoco"
        echo "✓ Found MuJoCo: $MUJOCO_DIR"
    else
        echo "❌ MuJoCo not found on Linux. Please install from:"
        echo "   https://github.com/google-deepmind/mujoco/releases"
        exit 1
    fi

    # Detect library path
    if [ -f "$MUJOCO_DIR/lib/libmujoco.so" ] || [ -f "$MUJOCO_DIR/lib/libmujoco.so.3" ]; then
        LIBMUJOCO_PATH="$MUJOCO_DIR/lib"
        echo "✓ Found Linux library: $LIBMUJOCO_PATH"
    else
        echo "❌ Could not find libmujoco.so"
        exit 1
    fi

    # Set Linux environment variables
    export MUJOCO_DYNAMIC_LINK_DIR="$LIBMUJOCO_PATH"
    export LD_LIBRARY_PATH="$LIBMUJOCO_PATH:$LD_LIBRARY_PATH"

    echo ""
    echo "✓ Linux environment variables set:"
    echo "  MUJOCO_DYNAMIC_LINK_DIR=$MUJOCO_DYNAMIC_LINK_DIR"
    echo "  LD_LIBRARY_PATH=$LD_LIBRARY_PATH"
}

# Main execution
echo "🔍 Detecting OS and MuJoCo installation..."

OS_TYPE=$(uname -s)

case "$OS_TYPE" in
    Darwin)
        echo "✓ macOS detected"
        setup_macos
        ;;
    Linux)
        echo "✓ Linux detected"
        setup_linux
        ;;
    *)
        echo "❌ Unsupported OS: $OS_TYPE"
        echo "Supported platforms: macOS (Darwin), Linux"
        exit 1
        ;;
esac

echo ""
echo "✓ Ready to build/run. Now execute:"
echo "  cargo run --features mujoco"
