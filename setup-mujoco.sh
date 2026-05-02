#!/bin/bash
# MuJoCo Environment Setup Script
# This script detects MuJoCo installation and sets up the environment

set -e

echo "🔍 Detecting MuJoCo installation..."

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
    echo "❌ MuJoCo not found. Please install via:"
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

# Set environment variables
export MUJOCO_DYNAMIC_LINK_DIR="$LIBMUJOCO_PATH"
export DYLD_FRAMEWORK_PATH="$MUJOCO_DIR"
export DYLD_LIBRARY_PATH="$LIBMUJOCO_PATH"

echo ""
echo "✓ Environment variables set:"
echo "  MUJOCO_DYNAMIC_LINK_DIR=$MUJOCO_DYNAMIC_LINK_DIR"
echo "  DYLD_FRAMEWORK_PATH=$DYLD_FRAMEWORK_PATH"
echo "  DYLD_LIBRARY_PATH=$DYLD_LIBRARY_PATH"
echo ""
echo "✓ Ready to build/run. Now execute:"
echo "  cargo run --features mujoco"
