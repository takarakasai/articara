# MuJoCo Environment Setup Script for Windows
# This script detects MuJoCo installation and sets up the environment
# Usage: . .\setup-mujoco.ps1

$ErrorActionPreference = "Stop"

Write-Host "🔍 Detecting Windows MuJoCo installation..." -ForegroundColor Cyan

# Common Windows MuJoCo installation paths
$PossiblePaths = @(
    "C:\mujoco",
    "C:\Program Files\mujoco",
    "C:\Program Files (x86)\mujoco",
    "$env:USERPROFILE\mujoco",
    "$env:USERPROFILE\.mujoco\mujoco-3.8.0",
    "$env:USERPROFILE\scoop\apps\mujoco\current"
)

$MuJocoDir = $null

# Search for MuJoCo installation
foreach ($path in $PossiblePaths) {
    if (Test-Path $path) {
        Write-Host "✓ Found MuJoCo: $path" -ForegroundColor Green
        $MuJocoDir = $path
        break
    }
}

# Alternative: check if mujoco is in PATH
if ($null -eq $MuJocoDir) {
    $MuJocoExe = Get-Command mjctrl -ErrorAction SilentlyContinue
    if ($null -ne $MuJocoExe) {
        $MuJocoDir = Split-Path $MuJocoExe.Source | Split-Path
        Write-Host "✓ Found MuJoCo in PATH: $MuJocoDir" -ForegroundColor Green
    }
}

if ($null -eq $MuJocoDir) {
    Write-Host "❌ MuJoCo not found on Windows." -ForegroundColor Red
    Write-Host ""
    Write-Host "Installation options:" -ForegroundColor Yellow
    Write-Host "  1. Scoop: scoop install mujoco"
    Write-Host "  2. Manual: Download from https://github.com/google-deepmind/mujoco/releases"
    Write-Host "  3. Extract to: C:\mujoco or $env:USERPROFILE\mujoco"
    exit 1
}

# Detect library path
$LibPath = $null

if (Test-Path "$MuJocoDir\lib\mujoco.dll") {
    $LibPath = "$MuJocoDir\lib"
    Write-Host "✓ Found Windows DLL: $LibPath" -ForegroundColor Green
} elseif (Test-Path "$MuJocoDir\bin\mujoco.dll") {
    $LibPath = "$MuJocoDir\bin"
    Write-Host "✓ Found Windows DLL in bin: $LibPath" -ForegroundColor Green
} else {
    Write-Host "❌ Could not find mujoco.dll" -ForegroundColor Red
    Write-Host "Expected location: $MuJocoDir\lib\mujoco.dll or $MuJocoDir\bin\mujoco.dll"
    exit 1
}

# Set environment variables
$env:MUJOCO_DYNAMIC_LINK_DIR = $LibPath
$env:MUJOCO_HOME = $MuJocoDir

# Add to PATH if not already present
if ($env:Path -notlike "*$LibPath*") {
    $env:Path = "$LibPath;$env:Path"
}

Write-Host ""
Write-Host "✓ Windows environment variables set:" -ForegroundColor Green
Write-Host "  MUJOCO_DYNAMIC_LINK_DIR=$env:MUJOCO_DYNAMIC_LINK_DIR"
Write-Host "  MUJOCO_HOME=$env:MUJOCO_HOME"
Write-Host "  PATH updated with: $LibPath"

Write-Host ""
Write-Host "✓ Ready to build/run. Now execute:" -ForegroundColor Green
Write-Host "  cargo run --features mujoco"
Write-Host ""
Write-Host "📌 Note: These environment variables are set for this PowerShell session only." -ForegroundColor Yellow
Write-Host "   To make them permanent, use:" -ForegroundColor Yellow
Write-Host "   [Environment]::SetEnvironmentVariable('MUJOCO_DYNAMIC_LINK_DIR', '$LibPath', 'User')" -ForegroundColor Yellow
