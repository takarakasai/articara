#!/usr/bin/env bash
# Run the full MuJoCo gait-control regression suite.
#
# Each entry below covers one slice of the pipeline:
#
#   gait_walk_stability   - CHAMP + MPC walks at the gait-controller layer
#                           (Position-PD only; the simplest "does it walk
#                           forward?" gate)
#   wbc_walk              - Hybrid joint command (Position-PD + WBC τ_ff);
#                           static stand passes, forward-walk is #[ignore]d
#                           pending joint-space swing reference (see
#                           tests/wbc_walk.rs docs)
#   lkf_pipeline          - LinearKalmanFilter e2e against MuJoCo ground
#                           truth (body-z tracking within ±5 cm)
#   integration_walk      - Cross-layer regression: PD-only / +WBC / +LKF
#                           in a single 1-second sim each, asserting
#                           min_z and forward-displacement envelopes
#
# This script is also the recommended pre-commit check for any change
# touching `quadruped-gait`, `src/wbc_pipeline.rs`, `src/estimator/`,
# `src/mujoco_sim.rs`, or `misarta::{qp, jacobian, fk}`.
#
# Usage:
#     ./scripts/test_regression.sh                # release-mode, all suites
#     ./scripts/test_regression.sh --debug        # debug-mode (faster build,
#                                                   slower run)
#     ./scripts/test_regression.sh --quick        # just lib + walk_stability
#                                                   (skips wbc + lkf + integration)
#
# MuJoCo paths are sourced from the standard env vars set by the
# repo's CLAUDE.md instructions:
#     MUJOCO_DOWNLOAD_DIR
#     MUJOCO_DYNAMIC_LINK_DIR
#     LD_LIBRARY_PATH

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

PROFILE_FLAG="--release"
QUICK_MODE=0

for arg in "$@"; do
    case "$arg" in
        --debug)   PROFILE_FLAG="" ;;
        --quick)   QUICK_MODE=1 ;;
        --help|-h)
            sed -n '2,30p' "$0"
            exit 0
            ;;
        *)
            echo "unknown arg: $arg" >&2
            exit 2
            ;;
    esac
done

# Default MuJoCo env if the user hasn't set them. Match the canonical
# install layout from the repo CLAUDE.md.
: "${MUJOCO_DOWNLOAD_DIR:=$HOME/.mujoco}"
: "${MUJOCO_DYNAMIC_LINK_DIR:=$HOME/.mujoco/mujoco-3.8.0/lib}"
: "${LD_LIBRARY_PATH:=$MUJOCO_DYNAMIC_LINK_DIR}"
export MUJOCO_DOWNLOAD_DIR MUJOCO_DYNAMIC_LINK_DIR
export LD_LIBRARY_PATH="$MUJOCO_DYNAMIC_LINK_DIR:$LD_LIBRARY_PATH"

run_step() {
    local label="$1"; shift
    echo
    echo "═══════════════════════════════════════════════════════════════"
    echo "  $label"
    echo "═══════════════════════════════════════════════════════════════"
    "$@"
}

# 1. Lib tests (fast — no MuJoCo). Catches QP / WBC / SRBD / estimator
#    math regressions before the slower MuJoCo suites run.
run_step "quadruped-gait lib tests" \
    cargo test $PROFILE_FLAG -p quadruped-gait --lib

run_step "articara lib tests (without mujoco)" \
    cargo test $PROFILE_FLAG --lib --package articara

run_step "misarta lib tests (with clarabel)" \
    cargo test $PROFILE_FLAG -p misarta --features clarabel --lib

if [ "$QUICK_MODE" -eq 1 ]; then
    run_step "gait_walk_stability (CHAMP + MPC)" \
        cargo test $PROFILE_FLAG --features mujoco --test gait_walk_stability
    echo
    echo "═══════════════════════════════════════════════════════════════"
    echo "  Quick regression PASSED"
    echo "═══════════════════════════════════════════════════════════════"
    exit 0
fi

# 2. Full MuJoCo regression. Each --test invocation is its own
#    integration-test crate so a build failure in one doesn't block
#    the others.
run_step "gait_walk_stability (CHAMP + MPC)" \
    cargo test $PROFILE_FLAG --features mujoco --test gait_walk_stability

run_step "wbc_walk (Hybrid joint static stand)" \
    cargo test $PROFILE_FLAG --features mujoco --test wbc_walk

run_step "lkf_pipeline (Linear KF e2e)" \
    cargo test $PROFILE_FLAG --features mujoco --test lkf_pipeline

run_step "integration_walk (cross-layer 1 s walks)" \
    cargo test $PROFILE_FLAG --features mujoco --test integration_walk

echo
echo "═══════════════════════════════════════════════════════════════"
echo "  Full regression PASSED"
echo "═══════════════════════════════════════════════════════════════"
