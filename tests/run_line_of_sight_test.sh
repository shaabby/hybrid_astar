#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
BUILD_DIR="${ROOT_DIR}/build"
BUILD_TYPE="${BUILD_TYPE:-Release}"

cd "${ROOT_DIR}"

if [[ ! -f "${BUILD_DIR}/CMakeCache.txt" ]]; then
    echo "[line-of-sight-test] Configuring CMake (${BUILD_TYPE})..."
    cmake -S . -B "${BUILD_DIR}" -DCMAKE_BUILD_TYPE="${BUILD_TYPE}"
fi

echo "[line-of-sight-test] Building line_of_sight_test..."
cmake --build "${BUILD_DIR}" --target line_of_sight_test --config "${BUILD_TYPE}"

echo "[line-of-sight-test] Running line_of_sight_test..."
ctest --test-dir "${BUILD_DIR}" --build-config "${BUILD_TYPE}" \
    --output-on-failure --tests-regex '^line_of_sight_test$'
