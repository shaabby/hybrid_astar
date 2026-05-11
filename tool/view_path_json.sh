#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
BUILD_DIR="${ROOT_DIR}/build"
BUILD_TYPE="${BUILD_TYPE:-Release}"

cd "${ROOT_DIR}"

if [[ ! -f "${BUILD_DIR}/CMakeCache.txt" ]]; then
    echo "[view-path] Configuring CMake (${BUILD_TYPE})..."
    cmake -S . -B "${BUILD_DIR}" -DCMAKE_BUILD_TYPE="${BUILD_TYPE}"
fi

echo "[view-path] Building path_json_viewer..."
cmake --build "${BUILD_DIR}" --target path_json_viewer --config "${BUILD_TYPE}"

EXECUTABLE="${BUILD_DIR}/path_json_viewer"
if [[ ! -x "${EXECUTABLE}" && -x "${BUILD_DIR}/${BUILD_TYPE}/path_json_viewer.exe" ]]; then
    EXECUTABLE="${BUILD_DIR}/${BUILD_TYPE}/path_json_viewer.exe"
fi

if [[ ! -x "${EXECUTABLE}" ]]; then
    echo "[view-path] Could not find path_json_viewer executable." >&2
    exit 1
fi

if [[ "$#" -eq 0 ]]; then
    echo "[view-path] Opening output/result.json..."
    "${EXECUTABLE}" output/result.json
else
    echo "[view-path] Opening $*..."
    "${EXECUTABLE}" "$@"
fi
