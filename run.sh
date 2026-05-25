#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${ROOT_DIR}/build"
BUILD_TYPE="${BUILD_TYPE:-Release}"
TMPDIR="${TMPDIR:-${BUILD_DIR}/tmp}"

cd "${ROOT_DIR}"

mkdir -p "${TMPDIR}"
export TMPDIR

if [[ ! -f "${BUILD_DIR}/CMakeCache.txt" ]]; then
    echo "[run] Configuring CMake (${BUILD_TYPE})..."
    cmake -S . -B "${BUILD_DIR}" -DCMAKE_BUILD_TYPE="${BUILD_TYPE}"
fi

echo "[run] Building hybrid_astar..."
cmake --build "${BUILD_DIR}" --config "${BUILD_TYPE}"

EXECUTABLE="${BUILD_DIR}/hybrid_astar"
if [[ ! -x "${EXECUTABLE}" && -x "${BUILD_DIR}/${BUILD_TYPE}/hybrid_astar.exe" ]]; then
    EXECUTABLE="${BUILD_DIR}/${BUILD_TYPE}/hybrid_astar.exe"
fi

if [[ "$#" -eq 0 ]]; then
    echo "[run] Running config/default.yaml..."
    "${EXECUTABLE}" config/default.yaml
else
    echo "[run] Running $*..."
    "${EXECUTABLE}" "$@"
fi

echo
echo "[run] Opening output/result.json in path_json_viewer..."
"${ROOT_DIR}/tool/view_path_json.sh" output/result.json
