#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
BUILD_DIR="${BUILD_DIR:-${ROOT_DIR}/build}"
BUILD_TYPE="${BUILD_TYPE:-Release}"

usage() {
    cat <<EOF
Usage: $0 [result.json|json_dir] [--path name|index]
       $0 [result.json|json_dir] --list
       $0 --list
       $0 --help

If no JSON file or directory is provided, defaults to output/result.json.
If a directory is provided, opens all *.json files in that directory in sorted order.
EOF
}

if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
    usage
    exit 0
fi

TARGET="output/result.json"
VIEWER_ARGS=()

if [[ "$#" -gt 0 ]]; then
    if [[ "${1}" == --* ]]; then
        VIEWER_ARGS=("$@")
    else
        TARGET="${1}"
        shift
        VIEWER_ARGS=("$@")
    fi
fi

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

if [[ -d "${TARGET}" ]]; then
    mapfile -t JSON_FILES < <(find "${TARGET}" -maxdepth 1 -type f -name '*.json' | sort)
    if [[ "${#JSON_FILES[@]}" -eq 0 ]]; then
        echo "[view-path] No *.json files found in directory: ${TARGET}" >&2
        exit 1
    fi

    echo "[view-path] Opening ${#JSON_FILES[@]} JSON file(s) from ${TARGET}..."
    for index in "${!JSON_FILES[@]}"; do
        json_file="${JSON_FILES[${index}]}"
        echo "[view-path] [$((index + 1))/${#JSON_FILES[@]}] ${json_file} ${VIEWER_ARGS[*]}"
        "${EXECUTABLE}" "${json_file}" "${VIEWER_ARGS[@]}"
    done
    exit 0
fi

if [[ ! -f "${TARGET}" ]]; then
    echo "[view-path] JSON file or directory not found: ${TARGET}" >&2
    exit 1
fi

echo "[view-path] Opening ${TARGET} ${VIEWER_ARGS[*]}..."
exec "${EXECUTABLE}" "${TARGET}" "${VIEWER_ARGS[@]}"
