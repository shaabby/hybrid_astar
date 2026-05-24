#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="${ROOT_DIR}/build"
BUILD_TYPE="${BUILD_TYPE:-Release}"
TMPDIR="${TMPDIR:-${BUILD_DIR}/tmp}"
WORK_DIR="${TMPDIR}/obs_heuristic_compare"
MAPS_DIR="${1:-map}"
OUTPUT_CSV="${2:-output/obs_heuristic_compare.csv}"
OUTPUT_MAP_DIR="${3:-output/obs_heuristic_compare_maps}"

cd "${ROOT_DIR}"
mkdir -p "${TMPDIR}" "${WORK_DIR}"
export TMPDIR

if [[ ! -f "${BUILD_DIR}/CMakeCache.txt" ]]; then
    echo "[compare-obs] Configuring CMake (${BUILD_TYPE})..."
    cmake -S . -B "${BUILD_DIR}" -DCMAKE_BUILD_TYPE="${BUILD_TYPE}"
fi

echo "[compare-obs] Building hybrid_astar_testbench..."
cmake --build "${BUILD_DIR}" --target hybrid_astar_testbench --config "${BUILD_TYPE}"

python3 - <<'PY'
from pathlib import Path

base = Path("config/default.yaml").read_text()
work = Path("build/tmp/obs_heuristic_compare")
work.mkdir(parents=True, exist_ok=True)

configs = {
    "visibility_graph": "visibility_graph",
    "reverse_dijkstra": "reverse_dijkstra",
}

for name, heuristic_type in configs.items():
    text = base
    if "obstacle_heuristic_type:" in text:
        lines = []
        for line in text.splitlines():
            stripped = line.lstrip()
            indent = line[:len(line) - len(stripped)]
            if stripped.startswith("obstacle_heuristic_type:"):
                line = f"{indent}obstacle_heuristic_type: {heuristic_type}"
            lines.append(line)
        text = "\n".join(lines) + "\n"
    else:
        text = text.replace(
            "  obstacle_lookup_resolution:",
            f"  obstacle_heuristic_type: {heuristic_type}\n  obstacle_lookup_resolution:",
            1,
        )
    (work / f"{name}.yaml").write_text(text)

(work / "groups.txt").write_text(
    "visibility_graph build/tmp/obs_heuristic_compare/visibility_graph.yaml\n"
    "reverse_dijkstra build/tmp/obs_heuristic_compare/reverse_dijkstra.yaml\n"
)
PY

echo "[compare-obs] Running testbench..."
"${BUILD_DIR}/hybrid_astar_testbench" \
    --groups "${WORK_DIR}/groups.txt" \
    --maps "${MAPS_DIR}" \
    --output "${OUTPUT_CSV}" \
    --output-map-dir "${OUTPUT_MAP_DIR}"

echo "[compare-obs] Done."
echo "  output: ${OUTPUT_CSV}"
echo "  output_map_dir: ${OUTPUT_MAP_DIR}"
