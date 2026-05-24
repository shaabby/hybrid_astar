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
CLEAN_CSV="${OUTPUT_CSV%.csv}_clean.csv"
REPORT_MD="${OUTPUT_CSV%.csv}_report.md"

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

echo "[compare-obs] Writing clean CSV and report..."
python3 - "${OUTPUT_CSV}" "${CLEAN_CSV}" "${REPORT_MD}" <<'PY'
import csv
import statistics
import sys
from collections import defaultdict
from pathlib import Path

raw_path = Path(sys.argv[1])
clean_path = Path(sys.argv[2])
report_path = Path(sys.argv[3])

rows = list(csv.DictReader(raw_path.open(newline="", encoding="utf-8")))
if not rows:
    raise SystemExit(f"no rows found in {raw_path}")

clean_fields = [
    "heuristic",
    "map",
    "success",
    "runtime_ms",
    "expanded_nodes",
    "iterations",
    "generated_nodes",
    "path_poses",
    "heuristic_prepare_ms",
    "search_loop_ms",
    "obstacle_heuristic_ms",
    "non_obstacle_heuristic_ms",
]

clean_rows = []
for row in rows:
    clean_rows.append({
        "heuristic": row.get("obstacle_heuristic_type") or row.get("parameter_group", ""),
        "map": Path(row.get("map_path", "")).name,
        "success": row.get("success", "0"),
        "runtime_ms": row.get("runtime_ms", "0"),
        "expanded_nodes": row.get("expanded_nodes", "0"),
        "iterations": row.get("iterations", "0"),
        "generated_nodes": row.get("generated_nodes", "0"),
        "path_poses": row.get("path_poses", "0"),
        "heuristic_prepare_ms": row.get("heuristic_prepare_ms", "0"),
        "search_loop_ms": row.get("search_loop_ms", "0"),
        "obstacle_heuristic_ms": row.get("obstacle_heuristic_ms", "0"),
        "non_obstacle_heuristic_ms": row.get("non_obstacle_heuristic_ms", "0"),
    })

clean_path.parent.mkdir(parents=True, exist_ok=True)
with clean_path.open("w", newline="", encoding="utf-8") as output:
    writer = csv.DictWriter(output, fieldnames=clean_fields)
    writer.writeheader()
    writer.writerows(clean_rows)


def number(row, key):
    try:
        return float(row.get(key, 0) or 0)
    except ValueError:
        return 0.0


def mean(values):
    return statistics.fmean(values) if values else 0.0

by_heuristic = defaultdict(list)
for row in clean_rows:
    by_heuristic[row["heuristic"]].append(row)

report_lines = [
    "# Obstacle Heuristic Compare Report",
    "",
    f"Raw CSV: `{raw_path}`",
    f"Clean CSV: `{clean_path}`",
    "",
    "## Summary by heuristic",
    "",
    "| heuristic | runs | success | success_rate | avg_runtime_ms | avg_expanded | avg_iterations | avg_path_poses |",
    "|---|---:|---:|---:|---:|---:|---:|---:|",
]

for heuristic in sorted(by_heuristic):
    group = by_heuristic[heuristic]
    runs = len(group)
    successes = sum(1 for row in group if row.get("success") == "1")
    success_rate = successes / runs if runs else 0.0
    report_lines.append(
        f"| {heuristic} | {runs} | {successes} | {success_rate:.2%} | "
        f"{mean([number(row, 'runtime_ms') for row in group]):.3f} | "
        f"{mean([number(row, 'expanded_nodes') for row in group]):.1f} | "
        f"{mean([number(row, 'iterations') for row in group]):.1f} | "
        f"{mean([number(row, 'path_poses') for row in group]):.1f} |"
    )

report_lines.extend([
    "",
    "## Failed maps",
    "",
])

failed = [row for row in clean_rows if row.get("success") != "1"]
if failed:
    report_lines.extend([
        "| heuristic | map | runtime_ms | expanded_nodes | iterations |",
        "|---|---|---:|---:|---:|",
    ])
    for row in failed:
        report_lines.append(
            f"| {row['heuristic']} | {row['map']} | "
            f"{number(row, 'runtime_ms'):.3f} | "
            f"{number(row, 'expanded_nodes'):.0f} | "
            f"{number(row, 'iterations'):.0f} |"
        )
else:
    report_lines.append("No failed maps.")

report_path.parent.mkdir(parents=True, exist_ok=True)
report_path.write_text("\n".join(report_lines) + "\n", encoding="utf-8")
PY

echo "[compare-obs] Done."
echo "  raw_csv: ${OUTPUT_CSV}"
echo "  clean_csv: ${CLEAN_CSV}"
echo "  report: ${REPORT_MD}"
echo "  output_map_dir: ${OUTPUT_MAP_DIR}"
