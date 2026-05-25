# Hybrid A* Demo

一个基于 `C++23 + FLTK` 的 Hybrid A* 路径规划课程项目，包含：

- JSON 栅格地图读取
- 简化车辆 bicycle model
- Hybrid A* 搜索
- Reeds-Shepp 解析扩展
- 路径 JSON 导出
- FLTK 本地查看器
- 批量实验与 CSV 日志

典型流程：

```text
地图 JSON -> C++ 规划 -> result.json / CSV -> FLTK 查看
```

## 目录

- `config/default.yaml`：默认运行配置
- `map/`：地图与地图编辑器
- `src/`、`include/`：核心实现
- `tool/`：查看器与 testbench
- `scripts/`：地图生成与实验脚本
- `output/`：运行输出

## 快速开始

推荐直接运行：

```bash
./run.sh
```

Windows 下可用：

```bat
run.bat
```

如果要用默认配置批量运行 `map\` 下已有地图，并依次打开生成结果，可直接运行：

```bat
demo.bat
```

如果要在 `empty01` 地图上对比三组搜索离散参数，并依次打开结果，可运行：

```bat
demo_para.bat
```

首次运行会自动配置并构建，默认执行：

```text
config/default.yaml
```

输出文件：

```text
output/result.json
output/single_run_timing.csv
```

手动构建：

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release
```

手动运行：

```bash
./build/hybrid_astar config/default.yaml
```

无图形环境下运行：

```bash
./build/hybrid_astar --no-view config/default.yaml
```

`--html-only` 是 `--no-view` 的兼容别名。

## 查看结果

打开单个路径 JSON：

```bash
./tool/view_path_json.sh output/result.json
```

Windows 下可用：

```bat
tool\view_path_json.bat output\result.json
```

只列出路径，不打开窗口：

```bash
./tool/view_path_json.sh output/result.json --list
```

也可以直接运行查看器：

```bash
./build/path_json_viewer output/result.json
```

## 测试

运行单元测试：

```bash
ctest --test-dir build --output-on-failure
```

当前主要测试包括：

- `reeds_shepp_empty_map_test`
- `line_of_sight_test`
- `app_config_test`

## 批量实验

运行默认参数组的 testbench：

```bash
./build/hybrid_astar_testbench \
  --groups config/testbench/default_groups.txt \
  --maps map \
  --output output/default_timing.csv \
  --output-map-dir output/default_timing_maps
```

常用参数：

- `--groups`：参数组列表
- `--maps`：地图目录
- `--output`：CSV 输出
- `--output-map-dir`：每次规划的 JSON 输出目录

Windows 下也提供了现成脚本：

```bat
demo.bat
```

它会使用 `config/testbench/default_groups.txt` 批量运行 `map\` 目录下已有地图，并打开 `output\demo\default\` 中的结果。

另一个参数对比脚本：

```bat
demo_para.bat
```

它会只运行 `map\empty01.json`，并比较以下三组参数：

- `xy_resolution=0.1`，`step_size=0.1`，`primitive_length=0.2`
- `xy_resolution=0.5`，`step_size=0.5`，`primitive_length=0.5`
- `xy_resolution=1`，`step_size=1`，`primitive_length=1`

对应输出：

```text
output/demo_para.csv
output/demo_para/
```

## 生成测试地图

生成多尺寸固定模板地图：

```bash
python3 scripts/generate_testbench_maps.py \
  --output-dir map/generated \
  --sizes 40x25 60x36 80x50
```

脚本会生成 `empty`、`simple`、`narrow`、`reverse`、`u`、`unreach` 等地图。

## 障碍物启发式对比

对比 `visibility_graph` 和 `reverse_dijkstra`：

```bash
./scripts/compare_obs_heuristics.sh \
  map/generated \
  output/generated_obs_heuristic_compare.csv \
  output/generated_obs_heuristic_compare_maps
```

输出：

```text
output/generated_obs_heuristic_compare.csv
output/generated_obs_heuristic_compare_clean.csv
output/generated_obs_heuristic_compare_report.md
```

## 配置说明

主配置见 `config/default.yaml`，核心项包括：

- `map_path`：地图文件
- `vehicle.*`：车辆尺寸和最大转向角
- `xy_resolution`、`theta_bins`：状态离散精度
- `step_size`、`primitive_length`：运动基元长度
- `max_iterations`：最大搜索迭代数
- `enable_analytic_expansion`：是否启用 Reeds-Shepp 直连
- `enable_obstacle_heuristic`：是否启用障碍物启发式
- `obstacle_heuristic_type`：`visibility_graph` 或 `reverse_dijkstra`
- `enable_timing`：是否写细分计时
- `debug`：是否输出调试日志

说明：

- `visibility_graph` 是当前默认障碍物启发式。
- `obstacle_heuristic_inflation_alpha` 只影响 `reverse_dijkstra`。

## 地图编辑

浏览器打开：

```text
map/grid_demo.html
```

可用于编辑障碍物、起点、终点和朝向，并导出 JSON 地图。

## 常见问题

`/tmp` 空间不足：

```bash
TMPDIR="$PWD/build/tmp" cmake --build build --config Release
```

无图形显示：

```bash
./build/hybrid_astar --no-view config/default.yaml
```

Windows 原生环境下，可执行文件通常在：

```text
build/Release/hybrid_astar.exe
```

如果使用本项目附带的批处理脚本，则入口分别为：

```text
run.bat
tool\view_path_json.bat
```

这些脚本统一通过 `cmake --build` 调用当前生成器，因而比直接写死 `mingw32-make` 更兼容不同 Windows 构建环境。
