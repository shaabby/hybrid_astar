# Hybrid A* 路径规划最小闭环 Demo

本项目是一个 C++23 + FLTK 的 Hybrid A* 路径规划课堂展示 demo。

当前目标不是完整工业级规划器，而是先打通最小闭环：

```text
Web 地图编辑器
    ↓ 导出 JSON 地图
C++ 读取地图
    ↓ 构造 GridMap / Car
Hybrid A* 生成车辆位姿序列
    ↓ 输出 result.json
FLTK 查看器渲染地图和小车运动
```

## 当前已完成

已经完成最小可运行闭环：

- `map/grid_demo.html`：浏览器中的方格地图编辑器；
- `GridMap`：从 JSON 读取地图、起点、终点和障碍物；
- `Car`：简化 bicycle model，小车以后轴中心为状态参考点；
- `HybridAstar`：最小 Hybrid A* 核心搜索；
- `JsonExporter`：输出 `map / vehicle / path / expanded`；
- `FltkViewer`：显示规划结果、路径动画和车辆姿态；
- `tool/path_json_viewer.cpp`：单独打开已导出的路径 JSON；
- `tool/view_path_json.sh`：构建并打开单个路径 JSON，或按顺序打开目录下所有 JSON。

当前默认地图可以规划成功，运行后会生成：

```text
output/result.json
output/single_run_timing.csv
```

FLTK 查看器支持路径播放、暂停、步进和时间轴拖动。

## 构建与运行

推荐直接使用项目脚本：

```bash
./run.sh
```

脚本会在首次运行时配置 CMake，之后构建并运行默认配置：

```text
config/default.yaml
```

`run.sh` 默认把编译临时目录设置到 `build/tmp`。这样即使系统 `/tmp`
空间不足，编译器生成的临时文件也会写在项目构建目录下。

构建：

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release
```

手动运行默认配置：

```bash
./build/hybrid_astar config/default.yaml
```

默认配置会读取 `map/default_map.json`。规划完成后程序会生成
`output/result.json` 和 `output/single_run_timing.csv`，并尝试打开 FLTK 动画窗口。

只生成输出文件、不打开 FLTK 窗口：

```bash
./build/hybrid_astar --no-view config/default.yaml
```

`--html-only` 是 `--no-view` 的兼容别名。无桌面显示、SSH、容器或 CI
环境下推荐使用这个模式，否则 FLTK 可能报 `Can't open display`。

指定其他配置：

```bash
./build/hybrid_astar config/other.yaml
```

查看已导出的路径 JSON：

```bash
./tool/view_path_json.sh output/result.json
```

查看目录下所有路径 JSON，脚本会按文件名排序逐个打开：

```bash
./tool/view_path_json.sh output/obs_heuristic_compare_maps/reverse_dijkstra
```

只列出 JSON 中包含的路径，不打开窗口：

```bash
./tool/view_path_json.sh output/result.json --list
./tool/view_path_json.sh output/obs_heuristic_compare_maps/reverse_dijkstra --list
```

也可以直接调用查看器可执行文件：

```bash
./build/path_json_viewer output/result.json
```

路径 JSON 查看器默认只加载最终路径，不加载 `expanded` 搜索扩展节点，因此大规模实验输出也可以快速打开。

每次运行还会追加一行简要实验日志：

```text
output/single_run_timing.csv
```

日志包含地图路径、是否成功、路径点数量、扩展节点数量、规划耗时、起终点位姿、主要代价参数和启发式名称，方便对比不同参数或地图下的规划效果。

如果配置中开启：

```yaml
hybrid_astar:
  enable_timing: true
```

CSV 还会写入细分计时列，用于分析性能瓶颈：

- `heuristic_prepare_ms`：启发式预处理总耗时；
- `search_loop_ms`：Hybrid A* 主搜索循环耗时；
- `obstacle_collect_ms`、`visibility_points_ms`、`visibility_graph_ms`、`visibility_dijkstra_ms`、`obstacle_lookup_ms`：`visibility_graph` 障碍物启发式预计算各阶段耗时；
- `reverse_dijkstra_inflation_ms`、`reverse_dijkstra_ms`：`reverse_dijkstra` 障碍物膨胀和反向 Dijkstra 耗时；
- `non_obstacle_heuristic_ms`、`obstacle_heuristic_ms`、`heuristic_estimate_calls`：搜索中两种启发式估价耗时和调用次数；
- `primitive_collision_check_ms`、`primitive_collision_check_calls`：运动基元碰撞检测耗时和调用次数；
- `analytic_expansion_ms`、`analytic_attempts`、`analytic_successes`：Reeds-Shepp 解析直连尝试耗时、次数和成功次数；
- `analytic_rs_generation_ms`、`analytic_rs_generation_calls`、`analytic_collision_check_ms`、`analytic_collision_check_calls`：解析直连中的曲线生成和碰撞检测细分统计。

关闭 `enable_timing` 时，上述细分计时列保留但值为 0，外层 `runtime_ms` 仍会记录。

障碍物启发式算法由 `obstacle_heuristic_type` 选择：

- `visibility_graph`：当前默认算法，使用障碍边界可视点、line-of-sight 可视图和查表估价；
- `reverse_dijkstra`：旧版反向 Dijkstra 算法，先按 `obstacle_heuristic_inflation_alpha * vehicle.width / 2` 膨胀障碍物，再从目标格反向计算 8 邻域 cost-to-go。

`obstacle_heuristic_inflation_alpha` 只影响 `reverse_dijkstra`，`0` 表示不膨胀，`1` 表示按车辆半宽膨胀。

### 生成测试地图

生成一组固定尺寸的测试地图：

```bash
python3 scripts/generate_testbench_maps.py \
  --output-dir map/generated \
  --width 60 \
  --height 36
```

生成多种尺寸的测试地图：

```bash
python3 scripts/generate_testbench_maps.py \
  --output-dir map/generated \
  --sizes 40x25 60x36 80x50
```

脚本使用固定模板生成 `empty`、`simple`、`narrow`、`reverse`、`u`、`unreach` 等类别地图，并写出 `maps.txt` 作为索引；不会使用随机障碍物。`testbench` 不需要读取索引文件，直接把输出目录传给 `--maps` 即可：

```bash
./build/hybrid_astar_testbench \
  --groups config/testbench/default_groups.txt \
  --maps map/generated \
  --output output/generated_maps_timing.csv \
  --output-map-dir output/generated_maps_results
```

使用生成地图对比两种障碍物启发式：

```bash
python3 scripts/generate_testbench_maps.py \
  --output-dir map/generated \
  --width 60 \
  --height 36

./scripts/compare_obs_heuristics.sh \
  map/generated \
  output/generated_obs_heuristic_compare.csv \
  output/generated_obs_heuristic_compare_maps
```

`compare_obs_heuristics.sh` 会自动生成两个临时参数组：

```text
visibility_graph
reverse_dijkstra
```

然后对 `map/generated` 下所有 JSON 地图分别运行 testbench。对比脚本会生成三份表格/报告文件：

```text
output/generated_obs_heuristic_compare.csv          # testbench 原始明细
output/generated_obs_heuristic_compare_clean.csv    # 精简后的对比明细
output/generated_obs_heuristic_compare_report.md    # 按启发式汇总的简单报告
```

报告包含每种障碍物启发式的运行次数、成功率、平均耗时、平均扩展节点数、平均迭代次数和失败地图列表。

每次规划的路径 JSON 写入：

```text
output/generated_obs_heuristic_compare_maps/<heuristic_name>/<map_name>.json
```

查看对比输出路径：

```bash
./tool/view_path_json.sh output/generated_obs_heuristic_compare_maps/visibility_graph --list
./tool/view_path_json.sh output/generated_obs_heuristic_compare_maps/reverse_dijkstra --list
```

### 批量实验 testbench

构建 testbench：

```bash
TMPDIR="$PWD/build/tmp" cmake --build build --target hybrid_astar_testbench --config Release
```

运行默认参数组和 `map/` 下所有地图：

```bash
./build/hybrid_astar_testbench \
  --groups config/testbench/default_groups.txt \
  --maps map \
  --output output/default_timing.csv \
  --output-map-dir output/default_timing_maps
```

常用参数：

- `--groups`：参数组列表，每行格式为 `组名 配置文件路径`；
- `--maps`：包含 `.json` 地图文件的目录；
- `--output`：实验 CSV 输出路径；
- `--output-map-dir`：每次规划的 JSON 路径输出目录。

例如只跑自动生成的多组参数：

```bash
./build/hybrid_astar_testbench \
  --groups config/testbench/generated/groups.txt \
  --maps map \
  --output output/generated_timing.csv \
  --output-map-dir output/generated_timing_maps
```

如果系统 `/tmp` 空间不足，和普通构建一样在命令前加：

```bash
TMPDIR="$PWD/build/tmp"
```

配置文件基本格式见 `config/default.yaml`：

```yaml
map_path: map/default_map.json

vehicle:
  length: 4.5
  width: 2.0
  wheelbase: 2.7
  rear_to_center: 1.35
  max_steer: 0.61

hybrid_astar:
  xy_resolution: 1.0
  theta_bins: 360
  step_size: 0.2
  primitive_length: 1.2
  goal_xy_tolerance: 0.2
  goal_theta_tolerance: 0.05
  reverse_penalty: 2.0
  steer_penalty: 1.0
  gear_switch_penalty: 1.0
  steer_change_penalty: 1.0
  max_iterations: 120000
  allow_reverse: true
  enable_analytic_expansion: true
  analytic_expansion_distance: 30.0
  analytic_expansion_interval: 25
  collision_safety_margin: 0.0
  enable_obstacle_heuristic: true
  obstacle_lookup_resolution: 1.0
  obstacle_heuristic_type: visibility_graph
  obstacle_heuristic_inflation_alpha: 1.0
  enable_timing: true
  debug: true
  debug_progress_interval: 500
```

Windows 使用 Visual Studio 生成器时，可执行文件通常在：

```text
build/Release/hybrid_astar.exe
```

### 常见运行问题

如果编译时报：

```text
fatal error: error writing to /tmp/...: No space left on device
```

说明系统临时目录所在分区空间不足。优先使用 `./run.sh`，它会自动使用
`build/tmp` 作为临时目录。手动构建时也可以显式指定：

```bash
TMPDIR="$PWD/build/tmp" cmake --build build --config Release
```

如果运行结束时报：

```text
Can't open display: :0
```

说明当前环境没有可用图形显示。规划结果 JSON 通常已经写出，可以改用：

```bash
./build/hybrid_astar --no-view config/default.yaml
```

## 地图编辑

打开：

```text
map/grid_demo.html
```

可以在浏览器中：

- 点击方格添加或删除障碍物；
- 设置起点；
- 设置终点；
- 设置起点/终点朝向；
- 导出 JSON 地图。

地图 JSON 基本格式：

```json
{
  "version": 1,
  "width": 60,
  "height": 36,
  "start": {"x": 6, "y": 6, "theta": 0.0},
  "goal": {"x": 52, "y": 28, "theta": 0.0},
  "obstacles": [
    {"x": 12, "y": 12}
  ]
}
```

`theta` 使用弧度。`obstacles` 使用栅格坐标。

## 车辆模型

当前采用简化 bicycle model：

```text
x, y   = 后轴中心
theta  = 车身朝向
steer  = 前轮转向角
```

每个小步使用后轴中心弧长 `ds`：

```text
kappa = tan(steer) / wheelbase

x_next     = x + direction * ds * cos(theta)
y_next     = y + direction * ds * sin(theta)
theta_next = theta + direction * ds * kappa
```

动作集合是 6 个 motion primitives：

```text
前进左转 / 前进直行 / 前进右转
倒车左转 / 倒车直行 / 倒车右转
```

也就是：

```text
direction ∈ {+1, -1}
steer ∈ {-max_steer, 0, +max_steer}
```

## 待改进点

这个版本已经打通“地图输入 -> C++ 规划 -> JSON 输出 -> FLTK 查看”的主链路。后续改进建议按下面优先级推进。

### P0 展示前优先处理

当前 P0 项已经完成：

- 文档与代码状态同步；
- 默认地图路径统一为 `map/default_map.json`；
- 运行时 HTML 生成逻辑已移除，统一输出 JSON 并使用 FLTK/独立查看器查看。

### P1 影响规划质量

1. 增强启发式（已完成第一版）  
   默认启发式已经改为组合启发式：`h = max(h_non_obs, h_obs)`。其中 `h_obs` 是膨胀障碍图上的 8 邻域 Dijkstra cost-to-go，`h_non_obs` 优先使用 Reeds-Shepp 风格无障碍距离，失败时回退到欧几里得距离。
2. 补齐更完整的 Reeds-Shepp 候选  
   当前 analytic expansion 是简化 Reeds-Shepp 风格直连，本质仍偏 Dubins + 反向候选，没有完整覆盖带 cusp 的 Reeds-Shepp words。泊车、窄通道和需要多次倒车的场景会受影响。
3. 优化代价函数（已完成第一版）  
   `g cost` 已包含实际轨迹段长度、倒车惩罚、转向幅度惩罚、换挡惩罚和转向变化惩罚。靠障碍惩罚暂不加入，避免和障碍物启发式重复耦合。

### P2 提升课堂展示效果

1. 在 Canvas 中可选显示 `expanded` 搜索节点  
   `expanded` 已输出到 JSON，但独立路径查看器默认跳过该字段以保证大文件打开速度。后续可增加命令行开关，在需要讲解搜索过程时再加载和绘制扩展点。
2. 增加路径平滑  
   当前 motion primitive 拼接路径可行但不够顺。可以增加简单 smoother，或至少做采样点降噪和曲率连续性优化。
3. 增加规划失败诊断信息  
   失败时建议输出迭代次数、扩展节点数量、是否起终点碰撞、最后距离目标多远，方便调参和课堂排查。

### P3 工程完善

1. 参数配置化  
   将 `HybridAstarConfig` 中的搜索分辨率、容差、代价权重、启发式和 analytic expansion 参数移入配置文件或命令行参数。
2. JSON 解析更稳健  
   当前地图解析是针对当前格式的轻量实现。课程 demo 可接受，后续可以换成标准 JSON 库以提升容错性。
3. 增加自动测试  
   建议覆盖空地图可达、起点碰撞失败、窄通道、必须倒车、不可达地图等典型场景。
4. 增加普通 A* 对比路径  
   用于课堂解释 Hybrid A* 与普通栅格 A* 在车辆运动约束上的区别。

更多设计细节见：

```text
design.md
output/design.md
```
