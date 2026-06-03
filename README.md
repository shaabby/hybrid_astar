# Hybrid A* 路径规划 Demo

基于 C++23 的 Hybrid A* 路径规划课程项目，包含 JSON 栅格地图读取、简化车辆 bicycle model、Hybrid A* 搜索、Reeds-Shepp 解析扩展、路径 JSON 导出、独立 FLTK 路径查看器及批量实验与 CSV 日志功能。

典型流程：

```text
地图 JSON -> C++ 规划 -> result.json / CSV -> 独立查看器展示
```

## 编译环境要求

| 依赖 | 版本要求 | 说明 |
|---|---|---|
| CMake | >= 3.23 | 跨平台构建工具 |
| C++ 编译器 | 支持 C++23 | 见下方各平台说明 |
| Python 3 | 可选 | 仅用于地图生成脚本与对比脚本 |
| FLTK | 内置（third_party） | 无需额外安装 |

其他第三方依赖均已内嵌在 `third_party/` 目录中，不需要单独安装。

---

## 一、Windows 编译与运行

### 编译器要求

- **MSVC**（Visual Studio 2022 或更高，需支持 `/std:c++latest`）
- 或 **MinGW-w64**（GCC 13+，支持 C++23）

### CMake 安装

从 [CMake Releases](https://github.com/Kitware/CMake/releases) 下载 Windows x64 ZIP 包，将 `cmake/bin/` 所在目录加入系统 `PATH` 环境变量。`build.bat` 会从 PATH 中查找 `cmake` 命令。

### 四个 bat 脚本说明

项目提供了 4 个 Windows 批处理脚本，覆盖构建、单次运行、批量实验全流程：

| 脚本 | 作用 |
|---|---|
| `build.bat` | 编译项目，生成可执行文件 |
| `run.bat` | 运行规划器（处理单个地图配置文件） |
| `demo.bat` | 使用默认参数组批量运行 `map\` 目录下所有地图，并逐个展示结果 |
| `demo_para.bat` | 在 `empty01` 地图上对比三组搜索离散参数（精细/中等/粗糙），分批展示结果 |

#### build.bat

自动检测 `cmake/bin/cmake.exe`，首次运行执行 CMake 配置，随后构建 Release 版可执行文件：

```bat
build.bat
```

编译成功后，可执行文件位于 `build\Release\` 目录下（MSVC 生成器）或 `build\` 目录下（MinGW 生成器）。

#### run.bat

运行规划器，使用默认配置文件 `config/default.yaml`：

```bat
run.bat
```

也可以指定其他配置文件：

```bat
run.bat config\my_config.yaml
```

程序输出 `output\result.json` 和 `output\single_run_timing.csv`，并自动调用 FLTK 查看器展示结果。

#### demo.bat

批量运行 `map\` 下所有地图，每组配置输出一条 CSV 日志，查看器逐张展示生成的所有 JSON 路径：

```bat
demo.bat
```

#### demo_para.bat

只在 `map\empty01.json` 上运行三组不同离散精度参数的对比实验（`fine` / `medium` / `coarse`），并分三批展示结果：

```bat
demo_para.bat
```

### 手动编译

如果不想使用 `build.bat`，可以直接调用 CMake：

```bat
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release
```

使用 MinGW 生成器时的示例：

```bat
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -G "MinGW Makefiles"
cmake --build build
```

### 手动运行

```bat
build\Release\hybrid_astar.exe config\default.yaml
```

---

## 二、Linux (Ubuntu) 编译与运行

### 编译器与依赖安装

```bash
# Ubuntu 24.04
sudo apt install build-essential cmake g++-14

# Ubuntu 22.04 需要手动安装较新的 GCC
sudo apt install build-essential cmake g++-13
```

确保编译器支持 C++23（GCC 13+ / Clang 17+）。

### 快速开始（推荐）

项目根目录提供了 `run.sh` 一键脚本，自动配置 CMake、构建并运行：

```bash
./run.sh
```

### 手动编译

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release
```

编译产物位于 `build/hybrid_astar`。

如果 `/tmp` 空间不足（FLTK 编译占用较大），可以指定临时目录：

```bash
TMPDIR="$PWD/build/tmp" cmake --build build --config Release
```

### 手动运行

```bash
./build/hybrid_astar config/default.yaml
```

输出文件：

```text
output/result.json
output/single_run_timing.csv
```

---

## 三、查看结果

### Windows

打开单个路径 JSON：

```bat
tool\view_path_json.bat output\result.json
```

打开目录下所有 JSON（批量查看）：

```bat
tool\view_path_json.bat output\demo\default
```

只列出路径，不打开窗口：

```bat
tool\view_path_json.bat output\result.json --list
```

也可以直接运行查看器：

```bat
build\Release\path_json_viewer.exe output\result.json
```

### Linux

```bash
./tool/view_path_json.sh output/result.json
```

只列出路径：

```bash
./tool/view_path_json.sh output/result.json --list
```

直接运行查看器：

```bash
./build/path_json_viewer output/result.json
```

---

## 四、测试

项目使用 CTest 管理测试：

```bash
ctest --test-dir build --output-on-failure
```

当前包含以下测试：

- `reeds_shepp_empty_map_test` — Reeds-Shepp 空地图测试
- `line_of_sight_test` — 视线检测测试
- `app_config_test` — 配置加载测试

---

## 五、批量实验

`hybrid_astar_testbench` 支持批量参数实验。

使用预定义参数组：

```bash
./build/hybrid_astar_testbench \
  --groups config/testbench/default_groups.txt \
  --maps map \
  --output output/default_timing.csv \
  --output-map-dir output/default_timing_maps
```

使用参数扫描（笛卡尔积）：

```bash
./build/hybrid_astar_testbench \
  --base-config config/default.yaml \
  --param hybrid_astar.xy_resolution=0.1,0.5,1 \
  --param hybrid_astar.step_size=0.1,0.5,1 \
  --maps map \
  --output output/param_sweep.csv \
  --output-map-dir output/param_sweep_maps
```

参数说明：

- `--groups`：参数组文件列表
- `--base-config`：基础配置 YAML
- `--param`：参数扫描定义，格式 `key=v1,v2,...`，可重复指定
- `--maps`：地图文件目录
- `--output`：CSV 结果输出路径
- `--output-map-dir`：每次规划的 JSON 输出目录

Windows 下对应可执行文件为 `build\Release\hybrid_astar_testbench.exe`。

---

## 六、配置说明

主配置文件为 `config/default.yaml`，可通过 `--base-config` 或指定任意 YAML 覆盖。

核心参数：

| 参数 | 说明 |
|---|---|
| `map_path` | 地图 JSON 文件路径 |
| `vehicle.length / width / wheelbase / max_steer` | 车辆物理参数 |
| `hybrid_astar.xy_resolution` | 位置去重分辨率，越小越精确但越慢 |
| `hybrid_astar.theta_bins` | 航向角分箱数 |
| `hybrid_astar.step_size` | 运动学积分步长 |
| `hybrid_astar.primitive_length` | 单个运动基元总长度 |
| `hybrid_astar.max_iterations` | 最大搜索迭代数 |
| `hybrid_astar.enable_analytic_expansion` | 是否启用 Reeds-Shepp 直连 |
| `hybrid_astar.enable_obstacle_heuristic` | 是否启用障碍物启发式 |
| `hybrid_astar.obstacle_heuristic_type` | `visibility_graph` 或 `reverse_dijkstra` |
| `hybrid_astar.reverse_penalty / steer_penalty` 等 | 代价函数惩罚系数 |

---

## 七、生成测试地图

```bash
python3 scripts/generate_testbench_maps.py --output-dir map/generated --sizes 40x25 60x36 80x50
```

---

## 八、障碍物启发式对比

```bash
./scripts/compare_obs_heuristics.sh \
  map/generated \
  output/generated_obs_heuristic_compare.csv \
  output/generated_obs_heuristic_compare_maps
```

---

## 九、地图编辑

浏览器打开 `map/grid_demo.html` 即可编辑障碍物、起点、终点和朝向，支持导出/导入 JSON 地图文件。无需本地服务器，双击即可使用。

---

## 目录结构

| 目录 | 说明 |
|---|---|
| `config/` | YAML 配置与 testbench 参数组 |
| `map/` | 地图文件与 Web 地图编辑器 |
| `src/` | 核心算法实现 |
| `include/` | 头文件 |
| `tool/` | 独立路径查看器与 testbench |
| `tests/` | 自动测试程序 |
| `scripts/` | 地图生成与实验分析脚本 |
| `output/` | 运行输出（JSON、CSV、报告） |
| `third_party/` | FLTK、Reeds-Shepp 等第三方依赖 |

## 常见问题

**无图形显示环境**：主程序只负责规划并输出 JSON；跳过查看器步骤即可。

**CMake 找不到**：Windows 下 `build.bat` 从系统 PATH 查找 `cmake`，请确保已安装 CMake 并加入 PATH。

**编译时 `/tmp` 空间不足**：
```bash
TMPDIR="$PWD/build/tmp" cmake --build build --config Release
```

**Windows 可执行文件路径**：MSVC 生成器下位于 `build/Release/`，MinGW 下位于 `build/`。
