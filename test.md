# Hybrid A* Testbench 测试说明

## 目标

本 testbench 用于批量验证 Hybrid A* 在不同地图和不同参数组下的搜索表现，主要记录：

- 搜索是否成功
- 搜索耗时 `runtime_ms`
- 扩展节点数 `expanded_nodes`
- 迭代次数 `iterations`
- 生成节点数 `generated_nodes`
- open set 剩余数量 `open_remaining`

测试会遍历 `map/` 目录下所有 `.json` 地图，并对每个参数组分别运行一次规划。

## 测试参数

当前重点测试 7 个参数：

| 参数 | baseline | 测试值 |
| --- | ---: | --- |
| `theta_bins` | `72` | `36`, `72`, `144` |
| `reverse_penalty` | `2.0` | `1.0`, `2.0`, `5.0` |
| `steer_penalty` | `1.0` | `0.0`, `1.0`, `4.0` |
| `gear_switch_penalty` | `1.0` | `0.0`, `1.0`, `4.0` |
| `steer_change_penalty` | `1.0` | `0.0`, `1.0`, `4.0` |
| `analytic_expansion_distance` | `8` | `4`, `8`, `16` |
| `analytic_expansion_interval` | `25` | `5`, `25`, `100` |

`max_iterations` 在所有参数组中固定，默认使用命令行传入的值。

## 参数组生成

使用脚本生成测试配置：

```bash
python3 scripts/generate_testbench_configs.py \
  --base config/default.yaml \
  --output-dir config/testbench/generated \
  --max-iterations 1200000 \
  --mode combo
```

生成结果：

- 参数配置文件：`config/testbench/generated/*.yaml`
- 参数组索引：`config/testbench/generated/groups.txt`

`groups.txt` 每行格式为：

```text
group_name config_path
```

例如：

```text
baseline config/testbench/generated/baseline.yaml
theta_bins_36 config/testbench/generated/theta_bins_36.yaml
reverse_penalty_1p0 config/testbench/generated/reverse_penalty_1p0.yaml
```

## 生成模式

脚本支持四种模式：

### `single`

只生成 baseline 和单因素参数组。

数量：

```text
1 + 7 * 2 = 15 组
```

适合快速观察单个参数变化的影响。

### `combo`

默认推荐模式。

包含：

- baseline
- 单因素参数组
- 若干关键组合参数组

当前约 25 组，适合日常测试和结果分析。

### `penalty`

只组合四个 penalty 参数：

- `reverse_penalty`: `1.0`, `2.0`, `5.0`
- `steer_penalty`: `0.0`, `1.0`, `4.0`
- `gear_switch_penalty`: `0.0`, `1.0`, `4.0`
- `steer_change_penalty`: `0.0`, `1.0`, `4.0`

数量：

```text
3^4 = 81 组
```

其他参数保持 baseline，包括：

- `theta_bins = 72`
- `analytic_expansion_distance = 8`
- `analytic_expansion_interval = 25`

生成 penalty 模式示例：

```bash
python3 scripts/generate_testbench_configs.py \
  --base config/default.yaml \
  --output-dir config/testbench/generated_penalty \
  --max-iterations 1200000 \
  --mode penalty
```

适合专门分析四个代价参数之间的组合影响。



```text
3^7 = 2187 组
```

适合最终完整实验或离线长时间运行，不建议日常调试直接使用。

生成 full 模式示例：

```bash
python3 scripts/generate_testbench_configs.py \
  --base config/default.yaml \
  --output-dir config/testbench/generated_full \
  --max-iterations 1200000 \
  --mode full
```

## 构建 testbench

配置并构建：

```bash
cmake -S . -B build
cmake --build build --target hybrid_astar_testbench
```

也可以构建原主程序：

```bash
cmake --build build --target hybrid_astar
```

## 运行 testbench

运行推荐组合测试：

```bash
./build/hybrid_astar_testbench \
  --groups config/testbench/generated/groups.txt \
  --maps map \
  --output output/testbench.csv \
  --output-map-dir output/testbench_maps
```

参数说明：

| 参数 | 含义 |
| --- | --- |
| `--groups` | 参数组索引文件路径 |
| `--maps` | 地图目录，程序会读取其中所有 `.json` 文件 |
| `--output` | CSV 输出路径 |
| `--output-map-dir` | 每次测试的 JSON/HTML outputmap 输出目录 |

程序会对每个 `参数组 × 地图` 组合运行一次 Hybrid A*。

单张地图规划失败不会中断整个 testbench，失败结果也会记录在 CSV 中。

每次测试还会自动保存一份 outputmap：

```text
output/testbench_maps/<parameter_group>/<map_name>.json
output/testbench_maps/<parameter_group>/<map_name>.html
```

成功和失败都会保存。成功时 JSON/HTML 包含 path 和 expanded nodes；失败时 path 为空，但仍保留 expanded nodes，方便查看搜索扩展到哪里。

## CSV 输出字段

输出文件默认为：

```text
output/testbench.csv
```

主要字段：

| 字段 | 含义 |
| --- | --- |
| `timestamp` | 记录时间 |
| `parameter_group` | 参数组名称 |
| `map_path` | 地图路径 |
| `success` | 是否搜索成功，`1` 表示成功，`0` 表示失败 |
| `path_poses` | 最终路径点数量 |
| `expanded_nodes` | 扩展节点数量 |
| `iterations` | 搜索循环迭代次数 |
| `generated_nodes` | 搜索过程中生成的节点数量 |
| `open_remaining` | 搜索结束时 open set 中剩余条目数量 |
| `runtime_ms` | 规划耗时，单位毫秒 |
| `max_iterations` | 最大搜索迭代次数 |
| `theta_bins` | 航向角离散数量 |
| `reverse_penalty` | 倒车代价倍率 |
| `steer_penalty` | 转向代价惩罚 |
| `gear_switch_penalty` | 前进/倒车切换惩罚 |
| `steer_change_penalty` | 相邻运动基元转向变化惩罚 |
| `analytic_expansion_distance` | 解析扩展距离阈值 |
| `analytic_expansion_interval` | 解析扩展尝试间隔 |
| `heuristic_name` | 使用的启发式名称 |
| `enable_obstacle_heuristic` | 是否启用障碍物启发式 |
| `enable_analytic_expansion` | 是否启用解析扩展 |

## 结果检查建议

运行完成后重点检查：

1. 每张地图是否都出现了 `参数组数量` 次。
2. `success = 1` 的行是否有 `path_poses > 0`。
3. `success = 0` 的行是否仍记录了 `runtime_ms`、`expanded_nodes`、`iterations`。
4. 比较不同参数组在同一张地图上的：
   - 成功率
   - 搜索时间
   - expanded nodes
   - iterations
5. 重点关注以下地图类型：
   - `reverse*.json`：观察倒车相关 penalty 的影响。
   - `u*.json`：观察换挡和倒车代价的影响。
   - `narrow*.json`：观察 `theta_bins` 和转向 penalty 的影响。
   - `unreach*.json`：观察失败场景的搜索成本。

## 快速 smoke test

如果只想快速验证 testbench 是否能跑通，可以创建一个只包含 baseline 的 groups 文件：

```text
baseline config/testbench/generated/baseline.yaml
```

例如保存为：

```text
output/testbench_smoke_groups.txt
```

然后运行：

```bash
./build/hybrid_astar_testbench \
  --groups output/testbench_smoke_groups.txt \
  --maps map \
  --output output/testbench_smoke.csv
```

这样只会运行 `baseline × 所有地图`。

## 现有测试

运行已有 CTest：

```bash
ctest --test-dir build --output-on-failure
```

当前验证时发现：

- `line_of_sight_test` 通过
- `app_config_test` 通过
- `reeds_shepp_empty_map_test` 失败，原因是测试期望 `map/empty.json`，但当前仓库中是 `empty01.json`、`empty02.json`、`empty03.json`

该失败与 testbench 改动无关，是既有测试数据路径问题。
