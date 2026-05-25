# 重新运行 `report.md` 中测试后的结果分析

本文基于当前仓库代码重新执行了 `report.md` 中第 5 章实际可复现的测试与实验，并对新结果进行了汇总分析。所有命令均在当前代码、当前参数和当前地图集上重新运行，而不是沿用旧报告中的历史输出。

## 1. 重新执行的项目

本次实际重跑了以下内容：

- `5.2 Reeds-Shepp 空地图测试`
- `5.3 Line of Sight 测试`
- `5.4 AppConfig 配置测试`
- `5.5 主程序集成测试`
- `5.8 高角度分辨率导致原地转圈`
- `5.9 障碍物启发式对比`
- `5.10 默认参数组批量 testbench 时间分解`

说明：

- `5.6 测试数据生成` 和 `5.7 发现的问题与修改` 不是独立实验，不需要“重跑”。
- `5.10` 的旧输出文件 `output/default_timing.csv` 采用追加写入，因此分析时只使用本次最新生成的最后 `14` 行数据；否则会把历史结果混入。

## 2. 基础测试结果

### 2.1 Reeds-Shepp 空地图测试

执行命令：

```bash
./build/reeds_shepp_empty_map_test
```

结果：

- 测试通过。
- 输出信息为 `Reeds-Shepp empty map tests passed`。
- 生成了 `output/reeds_shepp_empty_map_test.json`。

结论：

- `5.2` 中关于 Reeds-Shepp 模块基本正确性的描述，当前版本仍然成立。

### 2.2 Line of Sight 测试

执行命令：

```bash
./build/line_of_sight_test
```

结果：

- `17 line_of_sight checks passed`

结论：

- `5.3` 中关于 line-of-sight 网格穿越与遮挡判断的基本结论仍然成立。

### 2.3 AppConfig 配置测试

执行命令：

```bash
./build/app_config_test
```

结果：

- `25 app_config checks passed`

补充：

- 在重跑过程中，这个测试一度因为测试临时目录实现不稳而崩溃；现已修复为使用 `build/app_config_test_tmp/`，并已重新验证通过。

结论：

- `5.4` 中关于配置项解析和错误处理的结论，当前版本成立。

### 2.4 主程序集成测试

执行命令：

```bash
./build/hybrid_astar --no-view config/default.yaml
```

结果摘要：

- 地图：`map/default_map.json`
- 成功：`1`
- 路径点数：`135`
- 扩展节点数：`25200`
- 迭代次数：`29995`
- 生成节点数：`43882`
- 运行时间：`278.84 ms`
- 输出文件：`output/result.json`、`output/single_run_timing.csv`

结论：

- `5.5` 中关于主程序可以在默认地图上完成从读配置到生成结果文件的最小闭环，这一点成立。
- 不过当前默认配置里 `debug: true`，因此运行日志非常多；这与是否为 Release 构建无关。

## 3. 5.8 高角度分辨率实验重新分析

本次使用与报告一致的参数重新构造了实验配置，并执行：

```bash
./build/hybrid_astar --no-view build/report_5_8_theta360.yaml
```

实验结果如下：

- 地图：`map/default_map.json`
- `theta_bins = 360`
- 成功：`1`
- 路径点数：`677`
- 扩展节点数：`171045`
- 迭代次数：`223801`
- 生成节点数：`229326`
- 运行时间：`4078.43 ms`

从日志现象看，搜索过程中确实出现了大量近似姿态反复扩展：

- 直到 `223801` 次迭代才到达目标。
- 扩展节点达到 `171045`，远高于默认参数下同地图的 `25200`。
- 最终路径点数达到 `677`，明显长于默认参数下的 `135`。

结论：

- `5.8` 的核心判断仍然成立：`theta_bins = 360` 会显著放大状态空间，使闭集去重过细，导致大量近似姿态无法合并。
- 旧报告里把这种现象概括为“原地转圈”，从现有日志看，这个表述更准确地说应当是“局部重复搜索和小角度调整显著增加”。它不一定表现为完全不动的原地旋转，但会表现为非常明显的低效扩展。

## 4. 5.9 障碍物启发式对比重新分析

执行命令：

```bash
./scripts/compare_obs_heuristics.sh \
  map/generated \
  output/generated_obs_heuristic_compare.csv \
  output/generated_obs_heuristic_compare_maps
```

本次实验统计结果如下。

| 障碍物启发式 | 地图数 | 成功数 | 成功率 | 平均运行时间/ms | 平均扩展节点数 | 平均迭代次数 | 平均路径点数 |
|---|---:|---:|---:|---:|---:|---:|---:|
| `reverse_dijkstra` | 39 | 32 | 82.05% | 85.938 | 8741.1 | 10174.3 | 82.3 |
| `visibility_graph` | 39 | 33 | 84.62% | 70.724 | 6703.7 | 7759.8 | 87.6 |

与旧报告相比，结论方向没有变，但时间量级整体下降了很多：

- 旧报告平均运行时间是 `1358.891 ms` vs `1119.054 ms`
- 本次重跑是 `85.938 ms` vs `70.724 ms`

这说明当前版本或当前构建环境下，整体性能已明显提升；但**两种启发式的相对关系**没有变。

重新计算后的三个直接结论：

1. `visibility_graph` 的成功率仍然更高，高 `2.56%`。
2. `visibility_graph` 的平均运行时间更低，从 `85.938 ms` 降到 `70.724 ms`，平均耗时下降约 `17.71%`。
3. `visibility_graph` 的平均扩展节点更少，从 `8741.1` 降到 `6703.7`，平均扩展规模下降约 `23.31%`。

失败样例如下：

- `visibility_graph` 失败：`40x25_unreach01.json`、`60x36_unreach01.json`、`80x50_narrow03.json`、`80x50_u01.json`、`80x50_u02.json`、`80x50_unreach01.json`
- `reverse_dijkstra` 失败：`40x25_unreach01.json`、`60x36_unreach01.json`、`80x50_narrow02.json`、`80x50_narrow03.json`、`80x50_u01.json`、`80x50_u02.json`、`80x50_unreach01.json`

结论：

- 原报告中“在当前实现与参数下，`visibility_graph` 更适合作为默认障碍物启发式”的判断，当前重跑后依然成立。
- 之前已经修正过的那句严谨表述也仍然适用：这里比较的是 `max(h_non_obs, h_obs)` 这个组合启发式在当前 Hybrid A* 实现中的实际效果，而不是孤立比较两种 obstacle heuristic 公式的理论优劣。

## 5. 5.10 默认参数组批量 testbench 重新分析

执行命令：

```bash
./build/hybrid_astar_testbench \
  --groups config/testbench/default_groups.txt \
  --maps map \
  --output output/default_timing.csv \
  --output-map-dir output/default_timing_maps
```

注意：

- `output/default_timing.csv` 会追加写入。
- 这次分析只使用本次新增的最后 `14` 行结果。

### 5.10.1 总体统计

本次最新一轮 `14` 张地图的总体结果如下：

- 参数组数量：`1`
- 地图数量：`14`
- 成功次数：`12`
- 失败次数：`2`
- 成功率：`85.71%`

主要统计量如下：

| 指标 | 平均值 |
|---|---:|
| 平均总运行时间 `runtime_ms` | `106.198 ms` |
| 平均扩展节点数 `expanded_nodes` | `8340.4` |
| 平均迭代次数 `iterations` | `9628.1` |
| 平均生成节点数 `generated_nodes` | `14771.3` |
| 平均路径点数 `path_poses` | `76.0` |

这里有一个非常重要的差异：

- 旧报告写的是平均总运行时间 `1884.603 ms`
- 本次重跑得到的是 `106.198 ms`

但扩展节点数、迭代次数、生成节点数、路径点数几乎完全一致。

这说明：

- 当前版本的搜索行为与旧报告基本一致；
- 但运行时间量级已明显下降；
- 因而 `5.10` 中所有“毫秒级时间分解”的旧数字都已经过时，不能继续沿用。

### 5.10.2 成功样例上的细分计时

只统计本次 `12` 个成功样例，平均模块耗时如下：

| 模块 | 平均耗时/ms | 占成功样例平均总时间比例 |
|---|---:|---:|
| 启发式预处理 `heuristic_prepare_ms` | `5.345` | `8.79%` |
| 主搜索循环 `search_loop_ms` | `53.532` | `88.07%` |
| 运动基元碰撞检测 `primitive_collision_check_ms` | `19.550` | `32.17%` |
| 解析扩展 `analytic_expansion_ms` | `4.331` | `7.12%` |
| 障碍物查表构建 `obstacle_lookup_ms` | `5.141` | `8.46%` |
| 解析扩展中的 Reeds-Shepp 生成 `analytic_rs_generation_ms` | `2.562` | `4.21%` |
| 解析扩展中的碰撞检测 `analytic_collision_check_ms` | `1.706` | `2.81%` |

这里也和旧报告有明显差异：

- 旧报告中的成功样例平均 `search_loop_ms` 是 `1259.493 ms`
- 本次重跑只有 `53.532 ms`
- 旧报告中的 `primitive_collision_check_ms` 是 `991.263 ms`
- 本次重跑只有 `19.550 ms`

不过结论方向仍然一致：

- 程序的主要时间仍集中在主搜索循环。
- 运动基元扩展和碰撞检测仍然是重要热点。
- 解析扩展不是这组实验下的主要瓶颈。

只是需要把“主要热点”的表述写得更谨慎一些。当前数据下：

- `primitive_collision_check_ms` 当然仍然重要；
- 但它只占成功样例平均总时间的约 `32.17%`，已经不是旧报告里那种“压倒性占比 72.66%”的量级了。

因此，更准确的表述应改成：

“在默认参数组和 `map/` 测试集上，程序的主要耗时仍集中在主搜索循环，其中运动基元相关的碰撞检测是重要开销来源之一，但其占比已低于旧报告中的历史结果。”

### 5.10.3 失败样例

本次失败地图仍然是：

- `u01.json`
- `unreach01.json`

两者都在 `30000` 次迭代上限后退出：

- `u01.json`：`374.777 ms`，`26281` 扩展节点，`30000` 次迭代
- `unreach01.json`：`382.058 ms`，`25553` 扩展节点，`30000` 次迭代

与旧报告相比，这两个失败样例的时间也大幅下降，但失败模式没有变化。

结论：

- 原报告中“失败时高耗时主要来自搜索空间被持续扩展，而不是某个预处理阶段异常变慢”的判断仍然成立。
- 但旧报告里具体的时间数字已经失效，需要整体更新。

## 6. 对 `report.md` 当前内容的结论性判断

重新运行后，可以把 `report.md` 中第 5 章的内容分成三类看。

### 6.1 仍然成立的部分

- `5.2` Reeds-Shepp 空地图测试结论
- `5.3` Line of Sight 测试结论
- `5.4` AppConfig 配置测试结论
- `5.5` 主程序集成测试“能够打通最小闭环”的结论
- `5.8` 高 `theta_bins` 会显著放大状态空间的结论
- `5.9` `visibility_graph` 在当前实现与参数下整体优于 `reverse_dijkstra` 的结论

### 6.2 方向对，但数字已经过时的部分

- `5.9` 中平均运行时间的毫秒数
- `5.10` 中所有时间相关统计，尤其是：
  - `runtime_ms`
  - `heuristic_prepare_ms`
  - `search_loop_ms`
  - `primitive_collision_check_ms`
  - `analytic_expansion_ms`
  - `obstacle_lookup_ms`
  - `analytic_rs_generation_ms`
  - `analytic_collision_check_ms`

### 6.3 需要收紧表述的部分

- `5.8` 中“原地转圈”更适合改成“局部重复搜索和小角度调整显著增加”
- `5.10` 中“`primitive_collision_check_ms` 约占成功样例总时间的 `72.66%`，是主要热点”这类强表述，现在已经不适合继续保留

## 7. 建议

如果你准备把 `report.md` 更新到与当前代码一致，优先建议改两处：

1. 把 `5.9` 和 `5.10` 的时间统计全部替换成这次重跑的新数字。
2. 把 `5.10` 的性能瓶颈描述改得更保守，不要继续沿用旧版本中明显过时的时间占比结论。

如果你要，我下一步可以直接基于这份 `new.md`，把 `report.md` 第 5 章对应小节改成与当前实验结果一致的正式版文字。
