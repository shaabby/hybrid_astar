# Tool 使用说明

## 路径 JSON 查看器

`view_path_json.sh` 会自动构建 `path_json_viewer`，然后用 FLTK 窗口打开路径 JSON。

默认打开 `output/result.json`：

```bash
./tool/view_path_json.sh
```

打开单个 JSON 文件：

```bash
./tool/view_path_json.sh output/result.json
```

打开目录下所有 JSON 文件，脚本会按文件名排序逐个打开：

```bash
./tool/view_path_json.sh output/obs_heuristic_compare_maps/reverse_dijkstra
```

只列出 JSON 中包含的路径，不打开窗口：

```bash
./tool/view_path_json.sh output/result.json --list
./tool/view_path_json.sh output/obs_heuristic_compare_maps/reverse_dijkstra --list
```

选择指定路径打开：

```bash
./tool/view_path_json.sh output/reeds_shepp_empty_map_test.json --path map_start_to_goal
./tool/view_path_json.sh output/reeds_shepp_empty_map_test.json --path 1
```

`path_json_viewer` 默认只加载最终路径，不加载 `expanded` 搜索扩展节点，用于保证大规模实验 JSON 的打开速度。

## 批量实验工具

`testbench.cpp` 对参数组和地图目录做批量规划实验，输出 CSV 和每次规划的路径 JSON。

常用构建：

```bash
cmake --build build --target hybrid_astar_testbench --config Release
```

常用运行：

```bash
./build/hybrid_astar_testbench \
  --groups config/testbench/generated/groups.txt \
  --maps map \
  --output output/generated_timing.csv \
  --output-map-dir output/generated_timing_maps
```

## 测试地图生成

`../scripts/generate_testbench_maps.py` 可以批量生成用于 testbench 的地图 JSON。

生成默认尺寸地图：

```bash
python3 scripts/generate_testbench_maps.py \
  --output-dir map/generated \
  --width 60 \
  --height 36 \
  --seed 42
```

生成多尺寸地图：

```bash
python3 scripts/generate_testbench_maps.py \
  --output-dir map/generated \
  --sizes 40x25 60x36 80x50 \
  --seed 42
```

生成结果可以直接传给 testbench：

```bash
./build/hybrid_astar_testbench \
  --groups config/testbench/generated/groups.txt \
  --maps map/generated \
  --output output/generated_maps_timing.csv \
  --output-map-dir output/generated_maps_results
```

输出目录里的 JSON 可以继续用 `view_path_json.sh` 打开。
