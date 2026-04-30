# Hybrid A* 路径规划最小闭环 Demo

本项目是一个 C++23 + HTML Canvas 的 Hybrid A* 路径规划课堂展示 demo。

当前目标不是完整工业级规划器，而是先打通最小闭环：

```text
Web 地图编辑器
    ↓ 导出 JSON 地图
C++ 读取地图
    ↓ 构造 GridMap / Car
Hybrid A* 生成车辆位姿序列
    ↓ 输出 result.json / demo.html
浏览器 Canvas 动态渲染地图和小车运动
```

## 当前已完成

已经完成最小可运行闭环：

- `map/grid_demo.html`：浏览器中的方格地图编辑器；
- `GridMap`：从 JSON 读取地图、起点、终点和障碍物；
- `Car`：简化 bicycle model，小车以后轴中心为状态参考点；
- `HybridAstar`：最小 Hybrid A* 核心搜索；
- `JsonExporter`：输出 `map / vehicle / path / expanded`；
- `HtmlWriter`：生成单文件 Canvas 动画页面；
- `output/demo.html`：可直接双击打开查看动画。

当前默认地图可以规划成功，运行后会生成：

```text
output/result.json
output/demo.html
```

动画页面支持：

- 播放 / 暂停；
- 到达终点后自动停止；
- 点击 `Start` 重新开始；
- `Step` 逐帧播放；
- 进度条拖动；
- 当前帧数显示。

## 构建与运行

构建：

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release
```

运行默认地图：

```bash
./build/hybrid_astar
```

指定地图：

```bash
./build/hybrid_astar map/default_map.json
```

然后打开：

```text
output/demo.html
```

Windows 使用 Visual Studio 生成器时，可执行文件通常在：

```text
build/Release/hybrid_astar.exe
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
  "start": {"x": 6.5, "y": 6.5, "theta": 0.0},
  "goal": {"x": 52.5, "y": 28.5, "theta": 0.0},
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

## 当前局限

这个版本是“极简 Hybrid A*”，还有明显局限：

- 碰撞检测目前只检查车辆参考点所在栅格，没有检查完整车身矩形；
- 启发式只使用欧几里得距离，没有 Reeds-Shepp 或 Dubins 启发；
- 代价函数较简单，只包含基础距离、倒车惩罚和转向惩罚；
- 路径没有做平滑；
- `expanded` 已输出到 JSON，但当前 Canvas 还没有绘制搜索扩展点；
- 地图 JSON 解析是为当前格式写的轻量解析器，不是完整通用 JSON 解析库；
- 参数还没有整理成独立配置文件；
- 对复杂窄通道或严格泊车场景，当前版本可能搜索慢或失败。

这些限制是刻意保留的：当前阶段优先保证“地图输入 -> C++ 规划 -> JSON 输出 -> HTML 动画”的主链路稳定。

## 后续计划

建议下一步按顺序推进：

1. 在 Canvas 中显示 `expanded` 搜索节点；
2. 增加车辆矩形 footprint 碰撞检测；
3. 将 Hybrid A* 参数移入配置文件；
4. 优化代价函数，增加换挡惩罚和方向连续性惩罚；
5. 增加路径平滑；
6. 增加普通 A* 对比路径，用于课堂解释 Hybrid A* 的区别。

更多设计细节见：

```text
design.md
output/design.md
```
