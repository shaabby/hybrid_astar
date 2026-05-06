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

每次运行还会追加一行简要实验日志：

```text
output/experiments.csv
```

日志包含地图路径、是否成功、路径点数量、扩展节点数量、规划耗时、起终点位姿、主要代价参数和启发式名称，方便对比不同参数或地图下的规划效果。

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

## 待改进点

这个版本已经打通“地图输入 -> C++ 规划 -> JSON 输出 -> HTML 动画”的主链路。后续改进建议按下面优先级推进。

### P0 展示前优先处理

当前 P0 项已经完成：

- 文档与代码状态同步；
- 默认地图路径统一为 `map/default_map.json`；
- HTML 动画标题已从早期直线运动 demo 改为 Hybrid A* 路径规划 demo。

### P1 影响规划质量

1. 增强启发式（已完成第一版）  
   默认启发式已经改为组合启发式：`h = max(h_non_obs, h_obs)`。其中 `h_obs` 是膨胀障碍图上的 8 邻域 Dijkstra cost-to-go，`h_non_obs` 优先使用 Reeds-Shepp 风格无障碍距离，失败时回退到欧几里得距离。
2. 补齐更完整的 Reeds-Shepp 候选  
   当前 analytic expansion 是简化 Reeds-Shepp 风格直连，本质仍偏 Dubins + 反向候选，没有完整覆盖带 cusp 的 Reeds-Shepp words。泊车、窄通道和需要多次倒车的场景会受影响。
3. 优化代价函数（已完成第一版）  
   `g cost` 已包含实际轨迹段长度、倒车惩罚、转向幅度惩罚、换挡惩罚和转向变化惩罚。靠障碍惩罚暂不加入，避免和障碍物启发式重复耦合。

### P2 提升课堂展示效果

1. 在 Canvas 中显示 `expanded` 搜索节点  
   `expanded` 已输出到 JSON，但当前页面还没有绘制搜索扩展点。展示扩展过程能更直观解释 Hybrid A* 与普通 A* 的区别。
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
