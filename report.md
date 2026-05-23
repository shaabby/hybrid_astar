# Hybrid A* 路径规划系统设计与实现报告

## 摘要

本项目实现了一个面向车辆路径规划教学展示的 Hybrid A* 路径规划程序。程序以栅格地图、起点、终点和车辆参数为输入，使用带车辆运动学约束的 Hybrid A* 算法搜索从起点到终点的连续可行路径，并将规划结果导出为 JSON 和 HTML Canvas 动画页面。项目使用 C++23 编写核心规划逻辑，使用 FLTK 提供本地窗口展示，并额外生成可直接用浏览器打开的单文件 HTML 页面，便于课堂演示和结果复现。

---

## 1. 问题分析

### 1.1 问题来源

路径规划是机器人、自动驾驶、智能仓储和无人系统中的基础问题。传统栅格 A* 算法通常把机器人看作可以在栅格之间自由移动的点，这种模型适合二维平面上的点机器人或移动方向限制较少的机器人。但是汽车类车辆具有明显的非完整约束：车辆不能横向平移，不能原地旋转，转弯半径有限，并且前进和倒车的代价也可能不同。因此，如果直接使用普通 A* 规划车辆路径，得到的路径往往不能被真实车辆执行。

本项目解决的问题是：在带障碍物的二维地图中，为一个具有长度、宽度、轴距和最大转向角约束的车辆，规划一条从指定起点位姿到指定终点位姿的无碰撞路径。这里的“位姿”不仅包括位置 \((x, y)\)，还包括车辆朝向 \(\theta\)。

### 1.2 问题意义

车辆路径规划需要同时考虑可达性、安全性和运动学可行性。相比普通 A*，Hybrid A* 能在连续状态空间中保留车辆的真实位姿，并在搜索时使用车辆运动模型生成后继节点，因此规划出的轨迹更接近真实车辆可以执行的轨迹。该方法常用于自动泊车、低速无人车导航、仓库搬运车路径规划等场景。

本项目虽然是课程大作业规模的演示系统，但实现了一个完整闭环：地图编辑、配置读取、路径规划、碰撞检测、实验记录和可视化展示。通过该系统可以直观理解“点机器人路径规划”和“车辆运动学路径规划”的差异。

### 1.3 解决方法

项目采用 Hybrid A* 作为核心搜索算法。其基本思想是：

1. 搜索状态使用连续车辆位姿 \((x, y, \theta)\)，而不是只使用栅格坐标。
2. 每次节点扩展不直接移动到相邻栅格，而是使用简化自行车模型模拟车辆沿一段运动基元前进或倒车。
3. 使用离散化的 \((x, y, \theta)\) 索引维护 closed set，避免无限重复搜索。
4. 使用代价函数评价路径质量，代价包括路径长度、倒车惩罚、转向惩罚、换挡惩罚和转向变化惩罚。
5. 使用启发函数引导搜索方向。当前程序支持欧几里得启发式和组合启发式，组合启发式将无障碍 Reeds-Shepp 风格距离与考虑障碍物的启发式结合。
6. 接近目标时尝试 Reeds-Shepp 解析扩展，以加快收敛并改善终点连接质量。

车辆运动模型采用简化 bicycle model。车辆状态点定义为后轴中心，前轮转角决定曲率：

\[
\kappa = \frac{\tan(\delta)}{L}
\]

其中 \(\delta\) 为前轮转向角，\(L\) 为轴距。每个小步的状态更新为：

\[
x_{next} = x + d \cdot s \cdot \cos\theta
\]

\[
y_{next} = y + d \cdot s \cdot \sin\theta
\]

\[
\theta_{next} = \theta + d \cdot s \cdot \kappa
\]

其中 \(d = 1\) 表示前进，\(d = -1\) 表示倒车，\(s\) 表示积分步长。

---

## 2. 设计方案

### 2.1 总体路线

本项目的总体路线是构建一个从地图输入到路径输出再到动画展示的最小闭环：

```text
Web 地图编辑器
    ↓ 导出 JSON 地图
C++ 读取地图与 YAML 配置
    ↓ 构造 GridMap / Car / HybridAstar
Hybrid A* 搜索车辆可行路径
    ↓ 碰撞检测与目标判断
输出 result.json / demo.html / experiments.csv
    ↓
浏览器或 FLTK 窗口展示规划结果
```

项目的输入主要包括：

- 地图 JSON 文件：描述地图尺寸、障碍物、起点和终点。
- YAML 配置文件：描述地图路径、车辆参数和 Hybrid A* 参数。

项目的输出主要包括：

- `output/result.json`：包含地图、车辆、路径和扩展节点数据。
- `output/demo.html`：可直接打开的 Canvas 动画页面。
- `output/experiments.csv`：记录每次实验的成功状态、路径点数、扩展节点数和运行时间等指标。

### 2.2 程序结构

项目主要目录如下：

```text
final/
├── CMakeLists.txt
├── config/default.yaml
├── include/
├── src/
├── map/
├── output/
├── tests/
├── tool/
└── third_party/
```

其中：

- `include/` 存放头文件和主要类定义。
- `src/` 存放 C++ 实现文件。
- `map/` 存放地图编辑器和地图 JSON 文件。
- `config/` 存放程序运行配置。
- `tests/` 存放自动测试程序。
- `third_party/` 存放第三方依赖，包括 FLTK 和 Reeds-Shepp 相关代码。
- `output/` 存放程序运行后生成的可视化和实验结果。

### 2.3 核心类与函数

#### 2.3.1 `GridMap` 与 `MapLoader`

`GridMap` 是二维栅格地图类，负责保存地图宽度、高度、障碍物、起点和终点。其内部使用一维 `vector` 存储二维栅格，0 表示自由空间，1 表示障碍物。

主要功能包括：

- `width()` / `height()`：返回地图尺寸。
- `inBounds(x, y)`：判断栅格坐标是否在地图内。
- `isObstacle(x, y)`：判断指定栅格是否为障碍物。
- `isFree(x, y)`：判断指定栅格是否为空闲。
- `setObstacle(x, y, occupied)`：设置或清除障碍物。
- `setStart(pose)` / `setGoal(pose)`：设置起点和终点位姿。
- `start()` / `goal()`：读取起点和终点。

`MapLoader` 负责从 JSON 文件读取地图数据并构造 `GridMap`。地图格式支持点障碍物，也兼容矩形障碍物。

#### 2.3.2 `Car`、`VehicleConfig` 与 `CarPose`

`VehicleConfig` 保存车辆物理参数：

- `length`：车身长度。
- `width`：车身宽度。
- `wheelbase`：轴距。
- `rear_to_center`：后轴中心到车身几何中心的距离。
- `max_steer`：最大前轮转角。

`CarPose` 表示车辆位姿，包括后轴中心坐标、航向角、前轮转角和行驶方向。

`Car` 是车辆运动学模型类，主要功能包括：

- `minTurningRadius()`：根据轴距和最大转向角计算最小转弯半径。
- `clampSteer(steer)`：限制转向角不超过最大转角。
- `step(pose, steer, direction, distance)`：根据 bicycle model 推进车辆状态。
- `bodyCorners(pose)`：计算车辆矩形外轮廓四个角点，用于碰撞检测和可视化。

#### 2.3.3 `VehicleCollisionChecker` 与 `ReedsSheppCollisionChecker`

碰撞检测模块负责判断车辆在某个位姿或某段路径上是否与障碍物相交。

`VehicleCollisionChecker` 使用车辆矩形 footprint 进行检测。它先根据车辆位姿计算车身矩形角点，再检查矩形覆盖范围内的栅格是否与障碍物相交。主要功能包括：

- `collides(pose)`：判断单个位姿是否碰撞。
- `isCollisionFree(samples)`：判断一系列采样位姿是否全部无碰撞。
- `firstCollisionIndex(samples)`：返回路径中第一次发生碰撞的采样点下标。

`ReedsSheppCollisionChecker` 用于检测 Reeds-Shepp 采样路径是否无碰撞。它本质上复用车辆 footprint 检测器，对路径中的每个采样位姿逐一检查。

#### 2.3.4 `HybridAstar` 与 `HybridAstarConfig`

`HybridAstarConfig` 保存规划器参数，包括：

- `xy_resolution`：位置离散分辨率。
- `theta_bins`：航向角离散分箱数。
- `step_size`：运动学积分步长。
- `primitive_length`：单个运动基元长度。
- `goal_xy_tolerance`：目标位置容差。
- `goal_theta_tolerance`：目标航向角容差。
- `reverse_penalty`：倒车惩罚。
- `steer_penalty`：转向惩罚。
- `gear_switch_penalty`：换挡惩罚。
- `steer_change_penalty`：转向变化惩罚。
- `max_iterations`：最大搜索迭代次数。
- `allow_reverse`：是否允许倒车。
- `enable_analytic_expansion`：是否启用解析扩展。
- `analytic_expansion_distance`：距离目标多近时尝试解析扩展。
- `analytic_expansion_interval`：每隔多少次扩展尝试一次解析扩展。
- `collision_safety_margin`：碰撞检测安全外扩距离。
- `enable_obstacle_heuristic`：是否启用障碍物启发式。
- `obstacle_heuristic_inflate_alpha`：障碍物启发式膨胀系数。
- `debug` 与 `debug_progress_interval`：调试输出相关参数。

`HybridAstar` 是核心规划器。它在 \((x, y, \theta)\) 三维状态空间中搜索路径。与普通 A* 不同，它的节点扩展由车辆运动模型产生，每个子节点对应一段连续的车辆轨迹。

规划结果由 `PlanResult` 表示，包括：

- `success`：是否规划成功。
- `path`：最终路径点序列。
- `expanded`：搜索过程中扩展过的节点。
- `iterations`：搜索迭代次数。
- `generated_nodes`：生成节点数量。
- `open_remaining`：结束时 open set 中剩余节点数量。

#### 2.3.5 `Heuristic`、`EuclideanHeuristic` 与 `CombinedHeuristic`

`Heuristic` 是启发函数接口，主要包含：

- `prepare(map, car, config)`：规划前预处理地图、车辆和配置。
- `estimate(pose)`：估计当前位姿到目标的剩余代价。
- `name()`：返回启发式名称。

`EuclideanHeuristic` 使用当前位置到目标位置的欧几里得距离作为估计值。它实现简单、计算快，但不考虑障碍物和车辆运动约束。

`CombinedHeuristic` 使用组合启发式：

```text
h = max(h_non_obs, h_obs)
```

其中：

- `h_non_obs` 表示不考虑障碍物时的车辆可行距离估计，优先使用 Reeds-Shepp 风格距离，失败时回退到欧几里得距离。
- `h_obs` 表示考虑障碍物后的代价估计，通过障碍物相关的 lookup 或可视图距离指导搜索绕开障碍物。

这种组合方式兼顾了车辆运动学约束和障碍物环境信息，比单纯欧几里得距离更适合复杂地图。

#### 2.3.6 `ReedsSheppGenerator`

`ReedsSheppGenerator` 负责生成 Reeds-Shepp 路径。Reeds-Shepp 曲线描述允许前进和倒车的车辆在无障碍平面中连接两个位姿的短路径形式。

主要功能包括：

- `generate(start, goal)`：生成从起点到目标位姿的路径。
- `estimateDistance(start, goal)`：估计起点到目标的 Reeds-Shepp 距离。

在本项目中，Reeds-Shepp 模块主要用于两个方面：

1. 在启发函数中估计无障碍车辆距离。
2. 在 Hybrid A* 接近目标时尝试解析扩展，直接连接当前位姿和终点。

#### 2.3.7 `AppConfigLoader`

`AppConfigLoader` 从 YAML 配置文件读取程序运行参数，并生成 `AppConfig`。`AppConfig` 聚合了地图路径、车辆配置和 Hybrid A* 配置，使程序不需要把参数硬编码在源代码中。

#### 2.3.8 `JsonExporter` 与 `HtmlWriter`

`JsonExporter` 负责将地图、车辆参数、路径点和扩展节点导出为 JSON 字符串。该 JSON 文件既可用于调试，也可被浏览器端可视化读取。

`HtmlWriter` 负责把 JSON 数据嵌入 HTML 模板，生成完整的单文件 HTML 页面。这样最终结果不依赖本地服务器，直接双击 `output/demo.html` 即可查看动画。

#### 2.3.9 `FltkCanvas` 与 `FltkViewer`

FLTK 可视化模块用于在本地窗口中展示地图、障碍物、起点、终点和车辆路径。它与 HTML 输出形成互补：FLTK 适合运行程序时即时查看，HTML 更适合提交、演示和跨平台打开。

#### 2.3.10 `ExperimentLogger`

`ExperimentLogger` 将每次规划实验的关键指标追加写入 CSV 文件，包括地图路径、是否成功、路径点数、扩展节点数量、运行时间和启发式名称等。通过该模块可以比较不同参数、不同地图和不同启发式设置对规划结果的影响。

### 2.4 类之间的关系

各模块关系如下：

```text
AppConfigLoader
    ├── 读取 map_path
    ├── 生成 VehicleConfig
    └── 生成 HybridAstarConfig

MapLoader ──> GridMap
VehicleConfig ──> Car
GridMap + Car + HybridAstarConfig ──> HybridAstar
HybridAstar ──> Heuristic
HybridAstar ──> VehicleCollisionChecker
HybridAstar ──> ReedsSheppGenerator
HybridAstar ──> PlanResult
PlanResult + GridMap + Car ──> JsonExporter ──> result.json
result.json ──> HtmlWriter ──> demo.html
PlanResult + Config ──> ExperimentLogger ──> experiments.csv
```

这种设计将地图、车辆模型、搜索算法、碰撞检测、启发函数和可视化输出分离开来，使各模块职责清晰，便于调试和替换。

---

## 3. 创新性

本项目的创新性主要体现在以下几个方面。

### 3.1 从普通路径规划扩展到车辆运动学路径规划

课程中常见的路径规划示例多以点机器人或简单栅格搜索为主。本项目没有停留在普通 A* 的二维网格移动，而是引入了车辆位姿 \((x, y, \theta)\) 和自行车运动学模型，使规划路径满足车辆不能横移、不能原地旋转、转弯半径有限等约束。

### 3.2 完整实现“输入—规划—输出—可视化”闭环

项目不仅实现了算法本身，还实现了地图编辑、JSON 地图读取、YAML 参数配置、路径导出、HTML 动画展示和 CSV 实验记录。这样的闭环设计使程序不仅能“算出路径”，还能够方便地展示、复现和比较实验结果。

### 3.3 组合启发式提高搜索指导性

程序提供了组合启发式：

```text
h = max(h_non_obs, h_obs)
```

其中一部分考虑车辆无障碍距离，另一部分考虑障碍物环境信息。这比单纯欧几里得距离更贴近车辆路径规划问题，可以减少盲目扩展，提高搜索质量。

### 3.4 解析扩展与搜索结合

在接近目标时，程序尝试使用 Reeds-Shepp 风格路径直接连接目标。这种方法把离散搜索和连续解析曲线结合起来，可以减少末端搜索困难，使车辆更容易满足终点位姿约束。

### 3.5 课堂展示友好的工程设计

项目输出单文件 HTML 动画页面，不依赖 Python、本地 HTTP 服务器、现场网络或复杂图形库部署。只要有浏览器，就可以查看规划结果。这一点提高了课程展示的可靠性，也方便将结果提交给老师检查。

---

## 4. 运行方法和参数设置

### 4.1 构建方法

推荐使用项目脚本：

```bash
./run.sh
```

该脚本会自动配置 CMake、构建程序并运行默认配置。

也可以手动构建：

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release
```

### 4.2 运行默认配置

默认配置文件为：

```text
config/default.yaml
```

运行命令：

```bash
./build/hybrid_astar config/default.yaml
```

如果当前环境没有图形显示，例如 SSH、容器或 CI 环境，可以使用：

```bash
./build/hybrid_astar --no-view config/default.yaml
```

或：

```bash
./build/hybrid_astar --html-only config/default.yaml
```

运行后生成：

```text
output/result.json
output/demo.html
output/experiments.csv
```

其中 `output/demo.html` 可以直接用浏览器打开查看动画。

### 4.3 指定其他配置

可以复制 `config/default.yaml`，修改后作为新的配置文件，例如：

```bash
./build/hybrid_astar config/my_test.yaml
```

### 4.4 主要参数说明

配置文件主要分为三部分：地图路径、车辆参数和规划器参数。

#### 4.4.1 地图路径

```yaml
map_path: map/default_map.json
```

该参数指定程序读取哪个地图文件。更换地图会改变障碍物分布、起点和终点，从而影响路径是否可达、路径长度和搜索耗时。

#### 4.4.2 车辆参数

```yaml
vehicle:
  length: 4.5
  width: 2.0
  wheelbase: 2.7
  rear_to_center: 1.35
  max_steer: 0.61
```

- `length` 和 `width` 越大，车辆占用空间越大，碰撞检测越严格，窄通道更难通过。
- `wheelbase` 越大，车辆最小转弯半径通常越大，转弯能力下降。
- `max_steer` 越大，车辆可以更急地转弯；越小则路径更平缓，但更难在狭窄环境中到达目标。
- `rear_to_center` 主要影响车身矩形相对后轴中心的位置，用于碰撞检测和可视化。

#### 4.4.3 Hybrid A* 参数

```yaml
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
  analytic_expansion_distance: 8
  analytic_expansion_interval: 25
  collision_safety_margin: 0.0
  enable_obstacle_heuristic: true
  obstacle_heuristic_inflate_alpha: 1.0
  debug: true
  debug_progress_interval: 500
```

主要影响如下：

- `xy_resolution`：越小，位置去重越精细，可能得到更精确路径，但搜索节点更多、速度更慢。
- `theta_bins`：越大，航向角离散越细，终点朝向更容易精确匹配，但状态空间增大。
- `step_size`：越小，运动学积分越细，碰撞检测更准确，但计算量更大。
- `primitive_length`：越大，每次扩展走得更远，搜索速度可能变快，但路径细节可能变差。
- `goal_xy_tolerance` 和 `goal_theta_tolerance`：容差越大，越容易判定到达目标，但终点精度降低。
- `reverse_penalty`：越大，规划器越不倾向倒车；越小，则更容易生成倒车路径。
- `steer_penalty`：越大，规划器越偏好直行或小转向。
- `gear_switch_penalty`：越大，规划器越避免频繁前进/倒车切换。
- `steer_change_penalty`：越大，路径转向变化更平滑，但可能增加搜索难度。
- `max_iterations`：限制最大搜索次数，过小可能导致复杂地图规划失败。
- `allow_reverse`：关闭后车辆只能前进，适合测试普通前向规划；开启后更适合泊车和窄空间调头。
- `enable_analytic_expansion`：开启后接近目标时尝试 Reeds-Shepp 直连，通常可以加快成功收敛。
- `collision_safety_margin`：增大后车辆外轮廓会额外膨胀，路径更保守、更安全，但可通行空间变小。
- `enable_obstacle_heuristic`：开启后启发式考虑障碍物信息，通常能减少无效搜索。
- `debug`：开启后输出搜索进度，便于调试；关闭后运行输出更简洁。

---

## 5. 测试

### 5.1 测试方法

项目使用 CTest 管理测试程序。构建后可以运行：

```bash
ctest --test-dir build --output-on-failure
```

项目中包含以下主要测试：

```text
reeds_shepp_empty_map_test
line_of_sight_test
app_config_test
```

此外，也通过运行主程序并检查 `output/result.json`、`output/demo.html` 和 `output/experiments.csv` 来进行集成测试。

### 5.2 Reeds-Shepp 空地图测试

该测试使用 `map/empty.json`，验证 Reeds-Shepp 生成器和碰撞检测模块在空旷地图中的基本正确性。

测试内容包括：

1. 地图可以正确加载。
2. 地图尺寸符合预期。
3. 起点、终点和内部测试点不发生碰撞。
4. 直线路径可以生成，并且能到达目标。
5. 从地图起点到终点的路径可以生成，并且路径长度不小于欧几里得距离下界。
6. 倒车覆盖测试中生成的路径包含倒车段。
7. 路径采样点均为有限数值，不出现 NaN 或无穷大。
8. 生成路径通过碰撞检测。
9. 非法参数，例如零转弯半径或零采样步长，会导致路径生成失败，而不是产生错误结果。
10. 测试结果可以写出到 `output/reeds_shepp_empty_map_test.json`。

该测试说明 Reeds-Shepp 模块能够在基本场景中生成可用路径，也能正确处理部分异常参数。

### 5.3 Line of Sight 测试

`line_of_sight_test` 用于测试可视线或线段穿越障碍物判断逻辑。该功能与障碍物启发式、可视图构造等模块相关。测试可以验证：

- 两点之间没有障碍物时应判定可见。
- 线段经过障碍物时应判定不可见。
- 边界和斜线场景能被正确处理。

这类测试有助于避免启发式中错误连接穿越障碍物的路径。

### 5.4 AppConfig 配置测试

`app_config_test` 用于测试 YAML 配置加载模块。测试数据包括：

- 合法配置文件。
- 缺少地图路径的配置。
- 缺少车辆参数的配置。
- 缺少规划器参数的配置。
- 参数类型错误的配置。
- 未知字段配置。

通过这些测试可以确认程序在读取配置时能够正确解析合法配置，并对明显错误给出失败结果，从而避免运行时使用无效参数。

### 5.5 主程序集成测试

主程序集成测试的方式是运行：

```bash
./build/hybrid_astar --no-view config/default.yaml
```

然后检查：

1. 程序是否正常退出。
2. 是否生成 `output/result.json`。
3. 是否生成 `output/demo.html`。
4. 是否追加 `output/experiments.csv`。
5. `result.json` 中是否包含地图、车辆、路径和扩展节点字段。
6. `demo.html` 是否能在浏览器中显示地图、小车和轨迹动画。

### 5.6 测试数据生成

测试数据主要有三类来源：

1. 手工编写的地图 JSON，例如 `map/default_map.json`、`map/empty.json`、`map/parking_map.json` 等。
2. 使用 `map/grid_demo.html` 在浏览器中交互式编辑地图，然后导出 JSON。
3. 测试程序中直接构造起点、终点和车辆参数，例如 Reeds-Shepp 测试中的直线路径和倒车路径。

这种方式既覆盖了人工可控的简单场景，也覆盖了接近真实使用流程的地图输入场景。

### 5.7 发现的问题与修改

在开发和测试过程中，主要发现并处理了以下问题：

1. 普通欧几里得启发式在障碍物较多时指导性不足，容易扩展大量无效节点。后来加入组合启发式，使启发函数同时考虑车辆无障碍距离和障碍物环境。
2. 如果没有解析扩展，接近目标时可能需要较多迭代才能满足位置和角度容差。后来加入 Reeds-Shepp 风格的目标直连尝试，提高了末端连接能力。
3. 图形显示环境不可用时，FLTK 窗口可能无法打开。项目增加了 `--no-view` 和 `--html-only` 模式，使程序在无桌面环境下仍可生成结果文件。
4. 不同参数对搜索速度和结果影响较大，因此增加了 YAML 配置和 CSV 实验记录，便于多次运行后比较结果。
5. 车辆碰撞检测不能只检查后轴中心所在栅格，而必须检查车身矩形 footprint。项目通过 `bodyCorners` 和矩形覆盖栅格检测提高了碰撞检测真实性。

### 5.8 实验日志：高角度分辨率导致原地转圈

本次实验使用 `map/default_map.json` 作为地图输入，并采用较高的航向角去重精度。实验配置如下：

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
  max_iterations: 1200000
  allow_reverse: true
  enable_analytic_expansion: true
  analytic_expansion_distance: 8
  analytic_expansion_interval: 25
  collision_safety_margin: 0.0
  enable_obstacle_heuristic: true
  obstacle_heuristic_inflate_alpha: 1.0
  obstacle_lookup_resolution: 0.1
  debug: true
  debug_progress_interval: 500
```

实验现象是：当 `theta_bins` 设置为 `360` 时，航向角离散粒度非常细。搜索去重时，同一位置附近只要朝向角落入不同角度分箱，就会被视为不同状态继续扩展。这会显著放大状态空间，并且在狭窄或启发式约束不够强的区域中，车辆容易反复尝试小角度调整，表现为局部转圈或原地打转。

关键结论是：本次转圈现象的主要原因不是车辆运动模型错误，而是 `theta_bins: 360` 使闭集去重过细，导致大量近似重复的姿态状态没有被合并。后续调参时可以适当降低 `theta_bins`，或增大 `xy_resolution`、`primitive_length`、转向变化惩罚等参数，以减少局部重复搜索。
![alt text](image.png)

---

## 6. 学习心得和收获

通过完成本次大作业，我对 C++ 程序设计、路径规划算法和工程化开发都有了更深入的理解。

首先，我认识到算法实现不能只停留在数学公式层面。Hybrid A* 的基本思想并不复杂，但真正写成可运行程序时，需要处理状态离散化、open list、closed set、路径回溯、碰撞检测、代价函数、启发函数、目标容差和参数调试等许多细节。任何一个细节处理不当，都可能导致搜索失败或路径质量下降。

其次，我加深了对车辆运动学模型的理解。普通 A* 中的移动只是上下左右或八邻域扩展，而车辆路径规划必须考虑车辆朝向、转弯半径和倒车能力。通过实现 bicycle model，我理解了为什么车辆不能简单地被当作点来规划，也理解了后轴中心、前轮转角、最小转弯半径等概念在路径规划中的作用。

第三，我体会到可视化对调试算法非常重要。仅仅从命令行输出“规划成功”或“规划失败”，很难判断问题出在哪里。通过导出 JSON 和 HTML 动画，可以直观看到路径是否绕开障碍物、车辆是否转向合理、终点姿态是否正确。这种可视化反馈大大提高了调试效率。

第四，我学习到了模块化设计的重要性。本项目把地图、车辆、规划器、启发式、碰撞检测、配置读取、结果导出和实验记录拆分为不同模块。这样做虽然前期需要更多设计，但后续调试和扩展更方便。例如，如果要替换启发式或增加新的地图，只需要修改对应模块，而不必重写整个程序。

第五，我认识到参数配置和测试对工程程序非常重要。路径规划算法往往对分辨率、步长、容差和代价权重敏感。如果参数写死在代码中，调试会非常麻烦。通过 YAML 配置文件和 CSV 日志，可以更系统地比较不同参数的效果。自动测试也能及时发现配置解析、碰撞检测和 Reeds-Shepp 路径生成中的错误。

最后，本次项目让我对 C++23 的工程开发流程更加熟悉，包括 CMake 构建、头文件和源文件组织、类设计、标准库容器使用、文件读写、异常处理和测试程序编写。相比只写单个 `main.cpp` 的课程练习，这个项目更接近一个小型软件系统，也让我意识到良好的结构和文档对于后续维护非常重要。

对课程的建议是：如果后续课程中能增加一些与可视化、测试和工程组织相关的小练习，会更有助于学生从“会写语法”过渡到“能完成一个完整程序”。

---

## 7. 参考文献

[1] Hart, P. E., Nilsson, N. J., & Raphael, B. (1968). A Formal Basis for the Heuristic Determination of Minimum Cost Paths. *IEEE Transactions on Systems Science and Cybernetics*, 4(2), 100-107.

[2] Dolgov, D., Thrun, S., Montemerlo, M., & Diebel, J. (2010). Path Planning for Autonomous Vehicles in Unknown Semi-structured Environments. *The International Journal of Robotics Research*, 29(5), 485-501.

[3] Reeds, J. A., & Shepp, L. A. (1990). Optimal Paths for a Car That Goes Both Forwards and Backwards. *Pacific Journal of Mathematics*, 145(2), 367-393.

[4] LaValle, S. M. (2006). *Planning Algorithms*. Cambridge University Press.

[5] Thrun, S., Burgard, W., & Fox, D. (2005). *Probabilistic Robotics*. MIT Press.

[6] Siegwart, R., Nourbakhsh, I. R., & Scaramuzza, D. (2011). *Introduction to Autonomous Mobile Robots* (2nd ed.). MIT Press.
