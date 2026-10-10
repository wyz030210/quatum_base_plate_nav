# 室内 AMCL 导航问题记录

## 1. 文档目的

本文记录 `new_base_plate_indoor_ws` 室内导航测试中已经确认的问题、诊断证据和待验证项。本文只描述问题，不代表对应修改已经实施。

工作空间：

```text
/home/car/ros2/new_base_plate_indoor_ws
```

使用地图：

```text
/home/car/ros2/catographer_ws/map/003.yaml
/home/car/ros2/catographer_ws/map/003.pgm
```

主要日志：

```text
/home/car/.ros/log/component_container_isolated_25062_1789630009102.log
/home/car/.ros/log/cmd_vel_to_can_24950_1789629998538.log
```

## 2. 当前系统状态

测试时以下 Nav2 生命周期节点均处于 `active` 状态：

- `map_server`
- `amcl`
- `controller_server`
- `planner_server`
- `behavior_server`
- `bt_navigator`
- `waypoint_follower`
- `velocity_smoother`

传感器和里程计话题也在持续发布：

- `/livox/lidar`：约 10 Hz
- `/scan`：约 9～10 Hz
- `/Odometry`：约 10 Hz
- `/odometry/filtered_odom`：约 10 Hz

因此，当前问题不是 Nav2 或传感器节点整体没有启动。

## 3. 已确认的问题

### 3.1 全局规划起点被实时代价地图判定为碰撞区域

测试记录中共有 5 个导航目标，其中 2 个成功，3 个失败。失败目标的核心日志包括：

```text
GridBased: failed to create plan, no valid path found.
GridBased: failed to create plan, invalid use: Starting point in lethal space!
Planning algorithm GridBased failed to generate a valid path
```

恢复行为还报告：

```text
Collision Ahead - Exiting DriveOnHeading
backup failed
```

这说明失败主要发生在全局规划阶段。规划器认为机器人起点或其附近不可通行，恢复倒车也检测到了碰撞风险。

### 3.2 静态地图与实时障碍层不一致

对 `003.pgm` 进行像素检查后，失败目标附近以及若干 AMCL 初始位姿附近的机器人 footprint 大部分处于静态地图空闲区域。

对当前机器人位置进行交叉检查时：

- 静态地图 footprint 内没有占用栅格；
- 实时全局代价地图中，同一 footprint 内存在 93 个值为 253 的高代价栅格；
- 机器人中心栅格代价值约为 216。

因此，`Starting point in lethal space` 不是单纯由 `003.pgm` 的静态墙体导致，而主要来自 `/scan` 注入的实时障碍层或定位偏差造成的地图错位。

### 3.3 当前激光与静态地图重合度较低

将当前 `/scan` 端点按照 `map -> livox_frame` 变换投影到静态地图后，得到：

- 约 10.7% 的激光端点位于地图墙体 0.10 m 以内；
- 约 33.4% 位于地图墙体 0.20 m 以内；
- 激光端点到最近地图墙体的中位距离约为 0.56 m。

该重合度明显偏低，表明以下因素至少有一项存在问题：

1. AMCL 初始位姿不准确；
2. AMCL 尚未稳定收敛；
3. `base_link/base_footprint -> livox_frame` 外参不准确；
4. 3D Livox 点云投影为 2D LaserScan 时包含不适合定位的点；
5. 建图传感器与当前定位传感器的安装位置、朝向或观测特征差异较大；
6. 现场环境相对建图时发生了较大变化。

### 3.4 测试过程中多次重新设置 AMCL 初始位姿

日志中记录了多次：

```text
initialPoseReceived
Setting pose (...)
```

第一次导航成功后，仍多次通过 RViz 重新设置 AMCL 位姿。如果车辆没有被人工搬回所设置的位置，这会造成 `map -> odom` 突然跳变，使后续规划从错误位置开始。

正确操作原则是：

- 启动后设置一次准确初始位姿；
- 等待激光与地图基本重合；
- 正常导航过程中不要重复设置初始位姿；
- 只有车辆被搬动或 AMCL 明显丢失时才重新定位。

### 3.5 AMCL 初始位姿存在时间外推警告

每次设置初始位姿附近都出现：

```text
Failed to transform initial pose in time
Lookup would require extrapolation into the future
```

日志显示初始位姿时间戳通常比最新 `odom -> base_link` 变换领先约 40～90 ms。AMCL 随后仍执行了 `Setting pose`，所以这不是导航失败的直接原因，但说明里程计/TF 存在一定发布延迟，应继续监控。

### 3.6 规划器运动模型与车辆倒车能力不一致

当前 Smac Hybrid 配置为：

```yaml
motion_model_for_search: "DUBIN"
minimum_turning_radius: 0.5
```

`DUBIN` 只规划前进路径，但系统同时允许：

```yaml
FollowPath.vx_min: -0.10
```

并且行为树配置了 `BackUp`。在狭窄走廊或需要多次调整方向的位置，前进限定会使规划器更容易报告 `no valid path found`。

### 3.7 Nav2 角速度与底盘舵机命令的含义不一致

`cmd_vel_to_can_node.cpp` 当前将 `Twist.angular.z` 直接按比例映射到舵机行程。Nav2 中 `angular.z` 的含义是车辆期望偏航角速度，Ackermann 底盘应先按车辆运动学换算前轮转角：

```text
steering_angle = atan(wheel_base * angular_velocity / linear_velocity)
```

当前实现没有使用该关系，且 `wheel_base` 没有参与舵角换算。结果是：

- 规划器/MPPI 假设的最小转弯半径可能与车辆实际转弯半径不同；
- 低速转弯时误差尤其明显；
- 直线路径可能成功，但较急弯或狭窄通道的跟踪效果会变差。

### 3.8 车辆 footprint 和安全距离可能偏保守

当前 footprint 为：

```yaml
footprint: "[[-0.44, -0.34], [-0.44, 0.34], [0.44, 0.34], [0.44, -0.34]]"
```

即按 0.88 m × 0.68 m 的矩形处理。除此之外：

- 全局膨胀半径为 0.8 m；
- 局部膨胀半径为 0.6 m；
- MPPI `collision_margin_distance` 为 0.25 m。

如果这些数值大于车辆真实包络和所需安全距离，狭窄走廊会被进一步压缩。但在定位和激光外参确认之前，不应直接缩小 footprint 来掩盖地图错位。

### 3.9 速度上限存在大量重复警告

底盘日志反复出现：

```text
线速度超出范围[-0.10, 0.25]m/s，已限制为0.25m/s
```

这可能由浮点误差或速度平滑器输出略高于 0.25 m/s 引起。当前未发现其直接导致导航失败，但会污染日志，并可能掩盖其他重要警告。

## 4. 尚未确认的项目

以下数据需要现场标定后才能确定，不能仅凭当前日志下结论：

- 车辆实际长、宽及安全外扩量；
- 最大前轮转角和左右转向是否对称；
- 舵机命令角度与实际前轮角度之间的标定曲线；
- 车辆真实最小转弯半径；
- Livox 相对 `base_link` 的实际 XYZ 和 RPY 外参；
- `003` 地图是否由同一台 Livox、同一安装姿态建立；
- 失败时车身附近是否存在人员、墙体或临时障碍物；
- 车辆静止时 FAST-LIO、EKF 和 AMCL 的长期漂移量。

## 5. 问题优先级

| 优先级 | 问题 | 影响 |
|---|---|---|
| P0 | 激光与地图明显不重合 | 直接造成错误定位和起点碰撞 |
| P0 | 多次错误重设 AMCL 初始位姿 | 直接导致 `map -> odom` 跳变 |
| P1 | Ackermann 舵角换算不正确 | 实际轨迹无法满足规划运动学 |
| P1 | `DUBIN` 不允许倒车规划 | 狭窄区域可能无可行路径 |
| P1 | footprint/膨胀/碰撞余量待标定 | 通道可能被配置参数封死 |
| P2 | 初始位姿时间外推警告 | 增加初始化误差风险 |
| P2 | 速度限幅警告刷屏 | 影响日志可读性 |

## 6. 当前结论

本次导航失败的直接原因是全局规划器认为起点或周边处于高代价/碰撞区域。最优先需要解决的是 AMCL 位姿、激光外参和实时扫描与静态地图不重合的问题。运动模型和舵角换算虽然不是这三次规划失败的唯一原因，但会在定位恢复正常后继续限制转弯和路径跟踪效果。
