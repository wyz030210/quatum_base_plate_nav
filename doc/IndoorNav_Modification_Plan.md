# 室内 AMCL 导航修改方案(issues之后)

## 1. 目标与原则

目标是让室内导航具备以下能力：

1. AMCL 能稳定建立并保持 `map -> odom`；
2. `/scan` 与静态地图墙体正确重合；
3. 机器人起点不会被实时代价地图误判为碰撞；
4. 全局规划器可为 Ackermann 车辆生成包含必要倒车段的可行路径；
5. Nav2 输出能够正确转换为底盘前轮转角；
6. 参数调整有明确测试依据，不通过缩小车身模型掩盖定位错误。

所有修改应先在低速、空旷且有急停条件的环境中验证。

## 2. 修改顺序

必须按以下顺序执行：

1. 建立可重复的测试和日志；
2. 校准 TF 与激光投影；
3. 验证 AMCL；
4. 修正 Ackermann 舵角换算；
5. 调整全局规划运动模型；
6. 标定 footprint 和代价地图；
7. 最后优化速度、控制器和日志。

如果第 2～3 步没有通过，不应直接进行 footprint 或膨胀半径优化。

## 3. 阶段一：建立可重复的基线测试

### 3.1 操作要求

- 车辆固定在地图上一个容易识别的宽阔位置；
- 每轮启动只设置一次 AMCL 初始位姿；
- 设置初始位姿后等待 5～10 秒；
- 在 RViz 同时显示 Map、LaserScan、Global Costmap、Local Costmap、RobotModel 和 TF；
- 先测试同一条直走廊内的短目标，再测试转弯目标；
- 不允许在导航过程中重新点击 `2D Pose Estimate`。

### 3.2 建议记录的数据

```bash
ros2 topic hz /scan
ros2 topic hz /Odometry
ros2 topic hz /odometry/filtered_odom
ros2 run tf2_ros tf2_echo map base_link
ros2 run tf2_ros tf2_echo base_link livox_frame
ros2 topic echo /amcl_pose
```

同时保存完整 ROS 日志目录和测试时的起点、目标点、车辆朝向及现场照片。

### 3.3 通过标准

- `/scan` 稳定大于 8 Hz；
- TF 链 `map -> odom -> base_link -> livox_frame` 连续存在；
- 车辆静止 30 秒时，RViz 中激光墙面与地图墙面不发生明显漂移；
- 设置一次初始位姿后能够连续发送多个目标，不需要反复重定位。

## 4. 阶段二：校准 Livox 外参与二维激光投影

涉及文件：

```text
src/wheeltec_nav2/launch/nav2_indoor.launch.py
src/wheeltec_nav2/scripts/livox_to_pointcloud2.py
```

### 4.1 校准静态 TF

当前配置近似为：

```text
base_footprint -> livox_frame
x = 0.12 m, y = 0.0 m, z = 0.33 m
roll = pitch = yaw = 0
```

应实测 Livox 光学中心相对 `base_link` 或 `base_footprint` 的位置和朝向，特别检查：

- X/Y 是否存在厘米级偏差；
- Livox 是否绕 Z 轴旋转安装；
- Livox 坐标轴方向是否与 ROS 约定一致；
- 车辆前进方向是否确实对应激光 X 正方向。

建议将外参改成显式 launch 参数，避免继续硬编码。

### 4.2 调整点云高度过滤

当前投影参数为：

```python
"target_frame": "livox_frame"
"min_height": -0.5
"max_height": 0.5
"range_min": 0.1
```

建议改为在 `base_footprint` 坐标系中过滤，从而使高度阈值具有明确物理意义：

```python
"target_frame": "base_footprint"
"min_height": 0.05
"max_height": 0.60
```

上述数值只是首轮测试值，最终应通过 rosbag 或现场点云确定。需要确保：

- 排除地面点；
- 排除车身自身反射；
- 保留墙体及对导航有意义的低矮障碍物。

### 4.3 过滤 Livox 无效点

`livox_to_pointcloud2.py` 当前会把 `CustomMsg` 中所有点写入 PointCloud2，包括大量 `(0, 0, 0)` 点。虽然 `range_min` 会过滤大部分零点，但建议在转换节点中明确丢弃：

- 非有限坐标；
- 距离接近零的点；
- 根据 Livox `tag` 判定为无效或噪声的点。

建议同时统计每帧输入点数、有效点数和被过滤点数，但使用节流日志，避免刷屏。

### 4.4 通过标准

- 静止时主要墙面激光与地图偏差建议小于 0.10～0.15 m；
- 机器人 footprint 内不应持续存在由自身点云产生的障碍；
- Global Costmap 中机器人中心不应长期处于高代价区域；
- 清空实时障碍层后不应立即由车身自身反射重新填满 footprint 周边。

## 5. 阶段三：稳定 AMCL

涉及文件：

```text
src/wheeltec_nav2/param/wheeltec_params/param_mini_diff_indoor.yaml
```

### 5.1 操作流程修正

- 启动后只设置一次准确初始位姿；
- 初始箭头方向必须与车头方向一致；
- 设置后缓慢直行或做小角度转弯帮助粒子收敛；
- 确认激光与地图重合后再发送导航目标；
- 车辆被搬动后才重新设置位姿。

### 5.2 参数调整建议

外参与 LaserScan 修正后，再基于实测调整：

```yaml
max_beams
laser_likelihood_max_dist
sigma_hit
z_hit
z_rand
update_min_d
update_min_a
min_particles
max_particles
```

不要在激光仍明显错位时通过增大噪声参数强行让 AMCL“看起来稳定”。

### 5.3 时间戳与 TF 延迟

针对 `Failed to transform initial pose in time`：

- 检查 `/Odometry`、`/odometry/filtered_odom`、`/scan` 时间戳是否使用同一系统时钟；
- 确认 NUC 没有启用错误的仿真时间；
- 测量消息时间戳与当前时间的实际延迟；
- 若 FAST-LIO 固有延迟较大，再针对性调整 TF/滤波发布策略。

### 5.4 通过标准

- 初始定位后不再持续出现 `AMCL cannot publish a pose`；
- 连续行驶一圈返回起点时，激光仍能与地图对齐；
- 多次发送导航目标期间不需要重设初始位姿；
- `map -> odom` 不发生无原因的大幅跳变。

## 6. 阶段四：修正 Ackermann 舵角换算

涉及文件：

```text
src/vehicle_can_bridge/src/cmd_vel_to_can_node.cpp
src/vehicle_can_bridge/launch/auto_control.launch.py
```

### 6.1 正确的运动学关系

Nav2 输出：

```text
v = cmd_vel.linear.x
omega = cmd_vel.angular.z
```

Ackermann 前轮转角应按下式计算：

```text
delta = atan(wheel_base * omega / abs(v))
```

实现时需要处理：

- `abs(v)` 接近零时禁止除零；
- 倒车时方向符号必须通过低速实车测试确认；
- `delta` 必须限制到实测最大左/右前轮角；
- 前轮角再通过标定曲线转换成左右不对称的舵机命令；
- 收到超出车辆运动学能力的 `omega` 时应限幅并节流告警。

### 6.2 必须新增的参数

建议至少增加：

```yaml
wheel_base: 0.30
max_left_road_wheel_angle: <实测值>
max_right_road_wheel_angle: <实测值>
servo_center: 90.0
servo_left_limit: 40.0
servo_right_limit: 190.0
min_speed_for_steering_calculation: 0.03
```

舵机角与前轮角不一定线性，优先使用多点标定表；若首版使用线性映射，也应分别标定左转和右转。

### 6.3 测试要求

1. 架空驱动轮或限制最大速度；
2. 分别测试正向/反向、左转/右转；
3. 测量固定舵角下的实际转弯半径；
4. 确认命令符号和车辆响应一致；
5. 根据实测结果设置 Nav2 的 `minimum_turning_radius`。

### 6.4 通过标准

- 相同 `v`、`omega` 命令能够产生可预测的曲率；
- 左右转弯半径与配置误差处于可接受范围；
- MPPI 预测轨迹与车辆实际轨迹方向一致；
- 急停或超时后舵机和驱动安全归零。

## 7. 阶段五：允许全局规划倒车

涉及文件：

```text
src/wheeltec_nav2/param/wheeltec_params/param_mini_diff_indoor.yaml
```

建议将：

```yaml
motion_model_for_search: "DUBIN"
```

改为：

```yaml
motion_model_for_search: "REEDS_SHEPP"
```

保留并重新评估：

```yaml
reverse_penalty: 2.0
change_penalty: 0.2
non_straight_penalty: 1.5
minimum_turning_radius: <实测值>
```

同时检查 MPPI：

```yaml
vx_min: -0.10
enforce_path_inversion: false
PathAngleCritic.mode: 0
```

确保控制器能够识别路径方向切换点，不会在前进/倒车切换处错误地原地打舵。

### 7.1 通过标准

- 狭窄区域可生成带倒车段的路径；
- 规划路径不会要求原地旋转；
- 前进/倒车切换前车辆能够减速到接近零；
- 行为树恢复动作不会与正常倒车路径相互冲突。

## 8. 阶段六：标定 footprint 与代价地图

### 8.1 footprint 标定

实测车辆最外轮廓相对 `base_link` 的：

- 前端距离；
- 后端距离；
- 左侧距离；
- 右侧距离。

在实测轮廓上增加合理安全余量后填写 footprint。不要默认车辆一定关于 `base_link` 对称。

### 8.2 膨胀与碰撞余量

当前值：

```yaml
local inflation_radius: 0.6
global inflation_radius: 0.8
MPPI collision_margin_distance: 0.25
```

建议在定位正确后按走廊净宽逐步调整。原则是：

- footprint 表示硬碰撞边界；
- inflation layer 用于路径偏好和安全距离；
- MPPI 碰撞余量不应与膨胀层重复叠加到无法通行；
- 必须保留足够余量覆盖定位误差和车辆控制误差。

### 8.3 通过标准

- 车辆处于走廊中心时起点不再被判为 lethal；
- 真实不可通行的窄口仍然不能生成路径；
- 可通行走廊中心能够稳定生成路径；
- 局部控制器不会持续贴墙或大幅蛇形摆动。

## 9. 阶段七：速度和日志优化

### 9.1 速度限幅

统一以下参数的最大速度和加速度：

- MPPI；
- velocity smoother；
- `cmd_vel_to_can_node`；
- 启动命令显式传入的速度参数。

对浮点数限幅告警增加容差和节流，例如仅当超限超过小量阈值时告警，避免 `0.250000...` 造成日志刷屏。

### 9.2 初始安全参数

室内首轮验证建议保持：

```text
最大前进速度：0.15～0.25 m/s
最大倒车速度：0.05～0.10 m/s
```

在定位、规划和舵角换算都通过后再逐步提高速度。

## 10. 建议增加的自动化检查

建议为以下行为增加测试：

1. `REEDS_SHEPP` 配置被正确加载；
2. Ackermann 换算在直行、左右转、倒车和零速度时输出正确；
3. 舵机命令不会超过左右限位；
4. 所有速度参数从 launch 参数到控制节点保持一致；
5. footprint 在本地和全局代价地图中一致；
6. 行为树不包含 Ackermann 车辆无法执行的原地旋转；
7. launch 参数类型和范围错误时能够快速失败。

## 11. 推荐验收场景

按以下顺序测试，每项通过后再进入下一项：

1. 静止 30 秒观察激光与地图；
2. 直行 2 m 后停车；
3. 单次缓弯；
4. 同一走廊内短距离导航；
5. 90° 转弯导航；
6. 需要一次倒车调整的目标；
7. 连续执行 5 个目标且不重设 AMCL；
8. 绕行一圈返回起点，验证累计误差；
9. 在安全条件下测试临时障碍物出现和移除；
10. 验证急停、遥控接管和命令超时。

每个场景应记录成功/失败、规划耗时、最大横向误差、是否触发恢复行为和最终定位误差。

## 12. 预期修改文件清单

| 文件 | 计划修改 |
|---|---|
| `src/wheeltec_nav2/launch/nav2_indoor.launch.py` | TF 参数化、点云投影坐标系和高度过滤 |
| `src/wheeltec_nav2/scripts/livox_to_pointcloud2.py` | 无效点和噪声点过滤、统计日志 |
| `src/wheeltec_nav2/param/wheeltec_params/param_mini_diff_indoor.yaml` | AMCL、Reeds-Shepp、实测转弯半径、footprint 和代价地图参数 |
| `src/vehicle_can_bridge/src/cmd_vel_to_can_node.cpp` | Ackermann 曲率到舵机角的正确换算 |
| `src/vehicle_can_bridge/launch/auto_control.launch.py` | 暴露转向标定和安全限制参数 |
| 对应测试文件 | 增加运动学、参数传递和配置回归测试 |

## 13. 回退策略

- 修改前保存当前可运行版本的 Git 提交或补丁；
- 每个阶段单独提交，禁止把定位、规划、底盘控制混在一个提交中；
- 参数修改保留旧值和测试记录；
- 底盘控制修改必须能快速切回当前映射方式，以便对照测试；
- 任一阶段出现方向相反、舵机撞限位、车辆异常加速或 TF 跳变时立即停止测试。

## 14. 最终验收标准

完成修改后，应同时满足：

- 激光与地图稳定重合；
- 连续目标间不需要重新设置 AMCL；
- 机器人起点在静态和实时代价地图中均可通行；
- 直线、转弯和倒车路径均可规划并正确执行；
- 实际最小转弯半径与 Nav2 配置一致；
- 狭窄走廊没有持续的 `Starting point in lethal space`；
- 失败时日志能明确区分定位、规划、控制和底盘执行问题。
