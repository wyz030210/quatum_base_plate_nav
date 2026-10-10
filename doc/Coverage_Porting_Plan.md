# 全覆盖路径规划移植方案（Gazebo/TurtleBot3 → 真实阿克曼底盘）

> 目标工作区：`/home/wyz/ros2/new_base_plate_indoor_ws`
> 参考实现：`/home/wyz/ros2/turtlebot3_ws`（Fields2Cover v2.0.0 + opennav_coverage `humble-v2`）
> 编写日期：2026-09-26

---

## 0. 一句话结论

**这不是"从零移植"，而是"修好一个被中断的移植"。** 本工作区里已经存在覆盖功能的大部分代码（桥接节点 631 行、区域选择节点 412 行、参数文件、整合 launch），唯一的硬阻塞是 `opennav_coverage` 的**源码被删除且构建失败**。该失败的根因已经定位清楚，修复成本很低。

但**真正的工作量不在构建，在于把参数从仿真值改成这台阿克曼车的物理真值**，以及一个必须在上车前完成的实测项（最小转弯半径）。

> ### ✅ 当前进度（2026-09-26）
>
> **§3「构建失败的根因与修复」已执行完毕并实测通过**：F2C 已统一到 v2.0.0、`opennav_coverage` 已切到 `humble-v2`、构建 rc=0、根因消除有直接证据（详见 §3.8）。过程中另发现并修复了一个**文档初版未记录的、工作区级的第二个构建阻塞**（5 个包，详见 §3.7）。
>
> **§5「参数表」已执行完毕**：三处参数（`coverage_params.yaml` 全表、桥接节点超时与地头、全局膨胀半径）已按车辆真值改完并验证生效。过程中修正了文档两处错误（§5.2 **指错文件**、§5.1 `linear_curv_change` 语义），并查实了 `follow_timeout_sec` 的确切语义（§5.2.1）与三个生成器的 `UNKNOWN` 回退机制（§5.1.1）。
>
> **§4.3.1「覆盖链断开」已修复**：新增 `use_coverage` 按需启动开关，桥接节点与区域选择节点
> 已接入室内启动链并实测通过（`true` 时启动并就绪、`false` 时都不启动）。详见 §4.3.1。
>
**A-4 离线几何校验已完成并全部通过**（用数值测量代替肉眼看 RViz）：
修正参数后**越界 0.000 m**、条带间距与地头精确等于设定值、路径连续。过程中发现并修正了
§5.1 里 **4 个错误参数**（`operation_width` / `min_turning_radius` / `linear_curv_change` /
`default_headland_width`），并总结出设计规则 —— 详见 §5.1.2 ~ §5.1.4 与 §7 A-4。

**阶段 B1 已完成：实测最小转弯半径 = 0.95 m**（车体中心画圆）。

这个数字比按 `max_steering_angle_deg=30°` 反推的 0.52 m **大 1.83 倍**，并因此暴露出一个
**系统性标定错误**：该参数既是自行车模型的钳位、又是舵机的满量程，配 30° 时命令 30°
只得到 17.5° 的实际前轮转角（增益 0.583），**使 MPPI / Smac / F2C 全都按车做不到的
转弯能力规划，系统性转向不足 1.71 倍**。已修正 4 处参数（§5.1.3）。

**§4.4 安全层已实施**：桥接节点现已订阅 `/vehicle/remote_manual_override`
（遥控器接管即中止覆盖任务 + 下发零速）、提供软件急停话题 `/coverage/emergency_stop`、
并在抢占期间拒绝新任务。此前工作区**没有任何 ROS 侧安全机制**（无急停、无看门狗），
详见 §4.4。

> ### 现在只剩阶段 B2（且可并进首次现场跑）
>
> 1. **验证转向标定的线性假设**：命令 `v=0.1, ω=0.105`（期望 R≈0.95），
>    量实际圆半径是否 ≈ 0.95 m。若明显偏离，说明转向机构非线性，需分段标定
>    （方法见 §5.1.3 末尾）
> 2. **实测舵机速率**，把 `linear_curv_change` 从 200（偏乐观）收紧到物理值（约 4~14）
>
> 这两项各约 2 分钟，**可以折进第一次现场跑**，不必单独安排。
> 其余（构建、参数、启动链、路径几何、安全层）都已离线验证完毕。
>
> 缺失依赖 `pointcloud_to_laserscan` **已正式安装**（`sudo apt install ros-humble-pointcloud-to-laserscan`，
> 2.0.1）。它是 `wheeltec_nav2/package.xml` 已声明的依赖，此前本机未装会导致整条室内栈起不来。
> 非挂任何临时 overlay 的状态下，室内栈启动已实测通过。

---

## 1. 已确认的决策

| # | 决策 | 结论 |
|---|---|---|
| Q1 | 目标形态 | **(b) 闭环覆盖**（规划→跟随→完成/恢复） |
| Q2 | 交付物 | 方案文档 + 文件级改动清单（本文档） |
| Q3 | 地图来源 | **(a) 使用已有的建好的地图** |
| Q4 | 作业要求 | 基本的路径全覆盖，无严格覆盖率 KPI |
| Q5 | 版本组合 | **(a) Fields2Cover v2.0.0 + opennav_coverage `humble-v2`**（与仿真一致） |
| Q6 | 覆盖区域来源 | **RViz 中手动点选多边形** |
| Q7 | 验收判据 | **(a) 定性**：RViz 路径合理 + 车跟完不撞不卡 |
| Q8 | 验证顺序 | **(a) 真车优先**（本车无任何仿真模型，无选择余地） |
| Q9 | 动态障碍策略 | **(a) 停车等待 → 超时跳过并记录漏扫**；拒绝绕行 |
| Q10 | 场地 | 200–300 m²，中等障碍密度，硬质地面 |
| Q11 | 集成架构 | **(a) 移植"实际在用"的那套**（`coverage_server` + 自研桥接节点，不用 BT navigator） |
| Q12 | 已有代码 | **(a) 修，不重写** |
| Q13 | 参数冲突 | **(a) 以车辆物理参数为准重算** |
| Q14 | 现场安全 | **(a) 全程有人在场 + 遥控抢占中止覆盖任务** |
| Q15 | `operation_width` | ~~(b) 0.80（约 25% 重叠）~~ → ⚠️ **已修正为 0.60**：0.80 > 车宽 0.68 会留 0.12 m **漏扫带**，原"25% 重叠"的方向是反的。见 §5.1.2 |
| Q16 | 轨迹记录 | **(a) 不记录**（已作为风险接受，见 §9） |
| Q17 | 覆盖方向 | **(a) 固定角度 `SET_ANGLE`** |
| Q18 | 转弯几何 | **(a) `CONTINUOUS`** ✅ 保留；「地头加宽」的执行方式已按 A-4 修正：地头宽度由**转弯外冲量**决定（1.30 m，非 `R+0.10`），且 `default_path_type` 改为 **`REEDS_SHEPP`**（允许三点掉头，把地头需求从 2.80 降到 1.30）。见 §5.1.2 / §5.1.5 |
| Q19 | 地图 | **`src/wheeltec_nav2/map/map.yaml`**，已与现场核对 |
| Q20 | 验证阶梯 | **(a) 直接上目标场地跑全场** |
| Q21 | 定位状态 | **(a) 已稳定**；`doc/IndoorNav_Issues.md` 描述的是修复前状态 |
| Q22 | 推进方式 | **(b) 并行**：不碰车的部分先做完 |
| Q23 | 最小转弯半径 | ✅ **已实测 = 0.95 m**（车体中心画圆）。比几何反推的 0.52 大 1.83 倍，并暴露出转向标定错误（见 §5.1.3、R21）。**已回填** 4 处参数 |
| Q24 | `free_thresh` 缺陷 | **(b) 不修**，进风险清单 |

---

## 2. 现状盘点

### 2.1 已经存在的覆盖代码（不需要重写）

| 文件 | 规模 | 作用 |
|---|---|---|
| `src/nav2_mission/nav2_mission/coverage_navpath_bridge_node.py` | 631 行 | `ComputeCoveragePath` action → `FollowPath` action 的桥接；提供 `/execute_coverage_navpath` |
| `src/nav2_mission/nav2_mission/coverage_region_selector_node.py` | 412 行 | RViz `clicked_point` 点选多边形 → 触发覆盖 |
| `src/wheeltec_nav2/param/coverage_params.yaml` | — | `coverage_server` 参数（**值需重算**，见 §6） |
| `src/wheeltec_nav2/launch/navigation_with_coverage_launch.py` | — | 把 `coverage_server` 加入 lifecycle manager |
| `install/opennav_coverage_msgs/` | — | ~~humble 版本，需重建~~ → **已重建为 humble-v2** ✅ |
| ~~`src/Fields2Cover-1.2.1/`~~ | — | ~~F2C 1.2.1~~ → **已替换为 `src/Fields2Cover`（v2.0.0）** ✅，**旧源码已彻底删除** |
| `src/opennav_coverage/` | — | **新增** ✅（`humble-v2 @ 81b63a7`，5 个不需要的子包已加 `COLCON_IGNORE`） |

### 2.2 车辆物理参数（真值）

| 项 | 值 | 来源 |
|---|---|---|
| 运动学 | **Ackermann**（舵机转向 + 轮毂电机） | `vehicle_can_bridge/include/.../ackermann_steering.hpp` |
| 轴距 / 轮距 | 0.30 m / 0.25 m | `vehicle_can_bridge/config/vehicle_params.yaml:12-13` |
| 轮半径 | 0.1275 m | 同上 `:8-9` |
| **footprint** | `[[-0.44,-0.34],[-0.44,0.34],[0.44,0.34],[0.44,-0.34]]` = **0.88 × 0.68 m** | `param_mini_diff_indoor.yaml:203,245` |
| **半对角线（外接圆半径）** | **0.556 m** | 计算值 |
| 最大前进 / 倒车速度 | 0.25 / 0.10 m/s | `start_indoor_navigation.sh` |
| 最大角速度 | 0.50 rad/s | 同上 |
| 底盘驱动 | `vehicle_can_bridge`，订阅 **`/cmd_vel`** → CAN（2 Mbaud） | `cmd_vel_to_can_node.cpp:59` |
| 规划器 | `SmacPlannerHybrid` + **REEDS_SHEPP** | `param_mini_diff_indoor.yaml:311` |
| 控制器 | **MPPI**，`motion_model: "Ackermann"` | 同文件 `:71-186` |
| 定位 | FAST-LIO `/Odometry` → robot_localization EKF → `/odometry/filtered_odom` | `indoor_amcl.launch.py` |
| 地图 | `wheeltec_nav2/map/map.yaml`，res 0.12，origin `[-25,-20.2,0]`，655×244 px = 78.6 × 29.2 m | |

### 2.3 与仿真侧的关键差异（**不能照抄参数**）

| 项 | 仿真（TurtleBot3 Burger） | 真车（阿克曼） |
|---|---|---|
| 运动学 | 差速，可原地旋转 | **阿克曼，不能原地旋转** |
| 车宽 | 0.22 m | **0.68 m** |
| 控制器 | RPP（`CoverageFollowPath`） | **MPPI + Ackermann 模型** |
| 里程计 | Gazebo `/odom` | **FAST-LIO + EKF，`/odometry/filtered_odom`（无 `/odom`）** |
| 地图 | 仿真 world | **0.12 m 分辨率预建图** |
| `use_sim_time` | true | **必须 false** |

> **正面因素**：真车用 MPPI + Ackermann 模型跟随覆盖路径，比仿真的 RPP 更适合阿克曼车 —— RPP 不理解转向约束，MPPI 理解。这一项不需要改。

---

## 3. 构建失败的根因与修复（核心）

### 3.1 根因（已完整定位）

失败信息是 `fatal error: fields2cover.h: 没有那个文件或目录`，但**头文件是存在的**。完整因果链：

1. `install/fields2cover/include/fields2cover.h` **确实存在**；
2. 但失败构建的编译命令行里**完全没有 fields2cover 的 include 路径**：
   `build/opennav_coverage/CMakeFiles/opennav_coverage_core.dir/flags.make` 的 `CXX_INCLUDES` 末尾是 `-isystem /opt/ros/humble/include/smclib`，没有任何 fields2cover 条目；
3. 原因：**Fields2Cover 1.2.1 把 CMake 配置装到了 `<前缀>/cmake/fields2cover/`（小写）**。CMake 在 Linux 上**不搜索**这个路径 —— `find_package` 的 `<prefix>/(cmake|CMake)/` 模式是 **Windows 专用**（`(W)` 标记），所有 `(U)` 模式都要求路径中有 `lib*` 或 `share` 组件；
4. 因此 `find_package(Fields2Cover)` **跳过了本工作区自己的 1.2.1**，顺着继承的 `CMAKE_PREFIX_PATH` 落到了 **`turtlebot3_ws` 的 v2.0.0**（证据：`build/opennav_coverage/CMakeCache.txt:269` → `Fields2Cover_DIR:PATH=/home/wyz/ros2/turtlebot3_ws/install/fields2cover/lib/cmake/Fields2Cover`）；
5. 而 **v2.0.0 只导出带命名空间的目标 `Fields2Cover::Fields2Cover`**，被删除的那份源码（旧 `humble` 分支）的 `CMakeLists.txt` 写的是**裸名** `target_link_libraries(${library_name} Fields2Cover)`；
6. CMake 找不到名为 `Fields2Cover` 的 target，就把它当成普通 `-lFields2Cover` 处理，**静默地不添加任何 include 目录**（无警告）。于是编译器找不到头文件。

### 3.2 附带发现：`new_base_plate_ws2` 的成功是不可复现的

`new_base_plate_ws2` 之所以构建成功（rc=0），是因为它 CMakeCache 里的 `Fields2Cover_DIR` 指向 `/home/wyz/ros2/new_base_plate_ws/build/fields2cover` —— **一个已经不存在的工作区**。它恰好是 1.2.1、恰好导出裸名 target，所以撞对了。**不能把它当作"已验证可用"的参考。**

### 3.3 修复方式

采用 **Q5(a)**：统一到 **Fields2Cover v2.0.0 + opennav_coverage `humble-v2`**。这样做**同时消除**了这个 CMake 陷阱，因为 `humble-v2` 的 `CMakeLists.txt` 带有命名空间回退逻辑：

```cmake
find_package(Fields2Cover REQUIRED)
# FIELDS2COVER_LIBRARIES = Fields2Cover::Fields2Cover 若该 target 存在，否则回退 Fields2Cover
```

而 F2C **v2.0.0 的配置装在 `lib/cmake/Fields2Cover/`（大写、含 `lib/`）**，是 CMake 在 Linux 上**能搜索到**的标准路径。根因自然消失。

> **如果选择退回 1.2.1（未采纳）**：旧 `humble` 分支的 CMakeLists 没有命名空间回退，在同一个工作区里会**重现完全一样的失败**。这是 Q5(a) 优于 (b) 的决定性理由。

### 3.4 构建步骤

> ⚠️ **必须在「不继承 turtlebot3_ws」的环境里构建**。见 §3.6。

```bash
cd /home/wyz/ros2/new_base_plate_indoor_ws

# 1) 删除 1.2.1 源码（同一个工作区里留两份同名包会让 colcon 冲突）
#    建议：先 mv 到工作区外的临时位置，等 §3.4 第 6 步构建通过后再删，
#    这样一旦发现新版本有问题还能退回。本次执行即按此顺序，
#    验证通过后已把备份删除。
rm -rf src/Fields2Cover-1.2.1

# 2) 彻底清除污染（不能增量编译）
rm -rf build/fields2cover      install/fields2cover
rm -rf build/opennav_coverage  build/opennav_coverage_msgs
rm -rf install/opennav_coverage install/opennav_coverage_msgs

# 3) 引入 F2C v2.0.0（从 turtlebot3_ws 拷贝，或重新浅克隆 tag v2.0.0）
cp -a /home/wyz/ros2/turtlebot3_ws/src/Fields2Cover  src/Fields2Cover

# 4) 引入 opennav_coverage humble-v2（保留 .git 以便追溯）
cp -a /home/wyz/ros2/turtlebot3_ws/src/opennav_coverage  src/opennav_coverage

# 5) 确认没有别的 F2C 残留干扰 find_package
grep -rn "Fields2Cover_DIR" build/ install/ 2>/dev/null   # 应无输出

# 6) 重建 —— 在隔离环境里（关键！）
env -i HOME=$HOME USER=$USER \
  PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
  TERM=dumb LANG=C.UTF-8 \
  bash -c 'source /opt/ros/humble/setup.bash && \
    cd /home/wyz/ros2/new_base_plate_indoor_ws && \
    colcon build --symlink-install \
      --packages-up-to opennav_coverage wheeltec_nav2 nav2_mission'
```

**依赖闭包是 6 个包**（不是全工作区）：`fields2cover`、`opennav_coverage_msgs`、`opennav_coverage`、`livox_ros_driver2`、`nav2_mission`、`wheeltec_nav2`。预计耗时 **~2 分钟**（F2C 单独约 1 分 29 秒）。

> **注意**：`opennav_coverage` 包内含 7 个子包（`opennav_coverage_bt`、`opennav_coverage_navigator`、`backported_bt_navigator`、`opennav_row_coverage`、`opennav_coverage_demo` 等）。按 **Q11(a)**，**只需要 `opennav_coverage` + `opennav_coverage_msgs`**。建议在不需要的 5 个目录里放 `COLCON_IGNORE`，避免首次编译未验证的上游包：
>
> ```bash
> cd src/opennav_coverage
> for p in opennav_coverage_bt opennav_coverage_navigator backported_bt_navigator \
>          opennav_row_coverage opennav_coverage_demo; do touch $p/COLCON_IGNORE; done
> ```

### 3.5 桥接代码不需要改动（已验证）

逐字段核对结论：`coverage_navpath_bridge_node.py` 和 `coverage_region_selector_node.py` 使用的**每一个** opennav 字段在 `humble` 与 `humble-v2` **两个版本中都存在**：

`generate_headland` / `generate_route` / `generate_path` / `use_gml_file` / `frame_id` / `polygons` / `nav_path` / `error_code` / `Result.NONE` / `Coordinate.axis1,axis2` / `Coordinates.coordinates` / 整个 `NavigateCompleteCoverage`。

> 两版本的 `ComputeCoveragePath.action` **不是线兼容的**（goal 12 → 18 字段，result 新增 `task_time`，新增 `DecompMode.msg`），但因为只用了交集部分，**桥接零改动**。**必须重建 `opennav_coverage_msgs`**（当前 install 里是 humble 版本）。

**修复后的复核（用新装出来的 humble-v2 定义逐字段重查）**：桥接/选择器实际引用的 13 个字段 —— `generate_headland`、`generate_route`、`generate_path`、`use_gml_file`、`frame_id`、`polygons`、`nav_path`、`error_code`、`Coordinate`、`Coordinates`、`axis1`、`axis2`、`coordinates` —— **全部命中**。`NONE` 不是字段名而是枚举成员，`ComputeCoveragePath.action:38` 与 `NavigateCompleteCoverage.action:16` 均有 `uint16 NONE=0`，正是代码里 `Result.NONE` 的用法。**结论成立：桥接与选择器源码零改动。**

---

### 3.6 ⚠️ 构建环境必须隔离（文档初版遗漏的点）

**这是当初 `find_package` 漏到 turtlebot3_ws 的通道，必须堵住。**

`~/.bashrc:127` 有 `source ~/ros2/turtlebot3_ws/install/setup.bash`，因此**每开一个终端，`CMAKE_PREFIX_PATH` 里就带着 turtlebot3_ws（含 `install/fields2cover`、`install/opennav_coverage`、`install/opennav_coverage_msgs`）**。这是 §3.1 第 4 步能发生的前提。

修复后本工作区的包会**优先**被找到（colcon 把自己的 `install/` 前置到 `CMAKE_PREFIX_PATH`），所以日常构建不会再出错。但为了让构建**可复现、可自证**，重建时应显式隔离：

```bash
env -i HOME=$HOME USER=$USER \
  PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
  TERM=dumb LANG=C.UTF-8 \
  bash -c 'source /opt/ros/humble/setup.bash && cd <ws> && colcon build ...'
```

`env -i` + `bash -c`（**非** `-l`，否则会重新读 `.bashrc`）可确保 `CMAKE_PREFIX_PATH` 中**除 `/opt/ros/humble` 外为空**。已实测：在此隔离环境下构建通过，证明本工作区**不依赖 turtlebot3_ws**（全工作区 `package.xml` 无任何 turtlebot3 依赖，已核查）。

---

### 3.7 附带发现：第二个构建阻塞（工作区级，与覆盖功能无关但会挡住构建）

首次执行 §3.4 第 6 步时会**失败在 `livox_ros_driver2`**，报：

```
failed to create symbolic link
  'build/livox_ros_driver2/ament_cmake_python/livox_ros_driver2/livox_ros_driver2'
because existing path cannot be removed: Is a directory
```

**这不是本方案引入的，也早于本次修复**（冲突目录 mtime `2026-09-26 15:08:06`，而本次修复从 16:13 才开始）。

- 该目标的真实命令是 `cmake -E create_symlink`，把 `build/<包>/rosidl_generator_py/<包>` **软链**到 `build/<包>/ament_cmake_python/<包>/<包>`；
- 但目标位置**已存在一个内容相同的真实目录**（早先某次构建拷贝而非软链留下），`create_symlink` 拒绝覆盖目录，于是失败；
- 与 `--symlink-install` **无关**（该选项只影响 install 侧），加不加都失败；
- **受影响的是 5 个包**，特征完全一致 —— 都是「源侧没有同名 python 目录」的 rosidl 接口包：

  `livox_ros_driver2`、`vehicle_can_bridge`、`fast_lio`、`wheeltec_robot_msg`、`location`

**修复**：删除这些阻塞目录。删除前已用 `diff -rq` 逐个确认其内容与 `rosidl_generator_py/<包>/`（软链的权威目标）**逐字节相同**，因此**零损失**。修复后 5 个包全部构建通过。

```bash
for p in livox_ros_driver2 vehicle_can_bridge fast_lio wheeltec_robot_msg location; do
  SRC=build/$p/rosidl_generator_py/$p
  BLK=build/$p/ament_cmake_python/$p/$p
  [ -d "$BLK" ] && [ -d "$SRC" ] && diff -rq "$SRC" "$BLK" >/dev/null 2>&1 && rm -rf "$BLK"
done
```

> 值得留意：`vehicle_can_bridge` 也在其中 —— 它是 §8 分析的底盘驱动。这个阻塞此前一直在，意味着**该包当时也无法重新构建**。

---

### 3.8 修复后的实测结果（验收证据）

| 证据 | 修复前 | **修复后** |
|---|---|---|
| `build/opennav_coverage/CMakeCache.txt` 的 `Fields2Cover_DIR` | `/home/wyz/ros2/turtlebot3_ws/install/fields2cover/lib/cmake/Fields2Cover`（**turtlebot3_ws**） | **`/home/wyz/ros2/new_base_plate_indoor_ws/install/fields2cover/lib/cmake/Fields2Cover`**（本工作区） |
| `build/opennav_coverage/CMakeFiles/opennav_coverage_core.dir/flags.make` 的 `CXX_INCLUDES` | 无任何 fields2cover 条目 | `-isystem /home/wyz/ros2/new_base_plate_indoor_ws/install/fields2cover/include` |
| `ros2 interface show opennav_coverage_msgs/msg/DecompMode` | 不存在（humble 版无此消息） | **成功** → humble-v2 生效 |
| `import fields2cover` | 1.2.1 | **2.0.0** |
| `colcon build` 依赖闭包 | `fatal error: fields2cover.h` | **6 个包全部 `Finished`，rc=0** |
| **全工作区 15 个包** | 5 个包构建失败 | **`Summary: 15 packages finished`，rc=0** ✅ |

> 上表最后一行的验证方式：清空覆盖链三个包的 `build/`+`install/` 后，**按 §3.4 步骤从零重建**，再对**全部 15 个包**跑一次完整 `colcon build`。两次均 rc=0。因此 §3.4 记录的流程是**可复现的**，不只是"当前状态恰好能跑"。

各包实测耗时：`fields2cover` 1 min 29 s（首次）、`opennav_coverage_msgs` 7.0 s、`opennav_coverage` 20.2 s；增量复建整个闭包 4.1 s。

**已知无害噪音（上游自带，不必处理）**：`fields2cover` 构建期 stderr 有两条

```
CMake Error: Error evaluating generator expression:
  $<TARGET_OBJECTS:ortools_math_opt_core>  Objects of target ... referenced but no such target exists
```

来自上游 **SWIG Python 绑定**（`swig/python/CMakeLists.txt`）引用不存在的 OR-tools 子目标。**非致命**：cmake 继续完成，`libFields2Cover.so`、CMake 配置、`_fields2cover_python.so` 均正常产出，`import fields2cover` 成功。且桥接节点**不直接依赖** fields2cover（走 ROS action），故无影响。

> **环境备注**：本机活动 `cmake` 是 **`/usr/local/bin/cmake` 4.3.2**，不是 Ubuntu 自带的 3.22。构建日志中的大量 `CMake Deprecation Warning (CMP0148/CMP0144/CMP0167)` 均源于此，属无害。

---

## 4. 文件级改动清单

### 4.1 构建相关

| 文件 / 目录 | 现状 | 改动 |
|---|---|---|
| `src/Fields2Cover-1.2.1/` | F2C 1.2.1 | ✅ **已删除**，替换为 `src/Fields2Cover/`（v2.0.0） |
| `src/opennav_coverage/` | **不存在**（源码被删） | ✅ **已新增**（从 `turtlebot3_ws` 拷 `humble-v2 @ 81b63a7`），5 个不需要的子包已加 `COLCON_IGNORE` |
| `build/`、`install/` 下 `fields2cover`、`opennav_coverage*` | 陈旧/失败残留 | ✅ **已 `rm -rf`**（原 `install/opennav_coverage/` 只有 `share/`、没有 `lib/`），并已重建 |
| **（新增发现，见 §3.7）** `build/{livox_ros_driver2,vehicle_can_bridge,fast_lio,wheeltec_robot_msg,location}/ament_cmake_python/<包>/<包>` | 同名**真实目录**阻塞 `create_symlink`，导致这 5 个包无法构建 | ✅ **已删除**（删前 `diff -rq` 确认与 `rosidl_generator_py/<包>/` 逐字节相同，零损失） |

### 4.2 参数相关

| 文件 | 现状 | 改动 |
|---|---|---|
| `src/wheeltec_nav2/param/coverage_params.yaml` | 仿真遗留值 | **按 §5 全表重写** |
| `src/wheeltec_nav2/param/wheeltec_params/param_mini_diff_indoor.yaml` | 全局 `inflation_radius: 0.55` | → **`0.60`**（见 §5.3） |
| `src/wheeltec_nav2/param/coverage_params_chang.yaml` | 未被任何 launch 引用 | 删除，或加注释标注为未使用 |
| `src/nav2/map/map.yaml` | **0 字节空文件** | 删除（或补内容）；它被 3 个 launch 当默认值引用 |
| `src/wheeltec_nav2/map/其他地图/fastlio_map.*` | `.pgm` 实为 **JPEG**，`nav2_map_server` 会加载失败 | 删除，或改名 `.jpg` 并从所有 launch 默认值中移除 |

### 4.3 Launch / 脚本

| 文件 | 现状 | 改动 |
|---|---|---|
| `start_indoor_navigation.sh` | 默认地图硬编码 `/home/car/ros2/...` | 改为工作区相对路径（`${workspace_dir}/src/wheeltec_nav2/map/map.yaml`），保留 `INDOOR_MAP` 覆盖 |
| `src/robot_bringup/launch/system.launch.py` | 引用**不存在的** `turn_on_wheeltec_robot`；默认地图指向不存在的 `map/fastlio_map.yaml` | **删除**或在文件头标注"已知失效，勿使用" |
| `src/nav2/launch/nav2_launch.py` | 引用**不存在的** `turn_on_wheeltec_robot` | 同上 |
| `src/nav2/package.xml`、`src/robot_bringup/package.xml` | `depend` 里含不存在的 `turn_on_wheeltec_robot` | 移除该依赖（否则 `rosdep`/构建可能失败） |
| `src/wheeltec_nav2/launch/navigation_with_coverage_launch.py` | 已把 `coverage_server` 加入 lifecycle manager；`coverage_params_file` 指向 `param/coverage_params.yaml` ✅ 已核对 | 无需改动 |
| 🔴 **覆盖链缺两个节点（本次新发现，见 §4.3.1）** | `coverage_server` 起来了，但**桥接节点与区域选择节点都没有被启动** | ✅ **已补**，用 `use_coverage` 开关按需启动（见 §4.3.1） |
| ~~⚠️ 开发机缺 `pointcloud_to_laserscan`~~ | 该包未安装，`nav2_indoor.launch.py:115` 需要它 → 整条室内栈在本机起不来。它是 `src/wheeltec_nav2/package.xml:20` 已声明的依赖，只是本机没装 | ✅ **已 `sudo apt install` 正式安装（2.0.1）**，非挂 overlay 状态下整栈启动通过。过程记录见 §4.3.2 |

#### 4.3.2 ~~本机 `pointcloud_to_laserscan` 缺失~~ ✅ 已正式安装解决

**结论：已执行 `sudo apt install ros-humble-pointcloud-to-laserscan` 正式安装（2.0.1，装在
`/opt/ros/humble`，`ldd` 显示 0 个未解析库），临时 overlay 已删除。非挂 overlay 状态下整条室内栈启动通过。**

下文仅作记录，说明当时在**不具备 root** 的情况下如何绕过（思路可复用于其他缺包场景）：

本机 `sudo` 曾需要密码，无法 `apt install`。当时改用**不需要 root** 的两步把它挂到 ROS 搜索路径上：

```bash
# ① apt-get download 不需要 root
cd /tmp && apt-get download ros-humble-pointcloud-to-laserscan
# ② 解到本地目录，不碰系统路径
dpkg -x ros-humble-pointcloud-to-laserscan_*.deb /home/wyz/ros2/local_ros_deps/
# ③ 挂载（每次开新终端、跑室内栈之前）
source /home/wyz/ros2/local_ros_deps/setup.bash
```

已验证：挂载前 `ros2 pkg prefix` 报 `Package not found`，挂载后能找到，
且 `ldd` 显示该节点**动态库 0 个未解析**（它的全部依赖 `laser_geometry` / `message_filters` /
`tf2_sensor_msgs` / `rclcpp_components` 本机都已具备，故只解这一个包就够）。

> ⚠️ 该 overlay **不是正式安装**，只是当时本机验证用的临时手段（现已删除）：
> - 不写系统目录，完全可逆 —— `rm -rf /home/wyz/ros2/local_ros_deps` 即彻底清除；
> - 不在工作区内，不会被 rsync 到车上；
> - **本机已改用正式安装**，overlay 已删除，不存在两份副本混淆的问题。
>
> **车上同样请用正式方式**：`sudo apt install ros-humble-pointcloud-to-laserscan`
> （或 `rosdep install --from-paths src --ignore-src -y`）。

#### 4.3.1 🔴 覆盖链在室内启动链上是断的（本次发现，文档初版遗漏）

**实测追踪的调用链**（`start_indoor_navigation.sh:16` 为入口）：

```
start_indoor_navigation.sh
└─ robot_bringup/launch/indoor_amcl.launch.py
   ├─ auto_control.launch.py                （底盘）
   ├─ livox msg_MID360_launch.py / fast_lio mapping.launch.py
   └─ wheeltec_nav2/launch/nav2_indoor.launch.py
      └─ bringup_launch.py
         └─ navigation_with_coverage_launch.py
            ✅ coverage_server        （已加入 lifecycle manager）
            ❌ coverage_navpath_bridge    ← 修复前：没有
            ❌ coverage_region_selector   ← 修复前：没有
```

而这两个节点**只**出现在下列两个 launch 里 —— 正是 §4.3 上表判定为**已失效**的那两个：

| 启动者 | 状态 |
|---|---|
| `robot_bringup/launch/system.launch.py:118,205` | ❌ 引用不存在的 `turn_on_wheeltec_robot` |
| `src/nav2/launch/nav2_launch.py:97` | ❌ 同上 |

**后果**：走文档推荐的室内入口启动后，`/execute_coverage_navpath` 这个 action 根本不存在，
RViz 里点选多边形也**没有任何节点在监听** `clicked_point` —— 覆盖功能在整条链上是断开的。
§5 的参数改得再对也不会被触发。

#### ✅ 已实施：`use_coverage` 按需启动开关

按「覆盖节点按需启动」实施。新增 launch 参数 **`use_coverage`（默认 `false`）**，
贯穿四层并最终在 `navigation_with_coverage_launch.py` 上用 `IfCondition` 控制两个条件 include：

| 文件 | 改动 |
|---|---|
| `wheeltec_nav2/launch/navigation_with_coverage_launch.py` | 新增 `use_coverage` 声明；新增两个 `IncludeLaunchDescription`（`coverage_navpath_bridge.launch.py`、`coverage_region_selector.launch.py`），均带 `condition=IfCondition(use_coverage)` |
| `wheeltec_nav2/launch/bringup_launch.py` | 声明 `use_coverage` 并传入上一层 |
| `wheeltec_nav2/launch/nav2_indoor.launch.py` | 同上 |
| `robot_bringup/launch/indoor_amcl.launch.py` | 同上（顶层） |
| `start_indoor_navigation.sh` | 新增 `USE_COVERAGE` 环境变量；顺带修掉硬编码的 `/home/car/...` 地图默认路径（§4.3 原定项） |

用法：

```bash
USE_COVERAGE=true ./start_indoor_navigation.sh     # 覆盖任务
./start_indoor_navigation.sh                        # 日常室内导航（默认不起覆盖节点）
# 或直接：
ros2 launch robot_bringup indoor_amcl.launch.py use_coverage:=true map:=<...>/map.yaml
```

**实测验证**（真实启动 `bringup_launch.py` 两次对照）：

| 节点 | `use_coverage:=true` | `use_coverage:=false` |
|---|---|---|
| `coverage_navpath_bridge` | ✅ 启动并就绪 | 未启动 |
| `coverage_region_selector` | ✅ 启动并就绪 | 未启动 |
| `coverage_server` | 启动 | 启动（见下方说明） |

日志确认两者真正就绪且 action 名对得上：

```
[coverage_navpath_bridge]: coverage_navpath_bridge ready:
    execute=/execute_coverage_navpath, compute=/compute_coverage_path, follow=/follow_path
[coverage_region_selector]: coverage_region_selector ready:
    clicked_point_topic=/clicked_point, default_frame=map,
    execute_action=/execute_coverage_navpath
```

> #### ⚠️ 一处**刻意没有**gate 的地方（诚实说明）
>
> **`coverage_server` 仍然无条件随 lifecycle manager 启动**，`use_coverage:=false` 时它照样在跑（空闲）。
>
> **原因**：`use_composition` 默认为 **`True`**，即组合分支才是默认路径；而 `ComposableNode`
> **不支持 `condition`**，`coverage_server` 与另外 6 个 Nav2 节点在同一个 `LoadComposableNodes`
> 列表里。要单独 gate 它，必须把这条**当前正常工作的默认路径**拆成 4 个 action
> （主节点列表 / coverage 单独列表 / 两个 `node_names` 不同的 lifecycle manager）。
>
> 为了省一个空闲节点去动正常导航的启动结构，**风险不成比例** —— 所以选择纯增量的改法。
>
> **影响**：`use_coverage:=false` 时覆盖**无法被触发**（没有 action server、没有 RViz 监听者），
> 这正是"按需"要的效果；但 `coverage_server` 会空转、且 F2C 库常驻内存。
> 若将来确实要连它一起 gate，改法见上一段，届时务必回归测试普通导航。

> **另注**：`coverage_navpath_bridge` 的 `server_wait_timeout_sec`（默认为 10.0）**不需要**因为
> 「同一次启动内并发拉起」而调大 —— 该参数用在 `_execute_callback` 内
> （`coverage_navpath_bridge_node.py:179,189`），是**每次收到覆盖目标时**才等 action server，
> 与节点启动顺序无关。此点曾一度判断错误，已核实并撤回改动。

### 4.4 安全相关 ✅ 已实施

> **背景：本工作区原本没有任何 ROS 侧安全机制** —— 无急停话题/服务、无 `cmd_vel` 看门狗
> （`cmd_vel_to_can_node.cpp` 全文零个定时器，停发指令不会让车停下，连上一帧都不重传）。
> 唯一的兜底是 STM32 的 500 ms 超时→退回 RC。本节补上**自主运行期间的软件安全层**。

| 项 | 实现 | 原因 |
|---|---|---|
| ✅ **订阅 `/vehicle/remote_manual_override`** | `coverage_navpath_bridge_node.py` 新增订阅（`std_msgs/Bool`，约 1 Hz）；为 `true` 时**中止覆盖任务 + 下发零速** | 该信号语义正是"遥控器已接管、ROS 指令被固件忽略"，但此前**全工作区无任何订阅者**。不订阅不会撞车（遥控器硬件优先），但任务会停留在"执行中"；**一旦遥控器拨回自动挡，STM32 重新接受 ROS 指令，任务从原地继续跑** —— 表现为"我以为它停了，它又动了" |
| ✅ **软件急停入口** | 话题 `/coverage/emergency_stop`（`std_msgs/Bool`，发 `true` 即触发）→ 中止任务 + 下发零速 | 覆盖运行时 `keyboard_control.py` 并不在跑，此前没有任何软件停车手段 |
| ✅ **抢占期间拒绝新任务** | `_goal_callback` 在 `_manual_override_active` 为真时**拒绝**新覆盖目标 | 防止"人手持遥控器时车自己跑起来" |
| ✅ 禁止用 `complete_control.launch.py` 跑覆盖 | `start_indoor_navigation.sh` 已改走 `auto_control` 链（§4.3） | 该 launch 只设遗留参数 `max_linear_speed: 2.0`，导致 `max_forward_speed`/`max_reverse_speed` **回退到 2.0 m/s**（应为 0.25/0.10），代码里的速度钳位形同虚设 |

**实现要点**

- 抢占/急停回调只**置位一个标志**（`_safety_abort`），由 `_wait_future()` 检测 →
  取消内层 `FollowPath` → 外层 goal `abort()`，返回 `error_code = 909 (ERR_SAFETY_ABORT)`。
  不需要持有内层 handle，复用了既有的等待/取消机制。
- ⚠️ **不能沿用 `client.wait_for_server(timeout_sec=...)`**：那是一次**阻塞**调用，
  期间无法感知 `_safety_abort`。实测踩到 —— 抢占恰好发生在"等 action server"阶段时，
  零速**已经立即发出**（安全动作生效），但任务要等整个超时结束才收尾，
  且 `error_code` 变成 `901 (ACTION_SERVER_UNAVAILABLE)` 而不是 `909`。
  已改为轮询式 `_wait_for_action_server()`，每 50 ms 检查一次中止标志。
  （真机上 `/follow_path` 由 controller_server 提供，正常情况下不会长时间阻塞，
  但"等待期间被抢占"仍应即时响应。）
- **零速必须显式下发**（默认重复 5 帧，参数 `estop_zero_velocity_repeats`）：
  底盘命令侧没有看门狗，不发这一帧车不会停。
- `remote_manual_override` 是 **1 Hz 持续信号**，回调里只在该信号**上升沿**触发一次，
  避免 1 Hz 刷屏；`_manual_override_active` 会**锁存**，直到信号变回 `false`。
- 三个话题名都是节点参数（`manual_override_topic` / `emergency_stop_topic` / `cmd_vel_topic`），
  默认值即上表。**未**写进 `coverage_navpath_bridge.launch.py` —— 避免再制造一处
  "两处默认值要同步"的隐患（§5.2 的教训）。

**用法**

```bash
# 软件急停
ros2 topic pub --once /coverage/emergency_stop std_msgs/msg/Bool "{data: true}"

# 确认安全层在线（启动日志里会打印）
#   safety: manual_override=/vehicle/remote_manual_override,
#           emergency_stop=/coverage/emergency_stop, zero_vel_on=/cmd_vel
```

**实测验证**（起 `coverage_server` + 桥接节点，发覆盖目标，在任务执行中触发）

| 入口 | 结果 |
|---|---|
| `/vehicle/remote_manual_override = true` | ✅ `error_code = 909`，`/cmd_vel` 收到 **5 帧零速** |
| `/coverage/emergency_stop = true` | ✅ 同上 |

节点话题确认（`ros2 node info /coverage_navpath_bridge`）：

```
Subscribers:  /coverage/emergency_stop: std_msgs/msg/Bool
              /vehicle/remote_manual_override: std_msgs/msg/Bool
Publishers:   /cmd_vel: geometry_msgs/msg/Twist
```

抢占触发到任务收尾的延迟：**50 ms**（修复阻塞式等待前是 40 s，见下方要点）。

> **好消息**：底盘另有一层硬件兜底 —— STM32 固件有 **500 ms 消息超时看门狗**，ROS 侧停止发送
> 超过 500 ms 会**自动退回 RC 手动控制**（`docs/DEPLOYMENT.md:274`）。这是本系统里**真实存在的一道
> 保护**，但它同时意味着：一旦触发，车辆控制权交给遥控器 —— **RC 摇杆必须在中位、且有人持握**，
> 否则车会按遥控器当前位置动作。

### 4.5 文档同步

| 文件 | 改动 |
|---|---|
| `INDOOR_AMCL_RUNBOOK.md`、`doc/GettingStarted.md` | 地图路径由 `catographer_ws/map/003.yaml` 改为 `wheeltec_nav2/map/map.yaml`（两者内容当前**逐字节相同**，`md5 3f0ab00c...`） |
| `doc/IndoorNav_Issues.md` | 标注**已过时条目**：§3.6 规划器已是 `REEDS_SHEPP`（非 `DUBIN`）；§3.7 舵角换算已实现；§3.8 膨胀半径现为 0.55/0.45（非 0.8/0.6） |
| `doc/IndoorNav_Modification_Plan.md` | 标注「阶段四：Ackermann 舵角换算」「阶段五：允许倒车」**已执行** |
| `src/wheeltec_nav2/map/map.yaml` | 按 Q24(b) **不改** `free_thresh`；在文件内加注释说明该决定及其后果 |

---

### 4.6 依赖元数据问题（`rosdep` 无法解析的键）

执行 `rosdep check --from-paths src --ignore-src` 的结果：**"All system dependencies have been satisfied"**
—— 即**没有任何"该装而没装"的依赖**。但有 5 个 rosdep 键在数据库里**不存在**，
它们会让 `rosdep install` 报错（部署到车上时会挡路）。

| 包 | 键 | 判定 | 处理 |
|---|---|---|---|
| `nav2` | `turn_on_wheeltec_robot` | **幽灵依赖**，源码目录不存在（§4.3 已列） | ✅ **已移除** |
| `robot_bringup` | `turn_on_wheeltec_robot` | 同上 | ✅ **已移除** |
| `location` | `geographicLib` | **键名大小写拼错**，正确键是 `geographiclib` → `libgeographic-dev`。库本身**已装**（`pkg-config` 报 GeographicLib **1.52**），所以 `location` 能正常构建，纯粹是元数据错误 | ⬜ 待改 |
| `ascamera` | `rospy`、`catkin` | **ROS 1 遗留**（Humble 是 ROS 2）。源码里**未实际引用** `rospy`（已 grep 确认），该包构建正常，属 package.xml 陈旧 | ⬜ 待改 |
| `nav2_mission` | `ament_python` | 写成 `<buildtool_depend>ament_python</buildtool_depend>`，而 `ament_python` 不是有效的 rosdep 键。构建不受影响（该包构建通过），只影响 `rosdep install` | ⬜ 待改 |

> **影响面**：这 5 项**都不影响构建**（当前 15/15 包全部构建通过），只影响 `rosdep install` 的可用性。
> **重要性**：在车上部署时如果用 `rosdep install` 拉依赖，会因这些键报错而中断 —— 建议在正式部署前清掉。
> 后 3 项未改动，属本次新发现，需确认后再处理。

---

### 4.7 部署到车上（NUC）的关键约束 ⚠️

> 车：`car@172.20.10.5`（`car-NUC13VYKi5`），工作区 `/home/car/ros2/new_base_plate_indoor_ws`。
> 开发机与车同一子网（`172.20.10.3/28`）。

#### 4.7.1 🔴 车上 **GitHub 不可达** → F2C 必须用本地依赖源码

`Fields2Cover` v2.0.0 的 `f2c_declare_dependencies()`（`CMakeLists.txt:62` 无条件调用）**声明 4 个外部依赖**：

| 依赖 | 车上如何满足 |
|---|---|
| `ortools` | `sudo apt install ros-humble-ortools-vendor` —— 车上 **apt 是通的**（GitHub 不通但 ROS 镜像通）。装好后 `find_package(ortools_vendor QUIET)` + `find_package(ortools CONFIG QUIET)` 命中 `/opt/ros/humble/opt/ortools_vendor/lib/cmake/ortools/ortoolsConfig.cmake`，**跳过下载** |
| `json`（nlohmann） | 用 `FETCHCONTENT_SOURCE_DIR_JSON` 指向本地源码 |
| `matplot`（matplotplusplus） | 同上（源码 90 MB） |
| `steering_functions` | 同上 |

依赖源码从开发机的 `build/fields2cover/_deps/{json,matplot,steering_functions}-src`
同步而来（**排除 `.git`**，444 MB → **102 MB**），放在车上 `/home/car/ros2/f2c_deps/`。

#### 4.7.2 ⚠️ `defaults.yaml` 必须放在 `~/.colcon/`，**不是** `<工作区>/.colcon/`

**这是本次实际踩到的坑**：先写到了 `<工作区>/.colcon/defaults.yaml`，**colcon 根本不读它** ——
实际命令里没有任何 `-D` 参数，于是 FetchContent 照旧去 clone GitHub，**卡死 3 分钟、0 个文件被编译**，
表面看像"编译很久"，实为纯等待。

依据 `colcon_core/command.py:159-162`：

```python
set_default_config_path(path=(Path('~') / f'.{command_name}').expanduser(), ...)
#  → ~/.colcon/defaults.yaml
```

正确内容（车上已配好）：

```yaml
# ~/.colcon/defaults.yaml
build:
  cmake-args:
    - -DFETCHCONTENT_SOURCE_DIR_JSON=/home/car/ros2/f2c_deps/json
    - -DFETCHCONTENT_SOURCE_DIR_MATPLOT=/home/car/ros2/f2c_deps/matplot
    - -DFETCHCONTENT_SOURCE_DIR_STEERING_FUNCTIONS=/home/car/ros2/f2c_deps/steering_functions
```

**验证是否真的生效**（不要只看文件存在）：

```bash
grep "Invoking command" <ws>/log/latest_build/fields2cover/command.log | tr ' ' '\n' | grep FETCHCONTENT
# 必须打印出那三条 -D，否则没生效
```

> 这是 CMake 官方支持的机制，**不需要修改 Fields2Cover 源码** —— 与仿真侧保持同一份代码。
> 若将来车上有外网了，删掉 `~/.colcon/defaults.yaml` 即可让 F2C 自行下载。

#### 4.7.3 车上环境与开发机的差异

| 项 | 开发机 | **车上（NUC）** |
|---|---|---|
| cmake | 4.3.2（`/usr/local`） | **3.22.1**（系统）—— 对 ROS 2 Humble 兼容性更好 |
| 速度上限 | 0.25 / 0.10 m/s | **0.4 / 0.25 m/s**（用户实车调参，勿覆盖） |
| 地图默认路径 | 工作区相对 | `/home/car/...` 硬编码（**在车上是对的**） |
| GitHub | 通 | **不通** |
| `pointcloud_to_laserscan` | 已装 | 已装 |
| 旧 `opennav_coverage` 布局 | — | 曾是**单包 + 顶层 msgs**（旧 humble 版），迁移时已删除 |

#### 4.7.4 同步步骤（可复现）

```bash
# 1) 车上：备份并删除旧源码（旧文件已被 git 跟踪，可 git checkout 还原）
ssh car@172.20.10.5
cd /home/car/ros2/new_base_plate_indoor_ws
tar czf ~/backup_coverage_migration_$(date +%Y%m%d_%H%M).tar.gz \
    src/Fields2Cover-1.2.1 src/opennav_coverage src/opennav_coverage_msgs
rm -rf src/Fields2Cover-1.2.1 src/opennav_coverage src/opennav_coverage_msgs
rm -rf build/fields2cover build/opennav_coverage build/opennav_coverage_msgs \
       install/fields2cover install/opennav_coverage install/opennav_coverage_msgs

# 2) 开发机：同步源码与文档（-z 压缩，实测 22× 加速比）
rsync -az src/ car@172.20.10.5:/home/car/ros2/new_base_plate_indoor_ws/src/
rsync -az doc/ car@172.20.10.5:/home/car/ros2/new_base_plate_indoor_ws/doc/

# 3) 车上：装 F2C 的 ortools 依赖
sudo apt install -y ros-humble-ortools-vendor swig

# 4) 开发机：同步 F2C 的 FetchContent 依赖源码（排除 .git）
rsync -a --exclude=.git build/fields2cover/_deps/json-src/              .../f2c_deps/json/
rsync -a --exclude=.git build/fields2cover/_deps/matplot-src/           .../f2c_deps/matplot/
rsync -a --exclude=.git build/fields2cover/_deps/steering_functions-src/ .../f2c_deps/steering_functions/

# 5) 车上：配 ~/.colcon/defaults.yaml（见 §4.7.2）后构建
cd /home/car/ros2/new_base_plate_indoor_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to opennav_coverage wheeltec_nav2 nav2_mission
```

> ⚠️ **不要用 `rsync -av src/` 直接覆盖**：车上原本的 `opennav_coverage` 是**单包 + 顶层 msgs**
> 的旧布局，而新版是 **monorepo**。直接覆盖会合并出重复包名导致 colcon 报错，
> **必须先删旧目录**。
>
> ⚠️ **不要覆盖车上的 `start_indoor_navigation.sh`**：它含用户的实车调参（0.4/0.25 m/s）
> 与车上正确的硬编码地图路径。本次只**外科式新增了一行** `use_coverage`。

---

## 5. 参数表（按车辆真值重算）

### 5.1 `coverage_server` 参数

| 参数 | 原值 | **新值** | 依据 |
|---|---|---|---|
| `coordinates_in_cartesian_frame` | `true` | `true` | map 坐标系笛卡尔多边形，不涉及 UTM |
| `robot_width` | 0.53 | **0.68** | footprint 宽度 0.68 m（Q13a） ✅ 不变 |
| `operation_width` | 1.0 | **0.60** ⚠️ **A-4 修正（原定 0.80 是错的）** | 见 §5.1.2。0.80 > 车宽 0.68 → **0.12 m 漏扫带** |
| `min_turning_radius` | 0.25 | **0.95** ⚠️ **实测值** | 实车测量（车体中心画圆）。**不要用几何公式反推** —— 见 §5.1.3 |
| `default_headland_type` | 未设（编译默认 `CONSTANT`） | **显式写 `CONSTANT`** | 消除隐式依赖 ✅ 不变 |
| `default_headland_width` | 0.40 | **1.30** ⚠️ **A-4 修正（原定 0.70）** | `R + 0.10` 那条规则是错的；地头宽度应由**转弯外冲量**决定。见 §5.1.2 |
| `default_path_continuity_type` | `DISCONTINUOUS` | **`CONTINUOUS`** | Q18(a)：阿克曼车不能原地转向，必须连续 Dubins 路径 ✅ |
| `default_route_type` | `BOUSTROPHEDON` | `BOUSTROPHEDON` | 不变 |
| `default_swath_angle_type` | `SET_ANGLE` | `SET_ANGLE` | Q17(a)：固定角度，保证可复现 |
| `default_swath_angle` | 1.57 | 1.57（**现场可能需调 0.0**） | 取决于房间长边方向；若路径方向不理想改 0.0 再试 |
| `default_swath_type` | `NUMBER` | `NUMBER` | 固定角度下该目标函数不参与寻优，无影响 |
| `default_path_type` | `DUBIN` | **`REEDS_SHEPP`** ⚠️ **A-4 修正** | 允许掉头时倒车（三点掉头），把地头需求从 2.80 m 降到 1.30 m。见 §5.1.5 |
| `default_turn_point_distance` | 0.05 | 0.05 | 路径离散化；保持仿真值 |
| `default_allow_overlap` | `false` | `false` | 重叠已通过 `operation_width` 体现 |
| `linear_curv_change` | 1.0 | **200** ⚠️ **A-4 修正（原定"保持 1.0"是错的）** | 见 §5.1.1。1.0 使转弯半径膨胀 1.87 倍 |

#### 5.1.2 `operation_width` 与 `default_headland_width` 的 A-4 修正

**① `operation_width` 原定 0.80 是错的 —— 它会造成漏扫，不是重叠**

F2C 用 `robot.getCovWidth()`（= `operation_width`）确定条带间距（`fields2cover.cpp:111`），
而车体实际宽度是 `robot_width` = 0.68 m。**0.80 > 0.68 → 相邻两趟之间留 0.12 m 未覆盖带。**

文档初版 Q15(b) 写的"0.80 约 25% 重叠"**方向反了**。要实现真正的 25% 重叠应为
`0.68 × 0.75 = 0.51`；要实现零间隙则为 `0.68`。

| `operation_width` | 相对车宽 0.68 | 含义 |
|---|---|---|
| 0.80 | +18% | ❌ **0.12 m 漏扫带** |
| **0.60**（本次取值） | −12% | ✅ 12% 重叠，无漏扫，条带数适中 |
| 0.51 | −25% | ✅ 真正的 25% 重叠，但条带更多、路径更长 |

> **本次取 0.60**（12% 重叠）作为平衡值。**若定位/跟随误差较大，改成 0.51 更稳**（路径会变长）。

**② `default_headland_width` 的 `R + 0.10` 规则是错的**

初版写"地头宽度必须 ≥ 最小转弯半径 + 余量"，所以 R=0.60 → 0.70。
**实测证明这个规则不够**：地头要装下的不是半径，而是**整个转弯的外冲量**，
而外冲量取决于 `operation_width` 与 `2 × 转弯半径` 的关系。

A-4 实测（`operation_width` 0.68、`min_turning_radius` 0.52、`linear_curv_change` 200）：

| `default_headland_width` | **冲出田块的量** |
|---|---|
| 0.70（初版值） | ❌ **0.408 m** |
| 1.00 | ⚠️ 0.108 m |
| **1.30**（本次取值） | ✅ **0.000 m** |
| 1.60 | ✅ 0.000 m |

> 地头本身**也会被覆盖**（F2C 会为地头单独生成条带），所以加宽地头不是浪费作业面积。
> 而且加宽后路径反而**更短**（326 m vs 369 m），因为转弯更少更简单。

#### 5.1.3 ⚠️ `min_turning_radius`：实测 0.95 m，几何公式反推是错的

**实测值：0.95 m**（车体中心/后轴中心画出的圆，半径）。

**不要用 `R = 轴距 / tan(舵角)` 反推。** 按 `max_steering_angle_deg = 30°` 与轴距 0.30 m
算得 0.52 m，但实测是 0.95 m —— **差了 1.83 倍**。原因不是公式错，而是**参数与实际不符**：

```
tan(δ_实际) = 轴距 / R_实测 = 0.30 / 0.95 = 0.3158
→ δ_实际 ≈ 17.5°        （而参数里配的是 30°）
```

**为什么配错角度会导致 1.83 倍的半径偏差** —— 读 `ackermann_steering.hpp` 得到：

| 位置 | 作用 |
|---|---|
| `steeringAngleFromTwist:26-28` | `max_steering_angle` 作为自行车模型的**钳位** |
| `servoAngleFromSteering:42-51` | `max_steering_angle` 作为舵机的**满量程**：`normalized = |δ_cmd| / max_steering_angle`，线性映射到舵机行程 |

所以 `max_steering_angle_deg = 30°` 意味着"命令 30° → 舵机走满行程"。
但舵机走满行程时，前轮只到**机械极限 17.5°**。即：

```
δ_实际 = δ_命令 × (17.5 / 30) = δ_命令 × 0.583
```

**后果**：命令 30° 只得到 17.5°，实际半径 0.95 m 而预期 0.52 m ——
**整个控制系统（MPPI / Smac / F2C）都在按一个车做不到的转弯能力做规划**，
表现为**系统性转向不足 1.71 倍**（切内弯、跟丢路径）。

**修正（两处必须同时改，否则不自洽）**：

| 文件 | 参数 | 原值 | **新值** |
|---|---|---|---|
| `coverage_params.yaml` | `min_turning_radius` | 0.52 | **0.95** |
| `param_mini_diff_indoor.yaml:130` | MPPI `AckermannConstraints.min_turning_r` | 0.52 | **0.95** |
| `param_mini_diff_indoor.yaml:316` | Smac `minimum_turning_radius` | 0.52 | **0.95** |
| `auto_control.launch.py:38`、`indoor_amcl.launch.py:188` | `max_steering_angle_deg` | 30.0 | **17.5** |

**为什么 `max_steering_angle_deg` 必须一起改**：MPPI 按 `min_turning_r = 0.95` 最多命令 δ=17.5°；
若该参数仍是 30°，舵机只给到 17.5/30 = 58%，实际前轮 10.2°，**实际 R 变成 1.67 m** ——
仍然不自洽，只是偏差换了方向。

> **安全性**：本改动**不引入新的机械极限** —— 舵机限位（`SERVO_LEFT_LIMIT 40` /
> `SERVO_RIGHT_LIMIT 190`）不变，最大物理转角不变；改的只是"命令 → 转角"的增益。
>
> **⚠️ 上车验证**：命令一个已知的角速度（如 `ros2 topic pub /cmd_vel` 给 v=0.1, ω=0.105，
> 即期望 R≈0.95），量实际画出的圆半径是否 ≈ 0.95 m。若明显偏离，说明线性映射假设不成立
> （转向机构非线性），需要做分段标定。

#### 5.1.4 ✅ A-4 得出的设计规则（比逐参数试错更可靠）

A-4 扫描 15 组参数后，越界量的规律完全清楚：

> ### **越界 = 0 的充要条件是 `operation_width ≥ 2 × 实际转弯半径`**
>
> 且当 `linear_curv_change ≥ 50` 时，**实际转弯半径 ≈ `min_turning_radius`**。

| `operation_width` | 转弯 R | `2R` | 满足 `op ≥ 2R`? | 实测越界 |
|---|---|---|---|---|
| 0.80 | 0.400 | 0.800 | = | **0.000** |
| 1.04 | 0.520 | 1.040 | = | **0.000** |
| 1.20 | 0.600 | 1.200 | = | **0.000** |
| 0.68 | 0.411 | 0.822 | 0.68 < 0.822 | 0.009 |
| 0.80 | 0.538 | 1.076 | ✗ | 0.310 |
| 0.80 | 0.621 | 1.242 | ✗ | 0.566 |
| 0.80 | 1.121 | 2.242 | ✗ | **2.549** |

**当 `op < 2R` 时，靠加宽地头来吸收外冲量**（本次即为这种情况：
`op = 0.60 < 2 × 0.545 = 1.09`，故地头取 1.30）。

**最终推荐组合（用实测 R=0.95 复算，A-4 实测越界 = 0.000）**：

```yaml
robot_width: 0.68
operation_width: 0.60          # < 车宽 → 无漏扫；12% 重叠
min_turning_radius: 0.95       # 【实测值】
linear_curv_change: 200        # ≥ 50 即可
default_headland_width: 1.30   # 吸收转弯外冲
default_path_type: "REEDS_SHEPP"   # 见 §5.1.5
default_path_continuity_type: "CONTINUOUS"
```
→ 实测：**越界 0.000 m**，路径长 371 m
→ 真实耗时 ≈ 371 / 0.25 = **1484 s ≈ 24.7 min** < `follow_timeout_sec` 3600 ✅

#### 5.1.5 `default_path_type`：改用 `REEDS_SHEPP` 把地头需求从 2.80 m 降到 1.30 m

实测 R=0.95 后，`2R = 1.90 m`，而 `operation_width` 必须 ≤ 0.68（否则漏扫）——
**1.90 ≫ 0.68，靠条带间距容下掉头已完全不可能**，只能靠地头吸收。

A-4 实测（op=0.60、lin_curv=200）：

| `default_path_type` | 地头 | 越界 | 路径长 | 真实耗时 |
|---|---|---|---|---|
| `DUBIN` | 1.30 | ❌ 1.048 m | 397 m | — |
| `DUBIN` | 1.80 | ⚠️ 0.548 m | 341 m | — |
| `DUBIN` | **2.80** | ✅ 0.000 | **301 m** | 20.1 min |
| `REEDS_SHEPP` | 1.00 | ✅ 0.000（临界） | 402 m | 26.8 min |
| **`REEDS_SHEPP`** | **1.30**（本次取值） | ✅ **0.000** | 371 m | **24.7 min** |
| `REEDS_SHEPP` | 1.80 | ✅ 0.000 | 318 m | 21.2 min |

**原因**：`DUBIN` 只能前进，掉头时转不过来就**向外绕大回环**，需要很大空间；
`REEDS_SHEPP` 允许倒车，可以用**三点掉头**在小空间内完成，于是地头需求大降。

> **取舍**：`DUBIN + 地头 2.80` 路径最短（301 m / 20 min）且**全程前进**，但需要 2.8 m 地头；
> `REEDS_SHEPP + 地头 1.30` 只需 1.3 m 地头，代价是路径长 5 分钟且**掉头段要倒车**。
> 本次取后者（1.3 m 地头对实际场地更容易满足）。
> **若现场地头空间充裕且希望全程不倒车 → 改回 `"DUBIN"` 并把 `default_headland_width` 提到 2.80。**
>
> ⚠️ 倒车速度受 `vehicle_can_bridge` 的 `max_reverse_speed = 0.10 m/s` 限制（§8.8），会慢。

#### 5.1.1 三个参数的语义（读源码查实，非推测）

**① `linear_curv_change` = 曲率变化率上限（转向速率约束），不是"曲率上限"**

它经 `robot_params.hpp:59` 传到 `Robot::setMaxDiffCurv()`，再被传给 steering_functions 的
`CC00_Dubins_State_Space(max_curv, max_curv_change_rate, ...)`（见 `dubins_curves_cc.cpp:32-36`）。
含义：**值越大 → 允许曲率变化越快 → 转弯越急；值越小 → 要求越平缓的过渡弧 → 转弯越宽。**

| 来源 | 值 | 说明 |
|---|---|---|
| 仿真（turtlebot3_ws） | 200.0 | TurtleBot3 是差速车，该约束对它无意义，200 相当于"关掉" |
| F2C 库自身默认 | 0.3 | `Types/Robot.h:28` |
| opennav 代码默认 | 2.0 | `robot_params.hpp:58` |
| 本工作区旧值 | 1.0 | ❌ **实测证明这个值不可用**，见下方修正 |
| ✅ **A-4 实测推荐** | **≥ 50，200 也可以** | 见下 |

> ### ❌ 修正：本方案初版建议"保持 1.0"是**错的**（A-4 实测推翻）
>
> 初版的推理是"1.0 偏保守 → 转弯更平缓 → 更安全"。**实测结果正好相反。**
>
> A-4 参数扫描（18×14 m 矩形，闭合多边形，其余参数取 §5.1 值）：
>
> | `linear_curv_change` | 实际转弯半径 | **冲出田块的距离** | 路径长 |
> |---|---|---|---|
> | **1.0**（本方案初版保留值） | 1.121 m | ❌ **2.549 m** | 410 m |
> | 2.0（opennav 代码默认） | 0.818 m | ❌ 1.429 m | 364 m |
> | **200**（仿真值） | **0.621 m** | ✅ **0.566 m** | 331 m |
>
> **规律**：`linear_curv_change = 200` 时，F2C 实际生成的**转弯半径恰好等于 `min_turning_radius`**
> （0.40→0.400、0.52→0.520、0.60→0.600，三组完全吻合）。值越小，转弯半径被强制放大得越多：
> 取 1.0 时半径膨胀到设定值的 **1.87 倍**（0.60 → 1.121），转不过来只能向外绕大圈。
>
> **物理解释**：该参数限制的是**曲率变化率**，值太小意味着"方向盘只能极缓慢地打"，
> 于是从直线切到圆弧需要一段很长的过渡弧 —— 转弯被摊得很宽。
> 物理合理量级 = `κ_max / (T_舵机 · v) = (1/0.95)/(T·0.25) = 4.21/T`（κ_max 按实测 R=0.95 m）：
>
> | 舵机从回正打到满舵耗时 | 物理量级 |
> |---|---|
> | 0.3 s | ≈ 14 |
> | 0.5 s | ≈ 8.4 |
> | 1.0 s | ≈ 4.2 |
>
> 即物理上是 **4~14**（比按 R=0.52 估算时的 8~26 更低，因为 R 更大、κ_max 更小）。
> 但取 **≥ 50** 仍有充分余量，**200 也可用**（过渡弧短到可忽略）。
>
> 即物理上是 **8~26**，取 **≥ 50** 已足够（过渡弧短到可忽略），**200 等于假设舵机瞬时到位**。
>
> **结论**：`linear_curv_change` 应设为 **≥ 50**（推荐先取 **200**，与仿真一致，
> 待阶段 B2 实测舵机速率后再收紧到物理值）。**它比 `min_turning_radius` 更能决定路径是否冲出作业区。**
>
> 注：本节②③的结论（`*_mode` 不设即走节点参数、`coverage_params.yaml` 是唯一真源）**不受影响，仍然成立**。

**② `headland_mode` / `swath_mode` / `path_mode` 不设 = 走节点参数（已验证）**

桥接节点只设 `generate_*` / `use_gml_file` / `frame_id` / `polygons`，**不设**各 `*_mode` 字段，
因此 goal 里它们全是 `"UNKNOWN"`。三个生成器都有同一套回退逻辑：

| 生成器 | 回退条件 | 回退到 |
|---|---|---|
| `headland_generator.cpp:30-37` | `mode == UNKNOWN` | `default_headland_type` / **`default_headland_width`** |
| `swath_generator.cpp:32-38` | `objective` 与 `mode` **都** UNKNOWN | `default_swath_angle_type` / `default_swath_angle` / `default_swath_type` |
| `path_generator.cpp:33-39` | `mode` **或** `continuity_mode` UNKNOWN | `default_path_type` / **`default_path_continuity_type`** / `default_turn_point_distance` |

> **这解答了一个必须查清的陷阱**：`HeadlandMode.msg` 的 `width` 默认值是 **2.0**（不是 0），
> 若服务端直接采用它，本文件的 `default_headland_width`（当时为 0.70）就会被静默忽略、地头变成 2.0 m。
> 实测源码确认：因为 `mode == "UNKNOWN"` 走的是**回退分支**，`settings.width` 根本不会被读。
>
> **结论：`coverage_params.yaml` 是覆盖行为的唯一真源（single source of truth）。**
> 要调覆盖行为，改这个文件；不需要（也不应该）去改桥接节点的 goal 构造。

**③ 服务端另有 28 个参数**，文档只列了影响覆盖结果的 15 个。其余（`default_tsp_*`、`default_decomp_*`、
`default_reduce_path`、`default_spiral_n`、`max_swaths_for_global_route` 等）保持代码默认值，
未列入本表。完整清单见 `coverage_server.cpp` 与 `robot_params.hpp` 的 `declare_parameter_if_not_declared`。

### 5.2 桥接节点参数

> ### ⚠️ 文档初版指错了文件（已修正）
>
> 初版把本节标题写成 `coverage_navpath_bridge_node.py`。但该节点由
> `nav2_mission/launch/coverage_navpath_bridge.launch.py` 启动，而**该 launch 用 `DeclareLaunchArgument`
> 声明了同名参数并显式传入节点** —— launch 参数的值**覆盖**节点里 `declare_parameter` 的默认值。
>
> **只改 `.py` 里的默认值不会产生任何效果。** 两处都要改（本次已同时改，保持一致）。

**改动的两个文件及其分工：**

| 文件 | 作用 | 说明 |
|---|---|---|
| `src/nav2_mission/launch/coverage_navpath_bridge.launch.py` | **实际生效值** | `DeclareLaunchArgument` 的 `default_value` |
| `src/nav2_mission/nav2_mission/coverage_navpath_bridge_node.py` | 代码默认值 | 仅在不经该 launch 直接 `ros2 run` 时生效。**必须与 launch 保持一致** |

| 参数 | 原值 | **新值** | 依据 |
|---|---|---|---|
| `follow_timeout_sec` | **300.0** | ✅ **3600.0** | ⚠️ 见下方语义确认 |
| `compute_timeout_sec` | 60.0 | ✅ **120.0** | 与仿真侧 `planning_timeout_sec: 120.0` 对齐 |
| `generate_headland` | `false` | ✅ **`true`** | 阿克曼车掉头需要地头空间 |
| `controller_id` | `FollowPath` | `FollowPath` | 不变（真车 `FollowPath` = MPPI + Ackermann，正确） |
| `base_frame_id` | `base_link` | `base_link` | 不变 |

#### 5.2.1 `follow_timeout_sec` 语义确认（初版的未决项，已查实）

初版标注"语法待确认：是总执行超时，还是等待 action server 的超时？"—— **已读源码确认，是前者，也就是最坏的那种。**

`coverage_navpath_bridge_node.py:317-323`：

```python
follow_result_future = follow_goal_handle.get_result_async()
wait_status = self._wait_future(
    follow_result_future,
    self._follow_timeout_sec,          # ← 就是这里
    goal_handle,
    cancel_fn=lambda: self._cancel_goal(follow_goal_handle),   # ← 超时会主动取消
)
```

FollowPath 要**跑完整条覆盖路径才返回结果**，所以这个超时就是**覆盖任务的总执行时长上限**。
而且超时分支带 `cancel_fn` —— 超时不是"干等"，而是**主动取消 FollowPath，车直接停在半路**。

**200–300 m² 场地按 swath 0.8 m 估算：纯行驶约 1000–1500 s，加掉头约 1300–2000 s（22–33 分钟）。**
原值 300 s（5 分钟）**必然**在覆盖跑到约 1/5 时取消任务。这是本方案中风险最高的单个参数，
现已从"待确认"升级为**确认成立并已修复**。

**生效性验证**（`ros2 launch nav2_mission coverage_navpath_bridge.launch.py --show-args`）：

```
'compute_timeout_sec':   (default: '120.0')
'follow_timeout_sec':    (default: '3600.0')
'generate_headland':     (default: 'true')
```

#### 5.2.2 `task_time` 不能直接当作真实耗时读（A-4 实测发现）

coverage_server 返回的 `result.task_time` **是按 1.0 m/s 算出来的，不是本车的 0.25 m/s**。

原因：F2C 的 `Robot` 里巡航速度有默认值，而 opennav 的参数封装**从不设置它**：

```cpp
// Fields2Cover/include/fields2cover/types/Robot.h
/// Velocity of the robot when not doing turns.
double cruise_speed_ {1.0};        // ← 默认 1.0 m/s
```

`robot_params.hpp:45-59` 只设置了 `robot_width` / `operation_width` / `min_turning_radius` /
`linear_curv_change`，**没有** `setCruiseVel()` 或 `setTurnVel()`。
而 `task_time` 的定义是 `Σ(state.len / state.velocity)`（`Path.cpp:54-57`），
速度取的就是这个默认的 1.0。

**换算方法**（本车实际 0.25 m/s）：

```
真实耗时 ≈ task_time × (1.0 / 0.25) = task_time × 4
```

> **顺带一个有用的事实**：因为除数是 1.0，`task_time` 在数值上**就等于路径总长度（米）**。
> 想快速估路径长度，直接读 `task_time` 即可。

**实测（18×14 m = 252 m² 矩形，operation_width 0.80）**：

| 量 | 值 |
|---|---|
| `task_time`（F2C 报告值） | **410.2 s** |
| 折算路径长度 | ≈ **410 m** |
| **按 0.25 m/s 的真实耗时** | **≈ 1641 s ≈ 27.3 分钟** |
| 文档 §5.2 的原估算 | 22–33 分钟 ✅ **吻合** |

**对 `follow_timeout_sec: 3600.0` 的结论**：27.3 min = 1641 s < 3600 s，
**余量约 2.2×，够用**。即使平均速度掉到 0.2 m/s（2075 s）仍在限内。
反过来说：若当初保持 300 s，实测会跑到约 18% 处就被取消 —— 原判断成立。

> ⚠️ **注意：车上的实际速度与本文默认值不同**
>
> 车（NUC）上的 `start_indoor_navigation.sh` 把速度上限**有意调高**为
> `max_forward_speed: 0.4` / `max_reverse_speed: 0.25`（本工作区/本文其余部分用的是 0.25/0.10）。
> 折算真实耗时要用**车上实际生效的速度**：
>
> | 速度 | 252 m² 场地（`task_time` = 371.1 s）真实耗时 |
> |---|---|
> | 0.25 m/s（本文默认） | 1484 s ≈ **24.7 min** |
> | **0.4 m/s（车上实际）** | 928 s ≈ **15.5 min** |
>
> 两种情况下都远小于 `follow_timeout_sec = 3600`，**该参数无需调整**。
> 但如果你以后把速度调得更高，记得按 `task_time ÷ v` 重算。

### 5.3 Nav2 参数

| 文件 / 参数 | 原值 | **新值** | 依据 |
|---|---|---|---|
| `param_mini_diff_indoor.yaml:274` **全局** costmap `inflation_radius` | 0.55 | ✅ **0.60** | **0.55 < 车体外接圆半径 0.556**。`SmacPlannerHybrid` 按"点机器人 + 膨胀层"工作、不使用 footprint，因此规划器会认为车能贴得比物理允许更近 —— 车的**角会插进障碍**。中等障碍密度场地上这必然发生 |

> **该文件里有三处 `inflation_radius`，只有一处是本项改动目标**（改错地方不会有任何效果）：
>
> | 行号 | 所属 | 值 | 是否修改 |
> |---|---|---|---|
> | `:157` | MPPI `ObstaclesCritic`（控制器 critic 参数） | 0.45 | ❌ 未动 |
> | `:227` | **local** costmap `inflation_layer` | 0.45 | ❌ 未动 |
> | `:274` | **global** costmap `inflation_layer` | 0.55 → **0.60** | ✅ 本次改动 |
>
> **诚实说明这项改动的实际作用范围**：覆盖路径由 F2C **几何生成**，不是 `SmacPlannerHybrid` 规划的，
> 随后交给 **MPPI**（`FollowPath`）跟随；MPPI 走 **local** costmap，且其 `ObstaclesCritic`
> 已设 `consider_footprint: true`（真做 footprint 碰撞检查）。
>
> 所以全局膨胀半径**并不直接约束覆盖路径**，它影响的是：
> ① 普通点到点导航与恢复行为（走 Smac）；② 全局 costmap 的一致性。
> 本次仍按 §5.3 修正，因为 0.55 < 0.556 是**真实的几何不一致**，且改动无副作用。
>
> ⚠️ **需注意的权衡**：把膨胀半径提到外接圆半径，等于让规划器假设"车可能处于任意朝向"，
> 这比真实 footprint 检查**更保守** —— 若场地有宽度介于 1.10–1.20 m 的窄通道，
> Smac 可能报无路径（而车直着开其实过得去）。这是保守方向的失效，安全但可能挡住导航。
> 若实地遇到，可回调到 0.56（刚好覆盖外接圆）缓解。

---

## 6. 实施阶段

### 阶段 A — 开发机上完成（不碰车，可与定位加固并行）

> 依据 Q22(b)。以下全部可以在 `/home/wyz` 上做完并验证。

1. **A1** 按 §3.4 完成构建修复与版本统一，`colcon build` 通过（rc=0）
2. **A2** 验证消息版本正确：`ros2 interface show opennav_coverage_msgs/msg/DecompMode` 应**成功**（`DecompMode` 只存在于 `humble-v2`；这是判断版本是否切对了的**一行判定**）
3. **A3** 按 §5.1 重写 `coverage_params.yaml`
4. **A4** 按 §5.2 修改桥接节点参数
5. **A5** 按 §5.3 修正全局 `inflation_radius`
6. **A6** 离线验证覆盖路径（**不启动车**）：
   ```bash
   ros2 launch wheeltec_nav2 nav2_indoor.launch.py map:=<绝对路径>/map.yaml use_rviz:=true
   # RViz: 用 2D Pose Estimate 设定初始位姿 → 用 Publish Point 点选多边形
   ros2 action send_goal /compute_coverage_path opennav_coverage_msgs/action/ComputeCoveragePath \
     "{generate_headland: true, generate_route: true, generate_path: true, frame_id: 'map', polygons: [...]}"
   ```
   检查：路径闭合、无自交、**每个掉头圆弧半径 ≥ 实测/设定最小转弯半径**、地头内完成掉头、**不越出所选多边形**
7. **A7** 修正 §4.3 的 launch/脚本/文档
8. **A8** 补接覆盖链，加 `use_coverage` 按需启动开关（§4.3.1）
9. **A9** 实现安全层：遥控抢占订阅 + 软件急停（§4.4）

> **以上 A1–A9 已全部执行完毕**（A1–A5 见 §3.8 / §5；A6 即 §7 的 A-4 离线几何校验；
> A7 见 §4.3；A8 见 §4.3.1；A9 见 §4.4）。**阶段 A 剩余项**：无。
> 执行中另行发现并修复的问题见 §3.6/§3.7（构建阻塞）、§4.3.2（缺失依赖）、§4.6（rosdep 键）。

**阶段 A 的通过标准**：生成的覆盖路径在几何上完全合理，掉头圆弧不越出作业区域边界。

> ⚠️ **实际执行时把这一步做成了"数值测量"而非"RViz 目测"** —— 起 `coverage_server`
> 直接发 `ComputeCoveragePath`，把结果里的 `coverage_path`（条带/转弯分离）与 `nav_path`
> 取回来做几何计算。比肉眼在 RViz 里看严格得多，而且可复现、可回归。
> 脚本见 `$CLAUDE_JOB_DIR/tmp/a4_*.py|sh`（本机路径，未纳入仓库）。

### 阶段 B — 上车前的阻塞性实测（Q23b）

> **这两项不完成，不要开始覆盖。**

1. **B1 实测最小转弯半径**：把车放空地，舵机给到最大转角，低速画一个完整圆，量直径 ÷ 2。**这个数字决定了 `min_turning_radius` 和 `headland_width` 两个关键参数。**
2. **B2 实测舵角标定**：分别标定左/右最大前轮转角是否对称、舵机命令角与实际前轮角的对应关系。`doc/IndoorNav_Issues.md` §4 把这些列为"尚未确认"。
3. **B3** 用 B1/B2 结果回填 §5.1 的 `min_turning_radius` 与 `default_headland_width`（`headland = R + 0.10`）

### 阶段 C — 上车联调（按 Q20(a)：直接上目标场地）

> 📋 **现场执行请用单独的可打印清单：`doc/发车前检查清单.md`**
> （桌面也放了一份：`~/桌面/发车前检查清单.md`）
> 那份只列要做的事、带命令和期望值，不需要读本文；原理与依据留在本文。

由于**没有分级小场地**，首次运行就是集成测试。因此**阶段 A 和 B 必须做得足够扎实**，把能在开发机上验的全部验掉。

1. **C1** 按 `INDOOR_AMCL_RUNBOOK.md` 启动整栈，确认 `/scan`、`/odometry/filtered_odom` 正常，TF 链 `map → odom → base_link` 连续
2. **C2** 设一次初始位姿，确认激光与地图重合（这是 Q21(a) 的前提，**若发现不重合则立即停止，不要继续**）
3. **C3** 在 RViz 里点选**一个很小的区域**（3×3 m 左右），跑第一次覆盖
4. **C4** 逐步扩大区域 → 单房间 → 全场
5. **C5** 全程有人持急停/遥控器（Q14a）

---

## 7. 验证清单

按 **Q7(a) 定性验收** + **Q20(a) 直接上全场**，前置检查必须做厚：

### 阶段 A（开发机）

**A-1 构建与版本（§3 修复项，已全部实测通过 ✅）**

- [x] `colcon build` 对 `opennav_coverage` 返回 **rc=0**
- [x] `ros2 interface show opennav_coverage_msgs/msg/DecompMode` 成功（版本判定）
- [x] `build/opennav_coverage/CMakeFiles/opennav_coverage_core.dir/flags.make` 的 `CXX_INCLUDES` 中**出现 fields2cover 的 include 路径**（根因是否真正消除的直接证据）
- [x] `CMakeCache.txt` 的 `Fields2Cover_DIR` 指向**本工作区**而非 turtlebot3_ws（§3.8）
- [x] `import fields2cover` 版本为 **2.0.0**
- [x] 全依赖闭包 6 个包 + 5 个受 §3.7 影响的包全部构建通过

**A-2 参数改动（对应 §5，已全部执行 ✅）**

- [x] `coverage_params.yaml` 按 §5.1 重写完毕（15 个参数，YAML 解析通过）
- [x] 桥接节点参数按 §5.2 修改 —— **两处都改了**：launch 参数默认值（生效值）+ 节点代码默认值
- [x] 全局 `inflation_radius` 按 §5.3 改为 0.60（只动 `:274` 的 global 那处）
- [x] 生效性验证：`ros2 launch ... --show-args` 显示 `120.0 / 3600.0 / true`
- [x] 重建通过（6 个包，rc=0）

**A-3 补接覆盖链（§4.3.1，✅ 已实施并实测）**

- [x] 定为「按需启动」，新增 `use_coverage` 开关（默认 `false`）
- [x] `coverage_navpath_bridge.launch.py` 接入 `navigation_with_coverage_launch.py`（条件 include）
- [x] `coverage_region_selector.launch.py` 接入同上
- [x] 实测对照：`use_coverage:=true` 两个节点启动并就绪；`false` 时都不启动
- [ ] **待补**：起栈后用 `ros2 action list | grep execute_coverage_navpath` 与
      `ros2 topic info /clicked_point` 做一次运行期确认（本次只从节点日志确认了
      "ready" 与 action 名一致，未独立查订阅计数）
- [x] 记录已知取舍：`coverage_server` 未被 gate（原因见 §4.3.1）

**A-4 离线 RViz 路径复核（不启动车，**尚未执行**）**

**A-4 已执行（不是靠肉眼在 RViz 里看，而是把 action 结果取回来做数值测量）**

方法：起 `coverage_server` → 发 `ComputeCoveragePath` 目标（18×14 m = 252 m² 闭合矩形）
→ 取回 `coverage_path`（条带/转弯分离）+ `nav_path` → 用 numpy 量几何。
脚本：`$CLAUDE_JOB_DIR/tmp/a4_check.py`、`a4_one.py`、`a4_run.sh`、`a4_sweep.sh`。

- [x] 路径连续无断点：最大点间距 0.11 m，**>1 m 的跳变 0 处**
- [x] 覆盖带间距精确生效
- [x] 地头精确生效
- [x] 转弯半径 ≥ `min_turning_radius`：`lin_curv ≥ 50` 时实测半径**恰等于** `min_turning_radius`
- [x] **路径不越出多边形** —— 修正参数后实测**越界 0.000 m**
- [x] 顺带实证：`task_time` 数值 == 路径总长度（米），见 §5.2.2

**最终验收（用提交后的 `coverage_params.yaml`，无任何命令行覆盖）**：

```
error_code = 0        planning_time = 0.011 s      task_time = 371.9 s
条带 26 条            转弯 25 段                   nav_path 6628 点
```

**最终验收 —— 用实测 R=0.95 复算后的参数**（`op=0.60`、`R=0.95`、`lin_curv=200`、
`headland=1.30`、`REEDS_SHEPP`，无任何命令行覆盖）：

```
error_code = 0        task_time = 371.1 s → 真实 24.7 min
条带 26 条            转弯 25 段             nav_path 6703 点
```

| 检查项 | 实测 | 设定值 | 判定 |
|---|---|---|---|
| 条带间距 | **0.5996**（min 0.591 / max 0.609） | `operation_width` 0.60 | ✅ |
| 地头内缩 | **1.30 / 1.30**（条带 y ∈ [1.30, 12.70]） | `default_headland_width` 1.30 | ✅ |
| **冲出作业区边界** | **0.0000 m**（nav_path y ∈ [0.404, 13.596]） | — | ✅ |
| 路径连续性 | 最大点间距 0.10 m，>1 m 跳变 **0** 处 | CONTINUOUS | ✅ |
| 路径总长 | 371.1 m（`nav_path` 累加） vs `task_time` 371.1 | — | ✅ 吻合 |
| **真实耗时** | 371.1 / 0.25 = **1484 s ≈ 24.7 min** | `follow_timeout_sec` 3600 | ✅ 余量 2.4× |
| 转弯段形态 | 31 点 / 2.93 m / 外伸至 y=13.60 | 停在田块内 0.40 m | ✅ |

> **与初版参数的对比**（同 18×14 m 场地）：
> 初版（`R=0.60`、`lin_curv=1.0`、`DUBIN`、`headland=0.70`）转弯段 **71 点、外冲到 y=16.55
> （越界 2.55 m）**；修正后 **31 点、只到 y=13.60**。转弯形态从"大回环"变成"紧凑三点掉头"。

**A-4 的关键产出是发现并修正了 §5.1 里 4 个错误参数**（`operation_width`、
`min_turning_radius`、`linear_curv_change`、`default_headland_width`），
并总结出设计规则 `operation_width ≥ 2 × 转弯半径`（否则靠地头吸收外冲）。
详见 §5.1.2 ~ §5.1.4。

**A-4 未覆盖到的（留给阶段 C 现场）**：
- 自交检查、真实房间形状（本次用规则矩形）、障碍物规避 —— 覆盖路径由 F2C 按多边形几何生成，
  **不做避障**；避障由执行层的 MPPI footprint 检查负责
- `default_swath_angle` 是否与实际房间长边一致

**A-5 安全层（§4.4，✅ 已实施并实测）**

- [x] 桥接节点订阅 `/vehicle/remote_manual_override`，为 `true` 时中止任务 + 下发零速
- [x] 软件急停入口 `/coverage/emergency_stop`（`std_msgs/Bool`）
- [x] 抢占期间拒绝新覆盖目标
- [x] 实测：抢占触发 → 日志 `SAFETY ABORT` → 5 帧零速发到 `/cmd_vel` → 任务以
      `error_code = 909 (ERR_SAFETY_ABORT)` 终止
- [x] 顺带修掉一个响应性缺陷：改用轮询式 `_wait_for_action_server()`，
      使"等待 action server 期间"的抢占也能立即收尾（原先会等满超时、错误码变 901）

> #### ⚠️ 直接调 `ComputeCoveragePath` 时，外多边形**必须闭合**（首点 == 末点）
>
> 实测踩到：发 4 个顶点的开口矩形会被拒，报
> `803 INVALID_COORDS: Outer polygon malformed, first element must equal last!`
>
> **走 RViz 点选或 `/execute_coverage_navpath` 时不用管** —— 桥接节点的
> `_normalize_polygon`（`coverage_navpath_bridge_node.py:513-514`）会自动补上闭合点。
> 但写脚本直接调 `/compute_coverage_path` 时必须自己闭合。
>
> #### 另一个坑：同名节点共存会污染验证
>
> 若上一轮的 `coverage_server` 没被杀干净，图里会出现**两个 `/coverage_server`**，
> `ros2 node list` 会警告 "nodes in the graph that share an exact name"，
> 此时 `ros2 param get` 结果不确定、`ros2 lifecycle set` 行为异常。
> **每次验证前先 `ros2 node list` 确认只有一个实例。**

### 阶段 B（车上，静态）

- [ ] 最小转弯半径实测值记录在案，并回填参数
- [ ] 左右最大前轮转角实测，确认对称性

### 阶段 C（车上，动态）

> 📋 **本节的现场执行版已抽成单独文档：`doc/发车前检查清单.md`（桌面同份）。**
> 那里是**权威的可打印清单**，只列要做的事 + 命令 + 期望值，并补充了异常处理速查、
> 参数确认命令、绝对禁止事项。本节保留在此是为了让验证清单完整、便于追溯依据。

**发车前必做（对应 §8.3 的控制权移交风险 + §4.4 安全层）：**

- [ ] 确认 RC 遥控器**模式切换通道处于已知挡位**，摇杆**在中位**，且**有人持握**
- [ ] 确认启动的是 `auto_control.launch.py`（经 `indoor_amcl.launch.py`），**不是 `complete_control.launch.py`**（否则速度钳位是 2.0 m/s）
- [ ] 确认 `/vehicle/remote_manual_override` 有数据在发布（`ros2 topic hz /vehicle/remote_manual_override` ≈ 1 Hz）——
      **这是 §4.4 安全层的前提**：该信号来自 CAN ID `0x07`，CAN 链路静默降级时它会一起消失（R23）
- [ ] 确认安全层已就位：
      ```bash
      ros2 node info /coverage_navpath_bridge | grep -A8 Subscribers   # 应有 /vehicle/remote_manual_override
      ros2 node info /coverage_navpath_bridge | grep -A8 Publishers    # 应有 /cmd_vel
      ros2 topic list | grep coverage/emergency_stop                   # 急停话题应在
      ```
- [ ] **实测抢占**：覆盖任务跑到一半时拨动遥控器 → 桥接节点日志出现
      `SAFETY ABORT: remote manual override` → **任务立即中止且车停下**
- [ ] **实测急停**：`ros2 topic pub --once /coverage/emergency_stop std_msgs/msg/Bool "{data: true}"`
      → 任务立即中止 + `/cmd_vel` 收到零速帧
- [ ] ⚠️ 遥控器接管后**不要拨回"自动"挡**，除非确定要让覆盖继续（§8.3：拨回即重新接受 ROS 指令）

- [ ] `/scan` ≥ 8 Hz；`/odometry/filtered_odom` ≥ 8 Hz
- [ ] `ros2 lifecycle get /coverage_server` = `active`
- [ ] 激光与地图重合（目视：墙面与地图墙线基本重合）
- [ ] 3×3 m 小区域覆盖跑通，车不撞不停
- [ ] 单房间覆盖跑通
- [ ] 全场地覆盖跑通（**预计 22–33 分钟，务必确认 `follow_timeout_sec` 已提高**）
- [ ] 覆盖过程中人为触发遥控器/急停，确认**覆盖任务立即中止且车停下**

---

## 8. 底盘安全语义（已查明）

这些事实决定了 Q14(a) 怎么落地，**动手前必须读懂**。

### 8.1 ROS 侧**没有** cmd_vel 看门狗

`src/vehicle_can_bridge/src/cmd_vel_to_can_node.cpp` 全文**零个 `create_wall_timer`**，`main()` 是裸自旋。唯一的输出路径是 `cmdVelCallback` → `sendCombinedControlCommand`（发布在 `:187`）。因此：

- **没有**超时值、**没有**零速重发、**没有**停车 CAN 帧、**没有** `last_cmd_time_`；
- `/cmd_vel` 停发后**什么都不发**，连上一帧都不重传（不存在"最后指令保持"的 ROS 逻辑）。

### 8.2 唯一的兜底在 STM32 固件

`docs/DEPLOYMENT.md:274` 原文：

> STM32 端有 **500ms 消息超时看门狗**：如果 ROS 侧停止发送控制帧超过 500ms，STM32 会自动退回手动（RC）控制模式。这是一个安全兜底，但不能替代人工确认——部署/联调时应始终有人手持 RC 遥控器、随时准备接管，且确认 RC 模式切换通道的当前位置，不要在不确定 RC 挡位的情况下启动……

### 8.3 ⚠️ 控制权静默移交（**最容易出事的一条**）

`docs/DEPLOYMENT.md:273`：

> STM32 一旦在组合控制 CAN 通道（ID `0x06`）上收到任意一条消息，**如果此时 RC 遥控器的模式切换通道恰好处于"自动"位置**，STM32 就会从手动（RC 遥控）切到自动（ROS 指令）控制模式——也就是说，只要 `cmd_vel_to_can` 节点启动并开始发送（**哪怕是零速度指令**），配合 RC 遥控器处于自动挡，车辆控制权就已经交给 ROS 侧了。

**两个方向都要防**：启动时 RC 若在"自动"挡 → 控制权静默交给 ROS；而 500 ms 看门狗触发后又交回 RC → **此时若摇杆不在中位，车会突然动作**。

### 8.4 `/vehicle/remote_manual_override` —— 信号存在，但**无人订阅**

- 发布者：`can_bridge_node.cpp:99-100`（建 publisher）、`:633`（`remote_manual_override_pub_->publish(...)`），数据来自 CAN ID `0x07` 的 `data[1]`，约 1 Hz；
- 语义（`docs/ARCHITECTURE.md:97,316`）：`true` = **遥控器已手动接管，ROS 指令此刻被固件忽略**；
- **全工作区 `src/` 下没有任何 `create_subscription` 订阅它**。

→ Q14(a) 要挂的信号就是它，但**必须新增订阅节点**。

### 8.5 `/vehicle/enabled` 与 `/vehicle/enable_cmd`

| 话题 | 发布者 | 订阅者 |
|---|---|---|
| `/vehicle/enabled` | `can_bridge_node.cpp:97-98, 627`（`0x07` `data[0]`） | **无** |
| `/vehicle/enable_cmd` | **无** | `can_bridge_node.cpp:111-113, 346-361` |

固件上电**默认启用**（`docs/ARCHITECTURE.md:313`："`0`=禁用、非0=启用（固件上电默认值也是启用）"）。ROS 侧启动时既不发送使能命令，也不知情 —— 只能通过 1 Hz 的 `/vehicle/enabled` 反馈事后观察。

### 8.6 整个包**没有软件急停**

`estop` / `e_stop` / `emergency` / `急停` / `watchdog` 全包**零命中**。唯一的软件停车路径是 `scripts/keyboard_control.py` 的 `s` 键与 Ctrl-C（发一帧全零 Twist，`:135-138, 158-166`）。→ 覆盖执行需要一个**独立于键盘脚本**的急停入口。

### 8.7 CAN/串口故障是**静默降级**

- **打开失败**（`can_bridge_node.cpp:189-206`）：记日志、返回 `false`，构造函数**跳过** `setupCAN()`/`startReading()`（`:119-122`），节点**继续自旋**、发布 `/can_bridge_status = "Disconnected"`（`:665`），**不重试、不重连、不退出**；
- **发送失败**（`:290-305`）：**仅记日志后吞掉**，不重试、不关闭、不发布任何东西；且 `is_connected_` 只在 `:198` 置 `true`，**从不置回 `false`** —— 端口中途死掉，标志位不会翻转。

### 8.8 速度钳位确实存在（但取决于用哪个 launch）

代码里有三重钳位，与 launch 参数无关：

- 线/角速度：`cmd_vel_to_can_node.cpp:116-120`（非对称，倒车按 `-max_reverse_speed_`）；
- ERPM：`:148-152`（用 `wheel_radius_` 换算）；
- 舵机角：`:164`（`SERVO_LEFT_LIMIT` / `SERVO_RIGHT_LIMIT`）。

`auto_control.launch.py:35-37` 传入 `max_forward_speed 0.25` / `max_reverse_speed 0.10` / `max_yaw_rate 0.50` —— **这是室内栈使用的 launch，正确**。

⚠️ 但 `complete_control.launch.py:47-48` **只设了遗留的 `max_linear_speed: 2.0`**，于是两个非对称参数**回退到 2.0 m/s**。**室内覆盖绝不能走这个 launch。**

### 8.9 `steeringAngleFromTwist` 在零速附近的行为

`include/vehicle_can_bridge/ackermann_steering.hpp:19-25`：当 `|linear_speed| < min_steering_speed`（默认 **0.02 m/s**）时**返回 `0.0`**，即舵机回正 —— **不除零、也不保持上次转向**（该函数无任何静态状态）。

> **与 MPPI 的交互值得首轮观察**：MPPI 在急弯会降速，若降到 0.02 m/s 以下，转向会归中。0.02 m/s 极低，正常 0.25 m/s 覆盖速度下不会触发，但**如果发现急弯处车"不肯转"**，这就是原因 —— 应提高 `min_steering_speed` 的调参优先级，而不是去改规划器。

---

## 9. 已知并接受的风险

| # | 风险 | 来源 | 后果 |
|---|---|---|---|
| R1 | **不记录实际轨迹** | Q16(a) | 出现"看起来扫完了但角落没扫到"时**无任何数据可定位漏扫段**，只能整片重跑 |
| R2 | **无碰撞条/防撞传感器** | Q9 | 硬件层面无接触保护。**唯一自动生效的保护是 STM32 固件的 500 ms 消息超时→退回 RC**（§8.2），它保护的是"ROS 失联"，不是"即将撞上" |
| R3 | **低于 5 cm 的障碍不可见** | LiDAR 切片 `min_height 0.05 / max_height 0.80`，装高 z=0.615 | 线缆、门槛、地毯边、掉落小物件在感知上是空的。0.88×0.68 的车撞上去就是硬撞。**这是 Q14(a) 要求全程有人在场的直接原因** |
| R4 | **地图 `free_thresh: 0.25` 把 1500 m² 未知区域当作自由空间** | Q24(b) 选择不修 | 真实自由面积 757.1 m² 被读成 2257.1 m²。costmap 会认为未建图区域可通行；**在 RViz 里点选多边形时可能选到未建图区域**，F2C 会在那里规划覆盖。**操作规避：点选多边形时以地图上明确的墙体为界，不要依赖"空白=可走"** |
| R5 | **`min_turning_radius` 未实测** | Q23(b) | §5.1 先用保守值 0.60，阶段 B1 必须实测回填。若实测值大于 0.60，地头宽度与覆盖带数量都要重算 |
| R6 | **首次运行即集成测试** | Q20(a) | 无分级缓冲，失败时难以区分是定位/规划/控制/底盘哪一层的问题。**缓解：阶段 A 的离线验证必须做扎实** |
| R7 | **无 URDF，TF 全靠静态发布器** | 工作区无任何 `.urdf/.xacro/.sdf` | `base_link → base_footprint` 是**全零**静态 TF，不能表达真实底盘几何；`base_footprint → livox_frame` 外参 `(0.11, 0, 0.615)` 也未经实测。定位精度上限受此约束 |
| R8 | **文档与代码不一致** | `doc/IndoorNav_Issues.md` / `IndoorNav_Modification_Plan.md` | 部分已修复的条目仍写作"待修"，会误导后续调试。§4.5 已列入修正项 |
| R9 | **多个失效启动路径** | `system.launch.py`、`nav2/launch/nav2_launch.py` 引用不存在的 `turn_on_wheeltec_robot` | 误用会直接失败。§4.3 列入清理项 |
| R10 | **无轨迹记录 + 定性验收叠加** | Q7(a) × Q16(a) | 验收结论完全依赖人眼观察。**明确接受：本项目不产出覆盖率的量化证据** |
| R11 | `free_thresh` 不修带来的 RViz 观感问题 | Q24(b) | 大量未建图区域显示为"自由"，可能让人高估可作业面积 |
| R12 | ⚠️ **控制权静默移交 / 回切** | `docs/DEPLOYMENT.md:273-274` | ① 启动时若 RC 在"自动"挡，`cmd_vel_to_can` 一发帧控制权就静默交给 ROS；② 500 ms 看门狗触发后交回 RC，**此时若摇杆不在中位，车会突然动作**。**启动前必须确认 RC 挡位，并保持摇杆中位、有人持握** |
| R13 | **CAN/串口故障静默降级** | §8.7 | 覆盖中途 CAN 死掉，节点继续自旋、不重连、不退出，除日志外**无任何告警**；`is_connected_` 不会翻回 `false`。500 ms 后车交回 RC |
| R14 | **误用 `complete_control.launch.py` 会失去速度保护** | §8.8 | 该 launch 只设 `max_linear_speed: 2.0`，`max_forward_speed`/`max_reverse_speed` 回退到 **2.0 m/s**（应为 0.25/0.10） |
| R15 | **`/vehicle/remote_manual_override` 无订阅者** | §8.4 | 若不按 §4.4 新增订阅，**Q14(a) 的"遥控抢占中止"不会自动生效** —— 人按了遥控、车交给 RC 了，但覆盖任务**仍在"执行中"**，状态与实际不符 |
| R16 | **`.bashrc` 把 turtlebot3_ws 注入每个终端** | §3.6 | `CMAKE_PREFIX_PATH` 里的 turtlebot3_ws 是当初 `find_package` 漏过去的通道。日常构建已不会出错（本工作区优先），但**只有隔离环境构建才能自证本工作区不依赖它**。若将来 turtlebot3_ws 被删/移动，隔离构建仍应通过 |
| R17 | **`build/` 陈旧目录类故障可能重现** | §3.7 | `ament_cmake_python_symlink_*` 遇到同名真实目录即失败，且**报错发生在 `build/` 内部、与源码无关**，容易误判成代码问题。再次出现时按 §3.7 的 `diff -rq` 流程处理，**不要直接 `rm -rf` 整个 `build/`**（会触发全量重编，且 5 个包的阻塞会一起回来） |
| R18 | **活动 cmake 是 `/usr/local` 的 4.3.2**，非系统 3.22 | §3.8 备注 | 大量 `CMake Deprecation Warning` 由此而来（无害）。但 CMake 4.x 已移除对 `cmake_minimum_required(VERSION <3.5)` 的兼容 —— **若将来引入新包编译失败，应优先怀疑它**，而不是回头改覆盖相关代码 |
| R19 | ~~🔴 覆盖链在室内启动链上是断的~~ **已修复** | §4.3.1 | ✅ 已用 `use_coverage` 按需开关补接，实测两个节点在 `true` 时启动并就绪。**残留**：`coverage_server` 未被 gate，`false` 时仍空转（有意为之，原因见 §4.3.1） |
| R20 | ~~`min_turning_radius` 未实测~~ **已实测 0.95 m** | §5.1.3 | ✅ 阶段 B1 已完成。实测值比几何反推值（0.52）大 1.83 倍，已回填。**`linear_curv_change` 仍未实测**（舵机速率），当前 200 偏乐观，属阶段 B2 |
| R21 | ⚠️ **转向标定曾系统性偏差 1.71 倍** | §5.1.3 | `max_steering_angle_deg` 配 30° 而实际满舵仅 17.5°，导致命令角与实得角差 0.583 倍，**MPPI/Smac/F2C 全都按车做不到的转弯能力规划**（R 预期 0.52 实得 0.95）。已修正 4 处参数。**残留风险**：修正假设"命令→转角"是线性的（`ackermann_steering.hpp:42-51` 确为线性映射），但转向机构本身可能非线性 —— **需上车按 §5.1.3 末尾的方法复测验证** |
| R22 | **`REEDS_SHEPP` 引入倒车段** | §5.1.5 | 掉头时的三点掉头含倒车。倒车受 `max_reverse_speed = 0.10 m/s` 限制（慢），且 §8.1 无 ROS 看门狗、R3 低于 5 cm 障碍不可见 —— **倒车方向没有防撞传感器**，而覆盖作业时人的注意力通常在前方。缓解：§4.4 的 `remote_manual_override` 抢占订阅 + 全程有人持遥控 |
| R23 | **软件安全层依赖 CAN 链路存活** | §4.4、§8.7 | `/vehicle/remote_manual_override` 来自 CAN ID `0x07`。若 CAN/串口静默降级（§8.7：不重连、不退出，除日志外无告警），该信号**随之停止发布** → 桥接节点**收不到抢占通知**。此时软件安全层失效，**只剩 STM32 的 500 ms 硬件兜底**（它保护的是"ROS 失联"，且会把控制权交给遥控器）。**缓解：上车时必须先 `ros2 topic hz /vehicle/remote_manual_override` 确认 ≈1 Hz**，这已列入 §10 的硬性检查 |
| R24 | **软件急停是"一次触发即中止任务"** | §4.4 | `/coverage/emergency_stop` 发 `true` 会中止当前任务并锁存 `_safety_abort`；**下一帧覆盖目标会被 `_goal_callback` 正常接受**（新目标会清标志）。即它不能"禁用覆盖功能"，只是停止当前这次。另外它只发**有限帧**零速（默认 5 帧），若之后有其它节点继续发速度指令，车会重新动 —— 要彻底停住请用**物理急停** |

---

## 10. 参数速查（打印带到现场）

```yaml
# coverage_server —— ✅ 以下值已过 A-4 离线实测（越界 0.000 m，R 用实测 0.95）
robot_width: 0.68            # footprint 宽（车体实际覆盖宽度）
operation_width: 0.60        # 条带间距。⚠️必须 <= robot_width，否则漏扫！
                             #   0.68=零重叠 0.60=12%重叠 0.51=25%重叠
min_turning_radius: 0.95     # 【实测】车体中心画圆。⚠️勿用 轴距/tan(舵角) 反推！
linear_curv_change: 200      # ⚠️对越界影响最大！>=50 才行；1.0 会冲出田块 2.5 m
default_headland_type: "CONSTANT"
default_headland_width: 1.30 # 由转弯外冲量决定，不是 R+0.10
default_path_type: "REEDS_SHEPP"  # 允许倒车三点掉头 → 地头需求 2.80 → 1.30
                                  # 改回 "DUBIN" 则地头必须提到 2.80
default_path_continuity_type: "CONTINUOUS"   # Q18a
default_route_type: "BOUSTROPHEDON"
default_swath_angle_type: "SET_ANGLE"        # Q17a
default_swath_angle: 1.57
default_swath_type: "NUMBER"
default_turn_point_distance: 0.05
default_allow_overlap: false
coordinates_in_cartesian_frame: true

# 转向标定（⚠️ 三处必须一致，否则控制器按车做不到的能力规划）
max_steering_angle_deg: 17.5     # auto_control.launch.py / indoor_amcl.launch.py
                                 # 原 30.0 → 命令角与实得角差 0.583 倍
MPPI  AckermannConstraints.min_turning_r: 0.95   # param_mini_diff_indoor.yaml:130
Smac  minimum_turning_radius: 0.95               # param_mini_diff_indoor.yaml:316

# 设计规则（A-4 实测得出，改参数时照此判断）：
#   越界 = 0  ⟺  operation_width >= 2 × 转弯半径
#   不满足时，靠 default_headland_width 吸收外冲量（本车即属这种情况）
#   且 linear_curv_change >= 50 时，转弯半径 ≈ min_turning_radius
#
# 真实耗时估算：task_time(秒) ÷ 0.25   （task_time 是按 1.0 m/s 算的，见 §5.2.2）

# 桥接节点 —— ⚠️ 生效值在 launch，不在 .py（见 §5.2）
# 两个文件必须一致：
#   nav2_mission/launch/coverage_navpath_bridge.launch.py   ← 生效值
#   nav2_mission/nav2_mission/coverage_navpath_bridge_node.py ← 代码默认值
follow_timeout_sec: 3600.0   # ⚠️ 原 300.0 = 覆盖总执行超时，会在中途 cancel
compute_timeout_sec: 120.0
generate_headland: true
controller_id: "FollowPath"  # = MPPI + Ackermann

# Nav2
global_costmap.inflation_radius: 0.60   # ⚠️ 原 0.55 < 车体外接圆 0.556
```

```yaml
# 底盘（auto_control.launch.py 传入，代码内三重钳位）
max_forward_speed: 0.25      # ⚠️ 若误用 complete_control.launch.py 会变成 2.0
max_reverse_speed: 0.10
max_yaw_rate:      0.50
min_steering_speed: 0.02     # 低于此速度舵机回正（§8.9）

# 启动脚本选择
必须:  indoor_amcl.launch.py  → auto_control.launch.py
禁止:  complete_control.launch.py（速度钳位 2.0 m/s）
禁用:  system.launch.py / nav2/launch/nav2_launch.py（引用不存在的 turn_on_wheeltec_robot）

# 覆盖任务才需要开的开关（§4.3.1）
USE_COVERAGE=true ./start_indoor_navigation.sh   # 起覆盖桥接 + RViz 区域选择
./start_indoor_navigation.sh                     # 日常导航，默认不起覆盖节点
```

```bash
# 前置依赖检查：缺这个包整条室内栈起不来（§4.3）
ros2 pkg prefix pointcloud_to_laserscan || \
  sudo apt install ros-humble-pointcloud-to-laserscan
```

```bash
# 构建（§3.4）—— 必须在隔离环境里跑，否则 turtlebot3_ws 会混进来
env -i HOME=$HOME USER=$USER \
  PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
  TERM=dumb LANG=C.UTF-8 \
  bash -c 'source /opt/ros/humble/setup.bash && \
    cd /home/wyz/ros2/new_base_plate_indoor_ws && \
    colcon build --symlink-install \
      --packages-up-to opennav_coverage wheeltec_nav2 nav2_mission'

# 一行判定版本是否切对（humble-v2 才有 DecompMode）
ros2 interface show opennav_coverage_msgs/msg/DecompMode

# 一行判定根因是否消除（应输出本工作区 install 下的 fields2cover include）
grep -o '\-isystem [^ ]*fields2cover[^ ]*' \
  build/opennav_coverage/CMakeFiles/opennav_coverage_core.dir/flags.make
```

**上车前的硬性检查**：

1. `ros2 interface show opennav_coverage_msgs/msg/DecompMode` 必须成功（版本判定）；
2. `ros2 topic hz /vehicle/remote_manual_override` ≈ 1 Hz（遥控抢占信号在线）；
3. RC 挡位已知、摇杆中位、有人持握。

---

## 11. 与既有文档的关系

本方案**不替代** `doc/IndoorNav_Modification_Plan.md`，两者是**正交**的：

- `IndoorNav_Modification_Plan.md` 解决的是 **Nav2 基础导航**（定位、TF、舵角标定、footprint 标定）；
- 本方案解决的是 **全覆盖规划与执行**，它**建立在前者之上**。

按 Q21(a)（定位已稳定），本方案的阶段 A 可以与定位相关的收尾工作**并行**（Q22b）。
