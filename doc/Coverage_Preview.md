# 本机覆盖路径预览

工作区：`/home/wyz/ros2/new_base_plate_indoor_ws`。

## 启动与选点

```bash
cd /home/wyz/ros2/new_base_plate_indoor_ws
bash start_coverage_preview.sh
```

启动覆盖规划器、它的生命周期管理器、选点节点、静态参考坐标系与 RViz。
脚本使用本机 ROS 域 94，设置 `ROS_LOCALHOST_ONLY=1`。

1. 在 RViz 工具栏选择 **Publish Point**。
2. 按顺时针或逆时针依次点击区域的边界顶点，至少三个点。每格为 1 米；首次建议画约 8×8 米矩形。
3. 点击首点附近（0.35 米以内）闭合多边形，即自动规划。
4. 黄色线是选区边界，绿色线是覆盖路径，蓝色条带是实际作业带。终端会显示路径点数量。
5. 规划完成后点击一个新点，即开始新区域，并清除旧路径。滚轮可以缩放。

无需设置机器人初始位姿，也无需底盘、雷达、IMU、定位或导航控制器。
RViz 的 Fixed Frame 已设为 `map`；地面标记使没有地图时也能用 Publish Point 选点。
预览结果发布在 `/coverage_ui/preview_path`，使用 Transient Local QoS，晚打开显示也能收到最新路径。
选点节点只向 `/compute_coverage_path` 发送请求，不调用导航执行动作或发布速度命令。

## 清空、撤销与重新规划

在第二个终端先设置环境：

```bash
cd /home/wyz/ros2/new_base_plate_indoor_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=94 ROS_LOCALHOST_ONLY=1
```

```bash
# 清空选区及路径
ros2 service call /coverage_region_selector/clear std_srvs/srv/Trigger '{}'
# 撤销最后一个顶点；已闭合时先解除闭合
ros2 service call /coverage_region_selector/undo_last_point std_srvs/srv/Trigger '{}'
# 保留当前选区重新规划
ros2 service call /coverage_region_selector/execute_polygon std_srvs/srv/Trigger '{}'
```

## 修改规划

规划参数在 `src/wheeltec_nav2/param/coverage_params.yaml`，包括作业宽度、最小转弯半径、地头宽度、条带角度和转弯方式。
`operation_width` 是实际作业带宽度，`swath_spacing` 是相邻平行条带中心线间距，两者独立。
当前间距为 2.00 m、作业带宽为 0.60 m，直线作业带间约有 1.40 m 的未覆盖区。
`swath_spacing: 0.0` 可恢复间距等于作业宽度的旧行为；负数和非有限值会被拒绝。
当前转弯类型为 `DUBIN`，使用只前进的连续曲率转弯。它与原倒车换向类型 `REEDS_SHEPP` 不同。
转弯保留模型的原始位置和朝向采样点，不再分别做样条重采样。
启动脚本直接读取该源码文件；修改 YAML 后按 Ctrl+C 结束预览，再运行脚本即可，不需要编译。

算法源码位于 `src/opennav_coverage/opennav_coverage/src/`：

- `headland_generator.cpp`：地头内缩。
- `swath_generator.cpp`：条带生成。
- `route_generator.cpp`、`route_method.cpp`：条带访问顺序与连接路线。
- `path_generator.cpp`：转弯路径。
- `coverage_server.cpp`：规划流水线。
- `../include/opennav_coverage/utils.hpp`：消息转换、车体边界检查。

修改 C++ 后先结束预览，再编译并重新启动：

```bash
cd /home/wyz/ros2/new_base_plate_indoor_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --packages-select opennav_coverage --symlink-install
bash start_coverage_preview.sh
```

保持 `validate_path_bounds: true` 会拒绝区域不合法、车体越界或地头余量不足的规划，常见错误码为 803；具体原因见终端日志。
预览使用选定多边形和几何约束，不检查传感器障碍物或 Nav2 代价地图，也不验证实车跟踪。
当前外圈地头仍是掉头预留空间，未独立覆盖。

## 验证记录

2026-10-10：本机三个相关包编译成功；预览与控制器回归测试共 11 项通过。
8×8 米矩形完整流程返回 1017 个 `map` 坐标系路径点，RViz 正常显示。
真实鼠标 Publish Point、晚订阅获取路径、清空路径均验证通过；测试域无 `/cmd_vel` 或导航执行动作。

同日更新：条带中心距 2.00 m、实际作业宽 0.60 m、最小转弯半径 0.80 m，
8×8 m 和 18×14 m 矩形分别返回 428、2208 个路径点；无倒车段，蓝条宽度和中心距验证通过。
条带、路径、路线、可视化及相关几何参数共 8 组 C++ 回归测试通过。
斜边梯形仍可能因转弯时车身越界返回 803，本次没有关闭边界验证，也没有验证实车跟踪。
