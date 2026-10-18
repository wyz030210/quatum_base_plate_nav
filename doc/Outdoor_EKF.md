# 室外 RTK + 双 EKF 启动入口

## 入口与分工

整车入口为 `robot_bringup/launch/outdoor_ekf.launch.py`，取代原入口名称。
它自动启动 CAN 底盘、MID360、FAST-LIO、RTK、本地 EKF、全局 EKF 和 Nav2。
FAST-LIO 默认使用 `mid360.yaml`；当前实现默认不发布里程计 TF。
本地 EKF 发布 `odom -> base_link`；全局 EKF 发布 `map -> odom`。
室外导航只加载地图，不启动 AMCL。

启动阶段的延时只用于错开启动时间，不是传感器或定位就绪检测。
此入口不会自动发送导航或覆盖任务，也不启动键盘遥控。
不要同时运行室内整车入口或重复启动相同驱动。

## 编译和启动

```bash
cd /home/wyz/ros2/new_base_plate_indoor_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --symlink-install --packages-select robot_bringup wheeltec_nav2 location
source install/setup.bash
```

普通导航入口如下。执行前必须确认设备端口和定位条件；这里仅列出用法：

```bash
ros2 launch robot_bringup outdoor_ekf.launch.py
```

默认地图为安装后的 `wheeltec_nav2/map/map.yaml`，可通过 `map:=/完整路径/地图.yaml` 覆盖。
入口在任何硬件动作前检查地图 YAML 文件存在，并将相对路径规范为绝对路径。
这一检查不会验证地图内容或地图与 RTK 是否已配准。

按需开启覆盖功能：

```bash
ros2 launch robot_bringup outdoor_ekf.launch.py use_coverage:=true
```

`use_coverage` 默认 `false`。在室外入口中，关闭时不启动覆盖规划服务、任务桥接和区域选点；
生命周期管理器的节点列表也移除覆盖服务，避免等待不存在的节点。
开启只是加载功能，不会自动执行覆盖任务。
共用导航文件新增的开关保留原室内默认值：室内仍启动 AMCL，覆盖规划服务仍保留。

## 车辆参数

| 参数 | 默认值 |
|---|---:|
| `max_forward_speed` | 0.25 m/s |
| `max_reverse_speed` | 0.10 m/s |
| `max_yaw_rate` | 0.50 rad/s |
| `max_linear_accel` | 0.40 m/s² |
| `max_yaw_accel` | 0.50 rad/s² |
| `max_steering_angle_deg` | 17.5° |
| `wheel_base` | 0.30 m |
| `wheel_radius` | 0.1275 m |
| `min_steering_speed` | 0.02 m/s |

速度和横摆角速度限制同时传给底盘驱动、Nav2 控制器和速度平滑器。
加速度限制传给 Nav2 控制器和速度平滑器；底盘驱动本身没有加速度参数。
最大前轮转角和轴距传给 CAN 换算节点；室外规划最小半径按
`wheel_base / tan(max_steering_angle_deg)` 计算，并传给 MPPI、全局规划和覆盖规划。
这只是运动学约束一致性，不能替代实车转弯半径和转向响应标定。
室内配置文件和离线预览的覆盖参数没有因此改写。

## 必须核实的实车条件

- `can_serial_port` 默认保持当前 USB-CAN 的稳定设备路径。
- `rtk_serial_port` 默认使用 2026-10-17 在 NUC 仅连接 RTK 时确认的固定路径：
  `/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0`。
  独立 RTK 启动入口和节点自身的默认值也保持一致，不再依赖 `/dev/ttyUSB0` 编号。
  仍可通过 `rtk_serial_port`（独立入口使用 `serial_port`）显式覆盖。
  插回底盘后应检查两个固定路径分别指向各自设备；若解析到同一设备，室外入口会拒绝启动。
- `rtk_baud_rate` 默认 115200。`datum_lat`、`datum_lon` 默认沿用旧值
  29.4898045、106.5676103，并可作为启动参数覆盖。
- 默认地图存在不代表它是正确的室外地图；必须与上述 RTK 原点及 ENU 朝向一致。
- RTK 固定解降级标志不清除、旧数据被新时间戳重复发布的问题尚未在本次修复；
  未修复并完成失联处理验证前，不应把这个启动入口视为已经通过室外实车验收。
- 当前覆盖 YAML 中 2.0 m 条带间距、0.6 m 作业宽度会产生间隙；外圈预留掉头区也不覆盖。
  本次不改变这些作业策略。

本次验证仅检查启动对象、参数传递、条件分支和构建，不连接硬件、不发送运动命令。
