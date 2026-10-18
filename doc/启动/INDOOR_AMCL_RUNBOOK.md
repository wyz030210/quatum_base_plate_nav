# 室内 AMCL 导航运行手册

## 系统结构

- `AMCL`：唯一发布 `map -> odom`。
- 本地 EKF：唯一发布 `odom -> base_link`，输出 `/odometry/filtered_odom`。
- FAST-LIO：继续输出 `/Odometry`，但在室内配置中不发布里程计 TF。
- MID360 点云经转换后生成 `/scan`，供 AMCL 和 Nav2 使用。
- 室内入口不启动 RTK、`location` 或全局 EKF。

## 1. 安装运行依赖

在小车上执行：

```bash
sudo apt update
sudo apt install ros-humble-pointcloud-to-laserscan
```

## 2. 同步并编译

从开发电脑同步源码（网络可达时执行）：

```bash
rsync -av /home/wyz/ros2/new_base_plate_indoor_ws/src/ \
  car@172.20.10.5:/home/car/ros2/new_base_plate_indoor_ws/src/
```

在小车上编译：

```bash
cd /home/car/ros2/new_base_plate_indoor_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to robot_bringup
source install/setup.bash
```

## 3. 准备地图

当前使用的室内地图为：

```text
/home/car/ros2/catographer_ws/map/003.yaml
/home/car/ros2/catographer_ws/map/003.pgm
```

`003.yaml` 使用相对图像路径 `003.pgm`，两者必须保持在同一目录。

## 4. 启动

小车端无显示器时使用：

```bash
cd /home/car/ros2/new_base_plate_indoor_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch robot_bringup indoor_amcl.launch.py \
  map:=/home/car/ros2/catographer_ws/map/003.yaml \
  use_rviz:=false \
  can_serial_port:=/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 \
  max_forward_speed:=0.25 \
  max_reverse_speed:=0.10 \
  max_yaw_rate:=0.50 \
  max_linear_accel:=0.40 \
  max_yaw_accel:=0.50
```

如果在有图形界面的机器上运行，可改成 `use_rviz:=true`。

也可以直接运行工作空间根目录的一键启动脚本：

```bash
cd /home/car/ros2/new_base_plate_indoor_ws
./start_indoor_navigation.sh
```

临时调整速度时可使用环境变量，例如：

```bash
MAX_FORWARD_SPEED=0.15 MAX_YAW_RATE=0.35 ./start_indoor_navigation.sh
```

## 5. 手动设置初始位姿

启动完成后，在 RViz 中选择 **2D Pose Estimate**，在地图上点击小车实际位置并沿车头方向拖动。确认激光点与地图墙体基本重合后，再发送导航目标。

每次重新启动 AMCL 都需要重新设置初始位姿。

## 6. 导航前检查

```bash
ros2 topic hz /scan
ros2 topic hz /odometry/filtered_odom
ros2 lifecycle get /amcl
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_link
```

预期结果：

- `/scan` 和 `/odometry/filtered_odom` 持续更新。
- AMCL 状态为 `active`。
- `map -> odom -> base_link` 两段 TF 连续可用，且没有重复发布者。

首次实车测试请保留急停条件，先用默认低速参数，在空旷走廊发送短距离目标。
