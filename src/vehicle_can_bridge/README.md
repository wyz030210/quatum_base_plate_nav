# vehicle_can_bridge（ROS2 移植版）

外部 ROS1 (catkin) 包 `vehicle_can_bridge`（来源 `https://github.com/liuchongxiao/vehicle_can_bridge`）的忠实 ROS2 (Humble) 移植版：通过 USB-CAN 适配器把 AGV 底控 STM32 板接入 ROS2，控制车辆电机速度/转向，并读取舵机角度、驱动器转速等反馈。

**本包完全独立**，不接入本仓库（`AGV_Navigation`）现有的 ZMQ 通信体系（该体系是本仓库刻意的架构选择，详见根目录 `docs/RULES.md` 技术决策记录），不注册进 `core/scripts/launch.py`、`show_topology.py`、`port_allocator.py`。移植背景、设计取舍见根目录 `docs/superpowers/specs/2026-07-11-vehicle-can-bridge-ros2-port-design.md` 和 `docs/CHANGES.md` 2026-07-11 条目。

## 文档导航

- 自动任务的共同速度入口、停止/接管/急停契约与无硬件测试：[`docs/COMMAND_SAFETY.md`](docs/COMMAND_SAFETY.md)。任务模式须显式启用 `task_control_required`；默认普通导航保留空闲后接受新目标的行为。

每份文档都有 Markdown 源文件（含 Mermaid 图表，GitHub/大部分 Markdown 阅读器直接能看图）和对应 PDF（图表已渲染成矢量图，适合打印/离线分发）：

- **第一次用，想让车动起来** → [`docs/GETTING_STARTED.md`](docs/GETTING_STARTED.md) / [PDF](docs/GETTING_STARTED.pdf)：编译、确认串口、键盘遥控/发指令、看反馈，一步步来
- **要把这个包部署到实车/新机器上** → [`docs/DEPLOYMENT.md`](docs/DEPLOYMENT.md) / [PDF](docs/DEPLOYMENT.pdf)：目标环境要求、硬件接线（**CAN1/CAN2 别接错**，配接线图）、编译、故障排查（含 `brltty` 抢串口等实测踩过的坑）
- **想改代码/搞懂协议怎么实现的** → [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) / [PDF](docs/ARCHITECTURE.pdf)：三个节点的职责、数据流（配时序图/状态机图）、CAN 协议字节格式、ROS1→ROS2 移植要点、topic/参数速查表

PDF 由仓库外的 `Markdown-to-PDF` 项目（`/Users/kwhq/Documents/WorkSpace/Markdown-to-PDF/md2pdf.py`，Markdown+Mermaid → 自包含 HTML → headless Chromium 打印）生成，修改了对应 `.md` 后可重新生成：
```bash
/opt/miniconda3/envs/kwhq-venv/bin/python3 /Users/kwhq/Documents/WorkSpace/Markdown-to-PDF/md2pdf.py \
  docs/GETTING_STARTED.md docs/GETTING_STARTED.pdf
```

## 快速开始（最短路径）

```bash
# 放进你的 colcon workspace
cp -r ros2/vehicle_can_bridge <your_ws>/src/
cd <your_ws>
colcon build --packages-select vehicle_can_bridge
source install/setup.bash

# 键盘遥控测试：先起核心节点
ros2 launch vehicle_can_bridge auto_control.launch.py

# 再另开一个终端，起键盘遥控节点（只能 ros2 run，不能 ros2 launch）
ros2 run vehicle_can_bridge keyboard_control.py --ros-args -p speed:=0.5 -p turn:=1.0
```

详细步骤、按键说明、故障排查见上面的文档导航。

## 目录结构

```
vehicle_can_bridge/
├── package.xml / CMakeLists.txt
├── msg/Frame.msg                          # 自带 CAN 帧消息类型（不依赖外部 can_msgs）
├── include/vehicle_can_bridge/
│   └── serial_port.hpp                    # POSIX termios 串口封装（不依赖外部 serial 包）
├── src/
│   ├── can_bridge_node.cpp                # 可执行 can_bridge：串口↔CAN 桥接
│   ├── cmd_vel_to_can_node.cpp             # 可执行 cmd_vel_to_can：/cmd_vel → CAN 组合控制帧
│   └── speed_query_node.cpp               # 可执行 speed_query：驱动器测速查询
├── launch/                                # 3 个 .launch.py
├── config/vehicle_params.yaml             # 参考配置（未接入任何 launch，与原包一致）
├── scripts/keyboard_control.py            # 键盘遥控
└── docs/
    ├── GETTING_STARTED.md
    ├── DEPLOYMENT.md
    └── ARCHITECTURE.md
```
