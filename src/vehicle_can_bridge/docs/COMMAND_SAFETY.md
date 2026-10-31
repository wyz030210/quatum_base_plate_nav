# 共同自动速度入口与停止契约

`cmd_vel_to_can` 是自动速度进入现有 `0x06 / 0x01` CAN 组合控制帧的共同入口。
停止判定在该节点内部优先于 `/cmd_vel`，使用原有 ERPM 换算、前进/倒车限速和阿克曼转向映射。
停止输出为 ERPM 0、舵机中位 90；停止期间按配置周期持续输出，不能靠另一个节点竞争发布零速。
`can_bridge` 继续负责原串口/CAN 协议，不在这里新增硬件驱动。

## 公开接口

| 接口 | 消息/服务 | 行为 |
| --- | --- | --- |
| `/cmd_vel` | `geometry_msgs/Twist` | 新速度；被抑制的消息不会缓存或恢复重放 |
| `/task_control/stop` | `std_msgs/Bool` | true 停止并抑制自动速度；false 显式申请新一轮自动控制，只有 ready 时受理 |
| `/task_control/heartbeat` | `std_msgs/Empty` | 任务监督器存活；任务模式下必须持续到达 |
| `/task_control/emergency_stop` | `std_msgs/Bool` | true 锁存软件急停；false 仅解除本输入，不解除保持 |
| `/coverage/emergency_stop` | `std_msgs/Bool` | 旧急停入口，与新入口按 OR 合并，各自保留输入状态 |
| `/vehicle/remote_manual_override` | `std_msgs/Bool` | 遥控接管立即抑制自动速度；返回自动挡保留停止请求 |
| `/task_control/reset_stop` | `std_srvs/Trigger` | 两个急停输入和接管均解除，且所需心跳/停车反馈有效时，显式清除急停保持与故障；仍保持 stop_requested |
| `/task_control/state` | `std_msgs/String` | 周期 JSON，包含 ready、stopped、stop_requested、manual_override、emergency_stop、fault、feedback_valid、feedback_age_sec、feedback_stamp_sec |
| `feedback_topic` 参数 | `nav_msgs/Odometry` | 默认 `/odometry/filtered_odom`，用来源时间与本机接收时间检查反馈 |

`ready` 表示当前安全前提允许申请新一轮控制，与 `stop_requested` 独立。
任务管理器还须同时检查 `stopped`，并检查状态消息自身的接收新鲜度，才能受理下一任务。
停车反馈可在急停或接管仍生效时成立；停车成立不会隐式清除任何抑制条件。
尚无反馈时两个反馈时间字段为 null，JSON 不包含 NaN 或 Infinity。

## 恢复与配置

`config/command_safety.yaml` 是已接入 `auto_control.launch.py` 的默认配置。
`command_safety_params` 启动参数可替换该文件；任务启用的整车启动须传 `task_control_required:=true`，并沿用唯一的原转换节点。
普通导航默认 false，正常目标结束零速或速度中断后会停车，后续新的速度可以开始下一个普通导航目标。
显式停止、接管和急停仍须显式恢复，普通导航开关不会绕过这些状态。

| 参数 | 默认值 | 用途 |
| --- | --- | --- |
| command_timeout_sec | 1.0 s | 最近非零速度失联后输出停止；任务模式锁存 command_timeout |
| heartbeat_timeout_sec | 1.0 s | 任务监督器失联锁存 heartbeat_timeout |
| feedback_timeout_sec | 1.0 s | 来源和接收年龄均须在此范围内；运动中失效锁存 feedback_timeout_or_invalid |
| feedback_future_tolerance_sec | 0.1 s | 来源时间允许的小幅超前容差 |
| feedback_required | true | 默认要求有效反馈才能接收速度/复位；关闭后也不会凭无反馈宣称 stopped |
| stopped_linear_threshold | 0.01 m/s | 反馈平移速度近零阈值 |
| stopped_angular_threshold | 0.02 rad/s | 反馈角速度近零阈值 |
| stopped_hold_sec | 1.0 s | 连续新鲜、来源时间递增的近零反馈保持时间 |
| stopping_timeout_sec | 5.0 s | 运动后的本次停车未确认则锁存 stopping_timeout |
| safety_publish_period_sec | 0.05 s | 停止输出及状态检查周期 |

参数在启动时记录；时间必须为有限正数，容差和近零阈值必须为有限非负数。
速度非有限值锁存 invalid_command。负/非法来源时间、重复或过期时间戳、非有限速度/协方差与负协方差对角项均不能确认停车。
收到一次零速后仅等待时间不会成为已停车；新的停止过程也不会沿用上一次的静止样本。

故障/急停恢复顺序：解除所有有效输入 → 持续提供所需心跳和新鲜静止反馈 → 调用 reset_stop → 发送 stop=false → 提供新的速度。
复位本身、收到新任务或速度、恢复心跳、回到自动挡均不会自行恢复运动。

## 固件配合与现场核对

现有 `DEPLOYMENT.md` 记录 STM32 有 500 ms 控制帧超时看门狗并退回 RC 控制。
这里保留该协议约定：上游退出时转换节点仍运行，会在 command_timeout_sec 后持续发送 CAN 停止帧；
转换节点、串口桥或整机退出时无法继续发送，由固件超时/RC 互锁兜底。
当前仓库没有可据以验证的在车固件源码，不能把已有文档描述当作已测得的停车性能。

现场仍须核对实际固件版本、500 ms 的准确条件、返回 RC 时油门/挡位行为、0x07 接管反馈时延、
急停保持、转向回中，以及 odometry 来源/时间/频率。上述默认阈值是软件起始配置；
真实停止距离、停止时间和接管性能留给低速实车验收 #9，桌面测试不能关闭该验收。

## 无硬件消息回归

`test/test_command_safety.py` 启动实际 `cmd_vel_to_can` 可执行文件，通过 ROS 消息输入并解码 `/can_tx`，
固定专用 `ROS_DOMAIN_ID=173` 和 `ROS_LOCALHOST_ONLY=1`；不启动 `can_bridge`、串口或车辆。
覆盖失联、停止优先级、急停 OR 保持/复位、接管返回、心跳、反馈新鲜度/异常和原有限速/转向输出。
已注册到 CTest，构建后可通过 `colcon test --packages-select vehicle_can_bridge --ctest-args -R 'test_command_safety|test_ackermann_steering'` 运行。
