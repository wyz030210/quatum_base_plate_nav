// ROS2 (Humble) port of the original ROS1 vehicle_can_bridge/src/can_bridge_node.cpp.
//
// This is a line-by-line port of the original CANBridgeNode class. Ported
// verbatim (Task 3 of the vehicle_can_bridge ROS1->ROS2 port plan):
//
//  * ros::Publisher/Subscriber -> rclcpp::Publisher/Subscription, same
//    topic names and queue depths as the original.
//  * serial::Serial -> vehicle_can_bridge::SerialPort (Task 2), same
//    call sequence (setPort/setBaudrate/setTimeout/open, flushInput/
//    flushOutput, write/read/available), serial::IOException ->
//    vehicle_can_bridge::IOException.
//  * The two-timer auto speed-query mechanism (query_timer_ periodic +
//    drive_query_timer_ 10ms one-shot-ish cycle) is ported to
//    create_wall_timer with explicit cancel()/reset(): drive_query_timer_
//    is created and immediately cancel()'d (mirrors the original
//    createTimer(..., oneshot=false, autostart=false)); startQueryCycle()
//    calls reset() to (re)start it (mirrors ros::Timer::start()); rcl's
//    rcl_timer_reset() is documented to un-cancel a canceled timer, so
//    this reproduces the original start/stop semantics exactly.
//  * The original run() loop (ros::Rate(100) while(ros::ok())) cannot be
//    ported as a blocking loop under rclcpp::spin(); it is replaced with
//    a 10ms wall timer (pollLoop) that calls processReceivedData() every
//    tick and publishStatus() every 100th tick (matching the original's
//    100Hz x 100-ticks-per-second == 1 status publish/sec).
//  * processReceivedData()'s byte-parsing state machine (frame header
//    search, frame_length computation, standard/extended frame decode,
//    speed-query-response decode, servo angle -> angular velocity decode)
//    is ported byte-for-byte. This node's frame_length convention is
//    `5 + data_length` for standard frames and `7 + data_length` for
//    extended frames (NOT including a trailing 0x55) -- this is
//    deliberately different from the speed_query_node's convention and
//    must not be "harmonized" with it.
//  * The original's function-scoped `static` locals (buffer,
//    last_servo_angle, last_time) become CANBridgeNode members so they
//    are not shared across instances; last_servo_angle_/last_time_ use
//    an explicit last_servo_angle_initialized_ flag to reproduce C++'s
//    "initialize once, on first execution of the declaration" semantics
//    of the original function-local statics (both get initialized
//    together the first time the CAN ID 0x05 branch executes).
//
// 2026-07-13 新增（不是 ROS1 移植内容，是本包首次原生新增功能）：STM32
// 固件同日更新，在既有 CAN ID 0x06（ROS→STM32 控制方向，已用 data[0]
// command_type 区分 0x01 组合控制/0x02 电机/0x03 舵机）上新增两个
// command_type：0x04（CAN 使能状态写命令）、0x05（PF0 头灯/滚刷合并
// 继电器开关命令——同一路继电器输出同时接工作头灯和滚刷电机，是硬件上
// 捆绑在一起的单个开关量，不是纯粹的"灯"），并新增 CAN ID 0x07
// （STM32→ROS，DLC=3，与心跳同频周期性反馈：enabled/
// remote_manual_override/headlight_brush_state 三个 0/1 字节）。
// 遵循本文件既有约定：一个标量一个 std_msgs 话题，不新建聚合消息类型；
// 新命令复用既有 sendFrame() 发送路径，不另起一套发送机制。

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <vehicle_can_bridge/msg/frame.hpp>
#include <vehicle_can_bridge/serial_port.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <string>
#include <thread>
#include <vector>

using namespace std::chrono_literals;

class CANBridgeNode : public rclcpp::Node
{
public:
  CANBridgeNode()
  : rclcpp::Node("can_bridge_node")
  {
    // 初始化参数
    initParameters();

    // 初始化发布者和订阅者
    can_rx_pub_ = create_publisher<vehicle_can_bridge::msg::Frame>(
      "/can_rx", rclcpp::QoS(100));
    status_pub_ = create_publisher<std_msgs::msg::String>(
      "/can_bridge_status", rclcpp::QoS(10));
    speed_response_pub_ = create_publisher<std_msgs::msg::String>(
      "/vehicle/speed_response", rclcpp::QoS(10));
    servo_angle_pub_ = create_publisher<std_msgs::msg::Float32>(
      "/vehicle/servo_angle", rclcpp::QoS(10));
    angular_velocity_pub_ = create_publisher<std_msgs::msg::Float32>(
      "/vehicle/angular_velocity", rclcpp::QoS(10));
    twist_pub_ = create_publisher<geometry_msgs::msg::Twist>(
      "/vehicle/twist", rclcpp::QoS(10));
    // 2026-07-13 新增：CAN ID 0x07 反馈帧的三个字段，各开一个 Bool 话题
    // （沿用本文件"一个标量一个话题"的既有惯例，不新建聚合消息类型）。
    enabled_pub_ = create_publisher<std_msgs::msg::Bool>(
      "/vehicle/enabled", rclcpp::QoS(10));
    remote_manual_override_pub_ = create_publisher<std_msgs::msg::Bool>(
      "/vehicle/remote_manual_override", rclcpp::QoS(10));
    headlight_brush_state_pub_ = create_publisher<std_msgs::msg::Bool>(
      "/vehicle/headlight_brush_state", rclcpp::QoS(10));
    can_tx_sub_ = create_subscription<vehicle_can_bridge::msg::Frame>(
      "/can_tx", rclcpp::QoS(100),
      std::bind(&CANBridgeNode::canTxCallback, this, std::placeholders::_1));
    speed_query_sub_ = create_subscription<std_msgs::msg::UInt8>(
      "/vehicle/speed_query", rclcpp::QoS(10),
      std::bind(&CANBridgeNode::speedQueryCallback, this, std::placeholders::_1));
    // 2026-07-13 新增：ROS→STM32 的两个新命令，均复用既有 CAN ID 0x06
    // + sendFrame() 发送路径，只是 data[0] command_type 不同（0x04/0x05）。
    enable_cmd_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/vehicle/enable_cmd", rclcpp::QoS(10),
      std::bind(&CANBridgeNode::enableCmdCallback, this, std::placeholders::_1));
    headlight_brush_cmd_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/vehicle/headlight_brush_cmd", rclcpp::QoS(10),
      std::bind(&CANBridgeNode::headlightBrushCmdCallback, this, std::placeholders::_1));

    // 连接串口
    if (!connectSerial()) {
      throw std::runtime_error("CAN serial connection failed: " + port_);
    }
    setupCAN();
    startReading();

    // 启动自动速度查询定时器
    if (auto_query_enabled_ && query_frequency_ > 0.0) {
      // 启动查询周期定时器
      query_timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / query_frequency_),
        std::bind(&CANBridgeNode::startQueryCycle, this));

      // 启动单个驱动器查询定时器（10ms间隔），创建后立即 cancel
      // 对齐原 nh_.createTimer(..., oneshot=false, autostart=false)
      drive_query_timer_ = create_wall_timer(
        10ms, std::bind(&CANBridgeNode::queryNextDrive, this));
      drive_query_timer_->cancel();

      current_drive_id_ = 0;  // 初始化
      RCLCPP_INFO(
        get_logger(), "Non-blocking speed query timers started at %.1f Hz",
        query_frequency_);
    }

    // 原 run() 的 100Hz while(ros::ok()) 循环改写为 10ms wall timer：
    // 每 tick 调用 processReceivedData()，每 100 次 tick（约1秒）调用一次
    // publishStatus()（用成员计数器替代原 static int status_counter）。
    poll_timer_ = create_wall_timer(10ms, std::bind(&CANBridgeNode::pollLoop, this));

    RCLCPP_INFO(get_logger(), "CAN Bridge Node initialized");
  }

  ~CANBridgeNode() override
  {
    if (serial_.isOpen()) {
      serial_.close();
    }
  }

private:
  void initParameters()
  {
    port_ = declare_parameter<std::string>("port", "/dev/ttyUSB0");
    baudrate_ = declare_parameter<int>("baudrate", 2000000);
    enable_debug_ = declare_parameter<bool>("enable_debug", false);

    // 角度转换参数
    servo_center_angle_ = declare_parameter<double>("servo_center_angle", 90.0);
    servo_left_limit_ = declare_parameter<double>("servo_left_limit", 40.0);
    servo_right_limit_ = declare_parameter<double>("servo_right_limit", 190.0);
    steering_ratio_ = declare_parameter<double>("steering_ratio", 15.0);
    max_steering_angle_ = declare_parameter<double>("max_steering_angle", 30.0);

    // 速度查询参数
    query_frequency_ = declare_parameter<double>("query_frequency", 1.0);        // 默认1Hz
    auto_query_enabled_ = declare_parameter<bool>("auto_query_enabled", true);   // 默认启用自动查询

    RCLCPP_INFO(get_logger(), "CAN Bridge parameters:");
    RCLCPP_INFO(get_logger(), "  Port: %s", port_.c_str());
    RCLCPP_INFO(get_logger(), "  Baudrate: %d", baudrate_);
    RCLCPP_INFO(get_logger(), "  Debug: %s", enable_debug_ ? "enabled" : "disabled");
    RCLCPP_INFO(get_logger(), "  Servo center angle: %.1f", servo_center_angle_);
    RCLCPP_INFO(get_logger(), "  Servo left limit: %.1f", servo_left_limit_);
    RCLCPP_INFO(get_logger(), "  Servo right limit: %.1f", servo_right_limit_);
    RCLCPP_INFO(get_logger(), "  Steering ratio: %.1f", steering_ratio_);
    RCLCPP_INFO(get_logger(), "  Max steering angle: %.1f", max_steering_angle_);
    RCLCPP_INFO(get_logger(), "  Speed query frequency: %.1f Hz", query_frequency_);
    RCLCPP_INFO(get_logger(), "  Auto query: %s", auto_query_enabled_ ? "enabled" : "disabled");
  }

  bool connectSerial()
  {
    try {
      serial_.setPort(port_);
      serial_.setBaudrate(baudrate_);
      // 原 serial::Timeout::simpleTimeout(1000)
      serial_.setTimeout(1000);
      serial_.open();

      is_connected_ = true;
      RCLCPP_INFO(get_logger(), "Successfully connected to %s", port_.c_str());
      return true;
    } catch (const vehicle_can_bridge::IOException & e) {
      RCLCPP_ERROR(
        get_logger(), "Failed to open serial port %s: %s", port_.c_str(), e.what());
      return false;
    }
  }

  void setupCAN()
  {
    // 计算校验和函数
    auto calculate_checksum = [](const std::vector<uint8_t> & data) -> uint8_t {
        uint8_t checksum = 0;
        for (size_t i = 2; i < data.size(); i++) {
          checksum += data[i];
        }
        return checksum & 0xff;
      };

    // 配置命令：标准帧、可变长度协议、接收所有ID
    std::vector<uint8_t> set_can_baudrate = {
      0xAA, 0x55, 0x12, 0x03, 0x01,   // 0x01=标准帧
      0x00, 0x00, 0x00, 0x00,         // 过滤器
      0x00, 0x00, 0x00, 0x00,         // 掩码
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00
    };

    uint8_t checksum = calculate_checksum(set_can_baudrate);
    set_can_baudrate.push_back(checksum);

    // 发送配置命令
    serial_.write(set_can_baudrate);
    RCLCPP_INFO(get_logger(), "CAN configuration command sent");

    // 等待配置生效
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // 读取配置响应
    std::string response_str = serial_.read(100);
    std::vector<uint8_t> response(response_str.begin(), response_str.end());
    if (!response.empty()) {
      RCLCPP_INFO(get_logger(), "Device response received");
      if (enable_debug_) {
        std::string hex_response;
        for (uint8_t byte : response) {
          char hex[4];
          snprintf(hex, sizeof(hex), "%02x ", byte);
          hex_response += hex;
        }
        RCLCPP_DEBUG(get_logger(), "Response: %s", hex_response.c_str());
      }
    } else {
      RCLCPP_WARN(get_logger(), "No device response received");
    }
  }

  void startReading()
  {
    // 清空缓冲区
    serial_.flushInput();
    serial_.flushOutput();

    RCLCPP_INFO(get_logger(), "Starting CAN message reading...");
  }

  // canTxCallback 的帧构建体，供 canTxCallback 与 speedQueryCallback/
  // sendSpeedQueryToDrive 两处复用（行为与原 canTxCallback 一致）。
  void sendFrame(const vehicle_can_bridge::msg::Frame & msg)
  {
    // 构建CAN发送帧
    std::vector<uint8_t> frame;
    frame.push_back(0xAA);  // 帧头

    // 控制字节：标准帧数据帧
    uint8_t control_byte = 0xC0 | (msg.dlc & 0x0F);
    frame.push_back(control_byte);

    // CAN ID（小端序）
    frame.push_back(msg.id & 0xFF);           // ID低字节
    frame.push_back((msg.id >> 8) & 0xFF);    // ID高字节

    // 数据
    for (int i = 0; i < msg.dlc && i < 8; i++) {
      frame.push_back(msg.data[i]);
    }

    // 结束符
    frame.push_back(0x55);

    // 发送帧
    try {
      serial_.write(frame);
      tx_count_++;

      if (enable_debug_) {
        std::string hex_frame;
        for (uint8_t byte : frame) {
          char hex[4];
          snprintf(hex, sizeof(hex), "%02x ", byte);
          hex_frame += hex;
        }
        RCLCPP_DEBUG(get_logger(), "Sent CAN frame: %s", hex_frame.c_str());
      }
    } catch (const vehicle_can_bridge::IOException & e) {
      is_connected_ = false;
      RCLCPP_ERROR(get_logger(), "Failed to send CAN message: %s", e.what());
      throw;
    }
  }

  void canTxCallback(const vehicle_can_bridge::msg::Frame::SharedPtr msg)
  {
    if (!is_connected_) {
      RCLCPP_WARN(get_logger(), "Serial not connected, cannot send CAN message");
      return;
    }

    sendFrame(*msg);
  }

  void speedQueryCallback(const std_msgs::msg::UInt8::SharedPtr msg)
  {
    if (!is_connected_) {
      RCLCPP_WARN(get_logger(), "Serial not connected, cannot send speed query");
      return;
    }

    uint8_t drive_id = msg->data;

    // 构建速度查询CAN消息
    vehicle_can_bridge::msg::Frame can_msg;
    can_msg.header.stamp = this->now();
    can_msg.id = 0x05;   // ROS控制ID
    can_msg.dlc = 2;
    can_msg.is_extended = false;
    can_msg.data[0] = 0x05;      // 速度查询指令
    can_msg.data[1] = drive_id;  // 驱动器ID

    // 发送CAN消息
    sendFrame(can_msg);

    RCLCPP_INFO(get_logger(), "Speed query sent for drive %d", drive_id);
  }

  // 2026-07-13 新增：CAN 使能状态写命令，command_type=0x04。
  // data[1]=0 关闭、非0启用（同时也是固件上电默认值）。固件侧维护一套
  // RC 有/无 + CAN 可写位组合出来的三档互锁状态机，ROS 侧不需要理解那套
  // 状态机，只管把开关意图发出去即可。
  void enableCmdCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (!is_connected_) {
      RCLCPP_WARN(get_logger(), "Serial not connected, cannot send enable command");
      return;
    }

    vehicle_can_bridge::msg::Frame can_msg;
    can_msg.header.stamp = this->now();
    can_msg.id = 0x06;           // ROS控制ID（沿用组合/电机/舵机控制同一个ID）
    can_msg.dlc = 2;
    can_msg.is_extended = false;
    can_msg.data[0] = 0x04;      // command_type：CAN 使能状态写命令
    can_msg.data[1] = msg->data ? 1 : 0;

    sendFrame(can_msg);

    RCLCPP_INFO(
      get_logger(), "Enable command sent: %s", msg->data ? "enable" : "disable");
  }

  // 2026-07-13 新增：PF0 头灯/滚刷合并继电器开关命令，command_type=0x05。
  // 这一路继电器输出同时接工作头灯和滚刷电机，是硬件上捆绑在一起的单个
  // 开关量，不是纯粹的"灯"。data[1]=0 关闭、非0开启。效果等价于遥控器
  // 手动模式下拨动通道6——当 ROS/CAN 接管车辆时（无遥控器，或遥控器处于
  // 自动挡），这条命令就是驱动该输出的手段，取代通道6。
  void headlightBrushCmdCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (!is_connected_) {
      RCLCPP_WARN(get_logger(), "Serial not connected, cannot send headlight/brush command");
      return;
    }

    vehicle_can_bridge::msg::Frame can_msg;
    can_msg.header.stamp = this->now();
    can_msg.id = 0x06;           // ROS控制ID（沿用组合/电机/舵机控制同一个ID）
    can_msg.dlc = 2;
    can_msg.is_extended = false;
    can_msg.data[0] = 0x05;      // command_type：头灯/滚刷合并继电器开关命令
    can_msg.data[1] = msg->data ? 1 : 0;

    sendFrame(can_msg);

    RCLCPP_INFO(
      get_logger(), "Headlight/brush command sent: %s", msg->data ? "on" : "off");
  }

  void startQueryCycle()
  {
    if (!is_connected_) {
      return;
    }

    // 开始新的查询周期
    current_drive_id_ = 1;
    drive_query_timer_->reset();  // 启动单个驱动器查询定时器
  }

  void queryNextDrive()
  {
    if (!is_connected_ || current_drive_id_ < 1 || current_drive_id_ > 4) {
      drive_query_timer_->cancel();  // 停止定时器
      return;
    }

    // 查询当前驱动器
    sendSpeedQueryToDrive(current_drive_id_);

    // 移动到下一个驱动器
    current_drive_id_++;

    // 如果查询完所有驱动器，停止定时器
    if (current_drive_id_ > 4) {
      drive_query_timer_->cancel();
    }
  }

  void sendSpeedQueryToDrive(uint8_t drive_id)
  {
    // 构建速度查询CAN消息，直接发送给驱动器
    vehicle_can_bridge::msg::Frame query_msg;
    query_msg.header.stamp = this->now();
    query_msg.id = drive_id;  // 直接发送给驱动器ID
    query_msg.dlc = 2;
    query_msg.is_extended = false;
    query_msg.data[0] = 0x0F;  // 查询指令
    query_msg.data[1] = 0x01;  // 查询当前转速

    // 发送CAN消息
    sendFrame(query_msg);

    if (enable_debug_) {
      RCLCPP_DEBUG(get_logger(), "Speed query sent to drive %d", drive_id);
    }
  }

  void processReceivedData()
  {
    if (!is_connected_) {
      return;
    }

    // 读取可用数据
    size_t available_bytes = serial_.available();
    if (available_bytes == 0) {
      return;
    }

    std::string data_str = serial_.read(available_bytes);
    std::vector<uint8_t> data(data_str.begin(), data_str.end());

    // 处理接收到的数据（原 static std::vector<uint8_t> buffer -> 类成员 rx_buffer_）
    rx_buffer_.insert(rx_buffer_.end(), data.begin(), data.end());

    // 处理缓冲区中的完整帧
    while (rx_buffer_.size() > 0) {
      // 寻找帧头
      if (rx_buffer_[0] != 0xAA) {
        rx_buffer_.erase(rx_buffer_.begin());
        continue;
      }

      if (rx_buffer_.size() < 2) {
        break;
      }

      uint8_t control_byte = rx_buffer_[1];

      // 确定帧长度
      int frame_length = 0;
      if ((control_byte & 0xC0) == 0xC0) {  // CAN数据帧
        int data_length = control_byte & 0x0F;
        if (control_byte & 0x20) {  // 扩展帧
          frame_length = 7 + data_length;
        } else {  // 标准帧
          frame_length = 5 + data_length;
        }
      } else if (control_byte == 0x55) {  // 设备状态
        frame_length = 20;
      } else {
        frame_length = 5;  // 假设最小长度
      }

      if (static_cast<int>(rx_buffer_.size()) < frame_length) {
        break;
      }

      // 提取帧
      std::vector<uint8_t> frame_data(rx_buffer_.begin(), rx_buffer_.begin() + frame_length);
      rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + frame_length);

      // 解析CAN帧
      if ((frame_data[0] == 0xAA) && ((frame_data[1] & 0xC0) == 0xC0)) {
        int data_len = frame_data[1] & 0x0F;
        bool is_extended = (frame_data[1] & 0x20) != 0;

        if (is_extended) {
          // 扩展帧
          if (static_cast<int>(frame_data.size()) >= 7 + data_len) {
            uint32_t can_id = (frame_data[5] << 24) | (frame_data[4] << 16) |
              (frame_data[3] << 8) | frame_data[2];

            vehicle_can_bridge::msg::Frame can_msg;
            can_msg.header.stamp = this->now();
            can_msg.id = can_id;
            can_msg.dlc = data_len;
            can_msg.is_extended = true;

            for (int i = 0; i < data_len && i < 8; i++) {
              can_msg.data[i] = frame_data[6 + i];
            }

            can_rx_pub_->publish(can_msg);
            rx_count_++;

            if (enable_debug_) {
              RCLCPP_DEBUG(
                get_logger(), "Received extended CAN frame: ID=0x%08X, DLC=%d",
                can_id, data_len);
            }
          }
        } else {
          // 标准帧
          if (static_cast<int>(frame_data.size()) >= 5 + data_len) {
            uint16_t can_id = (frame_data[3] << 8) | frame_data[2];

            vehicle_can_bridge::msg::Frame can_msg;
            can_msg.header.stamp = this->now();
            can_msg.id = can_id;
            can_msg.dlc = data_len;
            can_msg.is_extended = false;

            for (int i = 0; i < data_len && i < 8; i++) {
              can_msg.data[i] = frame_data[4 + i];
            }

            can_rx_pub_->publish(can_msg);
            rx_count_++;

            // 处理速度查询响应（来自驱动器ID 1-4）
            if (can_id >= 1 && can_id <= 4 && data_len >= 6) {
              // 检查是否是速度查询响应：[0x0F, 0x01, erpm高字节, erpm, erpm, erpm低字节]
              if (frame_data[4] == 0x0F && frame_data[5] == 0x01) {
                // 解析ERPM (4字节，大端序) - 从frame_data[6]开始
                int32_t erpm = (frame_data[6] << 24) | (frame_data[7] << 16) |
                  (frame_data[8] << 8) | frame_data[9];

                // 计算RPM (假设磁极对数为15)
                double rpm = static_cast<double>(erpm) / 15.0;

                std_msgs::msg::String speed_msg;
                speed_msg.data = "Drive " + std::to_string(can_id) +
                  " Speed: " + std::to_string(erpm) + " ERPM (" +
                  std::to_string(rpm) + " RPM)";
                speed_response_pub_->publish(speed_msg);
              }
            }

            // 处理角度消息 (CAN ID 0x05) - 根据STM32发送的2字节角度数据
            if (can_id == 0x05 && data_len == 2) {
              // 解析2字节角度数据 (大端序)
              uint16_t angle_raw = (frame_data[4] << 8) | frame_data[5];

              // 转换为舵机角度 (0-180度)
              double servo_angle = static_cast<double>(angle_raw);

              // 发布舵机角度
              std_msgs::msg::Float32 angle_msg;
              angle_msg.data = servo_angle;
              servo_angle_pub_->publish(angle_msg);

              // 计算角速度 (舵机角度变化率)
              // 原 static double last_servo_angle = servo_angle;
              // 原 static ros::Time last_time = ros::Time::now();
              // 两者在原代码里于首次执行到本分支时一起初始化一次；
              // 用 last_servo_angle_initialized_ 标志复现这个「只初始化一次」的语义。
              if (!last_servo_angle_initialized_) {
                last_servo_angle_ = servo_angle;
                last_time_ = this->now();
                last_servo_angle_initialized_ = true;
              }

              rclcpp::Time current_time = this->now();

              double dt = (current_time - last_time_).seconds();
              if (dt > 0.0) {
                double angle_change = servo_angle - last_servo_angle_;
                double angular_velocity_rad_per_sec = (angle_change * M_PI / 180.0) / dt;

                // 限制角速度范围
                angular_velocity_rad_per_sec =
                  std::max(-2.0, std::min(2.0, angular_velocity_rad_per_sec));

                // 发布角速度
                std_msgs::msg::Float32 angular_velocity_msg;
                angular_velocity_msg.data = angular_velocity_rad_per_sec;
                angular_velocity_pub_->publish(angular_velocity_msg);

                // 发布Twist消息 (用于里程计)
                geometry_msgs::msg::Twist twist_msg;
                twist_msg.angular.z = angular_velocity_rad_per_sec;
                twist_pub_->publish(twist_msg);

                if (enable_debug_) {
                  RCLCPP_DEBUG(
                    get_logger(), "CAN ID 0x05: Angle=%.1f -> Angular Velocity=%.3f rad/s",
                    servo_angle, angular_velocity_rad_per_sec);
                }

                last_servo_angle_ = servo_angle;
                last_time_ = current_time;
              }
            }

            // 2026-07-13 新增：CAN ID 0x07，STM32→ROS 反馈帧，与心跳同频
            // （约1Hz周期性发送，不是请求/响应）。DLC=3，三个独立的0/1
            // 字节，都是单字节标量，不涉及字节序问题。
            if (can_id == 0x07 && data_len == 3) {
              // data[0]：最终使能状态（RC状态 + CAN可写使能位合并后的结果）
              std_msgs::msg::Bool enabled_msg;
              enabled_msg.data = (frame_data[4] != 0);
              enabled_pub_->publish(enabled_msg);

              // data[1]：遥控器当前是否手动接管；为1时说明此刻ROS发送的
              // 任何命令都会被固件忽略，供ROS侧判断"发了也没用"
              std_msgs::msg::Bool override_msg;
              override_msg.data = (frame_data[5] != 0);
              remote_manual_override_pub_->publish(override_msg);

              // data[2]：PF0 头灯/滚刷合并继电器当前实际状态（权威回读值，
              // 可能由ROS经0x06/0x05命令改变，也可能由遥控器通道6在其手动
              // 模式下改变）
              std_msgs::msg::Bool headlight_brush_msg;
              headlight_brush_msg.data = (frame_data[6] != 0);
              headlight_brush_state_pub_->publish(headlight_brush_msg);

              if (enable_debug_) {
                RCLCPP_DEBUG(
                  get_logger(),
                  "CAN ID 0x07: enabled=%d, remote_manual_override=%d, headlight_brush_state=%d",
                  frame_data[4], frame_data[5], frame_data[6]);
              }
            }

            if (enable_debug_) {
              RCLCPP_DEBUG(
                get_logger(), "Received standard CAN frame: ID=0x%04X, DLC=%d",
                can_id, data_len);
            }
          }
        }
      }
    }
  }

  void publishStatus()
  {
    std_msgs::msg::String status_msg;
    status_msg.data = std::string("CAN Bridge Status: ") +
      (is_connected_ ? "Connected" : "Disconnected") +
      ", RX: " + std::to_string(rx_count_) +
      ", TX: " + std::to_string(tx_count_);
    status_pub_->publish(status_msg);
  }

  // 原 run() 的 ros::Rate(100) while(ros::ok()) 循环 -> 10ms wall timer。
  // 每 tick 调 processReceivedData()；每 100 次 tick（原 status_counter 逻辑，
  // 现为成员 status_counter_）发布一次状态，等价于原 100Hz x 100次 = 每秒一次。
  void pollLoop()
  {
    processReceivedData();

    if (++status_counter_ >= 100) {
      publishStatus();
      status_counter_ = 0;
    }
  }

  // 发布者和订阅者
  rclcpp::Publisher<vehicle_can_bridge::msg::Frame>::SharedPtr can_rx_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr speed_response_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr servo_angle_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr angular_velocity_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
  // 2026-07-13 新增：CAN ID 0x07 反馈帧的三个字段发布者
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr enabled_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr remote_manual_override_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr headlight_brush_state_pub_;
  rclcpp::Subscription<vehicle_can_bridge::msg::Frame>::SharedPtr can_tx_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr speed_query_sub_;
  // 2026-07-13 新增：CAN ID 0x06 command_type 0x04/0x05 两个新命令的订阅者
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr headlight_brush_cmd_sub_;

  // 串口对象
  vehicle_can_bridge::SerialPort serial_;

  // 参数
  std::string port_;
  int baudrate_ = 0;
  bool enable_debug_ = false;

  // 状态变量
  bool is_connected_ = false;
  uint32_t rx_count_ = 0;
  uint32_t tx_count_ = 0;

  // 速度查询参数
  double query_frequency_ = 1.0;
  bool auto_query_enabled_ = true;
  rclcpp::TimerBase::SharedPtr query_timer_;
  rclcpp::TimerBase::SharedPtr drive_query_timer_;
  int current_drive_id_ = 0;

  // 角度转换参数（声明但本节点逻辑未使用，忠实照抄）
  double servo_center_angle_ = 90.0;
  double servo_left_limit_ = 40.0;
  double servo_right_limit_ = 190.0;
  double steering_ratio_ = 15.0;
  double max_steering_angle_ = 30.0;

  // run() -> 10ms wall timer 替代
  rclcpp::TimerBase::SharedPtr poll_timer_;
  int status_counter_ = 0;

  // processReceivedData 状态：原 static std::vector<uint8_t> buffer
  std::vector<uint8_t> rx_buffer_;

  // processReceivedData 状态：原 static double last_servo_angle / static ros::Time last_time
  double last_servo_angle_ = 0.0;
  rclcpp::Time last_time_;
  bool last_servo_angle_initialized_ = false;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<CANBridgeNode>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("can_bridge_node"), "CAN bridge stopped: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();

  return 0;
}
