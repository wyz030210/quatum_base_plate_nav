// ROS2 (Humble) port of the original ROS1 vehicle_can_bridge/src/speed_query_node.cpp.
//
// This is a line-by-line port of the original SpeedQueryNode class (Task 5
// of the vehicle_can_bridge ROS1->ROS2 port plan):
//
//  * ros::Publisher/Subscriber -> rclcpp::Publisher/Subscription, same
//    topic names and queue depths as the original. Note the original
//    publishes to `/can_tx` (can_tx_pub_) but never actually calls
//    can_tx_pub_.publish() anywhere in the source -- sendSpeedQuery() writes
//    directly to the serial port instead. This is carried over verbatim:
//    can_tx_pub_ is created but unused, matching the original dead code.
//  * serial::Serial -> vehicle_can_bridge::SerialPort (Task 2), same call
//    sequence (setPort/setBaudrate/setTimeout/open, flushInput/
//    flushOutput, write/read/available), serial::IOException ->
//    vehicle_can_bridge::IOException.
//  * setupCAN() here does NOT read a device response after sending the
//    configuration command -- unlike can_bridge_node's setupCAN(), the
//    original speed_query_node's setupCAN() has no read() call at all.
//    This asymmetry is intentional in the original source and is preserved
//    as-is (not "fixed up" to match can_bridge_node).
//  * This node has TWO independent speed-response parse paths, both ported
//    verbatim:
//      (a) canRxCallback(): parses already-decoded Frame messages arriving
//          on the /can_rx topic (published by can_bridge_node).
//      (b) processReceivedData(): parses raw bytes read directly from this
//          node's own serial port connection.
//    Both update the same drive_speeds_/waiting_response_/response_count_
//    state and publish to /vehicle/speed_response.
//  * processReceivedData()'s frame_length convention is
//    `5 + data_length + 1` for standard frames and `7 + data_length + 1`
//    for extended frames (INCLUDING the trailing 0x55 end marker) -- this
//    is deliberately different from can_bridge_node's convention (which
//    excludes the +1) and must not be "harmonized" with it. This function
//    also validates the trailing end_marker byte (frame_data[4+data_len])
//    against 0x55 and discards the frame with an RCLCPP_WARN (if
//    enable_debug_) when it doesn't match -- can_bridge_node has no such
//    check. Both differences are carried over unchanged from the original.
//  * The original's function-scoped `static std::vector<uint8_t> buffer`
//    local in processReceivedData() becomes the rx_buffer_ member so it is
//    not shared across instances.
//  * The original run() loop (ros::Rate(10) while(ros::ok())) cannot be
//    ported as a blocking loop under rclcpp::spin(); it is replaced with a
//    100ms wall timer (pollLoop) that reproduces the loop body exactly:
//    processReceivedData(), then (if the query cycle has elapsed) issue a
//    sendSpeedQuery() for every drive not currently waiting_response_,
//    then checkTimeouts(), then every 10th tick publishStatus() (matching
//    the original's 10Hz x 10-ticks-per-second == 1 status publish/sec).
//  * IMPORTANT variable distinction (see brief): the original run()'s
//    query-cycle gate was a *function-local scalar* `ros::Time
//    last_query_time` (no trailing underscore, not a map) used only to
//    decide when to kick off the next round of queries. This is completely
//    separate from the class member `last_query_time_` (WITH trailing
//    underscore, `std::map<int, ros::Time>`) which sendSpeedQuery()/
//    checkTimeouts() use for the *per-drive* response timeout. The two are
//    ported to two distinct members here:
//      - last_query_cycle_time_ (rclcpp::Time, single scalar) -- new
//        member, mirrors the old local `last_query_time`; used only in
//        pollLoop() to gate when to start the next query round.
//      - last_query_time_ (std::map<int, rclcpp::Time>) -- unchanged
//        per-drive member, written by sendSpeedQuery() and read by
//        checkTimeouts(); pollLoop() never reads or writes this map
//        directly.

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <vehicle_can_bridge/msg/frame.hpp>
#include <vehicle_can_bridge/serial_port.hpp>

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

using namespace std::chrono_literals;

class SpeedQueryNode : public rclcpp::Node
{
public:
  SpeedQueryNode()
  : rclcpp::Node("speed_query_node")
  {
    // 初始化参数
    initParameters();

    // 初始化发布者和订阅者
    can_tx_pub_ = create_publisher<vehicle_can_bridge::msg::Frame>(
      "/can_tx", rclcpp::QoS(10));
    speed_pub_ = create_publisher<std_msgs::msg::String>(
      "/vehicle/speed_response", rclcpp::QoS(10));
    status_pub_ = create_publisher<std_msgs::msg::String>(
      "/speed_query_status", rclcpp::QoS(10));
    can_rx_sub_ = create_subscription<vehicle_can_bridge::msg::Frame>(
      "/can_rx", rclcpp::QoS(100),
      std::bind(&SpeedQueryNode::canRxCallback, this, std::placeholders::_1));

    // 初始化驱动器状态
    for (int i = 1; i <= 4; i++) {
      drive_speeds_[i] = 0.0;
      last_query_time_[i] = this->now();
      waiting_response_[i] = false;
    }

    // 连接串口
    if (connectSerial()) {
      setupCAN();
      startReading();
    }

    // 查询周期门控（原 run() 函数体内局部标量 last_query_time，
    // 与逐驱动器 map 成员 last_query_time_ 是两个完全不同的变量，见文件头注释）
    last_query_cycle_time_ = this->now();

    // 原 run() 的 ros::Rate(10) while(ros::ok()) 循环 -> 100ms wall timer
    poll_timer_ = create_wall_timer(100ms, std::bind(&SpeedQueryNode::pollLoop, this));

    RCLCPP_INFO(get_logger(), "Speed Query Node initialized");
    RCLCPP_INFO(get_logger(), "Query frequency: %.1f Hz", query_frequency_);
    RCLCPP_INFO(get_logger(), "Motor pole pairs: %d", motor_pole_pairs_);
    RCLCPP_INFO(get_logger(), "Wheel radius: %.3f m", wheel_radius_);
  }

  ~SpeedQueryNode() override
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
    query_frequency_ = declare_parameter<double>("query_frequency", 1.0);   // 默认1Hz
    motor_pole_pairs_ = declare_parameter<int>("motor_pole_pairs", 15);    // 电机磁极对数
    wheel_radius_ = declare_parameter<double>("wheel_radius", 0.1275);     // 轮子半径

    RCLCPP_INFO(get_logger(), "Speed Query parameters:");
    RCLCPP_INFO(get_logger(), "  Port: %s", port_.c_str());
    RCLCPP_INFO(get_logger(), "  Baudrate: %d", baudrate_);
    RCLCPP_INFO(get_logger(), "  Query frequency: %.1f Hz", query_frequency_);
    RCLCPP_INFO(get_logger(), "  Motor pole pairs: %d", motor_pole_pairs_);
    RCLCPP_INFO(get_logger(), "  Wheel radius: %.3f m", wheel_radius_);
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
    // 原代码这里没有读取设备响应这一步（与 can_bridge_node 的 setupCAN 不同），照抄。
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  void startReading()
  {
    // 清空缓冲区
    serial_.flushInput();
    serial_.flushOutput();

    RCLCPP_INFO(get_logger(), "Starting CAN message reading...");
  }

  void sendSpeedQuery(int drive_id)
  {
    if (!is_connected_) {
      RCLCPP_WARN(get_logger(), "Serial not connected, cannot send speed query");
      return;
    }

    // 构建CAN发送帧
    std::vector<uint8_t> frame;
    frame.push_back(0xAA);  // 帧头

    // 控制字节：标准帧数据帧，DLC=2
    uint8_t control_byte = 0xC0 | 0x02;  // DLC=2
    frame.push_back(control_byte);

    // CAN ID（小端序）
    frame.push_back(drive_id & 0xFF);           // ID低字节
    frame.push_back((drive_id >> 8) & 0xFF);    // ID高字节

    // 数据：0F 01
    frame.push_back(QUERY_CMD);   // 0x0F
    frame.push_back(QUERY_DATA);  // 0x01

    // 结束符
    frame.push_back(0x55);

    // 发送帧
    try {
      serial_.write(frame);
      query_count_++;
      waiting_response_[drive_id] = true;
      last_query_time_[drive_id] = this->now();

      if (enable_debug_) {
        std::string hex_frame;
        for (uint8_t byte : frame) {
          char hex[4];
          snprintf(hex, sizeof(hex), "%02x ", byte);
          hex_frame += hex;
        }
        RCLCPP_DEBUG(
          get_logger(), "Sent speed query to drive %d: %s", drive_id, hex_frame.c_str());
      }
    } catch (const vehicle_can_bridge::IOException & e) {
      RCLCPP_ERROR(get_logger(), "Failed to send speed query: %s", e.what());
    }
  }

  void canRxCallback(const vehicle_can_bridge::msg::Frame::SharedPtr msg)
  {
    // 检查是否是驱动器响应 (ID 1-4)
    if (msg->id >= 1 && msg->id <= 4 && msg->dlc >= 6) {
      // 检查是否是速度查询响应
      if (msg->data[0] == QUERY_CMD && msg->data[1] == QUERY_DATA) {
        // 解析ERPM (4字节，大端序)
        int32_t erpm = (msg->data[2] << 24) | (msg->data[3] << 16) |
          (msg->data[4] << 8) | msg->data[5];

        // 计算RPM
        double rpm = static_cast<double>(erpm) / motor_pole_pairs_;

        // 更新驱动器速度
        drive_speeds_[msg->id] = rpm;
        waiting_response_[msg->id] = false;
        response_count_++;

        // 发布速度响应
        std_msgs::msg::String speed_msg;
        speed_msg.data = "Drive " + std::to_string(msg->id) +
          " Speed: " + std::to_string(erpm) + " ERPM (" +
          std::to_string(rpm) + " RPM)";
        speed_pub_->publish(speed_msg);

        if (enable_debug_) {
          RCLCPP_DEBUG(get_logger(), "Drive %d: ERPM=%d, RPM=%.1f", msg->id, erpm, rpm);
        }
      }
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

      // 确定帧长度（本节点口径含尾部 0x55，与 can_bridge_node 故意不同）
      int frame_length = 0;
      if ((control_byte & 0xC0) == 0xC0) {  // CAN数据帧
        int data_length = control_byte & 0x0F;
        if (control_byte & 0x20) {  // 扩展帧
          frame_length = 7 + data_length + 1;  // +1 for 0x55 end marker
        } else {  // 标准帧
          frame_length = 5 + data_length + 1;  // +1 for 0x55 end marker
        }
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

        if (!is_extended && frame_data.size() >= 5 + static_cast<size_t>(data_len) + 1) {
          uint16_t can_id = (frame_data[3] << 8) | frame_data[2];

          // 检查结束符
          uint8_t end_marker = frame_data[4 + data_len];
          if (end_marker != 0x55) {
            if (enable_debug_) {
              RCLCPP_WARN(
                get_logger(), "Invalid end marker: expected 0x55, got 0x%02X", end_marker);
            }
            continue;
          }

          // 检查是否是驱动器响应 (ID 1-4)
          if (can_id >= 1 && can_id <= 4 && data_len >= 6) {
            // 检查是否是速度查询响应
            if (frame_data[4] == QUERY_CMD && frame_data[5] == QUERY_DATA) {
              // 解析ERPM (4字节，大端序)
              int32_t erpm = (frame_data[6] << 24) | (frame_data[7] << 16) |
                (frame_data[8] << 8) | frame_data[9];

              // 计算RPM
              double rpm = static_cast<double>(erpm) / motor_pole_pairs_;

              // 更新驱动器速度
              drive_speeds_[can_id] = rpm;
              waiting_response_[can_id] = false;
              response_count_++;

              // 发布速度响应
              std_msgs::msg::String speed_msg;
              speed_msg.data = "Drive " + std::to_string(can_id) +
                " Speed: " + std::to_string(erpm) + " ERPM (" +
                std::to_string(rpm) + " RPM)";
              speed_pub_->publish(speed_msg);

              if (enable_debug_) {
                RCLCPP_DEBUG(
                  get_logger(), "Drive %d: ERPM=%d, RPM=%.1f", can_id, erpm, rpm);
              }
            }
          }
        }
      }
    }
  }

  void publishStatus()
  {
    std_msgs::msg::String status_msg;
    status_msg.data = std::string("Speed Query Status: ") +
      (is_connected_ ? "Connected" : "Disconnected") +
      ", Queries: " + std::to_string(query_count_) +
      ", Responses: " + std::to_string(response_count_);
    status_pub_->publish(status_msg);
  }

  void checkTimeouts()
  {
    rclcpp::Time now = this->now();
    for (int i = 1; i <= 4; i++) {
      if (waiting_response_[i]) {
        double timeout = (now - last_query_time_[i]).seconds();
        if (timeout > 1.0) {  // 1秒超时
          RCLCPP_WARN(get_logger(), "Drive %d response timeout (%.1fs)", i, timeout);
          waiting_response_[i] = false;
        }
      }
    }
  }

  // 原 run() 的 ros::Rate(10) while(ros::ok()) 循环 -> 100ms wall timer。
  //
  // 注意：这里的查询周期门控用 last_query_cycle_time_（新增独立标量成员，
  // 对应原 run() 函数体内的局部标量 `ros::Time last_query_time`），
  // 与逐驱动器 map 成员 last_query_time_（checkTimeouts()/sendSpeedQuery()
  // 专用，本函数既不读也不写它）是两个完全不同的变量，不得混用。
  void pollLoop()
  {
    // 处理接收到的数据
    processReceivedData();

    // 定期发送查询
    rclcpp::Time now = this->now();
    if ((now - last_query_cycle_time_).seconds() >= 1.0 / query_frequency_) {
      for (int i = 1; i <= 4; i++) {
        if (!waiting_response_[i]) {
          sendSpeedQuery(i);
        }
      }
      last_query_cycle_time_ = now;
    }

    // 检查超时
    checkTimeouts();

    // 发布状态信息（原 static int status_counter -> 成员 status_counter_）
    if (++status_counter_ >= 10) {  // 每秒发布一次状态
      publishStatus();
      status_counter_ = 0;
    }
  }

  // 发布者和订阅者
  rclcpp::Publisher<vehicle_can_bridge::msg::Frame>::SharedPtr can_tx_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr speed_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Subscription<vehicle_can_bridge::msg::Frame>::SharedPtr can_rx_sub_;

  // 串口对象
  vehicle_can_bridge::SerialPort serial_;

  // 参数
  std::string port_;
  int baudrate_ = 0;
  bool enable_debug_ = false;
  double query_frequency_ = 1.0;      // 查询频率 (Hz)
  int motor_pole_pairs_ = 15;         // 电机磁极对数
  double wheel_radius_ = 0.1275;      // 轮子半径 (m)

  // 状态变量
  bool is_connected_ = false;
  uint32_t query_count_ = 0;
  uint32_t response_count_ = 0;

  // 驱动器状态
  std::map<int, double> drive_speeds_;         // 驱动器ID -> RPM
  std::map<int, rclcpp::Time> last_query_time_;  // 每驱动器上次查询时间（供 checkTimeouts 用）
  std::map<int, bool> waiting_response_;       // 是否等待响应

  // 查询参数
  static constexpr uint8_t QUERY_CMD = 0x0F;   // 查询指令
  static constexpr uint8_t QUERY_DATA = 0x01;  // 查询数据

  // run() -> 100ms wall timer 替代
  rclcpp::TimerBase::SharedPtr poll_timer_;
  int status_counter_ = 0;

  // 查询周期门控：原 run() 函数体内局部标量 ros::Time last_query_time，
  // 与上面的逐驱动器 map 成员 last_query_time_ 是两个不同的变量（见文件头注释）。
  rclcpp::Time last_query_cycle_time_;

  // processReceivedData 状态：原 static std::vector<uint8_t> buffer
  std::vector<uint8_t> rx_buffer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<SpeedQueryNode>();
  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
