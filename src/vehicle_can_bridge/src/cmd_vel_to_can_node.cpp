// ROS2 (Humble) port of the original ROS1 vehicle_can_bridge/src/cmd_vel_to_can_node.cpp.
//
// Built on the original CmdVelToCANNode ROS1->ROS2 port, with a common
// automatic-command stop gate and fresh-feedback state contract:
//
//  * ros::Publisher/Subscriber -> rclcpp::Publisher/Subscription, same
//    topic names and queue depths as the original ("/cmd_vel" sub,
//    "/can_tx" and "/control_mode" pubs).
//  * This node has no serial port and no run() loop -- the original was
//    just ros::spin() -- so this is the most direct port in the whole
//    plan; main() maps 1:1 to rclcpp::init/spin/shutdown.
//  * sendCombinedControlCommand() preserves the original ERPM and asymmetric
//    servo calibration, but converts ROS yaw-rate commands to a physical
//    steering angle with the Ackermann bicycle model.
//  * The original's #define macro constants (MOTOR_POLE_PAIRS,
//    GEAR_RATIO, ROS_CONTROL_CAN_ID, CMD_TYPE_COMBINED,
//    SERVO_LEFT_LIMIT, SERVO_RIGHT_LIMIT, SERVO_CENTER_ANGLE) become
//    anonymous-namespace constexpr constants with identical names/values.

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <vehicle_can_bridge/msg/frame.hpp>
#include <vehicle_can_bridge/ackermann_steering.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <chrono>
#include <memory>
#include <stdexcept>
#include <sstream>
#include <iomanip>
#include <string>

namespace
{
// 电机控制相关常量 - 与STM32端保持一致
constexpr int MOTOR_POLE_PAIRS = 15;      // 电机极对数
constexpr float GEAR_RATIO = 1.0f;        // 减速比

// ROS控制CAN ID和组合控制指令码
constexpr int ROS_CONTROL_CAN_ID = 0x06;  // ROS控制消息CAN ID，STM32接收
constexpr int CMD_TYPE_COMBINED = 0x01;   // 组合控制指令码（线速度+角速度）

// 舵机角度校准常量
constexpr double SERVO_LEFT_LIMIT = 40.0;    // 左转限位角度（度）
constexpr double SERVO_RIGHT_LIMIT = 190.0;  // 右转限位角度（度）
constexpr double SERVO_CENTER_ANGLE = 90.0;  // 中间位置角度（度）
constexpr double SPEED_LIMIT_EPSILON = 1e-6;
}  // namespace

class CmdVelToCANNode : public rclcpp::Node
{
public:
  CmdVelToCANNode()
  : rclcpp::Node("cmd_vel_to_can_node")
  {
    // 初始化参数
    initParameters();

    // 订阅cmd_vel话题
    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", rclcpp::QoS(10),
      std::bind(&CmdVelToCANNode::cmdVelCallback, this, std::placeholders::_1));

    // 发布CAN消息
    can_pub_ = create_publisher<vehicle_can_bridge::msg::Frame>("/can_tx", rclcpp::QoS(10));

    // 发布状态信息
    status_pub_ = create_publisher<std_msgs::msg::Int32>("/control_mode", rclcpp::QoS(10));
    command_timeout_sec_ = declare_parameter<double>("command_timeout_sec", 1.0);
    task_control_required_ = declare_parameter<bool>("task_control_required", false);
    heartbeat_timeout_sec_ = declare_parameter<double>("heartbeat_timeout_sec", 1.0);
    feedback_required_ = declare_parameter<bool>("feedback_required", true);
    raw_feedback_required_ = declare_parameter<bool>("raw_feedback_required", task_control_required_);
    raw_feedback_frame_id_ = declare_parameter<std::string>("raw_feedback_frame_id", "odom");
    raw_feedback_child_frame_id_ = declare_parameter<std::string>("raw_feedback_child_frame_id", "base_link");
    if (raw_feedback_frame_id_.empty() || raw_feedback_child_frame_id_.empty()) {
      throw std::invalid_argument("raw feedback frames must not be empty");
    }
    feedback_timeout_sec_ = declare_parameter<double>("feedback_timeout_sec", 1.0);
    feedback_future_tolerance_sec_ =
      declare_parameter<double>("feedback_future_tolerance_sec", 0.1);
    stopped_hold_sec_ = declare_parameter<double>("stopped_hold_sec", 1.0);
    stopped_linear_threshold_ = declare_parameter<double>("stopped_linear_threshold", 0.01);
    stopped_angular_threshold_ = declare_parameter<double>("stopped_angular_threshold", 0.02);
    stopping_timeout_sec_ = declare_parameter<double>("stopping_timeout_sec", 5.0);
    const auto publish_period = declare_parameter<double>("safety_publish_period_sec", 0.05);
    for (const auto & parameter : {
        "command_timeout_sec", "heartbeat_timeout_sec", "feedback_timeout_sec",
        "stopped_hold_sec", "stopping_timeout_sec", "safety_publish_period_sec"})
    {
      const double value = get_parameter(parameter).as_double();
      if (!std::isfinite(value) || value <= 0.0) {
        throw std::invalid_argument(std::string(parameter) + " must be finite and positive");
      }
    }
    for (const auto & parameter : {
        "feedback_future_tolerance_sec", "stopped_linear_threshold", "stopped_angular_threshold"})
    {
      const double value = get_parameter(parameter).as_double();
      if (!std::isfinite(value) || value < 0.0) {
        throw std::invalid_argument(std::string(parameter) + " must be finite and nonnegative");
      }
    }
    stop_requested_ = task_control_required_;
    stop_source_ns_ = now().nanoseconds();
    state_pub_ = create_publisher<std_msgs::msg::String>("/task_control/state", 10);
    stop_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/task_control/stop", 10, [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
        enforceTimeouts();
        stop_requested_ = msg->data || !ready();
        beginStop();
        sendCombinedControlCommand(0.0, 0.0);
      });
    feedback_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      declare_parameter<std::string>("feedback_topic", "/odometry/filtered_odom"),
      rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {feedbackCallback(*msg);});
    raw_feedback_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      declare_parameter<std::string>("raw_feedback_topic", "/Odometry"),
      rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {rawFeedbackCallback(*msg);});
    emergency_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/task_control/emergency_stop", 10,
      [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
        emergency_input_ = msg->data;
        emergencyInputChanged();
      });
    legacy_emergency_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/coverage/emergency_stop", 10,
      [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
        legacy_emergency_input_ = msg->data;
        emergencyInputChanged();
      });
    manual_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/vehicle/remote_manual_override", 10,
      [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
        manual_override_ = msg->data;
        if (manual_override_) {
          stop_requested_ = true;
          beginStop();
          sendCombinedControlCommand(0.0, 0.0);
        }
      });
    reset_srv_ = create_service<std_srvs::srv::Trigger>(
      "/task_control/reset_stop",
      [this](std_srvs::srv::Trigger::Request::ConstSharedPtr,
      std_srvs::srv::Trigger::Response::SharedPtr response) {
        enforceTimeouts();
        if (emergency_input_ || legacy_emergency_input_ || manual_override_ ||
          (task_control_required_ && !heartbeatFresh()) ||
          (feedback_required_ && (!feedbackFresh() || !stopped_)))
        {
          response->success = false;
          response->message = "Stop inputs must be clear and fresh feedback must confirm rest";
          return;
        }
        emergency_held_ = false;
        fault_.clear();
        stop_requested_ = true;
        beginStop();
        response->success = true;
        response->message = "Stop reset; explicit run request and a new velocity are required";
      });
    heartbeat_sub_ = create_subscription<std_msgs::msg::Empty>(
      "/task_control/heartbeat", 10, [this](std_msgs::msg::Empty::ConstSharedPtr) {
        enforceTimeouts();
        heartbeat_received_ = true;
        last_heartbeat_ = std::chrono::steady_clock::now();
      });
    safety_timer_ = create_wall_timer(
      std::chrono::duration<double>(publish_period), [this]() {safetyTick();});

    RCLCPP_INFO(get_logger(), "Raw local measurement freshness required=%s topic=%s frames=%s->%s (no raw twist inference)",
      raw_feedback_required_ ? "true" : "false", get_parameter("raw_feedback_topic").as_string().c_str(),
      raw_feedback_frame_id_.c_str(), raw_feedback_child_frame_id_.c_str());
    RCLCPP_INFO(get_logger(), "CmdVelToCANNode initialized");
    RCLCPP_INFO(
      get_logger(), "Vehicle parameters: wheel_base=%.2fm, wheel_radius=%.3fm",
      wheel_base_, wheel_radius_);
    RCLCPP_INFO(
      get_logger(),
      "Control limits: forward=%.2fm/s, reverse=%.2fm/s, yaw_rate=%.2frad/s, "
      "steering=%.1fdeg",
      max_forward_speed_, max_reverse_speed_, max_angular_speed_, max_steering_angle_deg_);
    RCLCPP_INFO(get_logger(), "CAN ID: ROS_CONTROL=0x%02X (to STM32)", ROS_CONTROL_CAN_ID);
    RCLCPP_INFO(
      get_logger(),
      "Safety: task_control_required=%s, feedback_required=%s, command_timeout=%.3fs, "
      "heartbeat_timeout=%.3fs, feedback_timeout=%.3fs, stop_hold=%.3fs, "
      "stop_timeout=%.3fs, zero_linear=%.3fm/s, zero_angular=%.3frad/s, publish=%.3fs",
      task_control_required_ ? "true" : "false", feedback_required_ ? "true" : "false",
      command_timeout_sec_, heartbeat_timeout_sec_, feedback_timeout_sec_, stopped_hold_sec_,
      stopping_timeout_sec_, stopped_linear_threshold_, stopped_angular_threshold_, publish_period);
  }

private:
  void initParameters()
  {
    // 车辆物理参数
    wheel_base_ = declare_parameter<double>("wheel_base", 0.3);
    wheel_radius_ = declare_parameter<double>("wheel_radius", 0.1275);  // 255mm直径
    // Preserve the legacy symmetric limit for older launch files while allowing
    // the indoor launch to provide safer asymmetric limits.
    const double legacy_max_linear_speed =
      declare_parameter<double>("max_linear_speed", 2.0);
    max_forward_speed_ =
      declare_parameter<double>("max_forward_speed", legacy_max_linear_speed);
    max_reverse_speed_ =
      declare_parameter<double>("max_reverse_speed", legacy_max_linear_speed);
    max_angular_speed_ = declare_parameter<double>("max_angular_speed", 1.0);
    max_steering_angle_deg_ = declare_parameter<double>("max_steering_angle_deg", 30.0);
    min_steering_speed_ = declare_parameter<double>("min_steering_speed", 0.02);

    // 控制参数
    linear_scale_ = declare_parameter<double>("linear_scale", 1.0);
    angular_scale_ = declare_parameter<double>("angular_scale", 1.0);

    auto_mode_ = false;
  }

  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    enforceTimeouts();
    if (stop_requested_ || !ready()) {
      sendCombinedControlCommand(0.0, 0.0);
      return;
    }
    if (!std::isfinite(msg->linear.x * linear_scale_) ||
      !std::isfinite(msg->angular.z * angular_scale_))
    {
      latchFault("invalid_command");
      sendCombinedControlCommand(0.0, 0.0);
      return;
    }
    if (wheel_radius_ <= 0.0 || max_forward_speed_ <= 0.0 ||
      max_reverse_speed_ <= 0.0 || max_angular_speed_ <= 0.0 ||
      max_steering_angle_deg_ <= 0.0 || min_steering_speed_ < 0.0)
    {
      throw std::invalid_argument("wheel radius and speed limits must be positive");
    }

    // 前进、倒车和期望横摆角速度分别限幅。
    const double requested_linear_x = msg->linear.x * linear_scale_;
    const double linear_x = std::max(
      -max_reverse_speed_, std::min(max_forward_speed_, requested_linear_x));
    const double angular_z = std::max(
      -max_angular_speed_, std::min(max_angular_speed_, msg->angular.z * angular_scale_));

    // 记录速度限制信息
    if (requested_linear_x > max_forward_speed_ + SPEED_LIMIT_EPSILON ||
      requested_linear_x < -max_reverse_speed_ - SPEED_LIMIT_EPSILON)
    {
      RCLCPP_WARN(
        get_logger(), "线速度超出范围[-%.2f, %.2f]m/s，已限制为%.2fm/s",
        max_reverse_speed_, max_forward_speed_, linear_x);
    }

    // 发送组合控制指令（线速度+角速度）给STM32
    if (linear_x == 0.0) {
      beginStop();
    } else {
      stopping_ = false;
      stopped_ = false;
      zero_tracking_ = false;
      command_received_ = true;
      last_command_ = std::chrono::steady_clock::now();
    }
    sendCombinedControlCommand(linear_x, angular_z);

    // 发布控制模式状态
    std_msgs::msg::Int32 mode_msg;
    mode_msg.data = auto_mode_ ? 1 : 0;
    status_pub_->publish(mode_msg);
  }

  bool ready() const
  {
    return fault_.empty() && !emergency_held_ && !manual_override_ &&
           (!feedback_required_ || feedbackFresh()) &&
           (!task_control_required_ || heartbeatFresh());
  }

  bool heartbeatFresh() const
  {
    return heartbeat_received_ && std::chrono::duration<double>(
      std::chrono::steady_clock::now() - last_heartbeat_).count() <= heartbeat_timeout_sec_;
  }

  void emergencyInputChanged()
  {
    if (emergency_input_ || legacy_emergency_input_) {
      emergency_held_ = true;
      stop_requested_ = true;
      beginStop();
      sendCombinedControlCommand(0.0, 0.0);
    }
  }

  bool rawFeedbackFresh() const
  {
    if (!raw_feedback_required_) return true;
    if (!raw_feedback_valid_) return false;
    const double receive_age = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - last_raw_feedback_).count();
    const double source_age = (now().nanoseconds() - raw_feedback_stamp_ns_) * 1e-9;
    return receive_age <= feedback_timeout_sec_ && source_age <= feedback_timeout_sec_ &&
      source_age >= -feedback_future_tolerance_sec_;
  }

  void rawFeedbackCallback(const nav_msgs::msg::Odometry & msg)
  {
    enforceTimeouts();
    const int64_t stamp = static_cast<int64_t>(msg.header.stamp.sec) * 1000000000LL +
      msg.header.stamp.nanosec;
    const double source_age = (now().nanoseconds() - stamp) * 1e-9;
    const auto & pose = msg.pose.pose;
    const auto & q = pose.orientation;
    const double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    raw_feedback_valid_ = msg.header.frame_id == raw_feedback_frame_id_ &&
      msg.child_frame_id == raw_feedback_child_frame_id_ &&
      msg.header.stamp.sec >= 0 && msg.header.stamp.nanosec < 1000000000U &&
      stamp > 0 && stamp > last_raw_valid_stamp_ns_ && std::isfinite(pose.position.x) &&
      std::isfinite(pose.position.y) && std::isfinite(pose.position.z) &&
      std::isfinite(norm) && std::abs(norm - 1.0) < 0.01 &&
      source_age <= feedback_timeout_sec_ && source_age >= -feedback_future_tolerance_sec_;
    raw_feedback_stamp_ns_ = stamp;
    last_raw_feedback_ = std::chrono::steady_clock::now();
    if (raw_feedback_valid_) last_raw_valid_stamp_ns_ = stamp;
    // Raw FAST-LIO carries no measured twist. It is evidence that the EKF's
    // original measurements are current, never a zero-speed measurement.
  }

  bool feedbackFresh() const
  {
    if (!feedback_valid_ || !rawFeedbackFresh()) {
      return false;
    }
    const auto receive_age = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - last_feedback_).count();
    const double source_age = (now().nanoseconds() - feedback_stamp_ns_) * 1e-9;
    return receive_age <= feedback_timeout_sec_ && source_age <= feedback_timeout_sec_ &&
           source_age >= -feedback_future_tolerance_sec_;
  }

  void beginStop()
  {
    command_received_ = false;
    if (!stopping_) {
      stopping_ = true;
      stopped_ = false;
      zero_tracking_ = false;
      stop_started_ = std::chrono::steady_clock::now();
      stop_source_ns_ = now().nanoseconds();
      stop_deadline_active_ = true;
    }
  }

  void latchFault(const std::string & reason)
  {
    if (fault_.empty()) {
      fault_ = reason;
      RCLCPP_ERROR(get_logger(), "Automatic control inhibited: %s", reason.c_str());
    }
    stop_requested_ = true;
    beginStop();
    sendCombinedControlCommand(0.0, 0.0);
  }

  void feedbackCallback(const nav_msgs::msg::Odometry & msg)
  {
    // Detect a gap before replacing the expired sample. A late arrival must
    // not hide an active-run fault between periodic safety checks.
    enforceTimeouts();
    const auto received_at = std::chrono::steady_clock::now();
    const bool continuous = feedbackFresh();
    // A malformed source stamp must inhibit control rather than throw out of
    // the callback and leave the last physical command running.
    const int64_t stamp = static_cast<int64_t>(msg.header.stamp.sec) * 1000000000LL +
      msg.header.stamp.nanosec;
    const double source_age = (now().nanoseconds() - stamp) * 1e-9;
    const auto & twist = msg.twist.twist;
    bool finite = std::isfinite(twist.linear.x) && std::isfinite(twist.linear.y) &&
      std::isfinite(twist.linear.z) && std::isfinite(twist.angular.x) &&
      std::isfinite(twist.angular.y) && std::isfinite(twist.angular.z);
    for (const double covariance : msg.twist.covariance) {
      finite = finite && std::isfinite(covariance);
    }
    for (size_t index = 0; index < 36; index += 7) {
      finite = finite && msg.twist.covariance[index] >= 0.0;
    }
    feedback_received_ = true;
    last_feedback_ = received_at;
    feedback_stamp_ns_ = stamp;
    feedback_valid_ = finite && msg.header.stamp.sec >= 0 &&
      msg.header.stamp.nanosec < 1000000000U && stamp > 0 && stamp > last_valid_stamp_ns_ &&
      source_age <= feedback_timeout_sec_ && source_age >= -feedback_future_tolerance_sec_;
    if (feedback_valid_) {
      last_valid_stamp_ns_ = stamp;
    }
    const bool near_zero = std::hypot(twist.linear.x, twist.linear.y) <=
      stopped_linear_threshold_ && std::abs(twist.linear.z) <= stopped_linear_threshold_ &&
      std::hypot(twist.angular.x, twist.angular.y) <= stopped_angular_threshold_ &&
      std::abs(twist.angular.z) <= stopped_angular_threshold_;
    if (!feedback_valid_ || !near_zero || !stopping_ || stamp < stop_source_ns_) {
      zero_tracking_ = false;
      stopped_ = false;
      return;
    }
    if (!continuous || !zero_tracking_) {
      zero_tracking_ = true;
      zero_since_ = received_at;
      zero_source_ns_ = stamp;
      stopped_ = false;
    }
    // Only a newly received sample can extend the hold interval; a timer cannot
    // turn a single zero sample into confirmation of a physical stop.
    stopped_ = std::chrono::duration<double>(received_at - zero_since_).count() >=
      stopped_hold_sec_ && (stamp - zero_source_ns_) * 1e-9 >= stopped_hold_sec_;
    if (stopped_) {
      stop_deadline_active_ = false;
    }
  }

  void enforceTimeouts()
  {
    if (task_control_required_ && !stop_requested_ && !heartbeatFresh()) {
      latchFault("heartbeat_timeout");
    }
    if (!feedbackFresh()) {
      zero_tracking_ = false;
      stopped_ = false;
      if (command_received_ && feedback_required_) {
        latchFault("feedback_timeout_or_invalid");
      }
    }
    if (command_received_ &&
      std::chrono::duration<double>(std::chrono::steady_clock::now() - last_command_).count() >
      command_timeout_sec_)
    {
      if (task_control_required_) {
        latchFault("command_timeout");
      } else {
        beginStop();
      }
    }
    if (stop_deadline_active_ && !stopped_ &&
      std::chrono::duration<double>(std::chrono::steady_clock::now() - stop_started_).count() >
      stopping_timeout_sec_)
    {
      latchFault("stopping_timeout");
    }
  }

  void safetyTick()
  {
    enforceTimeouts();
    if (!ready() || stop_requested_ || !command_received_) {
      sendCombinedControlCommand(0.0, 0.0);
    }
    std_msgs::msg::String state;
    std::ostringstream json;
    json << std::boolalpha << "{\"ready\":" << ready() <<
      ",\"stopped\":" << (stopped_ && feedbackFresh()) << ",\"stop_requested\":" << stop_requested_ <<
      ",\"manual_override\":" << manual_override_ << ",\"emergency_stop\":" <<
      emergency_held_ << ",\"fault\":\"" << fault_ <<
      "\",\"feedback_valid\":" << feedbackFresh() << ",\"feedback_age_sec\":";
    if (feedback_received_) {
      json << std::max(0.0, std::max(
          std::chrono::duration<double>(std::chrono::steady_clock::now() - last_feedback_).count(),
          (now().nanoseconds() - feedback_stamp_ns_) * 1e-9));
    } else {
      json << "null";
    }
    json << ",\"feedback_stamp_sec\":";
    if (feedback_received_) {
      json << std::fixed << std::setprecision(9) << feedback_stamp_ns_ * 1e-9;
    } else {
      json << "null";
    }
    json << ",\"raw_feedback_required\":" << raw_feedback_required_ <<
      ",\"raw_feedback_valid\":" << rawFeedbackFresh() << "}";
    state.data = json.str();
    state_pub_->publish(state);
  }

  void sendCombinedControlCommand(double linear_x, double angular_z)
  {
    // 1. 线速度转换为电机ERPM
    // ERPM = 线速度 * 60 / (2 * PI * 轮子半径) * 电机极对数 * 减速比
    // 与STM32端公式保持一致：erpm_f = (linear_speed * 60.0f) / (2.0f * 3.14159f * WHEEL_RADIUS_M) * MOTOR_POLE_PAIRS * GEAR_RATIO
    int32_t motor_erpm = static_cast<int32_t>(
      linear_x * 60.0 / (2.0 * M_PI * wheel_radius_) * MOTOR_POLE_PAIRS * GEAR_RATIO);

    int32_t max_forward_erpm = static_cast<int32_t>(
      max_forward_speed_ * 60.0 / (2.0 * M_PI * wheel_radius_) * MOTOR_POLE_PAIRS * GEAR_RATIO);
    int32_t max_reverse_erpm = static_cast<int32_t>(
      max_reverse_speed_ * 60.0 / (2.0 * M_PI * wheel_radius_) * MOTOR_POLE_PAIRS * GEAR_RATIO);
    motor_erpm = std::max(-max_reverse_erpm, std::min(max_forward_erpm, motor_erpm));

    // 2. ROS Twist carries yaw rate, not steering position. Convert it using
    // the Ackermann bicycle model before applying the calibrated servo map.
    const double max_steering_angle = max_steering_angle_deg_ * M_PI / 180.0;
    const double steering_angle = vehicle_can_bridge::steeringAngleFromTwist(
      linear_x, angular_z, wheel_base_, max_steering_angle, min_steering_speed_);
    double servo_angle = vehicle_can_bridge::servoAngleFromSteering(
      steering_angle, max_steering_angle, SERVO_LEFT_LIMIT,
      SERVO_CENTER_ANGLE, SERVO_RIGHT_LIMIT);

    // 确保角度在有效范围内
    servo_angle = std::max(SERVO_LEFT_LIMIT, std::min(SERVO_RIGHT_LIMIT, servo_angle));

    // 3. 构建组合控制消息发送给STM32
    vehicle_can_bridge::msg::Frame combined_msg;
    combined_msg.id = ROS_CONTROL_CAN_ID;  // 发送给STM32的CAN ID
    combined_msg.dlc = 8;
    combined_msg.data[0] = CMD_TYPE_COMBINED;  // 组合控制指令码

    // 电机ERPM (4字节，大端序) - STM32会转发给布瑞特驱动器
    combined_msg.data[1] = static_cast<uint8_t>((motor_erpm >> 24) & 0xFF);
    combined_msg.data[2] = static_cast<uint8_t>((motor_erpm >> 16) & 0xFF);
    combined_msg.data[3] = static_cast<uint8_t>((motor_erpm >> 8) & 0xFF);
    combined_msg.data[4] = static_cast<uint8_t>(motor_erpm & 0xFF);

    // 舵机角度 (2字节，大端序) - STM32会转换为PWM信号
    int16_t servo_angle_int = static_cast<int16_t>(servo_angle);
    combined_msg.data[5] = static_cast<uint8_t>((servo_angle_int >> 8) & 0xFF);
    combined_msg.data[6] = static_cast<uint8_t>(servo_angle_int & 0xFF);

    // 保留字节
    combined_msg.data[7] = 0x00;

    // 发送组合控制消息给STM32
    can_pub_->publish(combined_msg);

    RCLCPP_DEBUG(
      get_logger(),
      "Combined Control to STM32: linear=%.2f, yaw_rate=%.2f, steering=%.3f -> "
      "motor_erpm=%d, servo_angle=%.1f",
      linear_x, angular_z, steering_angle, motor_erpm, servo_angle);
  }

  // 发布者和订阅者
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<vehicle_can_bridge::msg::Frame>::SharedPtr can_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr status_pub_;

  // 车辆参数
  double wheel_base_ = 0.0;         // 轮距 (m)
  double wheel_radius_ = 0.0;       // 轮子半径 (m)
  double max_forward_speed_ = 0.0;  // 最大前进速度 (m/s)
  double max_reverse_speed_ = 0.0;  // 最大倒车速度绝对值 (m/s)
  double max_angular_speed_ = 0.0;  // 最大角速度 (rad/s)
  double max_steering_angle_deg_ = 0.0;  // 最大前轮转角 (deg)
  double min_steering_speed_ = 0.0;      // 低于此速度时回正舵机 (m/s)

  // 控制参数
  double linear_scale_ = 1.0;   // 线速度缩放因子
  double angular_scale_ = 1.0;  // 角速度缩放因子

  // 控制模式
  bool auto_mode_ = false;  // 自动模式标志
  double command_timeout_sec_ = 1.0;
  bool command_received_ = false;
  std::chrono::steady_clock::time_point last_command_;
  rclcpp::TimerBase::SharedPtr safety_timer_;
  bool task_control_required_ = false;
  bool stop_requested_ = false;
  bool heartbeat_received_ = false;
  double heartbeat_timeout_sec_ = 1.0;
  std::chrono::steady_clock::time_point last_heartbeat_;
  std::string fault_;
  bool emergency_input_ = false;
  bool legacy_emergency_input_ = false;
  bool emergency_held_ = false;
  bool manual_override_ = false;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr manual_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr legacy_emergency_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;
  bool raw_feedback_required_ = false;
  std::string raw_feedback_frame_id_;
  std::string raw_feedback_child_frame_id_;
  bool raw_feedback_valid_ = false;
  int64_t raw_feedback_stamp_ns_ = 0;
  int64_t last_raw_valid_stamp_ns_ = 0;
  std::chrono::steady_clock::time_point last_raw_feedback_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr raw_feedback_sub_;
  bool feedback_required_ = true;
  bool feedback_received_ = false;
  bool feedback_valid_ = false;
  double feedback_timeout_sec_ = 1.0;
  double feedback_future_tolerance_sec_ = 0.1;
  double stopped_hold_sec_ = 1.0;
  double stopped_linear_threshold_ = 0.01;
  double stopped_angular_threshold_ = 0.02;
  double stopping_timeout_sec_ = 5.0;
  bool stopping_ = true;
  bool stopped_ = false;
  bool zero_tracking_ = false;
  bool stop_deadline_active_ = false;
  int64_t feedback_stamp_ns_ = 0;
  int64_t last_valid_stamp_ns_ = 0;
  int64_t stop_source_ns_ = 0;
  int64_t zero_source_ns_ = 0;
  std::chrono::steady_clock::time_point last_feedback_;
  std::chrono::steady_clock::time_point zero_since_;
  std::chrono::steady_clock::time_point stop_started_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr feedback_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stop_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr heartbeat_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<CmdVelToCANNode>();
    rclcpp::spin(node);
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("cmd_vel_to_can_node"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();

  return 0;
}
