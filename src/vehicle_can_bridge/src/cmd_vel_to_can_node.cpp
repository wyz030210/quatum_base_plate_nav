// ROS2 (Humble) port of the original ROS1 vehicle_can_bridge/src/cmd_vel_to_can_node.cpp.
//
// This is a line-by-line port of the original CmdVelToCANNode class (Task 4
// of the vehicle_can_bridge ROS1->ROS2 port plan):
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
#include <geometry_msgs/msg/twist.hpp>
#include <vehicle_can_bridge/msg/frame.hpp>
#include <vehicle_can_bridge/ackermann_steering.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>

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
    sendCombinedControlCommand(linear_x, angular_z);

    // 发布控制模式状态
    std_msgs::msg::Int32 mode_msg;
    mode_msg.data = auto_mode_ ? 1 : 0;
    status_pub_->publish(mode_msg);
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
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<CmdVelToCANNode>();
  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
