#ifndef VEHICLE_CAN_BRIDGE__ACKERMANN_STEERING_HPP_
#define VEHICLE_CAN_BRIDGE__ACKERMANN_STEERING_HPP_

#include <algorithm>
#include <cmath>

namespace vehicle_can_bridge
{

inline double steeringAngleFromTwist(
  double linear_speed,
  double yaw_rate,
  double wheel_base,
  double max_steering_angle,
  double min_steering_speed)
{
  // An Ackermann vehicle cannot generate yaw while stopped. Centering the
  // steering at very small speeds also avoids division noise near zero.
  if (std::abs(linear_speed) < min_steering_speed) {
    return 0.0;
  }

  // Bicycle model: yaw_rate = v / wheel_base * tan(steering_angle).
  // Keeping the sign of linear_speed is important when reversing.
  const double steering_angle = std::atan(wheel_base * yaw_rate / linear_speed);
  return std::max(
    -max_steering_angle,
    std::min(max_steering_angle, steering_angle));
}

inline double servoAngleFromSteering(
  double steering_angle,
  double max_steering_angle,
  double servo_left_limit,
  double servo_center_angle,
  double servo_right_limit)
{
  if (max_steering_angle <= 0.0) {
    return servo_center_angle;
  }

  const double normalized = std::min(
    std::abs(steering_angle) / max_steering_angle, 1.0);
  if (steering_angle > 0.0) {
    return servo_center_angle -
           normalized * (servo_center_angle - servo_left_limit);
  }
  if (steering_angle < 0.0) {
    return servo_center_angle +
           normalized * (servo_right_limit - servo_center_angle);
  }
  return servo_center_angle;
}

}  // namespace vehicle_can_bridge

#endif  // VEHICLE_CAN_BRIDGE__ACKERMANN_STEERING_HPP_
