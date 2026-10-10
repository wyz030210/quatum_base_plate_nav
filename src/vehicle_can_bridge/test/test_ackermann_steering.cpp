#include <gtest/gtest.h>

#include <cmath>

#include "vehicle_can_bridge/ackermann_steering.hpp"

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kWheelBase = 0.30;
constexpr double kMaxSteering = 30.0 * kPi / 180.0;
constexpr double kMinSpeed = 0.02;
}  // namespace

TEST(AckermannSteering, UsesBicycleModelForForwardMotion)
{
  const double result = vehicle_can_bridge::steeringAngleFromTwist(
    0.25, 0.30, kWheelBase, kMaxSteering, kMinSpeed);
  EXPECT_NEAR(result, std::atan(kWheelBase * 0.30 / 0.25), 1e-12);
}

TEST(AckermannSteering, ReversingFlipsSteeringForTheSameYawRate)
{
  const double forward = vehicle_can_bridge::steeringAngleFromTwist(
    0.25, 0.30, kWheelBase, kMaxSteering, kMinSpeed);
  const double reverse = vehicle_can_bridge::steeringAngleFromTwist(
    -0.25, 0.30, kWheelBase, kMaxSteering, kMinSpeed);
  EXPECT_NEAR(reverse, -forward, 1e-12);
}

TEST(AckermannSteering, CentersSteeringBelowMinimumSpeed)
{
  EXPECT_DOUBLE_EQ(
    vehicle_can_bridge::steeringAngleFromTwist(
      0.0, 0.50, kWheelBase, kMaxSteering, kMinSpeed),
    0.0);
}

TEST(AckermannSteering, ClampsToPhysicalSteeringLimit)
{
  EXPECT_DOUBLE_EQ(
    vehicle_can_bridge::steeringAngleFromTwist(
      0.05, 1.0, kWheelBase, kMaxSteering, kMinSpeed),
    kMaxSteering);
}

TEST(AckermannSteering, MapsPhysicalSteeringToAsymmetricServoLimits)
{
  EXPECT_DOUBLE_EQ(
    vehicle_can_bridge::servoAngleFromSteering(
      kMaxSteering, kMaxSteering, 40.0, 90.0, 190.0),
    40.0);
  EXPECT_DOUBLE_EQ(
    vehicle_can_bridge::servoAngleFromSteering(
      -kMaxSteering, kMaxSteering, 40.0, 90.0, 190.0),
    190.0);
  EXPECT_DOUBLE_EQ(
    vehicle_can_bridge::servoAngleFromSteering(
      0.0, kMaxSteering, 40.0, 90.0, 190.0),
    90.0);
}
