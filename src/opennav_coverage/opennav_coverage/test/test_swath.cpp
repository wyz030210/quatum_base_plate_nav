// Copyright (c) 2023 Open Navigation LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "opennav_coverage/robot_params.hpp"
#include "opennav_coverage/swath_generator.hpp"
#include "tf2/utils.h"
#include "fields2cover/utils/random.h"
#include <limits>

// Luckily, F2C has very high test coverage so we only need to test what we touch

class RosLockGuard
{
public:
  RosLockGuard() {rclcpp::init(0, nullptr);}
  ~RosLockGuard() {rclcpp::shutdown();}
};
RosLockGuard g_rclcpp;

namespace opennav_coverage
{

class SwathShim : public SwathGenerator
{
public:
  template<typename NodeT>
  explicit SwathShim(const NodeT & node, RobotParams * robot_params)
  : SwathGenerator(node, robot_params)
  {}

  SwathObjectivePtr createObjectiveShim(const SwathType & type)
  {
    return createObjective(type);
  }

  std::string toStringShim(const SwathType & type, const SwathAngleType & angle_type)
  {
    return toString(type, angle_type);
  }

  SwathType toTypeShim(const std::string & str)
  {
    return toType(str);
  }


  SwathAngleType toAngleTypeShim(const std::string & str)
  {
    return toAngleType(str);
  }
};

TEST(SwathTests, TestswathUtils)
{
  auto node = std::make_shared<rclcpp::Node>("test_node");
  RobotParams robot(node);
  auto generator = SwathShim(node, &robot);

  EXPECT_EQ(generator.toAngleTypeShim("FAKE"), SwathAngleType::UNKNOWN);
  EXPECT_EQ(generator.toAngleTypeShim("SET_ANGLE"), SwathAngleType::SET_ANGLE);
  EXPECT_EQ(generator.toAngleTypeShim("set_angle"), SwathAngleType::SET_ANGLE);
  EXPECT_EQ(generator.toAngleTypeShim("BRUTE_FORCE"), SwathAngleType::BRUTE_FORCE);
  EXPECT_EQ(generator.toAngleTypeShim("brute_force"), SwathAngleType::BRUTE_FORCE);

  EXPECT_EQ(generator.toTypeShim("FAKE"), SwathType::UNKNOWN);
  EXPECT_EQ(generator.toTypeShim("LENGTH"), SwathType::LENGTH);
  EXPECT_EQ(generator.toTypeShim("length"), SwathType::LENGTH);
  EXPECT_EQ(generator.toTypeShim("NUMBER"), SwathType::NUMBER);
  EXPECT_EQ(generator.toTypeShim("number"), SwathType::NUMBER);
  EXPECT_EQ(generator.toTypeShim("COVERAGE"), SwathType::COVERAGE);
  EXPECT_EQ(generator.toTypeShim("coverage"), SwathType::COVERAGE);
  EXPECT_EQ(generator.toTypeShim("NUMBER_MODIFIED"), SwathType::NUMBER_MODIFIED);
  EXPECT_EQ(generator.toTypeShim("number_modified"), SwathType::NUMBER_MODIFIED);

  EXPECT_EQ(generator.toStringShim(SwathType::NUMBER, SwathAngleType::SET_ANGLE).size(), 37u);
  EXPECT_EQ(generator.toStringShim(SwathType::COVERAGE, SwathAngleType::BRUTE_FORCE).size(), 41u);
  EXPECT_GT(generator.toStringShim(SwathType::UNKNOWN, SwathAngleType::UNKNOWN).size(), 20u);
  EXPECT_GT(generator.toStringShim(SwathType::UNKNOWN, SwathAngleType::BRUTE_FORCE).size(), 20u);

  EXPECT_EQ(
    generator.toStringShim(SwathType::NUMBER_MODIFIED, SwathAngleType::BRUTE_FORCE).size(), 48u);
  EXPECT_EQ(
    generator.toStringShim(SwathType::NUMBER_MODIFIED, SwathAngleType::SET_ANGLE).size(), 46u);

  EXPECT_TRUE(generator.createObjectiveShim(SwathType::LENGTH));
  EXPECT_TRUE(generator.createObjectiveShim(SwathType::NUMBER));
  EXPECT_TRUE(generator.createObjectiveShim(SwathType::COVERAGE));
  EXPECT_TRUE(generator.createObjectiveShim(SwathType::NUMBER_MODIFIED));
  EXPECT_FALSE(generator.createObjectiveShim(SwathType::UNKNOWN));

  generator.setSwathAngleMode("NUMBER");
  generator.setSwathMode("SET_ANGLE");
  generator.setSwathAngle(0.0);
  generator.setOVerlap(false);
  generator.setStepAngle(0.0);
}

TEST(SwathTests, IndependentSpacingPreservesOperationWidth)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    rclcpp::Parameter("operation_width", 0.60),
    rclcpp::Parameter("swath_spacing", 2.0),
    rclcpp::Parameter("default_swath_angle_type", "SET_ANGLE"),
    rclcpp::Parameter("default_swath_angle", 0.0)});
  auto node = std::make_shared<rclcpp::Node>("spacing_test", options);
  RobotParams robot(node);
  SwathGenerator generator(node, &robot);
  F2CLinearRing ring;
  for (const Point & point : std::vector<Point>{
      Point(0.0, 0.0), Point(10.0, 0.0), Point(10.0, 8.0),
      Point(0.0, 8.0), Point(0.0, 0.0)})
  {
    ring.addPoint(point);
  }
  F2CCells cells;
  cells.addGeometry(Field(ring));
  auto swaths = generator.generateSwathsByCells(
    cells, opennav_coverage_msgs::msg::SwathMode()).flatten();
  ASSERT_EQ(swaths.size(), 4u);
  for (size_t i = 0; i < swaths.size(); ++i) {
    EXPECT_DOUBLE_EQ(swaths[i].getWidth(), 0.60);
    if (i > 0) {
      EXPECT_NEAR(swaths[i].startPoint().getY() - swaths[i - 1].startPoint().getY(), 2.0, 1e-6);
    }
  }
  f2c::obj::FieldCoverage coverage;
  EXPECT_NEAR(coverage.computeCost(cells, swaths), 0.30, 1e-6);
  opennav_coverage_msgs::msg::SwathMode coverage_settings;
  coverage_settings.mode = "BRUTE_FORCE";
  coverage_settings.objective = "COVERAGE";
  coverage_settings.step_angle = 1.57079632679;
  auto best = generator.generateSwathsByCells(cells, coverage_settings).flatten();
  EXPECT_NEAR(coverage.computeCost(cells, best), 0.30, 1e-6);
  generator.setSwathSpacing(0.0);
  auto dense = generator.generateSwathsByCells(
    cells, opennav_coverage_msgs::msg::SwathMode()).flatten();
  EXPECT_GT(dense.size(), swaths.size());
  EXPECT_NEAR(dense[1].startPoint().getY() - dense[0].startPoint().getY(), 0.60, 1e-6);
  EXPECT_THROW(generator.setSwathSpacing(-1.0), std::invalid_argument);
  EXPECT_THROW(generator.setSwathSpacing(std::numeric_limits<double>::infinity()), std::invalid_argument);
}

TEST(SwathTests, TestswathGeneration)
{
  auto node = std::make_shared<rclcpp::Node>("test_node");
  RobotParams robot(node);
  auto generator = SwathShim(node, &robot);

  // Generate some toy field
  f2c::Random rand;
  auto field = rand.generateRandField(1e5, 5);
  F2CCells cells;
  cells.addGeometry(field.getField().getGeometry(0));

  // Shouldn't throw, results in valid output
  opennav_coverage_msgs::msg::SwathMode settings;
  auto swaths1 = generator.generateSwathsByCells(cells, settings);
  settings.mode = "BRUTE_FORCE";
  settings.objective = "LENGTH";
  auto swaths2 = generator.generateSwathsByCells(cells, settings);
  settings.mode = "SET_ANGLE";
  settings.objective = "NUMBER";
  auto swaths3 = generator.generateSwathsByCells(cells, settings);
  settings.mode = "BRUTE_FORCE";
  settings.objective = "NUMBER_MODIFIED";
  auto swaths4 = generator.generateSwathsByCells(cells, settings);
}

TEST(SwathTests, TestswathGenerationMultiCell)
{
  auto node = std::make_shared<rclcpp::Node>("test_node");
  RobotParams robot(node);
  auto generator = SwathShim(node, &robot);

  f2c::Random rand;
  auto field = rand.generateRandField(1e5, 5);
  Field cell = field.getField().getGeometry(0);

  opennav_coverage_msgs::msg::SwathMode settings;

  F2CCells one_cell;
  one_cell.addGeometry(cell);
  auto swaths_one = generator.generateSwathsByCells(one_cell, settings);
  EXPECT_GT(swaths_one.sizeTotal(), 0u);

  // Multi-cell input yields swaths per cell
  F2CCells cells;
  cells.addGeometry(cell);
  cells.addGeometry(cell);
  auto swaths_multi = generator.generateSwathsByCells(cells, settings);
  EXPECT_GT(swaths_multi.sizeTotal(), swaths_one.sizeTotal());
}

TEST(SwathTests, TestgenerateSwathsByCells)
{
  // Per-cell structure has one entry per cell; flatten() keeps every swath
  auto node = std::make_shared<rclcpp::Node>("test_node");
  RobotParams robot(node);
  auto generator = SwathShim(node, &robot);

  f2c::Random rand;
  auto field = rand.generateRandField(1e5, 5);
  Field cell = field.getField().getGeometry(0);
  F2CCells cells;
  cells.addGeometry(cell);
  cells.addGeometry(cell);

  opennav_coverage_msgs::msg::SwathMode settings;

  F2CSwathsByCells sbc = generator.generateSwathsByCells(cells, settings);
  EXPECT_EQ(sbc.size(), cells.size());
  EXPECT_EQ(sbc.flatten().size(), sbc.sizeTotal());
}

}  // namespace opennav_coverage
