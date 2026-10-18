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

#include <cmath>
#include <stdexcept>
#include <vector>
#include <string>

#include "opennav_coverage/swath_generator.hpp"

namespace opennav_coverage
{

namespace
{

// Coverage objectives must score the tool width, not the placement pitch.
class OperationWidthCoverageObjective : public f2c::obj::FieldCoverage
{
public:
  explicit OperationWidthCoverageObjective(double width) : width_(width) {}
  using f2c::obj::FieldCoverage::computeCost;

  double computeCost(const F2CCell & cell, const F2CSwaths & input) override
  {
    if (input.size() == 0) {return 0.0;}
    auto physical_swaths = input.clone();
    for (auto & swath : physical_swaths) {swath.setWidth(width_);}
    return f2c::obj::FieldCoverage::computeCost(cell, physical_swaths);
  }

private:
  double width_;
};

}  // namespace

SwathGenerator::ResolvedSwathParams SwathGenerator::resolveSwathParams(
  const opennav_coverage_msgs::msg::SwathMode & settings)
{
  SwathType action_type = toType(settings.objective);
  SwathAngleType action_angle_type = toAngleType(settings.mode);
  ResolvedSwathParams p;

  // If not set by action, use default mode
  if (action_type == SwathType::UNKNOWN && action_angle_type == SwathAngleType::UNKNOWN) {
    action_type = default_type_;
    p.angle_type = default_angle_type_;
    p.objective = default_objective_;
    p.swath_angle = default_swath_angle_;
    p.step_angle = default_step_angle_;
  } else {
    p.angle_type = action_angle_type;
    p.objective = createObjective(action_type);
    p.swath_angle = settings.best_angle;
    p.step_angle = settings.step_angle;
  }

  RCLCPP_DEBUG(
    logger_, "Generating Swaths with: %s", toString(action_type, p.angle_type).c_str());
  return p;
}

F2CSwathsByCells SwathGenerator::generateSwathsByCells(
  const F2CCells & cells, const opennav_coverage_msgs::msg::SwathMode & settings)
{
  ResolvedSwathParams p = resolveSwathParams(settings);
  const double op_width = robot_params_->getOperationWidth();
  if (!std::isfinite(op_width) || op_width <= 0.0) {
    throw std::invalid_argument("Operation width must be finite and positive.");
  }
  const double spacing = swath_spacing_ > 0.0 ? swath_spacing_ : op_width;
  F2CSwathsByCells swaths;
  generator_->setAllowOverlap(default_allow_overlap_);
  switch (p.angle_type) {
    case SwathAngleType::BRUTE_FORCE:
      if (!p.objective) {
        throw CoverageException("No valid swath mode set! Options: LENGTH, NUMBER, COVERAGE.");
      }
      generator_->setStepAngle(p.step_angle);
      if (std::dynamic_pointer_cast<f2c::obj::FieldCoverage>(p.objective)) {
        OperationWidthCoverageObjective objective(op_width);
        swaths = generator_->generateBestSwaths(objective, spacing, cells);
      } else {
        swaths = generator_->generateBestSwaths(*p.objective, spacing, cells);
      }
      break;
    case SwathAngleType::SET_ANGLE:
      swaths = generator_->generateSwaths(p.swath_angle, spacing, cells);
      break;
    default:
      throw CoverageException("No valid swath angle mode set! Options: BRUTE_FORCE, SET_ANGLE.");
  }
  // F2C uses one width for both placement and coverage. Restore the actual tool width.
  for (auto & cell_swaths : swaths) {
    for (auto & swath : cell_swaths) {
      swath.setWidth(op_width);
    }
  }
  return swaths;
}

void SwathGenerator::setSwathSpacing(double spacing)
{
  if (!std::isfinite(spacing) || spacing < 0.0) {
    throw std::invalid_argument("Swath spacing must be finite and nonnegative.");
  }
  swath_spacing_ = spacing;
}

void SwathGenerator::setSwathMode(const std::string & new_mode)
{
  default_type_ = toType(new_mode);
  default_objective_ = createObjective(default_type_);
}

void SwathGenerator::setSwathAngleMode(const std::string & new_mode)
{
  default_angle_type_ = toAngleType(new_mode);
}

SwathObjectivePtr SwathGenerator::createObjective(const SwathType & type)
{
  switch (type) {
    case SwathType::LENGTH:
      return std::move(std::make_shared<f2c::obj::SwathLength>());
    case SwathType::NUMBER:
      return std::move(std::make_shared<f2c::obj::NSwath>());
    case SwathType::COVERAGE:
      return std::move(std::make_shared<f2c::obj::FieldCoverage>());
    case SwathType::NUMBER_MODIFIED:
      return std::move(std::make_shared<f2c::obj::NSwathModified>());
    default:
      RCLCPP_WARN(logger_, "Unknown swath type set!");
      return SwathObjectivePtr{nullptr};
  }
}

std::string SwathGenerator::toString(const SwathType & type, const SwathAngleType & angle_type)
{
  std::string str;
  switch (type) {
    case SwathType::LENGTH:
      str = "Length";
      break;
    case SwathType::NUMBER:
      str = "Number";
      break;
    case SwathType::COVERAGE:
      str = "Coverage";
      break;
    case SwathType::NUMBER_MODIFIED:
      str = "Number Modified";
      break;
    default:
      str = "Unknown";
      break;
  }

  str += " Objective and ";

  switch (angle_type) {
    case SwathAngleType::SET_ANGLE:
      str += "Set Angle";
      break;
    case SwathAngleType::BRUTE_FORCE:
      str += "Brute Force";
      break;
    default:
      str += "Unknown";
      break;
  }

  str += " angle.";
  return str;
}

SwathType SwathGenerator::toType(const std::string & str)
{
  std::string mode_str = str;
  util::toUpper(mode_str);
  if (mode_str == "LENGTH") {
    return SwathType::LENGTH;
  } else if (mode_str == "NUMBER") {
    return SwathType::NUMBER;
  } else if (mode_str == "COVERAGE") {
    return SwathType::COVERAGE;
  } else if (mode_str == "NUMBER_MODIFIED") {
    return SwathType::NUMBER_MODIFIED;
  } else {
    return SwathType::UNKNOWN;
  }
}

SwathAngleType SwathGenerator::toAngleType(const std::string & str)
{
  std::string mode_str = str;
  util::toUpper(mode_str);
  if (mode_str == "SET_ANGLE") {
    return SwathAngleType::SET_ANGLE;
  } else if (mode_str == "BRUTE_FORCE") {
    return SwathAngleType::BRUTE_FORCE;
  } else {
    return SwathAngleType::UNKNOWN;
  }
}


}  // namespace opennav_coverage
