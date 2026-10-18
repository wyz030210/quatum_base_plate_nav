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

#ifndef OPENNAV_COVERAGE__UTILS_HPP_
#define OPENNAV_COVERAGE__UTILS_HPP_

#include <vector>
#include <string>
#include <algorithm>
#include <memory>
#include <utility>
#include <cmath>
#include <stdexcept>

#include "fields2cover.h" // NOLINT
#include "rclcpp/rclcpp.hpp"
#include "opennav_coverage/types.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "geometry_msgs/msg/point32.hpp"

namespace opennav_coverage
{

namespace util
{

/**
 * @brief Converts F2C Point to ROS Point32
 * @param Point point
 * @return geometry_msgs Point
 */
inline geometry_msgs::msg::Point pointToPoint32(const geometry_msgs::msg::Point32 & pt)
{
  geometry_msgs::msg::Point msg;
  msg.x = pt.x;
  msg.y = pt.y;
  msg.z = pt.z;
  return msg;
}

/**
 * @brief Converts F2C Point to ROS Point32
 * @param Point point
 * @return geometry_msgs Point
 */
inline geometry_msgs::msg::Point32 toMsg(const Point & pt)
{
  geometry_msgs::msg::Point32 msg;
  msg.x = pt.getX();
  msg.y = pt.getY();
  msg.z = pt.getZ();
  return msg;
}

/*
 * @brief Converts F2C Point to ROS Pose Stamped
 * @param Point point
 * @return geometry_msgs Point
 */
inline geometry_msgs::msg::PoseStamped toMsg(
  const PathState & state, const std_msgs::msg::Header & header = std_msgs::msg::Header())
{
  geometry_msgs::msg::PoseStamped msg;
  msg.header = header;
  msg.pose.position.x = state.point.getX();
  msg.pose.position.y = state.point.getY();
  msg.pose.position.z = state.point.getZ();
  msg.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(state.angle);
  return msg;
}

/**
 * @brief Converts swaths to coverage path message for action client
 * @param swaths Swaths to convert. May be ordered or unordered.
 * @param Field Field to use for conversion from UTM if necessary
 * @param header header
 * @param bool if the origional CRS is cartesian or not requiring conversion
 * @return PathComponents Info for action server to utilize
 */
inline opennav_coverage_msgs::msg::PathComponents toCoveragePathMsg(
  const Swaths & raw_swaths, const F2CField & field,
  const bool ordered, const std_msgs::msg::Header & header, const bool is_cartesian)
{
  opennav_coverage_msgs::msg::PathComponents msg;
  msg.contains_turns = false;
  msg.swaths_ordered = ordered;
  msg.header = header;
  msg.swaths.resize(raw_swaths.size());

  Swaths swaths = raw_swaths;
  if (!is_cartesian) {
    swaths = f2c::Transform::transformToPrevCRS(raw_swaths, field);
  } else {
    swaths.moveTo(field.getRefPoint());
  }

  for (unsigned int i = 0; i != swaths.size(); i++) {
    msg.swaths[i].start = toMsg(swaths[i].startPoint());
    msg.swaths[i].end = toMsg(swaths[i].endPoint());
  }

  return msg;
}

/**
 * @brief Converts an ordered route to coverage path message (ordered swaths only)
 * @param route Route whose swath groups to convert
 * @param Field Field to use for conversion from UTM if necessary
 * @param header header
 * @param bool if the origional CRS is cartesian or not requiring conversion
 * @return PathComponents Info for action server to utilize
 */
inline opennav_coverage_msgs::msg::PathComponents toCoveragePathMsg(
  const F2CRoute & route, const F2CField & field,
  const std_msgs::msg::Header & header, const bool is_cartesian)
{
  Swaths ordered;
  for (const auto & group : route.getVectorSwaths()) {
    for (const auto & s : group) {
      ordered.emplace_back(s);
    }
  }
  return toCoveragePathMsg(ordered, field, true, header, is_cartesian);
}

/**
 * @brief Converts full path to coverage path message for action client
 * @param path Full path to convert
 * @param Field Field to use for conversion from UTM if necessary
 * @param header header
 * @param bool if the origional CRS is cartesian or not requiring conversion
 * @return PathComponents Info for action server to utilize
 */
inline opennav_coverage_msgs::msg::PathComponents toCoveragePathMsg(
  const Path & raw_path, const F2CField & field,
  const std_msgs::msg::Header & header, const bool is_cartesian)
{
  using f2c::types::PathSectionType;
  opennav_coverage_msgs::msg::PathComponents msg;
  msg.contains_turns = true;
  msg.swaths_ordered = true;
  msg.header = header;

  if (raw_path.size() == 0) {
    return msg;
  }

  Point curr_swath_start(0.0, 0.0);
  nav_msgs::msg::Path * curr_turn = nullptr;

  Path path = raw_path;
  if (!is_cartesian) {
    path = f2c::Transform::transformToPrevCRS(raw_path, field);
  } else {
    path.moveTo(field.getRefPoint());
  }

  // Decomposition produces HL_SWATH states for inter-cell headland passes;
  // treat them like regular SWATH states.
  auto isSwathLike = [](PathSectionType t) {
      return t == PathSectionType::SWATH || t == PathSectionType::HL_SWATH;
    };

  PathSectionType curr_state = path[0].type;
  if (isSwathLike(curr_state)) {
    curr_swath_start = path[0].point;
  } else if (curr_state == PathSectionType::TURN) {
    msg.turns.push_back(nav_msgs::msg::Path());
    msg.turns.back().header = header;
    curr_turn = &msg.turns.back();
  }

  for (unsigned int i = 0; i != path.size(); i++) {
    const bool prev_swath = isSwathLike(curr_state);
    const bool prev_turn = curr_state == PathSectionType::TURN;
    const bool now_swath = isSwathLike(path[i].type);
    const bool now_turn = path[i].type == PathSectionType::TURN;

    if (prev_swath && now_swath) {
      // Continuing swath (SWATH or HL_SWATH) so...
      // (1) no action required.
    } else if (prev_turn && now_turn) {
      // Continuing a turn so...
      // (1) keep populating
      curr_turn->poses.push_back(toMsg(path[i], header));
    } else if (prev_swath && now_turn) {
      // Transitioning from a swath to a turn so...
      // (1) Complete the existing swath
      opennav_coverage_msgs::msg::Swath swath;
      swath.start = toMsg(curr_swath_start);
      swath.end = toMsg(path[i - 1].point);
      msg.swaths.push_back(swath);
      // (2) Start a new turn path
      msg.turns.push_back(nav_msgs::msg::Path());
      msg.turns.back().header = header;
      curr_turn = &msg.turns.back();
      curr_turn->poses.push_back(toMsg(path[i], header));
    } else if (prev_turn && now_swath) {
      // Transitioning from a turn to a swath so...
      // (1) Update new swath starting point
      curr_swath_start = path[i].point;
    }

    curr_state = path[i].type;

    if (!isSwathLike(path[i].type) &&
      path[i].type != PathSectionType::TURN)
    {
      throw std::runtime_error("Unknown type of path state detected, cannot obtain path!");
    }
  }

  if (isSwathLike(curr_state)) {
    opennav_coverage_msgs::msg::Swath swath;
    swath.start = toMsg(curr_swath_start);
    swath.end = toMsg(path.back().point);
    msg.swaths.push_back(swath);
  }

  return msg;
}

/**
 * @brief Converts full path to nav_msgs/path message for action client, visualization
 * and use in direct-sending to a controller to replace the planner server. Interpolates
 * the F2C path to make it dense for following semantics.
 * @param path Full path to convert
 * @param Field Field to use for conversion from UTM if necessary
 * @param header header
 * @param bool if the origional CRS is cartesian or not requiring conversion
 * @param out_velocities Optional: if non-null, filled with per-pose velocity (m/s), parallel to poses
 * @param out_is_backward Optional: if non-null, filled with per-pose reverse-direction flags
 * @return nav_msgs/Path Path
 */
inline nav_msgs::msg::Path toNavPathMsg(
  const Path & raw_path, const F2CField & field,
  const std_msgs::msg::Header & header, const bool is_cartesian,
  const float & pt_dist,
  std::vector<double> * out_velocities = nullptr,
  std::vector<bool> * out_is_backward = nullptr)
{
  nav_msgs::msg::Path msg;
  msg.header = header;
  if (out_velocities) {out_velocities->clear();}
  if (out_is_backward) {out_is_backward->clear();}

  if (raw_path.size() == 0) {
    return msg;
  }

  Path path = raw_path;
  if (!is_cartesian) {
    path = f2c::Transform::transformToPrevCRS(raw_path, field);
  } else {
    path.moveTo(field.getRefPoint());
  }

  // discretizeSwath splits only SWATH states at pt_dist intervals, leaving dense turns intact.
  path = path.discretizeSwath(static_cast<double>(pt_dist));

  // Reserve up front so the population loop below doesn't reallocate.
  const auto n = path.size();
  msg.poses.reserve(n);
  if (out_velocities) {out_velocities->reserve(n);}
  if (out_is_backward) {out_is_backward->reserve(n);}

  for (const auto & state : path) {
    msg.poses.push_back(toMsg(state, header));
    if (out_velocities) {out_velocities->push_back(state.velocity);}
    if (out_is_backward) {
      out_is_backward->push_back(state.dir == f2c::types::PathDirection::BACKWARD);
    }
  }

  return msg;
}

/**
 * @brief Converts full path to nav_msgs/path message in cartesian UTM frame
 * @param path Full path to convert
 * @param header header
 * @return nav_msgs/Path Path
 */
inline nav_msgs::msg::Path toCartesianNavPathMsg(
  const Path & raw_path,
  const std_msgs::msg::Header & header, const float & pt_dist)
{
  return toNavPathMsg(raw_path, F2CField(), header, true, pt_dist);
}

/**
 * @brief Converts action goal coordinates into a Field type
 * Note that this may be in GPS or cartesian coordinates!
 * @param goal Goal to pase
 * @return Field field of goal polygons
 */
inline F2CField getFieldFromGoal(
  typename std::shared_ptr<const typename ComputeCoveragePath::Goal> goal)
{
  auto polygons = goal->polygons;
  if (polygons.size() == 0) {
    throw std::invalid_argument("No field set in action goal!");
  }
  for (const auto & polygon : polygons) {
    if (polygon.coordinates.size() < 4 ||
      polygon.coordinates.front() != polygon.coordinates.back())
    {
      throw std::invalid_argument("Polygon needs at least 3 vertices and a closed ring.");
    }
    for (const auto & point : polygon.coordinates) {
      if (!std::isfinite(point.axis1) || !std::isfinite(point.axis2)) {
        throw std::invalid_argument("Polygon coordinates must be finite.");
      }
    }
  }

  // Get the outer most polygon (usually the only one if no voids)
  Polygon outer_polygon;
  for (unsigned int i = 0; i != polygons[0].coordinates.size(); i++) {
    outer_polygon.addPoint(
      Point(polygons[0].coordinates[i].axis1, polygons[0].coordinates[i].axis2));
  }
  Field field(outer_polygon);

  // Now, parse pontential inner rings (e.g. voids)
  for (unsigned int i = 1; i != polygons.size(); i++) {
    auto polygon = polygons[i];
    if (polygon.coordinates.front() != polygon.coordinates.back()) {
      throw std::invalid_argument("Inner polygon malformed, first element must equal last!");
    }

    Polygon inner_polygon;
    for (unsigned int idx = 0; idx != polygon.coordinates.size(); idx++) {
      inner_polygon.addPoint(Point(polygon.coordinates[idx].axis1, polygon.coordinates[idx].axis2));
    }

    field.addRing(inner_polygon);
  }

  if (!field.get()->IsValid() || field.area() <= 1e-8) {
    throw std::invalid_argument("Polygon has invalid rings, self-intersections or zero area.");
  }
  return F2CField(Fields(field));
}

// Check the whole vehicle, including swept space between successive path states.
inline void validatePathBounds(
  const Path & raw_path, const Field & field,
  double robot_length, double robot_width, double margin)
{
  if (!std::isfinite(robot_length) || robot_length <= 0.0 ||
    !std::isfinite(robot_width) || robot_width <= 0.0 ||
    !std::isfinite(margin) || margin < 0.0 || raw_path.size() == 0)
  {
    throw std::invalid_argument("Invalid vehicle geometry or empty coverage path.");
  }
  auto path = raw_path.discretizeSwath(0.025);
  auto endpoint = path[path.size() - 1];
  endpoint.point = endpoint.atEnd();
  endpoint.len = 0.0;
  path.addState(endpoint);
  f2c::types::MultiPoint previous;
  double previous_angle = 0.0;
  Point previous_point;
  const double radius = 0.5 * std::hypot(robot_length, robot_width);
  for (size_t i = 0; i < path.size(); ++i) {
    const auto & state = path[i];
    if (!std::isfinite(state.point.getX()) || !std::isfinite(state.point.getY()) ||
      !std::isfinite(state.angle))
    {
      throw std::invalid_argument("Coverage path contains non-finite poses.");
    }
    if (i > 0 && state.point.distance(previous_point) > 0.25) {
      throw std::invalid_argument("Coverage path has a discontinuous connection.");
    }
    f2c::types::MultiPoint corners;
    for (const auto & offset : std::vector<std::pair<double, double>>{
        {-robot_length / 2.0, -robot_width / 2.0},
        {-robot_length / 2.0, robot_width / 2.0},
        {robot_length / 2.0, robot_width / 2.0},
        {robot_length / 2.0, -robot_width / 2.0}})
    {
      corners.addGeometry(Point(
          state.point.getX() + std::cos(state.angle) * offset.first -
          std::sin(state.angle) * offset.second,
          state.point.getY() + std::sin(state.angle) * offset.first +
          std::cos(state.angle) * offset.second));
    }
    auto swept = corners.clone();
    if (i > 0) {
      for (size_t j = 0; j < previous.size(); ++j) {
        swept.addGeometry(previous.getGeometry(j));
      }
    }
    const double delta = i > 0 ?
      std::abs(std::remainder(state.angle - previous_angle, 2.0 * M_PI)) : 0.0;
    // Rotation between samples is conservatively enclosed, not just the corners.
    const double padding = margin + radius * delta / 2.0;
    const auto hull = Field::convexHull(swept);
    const auto envelope = padding > 0.0 ? Fields::buffer(hull, padding) : Fields(hull);
    if (envelope.difference(field).area() > 1e-8) {
      throw std::invalid_argument(
              "Vehicle footprint leaves the coverage region near path state " +
              std::to_string(i) + "; enlarge turn space or split the region.");
    }
    previous = corners;
    previous_point = state.point;
    previous_angle = state.angle;
  }
}

/**
 * @brief Converts a goal-frame point into the frame the field's swaths/cells live in
 * @param pt Point in the goal frame (cartesian or GPS)
 * @param field Field already transformed to its working frame
 * @param is_cartesian Whether the goal coordinates are cartesian
 * @return Point in the field's local frame
 */
inline F2CPoint toFieldFrame(const F2CPoint & pt, const F2CField & field, bool is_cartesian)
{
  // F2CField always stores geometry with its ref point subtracted, so the start point needs
  // the same ref point removed (and, for GPS, projecting to UTM first).
  F2CPoint abs = is_cartesian ?
    pt : f2c::Transform::transform(pt, field.getPrevCRS(), field.getCRS());
  return abs - field.getRefPoint();
}

/**
 * @brief Converts a string to uppercase
 * @param string String to change to uppercase
 */
inline void toUpper(std::string & string)
{
  std::transform(string.begin(), string.end(), string.begin(), ::toupper);
}

/**
 * @brief A Path Components iterator object to get next turn/swath from action return
 *
 * Example Use:
    for (opennav_coverage::util::PathComponentsIterator it(msg); it.isValid(); it.advance()) {
      auto curr_row_info = it.getNext();

      // Always should be valid -- Swath
      (void)std::get<0>(curr_row_info)->start;

      if (std::get<1>(curr_row_info)) {
        // Always should be before last -- Turn
        (void)std::get<1>(curr_row_info)->poses;
      }
    }

    auto last_row_info = it.getNext();
    ASSERT(std::get<1>(last_row_info) == nullptr);
 */
class PathComponentsIterator
{
public:
  /**
   * @brief A Path Components iterator constructor
   * @param PathComponents object to iterate over
   */
  explicit PathComponentsIterator(opennav_coverage_msgs::msg::PathComponents & msg)
  : path_components_(msg), idx_(0)
  {
    if (fabs(path_components_.swaths.size() - path_components_.turns.size()) > 1) {
      throw std::runtime_error("PathComponents size not valid for iteration!");
    } else if (!path_components_.contains_turns) {
      throw std::runtime_error("PathComponents cannot be iterated over without turns!");
    } else if (!path_components_.swaths_ordered) {
      throw std::runtime_error("PathComponents cannot be iterated over without ordered swaths!");
    }

    max_idx_ = path_components_.swaths.size();
  }

  /**
   * @brief For condition if still valid to continue iterating
   */
  bool isValid()
  {
    return idx_ <= max_idx_;
  }

  /**
   * @brief For loop marching
   */
  void advance()
  {
    idx_++;
  }

  /**
   * @brief Get the data of the current iteration
   * @return returns a pair of pointers to the current swath and its next turn
   * If at the end, the turn is nullptr, so be sure to check it!
   */
  std::pair<opennav_coverage_msgs::msg::Swath *, nav_msgs::msg::Path *>
  getNext()
  {
    if (idx_ < max_idx_ - 1) {
      return std::make_pair(&path_components_.swaths[idx_], &path_components_.turns[idx_]);
    } else {
      return std::make_pair(&path_components_.swaths[idx_], nullptr);
    }
  }

  opennav_coverage_msgs::msg::PathComponents & path_components_;
  unsigned int idx_;
  unsigned int max_idx_;
};

}  // namespace util

}  // namespace opennav_coverage

#endif  // OPENNAV_COVERAGE__UTILS_HPP_
