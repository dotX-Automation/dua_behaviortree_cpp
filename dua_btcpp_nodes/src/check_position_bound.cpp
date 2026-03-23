/**
 * Checks if a given position is inside a given zone.
 *
 * dotX Automation s.r.l. <info@dotxautomation.com>
 *
 * March 23, 2026
 */

/**
 * Copyright 2026 dotX Automation s.r.l.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <dua_btcpp_nodes/check_position_bound.hpp>

namespace dua_btcpp_nodes
{

CheckPositionBound::CheckPositionBound(
  const std::string & node_name,
  const BT::NodeConfig & node_config,
  dua_node::NodeBase * ros2_node)
: BT::SyncActionNode(node_name, node_config),
  ros2_node_(ros2_node)
{}

CheckPositionBound::~CheckPositionBound()
{}

BT::PortsList CheckPositionBound::providedPorts()
{
  return {
    BT::InputPort<PointStamped>("point", PointStamped{}, "Position to check"),
    BT::InputPort<PolygonStamped>("zone", PolygonStamped{}, "Zone to check against")
  };
}

BT::NodeStatus CheckPositionBound::tick()
{
  // Get input values
  PointStamped point = getInput<PointStamped>("point").value_or(PointStamped{});
  PolygonStamped zone = getInput<PolygonStamped>("zone").value_or(PolygonStamped{});

  // Check if the frame IDs coincide
  std::string point_frame_id = point.header.frame_id;
  std::string zone_frame_id = zone.header.frame_id;
  if (point_frame_id != zone_frame_id) {
    std::string err_msg = "CheckPositionBound::tick: Invalid frame IDs: point (" + point_frame_id + "), zone (" + zone_frame_id + ")";
    throw std::runtime_error(err_msg);
  }

  // Check if the polygon is big enough
  if (zone.polygon.points.size() < 3) {
    std::string err_msg = "CheckPositionBound::tick: Polygon is too small";
    throw std::runtime_error(err_msg);
  }

  // Check if the point is inside the zone using ray casting
  const size_t zone_size = zone.polygon.points.size();
  bool inside = false;
  const Eigen::Vector2d p(point.point.x, point.point.y);
  for (size_t i = 0, j = zone_size - 1; i < zone_size; j = i++) {
    const Eigen::Vector2d a(
      static_cast<double>(zone.polygon.points[i].x),
      static_cast<double>(zone.polygon.points[i].y));
    const Eigen::Vector2d b(
      static_cast<double>(zone.polygon.points[j].x),
      static_cast<double>(zone.polygon.points[j].y));
    bool crosses = ((a.y() > p.y()) != (b.y() > p.y())) &&
      (p.x() < (b.x() - a.x()) * (p.y() - a.y()) / (b.y() - a.y()) + a.x());
    if (crosses) {inside = !inside;}
  }

  return inside ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

} // namespace dua_btcpp_nodes
