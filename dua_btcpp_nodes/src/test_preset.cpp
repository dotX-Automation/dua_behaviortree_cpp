/**
 * Memory pre-setter test node.
 *
 * dotX Automation s.r.l. <info@dotxautomation.com>
 *
 * February 20, 2026
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

#include <dua_btcpp_nodes/test_preset.hpp>

namespace dua_btcpp_nodes
{

TestPreset::TestPreset(
  const std::string & node_name,
    const BT::NodeConfig & node_config,
    dua_node::NodeBase * ros2_node,
    BT::Blackboard::Ptr global_blackboard)
: BT::SyncActionNode(node_name, node_config),
  ros2_node_(ros2_node),
  global_blackboard_(global_blackboard)
{}

TestPreset::~TestPreset()
{}

BT::PortsList TestPreset::providedPorts()
{
  return {};
}

BT::NodeStatus TestPreset::tick()
{
  // CheckElapsedTime test: store current time
  rclcpp::Time now = ros2_node_->get_clock()->now();
  global_blackboard_->set<rclcpp::Time>("check_elapsed_time_time_point", now);

  // Explore test: store a sample exploration zone
  geometry_msgs::msg::Polygon zone;
  for (int i = 0; i < 4; ++i) {
    geometry_msgs::msg::Point32 point;
    point.x = i * 1.0;
    point.y = i * 2.0;
    point.z = 0.0;
    zone.points.push_back(point);
  }
  global_blackboard_->set<geometry_msgs::msg::Polygon>("explore_zone", zone);

  return BT::NodeStatus::SUCCESS;
}

} // namespace dua_btcpp_nodes
