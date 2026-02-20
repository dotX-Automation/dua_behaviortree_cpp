/**
 * Checks if a given time has elapsed since a specified start time.
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

#include <dua_btcpp_nodes/check_elapsed_time.hpp>

namespace dua_btcpp_nodes
{

CheckElapsedTime::CheckElapsedTime(
  const std::string & node_name,
  const BT::NodeConfig & node_config,
  dua_node::NodeBase * ros2_node)
: BT::StatefulActionNode(node_name, node_config),
  ros2_node_(ros2_node)
{}

CheckElapsedTime::~CheckElapsedTime()
{}

BT::PortsList CheckElapsedTime::providedPorts()
{
  return {
    BT::InputPort<int>("interval", 0, "Time interval (milliseconds)"),
    BT::InputPort<rclcpp::Time>("time_point", rclcpp::Time(0, 0, RCL_ROS_TIME), "Start time to compare against (RCL_ROS_TIME)"),
    BT::InputPort<bool>("wait", false, "Return RUNNING instead of FAILURE if the check fails.")
  };
}

BT::NodeStatus CheckElapsedTime::onStart()
{
  int interval = getInput<int>("interval").value();
  rclcpp::Time time_point = getInput<rclcpp::Time>("time_point").value();
  bool wait = getInput<bool>("wait").value();

  if (check_elapsed_time(time_point, interval)) {
    return BT::NodeStatus::SUCCESS;
  } else {
    return wait ? BT::NodeStatus::RUNNING : BT::NodeStatus::FAILURE;
  }
}

BT::NodeStatus CheckElapsedTime::onRunning()
{
  // The code is copied from onStart() because we have to do the exact same thing.
  // It is left explicitly like this, and not wrapped in a separate method,
  // to be explanatory and to allow easy future modifications should we decide to change
  // this behavior.

  int interval = getInput<int>("interval").value();
  rclcpp::Time time_point = getInput<rclcpp::Time>("time_point").value();
  bool wait = getInput<bool>("wait").value();

  if (check_elapsed_time(time_point, interval)) {
    return BT::NodeStatus::SUCCESS;
  } else {
    return wait ? BT::NodeStatus::RUNNING : BT::NodeStatus::FAILURE;
  }
}

void CheckElapsedTime::onHalted()
{}

bool CheckElapsedTime::check_elapsed_time(const rclcpp::Time & start_time, int interval)
{
  auto now = ros2_node_->get_clock()->now();
  auto interval_ros = rclcpp::Duration::from_nanoseconds(interval * 1e6);
  auto elapsed = now - start_time;
  return elapsed >= interval_ros;
}

} // namespace dua_btcpp_nodes
