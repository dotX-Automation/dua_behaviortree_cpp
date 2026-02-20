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

#pragma once

#include "visibility_control.h"

#include <dua_node_cpp/dua_node.hpp>

#include <rclcpp/rclcpp.hpp>

#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>

namespace dua_btcpp_nodes
{

/**
 * Node that checks if a given time has passed since a specified start time.
 */
class DUA_BTCPP_NODES_PUBLIC CheckElapsedTime : public BT::StatefulActionNode
{
public:
  /**
   * @brief Constructor.
   *
   * @param node_name Name of the node.
   * @param node_config Node configuration options.
   * @param ros2_node Pointer to the ROS 2 node.
   */
  CheckElapsedTime(
    const std::string & node_name,
    const BT::NodeConfig & node_config,
    dua_node::NodeBase * ros2_node);

  /**
   * @brief Destructor.
   */
  ~CheckElapsedTime();

  /**
   * @brief Returns the list of ports used by this node.
   *
   * @return Ports list.
   */
  static BT::PortsList providedPorts();

  /**
   * @brief Runs at the start of the action.
   */
  BT::NodeStatus onStart() override;

  /**
   * @brief Runs during the action.
   */
  BT::NodeStatus onRunning() override;

  /**
   * @brief Executed when the action is stopped.
   */
  void onHalted() override;

private:
  /* Pointer to ROS 2 node. */
  dua_node::NodeBase * ros2_node_ = nullptr;

  /**
   * @brief Does the actual check.
   *
   * @param start_time Start time to compare against.
   * @param interval Time interval in milliseconds.
   * @return Whether the current time has elapsed since the start time.
   */
  bool check_elapsed_time(const rclcpp::Time & start_time, int interval);
};

} // namespace dua_btcpp_nodes
