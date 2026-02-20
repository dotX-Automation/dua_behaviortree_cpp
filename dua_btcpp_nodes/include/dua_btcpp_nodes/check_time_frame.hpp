/**
 * Checks if the current time is within a specified time frame.
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

#include <stdexcept>
#include <string>

#include "visibility_control.h"

#include <dua_node_cpp/dua_node.hpp>

#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>

namespace dua_btcpp_nodes
{

/**
 * Node that checks if a given time is within a specified time frame.
 */
class DUA_BTCPP_NODES_PUBLIC CheckTimeFrame : public BT::StatefulActionNode
{
public:
  /**
   * @brief Constructor.
   *
   * @param node_name Name of the node.
   * @param node_config Node configuration options.
   * @param ros2_node Pointer to the ROS 2 node.
   */
  CheckTimeFrame(
    const std::string & node_name,
    const BT::NodeConfig & node_config,
    dua_node::NodeBase * ros2_node);

  /**
   * @brief Destructor.
   */
  ~CheckTimeFrame();

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
   * @param start_time_str Start time string (format: HH:MM:SS, 24hr).
   * @param end_time_str End time string (format: HH:MM:SS, 24hr).
   * @return Whether the current time is within the specified time frame.
   *
   * @throws std::invalid_argument if time strings are in an invalid format.
   * @throws std::runtime_error if there is an error getting the current time.
   */
  bool check_time_frame(const std::string & start_time_str, const std::string & end_time_str);
};

} // namespace dua_btcpp_nodes
