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

#pragma once

#include "visibility_control.h"

#include <dua_node_cpp/dua_node.hpp>

#include <rclcpp/rclcpp.hpp>

#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>

namespace dua_btcpp_nodes
{

/**
 * Memory pre-setter test node.
 */
class DUA_BTCPP_NODES_PUBLIC TestPreset : public BT::SyncActionNode
{
public:
  /**
   * @brief Constructor.
   *
   * @param node_name Name of the node.
   * @param node_config Node configuration options.
   * @param ros2_node Pointer to the ROS 2 node.
   * @param global_blackboard Pointer to the global blackboard.
   */
  TestPreset(
    const std::string & node_name,
    const BT::NodeConfig & node_config,
    dua_node::NodeBase * ros2_node,
    BT::Blackboard::Ptr global_blackboard);

  /**
   * @brief Destructor.
   */
  ~TestPreset();

  /**
   * @brief Returns the list of ports used by this node.
   *
   * @return Ports list.
   */
  static BT::PortsList providedPorts();

  /**
   * @brief Main tick function.
   */
  BT::NodeStatus tick() override;

private:
  /* Pointer to ROS 2 node. */
  dua_node::NodeBase * ros2_node_ = nullptr;

  /* Pointer to global blackboard. */
  BT::Blackboard::Ptr global_blackboard_ = nullptr;
};

} // namespace dua_btcpp_nodes
