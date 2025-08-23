/**
 * Waits for a trigger message.
 *
 * dotX Automation s.r.l. <info@dotxautomation.com>
 *
 * August 22, 2025
 */

/**
 * Copyright 2025 dotX Automation s.r.l.
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

#include <memory>
#include <stdexcept>

#include "visibility_control.h"

#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>

#include <dua_node_cpp/dua_node.hpp>

#include <std_msgs/msg/empty.hpp>

using namespace std_msgs::msg;

namespace dua_btcpp_nodes
{

/**
 * Node that waits for a trigger message.
 */
class DUA_BTCPP_NODES_PUBLIC SubscriberTrigger : public BT::StatefulActionNode
{
public:
  /**
   * @brief Constructor.
   *
   * @param node_name Name of the node.
   * @param node_config Node configuration options.
   * @param ros2_node Pointer to the ROS 2 node.
   * @throws std::runtime_error if the topic name has not been correctly specified.
   */
  SubscriberTrigger(
    const std::string & node_name,
    const BT::NodeConfig & node_config,
    const dua_node::NodeBase::SharedPtr & ros2_node);

  /**
   * @brief Destructor.
   */
  ~SubscriberTrigger();

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
  dua_node::NodeBase::SharedPtr ros2_node_ = nullptr;

  /* Pointer to topic subscriber. */
  rclcpp::Subscription<Empty>::SharedPtr subscriber_ = nullptr;

  /* Message received flag. */
  bool trigger_ = false;

  /* ROS 2 topic callback. */
  void trigger_clbk(const Empty::SharedPtr msg);

  /* ROS 2 topic callback group. */
  rclcpp::CallbackGroup::SharedPtr cgroup_ = nullptr;
};

} // namespace dua_btcpp_nodes
