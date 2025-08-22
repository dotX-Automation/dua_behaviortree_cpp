/**
 * Publishes a string on a given topic.
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

#include "entity_manager.hpp"

#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>

#include <std_msgs/msg/string.hpp>

using namespace std_msgs::msg;

namespace dua_btcpp
{

/**
 * Node that publishes a string on a given topic.
 */
class DUA_BTCPP_PUBLIC PublishString : public BT::SyncActionNode
{
public:
  /**
   * @brief Constructor.
   *
   * @param node_name Name of the node.
   * @param node_config Node configuration options.
   * @param ros2_node Pointer to the ROS 2 node.
   * @param publishers_cache Pointer to the publishers cache.
   * @throws std::runtime_error if the topic name has not been correctly specified.
   */
  PublishString(
    const std::string & node_name,
    const BT::NodeConfig & node_config,
    const dua_node::NodeBase::SharedPtr & ros2_node,
    const EntityManager::SharedPtr & publishers_cache);

  /**
   * @brief Destructor.
   */
  ~PublishString();

  /**
   * @brief Returns the list of ports used by this node.
   *
   * @return Ports list.
   */
  static BT::PortsList providedPorts();

  /**
   * @brief Performs the node operation.
   */
  BT::NodeStatus tick() override;

private:
  /* Pointer to ROS 2 node. */
  dua_node::NodeBase::SharedPtr ros2_node_ = nullptr;

  /* Pointer to publishers cache. */
  EntityManager::SharedPtr publishers_cache_ = nullptr;

  /* Pointer to topic publisher. */
  rclcpp::Publisher<String>::SharedPtr publisher_ = nullptr;
};

} // namespace dua_btcpp
