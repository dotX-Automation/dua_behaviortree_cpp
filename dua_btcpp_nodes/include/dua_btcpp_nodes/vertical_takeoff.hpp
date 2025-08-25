/**
 * Executes a vertical takeoff.
 *
 * dotX Automation s.r.l. <info@dotxautomation.com>
 *
 * August 25, 2025
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

#include <future>
#include <memory>
#include <stdexcept>
#include <string>

#include "visibility_control.h"

#include <dua_btcpp_base/entity_manager.hpp>

#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>

#include <dua_common_interfaces/msg/command_result_stamped.hpp>
#include <dua_aircraft_interfaces/action/takeoff.hpp>

using namespace dua_common_interfaces::msg;

using namespace dua_aircraft_interfaces::action;

namespace dua_btcpp_nodes
{

/**
 * Node that executes a vertical takeoff.
 */
class DUA_BTCPP_NODES_PUBLIC VerticalTakeoffNode : public BT::StatefulActionNode
{
public:
  /**
   * @brief Constructor.
   *
   * @param node_name Name of the node.
   * @param node_config Node configuration options.
   * @param ros2_node Pointer to the ROS 2 node.
   * @param clients_cache Pointer to the clients cache.
   * @param wait_server Wait for server to come up upon creation of the client.
   * @param spin Whether to spin the ROS 2 node when this node calls the action.
   * @throws std::runtime_error if the action name has not been correctly specified.
   */
  VerticalTakeoffNode(
    const std::string & node_name,
    const BT::NodeConfig & node_config,
    dua_node::NodeBase * ros2_node,
    const dua_btcpp_base::EntityManager::SharedPtr & clients_cache,
    bool wait_server = false,
    bool spin = false);

  /**
   * @brief Destructor.
   */
  ~VerticalTakeoffNode();

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

  /* Pointer to clients cache. */
  dua_btcpp_base::EntityManager::SharedPtr clients_cache_ = nullptr;

  /* Pointer to action client. */
  simple_actionclient::Client<Takeoff>::SharedPtr action_client_ = nullptr;

  /* Pointer to current goal. */
  rclcpp_action::ClientGoalHandle<Takeoff>::SharedPtr current_goal_ = nullptr;

  /* Current result future. */
  std::shared_future<rclcpp_action::ClientGoalHandle<Takeoff>::WrappedResult> current_res_future_;

  /* Wait for server to come up upon creation of the client. */
  bool wait_server_ = true;

  /* Spin the ROS 2 node when this node acts. */
  bool spin_ = false;
};

} // namespace dua_btcpp_nodes
