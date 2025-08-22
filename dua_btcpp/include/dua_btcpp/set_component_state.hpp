/**
 * Sets the (binary) state of a component.
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

#include "client_manager.hpp"

#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>

#include <std_srvs/srv/set_bool.hpp>

using namespace std_srvs::srv;

namespace dua_btcpp
{

/**
 * Node that sets the binary state of a component.
 */
class DUA_BTCPP_PUBLIC SetComponentState : public BT::SyncActionNode
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
   * @param spin Whether to spin the ROS 2 node when this node calls the service.
   * @throws std::runtime_error if the service name has not been correctly specified.
   */
  SetComponentState(
    const std::string & node_name,
    const BT::NodeConfig & node_config,
    const dua_node::NodeBase::SharedPtr & ros2_node,
    const ClientManager::SharedPtr & clients_cache,
    bool wait_server = false,
    bool spin = false);

  /**
   * @brief Destructor.
   */
  ~SetComponentState();

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

  /* Pointer to clients cache. */
  ClientManager::SharedPtr clients_cache_ = nullptr;

  /* Pointer to service client. */
  simple_serviceclient::Client<SetBool>::SharedPtr service_client_ = nullptr;

  /* Wait for server to come up upon creation of the client. */
  bool wait_server_ = true;

  /* Spin the ROS 2 node when this node acts. */
  bool spin_ = false;
};

} // namespace dua_btcpp
