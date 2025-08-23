/**
 * Base plugin class to load custom BT.CPP nodes with the btcpp_executor.
 *
 * dotX Automation s.r.l. <info@dotxautomation.com>
 *
 * August 23, 2025
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

#include "entity_manager.hpp"

#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>

namespace dua_btcpp_base
{

/**
 * Registers custom BT.CPP nodes.
 */
class BaseRegister
{
public:
  /**
   * @brief Destructor.
   */
  virtual ~BaseRegister()
  {}

  /**
   * @brief Registers custom nodes with the BehaviorTreeFactory (to be overridden).
   *
   * @param factory Pointer to the BT.CPP factory to use.
   * @param node Pointer to the ROS 2 node to use.
   * @param entity_manager Pointer to the entity manager to use.
   * @param wait_servers Enables waiting for service/action servers upon client creation.
   * @param spin Enables spinning the ROS 2 node when client nodes call services/actions.
   * @throws std::runtime_error if any node fails to register.
   * @throws std::bad_any_cast if clients instantiation is misconfigured.
   */
  virtual void register_nodes(
    BT::BehaviorTreeFactory & factory,
    std::shared_ptr<dua_node::NodeBase> node,
    std::shared_ptr<dua_btcpp_base::EntityManager> entity_manager,
    bool wait_servers,
    bool spin) = 0;

protected:
  /**
   * @brief Constructor, required by pluginlib to be protected and noop.
   */
  BaseRegister()
  {}
};

} // namespace dua_btcpp_base
