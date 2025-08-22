/**
 * Library of BehaviorTree.CPP nodes for the DUA framework.
 *
 * dotX Automation s.r.l. <info@dotxautomation.com>
 *
 * August 21, 2025
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

#include "visibility_control.h"

#include "entity_manager.hpp"

#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>

#include "arm_component.hpp"
#include "disarm_component.hpp"

#include "trigger_component.hpp"
#include "set_component_state.hpp"

#include "publish_string.hpp"

namespace dua_btcpp
{

/**
 * @brief Registers all the nodes in this library to the given factory.
 *
 * @param factory Pointer to the BT.CPP factory to use.
 * @param node Pointer to the ROS 2 node to use.
 * @param entity_manager Pointer to the entity manager to use.
 * @param wait_servers Enables waiting for service/action servers upon client creation.
 * @param spin Enables spinning the ROS 2 node when client nodes call services/actions.
 */
void DUA_BTCPP_PUBLIC register_nodes(
  std::shared_ptr<BT::BehaviorTreeFactory> factory,
  std::shared_ptr<dua_node::NodeBase> node,
  std::shared_ptr<EntityManager> entity_manager,
  bool wait_servers,
  bool spin)
{

  // Action clients
  factory->registerNodeType<ArmComponent>("ArmComponent", node, entity_manager, wait_servers, spin);
  factory->registerNodeType<DisarmComponent>("DisarmComponent", node, entity_manager, wait_servers, spin);

  // Service clients
  factory->registerNodeType<TriggerComponent>("TriggerComponent", node, entity_manager, wait_servers, spin);
  factory->registerNodeType<SetComponentState>("SetComponentState", node, entity_manager, wait_servers, spin);

  // Topic subscribers
  // TODO

  // Topic publishers
  factory->registerNodeType<PublishString>("PublishString", node, entity_manager);

}

} // namespace dua_btcpp
