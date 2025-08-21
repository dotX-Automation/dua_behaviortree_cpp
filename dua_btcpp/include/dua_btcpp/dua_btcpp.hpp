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

#include "client_manager.hpp"

#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>

#include "arm_component.hpp"
#include "disarm_component.hpp"

namespace dua_btcpp
{

/**
 * @brief Registers all the nodes in this library to the given factory.
 *
 * @param factory Pointer to the BT.CPP factory to use.
 * @param node Pointer to the ROS 2 node to use.
 * @param client_manager Pointer to the client manager to use.
 */
void DUA_BTCPP_PUBLIC register_nodes(
  std::shared_ptr<BT::BehaviorTreeFactory> factory,
  std::shared_ptr<dua_node::NodeBase> node,
  std::shared_ptr<ClientManager> client_manager)
{

// Action clients
factory->registerNodeType<ArmComponent>("ArmComponent", node, client_manager);
factory->registerNodeType<DisarmComponent>("DisarmComponent", node, client_manager);

// Service clients
// TODO

// Topic subscribers
// TODO

// Topic publishers
// TODO

}

} // namespace dua_btcpp
