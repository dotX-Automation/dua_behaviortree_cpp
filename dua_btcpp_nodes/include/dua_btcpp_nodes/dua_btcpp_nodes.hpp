/**
 * Library of BehaviorTree.CPP nodes for the DUA framework.
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

#include "visibility_control.h"

#include <dua_btcpp_base/base_register.hpp>

#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>

#include "arm_component.hpp"
#include "disarm_component.hpp"
#include "navigate.hpp"
#include "track.hpp"
#include "vertical_landing.hpp"
#include "vertical_safe_landing.hpp"
#include "vertical_takeoff.hpp"

#include "trigger_component.hpp"
#include "set_component_state.hpp"

#include "publish_string.hpp"

#include "subscriber_trigger.hpp"

#include "check_elapsed_time.hpp"
#include "check_time_frame.hpp"

#include "test_preset.hpp"

namespace dua_btcpp_nodes
{

/**
 * Registers BT.CPP nodes based on DUA standard interfaces and functionalities.
 */
class DUA_BTCPP_NODES_PUBLIC DUARegister : public dua_btcpp_base::BaseRegister
{
public:
  /**
   * @brief Registers all the nodes in this library to the given factory.
   *
   * @param factory BT.CPP factory to use.
   * @param node Pointer to the ROS 2 node to use.
   * @param entity_manager Pointer to the entity manager to use.
   * @param global_bb Pointer to the global blackboard to use.
   * @param wait_servers Enables waiting for service/action servers upon client creation.
   * @param spin Enables spinning the ROS 2 node when client nodes call services/actions.
   * @throws std::runtime_error if any node fails to register.
   * @throws std::bad_any_cast if clients instantiation is misconfigured.
   */
  void register_nodes(
    BT::BehaviorTreeFactory & factory,
    dua_node::NodeBase * node,
    std::shared_ptr<dua_btcpp_base::EntityManager> entity_manager,
    BT::Blackboard::Ptr global_bb,
    bool wait_servers,
    bool spin) override;
};

} // namespace dua_btcpp_nodes
