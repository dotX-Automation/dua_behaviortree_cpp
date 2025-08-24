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

#include <dua_btcpp_nodes/dua_btcpp_nodes.hpp>

namespace dua_btcpp_nodes
{

void DUARegister::register_nodes(
  BT::BehaviorTreeFactory & factory,
  dua_node::NodeBase * node,
  std::shared_ptr<dua_btcpp_base::EntityManager> entity_manager,
  bool wait_servers,
  bool spin)
{
  // Action clients
  factory.registerNodeType<ArmComponent>("ArmComponent", node, entity_manager, wait_servers, spin);
  factory.registerNodeType<DisarmComponent>("DisarmComponent", node, entity_manager, wait_servers, spin);
  factory.registerNodeType<NavigateNode>("Navigate", node, entity_manager, wait_servers, spin);

  // Service clients
  factory.registerNodeType<TriggerComponent>("TriggerComponent", node, entity_manager, wait_servers, spin);
  factory.registerNodeType<SetComponentState>("SetComponentState", node, entity_manager, wait_servers, spin);

  // Topic subscribers
  factory.registerNodeType<SubscriberTrigger>("SubscriberTrigger", node);

  // Topic publishers
  factory.registerNodeType<PublishString>("PublishString", node, entity_manager);
}

} // namespace dua_btcpp_nodes

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(dua_btcpp_nodes::DUARegister, dua_btcpp_base::BaseRegister)
