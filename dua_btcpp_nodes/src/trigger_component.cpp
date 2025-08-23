/**
 * Triggers a component.
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

#include <dua_btcpp_nodes/trigger_component.hpp>

namespace dua_btcpp_nodes
{

TriggerComponent::TriggerComponent(
  const std::string & node_name,
  const BT::NodeConfig & node_config,
  dua_node::NodeBase * ros2_node,
  const dua_btcpp_base::EntityManager::SharedPtr & clients_cache,
  bool wait_server,
  bool spin)
: BT::SyncActionNode(node_name, node_config),
  ros2_node_(ros2_node),
  clients_cache_(clients_cache),
  wait_server_(wait_server),
  spin_(spin)
{
  // The following cases must be handled to create the service client
  // - We use now the service_name in the port: a static string
  // - We use later the service_name in the port: a blackboard entry
  // If neither is feasible, we throw an exception

  auto port_it = config().input_ports.find("service_name");
  if (port_it == config().input_ports.end()) {
    throw std::runtime_error("dua_btcpp_nodes::TriggerComponent::TriggerComponent: Service name not found in input ports.");
  }

  const std::string & bb_service_name = port_it->second;
  if (bb_service_name.empty()) {
    throw std::runtime_error("dua_btcpp_nodes::TriggerComponent::TriggerComponent: Service name is empty.");
  }

  if (!isBlackboardPointer(bb_service_name)) {
    // Service name has been hardcoded so we can use it right away
    service_client_ = clients_cache_->get_service_client<Trigger>(bb_service_name, wait_server);
  }
  // If we reach this point, it means we are using a blackboard pointer which will be set later on,
  // so creation of the client is deferred to the tick method call
}

TriggerComponent::~TriggerComponent()
{}

BT::PortsList TriggerComponent::providedPorts()
{
  return {
    BT::InputPort<std::string>("service_name", "Name of the ROS 2 Trigger service.")
  };
}

BT::NodeStatus TriggerComponent::tick()
{
  // First, create the client if it wasn't created already
  if (service_client_ == nullptr) {
    auto service_name = getInput<std::string>("service_name");
    service_client_ = clients_cache_->get_service_client<Trigger>(service_name.value(), wait_server_);
  }

  // Try to trigger the component
  Trigger::Request::SharedPtr req = std::make_shared<Trigger::Request>();
  Trigger::Response::SharedPtr res = service_client_->call_sync(req, spin_);
  bool success = res != nullptr && res->success;

  if (success) {
    return BT::NodeStatus::SUCCESS;
  } else {
    return BT::NodeStatus::FAILURE;
  }
}

} // namespace dua_btcpp_nodes
