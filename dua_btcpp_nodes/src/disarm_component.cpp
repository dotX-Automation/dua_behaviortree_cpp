/**
 * Disarms a component.
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

#include <dua_btcpp_nodes/disarm_component.hpp>

namespace dua_btcpp_nodes
{

DisarmComponent::DisarmComponent(
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
  // The following cases must be handled to create the action client
  // - We use now the action_name in the port: a static string
  // - We use later the action_name in the port: a blackboard entry
  // If neither is feasible, we throw an exception

  auto port_it = config().input_ports.find("action_name");
  if (port_it == config().input_ports.end()) {
    throw std::runtime_error("dua_btcpp_nodes::DisarmComponent::DisarmComponent: Action name not found in input ports.");
  }

  const std::string & bb_action_name = port_it->second;
  if (bb_action_name.empty()) {
    throw std::runtime_error("dua_btcpp_nodes::DisarmComponent::DisarmComponent: Action name is empty.");
  }

  if (!isBlackboardPointer(bb_action_name)) {
    // Action name has been hardcoded so we can use it right away
    action_client_ = clients_cache_->get_action_client<Disarm>(bb_action_name, wait_server);
  }
  // If we reach this point, it means we are using a blackboard pointer which will be set later on,
  // so creation of the client is deferred to the tick method call
}

DisarmComponent::~DisarmComponent()
{}

BT::PortsList DisarmComponent::providedPorts()
{
  return {
    BT::InputPort<std::string>("action_name", "Name of the ROS 2 Disarm action"),
    BT::InputPort<int>("timeout", 0, "Client operations timeout [ms] (0 means no timeout: wait indefinitely and poll instantaneously)")
  };
}

BT::NodeStatus DisarmComponent::tick()
{
  // First, create the client if it wasn't created already
  if (action_client_ == nullptr) {
    auto action_name = getInput<std::string>("action_name");
    action_client_ = clients_cache_->get_action_client<Disarm>(action_name.value(), wait_server_);
  }

  // Try to disarm the component
  int timeout_ms = getInput<int>("timeout").value();
  Disarm::Goal disarm_goal{};
  auto disarm_res = action_client_->call_sync(disarm_goal, spin_, false, timeout_ms, timeout_ms);
  bool success = std::get<0>(disarm_res) &&
    (std::get<1>(disarm_res) == rclcpp_action::ResultCode::SUCCEEDED ||
     std::get<1>(disarm_res) == rclcpp_action::ResultCode::UNKNOWN) &&
    (*(std::get<2>(disarm_res))).result.result == CommandResultStamped::SUCCESS;

  if (success) {
    return BT::NodeStatus::SUCCESS;
  } else {
    return BT::NodeStatus::FAILURE;
  }
}

} // namespace dua_btcpp_nodes
