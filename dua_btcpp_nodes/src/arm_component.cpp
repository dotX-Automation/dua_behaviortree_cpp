/**
 * Arms a component.
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

#include <dua_btcpp_nodes/arm_component.hpp>

namespace dua_btcpp_nodes
{

ArmComponent::ArmComponent(
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
    throw std::runtime_error("dua_btcpp_nodes::ArmComponent::ArmComponent: Action name not found in input ports.");
  }

  const std::string & bb_action_name = port_it->second;
  if (bb_action_name.empty()) {
    throw std::runtime_error("dua_btcpp_nodes::ArmComponent::ArmComponent: Action name is empty.");
  }

  if (!isBlackboardPointer(bb_action_name)) {
    // Action name has been hardcoded so we can use it right away
    action_client_ = clients_cache_->get_action_client<Arm>(bb_action_name, wait_server);
  }
  // If we reach this point, it means we are using a blackboard pointer which will be set later on,
  // so creation of the client is deferred to the tick method call
}

ArmComponent::~ArmComponent()
{}

BT::PortsList ArmComponent::providedPorts()
{
  return {
    BT::InputPort<std::string>("action_name", "Name of the ROS 2 Arm action"),
    BT::InputPort<int>("goal_timeout", 0, "Timeout to wait for the goal [ms]"),
    BT::InputPort<int>("result_timeout", 0, "Timeout to wait for the result [ms]"),
    BT::OutputPort<int>("code", "CommandResultStamped result code"),
    BT::OutputPort<std::string>("message", "CommandResultStamped message"),
    BT::OutputPort<CommandResultStamped>("result", "CommandResultStamped result message")
  };
}

BT::NodeStatus ArmComponent::tick()
{
  // First, create the client if it wasn't created already
  if (action_client_ == nullptr) {
    auto action_name = getInput<std::string>("action_name");
    action_client_ = clients_cache_->get_action_client<Arm>(action_name.value(), wait_server_);
  }

  // Try to arm the component
  int goal_timeout_ms = getInput<int>("goal_timeout").value();
  int result_timeout_ms = getInput<int>("result_timeout").value();
  Arm::Goal arm_goal{};
  auto arm_res = action_client_->call_sync(arm_goal, spin_, false, goal_timeout_ms, result_timeout_ms);
  bool success = std::get<0>(arm_res) &&
    (std::get<1>(arm_res) == rclcpp_action::ResultCode::SUCCEEDED ||
     std::get<1>(arm_res) == rclcpp_action::ResultCode::UNKNOWN) &&
    std::get<2>(arm_res) != nullptr &&
    (*(std::get<2>(arm_res))).result.result == CommandResultStamped::SUCCESS;

  // Set output ports
  if (std::get<2>(arm_res) != nullptr) {
    setOutput<int>("code", static_cast<int>((*(std::get<2>(arm_res))).result.result));
    setOutput<std::string>("message", (*(std::get<2>(arm_res))).result.error_msg);
    setOutput<CommandResultStamped>("result", (*(std::get<2>(arm_res))).result);
  } else {
    setOutput<int>("code", static_cast<int>(CommandResultStamped::ERROR));
    setOutput<std::string>("message", "Server did not return a valid result");
    setOutput<CommandResultStamped>("result", CommandResultStamped{});
  }

  if (success) {
    return BT::NodeStatus::SUCCESS;
  } else {
    return BT::NodeStatus::FAILURE;
  }
}

} // namespace dua_btcpp_nodes
