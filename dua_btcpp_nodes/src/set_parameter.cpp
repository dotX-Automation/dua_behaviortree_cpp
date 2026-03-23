/**
 * Sets a parameter of a ROS 2 node.
 *
 * dotX Automation s.r.l. <info@dotxautomation.com>
 *
 * March 23, 2026
 */

/**
 * Copyright 2026 dotX Automation s.r.l.
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

#include <dua_btcpp_nodes/set_parameter.hpp>

namespace dua_btcpp_nodes
{

SetParameter::SetParameter(
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
    throw std::runtime_error("dua_btcpp_nodes::SetParameter::SetParameter: Service name not found in input ports.");
  }

  const std::string & bb_service_name = port_it->second;
  if (bb_service_name.empty()) {
    throw std::runtime_error("dua_btcpp_nodes::SetParameter::SetParameter: Service name is empty.");
  }

  if (!isBlackboardPointer(bb_service_name)) {
    // Service name has been hardcoded so we can use it right away
    service_client_ = clients_cache_->get_service_client<SetParameters>(bb_service_name, wait_server);
  }
  // If we reach this point, it means we are using a blackboard pointer which will be set later on,
  // so creation of the client is deferred to the tick method call
}

SetParameter::~SetParameter()
{}

BT::PortsList SetParameter::providedPorts()
{
  return {
    BT::InputPort<std::string>("pname", "", "Name of the parameter to set"),
    BT::InputPort<ParamType>("ptype", ParamType::PARAM_UNDEF, "Type of the ROS 2 parameter"),
    BT::InputPort<std::string>("pvalue", "", "Value of the parameter encoded as a string"),
    BT::InputPort<std::string>("service_name", "Name of the ROS 2 SetParameters service"),
    BT::OutputPort<std::string>("message", "Message returned by the server")
  };
}

BT::NodeStatus SetParameter::tick()
{
  // First, create the client if it wasn't created already
  if (service_client_ == nullptr) {
    auto service_name = getInput<std::string>("service_name");
    service_client_ = clients_cache_->get_service_client<SetParameters>(service_name.value(), wait_server_);
  }

  // Get the name of the parameter
  std::string p_name = getInput<std::string>("pname").value_or("");
  if (p_name.empty()) {
    RCLCPP_ERROR(ros2_node_->get_logger(), "SetParameter: empty parameter name");
    return BT::NodeStatus::FAILURE;
  }

  // Get the type of the parameter
  ParamType p_type = getInput<ParamType>("ptype").value_or(ParamType::PARAM_UNDEF);
  if (p_type == ParamType::PARAM_UNDEF) {
    RCLCPP_ERROR(ros2_node_->get_logger(), "SetParameter: undefined parameter type");
    return BT::NodeStatus::FAILURE;
  }
  uint8_t p_type_ros = static_cast<uint8_t>(p_type);

  // Get the new value for the parameter
  std::string p_value = getInput<std::string>("pvalue").value_or("");
  if (p_value.empty()) {
    RCLCPP_ERROR(ros2_node_->get_logger(), "SetParameter: empty parameter value");
    return BT::NodeStatus::FAILURE;
  }

  // Cast the parameter value
  ParameterValue value_msg{};
  value_msg.set__type(p_type_ros);
  switch (p_type) {
    case ParamType::PARAM_BOOL:
      value_msg.set__bool_value(p_value == "true" || p_value == "True" || p_value == "TRUE");
      break;
    case ParamType::PARAM_DOUBLE:
      try {
        value_msg.set__double_value(std::stod(p_value));
      } catch (const std::exception & e) {
        RCLCPP_ERROR(ros2_node_->get_logger(), "SetParameter: invalid double value: %s", e.what());
        return BT::NodeStatus::FAILURE;
      }
      break;
    case ParamType::PARAM_INT:
      try {
        value_msg.set__integer_value(std::stoll(p_value));
      } catch (const std::exception & e) {
        RCLCPP_ERROR(ros2_node_->get_logger(), "SetParameter: invalid integer value: %s", e.what());
        return BT::NodeStatus::FAILURE;
      }
      break;
    case ParamType::PARAM_STRING:
      value_msg.set__string_value(p_value);
      break;
    default:
      // Should never happen at this point
      RCLCPP_ERROR(ros2_node_->get_logger(), "SetParameter: unsupported parameter type");
      return BT::NodeStatus::FAILURE;
  }

  // Try to set the parameter
  Parameter param_msg{};
  param_msg.set__name(p_name);
  param_msg.set__value(value_msg);
  SetParameters::Request::SharedPtr req = std::make_shared<SetParameters::Request>();
  req->parameters.push_back(param_msg);
  SetParameters::Response::SharedPtr res = service_client_->call_sync(req, spin_);
  bool success = res != nullptr && res->results.size() == 1 && res->results[0].successful;
  if (res != nullptr && res->results.size() == 1) {
    setOutput<std::string>("message", res->results[0].reason);
  } else {
    setOutput<std::string>("message", "Server did not return a valid response");
  }

  if (success) {
    return BT::NodeStatus::SUCCESS;
  } else {
    return BT::NodeStatus::FAILURE;
  }
}

} // namespace dua_btcpp_nodes
