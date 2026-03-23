/**
 * Sets an array parameter of a ROS 2 node.
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

#include <dua_btcpp_nodes/set_array_parameter.hpp>

namespace dua_btcpp_nodes
{

SetArrayParameter::SetArrayParameter(
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
    throw std::runtime_error(
      "dua_btcpp_nodes::SetArrayParameter::SetArrayParameter: Service name not found in input ports.");
  }

  const std::string & bb_service_name = port_it->second;
  if (bb_service_name.empty()) {
    throw std::runtime_error(
      "dua_btcpp_nodes::SetArrayParameter::SetArrayParameter: Service name is empty.");
  }

  if (!isBlackboardPointer(bb_service_name)) {
    // Service name has been hardcoded so we can use it right away
    service_client_ = clients_cache_->get_service_client<SetParameters>(bb_service_name, wait_server);
  }
  // If we reach this point, it means we are using a blackboard pointer which will be set later on,
  // so creation of the client is deferred to the tick method call
}

SetArrayParameter::~SetArrayParameter()
{}

BT::PortsList SetArrayParameter::providedPorts()
{
  return {
    BT::InputPort<std::string>("pname", "", "Name of the array parameter to set"),
    BT::InputPort<ArrayParamType>("ptype", ArrayParamType::PARAM_ARRAY_UNDEF,"Type of the ROS 2 array parameter"),
    BT::InputPort<std::string>("pvalue", "", "Semicolon-separated values of the array parameter encoded as strings (spaces are ignored)"),
    BT::InputPort<std::string>("service_name", "Name of the ROS 2 SetParameters service"),
    BT::OutputPort<std::string>("message", "Message returned by the server")
  };
}

BT::NodeStatus SetArrayParameter::tick()
{
  // First, create the client if it wasn't created already
  if (service_client_ == nullptr) {
    auto service_name = getInput<std::string>("service_name");
    service_client_ = clients_cache_->get_service_client<SetParameters>(service_name.value(), wait_server_);
  }

  // Get the name of the parameter
  std::string p_name = getInput<std::string>("pname").value_or("");
  if (p_name.empty()) {
    RCLCPP_ERROR(ros2_node_->get_logger(), "SetArrayParameter: empty parameter name");
    return BT::NodeStatus::FAILURE;
  }

  // Get the type of the parameter
  ArrayParamType p_type = getInput<ArrayParamType>("ptype").value_or(ArrayParamType::PARAM_ARRAY_UNDEF);
  if (p_type == ArrayParamType::PARAM_ARRAY_UNDEF) {
    RCLCPP_ERROR(ros2_node_->get_logger(), "SetArrayParameter: undefined parameter type");
    return BT::NodeStatus::FAILURE;
  }
  uint8_t p_type_ros = static_cast<uint8_t>(p_type);

  // Get the new value for the parameter
  std::string p_value = getInput<std::string>("pvalue").value_or("");
  if (p_value.empty()) {
    RCLCPP_ERROR(ros2_node_->get_logger(), "SetArrayParameter: empty parameter value");
    return BT::NodeStatus::FAILURE;
  }

  // Tokenize the value string by ';'; each token is taken as-is (spaces are preserved)
  std::vector<std::string> tokens;
  std::string token;
  std::istringstream ss(p_value);
  while (std::getline(ss, token, ';')) {
    tokens.push_back(token);
  }
  if (tokens.empty()) {
    RCLCPP_ERROR(ros2_node_->get_logger(), "SetArrayParameter: no valid entries found in parameter value");
    return BT::NodeStatus::FAILURE;
  }

  // Cast the parameter value
  ParameterValue value_msg{};
  value_msg.set__type(p_type_ros);
  switch (p_type) {
    case ArrayParamType::PARAM_BOOL_ARRAY:
    {
      std::vector<bool> bool_values;
      bool_values.reserve(tokens.size());
      for (const auto & t : tokens) {
        bool_values.push_back(t == "true" || t == "True" || t == "TRUE");
      }
      value_msg.set__bool_array_value(bool_values);
      break;
    }
    case ArrayParamType::PARAM_DOUBLE_ARRAY:
    {
      std::vector<double> double_values;
      double_values.reserve(tokens.size());
      for (const auto & t : tokens) {
        try {
          double_values.push_back(std::stod(t));
        } catch (const std::exception & e) {
          RCLCPP_ERROR(
            ros2_node_->get_logger(),
            "SetArrayParameter: invalid double value '%s': %s", t.c_str(), e.what());
          return BT::NodeStatus::FAILURE;
        }
      }
      value_msg.set__double_array_value(double_values);
      break;
    }
    case ArrayParamType::PARAM_INT_ARRAY:
    {
      std::vector<int64_t> int_values;
      int_values.reserve(tokens.size());
      for (const auto & t : tokens) {
        try {
          int_values.push_back(std::stoll(t));
        } catch (const std::exception & e) {
          RCLCPP_ERROR(
            ros2_node_->get_logger(),
            "SetArrayParameter: invalid integer value '%s': %s", t.c_str(), e.what());
          return BT::NodeStatus::FAILURE;
        }
      }
      value_msg.set__integer_array_value(int_values);
      break;
    }
    case ArrayParamType::PARAM_STRING_ARRAY:
    {
      std::vector<std::string> string_values(tokens.begin(), tokens.end());
      value_msg.set__string_array_value(string_values);
      break;
    }
    default:
      // Should never happen at this point
      RCLCPP_ERROR(ros2_node_->get_logger(), "SetArrayParameter: unsupported parameter type");
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
