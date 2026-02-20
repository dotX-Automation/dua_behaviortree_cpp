/**
 * Executes a vertical landing.
 *
 * dotX Automation s.r.l. <info@dotxautomation.com>
 *
 * August 25, 2025
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

#include <dua_btcpp_nodes/vertical_landing.hpp>

namespace dua_btcpp_nodes
{

VerticalLandingNode::VerticalLandingNode(
  const std::string & node_name,
  const BT::NodeConfig & node_config,
  dua_node::NodeBase * ros2_node,
  const dua_btcpp_base::EntityManager::SharedPtr & clients_cache,
  bool wait_server,
  bool spin)
: BT::StatefulActionNode(node_name, node_config),
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
    throw std::runtime_error("dua_btcpp_nodes::VerticalLandingNode::VerticalLandingNode: Action name not found in input ports.");
  }

  const std::string & bb_action_name = port_it->second;
  if (bb_action_name.empty()) {
    throw std::runtime_error("dua_btcpp_nodes::VerticalLandingNode::VerticalLandingNode: Action name is empty.");
  }

  if (!isBlackboardPointer(bb_action_name)) {
    // Action name has been hardcoded so we can use it right away
    action_client_ = clients_cache_->get_action_client<Landing>(bb_action_name, wait_server);
  }
  // If we reach this point, it means we are using a blackboard pointer which will be set later on,
  // so creation of the client is deferred to the tick method call
}

VerticalLandingNode::~VerticalLandingNode()
{}

BT::PortsList VerticalLandingNode::providedPorts()
{
  return {
    BT::InputPort<std::string>("action_name", "Name of the ROS 2 Landing action"),
    BT::InputPort<double>("decision_altitude", "Altitude [m] at which final descent may be started"),
    BT::InputPort<bool>("descend", "Whether to perform a controlled descent or go straight down"),
    BT::InputPort<std::string>("frame_id", "Reference frame for the decision altitude setpoint"),
    BT::InputPort<int>("timeout", 0, "Client operations timeout [ms] (0 means no timeout: wait indefinitely and poll instantaneously)")
  };
}

BT::NodeStatus VerticalLandingNode::onStart()
{
  // First, create the client if it wasn't created already
  if (action_client_ == nullptr) {
    auto action_name = getInput<std::string>("action_name");
    action_client_ = clients_cache_->get_action_client<Landing>(action_name.value(), wait_server_);
  }

  // Get action goal data
  double decision_altitude = getInput<double>("decision_altitude").value();
  bool descend = getInput<bool>("descend").value();
  std::string frame_id = getInput<std::string>("frame_id").value();

  // Fill the action goal
  Landing::Goal lnd_goal{};
  lnd_goal.descend = descend;
  lnd_goal.minimums.header.set__stamp(ros2_node_->get_clock()->now());
  lnd_goal.minimums.header.set__frame_id(frame_id);
  lnd_goal.minimums.point.set__z(decision_altitude);

  // Start the landing operation
  int timeout_ms = getInput<int>("timeout").value();
  current_goal_ = action_client_->send_goal_sync(lnd_goal, spin_, timeout_ms);
  if (current_goal_ == nullptr) {
    RCLCPP_ERROR(
      ros2_node_->get_logger(),
      "Landing goal rejected");
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_WARN(
    ros2_node_->get_logger(),
    "Requested landing at %.2f m (%s)",
    decision_altitude,
    frame_id.c_str());
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus VerticalLandingNode::onRunning()
{
  int timeout_ms = getInput<int>("timeout").value();
  if (timeout_ms <= 0) {
    // Poll instantaneously
    timeout_ms = 10;
  }

  // Ask for the result (once)
  if (current_res_future_ == nullptr) {
    current_res_future_ =
      std::make_unique<std::shared_future<rclcpp_action::ClientGoalHandle<Landing>::WrappedResult>>(
        action_client_->get_result(current_goal_));
  }

  // Check if the goal has been completed
  rclcpp_action::ClientGoalHandle<Landing>::WrappedResult goal_result;
  if (!spin_) {
    // Have to manually check the future
    auto f_status = current_res_future_->wait_for(std::chrono::milliseconds(timeout_ms));
    if (f_status == std::future_status::ready) {
      goal_result = current_res_future_->get();
    } else {
      return BT::NodeStatus::RUNNING;
    }
  } else {
    // Have to spin, use the API
    auto err = rclcpp::spin_until_future_complete(
      ros2_node_->shared_from_this(),
      *current_res_future_,
      std::chrono::milliseconds(timeout_ms));
    if (err == rclcpp::FutureReturnCode::SUCCESS) {
      goal_result = current_res_future_->get();
    } else {
      return BT::NodeStatus::RUNNING;
    }
  }
  current_goal_.reset();
  current_res_future_.reset();

  // Check operation result
  rclcpp_action::ResultCode code = goal_result.code;
  Landing::Result::SharedPtr res = goal_result.result;
  bool success = (code == rclcpp_action::ResultCode::SUCCEEDED || code == rclcpp_action::ResultCode::UNKNOWN) &&
    res->result.result == CommandResultStamped::SUCCESS;
  if (success) {
    RCLCPP_WARN(ros2_node_->get_logger(), "Landing succeeded");
    return BT::NodeStatus::SUCCESS;
  } else {
    RCLCPP_ERROR(ros2_node_->get_logger(), "Landing failed");
    return BT::NodeStatus::FAILURE;
  }
}

void VerticalLandingNode::onHalted()
{
  // Cancel the current goal, if present
  if (current_goal_ != nullptr) {
    int timeout_ms = getInput<int>("timeout").value();

    try {
      // This might fail upon e.g. process termination
      action_client_->cancel_and_get_result_sync(current_goal_, spin_, timeout_ms);
    } catch (const std::exception & e) {
      RCLCPP_FATAL(
        ros2_node_->get_logger(),
        "Failed to cancel Landing on halt: %s",
        e.what());
    }
    current_goal_.reset();
  }
}

} // namespace dua_btcpp_nodes
