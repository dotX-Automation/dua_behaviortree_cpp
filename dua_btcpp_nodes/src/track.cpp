/**
 * Tracks a target.
 *
 * dotX Automation s.r.l. <info@dotxautomation.com>
 *
 * August 26, 2025
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

#include <dua_btcpp_nodes/track.hpp>

namespace dua_btcpp_nodes
{

TrackNode::TrackNode(
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
    throw std::runtime_error("dua_btcpp_nodes::TrackNode::TrackNode: Action name not found in input ports.");
  }

  const std::string & bb_action_name = port_it->second;
  if (bb_action_name.empty()) {
    throw std::runtime_error("dua_btcpp_nodes::TrackNode::TrackNode: Action name is empty.");
  }

  if (!isBlackboardPointer(bb_action_name)) {
    // Action name has been hardcoded so we can use it right away
    action_client_ = clients_cache_->get_action_client<Track>(bb_action_name, wait_server);
  }
  // If we reach this point, it means we are using a blackboard pointer which will be set later on,
  // so creation of the client is deferred to the tick method call
}

TrackNode::~TrackNode()
{}

BT::PortsList TrackNode::providedPorts()
{
  return {
    BT::InputPort<std::string>("action_name", "Name of the ROS 2 Track action"),
    BT::InputPort<TrackSide>("side", TrackSide::TRACK_CENTER, "Side at which the target has been spotted"),
    BT::InputPort<bool>("stop_centered", "Whether to stop the operation when the target has been centered in the view"),
    BT::InputPort<std::string>("target_id", "ID of the target to track"),
    BT::InputPort<int>("timeout", 0, "Client operations timeout [ms] (0 means no timeout: wait indefinitely and poll instantaneously)")
  };
}

BT::NodeStatus TrackNode::onStart()
{
  // First, create the client if it wasn't created already
  if (action_client_ == nullptr) {
    auto action_name = getInput<std::string>("action_name");
    action_client_ = clients_cache_->get_action_client<Track>(action_name.value(), wait_server_);
  }

  // Get action goal data
  TrackSide side = getInput<TrackSide>("side").value();
  bool stop_centered = getInput<bool>("stop_centered").value();
  std::string target_id = getInput<std::string>("target_id").value();

  // Fill the action goal
  Track::Goal track_goal{};
  track_goal.set__start_side(side);
  track_goal.set__stop_when_centered(stop_centered);
  track_goal.set__target_id(target_id);

  // Start the track operation
  int timeout_ms = getInput<int>("timeout").value();
  current_goal_ = action_client_->send_goal_sync(track_goal, spin_, timeout_ms);
  if (current_goal_ == nullptr) {
    RCLCPP_ERROR(
      ros2_node_->get_logger(),
      "Track goal rejected");
    return BT::NodeStatus::FAILURE;
  }

  if (!spin_) {
    current_res_future_ = action_client_->get_result(current_goal_);
  }

  RCLCPP_WARN(
    ros2_node_->get_logger(),
    stop_centered ? "Requested tracking of '%s' (%d)" : "Requested continuous tracking of '%s' (%d)",
    target_id.c_str(),
    static_cast<int>(side));
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus TrackNode::onRunning()
{
  int timeout_ms = getInput<int>("timeout").value();
  if (timeout_ms <= 0) {
    // Poll instantaneously
    timeout_ms = 10;
  }

  // Check if the goal has been completed
  rclcpp_action::ClientGoalHandle<Track>::WrappedResult goal_result;
  if (!spin_) {
    // Have to manually check the future
    auto f_status = current_res_future_.wait_for(std::chrono::milliseconds(timeout_ms));
    if (f_status == std::future_status::ready) {
      goal_result = current_res_future_.get();
    } else {
      return BT::NodeStatus::RUNNING;
    }
  } else {
    // Have to spin, use the API
    auto err = rclcpp::spin_until_future_complete(
      ros2_node_->shared_from_this(),
      current_res_future_,
      std::chrono::milliseconds(timeout_ms));
    if (err == rclcpp::FutureReturnCode::SUCCESS) {
      goal_result = current_res_future_.get();
    } else {
      return BT::NodeStatus::RUNNING;
    }
  }
  current_goal_.reset();

  // Check operation result
  rclcpp_action::ResultCode code = goal_result.code;
  Track::Result::SharedPtr res = goal_result.result;
  bool success = (code == rclcpp_action::ResultCode::SUCCEEDED || code == rclcpp_action::ResultCode::UNKNOWN) &&
    res->result.result == CommandResultStamped::SUCCESS;
  if (success) {
    RCLCPP_WARN(ros2_node_->get_logger(), "Track succeeded");
    return BT::NodeStatus::SUCCESS;
  } else {
    RCLCPP_ERROR(ros2_node_->get_logger(), "Track failed");
    return BT::NodeStatus::FAILURE;
  }
}

void TrackNode::onHalted()
{
  // Cancel the current goal, if present
  if (current_goal_ != nullptr) {
    int timeout_ms = getInput<int>("timeout").value();
    action_client_->cancel_and_get_result_sync(current_goal_, spin_, timeout_ms);
    current_goal_.reset();
  }
}

} // namespace dua_btcpp_nodes
