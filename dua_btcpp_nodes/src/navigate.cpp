/**
 * Navigates to a target position.
 *
 * dotX Automation s.r.l. <info@dotxautomation.com>
 *
 * August 24, 2025
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

#include <dua_btcpp_nodes/navigate.hpp>

namespace dua_btcpp_nodes
{

NavigateNode::NavigateNode(
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
    throw std::runtime_error("dua_btcpp_nodes::NavigateNode::NavigateNode: Action name not found in input ports.");
  }

  const std::string & bb_action_name = port_it->second;
  if (bb_action_name.empty()) {
    throw std::runtime_error("dua_btcpp_nodes::NavigateNode::NavigateNode: Action name is empty.");
  }

  if (!isBlackboardPointer(bb_action_name)) {
    // Action name has been hardcoded so we can use it right away
    action_client_ = clients_cache_->get_action_client<Navigate>(bb_action_name, wait_server);
  }
  // If we reach this point, it means we are using a blackboard pointer which will be set later on,
  // so creation of the client is deferred to the tick method call
}

NavigateNode::~NavigateNode()
{}

BT::PortsList NavigateNode::providedPorts()
{
  return {
    BT::InputPort<std::string>("action_name", "Name of the ROS 2 Navigate action"),
    BT::InputPort<int>("timeout", 0, "Client operations timeout [ms] (0 means no timeout: wait indefinitely and poll instantaneously)"),
    BT::InputPort<dua_btcpp_types::Point3D>("target", "Target position as dua_btcpp_types::Point3D")
  };
}

BT::NodeStatus NavigateNode::onStart()
{
  // First, create the client if it wasn't created already
  if (action_client_ == nullptr) {
    auto action_name = getInput<std::string>("action_name");
    action_client_ = clients_cache_->get_action_client<Navigate>(action_name.value(), wait_server_);
  }

  // Get the target position
  dua_btcpp_types::Point3D tgt = getInput<dua_btcpp_types::Point3D>("target").value();

  // Fill the action goal
  Navigate::Goal nav_goal{};
  nav_goal.target_pose.header.set__stamp(ros2_node_->get_clock()->now());
  nav_goal.target_pose.header.set__frame_id(tgt.frame_id);
  nav_goal.target_pose.pose.position.set__x(tgt.x);
  nav_goal.target_pose.pose.position.set__y(tgt.y);
  nav_goal.target_pose.pose.position.set__z(tgt.z);
  nav_goal.target_pose.pose.orientation.set__x(tgt.q_x);
  nav_goal.target_pose.pose.orientation.set__y(tgt.q_y);
  nav_goal.target_pose.pose.orientation.set__z(tgt.q_z);
  nav_goal.target_pose.pose.orientation.set__w(tgt.q_w);

  // Start the navigation operation
  int timeout_ms = getInput<int>("timeout").value();
  current_goal_ = action_client_->send_goal_sync(nav_goal, spin_, timeout_ms);
  if (current_goal_ == nullptr) {
    RCLCPP_ERROR(
      ros2_node_->get_logger(),
      "Navigate goal rejected");
    return BT::NodeStatus::FAILURE;
  }

  if (!spin_) {
    current_res_future_ = action_client_->get_result(current_goal_);
  }

  RCLCPP_WARN(
    ros2_node_->get_logger(),
    "Started navigation to: (%.2f, %.2f, %.2f) (%.2f, %.2f, %.2f, %.2f) (%s)",
    tgt.x, tgt.y, tgt.z,
    tgt.q_x, tgt.q_y, tgt.q_z, tgt.q_w,
    tgt.frame_id.c_str());
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus NavigateNode::onRunning()
{
  // Check if the goal has been completed
  int timeout_ms = getInput<int>("timeout").value();
  rclcpp_action::ClientGoalHandle<Navigate>::WrappedResult goal_result;
  if (!spin_) {
    // Have to manually check the future
    if (timeout_ms <= 0) {
      // Poll instantaneously
      timeout_ms = 1;
    }
    auto f_status = current_res_future_.wait_for(std::chrono::milliseconds(timeout_ms));
    if (f_status == std::future_status::ready) {
      goal_result = current_res_future_.get();
    } else {
      return BT::NodeStatus::RUNNING;
    }
  } else {
    // Have to spin, use the API
    auto res_ptr = action_client_->get_result_sync(current_goal_, true, timeout_ms);
    if (res_ptr != nullptr) {
      goal_result = *res_ptr;
    } else {
      return BT::NodeStatus::RUNNING;
    }
  }
  current_goal_.reset();

  // Check operation result
  rclcpp_action::ResultCode code = goal_result.code;
  Navigate::Result::SharedPtr res = goal_result.result;
  bool success = (code == rclcpp_action::ResultCode::SUCCEEDED || code == rclcpp_action::ResultCode::UNKNOWN) &&
    res->result.result == CommandResultStamped::SUCCESS;
  if (success) {
    RCLCPP_WARN(ros2_node_->get_logger(), "Navigate succeeded");
    return BT::NodeStatus::SUCCESS;
  } else {
    RCLCPP_ERROR(ros2_node_->get_logger(), "Navigate failed");
    return BT::NodeStatus::FAILURE;
  }
}

void NavigateNode::onHalted()
{
  // Cancel the current goal, if present
  if (current_goal_ != nullptr) {
    int timeout_ms = getInput<int>("timeout").value();
    action_client_->cancel_and_get_result_sync(current_goal_, spin_, timeout_ms);
    current_goal_.reset();
  }
}

} // namespace dua_btcpp_nodes
