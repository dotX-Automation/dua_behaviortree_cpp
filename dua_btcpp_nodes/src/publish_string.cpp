/**
 * Publishes a string on a given topic.
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

#include <dua_btcpp_nodes/publish_string.hpp>

namespace dua_btcpp_nodes
{

PublishString::PublishString(
  const std::string & node_name,
  const BT::NodeConfig & node_config,
  const dua_node::NodeBase::SharedPtr & ros2_node,
  const dua_btcpp_base::EntityManager::SharedPtr & publishers_cache)
: BT::SyncActionNode(node_name, node_config),
  ros2_node_(ros2_node),
  publishers_cache_(publishers_cache)
{
  // The following cases must be handled to create the topic publisher
  // - We use now the topic_name in the port: a static string
  // - We use later the topic_name in the port: a blackboard entry
  // If neither is feasible, we throw an exception

  auto port_it = config().input_ports.find("topic_name");
  if (port_it == config().input_ports.end()) {
    throw std::runtime_error("dua_btcpp_nodes::PublishString::PublishString: Topic name not found in input ports.");
  }

  const std::string & bb_topic_name = port_it->second;
  if (bb_topic_name.empty()) {
    throw std::runtime_error("dua_btcpp_nodes::PublishString::PublishString: Topic name is empty.");
  }

  if (!isBlackboardPointer(bb_topic_name)) {
    // Topic name has been hardcoded so we can use it right away
    publisher_ = publishers_cache_->get_publisher<String>(bb_topic_name);
  }
  // If we reach this point, it means we are using a blackboard pointer which will be set later on,
  // so creation of the publisher is deferred to the tick method call
}

PublishString::~PublishString()
{}

BT::PortsList PublishString::providedPorts()
{
  return {
    BT::InputPort<std::string>("topic_name", "Name of the ROS 2 topic to publish to."),
    BT::InputPort<std::string>("message", "Message string to publish."),
    BT::InputPort<bool>("ros_log", "Enables rclcpp logging of the message upon publishing.")
  };
}

BT::NodeStatus PublishString::tick()
{
  // First, create the publisher if it wasn't created already
  if (publisher_ == nullptr) {
    auto topic_name = getInput<std::string>("topic_name");
    publisher_ = publishers_cache_->get_publisher<String>(topic_name.value());
  }

  // Publish the message
  std::string msg = getInput<std::string>("message").value();
  String ros_msg{};
  ros_msg.set__data(msg);
  publisher_->publish(ros_msg);

  // Log the message, if required
  if (getInput<bool>("ros_log").value()) {
    RCLCPP_WARN(
      ros2_node_->get_logger(),
      "%s",
      msg.c_str());
  }

  return BT::NodeStatus::SUCCESS;
}

} // namespace dua_btcpp_nodes
