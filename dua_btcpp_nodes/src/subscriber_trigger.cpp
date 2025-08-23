/**
 * Waits for a trigger message.
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

#define UNUSED(arg) (void)(arg)

#include <dua_btcpp_nodes/subscriber_trigger.hpp>

namespace dua_btcpp_nodes
{

SubscriberTrigger::SubscriberTrigger(
  const std::string & node_name,
  const BT::NodeConfig & node_config,
  dua_node::NodeBase * ros2_node)
: BT::StatefulActionNode(node_name, node_config),
  ros2_node_(ros2_node)
{
  // The following cases must be handled to create the topic subscriber
  // - We use now the topic_name in the port: a static string
  // - We use later the topic_name in the port: a blackboard entry
  // If neither is feasible, we throw an exception

  auto port_it = config().input_ports.find("topic_name");
  if (port_it == config().input_ports.end()) {
    throw std::runtime_error("dua_btcpp_nodes::SubscriberTrigger::SubscriberTrigger: Topic name not found in input ports.");
  }

  const std::string & bb_topic_name = port_it->second;
  if (bb_topic_name.empty()) {
    throw std::runtime_error("dua_btcpp_nodes::SubscriberTrigger::SubscriberTrigger: Topic name is empty.");
  }

  if (!isBlackboardPointer(bb_topic_name)) {
    // Topic name has been hardcoded so we can use it right away
    cgroup_ = ros2_node_->dua_create_exclusive_cgroup();
    subscriber_ = ros2_node_->dua_create_subscription<Empty>(
      bb_topic_name,
      std::bind(
        &SubscriberTrigger::trigger_clbk,
        this,
        std::placeholders::_1),
      dua_qos::Reliable::get_datum_qos(),
      cgroup_);
  }
  // If we reach this point, it means we are using a blackboard pointer which will be set later on,
  // so creation of the subscriber is deferred to the tick method call
}

SubscriberTrigger::~SubscriberTrigger()
{}

BT::PortsList SubscriberTrigger::providedPorts()
{
  return {
    BT::InputPort<std::string>("topic_name", "Name of the ROS 2 topic to subscribe to."),
    BT::InputPort<bool>("reset", false, "Whether to reset the trigger state after it is received.")
  };
}

BT::NodeStatus SubscriberTrigger::onStart()
{
  // Check if the trigger was already received and the node is just being ticked again
  if (trigger_ && !getInput<bool>("reset").value()) {
    return BT::NodeStatus::SUCCESS;
  } else {
    trigger_ = false;
  }

  // Create the subscriber if it wasn't created already
  if (subscriber_ == nullptr) {
    std::string topic_name = getInput<std::string>("topic_name").value();
    cgroup_ = ros2_node_->dua_create_exclusive_cgroup();
    subscriber_ = ros2_node_->dua_create_subscription<Empty>(
      topic_name,
      std::bind(
        &SubscriberTrigger::trigger_clbk,
        this,
        std::placeholders::_1),
      dua_qos::Reliable::get_datum_qos(),
      cgroup_);
  }

  RCLCPP_INFO(
    ros2_node_->get_logger(),
    "Waiting for a trigger on %s",
    subscriber_->get_topic_name());

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SubscriberTrigger::onRunning()
{
  if (trigger_) {
    subscriber_.reset();
    if (getInput<bool>("reset").value()) {
      trigger_ = false;
    }
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::RUNNING;
}

void SubscriberTrigger::onHalted()
{
  subscriber_.reset();
  trigger_ = false;
}

void SubscriberTrigger::trigger_clbk(const Empty::SharedPtr msg)
{
  UNUSED(msg);
  trigger_ = true;
}

} // namespace dua_btcpp_nodes
