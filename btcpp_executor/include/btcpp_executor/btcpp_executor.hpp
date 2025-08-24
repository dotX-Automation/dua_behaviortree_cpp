/**
 * BehaviorTree.CPP tree executor.
 *
 * August 23, 2025
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

#pragma once

#include <atomic>
#include <chrono>
#include <cmath>
#include <iostream>
#include <fstream>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/bt_observer.h>
#include <behaviortree_cpp/loggers/groot2_publisher.h>
#include <behaviortree_cpp/xml_parsing.h>

#include <pluginlib/class_loader.hpp>

#include <dua_btcpp_base/base_register.hpp>

#include <rclcpp/rclcpp.hpp>

#include <dua_node_cpp/dua_node.hpp>
#include <dua_qos_cpp/dua_qos.hpp>

#include <std_srvs/srv/set_bool.hpp>

using namespace std_srvs::srv;

namespace btcpp_executor
{

/**
 * Executes BehaviorTree.CPP trees as a component of a ROS 2 network.
 */
class BTExecutor : public dua_node::NodeBase
{
public:
  /**
   * @brief Constructor.
   *
   * @param opts Node options.
   * @param verbose Verbosity flag.
   * @throws std::runtime_error on failure.
   */
  BTExecutor(const rclcpp::NodeOptions & opts = rclcpp::NodeOptions());

  /**
   * @brief Destructor.
   */
  virtual ~BTExecutor();

private:
  /* Node initialization routines. */
  /**
   * @brief Initializes node parameters.
   */
  void init_parameters() override;

  /**
   * @brief Initializes service servers.
   */
  void init_service_servers() override;

  /* Service servers. */
  rclcpp::Service<SetBool>::SharedPtr enable_server_;

  /* Service callbacks. */
  /**
   * @brief Handles executor state setting requests.
   *
   * @param req Request.
   * @param res Response.
   */
  void enable_clbk(
    SetBool::Request::SharedPtr req,
    SetBool::Response::SharedPtr res);

  /* Auxiliary routines. */
  /**
   * @brief Starts the executor.
   */
  bool start_bt_executor();

  /**
   * @brief Wraps global blackboard creation, handling persistency.
   */
  void create_global_blackboard();

  /**
   * @brief Wraps global blackboard deletion, handling persistency.
   */
  void delete_global_blackboard();

  /**
   * @brief Converts a NodeStatus value into a printable string.
   */
  std::string node_status_to_string(const BT::NodeStatus & status);

  /* Node status members. */
  /* BT executor thread. */
  std::thread bt_executor_;

  /**
   * @brief BT executor thread routine.
   */
  void bt_executor_routine();

  /* Node status flag. */
  std::atomic<bool> running_ = false;

  /* Plugins loader. */
  std::unique_ptr<pluginlib::ClassLoader<dua_btcpp_base::BaseRegister>> plugins_loader_ = nullptr;

  /* Plugins list. */
  std::map<std::string, std::shared_ptr<dua_btcpp_base::BaseRegister>> plugins_list_;

  /* BT.CPP factory. */
  std::unique_ptr<BT::BehaviorTreeFactory> bt_factory_ = nullptr;

  /* BT.CPP global blackboard. */
  BT::Blackboard::Ptr global_blackboard_ = nullptr;

  /* BT.CPP tree. */
  std::unique_ptr<BT::Tree> bt_ = nullptr;

  /* BT.CPP tree observer. */
  std::unique_ptr<BT::TreeObserver> bt_observer_ = nullptr;

  /* BT.CPP Groot2 publisher. */
  std::unique_ptr<BT::Groot2Publisher> groot2_publisher_ = nullptr;

  /* Cache of ROS 2 entities of this node. */
  dua_btcpp_base::EntityManager::SharedPtr entity_manager_ = nullptr;

  /* Node parameters. */
  bool debug_ = false;
  bool global_bb_persistent_ = false;
};

} // namespace btcpp_executor
