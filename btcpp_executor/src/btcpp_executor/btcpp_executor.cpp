/**
 * BehaviorTree.CPP tree executor implementation.
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

#include <btcpp_executor/btcpp_executor.hpp>

#include <dua_btcpp_types/point_3d.hpp>

namespace btcpp_executor
{

BTExecutor::BTExecutor(const rclcpp::NodeOptions & opts)
: NodeBase("btcpp_executor", opts, true)
{
  dua_init_node();

  // Create plugins loader
  plugins_loader_ = std::make_unique<pluginlib::ClassLoader<dua_btcpp_base::BaseRegister>>(
    "dua_btcpp_base",
    "dua_btcpp_base::BaseRegister");

  // Create entities cache
  entity_manager_ = std::make_shared<dua_btcpp_base::EntityManager>(this);

  // Register JSON definitions of BT.CPP custom types
  BT::RegisterJsonDefinition<dua_btcpp_types::Point3D>();

  RCLCPP_INFO(get_logger(), "Node initialized");

  // Start executor, if requested
  if (get_parameter("autostart").as_bool()) {
    start_bt_executor();
  }
}

BTExecutor::~BTExecutor()
{
  running_.store(false, std::memory_order_release);
  if (bt_executor_.joinable()) {
    bt_executor_.join();
  }
}

void BTExecutor::init_cgroups()
{
  enable_cgroup_ = dua_create_exclusive_cgroup();
}

void BTExecutor::init_service_servers()
{
  // enable
  enable_server_ = dua_create_service_server<SetBool>(
    "~/enable",
    std::bind(
      &BTExecutor::enable_clbk,
      this,
      std::placeholders::_1,
      std::placeholders::_2),
    enable_cgroup_);
}

} // namespace btcpp_executor
