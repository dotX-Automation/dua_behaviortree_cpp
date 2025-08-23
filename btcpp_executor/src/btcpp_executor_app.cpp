/**
 * BehaviorTree.CPP tree executor standalone application.
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

#include <cstdlib>
#include <csignal>

#include <rclcpp/rclcpp.hpp>

#include <ros2_app_manager/ros2_app_manager.hpp>
#include <ros2_signal_handler/ros2_signal_handler.hpp>

#include <btcpp_executor/btcpp_executor.hpp>

using namespace dua_app_management;

int main(int argc, char ** argv)
{
  ROS2AppManager<rclcpp::executors::MultiThreadedExecutor,
    btcpp_executor::BTExecutor> app_manager(
    argc,
    argv,
    "bt_executor_app");

  SignalHandler & sig_handler = SignalHandler::get_global_signal_handler();
  sig_handler.init(
    app_manager.get_context(),
    "bt_executor_app_signal_handler",
    app_manager.get_executor());
  sig_handler.install(SIGINT);
  sig_handler.install(SIGTERM);
  sig_handler.install(SIGQUIT);
  sig_handler.ignore(SIGHUP);
  sig_handler.ignore(SIGUSR1);
  sig_handler.ignore(SIGUSR2);

  app_manager.run();

  app_manager.shutdown();
  sig_handler.fini();

  exit(EXIT_SUCCESS);
}
