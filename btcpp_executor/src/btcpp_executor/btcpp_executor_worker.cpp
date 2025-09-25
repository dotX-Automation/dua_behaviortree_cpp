/**
 * BehaviorTree.CPP tree executor worker thread implementation.
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

namespace btcpp_executor
{

void BTExecutor::bt_executor_routine()
{
  int64_t tick_period_ms = static_cast<int64_t>(
    std::round(
      1000.0 / static_cast<double>(get_parameter("btcpp.tick_rate").as_int())));
  bool catch_exceptions = get_parameter("btcpp.debug.catch_exceptions").as_bool();

  RCLCPP_WARN(get_logger(), "BT executor started");

  BT::NodeStatus status = BT::NodeStatus::IDLE;
  while (true) {
    // Check if cancellation was requested
    if (!running_.load(std::memory_order_acquire)) {
      bt_->haltTree();
      break;
    }

    // Check if middleware crashed
    if (!rclcpp::ok()) {
      RCLCPP_FATAL(get_logger(), "rclcpp not ok");
      break;
    }

    // Do one tick
    if (catch_exceptions) {
      bool ok = true;
      try {
        status = bt_->tickOnce();
      } catch (const std::exception & e) {
        RCLCPP_FATAL(get_logger(), "Exception while ticking BT: %s", e.what());
        ok = false;
      }
      if (!ok) {
        break;
      }
    } else {
      status = bt_->tickOnce();
    }

    // Check tree status
    if (status != BT::NodeStatus::RUNNING) {
      break;
    }

    // Sleep, tracking events
    bt_->sleep(std::chrono::milliseconds(tick_period_ms));
  }

  if (status == BT::NodeStatus::SUCCESS) {
    RCLCPP_WARN(get_logger(), "BT execution succeeded");
  } else if (status == BT::NodeStatus::RUNNING) {
    RCLCPP_WARN(get_logger(), "BT execution halted");
  } else if (status == BT::NodeStatus::FAILURE) {
    RCLCPP_ERROR(get_logger(), "BT execution failed");
  } else {
    RCLCPP_FATAL(
      get_logger(),
      "BT execution stopped with status %s",
      node_status_to_string(status).c_str());
  }

  // Perform debugging operations
  if (debug_) {
    // Print all node statistics
    if (get_parameter("btcpp.debug.print_statistics").as_bool()) {
      std::cout << "------ BT RUN STATS ------" << std::endl;
      std::map<uint16_t, std::string> ordered_uid_to_path;
      for (const auto & [name, uid] : bt_observer_->pathToUID()) {
        ordered_uid_to_path[uid] = name;
      }
      for (const auto & [uid, name] : ordered_uid_to_path) {
        const BT::TreeObserver::NodeStatistics & stats = bt_observer_->getStatistics(uid);
        std::cout << "[" << name
                  << "] \tTran/Succ/Fail/Skip:  " << stats.transitions_count
                  << "/" << stats.success_count
                  << "/" << stats.failure_count
                  << "/" << stats.skip_count
                  << " \tcurr/last:  " << node_status_to_string(stats.current_status)
                  << "/" << node_status_to_string(stats.last_result)
                  << std::endl;
      }
    }
  }

  // Clean up
  // Groot2 publisher will be eventually reset later on, since it must stay on to let the software
  // get the last updates
  bt_observer_.reset();
  bt_.reset();
  delete_global_blackboard();
  bt_factory_.reset();
  entity_manager_->clear();
  plugins_list_.clear();
  running_.store(false, std::memory_order_release);
}

} // namespace btcpp_executor
