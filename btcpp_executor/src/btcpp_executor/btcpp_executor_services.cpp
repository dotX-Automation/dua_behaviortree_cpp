/**
 * BehaviorTree.CPP tree executor service callbacks.
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

void BTExecutor::enable_clbk(
  SetBool::Request::SharedPtr req,
  SetBool::Response::SharedPtr res)
{
  if (req->data) {
    bool expected = false;
    if (running_.compare_exchange_strong(
        expected,
        true,
        std::memory_order_release,
        std::memory_order_acquire))
    {
      // Start executor
      if (!start_bt_executor()) {
        running_.store(false, std::memory_order_release);
        res->set__success(false);
        res->set__message("Failed to start executor");
        return;
      }
    }
    res->set__success(true);
    res->set__message("");
  } else {
    bool expected = true;
    if (running_.compare_exchange_strong(
        expected,
        false,
        std::memory_order_release,
        std::memory_order_acquire))
    {
      if (bt_executor_.joinable()) {
        bt_executor_.join();
      }
    }
    res->set__success(true);
    res->set__message("");
  }
}

void BTExecutor::save_bb_clbk(
  SaveFile::Request::SharedPtr req,
  SaveFile::Response::SharedPtr res)
{
  res->result.header.set__stamp(get_clock()->now());

  // Consistency check
  if (global_blackboard_ == nullptr) {
    std::string err_msg = "No active blackboard";
    RCLCPP_ERROR(get_logger(), "Failed to export global blackboard: %s", err_msg.c_str());
    res->result.set__result(CommandResultStamped::ERROR);
    res->result.set__error_msg(err_msg);
    return;
  }

  // Export bb contents
  nlohmann::json bb_json;
  try {
    bb_json = BT::ExportBlackboardToJSON(*global_blackboard_);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Failed to export global blackboard: %s", e.what());
    res->result.set__result(CommandResultStamped::ERROR);
    res->result.set__error_msg(std::string(e.what()));
    return;
  }

  // Save bb contents to file
  try {
    std::ofstream out_file(req->path);
    out_file << std::setw(4) << bb_json << std::endl;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Failed to save global blackboard to file: %s", e.what());
    res->result.set__result(CommandResultStamped::ERROR);
    res->result.set__error_msg(std::string(e.what()));
    return;
  }

  RCLCPP_INFO(get_logger(), "Global blackboard saved to '%s'", req->path.c_str());
  res->result.set__result(CommandResultStamped::SUCCESS);
  res->result.set__error_msg("");
}

} // namespace btcpp_executor
