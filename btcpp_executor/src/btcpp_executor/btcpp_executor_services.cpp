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

} // namespace btcpp_executor
