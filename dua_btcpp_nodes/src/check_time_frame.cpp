/**
 * Checks if the current time is within a specified time frame.
 *
 * dotX Automation s.r.l. <info@dotxautomation.com>
 *
 * February 20, 2026
 */

/**
 * Copyright 2026 dotX Automation s.r.l.
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

#include <ctime>
#include <regex>
#include <stdexcept>
#include <string>

#include <errno.h>
#include <string.h>
#include <time.h>

#include <dua_btcpp_nodes/check_time_frame.hpp>

namespace dua_btcpp_nodes
{

CheckTimeFrame::CheckTimeFrame(
  const std::string & node_name,
  const BT::NodeConfig & node_config,
  dua_node::NodeBase * ros2_node)
: BT::StatefulActionNode(node_name, node_config),
  ros2_node_(ros2_node)
{}

CheckTimeFrame::~CheckTimeFrame()
{}

BT::PortsList CheckTimeFrame::providedPorts()
{
  return {
    BT::InputPort<std::string>("start_time", "", "Start time of the time frame (format: HH:MM:SS, 24hr)"),
    BT::InputPort<std::string>("end_time", "", "End time of the time frame (format: HH:MM:SS, 24hr)"),
    BT::InputPort<bool>("wait", false, "Return RUNNING instead of FAILURE if the check fails.")
  };
}

BT::NodeStatus CheckTimeFrame::onStart()
{
  std::string start_time_str = getInput<std::string>("start_time").value_or("");
  std::string end_time_str = getInput<std::string>("end_time").value_or("");
  bool wait = getInput<bool>("wait").value();

  if (check_time_frame(start_time_str, end_time_str)) {
    return BT::NodeStatus::SUCCESS;
  } else {
    return wait ? BT::NodeStatus::RUNNING : BT::NodeStatus::FAILURE;
  }
}

BT::NodeStatus CheckTimeFrame::onRunning()
{
  // The code is copied from onStart() because we have to do the exact same thing.
  // It is left explicitly like this, and not wrapped in a separate method,
  // to be explanatory and to allow easy future modifications should we decide to change
  // this behavior.

  std::string start_time_str = getInput<std::string>("start_time").value_or("");
  std::string end_time_str = getInput<std::string>("end_time").value_or("");
  bool wait = getInput<bool>("wait").value();

  if (check_time_frame(start_time_str, end_time_str)) {
    return BT::NodeStatus::SUCCESS;
  } else {
    return wait ? BT::NodeStatus::RUNNING : BT::NodeStatus::FAILURE;
  }
}

void CheckTimeFrame::onHalted()
{}

bool CheckTimeFrame::check_time_frame(
  const std::string & start_time_str,
  const std::string & end_time_str)
{
  // Validate format: exactly HH:MM:SS with valid ranges
  static const std::regex time_re(R"(^([01]\d|2[0-3]):([0-5]\d):([0-5]\d)$)");

  std::smatch start_match, end_match;
  if (!std::regex_match(start_time_str, start_match, time_re)) {
    std::string err_msg =
      "Invalid start time format: '" + start_time_str + "' (expected HH:MM:SS, 24hr)";
    RCLCPP_FATAL(ros2_node_->get_logger(), "%s", err_msg.c_str());
    throw std::invalid_argument(err_msg);
  }
  if (!std::regex_match(end_time_str, end_match, time_re)) {
    std::string err_msg =
      "Invalid end time format: '" + end_time_str + "' (expected HH:MM:SS, 24hr)";
    RCLCPP_FATAL(ros2_node_->get_logger(), "%s", err_msg.c_str());
    throw std::invalid_argument(err_msg);
  }

  // Parse start and end times into total seconds since midnight
  long int start_secs = std::stoi(start_match[1].str()) * 3600 +
    std::stoi(start_match[2].str()) * 60 +
    std::stoi(start_match[3].str());
  long int end_secs = std::stoi(end_match[1].str()) * 3600 +
    std::stoi(end_match[2].str()) * 60 +
    std::stoi(end_match[3].str());

  // Get current local time from the system, in seconds since (last) midnight
  struct timespec now;
  int res = clock_gettime(CLOCK_REALTIME, &now);
  if (res != 0) {
    char err_buf[256];
    char * err_msg = strerror_r(errno, err_buf, sizeof(err_buf));
    RCLCPP_ERROR_THROTTLE(
      ros2_node_->get_logger(), *ros2_node_->get_clock(), 1000,
      "clock_gettime failed: %s", err_msg);
    throw std::runtime_error("clock_gettime failed: " + std::string(err_msg));
  }
  struct tm local_tm;
  if (localtime_r(&now.tv_sec, &local_tm) == NULL) {
    char err_buf[256];
    char * err_msg = strerror_r(errno, err_buf, sizeof(err_buf));
    RCLCPP_ERROR_THROTTLE(
      ros2_node_->get_logger(), *ros2_node_->get_clock(), 1000,
      "localtime_r failed: %s", err_msg);
    throw std::runtime_error("localtime_r failed: " + std::string(err_msg));
  }
  long int now_secs = local_tm.tm_hour * 3600L + local_tm.tm_min * 60L + local_tm.tm_sec;

  // Check whether current time falls within the time frame
  // If start <= end the frame is within the same day
  // If start > end the frame spans midnight
  if (start_secs <= end_secs) {
    return now_secs >= start_secs && now_secs <= end_secs;
  } else {
    return now_secs >= start_secs || now_secs <= end_secs;
  }
}

} // namespace dua_btcpp_nodes
