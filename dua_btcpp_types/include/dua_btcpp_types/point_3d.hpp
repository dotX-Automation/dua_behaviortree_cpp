/**
 * Representation of a point in 3D space, with a heading information.
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

#pragma once

#include <cmath>
#include <string>

#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/json_export.h>

/**
 * INCLUDE THIS FILE ONLY ONCE IN A SINGLE COMPILATION UNIT!
 */

namespace dua_btcpp_types
{

/**
 * Useful to represent a target position and heading in 3D space.
 */
struct Point3D
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;

  double q_x = 0.0;
  double q_y = 0.0;
  double q_z = 0.0;
  double q_w = 0.0;

  std::string frame_id = "map";
};

BT_JSON_CONVERTER(Point3D, p)
{
  add_field("x", &p.x);
  add_field("y", &p.y);
  add_field("z", &p.z);

  add_field("q_x", &p.q_x);
  add_field("q_y", &p.q_y);
  add_field("q_z", &p.q_z);
  add_field("q_w", &p.q_w);

  add_field("frame_id", &p.frame_id);
}

} // namespace dua_btcpp_types

namespace BT
{

/**
 * @brief Converts a string representation of Point3D into its structured version.
 *
 * @param str The string representation to convert.
 * @return The converted structured data.
 */
template<>
inline dua_btcpp_types::Point3D convertFromString(StringView str)
{
  dua_btcpp_types::Point3D p;

  /**
   * We have multiple possibilities here.
   *
   * 1. Three arguments: no specific heading is requested, frame_id is "map".
   * 2. Four arguments: a specific heading is requested, frame_id is "map".
   * 3. Five arguments: specific heading and frame_id are requested.
   */
  auto parts = splitString(str, ';');
  if (parts.size() < 3) {
    throw RuntimeError("Invalid Point3D representation: too few arguments");
  }
  p.x = convertFromString<double>(parts[0]);
  p.y = convertFromString<double>(parts[1]);
  p.z = convertFromString<double>(parts[2]);
  if (parts.size() > 3) {
    double heading = convertFromString<double>(parts[3]);
    if (heading < -180.0 || heading > 180.0) {
      throw RuntimeError("Invalid Point3D representation: heading must be in [-180, +180]");
    }

    p.q_z = std::sin(heading * M_PI / 180.0 / 2.0);
    p.q_w = std::cos(heading * M_PI / 180.0 / 2.0);
  }
  if (parts.size() > 4) {
    p.frame_id = std::string(parts[4]);
  }

  return p;
}

} // namespace BT
