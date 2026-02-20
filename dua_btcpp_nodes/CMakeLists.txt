cmake_minimum_required(VERSION 3.10)
project(dua_btcpp_nodes)

set(CMAKE_BUILD_TYPE "RelWithDebInfo")

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -fvisibility=hidden)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(dua_aircraft_interfaces REQUIRED)
find_package(dua_btcpp_base REQUIRED)
find_package(dua_btcpp_types REQUIRED)
find_package(dua_common_interfaces REQUIRED)
find_package(dua_hardware_interfaces REQUIRED)
find_package(dua_movement_interfaces REQUIRED)
find_package(dua_node_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# Build library
file(GLOB DUA_BTCPP_NODES_SRCS
  RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}"
  "src/*.cpp")
add_library(dua_btcpp_nodes SHARED ${DUA_BTCPP_NODES_SRCS})
add_library(dua_btcpp_nodes::dua_btcpp_nodes ALIAS dua_btcpp_nodes)
target_compile_features(dua_btcpp_nodes PUBLIC c_std_99 cxx_std_17) # Require C99 and C++17
target_compile_definitions(dua_btcpp_nodes PRIVATE "DUA_BTCPP_NODES_BUILDING_LIBRARY")
target_include_directories(dua_btcpp_nodes PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
ament_target_dependencies(dua_btcpp_nodes
  behaviortree_cpp
  dua_aircraft_interfaces
  dua_btcpp_base
  dua_btcpp_types
  dua_common_interfaces
  dua_hardware_interfaces
  dua_movement_interfaces
  dua_node_cpp
  geometry_msgs
  pluginlib
  rclcpp
  rclcpp_action
  std_msgs
  std_srvs
)

# Install library and headers
install(
  DIRECTORY include/
  DESTINATION include)
install(
  TARGETS dua_btcpp_nodes
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# Export plugin
pluginlib_export_plugin_description_file(dua_btcpp_base plugins.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Export all dependencies, library targets, and headers of this package
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_libraries(dua_btcpp_nodes)
ament_export_include_directories(include)
ament_export_dependencies(
  behaviortree_cpp
  dua_aircraft_interfaces
  dua_btcpp_base
  dua_btcpp_types
  dua_common_interfaces
  dua_hardware_interfaces
  dua_movement_interfaces
  dua_node_cpp
  geometry_msgs
  pluginlib
  rclcpp
  rclcpp_action
  std_msgs
  std_srvs
)

ament_package()
