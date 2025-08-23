cmake_minimum_required(VERSION 3.10)
project(btcpp_executor)

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
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THREADS_PREFER_PTHREAD_FLAG ON)

# Find dependencies
find_package(behaviortree_cpp REQUIRED)
find_package(dua_app_management REQUIRED)
find_package(dua_btcpp_base REQUIRED)
find_package(dua_node_cpp REQUIRED)
find_package(dua_qos_cpp REQUIRED)
find_package(params_manager_cpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Threads REQUIRED)

# Generate parameters source code
generate_init_parameters(
  YAML_FILE "${CMAKE_CURRENT_SOURCE_DIR}/src/btcpp_executor/params.yaml"
  OUT_FILE "init_parameters.cpp"
)

# Configure source files
file(GLOB BTCPP_EXECUTOR_SRCS
  RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
  "src/btcpp_executor/*.cpp")
list(APPEND BTCPP_EXECUTOR_SRCS "init_parameters.cpp")

# BTCPP Executor library
add_library(btcpp_executor SHARED ${BTCPP_EXECUTOR_SRCS})
target_include_directories(btcpp_executor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(btcpp_executor PUBLIC c_std_99 cxx_std_17) # Required C99 and C++17
target_link_libraries(btcpp_executor
  Threads::Threads
)
ament_target_dependencies(btcpp_executor
  behaviortree_cpp
  dua_app_management
  dua_btcpp_base
  dua_node_cpp
  dua_qos_cpp
  params_manager_cpp
  pluginlib
  rclcpp
  rclcpp_components
  std_srvs
)
rclcpp_components_register_nodes(btcpp_executor "btcpp_executor::BTExecutor")

# BTCPP Executor standalone application
add_executable(btcpp_executor_app
  src/btcpp_executor_app.cpp)
target_include_directories(btcpp_executor_app PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(btcpp_executor_app btcpp_executor)
ament_target_dependencies(btcpp_executor_app
  dua_app_management
  rclcpp
)

# Install BTCPP Executor component
install(TARGETS btcpp_executor
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# Install BTCPP Executor app
install(TARGETS btcpp_executor_app
  DESTINATION lib/${PROJECT_NAME})

# Install Launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

# Install Config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
