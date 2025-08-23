cmake_minimum_required(VERSION 3.10)
project(dua_behaviortree_cpp)

find_package(ament_cmake REQUIRED)
find_package(dua_btcpp_base REQUIRED)
find_package(dua_btcpp_nodes REQUIRED)
find_package(btcpp_executor REQUIRED)

ament_export_dependencies(
  dua_btcpp_base
  dua_btcpp_nodes
  btcpp_executor
)

ament_package()
