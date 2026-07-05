cmake_minimum_required(VERSION 3.8)
project(ground_station_bridge_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(ground_station_msgs REQUIRED)

include_directories(
  include
)

# =========================
# Core library：原串口版核心
# =========================
add_library(ground_station_bridge_core
  src/protocol.cpp
  src/serial_port.cpp
  src/ground_station_bridge_node.cpp
)

ament_target_dependencies(ground_station_bridge_core
  rclcpp
  std_msgs
  px4_msgs
  ground_station_msgs
)

# =========================
# 原串口版节点
# =========================
add_executable(ground_station_bridge_node
  src/main.cpp
)

target_link_libraries(ground_station_bridge_node
  ground_station_bridge_core
)

ament_target_dependencies(ground_station_bridge_node
  rclcpp
  std_msgs
  px4_msgs
  ground_station_msgs
)

# =========================
# UDP 版节点
# =========================
add_executable(udp_ground_station_bridge_node
  src/udp_ground_station_bridge_main.cpp
  src/udp_ground_station_bridge_node.cpp
  src/protocol.cpp
)

ament_target_dependencies(udp_ground_station_bridge_node
  rclcpp
  std_msgs
  px4_msgs
  ground_station_msgs
)

target_include_directories(udp_ground_station_bridge_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# =========================
# Install
# =========================
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS
    ground_station_bridge_core
    ground_station_bridge_node
    udp_ground_station_bridge_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()