cmake_minimum_required(VERSION 3.8)
project(fastlio_to_px4)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(px4_msgs REQUIRED)

include_directories(
  include
)

add_executable(fastlio_to_px4_node
  src/fastlio_to_px4_node.cpp
)

ament_target_dependencies(fastlio_to_px4_node
  rclcpp
  nav_msgs
  geometry_msgs
  px4_msgs
)

install(TARGETS
  fastlio_to_px4_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_package()