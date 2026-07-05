cmake_minimum_required(VERSION 3.8)
project(px4_debug_recorder)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(debug_recorder_node src/debug_recorder_node.cpp)
ament_target_dependencies(debug_recorder_node
  rclcpp
  px4_msgs
  std_msgs
)

install(TARGETS
  debug_recorder_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
