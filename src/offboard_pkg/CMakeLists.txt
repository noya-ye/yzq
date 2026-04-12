cmake_minimum_required(VERSION 3.8)
project(offboard_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)

add_executable(offboard_1m_node src/offboard_1m.cpp)

ament_target_dependencies(offboard_1m_node
  rclcpp
  px4_msgs
)

install(TARGETS
  offboard_1m_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
