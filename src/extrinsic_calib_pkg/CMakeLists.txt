cmake_minimum_required(VERSION 3.8)
project(extrinsic_calib_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
)

add_executable(extrinsic_calib_node
  src/extrinsic_calib_node.cpp
)

ament_target_dependencies(extrinsic_calib_node
  rclcpp
  geometry_msgs
  nav_msgs
)

target_link_libraries(extrinsic_calib_node
  Eigen3::Eigen
)

install(TARGETS
  extrinsic_calib_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_package()