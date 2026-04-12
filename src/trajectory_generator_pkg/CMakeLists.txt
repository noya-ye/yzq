cmake_minimum_required(VERSION 3.8)
project(trajectory_generator_pkg)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(glog REQUIRED)

include_directories(
  include
  third_party/mav_trajectory_generation/include
  third_party/mav_trajectory_generation/src   # ⭐ 加这个
)

set(MAV_TRAJ_SRC
  third_party/mav_trajectory_generation/src/polynomial.cpp
  third_party/mav_trajectory_generation/src/segment.cpp
  third_party/mav_trajectory_generation/src/vertex.cpp
  third_party/mav_trajectory_generation/src/trajectory.cpp
  third_party/mav_trajectory_generation/src/timing.cpp
  third_party/mav_trajectory_generation/src/motion_defines.cpp
  third_party/mav_trajectory_generation/src/rpoly/rpoly_ak1.cpp   # ⭐ 必加
)

add_executable(trajectory_node
  src/trajectory_node.cpp
  src/generator.cpp
  ${MAV_TRAJ_SRC}
)

ament_target_dependencies(trajectory_node
  rclcpp
  px4_msgs
)


target_link_libraries(trajectory_node
  Eigen3::Eigen
  glog::glog   # ⭐ 必须加
)

install(TARGETS trajectory_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()