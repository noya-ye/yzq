cmake_minimum_required(VERSION 3.8)
project(offboard_core_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ========================
# dependencies
# ========================
find_package(quadrotor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(ground_station_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(glog REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(custom_vision_msgs REQUIRED)

# ========================
# include
# ========================
include_directories(
  include
  third_party/mav_trajectory_generation/include
  third_party/mav_trajectory_generation/src
  ${EIGEN3_INCLUDE_DIRS} 
)

# ========================
# core library
# ========================
add_library(offboard_core
  src/grid.cpp

  # ===== planners =====
  src/planners/min_snap_planner.cpp
  src/planners/tsp_min_snap_planner.cpp

  # ===== mav trajectory =====
  ${MAV_TRAJ_SRC}

  # ===== tasks =====
  src/tasks/arm_task.cpp
  src/tasks/disarm_task.cpp
  src/tasks/grid_goto_dwell_task.cpp
  src/tasks/home_stabilize_task.cpp
  src/tasks/hover_task.cpp
  src/tasks/px4_land_mode_task.cpp
  src/tasks/presetpoint_task.cpp
  src/tasks/return_home_task.cpp
  src/tasks/set_offboard_task.cpp
  src/tasks/takeoff_task.cpp
  src/tasks/wait_home_task.cpp
  src/tasks/yaw_spin_task.cpp
  src/tasks/snake_grid_task.cpp
  src/tasks/tsp_grid_task.cpp
  src/tasks/go_to_task.cpp
  src/tasks/start_down_blind_check_task.cpp
  src/tasks/front_pre_align_task.cpp
  src/tasks/yaw_spin2_task.cpp
  src/tasks/multi_locked_task.cpp
  src/tasks/trajectory_task.cpp
  src/tasks/ego_goto_task.cpp

  # ===== utils =====
  src/utils/mcu_serial_switch.cpp
)
ament_target_dependencies(offboard_core
  rclcpp
  std_msgs
  px4_msgs
  ground_station_msgs
  nlohmann_json
  custom_vision_msgs 
  quadrotor_msgs
  nav_msgs

)

target_include_directories(offboard_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(offboard_core PUBLIC cxx_std_17)


# ========================
# mav trajectory source
# ========================
set(MAV_TRAJ_SRC
  third_party/mav_trajectory_generation/src/polynomial.cpp
  third_party/mav_trajectory_generation/src/segment.cpp
  third_party/mav_trajectory_generation/src/trajectory.cpp
  third_party/mav_trajectory_generation/src/vertex.cpp
  third_party/mav_trajectory_generation/src/timing.cpp
  third_party/mav_trajectory_generation/src/io.cpp
  third_party/mav_trajectory_generation/src/motion_defines.cpp
  third_party/mav_trajectory_generation/src/polynomial_timing_evaluation.cpp
  third_party/mav_trajectory_generation/src/trajectory_sampling.cpp

  # ⭐最关键（否则直接炸）
  # third_party/mav_trajectory_generation/src/polynomial_optimization_linear.cpp

  # ⭐rpoly（必须）
  third_party/mav_trajectory_generation/src/rpoly/rpoly_ak1.cpp
)
target_link_libraries(offboard_core
  glog
  Eigen3::Eigen
)
# ========================
# node: d_advance
# ========================
add_executable(d_advance
  src/nodes/d_advance.cpp
)

target_link_libraries(d_advance
  offboard_core
)

ament_target_dependencies(d_advance
  rclcpp
  std_msgs
  px4_msgs
  nlohmann_json
  Eigen3  
  custom_vision_msgs 
  quadrotor_msgs
  nav_msgs
)

target_include_directories(d_advance PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(d_advance PUBLIC cxx_std_17)

# ========================
# node: offboard_test0
# ========================
add_executable(offboard_test0_node
  src/nodes/offboard_test0.cpp
)

target_link_libraries(offboard_test0_node
  offboard_core
)

ament_target_dependencies(offboard_test0_node
  rclcpp
  std_msgs
  px4_msgs
  nlohmann_json
  Eigen3  
  custom_vision_msgs 
  quadrotor_msgs
  nav_msgs
)

target_include_directories(offboard_test0_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(offboard_test0_node PUBLIC cxx_std_17)


# ========================
# node: offboard_test1
# ========================
add_executable(offboard_test1_node
  src/nodes/offboard_test1.cpp
)

target_link_libraries(offboard_test1_node
  offboard_core
)

ament_target_dependencies(offboard_test1_node
  rclcpp
  std_msgs
  px4_msgs
  nlohmann_json
  Eigen3  
  custom_vision_msgs 
)

target_include_directories(offboard_test1_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(offboard_test1_node PUBLIC cxx_std_17)


# # ========================
# # node: offboard_test2
# # ========================
# add_executable(offboard_test2_node
#   src/nodes/offboard_test2.cpp
# )

# target_link_libraries(offboard_test2_node
#   offboard_core
# )

# ament_target_dependencies(offboard_test2_node
#   rclcpp
#   std_msgs
#   px4_msgs
#   ground_station_msgs
#   nlohmann_json
#   Eigen3
# )

# target_include_directories(offboard_test2_node PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>
# )

# target_compile_features(offboard_test2_node PUBLIC cxx_std_17)

# ========================
# node: offboard_test3
# ========================
add_executable(offboard_test3_node
  src/nodes/offboard_test3.cpp
)

target_link_libraries(offboard_test3_node
  offboard_core
)

ament_target_dependencies(offboard_test3_node
  rclcpp
  std_msgs
  px4_msgs
  ground_station_msgs
  nlohmann_json
  Eigen3
  custom_vision_msgs
)

target_include_directories(offboard_test3_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(offboard_test3_node PUBLIC cxx_std_17)



# # ========================
# # node: offboard_test4
# # ========================
# add_executable(offboard_test4_node
#   src/nodes/offboard_test4.cpp
# )

# target_link_libraries(offboard_test4_node
#   offboard_core
# )

# ament_target_dependencies(offboard_test4_node
#   rclcpp
#   std_msgs
#   px4_msgs
#   nlohmann_json
#   Eigen3  
#   custom_vision_msgs 
# )

# target_include_directories(offboard_test4_node PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>
# )

# target_compile_features(offboard_test4_node PUBLIC cxx_std_17)

# # ========================
# # node: yaw_spin_test
# # ========================
# add_executable(yaw_spin_test_node
#   src/nodes/yaw_spin_test.cpp
# )

# target_link_libraries(yaw_spin_test_node
#   offboard_core
# )

# ament_target_dependencies(yaw_spin_test_node
#   rclcpp
#   std_msgs
#   px4_msgs
#   ground_station_msgs
#   nlohmann_json
#   Eigen3
#   custom_vision_msgs
# )

# target_include_directories(yaw_spin_test_node PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>
# )

# target_compile_features(yaw_spin_test_node PUBLIC cxx_std_17)

# ========================
# node: trajectory_test_node
# ========================
add_executable(trajectory_test_node
  src/nodes/trajectory_test.cpp
)

target_link_libraries(trajectory_test_node
  offboard_core
)

ament_target_dependencies(trajectory_test_node
  rclcpp
  std_msgs
  px4_msgs
  ground_station_msgs
  nlohmann_json
  Eigen3
  custom_vision_msgs
)

target_include_directories(trajectory_test_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(trajectory_test_node PUBLIC cxx_std_17)


# ========================
# node: snake_test
# ========================
add_executable(snake_test
  src/nodes/snake_test.cpp
)

target_link_libraries(snake_test
  offboard_core
)

ament_target_dependencies(snake_test
  rclcpp
  std_msgs
  px4_msgs
  ground_station_msgs
  nlohmann_json
  Eigen3
  custom_vision_msgs
)

target_include_directories(snake_test PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(snake_test PUBLIC cxx_std_17)


# # ========================
# # node: offboard_mission1
# # ========================
# add_executable(offboard_mission1_node
#   src/nodes/offboard_mission1.cpp
# )

# target_link_libraries(offboard_mission1_node
#   offboard_core
# )

# ament_target_dependencies(offboard_mission1_node
#   rclcpp
#   std_msgs
#   px4_msgs
#   nlohmann_json
# )

# target_include_directories(offboard_mission1_node PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>
# )

# target_compile_features(offboard_mission1_node PUBLIC cxx_std_17)

# ========================
# install
# ========================
install(
  TARGETS
    offboard_core
    offboard_test0_node
    offboard_test1_node
    # offboard_test2_node
    offboard_test3_node
    # offboard_test4_node
    # yaw_spin_test_node
    trajectory_test_node
    d_advance
    snake_test
    # offboard_mission1_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

# 如果你后面有 launch 文件，再打开
# install(
#   DIRECTORY launch/
#   DESTINATION share/${PROJECT_NAME}/launch
# )

ament_export_include_directories(include)
ament_export_libraries(offboard_core)

ament_package()
