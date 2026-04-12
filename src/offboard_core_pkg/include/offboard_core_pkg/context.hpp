#pragma once

#include <cstdint>
#include <vector>
#include <limits>
#include <cmath>

#include "px4_msgs/msg/vehicle_status.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "px4_msgs/msg/vehicle_attitude.hpp"
#include "px4_msgs/msg/vehicle_land_detected.hpp"  // ✅ 必加

// ==========================
// 视觉类型
// ==========================
enum class BlockType { NONE, YELLOW_HEX, BLUE_RECT, RED_CIRCLE };

// ==========================
// 视觉信息
// ==========================
struct VisionInfo {
  BlockType type{BlockType::NONE};
  float score{0.0f};
  uint64_t stamp_us{0};
};

struct VisionNumber {
  int number{-1};
  float score{0.f};
  uint64_t stamp_us{0};

  bool valid() const { return number >= 0 && number <= 13; }
};

struct VisionPosition {
  float x;
  float y;
  float z;
};

// ==========================
// 航向信息
// ==========================
struct YawInfo {
  float error_rad{0.0f};
  float quality{0.0f};
  uint64_t stamp_us{0};
};

// ==========================
// Context（核心）
// ==========================
struct Context {

  // ===== PX4 输入 =====
  px4_msgs::msg::VehicleStatus         vehicle_status{};
  px4_msgs::msg::VehicleLocalPosition  local_pos{};
  px4_msgs::msg::VehicleAttitude       vehicle_att{};
  px4_msgs::msg::VehicleLandDetected   land_detected{};  // ✅ NEW

  bool has_attitude{false};

  // ===== Home =====
  bool  home_inited{false};
  float home_x{0}, home_y{0}, home_z{0};

  bool  home_yaw_inited{false};
  float home_yaw{0};

  float takeoff_z{0};
  float land_z{0};

  // ===== 视觉坐标 =====
  std::vector<VisionPosition> detected_targets;

  // ===== 视觉门控 =====
  bool vision_enable{false};

  // ===== 网格 =====
  int rows{5}, cols{5};
  double cell_size{0.8};
  double origin_dx{0.8};
  double origin_dy{0.8};

  int current_r{-1}, current_c{-1};

  // ===== 视觉 =====
  VisionInfo vision;
  VisionNumber last_number;
  YawInfo yaw;

  // ===== 数字地图 =====
  std::vector<int>      grid_numbers;
  std::vector<float>    grid_num_best_score;
  std::vector<uint64_t> grid_num_stamp_us;

  std::vector<int>      grid_num_candidate;
  std::vector<float>    grid_num_candidate_score;

  // ===== 形状标记 =====
  std::vector<uint64_t> grid_shape_stamp_us;

  // ===== 任务状态 =====
  bool done_yellow_hex{false};
  bool done_blue_rect{false};
  bool seen_red_circle{false};

  // ===== 控制输出 =====
  bool use_vel_ctrl{false};

  float sp_x{0}, sp_y{0}, sp_z{0}, sp_yaw{0};
  float sp_vx{0}, sp_vy{0}, sp_vz{0};

  // ===== 工具函数 =====
  int idx(int r, int c) const { return r * cols + c; }

  float cx() const { return local_pos.x; }
  float cy() const { return local_pos.y; }
  float cz() const { return local_pos.z; }

  bool pos_valid() const { return local_pos.xy_valid && local_pos.z_valid; }

  // ===== LAND 状态 =====
  bool handover_to_px4_land{false};

  // ✅ 新增：是否已接地
  bool is_landed() const {
    return land_detected.landed;
  }

  // ✅ 新增：是否稳定（低速）
  bool is_low_speed() const {
    return std::fabs(local_pos.vz) < 0.1f;
  }
};