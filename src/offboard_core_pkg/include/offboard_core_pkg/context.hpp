#pragma once

#include <cstdint>
#include <vector>
#include <limits>
#include <cmath>

#include "px4_msgs/msg/vehicle_status.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "px4_msgs/msg/vehicle_attitude.hpp"
#include "px4_msgs/msg/vehicle_land_detected.hpp"

// ==========================
// 视觉类型
// ==========================
enum class BlockType
{
  NONE,
  YELLOW_HEX,
  BLUE_RECT,
  RED_CIRCLE
};

// ==========================
// 前视 D435 稳定采样状态
// ==========================
// IDLE:
//   正常 yaw 扫描阶段。
//   此时前视视觉只用于“发现目标触发停转”，不直接 push 坐标。
//
// WAIT_STABLE:
//   已发现目标，YawSpinTask 停止旋转。
//   此时等待无人机速度稳定，前视视觉一般关闭。
//
// SAMPLING:
//   无人机已稳定。
//   此时重新打开前视视觉，收集一小段时间的坐标，最后平均后只 push 一次。
enum class FrontVisionState
{
  IDLE,
  WAIT_STABLE,
  SAMPLING
};

// ==========================
// 视觉信息
// ==========================
struct VisionInfo
{
  BlockType type{BlockType::NONE};
  float score{0.0f};
  uint64_t stamp_us{0};
};

struct VisionOffset    //视觉对准
{
  float dx{0.0f};      // 图像中心偏差，右为正
  float dy{0.0f};      // 图像中心偏差，下为正
  float score{0.0f};
  int type{0};         // 0: 无效
  float cost{0.0f};    // 排序代价，越小越优先
  uint64_t stamp_us{0};
};

struct VisionNumber    //识别数字
{
  int number{-1};
  float score{0.0f};
  uint64_t stamp_us{0};

  bool valid() const
  {
    return number >= 0 && number <= 13;
  }
};

struct VisionPosition  //深度相机识别坐标
{
  float x{0.0f};
  float y{0.0f};
  float z{0.0f};

  bool valid() const
  {
    return std::isfinite(x) &&
           std::isfinite(y) &&
           std::isfinite(z);
  }
};

// ==========================
// 航向信息
// ==========================
struct YawInfo
{
  float error_rad{0.0f};
  float quality{0.0f};
  uint64_t stamp_us{0};
};

// ==========================
// Context（核心）
// ==========================
struct Context
{
  // ============================================================
  // PX4 输入
  // ============================================================
  px4_msgs::msg::VehicleStatus         vehicle_status{};
  px4_msgs::msg::VehicleLocalPosition  local_pos{};
  px4_msgs::msg::VehicleAttitude       vehicle_att{};
  px4_msgs::msg::VehicleLandDetected   land_detected{};

  bool has_attitude{false};

  float yaw{0.0f};
  float yaw_rate{0.0f};

  // ============================================================
  // Home
  // ============================================================
  bool  home_inited{false};
  float home_x{0.0f};
  float home_y{0.0f};
  float home_z{0.0f};

  bool  home_yaw_inited{false};
  float home_yaw{0.0f};

  float takeoff_z{0.0f};
  float land_z{0.0f};

  // ============================================================
  // 前视 D435 粗定位目标
  // ============================================================
  // 最终确认后的目标点。
  // 注意：不要在视觉回调里一看到目标就 push。
  // 正确流程：
  // 发现目标 -> 停 yaw -> 等稳定 -> 采样 -> 平均 -> push 一次。
  std::vector<VisionPosition> detected_targets;

  // 前视稳定采样状态机
  FrontVisionState front_vision_state{FrontVisionState::IDLE};

  // 前视采样缓存。
  // SAMPLING 阶段把短时间内的 D435 坐标放进这里，
  // 采样结束后求平均，再 push 到 detected_targets。
  std::vector<VisionPosition> front_sample_buffer;

  // 前视采样参数
  float front_stable_vxy_thresh{0.08f};      // xy 速度小于此值认为横向稳定
  float front_stable_vz_thresh{0.08f};       // z 速度小于此值认为高度稳定
  float front_duplicate_xy_m{1.20f};         // 前视粗定位去重半径
  int   max_front_targets{5};                // 最大前视目标数量，防止炸目标

  // 前视状态时间戳，单位 us。
  // 具体赋值可以在节点里用 now().nanoseconds()/1000。
  uint64_t front_state_stamp_us{0};

  // 前视采样是否已经推入过一个平均目标。
  // 防止同一次稳定窗口重复 push。
  bool front_sample_pushed{false};

  // ============================================================
  // 视觉门控
  // ============================================================
  bool vision_enable{false};
  bool vision_front_enable{false};
  bool vision_down_enable{false};

  // ============================================================
  // 视觉触发
  // ============================================================
  // vision_detected:
  //   前视看到目标触发 yaw 停止。
  //
  // vision_done:
  //   稳定采样完成，通知 YawSpinTask 恢复旋转。
  bool vision_detected{false};
  bool vision_done{false};

  // ============================================================
  // yaw 累计
  // ============================================================
  float yaw_accumulated{0.0f};
  float last_yaw{0.0f};

  // ============================================================
  // 网格
  // ============================================================
  int rows{5};
  int cols{5};
  double cell_size{0.8};
  double origin_dx{0.8};
  double origin_dy{0.8};

  int current_r{-1};
  int current_c{-1};

  // ============================================================
  // 下视视觉 / 数字 / 航向
  // ============================================================
  VisionInfo vision;
  VisionNumber last_number;
  YawInfo vision_yaw;
  VisionOffset vision_offset;

  // 当前帧下视所有目标，实时刷新
  std::vector<VisionOffset> vision_down_targets;
  uint64_t vision_down_targets_stamp_us{0};

  // ============================================================
  // 下视补盲队列 / 锁定
  // ============================================================
  std::vector<VisionOffset> blindcheck_queue;   // 按 cost 从小到大排序的待扫描队列
  int blindcheck_index{0};                      // 当前扫描到第几个
  bool blindcheck_locked{false};                // 当前是否锁定目标
  VisionOffset blindcheck_locked_target;        // 当前锁定目标

  float blindcheck_origin_x{0.0f};              // 扫描原点
  float blindcheck_origin_y{0.0f};

  // ============================================================
  // 时间戳
  // ============================================================
  uint64_t vision_last_update_us{0};

  // ============================================================
  // 状态
  // ============================================================
  bool vision_aligned{false};

  // ============================================================
  // 防抖 / 搜索
  // ============================================================
  bool vision_target_locked{false};
  int  vision_lost_count{0};
  bool vision_searching{false};

  // ============================================================
  // 数字地图
  // ============================================================
  std::vector<int>      grid_numbers;
  std::vector<float>    grid_num_best_score;
  std::vector<uint64_t> grid_num_stamp_us;

  std::vector<int>      grid_num_candidate;
  std::vector<float>    grid_num_candidate_score;

  // ============================================================
  // 形状标记
  // ============================================================
  std::vector<uint64_t> grid_shape_stamp_us;

  // ============================================================
  // 任务状态
  // ============================================================
  bool done_yellow_hex{false};
  bool done_blue_rect{false};
  bool seen_red_circle{false};


  //请求中断的信号
  bool front_pre_align_request{false};//前视矫正
  bool down_align_request{false};//下视矫正
  bool obstacle_request{false};//避障请求
  // ===== MCU serial switch request =====
  // true  表示请求主程序打开单片机开关
  // false 表示请求主程序关闭单片机开关
  bool mcu_switch_request{false};

  // ============================================================
  // 控制输出
  // ============================================================
  bool use_vel_ctrl{false};

  float sp_x{0.0f};
  float sp_y{0.0f};
  float sp_z{0.0f};
  float sp_yaw{0.0f};

  float sp_vx{0.0f};
  float sp_vy{0.0f};
  float sp_vz{0.0f};

  float sp_ax{0.0f};
  float sp_ay{0.0f};
  float sp_az{0.0f};
 // ============================================================
// EGO-Planner /position_cmd 输入
// 来自 quadrotor_msgs/msg/PositionCommand
// ============================================================
bool ego_cmd_valid{false};
uint64_t ego_cmd_stamp_us{0};

double ego_x{0.0};
double ego_y{0.0};
double ego_z{0.0};

double ego_vx{0.0};
double ego_vy{0.0};
double ego_vz{0.0};

double ego_ax{0.0};
double ego_ay{0.0};
double ego_az{0.0};

double ego_yaw{0.0};
double ego_yaw_dot{0.0};

uint32_t ego_traj_id{0};
uint8_t ego_traj_flag{0};


// ============================================================
// FAST-LIO / EGO Odometry 输入
// 用于计算 EGO 世界系下的相对误差：
//   d_ego = /position_cmd - /Odometry
//
// 注意：不再使用 ego_origin / px4_origin 对齐方法，
// 避免第一帧 /position_cmd 被误当成原点导致飞机乱飞。
// ============================================================
bool ego_odom_valid{false};
uint64_t ego_odom_stamp_us{0};

double ego_odom_x{0.0};
double ego_odom_y{0.0};
double ego_odom_z{0.0};

double ego_odom_vx{0.0};
double ego_odom_vy{0.0};
double ego_odom_vz{0.0};
  // ============================================================
  // LAND 状态
  // ============================================================
  bool handover_to_px4_land{false};

  // ============================================================
  // 工具函数
  // ============================================================
  int idx(int r, int c) const
  {
    return r * cols + c;
  }

  float cx() const
  {
    return local_pos.x;
  }

  float cy() const
  {
    return local_pos.y;
  }

  float cz() const
  {
    return local_pos.z;
  }

  bool pos_valid() const
  {
    return local_pos.xy_valid && local_pos.z_valid;
  }

  bool is_landed() const
  {
    return land_detected.landed;
  }

  bool is_low_speed() const
  {
    return std::fabs(local_pos.vz) < 0.1f;
  }

  bool front_pose_stable() const
  {
    const float vxy = std::sqrt(
      local_pos.vx * local_pos.vx +
      local_pos.vy * local_pos.vy);

    const float vz = std::fabs(local_pos.vz);

    return vxy < front_stable_vxy_thresh &&
           vz  < front_stable_vz_thresh;
  }

  void reset_front_vision_sampling()
  {
    front_vision_state = FrontVisionState::IDLE;
    front_sample_buffer.clear();
    front_state_stamp_us = 0;
    front_sample_pushed = false;

    vision_detected = false;
    vision_done = false;

    vision_front_enable = false;
  }

  bool is_front_duplicate(float x, float y) const
  {
    for (const auto& p : detected_targets) {
      const float dx = p.x - x;
      const float dy = p.y - y;
      const float d = std::sqrt(dx * dx + dy * dy);

      if (d < front_duplicate_xy_m) {
        return true;
      }
    }

    return false;
  }
};
