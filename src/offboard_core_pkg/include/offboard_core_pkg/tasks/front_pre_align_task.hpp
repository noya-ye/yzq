#pragma once

#include "offboard_core_pkg/itask.hpp"
#include "offboard_core_pkg/context.hpp"

#include <rclcpp/rclcpp.hpp>

#include <cstdint>
#include <string>

namespace offboard_core_pkg
{

/*
 * D题货架前置视觉矫正任务
 *
 * 使用 Context 中已有的 VisionOffset：
 *
 *   dx > 0：目标位于画面右侧
 *   dy > 0：目标位于画面下方
 *
 * 本任务只调整：
 *
 *   1. 机体系左右方向
 *   2. NED 高度方向
 *
 * 本任务不调整：
 *
 *   1. 与货架的前后距离
 *   2. 无人机 yaw
 *
 * 因此前一个粗定位任务必须已经让无人机：
 *
 *   1. 飞到货架面前
 *   2. 大致正对货架
 *   3. 保持合适的观察距离
 */
class FrontPreAlignTask : public ITask
{
public:
  FrontPreAlignTask(
    rclcpp::Logger logger,
    double timeout_s,
    double align_tol_m,
    int stable_required,
    double k_img_to_meter,
    double max_step_m,
    double score_thresh,
    double img_to_body_y_sign,
    double img_to_body_z_sign,
    int expected_type = -1);

  std::string name() const override;

  void onEnter(Context& ctx) override;

  Status tick(Context& ctx, double dt) override;

  void onExit(Context& ctx) override;

private:
  bool target_valid(const Context& ctx) const;

  void update_hold_setpoint(Context& ctx);

  void finish(Context& ctx, bool aligned);

private:
  rclcpp::Logger logger_;

  // 总任务超时
  double timeout_s_{8.0};

  // 图像偏差换算为米后的允许误差
  double align_tol_m_{0.04};

  // 连续满足条件的视觉帧数
  int stable_required_{12};

  // 图像偏差到实际位移的比例
  double k_img_to_meter_{0.35};

  // 每一帧允许修改的最大位置步长
  double max_step_m_{0.08};

  // 视觉目标最低得分
  double score_thresh_{500.0};

  // 图像右方向 -> 机体系右方向
  double img_to_body_y_sign_{1.0};

  // 图像下方向 -> NED z 正方向
  double img_to_body_z_sign_{1.0};

  // -1：接受所有非零 type
  // >0：只接受指定 type
  int expected_type_{-1};

  // 同一帧最多允许使用多久
  double frame_timeout_s_{0.30};

  // 相对进入任务位置的最大横向移动距离
  double max_lateral_move_m_{0.60};

  // 相对进入任务位置的最大高度修正距离
  double max_vertical_move_m_{0.50};

  // 稳定判断速度阈值
  double stable_vxy_mps_{0.12};
  double stable_vz_mps_{0.10};

  // 运行状态
  double elapsed_{0.0};
  double print_elapsed_{0.0};
  double frame_age_s_{999.0};

  int stable_count_{0};

  bool enter_failed_{false};
  bool ever_seen_{false};

  uint64_t last_frame_stamp_us_{0};

  // 进入任务时的观察位姿
  float origin_x_{0.0f};
  float origin_y_{0.0f};
  float origin_z_{0.0f};
  float hold_yaw_{0.0f};

  // 相对进入点的横向、高度指令
  float lateral_cmd_m_{0.0f};
  float vertical_cmd_m_{0.0f};
};

}  // namespace offboard_core_pkg
