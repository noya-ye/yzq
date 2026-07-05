#pragma once

#include "offboard_core_pkg/itask.hpp"
#include "offboard_core_pkg/context.hpp"

#include <rclcpp/rclcpp.hpp>

#include <string>
#include <vector>

namespace offboard_core_pkg
{

class StartDownBlindCheckTask : public ITask
{
public:
  StartDownBlindCheckTask(
    rclcpp::Logger logger,
    double timeout_s,
    double align_tol_m,
    int stable_required,
    double k_img_to_meter,
    double max_step_m,
    double score_thresh,
    double dup_remove_radius,
    double img_to_body_x_sign,
    double img_to_body_y_sign);

  std::string name() const override;
  void onEnter(Context& ctx) override;
  Status tick(Context& ctx, double dt) override;

private:
  enum class BlindCheckState
  {
    WAIT_FRAME,
    PICK_NEXT,
    ALIGN_TARGET,
    RETURN_ORIGIN,
    FINISH
  };

  // ===== 主流程子状态 =====
  Status tick_wait_frame(Context& ctx);
  Status tick_pick_next(Context& ctx);
  Status tick_align_target(Context& ctx, double dt);
  Status tick_return_origin(Context& ctx);

  // ===== 工具函数 =====
  bool offset_fresh(const Context& ctx) const;
  bool down_targets_fresh(const Context& ctx) const;

  bool find_locked_target_in_frame(
    const Context& ctx,
    VisionOffset& matched) const;

  void finish_task(Context& ctx, bool aligned);

  void image_offset_to_world_delta(
    const Context& ctx,
    float dx_img,
    float dy_img,
    float& dwx,
    float& dwy) const;

  void remove_nearby_rough_targets(
    Context& ctx,
    float x,
    float y);

private:
  rclcpp::Logger logger_;

  // ===== 参数 =====
  double timeout_s_{3.0};
  double align_tol_m_{0.08};
  int stable_required_{8};
  double k_img_to_meter_{0.35};
  double max_step_m_{0.12};
  double score_thresh_{500.0};
  double dup_remove_radius_{0.5};

  // 单个目标最大纠偏时间
  // 超过后直接认为当前目标无法继续纠偏，跳到下一个目标
  double align_timeout_s_{20.0};

  // 图像偏差 -> 机体系方向符号
  // dx_img: 图像右为正
  // dy_img: 图像下为正
  double img_to_body_x_sign_{-1.0};
  double img_to_body_y_sign_{1.0};

  // 锁定目标丢失后的跳过时间
  double align_lost_timeout_s_{2.5};

  // ===== 运行时状态 =====
  BlindCheckState state_{BlindCheckState::WAIT_FRAME};

  double elapsed_{0.0};
  double print_elapsed_{0.0};

  // 当前目标连续丢失时间
  double align_lost_elapsed_{0.0};

  // 当前目标总纠偏时间
  double align_elapsed_{0.0};

  int stable_count_{0};
  bool ever_seen_{false};

  float hold_z_{0.0f};
};

}  // namespace offboard_core_pkg