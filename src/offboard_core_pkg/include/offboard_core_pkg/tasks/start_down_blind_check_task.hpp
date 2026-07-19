#pragma once

#include "offboard_core_pkg/itask.hpp"
#include "offboard_core_pkg/context.hpp"

#include <rclcpp/rclcpp.hpp>

#include <cstdint>
#include <string>

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

  Status tick_wait_frame(Context& ctx);
  Status tick_pick_next(Context& ctx);
  Status tick_align_target(Context& ctx, double dt);
  Status tick_return_origin(Context& ctx);

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

  void remove_nearby_rough_targets(Context& ctx, float x, float y);

private:
  rclcpp::Logger logger_;

  double timeout_s_{3.0};
  double align_tol_m_{0.08};
  int stable_required_{8};
  double k_img_to_meter_{0.35};
  double max_step_m_{0.12};
  double score_thresh_{500.0};
  double dup_remove_radius_{0.5};

  double align_timeout_s_{15.0};
  double align_lost_timeout_s_{2.0};

  double img_to_body_x_sign_{-1.0};
  double img_to_body_y_sign_{1.0};

  BlindCheckState state_{BlindCheckState::WAIT_FRAME};

  double elapsed_{0.0};
  double print_elapsed_{0.0};
  double align_lost_elapsed_{0.0};
  double align_elapsed_{0.0};
  double down_frame_age_s_{1e9};

  int stable_count_{0};
  bool ever_seen_{false};
  float hold_z_{0.0f};

  uint64_t last_down_frame_stamp_us_{0};
  uint64_t last_control_frame_stamp_us_{0};

  float align_cmd_x_{0.0f};
  float align_cmd_y_{0.0f};

  bool mcu_a_sent_{false};
  bool mcu_b_sent_{false};
};

}  // namespace offboard_core_pkg