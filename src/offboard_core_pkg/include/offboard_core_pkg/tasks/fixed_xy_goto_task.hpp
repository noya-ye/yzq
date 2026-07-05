#pragma once

#include <cmath>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "offboard_core_pkg/itask.hpp"
#include "offboard_core_pkg/context.hpp"

namespace offboard_core_pkg
{

class FixedXYGotoTask : public ITask
{
public:
  FixedXYGotoTask(
    rclcpp::Logger logger,
    float target_x,
    float target_y,
    double xy_tol,
    double z_tol,
    double v_xy_tol,
    int stable_required)
  : logger_(logger),
    target_x_(target_x),
    target_y_(target_y),
    xy_tol_(xy_tol),
    z_tol_(z_tol),
    v_xy_tol_(v_xy_tol),
    stable_required_(stable_required)
  {
  }

  std::string name() const override
  {
    return "FIXED_XY_GOTO";
  }

  void onEnter(Context& ctx) override
  {
    stable_count_ = 0;
    print_count_ = 0;

    target_z_ = ctx.takeoff_z;

    ctx.vision_front_enable = false;
    ctx.vision_down_enable = false;
    ctx.vision_enable = false;
    ctx.vision_searching = false;
    ctx.vision_target_locked = false;
    ctx.vision_aligned = false;

    ctx.use_vel_ctrl = false;

    ctx.sp_x = target_x_;
    ctx.sp_y = target_y_;
    ctx.sp_z = target_z_;
    ctx.sp_yaw = ctx.home_yaw;

    ctx.sp_vx = 0.0f;
    ctx.sp_vy = 0.0f;
    ctx.sp_vz = 0.0f;

    RCLCPP_WARN(
      logger_,
      "[FixedXYGotoTask] enter target=(%.2f, %.2f, %.2f)",
      target_x_,
      target_y_,
      target_z_);
  }

  Status tick(Context& ctx, double /*dt*/) override
  {
    ctx.vision_front_enable = false;
    ctx.vision_down_enable = false;
    ctx.vision_enable = false;
    ctx.vision_searching = false;
    ctx.vision_target_locked = false;
    ctx.vision_aligned = false;

    ctx.use_vel_ctrl = false;

    ctx.sp_x = target_x_;
    ctx.sp_y = target_y_;
    ctx.sp_z = target_z_;
    ctx.sp_yaw = ctx.home_yaw;

    ctx.sp_vx = 0.0f;
    ctx.sp_vy = 0.0f;
    ctx.sp_vz = 0.0f;

    const float dx = ctx.cx() - target_x_;
    const float dy = ctx.cy() - target_y_;
    const float dz = ctx.local_pos.z - target_z_;

    const float err_xy = std::sqrt(dx * dx + dy * dy);
    const float err_z = std::fabs(dz);

    const float v_xy = std::sqrt(
      ctx.local_pos.vx * ctx.local_pos.vx +
      ctx.local_pos.vy * ctx.local_pos.vy);

    if (
      err_xy < static_cast<float>(xy_tol_) &&
      err_z < static_cast<float>(z_tol_) &&
      v_xy < static_cast<float>(v_xy_tol_))
    {
      stable_count_++;
    } else {
      stable_count_ = 0;
    }

    print_count_++;
    if (print_count_ >= 10) {
      print_count_ = 0;

      // RCLCPP_INFO(
      //   logger_,
      //   "[FixedXYGotoTask] target=(%.2f %.2f %.2f) pos=(%.2f %.2f %.2f) err_xy=%.3f err_z=%.3f v_xy=%.3f stable=%d/%d",
      //   target_x_,
      //   target_y_,
      //   target_z_,
      //   ctx.cx(),
      //   ctx.cy(),
      //   ctx.local_pos.z,
      //   err_xy,
      //   err_z,
      //   v_xy,
      //   stable_count_,
      //   stable_required_);
    }

    if (stable_count_ >= stable_required_) {
      RCLCPP_WARN(
        logger_,
        "[FixedXYGotoTask] reached target=(%.2f, %.2f), pos=(%.2f, %.2f)",
        target_x_,
        target_y_,
        ctx.cx(),
        ctx.cy());

      return Status::SUCCESS;
    }

    return Status::RUNNING;
  }

private:
  rclcpp::Logger logger_;

  float target_x_{0.0f};
  float target_y_{0.0f};
  float target_z_{0.0f};

  double xy_tol_{0.15};
  double z_tol_{0.15};
  double v_xy_tol_{0.15};
  int stable_required_{10};

  int stable_count_{0};
  int print_count_{0};
};

}  // namespace offboard_core_pkg