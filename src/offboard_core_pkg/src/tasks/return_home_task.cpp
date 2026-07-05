#include "offboard_core_pkg/tasks/return_home_task.hpp"

#include <cmath>
#include <algorithm>

ReturnHomeTask::ReturnHomeTask(
  rclcpp::Logger lg,
  double return_step_xy,
  double return_xy_tol,
  double return_vxy_tol)
: lg_(lg),
  return_step_xy_(return_step_xy),
  return_xy_tol_(return_xy_tol),
  return_vxy_tol_(return_vxy_tol)
{}

std::string ReturnHomeTask::name() const
{
  return "RETURN_HOME";
}

void ReturnHomeTask::onEnter(Context& ctx)
{
  print_elapsed_ = 0.0;

  RCLCPP_WARN(
    lg_,
    "[ReturnHomeTask] enter home=(%.2f %.2f) current=(%.2f %.2f %.2f) takeoff_z=%.2f",
    ctx.home_x,
    ctx.home_y,
    ctx.cx(),
    ctx.cy(),
    ctx.cz(),
    ctx.takeoff_z);
}

ITask::Status ReturnHomeTask::tick(Context& ctx, double dt)
{
  print_elapsed_ += dt;

  const float step_xy = static_cast<float>(return_step_xy_);

  const float sx = math_tool::clamp(
    ctx.cx(),
    ctx.home_x,
    step_xy);

  const float sy = math_tool::clamp(
    ctx.cy(),
    ctx.home_y,
    step_xy);

  ctx.use_vel_ctrl = false;

  ctx.sp_x = sx;
  ctx.sp_y = sy;
  ctx.sp_z = ctx.takeoff_z;
  ctx.sp_yaw = ctx.home_yaw;

  ctx.sp_vx = 0.0f;
  ctx.sp_vy = 0.0f;
  ctx.sp_vz = 0.0f;

  // 返航阶段关闭视觉
  ctx.vision_front_enable = false;
  ctx.vision_down_enable = false;
  ctx.vision_enable = false;
  ctx.vision_searching = false;
  ctx.vision_target_locked = false;
  ctx.vision_aligned = false;

  const float err_xy = math_tool::dist2d(
    ctx.cx(),
    ctx.cy(),
    ctx.home_x,
    ctx.home_y);

  const float err_z = std::fabs(ctx.cz() - ctx.takeoff_z);

  const float vxy =
    std::sqrt(
      ctx.local_pos.vx * ctx.local_pos.vx +
      ctx.local_pos.vy * ctx.local_pos.vy);

  const bool xy_ok =
    err_xy < static_cast<float>(return_xy_tol_);

  const bool z_ok =
    err_z < 0.35f;

  // 关键：返航结束不要被很小的速度阈值卡死
  const float vxy_tol =
    std::max(static_cast<float>(return_vxy_tol_), 0.25f);

  const bool v_ok =
    vxy < vxy_tol;

  if (print_elapsed_ > 0.5) {
    print_elapsed_ = 0.0;

    RCLCPP_INFO(
      lg_,
      "[ReturnHomeTask] pos=(%.2f %.2f %.2f) home=(%.2f %.2f) sp=(%.2f %.2f %.2f) "
      "err_xy=%.3f/%.3f err_z=%.3f/0.350 vxy=%.3f/%.3f | ok=(xy:%d z:%d v:%d)",
      ctx.cx(),
      ctx.cy(),
      ctx.cz(),
      ctx.home_x,
      ctx.home_y,
      ctx.sp_x,
      ctx.sp_y,
      ctx.sp_z,
      err_xy,
      static_cast<float>(return_xy_tol_),
      err_z,
      vxy,
      vxy_tol,
      xy_ok ? 1 : 0,
      z_ok ? 1 : 0,
      v_ok ? 1 : 0);
  }

  if (xy_ok && z_ok && v_ok) {
    RCLCPP_WARN(
      lg_,
      "[ReturnHomeTask] Arrived home -> HOME_STABILIZE | err_xy=%.3f err_z=%.3f vxy=%.3f",
      err_xy,
      err_z,
      vxy);

    return Status::SUCCESS;
  }

  return Status::RUNNING;
}