#include "offboard_core_pkg/tasks/return_home_task.hpp"


#include <cmath>

ReturnHomeTask::ReturnHomeTask(rclcpp::Logger lg,
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

ITask::Status ReturnHomeTask::tick(Context& ctx, double)
{
  const float step_xy = static_cast<float>(return_step_xy_);
  const float sx = math_tool::clamp(ctx.cx(), ctx.home_x, step_xy);
  const float sy = math_tool::clamp(ctx.cy(), ctx.home_y, step_xy);

  ctx.use_vel_ctrl = false;
  ctx.sp_x = sx;
  ctx.sp_y = sy;
  ctx.sp_z = ctx.takeoff_z;
  ctx.sp_yaw = ctx.home_yaw;

  const bool xy_ok =
    (math_tool::dist2d(ctx.cx(), ctx.cy(), ctx.home_x, ctx.home_y) <
     static_cast<float>(return_xy_tol_));

  const bool z_ok = math_tool::near(ctx.cz(), ctx.takeoff_z, 0.25f);

  const float vxy =
    std::sqrt(ctx.local_pos.vx * ctx.local_pos.vx +
              ctx.local_pos.vy * ctx.local_pos.vy);

  const bool v_ok = (vxy < static_cast<float>(return_vxy_tol_));

  if (xy_ok && z_ok && v_ok) {
    RCLCPP_INFO(lg_, "Arrived home -> HOME_STABILIZE");
    return Status::SUCCESS;
  }

  return Status::RUNNING;
}