#include "offboard_core_pkg/tasks/home_stabilize_task.hpp"

HomeStabilizeTask::HomeStabilizeTask(rclcpp::Logger lg, double home_stabilize_s)
: lg_(lg), home_stabilize_s_(home_stabilize_s)
{}

std::string HomeStabilizeTask::name() const
{
  return "HOME_STABILIZE";
}

void HomeStabilizeTask::onEnter(Context&)
{
  t_ = 0.0;
}

ITask::Status HomeStabilizeTask::tick(Context& ctx, double dt)
{
  ctx.use_vel_ctrl = false;
  ctx.sp_x = ctx.home_x;
  ctx.sp_y = ctx.home_y;
  ctx.sp_z = ctx.takeoff_z;
  ctx.sp_yaw = ctx.home_yaw;

  t_ += dt;

  if (t_ >= home_stabilize_s_) {
    RCLCPP_INFO(lg_, "HOME_STABILIZE done -> LAND_DESCEND");
    return Status::SUCCESS;
  }

  return Status::RUNNING;
}