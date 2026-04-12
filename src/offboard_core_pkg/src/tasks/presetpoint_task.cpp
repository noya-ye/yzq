#include "offboard_core_pkg/tasks/presetpoint_task.hpp"

PresetpointTask::PresetpointTask(rclcpp::Logger lg)
: lg_(lg)
{}

std::string PresetpointTask::name() const
{
  return "PRESETPOINT";
}

void PresetpointTask::onEnter(Context&)
{
  counter_ = 0;
}

ITask::Status PresetpointTask::tick(Context& ctx, double)
{
  ctx.use_vel_ctrl = false;

  ctx.sp_x = ctx.home_x;
  ctx.sp_y = ctx.home_y;
  ctx.sp_z = ctx.home_z;
  ctx.sp_yaw = ctx.home_yaw;

  counter_++;

  if (counter_ >= 20) {
    RCLCPP_INFO(lg_, "PRESETPOINT done -> SET_OFFBOARD");
    return Status::SUCCESS;
  }

  return Status::RUNNING;
}