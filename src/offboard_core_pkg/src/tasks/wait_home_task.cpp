#include "offboard_core_pkg/tasks/wait_home_task.hpp"

WaitHomeTask::WaitHomeTask(rclcpp::Logger lg)
: lg_(lg)
{}

std::string WaitHomeTask::name() const
{
  return "WAIT_HOME";
}

ITask::Status WaitHomeTask::tick(Context& ctx, double)
{
  if (!ctx.home_inited || !ctx.home_yaw_inited || !ctx.pos_valid()) {
    return Status::RUNNING;
  }

  RCLCPP_INFO(lg_, "Home ready -> PRESETPOINT");
  return Status::SUCCESS;
}