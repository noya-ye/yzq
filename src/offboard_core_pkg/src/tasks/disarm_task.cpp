#include "offboard_core_pkg/tasks/disarm_task.hpp"

DisarmTask::DisarmTask(rclcpp::Logger lg, Px4Iface& px4)
: lg_(lg), px4_(px4)
{}

std::string DisarmTask::name() const
{
  return "DISARM";
}

void DisarmTask::onEnter(Context&)
{
  sent_ = false;
}

ITask::Status DisarmTask::tick(Context&, double)
{
  if (!sent_) {
    px4_.disarm();
    sent_ = true;
    RCLCPP_INFO(lg_, "Sent DISARM command");
  }
  return Status::SUCCESS;
}