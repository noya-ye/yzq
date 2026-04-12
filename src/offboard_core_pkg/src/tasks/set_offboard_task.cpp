#include "offboard_core_pkg/tasks/set_offboard_task.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"

SetOffboardTask::SetOffboardTask(rclcpp::Logger lg, Px4Iface& px4)
: lg_(lg), px4_(px4)
{}

std::string SetOffboardTask::name() const
{
  return "SET_OFFBOARD";
}

void SetOffboardTask::onEnter(Context&)
{
  sent_ = false;
  t_ = 0.0;
  resend_t_ = 0.0;
}

ITask::Status SetOffboardTask::tick(Context& ctx, double dt)
{
  ctx.use_vel_ctrl = false;

  // 先稳定给一个当前位置/家点 setpoint
  ctx.sp_x = ctx.home_x;
  ctx.sp_y = ctx.home_y;
  ctx.sp_z = ctx.home_z;
  ctx.sp_yaw = ctx.home_yaw;

  t_ += dt;
  resend_t_ += dt;

  // 首次立即发送，之后每 0.5s 重发一次
  if (!sent_ || resend_t_ > 0.5) {
    px4_.set_offboard_mode();
    sent_ = true;
    resend_t_ = 0.0;
    RCLCPP_INFO(lg_, "Sent OFFBOARD mode command");
  }

  // 真正依据 PX4 状态判断是否成功
  if (ctx.vehicle_status.nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD) {
    RCLCPP_INFO(lg_, "SET_OFFBOARD success: PX4 entered OFFBOARD");
    return Status::SUCCESS;
  }

  return Status::RUNNING;
}