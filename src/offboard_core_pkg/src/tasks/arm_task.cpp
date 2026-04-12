#include "offboard_core_pkg/tasks/arm_task.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"

ArmTask::ArmTask(rclcpp::Logger lg, Px4Iface& px4)
: lg_(lg), px4_(px4)
{}

std::string ArmTask::name() const
{
  return "ARM";
}

void ArmTask::onEnter(Context&)
{
  sent_ = false;
  t_ = 0.0;
  resend_t_ = 0.0;
}

ITask::Status ArmTask::tick(Context& ctx, double dt)
{
  ctx.use_vel_ctrl = false;

  // ✅ 更安全：保持当前位置
  ctx.sp_x = ctx.cx();
  ctx.sp_y = ctx.cy();
  ctx.sp_z = ctx.cz();
  ctx.sp_yaw = ctx.home_yaw;

  t_ += dt;
  resend_t_ += dt;

  // 每 0.5s 重发 ARM
  if (!sent_ || resend_t_ > 0.5) {
    px4_.arm();
    sent_ = true;
    resend_t_ = 0.0;
    RCLCPP_INFO(lg_, "Sent ARM command");
  }

  using px4_msgs::msg::VehicleStatus;

  // ✅ 正确判断
  if (ctx.vehicle_status.arming_state ==
      VehicleStatus::ARMING_STATE_ARMED)
  {
    RCLCPP_INFO(lg_, "ARM success: PX4 armed");
    return Status::SUCCESS;
  }

  // ✅ 超时保护（必须）
  if (t_ > 5.0) {
    RCLCPP_ERROR(lg_, "ARM timeout!");
    return Status::FAILURE;
  }

  return Status::RUNNING;
}