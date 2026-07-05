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

  // 解锁前维持当前位置/家点
  ctx.sp_x = ctx.home_x;
  ctx.sp_y = ctx.home_y;
  ctx.sp_z = ctx.home_z;
  ctx.sp_yaw = ctx.home_yaw;

  t_ += dt;
  resend_t_ += dt;

  // 首次立即发，后续每 0.5s 重发一次
  if (!sent_ || resend_t_ > 0.5) {
    px4_.arm();
    sent_ = true;
    resend_t_ = 0.0;
    RCLCPP_INFO(lg_, "Sent ARM command");
  }

  // 真实依据 PX4 状态判断是否成功
  constexpr uint8_t ARMING_STATE_ARMED_VALUE = 2;

  if (ctx.vehicle_status.arming_state == ARMING_STATE_ARMED_VALUE ||
      ctx.vehicle_status.armed_time > 0) {
    RCLCPP_INFO(lg_, "ARM success: PX4 armed");
    return Status::SUCCESS;
  }

  return Status::RUNNING;
}