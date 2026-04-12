#include "offboard_core_pkg/tasks/yaw_spin_task.hpp"

namespace offboard_core_pkg
{

void YawSpinTask::onEnter(Context& ctx)
{
  RCLCPP_INFO(logger_, "[YawSpinTask] Start spinning");

  elapsed_ = 0.0;

  // 当前yaw
  target_yaw_ = ctx.home_yaw;

  // 锁定当前位置
  hold_x_ = ctx.local_pos.x;
  hold_y_ = ctx.local_pos.y;
  hold_z_ = ctx.local_pos.z;
}

ITask::Status YawSpinTask::tick(Context& ctx, double dt)
{
  elapsed_ += dt;

  // ===== yaw积分 =====
  target_yaw_ += yaw_rate_ * dt;

  // wrap到[-pi, pi]
  target_yaw_ = math_tool::wrap_pi(target_yaw_);

  // ===== 发布位置控制（只改yaw）=====
  
  ctx.sp_yaw=target_yaw_;

  // ===== 时间结束 =====
  if (elapsed_ >= duration_)
  {
    RCLCPP_INFO(logger_, "[YawSpinTask] Finished");
    return Status::SUCCESS;
  }

  return Status::RUNNING;
}

} // namespace offboard_core_pkg