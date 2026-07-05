#include "offboard_core_pkg/tasks/px4_land_mode_task.hpp"

#include <cmath>

namespace offboard_core_pkg
{

Px4LandModeTask::Px4LandModeTask(rclcpp::Logger lg, Px4Iface& px4)
: lg_(lg), px4_(px4)
{}

std::string Px4LandModeTask::name() const
{
  return "Px4LandModeTask";
}

void Px4LandModeTask::onEnter(Context& ctx)
{
  land_cmd_sent_ = false;
  force_disarm_sent_ = false;
  wait_time_s_ = 0.0;
  stable_time_s_ = 0.0;

  // 进入 LandTask 后，立刻声明准备交给 PX4 Land
  // 主循环看到这个标志后必须停止发布 offboard setpoint
  ctx.handover_to_px4_land = true;

  RCLCPP_INFO(lg_, "[LAND] enter, handover to PX4 land mode");
}

ITask::Status Px4LandModeTask::tick(Context& ctx, double dt)
{
  using px4_msgs::msg::VehicleStatus;

  wait_time_s_ += dt;

  // =========================
  // 1. 发送 LAND 命令，只发一次
  // =========================
  if (!land_cmd_sent_) {
    RCLCPP_INFO(lg_, "[LAND] send PX4 land command");

    px4_.land();

    land_cmd_sent_ = true;
    stable_time_s_ = 0.0;

    return Status::RUNNING;
  }

  // =========================
  // 2. 如果 PX4 已经自动 disarm，降落完成
  // =========================
  if (ctx.vehicle_status.arming_state ==
      VehicleStatus::ARMING_STATE_DISARMED)
  {
    RCLCPP_INFO(lg_, "[LAND] vehicle disarmed, land task finished");
    return Status::SUCCESS;
  }

  // =========================
  // 3. 落地稳定检测
  // =========================
  const bool landed = ctx.land_detected.landed;

  const bool vz_valid = std::isfinite(ctx.local_pos.vz);
  const bool low_speed = vz_valid && std::fabs(ctx.local_pos.vz) < 0.10f;

  if (landed && low_speed) {
    stable_time_s_ += dt;
  } else {
    stable_time_s_ = 0.0;
  }

   // =========================
  // 4. 只有确认落地稳定后，才允许主动 disarm
  // =========================
  if (stable_time_s_ > 1.5 && !force_disarm_sent_) {
    RCLCPP_WARN(lg_, "[LAND] landed stable, send disarm");
    px4_.disarm();
    force_disarm_sent_ = true;
  }

  // =========================
  // 5. 等待 PX4 回报 DISARMED
  // =========================
  if (wait_time_s_ > 15.0 && std::fmod(wait_time_s_, 2.0) < dt) {
    RCLCPP_WARN(
      lg_,
      "[LAND] still waiting for disarm | landed=%d stable_time=%.2f wait=%.2f",
      landed ? 1 : 0,
      stable_time_s_,
      wait_time_s_);
  }

  return Status::RUNNING;
}

} // namespace offboard_core_pkg