#include "offboard_core_pkg/tasks/px4_land_mode_task.hpp"

namespace offboard_core_pkg
{

Px4LandModeTask::Px4LandModeTask(rclcpp::Logger lg, Px4Iface& px4)
: lg_(lg), px4_(px4)
{}

std::string Px4LandModeTask::name() const
{
  return "Px4LandModeTask";
}

void Px4LandModeTask::onEnter(Context&)
{
  land_cmd_sent_ = false;
  force_disarm_sent_ = false;
  wait_time_s_ = 0.0;
  stable_time_s_ = 0.0;

  RCLCPP_INFO(lg_, "[LAND] enter");
}

ITask::Status Px4LandModeTask::tick(Context& ctx, double dt)
{
  ctx.use_vel_ctrl = false;
  ctx.sp_x = ctx.cx();
  ctx.sp_y = ctx.cy();
  ctx.sp_z = ctx.cz();
  ctx.sp_yaw = ctx.home_yaw;

  using px4_msgs::msg::VehicleStatus;

  // =========================
  // 1. 发送 LAND
  // =========================
  if (!land_cmd_sent_) {
    RCLCPP_INFO(lg_, "[LAND] send land command");

    px4_.land();
    ctx.handover_to_px4_land = true;

    land_cmd_sent_ = true;
    wait_time_s_ = 0.0;
    stable_time_s_ = 0.0;

    return Status::RUNNING;
  }

  wait_time_s_ += dt;

  // =========================
  // 2. 自动 disarm
  // =========================
  if (ctx.vehicle_status.arming_state ==
      VehicleStatus::ARMING_STATE_DISARMED)
  {
    RCLCPP_INFO(lg_, "[LAND] disarmed, finished");
    return Status::SUCCESS;
  }

  // =========================
  // 3. 稳定落地检测
  // =========================
  bool landed = ctx.land_detected.landed;
  bool low_speed = std::fabs(ctx.local_pos.vz) < 0.1f;

  if (landed && low_speed) {
    stable_time_s_ += dt;
  } else {
    stable_time_s_ = 0.0;
  }

  // 稳定1秒 → disarm
  if (stable_time_s_ > 1.0 && !force_disarm_sent_) {
    RCLCPP_WARN(lg_, "[LAND] stable landed → disarm");
    px4_.disarm();
    force_disarm_sent_ = true;
  }

  // =========================
  // 4. 超时兜底
  // =========================
  if (wait_time_s_ > 2.0 && !force_disarm_sent_) {
    RCLCPP_ERROR(lg_, "[LAND] timeout → force disarm");
    px4_.disarm();
    force_disarm_sent_ = true;
  }

  if (wait_time_s_ > 5.0) {
    RCLCPP_WARN(lg_, "[LAND] finish by timeout");
    return Status::SUCCESS;
  }

  return Status::RUNNING;
}

} // namespace offboard_core_pkg