#include <new>
#include <cmath>

#include "offboard_core_pkg/tasks/yaw_spin_task.hpp"

namespace offboard_core_pkg
{

enum class SpinState
{
  SPINNING,
  STABILIZING
};

void YawSpinTask::onEnter(Context& ctx)
{
  RCLCPP_INFO(logger_, "[YawSpinTask] Start spinning");

  elapsed_ = 0.0;
  target_yaw_ = ctx.yaw;
  ctx.last_yaw = ctx.yaw;
  ctx.yaw_accumulated = 0.0f;
  state_ = SpinState::SPINNING;

  hold_x_ = ctx.local_pos.x;
  hold_y_ = ctx.local_pos.y;
  hold_z_ = ctx.local_pos.z;

  ctx.sp_x = hold_x_;
  ctx.sp_y = hold_y_;
  ctx.sp_z = hold_z_;
  ctx.sp_yaw = target_yaw_;

  ctx.sp_vx = 0.0f;
  ctx.sp_vy = 0.0f;
  ctx.sp_vz = 0.0f;
  ctx.use_vel_ctrl = false;

  // YawSpin 初始阶段：前视 D435 开，等待发现目标作为“触发信号”
  ctx.vision_front_enable = true;
  ctx.vision_down_enable = false;
  ctx.vision_enable = true;

  ctx.vision_detected = false;
  ctx.vision_done = false;
  ctx.vision_searching = true;
  ctx.vision_target_locked = false;
  ctx.vision_aligned = false;

  RCLCPP_WARN(
    logger_,
    "[YawSpinTask] enter hold=(%.2f %.2f %.2f), vel=(%.2f %.2f %.2f), yaw=%.2f",
    hold_x_, hold_y_, hold_z_,
    ctx.local_pos.vx, ctx.local_pos.vy, ctx.local_pos.vz,
    ctx.yaw);

  RCLCPP_WARN(logger_, "[YawSpinTask] vision gate: FRONT=ON, DOWN=OFF");
}

ITask::Status YawSpinTask::tick(Context& ctx, double dt)
{
  elapsed_ += dt;

  const float current_yaw = ctx.yaw;

  // 始终锁住进入任务时的位置
  ctx.sp_x = hold_x_;
  ctx.sp_y = hold_y_;
  ctx.sp_z = hold_z_;

  ctx.sp_vx = 0.0f;
  ctx.sp_vy = 0.0f;
  ctx.sp_vz = 0.0f;
  ctx.use_vel_ctrl = false;

  // 下视相机在 YawSpin 阶段关闭
  ctx.vision_down_enable = false;
  ctx.vision_enable = true;
  ctx.vision_searching = true;

  const float ex = ctx.local_pos.x - hold_x_;
  const float ey = ctx.local_pos.y - hold_y_;
  const float ez = ctx.local_pos.z - hold_z_;

  const float e_xy = std::sqrt(ex * ex + ey * ey);
  const float e_z = std::fabs(ez);

  const float v_xy = std::sqrt(
    ctx.local_pos.vx * ctx.local_pos.vx +
    ctx.local_pos.vy * ctx.local_pos.vy);

  constexpr float DRIFT_PAUSE_XY_M = 0.25f;
  constexpr float DRIFT_RESUME_XY_M = 0.12f;
  constexpr float DRIFT_CRITICAL_XY_M = 0.60f;

  constexpr float DRIFT_PAUSE_VXY_MPS = 0.35f;
  constexpr float DRIFT_RESUME_VXY_MPS = 0.15f;

  constexpr float DRIFT_CRITICAL_Z_M = 0.40f;

  if (e_xy > DRIFT_CRITICAL_XY_M || e_z > DRIFT_CRITICAL_Z_M)
  {
    RCLCPP_ERROR(
      logger_,
      "[YawSpinTask] CRITICAL drift detected! e_xy=%.2f e_z=%.2f v_xy=%.2f. Request PX4 land.",
      e_xy, e_z, v_xy);

    ctx.handover_to_px4_land = true;

    ctx.vision_front_enable = false;
    ctx.vision_down_enable = false;
    ctx.vision_enable = false;

    ctx.vision_searching = false;
    ctx.vision_target_locked = false;
    ctx.vision_aligned = false;
    ctx.vision_detected = false;
    ctx.vision_done = false;

    return Status::FAILURE;
  }

  const bool drift_pause =
    (e_xy > DRIFT_PAUSE_XY_M) ||
    (v_xy > DRIFT_PAUSE_VXY_MPS);

  const bool drift_recovered =
    (e_xy < DRIFT_RESUME_XY_M) &&
    (v_xy < DRIFT_RESUME_VXY_MPS);

  const float dyaw = math_tool::wrap_pi(current_yaw - ctx.last_yaw);
  ctx.yaw_accumulated += std::fabs(dyaw);
  ctx.last_yaw = current_yaw;

  if (drift_pause && !drift_recovered)
  {
    target_yaw_ = current_yaw;
    ctx.sp_yaw = target_yaw_;

    RCLCPP_WARN(
      logger_,
      "[YawSpinTask] drift pause yaw: e_xy=%.2f e_z=%.2f v_xy=%.2f hold=(%.2f %.2f %.2f) pos=(%.2f %.2f %.2f)",
      e_xy, e_z, v_xy,
      hold_x_, hold_y_, hold_z_,
      ctx.local_pos.x, ctx.local_pos.y, ctx.local_pos.z);

    return Status::RUNNING;
  }

  switch (state_)
  {
    case SpinState::SPINNING:
    {
      // 只有旋转扫描阶段打开前视识别，用于发现目标触发停转
      ctx.vision_front_enable = true;

      const float yaw_err = math_tool::wrap_pi(target_yaw_ - current_yaw);

      constexpr float max_yaw_err =
        8.0f * static_cast<float>(M_PI) / 180.0f;

      if (std::fabs(yaw_err) < max_yaw_err)
      {
        target_yaw_ = math_tool::wrap_pi(
          target_yaw_ + static_cast<float>(yaw_rate_ * dt));
      }

      ctx.sp_yaw = target_yaw_;

      if (ctx.vision_detected)
      {
        RCLCPP_INFO(logger_, "[YawSpinTask] Target trigger -> stop yaw and wait stable");

        state_ = SpinState::STABILIZING;

        // 停在当前真实 yaw
        target_yaw_ = current_yaw;
        ctx.sp_yaw = target_yaw_;

        // 进入稳定阶段先关前视，等控制节点判断稳定后再重新打开采样
        ctx.vision_front_enable = false;
      }

      break;
    }

    case SpinState::STABILIZING:
    {
      // 稳定阶段只保持 yaw，不推进 yaw
      ctx.sp_yaw = target_yaw_;

      // 注意：这里不强行改 ctx.vision_front_enable
      // 前视采样开关交给 yaw_spin_test.cpp 的稳定采样状态机控制

      if (ctx.vision_done)
      {
        RCLCPP_INFO(logger_, "[YawSpinTask] Stable sample done -> resume spinning");

        ctx.vision_detected = false;
        ctx.vision_done = false;

        target_yaw_ = current_yaw;
        ctx.sp_yaw = target_yaw_;

        ctx.vision_front_enable = true;
        state_ = SpinState::SPINNING;
      }

      break;
    }
  }

  static double last_debug_print_s = -10.0;
  if (elapsed_ - last_debug_print_s > 0.5)
  {
    last_debug_print_s = elapsed_;
  }

  if (ctx.yaw_accumulated >= 2.0f * static_cast<float>(M_PI))
  {
    RCLCPP_INFO(logger_, "[YawSpinTask] Finished full rotation");

    ctx.vision_front_enable = false;
    ctx.vision_down_enable = false;
    ctx.vision_enable = false;

    ctx.vision_searching = false;
    ctx.vision_target_locked = false;
    ctx.vision_aligned = false;
    ctx.vision_detected = false;
    ctx.vision_done = false;

    return Status::SUCCESS;
  }

  return Status::RUNNING;
}

}  // namespace offboard_core_pkg
