#include <cmath>
#include <algorithm>

#include "offboard_core_pkg/tasks/yaw_spin2_task.hpp"

namespace offboard_core_pkg
{

void YawSpin2Task::onEnter(Context& ctx)
{
  RCLCPP_INFO(logger_, "[YawSpin2Task] Start 8-sector yaw scan");

  elapsed_ = 0.0;
  state_elapsed_ = 0.0;
  last_print_s_ = -10.0;

  current_segment_ = 0;
  state_ = State::ROTATING;

  hold_x_ = ctx.local_pos.x;
  hold_y_ = ctx.local_pos.y;
  hold_z_ = ctx.local_pos.z;

  start_yaw_ = ctx.yaw;
  target_yaw_ = start_yaw_;

  segment_target_yaw_ = math_tool::wrap_pi(
    start_yaw_ + kSegmentAngle);

  ctx.last_yaw = ctx.yaw;
  ctx.yaw_accumulated = 0.0f;

  ctx.sp_x = hold_x_;
  ctx.sp_y = hold_y_;
  ctx.sp_z = hold_z_;
  ctx.sp_yaw = target_yaw_;

  ctx.sp_vx = 0.0f;
  ctx.sp_vy = 0.0f;
  ctx.sp_vz = 0.0f;
  ctx.use_vel_ctrl = false;

  stop_vision(ctx);

  ctx.vision_searching = true;
  ctx.vision_target_locked = false;
  ctx.vision_aligned = false;
  ctx.vision_detected = false;
  ctx.vision_done = false;

  RCLCPP_WARN(
    logger_,
    "[YawSpin2Task] enter hold=(%.2f %.2f %.2f), start_yaw=%.2f, first_target_yaw=%.2f",
    hold_x_, hold_y_, hold_z_,
    start_yaw_,
    segment_target_yaw_);

  RCLCPP_WARN(
    logger_,
    "[YawSpin2Task] mode: rotate 45deg -> settle -> sample all visible targets, total 8 sectors");
}

ITask::Status YawSpin2Task::tick(Context& ctx, double dt)
{
  elapsed_ += dt;
  state_elapsed_ += dt;

  // 始终锁住进入任务时的位置，只改变 yaw
  ctx.sp_x = hold_x_;
  ctx.sp_y = hold_y_;
  ctx.sp_z = hold_z_;

  ctx.sp_vx = 0.0f;
  ctx.sp_vy = 0.0f;
  ctx.sp_vz = 0.0f;
  ctx.use_vel_ctrl = false;

  // YawSpin2 只使用前视相机，下视关闭
  ctx.vision_down_enable = false;
  ctx.vision_enable = true;
  ctx.vision_searching = true;

  if (drift_check(ctx)) {
    return Status::FAILURE;
  }

  const float current_yaw = ctx.yaw;

  const float dyaw = math_tool::wrap_pi(current_yaw - ctx.last_yaw);
  ctx.yaw_accumulated += std::fabs(dyaw);
  ctx.last_yaw = current_yaw;

  switch (state_)
  {
    case State::ROTATING:
    {
      // 旋转阶段关闭前视采样，避免运动模糊和重复加入
      ctx.vision_front_enable = false;
      ctx.vision_detected = false;
      ctx.vision_done = false;

      // 从当前 setpoint 平滑推进到本段标准目标角
      const float err_to_final =
        math_tool::wrap_pi(segment_target_yaw_ - target_yaw_);

      const float step =
        static_cast<float>(yaw_rate_ * dt);

      if (std::fabs(err_to_final) > step) {
        target_yaw_ = math_tool::wrap_pi(
          target_yaw_ + std::copysign(step, err_to_final));
      } else {
        target_yaw_ = segment_target_yaw_;
      }

      ctx.sp_yaw = target_yaw_;

      // 用真实 yaw 判断是否已经到达本段目标
      const float yaw_err =
        math_tool::wrap_pi(segment_target_yaw_ - current_yaw);

      // 原来 5 度偏大，可能还没稳就进入 SETTLING
      constexpr float YAW_REACHED_TOL =
        2.5f * static_cast<float>(M_PI) / 180.0f;

      if (std::fabs(yaw_err) < YAW_REACHED_TOL)
      {
        RCLCPP_INFO(
          logger_,
          "[YawSpin2Task] segment %d/%d reached, yaw=%.2f target=%.2f -> settling",
          current_segment_ + 1,
          kTotalSegments,
          current_yaw,
          segment_target_yaw_);

        // 关键修改：
        // 不要 target_yaw_ = current_yaw;
        // 到达后继续压住标准 45 度目标角，避免停住时 setpoint 回弹
        target_yaw_ = segment_target_yaw_;
        ctx.sp_yaw = target_yaw_;

        state_ = State::SETTLING;
        state_elapsed_ = 0.0;

        stop_vision(ctx);
      }

      break;
    }

    case State::SETTLING:
    {
      // 稳定阶段继续压住本段标准目标角
      // 不用 current_yaw，避免停住瞬间回弹
      target_yaw_ = segment_target_yaw_;
      ctx.sp_yaw = target_yaw_;

      stop_vision(ctx);

      if (state_elapsed_ >= settle_time_s_)
      {
        RCLCPP_INFO(
          logger_,
          "[YawSpin2Task] segment %d/%d stable -> enable front sampling",
          current_segment_ + 1,
          kTotalSegments);

        ctx.vision_detected = false;
        ctx.vision_done = false;
        ctx.vision_target_locked = false;
        ctx.vision_aligned = false;

        enable_front_sampling(ctx);

        state_ = State::SAMPLING;
        state_elapsed_ = 0.0;
      }

      break;
    }

    case State::SAMPLING:
    {
      // 采样阶段也继续压住本段标准目标角
      target_yaw_ = segment_target_yaw_;
      ctx.sp_yaw = target_yaw_;

      enable_front_sampling(ctx);

      if (ctx.vision_done)
      {
        RCLCPP_INFO(
          logger_,
          "[YawSpin2Task] segment %d/%d sample done",
          current_segment_ + 1,
          kTotalSegments);

        ctx.vision_detected = false;
        ctx.vision_done = false;
        ctx.vision_target_locked = false;
        ctx.vision_aligned = false;

        stop_vision(ctx);

        current_segment_++;

        if (current_segment_ >= kTotalSegments)
        {
          RCLCPP_INFO(
            logger_,
            "[YawSpin2Task] Finished full 360deg scan, 8 stable samples completed");

          stop_vision(ctx);

          ctx.vision_enable = false;
          ctx.vision_searching = false;
          ctx.vision_target_locked = false;
          ctx.vision_aligned = false;
          ctx.vision_detected = false;
          ctx.vision_done = false;

          return Status::SUCCESS;
        }

        // 下一段标准目标角
        segment_target_yaw_ = math_tool::wrap_pi(
          start_yaw_ + static_cast<float>(current_segment_ + 1) * kSegmentAngle);

        // 当前段结束后，从“上一段标准角”开始平滑推进
        // 不从 current_yaw 开始，避免姿态微小误差被带入下一段
        target_yaw_ = math_tool::wrap_pi(
          start_yaw_ + static_cast<float>(current_segment_) * kSegmentAngle);

        ctx.sp_yaw = target_yaw_;

        RCLCPP_INFO(
          logger_,
          "[YawSpin2Task] next segment %d/%d target_yaw=%.2f",
          current_segment_ + 1,
          kTotalSegments,
          segment_target_yaw_);

        state_ = State::ROTATING;
        state_elapsed_ = 0.0;
      }
      else if (state_elapsed_ >= sample_timeout_s_)
      {
        RCLCPP_WARN(
          logger_,
          "[YawSpin2Task] segment %d/%d sample timeout, continue next sector",
          current_segment_ + 1,
          kTotalSegments);

        ctx.vision_detected = false;
        ctx.vision_done = false;
        ctx.vision_target_locked = false;
        ctx.vision_aligned = false;

        stop_vision(ctx);

        current_segment_++;

        if (current_segment_ >= kTotalSegments)
        {
          RCLCPP_INFO(
            logger_,
            "[YawSpin2Task] Finished full 360deg scan, some sectors may have no target");

          stop_vision(ctx);

          ctx.vision_enable = false;
          ctx.vision_searching = false;

          return Status::SUCCESS;
        }

        // 下一段标准目标角
        segment_target_yaw_ = math_tool::wrap_pi(
          start_yaw_ + static_cast<float>(current_segment_ + 1) * kSegmentAngle);

        // timeout 分支也不要从 current_yaw 开始
        target_yaw_ = math_tool::wrap_pi(
          start_yaw_ + static_cast<float>(current_segment_) * kSegmentAngle);

        ctx.sp_yaw = target_yaw_;

        RCLCPP_INFO(
          logger_,
          "[YawSpin2Task] next segment %d/%d target_yaw=%.2f after timeout",
          current_segment_ + 1,
          kTotalSegments,
          segment_target_yaw_);

        state_ = State::ROTATING;
        state_elapsed_ = 0.0;
      }

      break;
    }
  }

  if (elapsed_ - last_print_s_ > 0.5)
  {
    last_print_s_ = elapsed_;

    const char* state_name = "UNKNOWN";
    if (state_ == State::ROTATING) {
      state_name = "ROTATING";
    } else if (state_ == State::SETTLING) {
      state_name = "SETTLING";
    } else if (state_ == State::SAMPLING) {
      state_name = "SAMPLING";
    }

    RCLCPP_INFO(
      logger_,
      "[YawSpin2Task] state=%s seg=%d/%d yaw=%.2f sp_yaw=%.2f seg_target=%.2f yaw_err=%.2f front=%d done=%d total_yaw=%.2f",
      state_name,
      current_segment_ + 1,
      kTotalSegments,
      current_yaw,
      ctx.sp_yaw,
      segment_target_yaw_,
      math_tool::wrap_pi(segment_target_yaw_ - current_yaw),
      ctx.vision_front_enable ? 1 : 0,
      ctx.vision_done ? 1 : 0,
      ctx.yaw_accumulated);
  }

  return Status::RUNNING;
}

void YawSpin2Task::stop_vision(Context& ctx)
{
  ctx.vision_front_enable = false;
  ctx.vision_down_enable = false;
  ctx.vision_enable = true;
}

void YawSpin2Task::enable_front_sampling(Context& ctx)
{
  ctx.vision_front_enable = true;
  ctx.vision_down_enable = false;
  ctx.vision_enable = true;
  ctx.vision_searching = true;
}

bool YawSpin2Task::drift_check(Context& ctx)
{
  const float ex = ctx.local_pos.x - hold_x_;
  const float ey = ctx.local_pos.y - hold_y_;
  const float ez = ctx.local_pos.z - hold_z_;

  const float e_xy = std::sqrt(ex * ex + ey * ey);
  const float e_z = std::fabs(ez);

  const float v_xy = std::sqrt(
    ctx.local_pos.vx * ctx.local_pos.vx +
    ctx.local_pos.vy * ctx.local_pos.vy);

  constexpr float DRIFT_CRITICAL_XY_M = 0.60f;
  constexpr float DRIFT_CRITICAL_Z_M = 0.40f;

  if (e_xy > DRIFT_CRITICAL_XY_M || e_z > DRIFT_CRITICAL_Z_M)
  {
    RCLCPP_ERROR(
      logger_,
      "[YawSpin2Task] CRITICAL drift detected! e_xy=%.2f e_z=%.2f v_xy=%.2f. Request PX4 land.",
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

    return true;
  }

  return false;
}

}  // namespace offboard_core_pkg