#include "offboard_core_pkg/tasks/start_down_blind_check_task.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>
#include <cstdlib>

namespace offboard_core_pkg
{

static std::string target_type_to_name(int type)
{
  switch (type) {
    case 1:  return "red_circle";
    case 2:  return "red_square";
    case 3:  return "red_triangle";
    case 4:  return "red_pentagon";
    case 5:  return "red_hexagon";

    case 6:  return "green_circle";
    case 7:  return "green_square";
    case 8:  return "green_triangle";
    case 9:  return "green_pentagon";
    case 10: return "green_hexagon";

    case 11: return "yellow_circle";
    case 12: return "yellow_square";
    case 13: return "yellow_triangle";
    case 14: return "yellow_pentagon";
    case 15: return "yellow_hexagon";

    case 16: return "blue_circle";
    case 17: return "blue_square";
    case 18: return "blue_triangle";
    case 19: return "blue_pentagon";
    case 20: return "blue_hexagon";

    case 21: return "illegal_shape";

    default: return "none";
  }
}

StartDownBlindCheckTask::StartDownBlindCheckTask(
    rclcpp::Logger logger,
    double timeout_s,
    double align_tol_m,
    int stable_required,
    double k_img_to_meter,
    double max_step_m,
    double score_thresh,
    double dup_remove_radius,
    double img_to_body_x_sign,
    double img_to_body_y_sign)
: logger_(logger),
  timeout_s_(timeout_s),
  align_tol_m_(align_tol_m),
  stable_required_(stable_required),
  k_img_to_meter_(k_img_to_meter),
  max_step_m_(max_step_m),
  score_thresh_(score_thresh),
  dup_remove_radius_(dup_remove_radius),
  img_to_body_x_sign_(img_to_body_x_sign),
  img_to_body_y_sign_(img_to_body_y_sign)
{}

void StartDownBlindCheckTask::onEnter(Context& ctx)
{
  RCLCPP_WARN(
    logger_,
    "[StartDownBlindCheckTask] enter: DOWN camera ON, FRONT camera OFF");

  elapsed_ = 0.0;
  print_elapsed_ = 0.0;
  align_lost_elapsed_ = 0.0;
  align_elapsed_ = 0.0;

  stable_count_ = 0;
  ever_seen_ = false;
  state_ = BlindCheckState::WAIT_FRAME;

  hold_z_ = ctx.takeoff_z;

  ctx.blindcheck_origin_x = ctx.cx();
  ctx.blindcheck_origin_y = ctx.cy();

  ctx.blindcheck_queue.clear();
  ctx.blindcheck_index = 0;
  ctx.blindcheck_locked = false;
  ctx.blindcheck_locked_target = VisionOffset{};

  ctx.vision_down_targets.clear();
  ctx.vision_down_targets_stamp_us = 0;

  ctx.vision_offset = VisionOffset{};
  ctx.vision_offset.stamp_us = 0;
  ctx.vision_offset.score = 0.0f;

  ctx.vision_last_update_us = 0;

  ctx.vision_front_enable = false;
  ctx.vision_down_enable = true;
  ctx.vision_enable = true;

  ctx.vision_searching = true;
  ctx.vision_target_locked = false;
  ctx.vision_aligned = false;
  ctx.mcu_switch_request = false;

  ctx.use_vel_ctrl = false;

  ctx.sp_x = ctx.cx();
  ctx.sp_y = ctx.cy();
  ctx.sp_z = hold_z_;
  ctx.sp_yaw = ctx.home_yaw;

  ctx.sp_vx = 0.0f;
  ctx.sp_vy = 0.0f;
  ctx.sp_vz = 0.0f;
}

ITask::Status StartDownBlindCheckTask::tick(Context& ctx, double dt)
{
  elapsed_ += dt;
  print_elapsed_ += dt;

  const float safe_dx = ctx.cx() - ctx.blindcheck_origin_x;
  const float safe_dy = ctx.cy() - ctx.blindcheck_origin_y;
  const float safe_dist_xy = std::sqrt(safe_dx * safe_dx + safe_dy * safe_dy);

  constexpr float BLINDCHECK_MAX_RADIUS_M = 2.0f;

  if (safe_dist_xy > BLINDCHECK_MAX_RADIUS_M) {
    RCLCPP_ERROR(
      logger_,
      "[StartDownBlindCheckTask] SAFETY LAND: drift too far from blindcheck origin! "
      "dist_xy=%.3f > %.3f, pos=(%.3f %.3f), origin=(%.3f %.3f)",
      safe_dist_xy,
      BLINDCHECK_MAX_RADIUS_M,
      ctx.cx(),
      ctx.cy(),
      ctx.blindcheck_origin_x,
      ctx.blindcheck_origin_y);

    ctx.vision_front_enable = false;
    ctx.vision_down_enable = false;
    ctx.vision_enable = false;
    ctx.vision_searching = false;
    ctx.vision_target_locked = false;
    ctx.vision_aligned = false;
    ctx.blindcheck_locked = false;
    ctx.mcu_switch_request = false;

    ctx.handover_to_px4_land = true;

    ctx.use_vel_ctrl = false;
    ctx.sp_x = ctx.cx();
    ctx.sp_y = ctx.cy();
    ctx.sp_z = ctx.local_pos.z;
    ctx.sp_yaw = ctx.yaw;
    ctx.sp_vx = 0.0f;
    ctx.sp_vy = 0.0f;
    ctx.sp_vz = 0.0f;

    finish_task(ctx, false);
    return Status::SUCCESS;
  }

  ctx.vision_front_enable = false;
  ctx.vision_down_enable = true;
  ctx.vision_enable = true;

  ctx.use_vel_ctrl = false;
  ctx.sp_z = hold_z_;
  ctx.sp_yaw = ctx.home_yaw;
  ctx.sp_vx = 0.0f;
  ctx.sp_vy = 0.0f;
  ctx.sp_vz = 0.0f;

  if (elapsed_ > timeout_s_ && !ever_seen_) {
    RCLCPP_WARN(
      logger_,
      "[StartDownBlindCheckTask] timeout %.2fs, no down target, skip blind check",
      elapsed_);

    finish_task(ctx, false);
    return Status::SUCCESS;
  }

  switch (state_) {
    case BlindCheckState::WAIT_FRAME:
      return tick_wait_frame(ctx);

    case BlindCheckState::PICK_NEXT:
      return tick_pick_next(ctx);

    case BlindCheckState::ALIGN_TARGET:
      return tick_align_target(ctx, dt);

    case BlindCheckState::RETURN_ORIGIN:
      return tick_return_origin(ctx);

    case BlindCheckState::FINISH:
      finish_task(ctx, true);
      return Status::SUCCESS;
  }

  return Status::RUNNING;
}

ITask::Status StartDownBlindCheckTask::tick_wait_frame(Context& ctx)
{
  ctx.sp_x = ctx.blindcheck_origin_x;
  ctx.sp_y = ctx.blindcheck_origin_y;
  ctx.vision_target_locked = false;

  if (!down_targets_fresh(ctx) || ctx.vision_down_targets.empty()) {
    if (print_elapsed_ > 0.5) {
      print_elapsed_ = 0.0;
      RCLCPP_INFO(
        logger_,
        "[StartDownBlindCheckTask] WAIT_FRAME | fresh=%d targets=%zu elapsed=%.2f",
        down_targets_fresh(ctx) ? 1 : 0,
        ctx.vision_down_targets.size(),
        elapsed_);
    }
    return Status::RUNNING;
  }

  ctx.blindcheck_queue = ctx.vision_down_targets;

  std::sort(
    ctx.blindcheck_queue.begin(),
    ctx.blindcheck_queue.end(),
    [](const VisionOffset& a, const VisionOffset& b) {
      return a.cost < b.cost;
    });

  ctx.blindcheck_index = 0;
  ever_seen_ = true;

  RCLCPP_WARN(
    logger_,
    "[StartDownBlindCheckTask] captured blindcheck queue, size=%zu",
    ctx.blindcheck_queue.size());

  state_ = BlindCheckState::PICK_NEXT;
  return Status::RUNNING;
}

ITask::Status StartDownBlindCheckTask::tick_pick_next(Context& ctx)
{
  ctx.sp_x = ctx.blindcheck_origin_x;
  ctx.sp_y = ctx.blindcheck_origin_y;
  ctx.vision_target_locked = false;
  ctx.mcu_switch_request = false;

  stable_count_ = 0;
  align_lost_elapsed_ = 0.0;
  align_elapsed_ = 0.0;

  if (ctx.blindcheck_index >= static_cast<int>(ctx.blindcheck_queue.size())) {
    RCLCPP_WARN(
      logger_,
      "[StartDownBlindCheckTask] all blindcheck targets finished");

    state_ = BlindCheckState::FINISH;
    return Status::RUNNING;
  }

  ctx.blindcheck_locked_target = ctx.blindcheck_queue[ctx.blindcheck_index];
  ctx.blindcheck_locked = true;

  RCLCPP_WARN(
    logger_,
    "[StartDownBlindCheckTask] PICK_NEXT index=%d/%zu type=%d(%s) dx=%.3f dy=%.3f score=%.1f cost=%.4f",
    ctx.blindcheck_index + 1,
    ctx.blindcheck_queue.size(),
    ctx.blindcheck_locked_target.type,
    target_type_to_name(ctx.blindcheck_locked_target.type).c_str(),
    ctx.blindcheck_locked_target.dx,
    ctx.blindcheck_locked_target.dy,
    ctx.blindcheck_locked_target.score,
    ctx.blindcheck_locked_target.cost);

  state_ = BlindCheckState::ALIGN_TARGET;
  return Status::RUNNING;
}

ITask::Status StartDownBlindCheckTask::tick_align_target(Context& ctx, double dt)
{
  align_elapsed_ += dt;

  // ============================================================
  // ALIGN 总超时保护：
  // 当前目标看得见/看不见都算时间。
  // 超过 align_timeout_s_ 后跳过当前目标，进入下一个目标，
  // 防止一直纠偏或一直等待。
  // ============================================================
  if (align_elapsed_ >= align_timeout_s_) {
  RCLCPP_WARN(
    logger_,
    "[StartDownBlindCheckTask] ALIGN timeout idx=%d/%zu type=%d(%s) after %.2fs, skip current target",
    ctx.blindcheck_index + 1,
    ctx.blindcheck_queue.size(),
    ctx.blindcheck_locked_target.type,
    target_type_to_name(ctx.blindcheck_locked_target.type).c_str(),
    align_elapsed_);

  ctx.blindcheck_locked = false;
  ctx.vision_target_locked = false;
  ctx.mcu_switch_request = false;

  stable_count_ = 0;
  align_lost_elapsed_ = 0.0;
  align_elapsed_ = 0.0;

  // 不要在这里 ++，统一交给 RETURN_ORIGIN 到原点后再 ++
  state_ = BlindCheckState::RETURN_ORIGIN;

  return Status::RUNNING;
}
  VisionOffset matched;

  if (!find_locked_target_in_frame(ctx, matched)) {
    ctx.sp_x = ctx.cx();
    ctx.sp_y = ctx.cy();
    ctx.vision_target_locked = false;
    ctx.mcu_switch_request = false;

    align_lost_elapsed_ += dt;

    if (print_elapsed_ > 0.5) {
      print_elapsed_ = 0.0;

      RCLCPP_WARN(
        logger_,
        "[StartDownBlindCheckTask] ALIGN_TARGET lost locked target | "
        "idx=%d/%zu locked_type=%d(%s) lost=%.2fs align=%.2fs",
        ctx.blindcheck_index + 1,
        ctx.blindcheck_queue.size(),
        ctx.blindcheck_locked_target.type,
        target_type_to_name(ctx.blindcheck_locked_target.type).c_str(),
        align_lost_elapsed_,
        align_elapsed_);
    }

    if (align_lost_elapsed_ >= align_lost_timeout_s_) {
  RCLCPP_WARN(
    logger_,
    "[StartDownBlindCheckTask] skip lost target idx=%d/%zu type=%d(%s) after lost=%.2fs align=%.2fs",
    ctx.blindcheck_index + 1,
    ctx.blindcheck_queue.size(),
    ctx.blindcheck_locked_target.type,
    target_type_to_name(ctx.blindcheck_locked_target.type).c_str(),
    align_lost_elapsed_,
    align_elapsed_);

  ctx.blindcheck_locked = false;
  ctx.vision_target_locked = false;
  ctx.mcu_switch_request = false;

  stable_count_ = 0;
  align_lost_elapsed_ = 0.0;
  align_elapsed_ = 0.0;

  // 不要在这里 ++，统一交给 RETURN_ORIGIN 到原点后再 ++
  state_ = BlindCheckState::RETURN_ORIGIN;
}
    return Status::RUNNING;
  }

  align_lost_elapsed_ = 0.0;

  ctx.blindcheck_locked_target = matched;
  ctx.vision_offset = matched;
  ctx.vision_target_locked = true;
  ever_seen_ = true;

  const float dx = ctx.blindcheck_locked_target.dx;
  const float dy = ctx.blindcheck_locked_target.dy;

  const float err_img = std::sqrt(dx * dx + dy * dy);
  const float err_m = static_cast<float>(k_img_to_meter_) * err_img;

  float dwx = 0.0f;
  float dwy = 0.0f;
  image_offset_to_world_delta(ctx, dx, dy, dwx, dwy);

  const float norm = std::sqrt(dwx * dwx + dwy * dwy);
  float dynamic_max_step = static_cast<float>(max_step_m_);

  if (err_m < 0.20f) {
    dynamic_max_step = std::min(dynamic_max_step, 0.20f);
  }
  if (err_m < 0.10f) {
    dynamic_max_step = std::min(dynamic_max_step, 0.10f);
  }
  if (err_m < 0.06f) {
    dynamic_max_step = std::min(dynamic_max_step, 0.05f);
  }

  if (norm > dynamic_max_step && norm > 1e-6f) {
    const float s = dynamic_max_step / norm;
    dwx *= s;
    dwy *= s;
  }

  if (err_m < static_cast<float>(align_tol_m_)) {
    ctx.sp_x = ctx.cx();
    ctx.sp_y = ctx.cy();
  } else {
    ctx.sp_x = ctx.cx() + dwx;
    ctx.sp_y = ctx.cy() + dwy;
  }

  if (err_m < static_cast<float>(align_tol_m_)) {
    stable_count_++;

    ctx.mcu_switch_request = true;

    std::system("stty -F /dev/ttyUSB1 115200 cs8 -cstopb -parenb raw");
    std::system("printf 'a' > /dev/ttyUSB1");
  } else {
    stable_count_ = 0;
    ctx.mcu_switch_request = false;
  }

  if (print_elapsed_ > 0.5) {
    print_elapsed_ = 0.0;

    RCLCPP_INFO(
      logger_,
      "[StartDownBlindCheckTask] ALIGN_TARGET idx=%d/%zu type=%d(%s) dx=%.3f dy=%.3f "
      "err=%.3f score=%.1f cost=%.4f dworld=(%.3f %.3f) sp=(%.3f %.3f %.3f) stable=%d/%d align=%.2fs",
      ctx.blindcheck_index + 1,
      ctx.blindcheck_queue.size(),
      ctx.blindcheck_locked_target.type,
      target_type_to_name(ctx.blindcheck_locked_target.type).c_str(),
      dx,
      dy,
      err_m,
      ctx.blindcheck_locked_target.score,
      ctx.blindcheck_locked_target.cost,
      dwx,
      dwy,
      ctx.sp_x,
      ctx.sp_y,
      ctx.sp_z,
      stable_count_,
      stable_required_,
      align_elapsed_);
  }

  if (stable_count_ >= stable_required_) {
    RCLCPP_WARN(
      logger_,
      "[StartDownBlindCheckTask] target %d/%zu aligned, pos=(%.3f, %.3f), err=%.3f, type=%d(%s)",
      ctx.blindcheck_index + 1,
      ctx.blindcheck_queue.size(),
      ctx.cx(),
      ctx.cy(),
      err_m,
      ctx.blindcheck_locked_target.type,
      target_type_to_name(ctx.blindcheck_locked_target.type).c_str());

    remove_nearby_rough_targets(ctx, ctx.cx(), ctx.cy());

    ctx.blindcheck_locked = false;
    ctx.vision_target_locked = false;
    ctx.mcu_switch_request = false;

    stable_count_ = 0;
    align_lost_elapsed_ = 0.0;
    align_elapsed_ = 0.0;

    state_ = BlindCheckState::RETURN_ORIGIN;
  }

  return Status::RUNNING;
}

ITask::Status StartDownBlindCheckTask::tick_return_origin(Context& ctx)
{
  ctx.sp_x = ctx.blindcheck_origin_x;
  ctx.sp_y = ctx.blindcheck_origin_y;
  ctx.vision_target_locked = false;
  ctx.mcu_switch_request = false;

  std::system("stty -F /dev/ttyUSB1 115200 cs8 -cstopb -parenb raw");
  std::system("printf 'b' > /dev/ttyUSB1");

  const float dx = ctx.cx() - ctx.blindcheck_origin_x;
  const float dy = ctx.cy() - ctx.blindcheck_origin_y;
  const float d = std::sqrt(dx * dx + dy * dy);

  if (print_elapsed_ > 0.5) {
    print_elapsed_ = 0.0;

    RCLCPP_INFO(
      logger_,
      "[StartDownBlindCheckTask] RETURN_ORIGIN idx=%d/%zu dist=%.3f",
      ctx.blindcheck_index + 1,
      ctx.blindcheck_queue.size(),
      d);
  }

  if (d < 0.10f) {
    ctx.blindcheck_index++;
    stable_count_ = 0;
    align_lost_elapsed_ = 0.0;
    align_elapsed_ = 0.0;
    state_ = BlindCheckState::PICK_NEXT;
  }

  return Status::RUNNING;
}

bool StartDownBlindCheckTask::offset_fresh(const Context& ctx) const
{
  if (ctx.vision_offset.stamp_us == 0 || ctx.vision_last_update_us == 0) {
    return false;
  }

  const uint64_t age_us =
    (ctx.vision_last_update_us > ctx.vision_offset.stamp_us)
      ? (ctx.vision_last_update_us - ctx.vision_offset.stamp_us)
      : 0ULL;

  return age_us < 300000ULL;
}

bool StartDownBlindCheckTask::down_targets_fresh(const Context& ctx) const
{
  if (ctx.vision_down_targets_stamp_us == 0 || ctx.vision_last_update_us == 0) {
    return false;
  }

  const uint64_t age_us =
    (ctx.vision_last_update_us > ctx.vision_down_targets_stamp_us)
      ? (ctx.vision_last_update_us - ctx.vision_down_targets_stamp_us)
      : 0ULL;

  return age_us < 300000ULL;
}

bool StartDownBlindCheckTask::find_locked_target_in_frame(
    const Context& ctx,
    VisionOffset& matched) const
{
  if (!down_targets_fresh(ctx) || ctx.vision_down_targets.empty()) {
    return false;
  }

  const int locked_type = ctx.blindcheck_locked_target.type;

  if (locked_type == 0) {
    return false;
  }

  std::vector<VisionOffset> valid_same_type_targets;
  valid_same_type_targets.reserve(ctx.vision_down_targets.size());

  for (const auto& t : ctx.vision_down_targets) {
    if (t.type == 0) {
      continue;
    }

    if (t.score < static_cast<float>(score_thresh_)) {
      continue;
    }

    if (t.type != locked_type) {
      continue;
    }

    valid_same_type_targets.push_back(t);
  }

  if (valid_same_type_targets.empty()) {
    return false;
  }

  float best_d2 = std::numeric_limits<float>::max();
  bool found = false;

  for (const auto& t : valid_same_type_targets) {
    const float ddx = t.dx - ctx.blindcheck_locked_target.dx;
    const float ddy = t.dy - ctx.blindcheck_locked_target.dy;
    const float d2 = ddx * ddx + ddy * ddy;

    if (d2 < best_d2) {
      best_d2 = d2;
      matched = t;
      found = true;
    }
  }

  if (!found) {
    return false;
  }

  constexpr float MATCH_GATE_SAME_TYPE = 0.25f;

  if (best_d2 >= MATCH_GATE_SAME_TYPE * MATCH_GATE_SAME_TYPE) {
    return false;
  }

  return true;
}

void StartDownBlindCheckTask::finish_task(Context& ctx, bool aligned)
{
  ctx.vision_down_enable = false;
  ctx.vision_enable = false;
  ctx.vision_searching = false;
  ctx.vision_target_locked = false;
  ctx.vision_aligned = aligned;
  ctx.blindcheck_locked = false;
  ctx.mcu_switch_request = false;
}

void StartDownBlindCheckTask::image_offset_to_world_delta(
    const Context& ctx,
    float dx_img,
    float dy_img,
    float& dwx,
    float& dwy) const
{
  const float body_dx =
    static_cast<float>(img_to_body_x_sign_ * k_img_to_meter_ * dy_img);

  const float body_dy =
    static_cast<float>(img_to_body_y_sign_ * k_img_to_meter_ * dx_img);

  const float cy = std::cos(ctx.yaw);
  const float sy = std::sin(ctx.yaw);

  dwx = cy * body_dx - sy * body_dy;
  dwy = sy * body_dx + cy * body_dy;
}

void StartDownBlindCheckTask::remove_nearby_rough_targets(
    Context& ctx,
    float x,
    float y)
{
  const std::size_t before = ctx.detected_targets.size();

  ctx.detected_targets.erase(
    std::remove_if(
      ctx.detected_targets.begin(),
      ctx.detected_targets.end(),
      [&](const VisionPosition& p) {
        const float dx = p.x - x;
        const float dy = p.y - y;

        return std::sqrt(dx * dx + dy * dy) <
               static_cast<float>(dup_remove_radius_);
      }),
    ctx.detected_targets.end());

  const std::size_t after = ctx.detected_targets.size();

  if (after != before) {
    RCLCPP_WARN(
      logger_,
      "[StartDownBlindCheckTask] removed %zu duplicate rough targets near down target",
      before - after);
  }
}

std::string StartDownBlindCheckTask::name() const
{
  return "START_DOWN_BLIND_CHECK";
}

}  // namespace offboard_core_pkg