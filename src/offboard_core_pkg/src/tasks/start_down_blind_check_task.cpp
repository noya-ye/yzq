#include "offboard_core_pkg/tasks/start_down_blind_check_task.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <string>
#include <vector>

namespace offboard_core_pkg
{

namespace
{

std::string target_type_to_name(int type)
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

void send_mcu_command(char command)
{
  std::system("stty -F /dev/ttyUSB1 115200 cs8 -cstopb -parenb raw");

  if (command == 'a') {
    std::system("printf 'a' > /dev/ttyUSB1");
  } else if (command == 'b') {
    std::system("printf 'b' > /dev/ttyUSB1");
  }
}

}  // namespace

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
  down_frame_age_s_ = 1e9;

  stable_count_ = 0;
  ever_seen_ = false;
  state_ = BlindCheckState::WAIT_FRAME;

  last_down_frame_stamp_us_ = 0;
  last_control_frame_stamp_us_ = 0;
  last_track_frame_stamp_us_ = 0;

  align_cmd_x_ = ctx.cx();
  align_cmd_y_ = ctx.cy();

  mcu_a_sent_ = false;
  mcu_b_sent_ = false;

  next_track_id_ = 1;
  locked_track_id_ = -1;
  tracks_.clear();
  track_queue_.clear();

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

  const uint64_t frame_stamp = ctx.vision_down_targets_stamp_us;

  if (frame_stamp != 0 && frame_stamp != last_down_frame_stamp_us_) {
    last_down_frame_stamp_us_ = frame_stamp;
    down_frame_age_s_ = 0.0;
  } else {
    down_frame_age_s_ += dt;
  }

  const float safe_dist = std::hypot(
    ctx.cx() - ctx.blindcheck_origin_x,
    ctx.cy() - ctx.blindcheck_origin_y);

  constexpr float MAX_RADIUS_M = 2.0f;

  if (safe_dist > MAX_RADIUS_M) {
    RCLCPP_ERROR(
      logger_,
      "[StartDownBlindCheckTask] SAFETY LAND: dist=%.3f > %.3f",
      safe_dist,
      MAX_RADIUS_M);

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
      "[StartDownBlindCheckTask] timeout %.2fs, no target",
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
        "[StartDownBlindCheckTask] WAIT_FRAME fresh=%d targets=%zu elapsed=%.2f",
        down_targets_fresh(ctx) ? 1 : 0,
        ctx.vision_down_targets.size(),
        elapsed_);
    }
    return Status::RUNNING;
  }

  capture_tracks(ctx);
  ever_seen_ = !track_queue_.empty();

  RCLCPP_WARN(
    logger_,
    "[StartDownBlindCheckTask] captured tracks=%zu",
    track_queue_.size());

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
  last_control_frame_stamp_us_ = 0;
  align_cmd_x_ = ctx.cx();
  align_cmd_y_ = ctx.cy();
  mcu_a_sent_ = false;
  mcu_b_sent_ = false;

  if (ctx.blindcheck_index >= static_cast<int>(track_queue_.size())) {
    RCLCPP_WARN(logger_, "[StartDownBlindCheckTask] all targets finished");
    state_ = BlindCheckState::FINISH;
    return Status::RUNNING;
  }

  locked_track_id_ = track_queue_[ctx.blindcheck_index];
  TargetTrack* track = find_track(locked_track_id_);

  if (track == nullptr) {
    ctx.blindcheck_index++;
    return Status::RUNNING;
  }

  ctx.blindcheck_locked_target = track->observation;
  ctx.blindcheck_locked = true;

  RCLCPP_WARN(
    logger_,
    "[StartDownBlindCheckTask] PICK_NEXT %d/%zu track=%d type=%d(%s) anchor=(%.3f %.3f)",
    ctx.blindcheck_index + 1,
    track_queue_.size(),
    track->id,
    track->type,
    target_type_to_name(track->type).c_str(),
    track->world_x,
    track->world_y);

  state_ = BlindCheckState::ALIGN_TARGET;
  return Status::RUNNING;
}

ITask::Status StartDownBlindCheckTask::tick_align_target(Context& ctx, double dt)
{
  align_elapsed_ += dt;

  if (align_elapsed_ >= align_timeout_s_) {
    RCLCPP_WARN(
      logger_,
      "[StartDownBlindCheckTask] ALIGN timeout track=%d after %.2fs",
      locked_track_id_,
      align_elapsed_);

    if (TargetTrack* track = find_track(locked_track_id_)) {
      track->processed = true;
    }

    ctx.blindcheck_locked = false;
    ctx.vision_target_locked = false;
    ctx.mcu_switch_request = false;
    mcu_b_sent_ = false;
    state_ = BlindCheckState::RETURN_ORIGIN;
    return Status::RUNNING;
  }

  VisionOffset matched;

  if (!find_locked_target_in_frame(ctx, matched)) {
    align_cmd_x_ = ctx.cx();
    align_cmd_y_ = ctx.cy();
    ctx.sp_x = align_cmd_x_;
    ctx.sp_y = align_cmd_y_;
    ctx.vision_target_locked = false;
    ctx.mcu_switch_request = false;
    align_lost_elapsed_ += dt;

    if (print_elapsed_ > 0.5) {
      print_elapsed_ = 0.0;
      RCLCPP_WARN(
        logger_,
        "[StartDownBlindCheckTask] track=%d lost=%.2fs align=%.2fs",
        locked_track_id_,
        align_lost_elapsed_,
        align_elapsed_);
    }

    if (align_lost_elapsed_ >= align_lost_timeout_s_) {
      RCLCPP_WARN(
        logger_,
        "[StartDownBlindCheckTask] skip lost track=%d",
        locked_track_id_);

      if (TargetTrack* track = find_track(locked_track_id_)) {
        track->processed = true;
      }

      ctx.blindcheck_locked = false;
      mcu_b_sent_ = false;
      state_ = BlindCheckState::RETURN_ORIGIN;
    }

    return Status::RUNNING;
  }

  align_lost_elapsed_ = 0.0;
  ctx.blindcheck_locked_target = matched;
  ctx.vision_offset = matched;
  ctx.vision_target_locked = true;
  ever_seen_ = true;

  const float dx = matched.dx;
  const float dy = matched.dy;
  const float err_m =
    static_cast<float>(k_img_to_meter_) * std::hypot(dx, dy);

  float dwx = 0.0f;
  float dwy = 0.0f;
  image_offset_to_world_delta(ctx, dx, dy, dwx, dwy);

  constexpr float CORRECTION_GAIN = 0.4f;
  dwx *= CORRECTION_GAIN;
  dwy *= CORRECTION_GAIN;

  const float norm = std::hypot(dwx, dwy);
  const float max_step = static_cast<float>(max_step_m_);

  if (norm > max_step && norm > 1e-6f) {
    const float scale = max_step / norm;
    dwx *= scale;
    dwy *= scale;
  }

  const uint64_t frame_stamp = ctx.vision_down_targets_stamp_us;
  const bool new_frame = frame_stamp != last_control_frame_stamp_us_;
  const float enter_tol = static_cast<float>(align_tol_m_);
  const float exit_tol = enter_tol * 1.5f;

  if (new_frame) {
    last_control_frame_stamp_us_ = frame_stamp;

    if (err_m < enter_tol) {
      align_cmd_x_ = ctx.cx();
      align_cmd_y_ = ctx.cy();
      stable_count_++;

      if (!mcu_a_sent_) {
        send_mcu_command('a');
        mcu_a_sent_ = true;
      }
    } else {
      align_cmd_x_ = ctx.cx() + dwx;
      align_cmd_y_ = ctx.cy() + dwy;

      if (err_m > exit_tol) {
        stable_count_ = 0;
      }
    }
  }

  ctx.sp_x = align_cmd_x_;
  ctx.sp_y = align_cmd_y_;
  ctx.mcu_switch_request = err_m < enter_tol;

  if (print_elapsed_ > 0.5) {
    print_elapsed_ = 0.0;
    RCLCPP_INFO(
      logger_,
      "[StartDownBlindCheckTask] ALIGN track=%d dx=%.3f dy=%.3f err=%.3f "
      "sp=(%.3f %.3f) stable=%d/%d new=%d age=%.2f",
      locked_track_id_,
      dx,
      dy,
      err_m,
      ctx.sp_x,
      ctx.sp_y,
      stable_count_,
      stable_required_,
      new_frame ? 1 : 0,
      down_frame_age_s_);
  }

  if (stable_count_ >= stable_required_) {
    RCLCPP_WARN(
      logger_,
      "[StartDownBlindCheckTask] track=%d aligned, err=%.3f",
      locked_track_id_,
      err_m);

    if (TargetTrack* track = find_track(locked_track_id_)) {
      track->processed = true;
    }

    remove_nearby_rough_targets(ctx, ctx.cx(), ctx.cy());

    ctx.blindcheck_locked = false;
    ctx.vision_target_locked = false;
    ctx.mcu_switch_request = false;
    mcu_b_sent_ = false;
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

  if (!mcu_b_sent_) {
    send_mcu_command('b');
    mcu_b_sent_ = true;
  }

  const float dist = std::hypot(
    ctx.cx() - ctx.blindcheck_origin_x,
    ctx.cy() - ctx.blindcheck_origin_y);

  if (print_elapsed_ > 0.5) {
    print_elapsed_ = 0.0;
    RCLCPP_INFO(
      logger_,
      "[StartDownBlindCheckTask] RETURN_ORIGIN %d/%zu dist=%.3f",
      ctx.blindcheck_index + 1,
      track_queue_.size(),
      dist);
  }

  if (dist < 0.10f) {
    ctx.blindcheck_index++;
    locked_track_id_ = -1;
    state_ = BlindCheckState::PICK_NEXT;
  }

  return Status::RUNNING;
}

bool StartDownBlindCheckTask::offset_fresh(const Context& ctx) const
{
  return ctx.vision_offset.stamp_us != 0 && down_frame_age_s_ < 0.30;
}

bool StartDownBlindCheckTask::down_targets_fresh(const Context& ctx) const
{
  return ctx.vision_down_targets_stamp_us != 0 &&
         ctx.vision_down_targets_stamp_us == last_down_frame_stamp_us_ &&
         down_frame_age_s_ < 0.30;
}

void StartDownBlindCheckTask::capture_tracks(Context& ctx)
{
  tracks_.clear();
  track_queue_.clear();
  ctx.blindcheck_queue.clear();

  struct QueueItem
  {
    float cost;
    int track_id;
  };

  std::vector<QueueItem> order;

  for (const auto& target : ctx.vision_down_targets) {
    if (target.type == 0 ||
        target.score < static_cast<float>(score_thresh_)) {
      continue;
    }

    TargetTrack track;
    track.id = next_track_id_++;
    track.type = target.type;
    track.observation = target;
    track.seen_stamp_us = ctx.vision_down_targets_stamp_us;
    target_world_position(ctx, target, track.world_x, track.world_y);

    tracks_.push_back(track);
    order.push_back({target.cost, track.id});
    ctx.blindcheck_queue.push_back(target);
  }

  std::sort(
    order.begin(),
    order.end(),
    [](const QueueItem& a, const QueueItem& b) {
      return a.cost < b.cost;
    });

  ctx.blindcheck_queue.clear();

  for (const auto& item : order) {
    track_queue_.push_back(item.track_id);

    if (TargetTrack* track = find_track(item.track_id)) {
      ctx.blindcheck_queue.push_back(track->observation);
    }
  }

  last_track_frame_stamp_us_ = ctx.vision_down_targets_stamp_us;
}

void StartDownBlindCheckTask::update_tracks_from_frame(const Context& ctx)
{
  const uint64_t stamp = ctx.vision_down_targets_stamp_us;

  if (!down_targets_fresh(ctx) ||
      stamp == last_track_frame_stamp_us_) {
    return;
  }

  last_track_frame_stamp_us_ = stamp;

  struct Detection
  {
    VisionOffset target;
    float world_x;
    float world_y;
  };

  struct Candidate
  {
    float dist2;
    std::size_t track_index;
    std::size_t detection_index;
  };

  std::vector<Detection> detections;

  for (const auto& target : ctx.vision_down_targets) {
    if (target.type == 0 ||
        target.score < static_cast<float>(score_thresh_)) {
      continue;
    }

    Detection detection;
    detection.target = target;
    target_world_position(
      ctx,
      target,
      detection.world_x,
      detection.world_y);
    detections.push_back(detection);
  }

  std::vector<Candidate> candidates;
  const float max_gate = static_cast<float>(track_match_gate_m_);
  std::vector<float> track_gate(tracks_.size(), max_gate);

  // 同类型轨迹使用互不重叠的归属半径。
  // 落在两个目标中间的模糊检测会被拒绝，而不是串到另一个目标。
  for (std::size_t i = 0; i < tracks_.size(); ++i) {
    for (std::size_t j = 0; j < tracks_.size(); ++j) {
      if (i == j || tracks_[i].type != tracks_[j].type) {
        continue;
      }

      const float separation = std::hypot(
        tracks_[i].world_x - tracks_[j].world_x,
        tracks_[i].world_y - tracks_[j].world_y);

      track_gate[i] = std::min(track_gate[i], 0.45f * separation);
    }
  }

  for (std::size_t i = 0; i < tracks_.size(); ++i) {
    const float gate2 = track_gate[i] * track_gate[i];

    for (std::size_t j = 0; j < detections.size(); ++j) {
      if (tracks_[i].type != detections[j].target.type) {
        continue;
      }

      const float dx = detections[j].world_x - tracks_[i].world_x;
      const float dy = detections[j].world_y - tracks_[i].world_y;
      const float dist2 = dx * dx + dy * dy;

      if (dist2 < gate2) {
        candidates.push_back({dist2, i, j});
      }
    }
  }

  std::sort(
    candidates.begin(),
    candidates.end(),
    [](const Candidate& a, const Candidate& b) {
      return a.dist2 < b.dist2;
    });

  std::vector<bool> track_used(tracks_.size(), false);
  std::vector<bool> detection_used(detections.size(), false);

  for (const auto& candidate : candidates) {
    if (track_used[candidate.track_index] ||
        detection_used[candidate.detection_index]) {
      continue;
    }

    TargetTrack& track = tracks_[candidate.track_index];
    const Detection& detection = detections[candidate.detection_index];

    track.observation = detection.target;
    track.seen_stamp_us = stamp;

    track_used[candidate.track_index] = true;
    detection_used[candidate.detection_index] = true;
  }
}

StartDownBlindCheckTask::TargetTrack*
StartDownBlindCheckTask::find_track(int id)
{
  for (auto& track : tracks_) {
    if (track.id == id) {
      return &track;
    }
  }
  return nullptr;
}

bool StartDownBlindCheckTask::find_locked_target_in_frame(
    Context& ctx,
    VisionOffset& matched)
{
  if (!down_targets_fresh(ctx) || locked_track_id_ < 0) {
    return false;
  }

  update_tracks_from_frame(ctx);

  TargetTrack* track = find_track(locked_track_id_);

  if (track == nullptr ||
      track->seen_stamp_us != ctx.vision_down_targets_stamp_us) {
    return false;
  }

  matched = track->observation;
  return true;
}

void StartDownBlindCheckTask::target_world_position(
    const Context& ctx,
    const VisionOffset& target,
    float& world_x,
    float& world_y) const
{
  float dwx = 0.0f;
  float dwy = 0.0f;
  image_offset_to_world_delta(ctx, target.dx, target.dy, dwx, dwy);

  world_x = ctx.cx() + dwx;
  world_y = ctx.cy() + dwy;
}

void StartDownBlindCheckTask::finish_task(Context& ctx, bool aligned)
{
  if (mcu_a_sent_ && !mcu_b_sent_) {
    send_mcu_command('b');
    mcu_b_sent_ = true;
  }

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
  const float body_dx = static_cast<float>(
    img_to_body_x_sign_ * k_img_to_meter_ * dy_img);

  const float body_dy = static_cast<float>(
    img_to_body_y_sign_ * k_img_to_meter_ * dx_img);

  const float cos_yaw = std::cos(ctx.yaw);
  const float sin_yaw = std::sin(ctx.yaw);

  dwx = cos_yaw * body_dx - sin_yaw * body_dy;
  dwy = sin_yaw * body_dx + cos_yaw * body_dy;
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
      [&](const VisionPosition& target) {
        return std::hypot(target.x - x, target.y - y) <
               static_cast<float>(dup_remove_radius_);
      }),
    ctx.detected_targets.end());

  const std::size_t removed = before - ctx.detected_targets.size();

  if (removed > 0) {
    RCLCPP_WARN(
      logger_,
      "[StartDownBlindCheckTask] removed %zu duplicate rough targets",
      removed);
  }
}

std::string StartDownBlindCheckTask::name() const
{
  return "START_DOWN_BLIND_CHECK";
}

}  // namespace offboard_core_pkg
