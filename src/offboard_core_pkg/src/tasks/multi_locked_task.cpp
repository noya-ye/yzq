#include "offboard_core_pkg/tasks/multi_locked_task.hpp"

namespace offboard_core_pkg
{

MultiLockedTask::MultiLockedTask(rclcpp::Logger logger)
: logger_(logger),
  cfg_(Config{})
{}

MultiLockedTask::MultiLockedTask(
    rclcpp::Logger logger,
    const Config& cfg)
: logger_(logger),
  cfg_(cfg)
{}

std::string MultiLockedTask::name() const
{
  return "MULTI_LOCKED";
}

void MultiLockedTask::onEnter(Context& ctx)
{
  elapsed_s_ = 0.0;
  print_elapsed_s_ = 0.0;
  align_elapsed_s_ = 0.0;
  align_lost_elapsed_s_ = 0.0;

  stable_count_ = 0;
  error_grow_count_ = 0;
  has_last_err_ = false;
  last_err_m_ = 0.0f;

  reset_mot();

  multi_state_ = MultiState::WAIT_FRAME;

  target_queue_.clear();
  target_index_ = 0;

  has_current_target_ = false;
  current_target_type_ = 0;
  current_seed_target_ = Detection{};

  origin_x_ = ctx.cx();
  origin_y_ = ctx.cy();
  hold_z_ = ctx.takeoff_z;

  ctx.vision_down_enable = true;
  ctx.vision_front_enable = false;
  ctx.vision_enable = true;

  ctx.vision_target_locked = false;
  ctx.vision_aligned = false;

  ctx.use_vel_ctrl = false;
  ctx.sp_x = origin_x_;
  ctx.sp_y = origin_y_;
  ctx.sp_z = hold_z_;
  ctx.sp_yaw = ctx.home_yaw;

  ctx.sp_vx = 0.0f;
  ctx.sp_vy = 0.0f;
  ctx.sp_vz = 0.0f;

  RCLCPP_WARN(
      logger_,
      "[MultiLockedTask] enter QUEUE mode: origin=(%.3f, %.3f) hold_z=%.3f "
      "timeout=%.2fs align_timeout=%.2fs align_tol=%.3fm stable_required=%d max_step=%.3fm",
      origin_x_,
      origin_y_,
      hold_z_,
      cfg_.timeout_s,
      cfg_.align_timeout_s,
      cfg_.align_tol_m,
      cfg_.stable_required,
      cfg_.max_step_m);
}

void MultiLockedTask::onExit(Context& ctx)
{
  ctx.vision_down_enable = false;
  ctx.vision_target_locked = false;

  RCLCPP_WARN(
      logger_,
      "[MultiLockedTask] exit: multi_state=%s mot_state=%s index=%d/%zu stable=%d lost=%d ambiguous=%d",
      multi_state_name(),
      mot_state_name(),
      target_index_,
      target_queue_.size(),
      stable_count_,
      track_.lost_count,
      track_.ambiguous_count);
}

ITask::Status MultiLockedTask::tick(Context& ctx, double dt)
{
  elapsed_s_ += dt;
  print_elapsed_s_ += dt;

  ctx.vision_down_enable = true;
  ctx.vision_front_enable = false;
  ctx.vision_enable = true;

  ctx.use_vel_ctrl = false;
  ctx.sp_z = hold_z_;
  ctx.sp_yaw = ctx.home_yaw;
  ctx.sp_vx = 0.0f;
  ctx.sp_vy = 0.0f;
  ctx.sp_vz = 0.0f;

  if (elapsed_s_ > cfg_.timeout_s) {
    RCLCPP_WARN(
        logger_,
        "[MultiLockedTask] total timeout %.2fs, finish queue at index=%d/%zu",
        elapsed_s_,
        target_index_,
        target_queue_.size());

    return finish_status();
  }

  switch (multi_state_) {
    case MultiState::WAIT_FRAME:
      return tick_wait_frame(ctx);

    case MultiState::PICK_NEXT:
      return tick_pick_next(ctx);

    case MultiState::ALIGN_TARGET:
      return tick_align_target(ctx, dt);

    case MultiState::RETURN_ORIGIN:
      return tick_return_origin(ctx);

    case MultiState::FINISH:
      return tick_finish(ctx);
  }

  return Status::RUNNING;
}

ITask::Status MultiLockedTask::tick_wait_frame(Context& ctx)
{
  ctx.sp_x = origin_x_;
  ctx.sp_y = origin_y_;
  ctx.sp_z = hold_z_;
  ctx.sp_yaw = ctx.home_yaw;

  ctx.vision_target_locked = false;
  ctx.vision_aligned = false;

  const auto detections = build_detections(ctx);

  if (detections.empty()) {
    if (print_elapsed_s_ > 0.5) {
      print_elapsed_s_ = 0.0;

      RCLCPP_INFO(
          logger_,
          "[MultiLockedTask] WAIT_FRAME | detections=0 elapsed=%.2f",
          elapsed_s_);
    }

    return Status::RUNNING;
  }

  target_queue_ = detections;

  std::sort(
      target_queue_.begin(),
      target_queue_.end(),
      [](const Detection& a, const Detection& b) {
        return a.cost < b.cost;
      });

  target_index_ = 0;

  RCLCPP_WARN(
      logger_,
      "[MultiLockedTask] captured target queue, size=%zu",
      target_queue_.size());

  for (std::size_t i = 0; i < target_queue_.size(); ++i) {
    RCLCPP_WARN(
        logger_,
        "[MultiLockedTask] queue[%zu] type=%d dx=%.3f dy=%.3f score=%.1f cost=%.4f",
        i,
        target_queue_[i].type,
        target_queue_[i].dx,
        target_queue_[i].dy,
        target_queue_[i].score,
        target_queue_[i].cost);
  }

  multi_state_ = MultiState::PICK_NEXT;
  return Status::RUNNING;
}

ITask::Status MultiLockedTask::tick_pick_next(Context& ctx)
{
  ctx.sp_x = origin_x_;
  ctx.sp_y = origin_y_;
  ctx.sp_z = hold_z_;
  ctx.sp_yaw = ctx.home_yaw;

  ctx.vision_target_locked = false;

  reset_align_state();
  reset_mot();

  if (target_index_ >= static_cast<int>(target_queue_.size())) {
    RCLCPP_WARN(
        logger_,
        "[MultiLockedTask] all queue targets finished");

    multi_state_ = MultiState::FINISH;
    return Status::RUNNING;
  }

  current_seed_target_ = target_queue_[target_index_];
  current_target_type_ = current_seed_target_.type;
  has_current_target_ = true;

  // 关键：用排序队列里的目标作为当前 MOT 初始锁定目标
  track_.valid = true;
  track_.id = next_track_id_++;
  track_.type = current_seed_target_.type;
  track_.dx = current_seed_target_.dx;
  track_.dy = current_seed_target_.dy;
  track_.score = current_seed_target_.score;
  track_.cost = current_seed_target_.cost;
  track_.vx = 0.0f;
  track_.vy = 0.0f;
  track_.age = 1;
  track_.hit_count = 1;
  track_.lost_count = 0;
  track_.ambiguous_count = 0;

  mot_state_ = MotState::LOCKED;

  RCLCPP_WARN(
      logger_,
      "[MultiLockedTask] PICK_NEXT %d/%zu type=%d dx=%.3f dy=%.3f score=%.1f cost=%.4f",
      target_index_ + 1,
      target_queue_.size(),
      current_seed_target_.type,
      current_seed_target_.dx,
      current_seed_target_.dy,
      current_seed_target_.score,
      current_seed_target_.cost);

  multi_state_ = MultiState::ALIGN_TARGET;
  return Status::RUNNING;
}

ITask::Status MultiLockedTask::tick_align_target(Context& ctx, double dt)
{
  align_elapsed_s_ += dt;

  if (align_elapsed_s_ > cfg_.align_timeout_s) {
    RCLCPP_WARN(
        logger_,
        "[MultiLockedTask] ALIGN timeout target %d/%zu type=%d after %.2fs, skip current",
        target_index_ + 1,
        target_queue_.size(),
        current_target_type_,
        align_elapsed_s_);

    target_index_++;
    reset_mot();
    reset_align_state();

    if (cfg_.return_origin_between_targets) {
      multi_state_ = MultiState::RETURN_ORIGIN;
    } else {
      multi_state_ = MultiState::PICK_NEXT;
    }

    return Status::RUNNING;
  }

  const auto detections = build_detections(ctx);
  const bool mot_ok = mot_update(detections, dt);

  if (!mot_ok || mot_state_ == MotState::FAILED) {
    RCLCPP_WARN(
        logger_,
        "[MultiLockedTask] MOT failed target %d/%zu type=%d, skip current",
        target_index_ + 1,
        target_queue_.size(),
        current_target_type_);

    target_index_++;
    reset_mot();
    reset_align_state();

    if (cfg_.return_origin_between_targets) {
      multi_state_ = MultiState::RETURN_ORIGIN;
    } else {
      multi_state_ = MultiState::PICK_NEXT;
    }

    return Status::RUNNING;
  }

  if (!has_valid_lock()) {
    ctx.vision_target_locked = false;
    ctx.sp_x = ctx.cx();
    ctx.sp_y = ctx.cy();

    if (print_elapsed_s_ > 0.5) {
      print_elapsed_s_ = 0.0;

      RCLCPP_WARN(
          logger_,
          "[MultiLockedTask] ALIGN no valid lock target %d/%zu type=%d state=%s det=%zu",
          target_index_ + 1,
          target_queue_.size(),
          current_target_type_,
          mot_state_name(),
          detections.size());
    }

    return Status::RUNNING;
  }

  ctx.vision_target_locked = true;

  ctx.blindcheck_locked_target.type = track_.type;
  ctx.blindcheck_locked_target.dx = track_.dx;
  ctx.blindcheck_locked_target.dy = track_.dy;
  ctx.blindcheck_locked_target.score = track_.score;
  ctx.blindcheck_locked_target.cost = track_.cost;

  ctx.vision_offset = ctx.blindcheck_locked_target;

  const float dx = track_.dx;
  const float dy = track_.dy;

  const float err_img = std::sqrt(dx * dx + dy * dy);
  const float err_m = static_cast<float>(cfg_.k_img_to_meter) * err_img;

  if (err_m < static_cast<float>(cfg_.align_tol_m)) {
    stable_count_++;
  } else {
    stable_count_ = 0;
  }

  if (stable_count_ >= cfg_.stable_required) {
    RCLCPP_WARN(
        logger_,
        "[MultiLockedTask] target %d/%zu aligned type=%d err=%.3f pos=(%.3f %.3f)",
        target_index_ + 1,
        target_queue_.size(),
        current_target_type_,
        err_m,
        ctx.cx(),
        ctx.cy());

    target_index_++;
    reset_mot();
    reset_align_state();

    if (cfg_.return_origin_between_targets) {
      multi_state_ = MultiState::RETURN_ORIGIN;
    } else {
      multi_state_ = MultiState::PICK_NEXT;
    }

    return Status::RUNNING;
  }

  if (check_error_growing(err_m)) {
    RCLCPP_ERROR(
        logger_,
        "[MultiLockedTask] error keeps growing target %d/%zu type=%d, skip current. "
        "err_m=%.3f last_err=%.3f grow_count=%d",
        target_index_ + 1,
        target_queue_.size(),
        current_target_type_,
        err_m,
        last_err_m_,
        error_grow_count_);

    target_index_++;
    reset_mot();
    reset_align_state();

    if (cfg_.return_origin_between_targets) {
      multi_state_ = MultiState::RETURN_ORIGIN;
    } else {
      multi_state_ = MultiState::PICK_NEXT;
    }

    return Status::RUNNING;
  }

  if (mot_state_ == MotState::LOST) {
    ctx.vision_target_locked = false;

    if (cfg_.hold_when_lost) {
      ctx.sp_x = ctx.cx();
      ctx.sp_y = ctx.cy();
    }

    align_lost_elapsed_s_ += dt;

    if (print_elapsed_s_ > 0.5) {
      print_elapsed_s_ = 0.0;

      RCLCPP_WARN(
          logger_,
          "[MultiLockedTask] target LOST %d/%zu type=%d lost=%.2fs lost_count=%d",
          target_index_ + 1,
          target_queue_.size(),
          current_target_type_,
          align_lost_elapsed_s_,
          track_.lost_count);
    }

    if (align_lost_elapsed_s_ > cfg_.align_lost_timeout_s) {
      RCLCPP_WARN(
          logger_,
          "[MultiLockedTask] lost timeout target %d/%zu type=%d, skip current",
          target_index_ + 1,
          target_queue_.size(),
          current_target_type_);

      target_index_++;
      reset_mot();
      reset_align_state();

      if (cfg_.return_origin_between_targets) {
        multi_state_ = MultiState::RETURN_ORIGIN;
      } else {
        multi_state_ = MultiState::PICK_NEXT;
      }
    }

    return Status::RUNNING;
  }

  align_lost_elapsed_s_ = 0.0;

  float dwx = 0.0f;
  float dwy = 0.0f;
  image_offset_to_world_delta(ctx, dx, dy, dwx, dwy);

  const float norm = std::sqrt(dwx * dwx + dwy * dwy);
  if (norm < 1e-5f) {
    return Status::RUNNING;
  }

  float max_step = calc_dynamic_step(err_m);

  if (mot_state_ == MotState::AMBIGUOUS) {
    max_step *= static_cast<float>(cfg_.ambiguous_step_scale);
  }

  const float scale = std::min(1.0f, max_step / norm);

  const float step_x = dwx * scale;
  const float step_y = dwy * scale;

  ctx.use_vel_ctrl = false;
  ctx.sp_x = ctx.cx() + step_x;
  ctx.sp_y = ctx.cy() + step_y;
  ctx.sp_z = hold_z_;
  ctx.sp_yaw = ctx.home_yaw;

  if (print_elapsed_s_ > 0.5) {
    print_elapsed_s_ = 0.0;

    RCLCPP_INFO(
        logger_,
        "[MultiLockedTask] ALIGN %d/%zu multi=%s mot=%s type=%d dx=%.3f dy=%.3f "
        "err=%.3f score=%.1f cost=%.4f step=(%.3f %.3f) sp=(%.3f %.3f %.3f) "
        "stable=%d/%d lost=%d amb=%d det=%zu align=%.2fs",
        target_index_ + 1,
        target_queue_.size(),
        multi_state_name(),
        mot_state_name(),
        current_target_type_,
        dx,
        dy,
        err_m,
        track_.score,
        track_.cost,
        step_x,
        step_y,
        ctx.sp_x,
        ctx.sp_y,
        ctx.sp_z,
        stable_count_,
        cfg_.stable_required,
        track_.lost_count,
        track_.ambiguous_count,
        detections.size(),
        align_elapsed_s_);
  }

  return Status::RUNNING;
}

ITask::Status MultiLockedTask::tick_return_origin(Context& ctx)
{
  ctx.vision_target_locked = false;

  ctx.use_vel_ctrl = false;
  ctx.sp_x = origin_x_;
  ctx.sp_y = origin_y_;
  ctx.sp_z = hold_z_;
  ctx.sp_yaw = ctx.home_yaw;

  const float dx = ctx.cx() - origin_x_;
  const float dy = ctx.cy() - origin_y_;
  const float dist = std::sqrt(dx * dx + dy * dy);

  if (print_elapsed_s_ > 0.5) {
    print_elapsed_s_ = 0.0;

    RCLCPP_INFO(
        logger_,
        "[MultiLockedTask] RETURN_ORIGIN next=%d/%zu dist=%.3f origin=(%.3f %.3f) pos=(%.3f %.3f)",
        target_index_ + 1,
        target_queue_.size(),
        dist,
        origin_x_,
        origin_y_,
        ctx.cx(),
        ctx.cy());
  }

  if (dist < static_cast<float>(cfg_.return_origin_tol_m)) {
    multi_state_ = MultiState::PICK_NEXT;
  }

  return Status::RUNNING;
}

ITask::Status MultiLockedTask::tick_finish(Context& ctx)
{
  ctx.vision_down_enable = false;
  ctx.vision_front_enable = false;
  ctx.vision_enable = false;

  ctx.vision_target_locked = false;
  ctx.vision_aligned = true;

  RCLCPP_WARN(
      logger_,
      "[MultiLockedTask] FINISH all targets, total=%zu",
      target_queue_.size());

  return Status::SUCCESS;
}

void MultiLockedTask::reset_mot()
{
  mot_state_ = MotState::IDLE;
  track_ = Track{};
}

void MultiLockedTask::reset_align_state()
{
  stable_count_ = 0;
  error_grow_count_ = 0;
  has_last_err_ = false;
  last_err_m_ = 0.0f;

  align_elapsed_s_ = 0.0;
  align_lost_elapsed_s_ = 0.0;
}

std::vector<MultiLockedTask::Detection>
MultiLockedTask::build_detections(const Context& ctx) const
{
  std::vector<Detection> detections;
  detections.reserve(ctx.vision_down_targets.size());

  for (const auto& t : ctx.vision_down_targets) {
    if (t.score < static_cast<float>(cfg_.score_thresh)) {
      continue;
    }

    if (t.type == 0) {
      continue;
    }

    Detection d;
    d.type = t.type;
    d.dx = t.dx;
    d.dy = t.dy;
    d.score = t.score;
    d.cost = t.cost;

    // 兜底：如果视觉节点没有填 cost，就用中心距离排序
    if (d.cost <= 0.0f) {
      d.cost = std::sqrt(d.dx * d.dx + d.dy * d.dy);
    }

    detections.push_back(d);
  }

  return detections;
}

bool MultiLockedTask::mot_update(
    const std::vector<Detection>& detections,
    double dt)
{
  if (!track_.valid) {
    return mot_init_lock(detections);
  }

  return mot_update_locked(detections, dt);
}

bool MultiLockedTask::mot_init_lock(
    const std::vector<Detection>& detections)
{
  if (detections.empty()) {
    mot_state_ = MotState::IDLE;
    return true;
  }

  int best_idx = -1;
  float best_cost = std::numeric_limits<float>::infinity();

  for (int i = 0; i < static_cast<int>(detections.size()); ++i) {
    const auto& d = detections[i];

    if (has_current_target_ && d.type != current_target_type_) {
      continue;
    }

    const float center_dist = std::sqrt(d.dx * d.dx + d.dy * d.dy);

    // 初始锁定：优先接近队列 seed，其次 score 大
    float seed_dist = 0.0f;
    if (has_current_target_) {
      const float sdx = d.dx - current_seed_target_.dx;
      const float sdy = d.dy - current_seed_target_.dy;
      seed_dist = std::sqrt(sdx * sdx + sdy * sdy);
    }

    const float cost = seed_dist + 0.3f * center_dist - 0.00001f * d.score;

    if (cost < best_cost) {
      best_cost = cost;
      best_idx = i;
    }
  }

  if (best_idx < 0) {
    mot_state_ = MotState::IDLE;
    return true;
  }

  const auto& d = detections[best_idx];

  track_.valid = true;
  track_.id = next_track_id_++;
  track_.type = d.type;
  track_.dx = d.dx;
  track_.dy = d.dy;
  track_.score = d.score;
  track_.cost = d.cost;
  track_.vx = 0.0f;
  track_.vy = 0.0f;
  track_.age = 1;
  track_.hit_count = 1;
  track_.lost_count = 0;
  track_.ambiguous_count = 0;

  mot_state_ = MotState::LOCKED;

  RCLCPP_WARN(
      logger_,
      "[MultiLockedTask] MOT init lock: id=%d type=%d dx=%.3f dy=%.3f score=%.1f cost=%.4f",
      track_.id,
      track_.type,
      track_.dx,
      track_.dy,
      track_.score,
      track_.cost);

  return true;
}

bool MultiLockedTask::mot_update_locked(
    const std::vector<Detection>& detections,
    double dt)
{
  track_.age++;

  if (detections.empty()) {
    return mark_lost();
  }

  const float dt_f = std::max(0.001f, static_cast<float>(dt));

  const float pred_dx = track_.dx + track_.vx * dt_f;
  const float pred_dy = track_.dy + track_.vy * dt_f;

  int best_idx = -1;
  int second_idx = -1;

  float best_cost = std::numeric_limits<float>::infinity();
  float second_cost = std::numeric_limits<float>::infinity();

  for (int i = 0; i < static_cast<int>(detections.size()); ++i) {
    const float cost = calc_match_cost(detections[i], pred_dx, pred_dy);

    if (!std::isfinite(cost)) {
      continue;
    }

    if (cost < best_cost) {
      second_cost = best_cost;
      second_idx = best_idx;

      best_cost = cost;
      best_idx = i;
    } else if (cost < second_cost) {
      second_cost = cost;
      second_idx = i;
    }
  }

  if (best_idx < 0) {
    return mark_lost();
  }

  const auto& best = detections[best_idx];

  const float dist_to_pred =
      std::sqrt(
          (best.dx - pred_dx) * (best.dx - pred_dx) +
          (best.dy - pred_dy) * (best.dy - pred_dy));

  if (dist_to_pred > static_cast<float>(cfg_.max_match_dist)) {
    return mark_lost();
  }

  if (second_idx >= 0) {
    const float gap = second_cost - best_cost;

    if (gap < static_cast<float>(cfg_.ambiguous_cost_gap)) {
      return mark_ambiguous();
    }
  }

  accept_detection(best, dt_f);
  return true;
}

float MultiLockedTask::calc_match_cost(
    const Detection& det,
    float pred_dx,
    float pred_dy) const
{
  // 队列模式下，当前目标 type 锁死，不允许其他 type 参与匹配
  if (has_current_target_ && det.type != current_target_type_) {
    return std::numeric_limits<float>::infinity();
  }

  // 非队列兜底：track type 不一致也拒绝
  if (track_.valid && det.type != track_.type) {
    return std::numeric_limits<float>::infinity();
  }

  const float pos_dist =
      std::sqrt(
          (det.dx - pred_dx) * (det.dx - pred_dx) +
          (det.dy - pred_dy) * (det.dy - pred_dy));

  float cost = pos_dist;

  // 距离初始队列目标也不要偏太多，防止跳到同 type 远处目标
  if (has_current_target_) {
    const float sdx = det.dx - current_seed_target_.dx;
    const float sdy = det.dy - current_seed_target_.dy;
    const float seed_dist = std::sqrt(sdx * sdx + sdy * sdy);
    cost += 0.25f * seed_dist;
  }

  // score 越大，代价略微降低
  cost -= 0.00001f * det.score;

  return cost;
}

void MultiLockedTask::accept_detection(
    const Detection& det,
    float dt)
{
  const float old_dx = track_.dx;
  const float old_dy = track_.dy;

  const float a_pos = static_cast<float>(cfg_.pos_alpha);
  const float a_vel = static_cast<float>(cfg_.vel_alpha);

  const float new_dx = a_pos * det.dx + (1.0f - a_pos) * track_.dx;
  const float new_dy = a_pos * det.dy + (1.0f - a_pos) * track_.dy;

  const float raw_vx = (new_dx - old_dx) / dt;
  const float raw_vy = (new_dy - old_dy) / dt;

  track_.vx = a_vel * raw_vx + (1.0f - a_vel) * track_.vx;
  track_.vy = a_vel * raw_vy + (1.0f - a_vel) * track_.vy;

  track_.dx = new_dx;
  track_.dy = new_dy;
  track_.score = det.score;
  track_.cost = det.cost;

  track_.hit_count++;
  track_.lost_count = 0;
  track_.ambiguous_count = 0;

  mot_state_ = MotState::LOCKED;
}

bool MultiLockedTask::mark_lost()
{
  track_.lost_count++;

  if (track_.lost_count > cfg_.max_lost_count) {
    mot_state_ = MotState::FAILED;
    track_.valid = false;
    return false;
  }

  mot_state_ = MotState::LOST;
  return true;
}

bool MultiLockedTask::mark_ambiguous()
{
  track_.ambiguous_count++;

  if (track_.ambiguous_count > cfg_.max_ambiguous_count) {
    return mark_lost();
  }

  mot_state_ = MotState::AMBIGUOUS;
  return true;
}

bool MultiLockedTask::has_valid_lock() const
{
  return track_.valid &&
      (mot_state_ == MotState::LOCKED ||
       mot_state_ == MotState::AMBIGUOUS ||
       mot_state_ == MotState::LOST);
}

const char* MultiLockedTask::mot_state_name() const
{
  switch (mot_state_) {
    case MotState::IDLE:
      return "IDLE";
    case MotState::LOCKED:
      return "LOCKED";
    case MotState::AMBIGUOUS:
      return "AMBIGUOUS";
    case MotState::LOST:
      return "LOST";
    case MotState::FAILED:
      return "FAILED";
    default:
      return "UNKNOWN";
  }
}

const char* MultiLockedTask::multi_state_name() const
{
  switch (multi_state_) {
    case MultiState::WAIT_FRAME:
      return "WAIT_FRAME";
    case MultiState::PICK_NEXT:
      return "PICK_NEXT";
    case MultiState::ALIGN_TARGET:
      return "ALIGN_TARGET";
    case MultiState::RETURN_ORIGIN:
      return "RETURN_ORIGIN";
    case MultiState::FINISH:
      return "FINISH";
    default:
      return "UNKNOWN";
  }
}

void MultiLockedTask::image_offset_to_world_delta(
    const Context& ctx,
    float dx_img,
    float dy_img,
    float& dwx,
    float& dwy) const
{
  // 按你之前确认过的方向：
  // 图像右侧 dx_img > 0，需要无人机向机体系 y 负方向修正
  // 图像下侧 dy_img > 0，需要无人机向机体系 x 负方向修正
  const float body_dx = -static_cast<float>(cfg_.k_img_to_meter) * dy_img;
  const float body_dy = static_cast<float>(cfg_.k_img_to_meter) * dx_img;

  const float cy = std::cos(ctx.yaw);
  const float sy = std::sin(ctx.yaw);

  dwx = cy * body_dx - sy * body_dy;
  dwy = sy * body_dx + cy * body_dy;
}

float MultiLockedTask::calc_dynamic_step(float err_m) const
{
  float dynamic_max_step = static_cast<float>(cfg_.max_step_m);

  // 越接近目标，步长越小，避免过冲
  if (err_m < 0.20f) {
    dynamic_max_step = std::min(dynamic_max_step, 0.25f);
  }

  if (err_m < 0.10f) {
    dynamic_max_step = std::min(dynamic_max_step, 0.20f);
  }

  if (err_m < 0.06f) {
    dynamic_max_step = std::min(dynamic_max_step, 0.15f);
  }

  return dynamic_max_step;
}

bool MultiLockedTask::check_error_growing(float err_m)
{
  if (!has_last_err_) {
    has_last_err_ = true;
    last_err_m_ = err_m;
    error_grow_count_ = 0;
    return false;
  }

  if (err_m > last_err_m_ + static_cast<float>(cfg_.error_grow_eps_m)) {
    error_grow_count_++;
  } else {
    error_grow_count_ = 0;
  }

  last_err_m_ = err_m;

  return error_grow_count_ >= cfg_.max_error_grow_count;
}

ITask::Status MultiLockedTask::finish_status() const
{
  // 为了兼容你当前任务链，这里不返回 FAILURE，避免 ITask::Status 没有 FAILURE 导致编译失败。
  return Status::SUCCESS;
}

}  // namespace offboard_core_pkg

