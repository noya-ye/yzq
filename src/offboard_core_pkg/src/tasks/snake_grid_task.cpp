#include "offboard_core_pkg/tasks/snake_grid_task.hpp"

#include <cmath>
#include <algorithm>

SnakeGridTask::SnakeGridTask(rclcpp::Logger logger, const Config& cfg)
: logger_(logger), cfg_(cfg)
{
}

std::string SnakeGridTask::name() const
{
  return "SNAKE_GRID";
}

void SnakeGridTask::onEnter(Context& ctx)
{
  waypoints_.clear();

  index_ = 0;
  hover_elapsed_s_ = 0.0;
  phase_ = Phase::MOVING;

  if (!ctx.pos_valid()) {
    RCLCPP_ERROR(logger_, "[SNAKE] enter failed: local position invalid");
    phase_ = Phase::FAILED;
    return;
  }

  if (cfg_.x_cells <= 0 || cfg_.y_cells <= 0 || cfg_.cell_size <= 0.0) {
    RCLCPP_ERROR(
      logger_,
      "[SNAKE] invalid config: x_cells=%d y_cells=%d cell_size=%.3f",
      cfg_.x_cells,
      cfg_.y_cells,
      cfg_.cell_size);
    phase_ = Phase::FAILED;
    return;
  }

  origin_x_ = ctx.cx();
  origin_y_ = ctx.cy();
  target_z_ = ctx.cz();

  cmd_x_ = origin_x_;
  cmd_y_ = origin_y_;
  cmd_z_ = target_z_;

  yaw_hold_ = std::isfinite(ctx.sp_yaw)
    ? static_cast<double>(ctx.sp_yaw)
    : static_cast<double>(ctx.yaw);

  // 同步到 Context，方便其他任务读取当前网格信息
  ctx.cols = cfg_.x_cells;
  ctx.rows = cfg_.y_cells;
  ctx.cell_size = cfg_.cell_size;
  ctx.origin_dx = origin_x_;
  ctx.origin_dy = origin_y_;
  ctx.current_r = -1;
  ctx.current_c = -1;

  buildWaypoints();

  if (waypoints_.empty()) {
    RCLCPP_WARN(logger_, "[SNAKE] no waypoint generated, finish directly");
    phase_ = Phase::FINISHED;
    return;
  }

  RCLCPP_WARN(
    logger_,
    "[SNAKE] enter: axis=%s stop=%s x_cells=%d y_cells=%d cell=%.2f total=%zu origin=(%.2f %.2f %.2f)",
    cfg_.first_axis == FirstAxis::X_FIRST ? "X_FIRST" : "Y_FIRST",
    cfg_.stop_mode == StopMode::EVERY_CELL ? "EVERY_CELL" : "LINE_END_ONLY",
    cfg_.x_cells,
    cfg_.y_cells,
    cfg_.cell_size,
    waypoints_.size(),
    origin_x_,
    origin_y_,
    target_z_);
}

ITask::Status SnakeGridTask::tick(Context& ctx, double dt_s)
{
  if (phase_ == Phase::FAILED) {
    return ITask::Status::FAILURE;
  }

  if (phase_ == Phase::FINISHED) {
    return ITask::Status::SUCCESS;
  }

  if (index_ >= waypoints_.size()) {
    phase_ = Phase::FINISHED;
    return ITask::Status::SUCCESS;
  }

  const Waypoint& wp = waypoints_[index_];

  ctx.current_c = wp.ix;
  ctx.current_r = wp.iy;

  // 如果短暂丢失定位，不切换目标，只继续发布上一次 setpoint
  if (!ctx.pos_valid()) {
    publishSetpoint(ctx);
    return ITask::Status::RUNNING;
  }

  if (phase_ == Phase::MOVING) {
    moveCommandToward(wp);
    publishSetpoint(ctx);

    if (arrived(ctx, wp)) {
      cmd_x_ = wp.x;
      cmd_y_ = wp.y;
      cmd_z_ = wp.z;
      publishSetpoint(ctx);

      if (wp.hover_after && cfg_.hover_s > 1e-3) {
        hover_elapsed_s_ = 0.0;
        phase_ = Phase::HOVERING;

        RCLCPP_INFO(
          logger_,
          "[SNAKE] arrived cell=(r=%d,c=%d), hover=%.2fs, wp=%zu/%zu",
          wp.iy,
          wp.ix,
          cfg_.hover_s,
          index_ + 1,
          waypoints_.size());
      } else {
        nextWaypoint();
      }
    }

    return ITask::Status::RUNNING;
  }

  if (phase_ == Phase::HOVERING) {
    cmd_x_ = wp.x;
    cmd_y_ = wp.y;
    cmd_z_ = wp.z;

    publishSetpoint(ctx);

    hover_elapsed_s_ += std::max(0.0, dt_s);

    if (hover_elapsed_s_ >= cfg_.hover_s) {
      nextWaypoint();
    }

    return ITask::Status::RUNNING;
  }

  return ITask::Status::RUNNING;
}

void SnakeGridTask::onExit(Context& ctx)
{
  (void)ctx;
  RCLCPP_WARN(logger_, "[SNAKE] exit");
}

void SnakeGridTask::onPause(Context& ctx)
{
  (void)ctx;
  RCLCPP_WARN(
    logger_,
    "[SNAKE] paused at wp=%zu/%zu",
    index_ + 1,
    waypoints_.size());
}

void SnakeGridTask::onResume(Context& ctx)
{
  (void)ctx;
  RCLCPP_WARN(
    logger_,
    "[SNAKE] resumed at wp=%zu/%zu",
    index_ + 1,
    waypoints_.size());

  // 不要重置 index_ / phase_ / cmd_x_ / cmd_y_ / cmd_z_
  // 这样才能从中断前的位置继续蛇形遍历
}

void SnakeGridTask::buildWaypoints()
{
  waypoints_.clear();
  waypoints_.reserve(static_cast<size_t>(cfg_.x_cells * cfg_.y_cells));

  if (cfg_.first_axis == FirstAxis::X_FIRST) {
    buildXFirstWaypoints();
  } else {
    buildYFirstWaypoints();
  }
}

void SnakeGridTask::buildXFirstWaypoints()
{
  /*
   * X_FIRST 示例：
   *
   * y=2:  → → →
   * y=1:  ← ← ←
   * y=0:  → → →
   *
   * 顺序：
   * (0,0)->(1,0)->(2,0)
   *      ->(2,1)->(1,1)->(0,1)
   *      ->(0,2)->(1,2)->(2,2)
   */

  for (int iy = 0; iy < cfg_.y_cells; ++iy) {
    const bool reverse = (iy % 2 == 1);

    for (int k = 0; k < cfg_.x_cells; ++k) {
      const int ix = reverse ? (cfg_.x_cells - 1 - k) : k;
      const bool line_end = (k == cfg_.x_cells - 1);

      pushWaypoint(ix, iy, line_end);
    }
  }
}

void SnakeGridTask::buildYFirstWaypoints()
{
  /*
   * Y_FIRST 示例：
   *
   * x=0:  ↑ ↑ ↑
   * x=1:  ↓ ↓ ↓
   * x=2:  ↑ ↑ ↑
   *
   * 顺序：
   * (0,0)->(0,1)->(0,2)
   *      ->(1,2)->(1,1)->(1,0)
   *      ->(2,0)->(2,1)->(2,2)
   */

  for (int ix = 0; ix < cfg_.x_cells; ++ix) {
    const bool reverse = (ix % 2 == 1);

    for (int k = 0; k < cfg_.y_cells; ++k) {
      const int iy = reverse ? (cfg_.y_cells - 1 - k) : k;
      const bool line_end = (k == cfg_.y_cells - 1);

      pushWaypoint(ix, iy, line_end);
    }
  }
}

void SnakeGridTask::pushWaypoint(int ix, int iy, bool line_end)
{
  if (!cfg_.include_start_cell && ix == 0 && iy == 0) {
    return;
  }

  Waypoint wp;

  wp.ix = ix;
  wp.iy = iy;

  wp.x = origin_x_ + static_cast<double>(cfg_.x_sign) * ix * cfg_.cell_size;
  wp.y = origin_y_ + static_cast<double>(cfg_.y_sign) * iy * cfg_.cell_size;
  wp.z = target_z_;

  if (cfg_.stop_mode == StopMode::EVERY_CELL) {
    wp.hover_after = true;
  } else {
    wp.hover_after = line_end;
  }

  waypoints_.push_back(wp);
}

void SnakeGridTask::moveCommandToward(const Waypoint& wp)
{
  const double dx = wp.x - cmd_x_;
  const double dy = wp.y - cmd_y_;
  const double dz = wp.z - cmd_z_;

  const double dist = std::sqrt(dx * dx + dy * dy + dz * dz);

  if (dist < 1e-6) {
    cmd_x_ = wp.x;
    cmd_y_ = wp.y;
    cmd_z_ = wp.z;
    return;
  }

  const double step = std::max(0.0, cfg_.max_step_m);

  if (step < 1e-6 || dist <= step) {
    cmd_x_ = wp.x;
    cmd_y_ = wp.y;
    cmd_z_ = wp.z;
    return;
  }

  const double ratio = step / dist;

  cmd_x_ += dx * ratio;
  cmd_y_ += dy * ratio;
  cmd_z_ += dz * ratio;
}

bool SnakeGridTask::arrived(const Context& ctx, const Waypoint& wp) const
{
  const double dx = static_cast<double>(ctx.cx()) - wp.x;
  const double dy = static_cast<double>(ctx.cy()) - wp.y;
  const double dz = static_cast<double>(ctx.cz()) - wp.z;

  const double err_xy = std::sqrt(dx * dx + dy * dy);
  const double err_z = std::fabs(dz);

  return err_xy < cfg_.arrive_xy_m &&
         err_z  < cfg_.arrive_z_m;
}

void SnakeGridTask::publishSetpoint(Context& ctx)
{
  ctx.use_vel_ctrl = false;

  ctx.sp_x = static_cast<float>(cmd_x_);
  ctx.sp_y = static_cast<float>(cmd_y_);
  ctx.sp_z = static_cast<float>(cmd_z_);
  ctx.sp_yaw = static_cast<float>(yaw_hold_);

  ctx.sp_vx = 0.0f;
  ctx.sp_vy = 0.0f;
  ctx.sp_vz = 0.0f;

  ctx.sp_ax = 0.0f;
  ctx.sp_ay = 0.0f;
  ctx.sp_az = 0.0f;
}

void SnakeGridTask::nextWaypoint()
{
  ++index_;
  hover_elapsed_s_ = 0.0;
  phase_ = Phase::MOVING;

  if (index_ >= waypoints_.size()) {
    phase_ = Phase::FINISHED;
    RCLCPP_WARN(logger_, "[SNAKE] finished all waypoints");
  }
}