#include "offboard_core_pkg/tasks/snake_grid_task.hpp"
#include "offboard_core_pkg/planners/a_star_planner.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <utility>
#include <vector>

namespace
{
std::atomic<std::uint32_t> g_next_plan_id{1U};
}  // namespace

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
  route_cells_.clear();
  index_ = 0;
  hover_elapsed_s_ = 0.0;
  phase_ = Phase::MOVING;
  plan_ready_ = false;
  plan_id_ = 0;

  if (!ctx.pos_valid()) {
    RCLCPP_ERROR(logger_, "[SNAKE] enter failed: local position invalid");
    phase_ = Phase::FAILED;
    return;
  }

  if (cfg_.x_cells <= 0 || cfg_.y_cells <= 0 || cfg_.cell_size <= 0.0) {
    RCLCPP_ERROR(logger_, "[SNAKE] invalid grid: x=%d y=%d cell=%.3f",
      cfg_.x_cells, cfg_.y_cells, cfg_.cell_size);
    phase_ = Phase::FAILED;
    return;
  }

  if ((cfg_.x_sign != 1 && cfg_.x_sign != -1) ||
      (cfg_.y_sign != 1 && cfg_.y_sign != -1)) {
    RCLCPP_ERROR(logger_, "[SNAKE] invalid axis sign: x=%d y=%d", cfg_.x_sign, cfg_.y_sign);
    phase_ = Phase::FAILED;
    return;
  }

  if (cfg_.hover_s < 0.0 || cfg_.max_step_m < 0.0 ||
      cfg_.arrive_xy_m <= 0.0 || cfg_.arrive_z_m <= 0.0) {
    RCLCPP_ERROR(logger_, "[SNAKE] invalid motion: hover=%.3f step=%.3f xy=%.3f z=%.3f",
      cfg_.hover_s, cfg_.max_step_m, cfg_.arrive_xy_m, cfg_.arrive_z_m);
    phase_ = Phase::FAILED;
    return;
  }

  origin_x_ = static_cast<double>(ctx.cx());
  origin_y_ = static_cast<double>(ctx.cy());
  target_z_ = static_cast<double>(ctx.cz());
  cmd_x_ = origin_x_;
  cmd_y_ = origin_y_;
  cmd_z_ = target_z_;

  if (std::isfinite(ctx.sp_yaw)) {
    yaw_hold_ = static_cast<double>(ctx.sp_yaw);
  } else if (std::isfinite(ctx.yaw)) {
    yaw_hold_ = static_cast<double>(ctx.yaw);
  } else {
    yaw_hold_ = 0.0;
  }

  ctx.cols = cfg_.x_cells;
  ctx.rows = cfg_.y_cells;
  ctx.cell_size = cfg_.cell_size;
  ctx.origin_dx = origin_x_;
  ctx.origin_dy = origin_y_;
  ctx.current_r = -1;
  ctx.current_c = -1;
  ctx.vision_down_enable=true;

  buildWaypoints();
  if (phase_ == Phase::FAILED) {
    return;
  }

  rebuildRouteCells();
  plan_id_ = g_next_plan_id.fetch_add(1U);
  plan_ready_ = true;

  if (waypoints_.empty()) {
    RCLCPP_WARN(logger_, "[SNAKE] plan=%u has no waypoint, finish directly",
      static_cast<unsigned>(plan_id_));
    phase_ = Phase::FINISHED;
    return;
  }

  RCLCPP_INFO(logger_,
    "[SNAKE] start plan=%u axis=%s stop=%s grid=%dx%d cell=%.2f wp=%zu obs=%zu "
    "origin=(%.2f %.2f %.2f)",
    static_cast<unsigned>(plan_id_),
    cfg_.first_axis == FirstAxis::X_FIRST ? "X_FIRST" : "Y_FIRST",
    cfg_.stop_mode == StopMode::EVERY_CELL ? "EVERY_CELL" : "LINE_END_ONLY",
    cfg_.x_cells, cfg_.y_cells, cfg_.cell_size, waypoints_.size(),
    cfg_.obstacle_cells.size(), origin_x_, origin_y_, target_z_);
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


  ctx.vision_down_enable=true;

  const Waypoint& wp = waypoints_[index_];
  ctx.current_c = wp.ix;
  ctx.current_r = wp.iy;

  if (!ctx.pos_valid()) {
    publishSetpoint(ctx);
    return ITask::Status::RUNNING;
  }

  if (phase_ == Phase::MOVING) {
  moveCommandToward(wp, dt_s);
  publishSetpoint(ctx);

  /*
   * 普通直线中间点不等待飞机实际到达。
   * 指令点到达该网格后立即推进下一格，让PX4沿整行连续飞行。
   */
  if (!wp.hover_after) {
    const double dx = wp.x - cmd_x_;
    const double dy = wp.y - cmd_y_;
    const double dz = wp.z - cmd_z_;

    if (std::sqrt(dx * dx + dy * dy + dz * dz) <= 1e-4) {
      nextWaypoint();
    }

    return ITask::Status::RUNNING;
  }

  /*
   * 行尾、A*转折点需要等待飞机实际到达。
   */
  if (!arrived(ctx, wp)) {
    return ITask::Status::RUNNING;
  }

  cmd_x_ = wp.x;
  cmd_y_ = wp.y;
  cmd_z_ = wp.z;
  publishSetpoint(ctx);

  if (cfg_.hover_s > 1e-3) {
    hover_elapsed_s_ = 0.0;
    phase_ = Phase::HOVERING;
  } else {
    nextWaypoint();
  }

  return ITask::Status::RUNNING;
}
void SnakeGridTask::onExit(Context& ctx)
{
  (void)ctx;
    ctx.vision_down_enable=false;
  if (failed()) {
    RCLCPP_ERROR(logger_, "[SNAKE] exit failed: plan=%u wp=%zu/%zu",
      static_cast<unsigned>(plan_id_), currentIndex(), totalWaypoints());
  } else if (!finished()) {
    RCLCPP_WARN(logger_, "[SNAKE] exit interrupted: plan=%u cell=%s wp=%zu/%zu",
      static_cast<unsigned>(plan_id_), currentCell().c_str(),
      displayWaypointIndex(), waypoints_.size());
  }
}

void SnakeGridTask::onPause(Context& ctx)
{
  if (ctx.pos_valid()) {
    cmd_x_ = static_cast<double>(ctx.cx());
    cmd_y_ = static_cast<double>(ctx.cy());
    cmd_z_ = static_cast<double>(ctx.cz());
    publishSetpoint(ctx);
  }

  hover_elapsed_s_ = 0.0;
  RCLCPP_WARN(logger_, "[SNAKE] paused: plan=%u cell=%s wp=%zu/%zu",
    static_cast<unsigned>(plan_id_), currentCell().c_str(),
    displayWaypointIndex(), waypoints_.size());
}

void SnakeGridTask::onResume(Context& ctx)
{
  if (phase_ == Phase::FAILED || phase_ == Phase::FINISHED || index_ >= waypoints_.size()) {
    return;
  }

  if (ctx.pos_valid()) {
    cmd_x_ = static_cast<double>(ctx.cx());
    cmd_y_ = static_cast<double>(ctx.cy());
    cmd_z_ = static_cast<double>(ctx.cz());
  }

  phase_ = Phase::MOVING;
  hover_elapsed_s_ = 0.0;
  RCLCPP_WARN(logger_, "[SNAKE] resumed: plan=%u cell=%s wp=%zu/%zu",
    static_cast<unsigned>(plan_id_), currentCell().c_str(),
    displayWaypointIndex(), waypoints_.size());
}

void SnakeGridTask::buildWaypoints()
{
  using AStarPlanner = offboard_core_pkg::AStarPlanner;

  waypoints_.clear();
  waypoints_.reserve(static_cast<std::size_t>(cfg_.x_cells * cfg_.y_cells));
  if (cfg_.first_axis == FirstAxis::X_FIRST) {
    buildXFirstWaypoints();
  } else {
    buildYFirstWaypoints();
  }

  std::vector<Waypoint> original_snake = std::move(waypoints_);
  waypoints_.clear();

  const int total_cells = cfg_.x_cells * cfg_.y_cells;
  std::vector<std::uint8_t> blocked(static_cast<std::size_t>(total_cells), 0U);
  std::vector<AStarPlanner::Cell> planner_obstacles;
  planner_obstacles.reserve(cfg_.obstacle_cells.size());

  const auto cellIndex = [this](int ix, int iy) {
    return iy * cfg_.x_cells + ix;
  };

  for (const ObstacleCell& obstacle : cfg_.obstacle_cells) {
    if (obstacle.ix < 0 || obstacle.ix >= cfg_.x_cells ||
        obstacle.iy < 0 || obstacle.iy >= cfg_.y_cells) {
      RCLCPP_ERROR(logger_, "[SNAKE] obstacle outside grid: r=%d c=%d grid=%dx%d",
        obstacle.iy, obstacle.ix, cfg_.y_cells, cfg_.x_cells);
      phase_ = Phase::FAILED;
      waypoints_.clear();
      return;
    }

    const auto blocked_index = static_cast<std::size_t>(cellIndex(obstacle.ix, obstacle.iy));
    if (blocked[blocked_index] != 0U) {
      continue;
    }

    blocked[blocked_index] = 1U;
    planner_obstacles.push_back({obstacle.ix, obstacle.iy});
  }

  const auto isBlocked = [&](int ix, int iy) {
    return blocked[static_cast<std::size_t>(cellIndex(ix, iy))] != 0U;
  };

  const bool start_is_line_end = cfg_.first_axis == FirstAxis::X_FIRST
    ? cfg_.x_cells == 1
    : cfg_.y_cells == 1;
  Waypoint start_wp = makeWaypoint(0, 0,
    cfg_.stop_mode == StopMode::EVERY_CELL || start_is_line_end);

  if (isBlocked(0, 0)) {
    RCLCPP_ERROR(logger_, "[SNAKE] start cell A1B1 cannot be an obstacle");
    phase_ = Phase::FAILED;
    waypoints_.clear();
    return;
  }

  std::vector<Waypoint> snake_sequence;
  snake_sequence.reserve(original_snake.size() + 1);
  snake_sequence.push_back(start_wp);
  for (const Waypoint& wp : original_snake) {
    if (wp.ix == 0 && wp.iy == 0) {
      snake_sequence.front() = wp;
    } else {
      snake_sequence.push_back(wp);
    }
  }

  const auto appendWaypoint = [&](const Waypoint& wp) {
    if (!waypoints_.empty() && waypoints_.back().ix == wp.ix && waypoints_.back().iy == wp.iy) {
      waypoints_.back().hover_after = waypoints_.back().hover_after || wp.hover_after;
      return;
    }
    waypoints_.push_back(wp);
  };

  std::size_t current_index = 0;
  if (cfg_.include_start_cell) {
    appendWaypoint(snake_sequence.front());
  }

  while (current_index + 1 < snake_sequence.size()) {
    const std::size_t next_index = current_index + 1;
    const Waypoint& current_wp = snake_sequence[current_index];
    const Waypoint& next_wp = snake_sequence[next_index];

    if (!isBlocked(next_wp.ix, next_wp.iy)) {
      appendWaypoint(next_wp);
      current_index = next_index;
      continue;
    }

    Waypoint stop_wp = current_wp;
    stop_wp.hover_after = true;
    appendWaypoint(stop_wp);

    std::size_t resume_index = next_index + 1;
    while (resume_index < snake_sequence.size() &&
           isBlocked(snake_sequence[resume_index].ix, snake_sequence[resume_index].iy)) {
      ++resume_index;
    }

    if (resume_index >= snake_sequence.size()) {
      RCLCPP_WARN(logger_, "[SNAKE] remaining cells blocked, finish at %s",
        cellToName(current_wp.ix, current_wp.iy).c_str());
      break;
    }

    const Waypoint& resume_wp = snake_sequence[resume_index];
    const auto result = AStarPlanner::plan(
      cfg_.x_cells, cfg_.y_cells, cfg_.cell_size,
      {current_wp.ix, current_wp.iy}, {resume_wp.ix, resume_wp.iy}, planner_obstacles);

    if (!result.success) {
      RCLCPP_ERROR(logger_, "[SNAKE] A* failed: %s -> %s, reason=%s",
        cellToName(current_wp.ix, current_wp.iy).c_str(),
        cellToName(resume_wp.ix, resume_wp.iy).c_str(), result.message.c_str());
      phase_ = Phase::FAILED;
      waypoints_.clear();
      return;
    }

    if (result.path.empty()) {
      RCLCPP_ERROR(logger_, "[SNAKE] A* returned empty path: %s -> %s",
        cellToName(current_wp.ix, current_wp.iy).c_str(),
        cellToName(resume_wp.ix, resume_wp.iy).c_str());
      phase_ = Phase::FAILED;
      waypoints_.clear();
      return;
    }

    RCLCPP_WARN(logger_, "[SNAKE] detour: blocked=%s, %s -> %s, points=%zu",
      cellToName(next_wp.ix, next_wp.iy).c_str(),
      cellToName(current_wp.ix, current_wp.iy).c_str(),
      cellToName(resume_wp.ix, resume_wp.iy).c_str(), result.path.size());

    for (std::size_t i = 1; i < result.path.size(); ++i) {
      const auto& path_point = result.path[i];
      Waypoint detour_wp = makeWaypoint(
        path_point.cell.x, path_point.cell.y, path_point.stop_after);
      if (detour_wp.ix == resume_wp.ix && detour_wp.iy == resume_wp.iy) {
        detour_wp.hover_after = detour_wp.hover_after || resume_wp.hover_after;
      }
      appendWaypoint(detour_wp);
    }

    current_index = resume_index;
  }
}

void SnakeGridTask::buildXFirstWaypoints()
{
  for (int iy = 0; iy < cfg_.y_cells; ++iy) {
    const bool reverse = iy % 2 == 1;
    for (int k = 0; k < cfg_.x_cells; ++k) {
      const int ix = reverse ? cfg_.x_cells - 1 - k : k;
      pushWaypoint(ix, iy, k == cfg_.x_cells - 1);
    }
  }
}

void SnakeGridTask::buildYFirstWaypoints()
{
  for (int ix = 0; ix < cfg_.x_cells; ++ix) {
    const bool reverse = ix % 2 == 1;
    for (int k = 0; k < cfg_.y_cells; ++k) {
      const int iy = reverse ? cfg_.y_cells - 1 - k : k;
      pushWaypoint(ix, iy, k == cfg_.y_cells - 1);
    }
  }
}

void SnakeGridTask::pushWaypoint(int ix, int iy, bool line_end)
{
  if (!cfg_.include_start_cell && ix == 0 && iy == 0) {
    return;
  }

  const bool hover_after = cfg_.stop_mode == StopMode::EVERY_CELL || line_end;
  waypoints_.push_back(makeWaypoint(ix, iy, hover_after));
}

SnakeGridTask::Waypoint SnakeGridTask::makeWaypoint(int ix, int iy, bool hover_after) const
{
  Waypoint wp;
  wp.ix = ix;
  wp.iy = iy;
  wp.x = origin_x_ + static_cast<double>(cfg_.x_sign) * ix * cfg_.cell_size;
  wp.y = origin_y_ + static_cast<double>(cfg_.y_sign) * iy * cfg_.cell_size;
  wp.z = target_z_;
  wp.hover_after = hover_after;
  return wp;
}

void SnakeGridTask::rebuildRouteCells()
{
  route_cells_.clear();
  route_cells_.reserve(waypoints_.size());
  for (const Waypoint& wp : waypoints_) {
    route_cells_.push_back(cellToName(wp.ix, wp.iy));
  }
}

std::string SnakeGridTask::cellToName(
  int ix,
  int iy) const
{
  return
    "A" + std::to_string(cfg_.x_cells - ix) +
    "B" + std::to_string(iy + 1);
}

void SnakeGridTask::moveCommandToward(
  const Waypoint& wp,
  double dt_s)
{
  const double dx = wp.x - cmd_x_;
  const double dy = wp.y - cmd_y_;
  const double dz = wp.z - cmd_z_;
  const double distance =
    std::sqrt(dx * dx + dy * dy + dz * dz);

  if (distance < 1e-6) {
    cmd_x_ = wp.x;
    cmd_y_ = wp.y;
    cmd_z_ = wp.z;
    return;
  }

  double safe_dt = dt_s;

  if (!std::isfinite(safe_dt) ||
      safe_dt <= 0.0) {
    safe_dt = 0.05;
  }

  safe_dt = std::min(safe_dt, 0.10);

  /*
   * 兼容现有参数：
   * 原max_step_m是50ms周期下的步长。
   * 换算成速度后，再乘实际dt。
   */
  const double max_speed_mps =
    std::max(0.0, cfg_.max_step_m) / 0.05;

  const double step =
    max_speed_mps * safe_dt;

  if (step < 1e-6 ||
      distance <= step) {
    cmd_x_ = wp.x;
    cmd_y_ = wp.y;
    cmd_z_ = wp.z;
    return;
  }

  const double ratio = step / distance;

  cmd_x_ += dx * ratio;
  cmd_y_ += dy * ratio;
  cmd_z_ += dz * ratio;
}

bool SnakeGridTask::arrived(const Context& ctx, const Waypoint& wp) const
{
  const double dx = static_cast<double>(ctx.cx()) - wp.x;
  const double dy = static_cast<double>(ctx.cy()) - wp.y;
  const double dz = static_cast<double>(ctx.cz()) - wp.z;
  return std::sqrt(dx * dx + dy * dy) <= cfg_.arrive_xy_m &&
         std::fabs(dz) <= cfg_.arrive_z_m;
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
    RCLCPP_WARN(logger_, "[SNAKE] finished: plan=%u total=%zu",
      static_cast<unsigned>(plan_id_), waypoints_.size());
  }
}

std::size_t SnakeGridTask::displayWaypointIndex() const
{
  if (waypoints_.empty()) {
    return 0;
  }
  return index_ >= waypoints_.size() ? waypoints_.size() : index_ + 1;
}

std::uint32_t SnakeGridTask::planId() const
{
  return plan_id_;
}

bool SnakeGridTask::planReady() const
{
  return plan_ready_;
}

const std::vector<std::string>& SnakeGridTask::routeCells() const
{
  return route_cells_;
}

std::size_t SnakeGridTask::currentIndex() const
{
  return std::min(index_, route_cells_.size());
}

std::size_t SnakeGridTask::totalWaypoints() const
{
  return route_cells_.size();
}

std::string SnakeGridTask::currentCell() const
{
  if (route_cells_.empty()) {
    return "";
  }
  return index_ < route_cells_.size() ? route_cells_[index_] : route_cells_.back();
}

bool SnakeGridTask::finished() const
{
  return phase_ == Phase::FINISHED;
}

bool SnakeGridTask::failed() const
{
  return phase_ == Phase::FAILED;
}
