#include "offboard_core_pkg/tasks/snake_grid_task.hpp"
#include "offboard_core_pkg/planners/a_star_planner.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <utility>
#include <vector>

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

  if (phase_ == Phase::FAILED) {
    return;
  }

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
  if (phase_ == Phase::FAILED ||
      phase_ == Phase::FINISHED ||
      index_ >= waypoints_.size()) {
    return;
  }

  // 中断任务可能已经移动飞机。
  // 以恢复时的实际位置作为新的平滑指令起点。
  if (ctx.pos_valid()) {
    cmd_x_ = ctx.cx();
    cmd_y_ = ctx.cy();
    cmd_z_ = ctx.cz();
  }

  // 仍然继续中断前的 waypoint，不改变 index_
  phase_ = Phase::MOVING;

  // 如果中断发生在悬停阶段，恢复后重新到达该点并重新悬停。
  hover_elapsed_s_ = 0.0;

  RCLCPP_WARN(
    logger_,
    "[SNAKE] resumed at wp=%zu/%zu",
    index_ + 1,
    waypoints_.size());
}

void SnakeGridTask::buildWaypoints()
{
  using AStarPlanner = offboard_core_pkg::AStarPlanner;

  /*
   * 第一步：仍然使用原来的函数生成完整蛇形路径。
   */
  waypoints_.clear();
  waypoints_.reserve(static_cast<std::size_t>(cfg_.x_cells * cfg_.y_cells));

  if (cfg_.first_axis == FirstAxis::X_FIRST) {
    buildXFirstWaypoints();
  } else {
    buildYFirstWaypoints();
  }

  /*
   * 保存原始蛇形路径，然后重新生成带 A* 绕障的最终路径。
   */
  std::vector<Waypoint> original_snake = std::move(waypoints_);
  waypoints_.clear();

  /*
   * 建立障碍栅格表，同时转换成 AStarPlanner 使用的格式。
   */
  const int total_cells = cfg_.x_cells * cfg_.y_cells;
  std::vector<std::uint8_t> blocked(
    static_cast<std::size_t>(total_cells), 0U);

  std::vector<AStarPlanner::Cell> planner_obstacles;
  planner_obstacles.reserve(cfg_.obstacle_cells.size());

  const auto cellIndex = [this](int ix, int iy) {
    return iy * cfg_.x_cells + ix;
  };

  for (const auto& obstacle : cfg_.obstacle_cells) {
    if (obstacle.ix < 0 || obstacle.ix >= cfg_.x_cells ||
        obstacle.iy < 0 || obstacle.iy >= cfg_.y_cells) {
      RCLCPP_ERROR(
        logger_,
        "[SNAKE] obstacle outside grid: cell=(r=%d,c=%d), grid=%dx%d",
        obstacle.iy,
        obstacle.ix,
        cfg_.y_cells,
        cfg_.x_cells);

      phase_ = Phase::FAILED;
      waypoints_.clear();
      return;
    }

    blocked[static_cast<std::size_t>(
      cellIndex(obstacle.ix, obstacle.iy))] = 1U;

    planner_obstacles.push_back(
      AStarPlanner::Cell{obstacle.ix, obstacle.iy});
  }

  const auto isBlocked = [&](int ix, int iy) {
    return blocked[static_cast<std::size_t>(
      cellIndex(ix, iy))] != 0U;
  };

  /*
   * 当前飞机所在位置对应蛇形起点 (0,0)。
   *
   * 即使 include_start_cell=false，也要在内部序列中保留起点，
   * 因为遇到第一个障碍时，A* 需要从 (0,0) 开始规划。
   */
  Waypoint start_wp;
  start_wp.ix = 0;
  start_wp.iy = 0;
  start_wp.x = origin_x_;
  start_wp.y = origin_y_;
  start_wp.z = target_z_;

  const bool start_is_line_end =
    cfg_.first_axis == FirstAxis::X_FIRST
      ? cfg_.x_cells == 1
      : cfg_.y_cells == 1;

  start_wp.hover_after =
    cfg_.stop_mode == StopMode::EVERY_CELL ||
    start_is_line_end;

  if (isBlocked(0, 0)) {
    RCLCPP_ERROR(logger_, "[SNAKE] start cell (r=0,c=0) cannot be an obstacle");
    phase_ = Phase::FAILED;
    waypoints_.clear();
    return;
  }

  /*
   * snake_sequence 一定包含起点。
   *
   * include_start_cell 只决定最终 waypoints_ 是否正常包含起点，
   * 不影响内部障碍判断。
   */
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

  /*
   * 添加最终航点。
   *
   * 如果连续添加同一个格子，则合并 hover_after，
   * 避免出现完全重复的 waypoint。
   */
  const auto appendWaypoint = [&](const Waypoint& wp) {
    if (!waypoints_.empty() &&
        waypoints_.back().ix == wp.ix &&
        waypoints_.back().iy == wp.iy) {
      waypoints_.back().hover_after =
        waypoints_.back().hover_after || wp.hover_after;
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

    /*
     * 下一个蛇形格不是障碍，保持原来的蛇形遍历。
     */
    if (!isBlocked(next_wp.ix, next_wp.iy)) {
      appendWaypoint(next_wp);
      current_index = next_index;
      continue;
    }

    /*
     * 下一个蛇形格是障碍：
     *
     * 1. 当前格设置为悬停点；
     * 2. 查找障碍之后第一个非障碍蛇形格；
     * 3. 使用 A* 从当前格绕到该格。
     */
    Waypoint stop_wp = current_wp;
    stop_wp.hover_after = true;
    appendWaypoint(stop_wp);

    RCLCPP_WARN(
      logger_,
      "[SNAKE] next cell blocked: current=(r=%d,c=%d), blocked=(r=%d,c=%d), hover before A*",
      current_wp.iy,
      current_wp.ix,
      next_wp.iy,
      next_wp.ix);

    std::size_t resume_index = next_index + 1;

    while (resume_index < snake_sequence.size() &&
           isBlocked(
             snake_sequence[resume_index].ix,
             snake_sequence[resume_index].iy)) {
      ++resume_index;
    }

    /*
     * 后续所有蛇形格都是障碍，则在当前可用格结束。
     */
    if (resume_index >= snake_sequence.size()) {
      RCLCPP_WARN(
        logger_,
        "[SNAKE] all remaining snake cells are blocked, finish at cell=(r=%d,c=%d)",
        current_wp.iy,
        current_wp.ix);
      break;
    }

    const Waypoint& resume_wp = snake_sequence[resume_index];

    const auto result = AStarPlanner::plan(
      cfg_.x_cells,
      cfg_.y_cells,
      cfg_.cell_size,
      AStarPlanner::Cell{current_wp.ix, current_wp.iy},
      AStarPlanner::Cell{resume_wp.ix, resume_wp.iy},
      planner_obstacles);

    if (!result.success) {
      RCLCPP_ERROR(
        logger_,
        "[SNAKE] A* failed: from=(r=%d,c=%d) to=(r=%d,c=%d), reason=%s",
        current_wp.iy,
        current_wp.ix,
        resume_wp.iy,
        resume_wp.ix,
        result.message.c_str());

      phase_ = Phase::FAILED;
      waypoints_.clear();
      return;
    }

    RCLCPP_WARN(
      logger_,
      "[SNAKE] A* detour: from=(r=%d,c=%d) to=(r=%d,c=%d), points=%zu",
      current_wp.iy,
      current_wp.ix,
      resume_wp.iy,
      resume_wp.ix,
      result.path.size());

    /*
     * result.path[0] 是当前格，已经加入 waypoints_，
     * 因此从第 1 个点开始插入。
     */
    for (std::size_t i = 1; i < result.path.size(); ++i) {
      const auto& path_point = result.path[i];

      Waypoint detour_wp;
      detour_wp.ix = path_point.cell.x;
      detour_wp.iy = path_point.cell.y;

      detour_wp.x =
        origin_x_ +
        static_cast<double>(cfg_.x_sign) *
        static_cast<double>(detour_wp.ix) *
        cfg_.cell_size;

      detour_wp.y =
        origin_y_ +
        static_cast<double>(cfg_.y_sign) *
        static_cast<double>(detour_wp.iy) *
        cfg_.cell_size;

      detour_wp.z = target_z_;

      /*
       * A* 转折点和 A* 终点会悬停。
       * 这样能避免飞机在转弯时由于惯性飞飘。
       */
      detour_wp.hover_after = path_point.stop_after;

      /*
       * 如果这个点是恢复蛇形遍历的目标点，
       * 同时保留它原来的蛇形悬停要求。
       */
      if (detour_wp.ix == resume_wp.ix &&
          detour_wp.iy == resume_wp.iy) {
        detour_wp.hover_after =
          detour_wp.hover_after || resume_wp.hover_after;
      }

      appendWaypoint(detour_wp);
    }

    /*
     * A* 已经到达 resume_index。
     * 后续从该点继续原来的蛇形遍历。
     */
    current_index = resume_index;
  }

  RCLCPP_WARN(
    logger_,
    "[SNAKE] path generated: snake=%zu final=%zu obstacles=%zu",
    snake_sequence.size(),
    waypoints_.size(),
    cfg_.obstacle_cells.size());
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
    const bool reverse = iy % 2 == 1;

    for (int k = 0; k < cfg_.x_cells; ++k) {
      const int ix = reverse ? cfg_.x_cells - 1 - k : k;
      const bool line_end = k == cfg_.x_cells - 1;

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
    const bool reverse = ix % 2 == 1;

    for (int k = 0; k < cfg_.y_cells; ++k) {
      const int iy = reverse ? cfg_.y_cells - 1 - k : k;
      const bool line_end = k == cfg_.y_cells - 1;

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
         err_z < cfg_.arrive_z_m;
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