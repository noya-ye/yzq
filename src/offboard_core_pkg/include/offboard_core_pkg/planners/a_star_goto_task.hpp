// 配置示例：

// using AStarGotoTask = offboard_core_pkg::AStarGotoTask;
// using Cell = AStarGotoTask::Cell;

// std::vector<Cell> obstacles = {
//   {1, 1},
//   {2, 1},
//   {2, 2}
// };

// sched_.add(std::make_unique<AStarGotoTask>(
//   get_logger(),
//   Cell{0, 0},   // 起点栅格
//   Cell{4, 4},   // 终点栅格
//   5,            // x 方向方格数量
//   5,            // y 方向方格数量
//   0.8,          // 每个方格边长，单位 m
//   0.03,         // 每次 tick 最大 setpoint 步长，单位 m
//   obstacles));

// 也可以通过 Config 设置更多参数：

// AStarGotoTask::Config astar_cfg;

// astar_cfg.cols = 5;
// astar_cfg.rows = 5;
// astar_cfg.cell_size = 0.8;

// astar_cfg.start_cell = {0, 0};
// astar_cfg.goal_cell = {4, 4};

// astar_cfg.obstacle_cells = {
//   {1, 1},
//   {2, 1},
//   {2, 2}
// };

// astar_cfg.max_step_m = 0.03;
// astar_cfg.arrive_xy_m = 0.08;
// astar_cfg.arrive_z_m = 0.10;
// astar_cfg.hover_s = 0.40;

// astar_cfg.x_sign = 1;
// astar_cfg.y_sign = 1;

// sched_.add(std::make_unique<AStarGotoTask>(
//   get_logger(),
//   astar_cfg));

#pragma once

#include "offboard_core_pkg/context.hpp"
#include "offboard_core_pkg/itask.hpp"
#include "a_star_planner.hpp"
#include "offboard_core_pkg/planners/ego_vel_planner.hpp"

#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

namespace offboard_core_pkg
{

/*
 * A* 栅格点到点飞行任务。
 *
 * 使用约定：
 *   1. start_cell、goal_cell 和 obstacle_cells 均为栅格索引。
 *   2. 进入任务瞬间的飞机当前位置，视为 start_cell 的中心。
 *   3. 飞行高度保持为进入任务瞬间的 local position z。
 *   4. 路径使用四邻域，只沿栅格 x/y 方向飞行。
 *   5. 每个路径点均为栅格中心，转折点和终点会悬停。
 *
 * 世界坐标换算：
 *   map_origin_x = enter_x - x_sign * start_cell.x * cell_size
 *   map_origin_y = enter_y - y_sign * start_cell.y * cell_size
 *
 *   world_x = map_origin_x + x_sign * cell.x * cell_size
 *   world_y = map_origin_y + y_sign * cell.y * cell_size
 */
class AStarGotoTask : public ITask
{
public:
  using Cell = AStarPlanner::Cell;

  struct Config
  {
    int cols{1};
    int rows{1};
    double cell_size{1.0};

    Cell start_cell{0, 0};
    Cell goal_cell{0, 0};
    std::vector<Cell> obstacle_cells;

    // 每次 tick 允许位置 setpoint 前进的最大距离。
    // 设为 0 时直接发布下一个路径点。
    double max_step_m{0.05};

    double arrive_xy_m{0.10};
    double arrive_z_m{0.10};

    // 转折点和终点的悬停时间。
    double hover_s{0.40};

    // 栅格坐标到世界坐标的方向映射。
    int x_sign{1};
    int y_sign{1};
  };

  struct ExecutablePoint
  {
    int ix{0};
    int iy{0};

    double x{0.0};
    double y{0.0};
    double z{0.0};

    bool is_turn{false};
    bool hover_after{false};
  };

  AStarGotoTask(rclcpp::Logger logger, const Config& cfg)
  : logger_(logger), cfg_(cfg)
  {
  }

  AStarGotoTask(
    rclcpp::Logger logger,
    const Cell& start_cell,
    const Cell& goal_cell,
    int cols,
    int rows,
    double cell_size,
    double max_step_m,
    const std::vector<Cell>& obstacle_cells = {})
  : logger_(logger)
  {
    cfg_.start_cell = start_cell;
    cfg_.goal_cell = goal_cell;
    cfg_.cols = cols;
    cfg_.rows = rows;
    cfg_.cell_size = cell_size;
    cfg_.max_step_m = max_step_m;
    cfg_.obstacle_cells = obstacle_cells;
  }

  std::string name() const override
  {
    return "A_STAR_GOTO";
  }

  void onEnter(Context& ctx) override
  {
    path_.clear();
    index_ = 0;
    hover_elapsed_s_ = 0.0;
    phase_ = Phase::MOVING;

    if (!ctx.pos_valid()) {
      fail("local position invalid");
      return;
    }

    if (!validateConfig()) {
      return;
    }

    target_z_ = static_cast<double>(ctx.cz());
    cmd_x_ = static_cast<double>(ctx.cx());
    cmd_y_ = static_cast<double>(ctx.cy());
    cmd_z_ = target_z_;

    yaw_hold_ = std::isfinite(ctx.sp_yaw)
      ? static_cast<double>(ctx.sp_yaw)
      : static_cast<double>(ctx.yaw);

    map_origin_x_ = cmd_x_ -
      static_cast<double>(cfg_.x_sign) * cfg_.start_cell.x * cfg_.cell_size;

    map_origin_y_ = cmd_y_ -
      static_cast<double>(cfg_.y_sign) * cfg_.start_cell.y * cfg_.cell_size;

    const AStarPlanner::Result result = AStarPlanner::plan(
      cfg_.cols,
      cfg_.rows,
      cfg_.cell_size,
      cfg_.start_cell,
      cfg_.goal_cell,
      cfg_.obstacle_cells);

    if (!result.success) {
      fail("A* planning failed: " + result.message);
      return;
    }

    buildExecutablePath(result);

    // 将当前地图信息同步到 Context，方便其他任务和日志读取。
    ctx.cols = cfg_.cols;
    ctx.rows = cfg_.rows;
    ctx.cell_size = cfg_.cell_size;
    ctx.origin_dx = map_origin_x_;
    ctx.origin_dy = map_origin_y_;
    ctx.current_c = cfg_.start_cell.x;
    ctx.current_r = cfg_.start_cell.y;

    if (path_.empty()) {
      phase_ = Phase::FINISHED;
      publishSetpoint(ctx);

      RCLCPP_WARN(
        logger_,
        "[A_STAR_GOTO] start already equals goal: cell=(%d,%d)",
        cfg_.start_cell.x,
        cfg_.start_cell.y);
      return;
    }

    RCLCPP_WARN(
      logger_,
      "[A_STAR_GOTO] enter: start=(%d,%d) goal=(%d,%d) grid=%dx%d cell=%.2f "
      "obstacles=%zu path=%zu origin=(%.2f %.2f) z=%.2f",
      cfg_.start_cell.x,
      cfg_.start_cell.y,
      cfg_.goal_cell.x,
      cfg_.goal_cell.y,
      cfg_.cols,
      cfg_.rows,
      cfg_.cell_size,
      cfg_.obstacle_cells.size(),
      path_.size(),
      map_origin_x_,
      map_origin_y_,
      target_z_);
  }

  Status tick(Context& ctx, double dt_s) override
  {
    if (phase_ == Phase::FAILED) {
      return Status::FAILURE;
    }

    if (phase_ == Phase::FINISHED) {
      publishSetpoint(ctx);
      return Status::SUCCESS;
    }

    if (index_ >= path_.size()) {
      phase_ = Phase::FINISHED;
      publishSetpoint(ctx);
      return Status::SUCCESS;
    }

    const ExecutablePoint& point = path_[index_];
    ctx.current_c = point.ix;
    ctx.current_r = point.iy;

    // 定位短暂失效时保持最后一个 setpoint，不推进路径。
    if (!ctx.pos_valid()) {
      publishSetpoint(ctx);
      return Status::RUNNING;
    }

    if (phase_ == Phase::MOVING) {
      moveCommandToward(point);
      publishSetpoint(ctx);

      if (arrived(ctx, point)) {
        cmd_x_ = point.x;
        cmd_y_ = point.y;
        cmd_z_ = point.z;
        publishSetpoint(ctx);

        if (point.hover_after && cfg_.hover_s > 1e-3) {
          hover_elapsed_s_ = 0.0;
          phase_ = Phase::HOVERING;

          RCLCPP_INFO(
            logger_,
            "[A_STAR_GOTO] arrived cell=(%d,%d) turn=%d hover=%.2fs point=%zu/%zu",
            point.ix,
            point.iy,
            point.is_turn ? 1 : 0,
            cfg_.hover_s,
            index_ + 1,
            path_.size());
        } else {
          nextPoint();
        }
      }

      return Status::RUNNING;
    }

    if (phase_ == Phase::HOVERING) {
      cmd_x_ = point.x;
      cmd_y_ = point.y;
      cmd_z_ = point.z;
      publishSetpoint(ctx);

      hover_elapsed_s_ += std::max(0.0, dt_s);
      if (hover_elapsed_s_ >= cfg_.hover_s) {
        nextPoint();
      }

      return Status::RUNNING;
    }

    return Status::RUNNING;
  }

  void onExit(Context& ctx) override
  {
    (void)ctx;
    RCLCPP_WARN(logger_, "[A_STAR_GOTO] exit");
  }

  void onPause(Context& ctx) override
  {
    if (ctx.pos_valid()) {
      cmd_x_ = static_cast<double>(ctx.cx());
      cmd_y_ = static_cast<double>(ctx.cy());
      cmd_z_ = static_cast<double>(ctx.cz());
      publishSetpoint(ctx);
    }

    RCLCPP_WARN(
      logger_,
      "[A_STAR_GOTO] paused at point=%zu/%zu",
      path_.empty() ? 0U : std::min(index_ + 1, path_.size()),
      path_.size());
  }

  void onResume(Context& ctx) override
  {
    if (phase_ == Phase::FAILED || phase_ == Phase::FINISHED || index_ >= path_.size()) {
      return;
    }

    if (ctx.pos_valid()) {
      cmd_x_ = static_cast<double>(ctx.cx());
      cmd_y_ = static_cast<double>(ctx.cy());
      cmd_z_ = static_cast<double>(ctx.cz());
    }

    phase_ = Phase::MOVING;
    hover_elapsed_s_ = 0.0;

    RCLCPP_WARN(
      logger_,
      "[A_STAR_GOTO] resumed at point=%zu/%zu",
      index_ + 1,
      path_.size());
  }

  const std::vector<ExecutablePoint>& path() const
  {
    return path_;
  }

private:
  enum class Phase
  {
    MOVING,
    HOVERING,
    FINISHED,
    FAILED
  };

  bool validateConfig()
  {
    if (cfg_.cols <= 0 || cfg_.rows <= 0) {
      fail("grid size must be positive");
      return false;
    }

    if (!std::isfinite(cfg_.cell_size) || cfg_.cell_size <= 0.0) {
      fail("cell_size must be finite and positive");
      return false;
    }

    if (!std::isfinite(cfg_.max_step_m) || cfg_.max_step_m < 0.0) {
      fail("max_step_m must be finite and non-negative");
      return false;
    }

    if (!std::isfinite(cfg_.arrive_xy_m) || cfg_.arrive_xy_m <= 0.0 ||
        !std::isfinite(cfg_.arrive_z_m) || cfg_.arrive_z_m <= 0.0) {
      fail("arrival tolerances must be finite and positive");
      return false;
    }

    if (!std::isfinite(cfg_.hover_s) || cfg_.hover_s < 0.0) {
      fail("hover_s must be finite and non-negative");
      return false;
    }

    if ((cfg_.x_sign != 1 && cfg_.x_sign != -1) ||
        (cfg_.y_sign != 1 && cfg_.y_sign != -1)) {
      fail("x_sign and y_sign must be 1 or -1");
      return false;
    }

    return true;
  }

  void buildExecutablePath(const AStarPlanner::Result& result)
  {
    path_.clear();

    if (result.path.size() <= 1) {
      return;
    }

    // result.path[0] 是进入任务时飞机已经所在的 start_cell，因此跳过。
    path_.reserve(result.path.size() - 1);

    for (std::size_t i = 1; i < result.path.size(); ++i) {
      const AStarPlanner::PathPoint& source = result.path[i];

      ExecutablePoint point;
      point.ix = source.cell.x;
      point.iy = source.cell.y;
      point.x = map_origin_x_ + static_cast<double>(cfg_.x_sign) * source.offset_x;
      point.y = map_origin_y_ + static_cast<double>(cfg_.y_sign) * source.offset_y;
      point.z = target_z_;
      point.is_turn = source.is_turn;
      point.hover_after = source.stop_after;

      path_.push_back(point);
    }
  }

  void moveCommandToward(const ExecutablePoint& point)
  {
    const double dx = point.x - cmd_x_;
    const double dy = point.y - cmd_y_;
    const double dz = point.z - cmd_z_;
    const double distance = std::sqrt(dx * dx + dy * dy + dz * dz);

    if (distance < 1e-6) {
      cmd_x_ = point.x;
      cmd_y_ = point.y;
      cmd_z_ = point.z;
      return;
    }

    if (cfg_.max_step_m < 1e-6 || distance <= cfg_.max_step_m) {
      cmd_x_ = point.x;
      cmd_y_ = point.y;
      cmd_z_ = point.z;
      return;
    }

    const double ratio = cfg_.max_step_m / distance;
    cmd_x_ += dx * ratio;
    cmd_y_ += dy * ratio;
    cmd_z_ += dz * ratio;
  }

  bool arrived(const Context& ctx, const ExecutablePoint& point) const
  {
    const double dx = static_cast<double>(ctx.cx()) - point.x;
    const double dy = static_cast<double>(ctx.cy()) - point.y;
    const double dz = static_cast<double>(ctx.cz()) - point.z;

    const double error_xy = std::sqrt(dx * dx + dy * dy);
    const double error_z = std::fabs(dz);

    return error_xy < cfg_.arrive_xy_m && error_z < cfg_.arrive_z_m;
  }

  void publishSetpoint(Context& ctx) const
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

  void nextPoint()
  {
    ++index_;
    hover_elapsed_s_ = 0.0;
    phase_ = Phase::MOVING;

    if (index_ >= path_.size()) {
      phase_ = Phase::FINISHED;
      RCLCPP_WARN(logger_, "[A_STAR_GOTO] reached goal");
    }
  }

  void fail(const std::string& reason)
  {
    phase_ = Phase::FAILED;
    RCLCPP_ERROR(logger_, "[A_STAR_GOTO] %s", reason.c_str());
  }

  rclcpp::Logger logger_;
  Config cfg_;

  std::vector<ExecutablePoint> path_;
  std::size_t index_{0};

  Phase phase_{Phase::MOVING};
  double hover_elapsed_s_{0.0};

  double map_origin_x_{0.0};
  double map_origin_y_{0.0};
  double target_z_{0.0};
  double yaw_hold_{0.0};

  double cmd_x_{0.0};
  double cmd_y_{0.0};
  double cmd_z_{0.0};
};

}  // namespace offboard_core_pkg
