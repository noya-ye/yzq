/*
 * ========================= SnakeGridTask 使用说明 =========================
 *
 * 这个任务用于在当前位置附近生成一个蛇形覆盖搜索轨迹。
 *
 * 坐标约定：
 *   - 进入任务瞬间的当前位置作为蛇形搜索起点 origin。
 *   - origin 对应网格坐标 (ix=0, iy=0)。
 *   - 飞行高度保持为进入任务瞬间的 ctx.local_pos.z。
 *
 * -------------------------------------------------------------------------
 * 1. first_axis：控制蛇形主扫描方向
 *
 *   X_FIRST：
 *     先沿 x 方向扫，一行扫完后沿 y 方向换行。
 *
 *     示例，x_cells=3, y_cells=3：
 *
 *       y=2:  → → →
 *       y=1:  ← ← ←
 *       y=0:  → → →
 *
 *     实际遍历顺序：
 *       (0,0) -> (1,0) -> (2,0)
 *              -> (2,1) -> (1,1) -> (0,1)
 *              -> (0,2) -> (1,2) -> (2,2)
 *
 *
 *   Y_FIRST：
 *     先沿 y 方向扫，一列扫完后沿 x 方向换列。
 *
 *     示例，x_cells=3, y_cells=3：
 *
 *       x=0:  ↑
 *             ↑
 *             ↑
 *
 *       x=1:  ↓
 *             ↓
 *             ↓
 *
 *       x=2:  ↑
 *             ↑
 *             ↑
 *
 *     实际遍历顺序：
 *       (0,0) -> (0,1) -> (0,2)
 *              -> (1,2) -> (1,1) -> (1,0)
 *              -> (2,0) -> (2,1) -> (2,2)
 *
 * -------------------------------------------------------------------------
 * 2. cell_size：控制每个格子的实际间距
 *
 *   cell_size = 0.8 表示相邻两个网格点间隔 0.8 米。
 *
 *   例如：
 *     ix=0 -> x = origin_x
 *     ix=1 -> x = origin_x + 0.8
 *     ix=2 -> x = origin_x + 1.6
 *
 * -------------------------------------------------------------------------
 * 3. x_cells / y_cells：控制搜索区域大小
 *
 *   x_cells 表示 x 方向有多少个格子点。
 *   y_cells 表示 y 方向有多少个格子点。
 *
 *   例如：
 *     x_cells = 5
 *     y_cells = 5
 *
 *   总搜索点数量为：
 *     5 * 5 = 25 个点
 *
 * -------------------------------------------------------------------------
 * 4. stop_mode：控制在哪里悬停
 *
 *   EVERY_CELL：
 *     每到一个格子点都悬停。
 *     适合需要每个点进行视觉识别、拍照、检测的任务。
 *
 *   LINE_END_ONLY：
 *     只在每一行或每一列的尽头悬停。
 *     适合快速覆盖搜索，不需要每格都停。
 *
 * -------------------------------------------------------------------------
 * 5. x_sign / y_sign：控制网格铺设到真实坐标系的方向
 *
 *   注意：
 *     x_sign / y_sign 控制的是“真实坐标方向”，不是蛇形遍历顺序。
 *
 *   x_sign =  1：
 *     ix 增大时，目标点 x 坐标增大。
 *
 *   x_sign = -1：
 *     ix 增大时，目标点 x 坐标减小。
 *
 *   y_sign =  1：
 *     iy 增大时，目标点 y 坐标增大。
 *
 *   y_sign = -1：
 *     iy 增大时，目标点 y 坐标减小。
 *
 *   计算方式：
 *     wp.x = origin_x + x_sign * ix * cell_size;
 *     wp.y = origin_y + y_sign * iy * cell_size;
 *
 *   举例：
 *
 *     x_sign = 1, y_sign = 1：
 *       网格向 PX4 local x 正方向、y 正方向展开。
 *
 *     x_sign = -1, y_sign = 1：
 *       网格向 PX4 local x 负方向、y 正方向展开。
 *
 *     x_sign = 1, y_sign = -1：
 *       网格向 PX4 local x 正方向、y 负方向展开。
 *
 *     x_sign = -1, y_sign = -1：
 *       网格向 PX4 local x 负方向、y 负方向展开。
 *
 * -------------------------------------------------------------------------
 * 6. max_step_m：控制 setpoint 平滑移动速度
 *
 *   每个控制周期最多让 setpoint 前进 max_step_m 米。
 *
 *   如果 timer 是 50ms：
 *
 *     max_step_m = 0.03
 *
 *   约等于：
 *
 *     0.03 / 0.05 = 0.6 m/s
 *
 *   这个值越大，飞机跟随目标点会越快；
 *   这个值越小，飞机运动越柔和。
 *
 * -------------------------------------------------------------------------
 * 7. arrive_xy_m / arrive_z_m：控制到点判断
 *
 *   arrive_xy_m：
 *     xy 平面误差小于该值，认为 xy 到达。
 *
 *   arrive_z_m：
 *     z 方向误差小于该值，认为高度到达。
 *
 *   两个条件都满足后，任务才会进入悬停或下一个点。
 *
 * -------------------------------------------------------------------------
 * 8. include_start_cell：是否把当前位置也作为第一个搜索点
 *
 *   true：
 *     第一个 waypoint 就是当前位置。
 *     任务开始后会先在当前位置悬停一次。
 *
 *   false：
 *     跳过当前位置，从下一个格子开始飞。
 *先沿 x 方向，每格悬停
SnakeGridTask::Config snake_cfg;
snake_cfg.first_axis = SnakeGridTask::FirstAxis::X_FIRST;
snake_cfg.cell_size = 0.8;
snake_cfg.x_cells = 5;
snake_cfg.y_cells = 5;
snake_cfg.stop_mode = SnakeGridTask::StopMode::EVERY_CELL;
snake_cfg.hover_s = 0.8;
snake_cfg.x_sign = 1;
snake_cfg.y_sign = 1;
先沿 y 方向，只在列尽头悬停
SnakeGridTask::Config snake_cfg;
snake_cfg.first_axis = SnakeGridTask::FirstAxis::Y_FIRST;
snake_cfg.cell_size = 0.8;
snake_cfg.x_cells = 5;
snake_cfg.y_cells = 5;
snake_cfg.stop_mode = SnakeGridTask::StopMode::LINE_END_ONLY;
snake_cfg.hover_s = 1.0;
snake_cfg.x_sign = 1;
snake_cfg.y_sign = 1;
 * =========================================================================
 */
#pragma once

#include <vector>
#include <string>
#include <cmath>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"

#include "offboard_core_pkg/itask.hpp"
#include "offboard_core_pkg/context.hpp"

class SnakeGridTask : public ITask
{
public:
  enum class FirstAxis
  {
    X_FIRST,   // 先沿 x 方向扫
    Y_FIRST    // 先沿 y 方向扫
  };

  enum class StopMode
  {
    EVERY_CELL,     // 每到一个格子都悬停
    LINE_END_ONLY   // 只在每一行/列尽头悬停
  };

  struct Config
  {
    FirstAxis first_axis{FirstAxis::X_FIRST};

    double cell_size{0.8};      // 单位格大小，单位 m
    int x_cells{5};             // x 方向格子数
    int y_cells{5};             // y 方向格子数

    StopMode stop_mode{StopMode::EVERY_CELL};

    double hover_s{0.8};        // 悬停时间
    double arrive_xy_m{0.12};   // xy 到达误差
    double arrive_z_m{0.15};    // z 到达误差

    // 每个控制周期 setpoint 最大移动距离。
    // 你的 timer 是 50ms，如果 max_step_m=0.03，相当于 setpoint 约 0.6m/s。
    double max_step_m{0.03};

    // 默认从当前位置开始，把当前位置也当成第一个格子。
    bool include_start_cell{true};

    // 方向符号。
    // 如果发现实际蛇形方向反了，可以把对应方向改成 -1。
    int x_sign{1};
    int y_sign{1};
  };

private:
  struct Waypoint
  {
    double x{0.0};
    double y{0.0};
    double z{0.0};

    int ix{0};
    int iy{0};

    bool hover_after{true};
  };

  enum class Phase
  {
    MOVING,
    HOVERING,
    FINISHED,
    FAILED
  };

public:
  SnakeGridTask(rclcpp::Logger logger, const Config& cfg)
  : logger_(logger), cfg_(cfg)
  {
  }

  std::string name() const override
  {
    return "SNAKE_GRID";
  }

  void onEnter(Context& ctx) override
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

    cmd_x_ = ctx.cx();
    cmd_y_ = ctx.cy();
    cmd_z_ = ctx.cz();

    yaw_hold_ = ctx.sp_yaw;

    // 同步到 Context 里的网格参数，方便别的任务读取
    ctx.cols = cfg_.x_cells;
    ctx.rows = cfg_.y_cells;
    ctx.cell_size = cfg_.cell_size;

    buildWaypoints();

    if (waypoints_.empty()) {
      RCLCPP_WARN(logger_, "[SNAKE] no waypoint generated, finish directly");
      phase_ = Phase::FINISHED;
      return;
    }

    RCLCPP_WARN(
      logger_,
      "[SNAKE] enter: first_axis=%s stop_mode=%s x_cells=%d y_cells=%d cell=%.2f total_wp=%zu origin=(%.2f %.2f %.2f)",
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

  Status tick(Context& ctx, double dt_s) override
  {
    if (phase_ == Phase::FAILED) {
      return Status::FAILURE;
    }

    if (phase_ == Phase::FINISHED) {
      return Status::SUCCESS;
    }

    if (index_ >= waypoints_.size()) {
      phase_ = Phase::FINISHED;
      return Status::SUCCESS;
    }

    const Waypoint& wp = waypoints_[index_];

    // 如果飞行中短暂丢失位置，不切换目标，只继续发布上一次 setpoint
    if (!ctx.pos_valid()) {
      publishSetpoint(ctx);
      return Status::RUNNING;
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
            "[SNAKE] arrived cell=(%d,%d), hover %.2fs, wp=%zu/%zu",
            wp.iy,
            wp.ix,
            cfg_.hover_s,
            index_ + 1,
            waypoints_.size());
        } else {
          nextWaypoint();
        }
      }

      return Status::RUNNING;
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

      return Status::RUNNING;
    }

    return Status::RUNNING;
  }

  void onExit(Context& ctx) override
  {
    (void)ctx;
    RCLCPP_WARN(logger_, "[SNAKE] exit");
  }

private:
  void buildWaypoints()
  {
    waypoints_.clear();
    waypoints_.reserve(static_cast<size_t>(cfg_.x_cells * cfg_.y_cells));

    if (cfg_.first_axis == FirstAxis::X_FIRST) {
      buildXFirstWaypoints();
    } else {
      buildYFirstWaypoints();
    }
  }

  void buildXFirstWaypoints()
  {
    // 轨迹示例：
    //
    // y=2:  ← ← ←
    // y=1:  → → →
    // y=0:  → → →
    //
    // 实际顺序：
    // (0,0)->(1,0)->...->(xmax,0)
    //       ->(xmax,1)->...->(0,1)
    //       ->(0,2)->...

    for (int iy = 0; iy < cfg_.y_cells; ++iy) {
      const bool reverse = (iy % 2 == 1);

      for (int k = 0; k < cfg_.x_cells; ++k) {
        const int ix = reverse ? (cfg_.x_cells - 1 - k) : k;
        const bool line_end = (k == cfg_.x_cells - 1);

        pushWaypoint(ix, iy, line_end);
      }
    }
  }

  void buildYFirstWaypoints()
  {
    // 轨迹示例：
    //
    // x=0:  ↑
    //       ↑
    //       ↑
    //
    // x=1:  ↓
    //       ↓
    //       ↓
    //
    // 实际顺序：
    // (0,0)->(0,1)->...->(0,ymax)
    //       ->(1,ymax)->...->(1,0)
    //       ->(2,0)->...

    for (int ix = 0; ix < cfg_.x_cells; ++ix) {
      const bool reverse = (ix % 2 == 1);

      for (int k = 0; k < cfg_.y_cells; ++k) {
        const int iy = reverse ? (cfg_.y_cells - 1 - k) : k;
        const bool line_end = (k == cfg_.y_cells - 1);

        pushWaypoint(ix, iy, line_end);
      }
    }
  }

  void pushWaypoint(int ix, int iy, bool line_end)
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

  void moveCommandToward(const Waypoint& wp)
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

    const double r = step / dist;

    cmd_x_ += dx * r;
    cmd_y_ += dy * r;
    cmd_z_ += dz * r;
  }

  bool arrived(const Context& ctx, const Waypoint& wp) const
  {
    const double dx = static_cast<double>(ctx.cx()) - wp.x;
    const double dy = static_cast<double>(ctx.cy()) - wp.y;
    const double dz = static_cast<double>(ctx.cz()) - wp.z;

    const double err_xy = std::sqrt(dx * dx + dy * dy);
    const double err_z = std::fabs(dz);

    return err_xy < cfg_.arrive_xy_m && err_z < cfg_.arrive_z_m;
  }

  void publishSetpoint(Context& ctx)
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

  void nextWaypoint()
  {
    ++index_;
    hover_elapsed_s_ = 0.0;
    phase_ = Phase::MOVING;

    if (index_ >= waypoints_.size()) {
      phase_ = Phase::FINISHED;

      RCLCPP_WARN(logger_, "[SNAKE] finished all waypoints");
    }
  }

private:
  rclcpp::Logger logger_;
  Config cfg_;

  std::vector<Waypoint> waypoints_;

  size_t index_{0};
  Phase phase_{Phase::MOVING};

  double origin_x_{0.0};
  double origin_y_{0.0};
  double target_z_{0.0};
  double yaw_hold_{0.0};

  double cmd_x_{0.0};
  double cmd_y_{0.0};
  double cmd_z_{0.0};

  double hover_elapsed_s_{0.0};
};