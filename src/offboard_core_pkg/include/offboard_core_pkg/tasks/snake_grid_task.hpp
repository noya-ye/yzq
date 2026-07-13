#pragma once

#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "offboard_core_pkg/itask.hpp"
#include "offboard_core_pkg/context.hpp"

class SnakeGridTask : public ITask
{
public:
  enum class FirstAxis
  {
    X_FIRST,
    Y_FIRST
  };

  enum class StopMode
  {
    EVERY_CELL,
    LINE_END_ONLY
  };
struct ObstacleCell
{
  int ix{0};
  int iy{0};
};
  struct Config
  {
    FirstAxis first_axis{FirstAxis::X_FIRST};

    double cell_size{0.8};
    int x_cells{5};
    int y_cells{5};

    StopMode stop_mode{StopMode::EVERY_CELL};

    double hover_s{0.8};
    double arrive_xy_m{0.12};
    double arrive_z_m{0.15};

    // 每个控制周期 setpoint 最大推进距离
    // timer=50ms 时，0.03m 约等于 0.6m/s
    double max_step_m{0.03};

    // 是否把当前位置也作为第一个搜索点
    bool include_start_cell{true};

    // 控制网格在 PX4 local 坐标系中的展开方向
    int x_sign{1};
    int y_sign{1};

    std::vector<ObstacleCell> obstacle_cells;
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
  SnakeGridTask(rclcpp::Logger logger, const Config& cfg);

  std::string name() const override;

  void onEnter(Context& ctx) override;

  ITask::Status tick(Context& ctx, double dt_s) override;

  void onExit(Context& ctx) override;

  // 需要你的 ITask 已经支持 onPause / onResume

  void onPause(Context& ctx) override;
  void onResume(Context& ctx) override;

private:
  void buildWaypoints();

  void buildXFirstWaypoints();

  void buildYFirstWaypoints();

  void pushWaypoint(int ix, int iy, bool line_end);

  void moveCommandToward(const Waypoint& wp);

  bool arrived(const Context& ctx, const Waypoint& wp) const;

  void publishSetpoint(Context& ctx);

  void nextWaypoint();

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