#pragma once

#include "offboard_core_pkg/itask.hpp"
#include "offboard_core_pkg/context.hpp"

#include <rclcpp/rclcpp.hpp>

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

/*
 * 蛇形网格巡查任务
 *
 * 坐标约定：
 *
 *   进入任务时无人机当前位置作为网格原点：
 *
 *     网格 (0, 0)
 *
 *   网格坐标转本地坐标：
 *
 *     x = origin_x + x_sign * ix * cell_size
 *     y = origin_y + y_sign * iy * cell_size
 *     z = 进入任务时的高度
 *
 *   地面站网格名称：
 *
 *     ix = 0, iy = 0  -> A1B1
 *     ix = 2, iy = 4  -> A3B5
 *
 * 任务功能：
 *
 *   1. 生成蛇形覆盖路线；
 *   2. 跳过禁飞网格；
 *   3. 使用 A* 绕过禁飞网格；
 *   4. 使用位置控制执行最终路线；
 *   5. 对外提供最终路线和当前执行进度。
 */
class SnakeGridTask final : public ITask
{
public:
  enum class FirstAxis
  {
    X_FIRST,
    Y_FIRST
  };

  enum class StopMode
  {
    // 每个网格点都悬停
    EVERY_CELL,

    // 只在每一行或每一列的末端悬停
    LINE_END_ONLY
  };

  /*
   * 禁飞网格。
   *
   * ix：x方向索引，从0开始
   * iy：y方向索引，从0开始
   *
   * 例如：
   *
   *   A3B5 -> ix=2, iy=4
   */
  struct ObstacleCell
  {
    int ix{0};
    int iy{0};
  };

  struct Config
  {
    FirstAxis first_axis{FirstAxis::X_FIRST};
    StopMode stop_mode{StopMode::EVERY_CELL};

    int x_cells{1};
    int y_cells{1};

    double cell_size{0.8};

    /*
     * 网格轴相对于PX4本地坐标系的方向。
     *
     * 通常只使用：
     *
     *   1  正方向
     *  -1  负方向
     */
    int x_sign{1};
    int y_sign{1};

    /*
     * 是否把进入任务时所在的 (0,0) 网格加入执行路线。
     *
     * true：
     *   路线从A1B1开始，并根据stop_mode悬停。
     *
     * false：
     *   A1B1只作为规划起点，不作为正常巡查航点。
     */
    bool include_start_cell{true};

    /*
     * 到达需要悬停的网格后的悬停时间。
     */
    double hover_s{0.8};

    /*
     * 每次tick最多推进的setpoint距离。
     *
     * 例如节点50Hz运行：
     *
     *   0.03 m/tick 约等于最大1.5m/s的setpoint推进速度。
     */
    double max_step_m{0.03};

    /*
     * 到达判定误差。
     */
    double arrive_xy_m{0.12};
    double arrive_z_m{0.15};

    /*
     * 地面站输入的禁飞网格。
     */
    std::vector<ObstacleCell> obstacle_cells;
  };

  SnakeGridTask(
    rclcpp::Logger logger,
    const Config& cfg);

  std::string name() const override;

  void onEnter(Context& ctx) override;

  ITask::Status tick(
    Context& ctx,
    double dt_s) override;

  void onExit(Context& ctx) override;

  void onPause(Context& ctx) override;

  void onResume(Context& ctx) override;

  /*
   * ---------------- 地面站读取接口 ----------------
   */

  /*
   * 当前规划编号。
   *
   * 每次成功生成一条新路线，plan_id都会变化。
   */
  std::uint32_t planId() const;

  /*
   * 路线是否已经成功生成。
   */
  bool planReady() const;

  /*
   * 最终实际执行路线。
   *
   * 这里包含：
   *
   *   1. 正常蛇形航点；
   *   2. A*绕障航点；
   *   3. 不包含禁飞网格。
   *
   * 数组顺序就是无人机的实际执行顺序。
   */
  const std::vector<std::string>& routeCells() const;

  /*
   * 当前正在执行的航点索引，0开始。
   *
   * 当任务完成时，返回totalWaypoints()。
   */
  std::size_t currentIndex() const;

  /*
   * 最终路线总航点数。
   */
  std::size_t totalWaypoints() const;

  /*
   * 当前目标网格名称。
   *
   * 任务完成后返回最终网格。
   * 路线为空时返回空字符串。
   */
  std::string currentCell() const;

  bool finished() const;

  bool failed() const;

private:
  struct Waypoint
  {
    int ix{0};
    int iy{0};

    double x{0.0};
    double y{0.0};
    double z{0.0};

    bool hover_after{false};
  };

  enum class Phase
  {
    MOVING,
    HOVERING,
    FINISHED,
    FAILED
  };

  void buildWaypoints();

  void buildXFirstWaypoints();

  void buildYFirstWaypoints();

  void pushWaypoint(
    int ix,
    int iy,
    bool line_end);

  Waypoint makeWaypoint(
    int ix,
    int iy,
    bool hover_after) const;

  void rebuildRouteCells();

  std::string cellToName(
    int ix,
    int iy) const;

  void moveCommandToward(
  const Waypoint& wp,
  double dt_s);
  bool arrived(
    const Context& ctx,
    const Waypoint& wp) const;

  void publishSetpoint(
    Context& ctx);

  void nextWaypoint();

  std::size_t displayWaypointIndex() const;

private:
  rclcpp::Logger logger_;
  Config cfg_;

  std::vector<Waypoint> waypoints_;

  /*
   * 与waypoints_严格一一对应。
   *
   * route_cells_[i]就是waypoints_[i]对应的网格名称。
   */
  std::vector<std::string> route_cells_;

  std::size_t index_{0};

  double origin_x_{0.0};
  double origin_y_{0.0};
  double target_z_{0.0};

  double cmd_x_{0.0};
  double cmd_y_{0.0};
  double cmd_z_{0.0};

  double yaw_hold_{0.0};

  double hover_elapsed_s_{0.0};

  Phase phase_{Phase::FINISHED};

  std::uint32_t plan_id_{0};
  bool plan_ready_{false};
};