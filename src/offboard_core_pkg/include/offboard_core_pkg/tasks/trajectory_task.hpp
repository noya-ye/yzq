#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "offboard_core_pkg/itask.hpp"
#include "offboard_core_pkg/context.hpp"

class TrajectoryTask : public ITask
{
public:
  enum class CoordMode
  {
    // waypoint 的 x/y/z 直接作为 PX4 local 坐标
    ABSOLUTE_LOCAL,

    // waypoint 的 x/y/z 相对进入任务瞬间的位置
    RELATIVE_TO_ENTER
  };

  struct InputWaypoint
  {
    double x{0.0};
    double y{0.0};
    double z{0.0};

    // 是否在该点停留
    bool hover_after{false};

    // 该点停留时间
    // < 0 时使用 Config::default_hover_s
    double hover_s{-1.0};

    // 是否使用该 waypoint 自带 yaw
    // false 时保持进入任务瞬间的 yaw
    bool use_yaw{false};
    double yaw{0.0};

    // 到达该点并完成停留后，
    // 是否请求 FRONT_PRE_ALIGN 中断任务
    bool pre_align_after{false};//中断接口
  };

  struct Config
  {
    std::vector<InputWaypoint> waypoints;

    CoordMode coord_mode{CoordMode::ABSOLUTE_LOCAL};

    // 每 tick setpoint 最大推进距离
    double max_step_m{0.03};

    // 到点判定误差
    double arrive_xy_m{0.08};
    double arrive_z_m{0.08};

    // 默认停留时间
    double default_hover_s{1.0};

    // 是否使用最近邻方式对航点进行 TSP 排序
    //
    // 否则粗定位点和扫描点可能被重新排序
    bool enable_tsp_sort{false};
  };

  explicit TrajectoryTask(
    rclcpp::Logger logger,
    const Config& cfg);

  std::string name() const override;

  void onEnter(Context& ctx) override;

  Status tick(
    Context& ctx,
    double dt_s) override;

  void onExit(Context& ctx) override;

  void onPause(Context& ctx) override;

  void onResume(Context& ctx) override;

  void onCancel(Context& ctx) override;

  // 仅建议在任务开始前调用
  void setWaypoints(
    const std::vector<InputWaypoint>& waypoints);

private:
  enum class Phase
  {
    MOVING,
    HOVERING,
    FINISHED,
    FAILED
  };

  struct Waypoint
  {
    // 已经解析完成的 PX4 local 绝对坐标
    double x{0.0};
    double y{0.0};
    double z{0.0};

    bool hover_after{false};
    double hover_s{0.0};

    bool use_yaw{false};
    double yaw{0.0};

    bool pre_align_after{false};

    // 该点在 Config::waypoints 中的原始编号
    size_t input_index{0};
  };

private:
  // 根据 Config 生成最终绝对坐标航点
  void buildWaypoints();

  // 把一个 InputWaypoint 转换成内部 Waypoint
  bool makeResolvedWaypoint(
    const InputWaypoint& in,
    size_t input_index,
    Waypoint& out) const;

  // 最近邻排序
  void sortWaypointsNearestNeighbor();

  // 让位置指令向当前航点平滑推进
  void moveCommandToward(
    const Waypoint& wp);

  // 使用飞机实际位置判断是否到点
  bool arrived(
    const Context& ctx,
    const Waypoint& wp) const;

  // 把 cmd_x_/cmd_y_/cmd_z_ 写入 Context
  void publishSetpoint(
    Context& ctx);

  // 当前航点完成后的处理：
  // 1. 需要前置矫正则发出中断请求
  // 2. 否则进入原轨迹下一个航点
  void completeCurrentWaypoint(
    Context& ctx);

  // 发起前置矫正请求
  void requestPreAlign(
    Context& ctx);

  // 进入原轨迹下一个航点
  void nextWaypoint();

  double waypointYaw(
    const Waypoint& wp) const;

  void holdCurrentPosition(
    Context& ctx);

private:
  rclcpp::Logger logger_;
  Config cfg_;

  std::vector<Waypoint> waypoints_;

  Phase phase_{Phase::FINISHED};

  // 当前正在执行的原轨迹航点编号
  size_t index_{0};

  double hover_elapsed_s_{0.0};

  // 进入 TrajectoryTask 时的实际位置
  double enter_x_{0.0};
  double enter_y_{0.0};
  double enter_z_{0.0};

  // 当前发送给 PX4 的平滑位置指令
  double cmd_x_{0.0};
  double cmd_y_{0.0};
  double cmd_z_{0.0};

  // 进入任务时的 yaw
  double yaw_hold_{0.0};

  // ============================================================
  // 前置矫正中断状态
  // ============================================================

  // 已到达粗定位点，正在等待 Scheduler 执行 interrupt
  bool waiting_pre_align_{false};

  // 当前暂停是否由 RACK_PRE_ALIGN 引起
  bool paused_for_pre_align_{false};

  // 触发前置矫正的“原轨迹粗定位点编号”
  //
  // 恢复时严格执行：
  // index_ = pre_align_waypoint_index_;
  // nextWaypoint();
  //
  // 因此目标一定是原粗定位点的下一个点
  size_t pre_align_waypoint_index_{0};

  // Scheduler 尚未完成切换时的固定悬停位置
  double interrupt_hold_x_{0.0};
  double interrupt_hold_y_{0.0};
  double interrupt_hold_z_{0.0};
};
