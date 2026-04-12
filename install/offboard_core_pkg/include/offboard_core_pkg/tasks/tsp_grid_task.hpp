#pragma once

#include "offboard_core_pkg/tasks/grid_goto_dwell_task.hpp"
#include "offboard_core_pkg/planners/tsp_planner.hpp"



// 这是一个GridGotoDwellTask的子类
// 实现功能：tsp规划出最优航点排列

// sched_.add(std::make_unique<TspGridTask>(
//     get_logger(),
//     grid_,
//     0.3,   // xy tol
//     0.2,   // z tol
//     0.3,   // vxy
//     0.2,   // vz
//     10,    // stable
//     2.0    // dwell
// ));

namespace offboard_core_pkg
{

class TspGridTask : public GridGotoDwellTask
{
public:
  using GridGotoDwellTask::GridGotoDwellTask;

protected:
  TspPlanner tsp_;

  void on_task_enter(Context& ctx) override
  {
    RCLCPP_INFO(lg_, "[TspGridTask] Start");

    // =========================
    // 1. 当前点
    // =========================
    Eigen::Vector3d start(
        ctx.cx(),
        ctx.cy(),
        ctx.cz());

    // =========================
    // 2. 目标点（来自视觉）
    // =========================
    std::vector<Eigen::Vector3d> targets;

    for (auto& p : ctx.detected_targets)   // ⚠️你需要有这个
    {
      targets.emplace_back(p.x, p.y, ctx.takeoff_z);
    }

    if (targets.empty()) {
      RCLCPP_WARN(lg_, "No targets, fallback to grid");
      return;  // 保留原grid
    }

    // =========================
    // 3. TSP 排序
    // =========================
    auto ordered = tsp_.solve(targets);

    // =========================
    // 4. 覆盖 Grid 路径
    // =========================
    grid_.waypoints.clear();

    // 👉 加入起点（可选）
    grid_.waypoints.emplace_back(start.x(), start.y());

    for (auto& p : ordered)
    {
      grid_.waypoints.emplace_back(p.x(), p.y());
    }

    grid_.wp_idx = 0;

    RCLCPP_INFO(lg_, "TSP path loaded: %zu waypoints", grid_.waypoints.size());
  }
};

}