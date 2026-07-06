#pragma once

#include <vector>
#include <string>
#include "ego_2d_planner_pkg/common/types.hpp"
#include "ego_2d_planner_pkg/plan_env/grid_map_2d.hpp"
#include "ego_2d_planner_pkg/path_searching/astar_2d.hpp"
#include "ego_2d_planner_pkg/bspline_opt/uniform_bspline_2d.hpp"
#include "ego_2d_planner_pkg/bspline_opt/bspline_optimizer_2d.hpp"

namespace ego_2d_planner_pkg
{

class PlannerManager2D
{
public:
  enum class PlanStatus
  {
    FAILED = 0,
    REBOUND_SUCCESS,
    ASTAR_FAILED,
    INVALID_INPUT,
    START_OR_GOAL_OCCUPIED,
    OPTIMIZATION_FAILED,
    COLLISION_AFTER_OPTIMIZATION
  };

  struct PlanResult
  {
    bool success{false};
    bool used_optimizer{false};
    bool smooth_safe{false};
    bool raw_path_safe{false};
    bool fallback_raw{false};  // kept only for diagnostics; normal EGO-style logic does not use it as success
    PlanStatus status{PlanStatus::FAILED};
    std::string message;

    Vec2 local_goal;
    std::vector<Vec2> raw_path;
    std::vector<Vec2> smooth_path;
    std::vector<Vec2> selected_path;

    // True executable 2D B-spline data. This is the 2D equivalent of EGO's
    // Bspline message: optimized control points + timing.
    std::vector<Vec2> selected_ctrl_pts;
    int bspline_order{3};
    double knot_span{0.25};
    double bspline_duration{0.0};

    double init_cost{0.0};
    double final_cost{0.0};
    int optimizer_iter{0};
    int rebound_attempt{0};
  };

  void setParams(const PlannerParams2D& params);

  // EGO-style initial trajectory generation. It still calls reboundReplan(),
  // but the semantic meaning matches EGO's GEN_NEW_TRAJ stage.
  PlanResult planGlobalTraj(const GridMap2D& map, const Vec2& start, const Vec2& goal);

  // EGO-style local replan/rebound. The optimized B-spline must pass
  // post-optimization collision checking. Raw A* is never treated as success.
  PlanResult reboundReplan(const GridMap2D& map, const Vec2& start, const Vec2& goal, bool init);

  // Backward-compatible alias.
  PlanResult plan(const GridMap2D& map, const Vec2& start, const Vec2& goal)
  {
    return reboundReplan(map, start, goal, true);
  }

private:
  bool checkInput(const GridMap2D& map, const Vec2& start, const Vec2& goal, PlanResult& out) const;

  PlannerParams2D p_;
  AStar2D astar_;
};

}  // namespace ego_2d_planner_pkg
