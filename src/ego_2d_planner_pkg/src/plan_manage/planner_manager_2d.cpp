#include "ego_2d_planner_pkg/plan_manage/planner_manager_2d.hpp"

#include <cmath>
#include <sstream>

namespace ego_2d_planner_pkg
{

void PlannerManager2D::setParams(const PlannerParams2D& params)
{
  p_ = params;
}

bool PlannerManager2D::checkInput(const GridMap2D& map, const Vec2& start, const Vec2& goal, PlanResult& out) const
{
  out.local_goal = start;

  if (!finite(start) || !finite(goal)) {
    out.status = PlanStatus::INVALID_INPUT;
    out.message = "invalid start/goal";
    return false;
  }

  int sx = 0, sy = 0, gx = 0, gy = 0;
  if (!map.worldToGrid(start, sx, sy)) {
    out.status = PlanStatus::INVALID_INPUT;
    out.message = "start out of map";
    return false;
  }
  if (!map.worldToGrid(goal, gx, gy)) {
    out.status = PlanStatus::INVALID_INPUT;
    out.message = "goal out of map";
    return false;
  }
  if (map.isOccupied(sx, sy)) {
    out.status = PlanStatus::START_OR_GOAL_OCCUPIED;
    out.message = "start occupied";
    return false;
  }
  if (map.isOccupied(gx, gy)) {
    out.status = PlanStatus::START_OR_GOAL_OCCUPIED;
    out.message = "goal occupied";
    return false;
  }

  return true;
}

PlannerManager2D::PlanResult PlannerManager2D::planGlobalTraj(const GridMap2D& map,
                                                              const Vec2& start,
                                                              const Vec2& goal)
{
  return reboundReplan(map, start, goal, true);
}

PlannerManager2D::PlanResult PlannerManager2D::reboundReplan(const GridMap2D& map,
                                                             const Vec2& start,
                                                             const Vec2& goal,
                                                             bool init)
{
  PlanResult out;
  if (!checkInput(map, start, goal, out)) {
    return out;
  }

  if (!astar_.search(map, start, goal, out.raw_path, p_.astar_max_iter)) {
    out.status = PlanStatus::ASTAR_FAILED;
    out.message = init ? "global AStar2D failed" : "local AStar2D failed";
    return out;
  }

  if (out.raw_path.size() < 2) {
    out.status = PlanStatus::ASTAR_FAILED;
    out.message = "raw path too short";
    return out;
  }

  out.raw_path_safe = map.isPathSafe(out.raw_path, p_.collision_check_step);
  if (!out.raw_path_safe) {
    // This normally should not happen because A* searches on the same occupied grid,
    // but keep it as a hard failure for consistency with EGO's safety-first behavior.
    out.status = PlanStatus::ASTAR_FAILED;
    out.message = "raw A* path failed dense collision check";
    return out;
  }

  if (!p_.optimizer_enable) {
    out.status = PlanStatus::OPTIMIZATION_FAILED;
    out.message = "optimizer disabled; EGO-style planner refuses raw fallback as normal success";
    return out;
  }

  const std::vector<Vec2> base_ctrl =
    UniformBspline2D::makeClampedControlPoints(out.raw_path, p_.control_points_distance);

  if (base_ctrl.size() < 7) {
    out.status = PlanStatus::OPTIMIZATION_FAILED;
    out.message = "insufficient B-spline control points";
    return out;
  }

  // EGO-style rebound/retry loop:
  // A* gives a guide path; optimizer tries several increasingly collision-aware variants.
  // Only a post-checked optimized B-spline can be selected.
  const int attempts = std::max(1, p_.max_rebound_attempts);
  for (int k = 0; k < attempts; ++k) {
    std::vector<Vec2> ctrl = base_ctrl;
    const std::vector<Vec2> ref_ctrl = base_ctrl;

    PlannerParams2D trial = p_;
    trial.lambda_collision *= std::pow(p_.retry_collision_scale, k);
    trial.lambda_smooth    *= std::pow(p_.retry_smooth_scale, k);
    trial.dist0            *= std::pow(p_.retry_dist0_scale, k);

    BsplineOptimizer2D optimizer;
    optimizer.setParams(trial);
    const auto opt = optimizer.optimize(ctrl, ref_ctrl, map);

    UniformBspline2D spline(ctrl, trial.knot_span);
    std::vector<Vec2> path = spline.sample(trial.bspline_sample_step);
    if (!path.empty()) {
      path.front() = start;
      path.back() = goal;
    }

    const bool path_safe = map.isPathSafe(path, trial.collision_check_step);

    if (opt.success && path_safe) {
      out.success = true;
      out.used_optimizer = true;
      out.smooth_safe = true;
      out.status = PlanStatus::REBOUND_SUCCESS;
      out.smooth_path = path;
      out.selected_path = path;
      out.selected_ctrl_pts = ctrl;
      out.bspline_order = 3;
      out.knot_span = trial.knot_span;
      out.bspline_duration = spline.duration();
      out.local_goal = UniformBspline2D::lookaheadPoint(path, start, trial.lookahead_dist);
      out.init_cost = opt.init_cost;
      out.final_cost = opt.final_cost;
      out.optimizer_iter = opt.iterations;
      out.rebound_attempt = k + 1;

      std::ostringstream ss;
      ss << (init ? "GEN_NEW_TRAJ" : "REPLAN_TRAJ")
         << " rebound success, attempt=" << (k + 1)
         << ", lambda_collision=" << trial.lambda_collision
         << ", dist0=" << trial.dist0;
      out.message = ss.str();
      return out;
    }

    // Keep the last failed smooth path only for visualization/debug.
    out.smooth_path = path;
    out.init_cost = opt.init_cost;
    out.final_cost = opt.final_cost;
    out.optimizer_iter = opt.iterations;
    out.rebound_attempt = k + 1;
  }

  out.status = PlanStatus::COLLISION_AFTER_OPTIMIZATION;
  out.message = "all rebound optimization attempts failed collision check; no raw fallback selected";
  out.success = false;
  return out;
}

}  // namespace ego_2d_planner_pkg
