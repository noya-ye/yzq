#include "offboard_core_pkg/planners/tsp_min_snap_planner.hpp"

namespace offboard_core_pkg
{

TspMinSnapPlanner::TspMinSnapPlanner() {}

bool TspMinSnapPlanner::generate(
    const Eigen::Vector3d& start,
    const std::vector<Eigen::Vector3d>& targets)
{
  if (targets.empty()) return false;

  // =========================
  // Step1: TSP 排序
  // =========================
  std::vector<Eigen::Vector3d> tsp_path =
    tsp_.solve(targets);   // ✅只传targets
  // =========================
  // Step2: 拼接起点
  // =========================
  ordered_.clear();
  ordered_.push_back(start);

  for (auto& p : tsp_path)
    ordered_.push_back(p);

  // =========================
  // Step3: Minimum Snap
  // =========================
  if (!min_snap_.generate(ordered_))
    return false;

  return true;
}

bool TspMinSnapPlanner::sample(double t, Eigen::Vector3d& pos) const
{
  return min_snap_.sample(t, pos);
}

double TspMinSnapPlanner::duration() const
{
  return min_snap_.duration();
}

const std::vector<Eigen::Vector3d>&
TspMinSnapPlanner::orderedWaypoints() const
{
  return ordered_;
}

}