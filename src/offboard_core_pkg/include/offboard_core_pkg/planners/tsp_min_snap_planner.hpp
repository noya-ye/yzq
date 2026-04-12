#pragma once

#include <vector>
#include <Eigen/Dense>

#include "offboard_core_pkg/planners/tsp_planner.hpp"
#include "offboard_core_pkg/planners/min_snap_planner.hpp"

namespace offboard_core_pkg
{

class TspMinSnapPlanner
{
public:
  TspMinSnapPlanner();

  // =========================
  // 输入：当前点 + 目标点
  // =========================
  bool generate(
      const Eigen::Vector3d& start,
      const std::vector<Eigen::Vector3d>& targets);

  // =========================
  // 采样轨迹
  // =========================
  bool sample(double t, Eigen::Vector3d& pos) const;

  double duration() const;

  const std::vector<Eigen::Vector3d>& orderedWaypoints() const;

private:
  TspPlanner tsp_;
  MinSnapPlanner min_snap_;

  std::vector<Eigen::Vector3d> ordered_;
};

}