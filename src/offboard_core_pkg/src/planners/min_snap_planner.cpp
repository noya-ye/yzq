#include "offboard_core_pkg/planners/min_snap_planner.hpp"

#include <mav_trajectory_generation/vertex.h>
#include <mav_trajectory_generation/polynomial_optimization_linear.h>
#include <mav_trajectory_generation/timing.h>

namespace offboard_core_pkg
{

MinSnapPlanner::MinSnapPlanner() {}

bool MinSnapPlanner::generate(
    const std::vector<Eigen::Vector3d>& waypoints)
{
  if (waypoints.size() < 2) return false;

  const int dimension = 3;
  const int derivative = mav_trajectory_generation::derivative_order::SNAP;

  std::vector<mav_trajectory_generation::Vertex> vertices;

  // =========================
  // 构造约束
  // =========================
  for (size_t i = 0; i < waypoints.size(); ++i)
  {
    mav_trajectory_generation::Vertex v(dimension);

    // 位置约束
    v.addConstraint(
        mav_trajectory_generation::derivative_order::POSITION,
        waypoints[i]);

    // 起点终点加约束
    if (i == 0 || i == waypoints.size() - 1)
    {
      v.addConstraint(
          mav_trajectory_generation::derivative_order::VELOCITY,
          Eigen::Vector3d::Zero());

      v.addConstraint(
          mav_trajectory_generation::derivative_order::ACCELERATION,
          Eigen::Vector3d::Zero());
    }

    vertices.push_back(v);
  }

  // =========================
  // 时间分配
  // =========================
  std::vector<double> segment_times =
      mav_trajectory_generation::estimateSegmentTimes(
          vertices, 1.0, 1.0);

  // =========================
  // 优化（Minimum Snap）
  // =========================
  const int N = 10;  // ⭐ 关键！必须 >= 2*derivative+2 = 10

  mav_trajectory_generation::PolynomialOptimization<N> opt(dimension);

  opt.setupFromVertices(vertices, segment_times, derivative);
  opt.solveLinear();

  opt.getTrajectory(&trajectory_);

  valid_ = true;
  return true;
}

bool MinSnapPlanner::sample(double t, Eigen::Vector3d& pos) const
{
  if (!valid_) return false;

  Eigen::VectorXd result =
      trajectory_.evaluate(
          t,
          mav_trajectory_generation::derivative_order::POSITION);

  pos = result.head<3>();

  return true;
}

double MinSnapPlanner::duration() const
{
  if (!valid_) return 0.0;
  return trajectory_.getMaxTime();
}

}