#pragma once

#include <vector>
#include <Eigen/Dense>

#include <mav_trajectory_generation/trajectory.h>

namespace offboard_core_pkg
{

class MinSnapPlanner
{
public:
  MinSnapPlanner();

  // =========================
  // 生成轨迹
  // =========================
  bool generate(const std::vector<Eigen::Vector3d>& waypoints);

  // =========================
  // 采样轨迹
  // =========================
  bool sample(double t, Eigen::Vector3d& pos) const;

  // =========================
  // 获取总时长
  // =========================
  double duration() const;

private:
  mav_trajectory_generation::Trajectory trajectory_;
  bool valid_{false};
};

}