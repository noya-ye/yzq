#pragma once

#include <Eigen/Dense>
#include <vector>
#include <mav_trajectory_generation/trajectory.h>

class TrajectoryGenerator {
public:
  bool generate(
    const std::vector<Eigen::Vector3d>& waypoints);

  Eigen::Vector3d samplePosition(double t);
  Eigen::Vector3d sampleVelocity(double t);

  double getTotalTime() const;

private:
  mav_trajectory_generation::Trajectory traj_;
};