#include "trajectory_generator_pkg/generator.hpp"

#include <mav_trajectory_generation/vertex.h>
#include <mav_trajectory_generation/polynomial_optimization_linear.h>
#include <mav_trajectory_generation/timing.h>

bool TrajectoryGenerator::generate(
    const std::vector<Eigen::Vector3d>& waypoints)
{
    const int dimension = 3;
    const int derivative = mav_trajectory_generation::derivative_order::SNAP;

    std::vector<mav_trajectory_generation::Vertex> vertices;

    for (size_t i = 0; i < waypoints.size(); ++i) {
        mav_trajectory_generation::Vertex v(dimension);
        v.addConstraint(mav_trajectory_generation::derivative_order::POSITION, waypoints[i]);

        if (i == 0 || i == waypoints.size() - 1) {
            v.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY,
                            Eigen::Vector3d::Zero());
            v.addConstraint(mav_trajectory_generation::derivative_order::ACCELERATION,
                            Eigen::Vector3d::Zero());
        }

        vertices.push_back(v);
    }

    // 时间分配（可调）
    std::vector<double> segment_times =
        mav_trajectory_generation::estimateSegmentTimes(vertices, 1.0, 1.0);

    mav_trajectory_generation::PolynomialOptimization<8> opt(dimension);
    opt.setupFromVertices(vertices, segment_times, derivative);

    opt.solveLinear();

    opt.getTrajectory(&traj_);

    return true;
}

Eigen::Vector3d TrajectoryGenerator::samplePosition(double t) {
    return traj_.evaluate(t, 0);
}

Eigen::Vector3d TrajectoryGenerator::sampleVelocity(double t) {
    return traj_.evaluate(t, 1);
}

double TrajectoryGenerator::getTotalTime() const {
    return traj_.getMaxTime();
}