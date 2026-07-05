#pragma once

#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"

#include "offboard_core_pkg/itask.hpp"
#include "offboard_core_pkg/context.hpp"

class GoToTask : public ITask
{
public:
  GoToTask(
    rclcpp::Logger lg,
    double xy_tol = 0.20,
    double z_tol = 0.15,
    int stable_required = 10);

  std::string name() const override;

  void onEnter(Context& ctx) override;

  ITask::Status tick(Context& ctx, double dt) override;

private:
  rclcpp::Logger lg_;

  double xy_tol_{0.20};
  double z_tol_{0.15};
  int stable_required_{10};

  int stable_count_{0};

  bool has_target_{false};

  float target_x_{0.0f};
  float target_y_{0.0f};
  float target_z_{0.0f};
};
