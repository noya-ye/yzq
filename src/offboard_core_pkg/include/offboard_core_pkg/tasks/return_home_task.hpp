#pragma once

#include "offboard_core_pkg/itask.hpp"
#include "offboard_core_pkg/context.hpp"
#include <offboard_core_pkg/math_tool.hpp>

#include <rclcpp/rclcpp.hpp>
#include <string>

class ReturnHomeTask : public ITask
{
public:
  ReturnHomeTask(
    rclcpp::Logger lg,
    double return_step_xy,
    double return_xy_tol,
    double return_vxy_tol);

  std::string name() const override;

  void onEnter(Context& ctx) override;

  Status tick(Context& ctx, double dt) override;

private:
  rclcpp::Logger lg_;

  double return_step_xy_{0.2};
  double return_xy_tol_{0.20};
  double return_vxy_tol_{0.25};

  double print_elapsed_{0.0};
};