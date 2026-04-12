#pragma once

#include <string>

#include "rclcpp/rclcpp.hpp"

#include "offboard_core_pkg/itask.hpp"
#include "offboard_core_pkg/context.hpp"
#include "offboard_core_pkg/math_tool.hpp"

class ReturnHomeTask : public ITask {
public:
  ReturnHomeTask(rclcpp::Logger lg,
                 double return_step_xy,
                 double return_xy_tol,
                 double return_vxy_tol);

  std::string name() const override;

  Status tick(Context& ctx, double dt) override;

private:
  rclcpp::Logger lg_;
  double return_step_xy_{0.3};
  double return_xy_tol_{0.2};
  double return_vxy_tol_{0.15};
};