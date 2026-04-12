#pragma once

#include <string>

#include "rclcpp/rclcpp.hpp"

#include "offboard_core_pkg/itask.hpp"
#include "offboard_core_pkg/context.hpp"
#include "offboard_core_pkg/grid.hpp"

class LandDescendTask : public ITask {
public:
  explicit LandDescendTask(rclcpp::Logger lg);

  std::string name() const override;

  Status tick(Context& ctx, double dt) override;

private:
  rclcpp::Logger lg_;
};