#pragma once

#include "rclcpp/rclcpp.hpp"
#include "offboard_core_pkg/itask.hpp"
#include "offboard_core_pkg/context.hpp"

class WaitHomeTask : public ITask {
public:
  WaitHomeTask(rclcpp::Logger lg);

  std::string name() const override;

  Status tick(Context& ctx, double dt) override;

private:
  rclcpp::Logger lg_;
};