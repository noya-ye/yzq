#pragma once

#include "rclcpp/rclcpp.hpp"
#include "offboard_core_pkg/itask.hpp"
#include "offboard_core_pkg/context.hpp"

class PresetpointTask : public ITask {
public:
  PresetpointTask(rclcpp::Logger lg);

  std::string name() const override;

  void onEnter(Context& ctx) override;

  Status tick(Context& ctx, double dt) override;

private:
  rclcpp::Logger lg_;
  int counter_{0};
};