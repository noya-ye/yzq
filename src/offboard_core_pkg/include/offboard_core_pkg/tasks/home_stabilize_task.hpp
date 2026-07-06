#pragma once

#include <string>

#include "rclcpp/rclcpp.hpp"

#include "offboard_core_pkg/itask.hpp"
#include "offboard_core_pkg/context.hpp"

class HomeStabilizeTask : public ITask {
public:
  HomeStabilizeTask(rclcpp::Logger lg, double home_stabilize_s);

  std::string name() const override;

  void onEnter(Context& ctx) override;

  Status tick(Context& ctx, double dt) override;

private:
  rclcpp::Logger lg_;
  double home_stabilize_s_{3.0};
  double t_{0.0};
};