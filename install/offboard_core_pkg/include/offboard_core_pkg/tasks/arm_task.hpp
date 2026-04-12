#pragma once

#include "rclcpp/rclcpp.hpp"

#include "offboard_core_pkg/itask.hpp"
#include "offboard_core_pkg/context.hpp"
#include "offboard_core_pkg/px4_iface.hpp"

class ArmTask : public ITask {
public:
  ArmTask(rclcpp::Logger lg, Px4Iface& px4);

  std::string name() const override;

  void onEnter(Context& ctx) override;

  Status tick(Context& ctx, double dt) override;

private:
  rclcpp::Logger lg_;
  Px4Iface& px4_;

  bool sent_{false};
  double t_{0};
  double resend_t_{0.0};
};