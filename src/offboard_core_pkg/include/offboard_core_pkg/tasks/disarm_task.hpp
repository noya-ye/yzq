#pragma once

#include <string>

#include "rclcpp/rclcpp.hpp"

#include "offboard_core_pkg/itask.hpp"
#include "offboard_core_pkg/context.hpp"
#include "offboard_core_pkg/px4_iface.hpp"

class DisarmTask : public ITask {
public:
  DisarmTask(rclcpp::Logger lg, Px4Iface& px4);

  std::string name() const override;

  void onEnter(Context& ctx) override;

  Status tick(Context& ctx, double dt) override;

private:
  rclcpp::Logger lg_;
  Px4Iface& px4_;
  bool sent_{false};
};