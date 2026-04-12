#pragma once

#include "rclcpp/rclcpp.hpp"
#include "offboard_core_pkg/itask.hpp"
#include "offboard_core_pkg/context.hpp"
#include "offboard_core_pkg/px4_iface.hpp"

namespace offboard_core_pkg
{

class Px4LandModeTask : public ITask
{
public:
  Px4LandModeTask(rclcpp::Logger lg, Px4Iface& px4);

  std::string name() const override;

  void onEnter(Context& ctx) override;

  Status tick(Context& ctx, double dt) override;

private:
  rclcpp::Logger lg_;
  Px4Iface& px4_;

  bool land_cmd_sent_{false};
  bool force_disarm_sent_{false};

  double wait_time_s_{0.0};
  double stable_time_s_{0.0};
};

} // namespace offboard_core_pkg