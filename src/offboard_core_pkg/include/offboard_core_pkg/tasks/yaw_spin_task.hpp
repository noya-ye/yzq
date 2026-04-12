#pragma once

#include "rclcpp/rclcpp.hpp"

#include "offboard_core_pkg/itask.hpp"
#include "offboard_core_pkg/context.hpp"
#include "offboard_core_pkg/math_tool.hpp"

namespace offboard_core_pkg
{

class YawSpinTask : public ITask
{
public:
  YawSpinTask(rclcpp::Logger logger,
              double yaw_rate ,   // rad/s（慢速）
              double duration )   // 秒
  : logger_(logger),
    yaw_rate_(yaw_rate),
    duration_(duration)
  {}

  std::string name() const override { return "YawSpinTask"; }

  void onEnter(Context& ctx) override;

  Status tick(Context& ctx, double dt) override;

private:
  rclcpp::Logger logger_;

  double yaw_rate_{0.0};
  double duration_{0.0};

  double elapsed_{0.0};

  double target_yaw_{0.0};

  float hold_x_, hold_y_, hold_z_;
};

} // namespace offboard_core_pkg