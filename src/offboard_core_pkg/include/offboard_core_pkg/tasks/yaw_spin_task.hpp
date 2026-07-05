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
              double yaw_rate)   // rad/s（慢速）
  : logger_(logger),
    yaw_rate_(yaw_rate)
  {}

  std::string name() const override { return "YawSpinTask"; }

  void onEnter(Context& ctx) override;

  Status tick(Context& ctx, double dt) override;

private:
  // =========================
  // 内部状态机
  // =========================
  enum class SpinState
  {
    SPINNING,
    STABILIZING
  };

  SpinState state_{SpinState::SPINNING};

  // =========================
  // 参数
  // =========================
  rclcpp::Logger logger_;
  double yaw_rate_{0.0};

  // =========================
  // 运行时变量
  // =========================
  double elapsed_{0.0};        // 可保留（调试用）
  double target_yaw_{0.0};

  // =========================
  // 位置锁定
  // =========================
  float hold_x_{0.0};
  float hold_y_{0.0};
  float hold_z_{0.0};
};

} // namespace offboard_core_pkg



