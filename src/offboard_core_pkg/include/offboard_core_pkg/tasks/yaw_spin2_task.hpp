#pragma once

#include <cmath>

#include "offboard_core_pkg/itask.hpp"
#include "offboard_core_pkg/context.hpp"
#include "offboard_core_pkg/math_tool.hpp"

#include "rclcpp/rclcpp.hpp"

namespace offboard_core_pkg
{

class YawSpin2Task : public ITask
{
public:
  std::string name() const override
  {
    return "YawSpin2Task";
  }

  explicit YawSpin2Task(
    rclcpp::Logger logger,
    double yaw_rate = 0.22,
    double settle_time_s = 1.0,
    double sample_timeout_s = 2.0)
  : logger_(logger),
    yaw_rate_(std::fabs(yaw_rate)),
    settle_time_s_(settle_time_s),
    sample_timeout_s_(sample_timeout_s)
  {
  }

  void onEnter(Context& ctx) override;
  Status tick(Context& ctx, double dt) override;

private:
  enum class State
  {
    ROTATING,
    SETTLING,
    SAMPLING
  };

private:
  void stop_vision(Context& ctx);
  void enable_front_sampling(Context& ctx);
  bool drift_check(Context& ctx);

private:
  rclcpp::Logger logger_;

  State state_{State::ROTATING};

  double elapsed_{0.0};
  double state_elapsed_{0.0};

  double yaw_rate_{0.35};
  double settle_time_s_{0.6};
  double sample_timeout_s_{2.0};

  static constexpr int kTotalSegments = 8;
  static constexpr float kSegmentAngle =
    static_cast<float>(M_PI) / 4.0f;

  int current_segment_{0};

  float start_yaw_{0.0f};
  float target_yaw_{0.0f};
  float segment_target_yaw_{0.0f};

  float hold_x_{0.0f};
  float hold_y_{0.0f};
  float hold_z_{0.0f};

  double last_print_s_{-10.0};
};

}  // namespace offboard_core_pkg