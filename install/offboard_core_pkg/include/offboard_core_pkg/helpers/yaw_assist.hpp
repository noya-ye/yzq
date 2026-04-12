#pragma once

#include <algorithm>
#include <cstdint>

#include "rclcpp/rclcpp.hpp"

#include "offboard_core_pkg/context.hpp"
#include "offboard_core_pkg/math_tool.hpp"

class YawAssist {
public:
  YawAssist() = default;

  YawAssist(uint64_t yaw_fresh_us,
            float yaw_quality_min,
            float yaw_kp,
            float yaw_max_rad)
  : yaw_fresh_us_(yaw_fresh_us),
    yaw_quality_min_(yaw_quality_min),
    yaw_kp_(yaw_kp),
    yaw_max_rad_(yaw_max_rad),
    clock_(RCL_STEADY_TIME)
  {}

  void set_params(uint64_t yaw_fresh_us,
                  float yaw_quality_min,
                  float yaw_kp,
                  float yaw_max_rad)
  {
    yaw_fresh_us_ = yaw_fresh_us;
    yaw_quality_min_ = yaw_quality_min;
    yaw_kp_ = yaw_kp;
    yaw_max_rad_ = yaw_max_rad;
  }

  uint64_t now_us() const
  {
    return static_cast<uint64_t>(clock_.now().nanoseconds() / 1000ULL);
  }

  bool yaw_data_fresh(const Context& ctx) const
  {
    const uint64_t t_us = now_us();

    return (ctx.yaw.stamp_us != 0ULL) &&
           (t_us >= ctx.yaw.stamp_us) &&
           ((t_us - ctx.yaw.stamp_us) <= yaw_fresh_us_);
  }

  bool yaw_data_usable(const Context& ctx) const
  {
    return yaw_data_fresh(ctx) && (ctx.yaw.quality >= yaw_quality_min_);
  }

  float compute_delta_yaw(const Context& ctx) const
  {
    if (!yaw_data_usable(ctx)) {
      return 0.0f;
    }

    const float raw = yaw_kp_ * ctx.yaw.error_rad;
    return std::clamp(raw, -yaw_max_rad_, yaw_max_rad_);
  }

  void apply_to_sp_yaw(Context& ctx) const
  {
    if (!yaw_data_usable(ctx)) {
      return;
    }

    const float delta = compute_delta_yaw(ctx);
    ctx.sp_yaw = math_tool::wrap_pi(ctx.sp_yaw + delta);
  }

  void apply_to_value(Context& ctx, float& yaw_value) const
  {
    if (!yaw_data_usable(ctx)) {
      return;
    }

    const float delta = compute_delta_yaw(ctx);
    yaw_value = math_tool::wrap_pi(yaw_value + delta);
  }

private:
  uint64_t yaw_fresh_us_{250000};   // 视觉 yaw 数据允许的最大陈旧时间
  float yaw_quality_min_{0.25f};    // 最低可接受质量
  float yaw_kp_{1.0f};              // 比例修正系数
  float yaw_max_rad_{0.35f};        // 单次修正最大弧度

  mutable rclcpp::Clock clock_{RCL_STEADY_TIME};
};