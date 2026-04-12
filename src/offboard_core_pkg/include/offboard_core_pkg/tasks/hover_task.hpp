#pragma once

#include "rclcpp/rclcpp.hpp"
#include "offboard_core_pkg/itask.hpp"
#include "offboard_core_pkg/context.hpp"

class HoverTask : public ITask
{
public:
    HoverTask(rclcpp::Logger lg, double hover_time_s);

    std::string name() const override;

    void onEnter(Context& ctx) override;

    Status tick(Context& ctx, double dt) override;

private:
    rclcpp::Logger lg_;

    double hover_time_s_{0.0};   // 悬停时长（秒）
    double t_{0.0};              // 已经过的时间
};