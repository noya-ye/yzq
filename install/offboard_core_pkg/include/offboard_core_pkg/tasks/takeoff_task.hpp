#pragma once

#include "rclcpp/rclcpp.hpp"
#include "offboard_core_pkg/itask.hpp"
#include "offboard_core_pkg/context.hpp"
#include "offboard_core_pkg/px4_iface.hpp"
#include "offboard_core_pkg/math_tool.hpp"

class TakeoffTask : public ITask
{
public:
    TakeoffTask(rclcpp::Logger lg, Px4Iface& px4, double arrival_error_max);

    std::string name() const override;
    void onEnter(Context& ctx) override;
    Status tick(Context& ctx, double dt) override;

private:
    Px4Iface& px4_;
    rclcpp::Logger lg_;
    double arrival_error_max_{0.0};
};