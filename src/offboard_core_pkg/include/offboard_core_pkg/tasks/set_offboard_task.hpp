#pragma once

#include "rclcpp/rclcpp.hpp"
#include "offboard_core_pkg/itask.hpp"
#include "offboard_core_pkg/context.hpp"
#include "offboard_core_pkg/px4_iface.hpp"

class SetOffboardTask :public ITask
{
public:
    SetOffboardTask(rclcpp::Logger lg,Px4Iface& px4);

    std::string name() const override;

    void onEnter(Context&) override;

    Status tick (Context& ctx, double dt) override;

private:
    Px4Iface& px4_;
    bool sent_{false};
    double t_{0.0};
    double resend_t_{0.0};
    rclcpp::Logger lg_;
};


/*private:的参数里面写什么：1.外部依赖：lg_,px4_
2.跨tick的信息
3.当前任务的状态量
4.当前任务独有参数
不写什么：context 不写！！！！*/