#include "offboard_core_pkg/tasks/takeoff_task.hpp"

TakeoffTask::TakeoffTask(rclcpp::Logger lg, Px4Iface& px4, double arrival_error_max)
: lg_(lg), px4_(px4), arrival_error_max_(arrival_error_max)
{}

std::string TakeoffTask::name() const
{
    return "TAKEOFF";
}

void TakeoffTask::onEnter(Context& ctx)
{
    RCLCPP_INFO(lg_, "TAKEOFF enter: target_z = %.3f", ctx.takeoff_z);
}

ITask::Status TakeoffTask::tick(Context& ctx, double)
{
    ctx.use_vel_ctrl = false;

    ctx.sp_x = ctx.home_x;
    ctx.sp_y = ctx.home_y;
    ctx.sp_z = ctx.takeoff_z;
    ctx.sp_yaw = ctx.home_yaw;

    if (math_tool::dist3d(ctx.cx(), ctx.cy(), ctx.cz(),
               ctx.sp_x, ctx.sp_y, ctx.sp_z) <= arrival_error_max_) {
        RCLCPP_INFO(lg_, "TAKEOFF done");
        return Status::SUCCESS;
    }

    return Status::RUNNING;
}